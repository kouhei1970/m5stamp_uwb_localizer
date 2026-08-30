/**
 * @file uwb_net_udp.cpp
 * @brief UDP ブロードキャスト送信 (forward) / 受信して集約 (aggregate)
 *        (uwb_net_internal.hpp の分担: uwb_net_udp.cpp)。
 *        UDP broadcast send (forward) and receive-and-aggregate (aggregate).
 *
 * scratchpad/NET_SPEC.md §2:
 *   - forward=true（アンカー）: 間引き後の行を 255.255.255.255:<udpPort> へ、
 *     1400バイト以内・行単位（改行をまたがない）でブロードキャストする。
 *   - aggregate=true（タグ）: 0.0.0.0:<udpPort> で受信し、受け取った各
 *     データグラムの中身を（末尾に'\n'が無ければ補って）sinkPushRaw() で
 *     自分のリングへ混ぜる。
 */
#include "uwb_net_internal.hpp"

#include "sdkconfig.h"

#if CONFIG_UWB_NET_ENABLE

#include <atomic>
#include <cerrno>
#include <cstring>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"

namespace uwb::net::internal {

namespace {

constexpr const char* kTag = "uwb_net_udp";

/** 1データグラムの最大ペイロード（scratchpad/NET_SPEC.md §2）。 */
constexpr size_t kMaxDatagram = 1400;

int g_sock = -1;
struct sockaddr_in g_broadcastAddr;
TaskHandle_t g_rxTaskHandle = nullptr;

std::atomic<uint32_t> g_txCount{0};
std::atomic<uint32_t> g_rxCount{0};

/** aggregate=true のときだけ動く受信専用タスク。届いたデータグラムをそのまま
 *  （末尾'\n'を保証した上で）sinkPushRaw() へ渡すだけ。 */
void rxTask(void* /*arg*/)
{
    // JSON_BUF_SIZE(main.cpp)は2048だが、UDPは1400バイト以内で送る取り決め
    // (kMaxDatagram)なので、受信側もそれで足りる。+1は末尾'\n'補完の余地。
    static char buf[kMaxDatagram + 1];

    while (true) {
        struct sockaddr_in from;
        socklen_t fromLen = sizeof(from);
        const int len =
            recvfrom(g_sock, buf, sizeof(buf) - 1, 0, reinterpret_cast<struct sockaddr*>(&from), &fromLen);
        if (len < 0) {
            // ソケットに一時的な問題（起こらない想定）。ビジーループにならない
            // よう少し待ってから再試行する。
            ESP_LOGW(kTag, "recvfrom() failed: errno=%d", errno);
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        if (len == 0) {
            continue;
        }
        g_rxCount.fetch_add(1, std::memory_order_relaxed);

        size_t pushLen = static_cast<size_t>(len);
        if (buf[pushLen - 1] != '\n') {
            // 送信側は必ず改行込みで送る取り決めだが、途中で切れた等の保険として
            // 末尾に改行を足す（バッファに1バイトの余地を常に残してある）。
            buf[pushLen] = '\n';
            ++pushLen;
        }
        sinkPushRaw(buf, pushLen);
    }
}

} // namespace

/* ==================================================================== *
 * uwb_net_internal.hpp で宣言された関数
 * ==================================================================== */

esp_err_t udpStart()
{
    const Config& cfg = config();
    if (!cfg.forward && !cfg.aggregate) {
        return ESP_OK; // どちらも使わない機体（今のところ想定していないが安全に無視する）
    }

    g_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (g_sock < 0) {
        ESP_LOGE(kTag, "socket() failed: errno=%d", errno);
        return ESP_FAIL;
    }

    if (cfg.forward) {
        const int enable = 1;
        if (setsockopt(g_sock, SOL_SOCKET, SO_BROADCAST, &enable, sizeof(enable)) < 0) {
            ESP_LOGW(kTag, "SO_BROADCAST の設定に失敗しました: errno=%d", errno);
        }
        std::memset(&g_broadcastAddr, 0, sizeof(g_broadcastAddr));
        g_broadcastAddr.sin_family      = AF_INET;
        g_broadcastAddr.sin_port        = htons(cfg.udpPort);
        g_broadcastAddr.sin_addr.s_addr = htonl(INADDR_BROADCAST); // 255.255.255.255 (limited broadcast)
    }

    if (cfg.aggregate) {
        struct sockaddr_in bindAddr;
        std::memset(&bindAddr, 0, sizeof(bindAddr));
        bindAddr.sin_family      = AF_INET;
        bindAddr.sin_addr.s_addr = htonl(INADDR_ANY);
        bindAddr.sin_port        = htons(cfg.udpPort);
        if (bind(g_sock, reinterpret_cast<struct sockaddr*>(&bindAddr), sizeof(bindAddr)) < 0) {
            ESP_LOGE(kTag, "bind(0.0.0.0:%u) failed: errno=%d", static_cast<unsigned>(cfg.udpPort), errno);
            close(g_sock);
            g_sock = -1;
            return ESP_FAIL;
        }

        const BaseType_t ok =
            xTaskCreatePinnedToCore(&rxTask, "uwb_net_udprx", 4096, nullptr, 5, &g_rxTaskHandle, 0);
        if (ok != pdPASS) {
            ESP_LOGE(kTag, "uwb_net_udprx タスクの起動に失敗しました");
            close(g_sock);
            g_sock = -1;
            return ESP_FAIL;
        }
    }

    ESP_LOGI(kTag, "udp: port=%u forward=%d aggregate=%d", static_cast<unsigned>(cfg.udpPort),
             static_cast<int>(cfg.forward), static_cast<int>(cfg.aggregate));
    return ESP_OK;
}

void udpSend(const char* data, size_t len)
{
    const Config& cfg = config();
    if (!cfg.forward || g_sock < 0 || data == nullptr || len == 0) {
        return;
    }

    // data は既に「最後は必ず'\n'」に揃えられた塊（uwb_net_sink.cpp
    // drainForSend()）だが、1400バイトを超えることがあるので、ここで
    // さらに「1400バイト以内・行をまたがない」データグラムへ分割する。
    size_t start = 0;
    while (start < len) {
        const size_t remaining = len - start;
        const size_t limit      = (remaining < kMaxDatagram) ? remaining : kMaxDatagram;

        size_t cut = 0;
        for (size_t i = limit; i > 0; --i) {
            if (data[start + i - 1] == '\n') {
                cut = i;
                break;
            }
        }

        if (cut == 0) {
            // limit バイト以内に改行が無い = この1行だけで1400バイトを超えている。
            // 「1400バイト以内・行単位」を守れないので、この行はUDP転送では
            // 丸ごと捨てて次の行へ進む（USBシリアル/ブラウザ側の同じ行は無事）。
            size_t j = start;
            while (j < len && data[j] != '\n') {
                ++j;
            }
            start = (j < len) ? (j + 1) : len;
            continue;
        }

        const int sent = sendto(g_sock, data + start, cut, 0,
                                 reinterpret_cast<struct sockaddr*>(&g_broadcastAddr), sizeof(g_broadcastAddr));
        if (sent >= 0) {
            g_txCount.fetch_add(1, std::memory_order_relaxed);
        }
        start += cut;
    }
}

uint32_t udpTxCount()
{
    return g_txCount.load(std::memory_order_relaxed);
}

uint32_t udpRxCount()
{
    return g_rxCount.load(std::memory_order_relaxed);
}

TaskHandle_t udpRxTaskHandle()
{
    return g_rxTaskHandle;
}

} // namespace uwb::net::internal

#else // !CONFIG_UWB_NET_ENABLE

namespace uwb::net::internal {

esp_err_t udpStart()
{
    return ESP_OK;
}

void udpSend(const char*, size_t) {}

uint32_t udpTxCount()
{
    return 0;
}

uint32_t udpRxCount()
{
    return 0;
}

TaskHandle_t udpRxTaskHandle()
{
    return nullptr;
}

} // namespace uwb::net::internal

#endif // CONFIG_UWB_NET_ENABLE
