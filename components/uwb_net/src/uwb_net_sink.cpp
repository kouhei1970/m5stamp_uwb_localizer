/**
 * @file uwb_net_sink.cpp
 * @brief 行のリングバッファ・配信タスク (uwb_net_tx)・"node" 行の生成
 *        (uwb_net_internal.hpp の分担: uwb_net_sink.cpp)。
 *        Line ring buffer, the uwb_net_tx delivery task, and "node" line generation.
 *
 * scratchpad/NET_SPEC.md §4:
 *   - sinkPushRaw() は間引き済みの1行（または複数行まとめて）を、絶対にブロック
 *     せずリングバッファへ積む。入らなければ丸ごと捨てて drops を増やす
 *     （部分書き込みは絶対にしない）。
 *   - uwb_net_tx タスク（コア0・優先度6）が 50ms ごとに最大3072バイトを
 *     「最後の改行で必ず切って」取り出し、httpBroadcast()/udpSend()へ渡す。
 *   - 1000ms ごとに "node" 行を組み立て、stdoutへも出しつつ同じリングへ積む
 *     （stdoutへ"node"行を書くのはここだけ）。
 */
#include "uwb_net_internal.hpp"
#include "uwb_port.h"

#include "sdkconfig.h"

#if CONFIG_UWB_NET_ENABLE

#include <cstdarg>
#include <cstdio>
#include <cstring>

#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

namespace uwb::net::internal {

namespace {

constexpr const char* kTag = "uwb_net_sink";

/** リング本体。.bss に静的確保（heap 断片化を避ける。CONFIG_UWB_NET_RING_BYTES
 *  既定12KBはPSRAM無しのESP32-S3内蔵SRAMでも余裕がある）。 */
char g_ring[CONFIG_UWB_NET_RING_BYTES];
constexpr size_t kRingCap = sizeof(g_ring);

size_t g_head = 0; // 次に書き込む位置
size_t g_tail = 0; // 次に読み出す位置
size_t g_used = 0; // 使用中バイト数（0〜kRingCap）
uint32_t g_drops = 0;

SemaphoreHandle_t g_mutex = nullptr;
TaskHandle_t g_txTaskHandle = nullptr;

/** 配信タスクが1周期ぶん切り出す先。送信中もタスクは他のことをしないので
 *  タスク専有の静的バッファでよい。 */
constexpr size_t kTxChunkCap = 3072;
char g_txBuf[kTxChunkCap];

/** "node" 行の組み立て先。role固有メンバ (StatusJsonFn) を含めても十分な余裕
 *  を見て 1024 バイト（uwb_net_http.cpp handleApiInfo() の buildNodeLine 呼び
 *  出しと同じ容量に揃えてある）。 */
constexpr size_t kNodeLineCap = 1024;

bool lock()
{
    return (g_mutex != nullptr) && (xSemaphoreTake(g_mutex, portMAX_DELAY) == pdTRUE);
}

void unlock()
{
    if (g_mutex != nullptr) {
        xSemaphoreGive(g_mutex);
    }
}

/* ---------------------------------------------------- JSON組み立てヘルパー
 * firmware/tag/main/main.cpp の jsonAppend() と同じ「溢れたら黙って打ち切る」
 * 流儀（このファイル用に複製。共有ヘッダは無い/作らない方針は本リポジトリの
 * 既存コメントに準じる: 数行のヘルパーを毎回共有するより単純さを優先）。 */
void appendf(char* buf, size_t cap, size_t* off, const char* fmt, ...) __attribute__((format(printf, 4, 5)));

void appendf(char* buf, size_t cap, size_t* off, const char* fmt, ...)
{
    if (*off >= cap) {
        return;
    }
    va_list args;
    va_start(args, fmt);
    const int n = vsnprintf(buf + *off, cap - *off, fmt, args);
    va_end(args);
    if (n > 0) {
        const size_t written   = static_cast<size_t>(n);
        const size_t remaining = cap - *off;
        *off += (written < remaining) ? written : (remaining - 1);
    }
}

/** JSONエスケープしながら1文字ずつ追記する。実体は uwb_net_cmd.cpp の
 *  jsonEscapeAppendByte()（'"' '\\' '\n' '\r' '\t' と 0x20未満の制御文字を
 *  \u00XX にする）を再利用する共有ヘルパー呼び出し。
 *  レビュー指摘(2026-08-31): 以前は '"' '\\' だけしかエスケープしておらず、
 *  ssid等に制御文字が混ざると壊れたJSONになりえた。 */
void appendEscaped(char* buf, size_t cap, size_t* off, const char* s)
{
    if (s == nullptr) {
        return;
    }
    for (const unsigned char* p = reinterpret_cast<const unsigned char*>(s); *p != '\0'; ++p) {
        jsonEscapeAppendByte(buf, *off, cap, *p);
    }
}

/* --------------------------------------------------------------- リング --- */

/** リングから最大 chunkCap バイトを「最後の改行までで」取り出し g_txBuf へ
 *  コピーする。取り出せた（コミットした）バイト数を返す（0なら送るものが
 *  無い、または改行が見つからず次回に持ち越し）。 */
size_t drainForSend()
{
    if (!lock()) {
        return 0;
    }
    if (g_used == 0) {
        unlock();
        return 0;
    }
    const size_t n = (g_used < kTxChunkCap) ? g_used : kTxChunkCap;

    // まずコピーするだけ（コミットはまだしない）。
    const size_t firstPart = (n < (kRingCap - g_tail)) ? n : (kRingCap - g_tail);
    std::memcpy(g_txBuf, g_ring + g_tail, firstPart);
    if (firstPart < n) {
        std::memcpy(g_txBuf + firstPart, g_ring, n - firstPart);
    }

    // n バイトの中で最後の '\n' を探す。無ければ「行の途中」なので今回は
    // 何もコミットせず次の周期に回す（次回は同じデータ + 追加分をまた見る）。
    size_t cut  = 0;
    bool found = false;
    for (size_t i = n; i > 0; --i) {
        if (g_txBuf[i - 1] == '\n') {
            cut   = i;
            found = true;
            break;
        }
    }
    if (!found) {
        unlock();
        return 0;
    }

    g_tail = (g_tail + cut) % kRingCap;
    g_used -= cut;
    unlock();
    return cut;
}

/* ------------------------------------------------------------- 配信タスク --- */

/** 起動から約10秒後に1回だけ、uwb_net が持つ各タスクのスタック残量を
 *  firmware/tag/main/main.cpp の既存ログ（"... task stack high-water mark:
 *  N bytes free"）と同じ書式で出す（コーディネータ指示 2026-08-31:
 *  実機でスタック余裕を確認したい）。作っていないタスク（handleがnullptr。
 *  aggregate=false のときの udprx 等）はスキップする。 */
void logStackHighWaterMarks()
{
    struct Entry {
        const char* name;
        TaskHandle_t handle;
    };
    const Entry entries[] = {
        {"uwb_net_tx", g_txTaskHandle},
        {"uwb_net_udprx", udpRxTaskHandle()},
        {"uwb_net_tcp", tcpConsoleTaskHandle()},
        {"uwb_net_wifi", wifiRescanTaskHandle()},
        {"uwb_net_wscmd", wsCmdTaskHandle()},
    };
    for (const Entry& e : entries) {
        if (e.handle == nullptr) {
            continue;
        }
        ESP_LOGI(kTag, "%s task stack high-water mark: %u bytes free", e.name,
                 static_cast<unsigned>(uxTaskGetStackHighWaterMark(e.handle)));
    }
}

void txTask(void* /*arg*/)
{
    TickType_t lastWake       = xTaskGetTickCount();
    int64_t lastNodeUs          = 0; // 0 にしておき、起動直後に必ず1回出す
    const int64_t taskStartUs = esp_timer_get_time();
    bool stackLogged             = false;

    while (true) {
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(50));

        const size_t n = drainForSend();
        if (n > 0) {
            httpBroadcast(g_txBuf, n);
            if (config().forward) {
                udpSend(g_txBuf, n);
            }
        }

        const int64_t nowUs = esp_timer_get_time();
        if ((nowUs - lastNodeUs) >= 1000 * 1000) {
            lastNodeUs = nowUs;
            char nodeBuf[kNodeLineCap];
            const size_t nodeLen = buildNodeLine(nodeBuf, sizeof(nodeBuf));
            if (nodeLen > 0) {
                // "node" 行を stdout へ出すのはここだけ（scratchpad/NET_SPEC.md §4）。
                if (uwb_port_usb_host_connected()) {  // ホスト不在時は USB へ書かない / skip USB writes with no host
                    std::fputs(nodeBuf, stdout);
                }
                sinkPushRaw(nodeBuf, nodeLen);
            }
        }

        if (!stackLogged && (nowUs - taskStartUs) >= 10LL * 1000 * 1000) {
            stackLogged = true;
            logStackHighWaterMarks();
        }
    }
}

} // namespace

/* ==================================================================== *
 * uwb_net_internal.hpp で宣言された関数
 * ==================================================================== */

esp_err_t sinkStart()
{
    if (g_mutex == nullptr) {
        g_mutex = xSemaphoreCreateMutex();
        if (g_mutex == nullptr) {
            return ESP_ERR_NO_MEM;
        }
    }

    // スタック6144: txTaskは1024バイトのnodeBufに加えbuildNodeLine内部の
    // 256バイトextraバッファ・%.3fのvsnprintf呼び出しをスタック上に積む
    // （レビュー指摘 2026-08-31: 4096では余裕が薄いと判断）。
    const BaseType_t ok =
        xTaskCreatePinnedToCore(&txTask, "uwb_net_tx", 6144, nullptr, 6, &g_txTaskHandle, 0);
    if (ok != pdPASS) {
        ESP_LOGE(kTag, "uwb_net_tx タスクの起動に失敗しました");
        return ESP_FAIL;
    }
    return ESP_OK;
}

void sinkPushRaw(const char* data, size_t len)
{
    if (data == nullptr || len == 0) {
        return;
    }
    if (len > kRingCap) {
        // JSON_BUF_SIZE(2048)/node行(1024) << kRingCap(既定12288) なので実際には
        // 起こらないが、万一に備えて丸ごと捨てる（部分書き込みは絶対にしない）。
        if (lock()) {
            ++g_drops;
            unlock();
        }
        return;
    }
    if (!lock()) {
        return;
    }
    if (len > (kRingCap - g_used)) {
        ++g_drops;
        unlock();
        return;
    }

    const size_t firstPart = (len < (kRingCap - g_head)) ? len : (kRingCap - g_head);
    std::memcpy(g_ring + g_head, data, firstPart);
    if (firstPart < len) {
        std::memcpy(g_ring, data + firstPart, len - firstPart);
    }
    g_head = (g_head + len) % kRingCap;
    g_used += len;
    unlock();
}

uint32_t sinkDrops()
{
    uint32_t v = 0;
    if (lock()) {
        v = g_drops;
        unlock();
    }
    return v;
}

size_t buildNodeLine(char* buf, size_t cap)
{
    if (buf == nullptr || cap == 0) {
        return 0;
    }
    size_t off              = 0;
    const Config& cfg        = config();
    const double tSec        = static_cast<double>(esp_timer_get_time()) / 1e6;

    WifiStatus wifi;
    wifiStatus(wifi);

    appendf(buf, cap, &off, "{\"v\":1,\"type\":\"node\",\"t\":%.3f,\"role\":\"%s\",\"name\":\"", tSec,
            (cfg.role == Role::Tag) ? "tag" : "anchor");
    appendEscaped(buf, cap, &off, cfg.name);
    appendf(buf, cap, &off, "\",\"addr\":\"");
    appendEscaped(buf, cap, &off, cfg.addr);
    appendf(buf, cap, &off, "\",\"wifi\":{\"mode\":\"%s\",\"ssid\":\"", wifi.mode);
    appendEscaped(buf, cap, &off, wifi.ssid);
    appendf(buf, cap, &off, "\",\"ip\":\"%s\",\"rssi\":%d,\"clients\":%u},", wifi.ip, wifi.rssi,
            static_cast<unsigned>(wifi.apClients));
    appendf(buf, cap, &off,
            "\"uptime_s\":%lu,\"heap_free\":%lu,\"heap_min\":%lu,\"drops\":%lu,"
            "\"ws_clients\":%d,\"tcp_console\":%s,\"udp_tx\":%lu,\"udp_rx\":%lu",
            static_cast<unsigned long>(tSec), static_cast<unsigned long>(esp_get_free_heap_size()),
            static_cast<unsigned long>(esp_get_minimum_free_heap_size()),
            static_cast<unsigned long>(sinkDrops()), httpWsClientCount(),
            tcpConsoleClientConnected() ? "true" : "false", static_cast<unsigned long>(udpTxCount()),
            static_cast<unsigned long>(udpRxCount()));

    // role固有メンバ (Config::statusFn)。書式は uwb_net.hpp のコメントどおり
    // 先頭カンマ・波括弧なし。0バイト（未設定/一時的に出す物が無い）なら
    // カンマごと省略し、末尾カンマの壊れたJSONにしない。
    if (cfg.statusFn != nullptr) {
        char extra[256] = "";
        const size_t extraLen = cfg.statusFn(extra, sizeof(extra), cfg.statusUser);
        if (extraLen > 0 && extra[0] != '\0') {
            appendf(buf, cap, &off, ",%s", extra);
        }
    }

    appendf(buf, cap, &off, "}\n");
    return off;
}

} // namespace uwb::net::internal

#else // !CONFIG_UWB_NET_ENABLE

namespace uwb::net::internal {

esp_err_t sinkStart()
{
    return ESP_OK;
}

void sinkPushRaw(const char*, size_t) {}

uint32_t sinkDrops()
{
    return 0;
}

size_t buildNodeLine(char* buf, size_t cap)
{
    if (buf != nullptr && cap > 0) {
        buf[0] = '\0';
    }
    return 0;
}

} // namespace uwb::net::internal

#endif // CONFIG_UWB_NET_ENABLE
