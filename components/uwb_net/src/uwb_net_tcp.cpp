/**
 * @file uwb_net_tcp.cpp
 * @brief TCP テキストコンソール (port 23) (uwb_net_internal.hpp の分担: uwb_net_tcp.cpp)。
 *        Plain-text TCP console (one client at a time).
 *
 * scratchpad/NET_SPEC.md §2:
 *   平文TCP、1クライアントのみ。バナー "uwb-net console (<name>). Type
 *   'exit' to close.\r\n"、プロンプト "<name>> "。1行最大160文字、CR/LFは
 *   取り除く。telnet の IAC シーケンス（0xFF + 2バイト、および
 *   0xFF 0xFA ... 0xFF 0xF0 のサブネゴシエーション）は読み捨てる。
 *   "exit"/"quit" で切断。出力は runCommandCaptured() が横取りしたもの。
 *
 * 実装方針: listen(backlog=1) の accept ループを1本のタスクで回す。
 * accept() は1回に1接続しか返らず、次のクライアントは前のセッションが
 * 終わって accept() へ戻るまでバックログで待たされるので、これだけで
 * 「同時は1クライアントまで」が自然に成り立つ（専用の排他制御は不要）。
 */
#include "uwb_net_internal.hpp"

#include "sdkconfig.h"

#if CONFIG_UWB_NET_ENABLE

#include <atomic>
#include <cerrno>
#include <cstdio>
#include <cstring>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"

namespace uwb::net::internal {

namespace {

constexpr const char* kTag  = "uwb_net_tcp";
constexpr size_t kLineMax     = 160; // scratchpad/NET_SPEC.md §2・§5 (runCommandCapturedと同じ上限)

int g_listenSock = -1;
std::atomic<bool> g_clientConnected{false};
TaskHandle_t g_tcpTaskHandle = nullptr;

/** runCommandCaptured() の CmdOutFn。'\n' を "\r\n" に正規化しつつソケットへ送る
 *  （テキストコンソールとして自然な改行にする）。送信エラーは無視する
 *  （次の recv()/send() で切断として検出されるので、ここで個別に扱う必要は無い）。 */
void sendCapturedOut(const char* data, size_t len, void* user)
{
    const int sock = *static_cast<int*>(user);
    if (sock < 0 || data == nullptr || len == 0) {
        return;
    }
    size_t start = 0;
    for (size_t i = 0; i < len; ++i) {
        if (data[i] == '\n') {
            if (i > start) {
                send(sock, data + start, i - start, 0);
            }
            send(sock, "\r\n", 2, 0);
            start = i + 1;
        }
    }
    if (start < len) {
        send(sock, data + start, len - start, 0);
    }
}

void sendPrompt(int sock, const char* name)
{
    char buf[40];
    const int n = std::snprintf(buf, sizeof(buf), "%s> ", name);
    if (n > 0) {
        const size_t sendLen = (static_cast<size_t>(n) < sizeof(buf)) ? static_cast<size_t>(n) : sizeof(buf) - 1;
        send(sock, buf, sendLen, 0);
    }
}

/** telnet の IAC（Interpret As Command）を読み捨てる小さな状態機械。
 *  scratchpad/NET_SPEC.md §2 の簡易仕様どおり: 通常のIACは「0xFFに続く2バイト」
 *  を丸ごと捨て、0xFF 0xFA (SB) の後は 0xFF 0xF0 (SE) が来るまで捨て続ける。 */
struct TelnetFilter {
    enum class State : uint8_t { Normal, IacSeen, SkipOne, SubNeg };
    State state          = State::Normal;
    bool subNegSawIac  = false;

    /** 1バイト処理する。データとして扱ってよければ true（*outC に書く）。 */
    bool feed(uint8_t c, char* outC)
    {
        switch (state) {
        case State::SubNeg:
            if (subNegSawIac && c == 0xF0) {
                state          = State::Normal;
                subNegSawIac = false;
            } else {
                subNegSawIac = (c == 0xFF);
            }
            return false;
        case State::SkipOne:
            state = State::Normal;
            return false;
        case State::IacSeen:
            if (c == 0xFA) { // SB: サブネゴシエーション開始
                state          = State::SubNeg;
                subNegSawIac = false;
            } else {
                // それ以外は「IACに続く2バイト」の2バイト目としてもう1バイト捨てる。
                state = State::SkipOne;
            }
            return false;
        case State::Normal:
        default:
            if (c == 0xFF) {
                state = State::IacSeen;
                return false;
            }
            *outC = static_cast<char>(c);
            return true;
        }
    }
};

void sessionLoop(int sock)
{
    const Config& cfg = config();

    char banner[128];
    int n = std::snprintf(banner, sizeof(banner), "uwb-net console (%s). Type 'exit' to close.\r\n", cfg.name);
    if (n > 0) {
        const size_t sendLen = (static_cast<size_t>(n) < sizeof(banner)) ? static_cast<size_t>(n) : sizeof(banner) - 1;
        send(sock, banner, sendLen, 0);
    }
    sendPrompt(sock, cfg.name);

    TelnetFilter telnet;
    char lineBuf[kLineMax + 1];
    size_t lineLen = 0;
    uint8_t rxBuf[128];

    while (true) {
        const int got = recv(sock, rxBuf, sizeof(rxBuf), 0);
        if (got <= 0) {
            break; // 切断またはエラー
        }

        bool shouldClose = false;
        for (int i = 0; i < got && !shouldClose; ++i) {
            char c;
            if (!telnet.feed(rxBuf[i], &c)) {
                continue; // telnet制御シーケンスの一部として読み捨てた
            }
            if (c == '\r') {
                continue; // CRは無視、LFで確定させる
            }
            if (c == '\n') {
                lineBuf[lineLen] = '\0';
                if (std::strcmp(lineBuf, "exit") == 0 || std::strcmp(lineBuf, "quit") == 0) {
                    send(sock, "bye\r\n", 5, 0);
                    shouldClose = true;
                    break;
                }
                runCommandCaptured(lineBuf, &sendCapturedOut, &sock);
                sendPrompt(sock, cfg.name);
                lineLen = 0;
                continue;
            }
            if (lineLen < kLineMax) {
                lineBuf[lineLen++] = c;
            }
            // 160文字を超えた分は黙って捨てる（次の改行までコマンド自体は無視され
            // 続ける。runCommandCaptured側の切り詰めとの二重防御）。
        }
        if (shouldClose) {
            break;
        }
    }
    close(sock);
}

void tcpTask(void* /*arg*/)
{
    while (true) {
        struct sockaddr_in clientAddr;
        socklen_t clientLen = sizeof(clientAddr);
        const int clientSock =
            accept(g_listenSock, reinterpret_cast<struct sockaddr*>(&clientAddr), &clientLen);
        if (clientSock < 0) {
            ESP_LOGW(kTag, "accept() failed: errno=%d", errno);
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        g_clientConnected.store(true, std::memory_order_relaxed);
        sessionLoop(clientSock);
        g_clientConnected.store(false, std::memory_order_relaxed);
    }
}

} // namespace

/* ==================================================================== *
 * uwb_net_internal.hpp で宣言された関数
 * ==================================================================== */

esp_err_t tcpConsoleStart()
{
    const Config& cfg = config();

    g_listenSock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (g_listenSock < 0) {
        ESP_LOGE(kTag, "socket() failed: errno=%d", errno);
        return ESP_FAIL;
    }

    const int reuse = 1;
    setsockopt(g_listenSock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    struct sockaddr_in addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port         = htons(cfg.consolePort);
    if (bind(g_listenSock, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        ESP_LOGE(kTag, "bind(0.0.0.0:%u) failed: errno=%d", static_cast<unsigned>(cfg.consolePort), errno);
        close(g_listenSock);
        g_listenSock = -1;
        return ESP_FAIL;
    }
    if (listen(g_listenSock, 1) < 0) {
        ESP_LOGE(kTag, "listen() failed: errno=%d", errno);
        close(g_listenSock);
        g_listenSock = -1;
        return ESP_FAIL;
    }

    // スタック8192: USBシリアルREPL(6144)と同じコマンド群を実行しうる上、
    // "net probe"/"wifi scan" 等の重いコマンドも通すため、REPLより余裕を持たせる
    // （レビュー指摘 2026-08-31）。
    const BaseType_t ok =
        xTaskCreatePinnedToCore(&tcpTask, "uwb_net_tcp", 8192, nullptr, 5, &g_tcpTaskHandle, 0);
    if (ok != pdPASS) {
        ESP_LOGE(kTag, "uwb_net_tcp タスクの起動に失敗しました");
        close(g_listenSock);
        g_listenSock = -1;
        return ESP_FAIL;
    }

    ESP_LOGI(kTag, "tcp console: listening on port %u", static_cast<unsigned>(cfg.consolePort));
    return ESP_OK;
}

bool tcpConsoleClientConnected()
{
    return g_clientConnected.load(std::memory_order_relaxed);
}

TaskHandle_t tcpConsoleTaskHandle()
{
    return g_tcpTaskHandle;
}

} // namespace uwb::net::internal

#else // !CONFIG_UWB_NET_ENABLE

namespace uwb::net::internal {

esp_err_t tcpConsoleStart()
{
    return ESP_OK;
}

bool tcpConsoleClientConnected()
{
    return false;
}

TaskHandle_t tcpConsoleTaskHandle()
{
    return nullptr;
}

} // namespace uwb::net::internal

#endif // CONFIG_UWB_NET_ENABLE
