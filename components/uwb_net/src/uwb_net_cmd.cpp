/**
 * @file uwb_net_cmd.cpp
 * @brief esp_console_run を stdout 差し替えで実行し出力を取り出す共通処理、
 *        および JSON エスケープの共通ヘルパー (uwb_net_internal.hpp の分担:
 *        uwb_net_cmd.cpp)。
 *        Runs esp_console_run with stdout/stderr redirected, capturing its
 *        output; also hosts the shared JSON-escaping helpers.
 *
 * scratchpad/NET_SPEC.md §5:
 *   fopencookie() で「書き込みだけ横取りする」FILE* を作り、呼び出しタスクの
 *   stdout/stderr を一時的にそれへ差し替えて esp_console_run() を実行する。
 *   ESP-IDF の newlib は FreeRTOS タスクごとに別々の _reent 構造体を持つため
 *   （`stdout` マクロは `_REENT->_stdout` == `__getreent()->_stdout` に展開され、
 *   `__getreent()`（components/freertos/esp_additions/freertos_tasks_c_additions.h）
 *   は「現在実行中のタスクの _reent」を返す）、この代入は**呼び出したタスク
 *   自身の stdout だけ**を差し替える。他のタスク（USBシリアルのコンソール
 *   REPLタスク等）が同時に printf しても影響し合わない。
 *   同時に複数のリモートコマンドが実行されないよう、ミューテックスで直列化する。
 *
 * ミューテックスは cmdInit()（uwb::net::start() の先頭で呼ぶ）で1回だけ作る。
 * runCommandCaptured() 側の遅延生成（「nullptrなら作る」）は複数タスクから
 * 同時に呼ばれるとTOCTOU（生成前チェックと生成の間に別タスクが割り込む）で
 * 二重生成しうるため、レビュー指摘（2026-08-31）でここへ一本化した。
 *
 * jsonEscapeAppendByte/jsonEscapeAppend/appendLiteral は元々 uwb_net_http.cpp
 * だけが持っていたが、uwb_net_sink.cpp の "node" 行の文字列（ssid/name/addr等）
 * でも同じ制御文字エスケープ（\uXXXX 含む）が要るとレビューで指摘されたため、
 * ここへ移して共有する（uwb_net_internal.hpp で宣言）。
 */
#include "uwb_net_internal.hpp"

#include "sdkconfig.h"

#if CONFIG_UWB_NET_ENABLE

#include <cstdio>
#include <cstring>
#include <sys/types.h>

#include "esp_console.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

namespace uwb::net::internal {

namespace {

constexpr const char* kTag      = "uwb_net_cmd";
constexpr size_t kMaxLineLen    = 160; // scratchpad/NET_SPEC.md §5・§2 (TCPコンソールと同じ上限)

SemaphoreHandle_t g_mutex = nullptr;

/** fopencookie() のクッキー。書き込み1回ぶんをそのまま CmdOutFn へ横流しする。 */
struct WriteCtx {
    CmdOutFn out;
    void* user;
};

/** cookie_io_functions_t::write。esp_console_run() 実行中の printf/ESP_LOGx が
 *  最終的にここへ来る。呼び出し側の CmdOutFn がバッファ管理を担うので、ここでは
 *  「渡された分は全部受け取った」ことにして n を返す。 */
ssize_t cookieWrite(void* cookie, const char* buf, size_t n)
{
    auto* ctx = static_cast<WriteCtx*>(cookie);
    if (ctx != nullptr && ctx->out != nullptr && buf != nullptr && n > 0) {
        ctx->out(buf, n, ctx->user);
    }
    return static_cast<ssize_t>(n);
}

} // namespace

/* ==================================================================== *
 * JSON エスケープ共通ヘルパー（uwb_net_internal.hpp で宣言。
 * uwb_net_http.cpp / uwb_net_sink.cpp の両方から使う）
 * ==================================================================== */

void jsonEscapeAppendByte(char* dst, size_t& off, size_t cap, uint8_t byte)
{
    char tmp[6];
    size_t tmpLen;
    switch (byte) {
        case '"': tmp[0] = '\\'; tmp[1] = '"'; tmpLen = 2; break;
        case '\\': tmp[0] = '\\'; tmp[1] = '\\'; tmpLen = 2; break;
        case '\n': tmp[0] = '\\'; tmp[1] = 'n'; tmpLen = 2; break;
        case '\r': tmp[0] = '\\'; tmp[1] = 'r'; tmpLen = 2; break;
        case '\t': tmp[0] = '\\'; tmp[1] = 't'; tmpLen = 2; break;
        default:
            if (byte < 0x20) {
                // その他の制御文字は \u00XX。 Other control characters as \u00XX.
                static const char kHex[] = "0123456789abcdef";
                tmp[0] = '\\'; tmp[1] = 'u'; tmp[2] = '0'; tmp[3] = '0';
                tmp[4] = kHex[(byte >> 4) & 0xF];
                tmp[5] = kHex[byte & 0xF];
                tmpLen = 6;
            } else {
                // 0x80 以上は UTF-8 の続きバイトとしてそのまま通す。
                // Bytes >= 0x80 are UTF-8 continuation/lead bytes; pass through unescaped.
                tmp[0] = static_cast<char>(byte);
                tmpLen = 1;
            }
            break;
    }
    for (size_t i = 0; i < tmpLen && off < cap; ++i) {
        dst[off++] = tmp[i];
    }
}

void jsonEscapeAppend(char* dst, size_t& off, size_t cap, const char* src)
{
    if (src == nullptr) {
        return;
    }
    for (const unsigned char* p = reinterpret_cast<const unsigned char*>(src); *p != '\0'; ++p) {
        jsonEscapeAppendByte(dst, off, cap, *p);
    }
}

void appendLiteral(char* dst, size_t& off, size_t cap, const char* src)
{
    if (src == nullptr) {
        return;
    }
    for (const char* p = src; *p != '\0' && off < cap; ++p) {
        dst[off++] = *p;
    }
}

/* ==================================================================== *
 * uwb_net_internal.hpp で宣言された残りの関数
 * ==================================================================== */

esp_err_t cmdInit()
{
    if (g_mutex != nullptr) {
        return ESP_OK; // 二重初期化は無害だが、生成し直さない。
    }
    g_mutex = xSemaphoreCreateMutex();
    if (g_mutex == nullptr) {
        ESP_LOGE(kTag, "コマンド実行用ミューテックスを作れませんでした");
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

int runCommandCaptured(const char* line, CmdOutFn out, void* user)
{
    if (line == nullptr) {
        return static_cast<int>(ESP_ERR_INVALID_ARG);
    }

    if (g_mutex == nullptr) {
        // cmdInit() が uwb::net::start() の先頭で必ず先に呼ばれる設計だが、
        // 万一それより前にここへ来た場合は「作ろうとして失敗した」ことにせず
        // 素直にエラーを返す（TOCTOUを避けるため、ここでは生成し直さない。
        // レビュー指摘 2026-08-31）。
        ESP_LOGE(kTag, "runCommandCaptured(): cmdInit() 未実行です");
        if (out != nullptr) {
            static const char kMsg[] = "internal error: command subsystem not initialized\n";
            out(kMsg, sizeof(kMsg) - 1, user);
        }
        return 1;
    }

    // 160バイトを超える行は許可しない（切り詰める）。
    char safeLine[kMaxLineLen + 1];
    const size_t rawLen = std::strlen(line);
    const size_t len      = (rawLen < kMaxLineLen) ? rawLen : kMaxLineLen;
    std::memcpy(safeLine, line, len);
    safeLine[len] = '\0';

    if (xSemaphoreTake(g_mutex, portMAX_DELAY) != pdTRUE) {
        return 1;
    }

    WriteCtx ctx{out, user};
    cookie_io_functions_t funcs = {};
    funcs.write                  = &cookieWrite;
    FILE* const f                 = fopencookie(&ctx, "w", funcs);
    if (f == nullptr) {
        ESP_LOGE(kTag, "fopencookie() failed");
        xSemaphoreGive(g_mutex);
        return 1;
    }
    // 行単位でバッファリングする: esp_console_run 実行中に printf の途中経過を
    // 溜め込みすぎないよう、かつ1バイトずつ横流ししてCmdOutFnを呼びすぎない
    // よう256バイトの行バッファを挟む。
    setvbuf(f, nullptr, _IOLBF, 256);

    FILE* const savedStdout = stdout;
    FILE* const savedStderr = stderr;
    stdout                    = f; // このタスクの _reent->_stdout だけを差し替える（上のファイルコメント参照）
    stderr                    = f;

    int cmdRet             = 1;
    const esp_err_t runErr = esp_console_run(safeLine, &cmdRet);

    std::fflush(f);
    stdout = savedStdout;
    stderr = savedStderr;
    fclose(f);

    int ret = cmdRet;
    if (runErr == ESP_ERR_NOT_FOUND) {
        static const char kMsg[] = "Unrecognized command\n";
        if (out != nullptr) {
            out(kMsg, sizeof(kMsg) - 1, user);
        }
        ret = 1;
    } else if (runErr == ESP_ERR_INVALID_ARG) {
        // 空行。何も出さない。
        ret = 0;
    } else if (runErr != ESP_OK) {
        ret = static_cast<int>(runErr);
    }

    xSemaphoreGive(g_mutex);
    return ret;
}

} // namespace uwb::net::internal

#else // !CONFIG_UWB_NET_ENABLE

namespace uwb::net::internal {

esp_err_t cmdInit()
{
    return ESP_OK;
}

int runCommandCaptured(const char*, CmdOutFn, void*)
{
    return 1;
}

void jsonEscapeAppendByte(char*, size_t&, size_t, uint8_t) {}
void jsonEscapeAppend(char*, size_t&, size_t, const char*) {}
void appendLiteral(char*, size_t&, size_t, const char*) {}

} // namespace uwb::net::internal

#endif // CONFIG_UWB_NET_ENABLE
