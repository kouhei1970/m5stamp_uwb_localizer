/**
 * @file uwb_net.cpp
 * @brief uwb_net.hpp の公開 API 本体（start/publishLine/isStarted/ipString/
 *        registerConsoleCommands）と internal::config()。
 *        Public API surface of uwb_net.hpp, plus internal::config().
 *
 * ここが行うのは各下請けファイル（wifi/sink/udp/cmd/tcp/http）を正しい順序で
 * 起動することと、publishLine() の「間引き（decimation）」の判断だけ。
 * This file's only jobs are: start the sub-modules (wifi/sink/udp/cmd/tcp/
 * http) in a safe order, and decide the decimation in publishLine().
 *
 * 間引きの実体を uwb_net_sink.cpp ではなくここに置く理由: uwb_net_internal.hpp
 * は sinkPushRaw()（間引きを通さない生の積み込み）しか公開しておらず、
 * 「行の種類ごとに直近の採用時刻を覚えて間引く」判断そのものは公開 API
 * publishLine() の仕事として scratchpad/NET_SPEC.md §4 に書かれている。
 * Decimation lives here (not in uwb_net_sink.cpp) because uwb_net_internal.hpp
 * only exposes sinkPushRaw() (an undecimated raw push) - the per-type
 * decimation decision is publishLine()'s job per the spec.
 */
#include "uwb_net.hpp"

#include "sdkconfig.h"

#if CONFIG_UWB_NET_ENABLE

#include <cstdio>
#include <cstring>

#include "esp_log.h"
#include "esp_timer.h"

#include "uwb_net_internal.hpp"

namespace uwb::net {

namespace {

constexpr const char* kTag = "uwb_net";

Config g_config;
bool g_started = false;

/** "meas"/"fix"/"range" 行ごとの直近採用時刻 [us]（esp_timer_get_time() 基準）。
 *  Last-accepted timestamp per high-rate line type, in esp_timer microseconds.
 *
 *  両ファームとも JSON 行を出すタスクは 1 つだけ（タグ: uwb_log タスク、
 *  アンカー: main タスク）なので、この 3 変数への書き込みは実質単一タスク
 *  からしか起きず、追加のミューテックスは不要（publishLine() は「長く
 *  ブロックしない」ことが要件であり、ロックはこの単純化と相性が悪い）。
 *  Both firmwares have exactly one task emitting JSON lines (tag: uwb_log
 *  task; anchor: main task), so these three counters are in practice
 *  single-writer - no mutex needed, which suits publishLine()'s
 *  never-block-for-long requirement. */
int64_t g_lastMeasUs  = 0;
int64_t g_lastFixUs   = 0;
int64_t g_lastRangeUs = 0;

/**
 * @brief line の "type" が高頻度種別（meas/fix/range）なら対応する直近時刻へ
 * のポインタを返す。それ以外（anchors/stats/anchor_stats/node/...）は
 * nullptr（間引きしない）。
 * @return 高頻度種別なら g_lastMeasUs/g_lastFixUs/g_lastRangeUs のいずれか、
 *         それ以外は nullptr。
 */
int64_t* highRateSlotFor(const char* line)
{
    const char* type = strstr(line, "\"type\":\"");
    if (type == nullptr) {
        return nullptr;
    }
    type += 8; // "\"type\":\"" の長さぶん進める。Skip past the literal.
    if (std::strncmp(type, "meas", 4) == 0 && type[4] == '"') {
        return &g_lastMeasUs;
    }
    if (std::strncmp(type, "fix", 3) == 0 && type[3] == '"') {
        return &g_lastFixUs;
    }
    if (std::strncmp(type, "range", 5) == 0 && type[5] == '"') {
        return &g_lastRangeUs;
    }
    return nullptr;
}

} // namespace

namespace internal {

const Config& config()
{
    return g_config;
}

} // namespace internal

esp_err_t registerConsoleCommands()
{
    // "wifi" と "net"（net probe ...）の両方をここでまとめて登録する。
    // 片方が失敗してももう片方は登録を試みる（診断用コマンドなので、可能な
    // 限り両方使える状態にしておきたい）。
    const esp_err_t wifiErr  = internal::wifiRegisterConsoleCommands();
    const esp_err_t probeErr = internal::probeRegisterConsoleCommands();
    return (wifiErr != ESP_OK) ? wifiErr : probeErr;
}

esp_err_t start(const Config& cfg)
{
    if (g_started) {
        ESP_LOGW(kTag, "start() が2回呼ばれました。無視します / start() called twice, ignoring");
        return ESP_OK;
    }

    g_config      = cfg;
    g_lastMeasUs  = 0;
    g_lastFixUs   = 0;
    g_lastRangeUs = 0;

    // リモートコマンド実行（uwb_net_cmd.cpp）用ミューテックスを最初に作る。
    // wifiStart() より前に済ませておけば、httpStart()/tcpConsoleStart() が
    // 有効になった瞬間から runCommandCaptured() が安全に呼べる
    // （レビュー指摘 2026-08-31: 遅延生成のTOCTOUを避けるため一本化した）。
    const esp_err_t cmdErr = internal::cmdInit();
    if (cmdErr != ESP_OK) {
        ESP_LOGE(kTag, "cmdInit() 失敗 (err=%s)。uwb_net を起動できません", esp_err_to_name(cmdErr));
        return cmdErr;
    }

    // Wi-Fi はすべての土台（HTTP/UDP/TCPコンソールが乗るネットワークそのもの）
    // なので、失敗したらここで諦める。
    // Wi-Fi is the foundation everything else rides on; bail out if it fails.
    esp_err_t err = internal::wifiStart();
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "wifiStart() 失敗 (err=%s)。uwb_net を起動できません", esp_err_to_name(err));
        return err;
    }

    // HTTP/UDP/TCPコンソールはそれぞれ独立した「あれば嬉しい」機能なので、
    // どれか1つが失敗しても残りは起動を試みる（USBシリアル運用は元々生きている）。
    // HTTP/UDP/TCP console are independent "nice to have" features - a
    // failure in one doesn't stop the others (USB serial still works either way).
    if ((err = internal::httpStart()) != ESP_OK) {
        ESP_LOGE(kTag, "httpStart() 失敗 (err=%s)。ブラウザダッシュボードは使えません",
                 esp_err_to_name(err));
    }
    if ((err = internal::udpStart()) != ESP_OK) {
        ESP_LOGE(kTag, "udpStart() 失敗 (err=%s)。UDP 配信/集約は使えません", esp_err_to_name(err));
    }
    if ((err = internal::tcpConsoleStart()) != ESP_OK) {
        ESP_LOGE(kTag, "tcpConsoleStart() 失敗 (err=%s)。TCP コンソールは使えません",
                 esp_err_to_name(err));
    }

    // sink（リングバッファ + 配信タスク）は最後に起動する: httpBroadcast()/
    // udpSend() を呼び始める前に、それぞれの起動処理を先に済ませておくため
    // （どちらも「未起動なら黙って何もしない」実装だが、順序を揃えておく方が
    // 素直で診断ログの見た目も自然になる）。
    // sink (ring buffer + tx task) starts last, after http/udp are already
    // set up (both degrade gracefully if called before their own start, but
    // this ordering is the more natural one to read and to log).
    if ((err = internal::sinkStart()) != ESP_OK) {
        ESP_LOGE(kTag, "sinkStart() 失敗 (err=%s)。行の配信ができません", esp_err_to_name(err));
        return err;
    }

    g_started = true;
    ESP_LOGI(kTag, "uwb_net started: role=%s name=%s addr=%s http_port=%u console_port=%u udp_port=%u",
             (cfg.role == Role::Tag) ? "tag" : "anchor", cfg.name, cfg.addr,
             static_cast<unsigned>(cfg.httpPort), static_cast<unsigned>(cfg.consolePort),
             static_cast<unsigned>(cfg.udpPort));
    return ESP_OK;
}

void publishLine(const char* line, size_t len)
{
    if (!g_started || line == nullptr || len == 0) {
        return;
    }

    int64_t* const slot = highRateSlotFor(line);
    if (slot != nullptr && g_config.highRateMinIntervalMs > 0) {
        const int64_t nowUs         = esp_timer_get_time();
        const int64_t minIntervalUs = static_cast<int64_t>(g_config.highRateMinIntervalMs) * 1000;
        if ((nowUs - *slot) < minIntervalUs) {
            return; // 間引き。Decimated.
        }
        *slot = nowUs;
    }

    internal::sinkPushRaw(line, len);
}

bool isStarted()
{
    return g_started;
}

size_t ipString(char* buf, size_t cap)
{
    if (buf == nullptr || cap == 0) {
        return 0;
    }
    internal::WifiStatus st;
    internal::wifiStatus(st);
    const char* ip     = (st.ip[0] != '\0') ? st.ip : "0.0.0.0";
    const int written    = std::snprintf(buf, cap, "%s", ip);
    if (written < 0) {
        buf[0] = '\0';
        return 0;
    }
    const size_t n = static_cast<size_t>(written);
    return (n < cap) ? n : (cap - 1);
}

} // namespace uwb::net

#else // !CONFIG_UWB_NET_ENABLE

/* CONFIG_UWB_NET_ENABLE=n: 公開 API はすべて no-op。ヘッダ (uwb_net.hpp) は
 * 無条件でそのまま使えるので、呼び出し側 (firmware/tag, firmware/anchor) の
 * main.cpp / *_console.cpp は #if で分岐せずに常に同じコードを書ける。
 * CONFIG_UWB_NET_ENABLE=n: every public API becomes a no-op. The header stays
 * unconditional, so callers never need their own #if branches. */
namespace uwb::net {

esp_err_t registerConsoleCommands()
{
    return ESP_OK;
}

esp_err_t start(const Config& cfg)
{
    (void)cfg;
    return ESP_OK;
}

void publishLine(const char* line, size_t len)
{
    (void)line;
    (void)len;
}

bool isStarted()
{
    return false;
}

size_t ipString(char* buf, size_t cap)
{
    if (buf != nullptr && cap > 0) {
        buf[0] = '\0';
    }
    return 0;
}

} // namespace uwb::net

#endif // CONFIG_UWB_NET_ENABLE
