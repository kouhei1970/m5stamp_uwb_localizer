/**
 * @file uwb_net.hpp
 * @brief Wi-Fi + browser dashboard + remote console for the UWB tag / anchor firmware.
 *        Wi-Fi 接続、ブラウザ用ダッシュボード（HTTP + WebSocket）、無線コンソール（TCP / WebSocket）。
 *
 * 設計は docs/HANDOFF.md §0-E と docs/NET_DASHBOARD.md を参照。
 *
 * 役割分担（呼び出し側 = firmware/tag, firmware/anchor の main.cpp）:
 *   1. ConfigStore::init() で NVS を初期化した後、コンソール開始 (consoleStart) の前に
 *      registerConsoleCommands() を呼ぶ（"wifi" コマンドを登録する）。
 *   2. UWB の初期化が済んでから start(cfg) を呼ぶ（Wi-Fi、HTTP、UDP、TCP コンソールを起動する）。
 *   3. JSON 行を stdout に書いた直後に publishLine(line, len) を呼ぶ（送り先を 1 つ増やすだけ）。
 *      publishLine はコピーして即座に返る（ブロックしない・電波を扱うタスクからは呼ばない）。
 *
 * すべてのネットワーク処理は core 0 で動く。電波（UWB）を扱うタスクは core 1 のまま。
 * All networking runs on core 0; the UWB radio tasks stay on core 1.
 */
#pragma once

#include <cstddef>
#include <cstdint>

#include "esp_err.h"

namespace uwb::net {

enum class Role : uint8_t { Tag = 0, Anchor = 1 };

/**
 * 1 Hz の "node" 行に役割固有の JSON メンバを追記するコールバック。
 * `"phy":"850k/256","temp_c":38.5` のように、先頭のカンマ・波括弧なしで書く。
 * snprintf 流に「書き込んだ（書こうとした）バイト数」を返す。cap を超えたら切り詰められる。
 * ネットワーク側のタスク（core 0）から呼ばれるので、UWB のレジスタを触らず、
 * 既にメモリにある値（起動時に決まった PHY 文字列など）だけを書くこと。
 */
using StatusJsonFn = size_t (*)(char* buf, size_t cap, void* user);

struct Config {
    Role role = Role::Tag;
    /** ホスト名（mDNS: "<name>.local"）と SoftAP の SSID 接頭辞。例 "uwb-tag" / "uwb-anchor-0002" */
    const char* name = "uwb-tag";
    /** "node" 行の "addr"。タグは "tag0"、アンカーは "0x0002" のような短縮アドレス */
    const char* addr = "tag0";
    /** true: 他機（アンカー）が UDP で送ってくる行を受信して自分の配信に混ぜる（集約役 = タグ） */
    bool aggregate = false;
    /** true: 自分の行を UDP ブロードキャストで送る（アンカー） */
    bool forward = false;
    uint16_t httpPort    = 80;
    uint16_t consolePort = 23;
    uint16_t udpPort     = 5006;
    /** 高頻度の行（"meas" / "fix" / "range"）を配信する最短間隔 [ms]。0 で間引きなし */
    uint32_t highRateMinIntervalMs = 50;
    StatusJsonFn statusFn = nullptr;
    void* statusUser      = nullptr;
};

/** "wifi" コンソールコマンドを登録する（esp_console_init 済みであること = consoleStart の中で
 *  registerCommands と同じ場所から呼ぶ）。CONFIG_UWB_NET_ENABLE=n のときは何もしない。 */
esp_err_t registerConsoleCommands();

/** Wi-Fi（NVS の設定に従い STA / SoftAP）、mDNS、HTTP+WebSocket、UDP、TCP コンソールを起動する。
 *  cfg はコピーされる（name/addr の文字列は静的寿命であること）。 */
esp_err_t start(const Config& cfg);

/** JSON 1 行（末尾 '\n' 込み）を配信キューへ積む。スレッド安全・非ブロッキング。
 *  キューが満杯なら捨ててカウンタ（"node" 行の "drops"）に計上する。 */
void publishLine(const char* line, size_t len);

/** 起動済みか（start が成功したか） */
bool isStarted();

/** IP アドレス文字列（未接続なら "0.0.0.0"）。戻り値は書き込んだ長さ */
size_t ipString(char* buf, size_t cap);

}  // namespace uwb::net
