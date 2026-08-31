/**
 * @file anchor_console.hpp
 * @brief アンカーのシリアルコンソール（USB-Serial/JTAG 上の REPL）と、
 * 測距ループ・コンソールタスク間で共有する状態。
 *
 * 目的は「アンカー 5 台に別々のショートアドレスを焼くために、5 回
 * ビルドし直す」のをやめること。実行時にコンソールから変更し、NVS へ
 * 保存できるようにする。
 *
 * ------------------------------------------------------------------
 * タスク構成と排他
 * ------------------------------------------------------------------
 * REPL は esp_console_start_repl() が専用の FreeRTOS タスクを作って回す。
 *
 * 【v2 (docs/ARCHITECTURE_V2.md §2.1)】測距（電波の送受信）は
 * `uwb_radio` タスク（main.cpp の radioTask()、`uwb::Responder::service()`
 * を無限に回すだけ）が専任で担い、REPL・統計行の出力は main タスク
 * （app_main、コア0）が行う。3 タスク構成になったが、このヘッダが共有する
 * 状態は変わらない:
 *
 *   - ショートアドレス（コンソールが書く。**radio タスクはもう読まない** -
 *     uwb::Responder は begin() 時の ResponderConfig を保持し続けるため、
 *     addr set は reboot するまで無線には反映されない。main.cpp 冒頭
 *     コメント「v2 での変更点」参照。この値は addr/info コマンドの表示にだけ使う）
 *   - 測距統計（main タスクが UWB_ANCHOR_STATS_INTERVAL_MS ごとに
 *     publishConsoleStats() で書き、コンソールの info が読む）
 *
 * の 2 つだけで、いずれも小さいので 1 本のミューテックスで保護する。
 * ミューテックスを保持したままブロッキング API を呼ぶ箇所は無い。
 *
 * 本ヘッダおよび anchor_console.cpp は CONFIG_UWB_ANCHOR_CONSOLE が有効な
 * ときだけビルドされる（main/CMakeLists.txt を参照）。
 */
#pragma once

#include <cstdint>

#include "esp_err.h"

#include "uwb_cfgstore.hpp"

namespace anchorapp {

/** 測距ループが公開する統計。info コマンドの表示にだけ使う。 */
struct Stats {
    uint32_t ok      = 0;   //!< 応答に成功した回数
    uint32_t fail    = 0;   //!< 失敗した回数（RxTimeout は数えない）
    uint32_t samples = 0;   //!< 距離統計に入れたサンプル数（DS-TWR のみ）
    double meanMm    = 0.0; //!< 距離の平均 [mm]
    double stdMm     = 0.0; //!< 距離の標準偏差 [mm]
};

/** 起動時に確定していて、以後変わらない表示用の情報。 */
struct StaticInfo {
    const char* boardName  = "";  //!< "M5StampS3A" 等
    const char* methodName = "";  //!< "DS-TWR" 等
    const char* chipName   = "";  //!< uwb::Qm33120::chipName()
    uint32_t deviceId      = 0;   //!< uwb::Qm33120::deviceId()
    uint16_t defaultAddr   = 0;   //!< Kconfig の CONFIG_UWB_ANCHOR_SHORT_ADDR
    uint16_t tagAddr       = 0;   //!< 対向タグのショートアドレス
    uint16_t panId         = 0;   //!< PAN ID
    uwb::ConfigSource source = uwb::ConfigSource::Default; //!< 起動時にどちらを採用したか
};

/**
 * @brief 共有状態を初期化する。app_main が REPL 起動前に 1 回だけ呼ぶ。
 * @param addr 起動時に採用したショートアドレス（NVS もしくは Kconfig 既定値）
 * @return ミューテックスを作れたら true。false なら呼び出し側はコンソールを
 *         起動せず、従来どおり固定アドレスで動作を続けること。
 */
bool sharedInit(uint16_t addr, const StaticInfo& info);

/**
 * @brief いま有効なショートアドレス（addr / info コマンドの表示用）。
 *
 * 【v2 (docs/ARCHITECTURE_V2.md §2.1)】以前は測距ループが1周ごとに読み直し、
 * addr set が次の応答から反映されていたが、uwb::Responder は begin() 時の
 * ResponderConfig を保持し続けるため、この関数はもう無線側からは読まれない
 * （main.cpp 冒頭コメント「v2 での変更点」参照）。addr set 直後の表示用の値
 * （まだ save/reboot していない「これから有効にしたい値」）を返すだけの
 * ものになった。
 */
uint16_t currentShortAddr();

/** 統計を公開する。main タスクが UWB_ANCHOR_STATS_INTERVAL_MS ごとに呼ぶ
 *  （publishConsoleStats()、main.cpp）。 */
void publishStats(const Stats& stats);

/**
 * @brief コンソール（REPL）を起動する。内部で専用タスクが起動する。
 * @return ESP_OK 以外なら REPL は動かないが、測距そのものは継続してよい。
 */
esp_err_t consoleStart();

} // namespace anchorapp
