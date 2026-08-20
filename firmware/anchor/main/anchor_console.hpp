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
 * 測距ループ（respondDSRange() をブロッキングで呼び続ける）は従来どおり
 * app_main のタスクのまま。共有するのは
 *
 *   - ショートアドレス（コンソールが書き、測距ループが毎回読む）
 *   - 測距統計（測距ループが書き、コンソールの info が読む）
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
    const char* boardName  = "";  //!< "AtomS3" 等
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
 * @brief いま有効なショートアドレス。測距ループが 1 周ごとに呼ぶ。
 *
 * コンソールで addr set した場合、**次の respond 呼び出しから**新しい値に
 * なる（進行中の 1 回の TWR には影響しない）。
 */
uint16_t currentShortAddr();

/** 統計を公開する。測距ループが応答のたびに呼ぶ。 */
void publishStats(const Stats& stats);

/**
 * @brief コンソール（REPL）を起動する。内部で専用タスクが起動する。
 * @return ESP_OK 以外なら REPL は動かないが、測距そのものは継続してよい。
 */
esp_err_t consoleStart();

} // namespace anchorapp
