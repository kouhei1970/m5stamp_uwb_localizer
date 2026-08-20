/**
 * @file tag_console.hpp
 * @brief タグのシリアルコンソール（USB-Serial/JTAG 上の REPL）と、
 * 測位ループ・コンソールタスク間で共有する状態。
 *
 * 目的は「アンカーの座標を数 cm 直すたびに再ビルド・再書き込み」をやめること。
 *
 * ------------------------------------------------------------------
 * 測位ループとコンソールの同時実行をどう解決したか
 * ------------------------------------------------------------------
 * **編集用のシャドウコピー + 1 周期境界での差し替え**（片方向の二重バッファ）
 * を採用した。
 *
 *   - コンソールのコマンドは、本モジュールが持つ**編集用コピー**
 *     （AnchorEntry の配列）だけを書き換え、pending フラグを立てる。
 *     測位に使われている生きたテーブル（uwb::AnchorTable）には触れない。
 *   - 測位ループは 1 周期の**先頭**で takePendingTable() を呼ぶ。変更が
 *     あればそこで初めて AnchorTable::set() を呼んで差し替える。差し替えは
 *     測距も測位も走っていない瞬間にしか起こらない。
 *   - ミューテックスが守るのは編集用コピー（数百バイトの memcpy）だけで、
 *     測距・測位・JSON 出力の間はロックを一切保持しない。
 *
 * この方式にした理由:
 *
 *   1. AnchorTable は uwb_config.anchors が内部記憶域を指す構造で、
 *      set() の最中に測位ソルバから読まれると n_anchors と実体が食い違う。
 *      「測位が走っていない瞬間にしか差し替えない」ことを構造で保証したい。
 *   2. 生きたテーブルを直接ロックで守る方式だと、測距 1 周（DS-TWR で
 *      アンカー 5 台ぶん、数十 ms）の間ロックを持ち続けることになり、
 *      コンソールの応答が測距周期に引きずられる。
 *   3. 「変更したら reboot」は最も単純だが、座標を数 cm ずつ追い込む作業で
 *      毎回再起動するのは、まさに今回無くしたかった手間そのもの。
 *
 * 委譲指示の「設定変更は次の測位周期から反映されればよい」を素直に満たす。
 *
 * 本ヘッダおよび tag_console.cpp は CONFIG_UWB_TAG_CONSOLE が有効なときだけ
 * ビルドされる（main/CMakeLists.txt を参照）。
 */
#pragma once

#include <cstddef>
#include <cstdint>

#include "esp_err.h"

#include "uwb_cfgstore.hpp"
#include "uwb_ranging_types.hpp"

namespace tagapp {

/** 測位ループが公開する状態。info コマンドの表示にだけ使う。 */
struct Status {
    uint32_t epochs   = 0;    //!< 回した周期数
    uint32_t okFixes  = 0;    //!< うち Lv2 が ok=1 だった回数
    uint32_t cycleMs  = 0;    //!< 直近 1 周のポーリング所要時間 [ms]
    bool lastOk       = false;//!< 直近の Lv2 が解けたか
    float p[3]        = {0, 0, 0}; //!< 直近の推定位置 [m]
    float gdop        = 0.0f;
    float residualRms = 0.0f;
    int nUsed         = 0;
    int nTotal        = 0;
};

/** 起動時に確定していて、以後変わらない表示用の情報。 */
struct StaticInfo {
    const char* boardName  = "";
    const char* methodName = "";
    const char* chipName   = "";
    uint32_t deviceId      = 0;
    uint16_t tagAddr       = 0;
    uint16_t panId         = 0;
    uwb::ConfigSource source = uwb::ConfigSource::Default;

    /** コンパイル時の既定テーブル（tag の kAnchors[]）。reset-config で戻す先。
     *  static 記憶域を指していること（寿命はプログラム全体）。 */
    const uwb::AnchorEntry* defaults = nullptr;
    size_t defaultCount               = 0;
};

/**
 * @brief 共有状態を初期化する。app_main が REPL 起動前に 1 回だけ呼ぶ。
 * @param initial      起動時に採用したテーブル（NVS もしくは既定値）
 * @param initialCount その件数
 * @return ミューテックスを作れたら true。
 */
bool sharedInit(const uwb::AnchorEntry* initial, size_t initialCount, const StaticInfo& info);

/**
 * @brief コンソールが編集したテーブルを取り出す。測位ループが 1 周期の
 * **先頭**で呼ぶ。
 *
 * @param out      取り出し先（outCap 件ぶん）
 * @param outCap   out の容量 [件]
 * @param outCount 取り出した件数
 * @return 変更があって out を書いたら true。変更が無ければ false（out は不変）。
 */
bool takePendingTable(uwb::AnchorEntry* out, size_t outCap, size_t* outCount);

/** 状態を公開する。測位ループが 1 周期ごとに呼ぶ。 */
void publishStatus(const Status& status);

/**
 * @brief JSON Lines を出力してよいか。
 *
 * 測位ループは毎周期 2 行の JSON を吐くので、そのままではコンソールへの
 * 入力が流れてしまい設定作業ができない。output コマンドで一時的に止められる
 * ようにしてある（既定は on ＝従来どおり出力する）。
 */
bool jsonOutputEnabled();

/**
 * @brief コンソール（REPL）を起動する。内部で専用タスクが起動する。
 * @return ESP_OK 以外なら REPL は動かないが、測位そのものは継続してよい。
 */
esp_err_t consoleStart();

} // namespace tagapp
