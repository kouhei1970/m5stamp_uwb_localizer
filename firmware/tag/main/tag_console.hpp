/**
 * @file tag_console.hpp
 * @brief タグのシリアルコンソール（USB-Serial/JTAG 上の REPL）と、
 * 測位サービス・コンソールタスク間で共有する状態。
 *
 * 目的は「アンカーの座標を数 cm 直すたびに再ビルド・再書き込み」をやめること。
 *
 * ------------------------------------------------------------------
 * v2（uwb::RangingService 導入後）の同時実行の解き方
 * ------------------------------------------------------------------
 * docs/ARCHITECTURE_V2.md §3.2 の指示により、v1 の「編集用シャドウコピー +
 * 1周期境界での差し替え」は廃止し、**コンソールが生きた uwb::AnchorTable を
 * service.lockTable() 保持中に直接編集する**方式に変えた。
 *
 *   - `anchor set/delay/enable/disable/count` は、`service.lockTable()`
 *     を取ってから `AnchorTable::update()`/`set()` を直接呼ぶ。
 *   - 測距・測位を回す処理タスク（uwb::RangingService 内部）は1周期の間
 *     ずっと同じテーブルミューテックスを保持するので
 *     （uwb_ranging_service.hpp の `tableMutex()` コメント参照）、
 *     コンソールの編集と測距・測位が同時に走ることはない。**代償として、
 *     編集コマンドは実行中の1周期（DS-TWR・再試行込みで最大数百ms程度）
 *     ぶん待たされることがある**（設定作業はホットパスではないため許容）。
 *   - **必ず `service.tableMutex()` の素のハンドルではなく
 *     `service.lockTable()`/`unlockTable()` を使うこと。** 処理タスクは
 *     1周期を終えるたびに素のミューテックスを離してすぐ取り直すため
 *     （cycleIntervalMs==0 だと数µs後）、素のハンドルへ直接
 *     xSemaphoreTake() すると、FreeRTOS のミューテックスが FIFO でない
 *     ことにより、このコンソールタスク（優先度2）が無期限に飢餓する
 *     （実機で `info` / `anchor list` の固まりとして確認・修正済み。
 *     詳細は uwb_ranging_service.hpp 冒頭「tableMutex() の優先度逆転」
 *     参照）。`lockTable()` は待ち手を処理タスクへ知らせ、処理タスク側が
 *     1ティック譲ることでこれを避ける。
 *   - 表の構成が変わったら `service.resetStats()`（**unlockTable() で
 *     離してから**呼ぶこと。内部で lockTable()/unlockTable() を使って
 *     取り直すため、離す前に呼ぶと二重取得でデッドロックする）
 *     と `service.reinitEkf()`（EKF無効時は no-op）を呼ぶ。
 *   - 下流のJSON可視化へ変更後のアンカー一覧を伝えるため、`tableGeneration()`
 *     を1つ進める。main.cpp の uwb_log タスクはこのカウンタの変化を見て
 *     "type":"anchors" 行を出し直す。
 *   - `info` コマンド（旧 `status` 相当）は `service.getLatest()` から
 *     直近の周期結果を読む（測位ループ側が `publishStatus()` で押し込む
 *     方式は廃止した）。
 *
 * 本ヘッダおよび tag_console.cpp は CONFIG_UWB_TAG_CONSOLE が有効なときだけ
 * ビルドされる（main/CMakeLists.txt を参照）。
 */
#pragma once

#include <cstddef>
#include <cstdint>

#include "esp_err.h"

#include "uwb_cfgstore.hpp"
#include "uwb_ranging_anchor_table.hpp"
#include "uwb_ranging_service.hpp"
#include "uwb_ranging_types.hpp"

namespace tagapp {

/**
 * @brief テーブル編集後の後処理（配置チェック + 2D自動フォールバック + ログ）。
 *
 * main.cpp が実装し、起動時（初期テーブル適用時）とコンソールでの編集後の
 * 両方から呼ぶ共通関数。CONFIG_UWB_TAG_AUTO_2D_FALLBACK 等の Kconfig 判断・
 * ESP_LOGW は main.cpp 側の責務なので、tag_console.cpp はこの関数ポインタ
 * 経由で呼ぶだけにして重複させない。
 *
 * Post-edit housekeeping (placement check + optional 2D fallback + logging),
 * implemented in main.cpp and shared between startup and console edits, so
 * tag_console.cpp doesn't duplicate the Kconfig-driven policy logic.
 *
 * 呼び出し側は既に service.lockTable() を取っていること。
 */
using PlacementPolicyFn = void (*)(uwb::AnchorTable& table);

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

    /** 上記参照。main.cpp が渡す。 */
    PlacementPolicyFn applyPlacementPolicy = nullptr;
};

/**
 * @brief 共有状態を初期化する。app_main が REPL 起動前（RangingService::start()
 * より後）に 1 回だけ呼ぶ。
 *
 * @param table   測位サービスが読み書きする生きたアンカー登録テーブル
 *                （呼び出し側が寿命を管理する。以後この参照を保持する）
 * @param service 起動済みの RangingService（lockTable()/unlockTable()/
 *                tableMutex()/resetStats()/reinitEkf()/getLatest() を
 *                使う。以後この参照を保持する）
 * @return ミューテックスを作れたら true。
 */
bool sharedInit(uwb::AnchorTable& table, uwb::RangingService& service, const StaticInfo& info);

/**
 * @brief アンカー登録テーブルの「世代」。コンソールが表を書き換えるたび、
 * および `output on` で出力を再開するたびに 1 増える。
 *
 * main.cpp の uwb_log タスクはこの値の変化を監視し、変化したら
 * "type":"anchors" 行を出し直す（下流の可視化に新しい表を伝えるため）。
 * CONFIG_UWB_TAG_CONSOLE が無効なときは呼ばれない想定（表は起動後不変）。
 */
uint32_t tableGeneration();

/**
 * @brief JSON Lines を出力してよいか。
 *
 * 測位サービスは毎周期 JSON を作るため、そのままではコンソールへの入力が
 * 流れてしまい設定作業ができない。output コマンドで一時的に止められる
 * ようにしてある（既定は on ＝従来どおり出力する）。
 */
bool jsonOutputEnabled();

/**
 * @brief コンソール（REPL）を起動する。内部で専用タスクが起動する。
 * @return ESP_OK 以外なら REPL は動かないが、測位そのものは継続してよい。
 */
esp_err_t consoleStart();

} // namespace tagapp
