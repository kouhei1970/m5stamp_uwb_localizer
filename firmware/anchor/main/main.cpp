/**
 * @file main.cpp
 * @brief v2 production anchor (responder) firmware for components/uwb_qm33120
 * (docs/ARCHITECTURE_V2.md §2.1/§2.4/§2.5)。
 *
 * v2 の再設計（背景は docs/ARCHITECTURE_V2.md §0/§1）:
 *   - 旧版は毎ループ `respondRange()`/`respondDSRange()` をブロッキング呼び
 *     出しする 1 本道の構造で、Poll 待ちに 200ms のホスト側窓があった
 *     （窓の切り替わりごとに受信機を切って入れ直す空白ができ、タグの周期と
 *     一致すると位相ロックして DS-TWR が 0% になった。docs/HANDOFF.md §0-C）。
 *   - v2 は電波を受信し続けたまま来たフレームの種類で分岐するイベント駆動の
 *     ステートマシン `uwb::Responder`（components/uwb_qm33120/include/
 *     uwb_qm33120_responder.hpp、判断ロジックの純粋関数は
 *     uwb_qm33120_responder_fsm.hpp の `uwb::decide()`）に置き換える。
 *   - This file (main.cpp) now only *drives* two tasks (§2.1's table):
 *       - `uwb_radio`（新設、コア UWB_ANCHOR_RADIO_TASK_CORE、優先度
 *         UWB_ANCHOR_RADIO_TASK_PRIO）: `responder.begin()` してから
 *         `responder.service()` を無限に呼ぶだけ。電波（チップ）を触る
 *         唯一のタスク（§1「電波を触るタスクは1つ」）。ループ中は
 *         begin() 失敗時以外ログを一切出さない。
 *       - `app_main` 自身がそのまま main タスク（コア0）になる（return
 *         せず、統計行・イベント行・LED・スタック監視のループへ入る）。
 *   - 旧 `runRole()`（respondRange()/respondDSRange() をブロッキング呼び
 *     出しするループ）はこのファームからは削除した。関数自体
 *     （respondRange()/respondDSRange()）は components/uwb_qm33120 の
 *     ドライバ層に残っており、firmware/twr は引き続きそれを使う
 *     （docs/ARCHITECTURE_V2.md §1「実験ファーム firmware/twr は当面旧構造
 *     のまま残し、A/B の基準として使う」）。
 *
 * firmware/twr との違い（変わらない部分）:
 *   - ロール選択自体が無い(常にANCHOR)。
 *   - 自分のショートアドレスをKconfig(UWB_ANCHOR_SHORT_ADDR)で個体ごとに
 *     設定できる。手持ちのM5Stamp UWB Module 5台それぞれに異なる値を書き込んで
 *     区別する運用を想定している。
 *
 * ショートアドレスは **NVS に保存された値を優先し、無ければ Kconfig の
 * UWB_ANCHOR_SHORT_ADDR を既定値として使う**(components/uwb_cfgstore)。
 * NVS が空・未初期化・破損のいずれでも必ず既定値へフォールバックするので、
 * 購入者が何も設定しなければ従来どおり Kconfig の値でそのまま動く。
 * 実行時の変更は USB-Serial/JTAG 上のシリアルコンソール(anchor_console.cpp、
 * CONFIG_UWB_ANCHOR_CONSOLE で無効化可)から addr set / save で行う。
 *
 * 【v2 での変更点】`uwb::Responder` は begin() 時に渡された `ResponderConfig`
 * （panId/shortAddr を含む）のコピーを、end() されるまでずっと保持する
 * (uwb_qm33120_responder.hpp begin() のコメント)。旧版は
 * `currentShortAddr()` を respondDSRange() 呼び出しのたびに読み直していた
 * ため `addr set` が「次の応答から」反映されたが、v2 の Responder は
 * begin() を呼ぶ app_main の起動シーケンス中に一度だけ ResponderConfig を
 * 組み立てるので、実行時の `addr set` は radio タスクには伝わらない。
 * このファームでは意図的に **`save` の後に `reboot` を要求する** 単純な
 * 方式を選んだ（anchor_console.cpp の cmdAddr() 側のメッセージを参照。
 * 複数タスクにまたがるミューテックス保護付きのホットスワップより、
 * 「アンカーは給電前提で常時稼働、設定変更は稀」という運用（§0-C）には
 * この方が単純で事故りにくいという判断）。
 *
 * 【docs/archive/REIMPL_PLAN.md R3-1/R9】以下の各 static constexpr は、旧
 * third_party/M5Stamp-UWB/examples の .ino 値をそのまま踏襲していたが、
 * RANGE_HOST_TIMEOUT_MS(100→10)/RESULT_RX_AFTER_FINAL_TX_DLY_UUS(500→200)/
 * RESULT_REPEAT_COUNT(3→1) の3つは uwb::RangeConfig/DSRangeConfig 側の
 * 新しい既定値（components/uwb_qm33120/include/uwb_qm33120_types.hpp）と
 * 揃える形で更新した。根拠はそちらのフィールドコメントを参照。
 */
#include <cmath>
#include <cstdio>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "uwb_cfgstore.hpp"
#include "uwb_port.h"
#include "uwb_status_led.h"
#include "uwb_qm33120.hpp"
#include "uwb_qm33120_phy_kconfig.hpp"  // docs/ARCHITECTURE_V2.md §4: phyConfigFromKconfig()
#include "uwb_qm33120_responder.hpp"    // docs/ARCHITECTURE_V2.md §2.3: uwb::Responder / ResponderConfig / ResponderStats / RangeEvent

#if CONFIG_UWB_ANCHOR_CONSOLE
#include "anchor_console.hpp"
#endif

// Wi-Fi + ブラウザダッシュボード + 無線コンソール（components/uwb_net）。
// CONFIG_UWB_NET_ENABLE=n でも公開関数は no-op になるだけなので、この
// #include と下の呼び出しは常に有効にしてよい（uwb_net.hpp 冒頭コメント）。
#include "uwb_net.hpp"

#if CONFIG_UWB_ANCHOR_BOARD_ATOMS3
#include "boards/atoms3.h"
#define BOARD_UWB_PORT_CONFIG BOARD_ATOMS3_UWB_PORT_CONFIG
#define BOARD_NAME            "AtomS3(pinout " BOARD_ATOMS3_PINOUT_NAME ")"
#else
#include "boards/stamps3.h"
#define BOARD_UWB_PORT_CONFIG BOARD_STAMPS3_UWB_PORT_CONFIG
#define BOARD_NAME            "M5StampS3A"
#define BOARD_STATUS_LED_GPIO BOARD_STAMPS3_STATUS_LED_GPIO
#endif

#if CONFIG_UWB_ANCHOR_METHOD_DS
#define METHOD_NAME "DS-TWR"
#else
#define METHOD_NAME "SS-TWR"
#endif

static const char* TAG = "uwb_anchor";

/* --- タイミングプリセット(docs/TIMING_PRESETS.md、タスクD) ---
 * Kconfig の choice UWB_TIMING_PROFILE から uwb::TimingProfile を決める。
 * タグ側(firmware/tag)と必ず同じものを選ぶこと。 */
#if CONFIG_UWB_TIMING_PROFILE_ANCHOR_IRQ
static constexpr uwb::TimingProfile TIMING_PROFILE = uwb::TimingProfile::AnchorIrq;
#elif CONFIG_UWB_TIMING_PROFILE_BOTH_IRQ
static constexpr uwb::TimingProfile TIMING_PROFILE = uwb::TimingProfile::BothIrq;
#else
static constexpr uwb::TimingProfile TIMING_PROFILE = uwb::TimingProfile::PollingBoth;
#endif

/* begin() 成功後に「実際に適用されたプリセット」で上書きする。
 * IRQ 線が死んでいると init() が PollingBoth へ降格するため、
 * コンパイル時の TIMING_PROFILE をそのまま使ってはいけない。
 * The profile init() actually applied - init() downgrades IRQ presets to
 * PollingBoth when the IRQ line turns out to be dead, so the compile-time
 * TIMING_PROFILE must not be used directly. */
static uwb::TimingProfile g_effectiveTimingProfile = TIMING_PROFILE;

#define UWB_DEV_ID_EXPECTED 0xDECA0314UL

/* --- ネットワーク共通パラメータ --- */
static constexpr uint16_t PAN_ID = 0xDECA;
// タグ側(initiator)のショートアドレス。firmware/tag の実装は固定値として扱う。
static constexpr uint16_t TAG_SHORT_ADDR = 0x0001;
// このアンカー自身のショートアドレスの **既定値**。Kconfigの
// UWB_ANCHOR_SHORT_ADDR (main/Kconfig.projbuild) は
// 「NVSが空のときの初期値」という位置づけに変わった。実運用では
// シリアルコンソールの addr set / save で個体ごとに設定する。
static constexpr uint16_t ANCHOR_SHORT_ADDR_DEFAULT = CONFIG_UWB_ANCHOR_SHORT_ADDR;

// 起動時に確定するショートアドレス。NVSに保存があればその値、無ければ
// ANCHOR_SHORT_ADDR_DEFAULT。この値は起動シーケンス中に一度だけ
// ResponderConfig::shortAddr へコピーされ、radio タスク開始後は変わらない
// （このファイル冒頭のコメント「v2 での変更点」参照）。
static uint16_t g_shortAddr = ANCHOR_SHORT_ADDR_DEFAULT;

/* ==================================================================== *
 * uwb_net（components/uwb_net）の "node" 行へ載せる role固有情報
 * ==================================================================== *
 * firmware/tag/main/main.cpp の同名セクションと同じ理由・同じ制約
 * （statusFn はネットワークタスク(コア0)から1秒ごとに呼ばれ、電波を一切
 * 触ってはいけない。scratchpad/NET_SPEC.md §8）。g_netName/g_netAddr は
 * uwb::net::Config::name/addr が「静的寿命の文字列であること」を要求する
 * （uwb_net.hpp Config コメント）ため、app_main で g_shortAddr が確定した
 * 直後に一度だけ整形する。 */
static char g_netName[24]        = "";  // "uwb-anchor-XXXX"
static char g_netAddr[8]          = "";  // "0xXXXX"
static char g_netPhyStr[40]       = "";
static char g_netPllCoarseStr[8] = "";

/** uwb_qm33120_phy_kconfig.cpp のファイル内 pacSizeCount() と同じ対応表
 *  （非公開のためログ表示用にここで複製する。firmware/tag/main/main.cpp と同一）。 */
static unsigned pacSizeCountForNet(uwb::PacSize pac)
{
    switch (pac) {
    case uwb::PacSize::Pac4:  return 4;
    case uwb::PacSize::Pac8:  return 8;
    case uwb::PacSize::Pac16: return 16;
    case uwb::PacSize::Pac32: return 32;
    }
    return 0;
}

/** uwb::net::Config::statusFn 本体。"node" 行へ
 *  `"phy":"...","pll_coarse":"...","method":"...","short_addr":"..."`
 *  を追記する（先頭カンマ・波括弧なし。uwb_net.hpp StatusJsonFn のコメント参照）。 */
static size_t anchorNetStatus(char* buf, size_t cap, void* /*user*/)
{
    const int n = std::snprintf(
        buf, cap,
        "\"phy\":\"%s\",\"pll_coarse\":\"%s\",\"method\":\"%s\",\"short_addr\":\"%s\"", g_netPhyStr,
        g_netPllCoarseStr,
#if CONFIG_UWB_ANCHOR_METHOD_DS
        "DS",
#else
        "SS",
#endif
        g_netAddr);
    return (n > 0) ? static_cast<size_t>(n) : 0;
}

/**
 * @brief 測距統計をコンソール(info コマンド)へ公開する。
 * uwb::ResponderStats -> anchorapp::Stats の変換。
 *
 * 【v2 での意味変化】旧版の ok/fail は respondRange()/respondDSRange() の
 * 1呼び出しごとの成否だった。Responder は事象駆動なので同じ粒度が無く、
 * このファームでは次の対応で近似する:
 *   - DS-TWR: ok = 完了した交換の数 (results。Result送信まで成功した回数)。
 *     fail = ホスト側から見える失敗の合計 (rxErrors + txFailures +
 *     finalTimeouts)。restarts（他タグのPollで進行中の交換を打ち切った回数）
 *     はそれ自体は失敗ではない（新しい交換として応答している）ため含めない。
 *   - SS-TWR: ok = 送信できたResponseの数 (responses)。SS-TWRのAnchor側は
 *     距離を計算しないため、Result相当の「完了」概念が無い。
 *
 * コンソール無効時は空関数になり、呼び出しは最適化で消える(従来と同じ挙動)。
 */
static inline void publishConsoleStats(const uwb::ResponderStats& s)
{
#if CONFIG_UWB_ANCHOR_CONSOLE
    anchorapp::Stats out;
#if CONFIG_UWB_ANCHOR_METHOD_DS
    out.ok = s.results;
#else
    out.ok = s.responses;
#endif
    out.fail    = s.rxErrors + s.txFailures + s.finalTimeouts;
    out.samples = s.distance.count;
    out.meanMm  = s.distance.mean;
    out.stdMm   = s.distance.stddev();
    anchorapp::publishStats(out);
#else
    (void)s;
#endif
}

/**
 * @brief boards 以下のヘッダの BOARD_UWB_PORT_CONFIG を uwb::Config へコピーする。
 * firmware/twr/main/main.cpp の makeConfigFromBoard() と同一の変換。
 */
static uwb::Config makeConfigFromBoard()
{
    const uwb_port_config_t port = BOARD_UWB_PORT_CONFIG;

    uwb::Config cfg;
    cfg.spi_host     = port.spi_host;
    cfg.pin_sck      = port.pin_sck;
    cfg.pin_mosi     = port.pin_mosi;
    cfg.pin_miso     = port.pin_miso;
    cfg.pin_cs       = port.pin_cs;
    cfg.pin_rst      = port.pin_rst;
    cfg.pin_irq      = port.pin_irq;
    cfg.pin_wakeup   = port.pin_wakeup;
    cfg.pin_gp7      = port.pin_gp7;
    cfg.spi_slow_hz  = port.spi_slow_hz;
    cfg.spi_fast_hz  = port.spi_fast_hz;
    cfg.init_spi_bus = port.init_spi_bus;
    // CONFIG_UWB_ENABLE_IRQ が n のとき、ESP-IDF の Kconfig はこのマクロ自体を
    // 定義しない（0として定義されるわけではない）ので #ifdef ではなく
    // #if defined(...) && ... で判定する (docs/IRQ_POLICY.md, main/Kconfig.projbuild)。
    // pin_irq が UWB_PORT_PIN_UNUSED の場合や ISR 登録に失敗した場合は
    // Qm33120::init() が自動的にポーリングへフォールバックする。
#if defined(CONFIG_UWB_ENABLE_IRQ) && CONFIG_UWB_ENABLE_IRQ
    cfg.use_irq = true;
#else
    cfg.use_irq = false;
#endif
    // タイミングプリセット(docs/TIMING_PRESETS.md、タスクD)。相手(Tag)へ
    // Poll/Response フレームで伝わり、不一致検出に使われる
    // (uwb_qm33120_twr.cpp checkTimingTag()/logTimingMismatch())。
    cfg.timing_profile = TIMING_PROFILE;
    return cfg;
}

/* ResponderConfig 組み立ては選択された方式(SS/DS)だけをビルドする。両方を
 * 常に定義すると、選択されなかった側の makeXxRangeConfig() が
 * -Wunused-function の対象になる(firmware/twr と同じ理由)。 */

#if CONFIG_UWB_ANCHOR_METHOD_DS
/* =========================================================================
 * DS-TWR: cfg.ds (uwb::DSRangeConfig) の組み立て。値そのものは旧runRole()の
 * makeRangeConfig()と同一（examples/DS_TWR_ANCHOR/DS_TWR_ANCHOR.ino 準拠、
 * その後のDS-TWR原因特定 docs/HANDOFF.md §0-C での修正込み）。
 * ========================================================================= */

static constexpr uint32_t RX_TIMEOUT_UUS = 3000;
// 2026-08-29 DS-TWR原因特定 (docs/HANDOFF.md §0-C): 10->20。
// components/uwb_qm33120 の DSRangeConfig::hostTimeoutMs フィールド
// コメント参照（respondDSRange()のFinal待ちがハードウェアRXFTOより先に
// 切れないための余裕）。firmware/twr の ANCHOR+DS-TWR ブロックと同じ変更。
// v2 の Responder::service() も同じ hostTimeoutMs をホスト側デッドラインの
// 計算に使う（uwb_qm33120_responder.cpp の onRespond() コメント参照）。
static constexpr uint32_t RANGE_HOST_TIMEOUT_MS = 20;
static constexpr uint32_t RESPONSE_RX_AFTER_TX_DLY_UUS = 1500;
static constexpr uint32_t RESPONSE_TX_DLY_UUS = 3000;
// 2026-08-29 DS-TWR原因特定 (docs/HANDOFF.md §0-C(2)): 1800->3000。
// Response側と対称にし、850kbps/preamble256でのDW3000 UM §9.4.1エラッタ
// を避ける。DSRangeConfig::finalTxDelayUus のフィールドコメント参照。
static constexpr uint32_t FINAL_TX_DLY_UUS = 3000;
// 2026-08-29 DS-TWR原因特定: 500->1500。上のfinalTxDelayUusと対称に
// (DSRangeConfig::finalRxAfterResponseTxDelayUus のフィールドコメント参照)。
static constexpr uint32_t FINAL_RX_AFTER_RESPONSE_TX_DLY_UUS = 1500;
static constexpr uint32_t RESULT_RX_AFTER_FINAL_TX_DLY_UUS   = 200;
static constexpr uint8_t RESULT_REPEAT_COUNT                 = 1;
static constexpr uint32_t RESULT_REPEAT_GAP_MS                = 3;

static uwb::DSRangeConfig makeDsRangeConfig(uint16_t selfAddr)
{
    uwb::DSRangeConfig range;
    range.panId                          = PAN_ID;
    range.initiatorAddress               = TAG_SHORT_ADDR;
    range.responderAddress               = selfAddr;
    range.responseRxAfterTxDelayUus      = RESPONSE_RX_AFTER_TX_DLY_UUS;
    range.responseTxDelayUus             = RESPONSE_TX_DLY_UUS;
    range.finalTxDelayUus                = FINAL_TX_DLY_UUS;
    range.finalRxAfterResponseTxDelayUus = FINAL_RX_AFTER_RESPONSE_TX_DLY_UUS;
    range.resultRxAfterFinalTxDelayUus   = RESULT_RX_AFTER_FINAL_TX_DLY_UUS;
    range.rxTimeoutUus                   = RX_TIMEOUT_UUS;
    range.hostTimeoutMs                  = RANGE_HOST_TIMEOUT_MS;
    range.resultRepeatCount              = RESULT_REPEAT_COUNT;
    range.resultRepeatGapMs              = RESULT_REPEAT_GAP_MS;
    // タイミングプリセットを適用する(docs/TIMING_PRESETS.md)。上の個別代入
    // (RESPONSE_RX_AFTER_TX_DLY_UUS等)より必ず後に呼び、プリセットの値が
    // 確実に効くようにする(*Uusフィールドのみ上書き。panId/アドレス/
    // hostTimeoutMs/resultRepeatCount/resultRepeatGapMsは触らない)。
    uwb::applyTimingProfile(range, g_effectiveTimingProfile);
    return range;
}

#else
/* =========================================================================
 * SS-TWR: cfg.ss (uwb::RangeConfig) の組み立て。値そのものは旧runRole()の
 * makeRangeConfig()と同一（examples/SS_TWR_ANCHOR/SS_TWR_ANCHOR.ino 準拠）。
 * ========================================================================= */

static constexpr uint32_t RX_TIMEOUT_UUS               = 3000;
static constexpr uint32_t RANGE_HOST_TIMEOUT_MS         = 10;
static constexpr uint32_t RESPONSE_RX_AFTER_TX_DLY_UUS  = 1500;
static constexpr uint32_t RESPONSE_TX_DLY_UUS           = 3000;

static uwb::RangeConfig makeSsRangeConfig(uint16_t selfAddr)
{
    uwb::RangeConfig range;
    range.panId                     = PAN_ID;
    range.initiatorAddress          = TAG_SHORT_ADDR;
    range.responderAddress          = selfAddr;
    range.responseRxAfterTxDelayUus = RESPONSE_RX_AFTER_TX_DLY_UUS;
    range.responseTxDelayUus        = RESPONSE_TX_DLY_UUS;
    range.rxTimeoutUus              = RX_TIMEOUT_UUS;
    range.hostTimeoutMs             = RANGE_HOST_TIMEOUT_MS;
    // タイミングプリセットを適用する(docs/TIMING_PRESETS.md)。上の個別代入
    // より必ず後に呼び、プリセットの値が確実に効くようにする(*Uusフィールド
    // のみ上書き。panId/アドレス/hostTimeoutMsは触らない)。
    //
    // 【この経路では uwb::Responder は個別のUus値を読まない】
    // uwb_qm33120_responder.cpp の onRespond() は
    // im.cfg.ss.responseTxDelayUus だけを読み、
    // responseRxAfterTxDelayUus/rxTimeoutUus はSS(即時応答)では使わない
    // （そもそも上書きしても電波の振る舞いは変わらない、旧makeRangeConfig()
    // 時代からの既知の不整合。値の出所を1本化するためにこの呼び出しは残す）。
    uwb::applyTimingProfile(range, g_effectiveTimingProfile);
    return range;
}
#endif

/**
 * @brief docs/ARCHITECTURE_V2.md §2.3 の uwb::ResponderConfig を組み立てる。
 * method・panId・shortAddr・ss/ds のタイミング・idleTickMs・
 * restartOnForeignPoll をすべてここで決める。app_main が起動シーケンス中に
 * 一度だけ呼ぶ（このファイル冒頭コメント「v2 での変更点」参照:
 * 実行時の addr set はこの呼び出しには影響しない）。
 */
static uwb::ResponderConfig makeResponderConfig(uint16_t selfAddr)
{
    uwb::ResponderConfig cfg;
    cfg.panId     = PAN_ID;
    cfg.shortAddr = selfAddr;
#if CONFIG_UWB_ANCHOR_METHOD_DS
    cfg.method = uwb::TwrMethod::DS;
    cfg.ds     = makeDsRangeConfig(selfAddr);
#else
    cfg.method = uwb::TwrMethod::SS;
    cfg.ss     = makeSsRangeConfig(selfAddr);
#endif
    cfg.idleTickMs = static_cast<uint32_t>(CONFIG_UWB_ANCHOR_IDLE_TICK_MS);
    // bool Kconfig が n のとき、ESP-IDF の Kconfig はこのマクロ自体を定義
    // しない（0として定義されるわけではない）ので、makeConfigFromBoard()の
    // cfg.use_irq と同じ理由で #if defined(...) && ... で判定する
    // （このファイルの makeConfigFromBoard() コメント参照。素の
    // `CONFIG_UWB_ANCHOR_RESTART_ON_FOREIGN_POLL` を式の中で直接使うと、
    // n が選ばれたビルドではマクロ未定義でコンパイルエラーになる）。
#if defined(CONFIG_UWB_ANCHOR_RESTART_ON_FOREIGN_POLL) && CONFIG_UWB_ANCHOR_RESTART_ON_FOREIGN_POLL
    cfg.restartOnForeignPoll = true;
#else
    cfg.restartOnForeignPoll = false;
#endif
    return cfg;
}

/* ==================================================================== *
 * uwb_radio タスク（docs/ARCHITECTURE_V2.md §2.1）: 電波の唯一の所有者。
 * responder.begin() してから責任を持って responder.service() を無限に
 * 呼ぶだけ。begin() 失敗時以外は一切ログを出さない
 * （§2.1「電波を扱うタスクは1つ」・§1「電波を触るタスクは統計・LED等の
 * 補助処理（統計出力・コンソール）から切り離す」の要請どおり、無線のホットループにログ出力の遅延を
 * 持ち込まない）。
 * ==================================================================== */

/** radioTask() に渡す引数。タスクが動き続ける間ずっと参照されるため static。 */
struct RadioTaskArgs {
    uwb::Qm33120* radio       = nullptr;
    uwb::Responder* responder = nullptr;
    uwb::ResponderConfig cfg;
};
static RadioTaskArgs g_radioTaskArgs;
static TaskHandle_t g_radioTaskHandle = nullptr;

static void radioTask(void* argRaw)
{
    RadioTaskArgs* args = static_cast<RadioTaskArgs*>(argRaw);

    if (!args->responder->begin(*args->radio, args->cfg)) {
        // begin()が失敗するのは radio 未初期化か、初回の dwt_rxenable() が
        // 失敗した場合のみ（uwb_qm33120_responder.hpp begin()のコメント）。
        // この1回だけがこのタスクの唯一のログ出力。
        ESP_LOGE(TAG, "Responder::begin() failed: error=%s", args->radio->lastErrorName());
        vTaskDelete(nullptr);
        return;
    }

    while (true) {
        args->responder->service();
    }
}

/* ==================================================================== *
 * main タスク（コア0、docs/ARCHITECTURE_V2.md §2.1/§2.5）が出す JSON 行。
 * タグ側(firmware/tag/main/main.cpp)と同じ流儀で、JSON行は printf 系で
 * 標準出力へ直接書く(ESP_LOGxはタイムスタンプ等のプレフィックスが付き
 * 先頭が'{'にならないため)。1行あたりの項目数が固定でループが要らないので、
 * タグ側のようなバッファ+jsonAppend()は使わず素のprintf/snprintfで足りる。
 * ==================================================================== */

/** ショートアドレスを "A0002"/"T0001" のような短縮ID文字列にする。 */
static void shortAddrToId(uint16_t addr, char prefix, char* out, size_t outSize)
{
    std::snprintf(out, outSize, "%c%04X", prefix, static_cast<unsigned>(addr));
}

/**
 * @brief docs/ARCHITECTURE_V2.md §2.5 の "type":"anchor_stats" 行。
 * UWB_ANCHOR_STATS_INTERVAL_MS ごとに main タスクが出す。旧版の
 * DS_RESP_STAT/SS_RESP_STAT テキストログを置き換える
 * （docs/GETTING_STARTED.md 参照。grepしていたスクリプトはこちらへ移行）。
 * dist_mean_m/dist_std_m は ResponderStats::distance が mm 単位で持つ値を
 * m へ変換する。
 */
static void printAnchorStatsLine(const uwb::ResponderStats& s, double tSec, const char* addrId)
{
    char buf[512];
    const int n = std::snprintf(
        buf, sizeof(buf),
        "{\"v\":1,\"type\":\"anchor_stats\",\"t\":%.3f,\"addr\":\"%s\",\"polls\":%lu,\"responses\":%lu,"
        "\"finals\":%lu,\"results\":%lu,\"restarts\":%lu,\"final_timeouts\":%lu,\"rx_errors\":%lu,"
        "\"tx_failures\":%lu,\"rearms\":%lu,\"other\":%lu,\"event_drops\":%lu,\"last_rx_status\":\"0x%08lX\","
        "\"dist_n\":%lu,\"dist_mean_m\":%.4f,\"dist_std_m\":%.4f}\n",
        tSec, addrId, static_cast<unsigned long>(s.polls), static_cast<unsigned long>(s.responses),
        static_cast<unsigned long>(s.finals), static_cast<unsigned long>(s.results),
        static_cast<unsigned long>(s.restarts), static_cast<unsigned long>(s.finalTimeouts),
        static_cast<unsigned long>(s.rxErrors), static_cast<unsigned long>(s.txFailures),
        static_cast<unsigned long>(s.rearms), static_cast<unsigned long>(s.other),
        static_cast<unsigned long>(s.eventDrops), static_cast<unsigned long>(s.lastRxStatus),
        static_cast<unsigned long>(s.distance.count), s.distance.mean / 1000.0, s.distance.stddev() / 1000.0);
    if (n <= 0) {
        return;
    }
    const size_t len = (static_cast<size_t>(n) < sizeof(buf)) ? static_cast<size_t>(n) : (sizeof(buf) - 1);
    if (uwb_port_usb_host_connected()) {  // ホスト不在時は USB へ書かない (uwb_port.h 参照) / skip USB writes with no host
        std::fputs(buf, stdout);
    }
    uwb::net::publishLine(buf, len);
}

#if CONFIG_UWB_ANCHOR_LOG_EVENTS
/**
 * @brief docs/ARCHITECTURE_V2.md §2.1/§2.5 の "type":"range" 行
 * (CONFIG_UWB_ANCHOR_LOG_EVENTS=y のときだけ)。DS-TWRの完了した交換1件ごと
 * に1行（SS-TWRはAnchor側で距離を計算しないため RangeEvent 自体が発生
 * しない。uwb_qm33120_responder_fsm.hpp RangeEvent のコメント参照）。
 *
 * Kconfigが n（既定）のときはこの関数自体をビルド対象から外す
 * （呼び出し側もこのKconfigで囲ってあるので、そうしないと
 * -Wunused-functionの対象になる。firmware/anchor の他の箇所と同じ流儀）。
 */
static void printRangeEventLine(const uwb::RangeEvent& ev, int64_t bootUs, const char* addrId)
{
    char peerId[8];
    shortAddrToId(ev.peer, 'T', peerId, sizeof(peerId));
    const double t = static_cast<double>(ev.tUs - bootUs) / 1e6;

    char buf[512];
    const int n =
        std::snprintf(buf, sizeof(buf),
                      "{\"v\":1,\"type\":\"range\",\"t\":%.3f,\"addr\":\"%s\",\"peer\":\"%s\",\"seq\":%u,\"d\":%.4f,"
                      "\"elapsed_us\":%lu}\n",
                      t, addrId, peerId, static_cast<unsigned>(ev.seq), static_cast<double>(ev.distanceMm) / 1000.0,
                      static_cast<unsigned long>(ev.elapsedUs));
    if (n <= 0) {
        return;
    }
    const size_t len = (static_cast<size_t>(n) < sizeof(buf)) ? static_cast<size_t>(n) : (sizeof(buf) - 1);
    if (uwb_port_usb_host_connected()) {  // ホスト不在時は USB へ書かない (uwb_port.h 参照) / skip USB writes with no host
        std::fputs(buf, stdout);
    }
    uwb::net::publishLine(buf, len);
}
#endif

extern "C" void app_main(void)
{
#ifdef BOARD_STATUS_LED_GPIO
/* Heartbeat colour tells the role apart once the boards are placed and no
 * serial monitor is attached: TAG = green, ANCHOR = red (components/
 * uwb_status_led). Only the M5StampS3A carries a WS2812 on a known GPIO -
 * the AtomS3 has an LCD instead - so the heartbeat compiles out there.
 * 設置後にシリアルを繋がない状態でも役割が分かるよう、ハートビートの色で
 * タグ(緑)とアンカー(赤)を見分ける。フルカラー LED を持つのは M5StampS3A
 * だけ(AtomS3 は LCD)なので、それ以外ではハートビートごと消える。 */
    (void)uwb_status_led_start_role_heartbeat(BOARD_STATUS_LED_GPIO, UWB_STATUS_LED_ROLE_ANCHOR);
#endif
    /* --- NVS からショートアドレスを読む（無ければKconfig既定値） ---
     * ConfigStore::init() が失敗しても loadAnchorAddr() は必ず既定値を返すので、
     * ここでは戻り値を見て中断しない。NVS破損で起動しなくなるのを避ける方針。 */
    uwb::ConfigStore::init();
    const uwb::ConfigSource addrSource =
        uwb::ConfigStore::loadAnchorAddr(ANCHOR_SHORT_ADDR_DEFAULT, &g_shortAddr);

    ESP_LOGI(TAG, "uwb_anchor firmware (v2/Responder), board=%s method=%s short_addr=0x%04X (%s)", BOARD_NAME,
             METHOD_NAME, g_shortAddr, uwb::configSourceName(addrSource));

    // uwb_net の Config::name/addr は静的寿命の文字列であることを要求する
    // ため、g_shortAddr が確定したこの時点で一度だけ整形する
    // （scratchpad/NET_SPEC.md §8: name="uwb-anchor-XXXX", addr="0xXXXX"）。
    std::snprintf(g_netName, sizeof(g_netName), "uwb-anchor-%04X", static_cast<unsigned>(g_shortAddr));
    // JSON 行の世界はアンカーを "A%04X" で表す（meas/stats/anchor_stats と同じ、
    // formatAnchorId()/shortAddrToId() の流儀）。"0x%04X" はコンソール表示用の
    // 表記で、ここで混ぜると同じ機体がダッシュボード上で 2 枚に割れる
    // (2026-08-31 に実機で確認)。
    // JSON lines use "A%04X" for anchors (same as meas/stats/anchor_stats);
    // "0x%04X" is the console notation. Mixing them split one device into
    // two dashboard cards (observed 2026-08-31).
    std::snprintf(g_netAddr, sizeof(g_netAddr), "A%04X", static_cast<unsigned>(g_shortAddr));

    // static にする理由: xTaskCreatePinnedToCore() が起こす uwb_radio タスクが
    // このオブジェクトへの生ポインタを保持し、タスクが動き続ける間ずっと
    // 参照する。app_main 自身は本関数末尾の while(1) から return しないので
    // 非staticでも実際には安全だが、firmware/tag/main/main.cpp が同じ理由で
    // static にしている（RangingService::start()向け）のに合わせ、制御フロー
    // の変更（将来 app_main にreturnを足す等）に対して頑健にしておく。
    // Must be static: the uwb_radio task (xTaskCreatePinnedToCore()) keeps a
    // raw pointer to this object for as long as it runs. app_main itself
    // never returns (its own while(1) below), so a plain local would in fact
    // stay valid too, but static matches firmware/tag/main/main.cpp's own
    // reasoning for the same pattern and is robust against a future change
    // to app_main's control flow.
    static uwb::Qm33120 uwbDevice;
    const uwb::Config cfg = makeConfigFromBoard();
    // docs/ARCHITECTURE_V2.md §4: PHY を共通 Kconfig (UWB_PHY_*,
    // components/uwb_qm33120/Kconfig) から選ぶ。firmware/tag も同じ関数を使う。
    const uwb::PhyConfig phy = uwb::phyConfigFromKconfig();

    if (!uwbDevice.begin(cfg, phy)) {
        ESP_LOGE(TAG, "begin() failed: error=%s", uwbDevice.lastErrorName());
        return;
    }

    ESP_LOGI(TAG, "deviceId=0x%08lX (expect 0x%08lX) chipName=%s isConnected=%d isInitialized=%d",
             (unsigned long)uwbDevice.deviceId(), (unsigned long)UWB_DEV_ID_EXPECTED, uwbDevice.chipName(),
             uwbDevice.isConnected(), uwbDevice.isInitialized());

    if (uwbDevice.deviceId() != UWB_DEV_ID_EXPECTED) {
        ESP_LOGE(TAG, "unexpected device id, aborting");
        return;
    }
    if (!uwbDevice.isInitialized()) {
        ESP_LOGE(TAG, "PHY init did not complete (isInitialized()==false), aborting");
        return;
    }

    // docs/ARCHITECTURE_V2.md §4: 実際に適用された PHY 設定を起動ログへ出す
    // （firmware/twr と同じ書式。スクリプトが grep する）。
    uwbDevice.logPhy(TAG);
    // 校正結果（PLL粗調整コード・OTP値等）を、強制する前の状態としてまず出す。
    uwbDevice.logCal(TAG);

    // 2026-08-30 実機結果: 本番既定(850kbps/256, PollingBoth, IRQ) + PLL
    // 自動校正まかせは初回試行成功率p=50.1%だったが、粗調整コードを0x23へ
    // 固定すると90%だった（docs/HANDOFF.md §0-B、components/uwb_qm33120/
    // Kconfig の UWB_PHY_PLL_COARSE_MODE ヘルプ参照）。0x23は基板固有の値
    // なので、既定は個体ごとのOTPから読む OTP モード。
#if CONFIG_UWB_PHY_PLL_COARSE_MODE_OTP
    {
        const bool pllOk = uwbDevice.forcePllCoarseFromOtp();
        if (!pllOk) {
            ESP_LOGW(TAG,
                     "forcePllCoarseFromOtp(): OTP field looked unprogrammed/invalid; left the chip's own "
                     "calibration in place");
        }
        uwbDevice.logCal(TAG);
    }
#elif CONFIG_UWB_PHY_PLL_COARSE_MODE_FIXED
    {
        const bool pllOk = uwbDevice.forcePllCoarse(static_cast<uint8_t>(CONFIG_UWB_PHY_PLL_COARSE_CH9));
        if (!pllOk) {
            ESP_LOGW(TAG,
                     "forcePllCoarse(0x%02X) could not lock the forced code; radio fell back to its normal "
                     "calibration path (see the DIAG_PLL_COARSE(*) lines above)",
                     (unsigned)CONFIG_UWB_PHY_PLL_COARSE_CH9);
        }
        uwbDevice.logCal(TAG);
    }
#endif
    // UWB_PHY_PLL_COARSE_MODE=Auto: no forcing call - the logCal() line
    // above already shows the chip's own auto-calibration result.

    // uwb_net の "node" 行用に、起動時に決まったPHY/PLL粗調整の要約を
    // 一度だけ静的バッファへ書いておく（firmware/tag/main/main.cpp の
    // 同名処理と同一の理由・同一のロジック）。
    std::snprintf(g_netPhyStr, sizeof(g_netPhyStr), "%s/pre%u/pac%u/ch%u",
                  (phy.dataRate == uwb::DataRate::Rate850K) ? "850k" : "6m8",
                  static_cast<unsigned>(phy.preambleLength), pacSizeCountForNet(phy.pacSize),
                  static_cast<unsigned>(phy.channel));
#if CONFIG_UWB_PHY_PLL_COARSE_MODE_FIXED
    std::snprintf(g_netPllCoarseStr, sizeof(g_netPllCoarseStr), "0x%02X",
                  static_cast<unsigned>(CONFIG_UWB_PHY_PLL_COARSE_CH9));
#elif CONFIG_UWB_PHY_PLL_COARSE_MODE_OTP
    std::snprintf(g_netPllCoarseStr, sizeof(g_netPllCoarseStr), "otp");
#else
    std::snprintf(g_netPllCoarseStr, sizeof(g_netPllCoarseStr), "auto");
#endif

    // init() may have downgraded the requested TIMING_PROFILE to PollingBoth
    // if the IRQ line turned out to be dead (Qm33120::verifyIrqLine()). Carry
    // the profile that was actually applied forward from here on.
    // IRQ 線が死んでいると init() が要求プリセットを PollingBoth へ降格させて
    // いることがあるため、以降は「実際に適用されたプリセット」を使う。
    g_effectiveTimingProfile = uwbDevice.config().timing_profile;

    // --- IRQ（起床信号）の実際の有効状態をログに出す (docs/IRQ_POLICY.md) ---
    // Kconfigの設定値(CONFIG_UWB_ENABLE_IRQ)ではなく、Qm33120::irqActive()が
    // 返す「実際に有効化できたか」を出す。フォールバックが起きていないかを
    // 起動ログだけで判別できるようにするため。
    if (uwbDevice.irqActive()) {
        ESP_LOGI(TAG, "irq=active (pin=%d)", cfg.pin_irq);
    } else if (cfg.pin_irq == UWB_PORT_PIN_UNUSED) {
        ESP_LOGI(TAG, "irq=polling (pin_irq unwired)");
    } else if (!cfg.use_irq) {
        ESP_LOGI(TAG, "irq=polling (disabled by Kconfig)");
    } else {
        ESP_LOGW(TAG, "irq=polling (enable failed)");
    }

    // --- タイミングプリセット(docs/TIMING_PRESETS.md) ---
    // g_effectiveTimingProfile（init()が実際に適用した値）と、requested=
    // （コンパイル時のTIMING_PROFILE）の両方を出す。実行時のPollingBothへの
    // 降格が起きていないかを起動ログだけで判別できるように。
#if CONFIG_UWB_ANCHOR_METHOD_DS
    {
        const uwb::TimingPresetDs ds = uwb::timingPresetDs(g_effectiveTimingProfile);
        ESP_LOGI(TAG,
                 "timing profile=%s (version=%u, requested=%s) wait=%s response_tx_delay=%luuus(%.0fus) "
                 "final_tx_delay=%luuus(%.0fus)",
                 uwb::timingProfileName(g_effectiveTimingProfile), (unsigned)uwb::kTimingPresetVersion,
                 uwb::timingProfileName(TIMING_PROFILE), uwbDevice.irqActive() ? "irq" : "polling",
                 (unsigned long)ds.responseTxDelayUus, ds.responseTxDelayUus * 1.02564,
                 (unsigned long)ds.finalTxDelayUus, ds.finalTxDelayUus * 1.02564);
    }
#else
    {
        const uwb::TimingPresetSs ss = uwb::timingPresetSs(g_effectiveTimingProfile);
        ESP_LOGI(TAG,
                 "timing profile=%s (version=%u, requested=%s) wait=%s response_tx_delay=%luuus(%.0fus)",
                 uwb::timingProfileName(g_effectiveTimingProfile), (unsigned)uwb::kTimingPresetVersion,
                 uwb::timingProfileName(TIMING_PROFILE), uwbDevice.irqActive() ? "irq" : "polling",
                 (unsigned long)ss.responseTxDelayUus, ss.responseTxDelayUus * 1.02564);
    }
#endif
    // 起動時チェック(docs/TIMING_PRESETS.md §4(b)): このプリセットが
    // アンカー側IRQを前提にしているのに、実際にはIRQが有効化されていない
    // 場合は目立つ警告を出す。**値は自動で変えない**(プリセットは相手と
    // 一致していることが要件のため。片側が勝手に変えると測距が壊れる)。
    if (uwb::timingProfileNeedsAnchorIrq(TIMING_PROFILE) && !uwbDevice.irqActive()) {
        ESP_LOGW(TAG,
                 "timing profile=%s は IRQ 前提ですが irqActive()=false です"
                 "(pin_irq 未配線 / Kconfigで無効 / ISR登録失敗のいずれか)。"
                 "待ちはポーリングへフォールバックしましたが、折返しに最大1.2msかかるため"
                 "このプリセットの締切に間に合わない可能性が高いです。"
                 "値は自動で変えません — タグ・アンカー双方を PollingBoth で焼き直すこと"
                 "（docs/TIMING_PRESETS.md）。",
                 uwb::timingProfileName(TIMING_PROFILE));
    }

#if CONFIG_UWB_ANCHOR_CONSOLE
    /* --- シリアルコンソール（別タスクで動く REPL）を起動する ---
     * 共有状態の初期化に失敗した場合や REPL が立たなかった場合でも、
     * 測距そのものは g_shortAddr のまま継続する（設定変更ができなくなるだけ）。 */
    anchorapp::StaticInfo consoleInfo;
    consoleInfo.boardName   = BOARD_NAME;
    consoleInfo.methodName  = METHOD_NAME;
    consoleInfo.chipName    = uwbDevice.chipName();
    consoleInfo.deviceId    = uwbDevice.deviceId();
    consoleInfo.defaultAddr = ANCHOR_SHORT_ADDR_DEFAULT;
    consoleInfo.tagAddr     = TAG_SHORT_ADDR;
    consoleInfo.panId       = PAN_ID;
    consoleInfo.source      = addrSource;
    if (anchorapp::sharedInit(g_shortAddr, consoleInfo)) {
        const esp_err_t consoleErr = anchorapp::consoleStart();
        // ホスト不在時のログ抑止 (uwb_port.h 参照) / arm the no-host log guard
        uwb_port_console_guard_init();
        if (consoleErr == ESP_OK) {
            ESP_LOGI(TAG, "シリアルコンソールを起動しました（help でコマンド一覧）");
        } else {
            ESP_LOGE(TAG, "シリアルコンソールを起動できませんでした (err=%s)。測距は継続します",
                     esp_err_to_name(consoleErr));
        }
    }
#endif

    /* --- docs/ARCHITECTURE_V2.md §2.1: uwb_radio タスクを起こす ---
     * ResponderConfig はここで一度だけ組み立てる（実行時の addr set が
     * radio タスクへ伝わらない理由はこのファイル冒頭のコメント参照）。
     * static uwb::Responder も uwbDevice と同じ理由で static にする。 */
    static uwb::Responder responder;
    g_radioTaskArgs.radio     = &uwbDevice;
    g_radioTaskArgs.responder = &responder;
    g_radioTaskArgs.cfg       = makeResponderConfig(g_shortAddr);

    const BaseType_t radioTaskOk = xTaskCreatePinnedToCore(
        radioTask, "uwb_radio", CONFIG_UWB_ANCHOR_RADIO_TASK_STACK, &g_radioTaskArgs,
        CONFIG_UWB_ANCHOR_RADIO_TASK_PRIO, &g_radioTaskHandle, CONFIG_UWB_ANCHOR_RADIO_TASK_CORE);
    if (radioTaskOk != pdPASS) {
        ESP_LOGE(TAG, "xTaskCreatePinnedToCore(uwb_radio) failed");
        return;
    }

    ESP_LOGI(TAG,
             "uwb_radio task started (core=%d prio=%d stack=%d), short_addr=0x%04X method=%s idle_tick_ms=%d "
             "restart_on_foreign_poll=%d",
             CONFIG_UWB_ANCHOR_RADIO_TASK_CORE, CONFIG_UWB_ANCHOR_RADIO_TASK_PRIO,
             CONFIG_UWB_ANCHOR_RADIO_TASK_STACK, g_shortAddr, METHOD_NAME, CONFIG_UWB_ANCHOR_IDLE_TICK_MS,
             // 素の CONFIG_UWB_ANCHOR_RESTART_ON_FOREIGN_POLL ではなく、既に
             // 解決済みの g_radioTaskArgs.cfg.restartOnForeignPoll を使う
             // （makeResponderConfig()のコメント参照:
             // n選択時はマクロ自体が未定義になるため、式の中で直接使えない）。
             (int)g_radioTaskArgs.cfg.restartOnForeignPoll);

    /* --- uwb_net（Wi-Fi + ブラウザダッシュボード + 無線コンソール）を起動する ---
     * uwb_radio タスク起動後・main タスクのループに入る前に呼ぶ
     * （scratchpad/NET_SPEC.md §8）。失敗してもUSBシリアルでの運用・測距
     * そのものは継続するので、ログだけ出して先へ進む。 */
    {
        uwb::net::Config netCfg;
        netCfg.role                    = uwb::net::Role::Anchor;
        netCfg.name                     = g_netName;
        netCfg.addr                     = g_netAddr;
        netCfg.aggregate                = false; // アンカーは他機からのUDPを集約しない
        netCfg.forward                   = true;  // 自分の行をUDPブロードキャストする
        netCfg.httpPort                 = static_cast<uint16_t>(CONFIG_UWB_NET_HTTP_PORT);
        netCfg.consolePort              = static_cast<uint16_t>(CONFIG_UWB_NET_CONSOLE_PORT);
        netCfg.udpPort                   = static_cast<uint16_t>(CONFIG_UWB_NET_UDP_PORT);
        netCfg.highRateMinIntervalMs = static_cast<uint32_t>(CONFIG_UWB_NET_HIGHRATE_MIN_INTERVAL_MS);
        netCfg.statusFn                 = &anchorNetStatus;
        netCfg.statusUser               = nullptr;
        const esp_err_t netErr = uwb::net::start(netCfg);
        if (netErr != ESP_OK) {
            ESP_LOGE(TAG, "uwb_net の起動に失敗しました (err=%s)。JSON出力・測距は継続します",
                     esp_err_to_name(netErr));
        }
    }

    /* ==================================================================== *
     * main タスク（このタスク自身、コア0）のループ (docs/ARCHITECTURE_V2.md
     * §2.1/§2.5): 統計行・イベント行・スタック監視。LEDは起動時に始めた
     * ハートビートタスクが背景で回り続けるので、ここで追加の駆動は不要
     * （uwb_status_led_start_role_heartbeat()のコメント参照）。
     * ==================================================================== */
    char addrId[8];
    shortAddrToId(g_shortAddr, 'A', addrId, sizeof(addrId));

    const int64_t bootUs   = esp_timer_get_time();
    int64_t lastStatsUs     = bootUs;
    bool stackLogged         = false;

    // 8件キューを取りこぼさずに済む間隔（§2.2の実測上限90Hz≒11ms/交換に
    // 対し8件分=88msの余裕がある）で events を排出する。stats行の間隔とは
    // 独立（UWB_ANCHOR_STATS_INTERVAL_MSはJSON行の頻度、こちらはキュー溢れ
    // 防止のための内部ポーリング周期で、Kconfig化するほどの調整対象では
    // ない）。
    constexpr TickType_t kMainLoopPeriod = pdMS_TO_TICKS(10);

    while (true) {
        vTaskDelay(kMainLoopPeriod);

        uwb::RangeEvent ev;
        while (responder.popEvent(ev)) {
#if CONFIG_UWB_ANCHOR_LOG_EVENTS
            printRangeEventLine(ev, bootUs, addrId);
#else
            (void)ev;
#endif
        }

        const int64_t nowUs = esp_timer_get_time();
        if ((nowUs - lastStatsUs) >= static_cast<int64_t>(CONFIG_UWB_ANCHOR_STATS_INTERVAL_MS) * 1000) {
            const uwb::ResponderStats snap = responder.snapshot();
            printAnchorStatsLine(snap, static_cast<double>(nowUs - bootUs) / 1e6, addrId);
            publishConsoleStats(snap);
            lastStatsUs = nowUs;
        }

        // 【スタック監視】起動5秒後に1回、uwb_radio タスクのスタック
        // ハイウォーターマーク（未使用の最小残量）を出す。単位はバイト
        // （ESP-IDF の uxTaskGetStackHighWaterMark() は vanilla FreeRTOS と
        // 違いワードではなく**バイト**を返す）。
        // UWB_ANCHOR_RADIO_TASK_STACK の既定値が妥当かをここで確認する。
        if (!stackLogged && (nowUs - bootUs) >= 5 * 1000 * 1000) {
            stackLogged = true;
            ESP_LOGI(TAG, "uwb_radio task stack high-water mark: %u bytes free",
                     (unsigned)uxTaskGetStackHighWaterMark(g_radioTaskHandle));
        }
    }
}
