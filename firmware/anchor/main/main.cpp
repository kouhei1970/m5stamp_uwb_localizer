/**
 * @file main.cpp
 * @brief Phase 4 Step 2 production anchor (responder) firmware for
 * components/uwb_qm33120 (uwb::Qm33120::respondRange() / respondDSRange())。
 *
 * firmware/twr はロール(TAG/ANCHOR)・方式(SS/DS)を両方Kconfigで切り替えられる
 * 評価用ファームだが、本ファームは常にANCHOR(レスポンダ)としてのみ動作する
 * 複数アンカー運用向けの正式版で、firmware/twr と以下の点が異なる。
 *   - ロール選択自体が無い(常にANCHOR)。
 *   - 自分のショートアドレスをKconfig(UWB_ANCHOR_SHORT_ADDR)で個体ごとに
 *     設定できる(firmware/twr は ANCHOR_SHORT_ADDR がソース上の固定値
 *     0x0002 だった)。手持ちのM5Stamp UWB Module 5台それぞれに異なる値を書き込んで
 *     区別する運用を想定している。
 *
 * Kconfig で選ぶ2軸 (main/Kconfig.projbuild):
 *   - ボード: M5StampS3A / M5 AtomS3        (boards ディレクトリ配下の
 *     stamps3.h / atoms3.h でピン定義切替。firmware/twr と同じ作法)
 *     既定は M5StampS3A。標準構成は M5StampS3A + StampS3 BreakOut に
 *     UWB モジュールを 0.5mm 12P FPC→DIP 変換基板で接続するもので、
 *     AtomS3 は「手元にあるなら使える代替」の位置づけ (docs/WIRING.md)。
 *   - 方式  : SS-TWR / DS-TWR (既定はDS-TWR。本プロジェクトの本番運用は
 *     DS-TWRを優先する判断のため)
 *
 * 動作ロジックは firmware/twr/main/main.cpp の ANCHOR+SS-TWR /
 * ANCHOR+DS-TWR ブロックをそのまま踏襲する: 毎ループ
 * respond{Range,DSRange}() をブロッキング呼び出しし、RxTimeout(まだPollが
 * 来ていないだけ)は無視、ANCHOR_LOG_INTERVAL 回成功するごとに統計をログに
 * 出す。DS-TWR は自分で距離を計算する(respondDSRange()の戻り値)ので、
 * こちらでも平均/標準偏差を出す。firmware/twr のANCHOR側ログには無かった
 * 成功率(%)は本ファームで追加した。
 *
 * ショートアドレスは **NVS に保存された値を優先し、無ければ Kconfig の
 * UWB_ANCHOR_SHORT_ADDR を既定値として使う**(components/uwb_cfgstore)。
 * NVS が空・未初期化・破損のいずれでも必ず既定値へフォールバックするので、
 * 購入者が何も設定しなければ従来どおり Kconfig の値でそのまま動く。
 * 実行時の変更は USB-Serial/JTAG 上のシリアルコンソール(anchor_console.cpp、
 * CONFIG_UWB_ANCHOR_CONSOLE で無効化可)から addr set / save で行う。
 * これにより「5台に別アドレスを焼くために5回ビルドし直す」必要が無くなる。
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
#include "uwb_qm33120_distance_stats.hpp" // uwb::DistanceStats (docs/ARCHITECTURE_V2.md §2.3, moved here from a local copy - see below)
#include "uwb_qm33120_phy_kconfig.hpp" // docs/ARCHITECTURE_V2.md §4: phyConfigFromKconfig()

#if CONFIG_UWB_ANCHOR_CONSOLE
#include "anchor_console.hpp"
#endif

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
// タグ側(initiator)のショートアドレス。firmware/tag(別タスクで作成予定)の
// 実装が固まるまでは、firmware/twr と同じくここでは固定値として扱う。
static constexpr uint16_t TAG_SHORT_ADDR = 0x0001;
// このアンカー自身のショートアドレスの **既定値**。Kconfigの
// UWB_ANCHOR_SHORT_ADDR (main/Kconfig.projbuild) は
// 「NVSが空のときの初期値」という位置づけに変わった。実運用では
// シリアルコンソールの addr set / save で個体ごとに設定する。
static constexpr uint16_t ANCHOR_SHORT_ADDR_DEFAULT = CONFIG_UWB_ANCHOR_SHORT_ADDR;

// 起動時に確定するショートアドレス。NVSに保存があればその値、無ければ
// ANCHOR_SHORT_ADDR_DEFAULT。コンソール無効時はこの値が最後まで使われる。
static uint16_t g_shortAddr = ANCHOR_SHORT_ADDR_DEFAULT;

/**
 * @brief いま有効なショートアドレスを返す。測距ループが1回ごとに呼ぶ。
 *
 * コンソール有効時は共有状態(ミューテックス保護)から読むので、addr set の
 * 結果が次の応答から反映される。無効時は起動時に確定した値を返すだけで、
 * 従来と同じく分岐も同期も入らない。
 */
static inline uint16_t currentShortAddr()
{
#if CONFIG_UWB_ANCHOR_CONSOLE
    return anchorapp::currentShortAddr();
#else
    return g_shortAddr;
#endif
}

/**
 * @brief 距離サンプル(mm)の平均・標準偏差をオンライン計算する
 * (Welfordのアルゴリズム)。全サンプルを保持せずに済むので長時間の実機運用に使える。
 *
 * 【docs/ARCHITECTURE_V2.md §2.3】以前はここにローカル定義（firmware/twr/main/
 * main.cpp の DistanceStats と同一実装）を持っていたが、新設の
 * uwb::Responder（components/uwb_qm33120/include/uwb_qm33120_responder_fsm.hpp
 * の ResponderStats::distance）も同じ実装を必要としたため、
 * components/uwb_qm33120/include/uwb_qm33120_distance_stats.hpp（ESP-IDF非依存）
 * へ移し、ここではそちらの型を使う（呼び出し側のコード
 * DistanceStats stats; stats.add(...); stats.mean; stats.stddev(); は無改造）。
 * firmware/twr は引き続き自前のローカル定義を持つ（未変更、単独ビルド可能）。
 */
using DistanceStats = uwb::DistanceStats;

/**
 * @brief 測距統計をコンソール(info コマンド)へ公開する。
 *
 * コンソール無効時は空関数になり、呼び出しは最適化で消える(従来と同じ挙動)。
 * SS-TWR ではAnchor側で距離を計算しないので、距離統計は空のまま渡す。
 */
static inline void publishStats(uint32_t okCount, uint32_t failCount, const DistanceStats& dist)
{
#if CONFIG_UWB_ANCHOR_CONSOLE
    anchorapp::Stats s;
    s.ok      = okCount;
    s.fail    = failCount;
    s.samples = dist.count;
    s.meanMm  = dist.mean;
    s.stdMm   = dist.stddev();
    anchorapp::publishStats(s);
#else
    (void)okCount;
    (void)failCount;
    (void)dist;
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

/* runRole() は選択された方式(SS/DS)だけをビルドする。両方を常に定義すると、
 * 選択されなかった側が -Wunused-function の対象になる(firmware/twr と同じ理由)。 */

#if CONFIG_UWB_ANCHOR_METHOD_DS
/* =========================================================================
 * DS-TWR ANCHOR : firmware/twr の ANCHOR+DS-TWR ブロック
 * (examples/DS_TWR_ANCHOR/DS_TWR_ANCHOR.ino 準拠)を踏襲。
 * ========================================================================= */

static constexpr uint32_t ANCHOR_LOG_INTERVAL                = 20;
static constexpr uint32_t RX_TIMEOUT_UUS                     = 3000;
// 2026-08-29 DS-TWR原因特定 (docs/HANDOFF.md §0-C): 10->20。
// components/uwb_qm33120 の DSRangeConfig::hostTimeoutMs フィールド
// コメント参照（respondDSRange()のFinal待ちがハードウェアRXFTOより先に
// 切れないための余裕）。firmware/twr の ANCHOR+DS-TWR ブロックと同じ変更。
static constexpr uint32_t RANGE_HOST_TIMEOUT_MS               = 20;
static constexpr uint32_t RESPONSE_RX_AFTER_TX_DLY_UUS        = 1500;
static constexpr uint32_t RESPONSE_TX_DLY_UUS                 = 3000;
// 2026-08-29 DS-TWR原因特定 (docs/HANDOFF.md §0-C(2)): 1800->3000。
// Response側と対称にし、850kbps/preamble256でのDW3000 UM §9.4.1エラッタ
// を避ける。DSRangeConfig::finalTxDelayUus のフィールドコメント参照。
static constexpr uint32_t FINAL_TX_DLY_UUS                    = 3000;
// 2026-08-29 DS-TWR原因特定: 500->1500。上のfinalTxDelayUusと対称に
// (DSRangeConfig::finalRxAfterResponseTxDelayUus のフィールドコメント参照)。
static constexpr uint32_t FINAL_RX_AFTER_RESPONSE_TX_DLY_UUS  = 1500;
static constexpr uint32_t RESULT_RX_AFTER_FINAL_TX_DLY_UUS    = 200;
static constexpr uint8_t RESULT_REPEAT_COUNT                  = 1;
static constexpr uint32_t RESULT_REPEAT_GAP_MS                = 3;

static uwb::DSRangeConfig makeRangeConfig(uint16_t selfAddr)
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
    // このDS-TWRブロックの個別値はPollingBothの表(docs/TIMING_PRESETS.md §2.2)
    // と完全に一致しているため、実効プリセットがPollingBoth(明示選択時、
    // またはIRQ線が死んでいてinit()が実行時に降格した場合)のときはこの1行は
    // 実質no-op。Kconfigの既定はBothIrqであり、その場合は値が変わる。
    uwb::applyTimingProfile(range, g_effectiveTimingProfile);
    return range;
}

static void runRole(uwb::Qm33120& uwb)
{
    uint32_t respCount = 0, failCount = 0;
    // DS-TWR は Anchor 側が距離を計算する側 (respondDSRange()) なので、
    // 運用中の平均/標準偏差はこちら側でも取れる。
    DistanceStats stats;

    while (1) {
        // ショートアドレスは1回ごとに読み直す。コンソールから addr set された
        // 場合、進行中の1回のTWRには影響させず、次の応答から新しい値になる。
        const uwb::DSResponderResult result = uwb.respondDSRange(makeRangeConfig(currentShortAddr()));
        if (!result.success) {
            if (result.error == uwb::Error::RxTimeout) {
                // まだPollが来ていないだけ。原本のANCHOR例と同じく無視して再ループ。
                continue;
            }
            failCount++;
            publishStats(respCount, failCount, stats);
            if ((failCount % ANCHOR_LOG_INTERVAL) == 0) {
                const uint32_t total = respCount + failCount;
                const float rate =
                    (total == 0) ? 0.0f : (100.0f * static_cast<float>(respCount) / static_cast<float>(total));
                ESP_LOGW(TAG, "DS_RESP_STAT ok=%lu fail=%lu rate=%.1f%% last=FAIL error=%s",
                         (unsigned long)respCount, (unsigned long)failCount, rate, uwb.lastErrorName());
            }
            continue;
        }

        respCount++;
        stats.add(result.distanceM * 1000.0f);
        publishStats(respCount, failCount, stats);
        if ((respCount % ANCHOR_LOG_INTERVAL) != 0) {
            continue;
        }
        const uint32_t total = respCount + failCount;
        const float rate =
            (total == 0) ? 0.0f : (100.0f * static_cast<float>(respCount) / static_cast<float>(total));
        ESP_LOGI(TAG,
                 "DS_RESP_STAT ok=%lu fail=%lu rate=%.1f%% last=OK seq=%u requester=0x%04X distance_mm=%ld "
                 "distance_m=%.3f mean_mm=%.1f std_mm=%.1f n=%lu elapsed_ms=%lu",
                 (unsigned long)respCount, (unsigned long)failCount, rate, result.sequence, result.requester,
                 (long)result.distanceMm, result.distanceM, stats.mean, stats.stddev(), (unsigned long)stats.count,
                 (unsigned long)result.elapsedMs);
    }
}

#else
/* =========================================================================
 * SS-TWR ANCHOR : firmware/twr の ANCHOR+SS-TWR ブロック
 * (examples/SS_TWR_ANCHOR/SS_TWR_ANCHOR.ino 準拠)を踏襲。
 * ========================================================================= */

static constexpr uint32_t ANCHOR_LOG_INTERVAL          = 20;
static constexpr uint32_t RX_TIMEOUT_UUS               = 3000;
static constexpr uint32_t RANGE_HOST_TIMEOUT_MS         = 10;
static constexpr uint32_t RESPONSE_RX_AFTER_TX_DLY_UUS  = 1500;
static constexpr uint32_t RESPONSE_TX_DLY_UUS           = 3000;

static uwb::RangeConfig makeRangeConfig(uint16_t selfAddr)
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
    // **上の RESPONSE_* / RX_TIMEOUT_UUS はプリセットに上書きされるので
    // 実効値ではない。** 遅延値の唯一の出所は docs/TIMING_PRESETS.md §2。
    //
    // 【この経路では挙動は変わらない】RESPONSE_RX_AFTER_TX_DLY_UUS(1500) と
    // RX_TIMEOUT_UUS(3000) は RangeConfig の既定値(500 / 4500 = PollingBoth)
    // と食い違っている(DS-TWR側の値を誤って流用したとみられる既存の不整合)
    // が、**本ファームが呼ぶ respondRange() はこの2つを読まず
    // dwt_setrxaftertxdelay(0) / dwt_setrxtimeout(0) を直書きしている**
    // (components/uwb_qm33120/src/uwb_qm33120_twr.cpp の respondRange())。
    // したがってアンカー役ではもともと無効な値であり、上書きしても電波の
    // 振る舞いは変わらない。値の出所を1本化するためにこの呼び出しを置く。
    uwb::applyTimingProfile(range, g_effectiveTimingProfile);
    return range;
}

static void runRole(uwb::Qm33120& uwb)
{
    uint32_t respCount = 0, failCount = 0;

    while (1) {
        // ショートアドレスは1回ごとに読み直す（DS-TWR側と同じ理由）。
        const uwb::ResponderResult result = uwb.respondRange(makeRangeConfig(currentShortAddr()));
        if (!result.success) {
            if (result.error == uwb::Error::RxTimeout) {
                // まだPollが来ていないだけ。原本のANCHOR例と同じく無視して再ループ。
                continue;
            }
            failCount++;
            publishStats(respCount, failCount, DistanceStats{});
            if ((failCount % ANCHOR_LOG_INTERVAL) == 0) {
                const uint32_t total = respCount + failCount;
                const float rate =
                    (total == 0) ? 0.0f : (100.0f * static_cast<float>(respCount) / static_cast<float>(total));
                ESP_LOGW(TAG, "SS_RESP_STAT ok=%lu fail=%lu rate=%.1f%% last=FAIL error=%s",
                         (unsigned long)respCount, (unsigned long)failCount, rate, uwb.lastErrorName());
            }
            continue;
        }

        respCount++;
        publishStats(respCount, failCount, DistanceStats{});
        if ((respCount % ANCHOR_LOG_INTERVAL) != 0) {
            continue;
        }
        const uint32_t total = respCount + failCount;
        const float rate =
            (total == 0) ? 0.0f : (100.0f * static_cast<float>(respCount) / static_cast<float>(total));
        ESP_LOGI(TAG, "SS_RESP_STAT ok=%lu fail=%lu rate=%.1f%% last=OK seq=%u requester=0x%04X elapsed_ms=%lu",
                 (unsigned long)respCount, (unsigned long)failCount, rate, result.sequence, result.requester,
                 (unsigned long)result.elapsedMs);
    }
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

    ESP_LOGI(TAG, "Phase 4 Step 2 uwb_qm33120 production anchor firmware, board=%s method=%s short_addr=0x%04X (%s)",
             BOARD_NAME, METHOD_NAME, g_shortAddr, uwb::configSourceName(addrSource));

    uwb::Qm33120 uwbDevice;
    const uwb::Config cfg = makeConfigFromBoard();
    // docs/ARCHITECTURE_V2.md §4: PHY を共通 Kconfig (UWB_PHY_*,
    // components/uwb_qm33120/Kconfig) から選ぶ。firmware/tag はまだこの関数を
    // 使っておらず uwb::PhyConfig の構造体既定のまま（別エージェント作業中）。
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

#if defined(CONFIG_UWB_PHY_PLL_COARSE_CH9) && (CONFIG_UWB_PHY_PLL_COARSE_CH9 != 0)
    // docs/ARCHITECTURE_V2.md §4: firmware/twr の DIAG_PLL_COARSE_CH9 の
    // 本番機版。非ゼロなら begin() 直後に ch9 の PLL 粗調整コードを強制する。
    {
        const bool pllOk = uwbDevice.forcePllCoarse(static_cast<uint8_t>(CONFIG_UWB_PHY_PLL_COARSE_CH9));
        if (!pllOk) {
            ESP_LOGW(TAG,
                     "forcePllCoarse(0x%02X) could not lock the forced code; radio fell back to its normal "
                     "calibration path (see the DIAG_PLL_COARSE(*) lines above)",
                     (unsigned)CONFIG_UWB_PHY_PLL_COARSE_CH9);
        }
    }
#endif

    // init() may have downgraded the requested TIMING_PROFILE to PollingBoth
    // if the IRQ line turned out to be dead (Qm33120::verifyIrqLine()). Carry
    // the profile that was actually applied forward from here on.
    // IRQ 線が死んでいると init() が要求プリセットを PollingBoth へ降格させて
    // いることがあるため、以降は「実際に適用されたプリセット」を使う。
    g_effectiveTimingProfile = uwbDevice.config().timing_profile;

    ESP_LOGI(TAG, "begin() + PHY config OK, starting ANCHOR/%s loop (short_addr=0x%04X)", METHOD_NAME,
             g_shortAddr);

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
        if (consoleErr == ESP_OK) {
            ESP_LOGI(TAG, "シリアルコンソールを起動しました（help でコマンド一覧）");
        } else {
            ESP_LOGE(TAG, "シリアルコンソールを起動できませんでした (err=%s)。測距は継続します",
                     esp_err_to_name(consoleErr));
        }
    }
#endif

    runRole(uwbDevice);
}
