/**
 * @file main.cpp
 * @brief Phase 2 Step 2 acceptance/evaluation firmware for
 * components/uwb_qm33120's TWR methods (uwb::Qm33120::requestRange() /
 * respondRange() / requestDSRange() / respondDSRange()).
 *
 * Kconfig で選ぶ3軸 (main/Kconfig.projbuild):
 *   - ボード: M5StampS3A / M5 AtomS3        (boards ディレクトリ配下の
 *     stamps3.h / atoms3.h でピン定義切替。firmware/probe, firmware/devtest
 *     と同じ作法)
 *   - ロール: TAG(initiator) / ANCHOR(responder)
 *   - 方式  : SS-TWR / DS-TWR
 *
 * 各組み合わせの処理は third_party/M5Stamp-UWB/examples 配下の
 * {SS,DS}_TWR_{TAG,ANCHOR} 各 .ino に準拠（レンジング設定の数値もそちらの値を
 * そのまま使用。uwb::RangeConfig/
 * uwb::DSRangeConfig の構造体既定値とは一部異なる - 例えば
 * responseRxAfterTxDelayUus は構造体既定 500 に対し examples は 1500 を明示指定
 * している。ここでは examples に合わせる）。
 *
 * TAG: 200ms間隔で1回レンジングし、10回に1回、成功率と距離の平均/標準偏差
 * （Welfordのオンラインアルゴリズムで起動からの累積を計算）をログに出す。
 * ANCHOR: 毎ループ respond{Range,DSRange}() をブロッキング呼び出しし、
 * RxTimeout（まだPollが来ていないだけ）は無視、20回成功するごとにログを出す。
 * DS-TWR の ANCHOR は自分で距離を計算する（respondDSRange()の戻り値）ので、
 * こちらでも同じ統計を出す。
 *
 * 【docs/archive/REIMPL_PLAN.md R3-1/R9】上記の「examplesの値をそのまま使用」の
 * 例外として、RANGE_HOST_TIMEOUT_MS(100→10)/RESULT_RX_AFTER_FINAL_TX_DLY_UUS
 * (500→200)/RESULT_REPEAT_COUNT(3→1) の3つは
 * uwb::RangeConfig/DSRangeConfig 側の新しい既定値
 * （components/uwb_qm33120/include/uwb_qm33120_types.hpp）に揃えて更新した
 * （examplesの値は検証されていない二次資料であり、この3つは特に
 * docs/archive/CRITICAL_REVIEW.md【重大3】で問題が指摘されていたため）。
 */
#include <cmath>
#include <cstdio>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "deca_device_api.h"
#include "uwb_port.h"
#include "uwb_qm33120.hpp"

#if CONFIG_UWB_TWR_BOARD_ATOMS3
#include "boards/atoms3.h"
#define BOARD_UWB_PORT_CONFIG BOARD_ATOMS3_UWB_PORT_CONFIG
#define BOARD_NAME            "AtomS3(pinout " BOARD_ATOMS3_PINOUT_NAME ")"
#elif CONFIG_UWB_TWR_BOARD_STAMPFLY
#include "boards/stampfly.h"
#define BOARD_UWB_PORT_CONFIG BOARD_STAMPFLY_UWB_PORT_CONFIG
#define BOARD_NAME            "StampFly"
#else
#include "boards/stamps3.h"
#define BOARD_UWB_PORT_CONFIG BOARD_STAMPS3_UWB_PORT_CONFIG
#define BOARD_NAME            "M5StampS3A"
#endif

#if CONFIG_UWB_TWR_ROLE_ANCHOR
#define ROLE_NAME "ANCHOR"
#else
#define ROLE_NAME "TAG"
#endif

#if CONFIG_UWB_TWR_METHOD_DS
#define METHOD_NAME "DS-TWR"
#else
#define METHOD_NAME "SS-TWR"
#endif

static const char* TAG = "uwb_twr";

/* ---------------------------------------------------------------------------
 * Diagnostics for the SS-TWR success rate seen on real hardware
 * (2026-08-28: 0-9% over 60cm..1.7m, independent of distance, orientation,
 * which board plays which role, and of re-running the PHY configuration).
 *
 * Logs the die temperature, the crystal trim and the measured clock offset,
 * and - when CONFIG_UWB_TWR_DIAG_REINIT_FAILS > 0 - re-runs the PHY
 * configuration (and with it the PLL calibration) after N consecutive
 * failures. That re-run was the test for task R10 (docs/GETTING_STARTED.md:
 * 1380): on channel 9, the only channel this module supports, the PLL must
 * be recalibrated whenever the die temperature moves by 20 degC (user manual
 * 10.4), which this repo does not do.
 *
 * MEASURED RESULT (2026-08-28): the die temperature is stable in time
 * (tag 34.5-35.5 degC, anchor 51-57 degC), and 13 re-runs of init() did not
 * restore the link. So a *drift over time* of one chip's PLL is NOT the
 * cause. What remains unexplained is the ~17-21 degC standing difference
 * between the two chips (the anchor self-heats in continuous RX).
 *
 * 実機の SS-TWR 成功率（2026-08-28: 60cm〜1.7m で 0〜9%。距離・向き・役割の
 * 割当て・PHY 再設定のいずれとも無関係）を切り分けるための診断。
 * ダイ温度・水晶トリム・クロックオフセットをログに出し、
 * CONFIG_UWB_TWR_DIAG_REINIT_FAILS > 0 のときは連続 N 回失敗で PHY 設定を
 * やり直す（＝課題 R10 の検証。ch9 は 20°C 変化で PLL 再校正が要る）。
 * 実測結果（2026-08-28）: ダイ温度は時間的に安定（タグ 34.5〜35.5°C、
 * アンカー 51〜57°C）で、init() を 13 回やり直しても復帰しなかった。
 * よって「片方の PLL が時間とともにずれる」ことは原因ではない。
 * 未解明なのは 2 個体間に定常的にある 17〜21°C の温度差のほう
 * （アンカーは連続受信で自己発熱する）。
 * ------------------------------------------------------------------------- */
static constexpr uint32_t DIAG_REINIT_AFTER_FAILS = CONFIG_UWB_TWR_DIAG_REINIT_FAILS;

/** @brief Read the on-chip die temperature [degC] / ダイ温度を読む */
static float dieTempC()
{
    const uint16_t raw = dwt_readtempvbat();
    return dwt_convertrawtemperature(static_cast<uint8_t>(raw >> 8));
}

/**
 * @brief Re-run the PHY configuration (and with it the PLL calibration), and
 *        log the die temperature before and after.
 *        PHY 設定（＝PLL 再校正）をやり直し、前後のダイ温度をログに出す。
 */
static void diagReinit(uwb::Qm33120& uwb, uint32_t consecutiveFails)
{
    const float before = dieTempC();
    const bool ok      = uwb.init();
    ESP_LOGW(TAG, "DIAG_REINIT after %lu consecutive failures: init()=%s temp_before=%.1fC temp_after=%.1fC",
             (unsigned long)consecutiveFails, ok ? "OK" : "FAILED", before, dieTempC());
}

#define UWB_DEV_ID_EXPECTED 0xDECA0314UL

/* --- Timing preset (docs/TIMING_PRESETS.md, task D) ---
 * Derived from the Kconfig choice UWB_TIMING_PROFILE. Both boards under
 * test (TAG and ANCHOR role) must select the same preset. */
#if CONFIG_UWB_TIMING_PROFILE_ANCHOR_IRQ
static constexpr uwb::TimingProfile TIMING_PROFILE = uwb::TimingProfile::AnchorIrq;
#elif CONFIG_UWB_TIMING_PROFILE_BOTH_IRQ
static constexpr uwb::TimingProfile TIMING_PROFILE = uwb::TimingProfile::BothIrq;
#else
static constexpr uwb::TimingProfile TIMING_PROFILE = uwb::TimingProfile::PollingBoth;
#endif

/* --- ネットワーク共通パラメータ（4組み合わせ共通。examples 配下の各 .ino と同じ値） --- */
static constexpr uint16_t PAN_ID           = 0xDECA;
// タスクD-1(docs/HANDOFF.md §5): Kconfig化。既定値は元のソース上の固定値
// 0x0001/0x0002のまま変えていない(firmware/anchorのUWB_ANCHOR_SHORT_ADDRと
// 同じ書き方)。
static constexpr uint16_t TAG_SHORT_ADDR    = CONFIG_UWB_TWR_TAG_ADDR;
static constexpr uint16_t ANCHOR_SHORT_ADDR = CONFIG_UWB_TWR_ANCHOR_ADDR;

#if !CONFIG_UWB_TWR_ROLE_ANCHOR
/* TAG側の200ms間隔制御にのみ使う。ANCHOR側は毎ループブロッキング呼び出しで
 * 自前のインターバル管理をしないため、定義したままだと -Wunused-function になる。 */
static uint32_t nowMs()
{
    return static_cast<uint32_t>(esp_timer_get_time() / 1000);
}
#endif

/**
 * @brief 距離サンプル(mm)の平均・標準偏差をオンライン計算する
 * (Welfordのアルゴリズム)。全サンプルを保持せずに済むので長時間の実機評価に使える。
 */
struct DistanceStats {
    uint32_t count = 0;
    double mean    = 0.0;
    double m2      = 0.0;

    void add(float distanceMm)
    {
        count++;
        const double delta = static_cast<double>(distanceMm) - mean;
        mean += delta / static_cast<double>(count);
        const double delta2 = static_cast<double>(distanceMm) - mean;
        m2 += delta * delta2;
    }

    double stddev() const
    {
        return (count < 2) ? 0.0 : std::sqrt(m2 / static_cast<double>(count - 1));
    }
};

/**
 * @brief boards 以下のヘッダの BOARD_UWB_PORT_CONFIG を uwb::Config へコピーする。
 * firmware/devtest/main/main.cpp の makeConfigFromBoard() と同一の変換。
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
    // タスクD-2(docs/HANDOFF.md §5): 既定0のときはboards/*.hの値(16MHz)を
    // 一切上書きせず、この関数を追加する前と完全に同一の挙動にする。
    // Device IDが読めない/たまに化けるときの切り分け用。
#if CONFIG_UWB_SPI_FAST_HZ > 0
    cfg.spi_fast_hz = CONFIG_UWB_SPI_FAST_HZ;
#endif
    cfg.init_spi_bus = port.init_spi_bus;
    // CONFIG_UWB_ENABLE_IRQ is undefined (not just 0) by ESP-IDF's Kconfig
    // when set to n, so #if defined(...) && ... is used instead of #ifdef
    // (docs/IRQ_POLICY.md, main/Kconfig.projbuild). An unwired pin_irq or a
    // failed ISR registration makes Qm33120::init() fall back to polling
    // automatically.
#if defined(CONFIG_UWB_ENABLE_IRQ) && CONFIG_UWB_ENABLE_IRQ
    cfg.use_irq = true;
#else
    cfg.use_irq = false;
#endif
    // Timing preset (docs/TIMING_PRESETS.md, task D). Carried to the peer in
    // the Poll/Response frames and used for mismatch detection
    // (uwb_qm33120_twr.cpp checkTimingTag()/logTimingMismatch()).
    cfg.timing_profile = TIMING_PROFILE;
    return cfg;
}

/* runRole() は選択された ロール×方式 の組み合わせだけをビルドする
 * (4通りの中から1つ)。他の3つを常に定義すると、選択されなかった側が
 * -Wunused-function の対象になる（firmware/devtest と同じ理由）。 */

#if !CONFIG_UWB_TWR_ROLE_ANCHOR && !CONFIG_UWB_TWR_METHOD_DS
/* =========================================================================
 * TAG + SS-TWR : examples/SS_TWR_TAG/SS_TWR_TAG.ino 準拠
 * ========================================================================= */

// Poll interval. Configurable so the tag's period can be made incommensurate
// with the anchor's poll-wait window, which is what a beat between the two
// would need to show up as a change in success rate.
// Poll 周期。アンカーの受信待ち窓と周期をずらせるように Kconfig 化した。
static constexpr uint32_t RANGE_INTERVAL_MS            = CONFIG_UWB_TWR_RANGE_INTERVAL_MS;
static constexpr uint32_t TAG_LOG_INTERVAL              = 10;
static constexpr uint32_t RX_TIMEOUT_UUS                = 3000;
static constexpr uint32_t RANGE_HOST_TIMEOUT_MS          = 10;
static constexpr uint32_t RESPONSE_RX_AFTER_TX_DLY_UUS   = 1500;
static constexpr uint32_t RESPONSE_TX_DLY_UUS            = 3000;

static uwb::RangeConfig makeRangeConfig()
{
    uwb::RangeConfig range;
    range.panId                     = PAN_ID;
    range.initiatorAddress          = TAG_SHORT_ADDR;
    range.responderAddress          = ANCHOR_SHORT_ADDR;
    range.responseRxAfterTxDelayUus = RESPONSE_RX_AFTER_TX_DLY_UUS;
    range.responseTxDelayUus        = RESPONSE_TX_DLY_UUS;
    range.rxTimeoutUus              = RX_TIMEOUT_UUS;
    range.hostTimeoutMs             = RANGE_HOST_TIMEOUT_MS;
    // Apply the timing preset last so it wins over the individual assignments
    // above (*Uus fields only; panId/addresses/hostTimeoutMs are untouched).
    // The RESPONSE_* / RX_TIMEOUT_UUS constants above are therefore NOT the
    // effective values: docs/TIMING_PRESETS.md section 2 is the single source.
    //
    // [Behaviour change] RESPONSE_RX_AFTER_TX_DLY_UUS (1500) and RX_TIMEOUT_UUS
    // (3000) above do not match RangeConfig's real default (500 / 4500 =
    // the PollingBoth preset; uwb_qm33120_types.hpp). They look like values
    // copied from the DS-TWR block by mistake. requestRange() DOES use both
    // (dwt_setrxaftertxdelay / dwt_setrxtimeout, uwb_qm33120_twr.cpp), so the
    // preset really does change this path: (1500, 4500) -> (500, 4500) even
    // under the default Kconfig (PollingBoth). This is safe - the new values
    // open the RX window earlier and keep it open longer, and they satisfy the
    // deadline inequality in docs/TIMING_PRESETS.md section 1.2 with more
    // margin than the old ones.
    uwb::applyTimingProfile(range, TIMING_PROFILE);
    return range;
}

static void runRole(uwb::Qm33120& uwb)
{
    uint32_t lastRangeMs = 0;
    uint32_t rangeCount = 0, rangeOkCount = 0, rangeFailCount = 0;
    uint32_t consecutiveFails = 0;
    DistanceStats stats;

    while (1) {
        if ((nowMs() - lastRangeMs) < RANGE_INTERVAL_MS) {
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }
        lastRangeMs = nowMs();

        const uwb::RangeResult result = uwb.requestRange(makeRangeConfig());
        rangeCount++;
        if (result.success) {
            rangeOkCount++;
            consecutiveFails = 0;
            stats.add(result.distanceM * 1000.0f);
        } else {
            rangeFailCount++;
            consecutiveFails++;
            // Diagnostic only; disabled unless CONFIG_UWB_TWR_DIAG_REINIT_FAILS > 0.
            // 診断用。既定 (0) では何も起きない。
            if (DIAG_REINIT_AFTER_FAILS > 0 && consecutiveFails >= DIAG_REINIT_AFTER_FAILS) {
                diagReinit(uwb, consecutiveFails);
                consecutiveFails = 0;
            }
        }

        if ((rangeCount % TAG_LOG_INTERVAL) != 0) {
            continue;
        }

        const float rate =
            (rangeCount == 0) ? 0.0f : (100.0f * static_cast<float>(rangeOkCount) / static_cast<float>(rangeCount));
        if (result.success) {
            ESP_LOGI(TAG,
                     "SS_RANGE_STAT count=%lu ok=%lu fail=%lu rate=%.1f%% last=OK seq=%u distance_mm=%ld "
                     "distance_m=%.3f mean_mm=%.1f std_mm=%.1f n=%lu elapsed_ms=%lu temp=%.1fC clock_ppm=%.2f",
                     (unsigned long)rangeCount, (unsigned long)rangeOkCount, (unsigned long)rangeFailCount, rate,
                     result.sequence, (long)result.distanceMm, result.distanceM, stats.mean, stats.stddev(),
                     (unsigned long)stats.count, (unsigned long)result.elapsedMs, dieTempC(), result.clockOffsetPpm);
        } else {
            ESP_LOGW(TAG, "SS_RANGE_STAT count=%lu ok=%lu fail=%lu rate=%.1f%% last=FAIL seq=%u error=%s temp=%.1fC",
                     (unsigned long)rangeCount, (unsigned long)rangeOkCount, (unsigned long)rangeFailCount, rate,
                     result.sequence, uwb.lastErrorName(), dieTempC());
        }
    }
}

#elif CONFIG_UWB_TWR_METHOD_DS && !CONFIG_UWB_TWR_ROLE_ANCHOR
/* =========================================================================
 * TAG + DS-TWR : examples/DS_TWR_TAG/DS_TWR_TAG.ino 準拠
 * ========================================================================= */

static constexpr uint32_t RANGE_INTERVAL_MS = 200;
static constexpr uint32_t TAG_LOG_INTERVAL   = 10;

static constexpr uint32_t RX_TIMEOUT_UUS               = 3000;
static constexpr uint32_t RANGE_HOST_TIMEOUT_MS         = 10;
static constexpr uint32_t RESPONSE_RX_AFTER_TX_DLY_UUS  = 1500;
static constexpr uint32_t RESPONSE_TX_DLY_UUS           = 3000;
static constexpr uint32_t FINAL_TX_DLY_UUS                   = 1800;
static constexpr uint32_t FINAL_RX_AFTER_RESPONSE_TX_DLY_UUS = 500;
static constexpr uint32_t RESULT_RX_AFTER_FINAL_TX_DLY_UUS   = 200;
static constexpr uint8_t RESULT_REPEAT_COUNT    = 1;
static constexpr uint32_t RESULT_REPEAT_GAP_MS  = 3;

static uwb::DSRangeConfig makeRangeConfig()
{
    uwb::DSRangeConfig range;
    range.panId                          = PAN_ID;
    range.initiatorAddress               = TAG_SHORT_ADDR;
    range.responderAddress               = ANCHOR_SHORT_ADDR;
    range.responseRxAfterTxDelayUus      = RESPONSE_RX_AFTER_TX_DLY_UUS;
    range.responseTxDelayUus             = RESPONSE_TX_DLY_UUS;
    range.finalTxDelayUus                = FINAL_TX_DLY_UUS;
    range.finalRxAfterResponseTxDelayUus = FINAL_RX_AFTER_RESPONSE_TX_DLY_UUS;
    range.resultRxAfterFinalTxDelayUus   = RESULT_RX_AFTER_FINAL_TX_DLY_UUS;
    range.rxTimeoutUus                   = RX_TIMEOUT_UUS;
    range.hostTimeoutMs                  = RANGE_HOST_TIMEOUT_MS;
    range.resultRepeatCount              = RESULT_REPEAT_COUNT;
    range.resultRepeatGapMs              = RESULT_REPEAT_GAP_MS;
    // Apply the timing preset last so it wins over the individual
    // assignments above (*Uus fields only; panId/addresses/hostTimeoutMs/
    // resultRepeatCount/resultRepeatGapMs are untouched). This DS-TWR
    // block's individual values already match the PollingBoth row of
    // docs/TIMING_PRESETS.md SS2.2 exactly, so under the default Kconfig
    // (PollingBoth) this call is effectively a no-op.
    uwb::applyTimingProfile(range, TIMING_PROFILE);
    return range;
}

static void runRole(uwb::Qm33120& uwb)
{
    uint32_t lastRangeMs = 0;
    uint32_t rangeCount = 0, rangeOkCount = 0, rangeFailCount = 0;
    DistanceStats stats;

    while (1) {
        if ((nowMs() - lastRangeMs) < RANGE_INTERVAL_MS) {
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }
        lastRangeMs = nowMs();

        // 距離は Anchor(respondDSRange) 側で計算済みのものをそのまま受け取るだけ
        // （requestDSRange() は再計算しない。cpp:1152-1153 相当）。
        const uwb::DSRangeResult result = uwb.requestDSRange(makeRangeConfig());
        rangeCount++;
        if (result.success) {
            rangeOkCount++;
            stats.add(result.distanceM * 1000.0f);
        } else {
            rangeFailCount++;
        }

        if ((rangeCount % TAG_LOG_INTERVAL) != 0) {
            continue;
        }

        const float rate =
            (rangeCount == 0) ? 0.0f : (100.0f * static_cast<float>(rangeOkCount) / static_cast<float>(rangeCount));
        if (result.success) {
            ESP_LOGI(TAG,
                     "DS_RANGE_STAT count=%lu ok=%lu fail=%lu rate=%.1f%% last=OK seq=%u distance_mm=%ld "
                     "distance_m=%.3f mean_mm=%.1f std_mm=%.1f n=%lu elapsed_ms=%lu",
                     (unsigned long)rangeCount, (unsigned long)rangeOkCount, (unsigned long)rangeFailCount, rate,
                     result.sequence, (long)result.distanceMm, result.distanceM, stats.mean, stats.stddev(),
                     (unsigned long)stats.count, (unsigned long)result.elapsedMs);
        } else {
            ESP_LOGW(TAG, "DS_RANGE_STAT count=%lu ok=%lu fail=%lu rate=%.1f%% last=FAIL seq=%u error=%s",
                     (unsigned long)rangeCount, (unsigned long)rangeOkCount, (unsigned long)rangeFailCount, rate,
                     result.sequence, uwb.lastErrorName());
        }
    }
}

#elif !CONFIG_UWB_TWR_METHOD_DS && CONFIG_UWB_TWR_ROLE_ANCHOR
/* =========================================================================
 * ANCHOR + SS-TWR : examples/SS_TWR_ANCHOR/SS_TWR_ANCHOR.ino 準拠
 * ========================================================================= */

static constexpr uint32_t ANCHOR_LOG_INTERVAL           = 20;
static constexpr uint32_t RX_TIMEOUT_UUS                = 3000;
static constexpr uint32_t RANGE_HOST_TIMEOUT_MS          = 10;
static constexpr uint32_t RESPONSE_RX_AFTER_TX_DLY_UUS   = 1500;
static constexpr uint32_t RESPONSE_TX_DLY_UUS            = 3000;

static uwb::RangeConfig makeRangeConfig()
{
    uwb::RangeConfig range;
    range.panId                     = PAN_ID;
    range.initiatorAddress          = TAG_SHORT_ADDR;
    range.responderAddress          = ANCHOR_SHORT_ADDR;
    range.responseRxAfterTxDelayUus = RESPONSE_RX_AFTER_TX_DLY_UUS;
    range.responseTxDelayUus        = RESPONSE_TX_DLY_UUS;
    range.rxTimeoutUus              = RX_TIMEOUT_UUS;
    range.hostTimeoutMs             = RANGE_HOST_TIMEOUT_MS;
    // Apply the timing preset last so it wins over the individual assignments
    // above (*Uus fields only; panId/addresses/hostTimeoutMs are untouched).
    // The RESPONSE_* / RX_TIMEOUT_UUS constants above are therefore NOT the
    // effective values: docs/TIMING_PRESETS.md section 2 is the single source.
    //
    // [No behaviour change here] RESPONSE_RX_AFTER_TX_DLY_UUS (1500) and
    // RX_TIMEOUT_UUS (3000) above do not match RangeConfig's real default
    // (500 / 4500 = the PollingBoth preset; uwb_qm33120_types.hpp) - they look
    // like values copied from the DS-TWR block by mistake. But respondRange(),
    // which is what the ANCHOR role calls, never reads these two fields: it
    // hardcodes dwt_setrxaftertxdelay(0) / dwt_setrxtimeout(0)
    // (components/uwb_qm33120/src/uwb_qm33120_twr.cpp, respondRange()).
    // They were already dead here, so overriding them changes nothing on air.
    uwb::applyTimingProfile(range, TIMING_PROFILE);
    return range;
}

static void runRole(uwb::Qm33120& uwb)
{
    uint32_t respCount = 0, failCount = 0;
    uint32_t consecutiveMisses = 0;
    // Diagnostic: how much of the time is this anchor actually listening?
    // respondRange() re-arms the receiver on every call, so any time spent
    // between calls is time the poll cannot be heard. If the tag polls every
    // 200ms and this loop also cycles every ~200ms, the two can beat against
    // each other - which would look exactly like the intermittent success we
    // measured. Log the cycle count and the mean cycle time every 5 seconds.
    // 診断: このアンカーが実際に聞いている時間の割合。respondRange() は毎回
    // 受信機を開き直すので、呼び出しの合間は Poll を聞けない。タグが 200ms
    // 周期で、このループも約 200ms 周期だと両者がうなりを起こしうる。
    // 5 秒ごとに周回数と平均周期を出す。
    uint32_t cycles = 0;
    uint32_t windowStartMs = static_cast<uint32_t>(esp_timer_get_time() / 1000);

    while (1) {
        const uwb::ResponderResult result = uwb.respondRange(makeRangeConfig());
        cycles++;
        {
            const uint32_t nowMsLocal = static_cast<uint32_t>(esp_timer_get_time() / 1000);
            const uint32_t spanMs      = nowMsLocal - windowStartMs;
            if (spanMs >= 5000) {
                ESP_LOGI(TAG, "SS_RESP_CYCLE cycles=%lu in %lums (mean %.1fms/cycle) ok=%lu fail=%lu temp=%.1fC",
                         (unsigned long)cycles, (unsigned long)spanMs,
                         (cycles == 0) ? 0.0 : (double)spanMs / (double)cycles, (unsigned long)respCount,
                         (unsigned long)failCount, dieTempC());
                cycles         = 0;
                windowStartMs  = nowMsLocal;
            }
        }
        if (!result.success) {
            if (result.error == uwb::Error::RxTimeout) {
                // まだPollが来ていないだけ。原本のANCHOR例と同じく無視して再ループ。
                // Diagnostic only; disabled unless CONFIG_UWB_TWR_DIAG_REINIT_FAILS > 0.
                // 診断用。既定 (0) では何も起きない。
                consecutiveMisses++;
                if (DIAG_REINIT_AFTER_FAILS > 0 && consecutiveMisses >= DIAG_REINIT_AFTER_FAILS) {
                    diagReinit(uwb, consecutiveMisses);
                    consecutiveMisses = 0;
                }
                continue;
            }
            failCount++;
            if ((failCount % ANCHOR_LOG_INTERVAL) == 0) {
                ESP_LOGW(TAG, "SS_RESP_STAT ok=%lu fail=%lu last=FAIL error=%s", (unsigned long)respCount,
                         (unsigned long)failCount, uwb.lastErrorName());
            }
            continue;
        }

        respCount++;
        consecutiveMisses = 0;
        if ((respCount % ANCHOR_LOG_INTERVAL) != 0) {
            continue;
        }
        ESP_LOGI(TAG, "SS_RESP_STAT ok=%lu fail=%lu last=OK seq=%u requester=0x%04X elapsed_ms=%lu temp=%.1fC",
                 (unsigned long)respCount, (unsigned long)failCount, result.sequence, result.requester,
                 (unsigned long)result.elapsedMs, dieTempC());
    }
}

#else
/* =========================================================================
 * ANCHOR + DS-TWR : examples/DS_TWR_ANCHOR/DS_TWR_ANCHOR.ino 準拠
 * ========================================================================= */

static constexpr uint32_t ANCHOR_LOG_INTERVAL          = 20;
static constexpr uint32_t RX_TIMEOUT_UUS               = 3000;
static constexpr uint32_t RANGE_HOST_TIMEOUT_MS         = 10;
static constexpr uint32_t RESPONSE_RX_AFTER_TX_DLY_UUS  = 1500;
static constexpr uint32_t RESPONSE_TX_DLY_UUS           = 3000;
static constexpr uint32_t FINAL_TX_DLY_UUS                   = 1800;
static constexpr uint32_t FINAL_RX_AFTER_RESPONSE_TX_DLY_UUS = 500;
static constexpr uint32_t RESULT_RX_AFTER_FINAL_TX_DLY_UUS   = 200;
static constexpr uint8_t RESULT_REPEAT_COUNT    = 1;
static constexpr uint32_t RESULT_REPEAT_GAP_MS  = 3;

static uwb::DSRangeConfig makeRangeConfig()
{
    uwb::DSRangeConfig range;
    range.panId                          = PAN_ID;
    range.initiatorAddress               = TAG_SHORT_ADDR;
    range.responderAddress               = ANCHOR_SHORT_ADDR;
    range.responseRxAfterTxDelayUus      = RESPONSE_RX_AFTER_TX_DLY_UUS;
    range.responseTxDelayUus             = RESPONSE_TX_DLY_UUS;
    range.finalTxDelayUus                = FINAL_TX_DLY_UUS;
    range.finalRxAfterResponseTxDelayUus = FINAL_RX_AFTER_RESPONSE_TX_DLY_UUS;
    range.resultRxAfterFinalTxDelayUus   = RESULT_RX_AFTER_FINAL_TX_DLY_UUS;
    range.rxTimeoutUus                   = RX_TIMEOUT_UUS;
    range.hostTimeoutMs                  = RANGE_HOST_TIMEOUT_MS;
    range.resultRepeatCount              = RESULT_REPEAT_COUNT;
    range.resultRepeatGapMs              = RESULT_REPEAT_GAP_MS;
    // Apply the timing preset last so it wins over the individual
    // assignments above (*Uus fields only; panId/addresses/hostTimeoutMs/
    // resultRepeatCount/resultRepeatGapMs are untouched). This DS-TWR
    // block's individual values already match the PollingBoth row of
    // docs/TIMING_PRESETS.md SS2.2 exactly, so under the default Kconfig
    // (PollingBoth) this call is effectively a no-op.
    uwb::applyTimingProfile(range, TIMING_PROFILE);
    return range;
}

static void runRole(uwb::Qm33120& uwb)
{
    uint32_t respCount = 0, failCount = 0;
    // DS-TWR は Anchor 側が距離を計算する側 (respondDSRange() cpp:1294-1319) なので、
    // 実機評価用の平均/標準偏差はこちら側でも取れる。
    DistanceStats stats;

    while (1) {
        const uwb::DSResponderResult result = uwb.respondDSRange(makeRangeConfig());
        if (!result.success) {
            if (result.error == uwb::Error::RxTimeout) {
                // まだPollが来ていないだけ。原本のANCHOR例と同じく無視して再ループ。
                continue;
            }
            failCount++;
            if ((failCount % ANCHOR_LOG_INTERVAL) == 0) {
                ESP_LOGW(TAG, "DS_RESP_STAT ok=%lu fail=%lu last=FAIL error=%s", (unsigned long)respCount,
                         (unsigned long)failCount, uwb.lastErrorName());
            }
            continue;
        }

        respCount++;
        stats.add(result.distanceM * 1000.0f);
        if ((respCount % ANCHOR_LOG_INTERVAL) != 0) {
            continue;
        }
        ESP_LOGI(TAG,
                 "DS_RESP_STAT ok=%lu fail=%lu last=OK seq=%u requester=0x%04X distance_mm=%ld distance_m=%.3f "
                 "mean_mm=%.1f std_mm=%.1f n=%lu elapsed_ms=%lu",
                 (unsigned long)respCount, (unsigned long)failCount, result.sequence, result.requester,
                 (long)result.distanceMm, result.distanceM, stats.mean, stats.stddev(), (unsigned long)stats.count,
                 (unsigned long)result.elapsedMs);
    }
}

#endif

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Phase 2 Step 2 uwb_qm33120 TWR firmware, board=%s role=%s method=%s", BOARD_NAME, ROLE_NAME,
             METHOD_NAME);

    uwb::Qm33120 uwbDevice;
    const uwb::Config cfg = makeConfigFromBoard();
    uwb::PhyConfig phy; // defaults: ch9, preamble128, PAC8, 6.8Mbps (see uwb_qm33120_types.hpp)
#if defined(CONFIG_UWB_TWR_DIAG_ROBUST_PHY) && CONFIG_UWB_TWR_DIAG_ROBUST_PHY
    // Diagnostic: trade update rate for link margin. The default preamble of
    // 128 symbols is the shortest the chip offers, so preamble detection has
    // the least margin; 1024 symbols with PAC32 and 850 kbps is the most
    // robust combination. Both boards of a link must use the same settings.
    // sfdTimeout stays 0 so it is recomputed for the new preamble length
    // (task R8 - a fixed sfdTimeout silently destroys the reception rate).
    // 診断用: 更新レートを犠牲にして受信の余裕を増やす。既定のプリアンブル
    // 128 シンボルはチップが出せる最短で、検出の余裕が最も小さい。
    // 1024 + PAC32 + 850kbps が最も頑健な組み合わせ。両機を同じ設定にすること。
    phy.preambleLength = uwb::PreambleLength::Len1024;
    phy.pacSize        = uwb::PacSize::Pac32;
    phy.dataRate       = uwb::DataRate::Rate850K;
    ESP_LOGW(TAG, "DIAG_ROBUST_PHY: preamble=1024 PAC=32 rate=850kbps (both boards must match)");
#endif

    // タスクD-2: 実際に使うSPI高速クロックを起動ログに出す(切り分け作業で
    // Kconfigの値が本当に反映されたかを確認できるように)。
    ESP_LOGI(TAG, "spi_fast=%lu", (unsigned long)cfg.spi_fast_hz);

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
    ESP_LOGI(TAG, "begin() + PHY config OK, starting %s/%s loop", ROLE_NAME, METHOD_NAME);

    // --- Log whether IRQ actually ended up active (docs/IRQ_POLICY.md) ---
    // Reports Qm33120::irqActive() (what actually happened), not the
    // Kconfig setting (CONFIG_UWB_ENABLE_IRQ), so a silent fallback to
    // polling is visible from the boot log alone.
    if (uwbDevice.irqActive()) {
        ESP_LOGI(TAG, "irq=active (pin=%d)", cfg.pin_irq);
    } else if (cfg.pin_irq == UWB_PORT_PIN_UNUSED) {
        ESP_LOGI(TAG, "irq=polling (pin_irq unwired)");
    } else if (!cfg.use_irq) {
        ESP_LOGI(TAG, "irq=polling (disabled by Kconfig)");
    } else {
        ESP_LOGW(TAG, "irq=polling (enable failed)");
    }

    // --- Timing preset (docs/TIMING_PRESETS.md, task D-2) ---
    ESP_LOGI(TAG, "timing profile=%s (version=%u)", uwb::timingProfileName(TIMING_PROFILE),
             (unsigned)uwb::kTimingPresetVersion);
    // Diagnostics: the crystal trim and the die temperature this chip booted
    // at. A large trim difference between the two boards - or a large die
    // temperature difference, since the crystal drifts with temperature -
    // shows up as a carrier frequency offset the receiver has to track.
    // 診断: この個体の水晶トリム値と起動時のダイ温度。2 個体間でこれらが
    // 大きく違うと搬送波の周波数ずれになり、受信側の追従範囲を超えうる。
    ESP_LOGI(TAG, "xtal_trim=%u die_temp=%.1fC", (unsigned)dwt_getxtaltrim(), dieTempC());
    // Boot-time check (docs/TIMING_PRESETS.md SS4(b)): warn loudly if the
    // selected preset needs IRQ on this device's role but it did not
    // actually come up active. **Never change the preset value
    // automatically** - it must match the peer, so a one-sided silent
    // change would defeat the whole point.
#if CONFIG_UWB_TWR_ROLE_ANCHOR
    if (uwb::timingProfileNeedsAnchorIrq(TIMING_PROFILE) && !uwbDevice.irqActive()) {
        ESP_LOGW(TAG,
                 "timing profile=%s requires IRQ on the ANCHOR, but irqActive()==false "
                 "(pin_irq unwired, disabled by Kconfig, or ISR registration failed). The wait fell back to "
                 "polling, whose ~1.2ms turnaround is likely to miss this preset's deadline. The preset value "
                 "is NOT changed automatically - reflash both TAG and ANCHOR with PollingBoth if IRQ cannot "
                 "be wired (docs/TIMING_PRESETS.md).",
                 uwb::timingProfileName(TIMING_PROFILE));
    }
#else
    if (uwb::timingProfileNeedsTagIrq(TIMING_PROFILE) && !uwbDevice.irqActive()) {
        ESP_LOGW(TAG,
                 "timing profile=%s also requires IRQ on the TAG, but irqActive()==false "
                 "(pin_irq unwired, disabled by Kconfig, or ISR registration failed). The wait fell back to "
                 "polling, whose ~1.2ms turnaround is likely to miss this preset's deadline. The preset value "
                 "is NOT changed automatically - reflash both TAG and ANCHOR with PollingBoth if IRQ cannot "
                 "be wired (docs/TIMING_PRESETS.md).",
                 uwb::timingProfileName(TIMING_PROFILE));
    }
#endif

    runRole(uwbDevice);
}
