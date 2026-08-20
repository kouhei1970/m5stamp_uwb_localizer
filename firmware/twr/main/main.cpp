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
 * 【docs/REIMPL_PLAN.md R3-1/R9】上記の「examplesの値をそのまま使用」の
 * 例外として、RANGE_HOST_TIMEOUT_MS(100→10)/RESULT_RX_AFTER_FINAL_TX_DLY_UUS
 * (500→200)/RESULT_REPEAT_COUNT(3→1) の3つは
 * uwb::RangeConfig/DSRangeConfig 側の新しい既定値
 * （components/uwb_qm33120/include/uwb_qm33120_types.hpp）に揃えて更新した
 * （examplesの値は検証されていない二次資料であり、この3つは特に
 * docs/CRITICAL_REVIEW.md【重大3】で問題が指摘されていたため）。
 */
#include <cmath>
#include <cstdio>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "uwb_port.h"
#include "uwb_qm33120.hpp"

#if CONFIG_UWB_TWR_BOARD_ATOMS3
#include "boards/atoms3.h"
#define BOARD_UWB_PORT_CONFIG BOARD_ATOMS3_UWB_PORT_CONFIG
#define BOARD_NAME            "AtomS3"
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

#define UWB_DEV_ID_EXPECTED 0xDECA0314UL

/* --- ネットワーク共通パラメータ（4組み合わせ共通。examples 配下の各 .ino と同じ値） --- */
static constexpr uint16_t PAN_ID           = 0xDECA;
static constexpr uint16_t TAG_SHORT_ADDR    = 0x0001;
static constexpr uint16_t ANCHOR_SHORT_ADDR = 0x0002;

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
    cfg.init_spi_bus = port.init_spi_bus;
    return cfg;
}

/* runRole() は選択された ロール×方式 の組み合わせだけをビルドする
 * (4通りの中から1つ)。他の3つを常に定義すると、選択されなかった側が
 * -Wunused-function の対象になる（firmware/devtest と同じ理由）。 */

#if !CONFIG_UWB_TWR_ROLE_ANCHOR && !CONFIG_UWB_TWR_METHOD_DS
/* =========================================================================
 * TAG + SS-TWR : examples/SS_TWR_TAG/SS_TWR_TAG.ino 準拠
 * ========================================================================= */

static constexpr uint32_t RANGE_INTERVAL_MS            = 200;
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

        const uwb::RangeResult result = uwb.requestRange(makeRangeConfig());
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
                     "SS_RANGE_STAT count=%lu ok=%lu fail=%lu rate=%.1f%% last=OK seq=%u distance_mm=%ld "
                     "distance_m=%.3f mean_mm=%.1f std_mm=%.1f n=%lu elapsed_ms=%lu",
                     (unsigned long)rangeCount, (unsigned long)rangeOkCount, (unsigned long)rangeFailCount, rate,
                     result.sequence, (long)result.distanceMm, result.distanceM, stats.mean, stats.stddev(),
                     (unsigned long)stats.count, (unsigned long)result.elapsedMs);
        } else {
            ESP_LOGW(TAG, "SS_RANGE_STAT count=%lu ok=%lu fail=%lu rate=%.1f%% last=FAIL seq=%u error=%s",
                     (unsigned long)rangeCount, (unsigned long)rangeOkCount, (unsigned long)rangeFailCount, rate,
                     result.sequence, uwb.lastErrorName());
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
    return range;
}

static void runRole(uwb::Qm33120& uwb)
{
    uint32_t respCount = 0, failCount = 0;

    while (1) {
        const uwb::ResponderResult result = uwb.respondRange(makeRangeConfig());
        if (!result.success) {
            if (result.error == uwb::Error::RxTimeout) {
                // まだPollが来ていないだけ。原本のANCHOR例と同じく無視して再ループ。
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
        if ((respCount % ANCHOR_LOG_INTERVAL) != 0) {
            continue;
        }
        ESP_LOGI(TAG, "SS_RESP_STAT ok=%lu fail=%lu last=OK seq=%u requester=0x%04X elapsed_ms=%lu",
                 (unsigned long)respCount, (unsigned long)failCount, result.sequence, result.requester,
                 (unsigned long)result.elapsedMs);
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
    const uwb::PhyConfig phy; // defaults: ch9, preamble128, PAC8, 6.8Mbps (see uwb_qm33120_types.hpp)

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

    runRole(uwbDevice);
}
