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
 *     0x0002 だった)。手持ちのStamp UWB-F 5台それぞれに異なる値を書き込んで
 *     区別する運用を想定している。
 *
 * Kconfig で選ぶ2軸 (main/Kconfig.projbuild):
 *   - ボード: M5Stamp S3 / M5 AtomS3        (boards ディレクトリ配下の
 *     stamps3.h / atoms3.h でピン定義切替。firmware/twr と同じ作法)
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
 * NVS保存・シリアルコンソールからのショートアドレス変更は未実装(将来課題)。
 * 現時点ではKconfigの UWB_ANCHOR_SHORT_ADDR の値を書き込み時に焼き込む形。
 */
#include <cmath>
#include <cstdio>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "uwb_port.h"
#include "uwb_qm33120.hpp"

#if CONFIG_UWB_ANCHOR_BOARD_ATOMS3
#include "boards/atoms3.h"
#define BOARD_UWB_PORT_CONFIG BOARD_ATOMS3_UWB_PORT_CONFIG
#define BOARD_NAME            "AtomS3"
#else
#include "boards/stamps3.h"
#define BOARD_UWB_PORT_CONFIG BOARD_STAMPS3_UWB_PORT_CONFIG
#define BOARD_NAME            "Stamp S3"
#endif

#if CONFIG_UWB_ANCHOR_METHOD_DS
#define METHOD_NAME "DS-TWR"
#else
#define METHOD_NAME "SS-TWR"
#endif

static const char* TAG = "uwb_anchor";

#define UWB_DEV_ID_EXPECTED 0xDECA0314UL

/* --- ネットワーク共通パラメータ --- */
static constexpr uint16_t PAN_ID = 0xDECA;
// タグ側(initiator)のショートアドレス。firmware/tag(別タスクで作成予定)の
// 実装が固まるまでは、firmware/twr と同じくここでは固定値として扱う。
static constexpr uint16_t TAG_SHORT_ADDR = 0x0001;
// このアンカー自身のショートアドレス。Kconfigの UWB_ANCHOR_SHORT_ADDR
// (main/Kconfig.projbuild) をそのまま使う。実機5台それぞれに異なる値を
// 書き込んで区別する運用のため、ソース上の固定値にはしない。
static constexpr uint16_t ANCHOR_SHORT_ADDR = CONFIG_UWB_ANCHOR_SHORT_ADDR;

/**
 * @brief 距離サンプル(mm)の平均・標準偏差をオンライン計算する
 * (Welfordのアルゴリズム)。全サンプルを保持せずに済むので長時間の実機運用に使える。
 * firmware/twr/main/main.cpp の DistanceStats と同一実装。
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
static constexpr uint32_t RANGE_HOST_TIMEOUT_MS               = 100;
static constexpr uint32_t RESPONSE_RX_AFTER_TX_DLY_UUS        = 1500;
static constexpr uint32_t RESPONSE_TX_DLY_UUS                 = 3000;
static constexpr uint32_t FINAL_TX_DLY_UUS                    = 1800;
static constexpr uint32_t FINAL_RX_AFTER_RESPONSE_TX_DLY_UUS  = 500;
static constexpr uint32_t RESULT_RX_AFTER_FINAL_TX_DLY_UUS    = 500;
static constexpr uint8_t RESULT_REPEAT_COUNT                  = 3;
static constexpr uint32_t RESULT_REPEAT_GAP_MS                = 3;

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
    // DS-TWR は Anchor 側が距離を計算する側 (respondDSRange()) なので、
    // 運用中の平均/標準偏差はこちら側でも取れる。
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
static constexpr uint32_t RANGE_HOST_TIMEOUT_MS         = 100;
static constexpr uint32_t RESPONSE_RX_AFTER_TX_DLY_UUS  = 1500;
static constexpr uint32_t RESPONSE_TX_DLY_UUS           = 3000;

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
                const uint32_t total = respCount + failCount;
                const float rate =
                    (total == 0) ? 0.0f : (100.0f * static_cast<float>(respCount) / static_cast<float>(total));
                ESP_LOGW(TAG, "SS_RESP_STAT ok=%lu fail=%lu rate=%.1f%% last=FAIL error=%s",
                         (unsigned long)respCount, (unsigned long)failCount, rate, uwb.lastErrorName());
            }
            continue;
        }

        respCount++;
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
    ESP_LOGI(TAG, "Phase 4 Step 2 uwb_qm33120 production anchor firmware, board=%s method=%s short_addr=0x%04X",
             BOARD_NAME, METHOD_NAME, ANCHOR_SHORT_ADDR);

    uwb::Qm33120 uwbDevice;
    const uwb::Config cfg = makeConfigFromBoard();
    const uwb::PhyConfig phy; // 既定値: ch9, preamble128, PAC8, 6.8Mbps (uwb_qm33120_types.hpp参照)

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
    ESP_LOGI(TAG, "begin() + PHY config OK, starting ANCHOR/%s loop (short_addr=0x%04X)", METHOD_NAME,
             ANCHOR_SHORT_ADDR);

    runRole(uwbDevice);
}
