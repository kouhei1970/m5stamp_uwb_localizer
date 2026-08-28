/**
 * @file main.c
 * @brief Phase 1 acceptance test: verify SPI connectivity to the Qorvo
 *        QM33120W/DW3720 (M5Stamp UWB Module) over components/uwb_port,
 *        via two independent checks (L1: raw SPI DEV_ID read, L2:
 *        dwt_probe()+dwt_readdevid()), mirroring
 *        third_party/M5Stamp-UWB/src/M5Stamp_UWB.cpp's begin()/probe().
 *
 * Board selection: Kconfig choice UWB_PROBE_BOARD (see
 * main/Kconfig.projbuild), default M5StampS3A. Pin definitions live in
 * boards/stamps3.h / boards/atoms3.h at the repo root (PROVISIONAL, not yet
 * verified against real wiring - see the warning banner in those headers).
 */
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "deca_device_api.h"
#include "deca_interface.h"
#include "uwb_status_led.h"
#include "uwb_port.h"

#if CONFIG_UWB_PROBE_BOARD_ATOMS3
#include "boards/atoms3.h"
#define BOARD_UWB_PORT_CONFIG BOARD_ATOMS3_UWB_PORT_CONFIG
#define BOARD_NAME            "AtomS3(pinout " BOARD_ATOMS3_PINOUT_NAME ")"
#elif CONFIG_UWB_PROBE_BOARD_STAMPFLY
#include "boards/stampfly.h"
#define BOARD_UWB_PORT_CONFIG BOARD_STAMPFLY_UWB_PORT_CONFIG
#define BOARD_NAME            "StampFly"
#else
#include "boards/stamps3.h"
#define BOARD_UWB_PORT_CONFIG BOARD_STAMPS3_UWB_PORT_CONFIG
#define BOARD_NAME            "M5StampS3A"
/* Only the M5StampS3A carries a WS2812 on a known GPIO; the AtomS3 has an
 * LCD instead, so the heartbeat is compiled out there.
 * 内蔵フルカラー LED を持つのは M5StampS3A のみ（AtomS3 は LCD）。
 * それ以外のボードではハートビート表示ごとコンパイルから外れる。 */
#define BOARD_STATUS_LED_GPIO BOARD_STAMPS3_STATUS_LED_GPIO
#endif

static const char *TAG = "uwb_probe";

#define UWB_DEV_ID_EXPECTED   0xDECA0314UL

/* Heartbeat blink: amber-yellow at 2 Hz. Green is perceptually brighter
 * than red on a WS2812, so the green component is kept lower than the red
 * one to land on yellow rather than yellow-green. Raise both to make it
 * brighter (0-255).
 * ハートビートの点滅色と周期。WS2812 は緑が赤より明るく見えるので、
 * 黄緑に転ばないよう緑を赤より小さくしてある。両方を上げれば明るくなる。 */
#define PROBE_RETRY_COUNT     5
#define PROBE_RETRY_DELAY_MS  20

/* Matches the Arduino reference's own top-of-file extern declaration
 * (M5Stamp_UWB.cpp); USE_DRV_DW3720 is not defined in this build so the
 * extern in deca_device_api.h is compiled out and we must declare it
 * ourselves. */
extern const struct dwt_driver_s dw3720_driver;

/**
 * @brief L1: raw SPI DEV_ID read, mirrors readRawDeviceId() in the Arduino
 * reference. Header byte 0x00 selects register file 0 (DEV_ID), no
 * sub-index. Retries up to PROBE_RETRY_COUNT times if the value comes back
 * as 0x00000000 or 0xFFFFFFFF (both mean "no response" / "bus idle", not
 * necessarily a real mismatch).
 */
static bool run_l1_raw_spi_check(struct dwt_spi_s *spi, uint32_t *out_dev_id)
{
    uint32_t raw_dev_id = 0;

    for (int attempt = 1; attempt <= PROBE_RETRY_COUNT; ++attempt) {
        uint8_t header  = 0x00;
        uint8_t buf[4]  = {0};

        int32_t rc = spi->readfromspi(1, &header, sizeof(buf), buf);
        if (rc != DWT_SUCCESS) {
            ESP_LOGW(TAG, "L1: readfromspi failed (attempt %d/%d)", attempt, PROBE_RETRY_COUNT);
            vTaskDelay(pdMS_TO_TICKS(PROBE_RETRY_DELAY_MS));
            continue;
        }

        raw_dev_id = ((uint32_t)buf[3] << 24) | ((uint32_t)buf[2] << 16) | ((uint32_t)buf[1] << 8) | (uint32_t)buf[0];

        if ((raw_dev_id != 0x00000000UL) && (raw_dev_id != 0xFFFFFFFFUL)) {
            break;
        }

        ESP_LOGW(TAG, "L1: no response (0x%08lX) on attempt %d/%d, retrying", (unsigned long)raw_dev_id, attempt,
                 PROBE_RETRY_COUNT);
        vTaskDelay(pdMS_TO_TICKS(PROBE_RETRY_DELAY_MS));
    }

    *out_dev_id = raw_dev_id;

    bool ok = (raw_dev_id == UWB_DEV_ID_EXPECTED);
    if (ok) {
        ESP_LOGI(TAG, "L1: raw DEV_ID = 0x%08lX (expect 0x%08lX) -> OK", (unsigned long)raw_dev_id,
                 (unsigned long)UWB_DEV_ID_EXPECTED);
    } else {
        ESP_LOGE(TAG, "L1: raw DEV_ID = 0x%08lX (expect 0x%08lX) -> MISMATCH", (unsigned long)raw_dev_id,
                 (unsigned long)UWB_DEV_ID_EXPECTED);
    }
    return ok;
}

/**
 * @brief L2: dwt_probe() + dwt_readdevid(), mirrors probe() in the Arduino
 * reference.
 */
static bool run_l2_dwt_probe_check(struct dwt_spi_s *spi, uint32_t *out_dev_id)
{
    struct dwt_driver_s *drivers[] = {(struct dwt_driver_s *)&dw3720_driver};
    struct dwt_probe_s probe_interface = {0};
    probe_interface.dw                    = NULL;
    probe_interface.spi                   = spi;
    probe_interface.wakeup_device_with_io = uwb_port_wakeup_device_with_io;
    probe_interface.driver_list           = drivers;
    probe_interface.dw_driver_num         = sizeof(drivers) / sizeof(drivers[0]);

    for (int attempt = 1; attempt <= PROBE_RETRY_COUNT; ++attempt) {
        if (dwt_probe(&probe_interface) == DWT_SUCCESS) {
            uint32_t dev_id = dwt_readdevid();
            *out_dev_id     = dev_id;

            bool ok = (dev_id == UWB_DEV_ID_EXPECTED);
            if (ok) {
                ESP_LOGI(TAG, "L2: dwt_probe + dwt_readdevid = 0x%08lX (expect 0x%08lX) -> OK",
                         (unsigned long)dev_id, (unsigned long)UWB_DEV_ID_EXPECTED);
            } else {
                ESP_LOGE(TAG, "L2: dwt_probe + dwt_readdevid = 0x%08lX (expect 0x%08lX) -> MISMATCH",
                         (unsigned long)dev_id, (unsigned long)UWB_DEV_ID_EXPECTED);
            }
            return ok;
        }
        ESP_LOGW(TAG, "L2: dwt_probe failed (attempt %d/%d)", attempt, PROBE_RETRY_COUNT);
        vTaskDelay(pdMS_TO_TICKS(PROBE_RETRY_DELAY_MS));
    }

    *out_dev_id = 0;
    ESP_LOGE(TAG, "L2: dwt_probe failed after %d attempts", PROBE_RETRY_COUNT);
    return false;
}

void app_main(void)
{
    ESP_LOGI(TAG, "Phase 1 UWB probe acceptance test, board=%s", BOARD_NAME);

    /* Start the heartbeat before anything else can fail, and note that the
     * blink runs in its own FreeRTOS task: it keeps going even when
     * app_main() returns early on an error below. A blinking LED therefore
     * means "the board booted and is running", not "the probe passed" - the
     * DEV_ID verdict is only in the log.
     * 何かが失敗するより先に点滅を始める。点滅は専用の FreeRTOS タスクで
     * 走るので、下でエラー復帰して app_main() を抜けても点滅は続く。
     * つまり点滅は「起動して動作中」の意味であり、「疎通 OK」ではない。
     * 判定はログにしか出ない。 */
#ifdef BOARD_STATUS_LED_GPIO
    /* probe has no TAG/ANCHOR role, so it keeps the amber heartbeat.
     * probe は役割を持たないので琥珀色のまま。 */
    (void)uwb_status_led_start_role_heartbeat(BOARD_STATUS_LED_GPIO, UWB_STATUS_LED_ROLE_NONE);
#endif

    uwb_port_config_t cfg = BOARD_UWB_PORT_CONFIG;

    /* Task D-2 (docs/HANDOFF.md SS5): override cfg.spi_fast_hz only when the
     * Kconfig value is non-zero. The default, 0, leaves the struct exactly
     * as BOARD_UWB_PORT_CONFIG built it (the per-board header under
     * boards/, currently 16 MHz) - identical to the behaviour before this
     * option existed. */
#if CONFIG_UWB_SPI_FAST_HZ > 0
    cfg.spi_fast_hz = CONFIG_UWB_SPI_FAST_HZ;
#endif
    ESP_LOGI(TAG, "spi_fast=%lu", (unsigned long)cfg.spi_fast_hz);

    esp_err_t err = uwb_port_init(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uwb_port_init failed: %s", esp_err_to_name(err));
        return;
    }

    uwb_port_hard_reset(5, 100);

    /* Arduino 版リファレンス M5Stamp_UWB::begin() は hardReset() の直後に
     * wakeupDeviceWithIoImpl() を1回無条件に呼んでから ID 読み出しリトライに入る。
     * ハードリセット済みならスリープ状態にはいないはずだが、リファレンスの
     * シーケンスに合わせておく（L1 が不安定な場合の切り分けを減らすため）。 */
    uwb_port_wakeup_device_with_io();

    struct dwt_spi_s *spi = uwb_port_spi();

    uint32_t l1_dev_id = 0;
    uint32_t l2_dev_id = 0;
    bool l1_ok = run_l1_raw_spi_check(spi, &l1_dev_id);
    bool l2_ok = run_l2_dwt_probe_check(spi, &l2_dev_id);

    ESP_LOGI(TAG, "=== L1: %s / L2: %s ===", l1_ok ? "PASS" : "FAIL", l2_ok ? "PASS" : "FAIL");

    /* Ongoing stability observation: redo the cheap L1 raw-SPI read every
     * second (no need to repeat L2/dwt_probe every second). */
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));

        uint8_t header = 0x00;
        uint8_t buf[4] = {0};
        int32_t rc     = spi->readfromspi(1, &header, sizeof(buf), buf);
        if (rc != DWT_SUCCESS) {
            ESP_LOGE(TAG, "L1 (periodic): readfromspi failed");
            continue;
        }

        uint32_t dev_id =
            ((uint32_t)buf[3] << 24) | ((uint32_t)buf[2] << 16) | ((uint32_t)buf[1] << 8) | (uint32_t)buf[0];
        ESP_LOGI(TAG, "L1 (periodic): raw DEV_ID = 0x%08lX", (unsigned long)dev_id);
    }
}
