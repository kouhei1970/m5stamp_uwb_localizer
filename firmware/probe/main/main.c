/**
 * @file main.c
 * @brief Phase 1 acceptance test: verify SPI connectivity to the Qorvo
 *        QM33120W/DW3720 (M5Stack M5Stamp UWB Module with FPC) over components/uwb_port,
 *        via two independent checks (L1: raw SPI DEV_ID read, L2:
 *        dwt_probe()+dwt_readdevid()), mirroring
 *        third_party/M5Stamp-UWB/src/M5Stamp_UWB.cpp's begin()/probe().
 *
 * Board selection: Kconfig choice UWB_PROBE_BOARD (see
 * main/Kconfig.projbuild), default M5Stamp S3. Pin definitions live in
 * boards/stamps3.h / boards/atoms3.h at the repo root (PROVISIONAL, not yet
 * verified against real wiring - see the warning banner in those headers).
 */
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "deca_device_api.h"
#include "deca_interface.h"
#include "uwb_port.h"

#if CONFIG_UWB_PROBE_BOARD_ATOMS3
#include "boards/atoms3.h"
#define BOARD_UWB_PORT_CONFIG BOARD_ATOMS3_UWB_PORT_CONFIG
#define BOARD_NAME            "AtomS3"
#else
#include "boards/stamps3.h"
#define BOARD_UWB_PORT_CONFIG BOARD_STAMPS3_UWB_PORT_CONFIG
#define BOARD_NAME            "Stamp S3"
#endif

static const char *TAG = "uwb_probe";

#define UWB_DEV_ID_EXPECTED   0xDECA0314UL
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

    uwb_port_config_t cfg = BOARD_UWB_PORT_CONFIG;

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
