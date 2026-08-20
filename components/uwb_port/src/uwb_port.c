/**
 * @file uwb_port.c
 * @brief ESP-IDF implementation of the uwb_port platform abstraction layer
 *        for the vendored Qorvo qm33120w_sdk driver.
 *
 * See include/uwb_port.h for the "Phase 1 = single-task only" scope note:
 * the SPI DMA scratch buffers below are module-level state, not per-call or
 * mutex-protected.
 */
#include "uwb_port.h"

#include <string.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* Only uwb_port.c needs the real qm33120w_sdk struct/API definitions
 * (struct dwt_spi_s, DWT_SUCCESS/DWT_ERROR, decaIrqStatus_t, ...). */
#include "deca_interface.h"

static const char *TAG = "uwb_port";

/* Fixed-size DMA scratch buffers. 4096 bytes comfortably covers any
 * header+body(+crc) combination used by the Qorvo driver's SPI transfers;
 * anything larger is rejected with DWT_ERROR rather than silently
 * truncated (see uwb_port.h Phase-1 scope note and requirement #6). */
#define UWB_PORT_SPI_SCRATCH_SIZE 4096

/* Wakeup pulse timings, matching the Arduino reference (M5Stamp_UWB.cpp
 * wakeupDeviceWithIoImpl()). */
#define UWB_PORT_WAKEUP_PULSE_MS 2
#define UWB_PORT_WAKEUP_IDLE_MS  5

static uwb_port_config_t s_cfg;
static bool s_initialized = false;

static spi_device_handle_t s_spi_slow   = NULL;
static spi_device_handle_t s_spi_fast   = NULL;
static spi_device_handle_t s_spi_active = NULL;

/* Two separate DMA-capable buffers (never aliased) so that full-duplex
 * transfers in readfromspi() do not corrupt tx while rx is being written. */
static uint8_t *s_tx_scratch = NULL;
static uint8_t *s_rx_scratch = NULL;

static portMUX_TYPE g_deca_mutex = portMUX_INITIALIZER_UNLOCKED;

/* ------------------------------------------------------------------------
 * Platform hooks required by the Qorvo SDK (declared in deca_device_api.h).
 * Signatures must match exactly or the SDK object files will fail to link.
 * ------------------------------------------------------------------------ */

void deca_sleep(unsigned int time_ms)
{
    vTaskDelay(pdMS_TO_TICKS(time_ms));
}

void deca_usleep(unsigned long time_us)
{
    esp_rom_delay_us((uint32_t)time_us);
}

decaIrqStatus_t decamutexon(void)
{
    portENTER_CRITICAL(&g_deca_mutex);
    return 0;
}

void decamutexoff(decaIrqStatus_t s)
{
    (void)s;
    portEXIT_CRITICAL(&g_deca_mutex);
}

/* ------------------------------------------------------------------------
 * struct dwt_spi_s implementation
 * ------------------------------------------------------------------------ */

/* ------------------------------------------------------------------------
 * CS はハードウェアCS(spics_io_num)ではなく、ソフトウェア制御（GPIO直接制御）
 * にしている。理由:
 *
 * ESP-IDFの spicommon_cs_initialize()
 * (~/esp/esp-idf/components/esp_driver_spi/src/gpspi/spi_common.c) は
 *     esp_rom_gpio_connect_out_signal(cs_io_num, spi_periph_signal[host].spics_out[cs_num], ...)
 * を呼び、1つのGPIOには1つの出力信号しか接続できない。slow/fast の2デバイスを
 * 同じ物理CSピン(spics_io_num = pin_cs)で spi_bus_add_device() すると、後から
 * 追加した方(cs_num=1, s_spi_fast)の spics_out[1] にピンが張り替えられてしまい、
 * 先に追加した方(cs_num=0, s_spi_slow)のCS信号(spics_out[0])はどのピンにも
 * 繋がらなくなる。結果、slowレートでの転送はCSが一度もLowにならず全滅する
 * （dwt_probe()の初期化がまさにこの経路を使う）。
 *
 * これを避けるため、両デバイスとも spics_io_num = -1（ハードウェアCS不使用）
 * にし、pin_cs は素のGPIO出力として自前でLow/Highを駆動する。これは Arduino版
 * リファレンス (third_party/M5Stamp-UWB/src/M5Stamp_UWB.cpp の
 * readFromSpiImpl/writeToSpiImpl) と同じ方式である。
 *
 * ※「ハードウェアCSの方が速いのでは」と将来ここを戻したくなった場合は、必ず
 *   上記の spicommon_cs_initialize() の制約（1GPIOにつき出力信号1本のみ）を
 *   再確認すること。
 * ------------------------------------------------------------------------ */

/**
 * @brief CSを自前でLow/High駆動しながら1回のSPIトランザクションを行う共通
 *        ヘルパ。readfromspi/writetospi(withcrc) は全てこれを経由させ、CS制御を
 *        1箇所に集約する。エラー経路でも必ずCSがHighに戻り、バスが解放される。
 *
 *        spi_device_acquire_bus() で明示的にバスを占有してからCSを操作する。
 *        手動CSでは、CSをLowにしている間に他ドライバのトランザクションが
 *        割り込むと通信が壊れるため（将来 StampFly へ統合すると SPI2_HOST を
 *        BMI270/PMW3901 と共有するので、これが実害になる）。
 */
static esp_err_t uwb_spi_xfer(const void *tx, void *rx, size_t len_bytes)
{
    spi_transaction_t t = {
        .length    = len_bytes * 8u,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };

    esp_err_t err = spi_device_acquire_bus(s_spi_active, portMAX_DELAY);
    if (err != ESP_OK) {
        return err;
    }
    gpio_set_level(s_cfg.pin_cs, 0);
    err = spi_device_polling_transmit(s_spi_active, &t);
    gpio_set_level(s_cfg.pin_cs, 1);
    spi_device_release_bus(s_spi_active);
    return err;
}

static int32_t uwb_spi_readfromspi(uint16_t headerLength, uint8_t *headerBuffer, uint16_t readlength,
                                    uint8_t *readBuffer)
{
    if (!s_initialized || (s_spi_active == NULL) || ((headerLength > 0) && (headerBuffer == NULL)) ||
        ((readlength > 0) && (readBuffer == NULL))) {
        return DWT_ERROR;
    }

    const uint32_t total = (uint32_t)headerLength + (uint32_t)readlength;
    if (total > UWB_PORT_SPI_SCRATCH_SIZE) {
        ESP_LOGE(TAG, "readfromspi: total length %u exceeds scratch buffer (%d)", (unsigned int)total,
                 UWB_PORT_SPI_SCRATCH_SIZE);
        return DWT_ERROR;
    }

    if (headerLength > 0) {
        memcpy(s_tx_scratch, headerBuffer, headerLength);
    }
    if (readlength > 0) {
        /* Zero-pad the read phase of the tx buffer (dummy clock-out bytes). */
        memset(s_tx_scratch + headerLength, 0x00, readlength);
    }

    esp_err_t err = uwb_spi_xfer(s_tx_scratch, s_rx_scratch, total);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "readfromspi: uwb_spi_xfer failed: %s", esp_err_to_name(err));
        return DWT_ERROR;
    }

    if (readlength > 0) {
        /* Payload starts right after the header bytes in the rx scratch
         * buffer, matching what dwt_readdevid()/readRawDeviceId()-style
         * callers expect in readBuffer (payload only, not header+payload). */
        memcpy(readBuffer, s_rx_scratch + headerLength, readlength);
    }

    return DWT_SUCCESS;
}

static int32_t uwb_spi_write_impl(uint16_t headerLength, const uint8_t *headerBuffer, uint16_t bodyLength,
                                   const uint8_t *bodyBuffer, const uint8_t *crc8)
{
    if (!s_initialized || (s_spi_active == NULL) || ((headerLength > 0) && (headerBuffer == NULL))) {
        return DWT_ERROR;
    }

    const uint32_t total = (uint32_t)headerLength + (uint32_t)bodyLength + (uint32_t)((crc8 != NULL) ? 1u : 0u);
    if (total > UWB_PORT_SPI_SCRATCH_SIZE) {
        ESP_LOGE(TAG, "writetospi: total length %u exceeds scratch buffer (%d)", (unsigned int)total,
                 UWB_PORT_SPI_SCRATCH_SIZE);
        return DWT_ERROR;
    }

    uint32_t offset = 0;
    if (headerLength > 0) {
        memcpy(s_tx_scratch, headerBuffer, headerLength);
        offset += headerLength;
    }
    if (bodyLength > 0) {
        if (bodyBuffer != NULL) {
            memcpy(s_tx_scratch + offset, bodyBuffer, bodyLength);
        } else {
            memset(s_tx_scratch + offset, 0x00, bodyLength);
        }
        offset += bodyLength;
    }
    if (crc8 != NULL) {
        s_tx_scratch[offset] = *crc8;
        offset += 1;
    }

    /* write-only: rx_buffer=NULL で受信ビットは破棄する */
    esp_err_t err = uwb_spi_xfer(s_tx_scratch, NULL, total);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "writetospi: uwb_spi_xfer failed: %s", esp_err_to_name(err));
        return DWT_ERROR;
    }

    return DWT_SUCCESS;
}

static int32_t uwb_spi_writetospi(uint16_t headerLength, const uint8_t *headerBuffer, uint16_t bodyLength,
                                   const uint8_t *bodyBuffer)
{
    return uwb_spi_write_impl(headerLength, headerBuffer, bodyLength, bodyBuffer, NULL);
}

static int32_t uwb_spi_writetospiwithcrc(uint16_t headerLength, const uint8_t *headerBuffer, uint16_t bodyLength,
                                          const uint8_t *bodyBuffer, uint8_t crc8)
{
    return uwb_spi_write_impl(headerLength, headerBuffer, bodyLength, bodyBuffer, &crc8);
}

static void uwb_spi_setslowrate(void)
{
    if (s_spi_slow != NULL) {
        s_spi_active = s_spi_slow;
    }
}

static void uwb_spi_setfastrate(void)
{
    if (s_spi_fast != NULL) {
        s_spi_active = s_spi_fast;
    }
}

static struct dwt_spi_s s_dwt_spi = {
    .readfromspi       = uwb_spi_readfromspi,
    .writetospi         = uwb_spi_writetospi,
    .writetospiwithcrc  = uwb_spi_writetospiwithcrc,
    .setslowrate        = uwb_spi_setslowrate,
    .setfastrate        = uwb_spi_setfastrate,
};

struct dwt_spi_s *uwb_port_spi(void)
{
    return &s_dwt_spi;
}

/* ------------------------------------------------------------------------
 * GPIO / reset / wakeup
 * ------------------------------------------------------------------------ */

void uwb_port_hard_reset(uint32_t reset_low_ms, uint32_t startup_ms)
{
    if (s_cfg.pin_rst == UWB_PORT_PIN_UNUSED) {
        vTaskDelay(pdMS_TO_TICKS(startup_ms));
        return;
    }

    /* RSTn is meant to be released and pulled high externally, not actively
     * driven high by the host: drive low for the pulse, then tri-state
     * (input) again afterwards. */
    gpio_set_direction(s_cfg.pin_rst, GPIO_MODE_OUTPUT);
    gpio_set_level(s_cfg.pin_rst, 0);
    vTaskDelay(pdMS_TO_TICKS(reset_low_ms));
    gpio_set_direction(s_cfg.pin_rst, GPIO_MODE_INPUT);
    vTaskDelay(pdMS_TO_TICKS(startup_ms));
}

void uwb_port_set_wakeup(bool level)
{
    if (s_cfg.pin_wakeup == UWB_PORT_PIN_UNUSED) {
        return;
    }
    gpio_set_level(s_cfg.pin_wakeup, level ? 1 : 0);
}

int uwb_port_read_gp7(void)
{
    if (s_cfg.pin_gp7 == UWB_PORT_PIN_UNUSED) {
        return -1;
    }
    return gpio_get_level(s_cfg.pin_gp7) ? 1 : 0;
}

void uwb_port_wakeup_device_with_io(void)
{
    if (s_cfg.pin_wakeup != UWB_PORT_PIN_UNUSED) {
        gpio_set_level(s_cfg.pin_wakeup, 1);
        vTaskDelay(pdMS_TO_TICKS(UWB_PORT_WAKEUP_PULSE_MS));
        gpio_set_level(s_cfg.pin_wakeup, 0);
        vTaskDelay(pdMS_TO_TICKS(UWB_PORT_WAKEUP_IDLE_MS));
        return;
    }

    /* フォールバック: WAKEUPピン未配線の場合はCSをパルスする。CSはソフトウェア
     * 制御（GPIO直接制御）なので、Arduino版リファレンス
     * (wakeupDeviceWithIoImpl()) と同様に gpio_set_level() で直接駆動できる。
     * バス共有下での安全性のため、CS操作はspi_device_acquire_bus()で
     * バスを占有した状態で行う（uwb_spi_xfer()と同じ理由）。 */
    if (!s_initialized || (s_spi_slow == NULL)) {
        return;
    }

    esp_err_t err = spi_device_acquire_bus(s_spi_slow, portMAX_DELAY);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "wakeup_device_with_io: spi_device_acquire_bus failed: %s", esp_err_to_name(err));
        return;
    }
    gpio_set_level(s_cfg.pin_cs, 0);
    vTaskDelay(pdMS_TO_TICKS(UWB_PORT_WAKEUP_PULSE_MS));
    gpio_set_level(s_cfg.pin_cs, 1);
    spi_device_release_bus(s_spi_slow);
    vTaskDelay(pdMS_TO_TICKS(UWB_PORT_WAKEUP_IDLE_MS));
}

/* ------------------------------------------------------------------------
 * init / deinit
 * ------------------------------------------------------------------------ */

esp_err_t uwb_port_init(const uwb_port_config_t *cfg)
{
    if ((cfg == NULL) || (cfg->pin_cs == UWB_PORT_PIN_UNUSED) || (cfg->pin_sck == UWB_PORT_PIN_UNUSED) ||
        (cfg->pin_mosi == UWB_PORT_PIN_UNUSED) || (cfg->pin_miso == UWB_PORT_PIN_UNUSED)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    s_cfg = *cfg;
    if (s_cfg.spi_slow_hz == 0) {
        s_cfg.spi_slow_hz = 2000000;
    }
    if (s_cfg.spi_fast_hz == 0) {
        s_cfg.spi_fast_hz = 16000000;
    }

    /* --- GPIO init (matches the Arduino reference's begin()) --- */
    /* pin_cs はソフトウェア制御CSとして使うため、spi_bus_add_device()より前に
     * gpio_reset_pin()でGPIOマトリクスの残留ルーティングを確実に切ってから、
     * 素のGPIO出力として初期化する（詳細は uwb_spi_xfer() 直前のコメント参照）。 */
    gpio_reset_pin(s_cfg.pin_cs);
    gpio_set_direction(s_cfg.pin_cs, GPIO_MODE_OUTPUT);
    gpio_set_level(s_cfg.pin_cs, 1); /* idle high */

    if (s_cfg.pin_rst != UWB_PORT_PIN_UNUSED) {
        gpio_set_direction(s_cfg.pin_rst, GPIO_MODE_INPUT); /* tri-state, no active drive */
    }
    if (s_cfg.pin_wakeup != UWB_PORT_PIN_UNUSED) {
        gpio_set_direction(s_cfg.pin_wakeup, GPIO_MODE_OUTPUT);
        gpio_set_level(s_cfg.pin_wakeup, 0);
    }
    if (s_cfg.pin_irq != UWB_PORT_PIN_UNUSED) {
        gpio_set_direction(s_cfg.pin_irq, GPIO_MODE_INPUT);
    }
    if (s_cfg.pin_gp7 != UWB_PORT_PIN_UNUSED) {
        gpio_set_direction(s_cfg.pin_gp7, GPIO_MODE_INPUT);
    }

    /* --- SPI bus --- */
    if (s_cfg.init_spi_bus) {
        spi_bus_config_t buscfg = {
            .mosi_io_num     = s_cfg.pin_mosi,
            .miso_io_num     = s_cfg.pin_miso,
            .sclk_io_num     = s_cfg.pin_sck,
            .quadwp_io_num   = -1,
            .quadhd_io_num   = -1,
            .max_transfer_sz = UWB_PORT_SPI_SCRATCH_SIZE,
        };
        esp_err_t err = spi_bus_initialize(s_cfg.spi_host, &buscfg, SPI_DMA_CH_AUTO);
        if (err == ESP_ERR_INVALID_STATE) {
            /* Someone else already initialized this bus; treat as success
             * (matches stampfly_ecosystem driver convention). */
            ESP_LOGW(TAG, "SPI host %d bus already initialized, reusing it", (int)s_cfg.spi_host);
        } else if (err != ESP_OK) {
            ESP_LOGE(TAG, "spi_bus_initialize failed: %s", esp_err_to_name(err));
            return err;
        }
    }

    /* spics_io_num = -1: ハードウェアCSは使わない。CSはソフトウェア制御
     * （pin_csを素のGPIOとして自前で駆動）にしている。理由は uwb_spi_xfer()
     * 直前のコメントを参照（同一GPIOに2デバイスのハードウェアCSを割り当てると
     * ESP-IDFのGPIOマトリクス上、片方のCS信号がどこにも繋がらなくなる）。 */
    spi_device_interface_config_t slow_devcfg = {
        .mode           = 0, /* DW3720 uses SPI mode 0 */
        .clock_speed_hz = (int)s_cfg.spi_slow_hz,
        .spics_io_num   = -1,
        .queue_size     = 1,
    };
    esp_err_t err = spi_bus_add_device(s_cfg.spi_host, &slow_devcfg, &s_spi_slow);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_add_device (slow) failed: %s", esp_err_to_name(err));
        return err;
    }

    spi_device_interface_config_t fast_devcfg = slow_devcfg;
    fast_devcfg.clock_speed_hz                = (int)s_cfg.spi_fast_hz;
    err                                        = spi_bus_add_device(s_cfg.spi_host, &fast_devcfg, &s_spi_fast);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_add_device (fast) failed: %s", esp_err_to_name(err));
        spi_bus_remove_device(s_spi_slow);
        s_spi_slow = NULL;
        return err;
    }

    /* Start in slow mode, matching the Arduino reference (slow rate is used
     * before/during probe, fast rate is switched to afterwards). */
    s_spi_active = s_spi_slow;

    s_tx_scratch = heap_caps_malloc(UWB_PORT_SPI_SCRATCH_SIZE, MALLOC_CAP_DMA);
    s_rx_scratch = heap_caps_malloc(UWB_PORT_SPI_SCRATCH_SIZE, MALLOC_CAP_DMA);
    if ((s_tx_scratch == NULL) || (s_rx_scratch == NULL)) {
        ESP_LOGE(TAG, "failed to allocate %d-byte DMA scratch buffers", UWB_PORT_SPI_SCRATCH_SIZE);
        heap_caps_free(s_tx_scratch);
        heap_caps_free(s_rx_scratch);
        s_tx_scratch = NULL;
        s_rx_scratch = NULL;
        spi_bus_remove_device(s_spi_fast);
        spi_bus_remove_device(s_spi_slow);
        s_spi_fast   = NULL;
        s_spi_slow   = NULL;
        s_spi_active = NULL;
        return ESP_ERR_NO_MEM;
    }

    s_initialized = true;
    return ESP_OK;
}

esp_err_t uwb_port_deinit(void)
{
    if (!s_initialized) {
        return ESP_OK;
    }

    s_spi_active = NULL;

    if (s_spi_fast != NULL) {
        spi_bus_remove_device(s_spi_fast);
        s_spi_fast = NULL;
    }
    if (s_spi_slow != NULL) {
        spi_bus_remove_device(s_spi_slow);
        s_spi_slow = NULL;
    }

    if (s_tx_scratch != NULL) {
        heap_caps_free(s_tx_scratch);
        s_tx_scratch = NULL;
    }
    if (s_rx_scratch != NULL) {
        heap_caps_free(s_rx_scratch);
        s_rx_scratch = NULL;
    }

    /* Intentionally does not call spi_bus_free(): the bus may be shared with
     * other devices/components (see init_spi_bus semantics). */

    s_initialized = false;
    return ESP_OK;
}
