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
#include "freertos/semphr.h"
#include "freertos/task.h"

/* Only uwb_port.c needs the real qm33120w_sdk struct/API definitions
 * (struct dwt_spi_s, DWT_SUCCESS/DWT_ERROR, decaIrqStatus_t, ...). */
#include "deca_interface.h"

static const char *TAG = "uwb_port";

/* 【M-1】本ファイルの deca_sleep()（下記）は「1 tick <= 1ms」
 * （= CONFIG_FREERTOS_HZ >= 1000）を前提に、tick境界の切り捨てを
 * +1 tick で吸収している（docs/REVIEW_2026-08-21.md §1 M-1）。
 * firmware/<app>/sdkconfig.defaults はいずれも CONFIG_FREERTOS_HZ=1000 を
 * 明示済みだが、将来どこかの sdkconfig.defaults がこの値を下げて
 * ビルドしてしまうと deca_sleep() の +1 tick 丸めが前提から外れ
 * （1 tick が 1ms を超えるため、+1 tick が deca_sleep(2) 等の短い
 * 待ちに対して過大/過小どちらの側にも意図と違う量になりうる）、
 * uwb_qm33120*.cpp 各所の pdMS_TO_TICKS(1) ベースの待ちループ粒度の
 * 前提も崩れる。実機を焼く前にビルドで気付けるよう #error で弾く。 */
#if !defined(CONFIG_FREERTOS_HZ) || (CONFIG_FREERTOS_HZ < 1000)
#error "uwb_port.c requires CONFIG_FREERTOS_HZ >= 1000 (1 tick <= 1ms); see firmware/*/sdkconfig.defaults and deca_sleep() below."
#endif

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

/* --- IRQ ("wakeup signal") state (docs/IRQ_POLICY.md) --- */
static SemaphoreHandle_t s_irq_sem    = NULL;
static bool s_irq_active              = false; /* uwb_port_irq_enable() に成功しているか */

/* ------------------------------------------------------------------------
 * Platform hooks required by the Qorvo SDK (declared in deca_device_api.h).
 * Signatures must match exactly or the SDK object files will fail to link.
 * ------------------------------------------------------------------------ */

void deca_sleep(unsigned int time_ms)
{
    /* 【M-1】pdMS_TO_TICKS(time_ms) だけだと、呼び出しタイミングが
     * tick境界の直前だった場合に実待ち時間が最短 (time_ms-1) tick まで
     * 短くなりうる（vTaskDelay() は「次のtick境界までの端数」を
     * 切り捨てるため）。SDK内部（例: dw3720_device.c の
     * ull_softreset() 等、「DW3720 needs 1.5ms to initialise」コメント
     * 付近）は deca_sleep(2) のような短い待ちを多用しており、切り捨てが
     * 起きるとチップ側が要求する最短待ち時間を割り込む恐れがある。
     * +1 して「実待ちは必ず time_ms 以上になる」側へ丸める
     * (docs/REVIEW_2026-08-21.md §1 M-1)。ファイル冒頭の #error により
     * CONFIG_FREERTOS_HZ>=1000（1 tick <= 1ms）が保証されているので、
     * この+1による超過分は高々1ms未満に収まる。 */
    vTaskDelay(pdMS_TO_TICKS(time_ms) + 1);
}

void deca_usleep(unsigned long time_us)
{
    esp_rom_delay_us((uint32_t)time_us);
}

/* ------------------------------------------------------------------------
 * decamutexon() / decamutexoff(): UWB IC の IRQ 線の禁止/復元
 *
 * (a) 契約: Qorvo SDK は「decamutexon/off は UWB IC の IRQ 線だけを禁止/
 *     復元するためのもの」と規定している（
 *     components/qm33120w_sdk/deca_device_api.h:3220-3223「at a minimum
 *     those interrupts coming from the Decawave device should be
 *     disabled/re-enabled by this activity」、:3231-3253
 *     decaIrqStatus_t 定義・decamutexon()/decamutexoff() 宣言）。公式
 *     nRF52840-DK 実装（docs/refs/qorvo_api/DW3XXX_API_rev9p3/API/
 *     Build_Platforms/nRF52840-DK/Source/platform/deca_mutex.c:49-82）も
 *     nrf_drv_gpiote_in_event_disable/enable(current_irq_pin) だけを行い、
 *     全割り込み禁止の critical section は使っていない。
 *
 * (b) 旧実装（portENTER_CRITICAL(&g_deca_mutex)）が不正だった理由: SDK は
 *     この区間内で SPI 転送（ブロッキング API）と ESP_LOGE を呼ぶ
 *     （dw3720_device.c:5443-5473 ull_setinterrupt、レジスタ
 *     read-modify-write を6回以上、:5832-5837 ull_forcetrxoff →
 *     dwt_writefastCMD。後者は stopRadioAndClearRxStatus() 経由で
 *     受信タイムアウトのたびに通る通常経路）。critical section 内で
 *     spi_device_acquire_bus(..., portMAX_DELAY)（uwb_spi_xfer() 参照）が
 *     ブロックすると、バス共有時（StampFly: BMI270/PMW3901 と
 *     SPI2_HOST を共有）に割り込み禁止のままコンテキストスイッチ不能に
 *     なりハングする。エラー経路の ESP_LOGE（stdout ロックを取る）も
 *     同様に不正。
 *
 * (c) 本ポートの ISR（uwb_irq_isr_handler、本ファイル下方の IRQ 節）は
 *     xSemaphoreGiveFromISR() だけを行い SPI やログを一切呼ばないため
 *     （同節冒頭の「IRQ ("wakeup signal") support」コメント
 *     参照）、そもそも decamutexon/off で ISR 実行を止める必要は無い。
 *     以下の「UWB IC の IRQ 線だけを禁止」実装は SDK の契約を満たすための
 *     保守的措置であり、実害の有無に関わらず入れておく。
 *
 * s_irq_active が false（IRQ 未使用、または pin_irq 未配線で
 * gpio_isr_handler_add() 自体が呼ばれていない）ときは gpio_intr_disable/
 * enable() を呼ばず 0 を返す。decamutexoff() の gpio_intr_enable() は
 * uwb_port_irq_enable() が gpio_isr_handler_add() で有効化したのと同じ
 * コアで有効化される: ESP-IDF v5.5.2 の gpio_intr_enable() は
 * gpio_context.isr_core_id
 * (~/esp/esp-idf/components/esp_driver_gpio/src/gpio.c:174-182) を使って
 * enable するコアを決め、この isr_core_id は gpio_install_isr_service() が
 * 最初に呼ばれたコアで確定する（同 gpio.c:624-625）。
 * uwb_port_irq_enable() 内の gpio_isr_handler_add()（本ファイル:407 相当）
 * も esp_intr_get_cpu() で同じコアに対し enable するため（同
 * gpio.c:555-566）、decamutexoff() の gpio_intr_enable() と整合する。
 * ------------------------------------------------------------------------ */
decaIrqStatus_t decamutexon(void)
{
    if (s_irq_active) {
        gpio_intr_disable(s_cfg.pin_irq);
        return 1;
    }
    return 0;
}

void decamutexoff(decaIrqStatus_t s)
{
    if (s) {
        gpio_intr_enable(s_cfg.pin_irq);
    }
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

/* uwb_port_spi_use_fast_rate() (uwb_port.h) が実体で、こちらは struct
 * dwt_spi_s::setslowrate/setfastrate コールバックからそれを呼ぶだけの
 * 薄いラッパ。SDK 内でこのコールバックが実際に呼ばれる経路は
 * dw3720_device.c の非標準 I/F init()/_init_no_chan() のみで、
 * dwt_initialise()（deca_compat.c）からは到達しない
 * （docs/REVIEW_2026-08-21.md §0 #2）。標準の dwt_probe()+dwt_initialise()
 * 経路（uwb_qm33120.cpp の Qm33120::begin()）を使う場合は
 * uwb_port_spi_use_fast_rate() を明示的に呼ぶ必要がある。 */
static void uwb_spi_setslowrate(void)
{
    uwb_port_spi_use_fast_rate(false);
}

static void uwb_spi_setfastrate(void)
{
    uwb_port_spi_use_fast_rate(true);
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

void uwb_port_spi_use_fast_rate(bool fast)
{
    if (fast) {
        if (s_spi_fast != NULL) {
            s_spi_active = s_spi_fast;
        }
    } else {
        if (s_spi_slow != NULL) {
            s_spi_active = s_spi_slow;
        }
    }
}

uint32_t uwb_port_spi_active_hz(void)
{
    if (!s_initialized || (s_spi_active == NULL)) {
        return 0;
    }
    return (s_spi_active == s_spi_fast) ? s_cfg.spi_fast_hz : s_cfg.spi_slow_hz;
}

/* ------------------------------------------------------------------------
 * GPIO / reset / wakeup
 * ------------------------------------------------------------------------ */

void uwb_port_hard_reset(uint32_t reset_low_ms, uint32_t startup_ms)
{
    /* 【M-4】uwb_port_init() 前は s_cfg がゼロ初期化のままで pin_rst==0
     * (有効な GPIO0 = strapping / ボタン) に化けるため、UNUSED 判定だけでは
     * 防げない。uwb_port_irq_enable() と同じく s_initialized も見る。 */
    if (!s_initialized || (s_cfg.pin_rst == UWB_PORT_PIN_UNUSED)) {
        vTaskDelay(pdMS_TO_TICKS(startup_ms));
        return;
    }

    /* 【M-5】RSTn はチップ内部の POR が H に駆動し、外部からは Low に引く
     * だけでよい（QM33120W DS「Must not be pulled high by the external
     * source」）。uwb_port_init() で open-drain 出力 + level 1 (= Hi-Z) に
     * 固定してあるので、ここでは level を 0 → 1 と書くだけ。push-pull で
     * 一瞬 H を駆動する旧来の順序（OUTPUT にしてから 0 を書く）を避ける。
     * 公式 STM 実装 (port.c) も GPIO_MODE_OUTPUT_OD。 */
    gpio_set_level(s_cfg.pin_rst, 0);
    vTaskDelay(pdMS_TO_TICKS(reset_low_ms));
    gpio_set_level(s_cfg.pin_rst, 1); /* open-drain: release = Hi-Z */
    vTaskDelay(pdMS_TO_TICKS(startup_ms));
}

void uwb_port_set_wakeup(bool level)
{
    if (!s_initialized || (s_cfg.pin_wakeup == UWB_PORT_PIN_UNUSED)) { /* 【M-4】 */
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
    if (!s_initialized) { /* 【M-4】未初期化なら GPIO0 を叩かない */
        return;
    }
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
 * IRQ ("wakeup signal") support (docs/IRQ_POLICY.md)
 *
 * IRQ は「起床信号」としてのみ使う。ステータスレジスタの判読は行わない
 * （それは呼び出し側 uwb_qm33120 が従来のポーリング経路と共通のコードで
 * 行う）。ISR は xSemaphoreGiveFromISR() だけを行い、SPI やログは一切
 * 呼ばない（ISR内でSPI転送やESP_LOGxを呼ぶとブロッキングや再入の危険が
 * あるため）。
 * ------------------------------------------------------------------------ */

static void IRAM_ATTR uwb_irq_isr_handler(void *arg)
{
    (void)arg;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (s_irq_sem != NULL) {
        xSemaphoreGiveFromISR(s_irq_sem, &xHigherPriorityTaskWoken);
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

esp_err_t uwb_port_irq_enable(void)
{
    if (!s_initialized || (s_cfg.pin_irq == UWB_PORT_PIN_UNUSED)) {
        /* uwb_port_init() 未実行時は s_cfg が既定(0クリア)のままで
         * pin_irq==0 (有効なGPIO番号) に化けてしまうため、s_initialized も
         * 併せて見る。UWB_PORT_PIN_UNUSED(-1)の判定だけでは防げない。 */
        return ESP_ERR_NOT_SUPPORTED;
    }
    if (s_irq_active) {
        /* 既に有効化済み。冪等に成功を返す。 */
        return ESP_OK;
    }

    if (s_irq_sem == NULL) {
        s_irq_sem = xSemaphoreCreateBinary();
        if (s_irq_sem == NULL) {
            ESP_LOGE(TAG, "irq_enable: xSemaphoreCreateBinary failed");
            return ESP_ERR_NO_MEM;
        }
    }

    /* --- GPIO割り込み設定ここから ---
     * intr_type = GPIO_INTR_POSEDGE (立ち上がりエッジ = アクティブHIGH前提):
     * DW3720のIRQはアクティブHIGHという記述が一次資料にある
     * (components/qm33120w_sdk/deca_device_api.h:2454 "The IRQ line has to
     * be low/inactive (i.e., no pending events) otherwise device will not
     * enter sleep" -- sleep可能条件としてIRQ=low=inactiveと明記されており、
     * これはactive=highを意味する)。
     * 【注意】この極性は実機でまだ検証していない。誤っていた場合の帰結は
     * 「常にタイムアウト待ちになる」または「即座にIRQで起床し続ける」
     * （後者は各待ちループの通常のステータス判定が不一致フレームとして
     * 処理し続けるだけで、実害はvTaskDelay(1)相当に留まる想定）。
     * pull_down_en = GPIO_PULLDOWN_DISABLE (2026-08-21訂正): 公式回路図
     * (assets/SCH_UWB_MODULE_SCH_main_V0.2_...pdf、docs/WIRING.md
     * §5.5(4)) で、M5Stamp UWB Module上にDW_IRQをVCC_3V3へプルアップする
     * 抵抗R2(10kΩ)が実装済みであることが判明した。ESP32-S3の内部プルダウン
     * (概算45kΩ程度)はこの10kΩに負けるため、有効化してもIRQピンをLow側へ
     * 倒す効果はほぼ無く、3.3Vを(10k+45k)で分圧した約60µAのリーク電流を
     * 常時流すだけになる。無効化しても実害はない。
     * 極性判定の既定Low（未配線時のフロート対策）は、モジュール側の外付け
     * プルアップに対抗できないため内部プルダウンでは実現できない。未配線
     * 時のフロート対策は、pin_irqをUWB_PORT_PIN_UNUSEDにする運用
     * （uwb_port_irq_enable()はs_cfg.pin_irq==UWB_PORT_PIN_UNUSEDなら
     * ESP_ERR_NOT_SUPPORTEDを返してGPIO設定自体を行わない）に委ねる。 */
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << (unsigned int)s_cfg.pin_irq),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_POSEDGE,
    };
    esp_err_t err = gpio_config(&io_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "irq_enable: gpio_config failed: %s", esp_err_to_name(err));
        return err;
    }

    /* アプリ側が既に gpio_install_isr_service() を呼んでいる場合がある。
     * ESP_ERR_INVALID_STATE は「既にインストール済み」を意味するので
     * 成功として扱う。 */
    err = gpio_install_isr_service(0);
    if ((err != ESP_OK) && (err != ESP_ERR_INVALID_STATE)) {
        ESP_LOGE(TAG, "irq_enable: gpio_install_isr_service failed: %s", esp_err_to_name(err));
        return err;
    }

    err = gpio_isr_handler_add(s_cfg.pin_irq, uwb_irq_isr_handler, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "irq_enable: gpio_isr_handler_add failed: %s", esp_err_to_name(err));
        return err;
    }

    s_irq_active = true;
    return ESP_OK;
}

esp_err_t uwb_port_irq_disable(void)
{
    if (!s_irq_active) {
        return ESP_OK;
    }

    gpio_isr_handler_remove(s_cfg.pin_irq);
    /* gpio_uninstall_isr_service() はここでは呼ばない: ISRサービスは
     * プロセス全体で共有されるリソースであり、アプリ側や他コンポーネントが
     * 同じサービスに別のハンドラを登録している可能性がある
     * （uwb_port_irq_enable() のコメント参照）。個別ハンドラの登録解除
     * (gpio_isr_handler_remove、上の呼び出し)だけを行い、サービス自体の
     * 生死には関与しない。 */
    s_irq_active = false;

    if (s_irq_sem != NULL) {
        vSemaphoreDelete(s_irq_sem);
        s_irq_sem = NULL;
    }
    return ESP_OK;
}

bool uwb_port_irq_available(void)
{
    return s_irq_active;
}

void uwb_port_irq_clear_pending(void)
{
    if (!s_irq_active || (s_irq_sem == NULL)) {
        return;
    }
    (void)xSemaphoreTake(s_irq_sem, 0);
}

bool uwb_port_irq_wait(uint32_t timeout_ms)
{
    if (!s_irq_active || (s_irq_sem == NULL)) {
        /* IRQが使えない: vTaskDelay(pdMS_TO_TICKS(timeout_ms))と完全に
         * 等価に振る舞う(docs/IRQ_POLICY.md 実装要件1)。 */
        vTaskDelay(pdMS_TO_TICKS(timeout_ms));
        return false;
    }
    return xSemaphoreTake(s_irq_sem, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
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
    gpio_set_level(s_cfg.pin_cs, 1); /* idle high。OE を立てる前に level を書く (【M-5】CS の Low グリッチ回避) */
    gpio_set_direction(s_cfg.pin_cs, GPIO_MODE_OUTPUT);

    if (s_cfg.pin_rst != UWB_PORT_PIN_UNUSED) {
        /* 【M-5】open-drain 出力 + level 1 = Hi-Z。内部プルアップ/ダウンも切る
         * （RSTn は POR が H に駆動。外部から H に引いてはいけない）。リセット
         * パルスは uwb_port_hard_reset() が level 0 → 1 で出す。 */
        gpio_reset_pin(s_cfg.pin_rst);
        gpio_set_pull_mode(s_cfg.pin_rst, GPIO_FLOATING);
        gpio_set_level(s_cfg.pin_rst, 1);
        gpio_set_direction(s_cfg.pin_rst, GPIO_MODE_OUTPUT_OD);
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

    /* IRQが有効化されたままdeinitされることのないよう、必ず先に無効化する
     * (要件5)。s_cfg.pin_irqを使い終わる前(s_initializedをfalseにする前)に
     * 呼ぶ必要がある。 */
    uwb_port_irq_disable();

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
