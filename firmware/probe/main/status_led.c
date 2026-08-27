/**
 * @file status_led.c
 * @brief WS2812/SK6812 heartbeat LED over the ESP-IDF built-in RMT TX
 *        driver. See status_led.h for the rationale.
 *
 * WS2812 系 LED をハートビート表示に使う実装。意図は status_led.h を参照。
 */
#include "status_led.h"

#include "driver/rmt_encoder.h"
#include "driver/rmt_tx.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "status_led";

/* WS2812 bit waveform at a 10 MHz resolution (1 tick = 0.1 us):
 *   T0H = 0.3us / T0L = 0.9us, T1H = 0.6us / T1L = 0.6us.
 * These are the values common to WS2812B and SK6812; both parts tolerate
 * +-150ns, so the same table drives either.
 *
 * WS2812 のビット波形。分解能 10MHz なので 1 tick = 0.1us。
 * WS2812B / SK6812 の共通値（どちらも +-150ns の許容がある）。 */
#define LED_RMT_RESOLUTION_HZ 10000000u
#define LED_T0H_TICKS         3
#define LED_T0L_TICKS         9
#define LED_T1H_TICKS         6
#define LED_T1L_TICKS         6

/* Task that drives the blink. 点滅を駆動するタスク。 */
#define LED_TASK_STACK_BYTES 2560
#define LED_TASK_PRIORITY    1

static rmt_channel_handle_t s_chan    = NULL;
static rmt_encoder_handle_t s_encoder = NULL;
static TaskHandle_t         s_task    = NULL;

/* Blink parameters, read by the task and rewritten by
 * status_led_start_blink(). Each field is written atomically on a 32-bit
 * core and the task only ever reads them, so no lock is needed - the worst
 * case is one blink cycle using a mix of the old and new values.
 *
 * 点滅パラメータ。タスクは読むだけ、書き換えは status_led_start_blink()
 * だけなので、ロックは要らない（最悪でも1周期だけ新旧が混ざる）。 */
static volatile uint8_t  s_r = 0, s_g = 0, s_b = 0;
static volatile uint32_t s_on_ms = 250, s_off_ms = 250;

esp_err_t status_led_init(int gpio_num)
{
    if (s_chan != NULL) {
        return ESP_OK; /* already initialized / 初期化済み */
    }

    rmt_tx_channel_config_t chan_cfg = {
        .gpio_num          = gpio_num,
        .clk_src           = RMT_CLK_SRC_DEFAULT,
        .resolution_hz     = LED_RMT_RESOLUTION_HZ,
        .mem_block_symbols = 64,
        .trans_queue_depth = 4,
    };
    esp_err_t err = rmt_new_tx_channel(&chan_cfg, &s_chan);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "rmt_new_tx_channel failed: %s", esp_err_to_name(err));
        s_chan = NULL;
        return err;
    }

    /* MSB first: WS2812 takes each colour byte most-significant bit first.
     * WS2812 は各色バイトを MSB から受け取る。 */
    rmt_bytes_encoder_config_t bytes_cfg = {
        .bit0 = {.level0 = 1, .duration0 = LED_T0H_TICKS, .level1 = 0, .duration1 = LED_T0L_TICKS},
        .bit1 = {.level0 = 1, .duration0 = LED_T1H_TICKS, .level1 = 0, .duration1 = LED_T1L_TICKS},
        .flags = {.msb_first = 1},
    };
    err = rmt_new_bytes_encoder(&bytes_cfg, &s_encoder);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "rmt_new_bytes_encoder failed: %s", esp_err_to_name(err));
        (void)rmt_del_channel(s_chan);
        s_chan = NULL;
        return err;
    }

    err = rmt_enable(s_chan);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "rmt_enable failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "on-board RGB LED on GPIO %d", gpio_num);
    return ESP_OK;
}

esp_err_t status_led_set(uint8_t r, uint8_t g, uint8_t b)
{
    if ((s_chan == NULL) || (s_encoder == NULL)) {
        return ESP_ERR_INVALID_STATE;
    }

    /* WS2812 wire order is G, R, B - not R, G, B.
     * WS2812 の線上の順序は G, R, B（R, G, B ではない）。 */
    const uint8_t grb[3] = {g, r, b};

    rmt_transmit_config_t tx_cfg = {.loop_count = 0};
    esp_err_t err = rmt_transmit(s_chan, s_encoder, grb, sizeof(grb), &tx_cfg);
    if (err != ESP_OK) {
        return err;
    }
    /* Waiting keeps the >50us reset gap before the next frame implicit in
     * the blink period, and bounds how long the encoder buffer is in use.
     * 送信完了を待つ。次フレームまでの 50us 以上のリセット間隔は
     * 点滅周期そのものが確保する。 */
    return rmt_tx_wait_all_done(s_chan, pdMS_TO_TICKS(100));
}

static void status_led_blink_task(void *arg)
{
    (void)arg;
    for (;;) {
        (void)status_led_set(s_r, s_g, s_b);
        vTaskDelay(pdMS_TO_TICKS(s_on_ms));
        (void)status_led_set(0, 0, 0);
        vTaskDelay(pdMS_TO_TICKS(s_off_ms));
    }
}

esp_err_t status_led_start_blink(uint8_t r, uint8_t g, uint8_t b, uint32_t on_ms, uint32_t off_ms)
{
    if (s_chan == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    s_r      = r;
    s_g      = g;
    s_b      = b;
    s_on_ms  = (on_ms == 0) ? 1 : on_ms;
    s_off_ms = (off_ms == 0) ? 1 : off_ms;

    if (s_task != NULL) {
        return ESP_OK; /* task already running, parameters swapped above */
    }

    BaseType_t ok = xTaskCreate(status_led_blink_task, "status_led", LED_TASK_STACK_BYTES, NULL, LED_TASK_PRIORITY,
                                &s_task);
    if (ok != pdPASS) {
        s_task = NULL;
        ESP_LOGE(TAG, "xTaskCreate failed");
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}
