/**
 * @file status_led.h
 * @brief On-board WS2812/SK6812 RGB LED heartbeat, so that a board running
 *        without a serial monitor attached still shows whether it is alive.
 *
 * シリアルモニタを繋いでいないときでも「基板が起動して動いているか」を
 * 目視で確認するための、内蔵フルカラー LED によるハートビート表示。
 *
 * ESP-IDF 内蔵の RMT ドライバ (esp_driver_rmt) だけで WS2812 の 1-Wire
 * 波形を作る。外部コンポーネント (espressif/led_strip) のダウンロードは不要。
 */
#pragma once

#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the RMT channel driving the on-board RGB LED.
 *        内蔵フルカラー LED を駆動する RMT チャネルを初期化する。
 *
 * @param gpio_num GPIO the LED data line is on (M5StampS3A: 21)
 *                 LED のデータ線が繋がっている GPIO（M5StampS3A は 21）
 */
esp_err_t status_led_init(int gpio_num);

/**
 * @brief Set the LED colour immediately (0,0,0 turns it off).
 *        LED の色を即座に設定する（0,0,0 で消灯）。
 */
esp_err_t status_led_set(uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief Start a background task blinking the given colour forever.
 *        指定色で点滅し続ける背景タスクを起動する。
 *
 * Calling it a second time replaces the colour/period of the running task
 * rather than creating another one.
 * 2 回目以降の呼び出しはタスクを増やさず、色と周期を差し替える。
 */
esp_err_t status_led_start_blink(uint8_t r, uint8_t g, uint8_t b, uint32_t on_ms, uint32_t off_ms);

#ifdef __cplusplus
}
#endif
