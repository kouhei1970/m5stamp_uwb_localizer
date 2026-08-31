/**
 * @file uwb_status_led.h
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
esp_err_t uwb_status_led_init(int gpio_num);

/**
 * @brief Set the LED colour immediately (0,0,0 turns it off).
 *        LED の色を即座に設定する（0,0,0 で消灯）。
 */
esp_err_t uwb_status_led_set(uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief Start a background task blinking the given colour forever.
 *        指定色で点滅し続ける背景タスクを起動する。
 *
 * Calling it a second time replaces the colour/period of the running task
 * rather than creating another one.
 * 2 回目以降の呼び出しはタスクを増やさず、色と周期を差し替える。
 */
esp_err_t uwb_status_led_start_blink(uint8_t r, uint8_t g, uint8_t b, uint32_t on_ms, uint32_t off_ms);

/**
 * @brief Which UWB role this board is running, for the heartbeat colour.
 *        ハートビートの色を決めるための、この基板の役割。
 *
 * The colour is how you tell an anchor from a tag once the boards are placed
 * around a room and no serial monitor is attached. The heartbeat only means
 * "powered and running" - it says nothing about whether ranging succeeds.
 * 部屋に設置してシリアルモニタを繋いでいない状態で、アンカーとタグを
 * 見分けるための色分け。ハートビートは「通電して動いている」ことだけを
 * 示し、測距が成立しているかどうかとは無関係。
 */
typedef enum {
    UWB_STATUS_LED_ROLE_NONE = 0,  //!< 役割を持たないファーム (probe / devtest)。琥珀色
    UWB_STATUS_LED_ROLE_TAG,       //!< タグ (initiator)。緑 / green
    UWB_STATUS_LED_ROLE_ANCHOR,    //!< アンカー (responder)。赤 / red
} uwb_status_led_role_t;

/**
 * @brief Initialize the LED and start the heartbeat in this role's colour.
 *        LED を初期化し、その役割の色でハートビートを開始する。
 *
 * Convenience wrapper around uwb_status_led_init() +
 * uwb_status_led_start_blink(). Returns the init error without starting the
 * blink if the LED could not be set up, so a board with no RGB LED simply
 * carries on without one.
 * init と start_blink をまとめただけのもの。LED を初期化できなければ
 * 点滅を始めずにそのエラーを返すので、フルカラー LED を持たない基板でも
 * そのまま動き続ける。
 *
 * @param gpio_num GPIO the LED data line is on / LED のデータ線の GPIO
 * @param role     TAG -> green, ANCHOR -> red, NONE -> amber
 */
esp_err_t uwb_status_led_start_role_heartbeat(int gpio_num, uwb_status_led_role_t role);

#ifdef __cplusplus
}
#endif
