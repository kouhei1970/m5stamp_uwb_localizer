idf_component_register(
    SRCS
        "src/uwb_status_led.c"
    INCLUDE_DIRS
        "include"
    REQUIRES
        # WS2812 の 1-Wire 波形を ESP-IDF 内蔵 RMT ドライバだけで作る。
        # 外部コンポーネント (espressif/led_strip) は使わない。
        # The WS2812 waveform is generated with the built-in RMT driver only.
        esp_driver_rmt
)
