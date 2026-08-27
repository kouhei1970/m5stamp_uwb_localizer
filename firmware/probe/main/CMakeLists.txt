idf_component_register(
    SRCS
        "main.c"
        "status_led.c"
    INCLUDE_DIRS
        "."
        # Repo root, three levels up from firmware/probe/main/, so that
        # #include "boards/stamps3.h" / "boards/atoms3.h" / "boards/stampfly.h"
        # resolves.
        "../../.."
    REQUIRES
        uwb_port
        qm33120w_sdk
        # 内蔵フルカラー LED のハートビート表示 (status_led.c) が使う
        # ESP-IDF 内蔵 RMT ドライバ / built-in RMT driver used by status_led.c
        esp_driver_rmt
)
