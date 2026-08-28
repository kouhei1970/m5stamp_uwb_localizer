idf_component_register(
    SRCS
        "main.c"
    INCLUDE_DIRS
        "."
        # Repo root, three levels up from firmware/probe/main/, so that
        # #include "boards/stamps3.h" / "boards/atoms3.h" / "boards/stampfly.h"
        # resolves.
        "../../.."
    REQUIRES
        # 役割別ハートビート LED (タグ=緑 / アンカー=赤)
        # Role heartbeat LED (tag = green, anchor = red)
        uwb_status_led
        uwb_port
        qm33120w_sdk
)
