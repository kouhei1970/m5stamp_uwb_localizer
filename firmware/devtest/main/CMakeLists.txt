idf_component_register(
    SRCS
        "main.cpp"
    INCLUDE_DIRS
        "."
        # Repo root, three levels up from firmware/devtest/main/, so that
        # #include "boards/stamps3.h" / "boards/stampfly.h"
        # resolves (same convention as firmware/probe/main/CMakeLists.txt).
        "../../.."
    REQUIRES
        # 役割別ハートビート LED (タグ=緑 / アンカー=赤)
        # Role heartbeat LED (tag = green, anchor = red)
        uwb_status_led
        uwb_qm33120
        uwb_port
        qm33120w_sdk
)
