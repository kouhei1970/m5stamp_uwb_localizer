idf_component_register(
    SRCS
        "main.cpp"
    INCLUDE_DIRS
        "."
        # Repo root, three levels up from firmware/tag/main/, so that
        # #include "boards/stamps3.h" / "boards/atoms3.h" resolves (same
        # convention as firmware/probe, firmware/devtest, firmware/twr).
        "../../.."
    REQUIRES
        uwb_ranging
        uwb_qm33120
        uwb_port
        qm33120w_sdk
        uwb_loc
        esp_timer
)
