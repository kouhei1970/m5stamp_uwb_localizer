idf_component_register(
    SRCS
        "main.cpp"
    INCLUDE_DIRS
        "."
        # Repo root, three levels up from firmware/anchor/main/, so that
        # #include "boards/stamps3.h" / "boards/atoms3.h" resolves (same
        # convention as firmware/twr, firmware/probe, firmware/devtest).
        "../../.."
    REQUIRES
        uwb_qm33120
        uwb_port
        qm33120w_sdk
)
