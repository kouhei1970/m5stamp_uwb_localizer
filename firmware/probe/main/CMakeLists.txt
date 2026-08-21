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
        uwb_port
        qm33120w_sdk
)
