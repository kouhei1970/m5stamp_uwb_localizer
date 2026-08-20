idf_component_register(
    SRCS
        "src/uwb_qm33120.cpp"
        # Phase 2 Step 2 (TWR): requestRange/respondRange/requestDSRange/
        # respondDSRange, defined as a second translation unit of the same
        # Qm33120 class (see src/uwb_qm33120_twr.cpp header comment).
        "src/uwb_qm33120_twr.cpp"
    INCLUDE_DIRS
        "include"
    REQUIRES
        # Public headers (uwb_qm33120_types.hpp) include uwb_port.h (SPI/GPIO
        # config passed through to uwb_port_init()) and directly embed
        # spi_host_device_t (driver/spi_master.h), so both must be public
        # REQUIRES, not PRIV_REQUIRES.
        "qm33120w_sdk"
        "uwb_port"
        "driver"
        # esp_timer_get_time() for the millis()-equivalent timeout helper
        # (uwb_qm33120_internal.hpp).
        "esp_timer"
)
