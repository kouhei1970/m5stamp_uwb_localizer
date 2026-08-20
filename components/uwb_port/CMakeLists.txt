idf_component_register(
    SRCS
        "src/uwb_port.c"
    INCLUDE_DIRS
        "include"
    REQUIRES
        # "driver" must be PUBLIC (not PRIV_REQUIRES): the public header
        # include/uwb_port.h itself includes driver/spi_master.h (needed for
        # spi_host_device_t in uwb_port_config_t), so any component that
        # includes uwb_port.h transitively needs driver's include paths too.
        "driver"
    PRIV_REQUIRES
        # Only uwb_port.c needs the Qorvo SDK's struct dwt_spi_s definition
        # and deca_device_api.h declarations (deca_sleep/deca_usleep/
        # decamutexon/decamutexoff, DWT_SUCCESS/DWT_ERROR, ...).
        "qm33120w_sdk"
        # esp_rom_delay_us() for deca_usleep().
        "esp_rom"
        # heap_caps_malloc()/heap_caps_free() for the DMA scratch buffers.
        "heap"
)
