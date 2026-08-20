/**
 * @file uwb_port.h
 * @brief ESP-IDF platform abstraction layer (SPI transport, GPIO, timing, mutex)
 *        for the vendored Qorvo QM33120W/DW3720 (qm33120w_sdk) driver.
 *
 * -----------------------------------------------------------------------
 * Phase 1 時点ではシングルタスク利用を前提とする (Phase 1: single-task use only)
 * -----------------------------------------------------------------------
 * The SPI DMA scratch buffer(s) used by this component are shared module-level
 * state. They are NOT allocated per call and are NOT protected by a mutex
 * against concurrent callers. Calling any uwb_port_* function, or any Qorvo
 * SDK function that goes through the struct dwt_spi_s returned by
 * uwb_port_spi(), from more than one FreeRTOS task concurrently is undefined
 * behavior.
 *
 * これは Phase 1 における意図的なスコープ制限である。複数タスクからの利用が
 * 必要になった場合は、この制限を見直すこと (revisit if/when multi-task use
 * is needed).
 * -----------------------------------------------------------------------
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/spi_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declaration only: consumers of uwb_port.h alone do not need the
 * qm33120w_sdk include paths. Only uwb_port.c includes the real SDK header
 * (deca_interface.h) which defines the full struct dwt_spi_s. */
struct dwt_spi_s;

/** -1 (unused) is allowed for pin_rst, pin_irq, pin_wakeup, pin_gp7. */
#define UWB_PORT_PIN_UNUSED (-1)

typedef struct {
    spi_host_device_t spi_host; /**< default SPI2_HOST */
    int pin_sck;
    int pin_mosi;
    int pin_miso;
    int pin_cs; /**< ソフトウェア制御CS（GPIO直接駆動、ハードウェアspics_io_numは使わない）。転送中は spi_device_acquire_bus() でバスを占有してから駆動するため、バス共有時も安全。 */
    int pin_rst;    /**< UWB_PORT_PIN_UNUSED (-1) allowed */
    int pin_irq;    /**< UWB_PORT_PIN_UNUSED (-1) allowed */
    int pin_wakeup; /**< UWB_PORT_PIN_UNUSED (-1) allowed */
    int pin_gp7;    /**< UWB_PORT_PIN_UNUSED (-1) allowed */
    uint32_t spi_slow_hz; /**< default 2000000 */
    uint32_t spi_fast_hz; /**< default 16000000 */
    bool init_spi_bus;    /**< false => caller already ran spi_bus_initialize() on spi_host */
} uwb_port_config_t;

/**
 * @brief Initialize the ESP-IDF UWB port: configure GPIOs, (optionally) init
 * the SPI bus, add slow/fast SPI devices, and allocate DMA scratch buffers.
 */
esp_err_t uwb_port_init(const uwb_port_config_t *cfg);

/**
 * @brief Release resources acquired by uwb_port_init() (SPI devices, DMA
 * scratch buffers). Does not call spi_bus_free() (the bus may be shared).
 */
esp_err_t uwb_port_deinit(void);

/**
 * @brief Pulse the RST pin low then release it (tri-state / input), matching
 * the DW3720 hardware reset sequence. If pin_rst is UWB_PORT_PIN_UNUSED, this
 * just delays startup_ms.
 *
 * @param reset_low_ms time to hold RST low, in ms
 * @param startup_ms   time to wait after releasing RST (or when no RST pin), in ms
 */
void uwb_port_hard_reset(uint32_t reset_low_ms, uint32_t startup_ms);

/**
 * @brief Directly set the level of pin_wakeup (no-op if pin_wakeup ==
 * UWB_PORT_PIN_UNUSED). For manual sleep/wake control by upper layers.
 */
void uwb_port_set_wakeup(bool level);

/**
 * @brief Read the level of pin_gp7.
 * @return 0 or 1, or -1 if pin_gp7 == UWB_PORT_PIN_UNUSED (not wired).
 */
int uwb_port_read_gp7(void);

/**
 * @brief Get the struct dwt_spi_s instance to hand to dwt_probe_s.spi (and,
 * once probed, dwchip_s.SPI).
 */
struct dwt_spi_s *uwb_port_spi(void);

/**
 * @brief Bare wakeup PULSE callback, matching the
 * `void (*wakeup_device_with_io)(void)` signature in struct dwt_probe_s /
 * struct dwchip_s. This is distinct from uwb_port_set_wakeup(bool): that
 * function just sets a level for manual control, while this one performs the
 * timed pulse sequence used to wake the DW3720 out of sleep during probing
 * (HIGH 2ms / LOW 5ms on pin_wakeup if wired, otherwise a CS LOW 2ms / HIGH
 * 5ms fallback pulse on pin_cs).
 *
 * Wire this up as: probe_interface.wakeup_device_with_io = uwb_port_wakeup_device_with_io;
 */
void uwb_port_wakeup_device_with_io(void);

#ifdef __cplusplus
}
#endif
