/**
 * @file uwb_qm33120.hpp
 * @brief ESP-IDF port of the M5Stamp_UWB Arduino wrapper class, renamed to
 * uwb::Qm33120.
 *
 * Ported from third_party/M5Stamp-UWB/src/M5Stamp_UWB.h (M5Stack Technology
 * CO LTD, MIT). Class shape (PImpl, method list) is kept 1:1 with the
 * original where possible; see uwb_qm33120.cpp for the per-method mapping
 * and PROGRESS.md Phase 2 Step 1 for the list of intentional deviations
 * (Arduino SPI/GPIO calls replaced by components/uwb_port, static SPI
 * trampoline methods removed because uwb_port already owns that dispatch).
 *
 * Scope (Phase 2 Step 1): device management, PHY configuration, raw frame
 * send/receive. Phase 2 Step 2 adds TWR (requestRange/respondRange/
 * requestDSRange/respondDSRange) as four more methods of this same class,
 * defined in a separate translation unit (src/uwb_qm33120_twr.cpp) - see that
 * file for the per-method mapping to M5Stamp_UWB.cpp.
 */
#pragma once

#include "uwb_qm33120_types.hpp"

namespace uwb {

/**
 * @brief ESP-IDF interface for probing, initializing, and transmitting with
 * a Qorvo QM33120W/DW3720 (M5Stack M5Stamp UWB Module with FPC) module via components/uwb_port.
 */
class Qm33120 {
public:
    Qm33120();
    ~Qm33120();
    Qm33120(const Qm33120&)            = delete;
    Qm33120& operator=(const Qm33120&) = delete;

    /**
     * @brief Initialize the platform port (unless config.port_already_initialized),
     * probe the UWB chip, and apply the PHY configuration.
     */
    bool begin(const Config& config = Config(), const PhyConfig& phy = PhyConfig());

    /**
     * @brief Stop UWB activity and release the active driver instance. Also
     * releases uwb_port (uwb_port_deinit()) if begin() was the one that
     * initialized it (config.port_already_initialized was false).
     */
    void end();

    /**
     * @brief Reconfigure the UWB PHY after begin().
     */
    bool init(const PhyConfig& phy = PhyConfig());

    /**
     * @brief Toggle the hardware RESET pin if one is configured (via
     * uwb_port_hard_reset()).
     */
    void hardReset(uint32_t reset_low_ms = 5, uint32_t startup_ms = 100);

    /**
     * @brief Return the probed Device ID. Returns 0 if the device is not connected.
     */
    uint32_t deviceId() const;

    /**
     * @brief Read DEV_ID directly through SPI without requiring a successful probe.
     */
    uint32_t readRawDeviceId();

    /**
     * @brief Return a short chip name based on the probed Device ID.
     */
    const char* chipName() const;

    /**
     * @brief Send a short-address IEEE 802.15.4 frame with a user payload.
     */
    TxResult sendFrame(const uint8_t* payload, size_t length, const FrameConfig& frame = FrameConfig(),
                        uint32_t timeoutMs = 100);

    /**
     * @brief Convenience overload for sending a C string payload.
     */
    TxResult sendFrame(const char* payload, const FrameConfig& frame = FrameConfig(), uint32_t timeoutMs = 100);

    /**
     * @brief Wait for one short-address IEEE 802.15.4 frame and copy its payload.
     */
    RxResult receiveFrame(uint8_t* payload, size_t payloadSize, uint32_t timeoutMs = 100);

    /**
     * @brief Run one SS-TWR initiator (Tag) exchange: send Poll, wait for
     * Response, compute the range from the embedded timestamps.
     * Phase 2 Step 2 - see src/uwb_qm33120_twr.cpp (M5Stamp_UWB.cpp:785-889).
     */
    RangeResult requestRange(const RangeConfig& range = RangeConfig());

    /**
     * @brief Wait for one SS-TWR Poll and send a timestamped Response.
     * Phase 2 Step 2 - see src/uwb_qm33120_twr.cpp (M5Stamp_UWB.cpp:891-1009).
     */
    ResponderResult respondRange(const RangeConfig& range = RangeConfig());

    /**
     * @brief Run one DS-TWR initiator (Tag) exchange: Poll / Response /
     * Final, then receive the distance computed by the responder.
     * Phase 2 Step 2 - see src/uwb_qm33120_twr.cpp (M5Stamp_UWB.cpp:1010-1161).
     */
    DSRangeResult requestDSRange(const DSRangeConfig& range = DSRangeConfig());

    /**
     * @brief Wait for one DS-TWR Poll, complete the Response/Final exchange,
     * compute the range, and send it back to the initiator.
     * Phase 2 Step 2 - see src/uwb_qm33120_twr.cpp (M5Stamp_UWB.cpp:1163-1377).
     */
    DSResponderResult respondDSRange(const DSRangeConfig& range = DSRangeConfig());

    bool isConnected() const;
    bool isInitialized() const;
    Error lastError() const;
    const char* lastErrorName() const;
    const Config& config() const;

private:
    struct Impl;
    Impl* _impl;

    bool probe();
    void setError(Error error);

    static Qm33120* _active;
};

} // namespace uwb
