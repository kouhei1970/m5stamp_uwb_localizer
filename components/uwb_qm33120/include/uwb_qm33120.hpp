/**
 * @file uwb_qm33120.hpp
 * @brief ESP-IDF port of the M5Stamp_UWB Arduino wrapper class, renamed to
 * uwb::Qm33120.
 *
 * Ported from third_party/M5Stamp-UWB/src/M5Stamp_UWB.h (M5Stack Technology
 * CO LTD, MIT). Class shape (PImpl, method list) is kept 1:1 with the
 * original where possible; see uwb_qm33120.cpp for the per-method mapping
 * and docs/archive/PROGRESS.md Phase 2 Step 1 for the list of intentional deviations
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
 * a Qorvo QM33120W/DW3720 (M5Stamp UWB Module) module via components/uwb_port.
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

    /**
     * @brief config.use_irq が有効で、かつ実際に uwb_port_irq_enable() +
     * dwt_setinterrupt() が成功した状態かどうか。false の場合、待ちループは
     * ポーリング（vTaskDelay(1)相当）で動作している（docs/IRQ_POLICY.md）。
     */
    bool irqActive() const;

    /**
     * @brief Force the DW3720's channel-9 PLL VCO coarse-tune code to `code`
     * and re-lock the PLL on it (docs/ARCHITECTURE_V2.md §4). Shared driver
     * version of firmware/twr's `diagForcePllCoarseCh9()`
     * (main.cpp, gated by Kconfig UWB_TWR_DIAG_PLL_COARSE_CH9) - same
     * register-level procedure (three escalating raw-register sequences,
     * see uwb_qm33120_phy_kconfig.cpp for the full rationale), parameterized
     * by `code` instead of a fixed Kconfig value so tag/anchor/twr can each
     * decide independently whether/when to call it.
     *
     * Background (measured on real hardware, firmware/twr): a boot at die
     * temperature <=31degC gets PLL_COARSE_CODE's ch9 VCO coarse-tune field
     * (bits[6:0]) = 0x23 (35, the OTP-programmed value) and 38-65% SS-TWR
     * success; a boot at >=32degC gets 0x24 (36) and 10-20% success.
     * Hypothesis: 0x24 leaves the VCO on a sub-band edge (worse phase
     * noise). This function lets a caller force whichever code the cold-boot
     * case gets, at whatever temperature the current boot happened to be at.
     *
     * Must be called after begin()/init() has succeeded (the chip has to be
     * probed, configured and idle). Logs each attempted sequence and a final
     * `"DIAG_PLL_COARSE: done ok=%d coarse_final=0x...` line under the
     * "Qm33120" ESP_LOG tag - deliberately the SAME message text
     * firmware/twr's own copy prints, since scripts grep for it.
     *
     * @param code  Ch9 VCO coarse-tune code to force, 0-127 (bits[6:0] of
     *              PLL_COARSE_CODE's ch9 field; values outside that range are
     *              masked to 7 bits before use, same as the Kconfig `range`).
     * @return true if the PLL reports locked on `code` afterwards (readback
     *         matched and PLL_STATUS's lock flag is set); false if every
     *         escalation failed to lock and the function fell back to the
     *         untouched normal calibration path (radio is left in a working,
     *         though not necessarily forced, state either way - see the .cpp
     *         for the fallback details).
     */
    bool forcePllCoarse(uint8_t code);

    /**
     * @brief Log the PHY configuration init() actually applied
     * (docs/ARCHITECTURE_V2.md §4), in the exact format firmware/twr prints
     * from main.cpp (`"phy: preamble=%u pac=%u rate=%s ch=%u code=%u/%u
     * sfd=%u txpower=0x%08lX pgdelay=0x%02X"`) - scripts grep this line, so
     * the format is reproduced verbatim rather than reinvented.
     *
     * Reads back Impl::applied_phy (the resolved PhyConfig init() stored on
     * success - see resolvePHYConfig() in uwb_qm33120.cpp), not whatever
     * PhyConfig the caller happened to pass to begin()/init(): if only
     * `channel` differed from PhyConfig{}'s defaults, init() substitutes the
     * built-in per-channel recommended profile, so the caller's own local
     * variable can differ from what was actually written to the chip. Calling
     * this before a successful begin()/init() logs PhyConfig{}'s defaults
     * (Impl::applied_phy's own initializer), not a real reading.
     *
     * @param tag  ESP_LOG tag to log under (the caller's own tag, e.g.
     *             "uwb_anchor") - unlike forcePllCoarse(), this line has no
     *             fixed tag requirement, only a fixed message format.
     */
    void logPhy(const char* tag) const;

    /**
     * @brief The TX antenna delay init() applied (Impl::tx_antenna_delay,
     * matching PhyConfig::txAntennaDelay from the last successful init()).
     * New public accessor (docs/ARCHITECTURE_V2.md §2.2/§2.3): the manual
     * antenna-delay addition when building a delayed-TX Response timestamp
     * (uwb_qm33120_twr.cpp respondRange()/respondDSRange() cpp:610/1183 -
     * "遅延送信の起動時刻" vs "アンテナから実際に電波が出る時刻" の差を
     * 埋めるもの, see uwb_qm33120_twr.cpp's file header comment) needs this
     * value, but `uwb::Responder` (uwb_qm33120_responder.cpp) is a separate
     * class from Qm33120 and cannot reach Impl::tx_antenna_delay directly
     * (Impl is a private nested type; Responder is not a friend). Read-only,
     * no equivalent in the original M5Stamp_UWB.
     */
    uint16_t txAntennaDelay() const;

private:
    struct Impl;
    Impl* _impl;

    bool probe();
    void setError(Error error);

    /**
     * @brief IRQ 線が実際にエッジを届けているかを起動時に実測する
     * （docs/IRQ_POLICY.md）。uwb_port_irq_enable() の成功は「ISR を登録
     * できた」ことしか意味せず、未配線・極性誤り・断線を検出できないため。
     * false のとき init() は IRQ を諦め、タイミングプリセットも
     * PollingBoth へ落とす。init() からのみ呼ばれる。
     */
    bool verifyIrqLine();

    static Qm33120* _active;
};

} // namespace uwb
