/**
 * @file uwb_qm33120_impl.hpp
 * @brief Definition of Qm33120::Impl, shared by uwb_qm33120's two
 * translation units.
 *
 * NOT a public header (lives under src/, not include/), same status as
 * uwb_qm33120_internal.hpp.
 *
 * The original third_party/M5Stamp-UWB/src/M5Stamp_UWB.cpp is a single file,
 * so `struct M5Stamp_UWB::Impl` (cpp:43-54) could be defined directly above
 * the member functions that use it. This port splits Qm33120 across two
 * translation units by design (src/uwb_qm33120.cpp for Phase 2 Step 1 -
 * device/PHY/frame I/O - and src/uwb_qm33120_twr.cpp for Phase 2 Step 2 -
 * TWR), and Step 2's requestRange()/respondRange()/requestDSRange()/
 * respondDSRange() need direct access to the same private state
 * (_impl->initialized, _impl->tx_sequence, _impl->tx_antenna_delay, exactly
 * like the original's member functions do) - Impl being merely
 * forward-declared in uwb_qm33120.hpp (`struct Impl;`) is not enough for
 * that from a second .cpp file.
 *
 * This header is therefore the one deliberate change to
 * uwb_qm33120.cpp that Phase 2 Step 2 makes to the Step 1 file: the
 * `struct Qm33120::Impl { ... };` body that used to be defined inline in
 * uwb_qm33120.cpp (right before `Qm33120::Qm33120()`) was moved here
 * unchanged, and uwb_qm33120.cpp now `#include`s this header instead. Field
 * set/defaults/comments are otherwise identical to that original inline
 * definition. See PROGRESS.md Phase 2 Step 2 for the rationale.
 */
#pragma once

#include "uwb_qm33120.hpp"
#include "uwb_qm33120_types.hpp"

namespace uwb {

/* cpp:43-54 相当のPImpl。SPISettings/dwt_spi_s メンバは無い
 * (uwb_port_spi() が返す単一インスタンスを直接使うため、instance毎に
 * 保持する必要が無い)。port_owned は原本に無い新規メンバ
 * （Config::port_already_initialized の実行時ミラー、uwb_port_deinit()の
 * 要否判定に使う）。 */
struct Qm33120::Impl {
    Config config;
    bool connected            = false;
    bool initialized          = false;
    bool port_owned           = false;
    uint32_t device_id        = 0;
    uint8_t tx_sequence       = 0;
    uint16_t tx_antenna_delay = 16385;
    Error last_error           = Error::Ok;
};

} // namespace uwb
