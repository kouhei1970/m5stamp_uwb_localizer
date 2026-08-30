/**
 * @file uwb_qm33120_phy_kconfig.hpp
 * @brief Shared PHY Kconfig (docs/ARCHITECTURE_V2.md §4) -> uwb::PhyConfig.
 *
 * `phyConfigFromKconfig()` reads the `UWB_PHY_*` options declared in
 * components/uwb_qm33120/Kconfig (auto-included by ESP-IDF under
 * "Component config" for any firmware that REQUIRES uwb_qm33120) and builds
 * a uwb::PhyConfig from them, so firmware/anchor and firmware/tag can pick
 * data rate / preamble length / TX power / PGdelay from one shared menu
 * instead of each hard-coding `uwb::PhyConfig phy;` (the struct default).
 *
 * This header (unlike uwb_qm33120_responder_fsm.hpp) is intentionally
 * ESP-IDF-dependent: it is only ever called from firmware app code (which
 * already links ESP-IDF and has sdkconfig.h available), not from
 * tests/host/. It is declared separately from uwb_qm33120.hpp (rather than
 * as a Qm33120 member) because it does not need an instance - it is a pure
 * Kconfig -> struct conversion, callable before Qm33120::begin().
 *
 * `Qm33120::forcePllCoarse()` / `Qm33120::logPhy()` (declared in
 * uwb_qm33120.hpp, §4's other two asks: moving firmware/twr's
 * DIAG_PLL_COARSE_CH9 procedure and its "phy: ..." boot log line into the
 * driver so tag/anchor/twr can share them) are implemented in the same
 * translation unit as this function (src/uwb_qm33120_phy_kconfig.cpp) since
 * all three exist to serve the same §4 use case, but they are declared on
 * the Qm33120 class itself (not here) because they need Qm33120::Impl state
 * (forcePllCoarse() operates on "whichever chip this Qm33120 instance owns";
 * logPhy() reads back the PhyConfig init() actually applied,
 * Impl::applied_phy).
 */
#pragma once

#include "uwb_qm33120_types.hpp"

namespace uwb {

/**
 * @brief Build a uwb::PhyConfig from the UWB_PHY_* Kconfig options
 * (components/uwb_qm33120/Kconfig, docs/ARCHITECTURE_V2.md §4).
 *
 * Always sets channel = Channel9 (the M5Stamp UWB Module is ch9-only; see
 * the "uwb-module-ch9-only" project note) and leaves every PhyConfig field
 * not covered by §4 (txPreambleCode/rxPreambleCode/sfdType/stsMode/pdoaMode/
 * txAntennaDelay/rxAntennaDelay/enableLnaPa/...) at PhyConfig's own struct
 * default.
 *
 * `sfdTimeout` is left at 0 (automatic - see PhyConfig::sfdTimeout, R8):
 * uwb_qm33120.cpp's makeSfdTimeout() recomputes it from
 * preambleLength/sfdType/pacSize on every init(), so it always tracks
 * whatever UWB_PHY_PREAMBLE_LEN selects here.
 *
 * If CONFIG_UWB_PHY_PREAMBLE_LEN is not one of 64/128/256/512/1024 (Kconfig's
 * `int` type cannot enforce a discrete set - see the Kconfig help text), this
 * logs an error and falls back to 128/Pac8 rather than producing an
 * inconsistent PhyConfig.
 */
PhyConfig phyConfigFromKconfig();

} // namespace uwb
