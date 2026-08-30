/**
 * @file uwb_qm33120_phy_kconfig.cpp
 * @brief docs/ARCHITECTURE_V2.md §4: `uwb::phyConfigFromKconfig()`,
 * `Qm33120::forcePllCoarse()`, `Qm33120::logPhy()`.
 *
 * All three exist to let firmware/anchor and firmware/tag share the PHY
 * setup firmware/twr already validated on real hardware (Kconfig-selectable
 * data rate / preamble / TX power / PGdelay, the ch9 PLL coarse-tune
 * workaround, and the boot-time "phy: ..." log line), without copy-pasting
 * firmware/twr/main/main.cpp's DIAG_* logic into each app. See
 * uwb_qm33120_phy_kconfig.hpp for why phyConfigFromKconfig() is a free
 * function (no Qm33120 instance needed) while the other two are Qm33120
 * methods (they need Impl state).
 */
#include "uwb_qm33120_phy_kconfig.hpp"

#include "esp_log.h"

extern "C" {
#include "deca_device_api.h"
#include "deca_private.h" // dwt_readfromdevice()/dwt_writetodevice(): raw register access for forcePllCoarse().
#include "dw3720_deca_regs.h" // PLL_COARSE_CODE_ID/PLL_CAL_ID/PLL_STATUS_ID/SEQ_CTRL_ID/CLK_CTRL_ID/SYS_STATUS_ID + their bit masks, DWT_AUTO_CLKS (via dw3720_deca_vals.h).
}

#include "uwb_qm33120_impl.hpp"

namespace uwb {

namespace {

static const char* const kPhyTag = "Qm33120";

/**
 * @brief Same mapping firmware/twr's CONFIG_UWB_TWR_DIAG_PHY_PREAMBLE ->
 * PacSize switch uses (main.cpp, ~line 2009), extended to preamble=64 (not
 * one of firmware/twr's DIAG values, but a valid uwb::PreambleLength/
 * UWB_PHY_PREAMBLE_LEN choice here): Qorvo's rule is preamble<=128 -> Pac8,
 * 256/512 -> Pac16, 1024 -> Pac32 (docs/ARCHITECTURE_V2.md §4).
 *
 * @param preambleLen  Kconfig value (already range-checked by the caller to
 *                      be one of 64/128/256/512/1024).
 */
PacSize pacSizeForPreamble(uint16_t preambleLen)
{
    switch (preambleLen) {
    case 256:
    case 512:
        return PacSize::Pac16;
    case 1024:
        return PacSize::Pac32;
    case 64:
    case 128:
    default:
        return PacSize::Pac8;
    }
}

/** @brief calReadReg32()/calWriteReg32() 等 - firmware/twr/main/main.cpp の
 * 同名ローカルヘルパと同じ実装（dwt_readfromdevice()/dwt_writetodevice()
 * 経由、リトルエンディアン）。forcePllCoarse() 専用に複製する（twr 側とは
 * 別の翻訳単位のため共有できない。他の static ヘルパ同様、意図的な複製）。 */
uint32_t calReadReg32(uint32_t regFileID)
{
    uint8_t buf[4] = {0, 0, 0, 0};
    dwt_readfromdevice(regFileID, 0, sizeof(buf), buf);
    return ((uint32_t)buf[3] << 24) | ((uint32_t)buf[2] << 16) | ((uint32_t)buf[1] << 8) | (uint32_t)buf[0];
}

void calWriteReg32(uint32_t regFileID, uint32_t value)
{
    uint8_t buf[4] = {(uint8_t)(value & 0xFFU), (uint8_t)((value >> 8) & 0xFFU), (uint8_t)((value >> 16) & 0xFFU),
                       (uint8_t)((value >> 24) & 0xFFU)};
    dwt_writetodevice(regFileID, 0, sizeof(buf), buf);
}

void calWriteReg16(uint32_t regFileID, uint16_t value)
{
    uint8_t buf[2] = {(uint8_t)(value & 0xFFU), (uint8_t)((value >> 8) & 0xFFU)};
    dwt_writetodevice(regFileID, 0, sizeof(buf), buf);
}

uint8_t calReadReg8(uint32_t regFileID)
{
    uint8_t buf[1] = {0};
    dwt_readfromdevice(regFileID, 0, sizeof(buf), buf);
    return buf[0];
}

void calWriteReg8(uint32_t regFileID, uint8_t value)
{
    dwt_writetodevice(regFileID, 0, sizeof(value), &value);
}

/** firmware/twr/main/main.cpp の pacSizeCount() と同じ（ログ表示用）。 */
unsigned pacSizeCount(PacSize pac)
{
    switch (pac) {
    case PacSize::Pac4:
        return 4;
    case PacSize::Pac8:
        return 8;
    case PacSize::Pac16:
        return 16;
    case PacSize::Pac32:
        return 32;
    default:
        return 0;
    }
}

} // namespace

PhyConfig phyConfigFromKconfig()
{
    PhyConfig phy;
    phy.channel = Channel::Channel9; // M5Stamp UWB Module は ch9 専用 (uwb-module-ch9-only)。

#if CONFIG_UWB_PHY_DATA_RATE_850K
    phy.dataRate = DataRate::Rate850K;
#else
    phy.dataRate = DataRate::Rate6M8;
#endif

    switch (CONFIG_UWB_PHY_PREAMBLE_LEN) {
    case 64:
        phy.preambleLength = PreambleLength::Len64;
        break;
    case 128:
        phy.preambleLength = PreambleLength::Len128;
        break;
    case 256:
        phy.preambleLength = PreambleLength::Len256;
        break;
    case 512:
        phy.preambleLength = PreambleLength::Len512;
        break;
    case 1024:
        phy.preambleLength = PreambleLength::Len1024;
        break;
    default:
        // Kconfig の int 型は「64/128/256/512/1024 のいずれか」という離散
        // 制約を表現できない（componets/uwb_qm33120/Kconfig のヘルプ参照）。
        // 想定外の値が sdkconfig に紛れ込んでいた場合、無言で壊れた PHY
        // 設定を使うより、既定 128/PAC8 へ倒して警告する。
        ESP_LOGE(kPhyTag, "UWB_PHY_PREAMBLE_LEN=%d is not one of 64/128/256/512/1024; falling back to 128",
                 (int)CONFIG_UWB_PHY_PREAMBLE_LEN);
        phy.preambleLength = PreambleLength::Len128;
        break;
    }
    phy.pacSize = pacSizeForPreamble(static_cast<uint16_t>(phy.preambleLength));

    phy.txPower = CONFIG_UWB_PHY_TX_POWER;
    phy.pgDelay = static_cast<uint8_t>(CONFIG_UWB_PHY_PG_DELAY);
    // sfdTimeout は 0（自動計算）のまま: uwb_qm33120.cpp の makeSfdTimeout()
    // が preambleLength/sfdType/pacSize から毎回計算し直す（R8）ので、上で
    // preambleLength を変えてもここで固定値を書く必要が無い。

    return phy;
}

bool Qm33120::forcePllCoarse(uint8_t code)
{
    // firmware/twr/main/main.cpp の diagForcePllCoarseCh9() を、
    // CONFIG_UWB_TWR_DIAG_PLL_COARSE_CH9 (コンパイル時定数) の代わりに
    // 引数 code を使うようパラメータ化して移植したもの。レジスタ操作の
    // 手順・順序・ログの文言は完全に同一（各段の根拠は twr 側の
    // diagForcePllCoarseCh9() ドキュメントコメント、docs/HANDOFF.md 参照）。
    // 【スクリプトが grep する行】"DIAG_PLL_COARSE: done ok=%d
    // coarse_final=0x..." は twr 側と一字一句同じ文言のまま維持する。
    const uint32_t forcedCode = (uint32_t)code & PLL_COARSE_CODE_CH9_VCO_COARSE_TUNE_BIT_MASK;

    dwt_forcetrxoff();

    const uint32_t coarseBefore    = calReadReg32(PLL_COARSE_CODE_ID);
    const uint32_t pllCalBefore    = calReadReg32(PLL_CAL_ID);
    const uint32_t pllStatusBefore = calReadReg32(PLL_STATUS_ID);
    const int lockBefore = ((pllStatusBefore & PLL_STATUS_PLL_LOCK_FLAG_BIT_MASK) != 0U) ? 1 : 0;
    ESP_LOGW(kPhyTag,
             "DIAG_PLL_COARSE: start forced=0x%02lX coarse_before=0x%08lX pll_cal_before=0x%08lX "
             "pll_status_before=0x%08lX lock_before=%d",
             (unsigned long)forcedCode, (unsigned long)coarseBefore, (unsigned long)pllCalBefore,
             (unsigned long)pllStatusBefore, lockBefore);

    auto writeCoarseCode = [&]() {
        const uint32_t cur = calReadReg32(PLL_COARSE_CODE_ID);
        calWriteReg32(PLL_COARSE_CODE_ID, (cur & ~PLL_COARSE_CODE_CH9_VCO_COARSE_TUNE_BIT_MASK) | forcedCode);
    };

    auto tryWithCalEn = [&](uint32_t extraCalBits, const char* seqTag) -> bool {
        (void)dwt_setdwstate(static_cast<int>(DWT_DW_IDLE_RC));
        writeCoarseCode();

        const uint32_t pllCalPre = calReadReg32(PLL_CAL_ID);
        calWriteReg32(PLL_CAL_ID, pllCalPre | extraCalBits);

        const int32_t rc = static_cast<int32_t>(dwt_setdwstate(static_cast<int>(DWT_DW_IDLE)));

        const uint32_t coarseAfter    = calReadReg32(PLL_COARSE_CODE_ID);
        const uint32_t pllCalAfter    = calReadReg32(PLL_CAL_ID);
        const uint32_t pllStatusAfter = calReadReg32(PLL_STATUS_ID);
        const int lock           = ((pllStatusAfter & PLL_STATUS_PLL_LOCK_FLAG_BIT_MASK) != 0U) ? 1 : 0;
        const uint32_t readback  = coarseAfter & PLL_COARSE_CODE_CH9_VCO_COARSE_TUNE_BIT_MASK;

        ESP_LOGW(kPhyTag,
                 "DIAG_PLL_COARSE(%s): forced=0x%02lX readback=0x%02lX pll_cal=0x%08lX->0x%08lX "
                 "pll_status=0x%08lX lock=%d rc=%ld",
                 seqTag, (unsigned long)forcedCode, (unsigned long)readback, (unsigned long)pllCalPre,
                 (unsigned long)pllCalAfter, (unsigned long)pllStatusAfter, lock, (long)rc);

        return readback == forcedCode;
    };

    // (A) USE_OLD alone was found not sticky on real hardware (see twr's
    // diagForcePllCoarseCh9() doc comment) - start directly with (B).
    bool ok               = false;
    uint32_t stickyCalBits = PLL_CAL_PLL_USE_OLD_BIT_MASK;
    if (!ok) {
        ok            = tryWithCalEn(PLL_CAL_PLL_USE_OLD_BIT_MASK | PLL_CAL_PLL_TUNE_OVR_BIT_MASK, "B");
        stickyCalBits = PLL_CAL_PLL_USE_OLD_BIT_MASK | PLL_CAL_PLL_TUNE_OVR_BIT_MASK;
    }

    if (!ok) {
        // Sequence C: bypass PLL_CAL_EN entirely (see twr's doc comment for
        // the register-level rationale of every step below).
        (void)dwt_setdwstate(static_cast<int>(DWT_DW_IDLE_RC));

        calWriteReg32(PLL_CAL_ID, pllCalBefore);
        writeCoarseCode();

        calWriteReg16(CLK_CTRL_ID, (uint16_t)DWT_AUTO_CLKS);
        calWriteReg8(SYS_STATUS_ID, (uint8_t)(calReadReg8(SYS_STATUS_ID) | SYS_STATUS_CP_LOCK_BIT_MASK));
        calWriteReg32(SEQ_CTRL_ID, calReadReg32(SEQ_CTRL_ID) | SEQ_CTRL_AINIT2IDLE_BIT_MASK);

        bool locked = (calReadReg8(PLL_STATUS_ID) & PLL_STATUS_PLL_LOCK_FLAG_BIT_MASK) != 0U;
        for (uint8_t cnt = 1; (cnt < MAX_RETRIES_FOR_PLL) && !locked; cnt++) {
            deca_usleep(DELAY_20uUSec);
            locked = (calReadReg8(PLL_STATUS_ID) & PLL_STATUS_PLL_LOCK_FLAG_BIT_MASK) != 0U;
        }

        const uint32_t coarseAfter    = calReadReg32(PLL_COARSE_CODE_ID);
        const uint32_t pllCalAfter    = calReadReg32(PLL_CAL_ID);
        const uint32_t pllStatusAfter = calReadReg32(PLL_STATUS_ID);
        const uint32_t readback       = coarseAfter & PLL_COARSE_CODE_CH9_VCO_COARSE_TUNE_BIT_MASK;

        ESP_LOGW(kPhyTag,
                 "DIAG_PLL_COARSE(C): forced=0x%02lX readback=0x%02lX pll_cal=0x%08lX->0x%08lX "
                 "pll_status=0x%08lX lock=%d rc=%ld",
                 (unsigned long)forcedCode, (unsigned long)readback, (unsigned long)pllCalBefore,
                 (unsigned long)pllCalAfter, (unsigned long)pllStatusAfter, locked ? 1 : 0,
                 (long)(locked ? DWT_SUCCESS : DWT_ERR_PLL_LOCK));

        stickyCalBits = 0U; // (C) never sets USE_OLD/TUNE_OVR.

        if (locked) {
            ok = true;
        } else {
            calWriteReg32(PLL_COARSE_CODE_ID, coarseBefore);
            (void)dwt_setdwstate(static_cast<int>(DWT_DW_IDLE_RC));
            const int32_t rcFallback = static_cast<int32_t>(dwt_setdwstate(static_cast<int>(DWT_DW_IDLE)));
            ESP_LOGW(kPhyTag, "DIAG_PLL_COARSE(C) failed, restored normal cal: rc=%ld", (long)rcFallback);
        }
    }

    const uint32_t extraMask   = PLL_CAL_PLL_USE_OLD_BIT_MASK | PLL_CAL_PLL_TUNE_OVR_BIT_MASK;
    const uint32_t pllCalFinal = (pllCalBefore & ~extraMask) | stickyCalBits;
    calWriteReg32(PLL_CAL_ID, pllCalFinal);

    const uint32_t coarseFinal    = calReadReg32(PLL_COARSE_CODE_ID);
    const uint32_t pllStatusFinal = calReadReg32(PLL_STATUS_ID);
    const int lockFinal = ((pllStatusFinal & PLL_STATUS_PLL_LOCK_FLAG_BIT_MASK) != 0U) ? 1 : 0;
    ESP_LOGW(kPhyTag,
             "DIAG_PLL_COARSE: done ok=%d coarse_final=0x%08lX pll_cal_final=0x%08lX pll_status_final=0x%08lX "
             "lock_final=%d",
             ok ? 1 : 0, (unsigned long)coarseFinal, (unsigned long)pllCalFinal, (unsigned long)pllStatusFinal,
             lockFinal);

    return ok;
}

void Qm33120::logPhy(const char* tag) const
{
    const PhyConfig& phy = _impl->applied_phy;
    // firmware/twr/main/main.cpp と一字一句同じ書式（スクリプトが grep する）。
    ESP_LOGI(tag, "phy: preamble=%u pac=%u rate=%s ch=%u code=%u/%u sfd=%u txpower=0x%08lX pgdelay=0x%02X",
             static_cast<unsigned>(phy.preambleLength), pacSizeCount(phy.pacSize),
             (phy.dataRate == DataRate::Rate850K) ? "850kbps" : "6.8Mbps", static_cast<unsigned>(phy.channel),
             phy.txPreambleCode, phy.rxPreambleCode, static_cast<unsigned>(phy.sfdType), (unsigned long)phy.txPower,
             phy.pgDelay);
}

} // namespace uwb
