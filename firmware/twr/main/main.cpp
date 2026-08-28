/**
 * @file main.cpp
 * @brief Phase 2 Step 2 acceptance/evaluation firmware for
 * components/uwb_qm33120's TWR methods (uwb::Qm33120::requestRange() /
 * respondRange() / requestDSRange() / respondDSRange()).
 *
 * Kconfig で選ぶ3軸 (main/Kconfig.projbuild):
 *   - ボード: M5StampS3A / M5 AtomS3        (boards ディレクトリ配下の
 *     stamps3.h / atoms3.h でピン定義切替。firmware/probe, firmware/devtest
 *     と同じ作法)
 *   - ロール: TAG(initiator) / ANCHOR(responder)
 *   - 方式  : SS-TWR / DS-TWR
 *
 * 各組み合わせの処理は third_party/M5Stamp-UWB/examples 配下の
 * {SS,DS}_TWR_{TAG,ANCHOR} 各 .ino に準拠（レンジング設定の数値もそちらの値を
 * そのまま使用。uwb::RangeConfig/
 * uwb::DSRangeConfig の構造体既定値とは一部異なる - 例えば
 * responseRxAfterTxDelayUus は構造体既定 500 に対し examples は 1500 を明示指定
 * している。ここでは examples に合わせる）。
 *
 * TAG: 200ms間隔で1回レンジングし、10回に1回、成功率と距離の平均/標準偏差
 * （Welfordのオンラインアルゴリズムで起動からの累積を計算）をログに出す。
 * ANCHOR: 毎ループ respond{Range,DSRange}() をブロッキング呼び出しし、
 * RxTimeout（まだPollが来ていないだけ）は無視、20回成功するごとにログを出す。
 * DS-TWR の ANCHOR は自分で距離を計算する（respondDSRange()の戻り値）ので、
 * こちらでも同じ統計を出す。
 *
 * 【docs/archive/REIMPL_PLAN.md R3-1/R9】上記の「examplesの値をそのまま使用」の
 * 例外として、RANGE_HOST_TIMEOUT_MS(100→10)/RESULT_RX_AFTER_FINAL_TX_DLY_UUS
 * (500→200)/RESULT_REPEAT_COUNT(3→1) の3つは
 * uwb::RangeConfig/DSRangeConfig 側の新しい既定値
 * （components/uwb_qm33120/include/uwb_qm33120_types.hpp）に揃えて更新した
 * （examplesの値は検証されていない二次資料であり、この3つは特に
 * docs/archive/CRITICAL_REVIEW.md【重大3】で問題が指摘されていたため）。
 */
#include <cmath>
#include <cstdio>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "deca_device_api.h"
#include "deca_private.h" // dwt_readfromdevice(): TX_POWER readback / TX_POWER の読み戻し
#include "dw3720_deca_regs.h" // TX_POWER_ID
#include "uwb_port.h"
#include "uwb_status_led.h"
#include "uwb_qm33120.hpp"

#if CONFIG_UWB_TWR_BOARD_ATOMS3
#include "boards/atoms3.h"
#define BOARD_UWB_PORT_CONFIG BOARD_ATOMS3_UWB_PORT_CONFIG
#define BOARD_NAME            "AtomS3(pinout " BOARD_ATOMS3_PINOUT_NAME ")"
#elif CONFIG_UWB_TWR_BOARD_STAMPFLY
#include "boards/stampfly.h"
#define BOARD_UWB_PORT_CONFIG BOARD_STAMPFLY_UWB_PORT_CONFIG
#define BOARD_NAME            "StampFly"
#define BOARD_STATUS_LED_GPIO BOARD_STAMPFLY_STATUS_LED_GPIO
#else
#include "boards/stamps3.h"
#define BOARD_UWB_PORT_CONFIG BOARD_STAMPS3_UWB_PORT_CONFIG
#define BOARD_NAME            "M5StampS3A"
#define BOARD_STATUS_LED_GPIO BOARD_STAMPS3_STATUS_LED_GPIO
#endif

/* The unselected members of a Kconfig choice are undefined, not 0, so the
 * role has to be folded into a constant with #if rather than a C expression.
 * Kconfig の choice は選ばれなかった側が「未定義」になるので、C の式では
 * なく #if で定数に畳む。 */
#if CONFIG_UWB_TWR_ROLE_ANCHOR
#define BOARD_STATUS_LED_ROLE UWB_STATUS_LED_ROLE_ANCHOR
#else
#define BOARD_STATUS_LED_ROLE UWB_STATUS_LED_ROLE_TAG
#endif

#if CONFIG_UWB_TWR_ROLE_ANCHOR
#define ROLE_NAME "ANCHOR"
#else
#define ROLE_NAME "TAG"
#endif

#if CONFIG_UWB_TWR_METHOD_DS
#define METHOD_NAME "DS-TWR"
#else
#define METHOD_NAME "SS-TWR"
#endif

static const char* TAG = "uwb_twr";

/* ---------------------------------------------------------------------------
 * Diagnostics for the SS-TWR success rate seen on real hardware
 * (2026-08-28: 0-9% over 60cm..1.7m, independent of distance, orientation,
 * which board plays which role, and of re-running the PHY configuration).
 *
 * Logs the die temperature, the crystal trim and the measured clock offset,
 * and - when CONFIG_UWB_TWR_DIAG_REINIT_FAILS > 0 - re-runs the PHY
 * configuration (and with it the PLL calibration) after N consecutive
 * failures. That re-run was the test for task R10 (docs/GETTING_STARTED.md:
 * 1380): on channel 9, the only channel this module supports, the PLL must
 * be recalibrated whenever the die temperature moves by 20 degC (user manual
 * 10.4), which this repo does not do.
 *
 * MEASURED RESULT (2026-08-28): the die temperature is stable in time
 * (tag 34.5-35.5 degC, anchor 51-57 degC), and 13 re-runs of init() did not
 * restore the link. So a *drift over time* of one chip's PLL is NOT the
 * cause. What remains unexplained is the ~17-21 degC standing difference
 * between the two chips (the anchor self-heats in continuous RX).
 *
 * 実機の SS-TWR 成功率（2026-08-28: 60cm〜1.7m で 0〜9%。距離・向き・役割の
 * 割当て・PHY 再設定のいずれとも無関係）を切り分けるための診断。
 * ダイ温度・水晶トリム・クロックオフセットをログに出し、
 * CONFIG_UWB_TWR_DIAG_REINIT_FAILS > 0 のときは連続 N 回失敗で PHY 設定を
 * やり直す（＝課題 R10 の検証。ch9 は 20°C 変化で PLL 再校正が要る）。
 * 実測結果（2026-08-28）: ダイ温度は時間的に安定（タグ 34.5〜35.5°C、
 * アンカー 51〜57°C）で、init() を 13 回やり直しても復帰しなかった。
 * よって「片方の PLL が時間とともにずれる」ことは原因ではない。
 * 未解明なのは 2 個体間に定常的にある 17〜21°C の温度差のほう
 * （アンカーは連続受信で自己発熱する）。
 * ------------------------------------------------------------------------- */
static constexpr uint32_t DIAG_REINIT_AFTER_FAILS = CONFIG_UWB_TWR_DIAG_REINIT_FAILS;

/**
 * @brief Decode the RX error bits of SYS_STATUS into a short name list.
 *        Error::RxError lumps every receive failure together; these bits are
 *        what actually tells them apart.
 *        SYS_STATUS の受信エラービットを名前に開く。Error::RxError は受信
 *        失敗を全部ひとまとめにするので、その内訳を見るために要る。
 */
static const char* rxStatusBits(uint32_t st, char* buf, size_t len)
{
    static const struct {
        uint32_t mask;
        const char* name;
    } kBits[] = {
        {DWT_INT_RXPHE_BIT_MASK, "RXPHE"},    // PHY header error / PHR の誤り
        {DWT_INT_RXFCE_BIT_MASK, "RXFCE"},    // CRC error / CRC の誤り
        {DWT_INT_RXFSL_BIT_MASK, "RXFSL"},    // Reed-Solomon / sync loss / 同期ロスト
        {DWT_INT_RXSTO_BIT_MASK, "RXSTO"},    // SFD timeout
        {DWT_INT_RXPTO_BIT_MASK, "RXPTO"},    // preamble timeout
        {DWT_INT_ARFE_BIT_MASK, "ARFE"},      // frame filtering / フレームフィルタ
        {DWT_INT_CIAERR_BIT_MASK, "CIAERR"},  // CIA timestamp estimator / タイムスタンプ推定
        {DWT_INT_RXFTO_BIT_MASK, "RXFTO"},    // frame wait timeout
    };
    size_t n = 0;
    buf[0]   = '\0';
    for (size_t i = 0; i < (sizeof(kBits) / sizeof(kBits[0])); i++) {
        if (((st & kBits[i].mask) != 0) && ((n + 8) < len)) {
            n += static_cast<size_t>(snprintf(buf + n, len - n, "%s%s", (n != 0) ? "|" : "", kBits[i].name));
        }
    }
    if (buf[0] == '\0') {
        snprintf(buf, len, "none");
    }
    return buf;
}

/**
 * @brief Format a signed Q8.8 dBm value (dwt_calculate_rssi() /
 *        dwt_calculate_first_path_power() convention, real dBm =
 *        value / 256.0) into buf, or "n/a" when q8 is INT16_MIN
 *        ("not available" - printing INT16_MIN/256 as a plain %.1f would
 *        show a misleadingly plausible -128.0).
 *        符号付き Q8.8 dBm 値（dwt_calculate_rssi() /
 *        dwt_calculate_first_path_power() の形式。実 dBm = 値/256.0）を buf
 *        に整形する。q8 が INT16_MIN（取得不可）のときは "n/a" にする
 *        （そのまま %.1f で出すと -128.0 という紛らわしい値になるため）。
 */
static const char* fmtDbmQ8(int16_t q8, char* buf, size_t len)
{
    if (q8 == INT16_MIN) {
        snprintf(buf, len, "n/a");
    } else {
        snprintf(buf, len, "%.1f", static_cast<float>(q8) / 256.0f);
    }
    return buf;
}

/**
 * @brief Convert a PacSize enum value to the PAC length it represents, in
 *        symbols (4/8/16/32), for logging.
 *        PacSize の enum 値を、ログ表示用に実際の PAC 長（4/8/16/32
 *        シンボル）へ変換する。
 */
static unsigned pacSizeCount(uwb::PacSize pac)
{
    switch (pac) {
    case uwb::PacSize::Pac4:
        return 4;
    case uwb::PacSize::Pac8:
        return 8;
    case uwb::PacSize::Pac16:
        return 16;
    case uwb::PacSize::Pac32:
        return 32;
    default:
        return 0;
    }
}

/** @brief Read the on-chip die temperature [degC] / ダイ温度を読む */
static float dieTempC()
{
    const uint16_t raw = dwt_readtempvbat();
    return dwt_convertrawtemperature(static_cast<uint8_t>(raw >> 8));
}

/**
 * @brief Read a register via dwt_readfromdevice() (little-endian byte order,
 * same as the SDK's own dwt_read32bitoffsetreg()/dwt_read16bitoffsetreg() -
 * see dw3720_device.c). Local to the boot-time calibration dump below;
 * firmware/probe has an equivalent (l11_read_reg32/16) but it lives in a
 * different component and is not included here, so it is duplicated
 * locally instead.
 * dwt_readfromdevice() 経由でレジスタを読む（リトルエンディアン、SDK自身の
 * dwt_read32bitoffsetreg()/dwt_read16bitoffsetreg() と同じ並び -
 * dw3720_device.c 参照）。下記の起動時キャリブレーションダンプ専用のローカル
 * ヘルパー。firmware/probe に同等品(l11_read_reg32/16)があるが別コンポーネント
 * なのでここには include せず、ローカルに複製する。
 */
static uint32_t calReadReg32(uint32_t regFileID)
{
    uint8_t buf[4] = {0, 0, 0, 0};
    dwt_readfromdevice(regFileID, 0, sizeof(buf), buf);
    return ((uint32_t)buf[3] << 24) | ((uint32_t)buf[2] << 16) | ((uint32_t)buf[1] << 8) | (uint32_t)buf[0];
}

/** @brief 16-bit version of calReadReg32() / calReadReg32()の16-bit版 */
static uint16_t calReadReg16(uint32_t regFileID)
{
    uint8_t buf[2] = {0, 0};
    dwt_readfromdevice(regFileID, 0, sizeof(buf), buf);
    return (uint16_t)(((uint16_t)buf[1] << 8) | (uint16_t)buf[0]);
}

/** @brief 8-bit version of calReadReg32() / calReadReg32()の8-bit版 */
static uint8_t calReadReg8(uint32_t regFileID)
{
    uint8_t buf[1] = {0};
    dwt_readfromdevice(regFileID, 0, sizeof(buf), buf);
    return buf[0];
}

#if defined(CONFIG_UWB_TWR_DIAG_PLL_COARSE_CH9) && (CONFIG_UWB_TWR_DIAG_PLL_COARSE_CH9 != 0)
/**
 * @brief Write a 32-bit register via dwt_writetodevice() (little-endian byte
 * order, matching calReadReg32()). Used by diagForcePllCoarseCh9() to
 * snapshot/restore registers around the forced PLL coarse-code test. Only
 * compiled in when that diagnostic is enabled, so it does not sit unused
 * otherwise.
 * dwt_writetodevice()経由で32bitレジスタを書く（calReadReg32()と同じ
 * リトルエンディアン）。diagForcePllCoarseCh9()が、強制PLL粗調整コード
 * テストの前後でレジスタを退避・復元するのに使う。この診断を無効化した
 * ビルドでは未使用のまま残らないよう、有効時のみコンパイルする。
 */
static void calWriteReg32(uint32_t regFileID, uint32_t value)
{
    uint8_t buf[4] = {(uint8_t)(value & 0xFFU), (uint8_t)((value >> 8) & 0xFFU), (uint8_t)((value >> 16) & 0xFFU),
                       (uint8_t)((value >> 24) & 0xFFU)};
    dwt_writetodevice(regFileID, 0, sizeof(buf), buf);
}

/** @brief 16-bit version of calWriteReg32() / calWriteReg32()の16-bit版 */
static void calWriteReg16(uint32_t regFileID, uint16_t value)
{
    uint8_t buf[2] = {(uint8_t)(value & 0xFFU), (uint8_t)((value >> 8) & 0xFFU)};
    dwt_writetodevice(regFileID, 0, sizeof(buf), buf);
}

/** @brief 8-bit version of calWriteReg32() / calWriteReg32()の8-bit版 */
static void calWriteReg8(uint32_t regFileID, uint8_t value)
{
    dwt_writetodevice(regFileID, 0, sizeof(value), &value);
}
#endif // defined(CONFIG_UWB_TWR_DIAG_PLL_COARSE_CH9) && (CONFIG_UWB_TWR_DIAG_PLL_COARSE_CH9 != 0)

/**
 * @brief Boot-time dump of the chip's one-shot init/config-time calibration
 * results (PLL coarse-tune + lock, PGF RX I/Q calibration, ADC offset
 * calibration, xtal trim, LDO/BIAS trim) so a cold boot and a warm reboot
 * can be diffed from the log alone.
 *
 * Why: measured today, the SS-TWR success rate is fixed at the tag's
 * init() time and depends on die temperature at that moment (cold boot at
 * 30degC -> 47-65%, warm reboot at 36-38degC -> 12-25%; unchanged afterwards
 * even as the chip warms up). One of the one-shot calibrations run inside
 * dwt_initialise()/dwt_configure() must be producing a temperature-dependent
 * result - this dump makes the raw register values visible so cold-boot and
 * warm-boot runs can be compared directly.
 *
 * ADC offset calibration is the highest-priority item here: the SDK's own
 * comment at dw3720_device.c:1716-1718 explicitly documents it as
 * temperature-dependent ("force ADC cal ... once when temperature >= 20C"),
 * making it the only calibration in this dump with a documented temperature
 * dependency in this exact range.
 *
 * 起動時に一度だけ走る init/configure 時キャリブレーションの結果
 * （PLL粗調整+ロック、PGF受信I/Q校正、ADCオフセット校正、水晶トリム、
 * LDO/BIASトリム）をダンプし、コールドブートとウォームリブートをログだけで
 * 比較できるようにする。
 *
 * 背景: 実測したところ、SS-TWRの成功率はタグのinit()時点で固定され、その瞬間の
 * ダイ温度に依存する（コールドブート30度→47-65%、ウォームリブート36-38度→
 * 12-25%、その後チップが温まっても変化しない）。dwt_initialise()/
 * dwt_configure() 内で一度だけ走るキャリブレーションのどれかが温度依存の結果を
 * 出しているはずなので、生のレジスタ値をログに出してコールド/ウォームを
 * 直接比較できるようにする。
 *
 * ここではADCオフセット校正を最優先で扱う: SDK自身のコメント
 * (dw3720_device.c:1716-1718) に「温度20度以上で1回ADC校正を強制」と明記
 * されており、この温度域で温度依存性が文書化されている唯一のキャリブレー
 * ションだから。
 */
static void logCalibrationDump()
{
    // --- PLL: coarse-tune code (from OTP or auto-cal), config/cal control
    // regs, hardware lock status, and the OTP-programmed coarse code for
    // comparison against the live register. Written by ull_initialise()
    // (dw3720_device.c:1021 xtal, :1023 OTP read of PLL_CC_ADDRESS=0x35,
    // :1026 PLL_COARSE_CODE_ID write) and by the hardware PLL-cal state
    // machine kicked from ull_setchannel() (dw3720_device.c:8098) ->
    // ull_run_hardware_pll_cal() (dw3720_device.c:8061) ->
    // ull_setdwstate(DWT_DW_IDLE) (dw3720_device.c:1097, sets
    // SEQ_CTRL_AINIT2IDLE + PLL_CAL_EN at dw3720_device.c:1126). This
    // project does not define AUTO_DW3300Q_DRIVER, so this hardware-cal
    // path runs (not the software ull_pll_ch5/ch9_auto_cal path).
    // PLL: 粗調整コード（OTP由来 or 自動校正）、設定/校正制御レジスタ、
    // ハードウェアのロック状態、比較用のOTP書き込み値（ライブレジスタとの
    // 比較用）。ull_initialise()（dw3720_device.c:1021でxtal、:1023で
    // OTPのPLL_CC_ADDRESS=0x35読み出し、:1026でPLL_COARSE_CODE_ID書き込み）と、
    // ull_setchannel()（dw3720_device.c:8098）->
    // ull_run_hardware_pll_cal()（dw3720_device.c:8061）->
    // ull_setdwstate(DWT_DW_IDLE)（dw3720_device.c:1097、
    // dw3720_device.c:1126でSEQ_CTRL_AINIT2IDLE + PLL_CAL_ENを立てる）が
    // 書き込む。本プロジェクトはAUTO_DW3300Q_DRIVERを定義していないため、
    // このハードウェア校正経路が実際に走る（ソフトウェアの
    // ull_pll_ch5/ch9_auto_cal経路ではない）。
    const uint32_t pllCoarse = calReadReg32(PLL_COARSE_CODE_ID);
    const uint32_t pllCfg    = calReadReg32(PLL_CFG_ID);
    const uint32_t pllCal    = calReadReg32(PLL_CAL_ID);
    const uint32_t pllStatus = calReadReg32(PLL_STATUS_ID);
    const int pllLock        = ((pllStatus & PLL_STATUS_PLL_LOCK_FLAG_BIT_MASK) != 0U) ? 1 : 0;
    uint32_t otpPllCc        = 0;
    // PLL_CC_ADDRESS=0x35 is a file-local #define in dw3720_device.c:75,
    // not exported by any public header - hardcoded here with that
    // provenance noted.
    // PLL_CC_ADDRESS=0x35 は dw3720_device.c:75 のファイルローカル定義で
    // 公開ヘッダには出ていないため、由来を明記した上でここに直書きする。
    dwt_otpread(0x35U, &otpPllCc, 1);
    const uint8_t xtalReg = calReadReg8(XTAL_ID);
    ESP_LOGI(TAG,
             "cal: pll_coarse=0x%08lX pll_cfg=0x%08lX pll_cal=0x%08lX pll_status=0x%08lX pll_lock=%d "
             "otp_pll_cc=0x%08lX xtal_reg=0x%02X",
             (unsigned long)pllCoarse, (unsigned long)pllCfg, (unsigned long)pllCal, (unsigned long)pllStatus,
             pllLock, (unsigned long)otpPllCc, xtalReg);

    // --- PGF (receive pulse-generator filter) I/Q calibration result, plus
    // the LDO/BIAS trim registers for cross-boot comparison. ull_pgf_cal()
    // (dw3720_device.c:2133) / ull_run_pgfcal() (dw3720_device.c:2172) runs
    // unconditionally from ull_configure() (dw3720_device.c:2104) on every
    // begin(); it sets RX_CAL_CFG_ID to trigger, polls RX_CAL_STS_ID, then
    // reads RX_CAL_RESI_ID/RX_CAL_RESQ_ID and fails
    // (DWT_ERR_RX_CAL_RESI/RESQ) if either equals ERR_RX_CAL_FAIL.
    // LDO_CTRL_ID/BIAS_CTRL_ID are not written by ull_pgf_cal() itself (it
    // only toggles+restores 4 LDO enable bits) - dumped here purely as the
    // chip's current trim state.
    // PGF（受信パルス生成フィルタ）のI/Q校正結果と、比較用のLDO/BIASトリム
    // レジスタ。ull_pgf_cal()（dw3720_device.c:2133）/ull_run_pgfcal()
    // （dw3720_device.c:2172）はbegin()のたびull_configure()
    // （dw3720_device.c:2104）から無条件に走る。RX_CAL_CFG_IDを立てて起動し、
    // RX_CAL_STS_IDをポーリングした後、RX_CAL_RESI_ID/RX_CAL_RESQ_IDを読み、
    // いずれかがERR_RX_CAL_FAILならDWT_ERR_RX_CAL_RESI/RESQとして失敗扱いする。
    // LDO_CTRL_ID/BIAS_CTRL_ID自体はull_pgf_cal()が書くわけではない（LDO有効化
    // ビット4本を一時トグルして戻すだけ）ため、ここでは単にチップの現在の
    // トリム状態として出す。
    const uint32_t rxCalResi = calReadReg32(RX_CAL_RESI_ID);
    const uint32_t rxCalResq = calReadReg32(RX_CAL_RESQ_ID);
    const uint32_t rxCalSts  = calReadReg32(RX_CAL_STS_ID);
    const uint32_t rxCalCfg  = calReadReg32(RX_CAL_CFG_ID);
    const uint32_t ldoCtrl   = calReadReg32(LDO_CTRL_ID);
    const uint16_t bias      = calReadReg16(BIAS_CTRL_ID);
    ESP_LOGI(TAG, "cal: pgf resi=0x%08lX resq=0x%08lX sts=0x%08lX cfg=0x%08lX ldo_ctrl=0x%08lX bias=0x%04X temp=%.1fC",
             (unsigned long)rxCalResi, (unsigned long)rxCalResq, (unsigned long)rxCalSts, (unsigned long)rxCalCfg,
             (unsigned long)ldoCtrl, bias, dieTempC());

    // --- ADC offset calibration (highest priority - see function doc
    // comment above: this is the one calibration the SDK documents as
    // temperature-dependent in this exact range). ull_adcoffsetscalibration()
    // (dw3720_device.c:2289), called from ull_configure()
    // (dw3720_device.c:2118) and from ull_restore_txrx() when
    // DWT_FORCE_ADCOFFSET_CAL is requested (dw3720_device.c:1802 area),
    // binary-searches the I/Q positive/negative zero-crossing thresholds
    // and writes the result to both ADC_THRESH_CFG_ID (the live register
    // actually used during RX, dw3720_device.c:2505) and
    // ADC_ZERO_THRESH_CFG_ID (the saved record, dw3720_device.c:2546) -
    // both end up holding the same value. Each packs 4 8-bit fields:
    // I_POS[7:0] I_NEG[15:8] Q_POS[23:16] Q_NEG[31:24]. thr_i/thr_q below
    // are the low/high 16 bits of that value (I_NEG:I_POS and
    // Q_NEG:Q_POS respectively).
    // ADCオフセット校正（最優先 - 上の関数コメント参照: この温度域でSDKが
    // 温度依存と明記している唯一のキャリブレーション）。
    // ull_adcoffsetscalibration()（dw3720_device.c:2289）は
    // ull_configure()（dw3720_device.c:2118）と、ull_restore_txrx()が
    // DWT_FORCE_ADCOFFSET_CALを要求されたとき（dw3720_device.c:1802付近）に
    // 呼ばれ、I/Qの正負ゼロクロス閾値を二分探索して、結果を
    // ADC_THRESH_CFG_ID（RX時に実際に使うライブレジスタ、
    // dw3720_device.c:2505）とADC_ZERO_THRESH_CFG_ID（保存用の記録、
    // dw3720_device.c:2546）の両方に書く（最終的に同じ値になる）。
    // どちらも8bit×4項目のパック: I_POS[7:0] I_NEG[15:8] Q_POS[23:16]
    // Q_NEG[31:24]。以下のthr_i/thr_qはその値の下位/上位16bit
    // （それぞれ I_NEG:I_POS と Q_NEG:Q_POS）。
    const uint32_t adcZeroThresh = calReadReg32(ADC_ZERO_THRESH_CFG_ID);
    const uint32_t adcThreshCfg  = calReadReg32(ADC_THRESH_CFG_ID);
    const uint32_t adcCfg        = calReadReg32(ADC_CFG_ID);
    const uint16_t adcThrI       = (uint16_t)(adcZeroThresh & 0xFFFFU);
    const uint16_t adcThrQ       = (uint16_t)((adcZeroThresh >> 16) & 0xFFFFU);
    ESP_LOGI(TAG, "cal: adc thr_i=0x%04X thr_q=0x%04X zero_thresh=0x%08lX thresh_cfg=0x%08lX adc_cfg=0x%08lX",
             adcThrI, adcThrQ, (unsigned long)adcZeroThresh, (unsigned long)adcThreshCfg, (unsigned long)adcCfg);
}

/**
 * @brief Re-run the PHY configuration (and with it the PLL calibration), and
 *        log the die temperature before and after.
 *        PHY 設定（＝PLL 再校正）をやり直し、前後のダイ温度をログに出す。
 */
static void diagReinit(uwb::Qm33120& uwb, uint32_t consecutiveFails)
{
    const float before = dieTempC();
    const bool ok      = uwb.init();
    ESP_LOGW(TAG, "DIAG_REINIT after %lu consecutive failures: init()=%s temp_before=%.1fC temp_after=%.1fC",
             (unsigned long)consecutiveFails, ok ? "OK" : "FAILED", before, dieTempC());
}

#if !CONFIG_UWB_TWR_DIAG_RECAL_NONE
/**
 * @brief Re-run one of the chip's one-shot calibrations at runtime (Kconfig
 *        UWB_TWR_DIAG_RECAL_KIND), log the result, and re-dump the "cal:"
 *        registers so this run can be diffed against the boot-time dump
 *        from the same log. See Kconfig.projbuild's UWB_TWR_DIAG_RECAL_KIND
 *        help for the SDK call chain each branch takes.
 *
 *        All branches are preceded by dwt_forcetrxoff() (deca_device_api.h:
 *        2301): ull_setdwstate() - which dwt_restoreconfig()/dwt_pll_cal()
 *        both go through internally - returns DWT_ERR_WRONG_STATE if the
 *        radio is in TX or RX when called (dw3720_device.c:1112-1116), and
 *        ull_adcoffsetscalibration()'s own doc comment (dw3720_device.c:
 *        2222) requires the device to already be in IDLE/IDLE_PLL.
 *        requestRange() already leaves the radio idle by the time it
 *        returns, so this call is a safety net, not a fix for an observed
 *        problem.
 *        選択したチップの一度きりキャリブレーション（Kconfigの
 *        UWB_TWR_DIAG_RECAL_KIND）をランタイムでやり直し、結果をログに
 *        出したうえで"cal:"レジスタダンプを出し直す（同じログ内で起動時
 *        ダンプと比較できるように）。各分岐が呼ぶSDK関数の詳細は
 *        Kconfig.projbuildのUWB_TWR_DIAG_RECAL_KINDのヘルプを参照。
 *
 *        どの分岐もdwt_forcetrxoff()（deca_device_api.h:2301）を先に呼ぶ:
 *        dwt_restoreconfig()/dwt_pll_cal()が内部で通るull_setdwstate()は、
 *        送受信中に呼ばれるとDWT_ERR_WRONG_STATEを返し
 *        （dw3720_device.c:1112-1116）、ull_adcoffsetscalibration()自身の
 *        コメント（dw3720_device.c:2222）もIDLE/IDLE_PLL状態を要求する。
 *        requestRange()は戻った時点で既に受信機をアイドルに戻しているはず
 *        なので、これは実害の対処ではなく念のための保険。
 */
static void diagRecal(uwb::Qm33120& uwb, uint32_t rangeCount, uint32_t rangeOkCount)
{
    (void)uwb; // Only the RECAL_FULL branch below uses it.
               // 下のRECAL_FULL分岐でのみ使う。

    dwt_forcetrxoff();

    const char* kind = "?";
    int32_t result   = 0;
#if CONFIG_UWB_TWR_DIAG_RECAL_ADC
    kind   = "ADC";
    result = dwt_restoreconfig(static_cast<dwt_restore_type_e>(DWT_RESTORE_TXRX_MODE | DWT_FORCE_ADCOFFSET_CAL));
#elif CONFIG_UWB_TWR_DIAG_RECAL_PGF
    kind   = "PGF";
    result = dwt_pgf_cal(1);
#elif CONFIG_UWB_TWR_DIAG_RECAL_PLL
    kind   = "PLL";
    result = dwt_pll_cal();
#elif CONFIG_UWB_TWR_DIAG_RECAL_FULL
    kind = "FULL";
    // Same call DIAG_REINIT (UWB_TWR_DIAG_REINIT_FAILS) uses - see
    // diagReinit() above. / DIAG_REINIT（UWB_TWR_DIAG_REINIT_FAILS）と
    // 同じ呼び出し - 上のdiagReinit()参照。
    result = uwb.init() ? 1 : 0;
#endif
    ESP_LOGW(TAG, "RECAL(%s) at count=%lu ok=%lu: result=%ld", kind, (unsigned long)rangeCount,
             (unsigned long)rangeOkCount, (long)result);
    logCalibrationDump();
}
#endif // !CONFIG_UWB_TWR_DIAG_RECAL_NONE

#if defined(CONFIG_UWB_TWR_DIAG_PLL_COARSE_CH9) && (CONFIG_UWB_TWR_DIAG_PLL_COARSE_CH9 != 0)
/**
 * @brief Force the DW3720's channel-9 PLL VCO coarse-tune code to a specific
 *        value (Kconfig UWB_TWR_DIAG_PLL_COARSE_CH9) and re-lock the PLL on
 *        it, to test the "coarse code 0x24 sits on a VCO sub-band edge"
 *        hypothesis without needing to cool the board (see
 *        logCalibrationDump()'s doc comment above, and this option's Kconfig
 *        help, for the measured background).
 *
 *        On real hardware the previous implementation - dwt_pll_chx_auto_cal()
 *        (deca_device_api.h:3928), the SDK's public entry point for locking a
 *        caller-supplied coarse code - returned 254 (not a documented DWT_*
 *        status code) and left PLL_COARSE_CODE unchanged: it never actually
 *        forced anything. This version instead drives the same registers the
 *        normal boot path drives, by hand, through up to three escalating
 *        raw-register sequences:
 *
 *        (A) dwt_setdwstate(DWT_DW_IDLE_RC) -> write the ch9 field of
 *            PLL_COARSE_CODE_ID (0x90004, bits[6:0]) to the forced code,
 *            keeping every other bit as read -> OR
 *            PLL_CAL_PLL_USE_OLD_BIT_MASK (0x2) into PLL_CAL_ID (0x90008) ->
 *            dwt_setdwstate(DWT_DW_IDLE). dwt_setdwstate() is the SDK's
 *            public wrapper (deca_compat.c:248-251, DWT_SETDWSTATE ioctl)
 *            for ull_setdwstate() (dw3720_device.c:1097-1150); its
 *            DWT_DW_IDLE branch (:1119-1129) is exactly what the normal boot
 *            path also calls at the end of ull_run_hardware_pll_cal()
 *            (:8085): switch the clock mux to auto
 *            (ull_force_clocks(FORCE_CLK_AUTO), :1123, :5624-5629), clear
 *            SYS_STATUS's CP_LOCK flag (:1124), OR
 *            PLL_CAL_PLL_CAL_EN_BIT_MASK (0x100) into PLL_CAL (:1126), set
 *            SEQ_CTRL's AINIT2IDLE bit (:1127), then poll PLL_STATUS's lock
 *            flag up to MAX_RETRIES_FOR_PLL=50 times / 20us apart
 *            (is_pll_locked(), :1064-1082; MAX_RETRIES_FOR_PLL is
 *            deca_device_api.h:121). USE_OLD is the interpretation this
 *            sequence tests: "use the code already sitting in
 *            PLL_COARSE_CODE instead of running a fresh hardware search".
 *            Nothing in this vendored SDK ever sets this bit -
 *            ull_pll_ch9_auto_cal() (dw3720_device.c:8218), the only other
 *            place PLL_CAL_PLL_USE_OLD_BIT_MASK / PLL_CAL_PLL_TUNE_OVR_
 *            BIT_MASK appear at all, explicitly CLEARS both (:8267-8268) -
 *            so pairing USE_OLD with the normal CAL_EN-driven IDLE
 *            transition is genuinely untested by the vendor, and this
 *            diagnostic verifies the outcome from the readback/lock log line
 *            below rather than assuming it works.
 *        (B) Tried only if (A)'s readback did not match the forced code:
 *            identical to (A), but also ORs in
 *            PLL_CAL_PLL_TUNE_OVR_BIT_MASK (0x4) - "override the tune value
 *            with the register contents", a stronger claim than USE_OLD's
 *            "prefer it over a search".
 *        (C) Tried only if (B) also failed: dwt_setdwstate(DWT_DW_IDLE_RC)
 *            -> reset PLL_CAL to its pre-diagnostic value (undoing (B)'s
 *            USE_OLD/TUNE_OVR) -> write the forced coarse code -> replicate
 *            ull_setdwstate(DWT_DW_IDLE)'s DWT_DW_IDLE branch
 *            (dw3720_device.c:1119-1129) BY HAND, omitting its PLL_CAL_EN
 *            write (line 1126) - i.e. force-clocks-auto, clear CP_LOCK, set
 *            AINIT2IDLE, then poll the lock flag, but never tell the
 *            hardware to (re)calibrate. This is the most direct test of
 *            whether the digital sequencer can lock straight onto whatever
 *            sits in PLL_COARSE_CODE without CAL_EN kicking off a search.
 *            If PLL_STATUS never reports lock within MAX_RETRIES_FOR_PLL
 *            retries, this diagnostic restores PLL_COARSE_CODE to its
 *            pre-diagnostic value and falls back to
 *            dwt_setdwstate(DWT_DW_IDLE_RC) + dwt_setdwstate(DWT_DW_IDLE) -
 *            the untouched normal path - so the radio is never left
 *            deliberately unlocked when this function returns.
 *
 *        Whichever sequence locks (or the (C)-failed fallback), PLL_CAL's
 *        bits other than USE_OLD/TUNE_OVR are written back to their
 *        pre-diagnostic values at the end. Unlike the old
 *        dwt_pll_chx_auto_cal()-based implementation, none of the sequences
 *        above touch PLL_CFG_ID/TX_CTRL_HI_ID/TX_CTRL_LO_ID/PLL_COMMON_ID at
 *        all, so there is nothing else to snapshot or restore.
 *
 *        実機では、以前の実装が使っていたdwt_pll_chx_auto_cal()
 *        （deca_device_api.h:3928、任意の粗調整コードをロックするための
 *        SDK公開エントリポイント）は254（DWT_*の文書化されたステータス
 *        コードではない）を返し、PLL_COARSE_CODEは変化しなかった - 実際
 *        には何も強制していなかった。この版は代わりに、通常の起動経路が
 *        触るのと同じレジスタを、最大3段階のraw-register（生レジスタ）
 *        シーケンスで手動で駆動する:
 *
 *        (A) dwt_setdwstate(DWT_DW_IDLE_RC) -> PLL_COARSE_CODE_ID
 *            （0x90004、bits[6:0]）のch9フィールドを強制コードへ書く
 *            （他のビットは読み出し値のまま） -> PLL_CAL_ID（0x90008）へ
 *            PLL_CAL_PLL_USE_OLD_BIT_MASK(0x2)をOR ->
 *            dwt_setdwstate(DWT_DW_IDLE)。dwt_setdwstate()は
 *            ull_setdwstate()（dw3720_device.c:1097-1150）のSDK公開
 *            ラッパー（deca_compat.c:248-251、DWT_SETDWSTATE ioctl経由）:
 *            そのDWT_DW_IDLE分岐（:1119-1129）は、通常の起動経路が
 *            ull_run_hardware_pll_cal()の最後（:8085）で呼んでいるのと
 *            全く同じもの: クロック選択をautoへ切替
 *            （ull_force_clocks(FORCE_CLK_AUTO)、:1123、:5624-5629）、
 *            SYS_STATUSのCP_LOCKフラグをクリア（:1124）、PLL_CALへ
 *            PLL_CAL_PLL_CAL_EN_BIT_MASK(0x100)をOR（:1126）、SEQ_CTRLの
 *            AINIT2IDLEビットをセット（:1127）、その後PLL_STATUSのロック
 *            フラグを最大MAX_RETRIES_FOR_PLL=50回・20us間隔でポーリング
 *            （is_pll_locked()、:1064-1082。MAX_RETRIES_FOR_PLLは
 *            deca_device_api.h:121）。USE_OLDはこのシーケンスが検証して
 *            いる解釈: 「新たなハードウェア探索を走らせず、
 *            PLL_COARSE_CODEに既に入っているコードを使う」。このベンダ
 *            SDKのどこもこのビットをセットしていない -
 *            ull_pll_ch9_auto_cal()（dw3720_device.c:8218、PLL_CAL_PLL_
 *            USE_OLD_BIT_MASK / PLL_CAL_PLL_TUNE_OVR_BIT_MASKが登場する
 *            唯一の他の場所）は両方を明示的にクリアしている
 *            （:8267-8268） - よってUSE_OLDを通常のCAL_EN駆動IDLE遷移と
 *            組み合わせるのはベンダによる検証が実際には無く、この診断は
 *            それが効くと仮定せず、下の読み戻し/ロック状態のログ行で
 *            結果を確認する。
 *        (B) (A)の読み戻しが強制コードと一致しなかった場合のみ試す:
 *            (A)と同一だが、PLL_CAL_PLL_TUNE_OVR_BIT_MASK(0x4)も追加で
 *            OR - 「レジスタの内容でtune値を上書きする」、USE_OLDの
 *            「探索より優先する」より強い意味。
 *        (C) (B)も失敗した場合のみ試す: dwt_setdwstate(DWT_DW_IDLE_RC)
 *            -> PLL_CALを診断前の値へ戻す（(B)のUSE_OLD/TUNE_OVRを
 *            取り消す） -> 強制粗調整コードを書く ->
 *            ull_setdwstate(DWT_DW_IDLE)のDWT_DW_IDLE分岐
 *            （dw3720_device.c:1119-1129）を手で再現するが、PLL_CAL_EN
 *            の書き込み（1126行目）だけは省く - つまりクロックauto
 *            切替・CP_LOCKクリア・AINIT2IDLEセットまでは同じだが、ロック
 *            フラグをポーリングするだけでハードウェアに（再）校正しろと
 *            は一切伝えない。CAL_ENが探索を起動しなくても、デジタル
 *            シーケンサがPLL_COARSE_CODEに入っている値へ直接ロックできる
 *            かどうかの、最も直接的な検証になる。MAX_RETRIES_FOR_PLL回
 *            以内にPLL_STATUSがロックを報告しなければ、この診断は
 *            PLL_COARSE_CODEを診断前の値へ戻したうえで
 *            dwt_setdwstate(DWT_DW_IDLE_RC) +
 *            dwt_setdwstate(DWT_DW_IDLE) - 手を加えていない通常経路 -
 *            にフォールバックする（この関数が戻るときに無線を意図的に
 *            未ロックのまま残さないため）。
 *
 *        どのシーケンスがロックしたか（あるいは(C)失敗後のフォール
 *        バックか）にかかわらず、PLL_CALのUSE_OLD/TUNE_OVR以外のビット
 *        は最後に診断前の値へ書き戻す。以前のdwt_pll_chx_auto_cal()ベース
 *        の実装と違い、上記のどのシーケンスもPLL_CFG_ID/TX_CTRL_HI_ID/
 *        TX_CTRL_LO_ID/PLL_COMMON_IDには一切触れないため、他に退避・復元
 *        すべきものが無い。
 */
static void diagForcePllCoarseCh9()
{
    const uint32_t forcedCode =
        (uint32_t)CONFIG_UWB_TWR_DIAG_PLL_COARSE_CH9 & PLL_COARSE_CODE_CH9_VCO_COARSE_TUNE_BIT_MASK;

    dwt_forcetrxoff();

    // Snapshot pre-diagnostic state / 診断前の状態を退避
    const uint32_t coarseBefore    = calReadReg32(PLL_COARSE_CODE_ID);
    const uint32_t pllCalBefore    = calReadReg32(PLL_CAL_ID);
    const uint32_t pllStatusBefore = calReadReg32(PLL_STATUS_ID);
    const int lockBefore = ((pllStatusBefore & PLL_STATUS_PLL_LOCK_FLAG_BIT_MASK) != 0U) ? 1 : 0;
    ESP_LOGW(TAG,
             "DIAG_PLL_COARSE: start forced=0x%02lX coarse_before=0x%08lX pll_cal_before=0x%08lX "
             "pll_status_before=0x%08lX lock_before=%d",
             (unsigned long)forcedCode, (unsigned long)coarseBefore, (unsigned long)pllCalBefore,
             (unsigned long)pllStatusBefore, lockBefore);

    // Write only the ch9 coarse-tune field (bits[6:0]) of PLL_COARSE_CODE,
    // keeping every other bit (CH9_ICAS/RCAS, the ch5 field, ...) as read.
    // PLL_COARSE_CODEのch9粗調整フィールド（bits[6:0]）だけを書き換え、
    // 他のビット（CH9_ICAS/RCAS、ch5用フィールド等）は読み出し値のまま残す。
    auto writeCoarseCode = [&]() {
        const uint32_t cur = calReadReg32(PLL_COARSE_CODE_ID);
        calWriteReg32(PLL_COARSE_CODE_ID, (cur & ~PLL_COARSE_CODE_CH9_VCO_COARSE_TUNE_BIT_MASK) | forcedCode);
    };

    // Sequences A/B - see the doc comment above for the full register-level
    // rationale for each. / シーケンスA/B - 各ステップの詳細は上の
    // ドキュメントコメント参照。
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

        ESP_LOGW(TAG,
                 "DIAG_PLL_COARSE(%s): forced=0x%02lX readback=0x%02lX pll_cal=0x%08lX->0x%08lX "
                 "pll_status=0x%08lX lock=%d rc=%ld",
                 seqTag, (unsigned long)forcedCode, (unsigned long)readback, (unsigned long)pllCalPre,
                 (unsigned long)pllCalAfter, (unsigned long)pllStatusAfter, lock, (long)rc);

        return readback == forcedCode;
    };

    bool ok                  = tryWithCalEn(PLL_CAL_PLL_USE_OLD_BIT_MASK, "A");
    uint32_t stickyCalBits    = PLL_CAL_PLL_USE_OLD_BIT_MASK;
    if (!ok) {
        ok            = tryWithCalEn(PLL_CAL_PLL_USE_OLD_BIT_MASK | PLL_CAL_PLL_TUNE_OVR_BIT_MASK, "B");
        stickyCalBits = PLL_CAL_PLL_USE_OLD_BIT_MASK | PLL_CAL_PLL_TUNE_OVR_BIT_MASK;
    }

    if (!ok) {
        // Sequence C: bypass PLL_CAL_EN entirely - see doc comment above.
        // シーケンスC: PLL_CAL_ENを一切使わない - 詳細は上のドキュメント
        // コメント参照。
        (void)dwt_setdwstate(static_cast<int>(DWT_DW_IDLE_RC));

        // Start from the pre-diagnostic PLL_CAL, not from (B)'s leftover
        // USE_OLD/TUNE_OVR bits - (C) tests a mechanism that uses neither.
        // (B)が残したUSE_OLD/TUNE_OVRビットではなく、診断前のPLL_CALから
        // 始める - (C)はどちらのビットも使わない機構を試す。
        calWriteReg32(PLL_CAL_ID, pllCalBefore);
        writeCoarseCode();

        // Replicate ull_setdwstate(DWT_DW_IDLE)'s DWT_DW_IDLE branch
        // (dw3720_device.c:1119-1129) by hand, omitting its PLL_CAL_EN write
        // (line 1126).
        // ull_setdwstate(DWT_DW_IDLE)のDWT_DW_IDLE分岐
        // （dw3720_device.c:1119-1129）を手で再現するが、PLL_CAL_ENの
        // 書き込み（1126行目）だけは省く。
        //  1) ull_force_clocks(dw, FORCE_CLK_AUTO) (:5624-5629): a plain
        //     16-bit overwrite of CLK_CTRL with DWT_AUTO_CLKS.
        calWriteReg16(CLK_CTRL_ID, (uint16_t)DWT_AUTO_CLKS);
        //  2) dwt_or8bitoffsetreg(dw, SYS_STATUS_ID, 0,
        //     SYS_STATUS_CP_LOCK_BIT_MASK) (:1124): SYS_STATUS is
        //     write-1-to-clear, so OR-ing in CP_LOCK clears that flag (and,
        //     like the SDK's own call, incidentally clears whatever other
        //     low-byte status bits happen to already be set - harmless here
        //     since dwt_forcetrxoff() above already left the radio idle).
        calWriteReg8(SYS_STATUS_ID, (uint8_t)(calReadReg8(SYS_STATUS_ID) | SYS_STATUS_CP_LOCK_BIT_MASK));
        //  3) dwt_or8bitoffsetreg(dw, SEQ_CTRL_ID, 0x01,
        //     AINIT2IDLE_BIT_MASK>>8) (:1127): set SEQ_CTRL's AINIT2IDLE bit
        //     - deliberately NOT touching PLL_CAL_EN.
        calWriteReg32(SEQ_CTRL_ID, calReadReg32(SEQ_CTRL_ID) | SEQ_CTRL_AINIT2IDLE_BIT_MASK);
        //  4) is_pll_locked(dw, MAX_RETRIES_FOR_PLL) (:1064-1082): check
        //     immediately, then retry every 20us up to MAX_RETRIES_FOR_PLL
        //     times.
        bool locked = (calReadReg8(PLL_STATUS_ID) & PLL_STATUS_PLL_LOCK_FLAG_BIT_MASK) != 0U;
        for (uint8_t cnt = 1; (cnt < MAX_RETRIES_FOR_PLL) && !locked; cnt++) {
            deca_usleep(DELAY_20uUSec);
            locked = (calReadReg8(PLL_STATUS_ID) & PLL_STATUS_PLL_LOCK_FLAG_BIT_MASK) != 0U;
        }

        const uint32_t coarseAfter    = calReadReg32(PLL_COARSE_CODE_ID);
        const uint32_t pllCalAfter    = calReadReg32(PLL_CAL_ID);
        const uint32_t pllStatusAfter = calReadReg32(PLL_STATUS_ID);
        const uint32_t readback       = coarseAfter & PLL_COARSE_CODE_CH9_VCO_COARSE_TUNE_BIT_MASK;

        ESP_LOGW(TAG,
                 "DIAG_PLL_COARSE(C): forced=0x%02lX readback=0x%02lX pll_cal=0x%08lX->0x%08lX "
                 "pll_status=0x%08lX lock=%d rc=%ld",
                 (unsigned long)forcedCode, (unsigned long)readback, (unsigned long)pllCalBefore,
                 (unsigned long)pllCalAfter, (unsigned long)pllStatusAfter, locked ? 1 : 0,
                 (long)(locked ? DWT_SUCCESS : DWT_ERR_PLL_LOCK));

        stickyCalBits = 0U; // (C) never sets USE_OLD/TUNE_OVR.
                             // (C)はUSE_OLD/TUNE_OVRを一切セットしない。

        if (locked) {
            ok = true;
        } else {
            // (C) failed to lock: undo our forced code and fall back to the
            // untouched normal path so the radio is not left stuck in
            // IDLE_RC/unlocked.
            // (C)がロックしなかった: 強制コードを元へ戻したうえで、無線を
            // IDLE_RC/未ロックのまま放置しないよう、手を加えていない通常
            // 経路にフォールバックする。
            calWriteReg32(PLL_COARSE_CODE_ID, coarseBefore);
            (void)dwt_setdwstate(static_cast<int>(DWT_DW_IDLE_RC));
            const int32_t rcFallback = static_cast<int32_t>(dwt_setdwstate(static_cast<int>(DWT_DW_IDLE)));
            ESP_LOGW(TAG, "DIAG_PLL_COARSE(C) failed, restored normal cal: rc=%ld", (long)rcFallback);
        }
    }

    // Restore PLL_CAL's non-USE_OLD/TUNE_OVR bits (lock-delay, WD_EN,
    // CH9_FB_OVR, and CAL_EN - which dwt_setdwstate(DWT_DW_IDLE) always
    // re-sets itself, dw3720_device.c:1126) to their pre-diagnostic values,
    // while keeping USE_OLD/TUNE_OVR set iff sequence A/B is what locked.
    // PLL_CALのUSE_OLD/TUNE_OVR以外のビット（ロック遅延・WD_EN・
    // CH9_FB_OVR、およびdwt_setdwstate(DWT_DW_IDLE)が毎回立て直すCAL_EN -
    // dw3720_device.c:1126）を診断前の値へ戻す。USE_OLD/TUNE_OVRは
    // シーケンスA/Bでロックできた場合のみ残す。
    const uint32_t extraMask   = PLL_CAL_PLL_USE_OLD_BIT_MASK | PLL_CAL_PLL_TUNE_OVR_BIT_MASK;
    const uint32_t pllCalFinal = (pllCalBefore & ~extraMask) | stickyCalBits;
    calWriteReg32(PLL_CAL_ID, pllCalFinal);

    const uint32_t coarseFinal    = calReadReg32(PLL_COARSE_CODE_ID);
    const uint32_t pllStatusFinal = calReadReg32(PLL_STATUS_ID);
    const int lockFinal = ((pllStatusFinal & PLL_STATUS_PLL_LOCK_FLAG_BIT_MASK) != 0U) ? 1 : 0;
    ESP_LOGW(TAG,
             "DIAG_PLL_COARSE: done ok=%d coarse_final=0x%08lX pll_cal_final=0x%08lX pll_status_final=0x%08lX "
             "lock_final=%d",
             ok ? 1 : 0, (unsigned long)coarseFinal, (unsigned long)pllCalFinal, (unsigned long)pllStatusFinal,
             lockFinal);

    logCalibrationDump();
}
#endif // defined(CONFIG_UWB_TWR_DIAG_PLL_COARSE_CH9) && (CONFIG_UWB_TWR_DIAG_PLL_COARSE_CH9 != 0)

#define UWB_DEV_ID_EXPECTED 0xDECA0314UL

/* --- Timing preset (docs/TIMING_PRESETS.md, task D) ---
 * Derived from the Kconfig choice UWB_TIMING_PROFILE. Both boards under
 * test (TAG and ANCHOR role) must select the same preset. */
#if CONFIG_UWB_TIMING_PROFILE_ANCHOR_IRQ
static constexpr uwb::TimingProfile TIMING_PROFILE = uwb::TimingProfile::AnchorIrq;
#elif CONFIG_UWB_TIMING_PROFILE_BOTH_IRQ
static constexpr uwb::TimingProfile TIMING_PROFILE = uwb::TimingProfile::BothIrq;
#else
static constexpr uwb::TimingProfile TIMING_PROFILE = uwb::TimingProfile::PollingBoth;
#endif

/* begin() 成功後に「実際に適用されたプリセット」で上書きする。
 * IRQ 線が死んでいると init() が PollingBoth へ降格するため、
 * コンパイル時の TIMING_PROFILE をそのまま使ってはいけない。
 * The profile init() actually applied - init() downgrades IRQ presets to
 * PollingBoth when the IRQ line turns out to be dead, so the compile-time
 * TIMING_PROFILE must not be used directly. */
static uwb::TimingProfile g_effectiveTimingProfile = TIMING_PROFILE;

/* --- ネットワーク共通パラメータ（4組み合わせ共通。examples 配下の各 .ino と同じ値） --- */
static constexpr uint16_t PAN_ID           = 0xDECA;
// タスクD-1(docs/HANDOFF.md §5): Kconfig化。既定値は元のソース上の固定値
// 0x0001/0x0002のまま変えていない(firmware/anchorのUWB_ANCHOR_SHORT_ADDRと
// 同じ書き方)。
static constexpr uint16_t TAG_SHORT_ADDR    = CONFIG_UWB_TWR_TAG_ADDR;
static constexpr uint16_t ANCHOR_SHORT_ADDR = CONFIG_UWB_TWR_ANCHOR_ADDR;

#if !CONFIG_UWB_TWR_ROLE_ANCHOR
/* TAG側の200ms間隔制御にのみ使う。ANCHOR側は毎ループブロッキング呼び出しで
 * 自前のインターバル管理をしないため、定義したままだと -Wunused-function になる。 */
static uint32_t nowMs()
{
    return static_cast<uint32_t>(esp_timer_get_time() / 1000);
}
#endif

/**
 * @brief 距離サンプル(mm)の平均・標準偏差をオンライン計算する
 * (Welfordのアルゴリズム)。全サンプルを保持せずに済むので長時間の実機評価に使える。
 */
struct DistanceStats {
    uint32_t count = 0;
    double mean    = 0.0;
    double m2      = 0.0;

    void add(float distanceMm)
    {
        count++;
        const double delta = static_cast<double>(distanceMm) - mean;
        mean += delta / static_cast<double>(count);
        const double delta2 = static_cast<double>(distanceMm) - mean;
        m2 += delta * delta2;
    }

    double stddev() const
    {
        return (count < 2) ? 0.0 : std::sqrt(m2 / static_cast<double>(count - 1));
    }
};

/**
 * @brief boards 以下のヘッダの BOARD_UWB_PORT_CONFIG を uwb::Config へコピーする。
 * firmware/devtest/main/main.cpp の makeConfigFromBoard() と同一の変換。
 */
static uwb::Config makeConfigFromBoard()
{
    const uwb_port_config_t port = BOARD_UWB_PORT_CONFIG;

    uwb::Config cfg;
    cfg.spi_host     = port.spi_host;
    cfg.pin_sck      = port.pin_sck;
    cfg.pin_mosi     = port.pin_mosi;
    cfg.pin_miso     = port.pin_miso;
    cfg.pin_cs       = port.pin_cs;
    cfg.pin_rst      = port.pin_rst;
    cfg.pin_irq      = port.pin_irq;
    cfg.pin_wakeup   = port.pin_wakeup;
    cfg.pin_gp7      = port.pin_gp7;
    cfg.spi_slow_hz  = port.spi_slow_hz;
    cfg.spi_fast_hz  = port.spi_fast_hz;
    // タスクD-2(docs/HANDOFF.md §5): 既定0のときはboards/*.hの値(16MHz)を
    // 一切上書きせず、この関数を追加する前と完全に同一の挙動にする。
    // Device IDが読めない/たまに化けるときの切り分け用。
#if CONFIG_UWB_SPI_FAST_HZ > 0
    cfg.spi_fast_hz = CONFIG_UWB_SPI_FAST_HZ;
#endif
    cfg.init_spi_bus = port.init_spi_bus;
    // CONFIG_UWB_ENABLE_IRQ is undefined (not just 0) by ESP-IDF's Kconfig
    // when set to n, so #if defined(...) && ... is used instead of #ifdef
    // (docs/IRQ_POLICY.md, main/Kconfig.projbuild). An unwired pin_irq or a
    // failed ISR registration makes Qm33120::init() fall back to polling
    // automatically.
#if defined(CONFIG_UWB_ENABLE_IRQ) && CONFIG_UWB_ENABLE_IRQ
    cfg.use_irq = true;
#else
    cfg.use_irq = false;
#endif
    // Timing preset (docs/TIMING_PRESETS.md, task D). Carried to the peer in
    // the Poll/Response frames and used for mismatch detection
    // (uwb_qm33120_twr.cpp checkTimingTag()/logTimingMismatch()).
    cfg.timing_profile = TIMING_PROFILE;
    return cfg;
}

/* runRole() は選択された ロール×方式 の組み合わせだけをビルドする
 * (4通りの中から1つ)。他の3つを常に定義すると、選択されなかった側が
 * -Wunused-function の対象になる（firmware/devtest と同じ理由）。 */

#if !CONFIG_UWB_TWR_ROLE_ANCHOR && !CONFIG_UWB_TWR_METHOD_DS
/* =========================================================================
 * TAG + SS-TWR : examples/SS_TWR_TAG/SS_TWR_TAG.ino 準拠
 * ========================================================================= */

// Poll interval. Configurable so the tag's period can be made incommensurate
// with the anchor's poll-wait window, which is what a beat between the two
// would need to show up as a change in success rate.
// Poll 周期。アンカーの受信待ち窓と周期をずらせるように Kconfig 化した。
static constexpr uint32_t RANGE_INTERVAL_MS            = CONFIG_UWB_TWR_RANGE_INTERVAL_MS;
static constexpr uint32_t TAG_LOG_INTERVAL              = 10;
static constexpr uint32_t RX_TIMEOUT_UUS                = 3000;
static constexpr uint32_t RANGE_HOST_TIMEOUT_MS          = 10;
static constexpr uint32_t RESPONSE_RX_AFTER_TX_DLY_UUS   = 1500;
static constexpr uint32_t RESPONSE_TX_DLY_UUS            = 3000;

static uwb::RangeConfig makeRangeConfig()
{
    uwb::RangeConfig range;
    range.panId                     = PAN_ID;
    range.initiatorAddress          = TAG_SHORT_ADDR;
    range.responderAddress          = ANCHOR_SHORT_ADDR;
    range.responseRxAfterTxDelayUus = RESPONSE_RX_AFTER_TX_DLY_UUS;
    range.responseTxDelayUus        = RESPONSE_TX_DLY_UUS;
    range.rxTimeoutUus              = RX_TIMEOUT_UUS;
    range.hostTimeoutMs             = RANGE_HOST_TIMEOUT_MS;
    // Apply the timing preset last so it wins over the individual assignments
    // above (*Uus fields only; panId/addresses/hostTimeoutMs are untouched).
    // The RESPONSE_* / RX_TIMEOUT_UUS constants above are therefore NOT the
    // effective values: docs/TIMING_PRESETS.md section 2 is the single source.
    //
    // [Behaviour change] RESPONSE_RX_AFTER_TX_DLY_UUS (1500) and RX_TIMEOUT_UUS
    // (3000) above do not match RangeConfig's real default (500 / 4500 =
    // the PollingBoth preset; uwb_qm33120_types.hpp). They look like values
    // copied from the DS-TWR block by mistake. requestRange() DOES use both
    // (dwt_setrxaftertxdelay / dwt_setrxtimeout, uwb_qm33120_twr.cpp), so the
    // preset really does change this path: (1500, 4500) -> (500, 4500)
    // whenever the effective profile ends up PollingBoth - which is NOT the
    // Kconfig default (that is BothIrq); it is what init() falls back to at
    // runtime when the IRQ line turns out to be dead (Qm33120::verifyIrqLine()),
    // or what a user explicitly selects. This is safe - the new values open
    // the RX window earlier and keep it open longer, and they satisfy the
    // deadline inequality in docs/TIMING_PRESETS.md section 1.2 with more
    // margin than the old ones.
    uwb::applyTimingProfile(range, g_effectiveTimingProfile);
    return range;
}

static void runRole(uwb::Qm33120& uwb)
{
    uint32_t lastRangeMs = 0;
    uint32_t rangeCount = 0, rangeOkCount = 0, rangeFailCount = 0;
    uint32_t consecutiveFails = 0;
    DistanceStats stats;

    while (1) {
        if ((nowMs() - lastRangeMs) < RANGE_INTERVAL_MS) {
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }
        lastRangeMs = nowMs();

        const uwb::RangeResult result = uwb.requestRange(makeRangeConfig());
        rangeCount++;
        if (result.success) {
            rangeOkCount++;
            consecutiveFails = 0;
            stats.add(result.distanceM * 1000.0f);
        } else {
            rangeFailCount++;
            consecutiveFails++;
            // Diagnostic only; disabled unless CONFIG_UWB_TWR_DIAG_REINIT_FAILS > 0.
            // 診断用。既定 (0) では何も起きない。
            if (DIAG_REINIT_AFTER_FAILS > 0 && consecutiveFails >= DIAG_REINIT_AFTER_FAILS) {
                diagReinit(uwb, consecutiveFails);
                consecutiveFails = 0;
            }
        }

#if !CONFIG_UWB_TWR_DIAG_RECAL_NONE
        // Diagnostic: run the selected one-shot calibration exactly once,
        // CONFIG_UWB_TWR_DIAG_RECAL_SEC seconds after boot, and log the
        // cumulative count/ok at that moment so the success rate before/
        // after can be computed from this log alone.
        // 診断用: 起動からCONFIG_UWB_TWR_DIAG_RECAL_SEC秒後に選択した
        // キャリブレーションを1回だけやり直し、その時点の累積count/okを
        // ログに出す（前後の成功率をこのログだけから計算できるように）。
        static bool recalDone = false;
        if (!recalDone && esp_timer_get_time() > (int64_t)CONFIG_UWB_TWR_DIAG_RECAL_SEC * 1000000LL) {
            recalDone = true;
            diagRecal(uwb, rangeCount, rangeOkCount);
        }
#endif

        if ((rangeCount % TAG_LOG_INTERVAL) != 0) {
            continue;
        }

        const float rate =
            (rangeCount == 0) ? 0.0f : (100.0f * static_cast<float>(rangeOkCount) / static_cast<float>(rangeCount));
        if (result.success) {
            char rslBuf[8];
            char fpBuf[8];
            ESP_LOGI(TAG,
                     "SS_RANGE_STAT count=%lu ok=%lu fail=%lu rate=%.1f%% last=OK seq=%u distance_mm=%ld "
                     "distance_m=%.3f mean_mm=%.1f std_mm=%.1f n=%lu elapsed_ms=%lu temp=%.1fC clock_ppm=%.2f "
                     "ipatov_power=%lu rsl_dbm=%s fp_dbm=%s accum=%u rx_seen=%u rx_rej=%u",
                     (unsigned long)rangeCount, (unsigned long)rangeOkCount, (unsigned long)rangeFailCount, rate,
                     result.sequence, (long)result.distanceMm, result.distanceM, stats.mean, stats.stddev(),
                     (unsigned long)stats.count, (unsigned long)result.elapsedMs, dieTempC(), result.clockOffsetPpm,
                     (unsigned long)result.ipatovPower, fmtDbmQ8(result.rslDbmQ8, rslBuf, sizeof(rslBuf)),
                     fmtDbmQ8(result.fpDbmQ8, fpBuf, sizeof(fpBuf)), (unsigned)result.rxAccumCount,
                     (unsigned)result.rxSeen, (unsigned)result.rxRejected);
        } else {
            char statusBuf[72];
            char rslBuf[8];
            char fpBuf[8];
            ESP_LOGW(TAG,
                     "SS_RANGE_STAT count=%lu ok=%lu fail=%lu rate=%.1f%% last=FAIL seq=%u error=%s temp=%.1fC "
                     "rx_status=0x%08lX [%s] rsl_dbm=%s fp_dbm=%s accum=%u elapsed_ms=%lu rx_seen=%u rx_rej=%u "
                     "rej_mask=0x%02X rej_len=%u rej_seq=%u",
                     (unsigned long)rangeCount, (unsigned long)rangeOkCount, (unsigned long)rangeFailCount, rate,
                     result.sequence, uwb.lastErrorName(), dieTempC(), (unsigned long)result.rxStatus,
                     rxStatusBits(result.rxStatus, statusBuf, sizeof(statusBuf)),
                     fmtDbmQ8(result.rslDbmQ8, rslBuf, sizeof(rslBuf)), fmtDbmQ8(result.fpDbmQ8, fpBuf, sizeof(fpBuf)),
                     (unsigned)result.rxAccumCount, (unsigned long)result.elapsedMs, (unsigned)result.rxSeen,
                     (unsigned)result.rxRejected, (unsigned)result.rejectMask, (unsigned)result.rejectFrameLen,
                     (unsigned)result.rejectSequence);
        }
    }
}

#elif CONFIG_UWB_TWR_METHOD_DS && !CONFIG_UWB_TWR_ROLE_ANCHOR
/* =========================================================================
 * TAG + DS-TWR : examples/DS_TWR_TAG/DS_TWR_TAG.ino 準拠
 * ========================================================================= */

static constexpr uint32_t RANGE_INTERVAL_MS = 200;
static constexpr uint32_t TAG_LOG_INTERVAL   = 10;

static constexpr uint32_t RX_TIMEOUT_UUS               = 3000;
static constexpr uint32_t RANGE_HOST_TIMEOUT_MS         = 10;
static constexpr uint32_t RESPONSE_RX_AFTER_TX_DLY_UUS  = 1500;
static constexpr uint32_t RESPONSE_TX_DLY_UUS           = 3000;
static constexpr uint32_t FINAL_TX_DLY_UUS                   = 1800;
static constexpr uint32_t FINAL_RX_AFTER_RESPONSE_TX_DLY_UUS = 500;
static constexpr uint32_t RESULT_RX_AFTER_FINAL_TX_DLY_UUS   = 200;
static constexpr uint8_t RESULT_REPEAT_COUNT    = 1;
static constexpr uint32_t RESULT_REPEAT_GAP_MS  = 3;

static uwb::DSRangeConfig makeRangeConfig()
{
    uwb::DSRangeConfig range;
    range.panId                          = PAN_ID;
    range.initiatorAddress               = TAG_SHORT_ADDR;
    range.responderAddress               = ANCHOR_SHORT_ADDR;
    range.responseRxAfterTxDelayUus      = RESPONSE_RX_AFTER_TX_DLY_UUS;
    range.responseTxDelayUus             = RESPONSE_TX_DLY_UUS;
    range.finalTxDelayUus                = FINAL_TX_DLY_UUS;
    range.finalRxAfterResponseTxDelayUus = FINAL_RX_AFTER_RESPONSE_TX_DLY_UUS;
    range.resultRxAfterFinalTxDelayUus   = RESULT_RX_AFTER_FINAL_TX_DLY_UUS;
    range.rxTimeoutUus                   = RX_TIMEOUT_UUS;
    range.hostTimeoutMs                  = RANGE_HOST_TIMEOUT_MS;
    range.resultRepeatCount              = RESULT_REPEAT_COUNT;
    range.resultRepeatGapMs              = RESULT_REPEAT_GAP_MS;
    // Apply the timing preset last so it wins over the individual
    // assignments above (*Uus fields only; panId/addresses/hostTimeoutMs/
    // resultRepeatCount/resultRepeatGapMs are untouched). This DS-TWR
    // block's individual values already match the PollingBoth row of
    // docs/TIMING_PRESETS.md SS2.2 exactly, so this call is a no-op only when
    // the effective profile is PollingBoth (explicitly selected, or
    // downgraded at runtime because the IRQ line is dead) - the Kconfig
    // default is BothIrq, for which it does change the values.
    uwb::applyTimingProfile(range, g_effectiveTimingProfile);
    return range;
}

static void runRole(uwb::Qm33120& uwb)
{
    uint32_t lastRangeMs = 0;
    uint32_t rangeCount = 0, rangeOkCount = 0, rangeFailCount = 0;
    DistanceStats stats;

    while (1) {
        if ((nowMs() - lastRangeMs) < RANGE_INTERVAL_MS) {
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }
        lastRangeMs = nowMs();

        // 距離は Anchor(respondDSRange) 側で計算済みのものをそのまま受け取るだけ
        // （requestDSRange() は再計算しない。cpp:1152-1153 相当）。
        const uwb::DSRangeResult result = uwb.requestDSRange(makeRangeConfig());
        rangeCount++;
        if (result.success) {
            rangeOkCount++;
            stats.add(result.distanceM * 1000.0f);
        } else {
            rangeFailCount++;
        }

        if ((rangeCount % TAG_LOG_INTERVAL) != 0) {
            continue;
        }

        const float rate =
            (rangeCount == 0) ? 0.0f : (100.0f * static_cast<float>(rangeOkCount) / static_cast<float>(rangeCount));
        if (result.success) {
            ESP_LOGI(TAG,
                     "DS_RANGE_STAT count=%lu ok=%lu fail=%lu rate=%.1f%% last=OK seq=%u distance_mm=%ld "
                     "distance_m=%.3f mean_mm=%.1f std_mm=%.1f n=%lu elapsed_ms=%lu",
                     (unsigned long)rangeCount, (unsigned long)rangeOkCount, (unsigned long)rangeFailCount, rate,
                     result.sequence, (long)result.distanceMm, result.distanceM, stats.mean, stats.stddev(),
                     (unsigned long)stats.count, (unsigned long)result.elapsedMs);
        } else {
            ESP_LOGW(TAG, "DS_RANGE_STAT count=%lu ok=%lu fail=%lu rate=%.1f%% last=FAIL seq=%u error=%s",
                     (unsigned long)rangeCount, (unsigned long)rangeOkCount, (unsigned long)rangeFailCount, rate,
                     result.sequence, uwb.lastErrorName());
        }
    }
}

#elif !CONFIG_UWB_TWR_METHOD_DS && CONFIG_UWB_TWR_ROLE_ANCHOR
/* =========================================================================
 * ANCHOR + SS-TWR : examples/SS_TWR_ANCHOR/SS_TWR_ANCHOR.ino 準拠
 * ========================================================================= */

static constexpr uint32_t ANCHOR_LOG_INTERVAL           = 20;
static constexpr uint32_t RX_TIMEOUT_UUS                = 3000;
static constexpr uint32_t RANGE_HOST_TIMEOUT_MS          = 10;
static constexpr uint32_t RESPONSE_RX_AFTER_TX_DLY_UUS   = 1500;
static constexpr uint32_t RESPONSE_TX_DLY_UUS            = 3000;

static uwb::RangeConfig makeRangeConfig()
{
    uwb::RangeConfig range;
    range.panId                     = PAN_ID;
    range.initiatorAddress          = TAG_SHORT_ADDR;
    range.responderAddress          = ANCHOR_SHORT_ADDR;
    range.responseRxAfterTxDelayUus = RESPONSE_RX_AFTER_TX_DLY_UUS;
    range.responseTxDelayUus        = RESPONSE_TX_DLY_UUS;
    range.rxTimeoutUus              = RX_TIMEOUT_UUS;
    range.hostTimeoutMs             = RANGE_HOST_TIMEOUT_MS;
    // Apply the timing preset last so it wins over the individual assignments
    // above (*Uus fields only; panId/addresses/hostTimeoutMs are untouched).
    // The RESPONSE_* / RX_TIMEOUT_UUS constants above are therefore NOT the
    // effective values: docs/TIMING_PRESETS.md section 2 is the single source.
    //
    // [No behaviour change here] RESPONSE_RX_AFTER_TX_DLY_UUS (1500) and
    // RX_TIMEOUT_UUS (3000) above do not match RangeConfig's real default
    // (500 / 4500 = the PollingBoth preset; uwb_qm33120_types.hpp) - they look
    // like values copied from the DS-TWR block by mistake. But respondRange(),
    // which is what the ANCHOR role calls, never reads these two fields: it
    // hardcodes dwt_setrxaftertxdelay(0) / dwt_setrxtimeout(0)
    // (components/uwb_qm33120/src/uwb_qm33120_twr.cpp, respondRange()).
    // They were already dead here, so overriding them changes nothing on air.
    uwb::applyTimingProfile(range, g_effectiveTimingProfile);
    // How long this anchor keeps its receiver armed while waiting for a poll.
    //
    // [2026-08-28 訂正] ここには以前「窓と Poll 周期がほぼ等しいために位相が
    // うなり、数十秒単位で全部聞こえる/全く聞こえないが入れ替わる」という
    // 説明が書いてあったが、これは誤りだった。respondRange() は
    // dwt_setrxtimeout(0)（タイムアウト無効）で受信機を開き、窓が満了するまで
    // 開けたままにする。呼び直しの合間に閉じている時間は SPI 数回ぶん
    // （µs オーダー）しかないので、受信デューティは窓の長さによらず 99% 以上
    // あり、窓と周期のうなりでは 1〜18% という成功率を説明できない。
    // 実際に周期性を生んでいたのは**アンカー側の 1ms tick と遅延送信の締切**
    // の関係である（docs/TIMING_PRESETS.md §4 / uwb_qm33120.cpp の
    // Qm33120::init() 内コメント）。この窓はもはや主要因ではないので、
    // 値は診断用に残すだけにする。
    //
    // The earlier comment here blamed a beat between this window and the tag's
    // poll period. That was wrong: respondRange() arms the receiver with the
    // frame-wait timeout disabled and leaves it armed for the whole window, so
    // the receive duty cycle is >99% regardless of the window length. The real
    // periodicity came from the anchor's 1 ms tick versus the delayed-TX
    // deadline. This knob is kept for diagnostics only.
#if CONFIG_UWB_TWR_POLL_WAIT_MS > 0
    range.pollHostTimeoutMs = CONFIG_UWB_TWR_POLL_WAIT_MS;
#endif
    return range;
}

static void runRole(uwb::Qm33120& uwb)
{
    uint32_t respCount = 0, failCount = 0;
    uint32_t consecutiveMisses = 0;
    // Diagnostic: how much of the time is this anchor actually listening?
    // respondRange() re-arms the receiver on every call, so any time spent
    // between calls is time the poll cannot be heard. If the tag polls every
    // 200ms and this loop also cycles every ~200ms, the two can beat against
    // each other - which would look exactly like the intermittent success we
    // measured. Log the cycle count and the mean cycle time every 5 seconds.
    // 診断: このアンカーが実際に聞いている時間の割合。respondRange() は毎回
    // 受信機を開き直すので、呼び出しの合間は Poll を聞けない。タグが 200ms
    // 周期で、このループも約 200ms 周期だと両者がうなりを起こしうる。
    // 5 秒ごとに周回数と平均周期を出す。
    uint32_t cycles = 0;
    uint32_t windowStartMs = static_cast<uint32_t>(esp_timer_get_time() / 1000);

    while (1) {
        const uwb::ResponderResult result = uwb.respondRange(makeRangeConfig());
        cycles++;
        {
            const uint32_t nowMsLocal = static_cast<uint32_t>(esp_timer_get_time() / 1000);
            const uint32_t spanMs      = nowMsLocal - windowStartMs;
            if (spanMs >= 5000) {
                ESP_LOGI(TAG, "SS_RESP_CYCLE cycles=%lu in %lums (mean %.1fms/cycle) ok=%lu fail=%lu temp=%.1fC",
                         (unsigned long)cycles, (unsigned long)spanMs,
                         (cycles == 0) ? 0.0 : (double)spanMs / (double)cycles, (unsigned long)respCount,
                         (unsigned long)failCount, dieTempC());
                cycles         = 0;
                windowStartMs  = nowMsLocal;
            }
        }
        if (!result.success) {
            if (result.error == uwb::Error::RxTimeout) {
                // まだPollが来ていないだけ。原本のANCHOR例と同じく無視して再ループ。
                // Diagnostic only; disabled unless CONFIG_UWB_TWR_DIAG_REINIT_FAILS > 0.
                // 診断用。既定 (0) では何も起きない。
                consecutiveMisses++;
                if (DIAG_REINIT_AFTER_FAILS > 0 && consecutiveMisses >= DIAG_REINIT_AFTER_FAILS) {
                    diagReinit(uwb, consecutiveMisses);
                    consecutiveMisses = 0;
                }
                continue;
            }
            failCount++;
            if ((failCount % ANCHOR_LOG_INTERVAL) == 0) {
                char statusBuf[72];
                char rslBuf[8];
                char fpBuf[8];
                // tx_margin_us: 遅延送信を予約した瞬間の締切までの残り [µs]。
                // error=TxStartFailed でこれが負なら、Poll は聞こえていたのに
                // 折返しが間に合わず**送信が取り消された**ことの直接の証拠。
                // Negative tx_margin_us with error=TxStartFailed is direct
                // evidence that the poll WAS heard but the response was
                // cancelled because the turnaround missed the deadline.
                ESP_LOGW(TAG,
                         "SS_RESP_STAT ok=%lu fail=%lu last=FAIL error=%s rx_status=0x%08lX [%s] tx_margin_us=%ld "
                         "rsl_dbm=%s fp_dbm=%s accum=%u",
                         (unsigned long)respCount, (unsigned long)failCount, uwb.lastErrorName(),
                         (unsigned long)result.rxStatus, rxStatusBits(result.rxStatus, statusBuf, sizeof(statusBuf)),
                         (long)result.txMarginUs, fmtDbmQ8(result.rslDbmQ8, rslBuf, sizeof(rslBuf)),
                         fmtDbmQ8(result.fpDbmQ8, fpBuf, sizeof(fpBuf)), (unsigned)result.rxAccumCount);
            }
            continue;
        }

        respCount++;
        consecutiveMisses = 0;
        if ((respCount % ANCHOR_LOG_INTERVAL) != 0) {
            continue;
        }
        // tx_margin_us: 成功時でも残りが小さければ締切ぎりぎりで動いている。
        // docs/TIMING_PRESETS.md §1.3 の折返し時間の見積りは、この値と
        // responseTxDelayUus の実µs値の差として実測できる。
        // Even on success, a small margin means the deadline is nearly missed.
        {
            char rslBuf[8];
            char fpBuf[8];
            ESP_LOGI(TAG,
                     "SS_RESP_STAT ok=%lu fail=%lu last=OK seq=%u requester=0x%04X elapsed_ms=%lu tx_margin_us=%ld "
                     "temp=%.1fC rsl_dbm=%s fp_dbm=%s accum=%u",
                     (unsigned long)respCount, (unsigned long)failCount, result.sequence, result.requester,
                     (unsigned long)result.elapsedMs, (long)result.txMarginUs, dieTempC(),
                     fmtDbmQ8(result.rslDbmQ8, rslBuf, sizeof(rslBuf)), fmtDbmQ8(result.fpDbmQ8, fpBuf, sizeof(fpBuf)),
                     (unsigned)result.rxAccumCount);
        }
    }
}

#else
/* =========================================================================
 * ANCHOR + DS-TWR : examples/DS_TWR_ANCHOR/DS_TWR_ANCHOR.ino 準拠
 * ========================================================================= */

static constexpr uint32_t ANCHOR_LOG_INTERVAL          = 20;
static constexpr uint32_t RX_TIMEOUT_UUS               = 3000;
static constexpr uint32_t RANGE_HOST_TIMEOUT_MS         = 10;
static constexpr uint32_t RESPONSE_RX_AFTER_TX_DLY_UUS  = 1500;
static constexpr uint32_t RESPONSE_TX_DLY_UUS           = 3000;
static constexpr uint32_t FINAL_TX_DLY_UUS                   = 1800;
static constexpr uint32_t FINAL_RX_AFTER_RESPONSE_TX_DLY_UUS = 500;
static constexpr uint32_t RESULT_RX_AFTER_FINAL_TX_DLY_UUS   = 200;
static constexpr uint8_t RESULT_REPEAT_COUNT    = 1;
static constexpr uint32_t RESULT_REPEAT_GAP_MS  = 3;

static uwb::DSRangeConfig makeRangeConfig()
{
    uwb::DSRangeConfig range;
    range.panId                          = PAN_ID;
    range.initiatorAddress               = TAG_SHORT_ADDR;
    range.responderAddress               = ANCHOR_SHORT_ADDR;
    range.responseRxAfterTxDelayUus      = RESPONSE_RX_AFTER_TX_DLY_UUS;
    range.responseTxDelayUus             = RESPONSE_TX_DLY_UUS;
    range.finalTxDelayUus                = FINAL_TX_DLY_UUS;
    range.finalRxAfterResponseTxDelayUus = FINAL_RX_AFTER_RESPONSE_TX_DLY_UUS;
    range.resultRxAfterFinalTxDelayUus   = RESULT_RX_AFTER_FINAL_TX_DLY_UUS;
    range.rxTimeoutUus                   = RX_TIMEOUT_UUS;
    range.hostTimeoutMs                  = RANGE_HOST_TIMEOUT_MS;
    range.resultRepeatCount              = RESULT_REPEAT_COUNT;
    range.resultRepeatGapMs              = RESULT_REPEAT_GAP_MS;
    // Apply the timing preset last so it wins over the individual
    // assignments above (*Uus fields only; panId/addresses/hostTimeoutMs/
    // resultRepeatCount/resultRepeatGapMs are untouched). This DS-TWR
    // block's individual values already match the PollingBoth row of
    // docs/TIMING_PRESETS.md SS2.2 exactly, so this call is a no-op only when
    // the effective profile is PollingBoth (explicitly selected, or
    // downgraded at runtime because the IRQ line is dead) - the Kconfig
    // default is BothIrq, for which it does change the values.
    uwb::applyTimingProfile(range, g_effectiveTimingProfile);
    return range;
}

static void runRole(uwb::Qm33120& uwb)
{
    uint32_t respCount = 0, failCount = 0;
    // DS-TWR は Anchor 側が距離を計算する側 (respondDSRange() cpp:1294-1319) なので、
    // 実機評価用の平均/標準偏差はこちら側でも取れる。
    DistanceStats stats;

    while (1) {
        const uwb::DSResponderResult result = uwb.respondDSRange(makeRangeConfig());
        if (!result.success) {
            if (result.error == uwb::Error::RxTimeout) {
                // まだPollが来ていないだけ。原本のANCHOR例と同じく無視して再ループ。
                continue;
            }
            failCount++;
            if ((failCount % ANCHOR_LOG_INTERVAL) == 0) {
                ESP_LOGW(TAG, "DS_RESP_STAT ok=%lu fail=%lu last=FAIL error=%s", (unsigned long)respCount,
                         (unsigned long)failCount, uwb.lastErrorName());
            }
            continue;
        }

        respCount++;
        stats.add(result.distanceM * 1000.0f);
        if ((respCount % ANCHOR_LOG_INTERVAL) != 0) {
            continue;
        }
        ESP_LOGI(TAG,
                 "DS_RESP_STAT ok=%lu fail=%lu last=OK seq=%u requester=0x%04X distance_mm=%ld distance_m=%.3f "
                 "mean_mm=%.1f std_mm=%.1f n=%lu elapsed_ms=%lu",
                 (unsigned long)respCount, (unsigned long)failCount, result.sequence, result.requester,
                 (long)result.distanceMm, result.distanceM, stats.mean, stats.stddev(), (unsigned long)stats.count,
                 (unsigned long)result.elapsedMs);
    }
}

#endif

extern "C" void app_main(void)
{
#ifdef BOARD_STATUS_LED_GPIO
/* Heartbeat colour tells the role apart once the boards are placed and no
 * serial monitor is attached: TAG = green, ANCHOR = red (components/
 * uwb_status_led). Only the M5StampS3A carries a WS2812 on a known GPIO -
 * the AtomS3 has an LCD instead - so the heartbeat compiles out there.
 * 設置後にシリアルを繋がない状態でも役割が分かるよう、ハートビートの色で
 * タグ(緑)とアンカー(赤)を見分ける。フルカラー LED を持つのは M5StampS3A
 * だけ(AtomS3 は LCD)なので、それ以外ではハートビートごと消える。 */
    (void)uwb_status_led_start_role_heartbeat(BOARD_STATUS_LED_GPIO, BOARD_STATUS_LED_ROLE);
#endif
    ESP_LOGI(TAG, "Phase 2 Step 2 uwb_qm33120 TWR firmware, board=%s role=%s method=%s", BOARD_NAME, ROLE_NAME,
             METHOD_NAME);

    uwb::Qm33120 uwbDevice;
    const uwb::Config cfg = makeConfigFromBoard();
    uwb::PhyConfig phy; // defaults: ch9, preamble128, PAC8, 6.8Mbps (see uwb_qm33120_types.hpp)
#if defined(CONFIG_UWB_TWR_DIAG_NO_LNA_PA) && CONFIG_UWB_TWR_DIAG_NO_LNA_PA
    // Diagnostic: do not put the chip into external LNA/PA mode. The default
    // (PhyConfig::enableLnaPa = true) calls dwt_setlnapamode(DWT_LNA_ENABLE |
    // DWT_PA_ENABLE), which repurposes DW3720 GPIOs to drive an external front
    // end. Whether this module has one - and whether the M5Stack wrapper does
    // the same - is unverified here, so this makes it switchable.
    // 診断: 外部 LNA/PA モードに入れない。既定は有効で、DW3720 の GPIO を
    // 外部フロントエンド制御に転用する。本モジュールに外部フロントエンドが
    // あるか、M5Stack のラッパが同じことをするかは、いずれも未確認。
    phy.enableLnaPa = false;
    ESP_LOGW(TAG, "DIAG_NO_LNA_PA: dwt_setlnapamode() is not called");
#endif
#if defined(CONFIG_UWB_TWR_DIAG_ROBUST_PHY) && CONFIG_UWB_TWR_DIAG_ROBUST_PHY
    // Diagnostic: trade update rate for link margin. The default preamble of
    // 128 symbols is the shortest the chip offers, so preamble detection has
    // the least margin; 1024 symbols with PAC32 and 850 kbps is the most
    // robust combination. Both boards of a link must use the same settings.
    // sfdTimeout stays 0 so it is recomputed for the new preamble length
    // (task R8 - a fixed sfdTimeout silently destroys the reception rate).
    // 診断用: 更新レートを犠牲にして受信の余裕を増やす。既定のプリアンブル
    // 128 シンボルはチップが出せる最短で、検出の余裕が最も小さい。
    // 1024 + PAC32 + 850kbps が最も頑健な組み合わせ。両機を同じ設定にすること。
    phy.preambleLength = uwb::PreambleLength::Len1024;
    phy.pacSize        = uwb::PacSize::Pac32;
    phy.dataRate       = uwb::DataRate::Rate850K;
    ESP_LOGW(TAG, "DIAG_ROBUST_PHY: preamble=1024 PAC=32 rate=850kbps (both boards must match)");
#endif

#if defined(CONFIG_UWB_TWR_DIAG_PHY_PREAMBLE) && (CONFIG_UWB_TWR_DIAG_PHY_PREAMBLE != 0)
    // Diagnostic: vary the preamble length alone, to see how much margin it
    // needs on its own. PAC size follows Qorvo's recommendation for the
    // chosen length. sfdTimeout stays 0 so it keeps tracking the automatic
    // calculation (task R8).
    // 診断: プリアンブル長だけを振って、どこまで余裕が要るかを測るための
    // 診断。PAC 長は Qorvo 推奨に従って長さから決める。sfdTimeout は 0 の
    // ままにして自動計算に追随させる（R8）。
#if CONFIG_UWB_TWR_DIAG_PHY_PREAMBLE == 128
    phy.preambleLength = uwb::PreambleLength::Len128;
    phy.pacSize        = uwb::PacSize::Pac8;
#elif CONFIG_UWB_TWR_DIAG_PHY_PREAMBLE == 256
    phy.preambleLength = uwb::PreambleLength::Len256;
    phy.pacSize        = uwb::PacSize::Pac16;
#elif CONFIG_UWB_TWR_DIAG_PHY_PREAMBLE == 512
    phy.preambleLength = uwb::PreambleLength::Len512;
    phy.pacSize        = uwb::PacSize::Pac16;
#elif CONFIG_UWB_TWR_DIAG_PHY_PREAMBLE == 1024
    phy.preambleLength = uwb::PreambleLength::Len1024;
    phy.pacSize        = uwb::PacSize::Pac32;
#else
#error "CONFIG_UWB_TWR_DIAG_PHY_PREAMBLE must be 0, 128, 256, 512 or 1024"
#endif
    ESP_LOGW(TAG, "DIAG_PHY_PREAMBLE: preamble=%d (PAC follows) (both boards must match)",
             CONFIG_UWB_TWR_DIAG_PHY_PREAMBLE);
#endif

#if defined(CONFIG_UWB_TWR_DIAG_PHY_850K) && CONFIG_UWB_TWR_DIAG_PHY_850K
    // Diagnostic: drop the data rate alone.
    // 診断: データ速度だけを落とす診断。
    phy.dataRate = uwb::DataRate::Rate850K;
    ESP_LOGW(TAG, "DIAG_PHY_850K: rate=850kbps (both boards must match)");
#endif

#if defined(CONFIG_UWB_TWR_DIAG_TXPOWER) && (CONFIG_UWB_TWR_DIAG_TXPOWER != 0xfefefefe)
    // Diagnostic: override the TX_POWER register value written verbatim to
    // the DW3720 (see components/uwb_qm33120/src/uwb_qm33120.cpp
    // toDwtTxConfig()/dwt_configuretxrf()). Tests the "PA overdrive / pulse
    // distortion" hypothesis for the RXFSL failures seen on this hardware:
    // if a lower value raises the success rate, the default (Qorvo ch9 max,
    // 0xfefefefe) was overdriving the PA. Both boards must match.
    // 診断: DW3720 の TX_POWER レジスタへそのまま書き込まれる値を上書きする
    // （components/uwb_qm33120/src/uwb_qm33120.cpp の toDwtTxConfig()/
    // dwt_configuretxrf() 参照）。本機で見えている RXFSL 失敗が「パワー
    // アンプの過駆動でパルスが歪んでいる」せいという仮説の検証用: 下げて
    // 成功率が上がれば既定値（Qorvo ch9 最大、0xfefefefe）が過駆動だった
    // ことになる。両機を同じ値にすること。
    phy.txPower = CONFIG_UWB_TWR_DIAG_TXPOWER;
    ESP_LOGW(TAG, "DIAG_TXPOWER: txPower=0x%08lX (default 0xfefefefe; both boards should match)",
             (unsigned long)phy.txPower);
#endif

#if defined(CONFIG_UWB_TWR_DIAG_PGDELAY) && (CONFIG_UWB_TWR_DIAG_PGDELAY != 0x34)
    // Diagnostic: override the TX pulse-generator delay (PGdly), which sets
    // the TX pulse bandwidth, written to the DW3720 TX_CTRL_HI PG_DELAY
    // field via dwt_configuretxrf() (components/uwb_qm33120/src/
    // uwb_qm33120.cpp toDwtTxConfig()). 0x34 is Qorvo's ch9 reference-design
    // default; Qorvo's API guide notes it may need per-board recalibration,
    // which this hardware lacks. Both boards must match.
    // 診断: 送信パルス発生器の遅延（PGdly、送信パルスの帯域幅を決める）を
    // 上書きする。DW3720 の TX_CTRL_HI PG_DELAY フィールドへ
    // dwt_configuretxrf() 経由で書き込まれる
    // （components/uwb_qm33120/src/uwb_qm33120.cpp の toDwtTxConfig()）。
    // 0x34 は Qorvo リファレンス設計の ch9 用既定値。Qorvo の API ガイドは
    // 基板ごとの再校正が必要な場合があるとしているが、本機にはそれがない。
    // 両機を同じ値にすること。
    phy.pgDelay = CONFIG_UWB_TWR_DIAG_PGDELAY;
    ESP_LOGW(TAG, "DIAG_PGDELAY: pgDelay=0x%02X (default 0x34; both boards should match)",
             (unsigned)phy.pgDelay);
#endif

    // タスクD-2: 実際に使うSPI高速クロックを起動ログに出す(切り分け作業で
    // Kconfigの値が本当に反映されたかを確認できるように)。
    ESP_LOGI(TAG, "spi_fast=%lu", (unsigned long)cfg.spi_fast_hz);

    if (!uwbDevice.begin(cfg, phy)) {
        ESP_LOGE(TAG, "begin() failed: error=%s", uwbDevice.lastErrorName());
        return;
    }

    ESP_LOGI(TAG, "deviceId=0x%08lX (expect 0x%08lX) chipName=%s isConnected=%d isInitialized=%d",
             (unsigned long)uwbDevice.deviceId(), (unsigned long)UWB_DEV_ID_EXPECTED, uwbDevice.chipName(),
             uwbDevice.isConnected(), uwbDevice.isInitialized());

    // 実機切り分け用: begin() に実際に効いた PHY 設定を1行で出す(DIAG_*
    // オプションのどれが実際に効いたかを、起動ログだけから確認できるように)。
    // Diagnostic: log the PHY settings that actually took effect in begin(),
    // so which of the DIAG_* options (if any) actually applied is visible
    // from the boot log alone.
    ESP_LOGI(TAG, "phy: preamble=%u pac=%u rate=%s ch=%u code=%u/%u sfd=%u txpower=0x%08lX pgdelay=0x%02X",
             static_cast<unsigned>(phy.preambleLength), pacSizeCount(phy.pacSize),
             (phy.dataRate == uwb::DataRate::Rate850K) ? "850kbps" : "6.8Mbps", static_cast<unsigned>(phy.channel),
             phy.txPreambleCode, phy.rxPreambleCode, static_cast<unsigned>(phy.sfdType),
             (unsigned long)phy.txPower, phy.pgDelay);

    if (uwbDevice.deviceId() != UWB_DEV_ID_EXPECTED) {
        ESP_LOGE(TAG, "unexpected device id, aborting");
        return;
    }
    if (!uwbDevice.isInitialized()) {
        ESP_LOGE(TAG, "PHY init did not complete (isInitialized()==false), aborting");
        return;
    }

    // init() may have downgraded the requested TIMING_PROFILE to PollingBoth
    // if the IRQ line turned out to be dead (Qm33120::verifyIrqLine()). Carry
    // the profile that was actually applied forward from here on.
    // IRQ 線が死んでいると init() が要求プリセットを PollingBoth へ降格させて
    // いることがあるため、以降は「実際に適用されたプリセット」を使う。
    g_effectiveTimingProfile = uwbDevice.config().timing_profile;

    {
        // Read back TX_POWER from the chip so the log proves what was actually applied
        // (DIAG_TXPOWER A/B showed no rsl change; rule out a lost write).
        // 実際にチップへ入った TX_POWER を読み戻してログに残す
        // (DIAG_TXPOWER の A/B で rsl が変わらなかったので、書き込み漏れを排除する)。
        uint8_t txp[4] = {0, 0, 0, 0};
        dwt_readfromdevice(TX_POWER_ID, 0, sizeof(txp), txp);
        const uint32_t txp32 = ((uint32_t)txp[3] << 24) | ((uint32_t)txp[2] << 16) | ((uint32_t)txp[1] << 8) | txp[0];
        ESP_LOGI(TAG, "tx_power readback=0x%08lX (requested 0x%08lX)", (unsigned long)txp32, (unsigned long)phy.txPower);
    }

    {
        // Read back the live PG delay register and compute its calibration
        // pulse count (dwt_calcpgcount()) so the DIAG_PGDELAY sweep can be
        // cross-checked against the chip's own bandwidth calibration from
        // the boot log alone.
        //
        // Safety/preconditions (dw3720_device.c ull_calcpgcount(),
        // ~6615-6651, reached via deca_compat.c dwt_calcpgcount() ->
        // DWT_CALCPGCOUNT ioctl, dw3720_device.c:9968-9974): the function's
        // own doc comment says it "presumes the PLL is already on (device is
        // in the IDLE state)" - true here, right after begin() succeeded and
        // before the ranging loop starts (no RX/TX exchange in flight, TRX
        // off). It temporarily forces the system clock to FOSC/4 with TX
        // clocks on and powers up the TX LDO/bias/enable blocks to run the
        // PG auto-cal, then powers them back down and restores AUTO clocking
        // when done; it never touches the TX/RX antenna switch
        // (switch_control=0 on both enable and disable), so it does not key
        // up an actual over-the-air transmission. It DOES write its pgdly
        // argument straight into the live TX_CTRL_HI PG_DELAY field as part
        // of the measurement, so it must only be called with phy.pgDelay
        // (the value already applied via dwt_configuretxrf()) - passing
        // anything else would silently move the chip's live TX pulse
        // bandwidth setting away from what was configured.
        //
        // 実際にチップへ入っている PG delay を読み戻し、その帯域校正用
        // パルスカウント（dwt_calcpgcount()）を算出してログへ残す。
        // DIAG_PGDELAY を振ったとき、チップ自身の帯域校正結果と突き合わせ
        // られるように、起動ログだけで確認できる。
        //
        // 前提・副作用（dw3720_device.c の ull_calcpgcount()、~6615-6651行、
        // deca_compat.c の dwt_calcpgcount() 経由で DWT_CALCPGCOUNT ioctl、
        // dw3720_device.c:9968-9974）: 関数自身のドキュメントコメントに
        // 「PLL は既にオン（デバイスは IDLE 状態）であることを前提とする」
        // とあり、ここは begin() 成功直後・測距ループ開始前（RX/TX の
        // やり取りが進行中でない、TRX オフ）なので条件を満たす。システム
        // クロックを一時的に FOSC/4 + TXクロックONへ強制し、TX の
        // LDO/バイアス/イネーブル系ブロックを立ち上げて PG 自動校正を
        // 走らせた後、元のAUTOクロックへ戻して立ち下げる。アンテナの
        // TX/RXスイッチには一切触れない（enable/disable とも
        // switch_control=0）ため、実際に電波を空間へ送信するわけではない。
        // ただし測定の一部として引数の pgdly を TX_CTRL_HI の PG_DELAY
        // レジスタへそのまま書き込むため、必ず phy.pgDelay
        // （dwt_configuretxrf() で実際に適用した値）を渡すこと - 別の値を
        // 渡すと、チップの生きている送信パルス帯域幅設定が設定値から
        // ずれてしまう。
        const uint8_t pgdelay_readback = dwt_readpgdelay();
        const uint16_t pgcount = dwt_calcpgcount(phy.pgDelay);
        ESP_LOGI(TAG, "pg: pgdelay_readback=0x%02X pgcount(0x%02X)=%u", pgdelay_readback, phy.pgDelay, pgcount);
    }
    ESP_LOGI(TAG, "begin() + PHY config OK, starting %s/%s loop", ROLE_NAME, METHOD_NAME);

    // --- Log whether IRQ actually ended up active (docs/IRQ_POLICY.md) ---
    // Reports Qm33120::irqActive() (what actually happened), not the
    // Kconfig setting (CONFIG_UWB_ENABLE_IRQ), so a silent fallback to
    // polling is visible from the boot log alone.
    if (uwbDevice.irqActive()) {
        ESP_LOGI(TAG, "irq=active (pin=%d)", cfg.pin_irq);
    } else if (cfg.pin_irq == UWB_PORT_PIN_UNUSED) {
        ESP_LOGI(TAG, "irq=polling (pin_irq unwired)");
    } else if (!cfg.use_irq) {
        ESP_LOGI(TAG, "irq=polling (disabled by Kconfig)");
    } else {
        ESP_LOGW(TAG, "irq=polling (enable failed)");
    }

    // --- Timing preset (docs/TIMING_PRESETS.md, task D-2) ---
    // Logs g_effectiveTimingProfile (what init() actually applied), plus
    // requested= (the compile-time TIMING_PROFILE) so a runtime downgrade to
    // PollingBoth is visible from the boot log alone.
    // g_effectiveTimingProfile（init() が実際に適用した値）と、
    // requested=（コンパイル時の TIMING_PROFILE）の両方を出す。実行時の
    // PollingBoth への降格が起きていないかを起動ログだけで判別できるように。
#if CONFIG_UWB_TWR_METHOD_DS
    {
        const uwb::TimingPresetDs ds = uwb::timingPresetDs(g_effectiveTimingProfile);
        ESP_LOGI(TAG,
                 "timing profile=%s (version=%u, requested=%s) wait=%s response_tx_delay=%luuus(%.0fus) "
                 "final_tx_delay=%luuus(%.0fus)",
                 uwb::timingProfileName(g_effectiveTimingProfile), (unsigned)uwb::kTimingPresetVersion,
                 uwb::timingProfileName(TIMING_PROFILE), uwbDevice.irqActive() ? "irq" : "polling",
                 (unsigned long)ds.responseTxDelayUus, ds.responseTxDelayUus * 1.02564,
                 (unsigned long)ds.finalTxDelayUus, ds.finalTxDelayUus * 1.02564);
    }
#else
    {
        const uwb::TimingPresetSs ss = uwb::timingPresetSs(g_effectiveTimingProfile);
        ESP_LOGI(TAG,
                 "timing profile=%s (version=%u, requested=%s) wait=%s response_tx_delay=%luuus(%.0fus)",
                 uwb::timingProfileName(g_effectiveTimingProfile), (unsigned)uwb::kTimingPresetVersion,
                 uwb::timingProfileName(TIMING_PROFILE), uwbDevice.irqActive() ? "irq" : "polling",
                 (unsigned long)ss.responseTxDelayUus, ss.responseTxDelayUus * 1.02564);
    }
#endif
    // Diagnostics: the crystal trim and the die temperature this chip booted
    // at. A large trim difference between the two boards - or a large die
    // temperature difference, since the crystal drifts with temperature -
    // shows up as a carrier frequency offset the receiver has to track.
    // 診断: この個体の水晶トリム値と起動時のダイ温度。2 個体間でこれらが
    // 大きく違うと搬送波の周波数ずれになり、受信側の追従範囲を超えうる。
    ESP_LOGI(TAG, "xtal_trim=%u die_temp=%.1fC", (unsigned)dwt_getxtaltrim(), dieTempC());
    // Boot-time calibration dump (see logCalibrationDump() doc comment for
    // why): lets cold-boot and warm-reboot runs be diffed from the log.
    // 起動時キャリブレーションダンプ（理由は logCalibrationDump() の
    // ドキュメントコメント参照）: コールドブートとウォームリブートを
    // ログだけで比較できるようにする。
    logCalibrationDump();

#if defined(CONFIG_UWB_TWR_DIAG_PLL_COARSE_CH9) && (CONFIG_UWB_TWR_DIAG_PLL_COARSE_CH9 != 0)
    // Diagnostic: force ch9's PLL VCO coarse-tune code and re-lock (see
    // diagForcePllCoarseCh9()'s doc comment and this Kconfig option's help
    // for why). Runs on both roles unconditionally when non-zero - whichever
    // board(s) happened to boot warm are the ones it actually matters for.
    // 診断用: ch9のPLL VCO粗調整コードを強制して再ロックする（理由は
    // diagForcePllCoarseCh9()のドキュメントコメントとこのKconfigオプション
    // のヘルプ参照）。非ゼロなら両ロールで無条件に走る - 実際に効くのは
    // ウォームブートした方の機体。
    diagForcePllCoarseCh9();
#endif
    // Boot-time check (docs/TIMING_PRESETS.md SS4(b)): warn loudly if the
    // selected preset needs IRQ on this device's role but it did not
    // actually come up active. **Never change the preset value
    // automatically** - it must match the peer, so a one-sided silent
    // change would defeat the whole point.
#if CONFIG_UWB_TWR_ROLE_ANCHOR
    if (uwb::timingProfileNeedsAnchorIrq(TIMING_PROFILE) && !uwbDevice.irqActive()) {
        ESP_LOGW(TAG,
                 "timing profile=%s requires IRQ on the ANCHOR, but irqActive()==false "
                 "(pin_irq unwired, disabled by Kconfig, or ISR registration failed). The wait fell back to "
                 "polling, whose ~1.2ms turnaround is likely to miss this preset's deadline. The preset value "
                 "is NOT changed automatically - reflash both TAG and ANCHOR with PollingBoth if IRQ cannot "
                 "be wired (docs/TIMING_PRESETS.md).",
                 uwb::timingProfileName(TIMING_PROFILE));
    }
#else
    if (uwb::timingProfileNeedsTagIrq(TIMING_PROFILE) && !uwbDevice.irqActive()) {
        ESP_LOGW(TAG,
                 "timing profile=%s also requires IRQ on the TAG, but irqActive()==false "
                 "(pin_irq unwired, disabled by Kconfig, or ISR registration failed). The wait fell back to "
                 "polling, whose ~1.2ms turnaround is likely to miss this preset's deadline. The preset value "
                 "is NOT changed automatically - reflash both TAG and ANCHOR with PollingBoth if IRQ cannot "
                 "be wired (docs/TIMING_PRESETS.md).",
                 uwb::timingProfileName(TIMING_PROFILE));
    }
#endif

    runRole(uwbDevice);
}
