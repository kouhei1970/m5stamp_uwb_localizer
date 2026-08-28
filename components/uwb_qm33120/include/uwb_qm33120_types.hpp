/**
 * @file uwb_qm33120_types.hpp
 * @brief Configuration / result / error types for uwb::Qm33120.
 *
 * Arduino-free, ESP-IDF-native port of the public types in
 * third_party/M5Stamp-UWB/src/M5Stamp_UWB_Types.h (M5Stack Technology CO LTD,
 * MIT). Field sets and default values are carried over 1:1 from that file
 * except where the Arduino-specific SPI/GPIO representation had to change
 * (see uwb::Config below) - see docs/archive/PROGRESS.md Phase 2 Step 1 for the full list
 * of intentional deviations.
 *
 * Phase 2 Step 2 additions (TWR): uwb::RangeConfig, uwb::DSRangeConfig,
 * uwb::RangeResult, uwb::DSRangeResult, uwb::ResponderResult,
 * uwb::DSResponderResult below are 1:1 ports of M5Stamp_UWBRangeConfig /
 * M5Stamp_UWBDSRangeConfig / M5Stamp_UWBRangeResult / M5Stamp_UWBDSRangeResult /
 * M5Stamp_UWBResponderResult / M5Stamp_UWBDSResponderResult
 * (M5Stamp_UWB_Types.h:184-258), including default values. They are consumed
 * by the TWR methods added to Qm33120 in the same step, implemented in
 * components/uwb_qm33120/src/uwb_qm33120_twr.cpp.
 *
 * Phase 2R（docs/archive/REIMPL_PLAN.md）での変更点: 一次資料（Qorvo API rev9p3 /
 * DW3720 API Guide）が手元に揃い、M5Stack既定値を検証できるようになった
 * ことで、上記の「M5Stackと1:1」の原則から意図的に外れた箇所が2つある。
 *  - R8: PhyConfig::sfdTimeout の既定値を、原本固定の129から0（自動計算）
 *    に変更した。129は既定のpreamble/SFD/PACの組み合わせでのみ正しい値で、
 *    非0のままだと他の組み合わせで自動計算式が無効化されたままになる
 *    （罠になっていた）。詳細はフィールドのコメント参照。
 *  - R7: PhyConfig::channel に Channel5 を選んだ場合、そのチャネル用の
 *    正しい pgDelay/txPower が適用されるようになった（従来はチャネルに
 *    関係なくChannel9用の既定値がそのまま使われていた。
 *    uwb_qm33120.cpp の recommendedPHYProfile() 参照）。Channel9
 *    （M5Stamp UWB Moduleが実際に使う唯一のチャネル）の既定値自体は変更なし。
 */
#pragma once

#include <cstddef>
#include <cstdint>

#include "uwb_port.h"

#include "uwb_qm33120_timing.hpp"
#include "uwb_qm33120_twr_config.hpp"
#include "uwb_qm33120_units.hpp"

namespace uwb {

/** QM33120/DW3720 DEV_ID register value (M5Stamp_UWB_Types.h:19, M5STAMP_UWB_QM33120_DEVICE_ID). */
static constexpr uint32_t kQm33120DeviceId = 0xDECA0314UL;

/**
 * @brief Board-level pin / SPI configuration for a Qm33120 connection.
 *
 * Arduino-free replacement for M5Stamp_UWBConfig (M5Stamp_UWB_Types.h:27-43).
 * The `SPIClass* spi` member is gone: this struct instead mirrors
 * uwb_port_config_t (components/uwb_port/include/uwb_port.h) directly, since
 * begin() converts it to one and hands it to uwb_port_init(). Pin values use
 * uwb_port's UWB_PORT_PIN_UNUSED (-1) sentinel, same numeric value as the
 * original M5STAMP_UWB_PIN_UNUSED.
 *
 * All pin numbers default to UWB_PORT_PIN_UNUSED (no board-specific default,
 * unlike the original which defaulted to the M5Stack Stamp C5 wiring): this
 * project supplies concrete pin sets via boards/stamps3.h / boards/atoms3.h
 * instead, so an un-overridden Config is intentionally invalid (begin()
 * rejects it with Error::InvalidConfig) rather than silently matching
 * hardware this repo does not target.
 */
struct Config {
    /* --- uwb_port_config_t 相当。begin() が uwb_port_init() へそのまま渡す。 --- */
    spi_host_device_t spi_host = SPI2_HOST; //!< SPI host, matches uwb_port_config_t.spi_host.
    int pin_sck                = UWB_PORT_PIN_UNUSED;
    int pin_mosi                = UWB_PORT_PIN_UNUSED;
    int pin_miso                = UWB_PORT_PIN_UNUSED;
    int pin_cs                  = UWB_PORT_PIN_UNUSED;
    int pin_rst                 = UWB_PORT_PIN_UNUSED; //!< optional hardware reset pin.
    //!< optional IRQ pin. IRQ対応済み(docs/IRQ_POLICY.md): 配線されており
    //!< かつ use_irq==true のときだけ uwb_port_irq_enable() で「起床信号」
    //!< として使われる。UWB_PORT_PIN_UNUSEDのまま(未配線)でも問題なく動作し、
    //!< その場合は use_irq の値に関わらずポーリングへ自動フォールバックする。
    int pin_irq                 = UWB_PORT_PIN_UNUSED;
    int pin_wakeup               = UWB_PORT_PIN_UNUSED; //!< optional WAKEUP pin.
    int pin_gp7                  = UWB_PORT_PIN_UNUSED; //!< optional DW_GP7 input pin.
    uint32_t spi_slow_hz         = 2000000;             //!< SPI speed used before/during probe (M5Stamp_UWBConfig.spi_slow_hz).
    uint32_t spi_fast_hz         = 16000000;            //!< SPI speed switched to after dwt_configure() succeeds, not merely after dwt_probe()/dwt_initialise() (uwb_qm33120.cpp Qm33120::init(); see docs/REVIEW_2026-08-21.md §0 #2). (M5Stamp_UWBConfig.spi_fast_hz).
    bool init_spi_bus            = true;                //!< false => caller already ran spi_bus_initialize() on spi_host (uwb_port_config_t.init_spi_bus).

    /* --- Qm33120 固有（原本 M5Stamp_UWBConfig の残りのフィールドをそのまま踏襲） --- */
    uint8_t probe_retry_count     = 5;   //!< number of Device ID/probe retries during begin() (M5Stamp_UWBConfig.probe_retry_count).
    uint16_t probe_retry_delay_ms = 20;  //!< delay between Device ID/probe retries (M5Stamp_UWBConfig.probe_retry_delay_ms).
    bool hard_reset_on_begin      = true; //!< toggle RESET before probing the UWB device (M5Stamp_UWBConfig.hard_reset_on_begin).

    /**
     * @brief If true (default), Qm33120::init() calls dwt_softreset(1) followed by a
     * bounded dwt_checkidlerc() wait immediately before dwt_initialise(), regardless of
     * pin_rst / hard_reset_on_begin (see uwb_qm33120.cpp Qm33120::init(),
     * docs/REVIEW_2026-08-21.md §1 M-2). Has no equivalent in the original
     * M5Stamp_UWBConfig.
     *
     * Background (external field report, GOROman, XIAO ESP32-C6 + the official Arduino
     * library, 6-wire wiring with RSTn not connected): on a cold boot dwt_configure()
     * succeeds, but after an MCU-only reset (e.g. re-flashing) the UWB chip retains its
     * previous internal state (e.g. still receiving) and the next dwt_configure() fails
     * with CONFIG_FAILED, even though DEV_ID still reads back correctly (SPI itself is
     * fine). Only a power cycle recovered it. Adding dwt_softreset(0); delay(2); right
     * before dwt_initialise() fixed it. Set this to false only to skip the extra ~3ms
     * softreset + IDLE_RC wait on every begin() once the target hardware is known not to
     * need it.
     */
    bool soft_reset_on_begin = true;

    /**
     * @brief If true, begin() does NOT call uwb_port_init() (or uwb_port_deinit()
     * in end()/on failure): it assumes another owner already initialized
     * uwb_port (e.g. a shared SPI bus set up once by platform bootstrap code).
     * Has no equivalent in the original (single-owner Arduino SPIClass model).
     * Intended for the future StampFly integration, where uwb_port may be
     * initialized once for a whole board rather than per Qm33120 instance.
     */
    bool port_already_initialized = false;

    /**
     * @brief IRQ を「起床信号」として使うか（docs/IRQ_POLICY.md）。**IRQ が本線。**
     *
     * true でも、次のいずれかなら **自動的にポーリングへフォールバックする**
     * （docs/IRQ_POLICY.md 実装要件1、docs/TIMING_PRESETS.md §4(a)）:
     *   - pin_irq が未配線（UWB_PORT_PIN_UNUSED）
     *   - ISR の登録に失敗した
     *   - **起動時の自己診断 Qm33120::verifyIrqLine() で、IRQ 線が実際には
     *     エッジを運んでいないと分かった**（2026-08-28 追加）
     *
     * 【既定を false から true にした経緯・2026-08-28】旧コメントは
     * 「既定 false: 実機未検証のため（Phase 1〜2 が通ってから既定を上げる）」
     * だったが、Phase 1（SPI 疎通）は 2026-08-27 に実機で通っており、
     * アンカー・タグとも IRQ 線を配線できることも確認済みで、ファーム側
     * （firmware/twr・anchor・tag の Kconfig UWB_ENABLE_IRQ）は既に既定 y。
     * この構造体既定だけがポーリング時代のまま取り残されていた。
     * verifyIrqLine() が入って安全に倒せるようになったので true にする。
     *
     * IRQ is the intended path. This still falls back to polling by itself if
     * the pin is unwired, the ISR cannot be installed, or the boot-time
     * self-test finds the line does not actually deliver edges.
     */
    bool use_irq = true;

    /**
     * @brief 本機が使う遅延プリセット（docs/TIMING_PRESETS.md）。
     * **タグとアンカーで一致していなければ測距が成立しない。**
     * begin() が呼んだ requestRange()/respondRange()/requestDSRange()/
     * respondDSRange() は、この値（_impl->config.timing_profile）を
     * Poll/Response フレームの末尾に載せて相手へ伝え、受信側は自分の値と
     * 比較して不一致なら警告する（uwb_qm33120_twr.cpp、タスクC）。
     * 既定 PollingBoth: RangeConfig/DSRangeConfig のメンバ初期化子と完全に
     * 一致する保守的な値。**実運用のファーム（firmware/twr・anchor・tag）は
     * Kconfig UWB_TIMING_PROFILE でこれを上書きしており、その既定は BothIrq。**
     * つまりこの構造体既定が実機に出ることは通常ない。
     *
     * init() は、IRQ が使えないと分かったときこのフィールドを PollingBoth へ
     * 書き換える（downgrade_timing_profile_when_polling 参照）。したがって
     * begin() 成功後にこのフィールドを読めば「実際に適用された種別」が得られ、
     * アプリはそれを applyTimingProfile() へ渡すこと。
     */
    TimingProfile timing_profile = TimingProfile::PollingBoth;

    /**
     * @brief 待ちがポーリングに落ちたとき、IRQ 前提のプリセット
     * (AnchorIrq / BothIrq) を PollingBoth へ**自動で降格**するか。既定 true。
     *
     * 【docs/TIMING_PRESETS.md §4(b) からの意図的な逸脱。2026-08-28】
     * 同 §4(b) は「遅延プリセットは自動で変えてはいけない。相手と一致して
     * いることが要件なので、片側が勝手に変えたら §0 の破綻そのものになる」と
     * 定めており、警告だけ出して値は据え置く設計だった。それを既定で
     * 降格する側へ変えた理由:
     *
     *  1. 据え置きは**必ず失敗する**。IRQ プリセットの responseTxDelayUus は
     *     878 UUS（約900µs）で、これは折返し約0.3ms（IRQ 駆動）を前提にした
     *     値である（同 §1.3）。ポーリングの折返しは同 §1.3 自身が「最大1000µs
     *     ＋SPI 100〜200µs ＝ 約1.2ms」と見積もっており、締切を構造的に
     *     割る。締切を割った遅延送信は dwt_starttx() が HPDWARN を見て
     *     CMD_TXRXOFF で送信ごと取り消すので、相手には電波が届かない。
     *     つまり「相手と値を揃えたまま両方とも動かない」状態になる。
     *  2. 降格は**両機が同じ挙動をする限り一致したまま**である。2台は同じ
     *     基板・同じ配線・同じファームなので、片方だけ降格する状況は考えにくい。
     *  3. 万一片側だけ降格しても、実際に適用した種別は Poll/Response
     *     フレームに載って相手へ伝わり、相手が不一致を警告する（同 §3）。
     *     この検出機構はまさにこの場合のために作られている。
     *
     * §4(b) の元の挙動（警告のみ・値は据え置き）に戻したいときは false に
     * する。アプリ側の起動時警告（firmware 各アプリの main.cpp）は降格の有無に
     * 関わらず出るので、どちらの設定でもログから状況は分かる。
     *
     * If the wait falls back to polling, downgrade an IRQ-only preset to
     * PollingBoth (default true). Set to false to restore the behaviour
     * documented in docs/TIMING_PRESETS.md section 4(b) - warn, but leave the
     * values alone.
     */
    bool downgrade_timing_profile_when_polling = true;
};

/**
 * @brief High-level errors returned by Qm33120. 1:1 with M5Stamp_UWBError
 * (M5Stamp_UWB_Types.h:48-68); only the enum/type name changed.
 */
enum class Error : int8_t {
    Ok = 0,
    InvalidConfig,
    SpiNotReady,
    ProbeFailed,
    DeviceIdMismatch,
    InitFailed,
    ConfigFailed,
    TxDataFailed,
    TxStartFailed,
    TxTimeout,
    RxStartFailed,
    RxTimeout,
    RxError,
    RxBufferTooSmall,
    FrameParseFailed,
    RangeTimestampInvalid, //!< unused until Step 2 (TWR); kept so the enum matches the original in full.
    RangeFrameMismatch,    //!< unused until Step 2 (TWR); kept so the enum matches the original in full.
    InvalidArgument,
    Busy,
};

enum class DataRate : uint8_t {
    Rate850K,
    Rate6M8,
};

enum class Channel : uint8_t {
    Channel5 = 5,
    Channel9 = 9,
};

enum class PreambleLength : uint16_t {
    Len64   = 64,
    Len128  = 128,
    Len256  = 256,
    Len512  = 512,
    Len1024 = 1024,
};

enum class PacSize : uint8_t {
    Pac4,
    Pac8,
    Pac16,
    Pac32,
};

enum class SfdType : uint8_t {
    IEEE4A,
    DW8,
    DW16,
    IEEE4Z,
};

enum class StsMode : uint8_t {
    Off,
    Mode1,
    Mode2,
    Sdc,
};

enum class PdoaMode : uint8_t {
    Off,
    Mode1,
    Mode3,
};

/**
 * @brief UWB PHY configuration used by Qm33120::init(). 1:1 with
 * M5Stamp_UWBPHYConfig (M5Stamp_UWB_Types.h:122-141), including default
 * values (ch9 / preamble128 / PAC8 / 6.8Mbps / antenna delay 16385).
 *
 * If only channel is changed, init() applies the built-in recommended profile
 * for that channel. If any other field is changed, the whole user-provided
 * configuration is used (see resolvePHYConfig() in uwb_qm33120.cpp).
 *
 * 【docs/archive/REIMPL_PLAN.md R7】上記「built-in recommended profile」の実体
 * (recommendedPHYProfile()、uwb_qm33120.cpp) は、pgDelay/txPower を
 * チャネルごとのQorvo推奨TXスペクトラム値に設定する。値の出典は
 * docs/refs/qorvo_api/DW3XXX_API_rev9p3/API/Src/config_options.c:23-33
 * (`txconfig_options`=ch5用, `txconfig_options_ch9`=ch9用)。旧実装は
 * この関数が完全な恒等関数（チャネルによらず常にch9用の値を返す）になって
 * おり、上記のコメント自体が嘘だった（Channel5を選んでもch9用のTX電力が
 * そのまま適用されていた）。
 */
struct PhyConfig {
    Channel channel               = Channel::Channel9;
    PreambleLength preambleLength = PreambleLength::Len128;
    PacSize pacSize                = PacSize::Pac8;
    uint8_t txPreambleCode         = 9;
    uint8_t rxPreambleCode         = 9;
    SfdType sfdType                 = SfdType::DW8;
    DataRate dataRate               = DataRate::Rate6M8;
    StsMode stsMode                 = StsMode::Off;
    uint16_t stsLength              = 64;
    PdoaMode pdoaMode               = PdoaMode::Off;
    /**
     * SFDタイムアウト（プリアンブルシンボル数単位）。0=自動計算
     * （preambleLength/sfdType/pacSizeから、Qorvo推奨式
     * 「preamble長 + 1 + SFD長 - PAC長」で計算する。
     * docs/refs/DW3720_API_Guide_v4.9.pdf §5.2.1）。正確な計算は
     * uwb::detail::sfdTimeoutFromPhy()（uwb_qm33120_units.hpp）、
     * それを dwt_configure()（dwt_config_t::sfdTO）に渡す箇所は
     * uwb_qm33120.cpp の makeSfdTimeout() を参照。
     *
     * 【docs/archive/REIMPL_PLAN.md R8】旧既定値は固定の129だった。これは既定の
     * preamble128/SFD8/PAC8の組み合わせでのみ正しい値（128+1+8-8=129）
     * であり、非0のため自動計算式（このフィールドが0のときしか走らない）
     * を常に無効化していた。preambleLengthだけ変える（例: Len256）と、
     * 本来257であるべきsfdTimeoutが129のまま固定され、SDKはエラーを
     * 返さずに受信率だけが激減する罠になっていた。既定を0にし、この式が
     * 常にpreambleLength/sfdType/pacSizeに追随するようにした。0を
     * そのままSDKに渡すとSDK既定の4161にフォールバックしてしまうため、
     * ラッパ（makeSfdTimeout()）は必ず計算済みの具体値を渡し、0を
     * dwt_configure()に渡すことはない。ユーザが非0を明示指定した場合は
     * 従来通りそのまま使う（挙動変更なし）。
     */
    uint16_t sfdTimeout             = 0;
    uint8_t phrMode                 = 0;
    uint8_t phrRate                 = 0;
    uint8_t pgDelay                 = 0x34;
    uint32_t txPower                = 0xfefefefe;
    uint16_t txAntennaDelay         = 16385;
    uint16_t rxAntennaDelay         = 16385;
    bool enableLnaPa                 = true;
};

/**
 * @brief IEEE 802.15.4 short-address frame parameters for TX helper APIs.
 * 1:1 with M5Stamp_UWBFrameConfig (M5Stamp_UWB_Types.h:146-153).
 */
struct FrameConfig {
    uint16_t panId   = 0xDECA;
    uint16_t src     = 0x0001;
    uint16_t dst     = 0xFFFF;
    uint8_t sequence = 0;
    bool useSequence = false;
    bool ranging     = false;
};

/**
 * @brief Result returned by Qm33120::sendFrame(). 1:1 with M5Stamp_UWBTxResult
 * (M5Stamp_UWB_Types.h:158-163).
 */
struct TxResult {
    bool success     = false;
    uint8_t sequence = 0;
    uint32_t elapsedMs = 0;
    Error error         = Error::Ok;
};

/**
 * @brief Result returned by Qm33120::receiveFrame(). 1:1 with
 * M5Stamp_UWBRxResult (M5Stamp_UWB_Types.h:168-179).
 */
struct RxResult {
    bool success          = false;
    uint8_t sequence      = 0;
    uint16_t panId        = 0;
    uint16_t src          = 0;
    uint16_t dst          = 0;
    size_t payloadLength  = 0;
    uint16_t frameLength  = 0;
    uint32_t elapsedMs    = 0;
    bool ranging          = false;
    Error error            = Error::Ok;
};

/*
 * uwb::RangeConfig / uwb::DSRangeConfig / uwb::applyTimingProfile() の実体は
 * uwb_qm33120_twr_config.hpp（同ディレクトリ、ESP-IDF/Qorvo SDK非依存）に
 * 移した。このファイルは #include するだけで、型名・シグネチャ・コメントは
 * 一切変わっていない（呼び出し側は無改造で通る）。
 */

/**
 * @brief Result returned by Qm33120::requestRange() (SS-TWR initiator). 1:1
 * with M5Stamp_UWBRangeResult (M5Stamp_UWB_Types.h:215-222).
 */
struct RangeResult {
    bool success       = false;
    uint8_t sequence   = 0;
    int32_t distanceMm = 0;
    float distanceM    = 0.0f;
    uint32_t elapsedMs = 0;
    Error error         = Error::Ok;
    /**
     * 【docs/archive/REIMPL_PLAN.md R4】requestRange() が dwt_readclockoffset() から
     * 求めたクロックオフセット比を ppm 単位で入れる（実機デバッグ用の可視化。
     * distanceM/distanceMm の算出には既に織り込み済みで、この値自体は
     * 加算・減算しない）。RangeConfig::enableClockOffsetCorrection が false
     * のとき、または成功しなかった呼び出しでは 0.0（未計算）のまま。
     * CIAが動作していないとdwt_readclockoffset()は0を返す
     * (DW3720 API Guide §5.4.13) ため、その場合もここは0になる。
     */
    float clockOffsetPpm = 0.0f;
    /**
     * Raw SYS_STATUS (low word) captured at the moment the receive wait gave
     * up, before stopRadioAndClearRxStatus() clears it. 0 when the exchange
     * succeeded or the wait simply ran out of host time without the radio
     * flagging anything. Lets the caller tell a PHY header error (RXPHE) from
     * a CRC error (RXFCE) from a sync loss (RXFSL) - Error::RxError lumps
     * them all together.
     * 受信待ちを打ち切った瞬間の SYS_STATUS（下位ワード）を、消される前に
     * そのまま持ち出す。Error::RxError では区別できない PHR エラー・CRC
     * エラー・同期ロストを呼び出し側が見分けるため。成功時は 0。
     */
    uint32_t rxStatus     = 0;
    /**
     * Ipatov channel power estimate from dwt_readdiagnostics(), captured on a
     * successful reception (0 otherwise). Raw register value - a relative
     * indication of how strong the received frame was.
     * 受信成功時の Ipatov チャネル電力推定（生値、失敗時は 0）。
     * 受信フレームの強さの相対指標。
     */
    uint32_t ipatovPower  = 0;
    /**
     * Received signal level (whole-channel power) for the Ipatov CIR,
     * captured via detail::readRxPower() - filled on BOTH the success and
     * the failure path (unlike ipatovPower above, which the older
     * dwt_readdiagnostics() call only fills on success). Signed Q8.8 dBm
     * (real dBm = value / 256.0); INT16_MIN means the CIA diagnostics could
     * not be read. Check rxAccumCount before trusting this value.
     * Ipatov CIR（前置信号相関器）のチャネル全体の受信電力（RSL）。
     * detail::readRxPower() で取得し、成功時・失敗時の両方で埋まる（上の
     * ipatovPower は旧来の dwt_readdiagnostics() 経路のままで成功時のみ）。
     * 符号付き Q8.8 形式の dBm（実 dBm = 値/256.0）。INT16_MIN は CIA 診断が
     * 読めなかったことを示す。信用してよいかは rxAccumCount も見て判断する
     * こと。
     */
    int16_t rslDbmQ8 = INT16_MIN;
    /**
     * First-path signal power for the Ipatov CIR - the power of just the
     * earliest-arriving path, as opposed to rslDbmQ8's whole-channel power.
     * Same encoding and caveats as rslDbmQ8 (signed Q8.8 dBm, INT16_MIN =
     * unavailable, check rxAccumCount before trusting).
     * Ipatov CIR の第一波（最初に到達したパス）だけの受信電力。符号化・
     * 注意点は rslDbmQ8 と同じ（符号付き Q8.8 dBm、INT16_MIN = 取得不可。
     * 信用してよいかは rxAccumCount も見て判断すること）。
     */
    int16_t fpDbmQ8 = INT16_MIN;
    /**
     * Number of symbols the CIA accumulated for the Ipatov CIR when
     * rslDbmQ8 / fpDbmQ8 were captured. 0 means the CIA never accumulated
     * anything (e.g. RXFTO with no preamble detected at all), in which case
     * rslDbmQ8 / fpDbmQ8 must not be trusted even when they are not
     * INT16_MIN.
     * rslDbmQ8 / fpDbmQ8 を取得した時点で CIA が Ipatov CIR に蓄積した
     * シンボル数。0 は CIA が何も蓄積していない（例: プリアンブルすら
     * 検出されない RXFTO）ことを意味し、その場合 rslDbmQ8 / fpDbmQ8 が
     * INT16_MIN でなくても信用してはならない。
     */
    uint16_t rxAccumCount = 0;
    /**
     * How many frames passed the hardware CRC check (RXFCG) during this one
     * ranging attempt, and how many of those the software then discarded
     * because frameMatchesExpectation() said they were not the Response we
     * were waiting for (R2 re-arms the receiver instead of failing).
     * Diagnostic only: rxRejected > 0 on a failed attempt means the radio
     * DID deliver a good frame and the loss happened in the software match,
     * not on the air.
     * この1回の測距で、ハードウェアの CRC を通った(RXFCG)フレーム数と、
     * そのうち「待っていた Response ではない」と照合で捨てた数(R2 は失敗に
     * せず受信を張り直す)。失敗した攻略で rxRejected > 0 なら、電波は届いて
     * いてソフトの照合で落としたことになる。
     */
    uint16_t rxSeen     = 0;
    uint16_t rxRejected = 0;
    /**
     * Which parts of the last discarded frame did not match, as a bit mask:
     * bit0 header, bit1 payload, bit2 sequence, bit3 PAN ID, bit4 source,
     * bit5 destination (1 = that part was OK). Plus the raw frame length and
     * sequence number of that frame. Zero when nothing was discarded.
     * 最後に捨てたフレームの、どこが合わなかったかのビットマスク:
     * bit0 ヘッダ / bit1 ペイロード / bit2 シーケンス / bit3 PAN ID /
     * bit4 送信元 / bit5 宛先（1 = そこは一致）。あわせてそのフレームの
     * 生の長さとシーケンス番号。何も捨てていなければ 0。
     */
    uint8_t rejectMask      = 0;
    uint16_t rejectFrameLen = 0;
    uint8_t rejectSequence  = 0;
};

/**
 * @brief Result returned by Qm33120::requestDSRange() (DS-TWR initiator).
 * 1:1 with M5Stamp_UWBDSRangeResult (M5Stamp_UWB_Types.h:227-234). Note: the
 * distance is computed by the responder (see DSResponderResult) and merely
 * relayed back here (see requestDSRange() cpp:1152 - the value is taken
 * as-is from the "DWD" result frame, not recomputed).
 */
struct DSRangeResult {
    bool success       = false;
    uint8_t sequence   = 0;
    int32_t distanceMm = 0;
    float distanceM    = 0.0f;
    uint32_t elapsedMs = 0;
    Error error         = Error::Ok;
    /**
     * Microseconds left before the scheduled delayed transmission when
     * dwt_starttx(DWT_START_TX_DELAYED) was issued. Negative means the
     * deadline had passed, in which case the chip raises HPDWARN and the
     * frame is CANCELLED, not sent late - so the peer sees "nothing arrived".
     * Same meaning as ResponderResult::txMarginUs; see that comment.
     * 遅延送信を予約した瞬間の締切までの残り [µs]。負なら送信は取り消される。
     * 意味は ResponderResult::txMarginUs と同じ（そちらのコメント参照）。
     */
    int32_t txMarginUs = 0;
};

/**
 * @brief Result returned by Qm33120::respondRange() (SS-TWR responder). 1:1
 * with M5Stamp_UWBResponderResult (M5Stamp_UWB_Types.h:239-245).
 */
struct ResponderResult {
    bool success       = false;
    uint8_t sequence   = 0;
    uint16_t requester = 0;
    uint32_t elapsedMs = 0;
    Error error         = Error::Ok;
    /**
     * Raw SYS_STATUS (low word) captured when the poll wait gave up, before
     * stopRadioAndClearRxStatus() clears it. Same purpose as
     * RangeResult::rxStatus - tells "nothing arrived" (RXFTO, no RXPRD) apart
     * from "arrived but could not be decoded" (RXPHE / RXFCE / RXFSL).
     * Poll 待ちを打ち切った瞬間の SYS_STATUS（下位ワード）。
     * 「何も来ていない」のか「来たが復調できない」のかを区別するため。
     */
    uint32_t rxStatus = 0;
    /**
     * How much time was left before the scheduled Response transmission when
     * dwt_starttx(DWT_START_TX_DELAYED) was issued, in microseconds. Negative
     * means the deadline had already passed, in which case the chip raises
     * HPDWARN and dwt_starttx() CANCELS the frame (CMD_TXRXOFF) - the peer
     * then hears nothing at all and reports RXFTO with no preamble detected.
     * Only meaningful when a Poll was actually received (success, or
     * error == TxStartFailed).
     *
     * 遅延送信を予約した瞬間に、予定送信時刻まで何µs残っていたか。負なら
     * 締切に間に合っておらず、dwt_starttx() は HPDWARN を見て**送信ごと
     * 取り消す**（dw3720_device.c の ull_starttx()）。相手からは「電波が
     * 来ていない」のと区別がつかない失敗になる。
     *
     * これは docs/TIMING_PRESETS.md §1.3 が見積りで置いた「折返しに要る
     * 時間」（ポーリング 約1.2ms / IRQ 約0.3ms）を実機で直接測るための値。
     * responseTxDelayUus の実µs値からこの残りを引けば、折返しに実際に
     * かかった時間が出る。
     */
    int32_t txMarginUs = 0;
    /**
     * Received signal level (whole-channel power) for the Ipatov CIR,
     * captured via detail::readRxPower() on both success and failure. Signed
     * Q8.8 dBm (real dBm = value / 256.0); INT16_MIN means the CIA
     * diagnostics could not be read. Check rxAccumCount before trusting.
     * Ipatov CIR（前置信号相関器）のチャネル全体の受信電力（RSL）。
     * detail::readRxPower() で取得し、成功時・失敗時の両方で埋まる。符号付き
     * Q8.8 形式の dBm（実 dBm = 値/256.0）。INT16_MIN は CIA 診断が読めな
     * かったことを示す。信用してよいかは rxAccumCount も見て判断すること。
     */
    int16_t rslDbmQ8 = INT16_MIN;
    /**
     * First-path signal power for the Ipatov CIR - the power of just the
     * earliest-arriving path, as opposed to rslDbmQ8's whole-channel power.
     * Same encoding and caveats as rslDbmQ8 (signed Q8.8 dBm, INT16_MIN =
     * unavailable, check rxAccumCount before trusting).
     * Ipatov CIR の第一波（最初に到達したパス）だけの受信電力。符号化・
     * 注意点は rslDbmQ8 と同じ（符号付き Q8.8 dBm、INT16_MIN = 取得不可。
     * 信用してよいかは rxAccumCount も見て判断すること）。
     */
    int16_t fpDbmQ8 = INT16_MIN;
    /**
     * Number of symbols the CIA accumulated for the Ipatov CIR when
     * rslDbmQ8 / fpDbmQ8 were captured. 0 means the CIA never accumulated
     * anything (e.g. RXFTO with no preamble detected at all), in which case
     * rslDbmQ8 / fpDbmQ8 must not be trusted even when they are not
     * INT16_MIN.
     * rslDbmQ8 / fpDbmQ8 を取得した時点で CIA が Ipatov CIR に蓄積した
     * シンボル数。0 は CIA が何も蓄積していない（例: プリアンブルすら
     * 検出されない RXFTO）ことを意味し、その場合 rslDbmQ8 / fpDbmQ8 が
     * INT16_MIN でなくても信用してはならない。
     */
    uint16_t rxAccumCount = 0;
};

/**
 * @brief Result returned by Qm33120::respondDSRange() (DS-TWR responder).
 * 1:1 with M5Stamp_UWBDSResponderResult (M5Stamp_UWB_Types.h:250-258). This
 * is the side that actually computes distanceMm/distanceM (respondDSRange()
 * cpp:1294-1319).
 */
struct DSResponderResult {
    bool success       = false;
    uint8_t sequence   = 0;
    uint16_t requester = 0;
    int32_t distanceMm = 0;
    float distanceM    = 0.0f;
    uint32_t elapsedMs = 0;
    Error error         = Error::Ok;
    /**
     * Microseconds left before the scheduled delayed transmission when
     * dwt_starttx(DWT_START_TX_DELAYED) was issued. Negative means the
     * deadline had passed, in which case the chip raises HPDWARN and the
     * frame is CANCELLED, not sent late - so the peer sees "nothing arrived".
     * Same meaning as ResponderResult::txMarginUs; see that comment.
     * 遅延送信を予約した瞬間の締切までの残り [µs]。負なら送信は取り消される。
     * 意味は ResponderResult::txMarginUs と同じ（そちらのコメント参照）。
     */
    int32_t txMarginUs = 0;
};

} // namespace uwb
