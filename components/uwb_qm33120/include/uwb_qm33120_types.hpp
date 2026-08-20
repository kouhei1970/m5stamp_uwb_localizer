/**
 * @file uwb_qm33120_types.hpp
 * @brief Configuration / result / error types for uwb::Qm33120.
 *
 * Arduino-free, ESP-IDF-native port of the public types in
 * third_party/M5Stamp-UWB/src/M5Stamp_UWB_Types.h (M5Stack Technology CO LTD,
 * MIT). Field sets and default values are carried over 1:1 from that file
 * except where the Arduino-specific SPI/GPIO representation had to change
 * (see uwb::Config below) - see PROGRESS.md Phase 2 Step 1 for the full list
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
 * Phase 2R（docs/REIMPL_PLAN.md）での変更点: 一次資料（Qorvo API rev9p3 /
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
 *    （M5Stamp UWB Module with FPCが実際に使う唯一のチャネル）の既定値自体は変更なし。
 */
#pragma once

#include <cstddef>
#include <cstdint>

#include "uwb_port.h"

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
    int pin_irq                 = UWB_PORT_PIN_UNUSED; //!< optional (unused: polling driver, see SURVEY_m5stamp_uwb_port.md).
    int pin_wakeup               = UWB_PORT_PIN_UNUSED; //!< optional WAKEUP pin.
    int pin_gp7                  = UWB_PORT_PIN_UNUSED; //!< optional DW_GP7 input pin.
    uint32_t spi_slow_hz         = 2000000;             //!< SPI speed used before/during probe (M5Stamp_UWBConfig.spi_slow_hz).
    uint32_t spi_fast_hz         = 16000000;            //!< SPI speed switched to after dwt_probe()/dwt_initialise() (M5Stamp_UWBConfig.spi_fast_hz).
    bool init_spi_bus            = true;                //!< false => caller already ran spi_bus_initialize() on spi_host (uwb_port_config_t.init_spi_bus).

    /* --- Qm33120 固有（原本 M5Stamp_UWBConfig の残りのフィールドをそのまま踏襲） --- */
    uint8_t probe_retry_count     = 5;   //!< number of Device ID/probe retries during begin() (M5Stamp_UWBConfig.probe_retry_count).
    uint16_t probe_retry_delay_ms = 20;  //!< delay between Device ID/probe retries (M5Stamp_UWBConfig.probe_retry_delay_ms).
    bool hard_reset_on_begin      = true; //!< toggle RESET before probing the UWB device (M5Stamp_UWBConfig.hard_reset_on_begin).

    /**
     * @brief If true, begin() does NOT call uwb_port_init() (or uwb_port_deinit()
     * in end()/on failure): it assumes another owner already initialized
     * uwb_port (e.g. a shared SPI bus set up once by platform bootstrap code).
     * Has no equivalent in the original (single-owner Arduino SPIClass model).
     * Intended for the future StampFly integration, where uwb_port may be
     * initialized once for a whole board rather than per Qm33120 instance.
     */
    bool port_already_initialized = false;
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
 * 【docs/REIMPL_PLAN.md R7】上記「built-in recommended profile」の実体
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
     * 【docs/REIMPL_PLAN.md R8】旧既定値は固定の129だった。これは既定の
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

/**
 * @brief Single-sided two-way ranging timing and address configuration.
 * 1:1 with M5Stamp_UWBRangeConfig (M5Stamp_UWB_Types.h:184-192), including
 * default values.
 *
 * --- 単位について（docs/REIMPL_PLAN.md R1） ---
 * `*Uus` フィールドの単位は UUS (UWB microsecond)。
 * 1 UUS = 512/499.2 us = 1.02564... us
 * (`deca_device_api.h:2681` dwt_setrxaftertxdelay() 「The delay is in UWB
 * microseconds」/ 同2360行 dwt_setrxtimeout() 「in 1.0256 us
 * (512/499.2MHz) units」より)。以下2フィールドはこの単位のまま SDK API に
 * 直接渡り、`responseTxDelayUus` だけ `uwb::detail::kUusToDwtTime`(=65536,
 * uwb_qm33120_internal.hpp) 倍して DTU (1 DTU=15.65ps) に変換してから
 * `dwt_setdelayedtrxtime()` へ渡す（uwb_qm33120_twr.cpp 参照。関数ごとの
 * 行はフィールドコメント参照）。
 * 【罠】Qorvo公式サンプルの `*_UUS` 定数（例:
 * `POLL_RX_TO_RESP_TX_DLY_UUS`）は実マイクロ秒であり、このUUSとは値が違う
 * （65536/63898≒1.0256倍）。そのまま代入すると約2.5%長くなる。移植する
 * ときは `uwb::detail::usToUus()`（uwb_qm33120_units.hpp）で変換すること。
 */
struct RangeConfig {
    uint16_t panId                     = 0xDECA;
    uint16_t initiatorAddress          = 0x0001;
    uint16_t responderAddress          = 0x0002;
    //!< UUS。dwt_setrxaftertxdelay() に直接渡る
    //!< (uwb_qm33120_twr.cpp: requestRange:76, respondRange は即時応答なので0固定:174)。
    uint32_t responseRxAfterTxDelayUus = 500;
    //!< UUS。× kUusToDwtTime してDTU化し dwt_setdelayedtrxtime() に渡る
    //!< (uwb_qm33120_twr.cpp: requestRange:214,226 で応答側の遅延送信時刻を計算)。
    uint32_t responseTxDelayUus        = 3000;
    //!< UUS。dwt_setrxtimeout() に直接渡る
    //!< (uwb_qm33120_twr.cpp: requestRange:77, respondRange は即時応答なので0固定:175)。
    uint32_t rxTimeoutUus              = 4500;
    /**
     * ms。ホスト側ポーリングループのタイムアウト。SDK APIには渡らない
     * (detail::nowMs() ベース。requestRange/respondRange 双方で使用)。
     *
     * 【docs/REIMPL_PLAN.md R9】旧既定値100はチップ側のRXタイムアウト
     * (rxTimeoutUus=4500UUS≈4.615ms)の20倍以上あり、5アンカー構成では
     * 「応答が来ない」ケース1つで最大100msストールしうる。
     * 【R2適用後の根拠】R2により、不一致フレームを受信するたびに
     * dwt_rxenable()でRXタイムアウトが再アームされる（本ファイルの
     * uwb_qm33120_twr.cppコメント、ull_setrxtimeout()の解析を参照）ため、
     * hostTimeoutMsは「チップ側1回分のRXタイムアウト」ではなく「不一致
     * フレームが何度再アームされても最終的に抜けるための上位バックストップ」
     * として機能する。10msは、チップ側の最大RXタイムアウト(4500UUS≈4.615ms)
     * を1回分丸ごと待っても打ち切らない値でありながら、不一致フレーム1枚
     * あたりの処理時間（フレーム受信+照合、air timeにして高々百数十µs
     * オーダー、docs/CRITICAL_REVIEW.md「フレーム air time の実測算」参照）
     * を何十回分も吸収できる余裕を残す。100msの1/10にすることで
     * 「5アンカー×hostTimeoutMs」のストール上限も同じ比率で縮む。
     */
    uint32_t hostTimeoutMs             = 10;
    /**
     * 【docs/REIMPL_PLAN.md R4】requestRange()（SS-TWR initiator）のToF計算に
     * dwt_readclockoffset() によるクロックオフセット補正を適用するか。
     * 既定で有効（true）。無効にすると旧挙動（無補正、rtd_resp係数=1固定）
     * に戻る。実機での比較検証用のフラグ（respondRange 側には効果はない。
     * SS-TWR responder は補正の主体ではなくフレームにタイムスタンプを
     * 載せるだけなので変更不要）。詳細は uwb_qm33120_twr.cpp
     * requestRange() 内のコメント参照。
     */
    bool enableClockOffsetCorrection    = true;
};

/**
 * @brief Double-sided two-way ranging timing and address configuration.
 * 1:1 with M5Stamp_UWBDSRangeConfig (M5Stamp_UWB_Types.h:197-210), including
 * default values.
 *
 * --- 単位について（docs/REIMPL_PLAN.md R1。RangeConfig と同じ規則） ---
 * `*Uus` フィールドの単位は UUS。1 UUS = 512/499.2 us = 1.02564... us。
 * `responseTxDelayUus` / `finalTxDelayUus` の2つだけが
 * `uwb::detail::kUusToDwtTime`(=65536) 倍されて DTU に変換され
 * `dwt_setdelayedtrxtime()` へ渡る。残りは UUS のまま
 * `dwt_setrxaftertxdelay()` / `dwt_setrxtimeout()` に直接渡る。
 * Qorvo公式の `*_UUS` 定数（実マイクロ秒）をそのまま代入しないこと
 * （`uwb::detail::usToUus()` で変換する。RangeConfig のコメント参照）。
 */
struct DSRangeConfig {
    uint16_t panId                          = 0xDECA;
    uint16_t initiatorAddress               = 0x0001;
    uint16_t responderAddress               = 0x0002;
    //!< UUS。dwt_setrxaftertxdelay() に直接渡る。Poll送信後、Response受信を
    //!< 開始するまでの待ち（uwb_qm33120_twr.cpp: requestDSRange:301）。
    uint32_t responseRxAfterTxDelayUus      = 1500;
    //!< UUS。× kUusToDwtTime してDTU化し dwt_setdelayedtrxtime() に渡る。
    //!< Responder側がPoll受信時刻を基準に自分のResponse遅延送信時刻を計算
    //!< するのに使う（uwb_qm33120_twr.cpp: respondDSRange:499,511）。
    uint32_t responseTxDelayUus             = 3000;
    //!< UUS。× kUusToDwtTime してDTU化し dwt_setdelayedtrxtime() に渡る。
    //!< Initiator側がResponse受信時刻を基準に自分のFinal遅延送信時刻を計算
    //!< するのに使う（uwb_qm33120_twr.cpp: requestDSRange:359,371）。
    uint32_t finalTxDelayUus                = 1800;
    //!< UUS。dwt_setrxaftertxdelay() に直接渡る。Responder側が自分の
    //!< 遅延Response送信後、Final受信を開始するまでの待ち
    //!< （uwb_qm33120_twr.cpp: respondDSRange:512）。
    uint32_t finalRxAfterResponseTxDelayUus = 500;
    /**
     * UUS。dwt_setrxaftertxdelay() に直接渡る。Initiator側がFinal送信後、
     * 結果("DWD")受信を開始するまでの待ち
     * （uwb_qm33120_twr.cpp: requestDSRange:372）。
     *
     * 【docs/REIMPL_PLAN.md R3-1】旧既定値500 UUS(≈512.8µs実us)は、
     * DWD結果フレームの送信がAnchor側で「delayed TXで時刻を予約する」
     * のではなく「Final受信直後にDWT_START_TX_IMMEDIATEで即時送信する」
     * 方式（respondDSRange()、SPIレジスタ書き込み数回のみでvTaskDelay等の
     * 意図的な遅延を挟まない）であるにもかかわらず大きすぎた。
     * Initiator側のRXが512.8µs後まで開かないため、Anchorがそれより速く
     * 返信した場合は最初のDWD送信を取りこぼす
     * （docs/CRITICAL_REVIEW.md【重大3】、L<363µsで約36%取りこぼしの実測算）。
     * 200 UUS(≈205.1µs実us)へ下げた根拠: DWDフレーム自体の空中線上の
     * 全長（SHR 138.4µs [preamble128+SFD8=136シンボル×1017.63ns/シンボル、
     * docs/refs/DW3000_Datasheet_wayback.txt Table 18（§4.4）より本値を
     * 独立に再検算し一致を確認] + PHR + データ、docs/CRITICAL_REVIEW.md
     * 「フレーム air time の実測算」より合計約179µs）をわずかに上回る
     * 205.1µsであれば、Anchor側の実処理時間（意図的な遅延なしの
     * SPIレジスタ書き込み数回、既存の1ms輪ポーリング実装でも数十〜百数十µs
     * オーダーと見積もれる）に対して十分な余裕がありながら、
     * 旧値(512.8µs)の半分以下に圧縮できる。
     */
    uint32_t resultRxAfterFinalTxDelayUus   = 200;
    //!< UUS。dwt_setrxtimeout() に直接渡る。Response/Final/結果それぞれの
    //!< 受信タイムアウトに共用される
    //!< （uwb_qm33120_twr.cpp: requestDSRange:302,373, respondDSRange:513）。
    uint32_t rxTimeoutUus                   = 3000;
    /**
     * ms。ホスト側ポーリングループのタイムアウト。SDK APIには渡らない。
     * 【docs/REIMPL_PLAN.md R9】RangeConfig::hostTimeoutMs と同じ理由・
     * 同じ根拠で100→10に変更（コメント参照）。DS-TWRのrxTimeoutUus=3000
     * (≈3.077ms)に対しても10msは1回分丸ごと待っても打ち切らない値。
     */
    uint32_t hostTimeoutMs                  = 10;
    /**
     * 単位なし（回数）。SDK APIには渡らない。Responder が計算した距離
     * ("DWD"結果フレーム)を送る回数（uwb_qm33120_twr.cpp: respondDSRange:603）。
     *
     * 【docs/REIMPL_PLAN.md R3-1】旧既定値3は、resultRepeatGapMs=3ms /
     * rxTimeoutUus=3000UUS(=3.077ms)と一度も整合していなかった
     * (docs/CRITICAL_REVIEW.md【重大3】)。DWD#3はTagのDWD受信ウィンドウが
     * 閉じた後(ウィンドウ終端の2〜3.5ms後)に送信されるため物理的に受信
     * 不能な上、次のAnchorの応答窓(8.6〜11.7ms)のど真ん中に着弾して
     * 次のリンクの測距を巻き添えにする。既定を1にしてこの破壊的な
     * リピートを止める。リピート機構自体は削除しない（respondDSRange()の
     * for(i<repeatCount)ループはそのまま残っており、呼び出し側がこの
     * フィールドを2以上に設定すれば実機比較検証用に旧挙動へ戻せる。
     * firmware/anchor, firmware/twr は現状Kconfig化しておらず
     * main.cpp内のstatic constexprで直接指定している）。
     */
    uint8_t resultRepeatCount               = 1;
    //!< ms。SDK APIには渡らない。結果フレーム再送の間隔
    //!< （uwb_qm33120_twr.cpp: respondDSRange:630, vTaskDelay）。
    //!< resultRepeatCount>1のときのみ意味を持つ（R3-1で既定1になったため
    //!< 既定構成では未使用）。
    uint32_t resultRepeatGapMs              = 3;
};

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
     * 【docs/REIMPL_PLAN.md R4】requestRange() が dwt_readclockoffset() から
     * 求めたクロックオフセット比を ppm 単位で入れる（実機デバッグ用の可視化。
     * distanceM/distanceMm の算出には既に織り込み済みで、この値自体は
     * 加算・減算しない）。RangeConfig::enableClockOffsetCorrection が false
     * のとき、または成功しなかった呼び出しでは 0.0（未計算）のまま。
     * CIAが動作していないとdwt_readclockoffset()は0を返す
     * (DW3720 API Guide §5.4.13) ため、その場合もここは0になる。
     */
    float clockOffsetPpm = 0.0f;
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
};

} // namespace uwb
