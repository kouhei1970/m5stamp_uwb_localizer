/**
 * @file uwb_qm33120_internal.hpp
 * @brief Private implementation-detail helpers shared by uwb_qm33120's
 * translation units.
 *
 * NOT a public header (lives under src/, not include/): it is not part of
 * uwb::Qm33120's public API, only #include'd by this component's own .cpp
 * files.
 *
 * These are 1:1 ports of the file-scope `static` helper functions in
 * third_party/M5Stamp-UWB/src/M5Stamp_UWB.cpp (lines 264-386 for the bit/
 * frame helpers, 787/1012/1165 for the TWR constants). In the original they
 * live directly in the single M5Stamp_UWB.cpp file; here they are split into
 * their own header because uwb_qm33120 spans two translation units by
 * design (src/uwb_qm33120.cpp for Phase 2 Step 1 - device/PHY/frame I/O -
 * and the future src/uwb_qm33120_twr.cpp for Phase 2 Step 2 - TWR), and
 * `static` functions defined directly in uwb_qm33120.cpp would not be
 * visible from uwb_qm33120_twr.cpp. Each function here stays `static`
 * (internal linkage), exactly like the original: including this header from
 * two .cpp files is safe (each TU gets its own private copy, no ODR/link
 * issues), it is just no longer literally "file-scope in one file".
 *
 * Everything below takes explicit parameters / touches only global dwt_*
 * state, same as the original - none of it needs access to Qm33120::Impl,
 * so plain free functions (not class members) are the faithful translation.
 */
#pragma once

#include <climits>
#include <cstdint>
#include <cstring>

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "deca_device_api.h"
#include "deca_private.h"     // dwt_readfromdevice() - raw register read, same pattern as firmware/twr, firmware/probe.
#include "dw3720_deca_regs.h" // SYS_STATE_LO_ID/LEN - both qm33120w_sdk's public INCLUDE_DIRS ("." / "dw3720").

#include "uwb_qm33120_frame_match.hpp"
#include "uwb_qm33120_timing.hpp" // kTimingPresetVersion - used by buildAndArmResponse() below.
#include "uwb_qm33120_types.hpp"

namespace uwb::detail {

/**
 * @brief SDK側の FCS_LEN が、frame_match.hpp が複製した detail::kFcsLen と
 * 今も一致していることのビルド時検算（docs/archive/REIMPL_PLAN.md R2、G-2）。
 * uwb_qm33120_frame_match.hpp はESP-IDF/Qorvo SDKに依存できないため
 * kFcsLen=2を独立に持っている。SDK側 (`deca_device_api.h:236`) が変わったら
 * ここでビルドが壊れて気づける。
 */
static_assert(detail::kFcsLen == FCS_LEN, "uwb::detail::kFcsLen (uwb_qm33120_frame_match.hpp) must match the SDK's FCS_LEN (deca_device_api.h)");

/* ---------------------------------------------------------------------
 * millis() 相当。esp_timer_get_time() (単調・64bit・us単位) を ms に丸めて
 * 32bit へ縮小する。以降の使い方は原本の `millis() - startMs < timeoutMs`
 * パターンをそのまま踏襲しており、符号なし32bit減算は巻き戻り（約49.7日
 * 周期）をまたいでも正しく差分を返す（標準的な wraparound-safe idiom）。
 * esp_timer_get_time() 自体は起動からの単調カウンタなので巻き戻りの起点は
 * このuint32_tへの縮小のみであり、Arduino millis()と同じ性質になる。
 * ---------------------------------------------------------------------
 */
static inline uint32_t nowMs()
{
    return static_cast<uint32_t>(esp_timer_get_time() / 1000);
}

/* --- 16/32/40bit little-endian バッファ読み書き（cpp:264-296） --- */

static inline void set16le(uint8_t* dst, uint16_t value)
{
    dst[0] = static_cast<uint8_t>(value & 0xFF);
    dst[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
}

static inline void set32le(uint8_t* dst, uint32_t value)
{
    dst[0] = static_cast<uint8_t>(value);
    dst[1] = static_cast<uint8_t>(value >> 8);
    dst[2] = static_cast<uint8_t>(value >> 16);
    dst[3] = static_cast<uint8_t>(value >> 24);
}

static inline uint16_t get16le(const uint8_t* src)
{
    return static_cast<uint16_t>(src[0]) | (static_cast<uint16_t>(src[1]) << 8);
}

static inline uint32_t get32le(const uint8_t* src)
{
    return static_cast<uint32_t>(src[0]) | (static_cast<uint32_t>(src[1]) << 8) |
           (static_cast<uint32_t>(src[2]) << 16) | (static_cast<uint32_t>(src[3]) << 24);
}

static inline uint64_t get40le(const uint8_t* src)
{
    uint64_t value = 0;
    for (int i = 4; i >= 0; --i) {
        value = (value << 8) | src[i];
    }
    return value;
}

/* Step 2 (TWR) 用に前倒しで移植。 現時点(Step 1)では未使用。
 * cpp:298-310 の readTxTimestamp64()/readRxTimestamp64() に対応。
 * dwt_ip_sts_segment_e の 0 キャストは原本のまま（Ipatov セグメント）。 */

static inline uint64_t readTxTimestamp64()
{
    uint8_t ts[5] = {0};
    dwt_readtxtimestamp(ts);
    return get40le(ts);
}

static inline uint64_t readRxTimestamp64()
{
    uint8_t ts[5] = {0};
    dwt_readrxtimestamp(ts, static_cast<dwt_ip_sts_segment_e>(0));
    return get40le(ts);
}

/* --- 短アドレス形式 IEEE 802.15.4 フレームの構築/解析（cpp:312-356） ---
 * kShortAddressHeaderLen / shortAddressFrameLength() は
 * uwb_qm33120_frame_match.hpp（ESP-IDF/Qorvo SDK非依存）に移した。
 * uwb::detail 名前空間なので、以下のコードから見た参照先（表記）は変わらない。
 */

static inline void buildShortAddressFrame(uint8_t* frame, uint8_t sequence, uint16_t panId, uint16_t src,
                                           uint16_t dst, const uint8_t* payload, size_t payloadLength)
{
    frame[0] = 0x41;
    frame[1] = 0x88;
    frame[2] = sequence;
    set16le(&frame[3], panId);
    set16le(&frame[5], dst);
    set16le(&frame[7], src);
    memcpy(&frame[9], payload, payloadLength);
}

static inline bool parseShortAddressFrame(const uint8_t* frame, uint16_t frameLen, RxResult& result)
{
    if ((frame == nullptr) || (frameLen < (kShortAddressHeaderLen + FCS_LEN))) {
        return false;
    }
    if ((frame[0] != 0x41) || (frame[1] != 0x88)) {
        return false;
    }

    result.sequence      = frame[2];
    result.panId         = get16le(&frame[3]);
    result.dst           = get16le(&frame[5]);
    result.src           = get16le(&frame[7]);
    result.payloadLength = frameLen - kShortAddressHeaderLen - FCS_LEN;
    result.frameLength   = frameLen;
    result.ranging       = false;
    return true;
}

/* payloadMatches() / payloadMatchesEither() / readTimingTag() も
 * uwb_qm33120_frame_match.hpp に移した（同じ理由・同じ uwb::detail
 * 名前空間）。呼び出し側からの見え方・シグネチャは変わらない。 */

/**
 * @brief 遅延送信の締切まで何µs残っているかを、チップ自身の時計で測る。
 *
 * `dxTime` は dwt_setdelayedtrxtime() に渡した値（40bit システム時刻の
 * 上位32bit = DX_TIME レジスタと同じ単位・同じ原点）。
 * dwt_readsystimestamphi32() は SYS_TIME レジスタ（同じく上位32bit）を
 * 読むので、両者は直接引き算できる。1単位 = 256 DTU = 256 x DWT_TIME_UNITS
 * ≈ 4.0064 ns。符号付き32bitで引くことで、約17.2秒周期の巻き戻りを
 * またいでも正しい残り時間になる（標準的な wraparound-safe idiom）。
 *
 * 戻り値が負なら締切に間に合っていない。その状態で dwt_starttx() を
 * 呼ぶとチップは HPDWARN を立て、SDK は CMD_TXRXOFF を発行して
 * **送信そのものを取り消し** DWT_ERROR を返す
 * （components/qm33120w_sdk/dw3720/dw3720_device.c の ull_starttx()）。
 * 相手からは「電波が来ていない」（RXFTO のみ・プリアンブル検出なし）と
 * 区別できない失敗に見えるので、この値を持ち出さないと原因を切り分け
 * られない。docs/TIMING_PRESETS.md §1.3 が見積りで置いた折返し時間
 * （ポーリング 約1.2ms / IRQ 約0.3ms）を実機で直接検算するための計器。
 *
 * Measures, on the chip's own clock, how long is left before a scheduled
 * delayed transmission. Negative means the deadline has passed, in which
 * case dwt_starttx() cancels the frame instead of sending it late.
 *
 * @param dxTime dwt_setdelayedtrxtime() に渡した値。
 * @return 締切までの残り時間 [µs]（負なら超過）。
 */
static inline int32_t delayedTxMarginUs(uint32_t dxTime)
{
    static constexpr double kDxTimeUnitUs = 256.0 * DWT_TIME_UNITS * 1.0e6;
    const uint32_t nowHi                   = dwt_readsystimestamphi32();
    const int32_t marginTicks              = static_cast<int32_t>(dxTime - nowHi);
    return static_cast<int32_t>(marginTicks * kDxTimeUnitUs);
}

/* --- ステータス→エラー変換、無線停止＋ステータスクリア（cpp:358-386） --- */

static inline Error rxStatusToError(uint32_t status)
{
    if ((status & ((uint32_t)DWT_INT_RXFTO_BIT_MASK | (uint32_t)DWT_INT_RXPTO_BIT_MASK)) != 0) {
        return Error::RxTimeout;
    }
    return Error::RxError;
}

static inline void stopRadioAndClearStatus(uint32_t statusMask)
{
    dwt_forcetrxoff();
    dwt_writesysstatuslo(statusMask);
}

static inline void stopRadioAndClearRxStatus()
{
    stopRadioAndClearStatus(SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_GOOD);
}

static inline void stopRadioAndClearTxStatus()
{
    stopRadioAndClearStatus(DWT_INT_TXFRS_BIT_MASK);
}

static inline void stopRadioAndClearIoStatus()
{
    stopRadioAndClearStatus(SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_GOOD |
                             DWT_INT_TXFRS_BIT_MASK);
}

/**
 * Received-power indication for the frame the radio last worked on.
 * 直前に受信機が扱ったフレームの受信電力の指標。
 * rslQ8 / fpQ8 are signed Q8.8 dBm (real dBm = value / 256.0); INT16_MIN means
 * "not available". accumCount == 0 means the CIA never accumulated anything
 * (e.g. RXFTO with no preamble), so the power values must not be trusted.
 * rslQ8 / fpQ8 は符号付き Q8.8 形式の dBm（実 dBm = 値/256.0）。INT16_MIN は
 * 取得不可。accumCount == 0 は CIA が何も蓄積していない（前置信号すら来て
 * いない）ことを意味し、そのときの電力値は信用してはならない。
 */
struct RxPower {
    int16_t  rslQ8       = INT16_MIN;
    int16_t  fpQ8        = INT16_MIN;
    uint16_t accumCount  = 0;
    uint32_t ipatovPower = 0;
};

/**
 * @brief dwt_readdiagnostics_acc() / dwt_calculate_rssi() /
 * dwt_calculate_first_path_power() をまとめて呼び、Ipatov（前置信号相関器、
 * DWT_ACC_IDX_IP_M）チャネルの受信電力指標を読み出す。
 * dwt_configciadiag(DW_CIA_DIAG_LOG_ALL) を（uwb_qm33120.cpp の init() で）
 * 一度呼んでいることが前提 - 呼んでいないと診断レジスタは常に0になる
 * （components/qm33120w_sdk/deca_device_api.h:3517-3518）。
 * 呼び出し側は、フレームを最後に扱った直後・かつ
 * stopRadioAndClearRxStatus() 等でステータスをクリアする前に呼ぶこと
 * （順序を間違えると診断レジスタの内容が消える）。
 *
 * Calls dwt_readdiagnostics_acc() / dwt_calculate_rssi() /
 * dwt_calculate_first_path_power() together to read the Ipatov
 * (DWT_ACC_IDX_IP_M) channel's received-power indication. Assumes
 * dwt_configciadiag(DW_CIA_DIAG_LOG_ALL) has already been called once (in
 * uwb_qm33120.cpp's init()) - otherwise the diagnostic registers always read
 * 0 (deca_device_api.h:3517-3518). Call this right after the radio last
 * worked on a frame and before anything clears the status registers (e.g.
 * stopRadioAndClearRxStatus()) - reversing the order loses the diagnostics.
 */
static inline RxPower readRxPower()
{
    RxPower result;
    dwt_cirdiags_t diag = {};
    if (dwt_readdiagnostics_acc(&diag, DWT_ACC_IDX_IP_M) == DWT_SUCCESS) {
        result.accumCount  = diag.accumCount;
        result.ipatovPower = diag.power;
        dwt_calculate_rssi(&diag, DWT_ACC_IDX_IP_M, &result.rslQ8);
        dwt_calculate_first_path_power(&diag, DWT_ACC_IDX_IP_M, &result.fpQ8);
    }
    return result;
}

/* Step 2 (TWR) 用に前倒しで用意した定数。cpp:787/1012/1165 で3回重複定義されて
 * いた speedOfLight と、cpp:944/1086/1225/1236他で使われる uusToDwtTime を
 * ここに集約する（Step 1 では未使用）。値は原本のまま。 */
static constexpr double kSpeedOfLightMPerS = 299702547.0; //!< 実効値（真空中の光速ではない）。cpp:787,1012,1165。
static constexpr uint64_t kUusToDwtTime    = 65536ULL;

/**
 * @brief DW3000 UM §9.4.1 の「遅延送信が無警告で送信されない」エラッタを
 * チップの生レジスタから検出する（2026-08-29 DS-TWR原因特定、
 * docs/HANDOFF.md §0-C(2)）。
 *
 * 背景: DS PollingBoth プリセットの旧 `finalTxDelayUus`（1800 UUS）は、
 * 850 kbps / preamble 256（本番機の実運用値）では、タグが Final の
 * dwt_starttx(DWT_START_TX_DELAYED) を呼ぶ時点で予約締切まで
 * 0.03〜1.0 ms しか残っていないケースがあった。DW3000 UM §9.4.1 は、
 * 予約時刻が「現在時刻 + プリアンブル長 + SFD 長 + 20 µs」より前だと
 * **HPDWARN すら立たず、dwt_starttx() は DWT_SUCCESS を返すのに実際には
 * 送信されない**と規定している。この状態でチップの SYS_STATE_LO
 * （0xF0030、下記 dw3720_deca_regs.h 参照）を読むと 0x000D0000
 * （PMSC_STATE=0xD=TX、TX_STATE=0=IDLE）に固まったままになる
 * （DW3000 User Manual Version 1.1 §9.4.1、Page 239。原文は docs/HANDOFF.md §0-C「証拠 2」に引用）。
 *
 * dwt_starttx() の DW3720 用実装 `ull_starttx()`
 * （components/qm33120w_sdk/dw3720/dw3720_device.c）は HPDWARN のみを
 * 見て判定するため、このエラッタのケースを検出できない（HPDWARNが立たない
 * のがこのエラッタの症状そのものなので）。dw3000用ドライバの同名関数には
 * `DW_SYS_STATE_TXERR`(=0xD0000、SYS_STATE_LOのビット位置に一致)を見る
 * 追加チェックがあるが、dw3720用にはこれが移植されていない
 * （`assets/DW3_QM33_SDK_1.1.1/Drivers/API/Shared/dwt_uwb_driver/dw3000/
 * dw3000_device.c:5253-5262`）。本関数はそのチェックをラッパ側で補う。
 *
 * The DW3000 UM §9.4.1 errata: if a delayed-TX deadline is less than
 * (now + preamble + SFD + 20 us) in the future, dwt_starttx() returns
 * DWT_SUCCESS but neither HPDWARN is set nor is the frame transmitted;
 * SYS_STATE_LO then reads 0x000D0000 (PMSC_STATE=TX, TX_STATE=IDLE). The
 * DW3720 driver we compile only checks HPDWARN (unlike the dw3000 driver,
 * which also checks DW_SYS_STATE_TXERR), so it misses this case - this
 * helper detects it directly from the raw register.
 *
 * @return true なら上記のエラッタ状態（無警告で未送信）にある。
 */
static inline bool delayedTxWedged()
{
    uint8_t buf[SYS_STATE_LO_LEN] = {0, 0, 0, 0};
    dwt_readfromdevice(SYS_STATE_LO_ID, 0, sizeof(buf), buf);
    const uint32_t sysStateLo = (static_cast<uint32_t>(buf[3]) << 24) | (static_cast<uint32_t>(buf[2]) << 16) |
                                (static_cast<uint32_t>(buf[1]) << 8) | static_cast<uint32_t>(buf[0]);
    return sysStateLo == 0x000D0000UL;
}

/**
 * @brief delayedTxWedged() が true のとき、送信を中断してステータスを
 * クリアする（2026-08-29 DS-TWR原因特定）。
 *
 * dwt_forcetrxoff() は「TSE が IDLE より上の状態にあるときだけ
 * CMD_TXRXOFF を発行する」（`ull_forcetrxoff()`,
 * components/qm33120w_sdk/dw3720/dw3720_device.c: `if (!(dwt_read8bitoffsetreg(
 * dw, SYS_STATE_LO_ID, 2U) <= DW_SYS_STATE_IDLE))`）。このエラッタの
 * 状態はチップが PMSC_STATE=0xD（> DW_SYS_STATE_IDLE=3）を報告している
 * ので、dwt_forcetrxoff() は実際に CMD_TXRXOFF を発行して送信を止める。
 * その後 dwt_writesysstatuslo() で TXFRS・RX系ビットをまとめてクリアし、
 * 呼び出し側の後続コード（次のRX起動等）が古いステータスを誤読しないよう
 * にする。
 *
 * delayedTxWedged() returning true means the chip reports PMSC_STATE=0xD,
 * which is > DW_SYS_STATE_IDLE(3), so dwt_forcetrxoff()'s internal state
 * check does issue CMD_TXRXOFF (see ull_forcetrxoff() cited above).
 *
 * @return true なら中断した（=呼び出し元は送信が行われなかったものとして
 * 扱うこと）。false なら delayedTxWedged() が false だったので何もしていない。
 */
static inline bool abortIfDelayedTxWedged()
{
    if (!delayedTxWedged()) {
        return false;
    }
    dwt_forcetrxoff();
    dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR |
                          SYS_STATUS_ALL_RX_GOOD);
    return true;
}

/* ---------------------------------------------------------------------
 * docs/ARCHITECTURE_V2.md §1 (2026-08-30 追記): "旧 respondRange()/
 * respondDSRange() と新 Responder は、フレーム生成・遅延送信の予約・
 * エラッタ検査・距離計算を detail:: の補助関数で共有し、コピーしない"。
 *
 * 以下の3組は、respondRange()/respondDSRange()（uwb_qm33120_twr.cpp、
 * ロジック・数式・バイト位置は一切変更しない）と uwb::Responder
 * （uwb_qm33120_responder.cpp、新規）の両方が呼ぶ共通実装。呼び出し側の
 * 失敗時クリーンアップ（stopRadioAndClearTxStatus() か
 * stopRadioAndClearIoStatus() か。SS/DS で元々違う）はここに含めない —
 * 呼び出し側の責務のまま据え置くことで、2関数間の既存の違いを壊さずに
 * 済む（詳細は各関数のコメント）。
 * --------------------------------------------------------------------- */

/**
 * @brief Response（"TWR"/"DWR"）を組み立てて遅延送信を予約する
 * （respondRange() cpp:604-663 / respondDSRange() cpp:1179-1238 相当）。
 *
 * SS は `rxAfterTxDelayUus=0, rxTimeoutUus=0, responseExpected=false`
 * （respondRange() が受信ループの先頭で一度だけ設定した値を、ここでも
 * 同じ値で再設定するだけなので無害 - 直前に別の値を設定するコードは
 * どこにも無い）。DS は `rxAfterTxDelayUus=range.finalRxAfterResponseTxDelayUus,
 * rxTimeoutUus=range.rxTimeoutUus, responseExpected=true` を渡す。
 *
 * 呼び出し側の失敗時クリーンアップ（dwt_forcetrxoff 相当）は含まない
 * （上のコメント参照）。`pollRxTsOut` には、DS 側が Final 到着時の距離
 * 計算（computeDsDistance()）で再び必要とする Poll の RX タイムスタンプ
 * （40bit チップ時刻）を返す。
 */
struct ResponseArmOutcome {
    bool dataFailed    = false; //!< dwt_writetxdata() failed (Error::TxDataFailed).
    bool startFailed   = false; //!< dwt_starttx() failed (Error::TxStartFailed).
    bool wedged        = false; //!< abortIfDelayedTxWedged() caught the UM §9.4.1 errata (Error::TxStartFailed, txWedged=true).
    int32_t txMarginUs = 0;     //!< delayedTxMarginUs() captured right before dwt_starttx() (0 if dataFailed, matching the original functions never reaching that line either).
};

static inline ResponseArmOutcome buildAndArmResponse(const char (&payloadPrefix)[3], bool responseExpected,
                                                       uint32_t rxAfterTxDelayUus, uint32_t rxTimeoutUus,
                                                       uint8_t pollSequence, uint16_t panId, uint16_t selfAddr,
                                                       uint16_t peerAddr, uint32_t responseTxDelayUus,
                                                       uint16_t txAntennaDelay, uint8_t timingProfileRaw,
                                                       uint64_t& pollRxTsOut)
{
    uint8_t ts[5] = {0};
    dwt_readrxtimestamp(ts, static_cast<dwt_ip_sts_segment_e>(0));
    const uint64_t pollRxTs = get40le(ts);
    pollRxTsOut              = pollRxTs;

    const uint32_t respTxTime =
        static_cast<uint32_t>((pollRxTs + (responseTxDelayUus * kUusToDwtTime)) >> 8);
    const uint64_t respTxTs = ((static_cast<uint64_t>(respTxTime & 0xFFFFFFFEUL)) << 8) + txAntennaDelay;

    uint8_t respPayload[13] = {static_cast<uint8_t>(payloadPrefix[0]), static_cast<uint8_t>(payloadPrefix[1]),
                                static_cast<uint8_t>(payloadPrefix[2])};
    set32le(&respPayload[3], static_cast<uint32_t>(pollRxTs));
    set32le(&respPayload[7], static_cast<uint32_t>(respTxTs));
    respPayload[11] = kTimingPresetVersion;
    respPayload[12] = timingProfileRaw;

    uint8_t respFrame[22] = {0};
    buildShortAddressFrame(respFrame, pollSequence, panId, selfAddr, peerAddr, respPayload, sizeof(respPayload));

    dwt_setdelayedtrxtime(respTxTime);
    dwt_setrxaftertxdelay(rxAfterTxDelayUus);
    dwt_setrxtimeout(rxTimeoutUus);
    dwt_setpreambledetecttimeout(0); // R9: PRETOC left disabled - see uwb_qm33120_twr.cpp's file header R9 comment.

    ResponseArmOutcome outcome;
    if (dwt_writetxdata(sizeof(respFrame), respFrame, 0) != DWT_SUCCESS) {
        outcome.dataFailed = true;
        return outcome;
    }
    dwt_writetxfctrl(sizeof(respFrame) + FCS_LEN, 0, 1);
    outcome.txMarginUs = delayedTxMarginUs(respTxTime);

    const uint32_t startFlags =
        static_cast<uint32_t>(DWT_START_TX_DELAYED) | (responseExpected ? static_cast<uint32_t>(DWT_RESPONSE_EXPECTED) : 0U);
    if (dwt_starttx(startFlags) != DWT_SUCCESS) {
        outcome.startFailed = true;
        return outcome;
    }
    if (abortIfDelayedTxWedged()) {
        outcome.wedged = true;
        return outcome;
    }
    return outcome;
}

/**
 * @brief TXFRS（送信完了）を最大 boundMs 待つ（respondRange() cpp:672-694 /
 * respondDSRange() cpp:1372-1380 の "while ((nowMs()-start)<20) {...
 * vTaskDelay(1);}" と同一構造）。true を返すときだけ TXFRS ビットを
 * クリア済み（呼び出し側が読みたい診断レジスタ - 受信電力等 - は、
 * 元のコードと同じくこの後で読むこと）。
 */
static inline bool waitTxFrsBounded(uint32_t boundMs)
{
    const uint32_t startMs = nowMs();
    while ((nowMs() - startMs) < boundMs) {
        if ((dwt_readsysstatuslo() & DWT_INT_TXFRS_BIT_MASK) != 0) {
            dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return false;
}

/**
 * @brief DS-TWR 非対称式による距離計算（respondDSRange() cpp:1320-1349
 * 相当。数式・バイト位置は一切変更しない）。
 * @param finalFrame  Final（"DWF"、24バイト、payload先頭3バイトを除いた
 *                     [12],[16],[20] に pollTxTs/respRxTs/finalTxTs が
 *                     入っている）の生バイト列。
 * @param pollRxTs64  buildAndArmResponse() が返した Poll の RX タイムスタンプ
 *                     （Response 送信時に読んだのと同じ値を再利用する。
 *                     respondDSRange() 本体が `pollRxTs` を関数スコープの
 *                     ローカル変数として使い回すのと同じ）。
 */
struct DsDistanceResult {
    bool valid          = false; //!< false なら denominator<=0 (Error::RangeTimestampInvalid相当)。
    int32_t distanceMm  = 0;
    float distanceM      = 0.0f;
};

static inline DsDistanceResult computeDsDistance(const uint8_t* finalFrame, uint64_t pollRxTs64)
{
    const uint64_t finalRxTs   = readRxTimestamp64();
    const uint32_t pollTxTs32  = get32le(&finalFrame[12]);
    const uint32_t respRxTs32  = get32le(&finalFrame[16]);
    const uint32_t finalTxTs32 = get32le(&finalFrame[20]);
    const uint32_t pollRxTs32  = static_cast<uint32_t>(pollRxTs64);
    const uint32_t respTxTs32  = static_cast<uint32_t>(readTxTimestamp64());
    const uint32_t finalRxTs32 = static_cast<uint32_t>(finalRxTs);

    const double ra          = static_cast<double>(static_cast<uint32_t>(respRxTs32 - pollTxTs32));
    const double rb          = static_cast<double>(static_cast<uint32_t>(finalRxTs32 - respTxTs32));
    const double da          = static_cast<double>(static_cast<uint32_t>(finalTxTs32 - respRxTs32));
    const double db          = static_cast<double>(static_cast<uint32_t>(respTxTs32 - pollRxTs32));
    const double denominator = ra + rb + da + db;

    DsDistanceResult out;
    if (denominator <= 0.0) {
        return out;
    }
    const double tofDtu = ((ra * rb) - (da * db)) / denominator;
    const double tof     = tofDtu * DWT_TIME_UNITS;
    out.distanceM         = static_cast<float>(tof * kSpeedOfLightMPerS);
    out.distanceMm = static_cast<int32_t>((out.distanceM * 1000.0f) + (out.distanceM >= 0 ? 0.5f : -0.5f));
    out.valid       = true;
    return out;
}

/**
 * @brief 結果（"DWD"）フレームの組み立てと送信（respondDSRange()
 * cpp:1351-1387 相当）。`repeatCount==0` は 1 として扱う（既存のまま）。
 */
struct DsResultSendOutcome {
    uint8_t sentCount = 0;
};

static inline DsResultSendOutcome sendDsResult(uint8_t sequence, uint16_t panId, uint16_t selfAddr,
                                                 uint16_t peerAddr, int32_t distanceMm, uint8_t repeatCount,
                                                 uint32_t repeatGapMs)
{
    uint8_t distPayload[7] = {'D', 'W', 'D'};
    set32le(&distPayload[3], static_cast<uint32_t>(distanceMm));
    uint8_t distFrame[16] = {0};
    buildShortAddressFrame(distFrame, sequence, panId, selfAddr, peerAddr, distPayload, sizeof(distPayload));

    DsResultSendOutcome outcome;
    const uint8_t actualRepeat = (repeatCount == 0) ? 1 : repeatCount;
    for (uint8_t i = 0; i < actualRepeat; ++i) {
        dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
        if (dwt_writetxdata(sizeof(distFrame), distFrame, 0) != DWT_SUCCESS) {
            continue;
        }
        dwt_writetxfctrl(sizeof(distFrame) + FCS_LEN, 0, 0);
        if (dwt_starttx(DWT_START_TX_IMMEDIATE) != DWT_SUCCESS) {
            stopRadioAndClearTxStatus();
            continue;
        }

        if (waitTxFrsBounded(20)) {
            outcome.sentCount++;
        } else {
            stopRadioAndClearTxStatus();
        }
        if ((i + 1) < actualRepeat) {
            vTaskDelay(pdMS_TO_TICKS(repeatGapMs));
        }
    }
    return outcome;
}

} // namespace uwb::detail
