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

#include <cstdint>
#include <cstring>

#include "esp_timer.h"

#include "deca_device_api.h"

#include "uwb_qm33120_frame_match.hpp"
#include "uwb_qm33120_types.hpp"

namespace uwb::detail {

/**
 * @brief SDK側の FCS_LEN が、frame_match.hpp が複製した detail::kFcsLen と
 * 今も一致していることのビルド時検算（docs/REIMPL_PLAN.md R2、G-2）。
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

/* Step 2 (TWR) 用に前倒しで用意した定数。cpp:787/1012/1165 で3回重複定義されて
 * いた speedOfLight と、cpp:944/1086/1225/1236他で使われる uusToDwtTime を
 * ここに集約する（Step 1 では未使用）。値は原本のまま。 */
static constexpr double kSpeedOfLightMPerS = 299702547.0; //!< 実効値（真空中の光速ではない）。cpp:787,1012,1165。
static constexpr uint64_t kUusToDwtTime    = 65536ULL;

} // namespace uwb::detail
