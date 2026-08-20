/**
 * @file uwb_qm33120_twr.cpp
 * @brief Phase 2 Step 2: SS-TWR / DS-TWR ranging methods for uwb::Qm33120.
 *
 * ESP-IDF port of third_party/M5Stamp-UWB/src/M5Stamp_UWB.cpp (M5Stack
 * Technology CO LTD, MIT), specifically:
 *   - M5Stamp_UWB::requestRange()    SS-TWR initiator  cpp:785-889
 *   - M5Stamp_UWB::respondRange()    SS-TWR responder  cpp:891-1009
 *   - M5Stamp_UWB::requestDSRange()  DS-TWR initiator  cpp:1010-1161
 *   - M5Stamp_UWB::respondDSRange()  DS-TWR responder（距離計算はここ） cpp:1163-1377
 *
 * 移植方針は Step 1 (uwb_qm33120.cpp) と同じ「1行ずつ突き合わせ」。
 * 各関数の中はロジック・計算式・バイト位置・エンディアン・32bit/40bit の別・
 * 符号付き/符号なしの別を一切変えていない。意図的な変更点は以下のみ:
 *
 *  - millis() -> detail::nowMs(), delay(N) -> vTaskDelay(pdMS_TO_TICKS(N))
 *    （Step 1 と同じ wraparound-safe idiom。sdkconfig.defaults で
 *    CONFIG_FREERTOS_HZ=1000 にしてあるので delay(1) 相当の粒度も同じ）
 *  - `static constexpr double speedOfLight = 299702547.0;` /
 *    `static constexpr uint64_t uusToDwtTime = 65536ULL;` が原本では
 *    requestRange/requestDSRange/respondDSRange の3関数それぞれの先頭で
 *    重複定義されていた（cpp:787-788, 1012-1013, 1165-1166）。
 *    speedOfLight は requestDSRange 内では未使用で `(void)speedOfLight;`
 *    で警告を抑制していた。ここでは Step 1 で前倒し用意済みの
 *    uwb_qm33120_internal.hpp の detail::kSpeedOfLightMPerS /
 *    detail::kUusToDwtTime に一本化し、使わない関数では単に参照しない
 *    （値は変更なし。速度は真空中の光速ではなく実効値なので
 *    299792458 に「修正」したりしないこと）
 *  - get16le/get32le/get40le/set16le/set32le, buildShortAddressFrame/
 *    parseShortAddressFrame/payloadMatches, rxStatusToError,
 *    stopRadioAndClear{Status,RxStatus,TxStatus,IoStatus},
 *    readTxTimestamp64()/readRxTimestamp64() は
 *    uwb_qm33120_internal.hpp の detail:: 版（Step 1 で前倒し移植済み）を使う
 *  - Qm33120::Impl 型の完全な定義を uwb_qm33120_impl.hpp から include する
 *    （uwb_qm33120.cpp 側の変更点についてはそちらのファイル先頭コメント参照）
 *
 * アンテナ遅延の手動加算（cpp:946, 1087, 1226 = respTxTs/finalTxTs/respTxTsPlan
 * の算出）は削らずそのまま残している。チップ側の自動アンテナ遅延補正
 * （dwt_setrxantennadelay()/dwt_settxantennadelay()、uwb_qm33120.cpp の
 * init() 参照）とは役割が別（「遅延送信の起動時刻」と「アンテナから実際に
 * 電波が出る時刻」の差を埋めるもの）であり、二重計上ではない。
 */
#include "uwb_qm33120.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

extern "C" {
#include "deca_device_api.h"
#include "deca_interface.h"
}

#include "uwb_qm33120_impl.hpp"
#include "uwb_qm33120_internal.hpp"

namespace uwb {

RangeResult Qm33120::requestRange(const RangeConfig& range)
{
    // cpp:785-889. SS-TWR initiator (Tag). ロジック変更なし。
    RangeResult result;
    if (!_impl->initialized) {
        result.error = Error::ConfigFailed;
        setError(result.error);
        return result;
    }

    uint8_t pollFrame[12]       = {0};
    const uint8_t pollPayload[] = {'T', 'W', 'P'};
    const uint8_t pollSeq       = ++_impl->tx_sequence;
    detail::buildShortAddressFrame(pollFrame, pollSeq, range.panId, range.initiatorAddress, range.responderAddress,
                                    pollPayload, sizeof(pollPayload));

    detail::stopRadioAndClearIoStatus();
    dwt_setpreambledetecttimeout(0);
    dwt_setrxaftertxdelay(range.responseRxAfterTxDelayUus);
    dwt_setrxtimeout(range.rxTimeoutUus);

    if (dwt_writetxdata(sizeof(pollFrame), pollFrame, 0) != DWT_SUCCESS) {
        detail::stopRadioAndClearIoStatus();
        result.error = Error::TxDataFailed;
        setError(result.error);
        return result;
    }
    dwt_writetxfctrl(sizeof(pollFrame) + FCS_LEN, 0, 1);
    if (dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED) != DWT_SUCCESS) {
        detail::stopRadioAndClearIoStatus();
        result.error = Error::TxStartFailed;
        setError(result.error);
        return result;
    }

    const uint32_t startMs = detail::nowMs();
    while ((detail::nowMs() - startMs) < range.hostTimeoutMs) {
        const uint32_t status = dwt_readsysstatuslo();
        if ((status & DWT_INT_RXFCG_BIT_MASK) != 0) {
            uint8_t rawFrame[32] = {0};
            uint8_t rangingBit   = 0;
            uint16_t frameLen    = dwt_getframelength(&rangingBit);
            if (frameLen > sizeof(rawFrame)) {
                frameLen = sizeof(rawFrame);
            }
            dwt_readrxdata(rawFrame, frameLen, 0);
            dwt_writesysstatuslo(SYS_STATUS_ALL_RX_GOOD | DWT_INT_TXFRS_BIT_MASK);

            RxResult parsed;
            if (!detail::parseShortAddressFrame(rawFrame, frameLen, parsed) ||
                !detail::payloadMatches(rawFrame, frameLen, "TWR", 3, 11) || (parsed.sequence != pollSeq) ||
                (parsed.panId != range.panId) || (parsed.src != range.responderAddress) ||
                (parsed.dst != range.initiatorAddress)) {
                detail::stopRadioAndClearRxStatus();
                result.sequence  = parsed.sequence;
                result.elapsedMs = detail::nowMs() - startMs;
                result.error     = Error::RangeFrameMismatch;
                setError(result.error);
                return result;
            }

            const uint32_t pollTxTs = dwt_readtxtimestamplo32();
            const uint32_t respRxTs = dwt_readrxtimestamplo32(static_cast<dwt_ip_sts_segment_e>(0));
            const uint32_t pollRxTs = detail::get32le(&rawFrame[12]);
            const uint32_t respTxTs = detail::get32le(&rawFrame[16]);
            const int32_t rtdInit   = static_cast<int32_t>(respRxTs - pollTxTs);
            const int32_t rtdResp   = static_cast<int32_t>(respTxTs - pollRxTs);

            if ((rtdInit <= 0) || (rtdResp <= 0)) {
                result.elapsedMs = detail::nowMs() - startMs;
                result.error     = Error::RangeTimestampInvalid;
                setError(result.error);
                return result;
            }

            const double tof = ((static_cast<double>(rtdInit) - static_cast<double>(rtdResp)) / 2.0) * DWT_TIME_UNITS;
            result.distanceM = static_cast<float>(tof * detail::kSpeedOfLightMPerS);
            result.distanceMm =
                static_cast<int32_t>((result.distanceM * 1000.0f) + (result.distanceM >= 0 ? 0.5f : -0.5f));
            result.sequence  = pollSeq;
            result.elapsedMs = detail::nowMs() - startMs;
            result.success   = true;
            result.error     = Error::Ok;
            setError(Error::Ok);
            return result;
        }

        if ((status & (SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)) != 0) {
            detail::stopRadioAndClearRxStatus();
            result.elapsedMs = detail::nowMs() - startMs;
            result.error     = detail::rxStatusToError(status);
            setError(result.error);
            return result;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    detail::stopRadioAndClearRxStatus();
    result.elapsedMs = detail::nowMs() - startMs;
    result.error     = Error::RxTimeout;
    setError(result.error);
    return result;
}

ResponderResult Qm33120::respondRange(const RangeConfig& range)
{
    // cpp:891-1009. SS-TWR responder (Anchor)。ロジック変更なし。
    ResponderResult result;
    if (!_impl->initialized) {
        result.error = Error::ConfigFailed;
        setError(result.error);
        return result;
    }

    detail::stopRadioAndClearIoStatus();
    dwt_setpreambledetecttimeout(0);
    dwt_setrxaftertxdelay(0);
    dwt_setrxtimeout(0);

    if (dwt_rxenable(DWT_START_RX_IMMEDIATE) != DWT_SUCCESS) {
        detail::stopRadioAndClearRxStatus();
        result.error = Error::RxStartFailed;
        setError(result.error);
        return result;
    }

    const uint32_t startMs = detail::nowMs();
    while ((detail::nowMs() - startMs) < range.hostTimeoutMs) {
        const uint32_t status = dwt_readsysstatuslo();
        if ((status & DWT_INT_RXFCG_BIT_MASK) != 0) {
            uint8_t pollFrame[32] = {0};
            uint8_t rangingBit    = 0;
            uint16_t frameLen     = dwt_getframelength(&rangingBit);
            if (frameLen > sizeof(pollFrame)) {
                frameLen = sizeof(pollFrame);
            }
            dwt_readrxdata(pollFrame, frameLen, 0);
            dwt_writesysstatuslo(SYS_STATUS_ALL_RX_GOOD);

            RxResult parsed;
            if (!detail::parseShortAddressFrame(pollFrame, frameLen, parsed) ||
                !detail::payloadMatches(pollFrame, frameLen, "TWP", 3, 3) || (parsed.panId != range.panId) ||
                (parsed.dst != range.responderAddress)) {
                detail::stopRadioAndClearRxStatus();
                result.sequence  = parsed.sequence;
                result.requester = parsed.src;
                result.elapsedMs = detail::nowMs() - startMs;
                result.error     = Error::RangeFrameMismatch;
                setError(result.error);
                return result;
            }

            uint8_t ts[5] = {0};
            dwt_readrxtimestamp(ts, static_cast<dwt_ip_sts_segment_e>(0));
            const uint64_t pollRxTs = detail::get40le(ts);
            const uint32_t respTxTime =
                static_cast<uint32_t>((pollRxTs + (range.responseTxDelayUus * detail::kUusToDwtTime)) >> 8);
            const uint64_t respTxTs =
                ((static_cast<uint64_t>(respTxTime & 0xFFFFFFFEUL)) << 8) + _impl->tx_antenna_delay;

            uint8_t respPayload[11] = {'T', 'W', 'R'};
            detail::set32le(&respPayload[3], static_cast<uint32_t>(pollRxTs));
            detail::set32le(&respPayload[7], static_cast<uint32_t>(respTxTs));

            uint8_t respFrame[20] = {0};
            detail::buildShortAddressFrame(respFrame, parsed.sequence, range.panId, range.responderAddress,
                                            parsed.src, respPayload, sizeof(respPayload));

            dwt_setdelayedtrxtime(respTxTime);
            if (dwt_writetxdata(sizeof(respFrame), respFrame, 0) != DWT_SUCCESS) {
                detail::stopRadioAndClearTxStatus();
                result.error = Error::TxDataFailed;
                setError(result.error);
                return result;
            }
            dwt_writetxfctrl(sizeof(respFrame) + FCS_LEN, 0, 1);
            if (dwt_starttx(DWT_START_TX_DELAYED) != DWT_SUCCESS) {
                detail::stopRadioAndClearTxStatus();
                result.error = Error::TxStartFailed;
                setError(result.error);
                return result;
            }

            const uint32_t txStartMs = detail::nowMs();
            while ((detail::nowMs() - txStartMs) < 20) {
                const uint32_t txStatus = dwt_readsysstatuslo();
                if ((txStatus & DWT_INT_TXFRS_BIT_MASK) != 0) {
                    dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
                    result.success   = true;
                    result.sequence  = parsed.sequence;
                    result.requester = parsed.src;
                    result.elapsedMs = detail::nowMs() - startMs;
                    result.error     = Error::Ok;
                    setError(Error::Ok);
                    return result;
                }
                vTaskDelay(pdMS_TO_TICKS(1));
            }

            detail::stopRadioAndClearTxStatus();
            result.error = Error::TxTimeout;
            setError(result.error);
            return result;
        }

        if ((status & (SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)) != 0) {
            detail::stopRadioAndClearRxStatus();
            result.elapsedMs = detail::nowMs() - startMs;
            result.error     = detail::rxStatusToError(status);
            setError(result.error);
            return result;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    detail::stopRadioAndClearRxStatus();
    result.elapsedMs = detail::nowMs() - startMs;
    result.error     = Error::RxTimeout;
    setError(result.error);
    return result;
}

DSRangeResult Qm33120::requestDSRange(const DSRangeConfig& range)
{
    // cpp:1010-1161. DS-TWR initiator (Tag)。ロジック変更なし。
    // 距離はここでは計算しない: Anchor(respondDSRange)が計算した値を
    // "DWD" フレームでそのまま受信して result.distanceMm/distanceM に採用する
    // (cpp:1152-1153)。
    DSRangeResult result;
    if (!_impl->initialized) {
        result.error = Error::ConfigFailed;
        setError(result.error);
        return result;
    }

    uint8_t pollFrame[12]       = {0};
    const uint8_t pollPayload[] = {'D', 'W', 'P'};
    const uint8_t pollSeq       = ++_impl->tx_sequence;
    detail::buildShortAddressFrame(pollFrame, pollSeq, range.panId, range.initiatorAddress, range.responderAddress,
                                    pollPayload, sizeof(pollPayload));

    detail::stopRadioAndClearIoStatus();
    dwt_setpreambledetecttimeout(0);
    dwt_setrxaftertxdelay(range.responseRxAfterTxDelayUus);
    dwt_setrxtimeout(range.rxTimeoutUus);

    if (dwt_writetxdata(sizeof(pollFrame), pollFrame, 0) != DWT_SUCCESS) {
        detail::stopRadioAndClearIoStatus();
        result.error = Error::TxDataFailed;
        setError(result.error);
        return result;
    }
    dwt_writetxfctrl(sizeof(pollFrame) + FCS_LEN, 0, 1);
    if (dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED) != DWT_SUCCESS) {
        detail::stopRadioAndClearIoStatus();
        result.error = Error::TxStartFailed;
        setError(result.error);
        return result;
    }

    uint8_t respFrame[32]  = {0};
    uint16_t respLen       = 0;
    const uint32_t startMs = detail::nowMs();
    while ((detail::nowMs() - startMs) < range.hostTimeoutMs) {
        const uint32_t status = dwt_readsysstatuslo();
        if ((status & DWT_INT_RXFCG_BIT_MASK) != 0) {
            uint8_t rangingBit = 0;
            respLen            = dwt_getframelength(&rangingBit);
            if (respLen > sizeof(respFrame)) {
                respLen = sizeof(respFrame);
            }
            dwt_readrxdata(respFrame, respLen, 0);
            dwt_writesysstatuslo(SYS_STATUS_ALL_RX_GOOD | DWT_INT_TXFRS_BIT_MASK);
            break;
        }
        if ((status & (SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)) != 0) {
            detail::stopRadioAndClearRxStatus();
            result.elapsedMs = detail::nowMs() - startMs;
            result.error     = detail::rxStatusToError(status);
            setError(result.error);
            return result;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    RxResult parsed;
    if (!detail::parseShortAddressFrame(respFrame, respLen, parsed) ||
        !detail::payloadMatches(respFrame, respLen, "DWR", 3, 11) || (parsed.sequence != pollSeq) ||
        (parsed.panId != range.panId) || (parsed.src != range.responderAddress) ||
        (parsed.dst != range.initiatorAddress)) {
        detail::stopRadioAndClearRxStatus();
        result.sequence  = parsed.sequence;
        result.elapsedMs = detail::nowMs() - startMs;
        result.error     = respLen == 0 ? Error::RxTimeout : Error::RangeFrameMismatch;
        setError(result.error);
        return result;
    }

    const uint64_t pollTxTs    = detail::readTxTimestamp64();
    const uint64_t respRxTs    = detail::readRxTimestamp64();
    const uint32_t finalTxTime =
        static_cast<uint32_t>((respRxTs + (range.finalTxDelayUus * detail::kUusToDwtTime)) >> 8);
    const uint64_t finalTxTs = ((static_cast<uint64_t>(finalTxTime & 0xFFFFFFFEUL)) << 8) + _impl->tx_antenna_delay;

    uint8_t finalPayload[15] = {'D', 'W', 'F'};
    detail::set32le(&finalPayload[3], static_cast<uint32_t>(pollTxTs));
    detail::set32le(&finalPayload[7], static_cast<uint32_t>(respRxTs));
    detail::set32le(&finalPayload[11], static_cast<uint32_t>(finalTxTs));

    uint8_t finalFrame[24] = {0};
    detail::buildShortAddressFrame(finalFrame, pollSeq, range.panId, range.initiatorAddress, range.responderAddress,
                                    finalPayload, sizeof(finalPayload));

    dwt_setdelayedtrxtime(finalTxTime);
    dwt_setrxaftertxdelay(range.resultRxAfterFinalTxDelayUus);
    dwt_setrxtimeout(range.rxTimeoutUus);
    if (dwt_writetxdata(sizeof(finalFrame), finalFrame, 0) != DWT_SUCCESS) {
        detail::stopRadioAndClearIoStatus();
        result.error = Error::TxDataFailed;
        setError(result.error);
        return result;
    }
    dwt_writetxfctrl(sizeof(finalFrame) + FCS_LEN, 0, 1);
    if (dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED) != DWT_SUCCESS) {
        detail::stopRadioAndClearIoStatus();
        result.error = Error::TxStartFailed;
        setError(result.error);
        return result;
    }

    uint8_t distFrame[32]        = {0};
    uint16_t distLen             = 0;
    const uint32_t resultStartMs = detail::nowMs();
    while ((detail::nowMs() - resultStartMs) < range.hostTimeoutMs) {
        const uint32_t status = dwt_readsysstatuslo();
        if ((status & DWT_INT_RXFCG_BIT_MASK) != 0) {
            uint8_t rangingBit = 0;
            distLen            = dwt_getframelength(&rangingBit);
            if (distLen > sizeof(distFrame)) {
                distLen = sizeof(distFrame);
            }
            dwt_readrxdata(distFrame, distLen, 0);
            dwt_writesysstatuslo(SYS_STATUS_ALL_RX_GOOD | DWT_INT_TXFRS_BIT_MASK);
            break;
        }
        if ((status & (SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)) != 0) {
            detail::stopRadioAndClearRxStatus();
            result.sequence  = pollSeq;
            result.elapsedMs = detail::nowMs() - startMs;
            result.error     = detail::rxStatusToError(status);
            setError(result.error);
            return result;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    if (!detail::parseShortAddressFrame(distFrame, distLen, parsed) ||
        !detail::payloadMatches(distFrame, distLen, "DWD", 3, 7) || (parsed.sequence != pollSeq) ||
        (parsed.panId != range.panId) || (parsed.src != range.responderAddress) ||
        (parsed.dst != range.initiatorAddress)) {
        detail::stopRadioAndClearRxStatus();
        result.sequence  = parsed.sequence;
        result.elapsedMs = detail::nowMs() - startMs;
        result.error     = distLen == 0 ? Error::RxTimeout : Error::RangeFrameMismatch;
        setError(result.error);
        return result;
    }

    result.distanceMm = static_cast<int32_t>(detail::get32le(&distFrame[12]));
    result.distanceM  = static_cast<float>(result.distanceMm) / 1000.0f;
    result.sequence   = pollSeq;
    result.elapsedMs  = detail::nowMs() - startMs;
    result.success    = true;
    result.error      = Error::Ok;
    setError(Error::Ok);
    return result;
}

DSResponderResult Qm33120::respondDSRange(const DSRangeConfig& range)
{
    // cpp:1163-1377. DS-TWR responder (Anchor)。距離計算はこの関数の中で行う
    // （非対称DS-TWR標準式、cpp:1294-1319）。ロジック変更なし。
    DSResponderResult result;
    if (!_impl->initialized) {
        result.error = Error::ConfigFailed;
        setError(result.error);
        return result;
    }

    detail::stopRadioAndClearIoStatus();
    dwt_setpreambledetecttimeout(0);
    dwt_setrxaftertxdelay(0);
    dwt_setrxtimeout(0);

    if (dwt_rxenable(DWT_START_RX_IMMEDIATE) != DWT_SUCCESS) {
        detail::stopRadioAndClearRxStatus();
        result.error = Error::RxStartFailed;
        setError(result.error);
        return result;
    }

    uint8_t pollFrame[32] = {0};
    uint16_t pollLen      = 0;
    RxResult parsed;
    const uint32_t startMs = detail::nowMs();
    while ((detail::nowMs() - startMs) < range.hostTimeoutMs) {
        const uint32_t status = dwt_readsysstatuslo();
        if ((status & DWT_INT_RXFCG_BIT_MASK) != 0) {
            uint8_t rangingBit = 0;
            pollLen            = dwt_getframelength(&rangingBit);
            if (pollLen > sizeof(pollFrame)) {
                pollLen = sizeof(pollFrame);
            }
            dwt_readrxdata(pollFrame, pollLen, 0);
            dwt_writesysstatuslo(SYS_STATUS_ALL_RX_GOOD);
            break;
        }
        if ((status & (SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)) != 0) {
            detail::stopRadioAndClearRxStatus();
            result.elapsedMs = detail::nowMs() - startMs;
            result.error     = detail::rxStatusToError(status);
            setError(result.error);
            return result;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    if (!detail::parseShortAddressFrame(pollFrame, pollLen, parsed) ||
        !detail::payloadMatches(pollFrame, pollLen, "DWP", 3, 3) || (parsed.panId != range.panId) ||
        (parsed.dst != range.responderAddress)) {
        detail::stopRadioAndClearRxStatus();
        result.sequence  = parsed.sequence;
        result.requester = parsed.src;
        result.elapsedMs = detail::nowMs() - startMs;
        result.error     = pollLen == 0 ? Error::RxTimeout : Error::RangeFrameMismatch;
        setError(result.error);
        return result;
    }

    const uint64_t pollRxTs = detail::readRxTimestamp64();
    const uint32_t respTxTime =
        static_cast<uint32_t>((pollRxTs + (range.responseTxDelayUus * detail::kUusToDwtTime)) >> 8);
    const uint64_t respTxTsPlan =
        ((static_cast<uint64_t>(respTxTime & 0xFFFFFFFEUL)) << 8) + _impl->tx_antenna_delay;

    uint8_t respPayload[11] = {'D', 'W', 'R'};
    detail::set32le(&respPayload[3], static_cast<uint32_t>(pollRxTs));
    detail::set32le(&respPayload[7], static_cast<uint32_t>(respTxTsPlan));

    uint8_t respFrame[20] = {0};
    detail::buildShortAddressFrame(respFrame, parsed.sequence, range.panId, range.responderAddress, parsed.src,
                                    respPayload, sizeof(respPayload));

    dwt_setdelayedtrxtime(respTxTime);
    dwt_setrxaftertxdelay(range.finalRxAfterResponseTxDelayUus);
    dwt_setrxtimeout(range.rxTimeoutUus);
    if (dwt_writetxdata(sizeof(respFrame), respFrame, 0) != DWT_SUCCESS) {
        detail::stopRadioAndClearIoStatus();
        result.error = Error::TxDataFailed;
        setError(result.error);
        return result;
    }
    dwt_writetxfctrl(sizeof(respFrame) + FCS_LEN, 0, 1);
    if (dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED) != DWT_SUCCESS) {
        detail::stopRadioAndClearIoStatus();
        result.error = Error::TxStartFailed;
        setError(result.error);
        return result;
    }

    uint8_t finalFrame[32]      = {0};
    uint16_t finalLen           = 0;
    const uint32_t finalStartMs = detail::nowMs();
    while ((detail::nowMs() - finalStartMs) < range.hostTimeoutMs) {
        const uint32_t status = dwt_readsysstatuslo();
        if ((status & DWT_INT_RXFCG_BIT_MASK) != 0) {
            uint8_t rangingBit = 0;
            finalLen           = dwt_getframelength(&rangingBit);
            if (finalLen > sizeof(finalFrame)) {
                finalLen = sizeof(finalFrame);
            }
            dwt_readrxdata(finalFrame, finalLen, 0);
            dwt_writesysstatuslo(SYS_STATUS_ALL_RX_GOOD | DWT_INT_TXFRS_BIT_MASK);
            break;
        }
        if ((status & (SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)) != 0) {
            detail::stopRadioAndClearRxStatus();
            result.sequence  = parsed.sequence;
            result.requester = parsed.src;
            result.elapsedMs = detail::nowMs() - startMs;
            result.error     = detail::rxStatusToError(status);
            setError(result.error);
            return result;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    RxResult finalParsed;
    if (!detail::parseShortAddressFrame(finalFrame, finalLen, finalParsed) ||
        !detail::payloadMatches(finalFrame, finalLen, "DWF", 3, 15) || (finalParsed.sequence != parsed.sequence) ||
        (finalParsed.panId != range.panId) || (finalParsed.src != parsed.src) ||
        (finalParsed.dst != range.responderAddress)) {
        detail::stopRadioAndClearRxStatus();
        result.sequence  = finalParsed.sequence;
        result.requester = parsed.src;
        result.elapsedMs = detail::nowMs() - startMs;
        result.error     = finalLen == 0 ? Error::RxTimeout : Error::RangeFrameMismatch;
        setError(result.error);
        return result;
    }

    const uint64_t finalRxTs   = detail::readRxTimestamp64();
    const uint32_t pollTxTs32  = detail::get32le(&finalFrame[12]);
    const uint32_t respRxTs32  = detail::get32le(&finalFrame[16]);
    const uint32_t finalTxTs32 = detail::get32le(&finalFrame[20]);
    const uint32_t pollRxTs32  = static_cast<uint32_t>(pollRxTs);
    const uint32_t respTxTs32  = static_cast<uint32_t>(detail::readTxTimestamp64());
    const uint32_t finalRxTs32 = static_cast<uint32_t>(finalRxTs);

    const double ra          = static_cast<double>(static_cast<uint32_t>(respRxTs32 - pollTxTs32));
    const double rb          = static_cast<double>(static_cast<uint32_t>(finalRxTs32 - respTxTs32));
    const double da          = static_cast<double>(static_cast<uint32_t>(finalTxTs32 - respRxTs32));
    const double db          = static_cast<double>(static_cast<uint32_t>(respTxTs32 - pollRxTs32));
    const double denominator = ra + rb + da + db;
    if (denominator <= 0.0) {
        result.sequence  = parsed.sequence;
        result.requester = parsed.src;
        result.elapsedMs = detail::nowMs() - startMs;
        result.error     = Error::RangeTimestampInvalid;
        setError(result.error);
        return result;
    }

    const double tofDtu = ((ra * rb) - (da * db)) / denominator;
    const double tof    = tofDtu * DWT_TIME_UNITS;
    result.distanceM    = static_cast<float>(tof * detail::kSpeedOfLightMPerS);
    result.distanceMm   = static_cast<int32_t>((result.distanceM * 1000.0f) + (result.distanceM >= 0 ? 0.5f : -0.5f));

    uint8_t distPayload[7] = {'D', 'W', 'D'};
    detail::set32le(&distPayload[3], static_cast<uint32_t>(result.distanceMm));
    uint8_t distFrame[16] = {0};
    detail::buildShortAddressFrame(distFrame, parsed.sequence, range.panId, range.responderAddress, parsed.src,
                                    distPayload, sizeof(distPayload));

    uint8_t sentCount         = 0;
    const uint8_t repeatCount = range.resultRepeatCount == 0 ? 1 : range.resultRepeatCount;
    for (uint8_t i = 0; i < repeatCount; ++i) {
        dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
        if (dwt_writetxdata(sizeof(distFrame), distFrame, 0) != DWT_SUCCESS) {
            continue;
        }
        dwt_writetxfctrl(sizeof(distFrame) + FCS_LEN, 0, 0);
        if (dwt_starttx(DWT_START_TX_IMMEDIATE) != DWT_SUCCESS) {
            detail::stopRadioAndClearTxStatus();
            continue;
        }

        bool txDone              = false;
        const uint32_t txStartMs = detail::nowMs();
        while ((detail::nowMs() - txStartMs) < 20) {
            if ((dwt_readsysstatuslo() & DWT_INT_TXFRS_BIT_MASK) != 0) {
                dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
                txDone = true;
                sentCount++;
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        if (!txDone) {
            detail::stopRadioAndClearTxStatus();
        }
        if (i + 1 < repeatCount) {
            vTaskDelay(pdMS_TO_TICKS(range.resultRepeatGapMs));
        }
    }

    result.success   = sentCount > 0;
    result.sequence  = parsed.sequence;
    result.requester = parsed.src;
    result.elapsedMs = detail::nowMs() - startMs;
    result.error     = result.success ? Error::Ok : Error::TxTimeout;
    setError(result.error);
    return result;
}

} // namespace uwb
