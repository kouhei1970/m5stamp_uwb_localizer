/**
 * @file uwb_qm33120_responder.cpp
 * @brief docs/ARCHITECTURE_V2.md §2.2/§2.3: `uwb::Responder`'s hardware
 * side. Drives the pure `uwb::decide()` FSM (uwb_qm33120_responder_fsm.hpp)
 * against the real DW3720 registers. Frame-build / delayed-TX-arm / errata-
 * check / distance-computation / Result-build logic is NOT duplicated here
 * (docs/ARCHITECTURE_V2.md §1, 2026-08-30 revision: "旧respondRange()/
 * respondDSRange()と新Responderは...detail::の補助関数で共有し、コピーしない"):
 * this file calls the same `uwb::detail::buildAndArmResponse()` /
 * `computeDsDistance()` / `sendDsResult()` / `waitTxFrsBounded()` helpers
 * (uwb_qm33120_internal.hpp) that `Qm33120::respondRange()`/
 * `respondDSRange()` (uwb_qm33120_twr.cpp) were refactored to call too - see
 * that header for the one implementation of each.
 *
 * Old respondRange()/respondDSRange() are otherwise untouched
 * (docs/ARCHITECTURE_V2.md §1's "実験ファーム firmware/twr は当面旧構造の
 * まま残し、A/B の基準として使う").
 *
 * --- RXAUTR (SYS_CFG.RXAUTR) - NOT used, decision recorded here ---
 * docs/ARCHITECTURE_V2.md §2.2 asks whether DW3000/DW3720's SYS_CFG.RXAUTR
 * ("re-enable the receiver automatically after an RX error") can be enabled
 * standalone (without double-buffer mode) and used here instead of the
 * software `dwt_rxenable(DWT_START_RX_IMMEDIATE)` re-arms below. Findings
 * (both checked against the vendored SDK, not just this repo's trimmed
 * copy but also the pristine assets/DW3_QM33_SDK_1.1.1 drop - no DW3000/
 * DW3720 User Manual is present in this working copy: docs/refs/ is
 * gitignored and was not fetched this session, and assets/"QM33120W Data
 * Sheet.pdf" has no register-level SYS_CFG/SYS_STATE detail, confirmed by
 * `pdftotext` + grep):
 *  1. RXAUTR (SYS_CFG bit 10, SYS_CFG_RXAUTR_BIT_MASK) is only ever touched
 *     by dwt_setdblrxbuffmode()'s internal ull_setdblrxbuffmode()
 *     (dw3720_device.c): `if (dbl_buff_mode == DBL_BUF_MODE_AUTO) { or_val
 *     |= SYS_CFG_RXAUTR_BIT_MASK; }`, independent of dbl_buff_state's own
 *     bit - so bit-for-bit it CAN be set without double-buffer mode
 *     (dwt_setdblrxbuffmode(DBL_BUF_STATE_DIS, DBL_BUF_MODE_AUTO) would
 *     leave DIS_DRXB=1/single-buffer while still setting RXAUTR=1).
 *  2. But deca_device_api.h's own doc comment only says DBL_BUF_MODE_AUTO
 *     means "the receiver is re-enabled automatically" (contrasted with
 *     DBL_BUF_MODE_MAN: "re-enabled by the host after processing the RX
 *     event") - it does not say whether this "automatic" re-enable is
 *     scoped to RX ERRORS only (what §2.2 asks about) or to EVERY
 *     RX-terminating event (good frame, timeout, AND error alike), which
 *     would need very different handling here (e.g. it would also auto
 *     re-enable after our own Poll/Final captures, racing this code's own
 *     frame processing).
 *  3. Every DBL_BUF_MODE_AUTO usage in the vendored SDK (both copies) pairs
 *     it with DBL_BUF_STATE_EN (grep for dwt_setdblrxbuffmode( across
 *     assets/DW3_QM33_SDK_1.1.1: deca_compat.c, dw3720_device.c,
 *     dw3000_device.c, ex_02e_rx_dbl_buff/double_buffer_rx.c,
 *     tests/automated_tests/unit_test.c) - none exercise AUTO mode with
 *     double buffering disabled, so the decoupled (2) configuration this
 *     Responder would need is untested by the vendor.
 * Given (2) and (3) - the exact trigger scope is unconfirmed, and the one
 * configuration that WOULD need to be exercised is untested upstream - this
 * is left 未確認 (unconfirmed) rather than adopted. This Responder therefore
 * re-enables RX purely in software (dwt_rxenable(DWT_START_RX_IMMEDIATE)
 * after every handled event, plus the Tick-driven idle check), exactly like
 * the existing respondRange()/respondDSRange() loops already do, and never
 * calls dwt_setdblrxbuffmode() at all (leaving SYS_CFG.RXAUTR at its
 * power-on-reset default, off).
 */
#include "uwb_qm33120_responder.hpp"

#include <cstring>

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

extern "C" {
#include "deca_device_api.h"
#include "deca_interface.h"
}

#include "uwb_port.h"
#include "uwb_qm33120.hpp"
#include "uwb_qm33120_internal.hpp"

namespace uwb {

namespace {

/** Length-8 RangeEvent queue (docs/ARCHITECTURE_V2.md §2.1). */
constexpr UBaseType_t kEventQueueLen = 8;

/**
 * @brief Bound for waitForCiaDone() below (2026-08-30 audit finding (c)).
 * No datasheet figure for CIA processing latency is available in this
 * working copy (未確認); chosen generously (1ms) relative to the
 * microsecond-scale internal pipeline delay this is meant to bridge, while
 * staying negligible next to the ~3-8ms DS-TWR exchange timings this
 * Responder already works with.
 */
constexpr uint32_t kCiaDoneWaitBoundUs = 1000;

/**
 * @brief Best-effort "is the chip currently receiving" check for the
 * Listen-state Tick handler (§2.2's "SYS_STATE を読み、受信状態でなければ
 * RX 再有効化"). See uwb_qm33120_internal.hpp's detail::delayedTxWedged()
 * for the same read pattern (SYS_STATE_LO via dwt_readfromdevice(), since
 * the SDK's own dwt_read8bitoffsetreg(dw, SYS_STATE_LO_ID, 2U) helper is not
 * a public/exported entry point).
 *
 * 未確認 (unconfirmed): no DW3000/DW3720 register documentation available in
 * this working copy names an exact "receiving" PMSC_STATE code - only
 * DW_SYS_STATE_INIT(1)/IDLE_RC(2)/IDLE(3) are named in the vendored SDK
 * (dw3720_deca_vals.h), plus the empirically-found TX code 0xD
 * (detail::delayedTxWedged()'s 0x000D0000 pattern). What IS verified in the
 * SDK's own source is the `dw_state > DW_SYS_STATE_IDLE` "busy" test used
 * throughout dw3720_device.c (ull_setdwstate()/ull_setchannel()/
 * ull_run_hardware_pll_cal() etc.) - i.e. any code above IDLE(3) means the
 * TSE is doing SOMETHING (not idle), whether that is RX, TX, or a
 * calibration state. During Listen this Responder never intentionally
 * drives the chip into anything but RX (Response/Result TX are
 * synchronous, bounded, and always re-arm RX afterwards on every path,
 * success or failure), so "not <= IDLE" is used here as "presumably still
 * receiving" without claiming the exact RX state code is confirmed.
 */
bool chipLooksIdleNotReceiving()
{
    uint8_t buf[SYS_STATE_LO_LEN] = {0, 0, 0, 0};
    dwt_readfromdevice(SYS_STATE_LO_ID, 0, sizeof(buf), buf);
    const uint8_t pmscState = buf[2]; // Byte offset 2 = PMSC_STATE, matching dw3720_device.c's own `dwt_read8bitoffsetreg(dw, SYS_STATE_LO_ID, 2U)` reads.
    return pmscState <= DW_SYS_STATE_IDLE;
}

} // namespace

struct Responder::Impl {
    Qm33120* radio = nullptr;
    ResponderConfig cfg;
    bool began = false;

    State state = State::Listen;

    /* WaitFinal bookkeeping (§2.2's `WaitFinal{peer, seq, timestamps,
     * deadline}` - kept here, in the hardware layer, rather than in the
     * pure `State` enum; see uwb_qm33120_responder_fsm.hpp's header comment
     * for why). */
    uint16_t waitPeer        = 0;
    uint8_t waitSeq           = 0;
    uint64_t waitPollRxTs64  = 0; //!< Poll's own RX timestamp (40-bit, chip units) - needed again at Final-arrival time for the DS-TWR distance formula (detail::computeDsDistance()'s `pollRxTs64`).
    uint32_t waitDeadlineMs  = 0; //!< Host-clock (detail::nowMs()) deadline; see onRespond()'s comment for how it is computed.
    int64_t exchangeStartUs  = 0; //!< esp_timer_get_time() when the in-flight exchange's Poll was accepted - RangeEvent::elapsedUs.

    /* Scratch, filled by classifyFrame(), consumed by onRespond()/
     * onComputeAndRespond() in the same service() call. */
    uwb::RxResult pollParsed;
    uint8_t finalFrameBuf[32] = {0};
    uint16_t finalFrameLen    = 0;

    portMUX_TYPE statsMux = portMUX_INITIALIZER_UNLOCKED;
    ResponderStats stats;

    QueueHandle_t eventQueue = nullptr;
};

namespace {

/** @brief Mutate `im.stats` under its critical section (§2.1: "radio タスクだけが更新し...portMUXの臨界区間で構造体コピー"). */
template <typename Fn>
void withStatsLock(Responder::Impl& im, Fn&& fn)
{
    portENTER_CRITICAL(&im.statsMux);
    fn(im.stats);
    portEXIT_CRITICAL(&im.statsMux);
}

/**
 * @brief Classify the frame RXFCG just delivered. Reads the raw bytes once
 * (dwt_getframelength()+dwt_readrxdata(), same as every RX loop in
 * uwb_qm33120_twr.cpp) and fills a FrameSummary; also stashes the parsed
 * header (Poll) or raw payload (Final) into `im` for the executor to use,
 * since decide() itself never sees the raw bytes.
 */
FrameSummary classifyFrame(Responder::Impl& im)
{
    FrameSummary out;

    uint8_t raw[32]     = {0};
    uint8_t rangingBit  = 0;
    uint16_t frameLen   = dwt_getframelength(&rangingBit);
    if (frameLen > sizeof(raw)) {
        frameLen = sizeof(raw);
    }
    dwt_readrxdata(raw, frameLen, 0);

    uwb::RxResult parsed;
    const bool headerOk = detail::parseShortAddressFrame(raw, frameLen, parsed);
    out.src              = parsed.src;
    out.dst               = parsed.dst;
    out.panId              = parsed.panId;
    out.seq                 = parsed.sequence;
    if (!headerOk) {
        out.kind = FrameKind::Other;
        return out;
    }

    // Final: only meaningful while WaitFinal, and only reported as such once
    // fully validated against the peer/sequence this Responder is currently
    // tracking (uwb_qm33120_responder_fsm.hpp's header comment "Why State
    // stays a bare 2-value enum"). "DWF" payload length is 15 bytes, same as
    // requestDSRange()'s finalPayload (uwb_qm33120_twr.cpp cpp:910).
    if (im.state == State::WaitFinal) {
        const bool payloadOk = detail::payloadMatches(raw, frameLen, "DWF", 3, 15);
        if (payloadOk && (parsed.src == im.waitPeer) && (parsed.sequence == im.waitSeq)) {
            out.kind             = FrameKind::Final;
            out.matchesTrackedPeer = true;
            std::memcpy(im.finalFrameBuf, raw, frameLen);
            im.finalFrameLen = frameLen;
            return out;
        }
    }

    // Poll: "TWP" (SS, 3 or 5 bytes incl. the version/profile tag) or "DWP"
    // (DS), same accept-either-length rule as respondRange()/
    // respondDSRange() (docs/TIMING_PRESETS.md §3.3).
    const bool ssPoll = detail::payloadMatchesEither(raw, frameLen, "TWP", 3, /*lenLegacy=*/3, /*lenTagged=*/5);
    const bool dsPoll = !ssPoll && detail::payloadMatchesEither(raw, frameLen, "DWP", 3, /*lenLegacy=*/3, /*lenTagged=*/5);
    if (ssPoll || dsPoll) {
        out.kind                = FrameKind::Poll;
        out.method                = dsPoll ? TwrMethod::DS : TwrMethod::SS;
        out.matchesTrackedPeer     = (im.state == State::Listen) ? true : (parsed.src == im.waitPeer);
        im.pollParsed              = parsed;
        return out;
    }

    out.kind = FrameKind::Other;
    return out;
}

/** @brief Action::Ignore - see uwb_qm33120_responder_fsm.hpp's Action::Ignore doc comment for the full state x event breakdown this implements. */
void onIgnore(Responder::Impl& im, Event ev)
{
    if (ev == Event::RxFrame) {
        dwt_writesysstatuslo(SYS_STATUS_ALL_RX_GOOD | DWT_INT_TXFRS_BIT_MASK);
        withStatsLock(im, [](ResponderStats& s) { s.other++; });
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        uwb_port_irq_clear_pending();
    } else if (ev == Event::RxError) {
        dwt_writesysstatuslo(SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_GOOD);
        withStatsLock(im, [](ResponderStats& s) { s.rxErrors++; });
        if (im.state == State::WaitFinal) {
            // §2.2's own note: "dwt_setrxtimeout to the remaining time in
            // UUS, or keep 0 and rely on the host deadline" - this Responder
            // takes the second option (service()'s own nowMs()>=
            // waitDeadlineMs check is the sole backstop from here on).
            dwt_setrxtimeout(0);
        }
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        uwb_port_irq_clear_pending();
    }
    // Event::Tick (WaitFinal, still within the deadline): nothing pending to
    // clear or re-arm - RX is already running toward its own hardware
    // timeout (or, after an earlier RxError, toward the host deadline set
    // above).
    // Event::TxDone: never generated by the caller - see Event::TxDone's doc
    // comment in uwb_qm33120_responder_fsm.hpp.
}

/** @brief Action::CheckIdleAndRearm (Listen + Tick only). */
void onCheckIdleAndRearm(Responder::Impl& im)
{
    if (chipLooksIdleNotReceiving()) {
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        uwb_port_irq_clear_pending();
        withStatsLock(im, [](ResponderStats& s) { s.rearms++; });
    }
}

/** @brief Common failure path for a Response's delayed TX (TxDataFailed/TxStartFailed/wedged) - matches respondRange()/respondDSRange()'s own TxDataFailed/TxStartFailed/txWedged branches. */
void failResponseToListen(Responder::Impl& im)
{
    detail::stopRadioAndClearIoStatus(); // forcetrxoff() + clear RX+TX status (§2.2's "任意 | 遅延送信の失敗 -> forcetrxoff -> RX再有効化").
    dwt_setrxtimeout(0);
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
    uwb_port_irq_clear_pending();
    im.state = State::Listen;
    withStatsLock(im, [](ResponderStats& s) { s.txFailures++; });
}

/**
 * @brief Action::RespondSs / Action::RespondDs. Builds and arms the
 * Response via detail::buildAndArmResponse() - the SAME helper
 * respondRange()/respondDSRange() call (uwb_qm33120_internal.hpp,
 * uwb_qm33120_twr.cpp) - addressed with `im.cfg.panId`/`im.cfg.shortAddr`
 * instead of a RangeConfig's own panId/responderAddress fields (§2.3:
 * "アドレスは shortAddr で上書き").
 *
 * Called both for a fresh Poll out of Listen and for a WaitFinal-time
 * restart (§2.2's "WaitFinal | Poll（どの相手からでも）| ... Listen+Poll と
 * 同じ動作") - `wasWaitFinal` (the state decide() was called with) is the
 * only thing distinguishing the two for stats purposes (`polls` vs
 * `restarts`); the response itself is built identically either way.
 */
void onRespond(Responder::Impl& im, bool ds)
{
    const bool wasWaitFinal = (im.state == State::WaitFinal);

    // 【2026-08-30 修正】この Poll は既に受理が確定している（decide() が
    // RespondSs/RespondDs を返したのはこの時点）ので、Response の送信結果が
    // 分かるより前に polls（と、restart なら restarts も）を確定させる。
    // responses/txFailures は下で送信結果が判明してから別に増やす
    // （ResponderStats のコメント参照 - こうしないと restart の分だけ
    // responses が polls を追い越しうる、実機 anchor_stats で発見されたバグ）。
    withStatsLock(im, [wasWaitFinal](ResponderStats& s) {
        s.polls++;
        if (wasWaitFinal) {
            s.restarts++;
        }
    });

    // Clear the Poll's RXFCG (and, on a restart, the earlier Response's
    // TXFRS, which may still be pending - respondDSRange() clears both
    // together the same way at its own Final-wait entry).
    dwt_writesysstatuslo(SYS_STATUS_ALL_RX_GOOD | DWT_INT_TXFRS_BIT_MASK);

    const uwb::RxResult& parsed = im.pollParsed;
    const uint32_t responseTxDelayUus = ds ? im.cfg.ds.responseTxDelayUus : im.cfg.ss.responseTxDelayUus;
    const char respPrefix[3]          = {ds ? 'D' : 'T', 'W', 'R'}; // 【タスクC-2】"TWR"/"DWR" (docs/TIMING_PRESETS.md §3.2)。

    // 【2026-08-30 実機結果】DS の W4R（finalRxAfterResponseTxDelayUus）が
    // Response 送信直後に作る「聞こえない窓」対策
    // (ResponderConfig::listenImmediatelyAfterTx のコメント参照)。
    // rxAfterTxDelayUus と rxTimeoutUus の配分を変えるだけで、両者の和
    // （締切がハードウェアRXFTOとして立つ絶対時刻）は変えない - ホスト側の
    // 締切計算（下の totalUus）が rxAfterTxDelayUus と rxTimeoutUus の
    // 個別値ではなく和だけを使っているのはこのため。
    // DSRangeConfig/プリセット/legacy respondDSRange() 自体は一切変更しない
    // （detail::buildAndArmResponse() は共有ヘルパーだが、この2値は
    // 呼び出し側=Responder がここで決めて渡すだけなので、legacy側の
    // 呼び出し (uwb_qm33120_twr.cpp) は影響を受けない）。
    uint32_t rxAfterTxDelayUus = 0U;
    uint32_t rxTimeoutUus      = 0U;
    if (ds) {
        if (im.cfg.listenImmediatelyAfterTx) {
            rxAfterTxDelayUus = 0U;
            rxTimeoutUus      = im.cfg.ds.finalRxAfterResponseTxDelayUus + im.cfg.ds.rxTimeoutUus;
        } else {
            rxAfterTxDelayUus = im.cfg.ds.finalRxAfterResponseTxDelayUus;
            rxTimeoutUus      = im.cfg.ds.rxTimeoutUus;
        }
    }

    uint64_t pollRxTs = 0;
    const detail::ResponseArmOutcome arm = detail::buildAndArmResponse(
        respPrefix, /*responseExpected=*/ds, rxAfterTxDelayUus, rxTimeoutUus, parsed.sequence, im.cfg.panId,
        im.cfg.shortAddr, parsed.src, responseTxDelayUus, im.radio->txAntennaDelay(),
        static_cast<uint8_t>(im.radio->config().timing_profile), pollRxTs);

    if (arm.dataFailed || arm.startFailed || arm.wedged) {
        failResponseToListen(im); // txFailures++ (polls/restarts はすでに上で確定済み)
        return;
    }

    im.exchangeStartUs = esp_timer_get_time();

    if (ds) {
        // §2.2: move straight to WaitFinal without waiting for TXFRS here -
        // DWT_RESPONSE_EXPECTED auto-starts RX once the TX actually
        // completes, exactly like respondDSRange() itself, which also does
        // not wait for the Response's TXFRS before entering its Final-wait
        // loop.
        im.waitPeer       = parsed.src;
        im.waitSeq          = parsed.sequence;
        im.waitPollRxTs64  = pollRxTs;

        // Host deadline (§2.2): responseTxDelayUus (Poll RMARKER -> Response
        // TX time) + finalRxAfterResponseTxDelayUus (Response TX end ->
        // Final RX window opens) + rxTimeoutUus (Final RX window length),
        // UUS -> real ms via the same *1.02564 factor used elsewhere in this
        // codebase (e.g. firmware/anchor/main/main.cpp's boot log), + 2ms
        // margin, measured from "now" (Response just armed). Unchanged by
        // listenImmediatelyAfterTx: it only redistributes
        // finalRxAfterResponseTxDelayUus/rxTimeoutUus between "wait before
        // opening RX" and "RX timeout length" above - their SUM (used here)
        // is the same either way, so the absolute deadline does not move.
        const double totalUus = static_cast<double>(im.cfg.ds.responseTxDelayUus) +
                                 static_cast<double>(im.cfg.ds.finalRxAfterResponseTxDelayUus) +
                                 static_cast<double>(im.cfg.ds.rxTimeoutUus);
        const uint32_t totalMs = static_cast<uint32_t>((totalUus * 1.02564) / 1000.0) + 2U;
        im.waitDeadlineMs        = detail::nowMs() + totalMs;
        im.state                  = State::WaitFinal;
        uwb_port_irq_clear_pending();

        // dwt_starttx() succeeded and the wedged-errata check passed - same
        // success criterion respondDSRange() itself uses (it never
        // re-confirms the Response's own TXFRS either).
        withStatsLock(im, [](ResponderStats& s) { s.responses++; });
        return;
    }

    // SS: wait for TXFRS (bounded, matches respondRange()) before -> Listen.
    const bool txOk = detail::waitTxFrsBounded(20);
    if (!txOk) {
        detail::stopRadioAndClearTxStatus();
    }
    dwt_setrxtimeout(0);
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
    uwb_port_irq_clear_pending();
    im.state = State::Listen;

    withStatsLock(im, [txOk](ResponderStats& s) {
        if (txOk) {
            s.responses++;
        } else {
            s.txFailures++;
        }
    });
}

/**
 * 2026-08-30 実機結果の監査 (badDistance)。coordinator の指定通り
 * (-0.5m, 500m) の範囲外を「ありえない値」として扱う。denominator<=0
 * (dist.valid==false) は符号が完全に壊れたケースで別扱い（そちらは
 * detail::computeDsDistance() 自体が RangeTimestampInvalid 相当として
 * 検出済み）。ここはそれを通り抜けた「符号は壊れていないが値が
 * 物理的にありえない」ケース（実機で見つかった -14.979m はこちら:
 * denominator>0 だが tofDtu が負になった、または極端に大きくなった
 * ケース）を拾う。
 * Rejects a computed distance outside (-0.5m, 500m) even when
 * detail::computeDsDistance() reports it as "valid" (denominator>0) -
 * catches the case a real run found (-14.979 m), which was not itself
 * denominator<=0.
 */
constexpr int32_t kMinPlausibleDistanceMm = -500;
constexpr int32_t kMaxPlausibleDistanceMm = 500000;

/**
 * @brief Action::ComputeAndRespond (WaitFinal + matching Final). Distance
 * (asymmetric DS-TWR) and the Result ("DWD") send both go through
 * detail::computeDsDistance()/detail::sendDsResult() - the SAME helpers
 * respondDSRange() calls (uwb_qm33120_internal.hpp).
 */
void onComputeAndRespond(Responder::Impl& im)
{
    dwt_writesysstatuslo(SYS_STATUS_ALL_RX_GOOD | DWT_INT_TXFRS_BIT_MASK);

    const uint16_t peer = im.waitPeer;
    const uint8_t seq   = im.waitSeq;

    withStatsLock(im, [](ResponderStats& s) { s.finals++; });

    const detail::DsDistanceResult dist = detail::computeDsDistance(im.finalFrameBuf, im.waitPollRxTs64);
    if (!dist.valid) {
        // RangeTimestampInvalid-equivalent: extremely rare (bad timestamps),
        // no Result sent, no RangeEvent.
        withStatsLock(im, [](ResponderStats& s) { s.other++; });
        dwt_setrxtimeout(0);
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        uwb_port_irq_clear_pending();
        im.state = State::Listen;
        return;
    }
    if ((dist.distanceMm < kMinPlausibleDistanceMm) || (dist.distanceMm > kMaxPlausibleDistanceMm)) {
        // ResponderStats::badDistance のコメント参照。この Final から得た
        // ペア(peer,seq)は既に消費済みなので Result は送らず・RangeEvent も
        // 出さない - タグ側はこの試行を失敗として扱い、通常のリトライで
        // 回復する（他の失敗パス - finalTimeouts 等 - と同じ扱い）。
        withStatsLock(im, [](ResponderStats& s) { s.badDistance++; });
        dwt_setrxtimeout(0);
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        uwb_port_irq_clear_pending();
        im.state = State::Listen;
        return;
    }

    const detail::DsResultSendOutcome send = detail::sendDsResult(
        seq, im.cfg.panId, im.cfg.shortAddr, peer, dist.distanceMm, im.cfg.ds.resultRepeatCount,
        im.cfg.ds.resultRepeatGapMs);

    dwt_setrxtimeout(0);
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
    uwb_port_irq_clear_pending();
    im.state = State::Listen;

    if (send.sentCount == 0) {
        withStatsLock(im, [](ResponderStats& s) { s.txFailures++; });
        return;
    }

    const int64_t nowUs = esp_timer_get_time();
    withStatsLock(im, [&dist](ResponderStats& s) {
        s.results++;
        s.distance.add(static_cast<float>(dist.distanceMm));
    });

    RangeEvent ev;
    ev.peer       = peer;
    ev.seq         = seq;
    ev.distanceMm = dist.distanceMm;
    ev.elapsedUs   = static_cast<uint32_t>(nowUs - im.exchangeStartUs);
    ev.tUs          = nowUs;
    if ((im.eventQueue != nullptr) && (xQueueSend(im.eventQueue, &ev, 0) != pdTRUE)) {
        // 満杯なら捨てる - radio タスクは決してブロックしない (§2.1).
        withStatsLock(im, [](ResponderStats& s) { s.eventDrops++; });
    }
}

/** @brief Action::Abandon (WaitFinal + Event::RxTimeout: RXFTO or the host deadline exceeded). */
void onAbandon(Responder::Impl& im)
{
    detail::stopRadioAndClearIoStatus(); // forcetrxoff() (the Response's own TX may still be armed/pending) + clear RX+TX status.
    dwt_setrxtimeout(0);
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
    uwb_port_irq_clear_pending();
    im.state = State::Listen;
    withStatsLock(im, [](ResponderStats& s) { s.finalTimeouts++; });
}

void handleEvent(Responder::Impl& im, Event ev, const FrameSummary& frame)
{
    switch (decide(im.state, ev, frame, im.cfg)) {
    case Action::Ignore:
        onIgnore(im, ev);
        return;
    case Action::CheckIdleAndRearm:
        onCheckIdleAndRearm(im);
        return;
    case Action::RespondSs:
        onRespond(im, /*ds=*/false);
        return;
    case Action::RespondDs:
        onRespond(im, /*ds=*/true);
        return;
    case Action::ComputeAndRespond:
        onComputeAndRespond(im);
        return;
    case Action::Abandon:
        onAbandon(im);
        return;
    }
}

/** @brief §1's idleTickMs (default 20ms - both the liveness alarm AND the edge-loss safety net), bounded further by however long is left of a WaitFinal deadline so service() returns promptly at/after it. */
uint32_t computeWaitMs(const Responder::Impl& im)
{
    uint32_t waitMs = (im.cfg.idleTickMs == 0) ? 1 : im.cfg.idleTickMs;
    if (im.state == State::WaitFinal) {
        const uint32_t now = detail::nowMs();
        const uint32_t remaining = (im.waitDeadlineMs > now) ? (im.waitDeadlineMs - now) : 0;
        if (remaining < waitMs) {
            waitMs = (remaining == 0) ? 1 : remaining;
        }
    }
    return waitMs;
}

/**
 * @brief §1/§2.2's "判読の経路は1本" - the ONE place IRQ-vs-polling differ.
 * Blocks (or spins) for up to `waitMs`; the caller re-reads
 * dwt_readsysstatuslo() itself afterwards regardless of how this returns
 * (IRQ is a wakeup hint only, docs/IRQ_POLICY.md) - everything past this
 * function (status classification, FSM, actions) is one shared code path.
 */
void waitForWake(Responder::Impl& im, uint32_t waitMs)
{
    if (im.radio->irqActive()) {
        (void)uwb_port_irq_wait(waitMs);
        return;
    }
    // Tight SPI-status poll with taskYIELD(), bounded to waitMs (§2.2).
    const uint32_t startMs = detail::nowMs();
    for (;;) {
        const uint32_t peek = dwt_readsysstatuslo();
        if ((peek & (DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)) != 0) {
            return;
        }
        if ((detail::nowMs() - startMs) >= waitMs) {
            return;
        }
        taskYIELD();
    }
}

/**
 * @brief 2026-08-30 実機結果の監査 (c): RXFCG（フレームCRC良好）が立って
 * いても、CIA（Channel Impulse Response Analyzer、先頭パス=leading edge
 * 検出アルゴリズム）が RX タイムスタンプの最終値を書き終えているとは
 * 限らない可能性の検査。SDK は両者を別ビットとして持ち
 * (deca_device_api.h:374 DWT_INT_CIADONE_BIT_MASK=0x400、
 * dw3720_deca_vals.h:157 `SYS_STATUS_RXOK = (DWT_INT_RXFCG_BIT_MASK |
 * DWT_INT_CIA_DONE_BIT_MASK)` という結合マスクを別途定義)、
 * `SYS_STATUS_ALL_RX_GOOD`（deca_device_api.h:432、このファイルが
 * clearに使っているマスク）自体もCIADONEを含む＝ベンダは両者を
 * 「良好受信に伴う一体のイベント群」として扱っている。
 * `ull_readrxtimestamp()`（dw3720_device.c:4069、RX_TIME_0_ID の生レジスタ
 * 読み出しのみ）はCIA完了を自分では待たない。
 *
 * 【未確認・完全には立証できていない】Qorvo公式リファレンス
 * (assets/DW3_QM33_SDK_1.1.1/.../ex_05b_ds_twr_resp/ds_twr_responder.c:164)
 * の受信待ちループ自体もRXFCGのみを見ており、CIADONEを個別に待たない
 * （shared_functions.c:588 waitforsysstatus()もタイトスピンで遅延を
 * 入れない）。したがって「RXFCG直後は常にCIA未完了でありうる」という
 * 主張は公式リファレンスの実装とは整合しない。それでも実機の症状
 * （距離±0.5〜500mの範囲外を含む異常値がポーリング実行でのみ出現、
 * IRQ実行・旧respondDSRange()のポーリング(1ms周期のvTaskDelay)実行では
 * 未検出）は「本 Responder のポーリング待ち(waitForWake()、taskYIELD()に
 * よるSPIレジスタのタイトスピン、旧コードのuwb_port_irq_wait(1)相当より
 * 高頻度)でだけ観測される」という一致を示しており、SYS_STATUS_ALL_RX_GOOD
 * がCIADONEを同列に扱っている事実と合わせて、無視できない仮説として
 * この待ちを追加する。原因を完全に特定した、とは主張しない
 * （文書化した上で導入する防御的な緩和策。距離レンジの健全性チェック
 * (onComputeAndRespond()のbadDistance判定)が最終防衛線）。
 *
 * 挙動: 引数の status に既にCIADONEが立っていれば即座に返す(追加コスト0)。
 * 立っていなければ、上限 boundUs [µs] まで dwt_readsysstatuslo() を
 * 追加でタイトスピンし、CIADONEが立ち次第、その時点の status を返す
 * （立たないまま上限に達しても、最後に読んだ status をそのまま返す -
 * RXFCG自体は既に確定した「良好受信」の根拠なので、処理を進めることに
 * 変わりはない。あくまで後続のタイムスタンプ読み出しに猶予を与えるだけ）。
 *
 * Audit finding (c): RXFCG alone does not guarantee the CIA (leading-edge
 * timestamp estimator) has finished writing the final RX timestamp -
 * ull_readrxtimestamp() (dw3720_device.c:4069) is a raw register read with
 * no CIA-completion wait of its own, and CIADONE is a distinct SYS_STATUS
 * bit (DWT_INT_CIADONE_BIT_MASK, deca_device_api.h:374) that
 * SYS_STATUS_ALL_RX_GOOD (deca_device_api.h:432) and the vendor's own
 * SYS_STATUS_RXOK combined mask (dw3720_deca_vals.h:157) both bundle
 * alongside RXFCG. NOT fully proven: Qorvo's own reference responder
 * (ex_05b_ds_twr_resp/ds_twr_responder.c:164) and its waitforsysstatus()
 * helper (shared_functions.c:588) also check RXFCG alone, with no delay in
 * their poll loop either - so "RXFCG can fire before CIADONE" is not
 * confirmed by the vendor's own usage pattern. Added as a documented,
 * low-cost defensive mitigation because it lines up with the observed
 * symptom (bad distances only in this Responder's polling mode, which
 * spins tighter than both this Responder's own IRQ path and the legacy
 * respondDSRange()'s 1ms-cadence poll, neither of which showed the defect)
 * - not a proven root cause. The distance-range sanity check in
 * onComputeAndRespond() (ResponderStats::badDistance) is the actual last
 * line of defense regardless of this mitigation's effect.
 *
 * @param status   直前に読んだ SYS_STATUS（RXFCG が立っている前提）。
 * @param boundUs  追加で待つ上限 [µs]。
 * @return CIADONE を確認できた（またはできなかった）時点の最新 status。
 */
uint32_t waitForCiaDone(uint32_t status, uint32_t boundUs)
{
    if ((status & DWT_INT_CIADONE_BIT_MASK) != 0) {
        return status;
    }
    const int64_t startUs = esp_timer_get_time();
    while ((esp_timer_get_time() - startUs) < static_cast<int64_t>(boundUs)) {
        status = dwt_readsysstatuslo();
        if ((status & DWT_INT_CIADONE_BIT_MASK) != 0) {
            break;
        }
    }
    return status;
}

} // namespace

Responder::Responder() : _impl(new Impl())
{
}

Responder::~Responder()
{
    end();
    if ((_impl != nullptr) && (_impl->eventQueue != nullptr)) {
        vQueueDelete(_impl->eventQueue);
    }
    delete _impl;
}

bool Responder::begin(Qm33120& radio, const ResponderConfig& cfg)
{
    if ((_impl == nullptr) || !radio.isInitialized()) {
        return false;
    }
    Impl& im = *_impl;

    im.radio             = &radio;
    im.cfg                = cfg;
    im.state               = State::Listen;
    withStatsLock(im, [](ResponderStats& s) { s = ResponderStats{}; });

    if (im.eventQueue == nullptr) {
        im.eventQueue = xQueueCreate(kEventQueueLen, sizeof(RangeEvent));
        if (im.eventQueue == nullptr) {
            return false;
        }
    } else {
        xQueueReset(im.eventQueue);
    }

    // Enable RX with no timeout - "受信は無期限" (§1). No dwt_setdblrxbuffmode()
    // call anywhere in this file - see this file's header comment (RXAUTR
    // decision).
    detail::stopRadioAndClearIoStatus();
    dwt_setpreambledetecttimeout(0);
    dwt_setrxaftertxdelay(0);
    dwt_setrxtimeout(0);
    if (dwt_rxenable(DWT_START_RX_IMMEDIATE) != DWT_SUCCESS) {
        return false;
    }
    uwb_port_irq_clear_pending();

    im.began = true;
    return true;
}

void Responder::end()
{
    if ((_impl == nullptr) || !_impl->began) {
        return;
    }
    dwt_forcetrxoff();
    _impl->began = false;
}

/**
 * §1 (2026-08-30 revision) edge-loss discipline: "状態を読む -> 空になるまで
 * 処理 -> 眠る直前にもう一度読む". A single wait (waitForWake(), the one
 * place IRQ/polling differ) is followed by a drain loop that keeps reading
 * dwt_readsysstatuslo() and handling whatever it finds until a read comes
 * back with none of RXFCG/RX_TO/RX_ERR set - that same clean read doubles
 * as "one more read right before going back to sleep" (there is nothing
 * left unhandled at the moment this function returns to the caller's own
 * `while (1) { service(); }` loop, which is the next "about to sleep"
 * point). If the very first read after waking is already clean, this was a
 * genuine idle wakeup (Event::Tick) - fired exactly once, not looped
 * forever doing nothing.
 */
void Responder::service()
{
    if ((_impl == nullptr) || !_impl->began) {
        return;
    }
    Impl& im = *_impl;

    // Deadline already passed before we even wait: skip straight to Abandon
    // rather than doing a pointless wait/poll pass first.
    if ((im.state == State::WaitFinal) && (detail::nowMs() >= im.waitDeadlineMs)) {
        handleEvent(im, Event::RxTimeout, FrameSummary{});
        return;
    }

    waitForWake(im, computeWaitMs(im));

    bool handledAny = false;
    for (;;) {
        uint32_t status = dwt_readsysstatuslo();
        const bool nothingPending =
            (status & (DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)) == 0;
        if (nothingPending) {
            if (!handledAny) {
                // Nothing was pending on the very first read either - a
                // genuine idle wakeup, not a drained event.
                handleEvent(im, Event::Tick, FrameSummary{});
            }
            return; // Clean read - safe to let the caller go back to sleep.
        }
        handledAny = true;

        Event ev;
        FrameSummary frame;
        if ((status & DWT_INT_RXFCG_BIT_MASK) != 0) {
            // 2026-08-30 実機結果の監査 (c) - waitForCiaDone() のコメント参照。
            // RX タイムスタンプ（Poll は onRespond() 内、Final は
            // computeAndRespond() 内で読む）が最終値になっているとより
            // 確信できるよう、CIADONE にも短時間だけ猶予を与える。
            status = waitForCiaDone(status, kCiaDoneWaitBoundUs);
            ev     = Event::RxFrame;
            frame  = classifyFrame(im);
        } else {
            withStatsLock(im, [status](ResponderStats& s) { s.lastRxStatus = status; });
            const bool isTimeout =
                (status & ((uint32_t)DWT_INT_RXFTO_BIT_MASK | (uint32_t)DWT_INT_RXPTO_BIT_MASK)) != 0;
            ev = isTimeout ? Event::RxTimeout : Event::RxError;
        }

        // Fold "the WaitFinal deadline has now passed" into RxTimeout
        // regardless of what actually woke us (RxError, or a stray
        // non-Final good frame) - see uwb_qm33120_responder_fsm.hpp's header
        // comment. A genuinely arrived good frame is always classified and
        // handled on its own merits instead, so a real matching Final is
        // never dropped purely because of a benign timing coincidence at the
        // deadline boundary.
        if ((im.state == State::WaitFinal) && (ev != Event::RxFrame) && (detail::nowMs() >= im.waitDeadlineMs)) {
            ev = Event::RxTimeout;
        }

        handleEvent(im, ev, frame);

        if (ev == Event::RxTimeout) {
            return; // Abandoned (or never entered) WaitFinal - nothing else to drain from this wake.
        }
        // Loop back and re-read status: handling the event above already
        // cleared its own bits (and re-armed RX), so this next read is both
        // "check for another back-to-back event" and, once it comes back
        // clean, "read once more before sleeping".
    }
}

bool Responder::popEvent(RangeEvent& out)
{
    if ((_impl == nullptr) || (_impl->eventQueue == nullptr)) {
        return false;
    }
    return xQueueReceive(_impl->eventQueue, &out, 0) == pdTRUE;
}

ResponderStats Responder::snapshot() const
{
    ResponderStats copy;
    if (_impl != nullptr) {
        portENTER_CRITICAL(&_impl->statsMux);
        copy = _impl->stats;
        portEXIT_CRITICAL(&_impl->statsMux);
    }
    return copy;
}

} // namespace uwb
