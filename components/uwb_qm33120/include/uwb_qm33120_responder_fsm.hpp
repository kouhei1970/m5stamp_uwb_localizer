/**
 * @file uwb_qm33120_responder_fsm.hpp
 * @brief docs/ARCHITECTURE_V2.md §2.2/§2.3: the event-driven anchor
 * responder's pure decision logic - `uwb::decide()` - plus the config/stats/
 * event types it and `uwb::Responder` (uwb_qm33120_responder.hpp/.cpp) share.
 *
 * ESP-IDF/Qorvo-SDK-free (only <cstdint>, uwb_qm33120_twr_config.hpp and
 * uwb_qm33120_distance_stats.hpp, both themselves ESP-IDF-free), so
 * tests/host/responder_fsm can #include this header and link nothing but
 * itself - no qm33120w_sdk, no ESP-IDF, no hardware.
 *
 * `uwb::ResponderConfig` / `uwb::ResponderStats` / `uwb::RangeEvent` live
 * here rather than in uwb_qm33120_responder.hpp (the file
 * docs/ARCHITECTURE_V2.md §2.3 nominally puts them in) because `decide()`'s
 * signature is fixed at `decide(State, Event, const FrameSummary&, const
 * ResponderConfig&)` and therefore needs ResponderConfig's full definition;
 * uwb_qm33120_responder.hpp in turn `#include`s this header, so a caller who
 * only ever includes uwb_qm33120_responder.hpp still sees every type §2.3
 * lists - the split is purely about which file physically defines what.
 *
 * --- Why `uwb::TwrMethod`, not `uwb::RangingMethod` ---
 * docs/ARCHITECTURE_V2.md §2.3's ResponderConfig pseudocode uses
 * `RangingMethod method = RangingMethod::DS;`. That name is already taken:
 * components/uwb_ranging/include/uwb_ranging_scheduler.hpp defines
 * `enum class uwb::RangingMethod { SS, DS };` for RangingScheduler
 * (tag-side). Reusing it here would need uwb_qm33120 to depend on
 * uwb_ranging - backwards (uwb_ranging already depends on uwb_qm33120) and
 * would drag ESP-IDF into this header (uwb_ranging_scheduler.hpp itself
 * `#include`s uwb_qm33120.hpp), breaking the host-testability requirement
 * above. Moving RangingMethod down into uwb_qm33120 (the textbook fix, since
 * it is fundamentally a TWR-level SS/DS choice, not a ranging-service-level
 * one) would require editing uwb_ranging_scheduler.hpp, which this task was
 * explicitly told not to touch (another agent owns components/uwb_ranging
 * this session). So this header defines its own `uwb::TwrMethod` (same SS/DS
 * meaning) instead of silently risking an ODR collision. Flagged here and in
 * the implementation report; unifying the two enums is a follow-up for
 * whoever next coordinates uwb_qm33120 and uwb_ranging together.
 *
 * --- Why `State` stays a bare 2-value enum (no embedded peer/seq/deadline) ---
 * docs/ARCHITECTURE_V2.md §2.2 describes the state as `WaitFinal{peer, seq,
 * timestamps, deadline}`, but `decide()`'s signature has no room for that
 * (and it should not: a pure function reused by tests/host/responder_fsm
 * cannot also carry live radio state). The peer/seq/deadline bookkeeping
 * instead lives in `Responder::Impl` (uwb_qm33120_responder.cpp - the
 * hardware layer, which has current wall-clock time and TX/RX register
 * access `decide()` deliberately does not). Two things the table's
 * WaitFinal-only rows need from that bookkeeping are threaded through
 * differently instead:
 *  - "is this the Final we are waiting for" (peer AND sequence match): the
 *    caller resolves this itself (it already has to - it is the one that
 *    remembers which peer/seq it sent the Response to, exactly like
 *    respondDSRange()'s existing Final-wait loop does with `parsed.src`/
 *    `parsed.sequence`) and reports the result as `FrameSummary::kind ==
 *    FrameKind::Final` (already resolved -> yes) vs `FrameKind::Other`
 *    (anything else, including a well-formed Final from the wrong
 *    peer/sequence). See FrameSummary's own comment.
 *  - "is this Poll from the peer we are already exchanging with, or a
 *    different one" (needed to apply `restartOnForeignPoll` correctly):
 *    reported via `FrameSummary::matchesTrackedPeer`, again computed by the
 *    caller from its own private peer bookkeeping. In `Listen` there is no
 *    "current peer" to compare against, so the caller always sets this true
 *    there (every properly addressed Poll starts a fresh exchange).
 *  - "has the WaitFinal deadline passed" (Response TX time +
 *    finalRxAfterResponseTxDelayUus + rxTimeoutUus + margin,
 *    docs/ARCHITECTURE_V2.md §2.2): the caller pre-resolves this into which
 *    `Event` it passes - a hardware RXFTO AND a host-deadline-exceeded
 *    RxError both become `Event::RxTimeout` (both mean "give up, we are past
 *    the window"); an RxError seen *before* the deadline stays
 *    `Event::RxError` ("keep waiting"). This mirrors how respondRange()/
 *    respondDSRange() already fold "RXFTO" and "ran out of hostTimeoutMs"
 *    into the same Error::RxTimeout outcome.
 */
#pragma once

#include <cstdint>

#include "uwb_qm33120_distance_stats.hpp"
#include "uwb_qm33120_twr_config.hpp"

namespace uwb {

/**
 * @brief SS-TWR / DS-TWR choice for uwb::Responder. Same SS/DS meaning as
 * uwb::RangingMethod (components/uwb_ranging), deliberately a distinct type
 * - see this file's header comment "Why uwb::TwrMethod, not
 * uwb::RangingMethod".
 */
enum class TwrMethod : uint8_t {
    SS, //!< respondRange()'s per-exchange logic (Poll -> Response, no Final).
    DS, //!< respondDSRange()'s per-exchange logic (Poll -> Response -> Final -> Result).
};

/**
 * @brief uwb::Responder configuration (docs/ARCHITECTURE_V2.md §2.3).
 * A single Responder instance runs exactly one `method` for its whole
 * lifetime (matching how respondRange()/respondDSRange() are two separate
 * blocking calls the firmware picks between at compile/Kconfig time, not
 * something inferred per received Poll).
 */
struct ResponderConfig {
    TwrMethod method       = TwrMethod::DS; //!< SS / DS.
    uint16_t panId          = 0xDECA;
    uint16_t shortAddr      = 0x0002; //!< This anchor's own short address; overrides ss.responderAddress/ds.responderAddress at each exchange (see uwb_qm33120_responder.cpp).
    RangeConfig ss;                    //!< SS timing (address fields overridden by shortAddr/panId).
    DSRangeConfig ds;                  //!< DS timing (same override).
    /**
     * Wakeup interval used while waiting with nothing pending. §1
     * (2026-08-30 revision): NOT just a liveness check - it is also the
     * safety net against a missed IRQ edge (the IRQ line is level-driven
     * while a status bit is set but edge-triggered on the GPIO side, so an
     * event that lands between "read status" and "clear status" can lose
     * its edge). Default lowered from 1000 to 20 (ms) accordingly - one
     * extra SPI status read every 20ms while idle in Listen is negligible
     * cost for that safety margin.
     */
    uint32_t idleTickMs        = 20;
    bool restartOnForeignPoll   = true; //!< Whether a WaitFinal-time Poll from a DIFFERENT peer restarts the exchange (§2.2's "複数タグ" note); a Poll from the SAME peer always restarts regardless of this flag.
};

/** @brief docs/ARCHITECTURE_V2.md §2.3 - all cumulative counters + distance stats. */
struct ResponderStats {
    uint32_t polls         = 0; //!< Valid Polls accepted while in Listen (fresh exchanges started).
    uint32_t responses     = 0; //!< Response frames successfully transmitted (SS or DS).
    uint32_t finals        = 0; //!< Matching Final frames received (DS only).
    uint32_t results       = 0; //!< Result ("DWD") frames successfully transmitted (DS only).
    uint32_t restarts      = 0; //!< WaitFinal exchanges abandoned because a Poll (same or, if restartOnForeignPoll, a different peer) arrived instead of the expected Final.
    uint32_t finalTimeouts = 0; //!< WaitFinal exchanges abandoned by RXFTO or the host deadline, with no Poll and no matching Final.
    uint32_t rxErrors      = 0; //!< Non-timeout RX errors absorbed (Listen: ignored and re-armed; WaitFinal: re-armed and kept waiting, within the deadline).
    uint32_t txFailures    = 0; //!< Delayed-TX failures (HPDWARN / the UM §9.4.1 "wedged" errata, detail::abortIfDelayedTxWedged()) while sending a Response or Result.
    uint32_t rearms        = 0; //!< Times the Tick handler found the chip not receiving in Listen and had to re-enable RX itself.
    uint32_t other         = 0; //!< Good frames (RXFCG) that were neither the Poll/Final this Responder was looking for, nor accepted as a restart.
    uint32_t lastRxStatus  = 0; //!< Raw SYS_STATUS (low word) captured at the most recent RX error/timeout, for diagnostics (same convention as RangeResult::rxStatus etc.).
    /**
     * RangeEvents dropped because the length-8 queue was full when
     * popEvent()'s producer side (Responder::service()) tried to push one
     * (§2.1: "満杯なら捨てる（radio タスクは決してブロックしない）"). Not
     * one of §2.3's listed fields, added because §2.2's own instructions
     * explicitly ask for a drop counter ("xQueueSend with 0 timeout (drop if
     * full, count drops)").
     */
    uint32_t eventDrops    = 0;
    DistanceStats distance;      //!< n / mean / std of computed distances (DS only - SS's anchor side never learns the distance, respondRange() cpp comment).
};

/** @brief One completed DS-TWR exchange's result (docs/ARCHITECTURE_V2.md §2.3). */
struct RangeEvent {
    uint16_t peer       = 0; //!< The tag's short address.
    uint8_t seq         = 0; //!< Poll's sequence number (same value threaded through Response/Final/Result).
    int32_t distanceMm  = 0;
    uint32_t elapsedUs  = 0; //!< Poll-to-Result wall-clock time.
    int64_t tUs         = 0; //!< Timestamp (esp_timer_get_time()) the Result was sent.
};

/* -------------------------------------------------------------------------
 * Pure decision logic (docs/ARCHITECTURE_V2.md §2.2's table).
 * ---------------------------------------------------------------------- */

/** @brief Responder's coarse state (§2.2). See this file's header comment for why WaitFinal's peer/seq/deadline are NOT embedded here. */
enum class State : uint8_t {
    Listen,    //!< RX armed, waiting for any Poll.
    WaitFinal, //!< Response sent (DS only), waiting for the matching Final.
};

/** @brief The hardware-visible events decide() reacts to (§2.2). */
enum class Event : uint8_t {
    RxFrame,   //!< RXFCG: a well-formed frame was decoded - see `frame` for what kind.
    RxError,   //!< A non-timeout RX error (SYS_STATUS_ALL_RX_ERR) - in WaitFinal, only when still within the deadline (see header comment).
    RxTimeout, //!< RXFTO, OR (in WaitFinal only) the host deadline was exceeded - the caller folds both into this one Event (see header comment).
    TxDone,    //!< A delayed/immediate TX completed (DWT_INT_TXFRS_BIT_MASK). Listed for parity with §2.2's event vocabulary; TX completion is waited for synchronously inside the hardware layer's own bounded poll (same pattern as respondRange()/respondDSRange()'s `while (... < 20) { ...TXFRS... }` loops) and is never itself routed through decide() - so decide() treats it as a no-op (Action::Ignore) in every state.
    Tick,      //!< The idle wakeup (idleTickMs) fired with nothing else pending.
};

/** @brief What kind of frame `RxFrame`/`RxError` was about (kind==None for RxError/RxTimeout/TxDone/Tick, which have no associated frame). */
enum class FrameKind : uint8_t {
    None,  //!< No frame (RxError / RxTimeout / TxDone / Tick).
    Poll,  //!< A Poll addressed to this responder, of the method (SS "TWP"/DS "DWP") this Responder is configured for.
    Final, //!< The Final the caller has already validated as belonging to the in-flight WaitFinal exchange (peer AND sequence match - see header comment). A Final-shaped frame that does NOT match is reported as Other, not Final.
    Other, //!< Anything else: unaddressed/foreign-PAN traffic, a Poll of the wrong method, a stray Final/Response/Result, noise that happened to pass CRC, ...
};

/**
 * @brief Everything decide() needs to know about one received frame.
 * `kind`/`matchesTrackedPeer` are pre-resolved by the caller (see this
 * file's header comment for exactly what each already encodes); the rest
 * are the plain parsed header fields, checked here against `cfg` for
 * completeness (so a caller that does NOT pre-filter by pan/dst still gets
 * correct behaviour from decide() alone - see the "Poll from other pan
 * ignored" host test).
 */
struct FrameSummary {
    FrameKind kind    = FrameKind::None;
    uint16_t src        = 0;
    uint16_t dst         = 0;
    uint16_t panId        = 0;
    uint8_t seq            = 0;
    TwrMethod method        = TwrMethod::DS; //!< The Poll's own declared method (parsed "TWP"/"DWP" payload prefix). Only meaningful when kind==Poll.
    /**
     * True when this frame's source matches the peer the caller is
     * currently tracking:
     *  - In State::Listen there is no "current peer" - the caller always
     *    sets this true (every validly addressed Poll starts a fresh
     *    exchange, so this field has no effect there).
     *  - In State::WaitFinal: true for the peer the Response was sent to
     *    (whether this is the matching Final, or that same peer retrying
     *    with another Poll); false for any other peer's Poll ("foreign
     *    Poll", gated by ResponderConfig::restartOnForeignPoll).
     */
    bool matchesTrackedPeer = true;
};

/** @brief What the hardware layer should do next (docs/ARCHITECTURE_V2.md §2.2's "動作" column). Next-state is always inferable from (current State, this Action) - see decide()'s doc comment. */
enum class Action : uint8_t {
    /** No frame-specific action; stay in the current state.
     *  - Listen: re-arm RX (dwt_rxenable(DWT_START_RX_IMMEDIATE)) after any
     *    non-Poll good frame or RX error (§2.2 counts these as `other`/
     *    `rxErrors`).
     *  - WaitFinal: re-arm RX (with a timeout appropriate to the remaining
     *    deadline, or 0 relying on the host deadline - implementation
     *    choice, §2.2's own note) and keep waiting, for an RX error or an
     *    unrelated good frame seen before the deadline, OR for a foreign
     *    Poll when cfg.restartOnForeignPoll is false, OR for Event::Tick
     *    (nothing to do while still within the deadline), OR for
     *    Event::TxDone (never generated by the caller - see Event::TxDone). */
    Ignore,
    /** Listen + Event::Tick only: read SYS_STATE_LO; re-enable RX (and count
     *  it as a `rearms`) only if the chip is not already receiving. */
    CheckIdleAndRearm,
    /** Begin an SS Response to `frame` (delayed TX, no W4R) -> Listen once
     *  TXFRS confirms. Returned for both a fresh Listen Poll and a
     *  WaitFinal-time restart (same physical response either way) - the
     *  caller tells the two apart from which State it called decide() with
     *  (Listen -> count `polls`+`responses`; WaitFinal -> count `restarts`
     *  instead of `polls`, still `responses` on success). Only returned
     *  when cfg.method == TwrMethod::SS. */
    RespondSs,
    /** Same as RespondSs, but for DS: delayed TX + W4R + RX timeout ->
     *  State::WaitFinal once TXFRS confirms. Only returned when
     *  cfg.method == TwrMethod::DS. */
    RespondDs,
    /** WaitFinal + the matching Final: compute the distance, send the
     *  Result ("DWD"), emit a RangeEvent, then -> Listen once TXFRS
     *  confirms (or -> Listen anyway, counted as a txFailure, if the Result
     *  TX fails - see Action's own note on TX failures below). */
    ComputeAndRespond,
    /** WaitFinal + Event::RxTimeout (RXFTO or host deadline exceeded, with
     *  no Poll and no matching Final in the meantime) -> Listen, counted as
     *  a finalTimeout. */
    Abandon,
};
/*
 * Delayed-TX failure (HPDWARN / the UM §9.4.1 "wedged" errata,
 * detail::abortIfDelayedTxWedged()) while executing RespondSs/RespondDs/
 * ComputeAndRespond is NOT modelled as a decide() outcome: it is a failure
 * mode of *carrying out* an Action decide() already returned, detected the
 * same way respondRange()/respondDSRange() already detect it (checking
 * dwt_starttx()'s return value and abortIfDelayedTxWedged() right after
 * arming the delayed TX). The hardware layer handles it inline
 * (forcetrxoff + re-arm + -> Listen + count txFailures) without another
 * decide() call, matching docs/ARCHITECTURE_V2.md §2.2's "任意" row.
 */

/**
 * @brief The one pure function driving uwb::Responder (docs/ARCHITECTURE_V2.md
 * §2.2/§2.3). See this file's header comment for the division of labour
 * between this function and its caller (peer/seq/deadline resolution).
 *
 * constexpr so tests/host/responder_fsm can `static_assert` it too, though
 * the host test suite mainly calls it at runtime for readable failure
 * messages.
 */
constexpr Action decide(State state, Event ev, const FrameSummary& frame, const ResponderConfig& cfg)
{
    const bool addressedToUs = (frame.panId == cfg.panId) && (frame.dst == cfg.shortAddr);
    const Action respond     = (cfg.method == TwrMethod::SS) ? Action::RespondSs : Action::RespondDs;
    const bool validPoll     = (frame.kind == FrameKind::Poll) && addressedToUs && (frame.method == cfg.method);

    if (state == State::Listen) {
        if (ev == Event::Tick) {
            return Action::CheckIdleAndRearm;
        }
        if ((ev == Event::RxFrame) && validPoll) {
            return respond;
        }
        return Action::Ignore; // RxError / RxTimeout / TxDone / any other or mismatched RxFrame.
    }

    // state == State::WaitFinal.
    if (ev == Event::RxTimeout) {
        return Action::Abandon;
    }
    if (ev == Event::RxFrame) {
        if ((frame.kind == FrameKind::Final) && frame.matchesTrackedPeer && addressedToUs) {
            return Action::ComputeAndRespond;
        }
        if (validPoll && (frame.matchesTrackedPeer || cfg.restartOnForeignPoll)) {
            return respond; // Same-peer retry always restarts; a foreign peer's Poll restarts only if cfg allows it.
        }
    }
    return Action::Ignore; // RxError (within the deadline) / Tick / TxDone / an ignored foreign Poll / any other non-matching RxFrame -> keep waiting.
}

} // namespace uwb
