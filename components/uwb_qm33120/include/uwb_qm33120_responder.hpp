/**
 * @file uwb_qm33120_responder.hpp
 * @brief docs/ARCHITECTURE_V2.md §2.2/§2.3: `uwb::Responder`, the
 * event-driven anchor state machine that replaces a `while(1)
 * respondRange()/respondDSRange()` loop with RX left enabled indefinitely
 * (§1's design principles) and a single "process one event and return"
 * `service()` call the radio task drives in its own loop.
 *
 * The public API below is exactly §2.3's:
 *   `ResponderConfig` / `ResponderStats` / `RangeEvent` (defined in
 *   uwb_qm33120_responder_fsm.hpp, re-exported here by #include - see that
 *   file's header comment for why) and `class Responder` with
 *   `begin()`/`service()`/`popEvent()`/`snapshot()`/`end()`.
 *
 * This header stays ESP-IDF-light on purpose (no FreeRTOS/Qorvo-SDK types
 * in the public surface): `Qm33120` is only forward-declared (Responder only
 * ever holds a reference to one), and the FreeRTOS queue / portMUX_TYPE /
 * Qm33120 include this class needs internally live behind a PImpl
 * (`struct Impl`, defined in uwb_qm33120_responder.cpp) - the same pattern
 * uwb_qm33120.hpp itself uses for Qm33120. Unlike
 * uwb_qm33120_responder_fsm.hpp this is not a *requirement* (only the FSM
 * header needs to be host-testable), just a low-cost consistency choice.
 */
#pragma once

#include "uwb_qm33120_responder_fsm.hpp"

namespace uwb {

class Qm33120; // uwb_qm33120.hpp - forward-declared only, see file header comment.

/**
 * @brief The anchor-side event-driven responder (docs/ARCHITECTURE_V2.md
 * §2.2/§2.3). Owns nothing about the radio itself (that stays in the
 * `Qm33120&` passed to begin()) - only the FSM state, per-exchange
 * bookkeeping, cumulative ResponderStats, and the RangeEvent queue.
 *
 * Threading contract (§2.1): exactly one task ("the radio task") may call
 * begin()/service()/end() - they are not reentrant and are not safe to call
 * from more than one task, mirroring components/uwb_port's own
 * single-task-use contract that Qm33120 already inherits. popEvent() and
 * snapshot() ARE safe to call from other tasks concurrently with
 * service() (queue + critical-section-protected copy respectively - see
 * the .cpp).
 */
class Responder {
public:
    Responder();
    ~Responder();
    Responder(const Responder&)            = delete;
    Responder& operator=(const Responder&) = delete;

    /**
     * @brief Enable RX (dwt_rxenable() with dwt_setrxtimeout(0) - i.e.
     * listen indefinitely, §1) and enter State::Listen. `radio` must already
     * be begin()/init()-ed (same precondition as respondRange()/
     * respondDSRange()). Keeps a reference to `radio` and a copy of `cfg`
     * for the lifetime of this Responder (until end() or destruction).
     * @return false if `radio` is not initialized, or if the initial
     *         dwt_rxenable() fails (Qm33120::lastError() has the reason).
     */
    bool begin(Qm33120& radio, const ResponderConfig& cfg);

    /**
     * @brief Process one event's worth of work and return - never blocks
     * longer than `cfg.idleTickMs` (Listen) or the current WaitFinal
     * exchange's own bounded waits (§1's "ホストタイムアウト" principle),
     * so the radio task can call this in a tight `while (1) { service(); }`
     * loop without starving other duties on other tasks/cores (§2.1).
     * A no-op (returns immediately) if begin() has not succeeded or end()
     * has already been called.
     */
    void service();

    /**
     * @brief Pop one completed exchange's result (DS only - SS never
     * produces a RangeEvent, since its anchor side never learns the
     * distance). Safe to call from a task other than the one calling
     * service().
     * @return false if the queue is empty (`out` left unmodified).
     */
    bool popEvent(RangeEvent& out);

    /**
     * @brief A thread-safe copy of the cumulative counters + distance
     * stats (portENTER_CRITICAL/portEXIT_CRITICAL-protected struct copy -
     * see the .cpp). Safe to call from a task other than the one calling
     * service().
     */
    ResponderStats snapshot() const;

    /** @brief Stop listening (dwt_forcetrxoff()) and release this Responder's state. Safe to call even if begin() was never called or already failed. */
    void end();

    /**
     * @brief Opaque implementation state (uwb_qm33120_responder.cpp), same
     * PImpl idiom as Qm33120::Impl. The forward declaration is public (not
     * `private:`, unlike Qm33120::Impl) only so uwb_qm33120_responder.cpp's
     * free helper functions (onRespond()/onIgnore()/handleEvent()/... - the
     * event handlers are free functions taking `Impl&`, not Responder
     * member functions, to keep the FSM-driving logic and the
     * Qm33120/uwb_port hardware calls in one flat set of small functions
     * rather than a long member-function body) can name `Responder::Impl`
     * in their own signatures; a free (non-member, non-friend) function
     * cannot otherwise use a private nested type name even from within the
     * same translation unit. The full `struct Impl` definition still lives
     * only in uwb_qm33120_responder.cpp, and `_impl` itself stays private,
     * so external callers still cannot observe or touch any Responder
     * state through this - true opacity is unaffected.
     */
    struct Impl;

private:
    Impl* _impl;
};

} // namespace uwb
