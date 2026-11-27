// ============================================================================
// warmup.h — "are the cells polarised yet?" as latched state, not a clock
//            subtraction. (Review batch D residual #4, found while extracting.)
//
// CAL AIR refuses to calibrate before the cells have polarised, and answers the
// diver with a countdown: "cells still warming up (N s left)". Every version of
// that test written here has been arithmetic on the boot epoch, and `millis()`
// is an UNSIGNED 32-bit clock that wraps at 49.7 days:
//
//   * `now < PREDIVE_WARMUP_MS` (original) — true again for the first 120 s
//     after every wrap, so a board 49.7 days up refused with a wrong reason;
//   * `int32_t(PREDIVE_WARMUP_MS - now) > 0` (batch A's "wrap-safe" fix) —
//     ALSO wrong, and worse in one place. The signed difference is exact only
//     for |a-b| < 2^31, and here a-b = 120000 - now with `now` free to reach
//     2^32: at now = 2^32 - 1000 the difference wraps POSITIVE (+121000) and
//     the fix refuses a machine 49.7 days into warm-up that the original form
//     accepted. It still refuses after the wrap, exactly as the original did.
//
// Comparing a boot-epoch budget against a wrapped clock cannot be fixed by the
// sign of the subtraction — once the clock wraps, "elapsed since boot" is not
// recoverable from it at all. So this gate does not keep comparing. The verdict
// "warm-up has passed" is monotone: once true, it stays true for the whole
// boot session. ControlTask calls update() EVERY cycle, so the comparison
// happens while `millis()` is still 120000 at most — small, monotone, before
// any wrap is reachable — and the latch flips exactly once, at t = warmupMs.
// After that the clock is never consulted again, and it cannot lie.
//
// What the gate still needs: the cycle actually runs during the first
// warmupMs (it does — ControlTask starts in setup() and cycles at ~2 ms), and
// warmupMs < 2^31. A budget over ~24.9 days is not supported; the shipped
// budget is 120 s.
//
// The countdown for the refusal (the `CALAIR,` warmupLeftS field and the
// console line) is still a subtraction, and it is honest only in the window
// where the gate is not latched: there, update() has seen now < warmupMs every
// cycle, so warmupMs - now is small, positive and exact.
// ============================================================================
#pragma once

#include <stdint.h>

namespace util {

class WarmupGate {
  public:
    // Call every control cycle. `warmupMs` is the budget, `nowMs` the boot
    // clock (millis()). Latches once `nowMs` reaches the budget and never
    // consults the clock again — see the header for why that is the fix.
    void update(uint32_t warmupMs, uint32_t nowMs) {
        if (!elapsed_) elapsed_ = (nowMs >= warmupMs);  // pre-wrap: exact
    }

    bool elapsed() const { return elapsed_; }

    // Countdown for the refusal message. Only called while !elapsed(), where
    // the difference is small and positive by construction. Never negative.
    uint32_t remainingMs(uint32_t warmupMs, uint32_t nowMs) const {
        if (elapsed_ || nowMs >= warmupMs) return 0u;
        return warmupMs - nowMs;  // exact: both small, unsigned
    }
    uint32_t remainingSeconds(uint32_t warmupMs, uint32_t nowMs) const {
        return remainingMs(warmupMs, nowMs) / 1000u;
    }

  private:
    bool elapsed_ = false;
};

}  // namespace util
