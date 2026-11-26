// ============================================================================
// pinfuse.h — the solenoid output fuse, evaluated where the pin is DRIVEN,
// not only where the logic runs (review H2).
//
// The injector's maxOnMs fuse lives inside ctrl::Loop::cycle(): it can only
// fire on a control cycle that RETURNS. A ControlTask that stalls after
// writing the pin HIGH — on a flash write, a mutex, anything — leaves the
// valve open until the supervisor's stall reboot cuts it, and the stall that
// abandoned the pin is exactly the stall that prevents the fuse from running.
// That ordering is the bug: the fuse must not depend on the thing it is a
// fuse FOR.
//
// This module is the pure policy — no Arduino, no FreeRTOS, host-tested:
//   * whoever DRIVES the pin goes through drive(): after the fuse has tripped
//     the desired level is refused and LOW is driven. A stalled logic path
//     cannot re-assert HIGH through the back door.
//   * whoever WOUND the last HIGH goes through shouldCut(nowMs): true means
//     "the pin has been continuously HIGH longer than maxOnMs and the normal
//     path has not cut it". main.cpp evaluates that at BOTH ends of a control
//     iteration (before cycle(), so a recovered stall cuts the pin BEFORE any
//     new logic runs) and from the esp_timer alarm (so it is cut even when
//     the control task never returns at all).
//
// Continuous ON is wall time since the last LOW write to the pin, not "one
// pulse": a chain of back-to-back HIGH writes with no LOW between them is one
// continuous ON run, which is exactly what a stalled task produces.
//
// Latching: a tripped fuse latches LOW until clear(). From the diver's side a
// fused output and a stuck-open valve are the same hazard (O2 flowing, not
// stopping), so they get the same discipline as the injector's own fuse —
// it latches like a stuck-open valve because the observable outcome is the
// same. The board that trips this fuse has already lost a control path; it
// does not get to resume driving oxygen until it has been accounted for.
// ============================================================================
#pragma once

#include <cstdint>

namespace outfuse {

class SolFuse {
public:
    // The fuse's view of the pin. level()/sinceHighMs()/tripped() are the
    // state an alarm or the HB line may report; nothing here reads a clock —
    // every method is handed the clock, because the ONLY caller allowed to
    // own time is the one physically driving the pin.
    bool level() const { return high_; }
    bool tripped() const { return tripped_; }
    uint32_t trips() const { return trips_; }

    // Milliseconds the pin has been continuously HIGH as of nowMs (0 when LOW).
    // The run STARTS at the LOW->HIGH transition — idle time at LOW is not
    // open-valve time — and a LOW of any length between pulses resets it, so
    // back-to-back HIGH writes without an intervening LOW are one run.
    uint32_t continuousOnMs(uint32_t nowMs) const {
        return high_ ? nowMs - highStartMs_ : 0u;
    }

    // True when the pin is HIGH and has been continuously HIGH past maxOnMs.
    // This is the whole evaluation, and it takes no state: it can run in a
    // timer callback, at the top of a control iteration, or at the write site.
    bool shouldCut(uint32_t nowMs, uint32_t maxOnMs) const {
        return high_ && (nowMs - highStartMs_) > maxOnMs;
    }

    // Drive the pin. Returns the level ACTUALLY driven — after a trip that is
    // always false, whatever was asked. Timestamps must be monotonic.
    bool drive(uint32_t nowMs, bool want) {
        const bool lvl = tripped_ ? false : want;
        if (lvl == high_) return high_;
        high_ = lvl;
        if (lvl)
            highStartMs_ = nowMs;  // the valve opened at this write
        return high_;
    }

    // The forced cut: same as drive(false) plus the latch. Called by whoever
    // evaluates shouldCut() — the pin write never reaches this on its own.
    void cut(uint32_t nowMs) {
        if (!high_ && tripped_) return;
        if (high_) trips_++;
        high_ = false;
        highStartMs_ = nowMs;
        tripped_ = true;
    }

    // Re-arm. Firmware policy: only at a system re-entry the bench can see
    // (boot, dive-start edge), never silently from the logic that fused.
    void clear() { tripped_ = false; }

private:
    bool high_ = false;
    bool tripped_ = false;
    uint32_t trips_ = 0;
    uint32_t highStartMs_ = 0;  // start of the current (or last) HIGH run
};

}  // namespace outfuse
