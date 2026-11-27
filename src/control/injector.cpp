#include "injector.h"

#include <algorithm>

namespace inj {

void Injector::logOn(uint32_t startMs, uint32_t endMs) {
    log_[logHead_] = {startMs, endMs};
    logHead_ = (logHead_ + 1) % MAX_LOG;
    if (logCount_ < MAX_LOG) logCount_++;
}

uint32_t Injector::pulsesInWindow() const {
    return logCount_; // capped history ~ window/gap; exact enough for telemetry
}

float Injector::dutyPct(uint32_t nowMs) const {
    uint32_t on = 0;
    for (int i = 0; i < logCount_; ++i) {
        const Pulse& p = log_[i];
        // Subtract first, then compare signed — the form the rest of this
        // codebase already uses (main.cpp's control-loop due checks). The two
        // unsigned comparisons this replaced (`nowMs >= p.startMs &&
        // p.startMs + windowMs >= nowMs`) both go false for the first window
        // after the 32-bit clock wraps, and `p.startMs + windowMs` can wrap
        // round past `nowMs` on its own. What that means in the water is the
        // point: duty is a diver-facing number, and the failure direction was
        // 0% — "the solenoid has not run at all this window" — printed about a
        // machine that had been injecting normally for the last ten seconds.
        // Signed differences are exact for any gap under 2^31 ms, which a
        // 10 s duty window is by four orders of magnitude.
        const int32_t ageMs = static_cast<int32_t>(nowMs - p.startMs);
        if (ageMs >= 0 && ageMs < static_cast<int32_t>(cfg_.windowMs)) {
            const int32_t onMs = static_cast<int32_t>(p.endMs - p.startMs);
            on += onMs > 0 ? static_cast<uint32_t>(onMs) : 0u;
        }
    }
    return 100.0f * on / static_cast<float>(cfg_.windowMs);
}

bool Injector::cutoffLatched(float v) const {
    // Latches at cutoff, releases only below highWall: no re-injection
    // hovering between the two walls.
    return cutoffLatch_;
}

inj::Output Injector::update(const Input& in) {
    Output out;
    out.stuckOpenLatched = stuckLatched_;

    const float wallSee = std::max(in.votedBar, in.highVetoBar);

    // ---- cutoff wall: hard, latching ---------------------------------------
    if (wallSee >= cfg_.cutoffBar) cutoffLatch_ = true;
    if (cutoffLatch_ && wallSee < cfg_.highWallBar) cutoffLatch_ = false;
    const bool wallBlocked = cutoffLatch_ || wallSee >= cfg_.highWallBar;
    out.inhibitedHigh = wallBlocked;

    // ---- stuck-open supervision (depth-stable, non-injecting only) ---------
    if (pulseRemMs_ == 0 && !stuckLatched_) {
        // Re-arm the evaluation window when it expires, when a pulse has just
        // ended, or when ppO2 genuinely falls back. A few mV of dither must
        // NOT re-arm it: that used to reset the timer on every sample and the
        // window never closed, hiding a valve that was wide open.
        if (!riseBaseValid_ || in.nowMs - riseBaseMs_ > cfg_.stuckEvalMs ||
            in.votedBar < riseBaseVal_ - cfg_.stuckHystBar) {
            riseBaseValid_ = true;
            riseBaseMs_ = in.nowMs;
            riseBaseVal_ = in.votedBar;
            offSinceMs_ = in.nowMs;
        } else if (in.depthStable &&
                   in.nowMs - offSinceMs_ >= cfg_.stuckEvalMs) {
            const float rise = in.votedBar - riseBaseVal_;
            const float secs = (in.nowMs - riseBaseMs_) / 1000.0f;
            if (rise >= cfg_.stuckRiseBar && rise / (secs > 0.001f ? secs : 0.001f)
                                            >= cfg_.stuckSlopeBarS) {
                stuckLatched_ = true;
                out.stuckOpenLatched = true;
            }
            // window elapsed without fault: slide baseline forward
            riseBaseMs_ = in.nowMs;
            riseBaseVal_ = in.votedBar;
            offSinceMs_ = in.nowMs;
        }
    }

    // ---- injection decision --------------------------------------------------
    const bool demand = (in.votedBar < in.setpointBar - cfg_.dbOnBar);
    out.pulseStarted = false;

    if (pulseRemMs_ > 0) {
        // mid-pulse: end on dt expiry, or early on crossing the off-band
        if (in.votedBar > in.setpointBar + cfg_.dbOffBar || wallBlocked ||
            !in.sensorHealthy) {
            logOn(pulseStartMs_, in.nowMs);
            pulseRemMs_ = 0;
            lastPulseEndMs_ = in.nowMs;
            riseBaseValid_ = false;
        } else {
            pulseRemMs_ = (pulseRemMs_ > in.dtMs) ? pulseRemMs_ - in.dtMs : 0;
            if (pulseRemMs_ == 0) {
                logOn(pulseStartMs_, in.nowMs);
                lastPulseEndMs_ = in.nowMs;
                riseBaseValid_ = false;
            } else {
                out.solenoid = true;
            }
        }
    }

    const bool wantDemand = in.autoMode && demand;
    const bool wantTest = testPending_ > 0;
    const bool wantPulse = wantDemand || wantTest;
    // Bench/calibration pulses are NOT demand-gated: the pulse cell test
    // must be able to raise ppO2 on purpose, even above the deadband.
    const bool canStart =
        !stuckLatched_ &&          // suspected open valve: no more O2 until cleared
        !wallBlocked &&
        in.sensorHealthy &&
        pulseRemMs_ == 0 &&
        wantPulse &&
        (in.nowMs - lastPulseEndMs_ >= cfg_.minGapMs || lastPulseEndMs_ == 0) &&
        (dutyPct(in.nowMs) + 100.0f * cfg_.pulseMs / cfg_.windowMs
             <= static_cast<float>(cfg_.maxDutyPct));

    if (canStart) {
        pulseRemMs_ = cfg_.pulseMs;
        pulseStartMs_ = in.nowMs;
        if (wantTest) testPending_--;  // this pulse is one of the requested ones
        pulses_++;
        out.pulseStarted = true;
        out.solenoid = true;
        riseBaseValid_ = false;
    } else if (in.autoMode && demand && !wallBlocked && in.sensorHealthy &&
               !stuckLatched_) {
        out.refused = true; // gap or duty cap
    }

    if (stuckLatched_ || wallBlocked || !in.sensorHealthy) out.refused = out.refused || demand;

    // ---- output fuse: no logic path gets to hold the valve open forever ----
    // This runs last and overrides everything above: it is the belt for the
    // case where the pin is HIGH because something broke, not because a pulse
    // was asked for. Latching it like a stuck-open valve is deliberate — from
    // the diver's side the consequence is identical.
    if (out.solenoid) {
        if (!onRun_) {
            onRun_ = true;
            onRunStartMs_ = in.nowMs;
        } else if (in.nowMs - onRunStartMs_ > cfg_.maxOnMs) {
            if (pulseRemMs_ > 0) logOn(pulseStartMs_, in.nowMs);
            pulseRemMs_ = 0;
            lastPulseEndMs_ = in.nowMs;
            riseBaseValid_ = false;
            onRun_ = false;
            fuseTripped_ = true;
            stuckLatched_ = true;
            out.solenoid = false;
            out.outputFuse = true;
            out.stuckOpenLatched = true;
        }
    } else {
        onRun_ = false;
    }

    return out;
}

} // namespace inj
