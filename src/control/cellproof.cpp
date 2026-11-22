// ============================================================================
// cellproof.cpp — see cellproof.h.
//
// Two rules carry the safety weight here:
//   * the verdict is per cell, and every FITTED cell is judged — including one
//     the voting layer has already dropped, because that is the cell whose
//     honesty settles a 2-vs-1 argument;
//   * a PASS can re-arm. Cells that were honest on first descent can fail
//     later in the dive, and the machine's own evidence that this happened is
//     a cell it rejected reading well above the vote.
// ============================================================================
#include "cellproof.h"

#include <algorithm>

namespace cellproof {

void Engine::beginRun(const Input& in, uint32_t nowMs) {
    state_ = State::RUNNING;
    startMs_ = nowMs;
    lastRunMs_ = nowMs;
    attempts_++;
    sawMax = false;
    maxSeen_ = in.votedBar;
    suspectSinceMs_ = 0;
    for (int i = 0; i < 3; ++i) {
        best_[i] = in.cellsBar[i];
        floor_[i] = in.cellsBar[i];
    }
}

Output Engine::update(const Input& in) {
    Output out;

    // dive lifecycle edges
    if (in.diveActive && !diveActive_) diveStarted();
    if (!in.diveActive && diveActive_) { diveActive_ = false; diveEnded(); if (state_ != State::INACTIVE) state_ = State::IDLE; }
    diveActive_ = in.diveActive;

    switch (state_) {
        case State::INACTIVE:
            break;
        case State::IDLE:
            if (in.diveActive && in.depthM < cfg_.depthM - 1.0f) state_ = State::ARMED;
            break;
        case State::ARMED:
            if (in.diveActive && in.depthM >= cfg_.depthM) {
                beginRun(in, in.nowMs);
                out.event = EV_STARTED;
                out.requestSpOverride = true; // SP must rise on this very cycle
            }
            break;
        case State::RUNNING: {
            out.requestSpOverride = true;
            if (in.nowMs - startMs_ <= 100) maxSeen_ = in.votedBar;
            else if (in.votedBar > maxSeen_) maxSeen_ = in.votedBar;
            // Track every fitted cell, not just the ones currently voting: the
            // excluded cell is exactly the one whose rise proves it honest.
            for (int i = 0; i < 3; ++i)
                if (in.presentMask & (1u << i)) {
                    best_[i] = best_[i] < in.cellsBar[i] ? in.cellsBar[i] : best_[i];
                    floor_[i] = floor_[i] > in.cellsBar[i] ? in.cellsBar[i] : floor_[i];
                }

            const bool timedOut = in.nowMs - startMs_ >= cfg_.timeoutMs;
            if (in.votedBar >= cfg_.minPassBar || timedOut) {
                uint8_t proven = 0, unproven = 0;
                for (int i = 0; i < 3; ++i) {
                    if (!(in.presentMask & (1u << i))) continue;
                    // Proven = reached the proof level AND moved with the loop.
                    // The level alone would let a cell frozen at that number
                    // vouch for itself.
                    const bool tracked = (best_[i] - floor_[i]) >= cfg_.trackMinBar;
                    if (tracked && best_[i] >= cfg_.minPassBar) proven |= (1u << i);
                    else unproven |= (1u << i);
                }
                if (proven) {
                    // At least one cell can carry a high setpoint. The rest are
                    // marked, and the control layer decides when to stop
                    // trusting them — the dive continues on the proven cells.
                    unproven_ = unproven;
                    state_ = State::PASS;
                    out.event = EV_PASS;
                } else if (timedOut) {
                    // Nothing reached the proof level: the loop cannot be run
                    // at a high setpoint on these cells, whatever the reason.
                    state_ = State::FAIL;
                    out.event = EV_FAIL;
                    out.clampActive = true; // clamp applies on the fail cycle itself
                }
            }
            break;
        }
        case State::PASS: {
            // Is this proof still describing reality? A cell the voting layer
            // has REJECTED and that disagrees with the vote by a wide margin is
            // the machine telling itself its cell picture is stale. Both
            // directions matter: the rejected cell reading ABOVE the vote means
            // the loop may be richer than shown (toxicity), reading BELOW means
            // the loop may be leaner than shown — and a vote that sits above the
            // setpoint means the valve never opens again, which starves the loop
            // (found by the 50-dive soak at 28 m, vote 0.82, real 0.16).
            float rejLo = 99.0f, rejHi = -99.0f;
            for (int i = 0; i < 3; ++i)
                if ((in.presentMask & (1u << i)) && !(in.activeMask & (1u << i))) {
                    rejLo = std::min(rejLo, in.cellsBar[i]);
                    rejHi = std::max(rejHi, in.cellsBar[i]);
                }
            const bool rejectedDisagree =
                rejHi > 0.0f &&
                (rejHi > in.votedBar + cfg_.suspectGapBar || rejLo < in.votedBar - cfg_.suspectGapBar);
            const bool suspect = in.diveActive && in.depthM >= cfg_.depthM &&
                                 attempts_ < cfg_.maxAttempts && rejectedDisagree;
            if (!suspect) {
                suspectSinceMs_ = 0;
                break;
            }
            if (suspectSinceMs_ == 0) { suspectSinceMs_ = in.nowMs; break; }
            if (in.nowMs - suspectSinceMs_ < cfg_.suspectMs) break;
            if (in.nowMs - lastRunMs_ < cfg_.rearmCooldownMs) break;
            beginRun(in, in.nowMs);
            out.event = EV_STARTED;
            out.requestSpOverride = true;
            break;
        }
        case State::FAIL:
            // No self re-arm: the clamp is the safe state and lifting it on a
            // second flaky result is the diver's call (override()).
            out.clampActive = true;
            break;
    }

    out.state = state_;
    out.unprovenHighMask = unproven_;
    return out;
}

} // namespace cellproof
