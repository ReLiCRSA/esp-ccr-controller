#include "cellproof.h"

namespace cellproof {

Output Engine::update(const Input& in) {
    Output out;

    // dive lifecycle edges
    if (in.diveActive && !diveActive_) diveStarted();
    if (!in.diveActive && diveActive_) { diveActive_ = false; diveEnded(); if (state_ != State::DISABLED) state_ = State::IDLE; }
    diveActive_ = in.diveActive;

    switch (state_) {
        case State::DISABLED:
            break;
        case State::IDLE:
            if (in.diveActive && in.depthM < cfg_.depthM - 1.0f) state_ = State::ARMED;
            break;
        case State::ARMED:
            if (in.diveActive && in.depthM >= cfg_.depthM) {
                state_ = State::RUNNING;
                startMs_ = in.nowMs;
                sawMax = false;
                for (int i = 0; i < 3; ++i) best_[i] = in.cellsBar[i];
                out.event = EV_STARTED;
                out.requestSpOverride = true; // SP must rise on this very cycle
            }
            break;
        case State::RUNNING: {
            out.requestSpOverride = true;
            if (in.nowMs - startMs_ <= 100) maxSeen_ = in.votedBar;
            else if (in.votedBar > maxSeen_) maxSeen_ = in.votedBar;
            for (int i = 0; i < 3; ++i)
                if (in.activeMask & (1u << i)) best_[i] = best_[i] < in.cellsBar[i] ? in.cellsBar[i] : best_[i];

            if (in.votedBar >= cfg_.minPassBar) {
                uint8_t unproven = 0;
                for (int i = 0; i < 3; ++i)
                    if (in.activeMask & (1u << i)) {
                        // cells excluded during the run are also unproven
                        if (best_[i] < cfg_.minPassBar) unproven |= (1u << i);
                    } else {
                        unproven |= (1u << i);
                    }
                unproven_ = unproven;
                state_ = State::PASS;
                out.event = EV_PASS;
            } else if (in.nowMs - startMs_ >= cfg_.timeoutMs) {
                state_ = State::FAIL;
                out.event = EV_FAIL;
                out.clampActive = true; // clamp applies on the fail cycle itself
            }
            break;
        }
        case State::PASS:
            // one attempt per dive; results persist (clamp off, no override)
            break;
        case State::FAIL:
            out.clampActive = true;
            break;
    }

    out.state = state_;
    out.unprovenHighMask = unproven_;
    return out;
}

} // namespace cellproof
