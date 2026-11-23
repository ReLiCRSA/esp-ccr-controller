#include "depth_manager.h"

namespace press {

bool DepthArbiter::begin(uint32_t nowMs) {
    // Locked is forever: the locked source may fault and be re-initialized
    // by its own driver, but the SLOT is never re-awarded mid-power-cycle.
    if (locked_) return chosen_->begin(nowMs);

    if (phase_ == Phase::WAIT) {
        if (nowMs < nextPhaseMs_) return false;
        phase_ = Phase::PREFERRED;
        phaseStartMs_ = nowMs;
        return false;
    }

    DepthSensor& cand = (phase_ == Phase::PREFERRED) ? preferred_ : fallback_;
    if (cand.begin(nowMs)) {
        chosen_ = &cand;
        locked_ = true;
        return true;
    }

    const uint32_t window =
        (phase_ == Phase::PREFERRED) ? PREFERRED_WINDOW_MS : FALLBACK_WINDOW_MS;
    if (nowMs - phaseStartMs_ >= window) {
        phase_ = (phase_ == Phase::PREFERRED) ? Phase::FALLBACK : Phase::WAIT;
        phaseStartMs_ = nowMs;
        nextPhaseMs_ = nowMs + RECIRCULATE_MS;
    }
    return false;
}

void DepthArbiter::start(uint32_t nowMs) {
    if (locked_) chosen_->start(nowMs);
}

bool DepthArbiter::poll(uint32_t nowMs) {
    return locked_ && chosen_->poll(nowMs);
}

}  // namespace press
