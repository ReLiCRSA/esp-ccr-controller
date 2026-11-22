#include "predive.h"

#include <cmath>

namespace predive {

void Sequencer::reset() {
    for (int i = 0; i < S_COUNT; ++i) st_[i] = PENDING;
    current_ = S_WARMUP;
    state_ = 0;
    airStable_ = 0;
    solReported_ = false;
}

void Sequencer::start(uint32_t nowMs) {
    reset();
    state_ = 1;
    warmStart_ = nowMs;
    st_[S_WARMUP] = RUNNING;
}

void Sequencer::advance(Step done) {
    st_[done] = PASS;
    if (done + 1 < S_COUNT) {
        current_ = static_cast<Step>(done + 1);
        st_[current_] = RUNNING;
    } else {
        state_ = 2; // all passed
    }
}

void Sequencer::fail(Step s) {
    st_[s] = FAIL;
    state_ = 3;
}

void Sequencer::reportSolTest(bool riseOk, uint32_t) {
    if (state_ != 1 || current_ != S_SOLENOID) return; // ignore out of turn
    solReported_ = true;
    solOk_ = riseOk;
}

void Sequencer::update(const Inputs& in, uint32_t nowMs) {
    if (state_ != 1) return;

    switch (current_) {
        case S_WARMUP:
            if (nowMs - warmStart_ >= cfg_.warmupMs) advance(S_WARMUP);
            break;

        case S_AIR: {
            bool ok = true;
            int present = 0;
            const float tol = cfg_.airTargetBar * cfg_.airTolPct / 100.0f;
            for (int i = 0; i < 3; ++i) {
                if (!(in.presentMask & (1u << i))) continue;
                present++;
                if (std::fabs(in.cells[i] - cfg_.airTargetBar) > tol) ok = false;
            }
            if (present < 2) ok = false; // need at least two cells to trust air
            if (ok) {
                if (++airStable_ >= cfg_.airStableSamples) advance(S_AIR);
            } else {
                airStable_ = 0;
            }
            // air check does not instant-fail; it waits until timeout-ish is
            // decided by the operator/handset (cells may still be warming).
            break;
        }

        case S_SOLENOID:
            if (solReported_) {
                if (solOk_) advance(S_SOLENOID);
                else fail(S_SOLENOID);
                solReported_ = false;
            }
            break;

        case S_DEPTH:
            if (std::fabs(in.depthM) <= cfg_.depthMaxM) advance(S_DEPTH);
            break;

        case S_BATTERY:
            if (in.vbat >= cfg_.vbatMin) { advance(S_BATTERY); }
            else fail(S_BATTERY);
            break;

        default: break;
    }
}

} // namespace predive
