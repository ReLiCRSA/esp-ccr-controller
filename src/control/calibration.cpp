#include "calibration.h"

#include <cmath>

namespace cal {

// ---------------------------------------------------------------------------
// Two-point solve. Reference slope for the sanity band: previous valid cal,
// else the caller's default (nominal cell spec). Rejected cells KEEP their
// previous calibration — never overwrite good data with a suspect point.
// ---------------------------------------------------------------------------
void Solver::solveO2(const Config& cfg,
                     const float rawO2[3],
                     const Cal prev[3], float defaultSlope,
                     Cal out[3], O2Result& res) {
    for (int i = 0; i < 3; ++i) {
        out[i] = prev[i];
        res.verdict[i] = CV_OK;
        if (!prev[i].hasAir) { res.verdict[i] = CV_NO_AIR; continue; }

        const float refSlope = prev[i].valid() ? prev[i].slope : defaultSlope;
        const float dRaw = rawO2[i] - prev[i].rawAir;

        // Dead/poisoned cell: barely moved toward O2.
        const float expectRise = (cfg.o2Fi - cfg.airRefBar) / refSlope;
        if (dRaw < cfg.minRiseFrac * expectRise) {
            res.verdict[i] = CV_NO_RISE;
            continue;
        }

        const float slope = (cfg.o2Fi - prev[i].airBar) / dRaw;
        const float band = refSlope * cfg.slopeBandPct / 100.0f;
        if (std::fabs(slope - refSlope) > band) {
            res.verdict[i] = CV_SLOPE_OUT;
            continue;
        }

        out[i].slope = slope;
        out[i].airBar = prev[i].airBar;
        out[i].rawO2 = rawO2[i];
        out[i].hasO2 = true;
    }
}

// ---------------------------------------------------------------------------
// CAL O2 enrichment sequencer
// ---------------------------------------------------------------------------
bool O2Sequencer::arm() {
    if (!airReady_ || state_ == O2State::ENRICHING || state_ == O2State::SETTLING)
        return false;
    armed_ = true;
    return true;
}

bool O2Sequencer::confirm(uint32_t nowMs) {
    if (!armed_) return false;
    armed_ = false;
    state_ = O2State::ENRICHING;
    runStart_ = nowMs;
    for (int i = 0; i < 3; ++i) rawSum_[i] = 0.0f;
    stable_ = 0;
    return true;
}

O2Output O2Sequencer::update(const O2Input& in) {
    O2Output out;
    out.state = state_;

    if (state_ != O2State::ENRICHING && state_ != O2State::SETTLING) return out;

    // Safety interlocks first: bench-only, and an absolute ceiling.
    if (in.diveActive) {
        state_ = O2State::FAILED;
        out.result.aborted = true;
        out.result.abortReason = "dive active";
        out.state = state_;
        return out;
    }
    if (in.votedBar > cfg_.o2CeilingBar) {
        state_ = O2State::FAILED;
        out.result.aborted = true;
        out.result.abortReason = "ppO2 ceiling";
        out.state = state_;
        return out;
    }
    if (in.nowMs - runStart_ >= cfg_.enrichTimeoutMs) {
        state_ = O2State::FAILED;
        out.result.aborted = true;
        out.result.abortReason = "timeout";
        out.state = state_;
        return out;
    }

    if (state_ == O2State::ENRICHING) {
        out.requestEnrich = true;
        if (in.votedBar >= cfg_.o2TargetBar) {
            state_ = O2State::SETTLING;
            settleStart_ = in.nowMs;
            for (int i = 0; i < 3; ++i) rawSum_[i] = 0.0f;
            stable_ = 0;
        }
    } else { // SETTLING
        out.requestEnrich = true; // hold enrichment until sampled
        if (in.votedBar < cfg_.o2TargetBar - 0.03f) {
            state_ = O2State::ENRICHING; // drifted back down: keep pushing
        } else {
            for (int i = 0; i < 3; ++i) rawSum_[i] += in.cellsRaw[i];
            stable_++;
            if (stable_ >= cfg_.stableSamples &&
                in.nowMs - settleStart_ >= cfg_.settleMs) {
                float rawAvg[3];
                for (int i = 0; i < 3; ++i) rawAvg[i] = rawSum_[i] / stable_;

                O2Result res;
                Cal updated[3];
                // nominal slope from cell 0's air point or defaults: caller
                // supplies prev via setAirCal; default slope ~ air spec.
                float defSlope = 0.21f / 52.0f; // R22D spec ballpark, per mV
                for (int i = 0; i < 3; ++i)
                    if (base_[i].valid()) { defSlope = base_[i].slope; break; }
                Solver::solveO2(cfg_, rawAvg, base_, defSlope, updated, res);
                for (int i = 0; i < 3; ++i) base_[i] = updated[i];

                state_ = O2State::DONE;
                out.result = res;
            }
        }
    }

    out.state = state_;
    if (state_ == O2State::DONE) out.result.aborted = false;
    if (state_ == O2State::DONE || state_ == O2State::FAILED)
        out.started = true; // one-shot completion marker
    return out;
}

// ---------------------------------------------------------------------------
// Pulse responsiveness test
// ---------------------------------------------------------------------------
PulseTestOutput PulseTest::update(const PulseTestInput& in) {
    PulseTestOutput out;
    out.state = state_;
    if (state_ == TestState::IDLE || state_ == TestState::DONE) return out;

    switch (state_) {
        case TestState::BASELINE:
            for (int i = 0; i < 3; ++i) base_[i] += in.cellsBar[i];
            if (++n_ >= cfg_.baselineSamples) {
                for (int i = 0; i < 3; ++i) { base_[i] /= n_; peak_[i] = base_[i]; }
                n_ = 0;
                state_ = TestState::PULSING;
                startPulseMs_ = in.nowMs;
            }
            break;

        case TestState::PULSING: {
            for (int i = 0; i < 3; ++i)
                if (in.cellsBar[i] > peak_[i]) peak_[i] = in.cellsBar[i];
            if (pulsesSent_ < cfg_.pulses) {
                out.requestPulse = true;
                pulsesSent_++;
            }
            // verdicts once peaks have had time to develop — or when the
            // pulse phase times out (a duty-capped injector may deliver
            // fewer than requested pulses; judge what actually happened)
            const bool deliveredAll = in.pulsesDelivered >= pulsesSent_;
            const bool timedOut = in.nowMs - startPulseMs_ >= cfg_.pulseTimeoutMs;
            if ((deliveredAll && in.nowMs - startPulseMs_ > 8000) || timedOut) {
                for (int i = 0; i < 3; ++i) {
                    if (!(in.presentMask & (1u << i))) continue;
                    if (peak_[i] - base_[i] >= cfg_.riseMinBar) passMask_ |= (1u << i);
                    else failMask_ |= (1u << i);
                }
                state_ = TestState::RECOVER;
                recStart_ = in.nowMs;
            }
            break;
        }

        case TestState::RECOVER: {
            bool allBack = true;
            for (int i = 0; i < 3; ++i)
                if ((in.presentMask & (1u << i)) &&
                    in.cellsBar[i] > base_[i] + cfg_.recoverTolBar) allBack = false;
            if (allBack || in.nowMs - recStart_ >= cfg_.recoverTimeoutMs) {
                state_ = TestState::DONE;
                out.done = true;
            }
            break;
        }
        default: break;
    }

    out.state = state_;
    out.passMask = passMask_;
    out.failMask = failMask_;
    return out;
}

} // namespace cal
