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
// ---------------------------------------------------------------------------
// CAL AIR: the disciplined averaging window (see header for the rules).
// ---------------------------------------------------------------------------
namespace {
// Least-squares line through (sample index, raw) for the current window.
// Cells are not solid constants — gentle drift and a bit of hysteresis are
// what they do — so stability is measured against the cell's OWN trend, not
// against a flat line. A momentary excursion (the cell wanders across a
// neighbour's level and back) is normal behaviour, not a settling failure:
// samples beyond `band` are COUNTED (outliers) rather than maxed, so one or
// two of them in a window do not restart it. What still fails: enough
// outliers to show sustained trouble, or a trend moving so fast it is a
// warm-up ramp rather than wander — averaging either would commit a lie as
// the air point. `maxResid` stays for the console/bench diagnostic only.
struct Trend { float slopePerSample; float maxResid; int outliers; };

Trend fitTrend(const float* ys, int n, float band) {
    float sx = 0, sy = 0, sxx = 0, sxy = 0;
    for (int k = 0; k < n; k++) {
        const float x = static_cast<float>(k);
        sx += x; sy += ys[k]; sxx += x * x; sxy += x * ys[k];
    }
    const float nf = static_cast<float>(n);
    const float den = nf * sxx - sx * sx;
    const Trend zero{0.0f, 0.0f, 0};
    if (den == 0.0f) return zero;
    Trend t;
    t.slopePerSample = (nf * sxy - sx * sy) / den;
    const float a = (sy - t.slopePerSample * sx) / nf;
    t.maxResid = 0.0f;
    t.outliers = 0;
    for (int k = 0; k < n; k++) {
        const float r = std::fabs(ys[k] - (a + t.slopePerSample * static_cast<float>(k)));
        if (r > t.maxResid) t.maxResid = r;
        if (r > band) t.outliers++;
    }
    return t;
}
} // namespace

void AirCal::resetWindow(uint32_t nowMs) {
    n_ = 0;
    for (int i = 0; i < 3; i++) sum_[i] = lo_[i] = hi_[i] = 0.0f;
    nextSampleMs_ = nowMs;  // the next update() may sample immediately
}

void AirCal::start(uint32_t nowMs) {
    st_ = St::RUN;
    deadlineMs_ = nowMs + cfg_.timeoutMs;
    resetWindow(nowMs);
}

AirCalOutput AirCal::update(const AirCalInput& in) {
    AirCalOutput o;
    if (st_ != St::RUN) return o;
    o.running = true;

    if (in.diving) {
        st_ = St::IDLE;
        o.running = false;
        o.failed = true;
        o.why = "diving: CAL AIR is a pre-dive act";
        o.whyCode = AirCalWhy::DIVING;
        return o;
    }
    if (in.nowMs >= deadlineMs_) {
        st_ = St::IDLE;
        o.running = false;
        o.failed = true;
        o.why = "timeout: cells never settled in the spread band";
        o.whyCode = AirCalWhy::UNSETTLED;
        return o;
    }
    if (in.injecting) {
        // A pulse enriches the loop: everything sampled around it is not air.
        resetWindow(in.nowMs);
        o.samples = 0;
        return o;
    }
    if (!in.fresh || in.nowMs < nextSampleMs_) {
        o.samples = n_;
        return o;
    }
    nextSampleMs_ = in.nowMs + cfg_.samplePeriodMs;

    if (n_ == 0) {
        for (int i = 0; i < 3; i++) {
            sum_[i] = lo_[i] = hi_[i] = in.cellsRaw[i];
            hist_[i][0] = in.cellsRaw[i];
        }
    } else {
        for (int i = 0; i < 3; i++) {
            sum_[i] += in.cellsRaw[i];
            if (in.cellsRaw[i] < lo_[i]) lo_[i] = in.cellsRaw[i];
            if (in.cellsRaw[i] > hi_[i]) hi_[i] = in.cellsRaw[i];
            hist_[i][n_] = in.cellsRaw[i];
        }
    }
    n_++;

    o.samples = n_;
    uint8_t need = cfg_.samples;  // clamped to the window buffer
    if (need == 0 || need > kMaxSamples) need = kMaxSamples;
    float worst = 0.0f;
    bool any = false, unstable = false;
    // The trend slope is per sample; the patience is per second.
    const float maxSlope = cfg_.driftMaxPerS * static_cast<float>(cfg_.samplePeriodMs) / 1000.0f;
    for (int i = 0; i < 3; i++) {
        if (!(in.presentMask & (1u << i))) continue;  // absent cell: its raw is someone else's
        any = true;
        const float spread = hi_[i] - lo_[i];
        if (spread > worst) worst = spread;
        if (n_ >= 3) {
            const Trend t = fitTrend(hist_[i], n_, cfg_.spreadMax);
            // The band rides the trend (that is the hysteresis allowance), and
            // momentary excursions are counted, not fatal: the bench shows
            // cells crossing a neighbour's level for one sample and coming
            // back. A window with more than outlierMax such samples is not
            // wandering — it is unsettled. This verdict is live: it restarts
            // the window as soon as it is certain, without waiting for 12.
            if (t.outliers > cfg_.outlierMax) unstable = true;
            // A trend moving faster than this is warm-up drift, not wander:
            // its window centre would not be the air point. Judged on the
            // FULL window only — a lone excursion pulls a short fit's slope
            // and would punish a cell for the very wander we allow.
            else if (n_ >= need && std::fabs(t.slopePerSample) > maxSlope) unstable = true;
        }
    }
    if (!any) {  // mask empty: programming error, refuse rather than average nothing
        st_ = St::IDLE;
        o.running = false;
        o.failed = true;
        o.why = "no cells present";
        o.whyCode = AirCalWhy::NOCELLS;
        return o;
    }
    if (unstable) {  // give the cell a chance to settle; count restarts
        resetWindow(in.nowMs);
        o.samples = 0;
        return o;
    }

    if (n_ >= need) {
        st_ = St::IDLE;
        o.running = false;
        o.done = true;
        o.spreadWorst = worst;
        for (int i = 0; i < 3; i++) o.rawAvg[i] = sum_[i] / static_cast<float>(n_);
    }
    return o;
}

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
