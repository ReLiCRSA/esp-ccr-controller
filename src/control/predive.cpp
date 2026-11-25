#include "predive.h"

#include <cmath>

namespace predive {

void Sequencer::reset() {
    for (int i = 0; i < S_COUNT; ++i) st_[i] = PENDING;
    current_ = S_WARMUP;
    state_ = 0;
    airStable_ = 0;
    solReported_ = false;
    depthSilentSinceMs_ = 0;
    depthSilent_ = false;
    depthGraceExpired_ = false;
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
        case S_WARMUP:  // AUDIT: pure elapsed time, no sensor input at all.
            if (nowMs - warmStart_ >= cfg_.warmupMs) advance(S_WARMUP);
            break;

        case S_AIR: {
            // AUDIT (grace window): S_AIR consumes cells + presentMask only — no
            // depth, no pressure — so a bad depth cycle cannot condemn it. It also
            // has no failure path at all: it waits for the cells to settle, which
            // is the opposite hazard (patience owned by the operator/handset, not a
            // single-cycle verdict). Deliberately unchanged by this fix: the one
            // place this step does react to a single bad cycle is airStable_ going
            // back to zero, which costs patience, never a verdict.
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
            // AUDIT (grace window): this step's verdict is REPORTED to it by
            // cal::PulseTest, whose input is cells, votedBar, presentMask and
            // pulsesDelivered — nothing depth-derived — so a bad depth cycle cannot
            // condemn it either. It cannot hang either: PulseTest's own
            // pulseTimeoutMs/recoverTimeoutMs bound the verdict.
            if (solReported_) {
                if (solOk_) advance(S_SOLENOID);
                else fail(S_SOLENOID);
                solReported_ = false;
            }
            break;

        case S_DEPTH:
            // Two questions, in this order: is there a depth at all, and is it
            // the surface.
            //
            // An unmeasured depth is never believed — that was finding C1 /
            // review H3, where the 0.0 m fabricated for a dead bus was exactly
            // the number this step wants, so a rig certified itself "on the
            // surface" by failing its own depth sensor. And since the grace
            // window it is no longer CONDEMNED on the single cycle it happens to
            // land on either: the step carries the silence and waits for
            // cfg_.depthGraceMs.
            //
            // Waiting is not hanging. A bus that never answers still ends in
            // FAIL, so the pre-dive always returns a verdict and the surface log
            // always carries which step stopped the dive. The bound is the same
            // patience the firmware already grants a depth sample on a dive
            // (PRESSURE_STALE_MS, main.cpp), so a shared-Wire1 hiccup or a
            // driver's first backoff steps fit inside it. ms5837's backoff tops
            // out at 4 s, past this bound: a driver that deep does lose the step,
            // and that trade is written down at PREDIVE_DEPTH_GRACE_MS rather
            // than hidden here.
            if (!in.depthValid) {
                // The grace clock starts at the first silent cycle the STEP sees,
                // not at the moment the bus died: the steps before this one have
                // their own patience and can take minutes.
                if (!depthSilent_) {
                    depthSilent_ = true;
                    depthSilentSinceMs_ = nowMs;
                }
                if (nowMs - depthSilentSinceMs_ >= cfg_.depthGraceMs) {
                    depthGraceExpired_ = true;
                    fail(S_DEPTH);
                }
                // Off the surface, off-scale or NaN while the bus is silent is all
                // equally irrelevant: the field is not a measurement, so it gets
                // neither verdict.
                break;
            }
            // A measurement landed. The silence is over — and it is over even on
            // the cycle where the grace had just run out, which is the off-by-one
            // side pinned here on purpose: the grace FAILs on a cycle whose bus is
            // STILL silent at or past depthGraceMs of continuous silence, so the
            // verdict is always a statement about the bus, never about a sample
            // that actually arrived. A dead bus therefore can never interpose its
            // own reprieve (it delivers no measurement to land on the boundary),
            // and a live one that was merely late is judged on what it measured.
            depthSilent_ = false;
            // NaN and out-of-band values are excluded by the comparison itself,
            // so no extra arithmetic is needed to distrust a wild sample: a wild
            // or off-scale reading leaves the step RUNNING, waiting the way a rig
            // that is genuinely off the surface always does.
            if (std::fabs(in.depthM) <= cfg_.depthMaxM) advance(S_DEPTH);
            break;

        case S_BATTERY:
            // AUDIT (grace window): vbat only, no depth anywhere. It does judge on
            // a single cycle — but a pack below vbatMin is a fault to refuse now,
            // and `start()` restarts the sequence in seconds, so there is nothing
            // for a grace to protect here.
            if (in.vbat >= cfg_.vbatMin) { advance(S_BATTERY); }
            else fail(S_BATTERY);
            break;

        default: break;
    }
}

} // namespace predive
