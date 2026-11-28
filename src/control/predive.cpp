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
    railWaitStart_ = 0;
    railWaiting_ = false;
    railSettleExpired_ = false;
    railUnwired_ = false;
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
            // AUDIT (grace window): vbat, vbatMid and vrail; no depth anywhere.
            // The pack side judges on a single cycle — a pack below vbatMin is a
            // fault to refuse now, and `start()` restarts the sequence in seconds,
            // so there is nothing for a grace to protect there. The rail side does
            // carry a wait, and why it has to is written at cfg_.vrailSettleMs.
            //
            // Four questions, and the second and fourth are the ones that used not
            // to exist. Pack above the floor AND below the plausibility ceiling: a
            // reading this high is a sense node on its rail, not a charged pack,
            // and "we cannot read the pack" must never reach a PASS. Then, for a
            // series pack only, the two halves must agree — one total voltage can
            // hide one dead cell, and this is the step where that would otherwise
            // be waved through. Then the rail the valve actually runs on, which is
            // the question whose absence made a dead boost stage invisible
            // (docs/REVIEW.md 1 P9).
            if (in.vbat < cfg_.vbatMin) { fail(S_BATTERY); break; }
            if (in.vbat > cfg_.vbatMax) { fail(S_BATTERY); break; }
            if (cfg_.vbatCells >= 2) {
                // Unusable midpoint: 0 is what an unwired or shorted sense node
                // reads, and on a series pack that is the exact condition the
                // check exists to be allowed to fail. It is not evidence of
                // matched cells.
                if (in.vbatMid <= 0.0f || in.vbatMid >= in.vbat) { fail(S_BATTERY); break; }
                const float upper = in.vbat - in.vbatMid;
                const float spread = std::fabs(in.vbatMid - upper);
                if (spread > cfg_.vbatSpreadMaxV) { fail(S_BATTERY); break; }
            }
            if (cfg_.vrailSensed) {
                // Never sensed. Deliberately not merged into the band test: with
                // an input-only pin a floating reader can land inside a band by
                // accident, and "0 V on a 12 V rail" is a different fault with a
                // different repair. No patience here either — a rail that has
                // never produced a reading is not going to produce one while the
                // sequence watches.
                if (in.vrail <= 0.0f) {
                    railUnwired_ = true;
                    railWaiting_ = false;
                    fail(S_BATTERY);
                    break;
                }
                if (in.vrail >= cfg_.vrailMin && in.vrail <= cfg_.vrailMax) {
                    railWaiting_ = false;
                    advance(S_BATTERY);
                    break;
                }
                // Out of band, but sensed: wait the settle out, saying so, and
                // fail at the deadline. The two reasons this can be transient are
                // the pulse the sweep landed on and the boost recovering from the
                // solenoid step's own pulses; both settle in far less than the
                // deadline, and a supply that is simply gone never settles at all.
                if (!railWaiting_) { railWaiting_ = true; railWaitStart_ = nowMs; }
                if (nowMs - railWaitStart_ >= cfg_.vrailSettleMs) {
                    railWaiting_ = false;
                    railSettleExpired_ = true;
                    fail(S_BATTERY);
                }
                break;
            }
            advance(S_BATTERY);
            break;

        default: break;
    }
}

} // namespace predive
