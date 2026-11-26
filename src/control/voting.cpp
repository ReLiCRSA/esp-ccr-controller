#include "voting.h"

#include <algorithm>
#include <cmath>

namespace voting {

namespace {

float medianF(const float* v, int n) {
    float t[3];
    for (int i = 0; i < n; ++i) t[i] = v[i];
    std::sort(t, t + n);
    return t[n / 2]; // n==3 -> middle
}

} // namespace

void Engine::setPresentMask(uint8_t mask) {
    present_ = mask & 0x7;
    for (uint8_t i = 0; i < NCELL; ++i) {
        if (!(present_ & (1u << i))) {
            st_[i] = CellStatus{};
            // Its history goes with it, including "this cell once answered":
            // Result::cellsSeen means "any FITTED cell has data", and a cell
            // the bench uninstalled must not keep that claim alive for a vote
            // it no longer takes part in. The detector state goes with it too:
            // the next cell fitted in that socket starts with a clean record,
            // not the frozen window of the cell that used to live there.
            seen_[i] = false;
            bar_[i] = BAR_NONE;
            implausibleStrikes_[i] = 0;
            stuckLo_[i] = stuckHi_[i] = 0.0f;
            stuckN_[i] = 0;
            stuckPulse_[i] = pulsesSeen_;
            riseMin_[i] = riseMax_[i] = 0.0f;
            riseN_[i] = 0;
            riseMs_[i] = 0;
            risePulse_[i] = pulsesSeen_;
        }
    }
}

bool Engine::plausible(uint8_t ch, float v) const {
    if (v < cfg_.minPpo2 || v > cfg_.maxPpo2) return false;
    if (seen_[ch] && std::fabs(v - st_[ch].last) > cfg_.glitchMaxJump) return false;
    return true;
}

void Engine::setResponsiveness(uint8_t ch, float r) {
    st_[ch].responsiveness = std::max(0.0f, std::min(1.0f, r));
}

Result Engine::update(const float ppo2[NCELL], uint32_t dtMs) {
    Result res;
    const float dtS = std::max(1u, dtMs) / 1000.0f;

    // ---- gate + dropout discrimination -----------------------------------
    float slope[NCELL];
    bool collapsingAny = false, collapsingAll = true;
    int activeCandidates = 0;
    for (uint8_t i = 0; i < NCELL; ++i) {
        slope[i] = 0.0f;
        if (!(present_ & (1u << i))) continue;

        if (!plausible(i, ppo2[i])) {
            st_[i].glitchCount++;
            // The terminal state the skip never had: one implausible sample is a
            // glitch (the previous value stands — correct), but a channel that has
            // been reading nonsense for implausibleWindow consecutive cycles is a
            // FAULT, and until 2026-11-26 it kept its seat in the vote on whatever
            // value was believed when the nonsense started. cellsSeen stayed true,
            // the vote ran on a fossil, and glitchCount only ever fed a display
            // score. Exclusion reuses the drift machinery (driftStrikes at the
            // cap, hysteresis decides when it may sit near the vote again); the
            // bar reason additionally walls its stale value off from the
            // last-resort fallback tier.
            if (implausibleStrikes_[i] < cfg_.implausibleWindow) implausibleStrikes_[i]++;
            if (implausibleStrikes_[i] >= cfg_.implausibleWindow) {
                if (bar_[i] != BAR_IMPLAUSIBLE) {
                    bar_[i] = BAR_IMPLAUSIBLE;
                    st_[i].active = false;
                    res.disagreement = true;
                    // Not dropoutEvent: that word means "collapsed vertically" to
                    // the alarm layer. A cell that stopped being believed is
                    // reported as disagreement, like a drift-out.
                }
                if (st_[i].driftStrikes < cfg_.driftWindow)
                    st_[i].driftStrikes = cfg_.driftWindow;  // same persistence
                                                             // as a dropout
            }
            continue; // keep previous value for this cycle
        }
        implausibleStrikes_[i] = 0;  // plausible again: the streak is broken
        slope[i] = (ppo2[i] - st_[i].last) / dtS;
        st_[i].last = ppo2[i];
        seen_[i] = true; // excluded cells still track their reading: that is
                         // how hysteresis measures them for re-admission

        if (!st_[i].active) continue; // already-excluded cells can't vote on
                                      // dropout/hypoxia; they can only return
        activeCandidates++;
        const bool collapsing = slope[i] < -cfg_.dropoutRateBarS;
        if (collapsing) collapsingAny = true;
        else collapsingAll = false;
    }

    // Does the engine have any believable cell data at all yet — including this
    // call's? Reported rather than inferred by the caller: the difference
    // between "never sampled" and "stopped being believed" is a fact this layer
    // owns, and it is the difference between a boot-time alarm and a real one.
    res.cellsSeen = seen_[0] || seen_[1] || seen_[2];

    if (collapsingAny && collapsingAll && activeCandidates >= 1) {
        // ALL still-trusted cells fell together: the loop really dropped
        // (or the last cell died) — treat as hypoxia, keep cells trusted.
        res.hypoxiaSuspect = true;
    } else if (collapsingAny) {
        for (uint8_t i = 0; i < NCELL; ++i) {
            if ((present_ & (1u << i)) && slope[i] < -cfg_.dropoutRateBarS && st_[i].active) {
                st_[i].active = false; // vertical collapse = sensor fault
                st_[i].driftStrikes = cfg_.driftWindow; // persistent exclusion;
                                                        // recovers only via
                                                        // hysteresis window
                res.dropoutEvent = true;
            }
        }
    }

    // ---- (re)activate the base set on first plausible samples --------------
    for (uint8_t i = 0; i < NCELL; ++i) {
        if ((present_ & (1u << i)) && seen_[i] && st_[i].driftStrikes == 0 &&
            bar_[i] == BAR_NONE && !(proofMask_ & (1u << i)))
            st_[i].active = true;
    }

    // ---- frozen-reading (stuck cell) detector ------------------------------
    // The fault shape the level gate, the slope gate and the drift gate all miss:
    // a channel delivering a PLAUSIBLE number that never changes — a wet connector
    // freezing two channels, a shorted input, an ADC register that stopped
    // converting. Everything else in this file compares cells against EACH OTHER
    // or against thresholds; nothing here asked whether a channel is still alive
    // as a channel. The answer is noise: a live analog front end crosses more
    // than eps of ADC-count spread within seconds, forever (bench-proven, see
    // Config::stuckEpsBar); a frozen reading never does.
    //
    // The state per cell is a BAND: the [min,max] span of readings since the
    // last excursion (the moment that span first exceeded eps — proof the
    // channel moves). stuckWindow says how many CONSECUTIVE samples may fit
    // inside an eps-wide band before that is called a frozen channel rather
    // than luck. Counted in SAMPLES, because the claim is about evidence
    // points: on the board the 200 control cycles ARE ~102 fresh conversions
    // per channel (the rest are control ticks redelivering a sample), and a
    // sparser fresh stream — a unit test at 225 ms — needs the same 200 points,
    // in more seconds. An aligned close/judge/reopen window was tried first and
    // rejected on the bench of the unit tests: a window that straddles the
    // fault instant carries the pre-fault dither into the verdict and hid a
    // frozen channel for whole extra windows. Here the band resets the moment
    // the channel demonstrably moves, the stillness count restarts clean at the
    // fault, and detection latency is one stuckWindow, never two.
    //
    // Honesty rules this detector obeys:
    //   * epsilon is NOISE-sized, not signal-sized. A quiescent loop is not an
    //     accusation: at steady state a healthy chain still wobbles by counts, so
    //     this condemns a channel that stopped changing, never a loop that stopped
    //     moving. The quiet twin in dive 9003 pins exactly that.
    //   * the band only advances on samples that are fresh (a dead stream is the
    //     sensor layer's SENSOR_STALE claim, not a cell claim) and plausible (a
    //     skipped sample is not evidence of either kind — and a cell barred for
    //     nonsense never accumulates a stuck accusation on top of it).
    //   * a cell already barred for being frozen re-runs the same test every
    //     sample: refuted by the next excursion (its own movement), re-confirmed
    //     by another full band of stillness. No thrash in either direction, and
    //     no second accusation EVENT while the first one still stands.
    if (sampleLive_) {
        for (uint8_t i = 0; i < NCELL; ++i) {
            if (!(present_ & (1u << i)) || !seen_[i] || bar_[i] == BAR_IMPLAUSIBLE) continue;
            const float eps = cfg_.stuckEpsBar;
            if (stuckN_[i] == 0) {  // band start (and after every excursion)
                stuckLo_[i] = stuckHi_[i] = st_[i].last;
                stuckPulse_[i] = pulsesSeen_;
            }
            stuckN_[i]++;
            stuckLo_[i] = std::min(stuckLo_[i], st_[i].last);
            stuckHi_[i] = std::max(stuckHi_[i], st_[i].last);
            if (stuckHi_[i] - stuckLo_[i] > eps) {
                // Excursion: this channel demonstrably moves. If a frozen
                // accusation stood, that accusation is REFUTED by the cell's own
                // evidence — but the vote is not handed back for free: the cell
                // returns when the drift hysteresis agrees its reading sits near
                // the vote again (thaw proves liveness, not trustworthiness).
                if (bar_[i] == BAR_STUCK) bar_[i] = BAR_NONE;
                stuckN_[i] = 1;
                stuckLo_[i] = stuckHi_[i] = st_[i].last;
                stuckPulse_[i] = pulsesSeen_;
            } else if (stuckN_[i] >= cfg_.stuckWindow && bar_[i] != BAR_STUCK) {
                    // Frozen reading. Same exclusion machinery as a drift-out or
                    // a dropout (driftStrikes at the cap; reported through
                    // disagreement and activeMask, no new vocabulary) — but
                    // unlike a drift-out the bar never lifts on hysteresis
                    // alone: only the channel moving again refutes it.
                    bar_[i] = BAR_STUCK;
                    st_[i].active = false;
                    res.disagreement = true;
                    if (st_[i].driftStrikes < cfg_.driftWindow)
                        st_[i].driftStrikes = cfg_.driftWindow;
                }

            // Responsiveness, scored over a rolling window: the engine knows how
            // many pulses the valve fired inside it (Loop reports the count) and
            // how far the cell moved in it. RELATIVE score — cell i against the
            // liveliest believed cell — because the voting layer owns no gas
            // model and cannot know how far a cell SHOULD move for N pulses;
            // pulses 3 s apart against a galvanic tau of ~10 s makes absolute
            // per-cell attribution a lie. Advisory only (health score + handset):
            // the exclusion decision stayed with the band detector above.
            if (riseN_[i] == 0) {
                riseMin_[i] = riseMax_[i] = st_[i].last;
                risePulse_[i] = pulsesSeen_;
                riseMs_[i] = 0;
            }
            riseMin_[i] = std::min(riseMin_[i], st_[i].last);
            riseMax_[i] = std::max(riseMax_[i], st_[i].last);
            riseN_[i]++;
            riseMs_[i] += std::max(1u, dtMs);
            if (riseMs_[i] >= static_cast<uint32_t>(cfg_.stuckWindow) * 100u) {
                const float obs = riseMax_[i] - riseMin_[i];
                if (pulsesSeen_ > risePulse_[i] &&
                    bar_[i] != BAR_STUCK && bar_[i] != BAR_IMPLAUSIBLE) {
                    float refRise = 0.0f;
                    int refN = 0;
                    for (uint8_t j = 0; j < NCELL; ++j) {
                        if (j == i || !(present_ & (1u << j)) || !seen_[j]) continue;
                        if (bar_[j] == BAR_STUCK || bar_[j] == BAR_IMPLAUSIBLE) continue;
                        refRise = std::max(refRise, riseMax_[j] - riseMin_[j]);
                        refN++;
                    }
                    // Nothing to compare against, or every believed cell pinned
                    // (reference rise at/below the noise eps): the ratio would be
                    // 0/0 or arithmetic noise — no claim, keep the last score.
                    if (refN > 0 && refRise > eps)
                        st_[i].responsiveness =
                            std::max(0.0f, std::min(1.0f, obs / refRise));
                }
                riseN_[i] = 0;  // next window
                riseMs_[i] = 0;
            }
        }
    }

    // ---- vote among active cells -------------------------------------------
    // Two tiers. Trusted cells vote; if the CellProof ban has just emptied the
    // set, a cell that is out on drift alone votes instead of the machine
    // admitting it cannot see the loop at all (the reading it tracks is still
    // the best evidence available, and no-vote means no injection).
    float vals[NCELL];
    int n = 0;
    uint8_t prevActive = 0;
    uint8_t fallback = 0;
    for (uint8_t i = 0; i < NCELL; ++i) {
        if (!(present_ & (1u << i)) || !seen_[i]) continue;
        // CellProof evidence outranks the drift machinery for as long as it
        // stands: a cell that could not prove at the high setpoint may not
        // carry a vote while the machine is running a high setpoint.
        if (proofMask_ & (1u << i)) { st_[i].active = false; continue; }
        if (st_[i].active) prevActive |= (1u << i);
        // Only a cell excluded for DISAGREEING is fallback-eligible. A cell the
        // machine caught FROZEN, or one whose recent samples were all nonsense,
        // votes on a fossil — that is the "dead cells reading a plausible value
        // keep the vote alive" hole, and the last-resort tier must not be a side
        // door around the detector.
        else if (bar_[i] == BAR_NONE || bar_[i] == BAR_DRIFT) fallback |= (1u << i);
    }
    if (prevActive == 0) {
        prevActive = fallback;
        if (fallback) res.disagreement = true;  // a lone drifting cell is still news
    }
    for (uint8_t i = 0; i < NCELL; ++i)
        if (prevActive & (1u << i)) vals[n++] = st_[i].last;
    res.activeMask = prevActive;

    if (n == 0) {
        res.mode = Mode::FAILED;
        // A blind engine must keep saying so. The disagreement loop below needs
        // a median to measure against and cannot run without a vote, so without
        // this the cycle AFTER the last cell was excluded would report a calm,
        // unremarkable failure — the handset's CELL_DRIFT bit would blink off
        // exactly when the drift/disagreement story became total. Boot (nothing
        // ever seen) stays disagreement-free: there is no excluded cell there,
        // just no data yet (see cellsSeen, and alarms.cpp).
        for (uint8_t i = 0; i < NCELL; ++i)
            if ((present_ & (1u << i)) && seen_[i] &&
                (bar_[i] != BAR_NONE || !st_[i].active || (proofMask_ & (1u << i))))
                res.disagreement = true;
        return res;
    }

    const float med = (n == 2) ? std::min(vals[0], vals[1]) : medianF(vals, n);
    res.votedPpo2 = med;
    res.mode = (n == 3) ? Mode::OK_3CELL : (n == 2) ? Mode::DEGRADED_2CELL : Mode::SINGLE;

    float lo = med, hi = med;
    for (int i = 0; i < n; ++i) { lo = std::min(lo, vals[i]); hi = std::max(hi, vals[i]); }
    res.spread = hi - lo;

    // ---- disagreement window + health --------------------------------------
    for (uint8_t i = 0; i < NCELL; ++i) {
        if (!(present_ & (1u << i)) || !seen_[i]) continue;
        const float dev = std::fabs(st_[i].last - med);
        st_[i].emaDeviation = st_[i].emaDeviation * 0.95f + dev * 0.05f;

        if (dev > cfg_.driftBar) {
            // Strikes are a threshold, not an accumulator: capping them at
            // driftWindow keeps the documented hysteresis honest (a cell is
            // back after driftWindow/2 clean samples, however long it was out).
            if (st_[i].driftStrikes < cfg_.driftWindow)
                st_[i].driftStrikes = static_cast<uint16_t>(st_[i].driftStrikes + 1);
            if (st_[i].driftStrikes >= cfg_.driftWindow && st_[i].active) {
                st_[i].active = false;
                // Record WHY, for the fallback tier: a drift-out keeps its
                // last-resort seat (documented behavior — its reading still
                // tracks the loop), a frozen or never-believed cell does not.
                // Never overwrite a more damning reason with this one.
                if (bar_[i] == BAR_NONE) bar_[i] = BAR_DRIFT;
                res.dropoutEvent = false; // not a collapse; slow drift-out
            }
        } else if (dev < cfg_.driftBar * 0.5f) {
            // hysteresis return: driftWindow/2 clean samples inside half-band
            if (st_[i].driftStrikes > 0) st_[i].driftStrikes--;
            // ...but a cell the frozen detector is holding stays held: sitting
            // near the median is not evidence that a stopped channel started
            // moving again. Its own movement (a band break) is, and only the
            // detector can say that.
            if (st_[i].driftStrikes == 0 && bar_[i] != BAR_STUCK) {
                st_[i].active = !(proofMask_ & (1u << i));
                // The bars this layer raised for drift or sustained nonsense
                // lift on the same evidence that re-admits the cell. Proof bars
                // belong to CellProof and lift only when that layer says so.
                if (bar_[i] == BAR_DRIFT || bar_[i] == BAR_IMPLAUSIBLE) bar_[i] = BAR_NONE;
            }
        }
        if (!st_[i].active) res.disagreement = true;

        // health: deviation, glitches and dead responsiveness all bite
        float score = 100.0f;
        score -= st_[i].emaDeviation * 1500.0f;              // 0.05 bar -> -75
        score -= st_[i].glitchCount * 2.0f;
        score -= (1.0f - st_[i].responsiveness) * 30.0f;
        // A cell the machine is excluding is not a healthy cell, whatever its
        // rolling deviation says: the display must not show 100 next to a cell
        // the vote is not using. Frozen first — a reading that stopped changing
        // is the more damning claim — and a stale-number or never-believed cell
        // loses its score too. The bar reasons are internal to this layer; the
        // handset sees only this 0..100, no new vocabulary.
        if (bar_[i] == BAR_STUCK) score -= 100.0f;
        else if (bar_[i] != BAR_NONE) score -= 40.0f;
        st_[i].health = static_cast<uint8_t>(std::max(0.0f, std::min(100.0f, score)));
    }
    return res;
}

} // namespace voting
