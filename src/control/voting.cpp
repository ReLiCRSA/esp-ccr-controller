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
            // it no longer takes part in.
            seen_[i] = false;
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
            continue; // keep previous value for this cycle
        }
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
            !(proofMask_ & (1u << i)))
            st_[i].active = true;
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
        else fallback |= (1u << i);
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
                res.dropoutEvent = false; // not a collapse; slow drift-out
            }
        } else if (dev < cfg_.driftBar * 0.5f) {
            // hysteresis return: driftWindow/2 clean samples inside half-band
            if (st_[i].driftStrikes > 0) st_[i].driftStrikes--;
            if (st_[i].driftStrikes == 0) st_[i].active = !(proofMask_ & (1u << i));
        }
        if (!st_[i].active) res.disagreement = true;

        // health: deviation, glitches and dead responsiveness all bite
        float score = 100.0f;
        score -= st_[i].emaDeviation * 1500.0f;              // 0.05 bar -> -75
        score -= st_[i].glitchCount * 2.0f;
        score -= (1.0f - st_[i].responsiveness) * 30.0f;
        st_[i].health = static_cast<uint8_t>(std::max(0.0f, std::min(100.0f, score)));
    }
    return res;
}

} // namespace voting
