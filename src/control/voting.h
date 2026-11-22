// ============================================================================
// voting.h — 3-cell ppO2 voting with failure discrimination (PLAN.md 4).
//
//   * plausibility gate first: out-of-band or single-sample teleport = glitch,
//     sample skipped (previous value stands).
//   * dropout detection: one cell falling vertically = SENSOR fault ->
//     immediate exclusion. All cells falling together = real hypoxia ->
//     alarm, cells stay trusted.
//   * disagreement: |cell - median| > driftBar for driftWindow consecutive
//     samples -> excluded; returns after driftWindow/2 samples back inside
//     driftBar/2 (hysteresis).
//   * vote = median of 3 active | min of 2 (conservative) | lone cell + flag
//     | none -> failed.
//   * per-cell health score (0..100): rolling deviation + glitch history +
//     injection responsiveness reported by the injector (M2 wires it).
//
// Pure C++: identical code in firmware, unit tests and the simulator.
// ============================================================================
#pragma once

#include <cstdint>

namespace voting {

struct Config {
    float driftBar = 0.05f;      // disagreement threshold
    uint8_t driftWindow = 10;    // consecutive samples before exclusion
    float glitchMaxJump = 0.5f;  // bar per sample: beyond = implausible
    float dropoutRateBarS = 0.15f; // vertical collapse -> sensor fault
    float minPpo2 = -0.05f;
    float maxPpo2 = 1.9f;
};

enum class Mode : uint8_t { OK_3CELL, DEGRADED_2CELL, SINGLE, FAILED };

struct Result {
    float votedPpo2 = 0.0f;
    float spread = 0.0f;         // max-min across active cells
    Mode mode = Mode::FAILED;
    bool dropoutEvent = false;   // a cell was excluded THIS update (sensor)
    bool hypoxiaSuspect = false; // all active cells collapsing together
    bool disagreement = false;   // any cell currently excluded/failing
    uint8_t activeMask = 0;      // cells currently in the vote
};

struct CellStatus {
    float last = 0.0f;           // last plausible reading
    bool active = false;         // in the vote
    uint16_t glitchCount = 0;
    uint16_t driftStrikes = 0;   // consecutive out-of-band samples
    float emaDeviation = 0.0f;   // rolling |cell - median|
    float responsiveness = 1.0f; // 0..1, set by injector (0 = ignored pulses)
    uint8_t health = 100;        // 0..100 display score
};

class Engine {
public:
    static constexpr uint8_t NCELL = 3;

    explicit Engine(const Config& cfg) : cfg_(cfg) {}

    void setPresentMask(uint8_t mask); // bench: 0b011 until 3rd cell fitted
    uint8_t presentMask() const { return present_; }
    /// Re-tune the bar without rebuilding (which would forget which cells are
    /// already excluded): a mid-dive CELL_DRIFT_BAR SET must not silently
    /// re-admit every excluded cell by resetting the engine.
    void setConfig(const Config& cfg) { cfg_ = cfg; }

    /// Evidence from outside this layer (CellProof): cells that have been shown
    /// not to track high ppO2. Treated like any other exclusion — reported in
    /// activeMask/disagreement, not allowed to vote — and it stays until the
    /// mask is cleared by a later proof or the dive ends. The drift hysteresis
    /// must not quietly re-admit a cell the machine has proof against.
    void setProofExclusion(uint8_t mask) { proofMask_ = static_cast<uint8_t>(mask & present_); }
    uint8_t proofExclusion() const { return proofMask_; }

    // One update per sensor cycle. ppo2[] in bar; dtMs since previous update.
    Result update(const float ppo2[NCELL], uint32_t dtMs);

    const CellStatus& cell(uint8_t ch) const { return st_[ch]; }
    void setResponsiveness(uint8_t ch, float r); // from injector

private:
    bool plausible(uint8_t ch, float v) const;

    Config cfg_;
    uint8_t present_ = 0x7;
    uint8_t proofMask_ = 0;  // cells excluded on CellProof evidence
    CellStatus st_[NCELL];
    bool seen_[NCELL] = {false, false, false};
};

} // namespace voting
