// ============================================================================
// voting.h — 3-cell ppO2 voting with failure discrimination (PLAN.md 4).
//
//   * plausibility gate first: out-of-band or single-sample teleport = glitch,
//     sample skipped (previous value stands) — but a streak of them EXCLUDES the
//     cell: a channel delivering nonsense does not keep voting on the last value
//     that made sense (Config::implausibleWindow).
//   * dropout detection: one cell falling vertically = SENSOR fault ->
//     immediate exclusion. All cells falling together = real hypoxia ->
//     alarm, cells stay trusted.
//   * disagreement: |cell - median| > driftBar for driftWindow consecutive
//     samples -> excluded; returns after driftWindow/2 samples back inside
//     driftBar/2 (hysteresis).
//   * frozen-reading (stuck) detection: a plausible reading whose range across
//     Config::stuckWindow samples never exceeds a noise-sized epsilon (bench
//     dither provenance) is a dead channel, not a calm loop -> excluded until it
//     moves again (Config::stuckEpsBar).
//   * vote = median of 3 active | min of 2 (conservative) | lone cell + flag
//     | none -> failed. The last-resort fallback tier admits drift-out cells
//     only — never a frozen or never-believed one.
//   * per-cell health score (0..100): rolling deviation + glitch history +
//     injection responsiveness measured here (Loop reports the pulse count;
//     the score is RELATIVE, advisory, and documented as such).
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
    // Plausibility floor. Raised from -0.05 on 2026-11-26 (residual #2 of the
    // held-ppO2 batch: three dead galvanic cells reading a PLAUSIBLE value kept
    // the vote alive). What is defensible here is the physics of the cell, not a
    // guess: this board is PGA=16 (LSB 7.8125 uV) and the factory cell slope is
    // 0.21 bar / 1490 counts, so 0.02 bar is ~142 counts ~ 1.1 mV — about a tenth
    // of the air signal (11 mV), and below the level where a galvanic cell's own
    // baseline error lives. A reading under that is indistinguishable from a dead
    // or shorted channel, so it is not something the machine may VOTE with.
    // What this does NOT do: it is not an alarm threshold (LOW stays 0.18), and it
    // cannot hide a collapsing loop, because a real collapse reaches 0.02 by
    // PASSING THROUGH the plausible band — and the slope gate (dropoutRateBarS,
    // hypoxiaSuspect) fires while the readings are still believed. Only a physical
    // impossibility (a loop teleporting below 0.02 bar in one sample) would skip
    // that. What is then reported is FAILED + BIT_SENSOR_FAIL, which is the truth:
    // below this floor the cells are not reporting anything.
    // Kept well below the ~0.05 bar a diluent flush at depth really reaches, and
    // far below the 0.12 bar the soak insists a real loop never falls under, so no
    // legitimate reading is rejected.
    float minPpo2 = 0.02f;
    float maxPpo2 = 1.9f;

    // ---- sustained implausibility: the terminal state the glitch skip lacked --
    // A sample outside the band, or a jump past glitchMaxJump, is a GLITCH: it is
    // skipped and the cell keeps voting on its last believed value. That is right
    // for one bad sample and dangerous forever — until 2026-11-26 a cell whose
    // channel had been reading nonsense for ten minutes kept its seat in the vote
    // on a value last refreshed when nonsense began, with cellsSeen still true and
    // no claim raised anywhere. glitchCount only ever fed a health score.
    // This is the missing terminal state: that many CONSECUTIVE implausible
    // samples and the cell is excluded like any drift-out (and its stale value is
    // barred from the last-resort fallback tier too — a number from before the
    // fault is not evidence about the loop now).
    // 20 control cycles = 2 s = ~10 real ADC conversions per channel on this board
    // (3 fitted channels round-robin, ADS_SLOT_MS=65 across three register steps
    // per conversion => ~5.1 Hz per channel), so ten consecutive fresh samples are
    // nonsense before it is called. Long enough to ride out a burst, short enough
    // that a stale value can never be believed for more than 2 s.
    uint8_t implausibleWindow = 20;

    // ---- frozen-reading (stuck cell) detection ------------------------------
    // The one fault shape the level gate, the slope gate and the drift gate all
    // miss: a channel that keeps delivering a PLAUSIBLE number that never changes.
    // A wet connector freezing two channels at 0.55 bar, a shorted input, an ADC
    // register that stopped converting — all plausible, all still voting, and the
    // loop starves or floods around them.
    //
    // The tell is noise, and the epsilon is therefore noise-sized, not
    // signal-sized. Bench evidence from this board (2026-11-26 flash + acceptance
    // runs, console HB line, which prints g_ads.raw() directly): three healthy
    // cells AT REST IN AIR wobbling c0=1511/1514/1515/1517, c1=1515..1519,
    // c2=1534..1540 — 4..7 counts peak-to-peak on every 5 s beat, at ~5.1 Hz per
    // channel. 2 counts = 2 x 1.4094e-4 = 0.00028 bar. A live channel's range over
    // the window below is the observed 4..7 counts; a frozen one's is exactly 0.
    // False-positive margin is not the 2x between eps and the quietest observed
    // dither, it is combinatorial: ~102 conversions in the window all landing
    // inside 2 counts would need (1/2)^102 with the observed per-sample jitter —
    // an event that does not happen on this hardware. If a future front end is
    // quieter than a count or two, this epsilon and window must be re-derived, and
    // that is why they are config rather than constants.
    //
    // This is also why the loop being quiescent is not a defence against the
    // detector and must not be: at true steady state a healthy CELL is flat except
    // noise, and a healthy CHAIN still dithers, so the detector condemns a channel
    // that stopped changing, not a loop that stopped moving. A CCR loop is never
    // truly quiescent anyway while a diver breathes from it.
    float stuckEpsBar = 0.00028f;      // 2 ADS counts at the factory slope
    uint16_t stuckWindow = 200;        // CONSECUTIVE fresh samples that may fit
                                       // inside an eps-wide band before the
                                       // channel is called frozen. 200 control
                                       // cycles = 20 s on the board ~ 102 real
                                       // conversions per channel (main.cpp
                                       // republishes each sample ~2 control
                                       // ticks); a sparser fresh stream (a unit
                                       // test at 225 ms) needs the same number
                                       // of points, in more seconds. If a front
                                       // end ever goes quieter than ~2 counts of
                                       // dither, THIS is what must be re-derived.
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
    // Has any fitted cell produced a plausible sample yet? It is false only in
    // the window before the first ADC reading lands, or with no cells fitted —
    // which is exactly the distinction the alarm layer needs, because Mode::
    // FAILED means "the machine cannot see the loop" in both of its readings:
    // a controller that has not sampled yet (boot: no fault to raise) and one
    // whose cells have all stopped being believable (a fault that must be
    // shouted). See alarms.cpp.
    bool cellsSeen = false;
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

    // Responsiveness, 0..1, ADVISORY only (health score + handset display). The
    // loop tells the engine how many O2 pulses have accumulated (notePulses) and
    // the engine measures each cell's own rise across the same window it already
    // uses for frozen-reading detection. It is a RELATIVE score — "does cell i
    // move like the other believed cells" — because nothing in the voting layer
    // can know how far a cell SHOULD move for N pulses without a gas model.
    void setResponsiveness(uint8_t ch, float r);

    // Accumulated injector pulses, fed by Loop every control cycle. The engine
    // reads the DELTA across each cell's window, which is exactly the interval
    // over which it also measures that cell's movement.
    void notePulses(uint32_t pulses) { pulsesSeen_ = pulses; }

    // Sample-source liveness, fed by Loop from SensorData::cellsFresh. A dead or
    // disconnected ADC stream must be reported STALE by the sensor layer (its
    // age/ageMs claims), not misattributed to the cells by the voting layer: the
    // frozen-reading timer therefore only advances while samples are fresh.
    // Defaulted true so tests and the simulator need no extra wiring.
    void setSampleSourceLive(bool live) { sampleLive_ = live; }

    // Diagnostic accessors (health display, tests): why a fitted cell is out.
    enum BarReason : uint8_t {
        BAR_NONE = 0,
        BAR_DRIFT,       // disagreed with the vote (driftWindow samples)
        BAR_STUCK,       // reading never moved across stuckWindow samples
        BAR_IMPLAUSIBLE, // samples out of band / teleporting, implausibleWindow
        BAR_PROOF,       // CellProof evidence: did not track the loop
    };
    uint8_t barReason(uint8_t ch) const { return bar_[ch]; }

private:
    bool plausible(uint8_t ch, float v) const;

    Config cfg_;
    uint8_t present_ = 0x7;
    bool sampleLive_ = true;
    uint8_t proofMask_ = 0;  // cells excluded on CellProof evidence
    CellStatus st_[NCELL];
    bool seen_[NCELL] = {false, false, false};
    // Why a cell is barred, independent of WHY its exclusion counter reached the
    // limit. Two decisions depend on it, and they are NOT the same decision:
    //   - persistence: a bar only lifts through the drift hysteresis, whatever
    //     the reason;
    //   - the last-resort fallback tier: a cell excluded for DISAGREEING may
    //     still vote when nothing else is left (that is its documented job), but
    //     a cell excluded for delivering a FROZEN or never-plausible number must
    //     not — that number says nothing about the loop now.
    uint8_t bar_[NCELL] = {BAR_NONE, BAR_NONE, BAR_NONE};
    // Sustained implausibility: consecutive rejected samples per cell.
    uint8_t implausibleStrikes_[NCELL] = {0, 0, 0};
    // Frozen-reading detection: the BAND ([min,max] since the last break) of
    // consecutive fresh plausible readings per cell, and how many samples that
    // band has held. A band of cfg_.stuckWindow samples inside eps bar = frozen
    // channel. Counted in samples (see the .cpp for why not seconds).
    float stuckLo_[NCELL] = {0, 0, 0};
    float stuckHi_[NCELL] = {0, 0, 0};
    uint16_t stuckN_[NCELL] = {0, 0, 0};
    uint32_t stuckPulse_[NCELL] = {0, 0, 0};
    // Responsiveness window: movement and pulses over a rolling ~20 s window.
    float riseMin_[NCELL] = {0, 0, 0};
    float riseMax_[NCELL] = {0, 0, 0};
    uint16_t riseN_[NCELL] = {0, 0, 0};
    uint32_t riseMs_[NCELL] = {0, 0, 0};
    uint32_t risePulse_[NCELL] = {0, 0, 0};
    uint32_t pulsesSeen_ = 0;
};

} // namespace voting
