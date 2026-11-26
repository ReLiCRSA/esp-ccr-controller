// ============================================================================
// calibration.h — cell calibration: math + the two calibration procedures.
//
// One raw->ppO2 mapping per cell (linear, two-point):
//     ppO2 = airRef + slope * (raw - rawAir)
//
//   CAL AIR     : mandatory. Averages stable raw readings while breathing
//                 air and stores the low point (0.21 bar).
//   CAL O2      : two-step confirmed enrichment. The engine asks the control
//                 layer to push the loop toward o2Fi (bench only!), watches
//                 for the ppO2 ceiling, then samples the high point and
//                 solves slope+offset. A cell whose slope lands outside the
//                 sanity band, or that barely moved between the two points,
//                 keeps its previous calibration and is reported bad — a
//                 lying calibration is worse than an old one.
//
// Raw units are whatever the ADC layer publishes (mV); calibration only ever
// works with differences, so units cancel through the slope.
// ============================================================================
#pragma once

#include <cstdint>

namespace cal {

struct Config {
    float airRefBar = 0.21f;
    float o2Fi = 0.99f;           // ppO2 reference at the O2 point (bench, 1 bar)
    float o2TargetBar = 0.95f;    // enrichment target (engine waits here)
    float o2CeilingBar = 1.00f;   // abort enrichment above this
    float slopeBandPct = 15.0f;   // slope vs previous/default: accept window
    float minRiseFrac = 0.30f;    // raw rise >= frac * expected rise (else dead cell)
    uint32_t enrichTimeoutMs = 300000;
    uint32_t settleMs = 15000;    // hold target before sampling
    uint8_t stableSamples = 10;
};

struct Cal {
    float slope = 0.0f;    // ppO2 per raw unit
    float airBar = 0.21f;  // reference ppO2 at the air point
    float rawAir = 0.0f;
    float rawO2 = 0.0f;
    bool hasAir = false;
    bool hasO2 = false;
    float apply(float raw) const { return airBar + slope * (raw - rawAir); }
    bool valid() const { return hasAir && slope > 0.0f; }
};

// ---- per-procedure results -------------------------------------------------
enum CellVerdict : uint8_t { CV_OK = 0, CV_SLOPE_OUT, CV_NO_RISE, CV_NO_AIR };

struct O2Result {
    bool aborted = false;          // ceiling breach or timeout
    const char* abortReason = nullptr;
    CellVerdict verdict[3] = {CV_OK, CV_OK, CV_OK};
};

// Pure math, so it is directly unit-testable with synthetic raw points.
struct Solver {
    // Given air raw + O2 raw per cell + reference config: fill Cal entries.
    // prev/defaultSlope provide the sanity-band reference (ppO2 per raw unit).
    static void solveO2(const Config& cfg,
                        const float rawO2[3],
                        const Cal prev[3], float defaultSlope,
                        Cal out[3], O2Result& res);
};

// ---- CAL AIR sampler --------------------------------------------------------
// The averaging window over raw ADC readings at the air point. ControlTask
// owns the raw snapshot, the dive state and the solenoid, and it commits the
// result into cal::Cal — this class is only the disciplined window:
//
//   * refuses to run while diving (a pre-dive act, decided where it runs);
//   * an injection pulse discards the window: ppO2 mid-pulse is not air;
//   * stale ADC data does not advance the window (never average a lie);
//   * every PRESENT cell must be STABLE, but not frozen: real electro-
//     chemical cells wander — gentle drift, hysteresis, noise, and the odd
//     momentary excursion across a neighbour cell's level — so each cell is
//     measured against its own trend across the window (a least-squares
//     line), not against a flat line. Samples must sit within spreadMax raw
//     counts of that trend, with outlierMax of them per window allowed to
//     step outside (an excursion is normal; a window full of them is not);
//     the trend itself may move at most driftMaxPerS counts/s. What still
//     fails: sustained instability (more than outlierMax samples outside the
//     band) and a true warm-up ramp breaking the rate limit — averaging
//     either would commit a lie as the air point. Absent cells are ignored
//     entirely: their raw register holds another channel's last value.
//
// One sample per samplePeriodMs, `samples` of them; a stability violation
// restarts the count (the cell gets a chance to settle), the wall-clock
// timeout is the outer patience.
struct AirCalInput {
    bool diving = false;
    bool injecting = false;       // solenoid actually fired this tick
    bool fresh = false;           // ADC channels fresh (not stale)
    uint8_t presentMask = 0x7;
    float cellsRaw[3] = {0, 0, 0};
    uint32_t nowMs = 0;
};

// Why the sampler gave up, as a NUMBER. `why` (English) is for the console a
// engineer reads; a number is for the wire a client has to branch on. Until
// 2026-11-24 the string was all there was, so the controller's real CAL AIR
// verdict could not be published at all without a client matching English
// sentences — and the console is the only place the verdict ever appeared.
// Append-only: never renumber, new reasons go on the right.
enum class AirCalWhy : uint8_t {
    NONE = 0,
    DIVING = 1,       // a dive started; CAL AIR is a pre-dive act
    UNSETTLED = 2,    // never stayed inside the spread band before the timeout
    NOCELLS = 3,      // present mask empty: nothing to average
};

struct AirCalOutput {
    bool running = false;
    bool done = false;            // set in the one call that completes
    bool failed = false;          // set in the one call that gives up
    float rawAvg[3] = {0, 0, 0};  // valid when done
    float spreadWorst = 0.0f;     // worst per-cell spread, raw counts
    uint8_t samples = 0;          // progress
    const char* why = nullptr;    // failure reason (English, console-bound)
    AirCalWhy whyCode = AirCalWhy::NONE;   // the same fact, wire-bound
};

class AirCal {
public:
    struct Cfg {
        uint32_t samplePeriodMs = 1000;
        uint8_t samples = 12;
        float spreadMax = 40.0f;       // raw counts per cell around the window trend
        float driftMaxPerS = 10.0f;    // raw counts/s the trend itself may carry
        uint8_t outlierMax = 2;        // band-crossing samples per cell the window may carry
        uint32_t timeoutMs = 90000;    // patience before declaring failure
    };
    explicit AirCal(const Cfg& cfg) : cfg_(cfg) {}
    void setConfig(const Cfg& cfg) { cfg_ = cfg; }

    void start(uint32_t nowMs);
    void abort() { st_ = St::IDLE; }
    bool running() const { return st_ == St::RUN; }

    // Call every control tick while running (cheap when idle). done/failed
    // arrive exactly once, in the output of the tick that decides them.
    AirCalOutput update(const AirCalInput& in);

private:
    enum class St : uint8_t { IDLE, RUN };
    void resetWindow(uint32_t nowMs);

    static constexpr uint8_t kMaxSamples = 16;  // hist capacity; cfg_.samples clamps to it

    Cfg cfg_;
    St st_ = St::IDLE;
    float sum_[3] = {0, 0, 0}, lo_[3] = {0, 0, 0}, hi_[3] = {0, 0, 0};
    float hist_[3][kMaxSamples] = {};  // window samples, for the trend fit
    uint8_t n_ = 0;
    uint32_t nextSampleMs_ = 0, deadlineMs_ = 0;
};

// ---- CAL O2 enrichment state machine ----------------------------------------
enum class O2State : uint8_t { IDLE, ENRICHING, SETTLING, DONE, FAILED };

struct O2Input {
    bool diveActive = false;      // refuses while diving: bench-only procedure
    /// The RAW vote, never ctrl::Loop's held display value: the enrichment verdict
    /// is about the loop's actual ppO2, and a held number would end the enrich at a
    /// target the loop never reached. Nothing wires this state machine into the
    /// firmware yet (no caller builds an O2Input), so this is the contract for
    /// whoever does.
    float votedBar = 0.21f;
    float cellsRaw[3] = {0, 0, 0};
    uint32_t nowMs = 0;
};

struct O2Output {
    O2State state = O2State::IDLE;
    bool requestEnrich = false;   // control layer: raise SP to cfg.o2TargetBar
    bool started = false;
    O2Result result;
};

class O2Sequencer {
public:
    explicit O2Sequencer(const Config& cfg) : cfg_(cfg) {}
    void setConfig(const Config& cfg) { cfg_ = cfg; }

    // Two-step UX: arm() then confirm() (handset/web both enforce this).
    bool arm();                   // needs air cal present (setAirCal)
    bool confirm(uint32_t nowMs);
    void rejectArm() { armed_ = false; }

    void setAirCal(const Cal c[3]) {
        for (int i = 0; i < 3; ++i) base_[i] = c[i];
        airReady_ = c[0].hasAir || c[1].hasAir || c[2].hasAir;
    }

    O2Output update(const O2Input& in);
    O2State state() const { return state_; }
    const Cal* cal() const { return base_; }

private:
    Config cfg_;
    O2State state_ = O2State::IDLE;
    bool armed_ = false, airReady_ = false;
    Cal base_[3] = {};
    float rawSum_[3] = {};
    uint32_t settleStart_ = 0, runStart_ = 0;
    uint8_t stable_ = 0;
    float rawMax_ = 0.0f; // voted ceiling already breached
};

// ---- scripted pulse responsiveness test (TEST CELLS) ------------------------
// Baseline -> control layer delivers K pulses -> per-cell rise must appear ->
// loop must fall back. A cell that doesn't move when O2 is definitely being
// added is poisoned/dead, whichever way its air cal looks.
enum class TestState : uint8_t { IDLE, BASELINE, PULSING, RECOVER, DONE };

struct PulseTestInput {
    float cellsBar[3];            // calibrated ppO2 per cell
    /// Copied by the chain and NOT read by PulseTest today (it grades the per-cell
    /// rise, not the level). If it gains a consumer it must be the RAW vote: a held
    /// baseline would flatten the very rise the test is looking for, making healthy
    /// cells look dead and dead ones look fine.
    float votedBar;
    uint8_t presentMask = 0b111;
    uint32_t pulsesDelivered = 0; // monotonic count from injector
    uint32_t nowMs = 0;
};

struct PulseTestOutput {
    TestState state = TestState::IDLE;
    bool requestPulse = false;    // one per call until pulsesDone reached
    uint8_t passMask = 0;         // bit set = cell responded
    uint8_t failMask = 0;
    bool done = false;
};

class PulseTest {
public:
    struct Cfg {
        uint8_t baselineSamples = 6;
        uint8_t pulses = 3;
        float riseMinBar = 0.03f;      // per-cell rise to count as responsive
        uint32_t pulseTimeoutMs = 20000;  // verdict even if duty cap throttles pulses
        uint32_t recoverTimeoutMs = 30000;
        float recoverTolBar = 0.05f;   // back within this of baseline
    };
    explicit PulseTest(const Cfg& c) : cfg_(c) {}

    void start() {
        state_ = TestState::BASELINE;
        n_ = 0; pulsesSent_ = 0; pending_ = false;
        passMask_ = failMask_ = 0;
        for (int i = 0; i < 3; ++i) { base_[i] = 0.0f; peak_[i] = 0.0f; }
    }

    PulseTestOutput update(const PulseTestInput& in);
    TestState state() const { return state_; }
    /// Which cells reacted / did not, for the handset `TEST CELLS` readout
    /// (the pre-dive sequence uses the same engine and only needs the verdict).
    uint8_t passMask() const { return passMask_; }
    uint8_t failMask() const { return failMask_; }

private:
    Cfg cfg_;
    TestState state_ = TestState::IDLE;
    uint8_t n_ = 0, pulsesSent_ = 0;
    bool pending_ = false;
    float base_[3] = {}, peak_[3] = {};
    uint32_t startPulseMs_ = 0, recStart_ = 0;
    uint8_t passMask_ = 0, failMask_ = 0;
};

} // namespace cal
