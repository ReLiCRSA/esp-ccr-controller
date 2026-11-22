// ============================================================================
// injector.h — O2 solenoid pulse control (PLAN.md 5). Pure logic + virtual
// clock: the identical code runs on the ESP32 ControlTask, the unit tests
// and the system simulator.
//
// Deterministic bang-bang: no PID windup. Hard walls enforced here, not
// scattered: never start a pulse at/above highWall; drive released at/above
// cutoff; duty capped over a rolling window so runaway firmware cannot
// flood the loop; pulses refused when sensing is unhealthy (safe bias).
// Stuck-open detection runs on depth-stable windows only (a ppO2 rise
// during descent is physics, not a fault).
// ============================================================================
#pragma once

#include <cstdint>

namespace inj {

struct Config {
    uint16_t pulseMs = 300;
    uint8_t maxDutyPct = 5;        // % of rolling window
    uint32_t windowMs = 10000;     // duty window
    uint16_t minGapMs = 2000;      // between pulses
    float dbOnBar = 0.03f;         // inject when voted < sp - dbOn
    float dbOffBar = 0.02f;        // stop when voted > sp + dbOff
    float highWallBar = 1.45f;     // never START pulses at/above
    float cutoffBar = 1.60f;       // force off at/above, latched below
    // Software fuse on the output path: the valve pin may not stay HIGH longer
    // than this, whatever the logic above asked for. A pulse is pulseMs, so
    // tripping means something in the driver/task chain has gone wrong. It
    // latches like a stuck-open valve, because the observable outcome is the
    // same: oxygen is flowing and not stopping.
    uint16_t maxOnMs = 900;
    float stuckSlopeBarS = 0.05f;  // rise while not injecting...
    float stuckRiseBar = 0.06f;    // ...by at least this much...
    uint32_t stuckEvalMs = 5000;   // ...over this steady-depth window
    float stuckHystBar = 0.02f;    // re-arm window only on a real fall back
};

struct Input {
    float votedBar;
    float setpointBar;
    bool sensorHealthy = true;
    bool autoMode = false;      // AUTO: normal control; else bench pulses only
    bool depthStable = true;    // for stuck-open evaluation
    uint32_t nowMs = 0;
    uint32_t dtMs = 0;
    // Optional (last on purpose: structs here are aggregate-initialised in
    // tests and firmware, and inserting a field in the middle would silently
    // re-map every positional initialiser). Highest reading any fitted cell
    // reports, even one excluded from the vote: the HIGH/CUTOFF walls must
    // trip on it too. When two of three cells stick LOW the median lies while
    // the honest third sees the truth — that cell keeps its veto.
    float highVetoBar = 0.0f;
};

struct Output {
    bool solenoid = false;
    bool pulseStarted = false;
    bool inhibitedHigh = false;     // highWall/cutoff latch active
    bool stuckOpenLatched = false;  // fault: valve suspected open
    bool outputFuse = false;        // fault: pin HIGH longer than maxOnMs
    bool refused = false;           // demand existed but injection refused
};

class Injector {
public:
    explicit Injector(const Config& cfg) : cfg_(cfg) {}
    void setConfig(const Config& cfg) { cfg_ = cfg; }

    /// Bench/calibration: a pulse that may also fire in non-AUTO modes (the
    /// walls still apply). Requests are counted, not latched: the caller asks
    /// for N pulses and the gap/duty rules decide when they leave. A single
    /// boolean here used to swallow requests — a test that asked for 3 pulses
    /// got 2, and the verdict had to wait for the test's own timeout.
    void requestTestPulse(uint8_t n = 1) {
        testPending_ = (testPending_ + n > kMaxTestQueue) ? kMaxTestQueue
                                                          : (uint8_t)(testPending_ + n);
    }
    /// Withdraw every queued test pulse once its test stops waiting: pulses
    /// that never got to fire must not sit pending and inject minutes later.
    void cancelTestPulse() { testPending_ = 0; }
    uint8_t testPulsesPending() const { return testPending_; }
    /// One clear path for both valve faults (suspected open valve and a
    /// tripped output fuse): the diver has physically dealt with it.
    void clearStuckOpen() {
        stuckLatched_ = false;
        riseBaseValid_ = false;
        onRun_ = false;
    }
    bool stuckOpenLatched() const { return stuckLatched_; }
    bool outputFuseTripped() const { return fuseTripped_; }

    Output update(const Input& in);

    uint32_t pulsesTotal() const { return pulses_; }
    uint32_t pulsesInWindow() const;
    float dutyPct(uint32_t nowMs) const;
    bool pulseActive() const { return pulseRemMs_ > 0; }

private:
    struct Pulse { uint32_t startMs, endMs; };
    static constexpr int MAX_LOG = 64;
    static constexpr uint8_t kMaxTestQueue = 8;  // queue cap: a test asks for 3
    float error(float voted, float sp) const { return sp - voted; }
    bool aboveHighWall(float v) const { return v >= cfg_.highWallBar; }
    bool cutoffLatched(float v) const;

    Config cfg_;
    Pulse log_[MAX_LOG];
    int logHead_ = 0, logCount_ = 0;
    void logOn(uint32_t startMs, uint32_t endMs);

    uint32_t pulseRemMs_ = 0;
    uint32_t pulseStartMs_ = 0;
    uint32_t lastPulseEndMs_ = 0;

    // output-fuse run tracking
    bool onRun_ = false;
    bool fuseTripped_ = false;
    uint32_t onRunStartMs_ = 0;
    uint8_t testPending_ = 0;  // queued test/calibration pulses (see requestTestPulse)
    bool cutoffLatch_ = false;
    bool stuckLatched_ = false;

    // stuck-open baseline while NOT injecting
    bool riseBaseValid_ = false;
    uint32_t riseBaseMs_ = 0;
    float riseBaseVal_ = 0.0f;
    uint32_t offSinceMs_ = 0;

    uint32_t pulses_ = 0;
};

} // namespace inj
