// ============================================================================
// predive.h — pre-dive check sequencer with hard AUTO lockout (PLAN.md 7).
//
//   IDLE -> WARMUP -> AIR -> SOLENOID -> DEPTH -> BATTERY -> PASS
//
// Air/depth/battery steps evaluate their own inputs; the solenoid step is
// executed by the calibration/pulse machinery, which reports the result
// back through reportSolTest(). Any failure aborts the run; start()
// restarts from scratch. AUTO is only allowed while allPassed().
// ============================================================================
#pragma once

#include <cstdint>

namespace predive {

enum Step : uint8_t { S_WARMUP = 0, S_AIR, S_SOLENOID, S_DEPTH, S_BATTERY, S_COUNT };
enum Status : uint8_t { PENDING, RUNNING, PASS, FAIL };

struct Config {
    uint32_t warmupMs = 120000;      // cell polarisation
    float airTargetBar = 0.21f;
    float airTolPct = 8.0f;          // all cells within X% of target
    uint8_t airStableSamples = 10;   // held consecutively
    float depthMaxM = 1.0f;          // must be essentially on the surface
    float vbatMin = 10.5f;
};

struct Inputs {
    float cells[3];
    uint8_t presentMask = 0b111;
    float depthM = 0.0f;
    float vbat = 12.0f;
    uint32_t nowMs = 0;
    // Optional (last on purpose — structs here are aggregate-initialised in
    // tests and firmware, and inserting a field in the middle would silently
    // re-map every positional initialiser). Is `depthM` a measurement? The
    // firmware feeds the pressure bus's own health flag. Default true keeps
    // every existing caller's meaning; what it must never be is "guessed from
    // depthM", because a dead bus used to be handed to the DEPTH step as 0.0 m
    // and PASSed it: "no sensor" and "on the surface" arrived as one number.
    bool depthValid = true;
};

class Sequencer {
public:
    explicit Sequencer(const Config& cfg) : cfg_(cfg) { reset(); }
    void setConfig(const Config& cfg) { cfg_ = cfg; }

    void start(uint32_t nowMs);
    void update(const Inputs& in, uint32_t nowMs);

    // callback from the solenoid/cell test machinery
    void reportSolTest(bool riseOk, uint32_t pulsesUsed);

    Status status(Step s) const { return st_[s]; }
    Step current() const { return current_; }
    bool running() const { return state_ == 1; }
    bool allPassed() const { return state_ == 2; }
    bool failed() const { return state_ == 3; }

private:
    void reset();
    void advance(Step done);
    void fail(Step s);
    Config cfg_;
    Status st_[S_COUNT];
    Step current_ = S_WARMUP;
    uint8_t state_ = 0;          // 0 idle, 1 running, 2 passed, 3 failed
    uint32_t warmStart_ = 0;
    uint8_t airStable_ = 0;
    bool solReported_ = false, solOk_ = false;
};

} // namespace predive
