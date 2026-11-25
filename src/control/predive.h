// ============================================================================
// predive.h — pre-dive check sequencer with hard AUTO lockout (PLAN.md 7).
//
//   IDLE -> WARMUP -> AIR -> SOLENOID -> DEPTH -> BATTERY -> PASS
//
// Air/depth/battery steps evaluate their own inputs; the solenoid step is
// executed by the calibration/pulse machinery, which reports the result
// back through reportSolTest(). Any failure aborts the run; start()
// restarts from scratch. AUTO is only allowed while allPassed().
//
// DEPTH and BATTERY are the two steps that can fail by judging their own
// inputs (S_AIR waits for patience the operator owns, S_SOLENOID's verdict is
// reported to it, S_WARMUP is pure time). DEPTH fails in exactly one way — the
// pressure bus not answering for longer than cfg_.depthGraceMs — and an
// unmeasured depth is never believed, never condemned on a single cycle, and
// never allowed to hang the sequence without a verdict.
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
    // How long the DEPTH step will wait for the pressure bus to answer at all,
    // measured from the first silent cycle the STEP sees (last field on purpose:
    // the same reason Inputs::depthValid is last — these structs are filled in
    // field by field by both firmware and tests, and a new optional field must not
    // re-map what a caller meant). Same patience the firmware already grants a
    // depth sample on a dive (PRESSURE_STALE_MS, main.cpp), which is why the two
    // are pinned together in defaults.h. A shared-Wire1 hiccup gets through it; a
    // bus that is really dead still ends in FAIL rather than a sequence that never
    // returns a verdict — and what the bound costs, in both shapes the fault
    // arrives in, is written at PREDIVE_DEPTH_GRACE_MS. Tuned like every other
    // pre-dive timing — a field here plus a PREDIVE_* macro in
    // include/config/defaults.h; no PREDIVE_* timing has a settings-registry key
    // (not the warm-up, not the pulse or recover timeouts), so this one doesn't
    // either. depthGraceMs = 0 is the old single-cycle verdict, and a test pins it.
    uint32_t depthGraceMs = 3000;
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

    /// The DEPTH step is waiting on a bus that is not answering: it is RUNNING and
    /// has been given `depthGraceMs` of patience. The console has to say so out
    /// loud, because on the `PRE,` line this looks exactly like an ordinary
    /// pending wait — and no wire field may change to tell them apart.
    bool depthBusSilent() const { return depthSilent_ && st_[S_DEPTH] == RUNNING; }

    /// Why the DEPTH step is waiting, or why it failed, in words; "" when the
    /// depth bus is not involved. Same idiom as cal::AirCal's `why`: a verdict
    /// carries the reason a diver can act on. The step number carried by the
    /// surface record and the `PRE,` line is unchanged.
    const char* depthWhy() const {
        if (st_[S_DEPTH] == FAIL)
            // The grace is the only thing that can fail this step today; the
            // other wording is here so a future second failure mode can never be
            // reported to the diver as a grace expiry.
            return depthGraceExpired_ ? "depth bus never answered (grace expired)"
                                      : "depth check failed";
        if (depthSilent_) return "depth bus silent, waiting";
        return "";
    }

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
    // The DEPTH step's carried silence (see cfg_.depthGraceMs): the timestamp of
    // the first silent cycle the step saw, and whether it was the grace that
    // eventually failed it.
    uint32_t depthSilentSinceMs_ = 0;
    bool depthSilent_ = false, depthGraceExpired_ = false;
};

} // namespace predive
