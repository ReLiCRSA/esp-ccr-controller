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

#include "config/defaults.h"

namespace predive {

enum Step : uint8_t { S_WARMUP = 0, S_AIR, S_SOLENOID, S_DEPTH, S_BATTERY, S_COUNT };
enum Status : uint8_t { PENDING, RUNNING, PASS, FAIL };

struct Config {
    uint32_t warmupMs = 120000;      // cell polarisation
    float airTargetBar = 0.21f;
    float airTolPct = 8.0f;          // all cells within X% of target
    uint8_t airStableSamples = 10;   // held consecutively
    float depthMaxM = 1.0f;          // must be essentially on the surface
    float vbatMin = VBAT_MIN_V;
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

    // --- the battery step judges three things, not one (all three last, for
    // the same reason depthGraceMs and Inputs::depthValid are: these structs are
    // filled field by field by firmware and by tests alike).
    //
    // vbatMax: the reading's plausibility ceiling. A pre-dive gate that asks
    // only "above the floor?" is PASSED by a sense node sitting on its rail, and
    // this bench has read vbat=34.71 from exactly that. A pack the board cannot
    // read is not a pack that is fine.
    //
    // vbatCells and vbatSpreadMaxV: series only. With VBAT_CELLS == 1 there is
    // no midpoint to compare and the check is inert; with 2, one pack voltage can
    // hide one dead cell (6.80 V total is compatible with 4.00 + 2.80), which is
    // precisely the failure a voltage-only gate is otherwise built to miss.
    float vbatMax = VBAT_IMPLAUSIBLE_V;
    uint8_t vbatCells = VBAT_CELLS;
    float vbatSpreadMaxV = VBAT_CELL_SPREAD_MAX_V;

    // --- the coil rail: the fourth question this step asks, and the one that
    // used not to exist (docs/REVIEW.md 1 P9). All of it last, for the same
    // reason as everything else optional here.
    //
    // Why the rail is a question inside the BATTERY step rather than a sixth
    // step: the `PRE,` line is one status field per step followed by the passed
    // bit, and the handset's parser reads that bit by position —
    // `handset/handset_link.h`'s parsePre() takes field 6 as "passed". A sixth
    // step inserted BEFORE it would hand a handset the new step's status as
    // "this dive passed", and there is no way to append a step AFTER the passed
    // bit. So the step count stays five and the step that already asks "is the
    // power this machine runs on trustworthy" asks it about both rails. What the
    // diver loses is a step number in the surface record; what the console gains
    // is railWhy(), which says which rail and why.
    float vrailMin = VRAIL_MIN_V;
    float vrailMax = VRAIL_IMPLAUSIBLE_V;
    // Whether this build reads the pin at all. 0 is today's truth (no divider
    // fitted): the question is then not asked, and defaults.h plus the boot
    // announce are what keep that from being mistaken for a PASS.
    uint8_t vrailSensed = VRAIL_SENSE;
    // Patience the pack does not need. The SOLENOID step pulses the valve and
    // runs BEFORE this step, and SensorTask's sweep can also land on a pulse at
    // random, so one out-of-band sweep is not evidence about the power supply —
    // and it is not evidence about it being fine either, which is why this is a
    // wait with a deadline rather than a retry that never ends. Same shape as
    // depthGraceMs, including that 0 is the old single-cycle verdict and a test
    // pins it.
    uint32_t vrailSettleMs = VRAIL_SETTLE_MS;
};

struct Inputs {
    float cells[3];
    uint8_t presentMask = 0b111;
    float depthM = 0.0f;
    float vbat = VBAT_MIN_V + 0.5f;
    uint32_t nowMs = 0;
    // Optional (last on purpose — structs here are aggregate-initialised in
    // tests and firmware, and inserting a field in the middle would silently
    // re-map every positional initialiser). Is `depthM` a measurement? The
    // firmware feeds the pressure bus's own health flag. Default true keeps
    // every existing caller's meaning; what it must never be is "guessed from
    // depthM", because a dead bus used to be handed to the DEPTH step as 0.0 m
    // and PASSed it: "no sensor" and "on the surface" arrived as one number.
    bool depthValid = true;

    // Series packs only: the midpoint between the cells, divider-scaled to PACK
    // volts like `vbat` — i.e. the voltage of the lower half of the stack, so
    // with two cells the pair disagrees by |2*vbatMid - vbat|. 0 means "not
    // sensed", and with VBAT_CELLS >= 2 that is a fault, not a free pass: the
    // one reason a series pack is riskier to judge by voltage is the thing this
    // input exists to check. With VBAT_CELLS == 1 the value is ignored.
    float vbatMid = 0.0f;

    // The coil rail (the ~12 V the valve runs on), divider-scaled to rail volts.
    // 0 means "not sensed", and when vrailSensed is set that is a refusal: the
    // defect this input exists to close is precisely the one where the machine
    // has no number for the coil's supply and decides anyway. Default 0 rather
    // than a plausible 12.0 on purpose — an input a caller forgot to fill must
    // fail closed, not read as healthy.
    float vrail = 0.0f;
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

    /// The BATTERY step is waiting for the coil rail to come back inside its
    /// band: RUNNING, and within cfg_.vrailSettleMs of the first out-of-band
    /// sweep. This exists for the same reason depthBusSilent() does — on the
    /// `PRE,` line a wait looks exactly like an ordinary pending step, and no
    /// wire field may change to tell them apart.
    bool railWaiting() const { return railWaiting_ && st_[S_BATTERY] == RUNNING; }

    /// Why the BATTERY step is waiting on the rail, or why it failed there; ""
    /// when the rail is not involved. It deliberately says nothing about the pack
    /// side of the step: that fault has never been ambiguous (HB's vbat and
    /// batlow name it), and a step that can fail two ways should not describe one
    /// of them in words that could be read as the other.
    const char* railWhy() const {
        if (st_[S_BATTERY] == FAIL) {
            if (railUnwired_) return "coil rail never sensed: the sense input reads 0";
            if (railSettleExpired_) return "coil rail never settled back inside its band";
            return "";
        }
        if (railWaiting_) return "coil rail outside its band, waiting for it to settle";
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
    // The BATTERY step's rail wait (see cfg_.vrailSettleMs): when the first
    // out-of-band sweep was seen, whether the step is in that wait, whether the
    // wait is what failed it, and whether the rail was never sensed at all. The
    // last one is kept separate because "there is no reading" and "the reading is
    // wrong" are different things for a diver to fix, and the console has to say
    // which one it is reporting.
    uint32_t railWaitStart_ = 0;
    bool railWaiting_ = false, railSettleExpired_ = false, railUnwired_ = false;
};

} // namespace predive
