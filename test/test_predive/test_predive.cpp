#include <unity.h>

#include <limits>

#include "control/predive.h"

using predive::Config;
using predive::Inputs;
using predive::Sequencer;
using predive::Status;
using predive::Step;

void setUp() {}
void tearDown() {}

static Inputs airGood() {
    Inputs in;
    in.cells[0] = 0.212f; in.cells[1] = 0.208f; in.cells[2] = 0.210f;
    in.presentMask = 0b111; in.depthM = 0.05f; in.vbat = VBAT_CELLS * 3.90f;
    // Matched halves, so the series check in the battery step is satisfied by
    // the fixture and not accidentally the thing under test. Ignored at 1 cell.
    in.vbatMid = (VBAT_CELLS * 3.90f) / 2.0f;
    return in;
}

static void runUntil(Sequencer& s, Inputs in, Step target, uint32_t& t) {
    while (s.current() != target && !s.failed() && !s.allPassed() && t < 900000) {
        s.update(in, t);
        if (s.current() == Step::S_SOLENOID && s.status(Step::S_SOLENOID) == Status::RUNNING)
            s.reportSolTest(true, 2);
        t += 1000;
    }
}

void test_full_pass_sequence() {
    Config c; c.warmupMs = 5000; // accelerated for the test
    Sequencer s{c};
    Inputs in = airGood();
    uint32_t t = 0;
    s.start(t);
    TEST_ASSERT_TRUE(s.running());
    TEST_ASSERT_FALSE(s.allPassed());

    runUntil(s, in, Step::S_COUNT, t);
    TEST_ASSERT_TRUE(s.allPassed());
    TEST_ASSERT_EQUAL_UINT8(Status::PASS, s.status(Step::S_WARMUP));
    TEST_ASSERT_EQUAL_UINT8(Status::PASS, s.status(Step::S_SOLENOID));
}

void test_air_check_needs_stability_and_two_cells() {
    Config c; c.warmupMs = 0;
    Sequencer s{c};
    Inputs in = airGood();
    uint32_t t = 0;
    s.start(t);
    // one bad sample resets the stable counter
    in.cells[1] = 0.10f;
    for (int i = 0; i < 5; ++i) s.update(in, t += 100);
    in.cells[1] = 0.208f;
    for (int i = 0; i < 5; ++i) s.update(in, t += 100);
    TEST_ASSERT_EQUAL_UINT8(Step::S_AIR, s.current()); // not advanced yet
    for (int i = 0; i < 12; ++i) s.update(in, t += 100);
    TEST_ASSERT_TRUE(s.current() > Step::S_AIR);       // now advanced
}

void test_solenoid_failure_aborts() {
    Config c; c.warmupMs = 0;
    Sequencer s{c};
    Inputs in = airGood();
    uint32_t t = 0;
    s.start(t);
    for (int i = 0; i < 15 && s.current() != Step::S_SOLENOID; ++i) s.update(in, t += 100);
    TEST_ASSERT_EQUAL_UINT8(Step::S_SOLENOID, s.current());
    s.reportSolTest(false, 2); // no ppO2 rise
    s.update(in, t += 100);
    TEST_ASSERT_TRUE(s.failed());
    TEST_ASSERT_FALSE(s.allPassed());
    TEST_ASSERT_EQUAL_UINT8(Status::FAIL, s.status(Step::S_SOLENOID));
}

void test_low_battery_fails_depth_gate_blocks() {
    Config c; c.warmupMs = 0;
    Sequencer s{c};
    Inputs in = airGood();
    uint32_t t = 0;
    s.start(t);
    in.depthM = 15.0f; // not on the surface: gate must not pass
    runUntil(s, in, Step::S_BATTERY, t);
    TEST_ASSERT_TRUE(s.current() == Step::S_DEPTH);
    TEST_ASSERT_FALSE(s.allPassed());
    in.depthM = 0.1f;                        // surfaced: gate opens
    s.update(in, t += 100);
    s.reportSolTest(true, 1); // (in case solenoid still pending — no-op otherwise)
    for (int i = 0; i < 3; ++i) s.update(in, t += 100);
    TEST_ASSERT_TRUE(s.allPassed());

    // low battery now fails at its step
    Sequencer s2{c};
    Inputs in2 = airGood(); in2.vbat = VBAT_CELLS * 3.10f;   // below the 1S floor (VBAT_CELL_MIN_V)
    t = 0; // fresh clock: runUntil's 900 s guard is per-run
    s2.start(t);
    runUntil(s2, in2, Step::S_COUNT, t);
    TEST_ASSERT_TRUE(s2.failed());
    TEST_ASSERT_EQUAL_UINT8(Status::FAIL, s2.status(Step::S_BATTERY));
}

// The battery step used to ask one question — "above the floor?" — and a reading
// from a broken sense node answers it YES. The divider is two resistors on a
// connector: open the top leg and the ADC pin drifts to its rail. This bench has
// actually read vbat=34.71 that way, so a pre-dive gate that passed it was
// passing a pack it could not see.
void test_battery_step_refuses_an_implausible_reading() {
    Config c; c.warmupMs = 0;
    Sequencer s{c};
    Inputs in = airGood();
    in.vbat = VBAT_IMPLAUSIBLE_V + 3.0f;              // a 12 V-style rail: not this pack, not readable
    uint32_t t = 0;
    s.start(t);
    runUntil(s, in, Step::S_COUNT, t);
    TEST_ASSERT_TRUE_MESSAGE(s.failed(), "an unreadable pack must not pass the battery step");
    TEST_ASSERT_EQUAL_UINT8(Status::FAIL, s.status(Step::S_BATTERY));

    // ... and the same sequence passes either side of the fault, so the refusal is
    // the reading, not the fixture: this is the direction that makes the check
    // above mean something.
    Sequencer ok{c};
    Inputs good = airGood();       // 3.90 V: inside the band
    t = 0;
    ok.start(t);
    runUntil(ok, good, Step::S_COUNT, t);
    TEST_ASSERT_TRUE(ok.allPassed());
    TEST_ASSERT_EQUAL_UINT8(Status::PASS, ok.status(Step::S_BATTERY));

    // Right at the ceiling is still a pack. The bound is a fault detector, not a
    // charge limit, so it must not bite the top of a real charge.
    Sequencer edge{c};
    Inputs full = airGood(); full.vbat = VBAT_IMPLAUSIBLE_V;
    full.vbatMid = VBAT_IMPLAUSIBLE_V / 2.0f;   // at the ceiling, still matched
    t = 0;
    edge.start(t);
    runUntil(edge, full, Step::S_COUNT, t);
    TEST_ASSERT_TRUE(edge.allPassed());
}

// Series packs: one total voltage is not two cells. 6.80 V is compatible with
// 4.00 + 2.80, i.e. a dead cell behind a gate that only reads the stack. The
// midpoint input exists to make that a refusal, and an unwired midpoint must
// refuse too — "we cannot compare" is not "the cells agree".
void test_series_pack_checks_cell_agreement() {
    Config c; c.warmupMs = 0; c.vbatCells = 2;
    c.vbatMin = 2 * VBAT_CELL_MIN_V;      // 6.80 V for two cells
    c.vbatMax = 2 * VBAT_CELL_IMPLAUSIBLE_V;

    // Matched: 3.85 + 3.85 = 7.70 V, midpoint 3.85.
    Sequencer matched{c};
    Inputs a = airGood(); a.vbat = 7.70f; a.vbatMid = 3.85f;
    uint32_t t = 0;
    matched.start(t);
    runUntil(matched, a, Step::S_COUNT, t);
    TEST_ASSERT_TRUE_MESSAGE(matched.allPassed(), "two matched cells must pass on a series pack");

    // One cell flat: 4.00 + 2.80 = 6.80 V, exactly at the floor, so the OLD
    // single-voltage gate passed it. This is the regression this leg exists for.
    Sequencer dead{c};
    Inputs b = airGood(); b.vbat = 6.80f; b.vbatMid = 4.00f;
    t = 0;
    dead.start(t);
    runUntil(dead, b, Step::S_COUNT, t);
    TEST_ASSERT_TRUE_MESSAGE(dead.failed(), "a series pack with one dead cell must refuse");
    TEST_ASSERT_EQUAL_UINT8(Status::FAIL, dead.status(Step::S_BATTERY));

    // Unwired midpoint: not evidence of agreement.
    Sequencer blind{c};
    Inputs d = airGood(); d.vbat = 7.70f; d.vbatMid = 0.0f;
    t = 0;
    blind.start(t);
    runUntil(blind, d, Step::S_COUNT, t);
    TEST_ASSERT_TRUE(blind.failed());
    TEST_ASSERT_EQUAL_UINT8(Status::FAIL, blind.status(Step::S_BATTERY));

    // And it refuses because of its own guard, not as a side effect of the spread
    // arithmetic: widen the bound until the subtraction would have passed, and
    // the unwired midpoint must still refuse. Without this leg, deleting the
    // guard changes nothing observable and the check above proves nothing about
    // it.
    Config loose = c; loose.vbatSpreadMaxV = 20.0f;
    Sequencer blind2{loose};
    Inputs e0 = airGood(); e0.vbat = 7.70f; e0.vbatMid = 0.0f;
    t = 0;
    blind2.start(t);
    runUntil(blind2, e0, Step::S_COUNT, t);
    TEST_ASSERT_TRUE_MESSAGE(blind2.failed(),
                             "an unwired midpoint must refuse on its own rule, not by arithmetic luck");
    TEST_ASSERT_EQUAL_UINT8(Status::FAIL, blind2.status(Step::S_BATTERY));

    // With ONE cell there is no midpoint to compare, and a zero input must not
    // become a refusal: the same fixture that fails above has to pass here, or
    // the check is a trap for the pack the rig actually has. Forced to 1 rather
    // than left at the default, so this leg means the same thing no matter what
    // defaults.h currently says the bench pack is.
    Config one; one.warmupMs = 0; one.vbatCells = 1;
    one.vbatMin = VBAT_CELL_MIN_V; one.vbatMax = VBAT_CELL_IMPLAUSIBLE_V;
    Sequencer single{one};
    Inputs e = airGood(); e.vbat = 3.90f; e.vbatMid = 0.0f;
    t = 0;
    single.start(t);
    runUntil(single, e, Step::S_COUNT, t);
    TEST_ASSERT_TRUE(single.allPassed());
}

void test_restart_after_failure() {
    Config c; c.warmupMs = 0;
    Sequencer s{c};
    Inputs in = airGood();
    uint32_t t = 0;
    s.start(t);
    while (s.current() != Step::S_SOLENOID) { s.update(in, t += 100); if (s.current() == Step::S_SOLENOID) s.reportSolTest(false, 2); }
    s.update(in, t += 100);
    TEST_ASSERT_TRUE(s.failed());
    s.start(t);
    TEST_ASSERT_TRUE(s.running());
    TEST_ASSERT_FALSE(s.failed());
}

// Finding C1 / review H3: a dead depth bus used to reach this step as the
// 0.0 m somebody fabricated for "unknown", and 0.0 m is exactly what the step
// wants to see — so the rig was certified "on the surface" by the failure of
// its own depth sensor. The step must answer the prior question first: is
// there a depth at all?
//
// The grace window changed WHEN the verdict arrives, not whether it does: one
// silent cycle is no longer a fault (a shared-Wire1 hiccup must not abort a
// 2-minute sequence), but a bus that never answers still ends in FAIL, and the
// wait is bounded and named. Both halves are asserted here, because each half
// is the failure mode of the other: an immediate FAIL condemns a transient, and
// a grace with no bound is a pre-dive that never returns a verdict.
void test_dead_depth_bus_fails_the_depth_step() {
    Config c; c.warmupMs = 0;  // depthGraceMs: the 3000 ms default
    Sequencer s{c};
    Inputs in = airGood();
    uint32_t t = 0;
    s.start(t);
    // The exact firmware shape of the fault: depthM is the surface (because it
    // was invented), and the bus is not believed.
    in.depthM = 0.0f;
    in.depthValid = false;
    runUntil(s, in, Step::S_DEPTH, t);
    TEST_ASSERT_EQUAL_UINT8(Step::S_DEPTH, s.current());

    // ---- mid-grace: not believed, not condemned, and visible as silence -----
    const uint32_t tArm = t + 100;  // the first silent cycle arms the clock
    for (int i = 0; i < 20; ++i) s.update(in, t += 100);  // 1.9 s of silence
    TEST_ASSERT_FALSE_MESSAGE(s.failed(), "the DEPTH step condemned a silent bus inside the grace");
    TEST_ASSERT_EQUAL_UINT8(Status::RUNNING, s.status(Step::S_DEPTH));
    TEST_ASSERT_TRUE_MESSAGE(s.depthBusSilent(),
                             "a silent depth bus was not visible as silence (the `PRE,` line "
                             "cannot tell this wait from a rig off the surface, so the "
                             "sequencer has to be able to say it)");
    TEST_ASSERT_EQUAL_STRING_MESSAGE("depth bus silent, waiting", s.depthWhy(),
                                     "the depth wait did not say what it is waiting on");

    // ---- past the grace: the C1 verdict still arrives, bounded and named ----
    while (!s.failed() && t < tArm + c.depthGraceMs + 1000) s.update(in, t += 100);
    TEST_ASSERT_TRUE_MESSAGE(s.failed(), "the DEPTH step never failed on a dead depth bus");
    TEST_ASSERT_TRUE_MESSAGE(t >= tArm + c.depthGraceMs,
                             "the depth FAIL landed before the grace had run");
    TEST_ASSERT_TRUE_MESSAGE(t <= tArm + c.depthGraceMs + 100,
                             "the depth FAIL was not bounded by the grace: the sequence hung "
                             "past it instead of returning a verdict");
    TEST_ASSERT_EQUAL_UINT8(Status::FAIL, s.status(Step::S_DEPTH));
    TEST_ASSERT_FALSE_MESSAGE(s.allPassed(), "pre-dive PREPASS with a dead depth bus");
    TEST_ASSERT_EQUAL_STRING_MESSAGE("depth bus never answered (grace expired)", s.depthWhy(),
                                     "the depth FAIL did not say the grace is what expired");
    TEST_ASSERT_FALSE(s.depthBusSilent());  // a verdict, not a wait, any more
}

// A bus that is alive but telling nonsense is refused too — the validity flag
// is not a licence for any number the driver happens to be holding. A sane but
// off-surface depth WAITS (the diver can still surface, and the existing depth
// gate test pins that); only "there is no measurement" is ever a verdict, and
// that one waits out the grace before it lands.
void test_invalid_and_wild_depth_both_refused() {
    Config c; c.warmupMs = 0;
    Sequencer s{c};
    Inputs in = airGood();
    uint32_t t = 0;
    s.start(t);
    runUntil(s, in, Step::S_DEPTH, t);
    TEST_ASSERT_EQUAL_UINT8(Step::S_DEPTH, s.current());

    in.depthM = 400.0f;  // believed, but no sane rig is there: still waits
    for (int i = 0; i < 5; ++i) s.update(in, t += 100);
    TEST_ASSERT_EQUAL_UINT8(Step::S_DEPTH, s.current());
    TEST_ASSERT_FALSE(s.failed());

    in.depthM = std::numeric_limits<float>::quiet_NaN();  // the comparison refuses it
    for (int i = 0; i < 5; ++i) s.update(in, t += 100);
    TEST_ASSERT_EQUAL_UINT8(Step::S_DEPTH, s.current());
    TEST_ASSERT_FALSE(s.failed());

    // and a held sample behind an invalid flag is not a measurement either: it
    // waits out the grace like any other silent cycle, then FAILs. Whatever
    // number the driver happens to be holding never decides the verdict.
    Sequencer s2{c};
    Inputs in2 = airGood();
    in2.depthM = 0.05f;
    in2.depthValid = false;
    t = 0;
    s2.start(t);
    runUntil(s2, in2, Step::S_DEPTH, t);
    const uint32_t tArm = t + 100;
    s2.update(in2, t += 100);
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(Status::RUNNING, s2.status(Step::S_DEPTH),
                                    "a held sample behind an invalid flag was condemned on "
                                    "one cycle");
    while (!s2.failed() && t < tArm + c.depthGraceMs + 1000) s2.update(in2, t += 100);
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(Status::FAIL, s2.status(Step::S_DEPTH),
                                    "a held sample behind a dead bus never reached its FAIL");
    TEST_ASSERT_TRUE(s2.failed());
    TEST_ASSERT_TRUE_MESSAGE(t >= tArm + c.depthGraceMs,
                             "a held sample behind an invalid flag failed inside the grace");
    TEST_ASSERT_EQUAL_STRING("depth bus never answered (grace expired)", s2.depthWhy());
}

// A bus that recovers is not a fault the diver has to restart for: while the
// sample is believed and sane the step still passes on its own.
void test_depth_step_recovers_with_the_bus() {
    Config c; c.warmupMs = 0;
    Sequencer s{c};
    Inputs in = airGood();
    uint32_t t = 0;
    s.start(t);
    in.depthValid = true;
    in.depthM = 0.05f;
    runUntil(s, in, Step::S_COUNT, t);
    TEST_ASSERT_TRUE(s.allPassed());
    TEST_ASSERT_EQUAL_UINT8(Status::PASS, s.status(Step::S_DEPTH));
}

// ---------------------------------------------------------------------------
// The grace window (follow-up #2). C1 closed the hole where a dead depth bus
// PASSed the DEPTH step; the fix condemned it on a single cycle, which meant a
// shared-Wire1 hiccup landing exactly then aborted a 2-minute sequence. These
// tests pin the middle ground: an unmeasured depth is never believed, never
// condemned on one cycle, and never allowed to hang the sequence either.
// ---------------------------------------------------------------------------

// Shorter than the grace: the dropout the diver should never have to notice.
void test_depth_bus_silent_shorter_than_the_grace_passes() {
    Config c; c.warmupMs = 0;  // depthGraceMs: the 3000 ms default
    Sequencer s{c};
    Inputs in = airGood();
    uint32_t t = 0;
    s.start(t);
    runUntil(s, in, Step::S_DEPTH, t);
    TEST_ASSERT_EQUAL_UINT8(Step::S_DEPTH, s.current());

    // 2 s of silence, inside the grace, and fed the exact number the old
    // firmware fabricated — it must not be what decides anything.
    in.depthValid = false;
    in.depthM = 0.0f;
    for (int i = 0; i < 20; ++i) s.update(in, t += 100);
    TEST_ASSERT_FALSE_MESSAGE(s.failed(), "a 2 s depth dropout aborted the pre-dive");
    TEST_ASSERT_TRUE(s.depthBusSilent());  // and it was visible while it lasted
    TEST_ASSERT_EQUAL_UINT8(Status::RUNNING, s.status(Step::S_DEPTH));

    // The bus answers: the step judges the sample it got and the sequence
    // finishes. No restart, no lost work — WARMUP/AIR/SOLENOID keep their verdicts.
    in.depthValid = true;
    in.depthM = 0.05f;
    s.update(in, t += 100);
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(Status::PASS, s.status(Step::S_DEPTH),
                                    "a depth dropout shorter than the grace failed the DEPTH step");
    TEST_ASSERT_FALSE(s.depthBusSilent());
    TEST_ASSERT_EQUAL_STRING_MESSAGE("", s.depthWhy(),
                                     "the depth step kept complaining after the bus came back");
    for (int i = 0; i < 5; ++i) s.update(in, t += 100);
    TEST_ASSERT_TRUE_MESSAGE(s.allPassed(),
                             "pre-dive did not finish after a depth dropout shorter than the grace");
    TEST_ASSERT_FALSE(s.failed());
}

// Longer than the grace: the FAIL must land on the grace's schedule, not wait to
// see whether the bus eventually comes back. This is the ms5837 deep-backoff
// trade written down: a driver that resumes at 4 s loses the step, because a
// pre-dive that polls a bus the dive itself already called dead is not serving
// the diver.
void test_depth_bus_silent_longer_than_the_grace_fails_when_the_grace_expires() {
    Config c; c.warmupMs = 0;
    Sequencer s{c};
    Inputs in = airGood();
    uint32_t t = 0;
    s.start(t);
    in.depthValid = false;
    in.depthM = 0.0f;
    runUntil(s, in, Step::S_DEPTH, t);
    const uint32_t tArm = t + 100;

    // 5 s of silence, twice the grace: the verdict arrives at the grace, well
    // before the silence ends, and it is bounded (no hang).
    uint32_t tFail = 0;
    for (int i = 0; i < 50 && !s.failed(); ++i) {
        s.update(in, t += 100);
        if (s.failed() && tFail == 0) tFail = t;
    }
    TEST_ASSERT_TRUE_MESSAGE(s.failed(), "5 s of silent depth bus produced no verdict");
    TEST_ASSERT_TRUE_MESSAGE(tFail >= tArm + c.depthGraceMs,
                             "the depth FAIL landed before the grace had run");
    TEST_ASSERT_TRUE_MESSAGE(tFail <= tArm + c.depthGraceMs + 100,
                             "the depth FAIL was not bounded by the grace (silent hang)");
    TEST_ASSERT_TRUE_MESSAGE(tFail + 1000 < tArm + 5000,
                             "the depth FAIL waited for the bus to come back instead of "
                             "landing on the grace's schedule");
    TEST_ASSERT_EQUAL_STRING("depth bus never answered (grace expired)", s.depthWhy());

    // A verdict is a verdict: the bus coming back 2 s later does not un-fail the
    // sequence, and AUTO stays locked until someone starts the check again.
    in.depthValid = true;
    in.depthM = 0.05f;
    for (int i = 0; i < 10; ++i) s.update(in, t += 100);
    TEST_ASSERT_TRUE(s.failed());
    TEST_ASSERT_FALSE(s.allPassed());
    TEST_ASSERT_EQUAL_UINT8(Status::FAIL, s.status(Step::S_DEPTH));

    // The escape hatch, pinned so it cannot rot: a maintainer who wants the old
    // single-cycle verdict sets the grace to zero and gets exactly that.
    Config c0 = c;
    c0.depthGraceMs = 0;
    Sequencer s0{c0};
    Inputs in0 = airGood();
    in0.depthValid = false;
    uint32_t t0 = 0;
    s0.start(t0);
    runUntil(s0, in0, Step::S_DEPTH, t0);
    s0.update(in0, t0 += 100);
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(Status::FAIL, s0.status(Step::S_DEPTH),
                                    "depthGraceMs = 0 did not restore the one-cycle verdict");
}

// The off-by-one, pinned deliberately on one side. The grace FAILs on a cycle
// whose bus is STILL silent at or past depthGraceMs of continuous silence, so the
// verdict is always a statement about the bus and never about a sample that
// actually arrived. Three legs:
//   * a measurement landing exactly on the boundary tick is JUDGED (here: PASS);
//   * a bus still silent on that same tick FAILs on it — a dead bus delivers no
//     measurement, so it can never interpose its own reprieve;
//   * an off-surface measurement on the boundary is judged too, which is what
//     keeps the grace a patience bound and not a blanket "timeout = FAIL".
void test_depth_grace_boundary_judges_a_sample_and_condemns_silence() {
    // Leg 1: sample exactly at the boundary.
    Config c; c.warmupMs = 0;
    Sequencer s{c};
    Inputs in = airGood();
    uint32_t t = 0;
    s.start(t);
    runUntil(s, in, Step::S_DEPTH, t);
    const uint32_t tArm = t + 100;
    in.depthValid = false;
    in.depthM = 0.0f;
    while (t < tArm + c.depthGraceMs - 100) s.update(in, t += 100);
    TEST_ASSERT_FALSE_MESSAGE(s.failed(),
                              "the DEPTH step failed on silence inside its grace");
    TEST_ASSERT_EQUAL_UINT8(Status::RUNNING, s.status(Step::S_DEPTH));
    in.depthValid = true;
    in.depthM = 0.05f;  // in band: judged, and it passes
    s.update(in, t += 100);  // this cycle is exactly depthGraceMs of silence old
    TEST_ASSERT_FALSE_MESSAGE(s.failed(),
                              "a measurement arriving at the grace boundary was condemned "
                              "instead of judged");
    TEST_ASSERT_EQUAL_UINT8(Status::PASS, s.status(Step::S_DEPTH));

    // Leg 2: the same tick, but the bus is still silent -> FAIL there, not later.
    Sequencer s2{c};
    Inputs in2 = airGood();
    in2.depthValid = false;
    in2.depthM = 0.0f;
    t = 0;
    s2.start(t);
    runUntil(s2, in2, Step::S_DEPTH, t);
    const uint32_t tArm2 = t + 100;
    while (!s2.failed() && t < tArm2 + c.depthGraceMs + 5000) s2.update(in2, t += 100);
    TEST_ASSERT_TRUE_MESSAGE(s2.failed(),
                             "silence running past the grace produced no verdict at all");
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(tArm2 + c.depthGraceMs, t,
                                     "silence at the grace boundary did not FAIL the step there");

    // Leg 3: measurement at the boundary, off the surface -> still a wait.
    Sequencer s3{c};
    Inputs in3 = airGood();
    uint32_t t3 = 0;
    s3.start(t3);
    runUntil(s3, in3, Step::S_DEPTH, t3);
    const uint32_t tArm3 = t3 + 100;
    in3.depthValid = false;
    while (t3 < tArm3 + c.depthGraceMs - 100) s3.update(in3, t3 += 100);
    in3.depthValid = true;
    in3.depthM = 12.0f;  // the rig is still down: it may still surface
    for (int i = 0; i < 40; ++i) s3.update(in3, t3 += 100);
    TEST_ASSERT_FALSE_MESSAGE(s3.failed(),
                              "the grace expired on a measurement the step was supposed to wait for");
    TEST_ASSERT_EQUAL_UINT8(Step::S_DEPTH, s3.current());
    TEST_ASSERT_EQUAL_UINT8(Status::RUNNING, s3.status(Step::S_DEPTH));
    TEST_ASSERT_FALSE(s3.depthBusSilent());  // waiting on the diver, not on the bus
}

// Where the grace clock starts: the first silent cycle the STEP sees, not the
// moment the bus died. Silence that began during WARMUP (far longer than the
// grace) must not have spent the DEPTH step's patience in advance — the steps
// before it have their own patience, and their own verdicts.
void test_depth_grace_clocks_from_the_step_not_from_the_bus() {
    Config c; c.warmupMs = 5000;
    Sequencer s{c};
    Inputs in = airGood();
    in.depthValid = false;
    in.depthM = 0.0f;
    uint32_t t = 0;
    s.start(t);
    runUntil(s, in, Step::S_DEPTH, t);  // ~16 s of silent bus before this step
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(Step::S_DEPTH, s.current(),
                                    "the DEPTH step never got its own patience");
    TEST_ASSERT_FALSE(s.failed());

    const uint32_t tArm = t + 100;
    s.update(in, t += 100);
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(Status::RUNNING, s.status(Step::S_DEPTH),
                                    "the DEPTH step had already failed before its own grace "
                                    "had a chance to run");
    while (!s.failed() && t < tArm + c.depthGraceMs + 5000) s.update(in, t += 100);
    TEST_ASSERT_TRUE_MESSAGE(s.failed(),
                             "the DEPTH step hung: a bus that never answers must still FAIL");
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(tArm + c.depthGraceMs, t,
                                     "the depth grace was partly spent by silence the DEPTH "
                                     "step never saw");
}

// ---------------------------------------------------------------------------
// The rail question inside the BATTERY step (docs/REVIEW.md 1 P9). Every test
// above runs with the default vrailSensed = VRAIL_SENSE = 0 — the honest answer
// for today's bench — and passes on the pack alone. These tests set
// vrailSensed = 1 and ask what a build that READS the pin decides, so the
// default-0 pass above can never be read as "the rail was checked and passed".
// ---------------------------------------------------------------------------

// Fail-closed default: a caller that forgot to fill Inputs::vrail leaves 0, and
// on a sensed build "0 V on a 12 V coil rail" is a refusal, not a free pass.
void test_sensed_build_refuses_a_rail_that_was_never_measured() {
    Config c; c.warmupMs = 0; c.vrailSensed = 1;
    Sequencer s{c};
    Inputs in = airGood();                 // vrail left at its 0 default
    uint32_t t = 0;
    s.start(t);
    runUntil(s, in, Step::S_COUNT, t);
    TEST_ASSERT_TRUE_MESSAGE(s.failed(),
                             "a sensed build passed the battery step with no rail "
                             "reading at all");
    TEST_ASSERT_EQUAL_UINT8(Status::FAIL, s.status(Step::S_BATTERY));
    TEST_ASSERT_EQUAL_STRING_MESSAGE("coil rail never sensed: the sense input reads 0",
                                     s.railWhy(),
                                     "no number and a wrong number are different faults "
                                     "to repair and must not share one sentence");

    // Mutation direction: the identical fixture with the question NOT asked
    // passes — so the refusal above is the sense flag, not the fixture.
    Config c0; c0.warmupMs = 0;             // vrailSensed defaults to VRAIL_SENSE = 0
    Sequencer off{c0};
    t = 0;
    off.start(t);
    runUntil(off, in, Step::S_COUNT, t);
    TEST_ASSERT_TRUE_MESSAGE(off.allPassed(),
                             "the un-sensed build must still pass on the pack alone, "
                             "or these tests cannot tell the two builds apart");
}

// Both edges of the band judge, and the middle passes. Right AT the limits is
// inside: the band is the coil's published supply tolerance, and a rail sitting
// at the edge of its tolerance is still a rail inside its tolerance.
void test_rail_band_edges_pass_and_both_sides_refuse() {
    Config c; c.warmupMs = 0; c.vrailSensed = 1;

    struct { float v; const char* what; } cases[] = {
        { VRAIL_MIN_V, "at the low edge: inside" },
        { VRAIL_IMPLAUSIBLE_V, "at the high edge: inside" },
    };
    for (auto& k : cases) {
        Sequencer s{c};
        Inputs in = airGood(); in.vrail = k.v;
        uint32_t t = 0;
        s.start(t);
        runUntil(s, in, Step::S_COUNT, t);
        TEST_ASSERT_TRUE_MESSAGE(s.allPassed(), k.what);
        TEST_ASSERT_EQUAL_STRING("", s.railWhy());
    }
}

// LOW is a wait, not an instant verdict — the SOLENOID step pulses the valve
// seconds earlier and SensorTask's 1 Hz sweep can land on a pulse at random.
// The wait has a deadline; a supply that is simply gone still ends in FAIL.
void test_low_rail_waits_then_fails_at_the_settle_deadline() {
    Config c; c.warmupMs = 0; c.vrailSensed = 1;
    Sequencer s{c};
    Inputs in = airGood(); in.vrail = 8.0f;          // sagging, but sensed
    uint32_t t = 0;
    s.start(t);
    runUntil(s, in, Step::S_BATTERY, t);
    const uint32_t tArm = t;

    // The step should be RUNNING, saying it waits, until the deadline — and the
    // FAIL must land at the deadline, not before it (no patience) and not never
    // (a sequence that never returns a verdict).
    while (s.status(Step::S_BATTERY) == Status::RUNNING && t < 900000)
        s.update(in, t += 100);
    TEST_ASSERT_TRUE_MESSAGE(s.failed(), "a rail that never came back must FAIL");
    TEST_ASSERT_TRUE_MESSAGE(t >= tArm + c.vrailSettleMs,
                             "the rail FAILed before its settle deadline: one "
                             "pulse-shaped sweep is not evidence about the supply");
    TEST_ASSERT_TRUE_MESSAGE(t <= tArm + c.vrailSettleMs + 200u,
                             "the rail step hung past its deadline instead of failing");
    TEST_ASSERT_EQUAL_STRING_MESSAGE("coil rail never settled back inside its band",
                                     s.railWhy(),
                                     "the deadline FAIL must name itself on the console");
}

// Mid-wait the step must SAY it is waiting: on the `PRE,` line a wait is
// byte-for-byte indistinguishable from an ordinary RUNNING step, and railWaiting()
// is the only surface that tells them apart — the same deal depthBusSilent() makes.
void test_rail_wait_is_visible_while_it_is_happening() {
    Config c; c.warmupMs = 0; c.vrailSensed = 1;
    Sequencer s{c};
    Inputs in = airGood(); in.vrail = 8.0f;
    uint32_t t = 0;
    s.start(t);
    runUntil(s, in, Step::S_BATTERY, t);
    s.update(in, t += 100);                       // first out-of-band sweep
    TEST_ASSERT_TRUE_MESSAGE(s.railWaiting(), "the step waited without saying so");
    TEST_ASSERT_EQUAL_STRING("coil rail outside its band, waiting for it to settle",
                             s.railWhy());
    TEST_ASSERT_FALSE_MESSAGE(s.failed(), "waiting is not failing yet");
}

// A rail that comes back inside the band during the wait PASSES: the wait exists
// precisely because one low sweep can be the pulse, not the supply.
void test_rail_that_recovers_during_the_wait_passes() {
    Config c; c.warmupMs = 0; c.vrailSensed = 1;
    Sequencer s{c};
    Inputs in = airGood(); in.vrail = 8.0f;
    uint32_t t = 0;
    s.start(t);
    runUntil(s, in, Step::S_BATTERY, t);
    s.update(in, t += 100);                       // arms the wait
    TEST_ASSERT_TRUE(s.railWaiting());
    in.vrail = VRAIL_NOMINAL_V;                   // boost recovered
    s.update(in, t += 100);
    TEST_ASSERT_TRUE_MESSAGE(s.allPassed(),
                             "a recovered rail must pass: the wait exists to let it");
    TEST_ASSERT_FALSE(s.railWaiting());
    TEST_ASSERT_EQUAL_STRING("", s.railWhy());
}

// The implausible side is judged too: a sense node on its rail is not "a healthy
// coil supply". It waits like LOW does — a steady fault costs the deadline and
// still ends in FAIL.
void test_high_rail_fails_too() {
    Config c; c.warmupMs = 0; c.vrailSensed = 1;
    Sequencer s{c};
    Inputs in = airGood(); in.vrail = VRAIL_IMPLAUSIBLE_V + 3.0f;
    uint32_t t = 0;
    s.start(t);
    runUntil(s, in, Step::S_BATTERY, t);
    while (s.status(Step::S_BATTERY) == Status::RUNNING && t < 900000)
        s.update(in, t += 100);
    TEST_ASSERT_TRUE_MESSAGE(s.failed(),
                             "a rail above its band passed: an open divider must not "
                             "look like a healthy coil supply");
    TEST_ASSERT_EQUAL_UINT8(Status::FAIL, s.status(Step::S_BATTERY));
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_full_pass_sequence);
    RUN_TEST(test_air_check_needs_stability_and_two_cells);
    RUN_TEST(test_solenoid_failure_aborts);
    RUN_TEST(test_low_battery_fails_depth_gate_blocks);
    RUN_TEST(test_battery_step_refuses_an_implausible_reading);
    RUN_TEST(test_series_pack_checks_cell_agreement);
    RUN_TEST(test_restart_after_failure);
    RUN_TEST(test_dead_depth_bus_fails_the_depth_step);
    RUN_TEST(test_invalid_and_wild_depth_both_refused);
    RUN_TEST(test_depth_step_recovers_with_the_bus);
    RUN_TEST(test_depth_bus_silent_shorter_than_the_grace_passes);
    RUN_TEST(test_depth_bus_silent_longer_than_the_grace_fails_when_the_grace_expires);
    RUN_TEST(test_depth_grace_boundary_judges_a_sample_and_condemns_silence);
    RUN_TEST(test_depth_grace_clocks_from_the_step_not_from_the_bus);
    RUN_TEST(test_sensed_build_refuses_a_rail_that_was_never_measured);
    RUN_TEST(test_rail_band_edges_pass_and_both_sides_refuse);
    RUN_TEST(test_low_rail_waits_then_fails_at_the_settle_deadline);
    RUN_TEST(test_rail_wait_is_visible_while_it_is_happening);
    RUN_TEST(test_rail_that_recovers_during_the_wait_passes);
    RUN_TEST(test_high_rail_fails_too);
    return UNITY_END();
}
