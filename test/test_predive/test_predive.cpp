#include <unity.h>

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
    in.presentMask = 0b111; in.depthM = 0.05f; in.vbat = 12.4f;
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
    Inputs in2 = airGood(); in2.vbat = 9.0f;
    t = 0; // fresh clock: runUntil's 900 s guard is per-run
    s2.start(t);
    runUntil(s2, in2, Step::S_COUNT, t);
    TEST_ASSERT_TRUE(s2.failed());
    TEST_ASSERT_EQUAL_UINT8(Status::FAIL, s2.status(Step::S_BATTERY));
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

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_full_pass_sequence);
    RUN_TEST(test_air_check_needs_stability_and_two_cells);
    RUN_TEST(test_solenoid_failure_aborts);
    RUN_TEST(test_low_battery_fails_depth_gate_blocks);
    RUN_TEST(test_restart_after_failure);
    return UNITY_END();
}
