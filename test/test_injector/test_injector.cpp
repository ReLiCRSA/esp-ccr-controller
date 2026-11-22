#include <unity.h>

#include "control/injector.h"

using inj::Config;
using inj::Injector;
using inj::Input;
using inj::Output;

void setUp() {}
void tearDown() {}

// step helper: run one 100 ms control tick
static Output step(Injector& inj, uint32_t& now, float voted, float sp,
                   bool autoMode = true, bool depthStable = true,
                   bool healthy = true) {
    Input in{voted, sp, healthy, autoMode, depthStable, now, 100};
    now += 100;
    return inj.update(in);
}

void test_pulse_when_below_setpoint() {
    Injector inj{Config{}};
    uint32_t now = 1000;
    bool started = false;
    int onTicks = 0;
    for (int i = 0; i < 100; ++i) {
        const Output o = step(inj, now, 0.30f, 0.70f); // far below
        started |= o.pulseStarted;
        if (o.solenoid) onTicks++;
    }
    TEST_ASSERT_TRUE(started);
    TEST_ASSERT_EQUAL_UINT32(1, inj.pulsesTotal());
    TEST_ASSERT_TRUE(onTicks >= 2); // 300 ms / 100 ms ticks
}

void test_no_pulse_within_deadband() {
    Injector inj{Config{}};
    uint32_t now = 0;
    for (int i = 0; i < 50; ++i) {
        const Output o = step(inj, now, 0.69f, 0.70f);
        TEST_ASSERT_FALSE(o.solenoid);
        TEST_ASSERT_FALSE(o.pulseStarted);
    }
    TEST_ASSERT_EQUAL_UINT32(0, inj.pulsesTotal());
}

void test_early_release_above_offband() {
    Injector inj{Config{}};
    uint32_t now = 0;
    step(inj, now, 0.60f, 0.70f);            // starts a pulse
    int onAfterRise = 0;
    for (int i = 0; i < 10; ++i)
        if (step(inj, now, 0.75f, 0.70f).solenoid) onAfterRise++; // above sp+db
    TEST_ASSERT_EQUAL(0, onAfterRise);       // released immediately
}

void test_high_wall_and_cutoff_latch() {
    Config c{}; // highWall 1.45 cutoff 1.60
    c.dbOnBar = 0.20f; // force demand: sp 1.65 voted 1.47 < sp-db
    Injector inj{c};
    uint32_t now = 0;
    for (int i = 0; i < 30; ++i) {
        const Output o = step(inj, now, 1.47f, 1.65f);
        TEST_ASSERT_FALSE(o.pulseStarted);
        TEST_ASSERT_TRUE(o.inhibitedHigh);
    }
    // above cutoff: latches; sinking below highWall releases
    for (int i = 0; i < 5; ++i) TEST_ASSERT_TRUE(step(inj, now, 1.62f, 1.65f).inhibitedHigh);
    bool midInhibited = true;
    for (int i = 0; i < 5; ++i) midInhibited = step(inj, now, 1.50f, 1.65f).inhibitedHigh;
    TEST_ASSERT_TRUE(midInhibited);              // between walls: stays off
    TEST_ASSERT_FALSE(step(inj, now, 1.40f, 1.65f).inhibitedHigh); // released
}

void test_duty_cap_limits_pulses() {
    Injector inj{Config{}}; // 300 ms pulses, 5% of 10 s = 500 ms
    uint32_t now = 0;
    for (int i = 0; i < 100; ++i) step(inj, now, 0.05f, 1.30f); // endless demand
    TEST_ASSERT_TRUE(inj.dutyPct(now) <= 5.1f);
    TEST_ASSERT_TRUE(inj.pulsesTotal() <= 2); // ~500 ms budget in 10 s
}

void test_test_pulse_works_in_standby_but_walls_apply() {
    Injector inj{Config{}};
    uint32_t now = 0;
    inj.requestTestPulse();
    bool started = false;
    for (int i = 0; i < 20; ++i)
        started |= step(inj, now, 0.30f, 0.40f, /*autoMode*/ false).pulseStarted;
    TEST_ASSERT_TRUE(started);

    inj.requestTestPulse();
    started = false;
    for (int i = 0; i < 20; ++i)
        started |= step(inj, now, 1.50f, 1.65f, false).pulseStarted;
    TEST_ASSERT_FALSE(started); // high wall beats the bench command
}

// The cell test asks for N pulses in consecutive cycles; gap and duty rules
// then decide when they leave. Each request must be kept, not overwritten —
// with a latched flag, the third request was swallowed and the pulse test only
// reached its verdict after its own timeout.
void test_test_pulse_requests_are_queued() {
    Config c{};
    c.minGapMs = 200;
    c.maxDutyPct = 40;  // duty is the subject of another test; keep it out of this one
    Injector inj{c};
    uint32_t now = 0;
    for (int i = 0; i < 3; ++i) inj.requestTestPulse();
    TEST_ASSERT_EQUAL_UINT8(3, inj.testPulsesPending());
    for (int i = 0; i < 20; ++i) step(inj, now, 0.30f, 0.40f, /*autoMode*/ false);
    TEST_ASSERT_EQUAL_UINT32(3, inj.pulsesTotal());
    TEST_ASSERT_EQUAL_UINT8(0, inj.testPulsesPending());

    inj.requestTestPulse();
    inj.requestTestPulse();
    inj.cancelTestPulse();  // the test moved on: nothing may inject later
    TEST_ASSERT_EQUAL_UINT8(0, inj.testPulsesPending());
    for (int i = 0; i < 20; ++i)
        TEST_ASSERT_FALSE(step(inj, now, 0.30f, 0.40f, false).pulseStarted);
    TEST_ASSERT_EQUAL_UINT32(3, inj.pulsesTotal());
}

// Output fuse: the driver layer refuses to hold the valve open past maxOnMs,
// whatever asked for it, and latches like the stuck-open valve it imitates.
void test_output_fuse_forces_the_valve_off() {
    Config c{};
    c.pulseMs = 2000;   // longer than the fuse allows
    c.maxOnMs = 500;
    c.maxDutyPct = 40;  // a 2 s pulse needs duty headroom just to start
    Injector inj{c};
    uint32_t now = 0;
    bool fuse = false, onAfterFuse = false;
    for (int i = 0; i < 40; ++i) {
        const Output o = step(inj, now, 0.30f, 0.70f);
        fuse |= o.outputFuse;
        if (fuse && o.solenoid) onAfterFuse = true;
    }
    TEST_ASSERT_TRUE_MESSAGE(fuse, "fuse never tripped");
    TEST_ASSERT_FALSE_MESSAGE(onAfterFuse, "valve still driven after the fuse");
    TEST_ASSERT_TRUE(inj.outputFuseTripped());
    TEST_ASSERT_TRUE(inj.stuckOpenLatched());

    for (int i = 0; i < 20; ++i)
        TEST_ASSERT_FALSE(step(inj, now, 0.30f, 0.70f).pulseStarted);
    inj.clearStuckOpen();
    bool started = false;
    for (int i = 0; i < 10; ++i) started |= step(inj, now, 0.30f, 0.70f).pulseStarted;
    TEST_ASSERT_TRUE(started);
}

void test_unhealthy_sensing_refuses_injection() {
    Injector inj{Config{}};
    uint32_t now = 0;
    for (int i = 0; i < 40; ++i) {
        const Output o = step(inj, now, 0.20f, 0.70f, true, true, /*healthy*/ false);
        TEST_ASSERT_FALSE(o.pulseStarted);
    }
    TEST_ASSERT_EQUAL_UINT32(0, inj.pulsesTotal());
}

void test_stuck_open_detection_and_lockout() {
    Injector inj{Config{}};
    uint32_t now = 0;
    // depth stable, no injection, ppO2 rising 0.4 -> ~0.76 over 6 s (0.06/s)
    float v = 0.40f;
    for (int i = 0; i < 60; ++i) { v += 0.006f; step(inj, now, v, 0.30f); }
    // once latched: refuses even with demand, until cleared
    Output o;
    bool latched = false;
    for (int i = 0; i < 10; ++i) { o = step(inj, now, 0.60f, 1.00f); latched |= o.stuckOpenLatched; }
    TEST_ASSERT_TRUE(latched);
    TEST_ASSERT_FALSE(o.pulseStarted);
    inj.clearStuckOpen();
    bool resumed = false;
    for (int i = 0; i < 30; ++i) resumed |= step(inj, now, 0.40f, 1.00f).pulseStarted;
    TEST_ASSERT_TRUE(resumed);
}

void test_descent_rise_is_not_stuck_open() {
    Injector inj{Config{}};
    uint32_t now = 0;
    float v = 0.40f;
    bool latched = false;
    for (int i = 0; i < 60; ++i) { v += 0.0033f; // same rise as above...
        latched |= step(inj, now, v, 0.30f, true, /*depthStable*/ false).stuckOpenLatched; }
    TEST_ASSERT_FALSE(latched); // ...but descending: physics, not fault
}

void test_stuck_open_window_survives_reading_dither() {
    // Regression: ppO2 sample noise used to re-arm the evaluation window every
    // cycle, so a valve that was wide open never accumulated 5 s of evidence.
    Injector inj{Config{}};
    uint32_t now = 0;
    bool latched = false;
    for (int i = 0; i < 80; ++i) {
        const float v = 0.40f + 0.006f * i + ((i % 2) ? -0.010f : 0.0f);  // trend + dither
        latched |= step(inj, now, v, 0.30f).stuckOpenLatched;
    }
    TEST_ASSERT_TRUE_MESSAGE(latched, "dither hid a stuck-open valve");
}

void test_honest_cell_vetoes_injection_when_the_vote_lies() {
    // Two cells stuck low: the vote says 0.50 (demand on) while an excluded
    // third cell sees 1.50. The high wall must act on the veto.
    Config c{};
    Injector inj{c};
    uint32_t now = 0;
    for (int i = 0; i < 20; ++i) {
        Input in{0.50f, 0.70f, true, true, true, now, 100};
        in.highVetoBar = 1.50f;
        now += 100;
        const Output o = inj.update(in);
        TEST_ASSERT_FALSE_MESSAGE(o.pulseStarted, "pulse started against the cell veto");
        TEST_ASSERT_TRUE(o.inhibitedHigh);
    }
    TEST_ASSERT_EQUAL_UINT32(0, inj.pulsesTotal());
}

void test_veto_also_holds_the_pulse_already_running() {
    Injector inj{Config{}};
    uint32_t now = 0;
    Input in{0.50f, 0.70f, true, true, true, now, 100};
    now += 100;
    TEST_ASSERT_TRUE(inj.update(in).pulseStarted);  // running pulse
    in.nowMs = now;
    now += 100;
    in.highVetoBar = 1.70f;  // honest cell suddenly tells the truth
    const Output o = inj.update(in);
    TEST_ASSERT_FALSE_MESSAGE(o.solenoid, "valve kept firing after the veto tripped");
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_pulse_when_below_setpoint);
    RUN_TEST(test_no_pulse_within_deadband);
    RUN_TEST(test_early_release_above_offband);
    RUN_TEST(test_high_wall_and_cutoff_latch);
    RUN_TEST(test_duty_cap_limits_pulses);
    RUN_TEST(test_test_pulse_works_in_standby_but_walls_apply);
    RUN_TEST(test_test_pulse_requests_are_queued);
    RUN_TEST(test_output_fuse_forces_the_valve_off);
    RUN_TEST(test_unhealthy_sensing_refuses_injection);
    RUN_TEST(test_stuck_open_detection_and_lockout);
    RUN_TEST(test_descent_rise_is_not_stuck_open);
    RUN_TEST(test_stuck_open_window_survives_reading_dither);
    RUN_TEST(test_honest_cell_vetoes_injection_when_the_vote_lies);
    RUN_TEST(test_veto_also_holds_the_pulse_already_running);
    return UNITY_END();
}
