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

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_pulse_when_below_setpoint);
    RUN_TEST(test_no_pulse_within_deadband);
    RUN_TEST(test_early_release_above_offband);
    RUN_TEST(test_high_wall_and_cutoff_latch);
    RUN_TEST(test_duty_cap_limits_pulses);
    RUN_TEST(test_test_pulse_works_in_standby_but_walls_apply);
    RUN_TEST(test_unhealthy_sensing_refuses_injection);
    RUN_TEST(test_stuck_open_detection_and_lockout);
    RUN_TEST(test_descent_rise_is_not_stuck_open);
    return UNITY_END();
}
