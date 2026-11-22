// ============================================================================
// test_loop — behaviour that only exists in the assembled chain: what happens
// between the modules (sensor staleness gating the valve, the output fuse, the
// pre-dive lockout, a spiked depth sample vs the setpoint bands).
// The fault scenarios in test_sim cover the same ground physically; these pin
// the interface contracts the firmware ControlTask depends on.
// ============================================================================
#include <unity.h>

#include "control/loop.h"

using namespace ctrl;

void setUp() {}
void tearDown() {}

namespace {

Config benchCfg() {
    Config c;
    c.presentMask = 0b111;
    c.predive.warmupMs = 1000;
    c.inj.maxDutyPct = 40;
    c.inj.minGapMs = 200;
    return c;
}

// Feeds the loop a fixed cell level / depth for `seconds`.
void feed(Loop& loop, uint32_t& now, float bar, float depth, float seconds,
          bool fresh = true) {
    const uint32_t n = static_cast<uint32_t>(seconds * 1000.0f / Loop::kTickMs);
    for (uint32_t i = 0; i < n; i++) {
        SensorData sd;
        for (int ch = 0; ch < 3; ch++) sd.cells[ch] = bar;
        sd.depthM = depth;
        sd.pressureValid = true;
        sd.cellsFresh = fresh;
        sd.vbat = 12.4f;
        now += Loop::kTickMs;
        loop.cycle(sd, now);
    }
}

// Tiny stand-in for the gas loop: every pulse that actually left the valve
// adds oxygen, the diver's metabolism and the bail-out take it back out.
// Deliberately crude — the physics lives in sim/world. This is only enough for
// the pre-dive sequence to see the rise (and the recovery) it checks for.
struct Plant {
    Loop& loop;
    float bar[3] = {0.21f, 0.21f, 0.21f};
    float base = 0.21f;
    float perPulse = 0.02f;
    float usePerS = 0.012f;
    uint8_t deafMask = 0;  // cells that never react (poisoned)
    uint32_t pulses = 0;

    explicit Plant(Loop& l) : loop(l) {}

    void tick(uint32_t& now, float depth, bool fresh = true) {
        const uint32_t p = loop.pulseCount();
        for (int ch = 0; ch < 3; ch++) {
            if (pulses < p && !(deafMask & (1u << ch))) bar[ch] += perPulse * (p - pulses);
            bar[ch] -= usePerS * (Loop::kTickMs / 1000.0f);
            if (bar[ch] < base) bar[ch] = base;  // bail-out keeps air at air
        }
        pulses = p;

        SensorData sd;
        for (int ch = 0; ch < 3; ch++) sd.cells[ch] = bar[ch];
        sd.depthM = depth;
        sd.pressureValid = true;
        sd.cellsFresh = fresh;
        sd.vbat = 12.4f;
        now += Loop::kTickMs;
        loop.cycle(sd, now);
    }

    void run(uint32_t& now, float depth, float seconds, bool fresh = true) {
        const uint32_t n = static_cast<uint32_t>(seconds * 1000.0f / Loop::kTickMs);
        for (uint32_t i = 0; i < n; i++) tick(now, depth, fresh);
    }
};

}  // namespace

// Safety rule the ControlTask relies on: a vote built from frozen samples is
// not a measurement, and the valve must stay shut on it.
void test_stale_sensors_stop_injection() {
    Loop loop(benchCfg());
    loop.setAutoMode(true);
    uint32_t now = 0;
    feed(loop, now, 0.30f, 0.0f, 3.0f);            // demand: 0.30 vs SP1 0.40
    const uint32_t pulsesBefore = loop.pulseCount();
    TEST_ASSERT_GREATER_THAN_UINT32(0u, pulsesBefore);

    feed(loop, now, 0.30f, 0.0f, 3.0f, /*fresh=*/false);
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(pulsesBefore, loop.pulseCount(),
                                     "injecting on stale readings");
    TEST_ASSERT_TRUE(loop.alarmBits() & (1u << alarms::BIT_SENSOR_STALE));
}

// The pin may not stay HIGH for longer than the fuse allows, whatever the
// logic above the driver asked for — and once it trips, injection stays off
// until the diver clears it.
void test_output_fuse_cuts_a_valve_that_will_not_close() {
    Config c = benchCfg();
    c.inj.pulseMs = 2000;  // a pulse longer than the fuse allows
    c.inj.maxOnMs = 500;
    Loop loop(c);
    loop.setAutoMode(true);

    bool fuseSeen = false, solAfterFuse = false;
    uint32_t now = 0;
    for (int i = 0; i < 40; i++) {
        SensorData sd;
        for (int ch = 0; ch < 3; ch++) sd.cells[ch] = 0.30f;
        sd.depthM = 0.0f;
        sd.pressureValid = true;
        sd.cellsFresh = true;
        sd.vbat = 12.4f;
        now += Loop::kTickMs;
        const State st = loop.cycle(sd, now);
        if (st.outputFuse) fuseSeen = true;
        if (fuseSeen && st.solenoid) solAfterFuse = true;
    }
    TEST_ASSERT_TRUE_MESSAGE(fuseSeen, "output fuse never tripped");
    TEST_ASSERT_FALSE_MESSAGE(solAfterFuse, "valve still driven after the fuse");
    TEST_ASSERT_TRUE(loop.alarmBits() & (1u << alarms::BIT_STUCK_OPEN));

    loop.clearStuckOpen();
    const uint32_t pulsesBefore = loop.pulseCount();
    feed(loop, now, 0.30f, 0.0f, 3.0f);
    TEST_ASSERT_GREATER_THAN_UINT32(pulsesBefore, loop.pulseCount());  // one clear path
}

// The hard lockout (PLAN 7): AUTO is a consequence of the checks, never a
// setting the diver can switch on first.
void test_auto_refused_until_pre_dive_passes() {
    Loop loop(benchCfg());
    Plant plant(loop);
    uint32_t now = 0;
    plant.run(now, 0.0f, 1.0f);
    TEST_ASSERT_FALSE_MESSAGE(loop.requestAuto(true), "AUTO granted before pre-dive");

    loop.prediveStart(now);
    plant.run(now, 0.0f, 15.0f);  // warmup + air + solenoid + depth + battery
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(predive::PASS, loop.prediveStatus(predive::S_SOLENOID),
                                    "solenoid/cell test did not pass on a healthy rig");
    TEST_ASSERT_TRUE_MESSAGE(loop.predivePassed(), "pre-dive did not complete");
    // Every pulse the test asked for must actually leave the valve: with a
    // one-slot request flag, gap/duty rules ate one and the verdict only came
    // from the test's own 20 s timeout.
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32_MESSAGE(3u, loop.pulseCount(),
                                                "test pulses lost on the way to the valve");
    TEST_ASSERT_TRUE(loop.requestAuto(true));
}

// A cell that ignores the oxygen the loop is provably receiving is a dead cell,
// and a dead cell must not be allowed to vote a diver into a hypoxic loop.
void test_unresponsive_cell_fails_the_cell_test() {
    Config c = benchCfg();
    Loop loop(c);
    Plant plant(loop);
    plant.deafMask = 0b100;  // cell 2 poisoned: passes the air check, hears nothing
    uint32_t now = 0;
    loop.prediveStart(now);
    plant.run(now, 0.0f, 15.0f);

    TEST_ASSERT_TRUE_MESSAGE(loop.prediveFailed(), "pulse test accepted a deaf cell");
    TEST_ASSERT_EQUAL_UINT8(predive::FAIL, loop.prediveStatus(predive::S_SOLENOID));
    TEST_ASSERT_FALSE(loop.requestAuto(true));
}

// Setpoint bands switch on absolute depth, so one spiked pressure sample must
// never push the loop into a richer setpoint by itself.
void test_depth_spike_does_not_move_the_setpoint() {
    Loop loop(benchCfg());
    uint32_t now = 0;
    feed(loop, now, 0.70f, 25.0f, 5.0f);
    const float spStable = loop.setpointBar();

    // Three samples, the middle one deep enough to reach the SP3 band (30 m).
    for (int i = 0; i < 3; i++) {
        SensorData sd;
        for (int ch = 0; ch < 3; ch++) sd.cells[ch] = 0.70f;
        sd.depthM = (i == 1) ? 45.0f : 25.0f;
        sd.pressureValid = true;
        sd.cellsFresh = true;
        sd.vbat = 12.4f;
        now += Loop::kTickMs;
        loop.cycle(sd, now);
        TEST_ASSERT_EQUAL_FLOAT_MESSAGE(spStable, loop.setpointBar(),
                                        "one spiked sample switched the setpoint band");
    }
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_stale_sensors_stop_injection);
    RUN_TEST(test_output_fuse_cuts_a_valve_that_will_not_close);
    RUN_TEST(test_auto_refused_until_pre_dive_passes);
    RUN_TEST(test_unresponsive_cell_fails_the_cell_test);
    RUN_TEST(test_depth_spike_does_not_move_the_setpoint);
    return UNITY_END();
}
