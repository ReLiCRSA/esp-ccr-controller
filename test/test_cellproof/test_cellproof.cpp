#include <unity.h>

#include "control/cellproof.h"

using cellproof::Config;
using cellproof::Engine;
using cellproof::Input;
using cellproof::Output;
using cellproof::State;

void setUp() {}
void tearDown() {}

static Input mk(bool dive, float depth, float voted, uint32_t now) {
    Input in;
    in.diveActive = dive; in.depthM = depth; in.votedBar = voted;
    in.cellsBar[0] = voted + 0.01f; in.cellsBar[1] = voted; in.cellsBar[2] = voted - 0.01f;
    in.activeMask = 0b111; in.nowMs = now;
    return in;
}

void test_pass_path_first_descent() {
    Engine e{Config{}};
    uint32_t t = 10000;
    Output o = e.update(mk(true, 5.0f, 0.70f, t));  // armed
    TEST_ASSERT_EQUAL_UINT8(State::ARMED, o.state);

    o = e.update(mk(true, 10.5f, 0.95f, ++t));      // crossed 10 m: running
    TEST_ASSERT_EQUAL_UINT8(State::RUNNING, o.state);
    TEST_ASSERT_TRUE(o.requestSpOverride);

    for (int i = 0; i < 10; ++i) { e.update(mk(true, 10.5f, 1.02f, t += 500)); }
    TEST_ASSERT_EQUAL_UINT8(State::PASS, e.state());
    TEST_ASSERT_EQUAL_UINT8(0, e.update(mk(true, 10.5f, 1.05f, t)).unprovenHighMask);

    // no second attempt this dive even after another crossing
    o = e.update(mk(true, 11.0f, 0.50f, t += 1000));
    TEST_ASSERT_EQUAL_UINT8(State::PASS, o.state);
    TEST_ASSERT_FALSE(o.requestSpOverride);
}

void test_fail_clamps_until_dive_end_or_override() {
    Engine e{Config{}}; // 60 s timeout
    uint32_t t = 0;
    e.update(mk(true, 5.0f, 0.70f, t));
    e.update(mk(true, 10.5f, 0.85f, t += 1000));    // running, cells stuck ~0.85
    for (int i = 0; i < 130; ++i) t += 500;          // burn past 60 s
    const Output o = e.update(mk(true, 10.5f, 0.85f, t));
    TEST_ASSERT_EQUAL_UINT8(State::FAIL, o.state);
    TEST_ASSERT_TRUE(o.clampActive);

    const Output o2 = e.update(mk(true, 12.0f, 0.85f, t += 1000));
    TEST_ASSERT_TRUE(o2.clampActive);                // persists deeper

    e.override();                                    // diver judgement (logged)
    const Output o3 = e.update(mk(true, 12.0f, 0.86f, t += 1000));
    TEST_ASSERT_FALSE(o3.clampActive);
    TEST_ASSERT_TRUE(e.overridden());
}

void test_unproven_high_cells_flagged() {
    Engine e{Config{}};
    uint32_t t = 0;
    e.update(mk(true, 5.0f, 0.70f, t));
    e.update(mk(true, 10.5f, 0.95f, t += 1000));
    Input good = mk(true, 10.5f, 1.05f, t += 5000);
    good.cellsBar[2] = 0.82f;   // cell 3 saturated low while others prove out
    e.update(good);
    TEST_ASSERT_EQUAL_UINT8(State::PASS, e.state());
    TEST_ASSERT_EQUAL_UINT8(0b100, e.update(mk(true, 10.5f, 1.05f, t)).unprovenHighMask);
}

void test_disabled_and_dive_reset() {
    Config c; c.enabled = false;
    Engine e{c};
    const Output o = e.update(mk(true, 20.0f, 0.9f, 1000));
    TEST_ASSERT_EQUAL_UINT8(State::DISABLED, o.state);
    TEST_ASSERT_FALSE(o.requestSpOverride);

    Engine e2{Config{}};
    uint32_t t = 0;
    e2.update(mk(true, 10.5f, 0.60f, t)); // runs (surface-arm skipped: arm at depth? see note)
    e2.update(mk(false, 0.0f, 0.21f, t += 1000)); // dive end resets
    TEST_ASSERT_EQUAL_UINT8(State::IDLE, e2.state());
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_pass_path_first_descent);
    RUN_TEST(test_fail_clamps_until_dive_end_or_override);
    RUN_TEST(test_unproven_high_cells_flagged);
    RUN_TEST(test_disabled_and_dive_reset);
    return UNITY_END();
}
