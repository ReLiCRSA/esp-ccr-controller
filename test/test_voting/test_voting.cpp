// Unit tests for the ppO2 cell voting engine.
#include <unity.h>

#include "control/voting.h"

using voting::Config;
using voting::Engine;
using voting::Mode;

static constexpr uint32_t DT = 225; // ms between updates (matches ~15 Hz/3ch)

void setUp() {}
void tearDown() {}

static void feed(Engine& e, float a, float b, float c, uint32_t dt = DT) {
    const float v[3] = {a, b, c};
    e.update(v, dt);
}

static voting::Result last;
static void feedRet(Engine& e, float a, float b, float c) {
    const float v[3] = {a, b, c};
    last = e.update(v, DT);
}

void test_median_of_three_rejects_outlier_after_window() {
    Engine e{Config{}};
    feed(e, 0.40f, 0.41f, 0.42f); // arm with plausible samples
    feedRet(e, 0.40f, 0.41f, 0.90f);
    TEST_ASSERT_FLOAT_WITHIN(0.005, 0.41, last.votedPpo2);
    TEST_ASSERT_EQUAL_UINT8(Mode::OK_3CELL, last.mode);

    for (int i = 0; i < Config{}.driftWindow; ++i) feedRet(e, 0.40f, 0.41f, 0.90f);
    TEST_ASSERT_FALSE(last.activeMask & 0x4); // cell 2 excluded
    TEST_ASSERT_EQUAL_UINT8(Mode::DEGRADED_2CELL, last.mode);
    TEST_ASSERT_TRUE(last.disagreement);
    TEST_ASSERT_FLOAT_WITHIN(0.005, 0.40, last.votedPpo2); // min of 2
}

void test_bench_two_cell_uses_conservative_min() {
    Engine e{Config{}};
    e.setPresentMask(0b011);
    feed(e, 0.40f, 0.55f, 0.0f);
    feedRet(e, 0.40f, 0.55f, 0.0f);
    TEST_ASSERT_EQUAL_UINT8(Mode::DEGRADED_2CELL, last.mode);
    TEST_ASSERT_FLOAT_WITHIN(0.001, 0.40, last.votedPpo2); // min, not mean
    TEST_ASSERT_EQUAL_UINT8(0b011, last.activeMask);       // A2 ignored entirely
}

void test_single_cell_vertical_collapse_is_sensor_fault() {
    Engine e{Config{}};
    feed(e, 0.40f, 0.40f, 0.41f);
    // cell 1 collapses to 0.01 in one 225 ms step (huge negative slope)
    feedRet(e, 0.40f, 0.01f, 0.41f);
    TEST_ASSERT_TRUE(last.dropoutEvent);
    TEST_ASSERT_FALSE(last.hypoxiaSuspect);
    TEST_ASSERT_FALSE(last.activeMask & 0x2);
    TEST_ASSERT_EQUAL_UINT8(Mode::DEGRADED_2CELL, last.mode);

    // exclusion is persistent: staying low does NOT re-admit it
    for (int i = 0; i < 20; ++i) feedRet(e, 0.40f, 0.02f, 0.41f);
    TEST_ASSERT_FALSE(last.activeMask & 0x2);
}

void test_all_cells_collapsing_is_hypoxia_not_sensor_fault() {
    Engine e{Config{}};
    feed(e, 0.40f, 0.40f, 0.40f);
    feedRet(e, 0.20f, 0.20f, 0.20f); // everyone falls together
    TEST_ASSERT_TRUE(last.hypoxiaSuspect);
    TEST_ASSERT_FALSE(last.dropoutEvent);
    TEST_ASSERT_EQUAL_UINT8(0b111, last.activeMask); // keep trusting the loop
}

void test_glitch_sample_skipped_not_voted() {
    Engine e{Config{}};
    feed(e, 0.40f, 0.40f, 0.40f);
    feedRet(e, 0.40f, 1.60f, 0.40f); // 1.2 bar teleport = implausible
    TEST_ASSERT_FLOAT_WITHIN(0.001, 0.40, last.votedPpo2);
    TEST_ASSERT_EQUAL_UINT8(0b111, last.activeMask);
    TEST_ASSERT_EQUAL_UINT16(1, e.cell(1).glitchCount);
}

void test_drifted_cell_excluded_then_returns_with_hysteresis() {
    Engine e{Config{}};
    feed(e, 0.40f, 0.40f, 0.40f);
    const uint8_t W = Config{}.driftWindow;
    for (int i = 0; i < W + 1; ++i) feedRet(e, 0.40f, 0.40f, 0.56f);
    TEST_ASSERT_FALSE(e.cell(2).active);

    // one clean sample must NOT re-admit instantly (no flapping)
    feedRet(e, 0.40f, 0.40f, 0.41f);
    TEST_ASSERT_FALSE(e.cell(2).active);

    // W more clean samples: strike drains and the cell rejoins (state),
    // effective in the reported vote one cycle later
    for (int i = 0; i < W; ++i) feedRet(e, 0.40f, 0.40f, 0.41f);
    TEST_ASSERT_TRUE(e.cell(2).active);
    feedRet(e, 0.40f, 0.40f, 0.41f);
    TEST_ASSERT_TRUE(last.activeMask & 0x4);
    TEST_ASSERT_EQUAL_UINT8(Mode::OK_3CELL, last.mode);
}

void test_lone_cell_reports_single_mode() {
    Engine e{Config{}};
    feed(e, 0.40f, 0.40f, 0.40f);
    feedRet(e, 0.40f, 0.01f, 0.40f); // dropout 1
    feedRet(e, 0.40f, 0.01f, 0.01f); // dropout 2
    TEST_ASSERT_EQUAL_UINT8(Mode::SINGLE, last.mode);
    TEST_ASSERT_FLOAT_WITHIN(0.001, 0.40, last.votedPpo2);

    feedRet(e, 0.01f, 0.01f, 0.01f); // last one collapses too: hypoxia path
    TEST_ASSERT_TRUE(last.hypoxiaSuspect);
}

void test_health_scores_degrade_for_bad_cells() {
    Engine e{Config{}};
    feed(e, 0.40f, 0.40f, 0.40f);
    e.setResponsiveness(2, 0.0f); // ignores injection pulses
    for (int i = 0; i < 30; ++i) feed(e, 0.40f, 0.40f, 0.55f);
    TEST_ASSERT_TRUE(e.cell(2).health < e.cell(0).health);
    TEST_ASSERT_TRUE(e.cell(0).health >= 90);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_median_of_three_rejects_outlier_after_window);
    RUN_TEST(test_bench_two_cell_uses_conservative_min);
    RUN_TEST(test_single_cell_vertical_collapse_is_sensor_fault);
    RUN_TEST(test_all_cells_collapsing_is_hypoxia_not_sensor_fault);
    RUN_TEST(test_glitch_sample_skipped_not_voted);
    RUN_TEST(test_drifted_cell_excluded_then_returns_with_hysteresis);
    RUN_TEST(test_lone_cell_reports_single_mode);
    RUN_TEST(test_health_scores_degrade_for_bad_cells);
    return UNITY_END();
}
