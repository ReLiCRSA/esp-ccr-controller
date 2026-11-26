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

// CellProof evidence has to bind the vote, and it has to survive the drift
// layer's own re-admission: a cell the machine has proof against may not slide
// back in because it happened to agree for a few samples.
void test_proof_exclusion_outranks_drift_hysteresis() {
    Engine e{Config{}};
    for (int i = 0; i < 5; ++i) feedRet(e, 1.20f, 1.20f, 1.45f);
    e.setProofExclusion(0b011);
    feedRet(e, 1.20f, 1.20f, 1.45f);
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(0b100, last.activeMask, "banned cells still voting");
    TEST_ASSERT_FLOAT_WITHIN(0.005, 1.45, last.votedPpo2);
    TEST_ASSERT_EQUAL_UINT8(Mode::SINGLE, last.mode);
    TEST_ASSERT_TRUE(last.disagreement);

    for (int i = 0; i < 30; ++i) feedRet(e, 1.20f, 1.20f, 1.45f);  // steady agreement
    TEST_ASSERT_EQUAL_UINT8(0b100, last.activeMask);               // still banned

    e.setProofExclusion(0);                                        // re-proved
    for (int i = 0; i < 15; ++i) feedRet(e, 1.20f, 1.20f, 1.20f);  // readings agree again
    TEST_ASSERT_EQUAL_UINT8(0b111, last.activeMask);               // redundancy back
}

// Banning cells must never leave the machine blind: if the ban empties the
// trusted set, a cell excluded on drift alone votes instead. No vote means no
// injection, so "no vote" has to be the last resort, not the first.
void test_ban_that_empties_the_vote_falls_back_to_a_drifted_cell() {
    Engine e{Config{}};
    for (int i = 0; i < 20; ++i) feedRet(e, 0.82f, 0.82f, 1.45f);  // drift drops cell 2
    TEST_ASSERT_EQUAL_UINT8(0b011, last.activeMask);

    e.setProofExclusion(0b011);   // the proof says the pair is the one lying
    feedRet(e, 0.82f, 0.82f, 1.45f);
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(0b100, last.activeMask,
                                    "ban left the machine with no vote at all");
    TEST_ASSERT_FLOAT_WITHIN(0.005, 1.45, last.votedPpo2);
    TEST_ASSERT_TRUE(last.disagreement);   // and the diver is still told why
}

void test_health_scores_degrade_for_bad_cells() {
    Engine e{Config{}};
    feed(e, 0.40f, 0.40f, 0.40f);
    e.setResponsiveness(2, 0.0f); // ignores injection pulses
    for (int i = 0; i < 30; ++i) feed(e, 0.40f, 0.40f, 0.55f);
    TEST_ASSERT_TRUE(e.cell(2).health < e.cell(0).health);
    TEST_ASSERT_TRUE(e.cell(0).health >= 90);
}

// Result::cellsSeen is what lets the alarm layer tell "the machine has never
// sampled" from "it can no longer see the loop": Mode::FAILED is the report in
// both cases, and only one of them is a fault the diver has to hear about
// (alarms.cpp, finding C2). Pinned here, at the layer that owns the fact.
void test_cells_seen_distinguishes_boot_from_a_lost_vote() {
    Engine e{Config{}};
    feedRet(e, 9.0f, 9.0f, 9.0f);  // every reading out of the plausible band
    TEST_ASSERT_EQUAL_UINT8(Mode::FAILED, last.mode);
    TEST_ASSERT_FALSE_MESSAGE(last.cellsSeen, "boot claimed cell data it never had");

    feedRet(e, 0.40f, 9.0f, 9.0f);  // one believable answer lands
    TEST_ASSERT_TRUE(last.cellsSeen);
    TEST_ASSERT_EQUAL_UINT8(Mode::SINGLE, last.mode);

    e.setProofExclusion(0b001);  // and the proof bans the only witness: FAILED again
    feedRet(e, 0.40f, 9.0f, 9.0f);
    TEST_ASSERT_EQUAL_UINT8(Mode::FAILED, last.mode);
    TEST_ASSERT_TRUE_MESSAGE(last.cellsSeen,
                             "a lost vote is indistinguishable from a boot");
}

// The claim is about FITTED cells: one the bench uninstalled takes its history
// with it, and must not keep "we have data" alive for a vote it left.
void test_unfitting_a_cell_forgets_its_history() {
    Engine e{Config{}};
    feedRet(e, 0.40f, 9.0f, 9.0f);  // only cell 0 has ever answered
    TEST_ASSERT_TRUE(last.cellsSeen);
    e.setPresentMask(0b110);  // cell 0 comes off the rig
    feedRet(e, 9.0f, 9.0f, 9.0f);
    TEST_ASSERT_EQUAL_UINT8(Mode::FAILED, last.mode);
    TEST_ASSERT_FALSE_MESSAGE(last.cellsSeen,
                              "an uninstalled cell kept the data claim alive");
}

// Finding N2's other half, pinned at the layer that owns it: a failed vote
// reports 0.00 ppO2 and keeps doing so. The held number the diver is shown lives
// in ctrl::Loop (loop.cpp step 2b), NOT here, precisely so that no consumer can
// pick up the last believable value from Result and treat it as this cycle's
// measurement. A "helpful" Engine that returned the last vote on FAILED would put
// a stale ppO2 back into the injector, the level alarms and the tissue model, and
// the compiler would never mention it.
void test_a_failed_vote_reports_zero_and_keeps_doing_it() {
    Engine e{Config{}};
    feedRet(e, 1.20f, 1.21f, 1.19f);
    TEST_ASSERT_EQUAL_UINT8(Mode::OK_3CELL, last.mode);
    TEST_ASSERT_GREATER_THAN_FLOAT_MESSAGE(1.0f, last.votedPpo2,
                                           "this case assumes a healthy vote first");
    const float believable = last.votedPpo2;

    e.setProofExclusion(0b111);  // every fitted cell banned: nothing may vote
    feedRet(e, 1.20f, 1.21f, 1.19f);
    TEST_ASSERT_EQUAL_UINT8(Mode::FAILED, last.mode);
    TEST_ASSERT_EQUAL_FLOAT_MESSAGE(0.0f, last.votedPpo2,
                                    "a failed vote returned a ppO2: consumers gated on haveVote "
                                    "would be handed a number that is not a measurement");
    TEST_ASSERT_GREATER_THAN_FLOAT_MESSAGE(0.0f, believable,
                                           "the healthy vote was already 0, so the assert above "
                                           "could not tell a held value from a real one");

    // And it stays 0 for as long as the vote stays failed: the failure is not a
    // one-cycle blip that leaks a value on the second cycle.
    for (int i = 0; i < 20; i++) {
        feedRet(e, 1.20f, 1.21f, 1.19f);
        TEST_ASSERT_EQUAL_UINT8(Mode::FAILED, last.mode);
        TEST_ASSERT_EQUAL_FLOAT(0.0f, last.votedPpo2);
    }
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
    RUN_TEST(test_proof_exclusion_outranks_drift_hysteresis);
    RUN_TEST(test_ban_that_empties_the_vote_falls_back_to_a_drifted_cell);
    RUN_TEST(test_cells_seen_distinguishes_boot_from_a_lost_vote);
    RUN_TEST(test_unfitting_a_cell_forgets_its_history);
    RUN_TEST(test_a_failed_vote_reports_zero_and_keeps_doing_it);
    return UNITY_END();
}
