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

// A live channel for the frozen-reading detector: ±0.0005 bar wobble (3.5 ADS
// counts p-p), wider than the 2-count eps the detector watches, quieter than
// anything the drift machinery notices. Feeding a constant instead is not a
// "calm" fixture — the engine is entitled to read it as a frozen channel.
float live(float bar, int i) { return bar + ((i % 2) ? 0.0005f : -0.0005f); }

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
    // cell 1 collapses to 0.04 in one 225 ms step (huge negative slope). The
    // target sits ABOVE the 0.02 plausibility floor on purpose: below it the
    // sample is not a collapse to be measured, it is nonsense to be skipped,
    // and the sustained-implausibility rule excludes that cell instead.
    feedRet(e, 0.40f, 0.04f, 0.41f);
    TEST_ASSERT_TRUE(last.dropoutEvent);
    TEST_ASSERT_FALSE(last.hypoxiaSuspect);
    TEST_ASSERT_FALSE(last.activeMask & 0x2);
    TEST_ASSERT_EQUAL_UINT8(Mode::DEGRADED_2CELL, last.mode);

    // exclusion is persistent: staying low does NOT re-admit it
    for (int i = 0; i < 20; ++i) feedRet(e, 0.40f, 0.04f, 0.41f);
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
    feedRet(e, 0.40f, 0.04f, 0.40f); // dropout 1 (0.04: plausible, just low)
    feedRet(e, 0.40f, 0.04f, 0.04f); // dropout 2
    TEST_ASSERT_EQUAL_UINT8(Mode::SINGLE, last.mode);
    TEST_ASSERT_FLOAT_WITHIN(0.001, 0.40, last.votedPpo2);

    feedRet(e, 0.04f, 0.04f, 0.04f); // last one collapses too: hypoxia path
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

// ---------------------------------------------------------------------------
// The latent bug this batch closes: an implausible sample was SKIPPED and the
// cell kept its seat in the vote on the last believable value — forever. These
// cases pin the terminal state.
// ---------------------------------------------------------------------------
void test_sustained_implausibility_excludes_the_cell() {
    Engine e{Config{}};
    feedRet(e, 0.40f, 0.41f, 0.42f);  // arm with plausible samples
    TEST_ASSERT_EQUAL_UINT8(0b111, last.activeMask);

    // Nonsense for longer than implausibleWindow (2 s of fresh samples).
    const uint8_t W = Config{}.implausibleWindow;
    for (uint8_t i = 0; i < W; ++i) feedRet(e, 0.40f, 9.0f, 0.42f);
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(0b101, last.activeMask,
                                    "a cell reading nonsense for the whole window still votes");
    TEST_ASSERT_EQUAL_UINT8(Engine::BAR_IMPLAUSIBLE, e.barReason(1));
    TEST_ASSERT_TRUE_MESSAGE(last.disagreement, "the exclusion was not reported");
    // cellsSeen stays true: the OTHER cells answered; that is what turns the
    // alarm layer's FAILED into a shouted fault rather than a boot-time shrug.
    TEST_ASSERT_TRUE(last.cellsSeen);
    // The vote itself is unaffected while two honest cells remain.
    TEST_ASSERT_EQUAL_UINT8(Mode::DEGRADED_2CELL, last.mode);
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.01, 0.41, last.votedPpo2,
                                     "the vote moved when it should have ignored the nonsense");

    // Hysteresis return: plausible and agreeing again. The bar parked
    // driftStrikes at the cap (dropout-style persistence), so the full
    // half-band walk back is driftWindow clean samples.
    for (int i = 0; i < Config{}.driftWindow + 2; ++i)
        feedRet(e, live(0.40f, i), live(0.41f, i + 1), live(0.42f, i + 2));
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(0b111, last.activeMask, "a recovered cell never came back");
    TEST_ASSERT_EQUAL_UINT8(Engine::BAR_NONE, e.barReason(1));
}

void test_implausible_bursts_alone_never_exclude() {
    Engine e{Config{}};
    feedRet(e, 0.40f, 0.41f, 0.42f);
    // Alternating glitch/plausible: never implausibleWindow CONSECUTIVE, so the
    // streak rule must stay out of it — glitches belong to the skip path.
    for (int i = 0; i < 60; ++i)
        feedRet(e, 0.40f, (i % 2 == 0) ? 1.9f + 0.5f : 0.41f, 0.42f);
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(0b111, last.activeMask,
                                    "intermittent glitches were treated as a sustained fault");
    TEST_ASSERT_EQUAL_UINT8(Engine::BAR_NONE, e.barReason(1));
    TEST_ASSERT_TRUE(e.cell(1).glitchCount > 0);  // the glitches were still counted
}

void test_three_dead_cells_cannot_vote_on_a_fossil() {
    // The maintainer's shape: every channel reads below the true-dead floor.
    // Before this batch these were simply SKIPPED — the vote kept running on
    // the last values from before the death, cellsSeen stayed true, and the
    // machine drove the valve against a fossil.
    Engine e{Config{}};
    feedRet(e, 0.40f, 0.41f, 0.42f);
    const float fossil = last.votedPpo2;
    TEST_ASSERT_GREATER_THAN_FLOAT(0.3f, fossil);

    for (int i = 0; i < Config{}.implausibleWindow + 5; ++i)
        feedRet(e, 0.01f, 0.01f, 0.01f);  // 0.01 < minPpo2 (0.02): true-dead
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(Mode::FAILED, last.mode,
                                    "dead cells kept a vote on their last believable value");
    TEST_ASSERT_EQUAL_UINT8(0, last.activeMask);
    TEST_ASSERT_TRUE_MESSAGE(last.cellsSeen,
                             "cellsSeen must stay true: this is blindness, not boot");
    TEST_ASSERT_TRUE(last.disagreement);
    for (int i = 0; i < 3; ++i) TEST_ASSERT_EQUAL_UINT8(Engine::BAR_IMPLAUSIBLE, e.barReason(i));
    // And the fossil is not handed back on the last-resort tier: FAILED means
    // FAILED, with a reason and an alarm path, not a secret min-of-nothing.
    for (int i = 0; i < 10; ++i) {
        feedRet(e, 0.01f, 0.01f, 0.01f);
        TEST_ASSERT_EQUAL_UINT8(Mode::FAILED, last.mode);
        TEST_ASSERT_EQUAL_FLOAT(0.0f, last.votedPpo2);
    }
}

void test_a_barred_nonsense_cell_is_not_fallback_fuel() {
    // The documented fallback tier: when everything else is gone, a cell out on
    // DRIFT alone votes (its tracked reading is still evidence). A cell whose
    // recent samples were all NONSENSE votes on a fossil — that is exactly the
    // hole this batch closes, and the fallback must not be a side door.
    Engine e{Config{}};
    for (int i = 0; i < 5; ++i) feedRet(e, live(0.40f, i), live(0.56f, i), 0.40f);
    for (int i = 0; i < Config{}.driftWindow + 25; ++i)
        feedRet(e, live(0.40f, i), live(0.56f, i), 9.0f);  // 1 drifts out, 2 drowns
    TEST_ASSERT_EQUAL_UINT8(Engine::BAR_DRIFT, e.barReason(1));
    TEST_ASSERT_EQUAL_UINT8(Engine::BAR_IMPLAUSIBLE, e.barReason(2));

    // Take the healthy cell and the drift cell out of play (proof evidence,
    // externally owned): the ONLY candidate left for the last-resort tier is
    // the cell whose every recent sample was nonsense. It must not vote.
    e.setProofExclusion(0b011);
    feedRet(e, live(0.40f, 1), live(0.56f, 1), 9.0f);
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(Mode::FAILED, last.mode,
                                    "the implausible cell carried the vote through the fallback tier");
    TEST_ASSERT_EQUAL_UINT8(0, last.activeMask);
}

// ---------------------------------------------------------------------------
// Frozen-reading (stuck cell) detection. Epsilon is bench-dither-sized
// (2 ADC counts), the window is 20 s of samples — see voting.h Config.
// ---------------------------------------------------------------------------

// A live chain, even a perfectly quiescent one, dithers. Feed one forever and
// the detector must stay silent. This is the false-positive gate.
void test_a_dithering_quiescent_loop_is_never_accused() {
    Engine e{Config{}};
    // Triangle dither of ±0.0005 bar (3.5 counts p-p): a loop that is DEAD
    // CALM, a chain that is alive. 60 s — three full detector windows.
    for (int i = 0; i < 600; ++i) {  // > two full stuckWindows of samples
        const float d = ((i / 5) % 2 == 0) ? 0.0005f : -0.0005f;
        feedRet(e, 0.40f + d, 0.41f - d, 0.42f + d);
        TEST_ASSERT_EQUAL_UINT8_MESSAGE(0b111, last.activeMask,
                                        "the stuck detector condemned a live, quiescent chain");
    }
    TEST_ASSERT_EQUAL_UINT8(Engine::BAR_NONE, e.barReason(0));
    TEST_ASSERT_FALSE(last.disagreement);
}

void test_a_frozen_plausible_reading_is_accused_and_held() {
    Engine e{Config{}};
    // Live first: all three dither for a full stuckWindow+, nothing is accused.
    const uint16_t W = Config{}.stuckWindow;
    for (int i = 0; i < W + 20; ++i)
        feedRet(e, live(0.40f, i), live(0.41f, i + 1), live(0.42f, i + 2));
    TEST_ASSERT_EQUAL_UINT8(0b111, last.activeMask);

    // Channel 0 freezes bit-identical while the others keep living (a connector
    // latched, not a cell poisoned: the reading never moves at all). The band
    // breaks once at the transition and then just counts: one stuckWindow of
    // stillness, and it is accused.
    for (int i = 0; i < W; ++i)
        feedRet(e, 0.40f, live(0.41f, i + 1), live(0.42f, i + 2));
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(0b110, last.activeMask,
                                    "a reading frozen for a full window kept its vote");
    TEST_ASSERT_EQUAL_UINT8(Engine::BAR_STUCK, e.barReason(0));
    TEST_ASSERT_TRUE(last.disagreement);
    TEST_ASSERT_EQUAL_UINT8(0, e.cell(0).health);  // the display may not call it healthy

    // Still frozen, second window: no thrash, and no sneaking back through the
    // drift hysteresis just because 0.40 sits right on the median — only the
    // channel's OWN movement refutes the accusation.
    for (int i = 0; i < W; ++i)
        feedRet(e, 0.40f, live(0.41f, i + 1), live(0.42f, i + 2));
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(0b110, last.activeMask,
                                    "a frozen cell rode the drift hysteresis back into the vote");
    TEST_ASSERT_EQUAL_UINT8(Engine::BAR_STUCK, e.barReason(0));

    // It thaws: one moving sample breaks the band and lifts the bar on the
    // spot; the VOTE return still waits for the drift hysteresis (thaw proves
    // liveness, not trustworthiness).
    for (int i = 0; i < 5; ++i)
        feedRet(e, live(0.40f, i), live(0.41f, i + 1), live(0.42f, i + 2));
    TEST_ASSERT_EQUAL_UINT8(Engine::BAR_NONE, e.barReason(0));
    for (int i = 0; i < Config{}.driftWindow + 2; ++i)
        feedRet(e, live(0.40f, i), live(0.41f, i + 1), live(0.42f, i + 2));
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(0b111, last.activeMask, "a thawed cell never returned");
}

void test_all_three_frozen_fails_the_vote_with_cells_seen() {
    // The maintainer's headline shape: three dead channels reading a PLAUSIBLE
    // value. Plausibility, slope and drift all pass; the vote must still die —
    // by evidence — with cellsSeen true so the alarm layer shouts.
    Engine e{Config{}};
    for (int i = 0; i < 20; ++i)
        feedRet(e, live(0.40f, i), live(0.41f, i + 1), live(0.42f, i + 2));
    TEST_ASSERT_EQUAL_UINT8(Mode::OK_3CELL, last.mode);

    // Every channel frozen, all plausible, for a full stuckWindow + the one
    // transition sample.
    for (int i = 0; i < Config{}.stuckWindow + 2; ++i)
        feedRet(e, 0.40f, 0.41f, 0.42f);
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(Mode::FAILED, last.mode,
                                    "three frozen readings kept the vote alive");
    TEST_ASSERT_TRUE_MESSAGE(last.cellsSeen, "FAILED must read as blindness, not boot");
    TEST_ASSERT_TRUE(last.disagreement);
    for (int i = 0; i < 3; ++i) TEST_ASSERT_EQUAL_UINT8(Engine::BAR_STUCK, e.barReason(i));
}

void test_stale_samples_pause_the_frozen_detector() {
    // A dead ADC stream is SENSOR_STALE's claim (sensor layer, age/ageMs), not
    // a cell claim. With the source not fresh, identical readings must NOT be
    // attributed to the cells.
    Engine e{Config{}};
    for (int i = 0; i < 20; ++i)
        feedRet(e, live(0.40f, i), live(0.41f, i + 1), live(0.42f, i + 2));
    e.setSampleSourceLive(false);
    for (int i = 0; i < Config{}.stuckWindow * 2; ++i)
        feedRet(e, 0.40f, 0.41f, 0.42f);  // bit-identical, but NOT fresh
    TEST_ASSERT_EQUAL_UINT8(Engine::BAR_NONE, e.barReason(0));
    TEST_ASSERT_EQUAL_UINT8(0b111, last.activeMask);
}

// ---------------------------------------------------------------------------
// Responsiveness: measured here now (Loop reports the pulse count), RELATIVE,
// advisory only. A cell may score 0 and keep its vote — exclusion is the
// detector's vocabulary, not this score's.
// ---------------------------------------------------------------------------
void test_responsiveness_scores_cells_against_their_peers() {
    Engine e{Config{}};
    e.notePulses(0);
    // One score window (20 s of feeds at this test's 225 ms cadence ~ 89
    // samples): cells 1 and 2 climb across it, cell 0 climbs a THIRD as much —
    // movement above eps (so NOT frozen) but clearly sluggish next to peers.
    const int CYCLES = Config{}.stuckWindow * 100 / static_cast<int>(DT) + 2;
    for (int i = 0; i < CYCLES; ++i) {
        feedRet(e, 0.40f + 0.00002f * i, 0.40f + 0.00006f * i, 0.41f + 0.00006f * i);
        e.notePulses(static_cast<uint32_t>(i / 3));  // the valve fired
    }
    TEST_ASSERT_LESS_THAN_FLOAT_MESSAGE(0.7f, e.cell(0).responsiveness,
                                        "a sluggish cell scored responsive while peers moved");
    TEST_ASSERT_GREATER_THAN_FLOAT_MESSAGE(0.9f, e.cell(1).responsiveness,
                                           "a responsive cell scored dead");
    // Advisory only: cell 0 keeps its seat (it moved, just less).
    TEST_ASSERT_TRUE_MESSAGE(last.activeMask & 0b001,
                             "a low responsiveness score excluded a cell — that is not its job");

    // A window with NO pulses: movement happened, but nothing was dosed, so no
    // cell can be credited or blamed. The score must not be silently reset to
    // full marks either — the last evidence stands.
    const float held = e.cell(0).responsiveness;
    const uint32_t lastPulses = static_cast<uint32_t>((CYCLES - 1) / 3);
    for (int i = 0; i < CYCLES + 2; ++i) {
        feedRet(e, 0.42f + 0.00004f * i, 0.43f + 0.00004f * i, 0.44f + 0.00004f * i);
        e.notePulses(lastPulses);  // same count every cycle: this window saw NO pulse
    }
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.001f, held, e.cell(0).responsiveness,
                                     "responsiveness changed in a window with no pulses");
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
    RUN_TEST(test_sustained_implausibility_excludes_the_cell);
    RUN_TEST(test_implausible_bursts_alone_never_exclude);
    RUN_TEST(test_three_dead_cells_cannot_vote_on_a_fossil);
    RUN_TEST(test_a_barred_nonsense_cell_is_not_fallback_fuel);
    RUN_TEST(test_a_dithering_quiescent_loop_is_never_accused);
    RUN_TEST(test_a_frozen_plausible_reading_is_accused_and_held);
    RUN_TEST(test_all_three_frozen_fails_the_vote_with_cells_seen);
    RUN_TEST(test_stale_samples_pause_the_frozen_detector);
    RUN_TEST(test_responsiveness_scores_cells_against_their_peers);
    return UNITY_END();
}
