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


// The proof that ran on first descent is stale evidence about cells that fail
// later in the dive. The trigger is the machine's own: a cell it has REJECTED
// reading well above the vote it is trusting. Left alone, that state starves
// the loop — a lying pair above the setpoint means the valve never opens.
void test_proof_rearms_when_a_rejected_cell_reads_high() {
    Engine e{Config{}};
    uint32_t t = 0;
    e.update(mk(true, 5.0f, 0.70f, t));                    // armed on descent
    e.update(mk(true, 10.5f, 0.95f, t += 1000));           // running
    for (int i = 0; i < 10; ++i) e.update(mk(true, 10.5f, 1.02f, t += 500));
    TEST_ASSERT_EQUAL_UINT8(State::PASS, e.state());       // all three proved
    TEST_ASSERT_EQUAL_UINT8(1, e.attempts());

    // 20 s into the dive at 40 m: two cells frozen low, voting has dropped the
    // honest third, and the honest one is the only one saying 1.90.
    Input bad;
    bad.diveActive = true;
    bad.depthM = 40.0f;
    bad.cellsBar[0] = 1.20f; bad.cellsBar[1] = 1.20f; bad.cellsBar[2] = 1.90f;
    bad.votedBar = 1.20f;
    bad.activeMask = 0b011;
    bad.presentMask = 0b111;

    Output o = e.update(bad);
    TEST_ASSERT_EQUAL_UINT8(State::PASS, o.state);         // evidence, not verdict
    TEST_ASSERT_FALSE(o.requestSpOverride);

    for (int i = 0; i < 40; ++i) {                          // one whole sample window
        bad.nowMs = t += 500;
        o = e.update(bad);
    }
    TEST_ASSERT_EQUAL_UINT8(State::PASS, o.state);          // cooldown still holding
    t += 300000;                                            // past the re-arm cooldown
    for (int i = 0; i < 45; ++i) {
        bad.nowMs = t += 500;
        o = e.update(bad);
        if (o.state == State::RUNNING) break;
    }
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(State::RUNNING, o.state, "proof never re-armed");
    TEST_ASSERT_TRUE_MESSAGE(o.requestSpOverride, "re-arm did not raise the setpoint");
    TEST_ASSERT_EQUAL_UINT8(2, e.attempts());
}

// The other direction: the rejected cells read BELOW the vote. This is the
// shape that starves a loop — the vote stays high enough that the machine sees
// no reason to inject, and the reading it dropped was the one telling the truth.
void test_proof_rearms_when_a_rejected_cell_reads_low() {
    Engine e{Config{}};
    e.update(mk(true, 5.0f, 0.70f, 0));
    uint32_t t = 400000;                       // well past any cooldown
    e.update(mk(true, 10.5f, 0.95f, t));
    for (int i = 0; i < 10; ++i) e.update(mk(true, 10.5f, 1.02f, t += 500));
    TEST_ASSERT_EQUAL_UINT8(State::PASS, e.state());

    Input bad;
    bad.diveActive = true; bad.depthM = 28.0f; bad.presentMask = 0b111;
    bad.cellsBar[0] = 0.82f; bad.cellsBar[1] = 0.82f;   // the pair, now rejected
    bad.cellsBar[2] = 0.97f;                            // voting alone, tracks the loop
    bad.votedBar = 0.97f; bad.activeMask = 0b100;
    t = 800000;                          // the first proof is 400 s old: cooldown over
    Output o = e.update(bad);
    for (int i = 0; i < 45 && o.state != State::RUNNING; ++i) {
        bad.nowMs = t += 500;
        o = e.update(bad);
    }
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(State::RUNNING, o.state,
                                    "no re-arm when the rejected cells read low");
}

// The re-arm must not become a polling loop: attempts are capped per dive.
void test_rearm_attempts_are_capped() {
    Config c{};
    c.maxAttempts = 2;
    c.rearmCooldownMs = 0;
    c.suspectMs = 1000;
    Engine e{c};
    uint32_t t = 0;
    e.update(mk(true, 5.0f, 0.70f, t));
    e.update(mk(true, 10.5f, 0.95f, t += 1000));
    for (int i = 0; i < 10; ++i) e.update(mk(true, 10.5f, 1.02f, t += 500));
    TEST_ASSERT_EQUAL_UINT8(State::PASS, e.state());

    Input bad;
    bad.diveActive = true; bad.depthM = 40.0f;
    bad.cellsBar[0] = 1.20f; bad.cellsBar[1] = 1.20f; bad.cellsBar[2] = 1.90f;
    bad.votedBar = 1.20f; bad.activeMask = 0b011; bad.presentMask = 0b111;
    for (int round = 0; round < 4; ++round) {
        for (int i = 0; i < 20; ++i) { bad.nowMs = t += 500; e.update(bad); }
        for (int i = 0; i < 130; ++i) {                        // burn a whole run
            bad.votedBar = 1.20f; bad.nowMs = t += 500;
            e.update(bad);
        }
    }
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(2, e.attempts(), "proof re-armed past its cap");
}

// An all-or-nothing verdict lets a lying pair hide the honest cell: the vote
// never reaches the proof level, the run times out, and all three cells get
// blamed. The verdict is per cell, so the dive continues on the one that proved.
void test_timeout_verdict_blames_only_the_failed_cells() {
    Engine e{Config{}};                                        // 60 s timeout
    uint32_t t = 0;
    e.update(mk(true, 5.0f, 0.70f, t));
    e.update(mk(true, 10.5f, 0.95f, t += 1000));

    Input in;
    in.diveActive = true; in.depthM = 30.0f; in.presentMask = 0b111;
    in.activeMask = 0b011;
    for (int i = 0; i < 130; ++i) {                             // past the timeout
        in.cellsBar[0] = 0.85f; in.cellsBar[1] = 0.86f;         // frozen: never move
        in.cellsBar[2] = 0.90f + 0.004f * i;                    // honest: follows the loop
        in.votedBar = 0.85f;                                    // the lie wins
        in.nowMs = t += 500;
        e.update(in);
    }
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(State::PASS, e.state(),
                                    "proof failed all three cells, not just two");
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(0b011, e.update(in).unprovenHighMask,
                                    "the cell that proved is still marked unproven");
}

// A cell frozen at a comfortably high number must not vouch for itself just by
// sitting above minPass — it is not reading the loop, it is stuck.
void test_frozen_cell_is_not_proven_by_its_level() {
    Engine e{Config{}};
    uint32_t t = 0;
    e.update(mk(true, 5.0f, 0.70f, t));
    e.update(mk(true, 10.5f, 1.20f, t += 1000));
    Input in;
    in.diveActive = true; in.depthM = 40.0f; in.presentMask = 0b111;
    in.activeMask = 0b111;
    for (int i = 0; i < 40; ++i) {                              // loop sweeps up/down
        in.cellsBar[0] = 1.20f; in.cellsBar[1] = 1.20f;         // stuck at a plausible 1.20
        in.cellsBar[2] = 1.05f + 0.01f * i;                     // 1.05 .. 1.44
        in.votedBar = in.cellsBar[2];
        in.nowMs = t += 500;
        e.update(in);
    }
    TEST_ASSERT_EQUAL_UINT8(State::PASS, e.state());
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(0b011, e.update(in).unprovenHighMask,
                                    "frozen cells passed the proof on their level alone");
}

// The other half of the same rule: if nothing reached the proof level, the
// clamp is still correct — the loop may not be run at a high setpoint.
void test_timeout_with_no_proven_cell_still_fails() {
    Engine e{Config{}};
    uint32_t t = 0;
    e.update(mk(true, 5.0f, 0.70f, t));
    const Output o = e.update(mk(true, 10.5f, 0.85f, t += 1000));
    TEST_ASSERT_EQUAL_UINT8(State::RUNNING, o.state);
    for (int i = 0; i < 130; ++i) t += 500;
    const Output f = e.update(mk(true, 10.5f, 0.85f, t));
    TEST_ASSERT_EQUAL_UINT8(State::FAIL, f.state);
    TEST_ASSERT_TRUE(f.clampActive);
}

void test_disabled_and_dive_reset() {
    Config c; c.enabled = false;
    Engine e{c};
    const Output o = e.update(mk(true, 20.0f, 0.9f, 1000));
    TEST_ASSERT_EQUAL_UINT8(State::INACTIVE, o.state);
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
    RUN_TEST(test_proof_rearms_when_a_rejected_cell_reads_high);
    RUN_TEST(test_proof_rearms_when_a_rejected_cell_reads_low);
    RUN_TEST(test_rearm_attempts_are_capped);
    RUN_TEST(test_timeout_verdict_blames_only_the_failed_cells);
    RUN_TEST(test_frozen_cell_is_not_proven_by_its_level);
    RUN_TEST(test_timeout_with_no_proven_cell_still_fails);
    RUN_TEST(test_disabled_and_dive_reset);
    return UNITY_END();
}
