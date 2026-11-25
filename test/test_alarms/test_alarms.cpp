#include <unity.h>

#include "control/alarms.h"

using alarms::Config;
using alarms::Manager;
using alarms::Inputs;
using alarms::Result;

void setUp() {}
void tearDown() {}

static Inputs base() {
    Inputs in;
    in.haveVote = true;
    in.votedBar = 0.55f;
    in.vbat = 12.4f;
    in.vote.mode = voting::Mode::OK_3CELL;
    return in;
}

void test_low_ppo2_edge_pair_and_critical_level() {
    Manager m{Config{}};
    Inputs in = base();
    Result r0 = m.update(in); // healthy baseline
    TEST_ASSERT_EQUAL_UINT16(0, r0.bits);
    TEST_ASSERT_EQUAL_UINT8(0, r0.edgeCount);

    in.votedBar = 0.15f;
    Result r1 = m.update(in);
    TEST_ASSERT_TRUE(r1.bits & (1u << alarms::BIT_LOW_PPO2));
    TEST_ASSERT_TRUE(r1.anyCritical);
    TEST_ASSERT_EQUAL_UINT8(1, r1.edgeCount);
    TEST_ASSERT_TRUE(r1.edges[0].raised);
    TEST_ASSERT_EQUAL_UINT8(alarms::BIT_LOW_PPO2, r1.edges[0].id);

    in.votedBar = 0.55f;
    Result r2 = m.update(in);
    TEST_ASSERT_EQUAL_UINT16(0, r2.bits);
    TEST_ASSERT_EQUAL_UINT8(1, r2.edgeCount);
    TEST_ASSERT_FALSE(r2.edges[0].raised); // cleared edge
}

void test_ack_suppresses_continuous_until_recovery_then_rearms() {
    Manager m{Config{}};
    Inputs in = base();
    in.votedBar = 0.10f;
    TEST_ASSERT_TRUE(m.update(in).bits & (1u << alarms::BIT_LOW_PPO2));

    m.ack();
    Result s = m.update(in); // still low, but diver acknowledged
    TEST_ASSERT_EQUAL_UINT16(0, s.bits);
    TEST_ASSERT_TRUE(s.ackSuppressed[alarms::BIT_LOW_PPO2]);

    in.votedBar = 0.55f;
    m.update(in); // recovery re-arms

    in.votedBar = 0.10f;
    Result again = m.update(in);
    TEST_ASSERT_TRUE(again.bits & (1u << alarms::BIT_LOW_PPO2)); // re-alarm edge
}

void test_hypoxia_event_latches_until_ack() {
    Manager m{Config{}};
    Inputs in = base();
    in.vote.hypoxiaSuspect = true;
    Result raised = m.update(in);
    TEST_ASSERT_TRUE(raised.bits & (1u << alarms::BIT_HYPOXIA));
    TEST_ASSERT_TRUE(raised.anyCritical);

    in.vote.hypoxiaSuspect = false; // one-shot condition already gone
    Result held = m.update(in);
    TEST_ASSERT_TRUE(held.bits & (1u << alarms::BIT_HYPOXIA)); // latched
    TEST_ASSERT_EQUAL_UINT8(0, held.edgeCount);               // no new edge

    m.ack();
    Result cleared = m.update(in);
    TEST_ASSERT_EQUAL_UINT16(0, cleared.bits); // event consumed by ACK
}

void test_warning_levels_and_sticky_bits() {
    Manager m{Config{}};
    Inputs in = base();
    in.vbat = 9.8f;
    Result rw = m.update(in);
    TEST_ASSERT_TRUE(rw.bits & (1u << alarms::BIT_BATTERY));
    TEST_ASSERT_TRUE(rw.anyWarning);
    TEST_ASSERT_FALSE(rw.anyCritical);

    in.spClamped = true;
    Result r2 = m.update(in);
    TEST_ASSERT_TRUE(r2.bits & (1u << alarms::BIT_SP_CLAMP));
    TEST_ASSERT_TRUE(r2.bits & (1u << alarms::BIT_BATTERY)); // previous persists
    TEST_ASSERT_EQUAL_UINT8(1, r2.edgeCount); // only the new one is an edge
}

void test_high_alarm_uses_the_honest_cell_veto() {
    // The vote lies (two poisoned cells agree at 0.50) while a third cell sees
    // 1.60: HIGH_PPO2 must still fire. The veto is a safety rule, not a detail.
    Manager m{Config{}};
    Inputs in = base();
    in.votedBar = 0.50f;
    in.highVetoBar = 1.60f;
    Result r = m.update(in);
    TEST_ASSERT_TRUE(r.bits & (1u << alarms::BIT_HIGH_PPO2));
    TEST_ASSERT_FALSE(r.bits & (1u << alarms::BIT_LOW_PPO2));  // low still follows the vote
}

void test_no_vote_suppresses_ppo2_alarms() {    Manager m{Config{}};
    Inputs in = base();
    in.haveVote = false;
    in.votedBar = 0.02f; // must not fire pre-arm garbage
    Result r = m.update(in);
    TEST_ASSERT_EQUAL_UINT16(0, r.bits);
}

// ---------------------------------------------------------------------------
// Finding C2: a total voting failure used to be silent. BIT_SENSOR_FAIL was
// evaluated only inside `if (in.haveVote)` — and haveVote IS `mode != FAILED`,
// so the bit could never be raised. These tests are the reason it can be.
// ---------------------------------------------------------------------------

// All cells unseen/implausible/excluded: the machine cannot see its loop, and
// injection is already stopped on that. The diver has to be told.
void test_total_vote_failure_raises_sensor_fail() {
    Manager m{Config{}};
    Inputs in = base();
    in.haveVote = false;                  // loop.cpp: FAILED means no vote
    in.vote.mode = voting::Mode::FAILED;
    in.vote.cellsSeen = true;             // the cells HAVE answered: a loss, not a boot
    in.votedBar = 0.0f;
    Result raised = m.update(in);
    TEST_ASSERT_TRUE_MESSAGE(raised.bits & (1u << alarms::BIT_SENSOR_FAIL),
                             "a total voting failure was silent");
    TEST_ASSERT_TRUE(raised.anyCritical);
    TEST_ASSERT_EQUAL_UINT8(1, raised.edgeCount);
    TEST_ASSERT_EQUAL_UINT8(alarms::BIT_SENSOR_FAIL, raised.edges[0].id);
    TEST_ASSERT_TRUE(raised.edges[0].raised);

    // Recovery is self-clearing. SENSOR_FAIL is a level, not an event: the
    // moment the vote is back the bit must go, or the handset is blamed for an
    // alarm the machine already knows is over.
    in.haveVote = true;
    in.vote.mode = voting::Mode::OK_3CELL;
    in.votedBar = 0.55f;
    Result cleared = m.update(in);
    TEST_ASSERT_FALSE_MESSAGE(cleared.bits & (1u << alarms::BIT_SENSOR_FAIL),
                              "SENSOR_FAIL outlived the vote that came back");
    TEST_ASSERT_EQUAL_UINT8(1, cleared.edgeCount);
    TEST_ASSERT_FALSE(cleared.edges[0].raised);
}

// The boot window, on its own: before the first ADC sample lands the vote is
// FAILED for a second reason — nothing has been measured yet — and a controller
// that is still coming up has no fault to announce. cellsSeen is what keeps the
// C2 fix from becoming a boot-time false alarm.
void test_no_vote_before_the_first_sample_is_not_a_fault() {
    Manager m{Config{}};
    Inputs in = base();
    in.haveVote = false;
    in.vote = voting::Result{};  // mode FAILED, cellsSeen false: no sample yet
    in.votedBar = 0.0f;
    Result boot = m.update(in);
    TEST_ASSERT_EQUAL_UINT16_MESSAGE(0, boot.bits,
                                     "a controller still booting raised a sensor fault");
    TEST_ASSERT_EQUAL_UINT8(0, boot.edgeCount);

    // One plausible sample later, the same FAILED mode IS news.
    in.vote.cellsSeen = true;
    Result after = m.update(in);
    TEST_ASSERT_TRUE_MESSAGE(after.bits & (1u << alarms::BIT_SENSOR_FAIL),
                             "SENSOR_FAIL stayed off once the cells had been believed");
}

// Latch/ACK behaviour of the newly-live bit: CRITICAL and continuous, so ACK
// silences the sound and the LED while the condition stands, the condition
// stays visible to the UI as ackSuppressed, and a recovery re-arms it.
void test_sensor_fail_ack_silences_until_the_vote_returns() {
    Manager m{Config{}};
    Inputs in = base();
    in.haveVote = false;
    in.vote.mode = voting::Mode::FAILED;
    in.vote.cellsSeen = true;
    TEST_ASSERT_TRUE(m.update(in).bits & (1u << alarms::BIT_SENSOR_FAIL));

    m.ack();
    Result quiet = m.update(in);
    TEST_ASSERT_EQUAL_UINT16(0u, quiet.bits);  // the diver asked for silence
    TEST_ASSERT_TRUE(quiet.ackSuppressed[alarms::BIT_SENSOR_FAIL]);  // the record still says so

    in.haveVote = true;
    in.vote.mode = voting::Mode::OK_3CELL;
    in.votedBar = 0.55f;
    m.update(in);  // recovery re-arms

    in.haveVote = false;
    in.vote.mode = voting::Mode::FAILED;
    Result again = m.update(in);
    TEST_ASSERT_TRUE_MESSAGE(again.bits & (1u << alarms::BIT_SENSOR_FAIL),
                             "SENSOR_FAIL did not re-alarm after a recovery");
}

// HYPOXIA leaves the gate too, on narrow grounds: hypoxiaSuspect needs an
// active cell collapsing (voting.cpp), and the CellProof ban can demote that
// cell in the same update — a collapsing loop with no vote is the last place to
// swallow the claim. It cannot fire on anything else, so nothing phantom is
// added by evaluating it unconditionally.
void test_a_collapsing_loop_is_still_hypoxia_with_no_vote() {
    Manager m{Config{}};
    Inputs in = base();
    in.haveVote = false;
    in.vote.mode = voting::Mode::FAILED;
    in.vote.cellsSeen = true;
    in.vote.hypoxiaSuspect = true;
    Result r = m.update(in);
    TEST_ASSERT_TRUE_MESSAGE(r.bits & (1u << alarms::BIT_HYPOXIA),
                             "a collapsing loop went unannounced because the vote failed");
    TEST_ASSERT_TRUE(r.bits & (1u << alarms::BIT_SENSOR_FAIL));
    TEST_ASSERT_TRUE(r.anyCritical);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_low_ppo2_edge_pair_and_critical_level);
    RUN_TEST(test_ack_suppresses_continuous_until_recovery_then_rearms);
    RUN_TEST(test_hypoxia_event_latches_until_ack);
    RUN_TEST(test_warning_levels_and_sticky_bits);
    RUN_TEST(test_high_alarm_uses_the_honest_cell_veto);
    RUN_TEST(test_no_vote_suppresses_ppo2_alarms);
    RUN_TEST(test_total_vote_failure_raises_sensor_fail);
    RUN_TEST(test_no_vote_before_the_first_sample_is_not_a_fault);
    RUN_TEST(test_sensor_fail_ack_silences_until_the_vote_returns);
    RUN_TEST(test_a_collapsing_loop_is_still_hypoxia_with_no_vote);
    return UNITY_END();
}
