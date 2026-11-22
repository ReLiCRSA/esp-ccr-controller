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

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_low_ppo2_edge_pair_and_critical_level);
    RUN_TEST(test_ack_suppresses_continuous_until_recovery_then_rearms);
    RUN_TEST(test_hypoxia_event_latches_until_ack);
    RUN_TEST(test_warning_levels_and_sticky_bits);
    RUN_TEST(test_high_alarm_uses_the_honest_cell_veto);
    RUN_TEST(test_no_vote_suppresses_ppo2_alarms);
    return UNITY_END();
}
