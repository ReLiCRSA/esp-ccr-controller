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
    in.vbat = VBAT_CELLS * 3.90f;
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
    in.vbat = VBAT_CELLS * 3.10f;   // below the 1S floor
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

// A voltage this high is not a charged pack, it is the sense node: the divider's
// top leg open, or the sense pin loose on its header. Before the upper bound the
// alarm said nothing at all about it — `vbat < vbatMin` is simply false — so the
// board ran blind and called that healthy.
void test_battery_alarm_also_raises_on_an_unreadable_pack() {
    Manager m{Config{}};
    Inputs in = base();
    in.vbat = VBAT_IMPLAUSIBLE_V + 3.0f;
    Result r = m.update(in);
    TEST_ASSERT_TRUE_MESSAGE(r.bits & (1u << alarms::BIT_BATTERY),
                             "a pack the board cannot read must not clear the battery marker");
    TEST_ASSERT_TRUE(r.logBits & (1u << alarms::BIT_BATTERY));

    // Both directions, or the leg proves nothing: inside the band it clears, and
    // it clears at the top of a real charge as well as in the middle.
    Inputs mid = base();                     // 3.90 V
    TEST_ASSERT_FALSE(m.update(mid).bits & (1u << alarms::BIT_BATTERY));
    Inputs full = base(); full.vbat = VBAT_IMPLAUSIBLE_V;
    TEST_ASSERT_FALSE_MESSAGE(m.update(full).bits & (1u << alarms::BIT_BATTERY),
                              "the bound is a fault detector, not a charge limit");
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

// ---- the two vocabularies (alarms.h) --------------------------------------
// `bits` is what the diver is being told; `logBits` is what is true. They are the
// same set except while an ACK is silencing a CONTINUOUS CRITICAL, and the dive
// file is written from logBits while the handset, the LED and the E, edges are
// driven by bits. This is the case that pins the semantics themselves; dive 9001
// in the soak grades the consequence in a real dive.
void test_the_log_view_keeps_what_an_ack_silences_from_the_live_one() {
    Manager m{Config{}};
    Inputs in = base();
    in.sensorStale = true;  // the depth bus has stopped answering

    const Result up = m.update(in);
    TEST_ASSERT_TRUE_MESSAGE(up.bits & (1u << alarms::BIT_SENSOR_STALE),
                             "a dead depth bus raised nothing the diver could hear");
    TEST_ASSERT_TRUE_MESSAGE(up.logBits & (1u << alarms::BIT_SENSOR_STALE),
                             "the log view was already missing a condition the chain knew");
    TEST_ASSERT_FALSE(up.ackSuppressed[alarms::BIT_SENSOR_STALE]);

    m.ack();
    const Result quiet = m.update(in);
    // The diver's side of the split: the console goes quiet.
    TEST_ASSERT_EQUAL_UINT16_MESSAGE(0u, quiet.bits,
                                     "ACK did not silence the live word — the machine "
                                     "cannot be acknowledged");
    // The file's side: the condition is still standing, so the marker stays.
    TEST_ASSERT_TRUE_MESSAGE(quiet.logBits & (1u << alarms::BIT_SENSOR_STALE),
                             "ACK deleted a fact about the instrument state — the dive file "
                             "would stop marking rows written with no depth measurement");
    TEST_ASSERT_TRUE(quiet.ackSuppressed[alarms::BIT_SENSOR_STALE]);
    TEST_ASSERT_TRUE_MESSAGE((quiet.logBits & quiet.bits) == quiet.bits,
                             "logBits is not a superset of bits");
    // The silence itself is the one edge this cycle owes: `E,<t>,2,7` (alarm
    // cleared, bit 7) tells a live reader the diver acknowledged. Unchanged by the
    // log view, which has no edges of its own — E, rows stay the live vocabulary.
    TEST_ASSERT_EQUAL_UINT8(1, quiet.edgeCount);

    // No edge spam: the suppressed condition is neither re-raised nor re-cleared
    // every cycle, or the E, rows would repeat while the diver sits in silence.
    for (int i = 0; i < 3; i++) {
        const Result same = m.update(in);
        TEST_ASSERT_EQUAL_UINT8_MESSAGE(0, same.edgeCount,
                                        "an ACK-suppressed condition emitted an edge while "
                                        "nothing changed");
        TEST_ASSERT_EQUAL_UINT16(0u, same.bits);
        TEST_ASSERT_TRUE(same.logBits & (1u << alarms::BIT_SENSOR_STALE));
    }

    // Recovery clears BOTH views, and re-arms: logBits reports conditions, it does
    // not latch them, so a file that keeps claiming blindness after the bus came
    // back would be lying in the other direction.
    in.sensorStale = false;
    const Result back = m.update(in);
    TEST_ASSERT_EQUAL_UINT16(0u, back.bits);
    TEST_ASSERT_EQUAL_UINT16_MESSAGE(0u, back.logBits,
                                     "the log view outlived the condition it reports");
    // No edge here either: the live word already went quiet at the ACK, and that
    // was the news. The log view going quiet writes no row of its own — a reader
    // of the E, stream sees one clear, not two.
    TEST_ASSERT_EQUAL_UINT8(0, back.edgeCount);

    in.sensorStale = true;
    const Result again = m.update(in);
    TEST_ASSERT_TRUE(again.bits & (1u << alarms::BIT_SENSOR_STALE));
    TEST_ASSERT_TRUE(again.logBits & (1u << alarms::BIT_SENSOR_STALE));
}

// The two views can only differ where ack() can suppress, and ack() only suppresses
// CRITICAL ids. So a WARNING is the same word in both vocabularies, before and
// after an ACK — pinning that here keeps a future change to ack() from quietly
// splitting a warning across the two views and making the dive file disagree with
// the handset about something the diver was never asked to acknowledge.
void test_a_warning_is_the_same_in_both_views_across_an_ack() {
    Manager m{Config{}};
    Inputs in = base();
    in.vbat = VBAT_CELLS * 3.0f;  // BATTERY is WARNING

    const Result up = m.update(in);
    TEST_ASSERT_TRUE(up.bits & (1u << alarms::BIT_BATTERY));
    TEST_ASSERT_TRUE(up.logBits & (1u << alarms::BIT_BATTERY));

    m.ack();
    const Result after = m.update(in);
    TEST_ASSERT_TRUE_MESSAGE(after.bits & (1u << alarms::BIT_BATTERY),
                             "an ACK silenced a WARNING on the live word");
    TEST_ASSERT_TRUE(after.logBits & (1u << alarms::BIT_BATTERY));
    TEST_ASSERT_EQUAL_UINT16_MESSAGE(after.bits, after.logBits,
                                     "bits and logBits diverged on a warning — they have no "
                                     "reason to, and a reader would have to guess which "
                                     "column the file holds");
    TEST_ASSERT_FALSE(after.ackSuppressed[alarms::BIT_BATTERY]);
}

// The exception, and the reason logBits is not simply `rawActive_` ORed forever:
// an EVENT latch (kLatchAck — today only HYPOXIA) is CONSUMED by the ACK rather
// than suppressed. A one-shot verdict the diver has heard is not a standing
// condition, so the log view lets it go with the live one; the rising edge already
// wrote its own E, row, and re-marking every later dive-file row with a consumed
// event would turn one decision into hundreds of rows.
void test_an_ack_consumes_an_event_latch_and_both_views_let_it_go() {
    Manager m{Config{}};
    Inputs in = base();
    in.haveVote = false;
    in.vote.mode = voting::Mode::FAILED;
    in.vote.cellsSeen = true;
    in.vote.hypoxiaSuspect = true;

    const Result up = m.update(in);
    TEST_ASSERT_TRUE(up.bits & (1u << alarms::BIT_HYPOXIA));
    TEST_ASSERT_TRUE(up.logBits & (1u << alarms::BIT_HYPOXIA));

    in.vote.hypoxiaSuspect = false;  // the event is over; the latch is what remains
    const Result latched = m.update(in);
    TEST_ASSERT_TRUE(latched.bits & (1u << alarms::BIT_HYPOXIA));
    TEST_ASSERT_TRUE(latched.logBits & (1u << alarms::BIT_HYPOXIA));

    m.ack();
    const Result after = m.update(in);
    TEST_ASSERT_FALSE_MESSAGE(after.bits & (1u << alarms::BIT_HYPOXIA),
                              "ACK left a consumed hypoxia event on the live word");
    TEST_ASSERT_FALSE_MESSAGE(after.logBits & (1u << alarms::BIT_HYPOXIA),
                              "the log view re-marked a one-shot event the diver had already "
                              "acknowledged — every later dive-file row would claim it");
    // The continuous condition from the same cycle keeps its marker: what the
    // consume rule applies to is latches, not facts.
    TEST_ASSERT_EQUAL_UINT16(0u, after.bits & (1u << alarms::BIT_SENSOR_FAIL));
    TEST_ASSERT_TRUE_MESSAGE(after.logBits & (1u << alarms::BIT_SENSOR_FAIL),
                             "the continuous condition lost its log marker beside the latch");
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_low_ppo2_edge_pair_and_critical_level);
    RUN_TEST(test_ack_suppresses_continuous_until_recovery_then_rearms);
    RUN_TEST(test_hypoxia_event_latches_until_ack);
    RUN_TEST(test_warning_levels_and_sticky_bits);
    RUN_TEST(test_battery_alarm_also_raises_on_an_unreadable_pack);
    RUN_TEST(test_high_alarm_uses_the_honest_cell_veto);
    RUN_TEST(test_no_vote_suppresses_ppo2_alarms);
    RUN_TEST(test_total_vote_failure_raises_sensor_fail);
    RUN_TEST(test_no_vote_before_the_first_sample_is_not_a_fault);
    RUN_TEST(test_sensor_fail_ack_silences_until_the_vote_returns);
    RUN_TEST(test_a_collapsing_loop_is_still_hypoxia_with_no_vote);
    RUN_TEST(test_the_log_view_keeps_what_an_ack_silences_from_the_live_one);
    RUN_TEST(test_a_warning_is_the_same_in_both_views_across_an_ack);
    RUN_TEST(test_an_ack_consumes_an_event_latch_and_both_views_let_it_go);
    return UNITY_END();
}
