// ============================================================================
// test_sim — system scenarios on the PC simulator: the full control chain
// (voting -> pre-dive/CellProof -> setpoints -> injector -> alarms -> deco)
// driving a physics world, with faults injected ONLY through the world.
// Nothing here pokes a control module directly: every detection has to earn
// its alarm the same way it would have to in the water.
//
// These scenarios are the regression net for the interaction bugs unit tests
// cannot see — above all "the fault that defeats the detector built for it".
// ============================================================================
#include <unity.h>

#include "sim/rig.h"

using namespace sim;
using alarms::BIT_CELL_DRIFT;
using alarms::BIT_HIGH_PPO2;
using alarms::BIT_HYPOXIA;
using alarms::BIT_LOW_PPO2;
using alarms::BIT_STUCK_OPEN;

void setUp() {}
void tearDown() {}

namespace {

/// Options for the scenarios. Duty/gap limits and injector flow are sized for
/// an 8 l loop with a litre-per-minute injector, so control can physically keep
/// up with the loop and a scenario failure always means a logic failure, not a
/// starved actuator. The firmware's real numbers come from the solenoid bench
/// milestone (PLAN 5).
RigOpts simOpts() {
    RigOpts o;
    o.presentMask = 0b111;
    o.predive.warmupMs = 2000;  // real cells polarise for minutes; the sim need not
    o.inj.maxDutyPct = 40;
    o.inj.minGapMs = 400;
    o.world.loopVolumeL = 8.0f;
    o.world.sacLpm = 1.2f;
    o.world.injLpm = 60.0f;
    return o;
}

/// Pre-dive with the diver holding the loop in air by hand — what the bench
/// procedure says you do while the cells polarise.
void prediveHealthy(Rig& rig) {
    rig.prediveStart();
    for (int i = 0; i < 15 && !rig.predivePassed() && !rig.prediveFailed(); i++) {
        rig.run(4);
        if (rig.prediveStatus(predive::S_AIR) != predive::PASS) rig.world().changeO2Bar(0.02f);
    }
}

/// Starts the dive at the surface and pre-mixes to SP1, then starts the
/// ppO2 envelope fresh: a loop that begins in air is not a hypoxia event.
void startAtSurface(Rig& rig) {
    rig.startDive();
    rig.world().setDepthM(0.0f);
    rig.run(20);
    rig.markEnvelope();
}

/// Descends and settles. Crossing 10 m runs the CellProof test, which needs a
/// while to reach its target, so deeper legs get that time up front.
void descend(Rig& rig, float depthM, float settleS = 15.0f) {
    rig.world().setDepthM(depthM);
    rig.run(depthM >= 10.0f ? 40.0f + settleS : settleS);
}

/// Runs the rig for `seconds`, returning true if `bit` was ever reported.
bool alarmSeen(Rig& rig, uint8_t bit, float seconds) {
    const uint32_t n = static_cast<uint32_t>(seconds * 10.0f);
    for (uint32_t i = 0; i < n; i++) {
        rig.tick();
        if (rig.alarmBits() & (1u << bit)) return true;
    }
    return false;
}

/// True if every bit in `mask` was reported at least once inside the window.
/// Fault transitions are what matter, and several of them are only visible for
/// a couple of seconds while control is already fighting back.
bool alarmsSeen(Rig& rig, uint16_t mask, float seconds) {
    const uint32_t n = static_cast<uint32_t>(seconds * 10.0f);
    uint16_t seen = 0;
    for (uint32_t i = 0; i < n; i++) {
        rig.tick();
        seen |= rig.alarmBits();
        if ((seen & mask) == mask) return true;
    }
    return false;
}

}  // namespace

// ---------------------------------------------------------------------------
// S1 — clean baseline. Full dive, three healthy cells: everything that must
// happen does, everything that must stay quiet does.
// ---------------------------------------------------------------------------
void test_clean_dive() {
    Rig rig(simOpts());
    prediveHealthy(rig);
    TEST_ASSERT_TRUE_MESSAGE(rig.predivePassed(), "pre-dive must pass on a healthy rig");
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(predive::PASS, rig.prediveStatus(predive::S_SOLENOID),
                                    "pulse-and-check cell test must pass");
    TEST_ASSERT_TRUE_MESSAGE(rig.pulseCount() > 0, "test pulses never fired");
    TEST_ASSERT_TRUE_MESSAGE(rig.requestAuto(true), "AUTO refused after a passed pre-dive");

    startAtSurface(rig);
    descend(rig, 10.0f);
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(cellproof::State::PASS, rig.cpState(),
                                    "CellProof did not pass at its arm depth");
    descend(rig, 40.0f, 40.0f);
    TEST_ASSERT_GREATER_THAN_FLOAT_MESSAGE(0.90f, rig.vote().votedPpo2, "deep setpoint not held");
    rig.world().setDepthM(12.0f);
    rig.run(20);
    rig.world().setDepthM(6.0f);
    rig.run(20);
    rig.world().setDepthM(0.0f);
    rig.run(15);
    rig.endDive();

    TEST_ASSERT_GREATER_THAN_FLOAT_MESSAGE(0.35f, rig.minVotedSeen, "loop went hypoxic");
    TEST_ASSERT_LESS_THAN_FLOAT_MESSAGE(1.60f, rig.maxVotedSeen, "loop went toxic");
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(0b111, rig.vote().activeMask, "cells lost on a clean dive");
    TEST_ASSERT_EQUAL_UINT16_MESSAGE(0, rig.alarmBits(), "alarms on a clean dive");
    TEST_ASSERT_GREATER_THAN_FLOAT_MESSAGE(0.0f, rig.cnsPct(), "CNS clock never ran");
    TEST_ASSERT_GREATER_THAN_UINT32(20u, rig.pulseCount());
}

// ---------------------------------------------------------------------------
// S2 — one cell saturates at 0.80 bar. Air cal cannot see this fault; the
// drift rule must, CellProof must name the cell, and the loop must survive.
// ---------------------------------------------------------------------------
void test_one_saturated_cell() {
    Rig rig(simOpts());
    rig.setAutoMode(true);
    startAtSurface(rig);
    descend(rig, 20.0f);  // CellProof passes here while all cells are healthy
    descend(rig, 35.0f);  // SP3 = 1.00: high enough for a 0.80 cap to be a lie
    rig.world().cells[2].fault = CellFault::SATURATE;
    rig.world().cells[2].satCapBar = 0.80f;
    rig.run(10);  // ~1 s lag, then 10 samples out of band -> excluded

    TEST_ASSERT_EQUAL_UINT8_MESSAGE(0b011, rig.vote().activeMask, "saturated cell still voting");
    TEST_ASSERT_TRUE_MESSAGE(rig.alarmBits() & (1u << BIT_CELL_DRIFT), "drift not alarmed");
    TEST_ASSERT_LESS_THAN_UINT8(100u, rig.cellStatus(2).health);

    descend(rig, 35.0f, 30.0f);
    TEST_ASSERT_GREATER_THAN_FLOAT_MESSAGE(0.35f, rig.minVotedSeen, "loop went hypoxic");
    TEST_ASSERT_LESS_THAN_FLOAT_MESSAGE(1.60f, rig.maxVotedSeen, "loop over-enriched");
    rig.endDive();
}

// ---------------------------------------------------------------------------
// S3 — a cell dies mid-dive. The collapse must read as a SENSOR fault
// (dropout, cell excluded), not as real hypoxia, and the dive continues 2-cell.
// ---------------------------------------------------------------------------
void test_cell_death_mid_dive() {
    Rig rig(simOpts());
    rig.setAutoMode(true);
    startAtSurface(rig);
    descend(rig, 30.0f, 30.0f);
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(0b111, rig.vote().activeMask,
                                    "not a 3-cell vote before the fault");

    rig.world().cells[1].fault = CellFault::DEAD;
    rig.run(5);

    TEST_ASSERT_EQUAL_UINT8_MESSAGE(0b101, rig.vote().activeMask, "dead cell still voting");
    TEST_ASSERT_TRUE_MESSAGE(!(rig.alarmBits() & (1u << BIT_HYPOXIA)),
                             "sensor dropout mistaken for real hypoxia");
    TEST_ASSERT_TRUE_MESSAGE(rig.alarmBits() & (1u << BIT_CELL_DRIFT), "dropout not alarmed");

    // the dive continues: two cells, control still working, no runaway
    const uint32_t pulsesBefore = rig.pulseCount();
    rig.run(30);
    TEST_ASSERT_GREATER_THAN_UINT32(pulsesBefore, rig.pulseCount());
    TEST_ASSERT_TRUE_MESSAGE(rig.vote().mode == voting::Mode::DEGRADED_2CELL, "not a 2-cell vote");
    TEST_ASSERT_LESS_THAN_FLOAT_MESSAGE(1.60f, rig.maxVotedSeen, "runaway after dropout");
    rig.endDive();
}

// ---------------------------------------------------------------------------
// S4 — TWO cells stick low at depth. The median now lies, and the honest
// third cell may even be the one the drift rule drops. Only the per-cell high
// veto can stop injection: this is the scenario that justified highVetoBar.
// ---------------------------------------------------------------------------
void test_two_lying_cells_veto() {
    RigOpts o = simOpts();
    o.cellproof.enabled = false;  // isolate the veto rule from the CellProof layer
    Rig rig(o);
    rig.setAutoMode(true);
    startAtSurface(rig);
    descend(rig, 25.0f);
    TEST_ASSERT_LESS_THAN_FLOAT_MESSAGE(0.75f, rig.spTarget(), "expected the 0.70 band at 25 m");

    // Two cells LYING LOW at 0.55 / 0.60 while the loop really is at 0.70: the
    // median can never exceed 0.60 so demand never stops, and because the
    // honest cell is now the odd one out the drift rule is just as likely to
    // exclude IT. Cell 2 is all that stands between this loop and toxicity.
    //
    // The lie must CREEP, not step (tauS 120 s toward the stuck value). A step
    // this large is past the dropout slope, so the dropout gate itself catches
    // the pair within a cycle, the honest cell keeps the vote, the loop never
    // climbs, and the test grades the dropout gate instead of the veto. A creep
    // stays under dropoutRateBarS (0.045 bar/s at fault time vs the 0.15 gate)
    // and under the stuck detector's notice (the reading keeps moving, and the
    // dither keeps wobbling), so what is really on trial here is the veto: the
    // lying pair holds the vote lean, the loop climbs, and only the honest
    // cell's raw reading reaches the wall.
    rig.world().cells[0].tauS = 120.0f;
    rig.world().cells[0].fault = CellFault::STUCK_AT;
    rig.world().cells[0].stuckValue = 0.55f;
    rig.world().cells[1].tauS = 120.0f;
    rig.world().cells[1].fault = CellFault::STUCK_AT;
    rig.world().cells[1].stuckValue = 0.60f;

    TEST_ASSERT_TRUE_MESSAGE(alarmSeen(rig, BIT_HIGH_PPO2, 90.0f),
                             "honest cell never raised the high alarm");
    // The wall is a state, not a moment: the alarm threshold sits below it, so
    // the alarm arrives first and the wall engages a little later.
    bool inhibited = false;
    for (int i = 0; i < 400 && !inhibited; i++) { rig.tick(); inhibited = rig.injOut().inhibitedHigh; }
    TEST_ASSERT_TRUE_MESSAGE(inhibited, "injector not inhibited at the wall");
    TEST_ASSERT_LESS_THAN_FLOAT_MESSAGE(1.80f, rig.maxTrueSeen,
                                        "injection not vetoed: loop went toxic");
    rig.endDive();
}

// ---------------------------------------------------------------------------
// S5 — real hypoxia: a diluent flush dumps the loop's O2. All cells collapse
// together, which must read as HYPOXIA (cells stay trusted), latch until ACK,
// and the injector must bring the loop back.
// ---------------------------------------------------------------------------
void test_diluent_flush_hypoxia() {
    Rig rig(simOpts());
    rig.setAutoMode(true);
    startAtSurface(rig);
    descend(rig, 20.0f);
    TEST_ASSERT_GREATER_THAN_FLOAT_MESSAGE(0.50f, rig.vote().votedPpo2, "not settled before flush");
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(0b111, rig.vote().activeMask, "cells already lost");

    rig.world().changeO2Bar(-0.75f);  // big diluent flush
    TEST_ASSERT_TRUE_MESSAGE(alarmsSeen(rig, (1u << BIT_HYPOXIA) | (1u << BIT_LOW_PPO2), 10.0f),
                             "flush: hypoxia and/or low-ppO2 alarm missing");
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(0b111, rig.vote().activeMask,
                                    "hypoxia wrongly treated as sensor failure");

    rig.run(20);
    rig.ackAlarms();
    rig.run(1);
    TEST_ASSERT_TRUE_MESSAGE(!(rig.alarmBits() & (1u << BIT_HYPOXIA)), "ACK did not clear hypoxia");

    rig.run(60);  // the loop must come back up to its setpoint
    TEST_ASSERT_GREATER_THAN_FLOAT_MESSAGE(0.45f, rig.vote().votedPpo2, "loop did not recover");
    rig.endDive();
}

// ---------------------------------------------------------------------------
// S6 — the poisoned cell the air check cannot see must fail the scripted
// pulse-and-check test, so the pre-dive never reaches PASS and AUTO can never
// be armed.
// ---------------------------------------------------------------------------
void test_predive_catches_poisoned_cell() {
    // (a) a cell reading nowhere near air must stop the run before it passes
    Rig rig(simOpts());
    rig.world().cells[2].fault = CellFault::STUCK_AT;
    rig.world().cells[2].stuckValue = 0.05f;
    rig.prediveStart();
    rig.run(40);
    TEST_ASSERT_TRUE_MESSAGE(!rig.predivePassed(),
                             "pre-dive passed with a cell reading 0.05 in air");
    TEST_ASSERT_TRUE_MESSAGE(rig.prediveRunning(), "air step must keep waiting, not pass");

    // (b) reads perfect air, deaf to oxygen: the pulse test is the only thing
    // that can see it, and it must fail the SOLENOID step.
    Rig rig2(simOpts());
    rig2.world().cells[2].fault = CellFault::STUCK_AT;
    rig2.world().cells[2].stuckValue = 0.208f;
    prediveHealthy(rig2);
    TEST_ASSERT_TRUE_MESSAGE(rig2.prediveFailed(), "stuck-reading cell passed the pulse test");
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(predive::FAIL, rig2.prediveStatus(predive::S_SOLENOID),
                                    "wrong pre-dive step failed");
    TEST_ASSERT_FALSE_MESSAGE(rig2.requestAuto(true), "AUTO armed after a failed pre-dive");
}

// ---------------------------------------------------------------------------
// S7 — a solenoid that will not close. Surface, depth-stable, nothing
// commanded: the stuck-open supervisor must latch and stop commanding the
// valve, and releasing that block must be an explicit diver act.
// ---------------------------------------------------------------------------
void test_stuck_open_valve() {
    Rig rig(simOpts());
    rig.setAutoMode(true);
    startAtSurface(rig);
    TEST_ASSERT_FALSE_MESSAGE(rig.alarmBits() & (1u << BIT_STUCK_OPEN), "false fault at rest");

    rig.world().solenoidStuckOpen = true;
    TEST_ASSERT_TRUE_MESSAGE(alarmSeen(rig, BIT_STUCK_OPEN, 30.0f), "stuck valve not detected");
    TEST_ASSERT_TRUE_MESSAGE(alarmSeen(rig, BIT_HIGH_PPO2, 30.0f), "high ppO2 not alarmed");

    rig.world().solenoidStuckOpen = false;  // diver closes the valve
    rig.run(5);
    TEST_ASSERT_FALSE_MESSAGE(rig.solenoidOn(), "controller commanding the valve after the latch");
    TEST_ASSERT_TRUE_MESSAGE(rig.alarmBits() & (1u << BIT_STUCK_OPEN),
                             "fault latch must survive the valve closing by itself");

    // Only an explicit diver action releases the injection block: the machine
    // does not decide on its own that the valve is fine again.
    rig.clearStuckOpen();
    rig.run(1);
    TEST_ASSERT_TRUE_MESSAGE(!(rig.alarmBits() & (1u << BIT_STUCK_OPEN)),
                             "clear did not release the stuck-open alarm");
    rig.endDive();
}

// ---------------------------------------------------------------------------
// S8 — the coil rail (docs/REVIEW.md 1 P9). The scenario this item was written
// for: a charged pack, a diving machine, and a boost stage that has died. The
// rig feeds the scripted rail through the REAL sensors/rail.h at the board's
// 1 Hz sweep — the sim witnesses the designed detection chain, not a rerun of
// it. What is NOT modelled is sag during a pulse: that figure is a C3-c
// measurement this bench still owes, and the rig does not invent it.
// ---------------------------------------------------------------------------
void test_dead_boost_alarms_mid_dive_while_the_pack_is_fine() {
    RigOpts o = simOpts();
    o.predive.vrailSensed = 1;           // the fitted-divider build
    Rig rig(o);
    prediveHealthy(rig);
    TEST_ASSERT_TRUE_MESSAGE(rig.predivePassed(),
                             "a healthy judged rig must pass pre-dive — otherwise "
                             "the FAIL below proves nothing");
    startAtSurface(rig);
    descend(rig, 10.0f);
    rig.run(20);
    TEST_ASSERT_FALSE_MESSAGE(rig.alarmOn(alarms::BIT_BATTERY),
                              "battery marker up before the boost was even killed");
    TEST_ASSERT_TRUE(rig.railSwept());
    TEST_ASSERT_TRUE_MESSAGE(rig.pulseCount() > 0, "no baseline pulses to compare against");

    // Kill the boost. The pack stays at its charged 3.9 V: this is exactly the
    // fault the old firmware could not see, and the point of the leg is that
    // the pack numbers stay quiet while the marker comes up.
    rig.setRail(0.0f);
    rig.run(1);   // one more sweep: not enough. LOW is counted in sweeps
                  // (VRAIL_LOW_SAMPLES) precisely so one pulse-shaped dip is
                  // not an alarm, and the sim has to honour that too.
    TEST_ASSERT_FALSE_MESSAGE(rig.alarmOn(alarms::BIT_BATTERY),
                              "the marker fired with less than the sweep debounce");
    rig.run(10);
    TEST_ASSERT_TRUE_MESSAGE(rig.alarmOn(alarms::BIT_BATTERY),
                             "a dead boost with a healthy pack cleared the battery "
                             "marker — the exact §1 P9 failure");
    TEST_ASSERT_TRUE(rig.railUntrusted());
    TEST_ASSERT_TRUE(rig.railSensed() < VRAIL_MIN_V);

    // And the machine keeps diving while it says so: pulses keep counting, the
    // dive stays active. `pulses=` counts commands, not flow — this is the
    // sentence that keeps that fact from going folklore again.
    TEST_ASSERT_TRUE(rig.diveActive());
    TEST_ASSERT_TRUE_MESSAGE(rig.pulseCount() > 0, "the dive stopped on the alarm");
}

// The same fault caught BEFORE the dive: pre-dive BATTERY must refuse, name
// which rail, and the refusal must come from the settle deadline expiring —
// the message matters: with the sim's sd.vrail feed broken it would read
// "never sensed" instead, and the test would still see a FAIL. Asserting the
// words is what makes this leg mutation-sensitive.
void test_predive_refuses_a_dead_boost() {
    RigOpts o = simOpts();
    o.predive.vrailSensed = 1;
    Rig rig(o);
    rig.run(5);                          // boot with a live boost: the sense chain
    rig.setRail(0.0f);                   // has a nominal reading to lose
    prediveHealthy(rig);
    TEST_ASSERT_TRUE_MESSAGE(rig.prediveFailed(),
                             "pre-dive passed with a 0 V coil rail");
    TEST_ASSERT_EQUAL_UINT8(predive::FAIL, rig.prediveStatus(predive::S_BATTERY));
    TEST_ASSERT_EQUAL_STRING_MESSAGE("coil rail never settled back inside its band",
                                     rig.prediveSeq().railWhy(),
                                     "the BATTERY FAIL did not name the rail and the reason");
    TEST_ASSERT_FALSE_MESSAGE(rig.requestAuto(true),
                              "AUTO unlocked on a dive the pre-dive refused");
}

// The pair that makes the two tests above mean something: TODAY's build —
// rail not judged (VRAIL_SENSE=0, mirrored by predive.vrailSensed defaulting
// to it) — must behave exactly as it did before the rail existed. Same dead
// boost, same scenario: pre-dive passes, no marker. The sim that mirrors the
// flashed board is also the only way to rehearse C3-c leg 5 before the
// divider goes on.
void test_unjudged_sim_sees_nothing_of_a_dead_boost() {
    RigOpts o = simOpts();               // vrailSensed stays VRAIL_SENSE = 0
    Rig rig(o);
    rig.setRail(0.0f);
    prediveHealthy(rig);
    TEST_ASSERT_TRUE_MESSAGE(rig.predivePassed(),
                             "the un-judged build must still pass on the pack alone, "
                             "exactly as today's board does — that is the bug, stated");
    startAtSurface(rig);
    descend(rig, 10.0f);
    rig.run(20);
    TEST_ASSERT_FALSE_MESSAGE(rig.alarmOn(alarms::BIT_BATTERY),
                              "the un-judged build raised a rail alarm: the flag "
                              "gates the alarm in sim too, as VRAIL_SENSE does on metal");
    TEST_ASSERT_FALSE(rig.railSwept());  // never even sampled: nothing reads that pin
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_clean_dive);
    RUN_TEST(test_one_saturated_cell);
    RUN_TEST(test_cell_death_mid_dive);
    RUN_TEST(test_two_lying_cells_veto);
    RUN_TEST(test_diluent_flush_hypoxia);
    RUN_TEST(test_predive_catches_poisoned_cell);
    RUN_TEST(test_stuck_open_valve);
    RUN_TEST(test_dead_boost_alarms_mid_dive_while_the_pack_is_fine);
    RUN_TEST(test_predive_refuses_a_dead_boost);
    RUN_TEST(test_unjudged_sim_sees_nothing_of_a_dead_boost);
    return UNITY_END();
}
