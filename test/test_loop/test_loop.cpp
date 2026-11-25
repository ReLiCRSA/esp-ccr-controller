// ============================================================================
// test_loop — behaviour that only exists in the assembled chain: what happens
// between the modules (sensor staleness gating the valve, the output fuse, the
// pre-dive lockout, a spiked depth sample vs the setpoint bands, a dead depth
// bus vs the depth readout and the setpoint rung, a dead depth bus vs the deco
// and CNS clocks, and what the pre-dive makes of a rig that cannot measure
// depth at all).
// The fault scenarios in test_sim cover the same ground physically; these pin
// the interface contracts the firmware ControlTask depends on.
// ============================================================================
#include <unity.h>

#include <cmath>

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
          bool fresh = true, bool pressureValid = true) {
    const uint32_t n = static_cast<uint32_t>(seconds * 1000.0f / Loop::kTickMs);
    for (uint32_t i = 0; i < n; i++) {
        SensorData sd;
        for (int ch = 0; ch < 3; ch++) sd.cells[ch] = bar;
        sd.depthM = depth;
        sd.pressureValid = pressureValid;
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

    void tick(uint32_t& now, float depth, bool fresh = true, bool pressureValid = true) {
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
        sd.pressureValid = pressureValid;
        sd.cellsFresh = fresh;
        sd.vbat = 12.4f;
        now += Loop::kTickMs;
        loop.cycle(sd, now);
    }

    void run(uint32_t& now, float depth, float seconds, bool fresh = true,
             bool pressureValid = true) {
        const uint32_t n = static_cast<uint32_t>(seconds * 1000.0f / Loop::kTickMs);
        for (uint32_t i = 0; i < n; i++) tick(now, depth, fresh, pressureValid);
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

// ---------------------------------------------------------------------------
// Finding C1: a depth bus that dies mid-dive makes the depth UNKNOWN. Unknown
// must not become 0 m anywhere in the chain — the two chips share one I2C port,
// so this is a single failure, and it used to arrive at the setpoint engine as
// "the diver is on the surface".
// ---------------------------------------------------------------------------
void test_dead_depth_bus_holds_depth_and_setpoint() {
    Config c = benchCfg();
    c.cellproof.enabled = false;  // depth leg only; the proof is test_cellproof's
    Loop loop(c);
    loop.setAutoMode(true);
    uint32_t now = 0;

    // A dive already in progress at 32 m: the SP3 band (1.00 bar), with the
    // loop a tenth below it so the valve has something to do.
    loop.startDive(now);
    feed(loop, now, 0.90f, 32.0f, 5.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 32.0f, loop.depthSmoothedM());
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.00f, loop.setpointBar());
    const uint32_t pulsesBefore = loop.pulseCount();
    TEST_ASSERT_GREATER_THAN_UINT32(0u, pulsesBefore);

    // 60 s of dead bus — fed the exact 0.0 m main.cpp used to fabricate for it.
    float minSp = 9.0f, minDepth = 999.0f;
    bool staleSeen = false;
    for (int i = 0; i < 600; i++) {
        SensorData sd;
        for (int ch = 0; ch < 3; ch++) sd.cells[ch] = 0.90f;
        sd.depthM = 0.0f;
        sd.pressureValid = false;
        sd.cellsFresh = true;
        sd.vbat = 12.4f;
        now += Loop::kTickMs;
        const State st = loop.cycle(sd, now);
        if (st.setpointBar < minSp) minSp = st.setpointBar;
        if (st.depthSmoothedM < minDepth) minDepth = st.depthSmoothedM;
        if (st.alarms.bits & (1u << alarms::BIT_SENSOR_STALE)) staleSeen = true;
    }

    // The readout never approaches the surface: 32 m held, not 0 m believed.
    TEST_ASSERT_GREATER_THAN_FLOAT_MESSAGE(20.0f, minDepth,
                                           "a dead depth bus drove the depth readout to 0 m");
    // The band is frozen by the frozen input — no separate inhibit latch, and
    // nothing walked it towards the surface rung (0.40 bar).
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.001f, 1.00f, minSp,
                                     "an invalid depth moved the setpoint band");
    // What the diver is owed on the wire: no new protocol bit, BIT_SENSOR_STALE
    // is the existing one that says the number on the display is held.
    TEST_ASSERT_TRUE_MESSAGE(staleSeen, "a dead depth bus raised no sensor alarm");
    // And the injector, documented by the count: it keeps regulating to the
    // held band, because the cells are still a measurement even when the depth
    // is not. This is the assertion that separates a held setpoint from the C1
    // walk — a loop sitting at 0.90 bar is far ABOVE the 0.40 bar surface rung,
    // so a setpoint that reached it would have injected nothing at all.
    TEST_ASSERT_GREATER_THAN_UINT32_MESSAGE(pulsesBefore, loop.pulseCount(),
                                            "injection stopped regulating to the held band");

    // Recovery: believed samples resume, the stale alarm clears, and the band
    // is free to move again — the freeze is a held input, not a latched mode.
    feed(loop, now, 0.90f, 32.0f, 2.0f);
    TEST_ASSERT_FALSE(loop.alarmBits() & (1u << alarms::BIT_SENSOR_STALE));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.00f, loop.setpointBar());
    feed(loop, now, 0.90f, 1.0f, 5.0f);
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.001f, 0.40f, loop.setpointBar(),
                                     "the band stayed frozen after the bus recovered");
}

// ---------------------------------------------------------------------------
// Follow-up #1: the depth bus dies, and the diver's exposure must not die with
// it. CNS comes from the oxygen cells (a different sensor, a different bus);
// tissue loading comes from ambient pressure, and during a depth fault the only
// pressure available is the held last-good one. Two rigs, identical feeds, one
// honest depth and one blind for 300 s: their CNS clocks must stay together,
// and the blind one's tissues must still be loading. The old chain froze both.
void test_blind_depth_window_still_clocks_cns_and_loads_tissues() {
    Config c = benchCfg();
    c.cellproof.enabled = false;  // the depth leg only; the proof is test_cellproof's
    Loop blind(c), healthy(c);
    uint32_t nb = 0, nh = 0;
    blind.setAutoMode(true);
    healthy.setAutoMode(true);
    blind.startDive(nb);
    healthy.startDive(nh);
    // 32 m on the SP3 band, loop at 1.30 bar: no injection (the loop is above
    // the setpoint) and CNS accruing at the NOAA deep row, ~0.5 %/min.
    feed(blind, nb, 1.30f, 32.0f, 10.0f);
    feed(healthy, nh, 1.30f, 32.0f, 10.0f);
    TEST_ASSERT_TRUE_MESSAGE(blind.haveVote(), "no vote on a rig with three honest cells");
    TEST_ASSERT_FLOAT_WITHIN(1e-6, blind.cnsPct(), healthy.cnsPct());

    const double cnsBlind0 = blind.cnsPct();
    const double cnsHealthy0 = healthy.cnsPct();
    const double fast0 = blind.deco().tissueBar(0);
    const double slow0 = blind.deco().tissueBar(15);
    const double ceilM0 = blind.deco().ceilingM();
    const double ndl0 = blind.deco().ndlMinutes(blind.deco().depthToBar(32.0f), 1.3f);

    // 300 s (3000 control cycles) with the depth leg dead, fed the 0.0 m that
    // main.cpp used to fabricate for it, while the cells stay a measurement.
    for (int i = 0; i < 3000; i++) {
        SensorData sd;
        for (int ch = 0; ch < 3; ch++) sd.cells[ch] = 1.30f;
        sd.depthM = 0.0f;
        sd.cellsFresh = true;
        sd.vbat = 12.4f;
        sd.pressureValid = false;
        nb += Loop::kTickMs;
        blind.cycle(sd, nb);
        sd.pressureValid = true;
        sd.depthM = 32.0f;
        nh += Loop::kTickMs;
        healthy.cycle(sd, nh);
    }

    // 1. The oxygen clock does not depend on the depth leg. Same ppO2, same
    // dt, so the two rigs must have clocked the same exposure — and it must be
    // real exposure (2.8-3.3 % over 5 min at 1.30 bar, depending on which NOAA
    // row the float lands on), not the frozen 0 the old gate produced.
    const double gainBlind = blind.cnsPct() - cnsBlind0;
    const double gainHealthy = healthy.cnsPct() - cnsHealthy0;
    TEST_ASSERT_GREATER_THAN_FLOAT_MESSAGE(
        1.0f, gainBlind,
        "a dead depth bus stopped the CNS clock: ppO2 exposure stopped being counted");
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(
        0.01, gainHealthy, gainBlind,
        "CNS depended on the depth leg: the blind rig clocked different exposure for the same ppO2");

    // 2. Tissues keep integrating, on the held depth. The fast compartment
    // (5 min half-time) must have closed most of the way toward the 32 m ppN2;
    // even the 630 min compartment has to have moved.
    TEST_ASSERT_GREATER_THAN_FLOAT_MESSAGE(
        fast0 + 0.8f, blind.deco().tissueBar(0),
        "tissues froze during a depth fault: the window's gas loading was never integrated");
    TEST_ASSERT_TRUE(blind.deco().tissueBar(15) > slow0);

    // 3. And the debt reaches the consumers the diver reads. Across this window
    // the demanded ceiling goes from "no stop" to a stop, and the NDL drops
    // (measured on this exact profile: 0 m -> 1.8 m, and 8.2 -> 3.0 min).
    // main.cpp writes exactly these two numbers (plus the schedule) into every
    // 30 s grid row, so a window whose loading was forgotten would show up in
    // the dive file as a shallower ceiling and a longer NDL than the diver had.
    const double ceilM1 = blind.deco().ceilingM();
    const double ndl1 = blind.deco().ndlMinutes(blind.deco().depthToBar(32.0f), 1.3);
    TEST_ASSERT_GREATER_THAN_FLOAT_MESSAGE(
        ceilM0 + 0.5f, ceilM1, "the window's loading never reached the ceiling the diver is shown");
    TEST_ASSERT_TRUE_MESSAGE(ndl1 + 1.0 < ndl0,
                             "the window's loading never reached the NDL the diver is shown");
}

// The residual, named in the test so nobody has to rediscover it: with no
// trustworthy ppO2 there is nothing to integrate, so the CNS clock stops — and
// "trustworthy" means the same thing it means to the injector: a vote, from
// samples that are actually new. All three fitted cells unplug at once (a
// harness letting go) on top of a dead depth bus, and then the cells come back
// but the ADC stalls. The alarm layer covers both windows — this asserts the
// pairing, so the under-record can never exist without the shout that says so.
void test_cns_freezes_without_a_trustworthy_ppo2_and_that_is_paired_with_an_alarm() {
    Config c = benchCfg();
    c.cellproof.enabled = false;
    Loop loop(c);
    loop.setAutoMode(true);
    uint32_t now = 0;
    loop.startDive(now);
    feed(loop, now, 1.30f, 32.0f, 10.0f);
    const double cns0 = loop.cnsPct();
    const double fast0 = loop.deco().tissueBar(0);
    TEST_ASSERT_GREATER_THAN_FLOAT(0.0f, cns0);  // the clock was running

    Config noCells = c;
    noCells.presentMask = 0;  // every fitted cell unplugged: no believable sample
    loop.setConfig(noCells);
    bool staleSeen = false;
    for (int i = 0; i < 3000; i++) {
        SensorData sd;
        for (int ch = 0; ch < 3; ch++) sd.cells[ch] = 1.30f;
        sd.depthM = 0.0f;
        sd.cellsFresh = true;
        sd.vbat = 12.4f;
        sd.pressureValid = false;
        now += Loop::kTickMs;
        const State st = loop.cycle(sd, now);
        if (st.alarms.bits & (1u << alarms::BIT_SENSOR_STALE)) staleSeen = true;
    }

    TEST_ASSERT_FALSE_MESSAGE(loop.haveVote(), "a rig with no fitted cell produced a vote");
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(
        1e-6, cns0, loop.cnsPct(),
        "CNS accrued oxygen exposure with no ppO2 measurement to integrate: the no-vote "
        "freeze is deliberate (the documented residual, covered by the sensor alarm)");
    // The tissues still integrate the held depth: ppO2 only shifts ppN2, and a
    // zero-vote ppO2 makes that HIGHER — the conservative direction.
    TEST_ASSERT_GREATER_THAN_FLOAT_MESSAGE(fast0 + 0.8f, loop.deco().tissueBar(0),
                                           "tissues froze when the cells were lost too");
    TEST_ASSERT_TRUE_MESSAGE(staleSeen,
                             "a window with no depth and no ppO2 raised no sensor alarm");

    // The other half of the same trust rule: a VOTE built from frozen samples is
    // not a measurement either (the injector is held to the identical rule), so
    // the oxygen clock stops for a stalled ADC as well. Cells back fitted, then
    // 300 s of readings that never update, depth bus still dead.
    loop.setConfig(c);
    const double cnsNoVoteEnd = loop.cnsPct();
    const double fastBeforeStaleLeg = loop.deco().tissueBar(0);
    for (int i = 0; i < 3000; i++) {
        SensorData sd;
        for (int ch = 0; ch < 3; ch++) sd.cells[ch] = 1.30f;
        sd.depthM = 0.0f;
        sd.cellsFresh = false;  // the ADC stalled: same number, not a new one
        sd.vbat = 12.4f;
        sd.pressureValid = false;
        now += Loop::kTickMs;
        loop.cycle(sd, now);
    }
    TEST_ASSERT_TRUE_MESSAGE(loop.haveVote(), "the vote did not come back with the cells fitted");
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(
        1e-6, cnsNoVoteEnd, loop.cnsPct(),
        "CNS accrued exposure from frozen cell readings: a stale ppO2 is not a measurement");
    // The depth half is unaffected by the cell stall: it keeps integrating the
    // held depth, which is the whole point of gating the two halves separately.
    // Direction here is deliberately not pinned — the no-vote leg above left the
    // tissues loaded toward a ppN2 computed with ppO2 = 0, so with the vote back
    // the compartment is entitled to come back down. What must not happen is
    // that nothing moves at all.
    TEST_ASSERT_TRUE_MESSAGE(std::fabs(loop.deco().tissueBar(0) - fastBeforeStaleLeg) > 0.1,
                             "tissues stopped integrating when the cells went stale: the depth "
                             "half is independent of the cell half");
}

// Review H3, at the chain level (the sequencer's own rule is test_predive's):
// a rig whose depth bus never worked must not walk away from the pre-dive with
// a PASS. A dead bus used to be handed the 0.0 m that means "on the surface",
// which is the one answer that makes the depth step happy.
void test_pre_dive_refuses_a_dead_depth_bus() {
    Config c = benchCfg();
    const int32_t graceTicks = (int32_t)(c.predive.depthGraceMs / Loop::kTickMs);
    Loop loop(c);
    Plant plant(loop);
    uint32_t now = 0;
    loop.prediveStart(now);

    // The bus is dead for the whole pre-dive. The grace means the FAIL is no
    // longer the first silent cycle — but it must still arrive, on the grace's
    // schedule, and it must say the grace is what expired (finding C1: an
    // unmeasured depth may never be believed, and may never hang the sequence).
    int32_t depthRunTick = -1, failTick = -1;
    bool silentSeen = false, pendingMidGrace = false;
    for (int i = 0; i < 400 && !loop.prediveFailed(); ++i) {  // 40 s
        plant.tick(now, 0.0f, /*fresh=*/true, /*pressureValid=*/false);
        if (depthRunTick < 0 && loop.prediveStatus(predive::S_DEPTH) == predive::RUNNING)
            depthRunTick = i;
        if (depthRunTick >= 0 && loop.prediveSeq().depthBusSilent()) silentSeen = true;
        if (depthRunTick >= 0 && i == depthRunTick + graceTicks / 2)
            pendingMidGrace = loop.prediveStatus(predive::S_DEPTH) == predive::RUNNING &&
                              !loop.prediveFailed();
        if (loop.prediveFailed()) failTick = i;
    }

    TEST_ASSERT_TRUE_MESSAGE(pendingMidGrace,
                             "the DEPTH step failed while the grace still had room for the "
                             "bus to answer");
    TEST_ASSERT_TRUE_MESSAGE(silentSeen,
                             "a dead depth bus was never visible as a silent depth bus");
    TEST_ASSERT_TRUE_MESSAGE(loop.prediveFailed(),
                             "a dead depth bus produced no FAIL: either the sequence passed "
                             "without a depth measurement (finding C1) or the grace expired "
                             "with no verdict (a hang)");
    TEST_ASSERT_TRUE_MESSAGE(failTick - depthRunTick >= graceTicks,
                             "the depth FAIL landed before the grace had run");
    TEST_ASSERT_TRUE_MESSAGE(failTick - depthRunTick <= graceTicks + 2,
                             "the depth FAIL was not bounded by the grace: the pre-dive hung "
                             "past it instead of returning a verdict");
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(predive::FAIL, loop.prediveStatus(predive::S_DEPTH),
                                    "the DEPTH step did not fail on a dead bus");
    TEST_ASSERT_EQUAL_STRING_MESSAGE("depth bus never answered (grace expired)",
                                     loop.prediveSeq().depthWhy(),
                                     "the pre-dive FAIL did not say the grace is what expired");
    TEST_ASSERT_FALSE(loop.predivePassed());
    TEST_ASSERT_FALSE_MESSAGE(loop.requestAuto(true),
                              "AUTO granted to a rig that cannot measure depth");
}

// Follow-up #2, end to end: the pressure bus hiccups for 2 s and lands exactly on
// the DEPTH step. Before the grace that aborted a 2-minute sequence on one cycle;
// now the sequence rides through it and the dive goes ahead. The pair of asserts
// is the point — a grace that never fails would be a lockout that never ends, and
// the dead-bus case above is the other half of the pair.
void test_pre_dive_rides_through_a_short_depth_bus_dropout() {
    Loop loop(benchCfg());
    Plant plant(loop);
    uint32_t now = 0;
    loop.prediveStart(now);

    // Normal bus until the DEPTH step is running.
    int i = 0;
    for (; i < 400 && loop.prediveStatus(predive::S_DEPTH) != predive::RUNNING; ++i)
        plant.tick(now, 0.0f);
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(predive::RUNNING, loop.prediveStatus(predive::S_DEPTH),
                                    "the pre-dive never reached the DEPTH step");

    // 2 s of silence — inside the grace — fed the exact 0.0 m the old firmware
    // fabricated for "unknown". It must decide nothing either way.
    for (int k = 0; k < 20; ++k) plant.tick(now, 0.0f, /*fresh=*/true, /*pressureValid=*/false);
    TEST_ASSERT_FALSE_MESSAGE(loop.prediveFailed(),
                              "a 2 s depth dropout aborted the pre-dive sequence");
    TEST_ASSERT_TRUE_MESSAGE(loop.prediveSeq().depthBusSilent(),
                             "the dropout was not visible as a silent depth bus");
    TEST_ASSERT_EQUAL_STRING("depth bus silent, waiting", loop.prediveSeq().depthWhy());
    TEST_ASSERT_FALSE(loop.predivePassed());  // still running: nothing was thrown away

    // The bus answers again: the step judges the sample it gets and the sequence
    // finishes on its own, with the earlier steps' verdicts intact.
    for (int k = 0; k < 100; ++k) plant.tick(now, 0.05f);
    TEST_ASSERT_TRUE_MESSAGE(loop.predivePassed(),
                             "the pre-dive did not finish after a depth dropout shorter than "
                             "the grace");
    TEST_ASSERT_EQUAL_UINT8(predive::PASS, loop.prediveStatus(predive::S_DEPTH));
    TEST_ASSERT_EQUAL_UINT8(predive::PASS, loop.prediveStatus(predive::S_SOLENOID));
    TEST_ASSERT_FALSE(loop.prediveFailed());
    TEST_ASSERT_TRUE_MESSAGE(loop.requestAuto(true),
                             "AUTO refused after a pre-dive that passed through a dropout");
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_stale_sensors_stop_injection);
    RUN_TEST(test_output_fuse_cuts_a_valve_that_will_not_close);
    RUN_TEST(test_auto_refused_until_pre_dive_passes);
    RUN_TEST(test_unresponsive_cell_fails_the_cell_test);
    RUN_TEST(test_depth_spike_does_not_move_the_setpoint);
    RUN_TEST(test_dead_depth_bus_holds_depth_and_setpoint);
    RUN_TEST(test_blind_depth_window_still_clocks_cns_and_loads_tissues);
    RUN_TEST(test_cns_freezes_without_a_trustworthy_ppo2_and_that_is_paired_with_an_alarm);
    RUN_TEST(test_pre_dive_refuses_a_dead_depth_bus);
    RUN_TEST(test_pre_dive_rides_through_a_short_depth_bus_dropout);
    return UNITY_END();
}
