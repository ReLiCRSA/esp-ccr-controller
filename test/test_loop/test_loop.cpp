// ============================================================================
// test_loop — behaviour that only exists in the assembled chain: what happens
// between the modules (sensor staleness gating the valve, the output fuse, the
// pre-dive lockout, a spiked depth sample vs the setpoint bands, a dead depth
// bus vs the depth readout and the setpoint rung, a dead depth bus vs the deco
// and CNS clocks, and what the pre-dive makes of a rig that cannot measure depth
// at all — or of one whose depth bus hiccups for two seconds in the middle of
// the depth check), and what an ACK leaves in the dive file's alarm word after it
// silenced the one the diver hears.
// The fault scenarios in test_sim cover the same ground physically; these pin
// the interface contracts the firmware ControlTask depends on.
// ============================================================================
#include <unity.h>

#include <cmath>

#include "control/loop.h"
#include "log/rowfill.h"   // the dive-file row fill, fed a real chain State below

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

// Live-channel dither for fixture feeds: ±1 ADC count (0.21/1490 per count) in
// a non-repeating pattern. The voting layer's frozen-reading detector condemns
// a channel whose reading never moves across its 20 s window, so a fixture that
// feeds a bit-identical value for 30 s is not a calm rig, it is an injected
// frozen-channel fault. Real cells wobble 4..7 counts p-p (bench, 2026-11-26);
// this is quieter than the bench on purpose — it only has to stay above the
// 2-count epsilon the detector uses — and it is far inside every tolerance
// these tests assert on.
float liveCell(float bar, uint32_t i, int ch) {
    const uint32_t h = (i * 2654435761u) >> 30;  // 0..3, non-repeating phase
    return bar + static_cast<float>(static_cast<int>((h + static_cast<uint32_t>(ch)) % 4u)) * 0.00021f - 0.00032f;
}

// Feeds the loop a fixed cell level / depth for `seconds`.
//
// The cell feeds carry a small ADC-count wobble, and that is not decoration:
// the voting layer's frozen-reading detector condemns a channel whose reading
// never moves (bench: a live front end dithers 4..7 counts p-p forever, a
// frozen one does not), so a fixture that feeds a bit-identical value for more
// than the 20 s window IS injecting the frozen-channel fault. A live chain is
// the normal state these tests want; a test that wants a frozen channel asks
// for it on purpose. Amplitude = ±3 counts at the factory slope
// (0.21/1490), matching the sim's bench-matched dither and well inside every
// tolerance below.
void feed(Loop& loop, uint32_t& now, float bar, float depth, float seconds,
          bool fresh = true, bool pressureValid = true) {
    const uint32_t n = static_cast<uint32_t>(seconds * 1000.0f / Loop::kTickMs);
    for (uint32_t i = 0; i < n; i++) {
        SensorData sd;
        for (int ch = 0; ch < 3; ch++)
            sd.cells[ch] = liveCell(bar, i, ch);
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
        for (int ch = 0; ch < 3; ch++)
            sd.cells[ch] = liveCell(bar[ch], pulses + static_cast<uint32_t>(ch), ch);
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
        for (int ch = 0; ch < 3; ch++) sd.cells[ch] = liveCell(0.30f, static_cast<uint32_t>(i), ch);
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
        for (int ch = 0; ch < 3; ch++) sd.cells[ch] = liveCell(0.70f, static_cast<uint32_t>(i), ch);
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
        for (int ch = 0; ch < 3; ch++) sd.cells[ch] = liveCell(0.90f, static_cast<uint32_t>(i), ch);
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
        for (int ch = 0; ch < 3; ch++) sd.cells[ch] = liveCell(1.30f, static_cast<uint32_t>(i), ch);
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

// The chain-level half of the vocabulary split (alarms.h): ControlTask writes the
// dive file's alarmBits from loop.alarmLogBits() and the handset's `T,` FLAGS from
// loop.alarmBits(). One dead depth bus and one ACK has to separate them — silence
// on the wire, marker in the file — and both have to let go when the bus answers
// again. Dive 9001 in the soak grades the same split over a whole dive with a
// climbing diver and 30 s rows; this pins the interface contract in one place, with
// the alarm ids visible.
void test_an_ack_silences_a_dead_depth_bus_on_the_wire_but_not_in_the_log() {
    Config c = benchCfg();
    c.cellproof.enabled = false;  // the depth leg only
    Loop loop(c);
    loop.setAutoMode(true);
    uint32_t now = 0;
    feed(loop, now, 1.30f, 32.0f, 10.0f);
    TEST_ASSERT_EQUAL_UINT16(0u, loop.alarmBits());

    // The bus stops answering. Both views say so: nothing has been silenced yet,
    // so the two words are the same word.
    feed(loop, now, 1.30f, 32.0f, 4.0f, /*fresh=*/true, /*pressureValid=*/false);
    TEST_ASSERT_TRUE_MESSAGE(loop.alarmBits() & (1u << alarms::BIT_SENSOR_STALE),
                             "a depth bus that stopped answering never raised STALE");
    TEST_ASSERT_TRUE(loop.alarmLogBits() & (1u << alarms::BIT_SENSOR_STALE));

    // The diver ACKs. Both halves of the split are requirements, not a preference:
    // the live word must go quiet or the handset's ACK button does nothing, and the
    // log view must keep the marker or the dive file stops marking the rows that
    // were written from a held depth (finding #5).
    loop.ackAlarms();
    feed(loop, now, 1.30f, 32.0f, 1.0f, /*fresh=*/true, /*pressureValid=*/false);
    TEST_ASSERT_EQUAL_UINT16_MESSAGE(0u, loop.alarmBits() & (1u << alarms::BIT_SENSOR_STALE),
                                     "ACK did not silence the depth alarm the diver hears");
    TEST_ASSERT_TRUE_MESSAGE(loop.alarmLogBits() & (1u << alarms::BIT_SENSOR_STALE),
                             "the dive-file view lost BIT_SENSOR_STALE to an ACK");

    // The bus answers again and both views let go: the log view reports conditions,
    // it does not latch them, so the file cannot keep claiming blindness that ended.
    feed(loop, now, 1.30f, 32.0f, 2.0f);
    TEST_ASSERT_EQUAL_UINT16(0u, loop.alarmBits() & (1u << alarms::BIT_SENSOR_STALE));
    TEST_ASSERT_EQUAL_UINT16_MESSAGE(0u, loop.alarmLogBits() & (1u << alarms::BIT_SENSOR_STALE),
                                     "the log view kept claiming a depth fault that had ended");
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
        for (int ch = 0; ch < 3; ch++) sd.cells[ch] = liveCell(1.30f, static_cast<uint32_t>(i), ch);
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
        for (int ch = 0; ch < 3; ch++) sd.cells[ch] = liveCell(1.30f, static_cast<uint32_t>(i), ch);
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

// ---- finding N2: a failed vote and the number the diver is shown -------------
// The state this grades is a wet connector in the cell harness: two of three
// galvanic channels die at DIFFERENT values (a flooded connector biases each
// channel differently), CellProof runs at depth and marks them unproven because
// they never moved, and then the last live channel stops being answered at all.
// The chain then has no fitted cell it is willing to believe: Mode::FAILED with
// cellsSeen, which is the one state in which the handset's ppO2 column stops
// being a measurement.
//
// That route is worth spelling out, because it is the ONLY one the chain can
// reach: voting.cpp returns FAILED only when every fitted AND seen cell is
// banned, and the ban a proof run produces can never cover the whole fitted set
// (a run with nothing proven sets the clamp, not the mask — cellproof.cpp), while
// changing presentMask clears the "this cell answered" claim that BIT_SENSOR_FAIL
// is built on. So a mid-dive FAILED + cellsSeen needs the fitted set to lose the
// last honest channel on top of a proof ban, and that is why the case below pulls
// that lever itself instead of pretending a dead cell reads something implausible.
// (It also proves the reachability result: in the shipping firmware presentMask is
// a boot constant — main.cpp CELLS_PRESENT_MASK — so today a diver can only reach
// this state if a SET changes it. Reported, not papered over.)
//
// Two rigs, identical to the cycle, and one difference: `dead` loses ch2's
// channel and `live` keeps it. Every assert is a comparison between them, so a
// held number that leaked into the valve, the alarms, the tissue model or the dive
// file would make the two rigs agree — and the assert says so.
namespace {

Config wetCfg() {
    Config c = benchCfg();
    c.cellproof.depthM = 8.0f;  // the proof runs on arrival at 32 m
    return c;
}

struct WetConnector {
    Loop dead, live;
    uint32_t nd = 0, nl = 0;
    ctrl::State lastDead{}, lastLive{};  // the State each rig last published
    float heldBar = 0.0f;        // the last believable ppO2 when the channel died
    uint32_t pulsesDead = 0, pulsesLive = 0;
    double cnsDead = 0.0, cnsLive = 0.0;
    double fastDead = 0.0, fastLive = 0.0;

    WetConnector() : dead(wetCfg()), live(wetCfg()) {
        dead.setAutoMode(true);
        live.setAutoMode(true);
        dead.startDive(nd);
        live.startDive(nl);
        drive();
    }

    /// One control cycle with the three channels scripted independently.
    State step(Loop& loop, uint32_t& now, float c0, float c1, float c2, float depth) {
        SensorData sd;
        sd.cells[0] = c0; sd.cells[1] = c1; sd.cells[2] = c2;
        sd.depthM = depth;
        sd.pressureValid = true;
        sd.cellsFresh = true;
        sd.vbat = 12.4f;
        now += Loop::kTickMs;
        return loop.cycle(sd, now);
    }

    // `live` first, `dead` last, so lastDead/lastLive name the rig they came
    // from: mixing them up would silently grade the control rig's State.
    void both(float c0, float c1, float c2, float depth) {
        lastLive = step(live, nl, c0, c1, c2, depth);
        lastDead = step(dead, nd, c0, c1, c2, depth);
    }

    void drive() {
        // 0. on the surface with the cells in air, so CellProof can ARM: it arms
        //    shallow and runs on the first crossing of its depth (cellproof.cpp),
        //    and a chain that starts at 32 m never gives it that edge.
        for (int i = 0; i < 20; i++) both(0.21f, 0.21f, 0.21f, 0.0f);

        // 1. descent to 7 m while the channels settle. The steps stay under the
        //    voting layer's glitch wall (0.5 bar per sample), which is what a real
        //    loop does on the way down; ch0/ch1 end up a little apart from each
        //    other, which is how a flooded connector biases its channels.
        for (int i = 0; i < 20; i++) {
            const float f = static_cast<float>(i + 1) / 20.0f;
            both(0.21f + 0.84f * f, 0.21f + 0.89f * f, 0.21f + 0.94f * f, 7.0f * f);
        }

        // 2. the rest of the descent with ch0/ch1 frozen where they are, and ch2
        //    free to move. The proof run starts at 8 m, so its per-cell tracking
        //    window sees exactly that: two channels that do not move at all.
        for (int i = 20; i < 40; i++) both(1.05f, 1.10f, 1.15f, 7.0f + 25.0f * (i - 20) / 20.0f);

        // 3. the loop goes lean (the fault the diver would be fighting) and the
        //    machine follows it on the one cell it still believes: 0.15 bar is a
        //    MEASUREMENT here, LOW and all. The fall is also what lets ch2 prove
        //    itself — a cell proves itself by TRACKING the loop, so a proof run in
        //    which nothing moves can only fail (cellproof.cpp sets its clamp and
        //    marks nobody), and the ban this case needs comes from a PASS that
        //    marks the two frozen channels unproven. It is deliberately slow
        //    (0.03 bar/s): a cell climbing at that depth while the valve is shut is
        //    the stuck-open signature, and a latched stuck-open valve would make
        //    the "injection stopped" comparison below vacuous.
        // The scripted descent moves the cell readings faster than a real loop's
        // gas chemistry does, which trips the injector's stuck-open gate (a ppO2 rise
        // with the valve shut at steady depth) — the gate is correct, it is just not the
        // fault this case is about. Clear the artifact, and the asserts below check no
        // latch is standing when the window starts.
        dead.clearStuckOpen();
        live.clearStuckOpen();

        for (int i = 0; i < 350; i++) {
            const float v = 1.15f - 1.00f * static_cast<float>(i + 1) / 350.0f;
            both(1.05f, 1.10f, v, 32.0f);
        }
        TEST_ASSERT_TRUE_MESSAGE(dead.cellproof().state == cellproof::State::PASS,
                                 "CellProof did not reach PASS, so no cell is proof-banned and "
                                 "the case below cannot reach a failed vote");
        TEST_ASSERT_EQUAL_UINT8_MESSAGE(0b011, dead.cellproof().unprovenHighMask,
                                        "the two channels that never moved were not marked unproven");
        TEST_ASSERT_EQUAL_UINT8_MESSAGE(0b011, dead.proofExclusion(),
                                        "the proof ban never reached the voting layer");
        TEST_ASSERT_TRUE_MESSAGE(dead.haveVote(),
                                 "the rig lost its vote before the last channel died: the "
                                 "window below would be grading a different fault");
        TEST_ASSERT_TRUE_MESSAGE(dead.alarmBits() & (1u << alarms::BIT_LOW_PPO2),
                                 "a measured 0.15 bar loop did not raise LOW: the level alarms "
                                 "are not behaving as this case assumes");
        heldBar = dead.shownPpo2();
        pulsesDead = dead.pulseCount();
        pulsesLive = live.pulseCount();
        TEST_ASSERT_GREATER_THAN_UINT_MESSAGE(
            0, pulsesDead, "the valve was never pulsing, so 'injection stopped' below is vacuous");

        // 3. the last live channel dies. ch2 stops being fitted; ch0/ch1 are still
        //    fitted and still seen, and still proof-banned — so nothing may vote.
        Config c2 = wetCfg();
        c2.presentMask = 0b011;
        dead.setConfig(c2);
    }
};

}  // namespace

void test_a_failed_vote_holds_the_shown_ppo2_instead_of_showing_zero() {
    WetConnector w;
    const float held = w.heldBar;
    TEST_ASSERT_GREATER_THAN_FLOAT_MESSAGE(0.05f, held,
                                           "the last believable ppO2 was not believable");

    const double cns0 = w.dead.cnsPct(), cnsLive0 = w.live.cnsPct();
    const double fast0 = w.dead.deco().tissueBar(0);
    const uint32_t pulses0 = w.dead.pulseCount(), pulsesLive0 = w.live.pulseCount();
    int sensorFailAt = -1, lowDuring = 0, highDuring = 0, zeroShown = 0, rawNonZero = 0;

    // 150 s with the channel dead, fed a loop that has recovered to 0.90 bar —
    // which the dead rig cannot see, and must not invent.
    for (int i = 0; i < 1500; i++) {
        const float v = 0.15f + 0.75f * static_cast<float>(i + 1) / 1500.0f;
        w.both(1.05f, 1.10f, v, 32.0f);
        const ctrl::State& st = w.lastDead;

        // The vote layer still reports 0 on FAILED: the hold lives in the chain, so
        // no consumer can pick up a held number and call it a measurement.
        if (st.vote.votedPpo2 != 0.0f) rawNonZero++;
        if (st.haveVote) continue;  // graded separately below
        if (st.shownPpo2 <= 0.001f) zeroShown++;
        if (st.alarms.bits & (1u << alarms::BIT_SENSOR_FAIL)) {
            if (sensorFailAt < 0) sensorFailAt = i;
        }
        if (st.alarms.bits & (1u << alarms::BIT_LOW_PPO2)) lowDuring++;
        if (st.alarms.bits & (1u << alarms::BIT_HIGH_PPO2)) highDuring++;
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(1e-6, held, st.shownPpo2,
                                         "the published ppO2 moved during a failed vote: the "
                                         "shown number must be held until a vote is believable");
    }

    TEST_ASSERT_EQUAL_INT_MESSAGE(0, rawNonZero,
                                  "the raw vote stopped reporting 0 when it failed: consumers "
                                  "that must not believe a held value lost their guarantee");
    TEST_ASSERT_EQUAL_INT_MESSAGE(0, zeroShown,
                                  "a failed vote published 0.00 ppO2 again — a fabricated "
                                  "catastrophic hypoxia on the handset and in the dive file");
    TEST_ASSERT_TRUE_MESSAGE(sensorFailAt >= 0,
                             "BIT_SENSOR_FAIL never went up: the held number arrived with no "
                             "marker saying it is held");
    if (sensorFailAt >= 0)
        TEST_ASSERT_LESS_OR_EQUAL_INT_MESSAGE(5, sensorFailAt,
                                              "BIT_SENSOR_FAIL took more than a few cycles to "
                                              "follow a failed vote");
    TEST_ASSERT_EQUAL_INT_MESSAGE(0, lowDuring,
                                  "a LOW alarm stood while the vote was failed: LOW is a claim "
                                  "about a measurement, and in this window there is none — the "
                                  "held 0.15 must not be graded as a level, and neither must the "
                                  "raw 0.0 the vote reports on failure (the failure alarm is what "
                                  "the diver is owed)");
    TEST_ASSERT_EQUAL_INT_MESSAGE(0, highDuring, "HIGH fired off a held ppO2");

    // Injection: the rig that can see the loop pulses at these same numbers, the
    // rig that cannot must not pulse at all. The held 0.15 is well below the 1.00
    // setpoint, so a held value reaching the valve would pulse the whole window.
    TEST_ASSERT_EQUAL_UINT_MESSAGE(pulses0, w.dead.pulseCount(),
                                   "the valve pulsed with no believable ppO2: the held number "
                                   "drove injection");
    TEST_ASSERT_GREATER_THAN_UINT_MESSAGE(pulsesLive0, w.live.pulseCount(),
                                          "the control rig stopped pulsing, so the comparison "
                                          "above proves nothing");

    // The tissue model: decision (a) of finding N2. With no vote the tissues are
    // loaded at kNoVotePpo2Bar (0 bar ⇒ ppN2 = pAmb ⇒ the most inert gas this
    // depth allows), which is MORE debt than the held ppO2 would give. If someone
    // feeds the held number to stepTissues, the two rigs agree and this reddens.
    TEST_ASSERT_TRUE_MESSAGE(Loop::kNoVotePpo2Bar == 0.0f,
                             "kNoVotePpo2Bar is no longer the conservative 0 the deco decision "
                             "was made on (loop.h: no vote ⇒ ppN2 = pAmb)");
    const double fastDead = w.dead.deco().tissueBar(0), fastLive = w.live.deco().tissueBar(0);
    TEST_ASSERT_GREATER_THAN_FLOAT_MESSAGE(fast0 + 0.05, fastDead,
                                           "the tissues stopped integrating during a failed vote: "
                                           "the window's gas loading never reached the model");
    TEST_ASSERT_TRUE_MESSAGE(fastDead > fastLive + 0.015,
                             "the failed-vote rig loaded its tissues like the rig that still had "
                             "a vote: the deco model is being fed the held ppO2 instead of "
                             "kNoVotePpo2Bar, which under-records decompression debt");

    // CNS: frozen with nothing to integrate. Stated residual (loop.cpp step 8),
    // pinned so it stays a decision and not an accident waiting to be forgotten.
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(1e-9, cns0, w.dead.cnsPct(),
                                     "the CNS clock advanced on a held ppO2 — either the "
                                     "residual changed or it changed silently");
    TEST_ASSERT_GREATER_THAN_FLOAT_MESSAGE(cnsLive0 + 0.01, w.live.cnsPct(),
                                           "the control rig stopped clocking CNS, so the frozen "
                                           "clock above proves nothing");

    // Recovery: the channel comes back, the vote comes back with it, and the shown
    // number steps to the fresh measurement — never through 0.00.
    Config back = wetCfg();
    back.presentMask = 0b111;
    w.dead.setConfig(back);
    // The reading coming back at 0.90 after minutes of nothing IS, to the
    // injector's stuck-open gate, a ppO2 rising with the valve shut at steady
    // depth — the latch is the gate doing its job, not a new fault, so the case
    // clears it the way the diver would (`CLEAR STUCK`) before asking whether
    // control resumed.
    w.dead.clearStuckOpen();
    const uint32_t pulsesAtRecovery = w.dead.pulseCount();
    bool voteBack = false;
    for (int i = 0; i < 100; i++) {
        w.both(1.05f, 1.10f, 0.90f, 32.0f);
        if (w.dead.haveVote()) { voteBack = true; break; }
    }
    TEST_ASSERT_TRUE_MESSAGE(voteBack, "the vote never came back when the channel did");
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.02f, 0.90f, w.dead.shownPpo2(),
                                     "after recovery the shown ppO2 is not the fresh vote");
    TEST_ASSERT_FALSE_MESSAGE(w.dead.alarmBits() & (1u << alarms::BIT_SENSOR_FAIL),
                              "BIT_SENSOR_FAIL stayed up after the vote returned");
    TEST_ASSERT_GREATER_THAN_UINT_MESSAGE(pulsesAtRecovery, w.dead.pulseCount(),
                                          "injection did not resume when the vote did");
}

// The row a diver would download from that window: logfmt::fillSnapshot fed the
// State of a chain that is actually in it, which is what main.cpp does. This is
// the assert-level version of the rowfill.h contract — the ppO2 column is the
// held number, and the marker bytes are what say so.
void test_the_row_written_in_a_failed_vote_holds_the_number_and_marks_it() {
    WetConnector w;
    for (int i = 0; i < 400; i++) w.both(1.05f, 1.10f, 0.60f, 32.0f);
    const ctrl::State& st = w.lastDead;
    TEST_ASSERT_FALSE_MESSAGE(st.haveVote, "the rig recovered before the row was written");

    logfmt::RowInputs ri{};
    ri.nowMs = w.nd;
    ri.diveStartMs = 0;
    ri.smoothedDepthM = st.depthSmoothedM;
    ri.haveVote = st.haveVote;
    ri.shownPpo2 = st.shownPpo2;
    for (int i = 0; i < 3; i++) ri.cellBar[i] = w.dead.cellBar(static_cast<uint8_t>(i));
    ri.setpointBar = st.setpointBar;
    ri.diveActive = true;
    ri.autoMode = st.autoMode;
    ri.alarmLogBits = st.alarms.logBits;
    ri.vbat = 12.4f;
    ri.deco = &w.dead.deco();

    logfmt::Snapshot row{};
    logfmt::fillSnapshot(row, ri);

    TEST_ASSERT_TRUE_MESSAGE(row.votedX1000 > 0,
                             "the dive file recorded a 0.000 ppO2 row for a window in which the "
                             "diver was shown a held value — the row and the handset disagree");
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(1e-3, w.heldBar, static_cast<float>(row.votedX1000) / 1000.0f,
                                     "the row's ppO2 column is not the held value the chain is "
                                     "publishing");
    TEST_ASSERT_TRUE_MESSAGE(row.alarmBits & (1u << alarms::BIT_SENSOR_FAIL),
                             "the row carries a ppO2 that is not a measurement with nothing "
                             "saying so: BIT_SENSOR_FAIL must be in the log alarm word");
    TEST_ASSERT_EQUAL_INT16_MESSAGE(480, row.ndlMin,
                                    "a row with no vote does not carry the 480 sentinel: the "
                                    "held ppO2 was allowed to compute an NDL");
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
    RUN_TEST(test_an_ack_silences_a_dead_depth_bus_on_the_wire_but_not_in_the_log);
    RUN_TEST(test_cns_freezes_without_a_trustworthy_ppo2_and_that_is_paired_with_an_alarm);
    RUN_TEST(test_pre_dive_refuses_a_dead_depth_bus);
    RUN_TEST(test_pre_dive_rides_through_a_short_depth_bus_dropout);
    RUN_TEST(test_a_failed_vote_holds_the_shown_ppo2_instead_of_showing_zero);
    RUN_TEST(test_the_row_written_in_a_failed_vote_holds_the_number_and_marks_it);
    return UNITY_END();
}
