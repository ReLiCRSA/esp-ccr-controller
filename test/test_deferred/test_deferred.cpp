// ============================================================================
// test_deferred — the A5e contract: the command path and the control chain
// never touch the engines at the same time, and the diver is never made to
// wait for that promise.
//
// The split (loop.h): gates (dive active, auto, cell test) flip synchronously,
// because dive-locked refusals and telemetry mode ask for them the same
// millisecond the button was pressed; chain work (config re-tune, dive-state
// reset, sequencer start, ack latch) is queued and applied at
// the top of the next cycle() — on the control thread. The handset is the
// in-water transport (the web is off during a dive), so "lands within one
// 100 ms tick" is a lifeline property, and it is asserted here, not assumed.
//
// Without these cases the race is invisible on the host and loads on the
// board: a SET retuning Zhl16Ccr's config half-way through its tissue step on
// the other core, or a DIVE START resetting proof state while cycle() is
// reading it, is not a wrong number — it is a wrong number *sometimes*.
// ============================================================================
#include <unity.h>

#include <atomic>
#include <cmath>
#include <thread>

#include "control/loop.h"

using namespace ctrl;

void setUp() {}
void tearDown() {}

namespace {

Config benchCfg() {
    Config c;
    c.predive.warmupMs = 1000;
    c.inj.minGapMs = 200;
    return c;
}

void tick(Loop& loop, uint32_t& now, uint32_t n = 1, float depth = 0.0f,
          float cell = 0.55f) {
    for (uint32_t i = 0; i < n; i++) {
        SensorData sd;
        for (int ch = 0; ch < 3; ch++) sd.cells[ch] = cell;
        sd.depthM = depth;
        sd.vbat = 12.4f;
        now += Loop::kTickMs;
        loop.cycle(sd, now);
    }
}

} // namespace

// The dive GATE is immediate — that is what makes a dive-locked SET refused
// on the very next command, with no cycle in between (test_router relies on
// this). The chain WORK waits for the tick boundary. And a second start
// mid-dive must not re-run the reset: tissues and proof state are the diver's
// safety record, not something a stray command may zero.
void test_the_dive_gate_is_now_and_a_second_start_cannot_wipe_the_dive() {
    Config cfg = benchCfg();
    Loop loop{cfg};
    uint32_t now = 0;
    tick(loop, now, 3);
    TEST_ASSERT_FALSE(loop.diveActive());

    loop.postStartDive(now);
    TEST_ASSERT_TRUE_MESSAGE(loop.diveActive(),
                             "dive gate lagged: dive-locked checks would pass mid-dive");

    // dive at 18 m on 1.7 bar: CNS accumulates through the deco engine
    tick(loop, now, 30, 18.0f, 1.70f);
    TEST_ASSERT_TRUE_MESSAGE(loop.cnsPct() > 0.0, "deco never accumulated at 1.7 bar");
    const float cns = loop.cnsPct();

    loop.postStartDive(now);  // the router refuses this; the chain must too
    tick(loop, now, 3, 18.0f, 1.70f);
    TEST_ASSERT_TRUE(loop.diveActive());
    TEST_ASSERT_TRUE_MESSAGE(loop.cnsPct() >= cns,
                             "a second DIVE START reset the dive mid-dive");

    loop.postEndDive(now);
    TEST_ASSERT_FALSE_MESSAGE(loop.diveActive(),
                              "dive gate lagged on END: wifi/lock checks see a stale dive");
    tick(loop, now, 2);
    TEST_ASSERT_FALSE(loop.diveActive());
}

// The queued config must decide the SAME tick it lands in — step 0 of cycle()
// runs before sensors — otherwise a setpoint/alarm change quietly costs an
// extra 100 ms of diving under the old rule.
void test_a_queued_config_decides_the_same_tick_it_lands() {
    Config cfg = benchCfg();
    Loop loop{cfg};
    uint32_t now = 0;
    tick(loop, now, 30, 0.0f, 0.25f);  // 0.25 bar: above the 0.18 default floor
    TEST_ASSERT_FALSE(loop.alarmBits() & (1u << alarms::BIT_LOW_PPO2));

    Config cfg2 = cfg;
    cfg2.alarms.lowBar = 0.30f;
    loop.postConfig(cfg2);
    // Nothing may change until the chain applies it...
    TEST_ASSERT_FALSE(loop.alarmBits() & (1u << alarms::BIT_LOW_PPO2));
    // ...and the very next cycle must already decide under it.
    tick(loop, now, 1, 0.0f, 0.25f);
    TEST_ASSERT_TRUE_MESSAGE(loop.alarmBits() & (1u << alarms::BIT_LOW_PPO2),
                             "queued config took more than the tick it was queued for");
}

// CHECK queues the sequencer; the sequencer itself only ever runs inside
// cycle(), on the control thread, and it starts at the stamp the command
// carried — not at "whenever the tick happened to land".
void test_a_posted_predive_starts_on_the_next_tick() {
    Config cfg = benchCfg();
    Loop loop{cfg};
    uint32_t now = 0;
    tick(loop, now, 3);

    loop.postPredive(now + Loop::kTickMs);
    TEST_ASSERT_FALSE(loop.prediveRunning());
    TEST_ASSERT_EQUAL_UINT8(predive::PENDING, loop.prediveStatus(predive::S_WARMUP));

    tick(loop, now, 1);
    TEST_ASSERT_TRUE(loop.prediveRunning());
    TEST_ASSERT_EQUAL_UINT8(predive::RUNNING, loop.prediveStatus(predive::S_WARMUP));
}

// The pulse test is now the ONLY producer of test pulses (`PULSE` retired
// 2026-11-26, HANDSET_PROTOCOL.md §8), and this case keeps the property the
// old `PULSE` regression taught the hard way: a pulse that a live test asked
// for must reach the valve — the "withdraw the stopped test's leftovers"
// cancel must not eat it between request and delivery — and a test that
// finished must leave nothing behind in the injector. `TEST CELLS` is the
// smallest live producer: no dive, no pre-dive chain, same engine, same
// cancel edge. A pulse that was asked for and silently never fired used to be
// worse than a refusal; now a whole pulse class of silent no-ops is gone, and
// this pins that the surviving class still lands.
void test_a_pulsing_test_gets_every_pulse_and_leaves_no_leftovers() {
    Config cfg = benchCfg();
    Loop loop{cfg};
    uint32_t now = 0;
    tick(loop, now, 2);
    TEST_ASSERT_EQUAL_UINT32(0, loop.pulseCount());

    loop.requestCellTest();
    // 200 ticks = 20 s of simulated time: BASELINE (6 samples), the three
    // requested pulses, the 8 s verdict hold (cells sit at 0.55 and never
    // rise, so the test must finish on schedule, not on success), RECOVER,
    // DONE. Exactly `cfg.pulseTest.pulses` may leave the valve in all of it:
    // short by one and a live request was cancelled; over and a leftover
    // outlived the test.
    tick(loop, now, 200);
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(cfg.pulseTest.pulses, loop.pulseCount(),
                                     "the pulse test did not get exactly the pulses it asked for");
    TEST_ASSERT_FALSE_MESSAGE(loop.cellTestRequested(),
                              "the standalone cell test never closed its gate");

    // And once it is over, the valve stays quiet: three 300 ms pulses use
    // 900 ms of the 1000 ms duty budget, so the window still has room — a
    // leftover would fire within these ticks, not hide under the cap.
    const uint32_t settled = loop.pulseCount();
    tick(loop, now, 100);
    TEST_ASSERT_EQUAL_UINT32(settled, loop.pulseCount());

    // The cancel half of the rule needs a run where the engine gives up while
    // a queued pulse has NOT yet reached the valve — the fast run above never
    // throttles, so no leftover can form there. 10 s min-gap (counted from the
    // END of the previous pulse) puts the train at ~0 / +10.3 / +21.2 s: the
    // 20 s verdict timeout lands with 2 of 3 delivered and ONE still queued.
    // That one belongs to a finished test — the edge-triggered cancel must
    // withdraw it, and the count must then never move. If the cancel ever
    // stops cancelling, the third pulse arrives at ~21.2 s, inside the tail
    // below, and the count moves. This is what makes `cancelTestPulse` itself,
    // not just the pulses, a tested rule.
    Config slow = benchCfg();
    slow.inj.minGapMs = 10000;
    Loop slowLoop{slow};
    uint32_t snow = 0;
    tick(slowLoop, snow, 2);
    slowLoop.requestCellTest();
    tick(slowLoop, snow, 210);   // baseline + pulses + the 20 s verdict + DONE
    const uint32_t throttled = slowLoop.pulseCount();
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(2, throttled,
        "throttled run should deliver exactly 2 of 3 before the verdict");
    tick(slowLoop, snow, 150);   // 15 more s: a surviving leftover would land
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(throttled, slowLoop.pulseCount(),
        "a leftover outlived its pulse test — the cancel edge stopped cancelling");
}

// AUTO is a gate, checked against the sequencer's own state where it lives:
// an in-flight pre-dive cannot be talked into granting AUTO.
void test_the_auto_gate_and_the_lockout_stay_synchronous() {
    Config cfg = benchCfg();
    Loop loop{cfg};
    uint32_t now = 0;
    tick(loop, now, 3);

    TEST_ASSERT_FALSE(loop.requestAuto(true));
    TEST_ASSERT_FALSE(loop.autoMode());

    loop.postPredive(now + Loop::kTickMs);
    tick(loop, now, 1);
    TEST_ASSERT_TRUE(loop.prediveRunning());
    TEST_ASSERT_FALSE_MESSAGE(loop.requestAuto(true),
                              "AUTO granted while pre-dive checks are still running");
    TEST_ASSERT_FALSE(loop.autoMode());

    loop.setAutoMode(true);  // the raw switch stays synchronous (sim/factory)
    TEST_ASSERT_TRUE(loop.autoMode());
    TEST_ASSERT_FALSE(loop.requestAuto(false));
    TEST_ASSERT_FALSE(loop.autoMode());
}

// The real shape: ControlTask cycling on one core, the command path posting
// from another, both for as long as the test runs. Nothing here asserts a
// delicate interleaving — interleaving is the point. What must survive any
// interleaving: the chain ends up *some* consistent application of the posted
// requests (last config wins, dive closed, vote sane), never a half-applied
// config, a dive that will not close, or NaN through a tissue step.
void test_two_threads_never_split_the_chain() {
    Config cfgA = benchCfg();
    Config cfgB = benchCfg();
    cfgB.sp.sp[0] = 1.20f;
    cfgB.sp.sp[1] = 1.50f;  // 5 m depth -> SP2 band

    Loop loop{cfgA};
    std::atomic<bool> stop{false};
    std::atomic<uint32_t> cycles{0};

    std::thread control([&] {
        uint32_t now = 0;
        while (!stop.load()) {
            SensorData sd;
            for (int ch = 0; ch < 3; ch++) sd.cells[ch] = 0.55f;
            sd.depthM = 5.0f;
            sd.vbat = 12.2f;
            now += 1;  // faster than the board's 100 ms: more edges per second
            loop.cycle(sd, now);
            cycles.fetch_add(1);
        }
    });

    std::thread handset([&] {
        Config c = cfgA;
        uint32_t t = 0;
        while (!stop.load()) {
            c.sp.sp[0] = (c.sp.sp[0] < 1.0f) ? 1.20f : 0.40f;
            c.sp.sp[1] = (c.sp.sp[1] < 1.0f) ? 1.50f : 0.70f;
            loop.postConfig(c);
            loop.postAckAlarms();
            loop.postPredive(t++);
            loop.postCpOverride();
            loop.postStartDive(t);
            loop.postEndDive(t);
            loop.requestCellTest();
        }
    });

    // let them fight for a while, then quiesce from the control side
    while (cycles.load() < 20000) {}
    stop.store(true);
    handset.join();
    control.join();

    // last word from the command path, then drain on this thread
    loop.postConfig(cfgB);
    loop.postEndDive(0);
    uint32_t now = 0;
    tick(loop, now, 3, 5.0f, 0.55f);

    TEST_ASSERT_FALSE_MESSAGE(loop.diveActive(), "a posted END could not close the dive");
    TEST_ASSERT_TRUE_MESSAGE(loop.haveVote(), "the vote died under concurrent posting");
    // The published ppO2, because that is the number a torn sample would corrupt
    // on a handset: with haveVote true it is the vote, and a torn read would show up
    // as a value that is not one any cell reported.
    TEST_ASSERT_TRUE_MESSAGE(std::isfinite(loop.shownPpo2()),
                             "a torn sample produced a NaN ppO2 on the published state");
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.01, 0.55f, loop.shownPpo2(),
                                     "the reported ppO2 drifted under concurrent config churn");
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.01, 1.50f, loop.setpointBar(),
                                     "the final config did not land whole (SP2 band at 5 m)");
    TEST_ASSERT_FLOAT_WITHIN(0.01, 5.0f, loop.depthSmoothedM());
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_the_dive_gate_is_now_and_a_second_start_cannot_wipe_the_dive);
    RUN_TEST(test_a_queued_config_decides_the_same_tick_it_lands);
    RUN_TEST(test_a_posted_predive_starts_on_the_next_tick);
    RUN_TEST(test_a_pulsing_test_gets_every_pulse_and_leaves_no_leftovers);
    RUN_TEST(test_the_auto_gate_and_the_lockout_stay_synchronous);
    RUN_TEST(test_two_threads_never_split_the_chain);
    return UNITY_END();
}
