// ============================================================================
// test_router — the command surface every client shares (handset UART2 and
// the web WebSocket). What matters here is not that commands work but that
// the refusals work: the pre-dive AUTO lockout, the dive-locked settings,
// the radio that must stay down during a dive, and the fact that no client,
// handset or browser, has a second path around any of it.
// ============================================================================
#include <unity.h>

#include <cstdio>
#include <cstring>

#include "comms/cmd_router.h"
#include "comms/settings_apply.h"

using comms::CommandRouter;
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

struct Bench {
    Config cfg = benchCfg();
    Loop loop{cfg};
    settings::Registry reg;
    uint32_t now = 0;
    CommandRouter router;

    Bench() : reg(), router(loop, reg, now) { reg.resetToDefaults(); }
    CommandRouter::Response run(const char* line) {
        router.setNowMs(now);
        return router.exec(line);
    }
    void ticks(float bar, float depth, float seconds) {
        const uint32_t n = static_cast<uint32_t>(seconds * 1000.0f / Loop::kTickMs);
        for (uint32_t i = 0; i < n; i++) {
            SensorData sd;
            for (int ch = 0; ch < 3; ch++) sd.cells[ch] = bar;
            sd.depthM = depth;
            sd.vbat = 12.4f;
            now += Loop::kTickMs;
            loop.cycle(sd, now);
        }
    }
};

bool has(const CommandRouter::Response& r, const char* needle) {
    return std::strstr(r.text, needle) != nullptr;
}

// Crude gas plant for the standalone cell test: cells rise with every pulse
// that actually left the valve, then drift back. Enough for PulseTest to see
// the rise/recover it judges; the physics lives in sim/world.
struct Plant {
    Loop& loop;
    float bar[3] = {0.50f, 0.50f, 0.50f};
    uint32_t pulses = 0;
    void tick(CommandRouter& router, uint32_t& now) {
        const uint32_t p = loop.pulseCount();
        while (pulses < p) { pulses++; for (int i = 0; i < 3; i++) bar[i] += 0.05f; }
        for (int i = 0; i < 3; i++) bar[i] -= 0.002f;
        SensorData sd;
        sd.cells[0] = bar[0]; sd.cells[1] = bar[1]; sd.cells[2] = bar[2];
        sd.depthM = 0.0f; sd.vbat = 12.4f;
        now += Loop::kTickMs;
        router.setNowMs(now);
        loop.cycle(sd, now);
    }
};

} // namespace

void test_auto_lockout_holds_over_the_router() {
    Bench b;
    CommandRouter::Response r = b.run("AUTO");
    TEST_ASSERT_TRUE_MESSAGE(has(r, "ERR"), "AUTO accepted with no pre-dive check");
    TEST_ASSERT_FALSE(b.loop.autoMode());

    // The raw switch is simulator/factory-only and is NOT reachable from any
    // command line: prove AUTO stays refused even with auto force-set.
    b.loop.setAutoMode(true);
    r = b.run("MANUAL");
    TEST_ASSERT_TRUE(has(r, "OK"));
    TEST_ASSERT_FALSE(b.loop.autoMode());
    r = b.run("AUTO");
    TEST_ASSERT_TRUE_MESSAGE(has(r, "ERR"), "AUTO bypassed the pre-dive lockout");
    TEST_ASSERT_FALSE(b.loop.autoMode());
}

void test_get_set_round_trip_on_the_shared_registry() {
    Bench b;
    CommandRouter::Response r = b.run("SET SP2=0.55");
    TEST_ASSERT_TRUE(has(r, "OK"));
    TEST_ASSERT_TRUE(r.actions & CommandRouter::ACT_SETTINGS);
    TEST_ASSERT_FLOAT_WITHIN(0.0001, 0.55f, b.reg.getFloat(b.reg.find("SP2")));

    r = b.run("GET SP2");
    TEST_ASSERT_TRUE(has(r, "SP2=0.550"));

    r = b.run("SET SP2=2.5");
    TEST_ASSERT_TRUE(has(r, "ERR"));
    TEST_ASSERT_FLOAT_WITHIN(0.0001, 0.55f, b.reg.getFloat(b.reg.find("SP2")));

    r = b.run("GET *");
    TEST_ASSERT_TRUE(r.len > 300);
    TEST_ASSERT_TRUE(has(r, "SP1=0.400\n"));

    // safety-flagged writes answer OK SAFETY so the client event-logs them
    r = b.run("SET GF_LOW=35");
    TEST_ASSERT_TRUE(has(r, "SAFETY"));

    r = b.run("GET NOPE");
    TEST_ASSERT_TRUE(has(r, "ERR"));
    r = b.run("FROBNICATE");
    TEST_ASSERT_TRUE(has(r, "ERR"));
    r = b.run("auto");  // lowercase is not a command
    TEST_ASSERT_TRUE(has(r, "ERR"));
    TEST_ASSERT_FALSE(b.loop.autoMode());
}

void test_dive_locked_keys_refused_mid_dive() {
    Bench b;
    TEST_ASSERT_TRUE(has(b.run("DIVE START"), "OK"));
    TEST_ASSERT_TRUE(b.loop.diveActive());

    CommandRouter::Response r = b.run("SET SP2=0.40");
    TEST_ASSERT_TRUE_MESSAGE(has(r, "ERR"), "setpoint rewritten under a diver");
    r = b.run("SET AP_PASS=otherpass");
    TEST_ASSERT_TRUE(has(r, "ERR"));
    // the refused write must not have landed
    TEST_ASSERT_EQUAL_STRING("diveonly", b.reg.getString(b.reg.find("AP_PASS")));

    // schedule tuning stays writable mid-dive (normal diving, not gas control)
    r = b.run("SET GF_HIGH=90");
    TEST_ASSERT_TRUE(has(r, "OK"));
    TEST_ASSERT_EQUAL(90, b.reg.getInt(b.reg.find("GF_HIGH")));

    r = b.run("DIVE START");
    TEST_ASSERT_TRUE(has(r, "ERR"));  // no double-start
}

void test_dive_start_drops_the_radio_and_end_restores_it() {
    Bench b;
    CommandRouter::Response r = b.run("DIVE START");
    TEST_ASSERT_TRUE(r.actions & CommandRouter::ACT_DIVE_START);
    TEST_ASSERT_TRUE(r.actions & CommandRouter::ACT_WIFI);
    TEST_ASSERT_FALSE(b.router.wifiEnabled());
    TEST_ASSERT_TRUE(b.router.wifiAutoOff());

    r = b.run("WIFI ON");
    TEST_ASSERT_TRUE_MESSAGE(has(r, "ERR"), "radio reopened during a dive");
    TEST_ASSERT_FALSE(b.router.wifiEnabled());

    r = b.run("DIVE END");
    TEST_ASSERT_TRUE(r.actions & CommandRouter::ACT_DIVE_END);
    TEST_ASSERT_TRUE(r.actions & CommandRouter::ACT_WIFI);
    TEST_ASSERT_TRUE(b.router.wifiEnabled());

    // If the diver had WiFi off before the dive, dive end must NOT turn it on.
    r = b.run("WIFI OFF");
    TEST_ASSERT_TRUE(has(r, "OK"));
    r = b.run("DIVE START");
    TEST_ASSERT_FALSE(b.router.wifiAutoOff());  // not our doing
    r = b.run("DIVE END");
    TEST_ASSERT_FALSE(b.router.wifiEnabled());
    TEST_ASSERT_FALSE(r.actions & CommandRouter::ACT_WIFI);

    r = b.run("DIVE END");
    TEST_ASSERT_TRUE(has(r, "ERR"));  // already ended
}

void test_diver_outranks_the_machine_cellproof_override() {
    Bench b;
    CommandRouter::Response r = b.run("CELLPROOF OVERRIDE");
    TEST_ASSERT_TRUE(has(r, "ERR"));  // nothing to lift: honest answer

    // Drive the real chain into a CellProof FAIL: arm shallow, descend past
    // the arm depth with cells that can never reach the proof level, wait out
    // the proof window.
    b.run("DIVE START");
    b.ticks(0.60f, 5.0f, 2.0f);     // shallow: the proof ARMS on descent
    b.ticks(0.60f, 12.0f, 70.0f);   // cross 10 m -> RUNNING; 60 s -> FAIL + clamp
    TEST_ASSERT_EQUAL_UINT8(cellproof::State::FAIL, b.loop.cellproof().state);
    TEST_ASSERT_TRUE(b.loop.cellproof().clampActive);

    // The diver lifts the clamp. The reply is the proof it happened — the
    // cpOut_ snapshot the UI reads is one cycle stale by design (it is
    // published by Loop::cycle), so confirm through a tick as well.
    r = b.run("CELLPROOF OVERRIDE");
    TEST_ASSERT_TRUE_MESSAGE(has(r, "CLAMP LIFTED"),
                             "diver could not lift the clamp (diver outranks machine)");
    b.ticks(0.60f, 12.0f, 0.3f);
    TEST_ASSERT_EQUAL_UINT8(cellproof::State::PASS, b.loop.cellproof().state);
    TEST_ASSERT_FALSE(b.loop.cellproof().clampActive);

    // A second override has nothing to lift.
    r = b.run("CELLPROOF OVERRIDE");
    TEST_ASSERT_TRUE(has(r, "ERR"));

    r = b.run("CHECK");  // pre-dive checks only on the surface
    TEST_ASSERT_TRUE(has(r, "ERR"));
}

void test_ack_always_honoured() {
    Bench b;
    CommandRouter::Response r = b.run("ACK");
    TEST_ASSERT_TRUE(has(r, "OK"));
    TEST_ASSERT_EQUAL_STRING("OK\n", r.text);  // no argument, no conditions
    TEST_ASSERT_EQUAL(0u, r.actions);          // and no hidden side effects
}

void test_testcells_runs_on_the_bench_and_reports_masks() {
    Bench b;
    b.ticks(0.50f, 0.0f, 3.0f);
    CommandRouter::Response r = b.run("TEST CELLS");
    TEST_ASSERT_TRUE(has(r, "OK"));
    TEST_ASSERT_TRUE(b.loop.cellTestRequested());
    r = b.run("TEST CELLS");
    TEST_ASSERT_TRUE(has(r, "ERR"));  // one at a time

    Plant plant{b.loop};
    for (int i = 0; i < 250 && b.loop.cellTestRequested(); i++) plant.tick(b.router, b.now);
    TEST_ASSERT_FALSE_MESSAGE(b.loop.cellTestRequested(), "standalone test never finished");
    TEST_ASSERT_EQUAL_UINT8(cal::TestState::DONE, b.loop.pulseTestState());
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(0b111, b.loop.cellTestPassMask(),
                                    "responsive cells reported as failed");

    r = b.run("GET CELLS");
    TEST_ASSERT_TRUE(has(r, "CELLS,4,"));   // DONE
    TEST_ASSERT_TRUE(has(r, "007,"));       // pass mask, all three, octal
}

void test_deco_and_status_lines_come_from_the_chain() {
    Bench b;
    b.ticks(0.55f, 0.0f, 5.0f);
    CommandRouter::Response r = b.run("DECO");
    // D,<ndl>,<total>,<ceiling>,[stops...],<cns> — at the surface: no stops,
    // zero ceiling, and the total is the literal 0.0, not a stop list.
    TEST_ASSERT_TRUE(r.text[0] == 'D' && r.text[1] == ',');
    TEST_ASSERT_NOT_NULL(std::strchr(r.text, '\n'));
    TEST_ASSERT_NULL(std::strchr(r.text, '|'));  // no stop entries
    r = b.run("GET D");
    TEST_ASSERT_TRUE(r.text[0] == 'D' && r.text[1] == ',');

    r = b.run("GET T");
    TEST_ASSERT_TRUE(r.text[0] == 'T' && r.text[1] == ',');
    r = b.run("GET PRE");
    TEST_ASSERT_TRUE(has(r, "PRE,"));
    r = b.run("GET FAULT");
    TEST_ASSERT_TRUE(has(r, "FAULT,0,0"));  // bench, no faults

    // a real key wins over a display key that would otherwise shadow it
    r = b.run("GET D1");
    TEST_ASSERT_TRUE(has(r, "D1=30.000"));
}

void test_set_actually_moves_the_live_chain() {
    // The full handset path: SET -> registry -> ACT_SETTINGS -> applier ->
    // live loop. Pins that the firmware's single applier actually changes
    // behaviour, not just stored text.
    Bench b;
    CommandRouter::Response r = b.run("SET ALLO_LOW=0.30");
    TEST_ASSERT_TRUE(r.actions & CommandRouter::ACT_SETTINGS);
    TEST_ASSERT_EQUAL_UINT8(comms::CLAMP_NONE, comms::applySettings(b.reg, b.cfg));
    b.loop.setConfig(b.cfg);

    b.ticks(0.25f, 0.0f, 5.0f);  // 0.25 bar: above the old 0.18 floor, below the new
    TEST_ASSERT_TRUE_MESSAGE(b.loop.alarmBits() & (1u << alarms::BIT_LOW_PPO2),
                             "SET ALLO_LOW never reached the live alarm engine");
}

void test_log_and_pulse_and_calibration_commands_delegations() {
    Bench b;
    CommandRouter::Response r = b.run("LOG LIST");
    TEST_ASSERT_TRUE(r.actions & CommandRouter::ACT_LOG_LIST);
    r = b.run("LOG EXPORT 3");
    TEST_ASSERT_TRUE(r.actions & CommandRouter::ACT_LOG_EXPORT);
    TEST_ASSERT_EQUAL_UINT16(3, b.router.logExportIndex());

    // PULSE is a bench command, not a bypass: it asks the injector, which
    // still owns wall/cutoff/duty/fuse. Reply is a plain OK.
    r = b.run("PULSE");
    TEST_ASSERT_TRUE(has(r, "OK"));

    // two-step CAL O2 handshake, enforced in one place for every client
    r = b.run("CAL O2 CONFIRM");
    TEST_ASSERT_TRUE(has(r, "ERR"));  // confirm without arm: refused
    r = b.run("CAL O2");
    TEST_ASSERT_TRUE(r.actions & CommandRouter::ACT_CAL_O2_ARM);
    r = b.run("CAL O2 CONFIRM");
    TEST_ASSERT_TRUE(r.actions & CommandRouter::ACT_CAL_O2_CONFIRM);
    r = b.run("CAL O2");
    TEST_ASSERT_TRUE(r.actions & CommandRouter::ACT_CAL_O2_ARM);
    r = b.run("CAL O2");  // bare CAL O2 while armed = back out
    TEST_ASSERT_TRUE(r.actions & CommandRouter::ACT_CAL_O2_CANCEL);

    r = b.run("CAL AIR");
    TEST_ASSERT_TRUE(r.actions & CommandRouter::ACT_CAL_AIR);
    r = b.run("CAL DEPTH");
    TEST_ASSERT_TRUE(r.actions & CommandRouter::ACT_CAL_DEPTH);
    TEST_ASSERT_TRUE(b.router.takeDepthCalRequest());
    TEST_ASSERT_FALSE(b.router.takeDepthCalRequest());  // one-shot

    // calibration never runs under a diver
    b.run("DIVE START");
    r = b.run("CAL AIR");
    TEST_ASSERT_TRUE(has(r, "ERR"));
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_auto_lockout_holds_over_the_router);
    RUN_TEST(test_get_set_round_trip_on_the_shared_registry);
    RUN_TEST(test_dive_locked_keys_refused_mid_dive);
    RUN_TEST(test_dive_start_drops_the_radio_and_end_restores_it);
    RUN_TEST(test_diver_outranks_the_machine_cellproof_override);
    RUN_TEST(test_ack_always_honoured);
    RUN_TEST(test_testcells_runs_on_the_bench_and_reports_masks);
    RUN_TEST(test_deco_and_status_lines_come_from_the_chain);
    RUN_TEST(test_set_actually_moves_the_live_chain);
    RUN_TEST(test_log_and_pulse_and_calibration_commands_delegations);
    return UNITY_END();
}
