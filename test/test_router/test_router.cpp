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

    // The reply buffer is storage the CALLER owns, as it is on the target:
    // CommandRouter::Response is not copyable (see cmd_router.h for the crash
    // that made it that way), so no test — and no transport — can move one
    // through a stack frame by value. `resp` is the bench's own buffer; a test
    // that must hold two replies at once passes its own.
    CommandRouter::Response resp;
    Bench() : reg(), router(loop, reg, now) { reg.resetToDefaults(); }
    const CommandRouter::Response& run(const char* line) {
        router.setNowMs(now);
        router.exec(line, resp);
        return resp;
    }
    void run(const char* line, CommandRouter::Response& out) {
        router.setNowMs(now);
        router.exec(line, out);
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

// replies are exactly one protocol line; a second '\n' inside the reply means
// either a leaked tail or a command that answered twice
size_t countLines(const char* text, size_t len) {
    size_t n = 0;
    for (size_t i = 0; i < len; ++i) if (text[i] == '\n') ++n;
    return n;
}

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
    CommandRouter::Response r;
        b.run("AUTO", r);
    TEST_ASSERT_TRUE_MESSAGE(has(r, "ERR"), "AUTO accepted with no pre-dive check");
    TEST_ASSERT_FALSE(b.loop.autoMode());

    // The raw switch is simulator/factory-only and is NOT reachable from any
    // command line: prove AUTO stays refused even with auto force-set.
    b.loop.setAutoMode(true);
    b.run("MANUAL", r);
    TEST_ASSERT_TRUE(has(r, "OK"));
    TEST_ASSERT_FALSE(b.loop.autoMode());
    b.run("AUTO", r);
    TEST_ASSERT_TRUE_MESSAGE(has(r, "ERR"), "AUTO bypassed the pre-dive lockout");
    TEST_ASSERT_FALSE(b.loop.autoMode());
}

void test_get_set_round_trip_on_the_shared_registry() {
    Bench b;
    CommandRouter::Response r;
        b.run("SET SP2=0.55", r);
    TEST_ASSERT_TRUE(has(r, "OK"));
    TEST_ASSERT_TRUE(r.actions & CommandRouter::ACT_SETTINGS);
    TEST_ASSERT_FLOAT_WITHIN(0.0001, 0.55f, b.reg.getFloat(b.reg.find("SP2")));

    b.run("GET SP2", r);
    TEST_ASSERT_TRUE(has(r, "SP2=0.550"));

    b.run("SET SP2=2.5", r);
    TEST_ASSERT_TRUE(has(r, "ERR"));
    TEST_ASSERT_FLOAT_WITHIN(0.0001, 0.55f, b.reg.getFloat(b.reg.find("SP2")));

    b.run("GET *", r);
    TEST_ASSERT_TRUE(r.len > 300);
    TEST_ASSERT_TRUE(has(r, "SP1=0.400\n"));

    // safety-flagged writes answer OK SAFETY so the client event-logs them
    b.run("SET GF_LOW=35", r);
    TEST_ASSERT_TRUE(has(r, "SAFETY"));

    b.run("GET NOPE", r);
    TEST_ASSERT_TRUE(has(r, "ERR"));
    b.run("FROBNICATE", r);
    TEST_ASSERT_TRUE(has(r, "ERR"));
    b.run("auto", r);  // lowercase is not a command
    TEST_ASSERT_TRUE(has(r, "ERR"));
    TEST_ASSERT_FALSE(b.loop.autoMode());
}

void test_dive_locked_keys_refused_mid_dive() {
    Bench b;
    TEST_ASSERT_TRUE(has(b.run("DIVE START"), "OK"));
    TEST_ASSERT_TRUE(b.loop.diveActive());

    CommandRouter::Response r;
        b.run("SET SP2=0.40", r);
    TEST_ASSERT_TRUE_MESSAGE(has(r, "ERR"), "setpoint rewritten under a diver");
    b.run("SET AP_PASS=otherpass", r);
    TEST_ASSERT_TRUE(has(r, "ERR"));
    // the refused write must not have landed
    TEST_ASSERT_EQUAL_STRING("diveonly", b.reg.getString(b.reg.find("AP_PASS")));

    // schedule tuning stays writable mid-dive (normal diving, not gas control)
    b.run("SET GF_HIGH=90", r);
    TEST_ASSERT_TRUE(has(r, "OK"));
    TEST_ASSERT_EQUAL(90, b.reg.getInt(b.reg.find("GF_HIGH")));

    b.run("DIVE START", r);
    TEST_ASSERT_TRUE(has(r, "ERR"));  // no double-start
}

void test_dive_start_drops_the_radio_and_end_restores_it() {
    Bench b;
    CommandRouter::Response r;
        b.run("DIVE START", r);
    TEST_ASSERT_TRUE(r.actions & CommandRouter::ACT_DIVE_START);
    TEST_ASSERT_TRUE(r.actions & CommandRouter::ACT_WIFI);
    TEST_ASSERT_FALSE(b.router.wifiEnabled());
    TEST_ASSERT_TRUE(b.router.wifiAutoOff());

    b.run("WIFI ON", r);
    TEST_ASSERT_TRUE_MESSAGE(has(r, "ERR"), "radio reopened during a dive");
    TEST_ASSERT_FALSE(b.router.wifiEnabled());

    b.run("DIVE END", r);
    TEST_ASSERT_TRUE(r.actions & CommandRouter::ACT_DIVE_END);
    TEST_ASSERT_TRUE(r.actions & CommandRouter::ACT_WIFI);
    TEST_ASSERT_TRUE(b.router.wifiEnabled());

    // If the diver had WiFi off before the dive, dive end must NOT turn it on.
    b.run("WIFI OFF", r);
    TEST_ASSERT_TRUE(has(r, "OK"));
    b.run("DIVE START", r);
    TEST_ASSERT_FALSE(b.router.wifiAutoOff());  // not our doing
    b.run("DIVE END", r);
    TEST_ASSERT_FALSE(b.router.wifiEnabled());
    TEST_ASSERT_FALSE(r.actions & CommandRouter::ACT_WIFI);

    b.run("DIVE END", r);
    TEST_ASSERT_TRUE(has(r, "ERR"));  // already ended
}

void test_diver_outranks_the_machine_cellproof_override() {
    Bench b;
    CommandRouter::Response r;
        b.run("CELLPROOF OVERRIDE", r);
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
    b.run("CELLPROOF OVERRIDE", r);
    TEST_ASSERT_TRUE_MESSAGE(has(r, "CLAMP LIFTED"),
                             "diver could not lift the clamp (diver outranks machine)");
    b.ticks(0.60f, 12.0f, 0.3f);
    TEST_ASSERT_EQUAL_UINT8(cellproof::State::PASS, b.loop.cellproof().state);
    TEST_ASSERT_FALSE(b.loop.cellproof().clampActive);

    // A second override has nothing to lift.
    b.run("CELLPROOF OVERRIDE", r);
    TEST_ASSERT_TRUE(has(r, "ERR"));

    b.run("CHECK", r);  // pre-dive checks only on the surface
    TEST_ASSERT_TRUE(has(r, "ERR"));
}

void test_safe_mode_lockout_refuses_everything_but_ack() {
    // SAFE mode = the control chain is not running. A handset that connects and
    // finds a live, chatty link must still not be able to author anything: each
    // of these gets the truth, not silence and not a fake OK.
    Bench b;
    b.ticks(0.5f, 0.0f, 3.0f);
    const float sp2Before = b.reg.getFloat(b.reg.find("SP2"));
    const bool wifiBefore = b.router.wifiEnabled();

    b.router.setSafeLockout(true);
    const char* const forbidden[] = {
        "AUTO", "MANUAL", "DIVE START", "SET SP2=0.55", "CAL AIR", "WIFI ON",
        "WIFI OFF", "CHECK", "GET *", "GET SP2", "LOG EXPORT 1", "CELLPROOF OVERRIDE",
    };
    for (const char* line : forbidden) {
        CommandRouter::Response r;
        b.run(line, r);
        TEST_ASSERT_TRUE_MESSAGE(has(r, "ERR safe mode"), line);
    }

    // refused means refused: no value moved, no radio changed
    TEST_ASSERT_EQUAL_FLOAT_MESSAGE(sp2Before, b.reg.getFloat(b.reg.find("SP2")),
                                    "SET landed despite the SAFE lockout");
    TEST_ASSERT_EQUAL_UINT8(wifiBefore ? 1 : 0, b.router.wifiEnabled() ? 1 : 0);
    TEST_ASSERT_FALSE(b.loop.diveActive());

    // ACK is the diver's way out, and it has to reach the transport, not just
    // look like a reply.
    CommandRouter::Response r;
        b.run("ACK", r);
    TEST_ASSERT_TRUE_MESSAGE(has(r, "OK"), "ACK refused in SAFE mode");
    TEST_ASSERT_TRUE_MESSAGE((r.actions & CommandRouter::ACT_SAFE_ACK) != 0,
                             "SAFE ACK never reached the transport hook");

    // Parse errors still report the real problem: a broken line is a broken
    // line before it is a SAFE refusal, or the bench chases the wrong cause.
    b.run("FROBNICATE", r);
    TEST_ASSERT_TRUE(has(r, "ERR unknown command"));

    // Out of SAFE, ACK is the ordinary alarm acknowledge again: no restart bit,
    // because the chain is running and there is nothing to restart.
    b.router.setSafeLockout(false);
    b.run("ACK", r);
    TEST_ASSERT_TRUE(has(r, "OK"));
    TEST_ASSERT_EQUAL_UINT32(0u, r.actions & CommandRouter::ACT_SAFE_ACK);
}

void test_schema_lists_every_key_and_fits_the_buffer() {
    // The schema is rendered once at boot (primeSchema) because it is the one
    // reply whose formatting uses %g, which has no business running on a
    // small-stack task. Two things can go wrong and both are loud here: the
    // listing outgrows its buffer (then GET SCHEMA refuses rather than serving
    // a prefix as if it were the whole table), or the reply buffer is smaller
    // than the listing (then a handset gets ERR and cannot configure itself).
    // A new setting key trips this test, which is the point.
    const size_t len = comms::primeSchema();
    size_t keys = 0;
    settings::Registry::definitions(keys);

    TEST_ASSERT_TRUE_MESSAGE(len > 0, "schema listing did not fit MAX_SCHEMA");
    TEST_ASSERT_TRUE_MESSAGE(len < CommandRouter::MAX_SCHEMA, "schema at MAX_SCHEMA limit");

    Bench b;
    CommandRouter::Response r;
    b.run("GET SCHEMA", r);
    TEST_ASSERT_EQUAL_HEX32_MESSAGE((uint32_t)len, (uint32_t)r.len,
                                      "schema reply is a prefix, not the whole table");
    TEST_ASSERT_EQUAL_HEX32_MESSAGE((uint32_t)keys, (uint32_t)countLines(r.text, r.len),
                                      "schema line count != key count");

    // and every key really is named in it
    size_t named = 0;
    for (const settings::Def* d = settings::Registry::definitions(keys), *p2 = d; p2 < d + keys; ++p2)
        if (std::strstr(r.text, p2->key)) ++named;
    TEST_ASSERT_EQUAL_HEX32_MESSAGE((uint32_t)keys, (uint32_t)named,
                                      "a key is missing from GET SCHEMA");
}

void test_reused_reply_buffer_never_leaks_the_previous_reply() {
    // The router no longer builds a fresh zeroed Response per command (that
    // 1 KB-per-call habit is what overflowed the uart2 task's stack), so one
    // buffer is reused and whatever the last command wrote is still in it. A
    // transport hands r.text to a sink that treats it as a C string, so a reply
    // that is not terminated puts the PREVIOUS reply's tail on the wire. That
    // happened for real: an ACK answered "OK\n unknown command\n" in the first
    // build after the change, and only a long reply before a short one exposes
    // it. Hence: long reply, short reply, every time, and the invariant holds.
    Bench b;
    CommandRouter::Response r;
    const char* seq[] = {
        "GET *",        // the longest reply the router makes
        "ACK",          // 3 bytes: this is the one that used to inherit a tail
        "GET SCHEMA",
        "OK?" ,
        "FROBNICATE",
        "GET T",
        "SET SP2=0.55",
        "GET SP2",
        "MANUAL",
        "GET D",
        "PULSE",
        "LOG LIST",
        "CELLPROOF OVERRIDE",
        "GET HIST",
    };
    for (const char* line : seq) {
        b.run(line, r);
        const size_t sl = std::strlen(r.text);
        TEST_ASSERT_EQUAL_HEX32_MESSAGE((uint32_t)r.len, (uint32_t)sl, line);  // no stale tail
        TEST_ASSERT_TRUE_MESSAGE(sl > 0 || r.len == 0, line);     // silent = empty
        if (r.len) {
            // Multi-line replies are legal (GET *, GET SCHEMA, GET HIST each
            // emit several lines), so the rule is not "one line": it is that the
            // reply is one terminated string — no NUL inside it (a sink writing
            // a C string would truncate silently), and it ends on a newline.
            // strlen(text)==len above is what catches a leaked tail.
            TEST_ASSERT_NULL_MESSAGE(std::memchr(r.text, 0, r.len), line);
            TEST_ASSERT_EQUAL_MESSAGE('\n', r.text[r.len - 1], line);
            TEST_ASSERT_TRUE_MESSAGE(countLines(r.text, r.len) >= 1, line);
        }
    }
}

void test_ack_always_honoured() {
    Bench b;
    CommandRouter::Response r;
        b.run("ACK", r);
    TEST_ASSERT_TRUE(has(r, "OK"));
    TEST_ASSERT_EQUAL_STRING("OK\n", r.text);  // no argument, no conditions
    TEST_ASSERT_EQUAL(0u, r.actions);          // and no hidden side effects
}

void test_testcells_runs_on_the_bench_and_reports_masks() {
    Bench b;
    b.ticks(0.50f, 0.0f, 3.0f);
    CommandRouter::Response r;
        b.run("TEST CELLS", r);
    TEST_ASSERT_TRUE(has(r, "OK"));
    TEST_ASSERT_TRUE(b.loop.cellTestRequested());
    b.run("TEST CELLS", r);
    TEST_ASSERT_TRUE(has(r, "ERR"));  // one at a time

    Plant plant{b.loop};
    for (int i = 0; i < 250 && b.loop.cellTestRequested(); i++) plant.tick(b.router, b.now);
    TEST_ASSERT_FALSE_MESSAGE(b.loop.cellTestRequested(), "standalone test never finished");
    TEST_ASSERT_EQUAL_UINT8(cal::TestState::DONE, b.loop.pulseTestState());
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(0b111, b.loop.cellTestPassMask(),
                                    "responsive cells reported as failed");

    b.run("GET CELLS", r);
    TEST_ASSERT_TRUE(has(r, "CELLS,4,"));   // DONE
    TEST_ASSERT_TRUE(has(r, "007,"));       // pass mask, all three, octal
}

void test_deco_and_status_lines_come_from_the_chain() {
    Bench b;
    b.ticks(0.55f, 0.0f, 5.0f);
    CommandRouter::Response r;
        b.run("DECO", r);
    // D,<ndl>,<total>,<ceiling>,[stops...],<cns> — at the surface: no stops,
    // zero ceiling, and the total is the literal 0.0, not a stop list.
    TEST_ASSERT_TRUE(r.text[0] == 'D' && r.text[1] == ',');
    TEST_ASSERT_NOT_NULL(std::strchr(r.text, '\n'));
    TEST_ASSERT_NULL(std::strchr(r.text, '|'));  // no stop entries
    b.run("GET D", r);
    TEST_ASSERT_TRUE(r.text[0] == 'D' && r.text[1] == ',');

    b.run("GET T", r);
    TEST_ASSERT_TRUE(r.text[0] == 'T' && r.text[1] == ',');
    b.run("GET PRE", r);
    TEST_ASSERT_TRUE(has(r, "PRE,"));
    b.run("GET FAULT", r);
    TEST_ASSERT_TRUE(has(r, "FAULT,0,0"));  // bench, no faults

    // a real key wins over a display key that would otherwise shadow it
    b.run("GET D1", r);
    TEST_ASSERT_TRUE(has(r, "D1=30.000"));
}

void test_set_actually_moves_the_live_chain() {
    // The full handset path: SET -> registry -> ACT_SETTINGS -> applier ->
    // live loop. Pins that the firmware's single applier actually changes
    // behaviour, not just stored text.
    Bench b;
    CommandRouter::Response r;
        b.run("SET ALLO_LOW=0.30", r);
    TEST_ASSERT_TRUE(r.actions & CommandRouter::ACT_SETTINGS);
    TEST_ASSERT_EQUAL_UINT8(comms::CLAMP_NONE, comms::applySettings(b.reg, b.cfg));
    b.loop.setConfig(b.cfg);

    b.ticks(0.25f, 0.0f, 5.0f);  // 0.25 bar: above the old 0.18 floor, below the new
    TEST_ASSERT_TRUE_MESSAGE(b.loop.alarmBits() & (1u << alarms::BIT_LOW_PPO2),
                             "SET ALLO_LOW never reached the live alarm engine");
}

// The clock is a command, not a settings key: a settings key would put the
// epoch in NVS, and a board that woke after a battery pull would then date every
// dive with a number from a previous life and never mention it. The router
// range-checks and hands the write to the transport, which owns the time.
void test_time_set_is_range_checked_and_delegated() {
    Bench b;
    wallclock::State clk;
    b.router.setClock(&clk);
    CommandRouter::Response r;

    // No clock: the answer is 0, not a 1970 date the caller might believe.
    b.run("GET TIME", r);
    TEST_ASSERT_TRUE(has(r, "TIME,0,0,0"));

    b.run("TIME SET 1795509660", r);              // 2026-11-24T08:41:00Z
    TEST_ASSERT_TRUE(has(r, "OK TIME"));
    TEST_ASSERT_TRUE(r.actions & CommandRouter::ACT_CLOCK_SET);
    TEST_ASSERT_EQUAL_UINT32(1795509660u, b.router.clockEpochPending());

    // A plausible-looking wrong epoch is worse than no epoch: it puts the dive
    // in 1970 or 2200 and every reader of the log believes it.
    b.run("TIME SET 7", r);
    TEST_ASSERT_TRUE(has(r, "ERR epoch out of range"));
    TEST_ASSERT_FALSE(r.actions & CommandRouter::ACT_CLOCK_SET);
    b.run("TIME SET 99999999999999", r);          // saturates: still refused
    TEST_ASSERT_TRUE(has(r, "ERR epoch out of range"));

    // What the transport does with the action, then what the client sees.
    wallclock::set(clk, b.router.clockEpochPending(), b.now);
    b.now += 5000;
    b.run("GET TIME", r);
    TEST_ASSERT_TRUE(has(r, "TIME,1795509665,1,5"));

    // Mid-dive is allowed on purpose: the dive header was written at DIVE START,
    // so this cannot rewrite the dive that is running, and a surface PC that has
    // only now noticed the clock is unset should be able to fix it before the
    // NEXT dive rather than be refused.
    b.run("DIVE START");
    b.run("TIME SET 1795509800", r);
    TEST_ASSERT_TRUE(has(r, "OK TIME"));
    b.run("DIVE END");

    // SAFE mode: the chain is not running, so no client may author anything —
    // including the clock the dive log would be stamped with.
    b.router.setSafeLockout(true);
    b.run("TIME SET 1795509900", r);
    TEST_ASSERT_TRUE(has(r, "ERR safe mode"));
    TEST_ASSERT_FALSE(r.actions & CommandRouter::ACT_CLOCK_SET);
    b.run("GET TIME", r);
    TEST_ASSERT_TRUE(has(r, "ERR safe mode"));
}

void test_log_and_pulse_and_calibration_commands_delegations() {
    Bench b;
    CommandRouter::Response r;
        b.run("LOG LIST", r);
    TEST_ASSERT_TRUE(r.actions & CommandRouter::ACT_LOG_LIST);
    b.run("LOG EXPORT 3", r);
    TEST_ASSERT_TRUE(r.actions & CommandRouter::ACT_LOG_EXPORT);
    TEST_ASSERT_EQUAL_UINT16(3, b.router.logExportIndex());

    // PULSE is a bench command, not a bypass: it asks the injector, which
    // still owns wall/cutoff/duty/fuse. Reply is a plain OK.
    b.run("PULSE", r);
    TEST_ASSERT_TRUE(has(r, "OK"));

    // two-step CAL O2 handshake, enforced in one place for every client
    b.run("CAL O2 CONFIRM", r);
    TEST_ASSERT_TRUE(has(r, "ERR"));  // confirm without arm: refused
    b.run("CAL O2", r);
    TEST_ASSERT_TRUE(r.actions & CommandRouter::ACT_CAL_O2_ARM);
    b.run("CAL O2 CONFIRM", r);
    TEST_ASSERT_TRUE(r.actions & CommandRouter::ACT_CAL_O2_CONFIRM);
    b.run("CAL O2", r);
    TEST_ASSERT_TRUE(r.actions & CommandRouter::ACT_CAL_O2_ARM);
    b.run("CAL O2", r);  // bare CAL O2 while armed = back out
    TEST_ASSERT_TRUE(r.actions & CommandRouter::ACT_CAL_O2_CANCEL);

    b.run("CAL AIR", r);
    TEST_ASSERT_TRUE(r.actions & CommandRouter::ACT_CAL_AIR);
    b.run("CAL DEPTH", r);
    TEST_ASSERT_TRUE(r.actions & CommandRouter::ACT_CAL_DEPTH);
    TEST_ASSERT_TRUE(b.router.takeDepthCalRequest());
    TEST_ASSERT_FALSE(b.router.takeDepthCalRequest());  // one-shot

    // calibration never runs under a diver
    b.run("DIVE START");
    b.run("CAL AIR", r);
    TEST_ASSERT_TRUE(has(r, "ERR"));
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_auto_lockout_holds_over_the_router);
    RUN_TEST(test_get_set_round_trip_on_the_shared_registry);
    RUN_TEST(test_dive_locked_keys_refused_mid_dive);
    RUN_TEST(test_dive_start_drops_the_radio_and_end_restores_it);
    RUN_TEST(test_diver_outranks_the_machine_cellproof_override);
    RUN_TEST(test_schema_lists_every_key_and_fits_the_buffer);
    RUN_TEST(test_reused_reply_buffer_never_leaks_the_previous_reply);
    RUN_TEST(test_safe_mode_lockout_refuses_everything_but_ack);
    RUN_TEST(test_ack_always_honoured);
    RUN_TEST(test_testcells_runs_on_the_bench_and_reports_masks);
    RUN_TEST(test_deco_and_status_lines_come_from_the_chain);
    RUN_TEST(test_set_actually_moves_the_live_chain);
    RUN_TEST(test_log_and_pulse_and_calibration_commands_delegations);
    RUN_TEST(test_time_set_is_range_checked_and_delegated);
    return UNITY_END();
}
