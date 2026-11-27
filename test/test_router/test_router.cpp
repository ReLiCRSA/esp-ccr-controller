// ============================================================================
// test_router — the command surface every client shares (handset UART2 and
// the web WebSocket). What matters here is not that commands work but that
// the refusals work: the pre-dive AUTO lockout, the dive-locked settings,
// the radio that must stay down during a dive, and the fact that no client,
// handset or browser, has a second path around any of it.
// ============================================================================
#include <unity.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

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
    // The published deco slot the router answers DECO / GET D from — on the
    // board it is the bridge's (Session::begin wires it); the Bench owns it
    // directly, exactly as main.cpp owns the publisher (deco/view.h).
    deco::ViewSlot viewSlot;
    CommandRouter router;

    // The reply buffer is storage the CALLER owns, as it is on the target:
    // CommandRouter::Response is not copyable (see cmd_router.h for the crash
    // that made it that way), so no test — and no transport — can move one
    // through a stack frame by value. `resp` is the bench's own buffer; a test
    // that must hold two replies at once passes its own.
    CommandRouter::Response resp;
    Bench() : reg(), router(loop, reg, now) {
        reg.resetToDefaults();
        router.setDecoView(&viewSlot);
        publishDeco();  // the board never answers from an unstarted chain:
                        // control has run a cycle before any client speaks
    }
    // The ControlTask publication, mirrored: cheap scalars + plan at the
    // chain's own depth (main.cpp publishes per cycle / at edges / at grid).
    void publishDeco() {
        deco::View pic{};
        deco::viewFull(pic, loop.deco(),
                       loop.deco().depthToBar(loop.depthSmoothedM()),
                       loop.haveVote(), loop.shownPpo2());
        deco::publishView(viewSlot, pic);
    }
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
            publishDeco();
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

    // The depth ruler. One density drives the `T,` metres, the setpoint tiers
    // those metres select, the cellproof arm depth and the deco engine's
    // ambient pressures, so a mid-dive write moves the diver's depth scale up to
    // ~2.5% (0.990..1.100) at a pressure that has not changed. Refusing it is the
    // same decision the setpoints already get — and the VALUE must not move
    // either, which is the part a refusal that only prints ERR would fake.
    b.run("SET WATER_DENSITY=1.000", r);
    TEST_ASSERT_TRUE_MESSAGE(has(r, "ERR"),
                             "the depth ruler was rewritten under a diver");
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.025f, b.reg.getFloat(b.reg.find("WATER_DENSITY")));

    // schedule tuning stays writable mid-dive (normal diving, not gas control)
    b.run("SET GF_HIGH=90", r);
    TEST_ASSERT_TRUE(has(r, "OK"));
    TEST_ASSERT_EQUAL(90, b.reg.getInt(b.reg.find("GF_HIGH")));

    // Close, restore the ruler, and show the refusal was about the dive and not
    // about the key: the identical write is accepted at the surface.
    TEST_ASSERT_TRUE(has(b.run("DIVE END"), "OK"));
    // The dive GATE flips at DIVE END, but the chain's dive-state work lands on
    // the next control tick (post* contract, loop.h) — and the bench clock only
    // moves when ticks() runs. So the gate must be off now, and it must still be
    // off after a tick, which is what makes the surface write below a real
    // surface write rather than one graded mid-dive.
    b.ticks(0.21f, 0.0f, 2.0f);
    TEST_ASSERT_FALSE_MESSAGE(b.loop.diveActive(),
                              "the dive gate never closed on the control chain");
    // Reusing `r` here is deliberate, not sloppiness: `exec()` resets len and
    // actions AND writes text[0] = '\\0', and every reply path terminates at len,
    // so a reused Response cannot hand a needle test the previous reply — that
    // invariant is pinned by test_reused_reply_buffer_never_leaks_the_previous_reply.
    // (An earlier draft of this case blamed a failing assertion on stale bytes in a
    // reused buffer. It was wrong, and the real cause is worth more: the case's own
    // DIVE END had invalidated the trailing DIVE START assertion, which was no
    // longer testing a double start at all. A case can make its own later
    // assertions meaningless; "it passed before I edited it" proves nothing.)
    b.run("SET WATER_DENSITY=1.000", r);
    TEST_ASSERT_TRUE_MESSAGE(has(r, "OK"),
                             "the diver cannot switch sea to fresh at the surface");
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.000f, b.reg.getFloat(b.reg.find("WATER_DENSITY")));
}

void test_re_dive_after_a_closed_dive_is_a_fresh_dive() {
    // This behaviour used to be covered only as the last needle of the case above,
    // inside a state that case no longer leaves in place. Stated on its own: a
    // second DIVE START while the gate is up is refused WITH THE REASON, and once
    // the dive has closed, starting again is a NEW dive rather than a refusal —
    // which is what a dive log's two headers depend on.
    Bench b;
    CommandRouter::Response r;
    TEST_ASSERT_TRUE(has(b.run("DIVE START"), "OK"));

    b.run("DIVE START", r);
    TEST_ASSERT_TRUE_MESSAGE(has(r, "ERR"), "a second DIVE START opened a dive");
    TEST_ASSERT_TRUE_MESSAGE(has(r, "dive active"),
                             "the refusal did not say why: a diver told only ERR "
                             "cannot tell an open dive from a broken command");

    TEST_ASSERT_TRUE(has(b.run("DIVE END"), "OK"));
    TEST_ASSERT_FALSE(b.loop.diveActive());     // gate flips at once (post* contract)
    b.ticks(0.21f, 0.0f, 1.0f);                 // let the chain apply the end body
    TEST_ASSERT_FALSE(b.loop.diveActive());

    b.run("DIVE START", r);
    TEST_ASSERT_TRUE_MESSAGE(has(r, "OK"), "the board would not open a second dive");
    TEST_ASSERT_TRUE(b.loop.diveActive());
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
        // Even a read-only query is refused: SAFE means the control chain is not
        // deciding, so "which chip owns depth, and is it being used?" has no
        // truthful answer to give, and a client that reads state from a machine
        // that is not running is a client that will act on it.
        "GET DS",
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

void test_deco_answers_err_when_nothing_published_the_picture() {
    // The SAFE-boot shape: no control cycle has published a deco generation,
    // so DECO / GET D must refuse rather than print a plan nobody computed.
    // (On the board Session::begin wires the slot; an unwired router is also
    // the wiring-bug case, and it must say so out loud.)
    Config cfg = benchCfg();
    Loop loop(cfg);
    settings::Registry reg;
    uint32_t now = 0;
    CommandRouter router(loop, reg, now);
    reg.resetToDefaults();
    CommandRouter::Response r;
    router.setNowMs(now);
    router.exec("DECO", r);
    TEST_ASSERT_TRUE_MESSAGE(std::strncmp(r.text, "ERR", 3) == 0,
                             "DECO answered a plan the control task never published");
    router.exec("GET D", r);
    TEST_ASSERT_TRUE_MESSAGE(std::strncmp(r.text, "ERR", 3) == 0,
                             "GET D answered a plan the control task never published");
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

// The applier is the only place an unsafe *combination* can be caught: each key
// passes its own range, the router answers OK SAFETY for both SETs, and the
// inversion exists only between them. Graded on the router path because that is
// the path a handset and the browser share — and the registry is asserted to
// still hold what the diver typed, which is why the clamp has to be printed
// rather than assumed: `GET GF_LOW` answers 90 while the chain runs 50.
void test_cross_key_sets_are_clamped_before_they_reach_the_chain() {
    Bench b;
    CommandRouter::Response r;

    b.run("SET GF_LOW=90", r);   // legal per key: GF_LOW tops out at 95
    TEST_ASSERT_TRUE(r.actions & CommandRouter::ACT_SETTINGS);
    b.run("SET GF_HIGH=50", r);  // legal per key: GF_HIGH starts at 50
    TEST_ASSERT_TRUE(r.actions & CommandRouter::ACT_SETTINGS);

    const uint8_t c = comms::applySettings(b.reg, b.cfg);
    TEST_ASSERT_TRUE_MESSAGE(c & comms::CLAMP_GF_ORDER,
                             "an inverted gradient schedule reached the control config unclamped");
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 50.0f, b.cfg.deco.gfLow);   // the stricter edge wins
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 50.0f, b.cfg.deco.gfHigh);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 90.0f, b.reg.getFloat(b.reg.find("GF_LOW")));

    b.run("SET D1=40", r);
    b.run("SET D2=15", r);
    TEST_ASSERT_TRUE_MESSAGE(comms::applySettings(b.reg, b.cfg) & comms::CLAMP_BAND_ORDER,
                             "crossed depth bands reached the control config unclamped");
    TEST_ASSERT_TRUE(b.cfg.sp.deep1M <= b.cfg.sp.deep2M);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 40.0f, b.cfg.sp.deep2M);    // both depths survive
}

// `GET T` answers from cmd_router's own copy of the alarm-word -> FLAGS mapping;
// UartLink::emitT has the other copy. Two mappings of one word is drift waiting
// to happen, so the same case is graded on both transports and must give the
// same byte: CNS_HIGH as bit 7, and no fault bit beside it.
void test_get_t_projects_cns_exactly_as_the_pushed_stream_does() {
    Bench b;
    // Raise the band's target gas to the ppO2 being fed, so the only alarm in the
    // word is the oxygen budget. A HIGH_PPO2 riding along would not prove the bit
    // is separate — it would only prove something got set.
    for (int i = 0; i < 4; i++) b.cfg.sp.sp[i] = 1.30f;
    b.cfg.alarms.cnsAlarmPct = 0.01f;   // seconds of exposure, not a dive's worth
    b.loop.setConfig(b.cfg);
    b.loop.startDive(b.now);            // CNS advances only during a dive
    b.ticks(1.30f, 5.0f, 3.0f);
    TEST_ASSERT_TRUE_MESSAGE(b.loop.alarmBits() & (1u << alarms::BIT_CNS_HIGH),
                             "the fixture never raised CNS_HIGH, so nothing is being graded");

    CommandRouter::Response r;
    b.run("GET T", r);
    const char* p = std::strstr(r.text, "FLAGS0x");
    TEST_ASSERT_NOT_NULL_MESSAGE(p, "GET T answered without a FLAGS field");
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(
        0x80u, std::strtoul(p + 7, nullptr, 16),
        "GET T projects CNS_HIGH differently from the T, stream");
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

void test_log_and_calibration_commands_delegations() {
    Bench b;
    CommandRouter::Response r;
        b.run("LOG LIST", r);
    TEST_ASSERT_TRUE(r.actions & CommandRouter::ACT_LOG_LIST);
    b.run("LOG EXPORT 3", r);
    TEST_ASSERT_TRUE(r.actions & CommandRouter::ACT_LOG_EXPORT);
    TEST_ASSERT_EQUAL_UINT16(3, b.router.logExportIndex());

    // `PULSE` retired 2026-11-26 (HANDSET_PROTOCOL.md §8): no handset-originated
    // manual injection. Pinned at the router so a regression that quietly
    // re-arms any pulse path — parse table, Kind, or a new case — is a red
    // test, not a quiet feature. It must be the parse-level refusal (unknown
    // command), not a runtime refusal that leaves the verb wired: and no
    // action bit may ride along, so the transport provably did nothing.
    b.run("PULSE", r);
    TEST_ASSERT_TRUE(has(r, "ERR unknown command"));
    TEST_ASSERT_EQUAL_UINT32(CommandRouter::ACT_NONE, r.actions);

    // CAL AIR is the one calibration with a consumer on the board, so it is
    // accepted (and its verdict reaches the wire: see test_uartlink).
    b.run("CAL AIR", r);
    TEST_ASSERT_TRUE(r.actions & CommandRouter::ACT_CAL_AIR);

    // calibration never runs under a diver
    b.run("DIVE START");
    b.run("CAL AIR", r);
    TEST_ASSERT_TRUE(has(r, "ERR"));
}

// Refusals are a safety surface. CAL O2 armed a two-step handshake whose
// consumer (cal::O2Sequencer) was never instantiated on the board, and CAL
// DEPTH raised a request flag nothing reads; both answered OK, which on a
// rebreather reads as "calibrated". They are refused now, and refused the same
// way for every client, because that is what the shared session is for.
void test_unimplemented_calibrations_are_refused_not_acked() {
    Bench b;
    CommandRouter::Response r;

    // The verbs are case-sensitive on the wire (see uart_proto); extra spaces
    // must not smuggle a different answer. Explicit array, not a braced
    // initializer_list: this suite compiles for the target too.
    static const char* const kRefused[] = {
        "CAL DEPTH", "CAL O2", "CAL O2 CONFIRM", "CAL  DEPTH", "CAL O2   CONFIRM"};
    for (const char* const cmd : kRefused) {
        b.run(cmd, r);
        TEST_ASSERT_TRUE_MESSAGE(has(r, "ERR not implemented"), cmd);
        // and nothing at all was asked of the machine: no action bit, so no
        // transport hook, so no flag that a future consumer might read
        TEST_ASSERT_EQUAL_UINT32_MESSAGE(CommandRouter::ACT_NONE, r.actions, cmd);
    }

    // The refusal outranks the mid-dive gate: "cal locked in dive" would imply
    // it works out of the water, which is the same lie in a different order.
    b.run("DIVE START");
    b.run("CAL DEPTH", r);
    TEST_ASSERT_TRUE(has(r, "ERR not implemented"));
    b.run("CAL O2", r);
    TEST_ASSERT_TRUE(has(r, "ERR not implemented"));

    // The grammar is unchanged, so the day a consumer lands no client has to
    // change: an unknown target is still a syntax error, not this refusal.
    b.run("CAL BALDERDASH", r);
    TEST_ASSERT_TRUE(has(r, "ERR unknown CAL target"));

    // CAL AIR is unaffected by any of this (it does have a consumer).
    b.run("DIVE END");
    b.run("CAL AIR", r);
    TEST_ASSERT_TRUE(has(r, "OK CAL AIR"));
    TEST_ASSERT_TRUE(r.actions & CommandRouter::ACT_CAL_AIR);
}

// GET CALAIR is how any client learns what the air calibration actually did:
// `OK CAL AIR` only ever meant "requested", and until 2026-11-24 the verdict
// (numbers, or the reason it refused) existed only on the USB console.
void test_get_calair_reports_the_verdict_not_the_request() {
    Bench b;
    bridge::CalAir cal;
    b.router.setCalAir(&cal);

    // Nothing asked yet: a stated nothing, not an error and not a result.
    TEST_ASSERT_TRUE(std::strstr(b.run("GET CALAIR").text, "CALAIR,0,0,0,0,0,0,0,0"));

    // Refused for warm-up: the seconds left are the useful part.
    cal.code = bridge::CALAIR_WARMUP;
    cal.atMs = 1000; cal.warmupLeftS = 117; cal.samples = 0;
    b.now = 60000;   // 60 s later: the age is part of the answer
    TEST_ASSERT_TRUE(std::strstr(b.run("GET CALAIR").text, "CALAIR,3,59,0,0,0,0,0,117"));

    // Committed: the raw averages and the worst spread, the numbers the console
    // used to be the only place to read them.
    cal.code = bridge::CALAIR_OK; cal.atMs = b.now; cal.warmupLeftS = 0;
    cal.samples = 12; cal.spreadWorst = 23.0f;
    cal.rawAvg[0] = 1502.0f; cal.rawAvg[1] = 1488.0f; cal.rawAvg[2] = 0.0f;
    TEST_ASSERT_TRUE(std::strstr(b.run("GET CALAIR").text, "CALAIR,2,0,12,23,1502,1488,0,0"));
    TEST_ASSERT_TRUE(std::strstr(b.run("GET CALAIR").text, "\n"));   // one line, terminated

    // In progress, so a client can tell "still sampling" from "refused".
    cal.code = bridge::CALAIR_RUNNING; cal.atMs = b.now; cal.samples = 7;
    TEST_ASSERT_TRUE(std::strstr(b.run("GET CALAIR").text, "CALAIR,1,0,7,23,1502,1488,0,0"));

    // A session with nothing wired up says so rather than inventing a result.
    b.router.setCalAir(nullptr);
    TEST_ASSERT_TRUE(std::strstr(b.run("GET CALAIR").text, "CALAIR,0,0,0,0,0,0,0,0"));

    // A display key, not a setting: GET CALAIR must never be answered out of
    // the registry, and CALAIR must not have become one.
    TEST_ASSERT_EQUAL(-1, b.reg.find("CALAIR"));
}


void test_a_published_max_length_value_lands_whole() {
    // `GET SCHEMA` publishes AP_PASS as 8..31 characters, and the page puts those
    // numbers in the input it renders. Until 2026-11-24 the command parser carried
    // 23 characters, so a diver typing the length the device advertised got `OK`
    // and a DIFFERENT passphrase — the ack-with-no-consumer shape again, one layer
    // further down. Reading the value back is the only witness, so this asserts
    // both the ack and the stored string, and asserts the reply line too because a
    // truncated GET answer would hide the same bug from every client.
    Bench b;
    CommandRouter::Response r;
    const char* pass = "correct horse battery staple XY";      // exactly 31
    TEST_ASSERT_EQUAL_INT_MESSAGE(31, static_cast<int>(std::strlen(pass)),
                                  "fixture length IS the published maximum");

    b.run((std::string("SET AP_PASS=") + pass).c_str(), r);
    TEST_ASSERT_TRUE_MESSAGE(has(r, "OK"), "a 31-character passphrase was refused");
    TEST_ASSERT_EQUAL_STRING_MESSAGE(pass, b.reg.getString(b.reg.find("AP_PASS")),
                                     "the ack was OK and the stored string is not "
                                     "what the client sent");

    b.run("GET AP_PASS", r);
    TEST_ASSERT_TRUE_MESSAGE(has(r, (std::string("AP_PASS=") + pass).c_str()),
                             "the read-back is shorter than what was written");

    // And the registry, not the transport, is what draws the line: 32 characters is
    // one past what a string key can hold, and that must be an honest refusal with
    // the registry's own words — never a shortened write that answers OK.
    b.run((std::string("SET AP_PASS=") + std::string(32, 'z')).c_str(), r);
    TEST_ASSERT_TRUE_MESSAGE(has(r, "ERR"), "32 characters were accepted");
    TEST_ASSERT_TRUE_MESSAGE(has(r, "value too long"),
                             "the refusal did not use the registry's own phrase");
    TEST_ASSERT_EQUAL_STRING_MESSAGE(pass, b.reg.getString(b.reg.find("AP_PASS")),
                                     "the refused write moved the value anyway");
}

// ---- GET DS: which chip owns depth, and whether anyone is using it ---------

void test_get_depth_source_names_the_chip_and_says_if_it_is_used() {
    // The claim a diver can check with his own eyes: the metre reading on `T,`
    // came from this named chip at this address, and it is being measured rather
    // than held. Until now the only place that fact was written down was the boot
    // announce, which says it once, to the console, before any client is listening.
    Bench b;
    b.ticks(0.5f, 3.0f, 60.0f);

    // First, with NOTHING wired: the command still answers. A protocol-visible
    // query that disappears under a build flag or a missing setter tells a client
    // less than one that says "no source" out loud — and `ERR unknown key` is the
    // answer a handset reads as "this controller is old", not as "no depth sensor".
    CommandRouter::Response r;
    b.run("GET DS", r);
    TEST_ASSERT_TRUE_MESSAGE(has(r, "DS,0,0,0,0,none"), r.text);
    TEST_ASSERT_FALSE_MESSAGE(has(r, "unknown key"),
                              "GET DS is not in the command table at all");
    TEST_ASSERT_EQUAL_INT_MESSAGE(1, static_cast<int>(countLines(r.text, r.len)),
                                  "GET DS answered more than one line");

    bridge::DepthSource ds;
    ds.locked = ds.used = ds.healthy = true;
    ds.lastGoodMs = b.now - 15'000u;
    std::snprintf(ds.identity, sizeof(ds.identity), "LPS28DFW @0x5D");
    b.router.setDepthSource(&ds);

    b.run("GET DS", r);
    TEST_ASSERT_EQUAL_STRING_MESSAGE("DS,1,1,1,15,LPS28DFW @0x5D\n", r.text,
                                     "the answer is not the published state");

    // THE field that matters: a source can hold the slot, be identified and be
    // healthy-looking while the control path has stopped taking depth from it
    // (stale bus, no sane sample). The metres on the wire are then HELD, and on
    // `T,` that is indistinguishable from a real reading. `locked` alone could
    // never say so, which is why used is a separate bit and not an alias.
    ds.used = false;
    b.run("GET DS", r);
    TEST_ASSERT_TRUE_MESSAGE(has(r, "DS,1,0,"), r.text);
    ds.healthy = false;
    b.run("GET DS", r);
    TEST_ASSERT_TRUE_MESSAGE(has(r, "DS,1,0,0,"), r.text);

    // No source: the name must be "none" even though the array still holds the
    // chip that held the slot before. A five-field snapshot copied without a lock
    // can be caught mid-update, and the pair that would mislead a reader is a name
    // beside a flag that claims no source — so the reply derives the name from
    // `locked` rather than printing whatever is in the array. That makes the
    // misleading pair unrepresentable on the wire.
    ds.locked = ds.used = ds.healthy = false;
    b.run("GET DS", r);
    TEST_ASSERT_EQUAL_STRING_MESSAGE("DS,0,0,0,0,none\n", r.text,
                                     "an unlocked slot named a chip it does not own");

    // Same rule the other way round, on a snapshot the publisher left inconsistent:
    // flags say no source, the array still says something else.
    std::snprintf(ds.identity, sizeof(ds.identity), "MS5837-30BA @0x76");
    ds.lastGoodMs = b.now - 9000u;
    b.run("GET DS", r);
    TEST_ASSERT_EQUAL_STRING_MESSAGE("DS,0,0,0,0,none\n", r.text,
                                     "a stale name printed beside locked=0");
    std::snprintf(ds.identity, sizeof(ds.identity), "none");

    // Age comes from the same clock the control chain uses, in seconds. A source
    // that has NEVER answered has lastGoodMs 0 — reporting that as "the whole
    // uptime" would look like a very old reading instead of no reading, so age is
    // 0 and `locked` carries the fact.
    ds.locked = ds.used = ds.healthy = true;   // used is still false from the step above
    ds.lastGoodMs = 0;
    b.run("GET DS", r);
    TEST_ASSERT_TRUE_MESSAGE(has(r, "DS,1,1,1,0,none"), r.text);
    ds.lastGoodMs = b.now - 250u;   // a quarter of a second: 0 s, not 1
    b.run("GET DS", r);
    TEST_ASSERT_TRUE_MESSAGE(has(r, "DS,1,1,1,0,none"), r.text);

    // A client that re-reads must see the current state, not a string rendered
    // once at boot: the boot announce is a one-shot, this answer is not.
    std::snprintf(ds.identity, sizeof(ds.identity), "MS5837-30BA @0x76");
    ds.lastGoodMs = b.now - 7000u;
    b.run("GET DS", r);
    TEST_ASSERT_EQUAL_STRING_MESSAGE("DS,1,1,1,7,MS5837-30BA @0x76\n", r.text,
                                     "GET DS is answering a stale snapshot");

    // And it is a query, not an act: nothing about the machine moved.
    TEST_ASSERT_FALSE(b.loop.diveActive());
    TEST_ASSERT_EQUAL_UINT32(0u, r.actions);
}

void test_a_depth_source_name_cannot_run_past_its_field() {
    // The identity string is not a promise. It lives in a char[24] the sensor
    // driver rewrites with snprintf every time it re-identifies after a bus
    // fault (lps28.cpp noteFail -> BUS_FAULT, depth_manager.cpp:8 re-begin,
    // lps28.cpp:45 snprintf), and the answer is printed with %s. An unbounded
    // copy — a strcpy, or a memcpy that trusts the source to be terminated —
    // reads past the array and prints whatever sits after it. The snapshot here
    // is padded with a sentinel to make that visible rather than merely unlikely.
    struct Padded {
        bridge::DepthSource ds;
        char sentinel[16];
    } pad;
    pad.sentinel[0] = '\0';
    std::memcpy(pad.sentinel, "SHOULD-NOT-PRINT", sizeof(pad.sentinel));
    std::memset(pad.ds.identity, 'A', sizeof(pad.ds.identity));   // no NUL at all
    pad.ds.locked = pad.ds.used = pad.ds.healthy = true;
    pad.ds.lastGoodMs = 1000u;   // b.now is 5000: age 4 s

    Bench b;
    b.now = 5000u;
    b.router.setDepthSource(&pad.ds);
    CommandRouter::Response r;
    b.run("GET DS", r);

    TEST_ASSERT_NULL_MESSAGE(std::strstr(r.text, "SHOULD-NOT-PRINT"),
                             (std::string("the identity field ran past its array: ") + r.text).c_str());
    TEST_ASSERT_EQUAL_INT_MESSAGE(1, static_cast<int>(countLines(r.text, r.len)),
                                  "a runaway identity field emitted more than one line");
    const char* comma = std::strrchr(r.text, ',');
    TEST_ASSERT_NOT_NULL(comma);
    const int field = static_cast<int>(std::strlen(comma + 1) - 1);   // less the newline
    // Exactly the array's own 24 bytes: an un-terminated array cannot make the
    // reply longer than the array, and a copy that stops early would make it
    // shorter, which is the other way this field goes wrong (a driver that
    // re-identifies mid-read hands out a prefix).
    TEST_ASSERT_EQUAL_INT_MESSAGE(static_cast<int>(bridge::DepthSource::ID_MAX), field,
                                  (std::string("the identity field is not the array, whole: ") + r.text).c_str());
    for (int i = 0; i < field; ++i)
        TEST_ASSERT_EQUAL_CHAR_MESSAGE('A', comma[1 + i],
                                       "the identity field holds a byte that is not in the array");
    TEST_ASSERT_TRUE_MESSAGE(has(r, "DS,1,1,1,4,"), r.text);
}

// ---- radio policy: depth/idle holds, GET WIFI, and who outranks whom -------

namespace {
// One policy step at a wall time, from the loopTask call site's point of view.
int wt(Bench& b, uint32_t atMs, bool depthUsed, float depthM, uint8_t stations) {
    b.now = atMs;
    return b.router.wifiTick(atMs, depthUsed, depthM, stations);
}
} // namespace

void test_the_wifi_policy_holds_on_depth_and_releases_shallow_and_straight() {
    // Defaults: off past 2.0 m, back below 1.0 m after 60 s straight, idle 10 min.
    Bench b;
    CommandRouter::Response r;

    // Prime: the first tick adopts reality and reports nothing — a controller
    // that booted with the AP up at the surface prints no fake transition.
    TEST_ASSERT_EQUAL_INT_MESSAGE(0, wt(b, 1000u, true, 0.0f, 0u),
                                  "the first tick invented a radio transition");
    TEST_ASSERT_TRUE(b.router.wifiRadioWanted());
    b.run("GET WIFI", r);
    TEST_ASSERT_EQUAL_STRING_MESSAGE("WIFI,1,0,600,0\n", r.text,
                                     "fresh boot, no station: the countdown should own the line");

    // The off threshold is strictly greater: 2.0 m exactly is not "past 2 m".
    TEST_ASSERT_EQUAL_INT(0, wt(b, 5000u, true, 2.0f, 0u));
    TEST_ASSERT_TRUE(b.router.wifiRadioWanted());

    // Past it: down, held by depth, and GET WIFI says who.
    TEST_ASSERT_EQUAL_INT(-1, wt(b, 6000u, true, 2.2f, 0u));
    TEST_ASSERT_FALSE(b.router.wifiRadioWanted());
    TEST_ASSERT_EQUAL_UINT8(CommandRouter::WH_DEPTH, b.router.wifiHold());
    b.run("GET WIFI", r);
    TEST_ASSERT_EQUAL_STRING("WIFI,0,3,0,0\n", r.text);

    // The hysteresis band is a band: 1.5 m is neither "past 2" nor "below 1",
    // and ten straight minutes of it changes nothing. A diver hovering a
    // safety stop must not flap the radio.
    for (uint32_t t = 7000u; t <= 607000u; t += 30000u) {
        TEST_ASSERT_EQUAL_INT_MESSAGE(0, wt(b, t, true, 1.5f, 0u),
                                      "the hysteresis band moved the radio");
        TEST_ASSERT_FALSE(b.router.wifiRadioWanted());
    }

    // Below 1 m but not for 60 straight seconds: still held — and the timer
    // restarts when the diver dips back into the band (59 s shallow, one dip,
    // then shallow again: the first 59 s must not count).
    TEST_ASSERT_EQUAL_INT(0, wt(b, 637000u, true, 0.5f, 0u));   // shallow starts
    TEST_ASSERT_EQUAL_INT(0, wt(b, 666000u, true, 0.5f, 0u));   // 29 s
    TEST_ASSERT_EQUAL_INT(0, wt(b, 696000u, true, 0.5f, 0u));   // 59 s: not yet
    TEST_ASSERT_EQUAL_INT(0, wt(b, 700000u, true, 1.5f, 0u));   // dip: restarts
    TEST_ASSERT_EQUAL_INT(0, wt(b, 759000u, true, 0.5f, 0u));   // timer restarts here
    TEST_ASSERT_EQUAL_INT(0, wt(b, 760000u, true, 0.5f, 0u));   // 1 s: far from it
    TEST_ASSERT_EQUAL_INT(0, wt(b, 818000u, true, 0.5f, 0u));   // 59 s from the restart
    TEST_ASSERT_FALSE(b.router.wifiRadioWanted());
    TEST_ASSERT_EQUAL_INT(1, wt(b, 819000u, true, 0.5f, 0u));   // 60 s straight: up
    TEST_ASSERT_TRUE(b.router.wifiRadioWanted());
    b.run("GET WIFI", r);
    TEST_ASSERT_EQUAL_STRING("WIFI,1,0,600,0\n", r.text);         // window restarted

    // STALE DEPTH FREEZES THE LATCH. This is GET DS's used=0 case wearing
    // a hard hat: a locked-but-unused sensor leaves the last depth HELD, and
    // driving a radio from a held number is policy inventing a measurement.
    // At depth, held: the source goes quiet for an hour at a fake-shallow
    // value; the latch must not budge, and the shallow timer must not even
    // start — the value is not believed, so it cannot be evidence.
    TEST_ASSERT_EQUAL_INT(-1, wt(b, 860000u, true, 5.0f, 0u));
    for (uint32_t t = 920000u; t <= 4400000u; t += 600000u)
        TEST_ASSERT_EQUAL_INT_MESSAGE(0, wt(b, t, false, 0.0f, 0u),
                                      "stale depth moved the radio policy");
    // And when the source comes back shallow, the 60 s starts THEN — the hour
    // of unbelieved silence is not 60 s of straight shallow water.
    TEST_ASSERT_EQUAL_INT(0, wt(b, 4400000u, true, 0.0f, 0u));
    TEST_ASSERT_EQUAL_INT(0, wt(b, 4459000u, true, 0.0f, 0u));
    TEST_ASSERT_FALSE(b.router.wifiRadioWanted());
    TEST_ASSERT_EQUAL_INT(1, wt(b, 4460000u, true, 0.0f, 0u));
    TEST_ASSERT_EQUAL_UINT8(CommandRouter::WH_NONE, b.router.wifiHold());
}

void test_the_idle_hold_counts_only_real_time_up_and_words_clear_it() {
    Bench b;
    CommandRouter::Response r;
    wt(b, 1000u, true, 0.0f, 0u);                 // prime at the surface

    // The countdown is protocol-visible: 540 s left at one minute in.
    b.now = 61000u;
    b.run("GET WIFI", r);
    TEST_ASSERT_EQUAL_STRING("WIFI,1,0,540,0\n", r.text);

    // Any association rewinds the window — the driver's station count, not a
    // WebSocket count: a phone sitting associated with no page open IS here.
    for (uint32_t t = 501000u; t <= 600000u; t += 10000u)
        TEST_ASSERT_EQUAL_INT(0, wt(b, t, true, 0.0f, 1u));       // station up
    TEST_ASSERT_EQUAL_INT(0, wt(b, 601000u, true, 0.0f, 0u));     // just left
    TEST_ASSERT_TRUE(b.router.wifiRadioWanted());                 // not 10 min of EMPTY
    // The idle rule's INPUT on the wire: the countdown alone cannot distinguish
    // "nobody is connected and the window is aging" from "someone IS connected
    // and the window keeps rewinding" — the stations field can, and a bench
    // that cannot tell those apart cannot tell a working idle rule from a stuck
    // one (found on the bench 2026-11-26, by the bench PC's own wlan0).
    wt(b, 602000u, true, 0.0f, 2u);
    b.run("GET WIFI", r);
    TEST_ASSERT_EQUAL_STRING("WIFI,1,0,600,2\n", r.text);

    // Ten straight minutes of nothing. (The stations=2 witness above ran a
    // tick at 602000 — a station rewinds the window, so the fire is 600 s from
    // THERE, and the arithmetic here matches that, not a rounded story about
    // the earlier rewind loop.)
    TEST_ASSERT_EQUAL_INT(0, wt(b, 1201000u, true, 0.0f, 0u));    // 599 s from the witness
    TEST_ASSERT_EQUAL_INT(-1, wt(b, 1202000u, true, 0.0f, 0u));   // exactly 10 min: down
    TEST_ASSERT_FALSE(b.router.wifiRadioWanted());
    TEST_ASSERT_EQUAL_UINT8(CommandRouter::WH_IDLE, b.router.wifiHold());
    b.run("GET WIFI", r);
    TEST_ASSERT_EQUAL_STRING("WIFI,0,4,0,0\n", r.text);

    // Words clear idle (it is the command's exact answer to "nobody is
    // here, so we sat down") and do NOT touch depth. The action bit rides
    // the EFFECTIVE change: idle was holding, so this is a real up.
    b.run("WIFI ON", r);
    TEST_ASSERT_TRUE(r.actions & CommandRouter::ACT_WIFI);
    TEST_ASSERT_TRUE(b.router.wifiRadioWanted());
    b.run("GET WIFI", r);
    TEST_ASSERT_EQUAL_STRING("WIFI,1,0,600,0\n", r.text);

    // The idle latch never releases itself: with nothing connected and no
    // command, an hour must not bring the radio back (a self-clearing idle
    // hold flaps the AP once a quarter-hour forever).
    // The command applied the radio immediately (the ACT above); the NEXT
    // tick is where the policy sees up: it reports +1 and starts that
    // radio's own window at 1790000 — a new fire belongs at 2390000, never
    // before, and it never "remembers" the pre-WIFI-On window. Between:
    TEST_ASSERT_EQUAL_INT(1, wt(b, 1790000u, true, 0.0f, 0u));    // up seen, window starts
    TEST_ASSERT_EQUAL_INT(0, wt(b, 2389000u, true, 0.0f, 0u));    // 599 s in
    TEST_ASSERT_EQUAL_INT(-1, wt(b, 2390000u, true, 0.0f, 0u));   // 600 s: idle again
    wt(b, 2391000u, true, 0.0f, 0u);
    TEST_ASSERT_EQUAL_INT(0, wt(b, 2691000u, true, 0.0f, 0u));    // 5 min held: no flap

    // WIFI_IDLE_MIN=0 means never, and it takes effect without a reboot:
    // a bench that sets 1 must see the fire one minute later.
    b.run("SET WIFI_IDLE_MIN=0", r);
    // The setting releases the hold it governs, on the next tick (loopTask
    // is at most 250 ms from this command on the real board — no command
    // path needs to know the policy exists):
    TEST_ASSERT_EQUAL_INT(1, wt(b, 2692000u, true, 0.0f, 0u));    // reports the up
    TEST_ASSERT_TRUE(b.router.wifiRadioWanted());
    for (uint32_t t = 3592000u; t <= 10001000u; t += 900000u)
        TEST_ASSERT_EQUAL_INT(0, wt(b, t, true, 0.0f, 0u));       // hours: never

    // And a live one-minute setting fires like clockwork — with a station
    // tick to start the window honestly (the hours of idleM=0 above did not
    // quietly age a window that had not been running).
    b.run("SET WIFI_IDLE_MIN=1", r);
    TEST_ASSERT_EQUAL_INT(0, wt(b, 10002000u, true, 0.0f, 1u));   // window start
    TEST_ASSERT_EQUAL_INT(0, wt(b, 10061000u, true, 0.0f, 0u));   // 59 s
    TEST_ASSERT_EQUAL_INT(-1, wt(b, 10062000u, true, 0.0f, 0u));  // 60 s: live without reboot
    b.run("SET WIFI_IDLE_MIN=10", r);
}

void test_the_diver_outranks_the_machine_and_depth_outranks_the_words() {
    Bench b;
    CommandRouter::Response r;
    wt(b, 1000u, true, 0.0f, 0u);

    // Diver says off. A whole depth cycle must not resurrect the radio:
    // every depth release restores the INTENT, never overrules it.
    b.run("WIFI OFF", r);
    TEST_ASSERT_TRUE(r.actions & CommandRouter::ACT_WIFI);
    TEST_ASSERT_EQUAL_UINT8(CommandRouter::WH_DIVER, b.router.wifiHold());
    // The tick reports the diver's own down (the command applied the radio;
    // the tick is the printer — loopTask prints every transition, whoever
    // caused it). The latch tracked depth on the same tick without an extra
    // report: intent was already holding the radio, so nothing changed then.
    TEST_ASSERT_EQUAL_INT(-1, wt(b, 5000u, true, 5.0f, 0u));
    TEST_ASSERT_EQUAL_UINT8(CommandRouter::WH_DIVER, b.router.wifiHold());
    TEST_ASSERT_EQUAL_INT(0, wt(b, 61000u, true, 0.0f, 0u));     // shallow 60 s: latch down
    TEST_ASSERT_EQUAL_UINT8(CommandRouter::WH_DIVER, b.router.wifiHold());
    b.run("GET WIFI", r);
    TEST_ASSERT_EQUAL_STRING("WIFI,0,1,0,0\n", r.text);

    // Words cannot clear depth. At 5 m, WIFI ON sets the intent — and raises
    // NO radio action, because the effective state is unchanged. The client
    // gets OK (truthful: intent is set) and GET WIFI shows the hold.
    // Intent is OFF at this point; at 5 m the latch is up and the words set
    // intent WITHOUT raising a radio action (effective state is unchanged:
    // held by depth either way). This is the wire's copy of "words cannot
    // clear depth".
    TEST_ASSERT_EQUAL_INT(0, wt(b, 65000u, true, 5.0f, 0u));
    b.run("WIFI ON", r);
    TEST_ASSERT_TRUE(b.router.wifiEnabled());
    TEST_ASSERT_FALSE_MESSAGE(r.actions & CommandRouter::ACT_WIFI,
                              "WIFI ON at depth asked for the radio the policy holds");
    b.run("GET WIFI", r);
    TEST_ASSERT_EQUAL_STRING("WIFI,0,3,0,0\n", r.text);

    // A commanded dive at depth: DIVE START at 5 m changes no effective state
    // (the radio was already down for depth) — and GET WIFI reports DIVE while
    // the dive runs, because the dive is the closer reason.
    const uint32_t tDive = 66000u;
    b.now = tDive;
    b.run("DIVE START", r);
    TEST_ASSERT_TRUE(r.actions & CommandRouter::ACT_DIVE_START);
    TEST_ASSERT_FALSE(r.actions & CommandRouter::ACT_WIFI);
    b.run("GET WIFI", r);
    TEST_ASSERT_EQUAL_STRING("WIFI,0,2,0,0\n", r.text);

    // DIVE END AT DEPTH (the behaviour change of this batch): intent is
    // restored, the DEPTH latch holds the radio, and no setRadio is asked for.
    b.now = tDive + 30000u;
    b.run("DIVE END", r);
    TEST_ASSERT_TRUE(r.actions & CommandRouter::ACT_DIVE_END);
    TEST_ASSERT_FALSE_MESSAGE(r.actions & CommandRouter::ACT_WIFI,
                              "dive end brought the radio up at depth");
    b.run("GET WIFI", r);
    TEST_ASSERT_EQUAL_STRING("WIFI,0,3,0,0\n", r.text);            // depth again, not dive

    // The diver ascends: 60 straight seconds below 1 m, and only now does
    // the radio come up — from the latch clearing, on the intent the dive
    // end restored.
    TEST_ASSERT_EQUAL_INT(0, wt(b, tDive + 31000u, true, 0.5f, 0u));
    TEST_ASSERT_EQUAL_INT(0, wt(b, tDive + 90000u, true, 0.5f, 0u));
    TEST_ASSERT_EQUAL_INT(1, wt(b, tDive + 91000u, true, 0.5f, 0u));
    b.run("GET WIFI", r);
    TEST_ASSERT_EQUAL_STRING("WIFI,1,0,600,0\n", r.text);

    // Surface dive, the old behaviour: nothing about dive start/end at the
    // surface changes, so every existing dive-cycle check stays honest.
    b.now += 1000u;
    b.run("DIVE START", r);
    TEST_ASSERT_TRUE(r.actions & CommandRouter::ACT_WIFI);        // up->down
    b.run("DIVE END", r);
    TEST_ASSERT_TRUE(r.actions & CommandRouter::ACT_WIFI);        // down->up

    // SAFE outranks everything — and it is reachable WITH a live policy tick:
    // the SAFE boot path initialises the session (handset gets ACK-only) while
    // web never starts and loopTask still runs wifiTick. Without the SAFE term
    // in wifiRadioWanted(), a SAFE board's policy believes its radio should be
    // up — and a later idle fire would drive setRadio(false) on a radio that
    // was never on, then any intent event setRadio(true) on the one board SAFE
    // exists to keep silent. The tick is the only actor here, so the tick is
    // where the outranking must show.
    b.router.setSafeLockout(true);
    TEST_ASSERT_EQUAL_INT(-1, wt(b, b.now + 1000u, true, 0.0f, 0u));   // reports the down
    TEST_ASSERT_FALSE(b.router.wifiRadioWanted());
    for (uint32_t t = b.now + 10000u; t <= b.now + 70000u; t += 10000u)
        TEST_ASSERT_EQUAL_INT(0, wt(b, t, true, 0.0f, 0u));            // stays down
    b.router.setSafeLockout(false);   // ACK reboots for real in firmware; the
                                      // lockout term alone is under test
    TEST_ASSERT_EQUAL_INT(1, wt(b, b.now + 80000u, true, 0.0f, 0u));   // intent, restored
}

void test_the_wifi_policy_oscillates_by_design_when_the_depths_are_inverted() {
    // WIFI_ON_DEPTH above WIFI_OFF_DEPTH is a configuration the schema cannot
    // express as a key-vs-key bound (min/max are per key). The policy does NOT
    // silently clamp it — silent correction is a settings lie, the same family
    // as shortening a passphrase. It runs as written, which oscillates, and
    // this case is the sentence in the settings table made executable: what an
    // inversion DOES is documented by the test, not by a promise.
    Bench b;
    CommandRouter::Response r;
    b.run("SET WIFI_OFF_DEPTH=1.0", r);
    b.run("SET WIFI_ON_DEPTH=5.0", r);
    b.run("SET WIFI_ON_HOLD_S=10", r);
    TEST_ASSERT_EQUAL_INT(-1, wt(b, 1000u, true, 3.0f, 0u));      // latches: 3 > 1
    TEST_ASSERT_EQUAL_INT(0, wt(b, 6000u, true, 3.0f, 0u));       // "below 5" timer starts here
    TEST_ASSERT_EQUAL_INT(0, wt(b, 11000u, true, 3.0f, 0u));      // 5 s of the 10
    TEST_ASSERT_EQUAL_INT(1, wt(b, 16000u, true, 3.0f, 0u));      // 10 s: up
    TEST_ASSERT_EQUAL_INT(-1, wt(b, 17000u, true, 3.0f, 0u));     // re-latched: down again
    // Flap period = WIFI_ON_HOLD_S. This is why the schema ranges keep the two
    // apart (ON max 20 < OFF max 30 is not the guard — the ranges merely make
    // the honest settings the easy ones to type).
}
int main() {
    UNITY_BEGIN();
    RUN_TEST(test_auto_lockout_holds_over_the_router);
    RUN_TEST(test_get_set_round_trip_on_the_shared_registry);
    RUN_TEST(test_a_published_max_length_value_lands_whole);
    RUN_TEST(test_dive_locked_keys_refused_mid_dive);
    RUN_TEST(test_re_dive_after_a_closed_dive_is_a_fresh_dive);
    RUN_TEST(test_dive_start_drops_the_radio_and_end_restores_it);
    RUN_TEST(test_diver_outranks_the_machine_cellproof_override);
    RUN_TEST(test_schema_lists_every_key_and_fits_the_buffer);
    RUN_TEST(test_reused_reply_buffer_never_leaks_the_previous_reply);
    RUN_TEST(test_safe_mode_lockout_refuses_everything_but_ack);
    RUN_TEST(test_ack_always_honoured);
    RUN_TEST(test_testcells_runs_on_the_bench_and_reports_masks);
    RUN_TEST(test_deco_answers_err_when_nothing_published_the_picture);
    RUN_TEST(test_deco_and_status_lines_come_from_the_chain);
    RUN_TEST(test_set_actually_moves_the_live_chain);
    RUN_TEST(test_cross_key_sets_are_clamped_before_they_reach_the_chain);
    RUN_TEST(test_get_t_projects_cns_exactly_as_the_pushed_stream_does);
    RUN_TEST(test_log_and_calibration_commands_delegations);
    RUN_TEST(test_unimplemented_calibrations_are_refused_not_acked);
    RUN_TEST(test_get_calair_reports_the_verdict_not_the_request);
    RUN_TEST(test_time_set_is_range_checked_and_delegated);
    RUN_TEST(test_get_depth_source_names_the_chip_and_says_if_it_is_used);
    RUN_TEST(test_a_depth_source_name_cannot_run_past_its_field);
    RUN_TEST(test_the_wifi_policy_holds_on_depth_and_releases_shallow_and_straight);
    RUN_TEST(test_the_idle_hold_counts_only_real_time_up_and_words_clear_it);
    RUN_TEST(test_the_diver_outranks_the_machine_and_depth_outranks_the_words);
    RUN_TEST(test_the_wifi_policy_oscillates_by_design_when_the_depths_are_inverted);
    return UNITY_END();
}
