// ============================================================================
// test_uart_link — the handset session: what a diver's terminal actually
// receives. Commands answer, periodic lines hold their cadence, events from
// the control core reach the wire as E, lines (and grab an event-driven
// snapshot with them), and a dive log that fails to open is announced —
// an unlogged dive is acceptable, an unannounced one is not.
// ============================================================================
#include <unity.h>

#include <cstring>
#include <algorithm>
#include <string>
#include <vector>

#include "comms/telemetry_bridge.h"
#include "comms/uart_link.h"

using comms::CommandRouter;
using comms::UartLink;
using namespace ctrl;

void setUp() {}
void tearDown() {}

namespace {

std::vector<std::string>* g_out = nullptr;  // sink context (test-local, single-thread)

void sinkLine(const char* line, void*) { g_out->push_back(line); }

bool hasLineStarting(const char* prefix) {
    for (auto& l : *g_out)
        if (l.compare(0, strlen(prefix), prefix) == 0) return true;
    return false;
}

struct Hooks {
    int wifi = 0, calAir = 0, settings = 0, snaps = 0;
    bool wifiLast = false, saveFails = false, openFails = false;
    int closes = 0, opens = 0;
    // What the dive file was handed, in call order: "E<t>,<id>,<value>" for an
    // event row and "S" for a snapshot. The order matters — the verdict row has
    // to be in the file alongside the state it caused, not somewhere else.
    std::vector<std::string> appends;
    logfmt::DiveMeta lastMeta{};
    bool hasMeta = false;
    bool open(uint32_t) { opens++; return !openFails; }
    void close(const logfmt::DiveMeta& m) { closes++; lastMeta = m; hasMeta = true; }
};

} // namespace

struct Fixture;
static Fixture* g_fix = nullptr;
static void* g_exportCtx = reinterpret_cast<void*>(0x1);  // 'not called yet'

struct Fixture {
    Config cfg;
    Loop loop{cfg};
    settings::Registry reg;
    uint32_t now = 0;
    CommandRouter router;
    bridge::Telemetry tel;
    bridge::EventRing events;
    bridge::DiveTotals totals;
    Hooks hooks;
    UartLink link;

    Fixture()
        : reg(), router(loop, reg, now), link(loop, reg, router, tel, events, hw()) {
        g_fix = this;   // hw() ran before this line; the hooks dereference g_fix
                        // only when the link fires, which is always after here.
    }

    UartLink::Hardware hw() {
        // static trampolines into a file-static fixture pointer: single test,
        // single thread, and the alternative (std::function) is not what the
        // firmware will use either. g_fix may still be null while the very
        // constructor is initializing (link is a member), hence the guards.
        UartLink::Hardware h;
        h.ctx = this;
        h.totals = &totals;
        h.onWifi = [](bool on, void*) { if (g_fix) { g_fix->hooks.wifi++; g_fix->hooks.wifiLast = on; } };
        h.openDive = [](uint32_t, void*) { return g_fix && g_fix->hooks.open(0); };
        h.closeDive = [](const logfmt::DiveMeta& m, void*) { if (g_fix) g_fix->hooks.close(m); };
        h.appendSnapNow = [](void*) {
            if (!g_fix) return;
            g_fix->hooks.snaps++;
            g_fix->hooks.appends.push_back("S");
        };
        h.appendEventNow = [](uint16_t id, uint16_t value, uint32_t tSec, void*) {
            if (!g_fix) return;
            g_fix->hooks.appends.push_back("E," + std::to_string(tSec) + "," +
                                           std::to_string(id) + "," +
                                           std::to_string(value));
        };
        h.saveSettings = [](void*) { return g_fix && !g_fix->hooks.saveFails; };
        h.onCalAir = [](void*) { if (g_fix) g_fix->hooks.calAir++; };
        h.onSettings = [](void*) { if (g_fix) g_fix->hooks.settings++; };
        h.listDives = [](int* slots, uint32_t* mt, uint32_t* seq, int max, void*) {
            if (!g_fix) return 0;
            int n = max < 2 ? max : 2;
            for (int i = 0; i < n; i++) {
                slots[i] = i + 1;
                mt[i] = 1700000000u + i;
                if (seq) seq[i] = 40u + i;      // the write order, not the epoch
            }
            return n;
        };
        h.exportDive = [](int slot, UartLink::Sink sink, void* ctx) {
            // The sink we were handed belongs to the caller, so the context
            // must be the caller's too: a transport sink that casts its ctx
            // (a WebSocket client id) would otherwise be handed the
            // firmware's hook object. Recorded here so the test can see it.
            g_exportCtx = ctx;
            if (!g_fix || slot != 3) return -1;
            sink("CTX-SEEN\n", ctx);
            sink("DIVE,3,1700000000,3\n", nullptr);
            sink("S,30,25.00,0.700,0.700,0.700,0.700,0.700,0x0000,12,0,480,0,5,12500\n", nullptr);
            return 1;
        };
        return h;
    }

    void send(const char* line) { link.handleLine(line, sinkLine, nullptr); }
    void sendWithCtx(const char* line, void* ctx) { link.handleLine(line, sinkLine, ctx); }
    void pump(uint32_t tMs) {
        router.setNowMs(tMs);
        now = tMs;
        link.pump(sinkLine, nullptr, tMs);
    }
};

void test_command_replies_and_hooks_fire() {
    std::vector<std::string> out;
    g_out = &out;
    Fixture f;

    f.send("WIFI OFF");
    TEST_ASSERT_TRUE(hasLineStarting("OK"));
    TEST_ASSERT_EQUAL(1, f.hooks.wifi);
    TEST_ASSERT_FALSE(f.hooks.wifiLast);

    out.clear();
    f.send("SET GF_LOW=35");
    TEST_ASSERT_TRUE(hasLineStarting("OK SAFETY"));
    TEST_ASSERT_EQUAL(1, f.hooks.settings);   // exactly one applier call

    out.clear();
    f.send("CAL AIR");
    TEST_ASSERT_EQUAL(1, f.hooks.calAir);

    // CAL DEPTH asks for a depth offset capture nothing performs, so it is
    // refused and, crucially, no hook fires: a client must not be able to set
    // in motion a calibration that has no consumer.
    out.clear();
    f.send("CAL DEPTH");
    TEST_ASSERT_TRUE(hasLineStarting("ERR not implemented"));
    TEST_ASSERT_EQUAL(1, f.hooks.calAir);   // unchanged: no hook fired for it

    out.clear();
    f.send("auto");
    TEST_ASSERT_TRUE(hasLineStarting("ERR"));
}

void test_periodic_lines_keep_their_cadence() {
    std::vector<std::string> out;
    g_out = &out;
    Fixture f;
    f.tel.votedBar = 0.62f;
    f.tel.cellBar[0] = 0.62f; f.tel.cellBar[1] = 0.63f; f.tel.cellBar[2] = 0.61f;
    f.tel.setpointBar = 0.70f;
    f.tel.depthM = 18.0f;
    f.tel.haveVote = true;
    f.tel.diveActive = true;
    f.tel.autoMode = true;
    f.tel.alarmBits = 0;

    f.pump(0);
    TEST_ASSERT_TRUE(hasLineStarting("T,"));
    TEST_ASSERT_TRUE(hasLineStarting("D,"));      // first pump: both due
    size_t tAfter1 = 0, dAfter1 = 0;
    for (auto& l : out) { if (l[0] == 'T') tAfter1++; if (l[0] == 'D') dAfter1++; }
    TEST_ASSERT_EQUAL(1u, tAfter1);
    TEST_ASSERT_EQUAL(1u, dAfter1);

    for (uint32_t t = 100; t <= 2000; t += 100) f.pump(t);
    size_t tCount = 0, dCount = 0;
    for (auto& l : out) { if (l[0] == 'T') tCount++; if (l[0] == 'D') dCount++; }
    TEST_ASSERT_EQUAL(5u, tCount);   // 0,500,...,2000: five T, lines
    TEST_ASSERT_EQUAL(2u, dCount);   // 0 and 2000 -> a quarter of the rate

    // a T, line must carry all three raw cells even when one is out of the vote
    bool sawCells = false;
    for (auto& l : out)
        if (l.compare(0, 2, "T,") == 0 && l.find("0.620,0.630,0.610") != std::string::npos)
            sawCells = true;
    TEST_ASSERT_TRUE_MESSAGE(sawCells, "T, line lost the per-cell readings");
}

void test_events_reach_the_wire_and_snapshot_the_log() {
    std::vector<std::string> out;
    g_out = &out;
    Fixture f;

    // ControlTask published an alarm edge through the ring:
    TEST_ASSERT_TRUE(f.events.push({42, logfmt::EVT_ALARM_RAISED, alarms::BIT_LOW_PPO2}));
    TEST_ASSERT_TRUE(f.events.push({43, logfmt::EVT_MODE_CHANGE, 1}));

    f.pump(1000);
    TEST_ASSERT_TRUE(hasLineStarting("E,42,1,0"));
    TEST_ASSERT_TRUE(hasLineStarting("E,43,3,1"));
    TEST_ASSERT_EQUAL(2, f.hooks.snaps);   // each event = a moment of interest
    f.pump(1500);                          // drained: nothing repeats
    size_t eCount = 0;
    for (auto& l : out) if (l[0] == 'E') eCount++;
    TEST_ASSERT_EQUAL(2u, eCount);
}

void test_event_rows_are_written_into_the_dive_file_beside_their_snapshot() {
    // The wire has always carried these rows; the dive file never did. A dive
    // downloaded afterwards showed the alarm word change and the setpoint get
    // capped — the EFFECTS — without the verdict that caused them (cellproof
    // FAIL, clamp ON, the diver's override). Same publication, both records,
    // same numbers: this is what stops the file and the wire drifting apart.
    std::vector<std::string> out;
    g_out = &out;
    Fixture f;

    TEST_ASSERT_TRUE(f.events.push({54, logfmt::EVT_CELLPROOF_FAIL, 0}));
    TEST_ASSERT_TRUE(f.events.push({54, logfmt::EVT_CLAMP_ON, 0}));
    TEST_ASSERT_TRUE(f.events.push({66, logfmt::EVT_CLAMP_OFF, 1}));

    f.pump(1000);

    // In the file, as three rows, with the dive-relative time the wire used...
    TEST_ASSERT_EQUAL(6u, f.hooks.appends.size());
    const std::vector<std::string> want{
        "E,54,10,0", "S", "E,54,11,0", "S", "E,66,12,1", "S"};
    TEST_ASSERT_EQUAL_MESSAGE(want.size(), f.hooks.appends.size(),
                              "wrong number of rows handed to the dive store");
    for (size_t i = 0; i < want.size(); i++) {
        TEST_ASSERT_EQUAL_STRING_MESSAGE(want[i].c_str(), f.hooks.appends[i].c_str(),
                                         "dive-file row out of order or wrong value");
    }
    // ...the verdict row BEFORE the state snapshot that goes with it, and the
    // same bytes the handset saw live.
    for (size_t i = 0; i + 1 < f.hooks.appends.size(); i += 2)
        TEST_ASSERT_EQUAL_STRING("S", f.hooks.appends[i + 1].c_str());
    TEST_ASSERT_TRUE(hasLineStarting("E,54,10,0"));
    TEST_ASSERT_TRUE(hasLineStarting("E,54,11,0"));
    TEST_ASSERT_TRUE(hasLineStarting("E,66,12,1"));
}

void test_dive_summary_states_the_plan_the_dive_ran_on() {
    // `M,` has had gf_low/gf_high/sp1..sp4 and snapshots/events columns since v2,
    // and toMeta() filled 6 of the 14 fields — so every dive on the board ended
    // with "gf 0/0, sp 0.000 0.000 0.000 0.000, snapshots 0, events 0" while the
    // file beside it held the GF pair, the ladder and a dozen samples. A dive
    // whose own summary contradicts it cannot be reviewed later.
    bridge::DiveTotals t;
    t.durationSec = 66;
    t.maxDepthCm = 743;
    t.minVotedX1000 = 212;
    t.maxVotedX1000 = 215;
    t.totalPulses = 7;
    t.cnsPeakPct = 3;
    t.gfLow = 30;
    t.gfHigh = 85;
    t.sp1X1000 = 400;
    t.sp2X1000 = 1000;
    t.sp3X1000 = 1000;
    t.sp4X1000 = 1300;

    const logfmt::DiveMeta m = t.toMeta();
    TEST_ASSERT_EQUAL_UINT32(66u, m.durationSec);
    TEST_ASSERT_EQUAL_INT(743, m.maxDepthCm);
    TEST_ASSERT_EQUAL_INT16(212, m.minVotedX1000);
    TEST_ASSERT_EQUAL_INT16(215, m.maxVotedX1000);
    TEST_ASSERT_EQUAL_UINT16(7u, m.totalPulses);
    TEST_ASSERT_EQUAL_UINT8(3u, m.cnsPeakPct);
    TEST_ASSERT_EQUAL_UINT8(30u, m.gfLow);
    TEST_ASSERT_EQUAL_UINT8(85u, m.gfHigh);
    TEST_ASSERT_EQUAL_INT16(400, m.sp1X1000);
    TEST_ASSERT_EQUAL_INT16(1000, m.sp2X1000);
    TEST_ASSERT_EQUAL_INT16(1000, m.sp3X1000);
    TEST_ASSERT_EQUAL_INT16(1300, m.sp4X1000);

    // A dive that never voted: the sentinels must not be published as numbers.
    bridge::DiveTotals quiet;
    const logfmt::DiveMeta q = quiet.toMeta();
    TEST_ASSERT_EQUAL_INT16(0, q.minVotedX1000);
    // The max side is the one that used to leak: `maxVotedX1000` STARTS at
    // -32768, and the old guard (`< -32768`) can never be true, so a dive with
    // no cells reading wrote -32.768 bar as its own maximum ppO2 — an impossible
    // value in the same three decimals as every real one.
    TEST_ASSERT_EQUAL_INT16_MESSAGE(0, q.maxVotedX1000,
                                    "the never-voted sentinel was published as a ppO2");
    TEST_ASSERT_EQUAL_UINT8(0u, q.gfLow);      // and an unset plan stays 0, never
    TEST_ASSERT_EQUAL_INT16(0, q.sp1X1000);    // a value someone made up

    // The count columns are NOT taken from the chain: the store counts the rows
    // it wrote (hookCloseDive patches them), so toMeta leaving them zero is the
    // contract, not a gap — asserted here so a "helpful" fill here cannot
    // silently double-count.
    TEST_ASSERT_EQUAL_UINT16(0u, m.eventCount);
    TEST_ASSERT_EQUAL_UINT16(0u, m.snapshotCount);
}

void test_publish_events_from_a_real_chain_cycle() {
    // The publisher is driven by ctrl::State, not hand-built states: an
    // alarm that fires in the chain must land in the ring exactly once.
    std::vector<std::string> out;
    g_out = &out;
    Fixture f;

    bridge::PublisherState ps;
    uint32_t now = 0;
    for (int i = 0; i < 100; i++) {
        SensorData sd;
        for (int c = 0; c < 3; c++) sd.cells[c] = 0.10f;   // hypoxic: alarms low
        sd.depthM = 0.0f; sd.vbat = 12.4f;
        now += Loop::kTickMs;
        const State st = f.loop.cycle(sd, now);
        bridge::publishEvents(st, now / 1000, f.events, ps);
    }
    // The ring drains onto the wire: one E, line for the alarm transition,
    // not one per control cycle, and nothing on the following pump.
    f.pump(now + 1000);
    size_t eCount = 0;
    for (auto& l : out)
        if (l.compare(0, 1, "E") == 0 && l.find(",1,0") != std::string::npos) eCount++;
    TEST_ASSERT_EQUAL_MESSAGE(1u, eCount, "chain alarm event missing or repeated on the wire");
    f.pump(now + 2000);
    eCount = 0;
    for (auto& l : out)
        if (l.compare(0, 1, "E") == 0 && l.find(",1,0") != std::string::npos) eCount++;
    TEST_ASSERT_EQUAL(1u, eCount);
}

void test_dive_lifecycle_opens_and_closes_the_log_and_announces_failure() {
    std::vector<std::string> out;
    g_out = &out;
    Fixture f;

    f.send("DIVE START");
    TEST_ASSERT_EQUAL(1, f.hooks.opens);
    TEST_ASSERT_EQUAL(0, f.hooks.closes);

    f.send("DIVE END");
    TEST_ASSERT_EQUAL(1, f.hooks.closes);

    // log fails to open: the reply chain must say so on the same session
    out.clear();
    f.hooks.openFails = true;
    f.send("DIVE START");
    TEST_ASSERT_TRUE(hasLineStarting("ERR LOG FAILED"));
}

void test_log_list_and_export_stream_over_the_session() {
    std::vector<std::string> out;
    g_out = &out;
    Fixture f;

    f.send("LOG LIST");
    TEST_ASSERT_TRUE(hasLineStarting("L,1,"));
    TEST_ASSERT_TRUE(hasLineStarting("L,2,"));
    // The appended fourth field is the dive sequence: on a board with no clock
    // every epoch in the listing is 0, and without it a client has ten dives and
    // no order. Appending is the only change allowed to an existing line, and a
    // three-field client is unaffected by it.
    TEST_ASSERT_TRUE(hasLineStarting("L,1,1700000000,40"));
    TEST_ASSERT_TRUE(hasLineStarting("L,2,1700000001,41"));

    out.clear();
    f.send("LOG EXPORT 3");
    TEST_ASSERT_TRUE(hasLineStarting("DIVE,3,"));
    TEST_ASSERT_TRUE(hasLineStarting("S,30,"));

    out.clear();
    f.send("LOG EXPORT 7");
    TEST_ASSERT_TRUE(hasLineStarting("ERR no such dive"));
}

// LOG EXPORT over a WebSocket is the case that bites: the export streams
// through the *transport's* sink, so it must receive the transport's context.
void test_an_export_hands_the_callers_sink_context_back() {
    std::vector<std::string> out;
    g_out = &out;
    Fixture f;
    int callerMarker = 0x5a;
    g_exportCtx = reinterpret_cast<void*>(0x1);
    f.sendWithCtx("LOG EXPORT 3", &callerMarker);
    TEST_ASSERT_TRUE_MESSAGE(g_exportCtx == &callerMarker,
                             "export ran with the firmware's ctx, not the caller's");
    TEST_ASSERT_TRUE(hasLineStarting("CTX-SEEN"));   // and the sink kept using it
}

void test_publish_state_is_not_static_between_dives() {
    // A second chain (or a second dive object in the sim) must not inherit
    // another chain's "previous" alarm bits and fabricate a CLEAR event.
    bridge::EventRing ring;
    bridge::PublisherState psA, psB;

    SensorData sd;
    for (int c = 0; c < 3; c++) sd.cells[c] = 0.10f;
    Loop a{Config{}}, b{Config{}};
    const State sa = a.cycle(sd, 100);
    bridge::publishEvents(sa, 0, ring, psA);
    const State sb = b.cycle(sd, 100);
    bridge::publishEvents(sb, 0, ring, psB);   // fresh state: must re-publish

    int raised = 0;
    bridge::Event e;
    while (ring.pop(e))
        if (e.id == logfmt::EVT_ALARM_RAISED) raised++;
    TEST_ASSERT_EQUAL(2, raised);              // one per chain, not one total
}

// ids 11/12 (clamp ON / OFF) were defined in the log format and never published:
// the diver-visible fact "your setpoints are capped until you say otherwise"
// existed only as a FLAGS bit and a `PROOF,` field, so a dive file could not say
// whether a clamp was applied or who lifted it. Driven through the real engine,
// because what matters here is the EDGE: FAIL is a state that lasts hundreds of
// cycles, and a per-cycle publisher would bury the dive in rows.
void test_cellproof_clamp_edges_publish_once() {
    bridge::EventRing ring;
    bridge::PublisherState ps;
    Loop loop{Config{}};

    SensorData sd{};
    for (int c = 0; c < 3; c++) sd.cells[c] = 0.45f;   // fitted, flat, never reaches 1.00
    sd.vbat = 12.4f;
    sd.depthM = 3.0f;                                  // shallow: the proof arms...

    uint32_t now = 0;
    loop.postStartDive(now);
    auto cycle = [&](int n) {
        for (int i = 0; i < n; ++i) {
            now += Loop::kTickMs;
            const State st = loop.cycle(sd, now);
            bridge::publishEvents(st, now / 1000, ring, ps);
        }
    };
    cycle(20);
    sd.depthM = 12.0f;                                 // ...and runs on the crossing
    cycle(700);                                        // 70 s: proof times out -> FAIL

    // The clamp is up, and the chain agrees with the publisher about that.
    TEST_ASSERT_TRUE_MESSAGE(loop.clampActive(), "the cellproof FAIL did not clamp");
    cycle(100);                                        // 10 s more in FAIL: no repeats

    std::vector<int> ids;
    bridge::Event e;
    int offValue = -1;
    while (ring.pop(e)) {
        ids.push_back(e.id);
        if (e.id == logfmt::EVT_CLAMP_OFF) offValue = e.value;
    }
    const int fail = static_cast<int>(std::count(ids.begin(), ids.end(), logfmt::EVT_CELLPROOF_FAIL));
    const int on = static_cast<int>(std::count(ids.begin(), ids.end(), logfmt::EVT_CLAMP_ON));
    TEST_ASSERT_EQUAL_MESSAGE(1, fail, "cellproof FAIL must publish exactly once");
    TEST_ASSERT_EQUAL_MESSAGE(1, on, "CLAMP_ON must publish exactly once per clamp");
    TEST_ASSERT_EQUAL(0, std::count(ids.begin(), ids.end(), logfmt::EVT_CLAMP_OFF));
    // Verdict first, then the consequence the diver acts on — the order a person
    // reading the dive file afterwards should see.
    const auto itFail = std::find(ids.begin(), ids.end(), logfmt::EVT_CELLPROOF_FAIL);
    const auto itOn = std::find(ids.begin(), ids.end(), logfmt::EVT_CLAMP_ON);
    TEST_ASSERT_TRUE(itFail != ids.end() && itOn != ids.end() && itOn > itFail);

    // The diver lifts it: `CELLPROOF OVERRIDE` queues the lift for the control
    // tick, exactly as the handset asks for it.
    loop.postCpOverride();
    cycle(5);
    TEST_ASSERT_FALSE(loop.clampActive());

    int off = 0, onAfter = 0;
    while (ring.pop(e)) {
        if (e.id == logfmt::EVT_CLAMP_OFF) { off++; offValue = e.value; }
        if (e.id == logfmt::EVT_CLAMP_ON) onAfter++;
    }
    TEST_ASSERT_EQUAL_MESSAGE(1, off, "CLAMP_OFF must publish when the diver lifts the clamp");
    TEST_ASSERT_EQUAL(1, offValue);                     // value 1 = lifted by OVERRIDE
    TEST_ASSERT_EQUAL_MESSAGE(0, onAfter, "lifting the clamp must not invent a second CLAMP_ON");
}

// The CAL AIR verdict used to exist only as text on the USB console, while the
// wire said `OK CAL AIR` whatever happened next. This is the publication that
// changed that, pinned at the level the diver's client sees: one `E,` row per
// verdict (not one per 100 ms control cycle), the bridge::CalAirCode vocabulary
// as the row's value, and the raw numbers readable afterwards.
void test_cal_air_verdicts_reach_the_wire_once_each() {
    std::vector<std::string> out;
    g_out = &out;
    Fixture f;
    bridge::CalAir cal;

    // Refused before the sampler even started (the warm-up gate): the client
    // that was told `OK CAL AIR` must be able to learn it was refused.
    bridge::publishCalAirRefused(cal, bridge::CALAIR_WARMUP, 117, 3000, 3, f.events);
    TEST_ASSERT_EQUAL_UINT8(bridge::CALAIR_WARMUP, cal.code);
    TEST_ASSERT_EQUAL_UINT32(117u, static_cast<uint32_t>(cal.warmupLeftS));
    bridge::Event e;
    TEST_ASSERT_TRUE(f.events.pop(e));
    TEST_ASSERT_EQUAL_UINT16(logfmt::EVT_CAL_AIR, e.id);
    TEST_ASSERT_EQUAL_UINT16(bridge::CALAIR_WARMUP, e.value);
    TEST_ASSERT_EQUAL_UINT32(3u, e.tSec);
    TEST_ASSERT_FALSE_MESSAGE(f.events.pop(e),
                              "a refusal must be one row, not one per control cycle");

    // Sampling: a state, so no row — 12 samples would otherwise bury the dive in
    // rows for one fact. `GET CALAIR` reports the progress instead.
    cal::AirCalOutput run;
    run.running = true;
    run.samples = 4;
    bridge::publishCalAir(cal, run, 9000, 9, f.events);
    TEST_ASSERT_EQUAL_UINT8(bridge::CALAIR_RUNNING, cal.code);
    TEST_ASSERT_EQUAL_UINT8(4, cal.samples);
    TEST_ASSERT_FALSE_MESSAGE(f.events.pop(e), "CAL AIR progress must not emit events");

    // Committed, with the numbers an installer needs (raw counts, worst spread).
    cal::AirCalOutput ok;
    ok.done = true;
    ok.samples = 12;
    ok.spreadWorst = 23.0f;
    for (int i = 0; i < 3; i++) ok.rawAvg[i] = 1500.0f + i;
    bridge::publishCalAir(cal, ok, 21000, 21, f.events);
    TEST_ASSERT_EQUAL_UINT8(bridge::CALAIR_OK, cal.code);
    TEST_ASSERT_EQUAL_UINT8(12, cal.samples);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1501.0f, static_cast<float>(cal.rawAvg[1]));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 23.0f, static_cast<float>(cal.spreadWorst));
    TEST_ASSERT_EQUAL_UINT32(0u, static_cast<uint32_t>(cal.warmupLeftS));
    TEST_ASSERT_TRUE(f.events.pop(e));
    TEST_ASSERT_EQUAL_UINT16(bridge::CALAIR_OK, e.value);
    TEST_ASSERT_EQUAL_UINT32(21u, e.tSec);

    // Each failure reason is its own code: "it failed" is not a diagnosis, and
    // the diver does something different for each of them.
    struct Case { cal::AirCalWhy why; uint16_t want; };
    const Case cases[] = {
        {cal::AirCalWhy::DIVING, bridge::CALAIR_DIVING},
        {cal::AirCalWhy::UNSETTLED, bridge::CALAIR_UNSETTLED},
        {cal::AirCalWhy::NOCELLS, bridge::CALAIR_NOCELLS},
        {cal::AirCalWhy::NONE, bridge::CALAIR_UNSETTLED},  // unmapped: never code 0
    };
    bridge::EventRing ring;
    for (const Case& c : cases) {
        cal::AirCalOutput fl;
        fl.failed = true;
        fl.whyCode = c.why;
        bridge::publishCalAir(cal, fl, 30000, 30, ring);
        TEST_ASSERT_TRUE(ring.pop(e));
        TEST_ASSERT_EQUAL_UINT16(logfmt::EVT_CAL_AIR, e.id);
        TEST_ASSERT_EQUAL_UINT16_MESSAGE(c.want, e.value,
                                         "failure reason reached the wire as the wrong code");
    }
    TEST_ASSERT_FALSE(ring.pop(e));

    // And through the session's own handset downlink: the ring is not the end of
    // the road, the handset's E, line is.
    out.clear();
    bridge::publishCalAirRefused(cal, bridge::CALAIR_WARMUP, 90, 40000, 40, f.events);
    f.pump(41000);
    TEST_ASSERT_TRUE_MESSAGE(hasLineStarting("E,40,7,3"),
                             "the refusal did not reach the handset downlink");
}

// ---- the depth-source snapshot (what `GET DS` answers from) -----------------

void test_the_depth_source_publisher_leaves_nothing_to_interpret() {
    // bridge::publishDepthSource exists so the rules are testable at all: the
    // call site is in main.cpp, which the native build excludes, so rules written
    // there would be rules no suite can see. Three of them, each a different lie
    // on a diver-facing field.
    struct Padded { bridge::DepthSource ds; char guard[8]; } pad;
    std::memcpy(pad.guard, "GUARD!!!", sizeof(pad.guard));

    // A named, locked source: every field passes through.
    bridge::publishDepthSource(pad.ds, true, true, true, 4321u, "LPS28DFW @0x5D");
    TEST_ASSERT_TRUE(pad.ds.locked);
    TEST_ASSERT_TRUE(pad.ds.used);
    TEST_ASSERT_TRUE(pad.ds.healthy);
    TEST_ASSERT_EQUAL_UINT32(4321u, pad.ds.lastGoodMs);
    TEST_ASSERT_EQUAL_STRING_MESSAGE("LPS28DFW @0x5D", pad.ds.identity,
                                     "the name a diver needs is not the name the driver gave");

    // Unlocked: there is no source to be used, healthy, or 400 s silent, and
    // there is no name. Forcing all five together is what lets `DS,0,0,0,0,none`
    // be read as one complete statement instead of "which of these fields mean
    // anything this cycle?". A name surviving an unlocked slot sends a reader to
    // check a chip the arbiter does not own.
    bridge::publishDepthSource(pad.ds, false, true, true, 400'000u, "LPS28DFW @0x5D");
    TEST_ASSERT_FALSE(pad.ds.locked);
    TEST_ASSERT_FALSE_MESSAGE(pad.ds.used, "an unlocked slot is still 'used'");
    TEST_ASSERT_FALSE_MESSAGE(pad.ds.healthy, "an unlocked slot still claims a healthy driver");
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(0u, pad.ds.lastGoodMs,
                                     "an unlocked slot keeps a reading time: `GET DS` would "
                                     "then report an age for a sensor that owns nothing");
    TEST_ASSERT_EQUAL_STRING_MESSAGE("none", pad.ds.identity,
                                     "an unlocked slot kept the old sensor's name");

    // A name longer than the array is truncated, not run over. The guard after
    // the struct is what makes an unbounded copy visible: without it, the copy
    // lands in whatever follows the global and nothing here notices until a
    // crash somewhere unrelated.
    char longish[64];
    std::memset(longish, 'B', sizeof(longish) - 1);
    longish[sizeof(longish) - 1] = '\0';
    bridge::publishDepthSource(pad.ds, true, true, true, 10u, longish);
    TEST_ASSERT_LESS_OR_EQUAL_INT_MESSAGE(static_cast<int>(sizeof(pad.ds.identity)) - 1,
                                          static_cast<int>(std::strlen(pad.ds.identity)),
                                          "a too-long identity overran the array");
    pad.ds.identity[sizeof(pad.ds.identity) - 1] = '\0';   // belt: the assert above
    TEST_ASSERT_EQUAL_MEMORY_MESSAGE("GUARD!!!", pad.guard, 4,
                                     "the identity copy wrote past the struct");
    TEST_ASSERT_EQUAL_CHAR('\0', pad.ds.identity[std::strlen(pad.ds.identity)]);

    // A null name is a driver bug, and the honest answer to it is "none" — the
    // alternative is a blank field the reader fills in himself.
    bridge::publishDepthSource(pad.ds, true, true, true, 10u, nullptr);
    TEST_ASSERT_EQUAL_STRING_MESSAGE("none", pad.ds.identity,
                                     "a null identity produced an empty field, not 'none'");

    // A source that still holds the slot keeps its name while it goes quiet:
    // used/healthy drop, age grows, and the name stays, because "which chip" and
    // "is it answering" are different questions and the answer to the first does
    // not become "none" just because the second went false.
    bridge::publishDepthSource(pad.ds, true, true, true, 100u, "MS5837-30BA @0x76");
    bridge::publishDepthSource(pad.ds, true, false, false, 100u, "MS5837-30BA @0x76");
    TEST_ASSERT_TRUE(pad.ds.locked);
    TEST_ASSERT_FALSE(pad.ds.used);
    TEST_ASSERT_EQUAL_STRING_MESSAGE("MS5837-30BA @0x76", pad.ds.identity,
                                     "a quiet source lost its name");
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_command_replies_and_hooks_fire);
    RUN_TEST(test_periodic_lines_keep_their_cadence);
    RUN_TEST(test_events_reach_the_wire_and_snapshot_the_log);
    RUN_TEST(test_event_rows_are_written_into_the_dive_file_beside_their_snapshot);
    RUN_TEST(test_dive_summary_states_the_plan_the_dive_ran_on);
    RUN_TEST(test_publish_events_from_a_real_chain_cycle);
    RUN_TEST(test_dive_lifecycle_opens_and_closes_the_log_and_announces_failure);
    RUN_TEST(test_log_list_and_export_stream_over_the_session);
    RUN_TEST(test_publish_state_is_not_static_between_dives);
    RUN_TEST(test_cellproof_clamp_edges_publish_once);
    RUN_TEST(test_cal_air_verdicts_reach_the_wire_once_each);
    RUN_TEST(test_an_export_hands_the_callers_sink_context_back);
    RUN_TEST(test_the_depth_source_publisher_leaves_nothing_to_interpret);
    return UNITY_END();
}
