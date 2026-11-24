// ============================================================================
// test_uart_link — the handset session: what a diver's terminal actually
// receives. Commands answer, periodic lines hold their cadence, events from
// the control core reach the wire as E, lines (and grab an event-driven
// snapshot with them), and a dive log that fails to open is announced —
// an unlogged dive is acceptable, an unannounced one is not.
// ============================================================================
#include <unity.h>

#include <cstring>
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
    int wifi = 0, calAir = 0, calDepth = 0, settings = 0, snaps = 0;
    bool wifiLast = false, saveFails = false, openFails = false;
    int closes = 0, opens = 0;
    bool open(uint32_t) { opens++; return !openFails; }
    void close(const logfmt::DiveMeta&) { closes++; }
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
        h.appendSnapNow = [](void*) { if (g_fix) g_fix->hooks.snaps++; };
        h.saveSettings = [](void*) { return g_fix && !g_fix->hooks.saveFails; };
        h.onCalAir = [](void*) { if (g_fix) g_fix->hooks.calAir++; };
        h.onCalDepth = [](void*) { if (g_fix) g_fix->hooks.calDepth++; };
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
    f.send("CAL DEPTH");
    TEST_ASSERT_EQUAL(1, f.hooks.calDepth);

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

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_command_replies_and_hooks_fire);
    RUN_TEST(test_periodic_lines_keep_their_cadence);
    RUN_TEST(test_events_reach_the_wire_and_snapshot_the_log);
    RUN_TEST(test_publish_events_from_a_real_chain_cycle);
    RUN_TEST(test_dive_lifecycle_opens_and_closes_the_log_and_announces_failure);
    RUN_TEST(test_log_list_and_export_stream_over_the_session);
    RUN_TEST(test_publish_state_is_not_static_between_dives);
    RUN_TEST(test_an_export_hands_the_callers_sink_context_back);
    return UNITY_END();
}
