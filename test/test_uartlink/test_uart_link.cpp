// ============================================================================
// test_uart_link — the handset session: what a diver's terminal actually
// receives. Commands answer, periodic lines hold their cadence, events from
// the control core reach the wire as E, lines (and grab an event-driven
// snapshot with them), and a dive log that fails to open is announced —
// an unlogged dive is acceptable, an unannounced one is not. A long LOG EXPORT
// is graded too: every row once, in order, in one call, and the watchdog feed
// that rides on that row loop only ever follows a row that reached the wire
// (review M-1).
// ============================================================================
#include <unity.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <algorithm>
#include <string>
#include <vector>

#include "comms/telemetry_bridge.h"
#include "comms/uart_link.h"
#include "control/cellproof.h"
#include "log/rowfill.h"
#include "util/wdtfeed.h"

using comms::CommandRouter;
using comms::UartLink;
using namespace ctrl;

void setUp() {}
void tearDown() {}

namespace {

std::vector<std::string>* g_out = nullptr;  // sink context (test-local, single-thread)
void* g_sinkCtx = nullptr;                  // the ctx the caller's sink last saw

void sinkLine(const char* line, void* ctx) {
    g_sinkCtx = ctx;
    g_out->push_back(line);
}

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
    // The instant and picture each event-driven snapshot row was built from
    // (bridge::EdgeSnap). One drain batch must hand over ONE capture: eight
    // edges drained together are eight rows about eight decisions, and if the
    // numbers underneath them differ between the rows then the rows are eight
    // measurements of eight control cycles, which is what the capture exists
    // to prevent (review M-4's sibling).
    struct SnapRec {
        uint32_t nowMs = 0;
        uint32_t diveStartMs = 0;
        float depthM = 0.0f;
        float shownPpo2 = 0.0f;
        bool hasPic = false;
        int16_t ceilingCm = 0;
        int16_t ndlMin = 0;
    };
    std::vector<SnapRec> snapRecs;
    // When true, the hook moves the bridge scalars after every row it is handed:
    // the control task does exactly that while the command task is halfway across
    // a drain batch. Set only by the coherence test below.
    bool bumpBetweenRows = false;
    logfmt::DiveMeta lastMeta{};
    bool hasMeta = false;
    bool open(uint32_t) { opens++; return !openFails; }
    void close(const logfmt::DiveMeta& m) { closes++; lastMeta = m; hasMeta = true; }
};

} // namespace

struct Fixture;
static Fixture* g_fix = nullptr;
static void* g_exportCtx = reinterpret_cast<void*>(0x1);  // 'not called yet'

// A 6 h dive at the 30 s grid is ~720 snapshot rows, which is the size that
// actually outran the task watchdog on the bench (review M-1). 400 rows is
// enough to cross the feed interval (util::kWdtFeedEvery) two dozen times,
// which is what makes the row-counting wrapper's bookkeeping observable here.
constexpr int kLongExportRows = 400;

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
        // On the board, ControlTask publishes a deco generation every cycle
        // and the `D,` stream answers from that publication (deco/view.h —
        // review N8: this task no longer walks the engine). A fixture that
        // never published would simulate a board where control never ran,
        // and its first answer would be the ERR, not a D, line.
        deco::View pic{};
        deco::viewFull(pic, loop.deco(),
                       loop.deco().depthToBar(loop.depthSmoothedM()),
                       loop.haveVote(), loop.shownPpo2());
        deco::publishView(tel.view, pic);
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
        h.appendSnapNow = [](const bridge::EdgeSnap& s, void*) {
            if (!g_fix) return;
            g_fix->hooks.snaps++;
            g_fix->hooks.appends.push_back("S");
            Hooks::SnapRec r;
            r.nowMs = s.nowMs;
            r.diveStartMs = s.diveStartMs;
            r.depthM = s.depthM;
            r.shownPpo2 = s.shownPpo2;
            r.hasPic = s.hasPic;
            r.ceilingCm = s.pic.ceilingCm;
            r.ndlMin = s.pic.ndlMin;
            g_fix->hooks.snapRecs.push_back(r);
            // Stand in for ControlTask integrating between the rows of one drain
            // batch. The capture the rows were built from must not move with it.
            if (g_fix->hooks.bumpBetweenRows) {
                g_fix->tel.depthM += 1.0f;
                g_fix->tel.shownPpo2 += 0.05f;
            }
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
            if (!g_fix) return -1;
            // Slot 5 stands for a dive with an afternoon in it, because what is
            // being exercised here is the row LOOP. The store hands the whole
            // dive to the link one row at a time (DiveStore::exportCsv ->
            // logfmt::emitCsvRows), and since 2026-11-26 the link counts those
            // calls so the task doing the streaming can feed the watchdog it is
            // registered with (review M-1, util/wdtfeed.h). A wrapper that
            // dropped, duplicated or reordered rows to get its count would be
            // invisible in a three-row export, so this one is long.
            if (slot == 5) {
                char row[24];
                sink("DIVE,5,1700000000,3\n", ctx);
                for (int i = 0; i < kLongExportRows; i++) {
                    std::snprintf(row, sizeof(row), "S,%d\n", i);
                    sink(row, ctx);
                }
                return kLongExportRows;
            }
            if (slot != 3) return -1;
            // Every call back carries the ctx the export was handed. That is the
            // store's contract (logfmt::emitCsvRows passes it to every row), and
            // the link's row counter now depends on it, so the fixture must not
            // wave it away with a nullptr the real store would never pass.
            sink("CTX-SEEN\n", ctx);
            sink("DIVE,3,1700000000,3\n", ctx);
            sink("S,30,25.00,0.700,0.700,0.700,0.700,0.700,0x0000,12,0,480,0,5,12500\n", ctx);
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
    f.tel.shownPpo2 = 0.62f;
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

// The `T,` FLAGS byte is a projection of the control chain's alarm word, done
// twice — once here for the handset link, once in cmd_router.cpp for `GET T` and
// the web — and the two mappings have to agree. CNS_HIGH is the one condition in
// the word that is not a machine fault: bit 3 is the aggregate for "the loop is
// not telling you what you need" (sensor fail/stale, stuck-open, hypoxia), and a
// diver over his oxygen budget is reading a working sensor. Folding CNS into that
// aggregate would ask a handset to fix a fault that does not exist, so CNS got a
// free bit (7) instead of a renumbering. Graded here on the wire, because the
// alarm word itself is unchanged and nothing else would notice the difference.
void test_cns_high_gets_its_own_flag_bit_and_stays_out_of_the_fault_aggregate() {
    std::vector<std::string> out;
    g_out = &out;
    Fixture f;
    f.tel.depthM = 18.0f;
    f.tel.shownPpo2 = 1.30f;
    for (int i = 0; i < 3; i++) f.tel.cellBar[i] = 1.30f;
    f.tel.setpointBar = 1.30f;
    f.tel.vbat = 12.4f;
    f.tel.haveVote = true;
    f.tel.diveActive = true;
    f.tel.autoMode = true;
    f.tel.cpState = static_cast<uint8_t>(cellproof::State::PASS);

    auto flagsOf = [&](uint32_t tMs) {
        out.clear();
        f.pump(tMs);
        for (const auto& l : out) {
            const size_t p = l.find("FLAGS0x");
            if (l.compare(0, 2, "T,") == 0 && p != std::string::npos)
                return std::strtoul(l.c_str() + p + 7, nullptr, 16);
        }
        TEST_FAIL_MESSAGE("no T, line with a FLAGS field");
        return 0UL;
    };

    // CNS alone: exactly bit 7. Nothing else may ride along — bit 3 in
    // particular, because that bit is a handset's "go check the sensors".
    f.tel.alarmBits = 1u << alarms::BIT_CNS_HIGH;
    TEST_ASSERT_EQUAL_UINT32(0x80u, flagsOf(0));

    // A real sensor fault: bit 3, and no invented CNS bit beside it. The
    // aggregate still means what §4 publishes it to mean.
    f.tel.alarmBits = 1u << alarms::BIT_SENSOR_STALE;
    TEST_ASSERT_EQUAL_UINT32(0x08u, flagsOf(500));

    // Both: two separate facts, readable separately by a handset that cares
    // about one of them.
    f.tel.alarmBits = (1u << alarms::BIT_SENSOR_STALE) | (1u << alarms::BIT_CNS_HIGH);
    TEST_ASSERT_EQUAL_UINT32(0x88u, flagsOf(1000));

    // The cell-proof FAIL pill (bit 6) is not an alarm bit at all, so it has to
    // coexist with bit 7 without either swallowing the other.
    f.tel.alarmBits = 1u << alarms::BIT_CNS_HIGH;
    f.tel.cpState = static_cast<uint8_t>(cellproof::State::FAIL);
    TEST_ASSERT_EQUAL_UINT32(0xC0u, flagsOf(1500));
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

void test_a_staged_batch_is_invisible_until_it_is_committed() {
    // beginTx/commitTx is the ordering the event row's freshness depends on. An
    // edge must not be drainable in the window between push() and the deco
    // republish that belongs to it: on the board the handset task runs on the
    // other core, and a drain landing in that window filed "the machine decided
    // X" beside the plan of the last 30 s grid — N1's freshness rule broken by
    // the order of two statements next to each other (main.cpp).
    bridge::EventRing ring;
    bridge::Event e{};
    ring.beginTx();
    TEST_ASSERT_TRUE(ring.push({10, logfmt::EVT_ALARM_RAISED, alarms::BIT_LOW_PPO2}));
    TEST_ASSERT_TRUE(ring.push({11, logfmt::EVT_MODE_CHANGE, 1}));
    TEST_ASSERT_FALSE_MESSAGE(ring.pop(e),
                              "a staged event is already readable: a drain could pair the edge "
                              "with a deco picture computed before it happened");
    ring.commitTx();
    TEST_ASSERT_TRUE(ring.pop(e));
    TEST_ASSERT_EQUAL_UINT32(10u, e.tSec);
    TEST_ASSERT_EQUAL_UINT16(static_cast<uint16_t>(logfmt::EVT_ALARM_RAISED), e.id);
    TEST_ASSERT_EQUAL_UINT16(alarms::BIT_LOW_PPO2, e.value);
    TEST_ASSERT_TRUE(ring.pop(e));
    TEST_ASSERT_EQUAL_UINT32(11u, e.tSec);
    TEST_ASSERT_FALSE_MESSAGE(ring.pop(e),
                              "the commit published more than the batch it staged");
}

void test_a_push_outside_a_batch_is_readable_at_once() {
    // Not every producer stages. A caller outside ControlTask's cycle (the host
    // tests drive publishEvents this way) must not have its event held back
    // waiting for a commit that never comes, and a commit with nothing staged
    // must be harmless rather than publish a stale cursor.
    bridge::EventRing ring;
    bridge::Event e{};
    TEST_ASSERT_TRUE(ring.push({5, logfmt::EVT_MODE_CHANGE, 2}));
    TEST_ASSERT_TRUE(ring.pop(e));
    TEST_ASSERT_EQUAL_UINT32(5u, e.tSec);
    ring.commitTx();
    TEST_ASSERT_FALSE(ring.pop(e));
    // And staging is still available afterwards: tx_ is not sticky.
    ring.beginTx();
    TEST_ASSERT_TRUE(ring.push({6, logfmt::EVT_MODE_CHANGE, 3}));
    TEST_ASSERT_FALSE(ring.pop(e));
    ring.commitTx();
    TEST_ASSERT_TRUE(ring.pop(e));
    TEST_ASSERT_EQUAL_UINT32(6u, e.tSec);
}

void test_a_staged_batch_still_respects_the_rings_capacity() {
    // Staging must not turn the ring into an unbounded buffer: the producer that
    // fills it is an alarm storm, and the honest answer to a full ring is a
    // refused push the caller counts (`dropped()`), not silent overwrite.
    bridge::EventRing ring;
    bridge::Event e{};
    ring.beginTx();
    int ok = 0;
    for (int i = 0; i < 64; i++)
        if (ring.push({static_cast<uint32_t>(i), logfmt::EVT_ALARM_RAISED,
                       static_cast<uint16_t>(i)}))
            ok++;
    TEST_ASSERT_EQUAL_INT_MESSAGE(static_cast<int>(bridge::EventRing::kSize) - 1, ok,
                                  "a staged batch holds something other than kSize-1 events: "
                                  "the ring's full condition is producer-cursor based");
    ring.commitTx();
    int seen = 0;
    while (ring.pop(e)) seen++;
    TEST_ASSERT_EQUAL_INT(ok, seen);
}

void test_one_drain_batch_reads_one_instant() {
    // Eight edges drained together are eight rows about eight decisions, and the
    // numbers underneath them must be ONE measurement. Reading the live scalars
    // per event made each row a sample of a different control cycle, so a dive
    // file showing the clamp engage beside an override showed the ppO2 and depth
    // drifting between the rows that recorded them.
    //
    // The hook here moves the bridge scalars after every row: that is what the
    // control task does while the command task is halfway across a batch.
    std::vector<std::string> out;
    g_out = &out;
    Fixture f;
    f.tel.depthM = 18.35f;
    f.tel.shownPpo2 = 0.712f;
    f.tel.diveStartMs = 5000u;
    f.hooks.bumpBetweenRows = true;

    f.events.beginTx();
    TEST_ASSERT_TRUE(f.events.push({12, logfmt::EVT_ALARM_RAISED, alarms::BIT_LOW_PPO2}));
    TEST_ASSERT_TRUE(f.events.push({12, logfmt::EVT_MODE_CHANGE, 1}));
    TEST_ASSERT_TRUE(f.events.push({12, logfmt::EVT_CLAMP_ON, 0}));
    f.events.commitTx();

    f.pump(8000);
    TEST_ASSERT_EQUAL_MESSAGE(3u, f.hooks.snapRecs.size(), "one snapshot row per drained event");
    const Hooks::SnapRec& first = f.hooks.snapRecs[0];
    TEST_ASSERT_TRUE_MESSAGE(first.hasPic,
                             "the batch took no deco picture: the fixture publishes a generation");
    TEST_ASSERT_EQUAL_FLOAT_MESSAGE(18.35f, first.depthM,
                                    "the batch's instant is not the depth the control task held");
    for (size_t i = 1; i < f.hooks.snapRecs.size(); i++) {
        const Hooks::SnapRec& r = f.hooks.snapRecs[i];
        // Unity's message is a plain C string, so the row number is composed
        // rather than printf'd — and a failing row has to say WHICH row.
        const std::string which = ", row " + std::to_string(i);
        TEST_ASSERT_EQUAL_UINT32_MESSAGE(first.nowMs, r.nowMs,
                                         ("the rows of one drain carry different stamps" + which).c_str());
        TEST_ASSERT_EQUAL_FLOAT_MESSAGE(first.depthM, r.depthM,
                                        ("a row of one drain was captured from a later control "
                                         "cycle: the rows of a batch must describe one instant" +
                                         which).c_str());
        TEST_ASSERT_EQUAL_FLOAT_MESSAGE(first.shownPpo2, r.shownPpo2,
                                        ("a row of one drain shows a different ppO2" + which).c_str());
        TEST_ASSERT_EQUAL_INT16_MESSAGE(first.ceilingCm, r.ceilingCm,
                                        ("a row of one drain carries a different deco picture" +
                                         which).c_str());
        TEST_ASSERT_EQUAL_INT16_MESSAGE(first.ndlMin, r.ndlMin,
                                        ("a row of one drain carries a different NDL" + which).c_str());
    }
    // The row's seconds come from the CHAIN's dive start — the same base the E,
    // line beside it was counted from — not from the instant this task happened
    // to open the file. A pump at 8000 ms on a dive started at 5000 ms is t_s = 3.
    TEST_ASSERT_EQUAL_UINT32(5000u, first.diveStartMs);
}

void test_a_capture_before_the_first_picture_says_there_is_no_picture() {
    // SAFE boot, or the first cycles of a cold start: ControlTask has published
    // nothing yet, so the picture a row would carry does not exist. hasPic is the
    // honest answer, and the fill turns it into the `ndl` sentinel rather than a
    // 0-minute NDL — which reads "your no-deco time is finished", the opposite
    // claim and the more alarming one.
    bridge::Telemetry fresh;
    bridge::EdgeSnap s{};
    bridge::captureEdge(fresh, nullptr, 1234u, s);
    TEST_ASSERT_FALSE_MESSAGE(s.hasPic,
                              "captureEdge claims a deco picture nobody has published");
    TEST_ASSERT_EQUAL_INT16(0, s.pic.ceilingCm);

    logfmt::RowInputs ri{};
    ri.nowMs = s.nowMs;
    ri.diveStartMs = s.diveStartMs ? s.diveStartMs : s.nowMs;
    ri.hasDecoPicture = s.hasPic;
    ri.deco = s.pic;
    logfmt::Snapshot row{};
    logfmt::fillSnapshot(row, ri);
    TEST_ASSERT_EQUAL_INT16_MESSAGE(deco::kNDL_UNKNOWN, row.ndlMin,
                                    "a row with no published picture states an NDL of 0 minutes");
    TEST_ASSERT_EQUAL_INT16(0, row.ceilingCm);
    TEST_ASSERT_EQUAL_INT16(0, row.decoTotalMin);
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
// Since M-1 the link sits between the two (it counts rows to keep the watchdog
// fed), so the contract is pinned at both ends now: the export is never handed
// the firmware's hook object, and the caller's sink still gets the caller's own
// ctx back — which is the fact a WebSocket sink actually depends on.
void test_an_export_hands_the_callers_sink_context_back() {
    std::vector<std::string> out;
    g_out = &out;
    Fixture f;
    int callerMarker = 0x5a;
    g_exportCtx = reinterpret_cast<void*>(0x1);
    g_sinkCtx = reinterpret_cast<void*>(0x1);
    f.sendWithCtx("LOG EXPORT 3", &callerMarker);
    // hw_.ctx is the firmware's hook object: handing it to a transport sink is
    // the wild-pointer cast this case exists to catch.
    TEST_ASSERT_TRUE_MESSAGE(g_exportCtx != static_cast<void*>(&f),
                             "export ran with the firmware's ctx, not the caller's");
    TEST_ASSERT_TRUE_MESSAGE(g_sinkCtx == &callerMarker,
                             "the caller's sink received someone else's ctx");
    TEST_ASSERT_TRUE(hasLineStarting("CTX-SEEN"));   // and the sink kept using it
}

// M-1: a read-only handset command must not be able to watchdog-panic a dive.
// LOG EXPORT holds no dive lock and streams EVERY row of the dive through the
// asking transport's sink in one handleLine() call; on the handset that sink is
// Serial2.print at 115200 (~87 µs/byte, no yield), and the uart2 task feeds the
// task watchdog once per pump. A long dive's export therefore blew the watchdog
// budget of a registered task, which on this build panics — a mid-dive reset
// into SAFE mode, caused by downloading a dive. The link now feeds from inside
// the export's per-row sink (util/wdtfeed.h).
//
// What the host can grade is the half that a feed change could break: the
// stream. The feed rides on the row loop, so the row loop must still hand over
// every row, exactly once, in order, in the one call — 400 rows of it. A
// wrapper that counted rows it never sent, or dropped one at the wrap, or
// stopped early to "get back to the pump", is invisible on a three-line export.
void test_a_long_export_streams_every_row_once() {
    std::vector<std::string> out;
    g_out = &out;
    Fixture f;

    f.send("LOG EXPORT 5");

    // One OK, one `DIVE,` identity line, then every row. Not one more, not one
    // fewer: the export has no framing and no resume, so a row lost inside the
    // wrapper is a row the diver never gets.
    TEST_ASSERT_EQUAL_MESSAGE(kLongExportRows + 2, static_cast<int>(out.size()),
                              "a long export did not stream every row in one call");
    TEST_ASSERT_TRUE_MESSAGE(!out.empty() && out[0].compare(0, 2, "OK") == 0,
                             "the reply is not the first line of the export");
    TEST_ASSERT_TRUE_MESSAGE(out.size() > 1 && out[1].compare(0, 5, "DIVE,") == 0,
                             "the DIVE identity line is not the first streamed row");
    for (int i = 0; i < kLongExportRows; i++) {
        const std::string want = "S," + std::to_string(i) + "\n";
        TEST_ASSERT_EQUAL_STRING_MESSAGE(want.c_str(), out[2 + i].c_str(),
                                         "a row was lost, duplicated or reordered");
    }
}

// ---- the source-scan kit (the same three helpers test_controlplane uses) ----
// The ESP watchdog call cannot execute on the host, so the rule that keeps the
// M-1 fix honest is pinned against the source. Comments are stripped first:
// test_controlplane's mutation run (M6) proved a green scan that is only green
// because the banned/called string survived in a comment.
namespace {

std::string readFile(const char* path) {
    FILE* f = std::fopen(path, "rb");
    if (!f) return "";
    std::string out;
    char buf[4096];
    size_t n = 0;
    while ((n = std::fread(buf, 1, sizeof(buf), f)) > 0) out.append(buf, n);
    std::fclose(f);
    return out;
}

// A top-level function: from its header line through the first line that is
// exactly "}" (project formatting puts closing braces at col 0).
std::string functionBody(const std::string& src, const std::string& header) {
    const size_t at = src.find(header);
    if (at == std::string::npos) return "";
    const size_t end = src.find("\n}\n", at);
    if (end == std::string::npos) return "";
    return src.substr(at, end - at + 3);
}

std::string stripComments(const std::string& s) {
    std::string out;
    enum { CODE, LINE, BLOCK } st = CODE;
    for (size_t i = 0; i < s.size();) {
        const char c = s[i];
        const char n = i + 1 < s.size() ? s[i + 1] : '\0';
        if (st == CODE) {
            if (c == '/' && n == '/') { st = LINE; i += 2; continue; }
            if (c == '/' && n == '*') { st = BLOCK; i += 2; continue; }
            out.push_back(c);
            ++i;
        } else if (st == LINE) {
            if (c == '\n') { st = CODE; out.push_back(c); }
            ++i;
        } else {
            if (c == '*' && n == '/') { st = CODE; i += 2; continue; }
            ++i;
        }
    }
    return out;
}

} // namespace

// The feed's cadence, graded on the host because the ESP call itself cannot be:
// every Nth completed row, never more often (the point is a light hot path) and
// never further apart than N rows (the point is the watchdog window). A constant
// that stopped being a power of two would silently break the mask below.
void test_the_feed_cadence_is_every_n_completed_rows() {
    int feeds = 0;
    for (unsigned rows = 1; rows <= 400; rows++)
        if (util::wdtFeedDue(rows)) feeds++;
    TEST_ASSERT_EQUAL_MESSAGE(400 / static_cast<int>(util::kWdtFeedEvery), feeds,
                              "the watchdog feed is not once per N rows");
    TEST_ASSERT_FALSE(util::wdtFeedDue(1));
    TEST_ASSERT_TRUE(util::wdtFeedDue(util::kWdtFeedEvery));
    TEST_ASSERT_FALSE(util::wdtFeedDue(util::kWdtFeedEvery + 1));
    TEST_ASSERT_TRUE(util::wdtFeedDue(util::kWdtFeedEvery * 2));
    // A feed at the point of the work is only honest if the work is bounded by
    // N; the mask only holds while the interval is a power of two.
    TEST_ASSERT_TRUE_MESSAGE((util::kWdtFeedEvery & (util::kWdtFeedEvery - 1)) == 0,
                             "kWdtFeedEvery is not a power of two: wdtFeedDue's mask lies");

    // On the host there is no watchdog to feed, and the call must still be safe
    // to make from a task that was never registered with one — the same call the
    // web transport makes from a task deliberately left unregistered (PLAN 9).
    // If the guard ever leaked an ESP call into the host build, this file would
    // not compile, which is the point.
    util::wdtFeed();
}

// The source scan half of M-1. The ESP watchdog call cannot run on the host, so
// the rule that makes the fix safe is pinned as text: the link's row sink hands
// the row to the caller's sink BEFORE it feeds, and it feeds only every N rows.
// A feed hoisted above the sink would keep a wedged UART alive indefinitely —
// exactly the hang uart2_task.cpp's header says must reach the supervisor.
void test_the_export_feeds_only_after_a_row_reached_the_wire() {
    const std::string ul = readFile("src/comms/uart_link.cpp");
    TEST_ASSERT_TRUE_MESSAGE(!ul.empty(), "cannot read src/comms/uart_link.cpp");
    const std::string body = stripComments(functionBody(ul, "void exportRowSink("));
    TEST_ASSERT_TRUE_MESSAGE(!body.empty(),
                             "exportRowSink not found: renamed? the scan must keep seeing it");
    const size_t sent = body.find("e.sink(");
    const size_t fed = body.find("wdtFeed()");
    TEST_ASSERT_TRUE_MESSAGE(sent != std::string::npos && fed != std::string::npos,
                             "the export's row sink no longer both sends the row and feeds");
    TEST_ASSERT_TRUE_MESSAGE(sent < fed,
                             "the watchdog feed moved above the row write: a blocked UART "
                             "would then be fed while it emitted nothing");
    TEST_ASSERT_TRUE_MESSAGE(body.find("wdtFeedDue") != std::string::npos,
                             "the export feeds every row instead of every Nth");
    // And the export must go through that wrapper, not the caller's bare sink.
    const std::string exp = stripComments(
        functionBody(ul, "void UartLink::streamLogExport("));
    TEST_ASSERT_TRUE_MESSAGE(!exp.empty(), "streamLogExport not found");
    TEST_ASSERT_TRUE_MESSAGE(exp.find("hw_.exportDive(slot, exportRowSink") != std::string::npos,
                             "the export bypasses the counting row sink: the row loop is "
                             "unbounded again, and the task that runs it is TWDT-registered");
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
    RUN_TEST(test_cns_high_gets_its_own_flag_bit_and_stays_out_of_the_fault_aggregate);
    RUN_TEST(test_events_reach_the_wire_and_snapshot_the_log);
    RUN_TEST(test_event_rows_are_written_into_the_dive_file_beside_their_snapshot);
    RUN_TEST(test_a_staged_batch_is_invisible_until_it_is_committed);
    RUN_TEST(test_a_push_outside_a_batch_is_readable_at_once);
    RUN_TEST(test_a_staged_batch_still_respects_the_rings_capacity);
    RUN_TEST(test_one_drain_batch_reads_one_instant);
    RUN_TEST(test_a_capture_before_the_first_picture_says_there_is_no_picture);
    RUN_TEST(test_dive_summary_states_the_plan_the_dive_ran_on);
    RUN_TEST(test_publish_events_from_a_real_chain_cycle);
    RUN_TEST(test_dive_lifecycle_opens_and_closes_the_log_and_announces_failure);
    RUN_TEST(test_log_list_and_export_stream_over_the_session);
    RUN_TEST(test_publish_state_is_not_static_between_dives);
    RUN_TEST(test_cellproof_clamp_edges_publish_once);
    RUN_TEST(test_cal_air_verdicts_reach_the_wire_once_each);
    RUN_TEST(test_an_export_hands_the_callers_sink_context_back);
    RUN_TEST(test_a_long_export_streams_every_row_once);
    RUN_TEST(test_the_feed_cadence_is_every_n_completed_rows);
    RUN_TEST(test_the_export_feeds_only_after_a_row_reached_the_wire);
    RUN_TEST(test_the_depth_source_publisher_leaves_nothing_to_interpret);
    return UNITY_END();
}
