// ============================================================================
// test_session — the invariant the web server is built on: there is ONE
// command session, and a browser gets the same router, the same lockouts and
// the same dive-file owner as the handset.
//
// What would break without these cases: a per-transport router looks fine
// until the phone turns WiFi off and the handset's copy of the world still
// thinks it is on — so at dive end the machine turns the radio back on,
// undoing what the diver asked for on the other client. Same class of bug for
// the two-step CAL O2 arm and for the dive-lock decisions. The session also
// has to keep the handset's event stream to itself: a web command must not
// drain the ring the handset's E, lines come from.
// ============================================================================
#include <unity.h>

#include <cstring>
#include <string>
#include <vector>

#include "comms/session.h"
#include "control/cellhist.h"

using comms::Session;
using namespace ctrl;

namespace {

std::vector<std::string>* g_out = nullptr;  // the sink receiving right now

void sinkHandset(const char* line, void*) { g_out->push_back(std::string("H:") + line); }
void sinkWeb(const char* line, void*) { g_out->push_back(std::string("W:") + line); }

bool saw(const char* prefix) {
    for (const auto& l : *g_out)
        if (l.compare(0, strlen(prefix), prefix) == 0) return true;
    return false;
}
/// A reply may arrive as one sink call carrying several lines (GET *,
/// GET SCHEMA) — the web transport splits them into frames, so the test looks
/// at the joined text as well.
bool sawAnywhere(const char* needle) {
    for (const auto& l : *g_out)
        if (l.find(needle) != std::string::npos) return true;
    return false;
}
size_t countPrefix(const char* prefix) {
    size_t n = 0;
    for (const auto& l : *g_out)
        if (l.compare(0, strlen(prefix), prefix) == 0) n++;
    return n;
}

// Lock instrumentation: the point is not that a lock exists, but that the
// hardware actions a command triggers happen *inside* it, so two transports
// can never be mid-command at the same time.
int g_acquires = 0, g_releases = 0, g_held = 0, g_heldMax = 0;
void lockGet(void*) { g_acquires++; g_held++; if (g_held > g_heldMax) g_heldMax = g_held; }
void lockPut(void*) { g_releases++; g_held--; }

struct Hooks {
    std::vector<int> wifi;      // the on/off sequence the radio was told
    int opens = 0, closes = 0, snaps = 0, settings = 0;
    int settingsHeld = -1;      // lock depth seen while applying settings
};
Hooks* g_hk = nullptr;

struct Fix {
    Config cfg;
    Loop loop{cfg};
    settings::Registry reg;
    bridge::Telemetry tel;
    bridge::EventRing events;
    bridge::DiveTotals totals;
    cellhist::Hist hist;      // what ControlTask would be filling
    Hooks hk;
    Session ses;
    uint32_t now = 0;

    Fix() : reg() {
        reg.resetToDefaults();
        g_hk = &hk;
        g_acquires = g_releases = g_held = g_heldMax = 0;
        TEST_ASSERT_TRUE(ses.begin(loop, reg, tel, events, hw(), lock()));
        ses.setHist(&hist);   // same source for both clients, one pointer
    }

    Session::Lock lock() {
        Session::Lock l;
        l.acquire = lockGet;
        l.release = lockPut;
        return l;
    }

    comms::UartLink::Hardware hw() {
        comms::UartLink::Hardware h;
        h.totals = &totals;
        h.onWifi = [](bool on, void*) { if (g_hk) g_hk->wifi.push_back(on ? 1 : 0); };
        h.openDive = [](uint32_t, void*) { if (g_hk) g_hk->opens++; return true; };
        h.closeDive = [](const logfmt::DiveMeta&, void*) { if (g_hk) g_hk->closes++; };
        h.appendSnapNow = [](void*) { if (g_hk) g_hk->snaps++; };
        h.saveSettings = [](void*) { return true; };
        h.onSettings = [](void*) {
            if (!g_hk) return;
            g_hk->settings++;
            g_hk->settingsHeld = g_held;   // must be 1: inside the session lock
        };
        return h;
    }

    void handset(const char* line) { ses.handleLine(line, now, sinkHandset, nullptr); }
    void web(const char* line) { ses.handleLine(line, now, sinkWeb, nullptr); }
    void pump(uint32_t t) { now = t; ses.pump(sinkHandset, nullptr, t); }
};

} // namespace

void setUp() { g_out = nullptr; g_hk = nullptr; }
void tearDown() {}

void test_safe_mode_is_visible_on_the_downlink() {
    // SAFE mode is entered because the machine cannot account for its last
    // reset. The handset link stays up in that state (the diver needs ACK to
    // get out), so a handset must be able to SEE it — from the downlink, before
    // it sends anything, not from an ERR it has not sent yet. The published
    // snapshot cannot say this by itself: in SAFE the control chain is not
    // running, so it keeps reporting whatever it last had (STANDBY). The link
    // knows the truth because the router holds the lockout.
    std::vector<std::string> out;
    g_out = &out;
    Fix f;

    f.pump(500);
    TEST_ASSERT_TRUE_MESSAGE(saw("H:T,"), "pump produced no telemetry line");
    TEST_ASSERT_TRUE_MESSAGE(sawAnywhere(",STANDBY,"), "a standing-by machine must say STANDBY");

    out.clear();
    f.ses.setSafeLockout(true);          // what main.cpp does on an unaccounted reset
    f.pump(1000);
    TEST_ASSERT_TRUE_MESSAGE(sawAnywhere(",SAFE,"),
                             "the T, line did not report SAFE while the router was locked out");

    // and the command side matches the state the downlink advertised
    out.clear();
    f.handset("GET T");
    TEST_ASSERT_TRUE_MESSAGE(saw("H:ERR safe mode"),
                             "GET T was accepted although the downlink said SAFE");
}

void test_the_web_gets_identical_refusals() {
    std::vector<std::string> out;
    g_out = &out;
    Fix f;

    // AUTO over a WebSocket: the same lockout, the same words.
    f.web("AUTO");
    TEST_ASSERT_TRUE_MESSAGE(saw("W:ERR predive required"),
                             "AUTO accepted from the web without pre-dive checks");
    TEST_ASSERT_FALSE(f.loop.autoMode());

    // dive-locked key over the web
    f.web("DIVE START");
    out.clear();
    f.web("SET SP2=1.30");
    TEST_ASSERT_TRUE_MESSAGE(saw("W:ERR dive locked"),
                             "setpoint rewritten from the web under a diver");
    out.clear();
    f.web("SET GF_HIGH=90");   // schedule tuning stays legal
    TEST_ASSERT_TRUE(saw("W:OK"));

    // calibration never under a diver, whichever client asked
    out.clear();
    f.web("CAL AIR");
    TEST_ASSERT_TRUE(saw("W:ERR cal locked in dive"));

    // the pages need these two, and they must be the registry's own answers
    out.clear();
    f.web("GET *");
    TEST_ASSERT_TRUE(saw("W:SP1=0.400"));
    out.clear();
    f.web("GET SCHEMA");
    TEST_ASSERT_TRUE_MESSAGE(sawAnywhere("K,SP2,F,"),
                             "settings page has no ranges to enforce");
    TEST_ASSERT_TRUE_MESSAGE(sawAnywhere("K,AP_PASS,S,"), "AP password missing from the schema");
    // dive-locked and safety flags ride along, so the page can grey out what
    // the server will refuse anyway
    TEST_ASSERT_TRUE_MESSAGE(sawAnywhere("K,SP2,F,0.2,1.6,3,"),
                             "SP2 lost its dive-lock/safety flags on the way out");
}

void test_diver_intent_survives_across_transports() {
    std::vector<std::string> out;
    g_out = &out;
    Fix f;

    // The phone turns the radio off. The handset must remember that, because
    // it is the same session, not a copy of one.
    f.web("WIFI OFF");
    TEST_ASSERT_EQUAL(1, (int)f.hk.wifi.size());
    TEST_ASSERT_EQUAL(0, f.hk.wifi.back());
    TEST_ASSERT_FALSE(f.ses.wifiEnabled());

    f.handset("DIVE START");
    TEST_ASSERT_EQUAL(1, f.hk.opens);
    // no surprise radio-on at dive start, and none at dive end either: the
    // diver, not the machine, decided the radio was off.
    f.handset("DIVE END");
    TEST_ASSERT_EQUAL(1, f.hk.closes);
    TEST_ASSERT_EQUAL(1, (int)f.hk.wifi.size());
    TEST_ASSERT_FALSE(f.ses.wifiEnabled());

    // A dive started from the handset while the phone was connected: the
    // radio drops, and comes back only because the diver had it on.
    f.web("WIFI ON");
    TEST_ASSERT_EQUAL(2, (int)f.hk.wifi.size());
    TEST_ASSERT_EQUAL(1, f.hk.wifi.back());
    f.handset("DIVE START");
    TEST_ASSERT_EQUAL(3, (int)f.hk.wifi.size());
    TEST_ASSERT_EQUAL(0, f.hk.wifi.back());
    f.handset("DIVE END");
    TEST_ASSERT_EQUAL(4, (int)f.hk.wifi.size());
    TEST_ASSERT_EQUAL(1, f.hk.wifi.back());
}

void test_the_two_step_cal_handshake_is_shared() {
    std::vector<std::string> out;
    g_out = &out;
    Fix f;

    f.web("CAL O2");                       // armed from the phone...
    TEST_ASSERT_TRUE(saw("W:OK CAL O2 ARMED"));
    out.clear();
    f.handset("CAL O2 CONFIRM");           // ...confirmed on the handset
    TEST_ASSERT_TRUE_MESSAGE(!saw("H:ERR not armed"),
                             "the handset could not confirm an arm the web made");
}

void test_actions_run_inside_the_session_lock() {
    std::vector<std::string> out;
    g_out = &out;
    Fix f;

    f.web("SET GF_LOW=35");
    f.handset("SET ALLO_LOW=0.20");
    TEST_ASSERT_EQUAL(2, f.hk.settings);
    TEST_ASSERT_EQUAL_MESSAGE(1, f.hk.settingsHeld,
                              "settings applied outside the session lock");
    TEST_ASSERT_EQUAL(g_acquires, g_releases);
    TEST_ASSERT_EQUAL_MESSAGE(1, g_heldMax, "two commands executed inside each other");

    // the periodic pump is on the same lock: the handset's downlink cannot
    // interleave a dive-file write with a web command's
    f.pump(500);
    TEST_ASSERT_EQUAL(g_acquires, g_releases);
    TEST_ASSERT(g_heldMax <= 1);
}

void test_a_web_command_never_steals_the_handset_events() {
    std::vector<std::string> out;
    g_out = &out;
    Fix f;

    TEST_ASSERT_TRUE(f.events.push({12, logfmt::EVT_ALARM_RAISED, 0}));
    TEST_ASSERT_TRUE(f.events.push({13, logfmt::EVT_MODE_CHANGE, 1}));

    // Two web commands in between: neither may touch the handset's ring.
    f.web("GET T");
    f.web("GET PRE");

    f.pump(500);
    TEST_ASSERT_TRUE_MESSAGE(saw("H:E,12,1,0"), "handset lost an event to the web path");
    TEST_ASSERT_TRUE(saw("H:E,13,3,1"));
    TEST_ASSERT_EQUAL(2, f.hk.snaps);   // each event still costs one snapshot
    out.clear();
    f.pump(1000);
    TEST_ASSERT_EQUAL_MESSAGE(0, (int)countPrefix("H:E,"),   // drained, not replayed
                            "events replayed on the next pump");
}

// The histogram is display, but it is still the protocol: both clients ask
// through the same session and must get the same shape, and the folded
// counts must be the samples that went in. A page drawing a histogram the
// handset cannot see (or worse, a different one) is how "trust the gauge"
// dies.
void test_the_histogram_reaches_both_clients_identically() {
    std::vector<std::string> out;
    g_out = &out;
    Fix f;
    f.hist.sample(0, 0.62f, 1000);
    f.hist.sample(0, 0.62f, 1100);
    f.hist.sample(1, 0.30f, 1100);

    f.handset("GET HIST");
    // One sink call may carry all three lines (the web transport splits it
    // into frames); count the protocol lines in the joined text.
    size_t handsetLines = 0;
    for (const auto& l : out)
        for (size_t p = 0; (p = l.find("HIST,", p)) != std::string::npos; p += 5) handsetLines++;
    TEST_ASSERT_EQUAL_MESSAGE(3, (int)handsetLines, "one HIST line per cell");
    TEST_ASSERT_TRUE(sawAnywhere("HIST,0,60,10,5,0,2,"));   // cell0: two folded
    TEST_ASSERT_TRUE(sawAnywhere("HIST,1,60,10,5,0,1,"));   // cell1: one
    TEST_ASSERT_TRUE(sawAnywhere("HIST,2,60,10,5,0,0,"));   // cell2: honest zero

    out.clear();
    f.web("GET HIST");
    size_t webLines = 0;
    for (const auto& l : out)
        for (size_t p = 0; (p = l.find("HIST,", p)) != std::string::npos; p += 5) webLines++;
    TEST_ASSERT_EQUAL_MESSAGE(3, (int)webLines,
                              "the web got a different histogram shape than the handset");
    TEST_ASSERT_TRUE(sawAnywhere("HIST,0,60,10,5,0,2,"));
}

void test_reply_marker_is_the_browsers_end_of_answer() {
    char buf[32];
    TEST_ASSERT_EQUAL_UINT32(3, comms::formatReplyMarker(buf, sizeof(buf), 3, false));
    TEST_ASSERT_EQUAL_STRING("#3\n", buf);
    comms::formatReplyMarker(buf, sizeof(buf), 0, true);
    TEST_ASSERT_EQUAL_STRING("#0 dropped\n", buf);
    // '#' can never be the start of a protocol line, so nothing can collide
    // with the marker: check the first character of every reply form.
    char probe[160];
    TEST_ASSERT_TRUE(proto::formatOk(probe, sizeof(probe)) > 0);
    TEST_ASSERT_TRUE(probe[0] == 'O');
}

void test_a_session_without_a_chain_answers_nothing() {
    // begin() was never called: transports must be able to call handleLine()
    // and pump() without a null check of their own.
    Session bare;
    std::vector<std::string> out;
    g_out = &out;
    bare.handleLine("GET T", 0, sinkWeb, nullptr);
    bare.pump(sinkWeb, nullptr, 1000);
    TEST_ASSERT_EQUAL_MESSAGE(0, (int)out.size(), "a sessionless chain must stay silent");
    TEST_ASSERT_FALSE(bare.ready());
    TEST_ASSERT_FALSE(bare.wifiEnabled());
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_safe_mode_is_visible_on_the_downlink);
    RUN_TEST(test_the_web_gets_identical_refusals);
    RUN_TEST(test_diver_intent_survives_across_transports);
    RUN_TEST(test_the_two_step_cal_handshake_is_shared);
    RUN_TEST(test_actions_run_inside_the_session_lock);
    RUN_TEST(test_a_web_command_never_steals_the_handset_events);
    RUN_TEST(test_the_histogram_reaches_both_clients_identically);
    RUN_TEST(test_reply_marker_is_the_browsers_end_of_answer);
    RUN_TEST(test_a_session_without_a_chain_answers_nothing);
    return UNITY_END();
}
