// Handset library conformance: every expectation below is a line literal the
// FIRMWARE emits (formats pinned by test_proto / test_uartlink / test_session
// from the same code paths) fed through hs::Link. If the controller's wire
// format changes without the handset library knowing, this suite is where it
// surfaces — which is exactly the contract docs/HANDSET_PROTOCOL.md promises
// to a handset built by someone else.
#include <unity.h>

#include <cstring>

#include "../../handset/handset_link.h"

using namespace hs;

namespace {

Link link;

struct Captured {
    int telemetry = 0, deco = 0, events = 0, ok = 0, err = 0;
    int settings = 0, schema = 0, pre = 0, cells = 0, proof = 0, fault = 0;
    int logList = 0, diveHeader = 0, raw = 0, linkDown = 0, linkUp = 0;
    Telemetry t{};
    Deco d{};
    Event e{};
    Setting s{};
    SchemaEntry sc{};
    PreStatus preS{};
    CellsTest ct{};
    Proof pr{};
    Fault fa{};
    LogEntry le{};
    DiveHeader dh{};
    char okNote[64] = {0}, errReason[64] = {0}, rawLine[1200] = {0};
};
Captured cap;

void hTel(const Telemetry& t, void*) { cap.t = t; cap.telemetry++; }
void hDeco(const Deco& d, void*) { cap.d = d; cap.deco++; }
void hEv(const Event& e, void*) { cap.e = e; cap.events++; }
void hOk(const char* note, void*) { cap.ok++;
    if (note) { std::strncpy(cap.okNote, note, 63); } else cap.okNote[0] = 0; }
void hErr(const char* why, void*) { cap.err++; std::strncpy(cap.errReason, why, 63); }
void hSet(const Setting& s, void*) { cap.s = s; cap.settings++; }
void hSc(const SchemaEntry& s, void*) { cap.sc = s; cap.schema++; }
void hPre(const PreStatus& p, void*) { cap.preS = p; cap.pre++; }
void hCells(const CellsTest& c, void*) { cap.ct = c; cap.cells++; }
void hProof(const Proof& p, void*) { cap.pr = p; cap.proof++; }
void hFault(const Fault& f, void*) { cap.fa = f; cap.fault++; }
void hLog(const LogEntry& l, void*) { cap.le = l; cap.logList++; }
void hDive(const DiveHeader& h, void*) { cap.dh = h; cap.diveHeader++; }
void hRaw(const char* l, void*) { std::strncpy(cap.rawLine, l, 1199); cap.raw++; }
void hDown(void*) { cap.linkDown++; }
void hUp(void*) { cap.linkUp++; }

// uplink capture: writer records the command line the library produced
char sent[kTxLineCap];
void writer(const char* line, void*) { std::strncpy(sent, line, kTxLineCap - 1); }

void reset() {
    cap = Captured{};
    Handlers h;
    h.ctx = nullptr;
    h.onTelemetry = hTel; h.onDeco = hDeco; h.onEvent = hEv;
    h.onOk = hOk; h.onErr = hErr; h.onSetting = hSet; h.onSchema = hSc;
    h.onPre = hPre; h.onCells = hCells; h.onProof = hProof; h.onFault = hFault;
    h.onLogList = hLog; h.onDiveHeader = hDive; h.onLine = hRaw;
    h.onLinkDown = hDown; h.onLinkUp = hUp;
    link.begin(writer, nullptr, h);
    sent[0] = 0;
}

}  // namespace

void setUp() { reset(); }
void tearDown() {}

// exact literal from proto::formatTelemetry
void test_parse_T_line() {
    link.feedText("T,18.35,0.712,0.708,0.715,0.711,0.700,AUTO,FLAGS0x10,12.41\n");
    TEST_ASSERT_EQUAL_INT(1, cap.telemetry);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 18.35f, cap.t.depthM);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.712f, cap.t.votedBar);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.711f, cap.t.cells[2]);
    TEST_ASSERT_EQUAL_INT(1, cap.t.mode);           // AUTO
    TEST_ASSERT_EQUAL_UINT8(0x10, cap.t.flags);     // battery
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 12.41f, cap.t.battV);
}

void test_parse_D_line_with_stops_and_unknown_ndl() {
    // D,<ndl>,<total>,<ceiling>[,<depth>|<min>...],<cns>
    link.feedText("D,17,24.5,6.00,6|4.5,3|20.0,12.3\n");
    TEST_ASSERT_EQUAL_INT(1, cap.deco);
    TEST_ASSERT_EQUAL_INT(17, cap.d.ndlMin);
    TEST_ASSERT_EQUAL_INT(2, cap.d.stopCount);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 6.0f, cap.d.stops[0].depthM);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 20.0f, cap.d.stops[1].minutes);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 12.3f, cap.d.cnsPct);

    reset();
    link.feedText("D,-1,0.0,0.00,0.0\n");           // no vote: NDL unknown
    TEST_ASSERT_EQUAL_INT(-1, cap.d.ndlMin);
    TEST_ASSERT_EQUAL_INT(0, cap.d.stopCount);

    reset();
    link.feedText("D,480,0.0,0.00,0.0\n");          // unlimited
    TEST_ASSERT_EQUAL_INT(480, cap.d.ndlMin);
}

void test_parse_status_lines() {
    link.feedText("PRE,2,2,1,0,2,0\n");             // warm/air pass, solenoid running
    TEST_ASSERT_EQUAL_INT(2, cap.preS.step[0]);
    TEST_ASSERT_EQUAL_INT(1, cap.preS.step[2]);
    TEST_ASSERT_FALSE(cap.preS.passed);

    link.feedText("CELLS,2,007,000,1\n");           // PULSING, masks are %03o bitmasks
    TEST_ASSERT_EQUAL_INT(2, cap.ct.state);
    TEST_ASSERT_EQUAL_HEX8(0b111, cap.ct.passMask);
    TEST_ASSERT_TRUE(cap.ct.requested);

    link.feedText("PROOF,4,004,001,3\n");           // PASS; cell2 unproven, cell0 excluded
    TEST_ASSERT_EQUAL_INT(4, cap.pr.state);
    TEST_ASSERT_EQUAL_HEX8(0b100, cap.pr.unprovenMask);
    TEST_ASSERT_EQUAL_HEX8(0b001, cap.pr.exclusionMask);

    link.feedText("FAULT,1,0,7.5\n");
    TEST_ASSERT_TRUE(cap.fa.stuckOpenLatched);
    TEST_ASSERT_FALSE(cap.fa.outputFuseTripped);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 7.5f, cap.fa.solDutyPct);
}

void test_parse_replies_and_settings() {
    link.feedText("OK\nOK SAFETY\nERR dive locked\n");
    TEST_ASSERT_EQUAL_INT(2, cap.ok);
    TEST_ASSERT_EQUAL_STRING("SAFETY", cap.okNote);
    TEST_ASSERT_EQUAL_INT(1, cap.err);
    TEST_ASSERT_EQUAL_STRING("dive locked", cap.errReason);

    link.feedText("GF_LOW=40\n");
    TEST_ASSERT_EQUAL_INT(1, cap.settings);
    TEST_ASSERT_EQUAL_STRING("GF_LOW", cap.s.key);
    TEST_ASSERT_EQUAL_STRING("40", cap.s.value);
}

void test_parse_schema_log_dive() {
    link.feedText("K,GF_LOW,U,10,95,2,30\n");
    TEST_ASSERT_EQUAL_INT(1, cap.schema);
    TEST_ASSERT_EQUAL_STRING("GF_LOW", cap.sc.key);
    TEST_ASSERT_EQUAL_CHAR('U', cap.sc.type);
    TEST_ASSERT_EQUAL_UINT32(2, cap.sc.flags);       // FL_SAFETY

    link.feedText("K,AP_PASS,S,8,31,1,diveonly\n");
    TEST_ASSERT_EQUAL_CHAR('S', cap.sc.type);
    TEST_ASSERT_EQUAL_STRING("diveonly", cap.sc.defStr);

    link.feedText("L,3,1764123456\n");
    TEST_ASSERT_EQUAL_INT(3, cap.le.slot);

    link.feedText("DIVE,3,1764123456,1\n");
    TEST_ASSERT_EQUAL_INT(3, cap.dh.slot);

    // dive CSV rows and HIST lines land in onLine (documented, not structured)
    link.feedText("S,120,18.35,0.712,0.708,0.715,0.711,0.700,0x0010,3,6.00,17,24,4,12.41\n");
    TEST_ASSERT_EQUAL_INT(1, cap.raw);
}

void test_event_lines() {
    link.feedText("E,125,1,0\n");                   // t=125 s, alarm raised, bit 0 (low ppO2)
    TEST_ASSERT_EQUAL_INT(1, cap.events);
    TEST_ASSERT_EQUAL_UINT32(125, cap.e.tSec);
    TEST_ASSERT_EQUAL_UINT16(1, cap.e.id);
    TEST_ASSERT_EQUAL_UINT16(0, cap.e.value);
}

void test_unknown_lines_counted_not_fatal() {
    link.feedText("FUTURE,x,y\n,,,\n");
    TEST_ASSERT_EQUAL_INT(2, cap.raw);
    TEST_ASSERT_EQUAL_INT(0, cap.err);
    // a malformed T, falls through to raw, does not crash or fabricate
    reset();
    link.feedText("T,garbage\n");
    TEST_ASSERT_EQUAL_INT(0, cap.telemetry);
    TEST_ASSERT_EQUAL_INT(1, cap.raw);
}

void test_line_assembly_crlf_and_overflow() {
    link.feed("OK\r", 3);
    link.feed("XTRA\n", 5);                    // stray: assembled with next? no: \r ended OK
    TEST_ASSERT_EQUAL_INT(1, cap.ok);          // "OK" came out at \r
    // \r\n pair produces one line, not an empty phantom:
    reset();
    const char two[] = {'O', 'K', '\r', '\n'};
    link.feed(two, 4);
    TEST_ASSERT_EQUAL_INT(1, cap.ok);
    // oversized line dropped whole, recovery works:
    reset();
    char big[2 * kRxLineCap];
    std::memset(big, 'A', sizeof(big));
    big[sizeof(big) - 1] = '\0';
    link.feedText(big);
    link.feedText("\nOK\n");
    TEST_ASSERT_EQUAL_UINT32(1, link.overflows());
    TEST_ASSERT_EQUAL_INT(1, cap.ok);
}

void test_command_builders_exact_strings() {
    TEST_ASSERT_TRUE(link.getT());      TEST_ASSERT_EQUAL_STRING("GET T", sent);
    TEST_ASSERT_TRUE(link.getAll());    TEST_ASSERT_EQUAL_STRING("GET *", sent);
    TEST_ASSERT_TRUE(link.set("GF_LOW", "40"));  TEST_ASSERT_EQUAL_STRING("SET GF_LOW=40", sent);
    TEST_ASSERT_TRUE(link.set("AP_SSID", "has space"));
    TEST_ASSERT_EQUAL_STRING("SET AP_SSID=has space", sent);
    TEST_ASSERT_TRUE(link.calO2Confirm()); TEST_ASSERT_EQUAL_STRING("CAL O2 CONFIRM", sent);
    TEST_ASSERT_TRUE(link.cellproofOverride()); TEST_ASSERT_EQUAL_STRING("CELLPROOF OVERRIDE", sent);
    TEST_ASSERT_TRUE(link.diveStart()); TEST_ASSERT_EQUAL_STRING("DIVE START", sent);
    TEST_ASSERT_TRUE(link.logExport(7)); TEST_ASSERT_EQUAL_STRING("LOG EXPORT 7", sent);
    // over-long command refused at the library, never half-sent on the wire
    char key[70]; std::memset(key, 'K', 61); key[61] = 0;   // "SET " + 61 + "=1" = 67 > 64
    sent[0] = 0;
    TEST_ASSERT_FALSE(link.set(key, "1"));
    TEST_ASSERT_EQUAL_STRING("", sent);
}

void test_link_health_timeouts() {
    uint32_t now = 0;
    link.tick(now);                              // silence: not up yet, no event
    TEST_ASSERT_FALSE(link.linkUp());
    TEST_ASSERT_EQUAL_INT(0, cap.linkUp);

    link.feedText("T,1.00,0.210,0.210,0.210,0.210,0.400,STANDBY,FLAGS0x00,12.00\n");
    now = 1000;
    link.tick(now);
    TEST_ASSERT_TRUE(link.linkUp());
    TEST_ASSERT_EQUAL_INT(1, cap.linkUp);

    now = 3500;                                  // 2.5 s < 3 s: still up
    link.tick(now);
    TEST_ASSERT_TRUE(link.linkUp());

    now = 4501;                                  // > timeout of silence
    link.tick(now);
    TEST_ASSERT_FALSE(link.linkUp());
    TEST_ASSERT_EQUAL_INT(1, cap.linkDown);

    link.feedText("OK\n");                       // back
    now = 5000;
    link.tick(now);
    TEST_ASSERT_TRUE(link.linkUp());
    TEST_ASSERT_EQUAL_INT(2, cap.linkUp);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_parse_T_line);
    RUN_TEST(test_parse_D_line_with_stops_and_unknown_ndl);
    RUN_TEST(test_parse_status_lines);
    RUN_TEST(test_parse_replies_and_settings);
    RUN_TEST(test_parse_schema_log_dive);
    RUN_TEST(test_event_lines);
    RUN_TEST(test_unknown_lines_counted_not_fatal);
    RUN_TEST(test_line_assembly_crlf_and_overflow);
    RUN_TEST(test_command_builders_exact_strings);
    RUN_TEST(test_link_health_timeouts);
    return UNITY_END();
}
