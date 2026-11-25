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
    int logList = 0, diveHeader = 0, clock = 0, calAir = 0, depthSrc = 0, wifiState = 0,
        raw = 0, linkDown = 0, linkUp = 0;
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
    Clock ck{};
    CalAir ca{};
    DepthSource ds{};
    WifiState wf{};
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
void hClock(const Clock& c, void*) { cap.ck = c; cap.clock++; }
void hCalAir(const CalAir& c, void*) { cap.ca = c; cap.calAir++; }
void hDepthSource(const DepthSource& d, void*) { cap.ds = d; cap.depthSrc++; }
void hWifiState(const WifiState& w, void*) { cap.wf = w; cap.wifiState++; }
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
    h.onLogList = hLog; h.onDiveHeader = hDive; h.onClock = hClock;
    h.onCalAir = hCalAir; h.onLine = hRaw;
    h.onDepthSource = hDepthSource;
    h.onWifiState = hWifiState;
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

    // current firmware literals (captured off the bench link, 2026-11-24)
    link.feedText("L,7,1795517641,27\n");
    TEST_ASSERT_EQUAL_INT(7, cap.le.slot);
    TEST_ASSERT_EQUAL_UINT32(27u, cap.le.seq);

    link.feedText("DIVE,7,1795517641,3,3,2026-11-24T11:47:21Z\n");
    TEST_ASSERT_EQUAL_INT(7, cap.dh.slot);
    TEST_ASSERT_EQUAL_INT(3, cap.dh.version);
    TEST_ASSERT_EQUAL_INT(3, cap.dh.rowFormat);
    TEST_ASSERT_EQUAL_STRING("2026-11-24T11:47:21Z", cap.dh.iso);

    // dive CSV rows and HIST lines land in onLine (documented, not structured)
    link.feedText("S,120,18.35,0.712,0.708,0.715,0.711,0.700,0x0010,3,6.00,17,24,4,12.41\n");
    TEST_ASSERT_EQUAL_INT(1, cap.raw);
}

// The clock is protocol-visible and the client package had not been carrying it
// (2026-11-24): TIME SET / GET TIME / TIME,. Literals below are what the board
// actually answered on the bench link, not what the spec's grammar suggests.
void test_parse_clock_line() {
    link.feedText("TIME,1795518150,1,509\n");        // set by a command, 509 s ago
    TEST_ASSERT_EQUAL_INT(1, cap.clock);
    TEST_ASSERT_EQUAL_UINT32(1795518150u, cap.ck.epoch);
    TEST_ASSERT_EQUAL_UINT8(1, cap.ck.source);
    TEST_ASSERT_EQUAL_UINT32(509u, cap.ck.ageS);

    // The honest "no clock" answer: epoch 0, NOT a 1970 date. A handset that
    // renders this as 01/01/1970 is lying to the diver about their own log.
    reset();
    link.feedText("TIME,0,0,0\n");
    TEST_ASSERT_EQUAL_INT(1, cap.clock);
    TEST_ASSERT_EQUAL_UINT32(0u, cap.ck.epoch);
    TEST_ASSERT_EQUAL_UINT8(0, cap.ck.source);

    // carried across a reset the firmware performed on itself
    reset();
    link.feedText("TIME,1795517641,2,86400\n");
    TEST_ASSERT_EQUAL_UINT8(2, cap.ck.source);
    TEST_ASSERT_EQUAL_UINT32(86400u, cap.ck.ageS);

    // `TIME SET` answers `OK TIME`: the note matters, "TIME" is how the handset
    // knows the clock write is the OK it was waiting for.
    reset();
    link.feedText("OK TIME\n");
    TEST_ASSERT_EQUAL_INT(1, cap.ok);
    TEST_ASSERT_EQUAL_STRING("TIME", cap.okNote);

    // and the refusal, which a handset must be able to show
    reset();
    link.feedText("ERR epoch out of range\n");
    TEST_ASSERT_EQUAL_STRING("epoch out of range", cap.errReason);
}

// `L,` gained a fourth field because ten dives that all carry epoch 0 cannot be
// ordered; the library must read it AND keep parsing an older controller's
// three-field line (append-only, spec §7).
void test_log_list_sequence_and_legacy_shape() {
    link.feedText("L,7,1795517641,27\n");            // dated, newest first
    TEST_ASSERT_EQUAL_INT(7, cap.le.slot);
    TEST_ASSERT_EQUAL_UINT32(1795517641u, cap.le.epoch);
    TEST_ASSERT_EQUAL_UINT32(27u, cap.le.seq);

    link.feedText("L,6,0,26\n");                     // undated dive, still ordered
    TEST_ASSERT_EQUAL_UINT32(0u, cap.le.epoch);
    TEST_ASSERT_EQUAL_UINT32(26u, cap.le.seq);

    reset();
    link.feedText("L,3,1764123456\n");               // pre-sequence controller
    TEST_ASSERT_EQUAL_INT(1, cap.logList);
    TEST_ASSERT_EQUAL_INT(3, cap.le.slot);
    TEST_ASSERT_EQUAL_UINT32(1764123456u, cap.le.epoch);
    TEST_ASSERT_EQUAL_UINT32(0u, cap.le.seq);        // 0 = unknown, like the firmware
}

// The exported dive's identity row: version, row format and the ISO date the
// v3 row appends. `clock-unset` must survive verbatim — it is the difference
// between "no date" and "31 December 1969".
void test_dive_header_dated_and_legacy() {
    link.feedText("DIVE,10,1795516836,3,3,2026-11-24T10:40:36Z\n");
    TEST_ASSERT_EQUAL_INT(10, cap.dh.slot);
    TEST_ASSERT_EQUAL_UINT32(1795516836u, cap.dh.startEpoch);
    TEST_ASSERT_EQUAL_INT(3, cap.dh.version);
    TEST_ASSERT_EQUAL_INT(3, cap.dh.rowFormat);
    TEST_ASSERT_EQUAL_STRING("2026-11-24T10:40:36Z", cap.dh.iso);

    reset();
    link.feedText("DIVE,6,0,3,3,clock-unset\n");
    TEST_ASSERT_EQUAL_UINT32(0u, cap.dh.startEpoch);
    TEST_ASSERT_EQUAL_STRING("clock-unset", cap.dh.iso);

    reset();
    link.feedText("DIVE,3,1764123456,1\n");          // older row: no date field
    TEST_ASSERT_EQUAL_INT(1, cap.diveHeader);
    TEST_ASSERT_EQUAL_INT(1, cap.dh.version);
    TEST_ASSERT_EQUAL_INT(0, cap.dh.rowFormat);      // unknown, not "0"
    TEST_ASSERT_EQUAL_STRING("", cap.dh.iso);
}

// `CALAIR,` — the air calibration's verdict. Every literal below was captured on
// the bench link 2026-11-24, in the order the board actually sent them: the empty
// answer, the warm-up refusal, progress, and a committed calibration with its raw
// counts. They matter because `CAL AIR`'s own reply is only a receipt: `OK CAL AIR`
// looks identical whether the cells were sampled, refused or failed, so a handset
// that shows "calibrated" from that reply is lying, and this record is the only
// thing that lets it tell.
void test_parse_calair_verdicts() {
    link.feedText("CALAIR,0,0,0,0,0,0,0,0\n");     // nothing asked since power-up
    TEST_ASSERT_EQUAL_INT(1, cap.calAir);
    TEST_ASSERT_EQUAL_INT(CalAir::NONE, cap.ca.code);
    TEST_ASSERT_EQUAL_STRING("no calibration since power-up", calAirText(cap.ca.code));

    // 1 s after a surface `CAL AIR` inside the warm-up gate: refused, 97 s to go.
    link.feedText("CALAIR,3,1,0,0,0,0,0,97\n");
    TEST_ASSERT_EQUAL_INT(CalAir::WARMUP, cap.ca.code);
    TEST_ASSERT_EQUAL_UINT32(1u, cap.ca.ageS);
    TEST_ASSERT_EQUAL_UINT32(97u, cap.ca.warmupLeftS);
    TEST_ASSERT_EQUAL_STRING("refused: cells still warming up", calAirText(cap.ca.code));

    // Progress, while it samples. `samples` is the progress bar.
    link.feedText("CALAIR,1,0,3,0,0,0,0,0\n");
    TEST_ASSERT_EQUAL_INT(CalAir::RUNNING, cap.ca.code);
    TEST_ASSERT_EQUAL_INT(3, cap.ca.samples);

    // Committed, 2 s ago: 12 samples, worst spread 21 counts, raw averages. Those
    // numbers are how an installer tells a dead cell from a fouled one.
    link.feedText("CALAIR,2,2,12,21,1514,1519,1528,0\n");
    TEST_ASSERT_EQUAL_INT(CalAir::OK, cap.ca.code);
    TEST_ASSERT_EQUAL_INT(12, cap.ca.samples);
    TEST_ASSERT_EQUAL_UINT32(2u, cap.ca.ageS);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 21.0f, cap.ca.spreadWorst);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1514.0f, cap.ca.rawAvg[0]);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1519.0f, cap.ca.rawAvg[1]);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1528.0f, cap.ca.rawAvg[2]);
    TEST_ASSERT_EQUAL_STRING("calibrated", calAirText(cap.ca.code));

    // The same verdict, arriving unprompted: the `E,` row's value is the SAME
    // enumeration, so a client never translates between two numberings.
    reset();
    link.feedText("E,18,7,3\n");
    TEST_ASSERT_EQUAL_INT(1, cap.events);
    TEST_ASSERT_EQUAL_UINT16(7, cap.e.id);
    TEST_ASSERT_EQUAL_UINT16(CalAir::WARMUP, cap.e.value);
    reset();
    link.feedText("E,129,7,2\n");
    TEST_ASSERT_EQUAL_UINT16(CalAir::OK, cap.e.value);

    // Append-only both ways: fewer fields still parses (the code is what a client
    // acts on), and an unknown future code is "a verdict with no name", never a
    // parse failure that would throw the record away.
    reset();
    link.feedText("CALAIR,2\n");
    TEST_ASSERT_EQUAL_INT(1, cap.calAir);
    TEST_ASSERT_EQUAL_INT(CalAir::OK, cap.ca.code);
    TEST_ASSERT_EQUAL_UINT32(0u, cap.ca.ageS);
    reset();
    link.feedText("CALAIR,9,5,12,3,100,100,100,0\n");
    TEST_ASSERT_EQUAL_INT(9, cap.ca.code);
    TEST_ASSERT_EQUAL_STRING("unknown verdict", calAirText(cap.ca.code));

    // `CALAIR,` must not fall through to the KEY=value fallback (it has no '='),
    // which is how `TIME,` was being counted as an unknown line until 2026-11-24.
    TEST_ASSERT_EQUAL_MESSAGE(0, cap.raw, "the CALAIR line was treated as unknown");
}

// The uplink half of the clock: exact command strings, and the epoch window
// enforced before the wire — the same window `wallclock::sane()` applies to the
// bytes if they arrive.
void test_clock_commands_exact_strings() {
    TEST_ASSERT_TRUE(link.getTime());
    TEST_ASSERT_EQUAL_STRING("GET TIME", sent);

    TEST_ASSERT_TRUE(link.getCalAir());
    TEST_ASSERT_EQUAL_STRING("GET CALAIR", sent);

    TEST_ASSERT_TRUE(link.timeSet(1795517641u));
    TEST_ASSERT_EQUAL_STRING("TIME SET 1795517641", sent);

    // both bounds are accepted, just outside them are not
    sent[0] = 0;
    TEST_ASSERT_TRUE(link.timeSet(kMinEpoch));
    TEST_ASSERT_EQUAL_STRING("TIME SET 946684800", sent);
    sent[0] = 0;
    TEST_ASSERT_TRUE(link.timeSet(kMaxEpoch));
    TEST_ASSERT_EQUAL_STRING("TIME SET 4102444800", sent);

    // refused client-side: nothing on the wire. A handset whose own clock has
    // gone to soup must not be able to date the dive log with it.
    const uint32_t bad[] = {0u, 7u, kMinEpoch - 1u, kMaxEpoch + 1u, 0xFFFFFFFFu};
    for (uint32_t e : bad) {
        sent[0] = 0;
        TEST_ASSERT_FALSE_MESSAGE(link.timeSet(e), "implausible epoch must not send");
        TEST_ASSERT_EQUAL_STRING_MESSAGE("", sent, "bytes reached the wire");
    }
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

// Two of the three calibration commands are refused by the controller because
// nothing on the board performs them. The builders stay (the grammar is unchanged,
// so a client keeps compiling when the work lands) and the refusal string is part
// of the contract, so it is pinned here: a handset that never sees `OK` for these
// cannot accidentally draw a green tick from one.
void test_unimplemented_calibrations_refuse_with_one_string() {
    TEST_ASSERT_TRUE(link.calDepth());
    TEST_ASSERT_EQUAL_STRING("CAL DEPTH", sent);
    link.feedText("ERR not implemented\n");
    TEST_ASSERT_EQUAL_STRING("not implemented", cap.errReason);

    reset();
    TEST_ASSERT_TRUE(link.calO2Arm());
    TEST_ASSERT_EQUAL_STRING("CAL O2", sent);
    link.feedText("ERR not implemented\n");
    TEST_ASSERT_EQUAL_STRING("not implemented", cap.errReason);

    reset();
    TEST_ASSERT_TRUE(link.calO2Confirm());
    TEST_ASSERT_EQUAL_STRING("CAL O2 CONFIRM", sent);
    link.feedText("ERR not implemented\n");
    TEST_ASSERT_EQUAL_STRING("not implemented", cap.errReason);

    // Asked twice, it is refused twice: there is no arm for a handset to manage,
    // so no client-side "armed" state can be right.
    reset();
    link.feedText("ERR not implemented\n");
    link.feedText("ERR not implemented\n");
    TEST_ASSERT_EQUAL(2, cap.err);
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

void test_depth_source_line_and_wording() {
    // Which chip is producing the metres, and whether the controller is using it.
    // The parse is not the interesting part; the two things that go wrong are (a)
    // the line falling through to onLine because dispatch did not claim it, and
    // (b) a client reading `locked` and concluding the depth is live.
    link.feedText("DS,1,1,1,4,LPS28DFW @0x5D\n");
    TEST_ASSERT_EQUAL_INT_MESSAGE(1, cap.depthSrc, "DS, line never reached onDepthSource");
    TEST_ASSERT_EQUAL_INT_MESSAGE(0, cap.raw,
                                  "DS, fell through to onLine: dispatch does not claim it");
    TEST_ASSERT_TRUE(cap.ds.locked); TEST_ASSERT_TRUE(cap.ds.used); TEST_ASSERT_TRUE(cap.ds.healthy);
    TEST_ASSERT_EQUAL_UINT32(4u, cap.ds.ageS);
    TEST_ASSERT_EQUAL_STRING("LPS28DFW @0x5D", cap.ds.identity);

    // The field a handset must act on: locked but not used means the metres on
    // `T,` are held, not measured. Same name, same slot, different meaning.
    link.feedText("DS,1,0,1,37,MS5837-30BA @0x76\n");
    TEST_ASSERT_TRUE(cap.ds.locked);
    TEST_ASSERT_FALSE_MESSAGE(cap.ds.used, "a held depth read as a live one");
    TEST_ASSERT_EQUAL_UINT32(37u, cap.ds.ageS);

    // No source is a complete answer, not a parse failure.
    link.feedText("DS,0,0,0,0,none\n");
    TEST_ASSERT_FALSE(cap.ds.locked);
    TEST_ASSERT_EQUAL_STRING("none", cap.ds.identity);

    // Append-only: a controller that grows the line on the right still parses,
    // because identity is the last field and never holds a comma.
    const int appendedBefore = cap.depthSrc;
    link.feedText("DS,1,1,1,0,LPS28DFW @0x5D,99\n");
    TEST_ASSERT_EQUAL_INT_MESSAGE(appendedBefore + 1, cap.depthSrc,
                                  "a DS, line with a field appended on the right was refused");
    TEST_ASSERT_EQUAL_STRING("LPS28DFW @0x5D", cap.ds.identity);

    // A truncated line is refused, not guessed: "1,1,1" with no age and no name
    // would otherwise become a source with an empty name and an age of 0.
    const int before = cap.depthSrc, rawBefore = cap.raw;
    link.feedText("DS,1,1,1\n");
    TEST_ASSERT_EQUAL_INT_MESSAGE(before, cap.depthSrc, "a truncated DS, line was accepted");
    TEST_ASSERT_EQUAL_INT_MESSAGE(rawBefore + 1, cap.raw,
                                  "a rejected DS, line should reach onLine, not vanish");

    // An empty identity field still means "none" rather than a blank on a display.
    link.feedText("DS,1,1,1,0,\n");
    TEST_ASSERT_EQUAL_STRING("none", cap.ds.identity);

    // A longer name than the client's field is truncated, not run over.
    link.feedText("DS,1,1,1,0,CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC\n");
    TEST_ASSERT_EQUAL_INT_MESSAGE(31, static_cast<int>(std::strlen(cap.ds.identity)),
                                  "an over-long identity was not truncated into the client's field");

    // The wording, because "1,0,1" is not something a diver reads.
    DepthSource d{};
    char txt[96];
    d.locked = d.used = d.healthy = true; d.ageS = 0;
    std::snprintf(d.identity, sizeof(d.identity), "LPS28DFW @0x5D");
    TEST_ASSERT_GREATER_THAN_INT(0, depthSourceText(d, txt, sizeof(txt)));
    TEST_ASSERT_NOT_NULL_MESSAGE(std::strstr(txt, "LPS28DFW @0x5D"), txt);

    d.used = false; d.ageS = 12;
    TEST_ASSERT_NOT_NULL_MESSAGE(std::strstr(depthSourceText(d, txt, sizeof(txt)) ? txt : "", "HELD"),
                                 "a held depth is not described as held");
    TEST_ASSERT_NOT_NULL_MESSAGE(std::strstr(txt, "12 s"), txt);

    d.locked = false;
    TEST_ASSERT_NOT_NULL_MESSAGE(std::strstr(depthSourceText(d, txt, sizeof(txt)) ? txt : "",
                                             "no depth sensor"), txt);

    // Too small a buffer: empty string, zero length. A half sentence on an OLED
    // ("depth HELD: no reading fr") is worse than nothing.
    char tiny[8];
    TEST_ASSERT_EQUAL_INT_MESSAGE(0, depthSourceText(d, tiny, sizeof(tiny)),
                                  "depthSourceText wrote a partial sentence");
    TEST_ASSERT_EQUAL_CHAR('\0', tiny[0]);
}

void test_wifi_state_line_words_and_append_only() {
    // The radio is policy-driven now (spec §4): a client that wants "can I
    // connect" needs this line, and the ways it goes wrong in a client mirror
    // the DS ones — the line falling through unclaimed, and a field read as
    // something it is not.
    link.feedText("WIFI,1,0,597\n");
    TEST_ASSERT_EQUAL_INT_MESSAGE(1, cap.wifiState, "WIFI, line never reached onWifiState");
    TEST_ASSERT_EQUAL_INT_MESSAGE(0, cap.raw,
                                  "WIFI, fell through to onLine: dispatch does not claim it");
    TEST_ASSERT_TRUE(cap.wf.on);
    TEST_ASSERT_EQUAL_UINT8(WifiState::HOLD_NONE, cap.wf.heldBy);
    TEST_ASSERT_EQUAL_UINT32(597u, cap.wf.idleLeftS);

    // The command string, byte for byte: a controller that never sees this
    // exact line never answers it.
    link.getWifiState();
    TEST_ASSERT_EQUAL_STRING_MESSAGE("GET WIFI", sent, "getWifiState sent the wrong line");

    // on=0 after a WIFI ON at depth is the HONEST answer, not a failure to
    // parse or a contradiction: intent on, held by depth. The client acts on
    // `on`, not on what it last typed.
    link.feedText("WIFI,0,3,0\n");
    TEST_ASSERT_FALSE_MESSAGE(cap.wf.on, "a held radio read as up");
    TEST_ASSERT_EQUAL_UINT8(WifiState::HOLD_DEPTH, cap.wf.heldBy);

    // Namespace discipline: the settings keys WIFI_IDLE_MIN etc. are KEY=value
    // answers, and the comma in WIFI, is what keeps them apart. Feed one of
    // each; each must reach its own handler.
    link.feedText("WIFI_IDLE_MIN=10\n");
    TEST_ASSERT_EQUAL_INT_MESSAGE(1, cap.settings,
                                  "WIFI, dispatch swallowed the WIFI_* settings answer");
    TEST_ASSERT_EQUAL_STRING("WIFI_IDLE_MIN", cap.s.key);

    // Append-only: a cause this library predates (5) and a field appended on
    // the right both parse — the protocol grows rightward and the client must
    // grow with it rather than refuse the line.
    link.feedText("WIFI,0,5,0,42\n");
    TEST_ASSERT_EQUAL_UINT8(5, cap.wf.heldBy);
    TEST_ASSERT_FALSE(cap.wf.on);

    // The stations field (appended 2026-11-26, after a bench run found the
    // countdown frozen with nobody able to see WHY — the bench PC's own wlan0
    // was the station). Present parses; absent means "that controller never
    // said", which is not the same number as "zero stations".
    link.feedText("WIFI,1,0,600,2\n");
    TEST_ASSERT_EQUAL_INT_MESSAGE(2, cap.wf.stations, "stations field not parsed");
    link.feedText("WIFI,1,0,600\n");
    TEST_ASSERT_EQUAL_INT_MESSAGE(-1, cap.wf.stations,
        "a controller that never sent the field must not read as zero stations");

    // A truncated line is refused, not guessed: "WIFI,1,0" with no countdown
    // would otherwise silently mean "idle-off in 0 s" = never.
    const int before = cap.wifiState, rawBefore = cap.raw;
    link.feedText("WIFI,1,0\n");
    TEST_ASSERT_EQUAL_INT_MESSAGE(before, cap.wifiState, "a truncated WIFI, line was accepted");
    TEST_ASSERT_EQUAL_INT_MESSAGE(rawBefore + 1, cap.raw,
                                  "a rejected WIFI, line should reach onLine, not vanish");

    // Wording: sentences a diver can act on, and unknown causes printed as
    // numbers rather than dressed in a stale word.
    WifiState w{};
    char txt[120];
    w.on = true; w.heldBy = WifiState::HOLD_NONE; w.idleLeftS = 597;
    TEST_ASSERT_GREATER_THAN_INT(0, wifiStateText(w, txt, sizeof(txt)));
    TEST_ASSERT_NOT_NULL_MESSAGE(std::strstr(txt, "auto-off"), txt);
    w.idleLeftS = 0;
    TEST_ASSERT_GREATER_THAN_INT(0, wifiStateText(w, txt, sizeof(txt)));
    TEST_ASSERT_NOT_NULL_MESSAGE(std::strstr(txt, "never"), txt);
    w.on = false; w.heldBy = WifiState::HOLD_DIVER;
    TEST_ASSERT_GREATER_THAN_INT(0, wifiStateText(w, txt, sizeof(txt)));
    TEST_ASSERT_NOT_NULL_MESSAGE(std::strstr(txt, "diver"), txt);
    w.heldBy = WifiState::HOLD_DIVE;
    TEST_ASSERT_GREATER_THAN_INT(0, wifiStateText(w, txt, sizeof(txt)));
    TEST_ASSERT_NOT_NULL_MESSAGE(std::strstr(txt, "dive"), txt);
    w.heldBy = WifiState::HOLD_DEPTH;
    TEST_ASSERT_GREATER_THAN_INT(0, wifiStateText(w, txt, sizeof(txt)));
    TEST_ASSERT_NOT_NULL_MESSAGE(std::strstr(txt, "depth"), txt);
    w.heldBy = WifiState::HOLD_IDLE;
    TEST_ASSERT_GREATER_THAN_INT(0, wifiStateText(w, txt, sizeof(txt)));
    TEST_ASSERT_NOT_NULL_MESSAGE(std::strstr(txt, "idle"), txt);
    w.heldBy = 9;   // a cause from a future firmware
    TEST_ASSERT_GREATER_THAN_INT(0, wifiStateText(w, txt, sizeof(txt)));
    TEST_ASSERT_NOT_NULL_MESSAGE(std::strstr(txt, "9"), txt);
    // on=0 with reason "none" is a contradiction; the wording must notice.
    w.heldBy = WifiState::HOLD_NONE;
    TEST_ASSERT_GREATER_THAN_INT(0, wifiStateText(w, txt, sizeof(txt)));
    TEST_ASSERT_NOT_NULL_MESSAGE(std::strstr(txt, "none"), txt);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_parse_T_line);
    RUN_TEST(test_parse_D_line_with_stops_and_unknown_ndl);
    RUN_TEST(test_parse_status_lines);
    RUN_TEST(test_parse_replies_and_settings);
    RUN_TEST(test_parse_schema_log_dive);
    RUN_TEST(test_parse_clock_line);
    RUN_TEST(test_log_list_sequence_and_legacy_shape);
    RUN_TEST(test_dive_header_dated_and_legacy);
    RUN_TEST(test_clock_commands_exact_strings);
    RUN_TEST(test_parse_calair_verdicts);
    RUN_TEST(test_unimplemented_calibrations_refuse_with_one_string);
    RUN_TEST(test_event_lines);
    RUN_TEST(test_unknown_lines_counted_not_fatal);
    RUN_TEST(test_line_assembly_crlf_and_overflow);
    RUN_TEST(test_command_builders_exact_strings);
    RUN_TEST(test_link_health_timeouts);
    RUN_TEST(test_depth_source_line_and_wording);
    RUN_TEST(test_wifi_state_line_words_and_append_only);
    return UNITY_END();
}
