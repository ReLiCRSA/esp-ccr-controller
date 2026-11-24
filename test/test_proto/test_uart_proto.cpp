#include <unity.h>

#include <cstring>

#include "comms/uart_proto.h"

using namespace proto;

void setUp() {}
void tearDown() {}

static Command parsed(const char* line, bool expectOk = true) {
    Command c;
    const char* err = nullptr;
    const bool ok = parseCommand(line, c, &err);
    TEST_ASSERT_EQUAL_MESSAGE(expectOk, ok, expectOk ? "should parse" : "should reject");
    return c;
}

void test_simple_verbs() {
    TEST_ASSERT_EQUAL_UINT8(Kind::CHECK, parsed("CHECK").kind);
    TEST_ASSERT_EQUAL_UINT8(Kind::AUTO, parsed("AUTO").kind);
    TEST_ASSERT_EQUAL_UINT8(Kind::ACK, parsed("ACK\r\n").kind);   // CRLF tolerated
    TEST_ASSERT_EQUAL_UINT8(Kind::PULSE, parsed("PULSE ").kind);  // trailing space
}

void test_get_and_set() {
    auto c = parsed("GET *");
    TEST_ASSERT_EQUAL_UINT8(Kind::GET, c.kind);
    TEST_ASSERT_EQUAL_STRING("*", c.key);

    c = parsed("GET SP2");
    TEST_ASSERT_EQUAL_STRING("SP2", c.key);

    c = parsed("SET SP2=0.70");
    TEST_ASSERT_EQUAL_UINT8(Kind::SET, c.kind);
    TEST_ASSERT_EQUAL_STRING("SP2", c.key);
    TEST_ASSERT_EQUAL_STRING("0.70", c.value);

    // values with spaces (SSID / password)
    c = parsed("SET AP_PASS=hunt er 2026");
    TEST_ASSERT_EQUAL_STRING("AP_PASS", c.key);
    TEST_ASSERT_EQUAL_STRING("hunt er 2026", c.value);
}

void test_two_step_cal_confirm() {
    auto c = parsed("CAL AIR");
    TEST_ASSERT_EQUAL_UINT8(Kind::CAL, c.kind);
    TEST_ASSERT_EQUAL_UINT8(CalAction::AIR, c.cal);

    c = parsed("CAL O2");               // arm step
    TEST_ASSERT_EQUAL_UINT8(CalAction::O2, c.cal);
    c = parsed("CAL O2 CONFIRM");       // execute step
    TEST_ASSERT_EQUAL_UINT8(CalAction::O2_CONFIRM, c.cal);
}

void test_compound_commands() {
    auto c = parsed("DIVE START");
    TEST_ASSERT_EQUAL_UINT8(Kind::DIVE_START, c.kind);
    c = parsed("LOG EXPORT 3");
    TEST_ASSERT_EQUAL_UINT8(Kind::LOG_EXPORT, c.kind);
    TEST_ASSERT_EQUAL_UINT16(3, c.logIndex);
    // The tenth slot: it has always existed and `LOG LIST` has always shown it,
    // but the parser's hard-coded 0..9 made it unreachable from a handset.
    c = parsed("LOG EXPORT 10");
    TEST_ASSERT_EQUAL_UINT8(Kind::LOG_EXPORT, c.kind);
    TEST_ASSERT_EQUAL_UINT16(10, c.logIndex);
    c = parsed("WIFI OFF");
    TEST_ASSERT_EQUAL_UINT8(Kind::WIFI, c.kind);
    TEST_ASSERT_EQUAL_STRING("OFF", c.key);
    c = parsed("CELLPROOF OVERRIDE");
    TEST_ASSERT_EQUAL_UINT8(Kind::CELLPROOF_OVERRIDE, c.kind);
    c = parsed("TEST CELLS");
    TEST_ASSERT_EQUAL_UINT8(Kind::TESTCELLS, c.kind);
}

// TIME SET is the only command that carries a number, and the grammar takes the
// digits without judging them: what counts as a believable dive date is the
// router's decision, not the parser's.
void test_time_set_grammar() {
    auto c = parsed("TIME SET 1795509660\r\n");
    TEST_ASSERT_EQUAL_UINT8(Kind::TIME_SET, c.kind);
    TEST_ASSERT_EQUAL_UINT32(1795509660u, c.epoch);

    c = parsed("TIME SET 7");            // nonsense value, valid grammar
    TEST_ASSERT_EQUAL_UINT8(Kind::TIME_SET, c.kind);
    TEST_ASSERT_EQUAL_UINT32(7u, c.epoch);

    parsed("TIME", false);
    parsed("TIME SET", false);
    parsed("TIME SET twelve", false);
    parsed("TIME SET 1 2", false);
    parsed("TIME NOW", false);

    // The read side needs no new verb: `GET TIME` is an ordinary GET key.
    c = parsed("GET TIME");
    TEST_ASSERT_EQUAL_UINT8(Kind::GET, c.kind);
    TEST_ASSERT_EQUAL_STRING("TIME", c.key);
}

void test_rejections_are_errs_not_crashes() {
    parsed("", false);
    parsed("SET", false);
    parsed("SET SP2", false);            // missing '='
    parsed("SET =0.70", false);          // missing key
    parsed("SET SP2=", false);           // missing value
    parsed("SET SP 2=0.70", false);      // '=' not inside the first token
    parsed("CAL", false);
    parsed("CAL BANANA", false);
    parsed("DIVE", false);
    parsed("LOG EXPORT", false);
    parsed("LOG EXPORT 11", false);      // past the store's slot count
    parsed("LOG EXPORT 12", false);      // ... and beyond it
    parsed("WIFI MAYBE", false);
    parsed("LAUNCH MISSILES", false);    // unknown verb
    const char* err = nullptr;
    Command c;
    char huge[MAX_LINE + 10];
    std::memset(huge, 'A', MAX_LINE + 9); huge[MAX_LINE + 9] = '\0';
    TEST_ASSERT_FALSE(parseCommand(huge, c, &err));
    TEST_ASSERT_EQUAL_STRING("line too long", err);
}

void test_format_telemetry_shape() {
    Telemetry t{};
    t.depthM = 12.34f; t.votedPpo2 = 0.71f;
    t.cells[0] = 0.72f; t.cells[1] = 0.70f; t.cells[2] = 0.71f;
    t.setpoint = 0.70f; t.mode = 1; t.flags = 0b100001;
    char buf[128];
    const size_t n = formatTelemetry(t, buf, sizeof(buf));
    TEST_ASSERT_TRUE(n > 10);
    TEST_ASSERT_EQUAL('T', buf[0]);
    TEST_ASSERT_NOT_NULL(std::strstr(buf, "12.34"));
    TEST_ASSERT_NOT_NULL(std::strstr(buf, "AUTO"));
    TEST_ASSERT_NOT_NULL(std::strstr(buf, "FLAGS0x21"));
    TEST_ASSERT_EQUAL('\n', buf[n - 1]);
    TEST_ASSERT_EQUAL(0, formatTelemetry(t, buf, 8)); // overflow -> 0
}

void test_format_deco_stops_and_overflow() {
    DecoInfo d{};
    d.ndlMin = 0; d.decoTotalMin = 14.5f; d.ceilingM = 6.0f;
    d.stops[0] = {9.0f, 3.5f}; d.stops[1] = {6.0f, 11.0f};
    d.stopCount = 2; d.cnsPct = 22.5f;
    char buf[128];
    const size_t n = formatDeco(d, buf, sizeof(buf));
    TEST_ASSERT_TRUE(n > 10);
    TEST_ASSERT_NOT_NULL(std::strstr(buf, "D,0,14.5,6.00"));
    TEST_ASSERT_NOT_NULL(std::strstr(buf, ",9|3.5,6|11.0,22.5\n"));
    TEST_ASSERT_EQUAL(0, formatDeco(d, buf, 12)); // overflow -> 0
}

void test_format_err_event() {
    char buf[64];
    TEST_ASSERT_EQUAL_STRING("ERR bad key\n", [&] { formatErr("bad key", buf, 64); return buf; }());
    const size_t n = formatEvent(930, 10, 1, buf, 64);
    TEST_ASSERT_EQUAL_STRING_LEN("E,930,10,1", buf, 10);
    TEST_ASSERT_EQUAL('\n', buf[n - 1]);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_simple_verbs);
    RUN_TEST(test_get_and_set);
    RUN_TEST(test_two_step_cal_confirm);
    RUN_TEST(test_compound_commands);
    RUN_TEST(test_time_set_grammar);
    RUN_TEST(test_rejections_are_errs_not_crashes);
    RUN_TEST(test_format_telemetry_shape);
    RUN_TEST(test_format_deco_stops_and_overflow);
    RUN_TEST(test_format_err_event);
    return UNITY_END();
}
