#include <unity.h>

#include <cstdio>
#include <cstring>
#include <string>

#include "comms/settings.h"
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

// ---- LineReader: the uplink framing rule, now visible to this suite --------
//
// Written because the same rule, inline in uart2_task.cpp, was wrong in two
// ways and both were observed on the bench (PLAN 16): the tail of an over-long
// line became a SECOND command, and inside the error rate-limit window the
// TRUNCATED prefix became a command — an over-long `SET SP2=0.33` + junk wrote
// 0.330. The question for each case: what would a broken build do differently?
struct Feed {
    LineReader rx;
    int lines = 0;
    int errs = 0;
    char last[MAX_LINE + 1] = {0};
    char err[64] = {0};

    void put(const char* s, uint32_t atMs) {
        for (; *s; ++s) {
            switch (rx.push(static_cast<unsigned char>(*s), atMs)) {
                case LineReader::Out::LINE:
                    ++lines;
                    std::strncpy(last, rx.line(), sizeof(last) - 1);
                    last[sizeof(last) - 1] = '\0';
                    break;
                case LineReader::Out::TOO_LONG:
                    ++errs;
                    std::strncpy(err, rx.errText(), sizeof(err) - 1);
                    err[sizeof(err) - 1] = '\0';
                    break;
                case LineReader::Out::NOTHING:
                default:
                    break;
            }
        }
    }
};

void test_line_reader_assembles_and_ignores_blanks() {
    Feed f;
    f.put("GET T\r\n", 0);
    TEST_ASSERT_EQUAL_INT(1, f.lines);
    TEST_ASSERT_EQUAL_STRING("GET T", f.last);

    // A bare newline is what the bench tools send to flush a bridge's boot
    // banner; it must draw nothing on its own.
    f.put("\n\r\n", 500);
    TEST_ASSERT_EQUAL_INT(1, f.lines);
    TEST_ASSERT_EQUAL_INT(0, f.errs);

    // Exactly MAX_LINE characters is NOT an overflow: the cap is a capacity,
    // and a build that rejects its own longest valid line breaks the protocol.
    Feed g;
    char ok[MAX_LINE + 3];
    std::memset(ok, 'B', MAX_LINE); ok[MAX_LINE] = '\n'; ok[MAX_LINE + 1] = '\0';
    g.put(ok, 0);
    TEST_ASSERT_EQUAL_INT(1, g.lines);
    TEST_ASSERT_EQUAL_INT(0, g.errs);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(MAX_LINE), static_cast<int>(std::strlen(g.last)));
}

void test_line_reader_overflow_drops_the_whole_line() {
    // The 2026 defect's first half: the bytes after the cap were collected and
    // terminated as a second command, so `GET ZZZ...` answered `ERR unknown
    // command` for its own tail. Here the physical line yields NO line at all.
    Feed f;
    char junk[MAX_LINE + 210];
    std::snprintf(junk, sizeof(junk), "GET %sZ\n", std::string(MAX_LINE, 'Z').c_str());
    f.put(junk, 0);
    TEST_ASSERT_EQUAL_INT(0, f.lines);          // nothing reached the router
    TEST_ASSERT_EQUAL_INT(1, f.errs);
    TEST_ASSERT_EQUAL_STRING("ERR line too long\n", f.err);

    // Recovery is immediate: the next line assembles normally.
    f.put("GET SP2\n", 50);
    TEST_ASSERT_EQUAL_INT(1, f.lines);
    TEST_ASSERT_EQUAL_STRING("GET SP2", f.last);
}

void test_line_reader_overflow_cannot_execute_a_write() {
    // The half the first bug report got wrong: with the ERR suppressed by the
    // rate limit, the old code left len at MAX_LINE and the terminator
    // dispatched the truncated first 64 bytes. Bench-observed: this exact shape
    // answered `OK SAFETY` and left SP2 at 0.330. Zero lines is the whole
    // verdict — the router must never see it, rate-limited or not.
    Feed f;
    char first[MAX_LINE + 130];
    std::snprintf(first, sizeof(first), "GET SP2\nSET SP2=0.33%s\n",
                  std::string(120, 'Z').c_str());
    f.put(first, 0);
    TEST_ASSERT_EQUAL_INT(1, f.lines);              // only the GET SP2 ever ran
    TEST_ASSERT_EQUAL_STRING("GET SP2", f.last);    // NOT a phantom SET
    TEST_ASSERT_EQUAL_INT(1, f.errs);

    // Same overflowing line again, inside the error window: the TEXT is
    // suppressed, the DISCARD is not — still no command, and still no write.
    char again[MAX_LINE + 130];
    std::snprintf(again, sizeof(again), "SET SP2=0.33%s\n", std::string(120, 'Z').c_str());
    f.put(again, 1000);
    TEST_ASSERT_EQUAL_INT(1, f.lines);
    TEST_ASSERT_EQUAL_INT(1, f.errs);               // rate limited: no second text
    TEST_ASSERT_EQUAL_STRING("GET SP2", f.last);
    f.put("GET SP2\n", 1500);
    TEST_ASSERT_EQUAL_INT(2, f.lines);
    TEST_ASSERT_EQUAL_STRING("GET SP2", f.last);
}

void test_line_reader_rate_limit_gates_the_text() {
    // "Gate the message, never the recovery" — checked from both sides: the
    // window suppresses repeats, and the window re-arms.
    Feed f;
    std::string big = std::string(MAX_LINE + 40, 'X') + "\n";
    f.put(big.c_str(), 100);
    f.put(big.c_str(), 1000);            // inside the 2 s window
    f.put(big.c_str(), 100 + LineReader::kErrGapMs + 1);   // just outside it
    TEST_ASSERT_EQUAL_INT(0, f.lines);
    TEST_ASSERT_EQUAL_INT(2, f.errs);

    // Unsigned subtraction makes the window wrap-safe: a board that has been up
    // ~50 days must not go quiet on overflow, and one that just wrapped must
    // not stampede.
    Feed w;
    std::string junk = std::string(MAX_LINE + 40, 'Y') + "\n";
    w.put(junk.c_str(), 0xFFFFFF00u);
    w.put(junk.c_str(), 0xFFFFFF00u + LineReader::kErrGapMs + 10);   // wrapped
    TEST_ASSERT_EQUAL_INT(0, w.lines);
    TEST_ASSERT_EQUAL_INT(2, w.errs);
}

// ---- the value/key caps: a parser that truncates can turn a refusal into an
// ---- accept, which is what happened on hardware (2026-11-24, PLAN 16):
// ---- AP_PASS publishes 8..31 characters while Command::value was 24 bytes, so a
// ---- 24..31 character passphrase answered `OK` and stored something shorter.
void test_a_full_length_passphrase_survives_the_parser() {
    // 31 is the published maximum in `GET SCHEMA`, and the number the page puts in
    // <input maxlength=>. Hand-typed, not computed from any constant: if the
    // schema's maximum ever moves, this case says so rather than following it.
    const char* pass = "correct horse battery staple X";      // 30 chars
    TEST_ASSERT_EQUAL_INT_MESSAGE(30, static_cast<int>(std::strlen(pass)),
                                  "fixture length: the point of the case is a "
                                  "specific length");
    auto c = parsed("SET AP_PASS=correct horse battery staple X");
    TEST_ASSERT_EQUAL_STRING("AP_PASS", c.key);
    TEST_ASSERT_EQUAL_STRING(pass, c.value);
    TEST_ASSERT_EQUAL_INT(30, static_cast<int>(std::strlen(c.value)));

    // 31, the actual declared maximum — and the value has a space in it, because
    // spaces are legal in a passphrase and the parser takes the REST OF THE LINE.
    c = parsed("SET AP_PASS=correct horse battery staple XY");
    TEST_ASSERT_EQUAL_INT(31, static_cast<int>(std::strlen(c.value)));
    TEST_ASSERT_EQUAL_STRING("correct horse battery staple XY", c.value);

    // Not just the schema's maximum: anything the REGISTRY could accept has to
    // arrive whole. 39 is longer than any string key allows, so the registry
    // refuses it — and that refusal, with the client's own words, is what must
    // reach the client instead of a truncated write.
    const std::string long39 = std::string("SET AP_PASS=") + std::string(39, 'p');
    c = parsed(long39.c_str());
    TEST_ASSERT_EQUAL_INT(39, static_cast<int>(std::strlen(c.value)));
}

void test_over_long_values_are_refused_not_truncated() {
    // The doctrine already established for LINES (an over-long line is dropped
    // whole, never truncated-and-run) applied to the FIELDS inside a line: this
    // used to copy 39 characters and answer OK. 40 is one past what the parser
    // can carry, and the refusal uses the registry's own phrase so a client
    // matches one vocabulary at either layer.
    const char* err = nullptr;
    Command c;
    const std::string line = std::string("SET AP_PASS=") + std::string(40, 'q');
    TEST_ASSERT_FALSE_MESSAGE(parseCommand(line.c_str(), c, &err),
                              "a 40-character value was accepted: the parser is "  
                              "truncating it again");
    TEST_ASSERT_EQUAL_STRING("value too long", err);

    // Same for the key: a truncated key could address something the client did not
    // name, which is worse than an unknown key.
    err = nullptr;
    const std::string kline = std::string("SET ") + std::string(24, 'K') + "=1";
    TEST_ASSERT_FALSE_MESSAGE(parseCommand(kline.c_str(), c, &err),
                              "a 24-character key was accepted");
    TEST_ASSERT_EQUAL_STRING("key too long", err);
    TEST_ASSERT_EQUAL_UINT8(Kind::SET, c.kind);   // refused, and nothing was read
    TEST_ASSERT_EQUAL_STRING("", c.key);

    // The GET path has to say the same thing, and this is the only witness that it
    // does — proven by removing the guard, which changed NOTHING observable: today's
    // longest published key is well under 24 characters, so a truncated GET key never
    // matches anything and the answer is `unknown key` either way. Which is exactly
    // why the words matter: "I have no such setting" and "your key is too long to be
    // one" are different diagnoses, and a client that is told the first will go looking
    // for a typo instead of its own length bug. It also stops a future long key from
    // turning truncation into addressing.
    err = nullptr;
    const std::string gline = std::string("GET ") + std::string(24, 'K');
    TEST_ASSERT_FALSE_MESSAGE(parseCommand(gline.c_str(), c, &err),
                              "a 24-character GET key was accepted");
    TEST_ASSERT_EQUAL_STRING_MESSAGE("key too long", err,
                                     "a too-long key was reported as an unknown key");
    TEST_ASSERT_EQUAL_STRING("", c.key);
}

void test_every_published_key_and_max_length_value_reaches_the_parser() {
    // The contract test, table-driven, so it cannot be satisfied by the one key
    // that happened to be under review. For every key the schema publishes:
    //   * the key itself must survive `GET <key>` and `SET <key>=x` intact, so a
    //     command can never address a DIFFERENT setting than the client named;
    //   * a value of exactly the key's published maximum length (strings: `maxv`
    //     characters) must survive the parse whole, so the registry — not the
    //     transport — is what decides whether a write is acceptable.
    // This is the check that would have failed for years on `AP_PASS`.
    size_t n = 0;
    const settings::Def* table = settings::Registry::definitions(n);
    int checked = 0;
    for (size_t i = 0; i < n; ++i) {
        const settings::Def& d = table[i];
        std::string line = std::string("GET ") + d.key;
        Command c;
        const char* err = nullptr;
        TEST_ASSERT_TRUE_MESSAGE(parseCommand(line.c_str(), c, &err),
                                 (std::string("GET ") + d.key + " was refused").c_str());
        TEST_ASSERT_EQUAL_STRING_MESSAGE(d.key, c.key,
                                         (std::string("key truncated by the parser: ") +
                                          d.key).c_str());

        // longest legal value for this key
        std::string val;
        if (d.type == settings::Type::STRING) {
            val = std::string(static_cast<size_t>(d.maxv), 'x');
        } else {
            val = "1";
        }
        line = std::string("SET ") + d.key + "=" + val;
        if (line.size() > MAX_LINE) continue;   // the LINE cap is a different rule
        err = nullptr;
        TEST_ASSERT_TRUE_MESSAGE(parseCommand(line.c_str(), c, &err),
                                 (std::string(d.key) + " with its maximum-length value "
                                  "was refused by the parser").c_str());
        TEST_ASSERT_EQUAL_STRING(d.key, c.key);
        TEST_ASSERT_EQUAL_STRING_MESSAGE(val.c_str(), c.value,
                                         (std::string(d.key) + ": the parser shortened a "
                                          "value the schema publishes as legal").c_str());
        ++checked;
    }
    // A walk that checked nothing is decoration.
    TEST_ASSERT_GREATER_OR_EQUAL_INT_MESSAGE(25, checked,
                                             "the schema walk covered too few keys");
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
    RUN_TEST(test_line_reader_assembles_and_ignores_blanks);
    RUN_TEST(test_line_reader_overflow_drops_the_whole_line);
    RUN_TEST(test_line_reader_overflow_cannot_execute_a_write);
    RUN_TEST(test_line_reader_rate_limit_gates_the_text);
    RUN_TEST(test_a_full_length_passphrase_survives_the_parser);
    RUN_TEST(test_over_long_values_are_refused_not_truncated);
    RUN_TEST(test_every_published_key_and_max_length_value_reaches_the_parser);
    return UNITY_END();
}
