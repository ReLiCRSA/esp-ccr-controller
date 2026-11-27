// ============================================================================
// test_designwire — the handset mockup's wire literals are real wire literals.
//
// design/handset-screen/ draws 41 panels for a handset that does not exist yet,
// and every panel carries the lines that produced it inside its <desc>, e.g.
//
//   T,18.35,0.712,0.708,0.715,0.711,0.700,AUTO,FLAGS0x00,12.41 | D,17,24.5,...
//
// Those lines are the design's data contract: a screen is not a picture, it is a
// picture plus the facts that make it. They were typed by hand from the spec, and
// a hand-typed wire literal rots in three ways, all of which look fine on screen:
//
//   * it stops parsing — the field count or the grammar moved (the `T,` battery
//     field, the `L,` fourth field and the `DIVE,` v3 date all moved in 2026 and a
//     2 Hz mock drawn before them still looks authoritative);
//   * it becomes IMPOSSIBLE — a `T,` line with FLAGS0x80 beside a deco picture
//     whose CNS is 12 % describes a state the controller can never produce, and a
//     developer implementing the screen then writes the CNS alarm against a
//     threshold that never fires;
//   * it contradicts the firmware's own projection — the panel shows a flag bit
//     for a fault and an `E,` event row naming a different one.
//
// So each line is fed to handset/handset_link.h — the same client library a real
// handset is built on, and the one the conformance suite grades — and the parsed
// values are checked against the firmware's own published semantics: comms/flags.h
// for what a flag bit means, and CNS_ALARM_PCT for what an alarm threshold is.
// A mock that no longer parses, or describes a state the controller cannot
// produce, fails here by screen name.
//
// This is a design-tree test, not a firmware test: it exists so the 41 drawings
// stay a usable handoff while the handset is being built.
// ============================================================================
#include <unity.h>

#include <dirent.h>

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "../../handset/handset_link.h"
#include "comms/flags.h"
#include "config/defaults.h"
#include "control/alarms.h"

namespace {

const char* kScreenDir = "design/handset-screen/screens";

std::string readFile(const char* path) {
    FILE* f = std::fopen(path, "rb");
    if (!f) return "";
    std::string out;
    char buf[8192];
    size_t n = 0;
    while ((n = std::fread(buf, 1, sizeof(buf), f)) > 0) out.append(buf, n);
    std::fclose(f);
    return out;
}

std::string unescape(std::string s) {
    const std::pair<const char*, const char*> subs[] = {
        {"&gt;", ">"}, {"&lt;", "<"}, {"&quot;", "\""}, {"&#39;", "'"}, {"&amp;", "&"},
    };
    for (const auto& sv : subs) {
        const std::string from = sv.first, to = sv.second;
        for (size_t at = s.find(from); at != std::string::npos; at = s.find(from, at + to.size()))
            s.replace(at, from.size(), to);
    }
    return s;
}

struct WireLine {
    std::string screen;   // the SVG's own name, so a failure says which panel
    std::string line;     // the protocol line as the panel publishes it
};

/// The `<desc>` payload of one screen, cut into candidate protocol lines.
/// Panels write `CTRL ·` prose and command echoes in the same field, so a
/// fragment counts as a wire line only when it BEGINS with a published prefix
/// followed by a value character — `D, arrives at 0.5 Hz` is prose, `D,17,...`
/// is a line.
std::vector<std::string> descLines(const std::string& svg) {
    const size_t open = svg.find("<desc");
    if (open == std::string::npos) return {};
    const size_t close = svg.find("</desc>", open);
    if (close == std::string::npos) return {};
    const size_t gt = svg.find('>', open);
    if (gt == std::string::npos || gt > close) return {};
    const std::string body = unescape(svg.substr(gt + 1, close - gt - 1));

    static const char* kPrefix[] = {"T,", "D,", "E,", "PRE,", "CELLS,", "PROOF,", "CALAIR,",
                                    "TIME,", "DS,", "WIFI,", "L,", "K,", "DIVE,", "M,"};
    std::vector<std::string> out;
    size_t pos = 0;
    while (true) {
        const size_t bar = body.find(" | ", pos);
        std::string frag = body.substr(pos, bar == std::string::npos ? std::string::npos : bar - pos);
        // A reply echo (`GET CALAIR -> CALAIR,2,...`) carries its line after the
        // arrow; a prose note is cut at the middle dot that separates it.
        const size_t arrow = frag.find("->");
        if (arrow != std::string::npos) frag = frag.substr(arrow + 2);
        const size_t dot = frag.find(" \xc2\xb7 ");
        if (dot != std::string::npos) frag = frag.substr(0, dot);
        // trim
        const size_t a = frag.find_first_not_of(" \t");
        const size_t b = frag.find_last_not_of(" \t");
        frag = (a == std::string::npos) ? std::string() : frag.substr(a, b - a + 1);
        for (const char* p : kPrefix) {
            if (frag.compare(0, std::strlen(p), p) != 0) continue;
            const char after = frag[std::strlen(p)];
            if (after == '\0' || after == ' ') break;      // prose that starts like a line
            out.push_back(frag);
            break;
        }
        if (bar == std::string::npos) break;
        pos = bar + 3;
    }
    return out;
}

std::vector<WireLine> allWireLines() {
    std::vector<WireLine> out;
    DIR* d = opendir(kScreenDir);
    TEST_ASSERT_NOT_NULL_MESSAGE(d, "cannot open the design screen directory — run from the project root");
    std::vector<std::string> names;
    while (dirent* e = readdir(d)) {
        const std::string n = e->d_name;
        if (n.size() > 4 && n.compare(n.size() - 4, 4, ".svg") == 0) names.push_back(n);
    }
    closedir(d);
    std::sort(names.begin(), names.end());
    for (const std::string& n : names) {
        const std::string svg = readFile((std::string(kScreenDir) + "/" + n).c_str());
        TEST_ASSERT_FALSE_MESSAGE(svg.empty(), n.c_str());
        const std::string screen = n.substr(0, n.size() - 4);
        for (const std::string& l : descLines(svg))
            out.push_back({screen, l});
    }
    return out;
}

int fieldCount(const std::string& line) {
    int n = 1;
    for (char c : line)
        if (c == ',') ++n;
    return n;
}

/// decimals after the point in a comma-separated field, -1 when there is none.
int decimalsOf(const std::string& line, int field) {
    int idx = 0, start = 0;
    for (size_t i = 0; i <= line.size(); ++i) {
        if (i == line.size() || line[i] == ',') {
            if (idx == field) {
                const std::string f = line.substr(start, i - start);
                const size_t dot = f.find('.');
                return dot == std::string::npos ? -1 : static_cast<int>(f.size() - dot - 1);
            }
            ++idx;
            start = static_cast<int>(i) + 1;
        }
    }
    return -2;   // field does not exist
}

/// The raw ` | `-separated fragments of every panel's `<desc>`, before any
/// arrow-cut. The wire-line scan above wants the answer side of a reply echo;
/// the command-honesty rule below wants the command side, which is exactly what
/// the handset would send.
std::vector<WireLine> allFragments() {
    std::vector<WireLine> out;
    DIR* d = opendir(kScreenDir);
    if (!d) return out;
    std::vector<std::string> names;
    while (dirent* e = readdir(d)) {
        const std::string n = e->d_name;
        if (n.size() > 4 && n.compare(n.size() - 4, 4, ".svg") == 0) names.push_back(n);
    }
    closedir(d);
    std::sort(names.begin(), names.end());
    for (const std::string& n : names) {
        const std::string svg = readFile((std::string(kScreenDir) + "/" + n).c_str());
        const size_t open = svg.find("<desc");
        if (open == std::string::npos) continue;
        const size_t close = svg.find("</desc>", open);
        const size_t gt = svg.find('>', open);
        if (close == std::string::npos || gt == std::string::npos || gt > close) continue;
        const std::string body = unescape(svg.substr(gt + 1, close - gt - 1));
        size_t pos = 0;
        while (true) {
            const size_t bar = body.find(" | ", pos);
            std::string frag = body.substr(pos, bar == std::string::npos ? std::string::npos : bar - pos);
            const size_t a = frag.find_first_not_of(" \t");
            const size_t b = frag.find_last_not_of(" \t");
            frag = (a == std::string::npos) ? std::string() : frag.substr(a, b - a + 1);
            if (!frag.empty()) out.push_back({n.substr(0, n.size() - 4), frag});
            if (bar == std::string::npos) break;
            pos = bar + 3;
        }
    }
    return out;
}

}  // namespace

void setUp() {}
void tearDown() {}

// Every line in the design must parse with the parser a real handset would use.
// This is the check that catches the design ageing: the battery field on `T,`,
// the fourth field on `L,` and the v3 dive header all arrived after these panels
// were first discussed, and a mock that predates a format change keeps looking
// authoritative on screen forever.
void test_every_protocol_line_in_the_design_parses_on_the_client_library() {
    const std::vector<WireLine> lines = allWireLines();
    TEST_ASSERT_GREATER_THAN_INT_MESSAGE(40, static_cast<int>(lines.size()),
                                         "the design's <desc> payloads yielded almost no protocol "
                                         "lines: the scan is grading nothing");
    int byPrefix[14] = {0};   // T, D, E, PRE, CELLS, PROOF, CALAIR, TIME, DS, WIFI, L, K, DIVE, M
    int parsed = 0;
    for (const WireLine& w : lines) {
        bool ok = false;
        const std::string& l = w.line;
        hs::Telemetry t{};
        hs::Deco d{};
        hs::Event ev{};
        hs::PreStatus pre{};
        hs::CellsTest cells{};
        hs::Proof proof{};
        hs::CalAir cal{};
        hs::Clock clk{};
        hs::DepthSource ds{};
        hs::WifiState wifi{};
        hs::LogEntry log{};
        hs::SchemaEntry sch{};
        hs::DiveHeader dh{};
        if (l.compare(0, 2, "T,") == 0) { ok = hs::parseTelemetry(l.c_str(), t); byPrefix[0]++; }
        else if (l.compare(0, 2, "D,") == 0) { ok = hs::parseDeco(l.c_str(), d); byPrefix[1]++; }
        else if (l.compare(0, 2, "E,") == 0) { ok = hs::parseEvent(l.c_str(), ev); byPrefix[2]++; }
        else if (l.compare(0, 4, "PRE,") == 0) { ok = hs::parsePre(l.c_str(), pre); byPrefix[3]++; }
        else if (l.compare(0, 6, "CELLS,") == 0) { ok = hs::parseCells(l.c_str(), cells); byPrefix[4]++; }
        else if (l.compare(0, 6, "PROOF,") == 0) { ok = hs::parseProof(l.c_str(), proof); byPrefix[5]++; }
        else if (l.compare(0, 7, "CALAIR,") == 0) { ok = hs::parseCalAir(l.c_str(), cal); byPrefix[6]++; }
        else if (l.compare(0, 5, "TIME,") == 0) { ok = hs::parseClock(l.c_str(), clk); byPrefix[7]++; }
        else if (l.compare(0, 3, "DS,") == 0) { ok = hs::parseDepthSource(l.c_str(), ds); byPrefix[8]++; }
        else if (l.compare(0, 5, "WIFI,") == 0) { ok = hs::parseWifiState(l.c_str(), wifi); byPrefix[9]++; }
        else if (l.compare(0, 2, "L,") == 0) { ok = hs::parseLogList(l.c_str(), log); byPrefix[10]++; }
        else if (l.compare(0, 2, "K,") == 0) { ok = hs::parseSchema(l.c_str(), sch); byPrefix[11]++; }
        else if (l.compare(0, 5, "DIVE,") == 0) { ok = hs::parseDiveHeader(l.c_str(), dh); byPrefix[12]++; }
        else if (l.compare(0, 2, "M,") == 0) { ok = true; byPrefix[13]++; }   // field count, next case
        TEST_ASSERT_TRUE_MESSAGE(ok, (w.screen + ": " + l + " does not parse on handset_link.h: "
                                      "the panel's data source is no longer a real line").c_str());
        if (ok) ++parsed;
    }
    // Non-vacuous: the design must keep exercising the line kinds it claims to.
    const int kMin[] = {10, 10, 5, 3, 1, 2, 5, 8, 3, 3, 3, 3, 1, 1};
    for (int i = 0; i < 14; ++i)
        TEST_ASSERT_GREATER_OR_EQUAL_INT_MESSAGE(kMin[i], byPrefix[i],
                                                 "the design lost the panels that exercise one of its "
                                                 "line kinds");
    printf("  designwire: %d protocol lines parsed across the 41 panels\n", parsed);
}

// The `M,` totals row has no client parser (a handset reads it as text), so its
// shape is graded against the spec's own published template rather than assumed:
// count the columns in HANDSET_PROTOCOL.md's `M,` line and require the design's
// mock to have the same number. A design row with one column fewer than the
// template is a screen someone will implement against the wrong field order.
void test_the_dive_totals_mock_has_the_columns_the_spec_publishes() {
    const std::string spec = readFile("docs/HANDSET_PROTOCOL.md");
    TEST_ASSERT_FALSE(spec.empty());
    const size_t at = spec.find("\nM,<");
    TEST_ASSERT_NOT_EQUAL_MESSAGE(std::string::npos, at, "the spec no longer publishes an M, template");
    const size_t eol = spec.find('\n', at + 1);
    const std::string tmpl = spec.substr(at + 1, spec.find(' ', at + 1) - at - 1);
    const int want = fieldCount(tmpl);
    TEST_ASSERT_GREATER_THAN_INT_MESSAGE(10, want, "the M, template read is wrong");

    int graded = 0;
    for (const WireLine& w : allWireLines()) {
        if (w.line.compare(0, 2, "M,") != 0) continue;
        ++graded;
        TEST_ASSERT_EQUAL_INT_MESSAGE(want, fieldCount(w.line),
                                      (w.screen + ": its M, row has a different column count than the "
                                       "spec publishes").c_str());
    }
    TEST_ASSERT_GREATER_OR_EQUAL_INT_MESSAGE(1, graded, "no M, row in the design to grade");
}

// The `T,` grammar is exact — depth 2 decimals, ppO2 / cells / setpoint 3,
// battery 2, MODE from a closed set — and it is the line the HUD's hero digits are
// drawn from. A mock with 1.3 instead of 1.300 teaches an implementer the wrong
// field width, which is the kind of thing that becomes a truncated display at 2 Hz.
void test_the_telemetry_mocks_respect_the_published_field_shapes() {
    static const char* kModes[] = {"STANDBY", "AUTO", "MANUAL", "SAFE"};
    int graded = 0;
    for (const WireLine& w : allWireLines()) {
        if (w.line.compare(0, 2, "T,") != 0) continue;
        ++graded;
        TEST_ASSERT_EQUAL_INT_MESSAGE(10, fieldCount(w.line), (w.screen + ": T, must have 10 fields").c_str());
        const int want[] = {2, 3, 3, 3, 3, 3, -1, -1, 2};   // depth, ppO2, c0..c2, sp, mode, flags, batt
        for (int f = 1; f <= 9; ++f) {
            if (want[f - 1] < 0) continue;
            const int got = decimalsOf(w.line, f);
            TEST_ASSERT_EQUAL_INT_MESSAGE(want[f - 1], got,
                                          (w.screen + ": T, field " + std::to_string(f) +
                                           " has the wrong number of decimals for the published format")
                                              .c_str());
        }
        // MODE from the closed published set (safe-mode is drawn on purpose: the
        // grammar allows it and the panel must show what a future build says).
        int idx = 0, start = 0;
        std::string mode;
        for (size_t i = 0; i <= w.line.size(); ++i) {
            if (i == w.line.size() || w.line[i] == ',') {
                if (idx == 7) mode = w.line.substr(start, i - start);
                ++idx;
                start = static_cast<int>(i) + 1;
            }
        }
        bool known = false;
        for (const char* m : kModes)
            if (mode == m) known = true;
        TEST_ASSERT_TRUE_MESSAGE(known, (w.screen + ": T, MODE \"" + mode + "\" is not in the "
                                         "published set").c_str());
    }
    TEST_ASSERT_GREATER_OR_EQUAL_INT_MESSAGE(10, graded, "not enough T, mocks to grade");
}

// The interesting one: a panel's mock data must describe a state the controller
// can actually produce, checked through the firmware's own rules.
//
//   * a FLAGS value must be one the projection can output — comms/flags.h is the
//     function the controller runs, so the mock is asked of it directly;
//   * a panel that lights the CNS flag must show a CNS at or over the shipping
//     alarm threshold (CNS_ALARM_PCT in include/config/defaults.h): FLAGS0x80
//     beside a 12 % gauge is a picture of a state that cannot exist, and the
//     developer who implements the alarm from it wires it to a number that never
//     fires;
//   * an `E,<t>,1|2,<bit>` alarm event on a panel must agree with that panel's
//     own FLAGS byte — the row says which fault fired and the telemetry line says
//     which pills are lit, and a panel that shows them disagreeing is a panel
//     describing a fault the machine is not reporting.
void test_the_panel_mocks_describe_states_the_controller_can_produce() {
    const std::vector<WireLine> lines = allWireLines();

    // screen -> its flags byte, its CNS reading, its alarm-edge event bits.
    struct PerScreen {
        std::string screen;
        int flags = -1;
        float cns = -1.0f;
        std::vector<int> alarmBits;
    };
    std::vector<PerScreen> screens;
    auto find = [&](const std::string& s) -> PerScreen& {
        for (auto& p : screens)
            if (p.screen == s) return p;
        screens.push_back({s, -1, -1.0f, {}});
        return screens.back();
    };

    for (const WireLine& w : lines) {
        if (w.line.compare(0, 2, "T,") == 0) {
            hs::Telemetry t{};
            TEST_ASSERT_TRUE(hs::parseTelemetry(w.line.c_str(), t));
            PerScreen& p = find(w.screen);
            TEST_ASSERT_EQUAL_INT_MESSAGE(-1, p.flags, (w.screen + ": two T, lines on one panel").c_str());
            p.flags = t.flags;
        } else if (w.line.compare(0, 2, "D,") == 0) {
            hs::Deco d{};
            TEST_ASSERT_TRUE(hs::parseDeco(w.line.c_str(), d));
            PerScreen& p = find(w.screen);
            if (p.cns < 0.0f) p.cns = d.cnsPct;
        } else if (w.line.compare(0, 2, "E,") == 0) {
            hs::Event e{};
            TEST_ASSERT_TRUE(hs::parseEvent(w.line.c_str(), e));
            if (e.id == static_cast<uint8_t>(1) || e.id == static_cast<uint8_t>(2))
                find(w.screen).alarmBits.push_back(e.value);   // alarm raised / cleared
        }
    }

    int flagged = 0, cnsFlagged = 0, alarmRows = 0;
    for (const PerScreen& p : screens) {
        if (p.flags < 0) continue;
        ++flagged;
        // (1) reachable by the projection at all: some alarm word + proof state
        //     produces exactly this byte.
        bool producible = false;
        for (uint16_t w = 0; w < 0x0400; ++w)
            if (comms::flagsFromAlarmWord(w, true) == static_cast<uint8_t>(p.flags) ||
                comms::flagsFromAlarmWord(w, false) == static_cast<uint8_t>(p.flags))
                producible = true;
        char hexbuf[8];
        std::snprintf(hexbuf, sizeof hexbuf, "%02X", p.flags);
        TEST_ASSERT_TRUE_MESSAGE(producible,
                                 (p.screen + ": FLAGS0x" + std::string(hexbuf) +
                                  " is not a byte the controller's projection can output").c_str());

        // (2) the CNS flag and the CNS reading on the same panel must agree.
        if (p.flags & (1 << 7)) {
            ++cnsFlagged;
            TEST_ASSERT_GREATER_OR_EQUAL_FLOAT_MESSAGE(
                static_cast<double>(CNS_ALARM_PCT), p.cns,
                (p.screen + ": the CNS flag is lit while the deco row beside it reads a CNS below "
                             "CNS_ALARM_PCT — this panel describes a state that cannot happen")
                    .c_str());
        }

        // (3) the alarm rows a panel shows must be visible in its FLAGS byte.
        for (int bit : p.alarmBits) {
            ++alarmRows;
            TEST_ASSERT_TRUE_MESSAGE(bit >= 0 && bit < alarms::BIT_COUNT,
                                     (p.screen + ": an E, alarm row names alarm bit " +
                                      std::to_string(bit) + ", which is not in the alarm word")
                                         .c_str());
            const uint8_t expect = comms::flagsFromAlarmWord(static_cast<uint16_t>(1u << bit), false);
            TEST_ASSERT_TRUE_MESSAGE(expect == 0 || (p.flags & expect),
                                     (p.screen + ": shows an alarm event for alarm bit " +
                                      std::to_string(bit) + " while its T, line leaves the matching "
                                                           "FLAGS bit clear").c_str());
        }
    }
    TEST_ASSERT_GREATER_OR_EQUAL_INT_MESSAGE(10, flagged, "no panel flags were graded");
    TEST_ASSERT_GREATER_OR_EQUAL_INT_MESSAGE(1, cnsFlagged,
                                             "the CNS-high panel lost its mock: nothing grades (2)");
    TEST_ASSERT_GREATER_OR_EQUAL_INT_MESSAGE(2, alarmRows, "no alarm event rows in the mocks to grade");
}

// Two rules the design sheet publishes about itself, graded so they cannot rot
// into habits: the panels may not offer a command the controller refuses as
// retired, and a command that answers `not implemented` must be drawn answering
// `not implemented` — a mock that shows CAL O2 succeeding teaches someone to
// build a button for a feature that does not exist.
void test_the_design_does_not_invent_supported_commands() {
    // The fragments' COMMAND side is what a handset would send, so this is where
    // a lie about what the controller supports would live.
    const std::vector<WireLine> frags = allFragments();
    TEST_ASSERT_GREATER_THAN_INT_MESSAGE(80, static_cast<int>(frags.size()),
                                         "the fragment scan found almost nothing to grade");
    int graded = 0;
    for (const WireLine& w : frags) {
        // `PULSE` was retired 2026-11-26 (spec §8): there is no handset-originated
        // injection path, so no panel may show the verb.
        TEST_ASSERT_NULL_MESSAGE(std::strstr(w.line.c_str(), "PULSE"),
                                 (w.screen + ": shows the retired PULSE verb").c_str());
        if (w.line.find("CAL O2") != std::string::npos) {
            ++graded;
            TEST_ASSERT_NOT_NULL_MESSAGE(std::strstr(w.line.c_str(), "not implemented"),
                                         (w.screen + ": CAL O2 is drawn as something other than "
                                          "`ERR not implemented` — the controller refuses it, and a "
                                          "mock that shows it working gets implemented as a button")
                                             .c_str());
        }
    }
    TEST_ASSERT_GREATER_OR_EQUAL_INT_MESSAGE(2, graded,
                                             "the CAL O2 panels stopped exercising the refusal");
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_every_protocol_line_in_the_design_parses_on_the_client_library);
    RUN_TEST(test_the_dive_totals_mock_has_the_columns_the_spec_publishes);
    RUN_TEST(test_the_telemetry_mocks_respect_the_published_field_shapes);
    RUN_TEST(test_the_panel_mocks_describe_states_the_controller_can_produce);
    RUN_TEST(test_the_design_does_not_invent_supported_commands);
    return UNITY_END();
}
