// ============================================================================
// test_flaglabels — one fault, one name, in all three places that name it.
//
// The `FLAGS` byte is named in four places, and they are allowed to disagree
// about nothing:
//   * the firmware's projection — comms/flags.h, executed here by name;
//   * the web page's pills — FLAGNAMES in src/comms/web_page.h, and the copy of
//     the page that is actually flashed, data/index.html;
//   * the contract a handset is built against — docs/HANDSET_PROTOCOL.md §4's
//     table.
// Three vocabularies with nothing checking them against each other is how a
// pill ends up naming the wrong fault, and the review's M-3 finding was exactly
// that: bit 6 (cell proof FAIL) was labelled "SENSOR" on the page, so a diver
// was told to check sensors while the machine's actual verdict was "the oxygen
// sensor's own output cannot be trusted at this setpoint" — and the fix at the
// time was to hand-edit three files, which is a fix that silently re-breaks the
// next time someone touches one of them.
//
// So this grades the AGREEMENT rather than the wording. Each check names the
// place it failed, because a test that says "the labels disagree" without saying
// which file is a test that gets ignored.
//
// Deliberately NOT graded: the exact wording. Prose is allowed to be prose —
// §4's row may read "cell proof currently FAIL" while the pill reads "PROOF
// FAIL". What is not allowed is a row and a pill that stop describing the same
// fault, so the join is on the fault's own word, per bit.
// ============================================================================
#include <unity.h>

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "comms/flags.h"
#include "control/alarms.h"
#include "control/cellproof.h"

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

std::string lower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return s;
}

/// The FLAGNAMES declaration, exactly as the page declares it. Empty when the
/// page no longer has one — which is itself the failure the caller reports.
std::string flagNamesLine(const std::string& page) {
    const size_t at = page.find("FLAGNAMES=[");
    if (at == std::string::npos) return "";
    const size_t end = page.find(']', at);
    if (end == std::string::npos) return "";
    return page.substr(at, end - at + 1);
}

/// The declared entries, unquoted, in declaration order — index i is bit i.
std::vector<std::string> flagNames(const std::string& decl) {
    std::vector<std::string> out;
    size_t p = decl.find('[');
    if (p == std::string::npos) return out;
    const size_t last = decl.rfind(']');
    std::string body = decl.substr(p + 1, last - p - 1);
    size_t i = 0;
    while (i < body.size()) {
        const size_t q1 = body.find('"', i);
        if (q1 == std::string::npos) break;
        const size_t q2 = body.find('"', q1 + 1);
        if (q2 == std::string::npos) break;
        out.push_back(body.substr(q1 + 1, q2 - q1 - 1));
        i = q2 + 1;
    }
    return out;
}

/// §4's FLAGS table, one entry per published bit: the row's bit number and the
/// row's meaning text. Parsed from the spec so the spec is the source of truth
/// about what a bit means, rather than a second copy in this test.
struct SpecRow {
    int bit;
    std::string meaning;
};

std::vector<SpecRow> specFlagRows(const std::string& doc) {
    std::vector<SpecRow> rows;
    const size_t anchor = doc.find("**FLAGS byte bits:**");
    if (anchor == std::string::npos) return rows;
    size_t p = anchor;
    bool started = false;
    while (true) {
        const size_t nl = doc.find('\n', p);
        if (nl == std::string::npos) break;
        const size_t next = doc.find('\n', nl + 1);
        const std::string line = doc.substr(nl + 1, next == std::string::npos ? std::string::npos : next - nl - 1);
        p = nl + 1;   // advance PAST the newline just consumed
        const size_t first = line.find_first_not_of(" \t");
        if (first == std::string::npos) {              // blank line
            if (started) break;                        // the table has ended
            continue;
        }
        if (line.compare(first, 2, "| ") != 0) {       // prose between anchor and table
            if (started) break;
            continue;
        }
        std::string cell = line.substr(first + 2, line.find('|', first + 2) - first - 2);
        const size_t cs = cell.find_first_not_of(" \t");
        const size_t ce = cell.find_last_not_of(" \t");
        cell = (cs == std::string::npos) ? std::string() : cell.substr(cs, ce - cs + 1);
        bool digits = !cell.empty();
        for (char c : cell)
            if (!std::isdigit(static_cast<unsigned char>(c))) digits = false;
        if (!digits) {                                  // header or |---|---| rule
            if (started) break;
            continue;
        }
        const size_t c2 = line.find('|', first + 2);
        const size_t c3 = line.find('|', c2 + 1);
        if (c3 == std::string::npos) break;
        rows.push_back({std::stoi(cell), line.substr(c2 + 1, c3 - c2 - 1)});
        started = true;
    }
    return rows;
}

// The word each published bit's fault is named by, in the page and in the spec.
// Chosen per bit, not derived, because the point is that three documents agree on
// a fault's NAME — and a keyword table is the shortest statement of "which fault
// is bit 3" that a reader can check against §4 by eye.
const char* kBitKeyword[] = {
    "low",      // 0  ppO2 low
    "high",     // 1  ppO2 high
    "drift",    // 2  cell drift
    "sensor",   // 3  sensor-class failure (the aggregate)
    "batt",     // 4  battery low
    "clamp",    // 5  setpoint clamp (cell proof ban)
    "proof",    // 6  cell proof FAIL
    "cns",      // 7  CNS over the diver's limit
};
constexpr int kPublishedBits = static_cast<int>(sizeof(kBitKeyword) / sizeof(kBitKeyword[0]));

// The controller's own alarm word, with the word each condition is named by. The
// aggregate members are listed individually on purpose: §4's bit-3 row has to
// name every one of them, because a row that names three and projects four is
// the exact shape of the defect that dropped CNS_HIGH while claiming coverage.
struct AlarmWordCase {
    const char* name;    // alarms.h's own spelling
    uint8_t bit;
    const char* word;    // the word §4's destination row must contain
};
const AlarmWordCase kAlarmWord[] = {
    {"BIT_LOW_PPO2",     alarms::BIT_LOW_PPO2,     "low"},
    {"BIT_HIGH_PPO2",    alarms::BIT_HIGH_PPO2,    "high"},
    {"BIT_CELL_DRIFT",   alarms::BIT_CELL_DRIFT,   "drift"},
    {"BIT_SENSOR_FAIL",  alarms::BIT_SENSOR_FAIL,  "sensor fail"},
    {"BIT_SENSOR_STALE", alarms::BIT_SENSOR_STALE, "stale"},
    {"BIT_STUCK_OPEN",   alarms::BIT_STUCK_OPEN,   "stuck"},
    {"BIT_HYPOXIA",      alarms::BIT_HYPOXIA,      "hypoxia"},
    {"BIT_BATTERY",      alarms::BIT_BATTERY,      "batt"},
    {"BIT_SP_CLAMP",     alarms::BIT_SP_CLAMP,     "clamp"},
    {"BIT_CNS_HIGH",     alarms::BIT_CNS_HIGH,     "cns"},
};

/// Which published FLAGS bit an alarm-word bit lands in, from the firmware's own
/// projection: one bit set, its index. -1 when nothing or several came out,
/// which the caller fails loudly.
int projectedBit(uint16_t alarmBit) {
    const uint8_t f = comms::flagsFromAlarmWord(static_cast<uint16_t>(1u << alarmBit), false);
    if (f == 0 || (f & (f - 1)) != 0) return -1;
    int idx = 0;
    while (!(f & (1u << idx))) ++idx;
    return idx;
}

}  // namespace

void setUp() {}
void tearDown() {}

// The page exists twice: the source in src/comms/web_page.h and the copy that is
// actually flashed (data/index.html, installed to LittleFS). The bench proves the
// served bytes equal data/index.html; nothing proved data/index.html still agreed
// with the source, and the M-3 fix edited both by hand. This is that proof.
void test_the_flashed_page_and_the_source_page_name_the_flags_alike() {
    const std::string src = readFile("src/comms/web_page.h");
    const std::string flashed = readFile("data/index.html");
    TEST_ASSERT_MESSAGE(!src.empty(), "cannot read src/comms/web_page.h — run from the project root");
    TEST_ASSERT_MESSAGE(!flashed.empty(), "cannot read data/index.html — the flashed page copy");

    const std::string a = flagNamesLine(src), b = flagNamesLine(flashed);
    TEST_ASSERT_MESSAGE(!a.empty(), "src/comms/web_page.h has no FLAGNAMES=[...] declaration");
    TEST_ASSERT_EQUAL_STRING_MESSAGE(a.c_str(), b.c_str(),
                                     "the flashed page's pill names differ from the source page's: "
                                     "regenerate data/index.html with tools/sync_webdata.py");
}

void test_there_is_one_label_per_published_bit_and_none_is_reused() {
    const std::string decl = flagNamesLine(readFile("src/comms/web_page.h"));
    const std::vector<std::string> names = flagNames(decl);
    TEST_ASSERT_EQUAL_INT_MESSAGE(kPublishedBits, static_cast<int>(names.size()),
                                  "the page does not have exactly one pill per published FLAGS bit");
    for (int i = 0; i < kPublishedBits; ++i) {
        // A pill with no text is a pill that can never be read; a pill whose name
        // does not contain its bit's keyword is a pill naming another fault —
        // which is what bit 6 used to be ("SENSOR" on a cell-proof failure).
        TEST_ASSERT_TRUE_MESSAGE(names[i].find_first_not_of(" \t") != std::string::npos,
                                 "a pill label is empty");
        TEST_ASSERT_NOT_NULL_MESSAGE(std::strstr(lower(names[i]).c_str(), kBitKeyword[i]),
                                     (std::string("pill ") + std::to_string(i) + " is labelled \"" +
                                      names[i] + "\" and does not name the fault §4 publishes for bit " +
                                      std::to_string(i)).c_str());
    }
    for (int i = 0; i < kPublishedBits; ++i)
        for (int j = i + 1; j < kPublishedBits; ++j)
            TEST_ASSERT_NOT_EQUAL_MESSAGE(names[i], names[j],
                                          "two pills carry the same name: the diver cannot tell which "
                                          "fault lit");
}

void test_the_spec_publishes_every_bit_once_and_names_its_own_keyword() {
    const std::string doc = readFile("docs/HANDSET_PROTOCOL.md");
    TEST_ASSERT_MESSAGE(!doc.empty(), "cannot read docs/HANDSET_PROTOCOL.md");
    const std::vector<SpecRow> rows = specFlagRows(doc);
    TEST_ASSERT_EQUAL_INT_MESSAGE(kPublishedBits, static_cast<int>(rows.size()),
                                  "§4's FLAGS table does not publish exactly bits 0..7, one row each");
    for (int i = 0; i < kPublishedBits; ++i) {
        TEST_ASSERT_EQUAL_INT_MESSAGE(i, rows[i].bit,
                                      "§4's FLAGS table is not ordered by bit, or is missing a row");
        const std::string meaning = lower(rows[i].meaning);
        TEST_ASSERT_NOT_NULL_MESSAGE(std::strstr(meaning.c_str(), kBitKeyword[i]),
                                     (std::string("§4's row for bit ") + std::to_string(i) +
                                      " does not contain the word \"" + kBitKeyword[i] +
                                      "\" the page's pill is built on").c_str());
    }
}

// The join. Every condition in the controller's alarm word lands in exactly one
// published FLAGS bit (measured through comms/flags.h, the projection both
// transports call), and the §4 row of THAT bit must name that condition by its
// own word. This is the check that would have failed in 2026-11-27's original
// defect: CNS_HIGH was dropped on both transports while §4-style prose claimed
// the aggregate covered it. A dropped condition, a doubled condition, or a row
// that stopped naming one of its own members all fail here, by name.
void test_every_alarm_condition_lands_where_its_documentation_says_it_does() {
    const std::string doc = readFile("docs/HANDSET_PROTOCOL.md");
    const std::vector<SpecRow> rows = specFlagRows(doc);
    TEST_ASSERT_EQUAL_INT_MESSAGE(kPublishedBits, static_cast<int>(rows.size()),
                                  "cannot grade the projection without §4's table");

    int graded = 0;
    for (const AlarmWordCase& c : kAlarmWord) {
        const int bit = projectedBit(c.bit);
        TEST_ASSERT_NOT_EQUAL_MESSAGE(-1, bit,
                                      (std::string(c.name) + " does not project to exactly one FLAGS "
                                       "bit — it is dropped, or doubled")
                                          .c_str());
        TEST_ASSERT_TRUE_MESSAGE(bit < kPublishedBits,
                                 (std::string(c.name) + " projects outside the published byte").c_str());
        const std::string meaning = lower(rows[bit].meaning);
        TEST_ASSERT_NOT_NULL_MESSAGE(
            std::strstr(meaning.c_str(), c.word),
            (std::string(c.name) + " projects to FLAGS bit " + std::to_string(bit) +
             ", whose §4 row never says \"" + c.word +
             "\": the row and the projection have drifted apart")
                .c_str());
        ++graded;
    }
    TEST_ASSERT_EQUAL_INT_MESSAGE(10, graded, "the alarm-word table in this test went empty or short");

    // Bit 6 is not an alarm condition at all — it is the cell-proof state — and
    // it must still be reachable, or the page has a pill that can never light.
    const uint8_t proofOnly = comms::flagsFromAlarmWord(0, true);
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(static_cast<uint8_t>(1u << 6), proofOnly,
                                    "cell proof FAIL no longer reaches FLAGS bit 6");
    TEST_ASSERT_NOT_NULL_MESSAGE(std::strstr(lower(rows[6].meaning).c_str(), "proof"),
                                 "§4's bit-6 row does not name cell proof");
}

// Nothing outside the published byte may be invented by the projection, and an
// empty word must produce an empty byte: an alarm word of 0 lighting a pill would
// be an alarm invented out of nothing, and a stray bit beyond 7 would be a
// meaning no client has ever been told about.
void test_the_projection_invents_nothing_on_either_side() {
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(0, comms::flagsFromAlarmWord(0, false),
                                    "an empty alarm word produced a flag bit");
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(0xFFu, comms::flagsFromAlarmWord(0xFFFFu, true),
                                    "the projection left a published bit unreachable, or set a bit "
                                    "outside the byte §4 publishes");
    // The alarm word has 10 bits (alarms.h BIT_COUNT). Anything the controller
    // never sets must not light a pill: bits 10..15 arriving set means somebody
    // widened the word without widening the documentation.
    for (unsigned b = alarms::BIT_COUNT; b < 16; ++b)
        TEST_ASSERT_EQUAL_UINT8_MESSAGE(0, comms::flagsFromAlarmWord(static_cast<uint16_t>(1u << b), false),
                                        "the projection answered for a bit that is not in the alarm word");
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_the_flashed_page_and_the_source_page_name_the_flags_alike);
    RUN_TEST(test_there_is_one_label_per_published_bit_and_none_is_reused);
    RUN_TEST(test_the_spec_publishes_every_bit_once_and_names_its_own_keyword);
    RUN_TEST(test_every_alarm_condition_lands_where_its_documentation_says_it_does);
    RUN_TEST(test_the_projection_invents_nothing_on_either_side);
    return UNITY_END();
}
