// ============================================================================
// test_numfmt — the fixed-point writer must be byte-identical to the float
// printf it replaces, or this is a protocol change dressed up as a fix.
//
// Two kinds of test. Pinned cases spell out the answers a human checked (the
// ties especially: round-half-to-even is where a naive llround() implementation
// silently differs). The sweep is a differential test: for a few hundred
// thousand values across the ranges this device actually sends, compare against
// the C library's own snprintf("%.*f"). The device's printf is newlib and the
// host's is glibc, and both round to nearest with ties to even, so agreement
// across the sweep is the evidence that no wire byte moved.
// ============================================================================
#include <unity.h>

#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <string>

#include "util/numfmt.h"

void setUp() {}
void tearDown() {}

namespace {

void check(const char* file, int line, double v, int decimals) {
    char mine[64], theirs[64];
    numfmt::fixed(mine, sizeof(mine), v, decimals);
    std::snprintf(theirs, sizeof(theirs), "%.*f", decimals, v);
    if (std::strcmp(mine, theirs) != 0) {
        char msg[192];
        std::snprintf(msg, sizeof(msg), "%s:%d  %.*f -> mine '%s', printf '%s'",
                      file, line, decimals, v, mine, theirs);
        TEST_FAIL_MESSAGE(msg);
    }
}

} // namespace

#define CK(v, d) check(__FILE__, __LINE__, (v), (d))

void test_ties_round_half_to_even() {
    // The whole reason this test exists. Exact binary ties: printf gives the
    // even digit, llround-style code gives the other one.
    CK(0.125, 2);      // "0.12"
    CK(0.375, 2);      // "0.38"
    CK(2.5, 0);        // "2"
    CK(0.5, 0);        // "0"
    CK(1.5, 0);        // "2"
    CK(3.5, 0);        // "4"
    CK(-0.125, 2);
    CK(0.125, 3);      // no rounding at all: "0.125"
    CK(1.005, 2);      // the classic: 1.00, because 1.005 is not representable
    CK(2.675, 2);
    CK(0.005, 3);
}

void test_protocol_ranges() {
    // ppO2 / setpoint / cell bars: 0.000..5.000 at 3 places
    for (int i = 0; i <= 5000; ++i) CK(i / 1000.0, 3);
    // depth in metres at 2 places, to 200 m
    for (int i = 0; i <= 20000; ++i) CK(i / 100.0, 2);
    // battery volts at 2 places
    for (int i = 0; i <= 4000; ++i) CK(i / 100.0, 2);
    // CNS % at 1 place, deco minutes at 1 place, stop depths at 1 place
    for (int i = 0; i <= 1200; ++i) CK(i / 10.0, 1);
    // and the same magnitudes negative (ceiling can be reported negative)
    for (int i = 0; i <= 2000; ++i) CK(-i / 10.0, 1);
}

void test_signed_zero_and_negatives() {
    CK(-0.0, 2);       // printf prints "-0.00", and so must we
    CK(-0.0001, 2);    // small negative -> "-0.00"
    CK(0.0, 2);
    char b[16];
    TEST_ASSERT_EQUAL_STRING("-0.00", numfmt::fixed(b, sizeof(b), -0.0, 2));
}

void test_nonfinite_matches_strtod_vocabulary() {
    char b[16];
    TEST_ASSERT_EQUAL_STRING("nan", numfmt::fixed(b, sizeof(b), NAN, 2));
    TEST_ASSERT_EQUAL_STRING("inf", numfmt::fixed(b, sizeof(b), INFINITY, 2));
    TEST_ASSERT_EQUAL_STRING("-inf", numfmt::fixed(b, sizeof(b), -INFINITY, 2));
    // the client library's strtod accepts all three spellings
}

void test_too_large_is_refused_not_wrapped() {
    char b[32];
    // nothing here is a value this device can produce; the point is that it
    // must not wrap, print a partial number, or call float printf.
    TEST_ASSERT_EQUAL_STRING("", numfmt::fixed(b, sizeof(b), 1e30, 3));
    TEST_ASSERT_EQUAL_STRING("", numfmt::fixed(b, sizeof(b), 1e18, 6));
}

void test_truncation_never_yields_a_partial_number() {
    char b[8];
    // "12345.50" needs 9 bytes: refuse rather than send a shortened number that
    // a handset would read as a real depth.
    TEST_ASSERT_EQUAL_STRING("", numfmt::fixed(b, sizeof(b), 12345.5, 2));
    // "1234.50" needs exactly 8: the boundary must succeed, not refuse.
    TEST_ASSERT_EQUAL_STRING("1234.50", numfmt::fixed(b, sizeof(b), 1234.5, 2));
    // Which way a "looks like a tie" value goes is decided by the stored bits,
    // not by the decimal spelling: 12.345 is stored just ABOVE the tie (-> 12.35)
    // and 2.675 just below it (-> 2.67). Both pinned so a future
    // "simplification" of the rounding cannot quietly move a wire byte.
    TEST_ASSERT_EQUAL_STRING("12.35", numfmt::fixed(b, sizeof(b), 12.345, 2));
    TEST_ASSERT_EQUAL_STRING("2.67", numfmt::fixed(b, sizeof(b), 2.675, 2));
    // cap 0 is a caller error: write nothing at all, and say nothing.
    char untouched[4] = {'x', 'x', 'x', '\0'};
    TEST_ASSERT_EQUAL_STRING("xxx", numfmt::fixed(untouched, 0, 1.0, 2));
    // a caller that passes exactly the bytes it needs must succeed
    char six[6];
    TEST_ASSERT_EQUAL_STRING("12.35", numfmt::fixed(six, sizeof(six), 12.35, 2));
}

void test_scaled_integers_match_the_divide_they_replace() {
    // The dive log stores cm and thousandths. Going straight from the integer
    // must print the same bytes as `X / 100.0` through %.2f did, for every value
    // the log can hold — and never disagree about a sign.
    for (int cm = -20000; cm <= 20000; ++cm) {
        char mine[32], theirs[32];
        numfmt::fixedScaled(mine, sizeof(mine), cm, 2);
        std::snprintf(theirs, sizeof(theirs), "%.2f", cm / 100.0);
        TEST_ASSERT_EQUAL_STRING_MESSAGE(theirs, mine, "depthCm round-trip");
    }
    for (int x = -5000; x <= 5000; ++x) {
        char mine[32], theirs[32];
        numfmt::fixedScaled(mine, sizeof(mine), x, 3);
        std::snprintf(theirs, sizeof(theirs), "%.3f", x / 1000.0);
        TEST_ASSERT_EQUAL_STRING_MESSAGE(theirs, mine, "X1000 round-trip");
    }
    // the hand-rolled "%d.%02d" in dive_store.cpp printed this as "-5.-30"
    char b[16];
    TEST_ASSERT_EQUAL_STRING("-5.30", numfmt::fixedScaled(b, sizeof(b), -530, 2));
    TEST_ASSERT_EQUAL_STRING("0.05", numfmt::fixedScaled(b, sizeof(b), 5, 2));
    TEST_ASSERT_EQUAL_STRING("-0.05", numfmt::fixedScaled(b, sizeof(b), -5, 2));
    TEST_ASSERT_EQUAL_STRING("0.00", numfmt::fixedScaled(b, sizeof(b), 0, 2));
}

void test_decimals_are_clamped_not_crashed() {
    char b[32];
    TEST_ASSERT_EQUAL_STRING("1", numfmt::fixed(b, sizeof(b), 1.4, -3));
    TEST_ASSERT_EQUAL_STRING("1.400000", numfmt::fixed(b, sizeof(b), 1.4, 99));
}

void test_awkward_step_values() {
    // not aligned to the decimal grid: these are what a real sensor chain
    // actually produces after voting, not the neat values above
    for (int i = 0; i < 4000; ++i) {
        const double v = 0.0001 * i * 7.0 / 3.0;   // irrational-ish steps
        CK(v, 3);
        CK(v, 2);
        CK(-v, 1);
    }
}

// ---- scaled(): the one way a physical value becomes a stored integer -------

namespace {

struct StoreCase { float v; int decimals; long long want; };

// Every `want` is hand-worked: the count of thousandths (or hundredths) a diver
// reads off the same value on the wire. Computed expectations would only prove
// the helper agrees with itself.
const StoreCase kStore[] = {
    // ppO2, setpoint, cells: thousandths — stored scaled(v,3), printed %.3f
    {0.0f, 3, 0},      {0.01f, 3, 10},     {0.05f, 3, 50},    {0.10f, 3, 100},
    {0.15f, 3, 150},   {0.21f, 3, 210},    {0.25f, 3, 250},   {0.29f, 3, 290},
    {0.30f, 3, 300},   {0.35f, 3, 350},    {0.40f, 3, 400},   {0.45f, 3, 450},
    {0.50f, 3, 500},   {0.55f, 3, 550},    {0.60f, 3, 600},   {0.66f, 3, 660},
    {0.70f, 3, 700},   {0.75f, 3, 750},    {0.80f, 3, 800},   {1.60f, 3, 1600},
    {0.0004f, 3, 0},   {0.0006f, 3, 1},
    // depth and ceiling: centimetres — stored scaled(v,2), printed %.2f
    {0.0f, 2, 0},      {0.29f, 2, 29},     {2.53f, 2, 253},   {5.07f, 2, 507},
    {11.15f, 2, 1115}, {20.30f, 2, 2030},  {40.0f, 2, 4000},  {99.99f, 2, 9999},
    {-0.29f, 2, -29},  {-11.15f, 2, -1115}, {-0.004f, 2, 0},
    // battery: millivolts
    {11.9f, 3, 11900}, {12.6f, 3, 12600},  {16.8f, 3, 16800},
    // whole counts: gf pairs, CNS %
    {35.0f, 0, 35},    {12.9f, 0, 13},     {12.1f, 0, 12},
};

// Values the renderer needs fma() to get right — the stored integer has to land
// on the same side of the tie that fixed() prints.
const double kAwkward[] = {0.0105, 1.005, 2.675, 0.125, 0.375, 0.005, 0.55, 1.305};

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

} // namespace

void test_scaled_lands_on_the_count_the_diver_reads() {
    for (const StoreCase& c : kStore) {
        const long long got = numfmt::scaled(c.v, c.decimals);
        char msg[128];
        std::snprintf(msg, sizeof(msg), "scaled(%.4f, %d) -> %lld, want %lld",
                      static_cast<double>(c.v), c.decimals, got, c.want);
        TEST_ASSERT_EQUAL_INT64_MESSAGE(c.want, got, msg);
    }
    TEST_ASSERT_GREATER_OR_EQUAL_INT_MESSAGE(30, static_cast<int>(sizeof(kStore) / sizeof(kStore[0])),
                                             "the storage table shrank below the point of the case");
}

void test_a_bare_cast_is_where_the_dive_file_went_wrong() {
    // Pinned, not illustrative: these are the expressions the firmware used, and
    // the numbers it therefore wrote on 2026-11-24. If someone "simplifies" the
    // helper back into a cast, this is the case that says what the cast does.
    const float depth = 20.30f;
    TEST_ASSERT_EQUAL_INT_MESSAGE(2029, static_cast<int>(static_cast<int32_t>(depth * 100.0f)),
                                  "(int32_t)(depth * 100.0f) no longer loses the count");
    TEST_ASSERT_EQUAL_INT64_MESSAGE(2030, numfmt::scaled(depth, 2),
                                    "the helper must store 2030 cm for a depth printed 20.30");

    const float ceiling = 11.15f;
    TEST_ASSERT_EQUAL_INT_MESSAGE(1114, static_cast<int>(static_cast<int16_t>(ceiling * 100.0)),
                                  "a DOUBLE literal used to promote the float and land low");
    TEST_ASSERT_EQUAL_INT64_MESSAGE(1115, numfmt::scaled(ceiling, 2),
                                    "this bench's measured ceiling of 11.15 m must store 1115 cm");

    const float vote = 0.29f;
    TEST_ASSERT_EQUAL_INT_MESSAGE(289, static_cast<int>(static_cast<int16_t>(vote * 1000.0)),
                                  "0.29 bar through a double multiply stored 289 thousandths");
    TEST_ASSERT_EQUAL_INT64_MESSAGE(290, numfmt::scaled(vote, 3),
                                    "and the handset was answered 0.290 for the same sample");

    // And the nuance that let this survive review: a FLOAT multiply re-rounds the
    // product to the nearest float, which usually lands back on the integer, so
    // 0.15f * 1000.0f is exactly 150.0f and the old cast looked fine. The double
    // literal above is what actually broke, on most decimals, and the float path
    // still fails often enough (20.30 m) that "it works on the values I tried"
    // was never a safe answer. Both halves are pinned here so nobody re-derives
    // it: the float multiply is safe for 0.15, and unsafe for 20.30.
    const float sp = 0.15f;
    TEST_ASSERT_EQUAL_INT_MESSAGE(150, static_cast<int>(static_cast<int16_t>(sp * 1000.0f)),
                                  "a float multiply re-rounds: 0.15 was never a miss");
}

void test_stored_and_printed_never_disagree() {
    // The invariant the whole helper exists for: storing then rendering must give
    // the bytes the handset was answered for the same sample. Two rounding rules
    // in one file is how a digit gets between them.
    int compared = 0;
    int neg_zero = 0;
    for (const StoreCase& c : kStore) {
        const long long sc = numfmt::scaled(c.v, c.decimals);
        char printed[64], round_tripped[64];
        numfmt::fixed(printed, sizeof(printed), c.v, c.decimals);
        numfmt::fixedScaled(round_tripped, sizeof(round_tripped), sc, c.decimals);
        if (sc == 0 && std::signbit(c.v)) {
            // printf spells a small negative "-0.00" and there is no integer for
            // that. Counted, not compared — and the count is asserted, so this
            // branch cannot quietly become the whole test.
            ++neg_zero;
            continue;
        }
        char msg[160];
        std::snprintf(msg, sizeof(msg), "%.4f at %d decimals: fixed '%s' vs stored-then-printed '%s'",
                      static_cast<double>(c.v), c.decimals, printed, round_tripped);
        TEST_ASSERT_EQUAL_STRING_MESSAGE(printed, round_tripped, msg);
        ++compared;
    }
    for (double v : kAwkward) {
        for (int d = 2; d <= 4; ++d) {
            const long long sc = numfmt::scaled(v, d);
            char printed[64], round_tripped[64];
            numfmt::fixed(printed, sizeof(printed), v, d);
            numfmt::fixedScaled(round_tripped, sizeof(round_tripped), sc, d);
            TEST_ASSERT_EQUAL_STRING_MESSAGE(printed, round_tripped,
                                             "an awkward value's stored count disagrees with its printed one");
            ++compared;
        }
    }
    TEST_ASSERT_GREATER_OR_EQUAL_INT_MESSAGE(40, compared,
                                             "the agreement walk compared too few values to mean anything");
    TEST_ASSERT_GREATER_OR_EQUAL_INT_MESSAGE(1, neg_zero,
                                             "no negative-rounds-to-zero case reached the walk: "
                                             "the excluded branch is untested");
}

void test_ties_go_to_the_even_side_like_the_renderer() {
    // NOT lround(): the renderer's rule is nearest-with-ties-to-even, because it
    // matches printf, so the storage helper has to use the same rule or the pair
    // differs on a tie. 0.5 -> 0, 1.5 -> 2, 2.5 -> 2 (lround gives 1, 2, 3).
    TEST_ASSERT_EQUAL_INT64_MESSAGE(0, numfmt::scaled(0.5, 0), "0.5 ties to the even 0");
    TEST_ASSERT_EQUAL_INT64_MESSAGE(2, numfmt::scaled(1.5, 0), "1.5 ties to the even 2");
    TEST_ASSERT_EQUAL_INT64_MESSAGE(2, numfmt::scaled(2.5, 0), "2.5 ties to the even 2");
    TEST_ASSERT_EQUAL_INT64_MESSAGE(-2, numfmt::scaled(-2.5, 0), "the sign comes back");
    // A scaled tie at the storage step: 0.125 at 2 decimals is "0.12" on the wire,
    // so the stored count must be 12, not 13.
    TEST_ASSERT_EQUAL_INT64_MESSAGE(12, numfmt::scaled(0.125, 2), "0.125 stores 12 hundredths");
    TEST_ASSERT_EQUAL_INT64_MESSAGE(38, numfmt::scaled(0.375, 2), "0.375 stores 38 hundredths");
}

void test_scaled_refuses_rather_than_wrapping() {
    // Documented contract: 0 for a value that cannot be held, never a plausible
    // wrap. These are not expected inputs — depth, bar and volts are far inside
    // range — but a sensor fault must not become a plausible wrong dive row.
    TEST_ASSERT_EQUAL_INT64(0, numfmt::scaled(std::numeric_limits<double>::quiet_NaN(), 3));
    TEST_ASSERT_EQUAL_INT64(0, numfmt::scaled(std::numeric_limits<double>::infinity(), 3));
    TEST_ASSERT_EQUAL_INT64(0, numfmt::scaled(1e30, 3));
    // Out-of-range decimals clamp the same way fixed() does rather than overflow.
    TEST_ASSERT_EQUAL_INT64(290, numfmt::scaled(0.29f, 3));
    TEST_ASSERT_EQUAL_INT64(0, numfmt::scaled(0.0, 99));
}

// scaled() answers long long, and the log fields are int16 and uint16. Between
// the two sits a narrowing that a plain cast gets wrong: out of range it WRAPS,
// and for the deco picture the wrap points the wrong way for a diver. Reproduced
// on host 2026-11-27 with every tissue seeded at kSeedMaxBar (100 bar) and the
// CNS clock at its cap: the engine's ceiling is 95336 cm, and the int16 column
// the wire and the dive file print held 29800 — still a number, still plausible,
// and about 3 m short of the truth a client reads as "you may ascend".
//
// Saturation is the only narrowing that cannot make an obligation disappear: the
// field's own maximum keeps the diver MORE obligated than the truth, and is at
// least recognisable as the limit of the column rather than a reading from a
// different dive.
void test_scaledInto_saturates_where_a_cast_wraps() {
    const double ceilingM = 953.36;  // the probe's worst case, in metres
    TEST_ASSERT_EQUAL_INT64(95336, numfmt::scaled(ceilingM, 2));
    // The wrap, pinned as the thing to prevent: 95336 - 65536 = 29800.
    TEST_ASSERT_EQUAL_INT16_MESSAGE(29800, static_cast<int16_t>(numfmt::scaled(ceilingM, 2)),
                                    "the cast no longer wraps past int16 — if this changes, "
                                    "the number the helper below protects against changed too");
    TEST_ASSERT_EQUAL_INT16_MESSAGE(std::numeric_limits<int16_t>::max(),
                                    numfmt::scaledInto<int16_t>(ceilingM, 2),
                                    "an unrepresentable ceiling must saturate at the field's "
                                    "maximum: a smaller number is a shorter stop");

    // The negative direction saturates low rather than wrapping high. No field
    // here should be negative, so the wrap it prevents would be an implausibly
    // LARGE positive number — which is what a wrapped spread or battery looks
    // like, and it reads as a dead sensor rather than a broken record.
    TEST_ASSERT_EQUAL_INT16(std::numeric_limits<int16_t>::min(),
                            numfmt::scaledInto<int16_t>(-400.0, 2));
    TEST_ASSERT_EQUAL_INT16(30000, numfmt::scaledInto<int16_t>(300.0, 2));  // in range: exact

    // Unsigned columns wrap into the wrong magnitude in the other direction:
    // 70.000 V of battery is 70000 counts, and a cast hands the reader 4464 mV
    // — a flat pack in a dive file that has to be comparable with the wire.
    TEST_ASSERT_EQUAL_UINT16(std::numeric_limits<uint16_t>::max(),
                             numfmt::scaledInto<uint16_t>(70.0, 3));
    TEST_ASSERT_EQUAL_UINT16(4464, static_cast<uint16_t>(numfmt::scaled(70.0, 3)));
    TEST_ASSERT_EQUAL_UINT16(12345, numfmt::scaledInto<uint16_t>(12.345, 3));

    // A uint8 column is where a deco figure is most likely to be forgotten: the
    // GF settings. The schema will not let a legal value reach the limit, which
    // is exactly why the guard belongs in the conversion, not in the schema.
    TEST_ASSERT_EQUAL_UINT8(std::numeric_limits<uint8_t>::max(),
                            numfmt::scaledInto<uint8_t>(999.0, 0));

    // 32-bit fields narrow too (depth and ppO2 in the row fill and the totals):
    // a long long is wider, so the same rule applies one level up.
    TEST_ASSERT_EQUAL_INT32(std::numeric_limits<int32_t>::max(),
                            numfmt::scaledInto<int32_t>(3e7, 2));
    TEST_ASSERT_EQUAL_INT32(-2000, numfmt::scaledInto<int32_t>(-20.0, 2));

    // A non-finite value takes scaled()'s refusal (0), which is a claim of
    // "nothing", not a saturated maximum. The engine gates its inputs
    // (setTissueState rejects non-finite, schedule() is bounded twice), so
    // nothing that reaches here is a NaN; the behaviour is recorded so a future
    // caller cannot mistake the 0 for a saturated value.
    TEST_ASSERT_EQUAL_INT16(0, numfmt::scaledInto<int16_t>(
                                  std::numeric_limits<double>::quiet_NaN(), 2));
}

// The integer form of the same rule: the dive summary's extremes and counters
// accumulate in wider types than the columns they land in.
void test_narrow_saturates_where_a_cast_wraps() {
    TEST_ASSERT_EQUAL_INT16(std::numeric_limits<int16_t>::max(), numfmt::narrow<int16_t>(40000));
    TEST_ASSERT_EQUAL_INT16(std::numeric_limits<int16_t>::min(), numfmt::narrow<int16_t>(-40000));
    TEST_ASSERT_EQUAL_UINT16(std::numeric_limits<uint16_t>::max(), numfmt::narrow<uint16_t>(70000));
    // The pulses counter is the case this exists for: uint32 on the chain, uint16
    // in the file's `totalPulses`. A dive that injected 70000 counts wrote 4464 —
    // a plausible number from a shorter dive, not an absence.
    TEST_ASSERT_EQUAL_UINT16(4464, static_cast<uint16_t>(70000u));
    TEST_ASSERT_EQUAL_UINT16(1234, numfmt::narrow<uint16_t>(1234));
    TEST_ASSERT_EQUAL_UINT8(std::numeric_limits<uint8_t>::max(), numfmt::narrow<uint8_t>(300));
}

void test_stored_values_go_through_the_helper() {
    // The structural claim, enforced on the text — the same trick that keeps one
    // depth formula in one file. main.cpp and session_wiring.cpp are not
    // host-compiled, so a scan is the only way the native suite can see them, and
    // a scan that shrugs when a file will not open is decoration: a missing file
    // fails loudly.
    //
    // 2026-11-25: session_wiring.cpp left this list when its row fill moved into
    // src/log/rowfill.cpp. It no longer stores a scaled value at all — it maps raw
    // scalars onto logfmt::RowInputs — and the file that now does the storing is
    // host-compiled and executed by test_logfmt, so the scan is now the weaker of
    // two checks over that one file. It stays scanned because the rule is about
    // the NEXT field someone adds, wherever they add it.
    //
    // 2026-11-27: src/deco/view.h joined the list (it stores the ceiling, the CNS
    // clock and the deco plan the wire and the dive file print), and the rule got
    // a second half: a scaled value must be NARROWED by scaledInto, never cast.
    // The fields these files store are int16 and uint16, numfmt::scaled answers in
    // long long, and a cast of a value past the field wraps — the dive file spent a
    // year one count away from the wire through exactly this shape, and a wrapped
    // ceiling reads as "no decompression obligation".
    const char* kUsers[] = {"src/main.cpp", "src/log/rowfill.cpp",
                            "src/comms/telemetry_bridge.cpp", "src/control/cellhist.cpp",
                            "src/deco/view.h"};
    int calls = 0;
    // Every narrowing that can wrap a scaled value, spelled out. `long long` and
    // `double` are absent on purpose: narrowing those is the helper's own business.
    const char* kNarrowingCasts[] = {"static_cast<int16_t>(numfmt::scaled",
                                     "static_cast<uint16_t>(numfmt::scaled",
                                     "static_cast<int8_t>(numfmt::scaled",
                                     "static_cast<uint8_t>(numfmt::scaled",
                                     "static_cast<int32_t>(numfmt::scaled",
                                     "static_cast<uint32_t>(numfmt::scaled",
                                     "static_cast<unsigned>(numfmt::scaled"};
    for (const char* path : kUsers) {
        const std::string src = readFile(path);
        TEST_ASSERT_MESSAGE(!src.empty(),
                            (std::string("cannot read ") + path +
                             " — run the native tests from the project root").c_str());
        TEST_ASSERT_NOT_NULL_MESSAGE(std::strstr(src.c_str(), "numfmt::scaled"),
                                     (std::string(path) + " stores a scaled value without "
                                      "util/numfmt.h").c_str());

        // No hand-rolled scaling or rounding left in a file that stores values:
        // no `* 100` / `* 1000` product and no `+ 0.5` tie-hack outside comments.
        // This is what catches the NEXT field, the one nobody has thought about.
        int lineno = 0;
        size_t pos = 0;
        while (pos <= src.size()) {
            size_t eol = src.find('\n', pos);
            const std::string line = src.substr(pos, eol == std::string::npos ? std::string::npos : eol - pos);
            ++lineno;
            const size_t first = line.find_first_not_of(" \t");
            const std::string t = (first == std::string::npos) ? std::string() : line.substr(first);
            // Full-line comments are exempt: explaining the old defect is allowed to
            // name it. Code with a trailing comment is not exempt, so a defect cannot
            // hide after a `;` on a line that starts with code.
            if (!t.empty() && t.rfind("//", 0) != 0) {
                for (const char* forbidden : {"* 100", "* 1000", "+ 0.5"}) {
                    TEST_ASSERT_NULL_MESSAGE(std::strstr(line.c_str(), forbidden),
                                             (std::string(path) + ":" + std::to_string(lineno) +
                                              " scales or rounds by hand (" + forbidden +
                                              "); go through numfmt::scaled").c_str());
                }
                for (const char* forbidden : kNarrowingCasts) {
                    TEST_ASSERT_NULL_MESSAGE(std::strstr(line.c_str(), forbidden),
                                             (std::string(path) + ":" + std::to_string(lineno) +
                                              " casts a scaled value into a narrower field, which "
                                              "wraps it; go through numfmt::scaledInto, which "
                                              "saturates").c_str());
                }
            }
            // Counted on the code line only: a comment that NAMES the helper is
            // not a call site, and a count that a comment can pad is a count that
            // cannot go below the threshold it exists to police.
            if (!t.empty() && t.rfind("//", 0) != 0) {
                for (const char* call : {"numfmt::scaled(", "numfmt::scaledInto"}) {
                    for (size_t at = line.find(call); at != std::string::npos;
                         at = line.find(call, at + 1))
                        ++calls;
                }
            }
            if (eol == std::string::npos) break;
            pos = eol + 1;
        }
    }
    // A scan is only as good as the number of sites it found.
    TEST_ASSERT_GREATER_OR_EQUAL_INT_MESSAGE(12, calls,
                                             "fewer than 12 numfmt::scaled/scaledInto call sites: "
                                             "someone removed the conversions the scan exists "
                                             "to police");

    // And there must still be exactly one rounding engine, used by both entry
    // points — otherwise "one rule" is a comment, not a fact.
    const std::string helper = readFile("src/util/numfmt.cpp");
    TEST_ASSERT_MESSAGE(!helper.empty(), "cannot read src/util/numfmt.cpp");
    int engine = 0;
    for (size_t at = helper.find("roundExact"); at != std::string::npos; at = helper.find("roundExact", at + 1))
        ++engine;
    TEST_ASSERT_GREATER_OR_EQUAL_INT_MESSAGE(3, engine,
                                             "numfmt.cpp no longer shares one rounding between "
                                             "fixed() and scaled()");
    for (const char* forbidden : {"lround", "std::round", "+ 0.5"}) {
        TEST_ASSERT_NULL_MESSAGE(std::strstr(helper.c_str(), forbidden),
                                 (std::string("util/numfmt.cpp grew a second rounding rule (") +
                                  forbidden + ")").c_str());
    }
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_ties_round_half_to_even);
    RUN_TEST(test_protocol_ranges);
    RUN_TEST(test_signed_zero_and_negatives);
    RUN_TEST(test_nonfinite_matches_strtod_vocabulary);
    RUN_TEST(test_too_large_is_refused_not_wrapped);
    RUN_TEST(test_truncation_never_yields_a_partial_number);
    RUN_TEST(test_scaled_integers_match_the_divide_they_replace);
    RUN_TEST(test_decimals_are_clamped_not_crashed);
    RUN_TEST(test_awkward_step_values);
    RUN_TEST(test_scaled_lands_on_the_count_the_diver_reads);
    RUN_TEST(test_a_bare_cast_is_where_the_dive_file_went_wrong);
    RUN_TEST(test_stored_and_printed_never_disagree);
    RUN_TEST(test_ties_go_to_the_even_side_like_the_renderer);
    RUN_TEST(test_scaled_refuses_rather_than_wrapping);
    RUN_TEST(test_scaledInto_saturates_where_a_cast_wraps);
    RUN_TEST(test_narrow_saturates_where_a_cast_wraps);
    RUN_TEST(test_stored_values_go_through_the_helper);
    return UNITY_END();
}
