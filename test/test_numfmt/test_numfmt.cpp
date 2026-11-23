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
    return UNITY_END();
}
