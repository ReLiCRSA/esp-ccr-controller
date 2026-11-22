// ============================================================================
// test_cellhist — the histogram page's data source. The failure mode this
// guards is a SHAPE that lies: a stale window counted forever, a slot reused
// without clearing (the ten-minute horizon becomes "whatever the ring holds"),
// or a NaN poisoning a bin. All of those render as a plausible chart.
// ============================================================================
#include <unity.h>

#include <cmath>
#include <cstring>

#include "control/cellhist.h"

using cellhist::Hist;
using cellhist::kBins;
using cellhist::kWindows;
using cellhist::kWindowS;

void setUp() {}
void tearDown() {}

namespace {
uint16_t bins[kBins];
constexpr uint32_t winMs = kWindowS * 1000u;
}

void test_bins_edges_and_rejects() {
    Hist h;
    h.sample(0, 0.00f, 0);
    h.sample(0, 0.049f, 0);   // still bin 0
    h.sample(0, 0.05f, 0);    // bin 1
    TEST_ASSERT_EQUAL_UINT32(3, h.fold(0, bins));
    TEST_ASSERT_EQUAL_UINT16(2, bins[0]);
    TEST_ASSERT_EQUAL_UINT16(1, bins[1]);

    h.sample(0, 1.95f, 1000);  // past the top: stacks in the last bin, never lost
    h.fold(0, bins);
    TEST_ASSERT_EQUAL_UINT16(1, bins[kBins - 1]);

    const uint32_t before = h.samples(0);
    h.sample(0, -0.1f, 1000);
    h.sample(0, NAN, 1000);   // NaN fails >=0: the shape stays clean
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(before, h.samples(0),
                                     "a rejected sample still counted toward the total");
}

// The fold is the shape of the last 10 minutes OF DATA, not of wall clock:
// a quiet period must not silently erase the evidence, but the age field has
// to admit how old the shape is.
void test_a_quiet_period_ages_the_shape_without_erasing_it() {
    Hist h;
    for (int i = 0; i < 100; i++) h.sample(0, 1.00f, 0);  // all in window 0
    TEST_ASSERT_EQUAL_UINT32(100, h.fold(0, bins));

    const uint32_t later = (kWindows + 2) * winMs;
    TEST_ASSERT_EQUAL_UINT32(100, h.fold(0, bins));  // shape kept...
    TEST_ASSERT_EQUAL_UINT32(later / 1000u, h.horizonAgeS(later));  // ...age honest
}

// A steady 10 Hz feed for 15 min must fold exactly the last 10 windows —
// not 15 (ring too generous) and not 9 (off-by-one on the horizon).
void test_rolling_window_keeps_exactly_the_horizon() {
    Hist h;
    for (uint32_t t = 0; t < 15u * winMs; t += 100)
        h.sample(0, 0.55f, t);
    const uint32_t n = h.fold(0, bins);
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(kWindows * kWindowS * 10u, n,
                                     "horizon is not exactly windows x windowS");
    TEST_ASSERT_EQUAL_UINT16(n, bins[11]);  // 0.55 / 0.05
}

void test_slot_reuse_clears_the_ghost() {
    Hist h;
    h.sample(1, 1.00f, 0);                        // window 0, slot 0
    h.sample(1, 0.50f, kWindows * winMs);         // window 10 reuses slot 0
    TEST_ASSERT_EQUAL_UINT32(1, h.fold(1, bins));
    TEST_ASSERT_EQUAL_UINT16(0, bins[20]);        // ghost gone, not added
    TEST_ASSERT_EQUAL_UINT16(1, bins[10]);
}

void test_format_lines_are_whole_lines_or_nothing() {
    Hist h;
    h.sample(0, 0.60f, 5000);
    h.sample(2, 0.21f, 5000);
    char buf[1600];
    const size_t n = cellhist::formatLines(h, 65000, buf, sizeof(buf));
    TEST_ASSERT_TRUE_MESSAGE(n > 3 * kBins, "histogram lines came out truncated");
    buf[n] = '\0';
    TEST_ASSERT_NOT_NULL(std::strstr(buf, "HIST,0,"));
    TEST_ASSERT_NOT_NULL(std::strstr(buf, "HIST,1,"));
    TEST_ASSERT_NOT_NULL(std::strstr(buf, "HIST,2,"));
    // field shape: 7 header fields + kBins counts, i.e. 7+38 commas per line
    const char* line0 = buf;
    const char* lineEnd = std::strchr(line0, '\n');
    TEST_ASSERT_NOT_NULL(lineEnd);
    uint16_t fields = 1;
    for (const char* p = line0; p < lineEnd; p++)
        if (*p == ',') fields++;
    TEST_ASSERT_EQUAL_UINT16(7 + kBins, fields);  // 7 header fields + 38 counts
    // the folded count field must match what fold() reports
    TEST_ASSERT_EQUAL_UINT32(1, h.fold(0, bins));
    TEST_ASSERT_NOT_NULL(std::strstr(line0, "HIST,0,60,10,5,65,1,"));

    // a too-small cap emits nothing at all (a half histogram reads as a shape)
    TEST_ASSERT_EQUAL_UINT32(0, cellhist::formatLines(h, 65000, buf, 64));
}

void test_clear_and_age() {
    Hist h;
    h.sample(0, 0.5f, 300000);              // mid window 5
    TEST_ASSERT_EQUAL_UINT32(30, h.horizonAgeS(330000));
    h.clear();
    TEST_ASSERT_EQUAL_UINT32(0, h.fold(0, bins));
    TEST_ASSERT_EQUAL_UINT32(0, h.samples(0));
    TEST_ASSERT_EQUAL_UINT32(0, h.horizonAgeS(400000));
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_bins_edges_and_rejects);
    RUN_TEST(test_a_quiet_period_ages_the_shape_without_erasing_it);
    RUN_TEST(test_rolling_window_keeps_exactly_the_horizon);
    RUN_TEST(test_slot_reuse_clears_the_ghost);
    RUN_TEST(test_format_lines_are_whole_lines_or_nothing);
    RUN_TEST(test_clear_and_age);
    return UNITY_END();
}
