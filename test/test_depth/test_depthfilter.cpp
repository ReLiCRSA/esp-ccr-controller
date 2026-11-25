// Unit tests for the depth smoothing that feeds the SP bands and CellProof.
#include <unity.h>

#include "control/depthfilter.h"

using depth::Filter;

void setUp() {}
void tearDown() {}

void test_single_spike_is_invisible() {
    Filter<> f;
    float worst = 0.0f;
    for (int i = 0; i < 20; ++i) {
        const float raw = (i == 10) ? 30.0f : 10.0f;  // 20 m glitch, one sample
        const float v = f.update(raw);
        if (i >= 8) worst = v > 10.5f ? v - 10.0f : worst;  // settled region
    }
    TEST_ASSERT_EQUAL_FLOAT_MESSAGE(0.0f, worst, "one bad sample moved the depth");
}

void test_two_spikes_of_five_are_rejected() {
    Filter<5, 4> f;
    float peak = 0.0f;
    for (int i = 0; i < 20; ++i) {
        const float raw = (i % 5 == 0 || i % 5 == 1) ? 25.0f : 10.0f;
        const float v = f.update(raw);
        if (i >= 8 && v > peak) peak = v;
    }
    // median-of-5 with 2 of 5 high still returns the honest value
    TEST_ASSERT_LESS_THAN_FLOAT_MESSAGE(12.0f, peak, "median lost to a 2-of-5 spike burst");
}

void test_step_settles_quickly_and_exactly() {
    Filter<> f;
    for (int i = 0; i < 5; ++i) f.update(0.0f);
    float last = 0.0f;
    int settledAt = -1;
    for (int i = 0; i < 20 && settledAt < 0; ++i) {
        last = f.update(30.0f);
        if (last > 29.99f) settledAt = i;
    }
    TEST_ASSERT_GREATER_THAN_INT32_MESSAGE(-1, settledAt, "depth never settled on a step");
    TEST_ASSERT_LESS_OR_EQUAL_UINT8_MESSAGE(9, static_cast<uint8_t>(settledAt),
                                            "depth filter too slow for a setpoint band");
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.001f, 30.0f, last, "depth filter steady-state offset");
}

void test_threshold_crossed_once_during_descent() {
    // Band flapping test: a noisy descent through 30 m must cross the 30 m
    // threshold exactly once, not every sample.
    Filter<> f;
    int crossings = 0;
    bool above = false;
    float noise = 0.0f;
    for (int i = 0; i < 400; ++i) {
        noise = (i % 2) ? 0.15f : -0.15f;              // ±15 cm of sensor noise
        const float d = f.update(28.0f + i * 0.02f + noise);
        const bool nowAbove = d >= 30.0f;
        if (nowAbove != above) {
            above = nowAbove;
            if (above) crossings++;
        }
    }
    TEST_ASSERT_EQUAL_INT_MESSAGE(1, crossings, "30 m band boundary flapped");
}

// ---------------------------------------------------------------------------
// Validity: an unmeasured sample is not a sample (finding C1).
// The firmware used to deliver 0.0 m for "the depth bus is dead"; these pin the
// rule that replaced it, because the whole depth leg (setpoint bands, the
// handset readout, CellProof) is only as honest as this one branch.
// ---------------------------------------------------------------------------

// A dead bus at depth must make the depth UNKNOWN, not 0 m: the value holds,
// and the windows are never touched by the samples that were never taken.
void test_invalid_samples_hold_the_last_good_depth() {
    Filter<> f;
    for (int i = 0; i < 20; ++i) TEST_ASSERT_FLOAT_WITHIN(0.001f, 30.0f, f.update(30.0f));

    float worst = 30.0f;
    for (int i = 0; i < 200; ++i) {  // 20 s of a dead bus: 200 ticks of garbage
        const float v = f.update((i % 2) ? 0.0f : -999.0f, /*valid=*/false);
        if (v < worst) worst = v;
    }
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.001f, 30.0f, worst,
                                     "an invalid depth moved the filtered depth");
}

// The old failure, stated as an assertion: the median-of-5 + boxcar-of-4 used
// to converge on the fabricated 0.0 m within about half a second of ticks, and
// the setpoint engine followed it to the surface rung.
void test_invalid_burst_never_approaches_zero() {
    Filter<> f;
    for (int i = 0; i < 10; ++i) f.update(28.0f);
    float floorSeen = 999.0f;
    for (int i = 0; i < 5; ++i) {
        const float v = f.update(0.0f, /*valid=*/false);
        if (v < floorSeen) floorSeen = v;
    }
    TEST_ASSERT_GREATER_THAN_FLOAT_MESSAGE(27.0f, floorSeen,
                                           "held depth fell towards the surface");
}

// Recovery: the windows still hold the real samples, so the first believed
// sample after a fault continues from where the dive was — no re-warmup, and
// no dip through the held value.
void test_recovery_resumes_without_a_dip() {
    Filter<> f;
    for (int i = 0; i < 12; ++i) f.update(30.0f);
    for (int i = 0; i < 50; ++i) f.update(0.0f, /*valid=*/false);

    float first = f.update(30.0f, /*valid=*/true);
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.001f, 30.0f, first,
                                     "first good sample after a fault was not continuous");
    for (int i = 0; i < 10; ++i) f.update(30.0f, /*valid=*/true);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 30.0f, f.value());

    // and it tracks again: a real descent resumes after the fault
    for (int i = 0; i < 12; ++i) f.update(20.0f, /*valid=*/true);
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.001f, 20.0f, f.value(),
                                     "filter stopped tracking after a fault");
}

// Refusing a sample must not count as taking one: the filter may never report
// itself settled on data it never looked at.
void test_invalid_samples_do_not_fill_the_windows() {
    Filter<5, 4> f;
    for (int i = 0; i < 100; ++i) f.update(0.0f, /*valid=*/false);
    TEST_ASSERT_FALSE_MESSAGE(f.settled(), "filter settled on invalid samples");
    f.update(15.0f, /*valid=*/true);
    TEST_ASSERT_FALSE(f.settled());  // one real sample: still filling
    for (int i = 0; i < 7; ++i) f.update(15.0f, /*valid=*/true);
    TEST_ASSERT_TRUE(f.settled());
}

// The believed-by-default call shape stays available (simulator, tests that
// have no health flag to pass), and behaves exactly as the old single argument.
void test_single_argument_call_is_believed() {
    Filter<> f;
    for (int i = 0; i < 12; ++i) f.update(12.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 12.0f, f.value());
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_single_spike_is_invisible);
    RUN_TEST(test_two_spikes_of_five_are_rejected);
    RUN_TEST(test_step_settles_quickly_and_exactly);
    RUN_TEST(test_threshold_crossed_once_during_descent);
    RUN_TEST(test_invalid_samples_hold_the_last_good_depth);
    RUN_TEST(test_invalid_burst_never_approaches_zero);
    RUN_TEST(test_recovery_resumes_without_a_dip);
    RUN_TEST(test_invalid_samples_do_not_fill_the_windows);
    RUN_TEST(test_single_argument_call_is_believed);
    return UNITY_END();
}
