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

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_single_spike_is_invisible);
    RUN_TEST(test_two_spikes_of_five_are_rejected);
    RUN_TEST(test_step_settles_quickly_and_exactly);
    RUN_TEST(test_threshold_crossed_once_during_descent);
    return UNITY_END();
}
