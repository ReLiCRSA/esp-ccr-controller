#include <unity.h>

#include "sensors/battery.h"

void setUp() {}
void tearDown() {}

static void burst(Battery& b, int mv) {
    for (int i = 0; i < 8; ++i) b.addSample(mv);
}

void test_divider_scaling_and_gate() {
    Battery b;
    // 1000 mV at the pin * 11.0 divider = 11.0 V: healthy.
    burst(b, 1000);
    TEST_ASSERT_TRUE(b.initialized());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 11.0f, b.volts());
    TEST_ASSERT_FALSE(b.low());

    // sag below VBAT_MIN_V (10.5) must gate.
    burst(b, 940);   // 10.34 V
    for (int i = 0; i < 40; ++i) burst(b, 940);
    TEST_ASSERT_TRUE(b.low());
}

void test_median_rejects_spike() {
    Battery b;
    // baseline 11.0 V
    burst(b, 1000);
    const float before = b.volts();

    // one monstrous spike inside an otherwise clean burst: median kills it
    const int samples[8] = {1000, 1000, 1000, 1000, 1000, 1000, 1000, 3300};
    for (int v : samples) b.addSample(v);
    TEST_ASSERT_FLOAT_WITHIN(0.3f, before, b.volts());
}

void test_ema_converges_monotonically() {
    Battery b;
    burst(b, 1200);                 // 13.2 V
    float prev = b.volts();
    for (int i = 0; i < 30; ++i) {
        burst(b, 950);              // step down to 10.45 V
        TEST_ASSERT_TRUE(b.volts() <= prev + 1e-4f);  // never jumps back up
        prev = b.volts();
    }
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 10.45f, prev);
    TEST_ASSERT_TRUE(b.low());
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_divider_scaling_and_gate);
    RUN_TEST(test_median_rejects_spike);
    RUN_TEST(test_ema_converges_monotonically);
    return UNITY_END();
}
