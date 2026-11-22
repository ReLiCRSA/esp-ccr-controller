#include <unity.h>

#include "control/setpoints.h"

using sp::Config;
using sp::Engine;

void setUp() {}
void tearDown() {}

void test_surface_start_and_crossovers() {
    Engine e{Config{}};
    auto r = e.update(0.5f);
    TEST_ASSERT_EQUAL_UINT8(0, r.index);
    TEST_ASSERT_FLOAT_WITHIN(0.001, 0.40f, r.value);

    r = e.update(3.0f); // past 2.0 + 0.5 hysteresis
    TEST_ASSERT_TRUE(r.changed);
    TEST_ASSERT_EQUAL_UINT8(1, r.index);
    TEST_ASSERT_FLOAT_WITHIN(0.001, 0.70f, r.value);

    r = e.update(31.0f); // past 30 + 0.5
    TEST_ASSERT_EQUAL_UINT8(2, r.index);
    r = e.update(51.0f); // past 50 + 0.5
    TEST_ASSERT_EQUAL_UINT8(3, r.index);
    TEST_ASSERT_FLOAT_WITHIN(0.001, 1.30f, r.value);
}

void test_hysteresis_no_flapping() {
    Engine e{Config{}};
    e.update(1.0f);
    // oscillate around 2.0 m within the hysteresis band: never leaves SP1
    for (float d = 1.8f; d <= 2.4f; d += 0.1f) {
        const auto r = e.update(d);
        TEST_ASSERT_EQUAL_UINT8(0, r.index);
    }
    // climb back down from deep inside SP2: needs 2.0-0.5 to drop to SP1
    e.update(2.6f);
    TEST_ASSERT_EQUAL_UINT8(1, e.update(2.6f).index);
    TEST_ASSERT_EQUAL_UINT8(1, e.update(1.7f).index); // 1.7 > 1.5: still SP2
    TEST_ASSERT_EQUAL_UINT8(0, e.update(1.4f).index);
}

void test_clamp_layer() {
    Engine e{Config{}};
    e.update(40.0f);
    e.update(40.0f);
    auto r = e.update(40.0f);
    TEST_ASSERT_EQUAL_UINT8(2, r.index);
    TEST_ASSERT_FLOAT_WITHIN(0.001, 1.00f, r.value);
    TEST_ASSERT_FALSE(r.clamped);

    e.setClamp(0.90f); // CellProof failed mid-dive
    r = e.update(40.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.001, 1.00f, r.nominal);
    TEST_ASSERT_FLOAT_WITHIN(0.001, 0.90f, r.value);
    TEST_ASSERT_TRUE(r.clamped);

    // clamp must NOT lift shallower bands (0.70 < 0.90 untouched)
    r = e.update(10.0f);
    TEST_ASSERT_EQUAL_UINT8(1, r.index);
    TEST_ASSERT_FALSE(r.clamped);

    e.setClamp(0.0f); // dive end / diver override
    r = e.update(40.0f);
    r = e.update(40.0f);
    TEST_ASSERT_FALSE(r.clamped);
    TEST_ASSERT_FLOAT_WITHIN(0.001, 1.00f, r.value);
}

void test_reconfigured_table_takes_effect() {
    Config c{};
    c.sp[1] = 0.50f; c.deep1M = 20.0f;
    Engine e{c};
    e.update(21.0f);
    const auto r = e.update(21.0f);
    TEST_ASSERT_EQUAL_UINT8(2, r.index);
    TEST_ASSERT_FLOAT_WITHIN(0.001, 1.00f, r.value);
    e.update(10.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.001, 0.50f, e.update(10.0f).value);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_surface_start_and_crossovers);
    RUN_TEST(test_hysteresis_no_flapping);
    RUN_TEST(test_clamp_layer);
    RUN_TEST(test_reconfigured_table_takes_effect);
    return UNITY_END();
}
