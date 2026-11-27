#include <unity.h>

#include <cmath>

#include "sensors/battery.h"

void setUp() {}
void tearDown() {}

// Everything fed to this class is millivolts AT THE PIN, because that is what
// the caller hands in (analogReadMilliVolts). The pack volts are what comes out
// after the divider, so the helper converts in the same direction the firmware
// does and the fixtures below can be written as pack volts — the thing the
// thresholds are about.
//
// The numbers are derived from the chemistry block in defaults.h rather than
// hardcoded, for the same reason the thresholds are: the pack in this project
// changed once already (a 12 V-style floor sat in defaults.h while the rig ran a
// 3.7 V cell), and fixtures frozen at the old rail are how that mismatch hid.
// VBAT_CELLS is the one line that moves, so these legs must still mean
// something after it moves — see test_chemistry_derived_fixtures_still_mean_the_same_thing.
static constexpr int pinMv(float packVolts) {
    return static_cast<int>(std::lround(packVolts * 1000.0f / VBAT_DIVIDER_RATIO));
}

static void burst(Battery& b, int mv) {
    for (int i = 0; i < 8; ++i) b.addSample(mv);
}

// A charged pack: half a cell-volt above the floor, and neither fault.
void test_divider_scaling_and_gate() {
    Battery b;
    const float good = VBAT_MIN_V + 0.5f;
    burst(b, pinMv(good));
    TEST_ASSERT_TRUE(b.initialized());
    TEST_ASSERT_FLOAT_WITHIN(0.02f, good, b.volts());
    TEST_ASSERT_FALSE(b.low());
    TEST_ASSERT_FALSE(b.implausible());

    // sag below the floor must gate, and must still be called what it is: flat,
    // not blind.
    const float flat = VBAT_MIN_V - 0.30f;
    for (int i = 0; i < 40; ++i) burst(b, pinMv(flat));
    TEST_ASSERT_TRUE(b.low());
    TEST_ASSERT_FALSE(b.implausible());
}

// The reading's upper bound is the fail-closed half of this class, and it is the
// half that was missing: a gate that only asks "above the floor?" is PASSED by a
// divider that fell open and left the ADC sitting on its rail. vbat=34.71 was
// read that way on this bench, and pre-dive called it healthy.
void test_implausible_reading_is_never_healthy() {
    Battery b;
    for (int i = 0; i < 40; ++i) burst(b, pinMv(VBAT_MIN_V + 3.0f));
    TEST_ASSERT_TRUE_MESSAGE(b.implausible(), "a pack volts this high is not this chemistry");
    TEST_ASSERT_TRUE_MESSAGE(b.low(), "an unreadable pack must not read as healthy");

    // The boundary, from both sides: the ceiling is a fault detector, not a
    // charge limit, so the top of a real charge must pass and a reading just
    // above it must not. 20 mV of margin on each side, because one LSB at the
    // pin is VBAT_DIVIDER_RATIO mV at the pack — a tighter gap would be testing
    // the rounding of this fixture, not the bound.
    Battery at;
    for (int i = 0; i < 40; ++i) burst(at, pinMv(VBAT_IMPLAUSIBLE_V - 0.02f));
    TEST_ASSERT_FALSE(at.implausible());
    TEST_ASSERT_FALSE(at.low());

    Battery over;
    for (int i = 0; i < 40; ++i) burst(over, pinMv(VBAT_IMPLAUSIBLE_V + 0.02f));
    TEST_ASSERT_TRUE(over.implausible());
    TEST_ASSERT_TRUE(over.low());
}

// Two thresholds for one battery is two answers to one question. `VBAT_MIN` is
// writable, and the alarm/pre-dive path read the written value while HB's
// `batlow` and the HUD warning read the compile-time one — so a diver's own
// setting could disagree with the board's own warning light.
void test_written_floor_is_the_floor_the_class_uses() {
    Battery b;
    const float good = VBAT_MIN_V + 0.5f;
    for (int i = 0; i < 40; ++i) burst(b, pinMv(good));
    TEST_ASSERT_FALSE(b.low());

    const float raised = VBAT_MIN_V + 0.60f;
    b.setMinVolts(raised);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, raised, b.minVolts());
    TEST_ASSERT_TRUE_MESSAGE(b.low(), "a written floor above the pack must gate");

    b.setMinVolts(VBAT_MIN_V);
    TEST_ASSERT_FALSE_MESSAGE(b.low(), "and a floor written back down must release it");
}

void test_median_rejects_spike() {
    Battery b;
    const float good = VBAT_MIN_V + 0.5f;
    burst(b, pinMv(good));
    const float before = b.volts();

    // one monstrous spike inside an otherwise clean burst: median kills it
    const int spike = pinMv(VBAT_MIN_V + 3.0f);
    const int samples[8] = {pinMv(good), pinMv(good), pinMv(good), pinMv(good),
                            pinMv(good), pinMv(good), pinMv(good), spike};
    for (int v : samples) b.addSample(v);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, before, b.volts());
}

void test_ema_converges_monotonically() {
    Battery b;
    const float start = VBAT_MIN_V + 0.5f;
    burst(b, pinMv(start));
    const float v0 = b.volts();
    const float sag = VBAT_MIN_V - 0.10f;
    burst(b, pinMv(sag));
    const float first = b.volts();
    TEST_ASSERT_TRUE(first < v0);
    TEST_ASSERT_TRUE(first > sag);   // one burst does not teleport the reading
    for (int i = 0; i < 60; ++i) burst(b, pinMv(sag));
    TEST_ASSERT_FLOAT_WITHIN(0.02f, sag, b.volts());
}

void test_uninitialised_is_not_low() {
    // `low()` before the first burst must be false: the alarm and the pre-dive
    // gate would otherwise fire at boot from a value that was never measured.
    Battery b;
    TEST_ASSERT_FALSE(b.low());
    TEST_ASSERT_FALSE(b.implausible());
}

// The promise made in defaults.h is that VBAT_CELLS is the only line a series
// pack changes. This leg is the witness, and it is the one that fails if anyone
// hardcodes a threshold anywhere in this class: it runs the SAME relative
// fixture — half a volt above the floor, then the ceiling — against whatever the
// chemistry says, so a 1-cell and a 2-cell build must answer identically.
void test_chemistry_derived_fixtures_still_mean_the_same_thing() {
    TEST_ASSERT_FLOAT_WITHIN(0.001f, VBAT_CELLS * VBAT_CELL_MIN_V, VBAT_MIN_V);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, VBAT_CELLS * VBAT_CELL_IMPLAUSIBLE_V, VBAT_IMPLAUSIBLE_V);
    TEST_ASSERT_TRUE_MESSAGE(VBAT_MIN_V < VBAT_IMPLAUSIBLE_V,
                             "a floor at or above the ceiling makes every reading a fault");

    Battery healthy;
    burst(healthy, pinMv(VBAT_MIN_V + 0.5f));
    TEST_ASSERT_FALSE(healthy.low());

    Battery tooHigh;
    for (int i = 0; i < 40; ++i) burst(tooHigh, pinMv(VBAT_IMPLAUSIBLE_V + 0.20f));
    TEST_ASSERT_TRUE(tooHigh.low());
    TEST_ASSERT_TRUE(tooHigh.implausible());

    // A reading that is fine for this many cells and would be nonsense for one
    // more: proves the bound tracks the cell count instead of being a constant.
    Battery mid;
    for (int i = 0; i < 40; ++i) burst(mid, pinMv(VBAT_CELL_IMPLAUSIBLE_V + 0.20f));
    if (VBAT_CELLS == 1) {
        TEST_ASSERT_TRUE_MESSAGE(mid.implausible(),
                                 "5.0 V cannot be one Li-ion cell — if this fails, the "
                                 "ceiling stopped being per-cell");
    } else {
        TEST_ASSERT_FALSE(mid.implausible());
    }
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_divider_scaling_and_gate);
    RUN_TEST(test_implausible_reading_is_never_healthy);
    RUN_TEST(test_written_floor_is_the_floor_the_class_uses);
    RUN_TEST(test_median_rejects_spike);
    RUN_TEST(test_ema_converges_monotonically);
    RUN_TEST(test_uninitialised_is_not_low);
    RUN_TEST(test_chemistry_derived_fixtures_still_mean_the_same_thing);
    return UNITY_END();
}
