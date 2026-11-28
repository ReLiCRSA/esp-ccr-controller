#include <unity.h>

#include <cmath>

#include "sensors/battery.h"
#include "sensors/rail.h"

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

// ---------------------------------------------------------------------------
// The rail input (sensors/rail.h) — the ~12 V the valve actually runs on.
// Same divider DSP as the pack (sensors/dividersense.h, graded above through
// Battery), different verdicts: LOW needs patience, HIGH does not, and an input
// that was never read is neither healthy nor condemned until a build decision
// says it exists at all.
// ---------------------------------------------------------------------------

static constexpr int railPinMv(float railVolts) {
    return static_cast<int>(std::lround(railVolts * 1000.0f / VRAIL_DIVIDER_RATIO));
}

static void railBurst(Rail& r, int mv) {
    for (int i = 0; i < 8; ++i) r.addSample(mv);
}

// One sweep below the band is a measurement of a pulse, not of the supply:
// SensorTask's 1 Hz burst lands on a 300 ms pulse roughly one sweep in seven,
// and a mid-dive nuisance alarm on BIT_BATTERY is how a diver learns to ignore
// the marker that means "do not trust this power". So LOW is a RUN COUNT, and
// the run only counts sweeps whose processed reading is genuinely out of band —
// the EMA lag means a single sag-shaped sweep does not even start the count.
void test_rail_low_needs_consecutive_low_sweeps() {
    Rail r;
    for (int i = 0; i < 3; ++i) railBurst(r, railPinMv(VRAIL_NOMINAL_V));
    TEST_ASSERT_FALSE(r.low());
    TEST_ASSERT_TRUE(r.sensed());

    // A supply held below the band. The assertion is an equivalence, checked on
    // EVERY sweep: the alarm is up exactly when the processed reading has been
    // out of band for VRAIL_LOW_SAMPLES sweeps in a row. That pins the run
    // counter itself — a test that only asked "does it eventually alarm, and
    // not on sweep 1?" survived the debounce being deleted outright, because
    // the EMA alone takes two or three sweeps to cross a band, which mimics
    // the count well enough to fool a loose window.
    int belowRun = 0;
    for (int sweep = 1; sweep <= 20; ++sweep) {
        railBurst(r, railPinMv(VRAIL_MIN_V - 1.0f));
        if (r.volts() < VRAIL_MIN_V) ++belowRun; else belowRun = 0;
        TEST_ASSERT_EQUAL_INT_MESSAGE(belowRun >= VRAIL_LOW_SAMPLES, r.low(),
                                      "the alarm left the rule 'out of band for "
                                      "VRAIL_LOW_SAMPLES sweeps in a row' on some sweep");
    }
    TEST_ASSERT_TRUE(r.low());
    TEST_ASSERT_TRUE_MESSAGE(belowRun >= VRAIL_LOW_SAMPLES,
                             "the fixture must actually drive the reading out of band");

    // And recovery releases it. The same equivalence, run backwards: the run
    // counter zeroes on the first PROCESSED sweep back inside the band — which
    // is not the first burst fed, because the EMA takes a sweep or two to come
    // back — and the alarm must leave exactly when the rule says it does, not
    // after an extra grace and not still lingering.
    for (int sweep = 1; sweep <= 12; ++sweep) {
        railBurst(r, railPinMv(VRAIL_NOMINAL_V));
        if (r.volts() < VRAIL_MIN_V) ++belowRun; else belowRun = 0;
        TEST_ASSERT_EQUAL_INT_MESSAGE(belowRun >= VRAIL_LOW_SAMPLES, r.low(),
                                      "the release left the same rule the raise used");
    }
    TEST_ASSERT_FALSE(r.low());
}

// HIGH has no run counter — an open bottom leg parks the pin on its rail and
// that is steady, and a steady fault needs no patience. The EMA still kills one
// spurious burst, which is a spike filter, not a debounce.
void test_rail_high_needs_no_run_counter() {
    Rail r;
    railBurst(r, railPinMv(VRAIL_NOMINAL_V));
    TEST_ASSERT_FALSE(r.implausible());

    // Open divider: the pin sits on its ~3.3 V rail, which through 11:1 is
    // tens of rail volts — one processed sweep puts the reading past the ceiling.
    railBurst(r, railPinMv(VRAIL_IMPLAUSIBLE_V + 20.0f));
    TEST_ASSERT_TRUE_MESSAGE(r.implausible(),
                             "a sense node on its rail must be implausible at once");
    TEST_ASSERT_TRUE(r.untrustworthy());

    // One clean sweep back does not clear it instantly — that is the EMA, not a
    // debounce — but it does not need a run either: the same number of bursts
    // that took LOW off the band take this one back inside.
    for (int i = 0; i < 12; ++i) railBurst(r, railPinMv(VRAIL_NOMINAL_V));
    TEST_ASSERT_FALSE(r.implausible());
}

// Never-sensed is its own state, and it is untrustworthy without being "low":
// with VRAIL_SENSE=0 nothing feeds this object, and "no reading" must not
// silently mean "healthy" — HB's vrail=0.00 railstate=3 is the honest answer.
void test_rail_never_sensed_is_untrustworthy_but_not_low() {
    Rail r;
    TEST_ASSERT_FALSE_MESSAGE(r.low(), "no reading is not a LOW reading: the alarm "
                                       "path is gated by VRAIL_SENSE, not by faking a level");
    TEST_ASSERT_FALSE(r.implausible());
    TEST_ASSERT_FALSE(r.sensed());
    TEST_ASSERT_TRUE(r.untrustworthy());
}

// The band follows the coil's rated voltage and its published supply tolerance,
// exactly the way the pack band follows VBAT_CELLS: one rated number, the rest
// derived. If anyone hardcodes 10.2 or 13.8 somewhere, this leg reddens.
void test_rail_band_follows_the_rated_voltage() {
    TEST_ASSERT_FLOAT_WITHIN(0.001f, VRAIL_NOMINAL_V * 0.85f, VRAIL_MIN_V);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, VRAIL_NOMINAL_V * 1.15f, VRAIL_IMPLAUSIBLE_V);
    TEST_ASSERT_TRUE(VRAIL_MIN_V < VRAIL_IMPLAUSIBLE_V);

    // Inside near both edges, outside past them — derived from the same two
    // numbers, so a change to VRAIL_NOMINAL_V or the tolerance moves the
    // fixtures with it and this still means the same thing.
    Rail inLo, inHi, outHi;
    for (int i = 0; i < 12; ++i) {
        railBurst(inLo, railPinMv(VRAIL_MIN_V + 0.05f));
        railBurst(inHi, railPinMv(VRAIL_IMPLAUSIBLE_V - 0.05f));
    }
    TEST_ASSERT_FALSE(inLo.low());
    TEST_ASSERT_FALSE(inHi.implausible());
    TEST_ASSERT_FALSE(inHi.low());
    for (int i = 0; i < 12; ++i) railBurst(outHi, railPinMv(VRAIL_IMPLAUSIBLE_V + 0.05f));
    TEST_ASSERT_TRUE(outHi.implausible());
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
    RUN_TEST(test_rail_low_needs_consecutive_low_sweeps);
    RUN_TEST(test_rail_high_needs_no_run_counter);
    RUN_TEST(test_rail_never_sensed_is_untrustworthy_but_not_low);
    RUN_TEST(test_rail_band_follows_the_rated_voltage);
    return UNITY_END();
}
