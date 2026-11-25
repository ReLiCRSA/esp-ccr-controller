// Unit tests for ZH-L16C CCR deco math: behavior + invariants, including the
// CNS/tissue split the control chain now runs under a depth fault (the two
// halves are separately gated, so they are separately tested). The
// published-table cross-validation lives in test_published.cpp (same suite:
// done 2026-11-23 vs the Buhlmann air tables and the paliers implementation;
// the pre-fix bands here were re-banded where the corrected M-form moved
// behavior, see git history of this file).
#include <unity.h>

#include <cmath>

#include "deco/zhl16c.h"

using deco::Config;
using deco::Zhl16Ccr;

// Published-data suite (test_published.cpp):
void test_pub_air_table_schedules();
void test_pub_ndl_structure();
void test_pub_vs_paliers_cross_impl();
void test_pub_gf3085_variant_documented();

static Config baseCfg() {
    Config c; // defaults: GF 30/85, sea water, 6 m/min
    return c;
}

// Dive helper: descend instantly to depthM (worst-case uptake for the
// simulation is fine here) then hold for minutes at given ppO2.
static void holdAt(Zhl16Ccr& d, float depthM, float ppO2, float minutes) {
    const double p = d.depthToBar(depthM);
    d.step(minutes * 60.0, p, ppO2);
}

void setUp() {}
void tearDown() {}

void test_reset_surface_balance_ceiling_zero() {
    Zhl16Ccr d(baseCfg());
    TEST_ASSERT_FLOAT_WITHIN(0.001, 0.0, d.ceilingM());
}

void test_compartment_matches_haldane_analytic() {
    // Compartment half-times must reproduce the closed-form step response.
    Zhl16Ccr d(baseCfg());
    const float depth = 30.0f, mins = 20.0f;
    const double p = d.depthToBar(depth);
    const double ppN2 = p - 0.0627 - 0.7; // SP 0.7 at 30 m
    holdAt(d, depth, 0.7f, mins);
    // fastest compartment (5 min):
    const double expected = 0.7516 + (ppN2 - 0.7516) * (1.0 - std::pow(2.0, -mins / 5.0));
    TEST_ASSERT_FLOAT_WITHIN(0.01, expected, d.tissueBar(0));
}

void test_ndl_semantics_and_obligation() {
    // Surface: effectively unlimited.
    Zhl16Ccr surface(baseCfg());
    TEST_ASSERT_FLOAT_WITHIN(0.01, 480.0, surface.ndlMinutes(surface.depthToBar(0), 0.21));

    // CCR at SP 0.7 on 18 m: modest loading -> long NDL (GF crush at depth
    // is real GF behavior; the CEILING is what signals obligation at deep
    // setpoints, not NDL).
    Zhl16Ccr shallow(baseCfg());
    holdAt(shallow, 18.0f, 0.7f, 2.0f);
    const double n18 = shallow.ndlMinutes(shallow.depthToBar(18.0), 0.7);
    TEST_ASSERT_TRUE_MESSAGE(n18 > 30.0 && n18 <= 480.0, "18m@0.7 NDL implausibly short");

    // Deep with high SP: no ceiling AT depth (crush), but after the dive a
    // deco obligation exists and grows with bottom time.
    Zhl16Ccr deep(baseCfg());
    holdAt(deep, 40.0f, 1.0f, 35.0f);
    TEST_ASSERT_TRUE(deep.ceilingM() > 10.0); // deep ceiling carried
}

void test_ceiling_monotonic_in_gf() {
    // Same tissue state: LOWER gradient factor tolerates less supersaturation
    // -> the ceiling it demands is DEEPER (higher pressure), and the stop
    // schedule is longer. (The first revision of this test asserted the
    // reverse direction and passed only because the ceiling formula was
    // wrong; corrected with the M = a + P/b fix.)
    Config g10 = baseCfg(); g10.gfLow = 10; g10.gfHigh = 10;
    Config g50 = baseCfg(); g50.gfLow = 50; g50.gfHigh = 50;
    Config g90 = baseCfg(); g90.gfLow = 90; g90.gfHigh = 90;
    Zhl16Ccr a(g10), b(g50), c(g90);
    holdAt(a, 40.0f, 1.0f, 35.0f);
    holdAt(b, 40.0f, 1.0f, 35.0f);
    holdAt(c, 40.0f, 1.0f, 35.0f);
    TEST_ASSERT_TRUE(a.ceilingBar() > b.ceilingBar());
    TEST_ASSERT_TRUE(b.ceilingBar() > c.ceilingBar());

    const auto sLo = a.schedule(a.depthToBar(40.0));
    const auto sHi = c.schedule(c.depthToBar(40.0));
    TEST_ASSERT_TRUE(sLo.count > 0 && sHi.count > 0);
    TEST_ASSERT_FALSE(sLo.capped);
    TEST_ASSERT_TRUE(sLo.stopTimeMin > sHi.stopTimeMin);
}

void test_schedule_deep_dive_has_ordered_stops_ending_at_3m() {
    Zhl16Ccr d(baseCfg());
    holdAt(d, 40.0f, 1.0f, 35.0f);
    const auto s = d.schedule(d.depthToBar(40.0));
    TEST_ASSERT_TRUE(s.count > 0);
    TEST_ASSERT_TRUE(s.stops[0].depthM <= 39.01f && s.stops[0].depthM >= 3.0f);
    for (int i = 1; i < s.count; ++i) {
        TEST_ASSERT_TRUE(s.stops[i].depthM < s.stops[i - 1].depthM);
        TEST_ASSERT_TRUE(s.stops[i].minutes > 0.0f);
    }
    TEST_ASSERT_FLOAT_WITHIN(0.01, 3.0f, s.stops[s.count - 1].depthM);
    TEST_ASSERT_TRUE(s.stopTimeMin > 0.0f);
    TEST_ASSERT_TRUE(s.totalTimeMin >= s.stopTimeMin);
    TEST_ASSERT_FALSE(s.capped);
}

void test_schedule_short_dive_shallow_stops_only() {
    // GF30/85 is genuinely assertive: even 2 min at 30 m carries a tiny
    // ceiling. With the corrected M-form, GF30 on the fast compartment puts
    // the ceiling near 8 m right after the descent, so the first stop sits
    // on the 9 m grid point — still a short, shallow schedule (re-banded
    // from the pre-fix model, which showed it at <= 6 m).
    Zhl16Ccr d(baseCfg());
    holdAt(d, 30.0f, 0.7f, 2.0f);
    const auto s = d.schedule(d.depthToBar(30.0));
    TEST_ASSERT_TRUE(s.count <= 3);
    for (int i = 0; i < s.count; ++i)
        TEST_ASSERT_TRUE(s.stops[i].depthM <= 9.01f);
    TEST_ASSERT_TRUE(s.totalTimeMin > 0.0f); // ascent time always accrues
}

void test_cns_clock_matches_noaa_16() {
    Zhl16Ccr d(baseCfg());
    d.step(45.0 * 60.0, d.depthToBar(0.0), 1.6); // 45 min @ 1.6
    TEST_ASSERT_FLOAT_WITHIN(2.0, 100.0, d.cnsPct());

    Zhl16Ccr c(baseCfg());
    c.step(60.0 * 60.0, c.depthToBar(5.0), 0.45); // below 0.5 => no accrual
    TEST_ASSERT_FLOAT_WITHIN(0.01, 0.0, c.cnsPct());

    // Corrected NOAA rows (the pre-validation table had 0.8 -> 360 and
    // 0.7 -> 540; the published table is 450 / 570 and internally
    // consistent: its %CNS/min column is 1/limit for every row).
    Zhl16Ccr e(baseCfg());
    e.step(450.0 * 60.0, e.depthToBar(0.0), 0.8);
    TEST_ASSERT_FLOAT_WITHIN(2.0, 100.0, e.cnsPct());
    Zhl16Ccr f(baseCfg());
    f.step(570.0 * 60.0, f.depthToBar(0.0), 0.7);
    TEST_ASSERT_FLOAT_WITHIN(2.0, 100.0, f.cnsPct());
}

// ---------------------------------------------------------------------------
// The two halves of integration are separable, and the split is exact.
//
// Why this exists: the control chain runs them under different conditions now.
// CNS counts oxygen exposure whenever the CELL vote is trustworthy; tissues
// count ambient pressure whenever the DEPTH leg is — which during a depth-bus
// fault means the held last-good pressure. If either half kept its own idea of
// elapsed time (a stored timestamp, a leftover dt), a window in which only one
// of them ran would make the other lose time or count it twice. These three
// tests are the witness that neither half can: step() is the wrapper, the halves
// are additive by construction, and the published-table suite above keeps
// passing through the wrapper unchanged.
// ---------------------------------------------------------------------------
void test_step_cns_only_clocks_time_without_touching_tissues() {
    Zhl16Ccr d(baseCfg());
    holdAt(d, 30.0f, 1.3f, 10.0f);  // a real dive first: tissues loaded, CNS running

    double before[Zhl16Ccr::NCOMP];
    for (int i = 0; i < Zhl16Ccr::NCOMP; ++i) before[i] = d.tissueBar(i);
    const double cns0 = d.cnsPct();

    // 10 min of ppO2 with no ambient pressure argument at all — this is the
    // shape of a depth-bus fault: the cells are a measurement, the depth is not.
    for (int k = 0; k < 10; k++) d.stepCns(60.0, 1.3);

    // NOAA 1.30 row: 180 min => 100/180 %/min => 5.56 % over 10 min.
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.01, cns0 + 100.0 * 10.0 / 180.0, d.cnsPct(),
                                     "stepCns did not accrue the ppO2 exposure it was given");
    for (int i = 0; i < Zhl16Ccr::NCOMP; ++i)
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(1e-9, before[i], d.tissueBar(i),
                                         "stepCns moved a tissue compartment");
}

void test_step_tissues_only_loads_tissues_without_clocking_cns() {
    Zhl16Ccr d(baseCfg());
    holdAt(d, 30.0f, 1.3f, 10.0f);
    const double cns0 = d.cnsPct();
    const double fast0 = d.tissueBar(0);
    const double p = d.depthToBar(30.0);

    for (int k = 0; k < 10; k++) d.stepTissues(60.0, p, 1.3);

    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(1e-9, cns0, d.cnsPct(), "stepTissues moved the CNS clock");
    // Haldane closed form on the 5 min compartment: 10 min at constant ppN2
    // closes 1 - 2^-2 = 75 % of the gap. Also the witness that the tissue half
    // really did integrate the 10 minutes it was handed.
    const double ppN2 = p - 0.0627 - 1.3;
    const double expected = fast0 + (ppN2 - fast0) * (1.0 - std::pow(2.0, -10.0 / 5.0));
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.005, expected, d.tissueBar(0),
                                     "stepTissues did not integrate the time it was given");
    TEST_ASSERT_TRUE(d.tissueBar(0) > fast0);
}

void test_step_wrapper_equals_the_two_halves_no_time_lost_or_double_counted() {
    const double p = Zhl16Ccr(baseCfg()).depthToBar(30.0);

    // Reference: 30 min through the wrapper, one call.
    Zhl16Ccr a(baseCfg());
    a.step(30.0 * 60.0, p, 1.3);

    // Same 30 min, but delivered as halves one minute at a time.
    Zhl16Ccr b(baseCfg());
    for (int k = 0; k < 30; k++) {
        b.stepCns(60.0, 1.3);
        b.stepTissues(60.0, p, 1.3);
    }
    TEST_ASSERT_FLOAT_WITHIN(1e-6, a.cnsPct(), b.cnsPct());
    for (int i = 0; i < Zhl16Ccr::NCOMP; ++i)
        TEST_ASSERT_FLOAT_WITHIN(1e-6, a.tissueBar(i), b.tissueBar(i));

    // And the fault shape: 10 min healthy, 10 min where only CNS ran (depth
    // lost), 10 min healthy again. The CNS clock must not care that a half sat
    // out — same 30 min of ppO2, same total — while the tissues, which missed
    // those 10 min, must be behind the reference. That is the pair of assertions
    // that fails if either half skips dt or counts it twice.
    Zhl16Ccr c(baseCfg());
    c.step(10.0 * 60.0, p, 1.3);
    c.stepCns(10.0 * 60.0, 1.3);
    c.step(10.0 * 60.0, p, 1.3);
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(1e-6, a.cnsPct(), c.cnsPct(),
                                     "a CNS-only window changed the oxygen clock total");
    TEST_ASSERT_TRUE_MESSAGE(c.tissueBar(0) < a.tissueBar(0),
                             "tissues counted time they were never handed");
}

void test_pco2_inclusion_is_conservative() {
    Config cNo = baseCfg();
    Config cWith = baseCfg();
    cWith.pco2Bar = 0.0534f; // alveolar CO2 lowers ppN2 -> less uptake
    Zhl16Ccr a(cNo), b(cWith);
    holdAt(a, 30.0f, 0.7f, 20.0f);
    holdAt(b, 30.0f, 0.7f, 20.0f);
    TEST_ASSERT_TRUE(a.tissueBar(0) >= b.tissueBar(0)); // pCO2=0 default loads more
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_reset_surface_balance_ceiling_zero);
    RUN_TEST(test_compartment_matches_haldane_analytic);
    RUN_TEST(test_ndl_semantics_and_obligation);
    RUN_TEST(test_ceiling_monotonic_in_gf);
    RUN_TEST(test_schedule_deep_dive_has_ordered_stops_ending_at_3m);
    RUN_TEST(test_schedule_short_dive_shallow_stops_only);
    RUN_TEST(test_cns_clock_matches_noaa_16);
    RUN_TEST(test_step_cns_only_clocks_time_without_touching_tissues);
    RUN_TEST(test_step_tissues_only_loads_tissues_without_clocking_cns);
    RUN_TEST(test_step_wrapper_equals_the_two_halves_no_time_lost_or_double_counted);
    RUN_TEST(test_pco2_inclusion_is_conservative);
    RUN_TEST(test_pub_air_table_schedules);
    RUN_TEST(test_pub_ndl_structure);
    RUN_TEST(test_pub_vs_paliers_cross_impl);
    RUN_TEST(test_pub_gf3085_variant_documented);
    return UNITY_END();
}
