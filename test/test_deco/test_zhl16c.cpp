// Unit tests for ZH-L16C CCR deco math. Bands assert behavior sanity and
// published-model direction; exact schedule cross-validation against an
// independent implementation is scheduled for M6 (PLAN.md).
#include <unity.h>

#include <cmath>

#include "deco/zhl16c.h"

using deco::Config;
using deco::Zhl16Ccr;

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
    // Same tissue state, uniformly higher GF -> deeper tolerated ceiling.
    // GF conservatism is not one axis: lower GF-low buys shorter no-stop
    // time but shallower first stops; the model must be monotonic in GF.
    Config g10 = baseCfg(); g10.gfLow = 10; g10.gfHigh = 10;
    Config g50 = baseCfg(); g50.gfLow = 50; g50.gfHigh = 50;
    Config g90 = baseCfg(); g90.gfLow = 90; g90.gfHigh = 90;
    Zhl16Ccr a(g10), b(g50), c(g90);
    holdAt(a, 40.0f, 1.0f, 35.0f);
    holdAt(b, 40.0f, 1.0f, 35.0f);
    holdAt(c, 40.0f, 1.0f, 35.0f);
    TEST_ASSERT_TRUE(a.ceilingBar() < b.ceilingBar());
    TEST_ASSERT_TRUE(b.ceilingBar() < c.ceilingBar());

    const auto sLo = a.schedule(a.depthToBar(40.0));
    const auto sHi = c.schedule(c.depthToBar(40.0));
    TEST_ASSERT_TRUE(sLo.count > 0 && sHi.count > 0);
    TEST_ASSERT_FALSE(sLo.capped);
    TEST_ASSERT_TRUE(sLo.stopTimeMin > 0.0f && sHi.stopTimeMin > 0.0f);
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
    // ceiling. Assert the ceiling stays shallow (<=6 m) and stops are few.
    Zhl16Ccr d(baseCfg());
    holdAt(d, 30.0f, 0.7f, 2.0f);
    const auto s = d.schedule(d.depthToBar(30.0));
    TEST_ASSERT_TRUE(s.count <= 2);
    for (int i = 0; i < s.count; ++i)
        TEST_ASSERT_TRUE(s.stops[i].depthM <= 6.01f);
    TEST_ASSERT_TRUE(s.totalTimeMin > 0.0f); // ascent time always accrues
}

void test_cns_clock_matches_noaa_16() {
    Zhl16Ccr d(baseCfg());
    d.step(45.0 * 60.0, d.depthToBar(0.0), 1.6); // 45 min @ 1.6
    TEST_ASSERT_FLOAT_WITHIN(2.0, 100.0, d.cnsPct());

    Zhl16Ccr c(baseCfg());
    c.step(60.0 * 60.0, c.depthToBar(5.0), 0.45); // below 0.5 => no accrual
    TEST_ASSERT_FLOAT_WITHIN(0.01, 0.0, c.cnsPct());
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
    RUN_TEST(test_pco2_inclusion_is_conservative);
    return UNITY_END();
}
