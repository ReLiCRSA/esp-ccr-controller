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
void test_the_published_view_saturates_rather_than_wraps();

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

// ===========================================================================
// Dive continuity (review H1): taking the tissue picture out of the engine and
// putting it back in. These are the cases the persistence layer's whole claim
// rests on — that a seeded engine is a real engine, that the surface interval it
// fast-forwards through is the SAME integration the engine would have done
// itself, and that a diver who surfaces obligated and dives again an hour later
// is holding a ceiling at entry rather than a zero.
// ===========================================================================

// Numbers a diver could not have produced must not become a plausible ceiling.
// A record torn in half, a number quantised into a different unit, a NaN: each
// one has to be refused, and refused WITHOUT touching the engine, because the
// caller's fallback is reset() and it must be falling back from a whole engine,
// not a half-seeded one.
void test_seed_refuses_garbage_and_leaves_the_engine_untouched() {
    Zhl16Ccr d(baseCfg());
    holdAt(d, 40.0f, 0.8f, 30.0f);
    const Zhl16Ccr::TissueState before = d.tissueState();
    TEST_ASSERT_TRUE(before.cnsPct > 0.0);   // the dive clocked oxygen: worth checking

    Zhl16Ccr::TissueState bad{};
    for (int i = 0; i < Zhl16Ccr::NCOMP; ++i) bad.pt[i] = before.pt[i];
    bad.cnsPct = 20.0;

    bad.pt[7] = std::nan("");
    TEST_ASSERT_FALSE(d.setTissueState(bad));
    bad.pt[7] = -3.0;
    TEST_ASSERT_FALSE(d.setTissueState(bad));
    bad.pt[7] = 0.0;                       // vacuum: no such tissue
    TEST_ASSERT_FALSE(d.setTissueState(bad));
    bad.pt[7] = 100.0;                     // == kSeedMaxBar: allowed
    bad.pt[8] = 100.01;                    // above anything this hardware can measure
    TEST_ASSERT_FALSE(d.setTissueState(bad));
    bad.pt[8] = 5.0;
    bad.cnsPct = std::nan("");
    TEST_ASSERT_FALSE(d.setTissueState(bad));

    // Nothing above changed anything: still exactly the dive it was on, oxygen
    // clock included. A caller that reacts to false by calling reset() must be
    // reacting to an engine that still holds its previous state, not to a
    // half-seeded one — which is why validation happens before any commit.
    for (int i = 0; i < Zhl16Ccr::NCOMP; ++i)
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.0, before.pt[i], d.tissueBar(i),
                                         "a refused seed must not move a compartment");
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.0, before.cnsPct, d.cnsPct(),
                                     "a refused seed must not move the oxygen clock");

    // The CNS clock is clamped to the engine's own ceiling rather than believed
    // past it (stepCns caps at 999 %): a stored 5000 % is a corrupt field, and
    // the honest answer is the cap, not a number the engine would never reach.
    bad.pt[7] = 5.0;
    bad.cnsPct = 5000.0;
    TEST_ASSERT_TRUE(d.setTissueState(bad));
    TEST_ASSERT_FLOAT_WITHIN(0.001, 999.0, d.cnsPct());
}

// A seeded engine round-trips: what comes out is what went in, to the last bit
// the engine itself keeps.
void test_tissue_state_round_trips_through_the_engine() {
    Zhl16Ccr a(baseCfg());
    holdAt(a, 52.0f, 1.3f, 60.0f);
    const Zhl16Ccr::TissueState s = a.tissueState();

    Zhl16Ccr b(baseCfg());
    TEST_ASSERT_TRUE(b.setTissueState(s));
    for (int i = 0; i < Zhl16Ccr::NCOMP; ++i)
        TEST_ASSERT_FLOAT_WITHIN(1e-12, a.tissueBar(i), b.tissueBar(i));
    TEST_ASSERT_FLOAT_WITHIN(1e-12, a.cnsPct(), b.cnsPct());
    // And it plans identically: ceiling, NDL and the whole schedule are pure
    // functions of these numbers, which is exactly why none of them is stored.
    TEST_ASSERT_FLOAT_WITHIN(1e-9, a.ceilingBar(), b.ceilingBar());
}

// The fast-forward's whole justification, stated as an assertion: this engine
// integrates a constant ppN2 in <=10 s substeps, each one multiplying the
// under-saturation by e^-k*h, and those compose EXACTLY into e^-k*t. So one
// closed-form step over a surface interval IS the integration the engine would
// have done, at 16 exp() calls instead of elapsed/10 substeps. If anyone ever
// changes stepTissues() to a different integrator, this test goes red and says
// which claim stopped being true.
void test_surface_interval_equals_the_engine_own_substep_integration() {
    const double kInspiredO2 = 0.21;   // inspired air, what the model resets to
    const Config c = baseCfg();
    Zhl16Ccr a(c), b(c);
    Zhl16Ccr::TissueState seed{};
    for (int i = 0; i < Zhl16Ccr::NCOMP; ++i) seed.pt[i] = 5.0 + 0.4 * i;
    seed.cnsPct = 33.3;
    TEST_ASSERT_TRUE(a.setTissueState(seed));
    TEST_ASSERT_TRUE(b.setTissueState(seed));

    // 90 minutes at the surface, in one step.
    a.surfaceInterval(90.0 * 60.0);
    // The same 90 minutes as the engine's own 10 s substeps, breathing air at the
    // configured surface pressure. The inspired ppO2 is 0.21 x (P - p_H2O), not
    // 0.21 flat: that is the single definition surfacePpN2Bar() aims reset() at,
    // and using the ambient-air value instead would be a second surface model.
    const double ps = c.pSurfaceBar, ph2o = c.ph2oBar, pco2 = c.pco2Bar;
    // The air the surface interval breathes is the air this model resets to:
    // inspired ppN2 == surfacePpN2Bar(), so ppO2 is whatever is left of
    // (P - p_H2O) after that. Taking 0.21 x (P - p_H2O) instead would be a
    // second surface model — it differs from the first by about 2e-8 bar
    // (0.79 vs 1-0.21 in binary), which is nothing to a diver and everything to
    // an assertion that claims one integration rather than two.
    const double ppO2 = (ps - ph2o) - (a.surfacePpN2Bar() + pco2);
    TEST_ASSERT_TRUE_MESSAGE(ppO2 > 0.15 && ppO2 < 0.25,
                             "the surface asymptote implies an impossible inspired ppO2");
    for (int s = 0; s < 90 * 6; ++s) b.stepTissues(10.0, ps, ppO2);
    // Compared in double, not through Unity's float macro: the claim here is that
    // the two integrations agree to floating-point noise, and float resolution at
    // these magnitudes (~6e-8) is a thousand times coarser than that.
    double worst = 0.0;
    for (int i = 0; i < Zhl16Ccr::NCOMP; ++i) {
        const double diff = std::fabs(a.tissueBar(i) - b.tissueBar(i));
        if (diff > worst) worst = diff;
    }
    TEST_ASSERT_TRUE_MESSAGE(worst < 1e-12,
                             "surfaceInterval() is no longer the engine's own integration");
    // The oxygen clock is NOT decayed, because this engine has no decay term to
    // decay it with. Pinned so a future "improvement" cannot quietly seed a dive
    // from physiology the engine does not run.
    TEST_ASSERT_FLOAT_WITHIN(1e-9, 33.3, a.cnsPct());
}

// Independent arithmetic, hand-computed rather than copied from the code: the
// fast compartment (5 min half-time) over an hour is 12 half-times, so its
// under-saturation must be down by 2^-12. Written with literals, the way
// test_depthscale pins its conversions, so a change to the model cannot be
// accompanied by a silent change to the expectation.
void test_surface_interval_is_hand_worked_arithmetic() {
    Zhl16Ccr d(baseCfg());
    const double ps = d.surfacePpN2Bar();
    Zhl16Ccr::TissueState seed{};
    for (int i = 0; i < Zhl16Ccr::NCOMP; ++i) seed.pt[i] = 5.0;   // 5 bar everywhere
    seed.cnsPct = 0.0;
    TEST_ASSERT_TRUE(d.setTissueState(seed));

    d.surfaceInterval(3600.0);   // exactly one hour

    // compartment 0: 5 min half-time -> 12 half-times in an hour
    TEST_ASSERT_FLOAT_WITHIN(1e-9, 5.0 + (ps - 5.0) * (1.0 - 1.0 / 4096.0), d.tissueBar(0));
    // compartment 1: 8 min -> 7.5 half-times
    TEST_ASSERT_FLOAT_WITHIN(1e-9, 5.0 + (ps - 5.0) * (1.0 - std::pow(2.0, -7.5)), d.tissueBar(1));
    // compartment 15: 635 min -> 60/635 half-times
    TEST_ASSERT_FLOAT_WITHIN(1e-9, 5.0 + (ps - 5.0) * (1.0 - std::pow(2.0, -60.0 / 635.0)),
                             d.tissueBar(15));
    // The slow compartment has barely moved; the fast one is nearly at the surface
    // asymptote. That ordering IS the point of the persistence layer: dive 2's
    // ceiling is set mostly by what the fast tissues still hold.
    TEST_ASSERT_TRUE(d.tissueBar(0) < d.tissueBar(15));
    TEST_ASSERT_TRUE(d.tissueBar(15) > 4.5);   // the 635 min tissue has barely begun
}

// A clock can name an interval of 136 years (a uint32 epoch difference). The
// clamp at 40 half-times of the slowest compartment makes that arithmetic
// boring — and it must not change the answer by anything this engine can
// represent, nor produce a NaN.
void test_an_absurd_interval_saturates_at_equilibrium_without_a_nan() {
    Zhl16Ccr a(baseCfg()), b(baseCfg());
    Zhl16Ccr::TissueState seed{};
    for (int i = 0; i < Zhl16Ccr::NCOMP; ++i) seed.pt[i] = 9.0;
    seed.cnsPct = 12.0;
    TEST_ASSERT_TRUE(a.setTissueState(seed));
    TEST_ASSERT_TRUE(b.setTissueState(seed));

    a.surfaceInterval(4.0e9);                    // 126 years
    b.surfaceInterval(40.0 * 635.0 * 60.0);      // the clamp: 40 slowest half-times
    for (int i = 0; i < Zhl16Ccr::NCOMP; ++i) {
        TEST_ASSERT_TRUE_MESSAGE(std::isfinite(a.tissueBar(i)), "a NaN seeded into the engine");
        TEST_ASSERT_FLOAT_WITHIN(1e-12, a.surfacePpN2Bar(), a.tissueBar(i));
        TEST_ASSERT_FLOAT_WITHIN(1e-12, b.tissueBar(i), a.tissueBar(i));
    }
    // Equilibrium is not an obligation: a diver who has been out of the water for
    // a century starts clean, and the machine is entitled to say so. Note the
    // quantity: ceilingBar() is a tolerated tissue PRESSURE and is positive even
    // for clean tissues; the thing a diver can act on is ceilingM(), that pressure
    // expressed as a depth above the surface, which is what the dive file's
    // ceiling_m column and the handset both carry.
    TEST_ASSERT_TRUE(a.ceilingBar() < a.surfacePpN2Bar() + 1e-9);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0, a.ceilingM());
    // Zero and negative intervals do nothing at all: no credit is not a step.
    a.surfaceInterval(0.0);
    a.surfaceInterval(-3600.0);
    TEST_ASSERT_FLOAT_WITHIN(1e-12, a.surfacePpN2Bar(), a.tissueBar(0));
}

// The reviewer's scenario, at engine level: a diver surfaces with an obligation
// and dives again. A reset engine gives a ceiling of zero at the surface; the
// same engine seeded from where the last dive left off gives a real one.
void test_a_seeded_engine_shows_a_ceiling_at_the_surface_and_a_reset_does_not() {
    Zhl16Ccr d(baseCfg());
    holdAt(d, 48.0f, 0.9f, 45.0f);
    TEST_ASSERT_TRUE(d.ceilingM() > 0.0);            // obligated at depth: premise
    const Zhl16Ccr::TissueState endOfDive = d.tissueState();

    // Today's firmware: the next dive starts from the tissues reset() leaves.
    Zhl16Ccr clean(baseCfg());
    TEST_ASSERT_FLOAT_WITHIN(1e-9, 0.0, clean.ceilingM());
    TEST_ASSERT_FLOAT_WITHIN(1e-9, 480.0, clean.ndlMinutes(clean.depthToBar(0.0f), 0.7f));

    // One hour of surface time, credited with the engine's own integration.
    Zhl16Ccr seeded(baseCfg());
    TEST_ASSERT_TRUE(seeded.setTissueState(endOfDive));
    seeded.surfaceInterval(3600.0);
    const float ceilSeeded = seeded.ceilingM();
    // NDL is asked at a DEPTH, not at the surface: sitting at 0 m nobody is ever
    // obligated (the tissues only off-gas), so the surface NDL is the 480 minute
    // cap for every engine and proves nothing. At 20 m the seeded engine runs out
    // of no-deco time long before a clean one, and that is what dive 2 loses.
    const float ndlSeeded = seeded.ndlMinutes(seeded.depthToBar(20.0f), 0.7f);
    const float ndlClean = clean.ndlMinutes(clean.depthToBar(20.0f), 0.7f);
    TEST_ASSERT_TRUE_MESSAGE(ceilSeeded > 0.0f,
                             "a diver who surfaced obligated must be given a ceiling on entry");
    // Carrying the tissues and NOT crediting the interval is the conservative
    // direction: a deeper ceiling than the truth, never a shallower one. That is
    // the inequality the whole no-credit policy (no clock, future epoch, mid-dive
    // record) rests on, pinned here as a number rather than as a comment.
    Zhl16Ccr stale(baseCfg());
    TEST_ASSERT_TRUE(stale.setTissueState(endOfDive));
    TEST_ASSERT_TRUE(stale.ceilingM() >= ceilSeeded);
    TEST_ASSERT_TRUE(stale.ceilingM() > 0.0f);
    TEST_ASSERT_TRUE_MESSAGE(ndlSeeded < ndlClean,
                             "a seeded dive 2 must have less no-deco time than a clean one");
    TEST_ASSERT_TRUE(ndlSeeded < 480.0f);
    // And the fast-forward is not free invention: it moved the loadings down
    // towards the surface asymptote by exactly the amount the model says.
    TEST_ASSERT_TRUE(stale.tissueBar(0) > seeded.tissueBar(0));
    TEST_ASSERT_TRUE(seeded.tissueBar(0) > seeded.surfacePpN2Bar());
    (void)ndlSeeded;
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
    RUN_TEST(test_seed_refuses_garbage_and_leaves_the_engine_untouched);
    RUN_TEST(test_tissue_state_round_trips_through_the_engine);
    RUN_TEST(test_surface_interval_equals_the_engine_own_substep_integration);
    RUN_TEST(test_surface_interval_is_hand_worked_arithmetic);
    RUN_TEST(test_an_absurd_interval_saturates_at_equilibrium_without_a_nan);
    RUN_TEST(test_a_seeded_engine_shows_a_ceiling_at_the_surface_and_a_reset_does_not);
    RUN_TEST(test_pub_air_table_schedules);
    RUN_TEST(test_pub_ndl_structure);
    RUN_TEST(test_pub_vs_paliers_cross_impl);
    RUN_TEST(test_pub_gf3085_variant_documented);
    RUN_TEST(test_the_published_view_saturates_rather_than_wraps);
    return UNITY_END();
}
