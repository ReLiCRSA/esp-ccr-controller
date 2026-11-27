// ============================================================================
// ZH-L16C published-data validation (M4 exit criterion: "unit tests match
// published NDL/deco tables"). Run 2026-11-23.
//
// Sources:
//  * Parameters + formulas: ZH-L16C a/b/half-time set verified cell-by-cell
//    against the published transcription in Wikipedia "Buhlmann decompression
//    algorithm" and the buehlmann_N2_a/_b arrays in Subsurface core/deco.cpp.
//    M-value form M = a + P/b and ceiling algebra: Subsurface core/deco.cpp
//    ((b*T - gf*a*b) / ((1-b)*gf + b)) and repied/paliers src/gf.ts
//    (getMValue, calculateCeilings) — two independent implementations.
//  * Schedule oracle: "Buhlmann Air Decompression Tables, Sea Level",
//    tech-diving.ru/Materials/Deco_Package.pdf. Stop minutes per entry are
//    TOT - bottom - ascent(depth at 10 m/min); the tables append a
//    "minimum of 1 minute at 3 meters" to EVERY dive, which this engine
//    does not force, so published values are normalized by -1 below.
//    Those tables were generated with the ZH-L16 **B** scheme (the printed-
//    tables scheme per the CMAS fact sheet "Buhlmann ZH-L"); this engine is
//    **C** — the documented envelope below is sized for that. The same
//    deviation shows against the independent C-scheme implementation at the
//    same entries (27 m worst), so the gap is B-vs-C, not engine error.
//  * Cross-implementation oracle: github.com/repied/paliers, src/gf.ts
//    upstream-unmodified (only the type import retargeted), executed under
//    node 26 over identical profiles; its outputs are pinned in kEntries
//    below. paliers uses dry ppN2 = 0.79*P; this engine uses Buhlmann's own
//    alveolar form 0.79*(P - pH2O) — wet uptake is slightly lower, hence
//    our consistently short(er) schedules.
// ============================================================================
#include <unity.h>

#include <string>

#include "deco/view.h"
#include "deco/zhl16c.h"

using deco::Config;
using deco::Zhl16Ccr;

namespace {

struct Entry {
    float depthM, bottomMin;
    float pubStopMin;      // incl. the universal 1-min-at-3m table rule
    float paliers100;      // paliers t_stops at GF100/100
    float paliers80;       // GF80/80
    float paliers30_85;    // GF30/85 (depth-interpolated GF — different variant)
};

const Entry kEntries[] = {
    {21, 40, 2, 1, 9, 7}, {21, 50, 8, 7, 20, 18}, {21, 60, 16, 14, 31, 30},
    {24, 40, 8, 7, 20, 18}, {24, 50, 17, 16, 33, 31},
    {27, 25, 5, 1, 7, 7}, {27, 35, 17, 9, 23, 22}, {27, 45, 32, 21, 39, 38},
    {30, 20, 2, 1, 7, 6}, {30, 25, 5, 5, 13, 12}, {30, 30, 9, 9, 23, 22},
    {30, 35, 17, 17, 31, 31}, {30, 40, 22, 23, 42, 43}, {30, 45, 32, 30, 53, 54},
    {33, 25, 9, 8, 20, 19}, {33, 35, 23, 24, 43, 45},
    {36, 20, 7, 6, 15, 14}, {36, 25, 13, 13, 28, 27}, {36, 30, 22, 23, 40, 41},
    {39, 15, 4, 4, 10, 9}, {39, 20, 10, 10, 22, 21}, {39, 25, 18, 19, 35, 36},
    {39, 30, 28, 29, 52, 54},
    {42, 18, 10, 10, 22, 22}, {42, 21, 16, 16, 31, 32},
};
constexpr int NENT = static_cast<int>(sizeof(kEntries) / sizeof(kEntries[0]));

Config tableCfg(float gf) {
    Config c;  // table replication: sea water, symmetric GF, 10 m/min all
               // the way up, open-circuit air during the simulated ascent
               // (the published tables are OC-air tables).
    c.gfLow = gf; c.gfHigh = gf;
    c.ascentRateM = 10.0f; c.ascentRateSlowM = 10.0f;
    c.decoAirFraction = 0.21f;
    return c;
}

float runStops(const Entry& e, float gf) {
    const Config cfg = tableCfg(gf);
    Zhl16Ccr z(cfg);
    z.reset();
    const double p = z.depthToBar(e.depthM);
    z.step(e.bottomMin * 60.0, p, 0.21 * (p - cfg.ph2oBar));  // OC air bottom
    const auto s = z.schedule(p);
    TEST_ASSERT_FALSE_MESSAGE(s.capped, "schedule hit the per-stop cap");
    return s.stopTimeMin;
}

float runStops(const Entry& e, float gfLow, float gfHigh) {
    Config c = tableCfg(gfLow);
    c.gfHigh = gfHigh;
    Zhl16Ccr z(c);
    z.reset();
    const double p = z.depthToBar(e.depthM);
    z.step(e.bottomMin * 60.0, p, 0.21 * (p - c.ph2oBar));  // OC air bottom
    const auto s = z.schedule(p);
    TEST_ASSERT_FALSE_MESSAGE(s.capped, "schedule hit the per-stop cap");
    return s.stopTimeMin;
}

}  // namespace

// Same model as the published tables (pure M-values: GF100/100 = no GF
// shaping). Engine stops vs published stops minus the 1-min table safety
// rule. Measured deltas 2026-11-23: -14.5..+0.5 min, mean ~-2.8; envelope
// [-16, +2] with the negative direction fully explained: ZH-L16 C-vs-B
// scheme (published tables' scheme has larger a on the 38.3/54.3/77-min
// compartments = more deco) and wet vs dry alveolar ppN2. An engine that
// has drifted back toward the old (a + b*P) form would miss the upper bound
// by tens of minutes on every entry.
void test_pub_air_table_schedules() {
    for (int i = 0; i < NENT; ++i) {
        const Entry& e = kEntries[i];
        const float ours = runStops(e, 100.0f);
        const float pubModel = e.pubStopMin - 1.0f;  // strip forced safety min
        const float d = ours - pubModel;
        TEST_ASSERT_TRUE_MESSAGE(d >= -16.0f && d <= 2.0f,
                                 "published-table stop time out of envelope");
    }
}

// NDL as published structure says it: with GF100 air and 10 m/min direct
// ascent, the tables flip from no-stop to obligatory stops at 30 m between
// the 15 and 20-minute entries (30/20 carries a model minute beyond the
// safety minute). Engine NDL must sit inside those transitions, and must
// fall strictly with depth.
void test_pub_ndl_structure() {
    const Config cfg = tableCfg(100.0f);
    auto ndlAt = [&](float d) {
        Zhl16Ccr z(cfg);
        const double p = z.depthToBar(d);
        return z.ndlMinutes(p, 0.21 * (p - cfg.ph2oBar));
    };
    const double n30 = ndlAt(30.0f), n21 = ndlAt(21.0f), n18 = ndlAt(18.0f);
    TEST_ASSERT_TRUE_MESSAGE(n30 > 17.0 && n30 < 22.0, "NDL(30 m) outside published transition");
    TEST_ASSERT_TRUE_MESSAGE(n21 > 38.0 && n21 < 50.0, "NDL(21 m) outside published transition");
    TEST_ASSERT_TRUE_MESSAGE(n18 > 52.0 && n18 < 75.0, "NDL(18 m) outside published transition");
    TEST_ASSERT_TRUE(ndlAt(18.0f) > ndlAt(21.0f));
    TEST_ASSERT_TRUE(ndlAt(21.0f) > ndlAt(30.0f));
    TEST_ASSERT_TRUE(ndlAt(30.0f) > ndlAt(36.0f));
    TEST_ASSERT_TRUE(ndlAt(36.0f) > ndlAt(42.0f));
}

// Tight cross-implementation anchor (same variant: symmetric GFs make
// depth- vs tissue-interpolation identical). The only remaining convention
// gap is wet vs dry alveolar ppN2, ~0.05 bar of N2 — bounded drift below.
void test_pub_vs_paliers_cross_impl() {
    for (int i = 0; i < NENT; ++i) {
        const Entry& e = kEntries[i];
        const float d100 = runStops(e, 100.0f) - e.paliers100;
        const float d80 = runStops(e, 80.0f) - e.paliers80;
        TEST_ASSERT_TRUE_MESSAGE(d100 >= -6.0f && d100 <= 6.0f,
                                 "GF100/100 diverged from independent implementation");
        TEST_ASSERT_TRUE_MESSAGE(d80 >= -11.0f && d80 <= 0.0f,
                                 "GF80/80 diverged from independent implementation");
    }
}

// GF30/85 uses different published GF variants on each side: this engine
// interpolates GF over tissue a-values (the classic per-tissue model),
// paliers over depth (a planner convention). Schedules legitimately differ;
// per-tissue gfLow=30 on the fast compartments keeps ours systematically
// LONGER here. This band is a variant-behavior guard, not an identity claim.
void test_pub_gf3085_variant_documented() {
    for (int i = 0; i < NENT; ++i) {
        const Entry& e = kEntries[i];
        const float d = runStops(e, 30.0f, 85.0f) - e.paliers30_85;
        TEST_ASSERT_TRUE_MESSAGE(d >= -2.0f && d <= 16.0f,
                                 "GF30/85 schedule moved outside documented variant band");
    }
}

// The published picture is int16, and the engine's own range is wider than that:
// the widest tissue load setTissueState will accept is kSeedMaxBar = 100 bar,
// which is a ceiling of about 950 m — past the 327.67 m an int16 centimetre field
// holds. Reproduced on host 2026-11-27 (all 16 compartments at the seed maximum,
// the CNS clock at its cap, 60 m absolute): viewFull wrote ceilingCm = 29800 and
// stop0Depth = 29864, both the wrapped remainder of a much larger number.
//
// No dive reaches this — the sensor bottoms at ~25 bar absolute (~250 m), so the
// only route is a continuity record that is well-formed and wrong. The guard is
// the net for that route, and the direction matters: 0 or a negative ceiling is
// what `D,` and the dive file both read as "no decompression obligation", so a
// wrap turns the largest obligation the engine can compute into permission to
// ascend. Saturation keeps the obligation and names the column's own limit.
void test_the_published_view_saturates_rather_than_wraps() {
    Config cfg;
    Zhl16Ccr z(cfg);
    Zhl16Ccr::TissueState s{};
    for (int i = 0; i < Zhl16Ccr::NCOMP; ++i) s.pt[i] = Zhl16Ccr::kSeedMaxBar;
    s.cnsPct = 999.0;
    TEST_ASSERT_TRUE_MESSAGE(z.setTissueState(s),
                             "the seed the saturation case is built on is no longer legal");

    deco::View v{};
    deco::viewFull(v, z, z.depthToBar(60.0), true, 1.30);

    TEST_ASSERT_EQUAL_INT16_MESSAGE(32767, v.ceilingCm,
                                    "the ceiling column wrapped instead of saturating: 29800 is "
                                    "a plausible shallow ceiling read as permission to ascend");
    TEST_ASSERT_TRUE_MESSAGE(v.ceilingCm > 0,
                             "an impossible ceiling must never read as no obligation");

    // Stops: a wrapped DEPTH would be negative (a stop below the surface, which a
    // handset would draw as a shallower stop than any real one), and a wrapped
    // minute would shorten the stop. Both saturate at the field's limit instead.
    TEST_ASSERT_TRUE_MESSAGE(v.stopCount > 0, "a fully loaded engine must demand stops");
    for (uint8_t i = 0; i < v.stopCount; ++i) {
        // Unity's message is a plain C string, so the stop number is composed — a
        // failure has to say which stop, or the reader has to re-run to find out.
        const std::string which = ", stop " + std::to_string(i);
        TEST_ASSERT_GREATER_THAN_INT16_MESSAGE(0, v.stopDepthCm[i],
                                               ("a demanded stop has a non-positive depth: a "
                                                "wrapped int16 depth" + which).c_str());
        TEST_ASSERT_GREATER_OR_EQUAL_INT16_MESSAGE(0, v.stopMinX10[i],
                                                   ("a demanded stop has negative minutes" + which).c_str());
    }
    // The total is minutes x10: this seed sums 2398.0 min, still inside int16, so
    // this case proves the sum is not being clamped away — only that a wrap cannot
    // happen. The bound is asserted rather than the exact figure, because the
    // figure is the engine's business (pinned by the table cases above).
    TEST_ASSERT_GREATER_OR_EQUAL_INT16_MESSAGE(1, v.decoTotalX10,
                                               "the deco total vanished on the widest legal load");
    TEST_ASSERT_LESS_OR_EQUAL_INT16_MESSAGE(32767, v.decoTotalX10,
                                            "the deco total overflowed its column");

    // The CNS clock: 999 % is inside the column (9990 tenths), and its own cap is
    // the engine's, so the picture can never print a clock the engine refused.
    TEST_ASSERT_EQUAL_INT16_MESSAGE(9990, v.cnsX10, "the CNS clock column lost a decimal");
}
