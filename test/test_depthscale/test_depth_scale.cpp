// ============================================================================
// test_depth_scale.cpp — the scale itself, pinned from outside the code.
//
// Every expected number in this file is a LITERAL, worked out from the physics
// (rho [kg/L] * 1000 * 9.80665 / 1e5 bar per metre), never from
// `util/depthscale.h` or from a macro. That is the whole point: a test that
// derived its expected depth from the constant under test would inherit an
// error instead of catching one, and that is why the 1000x unit error in
// `BAR_PER_M_SEAWATER` (2026-11-24, `T,37.04` at the surface of an open room)
// had no test to fail — nothing referenced the constant, and the line that
// converts lives in main.cpp, outside every tested unit.
//
// What is pinned:
//   * the scale, against hand-worked decimal literals, at two densities;
//   * the bench's own surface reading (0.037 m, not 37 m);
//   * the round trip depth -> ambient bar -> depth;
//   * that the deco ENGINE's own conversion agrees with the shared one, at both
//     densities — the tripwire for the two paths parting company again;
//   * that there is ONE density in play: the settings default, the deco
//     engine's default and the build default agree;
//   * that neither path writes its own arithmetic any more (a source scan: this
//     is the only place in src/ that states g), and that every depthscale call
//     is handed the CONFIGURED density rather than a literal.
//
// One case that did NOT survive review, so the reason is on the record: a
// "density reaches the ascent planner" probe (plan the same 30 m profile at
// 1.025/1.000/1.070 and require the published ceiling and total deco time to
// move). It is a true property — at 30 m the ceiling does shift, 11.15 m at sea
// versus 11.43 m fresh — but no mutation could make it the ONLY case that
// failed: hard-coding the density at any subset of the planner's call sites was
// already caught by the ruler test or by the scan. A check that cannot be the
// only witness is decoration, so it was dropped; the coverage it was reaching
// for is stated in PLAN 16 instead.
// ============================================================================
#include <unity.h>

#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>

#include "config/defaults.h"
#include "comms/settings.h"
#include "deco/zhl16c.h"
#include "util/depthscale.h"

namespace {

// Hand-worked, not computed from the code:
//   1.025 kg/L -> 1025 * 9.80665 = 10051.81625 Pa/m -> 0.1005181625 bar/m
//   1 bar / 0.1005181625 bar/m = 9.94844 m
//   0.1  bar / ...              = 0.994844 m   ("100 hPa is about a metre")
//   1.000 kg/L -> 9806.65 Pa/m -> 0.0980665 bar/m; 1 bar = 10.19716 m
constexpr double kBarPerMSea = 0.1005181625;
constexpr double kMetresPerBarSea = 9.94844;
constexpr double kBarPerMFresh = 0.0980665;
constexpr double kMetresPerBarFresh = 10.19716;

const double kDepths[] = {0.5, 2.0, 2.5, 3.0, 10.0, 30.0, 40.0, 60.0};
const double kDensities[] = {1.025, 1.000, 1.070};   // sea, fresh, saltier

std::string readFile(const char* path) {
    FILE* f = std::fopen(path, "rb");
    if (!f) return "";
    std::string out;
    char buf[4096];
    size_t n = 0;
    while ((n = std::fread(buf, 1, sizeof(buf), f)) > 0) out.append(buf, n);
    std::fclose(f);
    return out;
}

}  // namespace

void test_the_scale_is_the_physics_not_the_constant() {
    // Sea water, the density this rig ships with.
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(1e-6, kBarPerMSea, depthscale::barPerM(1.025),
                                      "bar per metre of sea water");
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(1e-3, kMetresPerBarSea,
                                      depthscale::depthFromBarDelta(1.0, 1.025),
                                      "one bar of sea water, in metres");
    // "100 hPa is about a metre": the rule of thumb a diver checks a depth
    // reading against. A 1000x slip cannot survive either assertion.
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(1e-4, 0.994844,
                                      depthscale::depthFromBarDelta(0.1, 1.025),
                                      "100 hPa of water, in metres");
    // Fresh water: a different number from the same formula. If a density ever
    // stops reaching the conversion, this is the case that notices.
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(1e-6, kBarPerMFresh, depthscale::barPerM(1.000),
                                      "bar per metre of fresh water");
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(1e-3, kMetresPerBarFresh,
                                      depthscale::depthFromBarDelta(1.0, 1.000),
                                      "one bar of fresh water, in metres");

    // Named explicitly, because this is the bug the file exists for: 1.005e-4
    // bar/m is what a kg/L density divided by 100000 gives, and it put the
    // bench 37 m deep in an open room.
    TEST_ASSERT_GREATER_THAN_FLOAT_MESSAGE(0.05f, depthscale::barPerM(1.025),
                                          "bar/m looks 1000x small — the kg/L vs "
                                          "kg/m3 unit error of 2026-11-24");
}

void test_the_bench_surface_reading_is_centimetres_not_metres() {
    // The actual observation of 2026-11-24, from the boot announce: this bench
    // sits at 1016.7 hPa against SURFACE_PRESSURE_BAR (1.013 fixed), so a
    // correct build reads about 3.7 cm at the surface. The broken constant
    // printed T,37.04 and the setpoint ladder went to SP3 on a bench sitting on
    // a table — hence the magnitude assertion, not only the digits.
    const double d = depthscale::depthFromSensorBar(1.0167, SURFACE_PRESSURE_BAR,
                                                    WATER_DENSITY_KGL);
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.001, 0.0368, d,
                                      "this bench's surface offset in metres");
    TEST_ASSERT_LESS_THAN_FLOAT_MESSAGE(0.1f, d,
                                        "a sane surface depth is centimetres; the "
                                        "1000x scale error read 37 m here");

    // And the floor: above the reference plane (weather, altitude, a sensor
    // drifting high) the diver reads 0 m, while the deco engine is allowed the
    // honest negative — a ceiling can sit above the surface. Different on
    // purpose, and neither path gets to invent its own rule.
    TEST_ASSERT_EQUAL_FLOAT_MESSAGE(0.0f,
                                    depthscale::depthFromSensorBar(1.010, 1.013, 1.025),
                                    "depth on the wire never goes negative");
    TEST_ASSERT_TRUE(depthscale::depthFromAmbientBar(1.010, 1.013, 1.025) < 0.0);
}

void test_depth_round_trips_through_ambient_pressure() {
    for (double density : kDensities) {
        for (double m : kDepths) {
            const double bar = depthscale::ambientBarAtDepthM(m, 1.013, density);
            const double back = depthscale::depthFromAmbientBar(bar, 1.013, density);
            TEST_ASSERT_FLOAT_WITHIN_MESSAGE(1e-4, m, back,
                                              "depth -> bar -> depth must not drift");
        }
    }
}

void test_the_deco_engine_uses_the_same_ruler() {
    // THE regression test for the old shape of this code: the deco engine kept
    // its own copy of the arithmetic in three places — correct, while the
    // telemetry constant was 1000x wrong — and its density came from a
    // different source than the telemetry depth's. If either ever goes back,
    // the ceiling and NDL are computed against a different depth than the one
    // on the `T,` row, and nothing downstream can see it.
    for (double density : kDensities) {
        deco::Config cfg;
        cfg.waterDensity = static_cast<float>(density);
        cfg.pSurfaceBar = 1.013f;
        deco::Zhl16Ccr zhl(cfg);

        for (double m : kDepths) {
            const double want = depthscale::ambientBarAtDepthM(m, 1.013, density);
            TEST_ASSERT_FLOAT_WITHIN_MESSAGE(1e-5, want, zhl.depthToBar(m),
                                              "deco ambient pressure is not the "
                                              "shared conversion");
            TEST_ASSERT_FLOAT_WITHIN_MESSAGE(1e-4, m, zhl.barToDepthM(want),
                                              "deco depth is not the shared conversion");
        }
    }
}

void test_one_density_serves_both_paths() {
    // 1.025 lives in three places: the build default, the settings default a
    // handset can change, and the deco engine's own default. Three chances to
    // disagree, so they are pinned together — a diver who never touches
    // WATER_DENSITY must not be able to end up with two scales by accident.
    settings::Registry reg;
    reg.resetToDefaults();
    const int idx = reg.find("WATER_DENSITY");
    TEST_ASSERT_GREATER_OR_EQUAL_INT_MESSAGE(0, idx,
                                             "WATER_DENSITY must stay in the schema: "
                                             "it is the density both depth paths use");
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(1e-6f, WATER_DENSITY_KGL, reg.def(idx).def,
                                     "the WATER_DENSITY schema default no longer "
                                     "matches the build default");
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(1e-6f, WATER_DENSITY_KGL, reg.getFloat(idx),
                                     "the WATER_DENSITY value at reset no longer "
                                     "matches the build default");
    deco::Config cfg;
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(1e-6, WATER_DENSITY_KGL, cfg.waterDensity,
                                      "the deco engine's default density no longer "
                                      "matches the build default");
}

void test_no_path_writes_its_own_conversion() {
    // The structural claim, enforced on the text: one formula, in one file.
    // The conversion line in main.cpp is not host-compiled (main.cpp is
    // excluded from the native env), so a source scan is the only way the host
    // suite can see it — and a scan that quietly passes when the file cannot be
    // opened is decoration, so a missing file fails loudly.
    const char* kUsers[] = {"src/main.cpp", "src/deco/zhl16c.cpp"};
    for (const char* path : kUsers) {
        const std::string src = readFile(path);
        TEST_ASSERT_MESSAGE(!src.empty(),
                            (std::string("cannot read ") + path +
                             " — run the native tests from the project root").c_str());
        TEST_ASSERT_NOT_NULL_MESSAGE(std::strstr(src.c_str(), "depthscale::"),
                                     (std::string(path) +
                                      " converts pressure and depth without "
                                      "util/depthscale.h").c_str());
        for (const char* forbidden : {"9.80665", "BAR_PER_M_SEAWATER", "rhoG"}) {
            TEST_ASSERT_NULL_MESSAGE(std::strstr(src.c_str(), forbidden),
                                     (std::string(path) + " re-derives the scale (" +
                                      forbidden + ") instead of calling the helper")
                                         .c_str());
        }
        // Calling the helper is not enough — it has to be called WITH the
        // diver's density. A call handed a literal (or the build default) is
        // the two-rulers bug back again, 2.5% apart between 1.000 and 1.025,
        // and no numeric test can see it at the shipping density, because
        // there a hard-coded value happens to be right.
        for (const char* call : {"depthFromSensorBar(", "depthFromAmbientBar(",
                                 "ambientBarAtDepthM("}) {
            for (size_t at = src.find(call); at != std::string::npos;
                 at = src.find(call, at + 1)) {
                TEST_ASSERT_NOT_NULL_MESSAGE(
                    std::strstr(src.substr(at, 200).c_str(), "waterDensity"),
                    (std::string(path) + " calls depthscale::" + call +
                     " without the configured density: the depth on the wire and "
                     "the deco plan would run on different rulers")
                        .c_str());
            }
        }
    }
    // The helper must still hold the constant: the scan above is only worth
    // anything if there is one place left to hold it.
    const std::string helper = readFile("src/util/depthscale.h");
    TEST_ASSERT_MESSAGE(!helper.empty(), "cannot read src/util/depthscale.h");
    TEST_ASSERT_NOT_NULL_MESSAGE(std::strstr(helper.c_str(), "9.80665"),
                                 "util/depthscale.h no longer carries g: where did "
                                 "the conversion go?");
}


int main() {
    UNITY_BEGIN();
    RUN_TEST(test_the_scale_is_the_physics_not_the_constant);
    RUN_TEST(test_the_bench_surface_reading_is_centimetres_not_metres);
    RUN_TEST(test_depth_round_trips_through_ambient_pressure);
    RUN_TEST(test_the_deco_engine_uses_the_same_ruler);
    RUN_TEST(test_one_density_serves_both_paths);
    RUN_TEST(test_no_path_writes_its_own_conversion);
    return UNITY_END();
}
