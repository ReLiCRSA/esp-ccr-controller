// ============================================================================
// depthscale.h — the ONE pressure <-> depth conversion on this board
//
// Two paths convert depth, and they are the same physics:
//
//   * the control path turns a bar reading into the depth a diver reads, the
//     depth that selects the setpoint bands, feeds the deco engine and is
//     printed in every `T,` row (`main.cpp`, sensorTask);
//   * the deco engine turns a depth into ambient pressure for tissue
//     modelling, and a ceiling's pressure back into a depth to publish
//     (`deco/zhl16c.cpp`, three places).
//
// Until 2026-11-24 each re-derived the arithmetic for itself — `BAR_PER_M_SEAWATER`
// in `include/config/defaults.h` for one, `rhoG = waterDensity * 1000.0 *
// 9.80665` written out three times in the deco engine for the other — in
// different types, from different density values, with no host test on either
// side. What that cost, measured on the bench the day the first depth sensor
// became valid: the macro divided a **kg/L** density by 100000, which is only
// right for kg/m3, so the constant came out 1000x too small and an open room
// read `T,37.04` with the setpoint ladder walking to SP3 1.00 bar — while the
// deco engine's private copy of the same formula was correct. A defect that
// lives in two places, one of which is right, is invisible to any comparison
// the machine can make with itself.
//
// So: one formula here, host-tested against LITERALS in test/test_depthscale (a
// test that derived its expected depth from this function would inherit an
// error here instead of catching one), called by both paths. And one DENSITY:
// the `WATER_DENSITY` setting. The second half of that old bug was that the two
// paths read different density sources — the telemetry depth used the
// compile-time `WATER_DENSITY_KGL` while the deco engine used
// `cfg.waterDensity`, which `SET WATER_DENSITY` overwrites. A diver setting
// fresh water (1.000, the schema allows 0.990..1.100) would have had the
// ceiling and NDL computed on one scale and the depth on the wire, and the
// setpoint bands, on another: about 2.5% at any depth, and neither side wrong
// in its own terms.
//
// Everything here is constexpr on purpose: no runtime cost, no float printf,
// nothing that grows a stack frame on the tasks that call it.
// ============================================================================
#pragma once

namespace depthscale {

// Bar of water column per metre of depth, for a density in kg/L.
//
// Units, written out because getting them wrong is silent and 1000x:
//   density [kg/L] * 1000 [L per m3]      -> kg/m3
//              * 9.80665 [m/s2]          -> Pa per m  (only valid in kg/m3)
//              / 1e5 [Pa per bar]        -> bar per m
// At 1.025 kg/L (sea) that is 0.1005182 bar/m: 1 bar of sea water is 9.948 m,
// and 100 hPa of weather is about 1 m. Fresh water (1.000) is 0.0980665 bar/m,
// 1 bar = 10.197 m.
inline constexpr double barPerM(double densityKgL) {
    return densityKgL * 1000.0 * 9.80665 / 1e5;
}

// Depth from a pressure DIFFERENCE (already relative to the surface plane).
inline constexpr double depthFromBarDelta(double barDelta, double densityKgL) {
    return barDelta / barPerM(densityKgL);
}

// Absolute ambient pressure at a depth below the surface plane.
inline constexpr double ambientBarAtDepthM(double depthM, double surfaceBar,
                                           double densityKgL) {
    return surfaceBar + depthM * barPerM(densityKgL);
}

// Depth from an absolute ambient pressure. Negative above the plane: the deco
// engine legitimately asks this question (a ceiling can sit above the surface),
// so it gets the honest number.
inline constexpr double depthFromAmbientBar(double ambientBar, double surfaceBar,
                                            double densityKgL) {
    return (ambientBar - surfaceBar) / barPerM(densityKgL);
}

// What a diver reads: never negative. Above the reference plane the answer is
// 0 m, because a negative depth on a handset is a weather change or an
// altitude setting, not something to dose or plan a stop against.
inline constexpr double depthFromSensorBar(double ambientBar, double surfaceBar,
                                           double densityKgL) {
    const double d = depthFromAmbientBar(ambientBar, surfaceBar, densityKgL);
    return d > 0.0 ? d : 0.0;
}

}  // namespace depthscale
