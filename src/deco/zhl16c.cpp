// ============================================================================
// zhl16c.cpp — see header for parameter source and GF formulation.
// ============================================================================
#include "zhl16c.h"

#include <algorithm>
#include <cmath>

#include "util/depthscale.h"

namespace deco {

namespace {

// ZH-L16C air/nitrox (Buhlmann, Tauchmedizin 2002). Linear form:
//   tolerated tissue pressure at ambient P: a + b*P.
// Sourced from the tabulated public transcription of the 2002 parameter set
// (see header). DO NOT EDIT without citing a source.
constexpr int NC = Zhl16Ccr::NCOMP;

constexpr double kHalfTimeMin[NC] = {
    5.0, 8.0, 12.5, 18.5, 27.0, 38.3, 54.3, 77.0,
    109.0, 146.0, 187.0, 239.0, 305.0, 390.0, 498.0, 635.0};
constexpr double kA[NC] = {
    1.1696, 1.0, 0.8618, 0.7562, 0.6200, 0.5043, 0.4410, 0.4000,
    0.3750, 0.3500, 0.3295, 0.3065, 0.2835, 0.2610, 0.2480, 0.2327};
constexpr double kB[NC] = {
    0.5578, 0.6514, 0.7222, 0.7825, 0.8126, 0.8434, 0.8693, 0.8910,
    0.9092, 0.9222, 0.9319, 0.9403, 0.9477, 0.9544, 0.9602, 0.9653};

constexpr double kLn2 = 0.6931471805599453;

} // namespace

Zhl16Ccr::Zhl16Ccr(const Config& cfg) : cfg_(cfg) {
    computeGf();
    reset();
}

void Zhl16Ccr::computeGf() {
    // gf per compartment: linear interpolation on the a-coefficient.
    // Config carries GF as percent (30 = 30%); the math uses fractions 0..1.
    // Fastest tissue (largest a) gets gfLow; slowest gets gfHigh.
    const double aFast = kA[0], aSlow = kA[NCOMP - 1];
    const double span = (aFast - aSlow);
    for (int i = 0; i < NCOMP; ++i) {
        const double t = (aFast - kA[i]) / span;
        gf_[i] = static_cast<float>((cfg_.gfLow + (cfg_.gfHigh - cfg_.gfLow) * t) / 100.0);
    }
}

void Zhl16Ccr::reset() {
    // Resting tissue load: inspired ppN2 breathing air at the surface.
    const double ppN2 =
        (cfg_.pSurfaceBar - cfg_.ph2oBar) * 0.79 - cfg_.pco2Bar;
    for (int i = 0; i < NCOMP; ++i) pt_[i] = ppN2;
    cnsPct_ = 0.0;
}

double Zhl16Ccr::barToDepthM(double bar) const {
    // One conversion for the whole board (util/depthscale.h): the depth this
    // engine plans against is the depth the control path measured, to the last
    // digit, from the same density setting.
    return depthscale::depthFromAmbientBar(bar, cfg_.pSurfaceBar, cfg_.waterDensity);
}

double Zhl16Ccr::depthToBar(double depthM) const {
    return depthscale::ambientBarAtDepthM(depthM, cfg_.pSurfaceBar, cfg_.waterDensity);
}

// The two halves of integration, deliberately separable. Neither one keeps any
// state about time: step() never stored a timestamp or carried a leftover dt,
// the CNS term uses the caller's dt once and the tissue loop subdivides a copy
// of it. So step() == stepCns() + stepTissues() for the same dt, by
// construction — and a caller that runs only one half for a while cannot make
// the other half double-count or lose time when it comes back.
void Zhl16Ccr::step(double dtSec, double pAmbBar, double ppO2Bar) {
    stepCns(dtSec, ppO2Bar);
    stepTissues(dtSec, pAmbBar, ppO2Bar);
}

void Zhl16Ccr::stepCns(double dtSec, double ppO2Bar) {
    if (dtSec <= 0.0) return;
    // CNS clock accumulates on real time regardless of substep size. ppO2 is its
    // only input: no ambient pressure, no depth term anywhere in cnsLimitMin.
    const double lim = cnsLimitMin(ppO2Bar);
    cnsPct_ += 100.0 * (dtSec / 60.0) / lim;
    cnsPct_ = std::min(cnsPct_, 999.0);
}

void Zhl16Ccr::stepTissues(double dtSec, double pAmbBar, double ppO2Bar) {
    if (dtSec <= 0.0) return;
    const double ppN2 = std::max(0.0, pAmbBar - cfg_.ph2oBar - cfg_.pco2Bar - ppO2Bar);

    // Subdivide for stability when the caller uses coarse steps.
    while (dtSec > 0.0) {
        const double h = std::min(dtSec, 10.0);
        dtSec -= h;
        for (int i = 0; i < NCOMP; ++i) {
            const double k = kLn2 / (kHalfTimeMin[i] * 60.0); // per second
            const double f = 1.0 - std::exp(-k * h);          // exact for const ppN2
            pt_[i] += (ppN2 - pt_[i]) * f;
        }
    }
}

double Zhl16Ccr::ceilingBarFrom(const double pt[NCOMP], const float gf[NCOMP]) {
    // GF condition pt <= P + gf*(M(P) - P) with M(P) = a + P/b, solved for P.
    // Algebraically identical to Subsurface core/deco.cpp's
    // (b*T - gf*a*b) / ((1-b)*gf + b) and to (T-a)*b at gf = 1.
    double pc = 0.0;
    for (int i = 0; i < NCOMP; ++i) {
        const double g = gf[i];
        const double tol = kB[i] * (pt[i] - g * kA[i]) / (kB[i] + g * (1.0 - kB[i]));
        pc = std::max(pc, tol);
    }
    return pc;
}

double Zhl16Ccr::ceilingBar() const { return ceilingBarFrom(pt_, gf_); }

double Zhl16Ccr::ceilingM() const {
    const double d = barToDepthM(ceilingBar());
    return d > 0.0 ? d : 0.0;
}

double Zhl16Ccr::ndlMinutes(double pAmbBar, double ppO2Bar, double maxMin) const {
    // "No decompression limit": the time that can still be spent HERE before
    // a direct ascent at the configured rates/gas plan would break the
    // ceiling somewhere on the way up. Feasibility is monotonic in time at
    // depth, so binary-search it; ~12 ascent simulations, each ~90 tissue
    // steps — cheap enough for a 1 Hz telemetry publish.
    const double ppN2Bottom = std::max(0.0, pAmbBar - cfg_.ph2oBar - cfg_.pco2Bar - ppO2Bar);
    auto d2b = [&](double m) {
        return depthscale::ambientBarAtDepthM(m, cfg_.pSurfaceBar, cfg_.waterDensity);
    };

    auto ascentClears = [&](const double t0[NCOMP]) {
        double sim[NCOMP];
        std::copy(t0, t0 + NCOMP, sim);
        auto ppN2At = [&](double m) {
            const double p = d2b(m);
            const double ppO2 = cfg_.decoAirFraction > 0.0f
                                    ? cfg_.decoAirFraction * (p - cfg_.ph2oBar)
                                    : (m >= cfg_.deepSwitchM ? cfg_.decoSpDeepBar
                                                             : cfg_.decoSpShallowBar);
            return std::max(0.0, p - cfg_.ph2oBar - cfg_.pco2Bar - ppO2);
        };
        double depth = barToDepthM(pAmbBar);
        while (depth > 1e-9) {
            const double rate = (depth <= cfg_.slowAscentDepthM) ? cfg_.ascentRateSlowM
                                                                 : cfg_.ascentRateM;
            if (rate <= 0.0f) return false;  // cannot ascend: never NDL-clear
            double hSec = 5.0;
            double drop = rate / 60.0 * hSec;
            if (drop > depth) { drop = depth; hSec = drop * 60.0 / rate; }
            const double ppN2 = ppN2At(depth - drop * 0.5);
            for (int i = 0; i < NCOMP; ++i) {
                const double k = kLn2 / (kHalfTimeMin[i] * 60.0);
                sim[i] += (ppN2 - sim[i]) * (1.0 - std::exp(-k * hSec));
            }
            depth -= drop;
            if (ceilingBarFrom(sim, gf_) > d2b(depth) + 1e-4) return false;
        }
        return true;
    };

    auto clearedFor = [&](double minutesAtDepth) {
        double sim[NCOMP];
        for (int i = 0; i < NCOMP; ++i) {
            const double k = kLn2 / (kHalfTimeMin[i] * 60.0);
            sim[i] = pt_[i] + (ppN2Bottom - pt_[i]) * (1.0 - std::exp(-k * minutesAtDepth * 60.0));
        }
        return ascentClears(sim);
    };

    if (!clearedFor(0.0)) return 0.0;   // already obligated: a stop is due now
    if (clearedFor(maxMin)) return maxMin;
    double lo = 0.0, hi = maxMin;
    for (int it = 0; it < 12 && hi - lo > 0.25; ++it) {
        const double mid = 0.5 * (lo + hi);
        if (clearedFor(mid)) lo = mid; else hi = mid;
    }
    return hi;  // conservative: first time at which the direct ascent breaks
}

Zhl16Ccr::Schedule Zhl16Ccr::schedule(double pAmbBar) const {
    Schedule sched{};
    double sim[NCOMP];
    std::copy(pt_, pt_ + NCOMP, sim);

    const double grid = cfg_.stopGridM;
    auto d2b = [&](double m) {
        return depthscale::ambientBarAtDepthM(m, cfg_.pSurfaceBar, cfg_.waterDensity);
    };

    auto ceilingDepth = [&]() -> double {
        const double bar = ceilingBarFrom(sim, gf_);
        return barToDepthM(bar);   // the same depth the rest of the board publishes
    };

    const double startM = std::max(0.0, barToDepthM(pAmbBar));
    double depth = std::floor(startM / grid + 1e-9) * grid; // first grid stop <= bottom
    float totalMin = 0.0f, stopMin = 0.0f;
    sched.capped = false;

    auto ppO2At = [&](double m) {
        if (cfg_.decoAirFraction > 0.0f)  // OC gas (published-table replication, bailout planning)
            return static_cast<double>(cfg_.decoAirFraction) * (d2b(m) - cfg_.ph2oBar);
        return static_cast<double>(m >= cfg_.deepSwitchM ? cfg_.decoSpDeepBar
                                                         : cfg_.decoSpShallowBar);
    };
    auto rateAt = [&](double fromM, double toM) {
        return toM <= cfg_.slowAscentDepthM && fromM <= cfg_.slowAscentDepthM + grid
                   ? cfg_.ascentRateSlowM
                   : cfg_.ascentRateM;
    };
    auto simStep = [&](double dtMin, double atDepth) {
        const double p = d2b(atDepth);
        const double ppN2 = std::max(0.0, p - cfg_.ph2oBar - cfg_.pco2Bar - ppO2At(atDepth));
        int sub = std::max(1, (int)(dtMin * 60.0 / 15.0)); // <=15 s substeps
        for (int s = 0; s < sub; ++s) {
            const double h = dtMin * 60.0 / sub;
            for (int i = 0; i < NCOMP; ++i) {
                const double k = kLn2 / (kHalfTimeMin[i] * 60.0);
                sim[i] += (ppN2 - sim[i]) * (1.0 - std::exp(-k * h));
            }
        }
    };

    int guard = 0;
    while (depth > 0.001 && guard++ < 1000) {
        const double ceilM = ceilingDepth();
        const double target = std::max(0.0, depth - grid);

        if (ceilM > depth + 0.01) {
            // Tissues demand a deeper stop than where we are (possible right
            // after a fast descent): descend to the grid point at the ceiling.
            depth = std::ceil(ceilM / grid) * grid;
            continue;
        }
        if (ceilM <= target + 0.01) {
            const double rate = rateAt(depth, target);
            const double dt = grid / rate;
            simStep(dt, depth); // conservative: load at the deeper depth
            totalMin += static_cast<float>(dt);
            depth = target;
            continue;
        }
        // Must hold at `depth` until ceiling drops to `target` (cap 180 min).
        float held = 0.0f;
        bool capped = false;
        while (ceilingDepth() > target + 0.01) {
            simStep(0.5, depth);
            held += 0.5f;
            if (held >= 180.0f) { capped = true; break; }
        }
        totalMin += held;
        stopMin += held;
        if (capped) { sched.capped = true; }
        if (sched.count < 16 &&
            (sched.count == 0 || sched.stops[sched.count - 1].depthM != (float)depth)) {
            sched.stops[sched.count++] = {static_cast<float>(depth), held};
        } else if (sched.count > 0) {
            sched.stops[sched.count - 1].minutes += held;
        }
    }
    sched.totalTimeMin = totalMin;
    sched.stopTimeMin = stopMin;
    return sched;
}

double Zhl16Ccr::cnsLimitMin(double ppO2) {
    // NOAA single-exposure limits (min), as published (NOAA Diving Manual;
    // the same table with matching %CNS/min column in the Deco_Package PDF,
    // tech-diving.ru — 1/limit equals its %/min column row for row). Below
    // ppO2 0.6 no accrual is counted (0.6 itself: 720 min). Above 1.6 there
    // is no safe limit; we accumulate at the 1.6 rate (45 min) and the
    // alarm layer (high ppO2 hard cutoff) owns protection.
    struct Row { double p; double min; };
    static const Row rows[] = {
        {0.00, 1e9}, {0.50, 1e9}, {0.60, 720.0}, {0.70, 570.0},
        {0.80, 450.0}, {0.90, 360.0}, {1.00, 300.0}, {1.10, 240.0},
        {1.20, 210.0}, {1.30, 180.0}, {1.40, 150.0}, {1.50, 120.0},
        {1.60, 45.0}, {9.99, 45.0},
    };
    for (size_t i = 1; i < sizeof(rows) / sizeof(rows[0]); ++i) {
        if (ppO2 <= rows[i].p) return rows[i].min;
    }
    return 45.0;
}

} // namespace deco
