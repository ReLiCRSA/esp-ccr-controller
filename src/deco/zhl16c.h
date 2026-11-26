// ============================================================================
// zhl16c.h — Buhlmann ZH-L16C for a CCR: tissue N2, gradient-factor ceiling,
// NDL, ascent-schedule prediction and CNS O2 clock.
//
// Parameter source: Buhlmann ZH-L16C air/nitrox set (a,b,half-times verified
// cell-by-cell against the published transcription in "Buhlmann decompression
// algorithm" (Wikipedia, accessed 2026) and the arrays in Subsurface's
// core/deco.cpp). The LINEAR tolerated-pressure form (Buhlmann 1983/86; the
// same expression in Subsurface core/deco.cpp and repied/paliers src/gf.ts):
//     M_i(Pamb) = a_i + Pamb / b_i       (a,b per compartment)
// Gradient factors scale the tolerated supersaturation (Pti - Pamb):
//     Pti - Pamb <= gf_i * (M_i(Pamb) - Pamb)
//   => ceiling pressure:  Pamb = b_i*(Pti - gf_i*a_i) / (b_i + gf_i*(1-b_i))
// gf_i interpolates gfLow (fastest tissue, largest a) -> gfHigh (slowest).
// (An earlier revision here used Ptol = a + b*Pamb with these same a,b —
// WRONG for this parameter set: it puts tolerated pressure BELOW ambient at
// depth and inflated deco 2-3x. Cross-validated 2026-11-23 against the
// published Buhlmann air tables and an independent implementation; see
// test/test_deco. The old form would also assert-fail M >= Pamb.)
//
// CCR inert model: alveolar ppN2 = Pamb - ppO2 - pH2O (- pCO2 if configured).
// pH2O default 0.0627 bar (37 C). pCO2 defaults 0 — omitting it makes ppN2
// HIGHER (more uptake = more deco): the conservative direction, documented
// deliberately. Config.decoAirFraction > 0 switches the SCHEDULE/NDL
// simulation gas to open-circuit air at that fraction (table validation and
// OC bailout planning); 0 = CCR setpoint bands.
//
// Pure C++ (no Arduino) so it builds on ESP32, PC tests, and the simulator.
// ============================================================================
#pragma once

#include <cstdint>

namespace deco {

struct Config {
    float gfLow = 30.0f;            // %
    float gfHigh = 85.0f;           // %
    float pSurfaceBar = 1.013f;     // bar (altitude adjusted)
    float waterDensity = 1.025f;    // kg/l (1.025 sea, 1.0 fresh)
    float ph2oBar = 0.0627f;        // alveolar water vapor
    float pco2Bar = 0.0f;           // alveolar CO2 (0 = conservative default)

    float ascentRateM = 6.0f;       // m/min
    float ascentRateSlowM = 1.0f;   // m/min at or shallower than slowAscentDepthM
    float slowAscentDepthM = 3.0f;  // ...this depth
    float stopGridM = 3.0f;         // stop depth granularity

    // ppO2 assumed during simulated deco ascent:
    float decoSpDeepBar = 1.3f;     // deeper than deepSwitchM
    float decoSpShallowBar = 0.7f;  // shallower
    float deepSwitchM = 21.0f;
    // > 0: the simulated ascent breathes OC gas at this O2 fraction
    // (ppO2 = f*(Pamb - pH2O) at each depth) instead of the SP bands above.
    // 0.21 reproduces the published Buhlmann AIR tables; default 0 = CCR.
    float decoAirFraction = 0.0f;
};

class Zhl16Ccr {
public:
    static constexpr int NCOMP = 16;

    struct Stop { float depthM; float minutes; };
    struct Schedule {
        Stop stops[16];         // 3 m grid from ~45 m = 15 stops max
        int count;
        float totalTimeMin;   // stops + simulated ascent
        float stopTimeMin;    // deco time in stops only
        bool capped;          // a hold hit the per-stop cap: schedule incomplete
    };

    explicit Zhl16Ccr(const Config& cfg);

    /// Re-tune (GF, density, ascent rates) without recomputing tissues from
    /// scratch — a mid-dive GF SET changes the schedule, not the dive.
    void setConfig(const Config& cfg) {
        cfg_ = cfg;
        computeGf();
    }

    void reset();  // tissues equilibrated to surface air (0.21)

    // ---- the part of the engine that has to outlive a power cycle (H1) ------
    // What a dive computer must not do is forget what the last dive left in the
    // diver: a surface interval is minutes, and a reboot between dives used to
    // make the next dive read as if the first never happened (review H1 — "the
    // one gap that can kill without any instrument failing"). Everything needed
    // to seed a dive is HERE, and nothing else is: ceiling, NDL and the ascent
    // schedule are DERIVED from these numbers plus the gradient factors in force,
    // so storing them would let a persisted picture disagree with the GF pair the
    // diver has set now. Loadings and the CNS clock are the whole state.
    //
    // ZH-L16C as implemented here is ONE inert gas (nitrogen): `pt_` is 16
    // compartments, there is no helium array anywhere in this engine, so 16
    // loadings are what round-trips — not 32.
    struct TissueState {
        double pt[NCOMP];   // absolute tissue pressure, bar (same units as tissueBar())
        double cnsPct;      // the CNS O2 clock, %
    };

    /// The copy ControlTask publishes for the persistence layer. Reading it is
    /// the same class of read as the bridge scalars: the engine is owned by the
    /// control thread (rowfill.h N8), so everyone else takes a COPY, never the
    /// live array.
    TissueState tissueState() const;

    /// Seeds the engine from stored state. FALSE and no change at all when the
    /// numbers are not numbers a diver could have produced — a torn record, a
    /// number quantised wrong or one written by a different model must never
    /// become a plausible ceiling. The caller then does what it always did
    /// (reset()) and says out loud that continuity was lost.
    bool setTissueState(const TissueState& s);

    /// Air at this board's configured surface pressure: the resting tissue load.
    /// ONE definition — reset() and surfaceInterval() both aim at it, so a
    /// fast-forward over a surface interval and a fresh reset cannot drift to
    /// two different asymptotes.
    double surfacePpN2Bar() const;

    /// Off-gas the tissues for `elapsedSec` AT THE SURFACE, breathing air: what
    /// a diver did between the record and this dive, when the machine was off.
    ///
    /// This is the engine's own model, not a second one. stepTissues() integrates
    /// a constant ppN2 in <=10 s substeps, each substep multiplying the
    /// under-saturation by (1 - (1 - e^-kh)) = e^-kh; those substeps compose to
    /// e^-k*total EXACTLY (modulo float rounding), so one closed-form step over
    /// the whole interval IS the same integration the engine would do — and it
    /// costs 16 exp() calls instead of elapsed/10 substeps. That is what keeps a
    /// 72-hour surface interval from becoming H2 work on the control task, which
    /// is where a dive start is applied. test_deco pins the equivalence.
    ///
    /// CNS is deliberately untouched: this engine models oxygen-clock
    /// ACCUMULATION only (stepCns, and cnsLimitMin has no decay term anywhere),
    /// so carrying the clock over unchanged is the model's own answer. Inventing
    /// a decay here would seed a dive from a physiology the engine does not run.
    /// elapsed <= 0 does nothing (the caller credits nothing when it cannot
    /// establish an interval — the conservative direction, and stated).
    void surfaceInterval(double elapsedSec);

    /// What setTissueState() refuses: a loading outside what any dive this
    /// hardware can measure could produce. The deepest pressure this sensor
    /// reports is ~25 bar absolute (LPS28DFW bottoms at 250 mbar), so 100 bar is
    /// "a number no dive made", not a physical limit.
    static constexpr double kSeedMaxBar = 100.0;

    // Integrate one real-time step: both halves, tissues AND the CNS clock.
    // pAmbBar = measured absolute pressure, ppO2Bar = VOTED loop ppO2.
    // Internally subdivided (<=10 s substeps). This is the wrapper: it forwards
    // the SAME dt to each half, and neither half keeps any state about time
    // (no timestamp, no leftover accumulator — see stepCns/stepTissues), so
    // calling the wrapper cannot double-count or skip time that the two halves
    // called separately would have seen.
    void step(double dtSec, double pAmbBar, double ppO2Bar);

    /// The CNS clock alone. Its entire input is ppO2 — the NOAA table is a
    /// function of ppO2 and time only, with no ambient or depth term anywhere
    /// (cnsLimitMin below) — because ppO2 comes from the oxygen cells, a
    /// different sensor on a different bus from depth. So a controller that has
    /// lost the depth leg can still count oxygen exposure honestly, and one
    /// that has lost the CELLS cannot (there is nothing to integrate).
    /// Accumulates on real time, not on substeps, exactly as step() always did.
    void stepCns(double dtSec, double ppO2Bar);

    /// The 16 tissue compartments alone, at a given absolute ambient pressure.
    /// CNS untouched. Inert take-up needs both pressure and ppO2
    /// (ppN2 = Pamb - pH2O - pCO2 - ppO2), so this half is the one that dies
    /// with the depth measurement — and the caller decides what pressure to
    /// hand it when that measurement is a held one.
    void stepTissues(double dtSec, double pAmbBar, double ppO2Bar);

    // Ceiling: tolerated ambient pressure across all compartments.
    double ceilingBar() const;
    double ceilingM() const;              // 0 when no stop demanded

    // NDL = minutes that can still be spent at this depth before a direct
    // ascent (config rates and deco gas assumptions) would break the ceiling
    // somewhere on the way up. That is the dive-computer meaning: "time
    // before any stop becomes obligatory" — NOT "time until the ceiling
    // exceeds the current depth" (which is essentially never at CCR SPs and
    // is what the old implementation measured).
    double ndlMinutes(double pAmbBar, double ppO2Bar, double maxMin = 480.0) const;

    // Full ascent schedule from current depth with current tissues.
    Schedule schedule(double pAmbBar) const;

    // CNS O2 single-trigger clock (NOAA exposure table), %.
    double cnsPct() const { return cnsPct_; }
    void resetCns() { cnsPct_ = 0.0; }

    double tissueBar(int i) const { return pt_[i]; }

    double barToDepthM(double bar) const;
    double depthToBar(double depthM) const;
    const Config& config() const { return cfg_; }

private:
    void computeGf();  // per-compartment GF from gfLow/gfHigh

    // returns ceiling computed from a given tissue array (used by const sims)
    static double ceilingBarFrom(const double pt[16], const float gf[16]);
    static double cnsLimitMin(double ppO2);

    Config cfg_;
    double pt_[NCOMP];
    float gf_[NCOMP];
    double cnsPct_ = 0.0;
};

} // namespace deco
