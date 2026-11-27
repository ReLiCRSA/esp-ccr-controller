// ============================================================================
// settings_apply.cpp — see settings_apply.h.
//
// The clamps are the point of this file. Registry ranges are per-key; the
// dangerous combinations are *between* keys (a high alarm above the injection
// wall, a cutoff below it, a pulse wider than the fuse that is supposed to
// catch it stuck). A SET can therefore be individually legal and still
// invert the safety ordering; the applier enforces the ordering at the last
// moment before the numbers steer the valve, and reports what it clamped so
// the event log and the handset say so out loud.
// ============================================================================
#include "settings_apply.h"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <utility>

namespace comms {

uint8_t applySettings(const settings::Registry& reg, ctrl::Config& cfg) {
    uint8_t clamps = CLAMP_NONE;
    auto f = [&](const char* k) { return reg.getFloat(reg.find(k)); };

    // setpoints + depth bands
    cfg.sp.sp[0] = f("SP1");
    cfg.sp.sp[1] = f("SP2");
    cfg.sp.sp[2] = f("SP3");
    cfg.sp.sp[3] = f("SP4");
    if (!(cfg.sp.sp[0] <= cfg.sp.sp[1] && cfg.sp.sp[1] <= cfg.sp.sp[2] &&
          cfg.sp.sp[2] <= cfg.sp.sp[3])) {
        // The setpoint engine assumes shallow->deep ascending bands; a
        // shuffled ladder would hand the deepest band to the shallowest
        // water. Sort rather than refuse: the diver gets what they asked
        // for in the only order that runs.
        std::sort(cfg.sp.sp, cfg.sp.sp + 4);
        clamps |= CLAMP_SP_ORDER;
    }
    cfg.sp.deep1M = f("D1");
    cfg.sp.deep2M = f("D2");
    cfg.sp.hysteresisM = f("SP_HYST");
    if (cfg.sp.deep1M > cfg.sp.deep2M) {
        // The engine reads the edges as one ascending ladder: `d < D1` is the
        // shallow band, `d > D2` the deep band, and everything between them the
        // middle. Crossed edges turn that middle band inside out and make every
        // depth past the deeper edge select band 0, which is the *lowest*
        // setpoint and the shortest deco — a diver at depth would be told a
        // shallower target gas than the water demands. Swapping the two
        // preserves both depths the diver entered and restores the direction the
        // ladder is supposed to run in.
        //
        // What it does not restore is the band's width: if the two numbers are
        // equal, or closer together than 2*SP_HYST, the middle band collapses to
        // nothing and the engine steps straight from band 0 to band 2. That is
        // not a runaway — the hysteresis is measured against the same edges it
        // switches on, so a single value simply means two bands, not an
        // oscillation — so nothing is invented here to prop the middle band up.
        std::swap(cfg.sp.deep1M, cfg.sp.deep2M);
        clamps |= CLAMP_BAND_ORDER;
    }

    // alarms
    cfg.alarms.lowBar = f("ALLO_LOW");
    cfg.alarms.highBar = f("ALLO_HIGH");
    cfg.alarms.cnsAlarmPct = static_cast<float>(reg.getInt(reg.find("CNS_ALARM")));
    cfg.alarms.vbatMin = f("VBAT_MIN");

    // injection
    cfg.inj.pulseMs = static_cast<uint16_t>(reg.getInt(reg.find("INJ_PULSE_MS")));
    cfg.inj.maxDutyPct = static_cast<uint8_t>(reg.getInt(reg.find("INJ_MAX_DUTY")));
    cfg.inj.cutoffBar = f("ALLO_CUTOFF");

    // ordering clamps, against whatever the compile-time wall is
    if (cfg.alarms.highBar >= cfg.inj.highWallBar) {
        cfg.alarms.highBar = cfg.inj.highWallBar - 0.05f;
        clamps |= CLAMP_HIGH_ALARM;
    }
    if (cfg.inj.cutoffBar <= cfg.inj.highWallBar + 0.05f) {
        cfg.inj.cutoffBar = cfg.inj.highWallBar + 0.15f;
        clamps |= CLAMP_CUTOFF;
    }
    if (cfg.inj.pulseMs >= cfg.inj.maxOnMs) {
        // A pulse as wide as the stuck-valve fuse would make every legitimate
        // pulse look like a failure.
        cfg.inj.pulseMs = static_cast<uint16_t>(cfg.inj.maxOnMs - 100);
        clamps |= CLAMP_PULSE_MS;
    }
    {
        const uint32_t need = cfg.pulseTest.pulses * cfg.inj.pulseMs;
        const uint32_t have = static_cast<uint32_t>(cfg.inj.maxDutyPct) * cfg.inj.windowMs / 100u;
        if (have < need) {
            cfg.inj.maxDutyPct = static_cast<uint8_t>(
                (need + cfg.inj.windowMs / 2) / (cfg.inj.windowMs / 100u));
            clamps |= CLAMP_DUTY;
        }
    }

    // voting
    cfg.voting.driftBar = f("CELL_DRIFT_BAR");

    // cellproof
    cfg.cellproof.enabled = reg.getBool(reg.find("CELLPROOF_EN"));
    cfg.cellproof.depthM = f("CELLPROOF_DEPTH");
    cfg.cellproof.spBar = f("CELLPROOF_SP");
    cfg.cellproof.minPassBar = f("CELLPROOF_MIN");
    cfg.cellproof.timeoutMs =
        static_cast<uint32_t>(reg.getInt(reg.find("CELLPROOF_TMO"))) * 1000u;
    cfg.cellproof.okMaxSpBar = f("CELLPROOF_MAXSP");

    // pre-dive battery floor follows the same VBAT_MIN the alarm uses
    cfg.predive.vbatMin = cfg.alarms.vbatMin;

    // deco
    cfg.deco.gfLow = f("GF_LOW");
    cfg.deco.gfHigh = f("GF_HIGH");
    if (cfg.deco.gfLow > cfg.deco.gfHigh) {
        // The gradient factor line interpolates across the tissue set from the
        // fast compartment down to the slow one, so the two numbers are an
        // ordered pair, not a range: the low edge is the *strict* ceiling on the
        // tissues that load and unload quickest, and the high edge is the
        // permissive one on the tissues that barely move. Crossed edges hand the
        // permissive factor to the fast tissues, and since the ceiling is taken
        // over all compartments, the deepest stop a diver is allowed to miss is
        // whichever compartment ends up most permissive — the ceiling drifts
        // deeper and deco silently shortens.
        //
        // A swap would not fix that: it would put the strict factor on the slow
        // tissues, where it does almost nothing, while the fast tissues keep the
        // permissive one. So the low edge follows the high one instead, and both
        // tissues run on the stricter of the two numbers the diver entered. Both
        // edges stay inside the per-key registry range, so this is a clamp the
        // handset can read back rather than a value it never sent.
        cfg.deco.gfLow = cfg.deco.gfHigh;
        clamps |= CLAMP_GF_ORDER;
    }
    cfg.deco.ascentRateM = f("ASC_RATE");
    cfg.deco.ascentRateSlowM = f("ASC_RATE_SLOW");
    cfg.deco.waterDensity = f("WATER_DENSITY");

    return clamps;
}

const char* clampName(uint8_t bit) {
    switch (bit) {
        case CLAMP_HIGH_ALARM: return "HIGH_ALARM";
        case CLAMP_CUTOFF:     return "CUTOFF";
        case CLAMP_PULSE_MS:   return "PULSE_MS";
        case CLAMP_DUTY:       return "DUTY";
        case CLAMP_SP_ORDER:   return "SP_ORDER";
        // The two ordering clamps read as what they did to the diver's numbers,
        // not as a generic "clamped": a diver who set GF_LOW=90 over GF_HIGH=50
        // needs to see that his gradient schedule was the thing refused.
        case CLAMP_GF_ORDER:   return "GF_ORDER";
        case CLAMP_BAND_ORDER: return "BAND_ORDER";
        default:               return nullptr;
    }
}

size_t clampList(uint8_t mask, char* out, size_t cap) {
    if (out == nullptr || cap == 0) return 0;
    out[0] = '\0';
    size_t written = 0;
    size_t used = 0;
    for (uint8_t bit = 1; bit != 0; bit <<= 1) {
        if ((mask & bit) == 0) continue;
        const char* name = clampName(bit);
        if (name == nullptr) continue;   // a bit with no name has nothing to say
        // Measure first. snprintf would otherwise write the first two characters
        // of the next name into a full buffer and leave a dangling comma on the
        // console — a truncated list has to stop at a whole name.
        const int need = snprintf(nullptr, 0, written ? ", %s" : "%s", name);
        if (need < 0 || used + static_cast<size_t>(need) >= cap) break;
        snprintf(out + used, cap - used, written ? ", %s" : "%s", name);
        used += static_cast<size_t>(need);
        written++;
    }
    return written;
}

} // namespace comms
