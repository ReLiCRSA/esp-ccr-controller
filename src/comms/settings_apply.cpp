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
#include <cstring>

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
    cfg.deco.ascentRateM = f("ASC_RATE");
    cfg.deco.ascentRateSlowM = f("ASC_RATE_SLOW");
    cfg.deco.waterDensity = f("WATER_DENSITY");

    return clamps;
}

} // namespace comms
