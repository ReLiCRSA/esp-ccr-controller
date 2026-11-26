// ============================================================================
// rig.cpp — see rig.h. All control behaviour lives in ctrl::Loop; this file is
// only the bolt between it and the gas model.
// ============================================================================
#include "sim/rig.h"

namespace sim {
namespace {
constexpr float kTickS = Rig::kTickMs / 1000.0f;
}

Rig::Rig(const RigOpts& opts)
    : opts_(opts), world_(opts.world), loop_(opts), vbat_(opts.vbat) {}

void Rig::setCellsFitted(uint8_t mask) {
    opts_.presentMask = mask;
    loop_.setConfig(opts_);
}

void Rig::tick() {
    const uint32_t now = tickMs_ + kTickMs;
    tickMs_ = now;

    // ---- physics advances under the valve state commanded last cycle -------
    // A stuck-open solenoid flows whether or not the controller asks: the
    // control chain sees the consequence (ppO2 rising, no command), which is
    // exactly how it has to detect it.
    world_.step(kTickS, lastSolOn_);

    // ---- the same SensorData the firmware would fill in --------------------
    ctrl::SensorData sd;
    for (int i = 0; i < 3; i++)
        sd.cells[i] = world_.cellValue(static_cast<uint8_t>(i), kTickS);
    // The depth leg of the world can fail now (World::DepthFault), and the way it
    // fails is the way the driver fails on the board: the last measured sample
    // keeps being delivered and the health flag goes false. Not 0.0f — that
    // substitution IS the C1 bug, and SensorData.depthM in control/loop.h now
    // says it is forbidden. The physics never follows the fault: the world stays
    // as deep as the profile says (and keeps burning SAC at that pressure),
    // which is the whole point of injecting it.
    sd.depthM = world_.depthReadingM();
    sd.pressureValid = world_.depthValid();
    sd.cellsFresh = true;
    sd.vbat = vbat_;

    const ctrl::State st = loop_.cycle(sd, now);
    lastState_ = st;  // published state, for state() — see rig.h
    lastSolOn_ = st.solenoid || world_.solenoidStuckOpen;

    // ---- running envelope, for scenario asserts ----------------------------
    // Only while the vote is real: a cold start (no samples yet) or a failed
    // sensor set legitimately reads 0 and must not pollute the dive envelope.
    if (st.haveVote) {
        // The envelope counts MEASUREMENTS: st.shownPpo2 is the held display
        // number, and a held value must not be counted again as if it were fresh.
        if (st.shownPpo2 > maxVotedSeen) maxVotedSeen = st.shownPpo2;
        if (st.shownPpo2 < minVotedSeen) minVotedSeen = st.shownPpo2;
    }
    const float truePpo2 = world_.ppo2True();
    if (truePpo2 > maxTrueSeen) maxTrueSeen = truePpo2;
    if (truePpo2 < minTrueSeen) minTrueSeen = truePpo2;
}

}  // namespace sim
