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
    : world_(opts.world), loop_(opts), vbat_(opts.vbat) {}

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
    sd.depthM = world_.depthM();
    sd.pressureValid = true;  // the sim's pressure bus never fails; see test_pressure
    sd.cellsFresh = true;
    sd.vbat = vbat_;

    const ctrl::State st = loop_.cycle(sd, now);
    lastSolOn_ = st.solenoid || world_.solenoidStuckOpen;

    // ---- running envelope, for scenario asserts ----------------------------
    // Only while the vote is real: a cold start (no samples yet) or a failed
    // sensor set legitimately reads 0 and must not pollute the dive envelope.
    if (st.haveVote) {
        if (st.votedBar > maxVotedSeen) maxVotedSeen = st.votedBar;
        if (st.votedBar < minVotedSeen) minVotedSeen = st.votedBar;
    }
    const float truePpo2 = world_.ppo2True();
    if (truePpo2 > maxTrueSeen) maxTrueSeen = truePpo2;
    if (truePpo2 < minTrueSeen) minTrueSeen = truePpo2;
}

}  // namespace sim
