// ============================================================================
// rig.cpp — see rig.h. This is the reference wiring: the firmware ControlTask
// will be assembled in this exact order, so a scenario that passes here tests
// the real interaction, not a paraphrase of it.
// ============================================================================
#include "sim/rig.h"

#include <algorithm>
#include <cmath>

namespace sim {
namespace {
constexpr float kTickS = Rig::kTickMs / 1000.0f;
}

Rig::Rig(const RigOpts& opts)
    : opts_(opts),
      cpCfg_(opts.cp),
      world_(opts.world),
      voting_(opts.voting),
      spEng_(opts.sp),
      injector_(opts.inj),
      alarmMgr_(opts.alarms),
      cpEng_(opts.cp),
      predive_(opts.predive),
      deco_(opts.deco),
      pulseTest_(opts.pulseTest) {
    voting_.setPresentMask(opts_.presentMask);
    deco_.reset();
}

// ---------------------------------------------------------------------------
// The scripted pulse-and-check test that executes the pre-dive SOLENOID step.
// The sequencer owns the schedule; the injector owns the valve; this loop only
// relays: PulseTest asks for a pulse -> injector is told -> the pulses that
// actually left the valve are reported back -> verdict goes to the sequencer.
// ---------------------------------------------------------------------------
void Rig::drivePulseTest() {
    const bool wantTest = predive_.running() && predive_.current() == predive::S_SOLENOID;

    if (wantTest && !ptActive_) {  // step just started
        ptActive_ = true;
        ptDone_ = false;
        ptPulseBase_ = injector_.pulsesTotal();
        pulseTest_.start();
    }
    if (!ptActive_) return;

    cal::PulseTestInput pi;
    for (int i = 0; i < 3; i++) pi.cellsBar[i] = cellBar_[i];
    pi.votedBar = vote_.votedPpo2;
    pi.presentMask = opts_.presentMask;
    pi.pulsesDelivered = injector_.pulsesTotal() - ptPulseBase_;
    pi.nowMs = tickMs_;
    const cal::PulseTestOutput po = pulseTest_.update(pi);

    if (po.requestPulse) injector_.requestTestPulse();

    if (po.done && !ptDone_) {
        ptDone_ = true;
        // The step passes when every fitted cell reacted to oxygen the loop
        // is provably receiving. A solenoid that refuses to fire is a failure
        // too: pulsesDelivered == 0 with no rise says exactly that.
        const bool allOk = (po.passMask & opts_.presentMask) == opts_.presentMask;
        predive_.reportSolTest(allOk, pi.pulsesDelivered);
    }
    if (!wantTest) ptActive_ = false;  // sequencer moved on / aborted
}

void Rig::tick() {
    const uint32_t now = tickMs_ + kTickMs;
    const uint32_t dt = kTickMs;
    tickMs_ = now;

    // ---- 1. physics advances with the valve state commanded last cycle -----
    world_.step(kTickS, lastSolOn_);

    // ---- 2. sensors (what the ADC + calibration chain would report) --------
    for (int i = 0; i < 3; i++)
        cellBar_[i] = world_.cellValue(static_cast<uint8_t>(i), kTickS);

    const float pAmb = world_.pAmbBar();
    const float depthM = world_.depthM();
    // Median then boxcar (control/depthfilter.h): bands and CellProof switch
    // on absolute thresholds, so one spiked sample must not move the setpoint.
    depthSmooth_ = depthF_.update(depthM);
    depthRate_ = std::fabs(depthSmooth_ - depthPrev_) / (dt / 1000.0f);
    depthPrev_ = depthSmooth_;
    const bool depthStable = depthRate_ < 0.02f;  // <1.2 m/min: stuck-open gate

    // ---- 3. voting ---------------------------------------------------------
    vote_ = voting_.update(cellBar_, dt);
    haveVote_ = vote_.mode != voting::Mode::FAILED;

    // ---- 4. pre-dive sequencer + its solenoid step -------------------------
    if (predive_.running()) {
        predive::Inputs pi;
        for (int i = 0; i < 3; i++) pi.cells[i] = cellBar_[i];
        pi.presentMask = opts_.presentMask;
        pi.depthM = depthM;
        pi.vbat = opts_.vbat;
        pi.nowMs = now;
        predive_.update(pi, now);
    }
    drivePulseTest();
    // A test pulse that its step stopped waiting for must not linger in the
    // injector and inject minutes later for no reason.
    if (pulseTest_.state() != cal::TestState::PULSING) injector_.cancelTestPulse();

    // ---- 5. CellProof (mid-dive high-ppO2 cell validation) -----------------
    cellproof::Input ci;
    ci.diveActive = diveActive_;
    ci.depthM = depthSmooth_;
    ci.votedBar = vote_.votedPpo2;
    for (int i = 0; i < 3; i++) ci.cellsBar[i] = cellBar_[i];
    ci.activeMask = vote_.activeMask;
    ci.nowMs = now;
    cpOut_ = cpEng_.update(ci);

    // ---- 6. setpoint selection (+ CellProof override/clamp) ----------------
    spEng_.setClamp(cpOut_.clampActive ? cpCfg_.okMaxSpBar : 0.0f);
    const sp::Result sp = spEng_.update(depthSmooth_);
    spTarget_ = sp.value;
    if (cpOut_.requestSpOverride) spTarget_ = cpCfg_.spBar;  // test demand
    else if (cpOut_.clampActive) spTarget_ = std::min(spTarget_, cpCfg_.okMaxSpBar);

    // ---- 7. injector -------------------------------------------------------
    inj::Input ii;
    ii.nowMs = now;
    ii.dtMs = dt;
    ii.votedBar = vote_.votedPpo2;
    ii.setpointBar = spTarget_;
    // A cell excluded from the vote because it reads LOW is not automatically
    // safe to ignore: if it were the honest one, the loop is heading into O2
    // toxicity and its reading still has to be able to stop injection. Every
    // fitted cell keeps a veto on the high side.
    ii.highVetoBar = ii.votedBar;
    for (int i = 0; i < 3; i++)
        if (opts_.presentMask & (1u << i)) ii.highVetoBar = std::max(ii.highVetoBar, cellBar_[i]);
    // Demand control is suspended while the pre-dive sequencer is scripting
    // test pulses: the test must own the valve, and a loop sitting in air at
    // 0.21 must not be "corrected" up to the setpoint under it.
    ii.autoMode = autoMode_ && !predive_.running();
    ii.depthStable = depthStable;
    // Injection needs a believable ppO2: with no vote at all, guessing is how
    // loops die, so the valve stays shut (the SENSOR_FAIL alarm tells the diver).
    ii.sensorHealthy = haveVote_;
    injOut_ = injector_.update(ii);
    lastSolOn_ = injOut_.solenoid || world_.solenoidStuckOpen;

    // ---- 8. alarms: the single authority -----------------------------------
    alarm::Inputs ai;
    ai.votedBar = vote_.votedPpo2;
    ai.highVetoBar = ii.highVetoBar;
    ai.cnsPct = static_cast<float>(deco_.cnsPct());
    ai.vbat = opts_.vbat;
    ai.sensorStale = false;  // stale = I2C fault; the sim bus never fails
    ai.spClamped = cpOut_.clampActive;
    ai.stuckOpen = injOut_.stuckOpenLatched;
    ai.vote = vote_;
    ai.haveVote = haveVote_;
    alarmRes_ = alarmMgr_.update(ai);

    // ---- 9. decompression + CNS (voted ppO2: the diver's exposure) ---------
    if (diveActive_) deco_.step(kTickS, pAmb, vote_.votedPpo2);

    // ---- 10. telemetry for scenario asserts --------------------------------
    // Only while the vote is real: a cold start (no samples yet) or a failed
    // sensor set legitimately reads 0 and must not pollute the dive envelope.
    if (haveVote_) {
        if (vote_.votedPpo2 > maxVotedSeen) maxVotedSeen = vote_.votedPpo2;
        if (vote_.votedPpo2 < minVotedSeen) minVotedSeen = vote_.votedPpo2;
    }
    const float truePpo2 = world_.ppo2True();
    if (truePpo2 > maxTrueSeen) maxTrueSeen = truePpo2;
    if (truePpo2 < minTrueSeen) minTrueSeen = truePpo2;
}

void Rig::startDive() {
    diveActive_ = true;
    cpEng_.diveStarted();
    deco_.reset();
}

void Rig::endDive() {
    diveActive_ = false;
    cpEng_.diveEnded();
}

}  // namespace sim
