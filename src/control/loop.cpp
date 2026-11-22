// ============================================================================
// loop.cpp — see loop.h. The order of the steps is part of the design:
// sensors are read once, then every consumer sees the same sample set, and the
// valve decision is made from exactly the numbers the alarm manager reports.
// ============================================================================
#include "loop.h"

#include <algorithm>
#include <cmath>

namespace ctrl {

ctrl::Loop::Loop(const Config& cfg)
    : cfg_(cfg),
      cpCfg_(cfg.cellproof),
      voting_(cfg.voting),
      spEng_(cfg.sp),
      injector_(cfg.inj),
      alarmMgr_(cfg.alarms),
      cpEng_(cfg.cellproof),
      predive_(cfg.predive),
      deco_(cfg.deco),
      pulseTest_(cfg.pulseTest) {
    voting_.setPresentMask(cfg_.presentMask);
    deco_.reset();
}

// ---------------------------------------------------------------------------
// The scripted pulse-and-check test that executes the pre-dive SOLENOID step.
// The sequencer owns the schedule, the injector owns the valve, and this only
// relays: the test asks for a pulse, the injector is told, the pulses that
// actually left the valve are reported back, verdict to the sequencer.
// ---------------------------------------------------------------------------
void Loop::drivePulseTest(uint32_t nowMs) {
    const bool prediveStep = predive_.running() && predive_.current() == predive::S_SOLENOID;
    const bool wantTest = prediveStep || cellTest_;

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
    pi.presentMask = cfg_.presentMask;
    pi.pulsesDelivered = injector_.pulsesTotal() - ptPulseBase_;
    pi.nowMs = nowMs;
    const cal::PulseTestOutput po = pulseTest_.update(pi);

    if (po.requestPulse) injector_.requestTestPulse();

    if (po.done && !ptDone_) {
        ptDone_ = true;
        cellTest_ = false;  // a standalone run is over; the walls never changed
        // The step passes when every fitted cell reacted to oxygen the loop is
        // provably receiving. A solenoid that refuses to fire is a failure too:
        // zero pulses delivered with no rise says exactly that.
        const bool allOk = (po.passMask & cfg_.presentMask) == cfg_.presentMask;
        if (prediveStep) predive_.reportSolTest(allOk, pi.pulsesDelivered);
        // `TEST CELLS` from the handset: the caller reads the masks.
    }
    if (!wantTest) ptActive_ = false;  // sequencer moved on / aborted
}

State Loop::cycle(const SensorData& in, uint32_t nowMs) {
    State st;

    // Elapsed time is measured, not assumed: a task that arrives late must not
    // have its setpoint physics papered over with a nominal dt.
    uint32_t dt = kTickMs;
    if (cycleSeen_ && nowMs > lastCycleMs_) dt = nowMs - lastCycleMs_;
    lastCycleMs_ = nowMs;
    cycleSeen_ = true;

    // ---- 1. sensors (what the ADC + calibration chain reported) ------------
    for (int i = 0; i < 3; i++)
        cellBar_[i] = (cfg_.presentMask & (1u << i)) ? in.cells[i] : 0.0f;

    // Median then boxcar: bands and CellProof switch on absolute thresholds,
    // so one spiked pressure sample must not move the setpoint (and a bare EMA
    // would asymptote towards a threshold without ever crossing it).
    depthSmooth_ = depthF_.update(in.depthM);
    depthRate_ = std::fabs(depthSmooth_ - depthPrev_) / (dt / 1000.0f);
    depthPrev_ = depthSmooth_;
    const bool depthStable = depthRate_ < 0.02f;  // <1.2 m/min: stuck-open gate

    // ---- 2. voting ---------------------------------------------------------
    vote_ = voting_.update(cellBar_, dt);
    haveVote_ = vote_.mode != voting::Mode::FAILED;

    // ---- 3. pre-dive sequencer + its solenoid step -------------------------
    if (predive_.running()) {
        predive::Inputs pi;
        for (int i = 0; i < 3; i++) pi.cells[i] = cellBar_[i];
        pi.presentMask = cfg_.presentMask;
        pi.depthM = in.depthM;
        pi.vbat = in.vbat;
        pi.nowMs = nowMs;
        predive_.update(pi, nowMs);
    }
    drivePulseTest(nowMs);
    // A test pulse its step stopped waiting for must not linger in the
    // injector and inject minutes later for no reason.
    if (pulseTest_.state() != cal::TestState::PULSING) injector_.cancelTestPulse();

    // ---- 4. CellProof (mid-dive high-ppO2 cell validation) -----------------
    cellproof::Input ci;
    ci.diveActive = diveActive_;
    ci.depthM = depthSmooth_;
    ci.votedBar = vote_.votedPpo2;
    for (int i = 0; i < 3; i++) ci.cellsBar[i] = cellBar_[i];
    ci.activeMask = vote_.activeMask;
    ci.presentMask = cfg_.presentMask;
    ci.nowMs = nowMs;
    cpOut_ = cpEng_.update(ci);

    // ---- 5. setpoint selection (+ CellProof override/clamp) ----------------
    spEng_.setClamp(cpOut_.clampActive ? cpCfg_.okMaxSpBar : 0.0f);
    const sp::Result sp = spEng_.update(depthSmooth_);
    spTarget_ = sp.value;
    if (cpOut_.requestSpOverride) spTarget_ = cpCfg_.spBar;  // test demand
    else if (cpOut_.clampActive) spTarget_ = std::min(spTarget_, cpCfg_.okMaxSpBar);

    // ---- 5b. and the proof has teeth ---------------------------------------
    // A cell that could not prove at the high setpoint may not carry the vote
    // while the machine is demanding one. Without this the proof is only a note
    // for the display: a lying pair goes on outvoting the honest cell, and a
    // vote sitting ABOVE the setpoint means the valve never opens again — the
    // loop starves at depth while the readout insists it is rich (found by the
    // 50-dive soak, dive 4: vote 0.82, real loop 0.12 bar at 27 m). The ban
    // takes effect on the next cycle's vote, and lifts in the shallow bands,
    // where unproven-at-high cells are good enough and the redundancy is worth
    // getting back.
    const uint8_t unproven = cpOut_.unprovenHighMask;
    float cellLo = 99.0f, cellHi = -99.0f;
    for (int i = 0; i < 3; i++)
        if (cfg_.presentMask & (1u << i)) {
            cellLo = std::min(cellLo, cellBar_[i]);
            cellHi = std::max(cellHi, cellBar_[i]);
        }
    // Gating the ban on the setpoint band alone was wrong: a cell that sits
    // still while the loop moves is useless at ANY setpoint, and lifting the ban
    // on the way up through the shallow bands handed the vote back to the liars
    // (dive 4: 0.82 bar frozen against a loop that was really at 0.42). So the
    // ban holds while the machine is demanding a high setpoint, and also while
    // the fitted cells still disagree by more than the drift bar — which is the
    // same disagreement the voting layer itself refuses to average over.
    const bool cellsDisagree = (cellHi - cellLo) > cfg_.voting.driftBar;
    const uint8_t proofBan =
        (unproven && (spTarget_ > cpCfg_.okMaxSpBar || cellsDisagree)) ? unproven : 0u;
    voting_.setProofExclusion(proofBan);

    // ---- 6. injector -------------------------------------------------------
    inj::Input ii;
    ii.votedBar = vote_.votedPpo2;
    ii.setpointBar = spTarget_;
    // Demand control is suspended while the pre-dive sequencer is scripting
    // test pulses: the test must own the valve, and a loop sitting in air at
    // 0.21 must not be "corrected" up to the setpoint underneath it.
    ii.autoMode = autoMode_ && !predive_.running();
    ii.depthStable = depthStable;
    // Injection needs a believable ppO2: injecting on a guess is how loops
    // die, so with no vote at all — or a vote built from frozen, no-longer
    // fresh samples — the valve stays shut and the alarm tells the diver.
    // A cell excluded from the vote because it reads LOW keeps its veto on the
    // HIGH side: if it were the honest one, the loop is heading into oxygen
    // toxicity and its reading still has to stop injection.
    ii.sensorHealthy = haveVote_ && in.cellsFresh;
    ii.highVetoBar = ii.votedBar;
    for (int i = 0; i < 3; i++)
        if (cfg_.presentMask & (1u << i)) ii.highVetoBar = std::max(ii.highVetoBar, cellBar_[i]);
    ii.nowMs = nowMs;
    ii.dtMs = dt;
    injOut_ = injector_.update(ii);
    solenoid_ = injOut_.solenoid;

    // ---- 7. alarms: the single authority -----------------------------------
    alarms::Inputs ai;
    ai.votedBar = vote_.votedPpo2;
    ai.highVetoBar = ii.highVetoBar;
    ai.cnsPct = static_cast<float>(deco_.cnsPct());
    ai.vbat = in.vbat;
    ai.sensorStale = !in.cellsFresh || !in.pressureValid;
    ai.spClamped = cpOut_.clampActive;
    ai.stuckOpen = injOut_.stuckOpenLatched;
    ai.vote = vote_;
    ai.haveVote = haveVote_;
    alarmRes_ = alarmMgr_.update(ai);

    // ---- 8. decompression + CNS (the diver's real exposure) ----------------
    // The deco converts depth with its own surface pressure and water density,
    // so depth and absolute pressure stay consistent inside one module.
    if (diveActive_ && in.pressureValid)
        deco_.step(dt / 1000.0, deco_.depthToBar(std::max(0.0f, depthSmooth_)), vote_.votedPpo2);

    // ---- 9. fill the report ------------------------------------------------
    st.solenoid = solenoid_;
    st.autoMode = ii.autoMode;
    st.diveActive = diveActive_;
    st.votedBar = vote_.votedPpo2;
    st.setpointBar = spTarget_;
    st.depthSmoothedM = depthSmooth_;
    st.haveVote = haveVote_;
    st.injectionInhibited = injOut_.inhibitedHigh;
    st.stuckOpenLatched = injOut_.stuckOpenLatched;
    st.outputFuse = injOut_.outputFuse;
    st.vote = vote_;
    st.alarms = alarmRes_;
    st.cellproof = cpOut_;
    st.injector = injOut_;
    st.alarmEdge = alarmRes_.edgeCount > 0;
    st.prediveSolTest = predive_.status(predive::S_SOLENOID);
    return st;
}

void Loop::startDive(uint32_t /*nowMs*/) {
    diveActive_ = true;
    cpEng_.diveStarted();
    deco_.reset();
}

void Loop::endDive(uint32_t /*nowMs*/) {
    diveActive_ = false;
    cpEng_.diveEnded();
}

bool Loop::requestAuto(bool on) {
    if (!on || !predive_.allPassed()) {
        autoMode_ = false;
        return false;
    }
    autoMode_ = true;
    return true;
}

void Loop::setConfig(const Config& cfg) {
    cfg_ = cfg;
    // In place, not rebuilt: the chain's memory (dive edge, cell exclusions,
    // latches, proof verdict, tissues) is control state, not configuration.
    // A SET must retune the rules without pretending the dive restarted.
    voting_.setConfig(cfg.voting);
    voting_.setPresentMask(cfg.presentMask);
    spEng_.setConfig(cfg.sp);
    injector_.setConfig(cfg.inj);
    alarmMgr_.setConfig(cfg.alarms);
    cpEng_.setConfig(cfg.cellproof);
    cpCfg_ = cfg.cellproof;
    predive_.setConfig(cfg.predive);
    deco_.setConfig(cfg.deco);
    // pulseTest's Cfg is deliberately not hot-reloaded: changing the verdict
    // thresholds mid-test would make an in-flight verdict meaningless.
    // It is re-applied at the next test start by the firmware.
}

}  // namespace ctrl
