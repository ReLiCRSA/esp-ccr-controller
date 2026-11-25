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

    // ---- 0. deferred command work, applied on this thread ------------------
    // The handset/web path flips gates synchronously but never touches the
    // engines: everything heavier arrives queued, and lands here at the tick
    // boundary — before sensors, so this tick already decides under the rules
    // the diver just set. Worst-case latency for a handset command is one
    // control tick (100 ms); the handset is the in-water transport, and a
    // diver pressing ACK or MANUAL underwater is owed that bound.
    {
        Pending p;
        {
            std::lock_guard<std::mutex> g(reqMux_);
            p = pending_;
            pending_ = Pending{};
        }
        if (p.cfg) setConfig(p.cfgVal);
        if (p.start) { cpEng_.diveStarted(); deco_.reset(); }
        if (p.predive) predive_.start(p.prediveStamp ? p.prediveStamp : nowMs);
        if (p.cpOverride) cpEng_.override();
        if (p.ack) alarmMgr_.ack();
        if (p.end) cpEng_.diveEnded();
    }

    // Gate snapshot: one per tick, so no step of this decision runs against
    // a mode that changed halfway through it.
    const bool diveActive = diveActive_.load();
    const bool autoMode = autoMode_.load();

    // Elapsed time is measured, not assumed: a task that arrives late must not
    // have its setpoint physics papered over with a nominal dt.
    uint32_t dt = kTickMs;
    if (cycleSeen_ && nowMs > lastCycleMs_) dt = nowMs - lastCycleMs_;
    lastCycleMs_ = nowMs;
    cycleSeen_ = true;

    // ---- 1. sensors (what the ADC + calibration chain reported) ------------
    for (int i = 0; i < 3; i++)
        cellBar_[i] = (cfg_.presentMask & (1u << i)) ? in.cells[i] : 0.0f;

    vbat_ = in.vbat;  // remembered for telemetry only

    // Median then boxcar: bands and CellProof switch on absolute thresholds,
    // so one spiked pressure sample must not move the setpoint (and a bare EMA
    // would asymptote towards a threshold without ever crossing it).
    //
    // And a sample the driver did not measure is not a sample: the bus health
    // flag goes to the filter, which skips it and keeps the last depth it
    // actually believed. Until 2026-11-26 main.cpp turned "no reading" into
    // 0.0 m here, the windows converged on it in half a second, and step 5
    // walked the setpoint to the 0.40 bar surface rung while the diver was at
    // depth — one failure mode, because both depth chips share Wire1
    // (include/config/pins.h) and depth_manager.h budgets for exactly that.
    // depthfilter.h carries the reasoning for hold-last-good.
    depthSmooth_ = depthF_.update(in.depthM, in.pressureValid);
    if (in.pressureValid) {
        depthRate_ = std::fabs(depthSmooth_ - depthPrev_) / (dt / 1000.0f);
        depthPrev_ = depthSmooth_;
    }
    // While the bus is invalid the rate is UNKNOWN, and the last measured rate
    // is held rather than replaced by 0. Zero is not "unknown": it is a claim
    // that the diver is stationary, and depthRate_'s only consumer is the
    // injector's stuck-open gate, which may only judge a ppO2 rise on windows
    // where depth is PROVABLY steady. A frozen filtered depth is precisely a
    // window where steadiness cannot be proved, so fabricating 0 would let the
    // gate latch a stuck-open valve on physics it never measured — and a false
    // STUCK_OPEN latch stops injection until the diver clears it, which is the
    // failure that starves a loop at depth. Holding the last real rate gets
    // this wrong in the harmless direction: a stuck-open suspicion is deferred,
    // never invented.
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
        // The step asks "is the rig on the surface", which is only answerable
        // with a measurement: a dead depth bus used to arrive here as the 0.0 m
        // main.cpp fabricated for it, and "no sensor" was rewarded with a PASS
        // (review H3 — closed as part of C1).
        pi.depthValid = in.pressureValid;
        pi.vbat = in.vbat;
        pi.nowMs = nowMs;
        predive_.update(pi, nowMs);
    }
    drivePulseTest(nowMs);
    // Publish the verdict for the command path (see predivePassed in loop.h):
    // everything this tick could report about the checks is in by now.
    predivePassed_ = predive_.allPassed();
    // A test pulse its step stopped waiting for must not linger in the
    // injector and inject minutes later for no reason. Edge-triggered on
    // purpose: the unconditional cancel also ate the handset's standalone
    // `PULSE` (queued between cycles, withdrawn before the valve ever saw it —
    // a command that silently did nothing). Only pulses belonging to a pulse
    // test that just STOPPED are withdrawn. (`PULSE` itself was retired
    // 2026-11-26, HANDSET_PROTOCOL.md §8 — but the edge-trigger stays: the
    // pre-dive step's queued pulse crosses the same boundary the old command
    // died on, and this is what keeps it alive between request and valve.)
    {
        const bool pulsing = pulseTest_.state() == cal::TestState::PULSING;
        if (wasPulsing_ && !pulsing) injector_.cancelTestPulse();
        wasPulsing_ = pulsing;
    }

    // ---- 4. CellProof (mid-dive high-ppO2 cell validation) -----------------
    cellproof::Input ci;
    ci.diveActive = diveActive;
    ci.depthM = depthSmooth_;
    ci.votedBar = vote_.votedPpo2;
    for (int i = 0; i < 3; i++) ci.cellsBar[i] = cellBar_[i];
    ci.activeMask = vote_.activeMask;
    ci.presentMask = cfg_.presentMask;
    ci.nowMs = nowMs;
    cpOut_ = cpEng_.update(ci);

    // ---- 5. setpoint selection (+ CellProof override/clamp) ----------------
    // Fed the smoothed depth, and that is the whole C1 inhibit — there is no
    // separate "depth invalid" latch here, deliberately. The band engine moves
    // only when the number it is given crosses a boundary, so while the bus is
    // invalid and depthSmooth_ holds, the band is frozen wherever the last real
    // sample left it and the setpoint cannot chase a surface the machine did
    // not measure. Verified end to end: sp::Engine::update() takes depth and
    // nothing else (setpoints.cpp), and its only input is depthSmooth_.
    // What the diver sees: the setpoint stays at the deep band while the depth
    // readout sits still, next to BIT_SENSOR_STALE saying the number is held.
    // Holding rich is the side with walls under it (HIGH alarm 1.40, injector
    // highWall 1.45 / cutoff 1.60, the duty cap); holding toward the surface
    // has none, which is why freezing is the direction this chain takes.
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
    ii.autoMode = autoMode && !predive_.running();
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
    // The deco engine integrates two things from two sensors, and they fail
    // independently, so the two halves are gated separately (deco::Zhl16Ccr
    // step/stepCns/stepTissues — verified: cnsLimitMin() has no ambient or
    // depth term anywhere, and neither half keeps state about time, so running
    // one half without the other cannot skip or double-count dt). Depth is
    // still converted inside the deco module, with its own surface pressure and
    // water density, so depth and absolute pressure stay on one ruler.
    //
    // CNS: ppO2 x time, from the OXYGEN CELLS — a different sensor on a
    // different bus from depth. So it advances whenever the chain has a ppO2
    // worth integrating, which is the same trust rule the injector is held to
    // in step 6: haveVote_ && in.cellsFresh. A dead depth bus must not stop the
    // oxygen clock — this used to freeze it, which under-records in the
    // direction that hurts: 300 blind seconds at a 1.30 bar setpoint is around
    // 2.8 % CNS the diver was never told about (the NOAA 1.30 row: 180 min), and
    // CNS is what the CNS alarm and the dive file's cnsPct column are built on.
    // Residual, stated rather than hidden: with NO vote there is nothing to
    // integrate, so CNS freezes there. That is the one remaining CNS
    // under-record window, and it is the window in which alarms.cpp already
    // raises BIT_SENSOR_FAIL (vote FAILED with cells seen) — the diver is being
    // told the machine cannot see the loop at all, which is also the moment
    // they stop trusting its deco picture.
    if (diveActive && haveVote_ && in.cellsFresh)
        deco_.stepCns(dt / 1000.0, vote_.votedPpo2);

    // Tissues: ambient pressure x time, and during a depth fault the pressure
    // available is the one depthSmooth_ last MEASURED (held, not believed —
    // finding C1). Stepping on the held depth is the choice that is wrong in
    // the safe direction, both ways it can be wrong:
    //   * Freezing used to be the behavior, and it under-records in exactly the
    //     fault response the manual asks for: a diver who loses depth and setpoint
    //     climbs on the instruments they still have. That off-gassing happened,
    //     and a frozen model forgets the debt (or the recovery from it) the
    //     diver just paid — the schedule and NDL shown after recovery would be
    //     computed from tissues that are not the diver's.
    //   * Holding last-good while the diver ASCENDS over-states the ambient
    //     pressure, so it over-counts uptake and the ceiling it demands is
    //     deeper than the truth needs. Over-recording debt is the fenced
    //     direction: a deeper ceiling can only ever ask for more deco, and the
    //     setpoint is frozen anyway (step 5), so the machine is not being walked
    //     anywhere on the held number.
    //   * The one case that under-records is a diver who descends DEEPER than
    //     the last known depth while blind and alarmed: uptake is counted at a
    //     shallower pressure than reality. Bounded — the window ends when the
    //     bus returns or the dive is aborted, BIT_SENSOR_STALE is on the display
    //     saying the depth shown is not measured, and injection is regulating to
    //     a frozen band rather than following the diver down. No phantom depth
    //     excursion is invented to paper over it: the model does not get to be
    //     cleverer than the sensor that went away.
    if (diveActive)
        deco_.stepTissues(dt / 1000.0, deco_.depthToBar(std::max(0.0f, depthSmooth_)),
                          vote_.votedPpo2);
    // (With no vote, votedPpo2 is 0.0f, so ppN2 is computed HIGHER and the
    // tissues load more than they would on a real ppO2 — unchanged from before
    // this split, and the conservative side of "no measurement".)

    // ---- 9. fill the report ------------------------------------------------
    st.solenoid = solenoid_;
    st.autoMode = ii.autoMode;
    st.diveActive = diveActive;
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

void Loop::startDive(uint32_t /*nowMs*/) {  // control thread: apply now
    diveActive_ = true;
    cpEng_.diveStarted();
    deco_.reset();
}

void Loop::endDive(uint32_t /*nowMs*/) {    // control thread: apply now
    diveActive_ = false;
    cpEng_.diveEnded();
}

// ---- the command-path surface (any thread) ---------------------------------
// Gates flip here, chain work waits for cycle() step 0. See loop.h — the split
// is what lets dive-locked refusals answer instantly while the engines are
// never retuned mid-decision on the other core.

void Loop::postConfig(const Config& cfg) {
    std::lock_guard<std::mutex> g(reqMux_);
    pending_.cfg = true;
    pending_.cfgVal = cfg;   // last writer wins: a newer SET supersedes
}

void Loop::postStartDive(uint32_t /*nowMs*/) {
    // Gate first: the dive is active the instant the diver says so. If it was
    // already active, do NOT queue a second start body — re-running it would
    // wipe tissues and proof state mid-dive. The router refuses this too; the
    // chain refuses it for itself.
    const bool wasActive = diveActive_.exchange(true);
    std::lock_guard<std::mutex> g(reqMux_);
    pending_.end = false;
    if (!wasActive) pending_.start = true;  // coalesced: three starts, one dive
}

void Loop::postEndDive(uint32_t /*nowMs*/) {
    diveActive_ = false;
    std::lock_guard<std::mutex> g(reqMux_);
    pending_.end = true;
}

void Loop::postAckAlarms() {
    std::lock_guard<std::mutex> g(reqMux_);
    pending_.ack = true;
}

void Loop::postPredive(uint32_t nowMs) {
    std::lock_guard<std::mutex> g(reqMux_);
    pending_.predive = true;
    pending_.prediveStamp = nowMs;
}

void Loop::postCpOverride() {
    std::lock_guard<std::mutex> g(reqMux_);
    pending_.cpOverride = true;
}

bool Loop::requestAuto(bool on) {
    if (!on || !predivePassed_) {  // published verdict, not raw sequencer state
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
