// ============================================================================
// loop.h — the control chain itself, hardware-free.
//
//   sensors -> voting -> pre-dive (+ pulse cell test) -> CellProof ->
//   setpoints -> injector -> alarms -> deco
//
// This is deliberately ONE module holding the whole chain, because the two
// builds that must agree are the firmware ControlTask and the PC simulator:
// firmware feeds it from ADS1115/BMP085/MS5837/GPIO, the simulator feeds it
// from its gas model, and neither owns a private copy of the wiring. A bug
// found in the simulator is therefore a bug found in what actually ships.
//
// cycle() is called once per control tick (100 ms). No I/O, no allocation,
// no blocking: everything time-related arrives through nowMs/dt.
// ============================================================================
#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>

#include "control/alarms.h"
#include "control/calibration.h"
#include "control/cellproof.h"
#include "control/depthfilter.h"
#include "control/injector.h"
#include "control/predive.h"
#include "control/setpoints.h"
#include "control/voting.h"
#include "deco/zhl16c.h"

namespace ctrl {

struct Config {
    uint8_t presentMask = 0b111;  // bench builds with two cells: 0b011
    voting::Config voting;
    sp::Config sp;
    inj::Config inj;
    alarms::Config alarms;
    cellproof::Config cellproof;
    deco::Config deco;
    predive::Config predive;
    cal::PulseTest::Cfg pulseTest;
};

/// What the drivers measured this tick. The firmware fills this from the
/// ADC/pressure tasks; the simulator from its world. Nothing else may differ.
struct SensorData {
    float cells[3] = {0.0f, 0.0f, 0.0f};  // calibrated ppO2 per channel, bar
    /// Raw depth in metres. MEANINGLESS when pressureValid is false: the
    /// drivers then hold their last sample, and the chain's filter holds it
    /// right back, so whatever is in this field is never believed.
    ///
    /// A caller must NOT substitute 0.0f for "not measured". 0 m is not a
    /// placeholder, it is a reading — the surface — and it wins the median: the
    /// setpoint engine follows it to the 0.40 bar surface rung while the diver
    /// is at depth, and the handset reads 0.0 m. That substitution was in
    /// main.cpp until 2026-11-26 and it is finding C1. Pass the last depth the
    /// bus actually measured (or anything at all) and set pressureValid false;
    /// depthfilter.h explains why hold-last-good is the survivable direction.
    float depthM = 0.0f;
    bool pressureValid = true;            // pressure bus healthy: depthM believed
    bool cellsFresh = true;               // ADC readings are not stale
    float vbat = 12.0f;
};

/// Chain outputs: the valve command plus everything telemetry reads.
struct State {
    bool solenoid = false;         // drive the O2 valve
    bool autoMode = false;
    bool diveActive = false;
    float votedBar = 0.0f;
    float setpointBar = 0.0f;
    /// Filtered depth. While the pressure bus is invalid this is the LAST depth
    /// the bus measured, not the current one: consumers (handset display, dive
    /// log grid, max-depth total, deco ceiling) may print it, but must not
    /// present it as a fresh measurement — BIT_SENSOR_STALE in the same report
    /// is what says it is held.
    float depthSmoothedM = 0.0f;
    bool haveVote = false;
    bool injectionInhibited = false;
    bool stuckOpenLatched = false;
    bool outputFuse = false;       // solenoid held open longer than allowed

    voting::Result vote{};
    alarms::Result alarms{};
    cellproof::Output cellproof{};
    inj::Output injector{};
    bool alarmEdge = false;        // an alarm changed state this tick
    predive::Status prediveSolTest = predive::PENDING;
};

class Loop {
public:
    static constexpr uint32_t kTickMs = 100;

    explicit Loop(const Config& cfg);

    /// Re-apply configuration to the live chain. The engines re-tune in
    /// place: their state — dive activity, exclusions, latches, tissues —
    /// survives, because reloading settings must never look like a fresh
    /// start to the control chain. Control-thread only; the handset/web SET
    /// path uses postConfig() (see the request section below).
    void setConfig(const Config& cfg);

    /// One control cycle. `nowMs` is a monotonic clock (millis() on the ESP32,
    /// the sim's virtual clock). dt is taken from kTickMs: a cycle that arrives
    /// late must not be papered over.
    State cycle(const SensorData& in, uint32_t nowMs);

    // ---- dive lifecycle (control thread / simulator) ------------------------
    // Direct application: for whoever owns the cycle() call. The handset and
    // web paths must use the post* requests below instead.
    void startDive(uint32_t nowMs);
    void endDive(uint32_t nowMs);
    bool diveActive() const { return diveActive_; }

    // ---- diver commands, safe from any thread (the command-path surface) ----
    // The session lock keeps commands off each other; these keep commands off
    // the control chain. Two halves, deliberately:
    //   * the GATES (dive active, auto, cell test) flip immediately, because
    //     dive-locked refusals, telemetry mode and "one test at a time" ask
    //     for them the same millisecond the diver pressed the button;
    //   * the CHAIN WORK (engine re-tune, tissue/proof reset, sequencer start,
    //     ack latch) is queued and applied at the top of the
    //     next cycle(), so the chain is never mutated mid-decision on the
    //     other core. One control tick (100 ms) is the worst-case latency —
    //     the handset is the in-water transport, so that bound is a lifeline
    //     property, and `test_deferred` pins it.
    void postConfig(const Config& cfg);
    void postStartDive(uint32_t nowMs);
    void postEndDive(uint32_t nowMs);
    void postAckAlarms();
    void postPredive(uint32_t nowMs);
    void postCpOverride();
    // (postTestPulses retired 2026-11-26 with the `PULSE` verb: see
    // docs/HANDSET_PROTOCOL.md §8. Test pulses now have exactly one producer —
    // the pre-dive solenoid step and `TEST CELLS`, both relayed by
    // drivePulseTest — and no diver-facing command can queue a valve pulse.)

    // ---- diver commands ------------------------------------------------------
    // requestAuto/requestCellTest flip GATES and are safe from any thread;
    // the raw switch helpers mutate chain state and belong to the thread that
    // calls cycle() (firmware ControlTask, simulator rig, factory paths).
    /// AUTO exactly as the handset must ask for it: refused unless the pre-dive
    /// checks passed (PLAN 7 hard lockout). Returns the mode in force.
    bool requestAuto(bool on);
    bool autoMode() const { return autoMode_; }
    /// Raw switch that bypasses the pre-dive lockout. Simulator and factory
    /// test paths only: no diver-facing command path may call this.
    void setAutoMode(bool on) { autoMode_ = on; }
    void ackAlarms() { alarmMgr_.ack(); }
    bool cellproofOverride() { return cpEng_.override(); }
    /// Is the CellProof clamp up right now (post-FAIL, awaiting the diver)?
    /// A read, so the command path can answer `CELLPROOF OVERRIDE` honestly
    /// without mutating the chain off-thread.
    bool clampActive() const { return cpEng_.state() == cellproof::State::FAIL; }
    void clearStuckOpen() { injector_.clearStuckOpen(); }  // control thread
    void prediveStart(uint32_t nowMs) { predive_.start(nowMs); }  // control thread
    /// Bench / handset `TEST CELLS`: run the cell responsiveness test now,
    /// outside the pre-dive sequence. Same engine, same walls. The request is
    /// just a gate flip — the test itself always runs inside cycle().
    void requestCellTest() { cellTest_ = true; }
    bool cellTestRequested() const { return cellTest_; }
    float cellBar(uint8_t ch) const { return cellBar_[ch]; }  // calibrated, pre-vote
    /// Last pack voltage seen by the chain. Telemetry reads it here rather
    /// than reaching into SensorData, so no client invents its own number.
    float vbatVolts() const { return vbat_; }
    uint8_t cellTestPassMask() const { return pulseTest_.passMask(); }
    uint8_t cellTestFailMask() const { return pulseTest_.failMask(); }
    bool prediveRunning() const { return predive_.running(); }
    /// Published by cycle(), never read across threads from the sequencer
    /// itself: the AUTO grant is a decision, and decisions read published
    /// values, not another core's mid-update state.
    bool predivePassed() const { return predivePassed_; }
    bool prediveFailed() const { return predive_.failed(); }
    predive::Status prediveStatus(predive::Step s) const { return predive_.status(s); }

    // ---- telemetry readers -------------------------------------------------
    const voting::Result& vote() const { return vote_; }
    const voting::CellStatus& cell(uint8_t ch) const { return voting_.cell(ch); }
    /// Diagnostics: which cells are currently barred from the vote on CellProof
    /// evidence (the handset and web pages show why the count dropped).
    uint8_t proofExclusion() const { return voting_.proofExclusion(); }
    const alarms::Result& alarms() const { return alarmRes_; }
    uint16_t alarmBits() const { return alarmRes_.bits; }
    const cellproof::Output& cellproof() const { return cpOut_; }
    const inj::Output& injector() const { return injOut_; }
    bool outputFuseTripped() const { return injector_.outputFuseTripped(); }
    float votedBar() const { return vote_.votedPpo2; }
    float setpointBar() const { return spTarget_; }
    float depthSmoothedM() const { return depthSmooth_; }
    bool haveVote() const { return haveVote_; }
    bool solenoid() const { return solenoid_; }
    uint32_t pulseCount() const { return injector_.pulsesTotal(); }
    float solDutyPct(uint32_t nowMs) const { return injector_.dutyPct(nowMs); }
    const deco::Zhl16Ccr& deco() const { return deco_; }
    float cnsPct() const { return static_cast<float>(deco_.cnsPct()); }
    const predive::Sequencer& prediveSeq() const { return predive_; }
    cal::TestState pulseTestState() const { return pulseTest_.state(); }

private:
    void drivePulseTest(uint32_t nowMs);

    Config cfg_;
    cellproof::Config cpCfg_;
    voting::Engine voting_;
    sp::Engine spEng_;
    inj::Injector injector_;
    alarms::Manager alarmMgr_;
    cellproof::Engine cpEng_;
    predive::Sequencer predive_;
    deco::Zhl16Ccr deco_;
    cal::PulseTest pulseTest_;
    depth::Filter<> depthF_;

    // Gate flags: flipped synchronously by both the control thread and the
    // command path (post*/request*). Aligned single bytes, read constantly by
    // cycle() — atomics so "the handset task writes while core 1 reads" is a
    // defined operation rather than a hope.
    std::atomic<bool> diveActive_{false};
    std::atomic<bool> autoMode_{false};

    bool solenoid_ = false;
    bool haveVote_ = false;
    float depthSmooth_ = 0.0f, depthPrev_ = 0.0f, depthRate_ = 0.0f;
    float cellBar_[3] = {0.21f, 0.21f, 0.21f};
    float vbat_ = 0.0f;  // last sample seen: telemetry, never a control input
    float spTarget_ = 0.0f;
    uint32_t lastCycleMs_ = 0;
    bool cycleSeen_ = false;

    // scripted pulse-and-check cell test: runs at the pre-dive solenoid step,
    // or standalone on the handset/web `TEST CELLS` command
    bool ptActive_ = false, ptDone_ = false;
    bool wasPulsing_ = false;  // edge for withdrawing a stopped test's pulses
    std::atomic<bool> cellTest_{false};  // the gate; the body runs in cycle
    std::atomic<bool> predivePassed_{false};  // sequencer verdict, published by cycle
    uint32_t ptPulseBase_ = 0;

    // Deferred chain work from the command path (see post* requests). The
    // mutex is held only to copy this small struct in or out — never while
    // deciding anything — so cycle() can never block meaningfully on it.
    struct Pending {
        bool cfg = false, start = false, end = false, ack = false,
             predive = false, cpOverride = false;
        Config cfgVal;
        uint32_t prediveStamp = 0;
    };
    std::mutex reqMux_;
    Pending pending_;

    voting::Result vote_{};
    alarms::Result alarmRes_{};
    cellproof::Output cpOut_{};
    inj::Output injOut_{};
};

}  // namespace ctrl
