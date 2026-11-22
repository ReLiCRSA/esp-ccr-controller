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

#include <cstdint>

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
    float depthM = 0.0f;                  // raw depth, metres
    bool pressureValid = true;            // pressure bus healthy
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

    /// One control cycle. `nowMs` is a monotonic clock (millis() on the ESP32,
    /// the sim's virtual clock). dt is taken from kTickMs: a cycle that arrives
    /// late must not be papered over.
    State cycle(const SensorData& in, uint32_t nowMs);

    // ---- dive lifecycle ----------------------------------------------------
    void startDive(uint32_t nowMs);
    void endDive(uint32_t nowMs);
    bool diveActive() const { return diveActive_; }

    // ---- diver commands (handset / web / buttons) --------------------------
    /// AUTO exactly as the handset must ask for it: refused unless the pre-dive
    /// checks passed (PLAN 7 hard lockout). Returns the mode in force.
    bool requestAuto(bool on);
    bool autoMode() const { return autoMode_; }
    /// Raw switch that bypasses the pre-dive lockout. Simulator and factory
    /// test paths only: no diver-facing command path may call this.
    void setAutoMode(bool on) { autoMode_ = on; }
    void ackAlarms() { alarmMgr_.ack(); }
    void cellproofOverride() { cpEng_.override(); }
    void clearStuckOpen() { injector_.clearStuckOpen(); }
    void prediveStart(uint32_t nowMs) { predive_.start(nowMs); }
    bool prediveRunning() const { return predive_.running(); }
    bool predivePassed() const { return predive_.allPassed(); }
    bool prediveFailed() const { return predive_.failed(); }
    predive::Status prediveStatus(predive::Step s) const { return predive_.status(s); }

    // ---- telemetry readers -------------------------------------------------
    const voting::Result& vote() const { return vote_; }
    const voting::CellStatus& cell(uint8_t ch) const { return voting_.cell(ch); }
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

    bool diveActive_ = false;
    bool autoMode_ = false;
    bool solenoid_ = false;
    bool haveVote_ = false;
    float depthSmooth_ = 0.0f, depthPrev_ = 0.0f, depthRate_ = 0.0f;
    float cellBar_[3] = {0.21f, 0.21f, 0.21f};
    float spTarget_ = 0.0f;
    uint32_t lastCycleMs_ = 0;
    bool cycleSeen_ = false;

    // scripted pulse-and-check cell test, executed at the pre-dive solenoid step
    bool ptActive_ = false, ptDone_ = false;
    uint32_t ptPulseBase_ = 0;

    voting::Result vote_{};
    alarms::Result alarmRes_{};
    cellproof::Output cpOut_{};
    inj::Output injOut_{};
};

}  // namespace ctrl
