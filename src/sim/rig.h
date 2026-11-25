// ============================================================================
// rig.h — the PC system simulator: a physics world bolted onto the real
// control chain (ctrl::Loop), which is the same code the firmware ControlTask
// runs. The rig adds only (a) the virtual clock, (b) the valve fault path and
// (c) the running ppO2 envelope the scenarios assert on.
//
// Fault injection goes through the world (cell faults, stuck solenoid, flushes,
// the shared depth bus), never through the control chain: every detection has to
// earn its alarm the way it would in the water.
// ============================================================================
#pragma once

#include <cstdint>

#include "control/loop.h"
#include "sim/world.h"

namespace sim {

/// Simulator options: the control chain's config plus the world's physics.
/// Derives from ctrl::Config so scenario code reads o.inj.* / o.predive.*
/// exactly as it reads the firmware's own settings.
struct RigOpts : ctrl::Config {
    WorldConfig world;
    float vbat = 12.4f;  // the sim's battery is a fixed rail unless scripted
};

class Rig {
public:
    static constexpr uint32_t kTickMs = ctrl::Loop::kTickMs;

    explicit Rig(const RigOpts& opts = {});

    /// One 100 ms control cycle: world steps, then the control chain reads it.
    void tick();

    /// Runs `seconds` of control (whole ticks).
    void run(float seconds) {
        const uint32_t n = static_cast<uint32_t>(seconds * 1000.0f / kTickMs);
        for (uint32_t i = 0; i < n; i++) tick();
    }

    // ---- dive lifecycle (the handset/web drive these on the device) -------
    void startDive() { loop_.startDive(tickMs_); }
    void endDive() { loop_.endDive(tickMs_); }
    bool diveActive() const { return loop_.diveActive(); }
    void setAutoMode(bool on) { loop_.setAutoMode(on); }
    bool autoMode() const { return loop_.autoMode(); }
    bool requestAuto(bool on) { return loop_.requestAuto(on); }

    // ---- pre-dive ----------------------------------------------------------
    void prediveStart() { loop_.prediveStart(tickMs_); }
    bool prediveRunning() const { return loop_.prediveRunning(); }
    bool predivePassed() const { return loop_.predivePassed(); }
    bool prediveFailed() const { return loop_.prediveFailed(); }
    predive::Status prediveStatus(predive::Step s) const { return loop_.prediveStatus(s); }
    const predive::Sequencer& prediveSeq() const { return loop_.prediveSeq(); }
    cal::TestState pulseTestState() const { return loop_.pulseTestState(); }

    // ---- diver controls ----------------------------------------------------
    void ackAlarms() { loop_.ackAlarms(); }
    void cpOverride() { loop_.cellproofOverride(); }
    void clearStuckOpen() { loop_.clearStuckOpen(); }

    // ---- observation surface for the scenarios/tests -----------------------
    uint32_t t() const { return tickMs_; }
    World& world() { return world_; }
    const World& world() const { return world_; }
    const ctrl::Loop& loop() const { return loop_; }
    const voting::Result& vote() const { return loop_.vote(); }
    const voting::CellStatus& cellStatus(uint8_t ch) const { return loop_.cell(ch); }
    const alarms::Result& alarmRes() const { return loop_.alarms(); }
    uint16_t alarmBits() const { return loop_.alarmBits(); }
    bool alarmOn(uint8_t bit) const { return loop_.alarmBits() & (1u << bit); }
    cellproof::State cpState() const { return loop_.cellproof().state; }
    uint8_t cpUnprovenMask() const { return loop_.cellproof().unprovenHighMask; }
    uint8_t proofExclusion() const { return loop_.proofExclusion(); }
    bool cpClampActive() const { return loop_.cellproof().clampActive; }
    const inj::Output& injOut() const { return loop_.injector(); }
    uint32_t pulseCount() const { return loop_.pulseCount(); }
    float solDutyPct() const { return loop_.solDutyPct(tickMs_); }
    float spTarget() const { return loop_.setpointBar(); }
    const deco::Zhl16Ccr& deco() const { return loop_.deco(); }
    float cnsPct() const { return loop_.cnsPct(); }
    /// Commanded OR physically flowing (a stuck valve flows without a command).
    bool solenoidOn() const { return lastSolOn_; }

    // Running extremes, for scenario asserts. markEnvelope() restarts the
    // window: pre-mixing a loop that starts in air is not a hypoxia event.
    float maxVotedSeen = 0.0f;
    float minVotedSeen = 99.0f;
    float maxTrueSeen = 0.0f;
    float minTrueSeen = 99.0f;
    void markEnvelope() {
        maxVotedSeen = 0.0f;
        minVotedSeen = 99.0f;
        maxTrueSeen = 0.0f;
        minTrueSeen = 99.0f;
    }

private:
    World world_;
    ctrl::Loop loop_;
    float vbat_ = 12.4f;
    uint32_t tickMs_ = 0;
    bool lastSolOn_ = false;
};

}  // namespace sim
