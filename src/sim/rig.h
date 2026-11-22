// ============================================================================
// rig.h — the full controller wired exactly as ControlTask will wire it:
//   world -> cells -> voting -> predive/CellProof -> setpoints -> injector
//                                   -> alarms -> deco
// plus the pre-dive sequencer running the scripted pulse-and-check cell test
// at its solenoid step.
//
// One tick = one 100 ms control cycle. Fault injection goes through the world
// (cell faults, stuck solenoid, flushes), never through the rig, so every
// scenario exercises the real detection paths.
// ============================================================================
#pragma once

#include <cstdint>

#include "control/alarms.h"
#include "control/calibration.h"
#include "control/cellproof.h"
#include "control/injector.h"
#include "control/predive.h"
#include "control/depthfilter.h"
#include "control/setpoints.h"
#include "control/voting.h"
#include "deco/zhl16c.h"
#include "sim/world.h"

namespace sim {

struct RigOpts {
    uint8_t presentMask = 0b111;  // bench builds with 2 cells: 0b011
    voting::Config voting;
    sp::Config sp;
    inj::Config inj;
    alarm::Config alarms;
    cellproof::Config cp;
    deco::Config deco;
    WorldConfig world;
    predive::Config predive;
    cal::PulseTest::Cfg pulseTest;
    float vbat = 12.4f;
};

class Rig {
public:
    static constexpr uint32_t kTickMs = 100;

    explicit Rig(const RigOpts& opts = {});

    /// One 100 ms control cycle: sensors -> voting -> checks -> SP -> valve ->
    /// alarms -> deco. See rig.cpp for why the steps run in this order.
    void tick();

    /// Runs `seconds` of control (whole ticks).
    void run(float seconds) {
        const uint32_t n = static_cast<uint32_t>(seconds * 1000.0f / kTickMs);
        for (uint32_t i = 0; i < n; i++) tick();
    }

    // ---- dive lifecycle (the handset/web drive these on the device) -------
    void startDive();
    void endDive();
    bool diveActive() const { return diveActive_; }
    void setAutoMode(bool on) { autoMode_ = on; }
    bool autoMode() const { return autoMode_; }
    /// Asks for AUTO the way the handset/web must: refused unless the pre-dive
    /// checks passed (PLAN 7 hard lockout). Returns the mode actually in force.
    bool requestAuto(bool on) {
        if (!on || !predive_.allPassed()) {
            autoMode_ = false;
            return false;
        }
        autoMode_ = true;
        return true;
    }

    // ---- pre-dive ----------------------------------------------------------
    void prediveStart() { predive_.start(tickMs_); }
    bool prediveRunning() const { return predive_.running(); }
    bool predivePassed() const { return predive_.allPassed(); }
    bool prediveFailed() const { return predive_.failed(); }
    predive::Status prediveStatus(predive::Step s) const { return predive_.status(s); }
    const predive::Sequencer& prediveSeq() const { return predive_; }
    cal::TestState pulseTestState() const { return pulseTest_.state(); }
    uint32_t pulseTestPulses() const { return injector_.pulsesTotal() - ptPulseBase_; }

    // ---- diver controls ----------------------------------------------------
    void ackAlarms() { alarmMgr_.ack(); }
    void cpOverride() { cpEng_.override(); }
    void clearStuckOpen() { injector_.clearStuckOpen(); }

    // ---- observation surface for the scenarios/tests -----------------------
    uint32_t t() const { return tickMs_; }
    World& world() { return world_; }
    const World& world() const { return world_; }
    const voting::Result& vote() const { return vote_; }
    const voting::CellStatus& cellStatus(uint8_t ch) const { return voting_.cell(ch); }
    const alarm::Result& alarmRes() const { return alarmRes_; }
    uint16_t alarmBits() const { return alarmRes_.bits; }
    bool alarmOn(uint8_t bit) const { return alarmRes_.bits & (1u << bit); }
    cellproof::State cpState() const { return cpOut_.state; }
    uint8_t cpUnprovenMask() const { return cpOut_.unprovenHighMask; }
    bool cpClampActive() const { return cpOut_.clampActive; }
    const inj::Output& injOut() const { return injOut_; }
    uint32_t pulseCount() const { return injector_.pulsesTotal(); }
    float solDutyPct() const { return injector_.dutyPct(tickMs_); }
    float spTarget() const { return spTarget_; }
    const deco::Zhl16Ccr& deco() const { return deco_; }
    float cnsPct() const { return static_cast<float>(deco_.cnsPct()); }
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
    void drivePulseTest();

    RigOpts opts_;
    cellproof::Config cpCfg_;
    World world_;
    voting::Engine voting_;
    sp::Engine spEng_;
    inj::Injector injector_;
    alarm::Manager alarmMgr_;
    cellproof::Engine cpEng_;
    predive::Sequencer predive_;
    deco::Zhl16Ccr deco_;
    cal::PulseTest pulseTest_;

    bool diveActive_ = false;
    bool autoMode_ = true;
    uint32_t tickMs_ = 0;

    float cellBar_[3] = {0.21f, 0.21f, 0.21f};
    depth::Filter<> depthF_;
    float depthSmooth_ = 0.0f, depthPrev_ = 0.0f, depthRate_ = 0.0f;
    bool lastSolOn_ = false;
    bool haveVote_ = false;

    // scripted pulse-and-check cell test (predive solenoid step)
    bool ptActive_ = false, ptDone_ = false;
    uint32_t ptPulseBase_ = 0;

    voting::Result vote_{};
    alarm::Result alarmRes_{};
    cellproof::Output cpOut_{};
    inj::Output injOut_{};
    float spTarget_ = 0.0f;
};

}  // namespace sim
