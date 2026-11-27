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

#include "config/defaults.h"   // CONTINUITY_SNAPSHOT_MS / CONTINUITY_SEED_REFRESH_MS
#include "control/loop.h"
#include "log/continuity.h"    // H1: the rig runs the REAL record store
#include "sim/world.h"

namespace sim {

/// Simulator options: the control chain's config plus the world's physics.
/// Derives from ctrl::Config so scenario code reads o.inj.* / o.predive.*
/// exactly as it reads the firmware's own settings.
struct RigOpts : ctrl::Config {
    WorldConfig world;
    float vbat = VBAT_CELLS * 3.90f;  // the sim's battery is a fixed rail unless scripted.
    // A charged 1S pack, not the 12.4 this project used to assume: a sim rail
    // outside the plausible band would now raise BATTERY in every scripted dive,
    // and a sim that always has a battery alarm cannot script anything else.
    // Series packs only: the midpoint, divider-scaled like `vbat`. 0 = not
    // sensed, which with VBAT_CELLS >= 2 refuses the pre-dive battery step —
    // the sim has to be able to script that case, not only the firmware.
    float vbatMid = (VBAT_CELLS * 3.90f) / 2.0f;

    // ---- dive continuity (review H1) ---------------------------------------
    // The rig runs the REAL record store (logfmt::ContinuitySlots: the same CRC
    // gate, slot choice and sequence rule the board uses) against a VIRTUAL wall
    // clock, because the acceptance scenario for dive continuity is a soak
    // scenario — dive 1 ends obligated, the power is cut, dive 2 starts an hour
    // later and must show a ceiling ON ENTRY. OFF (the default) is the pre-H1
    // behaviour in every particular: no snapshots, and a dive start resets the
    // tissues — which is why every existing soak witness stays byte-identical.
    bool continuity = false;
    /// The virtual wall clock at boot (unix epoch seconds). A dive start adds the
    /// sim's elapsed time to it, exactly as comms::clockEpoch() does on the board.
    /// 0 means the board has no clock at all — the case where a restore carries
    /// the tissues and credits no surface interval.
    uint32_t bootEpoch = 1800000000u;
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

    /// Mid-dive change to which galvanic channels the ADC chain reports at all —
    /// the shape of "this cell's channel went deaf": the driver stops delivering
    /// it, rather than delivering a number nobody believes. Rig-side only: the
    /// firmware's mask is a boot constant (main.cpp CELLS_PRESENT_MASK), so this
    /// knob exists to reach the one state a scenario has to be able to witness —
    /// every fitted, seen cell barred from the vote, i.e. Mode::FAILED with
    /// cellsSeen and BIT_SENSOR_FAIL up. Loop::setConfig re-tunes the rules in
    /// place and keeps dive activity, cell exclusions, the proof verdict and the
    /// tissues, so nothing else about the dive is restarted.
    void setCellsFitted(uint8_t mask);

    // ---- observation surface for the scenarios/tests -----------------------
    uint32_t t() const { return tickMs_; }
    World& world() { return world_; }
    const World& world() const { return world_; }
    const ctrl::Loop& loop() const { return loop_; }
    /// The State the chain PUBLISHED on the last cycle — what the firmware's own
    /// consumers read (main.cpp feeds this to the telemetry bridge, the HUD and
    /// the dive-file row fill). A scenario that reads loop() accessors instead is
    /// grading the chain's internals, not the numbers that reach a handset or a
    /// download, and the two can disagree exactly where a hold is involved.
    const ctrl::State& state() const { return lastState_; }
    const voting::Result& vote() const { return loop_.vote(); }
    const voting::CellStatus& cellStatus(uint8_t ch) const { return loop_.cell(ch); }
    /// Why a cell is currently excluded (voting::Engine::BarReason): the
    /// detector's evidence, visible to scenarios that grade WHY the machine
    /// stopped believing a channel, not just that it did.
    uint8_t cellBarReason(uint8_t ch) const { return loop_.barReason(ch); }
    const alarms::Result& alarmRes() const { return loop_.alarms(); }
    uint16_t alarmBits() const { return loop_.alarmBits(); }
    /// The dive-file view: the condition that was TRUE, ACK silences ignored.
    /// `alarmBits()`/`alarmOn()` stay the live word — they are what the handset
    /// and the soak's "was the diver told" grading read.
    uint16_t alarmLogBits() const { return loop_.alarmLogBits(); }
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

    // ---- dive continuity (review H1) ---------------------------------------
    // The rig's stand-in for the board's LittleFS store: the REAL two-slot store
    // object (logfmt::ContinuitySlots — same CRC gate, same slot choice, same
    // sequence rule), with the file calls left out, which is the same split the
    // dive-file soak uses. The cadence below is main.cpp's ControlTask cadence
    // (dive end, the snapshot grid, the 1 Hz seed refresh at the surface), so a
    // soak witness about dive 2 is a witness about the shipping behaviour and not
    // about a mirror of it.
    /// Virtual wall clock, or 0 = the board has lost it (the case where the
    /// restore carries tissues but credits no surface interval). The board's
    /// equivalent is comms::clockEpoch(): a base epoch plus the monotonic clock
    /// since it was set, which is exactly `bootEpoch + elapsed sim seconds`.
    uint32_t epoch() const {
        return clockLost_ ? 0u : opts_.bootEpoch + tickMs_ / 1000u;
    }
    void setClockLost(bool lost) { clockLost_ = lost; }
    /// The store, for witnesses that assert about the RECORD rather than about
    /// the chain's state (which slot, which seq, does it still decode).
    const logfmt::ContinuitySlots& contSlots() const { return contSlots_; }
    /// Whether any record decodes at all right now.
    bool contHasRecord() const { return contSlots_.newest() >= 0; }
    /// A record is present in bytes but nothing in it decodes: the torn-write
    /// case. Mirrors ContinuityStore::corrupt() (which caches the same fact).
    bool contCorrupt() const { return contBytesPresent_ && contSlots_.newest() < 0; }
    /// A record has been written at all this sim boot (the rig's cached stand-in
    /// for the store's cached `filePresent()` — no filesystem, so no stat).
    bool contBytesPresent() const { return contBytesPresent_; }
    /// Why the LAST dive start did not seed (logfmt::ContLoss), composed from the
    /// store's state exactly as main.cpp composes it. NONE means it did seed.
    logfmt::ContLoss contLastLoss() const { return contLoss_; }
    /// The seed the last dive start consumed (identity + credit), for witnesses
    /// that assert on the interval the machine CREDITED rather than on tissues.
    const ctrl::Loop::DiveSeed& contLastSeed() const { return contSeed_; }
    /// Bench hook: damages the newest record in place, standing in for a torn
    /// write or a flash page that has stopped holding its CRC.
    void contCorruptNewest();
    /// Bench hook: damages EVERY slot — the case where the picture itself is gone
    /// (a dead flash page, a partition wipe), as distinct from one torn write.
    void contCorruptAll();
    /// The store image, and the other half of the power-cycle simulation: a new rig
    /// handed the previous boot's image is a board that has just mounted a
    /// partition it did not write. This is the only way the soak can reach the
    /// reboot path, and it goes through the same ContinuitySlots::load the store's
    /// readFile() does.
    const uint8_t* contImage() const { return contSlots_.bytes(); }
    void contBootImage(const uint8_t* bytes, size_t len);
    /// The rig's stand-in for `TIME SET <epoch>` — and for the epoch a reboot
    /// carries, which is FROZEN at the last instant the clock was running (there
    /// is no RTC chip and no backup battery: wallclock.h measured this on the
    /// bench). Setting the base is how the soak says "a client set the clock at
    /// the surface" or "the board came back with the clock it had".
    void setEpochBase(uint32_t e) { opts_.bootEpoch = e; }
    uint32_t epochBase() const { return opts_.bootEpoch; }
    /// Bench hook: writes one record now (the soak uses this to place a snapshot
    /// at an exact instant, instead of waiting for the grid).
    bool contWriteNow(uint8_t kind);
    /// Republishes the seed the next dive will consume (the board does this at
    /// 1 Hz while it is not diving). Public so a scenario can force the decision
    /// at an exact instant.
    void contRefreshSeed();

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
    /// The continuity cadence, run from tick() after the chain has cycled:
    /// dive-start verdict, dive-END record, the snapshot grid during a dive, and
    /// the 1 Hz seed refresh at the surface. See rig.h's continuity block.
    void continuityStep(uint32_t now, bool diving);

    // The opts the rig was built with, kept because setCellsFitted() has to
    // re-apply the whole control config to change one mask bit — and re-applying
    // anything other than what the chain is running would silently retune it.
    RigOpts opts_;
    World world_;
    ctrl::Loop loop_;
    ctrl::State lastState_{};
    float vbat_ = VBAT_CELLS * 3.90f;
    float vbatMid_ = 0.0f;
    uint32_t tickMs_ = 0;
    bool lastSolOn_ = false;

    // ---- dive continuity (H1): see the rig.h block --------------------------
    logfmt::ContinuitySlots contSlots_;
    bool contBytesPresent_ = false;   // a record has been written this sim boot
    bool contSeedOffered_ = false;    // the seed now queued is a usable one
    logfmt::ContLoss contLoss_ = logfmt::CONT_LOSS_NONE;
    ctrl::Loop::DiveSeed contSeed_{};
    bool clockLost_ = false;
    bool contWasDiving_ = false;
    uint32_t contStarts_ = 0, contSnapMs_ = 0, contRefreshAt_ = 0;
};

}  // namespace sim
