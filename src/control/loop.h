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
    /// When pressureValid is false the chain does five separate things (each one
    /// measured by test_loop / test_divesoak): the depth filter holds the last
    /// sample, the setpoint band freezes with it, pre-dive's "am I on the
    /// surface?" answers FAIL, BIT_SENSOR_STALE goes up, and the tissue model
    /// keeps integrating the HELD depth (loop.cpp step 8). What does NOT stop is
    /// the CNS clock: ppO2 comes from the cells, on another bus, and it advances
    /// whenever there is a trustworthy vote — which is also the only case where
    /// it does stop (no vote, or a vote built from stale samples: nothing to
    /// integrate).
    bool pressureValid = true;            // pressure bus healthy: depthM believed
    bool cellsFresh = true;               // ADC readings are not stale
    // Last on purpose (the same rule as predive::Inputs::vbatMid): 12.0 is not a
    // plausible 1S pack, and an input left at its default must fail closed.
    float vbat = VBAT_MIN_V + 0.5f;
    /// Series packs only: midpoint volts, divider-scaled like `vbat`. 0 = not
    /// sensed, which with VBAT_CELLS >= 2 is a refusal, not a free pass.
    float vbatMid = 0.0f;
};

/// Chain outputs: the valve command plus everything telemetry reads.
struct State {
    bool solenoid = false;         // drive the O2 valve
    bool autoMode = false;
    bool diveActive = false;
    /// THE ppO2 this chain shows and records: the handset's `T,` field 2, the web
    /// gauge and the dive file's `voted` column. While the vote is FAILED this is
    /// the LAST ppO2 a healthy vote produced — not 0.00 — exactly like
    /// depthSmoothedM below. The name says it: it is the SHOWN number, and a
    /// shown number is not a measurement.
    ///
    /// It used to be `votedBar`, copied straight from `vote_` every cycle, so a
    /// failed vote erased the last believable value and the handset answered a
    /// hard `0.00` — a false catastrophic hypoxia, shown at the same moment
    /// BIT_SENSOR_FAIL was telling the diver the readout could not be trusted
    /// (finding N2). Consumers that must not believe a held value read
    /// `vote().votedPpo2` instead (loop.cpp steps 3, 4, 6, 7, 8), and that
    /// distinction is why this field is not called `voted`.
    /// The marker for "held" is not a new byte: `T,` FLAGS bit 3 carries
    /// BIT_SENSOR_FAIL (vote failed with cells seen) and BIT_SENSOR_STALE, and
    /// the dive row's alarm word carries the same bits un-aggregated.
    float shownPpo2 = 0.0f;
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

    /// The ppO2 substituted for a missing vote when step 8 steps the tissues.
    /// A constant, so the display hold (loop.cpp step 2b) cannot reach the deco
    /// model by accident: the row showing a held ppO2 must not silently change
    /// what the tissues were loaded with. 0.0 bar = ppN2 = pAmb = the maximum
    /// inert loading the depth allows, i.e. deliberately conservative. loop.cpp
    /// step 8 carries the decision and why the held value is the wrong input.
    static constexpr float kNoVotePpo2Bar = 0.0f;

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

    // ---- dive continuity (review H1) ---------------------------------------
    // What a dive starts with. A reboot between dives used to mean the tissues
    // were reset, so dive 2 was planned as if dive 1 had never happened — the
    // one decompression gap that needs no instrument to fail.
    //
    // The seed is PUBLISHED by whoever holds the newest record (the boot read,
    // and every continuity write) and CONSUMED by exactly one dive start, in
    // cycle() step 0 — the control thread, which is where the dive-state work
    // already lands (test_deferred). That ordering is the whole point: the
    // command task that answers DIVE START cannot touch deco_, and the chain
    // refuses to be mutated mid-decision from the other core.
    //
    // The interval arithmetic lives in logfmt::surfaceIntervalS (host-tested),
    // so what the chain applies is a number somebody else decided and some-
    // body-else's test can grade. `creditS` is the only physiology applied here,
    // and it is the engine's own surface integration (deco::Zhl16Ccr::
    // surfaceInterval).
    struct DiveSeed {
        bool valid = false;
        deco::Zhl16Ccr::TissueState st{};
        // Record identity, for the console line and the verdict record. Plain
        // integers because loop.h must not include src/log: the dependency
        // direction is log -> control at most (rowfill.h), never both ways.
        // `creditWhy` and `lossWhy` carry logfmt::IntervalWhy and
        // logfmt::ContLoss values; the values are stated at the consumer.
        uint32_t recordSeq = 0;
        uint32_t recordEpoch = 0;
        uint32_t creditS = 0;
        uint8_t kind = 0;        // logfmt::ContKind
        uint8_t creditWhy = 0;   // logfmt::IntervalWhy
        bool applied = false;    // this seed is what the last dive start used
    };

    /// WHY a dive start did not seed, when it did not, is deliberately NOT a
    /// field here. It is composed by the wiring (main.cpp), the only place that
    /// knows the store's state: "no record", "a record that does not decode" and
    /// "the store will not read" are facts about the STORE, and the chain has no
    /// business claiming them. logfmt::ContLoss is that vocabulary.

    /// Any thread. A copy under the request mutex — the same µs-scale critical
    /// section postConfig() uses, never held while anything decides.
    void postDiveSeed(const DiveSeed& s);
    /// What the LAST dive start did: seeded from a record, or not, and why.
    /// Published by the control thread when it applies the start; the console
    /// line and the surface verdict record read this.
    DiveSeed lastSeed() const;

    /// How many dive-start bodies have run this boot. ControlTask uses THIS as
    /// the dive-start edge rather than `diveActive`: with a deferred start the
    /// flag goes true on the command task and the body runs on a later tick, so
    /// the flag would announce a seed one cycle before the tissues existed. A
    /// verdict line printed before the thing it describes happened is the class
    /// of lie this repo already deleted once (`OK DIVE START` meaning
    /// "requested").
    uint32_t diveStarts() const { return diveStarts_; }

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
    /// Detector evidence for scenarios/telemetry: WHY a cell is out
    /// (voting::Engine::BarReason). Read-only; the exclusion vocabulary itself
    /// stays inside the voting layer.
    uint8_t barReason(uint8_t ch) const { return voting_.barReason(ch); }
    /// Diagnostics: which cells are currently barred from the vote on CellProof
    /// evidence (the handset and web pages show why the count dropped).
    uint8_t proofExclusion() const { return voting_.proofExclusion(); }
    const alarms::Result& alarms() const { return alarmRes_; }
    /// The LIVE alarm word — what the diver is being screamed at right now, ACK
    /// silences included. This is what `T,` FLAGS, the LED and the router answer
    /// with, and it must keep honouring an ACK.
    uint16_t alarmBits() const { return alarmRes_.bits; }
    /// The LOG view (alarms::Result::logBits): what was true, ACK silences
    /// ignored. This is what goes into the dive file's alarmBits column — see the
    /// header of alarms.h for why the two views exist and what an ACK does and
    /// does not erase.
    uint16_t alarmLogBits() const { return alarmRes_.logBits; }
    const cellproof::Output& cellproof() const { return cpOut_; }
    const inj::Output& injector() const { return injOut_; }
    bool outputFuseTripped() const { return injector_.outputFuseTripped(); }
    /// What the handset and the dive file show: the held value (see State's
    /// shownPpo2). Never use this where a MEASUREMENT is required — use
    /// vote().votedPpo2 together with haveVote().
    float shownPpo2() const { return shownPpo2_; }
    float setpointBar() const { return spTarget_; }
    float depthSmoothedM() const { return depthSmooth_; }
    /// Is there a believable ppO2 this cycle? false ⇒ shownPpo2() is the last
    /// believable value, not a measurement, and the raw vote is 0 by contract
    /// (voting::Result). Every consumer that must not act on a held value asks
    /// this question first.
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
    /// The dive-start body, shared by startDive() and cycle() step 0 so there is
    /// ONE answer to "what happens when a dive starts": proof state, then the
    /// tissues — seeded from the newest continuity record and fast-forwarded
    /// across the surface interval when there is one, reset when there is not,
    /// with the reason remembered for the console and the verdict record.
    void beginDiveBody();

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
    /// The published/recorded ppO2, held across a failed vote (loop.cpp step 2b).
    /// It starts at 0.0f, and that is a decision, not a leftover: before the first
    /// believable vote there is no history to hold, so 0.00 means "never measured"
    /// and the flag that says so is already up — the ADC chain has delivered nothing
    /// fresh yet (main.cpp feeds cellsFresh = !g_ads.anyStale(), and a channel with no
    /// sample is stale), so BIT_SENSOR_STALE — FLAGS bit 3 — is set from the first
    /// cycle. The one state where a no-vote 0.00 reaches a handset with bit 3 quiet is
    /// a chain that has NEVER had a plausible cell reading and is now getting fresh
    /// samples: cellsSeen is what tells "booting" from "it went blind" (alarms.cpp,
    /// finding C2) and it is still false, so SENSOR_FAIL stays down. Deliberate — a
    /// controller that has never measured its loop is not reporting a fault that
    /// started — and narrow: an instrument in that state has also failed the pre-dive
    /// air step, so AUTO is unreachable and the number shown is on a locked machine.
    /// Seeding air's 0.21 instead would be a phantom reading, and a harmful one: 0.21
    /// is above the LOW threshold (0.18), so a boot-time 0.21 would silence the
    /// low-ppO2 alarm on a number nobody measured. Same rule as depth: hold when there
    /// is something to hold, 0 plus a flag when there never was.
    float shownPpo2_ = 0.0f;
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

    // The continuity seed waiting for the next dive start (reqMux_-guarded, like
    // pending_), and what the last dive start actually did (written by the
    // control thread in beginDiveBody(), read by ControlTask right after).
    DiveSeed seed_;
    bool seedSet_ = false;
    DiveSeed lastSeed_{};
    uint32_t diveStarts_ = 0;   // dive-start bodies applied, this boot

    voting::Result vote_{};
    alarms::Result alarmRes_{};
    cellproof::Output cpOut_{};
    inj::Output injOut_{};
};

}  // namespace ctrl
