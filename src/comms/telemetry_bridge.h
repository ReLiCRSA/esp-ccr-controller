// ============================================================================
// telemetry_bridge.h — the narrow wall between the control core and the
// talker tasks (handset UART2, web). ControlTask publishes; comms reads.
// Scalars in a struct of volatiles: 32-bit load/store is atomic on this core
// and a status line may legitimately show last cycle's value. What must never
// happen is a talker dereferencing the Loop while the controller is mid-update.
//
// The event ring is SPSC (ControlTask produces, one consumer drains — the
// handset task; the web bridge mirrors from the same snapshot fields, not
// the ring, so two clients cannot fight over it).
// ============================================================================
#pragma once

#include <atomic>
#include <cstdint>

#include "control/calibration.h"
#include "control/loop.h"
#include "deco/view.h"
#include "log/divelog.h"

namespace bridge {

struct Telemetry {
    /// The ppO2 the handset and the dive file show: the chain's held value while
    /// the vote is failed (control/loop.h State::shownPpo2), never a 0 that the
    /// failure invented. haveVote below is what says whether it is a measurement.
    volatile float shownPpo2 = 0.0f;
    volatile float cellBar[3] = {0.0f, 0.0f, 0.0f};  // calibrated, pre-vote
    volatile float setpointBar = 0.0f;
    volatile float depthM = 0.0f;
    volatile float vbat = 0.0f;
    volatile uint16_t alarmBits = 0;
    /// The dive-file view (alarms::Result::logBits): the alarm conditions that
    /// were TRUE, including a CRITICAL the diver has ACKed into silence. Only the
    /// snapshot writers read this; every wire consumer reads alarmBits, because an
    /// ACK has to silence the wire.
    volatile uint16_t alarmLogBits = 0;
    volatile uint8_t voteMode = 0;
    volatile uint32_t pulses = 0;
    volatile uint16_t pulsesWindow = 0;   // pulses in the last T, interval
    volatile bool solenoid = false;
    volatile bool haveVote = false;
    volatile bool autoMode = false;
    volatile bool diveActive = false;
    volatile uint8_t cpState = 0;         // cellproof::State
    volatile uint8_t proofBan = 0;        // cells banned by proof right now
    volatile uint8_t unprovenMask = 0;
    volatile float solDutyPct = 0.0f;
    volatile uint32_t diveStartMs = 0;
    /// The decompression picture, published by ControlTask through a seqlock
    /// (deco/view.h): the event-driven dive row and every `D,` answer read
    /// this copy, never the engine. One writer; readers get one consistent
    /// generation or retry. This slot replaced the cross-task live walk that
    /// reviews N1/N8 named (rowfill.h carries the ownership vocabulary).
    deco::ViewSlot view;
};

struct Event {
    uint32_t tSec;
    uint16_t id;      // logfmt::EventId
    uint16_t value;
};

// ---- the last CAL AIR verdict ----------------------------------------------
// The air calibration runs in ControlTask (it owns the ADC samples), and until
// 2026-11-24 its outcome existed only as text on the USB console: `CAL AIR`
// answered `OK CAL AIR` from the command router whether the cells were then
// refused for warm-up, timed out, or committed — so a handset or a browser had
// no way to know which of those happened, and "calibrated" on any display would
// have been a guess. This is the fact, published where both transports can read
// it (`GET CALAIR`) and pushed as it happens (`E,<t>,7,<code>`).
//
// THE CODES ARE PROTOCOL-VISIBLE, in two places at once — the `CALAIR,` answer's
// status field and the `E,` id 7 event value — so they are never renumbered and
// new outcomes are appended on the right.
enum CalAirCode : uint8_t {
    CALAIR_NONE = 0,       // no CAL AIR since this power-up
    CALAIR_RUNNING = 1,    // sampling; `samples` says how far it has got
    CALAIR_OK = 2,         // air point committed for every fitted cell
    CALAIR_WARMUP = 3,     // refused: cells still polarising (warmupLeftS)
    CALAIR_DIVING = 4,     // aborted: a dive started (CAL AIR is a pre-dive act)
    CALAIR_UNSETTLED = 5,  // failed: no window stayed inside its stability band
    CALAIR_NOCELLS = 6,    // failed: no fitted cells to sample
};

struct CalAir {
    // Written by ControlTask, read by the command tasks. One 32-bit-at-a-time
    // struct of volatiles, payload first and `code` last: volatile writes are
    // not reordered with each other, so a reader that sees a terminal code also
    // sees the numbers that belong to that verdict. No lock — ControlTask must
    // never wait on the task that answers clients.
    volatile uint32_t atMs = 0;         // millis() of the verdict
    volatile uint8_t code = CALAIR_NONE;
    volatile uint8_t samples = 0;       // samples in the window (0..cfg.samples)
    volatile uint32_t warmupLeftS = 0;  // meaningful with CALAIR_WARMUP
    volatile float rawAvg[3] = {0.0f, 0.0f, 0.0f};  // raw ADC counts per cell
    volatile float spreadWorst = 0.0f;  // worst per-cell spread, raw counts
};

// Publishing those verdicts (payload plus the `E,` event) is declared below,
// after EventRing.

// ---- which chip owns depth, and whether anyone is using it ------------------
// Answered on the wire by `GET DS`. The facts live in press::DepthArbiter and its
// driver, both owned by SensorTask; the router must not reach into them, for two
// reasons. (1) It is deliberately hardware-free — host tests drive the whole
// command surface without a bus anywhere near it. (2) `identity()` hands out a
// pointer into the driver's own char array, which the driver rewrites with
// snprintf every time it re-identifies after a bus fault; a command task reading
// that array mid-write can see a prefix with no NUL in it, and printing it with
// %s reads past the end of the array. So SensorTask copies the facts here and the
// command tasks read the copy: same shape as CalAir above, one writer.
//
// `identity` is not volatile and is not safe to read while `locked` is false —
// the copy below always holds "none" in that case, so "no source" and "a source,
// named" are never confused. When locked, the array is always NUL-terminated and
// at most ID_MAX-1 characters, because it is filled by snprintf with that cap.
struct DepthSource {
    static constexpr uint8_t ID_MAX = 24;  // the drivers' own identity buffers

    // Written by SensorTask (it owns the arbiter), read by the command tasks.
    // Five fields copied without a lock cannot be atomic: whichever order they are
    // written in, a reader can catch the pair halfway across, and the two
    // transitions that ever happen here (unlocked -> locked at boot, and a faulted
    // source's fields being zeroed) want OPPOSITE orders. So consistency is
    // enforced where it can be — at the reply, which prints "none" unless `locked`
    // vouches for the name (see CommandRouter::buildDepthSourceLine) — and what
    // remains is a window of one sensor cycle, at most once per boot, never
    // mid-dive, because the arbiter's slot is never re-awarded.
    volatile bool locked = false;    // a source owns depth (locked at boot, and
                                     // it stays until the next power cycle)
    volatile bool used = false;      // the CONTROL PATH is taking depth from it
                                     // right now — false means the metres on the
                                     // wire are held, not measured
    volatile bool healthy = false;   // the driver says its last transaction was clean
    volatile uint32_t lastGoodMs = 0;  // millis() of that source's last good read
    char identity[ID_MAX] = "none";    // e.g. "LPS28DFW @0x5D" — the address is the
                                       // driver's, learned at its identity gate
};

/// SensorTask's one call per cycle into the snapshot above. Deliberately a
/// function and not five assignments at the call site: the rules it enforces
/// (below) are the ones a client's display depends on, and main.cpp — where the
/// call lives — is excluded from the native build, so assignments there would be
/// a rule with no test anywhere.
///
///   * `identity` is copied ONLY while `locked`. An unlocked snapshot always says
///     "none": "no source" and "a source, named" can never be confused, and a
///     stale name cannot outlive the slot it belonged to;
///   * the copy is bounded by the destination and always NUL-terminated — the
///     name is printed with %s, and the driver rewrites its own array with
///     snprintf whenever it re-identifies after a bus fault;
///   * `lastGoodMs` is zeroed unless the source is locked, so an answer's age can
///     never be computed from "never" (millis 0) and look like an old reading.
void publishDepthSource(DepthSource& out, bool locked, bool used, bool healthy,
                        uint32_t lastGoodMs, const char* identity);

class EventRing {
public:
    static constexpr uint16_t kSize = 32;  // power of two

    /// Stages a batch: everything pushed after this is invisible to pop() until
    /// commitTx(). ControlTask brackets its event publication with the pair,
    /// because the snapshot row an edge drags along must carry the deco picture
    /// the machine published FOR that edge (deco/view.h, N1's freshness rule).
    /// Staged, the ring used to be readable in the window between push() and the
    /// plan recompute — two statements in the wrong order, and the handset task
    /// filed "the machine decided X" beside the plan of the last 30 s grid.
    void beginTx() { tx_ = true; }

    /// Makes everything since beginTx() visible, ordered: the release store on
    /// head_ is what puts the event payloads before the head move, so a consumer
    /// that sees an event sees the whole Event and never a torn slot. (head_/
    /// tail_ became std::atomic with this rule: a volatile plain store carries no
    /// ordering, and "the payload is visible before the head is" is an ordering.)
    void commitTx() {
        tx_ = false;
        head_.store(write_, std::memory_order_release);
    }

    bool push(const Event& e) {  // producer: ControlTask
        const uint16_t n = (write_ + 1) & (kSize - 1);
        if (tail_.load(std::memory_order_acquire) == n) return false;  // full
        slot_[write_] = e;
        write_ = n;
        if (!tx_) head_.store(n, std::memory_order_release);  // no batch: at once
        return true;
    }
    bool pop(Event& e) {                    // consumer: handset task
        const uint16_t h = head_.load(std::memory_order_acquire);
        if (tail_ == h) return false;
        e = slot_[tail_];
        tail_.store((tail_ + 1) & (kSize - 1), std::memory_order_release);
        return true;
    }
    uint32_t dropped() const { return dropped_; }
    void noteDropped() { dropped_++; }

private:
    std::atomic<uint16_t> head_{0}, tail_{0};
    Event slot_[kSize];
    volatile uint32_t dropped_ = 0;
    uint16_t write_ = 0;   // producer-private write cursor: head_ is the only
                           // thing that tells the consumer what exists
    bool tx_ = false;
};

// Publication bookkeeping: what the chain looked like last cycle. Owned by
// ControlTask (one per chain), reset at dive start. No function-statics:
// the simulator runs many chains in one process and a hidden previous value
// from dive 3 leaking into dive 4 would fabricate an event out of nothing.
struct PublisherState {
    uint16_t prevAlarmBits = 0;
    bool prevAuto = false;
    int16_t prevCp = -1;
    float prevSp = -1.0f;
    uint8_t prevActiveMask = 0;
};

// Publish the chain's changes since last cycle as protocol events. One call
// per ControlTask cycle; cheap when nothing changed. Returns how many rows
// were pushed.
//
// The caller STAGES the batch it publishes into (EventRing::beginTx before,
// commitTx after) and republishes the deco picture in between, so an
// event-driven snapshot row reads a picture computed for the edge it records —
// not one from the last 30 s grid (N1's freshness), and not one the handset task
// caught half-way across the cycle that decided it.
int publishEvents(const ctrl::State& st, uint32_t tSec, EventRing& ring,
                  PublisherState& ps);

// Records an air-calibration step where the clients can see it, and puts the
// TERMINAL verdicts on the event ring as `E,<t>,7,<code>`. Running is not an
// event: a 12-sample calibration would otherwise bury the dive in twelve rows
// for one fact, and `GET CALAIR` already reports progress.
void publishCalAir(CalAir& out, const cal::AirCalOutput& a, uint32_t nowMs,
                   uint32_t tSec, EventRing& ring);

// A refusal that never reached the sampler (the warm-up gate): the same
// publication, so the diver's client sees the refusal instead of a silence
// after an `OK`.
void publishCalAirRefused(CalAir& out, uint8_t code, uint32_t warmupLeftS,
                          uint32_t nowMs, uint32_t tSec, EventRing& ring);

// The in-force dive totals. Filled by ControlTask while a dive runs, read
// and written out by the command task on DIVE END. Plain struct: ownership
// passes at the dive edge, both sides only touch it while it is theirs.
struct DiveTotals {
    uint32_t durationSec = 0;
    int32_t maxDepthCm = 0;
    int32_t minVotedX1000 = 32767;
    int32_t maxVotedX1000 = -32768;
    uint32_t totalPulses = 0;
    uint8_t cnsPeakPct = 0;
    // The plan the dive actually ran on. A dive file whose `M,` row says
    // gf 0/0 and sp 0.000/0.000/0.000/0.000 cannot be reviewed later: a
    // different GF pair or a different setpoint ladder explains a deco
    // schedule, and the columns have been in the format since v2 while nothing
    // ever filled them.
    uint8_t gfLow = 0, gfHigh = 0;
    int16_t sp1X1000 = 0, sp2X1000 = 0, sp3X1000 = 0, sp4X1000 = 0;
    logfmt::DiveMeta toMeta() const;
};

// ---- one instant, for the event-driven dive row ---------------------------
// The event-driven snapshot row used to read the bridge scalars and the deco
// generation at whatever moment it happened to run. A pump drains up to eight
// events, and ControlTask republishes every 100 ms, so three edges drained in
// one pump produced three rows whose depth, alarm word and deco picture came
// from three different control cycles — one row per edge, and none of them the
// instant of its own edge. The capture is what makes a batch describe ONE
// instant: copy here, then write every row of the batch from the copy.
//
// What this does NOT claim: the captured instant is not the edge's instant. It
// is never OLDER than the edge — the picture is published before the edges
// become readable (EventRing::beginTx, and main.cpp keeps that order) — and it
// is at most one pump interval newer. That bound replaces the one it had
// before, "the plan may be up to 30 s old", and it is the drain cadence rather
// than the plan grid. Naming the direction is the point: a row can read a
// slightly later machine, it can no longer read an earlier one.
struct EdgeSnap {
    uint32_t nowMs = 0;         // millis() of the capture: the row's own stamp
    // The dive's start on the CONTROL task's clock — the same base the `E,` line
    // beside this row was counted from (main.cpp publishes it at the dive edge
    // and publishEvents counts t_s from it). The command task used to count the
    // event row from its own file-open instant, which is a different instant by
    // up to one control cycle, so the `E,` line and the `S,` row written from the
    // same edge disagreed about when the edge was. 0 means the chain has not
    // announced a dive start yet; the row writer treats that as t_s = 0.
    uint32_t diveStartMs = 0;
    deco::View pic{};
    bool hasPic = false;        // false = no generation published yet (boot window)
    float depthM = 0.0f;
    float shownPpo2 = 0.0f;     // the held value included — see Telemetry
    float cellBar[3] = {0.0f, 0.0f, 0.0f};
    float setpointBar = 0.0f;
    float vbat = 0.0f;
    bool haveVote = false;
    bool diveActive = false;
    bool autoMode = false;
    uint16_t alarmLogBits = 0;  // the LOG view — see Telemetry::alarmLogBits
    uint8_t voteMode = 0;
    uint8_t cpState = 0;
    uint8_t cnsPeakPct = 0;     // the dive's running peak, from the totals
};

/// Copies the published scalars, the dive totals' CNS peak and ONE seqlock
/// generation into `out`. `totals` may be null (no dive open on a host test).
/// `out.hasPic` is false when nothing has been published yet, which is the only
/// case in which a row written from this has no deco columns at all — and
/// logfmt::fillSnapshot then says "no number" in `ndl` rather than 0.
void captureEdge(const Telemetry& tel, const DiveTotals* totals, uint32_t nowMs,
                 EdgeSnap& out);

} // namespace bridge
