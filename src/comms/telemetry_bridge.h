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

#include <cstdint>

#include "control/calibration.h"
#include "control/loop.h"
#include "log/divelog.h"

namespace bridge {

struct Telemetry {
    volatile float votedBar = 0.0f;
    volatile float cellBar[3] = {0.0f, 0.0f, 0.0f};  // calibrated, pre-vote
    volatile float setpointBar = 0.0f;
    volatile float depthM = 0.0f;
    volatile float vbat = 0.0f;
    volatile uint16_t alarmBits = 0;
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
    CALAIR_UNSETTLED = 5,  // failed: never stayed inside the spread band
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

    bool push(const Event& e) {  // producer: ControlTask
        const uint16_t n = (head_ + 1) & (kSize - 1);
        if (n == tail_) return false;      // full: newest event dropped, counted
        slot_[head_] = e;
        head_ = n;
        return true;
    }
    bool pop(Event& e) {                    // consumer: handset task
        if (tail_ == head_) return false;
        e = slot_[tail_];
        tail_ = (tail_ + 1) & (kSize - 1);
        return true;
    }
    uint32_t dropped() const { return dropped_; }
    void noteDropped() { dropped_++; }

private:
    volatile uint16_t head_ = 0, tail_ = 0;
    Event slot_[kSize];
    volatile uint32_t dropped_ = 0;
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
// per ControlTask cycle; cheap when nothing changed.
void publishEvents(const ctrl::State& st, uint32_t tSec, EventRing& ring,
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

} // namespace bridge
