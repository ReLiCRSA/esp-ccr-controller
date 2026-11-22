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
    logfmt::DiveMeta toMeta() const;
};

} // namespace bridge
