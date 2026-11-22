// ============================================================================
// alarms.h — one authority for conditions, severities and edges.
//
// The alarm bitmask IS the telemetry FLAGS field and drives the ERR LED;
// raised/cleared edges feed E, lines and dive-log events, so the handset,
// web and dive file can never disagree about what happened when.
//
// Critical alarms LATCH: they persist until the condition recovers or the
// diver ACKs (ACK suppresses the audible/LED state for that id until the
// condition clears once and re-arms — diver outranks the machine, but the
// event log never forgets).
// ============================================================================
#pragma once

#include <cstdint>
#include "voting.h"

namespace alarm {

// Bit positions are protocol-visible (T, FLAGS) — never renumber.
enum Bit : uint8_t {
    BIT_LOW_PPO2 = 0, BIT_HIGH_PPO2 = 1, BIT_CELL_DRIFT = 2, BIT_SENSOR_FAIL = 3,
    BIT_BATTERY = 4, BIT_SP_CLAMP = 5, BIT_STUCK_OPEN = 6, BIT_SENSOR_STALE = 7,
    BIT_CNS_HIGH = 8, BIT_HYPOXIA = 9,
    BIT_COUNT = 10,
};

enum class Level : uint8_t { CRITICAL, WARNING };

struct Config {
    float lowBar = 0.18f;
    float highBar = 1.45f;
    float cnsAlarmPct = 80.0f;
    float vbatMin = 10.5f;
};

struct Inputs {
    float votedBar = 0.5f;
    float cnsPct = 0.0f;
    float vbat = 12.0f;
    bool sensorStale = false;
    bool spClamped = false;
    bool stuckOpen = false;
    voting::Result vote{};
    bool haveVote = false;      // false before first sample
    // Optional (last on purpose — see inj::Input): highest reading any fitted
    // cell reports, even one excluded from the vote. Two poisoned cells can
    // agree on 0.50 bar in a loop that is really at 1.6; the honest third must
    // still be able to raise the high alarm.
    float highVetoBar = 0.0f;
};

struct Edge { uint8_t id; bool raised; };

struct Result {
    uint16_t bits = 0;             // active alarms (latch-applied)
    bool ackSuppressed[BIT_COUNT]; // latch masks (for UI display)
    Edge edges[BIT_COUNT];
    uint8_t edgeCount = 0;
    bool anyCritical = false;
    bool anyWarning = false;
};

class Manager {
public:
    explicit Manager(const Config& cfg) : cfg_(cfg) {}
    void setConfig(const Config& cfg) { cfg_ = cfg; }

    Result update(const Inputs& in);
    void ack(); // suppresses currently-latched criticals until they clear

    uint16_t bits() const { return bits_; }

private:
    Config cfg_;
    bool active_[BIT_COUNT] = {};     // what was REPORTED last cycle (edges)
    bool rawActive_[BIT_COUNT] = {};  // condition||latch, pre-suppression
    bool suppressed_[BIT_COUNT] = {};
    bool latchAck_[BIT_COUNT] = {};   // event-type latch (kLatchAck bits)
    uint16_t bits_ = 0;
    void evaluate(uint8_t bit, bool condition, Level level, Result& res);

    static constexpr Level kLevel[BIT_COUNT] = {
        Level::CRITICAL, // LOW_PPO2
        Level::CRITICAL, // HIGH_PPO2
        Level::WARNING,  // CELL_DRIFT
        Level::CRITICAL, // SENSOR_FAIL
        Level::WARNING,  // BATTERY
        Level::WARNING,  // SP_CLAMP
        Level::CRITICAL, // STUCK_OPEN
        Level::CRITICAL, // SENSOR_STALE
        Level::WARNING,  // CNS_HIGH
        Level::CRITICAL, // HYPOXIA
    };
    // Event-type criticals arrive as one-shot conditions: they latch until
    // the diver ACKs (continuous alarms instead self-clear on recovery and
    // ACK only silences while the condition persists).
    static constexpr bool kLatchAck[BIT_COUNT] = {
        false, false, false, false, false, false, false, false, false, true,
    };
};

} // namespace alarm
