// ============================================================================
// alarms.h — one authority for conditions, severities and edges.
//
// Critical alarms LATCH: they persist until the condition recovers or the
// diver ACKs (ACK suppresses the audible/LED state for that id until the
// condition clears once and re-arms — diver outranks the machine, but the
// event log never forgets).
//
// TWO VOCABULARIES, and what separates them is the diver's silence, not the fact:
//
//   bits     — the alarm as the DIVER experiences it: raised, silenced by an ACK,
//              cleared. This is the `T,` FLAGS word, the ERR LED, the toasts and
//              the raised/cleared `E,` edges. An ACK may turn it off, and that is
//              correct: the diver outranks the machine.
//   logBits  — the alarm as a FACT about the instrument state at this instant:
//              raised, and STILL raised while an ACK is silencing it. This is what
//              the dive file's alarmBits column carries, because "the diver asked
//              for quiet" says nothing about the sensor that went away, and a
//              reader of a dive an hour later must not be able to lose that.
//
// Before logBits existed the dive file borrowed the live view, so an ACK in the
// middle of a blind depth bus erased the marker from the rows beside the very
// numbers that were computed without a depth measurement (finding #5, dive 9001).
// Raising the marker there would have meant raising it on the handset too, which
// is the one thing an ACK exists to forbid — hence two views, one word each.
//
// Event-latch ids (kLatchAck — today only HYPOXIA) are deliberately NOT carried on
// the log side after an ACK. ACK CONSUMES such a latch: a one-shot verdict the
// diver has heard is not a standing condition, so there is nothing left that is
// true, and the rising edge already wrote its own `E,` row. Re-marking every
// later dive-file row with a consumed event would turn one decision into
// hundreds of rows.
// ============================================================================
#pragma once

#include <cstdint>
#include "config/defaults.h"
#include "voting.h"

// Namespace is `alarms`, not `alarm`: newlib declares POSIX `alarm(int)` in a
// header the Arduino core pulls in, and a namespace of the same name collides
// with it — a build-breaker that only appears on the ESP32 target, not in host
// tests. Keep the plural.
namespace alarms {

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
    float highBar = 1.40f;  // below the injection wall: tell the diver first
    float cnsAlarmPct = 80.0f;
    // The pack floor, and the ceiling above which the READING is the fault rather
    // than the pack. Both come from the chemistry block in defaults.h, and both
    // raise the same BATTERY marker on purpose: what the diver can act on is
    // "do not trust this pack", and the console prints the volts that say which
    // of the two it is. A battery flag with no upper bound is a flag that a
    // disconnected sense divider cannot fail.
    float vbatMin = VBAT_MIN_V;
    float vbatMax = VBAT_IMPLAUSIBLE_V;
};

struct Inputs {
    /// The RAW vote, not ctrl::Loop's held display value: LOW/HIGH/CELL_DRIFT are
    /// statements about a measurement, and update() evaluates them only when
    /// haveVote is true. A held number reaching here must not raise them.
    float votedBar = 0.5f;
    float cnsPct = 0.0f;
    // Chemistry-derived on purpose, and deliberately OUT of band: 12.0 used to sit
    // here as a pack voltage, which is the exact stale 12 V-style number that
    // refused a charged 3.7 V cell for a month (docs/REVIEW.md 1 P1). A caller
    // that forgets to fill this field must fail closed, so the default is a
    // reading this chemistry cannot produce — an alarm nobody asked for is the
    // survivable direction, a healthy-looking default is not.
    float vbat = VBAT_IMPLAUSIBLE_V + 1.0f;
    bool sensorStale = false;
    bool spClamped = false;
    bool stuckOpen = false;
    voting::Result vote{};
    // false when the vote failed (Mode::FAILED) — which includes the
    // pre-first-sample window. It says "there is no ppO2 number this tick"; it
    // does NOT say why, and the two reasons are not interchangeable: see the
    // SENSOR_FAIL / HYPOXIA evaluation in update(), which lives outside the
    // haveVote gate precisely because a missing vote is their subject.
    bool haveVote = false;
    // Optional (last on purpose — see inj::Input): highest reading any fitted
    // cell reports, even one excluded from the vote. Two poisoned cells can
    // agree on 0.50 bar in a loop that is really at 1.6; the honest third must
    // still be able to raise the high alarm.
    float highVetoBar = 0.0f;
    // Optional (last on purpose — see inj::Input). "We cannot vouch for the rail
    // the valve runs on": computed by sensors/rail.h, which is where the patience
    // lives (a rail droops when the coil is energised, and this evaluator runs
    // every control cycle while the rail is swept once a second — counting cycles
    // here would be counting the wrong thing). One marker, BIT_BATTERY, for both
    // rails: what the diver can do about "the pack is flat", "the pack is
    // unreadable" and "the coil has no voltage" is the same action, and the
    // console says which of the three it is. See docs/HANDSET_PROTOCOL.md 4 bit 4.
    bool railUntrusted = false;
};

struct Edge { uint8_t id; bool raised; };

struct Result {
    uint16_t bits = 0;             // active alarms (latch-applied) — the LIVE view
    /// The LOG view: what `bits` says, plus every condition that is active while
    /// an ACK is silencing it. Read it where the question is "what was the machine
    /// like" (the dive file's alarmBits column); never where the question is "what
    /// is the diver being told" (T, FLAGS, the LED, toasts, E, edges) — those keep
    /// `bits`, because an ACK that changed nothing on the console would be a
    /// machine that cannot be silenced.
    uint16_t logBits = 0;
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
    //
    // SENSOR_FAIL is deliberately NOT an event latch: "the machine cannot see
    // its loop" is a continuous condition, and the diver is owed the moment it
    // stops being true — the bit goes away with the vote, and a re-fault
    // re-alarms from scratch.
    static constexpr bool kLatchAck[BIT_COUNT] = {
        false, false, false, false, false, false, false, false, false, true,
    };
};

}  // namespace alarms
