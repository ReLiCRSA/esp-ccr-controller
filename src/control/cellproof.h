// ============================================================================
// cellproof.h — mid-dive high-ppO2 cell validation (PLAN.md: "CellProof").
//
// Galvanic cells that are linear at 0.4 but saturate at 1.0 pass an air cal
// and lie to you at depth. On first descent through CELLPROOF_DEPTH the
// controller sets the setpoint to CELLPROOF_SP and demands the voted ppO2
// actually reach CELLPROOF_MIN within CELLPROOF_TIMEOUT. What each cell is
// judged on is whether it TRACKED that demand change: on first descent the
// loop is driven up, deeper in a dive the same run is a step down, and a cell
// that does not move either way is not reading the loop.
//
//   PASS  -> cells proven; per-cell verdict — cells that never reached
//            MIN_PASS are marked unproven-at-high, and the control layer
//            removes them from the vote while it demands SP above OK_MAXSP
//   FAIL  -> clamp every setpoint above OK_MAXSP for the rest of the dive
//            + alarm; diver may OVERRIDE (loud, logged) — diver outranks
//            the machine, the machine never forgets.
//
// The verdict is PER CELL on purpose: an all-or-nothing proof lets a lying
// pair hide the one honest cell (the vote never reaches MIN_PASS, the proof
// times out, and the machine blames all three).
//
// The proof also re-arms mid-dive when the machine has evidence its cells have
// started lying — a REJECTED cell reading well off the vote in EITHER
// direction, held for SUSPECT_MS. A proof that only runs on first descent
// cannot see a cell that was honest at 10 m and failed at 40 m, and the 50-dive
// soak showed the consequence is a starved loop rather than a late toxicity
// alarm: a lying pair sitting ABOVE the setpoint means the valve never opens
// again, and sitting below it means the machine cannot see it happening.
// Only a PASS re-arms; after a FAIL the clamp stays until the diver overrides,
// because lifting a safety clamp on a second flaky result is the diver's call.
// ============================================================================
#pragma once

#include <cstdint>

namespace cellproof {

struct Config {
    bool enabled = true;
    float depthM = 10.0f;      // arm depth (first descent crossing)
    float spBar = 1.10f;       // temporary setpoint while testing
    float minPassBar = 1.00f;  // voted must reach this
    uint32_t timeoutMs = 60000;
    float okMaxSpBar = 0.90f;  // clamp ceiling applied on failure
    // Re-arm (optional fields append — see PLAN.md 16).
    float suspectGapBar = 0.10f;     // rejected cell this far off the vote...
    uint32_t suspectMs = 20000;      // ...held this long = proof is stale
    uint32_t rearmCooldownMs = 300000;
    uint8_t maxAttempts = 3;         // per dive: no endless thrash
    // A cell proves itself by TRACKING the loop, not by sitting at a number.
    // Over the run window each cell must move by at least this much (the
    // proof's setpoint change makes the loop move in one direction or the
    // other) — the sim's own cell noise is ±0.0025 bar, so 0.05 is ten times
    // the noise floor and a fifth of the drift bar. Without this, a cell frozen
    // at 1.20 bar "proves" itself by being above minPass in a loop at 1.20.
    float trackMinBar = 0.05f;
};

// INACTIVE, not DISABLED: the Arduino core's esp32-hal-gpio.h defines the
// object-like macro DISABLED, which expands inside an enumerator list even in
// a scoped enum. Host builds never see it; the ESP32 build breaks. Same reason
// the alarms namespace is plural (POSIX declares alarm()).
enum class State : uint8_t { INACTIVE, IDLE, ARMED, RUNNING, PASS, FAIL };

struct Input {
    bool diveActive = false;
    float depthM = 0.0f;
    float votedBar = 0.0f;
    float cellsBar[3] = {0, 0, 0};
    uint8_t activeMask = 0b111; // voting: which cells are in the vote
    uint32_t nowMs = 0;
    // Optional (appends): which cells are fitted at all. The per-cell verdict
    // must judge every fitted cell, including one the voting layer has already
    // dropped — that is the cell whose honesty settles a 2-vs-1 argument.
    uint8_t presentMask = 0b111;
};

enum Event : uint8_t { EV_NONE = 0, EV_STARTED, EV_PASS, EV_FAIL, EV_OVERRIDE };

struct Output {
    State state;
    bool requestSpOverride = false; // control layer: use cfg.spBar as SP
    bool clampActive = false;       // control layer: cap SP at cfg.okMaxSpBar
    uint8_t unprovenHighMask = 0;   // cells that never reached minPass
    Event event = EV_NONE;
    uint8_t attempts = 0;           // proof runs this dive (handset GET PROOF)
};

class Engine {
public:
    explicit Engine(const Config& cfg) : cfg_(cfg), state_(cfg.enabled ? State::IDLE : State::INACTIVE) {}
    void setConfig(const Config& cfg) { cfg_ = cfg; }

    void diveStarted() {
        state_ = cfg_.enabled ? State::IDLE : State::INACTIVE;
        unproven_ = 0;
        attempts_ = 0;
        suspectSinceMs_ = 0;
        lastRunMs_ = 0;
    }
    void diveEnded() { diveActive_ = false; }
    bool override() // diver lifts the clamp after a FAIL (logged by alarm mgr)
    {
        if (state_ == State::FAIL) { state_ = State::PASS; overridden_ = true; return true; }
        return false;
    }
    bool overridden() const { return overridden_; }
    State state() const { return state_; }
    uint8_t attempts() const { return attempts_; }

    Output update(const Input& in);

private:
    void beginRun(const Input& in, uint32_t nowMs);

    Config cfg_;
    State state_;
    bool diveActive_ = false;
    bool overridden_ = false;
    uint32_t startMs_ = 0;
    bool sawMax = false;
    float maxSeen_ = 0.0f;
    float best_[3] = {0.0f, 0.0f, 0.0f};
    float floor_[3] = {0.0f, 0.0f, 0.0f};  // lowest seen in this run: tracking, not level
    uint8_t unproven_ = 0;
    uint8_t attempts_ = 0;
    uint32_t suspectSinceMs_ = 0;  // 0 = no evidence pending
    uint32_t lastRunMs_ = 0;
};

} // namespace cellproof
