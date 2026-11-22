// ============================================================================
// cellproof.h — mid-dive high-ppO2 cell validation (PLAN.md: "CellProof").
//
// Galvanic cells that are linear at 0.4 but saturate at 1.0 pass an air cal
// and lie to you at depth. On first descent through CELLPROOF_DEPTH the
// controller raises the setpoint to CELLPROOF_SP and demands the voted
// ppO2 actually reach CELLPROOF_MIN within CELLPROOF_TIMEOUT.
//
//   PASS  -> cells proven; per-cell lagging cells marked unproven-at-high
//            (the voting layer should exclude them whenever SP > 1.0).
//   FAIL  -> clamp every setpoint above OK_MAXSP for the rest of the dive
//            + alarm; diver may OVERRIDE (loud, logged) — diver outranks
//            the machine, the machine never forgets.
// One attempt per dive; resets at dive end.
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
};

enum class State : uint8_t { DISABLED, IDLE, ARMED, RUNNING, PASS, FAIL };

struct Input {
    bool diveActive = false;
    float depthM = 0.0f;
    float votedBar = 0.0f;
    float cellsBar[3] = {0, 0, 0};
    uint8_t activeMask = 0b111; // voting: which cells are in the vote
    uint32_t nowMs = 0;
};

enum Event : uint8_t { EV_NONE = 0, EV_STARTED, EV_PASS, EV_FAIL, EV_OVERRIDE };

struct Output {
    State state;
    bool requestSpOverride = false; // control layer: use cfg.spBar as SP
    bool clampActive = false;       // control layer: cap SP at cfg.okMaxSpBar
    uint8_t unprovenHighMask = 0;   // cells that never reached minPass
    Event event = EV_NONE;
};

class Engine {
public:
    explicit Engine(const Config& cfg) : cfg_(cfg), state_(cfg.enabled ? State::IDLE : State::DISABLED) {}
    void setConfig(const Config& cfg) { cfg_ = cfg; }

    void diveStarted() { state_ = cfg_.enabled ? State::IDLE : State::DISABLED; unproven_ = 0; }
    void diveEnded() { diveActive_ = false; }
    void override() // diver lifts the clamp after a FAIL (logged by alarm mgr)
    {
        if (state_ == State::FAIL) { state_ = State::PASS; overridden_ = true; }
    }
    bool overridden() const { return overridden_; }
    State state() const { return state_; }

    Output update(const Input& in);

private:
    Config cfg_;
    State state_;
    bool diveActive_ = false;
    bool overridden_ = false;
    uint32_t startMs_ = 0;
    bool sawMax = false;
    float maxSeen_ = 0.0f;
    float best_[3] = {0.0f, 0.0f, 0.0f};
    uint8_t unproven_ = 0;
};

} // namespace cellproof
