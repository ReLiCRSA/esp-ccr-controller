// ============================================================================
// world.h — physics side of the system simulator (PC only). Loop gas model,
// depth drive, and O2 cell behaviour with injectable faults. Deterministic
// (fixed LCG noise) so scenario asserts are reproducible.
//
// Gas model: O2 content in litres-atmosphere (C = ppO2 x V). The diver burns
// SAC L/min at ambient (= SAC x Pamb litres-atm/min); the solenoid adds
// injLpm L/min of pure O2 while open. Good enough to exercise control,
// voting and alarms under realistic ppO2 dynamics — not a physiology model.
// ============================================================================
#pragma once

#include <cstdint>

namespace sim {

struct WorldConfig {
    float loopVolumeL = 8.0f;   // functional volume
    float sacLpm = 1.2f;        // O2 consumption, litres at ambient
    float injLpm = 150.0f;      // pure O2 flow with solenoid open
    float atmBar = 1.013f;
    float barPerM = 0.10053f;   // sea water 1.025 kg/l
};

// Cell faults a scenario can inject:
//   DEAD      depolarizes toward zero in ~0.5 s (wire/cell break)
//   SATURATE  output clips: linear in air, lies high up (the CellProof case)
//   SLOW      big tau (a sluggish cell the voting layer should outvote)
//   STUCK_AT  pins to a fixed value: the classic poisoned cell — reads
//             perfect air, never moves when the loop changes. Only the
//             pulse test and CellProof can see this one.
enum class CellFault : uint8_t { NONE, DEAD, SATURATE, SLOW, STUCK_AT };

struct CellModel {
    float gainErr = 1.0f;
    float offsetErr = 0.0f;
    float tauS = 0.6f;              // electrochemical lag
    CellFault fault = CellFault::NONE;
    float satCapBar = 0.80f;        // for SATURATE
    float stuckValue = 0.21f;       // for STUCK_AT
    float state = 0.21f;            // filtered internal
};

class World {
public:
    explicit World(const WorldConfig& cfg);

    // Scenario drives depth in metres (profile functions own it).
    void setDepthM(float m) { depthM_ = m; }
    float depthM() const { return depthM_; }
    float pAmbBar() const { return cfg_.atmBar + depthM_ * cfg_.barPerM; }

    // One physics step; solenoidOn = what actually flows (callers pass
    // command OR fault-injected stuck-open state via solenoidStuckOpen).
    void step(float dtS, bool commandedSolenoidOn);

    float ppo2True() const { return ppo2_; }

    // Fault-injection surface (public on purpose: scenarios set these).
    CellModel cells[3];
    bool solenoidStuckOpen = false;
    // Instantaneous loop O2 change, in bar of ppO2 (signed). A diluent flush
    // is negative; a hand top-up during pre-dive is positive.
    void changeO2Bar(float deltaBar);

    // What the ADC+cal chain would report for a cell this instant
    // (lag-filtered true value, error model, fault, bounded noise).
    float cellValue(uint8_t ch, float dtS);

private:
    uint32_t lcg() { lcg_ = lcg_ * 1103515245u + 12345u; return lcg_; }

    WorldConfig cfg_;
    float depthM_ = 0.0f;
    float o2ContentL_ = 0.21f * 8.0f;  // litres-atm
    float ppo2_ = 0.21f;
    uint32_t lcg_ = 0x5EEDC0FFu;
};

} // namespace sim
