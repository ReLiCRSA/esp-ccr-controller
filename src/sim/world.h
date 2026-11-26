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

// Depth/pressure-bus fault — the other measurement leg the world owns. The one
// shape worth modelling is the one the hardware actually has: MS5837 and
// LPS28DFW sit on the SAME I2C port (include/config/pins.h), so a wet harness
// or a latched chip takes both depth channels out together, which is the
// "both failed" state sensors/depth_manager.h budgets for. There is no
// "one chip dead" fault on purpose: the arbiter picks its source at boot by
// identity and LOCKS it until the next power cycle, so a second chip never
// takes over underneath a diver.
//
// While the bus is dead the world does not stop being deep — only the
// measurement dies. depthReadingM() keeps the last sample the bus delivered
// and depthValid() says it is not to be believed, which is exactly what
// main.cpp feeds the control chain. The gap between those two is what the
// depth filter's hold-last-good rule exists to survive.
enum class DepthFault : uint8_t { NONE, BUS_DEAD };

struct CellModel {
    float gainErr = 1.0f;
    float offsetErr = 0.0f;
    float tauS = 0.6f;              // electrochemical lag
    CellFault fault = CellFault::NONE;
    float satCapBar = 0.80f;        // for SATURATE
    float stuckValue = 0.21f;       // for STUCK_AT
    // A frozen RAW channel: the reading stops changing at all, dither included.
    // This is physically what a shorted input or a wet connector that latched the
    // multiplexer does — and it is the ONLY thing the voting layer's frozen-reading
    // detector can see, because that detector's evidence is that a live front end
    // dithers by a few ADC counts forever (bench, 2026-11-26) and a frozen one does
    // not. STUCK_AT alone is NOT frozen: it pins the electrochemical target and
    // keeps the ADC wobble — that models a poisoned cell, not a dead channel, and
    // it is deliberately left alone because the 50-dive pool uses it that way.
    bool noDither = false;
    float state = 0.21f;            // filtered internal
};

class World {
public:
    explicit World(const WorldConfig& cfg);

    // Scenario drives depth in metres (profile functions own it). This is the
    // TRUE depth: the physics always follows it, whether or not the bus is
    // reporting it. A healthy bus's last reading is this value; a dead one
    // freezes at whatever it last delivered.
    void setDepthM(float m) {
        depthM_ = m;
        if (depthFault == DepthFault::NONE) depthReadingM_ = m;
    }
    float depthM() const { return depthM_; }
    float pAmbBar() const { return cfg_.atmBar + depthM_ * cfg_.barPerM; }
    /// What the depth driver would hand the control path right now: the last
    /// sample the bus delivered (never a fabricated 0 for "no measurement"),
    /// plus the flag that says whether it was measured this cycle. Mirrors
    /// g_depthM / g_depthValid in main.cpp.
    float depthReadingM() const { return depthReadingM_; }
    bool depthValid() const { return depthFault == DepthFault::NONE; }

    // One physics step; solenoidOn = what actually flows (callers pass
    // command OR fault-injected stuck-open state via solenoidStuckOpen).
    void step(float dtS, bool commandedSolenoidOn);

    float ppo2True() const { return ppo2_; }

    // Fault-injection surface (public on purpose: scenarios set these).
    CellModel cells[3];
    bool solenoidStuckOpen = false;
    // The depth/pressure bus. BUS_DEAD takes both channels out at once — the
    // shared Wire1 failure — and it is an on-window fault: a scenario sets it,
    // and clears it, so one dive can witness the failure AND the recovery. The
    // world's own depth never moves because of it: only the measurement does.
    DepthFault depthFault = DepthFault::NONE;
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
    // The last sample the depth bus delivered. Kept separate from depthM_ (the
    // truth the physics follows) because that separation IS the fault: a dead
    // bus holds its last reading, and the flag says not to believe it.
    float depthReadingM_ = 0.0f;
    float o2ContentL_ = 0.21f * 8.0f;  // litres-atm
    float ppo2_ = 0.21f;
    uint32_t lcg_ = 0x5EEDC0FFu;
};

} // namespace sim
