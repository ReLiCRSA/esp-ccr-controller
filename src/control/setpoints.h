// ============================================================================
// setpoints.h — the 4-setpoint table with depth crossover, hysteresis and a
// clamp layer (cellproof clamp / future guard rails). Pure logic.
//
// Bands (defaults): SP1 surface 0.40 | SP2 in-water 0.70 | SP3 deep1 1.00
// at D1=30 m | SP4 deep2 1.30 at D2=50 m. A band is exited only when depth
// crosses the boundary by SP_HYST in the other direction — no flapping at
// the 2 m / D1 / D2 lines. Every returned change is expected to be logged.
// ============================================================================
#pragma once

#include <cstdint>

namespace sp {

struct Config {
    float sp[4] = {0.40f, 0.70f, 1.00f, 1.30f};
    float inWaterM = 2.0f;   // SP1 -> SP2 boundary
    float deep1M = 30.0f;    // SP2 -> SP3 boundary
    float deep2M = 50.0f;    // SP3 -> SP4 boundary
    float hysteresisM = 0.5f;
};

struct Result {
    uint8_t index;         // active band 0..3
    float value;           // SP after clamps
    float nominal;         // SP before clamps
    bool changed;          // band changed THIS update (log + snapshot it)
    bool clamped;          // clamp is biting
};

class Engine {
public:
    explicit Engine(const Config& cfg) : cfg_(cfg) {}

    void setConfig(const Config& cfg) { cfg_ = cfg; }
    const Config& config() const { return cfg_; }

    // maxSP <= 0 disables the clamp. Active while CellProof failed, etc.
    void setClamp(float maxSP) { clamp_ = maxSP; }
    float clamp() const { return clamp_; }

    // Call each control cycle with smoothed depth (metres).
    Result update(float depthM);

    uint8_t band() const { return band_; }

private:
    Config cfg_;
    float clamp_ = 0.0f;
    uint8_t band_ = 0;
};

} // namespace sp
