#include "world.h"

#include <algorithm>
#include <cmath>

namespace sim {

World::World(const WorldConfig& cfg) : cfg_(cfg) { o2ContentL_ = 0.21f * cfg_.loopVolumeL; }

void World::step(float dtS, bool commandedSolenoidOn) {
    const bool flowing = commandedSolenoidOn || solenoidStuckOpen;
    const float inj = flowing ? cfg_.injLpm : 0.0f;
    const float burn = cfg_.sacLpm * pAmbBar();

    o2ContentL_ += (inj - burn) * dtS / 60.0f;   // litres-atm
    if (o2ContentL_ < 0.0f) o2ContentL_ = 0.0f;
    if (o2ContentL_ > 40.0f * cfg_.loopVolumeL) o2ContentL_ = 40.0f * cfg_.loopVolumeL;
    ppo2_ = o2ContentL_ / cfg_.loopVolumeL;
}

void World::changeO2Bar(float deltaBar) {
    o2ContentL_ += deltaBar * cfg_.loopVolumeL;
    if (o2ContentL_ < 0.0f) o2ContentL_ = 0.0f;
    ppo2_ = o2ContentL_ / cfg_.loopVolumeL;
}

float World::cellValue(uint8_t ch, float dtS) {
    CellModel& cm = cells[ch];

    // Faults shape the physico-chemical target, then one first-order lag:
    // a dead cell depolarizes toward zero over ~0.5 s (fast, but not a
    // teleport — so voting sees a real collapse, not a glitch), stuck-low
    // decays on its own tau, saturated cells clip their output.
    float target = ppo2_;
    float tau = cm.tauS;
    switch (cm.fault) {
        case CellFault::NONE:      break;
        case CellFault::SLOW:      break;  // SLOW is just a big tau, set by scenario
        case CellFault::DEAD:      target = 0.002f; tau = 0.5f; break;
        case CellFault::STUCK_AT:  target = cm.stuckValue; break;
        case CellFault::SATURATE:  break;
    }
    const float k = tau > 0.001f ? std::min(1.0f, dtS / tau) : 1.0f;
    cm.state += (target - cm.state) * k;

    float v = cm.state * cm.gainErr + cm.offsetErr;
    if (cm.fault == CellFault::SATURATE) v = std::min(v, cm.satCapBar);
    v += static_cast<float>(lcg() % 501u) * 0.00001f - 0.0025f;  // ±2.5 mV-ish
    return v < 0.0f ? 0.0f : v;
}

} // namespace sim
