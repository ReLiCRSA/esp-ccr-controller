#include "world.h"

namespace {
// One ADS1115 count, in bar, at this board's gain and the factory cell slope:
// 0.21 bar / 1490 counts (defaults.h CELL_AIR_RAW_NOMINAL). Cell code below speaks
// in counts because that is the unit the bench noise is measured in.
constexpr float kBarPerAdcCount = 0.21f / 1490.0f;   // 1.4094e-4 bar/count
}  // namespace

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
    // Bench-matched cell dither, in the unit the bench measures it: ADS1115 counts.
    // On 2026-11-26 the console HB line (which prints g_ads.raw() directly) showed
    // three healthy cells AT REST IN AIR wobbling c0=1511/1514/1515/1517,
    // c1=1515..1519, c2=1534..1540 — 4..7 counts peak-to-peak, on every 5 s beat.
    // This board is PGA=16 (LSB 7.8125 uV) and the factory cell slope is
    // CAL_AIR_REF_BAR/CELL_AIR_RAW_NOMINAL = 0.21/1490 = 1.4094e-4 bar per count, so
    // that wobble is 0.0006..0.0010 bar peak-to-peak — NOT the ±0.0025 bar (±18
    // counts) this line used to add, which was five times the board. The amplitude
    // matters: a live channel dithering by a few counts, every sample, forever, is
    // the ONLY thing that distinguishes it from a channel whose reading froze (the
    // voting layer's stuck-cell detector), so a sim whose healthy cells are 5x
    // noisier than the board flatters that detector.
    // ±3 counts, quantized like the ADC is: p-p 6 counts, the middle of what the
    // bench showed. A frozen raw channel (CellModel::noDither) skips it entirely —
    // that absence of wobble is precisely the fault signature the voting layer's
    // frozen-reading detector is built to see.
    if (!cm.noDither)
        v += (static_cast<float>(lcg() % 7u) - 3.0f) * kBarPerAdcCount;
    return v < 0.0f ? 0.0f : v;
}

} // namespace sim
