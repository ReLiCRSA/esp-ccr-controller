// ============================================================================
// depth_manager.h — boot-time arbitration between the depth sources.
//
// The question it answers: which chip owns depth on THIS board, today?
// Policy (PLAN.md 13, decided 2026-11-23):
//
//   * MS5837-30BA is PREFERRED — it is the dive-rated part. If a rig has
//     both fitted during transition testing, the dive sensor wins.
//   * LPS28DFW is the fallback — the bench and interim wet-test sensor.
//   * Identity gates belong to the drivers (WHO_AM_I 0xB4 / PROM CRC-4).
//     The arbiter only ever locks onto a driver whose begin() said
//     "identified" — it cannot invent a sensor.
//   * Locked at boot: once a source is chosen it stays until the next
//     power cycle, even if it faults mid-dive. Nothing swaps the depth
//     source while a diver is breathing from the machine's numbers.
//   * Nothing fitted: keep cycling (preferred window, fallback window,
//     rest) forever, so a hot-fitted sensor comes up within seconds —
//     and pre-dive stays red until it does.
//
// Pure state machine over the DepthSensor interface: unit-tested on the
// host against the fake bus, no Wire anywhere near it.
// ============================================================================
#pragma once

#include <cstdint>

#include "pressure_bus.h"

namespace press {

class DepthArbiter {
public:
    // Window each candidate gets to claim depth per attempt round. The
    // windows are generous relative to each driver's own boot time
    // (MS5837: reset+PROM < 100 ms; LPS28: probe+BOOT ~60 ms), so a fitted
    // sensor always wins its window; only an absent/forged one burns time.
    static constexpr uint32_t PREFERRED_WINDOW_MS = 2500;
    static constexpr uint32_t FALLBACK_WINDOW_MS = 1500;
    static constexpr uint32_t RECIRCULATE_MS = 2000;  // both failed: rest, re-arm

    DepthArbiter(DepthSensor& preferred, DepthSensor& fallback)
        : preferred_(preferred), fallback_(fallback) {}

    // Drive every tick. False = no depth source yet (or the locked one is
    // mid-recovery). True = a source is READY.
    bool begin(uint32_t nowMs);
    void start(uint32_t nowMs);
    bool poll(uint32_t nowMs);

    bool locked() const { return locked_; }
    // Valid only while locked; nullptr while still arbitrating.
    DepthSensor* source() const { return locked_ ? chosen_ : nullptr; }
    const char* sourceName() const { return chosen_ ? chosen_->identity() : "none"; }

    // Delegated access — only touch these when locked() is true.
    float pressureBar() const { return chosen_->pressureBar(); }
    float temperatureC() const { return chosen_->temperatureC(); }
    uint32_t lastGoodMs() const { return chosen_->lastGoodMs(); }
    bool healthy() const { return locked_ && chosen_->healthy(); }

private:
    enum class Phase : uint8_t { PREFERRED, FALLBACK, WAIT };

    DepthSensor& preferred_;
    DepthSensor& fallback_;
    DepthSensor* chosen_ = nullptr;
    bool locked_ = false;
    Phase phase_ = Phase::PREFERRED;
    uint32_t phaseStartMs_ = 0;
    uint32_t nextPhaseMs_ = 0;
};

}  // namespace press
