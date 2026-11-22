// ============================================================================
// ads1115_bus — hang-proof, non-blocking ADS1115 driver for 3 O2 cells.
//
// Design (PLAN.md 2.3) — why it cannot hang the firmware:
//  * One I2C transaction maximum per tick() call. tick() is called from the
//    SensorTask every 10 ms and returns in well under one Wire timeout.
//  * All Wire transactions use a hard timeout; failures NEVER throw us into
//    a retry loop. A fault closes the state machine and schedules a
//    bit-banged bus recovery (16 SCL clocks + STOP + Wire re-init).
//  * The config register is rewritten ONLY when switching MUX to the next
//    cell (unavoidable with one ADS1115 and 3 single-ended channels) and
//    never more than once per channel cycle. The first conversion after a
//    MUX switch is DISCARDED — the conversion register still holds a sample
//    taken while the mux was moving (TI app note: results right after a
//    MUX change are contaminated). This is the classic ADS1115 "hang/glitch"
//    trap: we design around it instead of against it.
//  * Per-channel freshness watchdog: a present cell with no valid sample for
//    ADS_STALE_MS raises the stale flag; ControlTask (M2) degrades/safes on it.
// ============================================================================
#pragma once

#include <Arduino.h>
#include "config/defaults.h"

class Ads1115Bus {
public:
    static constexpr uint8_t ADDR = 0x48;
    static constexpr uint8_t REG_CONVERSION = 0x00;
    static constexpr uint8_t REG_CONFIG = 0x01;
    static constexpr uint8_t CHANNELS = 3;   // AIN0, AIN1, AIN2

    enum class Phase : uint8_t {
        PH_SETUP,        // write CONFIG (MUX=channel, continuous, 16 SPS, PGA16)
        PH_READ_DISCARD, // read & discard the contaminated conversion
        PH_READ_VALID,   // read the good conversion for this channel
    };

    struct Stats {
        uint32_t samples;          // valid samples stored
        uint32_t i2cErrors;        // failed transactions
        uint32_t recoveries;       // bus recovery attempts
        uint32_t consecutiveFails; // transactions failed since last success
    };

    void begin();

    // Non-blocking. Call every ~10 ms from the SensorTask.
    void tick(uint32_t nowMs);

    // Latest valid raw sample for channel (0..2). Only meaningful when fresh().
    int16_t raw(uint8_t ch) const { return _raw[ch]; }
    // Latest sample converted to volts at the input pin.
    float volts(uint8_t ch) const { return _raw[ch] * ADS_LSB_UV * 1e-6f; }
    // Volts interpreted at 0.21 bar air target: mV against nominal 11 mV.
    bool fresh(uint8_t ch, uint32_t nowMs) const;
    bool anyStale(uint32_t nowMs) const;

    const Stats& stats() const { return _stats; }
    bool busHealthy() const { return _stats.consecutiveFails < 3; }

private:
    // Register access with hard timeout + error accounting. Return false on
    // ANY bus anomaly; never loops.
    bool writeReg(uint8_t reg, uint16_t value);
    bool readReg(uint8_t reg, uint16_t& value);

    bool recoverBus();

    static uint16_t configForChannel(uint8_t ch);
    uint8_t nextPresentChannel(uint8_t ch) const;

    Phase   _phase = Phase::PH_SETUP;
    uint8_t _ch = 0;
    uint32_t _nextActionMs = 0;
    uint32_t _recoveryBackoffMs = 250; // grows on persistent failure

    int16_t  _raw[CHANNELS] = {0, 0, 0};
    uint32_t _lastValidMs[CHANNELS] = {0, 0, 0};
    Stats    _stats{};
    uint8_t  _presentMask;
};
