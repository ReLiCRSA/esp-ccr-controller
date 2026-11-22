// ============================================================================
// ms5837.h — MS5837-30BA gel-filled depth sensor driver logic (PLAN.md: wet
// prototype depth source). Pure state machine over the Transport interface:
// identical code is unit-tested on the host and runs under ControlTask on the
// ESP32. Never blocks, never loops: conversions are kicked and harvested on
// control ticks; bus faults fall back with exponential backoff (same
// hang-proof philosophy as the ADS1115 driver — a dead depth sensor must not
// take the dive down with it).
//
// Math per datasheet REV C (errata resolved: OFF = C2*2^16 + C4*dT/2^7,
// SENS = C1*2^15 + C3*dT/2^8, P = (D1*SENS/2^21 - OFF)/2^13 in 0.1 mbar)
// plus second-order temperature compensation, and the factory PROM CRC-4.
// Verified against the datasheet worked example in test_pressure.
// ============================================================================
#pragma once

#include <cstdint>

#include "pressure_bus.h"

namespace press {

class Ms5837 {
public:
    static constexpr uint8_t ADDR = 0x76;

    // Commands (datasheet I2C section)
    enum Cmd : uint8_t {
        CMD_ADC_READ = 0x00,
        CMD_RESET = 0x1E,
        CMD_CONV_D1 = 0x40,  // pressure; OSR bits added (0x08 = OSR4096)
        CMD_CONV_D2 = 0x50,  // temperature
        CMD_PROM = 0xA0,     // + 2*word
    };
    static constexpr uint8_t OSR4096 = 0x08;
    static constexpr uint32_t CONV_MS = 10;   // max conv time @OSR4096
    static constexpr uint32_t BOOT_MS = 15;   // reset -> PROM load

    enum class State : uint8_t { UNINIT, RESET_WAIT, PROM_READ, READY, BUS_FAULT };

    Ms5837(Transport& bus, uint8_t addr = ADDR) : bus_(bus), addr_(addr) {}

    // Call on every tick until true (or keep calling; it self-retries with
    // backoff). Loads factory coefficients and verifies the PROM CRC-4.
    bool begin(uint32_t nowMs);

    // Sample pipeline: one full cycle = D1 then D2. start(nowMs) kicks it;
    // poll(nowMs) advances; returns true exactly once per completed cycle.
    void start(uint32_t nowMs);
    bool poll(uint32_t nowMs);

    // Latest results
    float pressureBar() const { return pressureBar_; }
    float temperatureC() const { return tempC_; }
    uint32_t lastGoodMs() const { return lastGoodMs_; }
    bool healthy() const { return state_ == State::READY && readFails_ < K_MAX_FAILS; }
    State state() const { return state_; }
    uint16_t c1() const { return c_[1]; }

    // First-order + second-order math, exposed for unit tests.
    struct Raw { uint32_t d1, d2; };
    static void compensate(const uint16_t c[7], const Raw& d,
                           float& barOut, float& tempCOut);
    static bool checkPromCrc(const uint16_t prom[8]);

private:
    static constexpr uint8_t K_MAX_FAILS = 5;

    bool startConv(uint8_t cmd, uint32_t nowMs);
    bool harvest(uint32_t& out, uint32_t nowMs);
    void noteFail(uint32_t nowMs);

    Transport& bus_;
    const uint8_t addr_;
    State state_ = State::UNINIT;

    uint16_t c_[7] = {};      // c_[1..6] = C1..C6
    bool crcOk_ = false;

    uint32_t bootDeadlineMs_ = 0;
    uint32_t convDeadlineMs_ = 0;
    enum Phase : uint8_t { PH_IDLE, PH_D1, PH_D2 };
    Phase phase_ = PH_IDLE;
    uint32_t d1_ = 0, d2_ = 0;

    uint32_t backoffUntilMs_ = 0;
    uint8_t backoffShift_ = 0;  // 250ms << shift, capped ~4s
    uint8_t readFails_ = 0;

    float pressureBar_ = 0.0f;
    float tempC_ = 0.0f;
    uint32_t lastGoodMs_ = 0;
};

} // namespace press
