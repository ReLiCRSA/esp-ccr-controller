// ============================================================================
// lps28.h — ST LPS28DFW bench pressure driver (Adafruit breakout #6067),
// the bench stand-in since 2026-11-23. The BMP085 it replaces never ACKed
// once on the bench and is removed from the tree; the MS5837-30BA stays the
// wet-prototype sensor (PLAN.md §13).
//
// Why the DFW specifically: it is dual full-scale — 1260 hPa (mode 0) or
// 4060 hPa (mode 1). Mode 0 tops out around 2.4 m of sea water, which would
// clip the 10 m cell-proof pressurization; we always set FS_MODE=1
// (CTRL_REG2 bit 6), covering ~40 m of sea water at 1/2048 hPa LSB —
// ~5 mm resolution, ample for setpoint crossovers on the bench.
//
// Address: default 0x5C; the breakout's XSA pad can pull it to 0x5D. begin()
// probes both with a WHO_AM_I read and REQUIRES 0xB4 — the LPS28DFW's ID.
// A near-relative (LPS22HB=0xB1, LPS22HH=0xB3, old LPS28HH=0x64) is refused:
// a depth sensor whose identity is not checked is a depth sensor that lies.
//
// Same non-blocking Transport contract and retry/backoff shape as the other
// two pressure drivers, so the shared SensorTask rule holds: one I2C owner,
// no blocking past one Wire timeout.
// ============================================================================
#pragma once

#include <cstdint>

#include "pressure_bus.h"

namespace press {

class Lps28 : public DepthSensor {
public:
    static constexpr uint8_t ADDR = 0x5C;       // XSA open/LOW (Adafruit default)
    static constexpr uint8_t ADDR_ALT = 0x5D;   // XSA pulled high
    static constexpr uint8_t CHIP_ID = 0xB4;    // LPS28DFW WHO_AM_I

    enum Reg : uint8_t {
        REG_WHOAMI = 0x0F,
        REG_CTRL1 = 0x10,   // ODR[3:0] at 6:3, AVG[2:0] at 2:0
        REG_CTRL2 = 0x11,   // BOOT(7) FS_MODE(6) LP_ODR9(5) SWRESET(2) ONE_SHOT(0)
        REG_PRESS = 0x28,   // auto-increments XL,L,H then TEMP L,H
    };
    static constexpr uint8_t CTRL2_BOOT = 0x80;
    static constexpr uint8_t CTRL2_FS4060 = 0x40;  // 260..4060 hPa, 1/2048 LSB
    static constexpr uint8_t CTRL1_RUN = 0x20;     // ODR 25 Hz, AVG 4

    enum class State : uint8_t { UNINIT, READY, BUS_FAULT };

    // BOOT self-test duration (datasheet); the config writes follow it —
    // BOOT resets every register, so nothing may be configured before.
    static constexpr uint32_t BOOT_MS = 50;
    // Read cadence: continuous mode feeds us at ODR 25 Hz; sampling at
    // 10 Hz matches the control cycle and always finds a fresher sample.
    static constexpr uint32_t SAMPLE_MS = 100;

    Lps28(Transport& bus) : bus_(bus) {}

    // Multi-tick: probe WHO_AM_I (both addresses), soft-boot, configure
    // FS_MODE + ODR. False = not ready yet (waiting or backing off).
    bool begin(uint32_t nowMs) override;
    void start(uint32_t nowMs) override;   // arm the next sample window
    bool poll(uint32_t nowMs) override;    // true once per completed read

    float pressureBar() const override { return pressureBar_; }
    float temperatureC() const override { return tempC_; }
    uint32_t lastGoodMs() const override { return lastGoodMs_; }
    bool healthy() const override { return state_ == State::READY && readFails_ < K_MAX_FAILS; }
    const char* identity() const override { return identity_; }  // set at identification
    State state() const { return state_; }
    uint8_t addr() const { return addr_; }   // which address answered

    // FS_MODE=1 scaling; exposed for tests and for any future mode 0 use.
    static float pressureBarFromRaw(int32_t raw24) {
        return static_cast<float>(raw24) / 2048.0f / 1000.0f;  // hPa -> bar
    }
    static float temperatureCFromRaw(int16_t raw16) {
        return static_cast<float>(raw16) / 100.0f;
    }

private:
    static constexpr uint8_t K_MAX_FAILS = 5;
    enum class BootPhase : uint8_t { PROBE, BOOT_WAIT };
    void noteFail(uint32_t nowMs);

    Transport& bus_;
    uint8_t addr_ = ADDR;
    char identity_[24] = "LPS28 (unidentified)";
    State state_ = State::UNINIT;
    BootPhase bootPhase_ = BootPhase::PROBE;
    bool sampling_ = false;
    uint32_t deadlineMs_ = 0, backoffUntilMs_ = 0;
    uint8_t backoffShift_ = 0, readFails_ = 0;
    float pressureBar_ = 0.0f, tempC_ = 0.0f;
    uint32_t lastGoodMs_ = 0;
};

} // namespace press
