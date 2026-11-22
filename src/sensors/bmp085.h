// ============================================================================
// bmp085.h — BMP085 bench-phase pressure driver (BMP085 now, MS5837-30BA
// when the housing is potted — PLAN.md). Same non-blocking state machine and
// Transport contract as the MS5837 driver.
//
// Compensation per BMP085 datasheet Fig.4 (Rev 1.3) — note B6 = B5 - 4000:
// the -50000 constant floating around old drivers belongs to the legacy
// BMP380/390 API and produces physically impossible readings on BMP085
// hardware. The datasheet's own worked example (15.0 C, 69964 Pa from
// UT=27898 UP=23843) only reproduces with -4000; asserted in the tests.
// ============================================================================
#pragma once

#include <cstdint>

#include "pressure_bus.h"

namespace press {

class Bmp085 {
public:
    static constexpr uint8_t ADDR = 0x77;

    enum Reg : uint8_t {
        REG_CAL = 0xAA,   // 22 bytes AC1..MD
        REG_CTRL = 0xF4,
        REG_DATA = 0xF6,
        CMD_TEMP = 0x2E,
        CMD_PRESS = 0x34,  // | (oss << 6)
    };

    enum class State : uint8_t { UNINIT, READY, BUS_FAULT };
    enum class Phase : uint8_t { IDLE, TEMP_WAIT, PRESS_WAIT };

    // OSS 2: 13.5 ms conv, 0.04 hPa RMS — right ballpark for depth on the
    // bench (0.4 m resolution), still cheap.
    static constexpr uint8_t OSS = 2;
    static constexpr uint32_t TEMP_MS = 5;
    static constexpr uint32_t PRESS_MS = 14;

    Bmp085(Transport& bus, uint8_t addr = ADDR) : bus_(bus), addr_(addr) {}

    bool begin(uint32_t nowMs);   // load + sanity-check cal coefficients
    void start(uint32_t nowMs);   // temperature first (pressure needs B5)
    bool poll(uint32_t nowMs);    // true once per completed T+P cycle

    float pressureBar() const { return pressureBar_; }
    float temperatureC() const { return tempC_; }
    uint32_t lastGoodMs() const { return lastGoodMs_; }
    bool healthy() const { return state_ == State::READY && readFails_ < K_MAX_FAILS; }
    State state() const { return state_; }

    struct Raw { int32_t ut; uint32_t up; };
    struct Cal {
        int16_t ac1, ac2, ac3;
        uint16_t ac4, ac5, ac6;
        int16_t b1, b2, mb, mc, md;
    };
    // Datasheet Fig.3/4 math split so the driver can compute B5 at the
    // temperature read and reuse it for the pressure read of the same cycle
    // (the datasheet's own required order). compensate() composes both.
    static int32_t tempB5(const Cal& c, int32_t ut, float& tempCOut);
    static float pressurePa(const Cal& c, uint32_t up, int32_t b5, uint8_t oss);
    static void compensate(const Cal& c, const Raw& d, uint8_t oss,
                           float& barOut, float& tempCOut, int32_t& b5Out);

private:
    static constexpr uint8_t K_MAX_FAILS = 5;
    void noteFail(uint32_t nowMs);

    Transport& bus_;
    const uint8_t addr_;
    State state_ = State::UNINIT;
    Phase phase_ = Phase::IDLE;
    Cal cal_ = {};
    int32_t b5_ = 0;
    uint32_t deadlineMs_ = 0, backoffUntilMs_ = 0;
    uint8_t backoffShift_ = 0, readFails_ = 0;
    float pressureBar_ = 0.0f, tempC_ = 0.0f;
    uint32_t lastGoodMs_ = 0;
};

} // namespace press
