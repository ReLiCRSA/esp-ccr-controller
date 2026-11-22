#include "bmp085.h"

namespace press {

int32_t Bmp085::tempB5(const Cal& c, int32_t ut, float& tempCOut) {
    // Temperature (datasheet Fig.3): B5 drives the pressure chain too.
    const int32_t x1 = ((ut - static_cast<int32_t>(c.ac6)) * static_cast<int32_t>(c.ac5)) >> 15;
    const int32_t x2 = (static_cast<int32_t>(c.mc) << 11) / (x1 + c.md);
    const int32_t b5 = x1 + x2;
    tempCOut = static_cast<float>((b5 + 8) >> 4) / 10.0f;  // 0.1 C -> C
    return b5;
}

float Bmp085::pressurePa(const Cal& c, uint32_t up, int32_t b5, uint8_t oss) {
    // Pressure (Fig.4). B6 = B5 - 4000 per BMP085 datasheet (see header).
    const int32_t b6 = b5 - 4000;
    int32_t x1 = (static_cast<int32_t>(c.b2) * ((b6 * b6) >> 12)) >> 11;
    int32_t x2 = (static_cast<int32_t>(c.ac2) * b6) >> 11;
    int32_t x3 = x1 + x2;
    const int32_t b3 = ((((static_cast<int32_t>(c.ac1) * 4 + x3) << oss) + 2) >> 2);
    x1 = (static_cast<int32_t>(c.ac3) * b6) >> 13;
    x2 = (static_cast<int32_t>(c.b1) * ((b6 * b6) >> 12)) >> 16;
    x3 = ((x1 + x2 + 2) >> 2);
    const uint32_t b4 = static_cast<uint32_t>(c.ac4) * static_cast<uint32_t>(x3 + 32768) >> 15;
    if (b4 == 0) return 0.0f;  // corrupt cal: refuse to divide

    const uint32_t b7 = (up - static_cast<uint32_t>(b3)) * (static_cast<uint32_t>(50000) >> oss);
    int32_t p = (b7 < 0x80000000u) ? static_cast<int32_t>((b7 * 2) / b4)
                                   : static_cast<int32_t>((b7 / b4) * 2);
    x1 = (p >> 8) * (p >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * p) >> 16;
    p += (x1 + x2 + 3791) >> 4;
    return p < 0 ? 0.0f : static_cast<float>(p);
}

void Bmp085::compensate(const Cal& c, const Raw& d, uint8_t oss,
                        float& barOut, float& tempCOut, int32_t& b5Out) {
    b5Out = tempB5(c, d.ut, tempCOut);
    barOut = pressurePa(c, d.up, b5Out, oss) / 100000.0f;  // Pa -> bar
}

bool Bmp085::begin(uint32_t nowMs) {
    if (state_ == State::READY) return true;
    if (nowMs < backoffUntilMs_) return false;

    uint8_t raw[22];
    if (!bus_.writeThenRead(addr_, REG_CAL, raw, 22)) { noteFail(nowMs); return false; }

    auto be16 = [&](int i) { return static_cast<int16_t>((raw[i] << 8) | raw[i + 1]); };
    cal_.ac1 = be16(0);  cal_.ac2 = be16(2);  cal_.ac3 = be16(4);
    cal_.ac4 = static_cast<uint16_t>(be16(6));
    cal_.ac5 = static_cast<uint16_t>(be16(8));
    cal_.ac6 = static_cast<uint16_t>(be16(10));
    cal_.b1 = be16(12);  cal_.b2 = be16(14);
    cal_.mb = be16(16);  cal_.mc = be16(18);  cal_.md = be16(20);

    // Uninitialized EPROM reads as all-0x00 or all-0xFF; a sane AC1 proves
    // the calibration is real before we trust any reading derived from it.
    if (cal_.ac1 == 0 || cal_.ac1 == -1 || cal_.ac1 < 100 || cal_.ac1 > 2000) {
        noteFail(nowMs);
        return false;
    }
    readFails_ = 0;
    backoffShift_ = 0;
    state_ = State::READY;
    return true;
}

void Bmp085::start(uint32_t nowMs) {
    if (state_ != State::READY || phase_ != Phase::IDLE || nowMs < backoffUntilMs_) return;
    if (!bus_.writeRegData(addr_, REG_CTRL, CMD_TEMP)) { noteFail(nowMs); return; }
    deadlineMs_ = nowMs + TEMP_MS;
    phase_ = Phase::TEMP_WAIT;
}

bool Bmp085::poll(uint32_t nowMs) {
    if (state_ != State::READY || phase_ == Phase::IDLE) return false;
    if (nowMs < deadlineMs_ || nowMs < backoffUntilMs_) return false;

    if (phase_ == Phase::TEMP_WAIT) {
        uint8_t b[2];
        if (!bus_.writeThenRead(addr_, REG_DATA, b, 2)) { phase_ = Phase::IDLE; noteFail(nowMs); return false; }
        const int32_t ut = (static_cast<int32_t>(b[0]) << 8) | b[1];
        tempC_ = 0.0f;
        b5_ = tempB5(cal_, ut, tempC_);  // datasheet order: T before P
        if (!bus_.writeRegData(addr_, REG_CTRL,
                               static_cast<uint8_t>(CMD_PRESS | (OSS << 6)))) {
            phase_ = Phase::IDLE; noteFail(nowMs); return false;
        }
        deadlineMs_ = nowMs + PRESS_MS;
        phase_ = Phase::PRESS_WAIT;
        return false;
    }

    uint8_t b[3];
    if (!bus_.writeThenRead(addr_, REG_DATA, b, 3)) { phase_ = Phase::IDLE; noteFail(nowMs); return false; }
    const uint32_t up = ((static_cast<uint32_t>(b[0]) << 16) |
                         (static_cast<uint32_t>(b[1]) << 8) | b[2]) >> (8 - OSS);
    pressureBar_ = pressurePa(cal_, up, b5_, OSS) / 100000.0f;  // Pa -> bar

    phase_ = Phase::IDLE;
    deadlineMs_ = 0;
    lastGoodMs_ = nowMs;
    readFails_ = 0;
    backoffShift_ = 0;
    return true;
}

void Bmp085::noteFail(uint32_t nowMs) {
    if (readFails_ < 250) readFails_++;
    if (backoffShift_ < 4) backoffShift_++;
    backoffUntilMs_ = nowMs + (250u << backoffShift_);
    deadlineMs_ = 0;
    if (readFails_ >= K_MAX_FAILS && state_ == State::READY) state_ = State::BUS_FAULT;
}

} // namespace press
