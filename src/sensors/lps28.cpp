#include "lps28.h"

#include <cstdio>
#include <initializer_list>

namespace press {

bool Lps28::begin(uint32_t nowMs) {
    if (state_ == State::READY) return true;
    if (nowMs < backoffUntilMs_) return false;

    if (bootPhase_ == BootPhase::PROBE) {
        // Identity before anything else. The XSA pad decides 0x5C vs 0x5D;
        // probing both turns a jumper question into a boot-log line.
        bool found = false;
        for (uint8_t candidate : {ADDR, ADDR_ALT}) {
            uint8_t id = 0;
            if (bus_.writeThenRead(candidate, REG_WHOAMI, &id, 1) && id == CHIP_ID) {
                addr_ = candidate;
                found = true;
                break;
            }
        }
        if (!found) { noteFail(nowMs); return false; }

        if (!bus_.writeRegData(addr_, REG_CTRL2, CTRL2_BOOT)) { noteFail(nowMs); return false; }
        deadlineMs_ = nowMs + BOOT_MS;
        bootPhase_ = BootPhase::BOOT_WAIT;
        return false;
    }

    // BOOT_WAIT: the self-test clears every register, so the operating
    // configuration is written only now — 4060 hPa full scale, 25 Hz, AVG 4.
    if (nowMs < deadlineMs_) return false;
    if (!bus_.writeRegData(addr_, REG_CTRL2, CTRL2_FS4060) ||
        !bus_.writeRegData(addr_, REG_CTRL1, CTRL1_RUN)) {
        bootPhase_ = BootPhase::PROBE;
        noteFail(nowMs);
        return false;
    }
    readFails_ = 0;
    backoffShift_ = 0;
    bootPhase_ = BootPhase::PROBE;
    state_ = State::READY;
    std::snprintf(identity_, sizeof(identity_), "LPS28DFW @0x%02X", addr_);
    return true;
}

void Lps28::start(uint32_t nowMs) {
    if (state_ != State::READY || sampling_ || nowMs < backoffUntilMs_) return;
    // Continuous mode: nothing to command, only a window to pace the reads.
    deadlineMs_ = nowMs + SAMPLE_MS;
    sampling_ = true;
}

bool Lps28::poll(uint32_t nowMs) {
    if (state_ != State::READY || !sampling_) return false;
    if (nowMs < deadlineMs_ || nowMs < backoffUntilMs_) return false;

    // One burst from 0x28: PRESS XL,L,H then auto-increment into TEMP L,H —
    // one transaction, so P and T always come from the same output image.
    uint8_t b[5];
    if (!bus_.writeThenRead(addr_, REG_PRESS, b, 5)) {
        sampling_ = false;
        noteFail(nowMs);
        return false;
    }
    int32_t raw = (static_cast<int32_t>(b[2]) << 16) |
                  (static_cast<int32_t>(b[1]) << 8) | b[0];
    if (raw & 0x800000) raw -= 0x1000000;   // 24-bit two's complement

    pressureBar_ = pressureBarFromRaw(raw);
    tempC_ = temperatureCFromRaw(static_cast<int16_t>((b[4] << 8) | b[3]));

    sampling_ = false;
    deadlineMs_ = 0;
    lastGoodMs_ = nowMs;
    readFails_ = 0;
    backoffShift_ = 0;
    return true;
}

void Lps28::noteFail(uint32_t nowMs) {
    if (readFails_ < 250) readFails_++;
    if (backoffShift_ < 4) backoffShift_++;
    backoffUntilMs_ = nowMs + (250u << backoffShift_);
    deadlineMs_ = 0;
    if (readFails_ >= K_MAX_FAILS && state_ == State::READY) state_ = State::BUS_FAULT;
}

} // namespace press
