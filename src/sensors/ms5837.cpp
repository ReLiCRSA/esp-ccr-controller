#include "ms5837.h"

#include <cstdio>

namespace press {

// ---------------------------------------------------------------------------
// PROM CRC-4 (datasheet algorithm, poly 0x3000 over the 112 data bits;
// word 0 CRC nibble zeroed, word 7 subsidiary value zeroed).
// ---------------------------------------------------------------------------
bool Ms5837::checkPromCrc(const uint16_t prom[8]) {
    uint16_t rem = 0;
    for (int cnt = 0; cnt < 16; ++cnt) {
        const int w = cnt >> 1;
        uint16_t word = prom[w];
        if (w == 0) word &= 0x0FFF;  // CRC nibble replaced by 0
        if (w == 7) word = 0;        // subsidiary value not covered
        rem ^= (cnt & 1) ? (word & 0x00FF) : static_cast<uint16_t>(word >> 8);
        for (int bit = 0; bit < 8; ++bit)
            rem = (rem & 0x8000) ? static_cast<uint16_t>((rem << 1) ^ 0x3000)
                                 : static_cast<uint16_t>(rem << 1);
    }
    const uint8_t calc = static_cast<uint8_t>((rem >> 12) & 0x000F);
    return calc == (prom[0] >> 12);
}

// ---------------------------------------------------------------------------
// Datasheet REV C compensation (MS5837-30BA scaling) + second-order
// temperature compensation. P result unit is 0.1 mbar; converted to bar.
// ---------------------------------------------------------------------------
void Ms5837::compensate(const uint16_t c[7], const Raw& d, float& barOut, float& tempCOut) {
    const int32_t dT = static_cast<int32_t>(d.d2) - static_cast<int32_t>(c[5]) * 256;
    const int32_t temp = 2000 + static_cast<int32_t>((static_cast<int64_t>(dT) * c[6]) >> 23);

    int64_t off = (static_cast<int64_t>(c[2]) << 16) + ((static_cast<int64_t>(c[4]) * dT) >> 7);
    int64_t sens = (static_cast<int64_t>(c[1]) << 15) + ((static_cast<int64_t>(c[3]) * dT) >> 8);

    // Second order (REV C flow chart): below 20 C and above 20 C branches.
    int64_t offi = 0, sensi = 0;
    int32_t ti = 0;
    if (temp / 100 < 20) {
        ti = static_cast<int32_t>((3 * static_cast<int64_t>(dT) * dT) >> 33);
        offi = (3 * static_cast<int64_t>(temp - 2000) * (temp - 2000)) >> 1;
        sensi = (5 * static_cast<int64_t>(temp - 2000) * (temp - 2000)) >> 3;
        if (temp / 100 < -15) {
            offi += 7 * static_cast<int64_t>(temp + 1500) * (temp + 1500);
            sensi += 4 * static_cast<int64_t>(temp + 1500) * (temp + 1500);
        }
    } else {
        ti = static_cast<int32_t>((2 * static_cast<int64_t>(dT) * dT) >> 37);
        offi = (static_cast<int64_t>(temp - 2000) * (temp - 2000)) >> 4;
    }
    const int64_t off2 = off - offi;
    const int64_t sens2 = sens - sensi;
    const int64_t temp2 = static_cast<int64_t>(temp) - ti;

    int64_t p01mbar = ((((static_cast<int64_t>(d.d1) * sens2) >> 21) - off2) >> 13);
    p01mbar = (p01mbar < 0) ? 0 : p01mbar;  // sensor floor at vacuum

    barOut = static_cast<float>(p01mbar) / 10000.0f;      // 0.1 mbar -> bar
    tempCOut = static_cast<float>(temp2) / 100.0f;
}

// ---------------------------------------------------------------------------
// Non-blocking boot: reset -> (15 ms) -> PROM words -> CRC -> READY.
// Any bus fault arms exponential backoff (250 ms .. 4 s) and re-tries from
// the top; the caller only ever sees true/false, never a hang.
// ---------------------------------------------------------------------------
bool Ms5837::begin(uint32_t nowMs) {
    if (state_ == State::READY) return true;
    if (nowMs < backoffUntilMs_) return false;

    if (state_ == State::UNINIT || state_ == State::BUS_FAULT) {
        if (!bus_.writeByte(addr_, CMD_RESET)) { noteFail(nowMs); return false; }
        bootDeadlineMs_ = nowMs + BOOT_MS;
        state_ = State::RESET_WAIT;
        return false;
    }
    if (state_ == State::RESET_WAIT) {
        if (nowMs < bootDeadlineMs_) return false;
        state_ = State::PROM_READ;
    }
    // PROM_READ: 8 words of 2 bytes. Burst of 8 tiny transactions — each is
    // individually guaranteed non-blocking by the Transport contract.
    uint16_t prom[8];
    for (int i = 0; i < 8; ++i) {
        uint8_t b[2];
        if (!bus_.writeThenRead(addr_, static_cast<uint8_t>(CMD_PROM + 2 * i), b, 2)) {
            noteFail(nowMs);
            return false;
        }
        prom[i] = (static_cast<uint16_t>(b[0]) << 8) | b[1];
    }
    crcOk_ = checkPromCrc(prom);
    if (!crcOk_) {
        // CRC is a memory-integrity check, not a bus hiccup: repeated CRC
        // failure means the sensor itself is untrustworthy. Backoff keeps
        // retrying (it can be marginal wiring), health stays false.
        noteFail(nowMs);
        return false;
    }
    for (int i = 1; i <= 6; ++i) c_[i] = prom[i];
    readFails_ = 0;
    backoffShift_ = 0;
    state_ = State::READY;
    // Identity = family + unit: the PROM CRC just passed, so these are the
    // factory coefficients of THIS chip. Stronger than a WHO_AM_I.
    std::snprintf(identity_, sizeof(identity_), "MS5837-30BA @0x%02X", addr_);
    return true;
}

void Ms5837::start(uint32_t nowMs) {
    if (state_ != State::READY || phase_ != PH_IDLE || nowMs < backoffUntilMs_) return;
    if (startConv(static_cast<uint8_t>(CMD_CONV_D1 | OSR4096), nowMs)) phase_ = PH_D1;
}

bool Ms5837::startConv(uint8_t cmd, uint32_t nowMs) {
    if (!bus_.writeByte(addr_, cmd)) { noteFail(nowMs); return false; }
    convDeadlineMs_ = nowMs + CONV_MS;
    return true;
}

bool Ms5837::harvest(uint32_t& out, uint32_t /*nowMs*/) {
    uint8_t b[3];
    if (!bus_.writeThenRead(addr_, CMD_ADC_READ, b, 3)) return false;
    out = (static_cast<uint32_t>(b[0]) << 16) | (static_cast<uint32_t>(b[1]) << 8) | b[2];
    return true;
}

bool Ms5837::poll(uint32_t nowMs) {
    if (state_ != State::READY || phase_ == PH_IDLE) return false;
    if (nowMs < convDeadlineMs_ || nowMs < backoffUntilMs_) return false;

    if (phase_ == PH_D1) {
        if (!harvest(d1_, nowMs)) { phase_ = PH_IDLE; noteFail(nowMs); return false; }
        if (startConv(static_cast<uint8_t>(CMD_CONV_D2 | OSR4096), nowMs)) phase_ = PH_D2;
        else phase_ = PH_IDLE;  // startConv already armed backoff
        return false;
    }

    if (!harvest(d2_, nowMs)) { phase_ = PH_IDLE; noteFail(nowMs); return false; }
    phase_ = PH_IDLE;
    convDeadlineMs_ = 0;

    Raw d{d1_, d2_};
    compensate(c_, d, pressureBar_, tempC_);
    lastGoodMs_ = nowMs;
    readFails_ = 0;
    backoffShift_ = 0;
    return true;
}

void Ms5837::noteFail(uint32_t nowMs) {
    if (readFails_ < 250) readFails_++;
    if (backoffShift_ < 4) backoffShift_++;
    const uint32_t delay = 250u << backoffShift_;  // 500..4000 ms
    backoffUntilMs_ = nowMs + delay;
    phase_ = PH_IDLE;
    convDeadlineMs_ = 0;
    if (readFails_ >= K_MAX_FAILS && state_ == State::READY) state_ = State::BUS_FAULT;
    if (state_ == State::PROM_READ || state_ == State::RESET_WAIT) state_ = State::BUS_FAULT;
}

} // namespace press
