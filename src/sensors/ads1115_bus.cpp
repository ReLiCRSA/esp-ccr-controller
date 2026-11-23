// ============================================================================
// ads1115_bus.cpp — implementation. See header for the safety rationale.
// ============================================================================
#include <Wire.h>
#include <algorithm>
#include "sensors/ads1115_bus.h"
#include "sensors/i2c_recovery.h"
#include "config/pins.h"
#include "config/defaults.h"

// CONFIG register layout written on each MUX switch:
//   OS=1 (dummy in continuous) | MUX=AINx/GND | PGA=101 (x16) | MODE=0 cont.
//   | CPQ off | DR=0 | SPS=100 (16 SPS)
uint16_t Ads1115Bus::configForChannel(uint8_t ch) {
    uint16_t mux = (0b100 + ch) << 12;          // single-ended AIN0..AIN2 vs GND
    uint16_t pga = 0b101 << 9;                  // +/-0.256 V (gain x16)
    uint16_t sps = 0b100 << 0;                  // 16 SPS
    return 0x8000 | mux | pga | sps;            // MODE=0 (continuous), CPQ off
}

void Ads1115Bus::begin() {
    _presentMask = CELLS_PRESENT_MASK;

    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    Wire.setClock(100000);          // 100 kHz. Not 400. See PLAN.md 2.3.
    Wire.setTimeOut(50);            // hard cap: no transaction may spin >50 ms

    _ch = nextPresentChannel(0xFF); // first present channel
    _phase = Phase::PH_SETUP;
    _nextActionMs = millis();
}

bool Ads1115Bus::writeReg(uint8_t reg, uint16_t value) {
    Wire.beginTransmission(ADDR);
    Wire.write(reg);
    Wire.write(static_cast<uint8_t>(value >> 8));
    Wire.write(static_cast<uint8_t>(value & 0xFF));
    const uint8_t err = Wire.endTransmission(true);
    if (err != 0) {
        _stats.i2cErrors++;
        _stats.consecutiveFails++;
        return false;
    }
    _stats.consecutiveFails = 0;
    return true;
}

bool Ads1115Bus::readReg(uint8_t reg, uint16_t& value) {
    // Repeated start: address + register pointer, no STOP, then read 2 bytes.
    Wire.beginTransmission(ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) {
        _stats.i2cErrors++;
        _stats.consecutiveFails++;
        return false;
    }
    if (Wire.requestFrom(static_cast<int>(ADDR), 2) != 2) {
        _stats.i2cErrors++;
        _stats.consecutiveFails++;
        return false;
    }
    value = (static_cast<uint16_t>(Wire.read()) << 8) | Wire.read();
    _stats.consecutiveFails = 0;
    return true;
}

// Bit-banged I2C bus recovery: if a slave latched SDA low mid-transaction
// (the classic lockup), clock SCL until it releases, then issue a STOP.
// The mechanism is shared with the depth bus (i2c_recovery.h); what stays
// here is the ADC driver's own bookkeeping around it.
bool Ads1115Bus::recoverBus() {
    _stats.recoveries++;

    const bool sdaReleased = i2cutil::recoverBus(Wire, PIN_I2C_SDA, PIN_I2C_SCL);

    // Restart the channel cycle cleanly and re-program the config register.
    _ch = nextPresentChannel(0xFF);
    _phase = Phase::PH_SETUP;
    _nextActionMs = millis() + 10;
    return sdaReleased;
}

uint8_t Ads1115Bus::nextPresentChannel(uint8_t ch) const {
    // Returns the first present channel at/after ch+1 (wrapping),
    // or 0xFF when the mask is empty (programming error guard).
    for (int step = 1; step <= CHANNELS; ++step) {
        const uint8_t cand = static_cast<uint8_t>((ch + step) % CHANNELS);
        if (_presentMask & (1u << cand)) return cand;
    }
    return 0xFF;
}

void Ads1115Bus::tick(uint32_t nowMs) {
    if (static_cast<int32_t>(nowMs - _nextActionMs) < 0) return;

    // Fault path: too many consecutive transaction failures -> one recovery
    // attempt, then resume. A recovery never retries inline: the next fault
    // comes through the door as a fresh event, on the next tick.
    if (_stats.consecutiveFails >= ADS_FAULTS_BEFORE_RECOVERY) {
        recoverBus();
        _stats.consecutiveFails = 0; // give the recovered bus a clean slate
        // Persistent failure (dead/disconnected ADC): back off so we never
        // flood a faulting bus. Doubles per consecutive recovery with no
        // successful sample in between, capped; reset to base on any sample.
        _recoveryBackoffMs = std::min<uint32_t>(_recoveryBackoffMs * 2, 5000);
        _nextActionMs = millis() + _recoveryBackoffMs;
        return;
    }

    switch (_phase) {
        case Phase::PH_SETUP: {
            // Exactly one config write per channel cycle. Never re-write it
            // "for safety" — MUX/CONFIG churn during continuous conversions
            // is the documented source of phantom readings.
            if (!writeReg(REG_CONFIG, configForChannel(_ch))) break;
            _phase = Phase::PH_READ_DISCARD;
            _nextActionMs = nowMs + ADS_SLOT_MS;
            break;
        }
        case Phase::PH_READ_DISCARD: {
            uint16_t dummy;
            if (!readReg(REG_CONVERSION, dummy)) break; // contaminated: toss
            _phase = Phase::PH_READ_VALID;
            _nextActionMs = nowMs + ADS_SLOT_MS;
            break;
        }
        case Phase::PH_READ_VALID: {
            uint16_t v;
            if (!readReg(REG_CONVERSION, v)) break;
            _raw[_ch] = static_cast<int16_t>(v);
            _lastValidMs[_ch] = nowMs;
            _stats.samples++;
            _recoveryBackoffMs = 250; // bus proven alive -> forget the grudge
            _ch = nextPresentChannel(_ch);
            _phase = Phase::PH_SETUP;
            _nextActionMs = nowMs + ADS_SETUP_GAP_MS;
            break;
        }
    }
}

bool Ads1115Bus::fresh(uint8_t ch, uint32_t nowMs) const {
    if (!(_presentMask & (1u << ch))) return false;
    if (_lastValidMs[ch] == 0) return false;
    return (nowMs - _lastValidMs[ch]) < ADS_STALE_MS;
}

bool Ads1115Bus::anyStale(uint32_t nowMs) const {
    for (uint8_t ch = 0; ch < CHANNELS; ++ch) {
        if (_presentMask & (1u << ch)) {
            if (!fresh(ch, nowMs)) return true;
        }
    }
    return false;
}
