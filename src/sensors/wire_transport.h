// ============================================================================
// wire_transport.h — press::Transport over an Arduino Wire bus (ESP32 only).
//
// Header-only on purpose: nothing in the native/host build includes it, so a
// Wire dependency can never leak into the tested driver code. The drivers
// themselves stay pure and are unit-tested against a fake bus.
//
// Same rule as the ADS1115 driver: never block past a hard transaction
// timeout. A false return is a bus fault, and the pressure driver's own
// retry/backoff handles it — the control task never waits on I2C.
//
// Bound to a bus by constructor, not by assumption: the depth sensors live
// on Wire1 (pins.h) so a wet-harness fault can never wedge the bus feeding
// the cell ADC. Each WireTransport carries its OWN latched-slave recovery
// (i2c_recovery.h), keyed to its pins — the same rescue the cell bus gets.
// ============================================================================
#pragma once

#include <Wire.h>

#include "sensors/i2c_recovery.h"
#include "sensors/pressure_bus.h"

namespace press {

class WireTransport : public Transport {
public:
    static constexpr uint8_t kTimeoutMs = 50;  // same cap as the ADC driver

    // Consecutive failed transactions before assuming "latched slave" rather
    // than "absent sensor" and bit-banging the bus. Three is enough to skip
    // the ordinary NACK of a missing probe yet fast enough that a real latch
    // clears well inside one backoff window.
    static constexpr uint8_t kRecoverAfter = 3;
    static constexpr uint32_t kRecoverGapMs = 5000;  // min gap between recoveries

    WireTransport(TwoWire& wire, int sdaPin, int sclPin)
        : wire_(wire), sda_(sdaPin), scl_(sclPin) {}

    // Bring the bus up with this rig's electrical rules: 100 kHz, 50 ms cap.
    void begin() {
        wire_.begin(sda_, scl_);
        wire_.setClock(100000);
        wire_.setTimeOut(kTimeoutMs);
    }

    bool writeByte(uint8_t addr, uint8_t val) override {
        wire_.beginTransmission(addr);
        wire_.write(val);
        return ok(wire_.endTransmission(true) == 0);
    }

    bool writeRegData(uint8_t addr, uint8_t reg, uint8_t data) override {
        wire_.beginTransmission(addr);
        wire_.write(reg);
        wire_.write(data);
        return ok(wire_.endTransmission(true) == 0);
    }

    bool writeThenRead(uint8_t addr, uint8_t reg, uint8_t* out, uint8_t n) override {
        wire_.beginTransmission(addr);
        wire_.write(reg);
        if (!ok(wire_.endTransmission(false) == 0)) return false;  // repeated start
        if (wire_.requestFrom(static_cast<int>(addr), static_cast<int>(n)) != n)
            return ok(false);
        for (uint8_t i = 0; i < n; i++) {
            const int v = wire_.read();
            if (v < 0) return ok(false);
            out[i] = static_cast<uint8_t>(v);
        }
        return ok(true);
    }

private:
    // Central fault accounting. One NACK is just a missing chip; a streak of
    // them on a bus that should have a device is the latched-SDA signature,
    // so unstick it with clocks+STOP. kRecoverGapMs stops an absent sensor
    // from turning every probe cycle into a recovery routine.
    bool ok(bool good) {
        if (good) {
            faults_ = 0;
            return true;
        }
        if (++faults_ >= kRecoverAfter && millis() - lastRecoverMs_ >= kRecoverGapMs) {
            lastRecoverMs_ = millis();
            faults_ = 0;
            i2cutil::recoverBus(wire_, sda_, scl_);
        }
        return false;
    }

    TwoWire& wire_;
    const int sda_;
    const int scl_;
    uint8_t faults_ = 0;
    uint32_t lastRecoverMs_ = 0;
};

}  // namespace press
