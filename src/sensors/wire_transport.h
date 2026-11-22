// ============================================================================
// wire_transport.h — press::Transport over the Arduino Wire bus (ESP32 only).
//
// Header-only on purpose: nothing in the native/host build includes it, so a
// Wire dependency can never leak into the tested driver code. The drivers
// themselves stay pure and are unit-tested against a fake bus.
//
// Same rule as the ADS1115 driver: never block past a hard transaction
// timeout. A false return is a bus fault, and the pressure driver's own
// retry/backoff handles it — the control task never waits on I2C.
// ============================================================================
#pragma once

#include <Wire.h>

#include "sensors/pressure_bus.h"

namespace press {

class WireTransport : public Transport {
public:
    static constexpr uint8_t kTimeoutMs = 50;  // same cap as the ADC driver

    bool writeByte(uint8_t addr, uint8_t val) override {
        Wire.beginTransmission(addr);
        Wire.write(val);
        return Wire.endTransmission(true) == 0;
    }

    bool writeRegData(uint8_t addr, uint8_t reg, uint8_t data) override {
        Wire.beginTransmission(addr);
        Wire.write(reg);
        Wire.write(data);
        return Wire.endTransmission(true) == 0;
    }

    bool writeThenRead(uint8_t addr, uint8_t reg, uint8_t* out, uint8_t n) override {
        Wire.beginTransmission(addr);
        Wire.write(reg);
        if (Wire.endTransmission(false) != 0) return false;  // repeated start
        if (Wire.requestFrom(static_cast<int>(addr), static_cast<int>(n)) != n) return false;
        for (uint8_t i = 0; i < n; i++) {
            const int v = Wire.read();
            if (v < 0) return false;
            out[i] = static_cast<uint8_t>(v);
        }
        return true;
    }
};

}  // namespace press
