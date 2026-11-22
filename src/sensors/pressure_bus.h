// ============================================================================
// pressure_bus.h — transport contract for the pressure drivers.
//
// The driver state machines are pure logic: all I2C traffic goes through
// this interface, so the exact same code is unit-tested on the host against
// a fake bus (with fault injection) and runs on the ESP32 against Wire.
//
// Hard rule (same as the ADS1115 driver): implementations MUST NOT BLOCK.
// Every call returns true/false within a bounded hardware timeout; the
// drivers treat false as a bus fault and apply their own retry/backoff.
// ============================================================================
#pragma once

#include <cstdint>

namespace press {

class Transport {
public:
    virtual ~Transport() = default;

    // Single-byte write transaction (device commands).
    virtual bool writeByte(uint8_t addr, uint8_t val) = 0;

    // Register-addressed write: reg, then data byte(s) in one transaction.
    virtual bool writeRegData(uint8_t addr, uint8_t reg, uint8_t data) = 0;

    // Write a pointer/command byte, repeated-start, read n bytes.
    // For MS5837 the "pointer" is its command byte (ADC read / PROM read);
    // for BMP085 it is a plain register.
    virtual bool writeThenRead(uint8_t addr, uint8_t reg, uint8_t* out, uint8_t n) = 0;
};

} // namespace press
