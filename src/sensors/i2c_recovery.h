// ============================================================================
// i2c_recovery.h — shared bit-banged I2C bus recovery, ESP32 only.
//
// The classic I2C lockup: a slave latches SDA low mid-transaction (reset
// mid-read, brownout, noisy harness) and no amount of master retrying frees
// it. The fix is on the wire, not in software: clock SCL until the slave
// releases SDA, then issue a STOP, then re-init the Wire master on a known-
// idle bus.
//
// One function, used by BOTH buses: the ADS1115/cell bus (21/22) and the
// depth bus (Wire1, 18/19). It lives in a header alongside wire_transport.h
// because it touches Arduino pins directly — the native test build must
// never see it. What is unit-tested is the POLICY that decides when to call
// it (driver backoffs, transport fault counters), not this mechanism.
//
// Never call from an ISR; a full cycle is ~160 us of pin twiddling plus a
// Wire re-init — microseconds, safe on any task, but still sequential code.
// ============================================================================
#pragma once

#include <Arduino.h>
#include <Wire.h>

namespace i2cutil {

// Returns true if SDA was seen released (a genuine unstuck), false if the
// line stayed low — a shorted bus, not a latched slave, and no amount of
// clocking will help; the caller's backoff must handle that.
inline bool recoverBus(TwoWire& wire, int sda, int scl) {
    pinMode(scl, OUTPUT);
    pinMode(sda, INPUT);  // external pull-ups do the pulling

    bool sdaReleased = false;
    for (int i = 0; i < 16 && !sdaReleased; ++i) {
        digitalWrite(scl, LOW);
        delayMicroseconds(5);
        digitalWrite(scl, HIGH);
        delayMicroseconds(5);
        sdaReleased = (digitalRead(sda) == HIGH);
    }

    // STOP condition: SDA low->high while SCL is high.
    pinMode(sda, OUTPUT);
    digitalWrite(sda, LOW);
    delayMicroseconds(5);
    digitalWrite(scl, HIGH);
    delayMicroseconds(5);
    digitalWrite(sda, HIGH);
    delayMicroseconds(5);

    // Re-init the Wire master on top of a known-idle bus. Same electrical
    // budget as every other transaction on this rig: 100 kHz, 50 ms cap.
    wire.end();
    wire.begin(sda, scl);
    wire.setClock(100000);
    wire.setTimeOut(50);

    return sdaReleased;
}

}  // namespace i2cutil
