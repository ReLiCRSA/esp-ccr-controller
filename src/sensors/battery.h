// Battery sense: pure signal processing over ADC millivolt samples.
// The hardware caller (main.cpp) reads analogReadMilliVolts from the divider
// pin and feeds addSample(); keeping the DSP side Arduino-free makes the
// spike rejection, divider scaling and low-voltage gate unit-testable —
// these numbers gate a dive-blocking alarm.
#pragma once

#include <cstdint>

#include "config/defaults.h"

class Battery {
public:
    void begin() {}

    // Feed 8 samples per update burst; one processed value emerges per 8
    // samples: median (spike rejection) -> divider ratio -> EMA.
    void addSample(int adcMilliVolts);

    float volts() const { return _volts; }
    bool low() const { return _initialized && _volts < VBAT_MIN_V; }
    bool initialized() const { return _initialized; }

private:
    int _buf[8] = {};
    uint8_t _n = 0;
    float _volts = 0.0f;
    bool _initialized = false;
};
