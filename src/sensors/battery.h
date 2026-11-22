// Battery sense: ADC1 pin + resistive divider, sampled once per second.
// Median-filtered 8-sample burst, then EMA. Kept deliberately dumb and safe.
#pragma once
#include <Arduino.h>
#include "config/defaults.h"

class Battery {
public:
    void begin() {}

    // Call at ~VBAT_SAMPLE_INTERVAL_MS cadence from the SensorTask.
    void update();

    float volts() const { return _volts; }
    bool low() const { return _volts < VBAT_MIN_V; }
    bool initialized() const { return _initialized; }

private:
    float    _volts = 0.0f;
    bool     _initialized = false;
};
