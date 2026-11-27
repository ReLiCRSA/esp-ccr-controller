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
    bool initialized() const { return _initialized; }

    // The floor in force. Defaults to the compile-time chemistry value and is
    // overwritten from the applied settings, because `VBAT_MIN` is writable: two
    // thresholds in this machine (the HUD/HB path and the alarm/pre-dive path)
    // used to read different ones, which is how a diver's own setting could
    // disagree with the board's own warning light.
    void setMinVolts(float v) { _minV = v; }
    float minVolts() const { return _minV; }

    // A reading this high cannot be a pack of this chemistry: the divider has
    // fallen open, the sense pin has come loose, or the top leg is on the wrong
    // rail. Reported separately from `low()` because the two need different
    // words on a console, but never separately in the decision: an unreadable
    // pack is not a healthy pack.
    bool implausible() const { return _initialized && _volts > VBAT_IMPLAUSIBLE_V; }

    // True when the pack must not be trusted for a dive: below the floor, or too
    // high to be a pack at all. The second half is the part that used to be
    // missing, and it is the fail-closed half: a gate that only asks "above the
    // floor?" is passed by a sense node sitting on its rail, which on this bench
    // has actually been read as vbat=34.71.
    bool low() const {
        return _initialized && (_volts < _minV || _volts > VBAT_IMPLAUSIBLE_V);
    }

private:
    int _buf[8] = {};
    uint8_t _n = 0;
    float _volts = 0.0f;
    bool _initialized = false;
    float _minV = VBAT_MIN_V;
};
