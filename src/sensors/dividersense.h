// ============================================================================
// dividersense.h — the signal chain every resistor-divider ADC input shares.
//
// Why this is a base class and not a copy: the pack sense already had a spike
// filter, a divider scale and a two-sided band verdict, and the coil rail needs
// the same three things with different numbers. The failure mode this avoids is
// the one the battery block in defaults.h documents — the same rule written out
// in five places, where moving one of them leaves four behind. Here the pipeline
// exists once and each input carries its own constants.
//
// The two-sided band is not decoration. `low()`-style "is it above the floor?"
// questions are PASSED by a sense node sitting on its rail, and this bench has
// actually read `vbat=34.71` from exactly that (docs/REVIEW.md 1 P1). It is also
// what makes an UNWIRED input fail closed: a disconnected ADC pin is
// high-impedance and can land anywhere, and either edge of the band then refuses.
// With a floor only, the same unwired pin can pass.
//
// Arduino-free by design: the caller reads analogReadMilliVolts and feeds
// addSample(), so the DSP that gates a dive-blocking verdict is host-testable.
// ============================================================================
#pragma once

#include <cstdint>

class DividerSense {
public:
    virtual ~DividerSense() = default;

    void begin() {}

    // Feed 8 samples per burst; one processed value emerges per 8 samples:
    // median (spike rejection) -> divider ratio and trim -> EMA.
    // Returns true on the call that produced a new processed value, which is how
    // a subclass can count processed values rather than ADC reads (the rail's
    // debounce counts sweeps, not samples).
    bool addSample(int adcMilliVolts);

    float volts() const { return _volts; }
    bool initialized() const { return _initialized; }

    // The band in force. Defaults to the compile-time constants of the subclass;
    // writable where a value is a settings key (the pack floor is, and the HB/HUD
    // path and the alarm path must read the same one — that split is a closed
    // defect, see battery.h).
    void setBand(float minV, float maxV) { _minV = minV; _maxV = maxV; }
    float minVolts() const { return _minV; }
    float maxVolts() const { return _maxV; }

protected:
    DividerSense(float ratio, float cal, float minV, float maxV)
        : _ratio(ratio), _cal(cal), _minV(minV), _maxV(maxV) {}

    // Band verdicts on the processed value. Named for what they mean physically,
    // which differs per input; the subclasses give them the words.
    bool belowBand() const { return _initialized && _volts < _minV; }
    bool aboveBand() const { return _initialized && _volts > _maxV; }

    // Called on every burst, after the new processed value is in. Default does
    // nothing; the rail uses it to keep a run counter.
    virtual void onProcessed() {}

private:
    int _buf[8] = {};
    uint8_t _n = 0;
    float _volts = 0.0f;
    bool _initialized = false;
    float _ratio;
    float _cal;
    float _minV;
    float _maxV;
};
