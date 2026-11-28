// ============================================================================
// rail.h — the coil rail's divider input (the ~12 V the valve actually runs on).
//
// This input exists because of docs/REVIEW.md 1 P9: `vbat` is the pack, and the
// one battery verdict in the machine was a verdict about a cell. A pack at 3.9 V
// with a dead boost stage — or a boost that current-limits the moment the coil
// asks for current — passes every check the firmware had, unlocks AUTO, and
// produces a dive file that looks like a working loop with no oxygen in it,
// because `pulses=` counts what the firmware commanded the valve, not what the
// valve did.
//
// Whether this input is READ AT ALL is a build decision: VRAIL_SENSE in
// config/defaults.h. It is 0 today because the divider is not fitted, and the
// point of that constant is that "not judged" stays a statement the firmware
// makes out loud (boot line, HB `vrail=0.00`) rather than an absence a reader
// mistakes for a pass.
// ============================================================================
#pragma once

#include <cstdint>

#include "config/defaults.h"
#include "sensors/dividersense.h"

class Rail : public DividerSense {
public:
    Rail() : DividerSense(VRAIL_DIVIDER_RATIO, VRAIL_CAL, VRAIL_MIN_V,
                          VRAIL_IMPLAUSIBLE_V) {}

    // A reading above the band on a rail this module is supposed to hold fixed is
    // either a coil outside its rated supply or a divider that has fallen open.
    // Both are the same fact to act on, and neither is a pass.
    bool implausible() const { return aboveBand(); }

    // LOW is the one verdict here that has to be patient, and the reason is the
    // sampling, not the chemistry: the rail droops when the coil is energised and
    // SensorTask's 8-sample burst lands on a pulse about once in seven. One
    // low sweep is a measurement of a pulse; VRAIL_LOW_SAMPLES in a row is a
    // power supply that is not there. See defaults.h.
    bool low() const { return _lowRun >= VRAIL_LOW_SAMPLES; }

    /// "We cannot vouch for this rail." Never-sampled is inside that on purpose:
    /// an input nobody read is not evidence that the coil has voltage. Callers
    /// that must not act on an unrated build consult VRAIL_SENSE, not this.
    bool untrustworthy() const { return !initialized() || low() || implausible(); }

    /// True once at least one divider sweep has arrived. With VRAIL_SENSE 0 this
    /// stays false forever, which is what HB's `vrail=0.00` means.
    bool sensed() const { return initialized(); }

private:
    void onProcessed() override {
        if (belowBand()) {
            if (_lowRun < 255) ++_lowRun;
        } else {
            _lowRun = 0;
        }
    }
    uint8_t _lowRun = 0;
};
