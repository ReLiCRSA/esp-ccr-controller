// Battery sense: the pack's divider input.
// The hardware caller (main.cpp) reads analogReadMilliVolts from the divider
// pin and feeds addSample(); the DSP side lives in sensors/dividersense.h,
// Arduino-free, because these numbers gate a dive-blocking alarm and have to be
// unit-testable.
//
// The chemistry, the floor, the ceiling and the writable band all come from ONE
// constant — VBAT_CELLS in config/defaults.h. Do not restate them here: the last
// time a battery number was copied out of this file into a caller, the copy
// outlived the change and refused a charged pack for a month (see defaults.h and
// docs/REVIEW.md 1 P1).
#pragma once

#include <cstdint>

#include "config/defaults.h"
#include "sensors/dividersense.h"

class Battery : public DividerSense {
public:
    Battery() : DividerSense(VBAT_DIVIDER_RATIO, VBAT_CAL, VBAT_MIN_V,
                             VBAT_IMPLAUSIBLE_V) {}

    // The floor in force. Defaults to the compile-time chemistry value and is
    // overwritten from the applied settings, because `VBAT_MIN` is writable: two
    // thresholds in this machine (the HUD/HB path and the alarm/pre-dive path)
    // used to read different ones, which is how a diver's own setting could
    // disagree with the board's own warning light.
    void setMinVolts(float v) { setBand(v, maxVolts()); }

    // A reading this high cannot be a pack of this chemistry: the divider has
    // fallen open, the sense pin has come loose, or the top leg is on the wrong
    // rail. Reported separately from `low()` because the two need different
    // words on a console, but never separately in the decision: an unreadable
    // pack is not a healthy pack.
    bool implausible() const { return aboveBand(); }

    // True when the pack must not be trusted for a dive: below the floor, or too
    // high to be a pack at all. The second half is the part that used to be
    // missing, and it is the fail-closed half: a gate that only asks "above the
    // floor?" is passed by a sense node sitting on its rail, which on this bench
    // has actually been read as vbat=34.71.
    bool low() const { return belowBand() || aboveBand(); }
};
