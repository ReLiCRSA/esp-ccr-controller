// ============================================================================
// depth_announce.cpp — see depth_announce.h.
//
// The float goes through numfmt::fixed, like every other number this firmware
// prints: printf's %f drags newlib's float tables into the task that prints
// them, and tools/float_audit.py keeps that rule enforced.
// ============================================================================
#include "depth_announce.h"

#include "util/numfmt.h"

#include <cstdio>

namespace press {

Announcer::Action Announcer::update(uint32_t nowMs, bool locked, bool saneSample,
                                    float pressureBar) {
    if (!locked) return Action::NONE;   // nothing owns depth: nothing to report
    if (!lockSeen_) {
        lockSeen_ = true;
        lockSeenMs_ = nowMs;
    }
    lastBar_ = pressureBar;

    // The reading wins over the clock: the first sample the control path would
    // use is the line the bench wants, whatever the timing.
    if (saneSample && !readyDone_) {
        readyDone_ = true;
        return Action::READY;
    }
    // ...and if no such sample ever comes, silence is not an answer.
    if (!readyDone_ && !noDataDone_ && nowMs - lockSeenMs_ >= NO_DATA_GRACE_MS) {
        noDataDone_ = true;
        return Action::NO_DATA;
    }
    return Action::NONE;
}

size_t Announcer::line(Action action, const char* identity, char* out,
                       size_t cap) const {
    if (!out || cap == 0 || !identity) return 0;
    if (action == Action::NONE) return 0;

    char hp[16];
    numfmt::fixed(hp, sizeof(hp), static_cast<double>(lastBar_) * 1000.0, 1);

    const int w = (action == Action::READY)
        ? std::snprintf(out, cap, "depth: %s ready, %s hPa\n", identity, hp)
        : std::snprintf(out, cap,
                        "depth: ** %s identified, no sane pressure within %u s "
                        "(reads %s hPa) - depth will not be used\n",
                        identity,
                        static_cast<unsigned>(NO_DATA_GRACE_MS / 1000u), hp);
    // snprintf returns the length it WANTED: >= cap means the line was cut, and
    // a half-printed boot line is a line a reader (or a bench regex) misreads.
    if (w <= 0 || static_cast<size_t>(w) >= cap) {
        out[0] = '\0';
        return 0;
    }
    return static_cast<size_t>(w);
}

}  // namespace press
