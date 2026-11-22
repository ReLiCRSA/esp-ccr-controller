// ============================================================================
// cellhist.h — rolling per-cell ppO2 histograms (what the web histogram page
// is for: a cell quietly going bad, or foul air, shows as a *shape* long
// before a single reading crosses an alarm).
//
// Fixed memory, no heap, no clocks: ControlTask samples each fitted, fresh
// cell once per control tick (10 Hz real rate). Samples land in a ring of
// kWindows windows of kWindowS seconds, so the visible horizon is always
// "the last kWindows*WindowS seconds" — a ten-minute shape, not a forever
// average that dilutes the incident you actually wanted to see.
//
// fold()/formatLines() are pure and host-tested; sample() is the only writer
// and runs on the control thread — same single-writer discipline as the
// chain itself (A5e), so no locks here.
// ============================================================================
#pragma once

#include <cstdint>
#include <cstring>

namespace cellhist {

constexpr uint8_t kBins = 38;        // 0.00 .. 1.90 bar
constexpr float kBinWidth = 0.05f;   // bar per bin
constexpr float kMaxBar = 1.90f;
constexpr uint8_t kWindows = 10;     // ring depth
constexpr uint32_t kWindowS = 60;    // window length -> 10 min horizon

class Hist {
public:
    /// One sample, calibrated bar, on the control thread. Negative/NaN never
    /// enters a bin (a NaN must not poison the shape — it fails >=0).
    void sample(uint8_t cell, float bar, uint32_t nowMs);

    /// Sum the still-in-horizon windows into out[kBins] (caller zeroes
    /// nothing; fold writes every entry). Returns the total folded count.
    uint32_t fold(uint8_t cell, uint16_t out[kBins]) const;

    /// Age of the oldest counted sample, rounded up to whole seconds; 0 =
    /// nothing inside the horizon. Lets the page say "last 9.4 min" honestly.
    uint32_t horizonAgeS(uint32_t nowMs) const;

    /// Lifetime sample count (never decays) — the page uses it to tell
    /// "no data yet" from "a gap".
    uint32_t samples(uint8_t cell) const { return total_[cell]; }

    void clear();

private:
    struct Slot {
        uint16_t bins[kBins] = {};
        uint32_t id = 0;  // window id + 1 this slot currently holds (0 = empty)
    };
    // The ={} initializers are load-bearing: a Hist allocated over the memory
    // of a dead one (stack reuse — the test suite caught exactly this) must
    // start empty, or the first histogram of the boot is a ghost.
    Slot slots_[3][kWindows] = {};
    uint32_t total_[3] = {0, 0, 0};
    int64_t window_ = -1;  // current global window id (all cells advance together)
};

/// One line per cell: HIST,<cell>,<windowS>,<windows>,<binWidthX100>,
/// <ageS>,<foldedCount>,<c0>,...,<c37>\n. Returns chars written, 0 if `cap`
/// was too small (nothing partial is emitted — a truncated histogram reads
/// as a real shape, which is worse than no shape).
size_t formatLines(const Hist& h, uint32_t nowMs, char* out, size_t cap);

}  // namespace cellhist
