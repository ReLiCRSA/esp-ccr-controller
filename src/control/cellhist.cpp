// ============================================================================
// cellhist.cpp — see cellhist.h.
// ============================================================================
#include "cellhist.h"

#include <cstdio>

#include "util/numfmt.h"            // scaled(): one rounding rule, no hand-rolled +0.5f

namespace cellhist {

namespace {
inline size_t slotOf(int64_t winId) { return static_cast<size_t>(winId % kWindows); }
}

void Hist::sample(uint8_t cell, float bar, uint32_t nowMs) {
    if (cell >= 3) return;
    if (!(bar >= 0.0f)) return;  // rejects NaN too — never poison the shape

    const int64_t win = static_cast<int64_t>(nowMs / 1000u / kWindowS);
    const size_t slot = slotOf(win);
    Slot& s = slots_[cell][slot];
    const uint32_t id = static_cast<uint32_t>(win + 1);
    if (s.id != id) {
        // first write of this window into this slot: the occupant is at least
        // one full horizon old, and stale bins are not history, they are lies
        std::memset(s.bins, 0, sizeof(s.bins));
        s.id = id;
    }

    int bin = static_cast<int>(bar / kBinWidth);
    if (bin >= kBins) bin = kBins - 1;  // >= 1.90 stacks in the last bin
    if (s.bins[bin] < 0xFFFF) s.bins[bin]++;
    total_[cell]++;
    if (win > window_) window_ = win;
}

uint32_t Hist::fold(uint8_t cell, uint16_t out[kBins]) const {
    uint32_t sum = 0;
    for (uint8_t b = 0; b < kBins; b++) out[b] = 0;
    if (cell >= 3 || window_ < 0) return 0;
    for (uint8_t w = 0; w < kWindows; w++) {
        const int64_t win = window_ - w;
        if (win < 0) break;
        const Slot& s = slots_[cell][slotOf(win)];
        if (s.id != static_cast<uint32_t>(win + 1)) continue;  // stale occupant
        for (uint8_t b = 0; b < kBins; b++) out[b] += s.bins[b];
    }
    for (uint8_t b = 0; b < kBins; b++) sum += out[b];
    return sum;
}

uint32_t Hist::horizonAgeS(uint32_t nowMs) const {
    // Age of the oldest retained window's START, measured against nowMs: with
    // a healthy 10 Hz feed this is ~600 s; when sampling went quiet it keeps
    // growing, so the page can say "last data X s ago" instead of implying
    // the shape is live.
    if (window_ < 0) return 0;
    uint8_t oldest = 0;
    for (uint8_t c = 0; c < 3; c++) {
        for (uint8_t w = 0; w < kWindows; w++) {
            const int64_t win = window_ - w;
            if (win < 0) break;
            if (slots_[c][slotOf(win)].id == static_cast<uint32_t>(win + 1)) {
                if (w > oldest) oldest = w;
                break;
            }
        }
    }
    const int64_t oldestStartS = (window_ - oldest) * static_cast<int64_t>(kWindowS);
    const int64_t nowS = nowMs / 1000u;
    return nowS > oldestStartS ? static_cast<uint32_t>(nowS - oldestStartS) : 0;
}

void Hist::clear() {
    for (uint8_t c = 0; c < 3; c++) {
        total_[c] = 0;
        for (uint8_t w = 0; w < kWindows; w++) {
            std::memset(slots_[c][w].bins, 0, sizeof(slots_[c][w].bins));
            slots_[c][w].id = 0;
        }
    }
    window_ = -1;
}

size_t formatLines(const Hist& h, uint32_t nowMs, char* out, size_t cap) {
    uint16_t bins[kBins];
    int p = 0;
    for (uint8_t c = 0; c < 3; c++) {
        const uint32_t n = h.fold(c, bins);
        // one line per cell, computed fully before the first byte is
        // committed for that cell
        char line[32 + kBins * 6];
        int q = std::snprintf(line, sizeof(line),
                              "HIST,%u,%u,%u,%u,%u,%u,",
                              static_cast<unsigned>(c), static_cast<unsigned>(kWindowS),
                              static_cast<unsigned>(kWindows),
                              static_cast<unsigned>(numfmt::scaled(kBinWidth, 2)),
                              static_cast<unsigned>(h.horizonAgeS(nowMs)), n);
        if (q <= 0 || static_cast<size_t>(q) >= sizeof(line)) return 0;
        for (uint8_t b = 0; b < kBins; b++) {
            const int m = std::snprintf(line + q, sizeof(line) - q, "%u%s",
                                        static_cast<unsigned>(bins[b]),
                                        b + 1 == kBins ? "\n" : ",");
            if (m <= 0 || static_cast<size_t>(q + m) >= sizeof(line)) return 0;
            q += m;
        }
        if (static_cast<size_t>(p + q) >= cap) return 0;  // whole lines or nothing
        std::memcpy(out + p, line, static_cast<size_t>(q));
        p += q;
    }
    return static_cast<size_t>(p);
}

}  // namespace cellhist
