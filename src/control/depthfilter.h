// ============================================================================
// depthfilter.h — depth smoothing shared by setpoint bands and CellProof.
//
// Median-of-N first (a single bad pressure sample must not switch a setpoint
// band or fire a cell test), then a short boxcar (damping without the lag of
// an EMA). A filter matters here for safety, not cosmetics: SP bands switch
// on absolute thresholds, so a spike at 29.9 m would silently change the loop
// setpoint by 0.3 bar.
//
// The filter also knows whether a sample was MEASURED at all (update()'s
// `valid` argument) — see the comment there. "Depth unknown" and "depth 0 m"
// are different facts and only one of them is survivable.
//
// Header-only: identical code in firmware, unit tests and the simulator.
// ============================================================================
#pragma once

#include <cstddef>
#include <cstdint>

namespace depth {

template <uint8_t N = 5, uint8_t AVG = 4>
class Filter {
public:
    static_assert(N >= 1 && N <= 9, "median window 1..9");
    static_assert(AVG >= 1 && AVG <= 16, "boxcar window 1..16");

    /// One raw sample (metres), typically from the pressure driver, plus whether
    /// the driver actually MEASURED it.
    ///
    /// An invalid sample is not a zero-metre sample: it is no sample. It never
    /// enters either window, and the last filtered value stands — the depth
    /// becomes UNKNOWN rather than 0 m.
    ///
    /// This is not a nicety. Both depth chips share one I2C port by design
    /// (include/config/pins.h), so "both dead" is a SINGLE failure mode, and the
    /// firmware used to deliver it here as 0.0 m. Median-of-5 plus boxcar-of-4
    /// converged on that in about half a second, and the setpoint engine —
    /// which has no idea where its depth came from — walked the loop from the
    /// deep band (1.30 bar) down to the 0.40 bar surface rung while the diver
    /// was at depth and the handset printed 0.0 m.
    ///
    /// Holding the last measured depth freezes the bands instead of walking
    /// them. The error it can make is holding the deep band while the diver
    /// ascends blind, which is the direction already fenced by the HIGH alarm,
    /// the injector's highWall/cutoff and the duty cap. The error it replaces —
    /// driving the setpoint toward the surface — has no wall underneath it: a
    /// setpoint of 0.40 bar at 30 m starves the loop quietly, and no ppO2 alarm
    /// below the LOW threshold can tell the diver which rung the machine chose.
    float update(float rawM, bool valid) {
        if (!valid) return last_;  // hold-last-good: no window is touched

        buf_[head_] = rawM;
        head_ = (head_ + 1) % N;
        if (count_ < N) count_++;

        // median of the samples held so far
        float tmp[N];
        for (uint8_t i = 0; i < count_; i++) tmp[i] = buf_[i];
        for (uint8_t i = 1; i < count_; i++) {  // insertion sort: N is tiny
            const float k = tmp[i];
            int j = i - 1;
            while (j >= 0 && tmp[j] > k) {
                tmp[j + 1] = tmp[j];
                j--;
            }
            tmp[j + 1] = k;
        }
        const float med = tmp[count_ / 2];

        hist_[hIdx_] = med;
        hIdx_ = (hIdx_ + 1) % AVG;
        if (hCount_ < AVG) hCount_++;
        float sum = 0.0f;
        for (uint8_t i = 0; i < hCount_; i++) sum += hist_[i];
        last_ = sum / hCount_;
        return last_;
    }

    /// The old call shape: a sample with no validity flag attached is believed.
    /// Callers that have a health flag to pass should pass it; this overload
    /// exists so the simulator and the unit tests that feed clean samples keep
    /// saying so in one argument instead of two.
    float update(float rawM) { return update(rawM, true); }

    float value() const { return last_; }
    /// Settled means N + AVG samples the filter actually took. An invalid
    /// burst therefore cannot make this true: the filter never claims to be
    /// settled on data it refused to look at.
    bool settled() const { return count_ == N && hCount_ == AVG; }
    /// Forgets the held depth too: this is for a dive restart or a depth source
    /// swap, where "we no longer know" is the honest state.
    void reset() {
        count_ = hCount_ = head_ = hIdx_ = 0;
        last_ = 0.0f;
    }

private:
    float buf_[N] = {};
    float hist_[AVG] = {};
    uint8_t head_ = 0, count_ = 0, hIdx_ = 0, hCount_ = 0;
    float last_ = 0.0f;
};

}  // namespace depth
