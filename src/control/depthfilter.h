// ============================================================================
// depthfilter.h — depth smoothing shared by setpoint bands and CellProof.
//
// Median-of-N first (a single bad pressure sample must not switch a setpoint
// band or fire a cell test), then a short boxcar (damping without the lag of
// an EMA). A filter matters here for safety, not cosmetics: SP bands switch
// on absolute thresholds, so a spike at 29.9 m would silently change the loop
// setpoint by 0.3 bar.
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

    /// One raw sample (metres), typically from the pressure driver.
    float update(float rawM) {
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

    float value() const { return last_; }
    bool settled() const { return count_ == N && hCount_ == AVG; }
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
