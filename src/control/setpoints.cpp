#include "setpoints.h"

namespace sp {

Result Engine::update(float depthM) {
    const float h = cfg_.hysteresisM;
    uint8_t want = band_;

    // Descending: cross a boundary with margin. Ascending: leave it only
    // after h metres back. This keeps band edges identical to PLAN.md 5.
    switch (band_) {
        case 0: if (depthM > cfg_.inWaterM + h) want = 1; break;
        case 1:
            if (depthM > cfg_.deep1M + h) want = 2;
            else if (depthM < cfg_.inWaterM - h) want = 0;
            break;
        case 2:
            if (depthM > cfg_.deep2M + h) want = 3;
            else if (depthM < cfg_.deep1M - h) want = 1;
            break;
        case 3: if (depthM < cfg_.deep2M - h) want = 2; break;
    }

    Result r;
    r.changed = (want != band_);
    band_ = want;
    r.index = band_;
    r.nominal = cfg_.sp[band_];
    if (clamp_ > 0.0f && r.nominal > clamp_) {
        r.value = clamp_;
        r.clamped = true;
    } else {
        r.value = r.nominal;
        r.clamped = false;
    }
    return r;
}

} // namespace sp
