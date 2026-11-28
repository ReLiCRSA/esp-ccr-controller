// ============================================================================
// dividersense.cpp — see dividersense.h.
// ============================================================================
#include "sensors/dividersense.h"

// Small insertion sort median of 8 — no allocation, runs in microseconds.
// Moved out of battery.cpp verbatim: same filter, now one copy for every
// divider input.
static float median8(const int in[8]) {
    int a[8];
    for (int i = 0; i < 8; ++i) a[i] = in[i];
    for (int i = 1; i < 8; ++i) {
        const int key = a[i];
        int j = i - 1;
        while (j >= 0 && a[j] > key) { a[j + 1] = a[j]; --j; }
        a[j + 1] = key;
    }
    return (a[3] + a[4]) * 0.5f;
}

bool DividerSense::addSample(int adcMilliVolts) {
    _buf[_n++] = adcMilliVolts;
    if (_n < 8) return false;
    _n = 0;

    const float v = median8(_buf) * 1e-3f * _ratio * _cal;

    if (!_initialized) {
        _volts = v;
        _initialized = true;
    } else {
        _volts = _volts * 0.7f + v * 0.3f;  // EMA
    }
    onProcessed();
    return true;
}
