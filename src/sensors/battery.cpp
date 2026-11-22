#include "sensors/battery.h"
#include "config/pins.h"
#include "config/defaults.h"

// Small insertion sort median of 8 — no allocation, runs in microseconds.
static float median8(float a[8]) {
    for (int i = 1; i < 8; ++i) {
        const float key = a[i];
        int j = i - 1;
        while (j >= 0 && a[j] > key) { a[j + 1] = a[j]; --j; }
        a[j + 1] = key;
    }
    return (a[3] + a[4]) * 0.5f;
}

void Battery::update() {
    float s[8];
    for (int i = 0; i < 8; ++i) {
        s[i] = analogReadMilliVolts(PIN_VBAT_SENSE) * 1e-3f;
    }
    float v = median8(s) * VBAT_DIVIDER_RATIO * VBAT_CAL;

    if (!_initialized) {
        _volts = v;
        _initialized = true;
    } else {
        _volts = _volts * 0.7f + v * 0.3f; // EMA
    }
}
