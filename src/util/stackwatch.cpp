// ============================================================================
// stackwatch.cpp — see stackwatch.h
// ============================================================================
#include "util/stackwatch.h"

#include <cstddef>

#if defined(ARDUINO)
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#endif

namespace stackwatch {

namespace {

// 0 means "not sampled yet" and is skipped by worstOf(). A task cannot report a
// genuine 0: by then the stack canary has already fired (that is how the uart2
// overflow was caught at all).
volatile uint32_t g_free[SLOT_COUNT] = {0};
#if defined(ARDUINO)
volatile uint32_t g_last[SLOT_COUNT] = {0};
#endif

const char* const kNames[SLOT_COUNT] = {
    "sensor", "control", "hud", "supervisor", "uart2", "loop"};

} // namespace

uint32_t worstOf(const uint32_t* freeBytes, size_t n, size_t* outSlot) {
    uint32_t worst = 0;
    size_t at = 0;
    for (size_t i = 0; i < n; ++i) {
        if (freeBytes[i] == 0) continue;                 // not sampled yet
        if (worst == 0 || freeBytes[i] < worst) { worst = freeBytes[i]; at = i; }
    }
    if (outSlot) *outSlot = at;
    return worst;
}

uint32_t worstFree(uint8_t* outSlot) {
    uint32_t copy[SLOT_COUNT];
    for (size_t i = 0; i < SLOT_COUNT; ++i) copy[i] = g_free[i];
    size_t at = 0;
    const uint32_t w = worstOf(copy, SLOT_COUNT, &at);
    if (outSlot) *outSlot = static_cast<uint8_t>(at);
    return w;
}

const char* name(uint8_t slot) {
    return slot < SLOT_COUNT ? kNames[slot] : "?";
}

void sample(Slot slot) {
    if (slot >= SLOT_COUNT) return;
#if defined(ARDUINO)
    const uint32_t now = millis();
    if (g_last[slot] != 0 && now - g_last[slot] < kSampleMs) return;
    g_last[slot] = now;
    // ESP-IDF: bytes, not words. Reads the calling task's own watermark.
    g_free[slot] = static_cast<uint32_t>(uxTaskGetStackHighWaterMark(NULL));
#endif
}

} // namespace stackwatch
