#pragma once
#include <cstddef>
#include <cstdint>

// ============================================================================
// stackwatch — free stack bytes per task, so overflow is visible before it is fatal
//
// Why: on 2026-11-23 the uart2 task's stack canary fired on the first `GET T`
// from a real handset (cause and fix in util/numfmt.h). Nothing reported it
// first: no host test can see a target stack, and the bench only learned when
// the controller rebooted into SAFE mode. FreeRTOS already measures per-task
// stack headroom with a fill pattern; this is the "tell someone" half.
//
// Each task samples itself with uxTaskGetStackHighWaterMark(NULL) — the
// machine-wide uxTaskGetSystemState() needs configUSE_TRACE_FACILITY, which this
// build does not enable, and it would also drag a ~768-byte snapshot buffer into
// a 4096-byte task. Sampling is self-throttled to every kSampleMs.
//
// ESP-IDF reports the watermark in BYTES (vanilla FreeRTOS reports words).
// ============================================================================
namespace stackwatch {

enum Slot : uint8_t {
    SLOT_SENSOR = 0,
    SLOT_CONTROL,
    SLOT_HUD,
    SLOT_SUPERVISOR,
    SLOT_UART2,
    SLOT_LOOP,
    SLOT_COUNT
};

// Called from inside a task's own loop. Cheap and self-throttled; the value is
// only refreshed every kSampleMs, so calling it on every iteration is fine.
void sample(Slot slot);

// Smallest headroom among the tasks that have sampled. Returns 0 when nothing
// has sampled yet (so a missing number reads as "unknown", never as "plenty").
// `outSlot` receives which task it was, for the warning line.
uint32_t worstFree(uint8_t* outSlot = nullptr);

// The pure half of that, so it can be tested on the host: minimum of the
// non-zero entries (0 = not sampled), with the index of the winner.
uint32_t worstOf(const uint32_t* freeBytes, size_t n, size_t* outSlot);

const char* name(uint8_t slot);

constexpr uint32_t kSampleMs = 5000;
constexpr uint32_t kWarnFreeBytes = 512;   // shout below this

} // namespace stackwatch
