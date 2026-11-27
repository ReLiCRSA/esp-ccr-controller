// ============================================================================
// wdtfeed.h — feeding the task watchdog from a long loop that IS making
// progress, so progress is not mistaken for a stall (review M-1).
//
// The task watchdog answers one question: is this task still moving? It cannot
// tell a loop that never finishes from a loop with a great deal to do, and a
// dive-log export is the second kind: the store streams the whole dive, row by
// row, inside ONE handleLine() call, into whatever sink asked for it. On the
// handset that sink is Serial2.print at 115200 — about 87 µs a byte, with no
// yield inside it — and the uart2 task feeds the watchdog once per pump, which
// during an export is never. That task is registered (uart2_task.cpp), and on
// this build a registered task blowing its watchdog budget does not warn, it
// panics, and the panic reason forces the next boot into SAFE mode. So a
// diver's read-only `LOG EXPORT` of a long dive could reset his own
// controller mid-dive: the answer to a question was treated as a hang.
//
// The rule that keeps the watchdog honest anyway: feed AFTER a unit of work
// has completed, never before it. A feed that follows a row which actually
// reached the wire is evidence. A feed at the top of the loop is an excuse, and
// it would keep a genuinely wedged UART (blocked inside the sink, nothing
// draining) alive forever — which is exactly the hang uart2_task.cpp exists to
// catch. Interval by rows, not by time: the loop has no clock to read and the
// rows are the work.
//
// Off-target there is no watchdog to feed, so the call compiles to nothing —
// the same `#if defined(ARDUINO)` guard util/stackwatch.cpp uses. On-target,
// an unregistered caller gets ESP_ERR_NOT_FOUND and nothing else, which is what
// makes this safe from the web transport too: async_tcp is deliberately never
// registered with the watchdog (PLAN 9 — a wedged web server must never reset a
// diving controller) and this helper must not become a way to register or feed
// a task that policy keeps out.
// ============================================================================
#pragma once

#if defined(ARDUINO)
#include <esp_task_wdt.h>
#endif

namespace util {

/// Units of work (rows, for the export) between feeds. 16 is a sizing choice,
/// not a measured one: a snapshot CSV row is ~110 bytes, ~10 ms down a 115200
/// handset link, so an un-fed stretch of an export is ~0.15 s — comfortably
/// inside the 3 s window this build's task watchdog panics at (main.cpp's
/// esp_task_wdt_init(3, true) raises that timeout, and a TASK_WDT reason forces
/// SAFE at the next boot — see the toolchain evidence at that call site), while
/// a whole row's worth of work between feeds keeps the bookkeeping off the
/// per-byte path. Keep it a power of two: the feed test below is a mask, not a
/// division.
constexpr unsigned kWdtFeedEvery = 16u;

/// Is a feed due after this many COMPLETED units of work? Called with the
/// counter already incremented, so the first feed lands on unit 16 and they
/// repeat every 16 after that; the gap between feeds is therefore never longer
/// than kWdtFeedEvery units of work, whatever the total is.
inline bool wdtFeedDue(unsigned completed) {
    return (completed & (kWdtFeedEvery - 1u)) == 0u;
}

/// Feed the watchdog of the calling task. Only ever correct from the task that
/// owns the work: a watchdog feed from another task would be the one thing this
/// subsystem must never do — it would hide someone else's stall.
inline void wdtFeed() {
#if defined(ARDUINO)
    // On behalf of the calling task (no handle to pass on this framework — the
    // same call uart2_task.cpp makes once per pump). An unregistered caller gets
    // ESP_ERR_NOT_FOUND, an uninitialised watchdog ESP_ERR_INVALID_STATE, and
    // nothing else happens, which is what makes this safe from the web
    // transport's task (see the header comment for who that caller is on
    // purpose). Ignoring the code is deliberate: a task that is not registered
    // has nothing to be wrong about.
    (void)esp_task_wdt_reset();
#endif
}

} // namespace util
