// ============================================================================
// wallclock.h — the controller's idea of what time it is (PLAN.md 10, dive log).
//
// There is no RTC chip on this board and no NTP through an access point with no
// uplink, so wall time is something a client at the surface hands the
// controller: `TIME SET <epoch>` from the wired handset, or the built-in page
// posting the phone's clock when it loads. What survives and what does not was
// MEASURED on the bench 2026-11-24, because the assumption is wrong in one
// corner: resets the firmware performs on itself (panic, watchdog, SAFE-mode
// ACK) keep RTC no-init memory, so the clock carries and keeps advancing;
// pulling EN is a CHIP_PU power-down on this board — the next boot reports
// `rst:0x1 (POWERON_RESET)` — and the carry is gone, exactly like a battery
// pull. The rule the firmware implements: the clock is set at the surface, any
// loss says `unset` out loud, and nothing is ever dated by a number from a
// previous life.
//
// Deliberately pure arithmetic, with the monotonic millisecond clock handed in
// by the caller (millis() on the board, the sim clock in tests) exactly like
// CommandRouter::setNowMs: the current time is `epoch + elapsed since it was
// set`, so there is no OS clock to keep in step, nothing that depends on a TZ
// or a locale, and the behaviour that a dive log depends on can be unit-tested
// on the host. On 2026-11-24 the dive epoch was a hard-coded 0, every dive on
// the partition tied on the FIFO's epoch ordering, and the eviction order was
// decided by a loop instead of by age (PLAN 16) — this module is what makes the
// epoch mean something.
//
// A dive started with the clock unset is still recorded, with epoch 0, and says
// so: an event row in the dive (EVT_CLOCK_UNSET) and `clock-unset` where the CSV
// export would otherwise print 1970-01-01. Refusing to dive because nobody set a
// clock is the alternative, and it is worse for the diver.
// ============================================================================
#pragma once

#include <cstdint>

namespace wallclock {

/// How the current time got here. CARRIED matters on the bench and to a diver:
/// it means "the number survived a reset, but nobody has confirmed it since".
enum Source : uint8_t { UNSET = 0, SET = 1, CARRIED = 2 };

/// A client can name any uint32, and a plausible-looking wrong epoch is worse
/// than no epoch: it puts a dive in 1970 or in 2200 and every reader of the log
/// believes it. So the accepted window is 2000-01-01 .. 2100-01-01, and
/// everything outside is refused with an ERR on the same socket it arrived on.
constexpr uint32_t MIN_EPOCH = 946684800u;     // 2000-01-01T00:00:00Z
constexpr uint32_t MAX_EPOCH = 4102444800u;    // 2100-01-01T00:00:00Z

/// Written into the ESP32's RTC no-init memory next to the epoch. That memory
/// survives the resets the firmware performs on itself and does not survive an
/// EN cycle or a battery pull, where it holds whatever the silicon left there:
/// the magic is what turns "probably garbage" into "refuse it, start with no
/// clock". A carry that is refused is a lost afternoon on the dive log, which
/// is recoverable; a carry that is believed and wrong dates every dive in the
/// file with a confident lie.
constexpr uint32_t CARRY_MAGIC = 0x43435254u;  // 'CCRT'

struct State {
    uint32_t epochSet = 0;   // wall time at the instant it was set, 0 = never
    uint32_t setAtMs = 0;    // monotonic millisecond clock at that instant
    uint8_t source = UNSET;
};

bool sane(uint32_t epoch);

/// The epoch held in RTC carry memory, or 0 when it cannot be trusted.
/// `stored` is what was found there, `epoch` what came with it.
uint32_t carryPick(uint32_t magic, uint32_t epoch);

/// Sets the clock. Returns false and changes nothing when the epoch is outside
/// the accepted window — a refusal must not leave the controller running on the
/// new value while telling the client it failed.
bool set(State& s, uint32_t epoch, uint32_t nowMs);

/// Current wall time, or 0 meaning "no clock". Elapsed time is measured on the
/// monotonic millisecond clock: unsigned subtraction, so it stays correct
/// across that clock's ~49-day wrap as long as the clock was set within the
/// last wrap, which is always true for a dive computer that is powered up for
/// a dive.
uint32_t epoch(const State& s, uint32_t nowMs);

/// Seconds since the clock was set, 0 when it never was. Reported so a bench
/// can see a clock going stale rather than guessing.
uint32_t ageS(const State& s, uint32_t nowMs);

}  // namespace wallclock
