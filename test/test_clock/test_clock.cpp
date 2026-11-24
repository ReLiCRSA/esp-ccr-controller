// ============================================================================
// test_clock — the controller's idea of what time it is. There is no RTC chip
// on this board and no NTP through an access point with no uplink, so wall time
// is handed over by a client at the surface and then advanced on the monotonic
// millisecond clock.
//
// What would go wrong without these cases: on 2026-11-24 the dive epoch was a
// hard-coded 0, every dive on the flash tied on the FIFO's epoch ordering, and
// the eviction order was decided by a loop — dives were destroyed in the wrong
// order and nothing anywhere reported it (PLAN 16). A clock that drifts, wraps,
// or resurrects a garbage value after a battery pull puts that defect back with
// a plausible date on it, which is worse than the honest absence of one.
// ============================================================================
#include <unity.h>

#include "comms/wallclock.h"

using namespace wallclock;

void setUp() {}
void tearDown() {}

namespace {

constexpr uint32_t k2026 = 1795509660u;   // 2026-11-24T08:41:00Z

}  // namespace

// "No clock" is a state, not the number 0 pretending to be January 1970.
void test_unset_clock_reports_no_time() {
    State s;
    TEST_ASSERT_EQUAL_UINT32(0u, epoch(s, 0u));
    TEST_ASSERT_EQUAL_UINT32(0u, epoch(s, 123456789u));
    TEST_ASSERT_EQUAL_UINT32(0u, ageS(s, 999u));
    TEST_ASSERT_EQUAL_UINT8(UNSET, s.source);
}

void test_set_clock_advances_on_the_monotonic_clock() {
    State s;
    TEST_ASSERT_TRUE(set(s, k2026, 10000u));
    TEST_ASSERT_EQUAL_UINT32(k2026, epoch(s, 10000u));
    TEST_ASSERT_EQUAL_UINT32(k2026, epoch(s, 10999u));    // whole seconds only
    TEST_ASSERT_EQUAL_UINT32(k2026 + 1u, epoch(s, 11000u));
    TEST_ASSERT_EQUAL_UINT32(k2026 + 3600u, epoch(s, 3610000u));
    TEST_ASSERT_EQUAL_UINT32(3600u, ageS(s, 3610000u));
    TEST_ASSERT_EQUAL_UINT8(SET, s.source);
}

// A monotonic millisecond clock wraps (millis() on the ESP32 wraps at ~49.7
// days). Unsigned subtraction keeps the elapsed time right through the wrap, so
// a clock set before a long surface run does not lurch when it comes round.
void test_elapsed_survives_the_monotonic_ms_wrap() {
    State s;
    const uint32_t justBeforeWrap = 0xFFFFFFF0u;
    TEST_ASSERT_TRUE(set(s, k2026, justBeforeWrap));
    TEST_ASSERT_EQUAL_UINT32(k2026, epoch(s, justBeforeWrap));
    TEST_ASSERT_EQUAL_UINT32(k2026, epoch(s, 0x10u));      // 32 ms later, wrapped
    TEST_ASSERT_EQUAL_UINT32(k2026, epoch(s, 0x20u));      // 48 ms: not a second yet
    TEST_ASSERT_EQUAL_UINT32(k2026 + 1u, epoch(s, justBeforeWrap + 1500u));
}

// Refusing a value must leave the clock exactly as it was: a client told
// `ERR epoch out of range` must then read back the time it had, not the one it
// was just told it had not set.
void test_out_of_range_is_refused_and_changes_nothing() {
    State s;
    TEST_ASSERT_TRUE(set(s, k2026, 1000u));
    const uint32_t before = epoch(s, 2000u);
    TEST_ASSERT_FALSE(set(s, 0u, 3000u));                 // "unset" is not a value
    TEST_ASSERT_FALSE(set(s, MIN_EPOCH - 1u, 3000u));
    TEST_ASSERT_FALSE(set(s, MAX_EPOCH + 1u, 3000u));
    TEST_ASSERT_FALSE(set(s, 0xFFFFFFFFu, 3000u));
    TEST_ASSERT_EQUAL_UINT32(before, epoch(s, 2000u));     // same instant, same answer
    TEST_ASSERT_EQUAL_UINT8(SET, s.source);
    TEST_ASSERT_TRUE(set(s, MIN_EPOCH, 5000u));           // the bounds themselves
    TEST_ASSERT_TRUE(set(s, MAX_EPOCH, 5000u));
}

// RTC no-init memory survives a reset and holds whatever the silicon left after
// a battery pull. Without the magic, garbage that happens to look like an epoch
// would date every dive wrong in silence; without the range check, so would a
// word that does not look like one.
void test_carry_needs_both_magic_and_range() {
    TEST_ASSERT_EQUAL_UINT32(k2026, carryPick(CARRY_MAGIC, k2026));
    TEST_ASSERT_EQUAL_UINT32(0u, carryPick(0u, k2026));              // cleared
    TEST_ASSERT_EQUAL_UINT32(0u, carryPick(0xDEADBEEFu, k2026));     // not ours
    TEST_ASSERT_EQUAL_UINT32(0u, carryPick(CARRY_MAGIC, 0u));
    TEST_ASSERT_EQUAL_UINT32(0u, carryPick(CARRY_MAGIC, 946684799u));
    TEST_ASSERT_EQUAL_UINT32(0u, carryPick(CARRY_MAGIC, 0xFFFFFFFFu));
    // 0xDEADBEEF is 2088-01-23: a range check on its own would accept it, so it
    // is the magic that rejects it. Asserting this keeps the two checks from
    // being "simplified" into one.
    TEST_ASSERT_TRUE(sane(0xDEADBEEFu));
}

void test_the_accepted_window_is_stated_in_years_not_numbers() {
    // 2000-01-01 .. 2100-01-01: wide enough for any real dive, narrow enough
    // that a wrong clock is refused on the spot instead of exported as a date.
    TEST_ASSERT_TRUE(sane(MIN_EPOCH));
    TEST_ASSERT_TRUE(sane(MAX_EPOCH));
    TEST_ASSERT_FALSE(sane(MIN_EPOCH - 1u));
    TEST_ASSERT_FALSE(sane(MAX_EPOCH + 1u));
    TEST_ASSERT_EQUAL_UINT32(946684800u, MIN_EPOCH);
    TEST_ASSERT_EQUAL_UINT32(4102444800u, MAX_EPOCH);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_unset_clock_reports_no_time);
    RUN_TEST(test_set_clock_advances_on_the_monotonic_clock);
    RUN_TEST(test_elapsed_survives_the_monotonic_ms_wrap);
    RUN_TEST(test_out_of_range_is_refused_and_changes_nothing);
    RUN_TEST(test_carry_needs_both_magic_and_range);
    RUN_TEST(test_the_accepted_window_is_stated_in_years_not_numbers);
    return UNITY_END();
}
