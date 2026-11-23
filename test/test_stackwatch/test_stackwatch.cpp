// ============================================================================
// test_stackwatch — the decision half of the stack-headroom report.
//
// The sampling itself needs FreeRTOS, so it is one #ifdef deep in stackwatch.cpp.
// What can be tested on the host, and what actually decides whether the bench
// sees a warning, is the reduction: which task is tightest, and what "we know
// nothing yet" must never be mistaken for "there is plenty of room".
// ============================================================================
#include <unity.h>

#include <cstddef>

#include "util/stackwatch.h"

void setUp() {}
void tearDown() {}

void test_unsampled_is_not_plenty_of_room() {
    const uint32_t none[stackwatch::SLOT_COUNT] = {0, 0, 0, 0, 0, 0};
    size_t at = 99;
    // Everything reads 0 until each task has sampled: the answer must be 0
    // ("unknown"), never a large number that would look like a healthy margin.
    TEST_ASSERT_EQUAL_UINT32(0, stackwatch::worstOf(none, stackwatch::SLOT_COUNT, &at));
}

void test_picks_the_tightest_task() {
    const uint32_t free[stackwatch::SLOT_COUNT] = {
        1400, 300, 900, 2000, 120, 800};
    size_t at = 99;
    TEST_ASSERT_EQUAL_UINT32(120, stackwatch::worstOf(free, stackwatch::SLOT_COUNT, &at));
    TEST_ASSERT_EQUAL_UINT8(stackwatch::SLOT_UART2, at);
}

void test_ignores_tasks_that_have_not_sampled_yet() {
    // A task that has not reported must not drag the minimum to 0: that would
    // shout "stack low" on every boot before the tasks have had a turn.
    const uint32_t partial[stackwatch::SLOT_COUNT] = {900, 0, 0, 0, 0, 0};
    size_t at = 99;
    TEST_ASSERT_EQUAL_UINT32(900, stackwatch::worstOf(partial, stackwatch::SLOT_COUNT, &at));
    TEST_ASSERT_EQUAL_UINT8(stackwatch::SLOT_SENSOR, at);
}

void test_the_minimum_hides_the_task_you_care_about() {
    // Why HB carries u2stk= as well as stk=. The supervisor runs on 3072 bytes
    // and its headroom is flat no matter what a client asks for; the uart2 task
    // has 6144 and is where the command path lives. Reporting only the minimum
    // means the number on the bench belongs to a task that cannot regress, and
    // the one that overflowed on 2026-11-23 is invisible behind it.
    const uint32_t both[stackwatch::SLOT_COUNT] = {
        1600, 1900, 1700, 2132, 900, 3000};
    size_t at = 99;
    const uint32_t worst = stackwatch::worstOf(both, stackwatch::SLOT_COUNT, &at);
    TEST_ASSERT_EQUAL_UINT32(900, worst);
    TEST_ASSERT_EQUAL_UINT8(stackwatch::SLOT_UART2, at);

    // Now the reported situation on the board: uart2 has room, so the minimum
    // is the supervisor's, and a uart2 reading of 900 would never surface.
    const uint32_t healthy[stackwatch::SLOT_COUNT] = {
        1600, 1900, 1700, 2132, 4500, 3000};
    TEST_ASSERT_EQUAL_UINT32(1600, stackwatch::worstOf(healthy, stackwatch::SLOT_COUNT, &at));
    TEST_ASSERT_EQUAL_UINT8(stackwatch::SLOT_SENSOR, at);
    // ...which is exactly why freeFor(SLOT_UART2) needs its own field in HB.
}

void test_names_are_defined_for_every_slot() {
    for (int i = 0; i < stackwatch::SLOT_COUNT; ++i) {
        const char* n = stackwatch::name(static_cast<uint8_t>(i));
        TEST_ASSERT_NOT_NULL(n);
        TEST_ASSERT_NOT_EQUAL(0, n[0]);
    }
    TEST_ASSERT_NOT_EQUAL(0, stackwatch::name(200)[0]);   // out of range: "?"
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_unsampled_is_not_plenty_of_room);
    RUN_TEST(test_picks_the_tightest_task);
    RUN_TEST(test_ignores_tasks_that_have_not_sampled_yet);
    RUN_TEST(test_the_minimum_hides_the_task_you_care_about);
    RUN_TEST(test_names_are_defined_for_every_slot);
    return UNITY_END();
}
