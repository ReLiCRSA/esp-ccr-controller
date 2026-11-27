// ============================================================================
// test_warmup — the cell-polarisation verdict must survive a 49.7-day board.
//
// CAL AIR refuses to calibrate until the cells have polarised (120 s), and
// every version of that test written before util::WarmupGate compared the boot
// epoch against the budget using `millis()`, an unsigned 32-bit clock that
// wraps. The cases below are the actual failure table of the two previous
// forms: the original `now < PREDIVE_WARMUP_MS`, and batch A's signed
// difference, which a test — this file — showed was ALSO wrong, in more
// places than the one it fixed. The gate latches the verdict while the clock
// is honest and never reads it again; that is what "monotone fact" means, and
// this suite pins the monotonicity, the countdown, and the fact that
// main.cpp uses the gate instead of hand-rolling an epoch comparison.
// ============================================================================
#include <unity.h>

#include <cstdio>
#include <cstring>
#include <string>

#include "util/warmup.h"

using util::WarmupGate;

void setUp() {}
void tearDown() {}

namespace {

constexpr uint32_t kBudget = 120000u;          // PREDIVE_WARMUP_MS as shipped
constexpr uint32_t kWrap = 0xFFFFFFFFu;        // millis() rolls over past this

std::string readFile(const char* path) {
    FILE* f = std::fopen(path, "rb");
    if (!f) return "";
    std::string out;
    char buf[4096];
    size_t n = 0;
    while ((n = std::fread(buf, 1, sizeof(buf), f)) > 0) out.append(buf, n);
    std::fclose(f);
    return out;
}

std::string stripComments(const std::string& src) {
    std::string out;
    out.reserve(src.size());
    bool inLine = false, inBlock = false;
    for (size_t i = 0; i < src.size(); ++i) {
        const char c = src[i];
        const char nx = i + 1 < src.size() ? src[i + 1] : '\0';
        if (inLine) {
            if (c == '\n') { inLine = false; out += c; }
            continue;
        }
        if (inBlock) {
            if (c == '*' && nx == '/') { inBlock = false; ++i; }
            continue;
        }
        if (c == '/' && nx == '/') { inLine = true; ++i; continue; }
        if (c == '/' && nx == '*') { inBlock = true; ++i; continue; }
        out += c;
    }
    return out;
}

// Run the gate the way ControlTask does: update every cycle. The loops in the
// cases below are that cadence; there is no separate runner on purpose.

}  // namespace

// ---- the honest window: everything must be exact BEFORE the wrap ----

static void test_countdown_is_exact_during_warmup() {
    WarmupGate g;
    for (uint32_t t = 0; t <= kBudget; t += 1000u) {
        g.update(kBudget, t);
        if (t < kBudget) {
            TEST_ASSERT_FALSE_MESSAGE(g.elapsed(), "latched before the budget");
            TEST_ASSERT_EQUAL_UINT32_MESSAGE(kBudget - t, g.remainingMs(kBudget, t),
                                             "wrong countdown");
        } else {
            TEST_ASSERT_TRUE_MESSAGE(g.elapsed(), "not latched at the budget");
            TEST_ASSERT_EQUAL_UINT32(0u, g.remainingMs(kBudget, t));
        }
    }
    // Seconds for the wire field: exact, rounded down, never negative.
    TEST_ASSERT_EQUAL_UINT32(120u, WarmupGate{}.remainingSeconds(kBudget, 0u));
    TEST_ASSERT_EQUAL_UINT32(0u, WarmupGate{}.remainingSeconds(kBudget, kBudget));
}

// ---- the failure table of the two previous forms ----

static void test_old_forms_fail_the_wrap_table_the_gate_passes() {
    // The truth table. `up` is real uptime; millis() shows up mod 2^32.
    struct { uint64_t upMs; bool wantElapsed; } cases[] = {
        {          0ull, false },
        {     119000ull, false },
        {     120000ull, true  },
        {   40000000ull, true  },                  // 11 h: mid-day dive
        { 4000000000ull, true  },                  // 46.3 d, before the wrap
        { 4294967295ull, true  },                  // the last ms before the wrap
        { 4294967296ull, true  },                  // the wrap itself
        { 4295027296ull, true  },                  // wrap + 60 s: a CAL AIR now
        { 9000000000ull, true  },                  // two wraps in
    };

    for (const auto& c : cases) {
        const uint32_t now = static_cast<uint32_t>(c.upMs & 0xFFFFFFFFull);

        // ControlTask runs the gate every cycle from boot to this moment —
        // that is what a LATCH buys, and the loop below is the honest way to
        // demonstrate it rather than asserting the conclusion.
        WarmupGate g;
        for (uint64_t t = 0; t <= c.upMs; t += 2ull)  // CONTROL_TASK_PERIOD_MS
            g.update(kBudget, static_cast<uint32_t>(t & 0xFFFFFFFFull));
        if (c.wantElapsed)
            TEST_ASSERT_TRUE_MESSAGE(g.elapsed(), "gate wrong at this uptime");
        else
            TEST_ASSERT_FALSE_MESSAGE(g.elapsed(), "gate wrong at this uptime");

        // The original form: true again for the first 120 s after the wrap.
        const bool oldSaysElapsed = !(now < kBudget);
        // Batch A's "wrap-safe" fix: signed difference of two boot epochs.
        const bool signedSaysElapsed =
            static_cast<int32_t>(kBudget - now) <= 0;
        // Record what each form gets wrong — the assertions here are about
        // the TEST knowing the bug, so a future "simplification" back to an
        // epoch comparison cannot pass this suite by accident.
        if (c.upMs == 4295027296ull) {   // wrap + 60 s
            TEST_ASSERT_FALSE_MESSAGE(oldSaysElapsed,
                "expected the original form to REFUSE here (that is the bug)");
            TEST_ASSERT_FALSE_MESSAGE(signedSaysElapsed,
                "expected the signed form to REFUSE here too (that is the bug)");
        }
        if (c.upMs == 4294967295ull) {   // last ms before the wrap
            TEST_ASSERT_TRUE_MESSAGE(oldSaysElapsed, "original is fine here");
            TEST_ASSERT_FALSE_MESSAGE(signedSaysElapsed,
                "the signed form wraps POSITIVE past ~24.9 days and refuses a "
                "machine 49.7 days into warm-up: batch A's fix was not a fix");
        }
        if (c.upMs == 4000000000ull) {   // 46.3 days, both epochs far apart
            TEST_ASSERT_FALSE_MESSAGE(signedSaysElapsed,
                "signed form also refuses at 46 days: |120000-now| exceeds 2^31");
        }
    }
}

// ---- the latch is one-way: the clock going backwards cannot undo it ----

static void test_latch_never_unlatches() {
    WarmupGate g;
    g.update(kBudget, 120000u);
    TEST_ASSERT_TRUE(g.elapsed());
    // After the wrap millis() shows small values again. The cycle still calls
    // update(); it must not resurrect the refusal.
    for (uint32_t t = 0; t < kBudget; t += 500u) {
        g.update(kBudget, t);
        TEST_ASSERT_TRUE_MESSAGE(g.elapsed(), "the gate unlatched behind the clock");
    }
    // And the countdown reports zero once latched, whatever the clock says:
    // a refusal path can never print a fake countdown for a warm machine.
    TEST_ASSERT_EQUAL_UINT32(0u, g.remainingMs(kBudget, 0u));
    TEST_ASSERT_EQUAL_UINT32(0u, g.remainingSeconds(kBudget, 59999u));
}

// ---- the refusal path itself never underflows ----

static void test_remaining_never_borrows() {
    WarmupGate g;
    // Not latched, so remaining() is only called while now < budget — but pin
    // the boundary anyway: the call right at the budget and just under.
    TEST_ASSERT_EQUAL_UINT32(1u, g.remainingMs(kBudget, kBudget - 1u));
    TEST_ASSERT_EQUAL_UINT32(0u, g.remainingMs(kBudget, kBudget));
    TEST_ASSERT_EQUAL_UINT32(0u, g.remainingMs(kBudget, kWrap));  // wrapped clock
}

// ---- the firmware must actually use the gate ----

static void test_main_uses_the_gate_not_a_clock_subtraction() {
    const std::string mainCpp = stripComments(readFile("src/main.cpp"));
    TEST_ASSERT_TRUE_MESSAGE(!mainCpp.empty(), "cannot read src/main.cpp");
    // The gate is updated per cycle AND consulted on the request path.
    TEST_ASSERT_NOT_NULL_MESSAGE(std::strstr(mainCpp.c_str(), "g_warmupGate.update("),
        "ControlTask no longer updates the warm-up gate every cycle — the latch "
        "only closes while the clock is honest if the clock is actually read");
    TEST_ASSERT_NOT_NULL_MESSAGE(std::strstr(mainCpp.c_str(), "g_warmupGate.elapsed()"),
        "the CAL AIR path no longer reads the gate");
    // The two banned epoch forms, comments exempt: prose has to be able to
    // name what the old bugs were.
    TEST_ASSERT_NULL_MESSAGE(std::strstr(mainCpp.c_str(), "PREDIVE_WARMUP_MS - now"),
        "main.cpp hand-rolls the warm-up subtraction again (util/warmup.h)");
    TEST_ASSERT_NULL_MESSAGE(std::strstr(mainCpp.c_str(), "now < PREDIVE_WARMUP_MS"),
        "main.cpp compares the boot epoch against the budget again (util/warmup.h)");
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_countdown_is_exact_during_warmup);
    RUN_TEST(test_old_forms_fail_the_wrap_table_the_gate_passes);
    RUN_TEST(test_latch_never_unlatches);
    RUN_TEST(test_remaining_never_borrows);
    RUN_TEST(test_main_uses_the_gate_not_a_clock_subtraction);
    UNITY_END();
    return 0;
}
