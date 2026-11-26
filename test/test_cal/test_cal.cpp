#include <unity.h>

#include <cstring>

#include "control/calibration.h"

using cal::Cal;
using cal::Config;
using cal::O2Input;
using cal::O2Output;
using cal::O2Sequencer;
using cal::O2State;
using cal::PulseTest;
using cal::PulseTestInput;
using cal::PulseTestOutput;
using cal::Solver;

void setUp() {}
void tearDown() {}

// raw scale for tests: 1 bar O2 == +50 raw units -> slope 0.02/bar unit... i.e.
// ppO2 = 0.21 + (raw - rawAir)/100/2.4? Keep it simple: slope = 0.0021/raw.
static const float kSlope = 0.0021f;

static Cal airCal(float rawAir) {
    Cal c;
    c.slope = kSlope; c.airBar = 0.21f; c.rawAir = rawAir; c.hasAir = true;
    return c;
}

void test_solver_accepts_good_pair() {
    Config cfg{};
    const Cal prev[3] = {airCal(100), airCal(101), airCal(99)};
    // true slope 0.0021 -> rise for 0.99-0.21=0.78 bar = 371 raw
    const float rawO2[3] = {100 + 371, 101 + 375, 99 + 368};
    Cal out[3];
    cal::O2Result res;
    Solver::solveO2(cfg, rawO2, prev, kSlope, out, res);
    for (int i = 0; i < 3; ++i) {
        TEST_ASSERT_EQUAL_UINT8(cal::CV_OK, res.verdict[i]);
        TEST_ASSERT_TRUE(out[i].hasO2);
        TEST_ASSERT_FLOAT_WITHIN(0.0002f, kSlope, out[i].slope);
    }
    // applied calibration round-trips: raw for 0.21 reads 0.21
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.21f, out[0].apply(100.0f));
}

void test_solver_rejects_dead_and_wild_cells() {
    Config cfg{};
    const Cal prev[3] = {airCal(100), airCal(101), airCal(99)};
    const float rawO2[3] = {100 + 371,   // good
                            101 + 40,    // barely moved -> CV_NO_RISE
                            99 + 900};   // wild slope -> outside 15% band
    Cal out[3];
    cal::O2Result res;
    Solver::solveO2(cfg, rawO2, prev, kSlope, out, res);
    TEST_ASSERT_EQUAL_UINT8(cal::CV_OK, res.verdict[0]);
    TEST_ASSERT_EQUAL_UINT8(cal::CV_NO_RISE, res.verdict[1]);
    TEST_ASSERT_EQUAL_UINT8(cal::CV_SLOPE_OUT, res.verdict[2]);
    // rejected cells keep the previous calibration untouched
    TEST_ASSERT_FALSE(out[1].hasO2);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, kSlope, out[1].slope);
    TEST_ASSERT_EQUAL_FLOAT(101.0f, out[1].rawAir);
    TEST_ASSERT_FALSE(out[2].hasO2);
}

void test_o2_seq_needs_air_then_two_step_then_solves() {
    Config cfg{};
    cfg.settleMs = 5000; cfg.stableSamples = 5;
    O2Sequencer seq{cfg};

    // refuse without air cal
    TEST_ASSERT_FALSE(seq.arm());

    Cal base[3] = {airCal(100), airCal(101), airCal(99)};
    seq.setAirCal(base);
    TEST_ASSERT_TRUE(seq.arm());
    seq.rejectArm();                          // diver walked away from the prompt
    TEST_ASSERT_FALSE(seq.confirm(1000));     // confirm without arm is refused
    O2Input in; in.nowMs = 1000; in.votedBar = 0.21f;
    TEST_ASSERT_FALSE(seq.update(in).requestEnrich); // IDLE: no enrichment

    TEST_ASSERT_TRUE(seq.arm());
    TEST_ASSERT_TRUE(seq.confirm(2000));

    // ramp: 0.21 -> 0.97 over 40 s while engine requests enrichment
    float v = 0.21f;
    uint32_t t = 2000;
    in.nowMs = t; // must never run a cycle with nowMs < runStart_ (unsigned wrap)
    bool sawRequest = false;
    O2Output last{};
    while (seq.state() == O2State::ENRICHING && t < 200000) {
        v += 0.01f; if (v > 0.97f) v = 0.97f;
        in.votedBar = v;
        in.cellsRaw[0] = 100 + (v - 0.21f) / kSlope;
        in.cellsRaw[1] = 101 + (v - 0.21f) / kSlope;
        in.cellsRaw[2] = 99 + (v - 0.21f) / kSlope;
        last = seq.update(in);
        sawRequest |= last.requestEnrich;
        t += 1000; in.nowMs = t;
    }
    TEST_ASSERT_TRUE(sawRequest);
    TEST_ASSERT_EQUAL_UINT8(O2State::SETTLING, seq.state());
    // settle: hold 6 more samples
    for (int i = 0; i < 10 && seq.state() == O2State::SETTLING; ++i) {
        last = seq.update(in); t += 1000; in.nowMs = t;
    }
    TEST_ASSERT_EQUAL_UINT8(O2State::DONE, last.state);
    for (int i = 0; i < 3; ++i)
        TEST_ASSERT_EQUAL_UINT8(cal::CV_OK, last.result.verdict[i]);
    // stored calibration proves it
    TEST_ASSERT_TRUE(seq.cal()[0].hasO2);
    TEST_ASSERT_FLOAT_WITHIN(0.0003f, kSlope, seq.cal()[0].slope);
}

void test_o2_seq_ceiling_abort_and_dive_interlock() {
    Config cfg{};
    O2Sequencer seq{cfg};
    Cal base[3] = {airCal(100), airCal(101), airCal(99)};
    seq.setAirCal(base);
    seq.arm();
    seq.confirm(0);

    O2Input in; in.nowMs = 1000; in.votedBar = 1.05f; // above ceiling 1.00
    O2Output o = seq.update(in);
    TEST_ASSERT_TRUE(o.result.aborted);
    TEST_ASSERT_EQUAL_STRING("ppO2 ceiling", o.result.abortReason);
    TEST_ASSERT_EQUAL_UINT8(O2State::FAILED, seq.state());

    // diving mid-run aborts too
    O2Sequencer seq2{cfg};
    seq2.setAirCal(base); seq2.arm(); seq2.confirm(0);
    in.votedBar = 0.5f; in.diveActive = true; in.nowMs = 5000;
    O2Output o2 = seq2.update(in);
    TEST_ASSERT_TRUE(o2.result.aborted);
    TEST_ASSERT_EQUAL_STRING("dive active", o2.result.abortReason);
}

void test_pulse_test_flags_dead_cell() {
    PulseTest pt{PulseTest::Cfg{}};
    pt.start();

    PulseTestInput in;
    in.cellsBar[0] = 0.40f; in.cellsBar[1] = 0.40f; in.cellsBar[2] = 0.40f;
    in.votedBar = 0.40f; in.presentMask = 0b111;
    uint32_t delivered = 0;

    // baseline samples
    for (int i = 0; i < 6; ++i) { in.nowMs += 500; pt.update(in); }

    // pulses: cells 0,1 rise with each pulse; cell 2 flat (dead)
    uint32_t t = in.nowMs;
    PulseTestOutput o{};
    for (int i = 0; i < 40 && pt.state() != cal::TestState::RECOVER; ++i) {
        t += 250; in.nowMs = t;
        if (o.requestPulse) delivered++;
        in.pulsesDelivered = delivered;
        const float rise = delivered * 0.03f;
        in.cellsBar[0] = 0.40f + rise; in.cellsBar[1] = 0.40f + rise * 0.9f;
        in.cellsBar[2] = 0.40f;
        o = pt.update(in);
    }
    TEST_ASSERT_EQUAL_UINT8(0b011, o.passMask);
    TEST_ASSERT_EQUAL_UINT8(0b100, o.failMask);

    // recovery: everyone back down -> DONE (capture the one-shot done edge)
    for (int i = 0; i < 10; ++i) {
        t += 1000; in.nowMs = t;
        in.cellsBar[0] = in.cellsBar[1] = in.cellsBar[2] = 0.41f;
        const PulseTestOutput r = pt.update(in);
        if (r.done) { o.done = true; o.passMask = r.passMask; }
    }
    TEST_ASSERT_TRUE(o.done);
    TEST_ASSERT_EQUAL_UINT8(0b011, o.passMask);
}

// ---------------------------------------------------------------------------
// AirCal — the CAL AIR averaging window. Raw scale here matches the bench
// (air ~1490 counts). The rules under test: only fresh data counts, an
// injection pulse discards the window, a drifting cell never completes,
// absent cells are ignored, and diving is refused outright.
// ---------------------------------------------------------------------------
static cal::AirCal::Cfg airCfg() {
    cal::AirCal::Cfg c;  // header defaults: 1 s period, 12 samples, 40 counts
    return c;
}

static void feed(cal::AirCal& ac, uint32_t& t, uint32_t untilMs,
                 const float raw[3], bool injecting = false, bool diving = false) {
    for (; t < untilMs; t += 100) {
        cal::AirCalInput in;
        in.nowMs = t;
        in.fresh = true;
        in.injecting = injecting;
        in.diving = diving;
        in.presentMask = 0x7;
        std::memcpy(in.cellsRaw, raw, sizeof(raw));
        ac.update(in);
    }
}

void test_aircal_stable_window_completes_with_means() {
    cal::AirCal ac{airCfg()};
    ac.start(0);
    const float base[3] = {1490.0f, 1502.0f, 1487.0f};
    float raw[3];
    uint32_t t = 0;
    cal::AirCalOutput done{};
    // jitter each cell +-5 counts between samples; 12 samples at 1 Hz take
    // ~11 s, so the feed loop must outlast them.
    for (int i = 0; i < 200 && !done.done; i++) {
        for (int c = 0; c < 3; c++)
            raw[c] = base[c] + ((i & 1) ? 5.0f : -5.0f);
        cal::AirCalInput in;
        in.nowMs = t; in.fresh = true; in.presentMask = 0x7;
        std::memcpy(in.cellsRaw, raw, sizeof(raw));
        done = ac.update(in);
        t += 100;
    }
    TEST_ASSERT_TRUE(done.done);
    TEST_ASSERT_EQUAL_UINT8(12, done.samples);
    TEST_ASSERT_TRUE(done.spreadWorst <= 10.0f);
    // 6 highs then 6 lows alternate: the mean sits at base +/- 0 for an
    // even window; the exact per-cell means must sit near base regardless.
    for (int c = 0; c < 3; c++)
        TEST_ASSERT_FLOAT_WITHIN(6.0f, base[c], done.rawAvg[c]);
    // terminal is one-shot: a further update says nothing
    cal::AirCalInput in{};
    in.nowMs = t; in.fresh = true;
    const cal::AirCalOutput after = ac.update(in);
    TEST_ASSERT_FALSE(after.done || after.failed || after.running);
}

void test_aircal_stale_data_never_counts() {
    cal::AirCal ac{airCfg()};
    ac.start(0);
    const float raw[3] = {1500.0f, 1500.0f, 1500.0f};
    uint32_t t = 0;
    // 30 s of FROZEN, stale readings: the window must not advance at all
    for (; t < 30000; t += 100) {
        cal::AirCalInput in;
        in.nowMs = t; in.fresh = false; in.presentMask = 0x7;
        std::memcpy(in.cellsRaw, raw, sizeof(raw));
        const cal::AirCalOutput o = ac.update(in);
        TEST_ASSERT_EQUAL_UINT8(0, o.samples);
        TEST_ASSERT_FALSE(o.done);
    }
    TEST_ASSERT_TRUE(ac.running());
}

void test_aircal_injection_discards_the_window() {
    cal::AirCal ac{airCfg()};
    ac.start(0);
    const float air[3] = {1500.0f, 1510.0f, 1495.0f};
    const float enriched[3] = {2200.0f, 2210.0f, 2195.0f};  // mid-pulse: not air
    uint32_t t = 0;
    feed(ac, t, 6000, air);           // 6 clean samples banked
    feed(ac, t, 6400, enriched, true);  // solenoid fires: window must reset
    cal::AirCalOutput done{};
    for (; t < 30000 && !done.done; t += 100) {
        cal::AirCalInput in;
        in.nowMs = t; in.fresh = true; in.presentMask = 0x7;
        std::memcpy(in.cellsRaw, air, sizeof(air));
        done = ac.update(in);
    }
    TEST_ASSERT_TRUE(done.done);
    // If any enriched sample had leaked in, the mean would sit ~350 high.
    for (int c = 0; c < 3; c++) TEST_ASSERT_FLOAT_WITHIN(1.0f, air[c], done.rawAvg[c]);
}

void test_aircal_drifting_cell_never_completes() {
    cal::AirCal ac{airCfg()};
    ac.start(0);
    // one cell ramps at 20 counts/s: honest trend, but faster than the
    // 10 counts/s the drift allowance permits — the window centre of a
    // ramping cell is not its air point. Restarts forever, then times out.
    uint32_t t = 0;
    bool failed = false;
    cal::AirCalOutput last{};
    for (uint32_t i = 0; i < 1100 && !failed; i++, t += 100) {
        float raw[3] = {1500.0f + static_cast<float>(i) * 2.0f, 1500.0f, 1500.0f};
        cal::AirCalInput in;
        in.nowMs = t; in.fresh = true; in.presentMask = 0x7;
        std::memcpy(in.cellsRaw, raw, sizeof(raw));
        last = ac.update(in);
        failed = last.failed;
    }
    TEST_ASSERT_TRUE(failed);
    TEST_ASSERT_NOT_NULL(last.why);
    TEST_ASSERT_FALSE(ac.running());
}

void test_aircal_drift_and_hysteresis_completes() {
    // Real cells are not solid constants: a bit of drift plus hysteresis is
    // normal wander. A 4 counts/s ramp carrying +-8 counts of hysteresis
    // accumulates ~60 counts of spread across the window — a flat-line band
    // check would restart forever and time out UNSETTLED. Measured against
    // the cell's own trend it is stable, and must complete.
    cal::AirCal ac{airCfg()};
    ac.start(0);
    const float base[3] = {1500.0f, 1512.0f, 1494.0f};
    uint32_t t = 0;
    cal::AirCalOutput done{};
    for (; t < 30000 && !done.done; t += 100) {
        const int k = static_cast<int>(t / 1000);  // this second's sample index
        float raw[3];
        for (int c = 0; c < 3; c++)
            raw[c] = base[c] + 4.0f * k + ((k & 1) ? 8.0f : -8.0f);
        cal::AirCalInput in;
        in.nowMs = t; in.fresh = true; in.presentMask = 0x7;
        std::memcpy(in.cellsRaw, raw, sizeof(raw));
        done = ac.update(in);
    }
    TEST_ASSERT_TRUE(done.done);
    TEST_ASSERT_EQUAL_UINT8(12, done.samples);
    // The mean of a drift+hysteresis window sits at its centre: samples k
    // 0..11 average to base + 4*5.5, the +-8 hysteresis cancelling out.
    for (int c = 0; c < 3; c++)
        TEST_ASSERT_FLOAT_WITHIN(20.0f, base[c] + 22.0f, done.rawAvg[c]);
}

void test_aircal_step_glitch_restarts_then_completes() {
    // A one-time step (contact flap, bubble) discards the window — but the
    // cell is honest afterwards, so the restarted window must complete on
    // the post-glitch values with no trace of the pre-glitch ones.
    cal::AirCal ac{airCfg()};
    ac.start(0);
    uint32_t t = 0;
    cal::AirCalOutput done{};
    for (; t < 40000 && !done.done; t += 100) {
        const bool before = (t / 1000) < 5;
        float raw[3] = {before ? 1500.0f : 1700.0f,
                        before ? 1508.0f : 1712.0f,
                        before ? 1495.0f : 1698.0f};
        cal::AirCalInput in;
        in.nowMs = t; in.fresh = true; in.presentMask = 0x7;
        std::memcpy(in.cellsRaw, raw, sizeof(raw));
        done = ac.update(in);
    }
    TEST_ASSERT_TRUE(done.done);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 1700.0f, done.rawAvg[0]);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 1712.0f, done.rawAvg[1]);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 1698.0f, done.rawAvg[2]);
}

void test_aircal_bench_excursions_complete_and_count() {
    // The bench capture (15 min at ambient, 2026-11-26): cells sit flat and
    // every now and then one steps ~50 counts for a SINGLE sample — crossing
    // a neighbour's level — then comes straight back. That is the cells
    // themselves, not contamination, and readings must NOT be discarded:
    // a window carrying two such excursions completes, and the committed
    // mean includes them. Worst recorded windows had residual 44-48 vs the
    // band of 40; outlierMax=2 is what lets them settle.
    cal::AirCal ac{airCfg()};
    ac.start(0);
    const float base[3] = {1563.0f, 1565.0f, 1610.0f};
    uint32_t t = 0;
    cal::AirCalOutput done{};
    for (; t < 30000 && !done.done; t += 100) {
        const int k = static_cast<int>(t / 1000);  // this second's sample index
        float raw[3];
        for (int c = 0; c < 3; c++)
            raw[c] = base[c] + ((k & 1) ? 3.0f : -3.0f);
        if (k == 3 || k == 8) raw[1] += 50.0f;  // cell 1 crosses cell 2's level
        if (k == 6) raw[2] -= 50.0f;            // cell 2 crosses cell 1's level
        cal::AirCalInput in;
        in.nowMs = t; in.fresh = true; in.presentMask = 0x7;
        std::memcpy(in.cellsRaw, raw, sizeof(raw));
        done = ac.update(in);
    }
    TEST_ASSERT_TRUE(done.done);
    TEST_ASSERT_EQUAL_UINT8(12, done.samples);
    // Nothing discarded: two +50 excursions out of 12 shift cell 1's mean by
    // +8.3 counts; the +-3 alternation cancels. Cell 2 loses 4.2.
    TEST_ASSERT_FLOAT_WITHIN(2.0f, base[0], done.rawAvg[0]);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, base[1] + 100.0f / 12.0f, done.rawAvg[1]);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, base[2] - 50.0f / 12.0f, done.rawAvg[2]);
}

void test_aircal_persistent_excursions_still_fail() {
    // Excursions counted, not fatal — but a cell stepping outside the band
    // on most samples is not wandering, it is unsettled: outlierMax of them
    // and the window restarts, forever, and the timeout is what says so.
    cal::AirCal ac{airCfg()};
    ac.start(0);
    const float base[3] = {1563.0f, 1565.0f, 1610.0f};
    uint32_t t = 0;
    bool failed = false;
    cal::AirCalOutput last{};
    for (; t < 100000 && !failed; t += 100) {
        const int k = static_cast<int>(t / 1000);
        float raw[3];
        for (int c = 0; c < 3; c++) raw[c] = base[c];
        if (k % 2 == 0) raw[1] += 90.0f;  // swing too wide for any trend to hug
        cal::AirCalInput in;
        in.nowMs = t; in.fresh = true; in.presentMask = 0x7;
        std::memcpy(in.cellsRaw, raw, sizeof(raw));
        last = ac.update(in);
        failed = last.failed;
    }
    TEST_ASSERT_TRUE(failed);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(cal::AirCalWhy::UNSETTLED),
                          static_cast<int>(last.whyCode));
    TEST_ASSERT_FALSE(ac.running());
}

void test_aircal_diving_is_refused_and_absent_cells_ignored() {
    cal::AirCal divingAc{airCfg()};
    divingAc.start(0);
    const float raw[3] = {1500.0f, 1510.0f, 9999.0f};  // cell 2: garbage (absent)
    cal::AirCalInput in;
    in.nowMs = 100; in.fresh = true; in.diving = true; in.presentMask = 0b011;
    std::memcpy(in.cellsRaw, raw, sizeof(raw));
    const cal::AirCalOutput o = divingAc.update(in);
    TEST_ASSERT_TRUE(o.failed);
    TEST_ASSERT_FALSE(divingAc.running());

    // same raws, two present cells: wild garbage on the absent channel
    // must not pollute the spread check or the mean.
    cal::AirCal ac{airCfg()};
    ac.start(0);
    cal::AirCalOutput done{};
    for (uint32_t tt = 0; tt < 30000 && !done.done; tt += 100) {
        cal::AirCalInput i2;
        i2.nowMs = tt; i2.fresh = true; i2.presentMask = 0b011;
        std::memcpy(i2.cellsRaw, raw, sizeof(raw));
        done = ac.update(i2);
    }
    TEST_ASSERT_TRUE(done.done);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 1500.0f, done.rawAvg[0]);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 1510.0f, done.rawAvg[1]);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_solver_accepts_good_pair);
    RUN_TEST(test_solver_rejects_dead_and_wild_cells);
    RUN_TEST(test_o2_seq_needs_air_then_two_step_then_solves);
    RUN_TEST(test_o2_seq_ceiling_abort_and_dive_interlock);
    RUN_TEST(test_pulse_test_flags_dead_cell);
    RUN_TEST(test_aircal_stable_window_completes_with_means);
    RUN_TEST(test_aircal_stale_data_never_counts);
    RUN_TEST(test_aircal_injection_discards_the_window);
    RUN_TEST(test_aircal_drifting_cell_never_completes);
    RUN_TEST(test_aircal_drift_and_hysteresis_completes);
    RUN_TEST(test_aircal_step_glitch_restarts_then_completes);
    RUN_TEST(test_aircal_bench_excursions_complete_and_count);
    RUN_TEST(test_aircal_persistent_excursions_still_fail);
    RUN_TEST(test_aircal_diving_is_refused_and_absent_cells_ignored);
    return UNITY_END();
}
