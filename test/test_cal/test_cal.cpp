#include <unity.h>

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

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_solver_accepts_good_pair);
    RUN_TEST(test_solver_rejects_dead_and_wild_cells);
    RUN_TEST(test_o2_seq_needs_air_then_two_step_then_solves);
    RUN_TEST(test_o2_seq_ceiling_abort_and_dive_interlock);
    RUN_TEST(test_pulse_test_flags_dead_cell);
    return UNITY_END();
}
