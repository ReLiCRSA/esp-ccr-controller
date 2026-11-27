// ============================================================================
// test_settings_apply — the last gate before registry numbers steer the
// valve. Per-key ranges are the registry's job; the dangerous combinations
// are *between* keys, and this suite is where those get refused out loud
// (clamped + reported) rather than run silently inverted.
// ============================================================================
#include <unity.h>

#include <cstring>

#include "comms/settings_apply.h"
#include "control/loop.h"

using comms::applySettings;
using namespace settings;

void setUp() {}
void tearDown() {}

namespace {

Registry freshReg() {
    Registry r;
    r.resetToDefaults();
    return r;
}

ctrl::Config defaultCfg() {
    // Same defaults the firmware compiles from defaults.h: the applier only
    // overlays registry values on top, so the wall/fuse/window come from here.
    ctrl::Config c;
    c.inj.highWallBar = 1.45f;
    c.inj.cutoffBar = 1.60f;
    c.inj.maxOnMs = 900;
    c.inj.windowMs = 10000;
    c.inj.maxDutyPct = 10;
    c.inj.pulseMs = 300;
    return c;
}

void set(Registry& r, const char* k, const char* v) {
    const char* err = nullptr;
    TEST_ASSERT_TRUE_MESSAGE(r.setValueByKey(k, v, &err), k);
}

} // namespace

void test_defaults_apply_clean_and_complete() {
    Registry reg = freshReg();
    ctrl::Config cfg = defaultCfg();
    const uint8_t clamps = applySettings(reg, cfg);
    TEST_ASSERT_EQUAL_UINT8(comms::CLAMP_NONE, clamps);

    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.40f, cfg.sp.sp[0]);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.30f, cfg.sp.sp[3]);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.40f, cfg.alarms.highBar);   // = firmware
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.60f, cfg.inj.cutoffBar);
    TEST_ASSERT_EQUAL_UINT16(300, cfg.inj.pulseMs);
    TEST_ASSERT_EQUAL_UINT8(10, cfg.inj.maxDutyPct);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.05f, cfg.voting.driftBar);
    TEST_ASSERT_EQUAL_UINT32(60000u, cfg.cellproof.timeoutMs);     // s -> ms
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 30.0f, cfg.deco.gfLow);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, VBAT_MIN_V, cfg.predive.vbatMin);  // follows VBAT_MIN
    // The plausibility ceiling and the series-cell limits travel with it, so the
    // gate and the alarm cannot disagree about what a readable pack is.
    TEST_ASSERT_FLOAT_WITHIN(0.001f, VBAT_IMPLAUSIBLE_V, cfg.alarms.vbatMax);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, VBAT_IMPLAUSIBLE_V, cfg.predive.vbatMax);
    TEST_ASSERT_EQUAL_UINT8(VBAT_CELLS, cfg.predive.vbatCells);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, VBAT_CELL_SPREAD_MAX_V, cfg.predive.vbatSpreadMaxV);
}

void test_cross_key_combos_are_clamped_and_reported() {
    {   // high alarm above the injection wall: the silent-cap inversion
        Registry reg = freshReg();
        set(reg, "ALLO_HIGH", "1.60");
        ctrl::Config cfg = defaultCfg();
        const uint8_t c = applySettings(reg, cfg);
        TEST_ASSERT_TRUE(c & comms::CLAMP_HIGH_ALARM);
        TEST_ASSERT_TRUE(cfg.alarms.highBar < cfg.inj.highWallBar);
    }
    {   // cutoff below the wall: the wall could never be escaped downward
        Registry reg = freshReg();
        set(reg, "ALLO_CUTOFF", "1.50");
        ctrl::Config cfg = defaultCfg();
        const uint8_t c = applySettings(reg, cfg);
        TEST_ASSERT_TRUE(c & comms::CLAMP_CUTOFF);
        TEST_ASSERT_TRUE(cfg.inj.cutoffBar > cfg.inj.highWallBar);
    }
    {   // a pulse as wide as the stuck-valve fuse
        Registry reg = freshReg();
        set(reg, "INJ_PULSE_MS", "1000");
        ctrl::Config cfg = defaultCfg();
        const uint8_t c = applySettings(reg, cfg);
        TEST_ASSERT_TRUE(c & comms::CLAMP_PULSE_MS);
        TEST_ASSERT_TRUE(cfg.inj.pulseMs < cfg.inj.maxOnMs);
    }
    {   // duty window too small to deliver the pre-dive cell-test pulses:
        // the verdict would be "solenoid failed" on a solenoid that never
        // got its pulses.
        Registry reg = freshReg();
        set(reg, "INJ_MAX_DUTY", "1");
        ctrl::Config cfg = defaultCfg();
        const uint8_t c = applySettings(reg, cfg);
        TEST_ASSERT_TRUE(c & comms::CLAMP_DUTY);
        TEST_ASSERT_TRUE((uint32_t)cfg.inj.maxDutyPct * cfg.inj.windowMs / 100u >=
                         (uint32_t)cfg.pulseTest.pulses * cfg.inj.pulseMs);
    }
    {   // shuffled setpoint ladder: sorted, not refused silently
        Registry reg = freshReg();
        set(reg, "SP1", "1.20");
        set(reg, "SP4", "0.50");
        ctrl::Config cfg = defaultCfg();
        const uint8_t c = applySettings(reg, cfg);
        TEST_ASSERT_TRUE(c & comms::CLAMP_SP_ORDER);
        TEST_ASSERT_TRUE(cfg.sp.sp[0] <= cfg.sp.sp[1]);
        TEST_ASSERT_TRUE(cfg.sp.sp[2] <= cfg.sp.sp[3]);
    }
    {   // crossed gradient factors. Each key is inside its own range, so the
        // registry accepts both; together they put the permissive factor on the
        // fast tissues, which is the direction that lengthens the ceiling the
        // diver is allowed to skip. The repair keeps a number the diver typed
        // (the stricter one) and says so.
        Registry reg = freshReg();
        set(reg, "GF_LOW", "90");
        set(reg, "GF_HIGH", "50");
        ctrl::Config cfg = defaultCfg();
        const uint8_t c = applySettings(reg, cfg);
        TEST_ASSERT_TRUE(c & comms::CLAMP_GF_ORDER);
        TEST_ASSERT_TRUE_MESSAGE(cfg.deco.gfLow <= cfg.deco.gfHigh,
                                 "gradient schedule still inverted after the applier");
        TEST_ASSERT_FLOAT_WITHIN(0.001f, 50.0f, cfg.deco.gfLow);
        TEST_ASSERT_FLOAT_WITHIN(0.001f, 50.0f, cfg.deco.gfHigh);
    }
    {   // a legal GF pair is left exactly as entered, including the wide one:
        // nothing here narrows a schedule the diver chose on purpose.
        Registry reg = freshReg();
        set(reg, "GF_LOW", "30");
        set(reg, "GF_HIGH", "95");
        ctrl::Config cfg = defaultCfg();
        TEST_ASSERT_EQUAL_UINT8(comms::CLAMP_NONE, applySettings(reg, cfg));
        TEST_ASSERT_FLOAT_WITHIN(0.001f, 30.0f, cfg.deco.gfLow);
        TEST_ASSERT_FLOAT_WITHIN(0.001f, 95.0f, cfg.deco.gfHigh);
    }
    {   // crossed depth bands: D1 below D2 is what the engine's ladder means.
        // Crossed, every depth past the deeper edge selects band 0 — the
        // shallowest target gas at the deepest water. Both numbers survive, in
        // the order the ladder runs in.
        Registry reg = freshReg();
        set(reg, "D1", "40");
        set(reg, "D2", "15");
        ctrl::Config cfg = defaultCfg();
        const uint8_t c = applySettings(reg, cfg);
        TEST_ASSERT_TRUE(c & comms::CLAMP_BAND_ORDER);
        TEST_ASSERT_TRUE_MESSAGE(cfg.sp.deep1M <= cfg.sp.deep2M,
                                 "depth bands still crossed after the applier");
        TEST_ASSERT_FLOAT_WITHIN(0.001f, 15.0f, cfg.sp.deep1M);
        TEST_ASSERT_FLOAT_WITHIN(0.001f, 40.0f, cfg.sp.deep2M);
    }
    {   // the same two depths the right way round are not a clamp
        Registry reg = freshReg();
        set(reg, "D1", "15");
        set(reg, "D2", "40");
        ctrl::Config cfg = defaultCfg();
        TEST_ASSERT_EQUAL_UINT8(comms::CLAMP_NONE, applySettings(reg, cfg));
        TEST_ASSERT_FLOAT_WITHIN(0.001f, 15.0f, cfg.sp.deep1M);
        TEST_ASSERT_FLOAT_WITHIN(0.001f, 40.0f, cfg.sp.deep2M);
    }
}

void test_mid_dive_setconfig_keeps_the_chains_memory() {
    // A SET during a dive must retune rules without resetting engines: the
    // voting exclusions, the CellProof verdict and the tissue loads are the
    // dive, not the configuration.
    ctrl::Config cfg = defaultCfg();
    cfg.cellproof.timeoutMs = 5000;   // fast proof for the test
    ctrl::Loop loop{cfg};
    Registry reg = freshReg();

    uint32_t now = 0;
    auto ticks = [&](float bar, float depth, float seconds) {
        const uint32_t n = static_cast<uint32_t>(seconds * 1000.0f / ctrl::Loop::kTickMs);
        for (uint32_t i = 0; i < n; i++) {
            ctrl::SensorData sd;
            for (int c = 0; c < 3; c++) sd.cells[c] = bar;
            sd.depthM = depth; sd.vbat = VBAT_CELLS * 3.90f;
            now += ctrl::Loop::kTickMs;
            loop.cycle(sd, now);
        }
    };

    loop.startDive(now);
    ticks(0.60f, 5.0f, 1.0f);
    ticks(0.60f, 12.0f, 10.0f);              // proof runs, cannot pass: FAIL
    TEST_ASSERT_EQUAL_UINT8(cellproof::State::FAIL, loop.cellproof().state);

    // diver lowers the clamp ceiling mid-dive
    set(reg, "CELLPROOF_MAXSP", "0.70");
    const uint8_t c = applySettings(reg, cfg);
    TEST_ASSERT_EQUAL_UINT8(comms::CLAMP_NONE, c);
    loop.setConfig(cfg);

    TEST_ASSERT_EQUAL_UINT8_MESSAGE(cellproof::State::FAIL, loop.cellproof().state,
                                    "setConfig reset the cell-proof verdict");
    ticks(0.60f, 12.0f, 1.0f);
    TEST_ASSERT_TRUE_MESSAGE(loop.setpointBar() <= 0.70f + 0.001f,
                             "new clamp ceiling did not reach the live setpoint engine");
    loop.endDive(now);
}

void test_what_the_dive_file_quotes_reaches_the_config() {
    // The dive file's `M,` row publishes gf_low/gf_high/sp1..4 read off the LIVE
    // control config, and since 2026-11-24 both depth paths take the water
    // density from there too. So these keys are quoted back at a diver as facts
    // about the dive that was just closed: a SET that answers OK and never
    // lands is the ack-with-no-consumer bug wearing new clothes, and the file
    // then reports a plan the machine never ran on.
    //
    // Checking the DEFAULTS would prove nothing: the config struct's defaults
    // and the registry's are the same numbers, so an applier that forgot a key
    // still matches. Every assertion here starts from a value the registry
    // CHANGES.
    Registry reg = freshReg();
    set(reg, "GF_LOW", "45");
    set(reg, "GF_HIGH", "70");
    set(reg, "SP1", "0.35");
    set(reg, "SP2", "0.65");
    set(reg, "SP3", "0.95");
    set(reg, "SP4", "1.25");
    set(reg, "WATER_DENSITY", "1.000");

    ctrl::Config cfg = defaultCfg();
    TEST_ASSERT_EQUAL_UINT8(comms::CLAMP_NONE, applySettings(reg, cfg));

    TEST_ASSERT_FLOAT_WITHIN(0.001f, 45.0f, cfg.deco.gfLow);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 70.0f, cfg.deco.gfHigh);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.35f, cfg.sp.sp[0]);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.65f, cfg.sp.sp[1]);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.95f, cfg.sp.sp[2]);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.25f, cfg.sp.sp[3]);
    TEST_ASSERT_FLOAT_WITHIN(0.0005f, 1.000f, cfg.deco.waterDensity);
}

// A clamp the diver cannot read is a clamp nobody notices. applySettings hands
// back a mask and leaves the registry holding the number he typed, so `GET
// GF_LOW` answers what he asked for while the chain runs something else; the
// mask is the only thing that says so. That makes "every bit has a name" a
// requirement rather than a nicety — and a graded one, so adding a clamp bit
// without a name fails here instead of shipping silent.
void test_clamp_verdict_is_readable() {
    static const uint8_t kBits[] = {comms::CLAMP_HIGH_ALARM, comms::CLAMP_CUTOFF,
                                    comms::CLAMP_PULSE_MS, comms::CLAMP_DUTY,
                                    comms::CLAMP_SP_ORDER, comms::CLAMP_GF_ORDER,
                                    comms::CLAMP_BAND_ORDER};
    for (size_t i = 0; i < sizeof(kBits); i++) {
        const char* n = comms::clampName(kBits[i]);
        TEST_ASSERT_NOT_NULL_MESSAGE(n, "a clamp bit has no name");
        TEST_ASSERT_NOT_EQUAL('\0', n[0]);
        // These names go straight into one console line: a comma or a space in
        // one would break up the list that separates them.
        TEST_ASSERT_NULL(std::strchr(n, ','));
        TEST_ASSERT_NULL(std::strchr(n, ' '));
    }
    // 1<<7 is not a clamp. An unknown bit must not borrow somebody else's name:
    // a verdict that names the wrong refusal is worse than one that says nothing.
    TEST_ASSERT_NULL(comms::clampName(1 << 7));

    char line[64];
    TEST_ASSERT_EQUAL_UINT(0u, comms::clampList(comms::CLAMP_NONE, line, sizeof line));
    TEST_ASSERT_EQUAL_STRING("", line);

    TEST_ASSERT_EQUAL_UINT(2u, comms::clampList(comms::CLAMP_GF_ORDER | comms::CLAMP_BAND_ORDER,
                                                line, sizeof line));
    TEST_ASSERT_EQUAL_STRING("GF_ORDER, BAND_ORDER", line);

    // A buffer too small for the whole mask. The list stops at whole names, so
    // what is printed is true even when it is not complete — the alternative is
    // a boot log that says "HIGH_ALARM, CUTT".
    const uint8_t all = comms::CLAMP_HIGH_ALARM | comms::CLAMP_CUTOFF | comms::CLAMP_PULSE_MS |
                        comms::CLAMP_DUTY | comms::CLAMP_SP_ORDER | comms::CLAMP_GF_ORDER |
                        comms::CLAMP_BAND_ORDER;
    char shortLine[20];
    TEST_ASSERT_EQUAL_UINT(2u, comms::clampList(all, shortLine, sizeof shortLine));
    TEST_ASSERT_EQUAL_STRING("HIGH_ALARM, CUTOFF", shortLine);
    // Belt on the same rule: nothing dangles. A reader of a truncated list must
    // never be left with a separator that promises another name after it.
    const size_t len = std::strlen(shortLine);
    TEST_ASSERT_NOT_EQUAL(',', shortLine[len - 1]);
    TEST_ASSERT_NOT_EQUAL(' ', shortLine[len - 1]);

    // A caller that passes no buffer at all gets 0, not a write through nullptr.
    TEST_ASSERT_EQUAL_UINT(0u, comms::clampList(all, nullptr, 0));
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_defaults_apply_clean_and_complete);
    RUN_TEST(test_what_the_dive_file_quotes_reaches_the_config);
    RUN_TEST(test_cross_key_combos_are_clamped_and_reported);
    RUN_TEST(test_clamp_verdict_is_readable);
    RUN_TEST(test_mid_dive_setconfig_keeps_the_chains_memory);
    return UNITY_END();
}
