// ============================================================================
// test_settings_apply — the last gate before registry numbers steer the
// valve. Per-key ranges are the registry's job; the dangerous combinations
// are *between* keys, and this suite is where those get refused out loud
// (clamped + reported) rather than run silently inverted.
// ============================================================================
#include <unity.h>

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
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 10.5f, cfg.predive.vbatMin);  // follows VBAT_MIN
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
            sd.depthM = depth; sd.vbat = 12.4f;
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

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_defaults_apply_clean_and_complete);
    RUN_TEST(test_what_the_dive_file_quotes_reaches_the_config);
    RUN_TEST(test_cross_key_combos_are_clamped_and_reported);
    RUN_TEST(test_mid_dive_setconfig_keeps_the_chains_memory);
    return UNITY_END();
}
