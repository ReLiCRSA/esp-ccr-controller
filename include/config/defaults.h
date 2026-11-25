// ============================================================================
// defaults.h — startup defaults. Registry-backed settings (NVS) arrive in M2/M5;
// until then these constants are the source of truth.
// See PLAN.md sections 2, 3, 5, 11.
// ============================================================================
#pragma once

// --- ADS1115 ------------------------------------------------------------------
// PGA=16: full scale +/-0.256 V, LSB = 7.8125 uV.
// Nominal cell output: 11 mV at 0.21 bar O2 (air) -> ~1408 counts.
#define ADS_PGA_GAIN                16
#define ADS_PGA_FULLSCALE_V         0.256f
#define ADS_LSB_UV                  (ADS_PGA_FULLSCALE_V * 1e6f / 32768.0f) // 7.8125
#define ADS_DATA_RATE_16SPS         // config bit 0b100
#define ADS_CONVERSION_MS           62.5f          // 16 SPS
// Timing margins: conversion slot + small gap for the config write.
#define ADS_SLOT_MS                 65             // >= conversion time + margin
#define ADS_SETUP_GAP_MS            5
// A present cell whose last valid sample is older than this is STALE
// (driver hang, dead bus, dead channel) -> sensor-health fault.
#define ADS_STALE_MS                1000
// Consecutive I2C transaction failures before a bus recovery attempt.
#define ADS_FAULTS_BEFORE_RECOVERY  2
// Bus recoveries per minute before we raise the warning lamp.
#define ADS_RECOVERY_WARN_PER_MIN   3
// Cell presence: bench today = 2 cells on AIN0/AIN1, AIN2 empty.
// Bit per channel: bit0=AIN0 ... Auto-detection (air cal plausible) lands in M2.
// Three cells fitted on the bench as of 2026-11-22 (A0/A1/A2). This mask is
// what "fitted" means everywhere: ADS polling, voting membership, the
// dashboard's cell rows and the histogram feed. A wired-but-unmasked cell is
// invisible BY DESIGN — update this (and CAL AIR) when the rig changes.
#define CELLS_PRESENT_MASK          0x07

// --- Sensor task ----------------------------------------------------------------
#define SENSOR_TASK_PERIOD_MS       10

// --- Heartbeat / stall supervision (PLAN.md 2.1) --------------------------------
// A watched task must advance its heartbeat within STALL_MS or the supervisor
// fault-logs and restarts the MCU into SAFE mode.
#define TASK_STALL_SENSOR_MS        3000
#define TASK_STALL_HUD_MS           3000
#define SUPERVISOR_PERIOD_MS        500

// --- Battery ---------------------------------------------------------------------
#define VBAT_DIVIDER_RATIO          11.0f   // (100k+10k)/10k
#define VBAT_CAL                    1.00f   // trim after bench multimeter check
#define VBAT_MIN_V                  10.5f   // adjust to the real pack chemistry
#define VBAT_SAMPLE_INTERVAL_MS     1000

// --- Cell calibration (raw -> ppO2) ------------------------------------------
// Factory-fallback slope so an uncalibrated rig still shows plausible numbers;
// it is NOT a substitute for CAL AIR, which the pre-dive/AUTO path requires.
// Bench cells read ~1490 counts in air at PGA=16.
#define CELL_AIR_RAW_NOMINAL        1490.0f
#define CAL_AIR_REF_BAR             0.21f
#define CELL_DEFAULT_SLOPE          (CAL_AIR_REF_BAR / CELL_AIR_RAW_NOMINAL)

// --- CAL AIR sampling window (cal::AirCal) ---------------------------------
// 1 Hz x 12 raw samples; every PRESENT cell must hold within CAL_AIR_SPREAD_MAX
// ADC counts across the window, or the count restarts. A cell drifting more
// than that in 12 s is warm-up drift or a bad connection — averaging it
// would commit a lie as the air point.
#define CAL_AIR_SAMPLES             12
#define CAL_AIR_PERIOD_MS           1000
#define CAL_AIR_SPREAD_MAX          40.0f
#define CAL_AIR_TIMEOUT_MS          90000

// --- Control task -------------------------------------------------------------
// One control cycle every 100 ms. The cycle is what feeds the supervisor
// heartbeat, so a stalled controller reboots the MCU into SAFE.
#define CONTROL_TASK_PERIOD_MS      100
#define TASK_STALL_CONTROL_MS       1500
// Software fuse on the output path: the solenoid pin may not stay HIGH longer
// than this, whatever the control logic asked for. A pulse is 300 ms, so this
// only fires if something above the driver has gone wrong.
#define INJ_MAX_ON_MS               900

// --- Setpoints (PLAN.md 5) -----------------------------------------------------
#define SP1_BAR                     0.40f   // surface / in water < 2 m
#define SP2_BAR                     0.70f   // in water
#define SP3_BAR                     1.00f   // deeper than SP_DEEP1_M
#define SP4_BAR                     1.30f   // deeper than SP_DEEP2_M
#define SP_IN_WATER_M               2.0f
#define SP_DEEP1_M                  30.0f
#define SP_DEEP2_M                  50.0f
#define SP_HYST_M                   0.5f

// --- Injection -----------------------------------------------------------------
#define INJ_PULSE_MS                300
// Rolling-window cap. It must leave room for the pre-dive cell test's pulses
// (see the static_assert below): the injector may not START a pulse whose own
// width would push the window over the cap, so a cap that is too tight leaves
// the cell test with one pulse and its verdict falls back to a timeout — a
// silently degraded pre-dive check. Values still pending solenoid bench
// calibration; 10% of a 10 s window = 3 x 300 ms, exactly the test's need.
#define INJ_MAX_DUTY_PCT            10
#define INJ_DUTY_WINDOW_MS          10000   //   under-report rich, never over
#define INJ_MIN_GAP_MS              2000
#define INJ_DB_ON_BAR               0.03f
#define INJ_DB_OFF_BAR              0.02f
#define INJ_HIGH_WALL_BAR           1.45f   // never START a pulse at/above
#define INJ_CUTOFF_BAR              1.60f   // force off, latched until < wall
#define INJ_STUCK_SLOPE_BARS        0.05f
#define INJ_STUCK_RISE_BAR          0.06f
#define INJ_STUCK_EVAL_MS           5000
#define INJ_STUCK_HYST_BAR          0.02f

// --- CellProof (PLAN.md 4) ------------------------------------------------------
#define CELLPROOF_ENABLED           1
#define CELLPROOF_DEPTH_M           10.0f
#define CELLPROOF_SP_BAR            1.10f
#define CELLPROOF_MIN_BAR           1.00f
#define CELLPROOF_TIMEOUT_MS        60000
#define CELLPROOF_MAXSP_BAR         0.90f
// Re-arm: cells honest at 10 m can fail at 40 m, and a proof that only runs on
// first descent cannot see it. The trigger is the machine's own evidence - a
// cell it has REJECTED reading this much above the vote, held this long. The
// 50-dive soak showed why it matters: a lying pair stuck above the setpoint
// keeps the valve shut and starves the loop at depth (vote 0.82, real 0.12).
#define CELLPROOF_SUSPECT_GAP_BAR   0.10f   // 2x drift bar: contested, not wobbling
#define CELLPROOF_SUSPECT_MS        20000
#define CELLPROOF_REARM_COOLDOWN_MS 300000
#define CELLPROOF_MAX_ATTEMPTS      3

// --- Alarms --------------------------------------------------------------------
#define PPO2_LOW_ALARM_BAR          0.18f
// Order matters: ALARM (1.40) < INJ_HIGH_WALL (1.45) < INJ_CUTOFF (1.60).
// With the alarm at the wall value the wall silently caps enrichment — the
// loop sits at 1.45 with the diver never told. Found by the 50-dive soak.
#define PPO2_HIGH_ALARM_BAR         1.40f
#define CNS_ALARM_PCT               80.0f

// --- Pre-dive (PLAN.md 7) -------------------------------------------------------
#define PREDIVE_WARMUP_MS           120000  // cell polarisation, real cells
#define PREDIVE_AIR_TOL_PCT         8.0f
#define PREDIVE_AIR_STABLE          10
#define PREDIVE_DEPTH_MAX_M         1.0f
// How long S_DEPTH waits for the pressure bus to answer AT ALL before it FAILs.
// An invalid depth sample no longer condemns the step on the cycle it lands (a
// one-cycle verdict let a shared-Wire1 hiccup abort a 2-minute sequence), but a
// bus that never answers still ends in FAIL — never a silent hang. The bound is
// the patience a dive already grants a depth sample (PRESSURE_STALE_MS below,
// pinned equal by the static_assert there).
//
// What it costs, on the real firmware, in the two shapes the fault arrives in:
//   * bus dies mid-pre-dive: main.cpp only clears depthValid once
//     PRESSURE_STALE_MS has passed without a sample, so the sequence now waits
//     3 s + 3 s = 6 s from the last measurement before the FAIL. ms5837's backoff
//     tops out at 4 s, so a driver in backoff survives this — the case the grace
//     exists for.
//   * bus never answered (nothing locked, or locked and never converting):
//     depthValid is false from boot, so S_DEPTH's own 3 s grace is the whole
//     wait, and a driver sitting at the 4 s backoff cap can be silent past it and
//     take a FAIL it would have survived with a longer bound. Deliberate: a
//     verdict beats a sequence that never returns one. It is a Config field
//     (predive::Config::depthGraceMs), so the bound is tunable without touching
//     the state machine.
#define PREDIVE_DEPTH_GRACE_MS      3000
// The cell (pulse) test: pulse the valve, every fitted cell must react. The
// verdict is only trustworthy if the requested pulses actually left the valve,
// which is why the duty window is checked against them below.
#define PREDIVE_TEST_PULSES         3
#define PREDIVE_TEST_BASELINE       6       // samples of quiet before pulsing
#define PREDIVE_RISE_MIN_BAR        0.03f
#define PREDIVE_PULSE_TIMEOUT_MS    20000
#define PREDIVE_RECOVER_TIMEOUT_MS  30000
#define PREDIVE_RECOVER_TOL_BAR     0.05f

static_assert((INJ_MAX_DUTY_PCT * INJ_DUTY_WINDOW_MS / 100) >=
                  PREDIVE_TEST_PULSES * INJ_PULSE_MS,
              "injection duty window cannot hold the pre-dive cell-test pulses");

// --- Dive log ----------------------------------------------------------------
// 30 s snapshot grid (PLAN 10); event-driven snapshots ride the E, lines.
#define DIVE_SNAPSHOT_GRID_MS       30000

// --- Depth ---------------------------------------------------------------------
// Depth source is decided at boot by identity, not by build: the dive-rated
// MS5837-30bar is preferred, the LPS28DFW (Adafruit #6067) is the bench and
// interim wet-test fallback; both drivers live on Wire1 (pins.h) so a wet
// harness fault cannot reach the cell bus. The BMP085 that once held this
// role never ACKed once and was removed 2026-11-23. Depth is derived from
// absolute pressure and a water density, both below.
//
// Absolute pressure -> depth. 1 bar of sea water (1.025 kg/l) = 9.948 m, so
// 100 hPa of weather is about 1 m of water.
//
// There is no conversion constant here any more, and that is deliberate. The
// formula lives in one place — `src/util/depthscale.h` — and BOTH paths call
// it: the control path that turns the sensor's bar reading into the depth on
// the `T,` row, and the deco engine that turns depths into ambient pressure for
// tissue modelling. The comment used to read "kept as one constant because the
// depth the control chain sees and the depth the deco engine converts must
// agree" while the deco engine quietly carried its own copy of the arithmetic
// in three places, written out with a different type and a different density
// source. That is how a 1000x error survived: the constant here was wrong and
// the engine's copy was right, so no comparison the machine could make with
// itself showed anything.
//
// UNITS MATTER, and getting them wrong is silent: rho*g is Pa/m only if rho is
// in kg/m3, and 1 bar = 100 000 Pa, so a density given in kg/L converts to bar/m
// as density * g / 100. Written as / 100000 with a kg/L density — as this was
// until 2026-11-24 — the constant is 1000x too small and every depth is 1000x
// too deep: with the depth sensor live for the first time the bench reported
// `T,37.04` at the surface (ambient 1016.7 hPa against SURFACE_PRESSURE_BAR
// 1013.0) and the setpoint engine walked from SP1 0.40 bar to SP3 1.00 bar while
// the machine was sitting on a table. Depth had never been valid on any bench
// before that day, so nothing downstream had ever seen a non-zero number to
// disbelieve. The scale is now pinned against literals in
// `test/test_depthscale` — not derived from this file, which would inherit an
// error here rather than catch one.
//
// The density below is the COMPILE-TIME default for the `WATER_DENSITY`
// setting; that setting is what both paths actually use at run time, so a diver
// switching between sea and fresh moves the depth on the wire and the deco
// plan together, on one ruler.
#define PRESSURE_ENABLE             1
#define WATER_DENSITY_KGL           1.025f
#define SURFACE_PRESSURE_BAR        1.013f
#define PRESSURE_STALE_MS           3000
// The pre-dive's DEPTH step grants this same bus the same patience before it
// FAILs the sequence (PREDIVE_DEPTH_GRACE_MS). One policy, pinned: waiting on a
// depth bus longer than a dive is willing to wait on it would leave the diver
// watching a pre-dive that is politely polling something the rest of the
// firmware has already called dead. A shorter pre-dive grace is allowed.
static_assert(PREDIVE_DEPTH_GRACE_MS <= PRESSURE_STALE_MS,
              "pre-dive depth grace must not outlast the dive's depth staleness watchdog");

// --- HUD pattern timing (ms) ------------------------------------------------------
#define HB_AUTO_ON_MS               500     // 1 Hz
#define HB_AUTO_OFF_MS              500
#define HB_STANDBY_ON_MS            1000    // slow blink
#define HB_STANDBY_OFF_MS           1000
#define HB_DOUBLE_ON_MS             100     // double-beat: degraded / SAFE
#define HB_DOUBLE_OFF_MS            100
#define HB_DOUBLE_GAP_MS            700
