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
// Chemistry is ONE number here, and every battery threshold in the machine is
// derived from it. That is not tidiness: the pack changed under the firmware
// (a 12 V-style `VBAT_MIN_V 10.5` sat here while the rig ran on a single 3.7 V
// Li-ion with a step-up to 12 V for the solenoid), and that one literal was
// copied into five places — `defaults.h`, `predive.h`'s `Config::vbatMin`,
// `alarms.h`'s `Config::vbatMin`, the `VBAT_MIN` settings default, and
// `tools/pulse_witness.py` — with the settings *range* (3.0..30.0) wide enough to
// keep all of them loaded, and a test fixture asserting 10.5 as the correct
// answer. Moving them one at a time is how the old value survived: at 10.5 V a
// charged 1S pack failed the pre-dive BATTERY step permanently (`vbat=4.92
// batlow=1` on the bench, 2026-11-27), so `AUTO` could never unlock, and nobody
// had written that down anywhere. If the pack becomes two cells in series, change
// VBAT_CELLS and nothing else — except the one published row that follows the
// chemistry, `docs/HANDSET_PROTOCOL.md` §6's `VBAT_MIN` band, and the new
// settings-table join reddens that by key name if it is left behind. Witnessed
// both directions on 2026-11-27: at `2` the tree gives 448/448 native, 896/896
// across both optimisation levels and a clean `env:esp32` link; back at `1`, the
// same counts and the same clean link.
//
//   pack (measured on the bench, 2026-11-27): 1S Li-ion, 3.7 V nominal,
//   4.2 V full, divider top leg on the PACK rail, so `vbat` says nothing at all
//   about whether the 12 V the solenoid needs is actually there. That gap is
//   docs/REVIEW.md 1 P9, not a constant to tune.
#define VBAT_DIVIDER_RATIO          11.0f   // (100k+10k)/10k
#define VBAT_CAL                    1.00f   // trim after bench multimeter check

// --- chemistry ------------------------------------------------------------------
// The per-cell floor the pack must clear to start a dive. Chosen for the pack,
// not for the cells' datasheet: 3.40 V under the injection pulse is where a
// small 1S cell still has margin left, and the whole point of the gate is that
// a dive must not begin on a pack whose next pulse can brown the board out.
// The diver's own warning margin is the alarm, which reads the same number.
#define VBAT_CELLS                          1
#define VBAT_CELL_MIN_V                     3.40f
#define VBAT_MIN_V                          (VBAT_CELLS * VBAT_CELL_MIN_V)

// Fail closed on the reading itself, not just on it being low. The divider is
// two resistors on a connector: it can fall open, and the ADC pin then drifts to
// its rail and reports a *healthy* pack. `vbat=34.71` was once actually seen on
// this bench from a pinned sense pin, and a pre-dive gate that only asks "is it
// above the floor?" passes that. Any reading above this is a broken sense node,
// and a broken sense node is a refusal, never a pass.
#define VBAT_CELL_IMPLAUSIBLE_V             4.60f
#define VBAT_IMPLAUSIBLE_V                  (VBAT_CELLS * VBAT_CELL_IMPLAUSIBLE_V)

// The writable range for VBAT_MIN. Deliberately chemistry-shaped, and tighter
// than the old 3.0..30.0: a range that admits 10.5 V also *keeps* a stored 10.5,
// and the NVS on every board on this bench has one. nvs_store range-checks on
// load, so tightening the range is also the migration that clears the stale
// value instead of letting it outvote the new default forever.
#define VBAT_CELL_RANGE_MIN_V               2.80f
#define VBAT_CELL_RANGE_MAX_V               4.50f
#define VBAT_RANGE_MIN_V                    (VBAT_CELLS * VBAT_CELL_RANGE_MIN_V)
#define VBAT_RANGE_MAX_V                    (VBAT_CELLS * VBAT_CELL_RANGE_MAX_V)

// Series only: two cells whose midpoint is not sensed are one number, and one
// number can be 6.80 V with one cell at 4.00 V and the other at 2.80 V — a pack
// that passes a voltage gate with a dead cell in it. When VBAT_CELLS goes to 2
// the midpoint tap becomes a pre-dive input and this is the refusal.
#define VBAT_CELL_SPREAD_MAX_V              0.30f

#define VBAT_SAMPLE_INTERVAL_MS     1000

// --- Cell calibration (raw -> ppO2) ------------------------------------------
// Factory-fallback slope so an uncalibrated rig still shows plausible numbers;
// it is NOT a substitute for CAL AIR, which the pre-dive/AUTO path requires.
// Bench cells read ~1490 counts in air at PGA=16.
#define CELL_AIR_RAW_NOMINAL        1490.0f
#define CAL_AIR_REF_BAR             0.21f
#define CELL_DEFAULT_SLOPE          (CAL_AIR_REF_BAR / CELL_AIR_RAW_NOMINAL)

// --- CAL AIR sampling window (cal::AirCal) ---------------------------------
// 1 Hz x 12 raw samples. The cells are not solid constants — a little drift,
// hysteresis, and the odd momentary excursion (a cell crosses a neighbour's
// level for one sample and comes back) is what electrochemical cells do —
// so stability is judged against each PRESENT cell's own trend across the
// window, not a flat line: samples must sit within CAL_AIR_SPREAD_MAX ADC
// counts of the fitted trend, up to CAL_AIR_OUTLIER_MAX of them per cell may
// step outside, and the trend itself must not move faster than
// CAL_AIR_DRIFT_MAX_PS counts/s. A true warm-up ramp breaks the rate limit;
// sustained band-crossing is not wander — averaging either would commit a
// lie as the air point. Every reading counts in the mean, outliers included.
#define CAL_AIR_SAMPLES             12
#define CAL_AIR_PERIOD_MS           1000
#define CAL_AIR_SPREAD_MAX          40.0f
#define CAL_AIR_DRIFT_MAX_PS        10.0f
#define CAL_AIR_OUTLIER_MAX         2
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

// --- Dive continuity (H1): the crash-window grid ------------------------------
// One record carries the diver's tissues across a power cycle. The record is
// written at DIVE END plus on this grid during a dive, and the grid's period IS
// the crash window: restore uses the newest WRITTEN record and fast-forwards it
// across the surface interval exactly, so the worst a reset during a dive can
// cost is one period of depth time that the restore then credits as surface
// off-gassing. Sized against that, not against flash:
//   * at 60 s and 40 m, the tissue error that window can hide is bounded by
//     (ppN2_depth - ppN2_surface) * (1 - e^(-ln2*60/300)) = 3.54 * 0.129 =
//     0.46 bar on the FASTEST (5 min) compartment — every slower compartment
//     moves less in 60 s — which is a few metres of ceiling. The 5 min the
//     reviewer suggested would put that at ~1.8 bar and ~15 m for the same
//     depth, for a 5x saving in writes that this partition does not need.
//   * flash: 60 records/hour, against the dive log's 120 grid rows/hour on the
//     same partition — 10x less traffic than what already ships, and the writes
//     happen on HudTask, never on the control task (contgate.h).
// Seed refresh: how often the surface interval is recomputed while the board
// sits on the surface, so a dive started two days after boot is credited two
// days. One second of granularity against intervals of minutes to days.
#define CONTINUITY_SNAPSHOT_MS      60000
#define CONTINUITY_SEED_REFRESH_MS  1000
// How long HudTask may wait for the log mutex to commit one record. Its stall
// budget is 3000 ms (TASK_STALL_HUD_MS); a record is not worth anywhere near it.
#define CONTINUITY_DRAIN_BUDGET_MS  20

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
