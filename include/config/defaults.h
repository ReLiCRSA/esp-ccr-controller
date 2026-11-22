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
#define CELLS_PRESENT_MASK          0x03

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

// --- HUD pattern timing (ms) ------------------------------------------------------
#define HB_AUTO_ON_MS               500     // 1 Hz
#define HB_AUTO_OFF_MS              500
#define HB_STANDBY_ON_MS            1000    // slow blink
#define HB_STANDBY_OFF_MS           1000
#define HB_DOUBLE_ON_MS             100     // double-beat: degraded / SAFE
#define HB_DOUBLE_OFF_MS            100
#define HB_DOUBLE_GAP_MS            700
