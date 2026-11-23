// ============================================================================
// pins.h — the ONLY place GPIO assignments are defined.
// See PLAN.md section 1 for rationale.
// ============================================================================
#pragma once

// --- I2C bus (shared: ADS1115 @0x48, LPS28 @0x5C/0x5D / MS5837 @0x76) -----
#define PIN_I2C_SDA       21
#define PIN_I2C_SCL       22
// 100 kHz only. 4.7k external pull-ups. Never raise the clock for speed:
// a faster bus is a bus that eats noise as ppO2 errors.

// --- O2 solenoid drive ------------------------------------------------------
// External MOSFET driver; circuit MUST be gate-low = valve closed.
// We intentionally leave this INPUT_PULLDOWN until the ControlTask exists
// and proves it is alive (M2). Nothing drives it high in M1.
#define PIN_SOLENOID      27
// Bench: monitoring LED on the same net (series resistor) so injection
// pulses are visible.

// --- HUD LEDs (active-high, 330R series) -------------------------------------
#define PIN_LED_HB        2    // heartbeat / system state
#define PIN_LED_ERR       4    // warning / alarm patterns
#define PIN_LED_DECO      5    // deco ceiling demanded

// --- Handset UART (protocol defined in M5) -----------------------------------
#define PIN_UART_H_TX     17
#define PIN_UART_H_RX     16
#define UART_H_BAUD       115200

// --- Battery sense -----------------------------------------------------------
// MUST be on ADC1 (ADC2 is unusable while WiFi runs).
// Divider 100k/10k -> ratio 11.0 (see defaults.h, VBAT_CAL trims it).
#define PIN_VBAT_SENSE    35

// --- ADS1115 DRDY/READY (optional) --------------------------------------------
// Bench has only SDA/SCL wired; set to -1. Wire DRDY to a GPIO for the
// prototype and change this constant to enable ready-driven reads.
#define PIN_ADS_DRDY      (-1)
