// ============================================================================
// pins.h — the ONLY place GPIO assignments are defined.
// See PLAN.md section 1 for rationale.
// ============================================================================
#pragma once

// --- I2C bus 0 (cell chain ONLY: ADS1115 @0x48) ----------------------------
#define PIN_I2C_SDA       21
#define PIN_I2C_SCL       22
// 100 kHz only. 4.7k external pull-ups. Never raise the clock for speed:
// a faster bus is a bus that eats noise as ppO2 errors.

// --- I2C bus 1 (depth sensors ONLY: LPS28 @0x5C/0x5D, MS5837 @0x76) --------
// Wire1's factory-default pair (SDA 19 / SCL 18), kept off bus 0 on purpose:
// a wet depth harness short or a latched depth chip can never wedge the bus
// feeding ppO2, and the bit-banged recovery each bus runs stays local to its
// own pair. Pull-ups are per-bus: the 4.7k pair on 21/22 does NOT serve
// this one (the Adafruit breakout's on-board 10k works at 100 kHz; add a
// 4.7k pair for the wet harness). 18/19 are free on WROOM-32 and classic
// WROVER-32 (PSRAM borrows 16/17/26/32/33, not these); they are reserved
// only on 8 MB-flash D*WDH8 modules — check the shield stamp before fitting
// a different board.
#define PIN_I2C1_SDA      19
#define PIN_I2C1_SCL      18

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
// These two pins carry the in-water transport, so the choice is not casual:
// GPIO16/17 are also the PSRAM pins on WROVER modules. Free on WROOM and on
// classic WROVER (PSRAM does not claim them there), but a module with PSRAM
// mapped would steal the handset port. The boot line's heap size is the
// cross-check: ~200 KB means no PSRAM mapped, ~5 MB means this pin pair cannot
// be the handset UART on that board. Never confuse these with the carrier's
// TX/RX silkscreen either: those are GPIO1/GPIO3, the USB console, and they
// print HB lines that are not protocol (docs/HANDSET_PROTOCOL.md 7).
#define PIN_UART_H_TX     17
#define PIN_UART_H_RX     16
#define UART_H_BAUD       115200

// --- Battery sense -----------------------------------------------------------
// MUST be on ADC1 (ADC2 is unusable while WiFi runs).
// Divider 100k/10k -> ratio 11.0 (see defaults.h, VBAT_CAL trims it).
#define PIN_VBAT_SENSE    35

// Series packs only: the midpoint between the two cells. Needed because a pack
// reading 6.80 V is compatible with 4.00 + 2.80, i.e. a dead cell behind a
// passing voltage gate (defaults.h, VBAT_CELL_SPREAD_MAX_V). 36/39 are the other
// ADC1 inputs on this chip, input-only with no internal pull, and nothing else
// in this file uses them — 36 is also the SEN pin, which is what it is designed
// in this file uses them — 36 is also the SEN pin, which is what it is designed
// for. Nothing reads it while VBAT_CELLS is 1: the site that samples it is
// compiled only when defaults.h says 2 or more, so a single-cell build never
// judges a pin that has nothing connected to it.
#define PIN_VBAT_MID_SENSE  36

// --- ADS1115 DRDY/READY (optional) --------------------------------------------
// Bench has only SDA/SCL wired; set to -1. Wire DRDY to a GPIO for the
// prototype and change this constant to enable ready-driven reads.
#define PIN_ADS_DRDY      (-1)
