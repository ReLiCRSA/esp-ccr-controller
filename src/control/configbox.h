// ============================================================================
// configbox.h — the APPLIED control configuration as a published generation.
//
// ctrl::Config is the set of numbers the chain runs on: setpoint ladder and its
// depth bands, gradient factors, alarm edges, the injection walls, the cell-proof
// window, the pre-dive patience. Until 2026-11-27 it lived in one global
// (`g_ctrlCfg`) that the settings path wrote FIELD BY FIELD while three other
// tasks read it — SensorTask (water density), SupervisorTask (the solenoid
// max-on fuse) and ControlTask (present mask, gradient pair, setpoint ladder,
// pre-dive grace).
//
// Two things were wrong with that, and only one of them is a torn word:
//
//   * Coherence. A reader could catch the middle of an update: the new
//     ALLO_HIGH alongside the old ALLO_CUTOFF, or GF_LOW already lowered while
//     GF_HIGH still held the old number. settings_apply.cpp exists precisely to
//     make those pairs impossible (its clamps run over a complete set), so the
//     global handed back a combination the applier guarantees never exists. On
//     a 32-bit core no single word is torn; the PAIR is what lies.
//   * The applier read the global too. applySettings compares
//     `alarms.highBar` against `inj.highWallBar` and `inj.cutoffBar` against the
//     same wall, and reads `inj.maxOnMs`, `inj.windowMs` and `pulseTest.pulses`
//     — fields it does not write, carried from boot. Applying onto the same
//     object it reads means its own decisions are made against a config that is
//     in the middle of becoming something else.
//
// So: the settings path applies onto a PRIVATE copy, and publishes the finished
// result as one generation. Readers take a whole generation per use — one per
// control cycle, one per sensor sample — and never see half of a change.
//
// A mutex, not a seqlock: the payload is a struct copy in and a struct copy
// out, both bounded (a few hundred bytes), and the writer does its slow work —
// the registry walk and the clamps — BEFORE taking the lock. A seqlock would
// read the payload while it may be being written and decide afterwards; that is
// the data race the pattern accepts, and this value does not need to accept it.
// Blocking a control cycle for a struct copy is nothing beside the schedule()
// call a few lines later, and the tick budget is 1500 ms.
// ============================================================================
#pragma once

#include <cstdint>

#include "control/loop.h"

namespace ctrl {

/// Publishes the finished configuration as one generation. The settings path is
/// the ONLY writer (`hookSettings`, and setup() at boot); publishes from two
/// places would make "last writer wins" the whole contract, and that is not a
/// property worth relying on here.
void configPublish(const Config& cfg);

/// Copies the newest published generation into `out` and returns true.
///
/// Before the first publish it returns false and leaves `out` EXACTLY as it was,
/// so a caller that has pre-seeded something sensible (defaults.h at boot) keeps
/// it instead of receiving a value-initialised struct whose setpoints are 0.00
/// and whose duty is 0 % — a config that reads as "no oxygen ever" far more
/// convincingly than "nothing published yet".
///
/// The copy is atomic with respect to a publish: the reader gets one complete
/// generation, never a blend of two.
bool configRead(Config& out);

/// The same read, and the NUMBER of the generation it copied. A reader that wants
/// to say "these are the rules this cycle ran under" needs the pair — the number
/// belongs to the values, and asking for `configGeneration()` afterwards reports
/// whatever the settings path published in the meantime, which is the same class
/// of lie the box exists to remove. `*generation` is written only when the read
/// succeeds, for the same reason `out` is left alone.
bool configRead(Config& out, uint32_t* generation);

/// How many generations have been published (0 = nothing yet). Debug output and
/// tests use it to say which rules a surface was produced under.
uint32_t configGeneration();
}  // namespace ctrl
