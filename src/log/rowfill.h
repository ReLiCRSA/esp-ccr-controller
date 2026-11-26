// ============================================================================
// rowfill.h — the one place a dive-file snapshot row is filled.
//
// Until 2026-11-25 there were three fills and none of them could see the others:
// ControlTask built the 30 s grid row inline in main.cpp, the command task built
// the event-driven row inline in session_wiring.cpp, and the soak re-typed
// main.cpp's conversions by hand. The first two are not compiled for host tests
// at all — main.cpp is excluded by the native build filter and so is
// session_wiring.cpp — so the only fill a native test could grade was the
// hand-written mirror, which is exactly the copy that cannot catch a drift in the
// thing it mirrors. A conversion bug in main.cpp reached a diver's download
// green. Finding #5 (an ACK deleting BIT_SENSOR_STALE out of the dive file) was
// visible at all only because the soak happened to mirror the field and then
// noticed what the firmware wrote next to it.
//
// This is the fill every caller calls. It is hardware-free by construction — no
// Serial, no FreeRTOS, no globals, no task state, one plain inputs struct in and
// one logfmt::Snapshot out — so the native env compiles it and test_logfmt grades
// the arithmetic that decides what a dive file says about a dive. main.cpp and
// session_wiring.cpp map their own state onto RowInputs and call it; the soak
// calls the same function, so its mirror is gone and the conversions are graded
// against the code that actually ships.
//
// Why it lives here, in src/log: the output type is logfmt::Snapshot, every
// conversion is the scaling that logfmt's own CSV writer prints with (a stored
// number and its digits must agree — the class of bug that made the dive file one
// count away from the wire until 2026-11-24), and the only thing it borrows from
// the control side is the published deco picture — a value struct (src/deco/view.h),
// never the engine. The dependency points one way: nothing in src/control
// includes src/log, and src/deco includes neither, so this adds no cycle.
//
// What deliberately stays with the caller:
//   * the dive META totals (max/min/peak/GF/setpoint ladder) — a dive-long
//     accumulation, not a row;
//   * where the row's numbers come from (the live chain for the grid row, the
//     published bridge scalars for the event row). Which snapshot of the machine
//     a row is allowed to read is a task-ownership question, and this module must
//     not have an opinion about it.
//
// N8 — WHO OWNS DECO STATE, stated once, here (this vocabulary closed the
// structural half of review N8 on 2026-11-28; reviews N1/N3 landed with it):
//   * ctrl::Loop::deco_ is OWNED BY ControlTask. Only the control thread ever
//     advances it (step/stepTissues/stepCns), re-tunes it (setConfig, queued
//     through the A5e deferred-apply path) or COMPUTES anything from it: the
//     derived picture is produced by deco::viewScalars/viewPlan on the control
//     task and published as a plain-value deco::View through a seqlock slot
//     (src/deco/view.h).
//   * Every other reader — the 30 s grid row, the event-driven row, the
//     handset/web `D,` answer builder — reads that PUBLISHED COPY, never the
//     engine. The event row and `D,` therefore agree by construction: they
//     read the same generation, and what the file says a moment was, the
//     diver was or is being shown the same numbers.
//   * RowInputs::deco used to be a const pointer to the live engine, and the
//     safety of every reader was prose ("a benign torn read... tolerated, not
//     endorsed"). The prose is gone because the type is gone: a value struct
//     cannot alias an engine another task is integrating. There is no longer
//     a legal call path from the command or uart2 task into deco_.
// ============================================================================
#pragma once

#include <cstdint>

#include "deco/view.h"
#include "log/divelog.h"

namespace logfmt {

/// The facts a row needs, in the units the chain publishes them in (metres, bar,
/// volts). Every field is a plain scalar so a host test can name the input and
/// assert the stored integer; nothing here is a reference to a task, a clock or a
/// board.
struct RowInputs {
    // ---- when -----------------------------------------------------------
    uint32_t nowMs = 0;        // the instant the row is written
    uint32_t diveStartMs = 0;  // tSec is seconds since dive start, integer

    // ---- depth ----------------------------------------------------------
    // The CHAIN's depth: loop.h holds the last good measurement while the
    // pressure bus is invalid, so a row written blind carries a held depth and
    // its alarm marker says so. This module has exactly one depth field on
    // purpose: a row that measured its depth one way and its deco columns
    // another is the divergence this file exists to remove.
    float smoothedDepthM = 0.0f;

    // ---- the oxygen picture --------------------------------------------
    bool haveVote = false;
    /// The ppO2 column: the SHOWN number, which the chain holds across a failed
    /// vote (control/loop.h State::shownPpo2, loop.cpp step 2b). The claim this
    /// field used to make — "the row says what the chain last voted" — was false
    /// until 2026-11-26 (the code wrote voting::Result's default 0.0f, so every
    /// row of a sensor-failure window recorded a hard 0.00 ppO2: finding N2). It
    /// is now what the code does, and it is marked rather than naked: the row's
    /// alarm word carries BIT_SENSOR_FAIL / BIT_SENSOR_STALE for exactly those
    /// cycles (the LOG view, so an ACK cannot delete it) and `ndl` carries the
    /// unknown sentinel (deco::kNDL_UNKNOWN, -1) for "no vote, nothing to total".
    /// Before the first vote ever the
    /// column still says 0, because there is no history to hold — and
    /// BIT_SENSOR_STALE is up from the first cycle for the same reason.
    /// The deliberate asymmetry with the dive META next to these rows: min_ppo2/
    /// max_ppo2 IGNORE a held value (main.cpp gates them on haveVote), because the
    /// summary answers "what was measured" and a row answers "what the diver was
    /// shown". The cell columns are unaffected by all of this: raw readings.
    float shownPpo2 = 0.0f;
    float cellBar[3] = {};
    float setpointBar = 0.0f;

    // ---- mode and alarms ------------------------------------------------
    bool diveActive = false;   // 0 = the row landed outside a dive (an event row
                               // drained after DIVE END): neither auto nor manual
    bool autoMode = false;
    /// THE LOG VIEW (alarms::Result::logBits), never the live word: a condition
    /// the diver ACKed into silence is still true, and the row is read an hour
    /// later by someone who was not there. alarms.h owns the two vocabularies.
    uint16_t alarmLogBits = 0;

    /// The dive's running CNS peak, already rounded and clamped by the caller
    /// (DiveTotals owns that number because `M,` needs the same one). The row is
    /// a passthrough: a row and the summary beside it must not disagree.
    uint8_t cnsPeakPct = 0;

    // ---- battery --------------------------------------------------------
    float vbat = 0.0f;

    // ---- injection ------------------------------------------------------
    // Absolute pulse counters; the row carries the delta across this row's
    // window. The subtraction is uint32 and the field is uint16, which is what
    // the firmware has always done: a window over 65 535 pulses (more than 21
    // minutes of continuous pulsing at the fastest duty this chain allows) wraps.
    uint32_t pulsesNow = 0;
    uint32_t pulsesPrev = 0;

    // ---- the deco picture -----------------------------------------------
    // A PUBLISHED VALUE, never the engine (N8, 2026-11-28): whoever fills a
    // row hands over the deco::View that ControlTask published — the grid row
    // takes the generation computed for this very cycle, the event row takes
    // the seqlock snapshot the `D,` line is answering from. hasDecoPicture is
    // false only in the boot window before the first control cycle published
    // one: those rows keep zeroed deco columns, and they are the ONLY rows a
    // zero plan column can belong to now. Before this change, every event row
    // ever written left ceiling/NDL/deco at zero, so its `ceiling_m` column
    // read "no stop" whether or not a stop was demanded at the instant the
    // machine decided (finding N1 — the zeros are what this replaces).
    bool hasDecoPicture = false;
    deco::View deco{};

    // ---- the flags byte (N3, 2026-11-28) ---------------------------------
    // Documented in divelog.h since v2, written by nobody until now: every
    // dive file ever exported said "OK 3-cell, not clamped", including the
    // dives that ran on two cells and under the proof clamp. Both sources are
    // the same vocabulary the wire and the held-ppO2 rule already use:
    // voting::Mode (0 ok, 1 2cell, 2 single, 3 fail — a row written while the
    // chain HELD a ppO2 across a failed vote says FAILED, not OK) and the
    // CellProof clamp being up at the instant of the row.
    uint8_t voteMode = 0;
    bool spClamped = false;
};

/// Fills the row. Fields the firmware has never written (tempC) get no entry
/// in RowInputs at all, so a caller cannot smuggle a value into a column whose
/// meaning the format has not agreed to; they keep whatever the Snapshot held
/// when it came in, which for `Snapshot{}` is zero.
void fillSnapshot(Snapshot& s, const RowInputs& in);

}  // namespace logfmt
