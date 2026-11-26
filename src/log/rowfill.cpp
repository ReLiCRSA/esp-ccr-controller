// ============================================================================
// rowfill.cpp — see rowfill.h. Every conversion in here is the reason a dive
// file says what it says; the native suite (test/test_logfmt) grades these
// numbers field by field, which is the coverage main.cpp could never have.
// ============================================================================
#include "rowfill.h"

#include "util/numfmt.h"

namespace logfmt {

void fillSnapshot(Snapshot& s, const RowInputs& in) {
    s.tSec = (in.nowMs - in.diveStartMs) / 1000u;

    // numfmt::scaled with the same decimal count the CSV writer prints with —
    // never a bare cast. A decimal like 20.30f is not exact in binary and a
    // truncating cast stored 2029 cm, so the dive file spent a year one count
    // away from the wire it has to be comparable with (fixed 2026-11-24; the
    // same rule in the `M,` totals; test_numfmt fails if the two renderings of
    // one sample ever disagree).
    s.depthCm = static_cast<int32_t>(numfmt::scaled(in.smoothedDepthM, 2));

    // The row's ppO2 is the number the diver was shown, and the chain holds that
    // across a failed vote (RowInputs::shownPpo2 — the contract, and the finding
    // it closes, are both in rowfill.h). The absence of a vote is not recorded by
    // zeroing this column: it is recorded in the alarm word, in the flags byte's
    // voting-mode bits, and in `ndl` = deco::kNDL_UNKNOWN (-1, "no number" — the
    // same value `D,` answers, and now the same value everywhere; 480 means the
    // engine's own "unlimited" and nothing else).
    s.votedX1000 = static_cast<int16_t>(numfmt::scaled(in.shownPpo2, 3));
    for (int i = 0; i < 3; i++)
        s.cellX1000[i] = static_cast<int16_t>(numfmt::scaled(in.cellBar[i], 3));
    s.setpointX1000 = static_cast<int16_t>(numfmt::scaled(in.setpointBar, 3));

    s.mode = in.diveActive ? (in.autoMode ? 1 : 2) : 0;
    s.alarmBits = in.alarmLogBits;  // the LOG view — see RowInputs
    s.cnsPct = in.cnsPeakPct;

    if (in.hasDecoPicture) {
        // Copied from the PUBLISHED picture (deco/view.h), computed by
        // ControlTask for this instant — the fill performs no deco arithmetic
        // and holds no engine. That is not an optimisation, it is the shape
        // of the N8 fix: a command task used to run the engine's NDL search
        // and its full ascent schedule right here, against a live engine,
        // reading tissue states that never existed at one instant. The
        // published generation is what the diver was shown (the `D,` line
        // answers from the same slot), and the ceiling still describes the
        // chain's own depth, because the engine the picture was computed from
        // is the one that integrated the depth this row's depth column carries.
        s.ceilingCm = in.deco.ceilingCm;
        s.ndlMin = in.deco.ndlMin;  // sentinels: kNDL_UNKNOWN / kNDL_UNLIMITED
        // Integer minutes, as this column has always been: divide the stored
        // x10 — never re-round, so the column and `D,`'s decimal agree by
        // construction (both derive from one scaled store).
        s.decoTotalMin = static_cast<int16_t>(in.deco.decoTotalX10 / 10);
    }

    s.pulses = static_cast<uint16_t>(in.pulsesNow - in.pulsesPrev);
    s.battX1000 = static_cast<uint16_t>(numfmt::scaled(in.vbat, 3));
    // The documented flags byte (divelog.h): b0 = the setpoint clamp up,
    // b1..2 = the voting mode the row was written under (voting::Mode).
    // Until 2026-11-28 nothing wrote this column and every dive file said
    // "OK 3-cell, not clamped" (finding N3). The alarm word beside it marks
    // the same condition from its side; the two must agree, and both come
    // from the same instant's State.
    s.flags = static_cast<uint8_t>((in.spClamped ? 1u : 0u) |
                                   ((in.voteMode & 3u) << 1));
    // s.tempC is not written here and RowInputs has no field for it: the
    // column reads 0 in every dive file ever exported, and pinning that in
    // test_logfmt keeps a half-wired sensor (a temperature nobody reads) from
    // arriving by accident; giving it meaning is a format decision.
}

}  // namespace logfmt
