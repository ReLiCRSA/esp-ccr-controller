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
    s.depthCm = numfmt::scaledInto<int32_t>(in.smoothedDepthM, 2);

    // The row's ppO2 is the number the diver was shown, and the chain holds that
    // across a failed vote (RowInputs::shownPpo2 — the contract, and the finding
    // it closes, are both in rowfill.h). The absence of a vote is not recorded by
    // zeroing this column: it is recorded in the alarm word, in the flags byte's
    // voting-mode bits, and in `ndl` = deco::kNDL_UNKNOWN (-1, "no number" — the
    // same value `D,` answers, and now the same value everywhere; 480 means the
    // engine's own "unlimited" and nothing else).
    //
    // Every int16 field below goes through scaledInto rather than a cast for the
    // same reason the deco picture does (deco/view.h): the stored integer is
    // narrower than long long, a cast wraps, and a wrapped ppO2 or battery is a
    // plausible-looking number from a different dive. The ppO2 columns wrap at
    // 32.767 bar and the battery at 65.535 V — neither is reachable here, and
    // 0x0000 for a battery would read as a dead pack to whoever reviews the file.
    s.votedX1000 = numfmt::scaledInto<int16_t>(in.shownPpo2, 3);
    for (int i = 0; i < 3; i++)
        s.cellX1000[i] = numfmt::scaledInto<int16_t>(in.cellBar[i], 3);
    s.setpointX1000 = numfmt::scaledInto<int16_t>(in.setpointBar, 3);

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
    } else {
        // No picture published yet: the boot window before ControlTask's first
        // cycle, or a build where no control task runs at all. The three deco
        // columns cannot carry a number then, and what they say about that has to
        // be an absence rather than a reading. For `ndl` the honest absence is
        // NOT 0: 0 minutes means "your no-deco time is finished, you are
        // obligated now", which is the opposite of "no number" and the more
        // alarming one. It answers deco::kNDL_UNKNOWN (-1), the same sentinel the
        // wire has always used for "show —" (§4), so one parser rule serves both.
        //
        // `ceiling` and `deco_min` stay 0 because the format has no sentinel for
        // them: a negative ceiling is not a defined value and would read as a
        // shallower obligation, not as an absence. The ndl sentinel is the row's
        // marker for "no deco picture", which docs/HANDSET_PROTOCOL.md 5 states.
        // On this build the case is unreachable in normal operation — a dive file
        // only opens once the chain is running, and the chain publishes a picture
        // on its first cycle — so the honesty is for the wiring, not the bench: if
        // a row ever arrives before the first publication, it must not claim a
        // diver has no stop.
        s.ndlMin = deco::kNDL_UNKNOWN;
    }

    // A window's injections, narrowed the same way as every other column. The
    // counter is uint32 and the column is uint16, and the subtraction is unsigned
    // arithmetic, so a counter that ever moved BACKWARDS used to become a huge
    // count (0xFFFF-ish) rather than nothing — a dive file claiming injections
    // that no valve made. narrow() sends a negative delta to the column's minimum
    // (0, "nothing in this window") and a delta past 65535 to its maximum.
    s.pulses = numfmt::narrow<uint16_t>(static_cast<long long>(in.pulsesNow) -
                                        static_cast<long long>(in.pulsesPrev));
    s.battX1000 = numfmt::scaledInto<uint16_t>(in.vbat, 3);
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
