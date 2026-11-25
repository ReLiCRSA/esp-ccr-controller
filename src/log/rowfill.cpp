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

    // The vote is stored whether or not there is one: the row records what the
    // chain last voted, and the absence of a vote is recorded in `nd` (480) and
    // in the alarm word, not by zeroing a measurement.
    s.votedX1000 = static_cast<int16_t>(numfmt::scaled(in.votedBar, 3));
    for (int i = 0; i < 3; i++)
        s.cellX1000[i] = static_cast<int16_t>(numfmt::scaled(in.cellBar[i], 3));
    s.setpointX1000 = static_cast<int16_t>(numfmt::scaled(in.setpointBar, 3));

    s.mode = in.diveActive ? (in.autoMode ? 1 : 2) : 0;
    s.alarmBits = in.alarmLogBits;  // the LOG view — see RowInputs
    s.cnsPct = in.cnsPeakPct;

    if (in.deco) {
        // Measured at the chain's own depth, which is what the engine integrated
        // and what the row's depth column carries. Grading it at the diver's
        // true depth instead would write a ceiling the controller never showed
        // the diver — and in a blind window it would write the honest number
        // beside a held depth, which is the disagreement this module forbids.
        const double pAmbBar = in.deco->depthToBar(in.smoothedDepthM);
        // `* 100.0` (a double literal) was the old bug: it promotes the float and
        // pushes more products just under the integer, so this bench's measured
        // 11.15 m ceiling stored 1114 cm. The exact centimetres live in this
        // column; the `ceiling_m` column stays integer metres for v1 parsers and
        // is derived from it in the row writer.
        s.ceilingCm = static_cast<int16_t>(numfmt::scaled(in.deco->ceilingM(), 2));
        // 480 is the "no vote, nothing to total" sentinel, and it is the same
        // sentinel ndlMinutes() saturates at, so an unlimited NDL and an
        // unanswerable one cannot be confused with a real stop budget.
        s.ndlMin = in.haveVote ? static_cast<int16_t>(in.deco->ndlMinutes(pAmbBar, in.votedBar))
                               : 480;
        s.decoTotalMin = static_cast<int16_t>(in.deco->schedule(pAmbBar).totalTimeMin);
    }

    s.pulses = static_cast<uint16_t>(in.pulsesNow - in.pulsesPrev);
    s.battX1000 = static_cast<uint16_t>(numfmt::scaled(in.vbat, 3));
    // s.tempC and s.flags are not written here and RowInputs has no field for
    // them. Both columns are in the row format and nothing in the firmware has
    // ever filled them: they read 0 in every dive file ever exported. Pinning
    // that in test_logfmt keeps a half-wired sensor (a temperature nobody reads,
    // a voting-mode byte that duplicates the `E,` rows) from arriving by
    // accident; giving them meaning is a format decision, not a refactor.
}

}  // namespace logfmt
