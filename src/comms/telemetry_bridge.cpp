// ============================================================================
// telemetry_bridge.cpp — event publication from the chain's own outputs.
// Hardware-free on purpose: ControlTask calls publishEvents with the State it
// already has, and the handset task drains the ring. Both halves are testable
// without a board.
// ============================================================================
#include "telemetry_bridge.h"

#include "control/alarms.h"

namespace bridge {

logfmt::DiveMeta DiveTotals::toMeta() const {
    logfmt::DiveMeta m{};
    m.durationSec = durationSec;
    m.maxDepthCm = maxDepthCm;
    m.minVotedX1000 = minVotedX1000 == 32767 ? 0 : static_cast<int16_t>(minVotedX1000);
    m.maxVotedX1000 = maxVotedX1000 < -32768 ? 0 : static_cast<int16_t>(maxVotedX1000);
    m.totalPulses = totalPulses > 0xFFFFu ? 0xFFFFu : static_cast<uint16_t>(totalPulses);
    m.cnsPeakPct = cnsPeakPct;
    return m;
}

void publishEvents(const ctrl::State& st, uint32_t tSec, EventRing& ring,
                   PublisherState& ps) {
    // Alarms: raised/cleared transitions of the latch-applied bit set. The
    // ids are protocol-visible (BIT_LOW_PPO2 .. BIT_HYPOXIA): never renumber.
    const uint16_t changed = st.alarms.bits ^ ps.prevAlarmBits;
    for (uint8_t bit = 0; bit < 10; ++bit) {
        if (!(changed & (1u << bit))) continue;
        const bool raised = st.alarms.bits & (1u << bit);
        if (!ring.push({tSec,
                        raised ? logfmt::EVT_ALARM_RAISED : logfmt::EVT_ALARM_CLEARED,
                        static_cast<uint16_t>(bit)}))
            ring.noteDropped();
    }
    ps.prevAlarmBits = st.alarms.bits;

    // AUTO/MANUAL edge (mode id: 1 auto, 2 manual, matching the T, lines).
    if (st.autoMode != ps.prevAuto) {
        if (!ring.push({tSec, logfmt::EVT_MODE_CHANGE, st.autoMode ? 1 : 2}))
            ring.noteDropped();
        ps.prevAuto = st.autoMode;
    }

    // Cellproof verdict transitions.
    const int16_t cp = static_cast<int16_t>(st.cellproof.state);
    if (cp != ps.prevCp) {
        if (cp == static_cast<int16_t>(cellproof::State::PASS) &&
            ps.prevCp == static_cast<int16_t>(cellproof::State::RUNNING)) {
            if (!ring.push({tSec, logfmt::EVT_CELLPROOF_PASS, st.cellproof.unprovenHighMask}))
                ring.noteDropped();
        } else if (cp == static_cast<int16_t>(cellproof::State::FAIL)) {
            if (!ring.push({tSec, logfmt::EVT_CELLPROOF_FAIL, 0})) ring.noteDropped();
        }
        // The clamp is a separate fact from the verdict, and it is the one the
        // diver can act on: ids 11/12 exist because "the proof failed" and "your
        // setpoints are now capped until you override" are different news. The
        // FAIL edge says both, and it says them once: staying in FAIL for 200
        // cycles must not produce 200 rows.
        if (cp == static_cast<int16_t>(cellproof::State::FAIL) &&
            ps.prevCp != static_cast<int16_t>(cellproof::State::FAIL)) {
            if (!ring.push({tSec, logfmt::EVT_CLAMP_ON, 0})) ring.noteDropped();
        }
        // FAIL → PASS happens exactly one way: `CELLPROOF OVERRIDE`, because a
        // FAIL never re-arms itself (the clamp is the safe state, lifting it is
        // the diver's call). So value 1 = lifted by the diver, and a dive CSV
        // can later tell "the clamp came off because the engine relented" (it
        // cannot) from "the diver took it off" — a distinction that matters when
        // someone reads the file afterwards.
        if (ps.prevCp == static_cast<int16_t>(cellproof::State::FAIL) &&
            cp == static_cast<int16_t>(cellproof::State::PASS)) {
            if (!ring.push({tSec, logfmt::EVT_CLAMP_OFF, 1})) ring.noteDropped();
        }
        ps.prevCp = cp;
    }

    // Which cells carry the vote (a ban or a dropout is news the diver
    // should see land in the log, not only on the display).
    if (st.vote.activeMask != ps.prevActiveMask) {
        for (uint8_t ch = 0; ch < 3; ++ch) {
            const bool was = ps.prevActiveMask & (1u << ch);
            const bool now = st.vote.activeMask & (1u << ch);
            if (was == now) continue;
            if (!ring.push({tSec, now ? logfmt::EVT_CELL_REJOIN : logfmt::EVT_CELL_EXCLUDED,
                            ch}))
                ring.noteDropped();
        }
        ps.prevActiveMask = st.vote.activeMask;
    }

    // Setpoint band changes (only report real moves, not float jitter).
    if (ps.prevSp >= 0.0f && st.setpointBar > ps.prevSp + 0.02f) {
        if (!ring.push({tSec, logfmt::EVT_SP_CHANGE,
                        static_cast<uint16_t>(st.setpointBar * 100.0f)}))
            ring.noteDropped();
    }
    ps.prevSp = st.setpointBar;
}

// ---- the CAL AIR verdict ----------------------------------------------------
// ControlTask owns the sampler; this is the seam that lets the clients know how
// it ended. Payload first, code last (see CalAir): a command task that reads a
// terminal code also reads the numbers that belong to it.
void publishCalAir(CalAir& out, const cal::AirCalOutput& a, uint32_t nowMs,
                   uint32_t tSec, EventRing& ring) {
    uint8_t code = CALAIR_NONE;
    if (a.done)       code = CALAIR_OK;
    else if (a.failed) {
        switch (a.whyCode) {
            case cal::AirCalWhy::DIVING:    code = CALAIR_DIVING; break;
            case cal::AirCalWhy::NOCELLS:   code = CALAIR_NOCELLS; break;
            case cal::AirCalWhy::UNSETTLED: code = CALAIR_UNSETTLED; break;
            default:                        code = CALAIR_UNSETTLED; break;
        }
    } else if (a.running) code = CALAIR_RUNNING;
    else return;   // nothing to say: the sampler is idle

    for (int i = 0; i < 3; ++i) out.rawAvg[i] = a.rawAvg[i];
    out.spreadWorst = a.spreadWorst;
    out.samples = a.samples;
    out.warmupLeftS = 0;
    out.atMs = nowMs;
    out.code = code;   // last: see the header

    if (code != CALAIR_RUNNING) {   // one row per verdict, not per sample
        if (!ring.push({tSec, logfmt::EVT_CAL_AIR, code})) ring.noteDropped();
    }
}

void publishCalAirRefused(CalAir& out, uint8_t code, uint32_t warmupLeftS,
                          uint32_t nowMs, uint32_t tSec, EventRing& ring) {
    for (int i = 0; i < 3; ++i) out.rawAvg[i] = 0.0f;
    out.spreadWorst = 0.0f;
    out.samples = 0;
    out.warmupLeftS = warmupLeftS;
    out.atMs = nowMs;
    out.code = code;
    if (!ring.push({tSec, logfmt::EVT_CAL_AIR, code})) ring.noteDropped();
}

} // namespace bridge
