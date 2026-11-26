// ============================================================================
// telemetry_bridge.cpp — event publication from the chain's own outputs.
// Hardware-free on purpose: ControlTask calls publishEvents with the State it
// already has, and the handset task drains the ring. Both halves are testable
// without a board.
// ============================================================================
#include "telemetry_bridge.h"

#include "control/alarms.h"
#include "util/numfmt.h"           // scaled(): the payload digits must match the GET answer

#include <cstdio>              // snprintf for the depth-source identity copy (bounded)

namespace bridge {

logfmt::DiveMeta DiveTotals::toMeta() const {
    logfmt::DiveMeta m{};
    m.durationSec = durationSec;
    m.maxDepthCm = maxDepthCm;
    m.minVotedX1000 = minVotedX1000 == 32767 ? 0 : static_cast<int16_t>(minVotedX1000);
    // Symmetric with the min side. The previous guard (`< -32768`) could never
    // be true, so the "never voted" sentinel was published verbatim: a dive
    // started with no cells reading, or with them all rejected, wrote
    // `max_ppo2 = -32.768` into its own summary — an impossible ppO2, printed
    // with the same three decimals as every real one.
    m.maxVotedX1000 = maxVotedX1000 <= -32768 ? 0 : static_cast<int16_t>(maxVotedX1000);
    m.totalPulses = totalPulses > 0xFFFFu ? 0xFFFFu : static_cast<uint16_t>(totalPulses);
    m.cnsPeakPct = cnsPeakPct;
    // The plan in force, not a guess at one: these are the values the control
    // chain was running when the dive ended. Left at zero by a chain that has
    // nothing to say, which the CSV then shows as zero rather than as a number
    // someone made up.
    m.gfLow = gfLow;
    m.gfHigh = gfHigh;
    m.sp1X1000 = sp1X1000;
    m.sp2X1000 = sp2X1000;
    m.sp3X1000 = sp3X1000;
    m.sp4X1000 = sp4X1000;
    // `eventCount`/`snapshotCount` are deliberately NOT taken from here: the
    // store counts the rows it actually wrote, at close (see hookCloseDive).
    return m;
}

int publishEvents(const ctrl::State& st, uint32_t tSec, EventRing& ring,
                  PublisherState& ps) {
    // pushed = edges published this cycle. main.cpp uses a nonzero count as
    // the deco-plan republish trigger (the ring overflow counter is separate:
    // a DROPPED edge is still an edge — the picture must move for it too).
    int pushed = 0;
    auto push = [&](Event e) { if (!ring.push(e)) ring.noteDropped(); ++pushed; };
    // Alarms: raised/cleared transitions of the latch-applied bit set. The
    // ids are protocol-visible (BIT_LOW_PPO2 .. BIT_HYPOXIA): never renumber.
    const uint16_t changed = st.alarms.bits ^ ps.prevAlarmBits;
    for (uint8_t bit = 0; bit < 10; ++bit) {
        if (!(changed & (1u << bit))) continue;
        const bool raised = st.alarms.bits & (1u << bit);
        push({tSec, raised ? logfmt::EVT_ALARM_RAISED : logfmt::EVT_ALARM_CLEARED,
              static_cast<uint16_t>(bit)});
    }
    ps.prevAlarmBits = st.alarms.bits;

    // AUTO/MANUAL edge (mode id: 1 auto, 2 manual, matching the T, lines).
    if (st.autoMode != ps.prevAuto) {
        push({tSec, logfmt::EVT_MODE_CHANGE,
              static_cast<uint16_t>(st.autoMode ? 1 : 2)});
        ps.prevAuto = st.autoMode;
    }

    // Cellproof verdict transitions.
    const int16_t cp = static_cast<int16_t>(st.cellproof.state);
    if (cp != ps.prevCp) {
        if (cp == static_cast<int16_t>(cellproof::State::PASS) &&
            ps.prevCp == static_cast<int16_t>(cellproof::State::RUNNING)) {
            push({tSec, logfmt::EVT_CELLPROOF_PASS, st.cellproof.unprovenHighMask});
        } else if (cp == static_cast<int16_t>(cellproof::State::FAIL)) {
            push({tSec, logfmt::EVT_CELLPROOF_FAIL, 0});
        }
        // The clamp is a separate fact from the verdict, and it is the one the
        // diver can act on: ids 11/12 exist because "the proof failed" and "your
        // setpoints are now capped until you override" are different news. The
        // FAIL edge says both, and it says them once: staying in FAIL for 200
        // cycles must not produce 200 rows.
        if (cp == static_cast<int16_t>(cellproof::State::FAIL) &&
            ps.prevCp != static_cast<int16_t>(cellproof::State::FAIL)) {
            push({tSec, logfmt::EVT_CLAMP_ON, 0});
        }
        // FAIL → PASS happens exactly one way: `CELLPROOF OVERRIDE`, because a
        // FAIL never re-arms itself (the clamp is the safe state, lifting it is
        // the diver's call). So value 1 = lifted by the diver, and a dive CSV
        // can later tell "the clamp came off because the engine relented" (it
        // cannot) from "the diver took it off" — a distinction that matters when
        // someone reads the file afterwards.
        if (ps.prevCp == static_cast<int16_t>(cellproof::State::FAIL) &&
            cp == static_cast<int16_t>(cellproof::State::PASS)) {
            push({tSec, logfmt::EVT_CLAMP_OFF, 1});
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
            push({tSec, now ? logfmt::EVT_CELL_REJOIN : logfmt::EVT_CELL_EXCLUDED, ch});
        }
        ps.prevActiveMask = st.vote.activeMask;
    }

    // Setpoint band changes (only report real moves, not float jitter).
    if (ps.prevSp >= 0.0f && st.setpointBar > ps.prevSp + 0.02f) {
        // The payload is the setpoint in hundredths, and it has to be the number
        // the handset reads back with `GET SP2` — 0.29 must arrive as 29, not 28.
        push({tSec, logfmt::EVT_SP_CHANGE,
              static_cast<uint16_t>(numfmt::scaled(st.setpointBar, 2))});
    }
    ps.prevSp = st.setpointBar;
    return pushed;

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


// See telemetry_bridge.h for why these rules are a function rather than five
// assignments at the SensorTask call site: main.cpp is not compiled for the host
// build, so a rule that lives there is a rule no test can see.
void publishDepthSource(DepthSource& out, bool locked, bool used, bool healthy,
                        uint32_t lastGoodMs, const char* identity) {
    // Nothing means nothing: an unlocked slot has no source to be used, healthy
    // or 300 s silent, and no name. Forcing all five together is what lets a
    // client read `DS,0,0,0,0,none` as a complete statement instead of wondering
    // which fields are meaningful this cycle.
    if (!locked) {
        used = false;
        healthy = false;
        lastGoodMs = 0;
        identity = "none";
    }
    // No write order here can make five fields atomic (see the struct's note), so
    // this is best-effort coherence for anyone reading the struct directly, and
    // the authoritative guard is in the reply: `GET DS` prints a name only when
    // `locked` vouches for it.
    //
    // The name goes through snprintf with the destination's own size, so a driver
    // that ever hands out something longer is truncated rather than overrunning
    // the array, and the array is always terminated for the reply's %s. A null
    // here is a driver bug, and "none" is the honest answer to a driver bug: the
    // alternative is a blank field a reader fills in himself.
    out.locked = locked;
    out.used = used;
    out.healthy = healthy;
    out.lastGoodMs = lastGoodMs;
    std::snprintf(out.identity, sizeof(out.identity), "%s",
                  identity ? identity : "none");
}

} // namespace bridge
