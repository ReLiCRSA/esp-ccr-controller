#include "alarms.h"

namespace alarms {

void Manager::evaluate(uint8_t bit, bool condition, Level level, Result& res) {
    const bool prevReported = active_[bit];

    if (kLatchAck[bit] && condition) latchAck_[bit] = true;

    const bool rawActive = condition || latchAck_[bit];
    rawActive_[bit] = rawActive;

    // ACK suppression for CONTINUOUS criticals re-arms once the condition
    // has fully recovered. (Event latches are released by ack() itself.)
    if (level == Level::CRITICAL && !rawActive) suppressed_[bit] = false;

    const bool reported = rawActive && !suppressed_[bit];

    if (reported) {
        res.bits |= (1u << bit);
        if (level == Level::CRITICAL) res.anyCritical = true;
        else res.anyWarning = true;
    }
    res.ackSuppressed[bit] = suppressed_[bit] && rawActive;

    // The log view: the FACT, whether or not the diver has an ACK holding it
    // quiet. For every bit that is not suppressed this is the same set as
    // `reported`, so the two views only ever differ on a CONTINUOUS CRITICAL the
    // diver silenced (ack() sets suppressed_ for nothing else) — and a WARNING can
    // never appear here beyond `bits` either, for the same reason.
    //
    // The rule that keeps this from becoming noise: an EVENT latch (kLatchAck) is
    // not re-marked after its ACK, because ack() releases that latch instead of
    // suppressing it, so `rawActive` drops with the ACK and the log view stops
    // claiming it. What stands is a condition that is still standing.
    if (rawActive) res.logBits |= (1u << bit);

    if (reported != prevReported) res.edges[res.edgeCount++] = {bit, reported};

    active_[bit] = reported; // what the world saw last cycle (edge tracking)
}

Result Manager::update(const Inputs& in) {
    Result res;
    // HIGH_PPO2 must see every fitted cell, not just the median: a majority of
    // poisoned cells reading low cannot be allowed to hide a loop that is
    // actually heading into oxygen toxicity.
    const float highSeen = in.highVetoBar > in.votedBar ? in.highVetoBar : in.votedBar;
    // What a missing vote makes unevaluable is the ppO2 LEVELS: LOW, HIGH and
    // CELL_DRIFT are statements about a measurement the machine does not have,
    // so with no vote their condition is false — not unknown, false, and the
    // bit is not reported. (The walls that matter are still enforced without a
    // vote: the injector's highWall/cutoff act on the cell veto, not on alarms,
    // and injection itself needs a vote to start with.)
    if (in.haveVote) {
        evaluate(BIT_LOW_PPO2, in.votedBar < cfg_.lowBar, kLevel[BIT_LOW_PPO2], res);
        evaluate(BIT_HIGH_PPO2, highSeen > cfg_.highBar, kLevel[BIT_HIGH_PPO2], res);
        evaluate(BIT_CELL_DRIFT,
                 in.vote.disagreement || in.vote.dropoutEvent,
                 kLevel[BIT_CELL_DRIFT], res);
    } else {
        evaluate(BIT_LOW_PPO2, false, kLevel[BIT_LOW_PPO2], res);
        evaluate(BIT_HIGH_PPO2, false, kLevel[BIT_HIGH_PPO2], res);
        evaluate(BIT_CELL_DRIFT, false, kLevel[BIT_CELL_DRIFT], res);
    }

    // SENSOR_FAIL sits OUTSIDE that gate, and that is the whole point of this
    // comment. It used to be evaluated only inside `if (in.haveVote)`, with the
    // else branch passing false — but haveVote IS `mode != FAILED` (loop.cpp),
    // so the condition could never reach it: the bit was dead code. The
    // consequence was the worst kind of silence. Every cell unseen, implausible
    // or excluded stops injection (injection needs a vote) and said nothing to
    // the handset flag word, the ERR LED, the E, rows or the dive log — and it
    // is the escape hatch the 50-dive soak keys on when it decides an unalarmed
    // low loop was at least honestly reported as a sensing problem.
    //
    // It is asked of the vote's own report rather than of !haveVote, because
    // this bit is about WHY there is no vote, and `cellsSeen` is the half that
    // says so: before the first ADC sample lands, mode is FAILED for the
    // additional reason that nothing has been measured yet, and a controller
    // that is still booting has no fault to announce. With no cells fitted at
    // all (presentMask 0) the bit stays quiet too — there is nothing to have
    // failed, and the pre-dive air step refuses that rig on its own.
    evaluate(BIT_SENSOR_FAIL,
             in.vote.mode == voting::Mode::FAILED && in.vote.cellsSeen,
             kLevel[BIT_SENSOR_FAIL], res);

    // HYPOXIA comes out of the gate for the same reason, on narrower grounds:
    // hypoxiaSuspect is set on direct evidence (every still-trusted cell
    // falling together, faster than the dropout slope), and that evidence can
    // arrive in the one cycle where the CellProof ban has just emptied the
    // voting set — the collapse is judged while the cell is still active, and
    // the vote pass then demotes it for the ban, so n == 0 and the mode is
    // FAILED (voting.cpp). A collapsing loop with no vote is the least
    // acceptable moment to swallow the claim, and nothing else can raise it, so
    // moving it out of the gate adds no phantom alarm.
    evaluate(BIT_HYPOXIA, in.vote.hypoxiaSuspect, kLevel[BIT_HYPOXIA], res);

    // Below the floor, or too high to be a pack of this chemistry. The second
    // half is not a different alarm: a sense node on its rail is a pack the
    // machine cannot vouch for, and the honest word for that is the same one.
    evaluate(BIT_BATTERY, in.vbat < cfg_.vbatMin || in.vbat > cfg_.vbatMax,
             kLevel[BIT_BATTERY], res);
    evaluate(BIT_SP_CLAMP, in.spClamped, kLevel[BIT_SP_CLAMP], res);
    evaluate(BIT_STUCK_OPEN, in.stuckOpen, kLevel[BIT_STUCK_OPEN], res);
    evaluate(BIT_SENSOR_STALE, in.sensorStale, kLevel[BIT_SENSOR_STALE], res);
    evaluate(BIT_CNS_HIGH, in.cnsPct >= cfg_.cnsAlarmPct, kLevel[BIT_CNS_HIGH], res);

    bits_ = res.bits;
    return res;
}

void Manager::ack() {
    for (int i = 0; i < BIT_COUNT; ++i) {
        if (kLatchAck[i]) latchAck_[i] = false; // event consumed by the diver
        else if (kLevel[i] == Level::CRITICAL && rawActive_[i])
            suppressed_[i] = true; // silence continuous alarm; condition still
                                   // visible via ackSuppressed until recovered
    }
}

}  // namespace alarms
