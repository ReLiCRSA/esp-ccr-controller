#include "alarms.h"

namespace alarm {

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

    if (reported != prevReported) res.edges[res.edgeCount++] = {bit, reported};

    active_[bit] = reported; // what the world saw last cycle (edge tracking)
}

Result Manager::update(const Inputs& in) {
    Result res;
    // HIGH_PPO2 must see every fitted cell, not just the median: a majority of
    // poisoned cells reading low cannot be allowed to hide a loop that is
    // actually heading into oxygen toxicity.
    const float highSeen = in.highVetoBar > in.votedBar ? in.highVetoBar : in.votedBar;
    if (in.haveVote) {
        evaluate(BIT_LOW_PPO2, in.votedBar < cfg_.lowBar, kLevel[BIT_LOW_PPO2], res);
        evaluate(BIT_HIGH_PPO2, highSeen > cfg_.highBar, kLevel[BIT_HIGH_PPO2], res);
        evaluate(BIT_HYPOXIA, in.vote.hypoxiaSuspect, kLevel[BIT_HYPOXIA], res);
        evaluate(BIT_CELL_DRIFT,
                 in.vote.disagreement || in.vote.dropoutEvent,
                 kLevel[BIT_CELL_DRIFT], res);
        evaluate(BIT_SENSOR_FAIL, in.vote.mode == voting::Mode::FAILED,
                 kLevel[BIT_SENSOR_FAIL], res);
    } else {
        evaluate(BIT_LOW_PPO2, false, kLevel[BIT_LOW_PPO2], res);
        evaluate(BIT_HIGH_PPO2, false, kLevel[BIT_HIGH_PPO2], res);
        evaluate(BIT_HYPOXIA, false, kLevel[BIT_HYPOXIA], res);
        evaluate(BIT_CELL_DRIFT, false, kLevel[BIT_CELL_DRIFT], res);
        evaluate(BIT_SENSOR_FAIL, false, kLevel[BIT_SENSOR_FAIL], res);
    }
    evaluate(BIT_BATTERY, in.vbat < cfg_.vbatMin, kLevel[BIT_BATTERY], res);
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

} // namespace alarm
