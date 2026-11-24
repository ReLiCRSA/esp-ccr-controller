#include "comms/wallclock.h"

namespace wallclock {

bool sane(uint32_t epoch) { return epoch >= MIN_EPOCH && epoch <= MAX_EPOCH; }

uint32_t carryPick(uint32_t magic, uint32_t epoch) {
    // RTC no-init memory is uninitialised after a battery pull, not zeroed: it
    // holds whatever the silicon had there, which can be a number that looks
    // exactly like an epoch. Trusting that would date every dive in the log
    // wrong and quietly, so both the magic and the range have to hold.
    if (magic != CARRY_MAGIC) return 0;
    return sane(epoch) ? epoch : 0;
}

bool set(State& s, uint32_t epoch, uint32_t nowMs) {
    if (!sane(epoch)) return false;
    s.epochSet = epoch;
    s.setAtMs = nowMs;
    s.source = SET;
    return true;
}

uint32_t epoch(const State& s, uint32_t nowMs) {
    if (s.epochSet == 0u) return 0u;
    return s.epochSet + (uint32_t)((nowMs - s.setAtMs) / 1000u);
}

uint32_t ageS(const State& s, uint32_t nowMs) {
    if (s.epochSet == 0u) return 0u;
    return (uint32_t)((nowMs - s.setAtMs) / 1000u);
}

}  // namespace wallclock
