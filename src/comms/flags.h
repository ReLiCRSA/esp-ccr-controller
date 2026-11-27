// ============================================================================
// flags.h — the ONE projection of the controller's alarm word onto the `FLAGS`
// byte the wire publishes (docs/HANDSET_PROTOCOL.md §4).
//
// It used to exist twice: inline in UartLink::emitT for the pushed `T,` stream,
// and as telemetryFlags() in cmd_router.cpp for `GET T` and the web. Identical
// copies, on two files, edited by whoever noticed. That is not a style problem:
// the byte is what a handset raises an alarm from, and a divergence between the
// two copies means the same fault is reported differently depending on whether
// the client was lucky enough to be listening when the stream pushed or asked
// for a snapshot. The 2026-11-27 review found exactly that shape here — CNS_HIGH
// had been dropped on BOTH transports by a mapping that believed its own comment
// about the bits 6..9 aggregate — and one copy surviving a fix would have been
// invisible on the bench unless a run happened to compare the two transports at
// that moment.
//
// So the rule is the one this repo already applies to the dive-file row
// formatter (log/rowfill.h) and the depth scale (util/depthscale.h): one
// formula, called by every transport, and the host suite grades it by name.
//
// WHAT THIS IS NOT: a copy of the alarm word. The controller's own 10-bit word
// (control/alarms.h) is untouched — these bit positions are protocol-visible and
// were never renumbered. This is a projection, and a many-to-one one: alarm bits
// 3, 6, 7 and 9 (sensor fail, stuck-open, sensor stale, hypoxia) arrive as wire
// bit 3 because from a handset's point of view they are one fact — "the loop is
// not telling you what you need, act now". CNS (alarm bit 8) deliberately does
// NOT join that aggregate: a diver over his oxygen budget is reading a working
// sensor, and the two facts ask for different actions. CNS got the byte's free
// top bit (7) on 2026-11-27, an append inside the byte's free bit — bits 0..6
// have kept their published meanings since v1, so a client that stops reading at
// FLAGS is exactly as correct as one that reads it: it sees fewer facts, never
// different ones.
//
// Bit 6 is the only one not sourced from the alarm word: it is the cell-proof
// state, which is not an alarm and does not latch or get ACKed. It has to be
// passed in separately, and that is why this takes two arguments rather than one.
// ============================================================================
#pragma once

#include <cstdint>

#include "control/alarms.h"

namespace comms {

/// The published FLAGS byte from the controller's alarm word plus the cell-proof
/// state. Pure and inline: both transports call it while building a line, and
/// neither may reach into the control chain to get it (the bridge/telemetry
/// boundary in comms/telemetry_bridge.h).
inline uint8_t flagsFromAlarmWord(uint16_t alarmBits, bool proofFail) {
    uint8_t f = 0;
    if (alarmBits & (1u << alarms::BIT_LOW_PPO2))   f |= 1 << 0;
    if (alarmBits & (1u << alarms::BIT_HIGH_PPO2))  f |= 1 << 1;
    if (alarmBits & (1u << alarms::BIT_CELL_DRIFT)) f |= 1 << 2;
    if (alarmBits & ((1u << alarms::BIT_SENSOR_FAIL) | (1u << alarms::BIT_SENSOR_STALE) |
                     (1u << alarms::BIT_STUCK_OPEN) | (1u << alarms::BIT_HYPOXIA)))
        f |= 1 << 3;
    if (alarmBits & (1u << alarms::BIT_BATTERY))    f |= 1 << 4;
    if (alarmBits & (1u << alarms::BIT_SP_CLAMP))   f |= 1 << 5;
    if (proofFail)                                  f |= 1 << 6;
    if (alarmBits & (1u << alarms::BIT_CNS_HIGH))   f |= 1 << 7;
    return f;
}

}  // namespace comms
