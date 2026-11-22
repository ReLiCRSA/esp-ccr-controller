// ============================================================================
// settings_apply.h — the one place registry values become control-chain
// configuration. Handset SET and web Settings both raise ACT_SETTINGS from
// the router; the firmware calls applySettings() once in response and then
// pushes the result into ctrl::Loop::setConfig. One applier means a value can
// never mean something different on the handset than on the web page.
//
// Safety clamping lives here too, at the last moment before the numbers
// reach the chain: the alarm must stay below the injection wall (PLAN 16),
// whatever the registry ranges allowed. The applier never rewrites the
// registry — it reports the clamp so the event log records it.
// ============================================================================
#pragma once

#include "comms/settings.h"
#include "control/loop.h"

namespace comms {

/// Copy every relevant registry value into `cfg` (a full ctrl::Config).
/// Returns a bitmask of clamps applied (for event logging); a SET that was
/// legal per the registry but unsafe in combination gets clamped here and
/// the diver is told, rather than the machine silently running inverted
/// thresholds.
enum ApplyClamp : uint8_t {
    CLAMP_NONE = 0,
    CLAMP_HIGH_ALARM = 1 << 0,  // ALLO_HIGH raised to/above the injection wall
    CLAMP_CUTOFF = 1 << 1,      // ALLO_CUTOFF below wall+margin
    CLAMP_PULSE_MS = 1 << 2,    // pulse wider than the output-fuse hold
    CLAMP_DUTY = 1 << 3,        // duty window cannot hold the pre-dive pulses
    CLAMP_SP_ORDER = 1 << 4,    // setpoints re-sorted shallow -> deep
};

uint8_t applySettings(const settings::Registry& reg, ctrl::Config& cfg);

} // namespace comms
