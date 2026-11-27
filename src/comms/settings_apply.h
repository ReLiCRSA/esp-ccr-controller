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
// registry — it reports the clamp instead, and clampList() below is how that
// verdict reaches the console. It matters more than it looks: the registry
// keeps the number the diver typed, so `GET GF_LOW` answers what was asked
// for while the chain runs the clamped one. The wire carries the verdict as
// `E,<t>,19,<mask>` (logfmt::EVT_SETTINGS_CLAMPED, mask = this enum), so a
// handset learns WHICH ordering was enforced without any existing line
// growing a field — the event stream is the append-only place for it
// (docs/HANDSET_PROTOCOL.md §4).
// ============================================================================
#pragma once

#include <cstddef>
#include <cstdint>

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
    CLAMP_GF_ORDER = 1 << 5,    // GF_LOW above GF_HIGH: gradient schedule inverted
    CLAMP_BAND_ORDER = 1 << 6,  // D1 above D2: the ladder's depth bands crossed
};

uint8_t applySettings(const settings::Registry& reg, ctrl::Config& cfg);

/// Name of one clamp bit (exactly one bit set), or nullptr for a bit that is
/// not defined. Every bit in ApplyClamp has to have a name: a clamp a diver
/// cannot read is a clamp nobody notices. test_settings_apply grades that.
const char* clampName(uint8_t bit);

/// Comma-separated names of every bit in `mask` into the caller's buffer,
/// e.g. "GF_ORDER, BAND_ORDER". Returns how many names were written; a mask of
/// CLAMP_NONE writes nothing and returns 0. Names that do not fit inside `cap`
/// are left out rather than cut in half, so a short buffer still reads true.
size_t clampList(uint8_t mask, char* out, size_t cap);

} // namespace comms
