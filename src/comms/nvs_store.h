// ============================================================================
// nvs_store.h — persist the settings registry to NVS (PLAN.md 11).
//
// Load goes through the registry's own setValueByKey: a value saved under an
// older, wider range gets refused and falls back to its default instead of
// being trusted. That is deliberate — after a firmware update tightens a
// limit, the flash must not smuggle the old number past the new check.
// A schema key means a format change wipes stale values rather than mixing
// them into a dive-critical configuration.
//
// Compiled by the ESP32 envs only; the registry logic it drives is host-tested.
// ============================================================================
#pragma once

#include <Arduino.h>

#include "comms/settings.h"

namespace nvs {

class Store {
public:
    static constexpr char kNamespace[] = "ccr-cfg";
    static constexpr uint32_t kSchema = 4;  // bump on table/format change
                                          // (4: NVS-illegal >15-char keys renamed)

    /// Read every key into `reg` (missing/bad/out-of-range keys stay at their
    /// default). Returns the number of keys actually loaded.
    static int load(settings::Registry& reg);

    /// Write every key. Returns false on any write failure (the caller keeps
    /// the RAM values — the dive never depends on the save succeeding).
    static bool save(const settings::Registry& reg);

    static void erase();

private:
    static bool schemaOk();
    static void writeSchema();
};

} // namespace nvs
