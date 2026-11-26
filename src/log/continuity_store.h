// ============================================================================
// continuity_store.h — LittleFS adapter for the continuity record (H1).
//
// NVS or LittleFS? LittleFS, and the three reasons are the three that matter
// here:
//   * Native testability. The NVS seam (comms/nvs_store.cpp) is excluded from
//     the native build and nothing in the native env emulates `Preferences` —
//     test_settings grades the registry, never the flash. The dive and surface
//     stores set the other pattern: the byte format, CRC gate and ordering rule
//     live in a host-built module (logfmt::ContinuitySlots), and the adapter
//     here is file calls only. So every decision about a continuity record —
//     which slot wins, what a torn write costs, what the next sequence is — is
//     graded on the host, and the soak can run the real classes with no file at
//     all (src/sim/rig.cpp).
//   * Boot-time visibility. The boot banner already mounts this partition and
//     already reads two record streams off it (dive files, /surface.bin). The
//     continuity line rides the same mount and the same failure reporting
//     ("cannot read" is a different fact from "nothing there"); a record in NVS
//     would have no such line, because the settings load path says nothing about
//     tissues.
//   * Wear, which is the reason NVS is usually the answer and here is not: a
//     continuity write is at most one record per 60 s of diving plus one per
//     dive, against the dive log's 30 s grid row on the SAME partition — the
//     existing, bench-accepted cadence is already 10x the traffic. At 4 KB
//     blocks on a 1 441 792 B partition (~350 blocks) and LittleFS's
//     copy-on-write spreading, a hard dive-a-day year is a few hundred erases
//     per block against a ~100 000-cycle endurance. The settings area, by
//     contrast, is where the diver's GF and setpoints live: putting the diver's
//     tissues in the same namespace as his configuration means one bad
//     settings-schema wipe (nvs_store.h bumps kSchema and CLEARS the namespace)
//     would also eat his deco debt. It does not belong in a namespace whose
//     documented repair is "erase it".
//
// The file is fixed-size: CONT_SLOTS * CONT_SLOT_STRIDE = 384 bytes at
// /continuity.bin. Slot writes are seek + write of one slot, never a truncate:
// a reset in the middle of one can only damage the slot being written, and the
// other slot's bytes are not touched (continuity.h's two-slot contract).
//
// Hardware half only: compiled by the ESP32 envs; the native env excludes it.
// ============================================================================
#pragma once

#include <Arduino.h>
#include <FS.h>

#include "log/continuity.h"

class ContinuityStore {
public:
    /// Reads the record file if there is one (a board that has never written one
    /// simply has no file: that is "no record", not an error). Returns false
    /// only when a file EXISTS and will not open — the boot banner prints that
    /// as a different fact from an empty store. Fills the RAM image either way.
    bool begin();

    /// Writes one record into the older slot and flushes it. `r.seq` is assigned
    /// here (max on the slots + 1, the same durable-counter culture as the dive
    /// FIFO and /surface.bin) and reported back through the argument, so the
    /// caller's console line names the record that is actually on flash.
    /// Returns false when nothing was written.
    bool write(logfmt::ContinuityRecord& r);

    /// The newest valid record, false when none decodes. Cheap: the image is in
    /// RAM from begin()/write(); no file is opened, so this is safe to call
    /// from a task that must not wait on flash — under the log lock, which is
    /// what keeps it from being read mid-write.
    bool newest(logfmt::ContinuityRecord& out) const;

    /// Whether a record file exists at all (for the boot banner's wording:
    /// "no record yet" versus "a record is there and does not decode"). CACHED,
    /// not a stat: this is answered on the control task at every dive start
    /// (main.cpp builds the verdict record from it), and the control task does
    /// not go asking the filesystem for anything (review H2).
    bool filePresent() const { return present_; }
    /// True when at least one slot decodes.
    bool anyRecord() const { return slots_.newest() >= 0; }
    /// True when a file is present but NOTHING in it decodes: the torn-write
    /// and version-skew case a diver must be told about rather than have
    /// silently turned into a fresh dive.
    bool corrupt() const { return present_ && !anyRecord(); }

    /// Bench only.
    void eraseAll();

    static constexpr const char* kPath = "/continuity.bin";
    static constexpr size_t kImageSize = logfmt::ContinuitySlots::kImageSize;

private:
    bool readFile();
    /// Creates/normalises the file to exactly kImageSize bytes (zeros) so every
    /// later write can be a seek into an existing extent, never an extend-past-EOF.
    bool ensureFile();

    logfmt::ContinuitySlots slots_;
    bool fileOk_ = false;    // a file exists and opened at least once
    bool present_ = false;   // a file exists (cached: see filePresent())
};
