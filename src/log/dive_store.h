// ============================================================================
// dive_store.h — LittleFS adapter for the dive-file codec (PLAN.md 10).
//
// Ten FIFO files, /dive1.bin .. /dive10.bin. Recording is append-only with
// fflush() per record: the file is written while a diver depends on it, and
// a reset between dives must cost at most the last record, never the dive.
// The reader stops at the first record whose CRC fails — a torn final write
// leaves the log short, it does not turn into plausible garbage.
//
// Hardware half only: the byte format, CRCs and CSV live in logfmt/ and are
// unit-tested on the host. This file is compiled by the ESP32 envs only.
// ============================================================================
#pragma once

#include <Arduino.h>
#include <FS.h>

#include "log/divelog.h"

class DiveStore {
public:
    static constexpr int kMaxDives = logfmt::MAX_DIVE_SLOTS;

    /// Mounts LittleFS (formatting once on failure) and finds the next FIFO
    /// slot. Returns false only if the filesystem is unusable — the dive
    /// still runs, it just is not recorded, and the handset gets an ERR.
    bool begin();

    /// True when the last begin() could not mount the partition and had to
    /// format it, which erases every dive on it. A dive computer that eats its
    /// own history in silence is worse than one that says the log is offline,
    /// so the boot banner is allowed to ask (see DiveStore::begin()).
    bool lastBeginFormatted() const { return formatted_; }

    /// The partition's usage, for the boot banner: after a format these numbers
    /// are the filesystem's own overhead, and that is how the bench can tell a
    /// mount that found an empty log from one that just emptied the log.
    uint32_t usedBytes() const;
    uint32_t totalBytes() const;

    bool recording() const { return (bool)file_; }

    /// Open the next FIFO slot (evicting the oldest dive when full) and write
    /// the dive header. `startEpoch` may be 0 (no RTC): records are stamped
    /// relative to dive start either way.
    bool startDive(uint32_t startEpoch);
    void append(const logfmt::Snapshot& s);
    void append(const logfmt::Event& e);
    /// Write the final meta (totals) and close. Safe to call if not recording.
    void endDive(const logfmt::DiveMeta& meta);

    /// Records written into the dive that is being closed, counted HERE because
    /// this object is the one that writes them. The `M,` summary line claims
    /// these numbers, and a claim copied from some other counter is a claim
    /// that can drift away from the rows in the file: the dive that had 13
    /// `S,` rows used to print `snapshots 0`.
    uint16_t snapshotCount() const { return snaps_; }
    uint16_t eventCount() const { return evs_; }

    /// The dive-sequence range currently on the partition (false when no dive
    /// carries one, which is the state of a board whose dives predate 2026-11-24).
    /// The boot banner prints it because that is the only place the bench can see
    /// whether eviction rotated through the slots or overwrote the same one: the
    /// slot list alone cannot tell, since a full FIFO always lists ten slots.
    bool seqRange(uint32_t& lo, uint32_t& hi) const;

    /// Newest-written-first for the handset `LOG LIST`: fills slotOut[] (slot
    /// number, 1..10) and, when given, mtimeOut[] (the dive's start epoch) and
    /// seqOut[] (the dive sequence). The sequence is the field that says which
    /// of these dives is actually the most recent: on a board with no clock
    /// every start epoch is 0, and once the FIFO rotates the slot numbers carry
    /// no order either. Unsequenced dives list last; among them the order is
    /// slot-descending, exactly as before the sequence existed.
    int list(int* slotOut, uint32_t* mtimeOut, uint32_t* seqOut, int max) const;

    /// Stream one dive, decoded, through a line sink (UART chunk, WebSocket
    /// frame...). CSV header first; stops at the first corrupt record.
    /// Returns records emitted, or -1 if the slot does not exist.
    int exportCsv(int slot, void (*sink)(const char* line, void* ctx), void* ctx) const;

private:
    /// 1..10, evicting the oldest dive when the FIFO is full. `seqOut` is the
    /// sequence the chosen dive must carry: max on the partition + 1, derived
    /// from the dive files themselves so it survives a reboot with no clock and
    /// needs nothing stored anywhere else.
    int nextSlot(uint32_t& seqOut) const;
    /// One pass over the ten slots: which hold a dive file, and what each header
    /// says. Empty slots and unreadable headers come back as 0/0, which
    /// logfmt::pickEvictSlot reads as "age unknown" and evicts first.
    int scanFifo(uint32_t* seq, uint32_t* epoch, bool* present) const;
    static void pathFor(int slot, char* out, size_t cap);
    File file_;
    bool mounted_ = false;
    bool formatted_ = false;
    uint16_t snaps_ = 0, evs_ = 0;   // records in the dive being recorded
};
