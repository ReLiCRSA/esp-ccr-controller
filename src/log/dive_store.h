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

    /// Newest-first slot numbering for the handset `LOG LIST`:
    /// fills idx[] (slot number, 1..10) and exists/mtime arrays.
    int list(int* slotOut, uint32_t* mtimeOut, int max) const;

    /// Stream one dive, decoded, through a line sink (UART chunk, WebSocket
    /// frame...). CSV header first; stops at the first corrupt record.
    /// Returns records emitted, or -1 if the slot does not exist.
    int exportCsv(int slot, void (*sink)(const char* line, void* ctx), void* ctx) const;

    /// Raw record read for the web (binary, CRC-checked): fills up to cap,
    /// stops at the first failing record. Returns bytes written, -1 missing.
    int exportRaw(int slot, uint8_t* buf, size_t cap) const;

    void eraseAll();

private:
    int nextSlot() const;          // 1..10, evicting oldest when full
    static void pathFor(int slot, char* out, size_t cap);
    File file_;
    bool mounted_ = false;
    bool formatted_ = false;
};
