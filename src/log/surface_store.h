// ============================================================================
// surface_store.h — LittleFS adapter for the surface log (PLAN.md 16).
//
// One file, /surface.bin: a flat stream of fixed 24 B records of what the
// machine did at the surface — a calibration verdict, a pre-dive pass or
// failure. Those facts have no dive file to live in: the dive a failed pre-dive
// check prevents never gets a header to hold a row, and until 2026-11-24 the
// only record of "the cells were refused for warm-up" was the console stream,
// which nobody keeps.
//
// Same torn-write contract as the dive files, and for the same reason: append +
// flush per record, and a reader stops at the first record that fails its CRC, so
// a reset mid-write costs that record and leaves the log short rather than
// plausible-but-wrong.
//
// The sequence lives in the records themselves (max on the file + 1), exactly as
// the dive FIFO's does, because this board has no clock to order anything with.
// It is the surface log's OWN sequence, not the dives': one shared counter would
// make a dive's seq depend on how many calibrations happened, and that number is
// what decides which dive gets evicted.
//
// Capacity: 256 records, oldest shed first, by rewriting the file without its
// first `compactSurface()` records. The rewrite is staging-then-swap
// (/surface.tmp, remove, rename) so an interrupted compaction cannot leave a
// half-written /surface.bin. Cost, stated rather than hidden: between the remove
// and the rename there is a window in which a power cut loses the surface history
// — it happens at most once every 256 records, and it can never cost a dive.
//
// Web/HTTP only: no handset commands exist for this log (see divelog.h).
// Hardware half only: the byte format, CRCs and CSV rows live in logfmt/ and are
// unit-tested on the host.
// ============================================================================
#pragma once

#include <Arduino.h>
#include <FS.h>

#include "log/divelog.h"

class SurfaceStore {
public:
    static constexpr int kMaxRecords = logfmt::SURFACE_MAX_RECORDS;

    /// Appends one record and flushes it. `r.seq` is assigned here (max on the
    /// file + 1) and reported back through the argument, so the console line and
    /// the exported row cannot disagree about which record they are talking
    /// about; `r.epoch` and the payload come from the caller, because only the
    /// caller knows the clock's state. Returns false if nothing was written —
    /// the record is then simply not in the log, and the caller says so.
    ///
    /// Overflow is not an error: when the file is full the oldest records are
    /// shed first. If that rewrite fails (no heap for the file handle), the
    /// record is still appended, so the log is temporarily larger than
    /// kMaxRecords rather than missing the fact that just happened.
    bool append(logfmt::SurfaceRecord& r);

    /// One pass over the file: the number of readable records (a torn tail record
    /// is not counted), and when given, the lowest and highest sequence among
    /// them. -1 if a file exists but will not open; 0 covers "no log yet" and
    /// "log is empty" alike. This is the read API — count, seq range and the boot
    /// banner all come from a single pass, because every pass over the log starts
    /// with the core's own error line for an absent file, and the boot banner is
    /// the instrument this bench is read with.
    int records(uint32_t* seqLo, uint32_t* seqHi) const;

    /// Readable records on the file (a torn tail record is not counted).
    int count() const;

    /// Lowest and highest sequence on the file; false when empty. The pair is
    /// what lets the boot banner and the CSV header admit that history was shed:
    /// `seq 41..300` with 256 records says 44 went, `1..256` says nothing did.
    bool seqRange(uint32_t& lo, uint32_t& hi) const;

    /// Streams the whole log through a line sink: identity row first, then one
    /// `R,` row per record, oldest first. Returns the records emitted (0 for an
    /// empty or absent log), or -1 if the file could not be opened at all.
    int exportCsv(void (*sink)(const char* line, void* ctx), void* ctx) const;

    /// Bench only: deletes the surface history.
    void eraseAll();

private:
    static constexpr const char* kPath = "/surface.bin";
    static constexpr const char* kTmpPath = "/surface.tmp";

    /// Records before `drop`, copied to /surface.tmp, then swapped in. Byte-level
    /// copy in stride-sized chunks: no heap, and a partial record at the end of
    /// the source is dropped by the copy (a torn tail row stays torn and stays
    /// unread, which is the same contract the reader already applies).
    bool compact(int drop);
    uint32_t lastSeq() const;
};
