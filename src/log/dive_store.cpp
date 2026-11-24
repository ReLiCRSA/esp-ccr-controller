// ============================================================================
// dive_store.cpp — see dive_store.h.
//
// Rules that matter for a life-support log:
//   * append + fflush per record — a reset costs the last record, not the dive;
//   * every stored record is type-tagged and CRC-checked, and reads stop at
//     the first record that fails: a torn final write makes the log SHORT,
//     never plausible garbage;
//   * nothing here blocks on missing files or full directories: the dive
//     out-ranks the log, begin() failing is reported, it never stops control.
// ============================================================================
#include "dive_store.h"

#include "util/numfmt.h"

#include <LittleFS.h>

namespace {

using logfmt::HEADER_SIZE;

} // namespace

bool DiveStore::begin() {
    if (mounted_) return true;
    // Two attempts where there used to be one. `LittleFS.begin(true)` mounts,
    // and if the mount fails it FORMATS — erasing every dive on the partition —
    // and prints nothing at the default log level (the core's own "format()
    // successful" line is a log_d). So until now a mount problem looked exactly
    // like "the diver has no dives": the bench could not tell an empty log from
    // an erased one. Mounting without formatting first leaves the history intact
    // if the mount fails for a reason formatting would not fix, and the second
    // attempt is remembered so the boot banner can say it out loud.
    formatted_ = false;
    if (LittleFS.begin(false)) { mounted_ = true; return true; }
    formatted_ = true;
    if (!LittleFS.begin(true)) return false;
    mounted_ = true;
    return true;
}

uint32_t DiveStore::usedBytes() const {
    return mounted_ ? (uint32_t)LittleFS.usedBytes() : 0u;
}

uint32_t DiveStore::totalBytes() const {
    return mounted_ ? (uint32_t)LittleFS.totalBytes() : 0u;
}

void DiveStore::pathFor(int slot, char* out, size_t cap) {
    snprintf(out, cap, "/dive%d.bin", slot);
}

int DiveStore::scanFifo(uint32_t* seq, uint32_t* epoch, bool* present) const {
    // Slot occupancy is asked for BY NAME. The obvious implementation — open
    // "/" and match the directory entries against `diveN.bin` — is what this
    // used to do, and on this core the scan produced nothing that matched, so
    // every DIVE START concluded the FIFO was empty and wrote slot 1 again. Ten
    // dives were requested and one was kept, and nothing anywhere looked wrong:
    // `LOG LIST` always answered with the newest dive, which is what a diver
    // would expect from a working log until the third dive. Measured on the bench
    // 2026-11-24: a boot reporting `1 dive`, a DIVE START/END cycle, and the boot
    // after it reporting `1 dive` again.
    //
    // The Arduino VFS also logs a missing file at error level
    // (`open(): /littlefs/dive9.bin does not exist, no permits for creation`), one
    // line per empty slot, and `exists()` first does not silence it — measured,
    // the lines arrived anyway. They are core noise; the verdict is the boot
    // banner's `log:` line (BENCH.md).
    int held = 0;
    for (int s = 0; s < kMaxDives; s++) {
        seq[s] = 0u;
        epoch[s] = 0u;
        present[s] = false;
        if (!mounted_) continue;
        char p[24];
        pathFor(s + 1, p, sizeof(p));
        if (!LittleFS.exists(p)) continue;
        present[s] = true;
        held++;
        File f = LittleFS.open(p, "r");
        if (!f) continue;                 // occupied, age unknown
        uint8_t head[HEADER_SIZE];
        uint8_t slotChk = 0;
        if (f.read(head, sizeof(head)) == (int)sizeof(head))
            logfmt::unpackHeader(head, epoch[s], slotChk, seq[s]);
        f.close();                        // a header that will not parse stays 0/0
    }
    return held;
}

int DiveStore::nextSlot(uint32_t& seqOut) const {
    uint32_t seq[kMaxDives], epoch[kMaxDives];
    bool present[kMaxDives];
    scanFifo(seq, epoch, present);

    // All ten headers are read even when a slot is free, because the sequence the
    // new dive carries is max-on-the-partition + 1 and the only durable place
    // that maximum lives is the dive files themselves. Ten 16-byte reads at DIVE
    // START, on the command task, at the surface: the control task never runs
    // this path, and a dive is started once, not per second.
    seqOut = logfmt::nextSeq(seq, kMaxDives);

    for (int s = 0; s < kMaxDives; s++)
        if (!present[s]) return s + 1;

    // Full, so one dive has to go. Until 2026-11-24 this asked for the smallest
    // start epoch, and on a board with no clock every dive has epoch 0: all ten
    // keys tied, the comparison named the same slot every time, and dives eleven
    // and twelve landed in slot 1 one after the other. Ten 4 s dives then 8 s,
    // 40 s, 8 s left the partition holding 8, 2, 3, 4, 5, 7, 7, 9, 10, 10 seconds
    // — the 40 s dive was gone before the next boot, and the log was keeping
    // the nine OLDEST dives plus the newest. Neither `<` nor `<=` can fix a tie,
    // so the decision is made on the dive sequence instead, which exists whether
    // or not anyone has set a clock. See logfmt::pickEvictSlot and
    // test_eviction_rotates_when_there_is_no_clock.
    const int idx = logfmt::pickEvictSlot(seq, epoch, kMaxDives);
    return idx >= 0 ? idx + 1 : 1;
}

bool DiveStore::seqRange(uint32_t& lo, uint32_t& hi) const {
    uint32_t seq[kMaxDives], epoch[kMaxDives];
    bool present[kMaxDives];
    scanFifo(seq, epoch, present);
    lo = hi = 0u;
    bool any = false;
    for (int s = 0; s < kMaxDives; s++) {
        if (!present[s] || seq[s] == logfmt::SEQ_UNKNOWN) continue;
        if (!any) { lo = hi = seq[s]; any = true; continue; }
        if (seq[s] < lo) lo = seq[s];
        if (seq[s] > hi) hi = seq[s];
    }
    return any;
}

bool DiveStore::startDive(uint32_t startEpoch) {
    if (!mounted_ && !begin()) return false;
    if (file_) file_.close();
    uint32_t seq = 0;
    const int slot = nextSlot(seq);
    char p[24];
    pathFor(slot, p, sizeof(p));
    file_ = LittleFS.open(p, "w");
    if (!file_) return false;

    // Fixed 16 B header: magic, version, dive sequence, start epoch, slot, CRC
    // of the rest. Records follow, interleaved by time, each type-tagged; the
    // meta trailer goes at the end (the file is append-only, totals are only
    // known there).
    uint8_t head[HEADER_SIZE];
    logfmt::packHeader(startEpoch, (uint8_t)slot, seq, head);
    if (file_.write(head, sizeof(head)) != sizeof(head)) {
        file_.close();
        LittleFS.remove(p);
        return false;
    }
    file_.flush();
    return true;
}

void DiveStore::append(const logfmt::Snapshot& s) {
    if (!file_) return;
    uint8_t buf[1 + logfmt::SNAPSHOT_SIZE];
    buf[0] = logfmt::REC_SNAPSHOT;
    logfmt::packSnapshot(s, buf + 1);
    if (file_.write(buf, sizeof(buf)) != sizeof(buf)) {
        // A failed write is reported where a diver can see it, never retried
        // in a loop: the control task outranks the log every time.
        file_.close();
        return;
    }
    file_.flush();
}

void DiveStore::append(const logfmt::Event& e) {
    if (!file_) return;
    uint8_t buf[1 + logfmt::EVENT_SIZE];
    buf[0] = logfmt::REC_EVENT;
    logfmt::packEvent(e, buf + 1);
    if (file_.write(buf, sizeof(buf)) != sizeof(buf)) { file_.close(); return; }
    file_.flush();
}

void DiveStore::endDive(const logfmt::DiveMeta& meta) {
    if (!file_) return;
    uint8_t buf[1 + 64];
    buf[0] = logfmt::REC_META;
    logfmt::packMeta(meta, buf + 1);
    file_.write(buf, sizeof(buf));   // partial trailer fails its CRC
    file_.flush();
    file_.close();
}

int DiveStore::list(int* slotOut, uint32_t* mtimeOut, uint32_t* seqOut,
                    int max) const {
    if (!mounted_ || max <= 0) return 0;
    // Read every occupied slot, then emit newest-written first. "Newest first"
    // by slot NUMBER was only ever true while slot 1 meant oldest; once the FIFO
    // rotates, eviction replaces a dive wherever it sits and the slot numbers
    // carry no order at all. The dive sequence is the write order, so it is the
    // sort key. Unsequenced dives (seq 0: everything written before 2026-11-24,
    // or a header that will not parse) sort last, and ties keep the old slot-
    // descending order — a board of nothing but legacy dives lists exactly as it
    // did before this field existed, so the change is invisible there and an
    // improvement everywhere else.
    struct Row { int slot; uint32_t epoch, seq; };
    Row rows[kMaxDives];
    int found = 0;
    for (int s = 1; s <= kMaxDives; s++) {
        char p[24];
        pathFor(s, p, sizeof(p));
        // A missing FIFO slot is an ordinary state, but the Arduino VFS reports
        // it as an error: `open(): /littlefs/dive9.bin does not exist, no permits
        // for creation`, one line per empty slot. Asking with `exists()` first
        // does not silence it — measured 2026-11-24 on a freshly erased partition,
        // ten lines arrived anyway — so there is no cleverer probe here, just core
        // noise. What to read at boot is the store's own verdict, `log: LittleFS
        // mounted, N dives, seq a..b, used X of Y bytes` (BENCH.md), not these.
        File f = LittleFS.open(p, "r");
        if (!f) continue;
        uint8_t head[HEADER_SIZE];
        Row r{s, 0u, logfmt::SEQ_UNKNOWN};
        uint8_t slotInHeader = 0;   // checked by the CRC, not used: the path IS the slot
        if (f.read(head, sizeof(head)) >= (int)HEADER_SIZE)
            logfmt::unpackHeader(head, r.epoch, slotInHeader, r.seq);
        f.close();
        rows[found++] = r;
    }

    bool used[kMaxDives] = {false};
    int n = 0;
    while (n < max && n < found) {
        int best = -1;
        for (int i = 0; i < found; i++) {
            if (used[i]) continue;
            if (best < 0) { best = i; continue; }
            const Row& a = rows[i];
            const Row& b = rows[best];
            if (a.seq != b.seq) { if (a.seq > b.seq) best = i; continue; }  // 0 sorts last
            if (a.slot > b.slot) best = i;                                  // legacy tie: as before
        }
        used[best] = true;
        slotOut[n] = rows[best].slot;
        if (mtimeOut) mtimeOut[n] = rows[best].epoch;
        if (seqOut) seqOut[n] = rows[best].seq;
        n++;
    }
    return n;
}

namespace {

// Dive files are ~15 KB/hour by design; a whole file fits a one-shot heap
// buffer with room to spare. Export runs only on request (handset/web at
// the surface), never during control, so a temporary allocation is safe.
constexpr size_t kMaxFile = 48 * 1024;

bool slurp(const char* path, uint8_t*& buf, size_t& len) {
    File f = LittleFS.open(path, "r");
    if (!f) return false;
    const size_t sz = (size_t)f.size();
    buf = nullptr;
    len = 0;
    if (sz == 0 || sz > kMaxFile) { f.close(); return false; }
    buf = (uint8_t*)malloc(sz);
    if (!buf) { f.close(); return false; }
    if (f.read(buf, sz) != (int)sz) { free(buf); buf = nullptr; f.close(); return false; }
    f.close();
    len = sz;
    return true;
}

} // namespace

int DiveStore::exportCsv(int slot, void (*sink)(const char*, void*), void* ctx) const {
    if (!mounted_ || slot < 1 || slot > kMaxDives) return -1;
    char p[24];
    pathFor(slot, p, sizeof(p));
    uint8_t* img = nullptr;
    size_t len = 0;
    if (!slurp(p, img, len)) return -1;

    // One canonical row formatter for the whole project (see logfmt's CSV
    // section): the handset's `LOG EXPORT n` and the web's `/dive/<n>.csv` are
    // the same bytes out of the same code. They were two writers with two
    // different shapes until 2026-11-23, and the bench check that claimed they
    // matched had never compared them.
    const logfmt::ScanResult sc = logfmt::emitCsvRows(img, len, sink, ctx);
    free(img);
    return sc.headerOk ? sc.snapshots + sc.events : -1;
}

int DiveStore::exportRaw(int slot, uint8_t* buf, size_t cap) const {
    if (!mounted_ || slot < 1 || slot > kMaxDives) return -1;
    char p[24];
    pathFor(slot, p, sizeof(p));
    uint8_t* img = nullptr;
    size_t len = 0;
    if (!slurp(p, img, len)) return -1;

    const logfmt::ScanResult sc = logfmt::scanRecords(img, len, nullptr, nullptr);
    if (!sc.headerOk) { free(img); return -1; }
    // bytesConsumed is exactly "header + every record whose CRC validated":
    // the torn tail is never copied out.
    const size_t n = sc.bytesConsumed < cap ? sc.bytesConsumed : cap;
    memcpy(buf, img, n);
    free(img);
    return (int)n;
}

void DiveStore::eraseAll() {
    if (!mounted_) return;
    if (file_) file_.close();
    for (int s = 1; s <= kMaxDives; s++) {
        char p[24];
        pathFor(s, p, sizeof(p));
        LittleFS.remove(p);
    }
}
