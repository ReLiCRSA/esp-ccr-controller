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

int DiveStore::nextSlot() const {
    // Which slots are taken, asked for BY NAME. The obvious implementation —
    // open "/" and match the directory entries against `diveN.bin` — was what
    // this used to do, and on this core the scan produced nothing that matched:
    // every DIVE START therefore concluded the FIFO was empty and wrote slot 1
    // again. Ten dives were requested, one was kept, and nothing anywhere looked
    // wrong — `LOG LIST` always answered with the newest dive, which is exactly
    // what a diver would expect from a working log until the third dive. Measured
    // on the bench 2026-11-24: a boot reporting `1 dive`, a DIVE START/END cycle,
    // and the boot after it reporting `1 dive` again.
    //
    // The first free slot ends the search: finding one tells us everything DIVE
    // START needs, so the usual case is a handful of probes and no header reads.
    // Only a full FIFO has to open all ten, because eviction is by start epoch.
    for (int s = 1; s <= DiveStore::kMaxDives; s++) {
        char p[24];
        pathFor(s, p, sizeof(p));
        if (!LittleFS.exists(p)) return s;
    }

    int oldest = 0;
    uint32_t oldestTime = 0xFFFFFFFFu;
    for (int s = 1; s <= DiveStore::kMaxDives; s++) {
        char p[24];
        pathFor(s, p, sizeof(p));
        // Epoch 0 for anything unreadable — file that will not open, short read,
        // failed header CRC. A dive the reader cannot get at is the one to throw
        // away first, and it has to sort before every readable dive rather than
        // tie with them, which is what the 0 does.
        //
        // What this CANNOT do, measured on the bench 2026-11-24: with no RTC every
        // real epoch is 0 as well, so all ten keys tie, and a strict `<` keeps the
        // first slot scanned. Every dive after the tenth then overwrites slot 1:
        // ten 4 s dives, then 8 s, 40 s, 8 s left the board holding `8, 2, 3, 4, 5,
        // 7, 7, 9, 10, 10` seconds — the 40 s dive was gone before the next boot.
        // So the log keeps the nine oldest dives and the very newest, and eats each
        // recent one. The strict `<` is not the bug; the bug is that there is no
        // ordering identity to compare. Epoch ordering needs an epoch, and this
        // board has no clock (no RTC, no NTP over an AP with no uplink). The fix is
        // a monotonic dive sequence number, not a different comparison — open item,
        // PLAN 16.
        uint32_t epoch = 0u;
        File f = LittleFS.open(p, "r");
        if (f) {
            uint8_t head[HEADER_SIZE];
            uint8_t slotChk = 0;
            if (f.read(head, sizeof(head)) != (int)sizeof(head) ||
                !logfmt::unpackHeader(head, epoch, slotChk))
                epoch = 0u;
            f.close();
        }
        if (epoch < oldestTime) { oldestTime = epoch; oldest = s; }
    }
    return oldest ? oldest : 1;  // full: evict the oldest dive
}

bool DiveStore::startDive(uint32_t startEpoch) {
    if (!mounted_ && !begin()) return false;
    if (file_) file_.close();
    const int slot = nextSlot();
    char p[24];
    pathFor(slot, p, sizeof(p));
    file_ = LittleFS.open(p, "w");
    if (!file_) return false;

    // Fixed 16 B header: magic, version, start epoch, slot, CRC of the rest.
    // Records follow, interleaved by time, each type-tagged; the meta trailer
    // goes at the end (the file is append-only, totals are only known there).
    uint8_t head[HEADER_SIZE];
    logfmt::packHeader(startEpoch, (uint8_t)slot, head);
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

int DiveStore::list(int* slotOut, uint32_t* mtimeOut, int max) const {
    if (!mounted_) return 0;
    int n = 0;
    for (int s = kMaxDives; s >= 1 && n < max; s--) {  // newest-written first
        char p[24];
        pathFor(s, p, sizeof(p));
        // A missing FIFO slot is an ordinary state, but the Arduino VFS reports
        // it as an error: `open(): /littlefs/dive9.bin does not exist, no permits
        // for creation`, one line per empty slot. Asking with `exists()` first
        // does not silence it — measured 2026-11-24 on a freshly erased partition,
        // ten lines arrived anyway — so there is no cleverer probe here, just core
        // noise. What to read at boot is the store's own verdict, `log: LittleFS
        // mounted, N dives, used X of Y bytes` (BENCH.md), not these.
        File f = LittleFS.open(p, "r");
        if (!f) continue;
        uint8_t head[HEADER_SIZE];
        uint32_t epoch = 0;
        uint8_t slotGot = 0;
        if (f.read(head, sizeof(head)) >= (int)HEADER_SIZE)
            logfmt::unpackHeader(head, epoch, slotGot);
        f.close();
        slotOut[n] = s;
        if (mtimeOut) mtimeOut[n] = epoch;
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
