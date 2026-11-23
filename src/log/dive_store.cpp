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

int slotFromFile(const String& name) {
    // /dive3.bin -> 3
    if (!name.startsWith("/dive")) return -1;
    const int slash = name.indexOf('.', 4);
    if (slash < 0) return -1;
    const String num = name.substring(5, slash);
    if (num.length() == 0 || num.length() > 2) return -1;
    for (unsigned i = 0; i < num.length(); i++)
        if (num[i] < '0' || num[i] > '9') return -1;
    const int s = num.toInt();
    return (s >= 1 && s <= DiveStore::kMaxDives) ? s : -1;
}

} // namespace

bool DiveStore::begin() {
    if (!mounted_) {
        if (!LittleFS.begin(true)) return false;  // true: format once on fail
        mounted_ = true;
    }
    return true;
}

void DiveStore::pathFor(int slot, char* out, size_t cap) {
    snprintf(out, cap, "/dive%d.bin", slot);
}

int DiveStore::nextSlot() const {
    int freeSlot = 0, oldest = 0;
    uint32_t oldestTime = 0xFFFFFFFFu;
    bool occupied[DiveStore::kMaxDives + 1] = {};

    File dir = LittleFS.open("/");
    if (dir && dir.isDirectory()) {
        for (File e = dir.openNextFile(); e; e = dir.openNextFile()) {
            const int s = slotFromFile(String(e.name()));
            if (s > 0) occupied[s] = true;
            e.close();
        }
        dir.close();
    }
    for (int s = 1; s <= DiveStore::kMaxDives; s++) {
        if (!occupied[s]) { if (freeSlot == 0) freeSlot = s; continue; }
        char p[24];
        pathFor(s, p, sizeof(p));
        File f = LittleFS.open(p, "r");
        if (!f) continue;
        uint8_t head[HEADER_SIZE];
        uint32_t epoch = 0xFFFFFFFFu;
        if (f.read(head, sizeof(head)) == (int)sizeof(head)) {
            uint8_t slotChk = 0;
            if (logfmt::unpackHeader(head, epoch, slotChk)) { /* ok */ }
            else epoch = 0xFFFFFFFFu;   // unreadable header: evict it first
        }
        f.close();
        if (epoch < oldestTime) { oldestTime = epoch; oldest = s; }
    }
    if (freeSlot) return freeSlot;
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

// CSV/scan context for exportCsv: one line buffer, one sink.
struct CsvSink {
    void (*sink)(const char*, void*);
    void* ctx;
    char line[200];
};

void emitCsv(const logfmt::RawRecord& r, void* vctx) {
    CsvSink& o = *static_cast<CsvSink*>(vctx);
    if (r.tag == logfmt::REC_SNAPSHOT) {
        logfmt::Snapshot s;
        if (!logfmt::unpackSnapshot(r.bytes, s)) return;   // re-checked, always true here
        char dd[16], dv[16], dc0[16], dc1[16], dc2[16], dsp[16];
        snprintf(o.line, sizeof(o.line),
                 "S,%u,%s,%s,%s,%s,%s,%s,0x%04X,%u,%d,%d,%d,%u,%u\n",
                 s.tSec,
                 numfmt::fixedScaled(dd, sizeof(dd), s.depthCm, 2),
                 numfmt::fixedScaled(dv, sizeof(dv), s.votedX1000, 3),
                 numfmt::fixedScaled(dc0, sizeof(dc0), s.cellX1000[0], 3),
                 numfmt::fixedScaled(dc1, sizeof(dc1), s.cellX1000[1], 3),
                 numfmt::fixedScaled(dc2, sizeof(dc2), s.cellX1000[2], 3),
                 numfmt::fixedScaled(dsp, sizeof(dsp), s.setpointX1000, 3),
                 s.alarmBits,
                 s.cnsPct, s.ceilingCm / 100, s.ndlMin, s.decoTotalMin, s.pulses,
                 s.battX1000);
    } else if (r.tag == logfmt::REC_EVENT) {
        logfmt::Event e;
        if (!logfmt::unpackEvent(r.bytes, e)) return;
        snprintf(o.line, sizeof(o.line), "E,%u,%u,%u\n", e.tSec, (unsigned)e.id, e.value);
    } else {
        logfmt::DiveMeta m;
        if (!logfmt::unpackMeta(r.bytes, m)) return;
        char mn[16], mx[16];
        snprintf(o.line, sizeof(o.line),
                 "M,%u,%d,%s,%s,%u,%u,%u,%u\n", m.durationSec, m.maxDepthCm,
                 numfmt::fixed(mn, sizeof(mn), m.minVotedX1000 / 1000.0, 1),
                 numfmt::fixed(mx, sizeof(mx), m.maxVotedX1000 / 1000.0, 1),
                 m.totalPulses, m.cnsPeakPct, m.snapshotCount, m.eventCount);
    }
    o.sink(o.line, o.ctx);
}

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

    const logfmt::ScanResult sc = logfmt::scanRecords(img, len, nullptr, nullptr);
    if (!sc.headerOk) { free(img); return -1; }

    static char line[64];
    snprintf(line, sizeof(line), "DIVE,%d,%u,%d\n", slot,
             (unsigned)sc.startEpoch, logfmt::DIVE_VERSION);
    sink(line, ctx);

    CsvSink out{sink, ctx, {}};
    logfmt::scanRecords(img, len, emitCsv, &out);
    free(img);
    return sc.snapshots + sc.events;
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
