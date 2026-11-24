// ============================================================================
// surface_store.cpp — see surface_store.h.
// ============================================================================
#include "surface_store.h"

#include <LittleFS.h>

namespace {

constexpr size_t kStride = logfmt::SURFACE_RECORD_SIZE;

} // namespace

// Stops at the first record that fails its CRC or carries a version this build
// does not know, and does not skip it: a record that fails is where writing
// stopped, and anything after it is bytes nobody asked the machine to store.
int SurfaceStore::records(uint32_t* seqLo, uint32_t* seqHi) const {
    if (!LittleFS.exists(kPath)) return 0;
    File f = LittleFS.open(kPath, "r");
    if (!f) return -1;

    int n = 0;
    uint32_t a = 0, b = 0;
    uint8_t buf[kStride];
    logfmt::SurfaceRecord r{};
    while (f.read(buf, sizeof(buf)) == (int)sizeof(buf)) {
        if (!logfmt::unpackSurface(buf, r)) break;
        if (n == 0) { a = b = r.seq; }
        else {
            if (r.seq < a) a = r.seq;
            if (r.seq > b) b = r.seq;
        }
        n++;
    }
    f.close();
    if (seqLo) *seqLo = a;
    if (seqHi) *seqHi = b;
    return n;
}

int SurfaceStore::count() const {
    // -1 propagates on purpose: "the file exists and will not read" must not
    // arrive at the boot banner as "no records yet", which is what an empty log
    // says. The two are different facts and only one of them is harmless.
    return records(nullptr, nullptr);
}

bool SurfaceStore::seqRange(uint32_t& lo, uint32_t& hi) const {
    uint32_t a = 0, b = 0;
    const bool any = records(&a, &b) > 0;
    lo = a;
    hi = b;
    return any;
}

uint32_t SurfaceStore::lastSeq() const {
    uint32_t hi = 0;
    records(nullptr, &hi);
    return hi;
}

bool SurfaceStore::compact(int drop) {
    if (drop <= 0) return true;
    // A /surface.tmp left by an earlier failure is stale: it would be renamed
    // over the live log with records the live log no longer has.
    if (LittleFS.exists(kTmpPath)) LittleFS.remove(kTmpPath);

    File in = LittleFS.open(kPath, "r");
    if (!in) return false;
    if (!in.seek((off_t)drop * kStride)) { in.close(); return false; }

    File out = LittleFS.open(kTmpPath, "w");
    if (!out) { in.close(); return false; }
    uint8_t buf[kStride];
    size_t moved = 0;
    bool ok = true;
    while (in.read(buf, sizeof(buf)) == (int)sizeof(buf)) {
        if (out.write(buf, sizeof(buf)) != (int)sizeof(buf)) { ok = false; break; }
        moved++;
    }
    out.flush();
    out.close();
    in.close();
    if (!ok || moved == 0) {
        LittleFS.remove(kTmpPath);
        return false;   // live log untouched: the append that follows just overflows
    }
    // The swap. This is the one window in which a power cut loses the surface
    // history, and it is metadata only (no data is copied here); it is also the
    // reason the copy went to a staging file first — an interrupted rewrite of
    // /surface.bin in place would have destroyed the history long before this
    // point. Stated cost of a web-only convenience log: it can never cost a dive.
    LittleFS.remove(kPath);
    if (!LittleFS.rename(kTmpPath, kPath)) {
        LittleFS.remove(kTmpPath);   // nothing better to do; the log is already gone
        return false;
    }
    return true;
}

bool SurfaceStore::append(logfmt::SurfaceRecord& r) {
    const int held = count();
    const int drop = logfmt::compactSurface(held, kMaxRecords);
    if (drop > 0 && !compact(drop))
        Serial.printf("surface: compaction failed, log grows past %d for now\n",
                      kMaxRecords);

    r.seq = lastSeq() + 1u;   // the file is the only durable counter
    if (r.seq > logfmt::SEQ_MAX) r.seq = logfmt::SEQ_MAX;

    File f = LittleFS.open(kPath, "a");
    if (!f) return false;
    uint8_t buf[kStride];
    logfmt::packSurface(r, buf);
    const bool ok = f.write(buf, sizeof(buf)) == (int)sizeof(buf);
    f.flush();      // per record, as for the dive files: a reset costs this row only
    f.close();
    return ok;
}

int SurfaceStore::exportCsv(void (*sink)(const char* line, void* ctx),
                            void* ctx) const {
    uint32_t lo = 0, hi = 0;
    const int n = records(&lo, &hi);
    if (n < 0) return -1;

    char line[192];
    if (logfmt::surfaceCsvHeader(line, sizeof(line), n, lo, hi))
        sink(line, ctx);
    if (n == 0) return 0;

    File f = LittleFS.open(kPath, "r");
    if (!f) return -1;
    int emitted = 0;
    uint8_t buf[kStride];
    logfmt::SurfaceRecord r{};
    while (f.read(buf, sizeof(buf)) == (int)sizeof(buf)) {
        if (!logfmt::unpackSurface(buf, r)) break;
        if (!logfmt::surfaceCsvLine(line, sizeof(line), r)) break;
        sink(line, ctx);
        emitted++;
    }
    f.close();
    return emitted;
}

void SurfaceStore::eraseAll() {
    if (LittleFS.exists(kPath)) LittleFS.remove(kPath);
    if (LittleFS.exists(kTmpPath)) LittleFS.remove(kTmpPath);
}
