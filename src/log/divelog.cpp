// ============================================================================
// divelog.cpp — see header. Explicit little-endian encoding: the bytes on
// flash must be interpretable from any host tool forever, independent of the
// writing platform's struct layout or alignment.
// ============================================================================
#include "divelog.h"

#include "util/numfmt.h"

#include <cstdarg>
#include <cstdlib>
#include <cstring>
#include <cstdio>

namespace logfmt {

namespace {

void wr16(uint8_t*& p, uint16_t v) { *p++ = v & 0xFF; *p++ = v >> 8; }
void wr32(uint8_t*& p, uint32_t v) { wr16(p, static_cast<uint16_t>(v)); wr16(p, static_cast<uint16_t>(v >> 16)); }
uint16_t rd16(const uint8_t*& p) { const uint16_t v = p[0] | (p[1] << 8); p += 2; return v; }
// Sequenced on purpose, and this one is a live landmine, not a hypothetical:
// `return rd16(p) | (rd16(p) << 16)` has two unsequenced side effects on the same
// scalar, which is undefined behaviour — the compiler may read the HIGH half
// first, and every field after it in the record is then read from the wrong
// offset. It happened to work here because this compiler happens to evaluate the
// left operand first. Found on 2026-11-26 while writing the continuity codec
// (log/continuity.cpp had the same line, and test_continuity's round-trip caught
// it there); fixed in both codecs rather than left as a coincidence.
uint32_t rd32(const uint8_t*& p) { const uint32_t lo = rd16(p); const uint32_t hi = rd16(p); return lo | (hi << 16); }

} // namespace

uint16_t crc16(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int b = 0; b < 8; ++b)
            crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                                 : static_cast<uint16_t>(crc << 1);
    }
    return crc;
}

size_t packSnapshot(const Snapshot& s, uint8_t out[SNAPSHOT_SIZE]) {
    uint8_t* p = out;
    std::memset(out, 0, SNAPSHOT_SIZE);
    *p++ = DIVE_VERSION;
    wr32(p, s.tSec);
    wr32(p, static_cast<uint32_t>(s.depthCm));
    wr16(p, static_cast<uint16_t>(s.votedX1000));
    for (int i = 0; i < 3; ++i) wr16(p, static_cast<uint16_t>(s.cellX1000[i]));
    wr16(p, static_cast<uint16_t>(s.setpointX1000));
    *p++ = s.mode;
    *p++ = static_cast<uint8_t>(s.alarmBits & 0xFF);  // bits 8..9 ride in flags
    *p++ = s.cnsPct;
    wr16(p, static_cast<uint16_t>(s.ceilingCm));
    wr16(p, static_cast<uint16_t>(s.ndlMin));
    wr16(p, static_cast<uint16_t>(s.decoTotalMin));
    wr16(p, s.pulses);
    wr16(p, s.battX1000);
    *p++ = static_cast<uint8_t>(s.tempC);
    // bits 3..4 of flags carry alarm bits 8..9 (CNS_HIGH, HYPOXIA): the 32 B
    // snapshot grid has no spare byte and a dive file that cannot record a
    // hypoxia row is not a dive file.
    *p++ = static_cast<uint8_t>((s.flags & ~kFLAGS_ALARM_HI) |
                                (((s.alarmBits >> 8) << 3) & kFLAGS_ALARM_HI));
    // Per-record CRC: a dive file is written while the diver is underwater, and
    // a reset mid-write must not turn a torn row into plausible garbage. The
    // LittleFS reader stops at the first record that fails this check.
    wr16(p, crc16(out, SNAPSHOT_SIZE - 2));
    return SNAPSHOT_SIZE;
}

bool unpackSnapshot(const uint8_t in[SNAPSHOT_SIZE], Snapshot& out) {
    if (in[0] != DIVE_VERSION) return false;
    if (crc16(in, SNAPSHOT_SIZE - 2) !=
        (uint16_t)(in[SNAPSHOT_SIZE - 2] | (in[SNAPSHOT_SIZE - 1] << 8)))
        return false;  // torn or overwritten row: the reader stops here
    const uint8_t* p = in + 1;
    out.tSec = rd32(p);
    out.depthCm = static_cast<int32_t>(rd32(p));
    out.votedX1000 = static_cast<int16_t>(rd16(p));
    for (int i = 0; i < 3; ++i) out.cellX1000[i] = static_cast<int16_t>(rd16(p));
    out.setpointX1000 = static_cast<int16_t>(rd16(p));
    out.mode = *p++;
    const uint8_t alarmLo = *p++;
    out.cnsPct = *p++;
    out.ceilingCm = static_cast<int16_t>(rd16(p));
    out.ndlMin = static_cast<int16_t>(rd16(p));
    out.decoTotalMin = static_cast<int16_t>(rd16(p));
    out.pulses = rd16(p);
    out.battX1000 = rd16(p);
    out.tempC = static_cast<int8_t>(*p++);
    const uint8_t rawFlags = *p++;
    out.flags = static_cast<uint8_t>(rawFlags & ~kFLAGS_ALARM_HI);
    out.alarmBits = static_cast<uint16_t>(alarmLo |
                                          (((rawFlags & kFLAGS_ALARM_HI) >> 3) << 8));
    return true;
}

size_t packEvent(const Event& e, uint8_t out[EVENT_SIZE]) {
    uint8_t* p = out;
    wr32(p, e.tSec);
    wr16(p, static_cast<uint16_t>(e.id));
    wr16(p, e.value);
    wr16(p, crc16(out, EVENT_SIZE - 2));  // same torn-write rule as snapshots
    return EVENT_SIZE;
}

bool unpackEvent(const uint8_t in[EVENT_SIZE], Event& out) {
    if (crc16(in, EVENT_SIZE - 2) !=
        (uint16_t)(in[EVENT_SIZE - 2] | (in[EVENT_SIZE - 1] << 8)))
        return false;
    const uint8_t* p = in;
    out.tSec = rd32(p);
    out.id = static_cast<EventId>(rd16(p));
    out.value = rd16(p);
    return true;
}

size_t packMeta(const DiveMeta& m, uint8_t out[64]) {
    uint8_t* p = out;
    std::memset(out, 0, 64);
    wr32(p, DIVE_MAGIC);
    *p++ = DIVE_VERSION;
    wr32(p, m.durationSec);
    wr32(p, static_cast<uint32_t>(m.maxDepthCm));
    wr16(p, static_cast<uint16_t>(m.minVotedX1000));
    wr16(p, static_cast<uint16_t>(m.maxVotedX1000));
    wr16(p, m.totalPulses);
    *p++ = m.cnsPeakPct;
    *p++ = m.gfLow;
    *p++ = m.gfHigh;
    wr16(p, static_cast<uint16_t>(m.sp1X1000));
    wr16(p, static_cast<uint16_t>(m.sp2X1000));
    wr16(p, static_cast<uint16_t>(m.sp3X1000));
    wr16(p, static_cast<uint16_t>(m.sp4X1000));
    wr16(p, m.eventCount);
    wr16(p, m.snapshotCount);
    const size_t payloadLen = static_cast<size_t>(p - out);
    wr16(p, crc16(out, payloadLen)); // crc over everything before it
    return 64;
}

bool unpackMeta(const uint8_t in[64], DiveMeta& out) {
    const uint8_t* p = in;
    if (rd32(p) != DIVE_MAGIC) return false;
    if (*p++ != DIVE_VERSION) return false;
    out.durationSec = rd32(p);
    out.maxDepthCm = static_cast<int32_t>(rd32(p));
    out.minVotedX1000 = static_cast<int16_t>(rd16(p));
    out.maxVotedX1000 = static_cast<int16_t>(rd16(p));
    out.totalPulses = rd16(p);
    out.cnsPeakPct = *p++;
    out.gfLow = *p++;
    out.gfHigh = *p++;
    out.sp1X1000 = static_cast<int16_t>(rd16(p));
    out.sp2X1000 = static_cast<int16_t>(rd16(p));
    out.sp3X1000 = static_cast<int16_t>(rd16(p));
    out.sp4X1000 = static_cast<int16_t>(rd16(p));
    out.eventCount = rd16(p);
    out.snapshotCount = rd16(p);
    const size_t payloadLen = static_cast<size_t>(p - in);
    const uint16_t stored = in[payloadLen] | (in[payloadLen + 1] << 8);
    return stored == crc16(in, payloadLen);
}

namespace {
// One row, into the caller's buffer: bytes written including the newline, 0 if
// it would not fit. Every export row on every transport comes out of here, so
// "what the handset is told" and "what the browser downloads" cannot drift.
size_t row(char* out, size_t cap, const char* fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    const int n = std::vsnprintf(out, cap, fmt, ap);
    va_end(ap);
    if (n < 0 || static_cast<size_t>(n) >= cap) return 0;
    return static_cast<size_t>(n);
}
} // namespace

// v1 columns first, in their documented order and units, then the v2 additions
// on the right (docs/HANDSET_PROTOCOL.md 5). No float in any of them: every
// value is either an integer or a scaled integer rendered by fixedScaled, which
// is the point of the X1000 fields in `Snapshot`.
size_t diveCsvLine(char* out, size_t cap, uint8_t slot, uint32_t startEpoch) {
    // Field 6 is the v3 append: the same instant as field 2, readable without a
    // converter. Rendered from the epoch rather than stored, so there is one
    // truth on the flash and a dive written before this change still exports a
    // correct date. Epoch 0 exports `clock-unset`, not 1970-01-01.
    char iso[24];
    if (!isoUtc(iso, sizeof(iso), startEpoch)) return 0;
    return row(out, cap, "DIVE,%u,%u,%u,%u,%s\n", static_cast<unsigned>(slot),
               static_cast<unsigned>(startEpoch), static_cast<unsigned>(DIVE_VERSION),
               static_cast<unsigned>(ROW_FORMAT_VERSION), iso);
}

size_t snapshotCsvLine(char* out, size_t cap, const Snapshot& s) {
    char d[16], v[16], c0[16], c1[16], c2[16], sp[16];
    // v1: t_s depth_m voted c0 c1 c2 sp alarms(hex) cns ceiling_m ndl deco_min
    //     pulses batt(mV)
    // v2 appended: mode temp_c flags ceiling_cm
    // `ceiling_m` stays the integer metres a v1 parser expects; the exact value
    // is the appended `ceiling_cm`, because reformatting an existing column
    // would break a parser that reads it as an integer.
    return row(out, cap,
               "S,%u,%s,%s,%s,%s,%s,%s,0x%04X,%u,%d,%d,%d,%u,%u,%u,%d,%u,%d\n",
               static_cast<unsigned>(s.tSec),
               numfmt::fixedScaled(d, sizeof(d), s.depthCm, 2),
               numfmt::fixedScaled(v, sizeof(v), s.votedX1000, 3),
               numfmt::fixedScaled(c0, sizeof(c0), s.cellX1000[0], 3),
               numfmt::fixedScaled(c1, sizeof(c1), s.cellX1000[1], 3),
               numfmt::fixedScaled(c2, sizeof(c2), s.cellX1000[2], 3),
               numfmt::fixedScaled(sp, sizeof(sp), s.setpointX1000, 3),
               static_cast<unsigned>(s.alarmBits),
               static_cast<unsigned>(s.cnsPct),
               static_cast<int>(s.ceilingCm) / 100,
               static_cast<int>(s.ndlMin), static_cast<int>(s.decoTotalMin),
               static_cast<unsigned>(s.pulses),
               static_cast<unsigned>(s.battX1000),
               static_cast<unsigned>(s.mode),
               static_cast<int>(s.tempC),
               static_cast<unsigned>(s.flags),
               static_cast<int>(s.ceilingCm));
}

size_t eventCsvLine(char* out, size_t cap, const Event& e) {
    return row(out, cap, "E,%u,%u,%u\n", static_cast<unsigned>(e.tSec),
               static_cast<unsigned>(e.id), static_cast<unsigned>(e.value));
}

size_t metaCsvLine(char* out, size_t cap, const DiveMeta& m) {
    char mn[16], mx[16], s1[16], s2[16], s3[16], s4[16];
    // v1: duration_s max_depth_cm min_ppo2 max_ppo2 pulses cns_peak_pct
    //     snapshots events
    // v2 appended: gf_low gf_high sp1 sp2 sp3 sp4
    // The doc used to name field 2 `max_depth_m`; the firmware has always put
    // centimetres there and the web page divides it by 100, so the name was the
    // error and the bytes are unchanged. ppO2 and setpoints are 3 decimals, the
    // same as every other ppO2 column in the file: one format for one quantity.
    return row(out, cap, "M,%u,%d,%s,%s,%u,%u,%u,%u,%u,%u,%s,%s,%s,%s\n",
               static_cast<unsigned>(m.durationSec), static_cast<int>(m.maxDepthCm),
               numfmt::fixedScaled(mn, sizeof(mn), m.minVotedX1000, 3),
               numfmt::fixedScaled(mx, sizeof(mx), m.maxVotedX1000, 3),
               static_cast<unsigned>(m.totalPulses),
               static_cast<unsigned>(m.cnsPeakPct),
               static_cast<unsigned>(m.snapshotCount),
               static_cast<unsigned>(m.eventCount),
               static_cast<unsigned>(m.gfLow), static_cast<unsigned>(m.gfHigh),
               numfmt::fixedScaled(s1, sizeof(s1), m.sp1X1000, 3),
               numfmt::fixedScaled(s2, sizeof(s2), m.sp2X1000, 3),
               numfmt::fixedScaled(s3, sizeof(s3), m.sp3X1000, 3),
               numfmt::fixedScaled(s4, sizeof(s4), m.sp4X1000, 3));
}

namespace {
struct RowSink {
    void (*sink)(const char*, void*);
    void* ctx;
    char line[192];
};

void rowCb(const RawRecord& r, void* vctx) {
    RowSink& o = *static_cast<RowSink*>(vctx);
    size_t n = 0;
    if (r.tag == REC_SNAPSHOT) {
        Snapshot s;
        if (unpackSnapshot(r.bytes, s)) n = snapshotCsvLine(o.line, sizeof(o.line), s);
    } else if (r.tag == REC_EVENT) {
        Event e;
        if (unpackEvent(r.bytes, e)) n = eventCsvLine(o.line, sizeof(o.line), e);
    } else if (r.tag == REC_META) {
        DiveMeta m;
        if (unpackMeta(r.bytes, m)) n = metaCsvLine(o.line, sizeof(o.line), m);
    }
    if (n) o.sink(o.line, o.ctx);
}

struct Appender {
    char* out;
    size_t used;
    size_t cap;
    bool ok;
};

void appendSink(const char* line, void* v) {
    Appender& a = *static_cast<Appender*>(v);
    const size_t n = std::strlen(line);
    if (!a.ok || a.used + n >= a.cap) {   // NUL still has to fit
        a.ok = false;
        return;
    }
    std::memcpy(a.out + a.used, line, n);
    a.used += n;
    a.out[a.used] = '\0';
}
} // namespace

ScanResult emitCsvRows(const uint8_t* img, size_t len,
                       void (*sink)(const char* line, void*), void* ctx) {
    ScanResult sc = scanRecords(img, len, nullptr, nullptr);
    if (!sc.headerOk) return sc;
    uint32_t epoch = 0, seq = 0;
    uint8_t slot = 0;
    unpackHeader(img, epoch, slot, seq);
    char head[64];   // "DIVE,10,4294967295,3,3,9999-12-31T23:59:59Z" is 45 bytes
    if (diveCsvLine(head, sizeof(head), slot, epoch)) sink(head, ctx);
    RowSink rs{sink, ctx, {}};
    return scanRecords(img, len, rowCb, &rs);
}

size_t toCsvImage(const uint8_t* img, size_t len, char* out, size_t outCap) {
    if (outCap == 0) return 0;
    out[0] = '\0';
    Appender a{out, 0, outCap, true};
    emitCsvRows(img, len, appendSink, &a);
    if (!a.ok) {
        // A caller that gets 0 must not be able to read a partial export out of
        // the buffer it handed over: half a dive, printed as if it were one, is
        // worse than an error.
        out[0] = '\0';
        return 0;
    }
    return a.used;
}

size_t packImage(uint8_t* out, size_t cap, uint32_t startEpoch, uint8_t slot,
                 uint32_t seq, const ImageRecord* recs, size_t n,
                 const DiveMeta* meta) {
    size_t need = HEADER_SIZE + (meta ? 1u + 64u : 0u);
    for (size_t i = 0; i < n; ++i) {
        const size_t sz = recs[i].tag == REC_SNAPSHOT ? SNAPSHOT_SIZE
                        : recs[i].tag == REC_EVENT   ? EVENT_SIZE : 0;
        if (sz == 0) return 0;                 // unknown tag: caller error
        need += 1 + sz;
    }
    if (cap < need) return 0;

    uint8_t* p = out;
    uint8_t head[HEADER_SIZE];
    packHeader(startEpoch, slot, seq, head);
    std::memcpy(p, head, HEADER_SIZE);
    p += HEADER_SIZE;
    for (size_t i = 0; i < n; ++i) {
        *p++ = recs[i].tag;
        const size_t sz = recs[i].tag == REC_SNAPSHOT ? SNAPSHOT_SIZE : EVENT_SIZE;
        std::memcpy(p, recs[i].record, sz);
        p += sz;
    }
    if (meta) {
        *p++ = REC_META;
        size_t k = packMeta(*meta, p);
        p += k;
    }
    return static_cast<size_t>(p - out);
}

} // namespace logfmt

namespace logfmt {

void packHeader(uint32_t startEpoch, uint8_t slot, uint32_t seq,
                uint8_t out[HEADER_SIZE]) {
    for (size_t i = 0; i < HEADER_SIZE; ++i) out[i] = 0;
    out[0] = 'R'; out[1] = 'B'; out[2] = 'D'; out[3] = 'V';
    out[4] = DIVE_VERSION;
    // out[5..7]: the dive sequence. Saturating, never wrapping: a counter that
    // came back round to 1 would make the newest dive look like the oldest and
    // get it evicted first, which is the bug this field exists to prevent.
    if (seq > SEQ_MAX) seq = SEQ_MAX;
    out[5] = (uint8_t)(seq);
    out[6] = (uint8_t)(seq >> 8);
    out[7] = (uint8_t)(seq >> 16);
    out[8] = (uint8_t)(startEpoch);
    out[9] = (uint8_t)(startEpoch >> 8);
    out[10] = (uint8_t)(startEpoch >> 16);
    out[11] = (uint8_t)(startEpoch >> 24);
    out[12] = slot;
    // out[13] stays zero: reserved, and inside the CRC, so a field added there
    // later cannot be mistaken for an old zero.
    const uint16_t crc = crc16(out, 14);
    out[14] = (uint8_t)crc;
    out[15] = (uint8_t)(crc >> 8);
}

bool unpackHeader(const uint8_t in[HEADER_SIZE], uint32_t& startEpoch,
                  uint8_t& slot, uint32_t& seq) {
    if (in[0] != 'R' || in[1] != 'B' || in[2] != 'D' || in[3] != 'V') return false;
    if (in[4] != DIVE_VERSION) return false;
    const uint16_t crc = crc16(in, 14);
    if (in[14] != (uint8_t)crc || in[15] != (uint8_t)(crc >> 8)) return false;
    startEpoch = (uint32_t)in[8] | ((uint32_t)in[9] << 8) |
                 ((uint32_t)in[10] << 16) | ((uint32_t)in[11] << 24);
    slot = in[12];
    seq = (uint32_t)in[5] | ((uint32_t)in[6] << 8) | ((uint32_t)in[7] << 16);
    return true;
}

// ---- FIFO ordering identity ------------------------------------------------
// Eviction asks "which of these ten dives is the oldest", and answering it from
// the start epoch needs a clock this board does not have. Until 2026-11-24 every
// dive was stamped epoch 0, all ten keys tied, and the comparison named the same
// slot every time: the eleventh and twelfth dives went into slot 1 one after the
// other, so the log kept the nine oldest dives plus the newest and ate the ones
// in between (measured on the bench, PLAN 16). A sequence number fixes that with
// no clock and no NVS, because `nextSeq` is max(existing)+1 — the order is
// derived from the dive files themselves, so it survives a reboot.

uint32_t nextSeq(const uint32_t* seq, int n) {
    uint32_t hi = 0;
    for (int i = 0; i < n; i++)
        if (seq[i] > hi) hi = seq[i];
    return hi >= SEQ_MAX ? SEQ_MAX : hi + 1;   // empty log: 0 + 1 = 1
}

int pickEvictSlot(const uint32_t* seq, const uint32_t* epoch, int n) {
    if (n <= 0) return -1;
    int best = 0;
    for (int i = 1; i < n; i++) {
        const bool bUnk = seq[best] == SEQ_UNKNOWN, iUnk = seq[i] == SEQ_UNKNOWN;
        if (bUnk != iUnk) {          // unknown age is the one to give up first
            if (iUnk) best = i;
            continue;
        }
        if (seq[i] != seq[best]) { if (seq[i] < seq[best]) best = i; continue; }
        if (epoch && epoch[i] != epoch[best]) {
            if (epoch[i] < epoch[best]) best = i;   // a real clock still wins ties
            continue;
        }
        // Identical keys: keep the lower index, written down rather than left to
        // fall out of the scan order — an implicit scan-order rule is what
        // made the old eviction behaviour invisible until it destroyed a dive.
    }
    return best;
}

// ---- wall clock rendering --------------------------------------------------
namespace {
// Civil date from days since 1970-01-01: Howard Hinnant's closed form. No month
// table, no locale, no TZ, same answer on the host tests and on the target, and
// fixed-point only — `strftime`/`gmtime` would pull newlib's time and float
// formatting into the task that answers clients, which is the stack that has run
// out before (PLAN 16).
void civilFromDays(int64_t z, int& y, int& m, int& d) {
    z += 719468;
    const int64_t era = (z >= 0 ? z : z - 146096) / 146097;
    const unsigned doe = (unsigned)(z - era * 146097);
    const unsigned yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    const int64_t yy = (int64_t)yoe + era * 400;
    const unsigned doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    const unsigned mp = (5 * doy + 2) / 153;
    d = (int)(doy - (153 * mp + 2) / 5 + 1);
    // mp counts months from March, so Nov/Dec are mp 8/9 shifted by -9, not +3.
    // Getting this wrong is not loud: 1970-01-15 renders as 1969-11-15, a date
    // that looks plausible in a dive log. Caught by the host tests.
    m = (int)(mp + (mp < 10 ? 3 : -9));
    y = (int)(yy + (m <= 2));
}
}  // namespace

size_t isoUtc(char* out, size_t cap, uint32_t epoch) {
    if (epoch == 0) {   // no clock: say so where a reader expects a date
        const size_t n = std::strlen(kNoClock);
        if (cap < n + 1) return 0;
        std::memcpy(out, kNoClock, n + 1);
        return n;
    }
    if (cap < 21) return 0;
    int y, mo, d;
    civilFromDays((int64_t)(epoch / 86400u), y, mo, d);
    unsigned sod = epoch % 86400u;
    const int h = (int)(sod / 3600u); sod %= 3600u;
    const int mi = (int)(sod / 60u), s = (int)(sod % 60u);
    return (size_t)std::snprintf(out, cap, "%04d-%02d-%02dT%02d:%02d:%02dZ",
                                 y, mo, d, h, mi, s);
}

ScanResult scanRecords(const uint8_t* img, size_t len,
                       void (*cb)(const RawRecord&, void*), void* ctx) {
    ScanResult res;
    res.bytesConsumed = 0;
    if (len < HEADER_SIZE) return res;
    uint32_t epoch = 0, seq = 0;
    uint8_t slot = 0;
    if (!unpackHeader(img, epoch, slot, seq)) return res;
    res.headerOk = true;
    res.startEpoch = epoch;
    res.seq = seq;
    res.bytesConsumed = HEADER_SIZE;

    size_t p = HEADER_SIZE;
    while (p + 1 <= len) {
        const uint8_t tag = img[p];
        size_t sz = 0;
        if (tag == REC_SNAPSHOT) sz = SNAPSHOT_SIZE;
        else if (tag == REC_EVENT) sz = EVENT_SIZE;
        else if (tag == REC_META) sz = 64;
        else break;                       // unknown tag: torn/foreign, stop
        if (p + 1 + sz > len) break;      // truncated tail: stop
        const uint8_t* rec = img + p + 1;
        bool ok = false;
        if (tag == REC_SNAPSHOT) {
            Snapshot s;
            ok = unpackSnapshot(rec, s);
            if (ok) res.snapshots++;
        } else if (tag == REC_EVENT) {
            Event e;
            ok = unpackEvent(rec, e);
            if (ok) res.events++;
        } else {
            DiveMeta m;
            ok = unpackMeta(rec, m);
            if (ok) res.metaOk = true;
        }
        if (!ok) break;                   // CRC fail: stop, keep what came before
        if (cb) {
            const RawRecord r{tag, rec, sz};
            cb(r, ctx);
        }
        p += 1 + sz;
        res.bytesConsumed = p;
        if (tag == REC_META) break;       // trailer terminates the dive
    }
    return res;
}

// ---- the surface log -------------------------------------------------------
namespace {
void wr24(uint8_t*& p, uint32_t v) {
    *p++ = static_cast<uint8_t>(v);
    *p++ = static_cast<uint8_t>(v >> 8);
    *p++ = static_cast<uint8_t>(v >> 16);
}
uint32_t rd24(const uint8_t*& p) {
    const uint32_t v = p[0] | (static_cast<uint32_t>(p[1]) << 8) |
                       (static_cast<uint32_t>(p[2]) << 16);
    p += 3;
    return v;
}
} // namespace

size_t packSurface(const SurfaceRecord& r, uint8_t out[SURFACE_RECORD_SIZE]) {
    uint8_t* p = out;
    std::memset(out, 0, SURFACE_RECORD_SIZE);
    *p++ = SURFACE_REC_VERSION;
    wr32(p, r.epoch);
    // 3 bytes, like the dive header's seq: SEQ_MAX is one definition (0x00FFFFF F)
    // and a fourth byte would be a stride change for a number that will saturate
    // long before it needs one. A compaction never renumbers, so the seq a reader
    // sees here is the one the record was born with.
    wr24(p, r.seq);
    wr16(p, r.id);
    wr16(p, r.value);
    for (int i = 0; i < 3; ++i) wr16(p, static_cast<uint16_t>(r.raw[i]));
    wr16(p, r.spread);
    *p++ = r.samples;
    *p++ = r.flags;
    // Same torn-write rule as every other record on this partition: the CRC is
    // per record, so a reset between two writes costs that record and leaves the
    // reader a clean place to stop.
    wr16(p, crc16(out, SURFACE_RECORD_SIZE - 2));
    return SURFACE_RECORD_SIZE;
}

bool unpackSurface(const uint8_t in[SURFACE_RECORD_SIZE], SurfaceRecord& out) {
    if (crc16(in, SURFACE_RECORD_SIZE - 2) !=
        (uint16_t)(in[SURFACE_RECORD_SIZE - 2] |
                   (in[SURFACE_RECORD_SIZE - 1] << 8)))
        return false;
    if (in[0] != SURFACE_REC_VERSION) return false;  // unknown layout: stop, do not guess
    const uint8_t* p = in + 1;
    out.epoch = rd32(p);
    out.seq = rd24(p);
    out.id = rd16(p);
    out.value = rd16(p);
    for (int i = 0; i < 3; ++i) out.raw[i] = static_cast<int16_t>(rd16(p));
    out.spread = rd16(p);
    out.samples = *p++;
    out.flags = *p++;
    return true;
}

size_t surfaceName(char* out, size_t cap, uint16_t id) {
    // Names are for the CSV a human opens; the id stays in its own column, so a
    // rename here could never break a parser. `id<n>` for anything this build
    // does not know: an appended id must not become a row without a label.
    const char* n = nullptr;
    switch (id) {
        case EVT_CAL_AIR:  n = "cal_air"; break;
        case EVT_PREPASS:  n = "pre_dive_pass"; break;
        case EVT_PREFAIL:  n = "pre_dive_fail"; break;
        case EVT_CONT_RESTORED: n = "continuity_restored"; break;
        case EVT_CONT_LOST:     n = "continuity_lost"; break;
        default: break;
    }
    if (n) {
        const size_t l = std::strlen(n);
        if (cap < l + 1) return 0;
        std::memcpy(out, n, l + 1);
        return l;
    }
    const int w = std::snprintf(out, cap, "id%u", static_cast<unsigned>(id));
    return (w < 0 || static_cast<size_t>(w) >= cap) ? 0 : static_cast<size_t>(w);
}

size_t surfaceCsvHeader(char* out, size_t cap, int count, uint32_t seqLo,
                        uint32_t seqHi) {
    return row(out, cap, "SURFACE,%u,%d,%u,%u\n",
               static_cast<unsigned>(SURFACE_ROW_FORMAT_VERSION), count,
               static_cast<unsigned>(seqLo), static_cast<unsigned>(seqHi));
}

// Columns, append-only: seq, epoch, ISO date (or `clock-unset`), id, name,
// value, raw0..2, spread, samples, flags. `flags` is there because a date that
// was carried across a reset is not a verified date, and a diver reading this
// file months later cannot otherwise tell the two apart.
size_t surfaceCsvLine(char* out, size_t cap, const SurfaceRecord& r) {
    char iso[24], name[24];
    if (!isoUtc(iso, sizeof(iso), r.epoch)) return 0;
    if (!surfaceName(name, sizeof(name), r.id)) return 0;
    return row(out, cap, "R,%u,%u,%s,%u,%s,%u,%d,%d,%d,%u,%u,%u\n",
               static_cast<unsigned>(r.seq), static_cast<unsigned>(r.epoch), iso,
               static_cast<unsigned>(r.id), name, static_cast<unsigned>(r.value),
               static_cast<int>(r.raw[0]), static_cast<int>(r.raw[1]),
               static_cast<int>(r.raw[2]), static_cast<unsigned>(r.spread),
               static_cast<unsigned>(r.samples), static_cast<unsigned>(r.flags));
}

int compactSurface(int n, int capacity) {
    if (capacity <= 0 || n < capacity) return 0;   // room: rewrite nothing
    return n - capacity + 1;                      // oldest goes first, newest stays
}

bool countsFromFloat(double v, int16_t& out) {
    char b[24];
    numfmt::fixed(b, sizeof(b), v, 0);
    if (b[0] == '\0') return false;        // nan / inf / unrepresentable: no number
    char* end = nullptr;
    const long l = std::strtol(b, &end, 10);
    if (end == b || (end && *end != '\0')) return false;   // "inf" or anything odd
    if (l < -32768L || l > 32767L) return false;           // will not fit the column
    out = static_cast<int16_t>(l);
    return true;
}

} // namespace logfmt
