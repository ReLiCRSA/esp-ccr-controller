// ============================================================================
// divelog.cpp — see header. Explicit little-endian encoding: the bytes on
// flash must be interpretable from any host tool forever, independent of the
// writing platform's struct layout or alignment.
// ============================================================================
#include "divelog.h"

#include "util/numfmt.h"

#include <cstdarg>
#include <cstring>
#include <cstdio>

namespace logfmt {

namespace {

void wr16(uint8_t*& p, uint16_t v) { *p++ = v & 0xFF; *p++ = v >> 8; }
void wr32(uint8_t*& p, uint32_t v) { wr16(p, static_cast<uint16_t>(v)); wr16(p, static_cast<uint16_t>(v >> 16)); }
uint16_t rd16(const uint8_t*& p) { const uint16_t v = p[0] | (p[1] << 8); p += 2; return v; }
uint32_t rd32(const uint8_t*& p) { return rd16(p) | (static_cast<uint32_t>(rd16(p)) << 16); }

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
    return row(out, cap, "DIVE,%u,%u,%u,%u\n", static_cast<unsigned>(slot),
               static_cast<unsigned>(startEpoch), static_cast<unsigned>(DIVE_VERSION),
               static_cast<unsigned>(ROW_FORMAT_VERSION));
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
    uint32_t epoch = 0;
    uint8_t slot = 0;
    unpackHeader(img, epoch, slot);
    char head[48];
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
                 const ImageRecord* recs, size_t n, const DiveMeta* meta) {
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
    packHeader(startEpoch, slot, head);
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

void packHeader(uint32_t startEpoch, uint8_t slot, uint8_t out[HEADER_SIZE]) {
    for (size_t i = 0; i < HEADER_SIZE; ++i) out[i] = 0;
    out[0] = 'R'; out[1] = 'B'; out[2] = 'D'; out[3] = 'V';
    out[4] = DIVE_VERSION;
    out[8] = (uint8_t)(startEpoch);
    out[9] = (uint8_t)(startEpoch >> 8);
    out[10] = (uint8_t)(startEpoch >> 16);
    out[11] = (uint8_t)(startEpoch >> 24);
    out[12] = slot;
    const uint16_t crc = crc16(out, 14);
    out[14] = (uint8_t)crc;
    out[15] = (uint8_t)(crc >> 8);
}

bool unpackHeader(const uint8_t in[HEADER_SIZE], uint32_t& startEpoch, uint8_t& slot) {
    if (in[0] != 'R' || in[1] != 'B' || in[2] != 'D' || in[3] != 'V') return false;
    if (in[4] != DIVE_VERSION) return false;
    const uint16_t crc = crc16(in, 14);
    if (in[14] != (uint8_t)crc || in[15] != (uint8_t)(crc >> 8)) return false;
    startEpoch = (uint32_t)in[8] | ((uint32_t)in[9] << 8) |
                 ((uint32_t)in[10] << 16) | ((uint32_t)in[11] << 24);
    slot = in[12];
    return true;
}

ScanResult scanRecords(const uint8_t* img, size_t len,
                       void (*cb)(const RawRecord&, void*), void* ctx) {
    ScanResult res;
    res.bytesConsumed = 0;
    if (len < HEADER_SIZE) return res;
    uint32_t epoch = 0;
    uint8_t slot = 0;
    if (!unpackHeader(img, epoch, slot)) return res;
    res.headerOk = true;
    res.startEpoch = epoch;
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

} // namespace logfmt
