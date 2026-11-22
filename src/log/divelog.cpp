// ============================================================================
// divelog.cpp — see header. Explicit little-endian encoding: the bytes on
// flash must be interpretable from any host tool forever, independent of the
// writing platform's struct layout or alignment.
// ============================================================================
#include "divelog.h"

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
    *p++ = s.alarmBits;
    *p++ = s.cnsPct;
    wr16(p, static_cast<uint16_t>(s.ceilingCm));
    wr16(p, static_cast<uint16_t>(s.ndlMin));
    wr16(p, static_cast<uint16_t>(s.decoTotalMin));
    wr16(p, s.pulses);
    wr16(p, s.battX1000);
    *p++ = static_cast<uint8_t>(s.tempC);
    *p++ = s.flags;
    return SNAPSHOT_SIZE;
}

bool unpackSnapshot(const uint8_t in[SNAPSHOT_SIZE], Snapshot& out) {
    if (in[0] != DIVE_VERSION) return false;
    const uint8_t* p = in + 1;
    out.tSec = rd32(p);
    out.depthCm = static_cast<int32_t>(rd32(p));
    out.votedX1000 = static_cast<int16_t>(rd16(p));
    for (int i = 0; i < 3; ++i) out.cellX1000[i] = static_cast<int16_t>(rd16(p));
    out.setpointX1000 = static_cast<int16_t>(rd16(p));
    out.mode = *p++;
    out.alarmBits = *p++;
    out.cnsPct = *p++;
    out.ceilingCm = static_cast<int16_t>(rd16(p));
    out.ndlMin = static_cast<int16_t>(rd16(p));
    out.decoTotalMin = static_cast<int16_t>(rd16(p));
    out.pulses = rd16(p);
    out.battX1000 = rd16(p);
    out.tempC = static_cast<int8_t>(*p++);
    out.flags = *p++;
    return true;
}

size_t packEvent(const Event& e, uint8_t out[EVENT_SIZE]) {
    uint8_t* p = out;
    wr32(p, e.tSec);
    wr16(p, static_cast<uint16_t>(e.id));
    wr16(p, e.value);
    return EVENT_SIZE;
}

bool unpackEvent(const uint8_t in[EVENT_SIZE], Event& out) {
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
// snprintf into the remaining buffer; returns false when it would overflow.
bool emit(char*& out, size_t& remaining, const char* fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    const int n = std::vsnprintf(out, remaining, fmt, ap);
    va_end(ap);
    if (n < 0 || static_cast<size_t>(n) >= remaining) return false;
    out += n;
    remaining -= static_cast<size_t>(n);
    return true;
}
} // namespace

size_t toCsv(const DiveMeta& meta,
             const Snapshot* snaps, size_t nSnaps,
             const Event* events, size_t nEvents,
             char* out, size_t outCap) {
    char* p = out;
    size_t rem = outCap;
    if (!emit(p, rem, "ESP-CCR dive record,%u\n", DIVE_VERSION)) return 0;
    if (!emit(p, rem, "META,duration_s,max_depth_m,min_ppo2,max_ppo2,total_pulses,"
                      "cns_peak,gf_low,gf_high,sp1,sp2,sp3,sp4,snapshots,events\n")) return 0;
    if (!emit(p, rem, "META,%u,%.2f,%.3f,%.3f,%u,%u,%u,%u,%.2f,%.2f,%.2f,%.2f,%u,%u\n",
              meta.durationSec, meta.maxDepthCm / 100.0, meta.minVotedX1000 / 1000.0,
              meta.maxVotedX1000 / 1000.0, meta.totalPulses, meta.cnsPeakPct,
              meta.gfLow, meta.gfHigh,
              meta.sp1X1000 / 1000.0, meta.sp2X1000 / 1000.0,
              meta.sp3X1000 / 1000.0, meta.sp4X1000 / 1000.0,
              meta.snapshotCount, meta.eventCount)) return 0;
    if (!emit(p, rem, "S,t_s,depth_m,ppo2,cell0,cell1,cell2,setpoint,mode,alarms,"
                      "cns,ceiling_m,ndl_min,deco_min,pulses,batt_v,temp_c,flags\n")) return 0;
    for (size_t i = 0; i < nSnaps; ++i) {
        const Snapshot& s = snaps[i];
        if (!emit(p, rem, "S,%u,%.2f,%.3f,%.3f,%.3f,%.3f,%.3f,%u,%u,%u,%.2f,%d,%d,%u,%.2f,%d,%u\n",
                  s.tSec, s.depthCm / 100.0, s.votedX1000 / 1000.0,
                  s.cellX1000[0] / 1000.0, s.cellX1000[1] / 1000.0, s.cellX1000[2] / 1000.0,
                  s.setpointX1000 / 1000.0, s.mode, s.alarmBits, s.cnsPct,
                  s.ceilingCm / 100.0, s.ndlMin, s.decoTotalMin, s.pulses,
                  s.battX1000 / 1000.0, s.tempC, s.flags)) return 0;
    }
    for (size_t i = 0; i < nEvents; ++i) {
        if (!emit(p, rem, "E,%u,%u,%u\n", events[i].tSec,
                  static_cast<unsigned>(events[i].id), events[i].value)) return 0;
    }
    return static_cast<size_t>(p - out);
}

} // namespace logfmt
