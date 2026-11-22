// ============================================================================
// divelog.h — binary dive-file codec + CSV export (pure logic; the LittleFS
// file plumbing is a thin hardware adapter added in M3).
//
// Wire format (little-endian, versioned, CRC16-protected):
//   DiveHeader { magic "RBDV", version, meta..., crc16 }
//   Snapshot   { fixed 32 B, snapshot at dive-grid or event-driven }
//   Event      { fixed 8 B, timestamped machine event (alarm/ack/mode/cal) }
// The 32 B snapshot grid is why ~1 h dives cost ~15 KB total — ten dives
// fit with enormous margin on any flash scheme.
// ============================================================================
#pragma once

#include <cstdint>
#include <cstddef>

namespace logfmt {

constexpr uint32_t DIVE_MAGIC = 0x56444252; // "RBDV" little-endian
constexpr uint8_t DIVE_VERSION = 1;
constexpr size_t SNAPSHOT_SIZE = 32;
constexpr size_t EVENT_SIZE = 8;

// Event ids (handset D,/E, lines use the same numbers).
enum EventId : uint16_t {
    EVT_ALARM_RAISED = 1, EVT_ALARM_CLEARED = 2, EVT_MODE_CHANGE = 3,
    EVT_SP_CHANGE = 4, EVT_CELL_EXCLUDED = 5, EVT_CELL_REJOIN = 6,
    EVT_CAL_AIR = 7, EVT_CAL_O2 = 8, EVT_CELLPROOF_PASS = 9,
    EVT_CELLPROOF_FAIL = 10, EVT_CLAMP_ON = 11, EVT_CLAMP_OFF = 12,
    EVT_PREPASS = 13, EVT_RESET = 14,
};

struct Snapshot {
    uint32_t tSec;       // seconds since dive start
    int32_t depthCm;
    int16_t votedX1000;  // ppO2 in millibar-ish x1000 (0.40 -> 400)
    int16_t cellX1000[3];
    int16_t setpointX1000;
    uint8_t mode;        // 0 standby 1 auto 2 manual 3 safe
    uint8_t alarmBits;   // bit per active alarm
    uint8_t cnsPct;      // capped 255
    int16_t ceilingCm;
    int16_t ndlMin;      // 480 = effectively unlimited
    int16_t decoTotalMin;
    uint16_t pulses;     // injection pulses in the preceding window
    uint16_t battX1000;
    int8_t tempC;
    uint8_t flags;       // b0 sp-clamped, b1..2 voting mode(0 ok,1 2cell,2 single,3 fail)
};

struct Event {
    uint32_t tSec;
    EventId id;
    uint16_t value;
};

struct DiveMeta {
    uint32_t durationSec;
    int32_t maxDepthCm;
    int16_t minVotedX1000, maxVotedX1000;
    uint16_t totalPulses;
    uint8_t cnsPeakPct;
    uint8_t gfLow, gfHigh;         // config in force (percent)
    int16_t sp1X1000, sp2X1000, sp3X1000, sp4X1000;
    uint16_t eventCount;
    uint16_t snapshotCount;
};

// ---- serialization: return bytes written ----
size_t packSnapshot(const Snapshot& s, uint8_t out[SNAPSHOT_SIZE]);
bool unpackSnapshot(const uint8_t in[SNAPSHOT_SIZE], Snapshot& out);

size_t packEvent(const Event& e, uint8_t out[EVENT_SIZE]);
bool unpackEvent(const uint8_t in[EVENT_SIZE], Event& out);

size_t packMeta(const DiveMeta& m, uint8_t out[64]);      // incl. magic+version+crc
bool unpackMeta(const uint8_t in[64], DiveMeta& out);     // false on magic/CRC fail

// ---- CSV export into caller buffer; returns chars written (0 = overflow) ----
size_t toCsv(const DiveMeta& meta,
             const Snapshot* snaps, size_t nSnaps,
             const Event* events, size_t nEvents,
             char* out, size_t outCap);

// ---- CRC16-CCITT (used for record integrity on flash) ----
uint16_t crc16(const uint8_t* data, size_t len);

} // namespace logfmt
