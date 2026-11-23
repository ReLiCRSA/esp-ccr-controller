// ============================================================================
// divelog.h — binary dive-file codec + CSV export (pure logic; the LittleFS
// file plumbing is a thin hardware adapter added in M3).
//
// Wire format (little-endian, versioned, CRC16-protected):
//   DiveHeader { magic "RBDV", version, meta..., crc16 }
//   Snapshot   { fixed 36 B, snapshot at dive-grid or event-driven }
//   Event      { fixed 10 B, timestamped machine event (alarm/ack/mode/cal) }
// Every record carries its own CRC16, not just the header: the file is written
// while the diver is underwater, so a reset mid-write must leave the reader a
// place to stop instead of a row of plausible garbage.
// The fixed grid is why ~1 h dives cost ~15 KB total — ten dives fit with
// enormous margin on any flash scheme.
// ============================================================================
#pragma once

#include <cstdint>
#include <cstddef>

namespace logfmt {

constexpr uint32_t DIVE_MAGIC = 0x56444252; // "RBDV" little-endian
// v2: the snapshot alarm mask widened to 10 bits (see Snapshot::alarmBits).
// v1 files never left the bench, and a v1 decoder must not misread v2 flags.
// v3: on-disk records are TYPE-TAGGED (one byte before each packed record:
// 'S' snapshot, 'E' event, 'M' meta trailer). Snapshots and events are
// interleaved chronologically by the writer; without a tag, a reader cannot
// tell a 36-byte row from a 10-byte row without guessing on CRC collisions,
// and one wrong guess desynchronises every row after it. Records produced
// by pack* are unchanged — the tag belongs to the file, not the codec.
constexpr uint8_t DIVE_VERSION = 3;
// The FIFO depth, in one place: the file store, the command parser's index
// range and the web's /dive/<n>.csv routes all have to agree on it, and two
// of them are host-compiled while the store is not, so the constant lives
// here where both can see it.
constexpr uint8_t MAX_DIVE_SLOTS = 10;
constexpr uint8_t REC_SNAPSHOT = 'S';
constexpr uint8_t REC_EVENT = 'E';
constexpr uint8_t REC_META = 'M';
constexpr size_t SNAPSHOT_SIZE = 36;  // 34 B of fields + CRC16, see below
constexpr size_t EVENT_SIZE = 10;     // 8 B of fields + CRC16
// flags bits 3..4 are the wire home for alarm bits 8..9 (see Snapshot).
constexpr uint8_t kFLAGS_ALARM_HI = 0x18;

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
    // The alarm set is protocol-visible and has 10 bits (BIT_LOW_PPO2 ..
    // BIT_HYPOXIA); the snapshot grid has no spare byte, so the wire carries
    // bits 0..7 inline and borrows bits 3..4 of `flags` for 8..9. The struct
    // keeps the full mask and unpack clears the borrowed bits out of `flags`,
    // so a hypoxia or CNS row in a dive file is never silently lost.
    uint16_t alarmBits;
    uint8_t cnsPct;      // capped 255
    int16_t ceilingCm;
    int16_t ndlMin;      // 480 = effectively unlimited
    int16_t decoTotalMin;
    uint16_t pulses;     // injection pulses in the preceding window
    uint16_t battX1000;
    int8_t tempC;
    uint8_t flags;       // b0 sp-clamped, b1..2 voting mode(0 ok,1 2cell,2 single,3 fail)
                         // (b3..4 are used on the wire only, never by callers)
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

// ---- file header: fixed 16 B at the front of every dive file ---------------
constexpr size_t HEADER_SIZE = 16;
void packHeader(uint32_t startEpoch, uint8_t slot, uint8_t out[HEADER_SIZE]);
bool unpackHeader(const uint8_t in[HEADER_SIZE], uint32_t& startEpoch, uint8_t& slot);


// ---- CRC16-CCITT (used for record integrity on flash) ----
uint16_t crc16(const uint8_t* data, size_t len);

// ---- tagged record stream (the file layout; see DIVE_VERSION note) ---------
// Pure walk over the bytes of a dive file: fixed 16 B header, then
// tag+record pairs, meta trailer last. Parsing stops at the first tag that
// is unknown, the first record that fails its CRC, and nothing after that
// is reported — that is the contract between a mid-write reset and the
// diver reading the log later. Kept free of any file API so the host tests
// can feed it a byte image directly.
struct RawRecord {
    uint8_t tag;               // REC_SNAPSHOT / REC_EVENT / REC_META
    const uint8_t* bytes;      // packed record (codec-sized, tag excluded)
    size_t size;
};

struct ScanResult {
    bool headerOk = false;
    uint32_t startEpoch = 0;
    int snapshots = 0;
    int events = 0;
    bool metaOk = false;       // a complete dive ends with a valid trailer
    size_t bytesConsumed;      // header + every accepted record
};

ScanResult scanRecords(const uint8_t* img, size_t len,
                       void (*cb)(const RawRecord&, void*), void* ctx);

// ---- CSV export rows: ONE canonical formatter --------------------------------
// `LOG EXPORT <n>` on the handset and `GET /dive/<n>.csv` on the web are the
// same rows, produced by the functions below and by nothing else. Until
// 2026-11-23 there were two writers and they disagreed (14 vs 17 fields, alarms
// hex vs decimal, ceiling m vs cm, battery mV vs V) — BENCH.md C2 step 4 asserts
// the two agree, and the check was passing only because nothing compared them.
//
// Columns are APPEND-ONLY (docs/HANDSET_PROTOCOL.md 5): never renumber, reorder,
// remove or re-unit an existing column. A v1 parser reading the first N fields
// of a v2 row gets exactly what it got in v1; new information goes on the right.
// `ROW_FORMAT_VERSION` rides on the `DIVE,` line so a reader can tell how many
// columns to expect without counting.
constexpr uint8_t ROW_FORMAT_VERSION = 2;

size_t diveCsvLine(char* out, size_t cap, uint8_t slot, uint32_t startEpoch);
size_t snapshotCsvLine(char* out, size_t cap, const Snapshot& s);
size_t eventCsvLine(char* out, size_t cap, const Event& e);
size_t metaCsvLine(char* out, size_t cap, const DiveMeta& m);

// Streams the rows of a whole dive-file image (the bytes a diveN.bin holds)
// through `sink`, header line first, in file order. This is what the handset and
// the web actually send. Returns the scan, so the caller can report row counts
// and whether the dive was complete.
ScanResult emitCsvRows(const uint8_t* img, size_t len,
                       void (*sink)(const char* line, void*), void* ctx);

// The same walk, buffered into the caller's buffer. 0 on overflow.
size_t toCsvImage(const uint8_t* img, size_t len, char* out, size_t outCap);

// Builds a complete dive-file image in memory: 16 B header, tag+record pairs in
// the order given (the store interleaves snapshots and events chronologically),
// meta trailer last. The store writes the same bytes incrementally; this exists
// so the reader's contract can be tested against a byte image directly.
struct ImageRecord {
    uint8_t tag;         // REC_SNAPSHOT / REC_EVENT
    const void* record;  // packed Snapshot or Event, tag excluded
};
size_t packImage(uint8_t* out, size_t cap, uint32_t startEpoch, uint8_t slot,
                 const ImageRecord* recs, size_t n, const DiveMeta* meta);

} // namespace logfmt
