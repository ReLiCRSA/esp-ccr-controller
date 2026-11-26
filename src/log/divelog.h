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
// The version did NOT move when the four unused header bytes gained a meaning
// (the dive sequence, below): the layout, size and CRC coverage of the header
// are unchanged, a reader that ignores those bytes still decodes epoch and slot
// correctly, and bumping the number would have made every dive already on a
// board's flash unreadable by the firmware that replaced it. `unpackHeader`
// reads the sequence out of files written before the change as 0 = unknown.
// SEQ_MAX is 24 bits because that is what the three contiguous spare bytes
// (out[5..7]) hold; out[13] stays padding. At one dive a day that is 45—odd
// years, and the counter saturates rather than wrapping, so an overflow cannot
// make a new dive look older than an existing one.
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
    // 15: this dive was started with no wall clock, so its header epoch is 0 and
    // its exported date reads `clock-unset`. It belongs in the dive file rather
    // than only on the console: the dive log is the artifact a diver reads later,
    // and "this dive has no date" has to travel with it. Event ids are
    // append-only, so this is a new number, never a re-used one.
    EVT_CLOCK_UNSET = 15,
    // 16: a pre-dive check FAILED, `value` being the step that failed (the same
    // order as the five `PRE,` fields: 0 warm-up, 1 air, 2 solenoid, 3 depth,
    // 4 battery). It is the twin of 13, and both live only in the surface log
    // (see SurfaceRecord): a pre-dive result is a fact about the machine at the
    // surface, so an event row inside a dive file cannot speak for it — the dive
    // that follows a failure never happens, and the check that failed is the
    // thing a diver wants written down. Adding 16 rather than overloading 13's
    // value is the append-only rule doing its job: a client that knows 13 still
    // reads a pass correctly, and one that does not know 16 loses one row rather
    // than misreading a pass as a failure.
    EVT_PREFAIL = 16,
    // 17 and 18: the two halves of dive-start continuity (review H1), and both
    // are SURFACE records for the same reason 13/16 are: they are facts about the
    // machine between dives, and the dive they describe may be the one whose file
    // never gets written. `value` is id-specific, as everywhere in this log:
    //   17 EVT_CONT_RESTORED — a dive was seeded from a stored record; value is
    //      the surface interval credited, in seconds, saturated at 65535 (0 here
    //      means "seeded with NO credit", which is its own fact, and the ISO date
    //      column plus the continuity record's own seq are how a reader checks it).
    //   18 EVT_CONT_LOST — a dive started from clean tissues when continuity was
    //      the whole point; value is a logfmt::ContLoss reason (continuity.h).
    // This is the row that exists for the scenario the reviewer named: a diver
    // who was obligated, surfaced, and whose machine then forgot. "The dive starts
    // fresh" is a fact the machine must keep, because it is exactly the moment the
    // diver was owed a ceiling and was not given one.
    EVT_CONT_RESTORED = 17,
    EVT_CONT_LOST = 18,
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
    // What goes in it is the LOG view (alarms::Result::logBits): the conditions
    // that were TRUE at this instant, including a critical the diver has ACKed
    // into silence. Not the live `T,` FLAGS word — an ACK must stay able to
    // silence the handset without editing the record afterwards.
    uint16_t alarmBits;
    uint8_t cnsPct;      // capped 255
    int16_t ceilingCm;
    // The ndl column's sentinel pair is ONE convention for the file and the
    // wire (review N5, closed 2026-11-28): deco::kNDL_UNKNOWN (-1, "no valid
    // vote, no number" — the same value `D,` has answered since v1) and
    // deco::kNDL_UNLIMITED (480, the engine's saturation). Before that date
    // the file wrote 480 for BOTH facts, so an unlimited NDL and an
    // unanswerable one read alike beside each other.
    int16_t ndlMin;
    int16_t decoTotalMin;
    uint16_t pulses;     // injection pulses in the preceding window
    uint16_t battX1000;
    int8_t tempC;
    // b0 sp-clamped, b1..2 voting mode(0 ok,1 2cell,2 single,3 fail) — WRITTEN
    // since 2026-11-28 (review N3: a documented column that always answered
    // the healthy value was worse than an absent one). The mode is the same
    // voting::Mode the wire and the held-ppO2 rule speak: a row whose vote
    // FAILED says FAILED here, and its ppO2 column says the held number.
    // (b3..4 are used on the wire only, never by callers.)
    uint8_t flags;
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
// magic 'RBDV'(4) version(1) seq(3) startEpoch(4) slot(1) pad(1) crc16(2),
// the CRC covering the first 14 bytes.
constexpr size_t HEADER_SIZE = 16;

/// The dive's position in the order dives were written. `0` means "unknown":
/// every file written before 2026-11-24, and any file whose header could not be
/// read. See `nextSeq`/`pickEvictSlot` for why the FIFO needs this at all.
constexpr uint32_t SEQ_UNKNOWN = 0u;
constexpr uint32_t SEQ_MAX = 0x00FFFFFFu;

void packHeader(uint32_t startEpoch, uint8_t slot, uint32_t seq,
                uint8_t out[HEADER_SIZE]);
bool unpackHeader(const uint8_t in[HEADER_SIZE], uint32_t& startEpoch,
                  uint8_t& slot, uint32_t& seq);

// ---- FIFO ordering identity (pure; the file walking stays in dive_store) ----
// Eviction asks "which of these ten dives is the oldest?". Answering it from the
// start epoch needs a clock, and this board has none: no RTC chip, and no NTP
// through an access point with no uplink, so until 2026-11-24 every dive was
// stamped epoch 0, all ten keys tied, and the comparison named the same slot
// every time. On the bench that put the eleventh and twelfth dives in slot 1
// one after the other: the log kept the nine oldest dives and the very newest
// and silently ate everything in between (BENCH.md, PLAN 16).
// A sequence number fixes it without a clock: `nextSeq` is max(existing)+1, so
// it is monotonic across reboots — it is derived from the dive files
// themselves, not from NVS or a timer — and it costs three header bytes
// that were padding.
//
/// Sequence for the next dive: one past the highest on the partition, 1 for an
/// empty log, saturating at SEQ_MAX (a saturated counter stops ordering, which
/// is a 16.7-million-dive problem and is stated rather than hidden).
uint32_t nextSeq(const uint32_t* seq, int n);

/// Index (0-based, so slot = index + 1) of the dive to evict, or -1 for n <= 0.
/// Unknown-age dives go first: they predate every sequenced one, so a board
/// upgrading into this format sheds its pre-change dives before it sheds any
/// others. Among sequenced dives, smallest seq; then smallest epoch (a real
/// clock's ordering, which is still the right answer when two dives somehow
/// share a seq); then the lower slot, so the choice is reproducible rather than
/// whatever the scan order happened to be.
int pickEvictSlot(const uint32_t* seq, const uint32_t* epoch, int n);

// ---- wall clock rendering (pure, no libc time, no locale, no TZ) -----------
/// "2026-11-24T09:00:00Z" from a unix epoch, UTC, fixed 20 characters. Returns
/// the length written, or 0 if `cap` is too small. Epoch 0 (no clock) renders
/// `kNoClock`, so a dive exported without a clock says so in the field where a
/// reader expects a date instead of showing 1970-01-01.
constexpr const char* kNoClock = "clock-unset";
size_t isoUtc(char* out, size_t cap, uint32_t epoch);


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
    uint32_t seq = SEQ_UNKNOWN;
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
// v3 appends one column to the right of the `DIVE,` identity line: the dive
// start as ISO-8601 UTC. Nothing existing moved or changed units, so a v1/v2
// parser reading the first five fields still gets exactly what it got before.
constexpr uint8_t ROW_FORMAT_VERSION = 3;

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
                 uint32_t seq, const ImageRecord* recs, size_t n,
                 const DiveMeta* meta);

// ---- the surface log: fixed-stride records of what happened at the surface --
// A calibration verdict and a pre-dive result are facts about the machine, not
// about a dive, so they cannot live in a dive file: the dive that a failed
// pre-dive check prevents never gets a file to hold the row. Until 2026-11-24
// the only durable record of "the cells were refused for warm-up" or "the
// pre-dive failed on battery" was the console stream, which nobody keeps.
//
// One file, /surface.bin, a flat stream of fixed 24 B records, each carrying its
// own version byte and CRC16 — the same torn-write contract as the dive files: a
// reader stops at the first record that fails, so a reset mid-write costs that
// record and not the log. No file header on purpose: with one flat stream there
// is no per-file identity to carry, and a header would be the one place where a
// torn first write could lose everything.
//
// THIS LOG IS WEB/HTTP ONLY (PLAN 16, decided 2026-11-24): it has no handset
// commands. That is a stated cost, not an oversight — a wired handset, the
// in-water transport, cannot read its own machine's history; it can only be
// listening when the event happens (`E,*,7,<code>`). The reason is the
// append-only rule: `LOG LIST`/`LOG EXPORT` already mean "the dive log", their
// `L,` row shape is already published, and bolting a second class of record onto
// them would re-validate every existing client's parser for a shape it already
// consumes. A second command pair (`SLOG LIST`/`SLOG EXPORT`) is what a handset
// would need, and it is not built yet; when it is, it lands as spec + client
// library + example firmware + README + conformance suite in one commit.
constexpr uint8_t SURFACE_REC_VERSION = 1;
constexpr size_t SURFACE_RECORD_SIZE = 24;

// Capacity. 256 records x 24 B = 6 KB, which at a couple of records per power-up
// is months of surface history, and the eviction policy is the dive log's:
// oldest goes first. Full is not an error — see compactSurface.
constexpr int SURFACE_MAX_RECORDS = 256;

struct SurfaceRecord {
    uint32_t epoch;    // wall clock at the record; 0 = no clock, renders kNoClock
    uint32_t seq;      // 1..SEQ_MAX, monotonic across boots, derived from the file
    uint16_t id;       // logfmt::EventId (7 cal air, 13 pre-dive pass, 16 fail,
                       // 17 continuity restored, 18 continuity lost)
    uint16_t value;    // id-specific: 7 = bridge::CalAirCode, 16 = failing Step,
                       // 17 = surface interval credited (s, saturated),
                       // 18 = logfmt::ContLoss reason
    // Raw ADC counts, the numbers that tell a dead cell from a fouled one. They
    // are integers here because the protocol already publishes them as integers
    // (`CALAIR,` prints 0 decimals); storing a float would imply a precision the
    // wire never carried. Zero for records that are not about the cells.
    int16_t raw[3];
    uint16_t spread;   // worst per-cell spread across the window, raw counts
    uint8_t samples;   // samples in the window
    uint8_t flags;     // bit0: epoch was carried across a reset, unverified since
};
constexpr uint8_t SURF_FLAG_CLOCK_CARRIED = 0x01;

size_t packSurface(const SurfaceRecord& r, uint8_t out[SURFACE_RECORD_SIZE]);
// false on a CRC failure or a version this build does not understand — both mean
// "stop here", never "guess at the fields".
bool unpackSurface(const uint8_t in[SURFACE_RECORD_SIZE], SurfaceRecord& out);

// Stable name for an id, for the CSV export. Unknown ids render `id<n>`: an
// appended id must never become a row a reader silently drops or mislabels.
size_t surfaceName(char* out, size_t cap, uint16_t id);

// ---- CSV export rows: ONE canonical formatter (as for dives) ---------------
// Identity row, then one `R,` row per record in file order (oldest first).
// `count` and the seq range are what let a reader see that history was shed: a
// log whose seq_hi - seq_lo + 1 exceeds count has dropped records, and says so
// rather than presenting itself as complete. Columns are append-only: new
// information goes on the right of `R,`, nothing existing moves.
constexpr uint8_t SURFACE_ROW_FORMAT_VERSION = 1;

size_t surfaceCsvHeader(char* out, size_t cap, int count, uint32_t seqLo,
                        uint32_t seqHi);
size_t surfaceCsvLine(char* out, size_t cap, const SurfaceRecord& r);

// The record's raw/spread columns hold whole ADC counts, and the console line and
// the `CALAIR,` line render the same floats with numfmt::fixed(v, 0). Converting
// through that formatter — rather than writing a `(int16_t)(x + 0.5f)` here — is
// what guarantees the three surfaces cannot disagree by one count: the rounding
// rule (nearest, ties to even, like printf) lives in one place, and a 12-sample
// average lands on an exact .5 often enough to be reachable. Returns false if the
// value will not render, or does not fit the column, so the caller says "the
// record has no number" instead of storing a wrong one.
bool countsFromFloat(double v, int16_t& out);

// ---- capacity policy (pure, so the host can test the whole of it) ----------
// Records to drop from the FRONT of the log to make room for one more: 0 while
// there is space, n - capacity + 1 when full. Oldest-first is the whole policy —
// the newest record must always survive, because it is the one just measured —
// and dropping never renumbers anything, so a reader can still see the gap.
int compactSurface(int n, int capacity);

} // namespace logfmt
