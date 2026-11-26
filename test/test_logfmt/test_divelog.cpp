#include <unity.h>

#include <cstdio>
#include <cstring>
#include <string>

#include "deco/zhl16c.h"
#include "log/divelog.h"
#include "log/rowfill.h"
#include "util/numfmt.h"

using namespace logfmt;

void setUp() {}
void tearDown() {}

static Snapshot sampleSnap() {
    Snapshot s{};
    s.tSec = 3661;
    s.depthCm = 4023;
    s.votedX1000 = 402;
    s.cellX1000[0] = 405; s.cellX1000[1] = -12; s.cellX1000[2] = 399;
    s.setpointX1000 = 700;
    // 10-bit mask: BIT_LOW_PPO2(0) + BIT_STUCK_OPEN(6) + BIT_HYPOXIA(9)
    s.mode = 1; s.alarmBits = 0b1001000001; s.cnsPct = 37;
    s.ceilingCm = 900; s.ndlMin = 480; s.decoTotalMin = 12;
    s.pulses = 60000; s.battX1000 = 11840;
    s.tempC = -3; s.flags = 0b101;
    return s;
}

// Wire offsets, spelled out because the packing rules below depend on them.
constexpr int kOFF_ALARMS = 20;  // low 8 alarm bits
constexpr int kOFF_FLAGS = 33;   // flags byte (then 2 CRC bytes)

void test_snapshot_roundtrip_and_size() {
    const Snapshot a = sampleSnap();
    uint8_t bytes[SNAPSHOT_SIZE];
    TEST_ASSERT_EQUAL(SNAPSHOT_SIZE, packSnapshot(a, bytes));

    Snapshot b{};
    TEST_ASSERT_TRUE(unpackSnapshot(bytes, b));
    TEST_ASSERT_EQUAL_UINT32(a.tSec, b.tSec);
    TEST_ASSERT_EQUAL_INT32(a.depthCm, b.depthCm);
    TEST_ASSERT_EQUAL_INT16(a.votedX1000, b.votedX1000);
    TEST_ASSERT_EQUAL_INT16(a.cellX1000[1], b.cellX1000[1]); // negative cell ok
    TEST_ASSERT_EQUAL_UINT16(a.pulses, b.pulses);
    TEST_ASSERT_EQUAL_INT8(a.tempC, b.tempC);
    TEST_ASSERT_EQUAL_UINT8(a.flags, b.flags);
    TEST_ASSERT_EQUAL_UINT16(a.alarmBits, b.alarmBits);
}

// The record grid is a fixed stride: the LittleFS writer appends these back to
// back, so a writer that overruns the record by even one byte silently corrupts
// the next one (this did exactly that, writing 34 B into a 32 B record).
void test_writers_stay_inside_the_record() {
    uint8_t box[SNAPSHOT_SIZE + 8];
    std::memset(box, 0xAA, sizeof(box));
    TEST_ASSERT_EQUAL(SNAPSHOT_SIZE, packSnapshot(sampleSnap(), box));
    for (size_t i = SNAPSHOT_SIZE; i < sizeof(box); i++)
        TEST_ASSERT_EQUAL_UINT8_MESSAGE(0xAA, box[i], "packSnapshot wrote past its record");

    uint8_t ebox[EVENT_SIZE + 8];
    std::memset(ebox, 0xAA, sizeof(ebox));
    TEST_ASSERT_EQUAL(EVENT_SIZE, packEvent(Event{4213, EVT_ALARM_RAISED, 6}, ebox));
    for (size_t i = EVENT_SIZE; i < sizeof(ebox); i++)
        TEST_ASSERT_EQUAL_UINT8_MESSAGE(0xAA, ebox[i], "packEvent wrote past its record");
}

// Per-record CRC: a file written during a dive can be cut off mid-record by a
// reset. The reader must be able to stop at the bad row, not decode it.
void test_record_crc_detects_a_torn_write() {
    uint8_t bytes[SNAPSHOT_SIZE];
    packSnapshot(sampleSnap(), bytes);
    bytes[12] ^= 0xFF;  // one cell value flipped
    Snapshot b{};
    TEST_ASSERT_FALSE_MESSAGE(unpackSnapshot(bytes, b), "snapshot CRC did not catch the edit");

    uint8_t eb[EVENT_SIZE];
    packEvent(Event{900, EVT_CELLPROOF_FAIL, 4}, eb);
    eb[2] ^= 0xFF;  // flip, do not write a value that was already there
    Event out{};
    TEST_ASSERT_FALSE_MESSAGE(unpackEvent(eb, out), "event CRC did not catch the edit");
}

// The alarm set is 10 bits wide and the snapshot has no spare byte: bits 8..9
// (CNS_HIGH, HYPOXIA) ride in bits 3..4 of the flags byte. If that packing
// breaks, hypoxia rows disappear from dive files silently.
void test_high_alarm_bits_survive_the_record() {
    const uint16_t hi = (1u << 8) | (1u << 9);
    Snapshot a = sampleSnap();
    a.alarmBits = hi;
    a.flags = 0b000;
    uint8_t bytes[SNAPSHOT_SIZE];
    packSnapshot(a, bytes);

    TEST_ASSERT_EQUAL_UINT8(0, bytes[kOFF_ALARMS] & kFLAGS_ALARM_HI);  // low 8 only
    TEST_ASSERT_EQUAL_UINT8(kFLAGS_ALARM_HI, bytes[kOFF_FLAGS] & kFLAGS_ALARM_HI);

    Snapshot b{};
    TEST_ASSERT_TRUE(unpackSnapshot(bytes, b));
    TEST_ASSERT_EQUAL_UINT16_MESSAGE(hi, b.alarmBits, "CNS/HYPOXIA bits lost in the log");
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(0, b.flags, "borrowed wire bits leaked into flags");

    // real flags must still survive alongside the borrowed bits
    a.alarmBits = hi | 0b10000001;
    a.flags = 0b110;
    packSnapshot(a, bytes);
    TEST_ASSERT_TRUE(unpackSnapshot(bytes, b));
    TEST_ASSERT_EQUAL_UINT16(a.alarmBits, b.alarmBits);
    TEST_ASSERT_EQUAL_UINT8(0b110, b.flags);
}

// ---- tagged record stream: what a torn write means --------------------------

namespace {
struct Captured {
    int snaps = 0, events = 0;
    logfmt::Event lastEvent;
};
void capture(const logfmt::RawRecord& r, void* v) {
    Captured& c = *static_cast<Captured*>(v);
    if (r.tag == logfmt::REC_SNAPSHOT) c.snaps++;
    else if (r.tag == logfmt::REC_EVENT) {
        c.events++;
        logfmt::unpackEvent(r.bytes, c.lastEvent);
    }
}

size_t buildImage(uint8_t* img, const logfmt::Snapshot* ss, int nS,
                  const logfmt::Event* es, int nE, bool withMeta) {
    size_t p = 0;
    logfmt::packHeader(1700000000u, 3, 1, img + p);
    p += logfmt::HEADER_SIZE;
    int si = 0, ei = 0;
    // interleave on purpose: chronological is how the store writes
    while (si < nS || ei < nE) {
        if (si < nS && (si % 2 == 0 || ei >= nE)) {
            img[p++] = logfmt::REC_SNAPSHOT;
            logfmt::packSnapshot(ss[si++], img + p);
            p += logfmt::SNAPSHOT_SIZE;
        } else {
            img[p++] = logfmt::REC_EVENT;
            logfmt::packEvent(es[ei++], img + p);
            p += logfmt::EVENT_SIZE;
        }
    }
    if (withMeta) {
        logfmt::DiveMeta m{};
        m.durationSec = 3600;
        img[p++] = logfmt::REC_META;
        logfmt::packMeta(m, img + p);
        p += 64;
    }
    return p;
}

logfmt::Snapshot mkSnap(uint32_t t) {
    logfmt::Snapshot s{};
    s.tSec = t; s.depthCm = 2500; s.votedX1000 = 700;
    for (int i = 0; i < 3; i++) s.cellX1000[i] = 700;
    s.setpointX1000 = 700; s.mode = 1; s.alarmBits = 0; s.cnsPct = 12;
    return s;
}
} // namespace

void test_header_roundtrip_and_version_gate() {
    uint8_t head[logfmt::HEADER_SIZE];
    logfmt::packHeader(1700000000u, 7, 4242u, head);
    uint32_t epoch = 0, seq = 0;
    uint8_t slot = 0;
    TEST_ASSERT_TRUE(logfmt::unpackHeader(head, epoch, slot, seq));
    TEST_ASSERT_EQUAL_UINT32(1700000000u, epoch);
    TEST_ASSERT_EQUAL_UINT8(7, slot);
    TEST_ASSERT_EQUAL_UINT32(4242u, seq);
    head[9] ^= 0xFF;   // one flipped byte in the header
    TEST_ASSERT_FALSE(logfmt::unpackHeader(head, epoch, slot, seq));
}

// The header grew its sequence field out of bytes that used to be zero padding,
// so a file written before that change must still read — and must read as
// "age unknown", not as a dive that is sequenced. The board being flashed on
// 2026-11-24 had ten such dives on it; making them unreadable would have been a
// worse bug than the one being fixed.
void test_header_reads_a_pre_sequence_file_as_unknown() {
    uint8_t head[logfmt::HEADER_SIZE];
    logfmt::packHeader(1700000000u, 4, 0, head);   // what the old writer left in 5..7
    TEST_ASSERT_EQUAL_UINT8(0, head[5]);
    TEST_ASSERT_EQUAL_UINT8(0, head[6]);
    TEST_ASSERT_EQUAL_UINT8(0, head[7]);
    uint32_t epoch = 0, seq = 99;
    uint8_t slot = 0;
    TEST_ASSERT_TRUE(logfmt::unpackHeader(head, epoch, slot, seq));
    TEST_ASSERT_EQUAL_UINT32(logfmt::SEQ_UNKNOWN, seq);
    TEST_ASSERT_EQUAL_UINT32(1700000000u, epoch);
    TEST_ASSERT_EQUAL_UINT8(4, slot);
    // The sequence is inside the CRC: tampering with it is a corrupt header,
    // not a dive that suddenly looks older or newer than it is.
    head[5] ^= 0x01;
    TEST_ASSERT_FALSE(logfmt::unpackHeader(head, epoch, slot, seq));
}

// A sequence counter that came back round would make the newest dive look like
// the oldest and get it evicted first — the exact failure mode the field
// exists to prevent, so the saturation is asserted rather than assumed.
void test_sequence_saturates_and_never_wraps() {
    uint8_t head[logfmt::HEADER_SIZE];
    logfmt::packHeader(1u, 1, logfmt::SEQ_MAX + 7u, head);
    uint32_t epoch = 0, seq = 0;
    uint8_t slot = 0;
    TEST_ASSERT_TRUE(logfmt::unpackHeader(head, epoch, slot, seq));
    TEST_ASSERT_EQUAL_UINT32(logfmt::SEQ_MAX, seq);

    const uint32_t empty[1] = {0};
    TEST_ASSERT_EQUAL_UINT32(1u, logfmt::nextSeq(empty, 0));      // empty log
    const uint32_t legacy[3] = {0, 0, 0};
    TEST_ASSERT_EQUAL_UINT32(1u, logfmt::nextSeq(legacy, 3));     // pre-change dives
    const uint32_t full[10] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    TEST_ASSERT_EQUAL_UINT32(11u, logfmt::nextSeq(full, 10));
    const uint32_t afterEvict[10] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
    TEST_ASSERT_EQUAL_UINT32(12u, logfmt::nextSeq(afterEvict, 10));
    const uint32_t maxed[2] = {logfmt::SEQ_MAX, 0};
    TEST_ASSERT_EQUAL_UINT32(logfmt::SEQ_MAX, logfmt::nextSeq(maxed, 2));
}

// This is the bench defect written as a unit test. Ten dives filled the FIFO,
// then three more were written, and every one of them landed in slot 1 because
// all ten epoch keys were equal (no RTC: every dive is epoch 0). The 40-second
// dive was gone before the next boot, and the log kept the nine OLDEST dives
// plus the very newest. Epoch ordering needs an epoch; this ordering runs with
// no clock at all.
void test_eviction_rotates_when_there_is_no_clock() {
    const uint32_t noEpoch[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    uint32_t seq[10] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    uint32_t written = 11;
    // Twelve more dives: the evicted slot must advance every time, and the set
    // of surviving sequences must be the twelve most recent, not the nine
    // oldest plus the newest.
    int evicted[12];
    for (int i = 0; i < 12; i++) {
        const int s = logfmt::pickEvictSlot(seq, noEpoch, 10);
        TEST_ASSERT_TRUE(s >= 0);
        evicted[i] = s;
        seq[s] = written++;
    }
    for (int i = 0; i < 12; i++)
        TEST_ASSERT_EQUAL_MESSAGE(i % 10, evicted[i],
                                  "eviction must rotate through the slots, not "
                                  "hammer one of them");
    uint32_t lo = seq[0], hi = seq[0];
    for (int i = 1; i < 10; i++) { if (seq[i] < lo) lo = seq[i]; if (seq[i] > hi) hi = seq[i]; }
    // Twenty-two dives written; the ten that survive must be 13..22, the last
    // ten. The firmware of 2026-11-24 kept 2..10 plus 22 and lost 11..21.
    TEST_ASSERT_EQUAL_UINT32(13u, lo);
    TEST_ASSERT_EQUAL_UINT32(22u, hi);
    bool distinct[24] = {false};
    for (int i = 0; i < 10; i++) {
        TEST_ASSERT_FALSE_MESSAGE(distinct[seq[i]], "two dives sharing a sequence");
        distinct[seq[i]] = true;
    }
}

void test_unknown_age_is_evicted_before_any_sequenced_dive() {
    // A board upgrading into this format has unsequenced dives on it. Those are
    // the ones to shed first: they predate everything the new firmware wrote.
    const uint32_t seq[3] = {0, 5, 7};
    const uint32_t epoch[3] = {999999u, 0u, 0u};   // a legacy dive with a real epoch
    TEST_ASSERT_EQUAL(0, logfmt::pickEvictSlot(seq, epoch, 3));
    const uint32_t seq2[3] = {0, 0, 0};            // all legacy: epoch decides
    const uint32_t epoch2[3] = {500u, 100u, 300u};
    TEST_ASSERT_EQUAL(1, logfmt::pickEvictSlot(seq2, epoch2, 3));
    const uint32_t seq3[3] = {4, 4, 4};            // identical keys: lower slot wins
    const uint32_t epoch3[3] = {7u, 7u, 7u};       // deterministically, not by luck
    TEST_ASSERT_EQUAL(0, logfmt::pickEvictSlot(seq3, epoch3, 3));
    TEST_ASSERT_EQUAL(-1, logfmt::pickEvictSlot(seq3, epoch3, 0));
    // Equal sequences are impossible in a running log, but if two files somehow
    // say the same thing, the real clock is the tie-break.
    const uint32_t seq4[2] = {6, 6};
    const uint32_t epoch4[2] = {1800000000u, 1700000000u};
    TEST_ASSERT_EQUAL(1, logfmt::pickEvictSlot(seq4, epoch4, 2));
}

// Rendered by this project, printed on the diver's CSV, and never by newlib:
// `strftime`/`gmtime` would pull the time tables and float conversion into the
// task that answers clients, which is the stack that has run out before.
void test_utc_rendering_is_exact_and_says_when_there_is_no_clock() {
    char s[32];
    const struct { uint32_t epoch; const char* want; } cases[] = {
        {1234567u, "1970-01-15T06:56:07Z"},
        {1700000000u, "2023-11-14T22:13:20Z"},
        {1795509660u, "2026-11-24T08:41:00Z"},   // the day this was written
        {951782400u, "2000-02-29T00:00:00Z"},    // 2000 IS a leap year
        {4107542400u, "2100-03-01T00:00:00Z"},   // 2100 is not: there is no Feb 29
        {0u, logfmt::kNoClock},                  // no clock: not 1970-01-01
    };
    for (const auto& c : cases) {
        const size_t n = logfmt::isoUtc(s, sizeof(s), c.epoch);
        TEST_ASSERT_EQUAL_STRING_MESSAGE(c.want, s, "utc render");
        TEST_ASSERT_EQUAL_MESSAGE((int)std::strlen(c.want), (int)n, "utc length");
    }
    TEST_ASSERT_EQUAL(0, logfmt::isoUtc(s, 20, 1700000000u));   // 20 stops before the Z
}

void test_scan_walks_interleaved_records_in_order() {
    static uint8_t img[2048];
    logfmt::Snapshot ss[4];
    for (int i = 0; i < 4; i++) ss[i] = mkSnap(30 * i);
    const logfmt::Event es[2] = {{5, logfmt::EVT_ALARM_RAISED, 1},
                                 {11, logfmt::EVT_MODE_CHANGE, 2}};
    const size_t len = buildImage(img, ss, 4, es, 2, true);

    Captured c;
    const auto sc = logfmt::scanRecords(img, len, capture, &c);
    TEST_ASSERT_TRUE(sc.headerOk);
    TEST_ASSERT_EQUAL_INT(4, sc.snapshots);
    TEST_ASSERT_EQUAL_INT(2, sc.events);
    TEST_ASSERT_TRUE(sc.metaOk);
    TEST_ASSERT_EQUAL_INT(4, c.snaps);
    TEST_ASSERT_EQUAL_INT(2, c.events);
    TEST_ASSERT_EQUAL_UINT16(logfmt::EVT_MODE_CHANGE, c.lastEvent.id);
    TEST_ASSERT_EQUAL_UINT32(len, sc.bytesConsumed);   // whole file validated
}

// The contract with a reset mid-write: stop at the failing record, keep
// everything before it, report nothing after it.
void test_scan_stops_at_the_first_corrupt_record() {
    static uint8_t img[2048];
    logfmt::Snapshot ss[6];
    for (int i = 0; i < 6; i++) ss[i] = mkSnap(30 * i);
    const size_t len = buildImage(img, ss, 6, nullptr, 0, true);

    // find the 4th snapshot row and tear one payload byte
    size_t p = logfmt::HEADER_SIZE;
    int seen = 0;
    while (p < len && seen < 3) {
        const size_t sz = (img[p] == logfmt::REC_SNAPSHOT) ? logfmt::SNAPSHOT_SIZE
                                                           : logfmt::EVENT_SIZE;
        p += 1 + sz;
        seen++;
    }
    img[p + 1 + 4] ^= 0x01;   // inside the 4th snapshot: its CRC now fails

    Captured c;
    const auto sc = logfmt::scanRecords(img, len, capture, &c);
    TEST_ASSERT_EQUAL_INT(3, sc.snapshots);          // rows 1..3 survive
    TEST_ASSERT_EQUAL(0, sc.events);
    TEST_ASSERT_FALSE(sc.metaOk);                    // dive looks unfinished
    TEST_ASSERT_EQUAL_UINT32(p, sc.bytesConsumed);   // consumed up to the bad row

    // truncated tail (write cut short): same rule
    const auto sc2 = logfmt::scanRecords(img, p + 10, capture, &c);
    TEST_ASSERT_EQUAL_INT(3, sc2.snapshots);
}

void test_scan_rejects_unknown_tags_and_foreign_headers() {
    static uint8_t img[256];
    logfmt::Snapshot ss[1] = {mkSnap(0)};
    const size_t len = buildImage(img, ss, 1, nullptr, 0, false);
    img[logfmt::HEADER_SIZE] = 'X';   // corrupt tag
    Captured c;
    const auto sc = logfmt::scanRecords(img, len, capture, &c);
    TEST_ASSERT_TRUE(sc.headerOk);
    TEST_ASSERT_EQUAL_INT(0, sc.snapshots);

    uint8_t junk[logfmt::HEADER_SIZE] = {};
    const auto sc2 = logfmt::scanRecords(junk, sizeof(junk), nullptr, nullptr);
    TEST_ASSERT_FALSE(sc2.headerOk);
}

void test_snapshot_version_gate() {
    uint8_t bytes[SNAPSHOT_SIZE];
    packSnapshot(sampleSnap(), bytes);
    bytes[0] = 0xEE; // corrupt version
    Snapshot b{};
    TEST_ASSERT_FALSE(unpackSnapshot(bytes, b));
}

void test_meta_crc_detects_corruption() {
    DiveMeta m{};
    m.durationSec = 5400; m.maxDepthCm = 4010;
    m.minVotedX1000 = 178; m.maxVotedX1000 = 1305;
    m.totalPulses = 9999; m.cnsPeakPct = 62;
    m.gfLow = 30; m.gfHigh = 85;
    m.sp1X1000 = 400; m.sp2X1000 = 700; m.sp3X1000 = 1000; m.sp4X1000 = 1300;
    m.eventCount = 7; m.snapshotCount = 180;

    uint8_t bytes[64];
    packMeta(m, bytes);
    DiveMeta back{};
    TEST_ASSERT_TRUE(unpackMeta(bytes, back));
    TEST_ASSERT_EQUAL_UINT32(m.durationSec, back.durationSec);
    TEST_ASSERT_EQUAL_UINT16(m.snapshotCount, back.snapshotCount);

    bytes[10] ^= 0x01; // single bit flip somewhere in the payload
    TEST_ASSERT_FALSE(unpackMeta(bytes, back));
}

void test_event_roundtrip() {
    Event e{930, EVT_CELLPROOF_FAIL, 1010};
    uint8_t bytes[EVENT_SIZE];
    packEvent(e, bytes);
    Event b{};
    TEST_ASSERT_TRUE(unpackEvent(bytes, b));
    TEST_ASSERT_EQUAL_UINT32(930, b.tSec);
    TEST_ASSERT_EQUAL_UINT16(EVT_CELLPROOF_FAIL, b.id);
    TEST_ASSERT_EQUAL_UINT16(1010, b.value);
}

// ---- the canonical export rows ---------------------------------------------
//
// ONE formatter for both divers-facing exports: `LOG EXPORT <n>` on the handset
// and `GET /dive/<n>.csv` on the web. Before 2026-11-23 there were two writers
// with two shapes (14 vs 17 fields, alarms hex vs decimal, ceiling m vs cm,
// battery mV vs V) and BENCH.md step 4, which claims they match, had never
// compared them. These tests are what keeps that claim true.

static DiveMeta sampleMeta() {
    DiveMeta m{};
    m.durationSec = 120; m.maxDepthCm = 500;
    m.minVotedX1000 = 400; m.maxVotedX1000 = 420;
    m.totalPulses = 300; m.cnsPeakPct = 12;
    m.gfLow = 30; m.gfHigh = 85;
    m.sp1X1000 = 400; m.sp2X1000 = 700; m.sp3X1000 = 1000; m.sp4X1000 = 1300;
    m.snapshotCount = 2; m.eventCount = 1;
    return m;
}

static int countFields(const char* line) {
    int n = 1;
    for (const char* c = line; *c; ++c) {
        if (*c == ',') ++n;
    }
    return n;
}

static void test_dive_identity_line() {
    char l[48];
    const size_t n = diveCsvLine(l, sizeof(l), 7, 1234567);
    // v3 appends one column on the right; the first five fields are exactly the
    // v1/v2 bytes, which is what "append-only" has to mean in practice.
    const char* want = "DIVE,7,1234567,3,3,1970-01-15T06:56:07Z\n";
    TEST_ASSERT_EQUAL_STRING(want, l);
    TEST_ASSERT_EQUAL((int)std::strlen(want), (int)n);
    TEST_ASSERT_EQUAL_STRING_LEN("DIVE,7,1234567,3,3,", l, 19);
    int fields = 1;
    for (const char* c = l; *c; ++c) if (*c == ',') ++fields;
    TEST_ASSERT_EQUAL(6, fields);   // the five v1/v2 fields, plus the date
    // A dive recorded with no clock says so in the date field.
    char l2[48];
    diveCsvLine(l2, sizeof(l2), 7, 0);
    TEST_ASSERT_EQUAL_STRING("DIVE,7,0,3,3,clock-unset\n", l2);
}

static void test_snapshot_row_is_the_documented_shape() {
    const Snapshot s = sampleSnap();
    char l[192];
    const size_t n = snapshotCsvLine(l, sizeof(l), s);
    const char* want =
        "S,3661,40.23,0.402,0.405,-0.012,0.399,0.700,0x0241,37,9,480,12,"
        "60000,11840,1,-3,5,900\n";
    TEST_ASSERT_EQUAL_STRING(want, l);
    TEST_ASSERT_EQUAL((int)std::strlen(want), (int)n);
    // v1's fifteen fields, in v1's order, byte for byte. The contract is
    // append-only: a handset written against the 2026-11-22 document still finds
    // what it expects, and new information only ever arrives on the right.
    TEST_ASSERT_NOT_NULL(std::strstr(l, "S,3661,40.23,0.402,0.405,-0.012,0.399,"
                                        "0.700,0x0241,37,9,480,12,60000,11840,"));
    // tag + 14 v1 + 4 v2 (mode, temp_c, flags, ceiling_cm)
    TEST_ASSERT_EQUAL(19, countFields(l));
}

static void test_event_and_meta_rows() {
    char l[192];
    const Event e{60, EVT_ALARM_RAISED, 1};
    TEST_ASSERT_EQUAL_STRING("E,60,1,1\n", eventCsvLine(l, sizeof(l), e) ? l : "");

    const DiveMeta m = sampleMeta();
    TEST_ASSERT_EQUAL_STRING(
        "M,120,500,0.400,0.420,300,12,2,1,30,85,0.400,0.700,1.000,1.300\n",
        metaCsvLine(l, sizeof(l), m) ? l : "");
    // tag + 8 v1 + 6 v2 (gf_low gf_high sp1..sp4); field 2 is centimetres, as
    // the firmware has always sent it (the doc once called it metres).
    TEST_ASSERT_EQUAL(15, countFields(l));
}

static void test_rows_refuse_rather_than_truncate() {
    // The store used to format rows into a 64-byte static buffer: a wide row was
    // cut in half and the diver got a plausible-looking truncated dive.
    Snapshot s{};
    s.tSec = 0xFFFFFFFFu;
    s.depthCm = 2147483647;
    s.votedX1000 = -32768;
    for (int i = 0; i < 3; i++) s.cellX1000[i] = -32768;
    s.setpointX1000 = -32768;
    s.mode = 255; s.alarmBits = 0xFFFF; s.cnsPct = 255;
    s.ceilingCm = -32768; s.ndlMin = -32768; s.decoTotalMin = -32768;
    s.pulses = 65535; s.battX1000 = 65535; s.tempC = -128; s.flags = 255;

    char wide[192], tiny[40];
    const size_t n = snapshotCsvLine(wide, sizeof(wide), s);
    TEST_ASSERT_TRUE(n > 0 && n < sizeof(wide));   // the export buffer must fit
    TEST_ASSERT_EQUAL('\n', wide[n - 1]);          // ... and be a whole row
    TEST_ASSERT_EQUAL(19, countFields(wide));
    TEST_ASSERT_EQUAL(0, snapshotCsvLine(tiny, sizeof(tiny), s));
}

// Both ways of getting rows out of a dive file: the streaming sink, which is
// exactly what DiveStore::exportCsv does for the handset and the web, and the
// buffered form. They must produce the same bytes, or the two transports are
// two writers again by another name.
struct Collector {
    char buf[4096];
    size_t used;
};

static void collectLine(const char* line, void* v) {
    Collector& c = *static_cast<Collector*>(v);
    const size_t n = std::strlen(line);
    if (c.used + n + 1 <= sizeof(c.buf)) {
        std::memcpy(c.buf + c.used, line, n);
        c.used += n;
        c.buf[c.used] = '\0';
    }
}

static void test_both_export_paths_are_the_same_bytes() {
    Snapshot s1 = sampleSnap(); s1.tSec = 30;
    Snapshot s2 = sampleSnap(); s2.tSec = 60;
    const Event ev{45, EVT_MODE_CHANGE, 1};
    uint8_t sb1[SNAPSHOT_SIZE], sb2[SNAPSHOT_SIZE], eb[EVENT_SIZE];
    packSnapshot(s1, sb1);
    packSnapshot(s2, sb2);
    packEvent(ev, eb);

    // Chronological interleaving, the way the store writes: S, E, S.
    const ImageRecord recs[] = {
        {REC_SNAPSHOT, sb1}, {REC_EVENT, eb}, {REC_SNAPSHOT, sb2}};
    const DiveMeta m = sampleMeta();
    uint8_t img[512];
    const size_t len = packImage(img, sizeof(img), 4242, 3, 1, recs, 3, &m);
    TEST_ASSERT_TRUE(len > 0);

    Collector streamed{};
    const ScanResult sc = emitCsvRows(img, len, collectLine, &streamed);
    TEST_ASSERT_TRUE(sc.headerOk);
    TEST_ASSERT_EQUAL(2, sc.snapshots);
    TEST_ASSERT_EQUAL(1, sc.events);
    TEST_ASSERT_TRUE(sc.metaOk);

    char buffered[4096];
    const size_t n = toCsvImage(img, len, buffered, sizeof(buffered));
    TEST_ASSERT_EQUAL_STRING(streamed.buf, buffered);
    TEST_ASSERT_EQUAL((int)streamed.used, (int)n);

    // Identity line first, exactly once, carrying the slot the writer stamped.
    const char* kHead = "DIVE,3,4242,3,3,1970-01-01T01:10:42Z\n";
    TEST_ASSERT_EQUAL_STRING_LEN(kHead, buffered, (int)std::strlen(kHead));
    TEST_ASSERT_NULL(std::strstr(buffered + std::strlen(kHead), "DIVE,"));

    // Rows in file order (the E row belongs between the two S rows), trailer last.
    const char* a = std::strstr(buffered, "\nS,30,");
    const char* b = std::strstr(buffered, "\nE,45,");
    const char* c = std::strstr(buffered, "\nS,60,");
    const char* mm = std::strstr(buffered, "\nM,120,");
    TEST_ASSERT_NOT_NULL(a);
    TEST_ASSERT_NOT_NULL(b);
    TEST_ASSERT_NOT_NULL(c);
    TEST_ASSERT_NOT_NULL(mm);
    TEST_ASSERT_TRUE(a < b && b < c && c < mm);

    // A buffer that cannot hold everything reports failure, not half a dive.
    char tiny[40];
    TEST_ASSERT_EQUAL(0, toCsvImage(img, len, tiny, sizeof(tiny)));
    TEST_ASSERT_EQUAL('\0', tiny[0]);
}

static void test_pack_image_refuses_a_short_buffer() {
    Snapshot s = sampleSnap();
    uint8_t sb[SNAPSHOT_SIZE];
    packSnapshot(s, sb);
    const ImageRecord recs[] = {{REC_SNAPSHOT, sb}};
    const DiveMeta m = sampleMeta();
    uint8_t img[HEADER_SIZE + 1 + SNAPSHOT_SIZE];   // trailer will not fit
    TEST_ASSERT_EQUAL(0, packImage(img, sizeof(img), 1, 1, 1, recs, 1, &m));
    // ... and fits exactly when the trailer has room.
    uint8_t enough[HEADER_SIZE + 1 + SNAPSHOT_SIZE + 1 + 64];
    TEST_ASSERT_TRUE(packImage(enough, sizeof(enough), 1, 1, 1, recs, 1, &m) > 0);
}

// ---- the surface log -------------------------------------------------------
// /surface.bin holds what the machine did at the surface: a calibration verdict,
// a pre-dive pass or failure. Those facts have no dive file to live in — a failed
// pre-dive check prevents the dive that would have held the row — so they have a
// record of their own, and this is the only thing standing between that record and
// a future edit moving a field. The offsets below are the format.

static SurfaceRecord calAirOk() {
    // The bench's own verdict, captured 2026-11-24 as
    // `CALAIR,2,2,12,21,1514,1519,1528,0` on the handset link.
    SurfaceRecord r{};
    r.epoch = 1795518150u;  // 2026-11-24T11:02:30Z, the bench PC's clock
    r.seq = 3;
    r.id = EVT_CAL_AIR;
    r.value = 2;            // bridge::CALAIR_OK
    r.raw[0] = 1514; r.raw[1] = 1519; r.raw[2] = 1528;
    r.spread = 21;
    r.samples = 12;
    r.flags = 0;
    return r;
}

void test_surface_record_roundtrip_and_stride() {
    const SurfaceRecord a = calAirOk();
    // Padded box first: a packer that writes one byte past its record is worse
    // than a wrong field — it corrupts whatever the caller put next to it, and on
    // the target that is a stack frame. This is what reddens when a field is
    // widened without shrinking another.
    uint8_t box[SURFACE_RECORD_SIZE + 8];
    std::memset(box, 0xAA, sizeof(box));
    TEST_ASSERT_EQUAL(SURFACE_RECORD_SIZE, packSurface(a, box));
    for (size_t i = SURFACE_RECORD_SIZE; i < sizeof(box); i++)
        TEST_ASSERT_EQUAL_UINT8_MESSAGE(0xAA, box[i], "packSurface wrote past its record");
    TEST_ASSERT_EQUAL(24, SURFACE_RECORD_SIZE);   // the stride the store allocates

    uint8_t bytes[SURFACE_RECORD_SIZE];
    std::memcpy(bytes, box, SURFACE_RECORD_SIZE);

    SurfaceRecord b{};
    TEST_ASSERT_TRUE(unpackSurface(bytes, b));
    TEST_ASSERT_EQUAL_UINT32(a.epoch, b.epoch);
    TEST_ASSERT_EQUAL_UINT32(a.seq, b.seq);
    TEST_ASSERT_EQUAL_UINT16(a.id, b.id);
    TEST_ASSERT_EQUAL_UINT16(a.value, b.value);
    for (int i = 0; i < 3; i++)
        TEST_ASSERT_EQUAL_INT_MESSAGE(a.raw[i], b.raw[i], "raw counts must survive exactly");
    TEST_ASSERT_EQUAL_UINT16(a.spread, b.spread);
    TEST_ASSERT_EQUAL_UINT8(a.samples, b.samples);
    TEST_ASSERT_EQUAL_UINT8(a.flags, b.flags);

    // int16, so a negative raw count (a channel reading below zero at air — the
    // sign a fouled cell can show) is not silently made positive by an unsigned
    // field. That sign is how an installer spots a dead channel.
    SurfaceRecord neg = a;
    neg.raw[1] = -37;
    neg.flags = SURF_FLAG_CLOCK_CARRIED;
    packSurface(neg, bytes);
    TEST_ASSERT_EQUAL(SURFACE_RECORD_SIZE, packSurface(neg, box));
    for (size_t i = SURFACE_RECORD_SIZE; i < sizeof(box); i++)
        TEST_ASSERT_EQUAL_UINT8(0xAA, box[i]);
    unpackSurface(bytes, b);
    TEST_ASSERT_EQUAL_INT_MESSAGE(-37, b.raw[1], "negative raw count lost its sign");
    TEST_ASSERT_EQUAL_UINT8(SURF_FLAG_CLOCK_CARRIED, b.flags);
}

// Every field's byte offset, spelled out and decoded from hand-written bytes, so
// a future edit that inserts or widens a field cannot quietly re-label what is
// already on a diver's flash. seq is 3 bytes because SEQ_MAX is one definition
// (0x00FFFFFF) shared with the dive header: writing 4 here would land the CRC at
// the wrong offset, and this is the test that would notice.
void test_surface_record_offsets_are_the_documented_bytes() {
    uint8_t bytes[SURFACE_RECORD_SIZE] = {};
    bytes[0] = SURFACE_REC_VERSION;              // 1
    bytes[1] = 0xC6; bytes[2] = 0x6E; bytes[3] = 0x05; bytes[4] = 0x6B;  // epoch 1795518150 LE
    bytes[5] = 0x39; bytes[6] = 0x05; bytes[7] = 0x00;                   // seq = 1337
    bytes[8] = 0x07; bytes[9] = 0x00;                                    // id
    bytes[10] = 0x03; bytes[11] = 0x00;                                  // value = warm-up
    bytes[12] = 0x0A; bytes[13] = 0x00;                                  // raw0 = 10
    bytes[14] = 0x0B; bytes[15] = 0x00;                                  // raw1 = 11
    bytes[16] = 0x0C; bytes[17] = 0x00;                                  // raw2 = 12
    bytes[18] = 0x15; bytes[19] = 0x00;                                  // spread = 21
    bytes[20] = 12;                                                      // samples
    bytes[21] = SURF_FLAG_CLOCK_CARRIED;                                 // flags
    const uint16_t crc = crc16(bytes, SURFACE_RECORD_SIZE - 2);
    bytes[22] = static_cast<uint8_t>(crc);
    bytes[23] = static_cast<uint8_t>(crc >> 8);

    SurfaceRecord r{};
    TEST_ASSERT_TRUE_MESSAGE(unpackSurface(bytes, r), "hand-written record rejected");
    TEST_ASSERT_EQUAL_UINT32(1795518150u, r.epoch);   // the same value calAirOk() writes
    TEST_ASSERT_EQUAL_UINT32(1337u, r.seq);
    TEST_ASSERT_EQUAL_UINT16(EVT_CAL_AIR, r.id);
    TEST_ASSERT_EQUAL_UINT16(3u, r.value);
    TEST_ASSERT_EQUAL_INT(10, r.raw[0]);
    TEST_ASSERT_EQUAL_INT(11, r.raw[1]);
    TEST_ASSERT_EQUAL_INT(12, r.raw[2]);
    TEST_ASSERT_EQUAL_UINT16(21u, r.spread);
    TEST_ASSERT_EQUAL_UINT8(12, r.samples);
    TEST_ASSERT_EQUAL_UINT8(SURF_FLAG_CLOCK_CARRIED, r.flags);
}

void test_surface_record_crc_and_version_gates() {
    uint8_t bytes[SURFACE_RECORD_SIZE];
    packSurface(calAirOk(), bytes);

    // A torn write: one byte flipped mid-record. The reader must stop, not
    // decode a plausible-looking calibration with different raw counts.
    bytes[14] ^= 0xFF;
    SurfaceRecord r{};
    TEST_ASSERT_FALSE_MESSAGE(unpackSurface(bytes, r), "surface CRC did not catch the edit");

    // A layout this build does not know. The CRC is recomputed so the only thing
    // that can reject this is the version gate: without it, a v2 record's fields
    // would be read as v1 and the numbers would be wrong but convincing.
    packSurface(calAirOk(), bytes);
    bytes[0] = SURFACE_REC_VERSION + 1;
    const uint16_t crc = crc16(bytes, SURFACE_RECORD_SIZE - 2);
    bytes[22] = static_cast<uint8_t>(crc);
    bytes[23] = static_cast<uint8_t>(crc >> 8);
    TEST_ASSERT_FALSE_MESSAGE(unpackSurface(bytes, r), "unknown surface version was accepted");
}

void test_surface_names_are_stable_and_unknown_ids_keep_their_number() {
    char n[24];
    TEST_ASSERT_EQUAL_STRING("cal_air", (surfaceName(n, sizeof(n), EVT_CAL_AIR), n));
    TEST_ASSERT_EQUAL_STRING("pre_dive_pass", (surfaceName(n, sizeof(n), EVT_PREPASS), n));
    TEST_ASSERT_EQUAL_STRING("pre_dive_fail", (surfaceName(n, sizeof(n), EVT_PREFAIL), n));
    // H1: the two continuity verdicts. These rows are the only durable trace of a
    // dive that was planned from the wrong starting point, so a name is not
    // decoration — a row a spreadsheet cannot name is a row nobody reads.
    TEST_ASSERT_EQUAL_STRING("continuity_restored",
                             (surfaceName(n, sizeof(n), EVT_CONT_RESTORED), n));
    TEST_ASSERT_EQUAL_STRING("continuity_lost",
                             (surfaceName(n, sizeof(n), EVT_CONT_LOST), n));
    // An id appended by a future build still gets a label: a row without a name
    // is a row a reader drops, which is how append-only logs start lying.
    TEST_ASSERT_EQUAL_STRING("id99", (surfaceName(n, sizeof(n), 99), n));
    TEST_ASSERT_EQUAL(0, surfaceName(n, 4, EVT_CAL_AIR));   // 4 cannot hold "cal_air"
}

// The exported rows, byte for byte. `GET /surface.csv` is the only export path, so
// these strings ARE the format a diver's spreadsheet sees.
void test_surface_rows_are_the_documented_shape() {
    char line[128];

    surfaceCsvLine(line, sizeof(line), calAirOk());
    TEST_ASSERT_EQUAL_STRING(
        "R,3,1795518150,2026-11-24T11:02:30Z,7,cal_air,2,1514,1519,1528,21,12,0\n", line);

    // A failure at the surface with no clock: the date column says so instead of
    // rendering 1970, and `value` names the step that failed (4 = battery), the
    // same order as the five `PRE,` fields.
    SurfaceRecord f{};
    f.seq = 4; f.id = EVT_PREFAIL; f.value = 4;
    surfaceCsvLine(line, sizeof(line), f);
    TEST_ASSERT_EQUAL_STRING(
        "R,4,0,clock-unset,16,pre_dive_fail,4,0,0,0,0,0,0\n", line);

    // A date carried across a reset is not a verified date, and the flags column
    // is what says which is which months later.
    SurfaceRecord p = calAirOk();
    p.seq = 5; p.epoch = 1795517641u; p.id = EVT_PREPASS; p.value = 0;
    p.raw[0] = p.raw[1] = p.raw[2] = 0; p.spread = 0; p.samples = 0;
    p.flags = SURF_FLAG_CLOCK_CARRIED;
    surfaceCsvLine(line, sizeof(line), p);
    TEST_ASSERT_EQUAL_STRING(
        "R,5,1795517641,2026-11-24T10:54:01Z,13,pre_dive_pass,0,0,0,0,0,0,1\n", line);

    // The continuity verdicts (H1). The `value` column is id-specific in a way the
    // calibration rows never are: for id 17 it is the surface interval credited,
    // in seconds; for id 18 it is the reason nothing was credited (2 = "the record
    // there does not decode"). Both rows live outside the dive file on purpose: the
    // dive they describe may be the one whose file never got written, and "this
    // dive was planned from carried tissues" / "this dive was not" is the fact a
    // reader needs in order to believe the ceiling column of the dive itself.
    SurfaceRecord c1{};
    c1.seq = 6; c1.epoch = 1795521750u; c1.id = EVT_CONT_RESTORED; c1.value = 3600;
    surfaceCsvLine(line, sizeof(line), c1);
    TEST_ASSERT_EQUAL_STRING(
        "R,6,1795521750,2026-11-24T12:02:30Z,17,continuity_restored,3600,0,0,0,0,0,0\n", line);
    SurfaceRecord c2{};
    c2.seq = 7; c2.id = EVT_CONT_LOST; c2.value = 2;   // 2 = CONT_LOSS_BAD
    surfaceCsvLine(line, sizeof(line), c2);
    TEST_ASSERT_EQUAL_STRING(
        "R,7,0,clock-unset,18,continuity_lost,2,0,0,0,0,0,0\n", line);

    // Identity row: count plus the seq range, so a log that shed history admits
    // it (5..9 with only 3 records is two records gone) instead of presenting
    // itself as the whole story.
    surfaceCsvHeader(line, sizeof(line), 3, 5, 9);
    TEST_ASSERT_EQUAL_STRING("SURFACE,1,3,5,9\n", line);
    // The rows themselves still carry their original seq across a compaction.
    SurfaceRecord g = calAirOk();
    g.seq = 5;
    surfaceCsvLine(line, sizeof(line), g);
    TEST_ASSERT_EQUAL(0, std::strncmp("R,5,", line, 4));
}

void test_surface_rows_refuse_rather_than_truncate() {
    char line[16];
    TEST_ASSERT_EQUAL(0, surfaceCsvLine(line, sizeof(line), calAirOk()));
    TEST_ASSERT_EQUAL(0, surfaceCsvLine(line, 40, calAirOk()));   // one byte short
    char big[64];
    TEST_ASSERT(surfaceCsvHeader(big, sizeof(big), 1, 1, 1) > 0);
    TEST_ASSERT_EQUAL(0, surfaceCsvHeader(big, 8, 1, 1, 1));
}

// The record columns hold whole ADC counts, and the console line and the
// `CALAIR,` line print the same floats. If the record rounded differently, the
// spreadsheet row and the console would disagree by one count about the same
// calibration — small, and exactly the kind of drift the calibration work exists
// to kill. A 12-sample average lands on an exact .5 often enough (sum = 12k+6)
// for the tie rule to be reachable, so these pin ties to even, like printf.
void test_surface_counts_round_the_way_the_wire_prints_them() {
    const struct { double v; int want; } cases[] = {
        {1514.0, 1514}, {1519.0, 1519},
        {1514.4, 1514}, {1514.6, 1515},
        {0.5, 0}, {1.5, 2}, {2.5, 2},        // ties to even, not away from zero
        {1514.5, 1514}, {1515.5, 1516},
        {-37.0, -37}, {-37.6, -38},
        {21.0, 21},                          // a worst-spread value from the bench
    };
    char wire[24], again[24];
    // -0.5 is left out of the table on purpose: numfmt renders negative zero as
    // "-0" (printf's signbit rule) and an integer column cannot hold a sign with
    // no value behind it. Asserted separately below rather than quietly skipped.
    for (const auto& c : cases) {
        int16_t got = 0;
        TEST_ASSERT_TRUE_MESSAGE(logfmt::countsFromFloat(c.v, got), "count refused");
        TEST_ASSERT_EQUAL_INT_MESSAGE(c.want, static_cast<int>(got), "count rounding");
        // And the stored number prints back as the same bytes the console used.
        numfmt::fixed(wire, sizeof(wire), c.v, 0);
        std::snprintf(again, sizeof(again), "%d", static_cast<int>(got));
        TEST_ASSERT_EQUAL_STRING_MESSAGE(again, wire, "stored count is not what the wire prints");
    }

    int16_t z = 7;
    TEST_ASSERT_TRUE(logfmt::countsFromFloat(-0.5, z));
    TEST_ASSERT_EQUAL_INT(0, static_cast<int>(z));
    numfmt::fixed(wire, sizeof(wire), -0.5, 0);
    TEST_ASSERT_EQUAL_STRING("-0", wire);   // the one place the two may differ
}

void test_surface_counts_refuse_what_they_cannot_hold() {
    int16_t got = 0;
    TEST_ASSERT_FALSE(logfmt::countsFromFloat(40000.0, got));    // past int16
    TEST_ASSERT_FALSE(logfmt::countsFromFloat(-40000.0, got));
    TEST_ASSERT_FALSE(logfmt::countsFromFloat(1e30, got));       // no fixed rendering
    TEST_ASSERT_FALSE(logfmt::countsFromFloat(0.0 / 0.0, got));  // nan
    TEST_ASSERT_FALSE(logfmt::countsFromFloat(1.0 / 0.0, got));  // inf
}

void test_surface_capacity_keeps_the_newest_and_never_renumbers() {
    // Room: rewrite nothing. Rewriting a whole file to add one record is how a
    // log ends up lost when the rewrite is interrupted.
    TEST_ASSERT_EQUAL(0, compactSurface(0, SURFACE_MAX_RECORDS));
    TEST_ASSERT_EQUAL(0, compactSurface(SURFACE_MAX_RECORDS - 1, SURFACE_MAX_RECORDS));
    // Full: exactly one goes, the oldest.
    TEST_ASSERT_EQUAL(1, compactSurface(SURFACE_MAX_RECORDS, SURFACE_MAX_RECORDS));
    // Overflow by k (records written while the rewrite failed): k + 1.
    TEST_ASSERT_EQUAL(11, compactSurface(SURFACE_MAX_RECORDS + 10, SURFACE_MAX_RECORDS));
    // A nonsense capacity sheds nothing rather than everything: losing the whole
    // history is worse than one oversized file.
    TEST_ASSERT_EQUAL(0, compactSurface(50, 0));
    TEST_ASSERT_EQUAL(0, compactSurface(50, -1));

    // Applied to a real sequence: the newest survives, the oldest goes, and no
    // record is renumbered — a reader can still see the gap in the seq range.
    constexpr int kN = 10, kCap = 4;
    uint32_t seq[kN];
    for (int i = 0; i < kN; ++i) seq[i] = static_cast<uint32_t>(i + 1);
    const int drop = compactSurface(kN, kCap);
    TEST_ASSERT_EQUAL(7, drop);
    const int kept = kN - drop;
    TEST_ASSERT_EQUAL(kCap - 1, kept);          // room for the record being written
    TEST_ASSERT_EQUAL(8u, seq[drop]);           // oldest kept
    TEST_ASSERT_EQUAL(10u, seq[kN - 1]);        // newest kept, same number it was born with
}


void test_a_row_holds_the_digits_the_wire_gave_for_the_same_sample() {
    // The claim a diver can actually check: the number in his dive file is the
    // number the handset was answered at that moment. Until 2026-11-24 the two
    // could be one count apart — the value was stored with a truncating cast and
    // the CSV printed that integer, while `T,` printed the float. This is the
    // witness the soak rig could never be, because the rig used to re-implement
    // the same casts line for line and would have followed any bug.
    struct Src { const char* what; float v; int decimals; const char* digits; };
    const Src kSrc[] = {
        {"depth m",     20.30f, 2, "20.30"},   // the measured miss: 2029 cm stored
        {"depth m",      2.53f, 2,  "2.53"},
        {"voted bar",    0.29f, 3, "0.290"},
        {"setpoint bar", 0.55f, 3, "0.550"},
        {"cell bar",     0.66f, 3, "0.660"},
        // (the ceiling row is the one the old double-literal site lost a centimetre on)
        {"ceiling m",   11.15f, 2, "11.15"},
        {"battery V",   11.90f, 3, "11.900"},
    };
    for (const Src& c : kSrc) {
        const long long stored = numfmt::scaled(c.v, c.decimals);

        char printed[32];
        const char* got = numfmt::fixedScaled(printed, sizeof(printed), stored, c.decimals);
        char msg[160];
        std::snprintf(msg, sizeof(msg), "%s %.4f: stored %lld prints as '%s', want '%s'",
                      c.what, static_cast<double>(c.v), stored, got, c.digits);
        TEST_ASSERT_EQUAL_STRING_MESSAGE(c.digits, got, msg);   // hand-typed literal

        char wire[32];
        const char* direct = numfmt::fixed(wire, sizeof(wire), c.v, c.decimals);
        TEST_ASSERT_EQUAL_STRING_MESSAGE(c.digits, direct,
                                         "the handset's own rendering of the same sample "
                                         "disagrees with the digits this case pins");
    }

    // End to end through the row formatter, not only the number helpers.
    logfmt::Snapshot s{};
    s.tSec = 42;
    s.depthCm = static_cast<int32_t>(numfmt::scaled(20.30f, 2));
    s.votedX1000 = static_cast<int16_t>(numfmt::scaled(0.29f, 3));
    s.cellX1000[0] = static_cast<int16_t>(numfmt::scaled(0.66f, 3));
    s.setpointX1000 = static_cast<int16_t>(numfmt::scaled(0.55f, 3));
    s.ceilingCm = static_cast<int16_t>(numfmt::scaled(11.15f, 2));
    s.battX1000 = static_cast<uint16_t>(numfmt::scaled(11.90f, 3));
    char line[256];
    const size_t n = logfmt::snapshotCsvLine(line, sizeof(line), s);
    TEST_ASSERT_GREATER_THAN_INT_MESSAGE(40, static_cast<int>(n), "the row formatter wrote nothing");
    TEST_ASSERT_NOT_NULL_MESSAGE(std::strstr(line, ",20.30,0.290,"),
                                 (std::string("an S, row does not carry the depth and vote the "
                                              "handset was answered; row was: ") + line).c_str());
    TEST_ASSERT_NOT_NULL_MESSAGE(std::strstr(line, "1115"),
                                 (std::string("ceiling_cm is not 1115 for a 11.15 m ceiling; row was: ")
                                  + line).c_str());
    // The defect, pinned: 20.30 m used to reach the file as 20.29.
    TEST_ASSERT_NULL_MESSAGE(std::strstr(line, ",20.29,"),
                             "a truncating cast is back: 20.30 m stored 2029 cm again");
}

// ---- the shared row fill (src/log/rowfill.h) -------------------------------
// These cases are the point of the refactor. Until now the conversions that turn
// the chain's floats into the integers a dive file holds lived in main.cpp and
// session_wiring.cpp, neither of which the native env compiles, so the only thing
// grading them was the soak's hand-written copy of them. Everything below runs the
// function the firmware calls.

namespace {

logfmt::RowInputs benchRow() {
    logfmt::RowInputs ri{};
    ri.nowMs = 1500000u;
    ri.diveStartMs = 0u;
    ri.smoothedDepthM = 20.30f;
    ri.haveVote = true;
    ri.shownPpo2 = 0.55f;
    ri.cellBar[0] = 0.549f; ri.cellBar[1] = 0.55f; ri.cellBar[2] = 0.551f;
    ri.setpointBar = 0.70f;
    ri.diveActive = true;
    ri.autoMode = true;
    ri.alarmLogBits = 0x0010;
    ri.cnsPeakPct = 13;
    ri.vbat = 11.90f;
    ri.pulsesNow = 1000;
    ri.pulsesPrev = 940;
    return ri;
}

deco::Zhl16Ccr loadedEngine() {
    deco::Config dc;               // the shipped defaults: GF 30/85, sea, CCR
    deco::Zhl16Ccr eng(dc);
    eng.reset();
    // 20 min at 40 m on a 1.30 bar setpoint: deep enough that the ceiling, the
    // NDL and the schedule are all unambiguously non-zero afterwards.
    for (int i = 0; i < 120; i++) eng.step(10.0, eng.depthToBar(40.0), 1.30);
    return eng;
}

}  // namespace

// The defect this whole module guards against, at the exact inputs that caught it
// on the bench: a truncating cast or a `* 100.0` double product lands one count
// under the integer the renderer prints, and the dive file stops being comparable
// with what the handset was answered.
void test_the_fill_stores_the_count_the_renderer_prints() {
    logfmt::Snapshot s{};
    logfmt::RowInputs ri = benchRow();
    ri.smoothedDepthM = 11.15f;    // 11.15f is 11.1499996...: `* 100.0` gives 1114
    logfmt::fillSnapshot(s, ri);
    TEST_ASSERT_EQUAL_INT32_MESSAGE(1115, s.depthCm,
                                    "11.15 m stored as something other than 1115 cm — a "
                                    "hand-rolled product is back in the row fill");
    TEST_ASSERT_EQUAL_INT16_MESSAGE(550, s.votedX1000,
                                    "0.55 bar stored as something other than 550 (0.55f is "
                                    "0.54999999...)");
    TEST_ASSERT_EQUAL_INT16(549, s.cellX1000[0]);
    TEST_ASSERT_EQUAL_INT16(551, s.cellX1000[2]);
    TEST_ASSERT_EQUAL_INT16(700, s.setpointX1000);
    TEST_ASSERT_EQUAL_UINT16(11900, s.battX1000);
    // Whole seconds since dive start: the row's clock is a millisecond difference
    // divided, not a float the caller rounded.
    TEST_ASSERT_EQUAL_UINT32(1500, s.tSec);

    logfmt::Snapshot s2{};
    logfmt::RowInputs ri2 = benchRow();
    ri2.diveStartMs = 245000u;     // a dive that started mid-uptime
    logfmt::fillSnapshot(s2, ri2);
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(1255, s2.tSec,
                                     "tSec is not the dive-relative second");
    TEST_ASSERT_EQUAL_INT32(2030, s2.depthCm);   // 20.30 m, the old 2029 bug
}

// End to end through the row writer: the number the fill stored is the number a
// diver reads in the CSV, which is the only agreement that matters.
void test_the_fill_stores_the_digits_the_wire_prints() {
    logfmt::Snapshot s{};
    logfmt::RowInputs ri = benchRow();
    ri.smoothedDepthM = 20.30f;
    logfmt::fillSnapshot(s, ri);
    char line[256];
    const size_t n = logfmt::snapshotCsvLine(line, sizeof(line), s);
    TEST_ASSERT_GREATER_THAN_INT_MESSAGE(40, static_cast<int>(n), "the row formatter wrote nothing");
    TEST_ASSERT_NOT_NULL_MESSAGE(std::strstr(line, ",20.30,0.550,0.549,0.550,0.551,0.700,"),
                                 (std::string("a filled row does not print the numbers the "
                                              "handset was answered; row was: ") + line).c_str());
    TEST_ASSERT_NOT_NULL_MESSAGE(std::strstr(line, "0x0010"),
                                 (std::string("the alarm word is not the one the fill was "
                                              "given; row was: ") + line).c_str());
    TEST_ASSERT_NULL_MESSAGE(std::strstr(line, ",20.29,"),
                             "a truncating conversion is back: 20.30 m stored 2029 cm again");
}

// The deco columns are measured at the depth the CHAIN believed — the same value
// the row's depth column carries — and the absence of a vote is recorded in the
// ndl sentinel, not by zeroing the last measurement.
void test_the_deco_columns_are_measured_at_the_chains_depth() {
    deco::Zhl16Ccr eng = loadedEngine();
    logfmt::Snapshot s{};
    logfmt::RowInputs ri = benchRow();
    ri.smoothedDepthM = 40.0f;     // the chain's held depth; the diver may be elsewhere
    ri.shownPpo2 = 1.30f;
    ri.deco = &eng;
    logfmt::fillSnapshot(s, ri);

    TEST_ASSERT_TRUE_MESSAGE(s.ceilingCm > 0, "20 min at 40 m demanded no ceiling: the engine "
                                              "never reached the row");
    TEST_ASSERT_EQUAL_INT16_MESSAGE(static_cast<int16_t>(numfmt::scaled(eng.ceilingM(), 2)),
                                    s.ceilingCm,
                                    "ceiling_cm is not numfmt::scaled(ceilingM(), 2): the row "
                                    "and the row writer disagree by a rounding");
    TEST_ASSERT_TRUE_MESSAGE(s.decoTotalMin > 0, "the schedule never reached the row");
    TEST_ASSERT_EQUAL_INT16_MESSAGE(static_cast<int16_t>(eng.ndlMinutes(eng.depthToBar(40.0), 1.30)),
                                    s.ndlMin,
                                    "ndl is not ndlMinutes(depthToBar(the row's own depth))");
    // The `ceiling_m` column a v1 parser reads is the integer metres of that same
    // centimetre value — never a second, differently rounded measurement.
    char line[256];
    logfmt::snapshotCsvLine(line, sizeof(line), s);
    char want[32];
    std::snprintf(want, sizeof(want), ",%d,%d,%d,", static_cast<int>(s.ceilingCm) / 100,
                  static_cast<int>(s.ndlMin), static_cast<int>(s.decoTotalMin));
    TEST_ASSERT_NOT_NULL_MESSAGE(std::strstr(line, want),
                                 (std::string("the row does not print ceiling_m/ndl/deco from "
                                              "the values the fill stored; want '") + want +
                                  "' in: " + line).c_str());
    std::snprintf(want, sizeof(want), ",%d\n", static_cast<int>(s.ceilingCm));
    TEST_ASSERT_NOT_NULL_MESSAGE(std::strstr(line, want),
                                 (std::string("the appended ceiling_cm is not the value the "
                                              "fill stored: ") + line).c_str());

    // No vote: the NDL column says "nothing to total" (the same 480 the engine
    // saturates at), while the vote the chain last made stays in its column.
    logfmt::Snapshot q{};
    logfmt::RowInputs qi = ri;
    qi.haveVote = false;
    logfmt::fillSnapshot(q, qi);
    TEST_ASSERT_EQUAL_INT16_MESSAGE(480, q.ndlMin,
                                    "a row with no vote does not carry the 480 sentinel — an "
                                    "unanswerable NDL and an unlimited one are now the same "
                                    "number");
    TEST_ASSERT_EQUAL_INT16(1300, q.votedX1000);
    TEST_ASSERT_EQUAL_INT16(s.ceilingCm, q.ceilingCm);   // the ceiling is not a voted quantity
}

// The event-driven row's documented shape: no deco picture, so those three columns
// are zero. Pinned, because it is a difference a reader can be misled by and
// because "fixing" it must be a decision about a documented column, not an accident.
void test_a_row_with_no_deco_picture_leaves_the_deco_columns_zero() {
    deco::Zhl16Ccr eng = loadedEngine();
    logfmt::Snapshot s{};
    logfmt::RowInputs ri = benchRow();
    ri.deco = nullptr;
    logfmt::fillSnapshot(s, ri);
    TEST_ASSERT_EQUAL_INT16(0, s.ceilingCm);
    TEST_ASSERT_EQUAL_INT16(0, s.ndlMin);
    TEST_ASSERT_EQUAL_INT16(0, s.decoTotalMin);
    // Everything else is still filled: an event row is a measurement row, not a
    // stub. (A ceiling of 0 here reads as "no stop demanded", which is what needs
    // deciding before these columns are wired.)
    TEST_ASSERT_EQUAL_INT32(2030, s.depthCm);
    TEST_ASSERT_EQUAL_UINT16(0x0010, s.alarmBits);
    TEST_ASSERT_TRUE_MESSAGE(eng.ceilingM() > 0.0,
                             "the engine in this case demands no stop, so the zeros below "
                             "would prove nothing");
}

// The pulses column is a difference of two absolute counters, widened in uint32
// and truncated into the uint16 the record holds — the behaviour the firmware has
// always had, now named where a reader can see it.
void test_the_pulses_column_is_a_window_delta() {
    logfmt::Snapshot s{};
    logfmt::RowInputs ri = benchRow();
    ri.pulsesNow = 5000; ri.pulsesPrev = 4940;
    logfmt::fillSnapshot(s, ri);
    TEST_ASSERT_EQUAL_UINT16(60, s.pulses);

    logfmt::Snapshot w{};
    logfmt::RowInputs wi = benchRow();
    wi.pulsesNow = 65536u + 7u;    // 21+ min of continuous pulsing: wraps, as it always did
    wi.pulsesPrev = 0u;
    logfmt::fillSnapshot(w, wi);
    TEST_ASSERT_EQUAL_UINT16_MESSAGE(7, w.pulses,
                                     "the pulse window no longer truncates where it always "
                                     "did — a silent change to an existing column");

    logfmt::Snapshot z{};
    logfmt::RowInputs zi = benchRow();
    zi.pulsesNow = 0u; zi.pulsesPrev = 0u;   // the event row has no window of its own
    logfmt::fillSnapshot(z, zi);
    TEST_ASSERT_EQUAL_UINT16(0, z.pulses);
}

// mode is the dive state, not the autopilot flag: a row that lands outside a dive
// (an event drained after DIVE END) is neither auto nor manual.
void test_mode_is_the_dive_state_not_just_the_autopilot() {
    logfmt::Snapshot a{};
    logfmt::RowInputs ai = benchRow();
    ai.diveActive = true; ai.autoMode = true;
    logfmt::fillSnapshot(a, ai);
    TEST_ASSERT_EQUAL_UINT8(1, a.mode);

    logfmt::Snapshot m{};
    logfmt::RowInputs mi = benchRow();
    mi.diveActive = true; mi.autoMode = false;
    logfmt::fillSnapshot(m, mi);
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(2, m.mode,
                                    "a manual dive row is not mode 2 — the soak and the web "
                                    "page both read this column as the dive state");

    logfmt::Snapshot o{};
    logfmt::RowInputs oi = benchRow();
    oi.diveActive = false; oi.autoMode = true;
    logfmt::fillSnapshot(o, oi);
    TEST_ASSERT_EQUAL_UINT8(0, o.mode);
}

// The alarm column is the LOG view, and the record still carries bits 8..9
// through the borrowed flag bits (finding #5: an ACK must not be able to delete
// a marker from a row that was written while the condition stood).
void test_the_row_carries_the_log_alarm_view_whole() {
    logfmt::Snapshot s{};
    logfmt::RowInputs ri = benchRow();
    ri.alarmLogBits = 0x0180;      // BIT_SENSOR_STALE plus BIT_CNS_HIGH: the top half borrows
    logfmt::fillSnapshot(s, ri);
    TEST_ASSERT_EQUAL_UINT16(0x0180, s.alarmBits);

    uint8_t bytes[logfmt::SNAPSHOT_SIZE];
    logfmt::packSnapshot(s, bytes);
    logfmt::Snapshot back{};
    TEST_ASSERT_TRUE(logfmt::unpackSnapshot(bytes, back));
    TEST_ASSERT_EQUAL_UINT16_MESSAGE(0x0180, back.alarmBits,
                                     "the row's alarm marker lost a bit on the way to flash");
}

// CNS is the dive's running peak, passthrough: a row and the `M,` summary beside
// it must be the same number, so the fill may not round or clamp it again.
void test_the_cns_column_is_the_dive_peak_untouched() {
    logfmt::Snapshot s{};
    logfmt::RowInputs ri = benchRow();
    ri.cnsPeakPct = 13;
    logfmt::fillSnapshot(s, ri);
    TEST_ASSERT_EQUAL_UINT8(13, s.cnsPct);
    ri.cnsPeakPct = 254;           // DiveTotals' clamp, not the row's
    logfmt::fillSnapshot(s, ri);
    TEST_ASSERT_EQUAL_UINT8(254, s.cnsPct);
}

// Two columns of the row format have never had a writer. Keeping them unwritable
// from RowInputs is deliberate: giving `temp_c` or `flags` a value is a format
// decision, and a half-wired sensor arriving by accident would put a number a
// reader trusts into a byte nothing has ever defined.
void test_temp_and_flags_stay_unwritten_until_somebody_owns_them() {
    logfmt::Snapshot s{};
    s.tempC = 0; s.flags = 0;
    logfmt::RowInputs ri = benchRow();
    ri.smoothedDepthM = -3.0f;     // nothing here reaches those two columns
    logfmt::fillSnapshot(s, ri);
    TEST_ASSERT_EQUAL_INT32(-300, s.depthCm);   // a signed depth still scales
    TEST_ASSERT_EQUAL_INT8_MESSAGE(0, s.tempC,
                                   "temp_c gained a value with no sensor behind it");
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(0, s.flags,
                                    "the flags byte gained a value: it duplicates what the "
                                    "E, rows already say, and nothing defined it");
}

// The marker bytes, as a diver's download shows them (the requirement to say
// WHICH existing bytes mark a held ppO2, in a form that fails if they stop
// marking it): a row written while the vote is failed keeps the shown number in
// the `voted` column, carries BIT_SENSOR_FAIL (bit 3) in its alarm word, and
// states ndl 480 = "no vote, nothing to total". No new column anywhere.
void test_a_failed_vote_row_marks_the_held_number_with_existing_bytes() {
    deco::Zhl16Ccr eng = loadedEngine();  // a loaded engine: the sentinel must beat a real NDL
    logfmt::Snapshot s{};
    logfmt::RowInputs ri = benchRow();
    ri.shownPpo2 = 1.05f;              // the last believable vote
    ri.haveVote = false;               // and it is not a measurement any more
    ri.alarmLogBits = (1u << 3);       // alarms::BIT_SENSOR_FAIL, the LOG view
    ri.smoothedDepthM = 40.0f;
    ri.deco = &eng;
    logfmt::fillSnapshot(s, ri);

    char line[256];
    logfmt::snapshotCsvLine(line, sizeof(line), s);
    TEST_ASSERT_NOT_NULL_MESSAGE(std::strstr(line, ",1.050,"),
                                 (std::string("the ppO2 column of a failed-vote row is not the "
                                              "held value the handset was showing; row was: ") +
                                  line).c_str());
    TEST_ASSERT_NOT_NULL_MESSAGE(std::strstr(line, "0x0008"),
                                 (std::string("a held ppO2 reached the dive file with no "
                                              "BIT_SENSOR_FAIL beside it; row was: ") + line).c_str());
    TEST_ASSERT_NULL_MESSAGE(std::strstr(line, ",0.000,"),
                             "the failed vote wrote a 0.000 measurement into the row again");
    TEST_ASSERT_EQUAL_INT16_MESSAGE(480, s.ndlMin,
                                    "no 480 sentinel: the held ppO2 was allowed to compute an NDL");
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_snapshot_roundtrip_and_size);
    RUN_TEST(test_writers_stay_inside_the_record);
    RUN_TEST(test_record_crc_detects_a_torn_write);
    RUN_TEST(test_high_alarm_bits_survive_the_record);
    RUN_TEST(test_snapshot_version_gate);
    RUN_TEST(test_header_roundtrip_and_version_gate);
    RUN_TEST(test_header_reads_a_pre_sequence_file_as_unknown);
    RUN_TEST(test_sequence_saturates_and_never_wraps);
    RUN_TEST(test_eviction_rotates_when_there_is_no_clock);
    RUN_TEST(test_unknown_age_is_evicted_before_any_sequenced_dive);
    RUN_TEST(test_utc_rendering_is_exact_and_says_when_there_is_no_clock);
    RUN_TEST(test_scan_walks_interleaved_records_in_order);
    RUN_TEST(test_scan_stops_at_the_first_corrupt_record);
    RUN_TEST(test_scan_rejects_unknown_tags_and_foreign_headers);
    RUN_TEST(test_meta_crc_detects_corruption);
    RUN_TEST(test_event_roundtrip);
    RUN_TEST(test_dive_identity_line);
    RUN_TEST(test_snapshot_row_is_the_documented_shape);
    RUN_TEST(test_event_and_meta_rows);
    RUN_TEST(test_rows_refuse_rather_than_truncate);
    RUN_TEST(test_both_export_paths_are_the_same_bytes);
    RUN_TEST(test_pack_image_refuses_a_short_buffer);
    RUN_TEST(test_surface_record_roundtrip_and_stride);
    RUN_TEST(test_surface_record_offsets_are_the_documented_bytes);
    RUN_TEST(test_surface_record_crc_and_version_gates);
    RUN_TEST(test_surface_names_are_stable_and_unknown_ids_keep_their_number);
    RUN_TEST(test_surface_rows_are_the_documented_shape);
    RUN_TEST(test_surface_rows_refuse_rather_than_truncate);
    RUN_TEST(test_surface_counts_round_the_way_the_wire_prints_them);
    RUN_TEST(test_surface_counts_refuse_what_they_cannot_hold);
    RUN_TEST(test_surface_capacity_keeps_the_newest_and_never_renumbers);
    RUN_TEST(test_a_row_holds_the_digits_the_wire_gave_for_the_same_sample);
    RUN_TEST(test_the_fill_stores_the_count_the_renderer_prints);
    RUN_TEST(test_the_fill_stores_the_digits_the_wire_prints);
    RUN_TEST(test_the_deco_columns_are_measured_at_the_chains_depth);
    RUN_TEST(test_a_row_with_no_deco_picture_leaves_the_deco_columns_zero);
    RUN_TEST(test_the_pulses_column_is_a_window_delta);
    RUN_TEST(test_mode_is_the_dive_state_not_just_the_autopilot);
    RUN_TEST(test_the_row_carries_the_log_alarm_view_whole);
    RUN_TEST(test_the_cns_column_is_the_dive_peak_untouched);
    RUN_TEST(test_temp_and_flags_stay_unwritten_until_somebody_owns_them);
    RUN_TEST(test_a_failed_vote_row_marks_the_held_number_with_existing_bytes);
    return UNITY_END();
}
