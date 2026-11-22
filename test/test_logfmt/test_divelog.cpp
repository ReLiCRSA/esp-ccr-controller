#include <unity.h>

#include <cstdio>
#include <cstring>

#include "log/divelog.h"

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
    logfmt::packHeader(1700000000u, 3, img + p);
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
    logfmt::packHeader(1700000000u, 7, head);
    uint32_t epoch = 0;
    uint8_t slot = 0;
    TEST_ASSERT_TRUE(logfmt::unpackHeader(head, epoch, slot));
    TEST_ASSERT_EQUAL_UINT32(1700000000u, epoch);
    TEST_ASSERT_EQUAL_UINT8(7, slot);
    head[9] ^= 0xFF;   // one flipped byte in the header
    TEST_ASSERT_FALSE(logfmt::unpackHeader(head, epoch, slot));
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

void test_csv_output_shape_and_overflow() {
    DiveMeta m{};
    m.durationSec = 120; m.maxDepthCm = 500;
    m.minVotedX1000 = 400; m.maxVotedX1000 = 420;
    m.gfLow = 30; m.gfHigh = 85;
    m.snapshotCount = 2; m.eventCount = 1;
    const Snapshot snaps[2] = {sampleSnap(), sampleSnap()};
    const Event events[1] = {{60, EVT_ALARM_RAISED, 1}};

    char buf[2048];
    const size_t n = toCsv(m, snaps, 2, events, 1, buf, sizeof(buf));
    TEST_ASSERT_TRUE(n > 100);
    {   // the CSV declares the codec version it was written with
        char want[32];
        snprintf(want, sizeof(want), "ESP-CCR dive record,%u", logfmt::DIVE_VERSION);
        TEST_ASSERT_EQUAL_STRING_LEN(want, buf, strlen(want));
    }
    TEST_ASSERT_NOT_NULL(std::strstr(buf, "\nS,t_s,depth_m,ppo2,"));
    TEST_ASSERT_NOT_NULL(std::strstr(buf, "S,3661,40.23,0.402"));
    TEST_ASSERT_NOT_NULL(std::strstr(buf, "E,60,1,1\n"));
    TEST_ASSERT_EQUAL('\n', buf[n - 1]);

    // tight buffer must fail cleanly, not overflow
    char tiny[10];
    TEST_ASSERT_EQUAL(0, toCsv(m, snaps, 2, events, 1, tiny, sizeof(tiny)));
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_snapshot_roundtrip_and_size);
    RUN_TEST(test_writers_stay_inside_the_record);
    RUN_TEST(test_record_crc_detects_a_torn_write);
    RUN_TEST(test_high_alarm_bits_survive_the_record);
    RUN_TEST(test_snapshot_version_gate);
    RUN_TEST(test_header_roundtrip_and_version_gate);
    RUN_TEST(test_scan_walks_interleaved_records_in_order);
    RUN_TEST(test_scan_stops_at_the_first_corrupt_record);
    RUN_TEST(test_scan_rejects_unknown_tags_and_foreign_headers);
    RUN_TEST(test_meta_crc_detects_corruption);
    RUN_TEST(test_event_roundtrip);
    RUN_TEST(test_csv_output_shape_and_overflow);
    return UNITY_END();
}
