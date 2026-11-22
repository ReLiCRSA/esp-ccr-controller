#include <unity.h>

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
    s.mode = 1; s.alarmBits = 0b100100; s.cnsPct = 37;
    s.ceilingCm = 900; s.ndlMin = 480; s.decoTotalMin = 12;
    s.pulses = 60000; s.battX1000 = 11840;
    s.tempC = -3; s.flags = 0b101;
    return s;
}

void test_snapshot_roundtrip_and_size() {
    static_assert(sizeof(Snapshot) >= 27, "fields exceed 32-byte budget");
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
    TEST_ASSERT_EQUAL_UINT8(a.alarmBits, b.alarmBits);
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
    TEST_ASSERT_EQUAL_STRING_LEN("ESP-CCR dive record,1", buf, 21);
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
    RUN_TEST(test_snapshot_version_gate);
    RUN_TEST(test_meta_crc_detects_corruption);
    RUN_TEST(test_event_roundtrip);
    RUN_TEST(test_csv_output_shape_and_overflow);
    return UNITY_END();
}
