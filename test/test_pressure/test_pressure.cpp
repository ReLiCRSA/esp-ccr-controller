#include <unity.h>

#include <cmath>
#include <cstring>
#include <deque>
#include <map>
#include <vector>

#include "sensors/bmp085.h"
#include "sensors/ms5837.h"

using press::Bmp085;
using press::Ms5837;

void setUp() {}
void tearDown() {}

// ---------------------------------------------------------------------------
// Fake bus: canned per-register read queues + fault injection.
// A read with nothing queued NACKs — unprogrammed reads must fail loudly,
// exactly like an absent sensor.
// ---------------------------------------------------------------------------
class FakeBus : public press::Transport {
public:
    void queue(uint8_t reg, std::vector<uint8_t> bytes) { q_[reg].push_back(std::move(bytes)); }
    // Sticky response: like PROM — same bytes for every read, never consumed.
    void queueSticky(uint8_t reg, std::vector<uint8_t> bytes) { sticky_[reg] = std::move(bytes); }
    int failReads = 0, failWrites = 0;
    std::vector<uint8_t> writes;

    bool writeByte(uint8_t, uint8_t v) override {
        if (failWrites > 0) { failWrites--; return false; }
        writes.push_back(v);
        return true;
    }
    bool writeRegData(uint8_t, uint8_t reg, uint8_t data) override {
        if (failWrites > 0) { failWrites--; return false; }
        writes.push_back(reg);
        writes.push_back(data);
        return true;
    }
    bool writeThenRead(uint8_t, uint8_t reg, uint8_t* out, uint8_t n) override {
        if (failReads > 0) { failReads--; return false; }
        auto st = sticky_.find(reg);
        if (st != sticky_.end()) {
            if (st->second.size() != n) return false;
            std::memcpy(out, st->second.data(), n);
            return true;
        }
        auto& dq = q_[reg];
        if (dq.empty()) return false;
        const std::vector<uint8_t> resp = dq.front();
        dq.pop_front();
        if (resp.size() != n) return false;
        std::memcpy(out, resp.data(), n);
        return true;
    }
    bool sawWrite(uint8_t v) const {
        for (uint8_t w : writes) if (w == v) return true;
        return false;
    }

private:
    std::map<uint8_t, std::deque<std::vector<uint8_t>>> q_;
    std::map<uint8_t, std::vector<uint8_t>> sticky_;
};

static std::vector<uint8_t> adc24(uint32_t v) {
    return {static_cast<uint8_t>(v >> 16), static_cast<uint8_t>(v >> 8), static_cast<uint8_t>(v)};
}
static std::vector<uint8_t> adc16(uint32_t v) {
    return {static_cast<uint8_t>(v >> 8), static_cast<uint8_t>(v)};
}

// ===========================================================================
// MS5837-30BA — datasheet REV C worked example:
// C1..C6 = 34982 36352 20328 22354 26646 26146, D1=4958179, D2=6815414
// expected: 19.81 C, 3999.8 mbar (3.9998 bar)
// ===========================================================================
// (the arrays below are built per test so a corrupt-PROM case can shadow them)

static bool loadProm(FakeBus& bus, const uint16_t prom[8]) {
    for (int i = 0; i < 8; ++i)
        bus.queueSticky(static_cast<uint8_t>(0xA0 + 2 * i),
                        {static_cast<uint8_t>(prom[i] >> 8), static_cast<uint8_t>(prom[i])});
    return true;
}

static uint16_t findValidCrcWord0(const uint16_t c1[6]) {
    // Exactly one CRC nibble validates the PROM; brute-force it and prove
    // the CRC-4 implementation is discriminating (1 of 16).
    int hits = 0;
    uint16_t good = 0;
    for (uint16_t nib = 0; nib < 16; ++nib) {
        uint16_t p[8] = {static_cast<uint16_t>(nib << 12), c1[0], c1[1], c1[2],
                         c1[3], c1[4], c1[5], 0};
        if (Ms5837::checkPromCrc(p)) { good = p[0]; hits++; }
    }
    TEST_ASSERT_EQUAL(1, hits);
    return good;
}

void test_ms5837_boot_and_datasheet_vector() {
    FakeBus bus;
    const uint16_t c1[6] = {34982, 36352, 20328, 22354, 26646, 26146};
    const uint16_t w0 = findValidCrcWord0(c1);
    uint16_t prom[8] = {w0, c1[0], c1[1], c1[2], c1[3], c1[4], c1[5], 0};
    TEST_ASSERT_TRUE(Ms5837::checkPromCrc(prom));

    Ms5837 sensor{bus};
    // boot is spread over ticks: reset, 15 ms PROM load, then reads
    TEST_ASSERT_FALSE(sensor.begin(0));    // reset issued
    TEST_ASSERT_TRUE(bus.sawWrite(0x1E));
    TEST_ASSERT_FALSE(sensor.begin(10));   // still waiting on PROM load
    loadProm(bus, prom);
    TEST_ASSERT_TRUE(sensor.begin(16));    // PROM load done, words read, READY
    TEST_ASSERT_TRUE(sensor.begin(17));    // idempotent once READY
    TEST_ASSERT_TRUE(sensor.healthy());

    bus.queue(0x00, adc24(4958179));  // D1
    bus.queue(0x00, adc24(6815414));  // D2

    sensor.start(100);
    TEST_ASSERT_TRUE(bus.sawWrite(0x48));       // conv D1 OSR4096
    TEST_ASSERT_FALSE(sensor.poll(105));        // conversion not done yet
    TEST_ASSERT_FALSE(sensor.poll(110));        // D1 harvested, D2 kicked
    TEST_ASSERT_TRUE(bus.sawWrite(0x58));       // conv D2 OSR4096
    TEST_ASSERT_FALSE(sensor.poll(119));        // not before deadline
    TEST_ASSERT_TRUE(sensor.poll(120));         // complete

    TEST_ASSERT_FLOAT_WITHIN(0.05f, 19.81f, sensor.temperatureC());
    TEST_ASSERT_FLOAT_WITHIN(0.005f, 3.9998f, sensor.pressureBar());
    TEST_ASSERT_EQUAL_UINT32(120, sensor.lastGoodMs());
}

void test_ms5837_crc_corruption_is_refused() {
    // Realistic corruption model: memory/bus flips change a coefficient,
    // the factory CRC nibble stays where it was. (CRC-4 guards integrity
    // against random corruption — an adversary who may rewrite the CRC is
    // out of scope for a depth sensor.)
    FakeBus bus;
    const uint16_t c1[6] = {34982, 36352, 20328, 22354, 26646, 26146};
    const uint16_t w0 = findValidCrcWord0(c1);
    uint16_t good[8] = {w0, c1[0], c1[1], c1[2], c1[3], c1[4], c1[5], 0};
    TEST_ASSERT_TRUE(Ms5837::checkPromCrc(good));

    uint16_t bad[8];
    std::memcpy(bad, good, sizeof(bad));
    bad[3] += 5;  // one coefficient quietly corrupted
    TEST_ASSERT_FALSE(Ms5837::checkPromCrc(bad));

    loadProm(bus, bad);
    Ms5837 sensor{bus};
    bool booted = false;
    for (uint32_t t = 0; t < 20000 && !booted; t += 100) booted = sensor.begin(t);
    TEST_ASSERT_FALSE(booted);              // never becomes READY
    TEST_ASSERT_FALSE(sensor.healthy());
    TEST_ASSERT_TRUE(sensor.state() != Ms5837::State::READY);
}

void test_ms5837_bus_fault_backoff_and_recovery() {
    FakeBus bus;
    const uint16_t c1[6] = {34982, 36352, 20328, 22354, 26646, 26146};
    uint16_t prom[8] = {findValidCrcWord0(c1), c1[0], c1[1], c1[2], c1[3], c1[4], c1[5], 0};

    Ms5837 sensor{bus};
    bus.failWrites = 1;                      // dead bus on first reset
    TEST_ASSERT_FALSE(sensor.begin(0));

    // backoff: immediate retries must not spam the bus
    const size_t writesBefore = bus.writes.size();
    for (uint32_t t = 1; t < 400; t += 50) sensor.begin(t);
    TEST_ASSERT_EQUAL_UINT32(writesBefore, bus.writes.size());

    loadProm(bus, prom);
    bool booted = false;
    for (uint32_t t = 500; t < 3000 && !booted; t += 10) booted = sensor.begin(t);
    TEST_ASSERT_TRUE(booted);

    // mid-conversion NACKs degrade health, then recovery heals
    bus.failReads = 10;
    const size_t writesAtStart = bus.writes.size();
    for (uint32_t t = 3000; t < 60000 && sensor.healthy(); t += 100) {
        sensor.start(t);
        for (uint32_t p = t; p < t + 100; ++p)
            if (sensor.poll(p)) break;
    }
    // Backoff must be respected: ~5 failing attempts max, not one write per
    // 100 ms tick for a minute (that would be 600 bus writes).
    const size_t attempts = bus.writes.size() - writesAtStart;
    TEST_ASSERT_TRUE(attempts <= 8);

    // bus recovers
    bus.failReads = 0;
    bus.queue(0x00, adc24(4958179));
    bus.queue(0x00, adc24(6815414));
    bool done = false;
    for (uint32_t t = 60000; t < 70000 && !done; t += 50) {
        sensor.begin(t);                     // re-init if it dropped to BUS_FAULT
        sensor.start(t);
        for (uint32_t p = t; p < t + 200 && !done; ++p) done = sensor.poll(p);
    }
    TEST_ASSERT_TRUE(done);
    TEST_ASSERT_TRUE(sensor.healthy());
    TEST_ASSERT_FLOAT_WITHIN(0.005f, 3.9998f, sensor.pressureBar());
}

// ===========================================================================
// BMP085 — datasheet worked example: cal AC1=408 ... MD=2868,
// UT=27898, UP=23843 (OSS0 raw 16-bit), expected 15.0 C and 69964 Pa.
// Driver runs OSS=2: same raw bus bytes (UP<<8 in the 24-bit register) must
// reproduce the same pressure through the OSS-corrected math.
// ===========================================================================
static Bmp085::Cal datasheetCal() {
    Bmp085::Cal c;
    c.ac1 = 408; c.ac2 = -72; c.ac3 = -14383;
    c.ac4 = 32741; c.ac5 = 32757; c.ac6 = 23153;
    c.b1 = 6190; c.b2 = 4; c.mb = -32768; c.mc = -8711; c.md = 2868;
    return c;
}

void test_bmp085_math_datasheet_vector() {
    const Bmp085::Cal c = datasheetCal();
    float bar = 0, tc = 0;
    int32_t b5 = 0;
    Bmp085::compensate(c, Bmp085::Raw{27898, 23843}, 0, bar, tc, b5);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 15.0f, tc);
    // 69964 Pa expected; C++ zero-trunc division vs datasheet floor gives
    // intermediate +/-1s -> accept +/-10 Pa.
    TEST_ASSERT_FLOAT_WITHIN(10.0f, 69964.0f, bar * 100000.0f);
}

void test_bmp085_sample_cycle_oss2_matches_datasheet() {
    FakeBus bus;
    std::vector<uint8_t> calBytes;
    const int16_t vals[11] = {408, -72, -14383, (int16_t)32741, (int16_t)32757, (int16_t)23153,
                              6190, 4, -32768, -8711, 2868};
    for (int16_t v : vals)
        calBytes.insert(calBytes.end(),
                        {static_cast<uint8_t>(static_cast<uint16_t>(v) >> 8),
                         static_cast<uint8_t>(static_cast<uint16_t>(v) & 0xFF)});
    bus.queue(0xAA, calBytes);

    Bmp085 sensor{bus};
    TEST_ASSERT_TRUE(sensor.begin(0));
    TEST_ASSERT_TRUE(sensor.healthy());

    bus.queue(0xF6, adc16(27898));      // UT
    bus.queue(0xF6, {0x5D, 0x23, 0x00}); // UP register raw (23843 << 8 = 0x5D2300)

    sensor.start(100);                  // T first (B5 dependency)
    TEST_ASSERT_FALSE(sensor.poll(103));
    TEST_ASSERT_FALSE(sensor.poll(106));  // UT read + P conv kicked
    bool done = false;
    for (uint32_t t = 106; t < 200 && !done; ++t) done = sensor.poll(t);
    TEST_ASSERT_TRUE(done);

    TEST_ASSERT_FLOAT_WITHIN(0.05f, 15.0f, sensor.temperatureC());
    TEST_ASSERT_FLOAT_WITHIN(15.0f, 69964.0f, sensor.pressureBar() * 100000.0f);

    // command framing: ctrl writes 0x2E then 0x34|(OSS<<6)
    TEST_ASSERT_TRUE(bus.sawWrite(0x2E));
    TEST_ASSERT_TRUE(bus.sawWrite(0x34 | (Bmp085::OSS << 6)));
}

void test_bmp085_garbage_cal_rejected_and_backoff() {
    FakeBus bus;
    std::vector<uint8_t> zeros(22, 0x00);
    bus.queue(0xAA, zeros);
    Bmp085 sensor{bus};
    TEST_ASSERT_FALSE(sensor.begin(0));   // AC1==0: uncalibrated EPROM
    TEST_ASSERT_FALSE(sensor.healthy());

    bus.queue(0xAA, std::vector<uint8_t>(22, 0xFF));
    TEST_ASSERT_FALSE(sensor.begin(1));   // in backoff: bus untouched
    bool booted = false;
    for (uint32_t t = 2000; t < 6000 && !booted; t += 100) booted = sensor.begin(t);
    TEST_ASSERT_FALSE(booted);            // AC1==0xFFFF: still garbage
    TEST_ASSERT_FALSE(sensor.healthy());

    // real cal arrives -> boots
    std::vector<uint8_t> calBytes;
    const int16_t vals[11] = {408, -72, -14383, (int16_t)32741, (int16_t)32757, (int16_t)23153,
                              6190, 4, -32768, -8711, 2868};
    for (int16_t v : vals)
        calBytes.insert(calBytes.end(),
                        {static_cast<uint8_t>(static_cast<uint16_t>(v) >> 8),
                         static_cast<uint8_t>(static_cast<uint16_t>(v) & 0xFF)});
    bus.queue(0xAA, calBytes);
    for (uint32_t t = 6000; t < 12000; t += 100)
        if (sensor.begin(t)) { booted = true; break; }
    TEST_ASSERT_TRUE(booted);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_ms5837_boot_and_datasheet_vector);
    RUN_TEST(test_ms5837_crc_corruption_is_refused);
    RUN_TEST(test_ms5837_bus_fault_backoff_and_recovery);
    RUN_TEST(test_bmp085_math_datasheet_vector);
    RUN_TEST(test_bmp085_sample_cycle_oss2_matches_datasheet);
    RUN_TEST(test_bmp085_garbage_cal_rejected_and_backoff);
    return UNITY_END();
}
