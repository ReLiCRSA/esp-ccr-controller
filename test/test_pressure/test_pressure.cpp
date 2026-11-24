#include <unity.h>

#include <cmath>
#include <cstring>
#include <deque>
#include <map>
#include <utility>
#include <vector>

#include "sensors/depth_announce.h"
#include "sensors/depth_manager.h"
#include "sensors/lps28.h"
#include "sensors/ms5837.h"

using press::Announcer;
using press::Lps28;
using press::Ms5837;
using press::DepthArbiter;

void setUp() {}
void tearDown() {}

// ---------------------------------------------------------------------------
// Fake bus: canned per-register read queues + fault injection.
// A read with nothing queued NACKs — unprogrammed reads must fail loudly,
// exactly like an absent sensor. queueAt() programs a read for ONE address,
// which is how the LPS28 probe (0x5C vs 0x5D) gets tested honestly.
// ---------------------------------------------------------------------------
class FakeBus : public press::Transport {
public:
    void queue(uint8_t reg, std::vector<uint8_t> bytes) { q_[reg].push_back(std::move(bytes)); }
    void queueAt(uint8_t addr, uint8_t reg, std::vector<uint8_t> bytes) {
        qAt_[{addr, reg}].push_back(std::move(bytes));
    }
    // Sticky response: like PROM — same bytes for every read, never consumed.
    void queueSticky(uint8_t reg, std::vector<uint8_t> bytes) { sticky_[reg] = std::move(bytes); }
    int failReads = 0, failWrites = 0, reads = 0;
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
    bool writeThenRead(uint8_t addr, uint8_t reg, uint8_t* out, uint8_t n) override {
        reads++;
        if (failReads > 0) { failReads--; return false; }
        auto at = qAt_.find({addr, reg});
        if (at != qAt_.end()) {
            if (at->second.empty()) return false;
            const std::vector<uint8_t> resp = at->second.front();
            at->second.pop_front();
            if (resp.size() != n) return false;
            std::memcpy(out, resp.data(), n);
            return true;
        }
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
    bool sawWritePair(uint8_t reg, uint8_t data) const {
        for (size_t i = 0; i + 1 < writes.size(); ++i)
            if (writes[i] == reg && writes[i + 1] == data) return true;
        return false;
    }

private:
    std::map<uint8_t, std::deque<std::vector<uint8_t>>> q_;
    std::map<std::pair<uint8_t, uint8_t>, std::deque<std::vector<uint8_t>>> qAt_;
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
// LPS28DFW — boot: probe 0x5C then 0x5D by WHO_AM_I (0xB4), soft-boot
// (BOOT bit + 50 ms self-test), then FS_MODE=1 (4060 hPa) and ODR/AVG.
// The BOOT resets all registers, so the order of writes is load-bearing.
// ===========================================================================
void test_lps28_boot_probe_config_sequence() {
    FakeBus bus;
    // A breakout with XSA pulled high: only 0x5D answers.
    bus.queueAt(0x5D, 0x0F, {0xB4});
    Lps28 s{bus};

    TEST_ASSERT_FALSE(s.begin(0));   // probe: 0x5C silent, 0x5D says 0xB4, BOOT issued
    TEST_ASSERT_TRUE(bus.sawWritePair(0x11, 0x80));   // CTRL_REG2 = BOOT
    TEST_ASSERT_FALSE(s.begin(20));  // self-test still running
    TEST_ASSERT_FALSE(s.healthy());
    TEST_ASSERT_TRUE(s.begin(60));   // self-test done -> configure
    TEST_ASSERT_TRUE(s.healthy());
    TEST_ASSERT_EQUAL_UINT8(0x5D, s.addr());

    // Order after BOOT matters: FS_MODE (4060 hPa) then run at ODR 25 Hz.
    TEST_ASSERT_TRUE(bus.sawWritePair(0x11, 0x40));
    TEST_ASSERT_TRUE(bus.sawWritePair(0x10, 0x20));
    TEST_ASSERT_TRUE(s.begin(61));   // idempotent once READY
}

void test_lps28_wrong_identity_is_refused() {
    // A near-relative on the bus (LPS22HB's 0xB1) must NOT be accepted as
    // our depth sensor: same bus, similar registers, different calibration.
    FakeBus bus;
    bus.queueSticky(0x0F, {0xB1});
    Lps28 s{bus};
    bool booted = false;
    for (uint32_t t = 0; t < 20000 && !booted; t += 100) booted = s.begin(t);
    TEST_ASSERT_FALSE(booted);
    TEST_ASSERT_FALSE(s.healthy());
    // Nothing was ever commanded to a chip we did not identify:
    TEST_ASSERT_FALSE(bus.sawWritePair(0x11, 0x80));
}

void test_lps28_sample_cycle_and_scaling() {
    FakeBus bus;
    bus.queueSticky(0x0F, {0xB4});
    // 1013.25 hPa at 1/2048 LSB = 2075136 = 0x1FAA00; burst from 0x28 is
    // XL,L,H then TEMP L,H (auto-increment). 21.34 C = 2134 = 0x0856.
    bus.queueSticky(0x28, {0x00, 0xAA, 0x1F, 0x56, 0x08});
    Lps28 s{bus};
    TEST_ASSERT_FALSE(s.begin(0));   // probe + BOOT
    TEST_ASSERT_TRUE(s.begin(60));   // self-test over -> configured

    s.start(1000);
    TEST_ASSERT_FALSE(s.poll(1050));          // window not elapsed
    TEST_ASSERT_TRUE(s.poll(1100));           // one read, one answer
    TEST_ASSERT_FLOAT_WITHIN(0.0005f, 1.01325f, s.pressureBar());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 21.34f, s.temperatureC());
    TEST_ASSERT_EQUAL_UINT32(1100, s.lastGoodMs());
    TEST_ASSERT_FALSE(s.poll(1200));          // must re-arm via start()

    // FS_MODE=1 scaling directly: full-scale 4060 hPa fits 24-bit signed.
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 4.06f,
                             Lps28::pressureBarFromRaw(4060 * 2048));
}

void test_lps28_bus_fault_backoff_and_recovery() {
    FakeBus bus;
    bus.queueSticky(0x0F, {0xB4});
    bus.queueSticky(0x28, {0x00, 0xAA, 0x1F, 0x56, 0x08});
    Lps28 s{bus};
    TEST_ASSERT_FALSE(s.begin(0));   // probe + BOOT issued, not READY yet
    TEST_ASSERT_TRUE(s.begin(60));   // self-test over, configured
    TEST_ASSERT_FALSE(s.poll(0));
    s.start(100);

    bus.failReads = 5;                 // bus dies mid-run
    const int readsAtFault = bus.reads;
    for (uint32_t t = 200; t < 60000 && s.healthy(); t += 100) {
        s.start(t);
        for (uint32_t p = t; p < t + 100; ++p)
            if (s.poll(p)) break;
    }
    TEST_ASSERT_FALSE(s.healthy());
    // Backoff must be real: ~5 failing reads, not one read per tick for a
    // minute (that would be 600 reads and a bus held hostage).
    TEST_ASSERT_TRUE(bus.reads - readsAtFault <= 8);

    // bus recovers: begin re-probes and reboots, samples flow again
    bool done = false;
    for (uint32_t t = 60000; t < 90000 && !done; t += 50) {
        s.begin(t);
        s.start(t);
        for (uint32_t p = t; p < t + 200 && !done; ++p) done = s.poll(p);
    }
    TEST_ASSERT_TRUE(done);
    TEST_ASSERT_TRUE(s.healthy());
    TEST_ASSERT_FLOAT_WITHIN(0.0005f, 1.01325f, s.pressureBar());
}

// ===========================================================================
// DepthArbiter — boot-time arbitration between the dive-rated MS5837 and the
// bench LPS28DFW. The identity gates (PROM CRC / WHO_AM_I) belong to the
// drivers and are proven above; what is proven here is the POLICY:
// prefer, fall back, never claim an unidentified source, hot-plug support,
// and never change the locked source mid-run.
// ===========================================================================
static void seedMs5837(FakeBus& bus) {
    const uint16_t c1[6] = {34982, 36352, 20328, 22354, 26646, 26146};
    const uint16_t prom[8] = {findValidCrcWord0(c1), c1[0], c1[1], c1[2],
                              c1[3], c1[4], c1[5], 0};
    loadProm(bus, prom);
}

static bool lockArbiter(DepthArbiter& arb, uint32_t from, uint32_t to, uint32_t& at) {
    for (uint32_t t = from; t < to; t += 10)
        if (arb.begin(t)) { at = t; return true; }
    return false;
}

void test_arbiter_prefers_ms5837_when_both_fitted() {
    FakeBus bus;
    seedMs5837(bus);
    bus.queueSticky(0x0F, {0xB4});   // an LPS28 is fitted too
    Ms5837 ms{bus};
    Lps28 lp{bus};
    DepthArbiter arb{ms, lp};
    uint32_t at = 0;
    TEST_ASSERT_TRUE(lockArbiter(arb, 0, 3000, at));
    TEST_ASSERT_TRUE(arb.source() == &ms);   // dive-rated part wins
    TEST_ASSERT_TRUE(arb.healthy());
    TEST_ASSERT_EQUAL_STRING("MS5837-30BA @0x76", arb.sourceName());
}

void test_arbiter_falls_back_to_lps28() {
    FakeBus bus;
    bus.queueAt(0x5C, 0x0F, {0xB4});   // only an LPS28 answers
    bus.queueSticky(0x28, {0x00, 0xAA, 0x1F, 0x56, 0x08});
    Ms5837 ms{bus};
    Lps28 lp{bus};
    DepthArbiter arb{ms, lp};
    uint32_t at = 0;
    TEST_ASSERT_TRUE(lockArbiter(arb, 0, 10000, at));
    TEST_ASSERT_TRUE(arb.source() == &lp);
    // The MS5837 window had to expire first — no silent instant fallback.
    TEST_ASSERT_TRUE(at >= DepthArbiter::PREFERRED_WINDOW_MS);
    TEST_ASSERT_EQUAL_STRING("LPS28DFW @0x5C", arb.sourceName());

    bool sampled = false;
    for (uint32_t t = at; t < at + 2000 && !sampled; t += 10) {
        arb.start(t);
        for (uint32_t p = t; p < t + 20 && !sampled; ++p) sampled = arb.poll(p);
    }
    TEST_ASSERT_TRUE(sampled);
    TEST_ASSERT_FLOAT_WITHIN(0.0005f, 1.01325f, arb.pressureBar());
}

void test_arbiter_refuses_unidentified_sources() {
    // LPS22 lookalike (wrong WHO_AM_I) + MS5837 with corrupted PROM:
    // both drivers refuse themselves, so the arbiter must stay unlocked
    // forever — depth with no identity is not depth.
    FakeBus bus;
    bus.queueSticky(0x0F, {0xB1});
    const uint16_t c1[6] = {34982, 36352, 20328, 22354, 26646, 26146};
    uint16_t bad[8] = {findValidCrcWord0(c1), c1[0], c1[1],
                       static_cast<uint16_t>(c1[2] + 5), c1[3], c1[4], c1[5], 0};
    TEST_ASSERT_FALSE(Ms5837::checkPromCrc(bad));
    loadProm(bus, bad);
    Ms5837 ms{bus};
    Lps28 lp{bus};
    DepthArbiter arb{ms, lp};
    for (uint32_t t = 0; t < 30000; t += 10) {
        TEST_ASSERT_FALSE(arb.begin(t));
        TEST_ASSERT_FALSE(arb.locked());
        TEST_ASSERT_NULL(arb.source());
        TEST_ASSERT_FALSE(arb.healthy());
    }
}

void test_arbiter_hotplug_then_boot_lock() {
    FakeBus bus;
    Ms5837 ms{bus};
    Lps28 lp{bus};
    DepthArbiter arb{ms, lp};

    // Nothing fitted: keeps cycling windows, never claims a source.
    for (uint32_t t = 0; t < 8000; t += 10) {
        arb.begin(t);
        TEST_ASSERT_FALSE(arb.locked());
    }

    // LPS28 fitted mid-run: must come up within one recirculation cycle.
    bus.queueAt(0x5C, 0x0F, {0xB4});
    bus.queueSticky(0x28, {0x00, 0xAA, 0x1F, 0x56, 0x08});
    uint32_t at = 0;
    TEST_ASSERT_TRUE(lockArbiter(arb, 8000, 20000, at));
    TEST_ASSERT_TRUE(arb.source() == &lp);

    // Now the MS5837 "arrives" too. Identity is fixed at boot: the locked
    // source NEVER changes, no matter what shows up later.
    seedMs5837(bus);
    for (uint32_t t = at; t < at + 8000; t += 10) {
        TEST_ASSERT_TRUE(arb.begin(t));          // LPS28 stays READY
        TEST_ASSERT_TRUE(arb.source() == &lp);
    }
}

void test_arbiter_never_switches_midrun() {
    // Both fitted, MS5837 locked, then the depth bus dies. The arbiter must
    // fail HONESTLY (healthy false -> sensor-stale alarm) but must not
    // hand depth to the LPS28 sitting on the same bus: mid-dive sensor
    // swapping is a lie, not a recovery.
    FakeBus bus;
    seedMs5837(bus);
    bus.queueSticky(0x0F, {0xB4});
    bus.queueSticky(0x28, {0x00, 0xAA, 0x1F, 0x56, 0x08});
    Ms5837 ms{bus};
    Lps28 lp{bus};
    DepthArbiter arb{ms, lp};
    uint32_t at = 0;
    TEST_ASSERT_TRUE(lockArbiter(arb, 0, 3000, at));
    TEST_ASSERT_TRUE(arb.source() == &ms);

    bus.failReads = 30;                    // depth bus dies
    bool sawUnhealthy = false;
    for (uint32_t t = at; t < 20000; t += 10) {
        arb.begin(t);
        arb.start(t);                      // keep sampling: the fault must
        for (uint32_t p = t; p < t + 10; ++p) arb.poll(p);   // actually be met
        if (!arb.healthy()) sawUnhealthy = true;
        TEST_ASSERT_TRUE(arb.source() == &ms);               // same source, always
    }
    TEST_ASSERT_TRUE(sawUnhealthy);   // the fault was reported honestly...

    // ...the bus then recovers, and re-init re-probes the SAME chip (its
    // identity gate again) — never the slot. A healthy MS5837 is on the
    // bus, so health returns to the source that was locked at boot.
    bus.failReads = 0;
    bool healed = false;
    for (uint32_t t = 20000; t < 60000 && !healed; t += 10) {
        arb.begin(t);
        TEST_ASSERT_TRUE(arb.source() == &ms);
        healed = arb.healthy();
    }
    TEST_ASSERT_TRUE(healed);
    TEST_ASSERT_EQUAL_STRING("MS5837-30BA @0x76", arb.sourceName());
}

// ---------------------------------------------------------------------------
// The boot announce. Its whole job is to not print a number it has not
// measured: the drivers report READY at IDENTIFICATION and the first pressure
// conversion comes later, so an announce wired to the lock printed
// `ready, 0.0 hPa` in the line the bench reads as evidence (BENCH.md C3-a).
// These cases pin the ordering, the exact bytes, and the fact that a source
// which identifies but never reads says so instead of leaving silence.
// ---------------------------------------------------------------------------

void test_announce_prints_nothing_until_a_source_locks() {
    Announcer a;
    char line[160];
    // An absent sensor: nothing ever locks, so the boot stays silent. That is
    // the contract a sensor-less bench run depends on - the honest signal for
    // "no sensor fitted" is the absence of this line, and it must stay that way.
    for (uint32_t t = 1000; t < 10000; t += 100)
        TEST_ASSERT_TRUE(a.update(t, false, false, 0.0f) == Announcer::Action::NONE);
    TEST_ASSERT_EQUAL(0, a.line(Announcer::Action::NONE, "LPS28DFW @0x5D",
                                line, sizeof(line)));
}

void test_announce_waits_for_a_reading_not_for_a_lock() {
    Announcer a;
    char line[160];

    // Locked at 1.6 s (the arbiter's way), no conversion yet: NO line, because
    // the only pressure available would be the driver's initial zero.
    TEST_ASSERT_TRUE(a.update(1600, true, false, 0.0f) == Announcer::Action::NONE);
    TEST_ASSERT_TRUE(a.update(1700, true, false, 0.0f) == Announcer::Action::NONE);
    TEST_ASSERT_TRUE(a.update(2000, true, false, 0.0f) == Announcer::Action::NONE);

    // First sane sample wins: the announce is about this reading, whenever it
    // arrives, and it prints once.
    TEST_ASSERT_TRUE(a.update(2100, true, true, 1.0132f) == Announcer::Action::READY);
    // The returned length is what gets written to the console, so it is asserted
    // equal to the bytes too: a short return silently drops the tail of the line.
    const char* ready = "depth: LPS28DFW @0x5D ready, 1013.2 hPa\n";
    TEST_ASSERT_EQUAL(std::strlen(ready),
                      a.line(Announcer::Action::READY, "LPS28DFW @0x5D", line,
                             sizeof(line)));
    TEST_ASSERT_EQUAL_STRING(ready, line);

    TEST_ASSERT_TRUE(a.update(2200, true, true, 1.0133f) == Announcer::Action::NONE);
    TEST_ASSERT_TRUE(a.update(9000, true, true, 1.0130f) == Announcer::Action::NONE);
}

void test_announce_says_so_when_a_locked_source_never_reads() {
    Announcer a;
    char line[160];

    TEST_ASSERT_TRUE(a.update(1600, true, false, 0.0f) == Announcer::Action::NONE);
    // The grace is measured from the lock, not from boot.
    TEST_ASSERT_TRUE(a.update(1600 + Announcer::NO_DATA_GRACE_MS - 1, true, false,
                              0.0f) == Announcer::Action::NONE);
    TEST_ASSERT_TRUE(a.update(1600 + Announcer::NO_DATA_GRACE_MS, true, false,
                              0.0f) == Announcer::Action::NO_DATA);
    const char* warn =
        "depth: ** LPS28DFW @0x5D identified, no sane pressure within 3 s "
        "(reads 0.0 hPa) - depth will not be used\n";
    TEST_ASSERT_EQUAL(std::strlen(warn),
                      a.line(Announcer::Action::NO_DATA, "LPS28DFW @0x5D", line,
                             sizeof(line)));
    TEST_ASSERT_EQUAL_STRING(warn, line);
    // Once per boot.
    TEST_ASSERT_TRUE(a.update(6000, true, false, 0.0f) == Announcer::Action::NONE);

    // A reading that turns up late still gets the truth on the record, because
    // the last thing the boot says about depth is what the machine will use.
    TEST_ASSERT_TRUE(a.update(7000, true, true, 1.0121f) == Announcer::Action::READY);
    a.line(Announcer::Action::READY, "LPS28DFW @0x5D", line, sizeof(line));
    TEST_ASSERT_EQUAL_STRING("depth: LPS28DFW @0x5D ready, 1012.1 hPa\n", line);
}

void test_announce_shows_the_reading_that_was_refused() {
    // A chip answering with a plausible-but-wrong conversion is the case the
    // line exists to catch, and the number in it is the evidence: printing
    // `0.0` for a chip streaming garbage would send the bench looking in the
    // wrong place.
    Announcer a;
    char line[160];
    TEST_ASSERT_TRUE(a.update(1000, true, false, 0.0f) == Announcer::Action::NONE);
    TEST_ASSERT_TRUE(a.update(1100, true, false, 0.031f) == Announcer::Action::NONE);
    TEST_ASSERT_TRUE(a.update(1200, true, false, 0.031f) == Announcer::Action::NONE);
    TEST_ASSERT_TRUE(a.update(1000 + Announcer::NO_DATA_GRACE_MS, true, false,
                              0.031f) == Announcer::Action::NO_DATA);
    a.line(Announcer::Action::NO_DATA, "MS5837-30BA @0x76", line, sizeof(line));
    TEST_ASSERT_EQUAL_STRING(
        "depth: ** MS5837-30BA @0x76 identified, no sane pressure within 3 s "
        "(reads 31.0 hPa) - depth will not be used\n", line);
}

void test_announce_refuses_a_truncated_line() {
    Announcer a;
    char line[160];
    TEST_ASSERT_TRUE(a.update(2000, true, true, 1.0132f) == Announcer::Action::READY);
    // One byte short of the full line: refuse, do not emit half a claim.
    const char* full = "depth: LPS28DFW @0x5D ready, 1013.2 hPa\n";
    TEST_ASSERT_EQUAL(0, a.line(Announcer::Action::READY, "LPS28DFW @0x5D", line,
                                std::strlen(full)));
    TEST_ASSERT_EQUAL_STRING("", line);
    TEST_ASSERT(a.line(Announcer::Action::READY, "LPS28DFW @0x5D", line,
                       sizeof(line)) > 0);

    char tiny[8];
    TEST_ASSERT_EQUAL(0, a.line(Announcer::Action::NO_DATA, "LPS28DFW @0x5D",
                                tiny, sizeof(tiny)));
    TEST_ASSERT_EQUAL_STRING("", tiny);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_ms5837_boot_and_datasheet_vector);
    RUN_TEST(test_ms5837_crc_corruption_is_refused);
    RUN_TEST(test_ms5837_bus_fault_backoff_and_recovery);
    RUN_TEST(test_lps28_boot_probe_config_sequence);
    RUN_TEST(test_lps28_wrong_identity_is_refused);
    RUN_TEST(test_lps28_sample_cycle_and_scaling);
    RUN_TEST(test_lps28_bus_fault_backoff_and_recovery);
    RUN_TEST(test_arbiter_prefers_ms5837_when_both_fitted);
    RUN_TEST(test_arbiter_falls_back_to_lps28);
    RUN_TEST(test_arbiter_refuses_unidentified_sources);
    RUN_TEST(test_arbiter_hotplug_then_boot_lock);
    RUN_TEST(test_arbiter_never_switches_midrun);
    RUN_TEST(test_announce_prints_nothing_until_a_source_locks);
    RUN_TEST(test_announce_waits_for_a_reading_not_for_a_lock);
    RUN_TEST(test_announce_says_so_when_a_locked_source_never_reads);
    RUN_TEST(test_announce_shows_the_reading_that_was_refused);
    RUN_TEST(test_announce_refuses_a_truncated_line);
    return UNITY_END();
}
