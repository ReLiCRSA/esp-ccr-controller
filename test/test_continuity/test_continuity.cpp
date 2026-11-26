// ============================================================================
// test_continuity — the record that carries a diver's tissues across a power
// cycle (review H1: "the one gap that can kill without any instrument
// failing").
//
// Every startDive() used to reset the deco engine, so a diver who surfaced with
// deco debt and dived again an hour later was given a ceiling, an NDL and a
// schedule computed as if the first dive had never happened. That is not a
// display bug: it is a decompression sickness risk with no alarm attached to it.
// This file grades the parts that decision is built from — the byte format, the
// two-slot store, the surface-interval rule and the sentences the machine says
// about all of it — and it grades them HERE because the code that calls them
// (main.cpp) is not compiled for the native env at all. A rule about what the
// device stores or says, written in a file no test can read, is a rule nobody
// checks; that is the same reasoning that moved the dive-file row arithmetic into
// rowfill.h and the surface records into divelog.cpp.
//
// The failure shapes these cases are written against:
//   * a torn write that decodes into plausible garbage (a CRC-less record, or a
//     float image read back as a NaN) — the answer must be "no record", never a
//     tissue pressure;
//   * a store that keeps the wrong record (a slot policy that overwrites the
//     newest picture with an older one, or a sequence that re-wraps and reorders
//     a diver's dives);
//   * an interval that is credited when it cannot be established (no clock, or a
//     record dated after the clock) — every one of those buys a diver a shallower
//     ceiling than the truth;
//   * a dive that starts from clean tissues because the record was lost, and
//     nobody is told.
// ============================================================================
#include <unity.h>

#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "log/contgate.h"
#include "log/continuity.h"

using namespace logfmt;

void setUp() {}
void tearDown() {}

namespace {

/// A record with recognisable, in-range numbers: the shape a real dive-end
/// snapshot has (tissues loaded above resting, a bit of oxygen clock on it).
ContinuityRecord sample(uint32_t seq, uint32_t epoch, uint8_t kind) {
    ContinuityRecord r{};
    r.kind = kind;
    r.ncomp = CONT_NCOMP;
    r.seq = seq;
    r.epoch = epoch;
    for (int i = 0; i < CONT_NCOMP; ++i) {
        // Monotonic across the compartments, like a real dive: the fast tissues
        // are closer to the inspired pressure than the slow ones.
        r.tissueX1e4[i] = static_cast<int32_t>((4.0 + 0.25 * i) * CONT_TISSUE_SCALE);
    }
    r.cnsX10 = 137;   // 13.7 %
    return r;
}

}  // namespace

// ---- the byte format -------------------------------------------------------

// The layout is the contract a host tool reads off a board forever, so the
// offsets are asserted as numbers, not as an opaque round-trip. The dive-file
// codec once wrote 34 B fields into a 32 B record and every record after it was
// corrupt; a stride or field-order change here does the same thing to a diver's
// deco debt.
void test_record_layout_is_the_stated_size_and_stride() {
    TEST_ASSERT_EQUAL_UINT(86u, static_cast<unsigned>(CONT_RECORD_SIZE));
    TEST_ASSERT_EQUAL_UINT(192u, static_cast<unsigned>(CONT_SLOT_STRIDE));
    TEST_ASSERT_EQUAL_INT(2, CONT_SLOTS);
    // Payload arithmetic: 4 magic + version/kind/flags/ncomp + seq(3)+reserved +
    // epoch(4) + 16 x 4 tissue + cns(2) + reserved(2) + crc(2).
    TEST_ASSERT_EQUAL_UINT(4u + 4u + 4u + 4u + 16u * 4u + 2u + 2u + 2u,
                       static_cast<unsigned>(CONT_RECORD_SIZE));
    // The stride is wider than the record on purpose: a v2 that grows the payload
    // (+16 helium loadings = +64 B) still fits inside one slot, so the file layout
    // does not have to change under a diver's feet.
    TEST_ASSERT_TRUE(CONT_SLOT_STRIDE >= CONT_RECORD_SIZE + 64u);
}

void test_codec_round_trips_every_field() {
    const ContinuityRecord in = sample(7u, 1800000600u, CONT_KIND_DIVE_END);
    uint8_t bytes[CONT_RECORD_SIZE];
    TEST_ASSERT_EQUAL_UINT(static_cast<unsigned>(CONT_RECORD_SIZE),
                           static_cast<unsigned>(packContinuity(in, bytes)));

    ContinuityRecord out{};
    TEST_ASSERT_TRUE_MESSAGE(unpackContinuity(bytes, out), "a clean record must decode");
    TEST_ASSERT_EQUAL_UINT8(in.kind, out.kind);
    TEST_ASSERT_EQUAL_UINT8(CONT_NCOMP, out.ncomp);
    TEST_ASSERT_EQUAL_UINT32(in.seq, out.seq);
    TEST_ASSERT_EQUAL_UINT32(in.epoch, out.epoch);
    TEST_ASSERT_EQUAL_UINT16(in.cnsX10, out.cnsX10);
    for (int i = 0; i < CONT_NCOMP; ++i) TEST_ASSERT_EQUAL_INT32(in.tissueX1e4[i], out.tissueX1e4[i]);
}

// Every byte of the record is either a field or a CRC over the fields: nothing
// may be written and never read back.
void test_no_field_is_dropped_between_pack_and_unpack() {
    ContinuityRecord in = sample(1u, 1u, CONT_KIND_PERIODIC);
    in.flags = CONT_FLAG_CLOCK_CARRIED;
    uint8_t bytes[CONT_RECORD_SIZE];
    packContinuity(in, bytes);
    ContinuityRecord out{};
    TEST_ASSERT_TRUE(unpackContinuity(bytes, out));
    TEST_ASSERT_EQUAL_UINT8(CONT_FLAG_CLOCK_CARRIED, out.flags);
}

// The CRC is per record, over the record. Flipping ANY bit in it — a tissue
// column, the CNS column, the kind, the epoch, or the CRC itself — must make the
// record refuse to decode. A torn write whose CRC happens to survive is the only
// corruption this format has to catch, and it catches it at the byte level so
// that nothing downstream ever has to guess.
void test_any_bit_flip_anywhere_in_the_record_refuses_to_decode() {
    const ContinuityRecord in = sample(12u, 1800000000u, CONT_KIND_DIVE_END);
    uint8_t clean[CONT_RECORD_SIZE];
    packContinuity(in, clean);

    size_t caught = 0;
    for (size_t byte = 0; byte < CONT_RECORD_SIZE; ++byte) {
        for (int bit = 0; bit < 8; ++bit) {
            uint8_t bad[CONT_RECORD_SIZE];
            std::memcpy(bad, clean, sizeof(bad));
            bad[byte] ^= static_cast<uint8_t>(1u << bit);
            ContinuityRecord out{};
            if (!unpackContinuity(bad, out)) ++caught;
        }
    }
    TEST_ASSERT_EQUAL_UINT_MESSAGE(CONT_RECORD_SIZE * 8u, caught,
                                   "a single-bit flip somewhere must always fail the CRC");
}

// Erased flash is all zeros, and a neighbouring file's bytes are not a record.
// Both must answer "no record" rather than "a record with zero tissues".
void test_erasures_and_foreign_bytes_are_no_record() {
    uint8_t zeros[CONT_RECORD_SIZE] = {};
    ContinuityRecord out{};
    TEST_ASSERT_FALSE(unpackContinuity(zeros, out));

    uint8_t junk[CONT_RECORD_SIZE];
    for (size_t i = 0; i < sizeof(junk); ++i) junk[i] = static_cast<uint8_t>(i * 7u + 3u);
    TEST_ASSERT_FALSE(unpackContinuity(junk, out));

    // A record whose payload is intact but whose header belongs to another build:
    // unknown version, or another engine's compartment count. Both are "I do not
    // know this format", and guessing at either is how a plausible-but-wrong
    // ceiling gets made.
    const ContinuityRecord in = sample(3u, 1800000000u, CONT_KIND_DIVE_END);
    uint8_t bytes[CONT_RECORD_SIZE];
    packContinuity(in, bytes);
    bytes[4] = CONT_VERSION + 1;                       // version
    TEST_ASSERT_FALSE(unpackContinuity(bytes, out));
    packContinuity(in, bytes);
    bytes[7] = CONT_NCOMP + 16;                        // a helium engine's payload
    TEST_ASSERT_FALSE(unpackContinuity(bytes, out));
}

// A CRC pass is not the end of the argument. A record written by a build with
// different units, or a slot that tore in a way the CRC happened to like, must
// still not reach a diver as a tissue pressure: the range gate is IN the codec.
void test_numbers_no_dive_could_produce_are_refused_even_with_a_good_crc() {
    const size_t kTissueOff = 16, kCnsOff = 80;

    struct Case { const char* what; size_t off; int32_t value; };
    const Case cases[] = {
        {"a tissue of 0 bar (no diver is at vacuum)", kTissueOff, 0},
        {"a negative tissue", kTissueOff, -1},
        {"a tissue of 100.0001 bar (above any measurable depth)", kTissueOff,
         (CONT_TISSUE_MAX_BAR * CONT_TISSUE_SCALE) + 1},
        {"a tissue of 4 000 000 bar (a float's bit pattern read as an int)", kTissueOff,
         4000000},
        {"CNS above the engine's own 999 % cap", kCnsOff, 10000},
    };
    for (const Case& c : cases) {
        ContinuityRecord r = sample(5u, 1800000000u, CONT_KIND_DIVE_END);
        if (c.off == kCnsOff) r.cnsX10 = static_cast<uint16_t>(c.value);
        else r.tissueX1e4[3] = c.value;
        uint8_t bytes[CONT_RECORD_SIZE];
        packContinuity(r, bytes);      // packs what it is handed, CRC and all
        ContinuityRecord out{};
        TEST_ASSERT_FALSE_MESSAGE(unpackContinuity(bytes, out), c.what);
    }
}

// The scale is what makes a torn float impossible: 0.1 mbar counts on a tissue
// column is about a centimetre of water of ceiling, and the stored value is a
// number a host tool can read forever without knowing IEEE-754.
void test_scaled_integers_survive_the_round_trip_to_one_part_in_10000() {
    deco::Zhl16Ccr::TissueState st{};
    for (int i = 0; i < CONT_NCOMP; ++i) st.pt[i] = 0.79 + 2.345678 * (i + 1);
    st.cnsPct = 42.7;

    ContinuityRecord r{};
    TEST_ASSERT_TRUE(fromTissueState(st, CONT_KIND_DIVE_END, r));
    deco::Zhl16Ccr::TissueState back{};
    TEST_ASSERT_TRUE(toTissueState(r, back));
    for (int i = 0; i < CONT_NCOMP; ++i)
        TEST_ASSERT_FLOAT_WITHIN(0.0002, st.pt[i], back.pt[i]);   // 2x the quantum
    TEST_ASSERT_FLOAT_WITHIN(0.15, st.cnsPct, back.cnsPct);       // CNS is stored in tenths
}

// The engine's numbers going OUT of range is a refusal, not a clamp: a clamped
// record would seed the next dive from a loading this engine never had.
void test_out_of_range_engine_state_writes_no_record() {
    deco::Zhl16Ccr::TissueState st{};
    for (int i = 0; i < CONT_NCOMP; ++i) st.pt[i] = 4.0;
    ContinuityRecord r{};

    st.pt[9] = 0.0;                                   // vacuum
    TEST_ASSERT_FALSE(fromTissueState(st, CONT_KIND_DIVE_END, r));
    st.pt[9] = 100.5;                                 // beyond kSeedMaxBar
    TEST_ASSERT_FALSE(fromTissueState(st, CONT_KIND_DIVE_END, r));
    st.pt[9] = std::nan("");
    TEST_ASSERT_FALSE(fromTissueState(st, CONT_KIND_DIVE_END, r));
    st.pt[9] = 4.0;
    st.cnsPct = -1.0;
    TEST_ASSERT_FALSE(fromTissueState(st, CONT_KIND_DIVE_END, r));
    st.cnsPct = 1000.0;                               // above the engine's own cap
    TEST_ASSERT_FALSE(fromTissueState(st, CONT_KIND_DIVE_END, r));
    st.cnsPct = 42.0;
    TEST_ASSERT_TRUE(fromTissueState(st, CONT_KIND_DIVE_END, r));
}

// ---- the two-slot store ----------------------------------------------------

// The slots are separate records at a stride, and nothing about one write may
// reach into its neighbour. This is the bug class that made the dive-file codec
// corrupt its neighbours for a whole release: fields that outgrew the slot.
void test_a_slot_write_leaves_the_other_slot_and_the_gaps_untouched() {
    ContinuitySlots s;
    s.clear();
    const uint8_t* img = s.bytes();
    for (size_t i = 0; i < ContinuitySlots::kImageSize; ++i) TEST_ASSERT_EQUAL_UINT8(0u, img[i]);

    ContinuityRecord a = sample(1u, 1800000000u, CONT_KIND_DIVE_END);
    a.seq = 1u;
    TEST_ASSERT_TRUE(s.write(0, a));
    // Slot 1 and both inter-record gaps are still zeros: a record is 86 B, its
    // slot is 192 B, and the space between is padding, not somebody else's field.
    for (size_t i = CONT_RECORD_SIZE; i < ContinuitySlots::kImageSize; ++i)
        TEST_ASSERT_EQUAL_UINT8(0u, img[i]);
    ContinuityRecord b = sample(2u, 1800000600u, CONT_KIND_DIVE_END);
    b.seq = 2u;
    TEST_ASSERT_TRUE(s.write(1, b));
    TEST_ASSERT_TRUE(s.decode(0, a));                  // the first one is intact
    TEST_ASSERT_EQUAL_UINT32(1u, a.seq);
    TEST_ASSERT_TRUE(s.decode(1, b));
    TEST_ASSERT_EQUAL_UINT32(2u, b.seq);
    // Out-of-range slot numbers do not write anywhere.
    TEST_ASSERT_FALSE(s.write(-1, a));
    TEST_ASSERT_FALSE(s.write(ContinuitySlots::kSlots, a));
    TEST_ASSERT_NULL(s.rawSlotBytes(-1));
    TEST_ASSERT_NULL(s.rawSlotBytes(ContinuitySlots::kSlots));
}

// The store's whole safety argument: a reset during a write costs the newest
// SNAPSHOT, never the PICTURE, because the write always lands in the OLDER slot.
void test_writes_always_replace_the_older_slot() {
    ContinuitySlots s;
    s.clear();
    TEST_ASSERT_EQUAL_INT(-1, s.newest());
    TEST_ASSERT_EQUAL_UINT32(1u, s.nextSeq());

    // A fresh board fills slot 0, then slot 1, before it overwrites anything.
    TEST_ASSERT_EQUAL_INT(0, s.freeSlot());
    ContinuityRecord r = sample(1u, 1800000000u, CONT_KIND_DIVE_END);
    r.seq = s.nextSeq();
    TEST_ASSERT_TRUE(s.write(s.freeSlot(), r));
    TEST_ASSERT_EQUAL_INT(1, s.freeSlot());
    r.seq = s.nextSeq();
    TEST_ASSERT_TRUE(s.write(s.freeSlot(), r));

    // Both slots hold records now: the next write must take the OLDER one, so
    // the newest picture survives a reset in the middle of it.
    TEST_ASSERT_EQUAL_INT(0, s.freeSlot());
    ContinuityRecord oldest{};
    TEST_ASSERT_TRUE(s.decode(0, oldest));
    TEST_ASSERT_EQUAL_UINT32(1u, oldest.seq);
    r.seq = s.nextSeq();
    TEST_ASSERT_EQUAL_UINT32(3u, r.seq);
    TEST_ASSERT_TRUE(s.write(s.freeSlot(), r));
    ContinuityRecord newest{};
    TEST_ASSERT_TRUE(s.readNewest(newest));
    TEST_ASSERT_EQUAL_UINT32(3u, newest.seq);          // the new one
    ContinuityRecord other{};
    TEST_ASSERT_TRUE(s.decode(1, other));
    TEST_ASSERT_EQUAL_UINT32(2u, other.seq);           // the one it did not touch
}

// A slot whose last write was torn is repaired before it is ever asked to be the
// fallback: the next write goes there, not to the slot that still decodes.
void test_a_torn_slot_is_written_before_the_valid_one() {
    ContinuitySlots s;
    ContinuityRecord a = sample(1u, 1800000000u, CONT_KIND_DIVE_END);
    a.seq = 1u;
    ContinuityRecord b = sample(2u, 1800000600u, CONT_KIND_DIVE_END);
    b.seq = 2u;
    TEST_ASSERT_TRUE(s.write(0, a));
    TEST_ASSERT_TRUE(s.write(1, b));
    uint8_t* slot0 = s.rawSlotBytes(0);
    TEST_ASSERT_NOT_NULL(slot0);
    slot0[40] ^= 0x80;                                 // the CRC says so
    TEST_ASSERT_FALSE(s.decode(0, a));
    TEST_ASSERT_EQUAL_INT(1, s.newest());
    TEST_ASSERT_EQUAL_INT(0, s.freeSlot());            // repair the torn one first
    TEST_ASSERT_EQUAL_UINT32(3u, s.nextSeq());         // the torn record still counted
}

// A file that is shorter than the layout (a first write interrupted by a reset)
// must read as "nothing there", never as a record with zero-filled tissues.
void test_a_short_image_decodes_as_no_record_not_as_garbage() {
    ContinuitySlots s;
    ContinuityRecord a = sample(1u, 1800000000u, CONT_KIND_DIVE_END);
    a.seq = 1u;
    uint8_t full[CONT_RECORD_SIZE];
    TEST_ASSERT_EQUAL_UINT(static_cast<unsigned>(CONT_RECORD_SIZE),
                           static_cast<unsigned>(packContinuity(a, full)));

    s.load(full, 0);                                   // an empty file
    TEST_ASSERT_EQUAL_INT(-1, s.newest());
    s.load(full, CONT_RECORD_SIZE / 2);                // the file ended mid-record
    TEST_ASSERT_EQUAL_INT(-1, s.newest());
    // Loading the whole thing back is the repair: the bytes were never the
    // problem, the length was.
    s.load(full, CONT_RECORD_SIZE);
    TEST_ASSERT_EQUAL_INT(0, s.newest());
    // A load() is also the only way an image is replaced: it clears first, so a
    // shorter image cannot leave a previous boot's record in the tail.
    ContinuityRecord b = sample(2u, 1800000600u, CONT_KIND_DIVE_END);
    b.seq = 2u;
    TEST_ASSERT_TRUE(s.write(1, b));
    s.load(full, CONT_RECORD_SIZE);
    TEST_ASSERT_EQUAL_INT(0, s.newest());              // slot 1 is zeros again
}

// The sequence numbers are the only ordering this store has: the board can have
// no wall clock, and a clock that moves backwards must not be able to reorder a
// diver's dives.
void test_ordering_is_by_sequence_and_the_counter_never_rewraps() {
    ContinuitySlots s;
    // An OLD epoch in the newest slot: sequence still wins.
    ContinuityRecord oldClock = sample(9u, 1800000000u, CONT_KIND_DIVE_END);
    oldClock.seq = 9u;
    ContinuityRecord newClock = sample(8u, 2000000000u, CONT_KIND_DIVE_END);
    newClock.seq = 8u;
    TEST_ASSERT_TRUE(s.write(0, newClock));
    TEST_ASSERT_TRUE(s.write(1, oldClock));
    TEST_ASSERT_EQUAL_INT(1, s.newest());
    ContinuityRecord win{};
    TEST_ASSERT_TRUE(s.readNewest(win));
    TEST_ASSERT_EQUAL_UINT32(9u, win.seq);

    // At the counter's ceiling it saturates instead of wrapping: a re-wrapped
    // counter would make the oldest surviving record look the newest.
    ContinuitySlots t;
    ContinuityRecord atMax = sample(SEQ_MAX, 1800000000u, CONT_KIND_DIVE_END);
    atMax.seq = SEQ_MAX;
    TEST_ASSERT_TRUE(t.write(0, atMax));
    TEST_ASSERT_EQUAL_UINT32(SEQ_MAX, t.nextSeq());
}

// ---- the surface interval --------------------------------------------------

// The reviewer's scenario, as arithmetic: a dive-end record, an hour of clock,
// one hour of off-gassing credit.
void test_a_dive_end_record_credits_the_clock_difference() {
    const SurfaceInterval iv = surfaceIntervalS(CONT_KIND_DIVE_END, 1800000000u, 1800003600u);
    TEST_ASSERT_TRUE_MESSAGE(iv.credit, "an hour after surfacing must be credited");
    TEST_ASSERT_EQUAL_UINT32(3600u, iv.seconds);
    TEST_ASSERT_EQUAL_UINT8(INT_CREDIT, iv.why);
}

// A MID-DIVE snapshot cannot say when the diver surfaced, so none of the time
// after it is provably surface time. Crediting it would credit time he may have
// spent still at depth — the one error that buys a shallower ceiling than the
// truth. This is the case the write cadence exists for (a stale loading), not the
// case the interval arithmetic exists for.
void test_a_mid_dive_snapshot_never_credits_an_interval() {
    const SurfaceInterval short1 = surfaceIntervalS(CONT_KIND_PERIODIC, 1800000000u, 1800000060u);
    TEST_ASSERT_FALSE(short1.credit);
    TEST_ASSERT_EQUAL_UINT32(0u, short1.seconds);
    TEST_ASSERT_EQUAL_UINT8(INT_MID_DIVE_RECORD, short1.why);

    // Even with a perfect clock, and even after a day: a loading taken at 40 m is
    // not a loading that has been off-gassing since then.
    const SurfaceInterval day = surfaceIntervalS(CONT_KIND_PERIODIC, 1800000000u, 1800086400u);
    TEST_ASSERT_FALSE(day.credit);
    TEST_ASSERT_EQUAL_UINT8(INT_MID_DIVE_RECORD, day.why);
}

// Three clock failures, and all three credit nothing. Each one is a different
// sentence on the console, because each one is a different thing for the diver or
// the bench to fix.
void test_an_interval_that_cannot_be_established_credits_nothing() {
    const SurfaceInterval noRecClock = surfaceIntervalS(CONT_KIND_DIVE_END, 0u, 1800003600u);
    TEST_ASSERT_FALSE(noRecClock.credit);
    TEST_ASSERT_EQUAL_UINT8(INT_NO_RECORD_CLOCK, noRecClock.why);

    const SurfaceInterval noNowClock = surfaceIntervalS(CONT_KIND_DIVE_END, 1800000000u, 0u);
    TEST_ASSERT_FALSE(noNowClock.credit);
    TEST_ASSERT_EQUAL_UINT8(INT_NO_NOW_CLOCK, noNowClock.why);

    // A record dated AFTER now: somebody set the clock backwards. There is no
    // interval to integrate, and a negative one is worse than zero.
    const SurfaceInterval ahead = surfaceIntervalS(CONT_KIND_DIVE_END, 1800003600u, 1800000000u);
    TEST_ASSERT_FALSE(ahead.credit);
    TEST_ASSERT_EQUAL_UINT32(0u, ahead.seconds);
    TEST_ASSERT_EQUAL_UINT8(INT_EPOCH_AHEAD, ahead.why);

    // Zero elapsed is not an error and is not credit either.
    const SurfaceInterval same = surfaceIntervalS(CONT_KIND_DIVE_END, 1800000000u, 1800000000u);
    TEST_ASSERT_FALSE(same.credit);
    TEST_ASSERT_EQUAL_UINT8(INT_CREDIT, same.why);
}

// The age the boot banner prints is a separate question from the credit: a
// record's age is answerable ("42 min ago") even when no interval is creditable,
// and it is -1 (never a fabricated 0) when even that has no honest answer.
void test_age_is_reported_separately_from_credit() {
    TEST_ASSERT_EQUAL_INT64(2520, continuityAgeS(1800000000u, 1800002520u));
    TEST_ASSERT_EQUAL_INT64(-1, continuityAgeS(0u, 1800002520u));
    TEST_ASSERT_EQUAL_INT64(-1, continuityAgeS(1800000000u, 0u));
    TEST_ASSERT_EQUAL_INT64(-1, continuityAgeS(1800002520u, 1800000000u));
}

// ---- honesty text ----------------------------------------------------------
// These sentences are the boot banner and the dive-start console line, and they
// are the only place a diver learns that his machine carried or lost his deco
// debt. Three shapes must never collapse into one silence: the store will not
// read, a record is there and does not decode, and there is simply no record.

void test_boot_banner_keeps_cannot_read_apart_from_nothing_there() {
    char line[256];

    continuityBootText(line, sizeof(line), false, false, nullptr, 1800000000u);
    TEST_ASSERT_NOT_NULL(std::strstr(line, "will not read"));

    continuityBootText(line, sizeof(line), true, true, nullptr, 1800000000u);
    TEST_ASSERT_NOT_NULL(std::strstr(line, "nothing in it decodes"));

    continuityBootText(line, sizeof(line), true, false, nullptr, 1800000000u);
    TEST_ASSERT_NOT_NULL(std::strstr(line, "no record yet"));

    // A carried record: the banner names it, its age, and whether the clock that
    // dated it was verified since the last reset.
    ContinuityRecord r = sample(4u, 1800000000u, CONT_KIND_DIVE_END);
    continuityBootText(line, sizeof(line), true, false, &r, 1800002520u);
    TEST_ASSERT_NOT_NULL(std::strstr(line, "record 4"));
    TEST_ASSERT_NOT_NULL(std::strstr(line, "2520 s (42 min)"));
    r.flags = CONT_FLAG_CLOCK_CARRIED;
    continuityBootText(line, sizeof(line), true, false, &r, 1800002520u);
    TEST_ASSERT_NOT_NULL(std::strstr(line, "clock carried, unverified"));
}

// The dive-start line states the seed source, its age, and the credit — and when
// there is no credit, the reason, in the module's own words.
void test_dive_start_line_states_the_seed_and_the_credit_or_its_absence() {
    char line[384];
    ContinuityRecord r = sample(11u, 1800000000u, CONT_KIND_DIVE_END);

    const SurfaceInterval credited = surfaceIntervalS(r.kind, r.epoch, 1800003600u);
    continuitySeedText(line, sizeof(line), &r, 1800003600u, credited);
    TEST_ASSERT_NOT_NULL(std::strstr(line, "SEEDED from record 11"));
    TEST_ASSERT_NOT_NULL(std::strstr(line, "3600 s (60 min)"));
    TEST_ASSERT_NOT_NULL(std::strstr(line, "credited"));

    const SurfaceInterval none = surfaceIntervalS(r.kind, r.epoch, 0u);
    continuitySeedText(line, sizeof(line), &r, 0u, none);
    TEST_ASSERT_NOT_NULL(std::strstr(line, "NO surface"));
    TEST_ASSERT_NOT_NULL(std::strstr(line, "this boot has no clock"));
    TEST_ASSERT_NOT_NULL(std::strstr(line, "conservative"));

    // A mid-dive snapshot is said as what it is, not as a surface interval.
    ContinuityRecord mid = sample(12u, 1800000000u, CONT_KIND_PERIODIC);
    const SurfaceInterval midIv = surfaceIntervalS(mid.kind, mid.epoch, 1800003600u);
    continuitySeedText(line, sizeof(line), &mid, 1800003600u, midIv);
    TEST_ASSERT_NOT_NULL(std::strstr(line, "mid-dive snapshot"));
    TEST_ASSERT_NOT_NULL(std::strstr(line, "cannot say when the diver surfaced"));
}

// Every loss reason has to be sayable, and "a dive starts fresh" has to name
// which of the four it was.
void test_every_loss_reason_is_named() {
    char line[256];
    const uint8_t reasons[] = {CONT_LOSS_ABSENT, CONT_LOSS_BAD, CONT_LOSS_STORE, CONT_LOSS_SEED};
    for (const uint8_t why : reasons) {
        continuityLossText(line, sizeof(line), why);
        TEST_ASSERT_NOT_NULL(std::strstr(line, "NO CONTINUITY"));
        TEST_ASSERT_TRUE(std::strlen(line) > std::strlen("NO CONTINUITY: dive starts from clean tissues ()"));
    }
    TEST_ASSERT_NOT_NULL(std::strstr(contLossName(CONT_LOSS_BAD), "does not decode"));
    TEST_ASSERT_NOT_NULL(std::strstr(contLossName(CONT_LOSS_STORE), "will not read"));
    TEST_ASSERT_NOT_NULL(std::strstr(contLossName(CONT_LOSS_ABSENT), "no record"));
    TEST_ASSERT_NOT_NULL(std::strstr(contKindName(CONT_KIND_PERIODIC), "mid-dive"));
    TEST_ASSERT_NOT_NULL(std::strstr(contKindName(CONT_KIND_DIVE_END), "dive end"));
}

// Numbers are printed as integers: this device's float printf has rebooted the
// controller twice (tools/float_audit.py), and an age is not a number anybody
// needs a decimal for.
void test_honesty_text_prints_no_floats() {
    char line[384];
    ContinuityRecord r = sample(11u, 1800000001u, CONT_KIND_DIVE_END);
    const SurfaceInterval iv = surfaceIntervalS(r.kind, r.epoch, 1800003601u);
    continuitySeedText(line, sizeof(line), &r, 1800003601u, iv);
    TEST_ASSERT_NOT_NULL(std::strstr(line, "3600 s (60 min)"));   // 3600, not 3.6e3
    for (const char* p = line; *p; ++p)
        TEST_ASSERT_TRUE_MESSAGE(*p != '.' || std::isdigit(static_cast<unsigned char>(p[1])),
                                 "a bare decimal point in a device sentence");
}

// ---- who writes, and who must not ------------------------------------------
// contgate.h exists because the snapshot has to be taken by the task that owns
// the engine (ControlTask) and written by a task that can afford to wait for the
// log mutex (HudTask). ControlTask's stall budget is 1500 ms and the log mutex is
// the one a multi-second dive-log export holds on the command task.

void test_posting_never_calls_the_writer() {
    ContinuityGate g;
    bool writerCalled = false;
    g.setWriter([&](const ContinuityRecord&, uint32_t) {
        writerCalled = true;
        return ContinuityGate::WRITTEN;
    });
    ContinuityRecord r = sample(1u, 1800000000u, CONT_KIND_DIVE_END);
    for (int i = 0; i < 50; ++i) g.post(r);
    TEST_ASSERT_FALSE_MESSAGE(writerCalled,
                              "post() is ControlTask's half: it must copy, never write");
    TEST_ASSERT_TRUE(g.pending());
    TEST_ASSERT_EQUAL_UINT32(50u, g.posted());
    TEST_ASSERT_EQUAL_UINT32(49u, g.lostSnapshots());   // displaced, and counted
}

// A snapshot that never reaches flash is a fact, not a shrug: it is counted, and
// the HB line prints the count.
void test_displaced_and_refused_snapshots_are_counted() {
    ContinuityGate g;
    int calls = 0;
    g.setWriter([&](const ContinuityRecord&, uint32_t) {
        ++calls;
        return ContinuityGate::BUSY;                    // the log mutex is held by an export
    });
    ContinuityRecord r = sample(1u, 1800000000u, CONT_KIND_DIVE_END);
    g.post(r);
    TEST_ASSERT_EQUAL_INT(ContinuityGate::BUSY, g.drain(20));
    TEST_ASSERT_EQUAL_INT(ContinuityGate::BUSY, g.drain(0));   // retry, zero wait
    TEST_ASSERT_EQUAL_UINT32(0u, g.lostSnapshots());    // still pending: nothing lost yet
    TEST_ASSERT_EQUAL_UINT32(2u, g.retryCount());

    // A store that refuses outright is not retried forever: the record is dropped
    // (a newer one is better anyway) and counted where the bench can see it.
    g.setWriter([&](const ContinuityRecord&, uint32_t) {
        ++calls;
        return ContinuityGate::FAILED;
    });
    TEST_ASSERT_EQUAL_INT(ContinuityGate::FAILED, g.drain(20));
    TEST_ASSERT_EQUAL_UINT32(1u, g.lostSnapshots());
    TEST_ASSERT_FALSE(g.pending());
    TEST_ASSERT_EQUAL_INT(ContinuityGate::WRITTEN, g.drain(20));   // nothing pending
}

// What is written is what was posted — including the record identity and the
// snapshot's own epoch, which must be the SAMPLE instant and not the (later,
// deferred) write instant, or the age the next dive restores by is wrong.
void test_the_gate_hands_the_writer_exactly_what_was_posted() {
    ContinuityGate g;
    ContinuityRecord written{};
    bool sawPending = false;
    g.setWriter([&](const ContinuityRecord& r, uint32_t) {
        sawPending = true;
        written = r;
        return ContinuityGate::WRITTEN;
    });
    ContinuityRecord r = sample(3u, 1800000600u, CONT_KIND_DIVE_END);
    r.flags = CONT_FLAG_CLOCK_CARRIED;
    g.post(r);
    TEST_ASSERT_EQUAL_INT(ContinuityGate::WRITTEN, g.drain(20));
    TEST_ASSERT_TRUE(sawPending);
    TEST_ASSERT_EQUAL_UINT32(3u, written.seq);
    TEST_ASSERT_EQUAL_UINT32(1800000600u, written.epoch);
    TEST_ASSERT_EQUAL_UINT8(CONT_KIND_DIVE_END, written.kind);
    TEST_ASSERT_EQUAL_UINT8(CONT_FLAG_CLOCK_CARRIED, written.flags);
    TEST_ASSERT_EQUAL_UINT32(1u, g.writtenThroughGate());
}

// A record that reached the store is the newest truth this board has: the seed
// the next dive consumes has to come from it, in the same boot.
void test_a_written_record_is_what_the_next_dive_is_seeded_from() {
    ContinuitySlots slots;
    ContinuityGate g;
    g.setWriter([&](const ContinuityRecord& in, uint32_t) {
        ContinuityRecord r = in;
        r.seq = slots.nextSeq();
        const bool ok = slots.write(slots.freeSlot(), r);
        return ok ? ContinuityGate::WRITTEN : ContinuityGate::FAILED;
    });

    ContinuityRecord r = sample(0u, 1800000600u, CONT_KIND_DIVE_END);
    TEST_ASSERT_EQUAL_UINT32(0u, r.seq);              // the caller does not decide seq
    g.post(r);
    TEST_ASSERT_EQUAL_INT(ContinuityGate::WRITTEN, g.drain(20));

    ContinuityRecord stored{};
    TEST_ASSERT_TRUE(slots.readNewest(stored));
    TEST_ASSERT_EQUAL_UINT32(1u, stored.seq);         // the store assigned it
    const SurfaceInterval iv = surfaceIntervalS(stored.kind, stored.epoch, 1800004200u);
    TEST_ASSERT_EQUAL_UINT32(3600u, iv.seconds);      // and it is still creditable
}

// ============================================================================
int main() {
    UNITY_BEGIN();
    RUN_TEST(test_record_layout_is_the_stated_size_and_stride);
    RUN_TEST(test_codec_round_trips_every_field);
    RUN_TEST(test_no_field_is_dropped_between_pack_and_unpack);
    RUN_TEST(test_any_bit_flip_anywhere_in_the_record_refuses_to_decode);
    RUN_TEST(test_erasures_and_foreign_bytes_are_no_record);
    RUN_TEST(test_numbers_no_dive_could_produce_are_refused_even_with_a_good_crc);
    RUN_TEST(test_scaled_integers_survive_the_round_trip_to_one_part_in_10000);
    RUN_TEST(test_out_of_range_engine_state_writes_no_record);
    RUN_TEST(test_a_slot_write_leaves_the_other_slot_and_the_gaps_untouched);
    RUN_TEST(test_writes_always_replace_the_older_slot);
    RUN_TEST(test_a_torn_slot_is_written_before_the_valid_one);
    RUN_TEST(test_a_short_image_decodes_as_no_record_not_as_garbage);
    RUN_TEST(test_ordering_is_by_sequence_and_the_counter_never_rewraps);
    RUN_TEST(test_a_dive_end_record_credits_the_clock_difference);
    RUN_TEST(test_a_mid_dive_snapshot_never_credits_an_interval);
    RUN_TEST(test_an_interval_that_cannot_be_established_credits_nothing);
    RUN_TEST(test_age_is_reported_separately_from_credit);
    RUN_TEST(test_boot_banner_keeps_cannot_read_apart_from_nothing_there);
    RUN_TEST(test_dive_start_line_states_the_seed_and_the_credit_or_its_absence);
    RUN_TEST(test_every_loss_reason_is_named);
    RUN_TEST(test_honesty_text_prints_no_floats);
    RUN_TEST(test_posting_never_calls_the_writer);
    RUN_TEST(test_displaced_and_refused_snapshots_are_counted);
    RUN_TEST(test_the_gate_hands_the_writer_exactly_what_was_posted);
    RUN_TEST(test_a_written_record_is_what_the_next_dive_is_seeded_from);
    return UNITY_END();
}
