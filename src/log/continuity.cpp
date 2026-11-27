// ============================================================================
// continuity.cpp — see continuity.h. Explicit little-endian encoding, scaled
// integers, one CRC per record: the same rules as every other record on this
// partition (divelog.cpp), because a record written minutes before a diver goes
// back down is read by a machine that has no way to argue with it.
// ============================================================================
#include "continuity.h"

#include <cstdio>
#include <cstring>

#include "util/numfmt.h"   // scaled()/narrow<>(): the repo's one rounding rule
                           // and the one narrowing that cannot wrap

namespace logfmt {

namespace {

void wr16(uint8_t*& p, uint16_t v) { *p++ = static_cast<uint8_t>(v & 0xFF); *p++ = static_cast<uint8_t>(v >> 8); }
void wr24(uint8_t*& p, uint32_t v) { wr16(p, static_cast<uint16_t>(v)); *p++ = static_cast<uint8_t>((v >> 16) & 0xFF); }
void wr32(uint8_t*& p, uint32_t v) { wr16(p, static_cast<uint16_t>(v)); wr16(p, static_cast<uint16_t>(v >> 16)); }

uint16_t rd16(const uint8_t*& p) { const uint16_t v = static_cast<uint16_t>(p[0] | (p[1] << 8)); p += 2; return v; }
// Reads EXACTLY three bytes, to pair with wr24(). An earlier version of this
// helper also stepped over a fourth byte, which silently shifted every field
// after the sequence number by one and made every record unreadable — caught by
// test_continuity's round-trip, which is the only reason a bug like that cannot
// reach a partition again.
uint32_t rd24(const uint8_t*& p) { const uint32_t lo = rd16(p); const uint32_t hi = *p++; return lo | (hi << 16); }
// Sequenced on purpose. `return rd16(p) | (rd16(p) << 16)` is legal C++ and
// wrong code: the two calls are unsequenced within one expression, so which half
// of the word is read first is the compiler's, and the second read starts at an
// arbitrary offset. Every record on this partition is read through helpers like
// this one, and the codec's round-trip test (test_continuity) is what catches it.
uint32_t rd32(const uint8_t*& p) { const uint32_t lo = rd16(p); const uint32_t hi = rd16(p); return lo | (hi << 16); }

/// Field offsets, spelled out because the tests below cite them: the codec's
/// byte layout is the contract a host tool reads off a board forever, and a
/// stride change that moves one of these silently corrupts every record after it
/// (the dive-file codec did exactly that, writing 34 B into a 32 B record).
constexpr size_t kOFF_MAGIC = 0;
constexpr size_t kOFF_VERSION = 4;
constexpr size_t kOFF_KIND = 5;
constexpr size_t kOFF_FLAGS = 6;
constexpr size_t kOFF_NCOMP = 7;
constexpr size_t kOFF_SEQ = 8;
constexpr size_t kOFF_EPOCH = 12;
constexpr size_t kOFF_TISSUE = 16;
constexpr size_t kOFF_CNS = 80;
constexpr size_t kOFF_CRC = 84;

} // namespace

size_t packContinuity(const ContinuityRecord& r, uint8_t out[CONT_RECORD_SIZE]) {
    std::memset(out, 0, CONT_RECORD_SIZE);
    uint8_t* p = out;
    wr32(p, CONT_MAGIC);
    *p++ = CONT_VERSION;
    *p++ = r.kind;
    *p++ = r.flags;
    *p++ = CONT_NCOMP;          // the encoder always writes the engine's own
                                // count; a reader that sees another number is
                                // looking at a different engine's payload
    wr24(p, r.seq > SEQ_MAX ? SEQ_MAX : r.seq);
    *p++ = 0;                   // reserved (was padding, kept zero)
    wr32(p, r.epoch);
    for (int i = 0; i < CONT_NCOMP; ++i) wr32(p, static_cast<uint32_t>(r.tissueX1e4[i]));
    wr16(p, r.cnsX10);
    wr16(p, 0);                 // reserved
    // One CRC per record, over everything before it — the torn-write contract
    // this partition uses everywhere: a reset between two writes costs that
    // write, it never leaves a record that decodes to plausible garbage.
    wr16(p, crc16(out, kOFF_CRC));
    return CONT_RECORD_SIZE;
}

bool unpackContinuity(const uint8_t in[CONT_RECORD_SIZE], ContinuityRecord& out) {
    const uint8_t* m = in + kOFF_MAGIC;
    const uint32_t magic = static_cast<uint32_t>(m[0]) | (static_cast<uint32_t>(m[1]) << 8) |
                           (static_cast<uint32_t>(m[2]) << 16) | (static_cast<uint32_t>(m[3]) << 24);
    if (magic != CONT_MAGIC) return false;   // erased flash / another file's bytes
    if (crc16(in, kOFF_CRC) !=
        static_cast<uint16_t>(in[kOFF_CRC] | (in[kOFF_CRC + 1] << 8)))
        return false;
    if (in[kOFF_VERSION] != CONT_VERSION) return false;  // unknown layout: stop
    if (in[kOFF_NCOMP] != CONT_NCOMP) return false;      // different engine, do not guess

    ContinuityRecord r{};
    const uint8_t* p = in + kOFF_KIND;
    r.kind = *p++;
    r.flags = *p++;
    r.ncomp = *p++;
    r.seq = rd24(p);
    p += 1;                     // reserved
    r.epoch = rd32(p);
    int32_t t[CONT_NCOMP];
    for (int i = 0; i < CONT_NCOMP; ++i) t[i] = static_cast<int32_t>(rd32(p));
    r.cnsX10 = rd16(p);

    // Range-checked HERE, not left to the engine: a CRC pass on a record written
    // by a different build, or a half-torn slot the CRC happened to like, must
    // not reach a diver as a number. A tissue outside (0, 100] bar is not a
    // tissue pressure, and CNS above the engine's own cap is not a clock.
    for (int i = 0; i < CONT_NCOMP; ++i) {
        if (t[i] <= 0 || t[i] > CONT_TISSUE_MAX_BAR * CONT_TISSUE_SCALE) return false;
        r.tissueX1e4[i] = t[i];
    }
    if (r.cnsX10 > CONT_CNS_MAX_PCT * CONT_CNS_SCALE) return false;
    out = r;
    return true;
}

bool toTissueState(const ContinuityRecord& r, deco::Zhl16Ccr::TissueState& out) {
    if (r.ncomp != CONT_NCOMP) return false;
    deco::Zhl16Ccr::TissueState s{};
    for (int i = 0; i < CONT_NCOMP; ++i) {
        if (r.tissueX1e4[i] <= 0 ||
            r.tissueX1e4[i] > CONT_TISSUE_MAX_BAR * CONT_TISSUE_SCALE) return false;
        s.pt[i] = static_cast<double>(r.tissueX1e4[i]) / CONT_TISSUE_SCALE;
    }
    if (r.cnsX10 > CONT_CNS_MAX_PCT * CONT_CNS_SCALE) return false;
    s.cnsPct = static_cast<double>(r.cnsX10) / CONT_CNS_SCALE;
    out = s;
    return true;
}

bool fromTissueState(const deco::Zhl16Ccr::TissueState& st, uint8_t kind,
                     ContinuityRecord& out) {
    ContinuityRecord r{};
    r.kind = kind;
    r.ncomp = CONT_NCOMP;
    for (int i = 0; i < CONT_NCOMP; ++i) {
        // numfmt::scaled is the repo's one rounding rule for a stored number
        // (nearest, ties to the EVEN side, decided on the exact product) — the
        // same rule countsFromFloat reaches by rendering through numfmt::fixed
        // and reading the digits back, and the reason it is one rule is the
        // identity numfmt.h publishes: fixedScaled(scaled(v, d), d) == fixed(v, d).
        // These two lines used to hand-roll `* SCALE + 0.5` and truncate, which
        // agrees with that rule except on an exact tie, where it rounds half UP:
        // a tissue at 12.34005 bar stored 123401 while the console beside it
        // printed 12.3400 (nearest-even). One count, and the record is the thing
        // a diver or an investigator reads afterwards, so "close enough" is the
        // wrong answer. The layout did not change — the version byte is about
        // bytes, not about arithmetic — so old records still read; the change is
        // that a fresh record's value now matches its own printed digits.
        const long long v = numfmt::scaled(st.pt[i], CONT_TISSUE_DECIMALS);
        // scaled() answers 0 for a non-finite input, so a NaN or inf tissue
        // pressure falls out here rather than writing a plausible zero.
        if (v <= 0 || v > static_cast<long long>(CONT_TISSUE_MAX_BAR) * CONT_TISSUE_SCALE)
            return false;
        r.tissueX1e4[i] = numfmt::narrow<int32_t>(v);
    }
    // The float is range-checked BEFORE scaling, not after, because 0 % is a
    // legal CNS and numfmt::scaled answers 0 for a NaN or an inf — a post-scale
    // range check cannot tell "no nitrogen load" from "the engine lost the
    // number", and the difference is exactly what a continuity record is for.
    if (!(st.cnsPct >= 0.0) || st.cnsPct > static_cast<double>(CONT_CNS_MAX_PCT))
        return false;   // NaN fails `>= 0.0`, so this covers non-finite too
    const long long cns = numfmt::scaled(st.cnsPct, CONT_CNS_DECIMALS);
    if (cns < 0 || cns > static_cast<long long>(CONT_CNS_MAX_PCT) * CONT_CNS_SCALE)
        return false;
    r.cnsX10 = numfmt::narrow<uint16_t>(cns);
    out = r;
    return true;
}

// ---- the surface interval ---------------------------------------------------

SurfaceInterval surfaceIntervalS(uint8_t recordKind, uint32_t recordEpoch,
                                 uint32_t nowEpoch) {
    SurfaceInterval iv;
    // The kind rule comes FIRST, before any clock question, because it is not a
    // clock question. A mid-dive snapshot was taken with the diver AT DEPTH; the
    // machine has no idea when he surfaced, or whether he did. Crediting surface
    // off-gassing from that instant would be crediting time he may have spent
    // still down there, which is the one error direction that buys a diver a
    // shallower ceiling than the truth. So a mid-dive record carries the loading
    // and gets NO interval credit — it exists to bound the crash window (the
    // loading is at most one snapshot period stale), not to age the dive.
    // A dive-END record is the opposite: everything after it is surface time by
    // construction, so the interval is exactly what it says it is.
    if (recordKind == CONT_KIND_PERIODIC) { iv.why = INT_MID_DIVE_RECORD; return iv; }
    if (recordEpoch == 0u) { iv.why = INT_NO_RECORD_CLOCK; return iv; }
    if (nowEpoch == 0u)    { iv.why = INT_NO_NOW_CLOCK; return iv; }
    // A record dated in the future means the clock moved backwards between the
    // write and now (a wrong TIME SET, or a carried epoch that was itself wrong).
    // There is no interval to integrate, and "negative" is worse than zero: the
    // honest answer is that the machine does not know how long the diver has
    // been out of the water, so it credits nothing.
    if (nowEpoch < recordEpoch) { iv.why = INT_EPOCH_AHEAD; return iv; }
    iv.seconds = nowEpoch - recordEpoch;
    iv.credit = iv.seconds > 0u;
    iv.why = INT_CREDIT;
    return iv;
}

int64_t continuityAgeS(uint32_t recordEpoch, uint32_t nowEpoch) {
    if (recordEpoch == 0u || nowEpoch == 0u) return -1;
    if (nowEpoch < recordEpoch) return -1;
    return static_cast<int64_t>(nowEpoch) - static_cast<int64_t>(recordEpoch);
}

// ---- the two-slot store ----------------------------------------------------

void ContinuitySlots::load(const uint8_t* b, size_t len) {
    clear();
    if (!b || len == 0) return;
    const size_t n = len > kImageSize ? kImageSize : len;
    std::memcpy(img_, b, n);
    // A SHORT image (a torn first write, a file truncated by a reset) keeps zeros
    // for the rest: those slots fail the magic, so the store reads "no record"
    // rather than inventing one out of a partial file.
}

bool ContinuitySlots::decode(int slot, ContinuityRecord& out) const {
    if (slot < 0 || slot >= kSlots) return false;
    return unpackContinuity(img_ + static_cast<size_t>(slot) * kStride, out);
}

int ContinuitySlots::newest() const {
    int best = -1;
    ContinuityRecord bestRec{};
    for (int s = 0; s < kSlots; ++s) {
        ContinuityRecord r{};
        if (!decode(s, r)) continue;
        if (best < 0) { best = s; bestRec = r; continue; }
        // Sequence first: it is monotonic across boots with no clock anywhere in
        // its derivation. Epoch only breaks a tie (two records that somehow share
        // a sequence), never overrides it.
        if (r.seq > bestRec.seq || (r.seq == bestRec.seq && r.epoch > bestRec.epoch)) {
            best = s;
            bestRec = r;
        }
    }
    return best;
}

int ContinuitySlots::freeSlot() const {
    // An empty or torn slot goes first: on a fresh board that is slot 0 then
    // slot 1, and a slot whose last write was torn is repaired before it is
    // ever asked to be the fallback.
    for (int s = 0; s < kSlots; ++s) {
        ContinuityRecord r{};
        if (!decode(s, r)) return s;
    }
    int oldest = 0;
    ContinuityRecord oldestRec{};
    decode(0, oldestRec);
    for (int s = 1; s < kSlots; ++s) {
        ContinuityRecord r{};
        decode(s, r);
        if (r.seq < oldestRec.seq || (r.seq == oldestRec.seq && r.epoch < oldestRec.epoch)) {
            oldest = s;
            oldestRec = r;
        }
    }
    return oldest;
}

uint32_t ContinuitySlots::nextSeq() const {
    uint32_t hi = 0;
    for (int s = 0; s < kSlots; ++s) {
        ContinuityRecord r{};
        if (decode(s, r) && r.seq > hi) hi = r.seq;
    }
    if (hi >= SEQ_MAX) return SEQ_MAX;   // saturates: stops ordering, never re-wraps
    return hi + 1u;
}

bool ContinuitySlots::write(int slot, const ContinuityRecord& r) {
    if (slot < 0 || slot >= kSlots) return false;
    return packContinuity(r, img_ + static_cast<size_t>(slot) * kStride) == CONT_RECORD_SIZE;
}

// ---- honesty text ----------------------------------------------------------

const char* contKindName(uint8_t kind) {
    switch (kind) {
        case CONT_KIND_DIVE_END: return "dive end";
        case CONT_KIND_PERIODIC: return "mid-dive snapshot";
        default: return "unknown kind";
    }
}

const char* contLossName(uint8_t reason) {
    switch (reason) {
        case CONT_LOSS_ABSENT: return "no record on the partition";
        case CONT_LOSS_BAD: return "the record there does not decode";
        case CONT_LOSS_STORE: return "the continuity store will not read";
        case CONT_LOSS_SEED: return "the record decoded and the chain refused its numbers";
        default: return "unknown reason";
    }
}

const char* contIntervalName(uint8_t why) {
    switch (why) {
        case INT_NO_RECORD_CLOCK: return "the record was written with no clock";
        case INT_NO_NOW_CLOCK: return "this boot has no clock";
        case INT_EPOCH_AHEAD: return "the record is dated after this clock";
        case INT_MID_DIVE_RECORD: return "a mid-dive snapshot cannot say when the diver surfaced";
        case INT_CREDIT: return "interval established";
        default: return "unknown";
    }
}

namespace {

/// "3612 s (60 min)" — integers only. A record's age is read by a bench log and
/// by a diver, and neither needs a decimal to act on it.
size_t ageText(char* out, size_t cap, int64_t ageS) {
    if (ageS < 0) return static_cast<size_t>(std::snprintf(out, cap, "age unknown (no clock)"));
    return static_cast<size_t>(std::snprintf(out, cap, "%lld s (%lld min)",
                                             static_cast<long long>(ageS),
                                             static_cast<long long>(ageS / 60)));
}

} // namespace

size_t continuityBootText(char* out, size_t cap, bool storeReadable, bool corrupt,
                          const ContinuityRecord* rec, uint32_t nowEpoch) {
    if (!out || cap == 0) return 0;
    if (!storeReadable)
        return static_cast<size_t>(std::snprintf(out, cap,
            "** /continuity.bin will not read: every dive starts from clean tissues"));
    if (corrupt)
        return static_cast<size_t>(std::snprintf(out, cap,
            "** a record file is present and nothing in it decodes: the carried "
            "dive record is gone, and dives start from clean tissues"));
    if (!rec)
        return static_cast<size_t>(std::snprintf(out, cap,
            "no record yet: the first dive on this board starts from clean tissues"));
    char age[48];
    ageText(age, sizeof(age), continuityAgeS(rec->epoch, nowEpoch));
    return static_cast<size_t>(std::snprintf(out, cap,
        "record %lu from a %s, %s, carried into the next dive%s",
        static_cast<unsigned long>(rec->seq), contKindName(rec->kind), age,
        (rec->flags & CONT_FLAG_CLOCK_CARRIED) ? "  [clock carried, unverified]" : ""));
}

size_t continuitySeedText(char* out, size_t cap, const ContinuityRecord* rec,
                          uint32_t nowEpoch, const SurfaceInterval& iv) {
    if (!out || cap == 0 || !rec) return 0;
    char age[48];
    ageText(age, sizeof(age), continuityAgeS(rec->epoch, nowEpoch));
    if (iv.credit)
        return static_cast<size_t>(std::snprintf(out, cap,
            "dive SEEDED from record %lu (%s): tissues and CNS carried, %s of surface "
            "off-gassing credited",
            static_cast<unsigned long>(rec->seq), contKindName(rec->kind), age));
    return static_cast<size_t>(std::snprintf(out, cap,
        "dive SEEDED from record %lu (%s): tissues and CNS carried with NO surface "
        "credit (%s; %s) — conservative: a deeper ceiling than the truth, never a shallower one",
        static_cast<unsigned long>(rec->seq), contKindName(rec->kind), age,
        contIntervalName(iv.why)));
}

size_t continuityLossText(char* out, size_t cap, uint8_t lossReason) {
    if (!out || cap == 0) return 0;
    return static_cast<size_t>(std::snprintf(out, cap,
        "NO CONTINUITY: dive starts from clean tissues (%s)",
        contLossName(lossReason)));
}

} // namespace logfmt
