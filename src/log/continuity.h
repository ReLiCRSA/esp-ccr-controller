// ============================================================================
// continuity.h — the record that carries a diver's tissues across a power cycle
// (review H1: "the one gap that can kill without any instrument failing").
//
// Every startDive() reset deco_, so a diver who surfaced with deco debt and
// dived again an hour later got a ceiling, an NDL and a deco plan computed as if
// the first dive had never happened. Acceptance scenario this file exists for:
// dive 2, 60 minutes later, must show a ceiling ON ENTRY.
//
// Pure codec + a pure two-slot record store. The file calls live in
// log/continuity_store.{h,cpp} (LittleFS, target-only) exactly as the dive-file
// calls live in dive_store.cpp: the byte format, the CRC gate and the ordering
// rule are host-tested here, and the store adapter is thin plumbing that the
// native env does not build. The soak (src/sim/rig.cpp) drives THESE classes
// directly, so the simulator's continuity behaviour is the shipping logic minus
// the file calls — not a mirror of it, which is the failure mode rowfill.h was
// written to remove.
//
// ---- what is stored, and what deliberately is not --------------------------
// The engine's whole seedable state: 16 tissue loadings and the CNS clock.
// ZH-L16C as implemented is ONE inert gas (no helium array anywhere in
// deco/zhl16c.cpp), so 16 loadings round-trip, not 16+16.
// NOT stored, on purpose:
//   * the last ceiling, the NDL, the ascent schedule — all three are pure
//     functions of the loadings and the gradient factors (ceilingBarFrom(pt, gf),
//     ndlMinutes, schedule). A stored ceiling would be a number computed under
//     the GF pair in force when the record was written, and the diver is allowed
//     to change GF between dives; the machine would then be planning one dive on
//     another dive's rules. Derived values are derived again.
//   * the surface interval itself: it is arithmetic between the record's own
//     epoch and the clock at dive start, and it must be computed at the moment
//     the dive starts, not frozen when the record was written (a board that
//     boots and sits two days before diving is owed two days of off-gassing).
//   * the depth, setpoint, gas, water density: the loadings already encode what
//     those did, and re-deriving anything else from them would be inventing a
//     history.
//
// ---- numbers on flash are integers, not float bits -------------------------
// Every other record in this repo stores scaled integers (divelog.cpp: explicit
// little-endian, "interpretable from any host tool forever"). Tissue pressures
// go in as bar x 10 000 (0.1 mbar resolution: ~0.1 cm of water of ceiling,
// against an engine that integrates in double). That choice also deletes a whole
// failure class: a float image torn by a reset can be a NaN or a 1e30, and
// isfinite() has to catch it; a torn int32 is a number, and a RANGE check rejects
// it. Range checks are in both the codec and the engine (Zhl16Ccr::setTissueState)
// because a plausible-but-wrong loading is the one outcome this record must
// never produce: it is the difference between a ceiling and no ceiling, and
// nothing downstream of it has any reason to doubt the number.
//
// ---- two slots, so a torn write cannot take the whole picture --------------
// A record file that is rewritten in place has ONE copy of the truth: a reset in
// the middle of writing it costs the diver the continuity this file exists to
// give him. So the store is two fixed slots, and every write replaces the OLDER
// one: at the instant power dies there is always a complete, valid, older record
// left — the worst case is losing the most recent snapshot, which is exactly the
// crash-window the write cadence is sized around. A CRC failure on both slots is
// still survivable (today's behavior: fresh tissues), but it is no longer silent:
// the dive-start verdict record (EVT_CONT_LOST) says so, with the reason.
// ============================================================================
#pragma once

#include <cstdint>
#include <cstddef>

#include "deco/zhl16c.h"
#include "log/divelog.h"   // SEQ_MAX: one definition for every stored counter

namespace logfmt {

constexpr uint32_t CONT_MAGIC = 0x4F434252;   // "RBCO" little-endian
constexpr uint8_t CONT_VERSION = 1;
constexpr uint8_t CONT_NCOMP = deco::Zhl16Ccr::NCOMP;  // 16: one inert gas

// 84 B of fields + CRC16. Fixed stride, like every record on this partition.
constexpr size_t CONT_RECORD_SIZE = 86;

// The slots are laid out on a stride WIDER than the record: a v2 that grows the
// payload (a real helium set: +16 loadings = +64 B) still fits inside one slot,
// so the file layout does not have to change under a diver's feet.
constexpr size_t CONT_SLOT_STRIDE = 192;
constexpr int CONT_SLOTS = 2;

// Scales. Tissue pressure in 0.1 mbar counts; CNS in tenths of a percent (the
// engine caps the clock at 999 %, which fits uint16 at that scale).
constexpr int32_t CONT_TISSUE_SCALE = 10000;
constexpr int32_t CONT_TISSUE_MAX_BAR = 100;   // == Zhl16Ccr::kSeedMaxBar
constexpr int32_t CONT_CNS_SCALE = 10;
constexpr int32_t CONT_CNS_MAX_PCT = 999;      // the engine's own cap

/// What kind of moment the record was taken at. It is not decoration: it decides
/// what the machine may claim about the interval that followed (see
/// surfaceIntervalS()). A dive-end record is followed by surface time by
/// construction, so the elapsed clock difference is off-gassing time. A mid-dive
/// one is not — the diver may still have been down there — so it credits nothing
/// and only bounds how stale the loading can be. Both answers go on the console
/// and into the verdict record, because "seeded from a 40 m snapshot" and "seeded
/// from where you left the water" are different pictures.
enum ContKind : uint8_t {
    CONT_KIND_NONE = 0,
    CONT_KIND_DIVE_END = 1,   // written when the dive closed
    CONT_KIND_PERIODIC = 2,   // written on the crash-window grid, during a dive
};

enum ContFlags : uint8_t {
    CONT_FLAG_CLOCK_CARRIED = 0x01,  // the epoch came from a clock carried across
                                     // a reset: never verified since (wallclock.h)
    CONT_FLAG_CLOCK_UNSET = 0x02,    // there was no clock when this was written
};

/// Why a dive is starting with clean tissues when a record was expected. This is
/// the `value` column of the EVT_CONT_LOST verdict record: "continuity lost" is
/// one sentence, and the reason is the part a reader can act on.
enum ContLoss : uint8_t {
    CONT_LOSS_NONE = 0,
    CONT_LOSS_ABSENT = 1,     // nothing on the partition: first dive on this board
    CONT_LOSS_BAD = 2,        // bytes are there and do not decode (CRC/version/
                              // compartment count), or they decode to numbers no
                              // dive could have produced
    CONT_LOSS_STORE = 3,      // the store will not read at all (mount/permission)
    CONT_LOSS_SEED = 4,       // the record decoded; the engine refused the numbers
};

/// Why the surface interval was NOT credited, when it was not. All three answers
/// give ZERO credit, and that is the point: an interval the machine cannot
/// establish is an interval in which the diver may still have been at depth, so
/// crediting off-gassing would understate the debt. Tissues carried with no
/// credit can only ever ask for more deco than the truth; tissues carried with
/// invented credit can ask for less. Conservative is the only direction that is
/// wrong safely.
enum IntervalWhy : uint8_t {
    INT_CREDIT = 0,             // interval established: seconds credited
    INT_NO_RECORD_CLOCK = 1,    // the record was written with no wall clock
    INT_NO_NOW_CLOCK = 2,       // this boot has no wall clock
    INT_EPOCH_AHEAD = 3,        // the record is dated AFTER now: the clock moved
                                // backwards (a wrong TIME SET) — no interval exists
    INT_MID_DIVE_RECORD = 4,    // the record is a MID-DIVE snapshot: it cannot say
                                // when the diver surfaced, so none of the interval
                                // after it is provably surface time. See the kind
                                // rule below — this is the one answer that has
                                // nothing to do with clocks.
};

struct ContinuityRecord {
    uint8_t kind = CONT_KIND_NONE;
    uint8_t flags = 0;
    uint8_t ncomp = CONT_NCOMP;   // must equal CONT_NCOMP or the payload is a
                                  // different engine's and will not be guessed at
    uint32_t seq = 0;             // 1..SEQ_MAX, monotonic across boots, read back
                                  // off the slots (the dive FIFO's culture: this
                                  // board has no clock to order anything with)
    uint32_t epoch = 0;           // wall clock AT THE SNAPSHOT, 0 = none. Not the
                                  // write time: a deferred write must not make
                                  // 60-second-old tissues look fresher than they
                                  // are, or the age it restores by is wrong.
    int32_t tissueX1e4[CONT_NCOMP] = {};   // bar x CONT_TISSUE_SCALE
    uint16_t cnsX10 = 0;                   // % x CONT_CNS_SCALE
};

size_t packContinuity(const ContinuityRecord& r, uint8_t out[CONT_RECORD_SIZE]);
/// false = do not decode, do not guess: magic, version, compartment count, CRC or
/// a payload outside the sane range. Every one of those answers the same way to
/// the machine, and the caller turns it into CONT_LOSS_BAD.
bool unpackContinuity(const uint8_t in[CONT_RECORD_SIZE], ContinuityRecord& out);

/// record -> engine state. false when a stored number is outside what a dive
/// could produce (the same rule unpackContinuity applies; spelled again here so
/// the conversion is safe on a hand-built record too).
bool toTissueState(const ContinuityRecord& r, deco::Zhl16Ccr::TissueState& out);
/// engine state -> record. false when a number will not fit the column, in which
/// case the caller writes NO record rather than a wrong one (countsFromFloat's
/// rule, for the same reason).
bool fromTissueState(const deco::Zhl16Ccr::TissueState& st, uint8_t kind,
                     ContinuityRecord& out);

/// The surface interval between a record and the dive that wants to use it, and
/// whether any of it may be credited at all.
///
/// KIND MATTERS, and it is not a detail. A DIVE-END record bounds a surface
/// interval: everything after it is time out of the water, so the elapsed clock
/// difference IS off-gassing time. A PERIODIC (mid-dive) record does not: it was
/// taken at depth, and nothing on this board knows when the diver came up. The
/// mid-dive record exists to bound the CRASH WINDOW — after a power cut the
/// loading is at most one snapshot period stale — and it gets NO interval credit,
/// because crediting surface time from an instant when the diver was definitely
/// still down there is the one error that buys him a shallower ceiling than the
/// truth. That rule is why the kind is a stored field rather than a spare byte.
struct SurfaceInterval {
    uint32_t seconds = 0;
    bool credit = false;
    uint8_t why = INT_CREDIT;
};

SurfaceInterval surfaceIntervalS(uint8_t recordKind, uint32_t recordEpoch,
                                 uint32_t nowEpoch);

/// Age of the record at boot/dive start, or -1 when no honest answer exists.
int64_t continuityAgeS(uint32_t recordEpoch, uint32_t nowEpoch);

// ---- the two-slot record store, as pure bytes ------------------------------
// The LittleFS adapter owns a file of exactly CONT_SLOTS * CONT_SLOT_STRIDE
// bytes and copies it into one of these; the soak owns one outright. Every
// decision about WHICH slot, whether a slot decodes, and what the next sequence
// number is, lives here — so all of it is host-tested.
class ContinuitySlots {
public:
    static constexpr int kSlots = CONT_SLOTS;
    static constexpr size_t kStride = CONT_SLOT_STRIDE;
    static constexpr size_t kImageSize = CONT_SLOTS * CONT_SLOT_STRIDE;

    /// All zeros. An erased LittleFS partition reads as exactly this, and a
    /// zeroed slot fails the magic — "no record", which is the truth.
    void clear() { for (size_t i = 0; i < kImageSize; ++i) img_[i] = 0; }

    /// Loads an image read from flash. Short/absent images are padded with
    /// zeros, which decodes as "nothing there" rather than as garbage.
    void load(const uint8_t* bytes, size_t len);
    const uint8_t* bytes() const { return img_; }

    bool decode(int slot, ContinuityRecord& out) const;
    /// The slot holding the newest valid record, or -1 when nothing decodes.
    /// Ordering is by SEQUENCE, not epoch: the sequence is monotonic across
    /// boots with no clock involved, which is the same reason the dive FIFO
    /// carries one (divelog.h). Epoch is only the tie-breaker.
    int newest() const;
    bool readNewest(ContinuityRecord& out) const {
        const int s = newest();
        return s >= 0 && decode(s, out);
    }
    /// The slot to write next: an empty/torn one first (a fresh board fills slot
    /// 0, then 1), otherwise the OLDER of the two — which is what keeps a reset
    /// during the write from costing the newest picture.
    int freeSlot() const;
    uint32_t nextSeq() const;
    /// Packs `r` into the slot. Returns false without touching anything if the
    /// slot number is out of range. (The caller has already assigned seq/kind/
    /// epoch: this object stores bytes, it does not decide facts.)
    bool write(int slot, const ContinuityRecord& r);
    /// Raw access to one slot's bytes, for FAULT INJECTION only (the soak
    /// damaging a record to prove the store reports it as corrupt rather than
    /// silently turning it into a fresh dive). Nothing in the shipping path
    /// touches this: the store writes through write(), which re-packs and
    /// re-CRCs. Returns null for an out-of-range slot.
    uint8_t* rawSlotBytes(int slot) {
        if (slot < 0 || slot >= kSlots) return nullptr;
        return img_ + static_cast<size_t>(slot) * kStride;
    }

private:
    uint8_t img_[kImageSize] = {};
};

// ---- honesty text ----------------------------------------------------------
// These sentences are the boot banner and the dive-start console line, and they
// live here because main.cpp is not compiled for the native env: a rule about
// what the machine SAYS, written in a file no test can read, is a rule nobody
// checks. Every number is rendered as an integer (float_audit: the device's
// float printf has rebooted this controller twice).

/// Boot banner, after "continuity: ". Three separate facts, and the banner must
/// keep them apart: the store will not read at all (`storeReadable` false), a
/// record file IS there but nothing in it decodes (`corrupt`), and there is
/// simply no record yet (`rec` null). "Nothing there" and "cannot read" look the
/// same as a silence, and a silence is how this bench gets lied to.
size_t continuityBootText(char* out, size_t cap, bool storeReadable, bool corrupt,
                          const ContinuityRecord* rec, uint32_t nowEpoch);

/// The dive-start line: which record seeded the dive, how old it was, and how
/// much surface interval was credited — or why none was.
size_t continuitySeedText(char* out, size_t cap, const ContinuityRecord* rec,
                          uint32_t nowEpoch, const SurfaceInterval& iv);

/// The dive-start line when nothing seeded it, and the EVT_CONT_LOST verdict
/// record says the same thing in numbers.
size_t continuityLossText(char* out, size_t cap, uint8_t lossReason);

const char* contKindName(uint8_t kind);      // "dive end" / "mid-dive snapshot"
const char* contLossName(uint8_t reason);    // for the console line
const char* contIntervalName(uint8_t why);   // for the console line

} // namespace logfmt
