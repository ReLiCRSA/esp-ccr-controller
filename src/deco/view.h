// ============================================================================
// view.h — the DECO PICTURE as a published value snapshot (reviews N1/N8/N5).
//
// Until 2026-11-28, everything that wanted "the decompression picture the
// machine believes right now" (the event-driven dive row, the handset `D,`
// line, `GET D`) walked ctrl::Loop::deco_ — a live engine owned and advanced
// by ControlTask — from ANOTHER task. rowfill.h's N8 paragraph called that
// "a benign torn read, tolerated, not endorsed" and left the footgun in the
// type: RowInputs::deco was a bare const pointer whose honesty was prose.
// A command task calling schedule() while ControlTask integrated a cycle
// could read a tissue set that never existed at one instant. It was tolerated
// because the alternative looked like a freshness loss. It is not — this file
// is the alternative:
//
//   * ControlTask OWNS deco_ and computes the picture from it (cheap scalars
//     every control cycle; the schedule() plan at the 30 s grid, at every
//     event edge, and at every continuity seeding — the trigger set that
//     guarantees any row or `D,` answer is reading the plan the machine
//     published for ITS instant, not one 30 s old).
//   * Everyone else reads a published COPY through a seqlock: the reader
//     either gets one self-consistent generation or retries. The torn read is
//     not tolerated anymore; it is unrepresentable — RowInputs::deco is a
//     value of this struct, and no consumer holds the engine at all.
//
// SCALED INTS, one rule: every field stores exactly the scale the surface
// that prints it uses (centimetres for the ceiling, like the dive file;
// minutes x10 like `D,`'s one decimal). numfmt::scaled with that decimal
// count — the store and the digits must agree, the same rule the CSV columns
// follow (2026-11-24).
//
// THE NDL SENTINELS (review N5) — one pair, one home, THIS header, identical
// on the dive file and on the wire:
//   kNDL_UNKNOWN   = -1   "no number": no valid vote, nothing to total.
//                         `D,` has answered this since v1 (§2: show "—").
//   kNDL_UNLIMITED = 480  the engine's own saturation: unlimited NDL.
// Before 2026-11-28 the FILE wrote 480 for BOTH facts — "unlimited" and
// "unanswerable" read alike, and the file and wire used different numbers
// for the same fact. One convention now serves both surfaces.
// ============================================================================
#pragma once

#include <atomic>
#include <cstdint>

#include "deco/zhl16c.h"
#include "util/numfmt.h"

namespace deco {

constexpr int16_t kNDL_UNKNOWN = -1;
constexpr int16_t kNDL_UNLIMITED = 480;

// The plan's stop list is carried at the width the `D,` line publishes
// (proto::MAX_STOPS in comms/uart_proto.h — keep the two numbers equal;
// deco cannot include comms, so the coupling is stated, not compiled).
constexpr uint8_t kViewStops = 8;

struct View {
    // ---- scalars: recomputed every control cycle (cheap: one ceiling solve
    //      + the NDL binary search, each bounded by design) ----
    int16_t ceilingCm = 0;      // 0 = no decompression obligation
    int16_t ndlMin = kNDL_UNKNOWN;
    int16_t cnsX10 = 0;         // CNS percent x10 — `D,` prints one decimal

    // ---- the plan: recomputed at the 30 s grid, at every event edge, at
    //      every continuity seeding, and at dive start/end ----
    int16_t decoTotalX10 = 0;   // minutes x10 (the dive-file column divides by 10)
    int16_t stopDepthCm[kViewStops] = {};
    int16_t stopMinX10[kViewStops] = {};
    uint8_t stopCount = 0;
    uint8_t capped = 0;         // schedule() hit its per-stop cap: plan incomplete

    bool haveVote = false;      // what the ndl sentinel was computed against
};

// ---- computation: ControlTask only (the only task allowed to read pt_) ----

// The cheap scalars: ceiling + NDL (measured host worst case: <=17 us at the
// deepest legal load — see the budget cases in test/test_controlplane).
// The NDL arithmetic takes only a MEASURED ppO2: a held value (N2) may show
// the diver the last believed number, but it may not reduce his NDL.
inline void viewScalars(View& v, const Zhl16Ccr& z, double pAmbBar,
                        bool haveVote, double shownPpo2) {
    v.haveVote = haveVote;
    v.ceilingCm = static_cast<int16_t>(numfmt::scaled(z.ceilingM(), 2));
    if (haveVote) {
        const double ndl = z.ndlMinutes(pAmbBar, shownPpo2);
        v.ndlMin = ndl >= static_cast<double>(kNDL_UNLIMITED)
                       ? kNDL_UNLIMITED
                       : (ndl < 0.0 ? 0 : static_cast<int16_t>(ndl));
    } else {
        v.ndlMin = kNDL_UNKNOWN;
    }
    v.cnsX10 = static_cast<int16_t>(numfmt::scaled(z.cnsPct(), 1));
}

// The full plan including schedule(). Cost shape: quadratic in bottom depth,
// capped twice — the comment at Zhl16Ccr::schedule() carries the numbers and
// the cadence rule (due-driven, never per 100 ms by default). It refreshes
// ONLY the plan fields: the scalars were refreshed this same cycle by
// viewScalars, which is the discipline the publisher in main.cpp keeps.
inline void viewPlan(View& v, const Zhl16Ccr& z, double pAmbBar) {
    const auto sch = z.schedule(pAmbBar);
    v.decoTotalX10 = static_cast<int16_t>(numfmt::scaled(sch.totalTimeMin, 1));
    v.stopCount = static_cast<uint8_t>(sch.count > kViewStops ? kViewStops : sch.count);
    for (int i = 0; i < v.stopCount; ++i) {
        v.stopDepthCm[i] = static_cast<int16_t>(numfmt::scaled(sch.stops[i].depthM, 2));
        v.stopMinX10[i] = static_cast<int16_t>(numfmt::scaled(sch.stops[i].minutes, 1));
    }
    v.capped = sch.capped ? 1u : 0u;
}

// Both halves at once — the sequence a grid row's picture takes (and what
// host tests use to reproduce ControlTask's publication exactly).
inline void viewFull(View& v, const Zhl16Ccr& z, double pAmbBar,
                     bool haveVote, double shownPpo2) {
    viewScalars(v, z, pAmbBar, haveVote, shownPpo2);
    viewPlan(v, z, pAmbBar);
}

// ---- publication: a seqlock, so the cross-task read is a fact, not a hope ----
// One writer (ControlTask). Readers on any task: a snapshot is valid iff the
// generation did not change across the copy. The writer never blocks and the
// critical region is a ~40-byte struct copy, so a reader retries at most once
// in practice; the retry bound below is the honesty, not an expectation.
struct ViewSlot {
    std::atomic<int32_t> seq{0};  // odd = mid-write; 0 = never published
    View view;
};

inline void publishView(ViewSlot& s, const View& fresh) {
    s.seq.fetch_add(1, std::memory_order_acq_rel);      // -> odd
    s.view = fresh;
    // The payload is an ordinary struct copy: nothing in its own code orders
    // it against the atomic below it. Without this release fence a compiler is
    // free to sink the field writes past the even bump, and a reader then sees
    // "generation stable" over a half-written picture — a valid-looking View
    // whose stop list belongs to two different generations. The fence is what
    // makes "even means complete" a guarantee rather than a habit.
    std::atomic_thread_fence(std::memory_order_release);
    s.seq.fetch_add(1, std::memory_order_acq_rel);      // -> even
}

// false = nothing published yet (the boot window before the first control
// cycle): the caller says "no picture" instead of inventing one.
inline bool viewSnapshot(const ViewSlot& s, View& out) {
    for (int attempt = 0; attempt < 8; ++attempt) {
        const int32_t a = s.seq.load(std::memory_order_acquire);
        if (a == 0) return false;
        out = s.view;
        // Same reasoning on the read side, and the same hazard mirrored: the
        // payload reads are plain loads, so without this acquire fence the
        // compiler may hoist them above the recheck (or keep registers from
        // the first pass), and the recheck then proves nothing about the bytes
        // actually copied. One fence between the payload and the second seq
        // read is the whole seqlock contract.
        std::atomic_thread_fence(std::memory_order_acquire);
        if (s.seq.load(std::memory_order_acquire) == a) return (a & 1) == 0;
    }
    return false;  // never observed a stable generation (pathological)
}

}  // namespace deco
