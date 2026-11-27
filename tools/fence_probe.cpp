// Target-side codegen probe for the deco view's seqlock fences
// (docs/REVIEW.md, Round A finding m-4, residual "are the fences load-bearing?").
//
// Built with the SHIPPING toolchain for env:esp32 and disassembled; the count
// that matters is MEMW — Xtensa's memory barrier — around the payload accesses:
//
//   XT=$(ls ~/.platformio/packages/*/bin/xtensa-esp32-elf-g++ | head -1)
//   OD=$(ls ~/.platformio/packages/*/bin/xtensa-esp32-elf-objdump | head -1)
//   $XT -c -O2 -mlongcalls tools/fence_probe.cpp -o /tmp/fence_probe.o
//   $OD -d /tmp/fence_probe.o | awk \
//     '/^[0-9a-f]+ </{fn=$2; m[fn]=0} /memw/{m[fn]++} END{for(f in m) print f, m[f]}'
//
// Measured 2026-11-27 (xtensa-esp32, the toolchain env:esp32 links with):
//   pub_fenced 3 memw, pub_plain (release fence removed) 2
//   view_fenced 4 memw, view_plain (acquire fence removed) 3
//   the shipped publishView 3, viewSnapshot 5
// i.e. each explicit fence costs one real barrier instruction on the target,
// while on x86-64 removing both changes nothing the host suite can see
// (env:native_o2, 437/437 with the fences commented out). Necessity is still
// unproven in either direction; redundancy is not.
//
// The three variants are the seqlock in src/deco/view.h exactly as shipped, the
// same pattern with the two explicit fences removed, and the same pattern with
// the sequence reads relaxed (the change the fences are written to make safe).

#include <atomic>
#include <cstdint>

struct Pay {           // plain members, exactly like deco::View's payload
    float f;
    double d;
    uint64_t u;
    uint32_t k;
};

static std::atomic<uint32_t> g_seq{0};
static Pay g_pay;

// --- as shipped: acq_rel RMWs plus the explicit fences ----------------------
__attribute__((noinline))
void pub_fenced(const Pay& v) {
    g_seq.fetch_add(1, std::memory_order_acq_rel);   // odd: writing
    g_pay = v;
    __atomic_thread_fence(__ATOMIC_RELEASE);
    g_seq.fetch_add(1, std::memory_order_acq_rel);   // even: published
}

__attribute__((noinline))
bool view_fenced(Pay& out) {
    for (int i = 0; i < 8; ++i) {
        const uint32_t s0 = g_seq.fetch_add(1, std::memory_order_acq_rel);
        if (s0 & 1u) continue;
        out = g_pay;
        __atomic_thread_fence(__ATOMIC_ACQUIRE);
        if (g_seq.load(std::memory_order_acquire) == s0) return true;
    }
    return false;
}

// --- the fences removed, RMW orderings unchanged ----------------------------
__attribute__((noinline))
void pub_plain(const Pay& v) {
    g_seq.fetch_add(1, std::memory_order_acq_rel);
    g_pay = v;
    g_seq.fetch_add(1, std::memory_order_acq_rel);
}

__attribute__((noinline))
bool view_plain(Pay& out) {
    for (int i = 0; i < 8; ++i) {
        const uint32_t s0 = g_seq.fetch_add(1, std::memory_order_acq_rel);
        if (s0 & 1u) continue;
        out = g_pay;
        if (g_seq.load(std::memory_order_acquire) == s0) return true;
    }
    return false;
}

// --- what a relaxed sequence read would cost -------------------------------
__attribute__((noinline))
bool view_relaxed(Pay& out) {
    for (int i = 0; i < 8; ++i) {
        const uint32_t s0 = g_seq.load(std::memory_order_relaxed);
        if (s0 & 1u) continue;
        out = g_pay;
        if (g_seq.load(std::memory_order_relaxed) == s0) return true;
    }
    return false;
}
