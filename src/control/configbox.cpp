// ============================================================================
// configbox.cpp — see configbox.h. One mutex, one struct, one writer.
// ============================================================================
#include "configbox.h"

#include <mutex>

namespace ctrl {
namespace {
std::mutex g_mux;
Config g_current;          // the published generation
uint32_t g_generation = 0;  // protected by g_mux
}  // namespace

void configPublish(const Config& cfg) {
    // The lock is held for a struct copy and nothing else. applySettings runs
    // on the caller's own copy before it gets here, so the registry walk and the
    // ordering clamps are never inside the critical section — a control cycle
    // waiting on this lock waits on a copy, not on a settings pass.
    std::lock_guard<std::mutex> g(g_mux);
    g_current = cfg;
    ++g_generation;
}

bool configRead(Config& out) {
    std::lock_guard<std::mutex> g(g_mux);
    if (g_generation == 0) return false;   // nothing published: leave `out` alone
    out = g_current;
    return true;
}

uint32_t configGeneration() {
    std::lock_guard<std::mutex> g(g_mux);
    return g_generation;
}
}  // namespace ctrl
