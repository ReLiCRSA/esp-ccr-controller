// ============================================================================
// session.cpp — see session.h.
//
// No heap allocation: the router and the link live in static storage, so a
// fragmented heap can never be the reason a diver's command goes unanswered.
// ============================================================================
#include "session.h"

#include <cstdio>
#include <new>

namespace comms {

namespace {

Session g_session;

} // namespace

bool Session::begin(ctrl::Loop& loop, settings::Registry& reg, bridge::Telemetry& tel,
                    bridge::EventRing& events, const UartLink::Hardware& hw, const Lock& lock) {
    lock_ = lock;
    // The schema text belongs to the comms subsystem, so it is rendered here
    // rather than by whatever boot sequence happens to exist: a call site in
    // main.cpp was forgettable, and a forgotten one means GET SCHEMA answers
    // with nothing to enforce. It is rendered now, at init, because this runs
    // before the small-stack tasks exist — the listing's %g formatting must
    // never run on the task that answers a handset (util/numfmt.h).
    primeSchema();

    router_ = new (routerMem_) CommandRouter(loop, reg, 0);
    // The `D,` answer and the event dive row read the CONTROL-published deco
    // generation (deco/view.h), never the engine: wiring the slot here is
    // what makes that rule structural for both transports (review N8).
    router_->setDecoView(&tel.view);
    // The link's event ring is the handset's: only pump() drains it. The web
    // transport shares everything else.
    link_ = new (linkMem_) UartLink(loop, reg, *router_, tel, events, hw);
    return link_ != nullptr;
}

void Session::handleLine(const char* line, uint32_t nowMs, UartLink::Sink sink, void* sinkCtx) {
    if (!link_) return;
    if (lock_.acquire) {
        lock_.acquire(lock_.ctx);
        lockWaits_++;
    }
    router_->setNowMs(nowMs);
    link_->handleLine(line, sink, sinkCtx);
    if (lock_.release) lock_.release(lock_.ctx);
}

void Session::pump(UartLink::Sink sink, void* sinkCtx, uint32_t nowMs, uint32_t telemetryMs) {
    if (!link_) return;
    if (lock_.acquire) {
        lock_.acquire(lock_.ctx);
        lockWaits_++;
    }
    router_->setNowMs(nowMs);
    link_->pump(sink, sinkCtx, nowMs, telemetryMs);
    if (lock_.release) lock_.release(lock_.ctx);
}

void Session::withLock(void (*fn)(void*), void* ctx) {
    if (!fn) return;
    if (lock_.acquire) {
        lock_.acquire(lock_.ctx);
        lockWaits_++;
    }
    fn(ctx);
    if (lock_.release) lock_.release(lock_.ctx);
}

size_t formatReplyMarker(char* out, size_t cap, uint16_t lines, bool dropped) {
    const int n = std::snprintf(out, cap, "#%u%s\n", static_cast<unsigned>(lines),
                                dropped ? " dropped" : "");
    if (n < 0) return 0;
    return (static_cast<size_t>(n) < cap) ? static_cast<size_t>(n) : cap - 1;
}

Session& session() { return g_session; }

} // namespace comms
