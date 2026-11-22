// ============================================================================
// session.h — THE command session. One CommandRouter, one UartLink, one
// dive-log owner, one lock; every transport (handset UART2, web WebSocket,
// a future button menu) calls handleLine() on this one object.
//
// That sharing is the point. Two transports with their own routers would each
// keep their own memory of what the diver asked for — the handset would turn
// the radio back on at dive end because *its* copy of the router did not know
// the diver had switched WiFi off from the phone first. Diver-visible state
// exists once, and it is reached through one code path, which is also the
// only place the pre-dive AUTO lockout and the dive-locked settings live.
//
// Concurrency: the handset task (core 1) and the async network task (core 0)
// can both execute a line. The lock serialises them; it is never held across
// anything that can block indefinitely (sinks enqueue, they do not wait) and
// nothing inside a hook calls back into handleLine().
//
// Hardware-free: the transport hooks and the lock are injected at init, so
// the whole session — including the cross-transport state sharing above — is
// unit-tested on the host (test/test_session/).
// ============================================================================
#pragma once

#include <cstdint>

#include "comms/cmd_router.h"
#include "comms/settings.h"
#include "comms/telemetry_bridge.h"
#include "comms/uart_link.h"
#include "control/loop.h"

namespace comms {

class Session {
public:
    // Owns placement-new'd members inside its own storage: copying a Session
    // would leave the copy pointing at the original's objects.
    Session() = default;
    Session(const Session&) = delete;
    Session& operator=(const Session&) = delete;

    /// Lock pair injected by the firmware (a FreeRTOS mutex on the board).
    /// Null on the host: the tests are single-threaded and must exercise the
    /// same code path, so a missing lock means "no synchronisation needed",
    /// never "different behaviour".
    struct Lock {
        void (*acquire)(void* ctx) = nullptr;
        void (*release)(void* ctx) = nullptr;
        void* ctx = nullptr;
    };

    bool begin(ctrl::Loop& loop, settings::Registry& reg, bridge::Telemetry& tel,
               bridge::EventRing& events, const UartLink::Hardware& hw, const Lock& lock);
    bool ready() const { return link_ != nullptr; }

    /// Executes one complete command line and answers through `sink`. The sink
    /// belongs to the calling transport (Serial2 print, WebSocket frame...) and
    /// runs on the caller's task, inside the lock. `nowMs` is the caller's
    /// reading of the chain's clock (millis() on the board): dive edges and the
    /// pre-dive sequence have to land on the same timeline the controller uses,
    /// so the transport that saw the command supplies it.
    void handleLine(const char* line, uint32_t nowMs, UartLink::Sink sink, void* sinkCtx);

    /// Periodic downlink (T,/D,/E,) for the handset. Only the handset task
    /// pumps: a second pumper would steal the event ring's E, lines, and a
    /// browser asks for its own state instead of being pushed it.
    void pump(UartLink::Sink sink, void* sinkCtx, uint32_t nowMs, uint32_t telemetryMs = 500);

    /// Radio bookkeeping shared by every transport (WIFI ON|OFF, dive edges).
    bool wifiEnabled() const { return router_ ? router_->wifiEnabled() : false; }
    /// Optional histogram source (see CommandRouter::setHist). Safe before or
    /// after begin(); a session without one answers GET HIST with ERR.
    void setHist(const cellhist::Hist* h) { if (router_) router_->setHist(h); }

    /// Runs `fn` with the session lock held, for work that touches the same
    /// hardware the commands touch (a dive-file export answering an HTTP
    /// request, say). Anything that must not race with DIVE START|END belongs
    /// in here; the handler then never needs to know a lock exists.
    void withLock(void (*fn)(void*), void* ctx);

    CommandRouter* router() { return router_; }
    UartLink* link() { return link_; }

    /// Lock statistics: a transport that blocks here is a transport that is
    /// waiting for someone else's command, and the count makes that visible
    /// on the serial console instead of looking like a hang.
    uint32_t lockWaits() const { return lockWaits_; }

private:
    // Static storage, per instance: no heap on the path that answers a diver,
    // and no shared scratch that a second Session could alias by accident.
    alignas(CommandRouter) unsigned char routerMem_[sizeof(CommandRouter)];
    alignas(UartLink) unsigned char linkMem_[sizeof(UartLink)];
    CommandRouter* router_ = nullptr;
    UartLink* link_ = nullptr;
    Lock lock_;
    uint32_t lockWaits_ = 0;
};

/// The one session, created by initSession() during setup(). Returns a
/// reference even before that (ready() is false) so callers never dereference
/// null: a session-less controller still dives.
Session& session();

/// Builds the session over the firmware's own chain, registry, bridge and
/// hardware hooks. Returns false if allocation failed.
bool initSession(ctrl::Loop& loop, settings::Registry& reg, bridge::Telemetry& tel,
                 bridge::EventRing& events);

/// The WebSocket-only end-of-answer marker. The protocol is line-based and a
/// command may answer with many lines (GET *, LOG EXPORT, DECO), so a browser
/// needs to know the answer ended; '#' can never begin a protocol line, so it
/// cannot collide with one. Kept here (not in web_server.cpp) because it is
/// wire-visible and host-tested.
size_t formatReplyMarker(char* out, size_t cap, uint16_t lines, bool dropped);

/// Dive-log accessors for ControlTask's 30 s grid. The file itself is opened
/// and closed by diver commands (DIVE START|END), so the control task only
/// hands rows over to the single owner. No-ops when nothing is recording.
bool diveLogRecording();
void diveLogAppendGrid(const logfmt::Snapshot& s);

/// Dive-file reads for the web pages. Serialized with the command path (same
/// lock): an export must not race DIVE START opening the next file. Returns
/// records emitted, or -1 if the slot is empty.
int diveLogExportCsv(int slot, void (*sink)(const char*, void*), void* ctx);
int diveLogList(int* slots, uint32_t* mtimes, int max);

} // namespace comms
