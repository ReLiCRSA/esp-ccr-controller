// ============================================================================
// uart_link.h — the handset protocol session (PLAN.md 8), hardware-free:
// the transport hands bytes in and takes lines out. The ESP32 task is a
// ten-line Serial2 shim over this class, so everything that decides what a
// handset sees — command handling, downlink cadence, the log export stream
// — is unit-tested on the host.
//
// The router answers; this class only formats periodic lines and applies
// the router's action bits to the hardware handles it is given (dive store,
// settings save, radio). A command whose action could not be applied still
// replied OK — so the action application is deliberately dead simple and
// every failure prints where a bench operator can see it.
// ============================================================================
#pragma once

#include <cstdint>

#include "comms/cmd_router.h"
#include "comms/settings.h"
#include "comms/telemetry_bridge.h"
#include "control/loop.h"
#include "log/divelog.h"

namespace comms {

class UartLink {
public:
    // lines out: one call per complete protocol line (newline included)
    using Sink = void (*)(const char* line, void* ctx);

    struct Hardware {
        // all optional: a host test passes none, the firmware passes real ones
        void* ctx = nullptr;
        void (*onWifi)(bool on, void* ctx) = nullptr;
        bool (*openDive)(uint32_t epoch, void* ctx) = nullptr;
        /// The controller's wall time at this instant (0 = no clock). DIVE START
        /// stamps the dive header with it — until 2026-11-24 the value
        /// passed here was a literal 0 in this file, which is why every dive on
        /// a board's flash carried epoch 0 and the FIFO could not order them.
        uint32_t (*wallEpoch)(void* ctx) = nullptr;
        bool (*setClock)(uint32_t epoch, void* ctx) = nullptr;
        void (*closeDive)(const logfmt::DiveMeta&, void* ctx) = nullptr;
        bridge::DiveTotals* totals = nullptr;  // in-force totals for closeDive
        void (*appendSnapNow)(void* ctx) = nullptr;    // event-driven snapshot
        // The event row itself, into the dive file. It exists because the
        // snapshot alongside an event shows the STATE at that moment, while the
        // event row is the machine's VERDICT: a dive file full of samples whose
        // alarm word went 0x0010 -> 0x0030 still does not say that the cell
        // proof failed, that the clamp engaged, or that the diver lifted it.
        void (*appendEventNow)(uint16_t id, uint16_t value, uint32_t tSec,
                               void* ctx) = nullptr;
        bool (*saveSettings)(void* ctx) = nullptr;
        void (*onCalAir)(void* ctx) = nullptr;
        int (*listDives)(int* slots, uint32_t* mtimes, uint32_t* seqs, int max,
                         void* ctx) = nullptr;
        int (*exportDive)(int slot, Sink sink, void* ctx) = nullptr;  // CSV lines
        // settings changed: apply the registry to the live control config and
        // (surface only) persist to NVS. The firmware wires applySettings +
        // Loop::setConfig here; the link never knows the config's shape.
        void (*onSettings)(void* ctx) = nullptr;
        // SAFE-mode ACK: the reply has already gone out when this runs, so
        // the transport may restart the controller from here (it does not
        // return). Nothing else is attempted after it.
        void (*onSafeAck)(void* ctx) = nullptr;
    };


    UartLink(ctrl::Loop& loop, settings::Registry& reg, CommandRouter& router,
             const bridge::Telemetry& tel, bridge::EventRing& events,
             const Hardware& hw)
        : loop_(loop), reg_(reg), router_(router), tel_(tel), events_(events),
          hw_(hw) {}

    /// A full command line arrived (no trailing newline needed).
    void handleLine(const char* line, Sink sink, void* ctx);

    /// Periodic traffic: T, at telemetryHz, D, at its quarter rate, plus any
    /// queued events. Call as often as you like; it self-times from `nowMs`.
    void pump(Sink sink, void* ctx, uint32_t nowMs, uint32_t telemetryMs = 500);

    uint32_t linesOut() const { return linesOut_; }

private:
    void applyActions(uint32_t actions, Sink sink, void* ctx);
    void emitT(Sink sink, void* ctx);
    void emitD(Sink sink, void* ctx);
    void drainEvents(Sink sink, void* ctx);
    void streamLogList(Sink sink, void* ctx);
    void streamLogExport(int slot, Sink sink, void* ctx);

    ctrl::Loop& loop_;
    settings::Registry& reg_;
    CommandRouter& router_;
    const bridge::Telemetry& tel_;
    bridge::EventRing& events_;
    // BY VALUE: transports pass their Hardware as a temporary; holding a
    // reference would outlive it (the constructor's bound temporary dies at
    // construction — a dangling hw_ is exactly the kind of bug that only
    // shows up as a hook that "isn't called").
    Hardware hw_;

    uint32_t lastTMs_ = 0, lastDMs_ = 0;
    uint32_t lastPulseBase_ = 0;
    uint32_t linesOut_ = 0;

    // This transport's reply buffer and its line scratch, deliberately members
    // rather than locals. A Session is a file-scope global, so these live in
    // static RAM; the alternative (the pre-2026-11-23 shape) put a 1024-byte
    // Response copy and a 1024-byte format temporary on whatever task happened
    // to ask, which is what overflowed the uart2 task's stack. They are used by
    // one command at a time — the session mutex serialises handset and web —
    // and never nest, so one buffer serves the whole transport.
    CommandRouter::Response resp_;
    char lineBuf_[CommandRouter::MAX_REPLY];
    bool edgePending_ = false;
};

} // namespace comms
