// ============================================================================
// uart_link.cpp — see uart_link.h.
// ============================================================================
#include "uart_link.h"

#include <cstdio>
#include <cstring>

namespace comms {

namespace {
void emit(UartLink::Sink sink, void* ctx, char* line, size_t n) {
    if (n) {
        if (n < 512) line[n] = '\0';   // formatters return the NEEDED length
        sink(line, ctx);               // when the buffer was too small
    }
}
} // namespace

void UartLink::handleLine(const char* line, Sink sink, void* ctx) {
    // resp_ is a member, not a local: exec() fills it in place and nothing here
    // copies 1 KB of reply through the task's stack.
    router_.exec(line, resp_);
    if (resp_.len) {
        if (resp_.len < sizeof(resp_.text)) resp_.text[resp_.len] = '\0';
        sink(resp_.text, ctx);
        linesOut_++;
    }
    applyActions(resp_.actions, sink, ctx);
}

void UartLink::applyActions(uint32_t actions, Sink sink, void* ctx) {
    char buf[96];
    if (actions & CommandRouter::ACT_SETTINGS) {
        if (hw_.onSettings) hw_.onSettings(hw_.ctx);
        // Persist between dives only: a mid-dive save is flash work on the
        // task that answers the diver, and the values are already live in RAM.
        if (hw_.saveSettings && !router_.diving()) {
            if (!hw_.saveSettings(hw_.ctx)) {
                const size_t n = proto::formatErr("settings not saved", buf, sizeof(buf));
                emit(sink, ctx, buf, n);
            }
        }
    }
    if (actions & CommandRouter::ACT_WIFI)
        if (hw_.onWifi) hw_.onWifi(router_.wifiEnabled(), hw_.ctx);
    if (actions & CommandRouter::ACT_CAL_AIR)
        if (hw_.onCalAir) hw_.onCalAir(hw_.ctx);
    if (actions & CommandRouter::ACT_DIVE_START) {
        // The log is opened by whoever holds the file. If it fails, say so
        // out loud on the same socket the dive just started on: an unlogged
        // dive is acceptable, an *unannounced* unlogged dive is not.
        const uint32_t wall = hw_.wallEpoch ? hw_.wallEpoch(hw_.ctx) : 0u;
        if (hw_.openDive && !hw_.openDive(wall, hw_.ctx)) {
            const size_t n = proto::formatErr("LOG FAILED: dive unrecorded", buf, sizeof(buf));
            emit(sink, ctx, buf, n);
        }
    }
    if (actions & CommandRouter::ACT_DIVE_END) {
        if (hw_.closeDive)
            hw_.closeDive(hw_.totals ? hw_.totals->toMeta() : logfmt::DiveMeta{}, hw_.ctx);
    }
    if (actions & CommandRouter::ACT_CLOCK_SET) {
        // Same rule as the dive log: if the clock did not get set, say so on the
        // socket the command arrived on. A client that was told OK and then
        // finds the epoch is still 0 has learned nothing about what failed.
        if (!hw_.setClock || !hw_.setClock(router_.clockEpochPending(), hw_.ctx)) {
            const size_t n = proto::formatErr("clock not set", buf, sizeof(buf));
            emit(sink, ctx, buf, n);
        }
    }
    if (actions & CommandRouter::ACT_LOG_LIST) streamLogList(sink, ctx);
    if (actions & CommandRouter::ACT_LOG_EXPORT)
        streamLogExport(router_.logExportIndex(), sink, ctx);

    // Absolutely last: the reply and any streamed listing are already on the
    // wire, and this one does not return — the transport restarts the
    // controller from here (SAFE mode's acknowledged way out).
    if (actions & CommandRouter::ACT_SAFE_ACK)
        if (hw_.onSafeAck) hw_.onSafeAck(hw_.ctx);
}

void UartLink::emitT(Sink sink, void* ctx) {
    proto::Telemetry t;
    t.depthM = tel_.depthM;
    t.votedPpo2 = tel_.votedBar;
    for (int i = 0; i < 3; i++) t.cells[i] = tel_.cellBar[i];
    t.setpoint = tel_.setpointBar;
    t.battV = tel_.vbat;
    t.mode = tel_.diveActive ? (tel_.autoMode ? 1 : 2) : 0;
    uint16_t a = tel_.alarmBits;
    uint8_t f = 0;
    if (a & (1u << alarms::BIT_LOW_PPO2))   f |= 1 << 0;
    if (a & (1u << alarms::BIT_HIGH_PPO2))  f |= 1 << 1;
    if (a & (1u << alarms::BIT_CELL_DRIFT)) f |= 1 << 2;
    if (a & ((1u << alarms::BIT_SENSOR_FAIL) | (1u << alarms::BIT_SENSOR_STALE) |
             (1u << alarms::BIT_STUCK_OPEN) | (1u << alarms::BIT_HYPOXIA)))
        f |= 1 << 3;
    if (a & (1u << alarms::BIT_BATTERY))    f |= 1 << 4;
    if (a & (1u << alarms::BIT_SP_CLAMP))   f |= 1 << 5;
    if (tel_.cpState == static_cast<uint8_t>(cellproof::State::FAIL)) f |= 1 << 6;
    t.flags = f;
    // SAFE is visible on the wire before the diver tries anything: the router
    // knows the machine is locked out, and the snapshot published by a control
    // chain that is not running cannot say it for itself.
    if (router_.safeLockout()) t.mode = 3;

    const size_t n = proto::formatTelemetry(t, lineBuf_, sizeof(lineBuf_));
    emit(sink, ctx, lineBuf_, n);
    if (n) linesOut_++;
}

void UartLink::emitD(Sink sink, void* ctx) {
    // This one runs unprompted twice a second on the uart2 task, 24/7 — the
    // 1024-byte temporary here was part of the same overflow budget.
    const size_t n = proto::formatDeco(
        CommandRouter::decoInfoOf(loop_, tel_.haveVote), lineBuf_, sizeof(lineBuf_));
    emit(sink, ctx, lineBuf_, n);
    if (n) linesOut_++;
}

void UartLink::drainEvents(Sink sink, void* ctx) {
    bridge::Event e;
    char buf[32];
    int sent = 0;
    while (events_.pop(e) && sent < 8) {   // bounded: never flood one pump
        const size_t n = proto::formatEvent(e.tSec, e.id, e.value, buf, sizeof(buf));
        emit(sink, ctx, buf, n);
        if (n) linesOut_++;
        // Every protocol event is also a moment of interest for the dive
        // log (PLAN 10): grab an event-driven snapshot alongside it.
        if (hw_.appendSnapNow) hw_.appendSnapNow(hw_.ctx);
        sent++;
    }
}

void UartLink::pump(Sink sink, void* ctx, uint32_t nowMs, uint32_t telemetryMs) {
    const bool first = lastTMs_ == 0 && lastDMs_ == 0;
    if (first || nowMs - lastTMs_ >= telemetryMs) {   // first pump: answer now,
        lastTMs_ = nowMs;                             // a handset waits for T,
        emitT(sink, ctx);
    }
    if (first || nowMs - lastDMs_ >= telemetryMs * 4) {
        lastDMs_ = nowMs;
        emitD(sink, ctx);
    }
    drainEvents(sink, ctx);
}

void UartLink::streamLogList(Sink sink, void* ctx) {
    if (!hw_.listDives) {
        char buf[64];
        const size_t n = proto::formatErr("no log store", buf, sizeof(buf));
        emit(sink, ctx, buf, n);
        return;
    }
    int slots[10];
    uint32_t epochs[10], seqs[10];
    const int n = hw_.listDives(slots, epochs, seqs, 10, hw_.ctx);
    char buf[48];
    for (int i = 0; i < n; i++) {
        // `L,<slot>,<epoch>,<seq>`: the fourth field is appended, so a v1 client
        // reading three fields is unchanged. It exists because on a board with no
        // clock every dive carries epoch 0, and a handset or browser then has ten
        // dives and no way to say which is the newest. The sequence is the order
        // dives were written, and it is the same number the boot banner reports
        // as a range.
        const int m = std::snprintf(buf, sizeof(buf), "L,%d,%u,%u\n", slots[i],
                                    (unsigned)epochs[i], (unsigned)seqs[i]);
        emit(sink, ctx, buf, static_cast<size_t>(m));
        linesOut_++;
    }
}

void UartLink::streamLogExport(int slot, Sink sink, void* ctx) {
    if (!hw_.exportDive) {
        char buf[64];
        const size_t n = proto::formatErr("no log store", buf, sizeof(buf));
        emit(sink, ctx, buf, n);
        return;
    }
    // The sink belongs to whoever asked for the export (a handset print, a
    // browser socket), so the export must hand the *caller's* sink context
    // back to it — hw_.ctx is the firmware's hook object, and handing that to
    // a transport sink is a wild-pointer cast waiting for the first LOG
    // EXPORT over WiFi.
    if (hw_.exportDive(slot, sink, ctx) < 0) {
        char buf[64];
        const size_t n = proto::formatErr("no such dive", buf, sizeof(buf));
        emit(sink, ctx, buf, n);
    }
}

} // namespace comms
