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
    CommandRouter::Response r = router_.exec(line);
    if (r.len) {
        if (r.len < sizeof(r.text)) r.text[r.len] = '\0';
        sink(r.text, ctx);
        linesOut_++;
    }
    applyActions(r.actions, sink, ctx);
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
    if (actions & CommandRouter::ACT_CAL_DEPTH)
        if (hw_.onCalDepth) hw_.onCalDepth(hw_.ctx);
    if (actions & CommandRouter::ACT_DIVE_START) {
        // The log is opened by whoever holds the file. If it fails, say so
        // out loud on the same socket the dive just started on: an unlogged
        // dive is acceptable, an *unannounced* unlogged dive is not.
        if (hw_.openDive && !hw_.openDive(0, hw_.ctx)) {
            const size_t n = proto::formatErr("LOG FAILED: dive unrecorded", buf, sizeof(buf));
            emit(sink, ctx, buf, n);
        }
    }
    if (actions & CommandRouter::ACT_DIVE_END) {
        if (hw_.closeDive)
            hw_.closeDive(hw_.totals ? hw_.totals->toMeta() : logfmt::DiveMeta{}, hw_.ctx);
    }
    if (actions & CommandRouter::ACT_LOG_LIST) streamLogList(sink, ctx);
    if (actions & CommandRouter::ACT_LOG_EXPORT)
        streamLogExport(router_.logExportIndex(), sink, ctx);
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

    char buf[128];
    const size_t n = proto::formatTelemetry(t, buf, sizeof(buf));
    emit(sink, ctx, buf, n);
    if (n) linesOut_++;
}

void UartLink::emitD(Sink sink, void* ctx) {
    char buf[CommandRouter::MAX_REPLY];
    const size_t n = proto::formatDeco(
        CommandRouter::decoInfoOf(loop_, tel_.haveVote), buf, sizeof(buf));
    emit(sink, ctx, buf, n);
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
    uint32_t epochs[10];
    const int n = hw_.listDives(slots, epochs, 10, hw_.ctx);
    char buf[48];
    for (int i = 0; i < n; i++) {
        const int m = std::snprintf(buf, sizeof(buf), "L,%d,%u\n", slots[i],
                                    (unsigned)epochs[i]);
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
