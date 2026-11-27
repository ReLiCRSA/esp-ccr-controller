// ============================================================================
// uart_link.cpp — see uart_link.h.
// ============================================================================
#include "uart_link.h"

#include <cstdio>
#include <cstring>

#include "util/wdtfeed.h"

namespace comms {

namespace {
void emit(UartLink::Sink sink, void* ctx, char* line, size_t n) {
    if (n) {
        if (n < 512) line[n] = '\0';   // formatters return the NEEDED length
        sink(line, ctx);               // when the buffer was too small
    }
}

// ---- the export's row sink: the same bytes, counted (review M-1) -----------
// A handset `LOG EXPORT n` has no size: the store streams the whole dive file
// as CSV rows inside ONE handleLine() call, and there is no dive lock on the
// command (whether there should be is a separate decision, not this one). On
// the handset the sink below that stream is Serial2.print at 115200 — ~87 us a
// byte, no yield inside it — and the task running it is registered with the
// task watchdog, which it feeds once per pump (uart2_task.cpp). A 6-12 h dive
// therefore puts thousands of rows between two feeds, and a registered task
// that blows that budget on this build does not warn, it panics (main.cpp's
// esp_task_wdt_init(3, true) is the timeout this task is checked against — the
// SDK this env builds does not boot the watchdog itself — with panic enabled,
// and a TASK_WDT reset reason forces SAFE at the next boot). So the answer to a
// diver's read-only question can
// reset his own controller mid-dive: an export that takes a long time to write
// is indistinguishable, to the watchdog, from a task that stopped.
//
// So the link counts the rows and feeds the watchdog from the task that is
// doing the work, every util::kWdtFeedEvery rows (util/wdtfeed.h carries the
// sizing). What is NOT changed: one sink call per row, the row's own bytes, the
// caller's sink and ctx, and no feed until a row has actually been handed to
// that sink — so the feed is evidence of progress, not a cover for its absence.
// A UART that has genuinely wedged blocks inside the sink and never reaches the
// feed, and the watchdog still does its job.
struct ExportSink {
    UartLink::Sink sink;
    void* ctx;
    unsigned rows;      // rows already handed over
};

void exportRowSink(const char* line, void* v) {
    ExportSink& e = *static_cast<ExportSink*>(v);
    e.sink(line, e.ctx);                     // the wire first, always
    if (util::wdtFeedDue(++e.rows)) util::wdtFeed();
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
        // wifiRadioWanted(), NOT wifiEnabled(): the intent may be ON while a
        // holder (dive/depth/idle) legitimately keeps the physical radio down.
        // Passing the intent here is how a transport ends up fighting the
        // policy one tick later.
        if (hw_.onWifi) hw_.onWifi(router_.wifiRadioWanted(), hw_.ctx);
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
    t.votedPpo2 = tel_.shownPpo2;
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
    // CNS overload gets its OWN bit (7), added 2026-11-27 rather than folded
    // into bit 3. It used to be dropped here with a comment calling the bits
    // 6..9 aggregate "sensor/fuel failures", which is true of stuck-open,
    // sensor stale and hypoxia and false of CNS: a diver over his CNS limit is
    // not reading a sensor fault, and the handset that only aggregates cannot
    // tell "your loop is not telling you things" from "you have used up your
    // oxygen budget" — two different actions. Append-only: bits 0..6 keep the
    // meanings the protocol publishes (§4's table), bit 7 is newly named, and
    // nothing moved; a client that ignores the top bit is still correct, exactly
    // as one that stops at FLAGS is correct about the appended battery field.
    // The controller's own alarm word is untouched — this is a projection, and
    // the ids it publishes as E,*,1/2 are the same 10 bits as before.
    if (a & (1u << alarms::BIT_CNS_HIGH))   f |= 1 << 7;
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
    // 1024-byte temporary here was part of the same overflow budget. From the
    // published deco generation (deco/view.h): this task never touches the
    // engine ControlTask integrates. If nothing is published (SAFE boot: no
    // control task), the line is not printed at all — no stale plan on a wire.
    deco::View pic{};
    if (!deco::viewSnapshot(tel_.view, pic)) return;
    const size_t n = proto::formatDeco(CommandRouter::decoInfoOf(pic),
                                       lineBuf_, sizeof(lineBuf_));
    emit(sink, ctx, lineBuf_, n);
    if (n) linesOut_++;
}

void UartLink::drainEvents(Sink sink, void* ctx, uint32_t nowMs) {
    bridge::Event e;
    char buf[32];
    int sent = 0;
    bridge::EdgeSnap snap{};
    bool haveSnap = false;
    while (events_.pop(e) && sent < 8) {   // bounded: never flood one pump
        // The batch's instant, taken before the FIRST row of the batch is
        // written (and before its `E,` row, so a snapshot row's stamp can never
        // be later than the write that files the verdict beside it). Capture
        // once, not per event: eight edges drained together used to be eight
        // rows read from eight control cycles, and a dive file that shows the
        // alarm word moving, the clamp engaging and the diver overriding it is
        // exactly the burst where the numbers underneath them were moving too.
        if (!haveSnap) {
            bridge::captureEdge(tel_, hw_.totals, nowMs, snap);
            haveSnap = true;
        }
        const size_t n = proto::formatEvent(e.tSec, e.id, e.value, buf, sizeof(buf));
        emit(sink, ctx, buf, n);
        if (n) linesOut_++;
        // The same publication feeds both records, so the wire and the dive
        // file cannot drift: the EVENT row first (what the machine decided),
        // then the event-driven SNAPSHOT (what the machine looked like when it
        // decided it, PLAN 10). Before 2026-11-24 only the snapshot was
        // written: a dive downloaded after the fact showed the alarm word move
        // and the setpoint get capped, but never the FAIL, the clamp or the
        // diver's override - the effects survived, the verdicts did not.
        if (n && hw_.appendEventNow) hw_.appendEventNow(e.id, e.value, e.tSec, hw_.ctx);
        if (hw_.appendSnapNow) hw_.appendSnapNow(snap, hw_.ctx);
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
    drainEvents(sink, ctx, nowMs);
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
    // EXPORT over WiFi. The row counter rides beside them rather than in a
    // global because one export at a time is the rule the session lock already
    // enforces, and a stack frame that dies with the call cannot outlive it.
    ExportSink ex{sink, ctx, 0u};
    if (hw_.exportDive(slot, exportRowSink, &ex) < 0) {
        char buf[64];
        const size_t n = proto::formatErr("no such dive", buf, sizeof(buf));
        emit(sink, ctx, buf, n);
    }
}

} // namespace comms
