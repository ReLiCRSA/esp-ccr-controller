// ============================================================================
// session_wiring.cpp — the ESP32 half of the command session: the hardware
// hooks (dive file, NVS, radio, settings application, calibration requests),
// the session lock, and the dive-log owner. Compiled for the target only; the
// logic it drives (router, applier, codec) is host-tested.
//
// Two rules here:
//   * One dive file, one owner. The file is opened/closed by diver commands;
//     ControlTask hands over the 30 s grid rows and the handset link hands
//     over event-driven rows. Both appends take the same log mutex, because
//     two tasks writing one FILE* is exactly the kind of thing that corrupts
//     a life-support log quietly.
//   * Nothing in this file may block on anything a diver is waiting for. The
//     session lock is held only while a command executes; every hook is a
//     bounded, non-blocking action (flag set, record appended, radio moved).
// ============================================================================
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include "comms/nvs_store.h"
#include "comms/session.h"
#include "comms/settings_apply.h"
#include "comms/web_server.h"
#include "config/defaults.h"
#include "log/dive_store.h"

// The chain, the registry and the bridge belong to main.cpp; the session must
// drive the very same objects the sensors and the HUD use.
extern ctrl::Config g_ctrlCfg;
extern ctrl::Loop* g_loop;
extern settings::Registry g_settings;
extern bridge::Telemetry g_bridgeTel;
extern bridge::EventRing g_bridgeEvents;
extern bridge::DiveTotals g_bridgeTotals;
extern bool g_calAirRequest;
extern bool g_calDepthRequest;

namespace comms {

namespace {

DiveStore g_diveStore;
SemaphoreHandle_t g_sessionMtx = nullptr;
SemaphoreHandle_t g_logMtx = nullptr;
uint32_t g_diveStartMs = 0;

void lockAcquire(void*) {
    if (g_sessionMtx) xSemaphoreTake(g_sessionMtx, portMAX_DELAY);
}
void lockRelease(void*) {
    if (g_sessionMtx) xSemaphoreGive(g_sessionMtx);
}

// ---- dive file ------------------------------------------------------------

bool hookOpenDive(uint32_t epoch, void*) {
    if (!g_logMtx) return false;
    xSemaphoreTake(g_logMtx, portMAX_DELAY);
    g_diveStartMs = millis();
    const bool ok = g_diveStore.begin() && g_diveStore.startDive(epoch);
    xSemaphoreGive(g_logMtx);
    return ok;
}

void hookCloseDive(const logfmt::DiveMeta& meta, void*) {
    if (!g_logMtx) return;
    xSemaphoreTake(g_logMtx, portMAX_DELAY);
    g_diveStore.endDive(meta);
    xSemaphoreGive(g_logMtx);
}

/// Event-driven snapshot (PLAN 10): the moment an alarm/mode/cellproof edge
/// happened is worth a row of its own, not just a 30 s grid row nearby.
void hookAppendSnap(void*) {
    if (!g_loop || !g_logMtx) return;
    logfmt::Snapshot s{};
    // Built from the published bridge scalars, never from the live chain:
    // the control task owns that object.
    s.tSec = (millis() - g_diveStartMs) / 1000;
    s.depthCm = (int32_t)(g_bridgeTel.depthM * 100.0f);
    s.votedX1000 = (int16_t)(g_bridgeTel.votedBar * 1000.0f);
    for (int i = 0; i < 3; i++)
        s.cellX1000[i] = (int16_t)(g_bridgeTel.cellBar[i] * 1000.0f);
    s.setpointX1000 = (int16_t)(g_bridgeTel.setpointBar * 1000.0f);
    s.mode = g_bridgeTel.diveActive ? (g_bridgeTel.autoMode ? 1 : 2) : 0;
    s.alarmBits = g_bridgeTel.alarmBits;
    s.cnsPct = g_bridgeTotals.cnsPeakPct;
    s.pulses = (uint16_t)g_bridgeTel.pulsesWindow;
    s.battX1000 = (uint16_t)(g_bridgeTel.vbat * 1000.0f);

    xSemaphoreTake(g_logMtx, portMAX_DELAY);
    if (g_diveStore.recording()) g_diveStore.append(s);
    xSemaphoreGive(g_logMtx);
}

// ---- radio ----------------------------------------------------------------

void hookWifi(bool on, void*) { web::setRadio(on); }

// ---- settings -------------------------------------------------------------

bool hookSave(void*) { return nvs::Store::save(g_settings); }

void hookSettings(void*) {
    // The registry is the truth; this is the one place it becomes control
    // configuration. Loop::setConfig re-tunes the engines in place, so a SET
    // never looks like a restart to the dive in progress.
    applySettings(g_settings, g_ctrlCfg);
    if (g_loop) g_loop->setConfig(g_ctrlCfg);
}

void hookCalAir(void*) { g_calAirRequest = true; }
void hookCalDepth(void*) { g_calDepthRequest = true; }

// ---- log reads (handset LOG LIST / LOG EXPORT) ----------------------------

int hookListDives(int* slots, uint32_t* mt, int max, void*) {
    return g_diveStore.list(slots, mt, max);
}

int hookExport(int slot, UartLink::Sink sink, void* ctx) {
    return g_diveStore.exportCsv(slot, sink, ctx);
}

UartLink::Hardware makeHardware() {
    UartLink::Hardware hw;
    hw.totals = &g_bridgeTotals;
    hw.onWifi = hookWifi;
    hw.openDive = hookOpenDive;
    hw.closeDive = hookCloseDive;
    hw.appendSnapNow = hookAppendSnap;
    hw.saveSettings = hookSave;
    hw.onSettings = hookSettings;
    hw.onCalAir = hookCalAir;
    hw.onCalDepth = hookCalDepth;
    hw.listDives = hookListDives;
    hw.exportDive = hookExport;
    return hw;
}

// ---- locked dive-file reads ----------------------------------------------

struct CsvSinkCtx {
    void (*sink)(const char*, void*);
    void* ctx;
    int slot;
    int rc;
};

void exportUnderLock(void* vctx) {
    CsvSinkCtx& o = *static_cast<CsvSinkCtx*>(vctx);
    o.rc = g_diveStore.exportCsv(o.slot, o.sink, o.ctx);
}

struct ListCtx {
    int* slots;
    uint32_t* mtimes;
    int max;
    int n;
};

void listUnderLock(void* vctx) {
    ListCtx& o = *static_cast<ListCtx*>(vctx);
    o.n = g_diveStore.list(o.slots, o.mtimes, o.max);
}

} // namespace

bool initSession(ctrl::Loop& loop, settings::Registry& reg, bridge::Telemetry& tel,
                 bridge::EventRing& events) {
    if (g_sessionMtx == nullptr)
        g_sessionMtx = xSemaphoreCreateMutex();
    if (g_logMtx == nullptr) g_logMtx = xSemaphoreCreateMutex();
    if (g_sessionMtx == nullptr || g_logMtx == nullptr) return false;

    Session::Lock lock;
    lock.acquire = lockAcquire;
    lock.release = lockRelease;
    return session().begin(loop, reg, tel, events, makeHardware(), lock);
}

bool diveLogRecording() {
    if (!g_logMtx) return false;
    xSemaphoreTake(g_logMtx, portMAX_DELAY);
    const bool r = g_diveStore.recording();
    xSemaphoreGive(g_logMtx);
    return r;
}

void diveLogAppendGrid(const logfmt::Snapshot& s) {
    if (!g_logMtx) return;
    xSemaphoreTake(g_logMtx, portMAX_DELAY);
    g_diveStore.append(s);
    xSemaphoreGive(g_logMtx);
}

int diveLogExportCsv(int slot, void (*sink)(const char*, void*), void* ctx) {
    CsvSinkCtx o{sink, ctx, slot, -1};
    session().withLock(exportUnderLock, &o);
    return o.rc;
}

int diveLogList(int* slots, uint32_t* mtimes, int max) {
    ListCtx o{slots, mtimes, max, 0};
    session().withLock(listUnderLock, &o);
    return o.n;
}

} // namespace comms
