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
#include <esp_sleep.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include "comms/nvs_store.h"
#include "comms/session.h"
#include "comms/settings_apply.h"
#include "comms/wallclock.h"
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

// ---- wall clock: the state, and the memory that carries it across a reset --
wallclock::State g_clock;

/// RTC no-init memory: kept across the resets the firmware performs on itself
/// (panic, watchdog, SAFE-mode ACK), and gone after an EN cycle or a battery
/// pull — measured 2026-11-24: the bench's EN pulse boots as `rst:0x1
/// (POWERON_RESET)` and the carry is lost, exactly like pulling the pack. What
/// survives and what does not is stated at boot either way. After a power-down
/// that region holds whatever the silicon left there, which can be a number
/// that looks exactly like a plausible epoch — that is why the magic travels
/// with the value: a garbage word believed as a clock would date every dive in
/// the log wrong, silently, and nothing downstream would ever question it.
RTC_NOINIT_ATTR struct ClockCarry {
    uint32_t magic;
    uint32_t epoch;
} g_clockCarry;


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
    if (ok && epoch == 0u) {
        // The dive is recorded either way — refusing to log a dive because
        // nobody set a clock would cost the diver the dive data, which is the
        // opposite of what a dive computer is for. What must not be possible is
        // reading the log later and not knowing it has no date, so the dive says
        // so itself: an event row inside the file, and `clock-unset` in the CSV
        // export where a date would otherwise be printed.
        logfmt::Event ev{};
        ev.tSec = 0;
        ev.id = logfmt::EVT_CLOCK_UNSET;
        ev.value = 0;
        g_diveStore.append(ev);
    }
    xSemaphoreGive(g_logMtx);
    return ok;
}

void hookCloseDive(const logfmt::DiveMeta& meta, void*) {
    if (!g_logMtx) return;
    xSemaphoreTake(g_logMtx, portMAX_DELAY);
    g_diveStore.endDive(meta);
    xSemaphoreGive(g_logMtx);
}

uint32_t hookWallEpoch(void*) { return wallclock::epoch(g_clock, millis()); }

bool hookSetClock(uint32_t epoch, void*) {
    if (!wallclock::set(g_clock, epoch, millis())) return false;
    g_clockCarry.epoch = epoch;
    g_clockCarry.magic = wallclock::CARRY_MAGIC;
    return true;
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
    // configuration. postConfig queues the re-tune for the next control tick
    // (engines re-tune in place, never a restart), so a SET from the handset
    // or the browser never mutates the chain while ControlTask is mid-cycle.
    applySettings(g_settings, g_ctrlCfg);
    if (g_loop) g_loop->postConfig(g_ctrlCfg);
}

void hookCalAir(void*) { g_calAirRequest = true; }
void hookCalDepth(void*) { g_calDepthRequest = true; }

// ---- log reads (handset LOG LIST / LOG EXPORT) ----------------------------

int hookListDives(int* slots, uint32_t* mt, uint32_t* seq, int max, void*) {
    return g_diveStore.list(slots, mt, seq, max);
}

int hookExport(int slot, UartLink::Sink sink, void* ctx) {
    return g_diveStore.exportCsv(slot, sink, ctx);
}

// SAFE mode's acknowledged exit, chosen by the diver at the handset rather than
// by a power connector that has to come out of a housing. By the time this runs
// the reply is already in the UART's TX ring; the pause is only so those bytes
// reach the wire before the chip goes down.
//
// esp_restart() is a software reset: the supervisor's forceSafe list watches for
// PANIC / watchdog / brownout, not SW, and the fault record was cleared at the
// last boot — so the next boot comes up normally, with the settings from NVS.
// Deliberately not an exit that resumes control in place: see
// CommandRouter::setSafeLockout for why.
void hookSafeAck(void*) {
    Serial.println("** SAFE acknowledged over the link: restarting clean");
    delay(60);
    esp_restart();
}

UartLink::Hardware makeHardware() {
    UartLink::Hardware hw;
    hw.totals = &g_bridgeTotals;
    hw.onWifi = hookWifi;
    hw.openDive = hookOpenDive;
    hw.wallEpoch = hookWallEpoch;
    hw.setClock = hookSetClock;
    hw.closeDive = hookCloseDive;
    hw.appendSnapNow = hookAppendSnap;
    hw.saveSettings = hookSave;
    hw.onSettings = hookSettings;
    hw.onCalAir = hookCalAir;
    hw.onCalDepth = hookCalDepth;
    hw.onSafeAck = hookSafeAck;
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
    uint32_t* seqs;
    int max;
    int n;
};

void listUnderLock(void* vctx) {
    ListCtx& o = *static_cast<ListCtx*>(vctx);
    o.n = g_diveStore.list(o.slots, o.mtimes, o.seqs, o.max);
}

struct SeqRangeCtx {
    uint32_t lo;
    uint32_t hi;
    bool any;
};

void seqRangeUnderLock(void* vctx) {
    SeqRangeCtx& o = *static_cast<SeqRangeCtx*>(vctx);
    o.any = g_diveStore.seqRange(o.lo, o.hi);
}

} // namespace

// ---- wall clock: what the boot code and the control cycle call -------------
// These are outside the anonymous namespace on purpose: `namespace comms` alone
// is not enough to be reachable from main.cpp, and the first version of this put
// them in the anonymous block, where the link step caught it.

void clockBootRestore() {
    const uint32_t e = wallclock::carryPick(g_clockCarry.magic, g_clockCarry.epoch);
    if (e == 0u) return;              // nothing trustworthy: boot with no clock
    g_clock.epochSet = e;
    g_clock.setAtMs = millis();
    g_clock.source = wallclock::CARRIED;
}

void clockTick() {
    if (g_clock.epochSet == 0u) return;
    const uint32_t now = wallclock::epoch(g_clock, millis());
    if (now == 0u) return;
    // A RAM write, not flash: the RTC no-init region is live memory that the
    // reset leaves alone. Mirroring every control cycle is what makes a carried
    // clock correct — without it the value freezes at the instant it was
    // set, and a reset eight hours later resumes eight hours in the past.
    g_clockCarry.epoch = now;
    g_clockCarry.magic = wallclock::CARRY_MAGIC;
}

void clockBootText(char* out, size_t cap) {
    const uint32_t e = wallclock::epoch(g_clock, millis());
    if (e == 0u) {
        snprintf(out, cap, "unset: nothing has set the clock since power-up, "
                           "dives will carry no date");
        return;
    }
    char iso[24];
    logfmt::isoUtc(iso, sizeof(iso), e);
    if (g_clock.source == wallclock::CARRIED)
        snprintf(out, cap, "%s (carried across a reset, not verified since)", iso);
    else
        snprintf(out, cap, "%s (set %lu s ago)", iso,
                 (unsigned long)wallclock::ageS(g_clock, millis()));
}

bool initSession(ctrl::Loop& loop, settings::Registry& reg, bridge::Telemetry& tel,
                 bridge::EventRing& events) {
    if (g_sessionMtx == nullptr)
        g_sessionMtx = xSemaphoreCreateMutex();
    if (g_logMtx == nullptr) g_logMtx = xSemaphoreCreateMutex();
    if (g_sessionMtx == nullptr || g_logMtx == nullptr) return false;

    Session::Lock lock;
    lock.acquire = lockAcquire;
    lock.release = lockRelease;
    if (!session().begin(loop, reg, tel, events, makeHardware(), lock)) return false;
    session().setClock(&g_clock);   // `GET TIME` answers from the one clock
    return true;
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

int diveLogBoot(char* note, size_t cap) {
    if (note && cap) note[0] = '\0';
    if (g_logMtx) xSemaphoreTake(g_logMtx, portMAX_DELAY);
    const bool mounted = g_diveStore.begin();
    int n = -1;
    if (mounted) {
        static int slots[DiveStore::kMaxDives];
        n = g_diveStore.list(slots, nullptr, nullptr, DiveStore::kMaxDives);
    }
    const bool formatted = g_diveStore.lastBeginFormatted();
    if (g_logMtx) xSemaphoreGive(g_logMtx);
    if (note && cap) {
        if (!mounted) snprintf(note, cap, "not mounted");
        else if (formatted) snprintf(note, cap, "mounted after FORMAT: dive history erased");
    }
    return n;
}

long unsigned diveLogUsedBytes() { return (long unsigned)g_diveStore.usedBytes(); }
long unsigned diveLogTotalBytes() { return (long unsigned)g_diveStore.totalBytes(); }
bool diveLogMountFormatted() { return g_diveStore.lastBeginFormatted(); }

int diveLogList(int* slots, uint32_t* mtimes, uint32_t* seqs, int max) {
    ListCtx o{slots, mtimes, seqs, max, 0};
    session().withLock(listUnderLock, &o);
    return o.n;
}

bool diveLogSeqRange(uint32_t& lo, uint32_t& hi) {
    SeqRangeCtx o{0, 0, false};
    session().withLock(seqRangeUnderLock, &o);
    lo = o.lo;
    hi = o.hi;
    return o.any;
}

} // namespace comms
