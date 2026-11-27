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
//     a life-support log quietly. The surface log (/surface.bin, calibration
//     and pre-dive records) deliberately shares that mutex rather than having
//     its own: the ordering number its records carry is read back off the file,
//     so a second lock would let two appends write the same sequence.
//   * Nothing in this file may block on anything a diver is waiting for. The
//     session lock is held only while a command executes; every hook is a
//     bounded, non-blocking action (flag set, record appended, radio moved).
//     That sentence used to be a lie in one place, and review H2 caught it:
//     the CONTROL-task paths into the two stores (the 30 s grid row, the
//     CAL AIR / pre-dive verdict records) waited portMAX_DELAY on the very
//     mutex a multi-second log export holds — a diver browsing his dive list
//     mid-dive could stall the control loop into the supervisor's 1500 ms
//     reboot. Those paths are bounded now (rowgate.h): small budget on the
//     first attempt, zero-wait retries after, and a named lost-row counter
//     when deferral runs out of road. The portMAX_DELAY waits that remain in
//     this file are all command-task entry points, and a native source scan
//     (test/test_controlplane) fails the build if a control-task entry point
//     ever grows one again.
// ============================================================================
#include <Arduino.h>
#include <esp_sleep.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include <atomic>

#include "comms/nvs_store.h"
#include "comms/session.h"
#include "comms/settings_apply.h"
#include "comms/wallclock.h"
#include "comms/web_server.h"
#include "config/defaults.h"
#include "control/configbox.h"
#include "log/contgate.h"
#include "log/continuity.h"
#include "log/continuity_store.h"
#include "log/dive_store.h"
#include "log/rowfill.h"      // the one snapshot-row fill (grid row and event row)
#include "log/rowgate.h"      // the ControlTask grid-row deferral policy (H2)
#include "log/surface_store.h"

// The chain, the registry and the bridge belong to main.cpp; the session must
// drive the very same objects the sensors and the HUD use. The APPLIED
// configuration is not among them anymore: it is a published generation
// (control/configbox.h), which is what lets the settings path apply onto a copy
// instead of writing a global the control chain is reading.
extern ctrl::Loop* g_loop;
extern settings::Registry g_settings;
extern bridge::Telemetry g_bridgeTel;
extern bridge::EventRing g_bridgeEvents;
extern bridge::DiveTotals g_bridgeTotals;
// `volatile` because these are cross-task flags: the command task sets them and
// ControlTask reads/clears them (main.cpp defines them volatile). A declaration
// that drops the qualifier lets the compiler treat a flag as loop-invariant, and
// the linker cannot see the disagreement.
extern volatile bool g_calAirRequest;
// Settings clamps that still owe the wire an `E,19` verdict row. Set with
// fetch_or by the command task (handset and web can both SET), consumed by
// ControlTask's exchange — the ring is SPSC, so only ControlTask may push
// (EventRing::push), and the mask must survive two SETs in one control cycle,
// which is what the OR rather than an assign is for.
extern std::atomic<uint16_t> g_clampPending;

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
SurfaceStore g_surfaceStore;
SemaphoreHandle_t g_sessionMtx = nullptr;
SemaphoreHandle_t g_logMtx = nullptr;
uint32_t g_diveStartMs = 0;

// ---- the control task's bounded paths into the two stores (review H2) ------
// rowgate.h owns the grid-row deferral policy and its counters; the surface
// log gets the same shape inline (one pending record slot, zero-wait retries),
// because it is three call sites and does not need its own class.
logfmt::GridGate g_gridGate;
logfmt::SurfaceRecord g_surfPending{};
bool g_surfPendingDue = false;
uint32_t g_surfLost = 0;

// ---- dive continuity: the record that carries tissues across a reset (H1) --
// Three pieces, and the split is the point (contgate.h carries the reasoning):
//   * g_contGate — ControlTask posts snapshots into it (a copy, µs). It is the
//     only thing ControlTask touches on this path, and it never reaches flash.
//   * continuityDrain() — HudTask (priority 10, below ControlTask's 22) is the
//     one that takes the log mutex and commits the record, with a bounded wait
//     like every other control-adjacent store path.
//   * g_contMirror — the newest record this boot knows, guarded by its own tiny
//     mutex (held for a copy, never for I/O). It is what seeds the next dive,
//     which is why it is updated from a SUCCESSFUL store write and never before:
//     "what this boot seeds" and "what the boot after a reset reads back" are
//     allowed to differ only by a write that failed, and that difference is the
//     crash window the 60 s cadence is sized around.
ContinuityStore g_contStore;
logfmt::ContinuityGate g_contGate;
SemaphoreHandle_t g_contMtx = nullptr;
bool g_contMirrorValid = false;
logfmt::ContinuityRecord g_contMirror{};
bool g_contReadable = true;

/// The store seam, defined with the rest of the continuity wiring below and
/// wired into the gate in initSession(). Declared here because initSession sits
/// above its definition: one door, one wiring site, and the native source scan
/// (test_controlplane) keeps it bounded.
logfmt::ContinuityGate::Result contWriteAttempt(const logfmt::ContinuityRecord& in,
                                                uint32_t budgetMs);

// Take the log mutex for at most budgetMs. 0 is a true try-lock (xSemaphoreTake
// with 0 ticks never yields). FreeRTOS ticks are 10 ms on this build, so a
// sub-tick budget rounds UP to one tick: the bound is "at most one tick of
// waiting", never a silently-zero budget.
bool logLockTry(uint32_t budgetMs) {
    if (!g_logMtx) return false;
    TickType_t ticks = pdMS_TO_TICKS(budgetMs);
    if (budgetMs && ticks == 0) ticks = 1;  // round up: a real budget never becomes a try-lock
    return xSemaphoreTake(g_logMtx, ticks) == pdTRUE;
}

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
    // A grid row deferred by the H2 gate still belongs to THIS dive: the
    // closer flushes it, inside the same lock, before the totals are written
    // and the file closes. This runs on the command task, where the unbounded
    // wait is allowed. If a row is still pending after this, it was lost, and
    // the gate already counted it (HB `logskip=`).
    if (g_gridGate.pending() && g_diveStore.recording()) {
        g_diveStore.append(g_gridGate.pendingRow());
        g_gridGate.claimPendingWritten();  // in the file: not a loss
    }
    if (g_surfPendingDue) {
        if (g_surfaceStore.append(g_surfPending)) g_surfPendingDue = false;
        else g_surfLost++;
    }
    // The two count columns are filled from the store, not from the chain: the
    // store wrote the rows, so it is the only thing that can say how many there
    // are. `M,` claims them, and until 2026-11-24 it claimed 0/0 for a dive that
    // had thirteen samples in it and none of its verdict rows.
    logfmt::DiveMeta m = meta;
    m.snapshotCount = g_diveStore.snapshotCount();
    m.eventCount = g_diveStore.eventCount();
    g_diveStore.endDive(m);
    xSemaphoreGive(g_logMtx);
}

/// The verdict row for a protocol event, into the dive file. Same publication as
/// the `E,` line on the wire (UartLink::drainEvents), so a dive downloaded later
/// answers "what did the machine decide, and when" with the same bytes the
/// handset saw live — a FAIL, a clamp, an override — instead of leaving a reader
/// to infer them from an alarm word that changed between two samples.
void hookAppendEvent(uint16_t id, uint16_t value, uint32_t tSec, void*) {
    if (!g_logMtx) return;
    const logfmt::Event ev{tSec, static_cast<logfmt::EventId>(id), value};
    xSemaphoreTake(g_logMtx, portMAX_DELAY);
    if (g_diveStore.recording()) g_diveStore.append(ev);
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
///
/// Written from the instant the link captured before its first row of the batch
/// (bridge::captureEdge), never from the bridge as it is while the row is being
/// built. Eight edges drained together are eight rows about eight decisions;
/// reading the live scalars eight times made those rows eight measurements of
/// eight different control cycles, so the numbers underneath the verdicts moved
/// between the rows that recorded them.
void hookAppendSnap(const bridge::EdgeSnap& snap, void*) {
    if (!g_loop || !g_logMtx) return;
    // Built from the published bridge scalars, never from the live chain: the
    // control task owns that object. That is the ONLY difference between this row
    // and the 30 s grid row — where the numbers are read from, not what they are
    // converted into — so both rows go through the same fill (src/log/rowfill.h)
    // and the two kinds of row cannot drift apart in scaling, mode, alarm view or
    // the pulses delta.
    logfmt::RowInputs ri{};
    ri.nowMs = snap.nowMs;
    // The dive's start as the CONTROL task counts it, not as this task opened the
    // file: `E,<t_s>` is counted from the published base (main.cpp's dive edge),
    // so this row and the `E,` line written from the same edge state the same
    // second. The command task's file-open instant is up to one control cycle
    // later, and a row counted from it could sit a second away from the verdict
    // it exists to explain. 0 means the chain has not announced a start yet —
    // then this row IS the first thing about the dive, so it says t_s = 0 rather
    // than counting from boot.
    ri.diveStartMs = snap.diveStartMs ? snap.diveStartMs : snap.nowMs;
    ri.smoothedDepthM = snap.depthM;
    ri.haveVote = snap.haveVote;
    ri.shownPpo2 = snap.shownPpo2;
    for (int i = 0; i < 3; i++) ri.cellBar[i] = snap.cellBar[i];
    ri.setpointBar = snap.setpointBar;
    ri.diveActive = snap.diveActive;  // an edge drained after DIVE END is
                                     // neither auto nor manual: mode 0
    ri.autoMode = snap.autoMode;
    // Same view as the grid row: the dive file records what was true, so a
    // snapshot taken at the very edge where the diver ACKed an alarm still says
    // the condition was standing. That edge is exactly when the live word goes
    // quiet, and it is exactly when a row written from the live word used to lose
    // the marker (finding #5).
    ri.alarmLogBits = snap.alarmLogBits;
    ri.cnsPeakPct = snap.cnsPeakPct;
    ri.vbat = snap.vbat;
    // No window of its own, so 0 — which is what these rows have always recorded:
    // `Telemetry::pulsesWindow` is declared and read but written nowhere, so the
    // pulses column of every event row has always been 0. Fixing that means
    // publishing a real per-event window, which moves a byte on an existing
    // column: a decision, not a side effect of this refactor.
    ri.pulsesNow = 0;
    ri.pulsesPrev = 0;
    // The deco picture: the PUBLISHED generation ControlTask computed for the
    // edge this row records (deco/view.h — main.cpp publishes the plan while the
    // cycle's edges are still staged in the ring, so an alarm-edge row cannot be
    // drained against a picture older than the edge). Before 2026-11-28 this row
    // left ceiling/NDL/deco at zero and every one of them read "no stop" (finding
    // N1); and the numbers it now carries are the same ones the `D,` line answers
    // with, because both read one seqlock snapshot.
    ri.hasDecoPicture = snap.hasPic;
    ri.deco = snap.pic;
    // N3: the flags byte is written now — the vote this row was taken under
    // (a row during a held-ppO2 window says FAILED, beside a ppO2 column that
    // shows the held number and an alarm word that marks it) and whether the
    // proof clamp was up at this instant.
    ri.voteMode = snap.voteMode;
    ri.spClamped = snap.cpState == static_cast<uint8_t>(cellproof::State::FAIL);
    logfmt::Snapshot s{};
    logfmt::fillSnapshot(s, ri);

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
    //
    // Applied onto a COPY of the published generation, then published as the
    // next generation (control/configbox.h). Two reasons, and the second is the
    // one that bites:
    //   * Readers used to see the update field by field — a new ALLO_HIGH
    //     beside the old ALLO_CUTOFF, a lowered GF_LOW beside the old GF_HIGH.
    //     applySettings clamps exactly those pairs, over a complete set, so the
    //     global handed back a combination the applier guarantees cannot exist.
    //   * applySettings READS the injection wall, the max-on fuse and the
    //     proof-window pulse count — fields this pass does not write, carried
    //     from boot. Applying onto the object it reads meant its decisions were
    //     made against a config partway through becoming this one.
    // The four carried fields are why the copy starts from the published
    // generation rather than from a fresh struct: defaults.h sets them once.
    //
    // Publish BEFORE postConfig: ControlTask reads the generation at the top of
    // its cycle and the chain takes the queued one in step 0, so the solenoid
    // fuse can see a tighter wall one tick before the engine does — a limiter
    // is allowed to lead the machine it limits, never to lag it.
    ctrl::Config cfg{};
    if (!ctrl::configRead(cfg)) {
        // setup() publishes the boot config before it creates any task that
        // could receive a SET, so this is unreachable. It stays because the
        // alternative is worse: a value-initialised config is presentMask = 0
        // and maxDutyPct = 0 — a machine that reports no cells fitted and
        // injects nothing, which is what a half-initialised boot would leave on
        // a real dive. Say nothing was applied instead.
        Serial.println("settings: NOT APPLIED — no configuration published yet");
        return;
    }
    const uint8_t clamps = applySettings(g_settings, cfg);
    ctrl::configPublish(cfg);
    if (g_loop) g_loop->postConfig(cfg);
    if (clamps != CLAMP_NONE) {
        // `SET` answers OK SAFETY whether or not anything was refused, and the
        // registry keeps the number the diver typed, so the answer alone cannot
        // tell him that GF_LOW=90 over GF_HIGH=50 became two 50s. The verdict
        // now travels two ways: the console line below, at the moment it is
        // made, and `E,<t>,19,<mask>` — the staged latch ControlTask publishes
        // in its next cycle (logfmt::EVT_SETTINGS_CLAMPED; the mask bits are
        // comms::ApplyClamp, and §4's event table says what each one means).
        // The ring is SPSC, so this task latches instead of pushing.
        g_clampPending.fetch_or(clamps, std::memory_order_relaxed);
        char names[64];
        clampList(clamps, names, sizeof names);
        Serial.printf("settings: CLAMPED %s — ordering enforced over what was set\n",
                      names);
    }
}

void hookCalAir(void*) { g_calAirRequest = true; }

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
    hw.appendEventNow = hookAppendEvent;
    hw.saveSettings = hookSave;
    hw.onSettings = hookSettings;
    hw.onCalAir = hookCalAir;
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

uint32_t clockEpoch() { return wallclock::epoch(g_clock, millis()); }

uint8_t clockSource() { return g_clock.source; }

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

logfmt::GridGate::Result gridAppendAttempt(uint32_t budgetMs) {
    if (!logLockTry(budgetMs)) return logfmt::GridGate::BUSY;
    if (g_diveStore.recording()) g_diveStore.append(g_gridGate.pendingRow());
    // Not recording: nothing to write and nothing pending — the dive simply
    // has no file. Treated as WRITTEN (consumed) here because a dive that is
    // not being recorded must not have its grid counter cry wolf every cycle.
    xSemaphoreGive(g_logMtx);
    return logfmt::GridGate::WRITTEN;
}

void diveLogPostGrid(const logfmt::Snapshot& s) {
    g_gridGate.post(s.tSec, s);
}

void diveLogRetryGrid() { g_gridGate.retry(); }

uint32_t diveLogSkippedRows() { return g_gridGate.lostRows(); }

// ---- control-task surface records: same shape (CAL AIR / pre-dive edges) ---

SurfAppend surfaceLogAppendBounded(logfmt::SurfaceRecord& r) {
    if (g_surfPendingDue) g_surfLost++;  // a record displaced is a record lost
    r.epoch = wallclock::epoch(g_clock, millis());
    r.flags = (g_clock.source == wallclock::CARRIED)
                  ? logfmt::SURF_FLAG_CLOCK_CARRIED : 0u;
    g_surfPending = r;
    g_surfPendingDue = true;
    if (logLockTry(2u)) {
        const bool ok = g_surfaceStore.append(g_surfPending);
        xSemaphoreGive(g_logMtx);
        if (ok) {
            g_surfPendingDue = false;
            // Reflect the stamped copy (seq assigned by the store) back to the
            // caller: its console line `surface: record <seq>` is the join key
            // the bench uses to prove the console and the file say the same
            // thing. Printing the caller's pre-append record says `record 0`.
            r = g_surfPending;
            return SURF_WRITTEN;
        }
    }
    return SURF_BUSY;  // pending stands; retried with a zero wait every cycle
}

bool surfaceRetryPending(logfmt::SurfaceRecord& landed) {
    if (!g_surfPendingDue || !logLockTry(0u)) return false;
    const bool ok = g_surfaceStore.append(g_surfPending);
    xSemaphoreGive(g_logMtx);
    if (ok) {
        g_surfPendingDue = false;
        landed = g_surfPending;  // the announce follows from main.cpp, seq and all
        return true;
    }
    return false;
}

uint32_t surfaceSkippedRecords() { return g_surfLost; }

bool initSession(ctrl::Loop& loop, settings::Registry& reg, bridge::Telemetry& tel,
                 bridge::EventRing& events) {
    if (g_sessionMtx == nullptr)
        g_sessionMtx = xSemaphoreCreateMutex();
    if (g_logMtx == nullptr) g_logMtx = xSemaphoreCreateMutex();
    if (g_sessionMtx == nullptr || g_logMtx == nullptr) return false;
    // The gate's store seam (rowgate.h): attempt the append under a bounded
    // log-mutex wait. Wired once, here, so the ONLY way ControlTask reaches
    // the dive file is through this bounded door.
    g_gridGate.setAppender([](uint32_t budgetMs) { return gridAppendAttempt(budgetMs); });
    // Same shape for the continuity record (contgate.h), with one difference
    // that is the whole point of that file: nothing calls this seam from
    // ControlTask. ControlTask posts the snapshot; HudTask drains it.
    g_contGate.setWriter(contWriteAttempt);

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

// ---- control-task grid rows: bounded, deferred, counted (review H2) --------


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

// ---- surface log: the calibration and pre-dive record stream ---------------
// Same mutex as the dive file, deliberately: the sequence a surface record
// carries is read back off the file (max + 1), so two tasks appending at once
// would write the same seq and the log would lose the only thing that orders its
// rows on a board with no clock. Reads take it too — an export scans the file for
// its identity row and then streams it, and a compaction between those two passes
// would report a header and rows from different files.
//
// The clock fields are read outside the mutex, exactly as `GET TIME` reads them:
// they are scalars the command task writes, and ControlTask must not take the
// session lock (a command can hold it while it writes NVS). The cost is that a
// record written in the instant a `TIME SET` lands can carry the new epoch with
// the old source flag, i.e. a date off by however wrong the previous clock was.
// Stated rather than papered over: no lock here can make the clock and the record
// one atomic fact, and the record says which clock it believes.

bool surfaceLogAppend(logfmt::SurfaceRecord& r) {
    if (!g_logMtx) return false;
    r.epoch = wallclock::epoch(g_clock, millis());
    r.flags = (g_clock.source == wallclock::CARRIED)
                  ? logfmt::SURF_FLAG_CLOCK_CARRIED : 0u;
    xSemaphoreTake(g_logMtx, portMAX_DELAY);
    const bool ok = g_surfaceStore.append(r);
    xSemaphoreGive(g_logMtx);
    return ok;
}

int surfaceLogBoot(uint32_t& seqLo, uint32_t& seqHi) {
    seqLo = seqHi = 0u;
    if (g_logMtx) xSemaphoreTake(g_logMtx, portMAX_DELAY);
    const int n = g_surfaceStore.records(&seqLo, &seqHi);   // one pass, one scan line
    if (g_logMtx) xSemaphoreGive(g_logMtx);
    return n;
}

int surfaceLogExportCsv(void (*sink)(const char*, void*), void* ctx) {
    if (!g_logMtx) return -1;
    xSemaphoreTake(g_logMtx, portMAX_DELAY);
    const int n = g_surfaceStore.exportCsv(sink, ctx);
    xSemaphoreGive(g_logMtx);
    return n;
}

// ---- dive continuity (H1): the wiring, and nothing else --------------------
// The record format, the two-slot store logic, the surface-interval rule and the
// honesty sentences all live in logfmt (log/continuity.h) and are graded on the
// host. What is here is the FreeRTOS half: who may take which lock, and with
// what budget. The store shares the LOG mutex (one partition, one writer at a
// time, the same reason /surface.bin shares it), but it does NOT share the
// surface log's sequence: the continuity seq is read off the continuity slots,
// because a calibration record must not be able to reorder a diver's tissues.

namespace {

/// Publishes the chain's seed: the newest record this boot knows, plus the
/// surface interval decided as of `nowEpoch`. `budgetMs` is how long the caller
/// may wait for the mirror mutex — 0 for ControlTask (a skipped refresh retries
/// at the next 1 Hz tick), a few ms for the task that just wrote a record, and a
/// real budget at boot, where nothing else on this path is running yet. The
/// mutex is held for a ~90-byte copy and nothing else, never for I/O.
void publishContinuitySeed(ctrl::Loop* loop, uint32_t nowEpoch,
                           uint32_t budgetMs) {
    if (!loop) return;
    ctrl::Loop::DiveSeed s{};
    logfmt::ContinuityRecord r{};
    bool have = false;
    if (g_contMtx &&
        xSemaphoreTake(g_contMtx, budgetMs == 0u ? 0 : pdMS_TO_TICKS(budgetMs)) == pdTRUE) {
        have = g_contMirrorValid;
        r = g_contMirror;
        xSemaphoreGive(g_contMtx);
    }
    if (have && logfmt::toTissueState(r, s.st)) {
        const logfmt::SurfaceInterval iv =
            logfmt::surfaceIntervalS(r.kind, r.epoch, nowEpoch);
        s.valid = true;
        s.recordSeq = r.seq;
        s.recordEpoch = r.epoch;
        s.kind = r.kind;
        // Zero credit is a decision, not an absence: surfaceIntervalS() says why
        // (a mid-dive snapshot, no clock on one side, or a record dated after this
        // clock), and carrying tissues with no off-gassing credit is the only
        // direction that can be wrong safely.
        s.creditS = iv.credit ? iv.seconds : 0u;
        s.creditWhy = iv.why;
    }
    loop->postDiveSeed(s);
}

/// The store seam (contgate.h): attempt the commit with at most `budgetMs` of
/// waiting on the log mutex. Wired once, in initSession, so the only way the
/// continuity file is written is through this bounded door.
logfmt::ContinuityGate::Result contWriteAttempt(const logfmt::ContinuityRecord& in,
                                                uint32_t budgetMs) {
    if (!logLockTry(budgetMs)) return logfmt::ContinuityGate::BUSY;
    logfmt::ContinuityRecord r = in;
    const bool ok = g_contStore.write(r);
    if (ok) {
        // The mirror follows the flash, inside the same log lock that wrote it:
        // the record the next dive gets seeded from is the one that is actually
        // on the partition, not one that only exists in RAM.
        if (g_contMtx && xSemaphoreTake(g_contMtx, pdMS_TO_TICKS(5u)) == pdTRUE) {
            g_contMirror = r;
            g_contMirrorValid = true;
            xSemaphoreGive(g_contMtx);
        }
    }
    xSemaphoreGive(g_logMtx);
    return ok ? logfmt::ContinuityGate::WRITTEN : logfmt::ContinuityGate::FAILED;
}

} // namespace

ContinuityState continuityState() {
    ContinuityState st;
    st.readable = g_contReadable;
    st.corrupt = g_contStore.corrupt();   // cached flags, no filesystem call: this
                                          // answers on the control task
    if (g_contMtx && xSemaphoreTake(g_contMtx, pdMS_TO_TICKS(20u)) == pdTRUE) {
        st.mirrorValid = g_contMirrorValid;
        st.rec = g_contMirror;
        xSemaphoreGive(g_contMtx);
    }
    return st;
}

int continuityBoot(ctrl::Loop& loop, uint32_t nowEpoch,
                   char* text, size_t cap) {
    if (g_contMtx == nullptr) g_contMtx = xSemaphoreCreateMutex();
    // The unbounded wait is the boot's, and it is the same lock diveLogBoot and
    // surfaceLogBoot already take at this point in setup(): no control-path
    // caller exists on this path, and the partition has to be mounted first.
    if (g_logMtx) xSemaphoreTake(g_logMtx, portMAX_DELAY);
    const bool readable = g_contStore.begin();
    logfmt::ContinuityRecord rec{};
    const bool have = readable && g_contStore.newest(rec);
    const bool corrupt = g_contStore.corrupt();
    if (g_logMtx) xSemaphoreGive(g_logMtx);
    g_contReadable = readable;
    if (g_contMtx && xSemaphoreTake(g_contMtx, pdMS_TO_TICKS(100u)) == pdTRUE) {
        g_contMirror = rec;
        g_contMirrorValid = have;
        xSemaphoreGive(g_contMtx);
    }
    publishContinuitySeed(&loop, nowEpoch, 100u);
    if (text && cap)
        logfmt::continuityBootText(text, cap, readable, corrupt, have ? &rec : nullptr,
                                   nowEpoch);
    if (!readable) return -1;
    if (corrupt) return -2;    // a record file is there and nothing in it decodes
    return have ? 1 : 0;
}

void continuityPublish(const logfmt::ContinuityRecord& r) {
    g_contGate.post(r);   // a copy. No store, no log lock, no flash: contgate.h.
}

ContDrain continuityDrain(ctrl::Loop& loop, uint32_t budgetMs) {
    const bool had = g_contGate.pending();
    if (!had) return CONT_IDLE;
    const logfmt::ContinuityGate::Result r = g_contGate.drain(budgetMs);
    if (r == logfmt::ContinuityGate::WRITTEN) {
        // A record that reached flash is the newest truth this board has: re-seed
        // from it, so dive 2 in the SAME boot is seeded from dive 1's end record
        // and not from whatever boot happened to read.
        publishContinuitySeed(&loop, wallclock::epoch(g_clock, millis()), 5u);
        return CONT_WRITTEN;
    }
    return r == logfmt::ContinuityGate::BUSY ? CONT_BUSY : CONT_LOST;
}

uint32_t continuitySkipped() { return g_contGate.lostSnapshots(); }

void continuityRefreshSeed(ctrl::Loop& loop, uint32_t nowEpoch) {
    publishContinuitySeed(&loop, nowEpoch, 0u);
}

} // namespace comms
