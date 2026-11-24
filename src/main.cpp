// ============================================================================
// main.cpp — task layer: sensors, HUD, supervisor and the ControlTask.
//
// All the control *decisions* live in ctrl::Loop (control/loop.{h,cpp}), the
// same code the PC simulator runs its fault scenarios against. This file only
// moves data between hardware and that chain, and keeps it alive.
//
// Watchdog stack (PLAN.md 2.1):
//   1. TWDT (ESP-IDF task watchdog): diagnostic dumps when a task stops
//      feeding — it warns, the supervisor below acts.
//   2. Supervisor task: SensorTask / HudTask / ControlTask each post a
//      heartbeat; if any goes STALL_MS silent, the supervisor writes a fault
//      record into .noinit DRAM (survives software restart) and esp_restart()s
//      the MCU. A stalled controller is the most dangerous stall there is, so
//      it gets the tightest budget.
//   3. Post-reset policy: any watchdog-driven or brownout boot comes up in
//      SAFE mode: solenoid pin left failsafe-low (never taken over as an
//      output), HB double-beat, ERR solid. Handset ACK (M5) leaves SAFE.
// ============================================================================
#include <Arduino.h>
#include <new>
#include <esp_system.h>
#include <esp_task_wdt.h>

#include "config/pins.h"
#include "config/defaults.h"
#include "control/calibration.h"
#include "control/loop.h"
#include "sensors/ads1115_bus.h"
#include "sensors/battery.h"
#include "sensors/depth_announce.h"
#include "sensors/depth_manager.h"
#include "sensors/lps28.h"
#include "sensors/ms5837.h"
#include "sensors/wire_transport.h"
#include "comms/nvs_store.h"
#include "comms/settings.h"
#include "comms/settings_apply.h"
#include "comms/telemetry_bridge.h"
#include "comms/session.h"

#include "util/depthscale.h"
#include "util/numfmt.h"
#include "util/stackwatch.h"
#include "control/cellhist.h"
#include "comms/uart2_task.h"
#include "comms/web_server.h"
#include "hud.h"

// ---------------------------------------------------------------------------
// Fault record: .noinit DRAM is not zeroed on software reset, so this struct
// tells the next boot WHY we rebooted. The magic guards against garbage.
// ---------------------------------------------------------------------------
struct FaultRecord {
    uint32_t magic;
    uint32_t code;       // 0 = none, 1 = sensor stall, 2 = hud stall,
                         // 3 = brownout seen, 4 = twdt dump then stall
    uint32_t uptimeMs;
    uint32_t espReason;
};
constexpr uint32_t FAULT_MAGIC = 0x52424654; // "RBFT"

static FaultRecord g_fault __attribute__((section(".noinit")));

// ---------------------------------------------------------------------------
// Heartbeats: written by watched tasks, read by the supervisor.
// ---------------------------------------------------------------------------
struct Heartbeat { volatile uint32_t seq; };
static Heartbeat g_hbSensor{0};
static Heartbeat g_hbHud{0};

static Ads1115Bus  g_ads;
static Battery     g_battery;
static volatile bool g_sensorStale = false;
static HudState    g_systemState = HudState::STANDBY;

// ---------------------------------------------------------------------------
// Control chain. ctrl::Loop holds every control decision (and is what the PC
// simulator exercises); this file feeds it and actuates the result.
// ---------------------------------------------------------------------------
// Non-static (internal-linkage-free): the handset/web command tasks reference
// these through externs. The linker — not a comment — enforces the wiring;
// a static here would leave those externs unresolved and the map file would
// grow a undefined-weak hole nobody notices until a task dereferences NULL.
ctrl::Config g_ctrlCfg;
ctrl::Loop* g_loop = nullptr;   // constructed in setup(): needs config
static Heartbeat      g_hbControl{0};
static volatile bool  g_controlRunning = false;

// Cell calibration: raw ADC counts -> ppO2 bar. Seeded with a factory slope so
// an uncalibrated rig still shows plausible numbers; CAL AIR (and the pre-dive
// air check) is what makes those numbers trustworthy.
static cal::Cal g_cellCal[3];

// CAL AIR sampler (cal::AirCal): the command path only raises a request
// flag; the window runs here, in ControlTask, which owns the raw snapshot,
// the dive state and the solenoid.
static cal::AirCal g_airCal{cal::AirCal::Cfg{
    CAL_AIR_PERIOD_MS, static_cast<uint8_t>(CAL_AIR_SAMPLES),
    CAL_AIR_SPREAD_MAX, CAL_AIR_TIMEOUT_MS}};

#if PRESSURE_ENABLE
// Depth lives on its own hardware I2C port (Wire1, pins.h): a wet-harness
// fault or a latched depth chip can never wedge the bus feeding ppO2, and
// each bus recovers (bit-bang clocks + STOP) on its own pins.
// The arbiter picks the source at boot by identity — dive-rated MS5837
// preferred, bench LPS28DFW as fallback — and the choice is LOCKED until
// the next power cycle. Nothing swaps the depth source mid-dive.
static press::WireTransport g_pBus(Wire1, PIN_I2C1_SDA, PIN_I2C1_SCL);
static press::Ms5837        g_ms5837(g_pBus);
static press::Lps28         g_lps28(g_pBus);
static press::DepthArbiter  g_depth(g_ms5837, g_lps28);
#endif
static volatile float g_depthM = 0.0f;
static volatile bool  g_depthValid = false;

// Telemetry snapshot published by ControlTask, read by HUD / handset / web
// bridges. Scalars only, so no lock is needed: 32-bit loads and stores are
// atomic on this core, and a status line is allowed to catch last cycle's
// value. What must never happen is a reader dereferencing the loop's own
// state while the controller is mid-update — hence the copy. The struct and
// the event ring live in comms/telemetry_bridge.h (host-tested).
bridge::Telemetry g_bridgeTel{};
bridge::EventRing g_bridgeEvents;
bridge::DiveTotals g_bridgeTotals;
// The last CAL AIR verdict (running / OK / refused, with the raw numbers).
// ControlTask writes it — it owns the sampler — and the command path reads it
// for `GET CALAIR`; the terminal verdicts also go out as `E,<t>,7,<code>`.
// Until 2026-11-24 this fact existed only as console text.
bridge::CalAir g_calAirStatus;

// Rolling per-cell ppO2 histograms for the web histogram page (PLAN 9).
// One writer: ControlTask, on the control thread, once per cycle -- the
// same single-writer discipline as the chain itself. The command path only
// reads this for display (GET HIST), which the A5e rules allow.
cellhist::Hist g_cellHist;

// Settings + command surface (referenced by comms/uart2_task.cpp).
settings::Registry g_settings;
volatile bool g_calAirRequest = false;

static void setSystemState(HudState s) {
    if (g_systemState == s) return;
    g_systemState = s;
    hudSetState(s);
}

// ---------------------------------------------------------------------------
// SensorTask — core 0, high priority. Never blocks beyond one Wire timeout.
// ---------------------------------------------------------------------------
static void sensorTask(void*) {
    esp_task_wdt_add(NULL); // diagnostics: TWDT dumps backtraces on stall
    vTaskDelay(pdMS_TO_TICKS(200)); // let boot/serial settle
    uint32_t lastBat = 0;
    for (;;) {
        stackwatch::sample(stackwatch::SLOT_SENSOR);
        const uint32_t now = millis();
#ifdef TEST_SENSOR_STALL
        // Livelock simulation: keeps feeding TWDT (so the diagnostic layer
        // stays quiet) but stops advancing the heartbeat — only the
        // supervisor can catch this class of failure.
        if (now > 8000 && now < 15000) {
            esp_task_wdt_reset();
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
#endif
        g_ads.tick(now);

#if PRESSURE_ENABLE
        // Depth is polled by the SAME task that owns the ADC driver, even
        // though depth sits on Wire1: one task owns both buses, transactions
        // are strictly sequential, each capped by its Wire timeout. The
        // arbiter holds the boot-locked source and drives its state machine.
        g_depth.begin(now);      // arbitrates until locked; then re-inits on fault
        g_depth.start(now);      // no-ops unless the cycle is idle
        bool depthSane = false;
        if (g_depth.poll(now)) {
            const float p = g_depth.pressureBar();
            if (p > 0.2f) {      // a sane absolute pressure before deriving depth
                // The same conversion the deco engine uses, from the same
                // density the diver set (`WATER_DENSITY`): a depth on the wire
                // and a ceiling/NDL must be measured with one ruler. See
                // util/depthscale.h for why this line used to be its own
                // arithmetic and what that cost.
                g_depthM = static_cast<float>(depthscale::depthFromSensorBar(
                    p, SURFACE_PRESSURE_BAR, g_ctrlCfg.deco.waterDensity));
                g_depthValid = true;
                depthSane = true;
            }
        }
        // locked() first: there is no source to go stale before boot-lock.
        if (g_depth.locked() && now - g_depth.lastGoodMs() > PRESSURE_STALE_MS)
            g_depthValid = false;
        // Say it once, and say it about a READING rather than about a lock. The
        // drivers report READY the instant the chip is IDENTIFIED (WHO_AM_I,
        // PROM CRC) — the first conversion comes later, paced by the driver's
        // own sample window — so an announce wired to the lock printed
        // `ready, 0.0 hPa`, and that line is the bench's evidence for which
        // sensor, at which address, reading what (BENCH.md C3-a). `depthSane`
        // is the same `p > 0.2f` gate the control path uses, so there is one
        // rule about what counts as a reading. And a source that identifies but
        // never produces one now says so: "no sensor fitted" and "identified,
        // no data" are different facts, and silence reads like the first one.
        static press::Announcer depthAnnouncer;
        const press::Announcer::Action depthAct = depthAnnouncer.update(
            now, g_depth.locked(), depthSane,
            g_depth.locked() ? g_depth.pressureBar() : 0.0f);
        if (depthAct != press::Announcer::Action::NONE) {
            char dline[160];
            const size_t dn = depthAnnouncer.line(depthAct, g_depth.sourceName(),
                                                  dline, sizeof(dline));
            if (dn) Serial.write(dline, dn);
            else Serial.println("depth: ** announce line would not fit");
        }
#endif

        if (now - lastBat >= VBAT_SAMPLE_INTERVAL_MS) {
            lastBat = now;
            for (int i = 0; i < 8; ++i)
                g_battery.addSample(analogReadMilliVolts(PIN_VBAT_SENSE));
        }

        g_sensorStale = g_ads.anyStale(now);
        g_hbSensor.seq++;
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(SENSOR_TASK_PERIOD_MS));
    }
}

// ---------------------------------------------------------------------------
// HudTask — core 1. Owns LED patterns and the periodic serial status line
// (the status line keeps printing even if the sensor path is in trouble —
// that asymmetry is intentional for soak debugging).
// ---------------------------------------------------------------------------
static void hudTask(void*) {
    esp_task_wdt_add(NULL);
    for (;;) {
        stackwatch::sample(stackwatch::SLOT_HUD);
        const uint32_t now = millis();
        hudUpdate(now);
        // Captive DNS rides here, not on the handset task: UART2 may be
        // unwired on the bench (task never created), and a phone that cannot
        // resolve anything is a "page does not load" bug report every time.
        comms::web::service();

        static uint32_t lastStatus = 0;
        if (now - lastStatus >= 5000) {
            lastStatus = now;
            const auto& st = g_ads.stats();
            char vb[16], vt[16], sp[16];
            uint8_t stackSlot = 0;
            const uint32_t stackFree = stackwatch::worstFree(&stackSlot);
            // uart2 gets its own field rather than being folded into the
            // minimum: the minimum belongs to the smallest task (supervisor,
            // 3072 B) and stays flat no matter what a client asks for, so a
            // command-path spike in the task that actually crashed on
            // 2026-11-23 would hide behind it. Appended, per the HB rule.
            const uint32_t uart2Free = stackwatch::freeFor(stackwatch::SLOT_UART2);
            // NOTE: the leading fields are the format soak_logger.py parses.
            // New fields are appended, never reordered or renumbered. `stk=` is
            // appended for exactly that reason, and it is there because of the
            // uart2 panic of 2026-11-23: a stack overflow was invisible on the
            // host and invisible on the bench until it rebooted the controller.
            // Free bytes left in the tightest task is now a number we watch.
            Serial.printf("HB st=%d c0=%d c1=%d c2=%d fresh=%d%d%d "
                          "smp=%lu i2cerr=%lu rec=%lu vbat=%s "
                          "stale=%d batlow=%d"
                          " vote=%s sp=%s al=0x%04x mode=%d pulses=%lu sol=%d ctrl=%d"
                          " stk=%u u2stk=%u\n",
                          static_cast<int>(g_systemState),
                          g_ads.raw(0), g_ads.raw(1), g_ads.raw(2),
                          g_ads.fresh(0, now), g_ads.fresh(1, now),
                          g_ads.fresh(2, now),
                          (unsigned long)st.samples,
                          (unsigned long)st.i2cErrors,
                          (unsigned long)st.recoveries,
                          numfmt::fixed(vb, sizeof(vb), g_battery.volts(), 2),
                          g_sensorStale, g_battery.low(),
                          numfmt::fixed(vt, sizeof(vt), g_bridgeTel.votedBar, 3),
                          numfmt::fixed(sp, sizeof(sp), g_bridgeTel.setpointBar, 2),
                          g_bridgeTel.alarmBits,
                          (int)g_bridgeTel.voteMode, (unsigned long)g_bridgeTel.pulses,
                          (int)g_bridgeTel.solenoid, (int)g_controlRunning,
                          (unsigned)stackFree, (unsigned)uart2Free);
            // Shout for the machine overall AND for the handset task by name, so
            // a command-path regression is named instead of averaged away.
            if (stackFree != 0 && stackFree < stackwatch::kWarnFreeBytes) {
                Serial.printf("** stack low: %u bytes free in '%s'\n",
                              (unsigned)stackFree, stackwatch::name(stackSlot));
            }
            if (uart2Free != 0 && uart2Free < stackwatch::kWarnFreeBytes) {
                Serial.printf("** stack low: %u bytes free in 'uart2'\n", (unsigned)uart2Free);
            }
        }

        g_hbHud.seq++;
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ---------------------------------------------------------------------------
// Surface log records (PLAN 16) — the durable copy of a verdict
//
// A calibration verdict and a pre-dive result are facts about the machine, not
// about a dive, so they cannot live in a dive file: the dive that a failed
// pre-dive check prevents never gets a file to hold the row. They get a record in
// /surface.bin, which the web can export. The handset sees them live
// (`E,<t>,7,<code>`) but has no command to ask for the history — a stated cost,
// recorded in docs/HANDSET_PROTOCOL.md §8, not an oversight.
//
// Two or three flash writes per power-up, never one per control cycle, so the
// control loop's cost is the same class as the dive log's 30 s grid row (a missed
// beat at worst, which the loop's own `skip a beat, never pile up` guard absorbs).
// CAL AIR aborted by a dive (code 4) does land mid-dive; the dive's own grid rows
// are already being written there, so this adds a record, not a new kind of risk.
// ---------------------------------------------------------------------------
static void surfaceCalAir(uint8_t code, const cal::AirCalOutput& a) {
    logfmt::SurfaceRecord r{};
    r.id = logfmt::EVT_CAL_AIR;
    // The code, not a boolean: "it failed" is not a diagnosis, and the diver does
    // something different for warm-up, for a dive that started under it and for
    // cells that never settled. Same numbers as `CALAIR,` and `E,*,7,`.
    r.value = code;
    r.samples = a.samples;
    int16_t spread = 0, raw[3] = {0, 0, 0};
    bool nums = logfmt::countsFromFloat(a.spreadWorst, spread);
    for (int i = 0; i < 3; i++)
        nums = logfmt::countsFromFloat(a.rawAvg[i], raw[i]) && nums;
    if (nums) {
        r.spread = static_cast<uint16_t>(spread);
        for (int i = 0; i < 3; i++) r.raw[i] = raw[i];
    }
    // The seconds left in the warm-up gate are deliberately NOT recorded: they
    // count down, so a reader months later cannot act on them. The record says a
    // calibration was refused for warm-up at this instant, which is the fact that
    // stays true. `GET CALAIR` still carries the countdown for whoever asks now.
    const bool ok = comms::surfaceLogAppend(r);
    if (!ok)
        Serial.println("surface: record NOT written - surface log offline");
    else
        Serial.printf("surface: record %lu id 7 value %u\n",
                      (unsigned long)r.seq, static_cast<unsigned>(code));
}

static void surfacePredive(bool passed, uint16_t failedStep) {
    logfmt::SurfaceRecord r{};
    r.id = passed ? logfmt::EVT_PREPASS : logfmt::EVT_PREFAIL;
    // Which step, for a failure: 0 warm-up, 1 air, 2 solenoid, 3 depth,
    // 4 battery — the same order the `PRE,` line prints its five fields in, so a
    // diver reading the CSV and a handset reading PRE, are counting the same list.
    r.value = passed ? 0u : failedStep;
    const bool ok = comms::surfaceLogAppend(r);
    if (!ok)
        Serial.println("surface: record NOT written - surface log offline");
    else
        Serial.printf("surface: record %lu id %u\n", (unsigned long)r.seq,
                      static_cast<unsigned>(r.id));
}

// ---------------------------------------------------------------------------
// ControlTask — core 1, priority below the sensor task (which must never be
// starved) and above the HUD. Copies the latest driver samples, runs exactly
// one ctrl::Loop cycle, writes the solenoid pin. Nothing here may block: every
// input is a snapshot the SensorTask keeps fresh.
//
// The cycle is also the heartbeat the supervisor watches: a controller that
// stops deciding is a controller that stops injecting, and the dive needs to
// know about it within 1.5 s, not 3.
// ---------------------------------------------------------------------------
static void controlTask(void*) {
    esp_task_wdt_add(NULL);
    pinMode(PIN_SOLENOID, OUTPUT);  // owned from here on; SAFE mode never gets here
    digitalWrite(PIN_SOLENOID, LOW);
    g_controlRunning = true;

    uint32_t next = millis() + CONTROL_TASK_PERIOD_MS;
    for (;;) {
        stackwatch::sample(stackwatch::SLOT_CONTROL);
        const uint32_t now = millis();
        if ((int32_t)(now - next) < 0) {  // not due yet
            vTaskDelay(pdMS_TO_TICKS(2));
            continue;
        }
        next += CONTROL_TASK_PERIOD_MS;
        if ((int32_t)(next - now) < 0) next = now + CONTROL_TASK_PERIOD_MS;  // fell behind: skip a beat, never pile up

        ctrl::SensorData sd;
        for (int i = 0; i < 3; i++)
            sd.cells[i] = g_cellCal[i].apply(static_cast<float>(g_ads.raw(i)));
        sd.depthM = g_depthValid ? g_depthM : 0.0f;
        sd.pressureValid = g_depthValid;
        sd.cellsFresh = !g_ads.anyStale(now);
        sd.vbat = g_battery.volts();

        const ctrl::State st = g_loop->cycle(sd, now);
        digitalWrite(PIN_SOLENOID, st.solenoid ? HIGH : LOW);

        // CAL AIR: sample the raw cells at air, commit the air point.
        // Refused before warmup: an un-polarised cell's "average" is fiction.
        // The `OK CAL AIR` the diver got from the router meant "requested", so
        // every outcome below is published: `GET CALAIR` reads the current
        // state, and a terminal verdict also lands on the wire as E,*,7,<code>.
        // Same t base as the other E, rows (seconds since dive start, or since
        // boot at the surface — see the spec's E, section).
        const uint32_t calTSec = (now - g_bridgeTel.diveStartMs) / 1000u;
        if (g_calAirRequest) {
            g_calAirRequest = false;
            if (now < PREDIVE_WARMUP_MS) {
                const uint32_t left = (PREDIVE_WARMUP_MS - now) / 1000u;
                Serial.printf("cal air: REFUSED - cells still warming up (%u s left)\n",
                              (unsigned)left);
                bridge::publishCalAirRefused(g_calAirStatus, bridge::CALAIR_WARMUP,
                                             left, now, calTSec, g_bridgeEvents);
                // And the durable row: this is the fact that used to exist only
                // as the console line above, which nobody keeps.
                cal::AirCalOutput none;
                surfaceCalAir(g_calAirStatus.code, none);
            } else {
                g_airCal.start(now);
                Serial.println("cal air: sampling (1 Hz x 12, spread-limited)");
                cal::AirCalOutput started;   // running, no samples yet
                started.running = true;
                bridge::publishCalAir(g_calAirStatus, started, now, calTSec, g_bridgeEvents);
            }
        }
        if (g_airCal.running()) {
            cal::AirCalInput ai;
            ai.diving = st.diveActive;
            ai.injecting = st.solenoid;
            ai.fresh = sd.cellsFresh;
            ai.presentMask = g_ctrlCfg.presentMask;
            for (int i = 0; i < 3; i++)
                ai.cellsRaw[i] = static_cast<float>(g_ads.raw(i));
            ai.nowMs = now;
            const cal::AirCalOutput a = g_airCal.update(ai);
            bridge::publishCalAir(g_calAirStatus, a, now, calTSec, g_bridgeEvents);
            // Terminal verdict -> the surface log too. `g_calAirStatus.code` is
            // read back rather than re-derived here: the mapping from a failure
            // reason to a published code lives in one place, so the console line,
            // the `E,` row, the `CALAIR,` answer and the flash record cannot each
            // have their own opinion of what happened.
            if (a.done || a.failed) surfaceCalAir(g_calAirStatus.code, a);
            if (a.done) {
                for (int i = 0; i < 3; i++)
                    if (g_ctrlCfg.presentMask & (1u << i)) {
                        g_cellCal[i].rawAir = a.rawAvg[i];
                        g_cellCal[i].airBar = CAL_AIR_REF_BAR;
                        g_cellCal[i].hasAir = true;  // slope stays nominal until CAL O2
                    }
                char r0[16], r1[16], r2[16], ws[16];
                Serial.printf("cal air: OK  raw cell0=%s cell1=%s cell2=%s"
                              "  worst spread %s  (%u samples)\n",
                              numfmt::fixed(r0, sizeof(r0), a.rawAvg[0], 0),
                              numfmt::fixed(r1, sizeof(r1), a.rawAvg[1], 0),
                              numfmt::fixed(r2, sizeof(r2), a.rawAvg[2], 0),
                              numfmt::fixed(ws, sizeof(ws), a.spreadWorst, 0),
                              (unsigned)a.samples);
            } else if (a.failed) {
                Serial.printf("cal air: FAILED (%s)\n", a.why);
            }
        }

        // History shape, not just the reading: a cell drifting or a loop
        // going foul shows in the distribution well before an alarm trips.
        // Only fresh, fitted cells count -- a frozen ADC sample would
        // reshape history with a lie.
        if (sd.cellsFresh)
            for (int i = 0; i < 3; i++)
                if (g_ctrlCfg.presentMask & (1u << i))
                    g_cellHist.sample(i, g_loop->cellBar(i), now);

        g_bridgeTel.votedBar = st.votedBar;
        g_bridgeTel.setpointBar = st.setpointBar;
        g_bridgeTel.depthM = st.depthSmoothedM;
        g_bridgeTel.vbat = sd.vbat;
        g_bridgeTel.alarmBits = st.alarms.bits;
        g_bridgeTel.voteMode = static_cast<uint8_t>(st.vote.mode);
        g_bridgeTel.pulses = g_loop->pulseCount();
        g_bridgeTel.solenoid = st.solenoid;
        g_bridgeTel.haveVote = st.haveVote;
        for (int i = 0; i < 3; i++) g_bridgeTel.cellBar[i] = sd.cells[i];
        g_bridgeTel.diveActive = st.diveActive;
        g_bridgeTel.autoMode = st.autoMode;
        g_bridgeTel.cpState = static_cast<uint8_t>(st.cellproof.state);
        g_bridgeTel.proofBan = g_loop->proofExclusion();
        g_bridgeTel.unprovenMask = st.cellproof.unprovenHighMask;
        g_bridgeTel.solDutyPct = g_loop->solDutyPct(now);

        // Dive edges, in the chain's own clock: publisher bookkeeping, totals
        // and the snapshot grid all restart with the dive (the handset/web
        // command task opens and closes the actual file).
        static bool prevDive = false;
        static bridge::PublisherState pub;
        static uint32_t diveStartMs = 0, lastSnapMs = 0, lastPulseBase = 0;
        if (st.diveActive && !prevDive) {
            pub = bridge::PublisherState{};
            g_bridgeTotals = bridge::DiveTotals{};
            diveStartMs = now;
            lastSnapMs = now;
            lastPulseBase = g_loop->pulseCount();
            g_bridgeTel.diveStartMs = now;
        }
        prevDive = st.diveActive;

        // Protocol events from the chain's transitions (alarm edges, mode,
        // cellproof, vote membership): the handset E, lines and the dive
        // log's event rows both come from this one publication.
        bridge::publishEvents(st, (now - diveStartMs) / 1000u, g_bridgeEvents, pub);

        // A pre-dive PASS or FAIL becomes a surface record on its rising edge.
        // Edges, not levels: the check runs once and its verdict stands until
        // someone restarts it, so a level would write a row per control cycle —
        // the same reason the `E,` rows are edges. A FAIL record carries the step
        // that failed, which is the part a diver can act on ("battery", not
        // "check failed"), and it is the only durable trace there is when the
        // dive it prevented never happened.
        static bool prevPrePass = false, prevPreFail = false;
        const bool prePass = g_loop->predivePassed();
        const bool preFail = g_loop->prediveFailed();
        if (preFail && !prevPreFail) {
            uint16_t step = static_cast<uint16_t>(g_loop->prediveSeq().current());
            for (int s = 0; s < predive::S_COUNT; ++s)
                if (g_loop->prediveStatus(static_cast<predive::Step>(s)) == predive::FAIL) {
                    step = static_cast<uint16_t>(s);
                    break;
                }
            surfacePredive(false, step);
        } else if (prePass && !prevPrePass) {
            surfacePredive(true, 0);
        }
        prevPrePass = prePass;
        prevPreFail = preFail;

        if (st.diveActive) {
            // running totals for the dive meta (max/min/total only: cheap)
            g_bridgeTotals.durationSec = (now - diveStartMs) / 1000u;
            const int32_t dCm = (int32_t)(st.depthSmoothedM * 100.0f);
            if (dCm > g_bridgeTotals.maxDepthCm) g_bridgeTotals.maxDepthCm = dCm;
            const int32_t v = (int32_t)(st.votedBar * 1000.0f);
            if (st.haveVote) {
                if (v < g_bridgeTotals.minVotedX1000) g_bridgeTotals.minVotedX1000 = v;
                if (v > g_bridgeTotals.maxVotedX1000) g_bridgeTotals.maxVotedX1000 = v;
            }
            g_bridgeTotals.totalPulses = g_loop->pulseCount();
            const float cns = g_loop->cnsPct();
            g_bridgeTotals.cnsPeakPct = cns > 254.0f ? 254 : (uint8_t)cns;
            // The plan the dive ran on, recorded with the dive: which GF pair
            // and which setpoint ladder produced this deco schedule and these
            // doses. Read off the live config, so a dive is reviewed against the
            // settings it actually used rather than the ones on the board now.
            //
            // Rounded, not truncated, on purpose. A decimal like 0.55 is not
            // exact in binary (0.55f is 0.54999999403...), and float times
            // 1000 can land just under the integer it should be, so a truncating
            // cast can print 0.549 for a setting that reads 0.550 on `GET SP2`.
            // These columns exist to be compared with what the handset was
            // answered, so they must not be one count away from it. (The `S,`
            // rows' `votedX1000` still truncates — see PLAN 16's queue.)
            g_bridgeTotals.gfLow = (uint8_t)(g_ctrlCfg.deco.gfLow + 0.5f);
            g_bridgeTotals.gfHigh = (uint8_t)(g_ctrlCfg.deco.gfHigh + 0.5f);
            for (int i = 0; i < 4; i++) {
                const int32_t sx = (int32_t)(g_ctrlCfg.sp.sp[i] * 1000.0f + 0.5f);
                const int16_t v = sx > 32767 ? 32767 : (int16_t)sx;
                if (i == 0) g_bridgeTotals.sp1X1000 = v;
                else if (i == 1) g_bridgeTotals.sp2X1000 = v;
                else if (i == 2) g_bridgeTotals.sp3X1000 = v;
                else g_bridgeTotals.sp4X1000 = v;
            }

            // 30 s snapshot grid (PLAN 10). Event-driven snapshots are taken
            // by the command task alongside each E, line; ControlTask only
            // ever appends the grid row, keeping file ownership single-owner.
            if (comms::diveLogRecording() && now - lastSnapMs >= DIVE_SNAPSHOT_GRID_MS) {
                lastSnapMs = now;
                logfmt::Snapshot sn{};
                sn.tSec = (now - diveStartMs) / 1000u;
                sn.depthCm = dCm;
                sn.votedX1000 = (int16_t)v;
                for (int i = 0; i < 3; i++)
                    sn.cellX1000[i] = (int16_t)(sd.cells[i] * 1000.0f);
                sn.setpointX1000 = (int16_t)(st.setpointBar * 1000.0f);
                sn.mode = st.autoMode ? 1 : 2;
                sn.alarmBits = st.alarms.bits;
                sn.cnsPct = g_bridgeTotals.cnsPeakPct;
                sn.ceilingCm = (int16_t)(g_loop->deco().ceilingM() * 100.0);
                sn.ndlMin = st.haveVote ? (int16_t)g_loop->deco().ndlMinutes(
                    g_loop->deco().depthToBar(st.depthSmoothedM), st.votedBar) : 480;
                sn.decoTotalMin = (int16_t)g_loop->deco().schedule(
                    g_loop->deco().depthToBar(st.depthSmoothedM)).totalTimeMin;
                const uint32_t p = g_loop->pulseCount();
                sn.pulses = (uint16_t)(p - lastPulseBase);
                lastPulseBase = p;
                sn.battX1000 = (uint16_t)(sd.vbat * 1000.0f);
                comms::diveLogAppendGrid(sn);
            }
        }

        // HUD and telemetry read the chain's own state, never a guess about it.
        hudSetDeco(g_loop->haveVote() && g_loop->deco().ceilingBar() >
                                            g_loop->deco().depthToBar(st.depthSmoothedM) + 0.01);
        if (st.alarms.anyCritical) hudSetError(ErrLevel::ALARM);
        else if (st.alarms.anyWarning) hudSetError(ErrLevel::WARNING);
        else hudSetError(ErrLevel::NONE);

        // Keeps a carried clock honest across a reset: the mirror in RTC
        // no-init memory is the wall time as of this cycle, so the boot after a
        // reset resumes the same instant instead of the instant the clock was set.
        comms::clockTick();

        g_hbControl.seq++;
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// ---------------------------------------------------------------------------
// SupervisorTask — core 0, above the watched tasks. Acts when they stall.
// ---------------------------------------------------------------------------
static void supervisorTask(void*) {
    uint32_t lastSensor = 0, lastHud = 0, lastControl = 0;
    uint32_t lastSensorChange = millis(), lastHudChange = millis(), lastControlChange = millis();

    for (;;) {
        stackwatch::sample(stackwatch::SLOT_SUPERVISOR);
        vTaskDelay(pdMS_TO_TICKS(SUPERVISOR_PERIOD_MS));
        const uint32_t now = millis();

        if (g_hbSensor.seq != lastSensor) { lastSensor = g_hbSensor.seq; lastSensorChange = now; }
        if (g_hbHud.seq != lastHud) { lastHud = g_hbHud.seq; lastHudChange = now; }
        if (g_hbControl.seq != lastControl) { lastControl = g_hbControl.seq; lastControlChange = now; }

        if (now - lastSensorChange >= TASK_STALL_SENSOR_MS) {
            g_fault = {FAULT_MAGIC, 1, now, esp_reset_reason()};
            Serial.println("!! FATAL: SensorTask stalled — restart into SAFE");
            esp_restart();
        }
        if (now - lastHudChange >= TASK_STALL_HUD_MS) {
            g_fault = {FAULT_MAGIC, 2, now, esp_reset_reason()};
            Serial.println("!! FATAL: HudTask stalled — restart into SAFE");
            esp_restart();
        }
        if (g_controlRunning && now - lastControlChange >= TASK_STALL_CONTROL_MS) {
            // Tightest budget of the three: a controller that stops cycling
            // stops protecting the loop, and 1.5 s of silence is already a
            // decision we did not make.
            g_fault = {FAULT_MAGIC, 5, now, esp_reset_reason()};
            Serial.println("!! FATAL: ControlTask stalled — restart into SAFE");
            esp_restart();
        }

        // ERR lamp: while the control chain is running it owns the alarm
        // picture (it has the votes, the walls and the latch states). Only in
        // SAFE / no-controller operation does the supervisor fall back to the
        // raw sensor signals.
        if (g_controlRunning) continue;
        if (g_sensorStale) hudSetError(ErrLevel::ALARM);
        else if (g_battery.low()) hudSetError(ErrLevel::WARNING);
        else if (g_ads.stats().recoveries > ADS_RECOVERY_WARN_PER_MIN) hudSetError(ErrLevel::WARNING);
        else hudSetError(ErrLevel::NONE);
    }
}

// ---------------------------------------------------------------------------
// Startup configuration. defaults.h is the source of truth until the NVS-backed
// settings registry is wired (M5); one function, so what the bench reads out
// and what the modules run cannot drift apart.
// ---------------------------------------------------------------------------
static void buildControlConfig() {
    g_ctrlCfg.presentMask = CELLS_PRESENT_MASK;

    g_ctrlCfg.sp.sp[0] = SP1_BAR;   g_ctrlCfg.sp.sp[1] = SP2_BAR;
    g_ctrlCfg.sp.sp[2] = SP3_BAR;   g_ctrlCfg.sp.sp[3] = SP4_BAR;
    g_ctrlCfg.sp.inWaterM = SP_IN_WATER_M;
    g_ctrlCfg.sp.deep1M = SP_DEEP1_M;
    g_ctrlCfg.sp.deep2M = SP_DEEP2_M;
    g_ctrlCfg.sp.hysteresisM = SP_HYST_M;

    g_ctrlCfg.inj.pulseMs = INJ_PULSE_MS;
    g_ctrlCfg.inj.maxDutyPct = INJ_MAX_DUTY_PCT;
    g_ctrlCfg.inj.windowMs = INJ_DUTY_WINDOW_MS;
    g_ctrlCfg.inj.minGapMs = INJ_MIN_GAP_MS;
    g_ctrlCfg.inj.dbOnBar = INJ_DB_ON_BAR;
    g_ctrlCfg.inj.dbOffBar = INJ_DB_OFF_BAR;
    g_ctrlCfg.inj.highWallBar = INJ_HIGH_WALL_BAR;
    g_ctrlCfg.inj.cutoffBar = INJ_CUTOFF_BAR;
    g_ctrlCfg.inj.maxOnMs = INJ_MAX_ON_MS;
    g_ctrlCfg.inj.stuckSlopeBarS = INJ_STUCK_SLOPE_BARS;
    g_ctrlCfg.inj.stuckRiseBar = INJ_STUCK_RISE_BAR;
    g_ctrlCfg.inj.stuckEvalMs = INJ_STUCK_EVAL_MS;
    g_ctrlCfg.inj.stuckHystBar = INJ_STUCK_HYST_BAR;

    g_ctrlCfg.cellproof.enabled = (CELLPROOF_ENABLED != 0);
    g_ctrlCfg.cellproof.depthM = CELLPROOF_DEPTH_M;
    g_ctrlCfg.cellproof.spBar = CELLPROOF_SP_BAR;
    g_ctrlCfg.cellproof.minPassBar = CELLPROOF_MIN_BAR;
    g_ctrlCfg.cellproof.timeoutMs = CELLPROOF_TIMEOUT_MS;
    g_ctrlCfg.cellproof.okMaxSpBar = CELLPROOF_MAXSP_BAR;
    g_ctrlCfg.cellproof.suspectGapBar = CELLPROOF_SUSPECT_GAP_BAR;
    g_ctrlCfg.cellproof.suspectMs = CELLPROOF_SUSPECT_MS;
    g_ctrlCfg.cellproof.rearmCooldownMs = CELLPROOF_REARM_COOLDOWN_MS;
    g_ctrlCfg.cellproof.maxAttempts = CELLPROOF_MAX_ATTEMPTS;

    g_ctrlCfg.alarms.lowBar = PPO2_LOW_ALARM_BAR;
    g_ctrlCfg.alarms.highBar = PPO2_HIGH_ALARM_BAR;
    g_ctrlCfg.alarms.cnsAlarmPct = CNS_ALARM_PCT;
    g_ctrlCfg.alarms.vbatMin = VBAT_MIN_V;

    g_ctrlCfg.predive.warmupMs = PREDIVE_WARMUP_MS;
    g_ctrlCfg.predive.airTargetBar = CAL_AIR_REF_BAR;
    g_ctrlCfg.predive.airTolPct = PREDIVE_AIR_TOL_PCT;
    g_ctrlCfg.predive.airStableSamples = PREDIVE_AIR_STABLE;
    g_ctrlCfg.predive.depthMaxM = PREDIVE_DEPTH_MAX_M;

    g_ctrlCfg.pulseTest.pulses = PREDIVE_TEST_PULSES;
    g_ctrlCfg.pulseTest.baselineSamples = PREDIVE_TEST_BASELINE;
    g_ctrlCfg.pulseTest.riseMinBar = PREDIVE_RISE_MIN_BAR;
    g_ctrlCfg.pulseTest.pulseTimeoutMs = PREDIVE_PULSE_TIMEOUT_MS;
    g_ctrlCfg.pulseTest.recoverTimeoutMs = PREDIVE_RECOVER_TIMEOUT_MS;
    g_ctrlCfg.pulseTest.recoverTolBar = PREDIVE_RECOVER_TOL_BAR;

    g_ctrlCfg.deco.pSurfaceBar = SURFACE_PRESSURE_BAR;
    g_ctrlCfg.deco.waterDensity = WATER_DENSITY_KGL;
}

// Factory slope so an uncalibrated rig still shows plausible numbers. This is
// deliberately NOT marked as a cal: CAL AIR is what makes the numbers
// trustworthy, and the pre-dive air check is what gates AUTO.
static void seedCellCal() {
    for (int i = 0; i < 3; i++) {
        g_cellCal[i].slope = CELL_DEFAULT_SLOPE;
        g_cellCal[i].airBar = CAL_AIR_REF_BAR;
        g_cellCal[i].rawAir = CELL_AIR_RAW_NOMINAL;
        g_cellCal[i].hasAir = false;
        g_cellCal[i].hasO2 = false;
    }
}

// ---------------------------------------------------------------------------
static const char* resetReasonName(uint32_t r) {
    switch (r) {
        case ESP_RST_POWERON:  return "POWERON";
        case ESP_RST_EXT:      return "EXTERNAL";
        case ESP_RST_SW:       return "SOFTWARE";
        case ESP_RST_PANIC:    return "PANIC";
        case ESP_RST_INT_WDT:  return "INT_WDT";
        case ESP_RST_TASK_WDT: return "TASK_WDT";
        case ESP_RST_WDT:      return "WDT";
        case ESP_RST_BROWNOUT: return "BROWNOUT";
        default:               return "UNKNOWN";
    }
}

void setup() {
    Serial.begin(115200);
    delay(100);
    Serial.println();
    Serial.println("=== ESP-CCR controller M1 boot ===");
    Serial.printf("reset reason: %s\n", resetReasonName(esp_reset_reason()));

    hudInit();

    // Decide boot state: fault record from the supervisor, or a watchdog /
    // brownout reset reason, forces SAFE.
    bool forceSafe = false;
    if (g_fault.magic == FAULT_MAGIC) {
        Serial.printf("PRIOR FAULT: code=%lu uptime=%lums esp=%s\n",
                      (unsigned long)g_fault.code,
                      (unsigned long)g_fault.uptimeMs,
                      resetReasonName(g_fault.espReason));
        forceSafe = true;
        g_fault.magic = 0; // one-shot
    }
    const uint32_t r = esp_reset_reason();
    if (r == ESP_RST_TASK_WDT || r == ESP_RST_INT_WDT || r == ESP_RST_WDT ||
        r == ESP_RST_BROWNOUT || r == ESP_RST_PANIC) {
        forceSafe = true;
    }

    // Task watchdog: diagnostic layer (warns+backtraces). The supervisor
    // provides the deterministic reboot.
    esp_task_wdt_init(3, true);

    g_ads.begin();
    g_battery.begin();
#if PRESSURE_ENABLE
    g_pBus.begin();                // Wire1: 100 kHz, 50 ms cap, own pin pair
    g_depth.begin(millis());       // non-blocking; SensorTask keeps retrying
#endif

    // Boot order matters: NVS overlays the compile-time defaults, then ONE
    // applier pass (the same one the handset/web use mid-flight) puts the
    // numbers into the chain. A corrupt or schema-old NVS area falls back to
    // defaults silently — the boot log says what was loaded.
    const int loadedKeys = nvs::Store::load(g_settings);
    buildControlConfig();
    const uint8_t applyClamps = comms::applySettings(g_settings, g_ctrlCfg);
    if (loadedKeys > 0)
        Serial.printf("settings: %d keys from NVS%s\n", loadedKeys,
                      applyClamps ? "  [SOME VALUES CLAMPED: ordering enforced]" : "");
    seedCellCal();
    g_loop = new (std::nothrow) ctrl::Loop(g_ctrlCfg);
    if (g_loop == nullptr) {
        // No control chain, no injection: the pin is never taken over.
        Serial.println("** FATAL: control chain allocation failed — SAFE");
        forceSafe = true;
    }

    if (forceSafe) {
        setSystemState(HudState::SAFE);   // solenoid pin stays INPUT_PULLDOWN
        hudSetError(ErrLevel::HARD);
        Serial.println("** SAFE MODE: acknowledge over the handset link (ACK) or power-cycle after fix");
        Serial.println("**   solenoid pin left failsafe-high-impedance, no control task");
    } else {
        setSystemState(HudState::STANDBY);
        char bs1[16], bs2[16], bs3[16], bs4[16], bwall[16], bcut[16];
        Serial.printf("control: SP=%s/%s/%s/%s cells=%d%d%d wall=%s cutoff=%s "
                      "duty=%u%% pulse=%ums warmup=%us%s\n",
                      numfmt::fixed(bs1, sizeof(bs1), g_ctrlCfg.sp.sp[0], 2),
                      numfmt::fixed(bs2, sizeof(bs2), g_ctrlCfg.sp.sp[1], 2),
                      numfmt::fixed(bs3, sizeof(bs3), g_ctrlCfg.sp.sp[2], 2),
                      numfmt::fixed(bs4, sizeof(bs4), g_ctrlCfg.sp.sp[3], 2),
                      (g_ctrlCfg.presentMask >> 0) & 1, (g_ctrlCfg.presentMask >> 1) & 1,
                      (g_ctrlCfg.presentMask >> 2) & 1,
                      numfmt::fixed(bwall, sizeof(bwall), g_ctrlCfg.inj.highWallBar, 2),
                      numfmt::fixed(bcut, sizeof(bcut), g_ctrlCfg.inj.cutoffBar, 2),
                      (unsigned)g_ctrlCfg.inj.maxDutyPct, (unsigned)g_ctrlCfg.inj.pulseMs,
                      (unsigned)(g_ctrlCfg.predive.warmupMs / 1000),
                      g_cellCal[0].hasAir ? "" : "  [CELLS UNCALIBRATED: run CAL AIR]");
    }

    xTaskCreatePinnedToCore(sensorTask,    "sensor",     4096, nullptr, 24, nullptr, 0);
    xTaskCreatePinnedToCore(hudTask,       "hud",        4096, nullptr, 10, nullptr, 1);
    xTaskCreatePinnedToCore(supervisorTask,"supervisor", 3072, nullptr, 25, nullptr, 0);
    // The controller owns the valve, so it is created last and only when the
    // boot is clean: SAFE mode never starts it, and the pin stays pulled down.
    if (!forceSafe)
        xTaskCreatePinnedToCore(controlTask, "control", 4096, nullptr, 22, nullptr, 1);
    else
        g_controlRunning = false;

    // The handset and web session ride on the chain: no chain, no session.
    // SAFE mode keeps the WEB offline deliberately — a board that just
    // watchdog-reset must not be accepting DIVE START from a phone before
    // anyone knows why it reset. The handset link, though, comes up with the
    // router locked out to everything but ACK: the diver needs a way to
    // acknowledge (ACK restarts the machine clean, no housing to open), and the
    // downlink is still worth watching because the sensor task is alive and the
    // T, line says SAFE. Anything other than ACK is refused with the truth,
    // "ERR safe mode". One session object, and both transports are started over
    // it, so there is no way to end up with a handset that enforces one set of
    // rules and a browser that enforces another.
    if (forceSafe) {
        if (g_loop != nullptr) {
            if (comms::initSession(*g_loop, g_settings, g_bridgeTel, g_bridgeEvents)) {
                comms::session().setHist(&g_cellHist);
                comms::session().setCalAir(&g_calAirStatus);
                comms::session().setSafeLockout(true);
                if (!comms::startUart2Task())
                    Serial.println("** SAFE MODE: handset offline — power-cycle to clear");
                else
                    Serial.println("** SAFE MODE: handset link up, ACK only; web AP offline");
            } else {
                Serial.println("** SAFE MODE: session failed — handset offline, power-cycle to clear");
            }
        } else {
            Serial.println("** SAFE MODE: no control chain — handset and web offline");
        }
    } else if (!comms::initSession(*g_loop, g_settings, g_bridgeTel, g_bridgeEvents)) {
        Serial.println("** session NOT initialised: handset and web offline (control still runs)");
    } else {
        comms::session().setHist(&g_cellHist);   // GET HIST over either client
        comms::session().setCalAir(&g_calAirStatus);   // GET CALAIR: ControlTask's verdict
        if (!comms::startUart2Task())
            Serial.println("** uart2 session NOT started (task create failed)");
        if (!comms::web::start())
            Serial.println("** web AP failed to start: the controller still dives");
    }

    // GET SCHEMA is what a handset or the web page uses to put min/max on its
    // inputs, so a truncated listing is a real fault, not a cosmetic one: it is
    // reported at boot where the bench can see it. (Rendered by Session::begin;
    // see cmd_router.cpp for why it is never rendered on demand.)
    // The dive log gets its own boot line because the alternative is a silent
    // lie: with the filesystem mounted only by DIVE START, a browser's Dives tab
    // and `LOG LIST` on the handset both said "no dives" after every boot,
    // whether the ten files were on flash or not. `used=`/`total=` is the part
    // that separates an empty log from one the filesystem could not read: a
    // LittleFS mount that fails is formatted by begin(), and that is the one
    // thing about this filesystem a diver must never learn after the dive.
    char logNote[72];
    // Before the banner, and before anything can start a dive: a clock carried
    // across a reset is adopted only if its magic and range hold (a battery pull
    // leaves plausible-looking garbage in that memory).
    comms::clockBootRestore();
    char clockNote[96];
    comms::clockBootText(clockNote, sizeof(clockNote));

    const int logDives = comms::diveLogBoot(logNote, sizeof(logNote));
    if (logDives < 0)
        Serial.printf("log: ** LittleFS %s — dives will not be recorded\n", logNote);
    else {
        // `seq a..b` is the eviction evidence. A full FIFO always lists ten
        // dives, so the count alone cannot say whether the newest dive survived:
        // on 2026-11-24 it did not, and nothing on the console said so (PLAN 16).
        char seqTxt[32] = "";
        uint32_t seqLo = 0, seqHi = 0;
        if (logDives > 0 && comms::diveLogSeqRange(seqLo, seqHi))
            snprintf(seqTxt, sizeof(seqTxt), ", seq %lu..%lu",
                     (unsigned long)seqLo, (unsigned long)seqHi);
        else if (logDives > 0)
            snprintf(seqTxt, sizeof(seqTxt), ", seq n/a");   // dives predate the field
        Serial.printf("log: LittleFS mounted, %d dive%s%s, used %lu of %lu bytes%s%s\n",
                      logDives, logDives == 1 ? "" : "s", seqTxt,
                      comms::diveLogUsedBytes(), comms::diveLogTotalBytes(),
                      logNote[0] ? " ** " : "", logNote[0] ? logNote : "");
    }

    // The surface log (calibration + pre-dive records). It gets its own line even
    // when it is empty, because "no records" and "cannot read the records" are
    // different facts and a silence looks like the first one. The seq range is the
    // capacity evidence: 256 records at seq 41..300 says 44 were shed, 1..256 says
    // nothing was (the same reasoning as the dive FIFO's `seq a..b`).
    uint32_t surfLo = 0, surfHi = 0;
    const int surfRecords = comms::surfaceLogBoot(surfLo, surfHi);
    if (surfRecords < 0)
        Serial.println("surface: ** /surface.bin will not read — surface history at risk");
    else if (surfRecords == 0)
        Serial.println("surface: no records yet (GET /surface.csv will be empty)");
    else
        Serial.printf("surface: %d record%s, seq %lu..%lu\n", surfRecords,
                      surfRecords == 1 ? "" : "s",
                      (unsigned long)surfLo, (unsigned long)surfHi);

    Serial.printf("clock: %s\n", clockNote);

    size_t schemaKeys = 0;
    settings::Registry::definitions(schemaKeys);
    const size_t schemaBytes = comms::schemaLen();
    Serial.printf("schema: %u keys, %u bytes", (unsigned)schemaKeys, (unsigned)schemaBytes);
    if (schemaBytes == 0)
        Serial.printf("  ** NOT AVAILABLE: will not fit %u bytes, GET SCHEMA will refuse\n",
                      (unsigned)comms::CommandRouter::MAX_SCHEMA);
    else
        Serial.println();
}

void loop() {
    // The radio comes up and down HERE, not in the task that answered the
    // command: wifi teardown is slow and races the driver's own worker
    // (comms::web::setRadio). 250 ms tick, not 1 s, because this loop is what
    // applies that deferred change, and the reason the radio comes down at DIVE
    // START (PLAN 9: milliamps, and an antenna on the loop) is defeated if the
    // request can sit pending for a second. Polling here costs a wake-up, and
    // loopTask is the only task on this machine with nothing else to do.
    for (int tick = 0; tick < 4; ++tick) {
        stackwatch::sample(stackwatch::SLOT_LOOP);
        if (comms::web::radioPending()) {
            vTaskDelay(pdMS_TO_TICKS(50));   // let the command path finish first
            comms::web::serviceRadio();
            break;                            // re-phase the tick after a teardown
        }
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}
