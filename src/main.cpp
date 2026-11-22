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
#include "sensors/bmp085.h"
#include "sensors/wire_transport.h"
#include "comms/nvs_store.h"
#include "comms/settings.h"
#include "comms/settings_apply.h"
#include "comms/telemetry_bridge.h"
#include "comms/session.h"
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

#if PRESSURE_ENABLE
static press::WireTransport g_pBus;
static press::Bmp085        g_pressure(g_pBus);
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

// Settings + command surface (referenced by comms/uart2_task.cpp).
settings::Registry g_settings;
volatile bool g_calAirRequest = false;
volatile bool g_calDepthRequest = false;

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
        // Pressure is polled by the SAME task that owns the ADC driver: one
        // task, one I2C owner per transaction, so no half-finished transaction
        // can ever interleave with another driver's. If the ADC driver decides
        // to recover the bus, the pressure read fails — its own retry/backoff
        // absorbs that (that exact case is unit-tested in test_pressure).
        g_pressure.begin(now);   // no-ops once READY, backs off on failure
        g_pressure.start(now);   // no-ops unless the cycle is idle
        if (g_pressure.poll(now)) {
            const float p = g_pressure.pressureBar();
            if (p > 0.2f) {      // a sane absolute pressure before deriving depth
                g_depthM = (p > SURFACE_PRESSURE_BAR)
                               ? (p - SURFACE_PRESSURE_BAR) / BAR_PER_M_SEAWATER
                               : 0.0f;
                g_depthValid = true;
            }
        }
        if (now - g_pressure.lastGoodMs() > PRESSURE_STALE_MS) g_depthValid = false;
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
        const uint32_t now = millis();
        hudUpdate(now);

        static uint32_t lastStatus = 0;
        if (now - lastStatus >= 5000) {
            lastStatus = now;
            const auto& st = g_ads.stats();
            // NOTE: the leading fields are the format soak_logger.py parses.
            // New fields are appended, never reordered or renumbered.
            Serial.printf("HB st=%d c0=%d c1=%d c2=%d fresh=%d%d%d "
                          "smp=%lu i2cerr=%lu rec=%lu vbat=%.2f "
                          "stale=%d batlow=%d"
                          " vote=%.3f sp=%.2f al=0x%04x mode=%d pulses=%lu sol=%d ctrl=%d\n",
                          static_cast<int>(g_systemState),
                          g_ads.raw(0), g_ads.raw(1), g_ads.raw(2),
                          g_ads.fresh(0, now), g_ads.fresh(1, now),
                          g_ads.fresh(2, now),
                          (unsigned long)st.samples,
                          (unsigned long)st.i2cErrors,
                          (unsigned long)st.recoveries,
                          g_battery.volts(),
                          g_sensorStale, g_battery.low(),
                          g_bridgeTel.votedBar, g_bridgeTel.setpointBar, g_bridgeTel.alarmBits,
                          (int)g_bridgeTel.voteMode, (unsigned long)g_bridgeTel.pulses,
                          (int)g_bridgeTel.solenoid, (int)g_controlRunning);
        }

        g_hbHud.seq++;
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
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
    g_pressure.begin(millis());  // non-blocking; SensorTask keeps retrying
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
        Serial.println("** SAFE MODE: acknowledge via handset or power-cycle after fix");
        Serial.println("**   solenoid pin left failsafe-high-impedance, no control task");
    } else {
        setSystemState(HudState::STANDBY);
        Serial.printf("control: SP=%.2f/%.2f/%.2f/%.2f cells=%d%d%d wall=%.2f cutoff=%.2f "
                      "duty=%u%% pulse=%ums warmup=%us%s\n",
                      (double)g_ctrlCfg.sp.sp[0], (double)g_ctrlCfg.sp.sp[1],
                      (double)g_ctrlCfg.sp.sp[2], (double)g_ctrlCfg.sp.sp[3],
                      (g_ctrlCfg.presentMask >> 0) & 1, (g_ctrlCfg.presentMask >> 1) & 1,
                      (g_ctrlCfg.presentMask >> 2) & 1,
                      (double)g_ctrlCfg.inj.highWallBar, (double)g_ctrlCfg.inj.cutoffBar,
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
    // SAFE mode keeps them offline deliberately — a board that just
    // watchdog-reset must not be accepting DIVE START from a phone before
    // anyone knows why it reset. One session object, and both transports are
    // started over it, so there is no way to end up with a handset that
    // enforces one set of rules and a browser that enforces another.
    if (forceSafe) {
        Serial.println("** SAFE MODE: handset (UART2) and web AP stay offline until a clean boot");
    } else if (!comms::initSession(*g_loop, g_settings, g_bridgeTel, g_bridgeEvents)) {
        Serial.println("** session NOT initialised: handset and web offline (control still runs)");
    } else {
        if (!comms::startUart2Task())
            Serial.println("** uart2 session NOT started (task create failed)");
        if (!comms::web::start())
            Serial.println("** web AP failed to start: the controller still dives");
    }
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}
