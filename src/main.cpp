// ============================================================================
// main.cpp — M1: safety skeleton.
//
// Watchdog stack (PLAN.md 2.1):
//   1. TWDT (ESP-IDF task watchdog): diagnostic dumps when a task stops
//      feeding — it warns, the supervisor below acts.
//   2. Supervisor task: SensorTask/HudTask each post a heartbeat; if either
//      goes STALL_MS silent, the supervisor writes a fault record into .noinit
//      DRAM (survives software restart) and esp_restart()s the MCU.
//   3. Post-reset policy: any watchdog-driven or brownout boot comes up in
//       SAFE mode: solenoid pin left failsafe-low, HB double-beat, ERR solid.
//      M2 adds the handset ACK to leave SAFE.
// ============================================================================
#include <Arduino.h>
#include <esp_system.h>
#include <esp_task_wdt.h>

#include "config/pins.h"
#include "config/defaults.h"
#include "sensors/ads1115_bus.h"
#include "sensors/battery.h"
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

        if (now - lastBat >= VBAT_SAMPLE_INTERVAL_MS) {
            lastBat = now;
            g_battery.update();
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
            Serial.printf("HB st=%d c0=%d c1=%d c2=%d fresh=%d%d%d "
                          "smp=%lu i2cerr=%lu rec=%lu vbat=%.2f "
                          "stale=%d batlow=%d\n",
                          static_cast<int>(g_systemState),
                          g_ads.raw(0), g_ads.raw(1), g_ads.raw(2),
                          g_ads.fresh(0, now), g_ads.fresh(1, now),
                          g_ads.fresh(2, now),
                          (unsigned long)st.samples,
                          (unsigned long)st.i2cErrors,
                          (unsigned long)st.recoveries,
                          g_battery.volts(),
                          g_sensorStale, g_battery.low());
        }

        g_hbHud.seq++;
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ---------------------------------------------------------------------------
// SupervisorTask — core 0, above both watched tasks. Acts when they stall.
// ---------------------------------------------------------------------------
static void supervisorTask(void*) {
    uint32_t lastSensor = 0, lastHud = 0;
    uint32_t lastSensorChange = millis(), lastHudChange = millis();

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(SUPERVISOR_PERIOD_MS));
        const uint32_t now = millis();

        if (g_hbSensor.seq != lastSensor) { lastSensor = g_hbSensor.seq; lastSensorChange = now; }
        if (g_hbHud.seq != lastHud) { lastHud = g_hbHud.seq; lastHudChange = now; }

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

        // Sensor freshness drives the ERR lamp immediately (M2 routes the same
        // signal into voting and control degradation).
        if (g_sensorStale) hudSetError(ErrLevel::ALARM);
        else if (g_battery.low()) hudSetError(ErrLevel::WARNING);
        else if (g_ads.stats().recoveries > ADS_RECOVERY_WARN_PER_MIN) hudSetError(ErrLevel::WARNING);
        else hudSetError(ErrLevel::NONE);
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

    if (forceSafe) {
        setSystemState(HudState::SAFE);   // solenoid pin stays INPUT_PULLDOWN
        hudSetError(ErrLevel::HARD);
        Serial.println("** SAFE MODE: acknowledge via handset (M2) or power-cycle after fix");
    } else {
        setSystemState(HudState::STANDBY);
    }

    xTaskCreatePinnedToCore(sensorTask,    "sensor",     4096, nullptr, 24, nullptr, 0);
    xTaskCreatePinnedToCore(hudTask,       "hud",        4096, nullptr, 10, nullptr, 1);
    xTaskCreatePinnedToCore(supervisorTask,"supervisor", 3072, nullptr, 25, nullptr, 0);
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}
