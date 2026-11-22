// ============================================================================
// uart2_task.cpp — see uart2_task.h. Owns the Serial2 session and the
// hardware halves of the UartLink hooks: dive store, NVS save, radio,
// settings application, CAL AIR / depth capture requests.
//
// Task-level hang rules: the loop never blocks longer than the read timeout,
// every export stream is bounded, and the watchdog is reset once per pump.
// A wedged UART kills this task, and the supervisor takes the whole board
// into SAFE — logging dies before control, never the other way round.
// ============================================================================
#include "uart2_task.h"

#include <Arduino.h>
#include <WiFi.h>
#include <esp_task_wdt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <cstdio>

#include "comms/cmd_router.h"
#include "comms/nvs_store.h"
#include "comms/settings_apply.h"
#include "comms/telemetry_bridge.h"
#include "comms/uart_link.h"
#include "config/pins.h"
#include "log/dive_store.h"

// The command surface needs the same objects the rest of the firmware uses.
// main.cpp publishes them (it owns creation order); this task never creates
// or destroys them.
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
CommandRouter* g_router = nullptr;
UartLink* g_link = nullptr;
uint32_t g_diveStartEpoch = 0;

void sinkWrite(const char* line, void*) { Serial2.print(line); }

void hookWifi(bool on, void*) {
    if (on) {
        WiFi.persistent(false);   // never wear flash for a reconnect
        WiFi.mode(WIFI_AP);
    } else {
        WiFi.mode(WIFI_OFF);      // truly off: PLAN 9, ~40-70 mA saved
    }
}

bool hookOpenDive(uint32_t epoch, void*) {
    g_diveStartEpoch = epoch;
    if (!g_diveStore.begin()) return false;
    return g_diveStore.startDive(epoch);
}

void hookCloseDive(const logfmt::DiveMeta& meta, void*) {
    g_diveStore.endDive(meta);
}

void hookAppendSnap(void*) {
    if (!g_loop || !g_diveStore.recording()) return;
    logfmt::Snapshot s{};
    // The command task can read the published snapshot, not the chain itself.
    s.tSec = (millis() - g_bridgeTel.diveStartMs) / 1000;
    s.depthCm = (int32_t)(g_bridgeTel.depthM * 100.0f);
    s.votedX1000 = (int16_t)(g_bridgeTel.votedBar * 1000.0f);
    for (int i = 0; i < 3; i++) s.cellX1000[i] = (int16_t)(g_bridgeTel.cellBar[i] * 1000.0f);
    s.setpointX1000 = (int16_t)(g_bridgeTel.setpointBar * 1000.0f);
    s.mode = g_bridgeTel.diveActive ? (g_bridgeTel.autoMode ? 1 : 2) : 0;
    s.alarmBits = g_bridgeTel.alarmBits;
    s.pulses = (uint16_t)g_bridgeTel.pulsesWindow;
    s.battX1000 = (uint16_t)(g_bridgeTel.vbat * 1000.0f);
    g_diveStore.append(s);
}

bool hookSave(void*) { return nvs::Store::save(g_settings); }

void hookSettings(void*) {
    applySettings(g_settings, g_ctrlCfg);
    if (g_loop) g_loop->setConfig(g_ctrlCfg);
}

void hookCalAir(void*) { g_calAirRequest = true; }
void hookCalDepth(void*) { g_calDepthRequest = true; }

int hookListDives(int* slots, uint32_t* mt, int max, void*) {
    return g_diveStore.list(slots, mt, max);
}

int hookExport(int slot, UartLink::Sink sink, void*) {
    return g_diveStore.exportCsv(slot, sink, nullptr);
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

void uart2Task(void*) {
    esp_task_wdt_add(NULL);
    Serial2.begin(115200, SERIAL_8N1, PIN_UART_H_RX, PIN_UART_H_TX);

    g_router = new (std::nothrow) CommandRouter(*g_loop, g_settings, millis());
    g_link = new (std::nothrow) UartLink(*g_loop, g_settings, *g_router,
                                         g_bridgeTel, g_bridgeEvents, makeHardware());
    if (g_link == nullptr) {   // no session: the controller still dives
        esp_task_wdt_delete(NULL);
        vTaskDelete(NULL);
        return;
    }

    char line[proto::MAX_LINE + 2];
    size_t len = 0;
    uint32_t overflowArmed = 0;   // one ERR per overlong burst, no echo storm

    for (;;) {
        // Read with the stream timeout (never blocks past it): a silent
        // handset costs one vTaskDelay, a chatty one stays inside the pump.
        while (Serial2.available()) {
            const int c = Serial2.read();
            if (c < 0) break;
            if (c == '\n' || c == '\r') {
                if (len) {
                    line[len] = '\0';
                    g_link->handleLine(line, sinkWrite, nullptr);
                    len = 0;
                }
                continue;
            }
            if (len < proto::MAX_LINE) {
                line[len++] = (char)c;
            } else if (millis() - overflowArmed > 2000) {
                overflowArmed = millis();
                len = 0;                       // drop the runaway line
                char buf[48];
                const size_t n = proto::formatErr("line too long", buf, sizeof(buf));
                if (n) { buf[n] = '\0'; Serial2.print(buf); }
            }
        }
        g_router->setNowMs(millis());
        g_link->pump(sinkWrite, nullptr, millis());
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

} // namespace

bool diveLogRecording() { return g_diveStore.recording(); }
void diveLogAppendGrid(const logfmt::Snapshot& s) { g_diveStore.append(s); }

bool startUart2Task() {
    return pdTRUE == xTaskCreatePinnedToCore(uart2Task, "uart2", 6144, nullptr, 12,
                                             nullptr, 1);
}

} // namespace comms
