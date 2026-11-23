// ============================================================================
// uart2_task.cpp — the Serial2 shim for the handset (PIN_UART_H_TX/RX, 115200).
//
// All protocol behaviour lives in comms::Session / UartLink / CommandRouter
// (host-tested); all hardware side effects live in session_wiring.cpp. This
// file moves bytes and nothing else: line assembly with a length cap, the
// periodic downlink, and the watchdog.
//
// Hang rules for this task: the read never blocks past the stream timeout, an
// overlong burst gets one ERR and is dropped (no echo storm, no unbounded
// buffer), and the watchdog is fed once per pump. If Serial2 ever wedges, this
// task dies into the supervisor's SAFE policy — logging dies before control,
// never the other way round.
// ============================================================================
#include "uart2_task.h"

#include <Arduino.h>
#include <esp_task_wdt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <cstdio>

#include "comms/session.h"
#include "util/stackwatch.h"
#include "comms/uart_proto.h"
#include "config/pins.h"

namespace comms {

namespace {

void sinkWrite(const char* line, void*) { Serial2.print(line); }

void uart2Task(void*) {
    esp_task_wdt_add(NULL);
    Serial2.begin(115200, SERIAL_8N1, PIN_UART_H_RX, PIN_UART_H_TX);
    Serial2.setTimeout(20);   // read() never blocks longer than this

    if (!session().ready()) {   // no chain: nothing to talk to
        Serial.println("** uart2 session not initialised");
        esp_task_wdt_delete(NULL);
        vTaskDelete(NULL);
        return;
    }

    char line[proto::MAX_LINE + 2];
    size_t len = 0;
    uint32_t overflowArmed = 0;

    for (;;) {
        while (Serial2.available()) {
            const int c = Serial2.read();
            if (c < 0) break;
            if (c == '\n' || c == '\r') {
                if (len) {
                    line[len] = '\0';
                    session().handleLine(line, millis(), sinkWrite, nullptr);
                    len = 0;
                }
                continue;
            }
            if (len < proto::MAX_LINE) {
                line[len++] = (char)c;
            } else if (millis() - overflowArmed > 2000) {
                overflowArmed = millis();
                len = 0;
                char buf[48];
                const size_t n = proto::formatErr("line too long", buf, sizeof(buf));
                if (n) {
                    buf[n] = '\0';
                    Serial2.print(buf);
                }
            }
        }
        session().pump(sinkWrite, nullptr, millis());
        // The task that overflowed on 2026-11-23 now reports its own headroom
        // every 5 s (util/stackwatch.h), so the next one shows up in HB before
        // it becomes a panic.
        stackwatch::sample(stackwatch::SLOT_UART2);
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

} // namespace

bool startUart2Task() {
    if (!session().ready()) return false;
    return pdTRUE == xTaskCreatePinnedToCore(uart2Task, "uart2", 6144, nullptr, 12,
                                             nullptr, 1);
}

} // namespace comms
