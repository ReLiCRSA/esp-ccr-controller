// ============================================================================
// uart2_task.cpp — the Serial2 shim for the handset (PIN_UART_H_TX/RX, 115200).
//
// All protocol behaviour lives in comms::Session / UartLink / CommandRouter
// (host-tested); all hardware side effects live in session_wiring.cpp. This
// file moves bytes and nothing else: it feeds them to proto::LineReader (the
// length cap and the overflow rule, host-tested), runs the periodic downlink,
// and feeds the watchdog.
//
// Hang rules for this task: the read never blocks past the stream timeout, an
// overlong burst gets one ERR and its whole line is dropped (no echo storm, no
// unbounded buffer, no truncated command executed), and the watchdog is fed
// once per pump. One command does not return from a pump in bounded time:
// `LOG EXPORT` streams the whole dive through the sink below, so the link feeds
// from inside that row loop as each row reaches the wire (review M-1,
// util/wdtfeed.h). Neither feed is a way to survive a hang: if Serial2 wedges,
// both stop being reached, and this task dies into the supervisor's SAFE
// policy — logging dies before control, never the other way round.
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

    proto::LineReader rx;

    for (;;) {
        while (Serial2.available()) {
            const int c = Serial2.read();
            if (c < 0) break;
            // Line assembly is proto::LineReader (host-tested): an over-long
            // line is dropped WHOLE, so a truncated command can never reach the
            // router. See PLAN 16 and test_uart_proto's overflow cases.
            switch (rx.push(c, millis())) {
                case proto::LineReader::Out::LINE:
                    session().handleLine(rx.line(), millis(), sinkWrite, nullptr);
                    break;
                case proto::LineReader::Out::TOO_LONG:
                    Serial2.print(rx.errText());
                    break;
                case proto::LineReader::Out::NOTHING:
                default:
                    break;
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
