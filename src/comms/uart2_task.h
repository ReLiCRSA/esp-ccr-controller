// ============================================================================
// uart2_task.h — the ESP32 shim: UART2 (PIN_UART_H_TX/RX) at 115200 feeding
// the host-tested UartLink. Everything that decides protocol behaviour lives
// in UartLink/CommandRouter (unit-tested); this file only moves bytes, owns
// the dive-log radio-side hooks, and keeps its own task healthy:
//   * line-based receive with a length cap (a stuck terminal cannot grow the
//     buffer; overlong lines are dropped with ERR),
//   * the task registers with the watchdog; if Serial2 wedges, the supervisor
//     reboot policy applies exactly as for the other tasks,
//   * no dynamic allocation per line.
// ============================================================================
#pragma once

#include <cstdint>

#include "log/divelog.h"

namespace comms {

/// Starts the handset task. Returns false if it could not be created —
/// the controller runs regardless (a handset-less CCR is a CCR; a
/// control-less handset is a paperweight).
bool startUart2Task();

/// Dive-log accessors for ControlTask's 30 s grid: the file itself is owned
/// by this task (opened/closed by diver commands), so the control task only
/// hands rows over. No-ops when nothing is recording.
bool diveLogRecording();
void diveLogAppendGrid(const logfmt::Snapshot& s);

} // namespace comms
