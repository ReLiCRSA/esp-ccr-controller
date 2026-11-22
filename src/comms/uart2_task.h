// ============================================================================
// uart2_task.h — starts the handset session task (UART2, PLAN.md 8). The
// protocol itself is comms::Session (host-tested); this is the wire.
// ============================================================================
#pragma once

#include <cstdint>

namespace comms {

/// Starts the handset task. Returns false if the session was never
/// initialised or the task could not be created — the controller runs
/// regardless (a handset-less CCR is still a CCR).
bool startUart2Task();

} // namespace comms
