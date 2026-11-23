// ============================================================================
// web_server.h — the WiFi side of the session (PLAN.md 9). ESP32 only.
//
// One rule carries the safety weight: the radio is a policy, not a detail.
// DIVE START takes it down (not "server unreachable" — off, ~40-70 mA and an
// antenna inside the loop housing), DIVE END brings it back if — and only if —
// the diver had it on beforehand. That decision lives in CommandRouter; this
// module is the single place that physically obeys it, for the handset's
// WIFI ON|OFF and for the web's own dive buttons alike.
//
// The WebSocket speaks the handset protocol verbatim: a frame in is one
// command line through comms::session(), reply frames out on the same socket.
// There is no second command surface here, so a browser cannot reach anything
// the handset cannot, and cannot skip the lockouts either.
// ============================================================================
#pragma once

#include <cstddef>
#include <cstdint>

namespace comms {
namespace web {

/// Brings up the AP (SSID/password from the settings registry) and starts the
/// server. Idempotent: calling it when the radio is already up is a no-op.
/// Returns false if the radio or the server could not start — the controller
/// dives regardless, it is just not reachable by phone.
bool start();

/// Closes the WebSocket clients and takes the radio down.
void stop();

/// Obeys the router's radio policy: on -> AP + server, off -> radio off.
/// This is the hook every transport registers, so "WiFi off during a dive"
/// cannot be implemented differently by different clients.
/// Ask for the radio to come up or go down. This only RECORDS the request:
/// the teardown itself runs on loopTask via serviceRadio(). esp_wifi_stop()
/// called from the task that is answering a client — with a station still
/// associated and the driver mid-ioctl — races the WiFi driver's own worker
/// and asserts inside it (`assert failed: spinlock_acquire spinlock.h:122`,
/// backtrace semphr_give_wrapper <- ieee80211_ioctl_process <- ppTask), which
/// is how DIVE START rebooted the controller on the bench on 2026-11-23, i.e.
/// at the one moment the radio is designed to drop.
void setRadio(bool on);

/// True when a recorded request differs from the current state. loopTask uses
/// this to poll briskly instead of once a second.
bool radioPending();

/// Performs a pending radio change. Call from loopTask only: the teardown may
/// block for a driver drain, and no task with control duty should pay that.
void serviceRadio();

/// Pump the captive DNS responder once. Non-blocking; call from a periodic
/// task (the handset task does it every 20 ms). A joined phone resolves
/// nothing without this, and "the page does not load" is what a missing
/// port-53 answer looks like on every phone ever built.
void service();

bool isOn();

/// Live WebSocket clients (the status page shows it; a dive with a browser
/// still connected before the radio drops is worth seeing).
size_t clientCount();

/// Heap high-water seen by the network stack, for the boot log / serial
/// diagnostics: dive exports allocate a file buffer, and the number matters
/// when the log grows.
uint32_t minFreeHeap();

} // namespace web
} // namespace comms
