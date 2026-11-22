// ============================================================================
// cmd_router.h — one command surface for every client: handset UART2 and the
// web WebSocket both feed parsed lines through exactly this code (PLAN.md 8/9:
// "the handset and web edit the identical settings registry — one code path").
//
// Deliberately hardware-free: it mutates the control chain and the settings
// registry, and reports anything a transport must physically do as action
// bits — open the dive log, enable the radio, run CAL AIR on real cells.
// That keeps the whole diver-command surface unit-testable on the host, and
// means the web bridge cannot grow a secret second path around the pre-dive
// lockout or the dive-locked settings.
//
// Rules enforced here (not in the transports):
//   * AUTO only through Loop::requestAuto — refused until pre-dive passed.
//     No client, handset or browser, gets a bypass.
//   * FL_DIVE_LOCK keys refused while a dive runs; FL_SAFETY writes reply
//     `OK SAFETY` so the client can event-log them; every write sets
//     ACT_SETTINGS so exactly one applier reloads the control config.
//   * DIVE START drops the radio (wifiAutoOff_) and auto-restores it at
//     dive end — unless the diver turned WiFi off beforehand.
//   * Diver outranks the machine: ACK and CELLPROOF OVERRIDE are honoured
//     and replied honestly, never silently re-imposed.
// ============================================================================
#pragma once

#include <cstddef>
#include <cstdint>

#include "comms/settings.h"
#include "comms/uart_proto.h"
#include "control/calibration.h"
#include "control/cellhist.h"
#include "control/loop.h"

namespace comms {

class CommandRouter {
public:
    static constexpr size_t MAX_REPLY = 1024;  // must also fit GET * (32 keys)

    enum Actions : uint32_t {
        ACT_NONE = 0,
        ACT_SETTINGS = 1 << 0,     // a SET landed: re-apply the control config
        ACT_DIVE_START = 1 << 1,   // open the dive log file
        ACT_DIVE_END = 1 << 2,     // close/finalise it
        ACT_WIFI = 1 << 3,         // wifiEnabled() changed: bring radio up/down
        ACT_CAL_AIR = 1 << 4,      // run the air calibration on live samples
        ACT_CAL_O2_ARM = 1 << 5,   // arm the two-step O2 calibration
        ACT_CAL_O2_CONFIRM = 1 << 6,
        ACT_CAL_O2_CANCEL = 1 << 7,  // a bare CAL O2 while armed = back out
        ACT_CAL_DEPTH = 1 << 8,    // one-shot depth offset capture
        ACT_LOG_LIST = 1 << 9,     // transport streams the dive directory
        ACT_LOG_EXPORT = 1 << 10,  // ...or one dive, logExportIndex()
    };

    struct Response {
        char text[MAX_REPLY];
        size_t len = 0;      // 0 = no reply line (silent OK)
        uint32_t actions = ACT_NONE;
    };

    /// `nowMs` is the monotonic clock the control chain uses (millis() on the
    /// board, the sim clock in tests): dive edges must land on the same clock.
    CommandRouter(ctrl::Loop& loop, settings::Registry& reg, uint32_t nowMs)
        : loop_(loop), reg_(reg), nowMs_(nowMs) {}

    void setNowMs(uint32_t t) { nowMs_ = t; }

    /// Parses `line` and executes it. Parse refusals never reach the chain.
    Response exec(const char* line);

    // ---- state the transport / firmware reads ------------------------------
    bool wifiEnabled() const { return wifiOn_; }
    bool diving() const { return loop_.diveActive(); }
    /// True when DIVE START turned the radio off (so dive end knows to restore).
    bool wifiAutoOff() const { return wifiAutoOff_; }
    uint16_t logExportIndex() const { return logExportIdx_; }

    /// The rolling per-cell histograms ControlTask feeds (PLAN 9: the web
    /// histogram page). Optional: GET HIST without one answers ERR, it does
    /// not fabricate a shape. Read-only here, and display-only — like the
    /// other live status keys this samples while the control thread writes
    /// it; no decision hangs on it (the A5e rule).
    void setHist(const cellhist::Hist* h) { hist_ = h; }
    /// CAL DEPTH is a one-shot hardware capture; the sensor owner consumes it.
    bool takeDepthCalRequest() {
        const bool w = depthCalPending_;
        depthCalPending_ = false;
        return w;
    }

    /// Deco picture as the protocol wants it (shared by `DECO`, `GET D` and
    /// the periodic D, line in the UART task).
    static proto::DecoInfo decoInfoOf(const ctrl::Loop& loop, bool hasVote);

private:
    void replyOk(Response& r, const char* note = nullptr);
    void replyErr(Response& r, const char* why);
    void replyText(Response& r, const char* text, size_t n);

    void applyWifi(Response& r, bool on);
    void startDive(Response& r);
    void endDive(Response& r);
    void getSpecial(Response& r, const char* key);
    void buildCellsLine(Response& r);

    ctrl::Loop& loop_;
    settings::Registry& reg_;
    uint32_t nowMs_;

    bool wifiOn_ = true;       // diver-visible master switch (WIFI ON|OFF)
    bool wifiAutoOff_ = false; // radio dropped by DIVE START, not by the diver
    bool depthCalPending_ = false;
    const cellhist::Hist* hist_ = nullptr;
    bool o2Armed_ = false;     // first step of the two-step CAL O2 seen here;
                               // the O2Sequencer itself lives in the firmware
                               // and consumes ACT_CAL_O2_* (cancel included)
    uint16_t logExportIdx_ = 0;
};

} // namespace comms
