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

/// Renders the settings schema (the GET SCHEMA K, lines) into the router's
/// static text and reports its length. Call once at boot, before the small-stack
/// tasks start: it is the only place the schema's float formatting ever runs.
/// Returns 0 if the table did not fit MAX_SCHEMA.
size_t primeSchema();

/// Length of the rendered schema text; 0 before primeSchema() has run, or if the
/// table does not fit MAX_SCHEMA. Printed at boot so a truncation is visible on
/// the bench instead of being a handset mystery.
size_t schemaLen();

class CommandRouter {
public:
    static constexpr size_t MAX_REPLY = 1024;  // must also fit GET * (32 keys)
    /// Room for the whole K, schema listing. Sized by test_schema, which
    /// renders the real table and requires headroom, so this cannot silently
    /// become a partial list the day someone adds a setting.
    static constexpr size_t MAX_SCHEMA = 2048;

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
        ACT_SAFE_ACK = 1 << 11,    // SAFE-mode ACK: the transport restarts
                                   // the controller clean (see setSafeLockout)
    };


    // The reply buffer lives where the CALLER can afford it, never on a task's
    // stack as a value. This is not style: on 2026-11-23 `GET T` panicked the
    // controller with "Stack canary watchpoint triggered (uart2)", and the
    // backtrace's stack pointers showed why — Response was returned BY VALUE,
    // so one command line held three simultaneous copies of this 1024-byte
    // struct (exec's local, the compiler's hidden return slot, the transport's
    // local) = ~3.1 KB of the uart2 task's 6144, before newlib's _svfprintf_r
    // spent another ~784 on top. Removing float printf did not save it; the
    // buffer movement was the leak.
    struct Response {
        char text[MAX_REPLY];
        size_t len = 0;      // 0 = no reply line (silent OK)
        uint32_t actions = ACT_NONE;

        Response() = default;
        Response(const Response&) = delete;          // enforced by the compiler,
        Response& operator=(const Response&) = delete;  // so it cannot come back
    };

    /// `nowMs` is the monotonic clock the control chain uses (millis() on the
    /// board, the sim clock in tests): dive edges must land on the same clock.
    CommandRouter(ctrl::Loop& loop, settings::Registry& reg, uint32_t nowMs)
        : loop_(loop), reg_(reg), nowMs_(nowMs) {}

    void setNowMs(uint32_t t) { nowMs_ = t; }

    /// SAFE mode: the control chain is NOT running, so a client must not be
    /// able to author anything. Everything is refused with "ERR safe mode"
    /// except ACK, which raises ACT_SAFE_ACK: the acknowledged way out of SAFE
    /// is a clean restart (same recovery as a power-cycle, reachable from the
    /// handset without opening the housing). Resuming control in place was
    /// rejected: it continues on a task set that just lost its stack, and a
    /// second failure mid-dive would find the diver already breathing from it.
    /// Refusing the rest is what keeps "SAFE" a claim about the machine rather
    /// than a display state: a handset that sends AUTO gets the truth.
    void setSafeLockout(bool on) { safeLockout_ = on; }
    bool safeLockout() const { return safeLockout_; }

    /// Parses `line` and executes it. Parse refusals never reach the chain.
    ///
    /// `r` is the caller's storage and is reset here (len/actions/text) before
    /// anything is written; on return it holds exactly one reply line, plus the
    /// action bits the transport must carry out. Out-parameter rather than a
    /// return value on purpose — see the Response comment above for the crash
    /// that decided it. The transport keeps one Response for its lifetime (in
    /// the session arena, not on a task stack) and reuses it per line.
    void exec(const char* line, Response& r);

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
    /// Adopts a reply a formatter wrote straight into r.text (n = bytes it
    /// wrote, 0 = it refused). Same all-or-nothing rule as replyText: a
    /// truncated line is replaced by ERR, never sent short. Used by the big
    /// formatters (GET *, GET D, GET HIST, SCHEMA) so they never need a
    /// MAX_REPLY temporary of their own.
    void replyInPlace(Response& r, size_t n);
    /// Enforces the reply invariant: text is NUL-terminated and strlen(text) ==
    /// len. Every reply path calls this; nothing else writes r.len.
    static void terminate(Response& r);

    void applyWifi(Response& r, bool on);
    void startDive(Response& r);
    void endDive(Response& r);
    void getSpecial(Response& r, const char* key);
    void buildCellsLine(Response& r);

    ctrl::Loop& loop_;
    settings::Registry& reg_;
    uint32_t nowMs_;
    bool safeLockout_ = false;

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
