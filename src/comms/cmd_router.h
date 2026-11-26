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
//   * The radio is one policy with three holders (2026-11-26): DIVE (DIVE
//     START drops it, dive end restores it unless the diver had WiFi off),
//     DEPTH (off past WIFI_OFF_DEPTH, back after WIFI_ON_HOLD_S below
//     WIFI_ON_DEPTH), IDLE (off after WIFI_IDLE_MIN minutes with no station
//     associated). Diver intent outranks the machine: auto never re-arms a
//     WIFI OFF, and only the words can clear IDLE. See wifiTick below.
//   * Diver outranks the machine: ACK and CELLPROOF OVERRIDE are honoured
//     and replied honestly, never silently re-imposed.
// ============================================================================
#pragma once

#include <cstddef>
#include <cstdint>

#include "comms/settings.h"
#include "comms/telemetry_bridge.h"
#include "comms/uart_proto.h"
#include "comms/wallclock.h"
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
        // Bits 5..8 (CAL O2 ARM/CONFIRM/CANCEL, CAL DEPTH) were retired
        // 2026-11-24, not renumbered: the commands they served are refused on
        // the wire now (`ERR not implemented`), because nothing on the board
        // consumed them — the O2Sequencer was never instantiated and the depth
        // request flag was raised and never read. They are recorded here so the
        // next action bit added is 1 << 13, never a recycled number: a hook
        // that means something different from what the sender thought is exactly
        // the class of bug the protocol's append-only rule exists to prevent.
        ACT_LOG_LIST = 1 << 9,     // transport streams the dive directory
        ACT_LOG_EXPORT = 1 << 10,  // ...or one dive, logExportIndex()
        ACT_SAFE_ACK = 1 << 11,    // SAFE-mode ACK: the transport restarts
                                   // the controller clean (see setSafeLockout)
        ACT_CLOCK_SET = 1 << 12,   // TIME SET arrived: the transport writes the
                                   // wall clock (this class does not own one)
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

    // ---- radio policy (2026-11-26): the radio is intent AND not held -------
    // `wifiEnabled()` stays what WIFI ON|OFF changes and remembers: the
    // diver's intent. `wifiRadioWanted()` is what the transport OBEYS —
    // intent AND no holder AND not SAFE-locked. The holders:
    //
    //   DIVE   — today's mechanism, unchanged: DIVE START drops the radio and
    //            leaves wifiAutoOff_ so dive end knows to restore the intent.
    //            Restored at end even at depth: the intent comes back; if the
    //            DEPTH latch is up (it tracks depth whether or not anyone
    //            asked for a radio), the radio stays down until the diver is
    //            actually shallow.
    //   DEPTH  — depth > WIFI_OFF_DEPTH: held off immediately, commanded dive
    //            or not — a diver who forgot DIVE START still has an antenna
    //            on the loop. Released only after depth stays < WIFI_ON_DEPTH
    //            for WIFI_ON_HOLD_S straight; the 1..2 m band changes nothing,
    //            so a safety stop does not flap the radio. Stale/held depth
    //            (`GET DS`'s used=0) freezes the latch: the policy refuses to
    //            drive a radio from a number that is not being measured.
    //   IDLE   — radio up and no station associated for WIFI_IDLE_MIN minutes
    //            (0 = never). Any association rewinds the window; so does the
    //            radio coming up, so a long dive cannot end with the AP dying
    //            the instant dive end restores it. Words clear it (WIFI ON is
    //            an intent event), depth does not: a dive that went down past
    //            2 m rewinds the window for the surface stretch that follows.
    //
    // Rules with tests pinned to them: auto never overrides the diver (every
    // depth/idle release restores the intent, never overrules a WIFI OFF);
    // the policy never writes NVS; SAFE lockout outranks everything, so no
    // policy path brings up the AP of a controller that came up in SAFE.
    //
    // wifiTick() runs on loopTask UNDER THE SESSION LOCK — the same lock that
    // serializes every exec() — so these fields have one writer at a time.
    // ControlTask never calls it: control must never wait on the task that
    // answers clients (the telemetry_bridge rule).
    enum WifiHold : uint8_t {
        WH_NONE = 0,     // radio up
        WH_DIVER = 1,    // WIFI OFF: the diver's own choice
        WH_DIVE = 2,     // DIVE START, awaiting DIVE END
        WH_DEPTH = 3,    // past the off depth, or not yet shallow long enough
        WH_IDLE = 4,     // no station associated for WIFI_IDLE_MIN minutes
    };

    /// What the physical radio should be: intent, no holder, not SAFE. This —
    /// not wifiEnabled() — is what the ACT_WIFI path hands to web::setRadio.
    bool wifiRadioWanted() const {
        return wifiOn_ && !depthHeld_ && !idleHeld_ && !safeLockout_;
    }
    /// Why the radio is down (WH_NONE when up). The console transition line
    /// and GET WIFI both answer from this, so they cannot disagree.
    uint8_t wifiHold() const;

    /// One policy step. Inputs from loopTask: millis(), the depth snapshot
    /// with its used flag (GET DS's), and the associated-station count.
    /// Returns 0 when nothing changed, +1 when the radio should come up,
    /// -1 when it should go down — loopTask applies that and prints why.
    int wifiTick(uint32_t nowMs, bool depthUsed, float depthM, uint8_t stations);
    uint16_t logExportIndex() const { return logExportIdx_; }

    /// The rolling per-cell histograms ControlTask feeds (PLAN 9: the web
    /// histogram page). Optional: GET HIST without one answers ERR, it does
    /// not fabricate a shape. Read-only here, and display-only — like the
    /// other live status keys this samples while the control thread writes
    /// it; no decision hangs on it (the A5e rule).
    void setHist(const cellhist::Hist* h) { hist_ = h; }
    /// The last CAL AIR verdict, published by ControlTask (it runs the
    /// sampler). Null is allowed and answers `CALAIR,0,...` — "no calibration
    /// since this power-up" — rather than inventing a result.
    void setCalAir(const bridge::CalAir* c) { calAir_ = c; }

    /// The controller's wall clock, published by whoever owns it (the session).
    /// Null is allowed and reads as "no clock", so a transport that has not
    /// wired one up yet is not answering a made-up date.
    void setClock(const wallclock::State* c) { clock_ = c; }

    /// Which chip owns depth right now, published by SensorTask (it owns the
    /// arbiter and its drivers). Null, or a board built without a depth sensor,
    /// answers `DS,0,0,0,0,none` — a stated nothing. `GET DS` always answers: a
    /// protocol-visible command that vanishes under a build flag tells a client
    /// less than one that says "no source" out loud.
    void setDepthSource(const bridge::DepthSource* d) { depthSrc_ = d; }

    /// The epoch TIME SET asked for, valid until the next command. The router
    /// checked the range; the transport performs the write.
    uint32_t clockEpochPending() const { return clockEpoch_; }

    /// Deco picture as the protocol wants it (shared by `DECO`, `GET D` and
    /// the periodic D, line in the UART task).
    /// The published deco picture slot (deco/view.h). Wired by Session::begin
    /// from the bridge it is built over; DECO/GET D answer from the published
    /// generation, never from the live engine (review N8: the cross-task walk
    /// that used to happen here is what the seqlock slot replaced). When no
    /// slot is wired the answer is ERR: a missing publication is a wiring bug,
    /// and it must say so rather than fabricate a plan.
    void setDecoView(const deco::ViewSlot* v) { decoView_ = v; }

    /// Builds a `D,` answer from a PUBLISHED picture (same numbers the event
    /// dive row carries at that instant — N1's agreement is by construction).
    static proto::DecoInfo decoInfoOf(const deco::View& v);

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
    void buildCalAirLine(Response& r);
    void buildDepthSourceLine(Response& r);
    void buildWifiLine(Response& r);

    ctrl::Loop& loop_;
    settings::Registry& reg_;
    uint32_t nowMs_;
    bool safeLockout_ = false;

    bool wifiOn_ = true;       // diver-visible master switch (WIFI ON|OFF)
    bool wifiAutoOff_ = false; // radio dropped by DIVE START, not by the diver
    bool depthHeld_ = false;   // WH_DEPTH latch (tracks depth even at intent off)
    uint8_t wifiStations_ = 0; // last station count the policy was fed (GET WIFI
                               // echoes it: the idle rule's INPUT, made visible)
    bool idleHeld_ = false;    // WH_IDLE latch; cleared by intent events only
    bool wifiPrimed_ = false;  // first tick adopts the state silently, no line
    bool wifiLastWanted_ = false;
    uint32_t shallowSinceMs_ = 0;  // 0 = not shallow; the WIFI_ON_HOLD_S timer
    uint32_t idleSinceMs_ = 0;     // 0 = not counting; start of the idle window
    const wallclock::State* clock_ = nullptr;
    const deco::ViewSlot* decoView_ = nullptr;
    uint32_t clockEpoch_ = 0;
    const cellhist::Hist* hist_ = nullptr;
    const bridge::CalAir* calAir_ = nullptr;
    const bridge::DepthSource* depthSrc_ = nullptr;
    uint16_t logExportIdx_ = 0;
};

} // namespace comms
