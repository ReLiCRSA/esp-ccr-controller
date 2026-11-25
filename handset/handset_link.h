// ============================================================================
// handset_link.h — client library for the ESP-CCR handset protocol (v1).
//
// The full contract is docs/HANDSET_PROTOCOL.md; this library is that
// contract made callable: line assembly for the downlink, parsers for every
// record, builders for every command. It is deliberately:
//
//   * HEADER-ONLY C++17, <cstdint>/<cstdio>/<cstdlib>/<cstring> only — drop
//     the file into any project (ESP32 handset, Linux gateway, host tests).
//   * TRANSPORT-AGNOSTIC: you hand it a `put(line)` writer (Serial2.print
//     wrapper, socket write, ...) and feed() it received bytes. The UART2
//     link and the web /ws link carry identical lines, so the same client
//     code drives both.
//   * ZERO ALLOCATION on the protocol path: fixed buffers, C callback
//     struct with void* ctx — the same idiom the controller firmware uses,
//     so no surprises about C++ runtime on an MCU toolchain.
//
// SAFETY semantics this library enforces for you:
//   * Link health: the controller pushes `T,` at 2 Hz. tick(nowMs) raises
//     onLinkDown after kLinkTimeoutMs of silence and onLinkUp again on the
//     first line. A handset MUST alarm on link-down — a dead handset link
//     means a diver with no display.
//   * Defensive parsing: malformed or unknown lines are counted, never
//     crash, and reach the optional onLine observer. Unknown fields inside a
//     known line type are ignored (the protocol is append-only, §7 of the
//     spec: newer controllers stay parseable).
//
// NDL/ceiling honesty: Deco::ndlMin == -1 means "unknown, no valid vote" —
// display it as "---", never as 0 and never as unlimited.
//
// DEPTH SOURCE: one command tells a client which chip is producing the metres and
// whether the controller is using it right now — `getDepthSource()`,
// `onDepthSource`, `depthSourceText()`. Read `used` rather than `locked`: a source
// that holds the slot but has gone quiet leaves the depth on `T,` HELD rather than
// measured, and the `T,` row itself cannot tell you that. See spec §3.
//
// RADIO POLICY: the AP is no longer only what WIFI ON|OFF left it — the controller
// drops it past WIFI_OFF_DEPTH and after WIFI_IDLE_MIN idle minutes, and brings it
// back on its own — so "can I connect, and if not why" is a query, not knowledge:
// `getWifiState()`, `onWifiState`, `wifiStateText()`. `held_by` is an append-only
// enum; switch on the numbers, handle unknown ones as "some newer reason". See §4.
//
// WALL CLOCK: the controller has no RTC chip and its access point has no
// uplink, so a client at the surface is its time source — `timeSet(epoch)`,
// `getTime()`, `onClock`. A handset that keeps time itself should set the
// controller's clock at every link-up, and should show `Clock::epoch == 0` to
// the diver as "dives will be undated". See spec §3 and Clock below.
// ============================================================================
#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace hs {

constexpr int kMaxStops = 8;
constexpr int kRxLineCap = 1024;   // every downlink line fits (spec §1)
constexpr int kTxLineCap = 66;     // uplink hard cap is 64 chars + CRLF
constexpr uint32_t kLinkTimeoutMs = 3000;  // T, comes at 2 Hz

// ---- parsed records ----------------------------------------------------------

struct Telemetry {
    float depthM, votedBar, cells[3], setpointBar, battV;
    int mode;                 // 0 STANDBY 1 AUTO 2 MANUAL 3 SAFE
    uint8_t flags;            // bit0 low, bit1 high, bit2 drift, bit3 major,
                              // bit4 battery, bit5 clamp, bit6 cellproof-fail
};

struct Deco {
    int ndlMin;               // 480 unlimited, -1 UNKNOWN (display "---")
    float decoTotalMin, ceilingM, cnsPct;
    struct Stop { float depthM, minutes; } stops[kMaxStops];
    int stopCount;
};

struct Event { uint32_t tSec; uint16_t id, value; };

struct PreStatus { uint8_t step[5]; bool passed; };  // 0 PEND 1 RUN 2 PASS 3 FAIL
                                                     // order: warmup air solenoid depth battery
struct CellsTest { int state;               // 0 IDLE 1 BASELINE 2 PULSING 3 RECOVER 4 DONE
                   uint8_t passMask, failMask; bool requested; };
struct Proof { int state;                    // 0 INACTIVE 1 IDLE 2 ARMED 3 RUNNING 4 PASS 5 FAIL
               uint8_t unprovenMask, exclusionMask; uint32_t attempts; };
struct Fault { bool stuckOpenLatched, outputFuseTripped; float solDutyPct; };

// `CALAIR,<code>,<age_s>,<samples>,<spread>,<raw0>,<raw1>,<raw2>,<warmup_left_s>`
// — the air calibration's real verdict, added 2026-11-24. It exists because
// `CAL AIR`'s reply is only a receipt: `OK CAL AIR` is answered the same way
// whether the cells are then sampled, refused for warm-up or failed, and until
// then the outcome existed only on the controller's USB console. So do not show
// a "calibrated" tick from the reply — read this line (or the live `E,<t>,7,<code>`
// event, whose value is the same `Code` enumeration).
//
// `<code>` is append-only, like every protocol number: new outcomes are added on
// the right, never renumbered, so a client written today must treat an
// unrecognised code as "some verdict I have no name for", not as an error.
struct CalAir {
    enum Code {
        NONE = 0,        // no CAL AIR since this power-up (fields are all zero)
        RUNNING = 1,     // sampling; `samples` is progress out of 12
        OK = 2,          // air point committed for every fitted cell
        WARMUP = 3,      // refused: cells still polarising, `warmupLeftS` to go
        DIVING = 4,      // aborted: a dive started (CAL AIR is a pre-dive act)
        UNSETTLED = 5,   // failed: cells never stayed inside the spread band
        NOCELLS = 6,     // failed: no fitted cells to sample
    };
    int code;
    uint32_t ageS;       // seconds since that verdict, 0 while running
    int samples;         // samples in the window
    float spreadWorst;   // worst per-cell spread, raw ADC counts
    float rawAvg[3];     // raw ADC counts per cell (0 for an unfitted channel)
    uint32_t warmupLeftS;
};

// Stable English for a code, for a handset that has a display and no wording of
// its own. Words, not numbers, are what a diver reads.
inline const char* calAirText(int code) {
    switch (code) {
        case CalAir::NONE:      return "no calibration since power-up";
        case CalAir::RUNNING:   return "sampling";
        case CalAir::OK:        return "calibrated";
        case CalAir::WARMUP:    return "refused: cells still warming up";
        case CalAir::DIVING:    return "aborted: a dive started";
        case CalAir::UNSETTLED: return "failed: cells never settled";
        case CalAir::NOCELLS:   return "failed: no cells fitted";
        default:                return "unknown verdict";
    }
}

// `DS,<locked>,<used>,<healthy>,<age_s>,<identity>` — which chip is producing the
// metres, and whether the control path is taking them from it right now. Ask with
// `getDepthSource()`; the same facts appear once on the controller's console at
// boot ("depth: LPS28DFW @0x5D ready, ..."), and never again, so a handset that
// wants to show them has to ask.
//
// `used` is the field to look at. A source can hold the slot, be identified and
// be healthy, and still not be feeding the control path — stale bus, no sane
// sample — in which case the `depthM` on the `T,` row is HELD, not measured. That
// is invisible on `T,`, which prints a number either way.
//
// `identity` names the chip and the I2C address it was found at, or "none" when no
// source owns depth. It is the driver's own string, so treat it as free text to
// display rather than something to switch on: a future controller can rename it.
struct DepthSource {
    bool locked;        // a source owns depth (chosen at boot, unchanged until
                        // the next power cycle — nothing swaps it mid-dive)
    bool used;          // the controller is taking depth from it NOW
    bool healthy;       // its driver reports clean transactions
    uint32_t ageS;      // seconds since that source's last good reading (0 when
                        // there is no source, or it never answered)
    char identity[32];  // e.g. "LPS28DFW @0x5D", or "none"
};

// Wording for a handset with a display and no text of its own. Deliberately says
// what to conclude, not just the flags: "1,1,1" means nothing to a diver, "depth
// held: no reading from LPS28DFW @0x5D for 12 s" does. Returns bytes written
// (excluding NUL), 0 if the buffer is too small; never a partial sentence.
inline int depthSourceText(const DepthSource& d, char* buf, std::size_t cap) {
    if (!buf || cap == 0) return 0;
    int w;
    if (!d.locked || d.identity[0] == '\0' || std::strcmp(d.identity, "none") == 0)
        w = std::snprintf(buf, cap, "no depth sensor: depth is not being measured");
    else if (!d.used)
        w = std::snprintf(buf, cap, "depth HELD: no reading from %s for %lu s",
                          d.identity, static_cast<unsigned long>(d.ageS));
    else if (!d.healthy)
        w = std::snprintf(buf, cap, "%s is answering but faulting (%lu s since the "
                                    "last clean read)", d.identity,
                          static_cast<unsigned long>(d.ageS));
    else
        w = std::snprintf(buf, cap, "depth from %s, last read %lu s ago",
                          d.identity, static_cast<unsigned long>(d.ageS));
    if (w < 0 || static_cast<std::size_t>(w) >= cap) { buf[0] = '\0'; return 0; }
    return w;
}

// `WIFI,<on>,<held_by>,<idle_left_s>,<stations>` — whether the AP is physically
// up, and if
// not, who is holding it down (spec §4, added 2026-11-26). `on` is what the
// transport OBEYS, not what was last typed: after `WIFI ON` at depth the honest
// answer is on=0, held_by=DEPTH. The holds:
//
//   DIVE   — dropped by DIVE START; dive end restores intent, and the radio
//            follows DEPTH below.
//   DEPTH  — past WIFI_OFF_DEPTH the radio is down immediately; it returns only
//            after depth stays below WIFI_ON_DEPTH for WIFI_ON_HOLD_S straight.
//            No word clears this hold — only depth does.
//   IDLE   — no station associated for WIFI_IDLE_MIN minutes (0 = never). A
//            `WIFI ON` clears it; depth does not.
//
// An older controller answers `ERR unknown key`, which means "this controller
// cannot say" — not "held by nothing". Never show a reason you did not read.
struct WifiState {
    enum Hold : uint8_t {       // append-only: newer controllers append causes
        HOLD_NONE = 0,          // radio up
        HOLD_DIVER = 1,         // the diver's own WIFI OFF — auto never overrides
        HOLD_DIVE = 2,          // dive in progress
        HOLD_DEPTH = 3,         // past the off depth, or not shallow long enough
        HOLD_IDLE = 4,          // no station associated for WIFI_IDLE_MIN minutes
    };
    bool on;             // AP is up
    uint8_t heldBy;      // one of Hold, or a cause this library predates
    uint32_t idleLeftS;  // seconds until the idle hold would fire; 0 when the
                         // radio is down or WIFI_IDLE_MIN=0 ("never off")
    int16_t stations;    // stations associated as of the last policy tick — the
                         // idle rule's INPUT, on the wire. A countdown that
                         // never shrinks while this is > 0 is the rule working,
                         // not a stall. -1: this controller did not send the
                         // field (it predates the append; 0 would be a lie).
};

// Wording for a handset with a display and no text of its own, in the same
// spirit as depthSourceText(): the sentence states the conclusion, and a future
// cause prints "reason <n>" rather than lying with a stale word. Returns bytes
// written (excluding NUL), 0 if the buffer is too small; never a partial sentence.
inline int wifiStateText(const WifiState& w, char* buf, std::size_t cap) {
    if (!buf || cap == 0) return 0;
    int n;
    if (w.on) {
        if (w.idleLeftS)
            n = std::snprintf(buf, cap, "wifi AP up: auto-off in %lu s "
                                        "if nothing connects",
                              static_cast<unsigned long>(w.idleLeftS));
        else
            n = std::snprintf(buf, cap, "wifi AP up: never auto-offs "
                                        "(WIFI_IDLE_MIN=0)");
    } else switch (w.heldBy) {
        case WifiState::HOLD_DIVER:
            n = std::snprintf(buf, cap, "wifi off: the diver turned it off "
                                        "(WIFI ON to restore)");
            break;
        case WifiState::HOLD_DIVE:
            n = std::snprintf(buf, cap, "wifi off: dive in progress");
            break;
        case WifiState::HOLD_DEPTH:
            n = std::snprintf(buf, cap, "wifi off: held at depth (returns when "
                                        "shallow for WIFI_ON_HOLD_S straight)");
            break;
        case WifiState::HOLD_IDLE:
            n = std::snprintf(buf, cap, "wifi off: idle, no station connected "
                                        "(WIFI ON to restore)");
            break;
        case WifiState::HOLD_NONE:
            // on=0 with reason "nothing": a controller newer than this library
            // appended a cause before defining it, or lied. Say the numbers.
            n = std::snprintf(buf, cap, "wifi off: reported reason is 'none' "
                                        "(controller newer than this library?)");
            break;
        default:
            n = std::snprintf(buf, cap, "wifi off: reason %u "
                                        "(newer controller: this library predates it)",
                              static_cast<unsigned>(w.heldBy));
            break;
    }
    if (n < 0 || static_cast<std::size_t>(n) >= cap) { buf[0] = '\0'; return 0; }
    return n;
}

struct Setting { char key[24]; char value[40]; };      // "KEY=value" answers
// `L,<slot>,<epoch>,<seq>`. `seq` is the fourth field, appended 2026-11-24;
// 0 means the dive predates the field (or the controller is older), and 0 is
// also what the firmware's own FIFO treats as "unknown, evict first". It exists
// because on a board with no clock every dive carries epoch 0, and a handset
// then has ten dives and no way to say which is the newest. The controller
// lists newest-written first — sort by seq yourself if you read a different
// order from an older controller.
struct LogEntry { int slot; uint32_t epoch; uint32_t seq; };

// `DIVE,<slot>,<epoch>,<dive_ver>,<row_ver>,<iso>` — the first line of a
// `LOG EXPORT` dive. `rowFormat` and `iso` are the appends: 0 and "" mean the
// controller did not send them (older firmware). `iso` is `clock-unset` for a
// dive that has no date, never 1970-01-01: an undated dive must look undated.
struct DiveHeader { int slot; uint32_t startEpoch; int version;
                    int rowFormat; char iso[24]; };

// `TIME,<epoch>,<source>,<age_s>` — the controller's wall clock. There is no
// RTC chip and the access point has no uplink, so a client at the surface IS
// the time source (spec §3 "Wall clock"): if your handset can keep time, it
// should set the controller's, because a dive logged without it carries no
// date. epoch 0 = there is no clock, which is an answer, not a 1970 date.
struct Clock {
    uint32_t epoch;
    uint8_t source;    // 0 never set since power-up, 1 set by a command,
                       // 2 carried across a reset the FIRMWARE performed on
                       // itself (panic/watchdog/SAFE ACK). It does NOT survive
                       // the EN pin or a battery pull — measured on the bench
                       // 2026-11-24: EN is a CHIP_PU power-down on this board.
    uint32_t ageS;     // seconds since the clock was last set
};

// Accepted `TIME SET` window, mirroring `src/comms/wallclock.h`. A
// plausible-looking wrong epoch is worse than no epoch — it puts a dive in 1970
// or 2200 and every reader of the log believes it — so the controller refuses
// anything outside it, and so does `Link::timeSet()` below, before the bytes
// reach the wire. Keep these two definitions in step; the conformance suite
// feeds the firmware's own `ERR epoch out of range` and pins both bounds.
constexpr uint32_t kMinEpoch = 946684800u;    // 2000-01-01T00:00:00Z
constexpr uint32_t kMaxEpoch = 4102444800u;   // 2100-01-01T00:00:00Z
inline bool saneEpoch(uint32_t e) { return e >= kMinEpoch && e <= kMaxEpoch; }

struct SchemaEntry {
    char key[24];
    char type;                // 'F' 'I' 'U' 'B' 'S'
    float minv, maxv, def;    // numeric form; STRING: minv/maxv are lengths
    uint32_t flags;           // bit0 dive-lock, bit1 safety
    char defStr[40];          // populated for 'S'
};

// ---- handlers (all optional; NULL = ignore that record) ----------------------

struct Handlers {
    void* ctx = nullptr;
    void (*onTelemetry)(const Telemetry&, void*) = nullptr;
    void (*onDeco)(const Deco&, void*) = nullptr;
    void (*onEvent)(const Event&, void*) = nullptr;
    void (*onOk)(const char* noteOrNull, void*) = nullptr;
    void (*onErr)(const char* reason, void*) = nullptr;
    void (*onSetting)(const Setting&, void*) = nullptr;
    void (*onSchema)(const SchemaEntry&, void*) = nullptr;
    void (*onPre)(const PreStatus&, void*) = nullptr;
    void (*onCells)(const CellsTest&, void*) = nullptr;
    void (*onProof)(const Proof&, void*) = nullptr;
    void (*onFault)(const Fault&, void*) = nullptr;
    void (*onLogList)(const LogEntry&, void*) = nullptr;
    void (*onDiveHeader)(const DiveHeader&, void*) = nullptr;
    void (*onClock)(const Clock&, void*) = nullptr;
    void (*onDepthSource)(const DepthSource&, void*) = nullptr;
    void (*onWifiState)(const WifiState&, void*) = nullptr;
    void (*onCalAir)(const CalAir&, void*) = nullptr;
    // Everything else: CSV export rows (S,/M,), HIST, lines, WebSocket '#'
    // markers, and any line a future controller adds. Observing is optional;
    // ignoring is always safe.
    void (*onLine)(const char* line, void*) = nullptr;
    void (*onLinkDown)(void*) = nullptr;
    void (*onLinkUp)(void*) = nullptr;
};

// ---- pure parsers (public so host tests can pin them) ------------------------
// Each takes a complete line WITHOUT the newline. True = parsed into out.

inline bool starts(const char* l, const char* p) {
    while (*p) { if (*l++ != *p++) return false; }
    return true;
}

inline int split(char* line, char* fields[], int maxFields) {
    int n = 0;
    char* p = line;
    while (n < maxFields) {
        fields[n++] = p;
        while (*p && *p != ',') ++p;
        if (!*p) break;
        *p++ = '\0';
    }
    return n;
}

inline float pf(const char* s) { return static_cast<float>(std::strtod(s, nullptr)); }
inline long  pl(const char* s) { return std::strtol(s, nullptr, 10); }
inline int   pmask(const char* s) { return static_cast<int>(std::strtol(s, nullptr, 8)); }  // %03o

inline bool parseTelemetry(const char* line, Telemetry& t) {
    if (!starts(line, "T,")) return false;
    char buf[128];
    std::strncpy(buf, line, sizeof(buf) - 1); buf[sizeof(buf) - 1] = '\0';
    char* f[10];
    if (split(buf, f, 10) < 9) return false;
    t.depthM = pf(f[1]); t.votedBar = pf(f[2]);
    t.cells[0] = pf(f[3]); t.cells[1] = pf(f[4]); t.cells[2] = pf(f[5]);
    t.setpointBar = pf(f[6]);
    t.mode = std::strcmp(f[7], "AUTO") == 0 ? 1
           : std::strcmp(f[7], "MANUAL") == 0 ? 2
           : std::strcmp(f[7], "SAFE") == 0 ? 3 : 0;
    // firmware prints "FLAGS0x%02X"; the tag is checked, not assumed
    if (!starts(f[8], "FLAGS0x")) return false;
    t.flags = static_cast<uint8_t>(std::strtol(f[8] + 7, nullptr, 16));
    t.battV = pf(f[9]);
    return true;
}

inline bool parseDeco(const char* line, Deco& d) {
    if (!starts(line, "D,")) return false;
    char buf[kRxLineCap];
    std::strncpy(buf, line, sizeof(buf) - 1); buf[sizeof(buf) - 1] = '\0';
    // D, ndl, total, ceiling, [stop]..., cns  ->  5 + stopCount fields
    char* f[5 + kMaxStops];
    const int n = split(buf, f, 5 + kMaxStops);
    if (n < 5) return false;
    d.ndlMin = static_cast<int>(pl(f[1]));
    d.decoTotalMin = pf(f[2]);
    d.ceilingM = pf(f[3]);
    d.stopCount = 0;
    // middle fields are stops "<depth>|<min>"; the LAST field is CNS.
    for (int i = 4; i < n - 1 && d.stopCount < kMaxStops; ++i) {
        char* bar = std::strchr(f[i], '|');
        if (!bar) return false;
        *bar = '\0';
        d.stops[d.stopCount].depthM = pf(f[i]);
        d.stops[d.stopCount].minutes = pf(bar + 1);
        d.stopCount++;
    }
    d.cnsPct = pf(f[n - 1]);
    return true;
}

inline bool parseEvent(const char* line, Event& e) {
    if (!starts(line, "E,")) return false;
    char buf[48];
    std::strncpy(buf, line, sizeof(buf) - 1); buf[sizeof(buf) - 1] = '\0';
    char* f[4];
    if (split(buf, f, 4) < 4) return false;
    e.tSec = static_cast<uint32_t>(pl(f[1]));
    e.id = static_cast<uint16_t>(pl(f[2]));
    e.value = static_cast<uint16_t>(pl(f[3]));
    return true;
}

inline bool parsePre(const char* line, PreStatus& p) {
    if (!starts(line, "PRE,")) return false;
    char buf[48];
    std::strncpy(buf, line, sizeof(buf) - 1); buf[sizeof(buf) - 1] = '\0';
    char* f[7];
    if (split(buf, f, 7) < 7) return false;
    for (int i = 0; i < 5; ++i) p.step[i] = static_cast<uint8_t>(pl(f[1 + i]));
    p.passed = pl(f[6]) != 0;
    return true;
}

inline bool parseCells(const char* line, CellsTest& c) {
    if (!starts(line, "CELLS,")) return false;
    char buf[48];
    std::strncpy(buf, line, sizeof(buf) - 1); buf[sizeof(buf) - 1] = '\0';
    char* f[5];
    if (split(buf, f, 5) < 5) return false;
    c.state = static_cast<int>(pl(f[1]));
    c.passMask = static_cast<uint8_t>(pmask(f[2]));
    c.failMask = static_cast<uint8_t>(pmask(f[3]));
    c.requested = pl(f[4]) != 0;
    return true;
}

inline bool parseProof(const char* line, Proof& p) {
    if (!starts(line, "PROOF,")) return false;
    char buf[48];
    std::strncpy(buf, line, sizeof(buf) - 1); buf[sizeof(buf) - 1] = '\0';
    char* f[5];
    if (split(buf, f, 5) < 5) return false;
    p.state = static_cast<int>(pl(f[1]));
    p.unprovenMask = static_cast<uint8_t>(pmask(f[2]));
    p.exclusionMask = static_cast<uint8_t>(pmask(f[3]));
    p.attempts = static_cast<uint32_t>(pl(f[4]));
    return true;
}

// Append-only in both directions: a controller that sends fewer than the nine
// fields above still parses (the code is the part a client acts on), and fields
// a future controller appends are ignored rather than rejected.
inline bool parseCalAir(const char* line, CalAir& c) {
    if (!starts(line, "CALAIR,")) return false;
    char buf[96];
    std::strncpy(buf, line, sizeof(buf) - 1); buf[sizeof(buf) - 1] = '\0';
    char* f[9];
    const int n = split(buf, f, 9);
    if (n < 2) return false;
    c.code = static_cast<int>(pl(f[1]));
    c.ageS = n >= 3 ? static_cast<uint32_t>(pl(f[2])) : 0u;
    c.samples = n >= 4 ? static_cast<int>(pl(f[3])) : 0;
    c.spreadWorst = n >= 5 ? std::strtof(f[4], nullptr) : 0.0f;
    for (int i = 0; i < 3; i++)
        c.rawAvg[i] = n >= 6 + i ? std::strtof(f[5 + i], nullptr) : 0.0f;
    c.warmupLeftS = n >= 9 ? static_cast<uint32_t>(pl(f[8])) : 0u;
    return true;
}

inline bool parseFault(const char* line, Fault& x) {
    if (!starts(line, "FAULT,")) return false;
    char buf[48];
    std::strncpy(buf, line, sizeof(buf) - 1); buf[sizeof(buf) - 1] = '\0';
    char* f[4];
    if (split(buf, f, 4) < 4) return false;
    x.stuckOpenLatched = pl(f[1]) != 0;
    x.outputFuseTripped = pl(f[2]) != 0;
    x.solDutyPct = pf(f[3]);
    return true;
}

inline bool parseLogList(const char* line, LogEntry& e) {
    if (!starts(line, "L,")) return false;
    char buf[48];
    std::strncpy(buf, line, sizeof(buf) - 1); buf[sizeof(buf) - 1] = '\0';
    char* f[4];
    const int n = split(buf, f, 4);
    if (n < 3) return false;
    e.slot = static_cast<int>(pl(f[1]));
    e.epoch = static_cast<uint32_t>(pl(f[2]));
    // A controller that predates the field still parses; seq 0 = unknown order.
    e.seq = n >= 4 ? static_cast<uint32_t>(pl(f[3])) : 0u;
    return true;
}

inline bool parseDiveHeader(const char* line, DiveHeader& h) {
    if (!starts(line, "DIVE,")) return false;
    // The v3 row is 45 bytes at worst ("DIVE,10,4294967295,3,3,9999-12-31T23:59:59Z"),
    // so 64 here is the firmware's own bound, not a guess.
    char buf[64];
    std::strncpy(buf, line, sizeof(buf) - 1); buf[sizeof(buf) - 1] = '\0';
    char* f[6];
    const int n = split(buf, f, 6);
    if (n < 4) return false;
    h.slot = static_cast<int>(pl(f[1]));
    h.startEpoch = static_cast<uint32_t>(pl(f[2]));
    h.version = static_cast<int>(pl(f[3]));
    h.rowFormat = n >= 5 ? static_cast<int>(pl(f[4])) : 0;
    h.iso[0] = '\0';
    if (n >= 6) {
        std::strncpy(h.iso, f[5], sizeof(h.iso) - 1);
        h.iso[sizeof(h.iso) - 1] = '\0';
    }
    return true;
}

inline bool parseClock(const char* line, Clock& c) {
    if (!starts(line, "TIME,")) return false;
    char buf[48];
    std::strncpy(buf, line, sizeof(buf) - 1); buf[sizeof(buf) - 1] = '\0';
    char* f[4];
    const int n = split(buf, f, 4);
    if (n < 3) return false;              // TIME,<epoch>,<source>[,<age_s>]
    c.epoch = static_cast<uint32_t>(pl(f[1]));
    c.source = static_cast<uint8_t>(pl(f[2]));
    c.ageS = n >= 4 ? static_cast<uint32_t>(pl(f[3])) : 0u;
    return true;
}

inline bool parseDepthSource(const char* line, DepthSource& d) {
    if (!starts(line, "DS,")) return false;
    char buf[64];
    std::strncpy(buf, line, sizeof(buf) - 1); buf[sizeof(buf) - 1] = '\0';
    char* f[8];
    const int n = split(buf, f, 8);
    if (n < 6) return false;                 // DS,<locked>,<used>,<healthy>,<age_s>,<identity>
    d.locked  = pl(f[1]) != 0;
    d.used    = pl(f[2]) != 0;
    d.healthy = pl(f[3]) != 0;
    d.ageS    = static_cast<uint32_t>(pl(f[4]));
    // Identity is the last field and never contains a comma, so a controller that
    // appends a sixth field later still leaves this one readable — the protocol's
    // append-only rule, honoured by the parser rather than assumed.
    std::strncpy(d.identity, f[5], sizeof(d.identity) - 1);
    d.identity[sizeof(d.identity) - 1] = '\0';
    if (d.identity[0] == '\0') std::strncpy(d.identity, "none", sizeof(d.identity) - 1);
    return true;
}

inline bool parseWifiState(const char* line, WifiState& w) {
    if (!starts(line, "WIFI,")) return false;
    char buf[32];
    std::strncpy(buf, line, sizeof(buf) - 1); buf[sizeof(buf) - 1] = '\0';
    char* f[6];
    const int n = split(buf, f, 6);
    if (n < 4) return false;               // WIFI,<on>,<held_by>,<idle_left_s>[,stations]
    w.on = pl(f[1]) != 0;
    w.heldBy = static_cast<uint8_t>(pl(f[2]));
    w.idleLeftS = static_cast<uint32_t>(pl(f[3]));
    // The stations field was appended 2026-11-26 (the bench PC's own wlan0
    // proved a client cannot guess it): four-field lines from an in-between
    // controller parse with stations = -1, "not sent", never 0 — 0 claims
    // "nobody is connected", which that controller never said. A sixth field
    // a future controller appends is ignored here, the way the DS parser
    // honours whatever grows after `identity`.
    w.stations = n >= 5 ? static_cast<int16_t>(pl(f[4])) : -1;
    return true;
}

inline bool parseSchema(const char* line, SchemaEntry& s) {
    if (!starts(line, "K,")) return false;
    char buf[kRxLineCap];
    std::strncpy(buf, line, sizeof(buf) - 1); buf[sizeof(buf) - 1] = '\0';
    char* f[7];
    if (split(buf, f, 7) < 7) return false;
    std::strncpy(s.key, f[1], sizeof(s.key) - 1); s.key[sizeof(s.key) - 1] = '\0';
    s.type = f[2][0];
    s.minv = pf(f[3]); s.maxv = pf(f[4]);
    s.flags = static_cast<uint32_t>(pl(f[5]));
    s.def = 0.0f; s.defStr[0] = '\0';
    if (s.type == 'S') std::strncpy(s.defStr, f[6], sizeof(s.defStr) - 1);
    else s.def = pf(f[6]);
    return true;
}

// "KEY=value": key is [A-Za-z0-9_]+ then '='. Distinguishes settings GET
// answers from unknown tags.
inline bool parseSetting(const char* line, Setting& s) {
    const char* eq = std::strchr(line, '=');
    if (!eq || eq == line) return false;
    int i = 0;
    for (const char* p = line; p < eq && i < static_cast<int>(sizeof(s.key)) - 1; ++p, ++i) {
        const char c = *p;
        const bool ok = (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
                        (c >= '0' && c <= '9') || c == '_';
        if (!ok) return false;
        s.key[i] = c;
    }
    s.key[i] = '\0';
    std::strncpy(s.value, eq + 1, sizeof(s.value) - 1);
    s.value[sizeof(s.value) - 1] = '\0';
    return true;
}

// ---- the link ----------------------------------------------------------------

class Link {
public:
    using Writer = void (*)(const char* line, void* ctx);

    void begin(Writer put, void* putCtx, const Handlers& h) {
        put_ = put; putCtx_ = putCtx; h_ = h;
        len_ = 0; overflowing_ = false; lastRxMs_ = 0; lastCount_ = 0; linkUp_ = false;
        rxLines_ = 0; oobLines_ = 0; overflows_ = 0;
    }

    // ---- downlink: feed received bytes as they arrive ----
    void feedByte(int c) {
        if (c < 0) return;
        if (c == '\n' || c == '\r') {
            if (overflowing_) { overflowing_ = false; len_ = 0; return; }  // drop whole line
            if (len_) { buf_[len_] = '\0'; dispatch(buf_); len_ = 0; }
            return;
        }
        if (overflowing_) return;
        if (len_ < kRxLineCap - 1) buf_[len_++] = static_cast<char>(c);
        else { overflowing_ = true; overflows_++; len_ = 0; }
    }
    void feed(const void* data, size_t n) {
        const uint8_t* p = static_cast<const uint8_t*>(data);
        for (size_t i = 0; i < n; ++i) feedByte(p[i]);
    }
    void feedText(const char* s) { while (*s) feedByte(*s++); }

    // ---- link health: call periodically (e.g. every loop iteration).
    // feed() has no clock; activity is detected by the line counter moving
    // between ticks, timestamps come from the caller's monotonic clock.
    void tick(uint32_t nowMs) {
        if (rxLines_ != lastCount_) {
            lastCount_ = rxLines_;
            lastRxMs_ = nowMs;
            if (!linkUp_) {
                linkUp_ = true;
                if (h_.onLinkUp) h_.onLinkUp(h_.ctx);
            }
        }
        if (linkUp_ && nowMs - lastRxMs_ > linkTimeoutMs_) {
            linkUp_ = false;
            if (h_.onLinkDown) h_.onLinkDown(h_.ctx);
        }
    }
    void setLinkTimeout(uint32_t ms) { linkTimeoutMs_ = ms; }
    bool linkUp() const { return linkUp_; }
    uint32_t lastRxMs() const { return lastRxMs_; }
    uint32_t rxLines() const { return rxLines_; }
    uint32_t unknownLines() const { return oobLines_; }
    uint32_t overflows() const { return overflows_; }

    // ---- uplink: command builders. True = sent (false: line too long) ----
    bool send(const char* line) {
        if (std::strlen(line) > 64) return false;
        if (put_) put_(line, putCtx_);   // writer appends its own newline,
                                         // or use \r\n — controller accepts either
        return true;
    }
    bool get(const char* key)  { char b[40]; std::snprintf(b, sizeof b, "GET %s", key); return send(b); }
    bool getAll()              { return send("GET *"); }
    bool getT()                { return send("GET T"); }
    bool getD()                { return send("GET D"); }
    bool getPre()              { return send("GET PRE"); }
    bool getCells()            { return send("GET CELLS"); }
    bool getProof()            { return send("GET PROOF"); }
    bool getFault()            { return send("GET FAULT"); }
    bool getCalAir()           { return send("GET CALAIR"); }
    bool getHist()             { return send("GET HIST"); }
    bool getSchema()           { return send("GET SCHEMA"); }
    bool set(const char* key, const char* value) {
        char b[kTxLineCap]; std::snprintf(b, sizeof b, "SET %s=%s", key, value); return send(b);
    }
    bool check()               { return send("CHECK"); }
    bool setAuto()             { return send("AUTO"); }
    bool setManual()           { return send("MANUAL"); }
    bool ack()                 { return send("ACK"); }
    // `pulse()` was retired 2026-11-26 with the `PULSE` verb: the controller
    // has no handset-originated injection path (HANDSET_PROTOCOL.md §8). The
    // method is gone from the library so a new handset cannot grow the button.
    bool deco()                { return send("DECO"); }
    // CAL AIR: the reply is a receipt, not a verdict. Read the outcome with
    // getCalAir(), or watch the live `E,<t>,7,<code>` event.
    bool calAir()              { return send("CAL AIR"); }
    // Refused by the controller as things stand — `ERR not implemented` — because
    // nothing on the board performs them: the O2 sequencer has never been
    // instantiated, and no depth-offset trim exists. The commands stay in the
    // grammar (so no client has to change when the work lands) and the builders
    // stay for the same reason, but a handset must NOT draw a green tick from
    // these replies: show the refusal. There is no two-step arm to cancel.
    bool calO2Arm()            { return send("CAL O2"); }
    bool calO2Confirm()        { return send("CAL O2 CONFIRM"); }
    bool calDepth()            { return send("CAL DEPTH"); }
    bool testCells()           { return send("TEST CELLS"); }
    bool cellproofOverride()   { return send("CELLPROOF OVERRIDE"); }
    bool diveStart()           { return send("DIVE START"); }
    bool diveEnd()             { return send("DIVE END"); }
    bool logList()             { return send("LOG LIST"); }
    bool logExport(int slot)   { char b[24]; std::snprintf(b, sizeof b, "LOG EXPORT %d", slot); return send(b); }
    bool wifiOn()              { return send("WIFI ON"); }
    bool wifiOff()             { return send("WIFI OFF"); }
    bool getTime()             { return send("GET TIME"); }
    bool getDepthSource()      { return send("GET DS"); }
    bool getWifiState()        { return send("GET WIFI"); }
    // `TIME SET <epoch>`: the controller has no RTC, so this client IS the time
    // source (spec §3). Out-of-window epochs are refused HERE, with no bytes on
    // the wire: a handset whose own timekeeping has gone to soup should hear it
    // from its own code, not as an `ERR epoch out of range` it has to be
    // listening for. The controller refuses them too — this is a second gate,
    // not a replacement for the first. A handset with a real RTC should set the
    // clock at every link-up: an undated dive is honest but useless to a diver
    // who wants to know which day it was.
    bool timeSet(uint32_t epoch) {
        if (!saneEpoch(epoch)) return false;
        char b[32];
        std::snprintf(b, sizeof b, "TIME SET %lu", static_cast<unsigned long>(epoch));
        return send(b);
    }

private:
    void call(void (*fn)(const char*, void*), const char* a) { if (fn) fn(a, h_.ctx); }
    template <typename T>
    void callS(void (*fn)(const T&, void*), const T& a) { if (fn) fn(a, h_.ctx); }

    void dispatch(char* line) {
        rxLines_++;
        if (line[0] == 'T' && line[1] == ',') {
            Telemetry t;
            if (parseTelemetry(line, t)) { callS<Telemetry>(h_.onTelemetry, t); return; }
        } else if (line[0] == 'D' && line[1] == ',') {
            Deco d; if (parseDeco(line, d)) { callS<Deco>(h_.onDeco, d); return; }
        } else if (line[0] == 'E' && line[1] == ',') {
            Event e; if (parseEvent(line, e)) { callS<Event>(h_.onEvent, e); return; }
        } else if (std::strcmp(line, "OK") == 0) {
            call(h_.onOk, nullptr); return;
        } else if (starts(line, "OK ")) {
            call(h_.onOk, line + 3); return;
        } else if (starts(line, "ERR ")) {
            call(h_.onErr, line + 4); return;
        } else if (starts(line, "PRE,")) {
            PreStatus p; if (parsePre(line, p)) { callS<PreStatus>(h_.onPre, p); return; }
        } else if (starts(line, "CELLS,")) {
            CellsTest c; if (parseCells(line, c)) { callS<CellsTest>(h_.onCells, c); return; }
        } else if (starts(line, "PROOF,")) {
            Proof p; if (parseProof(line, p)) { callS<Proof>(h_.onProof, p); return; }
        } else if (starts(line, "CALAIR,")) {
            CalAir c; if (parseCalAir(line, c)) { callS<CalAir>(h_.onCalAir, c); return; }
        } else if (starts(line, "FAULT,")) {
            Fault f; if (parseFault(line, f)) { callS<Fault>(h_.onFault, f); return; }
        } else if (starts(line, "L,")) {
            LogEntry l; if (parseLogList(line, l)) { callS<LogEntry>(h_.onLogList, l); return; }
        } else if (starts(line, "DIVE,")) {
            DiveHeader h; if (parseDiveHeader(line, h)) { callS<DiveHeader>(h_.onDiveHeader, h); return; }
        } else if (starts(line, "TIME,")) {
            // Before the KEY=value fallback: a `TIME,` answer has no '=', so it
            // would otherwise be counted as an unknown line.
            Clock c; if (parseClock(line, c)) { callS<Clock>(h_.onClock, c); return; }
        } else if (starts(line, "DS,")) {
            // Before the KEY=value fallback (a `DS,` answer has no '='), and note
            // the `D,` test above cannot swallow it: that one checks for a comma
            // in position 1.
            DepthSource d;
            if (parseDepthSource(line, d)) { callS<DepthSource>(h_.onDepthSource, d); return; }
        } else if (starts(line, "WIFI,")) {
            // Before the KEY=value fallback: a `WIFI,` answer has no '=', and
            // note the SETTINGS key is WIFI_* never WIFI, — the comma test keeps
            // the two namespaces apart.
            WifiState w;
            if (parseWifiState(line, w)) { callS<WifiState>(h_.onWifiState, w); return; }
        } else if (starts(line, "K,")) {
            SchemaEntry s; if (parseSchema(line, s)) { callS<SchemaEntry>(h_.onSchema, s); return; }
        }
        Setting s;
        if (parseSetting(line, s)) { callS<Setting>(h_.onSetting, s); return; }
        oobLines_++;
        call(h_.onLine, line);
    }

    Writer put_ = nullptr;
    void* putCtx_ = nullptr;
    Handlers h_{};
    char buf_[kRxLineCap];
    size_t len_ = 0;
    bool overflowing_ = false;
    uint32_t lastRxMs_ = 0;
    uint32_t lastCount_ = 0;
    uint32_t linkTimeoutMs_ = kLinkTimeoutMs;
    bool linkUp_ = false;
    uint32_t rxLines_ = 0, oobLines_ = 0, overflows_ = 0;
};

}  // namespace hs
