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

struct Setting { char key[24]; char value[40]; };      // "KEY=value" answers
struct LogEntry { int slot; uint32_t epoch; };
struct DiveHeader { int slot; uint32_t startEpoch; int version; };

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
    char* f[3];
    if (split(buf, f, 3) < 3) return false;
    e.slot = static_cast<int>(pl(f[1]));
    e.epoch = static_cast<uint32_t>(pl(f[2]));
    return true;
}

inline bool parseDiveHeader(const char* line, DiveHeader& h) {
    if (!starts(line, "DIVE,")) return false;
    char buf[64];
    std::strncpy(buf, line, sizeof(buf) - 1); buf[sizeof(buf) - 1] = '\0';
    char* f[4];
    if (split(buf, f, 4) < 4) return false;
    h.slot = static_cast<int>(pl(f[1]));
    h.startEpoch = static_cast<uint32_t>(pl(f[2]));
    h.version = static_cast<int>(pl(f[3]));
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
    bool getHist()             { return send("GET HIST"); }
    bool getSchema()           { return send("GET SCHEMA"); }
    bool set(const char* key, const char* value) {
        char b[kTxLineCap]; std::snprintf(b, sizeof b, "SET %s=%s", key, value); return send(b);
    }
    bool check()               { return send("CHECK"); }
    bool setAuto()             { return send("AUTO"); }
    bool setManual()           { return send("MANUAL"); }
    bool ack()                 { return send("ACK"); }
    bool pulse()               { return send("PULSE"); }
    bool deco()                { return send("DECO"); }
    bool calAir()              { return send("CAL AIR"); }
    bool calO2Arm()            { return send("CAL O2"); }        // bare CAL O2 again = cancel
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
        } else if (starts(line, "FAULT,")) {
            Fault f; if (parseFault(line, f)) { callS<Fault>(h_.onFault, f); return; }
        } else if (starts(line, "L,")) {
            LogEntry l; if (parseLogList(line, l)) { callS<LogEntry>(h_.onLogList, l); return; }
        } else if (starts(line, "DIVE,")) {
            DiveHeader h; if (parseDiveHeader(line, h)) { callS<DiveHeader>(h_.onDiveHeader, h); return; }
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
