// ============================================================================
// uart_proto.h — handset wire grammar (PLAN.md 8). Pure parse/format so the
// protocol is unit-testable on the host and reusable by the simulator.
//
// Downlink:  T,<depth>,<voted>,<c0>,<c1>,<c2>,<sp>,<mode>,<flags>
//            D,<ndl_min>,<deco_total_min>,<ceiling_m>,<d>|<t>,...,<cns_pct>
//            E,<t_s>,<event_id>,<value>          (on change)
//            OK | ERR <reason>                   (command replies)
// Uplink:    GET [*|KEY] | SET KEY=VAL | CHECK | AUTO | MANUAL | ACK |
//            CAL AIR | CAL O2 | CAL O2 CONFIRM | CAL DEPTH | TEST CELLS |
//            CELLPROOF OVERRIDE | PULSE | DECO | DIVE START|END |
//            LOG LIST | LOG EXPORT n | WIFI ON|OFF
// ============================================================================
#pragma once

#include <cstddef>
#include <cstdint>

namespace proto {

constexpr size_t MAX_LINE = 64;      // longer uplink lines are rejected
constexpr int MAX_STOPS = 8;

enum class Kind : uint8_t {
    GET, SET, CHECK, AUTO, MANUAL, ACK, CAL, TESTCELLS, CELLPROOF_OVERRIDE,
    PULSE, DECO, DIVE_START, DIVE_END, LOG_LIST, LOG_EXPORT, WIFI, UNKNOWN,
};

enum class CalAction : uint8_t { NONE, AIR, O2, O2_CONFIRM, DEPTH };

struct Command {
    Kind kind = Kind::UNKNOWN;
    char key[24] = {0};       // GET/SET key, WIFI arg ("ON"/"OFF"), LOG arg
    char value[24] = {0};     // SET raw value text (registry validates)
    CalAction cal = CalAction::NONE;
    bool logExport = false;   // LOG EXPORT vs LOG LIST
    uint16_t logIndex = 0;
};

// Returns false and sets *err (static string) on malformed input.
bool parseCommand(const char* line, Command& out, const char** err);

// ---- downlink formatters: return length written, 0 on overflow ----
struct Telemetry {
    float depthM;
    float votedPpo2;
    float cells[3];           // raw cell estimates even when excluded
    float setpoint;
    uint8_t mode;             // 0 standby 1 auto 2 manual 3 safe
    uint8_t flags;            // alarm bits (bit0 low, bit1 high, bit2 drift,
                              // bit3 sensor, bit4 battery, bit5 clamp, bit6 cellproof)
    float battV = 0.0f;       // appended last: a client that stops at FLAGS
                              // is still correct (PLAN 8 downlink)
};
size_t formatTelemetry(const Telemetry& t, char* out, size_t cap);

struct DecoInfo {
    int ndlMin;               // 480 = effectively unlimited
    float decoTotalMin;
    float ceilingM;
    struct { float depthM; float minutes; } stops[MAX_STOPS];
    int stopCount;
    float cnsPct;
};
size_t formatDeco(const DecoInfo& d, char* out, size_t cap);

size_t formatEvent(uint32_t tSec, uint16_t id, uint16_t value, char* out, size_t cap);

inline size_t formatOk(char* out, size_t cap) {
    if (cap < 3) return 0;
    out[0] = 'O'; out[1] = 'K'; out[2] = '\n';
    // NUL-terminated when there is room. Until 2026-11-23 this did not matter:
    // the router built its Response with `Response r{}`, which zeroed all 1024
    // bytes, so the byte after every reply was already '\0'. The reply buffer is
    // now REUSED (see cmd_router.h: a fresh 1 KB per command was what overflowed
    // the uart2 task), so leftovers from the previous reply are live memory and
    // a caller printing the buffer as a C string would emit them. The router's
    // transports sink r.text as a string, so terminating is a wire correctness
    // property, not tidiness; test_router asserts strlen(text)==len after every
    // command on one reused buffer.
    if (cap >= 4) out[3] = '\0';
    return 3;
}
size_t formatErr(const char* reason, char* out, size_t cap);

} // namespace proto
