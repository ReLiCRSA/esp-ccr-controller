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
//            CELLPROOF OVERRIDE | DECO | DIVE START|END |
//            LOG LIST | LOG EXPORT n | WIFI ON|OFF | TIME SET <epoch>
// There is no RTC on the board and no NTP through an AP with no uplink, so wall
// time is handed to the controller by a client at the surface. It is a command,
// not a settings key: a settings key lives in NVS, and resurrecting a saved
// epoch after a battery pull would date every dive in the log with a number
// that is quietly wrong.
// ============================================================================
#pragma once

#include <cstddef>
#include <cstdint>

namespace proto {

constexpr size_t MAX_LINE = 64;      // longer uplink lines are rejected
// SET value text. This is NOT a free choice: it must be at least as long as the
// longest value the SETTINGS registry can accept, or the parser silently hands
// the registry a shorter string than the client sent and the write lands
// truncated while still answering `OK`. It was exactly that until 2026-11-24:
// this field was 24 bytes (23 chars) while `AP_PASS` publishes a range of 8..31,
// so a 24..31 character passphrase was legal, answered `OK`, and stored short —
// the diver then could not join the access point he had just configured. The
// string bounds are integer length checks, so unlike the numeric endpoints (see
// settings.cpp, same day) this was never masked by anything: it was live the
// whole time. The
// client library's own field is 40 bytes (`Setting::value`), so a handset was
// never the limiting factor. Pinned by a static_assert in cmd_router.cpp and by a
// schema walk in test_proto, because either side drifting reopens the lie.
// Budget check: the longest legal line is `SET AP_PASS=` + 31 = 43 chars, inside
// MAX_LINE, so this cannot make an acceptable line overflow the reader.
constexpr size_t MAX_VALUE = 40;
constexpr int MAX_STOPS = 8;

enum class Kind : uint8_t {
    GET, SET, CHECK, AUTO, MANUAL, ACK, CAL, TESTCELLS, CELLPROOF_OVERRIDE,
    // `PULSE` was retired 2026-11-26 (docs/HANDSET_PROTOCOL.md §8): the verb
    // no longer parses, so it answers `ERR unknown command`. Its enumerator is
    // gone rather than left as a stub — a manual valve command on the handset
    // is a decision this project has made against, and an enum value that
    // nothing refuses is a value something eventually wires up again.
    DECO, DIVE_START, DIVE_END, LOG_LIST, LOG_EXPORT, WIFI, TIME_SET,
    UNKNOWN,
};

enum class CalAction : uint8_t { NONE, AIR, O2, O2_CONFIRM, DEPTH };

struct Command {
    Kind kind = Kind::UNKNOWN;
    char key[24] = {0};       // GET/SET key, WIFI arg ("ON"/"OFF"), LOG arg.
                              // 24 matches the client library's SchemaEntry::key,
                              // and every published key is <= settings::MAX_KEY.
    char value[MAX_VALUE] = {0};   // SET raw value text (registry validates)
    CalAction cal = CalAction::NONE;
    bool logExport = false;   // LOG EXPORT vs LOG LIST
    uint16_t logIndex = 0;
    uint32_t epoch = 0;       // TIME SET: the digits as a number; the router
                              // decides whether the value is believable
};

// Returns false and sets *err (static string) on malformed input.
bool parseCommand(const char* line, Command& out, const char** err);

// ---- uplink line assembly --------------------------------------------------
// Bytes in, one whole command line out. This used to be written inline in
// uart2_task.cpp, which meant the native suite could not see the rule at all —
// and the rule was wrong there (2026-11-23, PLAN 16): on overflow the old code
// reset the buffer but kept reading, so the REST of the physical line was
// terminated as a second command; and when the ERR fell inside its own rate
// limit window the buffer was not reset at all, so the terminator dispatched the
// TRUNCATED first 64 bytes as a command. Bench-observed: an over-long
// `SET SP2=0.33` + 120 junk bytes answered `OK SAFETY` and wrote 0.330. Framing
// decides whether the router sees a command at all, so this is not cosmetic.
//
// The rule, now in one host-tested place: an over-long line is DROPPED WHOLE —
// the rate limit gates the error TEXT, never the recovery. Same verdict the
// handset client library already applies to itself (`overflowing_`) and the same
// verdict the web transport already takes (a whole WS frame is discarded).
class LineReader {
public:
    enum class Out : uint8_t { NOTHING, LINE, TOO_LONG };

    /// Feed one received byte. `nowMs` is the CALLER's clock: the reader owns
    /// its own rate limit so the byte mover never has to time anything.
    Out push(int c, uint32_t nowMs);

    const char* line() const { return buf_; }      // valid after LINE
    const char* errText() const { return err_; }   // valid after TOO_LONG

    // Error text for one over-long line is emitted at most once per this long,
    // so a wall of junk cannot storm a 6 KB-stack task with error replies.
    static constexpr uint32_t kErrGapMs = 2000;

private:
    char buf_[MAX_LINE + 1] = {0};
    char err_[48] = {0};
    size_t len_ = 0;
    bool swallowing_ = false;   // rest of this line is on the floor
    bool haveErrAt_ = false;
    uint32_t errAt_ = 0;
};

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
