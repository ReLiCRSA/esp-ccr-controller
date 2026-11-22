// ============================================================================
// uart_proto.cpp — defensive parsing: every malformed line is an ERR with a
// reason, never a crash, never a silent accept.
// ============================================================================
#include "uart_proto.h"

#include <cctype>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace proto {

namespace {

char* tok(char* s, const char** rest) {
    // split next space-delimited token in place
    while (*s == ' ' || *s == '\t') ++s;
    char* head = s;
    while (*s && *s != ' ' && *s != '\t') ++s;
    if (*s) { *s = '\0'; *rest = s + 1; } else { *rest = s; }
    return head;
}

void copyField(char* dst, size_t cap, const char* src) {
    size_t i = 0;
    for (; src[i] && i + 1 < cap; ++i) dst[i] = src[i];
    dst[i] = '\0';
}

int emit(char* out, size_t cap, const char* fmt, ...) {
    va_list ap; va_start(ap, fmt);
    const int n = std::vsnprintf(out, cap, fmt, ap);
    va_end(ap);
    return n;
}

bool validKey(const char* k) {
    if (!*k) return false;
    for (const char* p = k; *p; ++p) {
        if (!(std::isalnum(static_cast<unsigned char>(*p)) || *p == '_')) return false;
    }
    return true;
}

} // namespace

bool parseCommand(const char* line, Command& out, const char** err) {
    out = Command{};
    if (!line) { *err = "empty"; return false; }

    char buf[MAX_LINE + 1];
    const size_t len = std::strlen(line);
    if (len > MAX_LINE) { *err = "line too long"; return false; }
    std::memcpy(buf, line, len + 1);
    // trim trailing CR/LF and spaces
    size_t n = len;
    while (n && (buf[n - 1] == '\r' || buf[n - 1] == '\n' || buf[n - 1] == ' ' ||
                 buf[n - 1] == '\t')) buf[--n] = '\0';
    if (n == 0) { *err = "empty"; return false; }

    const char* rest = buf;
    char* verb = tok(buf, &rest);

    if (std::strcmp(verb, "GET") == 0) {
        out.kind = Kind::GET;
        if (*rest) copyField(out.key, sizeof(out.key), const_cast<char*>(tok(const_cast<char*>(rest), &rest)));
        else copyField(out.key, sizeof(out.key), "*");
        return true;
    }
    if (std::strcmp(verb, "SET") == 0) {
        out.kind = Kind::SET;
        if (!*rest) { *err = "SET needs KEY=VALUE"; return false; }
        // The VALUE is the REST OF THE LINE (passphrases/SSIDs may contain
        // spaces). The KEY is everything before the first '=' (a space in
        // the key fails validKey below).
        char* kv = const_cast<char*>(rest);
        char* eq = std::strchr(kv, '=');
        if (!eq) { *err = "SET needs '='"; return false; }
        *eq = '\0';
        size_t klen = std::strlen(kv);
        while (klen && (kv[klen - 1] == ' ' || kv[klen - 1] == '\t')) kv[--klen] = '\0';
        copyField(out.key, sizeof(out.key), kv);
        const char* val = eq + 1;
        while (*val == ' ' || *val == '\t') ++val;
        if (!*val) { *err = "SET empty value"; return false; }
        copyField(out.value, sizeof(out.value), val);
        if (!validKey(out.key)) { *err = "bad key"; return false; }
        return true;
    }
    if (std::strcmp(verb, "CHECK") == 0) { out.kind = Kind::CHECK; return true; }
    if (std::strcmp(verb, "AUTO") == 0) { out.kind = Kind::AUTO; return true; }
    if (std::strcmp(verb, "MANUAL") == 0) { out.kind = Kind::MANUAL; return true; }
    if (std::strcmp(verb, "ACK") == 0) { out.kind = Kind::ACK; return true; }
    if (std::strcmp(verb, "PULSE") == 0) { out.kind = Kind::PULSE; return true; }
    if (std::strcmp(verb, "DECO") == 0) { out.kind = Kind::DECO; return true; }

    if (std::strcmp(verb, "CAL") == 0) {
        out.kind = Kind::CAL;
        if (!*rest) { *err = "CAL needs target"; return false; }
        char* a = tok(const_cast<char*>(rest), &rest);
        if (std::strcmp(a, "AIR") == 0) out.cal = CalAction::AIR;
        else if (std::strcmp(a, "O2") == 0) {
            if (*rest && std::strcmp(tok(const_cast<char*>(rest), &rest), "CONFIRM") == 0)
                out.cal = CalAction::O2_CONFIRM;
            else out.cal = CalAction::O2; // first step of two-step confirm
        }
        else if (std::strcmp(a, "DEPTH") == 0) out.cal = CalAction::DEPTH;
        else { *err = "unknown CAL target"; return false; }
        return true;
    }
    if (std::strcmp(verb, "TEST") == 0) {
        if (!*rest || std::strcmp(tok(const_cast<char*>(rest), &rest), "CELLS") != 0) {
            *err = "TEST needs CELLS"; return false;
        }
        out.kind = Kind::TESTCELLS;
        return true;
    }
    if (std::strcmp(verb, "CELLPROOF") == 0) {
        if (!*rest || std::strcmp(tok(const_cast<char*>(rest), &rest), "OVERRIDE") != 0) {
            *err = "CELLPROOF needs OVERRIDE"; return false;
        }
        out.kind = Kind::CELLPROOF_OVERRIDE;
        return true;
    }
    if (std::strcmp(verb, "DIVE") == 0) {
        if (!*rest) { *err = "DIVE needs START/END"; return false; }
        char* a = tok(const_cast<char*>(rest), &rest);
        if (std::strcmp(a, "START") == 0) { out.kind = Kind::DIVE_START; return true; }
        if (std::strcmp(a, "END") == 0) { out.kind = Kind::DIVE_END; return true; }
        *err = "unknown DIVE action"; return false;
    }
    if (std::strcmp(verb, "LOG") == 0) {
        if (!*rest) { *err = "LOG needs LIST/EXPORT"; return false; }
        char* a = tok(const_cast<char*>(rest), &rest);
        if (std::strcmp(a, "LIST") == 0) { out.kind = Kind::LOG_LIST; return true; }
        if (std::strcmp(a, "EXPORT") == 0) {
            if (!*rest) { *err = "LOG EXPORT needs index"; return false; }
            char* idx = tok(const_cast<char*>(rest), &rest);
            char* endp = nullptr;
            const long v = std::strtol(idx, &endp, 10);
            if (endp == idx || *endp || v < 0 || v > 9) { *err = "bad dive index"; return false; }
            out.kind = Kind::LOG_EXPORT;
            out.logExport = true;
            out.logIndex = static_cast<uint16_t>(v);
            return true;
        }
        *err = "unknown LOG action"; return false;
    }
    if (std::strcmp(verb, "WIFI") == 0) {
        if (!*rest) { *err = "WIFI needs ON/OFF"; return false; }
        char* a = tok(const_cast<char*>(rest), &rest);
        if (std::strcmp(a, "ON") == 0 || std::strcmp(a, "OFF") == 0) {
            out.kind = Kind::WIFI;
            copyField(out.key, sizeof(out.key), a);
            return true;
        }
        *err = "WIFI wants ON or OFF"; return false;
    }

    out.kind = Kind::UNKNOWN;
    *err = "unknown command";
    return false;
}

size_t formatTelemetry(const Telemetry& t, char* out, size_t cap) {
    static const char* kMode[] = {"STANDBY", "AUTO", "MANUAL", "SAFE"};
    const int n = emit(out, cap, "T,%.2f,%.3f,%.3f,%.3f,%.3f,%.3f,%s,FLAGS0x%02X,%.2f\n",
                       static_cast<double>(t.depthM), static_cast<double>(t.votedPpo2),
                       static_cast<double>(t.cells[0]), static_cast<double>(t.cells[1]),
                       static_cast<double>(t.cells[2]), static_cast<double>(t.setpoint),
                       kMode[t.mode & 3], t.flags, static_cast<double>(t.battV));
    return (n > 0 && static_cast<size_t>(n) < cap) ? static_cast<size_t>(n) : 0;
}

size_t formatDeco(const DecoInfo& d, char* out, size_t cap) {
    int n = emit(out, cap, "D,%d,%.1f,%.2f", d.ndlMin,
                 static_cast<double>(d.decoTotalMin), static_cast<double>(d.ceilingM));
    if (n <= 0 || static_cast<size_t>(n) >= cap) return 0; // vsnprintf returns
                                                           // NEEDED length:
                                                           // guard before cap-n
    for (int i = 0; i < d.stopCount && i < MAX_STOPS; ++i) {
        const int m = emit(out + n, cap - n, ",%.0f|%.1f",
                           static_cast<double>(d.stops[i].depthM),
                           static_cast<double>(d.stops[i].minutes));
        if (m <= 0 || static_cast<size_t>(n + m) >= cap) return 0;
        n += m;
    }
    const int m = emit(out + n, cap - n, ",%.1f\n", static_cast<double>(d.cnsPct));
    if (m <= 0 || static_cast<size_t>(n + m) >= cap) return 0;
    return static_cast<size_t>(n + m);
}

size_t formatEvent(uint32_t tSec, uint16_t id, uint16_t value, char* out, size_t cap) {
    const int n = emit(out, cap, "E,%u,%u,%u\n", tSec, id, value);
    return (n > 0 && static_cast<size_t>(n) < cap) ? static_cast<size_t>(n) : 0;
}

size_t formatErr(const char* reason, char* out, size_t cap) {
    const int n = emit(out, cap, "ERR %s\n", reason ? reason : "?");
    return (n > 0 && static_cast<size_t>(n) < cap) ? static_cast<size_t>(n) : 0;
}

} // namespace proto
