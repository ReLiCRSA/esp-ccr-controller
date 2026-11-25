// ============================================================================
// settings.cpp — definition table + validation. Every number that guards a
// diver has an explicit range here; a SET outside it is refused with a
// reason, on handset and web alike.
// ============================================================================
#include "settings.h"

#include "util/numfmt.h"

#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace settings {

namespace {

constexpr size_t TCOUNT = 36;

// key                  type     default  min      max      flags        str-default
const Def kTable[TCOUNT] = {
    // setpoints
    {"SP1", Type::FLOAT, 0.40f, 0.20f, 1.60f, FL_SAFETY | FL_DIVE_LOCK, nullptr},
    {"SP2", Type::FLOAT, 0.70f, 0.20f, 1.60f, FL_SAFETY | FL_DIVE_LOCK, nullptr},
    {"SP3", Type::FLOAT, 1.00f, 0.20f, 1.60f, FL_SAFETY | FL_DIVE_LOCK, nullptr},
    {"SP4", Type::FLOAT, 1.30f, 0.20f, 1.60f, FL_SAFETY | FL_DIVE_LOCK, nullptr},
    {"D1", Type::FLOAT, 30.0f, 5.0f, 100.0f, FL_NONE, nullptr},
    {"D2", Type::FLOAT, 50.0f, 10.0f, 150.0f, FL_NONE, nullptr},
    {"SP_HYST", Type::FLOAT, 0.5f, 0.1f, 3.0f, FL_NONE, nullptr},
    // alarms (defaults match firmware: alarm 1.40 < injection wall 1.45 < cutoff 1.60)
    {"ALLO_LOW", Type::FLOAT, 0.18f, 0.05f, 0.40f, FL_SAFETY | FL_DIVE_LOCK, nullptr},
    {"ALLO_HIGH", Type::FLOAT, 1.40f, 1.00f, 1.60f, FL_SAFETY | FL_DIVE_LOCK, nullptr},
    {"ALLO_CUTOFF", Type::FLOAT, 1.60f, 1.50f, 1.80f, FL_SAFETY | FL_DIVE_LOCK, nullptr},
    {"CNS_ALARM", Type::UINT8, 80.0f, 10.0f, 100.0f, FL_SAFETY | FL_DIVE_LOCK, nullptr},
    // voting
    {"CELL_DRIFT_BAR", Type::FLOAT, 0.05f, 0.01f, 0.30f, FL_NONE, nullptr},
    // injection
    {"INJ_PULSE_MS", Type::INT16, 300.0f, 50.0f, 1000.0f, FL_SAFETY | FL_DIVE_LOCK, nullptr},
    {"INJ_MAX_DUTY", Type::UINT8, 10.0f, 1.0f, 25.0f, FL_SAFETY | FL_DIVE_LOCK, nullptr},
    // calibration
    {"CAL_O2_FI", Type::FLOAT, 0.99f, 0.85f, 1.00f, FL_NONE, nullptr},
    {"CAL_O2_TARGET", Type::FLOAT, 0.95f, 0.80f, 1.00f, FL_NONE, nullptr},
    {"CAL_O2_MAX_PPO2", Type::FLOAT, 1.00f, 0.90f, 1.10f, FL_SAFETY, nullptr},
    {"CAL_SLOPE_BAND", Type::UINT8, 15.0f, 5.0f, 50.0f, FL_NONE, nullptr},
    // cellproof
    {"CELLPROOF_EN", Type::BOOL, 1.0f, 0.0f, 1.0f, FL_NONE, nullptr},
    {"CELLPROOF_DEPTH", Type::FLOAT, 10.0f, 3.0f, 30.0f, FL_NONE, nullptr},
    {"CELLPROOF_SP", Type::FLOAT, 1.10f, 1.00f, 1.40f, FL_SAFETY | FL_DIVE_LOCK, nullptr},
    {"CELLPROOF_MIN", Type::FLOAT, 1.00f, 0.90f, 1.30f, FL_SAFETY | FL_DIVE_LOCK, nullptr},
    {"CELLPROOF_TMO", Type::UINT8, 60.0f, 20.0f, 120.0f, FL_NONE, nullptr},
    {"CELLPROOF_MAXSP", Type::FLOAT, 0.90f, 0.60f, 1.00f, FL_SAFETY | FL_DIVE_LOCK, nullptr},
    // deco
    {"GF_LOW", Type::UINT8, 30.0f, 10.0f, 95.0f, FL_SAFETY, nullptr},
    {"GF_HIGH", Type::UINT8, 85.0f, 50.0f, 100.0f, FL_SAFETY, nullptr},
    {"ASC_RATE", Type::FLOAT, 6.0f, 3.0f, 18.0f, FL_SAFETY, nullptr},
    {"ASC_RATE_SLOW", Type::FLOAT, 1.0f, 0.5f, 3.0f, FL_SAFETY, nullptr},
    // The depth ruler: dive-locked since 2026-11-24, and that flag is not
    // decoration. One density now drives the depth in the `T,` row, the setpoint
    // tiers those metres select, the cellproof arm depth AND the deco engine's
    // ambient pressures, so a mid-dive write used to move the diver's whole
    // depth scale ~2.5% under his feet (0.990..1.100 is the legal range) while
    // the physical pressure stayed put. Setpoints and alarms are locked for less
    // than that. The refusal is graded on hardware (hw_accept matrix).
    {"WATER_DENSITY", Type::FLOAT, 1.025f, 0.990f, 1.100f, FL_DIVE_LOCK, nullptr},
    // battery
    {"VBAT_MIN", Type::FLOAT, 10.5f, 3.0f, 30.0f, FL_SAFETY, nullptr},
    // dive lock: everything that steers the gas loop or the sensing verdicts
    // is frozen once a dive starts (the router enforces it for handset AND
    // web). Tuning that only moves the schedule (D1/D2, SP_HYST, drift,
    // CAL_*, GF_*, ascent rates) stays writable: GF adjustments mid-dive are
    // normal diving, re-writing the oxygen setpoints underneath a diver is not.
    // system (strings)
    {"AP_SSID", Type::STRING, 0, 1, 16, FL_DIVE_LOCK, "ESP-CCR"},
    {"AP_PASS", Type::STRING, 0, 8, 31, FL_DIVE_LOCK, "diveonly"},
    // radio policy (2026-11-26). These tune ONE state machine — CommandRouter's
    // wifi policy, answered on the wire by GET WIFI and printed on the console
    // at every transition. WIFI_IDLE_MIN 0 means "never auto-off": the AP stays
    // up with nobody connected, which is what a diver on a boat may want and what
    // a pack on the bench does not. WIFI_ON_DEPTH must stay below WIFI_OFF_DEPTH
    // or the hold oscillates (released below ON, re-set above OFF, every
    // WIFI_ON_HOLD_S) — the schema cannot express key-vs-key bounds, so this one
    // sentence is the guard, and test_the_wifi_policy_oscillates_by_design records
    // what an inversion does rather than pretending it cannot happen.
    {"WIFI_IDLE_MIN", Type::UINT8, 10.0f, 0.0f, 240.0f, FL_NONE, nullptr},
    {"WIFI_OFF_DEPTH", Type::FLOAT, 2.0f, 0.5f, 30.0f, FL_NONE, nullptr},
    {"WIFI_ON_DEPTH", Type::FLOAT, 1.0f, 0.2f, 20.0f, FL_NONE, nullptr},
    {"WIFI_ON_HOLD_S", Type::UINT8, 60.0f, 0.0f, 240.0f, FL_NONE, nullptr},
};

static_assert(sizeof(kTable) / sizeof(kTable[0]) == TCOUNT, "TCOUNT drift");
static_assert(TCOUNT == Registry::NDEFS, "header NDEFS out of sync with table");

} // namespace

const Def* Registry::definitions(size_t& count) {
    count = TCOUNT;
    return kTable;
}

const Def* Registry::table() { return kTable; }

void Registry::resetToDefaults() {
    for (size_t s = 0; s < MAX_STRINGS; ++s) m_strs[s][0] = '\0';
    int strSlot = 0;
    for (size_t i = 0; i < TCOUNT; ++i) {
        const Def& d = kTable[i];
        m_strSlot[i] = -1;
        if (d.type == Type::STRING) {
            m_strSlot[i] = strSlot;
            std::snprintf(m_strs[strSlot++], MAX_VAL_STR, "%s", d.defStr ? d.defStr : "");
        } else {
            m_num[i] = d.def;
        }
    }
}

int Registry::find(const char* key) const {
    for (size_t i = 0; i < TCOUNT; ++i)
        if (std::strcmp(kTable[i].key, key) == 0) return static_cast<int>(i);
    return -1;
}

bool Registry::setValue(int idx, const char* text, const char** err) {
    if (idx < 0 || idx >= static_cast<int>(TCOUNT)) { *err = "unknown key"; return false; }
    const Def& d = kTable[idx];
    if (!text || !*text) { *err = "empty value"; return false; }

    if (d.type == Type::STRING) {
        if (std::strlen(text) > MAX_VAL_STR - 1) { *err = "value too long"; return false; }
        if (std::strlen(text) < d.minv) { *err = "value too short"; return false; }
        std::snprintf(m_strs[m_strSlot[idx]], MAX_VAL_STR, "%s", text);
        return true;
    }

    if (d.type == Type::BOOL) {
        char up[8];
        size_t n = 0;
        for (; text[n] && n < 7; ++n) up[n] = static_cast<char>(std::toupper(static_cast<unsigned char>(text[n])));
        up[n] = '\0';
        if (std::strcmp(up, "ON") == 0 || std::strcmp(up, "YES") == 0 ||
            std::strcmp(up, "TRUE") == 0 || std::strcmp(up, "1") == 0) { m_num[idx] = 1; return true; }
        if (std::strcmp(up, "OFF") == 0 || std::strcmp(up, "NO") == 0 ||
            std::strcmp(up, "FALSE") == 0 || std::strcmp(up, "0") == 0) { m_num[idx] = 0; return true; }
        *err = "wants ON/OFF";
        return false;
    }

    char* endp = nullptr;
    // Parse AND compare in float, the type the schema and the storage both use.
    // Parsing with strtod and comparing against a `float` bound promotes the
    // bound to double, which moves it by up to one ULP of its own — and the
    // schema's own endpoints are not exact binary values. Measured on
    // 2026-11-24: `SET WATER_DENSITY=0.990` answered `ERR out of range` against a
    // declared minimum of 0.990, because 0.990f is 0.99000000953674316 and the
    // double 0.990 is below it. Ten keys had their declared MIN unreachable
    // (SP1..SP4 at 0.20, SP_HYST 0.1, ALLO_LOW 0.05, CAL_O2_FI 0.85, CAL_O2_TARGET
    // 0.80, CELLPROOF_MAXSP 0.60, WATER_DENSITY 0.990) and three their MAX
    // (ALLO_CUTOFF 1.80, CELLPROOF_SP 1.40, CELLPROOF_MIN 1.30) — 13 of the 30
    // numeric keys. The schema promised an inclusive range and delivered an
    // exclusive one at those edges, on the very endpoints a diver or the page
    // would type. The bench found it by accident: a dive-lock check that used the
    // boundary value got the right `ERR` for the wrong reason, which is also a
    // lesson for refusal checks — never pick a value that a second refusal rule
    // could produce.
    const float v = std::strtof(text, &endp);
    if (endp == text || (endp && *endp != '\0' && *endp != ' ')) { *err = "not a number"; return false; }
    if (v < d.minv || v > d.maxv) { *err = "out of range"; return false; }
    if ((d.type == Type::INT16 || d.type == Type::UINT8) &&
        v != static_cast<float>(static_cast<long>(v))) { *err = "needs integer"; return false; }
    m_num[idx] = static_cast<float>(v);
    return true;
}

bool Registry::setValueByKey(const char* key, const char* text, const char** err) {
    const int idx = find(key);
    if (idx < 0) { *err = "unknown key"; return false; }
    return setValue(idx, text, err);
}

float Registry::getFloat(int idx) const { return m_num[idx]; }
int Registry::getInt(int idx) const { return static_cast<int>(m_num[idx] + (m_num[idx] >= 0 ? 0.5f : -0.5f)); }
bool Registry::getBool(int idx) const { return m_num[idx] >= 0.5f; }

const char* Registry::getString(int idx) const {
    return m_strSlot[idx] < 0 ? "" : m_strs[m_strSlot[idx]];
}

bool Registry::valueAsString(int idx, char* out, size_t cap) const {
    const Def& d = kTable[idx];
    int n = 0;
    switch (d.type) {
        case Type::STRING: n = std::snprintf(out, cap, "%s", getString(idx)); break;
        case Type::BOOL:   n = std::snprintf(out, cap, "%s", getBool(idx) ? "ON" : "OFF"); break;
        case Type::INT16:
        case Type::UINT8:  n = std::snprintf(out, cap, "%d", getInt(idx)); break;
        case Type::FLOAT: {
            // no %.3f: see util/numfmt.h (float printf is what panicked the
            // uart2 task); MAX_VAL_STR is roomy for every float in the table.
            char b[MAX_VAL_STR];
            numfmt::fixed(b, sizeof(b), m_num[idx], 3);
            n = std::snprintf(out, cap, "%s", b);
            break;
        }
    }
    return n > 0 && static_cast<size_t>(n) < cap;
}

size_t Registry::listAll(char* out, size_t cap) const {
    char* p = out;
    size_t rem = cap;
    for (size_t i = 0; i < TCOUNT; ++i) {
        char val[MAX_VAL_STR];
        if (!valueAsString(static_cast<int>(i), val, sizeof(val))) return 0;
        const int n = std::snprintf(p, rem, "%s=%s\n", kTable[i].key, val);
        if (n < 0 || static_cast<size_t>(n) >= rem) return 0;
        p += n;
        rem -= static_cast<size_t>(n);
    }
    return static_cast<size_t>(p - out);
}

} // namespace settings
