// ============================================================================
// settings.h — the single key/value registry shared by handset and web
// (PLAN.md 11). Definitions are a compile-time table: one place to add a
// key, one place range checks live, one place dive-lock flags live.
// The NVS/LittleFS persistence adapter wraps load()/save() around this and
// is added when we're back on hardware; logic here stays host-testable.
// ============================================================================
#pragma once

#include <cstddef>
#include <cstdint>

namespace settings {

constexpr size_t MAX_KEY = 16;
constexpr size_t MAX_VAL_STR = 32;

enum class Type : uint8_t { FLOAT, INT16, UINT8, BOOL, STRING };

enum DefFlags : uint8_t {
    FL_NONE = 0,
    FL_DIVE_LOCK = 1 << 0, // writes refused while a dive is active
    FL_SAFETY = 1 << 1,    // alarms/limits: changes are event-logged
};

struct Def {
    const char* key;
    Type type;
    float def;            // numeric default (ignored for STRING)
    float minv, maxv;     // validation range (STRING: length only)
    uint8_t flags;
    const char* defStr;   // STRING default (nullptr otherwise)
};

class Registry {
public:
    static constexpr size_t NDEFS = 32; // keep in sync via static_assert in .cpp
    static const Def* definitions(size_t& count);

    void resetToDefaults();

    int find(const char* key) const; // -1 unknown

    // SET path used by handset and web alike: parse + validate by type.
    bool setValue(int idx, const char* text, const char** err);
    bool setValueByKey(const char* key, const char* text, const char** err);

    // Getters (numeric values normalized: BOOL 0/1, INT rounded).
    float getFloat(int idx) const;
    int getInt(int idx) const;
    bool getBool(int idx) const;
    const char* getString(int idx) const;
    const Def& def(int idx) const { return table()[idx]; }

    // value(index) -> text for GET replies
    bool valueAsString(int idx, char* out, size_t cap) const;

    // GET * body: "KEY=val\nKEY=val\n..." returns chars written (0=overflow)
    size_t listAll(char* out, size_t cap) const;

    bool isDiveLocked(int idx) const { return def(idx).flags & FL_DIVE_LOCK; }

private:
    static const Def* table();

    float m_num[NDEFS];
    // STRING values live in a slot per STRING definition, in table order.
    static constexpr size_t MAX_STRINGS = 4;
    char m_strs[MAX_STRINGS][MAX_VAL_STR];
    int m_strSlot[NDEFS]; // def index -> string slot, -1 for numeric
};

} // namespace settings
