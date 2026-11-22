// ============================================================================
// nvs_store.cpp — see nvs_store.h.
// ============================================================================
#include "nvs_store.h"

#include <Preferences.h>
#include <cstdio>

namespace nvs {

namespace {

const char* typeTag(settings::Type t) {
    switch (t) {
        case settings::Type::FLOAT: return "f";
        case settings::Type::INT16: return "i";
        case settings::Type::UINT8: return "u";
        case settings::Type::BOOL: return "b";
        case settings::Type::STRING: return "s";
    }
    return "?";
}

} // namespace

bool Store::schemaOk() {
    Preferences p;
    if (!p.begin(kNamespace, true)) return false;
    const uint32_t got = p.getUInt("_schema", 0);
    p.end();
    if (got == kSchema) return true;
    erase();
    return false;  // caller reloads defaults; writeSchema stores the new one
}

void Store::writeSchema() {
    Preferences p;
    if (!p.begin(kNamespace, false)) return;
    p.putUInt("_schema", kSchema);
    p.end();
}

int Store::load(settings::Registry& reg) {
    reg.resetToDefaults();
    if (!schemaOk()) return 0;

    Preferences p;
    if (!p.begin(kNamespace, true)) return 0;

    size_t count = 0;
    const settings::Def* defs = settings::Registry::definitions(count);
    int loaded = 0;
    for (size_t i = 0; i < count; i++) {
        const settings::Def& d = defs[i];
        char text[40];
        if (d.type == settings::Type::STRING) {
            String s = p.getString(d.key, "");
            if (s.length() == 0) continue;
            snprintf(text, sizeof(text), "%s", s.c_str());
        } else {
            const String defTxt =
                String(typeTag(d.type)) + ":" + String(d.def, 4);
            const String s = p.getString(d.key, defTxt);
            if (!s.startsWith(typeTag(d.type))) continue;  // foreign/old type
            snprintf(text, sizeof(text), "%s", s.c_str() + 2);  // strip "x:"
        }
        const char* err = nullptr;
        // Range-checked by the registry: a stale value outside a tightened
        // range falls back to the default rather than loading.
        if (reg.setValueByKey(d.key, text, &err)) loaded++;
    }
    p.end();
    return loaded;
}

bool Store::save(const settings::Registry& reg) {
    Preferences p;
    if (!p.begin(kNamespace, false)) return false;
    bool ok = true;

    size_t count = 0;
    const settings::Def* defs = settings::Registry::definitions(count);
    for (size_t i = 0; i < count; i++) {
        const settings::Def& d = defs[i];
        char text[40];
        if (!reg.valueAsString((int)i, text, sizeof(text))) { ok = false; continue; }
        String packed;
        if (d.type == settings::Type::STRING) packed = String(text);
        else packed = String(typeTag(d.type)) + ":" + String(text);
        if (!p.putString(d.key, packed)) {
            ok = false;
            Serial.printf("nvs: save FAILED for key %s\n", d.key);  // bench-visible
        }
    }
    p.end();
    writeSchema();
    return ok;
}

void Store::erase() {
    Preferences p;
    if (!p.begin(kNamespace, false)) return;
    p.clear();
    p.end();
}

} // namespace nvs
