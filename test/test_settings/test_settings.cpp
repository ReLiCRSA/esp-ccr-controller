#include <unity.h>

#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "comms/settings.h"
#include "config/defaults.h"

using namespace settings;

void setUp() {}
void tearDown() {}

// The pack is one 3.7 V Li-ion, and the NVS on every board on this bench holds
// the 10.5 this project used to assume. nvs_store range-checks what it loads, so
// the writable band IS the migration: a stored 10.5 falls back to the chemistry
// default instead of outliving the change and refusing the pre-dive BATTERY step
// forever on a board whose pack measures 4.1 V. That is why the band is
// chemistry-shaped rather than the old 3.0..30.0, and why the old rail has to be
// refused HERE, where a host test can see it, and not only on metal.
void test_battery_band_is_chemistry_shaped_and_refuses_the_old_rail() {
    size_t n = 0;
    const Def* defs = Registry::definitions(n);
    const Def* vb = nullptr;
    for (size_t i = 0; i < n; ++i)
        if (std::strcmp(defs[i].key, "VBAT_MIN") == 0) vb = &defs[i];
    TEST_ASSERT_NOT_NULL(vb);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, VBAT_MIN_V, vb->def);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, VBAT_CELLS * 2.80f, vb->minv);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, VBAT_CELLS * 4.50f, vb->maxv);
    TEST_ASSERT_TRUE_MESSAGE(vb->def > vb->minv && vb->def < vb->maxv,
                             "a default outside its own band is a default nothing can reach");

    // Written as strings because that is what the wire and the console hand in —
    // and built from the band, not typed, so this leg means the same thing after
    // VBAT_CELLS moves. The point of the leg is "the band is chemistry-shaped",
    // not "the band is 2.80..4.50".
    char atMin[16], atMax[16], belowMin[16];
    std::snprintf(atMin, sizeof atMin, "%.2f", (double)(VBAT_CELLS * 2.80f));
    std::snprintf(atMax, sizeof atMax, "%.2f", (double)(VBAT_CELLS * 4.50f));
    std::snprintf(belowMin, sizeof belowMin, "%.2f", (double)(VBAT_CELLS * 2.80f - 0.10f));
    Registry r;
    const char* err = nullptr;
    TEST_ASSERT_TRUE_MESSAGE(r.setValueByKey("VBAT_MIN", atMin, &err),
                             "the bottom of the band must be settable");
    TEST_ASSERT_TRUE_MESSAGE(r.setValueByKey("VBAT_MIN", atMax, &err),
                             "the top of the band must be settable");
    TEST_ASSERT_FALSE_MESSAGE(r.setValueByKey("VBAT_MIN", belowMin, &err),
                              "below a pack worth diving on must be refused");

    // With nothing written, the registry answers the chemistry — not a rail.
    Registry fresh;
    fresh.resetToDefaults();
    TEST_ASSERT_FLOAT_WITHIN(0.001f, VBAT_MIN_V, fresh.getFloat(fresh.find("VBAT_MIN")));
}

// The load-behaviour claim, on its own because it is the one that decides
// whether this change actually reaches a board: the NVS holds 10.5 from the old
// build, nvs_store range-checks what it loads, and a band that still admits 10.5
// keeps refusing the pre-dive BATTERY step forever on a rig whose pack measures
// 4.1 V. Refusing the old rail here is what clears it on the next boot.
void test_the_old_12v_rail_is_refused_so_a_stored_one_cannot_survive() {
    Registry r;
    r.resetToDefaults();      // a bare Registry holds nothing until reset
    const char* err = nullptr;
    TEST_ASSERT_FALSE_MESSAGE(r.setValueByKey("VBAT_MIN", "10.5", &err),
                              "the rail this project used to assume must not load");
    TEST_ASSERT_FLOAT_WITHIN(0.001f, VBAT_MIN_V, r.getFloat(r.find("VBAT_MIN")));

    // And a value written inside the band does stick, or the refusal above is
    // just "this key is broken". Derived from the band so this stays true when
    // VBAT_CELLS moves; the 10.5 above stays LITERAL on purpose — it is a
    // specific number that is sitting in a specific NVS on this bench.
    char inside[16];
    std::snprintf(inside, sizeof inside, "%.2f", (double)(VBAT_CELLS * 3.60f));
    TEST_ASSERT_TRUE(r.setValueByKey("VBAT_MIN", inside, &err));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, VBAT_CELLS * 3.60f, r.getFloat(r.find("VBAT_MIN")));
}

void test_defaults_load() {
    Registry r;
    r.resetToDefaults();
    const int i = r.find("SP2");
    TEST_ASSERT_TRUE(i >= 0);
    TEST_ASSERT_FLOAT_WITHIN(0.0001, 0.70f, r.getFloat(i));
    TEST_ASSERT_EQUAL_STRING("ESP-CCR", r.getString(r.find("AP_SSID")));
    TEST_ASSERT_TRUE(r.getBool(r.find("CELLPROOF_EN")));
    TEST_ASSERT_EQUAL(30, r.getInt(r.find("GF_LOW")));
}

void test_valid_and_invalid_sets() {
    Registry r;
    r.resetToDefaults();
    const char* err = nullptr;

    TEST_ASSERT_TRUE(r.setValueByKey("SP2", "0.55", &err));
    TEST_ASSERT_FLOAT_WITHIN(0.0001, 0.55f, r.getFloat(r.find("SP2")));

    TEST_ASSERT_FALSE(r.setValueByKey("SP2", "2.5", &err));   // > max
    TEST_ASSERT_EQUAL_STRING("out of range", err);
    TEST_ASSERT_FALSE(r.setValueByKey("SP2", "0.05", &err));  // < min
    TEST_ASSERT_FALSE(r.setValueByKey("GF_LOW", "30.5", &err)); // int required
    TEST_ASSERT_EQUAL_STRING("needs integer", err);
    TEST_ASSERT_FALSE(r.setValueByKey("GF_LOW", "abc", &err));
    TEST_ASSERT_EQUAL_STRING("not a number", err);
    TEST_ASSERT_FALSE(r.setValueByKey("NOT_A_KEY", "1", &err));
    TEST_ASSERT_EQUAL_STRING("unknown key", err);

    TEST_ASSERT_TRUE(r.setValueByKey("CELLPROOF_EN", "OFF", &err));
    TEST_ASSERT_FALSE(r.getBool(r.find("CELLPROOF_EN")));
    TEST_ASSERT_TRUE(r.setValueByKey("CELLPROOF_EN", "yes", &err)); // case-insensitive
    TEST_ASSERT_TRUE(r.getBool(r.find("CELLPROOF_EN")));
}

void test_string_bounds_and_dive_lock_flags() {
    Registry r;
    r.resetToDefaults();
    const char* err = nullptr;

    TEST_ASSERT_TRUE(r.setValueByKey("AP_PASS", "sealions-2026", &err));
    TEST_ASSERT_EQUAL_STRING("sealions-2026", r.getString(r.find("AP_PASS")));
    TEST_ASSERT_FALSE(r.setValueByKey("AP_PASS", "short", &err)); // < min length 8
    TEST_ASSERT_EQUAL_STRING("value too short", err);

    TEST_ASSERT_TRUE(r.isDiveLocked(r.find("AP_PASS")));
    TEST_ASSERT_TRUE(r.isDiveLocked(r.find("AP_SSID")));
    // the gas loop itself is frozen during a dive; schedule tuning is not
    TEST_ASSERT_TRUE(r.isDiveLocked(r.find("SP2")));
    TEST_ASSERT_TRUE(r.isDiveLocked(r.find("ALLO_HIGH")));
    TEST_ASSERT_TRUE(r.isDiveLocked(r.find("INJ_MAX_DUTY")));
    TEST_ASSERT_TRUE(r.isDiveLocked(r.find("CELLPROOF_SP")));
    TEST_ASSERT_FALSE(r.isDiveLocked(r.find("D1")));
    TEST_ASSERT_FALSE(r.isDiveLocked(r.find("GF_HIGH")));
    TEST_ASSERT_FALSE(r.isDiveLocked(r.find("CELL_DRIFT_BAR")));
    // registry defaults must equal the firmware constants the soak validated
    TEST_ASSERT_FLOAT_WITHIN(0.0001, 1.40f, r.def(r.find("ALLO_HIGH")).def);
    TEST_ASSERT_FLOAT_WITHIN(0.0001, 10.0f, r.def(r.find("INJ_MAX_DUTY")).def);
    // safety flag exists on life-limiting keys (callers event-log writes to them)
    TEST_ASSERT_TRUE(r.def(r.find("ALLO_CUTOFF")).flags & FL_SAFETY);
}

void test_value_as_string_and_list_all() {
    Registry r;
    r.resetToDefaults();

    char buf[16];
    TEST_ASSERT_TRUE(r.valueAsString(r.find("SP1"), buf, sizeof(buf)));
    TEST_ASSERT_EQUAL_STRING("0.400", buf);
    TEST_ASSERT_TRUE(r.valueAsString(r.find("GF_LOW"), buf, sizeof(buf)));
    TEST_ASSERT_EQUAL_STRING("30", buf);
    TEST_ASSERT_TRUE(r.valueAsString(r.find("CELLPROOF_EN"), buf, sizeof(buf)));
    TEST_ASSERT_EQUAL_STRING("ON", buf);

    char all[2048];
    const size_t n = r.listAll(all, sizeof(all));
    TEST_ASSERT_TRUE(n > 300);
    TEST_ASSERT_NOT_NULL(std::strstr(all, "SP1=0.400\n"));
    TEST_ASSERT_NOT_NULL(std::strstr(all, "CELLPROOF_MAXSP=0.900\n"));
    TEST_ASSERT_NOT_NULL(std::strstr(all, "AP_PASS=diveonly\n"));
    // tight buffer fails cleanly
    char tiny[8];
    TEST_ASSERT_EQUAL(0, r.listAll(tiny, sizeof(tiny)));

    // ...and valueAsString must REFUSE rather than answer half a value. The router
    // turns `false` into `ERR value too long`; a success here with a truncated
    // string would show a handset or the page a setting value that is not the one
    // stored — the same silent-shortening as the 23-character parser field
    // (2026-11-24), one layer further down. Mutation-proved: dropping the
    // overflow test from valueAsString reddens this and nothing else.
    char small[6];
    TEST_ASSERT_FALSE_MESSAGE(r.valueAsString(r.find("AP_PASS"), small, sizeof(small)),
                              "a 5-byte buffer reported an 8-character passphrase "
                              "as rendered");
}

void test_reset_restores() {
    Registry r;
    r.resetToDefaults();
    const char* err = nullptr;
    r.setValueByKey("SP4", "1.50", &err);
    r.setValueByKey("AP_SSID", "OTHERNET", &err);
    r.resetToDefaults();
    TEST_ASSERT_FLOAT_WITHIN(0.0001, 1.30f, r.getFloat(r.find("SP4")));
    TEST_ASSERT_EQUAL_STRING("ESP-CCR", r.getString(r.find("AP_SSID")));
}

// Every settings key is an NVS key on the device: the ESP32 NVS layer
// REJECTS keys over 15 characters, and it does so per-write deep inside
// save() - the native suite never touches NVS, so only the bench sees it,
// as "ERR settings not saved" on every save. Found on the bench 2026-11-23
// (CELLPROOF_OK_MAXSP=18, CELLPROOF_TIMEOUT=17); the registry pins the
// limit here so the build fails before the board ever has to.
void test_keys_fit_nvs() {
    size_t count = 0;
    const Def* defs = Registry::definitions(count);
    TEST_ASSERT_TRUE(count > 0);
    for (size_t i = 0; i < count; i++)
        TEST_ASSERT_MESSAGE(std::strlen(defs[i].key) <= 15, defs[i].key);
}

void test_every_key_reaches_its_own_declared_bounds() {
    // The ranges in this table are a PUBLISHED contract: `GET SCHEMA` hands them
    // to a handset, and the web page builds its <input min= max=> from them. So an
    // endpoint that the parser refuses is a device advertising 0.99..1.10 and
    // answering `ERR out of range` when the diver types the number it showed.
    //
    // This case exists because that was true on hardware on 2026-11-24. The value
    // was parsed with strtod (double) and compared against a `float` bound, and
    // the comparison promotes the bound to double — moving it by up to one ULP of
    // its own. The schema's decimal bounds are not exact binary values, so ten
    // keys had an unreachable MIN (SP1..SP4 at 0.20, SP_HYST 0.1, ALLO_LOW 0.05,
    // CAL_O2_FI 0.85, CAL_O2_TARGET 0.80, CELLPROOF_MAXSP 0.60, WATER_DENSITY
    // 0.990) and three an unreachable MAX (ALLO_CUTOFF 1.80, CELLPROOF_SP 1.40,
    // CELLPROOF_MIN 1.30) — 13 of the 30 numeric keys then in the table (34
    // today: the four WIFI_* policy keys joined). The type is what both the
    // schema and the storage use,
    // so parsing AND comparing in float is the fix — and this loop is what keeps
    // the whole table honest, rather than the one key that happened to be under
    // test that day.
    Registry r;
    r.resetToDefaults();
    size_t n = 0;
    const Def* table = Registry::definitions(n);
    int checked = 0;
    char txt[32];

    for (size_t i = 0; i < n; ++i) {
        const Def& d = table[i];
        if (d.type == Type::STRING || d.type == Type::BOOL) continue;

        std::snprintf(txt, sizeof(txt), "%.8g", static_cast<double>(d.minv));
        const char* err = nullptr;
        TEST_ASSERT_TRUE_MESSAGE(r.setValue(static_cast<int>(i), txt, &err),
                                 (std::string(d.key) + " refuses its own minimum " +
                                  txt + (err ? ": " + std::string(err) : "")).c_str());

        std::snprintf(txt, sizeof(txt), "%.8g", static_cast<double>(d.maxv));
        err = nullptr;
        TEST_ASSERT_TRUE_MESSAGE(r.setValue(static_cast<int>(i), txt, &err),
                                 (std::string(d.key) + " refuses its own maximum " +
                                  txt + (err ? ": " + std::string(err) : "")).c_str());

        // And the range really is a range: a little beyond each end must still be
        // refused, or this loop would be satisfied by a parser that accepts
        // anything. 1% is far outside float noise and inside nothing else.
        if (d.minv > 0.0f) {
            std::snprintf(txt, sizeof(txt), "%.8g",
                          static_cast<double>(d.minv / 1.01f));
            err = nullptr;
            TEST_ASSERT_FALSE_MESSAGE(r.setValue(static_cast<int>(i), txt, &err),
                                      (std::string(d.key) + " accepted " + txt +
                                       ", below its minimum " +
                                       std::to_string(d.minv)).c_str());
        }
        std::snprintf(txt, sizeof(txt), "%.8g",
                      static_cast<double>(d.maxv * 1.01f));
        err = nullptr;
        TEST_ASSERT_FALSE_MESSAGE(r.setValue(static_cast<int>(i), txt, &err),
                                  (std::string(d.key) + " accepted " + txt +
                                   ", above its maximum " +
                                   std::to_string(d.maxv)).c_str());
        ++checked;
    }
    // A loop that walked nothing is decoration: the table has 34 numeric keys,
    // and a floor this close to the real count means a key that vanishes from
    // the table reddens this case rather than quietly lowering the tally.
    TEST_ASSERT_GREATER_OR_EQUAL_INT_MESSAGE(30, checked,
                                             "the boundary walk covered too few keys");
}

// --- §6 of the handset contract against the registry it documents -------------
//
// Why this exists: HANDSET_PROTOCOL.md §6 is the table a handset builder reads
// instead of reading firmware, and it publishes each key's default and legal
// range. Nothing checked it, and on 2026-11-27 the battery key changed in the
// firmware (`VBAT_MIN` 10.5 / 3.0–30.0 → 3.40 / 2.80–4.50, derived from
// `VBAT_CELLS`) while the table still advertised the old numbers. A handset that
// offers "10.5" or greys out 3.40 is a handset built against a range the board
// refuses — and this repo has already been burned twice by a published range
// that the parser did not honour. The fix for "the doc drifted" is not a
// reminder, it is a check: the same shape test_flaglabels uses for the FLAGS
// table, applied to the settings table.
std::string readFile(const char* path) {
    FILE* f = std::fopen(path, "rb");
    if (!f) return "";
    std::string out;
    char buf[4096];
    size_t n = 0;
    while ((n = std::fread(buf, 1, sizeof(buf), f)) > 0) out.append(buf, n);
    std::fclose(f);
    return out;
}

// Splits one markdown row into its cells. Returns false unless the row has the
// settings table's five columns, which is also what keeps the ORDERING table
// further down §6 (three columns) out of this parser.
bool splitRow(const std::string& line, std::vector<std::string>& cells) {
    if (line.size() < 3 || line.compare(0, 2, "| ") != 0) return false;
    cells.clear();
    std::string cur;
    for (char c : line) {
        if (c == '|') { cells.push_back(cur); cur.clear(); }
        else cur += c;
    }
    // Expect a leading empty cell and NO trailing one: six cells for five columns,
    // because the text after the last '|' is empty and is never pushed.
    if (cells.size() != 6) return false;
    for (size_t k = 1; k <= 5; ++k) {
        // trim
        size_t a = cells[k].find_first_not_of(" \t");
        size_t b = cells[k].find_last_not_of(" \t\r\n");
        cells[k] = (a == std::string::npos) ? std::string() : cells[k].substr(a, b - a + 1);
    }
    return cells[2].size() == 1 && std::string("FIUBS").find(cells[2]) != std::string::npos;
}

// Pulls the decimal numbers out of a cell in order. The table writes ranges with
// an EN DASH ("2.80–4.50"), whose UTF-8 bytes are not digits, so scanning for
// number runs ignores the punctuation entirely — and "len 1–16" yields 1, 16.
std::vector<float> numbers(const std::string& cell) {
    std::vector<float> out;
    size_t i = 0;
    while (i < cell.size()) {
        const char c = cell[i];
        if (std::isdigit(static_cast<unsigned char>(c)) ||
            (c == '.' && i + 1 < cell.size() &&
             std::isdigit(static_cast<unsigned char>(cell[i + 1])))) {
            size_t j = i;
            bool dot = false;
            while (j < cell.size() &&
                   (std::isdigit(static_cast<unsigned char>(cell[j])) ||
                    (cell[j] == '.' && !dot))) {
                if (cell[j] == '.') dot = true;
                ++j;
            }
            out.push_back(std::strtof(cell.substr(i, j - i).c_str(), nullptr));
            i = j;
        } else {
            ++i;
        }
    }
    return out;
}

struct SpecKey {
    std::string key;
    char type = '?';
    bool hasDefault = false;
    float def = 0.0f;
    bool hasRange = false;
    float minv = 0.0f, maxv = 0.0f;
    bool diveLock = false, safety = false;
};

// Expands §6's table into one entry per KEY (the table groups a row when keys
// share a type and band: `SP1 SP2 SP3 SP4`, and `D1 D2` with one range each).
std::vector<SpecKey> specKeys(const std::string& doc) {
    std::vector<SpecKey> out;
    std::vector<std::string> cells;
    size_t pos = 0;
    while (pos < doc.size()) {
        size_t eol = doc.find('\n', pos);
        if (eol == std::string::npos) eol = doc.size();
        const std::string line = doc.substr(pos, eol - pos);
        pos = eol + 1;
        if (!splitRow(line, cells)) continue;

        // Grouped keys, in the order the table prints them.
        std::vector<std::string> keys;
        {
            std::string k = cells[1];
            size_t a = 0;
            while (a < k.size()) {
                size_t sp = k.find(' ', a);
                if (sp == std::string::npos) sp = k.size();
                if (sp > a) keys.push_back(k.substr(a, sp - a));
                a = sp + 1;
            }
        }
        if (keys.empty() || keys[0] == "key") continue;

        const std::vector<float> defs = numbers(cells[3]);
        // A range cell may carry prose in parentheses — `0–240 (**0 = never
        // off**)`. The band is what comes before the parenthesis; reading the
        // prose's digits too would make this parser invent a third number and
        // silently lose the range.
        std::string rangeCell = cells[4];
        const size_t par = rangeCell.find('(');
        if (par != std::string::npos) rangeCell = rangeCell.substr(0, par);
        const std::vector<float> rng = numbers(rangeCell);
        const bool oneDef = defs.size() == 1, perKeyDef = defs.size() == keys.size();
        const bool oneRange = rng.size() == 2, perKeyRange = rng.size() == 2u * keys.size();

        for (size_t i = 0; i < keys.size(); ++i) {
            SpecKey s;
            s.key = keys[i];
            s.type = cells[2][0];
            if (s.type == 'B') {
                s.hasDefault = true;
                if (cells[3] == "ON" || cells[3] == "on") s.def = 1.0f;
                else if (cells[3] == "OFF" || cells[3] == "off") s.def = 0.0f;
                else s.hasDefault = false;
            } else if (s.type != 'S' && perKeyDef) {
                s.hasDefault = true; s.def = defs[i];
            } else if (s.type != 'S' && oneDef) {
                s.hasDefault = true; s.def = defs[0];
            }
            if (oneRange) { s.hasRange = true; s.minv = rng[0]; s.maxv = rng[1]; }
            else if (perKeyRange) {
                s.hasRange = true; s.minv = rng[2 * i]; s.maxv = rng[2 * i + 1];
            }
            const std::string fl = cells[5];
            s.diveLock = fl.find("dive-lock") != std::string::npos;
            s.safety = fl.find("safety") != std::string::npos;
            out.push_back(s);
        }
    }
    return out;
}

const SpecKey* findSpec(const std::vector<SpecKey>& v, const std::string& key) {
    for (const SpecKey& s : v)
        if (s.key == key) return &s;
    return nullptr;
}

void test_the_published_settings_table_matches_the_registry_it_documents() {
    const std::string doc = readFile("docs/HANDSET_PROTOCOL.md");
    TEST_ASSERT_MESSAGE(!doc.empty(), "cannot read docs/HANDSET_PROTOCOL.md");
    const std::vector<SpecKey> spec = specKeys(doc);

    size_t n = 0;
    const Def* defs = Registry::definitions(n);

    // The anti-vacuous guard, in both directions: every registry key must be
    // published, and the table must not publish keys the board does not have.
    TEST_ASSERT_EQUAL_MESSAGE(static_cast<int>(n), static_cast<int>(spec.size()),
                              "the settings table and the registry do not hold the same number "
                              "of keys — a key appeared in one and not the other");
    TEST_ASSERT_TRUE_MESSAGE(n >= 30, "the registry shrank below the size this guard expects");

    int graded = 0;
    for (size_t i = 0; i < n; ++i) {
        const Def& d = defs[i];
        const SpecKey* s = findSpec(spec, d.key);
        TEST_ASSERT_NOT_NULL_MESSAGE(s,
                                     (std::string(d.key) + " is in the registry but not published "
                                      "in the contract's settings table").c_str());
        const char tc = d.type == Type::FLOAT ? 'F'
                      : d.type == Type::INT16 ? 'I'
                      : d.type == Type::UINT8 ? 'U'
                      : d.type == Type::BOOL ? 'B' : 'S';
        TEST_ASSERT_EQUAL_MESSAGE(tc, s->type,
                                  (std::string(d.key) + ": the table publishes type " + s->type +
                                   ", the registry says " + tc).c_str());
        if (d.type != Type::STRING) {
            TEST_ASSERT_TRUE_MESSAGE(s->hasDefault,
                                     (std::string(d.key) + ": no default this test can read out of "
                                      "the table — the row's default cell changed shape").c_str());
            TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.001, d.def, s->def,
                                             (std::string(d.key) + ": the published default is not "
                                              "the registry's default").c_str());
            // A published range the parser refuses is the defect this whole
            // family exists to stop (13 keys had unreachable endpoints once), so
            // both endpoints are compared, not just the printed shape. BOOL is
            // the exception the table is honest about: it prints `—` because the
            // key takes ON/OFF, and the registry's 0..1 is not a published band.
            if (d.type != Type::BOOL) {
                TEST_ASSERT_TRUE_MESSAGE(s->hasRange,
                                         (std::string(d.key) + ": the table gives no range and the "
                                          "registry has one").c_str());
                TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.001, d.minv, s->minv,
                                                 (std::string(d.key) + ": the published MINIMUM is not "
                                                  "the range the parser accepts").c_str());
                TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.001, d.maxv, s->maxv,
                                                 (std::string(d.key) + ": the published MAXIMUM is not "
                                                  "the range the parser accepts").c_str());
            }
        } else if (s->hasRange) {
            // Strings publish their LENGTH band ("len 1–16"), and the registry
            // puts those same two numbers in minv/maxv. A table that printed
            // `len 1–31` for a 16-char SSID is the same class of lie.
            TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.001, d.minv, s->minv,
                                             (std::string(d.key) + ": the published minimum LENGTH "
                                              "is not what the parser takes").c_str());
            TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.001, d.maxv, s->maxv,
                                             (std::string(d.key) + ": the published maximum LENGTH "
                                              "is not what the parser takes").c_str());
        }
        TEST_ASSERT_EQUAL_MESSAGE(static_cast<bool>(d.flags & FL_DIVE_LOCK), s->diveLock,
                                  (std::string(d.key) + ": dive-lock in the table and in the "
                                   "registry disagree").c_str());
        TEST_ASSERT_EQUAL_MESSAGE(static_cast<bool>(d.flags & FL_SAFETY), s->safety,
                                  (std::string(d.key) + ": the safety flag in the table and in the "
                                   "registry disagree").c_str());
        ++graded;
    }
    TEST_ASSERT_GREATER_OR_EQUAL_MESSAGE(30, graded, "graded too few keys to mean anything");
}

// The battery row specifically, by name, because it is the one that drifted and
// the one whose band is not a hand-typed number: it is per-cell times
// VBAT_CELLS, so the table is only ever right by derivation.
void test_the_published_battery_band_is_the_chemistry_band() {
    const std::string doc = readFile("docs/HANDSET_PROTOCOL.md");
    const std::vector<SpecKey> spec = specKeys(doc);
    const SpecKey* s = findSpec(spec, "VBAT_MIN");
    TEST_ASSERT_NOT_NULL(s);
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.001, VBAT_MIN_V, s->def,
                                     "the contract publishes a battery default that is not the "
                                     "chemistry in defaults.h");
    TEST_ASSERT_FLOAT_WITHIN(0.001, VBAT_RANGE_MIN_V, s->minv);
    TEST_ASSERT_FLOAT_WITHIN(0.001, VBAT_RANGE_MAX_V, s->maxv);
    TEST_ASSERT_TRUE_MESSAGE(s->safety, "VBAT_MIN must stay an event-logged safety key");
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_defaults_load);
    RUN_TEST(test_battery_band_is_chemistry_shaped_and_refuses_the_old_rail);
    RUN_TEST(test_the_old_12v_rail_is_refused_so_a_stored_one_cannot_survive);
    RUN_TEST(test_valid_and_invalid_sets);
    RUN_TEST(test_string_bounds_and_dive_lock_flags);
    RUN_TEST(test_value_as_string_and_list_all);
    RUN_TEST(test_reset_restores);
    RUN_TEST(test_keys_fit_nvs);
    RUN_TEST(test_every_key_reaches_its_own_declared_bounds);
    RUN_TEST(test_the_published_settings_table_matches_the_registry_it_documents);
    RUN_TEST(test_the_published_battery_band_is_the_chemistry_band);
    return UNITY_END();
}
