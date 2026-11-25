#include <unity.h>

#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>

#include "comms/settings.h"

using namespace settings;

void setUp() {}
void tearDown() {}

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

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_defaults_load);
    RUN_TEST(test_valid_and_invalid_sets);
    RUN_TEST(test_string_bounds_and_dive_lock_flags);
    RUN_TEST(test_value_as_string_and_list_all);
    RUN_TEST(test_reset_restores);
    RUN_TEST(test_keys_fit_nvs);
    RUN_TEST(test_every_key_reaches_its_own_declared_bounds);
    return UNITY_END();
}
