#include <unity.h>

#include <cstring>

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

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_defaults_load);
    RUN_TEST(test_valid_and_invalid_sets);
    RUN_TEST(test_string_bounds_and_dive_lock_flags);
    RUN_TEST(test_value_as_string_and_list_all);
    RUN_TEST(test_reset_restores);
    RUN_TEST(test_keys_fit_nvs);
    return UNITY_END();
}
