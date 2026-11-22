#include "hud.h"
#include "config/pins.h"
#include "config/defaults.h"

static HudState  g_state = HudState::STANDBY;
static ErrLevel  g_err = ErrLevel::NONE;
static bool      g_deco = false;

// Pattern phase trackers
static uint32_t g_hbPhaseStart = 0;
static uint8_t  g_hbStep = 0;
static uint32_t g_errPhaseStart = 0;
static uint8_t  g_errStep = 0;

void hudInit() {
    pinMode(PIN_LED_HB, OUTPUT);
    pinMode(PIN_LED_ERR, OUTPUT);
    pinMode(PIN_LED_DECO, OUTPUT);
    digitalWrite(PIN_LED_HB, LOW);
    digitalWrite(PIN_LED_ERR, LOW);
    digitalWrite(PIN_LED_DECO, LOW);
    // Solenoid stays failsafe-low until the ControlTask owns it (M2).
    pinMode(PIN_SOLENOID, INPUT_PULLDOWN);
}

void hudSetState(HudState s) { g_state = s; g_hbStep = 0; g_hbPhaseStart = millis(); }
void hudSetError(ErrLevel l) { g_err = l; g_errStep = 0; g_errPhaseStart = millis(); }
void hudSetDeco(bool on) { g_deco = on; }

static void hbUpdate(uint32_t nowMs) {
    uint32_t& start = g_hbPhaseStart;
    uint8_t& step = g_hbStep;
    uint32_t onMs = 0, offMs = 0;
    bool blink2 = false;

    switch (g_state) {
        case HudState::AUTO:
            onMs = HB_AUTO_ON_MS; offMs = HB_AUTO_OFF_MS; break;
        case HudState::STANDBY:
            onMs = HB_STANDBY_ON_MS; offMs = HB_STANDBY_OFF_MS; break;
        case HudState::DEGRADED:
        case HudState::SAFE:
            onMs = HB_DOUBLE_ON_MS; offMs = HB_DOUBLE_OFF_MS;
            blink2 = true; break;
    }

    const uint32_t phaseLen = (blink2 && step == 0) ? onMs
                            : (blink2 && step == 1) ? offMs
                            : (blink2 && step == 2) ? onMs
                            : offMs + HB_DOUBLE_GAP_MS;
    if (nowMs - start >= phaseLen) {
        start = nowMs;
        step = blink2 ? static_cast<uint8_t>((step + 1) % 4)
                      : static_cast<uint8_t>((step + 1) % 2);
    }
    const bool lit = blink2 ? (step == 0 || step == 2) : (step == 0);
    digitalWrite(PIN_LED_HB, lit ? HIGH : LOW);
}

static void errUpdate(uint32_t nowMs) {
    uint32_t& start = g_errPhaseStart;
    uint8_t& step = g_errStep;
    switch (g_err) {
        case ErrLevel::NONE:
            digitalWrite(PIN_LED_ERR, LOW);
            step = 0; start = nowMs;
            return;
        case ErrLevel::HARD:
            digitalWrite(PIN_LED_ERR, HIGH);
            return;
        case ErrLevel::WARNING: { // slow blink
            const uint32_t half = 800;
            if (nowMs - start >= half) { start = nowMs; step ^= 1; }
            digitalWrite(PIN_LED_ERR, step ? LOW : HIGH);
            return;
        }
        case ErrLevel::ALARM: { // fast blink
            const uint32_t half = 180;
            if (nowMs - start >= half) { start = nowMs; step ^= 1; }
            digitalWrite(PIN_LED_ERR, step ? LOW : HIGH);
            return;
        }
    }
}

void hudUpdate(uint32_t nowMs) {
    hbUpdate(nowMs);
    errUpdate(nowMs);
    digitalWrite(PIN_LED_DECO, g_deco ? HIGH : LOW);
}
