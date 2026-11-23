// ============================================================================
// example_esp32_handset — a complete, deliberately boring handset for the
// ESP-CCR controller. Read docs/HANDSET_PROTOCOL.md first; this file is that
// protocol made physical.
//
// Transport: handset UART ↔ controller UART2.
//
//   handset GPIO13 (Serial2 RX) ──▶ controller GPIO17 (controller TX)
//   handset GPIO14 (Serial2 TX) ──▶ controller GPIO16 (controller RX)
//   handset GND ─────────────────── controller GND     (mandatory)
//
// 3.3 V logic both sides, 115200 8N1. The handset may be powered from the
// controller's 3V3 rail or its own cell; if it has its own cell, add a low
// battery indication of its own — a handset that dies quietly is a diver with
// no display.
//
// What this firmware does, and why:
//   * Redraws from the pushed stream only (T, at 2 Hz). No polling.
//   * Sounds an alarm from the controller's FLAGS byte. It keeps no alarm
//     state of its own: the controller's ACK already removes suppressed bits
//     from FLAGS, so "alarm while the bit is set" is the correct rule and
//     duplicating the latch logic here would drift from the controller.
//   * Alarms on link silence (>3 s with no line at all) — this is the failure
//     a wired handset is most likely to suffer in water.
//   * Two-press confirmation on DIVE START / DIVE END: those two commands
//     start and stop the recording, so one brush of a button must not.
//   * No dynamic memory, no RTOS tricks, one loop().
//
// The readout here is text on the handset's own USB serial, because a display
// driver is the builder's choice and must not be a dependency of the protocol
// example. Swap the `draw()` body for your display; everything else stands.
// ============================================================================
#include <Arduino.h>

#include "../handset_link.h"

// ---- pins -------------------------------------------------------------------
static const int H_RX_PIN = 13;      // ← controller GPIO17
static const int H_TX_PIN = 14;      // → controller GPIO16

static const int BTN_ACK = 25;
static const int BTN_AUTO = 26;
static const int BTN_MANUAL = 27;
static const int BTN_CHECK = 32;
static const int BTN_DIVE = 33;      // START/END toggle, two presses to confirm

static const int PIN_BUZZER = 19;    // active-high piezo (or transistor drive)
static const int PIN_LINK_LED = 21;  // steady = link up

// ---- link -------------------------------------------------------------------
// Not called `link`: <unistd.h> (pulled in by Arduino.h) declares
// `int link(const char*, const char*)`, and a global named `link` collides with
// it. Renaming the object, not the library — hs::Link itself is fine.
static hs::Link hsLink;

// Latest protocol picture. Written from the link callbacks, read by draw().
struct View {
    hs::Telemetry t{};
    hs::Deco d{};
    bool haveT = false, haveD = false;
    bool linkUp = false;
    char lastNote[48] = "";
    char lastErr[48] = "";
    uint32_t events = 0, schemaKeys = 0, unknown = 0;
    uint8_t alarmSeen = 0;         // flags that were set on the last T,
    bool predivePassed = false;
} view;

// A callback writer: hs::Link calls this for every command it sends.
static void putLine(const char* line, void*) { Serial2.println(line); }

// ---- callbacks (ctx = &view, so no globals are required) --------------------
static void onT(const hs::Telemetry& t, void* ctx) {
    View& v = *static_cast<View*>(ctx);
    v.t = t;
    v.haveT = true;
    v.alarmSeen = t.flags;
}
static void onD(const hs::Deco& d, void* ctx) {
    View& v = *static_cast<View*>(ctx);
    v.d = d;
    v.haveD = true;
}
static void onEv(const hs::Event&, void* ctx) {
    static_cast<View*>(ctx)->events++;
}
static void onOk(const char* note, void* ctx) {
    View& v = *static_cast<View*>(ctx);
    if (note) snprintf(v.lastNote, sizeof(v.lastNote), "%s", note);
    else snprintf(v.lastNote, sizeof(v.lastNote), "ok");
}
static void onErr(const char* why, void* ctx) {
    // Refusals are shown, never hidden: "predive required" and "dive locked"
    // are the safety model talking.
    snprintf(static_cast<View*>(ctx)->lastErr, 48, "%s", why);
}
static void onSchema(const hs::SchemaEntry&, void* ctx) {
    static_cast<View*>(ctx)->schemaKeys++;
}
static void onPre(const hs::PreStatus& p, void* ctx) {
    static_cast<View*>(ctx)->predivePassed = p.passed;
}
static void onLine(const char* line, void* ctx) {
    // Unknown / unstructured (dive CSV rows, HIST, future lines): counted.
    View& v = *static_cast<View*>(ctx);
    v.unknown++;
    Serial.printf("[raw] %s\n", line);
}
static void onLinkDown(void* ctx) { static_cast<View*>(ctx)->linkUp = false; }
static void onLinkUp(void* ctx) { static_cast<View*>(ctx)->linkUp = true; }

// ---- buttons: non-blocking, debounced, edge-detected ------------------------
// The debounce is a member, not a top-level function, and that is not style:
// the Arduino build auto-generates prototypes for top-level functions and
// inserts them *before* the type declarations below, so a top-level
// `bool pressed(Button&, uint32_t)` fails to compile ("Button was not declared
// in this scope"). Rule for .ino files: keep top-level signatures over Arduino
// and library types, or make them members.
struct Button {
    int pin;
    bool last = true;          // pull-up: idle high
    uint32_t changeMs = 0;
    bool latched = false;      // this press has already been reported
    uint32_t armedMs = 0;      // two-press confirm (DIVE button)

    // true once per press, after the contact settled.
    bool pressed(uint32_t now) {
        const bool raw = digitalRead(pin) == LOW;
        if (raw != last) { last = raw; changeMs = now; return false; }
        if (raw && !latched && now - changeMs > 30) { latched = true; return true; }
        if (!raw) latched = false;
        return false;
    }
};
static Button buttons[] = {
    { BTN_ACK }, { BTN_AUTO }, { BTN_MANUAL }, { BTN_CHECK }, { BTN_DIVE },
};

// ---- buzzer: a pattern per alarm class, never a blocking tone() -------------
// Steady piezo at 3 kHz would need PWM; for the example an on/off tick at the
// loop rate is enough to be heard and to keep this file free of timer setup.
static uint32_t buzzGapMs = 0;
static uint32_t nextPatternMs = 0;

static void alarmPattern(uint32_t now) {
    if (now < nextPatternMs) return;
    const uint8_t f = view.t.flags;
    if (!view.linkUp) {                      // link loss outranks everything:
        buzzGapMs = 60;                      // the diver may not see the screen
        nextPatternMs = now + 240;
    } else if (f & (1 << 3)) {               // major fault (sensor/hypoxia/CNS)
        buzzGapMs = 50;
        nextPatternMs = now + 180;
    } else if (f & (1 << 0)) {               // ppO2 LOW: act now
        buzzGapMs = 70;
        nextPatternMs = now + 350;
    } else if (f & (1 << 1)) {               // ppO2 HIGH
        buzzGapMs = 90;
        nextPatternMs = now + 600;
    } else if (f & (1 << 4)) {               // battery: nag, do not shout
        buzzGapMs = 120;
        nextPatternMs = now + 10000;
    } else if (f & (1 << 2)) {               // cell drift: single periodic chirp
        buzzGapMs = 60;
        nextPatternMs = now + 8000;
    } else {
        buzzGapMs = 0;
        nextPatternMs = now + 500;
    }
}

static void runBuzzer(uint32_t now) {
    if (!buzzGapMs) { digitalWrite(PIN_BUZZER, LOW); return; }
    if ((now / buzzGapMs) % 2 == 0) digitalWrite(PIN_BUZZER, HIGH);
    else digitalWrite(PIN_BUZZER, LOW);
}

// ---- drawing ----------------------------------------------------------------
static void draw() {
    const hs::Telemetry& t = view.t;
    const hs::Deco& d = view.d;
    Serial.printf("\r depth %6.2f m  ppO2 %.3f  sp %.3f  cell %6.3f/%6.3f/%6.3f  batt %5.2f V  %s   ",
                  t.depthM, t.votedBar, t.setpointBar, t.cells[0], t.cells[1],
                  t.cells[2], t.battV,
                  t.mode == 1 ? "AUTO" : t.mode == 2 ? "MANUAL" : t.mode == 3 ? "SAFE!" : "standby");
    if (view.haveD) {
        // -1 = unknown (no valid vote): "---", never 0, never "unlimited".
        if (d.ndlMin < 0) Serial.printf(" NDL  ---");
        else if (d.ndlMin >= 480) Serial.printf(" NDL  inf");
        else Serial.printf(" NDL %dmin", d.ndlMin);
        Serial.printf(" deco %.1fmin ceil %.1fm CNS %.0f%%", d.decoTotalMin, d.ceilingM, d.cnsPct);
        if (d.stopCount) {
            Serial.printf(" stops:");
            for (int i = 0; i < d.stopCount; ++i)
                Serial.printf(" %.0f@%.1f", d.stops[i].minutes, d.stops[i].depthM);
        }
    }
    if (t.flags) {
        Serial.printf("  FLAGS");
        if (t.flags & (1 << 0)) Serial.printf(" LOW-O2");
        if (t.flags & (1 << 1)) Serial.printf(" HIGH-O2");
        if (t.flags & (1 << 2)) Serial.printf(" DRIFT");
        if (t.flags & (1 << 3)) Serial.printf(" MAJOR");
        if (t.flags & (1 << 4)) Serial.printf(" BATT");
        if (t.flags & (1 << 5)) Serial.printf(" CLAMP");
        if (t.flags & (1 << 6)) Serial.printf(" PROOF-FAIL");
    }
    if (!view.linkUp) Serial.printf("   *** LINK DOWN ***");
    if (view.lastErr[0]) { Serial.printf("  ERR:%s", view.lastErr); view.lastErr[0] = 0; }
    else if (view.lastNote[0]) { Serial.printf("  (%s)", view.lastNote); view.lastNote[0] = 0; }
    Serial.printf("\n");
}

// ---- Arduino ----------------------------------------------------------------
void setup() {
    Serial.begin(115200);                 // handset's own console/readout
    Serial2.begin(115200, SERIAL_8N1, H_RX_PIN, H_TX_PIN);

    for (Button& b : buttons) pinMode(b.pin, INPUT_PULLUP);
    pinMode(PIN_BUZZER, OUTPUT);
    pinMode(PIN_LINK_LED, OUTPUT);
    digitalWrite(PIN_BUZZER, LOW);

    hs::Handlers h;
    h.ctx = &view;
    h.onTelemetry = onT;
    h.onDeco = onD;
    h.onEvent = onEv;
    h.onOk = onOk;
    h.onErr = onErr;
    h.onSchema = onSchema;
    h.onPre = onPre;
    h.onLine = onLine;
    h.onLinkDown = onLinkDown;
    h.onLinkUp = onLinkUp;
    hsLink.begin(putLine, nullptr, h);

    // Build the settings picture from the controller, not from a hard-coded
    // table (spec §6): a handset that does this keeps working when keys change.
    hsLink.getSchema();
    hsLink.getPre();
    Serial.println("ESP-CCR handset up");
}

void loop() {
    const uint32_t now = millis();

    while (Serial2.available() > 0) hsLink.feedByte(Serial2.read());
    hsLink.tick(now);                       // link-health watchdog

    // buttons
    for (size_t i = 0; i < sizeof(buttons) / sizeof(buttons[0]); ++i) {
        Button& b = buttons[i];
        if (!b.pressed(now)) continue;
        switch (b.pin) {
            case BTN_ACK:      hsLink.ack(); break;
            case BTN_AUTO:     hsLink.setAuto(); break;
            case BTN_MANUAL:   hsLink.setManual(); break;
            case BTN_CHECK:    hsLink.check(); break;
            case BTN_DIVE: {
                // Two presses within 3 s: the first arms, the second commits.
                const bool diving = view.t.mode != 0;   // STANDBY means no dive
                if (!b.armedMs || now - b.armedMs > 3000) {
                    b.armedMs = now;
                    snprintf(view.lastNote, sizeof(view.lastNote),
                             "press again: %s", diving ? "DIVE END" : "DIVE START");
                } else {
                    b.armedMs = 0;
                    // DIVE START replies ERR predive required until the
                    // pre-dive passed: show the refusal rather than
                    // pre-empting it, so the handset never disagrees with the
                    // controller about what is safe to start.
                    if (diving) hsLink.diveEnd();
                    else hsLink.diveStart();
                }
                break;
            }
        }
    }

    // periodic housekeeping: keep the pre-dive picture fresh until it passes
    static uint32_t lastPreMs = 0;
    if (!view.predivePassed && now - lastPreMs > 5000) { lastPreMs = now; hsLink.getPre(); }

    alarmPattern(now);
    runBuzzer(now);
    digitalWrite(PIN_LINK_LED, view.linkUp ? HIGH : LOW);

    static uint32_t lastDraw = 0;
    if (now - lastDraw > 500) { lastDraw = now; draw(); }
}
