// ============================================================================
// handset_bridge — a second ESP32 standing in for a USB-UART adapter on the
// bench, so the handset link (controller UART2, GPIO17 TX / GPIO16 RX) can be
// driven from the PC and watched at the same time.
//
// Two builds, one file, chosen by BRIDGE_TAP below.
//
//   pipe (default) — a byte-for-byte pipe between the PC (this board's own
//     USB-UART bridge chip on UART0) and the controller's UART2.
//     tools/hw_accept.py talks through it unchanged. It ALSO runs the handset
//     client library over the bytes it copies, purely as an observer: the two
//     LEDs are the library's verdict on the link, not my guess. Nothing is
//     ever printed to the PC stream in this mode, because the PC stream is
//     somebody else's protocol.
//
//   tap (BRIDGE_TAP 1) — no forwarding. Parses the controller's downlink and
//     prints a readable picture on the PC console: telemetry, events, replies,
//     raw unknown lines, link loss. This is the "watch a dive from the handset
//     side" build, and the base you add buttons and a buzzer to when it
//     becomes a real handset (see ../example_esp32_handset/).
//
// Why this works at all: on a classic ESP32 dev module the USB connector is a
// CP210x/CH340 whose data lines are the module's GPIO3/GPIO1 — that is the PC
// side (`Serial`), a plain 115200 UART with no CDC packet quirks. The
// controller side uses UART2 on GPIO16/17. Different peripherals, so there is
// no pin contention: the module's ROM boot log goes to the PC, never into the
// controller.
//
//   bridge GPIO16 (U2RXD) ← controller GPIO17 (controller TX)
//   bridge GPIO17 (U2TXD) → controller GPIO16 (controller RX)
//   bridge GND ───────────── controller GND      (power this board from its
//                                                 own USB; GND only)
//
// Bench note: opening this board's port resets THIS board (its own DTR/EN
// auto-reset), never the controller — an improvement on the console port,
// where the open is a reset pulse. Expect ~250 ms of ROM boot noise on the PC
// side after the port opens; tools/hw_accept.py settles and retries.
//
// Both modes are always compiled, chosen by a constexpr flag rather than
// #if/#else. That is deliberate: the Arduino build's automatic prototype
// generation reads the file without understanding the preprocessor, and
// functions hidden in an inactive #else branch come back as
// "declared 'static' but never defined". One compile, both paths checked.
// ============================================================================
#include <Arduino.h>

#include "../handset_link.h"

#ifndef BRIDGE_TAP
#define BRIDGE_TAP 0
#endif

// ---- pins -------------------------------------------------------------------
static const int U2_RX_PIN = 16;      // ← controller GPIO17
static const int U2_TX_PIN = 17;      // → controller GPIO16
static const int LED_LINK = 25;       // steady = client library says link up
static const int LED_TRAFFIC = 26;    // blinks on downlink lines

static const unsigned long BAUD = 115200;

static hs::Link obs;                  // observer: parses, never transmits

// ---- observer picture (written by callbacks, read by loop) ------------------
struct View {
    hs::Telemetry t{};
    hs::Deco d{};
    bool haveD = false;
    bool linkUp = false;
    uint32_t lastLineMs = 0;
} view;

static void onT(const hs::Telemetry& t, void* ctx) {
    static_cast<View*>(ctx)->t = t;
}
static void onD(const hs::Deco& d, void* ctx) {
    View& v = *static_cast<View*>(ctx);
    v.d = d; v.haveD = true;
}
static void onUp(void* ctx) {
    static_cast<View*>(ctx)->linkUp = true;
    view.lastLineMs = millis();
}
static void onDown(void* ctx) { static_cast<View*>(ctx)->linkUp = false; }

// ---- tap-only output (compiled in both modes, called from one branch) ------
static void onOk(const char* note, void*) { Serial.printf("  <-- OK%s\n", note ? note : ""); }
static void onErr(const char* why, void*) { Serial.printf("  <-- ERR %s\n", why); }
static void onEv(const hs::Event& e, void*) {
    Serial.printf("EV  t=%us id=%u val=%u\n", static_cast<unsigned>(e.tSec),
                  static_cast<unsigned>(e.id), static_cast<unsigned>(e.value));
}
static void onPre(const hs::PreStatus& p, void*) {
    static const char* kSt[] = {"pend", "run ", "pass", "FAIL"};
    Serial.printf("PRE warm=%s air=%s sol=%s depth=%s batt=%s  %s\n",
                  kSt[p.step[0]], kSt[p.step[1]], kSt[p.step[2]], kSt[p.step[3]],
                  kSt[p.step[4]], p.passed ? "PASSED" : "not passed");
}
static void onUnknown(const char* line, void*) { Serial.printf("RAW %s\n", line); }
static void onTapUp(void* ctx) { onUp(ctx); Serial.println("link up"); }
static void onTapDown(void* ctx) {
    onDown(ctx);
    Serial.println("!! LINK DOWN — no downlink line for >3 s");
}

static void draw() {
    const hs::Telemetry& t = view.t;
    Serial.printf("%6lus  d=%5.2fm  ppO2=%.3f  sp=%.3f  cells=%5.3f/%5.3f/%5.3f  %5.2fV  %s  flags=0x%02X",
                  static_cast<unsigned long>(millis() / 1000u), t.depthM, t.votedBar,
                  t.setpointBar, t.cells[0], t.cells[1], t.cells[2], t.battV,
                  t.mode == 1 ? "AUTO" : t.mode == 2 ? "MANUAL" : t.mode == 3 ? "SAFE" : "stby",
                  t.flags);
    if (view.haveD) {
        const hs::Deco& d = view.d;
        if (d.ndlMin < 0) Serial.printf("  NDL ---");
        else if (d.ndlMin >= 480) Serial.printf("  NDL inf");
        else Serial.printf("  NDL %d", d.ndlMin);
        Serial.printf("  deco=%.1f ceil=%.1fm CNS=%.0f%%", d.decoTotalMin, d.ceilingM, d.cnsPct);
    }
    Serial.printf("\n");
}

static hs::Handlers handlersForMode() {
    hs::Handlers h;
    h.ctx = &view;
    h.onTelemetry = onT;
    h.onDeco = onD;
    if (BRIDGE_TAP) {
        h.onLinkUp = onTapUp;
        h.onLinkDown = onTapDown;
        h.onOk = onOk;
        h.onErr = onErr;
        h.onEvent = onEv;
        h.onPre = onPre;
        h.onLine = onUnknown;
    } else {
        h.onLinkUp = onUp;
        h.onLinkDown = onDown;
    }
    return h;
}

void setup() {
    Serial.begin(BAUD);                       // PC side (module's bridge chip)
    Serial2.begin(BAUD, SERIAL_8N1, U2_RX_PIN, U2_TX_PIN);
    pinMode(LED_LINK, OUTPUT);
    pinMode(LED_TRAFFIC, OUTPUT);
    digitalWrite(LED_LINK, LOW);
    digitalWrite(LED_TRAFFIC, LOW);
    // NULL writer: the observer can parse but never put anything on the wire.
    obs.begin(nullptr, nullptr, handlersForMode());
    if (BRIDGE_TAP) {
        Serial.println("ESP-CCR handset TAP (observe only, no forwarding)");
        Serial.println("wiring: bridge GPIO16 <- controller GPIO17, GPIO17 -> GPIO16, GND common");
    }
    // Pipe mode prints nothing: that Serial *is* the handset protocol stream.
}

void loop() {
    const uint32_t now = millis();

    // controller → this board
    while (Serial2.available() > 0) {
        const int c = Serial2.read();
        if (c < 0) break;
        if (!BRIDGE_TAP) Serial.write(static_cast<uint8_t>(c));
        obs.feedByte(c);                      // observe; the copy is the point
    }
    // PC → controller (pipe mode only: in tap mode the PC is an operator's
    // console, not a client, and sending junk into a live dive is not useful)
    if (!BRIDGE_TAP) {
        while (Serial.available() > 0) {
            const int c = Serial.read();
            if (c < 0) break;
            Serial2.write(static_cast<uint8_t>(c));
        }
    }

    obs.tick(now);
    digitalWrite(LED_LINK, obs.linkUp() ? HIGH : LOW);
    digitalWrite(LED_TRAFFIC, (now - obs.lastRxMs()) < 40 ? HIGH : LOW);

    static uint32_t lastDraw = 0;
    if (BRIDGE_TAP && obs.linkUp() && now - lastDraw > 2000) { lastDraw = now; draw(); }
}
