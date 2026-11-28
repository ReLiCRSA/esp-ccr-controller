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
// The same sketch runs on the ESP32-S3 bench board since 2026-11-28 (env
// bridge_s3), standing in for the classic dev module the bridge lived on. The
// S3 is NOT pin compatible, so the pin map below is per-board, chosen by the
// board define — never hand-edit it to swap boards:
//
//   PC side: `Serial` = UART0 on GPIO43/44 = the board's 1a86 UART bridge,
//     /dev/ttyACM1. Plain UART again, which is what hw_accept's timing was
//     built against. /dev/ttyACM0 is the native USB-Serial-JTAG: flash and ROM
//     logs only, never the pipe (CDC has DTR-gated TX and no baud semantics).
//   controller side: the wire stays where the handset image had it —
//
//   bridge GPIO13 (S3 U2RX) ← controller GPIO17      (witnessed on metal:
//   bridge GPIO14 (S3 U2TX) → controller GPIO16       handset link self-test,
//                                                     1154 B in 1.5 s)
//
//   The S3 devkit has no D25/D26 and its onboard RGB (GPIO48) has no driver in
//   this platform's core (2.0.17: no rgbLedWrite), so the S3 pipe build has no
//   status LEDs: it prints the uart2 self-test verdict on the PC port instead.
//   Boot-time only — steady-state printing would corrupt the protocol stream,
//   and the harness settles and flushes that port before its first command
//   anyway (that flush exists for the ROM banner already).
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
// Per-board map. The two bench boards share nothing here except the duty:
// U2_RX faces the controller's TX pin (GPIO17), U2_TX faces its RX (GPIO16).
#ifdef ARDUINO_ESP32S3_DEV
static const int U2_RX_PIN = 13;      // ← controller GPIO17 (handset's old hole)
static const int U2_TX_PIN = 14;      // → controller GPIO16 (handset's old hole)
static const int LED_LINK = -1;       // no D25/D26; RGB 48 undriven in core 2.0.17
static const int LED_TRAFFIC = -1;
#else
static const int U2_RX_PIN = 16;      // ← controller GPIO17
static const int U2_TX_PIN = 17;      // → controller GPIO16
static const int LED_LINK = 25;       // steady = client library says link up
static const int LED_TRAFFIC = 26;    // blinks on downlink lines
#endif

static const unsigned long BAUD = 115200;

// One guard instead of #if-ing every LED line: -1 means this board has none,
// and a missing status LED must never turn into a crash or a Serial print in
// pipe mode.
static void ledWrite(int pin, int value) {
    if (pin >= 0) digitalWrite(pin, value);
}

static hs::Link obs;                  // observer: parses, never transmits

// ---- boot-time proof that THIS board's UART2 reaches its pins ---------------
// Pipe mode is deliberately silent, which is also how a deaf app looks. This
// test is the difference: 8 bytes out U2TX, counted back on U2RX.
//   jumper D16 <-> D17 on this board  -> 8/8 (UART2, pins and PC path all fine)
//   controller wired instead          -> some bytes back, and the controller's
//                                        console shows it answered ERR unknown
//                                        command: the UPLINK works, both ways
//   nothing wired                     -> 0/8 (and that is the only case where
//                                        0/8 means "fine, nothing attached")
// Result is printed in TAP mode only. In pipe mode the PC stream is somebody
// else's protocol, so the verdict goes to the LEDs instead: both LEDs blink 3x
// = UART2 OK, LED_LINK fast-blinking = no echo. Remove the jumper afterwards:
// left in, the bridge's TX feeds back into its own RX. On the S3 pipe build,
// which has no LEDs, the verdict is printed once at boot on the PC port: one
// boot-time line is not steady-state printing, and every harness that opens
// that port settles and flushes it before its first command (the flush exists
// for the ROM banner whether this line is there or not).
static int uart2SelfTest() {
    static const uint8_t kPat[] = {0x55, 0xAA, 'H', 'S', 'L', 'F', 0x0D, 0x0A};
    Serial2.flush();
    while (Serial2.available() > 0) Serial2.read();       // flush RX first
    for (const uint8_t b : kPat) Serial2.write(b);
    Serial2.flush();
    const uint32_t deadline = millis() + 100;
    int got = 0;
    while (millis() < deadline && got < static_cast<int>(sizeof(kPat))) {
        if (Serial2.read() >= 0) got++;
    }
    return got;
}

static void ledPattern(int blinks) {
    for (int i = 0; i < blinks; i++) {
        ledWrite(LED_LINK, HIGH);
        ledWrite(LED_TRAFFIC, HIGH);
        delay(120);
        ledWrite(LED_LINK, LOW);
        ledWrite(LED_TRAFFIC, LOW);
        delay(120);
    }
}

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
static void onCalAir(const hs::CalAir& c, void*) {
    // The air calibration's verdict, in words: `CAL AIR`'s reply is only a
    // receipt, so a tap that prints replies alone would report a calibration as
    // if it had happened.
    Serial.printf("AIR %s", hs::calAirText(c.code));
    if (c.code == hs::CalAir::OK)
        Serial.printf("  raw %d/%d/%d  worst spread %.0f  %d samples",
                      static_cast<int>(c.rawAvg[0]), static_cast<int>(c.rawAvg[1]),
                      static_cast<int>(c.rawAvg[2]), static_cast<double>(c.spreadWorst),
                      c.samples);
    else if (c.code == hs::CalAir::WARMUP)
        Serial.printf("  %lu s left", static_cast<unsigned long>(c.warmupLeftS));
    Serial.printf("  (%lu s ago)\n", static_cast<unsigned long>(c.ageS));
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
        h.onCalAir = onCalAir;
        h.onLine = onUnknown;
    } else {
        h.onLinkUp = onUp;
        h.onLinkDown = onDown;
    }
    return h;
}

static void printTapBanner() {
    // The wiring line names the pins the build ACTUALLY uses: 16/17 on the
    // classic, 13/14 on the S3. Hard-coded numbers here once pointed a human
    // at holes that were not wired on the board in front of them.
    Serial.println("ESP-CCR handset TAP (observe only, no forwarding)");
    Serial.printf("wiring: bridge GPIO%d <- controller GPIO17, GPIO%d -> GPIO16, GND common\n",
                  U2_RX_PIN, U2_TX_PIN);
}

void setup() {
    Serial.begin(BAUD);                       // PC side (module's bridge chip)
    Serial2.begin(BAUD, SERIAL_8N1, U2_RX_PIN, U2_TX_PIN);
    if (LED_LINK >= 0) pinMode(LED_LINK, OUTPUT);
    if (LED_TRAFFIC >= 0) pinMode(LED_TRAFFIC, OUTPUT);
    ledWrite(LED_LINK, LOW);
    ledWrite(LED_TRAFFIC, LOW);

    const int echo = uart2SelfTest();         // see the comment above the test
    if (BRIDGE_TAP) {
        printTapBanner();
        Serial.printf("uart2 self-test: %d bytes back on RX from TX: %s\n", echo,
                      echo ? "OK - this board's UART2 and its pins are good"
                           : "no echo - jumper not seen (check the silkscreen: "
                             "the link pins, not TX0/RX0)");
    } else if (LED_LINK >= 0) {
        if (echo > 0) ledPattern(echo >= 8 ? 3 : 2);  // pipe: LEDs are the safe output
        else ledPattern(1);
    } else {
        // Pipe on a board with no LEDs (S3): boot-time verdict on the PC port,
        // flushed by every harness that opens the port. Then silence, as pipe
        // mode requires.
        Serial.printf("bridge ready rx%d tx%d (pipe)\n", U2_RX_PIN, U2_TX_PIN);
        Serial.printf("uart2 self-test: %d bytes back on RX from TX: %s\n", echo,
                      echo ? "OK - this board's UART2 and its pins are good"
                           : "no echo - jumper not seen (check the silkscreen: "
                             "the link pins, not TX0/RX0)");
    }

    // NULL writer: the observer can parse but never put anything on the wire.
    obs.begin(nullptr, nullptr, handlersForMode());
    if (BRIDGE_TAP) printTapBanner();
    // Pipe mode prints nothing past boot: that Serial *is* the handset protocol
    // stream.
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
    ledWrite(LED_LINK, obs.linkUp() ? HIGH : LOW);
    ledWrite(LED_TRAFFIC, (now - obs.lastRxMs()) < 40 ? HIGH : LOW);

    static uint32_t lastDraw = 0;
    if (BRIDGE_TAP && obs.linkUp() && now - lastDraw > 2000) { lastDraw = now; draw(); }
}
