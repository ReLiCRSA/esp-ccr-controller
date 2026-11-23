# Handset client package

Everything you need to build a handset (or any other client) that talks to the
ESP-CCR controller and stays compatible as the controller evolves.

| file | what it is |
|---|---|
| `../docs/HANDSET_PROTOCOL.md` | the contract: framing, every command, every downlink line, refusal strings, compatibility rules, known limits. **Read this first.** |
| `handset_link.h` | header-only C++17 client library: line assembly, parsers for every record, builders for every command, link-health watchdog. No OS dependency, no allocation on the protocol path. |
| `example_esp32_handset/` | complete working handset firmware (ESP32 + Arduino): wiring, buttons, buzzer alarm, link-loss alarm. Use it as a starting point or as a reference reading. |
| `../test/test_handset/test_handset.cpp` | conformance suite: the library's parsers fed the *firmware's* exact wire literals. Run it (`pio test -e native`) to prove the library still matches the controller. |

## 60-second orientation

The controller pushes telemetry at you continuously — a handset is passive by
default and never needs to poll to have a live display:

```
controller ──GPIO17(TX)──▶ handset RX      115200 8N1, 3.3 V, common GND
controller ◀─GPIO16(RX)─── handset TX
```

Downlink you receive unprompted: `T,` every 500 ms, `D,` every 2 s, `E,` on
every state change. Uplink is one text line per command (`AUTO`, `ACK`,
`DIVE START`, `SET GF_LOW=40`, …); the answer is always `OK`, `OK <NOTE>` or
`ERR <reason>` — never silence.

The **same** protocol runs over the controller's web socket (`ws://…/ws`), so
client code written against this library drives both transports. (The socket
adds one extra convention: a `#<n>` marker after each answer. UART never sees
it. The library treats it as an unrecognised line, which is harmless.)

## Using the library

```cpp
#include "handset_link.h"

static hs::Link link;

static void putLine(const char* line, void*) {   // your transport
    Serial2.println(line);                      // controller accepts \n or \r\n
}

void setup() {
    Serial2.begin(115200, SERIAL_8N1, /*RX=*/17, /*TX=*/16);  // see example
    hs::Handlers h;
    h.ctx = nullptr;                 // your state, handed back to every callback
    h.onTelemetry = [](const hs::Telemetry& t, void*) { /* redraw */ };
    h.onDeco      = [](const hs::Deco& d, void*)      { /* NDL / stops */ };
    h.onEvent     = [](const hs::Event& e, void*)     { /* alarm edges */ };
    h.onErr       = [](const char* why, void*)        { /* show it, don't hide it */ };
    h.onLinkDown  = [](void*) { /* ALARM: diver has no display */ };
    link.begin(putLine, nullptr, h);
}

void loop() {
    while (Serial2.available()) link.feedByte(Serial2.read());
    link.tick(millis());             // link-health watchdog; call it often
}
```

Rules that matter for a diver-safety device:

* **`tick(nowMs)` every loop iteration.** The controller sends `T,` at 2 Hz;
  more than 3 s of silence is `onLinkDown`. A handset that stays quiet when
  its cable fails is worse than no handset.
* **`Deco::ndlMin == -1` means unknown** (no valid cell vote yet) — render
  `---`. `480` means effectively unlimited. Never print `-1` or `0` for it.
* **`Telemetry::flags` bit 0 (low ppO2) and bit 3 (major fault) demand an
  audible alarm**, not a colour change. `ACK` silences an acknowledged alarm;
  a condition that persists keeps alarming.
* **Unknown downlink lines are ignored**, by the library and by you. The
  protocol is append-only (`../docs/HANDSET_PROTOCOL.md` §7): new fields are
  added after the documented ones, event ids are never renumbered, so a parser
  that reads the prefix it knows and ignores the rest never breaks against a
  newer controller.
* **Refusals are part of the safety model**, not errors to swallow:
  `ERR predive required` (AUTO before the pre-dive check passed),
  `ERR dive locked` / `ERR cal locked in dive` (setpoints and calibration are
  frozen mid-dive). Match the leading words, not whole sentences.

## Building the settings UI from the controller, not from a table

Do not hard-code the settings list. Ask `getSchema()` (`GET SCHEMA`): the
controller answers one `K,<key>,<type>,<min>,<max>,<flags>,<default>` line per
key, including types and ranges. A handset built from the schema stays correct
when keys are added. Useful details:

* `type`: `F` float, `I` int16, `U` uint8, `B` bool (`ON`/`OFF`), `S` string
  (then `min`/`max` are lengths).
* `flags` bit 0 → dive-locked: grey the control out while a dive is running.
  bit 1 → safety: log the change locally, the controller replies `OK SAFETY`.

## Verifying your handset against real firmware

`../tools/hw_accept.py` drives the controller over the handset UART with the
same commands this library emits, and checks the replies — including the
refusals, which are the part that must not be implemented twice and drift:

```
/home/wernervg/.local/share/pipx/venvs/platformio/bin/python \
    tools/hw_accept.py --expect-web --handset /dev/ttyUSB1
```

(`--handset` is a USB-UART adapter on the controller's GPIO16/17; omit it to
run only the boot checks. Exit 0 = every check passed.)

**No adapter in the drawer?** A second classic ESP32 dev module *is* one — its
USB connector's bridge chip has its data lines on GPIO3/GPIO1. Either jumper
that module's EN to GND and use the chip directly, or flash the ready bridge:

```
pio run -d handset/bridge_esp32 -t upload      # PC ↔ controller UART2 pipe
```

Wiring is the same as the example handset (bridge GPIO16 ← controller GPIO17,
GPIO17 → GPIO16, GND common). The bridge also runs this library as a
passive observer, so its two LEDs are the link's real state. Full recipe,
including why opening that port cannot reset the controller: `../BENCH.md`,
"The handset link on the bench".

Two bench rules that bite: **never run two readers on one port** (stop
`tools/soak_logger.py` first), and **opening the USB-CDC console resets the
board** — pyserial asserts DTR/RTS, and on this board DTR=IO0, RTS=EN, so the
`open()` is a reset pulse. Open the console once, or use
`tools/hw_accept.py --replay FILE` to grade a saved capture without touching
hardware.

If you change anything in `handset_link.h` or in the firmware's protocol code,
`pio test -e native` fails loudly — `../test/test_handset/` pins the library
against the wire literals both sides promise.

## Two Arduino `.ino` traps that cost a real compile failure here

* **Don't name a global `link`.** `Arduino.h` pulls in `<unistd.h>`, which
  declares `int link(const char*, const char*)`; a global object called `link`
  collides with it (`redeclared as different kind of symbol`). The example uses
  `hsLink`; `hs::Link` itself is fine.
* **The Arduino build auto-generates prototypes for top-level functions and
  inserts them before your type declarations.** A top-level
  `bool pressed(Button&, uint32_t)` therefore fails with "Button was not
  declared in this scope" even though the struct sits above it in the file.
  Keep top-level signatures over Arduino/library types, or make the function a
  member — the example's debounce is `Button::pressed`.

## Example firmware

`example_esp32_handset/example_esp32_handset.ino` — ESP32 handset: text
readout on the handset's own USB serial (swap in your display), five buttons
(ACK / AUTO / MANUAL / CHECK / DIVE, the dive one being a two-press START↔END
toggle), buzzer patterns per alarm class, a link-loss alarm that outranks
everything, and a `GET SCHEMA` at boot so the settings picture comes from the
controller. It is deliberately boring code: no RTOS tricks, no dynamic memory,
and everything you must not get wrong is in comments.
