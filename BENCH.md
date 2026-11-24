# Bench acceptance & hardware checklists (C2 / C3, PLAN.md 15)

Rules that never expire: **never open `/dev/ttyUSB0` while
`tools/soak_logger.py` runs** (two readers, both blind), and nothing here
counts as done until it is *watched happening on the bench*.

**Opening the console port resets the board.** pyserial asserts DTR/RTS on
`open()`; on this board's classic auto-reset circuit DTR=IO0, RTS=EN, so
the open is a reset pulse (every "mystery POWERON" during the web debugging
was the *laptop* opening the port, not the board). Keep one reader, expect
a boot banner at the start of every capture, and never conclude "the board
reboots" from a log that begins with one.

## C2 — flash + acceptance (run first, every flash)

```
fuser /dev/ttyUSB0            # must be empty; a reader here corrupts the run
pio test -e native            # host gate, currently 206 cases
tools/float_audit.py          # no float printf in src/ except marked boot-only sites
pio run -e esp32 -t upload    # only once the soak has ended
tools/.../python tools/hw_accept.py --expect-web --handset /dev/ttyUSB1 \
                                 --dive-persist
tools/.../python tools/web_probe.py --handset /dev/ttyUSB1 --dive-cycle
```

There is no `uploadfs` step. The web page is compiled into the app image and
served from there; the LittleFS partition belongs to the dive logs, and writing
it (`pio run -e esp32 -t uploadfs`) erases dive history. `data/index.html` is the
bench's reference copy — `web_probe.py` fails unless the bytes on the wire equal
it — not an install target. If the partition ever has to be written by hand it
needs `--upload-port` (two identical CP2102s) and it is a destructive step.

`hw_accept.py --dive-persist` closes one dive, resets the controller through the
console's EN line and asks `LOG LIST` again: the slots must be the same ones, and
the fresh boot banner must report the same count with no format warning. Ask for
it by name — it adds ~35 s and a deliberate reboot, so it is not part of the
short run. Reading the dive log before the first `DIVE START` of a boot used to be
blind (the filesystem was only mounted by `DIVE START`), and the FIFO reused slot 1
for every dive, so the board kept exactly one dive; both are in 16 and both were
found by asking this question on the bench. Once the FIFO holds its full ten, the
new dive *replaces* one, so the check expects ten slots and not eleven — and it
deliberately does not claim the replaced dive was the right one, because it was
not (PLAN 16: with every dive stamped epoch 0 the eviction order is undecided).

`web_probe.py` also compares `LOG EXPORT <n>` on the handset with
`GET /dive/<n>.csv` over HTTP, row for row — the C2 claim that "the download
matches the handset export" had been asserted by hand since it was written and
was false the whole time (two writers, two shapes). Both now come out of one
formatter, and the comparison is the tripwire.

**Fresh-filesystem leg** (run it after anything that wipes the log partition, and
now and then just to prove the recovery path): erase the partition the dive files
live on, then boot.

```
~/.platformio/packages/tool-esptoolpy/esptool.py --chip esp32 --port /dev/ttyUSB0 \
    --baud 921600 erase_region 0x290000 0x160000          # LittleFS, 1 441 792 B
```

Expect, in the boot banner: `esp_littlefs: mount failed, (-84)` and
`Corrupted dir pair at {0x0, 0x1}`, then the store's own verdict —

```
log: LittleFS mounted, 0 dives, used 8192 of 1441792 bytes  ** mounted after FORMAT: dive history erased
```

and **nothing else has to be done**: no install, no reflash. The store tries
`LittleFS.begin(false)` first and only formats if that fails, and the
`** mounted after FORMAT` suffix is what stops a wiped history from being
silent. `hw_accept.py --dive-persist` asserts the suffix is *absent* after an
ordinary reset, so a format at the wrong time fails the gate.

While the log is being probed, the console also shows ten lines like
`E (658) vfs_api: open(): /littlefs/dive10.bin does not exist, no permits for
creation`. That is core noise, not a fault: `list()` probes the FIFO slots and the
Arduino VFS logs a missing one at error level; asking with `LittleFS.exists()`
first does not silence it (measured 2026-11-24 — the lines arrived anyway). The
verdict is the `log:` line, not those.

`tools/float_audit.py` is in the gate because the same bug came back once: a
`%f`/`%g` conversion pulls in newlib's float conversion, which wants kilobytes
of stack, and the tasks that answer clients have a few. It scans `src/` and
fails on any float format specifier that is not marked `/*float-ok: why*/`; the
one marked exception is the settings-schema renderer, which runs once from
`Session::begin` before the small-stack tasks exist.

`hw_accept.py` checks the boot console (fresh boot banner, ≥2 HB lines
matching the soak_logger contract, no SAFE/FATAL, `settings: N keys from
NVS`, `control: SP=…`, `web: AP "…"`, `log: LittleFS mounted, N dives,
seq a..b, used X of Y bytes`, and a `clock:` line) and then runs the
**refusal matrix**
over the wired handset: `AUTO` cold → `ERR predive required`; `SET SP2`
mid-dive → `ERR dive locked`; `WIFI ON` mid-dive → `ERR wifi locked in
dive`; `CAL AIR` mid-dive → `ERR cal locked in dive`; overlong line →
`ERR line too long`; junk → `ERR unknown command`. Every one of those is a
host-tested invariant — the bench exists to prove the wire, not the logic.

The `log:` boot line is a check, not decoration. A LittleFS mount that fails is
answered with a format, which erases every dive on the partition, and the core
does that in silence; the line says how many dives the mount found and warns if
it had to format. A dive computer that quietly eats its own history is worse
than one that says the log is offline.

Two things the harness did NOT do until 2026-11-23, and both let a real fault
through:

* **It scored `GET T` as PASS while that command was rebooting the board.**
  `GET T` answers with a `T,` line, and the controller also streams `T,` at 2 Hz
  unprompted, so a prefix match proved nothing. `hw_accept.py` now *anchors on
  the cadence*: it waits for a streamed line, sends immediately after it, and
  requires a matching line within 450 ms — the next unsolicited one cannot
  arrive for another 500 ms, so what it finds is the reply. `GET D` (0.5 Hz) is
  checked the same way.
* **A crash in the middle of the matrix looked like ten unrelated timeouts.**
  The console is now kept open with a reader through the whole handset phase, plus
  `--watch-tail 12` seconds after the last command (the matrix itself runs in ~2 s,
  and HB beats every 5 s, so without the tail the headroom checks have nothing to
  read). Any `Guru Meditation`, stack-canary, `** SAFE MODE`, `!! FATAL`,
  `PRIOR FAULT` or `reset reason: PANIC` in that window is a named FAIL, and the
  `stk=` / `u2stk=` headroom HB reports must stay at or above 512 bytes.

### The radio drops at DIVE START — stress it, do not trust one clean run

`DIVE START` turns the WiFi off physically (PLAN 9: milliamps and an antenna in
water), and that teardown is a race against the driver's own worker. On
2026-11-23 it asserted inside ESP-IDF (`spinlock_acquire` in
`semphr_give_wrapper <- ieee80211_ioctl_process <- ppTask`) and rebooted the
controller, in 2 of 4 acceptance runs. `comms::web::setRadio()` now only records
the intent; loopTask performs it after a driver drain. Check the ordering with:

```
tools/.../python tools/radio_stress.py --cycles 12 --log logs/radio.log
```

One `WIFI OFF` is a full `esp_wifi_stop()`; one `WIFI ON` is a full `softAP`.
The tool does **not** stop at "no crash" — a deferred action needs its
completion proven, so every cycle is graded on what the firmware itself reports:

| Per cycle | Why |
|---|---|
| `web: radio off (dive)` within `--max-down` (default 3 s) of `WIFI OFF` | the intent was recorded *and* carried out |
| `web: AP "…" on <ip> (heap N)` within `--max-up` (default 6 s) of `WIFI ON` | the diver gets a surface UI back; `OK` on the command means only that the intent was recorded |
| exactly N down and N up over N cycles | a pending change that gets reversed before loopTask acts is silently cancelled (coalescing), which is correct for a 0.5 s dive and a fault for a real one |
| identical AP **name** every cycle, and a MAC tail present | a renamed AP, or one without the `-XXXX` tail, is a network the phone that joined before the dive cannot rejoin |
| heap from each AP line | drift over cycles = a leak on the dive path. Measured: −192 B over 12 cycles, i.e. the 2.5 KB drop on the *first* cycle is one-time driver allocation, not per-dive |

Measured timings: radio down in 0.36–0.56 s, AP back in 0.31–0.51 s. `loopTask`
polls for a pending radio change every 250 ms (it used to be 1 s) and `stop()`
drains the driver for 250 ms, so ~0.5 s is the expected worst case.

**Known console noise, not a fault:** `E (nnnnn) wifi_init_default:
esp_wifi_get_mac failed with 12289` appears on some teardowns. It is ESP-IDF's
own read inside `esp_wifi_stop()` — the Arduino WiFi library never calls
`esp_wifi_get_mac` anywhere (`WiFiProv` does, and that is not used here). It was
proved not-ours by re-injecting the firmware's earlier call order and watching
the marker land in the same place either way. `hw_accept.py` and
`radio_stress.py` report it as a note and do not fail on it; a check that fires
on healthy hardware is worse than no check.

**What that tool still cannot prove:** the page and the WebSocket actually
serving after a radio cycle. `radio_stress.py` never opens the network — it reads
the console. For that there is `tools/web_probe.py`, next section.

### The surface web stack, with a station joined (`tools/web_probe.py`)

The handset is the in-water transport; the AP is how a diver reads the dive plan,
changes settings and pulls the CSVs at the surface. Both are supposed to be the
**same command path with the same refusals**, and until 2026-11-23 nothing on the
bench had ever proved it — this section used to say "join it from a phone and
look", which is not a check anyone can run twice and compare.

```
tools/.../python tools/web_probe.py --host 192.168.4.1 \
    --handset /dev/ttyUSB1 --dive-cycle
```

It needs a station already joined to the controller's AP (SSID `ESP-CCR-XXXX`,
WPA2, password = the `AP_PASS` setting). **Joining is the operator's setup**: the
tool never connects, disconnects or reconfigures anything on the machine it runs
on, and only ever *scans* the air to confirm the AP is broadcasting. Checks that
need a link it does not have report `SKIP` ("not evaluated") rather than failing
the firmware for the operator's network.

| Phase | What it asserts, and why it matters |
|---|---|
| HTTP | `/` serves and is **byte-identical to `data/index.html`**. This is the check that found the page had never actually been installed (C2-f below). `/favicon.ico` is 204, unknown paths 302 → `/` (that redirect *is* the captive portal), `/app.js` 404 because the page is one self-contained file |
| Page | references nothing external (`page links: []`). The AP has no uplink, ever, so any CDN or cross-site reference is a dead page on the day it matters |
| DNS | any name at all resolves to the AP IP — the other half of the captive portal (C2-e above: without it a phone resolves nothing and "the page does not load") |
| WebSocket | handshake verified against RFC 6455 (`Sec-WebSocket-Accept` recomputed locally, no library: the point is to verify the server, so a dependency would be verifying itself); the `#boot <uptimeMs> <freeHeap>` hello on connect; then every command's answer must end with a `#<n>` marker whose **n equals the answer's line count** — if it does not, the browser renders half a table |
| Equality | each command sent twice **back to back** — once over the WebSocket, once on UART2 — with identical non-stream reply lines. That is the project's central claim (PLAN: one command session, not one per transport) |
| `--dive-cycle` | `DIVE START` from the browser takes the radio out from under the associated client (HTTP stops answering, the socket dies), the dive is then closed over UART2 — the web cannot close the dive that switched itself off, which is itself the argument for the handset being the in-water transport — and the AP is broadcasting again with a fresh WebSocket working |

Measured 2026-11-23, 34 checks, 0 failures: radio unreachable under the client
0.5–1.0 s after `DIVE START`, AP back and serving 200 after `DIVE END`, page
25 445 bytes identical to the repo, all eight commands identical on both
transports, markers matched every answer (`GET *` → 32 lines, `#32`).

Three traps this tool walked into, all of them harness bugs that looked like
firmware faults — worth re-reading before adding a check:

* **`#` is not one vocabulary.** The server also sends `#boot <uptimeMs> <freeHeap>`
  when a socket connects. Treating any `#` line as the answer marker stopped
  reading at the hello, so every reply was attributed to the *next* command and
  all eight equality checks failed on good firmware. Match the marker shape
  exactly (`#\d+( dropped)?`).
* **Two separate passes compare different states.** Running the web matrix and
  then the UART matrix "disagreed" on `GET *` (`SP2=0.550` vs `0.700`) purely
  because the web pass had already executed `SET SP2=0.70`. Interleaving is the
  only correct shape.
* **Un-terminated bytes sit in the controller's line buffer** and get parsed with
  the next line, so the first command of a run can be answered `ERR unknown
  command` for a reason that has nothing to do with the firmware. See the
  handset-link section below; `hw_accept.py` and `web_probe.py` both flush the
  controller's buffer, and report what the flush drew.

### The handset link on the bench (no USB-UART adapter in the drawer)

A second **classic ESP32 dev module is a USB-UART adapter** — its USB socket is
a CP210x/CH340 whose data lines are the module's GPIO3/GPIO1. Two ways to use
it; both land on `/dev/ttyUSB1` at 115200 8N1 for `--handset`.

**A. Dead-module bridge (no firmware, no code).** Jumper the spare module's
**EN to GND** so the ESP32 never runs and only its bridge chip lives:

```
spare GPIO3 (chip TX) → controller GPIO16   (controller RX)
spare GPIO1 (chip RX) ← controller GPIO17   (controller TX)
spare GND ─────────────── controller GND
spare EN  ── GND          (module dead; its pins float, idle-high on the wire)
```

Hold EN low or the live ESP32 drives GPIO1 itself (ROM boot log on UART0 at
115200): that fights the controller's TX driver on the same wire and feeds the
controller junk it dutifully answers `ERR unknown command` about. Remove the
jumper to flash that board again.

**Bridge firmware (option B) has a side effect every harness must handle.**
Opening `/dev/ttyUSB1` resets the spare ESP32, and the boot banner it prints goes
out its TX straight into the controller's RX. Those bytes have no line terminator
of their own, and the controller **keeps un-terminated bytes in its line buffer
and parses them together with the next line**: verified on the bench 2026-11-23
by sending `XY`, then `AUTO\r\n` — answered `ERR unknown command`, because it saw
`XYAUTO` — and the following `AUTO\r\n` answered correctly. So one bad line costs
one error reply and the parser resynchronises by itself (also written up in
`docs/HANDSET_PROTOCOL.md`). `reset_input_buffer()` does **not** help: it clears
our side, never the controller's. `hw_accept.py` and `web_probe.py` therefore
send a newline first and discard what it draws; a bare newline answered
`ERR unknown command` is the proof that bytes were pending, and both tools print
it instead of letting the first check of the run fail mysteriously.

**Same path, second trap: `ERR line too long` is rate limited to one per 2 s, and
in the suppressed case the line is not discarded** — the controller dispatches
the truncated first 64 bytes as a command. So the over-long case in
`hw_accept.py`'s refusal matrix is only meaningful because the matrix overflows
once; and an over-long `SET` that lands inside that window **takes effect** —
observed 2026-11-23 (`SET SP2=0.33` plus 120 junk bytes answered `OK SAFETY` and
`GET SP2` read back `0.330`; restored afterwards, and refused with the value
unchanged once the window had expired). Recorded as an open defect in PLAN 16 and
in `docs/HANDSET_PROTOCOL.md`: rate-limit the message, always discard to the
terminator.

**B. Bridge firmware (`handset/bridge_esp32`, recommended).** The spare board
runs a byte-for-byte pipe PC↔controller and *observes* the stream with the
handset client library:

```
pio run -d handset/bridge_esp32 -t upload
```

```
bridge GPIO16 (U2RXD) ← controller GPIO17   (controller TX)
bridge GPIO17 (U2TXD) → controller GPIO16   (controller RX)
bridge GND ───────────── controller GND     (power the bridge from its own USB)
bridge GPIO25 → LED link up        bridge GPIO26 → LED traffic
```

No pin contention, because the PC side stays on the module's own UART0
(GPIO3/GPIO1) and the controller sits on UART2 — the bridge's ROM boot log goes
to the PC, never into the controller. The traffic LED blinks at the `T,` rate
and the link LED is the **library's** verdict (3 s silence = down), so link
health is visible at a glance without touching the data stream. Build it as a
tap instead with

```
PLATFORMIO_BUILD_FLAGS="-DBRIDGE_TAP=1" pio run -d handset/bridge_esp32 -t upload
```

for a readable handset-side console (telemetry, `E,` events, replies, link-loss
messages) — the build for watching a dive from the handset side, and the base
for the real handset (`handset/example_esp32_handset`, same wiring). Both modes
live in one file, chosen by a `constexpr` flag rather than `#if/#else`, because
the Arduino preprocessor-generated prototypes do not understand conditional
blocks.

Two bench consequences: **opening this port resets the bridge, never the
controller** (its DTR/EN only touch the spare board — safer than the console
port), and the bridge needs ~250 ms to boot after that open, so
`hw_accept.py` settles 1.5 s and retries a command that got no line at all
(`--handset-settle 0` for a plain adapter).

### When the link is silent: bisect it, do not reflash at it

Pipe mode is **silent by design**, so silence cannot tell a healthy bridge from
a deaf one. Three tests, cheapest first, each answering a different question:

```
tools/port_id.py --listen 8                  # what is each port, really?
tools/port_id.py --loopback /dev/ttyUSB1     # bridge's own UART2, controller out of the picture
```

`--loopback` needs one jumper: **D16 ↔ D17 on the bridge board itself** (pins
labelled D16/D17, never TX0/RX0 — those are GPIO1/GPIO3, the console path).
Pipe mode hands back byte-for-byte what the PC sends, so an echo proves the PC
path, the sketch and the board's UART2 are fine and the fault is in the two
signal wires or the controller end. Remove the jumper afterwards: left in, the
bridge feeds its own TX back into its RX.

The tap build adds a **boot-time UART2 self-test** for the same question when
the PC side is not available: 8 bytes out U2TX, counted back on U2RX, printed
with the banner (`uart2 self-test: 8 bytes back on RX from TX: OK`). In pipe
mode nothing is printed to the protocol stream, so the verdict is on the LEDs
instead: 3 double-blinks = echo seen, 1 blink = none.

Firmware says the downlink is unconditional, so a silent handset port is a
wiring question, not a firmware one: `uart2_task.cpp` calls `Serial2.begin()`
and `UartLink::pump()` emits `T,` on the **first** pump and every 500 ms
after, with no handshake and no client required. A missing `T,` therefore means
the bytes never reached the PC — check, in order: controller end on **GPIO17**
(not the silkscreen `TX`, which is GPIO1 and prints `HB`), common GND, and the
bridge end on **D16**.

`tools/port_id.py` also keeps the two identical CP2102 boards apart: it labels a
port by what it *hears* at its own boot, so a spare that is merely relaying the
controller's `HB` lines is not mistaken for the controller (that mislabel once
nearly sent the bridge image into the wrong board — and the reverse mistake
would destroy a dive controller). Always pass `--upload-port` explicitly.

### Flashing this bench board (why it refuses, and the layout trap)

The board's USB bridge chip drives **EN but not IO0**, so esptool's auto-reset
cannot reach download mode: expect `Wrong boot mode detected (0x13)` unless the
**BOOT (GPIO0) button is held** through the attempt, or GPIO0 is jumpered to GND
and the board is power-cycled. `handset/bridge_esp32/platformio.ini` sets
`--connect-attempts=200` so you can hold the button whenever convenient instead
of racing the tool — esptool pulses reset on every attempt and gets in on the
first one where IO0 is low.

Never hand-write `firmware.bin` at `0x1000`. For a PlatformIO esp32 build that
file is the **application at 0x10000**; the four images are 0x1000 bootloader,
0x8000 partitions, 0xe000 otadata, 0x10000 app. Writing the app at 0x1000
overwrites the bootloader and the board boot-loops in the ROM stage
(`rst:0x10 (RTCWDT_RTC_RESET)`, never a second `load:` line) — which looks
exactly like a broken sketch. Use `pio run -d handset/bridge_esp32 -t upload`.
Do not pipe an upload through `head`: SIGPIPE can kill esptool mid-write.

Expected boot order (post-A5e firmware):

```
=== ESP-CCR controller M1 boot ===
reset reason: power-on
settings: N keys from NVS
control: SP=0.40/0.70/1.00/1.30 cells=111 wall=1.45 cutoff=1.60 ...
log: LittleFS mounted, N dives, seq a..b, used X of Y bytes
clock: unset: nothing has set the clock since power-up, dives will carry no date
web: AP "ESP-CCR-xxxx" on 192.168.4.1 (heap ...)
HB st=... (every 5 s)
```

`seq a..b` is the eviction evidence: a full FIFO always lists ten dives, so
only the sequence range says the NEWEST dive survived. On 2026-11-24 it did
not — every dive carried epoch 0 (no RTC), the eviction comparison tied, and
dives eleven and twelve both landed in slot 1, keeping the nine OLDEST dives
plus the newest (PLAN 16). `seq n/a` means every dive on the partition
predates the sequence field.

`clock:` after every boot is the dive log's calendar stated out loud. There is
no RTC chip and no NTP through the AP, so a client at the surface sets it —
the web page posts its phone's clock automatically on load, or `TIME SET
<epoch>` on UART2 (docs/HANDSET_PROTOCOL.md, Wall clock). The carry in RTC
no-init memory survives the resets the firmware performs on itself (panic,
watchdog, SAFE `ACK`); **EN and the battery pack both lose it** — measured
2026-11-24, the bench EN pulse boots as `rst:0x1 (POWERON_RESET)` and the
magic guard turns the leftover silicon-garbage into an honest `unset` rather
than a confident wrong date. So the bench rule is: **set the clock at the
surface after every power-up.** A dive started before that is still recorded,
with `clock-unset` in its CSV and an `E,,15` row in the file.
`hw_accept.py --dive-persist` asserts the chain it can: `TIME SET` accepted →
the closed dive carries a non-zero sequence **and a real epoch** → after the
EN reset the boot still states the clock (as `unset`, correctly, after a
power-down) and the dive's sequence is still in the `seq a..b` range.

Then by hand, in this order (each mirrors a sim scenario):

1. Phone joins the AP → status page live at <1 s, `NO DATA` banner within
   3 s of pulling power.
2. `DIVE START` from the phone → AP **gone** (radio off), `GET T` over
   UART2 still streaming, HB keeps printing.
3. `SET SP2=0.60` over UART2 mid-dive → refused; `SET GF_LOW=35` → OK and
   `E,` line + `SAFETY` reply (event-logged).
4. `DIVE END` → AP returns (it was on before the dive), log slot appears
   in `LOG LIST`, `/dive/<n>.csv` downloads and matches `LOG EXPORT <n>`.
5. `WIFI OFF` from the phone, dive, end dive → radio stays **off** (diver
   outranks the machine's auto-restore).
6. `GET T` over the handset link, then watch `stk=` in HB. On 2026-11-23 this
   exact command panicked the controller: the `uart2` task's stack canary fired
   inside newlib's `_dtoa_r` (seven `%f` conversions in one `T,` line, reached
   through a deeper call chain than the 2 Hz downlink uses). Fixed by formatting
   every protocol/log/console number in `util/numfmt.cpp` — no float printf on
   any bounded-stack task. The permanent tripwire is `stk=`: **free bytes in the
   tightest task**, appended to HB every 5 s. Below 512 the console also shouts
   `** stack low: N bytes free in '<task>'`. A soak that shows `stk=` trending
   down is a bug finding even when nothing crashes.
7. SAFE-mode recovery, with `esp32_stalltest` flashed: the supervisor reboots the
   board into SAFE (`HB st=3`), the handset link still streams (`T,` says `SAFE`),
   every command answers `ERR safe mode` — and `ACK` answers `OK SAFE RESTART`
   and brings the board back in STANDBY (`HB st=0`) without touching the power.
   That is the diver's escape route; a SAFE state with no reachable way out is a
   device you cannot recover in the water.

## C2-e — phone/web findings (2026-11-22 evening, all closed)

* "Password incorrect" **was not the password**: wpa_supplicant completed the
  4-way handshake with the correct PSK, then `key addition failed` from the
  kernel — a wedged iwlwifi/Intel 9260 after a kernel update removed the
  running kernel's modules. Reboot fixed it; the PC then joined `diveonly`
  first try, and the bare `softAP` probe proved the ESP side honest.
* **Captive DNS is load-bearing**: with the AP up but nothing answering :53,
  a phone resolves *nothing* and "the page does not load" — even though the
  302 redirect and the page itself were perfect. Fixed: DNSServer answers
  every A query with the AP IP (library wildcard is `"*"`, not `""` —
  verified empty-domain replies came back malformed on the wire), pumped by
  hudTask (NOT the handset task: UART2 may be unwired on the bench, task
  never created).
* **Reply bursts stall the link**: every ws reply line used to be its own
  frame — GET SCHEMA put 33 tiny TCP segments on the wire at once, and a
  >13 s telemetry hole followed. Replies are now batched into ≤1 KB frames
  (~2 frames for SCHEMA); protocol and `#<n>` marker unchanged; page splits
  frames on newlines as before. Telemetry pull cadence 600 ms -> 2 s.
* The SCHEMA fetch is socket-open-only (never re-fetched; saves reuse the
  cached copy), so reconnects no longer stampede.
* **`ERR settings not saved` was an 18-character key**: NVS rejects keys over
  15 chars, per-write, deep inside `save()` — and the native suite never
  touches NVS, so only the bench could catch it. Two offenders renamed
  (`CELLPROOF_TMO`, `CELLPROOF_MAXSP`), `test_settings` now pins the 15-char
  limit, schema marker bumped. Verified on the board: SET -> OK in ~40 ms,
  reboot, boot line `settings: 32 keys from NVS`, values survive.

## C2-f — the page the board served was not the page in the repo (2026-11-23)

Probing the web stack over a joined station (`tools/web_probe.py`, above) turned
up a defect that no amount of flashing or code reading would have shown:

* **`board_build.filesystem` was never set**, so PlatformIO built **SPIFFS**
  images for `uploadfs` while the firmware mounts **LittleFS**
  (`LittleFS.begin(true)`, `src/log/dive_store.cpp:39`). Every `uploadfs` until
  this session wrote a SPIFFS-formatted image into the partition LittleFS lives
  on, so `data/index.html` had *never actually been installed* — and BENCH.md
  had been calling that step "optional" the whole time. Fixed in `platformio.ini`
  (`board_build.filesystem = littlefs`); the build log now says
  `littlefs.bin`, not `spiffs.bin`.
* What was in the partition at the time — a 4 219-byte `index.html` plus a
  9 114-byte `/app.js` that speak JSON and `/api/status`, an API this firmware
  has never implemented — was **leftover from the mismatched installs, i.e. an
  artifact of testing, not a design defect**: owner decision 2026-11-23, drop the
  "LittleFS could shadow a firmware fix" worry unless it shows up again. The
  `web_probe.py` byte-exact check stays, because that is what noticed it. Real
  housekeeping that came out of the same look: `web/` is a superseded page
  generation whose assets match nothing in the router (`/style.css` is
  registered and the repo never had that file; `/app.css` and the four
  sub-pages have no routes).
* **Page and dive logs share one partition.** `uploadfs` writes that partition's
  whole image, so it deletes dive history. Observed, not just inferred: a
  338-byte dive sat in slot 1 before the first real install; after it,
  `/dive/1.csv` was one of that session's 3-record bench dives and slot 2 was
  empty — if the older dive had survived, `nextSlot()` would have used slot 2.
* The check that now catches this class is in `web_probe.py`: **the bytes on the
  wire must equal the bytes in `data/index.html`**. Cheap, and it is the only
  assertion that compares the three artifacts (`web_page.h` → `sync_webdata.py`
  → `data/` → what the router picks) end to end.

So: the page never needs installing — the built-in copy in app flash is the one
that answers (`serveIndex` in `src/comms/web_server.cpp`, decided 2026-11-24), and
`web_probe.py`'s byte-exact check against `data/index.html` is what proves the
page in the repo, the page in the image and the page on the phone are the same
bytes. The LittleFS copy of the page is now dead weight: it is never read. The
board being flashed still holds the older 25 445-byte copy from the mismatched
installs while serving the current 25 642-byte one — that difference is the
preference bug, and it is the reason the preference is gone.

One check that had to be repaired on the way: `/` was graded as "contains `<html`
within the first 200 bytes". A comment at the top of the page pushed the tag to
byte 213 and the check went red on a page that was fine — it was asserting the
page's first two hundred bytes, not that the response is a document. It now
requires a doctype and an `<html` tag inside 4 KiB, and was tried against an
empty body, a plain-text error, a stripped doctype, a missing `<html`, and HTTP
500, failing all five.

## C3-a — depth sensor: LPS28DFW on its own bus (decided 2026-11-23)


The BMP085 that stood in here **never acknowledged once** — every poll since
the first boot that tried it was `requestFrom(): Error -1`, and it is now
removed from the firmware entirely. Depth is now claimed at **boot by
identity**, not by build: the dive-rated **MS5837-30BA** is preferred, the
**Adafruit LPS28 breakout (LPS28DFW, #6067)** is the bench/wet-interim
fallback. Whichever passes its identity gate (LPS28: WHO_AM_I `0xB4`;
MS5837: reset + factory PROM CRC-4) owns depth **until the next power
cycle** — mid-dive the source never changes, and a forged or corrupt chip
is refused outright. Fit a sensor, power-cycle, read the boot line.

The depth bus is **Wire1, its own pin pair (SDA 19 / SCL 18)** — off the
cell bus on purpose: a wet-harness short or latched depth chip can never
wedge the bus feeding ppO2, and each bus bit-bang-recovers its own pins
(same rescue, both buses).

Wiring:

- [ ] VIN → **3V3** (the breakout regulates; do not feed 5 V into a 3V3 bus),
      GND to the ADS1115 GND point, **SDA→19, SCL→18**. STEMMA QT cable fits.
- [ ] Pull-ups are per-bus now: the 4.7 k pair on 21/22 does **not** serve
      18/19. The breakout's on-board 10 k works at 100 kHz on the bench;
      add a 4.7 k pair on 18/19 for the wet harness.
- [ ] Boot must show `depth: LPS28DFW @0x5C ready, <n> hPa` once (or the
      MS5837 line if it is fitted — it wins on ties), and then silence: no
      periodic `requestFrom()` error lines, HB `stale=` drops out of the
      alarm word, `al` should fall toward `0x0010` (battery only). The
      sensor that booted is the sensor that dove — the line is part of the
      build record.
- [ ] Squeeze test: press the port gently (or breathe through a tube into
      the metal port) — the dashboard depth/pressure must move and come
      back. The driver runs **FS_MODE=1 (260–4060 hPa ≈ 40 m sea water)**
      so the 10 m cell-proof pressurization is inside range; mode 0 would
      clip at ~2.4 m and silently flat-top.
- [ ] Then `CAL DEPTH` (surface tare) and re-check the pre-dive list.
      **The tare belongs to sensor+package**: after any sensor swap, re-tare.

Until the sensor is fitted the controller behaves correctly with no depth —
`HB … al=0x0090` includes `BIT_SENSOR_STALE` — it just refuses to pretend.

## C3-b — MS5837-30BA swap (before ANY wet test)

- [ ] Potting inspection: gel-filled port free of voids, epoxy not lifted
      at the rim; any wicking under the seal = scrap, not rework.
- [ ] Breather orientation per datasheet; if the case gets condensation in
      a cold-water sim bucket, the potting is already suspect.
- [ ] I2C: address 0x70 (verify with the boot line, not a datasheet),
      10 kΩ pull-ups, leads twisted, ≤30 cm — same bus-recovery drill as
      the M1 soak (yank SDA/SCL, expect `i2cerr/rec` move, **no reset**).
- [ ] Cross-check vs BMP085 on the same standpipe: 0.0 / 2.0 / 5.0 m, both
      readings within 0.3 m; MS5837 hysteresis checked by going up and
      down through 3 m.
- [ ] Firmware still refuses a bad bus: cover the sensor, expect
      `stale`/`SENSOR` alarm, injection stops, valve stays shut.

## C3-c — cells, grounding, battery

- [ ] All three cell grounds and the ADS1115 GND on one point; handset
      powered from the same rail (UART common ground) — a ground loop here
      fakes exactly the drift the voting layer hunts.
- [ ] Cell count is a BUILD decision: `CELLS_PRESENT_MASK` in
      `include/config/defaults.h` (0x07 = three fitted since 2026-11-22 —
      the dashboard, voting and histograms follow it exactly). Re-check it
      whenever the rig changes, and run CAL AIR after any change.
- [ ] Battery: HB `vbat=` within 0.1 V of a meter at three loads;
      `VBAT_MIN` alarm fires with the meter confirming; log the divider
      trim for the build.

## C3-d — mechanical inhibit (zero firmware cooperation)

- [ ] Actuate the inline manual O₂ inhibit with the board injecting:
      solenoid LED and `pulses=` keep counting, loop ppO₂ falls — the
      diver's bailout must work identically with the firmware brain-dead.
- [ ] Solenoid driver fails **closed**: signal lead unplugged → no O₂.

## M2/M3 bench sequence (once control firmware is flashed)

1. `CHECK` → `PRE,` steps pass, `AUTO` unlocks (a cold boot refuses it).
2. `TEST CELLS` on the jar → masks `007` with three live cells.
3. `CAL AIR` in air; `CAL O2` two-step on the bench jar at `CAL_O2_FI`.
   *Done 2026-11-23, firmware `m2-depth-detect`+cal air:* first bench CAL
   AIR — the command had been acknowledged by the router but nothing in the
   firmware consumed it; the sampler (`cal::AirCal`, 1 Hz × 12, spread-limited)
   closed the gap. Result recorded: raw cell0=1503 cell1=1506 cell2=1511
   counts, worst spread 12 (band 40), vote then 0.210–0.212 = true air.
   Refused before warmup (120 s); the air point is volatile by design —
   CAL AIR belongs to every dive session, and CAL O2 (slope) comes later.
4. Descend the jar: SP bands cross at the right depths (±0.2 m),
   CellProof arms >`CELLPROOF_DEPTH` and passes with honest cells.
5. Fault drills the sim already proves the firmware should survive:
   disconnect one cell (drift exclusion), yank I2C mid-dive (injection
   stops, no reset), stick the valve (stuck-open alarm, duty cap).
