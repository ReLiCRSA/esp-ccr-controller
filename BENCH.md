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
fuser /dev/ttyUSB0 /dev/ttyUSB1   # must be empty; a reader here corrupts the run
pio test -e native            # host gate, currently 258 cases
tools/float_audit.py          # no float printf in src/ except marked boot-only sites
pio run -e esp32 -t upload    # only once the soak has ended
tools/.../python tools/hw_accept.py --expect-web --expect-depth \
                                 --handset /dev/ttyUSB1 \
                                 --cal-air --predive --dive-persist    # 136 checks
tools/.../python tools/hw_accept.py --selftest   # the log/depth/ruler/dive-file graders, no board: 54 cases
tools/.../python tools/web_probe.py --handset /dev/ttyUSB1 --dive-cycle  # 62 checks
```

`--expect-depth` is how the operator says *this bench has a depth sensor fitted*:
it turns C3-a's boot line into three graded checks. Without it, a boot with no
`depth:` line at all is accepted — the absence is the honest signal for "nothing
fitted", and it must stay that way.

The run also grades **the depth ruler itself**, and that is not the same check as
the scale being right. `WATER_DENSITY` is dive-locked (it is the one density behind
every depth the board publishes, so a mid-dive change moves the diver's scale
~2.5%), and the matrix refuses a mid-range write, reads the value back *during* the
dive, restores it at the surface and reads it again. Four checks, and the mid-dive
read is the only one of them that can see a leak — the surface restore hides it from
every later read. See the C3-a section "The ruler is dive-locked".

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

`--dive-persist` also **watches the link while its dive runs** and then grades
the dive it just closed against what the handset saw live: every `E,` row that
streamed past must be in the file, in the order it happened, and the `M,` summary
must agree with the rows beside it (its snapshot and event counts equal the
`S,`/`E,` rows actually exported, and its `gf_low`/`gf_high`/`sp1..4` equal what
`GET GF_LOW`…`GET SP4` answer on the same link). The watch starts the moment
`DIVE START` goes out — a capture that begins after a pause misses the dive's own
opening rows and leaves the comparison against nothing, which is a check that can
never fail. It is deliberately one-way: rows streamed *outside* the dive window
have no file row, correctly. `hw_accept.py --selftest` grades those graders with
real captures — an honest dive exported off the fixed build, and the clamp dive
of 2026-11-24 recorded before the fix, whose file held 13 samples, no verdict
rows and an `M,` line claiming 0 of each.

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
seq a..b, used X of Y bytes`, a `surface: N records, seq a..b` line and a
`clock:` line) and then runs the
**refusal matrix**
over the wired handset: `AUTO` cold → `ERR predive required`; `SET SP2`
mid-dive → `ERR dive locked`; `SET WATER_DENSITY` mid-dive → `ERR dive locked`,
then `GET WATER_DENSITY` **still mid-dive** to ask which ruler that dive is
actually running on, and the same write accepted after `DIVE END`; `WIFI ON`
mid-dive → `ERR wifi locked in dive`; `CAL AIR` mid-dive → `ERR cal locked in
dive`; overlong line → `ERR line too long`; junk → `ERR unknown command`. Every
one of those is a host-tested invariant — the bench exists to prove the wire, not
the logic. Two rules this matrix now follows, both learned the hard way (see C3-a
"The ruler is dive-locked"): a refusal probe never uses a **range endpoint** (a
boundary value can be refused by a second rule, and the entry passes for the wrong
reason — that is exactly what hid a dive-lock regression for one run), and a
locked write is graded by **reading the value back while the refusing state still
holds**, never by trusting the `ERR`.

The `log:` boot line is a check, not decoration. A LittleFS mount that fails is
answered with a format, which erases every dive on the partition, and the core
does that in silence; the line says how many dives the mount found and warns if
it had to format. A dive computer that quietly eats its own history is worse
than one that says the log is offline.

The `surface:` line is a check for the same reason and a newer one. The surface
log (calibration verdicts and pre-dive results, `/surface.bin`, §"The surface log"
below) has no dive file and **no handset command**, so that boot line plus
`GET /surface.csv` are the only two ways anyone can see it after the moment the
verdict happened. It prints in all three states — records with a seq range,
`no records yet`, or `** /surface.bin will not read` — because an unreadable log
and an empty one otherwise look identical, which is the exact confusion the line
exists to prevent. Absence of the line is a FAIL, and a replay of a pre-feature
build's console log proves the check can fail.

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
| HTTP | `/` serves and is **byte-identical to `data/index.html`**. This is the check that found the page had never actually been installed (C2-f below). `/favicon.ico` is 204, unknown paths 302 → `/` (that redirect *is* the captive portal), `/app.js` 404 because the page is one self-contained file, `/surface.csv` 200 whose **first line is the `SURFACE,` identity row** (a file that starts with a data row has no count and no seq range, so a shed history reads as a complete one) |
| Surface log | the export graded against itself: identity count equals the rows delivered, the seq range spans them, every row carries all 13 columns, each known id carries its own name, an unknown id arrives as `id<n>` rather than being dropped, and a record with epoch 0 renders `clock-unset` instead of a 1970 date. Captured 2026-11-24: `SURFACE,1,11,1,11` over 11 rows, two of them `pre_dive_fail` at step 2, three `cal_air` rows with `value 0` — those are the history the mutated build (see below) wrote, and the export shows them rather than hiding them |
| Page (surface) | it can read the surface log (`/surface.csv` is referenced) and renders a record as words — `shed by capacity` and `pre_dive_fail` both present — because a page that shows a bare `16,4` has recorded the fact and lost the meaning |
| Page (self-contained) | references nothing external but itself. The AP has no uplink, ever, so any CDN or cross-site reference is a dead page on the day it matters |
| DNS | any name at all resolves to the AP IP — the other half of the captive portal (C2-e above: without it a phone resolves nothing and "the page does not load") |
| WebSocket | handshake verified against RFC 6455 (`Sec-WebSocket-Accept` recomputed locally, no library: the point is to verify the server, so a dependency would be verifying itself); the `#boot <uptimeMs> <freeHeap>` hello on connect; then every command's answer must end with a `#<n>` marker whose **n equals the answer's line count** — if it does not, the browser renders half a table |
| Equality | each command sent twice **back to back** — once over the WebSocket, once on UART2 — with identical non-stream reply lines. That is the project's central claim (PLAN: one command session, not one per transport) |
| `--dive-cycle` | `DIVE START` from the browser takes the radio out from under the associated client (HTTP stops answering, the socket dies), the dive is then closed over UART2 — the web cannot close the dive that switched itself off, which is itself the argument for the handset being the in-water transport — and the AP is broadcasting again with a fresh WebSocket working |

Measured 2026-11-24 with the surface log, 62 checks, 0 failures; on 2026-11-23,
34 checks, 0 failures: radio unreachable under the client
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

**Same path, second trap, now closed (fixed 2026-11-24): `ERR line too long` is
rate limited to one per 2 s — and until the fix, in the suppressed case the line
was not discarded**, so the controller dispatched the truncated first 64 bytes as
a command. An over-long `SET` landing inside that window **took effect** —
observed 2026-11-23 (`SET SP2=0.33` plus 120 junk bytes answered `OK SAFETY` and
`GET SP2` read back `0.330`; restored afterwards, and refused with the value
unchanged once the window had expired). `proto::LineReader` now drops an
over-long line whole, terminator included, and the rate limit gates only the
error text (`docs/HANDSET_PROTOCOL.md`, PLAN 16). `hw_accept.py` has a framing
phase for it, and the phase is worth reading as a bench procedure: it reads the
baseline `SP2`, sends an over-long `SET SP2=0.33 <junk>` in a fresh error window,
sends a second one **inside** that window, then reads `SP2` again — the correct
answers are one `ERR line too long`, then silence, then the unchanged value.
Two lessons from proving it. The junk must start with a **space**: glued straight
onto the digits (`0.31ZZZ…`) the settings parser rejects the value as "not a
number" and the "did it write?" check passes for the wrong reason. And the phase
only means something if it can fail — flashing the pre-fix semantics made the
bench reproduce the 2026 observation exactly (`ERR unknown command` from the
line's own tail, then `OK SAFETY` and `SP2=0.310` written in the muted window).

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

### Calibration honesty: the reply is a receipt, so the verdict is its own row

`hw_accept.py` asks the calibration questions on the handset link and fails if the
machine implies work it did not do:

```
GET CALAIR        -> CALAIR,0,0,0,0,0,0,0,0     (nothing asked: code 0, not a result)
CAL DEPTH         -> ERR not implemented        (no consumer, so refused)
CAL O2            -> ERR not implemented
CAL O2 CONFIRM    -> ERR not implemented        (no two-step arm to confirm)
CAL AIR           -> OK CAL AIR                 (a receipt: nothing is claimed here)
GET CALAIR        -> CALAIR,3,<age>,0,0,0,0,0,<left>   (refused: still warming up)
```

plus two checks over the **whole** handset stream rather than one reply: an
unsolicited `E,<t>,7,3` row must have arrived, and no `E,<t>,7,2` (which would
claim a calibration that was refused). The console watch adds the counterpart —
`cal air: REFUSED` must appear there too, and `cal air: sampling` must not follow
the refused request. Two records of one event that cannot disagree is the whole
point of the change.

The full verdict leg costs about two minutes and is opt-in:

```
tools/.../python tools/hw_accept.py --expect-web --handset /dev/ttyUSB1 --cal-air
```

It does not guess the 120 s warm-up clock: it asks inside the gate, reads
`warmup_left_s` back off the protocol and waits exactly that long — the honest
refusal schedules the check. Then it samples and asserts `CALAIR,2,0,12,<spread>,
<raw0>,<raw1>,<raw2>,0` arrives, that it ended unprompted as `E,<t>,7,2`, and that
the console's `cal air: OK raw cell0=… worst spread … (12 samples)` carries **the
same numbers** as the wire line. Captured on 2026-11-24:

```
E,129,7,2
CALAIR,2,2,12,21,1514,1519,1528,0
cal air: OK  raw cell0=1514 cell1=1519 cell2=1528  worst spread 21  (12 samples)
```

`web_probe.py` compares the refusals and `GET CALAIR` across the two transports
byte for byte, and it fails the run if the served page still offers a button for
`CAL O2` or `CAL DEPTH`: a control that does nothing on a machine that dives is a
fault, so the page carries `CAL AIR` plus `CAL AIR status` and shows the verdict
in words. It leaves the cells calibrated until the next reset, which changes what
the pre-dive list says — expect that, do not chase it.

### The surface log: a verdict that outlives the moment (`--predive`, `--selftest`)

A failed pre-dive check is the cleanest case in the project for "the machine must
write it down": the dive it prevents never happens, so there is **no dive file**
that could hold the row, and until 2026-11-24 the only trace of "the cells were
refused for warm-up" or "the checks failed on battery" was a console line nobody
keeps. The records live in `/surface.bin`, export over `GET /surface.csv`, and are
**web/HTTP only** — there is deliberately no handset command for them (PLAN 16,
spec §5.1 and §8 state the cost rather than hiding it). So the bench has to check
the flash, not just the wire.

```
tools/.../python tools/hw_accept.py --expect-web --handset /dev/ttyUSB1 --cal-air --predive
```

`--cal-air` ends by reading `/surface.csv` back and requiring that the newest
`cal_air` record carries **the same numbers as the `CALAIR,` line the handset was
answered with** — value, samples, spread and all three raw counts — plus a date
(the run stamps the clock first, so `clock-unset` here means the record was made
by a machine that had lost its clock). `--predive` then runs the pre-dive checks
for real, reads the verdict twice (polled `PRE,` on the handset, then the row out
of flash) and requires both to name **the same failing step**. It pulses the
solenoid on the bench, so the jar and the wiring must be as you want them; run it
after `--cal-air` and the 120 s warm-up is already paid for.

The sequence aborts at its first failure, so the step it names is whichever one
the bench actually defeats. As configured today (no pressure rig on the solenoid,
battery pack unplugged) the verdict is `pre_dive_fail` at **step 2, solenoid** —
the battery step never runs, so do not go looking for a battery row. That is the
honest state of the machine, not a harness fault: what is graded is that the
record and the protocol name the same step, not that the check passed.

Four records of one event, and the checks are that they are the same event:

```
~ [154.82s] surface: record 10 id 7 value 2       <- console; its seq is the join key
CALAIR,2,1,12,13,1514,1518,1521,0                 <- handset answer
E,129,7,2                                         <- unprompted protocol row
R,10,1795528182,2026-11-24T13:49:42Z,7,cal_air,2,1514,1518,1521,13,12,0  <- /surface.csv
```

The seq is read back off the file (max + 1) rather than kept in RAM, so the
console's number *is* the CSV row's number, and the join is on the number rather
than on "some record was written this run".

**`--selftest` grades lying logs, no board needed.** Twenty-one cases: twelve of
a surface export that is wrong in exactly one known way — header claims 3 records
and 2 arrive, no identity row at all, a seq range describing a different file,
this run wrote nothing, `raw0` disagreeing with the wire, a warm-up refusal
recorded as OK, a failure filed under the pass id, an appended id with no name,
a record with no clock rendered as a date, the console never reporting the write,
and the console reporting a *different* record number — and nine lying boots for
the depth announce: an honest boot with and without `--expect-depth`, the
placeholder `ready, 0.0 hPa` (the real defect), an unidentified chip holding the
depth slot, a `no sane pressure` line the alarm word contradicts, a `ready` line
the alarm word contradicts, a claimed sensor that never announced, a sensor-less
boot that is honest, and the identified-but-deaf boot that is honest *because* it
raised the alarm. Run it after touching the graders or the record format; a case
that stops failing means that check is decoration.

Two harness bugs it caught on the day, both of which read as firmware faults:

* **Anchoring `SURFACE_LINE` with `$`.** The empty-log line goes on to name the
  export — `surface: no records yet (GET /surface.csv will be empty)` — so an
  anchored pattern matched **none of the three forms** and the run reported the
  boot as silent. Unanchored, with the reason written down at the pattern.
* **A page-honesty regex that the page satisfied without the feature.** "renders
  records as words, not codes" passed on the *previous* page, where `pre-dive
  checks` came from the CHECK button's label and `no calibration since power-up`
  from the checklist view. Tightened to `shed by capacity` + `pre_dive_fail`,
  and verified against `git show HEAD:data/index.html` — the old page must make
  it red, which is the whole test.

The firmware side is mutation-proven on hardware, not just in host tests: with
`r.value = 0` in `surfaceCalAir`, exactly one bench check goes red —
`surface log: the cal_air record carries the same numbers as the wire: value=0
want 2` — and the record's own id/name row still passes, because the row is
internally consistent. That is the check that earns its place. The same is true of
the persistence leg: with `SurfaceStore::append` opening `"w"` instead of `"a"`
(so every write truncates the log to its newest record), exactly one check goes
red —

```
[FAIL] every surface record written before the reset survived it: boot after reset:
'surface: 1 record, seq 23..23'; the first boot said 19 records and this run wrote
4 (seq 20..23), want at least 23
```

— and every other surface check stays green, because the one surviving record is
internally consistent. A log that quietly keeps only its last row is invisible to
anything that reads the file; only the boot banner's own arithmetic sees it.

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

- [x] VIN → **3V3** (the breakout regulates; do not feed 5 V into a 3V3 bus),
      GND to the ADS1115 GND point, **SDA→19, SCL→18**. STEMMA QT cable fits.
      (Fitted and proven 2026-11-24. It was first wired wrong: the symptom was
      `requestFrom(): Error 263` — `ESP_ERR_TIMEOUT`, a transaction that starts
      and never finishes — rather than the clean `Error -1` of an absent chip,
      and the depth retries were long enough to make cell channels go stale
      between them. Wrong wiring, not a finding; corrected and re-measured.)
- [x] Pull-ups are per-bus now: the 4.7 k pair on 21/22 does **not** serve
      18/19. The breakout's on-board 10 k works at 100 kHz on the bench;
      add a 4.7 k pair on 18/19 for the wet harness.
- [x] Boot must show `depth: LPS28DFW @0x5C ready, <n> hPa` once (or the
      MS5837 line if it is fitted — it wins on ties), and then silence: no
      periodic `requestFrom()` error lines, HB `stale=` drops out of the
      alarm word, `al` should fall toward `0x0010` (battery only). The
      sensor that booted is the sensor that dove — the line is part of the
      build record.
      **This bench answers at `@0x5D`, not `0x5C`** — the XSA pad is high. The
      driver probes `0x5C` then `0x5D` and prints which one answered, so the
      line names the real address; check the pad if that was not deliberate.
      Measured 2026-11-24: `depth: LPS28DFW @0x5D ready, 1016.7 hPa`, one
      `requestFrom` error in 75 s (the MS5837 probe inside the preferred
      window, expected), 15/15 HB lines `fresh=111 i2cerr=0 rec=0` and
      `al=0x0010` — battery only, so `BIT_SENSOR_STALE` is clear, which is the
      proof the pressure is sane: `ai.sensorStale = !cellsFresh || !pressureValid`.
- [x] Squeeze test: press the port gently (or breathe through a tube into
      the metal port) — the dashboard depth/pressure must move and come
      back. The driver runs **FS_MODE=1 (260–4060 hPa ≈ 40 m sea water)**
      so the 10 m cell-proof pressurization is inside range; mode 0 would
      clip at ~2.4 m and silently flat-top.
      **Done 2026-11-24, on the handset wire (`T,` rows, 500 ms cadence, 286
      samples).** Baseline 0.04 m steady for 44 s; press 1 ramped to 1.55 m and
      then 3.08 m, held ~25 s, released to 0.04 m within 1.0 s; press 2 peaked at
      **5.57 m ≈ 560 hPa**, held 25 s, released to 0.04 m in 0.5 s. Release
      undershoots nothing. The slow decay while held (5.57 → 5.09 m over 25 s,
      ~1.9 hPa/s) is the finger seal, not the sensor.
      It also proved two things nobody asked for: `sp=` in the same `T,` row went
      **0.400 → 0.700 at 2.67 m and back to 0.400 at 0.47 m** — both hysteresis
      edges of `SP_IN_WATER_M=2.0` ± `SP_HYST_M=0.5`, sighted by accident and
      both correct — and `mode=STANDBY`/`al=0x0010` throughout, so depth alone
      starts neither dosing nor alarms.
      **Still the only check that proves the sensor measures.** Identification,
      a plausible hPa and a cleared alarm bit all prove it *talks*; at the
      surface, depth is 0.04 m whether the port is open to the room or taped
      shut. Do it over the handset, not by eye: `/tmp`-style observers aside,
      the check is that `T,` depth moves AND the setpoint tier follows it.
- [ ] Then `CAL DEPTH` (surface tare) and re-check the pre-dive list.
      **The tare belongs to sensor+package**: after any sensor swap, re-tare.
      It still answers `ERR not implemented` — now that a sensor exists, the
      tare has a consumer and lands with one. The offset it must remove is
      measured, not guessed: `SURFACE_PRESSURE_BAR` is a fixed 1013.0 hPa while
      this bench sits at 1016.7 hPa, so a correct build reads **0.04 m at the
      surface** (it was 0.04 m on the wire all through the squeeze test). Any
      surface check needs a tolerance (±0.5 m), not "exactly zero".

**Depth was overstated 1000×, and the sensor fitting is what exposed it
(fixed 2026-11-24).** The day the LPS28DFW first went live, the handset read
`T,37.04,…,STANDBY` on a bench sitting in an open room, and HB's `sp=` moved
from `0.40` to `1.00`: `BAR_PER_M_SEAWATER` was `WATER_DENSITY_KGL * 9.80665 /
100000` with the density in **kg/L**, but `ρg` is Pa/m only for **kg/m³**, so the
constant came out 1000× too small (1.005e-4 bar/m instead of 0.1005) and every
depth 1000× too deep — 3.7 hPa of ambient weather became 37 m, which crossed
`SP_IN_WATER_M=2` and `SP_DEEP1_M=30` and selected the **SP3 1.00 bar** tier at
the surface: 2.5× the intended ppO₂, plus `CELLPROOF_DEPTH` "crossed" instantly
and the deco engine fed 37 m. The constant's own comment said the right thing
("1 bar of sea water (1.025 kg/l) = 9.947 m"); the expression contradicted it.
Nothing in the host suite could see it: **no test referenced `BAR_PER_M_SEAWATER`
at all** — the `p → depth` line lives in `main.cpp`, outside every tested unit —
and a test that computed expected depth *from the macro* would inherit the error
anyway. Recorded the same day as an open test gap; **closed later the same day,
and closing it found a second defect** (see "one conversion, one density" below).
`src/deco/zhl16c.cpp` has always had the conversion right (`waterDensity * 1000.0
* 9.80665`, written out three times), which is the second reason this was
invisible: the two paths disagreed and only one of them feeds the `T,` row.
Why it hid for so long: no depth sensor had EVER been valid on this bench, so
`pressureValid` was false and `depthM` was pinned to 0 in every log ever taken.
A constant that is only wrong once its input exists.
Verified on the wire, both directions: same squeeze, `T,0.04` and `sp=0.400` at
the surface (predicted 0.037 m from the announce's independent 1016.7 hPa, and
a thumb press worth ~100 hPa read 1.0 m, not 1000 m).

**One conversion, one density — the test gap closed, and what it found
(fixed 2026-11-24).** The gap was "no test touches the conversion". Writing that
test meant reading both call sites again, and they were not the same
measurement: `main.cpp` divided by the compile-time `BAR_PER_M_SEAWATER`
(`WATER_DENSITY_KGL`, 1.025), while the deco engine used `cfg_.waterDensity` —
which `SET WATER_DENSITY` overwrites (schema range 0.990…1.100, so a diver in a
fresh-water quarry sets 1.000). One ruler said 9.948 m/bar, the other 10.197:
**2.5% at every depth**, so the depth on the wire and the setpoint bands came
from one scale and the ceiling, NDL and tissue model from another. Neither side
was wrong in its own terms, which is why no comparison the machine makes with
itself could see it.

The fix is one formula in one file, `src/util/depthscale.h` (`barPerM`,
`depthFromBarDelta`, `ambientBarAtDepthM`, `depthFromAmbientBar`,
`depthFromSensorBar`), `constexpr`, and called by both paths: the `T,` depth line
in `main.cpp`, and all three `rhoG` sites in `zhl16c.cpp` (including the two
private lambdas the NDL binary search and the stop-grid walk build with).
`BAR_PER_M_SEAWATER` is **deleted** from `defaults.h` — a constant nobody is
allowed to call is worse than none, and the comment that lived there is now a
warning about the units. The density is now also single-source: the `WATER_DENSITY`
setting, via `cfg.deco.waterDensity`, at both sites. (The correct figure that used
to headline that comment, "1 bar = 9.947 m", is 9.948 m at 1.025 kg/L.)

Host gate, `test/test_depthscale`, 6 cases, and **every expected number is a
hand-worked literal** (`0.1005181625` bar/m at sea, `0.0980665` fresh, 9.94844 m
per bar, `0.0368` m for this bench's surface offset), never derived from the code
under test — round-trip tests cannot catch a scale error at all, because a wrong
constant round-trips perfectly. Proven by mutation, host side:

| injected bug | what reddens |
|---|---|
| `/1e5` again (the original unit error) | 3 cases: `Expected 0.1005182 Was 0.0001005182`, `Expected 0.0368 Was 36.80915`, and the deco cross-ruler case |
| density argument ignored, 1.025 hard-coded in the helper | the fresh-water anchor (`Expected 0.0980665 Was 0.1005182`) |
| `zhl16c.cpp` writes its own `rhoG` again | the source scan (`re-derives the scale (9.80665)`) |
| the NDL binary search's own depth grid handed a literal density | the scan's second rule (`calls depthscale::ambientBarAtDepthM( without the configured density`) — **and nothing else** |
| the 3 m stop-grid walk's depth grid handed a literal density | the same scan rule, alone |
| `main.cpp` uses `WATER_DENSITY_KGL` rather than the live config | the same scan rule, named for `main.cpp` |
| the never-negative floor dropped for the diver's depth | `Expected 0 Was -0.02984535` |

The middle three rows are the honest limit of the numeric coverage, stated
plainly: the two private depth grids inside the deco engine (the NDL binary
search and the stop-grid walk) are guarded by the **scan**, not by an assertion
about a plan. Their published numbers are quantised — stops on a 3 m grid, the
`D,` ceiling in whole metres — so a plan-level "fresh versus sea must move the
ceiling" probe was written, measured (the ceiling does move: 11.15 m at 1.025,
11.43 m at 1.000 for the same 30 m/25 min profile), and then **dropped**: no
mutation made it the only witness, since the ruler test or the scan already
caught every subset of call sites. A check that can never be the only witness is
decoration. What is therefore pinned: one formula, one density reaching every
call, and the two public conversions (`barToDepthM`/`depthToBar`) numerically at
three densities. What is **not** pinned: exact stop minutes and NDL at a density
other than the shipping 1.025 — `test/test_deco` pins schedules at 1.025 only.
If fresh-water deco shape ever matters commercially, that is the test to write.

The last row is worth repeating as a toolchain trap: `TEST_ASSERT_EQUAL(0.0, …)`
in this repo's Unity is the **integer** variant, silently truncates, and passed on
`-0.0298` while the bug was live. Float assertions here must be
`TEST_ASSERT_FLOAT_WITHIN` / `_EQUAL_FLOAT` / `_GREATER_THAN_FLOAT` — and this
Unity is built **without double support**, so `TEST_ASSERT_*_WITHIN_MESSAGE` on
doubles fails to compile ("Unity Double Precision Disabled").

Bench side, the property is now a C2 check rather than arithmetic in a script:
`hw_accept.py` reads the boot announce's hPa and the first `T,` row's metres and
requires them to be the same sample, on the density the diver set
(`GET WATER_DENSITY`, now also in the protocol matrix so a client can read the
ruler back — it answered `WATER_DENSITY=1.025`), tolerance 5 cm of water —
~0.5 hPa, generous for weather between the two rows and three orders of magnitude
tighter than the error being hunted. The tool states the physics itself
(`depth_from_bar_abs`), it does not import the firmware's formula, or it could not
catch the firmware's formula being wrong. Four grader mutations each reddened
exactly their own case: the tool's scale given the old `/100000` (the honest boot
goes red), the tolerance widened to 50 m (the 1000× capture stops failing), the
never-negative clamp removed (the altitude pair goes red), and a silent
`return []` instead of failing when no `T,` row arrives (the vacuous-pass case).
The gate at that point was **129 checks, 0 failures**; it has grown since (below).

**The ruler is dive-locked, and a refusal is only half a check (2026-11-24).**
Single-sourcing the density made `WATER_DENSITY` the one number behind the metres
in `T,`, the setpoint tiers those metres select, the cellproof arm depth and the
deco engine's ambient pressures. So a mid-dive `SET WATER_DENSITY=1.000` — legal,
inside 0.990…1.100 — moves the diver's whole depth scale ~2.5% at a pressure that
has not changed. That is a scale change under a diver's feet, and setpoints and
alarms are already locked for less than it, so the key is `FL_DIVE_LOCK` now
(spec §6 documents it as a refusal, and bit 0 of the `K,` line is what makes the
page grey the input out mid-dive without knowing the key's name).

The bench leg refuses to stop at the refusal's words, because `ERR dive locked`
only reports what the router *said*. A router that prints the refusal and writes
the registry anyway passes every reply-only check — that is the "ack with no
consumer" defect in refusal clothing — and at the surface a wrong density is worth
4 mm of depth, so no depth comparison can catch it either. So `hw_accept.py`
writes a **mid-range** value while the dive is open, reads the ruler back *during*
the dive, restores it at the surface and reads it again, with each command paired
to its own reply (`run_protocol(..., pairs=)`). Four graded checks: the refusal's
words; **the ruler did not move DURING the dive**; the box still says 1.025 after
the dive; the key is writable at the surface (a key that can never be written is
broken, not locked). `grade_density_lock` plus 10 lying-capture selftest cases.

Proven on hardware, not only on the host: flashing `FL_NONE` back reddened **three**
C2 checks — and the interesting one was
`the ruler did not move DURING the dive: the mid-dive read says 1.025, not 1.000`.
The post-dive read alone stayed green in that run, because the matrix's own surface
restore overwrites the leaked value: **the mid-dive read is the only witness** to a
dive that ran on a scale nobody agreed to. Host side, dropping the flag reddens the
router refusal and the `test_depthscale` flag pin; and grader mutations (mid-dive
read ignored, each of the four checks made unconditional, the two-read requirement
relaxed, one check deleted from the grader, the leg-ran guard removed) each reddened
exactly their own selftest case.

**A refusal probed with a schema endpoint hid that defect (found on hardware
2026-11-24, and the lesson is the useful part).** The first version of that leg
asked for `0.990`, the *declared minimum* of the range. A build without the flag
answered `ERR out of range` — the right shape of `ERR`, for a completely unrelated
reason — and the matrix entry called it a dive lock. The reason was a real defect
in its own right: `setValue` parsed with `std::strtod` (double) and compared
against a `float` bound, and that comparison promotes the bound, moving it by up to
one ULP. The schema's decimal bounds are not exact binary values, so
`SET WATER_DENSITY=0.990` was refused against a published minimum of `0.990`,
because `0.990f` is 0.99000000953674316. Measured across the whole table: **ten
keys had an unreachable MIN** (SP1…SP4 at 0.20, `SP_HYST` 0.1, `ALLO_LOW` 0.05,
`CAL_O2_FI` 0.85, `CAL_O2_TARGET` 0.80, `CELLPROOF_MAXSP` 0.60, `WATER_DENSITY`
0.990) **and three an unreachable MAX** (`ALLO_CUTOFF` 1.80, `CELLPROOF_SP` 1.40,
`CELLPROOF_MIN` 1.30) — **13 of the 30 numeric keys** advertising a range in `GET
SCHEMA`, and in the page's own `<input min= max=>`, that their parser refused on the
exact number it showed. Fixed by parsing and comparing in `float` (the type the schema and the
storage both use), plus the same class of bug in the `needs integer` check. The
regression test is a walk, not a spot check:
`test_every_key_reaches_its_own_declared_bounds` asserts, for every one of the 30
numeric keys, that both declared endpoints parse AND that ~1% beyond each end is
refused — with a floor on how many keys it walked, so a loop that matched nothing
cannot pass. Restoring `strtod` reddens it immediately (`SP1 refuses its own
minimum 0.2: out of range`).

Two rules came out of that, and they are the part worth keeping:
* **Never probe a refusal with a schema endpoint.** A boundary value can be
  refused by a second rule, and the check passes for the wrong reason. The
  dive-lock probe now uses `1.000`, comfortably mid-range.
* **A refusal is not proof that nothing changed.** Any client that needs to know
  whether a locked write took effect must read the value back, in the same
  session, while the state that caused the refusal still holds. That is now a
  client rule in spec §6, not just a bench trick.

One thing this batch got **wrong at first and had to take back**, which is worth
more space than the fix: while extending
`test_dive_locked_keys_refused_mid_dive` its trailing assertion
(`b.run("DIVE START", r); has(r, "ERR")  // no double-start`) started failing, and
the explanation written into the code was "stale bytes in a reused `Response` —
`exec()` resets `len` but not what follows". **That is false.** `exec()` also writes
`text[0] = '\0'` and every reply path terminates at `len`, so a reused buffer cannot
hand a needle test the previous reply — an invariant already pinned since `15e08a1`
by `test_reused_reply_buffer_never_leaks_the_previous_reply`, which exists because a
real regression once did put an old reply's tail on the wire. The real cause was
quieter and worse to notice: **the case's own new `DIVE END` invalidated its own
trailing assertion.** Before the edit the dive was still open at that line, so the
assertion was testing something real. After it, `DIVE START` correctly answers
`OK DIVE`, and the same line was asserting a refusal that the firmware was right not
to give. Nothing was broken in the board — but the failure is still the useful part:
the behaviour ("a re-dive is a fresh dive") had no case of its own, it was riding at
the end of someone else's case, and it vanished the moment that case's state changed.
"it passed before I edited it" is not evidence a check still checks anything. The
behaviour went into its own case, `test_re_dive_after_a_closed_dive_is_a_fresh_dive`, which also pins something no
test had ever asserted: the refusal says **why** (`dive active` — a diver told only
`ERR` cannot tell an open dive from a broken command), and a `DIVE START` after a
closed dive really does open a second dive, which is what a dive log's two headers
depend on.

Gate after all of it, on the flashed build: native **258**, `hw_accept.py
--dive-persist` **136 checks 0 failures**, `web_probe.py` **62**, `--selftest`
**54** lying cases (9 depth, 10 scale, 10 ruler-lock, 13 dive-file, 12 surface),
esp32 SUCCESS, float audit clean.

**What the surface check cannot see, and the leg that can.** At the surface the
depth is centimetres, so the whole legal density range moves it by 0.004 m:
a density that stops reaching the conversion is invisible in `T,` at two decimals.
That is pinned on the host (fresh-water anchor + the scan rule). The wire
demonstration has to respect the dive-lock, so it is a **two-configuration** squeeze
leg, with the ruler set *before* the dive and the port held deep enough for the
2.5% to clear two decimals (at 5 m the difference is ~0.13 m):

```
SET WATER_DENSITY=1.000        # fresh, at the SURFACE: dive-locked, so set it here
DIVE START                     # then hold the port at ~5 m
expect  T, depth ~2.5% larger than the console's hPa implies at 1.025,
        and hw_accept's depth check still green — it grades on 1.000, because
        it reads GET WATER_DENSITY off the link instead of assuming the default
DIVE END
SET WATER_DENSITY=1.025
```

Optional, not in the C2 gate (it needs a steady squeeze held long enough to read a
`D,` ceiling). It is also why `grade_depth_scale` takes the density off the link
rather than using 1.025: the check must stay honest on a bench configured for a
different ruler, instead of failing at 2.5% for the wrong reason. Recorded here so
the next bench day does not have to re-derive why the surface comparison is not
enough.

**The announce must describe a reading, not a lock (fixed 2026-11-24).** Every
healthy boot used to print `depth: LPS28DFW @0x5D ready, 0.0 hPa`. That was not
a bad sensor: `begin()` reports READY the instant the chip is *identified*
(WHO_AM_I `0xB4`), while the first conversion comes later, paced by the driver's
own sample window — so the one line the bench reads as evidence always carried a
placeholder. A number that is always zero is worse than no number: it reads like
a measurement. The announce now fires on the first sample the **control path**
would use — the same `p > 0.2f` gate in `main.cpp`, passed across as `saneSample`
so the two rules cannot drift — and a source that identifies but never produces
one says so out loud instead of leaving silence:

```
depth: LPS28DFW @0x5D ready, 1016.7 hPa
depth: ** LPS28DFW @0x5D identified, no sane pressure within 3 s (reads 0.0 hPa) - depth will not be used
```

The second form is a different fact from "no sensor fitted", which stays silent;
3 s is the grace, measured from the lock rather than from boot. `hw_accept.py
--expect-depth` grades all of it — identity+address pair the gates would actually
accept, an hPa inside 250..1200 (loose on purpose: weather and altitude — the
check is about *placeholder vs reading*, not calibration), and the line agreeing
with HB's `al` word in both directions: a `ready` line with `BIT_SENSOR_STALE`
set, or a `no sane pressure` line with the chain believing the sensor, is a
console disagreeing with the machine. Host side: `test_pressure`'s five announce
cases pin the ordering, the exact bytes, and the refusal to emit a truncated
line. Proven by replay — the pre-fix capture fails "the depth announce carries a
reading, not a placeholder", the post-fix capture is clean; five grader mutations
each redden `--selftest`. On the bench itself the whole gate is **118 checks, 0
failures** (`--expect-web --expect-depth --handset /dev/ttyUSB1 --cal-air
--predive --dive-persist`), and the *once per boot* property shows up for free:
`--dive-persist`'s deliberate reset produces a second, equally honest
`depth: LPS28DFW @0x5D ready, 1016.7 hPa` about 2.5 s into the new boot — the
announce is late now, by design, because it waits for a sample. Sensor-task
headroom moved 2702 → 2700 bytes with the announce's 160 B line buffer in place.


**Cell-proof clamp events (`E,` ids 10/11/12) — seen on the wire 2026-11-24,
and the dive-file gap they exposed (fixed the same day).** The proof arms on the
descent crossing `CELLPROOF_DEPTH`, so with no depth sensor it never runs and
these three rows can never appear: `test_uartlink`'s `test_cellproof_clamp_edges_publish_once`
drives the real engine instead. With the LPS28DFW fitted this is reachable on
the bench — force the failure cheaply and watch the wire (handset side, and the
dive file):

```
SET CELLPROOF_DEPTH=3        # arm depth, minimum the schema allows
SET CELLPROOF_TMO=20         # fail fast: 20 s instead of 60
SET SP2=1.00                 # the tier the fake depth selects, pushed ABOVE the
                             # clamp ceiling (CELLPROOF_MAXSP=0.90): a cap the
                             # loop would never reach is a cap nobody can see
TIME SET <epoch> ; DIVE START
# press the port / breathe through a tube until depth passes 3 m, then hold it:
#   the bench cells sit near 0.21 bar, so the vote never reaches CELLPROOF_MIN
expect  E,<t>,10,0           # verdict: the proof failed
        E,<t>,11,0           # consequence: setpoints capped — once, not per cycle
        T, ... CLAMP         # FLAGS bit 5, the same fact held live
CELLPROOF OVERRIDE           # the diver lifts it
expect  E,<t>,12,1           # value 1 = the diver lifted it (never the machine)
DIVE END ; LOG EXPORT <slot>
```

**Done on the bench 2026-11-24** (dive slot 8 / seq 58, 66 s, thumb to 7.42 m,
the whole leg over UART2 through the handset bridge):

```
E,0,1,4   E,0,6,0  E,0,6,1  E,0,6,2      dive opens: battery alarm, three rejoins
E,34,4,110                               proof demands SP 1.10 at the crossing
E,54,1,5                                 alarm raised: bit 5 = BIT_SP_CLAMP
E,54,10,0                                verdict: the proof failed
E,54,11,0                                consequence: setpoints capped, once
CELLPROOF OVERRIDE -> OK CLAMP LIFTED
E,66,12,1                                clamp OFF, value 1 = the diver lifted it
GET PROOF -> PROOF,4,000,000,1
```

HB `sp=0.90 al=0x0030` inside the clamp, `sp=1.00 al=0x0010` after the override;
one row per edge, all nine rows in one dive. The same arc is in the `S,` rows'
`sp`/alarm columns: `0.400 → 1.100 → 0.900 → 1.000` with `0x0010 → 0x0030 →
0x0010`, so the cap (0.90, then back above it by the diver's own order) is
readable from the file, not only from the live line. `SP2` was restored to 0.550
at the end of the leg (`OK SAFETY`).

**What it exposed: the dive file kept the clamp's effects and none of its
verdicts.** That dive exported **0 `E,` rows** — checked from both transports,
`LOG EXPORT 8` and `GET /dive/8.csv`, both `grep -c "^E,"` → 0. So a dive
downloaded afterwards answered "the setpoint got capped" and never "the cell
proof failed, the clamp engaged, the diver overrode it": the alarm word moves
between two samples and the reader has to infer the rest. Cause is precise:
`DiveStore::append(const logfmt::Event&)` existed and worked, but nothing called
it for protocol events — `UartLink::drainEvents` appended an event-driven
**snapshot** per event and threw the event row away. The only event row any dive
ever held was `EVT_CLOCK_UNSET` at open, and this dive had a clock set. The same
dive's `M,` line read `M,66,743,0.212,0.215,0,0,0,0,0,0,0.000,0.000,0.000,0.000`
beside 13 `S,` rows: `DiveTotals::toMeta()` filled 6 of the 14 fields, so
`snapshots 0`, `events 0`, `gf 0/0` (the plan is 30/85) and a `0.000` setpoint
ladder (the dive ran 0.40/1.00/1.00/1.30). The format has carried those columns
since v2 with nothing behind them; a summary line that contradicts its own dive
cannot be reviewed later.

**Fixed 2026-11-24, as one publication rather than two writers.** `drainEvents`
now emits the **event row first, then the snapshot that shows the state it was
decided in** (`Hardware::appendEventNow` → `session_wiring` → `DiveStore::append`),
so the wire and the file cannot drift: same pop, same `t_s`, same numbers. The
`M,` count columns come from `DiveStore` — the object that wrote the rows —
patched in at close, never from a chain-side counter. `gf_low`/`gf_high`/`sp1..4`
ride in `bridge::DiveTotals`, filled by ControlTask from the live config, so a
dive is reviewed against the plan it used rather than the one on the board now.
The host test that pinned this caught a third bug on the way: `toMeta()` guarded
the min-ppO₂ sentinel but not the max (`maxVotedX1000 < -32768` can never be
true, and the field *starts* at −32768), so a dive where no vote ever arrived
wrote `max_ppo2 = -32.768` bar into its own summary — an impossible ppO₂ printed
with the same three decimals as a real one. Our dive had votes, so nobody saw it.
Bench side the property is now a C2 leg, not a one-off script: `--dive-persist`
watches a dive's `E,` rows live and requires them, in order, in the file, and
requires the summary to agree with the rows beside it — **127 checks, 0
failures**. `--selftest` grows to 34 lying cases including the pre-fix dive above;
five grader mutations each reddened exactly its own case (join removed, order
ignored, counts trusted, an unverifiable setting counted as agreement, the
sentinel tolerated), and removing the "the capture was not empty" guard reddens
the case that exists to catch a vacuous join.

**One honesty note that stands, and is not a defect:** after `CELLPROOF
OVERRIDE`, `GET PROOF` answers state `4` = PASS, because `Engine::override()`
moves `FAIL → PASS` — the diver outranks the machine. A handset reading that line
cannot tell "cells proven" from "diver overrode a failure"; the distinction lives
in the `E,<t>,12,1` row and the id-10 row that preceded it, which the dive file
now keeps.

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
