# ESP-CCR Handset Protocol — v1 (revised 2026-11-24)

Revision of 2026-11-24: uplink overflow is now dropped whole on both transports
(§1). No line, id or field was added, removed or renumbered, so the version stays
v1 — a v1 client needs no change; what changed is that the firmware now does what
this document always said a long line should do.

The complete contract between the ESP-CCR nitrox CCR controller and any client
that commands it: the wired **handset** (UART2) and the built-in **web page**
(WebSocket) speak the *identical* line protocol through *one* command router on
the controller — there is no second path, and no client gets a private bypass
of the safety lockouts. What follows was derived from the shipped code and is
kept exact on purpose: this file is the compatibility contract for building a
handset (a library is provided in `handset/`, see `handset/README.md`).

Sources of truth (read these if a detail below seems wrong — the code wins):
`src/comms/uart_proto.{h,cpp}` (grammar), `src/comms/cmd_router.{h,cpp}`
(commands, refusals, replies), `src/comms/uart_link.cpp` (downlink cadence,
log streaming), `src/comms/session.{h,cpp}` (one shared session), and the
host suites `test/test_proto/`, `test/test_uartlink/`, `test/test_session/`.

---

## 1. Physical / transport

| Property | Value |
|---|---|
| Port | UART2 on the controller: **GPIO17 = controller TX**, **GPIO16 = controller RX** |
| Settings | **115200 baud, 8N1**, idle-low, 3.3 V logic |
| Wiring | handset RXD ← GPIO17, handset TXD → GPIO16, **common ground mandatory** |
| Framing | plain text, one command or one record per line; `\n` or `\r` (or both) terminate |
| Uplink limit | **max 64 characters** per command line. The 65th answers `ERR line too long` (at most one per 2 s) and the **whole line is then discarded**, terminator included: neither its tail nor its truncated prefix is ever executed. Keep commands ≤64 chars — the library refuses longer ones before they reach the wire |
| Downlink size | every downlink line fits in **1024 bytes**; clients should buffer ≥1024 |
| Auth / handshake | none on UART. The link is passive: the controller starts pushing telemetry immediately and forever |

### Why this physical layer, and what would replace it

The layer above is transport-agnostic on purpose: everything is lines, and the
firmment's session object hands each line to a `sink(text, ctx)` callback, so the
handset port and the browser WebSocket run the identical command path and the
identical refusals. Changing the wire is therefore a new link, not a new
protocol — which is why the choice below is about connectors and conductors, not
about the message formats in this document.

| Option | Verdict |
|---|---|
| **UART2, 3 conductors** (chosen) | Full duplex, which the protocol assumes: the controller pushes `T,` at 2 Hz while a command may arrive at any instant. Three wires, no parts, 115200 baud is ~100× the traffic. Hazard: GPIO16/17 are also the WROVER PSRAM pins — the boot heap line is the cross-check (~200 KB means no PSRAM mapped, so the pair is free; ~5 MB would mean this port cannot live there). |
| RS-485, 2-wire half duplex | Differential and rugged, but **half duplex breaks this protocol**: unprompted downlink plus uplink-at-any-time needs turnaround and collision rules that do not exist here. A 4-wire full-duplex transceiver pair keeps the bytes and the protocol untouched, and is the cheap answer if the problem is cable length or noise. |
| CAN (TWAI) | The best of the wired alternatives, because CSMA/BA arbitration keeps both ends able to speak unprompted. Classic ESP32 has a TWAI controller and the IDF driver ships in this toolchain. Cost: 8-byte frames, so a 62-byte `T,` line becomes 8 frames and needs a framing annex — chunk header, sequence, end marker, and a rule that turns bus-off into a protocol-visible condition. Worth revisiting for **multiple nodes** (the point-to-point limit is permanent), a console **metres** away, or **galvanic isolation** through the housing. |
| Single-wire / current loop / optical | Fewer conductors and (for the loop) isolation, at the cost of half duplex again. |
| BLE | No penetration at all, but 2.4 GHz dies in about ten centimetres of water: this is a **surface** transport, the same class as the web AP, not a handset-on-the-counterlung transport. |

Two of the reasons people reach for CAN or 485 are already covered here:
link-fault detection (the 2 Hz `T,` stream is a heartbeat; a handset that stops
seeing `T,` knows within a second) and noise margin on a short hose.

### Un-terminated bytes (verified on the bench, 2026-11-23)

Bytes that arrive without a line terminator **stay in the line buffer** and are
parsed as part of the NEXT line. Sending `XY`, then `AUTO\r\n`, is answered
`ERR unknown command` because the controller saw `XYAUTO`; the following
`AUTO\r\n` answers correctly. So one bad line costs one error reply and the
parser recovers by itself - a handset does not have to resynchronise.

This matters on the bench more than in the water: opening the handset adapter's
serial port resets the bridge ESP32, and the boot banner it prints goes out
straight into the controller's RX. Unless the harness sends a newline first and
discards the reply, the first command of a run is glued to that banner and fails
for a reason that has nothing to do with the firmware. `tools/hw_accept.py` and
`tools/web_probe.py` both flush this way; a bare newline answered
`ERR unknown command` is the proof that bytes were pending.

Note also that the link is **passive in both directions**: downlink lines
(`T,` at 2 Hz, `D,` at 0.5 Hz, `E,` as raised) start arriving the moment the
port is open, before any command has been sent. A client must not treat those as
replies.

### Over-long lines (rule fixed 2026-11-24; the bench observations below are what
### the firmware did before the fix)

A line longer than `proto::MAX_LINE` (64 characters) is **dropped whole**:

* `ERR line too long` is sent **once**, when the limit is passed.
* Everything after that point on the same physical line is discarded, including
  the terminator — the line never reaches the command router, in any form. It is
  not split in two, and the truncated first 64 bytes are never executed.
* The error text is rate limited to **one per 2 s**, so a wall of junk cannot
  storm the 6 KB-stack UART task with error replies. The rate limit gates the
  *message* only: a suppressed overflow is still discarded, and the next line is
  processed normally.
* A line of exactly 64 characters is valid — 64 is a capacity, not a refusal.

Both transports take that verdict, so an over-long send gets the same answer over
UART2 and over `/ws` (verified: `web_probe.py` compares them byte for byte).

**What this replaced, and why the rule is now strict.** Until 2026-11-24 the UART
task handled overflow as it happened rather than at the terminator: it reset the
buffer and kept reading, so the rest of one physical line became a *second*
command; and in the rate-limited case it did not reset the buffer at all, so the
terminator dispatched the truncated first 64 bytes. Observed on the bench
2026-11-23 (pre-fix firmware):

| Send (each 204 bytes) | Answer then | Effect |
|---|---|---|
| `GET SP2 ZZZ…` (fresh 2 s window) | `ERR line too long`, then `ERR unknown command` | refused, and its tail ran as a phantom command |
| the same line again inside that window | `SP2=0.700` | **executed truncated** |
| `SET SP2=0.33 YYY…` after arming the window | `OK SAFETY`, and `GET SP2` read back `0.330` | **the write landed** |
| the same `SET` after the window expired | `ERR line too long`, value unchanged | refused |

A handset library never sends more than 64 bytes, so reaching this needs a
corrupted or mangled stream — which is exactly why the rule is strict now. A
client that receives an over-long line from its own side should assume nothing
happened, and should not retry blindly if the command had an effect: it cannot
know whether the controller saw the bytes at all. `hw_accept.py`'s framing phase
proves the three properties above on real wires.

The web transport differs in framing only: a WebSocket frame arrives whole, so
the server answers `ERR line too long` once, marks that answer `#1 dropped`, and
discards the frame (`web_server.cpp:136-146`). Same session, same router, same
refusal text — and since 2026-11-24 the **same verdict**, which is why
`tools/web_probe.py` compares all nine state-neutral commands, the over-long case
included, **byte for byte**.

The web transport (`ws://<controller>/ws`) feeds the same lines; it adds
WebSocket-only conventions:

| Line | When | Meaning |
|---|---|---|
| `#<lines>` | after every command answer | answer is complete; `lines` = **exactly** the number of answer lines just sent |
| `#<lines> dropped` | same, when the **command** was over the frame limit | the reply is `ERR line too long`; the flag tells the page its own command arrived truncated, not that the reply was cut |
| `#boot <uptimeMs> <freeHeap>` | once, when a socket opens | hello: uptime lets a page that connects after a reboot say so; the free heap rides along. **Not** an answer marker |

`#` can never start a protocol line, so nothing collides. A UART handset never
sees `#`.

A client must match the marker shape exactly (`#\d+` with an optional
` dropped`). Treating *any* `#` line as the end of an answer means stopping at
the `#boot` hello, and then every reply is attributed to the command after the
one that produced it — which is how `tools/web_probe.py` reported nine
cross-transport disagreements on firmware that was answering correctly.

## 2. Downlink cadence (UART; what a handset receives unprompted)

| Line | Every | Meaning |
|---|---|---|
| `T,` | 500 ms (2 Hz) | live telemetry |
| `D,` | 2000 ms (0.5 Hz) | decompression picture |
| `E,` | on change | event (alarm edges, mode change, cell voting changes, …) |

**Link health rule for handsets:** `T,` arriving at 2 Hz means the controller
is alive and the UART is intact. A handset must alarm when no line at all has
arrived for >3 s (the provided library does exactly this). A silent handset
link is a safety problem, not a UI problem.

## 3. Command lines (uplink) and their exact replies

Replies are always one line: `OK`, `OK <NOTE>`, or `ERR <reason>`. Some
commands answer with a data line instead of, or before, `OK` (marked below).
Refusals are honest `ERR` lines — a handset should display them, not hide them.

### Mode & diver controls

| Command | Reply | Notes |
|---|---|---|
| `AUTO` | `OK AUTO` / `ERR predive required` | AUTO is **refused until the pre-dive sequence passed**. No client can bypass it. |
| `MANUAL` | `OK MANUAL` | always accepted |
| `ACK` | `OK` / `OK SAFE RESTART` | acknowledge/silence current alarms. The diver outranks the machine: the condition persists and keeps alarming if it persists, but the latch/silence is honoured. In **SAFE mode** it means something else and bigger — see "SAFE mode" below. |
| `CHECK` | `OK CHECK` / `ERR dive active` | run the pre-dive sequence |
| `PULSE` | `OK` | request one test injection pulse (posted to the control task; it is a real pulse, subject to the injector's own duty/stuck-open rules) |
| `TEST CELLS` | `OK TEST CELLS` / `ERR test running` | start the cell pulse test; watch it via `GET CELLS` |
| `DECO` | `D,` line (no OK) | ask for the deco picture now |
| `CELLPROOF OVERRIDE` | `OK CLAMP LIFTED` / `ERR no clamp active` | lift the setpoint clamp imposed by a failed cell proof |

### Calibration (all refused mid-dive with `ERR cal locked in dive`)

| Command | Reply |
|---|---|
| `CAL AIR` | `OK CAL AIR` — starts a ~12 s sampling window (1 Hz × 12, spread-limited); the outcome is printed on the controller's **USB console** only |
| `CAL O2` | `OK CAL O2 ARMED` (first) — bare `CAL O2` again = `OK CAL O2 CANCEL` |
| `CAL O2 CONFIRM` | `OK CAL O2` / `ERR not armed` |
| `CAL DEPTH` | `OK CAL DEPTH` — one-shot depth offset capture |

Two honest caveats a handset must not paper over (details in §8):

* `CAL AIR` answers `OK CAL AIR` **even when it is then refused**: the reply
  comes from the command router, the warm-up gate (< 120 s after power-on) and
  the sampling result live in the control task and are reported on the console.
  So `OK` means "requested", not "calibrated" — see §8.
* `CAL O2` currently has **no consumer in the firmware**: the two-step state
  machine (`cal::O2Sequencer`) is implemented and host-tested, but nothing
  instantiates it on the board yet, so `ARMED`/`OK CAL O2` are protocol-true
  and physically inert. A handset may send it; it must not present it as a
  working calibration.
* `CAL DEPTH` likewise has no consumer today: the command raises a request
  flag that nothing reads, and no depth-offset trim is implemented in the
  depth chain (depth is the smoothed sensor reading, taken as-is). Same rule:
  the reply is honest about the *command*, not about an effect.

### Dive lifecycle

| Command | Reply | Notes |
|---|---|---|
| `DIVE START` | `OK DIVE` / `ERR dive active` (+ `ERR LOG FAILED: dive unrecorded` if the dive file could not open) | starts logging; **also turns the WiFi radio off** and remembers to restore it at dive end — unless the diver had already turned WiFi off themselves |
| `DIVE END` | `OK DIVE` / `ERR no dive` | closes the dive file; restores WiFi only if `DIVE START` dropped it |
| `WIFI ON` | `OK` / `ERR wifi locked in dive` | the radio never comes back on during a dive |
| `WIFI OFF` | `OK` | diver's choice is remembered; dive end will not re-enable |

### Status queries

| Command | Answer line(s) |
|---|---|
| `GET T` | one `T,` line (same format as periodic) |
| `GET D` | one `D,` line |
| `GET PRE` | `PRE,` line |
| `GET CELLS` | `CELLS,` line |
| `GET PROOF` | `PROOF,` line |
| `GET FAULT` | `FAULT,` line |
| `GET HIST` | 3 × `HIST,` lines / `ERR no histogram` |
| `GET TIME` | `TIME,<epoch>,<source>,<age_s>` — wall clock (see Wall clock below) |
| `GET SCHEMA` | one `K,` line per settings key |
| `GET <KEY>` | `<KEY>=<value>` for a settings key |
| `GET *` | one `<KEY>=<value>` line per settings key (32 keys today) |

### Settings

`SET KEY=VALUE` — VALUE is **the rest of the line** (may contain spaces;
important for `AP_SSID`/`AP_PASS`). Reply `OK`, or `OK SAFETY` when the key is
safety-relevant (the client should event-log these), or:
`ERR dive locked` (key is frozen while a dive runs), `ERR bad value`,
`ERR unknown key`, and registry validation reasons (`wants ON/OFF`,
`value too long`, `value too short`).

The authoritative key list with types, ranges and flags is **self-describing at
runtime via `GET SCHEMA`** — build the handset settings UI from that, not from
a hard-coded table. Section 6 lists today's keys for reference.

### Wall clock (`TIME SET`)

The controller has **no RTC chip**, and its access point has no uplink, so
there is nothing to take time from: a client at the surface IS the time
source. This is a command, deliberately **not** a settings key — settings
live in NVS, and resurrecting a saved epoch after a battery pull would date
every dive in the log with a number from a previous life, in silence.

| Command | Reply |
|---|---|
| `TIME SET <epoch>` | `OK TIME` / `ERR epoch out of range` / `ERR safe mode` |
| `GET TIME` | `TIME,<epoch>,<source>,<age_s>` |

* `<epoch>` is unix seconds, decimal digits. The accepted window is
  **2000-01-01 .. 2100-01-01**; anything outside is refused and changes
  nothing (a plausible-looking wrong epoch is worse than no epoch — every
  reader of the log would believe it).
* `<source>`: `0` never set since this power-up, `1` set by a command,
  `2` carried across a reset the **firmware performed on itself** (panic,
  watchdog, SAFE-mode `ACK` — RTC no-init memory survives those). It does
  **not** survive the EN pin or a battery pull: on this board EN is a
  CHIP_PU power-down, measured on the bench 2026-11-24 — the next boot
  reports `rst:0x1 (POWERON_RESET)` and the clock is honestly `unset`, never
  a date from a previous life.
* `<age_s>`: seconds since the clock was set.
* Allowed during a dive: the dive header was stamped at `DIVE START`, so a
  mid-dive `TIME SET` cannot rewrite the running dive, and a surface client
  that has only now noticed the unset clock should fix it before the **next**
  dive rather than be refused. Refused in SAFE mode: the chain is not running,
  so no client may author anything.
* A battery pull **or an EN cycle** clears the clock. **Set it at the surface
  after every power-up.** A dive started with the clock unset is still
  recorded (epoch 0, an `E,,15` row inside the dive file, and `clock-unset`
  in the CSV date column) — refusing to log a dive over a calendar is the
  worse failure.

The built-in web page posts its own clock (`TIME SET`) every time its
WebSocket opens, so a phone at the surface is normally the one setting it.

### Dive log

| Command | Answer |
|---|---|
| `LOG LIST` | `OK` then zero or more `L,<slot>,<epoch>,<seq>` lines, **newest-written first** (by `<seq>`; unsequenced legacy dives list last, slot-descending) |
| `LOG EXPORT n` (n = 1..10) | `OK` then the dive as lines: `DIVE,<slot>,<startEpoch>,<codec_version>,<row_format>,<iso_utc>` followed by `S,` snapshot rows, `E,` event rows and one `M,` totals row (format §5) |

The fourth `L,` field, `<seq>`, is the dive sequence: the order dives were
written, 1 upward, `0` = unknown (files written before 2026-11-24). Without
it a full FIFO on a board with no clock is ten dives and no order — slot
numbers stop meaning anything once eviction rotates, and the epoch is 0 when
nobody set the clock. It is also the key eviction orders by (PLAN 16: with
every epoch 0, eviction hammered slot 1 and destroyed the newest dives).

### SAFE mode (what a handset can and cannot do)

The controller boots into SAFE when it cannot account for its own last reset
(supervisor-detected task stall, watchdog, brownout, panic). The control chain
is **not running** in that state, and the protocol divides cleanly:

| | behaviour in SAFE mode |
|---|---|
| downlink | **unchanged** — `T,`/`D,`/`E,` keep streaming and `T,` carries mode `SAFE`. The sensor task is alive, so a handset can still watch the machine decide. Note that the mode field is asserted by the link, not by the control chain: the chain is stopped in SAFE and would report whatever it last had, so the router's lockout is the source of truth. |
| every command except `ACK` | `ERR safe mode` |
| `ACK` | `OK SAFE RESTART`, then the controller restarts itself cleanly |

* `ERR safe mode` is a refusal like any other: match the `ERR` prefix, and treat
  the reason text as informational — wording may change, the refusal may not.
* `ACK` here is a **restart**, not an alarm silence, and a handset should label
  it that way. Why a restart rather than resuming control where it stands: SAFE
  is entered because a task died, and continuing on a task set that just lost
  its stack is a state nobody can reason about — a second failure mid-dive would
  find the diver already breathing from it. A clean boot is the state the
  firmware was designed to come up in, and it is the same recovery a power-cycle
  gives, reachable with a button instead of wet hands on a connector.
* That restart is a software reset: settings come back from NVS, the dive log is
  intact, and the boot is normal — the fault record is one-shot, so it does not
  re-latch into SAFE.
* A malformed line is still reported as a malformed line (`ERR unknown command`)
  even in SAFE mode: the reason says what is wrong, not merely that the machine
  is locked.
* The web AP stays offline in SAFE mode. The handset link is the only client
  that comes up, and it comes up for `ACK`.

A handset should detect SAFE from the `T,` line's mode field rather than from an
`ERR` — the downlink never stopped, so the state is visible before the diver
tries anything.

## 4. Downlink line formats (exact)

### `T,` telemetry (2 Hz)

```
T,<depth_m>,<voted_ppo2>,<cell0>,<cell1>,<cell2>,<setpoint>,<MODE>,FLAGS0x<hex>,<batt_V>
e.g. T,18.35,0.712,0.708,0.715,0.711,0.700,AUTO,FLAGS0x10,12.41
```
Fields: depth 2 decimals, ppO2/setpoint/cells 3 decimals, battery volts 2
decimals. `<MODE>` ∈ `STANDBY | AUTO | MANUAL | SAFE`. **Today the firmware
emits only `STANDBY` (no dive running), `AUTO` and `MANUAL`** — `SAFE` is in
the grammar (a client must parse it, and a future build will use it) but the
supervisor's SAFE policy currently surfaces as a reset into standby with the
alarm flags set, not as `SAFE` on this line. **FLAGS byte bits:**

| bit | meaning |
|---|---|
| 0 | ppO2 low |
| 1 | ppO2 high |
| 2 | cell drift (voting excluded a cell / disagreement) |
| 3 | sensor-class failure — aggregates sensor fail, sensor stale, stuck-open valve, hypoxia |
| 4 | battery low |
| 5 | setpoint clamp active (cellproof ban) |
| 6 | cell proof currently FAIL |

The full controller alarm word has 10 bits; the handset-visible FLAGS
aggregates bits 6..9 (stuck open, sensor stale, CNS, hypoxia) into bit 3 —
they are all "major fault, act now" from a handset's point of view. A handset
must treat bit 0 and 1 as critical (ppO2 low = act immediately), 3 as major.

### `D,` decompression (0.5 Hz)

```
D,<ndl_min>,<deco_total_min>,<ceiling_m>[,<stop_depth>|<stop_min>]...,<cns_pct>
e.g. D,17,24.5,6.00,6|4.5,3|20.0,12.3
```
`<ndl_min>`: integer; **480 = effectively unlimited; -1 = unknown (no valid
vote yet — show "—", never 0)**. Stop pairs appear between ceiling and CNS,
zero or more (max 8), depth integer metres, minutes 1 decimal. CNS % is last
(1 decimal). A client that ignores stop pairs must still find CNS as the
**last** field — new pairs are only ever inserted before it (append-only rule, §7).

### `E,` event (on change)

```
E,<t_s>,<id>,<value>
```
`t_s`: seconds since dive start. Before the first dive has ever started it is
seconds since controller boot (the reference is **not** reset at dive end, so
post-dive events keep counting from that dive's start).

`id`/`value` — the ids are protocol-visible and **never renumbered**, but note
the split: only the marked ids are actually published by today's firmware.

| id | event | value | published today? |
|---|---|---|---|
| 1 | alarm raised | alarm bit index (0..9 of the full alarm word) | yes |
| 2 | alarm cleared | alarm bit index | yes |
| 3 | mode change | 1 = AUTO, 2 = MANUAL | yes |
| 4 | setpoint change | setpoint × 100 (0.70 bar → 70) | yes |
| 5 | cell excluded from vote | channel 0..2 | yes |
| 6 | cell rejoined the vote | channel 0..2 | yes |
| 7 | CAL AIR done | — | **no** (reserved) |
| 8 | CAL O2 done | — | **no** (reserved) |
| 9 | cellproof PASS | unproven-high mask (bit per cell) | yes |
| 10 | cellproof FAIL | 0 | yes |
| 11 | clamp ON | 0 | **no** (reserved) |
| 12 | clamp OFF | 0 | **no** (reserved) |
| 13 | pre-dive PASS | 0 | **no** (reserved) |
| 14 | controller reset | — | **no** (reserved) |
| 15 | dive started with no wall clock | 0 | dive **file** only (written into the dive at `DIVE START`; not streamed live) |

Do not build a "CAL AIR done" or "pre-dive passed" *notification* on those
reserved ids — they never arrive. Learn the same facts from what is live now:
`GET PRE` (poll, `PRE,` line), the `T,` FLAGS clamp bit (5) for a clamp, and
the controller's console output for calibration results.

### Status answer lines

```
PRE,<warmup>,<air>,<solenoid>,<depth>,<battery>,<passed>
```
step status ∈ 0 PENDING, 1 RUNNING, 2 PASS, 3 FAIL; `passed` 0/1. The order of
the five steps is fixed.

```
CELLS,<state>,<pass_mask>,<fail_mask>,<requested>
```
state ∈ 0 IDLE 1 BASELINE 2 PULSING 3 RECOVER 4 DONE; masks are **bitmask
printed in octal** (`%03o`, bit0 = cell0), so `007` = all three cells.

```
PROOF,<state>,<unproven_mask>,<exclusion_mask>,<attempts>
```
state ∈ 0 INACTIVE 1 IDLE 2 ARMED 3 RUNNING 4 PASS 5 FAIL; masks are octal
bitmasks as above (`004` = cell 2 only).

```
FAULT,<stuck_open_latched>,<output_fuse_tripped>,<solenoid_duty_pct>
```
duty 1 decimal, percent.

```
HIST,<cell>,<window_s>,<windows>,<bin_width_centi>,<horizon_age_s>,<samples>,<bin0>,...,<bin37>
```
one line per cell (3 lines). 38 bins of 0.05 bar covering 0.00–1.90 bar
(bin width printed in centibar: 5). Counts per bin. Display-only.

```
L,<slot>,<epoch>,<seq>    dive directory entry (slot 1..10, seq 0 = unknown/pre-2026-11-24)
TIME,<epoch>,<source>,<age_s>  wall clock answer (GET TIME); epoch 0 = no clock, source 0 unset / 1 set / 2 carried across a reset
K,<key>,<T>,<min>,<max>,<flags>,<default>   settings schema; T ∈ F I U B S (STRING defaults are text)
<KEY>=<value>             settings GET answer
```

## 5. Dive export rows (`LOG EXPORT n`)

One formatter produces these rows in the firmware (`logfmt::snapshotCsvLine` and
friends in `src/log/divelog.{h,cpp}`), and `GET /dive/<n>.csv` on the web sends
exactly the same bytes. They used to be two writers with two shapes;
`tools/web_probe.py` compares them on the bench now.

```
DIVE,<slot>,<start_epoch>,<codec_version>,<row_format>,<iso_utc>
S,<t_s>,<depth_m>,<voted>,<c0>,<c1>,<c2>,<sp>,0x<alarm16>,<cns>,<ceiling_m>,<ndl>,<deco_min>,<pulses>,<batt>,<mode>,<temp_c>,<flags>,<ceiling_cm>
E,<t_s>,<id>,<value>
M,<duration_s>,<max_depth_cm>,<min_ppo2>,<max_ppo2>,<pulses>,<cns_peak_pct>,<snapshots>,<events>,<gf_low>,<gf_high>,<sp1>,<sp2>,<sp3>,<sp4>
```

The `S,` and `M,` rows carry the `row_format` version in the `DIVE,` line, and
columns are **append-only**: field 1 is always the tag, fields 2-15 of an `S,`
row are always the v1 set in the v1 order and units, and new information only
ever arrives on the right. A parser written against v1 reads the first fifteen
fields of a v2 row and gets what it got before.

| version | `S,` fields | `M,` fields | added |
|---|---|---|---|
| v1 (2026-11-22) | 14 | 8 | the original contract |
| v2 (2026-11-23) | 19 | 15 | `mode`, `temp_c`, `flags`, `ceiling_cm`; `gf_low`, `gf_high`, `sp1`…`sp4` |
| v3 (2026-11-24) | 19 | 15 | `DIVE,` line gains a 6th field: start as ISO-8601 UTC (`clock-unset` when the dive started with no clock). `S,`/`M,` unchanged |

The `iso_utc` field is **rendered from the stored epoch, never stored**: one
truth on the flash, and a dive written before v3 still exports a correct date.
The renderer is the firmware's own (`logfmt::isoUtc`, closed-form civil date)
— not newlib's `gmtime`/`strftime`, whose time and float tables do not belong
on the task that answers clients.

Two corrections worth knowing, both of which were documentation errors rather
than wire errors:

* `max_depth_cm` — the field was always named `max_depth_m` in this document,
  but the firmware has always put **centimetres** there and the web page divides
  it by 100. The bytes never changed; the name was wrong.
* `ceiling_m` stays integer metres (a v1 parser reads it as an integer), which is
  why the exact value is the appended `ceiling_cm` instead of a reformat.
* `batt` is millivolts (`11840` = 11.84 V), not volts.

Snapshots land on a 30 s grid plus event-driven ones. Handsets can ignore the
export rows entirely (the web page renders them); the format is documented so
a handset *can* offer a dump-to-host feature.

**`E,` rows are ambiguous inside an export, on purpose and by shape.** An
exported event row is `E,<t_s>,<id>,<value>` — byte-identical to the live event
the controller pushes unprompted (§2). A client that needs the exported event
list reads the file (`/dive/<n>.csv`), not the UART stream. `tools/web_probe.py`
counts exported `E,` rows but does not compare them for this reason.

## 6. Settings keys (today; the runtime truth is `GET SCHEMA`)

Type chars: F float, I int16, U uint8, B bool (accepts ON/OFF/YES/NO/TRUE/
FALSE/1/0), S string. Flags bit 0 = dive-locked (refused mid-dive), bit 1 =
safety (replies `OK SAFETY`).

| key | type | default | range | flags |
|---|---|---|---|---|
| SP1 SP2 SP3 SP4 | F | 0.40 0.70 1.00 1.30 | 0.20–1.60 | safety, dive-lock |
| D1 D2 | F | 30 50 | 5–100 / 10–150 | — |
| SP_HYST | F | 0.5 | 0.1–3.0 | — |
| ALLO_LOW | F | 0.18 | 0.05–0.40 | safety, dive-lock |
| ALLO_HIGH | F | 1.40 | 1.00–1.60 | safety, dive-lock |
| ALLO_CUTOFF | F | 1.60 | 1.50–1.80 | safety, dive-lock |
| CNS_ALARM | U | 80 | 10–100 | safety, dive-lock |
| CELL_DRIFT_BAR | F | 0.05 | 0.01–0.30 | — |
| INJ_PULSE_MS | I | 300 | 50–1000 | safety, dive-lock |
| INJ_MAX_DUTY | U | 10 | 1–25 | safety, dive-lock |
| CAL_O2_FI | F | 0.99 | 0.85–1.00 | — |
| CAL_O2_TARGET | F | 0.95 | 0.80–1.00 | — |
| CAL_O2_MAX_PPO2 | F | 1.00 | 0.90–1.10 | safety |
| CAL_SLOPE_BAND | U | 15 | 5–50 | — |
| CELLPROOF_EN | B | ON | — | — |
| CELLPROOF_DEPTH | F | 10 | 3–30 | — |
| CELLPROOF_SP | F | 1.10 | 1.00–1.40 | safety, dive-lock |
| CELLPROOF_MIN | F | 1.00 | 0.90–1.30 | safety, dive-lock |
| CELLPROOF_TMO | U | 60 | 20–120 | — |
| CELLPROOF_MAXSP | F | 0.90 | 0.60–1.00 | safety, dive-lock |
| GF_LOW | U | 30 | 10–95 | safety |
| GF_HIGH | U | 85 | 50–100 | safety |
| ASC_RATE | F | 6.0 | 3–18 | safety |
| ASC_RATE_SLOW | F | 1.0 | 0.5–3.0 | safety |
| WATER_DENSITY | F | 1.025 | 0.990–1.100 | — |
| VBAT_MIN | F | 10.5 | 3.0–30.0 | safety |
| AP_SSID | S | `ESP-CCR` | len 1–16 | dive-lock |
| AP_PASS | S | `diveonly` | len 8–31 | dive-lock |

Note: GF_* and ascent rates stay writable **during** a dive (normal diving:
the schedule moves, the gas plan does not); setpoints, alarms and injection do
not.

## 7. Compatibility rules (how this protocol evolves)

1. **Append-only.** New `E,` ids, new settings keys, new stop pairs inside
   `D,`, new fields **after** the last documented field of a line: legal.
   Renumbering ids, reordering fields, or removing anything: not done. A
   parser that reads the documented prefix and ignores what it does not know
   never breaks.
2. **Unknown uplink** always answers `ERR unknown command` — never silent.
3. **Unknown downlink** lines must be ignored by clients (defensive: forward
   compatibility).
4. Refusal strings are stable words (`dive locked`, `predive required`, …) —
   match the leading words, not full-sentence equality, when reacting.
5. The controller console heartbeat (`HB st=...`) is a debug line on the USB
   serial console, **not** part of this protocol — do not parse it from UART2.

## 8. Known current limits (truthful as of this revision)

* `SET` values are truncated to 23 characters by the command parser, even
  though `AP_PASS` allows 31 — keep passphrases ≤23 for now.
* `LOG EXPORT n`: `n` is the slot number itself, 0..10. `LOG EXPORT 0` answers
  `ERR no such dive` (deliberate: that is what v1 did). The upper bound is the
  firmware's own `logfmt::MAX_DIVE_SLOTS`, not a number copied into the parser:
  until 2026-11-23 the parser had a hard-coded 0..9, so slot 10 existed, showed up
  in `LOG LIST` and downloaded over the web, but a handset could not export it.
* `E,` ring is 32 deep: a burst can drop events (the controller counts drops);
  a handset should not treat `E,` as a reliable archive — the dive file is.
* **Uplink overflow is dropped whole** (fixed 2026-11-24, and a defect until
  then). At 65 characters the UART task answers `ERR line too long` and discards
  everything up to the next terminator, so neither the tail nor the truncated
  first 64 bytes reaches the command router. Before the fix it *did* resynchronise
  mid-word — a 100-character line could execute characters 66..100 as a command,
  and an overflow inside the error rate-limit window executed the truncated
  prefix (an over-long `SET SP2=0.33` wrote 0.330). Client rule unchanged: never
  send a line the library has not length-checked. (The library itself drops an
  oversized *downlink* line whole, which is the behaviour a handset wants from a
  corrupt stream — the firmware now behaves the same way.)
* **`OK` means "the command was accepted", not "the effect happened".** Three
  calibration paths are parsed, replied and armed but have no firmware
  consumer yet: `CAL O2` (`cal::O2Sequencer` is host-tested, never
  instantiated on the board), `CAL DEPTH` (request flag is never read; no
  depth-offset trim exists) and the warm-up half of `CAL AIR` (`OK CAL AIR` is
  answered even when the control task then refuses it and says so on the USB
  console only). A handset must not draw a green "calibrated" tick from these
  replies. A firmware-side fix (an `ERR` or an event when the effect fails) is
  the right way to close this; until then, treat these replies as advisory.
* Event ids 7, 8, 11, 12, 13, 14 are reserved in the log format and **not
  published** (§4 table). Do not build notifications on them.
* `PULSE` works during a dive as well as at the surface — it is a diver
  command with a physical effect, so a handset should keep it off an
  unguarded button.

## 9. Building a handset

Start with `handset/handset_link.h` (transport-agnostic C++17, no OS
dependency) and `handset/example_esp32_handset/` (complete wiring + main
loop). Self-certification: the controller-side suites pin every format above,
and `tools/hw_accept.py` (host, Python) drives the same commands the handset
sends — run it with a handset wired to see the protocol from the other side.
