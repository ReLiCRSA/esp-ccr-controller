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

### Calibration

| Command | Reply |
|---|---|
| `CAL AIR` | `OK CAL AIR` / `ERR cal locked in dive` — **a receipt, not a verdict**: it starts (or queues) a ~12 s sampling window (1 Hz × 12, spread-limited). Read the outcome with `GET CALAIR` or from the `E,<t>,7,<code>` event |
| `GET CALAIR` | `CALAIR,<code>,<age_s>,<samples>,<spread>,<raw0>,<raw1>,<raw2>,<warmup_left_s>` |
| `CAL O2` | `ERR not implemented` |
| `CAL O2 CONFIRM` | `ERR not implemented` |
| `CAL DEPTH` | `ERR not implemented` |

**`OK CAL AIR` means the request was accepted.** The warm-up gate (cells must
polarise for 120 s after power-on) and the sampling result live in the control
task, so the verdict arrives on its own, two ways:

* **unprompted**: `E,<t>,7,<code>` the moment it ends (see the event table in §4);
* **on request**: `CALAIR,<code>,<age_s>,<samples>,<spread>,<raw0>,<raw1>,<raw2>,<warmup_left_s>`
  — the last verdict since this power-up and its age in seconds. The numbers are
  raw ADC counts, not ppO2: `<raw0..2>` are the per-cell air averages that were
  committed and `<spread>` the worst per-cell spread across the window; unfitted
  channels read 0. `<warmup_left_s>` is meaningful only with code 3.

| `<code>` | meaning | sent as an `E,` id 7 value |
|---|---|---|
| 0 | no `CAL AIR` since this power-up (all other fields 0) | never |
| 1 | sampling; `<samples>` is progress out of 12 | never |
| 2 | calibrated: air point committed for every fitted cell | yes |
| 3 | refused: cells still warming up, `<warmup_left_s>` to go | yes |
| 4 | aborted: a dive started (CAL AIR is a pre-dive act) | yes |
| 5 | failed: cells never settled inside the spread band | yes |
| 6 | failed: no fitted cells to sample | yes |

The codes are append-only: never renumbered, new outcomes are added on the right,
and an unrecognised code means "a verdict this client has no name for", not an
error. A client that renders code 0 as "no calibration since power-up" is telling
the truth; one that renders it as a green tick is not.

`CAL O2` and `CAL DEPTH` are **refused, not acked**, because nothing on the board
performs them: the O2 sequencer (`cal::O2Sequencer`) is implemented and
host-tested but has never been instantiated in the firmware, and there is no
depth-offset trim in the depth chain (depth is the smoothed sensor reading, taken
as-is). Until 2026-11-24 they answered `OK CAL O2 ARMED` and `OK CAL DEPTH`,
implying work that had not happened — on a rebreather that is a false all-clear,
and a client has no way to tell it from the real thing. The commands stay in the
grammar so no client has to change when the work lands; a handset must show the
refusal. There is no two-step arm to cancel any more: `CAL O2 CONFIRM` is refused
outright.

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
| `GET CALAIR` | `CALAIR,<code>,<age_s>,<samples>,<spread>,<raw0>,<raw1>,<raw2>,<warmup_left_s>` — the air calibration's verdict (§3) |
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
| 7 | air calibration verdict | the `CALAIR,` status code (2 OK, 3 warm-up, 4 diving, 5 unsettled, 6 no cells) | yes: one row per verdict, never one per sample |
| 8 | CAL O2 done | — | **no** (reserved — and `CAL O2` itself is refused as unimplemented, §3) |
| 9 | cellproof PASS | unproven-high mask (bit per cell) | yes |
| 10 | cellproof FAIL | 0 | yes |
| 11 | setpoint clamp ON (cellproof FAIL capped the setpoints) | 0 | yes (edge: once per clamp, not once per cycle) |
| 12 | setpoint clamp OFF | 1 = lifted by the diver's `CELLPROOF OVERRIDE` | yes |
| 13 | pre-dive PASS | 0 | **not on the wire**: recorded in the surface log instead (§5.1), where a pre-dive result is a fact about the machine rather than about a dive |
| 14 | controller reset | — | **no** (reserved) |
| 16 | pre-dive FAIL | the step that failed: 0 warm-up, 1 air, 2 solenoid, 3 depth, 4 battery — the same order as the five `PRE,` fields | **not on the wire**: surface log only (§5.1) |
| 15 | dive started with no wall clock | 0 | dive **file** only (written into the dive at `DIVE START`; not streamed live) |

Do not build a "pre-dive passed" *notification* on the remaining reserved ids —
it never arrives. Learn the same facts from what is live now: `GET PRE` (poll,
`PRE,` line), `GET PROOF` / the `T,` FLAGS clamp bit (5) for a clamp, and
`GET CALAIR` or `E,*,7,<code>` for the air calibration.

Ids 13 and 16 are the interesting middle case: the firmware **does** record a
pre-dive verdict durably, but in the surface log rather than on this stream,
because the dive a failure prevents never happens and so there is no dive file to
put the row in. A handset that wants that history today has no command for it
(§8); it can only poll `PRE,` while it is listening.

Ids 11/12 are published as **edges**: a FAIL lasts hundreds of control cycles,
and one row each is what the dive file needs (id 10 says the verdict, id 11 says
the consequence the diver can act on, id 12 says who lifted it — value 1, the
diver, because a FAIL never re-arms itself).

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
CALAIR,<code>,<age_s>,<samples>,<spread>,<raw0>,<raw1>,<raw2>,<warmup_left_s>  air calibration verdict (GET CALAIR); codes in §3, counts raw
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

**What the `M,` totals row says when there is nothing to total.** Pinned
2026-11-24, because every one of these was previously a lie on real flash:

* `snapshots` and `events` count the `S,` and `E,` rows **in the same export**,
  so a client can use them as a truncation check. A dive's event rows are the
  same bytes the controller pushed live during that dive (`E,<t_s>,<id>,<value>`,
  §2), including the verdicts a dive cannot otherwise be reviewed without:
  cellproof PASS/FAIL (9/10), clamp ON/OFF (11/12, value 1 on the OFF row meaning
  the diver lifted it), alarms (1/2), mode (3), setpoint (4). Before that date
  the firmware wrote **no** event rows into the dive file and both counts said 0
  beside a dozen samples — the effects of a decision survived, the decision did
  not.
* `min_ppo2`/`max_ppo2` are `0.000` when no vote ever arrived — not a sentinel.
  The max side used to publish its internal `-32768` verbatim, so a dive with no
  cells reading answered `max_ppo2 = -32.768`: an impossible ppO₂, printed with
  the same three decimals as a real one.
* `gf_low`/`gf_high` and `sp1`…`sp4` are the plan **the dive ran on** — the
  settings in force during it, not the ones on the board afterwards. `0` means
  the firmware had nothing to say: the settings schema will not let a legal
  configuration reach it (`GF_LOW` ≥ 10, `GF_HIGH` ≥ 50, setpoints ≥ 0.20).

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
list reads the file (`/dive/<n>.csv`), not the UART stream: on the stream there
is no marker that tells an exported row from a live one arriving beside it, and
that is a limit of the *reader's* position, not the controller disagreeing with
itself. The two sets are the same rows by construction — the file row and the
stream line come out of one publication — which is what `hw_accept.py
--dive-persist` now asserts: every `E,` row streamed while a dive was open is in
that dive's file, in order. `tools/web_probe.py` still counts exported `E,` rows
without putting them in its row-for-row cross-transport comparison, for the
reader-side reason above.

### 5.1 The surface log rows (`GET /surface.csv`) — web/HTTP only

Calibration verdicts and pre-dive results are recorded on the controller in
`/surface.bin`: fixed 24-byte records, each with its own version byte and CRC16,
read by a reader that stops at the first bad record. Same torn-write contract as
the dive files, for the same reason.

There is **no handset command for this log** — `LOG LIST` and `LOG EXPORT` mean
"the dives", their `L,` and `DIVE,` shapes are already published, and welding a
second class of record onto them would make every existing client re-validate a
shape it already consumes. The rows below are what `GET /surface.csv` answers over
the AP (see §8 for what that costs a wired handset). If a handset command pair is
added later (`SLOG LIST` / `SLOG EXPORT`), it arrives as new commands with new
line tags, never as a new field inside `L,`.

```
SURFACE,<row_format>,<count>,<seq_lo>,<seq_hi>
R,<seq>,<epoch>,<iso_utc>,<id>,<name>,<value>,<raw0>,<raw1>,<raw2>,<spread>,<samples>,<flags>
```

| field | meaning |
|---|---|
| `count`, `seq_lo`, `seq_hi` | records delivered, and the sequence range on the log. `seq_hi - seq_lo + 1 > count` **means history was shed** by capacity: the log admits the gap instead of presenting a short list as the whole story |
| `seq` | 1, 2, 3 … monotonic across reboots, derived from the log itself (this board has no clock to order records with). A compaction never renumbers |
| `epoch`, `iso_utc` | wall clock at the record; `0` and `clock-unset` when nothing had set one. `flags` bit 0 says the clock was carried across a reset, i.e. not verified since |
| `id`, `name` | the same event ids as §4 (`7` cal air, `13` pre-dive pass, `16` pre-dive fail) with a stable name beside them; an id this build cannot name is written `id<n>`, never dropped |
| `value` | id-specific: for 7, the `CALAIR,` code (§3); for 16, the failing step in `PRE,` order |
| `raw0..2`, `spread`, `samples` | raw ADC counts, worst per-cell spread, samples in the window — the numbers that tell a dead cell from a fouled one. Zero for records that are not about the cells. They are integers because the protocol already publishes them as integers |
| `flags` | bit 0 = clock carried across a reset. Append-only like every other column |

Columns are append-only as everywhere in this document: nothing existing moves or
changes units, new information goes on the right of `R,`.

Capacity is 256 records; the oldest are shed first by rewriting the file without
them. That rewrite is staging-then-swap (`/surface.tmp` then rename), so an
interrupted compaction cannot leave a half-written log — at the cost of a small
window in which a power cut loses the surface history (never a dive).


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
| WATER_DENSITY | F | 1.025 | 0.990–1.100 | dive-lock |
| VBAT_MIN | F | 10.5 | 3.0–30.0 | safety |
| AP_SSID | S | `ESP-CCR` | len 1–16 | dive-lock |
| AP_PASS | S | `diveonly` | len 8–31 | dive-lock |

Note: GF_* and ascent rates stay writable **during** a dive (normal diving:
the schedule moves, the gas plan does not); setpoints, alarms and injection do
not.

**`WATER_DENSITY` is the ruler, not a cosmetic preference.** Since 2026-11-24 it
is the one density behind *every* depth this controller publishes: the metres in
the `T,` row, the setpoint tiers those metres select, and the ambient pressures
the deco engine models and derives its ceiling and NDL from. Before that they
disagreed — telemetry used a compile-time 1.025 while the deco engine used this
setting — which put a fresh-water diver's ceiling 2.5% away from the depth he was
reading. A handset that displays depth should therefore read the value back
(`GET WATER_DENSITY`) rather than assume 1.025, and a dive file's depths mean
whatever the setting was during the dive.

It carries the **dive-lock** flag (decided 2026-11-24): `SET WATER_DENSITY=…`
mid-dive answers `ERR dive locked`, because a legal change (0.990…1.100) moves the
diver's whole depth scale ~2.5% at a pressure that has not changed — displayed
depth, tier boundaries, ceiling and NDL together. Setpoints and alarms are locked
for less than that. The flag is bit 0 of the `K,` schema line, so a handset built
from `GET SCHEMA` greys the control out without knowing the key's name, and the
controller's own page disables the input while a dive is running.

**A refusal is not proof that nothing changed.** A client that needs to know
whether a locked write took effect must read the value back, in the same session,
while the state that caused the refusal still holds: `ERR dive locked` only says
what the router *said*. `hw_accept.py` demonstrates the shape — refused write,
then `GET WATER_DENSITY` mid-dive, then the value restored at the surface and read
again — because a post-dive read alone cannot see a value that leaked during the
dive and was rewritten afterwards.

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
* **Numeric ranges are inclusive at both ends, and the endpoints are reachable**
  (fixed 2026-11-24; they were not before). Until then the value was parsed as a
  `double` and compared against a `float` bound, which promotes the bound and moves
  it by up to one ULP — and the published decimal bounds are not exact binary
  numbers. `SET WATER_DENSITY=0.990` answered `ERR out of range` against a declared
  minimum of `0.990`, and **13 of the 30 numeric keys** had an unreachable endpoint:
  SP1…SP4 (min 0.20), `SP_HYST` 0.1, `ALLO_LOW` 0.05, `CAL_O2_FI` 0.85,
  `CAL_O2_TARGET` 0.80, `CELLPROOF_MAXSP` 0.60, `WATER_DENSITY` 0.990 (mins), and
  `ALLO_CUTOFF` 1.80, `CELLPROOF_SP` 1.40, `CELLPROOF_MIN` 1.30 (maxes). Client
  rule: a handset
  or page may offer exactly the `K,` line's min and max, and a host case now walks
  every key asserting both endpoints parse. Corollary for anyone writing a check
  about a *refusal*: do not pick a range endpoint as your probe value, or an
  unrelated refusal looks like the one you were testing for.
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
* **The surface log is web/HTTP only: a wired handset cannot read its own
  machine's history.** A calibration verdict and a pre-dive result are recorded
  durably (§5.1), and `GET /surface.csv` exports them; on UART2 the air-calibration
  verdict is live (`E,*,7,<code>`, §3) but a pre-dive verdict can only be polled
  (`GET PRE`) while something is listening, and nothing on either transport can
  ask "what did the checks say an hour ago". The reason is the append-only rule,
  not laziness: `LOG LIST`/`LOG EXPORT` already mean dives, and a second record
  class inside `L,` would force every published client to re-validate a shape it
  already consumes. Two costs follow and are not hidden: a handset in the water is
  the one client that cannot read them, and a record cannot say which power-up it
  belongs to (no session-start record is written, by the same decision).
* **`OK` means "the command was accepted", not "the effect happened"** — and for
  calibration the effect is now reported separately instead of being implied.
  Fixed 2026-11-24: `CAL O2` and `CAL DEPTH` answer `ERR not implemented` (they
  have no consumer on the board — the O2 sequencer is host-tested and never
  instantiated, and no depth trim exists), and `CAL AIR`'s verdict — including
  the warm-up refusal that used to appear only on the USB console — is on the
  wire as `CALAIR,` and as an `E,<t>,7,<code>` row (§3). Rule that survives:
  never infer an effect from `OK`. Prove it with a read-back of the state the
  command was supposed to change (`GET CALAIR`, `GET TIME`, `GET SP2`, …).
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

The library carries the whole v1 surface, including the wall clock
(`timeSet`/`getTime`/`onClock`, §3), the dive-list sequence field
(`LogEntry::seq`, §4) and the dated export header (`DiveHeader::iso`, §5). If
your handset keeps time, it is the controller's time source: set the clock at
every link-up and show `epoch == 0` to the diver as "dives will be undated".
`handset/README.md` has the working snippets.
