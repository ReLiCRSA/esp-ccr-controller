# ESP-CCR Handset Protocol — v1 (2026-11-23)

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
| Uplink limit | **max 64 characters** per command line. The 65th character discards what was buffered so far and answers `ERR line too long` (at most one per 2 s); the **characters after it still form a line**, so an overlong command can end up executing its own tail as a second command. Keep commands ≤64 chars — the library refuses longer ones before they reach the wire |
| Downlink size | every downlink line fits in **1024 bytes**; clients should buffer ≥1024 |
| Auth / handshake | none on UART. The link is passive: the controller starts pushing telemetry immediately and forever |

The web transport (`ws://<controller>/ws`) feeds the same lines; it adds one
WebSocket-only convention: after every command answer, a marker line
`#<lines>` (or `#<lines> dropped`) is sent so the browser knows the answer is
complete. `#` can never start a protocol line. A UART handset never sees `#`.

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

### Dive log

| Command | Answer |
|---|---|
| `LOG LIST` | `OK` then zero or more `L,<slot>,<epoch>` lines |
| `LOG EXPORT n` (n = 0..9) | `OK` then the dive as lines: `DIVE,<slot>,<startEpoch>,<version>` followed by `S,` snapshot rows, `E,` event rows and one `M,` totals row (format §5.3) |

### SAFE mode (what a handset can and cannot do)

The controller boots into SAFE when it cannot account for its own last reset
(supervisor-detected task stall, watchdog, brownout, panic). The control chain
is **not running** in that state, and the protocol divides cleanly:

| | behaviour in SAFE mode |
|---|---|
| downlink | **unchanged** — `T,`/`D,`/`E,` keep streaming and `T,` carries mode `SAFE`. The sensor task is alive, so a handset can still watch the machine decide. |
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
L,<slot>,<epoch>          dive directory entry (slot 1..10)
K,<key>,<T>,<min>,<max>,<flags>,<default>   settings schema; T ∈ F I U B S (STRING defaults are text)
<KEY>=<value>             settings GET answer
```

## 5. Dive export rows (`LOG EXPORT n`)

```
DIVE,<slot>,<start_epoch>,<format_version>
S,<t_s>,<depth_m>,<voted>,<c0>,<c1>,<c2>,<sp>,0x<alarm16>,<cns>,<ceiling_m>,<ndl>,<deco_min>,<pulses>,<batt>
E,<t_s>,<id>,<value>
M,<duration_s>,<max_depth_m>,<min_ppo2>,<max_ppo2>,<pulses>,<cns_peak_pct>,<snapshots>,<events>
```
Snapshots land on a 30 s grid plus event-driven ones. Handsets can ignore the
export rows entirely (the web page renders them); the format is documented so
a handset *can* offer a dump-to-host feature.

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
* `LOG EXPORT n`: `n` is the slot number itself. The parser accepts 0..9 and
  slots run 1..10, so `LOG EXPORT 0` answers `ERR no such dive` and slot 10
  is currently unreachable from the protocol (it remains visible in
  `LOG LIST` and on the web dives page).
* `E,` ring is 32 deep: a burst can drop events (the controller counts drops);
  a handset should not treat `E,` as a reliable archive — the dive file is.
* **Uplink overflow does not resynchronise to the next newline.** At 65
  characters the UART task throws away the buffered 64, answers
  `ERR line too long` (≥2 s apart) and starts collecting again mid-word — so a
  100-character line can execute characters 66..100 as a command. That is a
  firmware hardening item (§16), and the client rule stands regardless: never
  send a line the library has not length-checked. (The library itself drops an
  oversized *downlink* line whole, which is the behaviour a handset wants from
  a corrupt stream.)
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
