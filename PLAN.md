# ESP32 Nitrox CCR Controller — Master Plan

Rebreather controller on ESP32: 3× R22D galvanic O₂ cells on an ADS1115 (PGA=16),
depth sensing, ZHL-16C decomputation, handset over UART, web UI over a private AP,
and a PC simulator to validate the control/deco logic before hardware trust.

**This file is the product definition — what the machine is and how it behaves.** What
lives elsewhere: **`docs/REVIEW.md`** is the single source of truth for review findings
and open items (start at its §1); **`BENCH.md`** is the acceptance and hardware checklist,
and nothing counts as done until a bench leg watched it happen; **`docs/HANDSET_PROTOCOL.md`**
is the wire contract a handset is built against; **`design/handset-screen/DESIGN.md`**
covers the handset design sheet. A change here to §10 or §11 is a change to the product,
and those documents have to follow it.

## Decisions locked

| Topic | Decision |
|---|---|
| Platform | ESP32, PlatformIO, Arduino framework (FreeRTOS), C++17 |
| Gas config | Nitrox CCR — controller manages O₂ injection only; diluent switching manual |
| ADC | ADS1115, PGA=16, continuous 16 SPS, cells on A0/A1/A2 single-ended; **bench today: 2 cells on A0/A1, A2 free** |
| Air reference | 11 mV @ 21% O₂ nominal; per-cell calibration factors in NVS |
| Depth sensor | **boot auto-detect, identity-gated, MS5837-30bar preferred**; **LPS28DFW (Adafruit #6067)** bench + interim wet-test fallback (BMP085 removed 2026-11-23, never ACKed) |
| ADS1115 DRDY | Not wired on bench → timed non-blocking reads + bus recovery; driver supports DRDY later (`PIN_DRDY=-1` disables) |
| Cell calibration | **2-point**: air (mandatory, always applied) + high-O₂ plateau at configurable `CAL_O2_FI` (default 0.99); scripted `CAL O2` via solenoid enrichment of a sealed chamber |
| Mid-dive cell proof | At ~10 m push setpoint to 1.10; vote group must prove >1.0, else clamp all setpoints to ≤0.90 + alarm (diver-overridable, always logged) |
| Alarms | Low 0.18, high 1.40, injection wall 1.45, hard cutoff 1.60, CNS alarm 80% — all configurable (alarm **below** the wall: §16) |
| Setpoints | 4 configurable with depth crossover + hysteresis (defaults 0.40 / 0.70 / 1.00 / 1.30) |
| Deco | Bühlmann ZHL-16C + gradient factors, GF low/high configurable; UART exposes NDL, total deco time, stop ladder (next stop depth+time) |
| Handset | ASCII line protocol, defined from scratch, UART2 @ 115200 |
| Battery | Full support: divider sense on ADC1, alarms, pre-dive check, per-snapshot logging. The coil rail is the second rail and now has its own sense input, band and verdict (§1 "Power architecture"); it is compiled off (`VRAIL_SENSE=0`) until the divider is fitted, and says so out loud while off |
| Dives | 10-dive FIFO, 30 s snapshots + event-driven snapshots, CSV export |
| Validation | PC simulator + unit tests vs published deco tables before hardware trust |
| Web UI | Mobile-first, clean, responsive (phone portrait → single column; desktop → dashboard grid); no framework, no CDN |

## 1. Architecture

```
espRebController/
├── platformio.ini            # env:esp32 + env:native (unit tests) + env:sim
├── include/config/
│   ├── pins.h                # every GPIO in one header
│   └── defaults.h            # all startup defaults
├── src/
│   ├── main.cpp              # task bootstrap, SAFE-mode boot, reset-reason log
│   ├── sensors/
│   │   ├── ads1115_bus.cpp   # custom hang-proof ADS1115 driver (no stock lib)
│   │   ├── depth_sensor.*    # interface; bmp085.cpp, ms5837.cpp
│   │   ├── dividersense.*    # shared divider DSP: median-of-8 → ratio → EMA
│   │   ├── battery.h         # pack input; chemistry band (Battery : DividerSense)
│   │   └── rail.h            # coil rail input; two-sided band, LOW debounced by sweeps
│   ├── control/
│   │   ├── voting.cpp        # median-of-N, disagreement, health scores
│   │   ├── setpoints.cpp     # SP table, crossovers, clamp machinery
│   │   ├── injector.cpp      # pulse control, stuck-open detection
│   │   ├── calibration.cpp   # 2-point cal, CAL O2 sequencer, TEST CELLS
│   │   ├── cellproof.cpp     # mid-dive high-ppO2 validation
│   │   └── predive.cpp       # check sequencer + AUTO lockout
│   ├── deco/zhl16c.cpp       # 16 compartments, GF, NDL, schedule, CNS clock
│   ├── comms/
│   │   ├── settings.cpp      # ONE key/value registry → NVS (handset+web shared)
│   │   ├── uart_proto.cpp    # handset protocol
│   │   └── web_server.cpp    # ESPAsyncWebServer + WebSocket; the page text is web_page.h
│   ├── log/divelog.cpp       # dive records, events, CSV export
│   └── hud.cpp               # 3-LED pattern engine
├── sim/                      # PC build of the real logic + world model
└── test/                     # native unit tests (voting, deco, setpoints, proto)
```

**RTOS layout**

| Task | Core | Job |
|---|---|---|
| `SensorTask` | 0, high prio | ADS1115 + depth + battery @ ~15 Hz; never blocks |
| `ControlTask` | 1 | voting → setpoints → injector; cellproof/predive state machines |
| `UARTTask` | — | handset line protocol, telemetry at 2 Hz |
| `WebTask` | — | alive only when WiFi enabled (off during dive) |
| `LogTask` | — | LittleFS writes off the critical path |

### Pin defaults (`pins.h`, all changeable in one header)

| Function | GPIO | Notes |
|---|---|---|
| I2C bus 0 SDA / SCL | 21 / 22 | ADS1115 only (the cell chain), 100 kHz, 4.7 kΩ pull-ups |
| I2C bus 1 SDA / SCL | 19 / 18 | depth sensors only (LPS28/MS5837), `Wire1`, 100 kHz, own pull-ups |
| Solenoid drive | 27 | MOSFET gate; **INPUT_PULLDOWN until ControlTask alive**; monitor LED on same net |
| HB / ERR / DECO LEDs | 2 / 4 / 5 | active-high |
| Handset UART TX / RX | 17 / 16 | UART2, 115200 |
| Battery sense | 35 | **ADC1 only** (ADC2 collides with WiFi); divider 100k/10k, `VBAT_CAL` trims. Top leg on the **pack** rail, so `vbat` is the cell and says nothing about the 12 V the solenoid runs on — that second number is the row below |
| Coil rail sense | 39 | the ~12 V boost rail the valve coil actually runs on (§1 "Power architecture"). ADC1, input-only, same 100k/10k ratio as the pack so the two numbers share one metering procedure. **`VRAIL_SENSE=0` today: the divider is not fitted**, and the firmware then says so out loud at boot and in HB (`vrail=0.00 railstate=3`) instead of leaving the silence to be read as a pass. BENCH.md C3-c is the fitting-and-metering leg |
| Battery midpoint sense | 36 | series packs only, and compiled only when `VBAT_CELLS >= 2`: the pack voltage alone cannot see one dead cell (6.80 V = 4.00 + 2.80). ADC1, input-only, no internal pull; unwired is a **refusal**, not a pass |
| ADS1115 DRDY (optional) | 18 | supported by driver; `-1` disables (bench today) |

### Power architecture (what feeds what — docs/REVIEW.md 1 P9)

Written down 2026-11-28 because P9's whole diagnosis was that this existed only in
the builder's head: every runtime number the project quotes (dive-length margin,
boost input current at a pulse, expected sag) is uncomputable while the chain is
undocumented. Facts as the builder described them, each marked with what it is.

| Stage | Fact | Status |
|---|---|---|
| Pack | **one protected 18650** (protection board visible on the bench). `VBAT_CELLS 1` follows | builder statement 2026-11-28 |
| Logic supply | pack → **its own 5 V regulator** → the board's 5V/3V3 side. **Not** fed from the boost | builder statement 2026-11-28; the reason a dead boost leaves the brain alive and the coil dead — the split that makes rail sensing worth a pin |
| Coil rail | boost module marked **"TEAYUEW mini boost module 0260508"** (Amazon generic), ~12 V out, feeding **the valve coil only** | marking recorded; the chip on the module is **unidentified** — a generic clone family, and no search of the marking resolved it. Treat part, input range and switch-current limit as unknowns |
| Valve | **Heschen 2V025-08**, DC 12 V coil — the AirTAC 2V family's `F` voltage code. Family catalogue figures: direct acting, normally closed, 2.5 mm orifice, Cv 0.25, working pressure 0–1.0 MPa, activating time ≤ 0.05 s, permitted coil-voltage variation ±15% | family figures from the catalogue page for the 2V025-08 (airtacs.com product page, fetched 2026-11-28); **not** a per-unit datasheet for the part on this bench |
| Coil DC resistance | **unknown.** The catalogue publishes 7.0 VA for the AC110 coil option, which says nothing about this DC12 coil. A meter across the coil is BENCH.md C3-c | unknown, deliberately not guessed |
| Minimum working differential pressure | **unknown and load-bearing.** A pilot-operated 2V025 needing ~1 bar differential would not open at ambient loop pressure, and that would be a design fault no firmware check can paper over. The pre-dive SOLENOID step's ppO₂ rise is the only evidence this project has ever had that the valve opens, and the bench has never gotten past that step | unknown; C3-c leg before any claim either way |

The sense decision that follows from this table: the boost output gets a divider on
**GPIO39** (`PIN_VRAIL_SENSE`), same 100k/10k ratio as the pack, and its verdict
rides the pre-dive BATTERY step (fourth question, appended inside the step — the
`PRE,` wire shape cannot gain a sixth field), `BIT_BATTERY` (one marker, the
console says which rail), and HB (`vrail=`/`railstate=`). Publication on the
dive-file/`T,` surface is **deliberately deferred until the rail is metered** —
publishing an unmetered number on the wire is exactly how `vbat` became P1.
Until the divider is fitted, `VRAIL_SENSE=0`: nothing judges the rail, and boot,
the `[RAIL]` console line and HB `vrail=0.00 railstate=3` all say so, because a
silent absence is what got read as a pass before.

## 2. Safety skeleton (cross-cutting)

The ADS1115/ADS1x15 hang problem (blocking reads during conversions, I2C
lockup with SDA held low, MUX/CONFIG churn in continuous mode) is treated as a
life-safety issue:

1. **Three watchdog layers**
   - TWDT on SensorTask and ControlTask.
   - Hardware RNG-WDT fed only when both tasks heartbeat.
   - Any watchdog reset → boot into **SAFE mode** (no injection, alarm, HB
     double-beat) requiring handset `ACK`; reset reason logged every boot.
2. **Solenoid failsafe at GPIO level**: drive pin failsafe-low; driver circuit
   must be gate-low = valve closed. Firmware walls: never inject above 1.45;
   drive forced OFF at ≥1.60 regardless of mode.
3. **I2C hang-proofing**
   - ADS1115 config register written **once** at init (continuous mode, 16 SPS,
     PGA=16) and never touched again.
   - Reads scheduled by conversion timing (62.5 ms period) — no polling of the
     conversion-done bit, no busy-waits.
   - All transactions under an I2C mutex with hard timeouts.
   - Failure → **bus recovery**: ≤16 bit-banged SCL clocks + STOP + `Wire.begin()`
     re-init. Recovery counter surfaced on web/handset; sustained recoveries →
     warning.
   - Same raw value for >5 s → sensor-health fault.
   - 100 kHz bus speed, 4.7 kΩ pull-ups.
4. **Reading plausibility**: absurd raw values or implausible sample-to-sample
   jumps rejected before voting.
5. **Degradation ladder** (ppO₂ sensing): 3 healthy → 2 (vote = **min** of two,
   conservative) → 1 (use it + alarm) → 0 (**SAFE mode**, solenoid off).
   Bench runs the full 2-cell path natively; 3rd cell auto-detected when fitted.
6. **Stuck-open solenoid**: ppO₂ rising with zero commanded pulses → drive cut
   + alarm.
7. **Dropout discrimination**: one cell collapses vertically = sensor fault;
   all cells collapse = real hypoxia. Both alarm; different ERR LED patterns.
8. Hardware guidance (documented, not coded): physical manual-inhibit switch in
   the O₂ line; keep max solenoid duty low (≤5%) so runaway firmware cannot
   flood the loop.

## 3. Signal chain & calibration

- PGA=16 → ±256 mV, 7.8 µV LSB: 11 mV ≈ 1408 counts, ~0.001 bar resolution.
- **Voted ppO₂ = median of healthy cells.**
- **Disagreement**: |cell − median| > `CELL_DRIFT_BAR` (default 0.05) for 10
  consecutive samples → cell excluded from vote, logged, warning LED.
- **Health score** per cell: rolling variance vs median + responsiveness to
  injection pulses (dead cells stop responding). Drives preferred-cell display
  and early degradation warnings before outright failure.
- **2-point calibration** per cell (NVS):
  `ppO2 = (mV − mV₁)/scale + 0.21`, `scale = (mV₂ − mV₁)/(CAL_O2_FI − 0.21)`.
  - Cal 1 (air, 0.21) **mandatory** — system never arms without it.
  - Cal 2 at `CAL_O2_FI` (default 0.99 — settable, gas need not be pure).
  - Sanity gates: implied slope within `CAL_SLOPE_BAND` (±15%) of single-point
    slope, plausible offset → else reject 2-point **for that cell**, keep
    air-only cal, warn, log.
  - Calibration age shown; staleness flagged.
- **`CAL O2` scripted enrichment calibration** (bench/surface only):
  1. Preconditions enforced: air-cal passed, cells warm, depth ≈ 0, not dived;
     two-step confirm (`CAL O2` then `CAL O2 CONFIRM`) — it enriches a closed
     volume toward 1.0 bar O₂, must never fire from a stray UART byte.
  2. Inject pulses until plateau ≥ `CAL_O2_TARGET` (0.95) or timeout; hard cap
     `CAL_O2_MAX_PPO2` (1.0); capped soak time at plateau (cells degrade).
  3. Hold, sample 30 s medians; validate spread / stability / plateau; compute
     Cal-2 per passing cell.
  4. Captures rise time + pulse responsiveness per cell → feeds health scores.
- **`TEST CELLS`** standalone: the same pulse-and-measure scripted check
  (inject a few seconds, verify all cells respond) without committing a cal.
- Temperature: LPS28/MS5837 die temp for coarse cell compensation (~2%/°C);
  dedicated loop temp sensor deferred to prototype.

## 4. CellProof — mid-dive high-ppO₂ validation

Galvanic cells that are linear at 0.4 but saturate at 1.0 pass an air cal and
lie at depth. CellProof catches exactly this:

- Arms **once per dive** on first descent through `CELLPROOF_DEPTH` (default 10 m).
- Ramps active setpoint to `CELLPROOF_SP` (1.10) — ramped, and never above the
  1.45 hard wall.
- **Pass**: voted ≥ `CELLPROOF_MIN` (1.00) within `CELLPROOF_TIMEOUT` (60 s),
  cross-checked against commanded-O₂ pulse integration so a genuinely slow loop
  doesn't blame healthy cells.
- Per-cell: cells that reach the group = **proven**; lagging cells =
  **unproven** → excluded from voting whenever active SP > 1.0.
- **Failure policy**: vote group fails → clamp every setpoint above
  `CELLPROOF_OK_MAXSP` (default 0.90) to that cap **for the rest of the dive**;
  alarm + ERR pattern + forced snapshot; web/handset show the clamp and its
  expiry (= dive end). `CELLPROOF OVERRIDE` acknowledges and lifts the clamp —
  diver outranks machine, but it is loud and logged.

## 5. Setpoints & injection control

| SP | Default | Active when |
|---|---|---|
| SP1 surface | 0.40 | depth < 2.0 m |
| SP2 in-water | 0.70 | 2.0 m – D1 |
| SP3 deep-1 | 1.00 | D1 (30 m) – D2 |
| SP4 deep-2 | 1.30 | > D2 (50 m) |

- Crossover depths + `SP_HYST` (0.5 m) prevent flapping; every SP change logged
  with a forced snapshot. Clamps (cellproof, high-ppO₂) apply as a layer on top.
- **Bang-bang + pulse width**: inject `INJ_PULSE_MS` (300 ms) when
  voted < SP − 0.03; stop above SP + 0.02; max duty `INJ_MAX_DUTY` 5%.
  Deterministic, no windup — right behavior for a CCR. Low ppO₂ increases
  aggressiveness but never bypasses the high wall.
- Bench: solenoid GPIO also drives a monitoring LED (same net) so injection is
  visible. Test pulses come only from the engines — the pre-dive solenoid step
  and `TEST CELLS`; there is no handset manual-pulse command (`PULSE` was
  retired 2026-11-26, §16).

## 6. Deco engine — ZHL-16C + GF

- 16 N₂ compartments, fitted nitrox parameters; inspired ppN₂ from **measured**
  ambient pressure (depth) and **measured** loop ppO₂ — rebreather-correct inert
  partial pressure, O₂ treated as inert.
- GF low/high configurable (default 30/85) → ceiling depth, stop ladder
  (depth + time per stop, ~6 stops), total ascent time at `ASC_RATE` (6 m/min)
  with last 3 m at `ASC_RATE_SLOW` (1 m/min).
- **NDL** = no-stop time remaining at current depth: can a **direct ascent**
  (config rates + deco gas assumptions, no stops) still be flown without
  breaking the ceiling anywhere? That is the dive-computer/published-table
  meaning. (The first revision asked "when does the ceiling exceed current
  depth", which never happens at CCR setpoints and reported 480 min at 30 m;
  fixed 2026-11-23.) 0 whenever a stop is already obligatory.
- **CNS O₂** single-trigger clock (NOAA table), alarm at 80%.
- **Cross-validated 2026-11-23** (`test/test_deco/test_published.cpp`):
  parameters cell-by-cell vs Wikipedia ZH-L16C table and Subsurface
  `core/deco.cpp`; M-form `M = a + P/b` and ceiling algebra vs Subsurface and
  repied/paliers `src/gf.ts` (run as an independent oracle under node); stop
  schedules vs the published "Buhlmann Air Decompression Tables" (Deco_Package,
  tech-diving.ru) at GF100/100 with the tables' own conventions — 25 entries,
  mean delta ≈ −2.8 min, fully attributable to ZH-L16 **B**-(tables) vs
  **C**(engine) scheme and wet vs dry alveolar ppN₂; NDL transitions land on
  the published ones (30 m: engine 19.7 min, tables flip at 20). Two real
  bugs fixed on the way: the M-form (was `a + b·P` — inflated deco 2–3×)
  and the NDL definition above; also the CNS table (NOAA 0.8→450 not 360,
  0.7→570 not 540).
- **GF variant, decided:** per-tissue GF interpolation (gfLow→gfHigh over
  the a-coefficients — the classic dive-computer model, what Shearwater/DJD
  do). paliers/Subsurface planners interpolate GF by depth instead; at
  symmetric GFs the variants coincide (that is where the tight cross-impl
  asserts live), at 30/85 the per-tissue variant is systematically longer —
  documented band, not an identity claim.
- Caveat: simulator + unit validation is bench confidence, **not** a dive-
  release certification.

## 7. Pre-dive sequence (AUTO lockout)

```
IDLE → WARMUP (120 s, cells)
     → AIR CHECK (all cells within 8% of 0.21)
     → SOLENOID TEST (ppO2 rises ≥0.03 within 2 pulses, then falls back;
        per-cell response measured)
     → DEPTH ≈ 0 + sensor-identity check (MS5837 coefficients match NVS copy)
     → BATTERY ≥ min, AND the reading must be plausible for the chemistry
        (above the floor but above `VBAT_IMPLAUSIBLE_V` is a sense node on its
        rail, and refuses), AND on a series pack the two halves must agree
        within `VBAT_CELL_SPREAD_MAX_V` — an unwired midpoint refuses too,
        AND (when `VRAIL_SENSE=1`) the coil rail is inside its band — never
        sensed refuses outright, out-of-band waits `VRAIL_SETTLE_MS` and
        then fails; the step count stays five because `PRE,`'s passed bit is
        positional (see §1 "Power architecture" and docs/REVIEW.md §1 P9)
     → PASS (enables AUTO)
```

No AUTO before PASS. `CHECK` re-runs the sequence; the web checklist page mirrors
every step with pass/fail; every step logged. `CAL AIR` stores cell scale
factors; `CAL DEPTH` surface-tares depth.

## 8. Handset UART protocol (UART2 @ 115200, ASCII lines)

**Downlink** (2 Hz):

```
T,<depth_m>,<voted>,<c1>,<c2>,<c3>,<sp>,<mode>,<flags_hex>,<vbat>
                                    ^ vbat appended last: a client that stops at
                                      FLAGS is still correct (add-only rule)
D,<ndl_min>,<deco_total_min>,<ceiling_m>,<stop1_depth>|<stop1_min>,<stop2_depth>|<stop2_min>,...,<cns_pct>
E,...            event lines (alarms, SP/mode changes, cellproof, calibrations) on change
OK / ERR <why>   command replies
```

**Uplink commands**:
`GET *|<key>` · `SET <key>=<val>` · `CHECK` · `AUTO` · `MANUAL` · `ACK` ·
`CAL AIR` · `CAL O2` / `CAL O2 CONFIRM` · `CAL DEPTH` · `TEST CELLS` ·
`CELLPROOF OVERRIDE` · `DECO` (one-shot `D,` line) ·
`DIVE START|END` · `LOG LIST` · `LOG EXPORT <n>` · `WIFI ON|OFF`.

Display keys (resolved **after** the registry, so a real key like `D1` or
`CAL_O2_FI` can never be shadowed): `GET T` · `GET D` · `GET CELLS` ·
`GET PROOF` · `GET PRE` · `GET FAULT` · `GET SCHEMA` · `GET HIST`
(`HIST,<cell>,<winS>,<windows>,<binWidth×100>,<ageS>,<folded>,<38 counts>`
per cell — the rolling shape, whole lines or nothing).

Unknown command → `ERR`; all writes reply `OK`/`ERR <reason>`. The handset and
web edit the identical settings registry — one code path: `comms::CommandRouter`
(`src/comms/cmd_router.{h,cpp}`, host-tested in `test/test_router/`) executes
every parsed line and returns hardware side effects as action bits (dive log
open/close, radio, CAL procedures, log export), so no client grows a second
path around the pre-dive AUTO lockout, the dive-locked settings or the
two-step `CAL O2` confirm. Registry defaults are asserted equal to the
firmware constants (`test_settings`), and everything steering the gas loop
(SP*, ALLO_*, CNS_ALARM, INJ_*, CELLPROOF_SP/MIN/OK_MAXSP) is `FL_DIVE_LOCK`;
schedule tuning (D1/D2, GF_*, ascent rates, drift) stays writable mid-dive.

`GET SCHEMA` answers one line per key —
`K,<key>,<type>,<min>,<max>,<flags>,<default>` — so a client can put real
min/max on its inputs and grey out what a dive locks, instead of inventing
limits the server would refuse anyway.

One session, not one per client: `comms::Session` (`src/comms/session.{h,cpp}`,
host-tested in `test/test_session/`) owns the router, the protocol link, the
session lock and the dive-file owner, and every transport calls `handleLine()`
on it. Per-transport routers looked harmless and were not: each keeps its own
memory of what the diver asked for, so a handset would bring the radio back at
dive end because *its* router never heard the phone turn WiFi off.

**The full wire contract, written out for whoever builds the handset:**
`docs/HANDSET_PROTOCOL.md` (v1) — every command with its exact reply and
refusal strings, every downlink format, the append-only evolution rules, and
the current limits stated honestly. Client library `handset/handset_link.h`
(header-only C++17, transport-agnostic, link-health watchdog), reference
handset firmware `handset/example_esp32_handset/`, orientation
`handset/README.md`. Drift guard: `test/test_handset/` feeds the library the
firmware's own wire literals.

## 9. Web server (WiFi truly off during dive)

AP `<AP_SSID>-<mac tail>` (default `ESP-CCR-A1B2`) with the WPA2 password from
the registry (`AP_SSID`/`AP_PASS`, both `FL_DIVE_LOCK`: not editable during a
dive). On `DIVE START` the **radio is disabled** (not just the server down —
saves ~40–70 mA too); auto-on at dive end; handset `WIFI ON|OFF` override.

**Built** (`src/comms/web_server.{h,cpp}`, deps pinned in `platformio.ini`:
`ESP32Async/AsyncTCP @ 3.6.1` + `ESP32Async/ESPAsyncWebServer @ 3.12.1` — the
older forks' download URLs are dead):

- `/ws` carries the handset protocol verbatim: one frame in = one command line
  through `comms::session()`, reply frames out on the same socket. No JSON
  command layer, no REST for settings — a browser cannot reach anything the
  handset cannot, and cannot skip a lockout either.
- **Answer framing:** the protocol is line-based and a command may answer with
  many lines (`GET *`, `GET SCHEMA`, `LOG EXPORT`), so the bridge ends every
  command with a `#<n>` control line (n = reply lines) and splits multi-line
  replies into one frame per line. `#` can never start a protocol line, so
  nothing collides. A browser then knows an answer is complete, and its
  staleness banner is a round-trip timeout rather than a guess.
- Pages: served from LittleFS `/index.html` when present, otherwise from the
  build (`src/comms/web_page.h`) — so a wiped or unmounted filesystem costs the
  UI nothing. `tools/sync_webdata.py` writes `data/index.html` from that same
  literal for `pio run -e esp32 -t uploadfs` (one source of truth). Built-in
  page: status, checklist, settings (ranges from `GET SCHEMA`), dives
  (list → inline depth/ppO₂ chart → CSV), console. 23 KB, no CDN, no framework.
   One caveat found on the bench 2026-11-23, in section 16: `uploadfs` writes
   the partition the dive logs share, so it erases dive history.
   `web_probe.py` compares the served bytes to `data/index.html` so a page
   install that did not land cannot pass unnoticed.
  - **Cell histograms**: `ctrl::cellhist` (`src/control/cellhist.{h,cpp}`) —
    rolling 10-minute horizon per fitted cell (10 windows × 60 s ring, 38
    bins of 0.05 bar, fixed static, no heap), fed by ControlTask once per
    cycle from the calibrated cells (fresh + fitted only — a frozen ADC
    sample must not reshape history). Served as `GET HIST` through the same
    session, rendered as the built-in page's Cells tab. Display-only read
    across threads per the A5e rule: no decision hangs on a chart.
  **Not built yet:** the service worker.
- `GET /dive/<n>.csv` answers the same dive file as `LOG EXPORT <n>`, through
  the same session lock, as a file download for the browser.
- Task layout: `async_tcp` is pinned to **core 0** (with the WiFi driver) so
  core 1 keeps control (prio 22), handset (12) and HUD, and the sensor task
  (24) still preempts it. It is deliberately **not** registered with TWDT: a
  wedged web server must never reset a diving controller.
- Sockets are never closed from the command path — a client that sends
  `DIVE START` must not have its own socket destroyed inside its own callback.
  The radio going down ends the connections; `start()` runs `cleanupClients()`
  afterwards.

### Mobile-first, responsive UI

- Plain HTML/CSS/vanilla JS from LittleFS. **No framework, no CDN** — the AP
  has no internet, ever. Tiny WebSocket + DOM helper layer.
- CSS grid: **single narrow column on phone**, multi-panel dashboard on
  desktop/tablet. Touch targets ≥44 px, no hover-dependent UI.
- Page budget ~50 KB + shared ~20 KB CSS/JS → instant over ESP32 flash serving.
- Dark high-contrast theme, big numerals (the ppO2 number dominates). Page
  accent color shifts with system state: clean / amber warning / red alarm —
  glanceable.
- `viewport` meta + apple-mobile-web-app-capable: "Add to Home Screen" gives a
  fullscreen app feel; a tiny service worker caches pages so the UI opens
  before the WebSocket connects.
- WebSocket auto-reconnect + staleness banner — always obvious whether the
  data is live.
  (built: exponential-backoff reconnect, `NO DATA` banner, accent colour follows the alarm set)
- State changes (mode, clamps, WiFi off) push toast notifications.
- Client-side min/max on inputs, re-validated server-side through the same
  `SET` path as the handset.

### Pages

| Page | Content |
|---|---|
| **Status** | Big ppO₂ + target, 3 cell bars + health, depth, mode, ceiling/NDL/deco ladder, CNS, battery, clamp status, radio state (`GET WIFI`: up + stations, or down + the console's own reason word; idle countdown only while no station is associated) |
| **Histogram** | Per-cell ppO₂ histograms (~60 bins, 0–1.9 bar, ~15 Hz accumulation since power-on) — the pre-dive cell-health picture; side-by-side desktop, tabbed on phone; canvas, no chart lib |
| **Settings** | Grouped accordions; same registry as handset; dangerous actions behind confirm dialogs |
| **Checklist** | Pre-dive stepper + calibration wizard (`CAL O2` sequence live) |
| **Dives** | 10-dive list → detail with inline depth/ppO₂ charts → CSV export; the detail also lists the dive's `E,` verdict rows in words (alarms, cell-proof FAIL, clamp lifts, calibration outcomes) — what the machine decided, not just where the diver went |

## 10. Dive logging (LittleFS)

- **30 s snapshots**, 32 B binary records: depth, voted ppO₂, 3 cells, active
  SP, mode + alarm bitmask, CNS%, ceiling, NDL, deco-total, pulses in window,
  battery, temp → ~4 KB per dive-hour.
- **Event-driven snapshots** on any alarm / SP change / mode change / cellproof
  / calibration — the 30 s grid never misses the moment of interest.
- **Dive header**: max depth, bottom time, min/max ppO₂, total pulses (≈ gas
  used), full event log with timestamps, GF + SP config in force, calibration
  age/quality, cellproof outcome, cell health start/end.
- FIFO of **10 dives**; CSV export via handset and web.

## 11. Settings registry

Single source of truth, NVS-persisted, identical surface over handset `SET/GET`
and the web Settings page.

| Group | Keys (defaults) |
|---|---|
| Alarms | `ALLO_LOW` 0.18 · `ALLO_HIGH` 1.40 · `ALLO_CUTOFF` 1.60 · `CNS_ALARM` 80 |
| Setpoints | `SP1` 0.40 · `SP2` 0.70 · `SP3` 1.00 · `SP4` 1.30 · `D1` 30 · `D2` 50 · `SP_HYST` 0.5 |
| Injection | `INJ_PULSE_MS` 300 · `INJ_MAX_DUTY` 10 · deadbands |
| Voting | `CELL_DRIFT_BAR` 0.05 · drift window |
| Calibration | `CAL_O2_FI` 0.99 · `CAL_O2_TARGET` 0.95 · `CAL_O2_MAX_PPO2` 1.0 · `CAL_SLOPE_BAND` 15 · per-cell cal factors |
| CellProof | `CELLPROOF_EN` on · `CELLPROOF_DEPTH` 10 · `CELLPROOF_SP` 1.10 · `CELLPROOF_MIN` 1.00 · `CELLPROOF_TIMEOUT` 60 · `CELLPROOF_OK_MAXSP` 0.90 |
| Deco | `GF_LOW` 30 · `GF_HIGH` 85 · `ASC_RATE` 6 · `ASC_RATE_SLOW` 1 · `WATER_DENSITY` 1.025 |
| Battery | `VBAT_MIN` (one knob; band is per-cell × `VBAT_CELLS`, 2.80…4.50 V — tightening it is also what clears a stale 10.5 V from NVS) · `VBAT_CAL`. The floor, the plausibility ceiling and the cell-spread limit are **derived from `VBAT_CELLS`** in `defaults.h`, not keys: a writable ceiling would only re-arm the hole it closes |
| System | WiFi AP SSID/pass · telemetry rate (depth source is boot auto-detect, not a setting) |

## 12. HUD (3 LEDs)

| LED | Pattern |
|---|---|
| **HB** | 1 Hz = AUTO alive · slow = standby · **double-beat = degraded or post-reset SAFE** — a glance tells the state |
| **ERR/WARN** | off = clean · slow blink = warning (cell disagreement, high warn, low battery) · fast blink = alarm (low ppO₂, dropout, cellproof clamp) · solid = hard fault / SAFE mode. Cleared via `ACK` |
| **DECO** | solid while the GF ceiling demands a stop |

## 13. Depth sensor abstraction

`DepthSensor` interface (`pressure_bus.h`); `ms5837.cpp` (dive-rated wet
prototype) and `lps28.cpp` (bench + interim wet-test, ST LPS28DFW on the
Adafruit #6067 breakout). Selection is **boot-time auto-detect, not a build
constant** (decision 2026-11-23, `depth_manager.h`): the arbiter gives the
MS5837 the first window (reset + factory PROM CRC-4 must pass), then the
LPS28 (WHO_AM_I `0xB4` only — LPS22 near-relatives refused); whichever is
identified owns depth **locked until the next power cycle**, so a sensor
swap is fit-and-power-cycle with no reflash, and nothing can change the
depth source mid-dive. Nothing fitted → windows keep recirculating, so a
hot-fitted sensor appears in seconds and the boot log names the winner
(`depth: LPS28DFW @0x5D ready, 1016.7 hPa` — the hPa is a first **reading**, not
the state of the lock, see 16) — which sensor dove is part of the
record. The BMP085 that held the bench slot is **removed**: it never ACKed
once, and a sensor that has answered zero transactions in its life earns no
place in the tree. The LPS28DFW is dual full-scale — 260–1260 hPa (mode 0)
or 260–4060 hPa (mode 1); firmware always sets **mode 1**, because mode 0
clips at ~2.4 m of sea water, inside the 10 m cell-proof test. The 0x5C/0x5D
choice made by the breakout's XSA pad is resolved by probing both.

Depth lives on **Wire1 (SDA 19 / SCL 18)**, its own pin pair and its own
4.7 k pull-ups: a wet depth harness short or latched depth chip can never
wedge the bus feeding ppO2, and the bit-banged bus recovery (`i2c_recovery.h`,
one mechanism used by BOTH buses) stays local to each bus. One task still
owns every transaction on both buses; each Wire timeout still caps every
call.

MS5837-30bar specifics:

- CSB tied to GND → address `0x76` (kept off 0x5C–0x5D/0x48 territory; both
  bench and wet depth sensors can sit on one bus during transition testing).
- OSR 4096 (~10 ms) pressure + temperature, interleaved at ~8 Hz; same
  non-blocking command→wait-on-timestamp→timed-read pattern as the ADC driver.
- Factory coefficients C1–C6 read at init, cached, saved to NVS → a swapped
  sensor is detected (coefficient checksum mismatch = pre-dive failure, and a
  sensor-identity check).
- **Errata 2 correction implemented** (`GShock`/`OT` oversampling/low-temp
  fixups) — without it readings drift below ~10 °C, exactly diver territory.
- Resolution ~0.5 mbar ≈ 5 mm at depth — ample for setpoint crossovers and
  deco ceilings.
- Depth = f(pressure, `WATER_DENSITY` default 1.025) with `CAL DEPTH` surface
  tare stored in NVS.
- Wet-prototype practice: MS5837 must be potted (epoxy) with a vented capillary
  for pressure; the die is not directly submersible. Bench-validate the real
  driver against the LPS28 side-by-side before potting.

## 14. Simulator (`sim/`, native PlatformIO target)

PC build of the **real** firmware logic — voting, setpoints, injector,
calibration, cellproof, ZHL-16C, logging, UART protocol — with the hardware
layer (ADS1115, I2C, GPIO, NVS, WiFi) stubbed. Not a re-implementation.

**Status: built and green (`src/sim/`, `test/test_sim/`, 7 scenarios).**
The chain itself moved into `ctrl::Loop` (`src/control/loop.{h,cpp}`), which the
firmware `ControlTask` instantiates unchanged — `sim/rig` is now only the bolt
between that chain and the gas model. `test/test_loop/` covers what only exists
between the modules (staleness gating the valve, the output fuse, the AUTO
lockout, a spiked depth sample vs the bands). `test/test_divesoak/` is the M6
exit criterion: **50 seeded dives** (7 of them pre-dive lockouts) with faults
injected through the world only, asserting per-tick invariants rather than
expectations. Suite total: **157 cases across 24 native suites**, plus
`pio run -e esp32` and `esp32_stalltest` building clean.

The soak is checked for **sensitivity, not just greenness**: removing the
per-cell high veto, moving the high alarm back to the wall value, un-binding the
CellProof verdict from the vote, or dropping the codec's high alarm bits each
makes it fail. What it deliberately cannot reach: the staleness gate, the depth
filter and the stuck-open hysteresis (the gas model never makes a sample stale
or a reading dither) — `test_loop`, `test_depth` and `test_injector` own those.
Two real firmware bugs and one modelling bug came out of writing it, all
recorded in §16.
`sim/world.{h,cpp}` is the physics side (loop O₂ content in litres-atm, SAC ×
ambient consumption, deterministic LCG noise, cell faults DEAD / SATURATE /
SLOW / STUCK_AT, stuck-open solenoid, diluent flush). `sim/rig.{h,cpp}` wires
the control chain in exactly the order ControlTask will — cells → voting →
pre-dive (which drives `cal::PulseTest` at its solenoid step) → CellProof →
setpoints → injector → alarms → deco — so a scenario that passes here is
testing the real interaction, not a paraphrase. Faults are injected **only**
through the world; nothing pokes a control module. Scenarios: clean 40 m dive
(predive → AUTO lockout → CellProof → deep band → CNS), one saturated cell
(drift exclusion + CellProof naming it), mid-dive cell death (dropout ≠
hypoxia), two cells lying low (high-veto saves the loop), diluent-flush
hypoxia (latch + ACK + recovery), poisoned cell failing the pulse test
(no AUTO), stuck-open valve (latch + explicit clear).
Still to do here: the `T,`/`D,`/`E,` stdout bridge and the headless web view.

- **World model**: scripted depth profiles (embedded set + CSV input),
  configurable SAC rate, loop volume, air diluent. Virtual cells with real
  physics: injection raises ppO₂, metabolism lowers it, proportional to ambient
  pressure; noise, drift.
- **Fault injection flags**: dead cell, noisy cell, slow cell, **cell saturated
  above ~0.8 bar**, cell disagreement at time T, stuck-open solenoid.
- Runs the real control loop and asserts: setpoints held, alarms fire on time,
  voting degrades correctly, **CellProof fires on the saturated cell and the
  clamp engages**.
- Output: identical `T,`/`D,`/`E,` UART stream to stdout (handset protocol
  testable) + real dive-file format (log format validated as a side effect) +
  optional headless HTTP (localhost) to watch a simulated dive in the web UI.
- **Deco schedule calculator**: dive profile in → full stop table out → compared
  against unit-test expectations and an independent reference implementation.

## 15. Milestones (bench-first)

| # | Deliverable | Exit criteria |
|---|---|---|
| **M1** | Scaffold, `pins.h`/`defaults.h`, hang-proof ADS1115 driver, I2C bus recovery, 3 watchdog layers, battery sense, HB LED | **DONE 2026-11-22 (12.49 h soak, ended early by owner decision after 12 h clean)**: 8992 status lines, yank drills = 77 I²C errors absorbed by 38 bus recoveries, **0 boots, 0 SAFE, 0 FATAL, 0 HB gaps**; cells stable ~1490 counts, `fresh=110` all run. Summary: `logs/soak_20261122_094738_summary.json` (regenerated from the raw log after SIGINT skipped the JSON). M1 firmware `0e46252`, tag `m1-soak-flashed` |
| **M2** | Median voting + health scores, 2-cell bench mode, solenoid LED pulsing, `CAL AIR`, `CAL O2` + `TEST CELLS` sequencers, pre-dive sequence | Bench pre-dive passes with 2 cells; scripted enrichment cal works end-to-end on the bench jar setup |
| **M3** | BMP085 driver (+ breathe-on-it depth sim), setpoint crossovers + clamps, dive file format, MS5837 driver incl. Errata 2, CellProof | Setpoint crossovers verified by breathing on the BMP085; dive CSV exports correctly |
| **M4** | **Simulator-first** deco: ZHL-16C + NDL + schedule + CNS → unit tests vs published tables → sim world model + fault injection → DECO LED + `D,` UART line + web ladder | Unit tests match published NDL/deco tables; sim soak runs show no setpoint escapes |
| **M5** | UART protocol complete, web server (status/histogram/settings/checklist/dives, mobile-first), WiFi auto-off | Handset integration on real UART; UI usable on a phone over the AP |
| **M6** | System sim soak + hardening: 50 scripted dives incl. faults; MS5837 swap/potting checklist; battery redundancy notes | **Sim half done:** 50-dive seeded soak green (`test_divesoak`), mutation-checked for sensitivity; it paid for itself with the CellProof 2-vs-1 starvation chain (§16). Bench half pending the port: MS5837 swap + potting checklist, mechanical inhibit bench test. Residual hardening candidate: a scheduled proof (`maxAttempts`/cooldown currently bound the cost) and the diver's own bailout as the physical backstop |

### Current work order (between milestones)

The 24 h soak ended on 2026-11-23 (owner decision after 12.49 clean hours, so the
M1 row is closed) and the port has been free since: everything below is flashed
and bench-verified unless its own row says otherwise. What is left lives in the
C2 row's open clause, in C3, and in the recorded-but-unfixed items of 16.

| Step | Deliverable | State |
|---|---|---|
| A1 | `ControlTask` runs the real `ctrl::Loop` on the board (compile-only when it was written) | done `1b921b4` |
| B1 | 50-dive seeded sim soak as the regression net for all logic work | done `bda1825` |
| A5a | one command router for handset and web — refusals are the surface | done `f247d30` |
| A5b | one settings applier (in-place re-tune) + NVS + dive-file store | done `b322bc0` |
| A5c | handset session on UART2 | done `0e681e1` |
| A5d | web AP + `/ws` bridge + built-in page + dive CSV | done `6c11ac6` — web-fix build `a8cd6bf`, dead page tabs fixed `70049fa` |
| A5e | close the control-vs-command race: defer chain mutations (`setConfig`, `startDive`/`endDive`, pre-dive, pulse requests) into ControlTask's cycle boundary, so the handset/web task never mutates the chain mid-`cycle()` on the other core; host test | done `0d446fc` |
| C2 | bench acceptance: `tools/hw_accept.py` (boot checks + refusal matrix over a wired handset; `--replay` validated against the soak log) + `BENCH.md` sequence, then flash the current build | **PASSes on the flashed build: 48 checks 0 failures** (harness `342d2b3`, first green `1257bec` on `1b8c807`, green at `b1d8d2e` (tag `m4-radio-cycled`) and again at `0256df6`, which added the `log:` boot check and `--dive-persist`): boot console (fresh-boot banner, HB contract, no SAFE/FATAL, `settings: 32 keys from NVS`, `web: AP "..."`), the refusal matrix over the spare-ESP32 handset bridge (`AUTO` cold, `SET SP2` mid-dive, `WIFI ON` mid-dive, `CAL AIR` mid-dive, overlong, junk), `GET T`/`GET D` proven to be real replies by cadence anchoring, a console watcher that fails the run on any panic, SAFE latch or lost stack headroom, and `stk=`/`u2stk=` headroom at or above 512 bytes. The link paid for itself immediately: the first `GET T` over it panicked the controller (uart2 stack overflow — 16), fixed structurally (`util/numfmt.cpp`, no by-value reply buffers, `stk=`/`u2stk=` in HB) together with the SAFE-mode `ACK` the boot log had been promising but never implemented, now verified on hardware (`ACK` — `OK SAFE RESTART` — clean boot). **Closed since that run:** the `LOG EXPORT` vs `/dive/<n>.csv` row-shape mismatch (16) — one row formatter now, columns append-only, and `web_probe.py` compares the two transports on the wire. Asking the question that check should have asked (what would a broken build do differently?) then turned up two worse dive-log defects behind it, both fixed in `0256df6` and both bench-checked (16). The FIFO identity defect behind them is closed too (16): a 24-bit dive sequence in the header's spare bytes, `seq a..b` in the boot line, the `L,` fourth field with newest-written-first listing, `TIME SET`/`GET TIME` as the surface-set wall clock, and ISO-8601 dive dates appended to the CSV (v3). The matrix now carries the clock accept and the range refusal, and `--dive-persist` asserts WHICH dive survived — newest sequence present after the reset, a real epoch stamped on the closed dive, the boot stating the clock honestly after a power-down. The bench re-run of the original defect (ten 2 s dives, then 8 s, 40 s, 8 s) now leaves the ten most recent dives on the board, 40 s dive included, where the old firmware kept the nine oldest and ate the rest. The over-long-line defect is closed too (16): line assembly moved into `proto::LineReader` where the native suite can see it, an over-long line is dropped WHOLE, and a new framing phase asserts it on the wire — the refusal is the only answer (no phantom command from a line's own tail), an overflow inside the error rate-limit window answers nothing and still executes nothing, and neither over-long `SET SP2=… <junk>` changed the value. **Calibration honesty is the newest C2 leg (2026-11-24).** The refusal matrix now answers `GET CALAIR` → `CALAIR,0,0,0,0,0,0,0,0` (nothing asked: code 0, not a result invented out of a zero), `CAL DEPTH`/`CAL O2`/`CAL O2 CONFIRM` → `ERR not implemented`, and the warm-up refusal of a surface `CAL AIR` → `CALAIR,3,<age>,0,0,0,0,0,<left>`. Two more checks grade the whole handset stream rather than one reply: an unsolicited `E,<t>,7,3` row must have arrived, and no `E,<t>,7,2` (which would claim a calibration that was refused); the console watch demands the counterpart `cal air: REFUSED` there too, and that no `cal air: sampling` follows a refused request. A new opt-in `--cal-air` phase (~2 min) proves the verdict that matters to a diver: it asks inside the warm-up gate, reads `warmup_left_s` back off the protocol and waits exactly that long — the honest refusal schedules the check — then asserts `CALAIR,2,0,12,<spread>,<raw…>,0`, an unprompted `E,<t>,7,2`, and that the console's `cal air: OK raw cell0=… (12 samples)` carries the same numbers as the wire line (`E,129,7,2` / `CALAIR,2,2,12,21,1514,1519,1528,0` / `cal air: OK raw cell0=1514 cell1=1519 cell2=1528 worst spread 21 (12 samples)`). **81 checks 0 failures** with `--cal-air --dive-persist`, 63 without them. That phase was proven by flashing the pre-fix semantics and watching the bench reproduce the 2026 observation byte for byte: `ERR unknown command` from the tail, then `OK SAFETY` and `SP2=0.310` **written** in the muted window; it also taught the payload shape, because junk glued straight onto the digits is rejected by the settings parser for a different reason and the check would have been about the parser, not about framing (that leg was 62 checks when it landed). **The surface log closed the last hardware-free honesty gap in C2 (2026-11-24).** A calibration verdict and a pre-dive result are facts about the machine, not about a dive, so until this batch the only durable copy of "the cells were refused for warm-up" or "the checks failed at the solenoid step" was a console line nobody keeps — and a failed pre-dive check prevents the very dive whose file could have held the row. `/surface.bin` is a flat stream of fixed 24 B records (per-record version byte + CRC16, reader stops at the first bad one, append + flush per record — the dive files' torn-write contract, for the same reason), capacity 256 with oldest-shed-first compaction by staging-then-swap, and its **own** monotonic sequence read back off the file (`max + 1`), deliberately not shared with the dive sequence because that number decides which dive gets evicted. `EVT_PREFAIL = 16` is the twin of 13 and, like it, is **never published on the wire**. `GET /surface.csv` is the only export and the boot banner (`surface: N records, seq a..b`) the only pre-download view. Two new bench legs: `--predive` runs the checks for real and requires the polled `PRE,` verdict and the row out of flash to name the **same failing step**, and `--cal-air` now cross-checks the newest `cal_air` record's value/samples/spread/three raw counts against the `CALAIR,` line the handset was answered with, joining on the console's `surface: record <seq>` number. `--dive-persist` also asks the durability question of this log, which has no other reader: the boot after the deliberate reset must state at least `records at first boot + records written this run`, and a top seq at least as high as the newest one written — on 2026-11-24 that read `the first boot said 15 records and this run wrote 4 (seq 16..19)` against `surface: 19 records, seq 1..19`. **115 checks 0 failures** with all three legs; **118 checks 0 failures** once the depth announce is graded too (`--expect-depth`, 2026-11-24, the day the LPS28DFW was fitted). `hw_accept.py --selftest` (21 lying-log cases, no board: 12 surface exports and 9 depth announces) grades the graders: a truncated file, a missing identity row, a seq range describing a different file, a record whose raw counts disagree with the wire, a failure filed under the pass id, an appended id with no name, a console line naming a different record, and so on. Proven on hardware by mutation, not only on the host: `r.value = 0` in `surfaceCalAir` reddens exactly one check (`the cal_air record carries the same numbers as the wire: value=0 want 2`) while the row's own id/name check stays green, because that row is internally consistent; with `append` opening `"w"` instead of `"a"` — a log that keeps only its newest row — exactly one check reddens (`every surface record written before the reset survived it: boot after reset: 'surface: 1 record, seq 23..23'; the first boot said 19 records and this run wrote 4`), and every other surface check stays green because the surviving record is consistent with itself. **115 checks 0 failures** with `--cal-air --predive --dive-persist`. Bench truth: the sequence aborts at its first failure, so this bench records `pre_dive_fail` at **step 2 (solenoid)** — no pressure rig — and the battery step never runs. **Still open for C2:** the `CAL O2` leg, which needs the bench jar and the un-instantiated `cal::O2Sequencer`; the cell-proof clamp rows (ids 10/11/12), which need a dive running while the port is held past `CELLPROOF_DEPTH`; and no battery-margin or pre-dive PASS is trustworthy yet, because the pack is unplugged (`vbat` about 1.56 V, `batlow=1`) and `[CELLS UNCALIBRATED: run CAL AIR]` stands. **Closed since that sentence was written:** the LPS28DFW is fitted and identified (`depth: LPS28DFW @0x5D ready, 1016.7 hPa` — XSA pad high, so `0x5D` not `0x5C`), `BIT_SENSOR_STALE` clears with `fresh=111` on every HB line, the boot announce is a graded C2 leg (`--expect-depth`) after the `ready, 0.0 hPa` announce defect was found and fixed (16), the squeeze test is done (thumb to 5.57 m / 560 hPa, held, released, back inside a second, with the `sp=` tier following the depth), and the depth scale is right after the 1000× `BAR_PER_M_SEAWATER` unit bug was found on the wire (16) — the same squeeze that proved the sensor measures is what proved the constant was wrong. The BMP085 that used to block the depth leg never ACKed on the shared bus and has been replaced on the bench by the LPS28DFW (`1e1e15a`, auto-detect `de97ab9`), and the cell-proof clamp rows (ids 10/11/12) are now sighted on the wire **and inside the dive** that produced them. **The dive file must hold its verdicts, not only their effects (2026-11-24, 16).** That leg streamed `E,54,10,0` / `E,54,11,0` / `E,66,12,1` and then exported the same dive with **no `E,` row in it** — from either transport — beside an `M,` line claiming `snapshots 0, events 0, gf 0/0, sp 0.000 ×4` next to 13 samples. The published format has had `E,` rows since v1 and the plan columns since v2; the firmware had written neither. `drainEvents` now emits the event row and the event-driven snapshot from the same pop (the verdict first, then the state it was decided in), the `M,` count columns come from `DiveStore` — the object that wrote the rows — and `gf_low`/`gf_high`/`sp1..4` ride in `DiveTotals` from the config in force. No format change, so no client has to re-validate anything: the bytes a handset saw live are now the bytes its dive download answers with. A third defect fell straight out of the new host test — the never-voted max-ppO₂ sentinel was published verbatim as `-32.768` bar. C2 grows nine checks for all of it: **127 checks 0 failures**, `--selftest` up to 34 lying cases (including that pre-fix dive, captured not typed), and the wire/file join carries a guard against being vacuous. **The depth scale is now pinned on both sides of the link (2026-11-24, 16).** The deferred conversion test landed and found a second defect while being written: the two paths that convert pressure to depth read *different densities* (the telemetry depth used the compile-time `WATER_DENSITY_KGL`, the deco engine used the `WATER_DENSITY` setting), so a fresh-water diver had the `T,` depth and setpoint bands 2.5% away from his ceiling and NDL. One formula now lives in `src/util/depthscale.h`, called by `main.cpp` and by all four depth conversions in `zhl16c.cpp`, and `BAR_PER_M_SEAWATER` is deleted; one density serves both. `test/test_depthscale` (6 cases, every expected number a hand-worked literal — round-trips cannot catch a scale error) plus a source scan that also catches a helper call handed a literal density; `hw_accept.py` reconciles the announce's hPa with the first `T,` row's metres and reads the ruler back (`GET WATER_DENSITY` in the matrix). **129 checks 0 failures** on the flashed build, `web_probe.py` 62, native 256, `--selftest` 42 lying cases, and four grader mutations each reddened exactly their own case. **The ruler is now frozen under a diver, and the bench refuses to take the refusal's word for it (2026-11-24, 16).** Having single-sourced the density made `WATER_DENSITY` the one number behind the `T,` metres, the setpoint tiers, the cellproof arm depth and the deco plan, so a legal mid-dive write (0.990…1.100) moves a diver's whole depth scale ~2.5% at an unchanged pressure; it is `FL_DIVE_LOCK` now, a protocol-visible refusal documented in spec §6, reachable through the `K,` line's dive-lock bit so a handset and the page grey the control out without knowing the key's name. `hw_accept.py` pairs every command with its own reply (`run_protocol(..., pairs=`) and grades four things: the refusal's words, **the ruler not moving DURING the dive**, the box still holding 1.025 afterwards, and the key being writable at the surface (never writable is broken, not locked), with `grade_density_lock` and 10 lying-capture selftest cases. Flashing `FL_NONE` reddened three C2 checks, and the mid-dive read was the only witness — the post-dive read stayed green because the matrix's own surface restore overwrites a leaked value. Probing that lock is what exposed a second defect: the first version asked for `0.990`, the *declared minimum*, and a build without the flag answered `ERR out of range` — an `ERR` of the right shape for an unrelated reason. `setValue` parsed with `strtod` and compared against a `float` bound, which promotes the bound by up to one ULP, and the published decimal bounds are not exact binary numbers: **13 of the 30 numeric keys had an unreachable endpoint** (mins: SP1…SP4 0.20, `SP_HYST` 0.1, `ALLO_LOW` 0.05, `CAL_O2_FI` 0.85, `CAL_O2_TARGET` 0.80, `CELLPROOF_MAXSP` 0.60, `WATER_DENSITY` 0.990; maxes: `ALLO_CUTOFF` 1.80, `CELLPROOF_SP` 1.40, `CELLPROOF_MIN` 1.30) — `GET SCHEMA` and the page's own `<input min=>` advertising a number the parser refused. Fixed by parsing and comparing in `float`; guarded by `test_every_key_reaches_its_own_declared_bounds`, which walks all 30 numeric keys asserting both endpoints parse, ~1% beyond each is refused, and that it walked at least 25 keys, so the loop cannot pass by matching nothing (restoring `strtod` reddens it: `SP1 refuses its own minimum 0.2: out of range`). Two rules from it: **never probe a refusal with a schema endpoint**, and **a refusal is not proof that nothing changed** — a client must read the value back while the refusing state still holds (now a client rule in spec §6). The same test edit also produced a claim in the code that had to be taken back: the failing `no double-start` assertion was first blamed on stale bytes in a reused `Response` — wrong, because `exec()` writes `text[0]='\0'` too and every reply path terminates at `len` (pinned since `15e08a1`). The real cause: the case's own new `DIVE END` invalidated its own trailing assertion, and the behaviour it covered had no case of its own; it now has `test_re_dive_after_a_closed_dive_is_a_fresh_dive`, which also pins the refusal's reason (`dive active`), never asserted before. **136 checks 0 failures** on the flashed build, native 258, `web_probe.py` 62, `--selftest` 54 lying cases. Then the parser's value cap (see the §16 entry on shortening): the matrix now reads `AP_PASS`, writes a value the parser cannot carry, reads it again, and `grade_value_caps` requires the refusal to be `ERR value too long` — not `ERR line too long`, which would be a different rule — and the passphrase to be byte-identical across the two reads. **141 checks 0 failures**, native 262, `web_probe.py` 62, `--selftest` 62 lying cases, `u2stk=` still bottoming at 3088 exactly as before the wider field. The rounding family (16, below) then landed with **no new leg at all** — measured as unable to witness anything on this bench — and the gate is still **141 checks 0 failures** with native at 269 and `u2stk=`/`stk=` unchanged at 3088/1040. **Which chip produced the depth is now a graded C2 leg (2026-11-25, 16).** The matrix row asks only the shape (`^DS,\d,\d,\d,\d+,`); `grade_depth_source` grades the meaning against the boot announce — the wire and the console must name the same chip, that name+address pair must be one the identity gates accept, the slot must be owned, `used` must be true on a bench whose sensor is reading, `healthy` must be true, and `age_s` must not contradict a claimed-in-use source (13 lying-capture cases: an address different from the one the console probed, an `LPS22HB @0x5C` handed the slot, locked with no name, depth HELD, `age_s=900` beside `used=1`, a truncated line, `ERR unknown key` from older firmware — which means *cannot say*, not *no sensor* — and the leg never running at all). **149 checks 0 failures** on the flashed DS build, native 273, `--selftest` 75; `u2stk=` bottomed at 3096 (the snapshot and reply cost 8 bytes), `stk=` unchanged at 1040. **The radio went policy-driven, and the bench learned to ask it why (2026-11-26, 16).** The open M5 clause (WiFi auto-off) closed as the full machine: radio = diver intent AND not held by DIVE (existing) / DEPTH (past `WIFI_OFF_DEPTH`, back only after `WIFI_ON_HOLD_S` straight below `WIFI_ON_DEPTH`) / IDLE (`WIFI_IDLE_MIN` minutes with no associated station, **0 = never**) — four new settings keys, table 32→36, and an inverted ON/OFF pair **oscillates by design** because silent correction is a settings lie. The policy lives in `CommandRouter`, ticks on loopTask under the session lock, and `wifiRadioWanted()` — not `wifiEnabled()` — is what the transport obeys; a dive ended at depth restores the **intent** while DEPTH keeps the antenna off the loop. On the wire it answers `GET WIFI` → `WIFI,<on>,<held_by>,<idle_left_s>,<stations>` (append-only reason enum; `stations` is the policy's **input**, published for the same reason `used` is: the first idle run failed with the countdown pinned at exactly the window and nothing firing — the bench PC's own `wlan0` had autojoined ESP-CCR from an earlier web_probe, and the rule was correctly refusing to fire *and unable to say so*). The matrix now demands mid-dive read exactly `WIFI,0,2,0,0` (the refusal `ERR wifi locked in dive` says a request was denied; this line says the state matches the denial), `grade_wifi` cross-grades the fields against each other and against the same session's `GET *` ruler (10 lying captures), and the console watcher requires the policy's own `wifi: radio down/up (cause)` pair across the matrix dive — which is why `web: radio off` lost its `(dive)` suffix: the reason is no longer always the dive. Two opt-in legs: `--wifi-idle [--wifi-idle-wlan wlan0]` arms the window with the intent event, sets one minute, and requires the countdown seen shrinking, still up at 50 s, fired at 60–130 s blamed on **4**, console agreeing, `WIFI ON` restoring, setting restored; the wlan lever `nmcli`-disconnects the bench PC's autojoin before arming and rejoins it after (name and passphrase read from the machine, not assumed), and a seen station FAILS the leg by attribution instead of cascading four timer-shaped failures. `--wifi-depth` is scripted around the operator's thumb: reason **3** past the off depth, **still down at shallow+20 s** (the check a flap-prone build cannot pass), back no sooner than the straight-shallow hold allows; an unmet cue fails the leg, it does not skip. **170 checks 0 failures** (149 + two matrix rows + the policy console pair + 7 `grade_wifi` + 11 idle-leg), the idle fire witnessed on hardware at 66 s from the intent event with the console matching both transitions; the depth leg was **bench-witnessed the same day, 8/8** (squeeze → reason 3 + console, still down at shallow+20 s, back at exactly 60 s + console clear) via the module's own leg functions through a standalone driver — the in-matrix `--wifi-depth` run still not yet driven, and the runs before the green one produced a harness fix (`_ask_depth`: the leg polled depth through `_ask`, which drops the `T,` stream) and one open recorded observation; the §16 radio entry carries all of it. Two harness defects fell out first, both old families: the matrix's first-match window provably truncated `GET *` once the table hit 36 keys (`WIFI_*` sit at the END — the ruler grader was reading an amputated line list; `GET *` now has a `collect` window), and the idle leg timed its minute from the `SET` while the firmware restarts the window at the **intent event** seven seconds earlier — an honest board reddened for seven seconds of harness offset; anchor where the machine rewinds. `u2stk=3088`, `stk=1040`. |
| C2w | the surface web stack, verified over a joined station instead of "join a phone and look": `tools/web_probe.py` → page served byte-identical to `data/index.html`, captive DNS answers any name, WebSocket handshake verified against RFC 6455, every `#<n>` marker count matches its answer, each command sent over `/ws` and UART2 **back to back** and compared, and `--dive-cycle` proving `DIVE START` takes the radio out from under an associated client and the surface comes back after `DIVE END` on the other transport | done `1d8f284` (tag `m4-web-verified`), extended `0256df6` — 38 checks, 0 failures; the 38th compares `LOG EXPORT n` with `GET /dive/<n>.csv` row for row; it found `board_build.filesystem` defaulting to SPIFFS while the firmware mounts LittleFS, so the page in `data/` had never actually been installed (16); extended again by the dive-log clock work — 41 checks now, the page itself must post `TIME SET` on socket open and show `GET TIME` (the page IS the surface clock source), and the exported dive's identity row must be the v3 shape with an ISO-8601 start. The over-long-line case was upgraded from "the refusal words match" to **byte-for-byte equality** once `proto::LineReader` made the two transports genuinely agree (16). Now 52 checks with `--dive-cycle`: the calibration refusals and `GET CALAIR` are compared across the two transports too, and the served page itself is graded for honesty — it may not offer a button for a command the controller refuses (the `CAL O2`/`CAL DEPTH` buttons are gone; `CAL AIR` plus `CAL AIR status` remain, and the checks view renders the verdict in words from `CALAIR,`). Now **62 checks with `--dive-cycle`**: `/surface.csv` must answer 200 with its `SURFACE,` identity row as the first line, the export is graded against itself (identity count vs rows delivered, seq range spanning them, 13 columns per row, each known id carrying its own name, an unknown id arriving as `id<n>` rather than being dropped, epoch 0 rendering `clock-unset`), and the page must be able to read that log and render a record as words — `shed by capacity` and `pre_dive_fail` both present, a new "Surface" tab being the only reader it has. Thirteenth cross-transport comparison (2026-11-25): `GET DS`, compared byte-identical with one field excepted and the exception stated — `age_s` is seconds since the sensor's last good read and the two sends are ~0.4 s apart; name, address and flags must match exactly. Then the dive-identity grader's own defect was closed (16): it graded `max(slot)` instead of the newest dive and demanded a date from a dive that honestly had none; now it grades epoch↔ISO agreement on the newest dive by `seq`, `--dive-cycle` posts `TIME SET` like the page does and holds the dive it creates to that epoch (0..5 s, which is what would catch close-stamping), and `--selftest` is new — 13 lying rows and FIFO selections, seven mutations seven reds including an accusation printed beside a PASS. **69 checks 0 failures.** Fourteenth cross-transport comparison (2026-11-26): `GET WIFI`, the radio policy's state, compared with **two fields excepted and both exceptions named** — the idle countdown and the station count can each move between two sends 0.4 s apart (a phone does not schedule its joins around the probe), while `on` and `held_by` must be byte-identical: one router answers both transports, so two different answers about *who holds the radio* is exactly the drift this row exists to catch. And the scrubber is itself under test now — lying pairs (an `on` flip, a reason flip, a non-numeric countdown) prove the exception cannot quietly become a loophole. **71 checks 0 failures**, `--selftest` 20 cases. Page radio row (2026-11-26): four new page checks — the page must ask `GET WIFI`, must read its fields into words, must show the idle countdown only while `stations==0` (the router rewinds that window every tick a station is associated — a countdown shown to a connected client is a timer that can never fire), and must render the hold reason as a word *and use it* (a word list never rendered is as mute as a renderer without words); the graders became host-gradable and lie at themselves — four lies, four reds, the first battery catching a decoration the batch itself had just written. **75 checks 0 failures**, `--selftest` **28**. Same pattern extended to the dive view: it used to parse `S,`+`M,` and drop every `E,` verdict row, so the surface reader saw a chart of a failed-proof dive and nothing about the failure; it now lists the rows in file order, worded by the shared `eventName` map, with the empty case rendered as a fact. Three more page checks, three more lies, three reds — grepped against the DIVE-side parse specifically, because the live stream's own `E,` handler would have kept any lazy version of this check green through all the years the dive view was dropping rows. **78 checks 0 failures**, `--selftest` **37** |
| C3 | MS5837 swap + potting checklist, pre-dive on real cells — checklists in `BENCH.md` | pending — hardware |
| — | per-cell histogram: `ctrl::cellhist` accumulator + `GET HIST` + Cells tab | done `1b8c807`, page tabs working since `70049fa` |
| M4a | **ZHL-16C published cross-validation**: 25 schedule entries + NDL transitions from the published Buhlmann air tables (Deco_Package PDF) and GF100/80 cross-impl anchors from repied/paliers `src/gf.ts` run as a node oracle; fixed the M-value form (was `a + b·P`, deco inflated 2–3× — Subsurface/paliers both use `a + P/b`), the NDL definition (direct-ascent feasibility), the NOAA CNS rows (0.8→450, 0.7→570), and one inverted test assertion. Decision logged in §6: per-tissue GF interpolation kept | done `73e004d` (tag `m4-deco-validated`) |
| M5a | **handset client package** (so a handset can be built by someone else, against a stated contract rather than by reading firmware): `docs/HANDSET_PROTOCOL.md` v1 — framing, cadence, every command/reply/refusal string, every downlink line format, the append-only evolution rules and the honest limits (§8: `SET` value truncated to 23 chars, `LOG EXPORT n` slot mapping, 32-deep `E,` ring); `handset/handset_link.h` — header-only C++17 client, transport-agnostic (UART2 and `/ws` carry the same lines), zero-allocation callback handlers, link-health watchdog (3 s silence = alarm); `handset/example_esp32_handset/` — reference handset firmware (wiring, buttons, buzzer classes, link-loss alarm); `handset/README.md`; `test/test_handset/` — 10 conformance cases feeding the library the firmware's own wire literals | done `acfada8` — **183/183 native**, both esp32 builds clean. The protocol contract is now verified against real firmware on real wires: the C2 matrix runs over the bench bridge on 16/17 (48 checks), and `web_probe.py` proves the same lines over `/ws`. Never yet flashed: the example handset board itself. **Caught up 2026-11-24** with the clock batch (`7f98a2c`), which had reached the spec and the firmware but not the package: `timeSet`/`getTime`/`onClock` + `Clock`, `LogEntry::seq` (the fourth `L,` field; a legacy three-field line still parses, `seq 0` = unknown order), `DiveHeader::rowFormat`/`iso` (`clock-unset` passes through verbatim), `timeSet`'s epoch window enforced client-side too — mirroring `wallclock::sane`, refusing **without sending** — a README section on being the time source, and the example firmware now carries the clock state on its main line (`** NO CLOCK: dives undated **`) and can set it from its own console. 14 conformance cases, fed literals **captured off the bench link** (`TIME,1795518150,1,509`, `L,7,1795517641,27`, `L,6,0,26` — an undated dive, `DIVE,10,1795516836,3,3,2026-11-24T10:40:36Z`) rather than literals typed from the spec's grammar; four library mutations (drop the `TIME,` dispatch, drop `L,`'s fourth field, drop the dated `DIVE,` fields, drop the epoch gate) turned five cases red, each with the matching reason. The bench bridge runs this same library as a live observer: reflashed with it, the C2 short gate stayed green (53 checks, 0 failures). **Level again 2026-11-24** with the calibration batch, in one commit as 16 now requires: `CalAir` + `parseCalAir` + `onCalAir` + `getCalAir()` + `calAirText()` (words for a code, because a diver reads words), the `CALAIR,` dispatch, and the refusal documented on the builders that stay in the grammar; the example handset shows `** CELLS NOT CALIBRATED: <why> **` on its main line and re-asks until the calibration is actually done; the tap prints the verdict. 16 conformance cases, the new ones fed literals captured off the bench (`CALAIR,0,0,0,0,0,0,0,0`, `CALAIR,3,1,0,0,0,0,0,97`, `CALAIR,1,0,3,0,0,0,0,0`, `CALAIR,2,2,12,21,1514,1519,1528,0`, `E,18,7,3`, `E,129,7,2`), including the append-only cases (a short `CALAIR,2` still parses; an unknown code 9 is "a verdict with no name", never a parse failure). Three library mutations — no `CALAIR,` dispatch, all nine fields mandatory, `warmup_left_s` ignored — each turned the matching case red |
| M5b | **handset link on the bench without an adapter**: `handset/bridge_esp32` — a spare classic ESP32 as the USB-UART (its own bridge chip for the PC, UART2 16/17 for the controller), a byte-for-byte pipe that also runs the client library as a passive observer (link/traffic LEDs = the library's verdict, not a guess), plus `-DBRIDGE_TAP=1` for a handset-side console. Bench sketches are now their own mini-projects (`pio run -d handset/<name>`), so a handset build can never produce a controller image | done `233ad2c`, **and the bench link is live**: the bridge is flashed and the C2 refusal matrix has been run over it many times (48 checks, 0 failures), with the client library on the bridge side watching the same stream. Two facts the wiring taught, both now in 16 and `BENCH.md`: opening the bridge's serial port resets it and its boot banner lands in the controller's line buffer, where un-terminated bytes are glued onto the next command, so `hw_accept.py` and `web_probe.py` send a newline first and discard what it draws; and both CP2102s report the same `/dev/serial/by-id`, so an upload must name
`--upload-port` — a rule that sat in prose and not in the copy-paste runbook,
and got stepped over on 2026-11-26 with the confusion C2-g in `BENCH.md`
records; the port naming is now part of the C2 command list itself.
`tools/port_id.py` (`--loopback` jumper test, boot-time UART2 self-test in
the tap build) stays as the bisect path if the link ever goes silent again |

## 16. Design watch-outs (recorded decisions)

- BMP085 is a **bench stand-in only** — MS5837-30bar before any wet test; the
  driver ships in M3 so there is no last-minute code change before wet testing.
- **The high wall sees every fitted cell, not the vote** (`inj::Input::highVetoBar`,
  same veto in `alarms::Inputs`). Found by the simulator: two cells stuck at
  0.55/0.60 keep demanding O₂ while the honest third screams 1.5 — and the drift
  rule may exclude *that* cell as the odd one out. A cell excluded from the vote
  for reading low never loses its veto on the high side. `test_sim` scenario
  `test_two_lying_cells_veto` is the regression test.
- **Stuck-open window needs hysteresis** (`inj::Config::stuckHystBar`): a few mV
  of ppO₂ dither used to re-arm the 5 s evaluation window every sample, so a
  valve that was wide open never accumulated evidence (the simulator caught it;
  the bench never would have).
- **Depth feeding the bands is median-of-5 then boxcar-of-4**
  (`control/depthfilter.h`), shared by firmware and sim: one spiked pressure
  sample must not move the setpoint by 0.3 bar, and a bare EMA asymptotes toward
  a band threshold without ever crossing it (CellProof then never arms).
- **Optional struct fields go last** in `inj::Input` / `alarms::Inputs`. These
  structs are aggregate-initialised positionally in tests and firmware; adding
  `highVetoBar` in the middle silently re-mapped `nowMs` into it and broke three
  injector tests. New fields append, always with a safe default.
- **Stale sensing stops injection** (`ii.sensorHealthy = haveVote_ && cellsFresh`).
  A vote built from frozen ADC samples looks healthy and keeps the loop
  comfortably wrong; the valve stays shut and `BIT_SENSOR_STALE` says why.
  `test_loop/test_stale_sensors_stop_injection` is the regression test.
- **Output fuse on the valve pin** (`inj::Config::maxOnMs`, default 900 ms): the
  driver layer counts how long the pin has been HIGH and forces it low past the
  limit, latching as a stuck-open valve — from the diver's side the consequence
  is identical, so the response must be too. One clear path (`clearStuckOpen`).
  This is the belt for "the pin is HIGH because something broke", not because a
  pulse was asked for.
- **Test-pulse requests are queued, not latched** (`Injector::requestTestPulse(n)`
  counts). The pulse test asks for one pulse per cycle for 3 cycles; with a
  boolean flag, gap/duty rules swallowed two of the requests and the verdict
  only ever came from the test's 20 s timeout — a pre-dive check silently
  degraded to judging one pulse.
- **The duty cap must fit the pre-dive cell test**: the injector will not START a
  pulse whose own width crosses the rolling-window cap, so a cap below
  `pulses × pulseMs / window` starves the cell test. `defaults.h` now
  `static_assert`s that relationship (10 % of a 10 s window = 3 × 300 ms).
- **Names that only break on the ESP32 target**: namespace `alarms` (newlib
  declares POSIX `alarm(int)`), and `cellproof::State::INACTIVE` instead of
  `DISABLED` (the Arduino core `#define DISABLED 0x00`, which expands inside an
  enumerator list even in a scoped enum). Host tests never see either — so the
  `esp32` build is run alongside `pio test -e native`, never instead of it.
- **ControlTask ownership of the valve**: SAFE boot never creates the task, so
  `PIN_SOLENOID` is never taken over from `INPUT_PULLDOWN`. The task's cycle is
  the supervisor heartbeat, with the tightest stall budget (1.5 s vs 3 s) — a
  controller that stops deciding stops protecting the loop.
- **Status-line fields append, never reorder**: `soak_logger.py:HB_RE` parses the
  leading fields; `vote= sp= al= mode= pulses= sol= ctrl=` were appended. The
  soak harness and the firmware must stay in step or a 24 h run silently loses
  its telemetry columns.
- **Alarm threshold sits BELOW the injection wall** (`alarms::Config::highBar`
  1.40 < `inj::highWallBar` 1.45 < `inj::cutoffBar` 1.60). With the alarm at the
  wall value the wall silently caps enrichment: the loop parks at 1.45 and the
  diver is never told why the setpoint is not being met. Found by the 50-dive
  soak (dive 26/42); moving the alarm back to 1.60 makes that suite fail —
  re-verified 2026-11-26 on the bench-matched sim noise (`d0b60cf`), where the
  failure now reads with numbers: `dive 26: loop at 1.59 bar for 3.1 s, a cell
  read 1.59, and the high alarm stayed off`, dive 42 the same at 1.59/1.60 —
  and the mutation also reddens dives 4/14's low-drift detection asserts, a
  wider blast than this entry had been claiming.
- **Dive-file records are fixed-stride and each carries its own CRC16**. The v1
  codec wrote **34 bytes into a 32-byte record** — appended back to back, every
  snapshot silently corrupted its neighbour. `SNAPSHOT_SIZE` is 36 (34 B of
  fields + CRC) and `EVENT_SIZE` 10; `test_logfmt` guards the stride with
  canary bytes. The alarm set is 10 bits wide and the record has no spare byte,
  so bits 8..9 (CNS_HIGH, HYPOXIA) ride in bits 3..4 of `flags`
  (`kFLAGS_ALARM_HI = 0x18`), cleared back out on unpack. The v3 stream (the
  one on disk today, `DIVE_VERSION = 3`) prefixes every record with a type tag
  (`REC_SNAPSHOT 'S'` / `REC_EVENT 'E'` / `REC_META 'M'`) and puts a fixed
  16 B CRC'd header at the front of the file, so a reader can walk the file
  without knowing write order. `logfmt::scanRecords()` is the one pure walk
  (byte image in, records out — host-tested by feeding it images directly).
  The LittleFS reader must **stop at the first tag unknown or record whose CRC
  fails** — a reset mid-write may truncate a dive, it may not turn it into
  plausible garbage.
- **A cell proof is only worth what it changes.** Four separate gaps, all found
  by the soak (dive 4: vote 0.82 bar, real loop 0.12 bar at 27 m — the loop was
  being *starved*, not over-enriched. The numbers are as-found at `bda1825`,
  under that era's sim noise; today's dive 4 never reaches that floor — its
  witness reads `ppO2 0.24-1.48` — because the four gaps are closed. The
  record stands for the discovery, not for current bytes):
  * the verdict is **per cell**, and every *fitted* cell is judged including one
    the voting layer dropped (requiring the vote to reach the proof level first
    lets a lying pair hide the honest cell);
  * a cell proves itself by **tracking** the demand change, not by sitting at a
    plausible number (`trackMinBar` 0.05 — ten times the sim's cell noise when
    written, over fifty times since the dither was corrected to bench-matched
    ±3 counts on 2026-11-26) — a cell
    frozen at 1.20 bar would otherwise "prove" itself in a loop at 1.20;
  * a PASS **re-arms** when a rejected cell is persistently off the vote in
    either direction (gap 0.10 = 2× drift bar, cooldown 300 s, 3 attempts per
    dive). A proof that only runs on first descent cannot see a cell that was
    honest at 10 m and failed at 40 m. A FAIL does not self-re-arm: the clamp is
    the safe state and lifting it is the diver's `OVERRIDE`;
  * and the verdict **must bind the vote** (`ctrl::Loop` step 5b →
    `voting::Engine::setProofExclusion`) while the setpoint is above `okMaxSpBar`
    or the fitted cells still disagree. Un-bound, the proof is a display note:
    the lying pair goes on outvoting the honest cell, a vote above setpoint
    means the valve never opens, and consumption empties the loop.
- **A voting ban must never leave the machine with no vote.** If the CellProof
  exclusion empties the trusted set, a cell excluded on drift alone votes
  instead (tier 2) and `disagreement` stays set — no vote means no injection, so
  it has to be the last resort, not the first. Related: drift strikes are a
  **threshold, capped at `driftWindow`**, not an accumulator, so the documented
  hysteresis (back after `driftWindow/2` clean samples) stays true however long
  a cell was out.
- **Simulator authoring traps** (each one made the suite meaningless rather than
  failing it): a diver model that descends to chase a rising ceiling loads more
  tissue and deepens it — the ceiling is an *ascent* floor only (one soak diver
  ran itself to 208 m); a depth clamp written as an unconditional
  `if (depth < target) depth = target` after the rate step **teleports** the
  diver and makes every dive instantaneous; a fault generator must never leave a
  3-cell rig with no honest channel (asserting detection there is asserting the
  impossible — that case belongs in `test_sim`, with the third cell left honest
  on purpose); and a cell "jump" must stay under the plausibility gate
  (0.5 bar/sample), because the gate is right to reject bigger ones.
- **One command session, not one per transport.** A per-transport router looks
  fine in isolation and then the phone turns WiFi off, the handset's private
  router never heard it, and at dive end the machine turns the radio back on —
  undoing what the diver asked for on the client in front of him. Same failure
  class for the two-step `CAL O2` arm (armed on the web, refused "not armed" on
  the handset). `comms::Session` owns the router, link, lock and dive-file
  owner; the UART2 task and the `/ws` bridge are byte movers only.
  `test/test_session/` pins the invariants: identical refusals, diver intent
  across transports, one shared arm, actions inside the session lock, and no
  web command draining the handset's event ring.
- **A sink callback gets the caller's context, never the firmware's hook
  object.** `UartLink::streamLogExport` passed `hw_.ctx` into the transport's
  sink — the first `LOG EXPORT` over WiFi would have handed a socket id cast to
  the firmware's hook struct. The export must stream through the *asker's*
  sink with the *asker's* ctx; `test_uartlink`'s export case records the ctx
  and was verified by mutation (reverting the fix fails it).
- **Web answer framing is wire-visible protocol, not UI detail.** The protocol
  is line-based and one command may answer with many lines, so the bridge ends
  every answer with `#<n>` (n = reply lines, `#0 dropped` on overflow) and
  splits multi-line replies into one frame per line — unsplit, the browser's
  parser sees one giant line and the page half-works silently. `#` can never
  begin a protocol line, which is what makes the marker collision-free.
- **Radio policy lives in exactly one function, and sockets are never closed
  from the command path.** A client that sends `DIVE START` must not have its
  own socket destroyed inside its own callback; the radio drop ends the
  connections and `start()` runs `cleanupClients()` afterwards. Same reasoning
  for `async_tcp`: pinned to core 0 (with the WiFi driver, away from control 22
  / handset 12 / sensor 24 on core 1) and deliberately **not** TWDT-registered
  — a wedged web server must never reset a diving controller.
- **The page lives in firmware flash; the LittleFS copy is generated, not
  edited.** `src/comms/web_page.h` is the single source; `tools/sync_webdata.py`
  writes `data/index.html` from it, and the server falls back to the embedded
  copy when the filesystem is missing, half-written or wiped. Hand-editing the
  data copy guarantees the two versions only drift on the one board that has a
  filesystem.
- **ID-stamped ring slots must value-initialize.** `cellhist` reuses ring
  slots only when the stamped window id matches — but the first `Hist` built
  over a dead one's stack memory inherited matching ids and counts from the
  previous test (a fresh boot would inherit RAM garbage): the histogram's
  first "shape" is a ghost that *renders plausibly*. The suite caught it as
  "expected 100, was 104"; the slots are now `= {}` load-bearingly. Any
  future id/stamp ring takes the same initializer discipline.
- **The dive log has one owner but two writers.** ControlTask appends the 30 s
  grid rows, the handset link appends event-driven rows; both go through
  `g_logMtx` in `session_wiring.cpp`, and only the command path opens/closes
  files. Two tasks appending to one LittleFS file without that mutex produces
  interleaved records that each still pass their own CRC.
- **Display keys resolve after the registry** in `CommandRouter::exec` —
  `GET SCHEMA`/`GET T`/… are asked after the real keys, so a display key can
  never shadow a real setting (`D1`, `CAL_O2_FI`).
- **The chain is mutated on one thread only — the tick boundary.** The
  session lock keeps commands off each other; it never kept them off
  `ControlTask`, which cycles on its own priority and can preempt the handset
  task mid-`setConfig`, or run truly parallel to the web task on core 0.
  `ctrl::Loop` now splits every command into a **gate** and the **chain work**:
  gates (`diveActive_`, `autoMode_`, `cellTest_`, atomics) flip synchronously,
  because dive-locked refusals, telemetry mode and "one test at a time" must
  see the diver's intent on the very next read; the heavier work (config
  re-tune, dive-state reset, sequencer start, ack latch, pulses) is queued in
  a tiny mutex-guarded mailbox that `cycle()` drains as step 0 — so the queued
  config already decides the same tick it lands in, and worst-case latency
  for a handset command is one 100 ms tick. That bound is a lifeline property,
  not a nicety: **the handset is the in-water transport; the web is off during
  a dive.** `test/test_deferred/` pins it, and a ThreadSanitizer run of the
  two-threads stress shape is clean.
- **Decisions read published verdicts, never another core's mid-update
  state.** TSAN caught `requestAuto` computing `predive_.allPassed()` from the
  command thread while `cycle()` rewrote the sequencer: the AUTO grant is a
  decision, so the sequencer verdict is now *published* per tick
  (`predivePassed_`). Display reads (`GET PRE`, `GET PROOF`) stay point-in-time
  single-byte samples — no decision hangs on them, and anything that acts is
  re-guarded inside the engine at apply time (a `CELLPROOF OVERRIDE` posted
  against a stale FAIL is a no-op, not a bypass). A second-start guard
  (`postStartDive` refuses while a dive is active, for the chain itself, not
  just the router) keeps a stray command from zeroing tissues mid-dive — the
  test watches CNS to prove it.
- **`PULSE` never pulsed.** The cycle's "withdraw the stopped test's leftover
  pulses" rule was unconditional, so it also ate the handset's standalone
  `PULSE` queued between cycles — the command replied `OK` and the valve never
  moved (pre-A5e it worked only when the write raced into the gap between
  steps 3 and 6 of a cycle). The cancel is now edge-triggered on the pulse
  test *leaving* PULSING. `test_deferred`'s pulse case is the regression:
  post one pulse, watch exactly one leave the valve.
- Solenoid driver circuit must fail **closed** on a floating/pulled-low gate;
  keep firmware max duty low; recommend a hardware manual-inhibit switch in the
  O₂ line that needs zero firmware cooperation.
- WiFi credentials fixed, not editable mid-dive.
- Simulator-validated deco ≠ dive-release certification; cross-check against
  independent implementations before anyone wears it.
- Bench cells share a common ground referenced to ADS1115 GND; handset powered
  from the same rail (UART common ground).
- **Captive DNS on the AP is load-bearing.** A phone joined to the hotspot but
  able to resolve nothing reports "the page does not load" even though the
  server is perfect; DNSServer now answers every A query with the AP IP, and
  it is pumped from hudTask — not the handset task, which may never be created
  when UART2 is unwired. Library quirk (verified on the wire): the wildcard
  domain is `"*"`; an empty domain answers with a malformed empty packet.
- **One WebSocket frame per reply line is a burst the radio punishes.** A
  32-line SCHEMA reply became 33 tiny TCP segments; AsyncTCP's output stalled
  for >13 s behind them and the page correctly declared NO DATA. Web replies
  are batched into ≤1 KB frames (marker flushes the tail); the protocol and
  the `#<n>` completion marker are unchanged.
- **AsyncTCP's default 5 s ACK watchdog closes live dashboard sockets** on
  any congested 2.4 GHz link (bench laptop NICs scan channels; a 5 s stall
  is routine, `AsyncWebSocket::_onTimeout -> close()` made every connection
  die at 30–70 s). `CONFIG_ASYNC_TCP_MAX_ACK_TIME=30000` in the esp32 build
  flags: sheds genuinely dead clients, keeps the page alive through stalls —
  180 s / 178 replies / 0 drops after, vs. a drop every ~40 s before.
- Opening the console serial port RESETS the board (pyserial asserts DTR/RTS
  on open; DTR=IO0, RTS=EN on the classic auto-reset circuit). Several "the
  board reboots by itself" observations during web debugging were the laptop
  opening `/dev/ttyUSB0`, not the firmware.
- **Settings keys are NVS keys: hard 15-character limit**, enforced by the
  ESP32 NVS layer per-write — a longer key fails every save with
  `ERR settings not saved`. `test_settings` pins the limit so the build
  fails before the bench has to find it again. Renaming a key is legal only
  with a `kSchema` bump (old NVS erased wholesale), and keys are
  protocol-visible to the handset, so rename early or not at all.
- **BMP085 out, LPS28DFW in for the bench phase (2026-11-23).** The BMP085
  never ACKed once since the first boot that polled it; keeping a phantom
  part in the tree buys nothing. The LPS28DFW (Adafruit #6067) is probed at
  both possible addresses (XSA pad) and accepted only with its own
  WHO_AM_I `0xB4` — a sensor that isn't identified is not believed. Always
  run it in FS_MODE=1 (4060 hPa): mode 0 flat-tops at ~2.4 m of sea water,
  silently, which is inside the 10 m cell-proof envelope — the exact failure
  mode a diver must never meet as a "shallow" reading. MS5837 stays the wet
  prototype's sensor; nothing about the dive-rated part changes.
- **Depth selection: boot auto-detect, not build constant, not a setting
  (2026-11-23).** The bench→wet-LPS28→MS5837 march means three physical
  sensor changes; a build constant makes each one a reflash (which drops
  the dive link and the phone), and a settings key would configure what the
  hardware can simply tell us. The arbiter awards depth to whichever chip
  passes its OWN identity gate first — MS5837 (PROM CRC-4) preferred, LPS28
  (WHO_AM_I 0xB4) fallback — and **locks the choice until power cycle**:
  auto-detect is safe exactly because nothing can switch the depth source
  while a diver is breathing from the numbers. For M-phase sign-off dives
  the flashed artifact is still pinned by tag; the boot line records which
  sensor dove.
- **Depth gets its own I2C port (Wire1, 18/19) with its own bit-banged
  recovery.** The coupling on one bus cut both ways: ADC bus recovery
  interrupted in-flight pressure reads, and a wet depth harness fault could
  wedge the bus feeding ppO2 — the failure that must NEVER be downstream of
  another failure. 18/19 are Wire1's factory pair, free on WROOM/WROVER
  classic (reserved only on 8 MB-flash D*WDH8 — check the module stamp).
  Recovery is now one shared mechanism (`i2c_recovery.h`) used by both
  buses; the transport triggers it after a streak of failed transactions
  (the latched-SDA signature) with a minimum gap so an absent sensor never
  becomes a recovery storm.
- **The depth boot line announced a lock, and so always reported 0.0 hPa
  (found on the bench 2026-11-24, the day the LPS28DFW was finally fitted).**
  The line exists to be evidence — *which* sensor owns depth, at which address,
  reading what — and it is what `BENCH.md` C3-a is checked against. But it fired
  on `g_depth.healthy()`, and `begin()` reports READY the instant a chip is
  IDENTIFIED (WHO_AM_I `0xB4`, or the MS5837's PROM CRC): the first pressure
  conversion comes later, paced by the driver's own sample window. Every healthy
  boot therefore printed `depth: LPS28DFW @0x5D ready, 0.0 hPa`, healthy sensor or
  not. A number that is always zero is worse than no number, because it reads
  like a measurement — and this one sat in the place a diver or a bench operator
  is told to look at. Fixed by announcing on **the first sample the control path
  would use** rather than on the lock: `main.cpp`'s existing `p > 0.2f` gate is
  passed across as `saneSample`, so there is one rule about what counts as a
  reading and the two cannot drift. A source that identifies and then never
  produces one now says so (`no sane pressure within 3 s (reads … hPa) - depth
  will not be used`, grace measured from the lock, not from boot) instead of
  leaving silence, because "nothing fitted" and "fitted but deaf" are different
  facts and silence reads like the first one — the same split the surface log
  makes between `no records yet` and `** will not read`. The state machine is
  separate and pure (`press::Announcer`, no Wire, no Serial): five host cases pin
  the ordering, the exact bytes and the refusal to emit a truncated line. Four
  mutations redden them — announce at the lock (the original defect), silence
  instead of the no-data line, grace measured from boot, truncation allowed.
  `hw_accept.py --expect-depth` now grades the line on hardware: an identity and
  address the firmware's gates would accept, an hPa inside 250..1200 (loose on
  purpose — the check is placeholder-vs-reading, not calibration), and the line
  agreeing with HB's `al` word **in both directions**: `ready` with
  `BIT_SENSOR_STALE` set, or `no sane pressure` with the bit clear, is a console
  disagreeing with the machine. Proven by replay of the two real captures — the
  pre-fix log is red on the hPa check, the post-fix log is green — and by five
  grader mutations, each reddening exactly one `--selftest` case.
  **What it did NOT establish, recorded here so the next reader does not repeat
  it:** the first afternoon's `Error 263` (`ESP_ERR_TIMEOUT`) bursts, and the
  cell channels going stale between depth retries, were this bench's wiring
  fault, not a bus-coupling finding. The rewired bench shows one `requestFrom`
  error in 75 s (the MS5837 probe inside the preferred window, expected), 15/15
  HB lines `fresh=111 i2cerr=0 rec=0`, and `al=0x0010` — battery only. With the
  depth bus healthy, `BIT_SENSOR_STALE` clearing is itself the first proof the
  pressure is real, since `ai.sensorStale = !cellsFresh || !pressureValid`. The
  squeeze test — watch `T,` depth while the port is pressed — is still the only
  check that proves the sensor *measures*: at the surface, depth is 0.04 m either
  way (0.04, not 0.00: `SURFACE_PRESSURE_BAR` is a fixed 1013.0 hPa and this bench
  sits at 1016.7). **Done 2026-11-24 and it passed** — thumb press to 5.57 m
  (≈560 hPa) held 25 s, back to baseline inside a second, no undershoot — and the
  same rows proved the control chain is listening: `sp=` in the `T,` line went
  0.400 → 0.700 at 2.67 m and back at 0.47 m, both hysteresis edges of
  `SP_IN_WATER_M=2.0` ± `SP_HYST_M=0.5`, seen by accident and both correct.
- **A unit bug made every depth 1000× too deep, and it could only be found the
  day a depth sensor worked (2026-11-24).** First boot with the LPS28DFW healthy
  and the handset answered `T,37.04,…,STANDBY` — on a bench in an open room — and
  HB's `sp=` had moved from 0.40 to 1.00. `BAR_PER_M_SEAWATER` was
  `WATER_DENSITY_KGL * 9.80665 / 100000` with the density held in **kg/L**, but
  `ρ·g` is Pa/m only for **kg/m³**: the constant came out 1.005e-4 bar/m instead
  of 0.1005, so 3.7 hPa of weather above `SURFACE_PRESSURE_BAR` became 37 m. The
  consequences were already live on the bench, not hypothetical: the setpoint
  engine crossed `SP_IN_WATER_M=2` and `SP_DEEP1_M=30` and chose the **SP3 1.00
  bar** tier at the surface — 2.5× the intended ppO₂, dosing oxygen that no
  dive-rated setpoint asked for; `CELLPROOF_DEPTH` was "crossed" from the first
  second of any dive; and the deco engine was fed 37 m. The constant's own comment
  stated the correct physics ("1 bar of sea water (1.025 kg/l) = 9.947 m") and the
  expression contradicted it, which is the shape of bug a code review walks past.
  **Why no test saw it, and what would:** no host test references
  `BAR_PER_M_SEAWATER` at all — the `p → depth` line sits in `main.cpp`, outside
  every tested unit — and a test that derived its expected depth *from the macro*
  would inherit the error rather than catch it. The test that catches this kind of
  bug pins the physical scale from outside the code (`1 bar ↔ 9.947 m` at
  1.025 kg/l; `Δ100 hPa ↔ 1 m`), which is an open gap, deliberately deferred.
  `src/deco/zhl16c.cpp` had the conversion right all along (`waterDensity * 1000.0
  * 9.80665` Pa/m), so the two depth paths disagreed — a second reason it stayed
  invisible: nothing compared a deco-computed ambient with the depth in the
  telemetry row. Verified on the wire both ways: surface `T,0.04` with `sp=0.400`
  (predicted 0.037 m from the announce's independent 1016.7 hPa reading), and the
  squeeze moving depth ~1 m per 100 hPa of thumb instead of ~1000 m.
  **The general lesson, recorded because it recurs: a constant can be wrong
  forever if its input has never existed.** Depth had been invalid on this bench
  for the project's whole life (`pressureValid` false, `depthM` pinned to 0 in
  every log ever taken), so no downstream consumer had ever had a number to
  disbelieve. Fitting the sensor was the test.
- **`OK` is about the command, not the effect — found while writing the
  handset contract (2026-11-23).** Enumerating every reply for
  `docs/HANDSET_PROTOCOL.md` forced the question "what does the diver actually
  see when this fails?", and three answers were "nothing on the wire":
  `CAL O2` arm/confirm set action bits that **no firmware object consumes**
  (`cal::O2Sequencer` is host-tested and never instantiated — the same
  missing-consumer hole `CAL AIR` had before `cal::AirCal` was wired);
  `CAL DEPTH` raises `g_calDepthRequest`, which **nothing reads** — no
  depth-offset trim exists, so the surface tare in §13 is still a design
  intent, not a behaviour; and `CAL AIR` answers `OK CAL AIR` *before* the
  control task applies the 120 s warm-up gate, so a too-early call says OK and
  reports the refusal only on the USB console. Event ids 7/8/11/12/13/14 are
  the same shape: defined in the log format, never published. **Partly closed
  2026-11-24: ids 11/12 (clamp ON / clamp OFF) are published now** — as edges,
  one row each (`CLAMP_OFF` value 1 = the diver lifted it, because a FAIL never
  re-arms itself), pinned by `test_uartlink` driving the real cell-proof engine
  to a FAIL and then lifting it with the handset's own path, and each half of the
  pair proven to bite by deleting it. Their hardware sighting is queued in
  `BENCH.md` C3-a, because the proof arms on a depth crossing and this bench has
  no depth sensor: host-proven, never yet seen on the wire. **7/8/13/14 remain
  reserved, and the reason is structural, not laziness** — an event row lives
  inside a dive file, and every one of those facts is made at the surface
  (calibration, pre-dive) or at the moment a dive file was left unclosed by a
  reset. Giving them a home is a decision, not a patch. A handset built
  by someone else would have drawn a "calibrated" tick from all of this, so
  the spec states it plainly (§3 caveats, §4 id table, §8) and the library
  refuses to invent the missing feedback.

  **Closed on the firmware side 2026-11-24, in the direction chosen deliberately:
  refuse what does not exist, and report what does.** `CAL O2`, `CAL O2 CONFIRM`
  and `CAL DEPTH` now answer `ERR not implemented`, and the phantom plumbing went
  with them (`o2Armed_`, `ACT_CAL_O2_*`, `ACT_CAL_DEPTH`, `depthCalPending_`,
  `onCalDepth`, `g_calDepthRequest`); the retired action-bit numbers are recorded
  in `cmd_router.h` so the next one added is `1 << 13` and never a recycled value
  — an internal bit meaning something different from what the sender thought is
  the same class of bug the protocol's append-only rule exists to prevent. The
  commands stay in the grammar, so the day `cal::O2Sequencer` is instantiated or a
  depth tare exists, no client has to change. `CAL AIR`, which does have a
  consumer, reports its verdict on the protocol now: `bridge::CalAir` with
  `bridge::publishCalAir`/`publishCalAirRefused`, `GET CALAIR` →
  `CALAIR,<code>,<age_s>,<samples>,<spread>,<raw0>,<raw1>,<raw2>,<warmup_left_s>`,
  and one `E,<t>,7,<code>` row per terminal verdict (never one per sample), with
  `cal::AirCalOutput::whyCode` added so a failure reason reaches the wire as a
  number instead of an English sentence a client would have to string-match. The
  vocabulary is defined **once** and shared by both surfaces — the `CALAIR,`
  status field and the id 7 value are the same numbers — because two numberings
  for one fact is how clients start disagreeing.
  * **What a reply cannot do is carry a fact that happens later.** The warm-up
    refusal is decided in ControlTask about a second after the command, so no
    reply to that command can report it: the fact needs its own row, volunteered
    (`E,`) and pullable (`GET CALAIR`). That is the general rule behind "`OK` is
    about the command", and the shape to ask for afterwards — "who learns this if
    nobody was asking?" before adding a command.
  * **Two records of one event have to be compared, not merely produced.** The
    bench phase asserts that the console's `cal air: OK raw cell0=… (12 samples)`
    carries the same numbers as the `CALAIR,` line. Writing that check found the
    harness discarding its own evidence: its line filter `^[TDE],` treats `E,` as
    telemetry noise, so the `E,<t>,7,2` row it was hunting had been filtered out —
    and a check that can never pass is worse than no check, because it trains you
    to ignore the section. Same batch: the console-watch check "a refused
    calibration did not also start sampling" was scoped to the whole run, so a
    *deliberate* calibration two minutes later failed it; the window it describes
    is now marked where it belongs.
  * **Still open, by decision (2026-11-24):** id 7 is published but not stored
    durably, and 8/13/14 stay reserved. Where the surface facts live is now
    decided — **a separate surface log** (`/surface.bin`: calibration, pre-dive
    and reset records outside the dive files, with its own capacity policy and its
    own list/export semantics), because an event row inside a dive file cannot
    speak for a dive that had not started. `CAL DEPTH`'s real consumer and
    `cal::O2Sequencer` stay M2/M3 exit criteria, tied to the LPS28DFW and the
    bench jar; until then the wire says `not implemented`, which is the honest
    state of the machine. Small related finding, **fixed 2026-11-24**: `session_wiring.cpp` declares
  `extern bool g_calAirRequest` while `main.cpp` defines it `volatile bool` —
  the linker cannot see the disagreement and the flag is cross-task, so the
  qualifier belongs in the declaration (both request flags are now externed
  `volatile`, with a comment saying why the declaration must not drop it).
- **Uplink overflow does not resynchronise to the newline (found writing the
   handset contract, 2026-11-23; measured on the board the same day).**
   `uart2_task.cpp` accepts 64 characters; the 65th answers `ERR line too long`
   (rate-limited to one per 2 s) and resets the buffer — but keeps reading, so
   the remainder of an overlong line is collected and terminated as a *second*
   command. Harmless for a well-behaved handset and for the web page, but "a long
   line is dropped" is not what the code does, so the spec documents what it does.
   **The rate limit makes it worse than framing noise, and this is the part the
   first version of this entry got wrong: in the suppressed case `len` is *not*
   reset, so the terminator dispatches the truncated first 64 bytes as a command.**
   Observed on the bench: over-long `GET SP2 ZZZ...` inside the window answered
   `SP2=0.700` (executed, not refused); an over-long `SET SP2=0.33` plus 120 junk
   bytes answered `OK SAFETY` and `GET SP2` read back `0.330`, while the same bytes
   outside the window were refused and changed nothing (value restored afterwards).
   So the documented refusal is only guaranteed for the first overflow in two
   seconds, and an over-long *write* can land. What the first entry asserted
   alongside this — "not a security boundary: the router refuses what it refuses
   regardless of framing" — was wrong. Framing decides whether the router sees
   the command at all. Hardening when this file is next open: on overflow always
   swallow to the next terminator and rate-limit only the text; and extract the
   line-assembly loop into `uart_proto`, because inline in an ESP-only task the
   native suite cannot see the rule at all. The client library already behaves the
   way the firmware should (`overflowing_` drops the whole line). The lesson for
   every rate limit in this project: gate the message, never the recovery.
   **Fixed 2026-11-24, exactly as prescribed.** `proto::LineReader`
   (`uart_proto.{h,cpp}`) is now the uplink assembler: past `MAX_LINE` it sets a
   `swallowing_` flag and discards every byte to the terminator, so the line
   reaches the router in no form — the 2 s window gates only the error *text*,
   and a swallowed line still costs nothing to recover from. `uart2_task.cpp`
   feeds it bytes and owns no framing state; the web transport already dropped the
   whole frame, so both transports now take the same verdict and `web_probe.py`
   upgraded its over-long case from "the refusal words match" to byte-for-byte
   equality with the handset answer. Proof, in both directions: four `LineReader`
   cases in `test_proto` (an over-long `SET SP2=0.33` + 120 junk bytes yields ZERO
   lines, rate-limited or not) — and re-running them against the old semantics fed
   back a red suite with three failures, one of them "Expected 0 Was 3", i.e. the
   old code dispatched three phantom commands from three overflowing lines. On the
   wire, `hw_accept.py`'s new framing phase asserts the refusal is the ONLY answer
   (no second command from a line's own tail), that an overflow inside the error
   window answers nothing and still executes nothing, and that SP2 is unchanged
   afterwards by either write — and the phase was itself proven by flashing the
   pre-fix semantics and watching the bench reproduce the 2026 observation
   exactly: `ERR line too long` then `ERR unknown command` for one send, then
   silence followed by `OK SAFETY` and `SP2=0.310` in the muted window, i.e. the
   write landed. That mutation run also exposed a weak payload: junk glued
   straight onto the digits (`SET SP2=0.31ZZZ…`) is rejected by the settings
   parser as "not a number", so the "did it write" check passed for the wrong
   reason; the tail has to start with a space, because `settings.cpp` tolerates a
   trailing tail after one, which is exactly the shape that landed in 2026. The
   writing of the phase caught a bug in the new assembler too: it did not reset
   `len_` after dispatching a line, so a bare
   newline re-dispatched the previous command — which is what a check FOR silence
   is worth: the host suite went red on the first run.
- **Two identical CP2102 boards make "which port is which" a safety question
  (bench, 2026-11-23).** The handset bridge and the controller are both
  `esp32dev` images on the same hardware, and esptool will happily write either;
  writing the bridge into the controller destroys a dive controller. Both boards
  also report the *same* `/dev/serial/by-id` string, so by-id cannot
  disambiguate them. Rule: classify first with `tools/port_id.py` (DTR/RTS
  pre-cleared before `open()`, so listening is passive — the pyserial default is
  a reset pulse), then pass `--upload-port` explicitly. Classify by what a port
  hears **at its own boot**: a spare that merely relays the controller's `HB`
  lines is a wiring mistake, not the controller.
- **`firmware.bin` is the application at 0x10000, not a whole-flash image.**
  Hand-writing it at 0x1000 overwrites the 0x1000 bootloader, and the board then
  boot-loops in the ROM stage (`rst:0x10 (RTCWDT_RTC_RESET)`, never a second
  `load:` line) — which looks exactly like a broken sketch. PlatformIO's own
  upload writes the four images (0x1000 / 0x8000 / 0xe000 / 0x10000); use it, and
  never pipe an upload through `head` (SIGPIPE kills esptool mid-write).
- **A substring match in a console log is not evidence (my own false positive,
  2026-11-23).** An "uplink works" verdict matched the boot banner's
  `[CELLS UNCALIBRATED: run CAL AIR]`, not a `cal air:` reply, and reported a
  link working that had never carried a byte. Rule for every bench assertion:
  match the exact string the firmware prints for that action, and assume a boot
  banner at the start of any capture — opening the console port *is* a reset.
- **GPIO16/17 are the handset UART *and* the WROVER PSRAM pins.** The handset
  link is on 16/17 by design (`pins.h`), which is safe on WROOM and on classic
  WROVER where PSRAM does not claim them; a board with PSRAM active would steal
  the handset port. The console's `heap 203904` is the evidence that this bench
  board has no PSRAM mapped, so the pins are free. Cross-check that number
  before fitting a different controller module: `heap ~5 MB` would mean the
  handset UART cannot live on 16/17.
- **The first real handset command crashed the controller (bench, 2026-11-23,
  C2).** `GET T` tripped the FreeRTOS stack canary in the `uart2` task:
  `Guru Meditation Error: Core 1 panic'ed ... Stack canary watchpoint triggered
  (uart2)`, backtrace through `_dtoa_r ← cvt ← _vsnprintf_r ← proto::emit()`.
  The `T,` line is seven `%f` conversions, and newlib's float printf wants
  several kilobytes of stack — the task had 6144 B, the largest on the board,
  and the 2 Hz downlink survived only because it reaches `emit()` through a
  shallower chain than a routed command does. **The margin was one stack
  frame.** No host test could see this (`test_router` runs with megabytes), and
  the bench could not see it until it rebooted the controller into SAFE.
  Fixed structurally: `src/util/numfmt.cpp` renders every number the device
  puts on the wire, in a dive log, or on the console with integer digits, so no
  bounded-stack task ever calls float printf (the image also shrank ~4 KB).
  `test_numfmt` pins byte-identity against the C library's own `printf` —
  pinned ties (`0.125`→`0.12`, `2.675`→`2.67`, `12.345`→`12.35`) plus a
  ~150 000-value differential sweep across every range the protocol carries.
  That sweep is what found two of my own bugs: fraction digits that never
  advanced, and a scaling multiply that lies (`0.0105 * 1000.0` is exactly
  `10.5`, so a naive implementation applies the tie rule to a value that was
  never a tie — 40 wrong ppO2 digits below 0.4). The exact-product correction
  via `fma()` is why the sweep passes.
- **The missing safety net was a number, not a crash report.** `HB` now carries
  `stk=`: free bytes in the tightest task, self-sampled by each task every 5 s
  (`src/util/stackwatch.cpp` — `uxTaskGetSystemState` needs
  `configUSE_TRACE_FACILITY`, which this build does not enable, and it would
  also drag a ~768 B snapshot buffer onto a 4096 B task). Below 512 the console
  shouts which task it was. `stk=` is **appended** to HB (never reordered), and
  both `soak_logger.py` and `hw_accept.py` match HB by prefix, so the soak
  contract is unchanged. A soak showing `stk=` trending down is a finding even
  when nothing crashes — which is the point.
- **SAFE mode advertised an exit that did not exist.** The boot log said
  "acknowledge via handset" while the next line said the handset is offline
  until a clean boot. Fixed by building the feature rather than rewording the
  message: in SAFE the handset link comes up, the downlink keeps streaming (the
  `T,` line says `SAFE`), every command is refused `ERR safe mode`, and `ACK`
  answers `OK SAFE RESTART` and restarts the controller cleanly. Resume-control-
  in-place was **rejected** on the bench decision of 2026-11-23: SAFE is entered
  because a task died, and continuing on a task set that just lost its stack is
  a state nobody can reason about — a second failure mid-dive finds the diver
  already breathing from it. The web AP stays offline in SAFE.
- **Two writers produced the dive CSV, and they disagreed (resolved
  2026-11-23, found 2026-11-23).** `LOG EXPORT n` (`divelog::toCsv`) emitted 17
  fields per `S,` row — including `mode`, `temp_c` and `flags` — with
  alarms decimal and battery in volts. `/dive/<n>.csv` (`dive_store::exportCsv`)
  emitted 14, with alarms as `0x%04X`, ceiling in integer cm, battery in integer
  mV, and no mode/temp/flags. `BENCH.md` C2 step 4 asserted the download
  "matches `LOG EXPORT <n>`"; it did not, and nothing had ever compared them.
  Fixed by making the row shape a documented, versioned contract instead of
  picking a winner: `logfmt::snapshotCsvLine` / `eventCsvLine` / `metaCsvLine` /
  `diveCsvLine` are the only row formatters in the project, `DiveStore::exportCsv`
  streams through `logfmt::emitCsvRows`, and the old `toCsv` (which only tests
  called) is gone, replaced by `toCsvImage` over a packed file image.
  **Columns are append-only**, which is the part a maintainer must not undo: the
  v1 fields keep their positions, order and units (alarms stay `0x%04X`, ceiling
  stays integer metres, battery stays mV) and v2's additions land on the right
  (`mode,temp_c,flags,ceiling_cm` on `S,`; `gf_low,gf_high,sp1..sp4` on `M,`);
  `ROW_FORMAT_VERSION` rides on the `DIVE,` line. Choosing the richer shape
  instead would have renumbered every field after `setpoint` for anyone with a
  v1 parser, for no benefit a diver can see. Tripwires: six host tests pin the
  exact strings, the field counts, the v1 prefix and that both export paths are
  byte-identical; `tools/web_probe.py` then compares the two transports on the
  wire, where truncation would show up. Two more defects fell out of the same
  reading: `exportCsv` formatted rows into a **64-byte `static char line[64]`**
  (a wide row would have been cut in half and sent as a plausible dive —
  19 fields needs ~90), and the `LOG EXPORT` parser hard-coded `0..9` while the
  store holds ten slots, so slot 10 was listed and web-downloadable but not
  exportable by handset; both bounds now come from `logfmt::MAX_DIVE_SLOTS`.
  Documentation corrections, all doc errors rather than wire errors: field 2 of
  `M,` is `max_depth_cm` (always was — the page divides it by 100),
  `batt` is millivolts, and exported `E,` rows are shape-identical to live
  `E,` downlinks, which is now said out loud in docs 5. Related formatting wart
  fixed in passing on the way: the hand-rolled `%d.%02d` in `dive_store`
  printed a negative value as `-5.-30`.

- **The dive FIFO kept exactly one dive, and the log was unreadable until the
  first dive of a boot (found and fixed 2026-11-24, `0256df6`).** Two defects
  that hid each other. `nextSlot()` decided which FIFO slots were taken by
  opening `/` and matching the directory entries against `diveN.bin`; on this
  core the scan yielded nothing that matched, so every `DIVE START` concluded the
  log was empty and wrote slot 1 again. Ten dives requested, one kept, and the
  symptom was invisible from the surface: `LOG LIST` always answered with the
  newest dive, which is what a working log looks like until the third dive. The
  scan is gone — the ten names are probed with `exists()`, which cannot be
  misread — and the bench now says `before [2, 1]`, after DIVE END
  `[3, 2, 1]`. Separately, nothing mounted LittleFS until `DIVE START`, so after
  a boot `LOG LIST` answered `OK` with nothing in it and `GET /dive/<n>.csv`
  answered `no such dive`, while an esptool `read_flash` of the partition (0x290000)
  showed `/dive1.bin` sitting there with its records intact. That is why "the dive
  log vanished on reset" looked true for a session: an unmounted filesystem, a
  one-slot FIFO and a genuinely lost write all produce the same empty answer, and
  the firmware reported none of them. It reports all three now: the boot banner
  says `log: LittleFS mounted, 3 dives, used 36864 of 1441792 bytes`, and
  `LittleFS.begin(false)` is tried before `begin(true)` because the `true` form
  answers a failed mount by formatting — erasing every dive — at debug
  log level, i.e. silently. `hw_accept.py --dive-persist` closes a dive, resets
  the controller through the console's EN line and requires the same slots and a
  clean boot banner afterwards; `LOG LIST` is also answered by the handset before
  any dive has been started this boot. Lesson for the next thin adapter: a
  persistence claim has to be tested across a reset, by something that counts, or
  it is a claim about the writer and not about the history.

- **Mounting the log at boot made an installed copy of the web page the one the
  browser gets (decided, 2026-11-24).** `serveIndex` used to prefer
  `LittleFS:/index.html` whenever `pageInFlash` said it was there, and until then
  LittleFS was mounted only after the first `DIVE START` of a boot, so on a fresh
  surface the preference never resolved and the built-in PROGMEM copy answered.
  Once the store mounted at boot, the filesystem copy won — and that copy is a
  generation behind whatever firmware was flashed last, because refreshing it
  takes `uploadfs`, which rewrites the partition the dive files live on. The two
  artefacts were demonstrably different on the bench board: it served 25 642 bytes
  while `LittleFS:/index.html` still held the 25 445-byte copy from before.
  **Decision, taken and implemented 2026-11-24: the built-in copy is the only one
  the controller serves.** `serveIndex` answers from `kIndexHtml` in app flash,
  unconditionally; the route that preferred the filesystem copy is gone, and
  `pageInFlash` is now `assetInstalled`, used only for optional assets
  (`/style.css`, `/app.js` answer 404 on a normal board, and `web_probe.py`
  asserts that). Nothing was installed or erased to make that change, and a board
  may still be holding an old installed copy that nothing reads. Consequences: the
  page cannot be a generation behind, cannot be half-written, `data/index.html` is
  a reference copy rather than an install target, and `uploadfs` leaves the C2 gate
  (BENCH.md). Verified on the flashed board: served 25 642 bytes including the new
  marker comment near the top of the page, byte-identical to `data/index.html`,
  with the older copy still sitting in the filesystem.
  Repairing that check was part of the work: `/` was graded as "contains `<html`
  within the first 200 bytes", and the comment pushed the tag to byte 213, so a
  good page failed. It now requires a doctype and an `<html` inside 4 KiB, and was
  injected with an empty body, a plain-text error, a stripped doctype, a missing
  `<html` and HTTP 500 — it fails all five.

- **Removing float printf did not stop the crash, and the backtrace said why
  (bench, 2026-11-23, C2).** `GET T` panicked `uart2` a second time, on a build
  with every `%f` gone. The stack pointers in the new backtrace were the answer:
  `_svfprintf_r` at 0x3ffca270 under `uart2Task` at 0x3ffcb960 — **5872 of the
  task's 6144 bytes consumed on the way in**, and the frames were
  `UartLink::handleLine` 2240 B, `CommandRouter::exec` 1232 B,
  `Session::handleLine` 1072 B. `Response` contains `char text[MAX_REPLY]`
  (1024 B) and was **returned by value**, so one command line held three
  simultaneous copies of it (exec's local, the compiler's hidden return slot, the
  transport's local) ≈ 3.1 KB, and eight more `char buf[MAX_REPLY]` temporaries
  sat in the same path — `GET SP2` alone allocated two of them. Float printf was
  only the last straw; the buffer movement was the leak. Fixed structurally:
  `exec(line, Response&)` takes caller-owned storage, `Response` is **non-copyable**
  (the compile error is the guard — it turned every offender into a build failure,
  in the firmware and in the tests), the transport owns one `Response` and one line
  scratch in the session's static arena, and the big formatters write straight into
  the reply buffer via `replyInPlace` instead of into a 1 KB temporary. Measured on
  the board: `u2stk` 4192 B free idle, **3280 B free after the whole refusal
  matrix**, against 6144 total — the same chain that used to need 5872+ and trip.
- **`Response r{}` had been hiding a wire bug for months.** Zero-initialising a
  fresh reply buffer per command meant the byte after every reply was already
  `'\0'`, so `formatOk`/`formatErr` never had to terminate. With the buffer
  **reused**, leftovers from the previous reply are live: the first build after
  the change answered `ACK` with `OK\n unknown command\n`. Every reply path now
  goes through one `terminate()` helper, and `test_router` asserts
  `strlen(text) == len` after every command on a reused buffer — long reply,
  short reply, repeatedly, because only that order exposes it. A transport sinks
  `r.text` as a C string, so this was bytes on the wire, not tidiness.
- **The acceptance harness reported PASS for the command that was crashing the
  board (found 2026-11-23).** `GET T` answers with a `T,` line and the controller
  streams `T,` at 2 Hz unprompted, so `want ^T,` matched a line the firmware sent
  anyway — while the command itself was busy panicking the task. `hw_accept.py`
  now anchors on the cadence (wait for a streamed line, send immediately after it,
  require a match within 450 ms — the next unsolicited one cannot arrive for
  another 500 ms), which proves a reply deterministically; the naive "two matching
  lines within 400 ms" version I first wrote would have flaked ~20 % of runs,
  because a reply can land exactly in the slot the next unsolicited line would
  have used. It also keeps a console reader open through the command phase plus a
  12 s tail, and fails loudly on `Guru Meditation` / stack canary / `** SAFE MODE`
  / `!! FATAL` / `PRIOR FAULT` / `reset reason: PANIC`, and on missing or low
  `stk=`/`u2stk=`. Before that, a mid-matrix crash presented as ten unrelated
  timeouts and one bogus PASS.
- **`stk=` alone was the wrong number to watch, and the bench proved it.**
  Reporting the *minimum* headroom across tasks pins the reading to the smallest
  task (supervisor, 3072 B), which never moves no matter what a client asks for —
  it sat at `2132` through the entire command matrix while the task that actually
  crashed, `uart2` (6144 B), was invisible behind it. HB now appends `u2stk=`
  alongside `stk=` (append-only, per the HB rule) and the harness requires both to
  stay ≥512. "Worst on the machine" and "the one that can regress" are different
  questions.
- **A bug class that returns once needs a command, not a memory (2026-11-23).**
  The first fix removed the seven `%f` in the telemetry line and the crash
  recurred — `%g` was hiding in `GET SCHEMA`. `tools/float_audit.py` (in the
  pre-flash gate, next to `pio test -e native`) fails on any float format
  specifier in `src/` that is not marked `/*float-ok: why*/`; exactly one marked
  exception exists. While fixing that one: `GET SCHEMA` also formatted its 32-key
  listing **on the task answering a diver**, into a 1 KB temporary, with a guard
  that could serve a truncated listing as if it were complete. The schema is
  derived from a compile-time table, so it is now rendered once by
  `Session::begin()` (not from `main.cpp` — `test_session` immediately caught that
  a boot-sequence call site is forgettable, which is exactly the failure mode) and
  every later `GET SCHEMA` is a copy. `test_schema` pins that all 32 keys fit with
  headroom (858 bytes today) and that the reply is the whole table, never a prefix.
- **Transport decision, recorded: UART2 stays (bench discussion 2026-11-23).**
  Weighed CAN/TWAI seriously — the classic ESP32 has a TWAI controller and the IDF
  driver ships in this toolchain — and RS-485. UART2 wins because the protocol
  assumes full duplex: the controller pushes `T,` at 2 Hz while a command may
  arrive at any instant. 2-wire RS-485 is half duplex and would force
  request/response discipline the compatibility rules make expensive; its
  4-wire full-duplex mode keeps bytes and protocol untouched and is the cheap
  answer for cable length or noise. CAN is the best of the wired alternatives
  (arbitration preserves the model) but a 62-byte `T,` line becomes eight 8-byte
  frames, needing a framing annex — chunk header, sequence, end marker, bus-off as
  a protocol condition. **Revisit CAN for multiple nodes (point-to-point is
  permanent), a console metres away, or galvanic isolation through the housing.**
  BLE is a surface transport, not an in-water one: 2.4 GHz dies in ~10 cm of
  water, so it belongs in the same class as the web AP. Two of the usual reasons
  for CAN/485 are already covered here: the 2 Hz `T,` stream is a link heartbeat
  (a handset knows the link is dead within a second), and a short hose has no
  noise margin to buy.
- **`DIVE START` rebooted the controller inside the WiFi driver — a second,
  unrelated defect, found only because the harness started watching the console
  (bench, 2026-11-23).** The refusal matrix had gone green, then failed twice in
  four runs with the board in SAFE; the console watch named it, and the backtrace
  was not ours at all:

      assert failed: spinlock_acquire spinlock.h:122 (result == core_id || result == SPINLOCK_FREE)
      vPortEnterCritical ← semphr_give_wrapper ← ieee80211_ioctl_process ← ppTask

  `web::stop()` ran `softAPdisconnect(true)` + `WiFi.mode(WIFI_OFF)` **on the task
  that was answering the client**, with a station still associated. The deauth is
  asynchronous: the driver's own worker is still inside an ioctl when
  `esp_wifi_stop()` runs, and it asserts on a semaphore it can no longer take.
  It fires on the dive-start radio drop — the one moment the radio is designed to
  come down — and it is intermittent, which is why `hw_accept.py` scoring only
  the handset replies missed it. Fixed with the pattern already used for other
  hardware side effects (read to answer, post to act): `comms::web::setRadio()`
  now only records intent, `loopTask` performs the change (`radioPending()` /
  `serviceRadio()`), and `stop()` drains the driver (250 ms after the deauth)
  before `esp_wifi_stop()`. Deferred rather than done-inline also keeps a
  several-hundred-millisecond WiFi teardown from holding the session mutex and
  stalling the 2 Hz downlink. Verified: 2 full acceptance runs clean, then
  `tools/radio_stress.py` — 12 explicit OFF/ON teardown cycles, 0 fault markers.
  A race that "did not happen twice" is not fixed, hence the committed stress
  tool. Not host-testable: `web_server.cpp` is excluded from the native build.
- **A deferred action is a cancelled action, and the acceptance harness had to
  be fixed before it could see that (bench, 2026-11-23).** `web::setRadio()` now
  only records intent, so the radio actually moves when `loopTask` next looks.
  `hw_accept.py`'s matrix runs in under a second — `DIVE START` at t+22.49 s and
  `DIVE END` at t+22.94 s — so the OFF intent and the ON intent cancelled each
  other before any task acted and the radio never came down. `radioPending()`
  compares *desired* with *current*, which is the right semantics (a dive of
  0.45 s gains nothing from a radio drop) but it made the new check report
  "0 radio lines" on perfectly good hardware. Two changes: the matrix now holds
  the dive open (`dwell:2.0` after `DIVE START`, `dwell:4.0` after `DIVE END`),
  because a check that demands an event the sequence cannot produce is not a
  check; and `loopTask`'s radio poll went from 1 s to 250 ms, since "the antenna
  comes down at DIVE START" (PLAN 9: milliamps, water) is hollow if the request
  can sit pending for a second. Measured: down in 0.36–0.56 s, AP back in
  0.31–0.51 s, 8/8 and 12/12 cycles, AP name identical every time, heap drift
  −192 B over 12 cycles (the 2.5 KB drop on the first cycle is one-time driver
  allocation, not per-dive).
- **`esp_wifi_get_mac failed with 12289` on the console is ESP-IDF's, not
  ours** — and the way it got proved is the useful part. It appears on some
  teardowns (`12289` = `ESP_ERR_WIFI_NOT_INIT`). The first diagnosis was that our
  own `WiFi.macAddress()` was being read before `WiFi.mode(WIFI_AP)`; moving the
  read was plausible, and **wrong**: injecting the old call order back and
  watching the console put the marker in exactly the same place (always before
  `web: radio off`, i.e. inside `esp_wifi_stop()`), and `grep` over the Arduino
  WiFi library shows it never calls `esp_wifi_get_mac` at all (only `WiFiProv`,
  unused). `WiFi.macAddress()` returns the core's cached address without touching
  a stopped driver. So the AP name was never fragile the way the comment claimed;
  the tail is cached. The ordering and the once-only tail cache are kept because
  they are the defined-to-work order and they keep the name from drifting, but
  the comment now says what was measured. Both tools report the line as a note
  and do not fail on it.
- **Three acceptance checks found this session to be vacuous, which is the
  pattern to watch.** `GET T` matched an unsolicited `T,` line, so the command
  that was rebooting the board scored PASS. The stack tripwire reported the
  minimum across tasks, which belongs to the supervisor and never moves while
  `uart2` falls over. The radio check demanded a teardown the matrix's 0.45 s
  dive could never trigger, so it failed good hardware — the mirror image of the
  same error. Each was fixed by making the assertion about something the harness
  can actually distinguish: cadence anchoring for replies, a named per-task
  reading (`u2stk=`), a dwell that lets a deferred action land. The reflex to
  add: for every new check, ask what a *broken* build would do differently, and
  if that is not obvious, inject the bug and watch the check fire. That was done
  here for the MAC regression — and it is what showed the diagnosis was wrong.
- **`AsyncWebSocket::enable(bool)` is not a socket close** (ESPAsyncWebServer
  3.12.1: it sets the flag `canHandle()` tests, `AsyncWebSocket.h:346` consumed
  at `AsyncWebSocket.cpp:1344`). `stop()` was calling `enable(true)`, a no-op
  that reads like teardown; removed, with the reason written down. Verified
  against the vendored library rather than from memory, because the call looked
  plausible and the comment next to it was about something else.
- **The boot line's AP address was a literal** (`on 192.168.4.1`), while the
  captive DNS answers with `WiFi.softAPIP()` — the two could drift and the bench
  would be told to look at an address nothing serves on. Now prints `softAPIP()`.
- **`board_build.filesystem` defaults to SPIFFS, and the firmware mounts
  LittleFS** — so `pio run -e esp32 -t uploadfs` had *never* installed the web
  page. The image went to the same partition (1 441 792 bytes at 0x290000) in a
  format nothing reads, `LittleFS.begin(true)` carried on with whatever was
  already in the partition, and whatever older copy the bench happened to be
  holding is what got served. Set explicitly in `platformio.ini`, with the
  reason written down. The general shape: a build-system default that has to
  agree with a runtime call in another file is a latent defect with no compile
  error and no host test — only fetching the URL compares them. `web_probe.py`
  now asserts **the served bytes equal `data/index.html`**, the only end-to-end
  check over `web_page.h` → `sync_webdata.py` → `data/` → the router's
  LittleFS-or-flash choice.
- **`web/` is gone (2026-11-25).** A superseded page generation, 635 lines, whose
  assets matched nothing in the router — `/app.css` and the four sub-pages
  (`checklist`, `dives`, `histogram`, `settings`) had no routes at all — and whose
  README stated something false and dangerous: that the pages were "served from LittleFS
  on the device". They are not: the served page is `src/comms/web_page.h`, compiled into
  the app image, and that partition holds the **dive logs**, so writing it erases dive
  history. The firmware was already honest about this — `/style.css` and `/app.js` answer
  404 unless someone deliberately installs a file, and `web_probe.py` asserts exactly
  those 404s — the lie lived only in prose, which is the kind of thing that sends the
  next reader to fix a UI the board never serves. That drift has already bitten the bench
  once (C2-f: the page the board served was not the page in the repo). What keeps a
  second UI from creeping back is not the deletion but the byte-exact check:
  `web_probe.py` requires the served bytes to equal `data/index.html`, which
  `tools/sync_webdata.py` generates from `web_page.h`. One page, one source, checked on
  the wire.
  (Recorded and closed 2026-11-23: the worry that a LittleFS install could
  shadow a firmware fix was **an artifact of the mismatched installs on this
  bench**, not evidence about the design — owner decision, do not carry it until it
  is seen again.)
- **The web page and the dive logs share one LittleFS partition**, so `uploadfs`
  deletes dive history. Observed rather than inferred: slot 1 held a 338-byte
  dive before the first real install; after it, slot 1 was one of that session's
  3-record bench dives and slot 2 was empty — had the older dive survived,
  `nextSlot()` would have taken slot 2. A future filesystem layout should
  separate the two; until then `uploadfs` is a destructive step and BENCH.md says
  so.
- **Un-terminated bytes are glued onto the next command line** (proved by
  injection, 2026-11-23): send `XY` with no newline, then `AUTO\r\n`, and the
  controller answers `ERR unknown command` because it parsed `XYAUTO`; the next
  `AUTO\r\n` is answered correctly. So one bad line costs one error reply and the
  parser resynchronises without help — worth knowing before anyone writes resync
  logic for a handset. What makes it a bench problem is that opening the handset
  adapter's port resets the bridge ESP32 and its boot banner goes straight into
  the controller's RX, and `reset_input_buffer()` clears only our side. Both
  harnesses now send a newline first and report what it drew: a bare newline
  answered `ERR unknown command` is proof bytes were pending. Without that, the
  first check of a run fails for a reason unrelated to the firmware — which is
  how this was first mis-read as a cross-transport disagreement.
- **Cross-transport equality must interleave, not run as two passes.** Proving
  "one command path, two transports" means the same command twice, and if the web
  pass runs its whole matrix first, its own writes are still in effect when UART2
  reads: `GET *` "disagreed" (`SP2=0.550` vs `0.700`) for no reason but ordering.
  One command, both transports, back to back. The matrix is also ordered
  refusals-and-reads first and the single write plus its read-back last, so a run
  is state-neutral up to that point.
- **`#` is not one vocabulary on the web transport.** `#<n>` means "answer
  complete"; `#boot <uptimeMs> <freeHeap>` is a hello sent when a socket connects.
  A client that treats any `#` line as the marker stops reading at the hello and
  attributes every reply to the *next* command — nine checks failed on good
  firmware that way. Clients must match the marker shape exactly; both lines are
  named in `docs/HANDSET_PROTOCOL.md`.

- **A bench tool must not reconfigure the machine it runs on.** An early
  `web_probe.py` brought a network connection up when the station did not rejoin
  after a dive, naming a connection profile this project knows nothing about.
  Joining the AP is the operator's setup; inventing it made the tool fail the
  moment the operator's own setup was in place. Now the tool talks HTTP and
  WebSocket to `--host`, only ever *scans* to confirm the AP is broadcasting, and
  carries three verdicts — a check it cannot observe is "not evaluated", not a
  failure charged to the firmware.

- **A full dive FIFO has no way to tell which dive is oldest (open defect, found
  on the bench 2026-11-24, C2).** `nextSlot()` evicts the dive with the smallest
  start epoch, and every dive on this board carries epoch 0 — no RTC, and no NTP
  through an access point with no uplink — so all ten keys tie and the comparison
  decides by accident. The bench sequence: fill ten slots with 4 s dives, then
  write 8 s, 40 s, 8 s. The board came back holding durations
  `8, 2, 3, 4, 5, 7, 7, 9, 10, 10` seconds — **the 40 s dive was gone before the
  next boot**, and the same thing happened in an earlier fill: twelve dives of
  2, 4, ... 24 s left slots 2..10 holding dives 2..10 and slot 1 holding dive 12,
  with dive 11 simply gone. So the log keeps the *nine oldest* dives plus the very
  newest, and silently discards recent ones. It is not the comparison operator:
  `<` and `<=` both pick a fixed slot when every key is equal, so no tie-break can
  fix it. What is missing is an ordering identity. Epoch ordering needs an epoch,
  and there is no clock.
  **Closed on the bench, 2026-11-24.** The header gained a monotonic dive
  sequence in `out[5..7]` — 24-bit, saturating — and `DIVE_VERSION` did NOT
  move: the layout, size and CRC coverage of the header are unchanged, so
  every pre-change file still reads, as sequence 0 = "age unknown", and
  unknown-age dives are evicted before any sequenced one. The ordering
  decision is pure host-tested code (`logfmt::nextSeq` = max-on-partition + 1,
  `logfmt::pickEvictSlot`: smallest sequence first, real epoch as tie-break,
  then lower slot, so the choice is stated rather than emergent from a scan).
  A counter in NVS was rejected: the dive files are the durable history, and
  a second copy of the order somewhere else is one more thing that can
  disagree with them. The order is now visible where it was invisible — the
  boot banner says `log: LittleFS mounted, N dives, seq a..b`, and the `L,`
  lines carry a fourth field — so `hw_accept.py --dive-persist` asserts WHICH
  dive survived (the newest sequence must still be present after the reset),
  not just how many. The bench re-run with distinguishable durations: after
  twelve more dives the surviving set is exactly the last twelve, and the
  eviction index rotates through the slots instead of hammering slot 1. The
  injected bug ("ignore the sequence, order by epoch as before") makes the
  host test fail with `Expected 1 Was 0`, which is the bench defect in one
  line. Epochs also mean something now — see the wall-clock entry below.
- **The harness promised something the bench contradicted: opening a port does not
  reset the board (2026-11-24).** BENCH.md's first rule says the open *is* a reset
  pulse, and `tools/port_id.py`'s `open_quiet()` docstring said the opposite —
  "no reset pulse" — with DTR/RTS held low offered as the reason. Both sentences
  were in the repo at the same time, and measurements were being made against the
  wrong one: a quiet console open was used as a look that did not disturb the
  board, and the boot banner that followed was read as firmware behaviour.
  Measured: nine seconds of quiet listening on `/dev/ttyUSB0` after the board had
  been running produced `rst:0x1 (POWERON_RESET)` and a fresh banner. Holding DTR
  low buys one thing — no IO0 download-mode latch, which does look like a dead
  board — and not the other. So a quiet console read means "restart the controller
  and read its boot log": never conclude that a board rebooted by itself from
  traffic that starts with a banner, and never compare state across a port open
  without counting the reset in between (a dive-log slot list read that way is a
  list read across a reboot). The docstring says so now.
- **Two readers on one port do not both fail loudly.** The second reader gets
  `SerialException: device reports readiness to read but returned no data
  (device disconnected or multiple access on port?)`, and during this session that
  exception came from a probe of mine that was still holding `/dev/ttyUSB1` while
  a new one opened it — the first reader alive, not the firmware misbehaving.
  `fuser /dev/ttyUSB0 /dev/ttyUSB1` before any run; rc=1 means free.

- **Wall clock: a client at the surface IS the time source (decision,
  2026-11-24).** The dive epoch had been a hard-coded `0` in `uart_link.cpp`
  since the log existed: no RTC chip, no NTP through an access point with no
  uplink, and nothing anywhere said so. Since the FIFO defect above was
  partly a clock problem in disguise, the clock got designed rather than
  patched. Decisions, all deliberate:
  * `TIME SET <epoch>` and `GET TIME` on the handset protocol (a command,
    never a settings key — settings live in NVS, and resurrecting a saved
    epoch after a battery pull would date every dive with a number from a
    previous life in silence). The built-in page posts its phone's clock on
    every WebSocket open, so the normal surface flow sets it without anyone
    typing.
  * Accepted window 2000-01-01..2100-01-01; a refusal changes nothing and
    says `ERR epoch out of range` on the socket it arrived on. A
    plausible-looking wrong epoch is worse than no epoch, because every
    reader of the log believes it. `0xDEADBEEF` passes the range check —
    that is why the RTC carry needs the magic too.
  * The clock state is pure arithmetic (`epoch = epochSet + (nowMs -
    setAtMs)/1000`, `src/comms/wallclock.{h,cpp}`), with the monotonic
    millisecond clock handed in by the caller — same shape as
    `CommandRouter::setNowMs`, host-testable including the ~49.7-day ms wrap.
    It is mirrored into `RTC_NOINIT_ATTR` every control cycle (magic +
    epoch), so a carried clock stays CURRENT rather than freezing at the
    instant it was set. What the carry survives, measured on the bench
    2026-11-24 (the design assumption was wrong in one corner and the bench
    caught it): firmware-initiated resets (panic, watchdog, SAFE `ACK`) keep
    RTC no-init memory; the **EN pin does not** — pulling EN on this board
    is a CHIP_PU power-down, the next boot reports `rst:0x1 (POWERON_RESET)`,
    and the clock is honestly `unset` afterwards, exactly like a battery
    pull. A power-down also leaves plausible-looking garbage in that region,
    which is what the magic+range rejection is for: `GET TIME` and the boot
    banner then say unset, never a date from a previous life.
  * A dive started with the clock unset is **recorded anyway**, and says so
    in three places: an `E,,15` (EVT_CLOCK_UNSET) row inside the dive file,
    `clock-unset` in the CSV date column, and `clock: unset...` at boot.
    Refusing to log a dive over a calendar would cost the diver the dive data
    — the opposite of what the machine is for.
  * The dive CSV identity row went v2→v3 by **appending** field 6, ISO-8601
    UTC (`DIVE,<slot>,<epoch>,3,3,2026-11-24T08:41:00Z`). It is rendered
    from the stored epoch, never stored, so there is one truth on the flash
    and pre-v3 dives export correct dates too. The renderer is the project's
    own (`logfmt::isoUtc`, Hinnant's civil-from-days): `gmtime`/`strftime`
    would pull newlib's time and float tables into the task that answers
    clients, the stack that has run out before.
  * `LOG LIST` now lists by sequence, newest-written first. Slot-number
    ordering became a lie the moment eviction started rotating — a full FIFO
    with the newest dive in slot 4 lists slot 4 first. Unsequenced (legacy)
    dives list last, slot-descending, exactly as before.
  * The bench rule this creates: **set the clock at the surface after every
    power-up.** `hw_accept.py` does it in the refusal matrix, and
    `--dive-persist` asserts the whole chain — dive stamped with a real
    epoch, boot after the reset says `clock: ... (carried across a reset...)`.
- **The contract document and the client package drift apart, and the drift is
  invisible to every suite (found by reading, 2026-11-24).** The dive-log batch
  added `TIME SET`, `GET TIME`, the `TIME,` answer, the fourth `L,` field and
  the dated `DIVE,` row. The spec described all of them, the firmware emitted
  all of them, the bench gates asserted all of them — and `handset/handset_link.h`
  knew none of them: no way to set the clock, no way to see dive order, no way
  to read a dive's date. Nothing failed. That package exists so a handset can
  be built **by someone else from the contract**, so the gap was not
  cosmetic: a third-party handset built the week's "correct" way could date
  nothing and order nothing, and `test_handset` would have certified it,
  because the suite pins the library against wire literals — from the last time
  someone updated it. Two rules come out of this, both now in force:
  * **A protocol-visible addition lands in ONE commit across the whole
    package** — spec, client library, example firmware, README and the
    conformance suite. The firmware and the spec agreeing is not the invariant;
    the firmware and every published client of it agreeing is.
  * **The suite's literals are captured off the wire, not typed from the
    spec.** A literal copied from the grammar proves the parser matches the
    grammar; a literal copied from the board proves it matches the controller.
    The 2026-11-24 refresh used `TIME,1795518150,1,509`, `L,7,1795517641,27`,
    `L,6,0,26` (a real undated dive) and
    `DIVE,10,1795516836,3,3,2026-11-24T10:40:36Z`, all read from the bench
    link first. It also retired two stale literals the suite had been passing
    on (`L,3,1764123456`, `DIVE,3,1764123456,1`): both are kept **as legacy
    cases** — a v1 client must still parse a pre-append controller — which is
    the difference between an out-of-date test and a compatibility test.
  The same drift is what §16's "`OK` is about the command, not the effect"
  entry is about, seen from the other side: a contract is only worth the
  weakest published client of it.

- **The surface log is Web/HTTP only, and the costs of that are written down
  rather than discovered later (decision + build, 2026-11-24).** Calibration
  verdicts and pre-dive results now have a durable record (`/surface.bin`,
  spec §5.1, web "Surface" tab). Two questions were decided, not drifted into:

  * **Does a handset get commands for it?** No. `LOG LIST`/`LOG EXPORT` already
    mean "the dives": their `L,` and `DIVE,` shapes are published, and welding a
    second class of record onto them would make every published client
    re-validate a shape it already consumes. A second pair (`SLOG LIST`/
    `SLOG EXPORT`) is what a handset would need, and it is not built; when it is,
    it lands as spec + client library + example firmware + README + conformance
    suite in one commit, with new line tags, **never** as a new field inside `L,`.
    Stated costs, both in spec §8: the wired handset — the in-water transport,
    and the client that matters most — cannot ask "what did the checks say an
    hour ago", and with no session-start record a surface record cannot say which
    power-up it belongs to. A handset can still hear the air-calibration verdict
    live (`E,*,7,<code>`); it can only poll `PRE,` for a pre-dive one.
  * **Why no client-library change then, given the one-commit rule above?**
    Because the rule is about *protocol-visible* additions, and this batch adds
    none on UART2: no new command, no new line tag, no new field on any existing
    line. Event id 16 is in the spec's id table marked **not published**, so a
    client that does not know it loses nothing — which is the append-only rule
    working, not dodging it. The spec gained §5.1 (the row shapes) and §8 (the
    limit); the firmware, the web page, the tools and the host suite gained the
    rest, in one commit.

  Design details worth not re-litigating: 24 B stride with its own version byte;
  no file header, because with one flat stream there is no per-file identity to
  carry and a header would be the one place a torn first write could lose
  everything; per-record CRC so a reset costs one record and leaves the reader a
  clean place to stop; the sequence read back off the file (this board has no
  clock to order anything with) and **not shared** with the dive sequence,
  because the dive sequence decides eviction; compaction staging to
  `/surface.tmp` then rename, whose one power-cut window is stated in the header
  and happens at most once every 256 records; `count()` propagating `-1` so
  "the file will not read" never arrives at the banner as "no records yet"; and
  `SURFACE,<ver>,<count>,<seq_lo>,<seq_hi>` plus `seq a..b` at boot being the
  capacity evidence — a range wider than the count means history was shed, and
  the log says so instead of presenting a short list as the whole story.
  Raw ADC counts are stored as integers through `logfmt::countsFromFloat`, which
  rounds *through* `numfmt::fixed(v, 0)`: the console, the `CALAIR,` line and the
  flash record cannot then disagree by one count about the same calibration,
  because the tie rule (nearest, ties to even, like printf) lives in one place —
  a 12-sample average lands on an exact `.5` often enough to reach it.

  Two harness bugs from the same day, both of which read as firmware faults and
  both now written next to the code (see `BENCH.md`): anchoring the boot-line
  pattern with `$` matched **none** of the three forms the firmware prints, so
  the run reported a silent boot; and a page-honesty regex ("renders records as
  words, not codes") passed on the *previous* page, where its two patterns came
  from the CHECK button's label and the checklist view — the fix was verified
  against `git show HEAD:data/index.html`, because "would this have been red
  before the feature?" is the only question that grades a check.

- **A dive file that records only the effects of a decision cannot be reviewed
  (found on the wire 2026-11-24, fixed the same day).** The cell-proof clamp leg
  (`BENCH.md` C3-a) streamed its three verdict rows — `E,54,10,0` proof FAILED,
  `E,54,11,0` clamp ON, `E,66,12,1` clamp OFF by the diver — and the dive file
  exported afterwards held **none of them**, from either transport (`LOG EXPORT 8`
  and `GET /dive/8.csv` both answer `grep -c "^E,"` → 0). A reader of that file
  sees the setpoint column run `1.100 → 0.900 → 1.000` and the alarm word run
  `0x0010 → 0x0030 → 0x0010`, and must *infer* that a proof failed and a human
  overrode it. That is the difference between a dive log and a strip chart. The
  same dive's `M,` line read `M,66,743,0.212,0.215,0,0,0,0,0,0,0.000,0.000,0.000,0.000`
  beside 13 `S,` rows: a summary contradicting its own dive.

  Three decisions worth not re-litigating:

  * **One publication, two records.** The wire line and the file row come out of
    the *same* pop in `UartLink::drainEvents` (`Hardware::appendEventNow`), so
    they cannot drift; before this, `drainEvents` appended an event-driven
    **snapshot** per event and discarded the event row, and `DiveStore::append
    (const Event&)` — which works, and is unit-tested — was called only for
    `EVT_CLOCK_UNSET`. Order inside the file is the verdict **first**, then the
    snapshot that shows the state it was decided in.
  * **A count belongs to the object that wrote the rows.** `M,`'s `snapshots`/
    `events` are patched from `DiveStore`'s own counters at close (incremented
    only on a write that *worked*, reset by `startDive`), never copied from a
    chain-side total. `toMeta()` deliberately leaves them zero, and a host test
    asserts that, so a "helpful" fill cannot double-count.
  * **The plan a dive ran on is part of the dive.** `gf_low`/`gf_high`/`sp1..4`
    are read off the live config by ControlTask during the dive and carried in
    `bridge::DiveTotals`, so a file is reviewed against the GF pair and ladder it
    actually used, not the ones on the board now. An unset plan stays 0 — the
    format's own "nothing to say" value — rather than a number someone made up.

  No protocol-visible addition: `E,` rows have been in the published dive CSV
  since v1 and the plan columns since v2, so no client re-validates anything and
  the one-commit spec/client-library rule was not triggered. What changed is that
  the firmware started keeping the contract it had already published.

  The new host test earned its keep immediately: it reddened on a bug nobody was
  looking for. `DiveTotals::toMeta()` guarded the min-ppO₂ sentinel but not the
  max — `maxVotedX1000 < -32768` is never true, and the field *starts* at −32768
  — so a dive where no vote ever arrived wrote `max_ppo2 = -32.768` bar into its
  own summary, in the same three decimals as a real reading. It hid because every
  dive taken on this bench had votes.

  Related honesty note, deliberately **not** changed: after `CELLPROOF OVERRIDE`,
  `GET PROOF` answers state `4` = PASS, because `Engine::override()` moves
  FAIL→PASS — the diver outranks the machine. A handset cannot then distinguish
  "cells proven" from "diver overrode a failure"; the distinction is in the
  `E,<t>,12,1` row and the id-10 row before it, which the dive file now keeps.

- **Depth was measured with two rulers, and one of them was 1000× wrong
  (found on the wire 2026-11-24, closed the same day).** The 1000× story itself is
  in `BENCH.md` C3-a (`T,37.04` in an open room, `BAR_PER_M_SEAWATER` dividing a
  **kg/L** density by 100000 when `ρg` is Pa/m only for **kg/m³**). Writing the
  deferred pin test for it exposed the second, quieter half: the two paths that
  convert pressure to depth did not merely re-derive the same arithmetic, they
  read **different densities**. `main.cpp` used the compile-time
  `WATER_DENSITY_KGL` (1.025); `zhl16c.cpp` used `cfg_.waterDensity`, which
  `SET WATER_DENSITY` (schema 0.990…1.100) overwrites. A diver in fresh water
  would have had the depth on the `T,` row and the setpoint bands on a 9.948 m/bar
  scale while the ceiling, NDL and tissue model ran on 10.197 — **2.5% at every
  depth**, and neither path wrong in its own terms, so no comparison the machine
  makes with itself could see it. Three decisions:

  * **One formula, in one file**: `src/util/depthscale.h`, `constexpr`, with
    `barPerM` / `depthFromBarDelta` / `ambientBarAtDepthM` /
    `depthFromAmbientBar` / `depthFromSensorBar`. `BAR_PER_M_SEAWATER` is
    **deleted** from `defaults.h` — leaving a constant nobody is required to call
    is how the split survived; the units lesson that used to be its comment is
    now a warning about why there is no constant. (The figure that comment
    headline'd, 9.947 m/bar, is 9.948 at 1.025 kg/L.)
  * **One density**: the `WATER_DENSITY` setting, through `cfg.deco.waterDensity`,
    at both sites. `WATER_DENSITY_KGL` survives only as the schema's compile-time
    default, and a host case pins the three places 1.025 lives (build default,
    settings default, deco engine default) so they cannot drift.
  * **Two depth rules, deliberately different**: what a diver reads
    (`depthFromSensorBar`) is never negative — above the reference plane the
    answer is 0 m, because a negative depth is weather or altitude, not something
    to dose against; the deco engine (`depthFromAmbientBar`) gets the honest
    negative, since a ceiling can sit above the surface.

  Test shape, because the obvious test proves nothing: **a round-trip test cannot
  catch a scale error** (a wrong constant round-trips perfectly), and a test that
  derived expected depth from the macro would have inherited the 1000× error. So
  `test/test_depthscale` asserts hand-worked literals — `0.1005181625` bar/m at
  sea, `0.0980665` fresh, 9.94844 m/bar, `0.0368` m for this bench's own surface
  offset (1016.7 hPa against the fixed `SURFACE_PRESSURE_BAR`) — plus a cross-
  ruler case against `Zhl16Ccr::barToDepthM`/`depthToBar` at three densities, and
  a **source scan** (both users must call `depthscale::`, must not contain
  `9.80665`/`rhoG`/`BAR_PER_M_SEAWATER`, and every call must name `waterDensity`).
  The scan is not laziness: `main.cpp` is excluded from the native env, so a text
  rule is the only way the host suite can see that line at all — and it is the
  **only** thing that catches a `depthscale::` call handed a literal density,
  which no numeric assertion at the shipping density can see.

  Coverage limit, recorded rather than papered over: the deco engine's two
  *private* depth grids (NDL binary search, 3 m stop walk) are guarded by that
  scan alone. A plan-level "fresh versus sea must move the published ceiling and
  total deco time" case was written and measured (ceiling 11.15 m at 1.025 vs
  11.43 m at 1.000 for one 30 m/25 min profile), then **deleted**: no mutation
  made it the only witness, since the ruler case or the scan already caught every
  subset of call sites. Exact schedules stay pinned only at the shipping density
  (`test/test_deco`); fresh-water deco shape is untested on purpose.

  Bench side the property stopped being arithmetic in a script: `hw_accept.py`
  now reconciles the console's announce (`1016.7 hPa`) with the first `T,` row's
  metres on the density the client can read back (`GET WATER_DENSITY`, added to
  the protocol matrix), tolerance 5 cm of water. The tool computes the physics
  itself rather than importing the firmware's formula — an independent witness is
  the point. Four grader mutations reddened exactly their own cases (the tool's
  scale given `/100000`, tolerance widened to 50 m, the never-negative clamp
  dropped, a silent no-check return when no `T,` row arrives). A squeeze leg with
  the ruler moved is written into C3-a as the wire demonstration, and deliberately
  **not** automated — at the surface the whole legal density range moves the
  reading by 0.004 m, so a boot-line-only check could never see a density that
  stopped reaching the conversion. (As first written it said "set the density
  while holding a few metres"; that command is refused now — see the dive-lock
  entry below — so the leg sets the ruler at the surface, dives, and squeezes to
  ~5 m where the 2.5% clears two decimals.)

- **The density is the ruler, so it is dive-locked — and writing the check for that
  found a second defect (2026-11-24, 16).** One density now drives the `T,`
  metres, the setpoint tiers those metres select, the cellproof arm depth and the
  deco engine's ambient pressures. A mid-dive `SET WATER_DENSITY=1.000` is inside
  the published range (0.990…1.100) and moves a diver's whole depth scale ~2.5% at
  a pressure that has not changed. Decision: **`FL_DIVE_LOCK`** — the same
  treatment setpoints and alarms already get, for more than they do. It is a
  protocol-visible refusal (`ERR dive locked`), so spec §6 gained the flag, the
  table row and the reason; no client-library change was needed, because the
  dive-lock bit is already in the published `K,` bitmap and the page already
  disables inputs carrying `data-lock="1"` while a dive runs. That is the useful
  reading of the one-commit rule: a client built from the schema needed nothing,
  and the commit that changed the behaviour also changed the schema bit it reads.

  The check had to be built past the refusal. `ERR dive locked` reports what the
  router *said*; it says nothing about the registry. A router that printed the
  refusal and wrote the value anyway passes every reply-only check on the bench
  and in the host suite — the "ack with no consumer" defect wearing refusal
  clothes — and no depth comparison can catch it at the surface, where the whole
  legal density range is worth 4 mm. So the matrix writes a mid-range value
  mid-dive, reads the ruler back **during** the dive, restores at the surface,
  reads again, with every command paired to its own reply, and `hw_accept.py`
  grades four things including "the ruler did not move DURING the dive"
  (`grade_density_lock`, 10 lying-capture selftest cases). Proven by mutation on
  hardware: `FL_NONE` reddened three checks, and the post-dive read alone stayed
  **green**, because the matrix's own surface restore overwrites a leaked value.
  The mid-dive read is the only witness to a dive that ran on a scale nobody
  agreed to — the general shape being that a check placed *after* the state that
  matters cannot see it.

  What that probing found: the first version asked for `0.990`, the **declared
  minimum**, and a build without the flag answered `ERR out of range` — the right
  shape of `ERR`, for an unrelated reason, so the entry passed for the wrong
  reason. The unrelated reason was itself a defect: `settings::setValue` parsed
  with `std::strtod` (double) and compared against a `float` bound, and that
  comparison promotes the bound by up to one ULP. The schema's decimal bounds are
  not exact binary values (`0.990f` is 0.99000000953674316, above the double
  0.990), so 13 of the 30 numeric keys refused their own published endpoint — mins
  SP1…SP4 (0.20), `SP_HYST` 0.1, `ALLO_LOW` 0.05, `CAL_O2_FI` 0.85, `CAL_O2_TARGET`
  0.80, `CELLPROOF_MAXSP` 0.60, `WATER_DENSITY` 0.990; maxes `ALLO_CUTOFF` 1.80,
  `CELLPROOF_SP` 1.40, `CELLPROOF_MIN` 1.30. `GET SCHEMA` hands those numbers to a
  handset and the page puts them in `<input min= max=>`, so the device advertised a
  value it refused when the diver typed it. Fixed by parsing **and** comparing in
  `float`, the type both the schema and the storage use (same class of bug in the
  `needs integer` check, which compared a double against a `long` cast). Guarded
  by a walk rather than a spot check: `test_every_key_reaches_its_own_declared_bounds`
  asserts both endpoints parse and ~1% beyond each is refused, for every numeric
  key, plus a floor of 25 keys walked so a loop that matched nothing cannot pass.
  Restoring `strtod` reddens it (`SP1 refuses its own minimum 0.2: out of range`),
  and the endpoints are now a published contract in spec §8.

  Rules recorded, because they generalise:
  * **Never probe a refusal with a schema endpoint.** A boundary value can be
    refused by a second rule, and the check then passes for the wrong reason.
    Refusal probes use comfortably mid-range values (`1.000` here).
  * **A refusal is not proof that nothing changed.** A client that needs to know
    whether a locked write took effect reads the value back, in the same session,
    while the refusing state still holds.
  * **"It passed before I edited it" is not evidence that a check checks anything.**
     The failing `no double-start` assertion in this work was first explained as
     stale bytes in a reused `Response` ("`exec()` resets `len` but not what
     follows"), and that explanation went into the code before it was tested. It is
     FALSE: `exec()` also writes `text[0] = '\0'`, and every reply path terminates at
     `len`, so a reused buffer cannot leak an old reply into a needle test — an
     invariant pinned since `15e08a1` by `test_reused_reply_buffer_never_leaks_the_previous_reply`,
     which exists because a real regression once did put an old reply's tail on the
     wire. What actually happened is the transferable lesson: the `DIVE END` added
     earlier in that same case changed the state the trailing assertion depended on,
     so it was no longer testing a double start at all — and the behaviour it had been
     covering had no case of its own, so it disappeared with the state. It now has
     one, `test_re_dive_after_a_closed_dive_is_a_fresh_dive`, which additionally
     pins the refusal's reason (`dive active`: a diver told only `ERR` cannot tell an
     open dive from a broken command) and that a `DIVE START` after a closed dive
     opens a NEW dive — which is what a dive log's two headers depend on.

- **The parser was shortening values the schema promises — and the endpoint fix is
   what made that dangerous (2026-11-24, 16).** A queued known lie, closed. The command
   parser carried `char value[24]` (23 characters) while `AP_PASS` publishes a
   **length** range of 8..31, so a 24..31 character passphrase was accepted by the
   registry, answered `OK`, and stored the first 23 characters: the access point then
   ran on a key the diver never typed, and the only evidence on the wire was a
   `GET AP_PASS` that came back shorter than what he sent.

   A claim in the first draft of this entry had to be withdrawn: it said the defect
   had been latent until the numeric-endpoint fix made long passphrases legal. **No.**
   A string key's `min`/`max` are lengths, validated with integer comparisons
   (`strlen` against 8 and 31), so the float-ULP bug never touched them — the
   truncation was reachable before that fix and after it. Checked by mutation, not by
   reasoning: restoring the pre-fix `strtod` reddens **only**
   `test_every_key_reaches_its_own_declared_bounds`; the value-cap cases stay green.
   Two independent rules that never masked each other. The page is the proof of
   reachability: it builds its inputs from `GET SCHEMA` and sets `maxLength` from the
   `K,` line, so the page let a diver type exactly 31.

   Fixed as a rule, not as a bigger number:
   * the parser's field is 40 bytes and a `static_assert` in `cmd_router.cpp` ties it
     to `settings::MAX_VAL_STR` — the transport can never again be narrower than what
     the registry accepts. The client library's own `Setting::value` has been 40 bytes
     all along, so the controller was always the limiting factor, not the handset;
   * a value that will not fit is **refused** (`ERR value too long`), the registry's
     own phrase, so a client learns one vocabulary at either layer. That is the
     decision already taken for over-long *lines* (an over-long line is dropped whole,
     never truncated-and-run) applied to the fields inside a line;
   * an over-long **key** is refused too (`ERR key too long`). A truncated key is
     worse than a truncated value: it can address a different setting than the client
     named.

   Host gates, each proven by injecting the bug: copying into 23 bytes anyway reddened
   `test_a_full_length_passphrase_survives_the_parser`, the schema walk (`AP_PASS: the
   parser shortened a value the schema publishes as legal`) and the router case
   (`the ack was OK and the stored string is not what the client sent`); removing the
   refusal guards reddened `test_over_long_values_are_refused_not_truncated`; setting
   the field back to 24 is a **compile failure** via the static_assert; shrinking the
   router's `GET` reply buffer reddened the read-back assertion. The walk
   (`test_every_published_key_and_max_length_value_reaches_the_parser`) checks every
   published key's own maximum-length value rather than the one key under review, and
   fails if it examines too few keys.

   One hole the probing opened that had nothing to do with the queued item:
   `Registry::valueAsString` returned a truncated render **and reported success**, so
   `GET AP_PASS` on a too-small buffer would have answered half a passphrase. Nothing
   in the suite had ever asked, which is why the fix came with the test
   (`a 5-byte buffer reported an 8-character passphrase as rendered`) rather than the
   other way round.

   On the wire, 2026-11-24, on the flashed build: the matrix reads `AP_PASS`, writes 40
   characters, reads it again, and `grade_value_caps` grades two things — the refusal is
   `ERR value too long` (**not** `ERR line too long`, which would be the 64-character
   LINE cap answering and the field cap never reached) and the two reads return the same
   passphrase. 2 checks, 8 lying-capture cases. Gate: **141 checks 0 failures**, native
   **262**, `web_probe.py` **62**, `--selftest` **62**. Headroom was the thing to watch,
   since `Command` sits on the uart2 task's stack: `u2stk=` bottomed at 3088 in this run
   and 3088 in the run before it, `stk=` at 1040 in both — the 16 extra bytes are
   invisible against the 512-byte floor, and that is measured rather than assumed.

   The bench leg is built entirely out of **refused** writes — `GET AP_PASS`,
   `SET AP_PASS=` + 40 characters, `GET AP_PASS` — so the access point is never
   re-armed and no later phase waits on a station rejoining. Writing a legal 24..31
   character passphrase on the bench would churn the radio twice for no extra
   information: the truncation lives in `proto::parseCommand`, which the handset and
   web transports share, so by this project's own rule ("the bench exists to prove the
   wire, not the logic") that half stays on the host. What the wire leg does prove is
   the distinction `grade_value_caps` insists on: `ERR line too long` (the 64-character
   LINE cap) is **not** an acceptable answer, because it would mean a different rule
   refused and the field cap was never reached — the same "right `ERR`, wrong reason"
   trap as the dive-lock probe one section earlier.

- **A dive row's digits and the wire's digits were one count apart, at nine sites
  (2026-11-24, 16).** The queued note called it "`votedX1000` truncates" and said no check
  depended on it. Both halves need correcting. Every number in a dive row is stored as an
  integer count — centimetres, thousandths of a bar, millivolts — and printed from that
  integer, so the digits are decided at the **store** step, and each of nine sites did its
  own arithmetic (`(int32_t)(depthM * 100.0f)`, `(int16_t)(ceilingM() * 100.0)`,
  `(uint16_t)(vbat * 1000.0f)` …). A cast truncates toward zero and a decimal times 100 or
  1000 can land under the integer it stands for. Swept on the host across the ranges the
  firmware stores, values that reached the file one count low: ppO₂/cells at three
  decimals **10 of 1551** (0.251 bar → 250); depth in centimetres **266 of 4001, 6.6%**
  (0.53 m → 52 cm); **ceiling 1922 of 4001, 48%** — that site multiplied by a **double**
  literal, which promotes the float and keeps the shortfall (0.01 m → 0); battery 9 of 701
  (16.05 V → 16049 mV). Setpoints at two decimals: 0 of 13, because **a float multiply
  re-rounds** the product back onto the integer. That is the whole reason this survived:
  every spot check anyone ran used a value the float path rescued.

  Worse than a truncation, it was **two rules in one file**: the dive header's `sp1..4`
  columns used `+ 0.5f` while the `S,` rows' `setpointX1000` did not, so one dive file
  could carry 550 in its header and 549 in its rows for the same setpoint. The same
  quantity, two answers — the depth-scale pattern in miniature.

  Fixed structurally rather than site by site: `numfmt::scaled(v, d)` is now the only way
  a physical value becomes a stored integer, `d` being the decimals the field prints with,
  and it routes through the **same** `roundExact()` as `numfmt::fixed()` — nearest, ties to
  even, decided on the exact product via `fma`. The invariant is
  `fixedScaled(scaled(v, d), d) == fixed(v, d)` byte for byte, which is exactly the claim a
  diver can check: his dive file says the number his handset was answered. `lround()` is
  forbidden by name in the suite because it breaks that on a tie (`lround(0.5)` is 1 while
  the wire prints 0).

  Witnesses, all mutation-proved: a literal table (30 hand-worked counts); the agreement
  walk (fixed versus stored-then-printed, with a floor on pairs compared and an explicitly
  counted exclusion for printf's `-0.00`); the bare-cast case that pins what the old
  expressions produced (2029 cm for 20.30 m, 1114 cm for an 11.15 m ceiling, 289 thousandths
  for 0.29 bar through a double) **and** what they did not do (0.15f through a float
  multiply is exactly 150 — my first draft claimed otherwise and the test caught it); a
  source scan that fails on any `* 100`, `* 1000` or `+ 0.5` left in a file that stores
  values, naming file and line, proven to fire for a **newly added** hand-rolled field —
  the case it really exists for — plus a floor on how many `numfmt::scaled` sites the scan
  found, and a check that `numfmt.cpp` still holds one rounding engine rather than two.
  Injecting each bug reddens its own cases: a reverted site names `src/main.cpp:593`;
  `scaled()` truncating reddens seven cases including the row formatter
  (`Expected '20.30' Was '20.29'`); `std::lround` reddens the agreement walk, the tie cases
  and the non-finite guard (it yields `INT64_MIN` where the helper must yield 0); `fixed()`
  growing its own rule reddens the differential sweep against `printf`.

  `test_divesoak`'s rig was part of the blindness: it re-implemented the same casts line
  for line, so the longest-running dive suite on the host mirrored the defect instead of
  witnessing it. The rig now calls the helper, and the witness is a literal one in the
  format suite — `test_a_row_holds_the_digits_the_wire_gave_for_the_same_sample`, which
  formats a real `S,` row and asserts `,20.30,0.290,` and `1115` appear and `,20.29,` never
  does.

  Deliberately **no bench leg**, with numbers rather than a shrug: comparing the handset
  export with the web export cannot see a scaling error (one formatter, and a round-trip
  cannot catch a scale error — a rule already recorded here); comparing the file's
  centimetres with `T,`'s compares two different samples (a 30 s grid row against a 2 Hz
  stream) and this bench has no surface tare, so a cm-level match sits inside its noise;
  and of the values this bench can produce the old casts kept nearly all of them (0 of 13
  setpoints, 0.6% of three-decimal cell readings), so a leg would have been green on the
  broken build. Host-witnessed, and the sweep is why.

- **Which chip produced the depth is now a question the wire can answer (2026-11-25,
  16).** The queued version of this item asked for "a depth-source field (`ds=`) so a
  reader of `HB` or a dive header can tell which sensor produced a depth". The answer
  landed as a **query instead: `GET DS`** — deliberately, and what that gives up is
  recorded rather than hidden. Appending `ds=` to `T,` costs bytes at 2 Hz on the
  in-water link and asks every existing client to re-validate a row shape it already
  consumes; the facts it would carry change at most once per boot (the arbiter awards
  the depth slot by identity at power-up and never re-awards it — `depth_manager.cpp`
  is explicit that a faulted source is re-initialised by its own driver but the slot
  is not re-given), and the boot announce already states the choice once. So:
  `DS,<locked>,<used>,<healthy>,<age_s>,<identity>`, answered on both transports, with
  the boot announce left exactly as it was. **What this gives up: a dive file still
  cannot say which sensor its depths came from.** On a board fitted with both chips, a
  logged dive carries no provenance. If that ever matters, the field goes on the right
  of the dive header — the append-only rule already allows it — and not on `T,`.

  The field that makes the command worth having is `used`, not `locked`. A sensor can
  own the slot, be identified and be healthy while its readings stop arriving; the
  controller then keeps printing the last good depth, and `T,` looks identical — there
  is no stale-sensor bit in its FLAGS byte either. So `locked=1, used=0` is the state a
  handset must be able to see, and it is now the state the client documentation tells
  builders to look at first ("read `used`, not `locked`", `handset/README.md`).

  The snapshot lives in `bridge::DepthSource`, published by SensorTask, and the router
  reads the copy rather than the arbiter. Two reasons, one of them measured. (1) The
  router is deliberately hardware-free; host tests drive the whole command surface with
  no bus anywhere near it. (2) `DepthSensor::identity()` hands out a pointer into the
  driver's own `char[24]`, and that array is **rewritten** — `noteFail` puts a locked
  driver in `BUS_FAULT` (`lps28.cpp:88`), the arbiter keeps calling `begin()` on it
  (`depth_manager.cpp:8`), and the re-probe snprintfs `identity_` again
  (`lps28.cpp:45`; same in `ms5837.cpp:108`). A command task reading it live can print
  a prefix with no NUL in it. Five fields copied without a lock cannot be made atomic —
  the unlocked→locked transition and a fault-zeroing want opposite write orders — so the
  invariant is enforced at the reply instead: **`GET DS` prints a name only if `locked`
  vouches for it**, which makes the misleading pair (a name beside a flag claiming no
  source) unrepresentable on the wire. What remains is a window of one sensor cycle at
  most, at most once per boot, never mid-dive.

  The rules that decide the snapshot are a function, `bridge::publishDepthSource`, not
  assignments at the call site — because the call site is in `main.cpp`, which the
  native build excludes, and a rule living there is a rule no test can see. That was
  not a preemptive design choice: the first mutation run injected an unbounded copy in
  the publisher and **nothing reddened**, because no test called the function at all.

  Witnesses, all proven by injection: **seventeen injections, seventeen reds**
(11 in the firmware's reply path and snapshot publisher, 6 in the client library). Host: the command answers
  even with nothing wired (`DS,0,0,0,0,none`, not `ERR unknown key`); the reply is the
  published state (`DS,1,1,1,15,LPS28DFW @0x5D`); `used` is not an alias of `locked`;
  age is seconds, from the router's own clock, and is 0 for a source that never
  answered rather than "the whole uptime"; a publisher that leaves an unlocked slot
  inconsistent still cannot name a chip it does not own; the client's parser refuses a
  truncated line instead of guessing (and the rejected line must reach `onLine`, not
  vanish); `depthSourceText` returns nothing rather than half a sentence on an OLED.

  One of those witnesses had to be rebuilt, and the reason is worth keeping as a lesson
  next to "a round-trip cannot catch a scale error": the test planted a sentinel after
  the snapshot array and asserted the answer never contained it. Injecting an unbounded
  `strcpy` left it **green** — with a destination sized exactly like the source, the
  copy is stopped by whatever stale zero happens to sit at the end of the local, so the
  printed field is correct and the overflow is invisible to everything except a later
  crash. The destination is now roomier than the source on purpose, so an overrun shows
  up in the answer. A check that cannot be the only witness is decoration, and this one
  was decoration until it could fail.

  Bench: the matrix asks for the shape, `grade_depth_source` grades the meaning against
  the boot announce — same chip on both transports, an identity the gates accept, the
  slot owned, `used` true on a reading bench, `healthy` true, and an age that cannot
  contradict "in use" — with 13 lying-capture cases in `--selftest` (different address
  than the console probed, an `LPS22HB @0x5C` handed the slot, locked with no name,
  depth held, `age_s=900` beside `used=1`, a truncated line, `ERR unknown key` from
  older firmware, and a leg that never ran). `web_probe.py` adds the cross-transport
  comparison with **one field excepted and the exception named**: `age_s` is seconds
  since the last good read and the two sends are ~0.4 s apart; the name, address and
  flags must be byte-identical. Proven by flashing? No — the graders are proven on
  lying captures, which is the honest description of what `--selftest` is.

  Protocol-visible, so one commit across spec + client + example + README + conformance
  (spec §3 table, §4 line format with the field table and the "read `used`" rule, §7
  rule 6 — a command an older controller lacks answers `ERR unknown key`, which is
  *"cannot say"*, not *"no sensor"* — §8 limits, including that this says **where** the
  depth came from and not that it is accurate, since `CAL DEPTH` is still unimplemented
  and the surface offset measured here is ~0.04 m). Native 273, `--selftest` 75 lying
  cases. Bench after flashing: `hw_accept.py` **149 checks 0 failures** (141 + the
  matrix shape row + 7 graded depth-source claims), `u2stk=3096` (was 3088),
  `stk=1040` unchanged; `web_probe.py` 64 checks **1 failure** — that failure is
  the dive-identity grader picking `max(slot)` instead of the newest dive and
  demanding a date from a dive that honestly has none: unrelated to depth, closed
  in the very next commit.

- **The radio is a policy now: intent, holders, and one line that explains both
  (2026-11-26, the closed M5 criterion).** WiFi auto-off was the open M5 clause, and
  it grew into the state machine a diver actually needs, because "the AP died by
  itself and the phone cannot ask why" is the ack-with-no-consumer family again:
  **`radio = diver intent AND NOT (DIVE | DEPTH | IDLE)`**, all of it inside
  `CommandRouter` — where `wifiOn_` already lived — ticked on **loopTask under the
  session lock** that serializes every `exec()`, never from ControlTask (the
  telemetry_bridge rule: control must not wait on the task that answers clients).
  `wifiEnabled()` stays what `WIFI ON|OFF` changes: the **intent**.
  `wifiRadioWanted()` is what the transport **obeys**. The answer on the wire is
  `GET WIFI` → `WIFI,<on>,<held_by>,<idle_left_s>,<stations>`, `held_by` an
  append-only enum (0 none / 1 diver / 2 dive / 3 depth / 4 idle), and `stations`
  is there because of the bench, not of design taste: the first idle-leg run saw
  the countdown pinned at exactly the full window forever and nothing ever fired —
  the rule was **correctly refusing to fire**, because the bench PC's own `wlan0`
  had autojoined ESP-CCR from an earlier web_probe. A policy input nobody can read
  makes a working rule indistinguishable from a dead timer; publishing the input is
  what makes both states legible to a client, the same reason `used` exists on `DS,`.
  The thresholds are four settings keys (the operator asked): `WIFI_IDLE_MIN`
  (**0 = never**, default 10), `WIFI_OFF_DEPTH` 2.0 m, `WIFI_ON_DEPTH` 1.0 m and
  `WIFI_ON_HOLD_S` 60 s — names, ranges and the 32→36 key count are in spec §6. The schema cannot express a
  key-vs-key bound, so the guard against `WIFI_ON_DEPTH` above `WIFI_OFF_DEPTH` is
  a sentence in the settings table plus a test: the inversion **oscillates by
  design** — the firmware does not silently clamp it (silent correction is a
  settings lie, the truncation family again).
  The rules that pinned themselves under test before they were written down: auto
  never overrides a diver's `WIFI OFF`; DEPTH is cleared by depth and by nothing
  else (at depth the words set intent and raise **no** radio action); IDLE is
  cleared by intent events or by another holder taking over, never by itself (a
  self-releasing hold flaps the AP once an idle-minute forever); **stale/held depth
  — `GET DS`'s `used=0` case wearing a hard hat — freezes the DEPTH latch**, and
  the straight-shallow timer starts when belief resumes, not at the last believed
  sample; and the **idle window starts when the radio becomes up, before any fire
  check** — a fire evaluated against the pre-dive window would have greeted a
  surfacing diver with "idle" instead of an AP after a 14-minute dive. The first
  two of those were firmware bugs the tests caught, not design notes: priming
  swallowed a boot-at-depth transition (the first tick is where that down happens,
  and suppressing first-tick changes would leave the AP up at depth forever), and
  `SET WIFI_IDLE_MIN=0` released only the *next* hold, leaving the radio down under
  a rule that says "never sit down".
  Behaviour changes, named because they change what a client sees: **`DIVE END` at
  depth restores the intent, and DEPTH keeps the radio down until the diver is
  actually shallow for `WIFI_ON_HOLD_S` straight** — ending a dive at 8 m used to
  raise the antenna at 8 m, which is the exact milliamps the dive-drop exists to
  save (at the surface the restore is unchanged, so every older dive-cycle check
  stays honest); and the console line `web: radio off (dive)` lost its `(dive)`,
  because the teardown no longer always means dive — the policy prints the one
  reason-bearing line per transition (`wifi: radio down (depth)`).
  Mutation battery, twelve mutants: ten reddened their own case (inverted
  off-comparison, dropped stale-freeze, band not restarting the straight timer,
  self-clearing idle latch, window not started on up, `IDLE_MIN=0` latch kept,
  `applyWifi` raising ACT on *intent* change, priming comparing post-update state,
  `endDive` always raising ACT, SAFE term dropped from `wifiRadioWanted`). Two
  stayed green **and stay in**: `uart_link` obeying `wifiEnabled()` instead of
  `wifiRadioWanted()`, and the redundant idle-clear inside `startDive`. Not
  decoration — **structurally unobservable**: `handleLine` holds the session lock
  across the diff and the transport's read, and holders only change inside the
  tick, so intent ≡ effective at every point where `ACT_WIFI` can fire; the tick
  clears the idle case before anything observable could happen. Belt-and-braces
  lines, and the ledger says so rather than letting them pose as tested rules.
  Two harness defects fell out before the batch went green, both the same family:
  the matrix's first-match window **provably truncated `GET *`** once the table
  reached 36 keys with `WIFI_*` at the end — the ruler grader was reading an
  amputated line list (GET * now has a `collect` window), and `grade_wifi` read
  `pair[0]` where a previous answer's leaked tail can sit — scan the pair, don't
  read positionally. And the idle leg reddened an **honest board** on its first bench
  run: it timed the one-minute window from the `SET`, while the firmware restarts the
  window at the intent event seven seconds earlier — the fire was exactly the setting
  when measured from the right clock. Anchor where the machine rewinds, not where the
  harness happened to type.

  Bench after flashing: `hw_accept.py` **170 checks 0 failures** (idle leg witnessed on
  hardware: fire at 66 s from the intent event, blamed on idle by number, console line
  matched at both transitions, setting restored to 10), `web_probe.py` **71** with the
  fourteenth cross-transport comparison, native **278**, selftests **107 + 20** lying
  cases, mutation battery **12/12 red** on their own cases (plus the two recorded as
  structurally unobservable).

  **The depth leg is bench-witnessed (2026-11-26, operator's thumb at the bench):
  8/8.** Squeeze past 2.0 m → `WIFI,0,3` and console `wifi: radio down (depth)`;
  release; **still down at shallow+20 s** (the flap-catcher); back at exactly
  **60 s**, no sooner, within the 55–75 s band; console `wifi: radio up (clear)`
  and the AP line. Witnessed through a standalone driver importing the module's
  OWN `run_wifi_depth`/`grade_wifi_depth` — code under test identical to the
  gated path; the full in-matrix `--wifi-depth` run has still never been driven,
  and that stays recorded rather than glossed. Evidence: `wifi_depth_leg8.log`
  plus its transcript (`wifi_depth_leg8_transcript.log`): 101 depth samples,
  the flat-0.00 drain, both console transitions.

  The three runs before the green one earned their keep. **(a)** The leg polled
  depth through `_ask` — which DROPS the unsolicited `T,`/`D,` stream — so the
  release half had never parsed a single depth: run 1 even passed the squeeze
  half off an unrelated `E,` row's timestamp. No host selftest could ever catch
  that: the selftests grade the SCORER on synthetic caps, not the poller that
  produces them, and this poller only ever ran on hardware. New `_ask_depth`
  reads `T,` lines; the witness then matched what the operator had SEEN with
  his eyes — the radio genuinely dropped and returned all along; the policy was
  right and the observer was blind. **(b)** An early squeeze tripped the latch
  before the leg started, and the pre-check aborted rather than run the
  down→up latch from an already-down state; the board then rode the full
  cycle unaided (down, held, back 60 s later) — an unprompted witness of the
  release path. **(c) One open observation, recorded not asserted.** The third
  attempt vented cleanly — flat 0.00 m — yet stayed `WIFI,0,3` at shallow+65 s,
  and a probe **45 minutes later** found the same hold with `DS,1,1,1,0`,
  fresh at 0.08 m; the `T,` flags gained sensor bit 0x08 at the instant depth
  jumped 3.46→0.00 m in one 0.5 s sample (~7 m/s, physically impossible for a
  diver), and the bench wire came loose under exactly this kind of handling
  around the same session. A power cycle cleared it; every run since released
  normally. Not root-caused, not reproduced on demand. The candidate (a
  hypothesis, not a finding): flapping depth samples ≥ `WIFI_ON_DEPTH` restart
  the straight-shallow timer forever — the hover rule, working as designed
  against a broken wire. **Ledger says: observed once, cleared by power cycle,
  watch for a reproduction.**

- **`PULSE` is dead: the handset has no manual injection path (2026-11-26).**
  The decision came from the diver side, not from a bug: adding diluent by hand
  is an **analog** act — physical, a bypass of the loop, done by hand in the
  real sense — and anything the handset triggers mechanically must come from a
  clear decision the firmware itself made: the pre-dive solenoid step, `TEST
  CELLS`, closed-loop control under AUTO. A manual-pulse button is neither, and
  it was *protocol*: a spec §3 line, and a §8 note telling handsets `PULSE`
  works mid-dive and to "keep it off an unguarded button." That second half was
  the spec admitting it managed a hazard instead of refusing one. So the verb is
  gone, not disarmed: it no longer **parses**, and the answer is
  `ERR unknown command` — the controller saying truthfully that it has no
  manual pulse path. That deliberately differs from the CAL family, which keeps
  parsing and refuses `ERR not implemented`: those calibrations have real
  consumers waiting; this has none, by decision. `Kind::PULSE` is removed from
  the enum rather than left as a stub — an enum value nothing refuses is an
  enum value something eventually re-wires. `Loop::postTestPulses`,
  `Pending::pulses` and the dead `Loop::requestTestPulse` wrapper are gone; the
  injector's `testPending_` mechanism **stays**, because the sequencer that
  uses it is exactly the kind of decision the handset is allowed to trigger.
  `pulse()` is deleted from the reference client so a new handset cannot grow
  the button, and because the older firmware answered `PULSE` **mid-dive**, the
  bench pin sits in the dive half of the hw_accept matrix: a handset built
  against the old spec meets the refusal at the exact moment it used to be able
  to open the valve.

  Regression coverage moved rather than vanished. `test_deferred`'s pulse case
  pinned plumbing that no longer exists, but the regression inside it — *the
  leftover-cancel rule must not eat a pulse between request and valve* — now
  has only the engine's own pulses as producers, and the rewritten case drives
  a standalone `TEST CELLS` to pin it: exactly the 3 pulses the test asks for
  reach the valve, and the valve stays quiet after the test ends (three 300 ms
  pulses spend 900 ms of the 1000 ms duty budget, so a leftover would fire
  inside the next ticks, not hide under the cap). The edge-triggered cancel in
  `loop.cpp` keeps its comment — and the same reason.

  The ledger lesson, which this project has now paid for twice: **a verb's own
  `OK` is never a witness.** This firmware once had a `PULSE` that replied `OK`
  and never fired the valve (the unconditional-cancel bug, recorded above), with
  the host suite green — only the injector's delivered-pulse counter told the
  truth. Hence the retirement pins a refusal, and the surviving pulse path is
  pinned by a delivery count, never by the reply of the thing that asked. The
  cancel rule got its own leg too: a second throttled run (10 s min-gap) leaves
  the engine's verdict landing with one pulse still queued, so a `cancelTestPulse`
  that stopped cancelling is a red test, not a comment. Mutation battery for the
  package: re-armed verb reddens the parser and the router, unconditional cancel
  reddens the fast leg (the original bug, `Expected 3 Was 0`), removed cancel
  reddens the throttled leg — three mutations, three reds, on their own cases.

  Bench after flashing (and after C2-g in BENCH.md: an unnamed `upload` had put
  the image into the spare, which fakes an I2C catastrophe — read that entry
  before trusting any bench mystery): `hw_accept.py` **171 checks 0 failures**
  on the controller itself, the new row witnessed mid-dive — `PULSE` answered
  `ERR unknown command` exactly where the old protocol opened the valve — idle
  leg, web, depth, dive-persist all green, `stk=1040 u2stk=3088` unchanged from
  the radio-policy board, `web_probe` **71 / 0** and selftests **107 + 20**,
  native **278**.

- **Queued next, none of it waiting on hardware** (2026-11-24, after the
  calibration batch; the surface log is now done, the rest still is not):
  the small known lies: ~~WiFi idle auto-off~~ (an open
  M5 criterion — **closed by the radio-policy entry above**, 2026-11-26); ~~a
  `PULSE` button on the page~~ — **closed by retiring the verb instead**, same
  decision as the entry above: the page and the handset both get no manual
  injection at all, so the queue item that wanted the button is void. The
  page's command set stays AUTO/MANUAL/ACK. The depth-source item that used to open this list closed as
  `GET DS` — the entry above records why a query and not an appended field.
  Added by the dive-file batch: the page's dive view **ignores `E,` rows** — it
  renders the chart from `S,` and one hint line from `M,`, so the verdicts are in
  the file and in the download but a surface reader on the AP never sees that the
  proof failed or that the diver lifted a clamp. Rows, or at least a count, would
  close it; it changes no byte on any transport. — **closed 2026-11-26 by the
  dive-verdicts entry below.**
  Closed since, and understated when written: the `votedX1000` truncation was a family
  of nine store sites, the ceiling site lost a centimetre on **48%** of values, and the
  fault was never "the two transports disagree" (they share one formatter, so they always
  agreed) — it is that the file disagreed with the **wire**.
  The depth batch's queued item — the **depth-scale pin test** — is done, same
  day, and what it turned up is recorded above rather than here. The M5 clause
  "WiFi auto-off" closed with the radio-policy batch above; that batch added one
  queued item of its own: ~~the status page should surface `GET WIFI`~~ —
  **closed 2026-11-26 by the status-page radio entry below**: the row is on the
  page, graded by four new web_probe checks and by lying captures against the
  repo's own page text. What remains open in this list is the dive view's
  ignored `E,` rows.

- **The page can now say why its own AP vanished (2026-11-26).** The radio
  policy answers `GET WIFI` and the console prints a reason line per
  transition, but the one client whose connection *is* the radio — the surface
  page — had neither. Added: a **radio** row on the status card, polled like
  every other status line (`GET WIFI`, 2.4 s, status view only), rendering
  `WIFI,on,held_by,idle_left_s,stations` in words that match the console's
  transition line (`down · depth`, `down · diver's WIFI OFF`), and a station
  count while up. One rule took a read of `cmd_router.cpp` to get right and it
  is the whole difference between a status line and a lie: **the idle countdown
  is shown only while `stations == 0`.** The router rewinds the idle window
  *every tick a station is associated* (`cmd_router.cpp:424`), so a connected
  reader shown "sleeps in 10:00" is being shown a timer it is itself cancelling
  — a countdown that can never fire. A page can almost never see that state
  (its own socket is a station), which is exactly why writing the row without
  reading the policy would have got it wrong.

  Four new web_probe checks grade the served page: it must ask, must read the
  fields rather than print the raw line, must gate the countdown on
  `stations==0`, and must render the hold reason as a word — both halves, word
  list *and* its use, because a word list nobody renders is as mute as a
  renderer with no words. The graders were then made host-gradable
  (`grade_page`/`selftest_page`) and made to lie at themselves: four lies,
  four reds on their own cases — and the first battery caught a decoration of
  my own making, a check that keyed on the word list only and stayed green
  when the lie deleted the *rendering*. `--selftest` **28 cases**, probe
  **75 checks 0 failures** with the two transports still agreeing on `GET WIFI`
  byte-for-byte except the two fields whose motion is named.

  Found and fixed on the way: `web_probe` crashed with a `KeyError` on `/` when
  the board was probed still-coming-up after a flash — the summary line reached
  for the page before the check list could print the fetch failure it had
  already recorded, turning a reportable condition into a traceback. The
  summary now defers to the FAIL line. Gates: native **278**, hw_accept
  **171 checks 0 failures**, `stk=1040 u2stk=3040` — the uart2 floor moved 48
  bytes deeper than the PULSE board while `stk` is unchanged; the page lives in
  progmem and adds nothing to any stack, so this is linker layout, recorded
  rather than waved at, and the next board run tells us whether it is stable.
  (It was: the dive-verdicts board run below reads `stk=1040 u2stk=3088` —
  exactly the pre-page numbers. Layout, as recorded, not a leak.)

- **The dive page stopped hiding its own verdicts (2026-11-26).** The dive
  file has always carried `E,` rows — alarms, cell-proof FAIL, clamp lifts,
  calibration outcomes — and the CSV download has always shown them, but the
  page's dive view parsed only `S,` (chart) and `M,` (hint line) and dropped
  every verdict row on the floor. A surface reader on the AP could watch a
  chart of a dive in which the proof had failed and see nothing about it. The
  queue item asked for "rows, or at least a count"; rows won, in file order:
  the dive-file grader already pins the file's `E,` sequence against the
  wire's, so the page may render "the failure, then the lift" without sorting,
  and the words come from `eventName` — the same map the live toasts use — so
  one event never has two vocabularies depending on whether the diver is
  watching it now or reading it an hour later. A dive with zero verdict rows
  says so out loud: the absence of rows is a fact a reader needs, not a blank
  to be guessed at (the PASS-line rule runs the other way too: a quiet render
  must not imply a quiet dive).
  Three new web_probe page checks — parses the rows, words them through the
  shared map, renders the empty case as words — each with its lying capture
  in `selftest_page`: three lies, three reds. The greps are written against
  the DIVE-side code deliberately: the live stream has an `E,` handler too,
  and a check that keyed on it would have been green all through the years
  the dive view was dropping the rows — the decoration this family keeps
  finding, a check that witnesses a different site than the one that broke.
  Gates: probe **78 checks 0 failures** with the dive cycle, `--selftest`
  **37 cases**, hw_accept **171 / 0**, native **278**, `stk=1040 u2stk=3088`.

- **The sim's cell noise now matches the board, and the witness baseline is
  re-cited with it (2026-11-26, `d0b60cf`).** The sim's healthy cells dithered
  ±18 ADC counts — five times the board. The bench HB line (which prints
  `g_ads.raw()` directly) showed three healthy cells AT REST IN AIR wobbling
  4..7 counts peak-to-peak, and after the frozen-reading detector that
  amplitude stopped being cosmetic: a live channel dithering a few counts every
  sample is the ONLY thing separating it from a channel whose reading froze, so
  a sim five times noisier than the board flatters the detector. Corrected to
  ±3 quantized counts (`sim/world.cpp` carries the arithmetic: PGA=16, LSB
  7.8125 uV, 0.21/1490 = 1.41e-4 bar/count), and the seeded pool re-ran: **23
  of the 54 witness lines that existed then moved — 22 of the 50 seeded dives,
  plus the 9002 `ppO2-view` line — and dive 9003's pair was added, for the
  current baseline of 56 lines.** Attribution by control run, not assertion:
  old voting engine + new sim reproduces exactly those 23 moves, and the
  engine-attribution diff between control run and final run is additions-only.
  The moved fields, old → new:

  | dive | move |
  |---|---|
  | 2  | snaps 248→247, ppO2 top 1.44→1.43 |
  | 4  | snaps 491→492, ev 7→5, pulses 1083→1084, ppO2 top 1.47→1.48 |
  | 5  | ppO2 top 1.03→1.07 |
  | 7  | ev 7→5, pulses 176→177, ppO2 top 1.52→1.51 |
  | 8  | ev 11→9 |
  | 9  | ev 7→5, pulses 750→757 |
  | 11 | ev 15→11 |
  | 12 | snaps 177→178 |
  | 13 | pulses 601→600 |
  | 14 | snaps 187→188, pulses 446→443, ppO2 top 1.47→1.48 |
  | 16 | ppO2 top 1.07→1.08 |
  | 18 | snaps 471→472, ev 7→3, pulses 1030→1032 |
  | 19 | ev 4→2, alarms 0x204→0x200 |
  | 20 | snaps 250→251 |
  | 23 | snaps 306→307, ppO2 top 1.30→1.31 |
  | 25 | ev 7→3, ppO2 top 1.07→1.08 |
  | 26 | pulses 77→79, ppO2 top 1.70→1.60 |
  | 27 | ev 6→4, ppO2 top 1.61→1.63 |
  | 35 | snaps 175→176, ppO2 top 1.08→1.07 |
  | 46 | ppO2 top 1.03→1.07 |
  | 47 | ev 7→3, pulses 497→498 |
  | 50 | snaps 456→457, ev 7→5, pulses 1015→1017, ppO2 top 1.47→1.48 |
  | 9002 | rows hold 0.989→0.990 |

  Why each kind of move is acceptable: snaps and pulses shift by 1..7 because
  the 30 s grid and the injection schedule sample different noise on the same
  dive; `ev` (alarm-edge count) DROPS on eleven dives because the old noise
  crossed alarm thresholds on blips the bench does not produce — and the alarm
  union itself moved on exactly one dive of the fifty: dive 19 `0x204→0x200`,
  where the phantom `BIT_CELL_DRIFT` was one of those blips and the dive's own
  fact, diluent-flush `BIT_HYPOXIA`, stands with its asserts untouched. Dive
  26's true-loop top 1.70→1.60 is the largest single move: the stuck-open
  runaway now gets caught ~0.1 bar earlier, because the HIGH alarm lands on a
  different sample — the same rule, less twitch. The 9002 held-ppO2 quote went
  0.989→0.990 because the last believable vote is itself a median of dithered
  cells; the `be772aa` commit message citing 0.989 is history text, not a byte
  to chase. What did NOT move: 28 seeded lines byte-identical, the 9001 pair,
  the 9002 `wet-connector` line — and every assert the suite runs on a moved
  dive (detection delays, the 0.12/2.20 bar envelope, alarm pairing, the held
  windows) is the same assert, unchanged. Two claims that cite specific dives
  were re-verified against this baseline before accepting it: the wall/alarm
  entry above (dives 26/42 still redden the suite, now reading 1.59/1.60), and
  the `bda1825` dive-4 starvation quote, kept as the as-found record and
  annotated as such. The stale noise multipliers that the old ±18-count sim
  justified were re-cited with it: `cellproof.h`'s comment still said
  "±0.0025 bar... ten times" — the fix `d0b60cf`'s message claimed for that
  comment had in fact not landed, and now it says what the sim does (±3
  counts); this section's `trackMinBar` ratio moved from ten to over fifty.
  Baseline gates: native **333**, the 56 lines byte-identical run-to-run, and
  `d0b60cf` bench-accepted on the board at **140 checks 0 failures**
  (`--cal-air --predive --expect-depth --expect-web`: surface records 138/139
  joined console↔wire, `DS,1,1,1,0,LPS28DFW @0x5D`, `stk=1120 u2stk=3088`);
  the bench HB dither that set the epsilon, c0=1534..1541, is the same
  4..7-count wobble the sim now writes.
