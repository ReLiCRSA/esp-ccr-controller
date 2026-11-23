# ESP32 Nitrox CCR Controller — Master Plan

Rebreather controller on ESP32: 3× R22D galvanic O₂ cells on an ADS1115 (PGA=16),
depth sensing, ZHL-16C decomputation, handset over UART, web UI over a private AP,
and a PC simulator to validate the control/deco logic before hardware trust.

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
| Battery | Full support: divider sense on ADC1, alarms, pre-dive check, per-snapshot logging |
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
│   │   └── battery.cpp       # ESP32 ADC1 + divider, smoothed, calibratable
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
│   │   └── web/              # ESPAsyncWebServer + WebSocket + LittleFS pages
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
| Battery sense | 35 | **ADC1 only** (ADC2 collides with WiFi); divider 100k/10k, `VBAT_CAL` trims |
| ADS1115 DRDY (optional) | 18 | supported by driver; `-1` disables (bench today) |

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
  visible. `PULSE` handset command = single manual test pulse.

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
     → BATTERY ≥ min
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
`CELLPROOF OVERRIDE` · `PULSE` · `DECO` (one-shot `D,` line) ·
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
| **Status** | Big ppO₂ + target, 3 cell bars + health, depth, mode, ceiling/NDL/deco ladder, CNS, battery, clamp status |
| **Histogram** | Per-cell ppO₂ histograms (~60 bins, 0–1.9 bar, ~15 Hz accumulation since power-on) — the pre-dive cell-health picture; side-by-side desktop, tabbed on phone; canvas, no chart lib |
| **Settings** | Grouped accordions; same registry as handset; dangerous actions behind confirm dialogs |
| **Checklist** | Pre-dive stepper + calibration wizard (`CAL O2` sequence live) |
| **Dives** | 10-dive list → detail with inline depth/ppO₂ charts → CSV export |

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
| Battery | `VBAT_MIN` · `VBAT_CAL` |
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
(`depth: LPS28DFW @0x5C ready …`) — which sensor dove is part of the
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

### Current work order (between milestones, while the 24 h soak owns the port)

Nothing below is flashed or bench-tested until the soak ends (~09:45 on
2026-11-23): logic, host tests and compile-only firmware work only.

| Step | Deliverable | State |
|---|---|---|
| A1 | `ControlTask` runs the real `ctrl::Loop` on the board (compile-only during the soak) | done `1b921b4` |
| B1 | 50-dive seeded sim soak as the regression net for all logic work | done `bda1825` |
| A5a | one command router for handset and web — refusals are the surface | done `f247d30` |
| A5b | one settings applier (in-place re-tune) + NVS + dive-file store | done `b322bc0` |
| A5c | handset session on UART2 | done `0e681e1` |
| A5d | web AP + `/ws` bridge + built-in page + dive CSV | done (this batch) |
| A5e | close the control-vs-command race: defer chain mutations (`setConfig`, `startDive`/`endDive`, pre-dive, pulse requests) into ControlTask's cycle boundary, so the handset/web task never mutates the chain mid-`cycle()` on the other core; host test | done (this batch) |
| C2 | bench acceptance: `tools/hw_accept.py` (boot checks + refusal matrix over a wired handset; `--replay` validated against the soak log) + `BENCH.md` sequence, then flash the current build | **boot/console half PASSes on the board**; web half verified over the air from the PC (AP `ESP-CCR-3828`: captive DNS, batched replies, 180 s / 178 replies / 0 socket drops). Reflash chain: `1b8c807` (tag `m2-flashed`) → web-fix build (tag `m2-web-fixes`). **Both halves PASS on the flashed build: 30/30 checks, 0 failures** (boot console + the refusal matrix over the spare-ESP32 handset bridge, now including `GET T`/`GET D` proven as real replies by cadence anchoring, `GET *`/`GET SCHEMA`/`GET HIST`/`GET FAULT`, and a console watcher that fails the run on any panic, SAFE latch or lost stack headroom). The link paid for itself immediately: the first `GET T` over it panicked the controller (uart2 stack overflow — §16), fixed structurally (`util/numfmt.cpp`, no by-value reply buffers, `stk=`/`u2stk=` in HB) together with the SAFE-mode `ACK` the boot log had been promising but never implemented, now verified on hardware (`ACK` → `OK SAFE RESTART` → clean boot). Still open for C2: `LOG EXPORT` vs `/dive/<n>.csv` row-shape mismatch (§16), and the depth/`CAL O2` legs that need the LPS28DFW on 19/18 and the bench jar. Note: BMP085 @0x77 (shared bus, first-ever poll — M1 never probed it) does not ACK: bench wiring question, see BENCH.md |
| C3 | MS5837 swap + potting checklist, pre-dive on real cells — checklists in `BENCH.md` | pending — hardware |
| — | per-cell histogram: `ctrl::cellhist` accumulator + `GET HIST` + Cells tab | done (this batch) |
| M4a | **ZHL-16C published cross-validation**: 25 schedule entries + NDL transitions from the published Buhlmann air tables (Deco_Package PDF) and GF100/80 cross-impl anchors from repied/paliers `src/gf.ts` run as a node oracle; fixed the M-value form (was `a + b·P`, deco inflated 2–3× — Subsurface/paliers both use `a + P/b`), the NDL definition (direct-ascent feasibility), the NOAA CNS rows (0.8→450, 0.7→570), and one inverted test assertion. Decision logged in §6: per-tissue GF interpolation kept | done (this batch) |
| M5a | **handset client package** (so a handset can be built by someone else, against a stated contract rather than by reading firmware): `docs/HANDSET_PROTOCOL.md` v1 — framing, cadence, every command/reply/refusal string, every downlink line format, the append-only evolution rules and the honest limits (§8: `SET` value truncated to 23 chars, `LOG EXPORT n` slot mapping, 32-deep `E,` ring); `handset/handset_link.h` — header-only C++17 client, transport-agnostic (UART2 and `/ws` carry the same lines), zero-allocation callback handlers, link-health watchdog (3 s silence = alarm); `handset/example_esp32_handset/` — reference handset firmware (wiring, buttons, buzzer classes, link-loss alarm); `handset/README.md`; `test/test_handset/` — 10 conformance cases feeding the library the firmware's own wire literals | done (this batch) — **183/183 native**, both esp32 builds clean. Not yet run against a physical handset: C2 refusal matrix still waits on the UART2 adapter on 16/17 |
| M5b | **handset link on the bench without an adapter**: `handset/bridge_esp32` — a spare classic ESP32 as the USB-UART (its own bridge chip for the PC, UART2 16/17 for the controller), a byte-for-byte pipe that also runs the client library as a passive observer (link/traffic LEDs = the library's verdict, not a guess), plus `-DBRIDGE_TAP=1` for a handset-side console. Bench sketches are now their own mini-projects (`pio run -d handset/<name>`), so a handset build can never produce a controller image. `hw_accept.py` settles 1.5 s after opening the handset port and retries a silent command once, because that open resets the bridge | done (this batch) — bridge and example handset both compile for the real toolchain (zero warnings); **the C2 refusal matrix still has not been run**: the bench link is built but silent, and `tools/port_id.py` (`--loopback` jumper test, boot-time UART2 self-test in the tap build) is the bisect path. Firmware side is cleared: `uart2_task` starts and `UartLink::pump` emits `T,` every 500 ms with no handshake, so the fault is in the wires |

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
  soak (dive 26/42); moving the alarm back to 1.60 makes that suite fail.
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
  being *starved*, not over-enriched):
  * the verdict is **per cell**, and every *fitted* cell is judged including one
    the voting layer dropped (requiring the vote to reach the proof level first
    lets a lying pair hide the honest cell);
  * a cell proves itself by **tracking** the demand change, not by sitting at a
    plausible number (`trackMinBar` 0.05, 10× the sim's cell noise) — a cell
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
  the same shape: defined in the log format, never published. A handset built
  by someone else would have drawn a "calibrated" tick from all of this, so
  the spec states it plainly (§3 caveats, §4 id table, §8) and the library
  refuses to invent the missing feedback. Fix on our side: give `CAL O2` and
  `CAL DEPTH` real consumers (M2/M3 exit criteria), report failure through the
  protocol (`ERR`, or emit the reserved ids as designed), and keep the reply
  honest. Small related finding: `session_wiring.cpp` declares
  `extern bool g_calAirRequest` while `main.cpp` defines it `volatile bool` —
  the linker cannot see the disagreement and the flag is cross-task, so the
  qualifier belongs in the declaration.
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
- **Two writers produce the dive CSV, and they disagree (open, found
  2026-11-23).** `LOG EXPORT n` (`divelog::toCsv`) emits 17 fields per `S,`
  row — including `mode`, `temp_c` and `flags` — with alarms decimal and
  battery in volts. `/dive/<n>.csv` (`dive_store::exportCsv`) emits 14, with
  alarms as `0x%04X`, ceiling in integer cm, battery in integer mV, and no
  mode/temp/flags. `BENCH.md` C2 step 4 asserts the download "matches
  `LOG EXPORT <n>`": it does not, and would fail on the bench. Needs one
  canonical row shape (the richer one) with both writers calling it, plus a
  host test that feeds one record set through both and compares bytes. Related
  formatting wart fixed in passing on the way: the hand-rolled `%d.%02d` in
  `dive_store` printed a negative value as `-5.-30`.
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
- **`web/` is a superseded page generation whose assets match nothing in the
  router** — housekeeping, not a defect. `/style.css` is registered and the repo has
  never had that file; `/app.css` and the four sub-pages (`checklist`, `dives`,
  `histogram`, `settings`) have no routes at all. The live page is the single
  file in `src/comms/web_page.h`, extracted to `data/index.html`. Either delete
  `web/` or route it; today it is only a trap for the next reader.
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
