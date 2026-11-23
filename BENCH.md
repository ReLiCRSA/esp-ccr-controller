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
pgrep -f soak_logger          # must be empty; kill before touching the port
pio run -e esp32 -t upload    # only once the soak has ended
pio run -e esp32 -t uploadfs  # page into LittleFS (optional: fallback is in flash)
tools/.../python tools/hw_accept.py --expect-web --handset /dev/ttyUSB1
```

`hw_accept.py` checks the boot console (fresh boot banner, ≥2 HB lines
matching the soak_logger contract, no SAFE/FATAL, `settings: N keys from
NVS`, `control: SP=…`, `web: AP "…"`) and then runs the **refusal matrix**
over the wired handset: `AUTO` cold → `ERR predive required`; `SET SP2`
mid-dive → `ERR dive locked`; `WIFI ON` mid-dive → `ERR wifi locked in
dive`; `CAL AIR` mid-dive → `ERR cal locked in dive`; overlong line →
`ERR line too long`; junk → `ERR unknown command`. Every one of those is a
host-tested invariant — the bench exists to prove the wire, not the logic.

Expected boot order (post-A5e firmware):

```
=== ESP-CCR controller M1 boot ===
reset reason: power-on
settings: N keys from NVS
control: SP=0.40/0.70/1.00/1.30 cells=111 wall=1.45 cutoff=1.60 ...
web: AP "ESP-CCR-xxxx" on 192.168.4.1 (heap ...)
HB st=... (every 5 s)
```

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
