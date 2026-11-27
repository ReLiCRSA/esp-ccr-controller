# Review batch D, 2026-11-27 — the last five findings, one measured premise that was wrong

Batches [A](REVIEW_BATCH_A_2026-11-27.md), [B](REVIEW_BATCH_B_2026-11-27.md) and
[C](REVIEW_BATCH_C_2026-11-27.md) closed the safety findings. This one closes the
remaining five items from that review's "code findings, no new rigging" list.
Every number below came off the controller or out of the repo's own logs, and
where a review claim did not survive checking, that is said with the evidence.

Bench state for the whole batch: controller on `/dev/ttyUSB0` (identified with
`tools/port_id.py` — two identical CP2102s are attached, the handset bridge is
`ttyUSB1`), pack unplugged (`vbat` ≈ 1.56 V, `batlow=1`), cells uncalibrated at
first boot, LPS28DFW depth chip reading.

---

## 1. The 3 s task watchdog — measured at last, and the review's mechanism was wrong

**What the review said.** `esp_task_wdt_init(3, true)` is a 3 s timeout, `worst=`
had been "observed at 3254 ms", and `CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU0`
means the CPU0 idle task is subscribed and can be the thing that trips — with the
pinned priorities making that more likely, not less.

**What was true, and what had never been checked.** Nobody had measured the
stall→panic interval, and the one fixture that looks like it would —
`TEST_SENSOR_STALL` — deliberately calls `esp_task_wdt_reset()` while stalled,
because it grades the *supervisor*. A watchdog cannot be measured by a fixture
that keeps it fed. So two bench-only envs went into `platformio.ini`:

| env | what it changes | what it answers |
|---|---|---|
| `env:esp32_twdtmeasure` | SensorTask stalls at t=8 s **without** feeding TWDT; hud/control/uart2 are not subscribed; the supervisor's heartbeat reboot is compiled out | how long a stall takes to expire, with no other subscription in the way |
| `env:esp32_twdtmask` | same stall, **every** subscription kept | whether a healthy neighbour's feed re-arms the one hardware timer and hides the stalled task |

Both builds print the interval themselves. The `E (<ms>)` stamp ESP_LOG puts on
the panic line is **not** on the same epoch as `millis()` or
`esp_timer_get_time()` in this build — the first version of the probe subtracted
them and reported 14 s for a 3 s gap. So the probe records the expiry inside
`esp_task_wdt_isr_user_handler()` (the hook the SDK calls from `task_wdt_isr`)
into a `.noinit` record — the same trick the supervisor's fault record uses —
and the next boot prints the difference, measured on one clock on the board:

```
10:39:50.958 TWDTPROBE: sensor stalled at espMs=8006 (last fed millis=7995, that loop started 11 ms ago), sensor is the ONLY TWDT subscription: expect a task_wdt expiry
10:39:53.962 E (22347) task_wdt: Task watchdog got triggered. The following tasks did not reset the watchdog in time:
10:39:53.962 E (22347) task_wdt:  - sensor (CPU 0)
10:39:53.962 E (22347) task_wdt: Tasks currently running:
10:39:53.962 E (22347) task_wdt: CPU 0: IDLE0
10:39:53.962 E (22347) task_wdt: CPU 1: IDLE1
10:39:53.962 E (22347) task_wdt: Aborting.
10:39:53.962 abort() was called at PC 0x400fc830 on core 0
10:39:54.963 reset reason: TASK_WDT
10:39:54.963 TWDTPROBE: task_wdt expired 3061 ms after the last fed loop (stall->expiry interval, one clock, measured on the board)
```

Four boots, two builds (`logs/twdt_measure_1.log`, `logs/twdt_mask_1.log`):

| build | subscriptions | stall → expiry |
|---|---|---|
| `esp32_twdtmeasure` | SensorTask only | **3061 ms**, **3011 ms** |
| `esp32_twdtmask` | sensor, hud, control, uart2 | **3059 ms**, **3019 ms** |

Two results, and the second one is the useful one:

* **The timeout is 3 s plus the console dump.** ~3011–3061 ms against the 3000 ms
  `esp_task_wdt_init` asks for. Stage 3 (the `RESET_SYSTEM` stage at 2× the
  timeout) never fired: the reset reason is `TASK_WDT`, not `WDT`, and the next
  boot is forced into SAFE — which is the chain the review predicted, now seen
  four times rather than argued.
* **A neighbour's feed does not mask a stalled task.** Keeping all four
  subscriptions alive changed the interval by tens of milliseconds, not by
  seconds. `esp_task_wdt_reset()` sets the bit of the task that called it; the ISR
  runs on the timer's own period and names the subscriptions whose bits were not
  set — which is exactly what the dump shows (only `sensor` missed, while both idle
  tasks were running). The naive worry was that hud/control/uart2 would re-arm the
  countdown on the sensor task's behalf; they only account for themselves. That
  dump is the diagnostic the review asked the watchdog to provide, working.

**What the review got wrong.** No idle task is subscribed in this build. Not
"subscribed but not the one you'd expect" — subscribed at all:

* the SDK this env compiles against
  (`framework-arduinoespressif32@3.20017…/tools/sdk/esp32/qio_qspi/include/sdkconfig.h:343-346`)
  sets `CONFIG_ESP_TASK_WDT 1`, `..._PANIC 1`, `..._TIMEOUT_S 5` and
  `..._CHECK_IDLE_TASK_CPU0 1`, and does **not** define `CONFIG_ESP_TASK_WDT_INIT`;
* disassembling the linked image, `esp_task_wdt_init` (0x400fc840..0x400fc957)
  calls `esp_intr_alloc` and `periph_module_enable` — no
  `xTaskGetIdleTaskHandleForCPU` (0x4008deec), no `esp_task_wdt_add`. The
  idle-hook machinery exists, but `esp_task_wdt_add` reaches it only when the
  handle passed *in* is an idle handle, and every registration in `src/` passes
  `NULL` for a real task.

Batch A's own comment was also too strong, and is corrected in the source: it
claimed no SDK archive references `esp_task_wdt_init`. `libfreertos.a`'s
`port_common.c.obj` does (as an undefined symbol). What is true, and what
matters, is that the linked image has exactly **one** call site, inside
`setup()` — checked by disassembling `firmware.elf` and searching for calls to
the symbol. So whatever the SDK's 5 s says, the timeout in force is the one this
firmware passes, and a review premised on "the idle task trips it when a pinned
task hogs a core" described a mechanism this firmware does not have.

**The 3254 ms figure has no source here.** Every `worst=` sample this repo keeps
— 639 of them, across every acceptance and soak log under `logs/` — tops out at
**505 ms** (`logs/accept_20261127_122109_watch.log`, this batch's own acceptance
run; the previous maximum was 457 ms in `logs/accept_20261126_190333_watch.log`).
The 3 s watchdog has ~2.5 s of margin against the slowest control cycle ever
recorded on this bench, not −0.25 s. The review's number may have come from a
board or a build outside this tree; nothing in this repo supports it, so it is
treated as unmeasured rather than as a finding.

**What was loaded, and what that bought.** `tools/web_storm.py` is new: it floods
the HTTP surface the dashboard actually uses (`/`, `/style.css`, `/app.js`,
`/surface.csv`, `/dive/N.csv` — the `.csv` routes are LittleFS reads on the same
flash the dive log writes to) while reading the console, so the firmware's own
`worst=` is collected *during* the load. Four configurations, two passes each,
25 s windows (the second sweep: `logs/storm_final_c4.log`, `..._c8.log`, `..._c16.log`,
`..._c24.log`; the first sweep at the same settings was overwritten by it, and its
client-side numbers are quoted below as observations). The numbers taken off the
controller's own heartbeat:

| clients | half-open held | max `worst=` | max HB gap | `cfggen=` | fatals |
|---|---|---|---|---|---|
| 4 | 0 | 4–6 ms | ~5.0 s | 3/3 unchanged | none |
| 8 | 0 | 4–9 ms | ~5.0 s | 3/3 unchanged | none |
| 16 | 0 | 4 ms | ~5.0 s | 3/3 unchanged | none |
| 24 | 120 | 4–8 ms | ~5.0 s | 3/3 unchanged | none |

The control cycle did not move: **`worst=` never left single digits**, the HB beat
kept arriving about every 5 s, and no panic, reset or SAFE entry appeared in any
pass. AsyncTCP and the WiFi driver are pinned to core 0, the control chain owns
core 1, and the network work never reached the cycle-to-cycle span the watchdog
measures. That is the premise measured rather than argued.

The client-side counts are reported without being credited to the controller, and
the reason is worth stating: the same 8-client setting answered 101 requests
(60×200, 23×404, 10×503, 8×302, 636 KB) in one pass and **nothing** in the next,
with 38 connects timing out. A spread that large is the 2.4 GHz link and the bench
PC's own association, not the board — an earlier run caught NetworkManager keeping
`ESP-CCR-3828` "activated" while ARP for 192.168.4.1 went `INCOMPLETE`. So the tool
prints every outcome including the errno of each failed connect, and grades the
answer count only against `--min-ok` (whose default assumes a wired link), while
the safety claim rests on the board-side columns above.

One client-side answer that *is* the controller's: 10 of those 101 were
`503 "no memory for export"` (`web_server.cpp:288`) — the web layer refusing to
build an export it cannot hold in RAM, which is the designed degradation and now
has a bench observation behind it.

What the storm does *not* cover, said plainly: the board was at the surface (no
dive, so no dive rows were being written while the exports were read), and the
flood is HTTP-only — the live telemetry stream is a WebSocket this tool does not
speak. Flash contention *during a dive*, with exports running, is still unmeasured;
the `--expect-depth` leg needs a thumb on the pressure sensor and is the natural
next step for that number.

The tool also refuses the easiest false pass: it polls with a real `GET` until the
server answers (a bare TCP connect is not enough — while the box is saturated its
listener still completes SYNs and sends nothing), can re-join the access point with
`--nmcli-profile`, measures how long the listener takes to answer again after the
window, and fails the run if the flood never reached the controller. A load test
that quietly loads nothing is the easiest false PASS there is.

`tools/hw_accept.py` now quotes the measured expiry in the `worst=` check, so the
margin sits next to the threshold instead of in a comment:

```
[PASS] worst= cycle margin <= 800 ms (worst 505, reboot budget 1500, measured TWDT expiry 3011..3061 ms)
```

**Deliberately not done:** raising the timeout to 5 s. That would have made the
finding disappear on paper by trading a measured 3 s detection of a wedged
control task for a 5 s one, on a machine that injects oxygen, to fix a margin
that the measurements above put at ~2.5 s.

---

## 2. Flag labels — one fault, one name, in all four places that name it

Finding M-3 (bit 6 labelled `SENSOR` on a cell-proof failure) had been fixed by
hand-editing three files, which re-breaks the next time one of them is touched.

The projection is now **one function**: `src/comms/flags.h` →
`comms::flagsFromAlarmWord(alarmBits, proofFail)`, called by both transports —
`UartLink::emitT` for the pushed `T,` stream and `cmd_router`'s `telemetryFlags`
for `GET T` and the web. It used to be two inline copies with the same body on
two files, which is how the original defect happened: `CNS_HIGH` was dropped by
*both* under a comment that believed the bits 3/6/7/9 aggregate covered it, and
fixing one copy would have been invisible on the bench.

`test/test_flaglabels/` (new, 5 cases) grades the **agreement**, not the wording:

| case | what it reddens on |
|---|---|
| flashed page vs source page | `data/index.html`'s `FLAGNAMES` differs from `src/comms/web_page.h`'s — someone forgot `tools/sync_webdata.py` |
| one pill per published bit | a pill whose label doesn't contain its bit's keyword (bit 6 = `proof`), or two pills sharing a name |
| §4 publishes bits 0..7 once each | the spec's FLAGS table missing / duplicating / reordering a row |
| every alarm condition lands where the docs say | an alarm-word bit dropped, doubled, or projected to a row that no longer names it (all four aggregate members named individually) |
| nothing invented | an empty alarm word lighting a pill, or a bit set outside the published byte |

Verified by mutation, in both directions:

```
pill 6 renamed back to "SENSOR FAIL":
  the flashed page's pill names differ from the source page's: regenerate data/index.html …  FAILED
  pill 6 is labelled "SENSOR FAIL" and does not name the fault §4 publishes for bit 6       FAILED
§4's bit-3 row shortened so it stops naming stuck-open and hypoxia:
  BIT_STUCK_OPEN projects to FLAGS bit 3, whose §4 row never says "stuck"                   FAILED
```

`test/test_uartlink` also gained `test_every_published_flag_bit_is_reachable_and_names_exactly_one_fault`:
every published bit reached by name, exactly one bit per condition, alarm bits
3/6/7/9 collapsing to wire bit 3, CNS to bit 7, cell proof to bit 6, the union
coming to `0xFF`, and `0xFFFF` + proof still never setting a bit outside the byte.

One stale comment fell out of the same pass: `handset/handset_link.h`'s
`UNSETTLED` said "the spread band" where the code uses per-window **stability**
bands (`spread` is the `CALAIR,` reply's own field — a different quantity, and the
sentence that first explained it confused the two).

---

## 3. Dive-log rounding — the stored count and the printed digits share one rule

`continuity.cpp` stored tissue and CNS with hand-written `* SCALE + 0.5`, while
the row formatter printed through `numfmt` and `printf` rounds half-to-even on
floats. A disagreement in the last stored unit, for exact ties only — and
`test_numfmt`'s own comment claimed no production file hand-rolls the pattern,
because its anchor scan listed three files and `continuity.cpp` was not one of
them. **The rule was right; the file list was the weak half**, so the scan now
covers `src/log/continuity.cpp` too.

Continuity now calls `numfmt::scaled()` / `numfmt::narrow<int32_t>()`, and the
scale factors and the decimals are one pair in the header
(`CONT_TISSUE_DECIMALS`, `CONT_CNS_DECIMALS`, `contPow10()` with `static_assert`s
pinning that the literal scale equals `10**decimals`). New cases assert both
halves of the claim — the stored count, and that the stored count prints as the
formatted value: `0.03125 bar → 312` (not 313), `0.09375 → 938`, `1.23456 →
12346`, CNS `0.25 % → 2`, `0.75 % → 8`, `42.7 → 427`, and
`fixed(v,d) == fixedScaled(stored,d)` in every case. CNS is also range-checked on
the **float before** scaling, because `numfmt::scaled(NaN)` returns 0 and 0 % is
a legal CNS — a NaN would otherwise have been stored as a plausible zero.

Restoring either `+ 0.5` turns it red, both the behaviour and the scan:

```
test_stored_values_go_through_the_helper: src/log/continuity.cpp:146 scales or rounds by hand (+ 0.5); go through numfmt::scaled  FAILED
test_an_exact_tissue_tie_goes_to_the_even_side_like_the_printed_digits: Expected 312 Was 313 …                                FAILED
test_an_exact_cns_tie_goes_to_the_even_side_like_the_printed_digits: Expected 2 Was 3 …                                         FAILED
```

No record-version bump: the layout is unchanged, only the arithmetic, so records
written before this still read. This is the item that changes stored bytes, so it
is its own commit.

---

## 4. Config generation — a third publisher is refused, and the skew is a number on HB

Batch C's rule ("the settings path is the only mid-flight writer") lived only in
a comment. `test_configbox` now counts `configPublish(` call sites across the
source and fails with three, with a message that says what a new writer must do
instead — read the published generation, apply onto that copy, publish the
result:

```
test_only_the_boot_and_the_settings_path_publish: Expected 2 Was 3. the number of configPublish call sites moved: the single-writer rule is the box's contract   FAILED
```

For the skew itself, HB gained `cfggen=<published>/<chain>` — appended after
`contskip=`, per the append-only rule:

```
HB st=0 … stk=2176 u2stk=3864 worst=5 logskip=0 surfskip=0 contskip=0 cfggen=1/1
```

Equal means the chain is running exactly what the last `SET` produced. One apart
is normal and bounded — a `SET` lands between ControlTask's read and the solenoid
pin fuse of the next cycle. Two behind means a control cycle stopped reading,
which is the failure the supervisor reboots for, now visible a beat earlier. The
chain half comes from the *same* `configRead` that carried the values
(`configRead(cfg, &gen)`), never from a later `configGeneration()` call — that
would report what the settings path published *by now*, the same half-pair
problem the box exists to remove. `configRead(Config*, uint32_t*)` is the
overload that keeps them together, and a test pins that pair, the call-site
count, and the fact that `cfggen=` sits after `contskip=`.

`hw_accept.py` grades it on the wire — field present, chain never ahead, chain at
most one behind — with four lying-capture cases in `--selftest` (missing field;
one behind is normal; two behind; chain ahead of published). On the flashed
board:

```
[PASS] cfggen= chain never ahead of published (worst 0)
[PASS] cfggen= chain at most one generation behind (worst 0)
```

Mutations: dropping the `g_chainCfgGen = cfgGen` assignment reddens
`controlTask no longer records which generation it applied`; adding a third
publisher to `session_wiring.cpp` reddens the count check.

---

## 5. The design's mock wire lines are graded as wire lines

`design/handset-screen/screens/*.svg` is 41 panels, each publishing the protocol
lines it depicts inside its `<desc>`. Those lines were typed by hand from the
spec, and a hand-typed wire literal rots in three ways that all look fine on
screen: it stops parsing (the `T,` battery field, the `L,` fourth field and the
`DIVE,` v3 date all moved during 2026); it becomes impossible (a `FLAGS0x80`
beside a 12 % CNS gauge); or it contradicts the firmware's projection (an `E,`
alarm row naming a fault the panel's own FLAGS byte leaves clear).

`test/test_designwire` (new, 5 cases) reads every panel's `<desc>`, decodes
entities, splits the payload, and feeds each protocol line to
`handset/handset_link.h` — the same client library the conformance suite grades —
then checks the parsed values against the firmware's own rules. Measured on the
tree: **82 protocol lines across the 41 panels**, all parsing, with a per-prefix
floor (10 `T,`, 10 `D,`, 5 `E,`, 5 `CALAIR,`, 8 `TIME,` …) so the suite cannot
pass by finding nothing.

| case | rule |
|---|---|
| every line parses on the client library | grammar drift, caught by the parser a real handset uses |
| the `M,` totals row matches the spec template | the panel's column count vs the count `docs/HANDSET_PROTOCOL.md` publishes |
| `T,` field shapes | 10 fields, depth 2 decimals, ppO2/cells/setpoint 3, battery 2, MODE in STANDBY/AUTO/MANUAL/SAFE |
| panels describe reachable states | every FLAGS byte is producible by `comms::flagsFromAlarmWord`; a panel lighting the CNS bit must show CNS ≥ `CNS_ALARM_PCT`; every `E,` alarm row must be visible in that panel's FLAGS byte |
| the design invents no commands | the retired `PULSE` verb must not appear; `CAL O2` must be drawn answering `ERR not implemented` |

The design's bit-7 caveat (§6 of the design sheet) is now *executed* rather than
asserted in prose: `alarm-cns-high.svg`'s `FLAGS0x80` is checked against the
projection and against the deco row printed beside it. That closes the open
question the earlier docs-only commit left: the protocol in use carries bit 7,
`CNS_ALARM_PCT` is 80.0, and `test_uartlink` / `test_router` pin the byte
(`0x80`, `0x08`, `0x88`, `0xC0`, and `GET T` = `0x80`).

What the suite enforces is a count, not a feeling: a per-prefix floor (14 `T,` and
13 `D,` mocks, 15 `TIME,`, 10 `E,` alarm rows, 6 `CALAIR,`, 6 `K,` schema rows, 4
`DS,`, 3 each of `L,`/`PRE,`/`WIFI,`, 2 `PROOF,`, 1 `CELLS,`, 1 `DIVE,`, 1 `M,`)
so it cannot pass by finding nothing.

The scan lesson from item 2 applies here too and is worth repeating because it
nearly cost a day: the first version of the **`test_flaglabels`** §4-table scanner
hung forever — it advanced to the newline it had just consumed instead of past it,
so a 600 s timeout was what exposed it. That is the same family of fault as the
`test_configbox` scan matching a comment about a rule instead of the rule: an
anchor satisfied by the wrong thing. Every scan in this batch therefore asserts a
minimum count of what it actually graded.

---

## Verification, batch D

| step | result |
|---|---|
| native suite | **429 cases, 429 passed** across 33 suites (33 directories in `test/`, all built) |
| `esp32`, `esp32_stalltest`, `esp32_twdtmeasure`, `esp32_twdtmask` | SUCCESS, no new warnings from the touched sources |
| flashing image (`env:esp32`) → controller | `pio run -e esp32 -t upload --upload-port /dev/ttyUSB0`; boot `reset reason: POWERON`, mode CTRL, `TWDT: 3 s timeout, panic on (registered: sensor, hud, control, uart2)`, `[CELLS UNCALIBRATED: run CAL AIR]`, no `rst:0xc` |
| full acceptance, `--expect-web --handset /dev/ttyUSB1 --cal-air` | **0 failures**, against the image built from this tree (boot banner names all four TWDT subscriptions and `cfggen=` on HB). Over its 30 HB beats: `worst=` max 505 ms, `u2stk=` floor 3096 B, `cfggen=` 1/1 → 2/2 → 3/3 through the run's three SETs, never a generation skewed. `logs/accept_20261127_122109_*.log` |
| `stk=` headroom | 512 B — the graded floor exactly, hit during the `CAL AIR` leg. It passes, and the number sits exactly on the line rather than comfortably above it; that is recorded here rather than rounded away |
| the review's claim, measured | stall → TWDT expiry **3061 / 3011 ms** (sensor only) and **3059 / 3019 ms** (all four tasks subscribed); panic text names `sensor (CPU 0)`; `abort() was called at PC 0x400fc830 on core 0`; next boot `reset reason: TASK_WDT` → SAFE. `logs/twdt_measure_1.log`, `logs/twdt_mask_1.log` |
| the review's CPU0-idle mechanism | **absent in this build** — `esp_task_wdt_init` in the linked image references neither `xTaskGetIdleTaskHandleForCPU` nor `esp_task_wdt_add`; batch A's "no archive references `esp_task_wdt_init`" was too strong (`libfreertos.a`'s `port_common.c.obj` does) and is corrected in the comment, alongside the fact that the linked image has one call site, in `setup()` |
| the 3254 ms figure | no support in this tree: 639 `worst=` samples across `logs/`, maximum **505 ms** — and that maximum is this batch's own run, 295 ms below the graded 800 ms line and ~2.5 s below the measured watchdog expiry |
| web load (`tools/web_storm.py`, 4/8/16/24 clients, 24+120 with half-open held, 25 s windows, two passes each) | board-side: `worst=` stayed at **4–9 ms**, HB gap ~5.0 s, `cfggen=` unmoved, no panic/reset/SAFE in any pass. Client-side counts vary from 101 answered requests (636 KB, 10 of them honest `503 "no memory for export"`) to zero at the same setting, so they are reported, not credited to the board. `logs/storm_final_c*.log`. Not covered: exports while a dive is writing rows, and the WebSocket stream |
| the storm's own first runs | found the PC believing it was joined while ARP was `INCOMPLETE` → 0 replies, and a connect-only preflight that called that "reached". The tool now polls with a real `GET`, can re-join via `--nmcli-profile`, times how long the listener takes to answer again, and fails loudly when the flood never reached the controller |
| mutations | continuity `+0.5` restored → 3 failed; pill 6 renamed → 2 failed; §4 bit-3 row shortened → 1 failed; `g_chainCfgGen` assignment dropped → 1 failed; third `configPublish` added → `Expected 2 Was 3`; design panel's `FLAGS0x80` → `FLAGS0x10` → "shows an alarm event for alarm bit 8 while its T, line leaves the matching FLAGS bit clear"; `CAL O2 → ERR not implemented` changed to a success → refused; `M,` row loses a column → `Expected 15 Was 14`; `T,` field 2 to 2 decimals → `Expected 3 Was 2`; `hw_accept.py --selftest` → 0 failures with the four new `cfggen=` lies among the lying captures |
| board left in | the `env:esp32` shipping image, verified by boot banner rather than assumed — both probe builds stall on purpose and must never be the image an acceptance run starts from. Bench state after the run: cells **calibrated** (the run's `CAL AIR` committed, `cal air: OK raw cell0=1546 cell1=1556 cell2=1580`, surface record 193), pack still unplugged (`vbat=1.56`, `batlow=1`) |

## Still open

- Flash contention **during a dive**: exports read while the dive log writes, with
  the depth sensor squeezed, is the unmeasured case behind the review's slow-cycle
  worry. `worst=` peaked at 8 ms at the surface, so the margin question is now
  about that configuration specifically.
- `worst=` is graded against 800 ms; the measured watchdog expiry (3011..3061 ms)
  and the supervisor's 1500 ms budget are printed beside it, but nothing yet
  *stress-tests* the control cycle toward those numbers on purpose.
- The `CAL O2` / `CAL DEPTH` sequencers are still `ERR not implemented`;
  `cal::O2Sequencer` is host-tested and still never instantiated, and
  `test_designwire` now pins that the design shows the refusal, not a success.
- C3 bench items unchanged: MS5837 swap/potting, battery measurement, mechanical
  solenoid inhibit, fails-closed rig.
- The handset UI itself is still unimplemented: the design is a handoff, and the
  example handset board has never been flashed.
