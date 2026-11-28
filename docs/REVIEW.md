# Review — findings, fixes, and what is still open

**This is the single source of truth for review findings and open items.** It replaces
the five per-batch review documents (`REVIEW_BATCH_A…E_2026-11-27.md`), which were
consolidated into this file on 2026-11-27 and removed. If you are looking for a finding
from the 2026-11-27 review, it is here; if you are looking for the *sequence* in which
they were fixed, §2 has that, and the commits have the rest.

Three things about this document are deliberate, and ignoring them is how it goes stale:

* **§1 is the only open list.** An item that stops being open stops being in §1 — it
  moves to §4 — closed without a fix, or closed by a decision not to fix, with the reason
  and the anchor that keeps it closed — or to §5, where the finding or the claim itself
  turned out to be wrong. Anything in §1 is
  something to do.
* **Every claim states what it was measured against, and re-verification is recorded
  next to the claim, not in a new document.** Several findings in here were corrected
  after they were written, by checking the claim instead of trusting the earlier line.
  §5 keeps those corrections visible on purpose.
* **An open item needs an anchor: a named test, tool or bench leg that shows it is
  open, and when it has been done.** A line in a doc is a claim; the anchor is the
  evidence. §1 gives the anchor for every item.

**Start at §1.**


---

## 0. How to work from this document

The review was one pass over the firmware (2026-11-27) that produced findings labelled
**M-2, M-3, m-1, m-2, m-4 and N1**, plus the further items that review produced, which
the rounds have named by subject. Five rounds of fixing them followed, each on a real
bench. Each
round is kept in §2 as its own section, with the commit range and gate counts it left
behind; the findings themselves are in §3. What changed across rounds is recorded once,
in §8, rather than left to be inferred from the sections.

Two conventions that make the rest of this readable:

* A finding's section ends with what **witnesses** the fix — a test name, a tool, a
  bench leg — and with what is **still not proven** about it. "Fixed" never means
  "settled"; the residuals feed §1.
* Numbers are quoted with where and when they came from. A count without a date is a
  guess.

**Re-verify before citing.** This rule is not decoration; three times in this record a
line that had been written down was false by the time it was read:

* Round E's open list repeated Round A's flag-label residual ("nothing tests that the
doc table and the JS array agree") and it was **wrong when written** — Round D had
landed `test/test_flaglabels` the same day, which parses all four vocabularies. Found by
grepping for the test instead of trusting the line; §5 keeps the correction.
* Round A's residual 4 said the warm-up extraction "was implemented but did not survive
concurrent edits", i.e. a coverage gap. What was actually on disk was worse than
untested: the extracted code wrapped after ~24.9 days and made the gate refuse forever.
Round E item 4.
* Round A's residual 1 said "3 s is now real" and "the trip is **not measured on
hardware**". The first half was a claim about a Kconfig symbol that does not exist on
this toolchain, and the second half was still true a day later. Round D item 1 measured
it at 3011–3061 ms and corrected the mechanism.

The rule: **the doc records what was verified and how; a line written yesterday can be
false today, so check it before you cite it as evidence or treat it as an open item.**


---

## 1. Open items

In the order I would take them. Each entry says what is missing, why it matters, and
what would close it. Nothing here is a style preference: every entry is either a claim
the repo makes that it cannot yet witness, or a gap where a number that cannot be
believed reaches a decision.

### P1 — the battery sense path is not trustworthy; the firmware now refuses it, the bench has not yet metered it

The bench drove the `vbat` reading to a value no pack could produce, and no check in the
firmware noticed. It is the only open item that changes a dive-blocking decision, and
the only one this bench *demonstrated live* rather than reasoned about.

**Status as of 2026-11-27: the firmware half is closed.** A reading that cannot be this
project's pack now raises `BIT_BATTERY`, refuses the pre-dive BATTERY step, and says so on
the console. What is **still open is the bench half** — the divider has never been metered,
`VBAT_CAL` has never been trimmed, and no acceptance leg compares `vbat=` to a meter. The
evidence below stays exactly as recorded, because it is what the fix was written against.

**The chemistry fix is now on metal (flashed 2026-11-28, this bench).** The board reports
its own migration: `settings: NVS VBAT_MIN="10.500" refused (out of range) - the registry
default for that key is used instead`, then `settings: 35 keys from NVS`. HB went from
`vbat=4.92 batlow=1` to **`vbat=3.62…3.65 batlow=0 al=0x0000`** across seven beats, and the
handset readout on the same bench showed `batt 3.64 V` with no battery marker on screen
(it had shown `FLAGS MAJOR BATT` before). Note the shape of that number: it jitters in the
last digit, where `34.71` was dead flat across six beats — so this boot is not the pinned
pad, and the pinned-pad condition remains something the bench has produced once and not
yet re-produced under a meter. Still open, unchanged: metering the divider. The other
half of that sentence — that the refused 10.5 is **not written back**, so the board refuses
it every boot — was right about this board's history and wrong as a rule: corrected
2026-11-28, §5. `load()` really does not write back, but `Store::save` rewrites **every**
key from the in-force registry, so the stale string survives only until the next settings
write of **any** key.

**That refusal sentence became a graded leg on 2026-11-28, and it grades the part the
sentence is really claiming.** The boot line alone proves only that the firmware *said*
something: a board that prints the refusal and **still runs the refused number** passes
every check the harness had, and so does one that accepts the stale value in silence and
prints nothing. The acceptance matrix now reads the key back (`GET VBAT_MIN`) and joins
three answers taken seconds apart — the boot's refusal, the key's band and default from
the board's own `GET SCHEMA`, and the live value — so "the registry default is used
instead" is a compared fact, not a quoted one. No number is copied into the tool: the
band and the default come off the same schema row, which keeps this join board-to-board.
Eight host fixtures, each proven to be able to fail: refuse-and-still-run, refuse-and-run
-something-that-is-neither, the sentence losing its tail (the half that says what runs
instead), a refusal naming a key absent from the published table, acceptance in silence
with an out-of-band value live, and a refused key never read back — plus the honest boot
and a genuinely clean NVS, which must stay green. Wiring is graded too: a selftest case
fails if no acceptance run reaches the grader, because a check that exists and never runs
is this bench's oldest failure mode. Fixing the leg also exposed a window bug in the
matrix itself: `GET SCHEMA` answers one line per key (36), and first-match semantics
stopped after the first `K,` row and let the other 35 smear into the next check's window —
the same amputation `GET *` had and was fixed for. **Driven on metal 2026-11-28**, over the
new S3 bench bridge (`--handset /dev/ttyACM1`), in both of its states. Clean NVS: the join
ran inside the first full battery — **181 checks, 0 failures** with the pack plugged in, so
the BATTERY step ran (`logs/accept_20261128_185837_*`) — where the phase contributes its
two always-on checks and nothing else. Refusal NVS: seeded deliberately, since this bench's
flash is clean (`SET VBAT_MIN=4.400` inside the real band, a temporary build with
`VBAT_CELL_RANGE_MAX_V 4.30f` — bench seed, reverted, never committed), which makes the boot
refuse a stored value and puts the five refusal-specific checks on real numbers —
**123 checks, 0 failures** (`logs/accept_20261128_nvs_migration_seed_123p_0f.log`), against
**118** for the same matrix clean afterwards
(`logs/accept_20261128_after_seed_clean_118p_0f.log`). Driving the matrix is what found the
two defects §2 names: its own family drain livelocked on the live 2 Hz stream, and `M,`'s
pulse total charged the dive with the pre-dive test's pulses.

What was seen, on the bench, 2026-11-27:

| observation | value |
|---|---|
| HB reported | `vbat=34.71` |
| the same reading, six heartbeat beats running | **identical to the last digit** |
| implied voltage at GPIO35 (`34.71 / VBAT_DIVIDER_RATIO 11.0`) | **3155 mV** |
| ESP32 ADC1 at 11 dB, usable top | ~3.1–3.3 V |
| ESP32 ADC pin absolute maximum | VDD + 0.3 ≈ 3.6 V |
| what a working 100 k/10 k divider with 5 V on its input should give | 455 mV at the pin → HB `vbat=5.00` |

A divider reading jitters in the last digit; a reading dead flat across six beats is a
pinned input. So the 5 V feed is reaching the sense pad essentially unattenuated, and
`34.71` is the pad's clamp multiplied by 11 — not a measurement, and not even the
saturating value of the divider (that would read 55.0).

The sense path has also never been witnessed at all:

| evidence | what it shows |
|---|---|
| `grep vbatMax\|VBAT_MAX\|batValid\|senseValid` over `src/ include/ test/` | **no upper bound, no range check, no validity flag — anywhere** (measured 2026-11-27, before the fix; `VBAT_IMPLAUSIBLE_V` and `Battery::implausible()` now exist, so re-run this grep before quoting the row) |
| `tools/hw_accept.py` `HB_RE` (the heartbeat contract parser) | parses `vbat=` for **shape only**; no leg of any acceptance run ever checks its value |
| BENCH.md C3-c | the battery item is still `- [ ]` — "HB `vbat=` within 0.1 V of a meter at three loads" has never been done on this bench |
| every `vbat=` in every log this project has produced | 8992 beats in `logs/soak_20261122_094738.log`: **1.56–2.10 V**, 249 distinct values, noise everywhere. 6 beats out of 8992 exceeded 4.5 V, longest run 2, each beside an `i2cerr`/`rec` bump. All-time maximum before today: **8.94 V**, a single beat |

A 10 k bottom leg to ground would hold a floating pin near 0 V and stable; 1.56–2.10 V
drifting everywhere is a floating input. Today is the first time anything has been
driven onto that net.

Why it mattered — the three places that consume it, each with the whole test it made **when this was measured** (line numbers and tests as they now stand, after the fix):

| site | the test it made then | the test it makes now |
|---|---|---|
| `src/control/alarms.cpp:102` | `evaluate(BIT_BATTERY, in.vbat < cfg_.vbatMin, …)` — nothing else | `in.vbat < cfg_.vbatMin \|\| in.vbat > cfg_.vbatMax` — the ceiling raises the same marker, because "do not trust this pack" is the word a diver can act on either way |
| `src/control/predive.cpp:161` | `if (in.vbat >= cfg_.vbatMin) { advance(S_BATTERY); }` — the dive-blocking gate | floor, then ceiling, then — series packs only — the two halves must agree; an unwired midpoint refuses |
| `src/main.cpp:910` | `sd.vbat = g_battery.volts();` — straight into the dive snapshot, and from there to `T,` (`:1029`) and the dive row (`:1342`) | unchanged, and deliberately so: the dive file records the number, and the alarm/pre-dive verdicts are the places that act on it |

So the pinned pad produced `batlow=0`, **passed the dive-blocking battery step on a 5 V
feed**, and would have stored 34.71 V as that dive's pack voltage. `stale=` is
`g_ads.anyStale(now)` (`main.cpp:413`) — the cell ADC — and says nothing about the battery
ADC, so `batlow=` is the only battery verdict the heartbeat carries. A saturated sense input and a
healthy pack are no longer indistinguishable to this firmware — `Battery::implausible()`
(`src/sensors/battery.h`) and `VBAT_IMPLAUSIBLE_V` (`include/config/defaults.h`) exist, HB's
`batlow=` answers yes for either fault, and the console says which one it is
(`battery: sense IMPLAUSIBLE: … V cannot be a 1-cell pack of this chemistry`).

**Closed on 2026-11-27 (item 2 of the list below), and how to re-check it:**
`include/config/defaults.h` now holds the chemistry as one number — `VBAT_CELLS` (1),
`VBAT_CELL_MIN_V` (3.40 V), `VBAT_CELL_IMPLAUSIBLE_V` (4.60 V) — and every threshold is
derived from it, including the `VBAT_MIN` settings band (`VBAT_RANGE_MIN_V`…`VBAT_RANGE_MAX_V`).
The band being chemistry-shaped is also the migration: `nvs_store` range-checks what it
loads, so the 10.5 V sitting in this board's NVS falls back to 3.40 V on the next boot
instead of outliving the change. That was not a hypothetical: at 10.5 V a charged 3.7 V
pack failed the pre-dive BATTERY step **permanently**, which is why `AUTO` never unlocked
and §1 P5 stayed blocked. The witness that the derivation holds: changing that one line to
`2` reddens exactly the two settings-table legs — the published contract band in
§6 of `docs/HANDSET_PROTOCOL.md` moves with the chemistry, and the new settings-table
join says so by key name — and with that one doc row updated the same tree gives
448/448 native and 896/896 across both levels with a clean `env:esp32` link. Run
2026-11-27, in both directions — and mutating any of the three rules reddens exactly its own test leg.

**Still open — the bench half:** (1) meter the divider — power off, pack input to GPIO35 should be
~100 k and GPIO35 to GND ~10 k, an open top leg being the reliable tell; then 5.00 V at
the pack input with a meter at GPIO35, where ~455 mV means the divider is correct and
~3.1–3.4 V flat means there is no attenuation in the path; (3) the C3-c leg: HB `vbat=` within 0.1 V of a
meter at three loads, and only then a `VBAT_CAL` trim (still `1.00f`, never trimmed).
At the pack this rig actually has — one 3.7 V cell, reading ~4.9 V on the unmetered
divider (HB `vbat=4.92 batlow=1`, 2026-11-27: `batlow=1` is the old 10.5 V floor answering
a pack it was never meant to judge) — a correct 11.0 divider puts ~445 mV on the pin, well
inside the 11 dB range.
A firmware bound cannot fix a divider: `vbat=34.71` is now refused, but a
divider that reads 20 % off still passes every check in the tree, which is what the
metering leg is for.

Note the knock-on: §1 P5 (the non-vacuous pulse witness) is blocked by *this*, not by an
absent pack — and it was blocked by the floor itself until 2026-11-27, because 10.5 V
refused every pre-dive on a 3.7 V pack whether the sense path worked or not.


### P2 — flash contention mid-dive, from the web export only

The handset half of this is closed (Round E item 3): `LOG EXPORT`
is refused mid-dive, witnessed on metal both ways. `/dive/N.csv` deliberately still reads
the flash during a dive and **has never been measured in a dive** — every storm this repo
has (`tools/web_storm.py`, 4/8/16/24 clients, BENCH.md's table) ran at the surface, where
the control cycle held at 4–9 ms. The highest `worst=` anywhere in the repo's logs is
**505 ms**, also at the surface and from a different cause (P3).

**To close it:** the same harness with a dive running, and a `worst=` number to compare
with the 1500 ms budget and the 3011–3061 ms expiry. Decide from the number: lock the web
export too, or document the margin.


### P3 — nothing stress-tests the control cycle toward the watchdog

Round D measured the detector. `esp_task_wdt_init(3, true)` at `src/main.cpp:1627` asks
for 3000 ms and the measured expiry is **3011–3061 ms** (3061/3011 with SensorTask
subscribed alone, 3059/3019 with all four — `logs/twdt_measure_1.log`,
`twdt_measure_2.log`, `twdt_mask_1.log`). The SDK's own `CONFIG_ESP_TASK_WDT_TIMEOUT_S 5`
decides nothing, because nothing in the linked image initialises the watchdog before that
call, and `CONFIG_ESP_TASK_WDT_INIT` is a macro this SDK never reads. And it **subscribes
no idle task at all** — the review's premise that a hogged core trips the watchdog through
a CPU0 idle subscription describes machinery this build does not run, so the watchdog
watches exactly the tasks that register.

What has not been done is driving the control cycle *toward* that number on purpose.
`worst=` is graded at 800 ms and the supervisor reboots at `TASK_STALL_CONTROL_MS = 1500`
ms (`include/config/defaults.h:223`, enforced at `src/main.cpp:1524` and `:1543` — the
citation this entry carried, `src/main.cpp:1373`, had drifted and pointed at a comment).
Across the 639 `worst=` samples this repo keeps, the maximum is
**505 ms** — a `CAL AIR` leg, `logs/accept_20261127_122109_watch.log`; the previous
maximum was 457 ms — and quiet benches beat at 4–300 ms while the storm configurations sit
at 4–9 ms. So the trip time is known, the largest cycle ever observed is known, and the
gap between them has never been pushed on purpose.

**The 800-versus-1500 choice is now an executable, not a comment (2026-11-28).** It was
three numbers in two files: the constant in `tools/hw_accept.py`, the define in
`defaults.h`, and a third literal `1500` inside the bench check's own words. That last
copy is gone — `grade_watch` now prints the budget **it read out of `defaults.h`** at
import time, and `hw_accept.py --selftest`'s stall-budget cases pin the relationship in
both directions: a grade at or above the reboot budget reddens (a board would reboot
before the console said anything), a grade below 30% of it reddens (it would stop being
about stalling and start failing a healthy bench — the floor comes from this board's own
505 ms record, not from taste), renaming the define reddens the join rather than letting
the tool borrow a look-alike `#define`, and putting the literal back reddens a source
check. Proven in both directions: moving the grade to 2000 ms reddens four cases including
the two pre-existing control-margin ones, moving it to 100 ms reddens the floor, flipping
the comparison from `<=` to `<` reddens the exactly-at-grade fixture.

**To close it:** a bench leg that lengthens the control cycle deliberately (a flash storm
during a dive is the obvious lever, which is also P2) and reports `worst=` against the
1500 ms budget rather than the 800 ms grade. That metal half is unchanged by the above:
pinning the decision makes the margin auditable, it does not narrow it.


### P4 — m-4: the fences are measured, and still unjustified

The seqlock's two fences — `std::atomic_thread_fence(std::memory_order_release)` in
`publishView` (`src/deco/view.h:155`) and `…_acquire` in `viewSnapshot` (`:172`), the
MbedTLS-style pairing. Round E item 5 measured both ends and neither end supports the
strong reading:

| platform | fences in | fences out |
|---|---|---|
| xtensa, `env:esp32`, real ELF | `publishView` 3 `memw`, `viewSnapshot` 5 | (not buildable — that is the point) |
| x86-64 host, `env:native_o2`, `-O0` and `-O2` | 874/874 | **874/874 green at both optimisations with both fences commented out** |

Neither half is inherited. Re-witnessed on 2026-11-27, on the tree this file describes:
commenting out both fences in `src/deco/view.h` gives **874/874 green at `-O0` and at
`-O2`**; restoring them leaves an empty `git diff` and 437/437 again. The target half was
re-run the same day — `tools/fence_probe_report.py` rebuilt `tools/fence_probe.cpp` with
the shipping xtensa toolchain and regenerated `logs/m4_fence_probe_20261127_disasm.txt`
against an ELF from this tree (sha256 `63e3e0ec…`): probe publish 3 `memw` with the release
fence and 2 without, probe read 4 with the acquire fence and 3 without, shipped
`publishView` 3, shipped `viewSnapshot` 5.

So they are one real instruction each on the target (which kills the "zero cost"
argument) and invisible to every test we run (which kills the "they must be load-bearing,
the suite is green with them" argument). Nobody has demonstrated a payload/sequence
pairing that breaks without them. They stay — a cheap correct default, and deleting them
would be a change I cannot justify either.

**To close it:** either a weakly-ordered host stress harness that fails without the
fences, or a targeted ESP32 stress; either way the result becomes an anchor, not an
argument. `tools/fence_probe.cpp` + `tools/fence_probe_report.py` already carry the
counterfactual half, and `env:native_o2` is in `platformio.ini` as the gate.

Residual to state honestly: the disassembly count includes a compiler-prologue `memw`
shared with unrelated loads, so "one `memw` per fence" is measured by *difference*
(fences out removes exactly one), which is the honest form of the claim.


### P5 — the non-vacuous `pulses` witness

Round E item 2 made `M,`'s `pulses` column real — base counts, overlapping
windows, one owner, one total. What it has **not** been witnessed by is a dive this board
actually injected. The window arithmetic is anchored by native tests and by two lying
captures in `hw_accept.py --selftest`, and `tools/pulse_witness.py` will refuse with exit
2 rather than print a vacuous PASS. The acceptance dive this bench can produce has
`M, total 0`, because `AUTO` requires PREDIVE PASS and PREDIVE's BATTERY step refuses
below `vbatMin` — a board on USB power alone shows HB `vbat=1.56 batlow=1`.

**The floor is no longer the blocker (2026-11-27).** `vbatMin` was a hardcoded 10.5 V,
which on this rig's single 3.7 V cell refused even a fully charged pack (HB `vbat=4.92
batlow=1`); it is now derived from `VBAT_CELLS` in `include/config/defaults.h` — 3.40 V for
one cell — and `tools/pulse_witness.py`'s own copy of the number was wrong in the same way
(it carried `VBAT_MIN = 10.5`) and now reads 3.4 V. What keeps this item open is hardware,
not arithmetic: a pack **with enough capacity to pulse a valve**, metered, and the metered
sense path from §1 P1.

**Witnessed on metal, partly (2026-11-28).** With the fix flashed, this bench boots at
`vbat=3.62…3.65 batlow=0 al=0x0000` and the handset shows no battery marker — the battery
verdict is no longer what refuses. What has **not** been witnessed is a pre-dive run at
all: no pressure on the sensor, the cells are uncalibrated (`CELLS UNCALIBRATED: run CAL
AIR` on the boot line), and `AUTO` still needs the whole PREDIVE sequence, so the remaining
blockers are the ones named above, not the threshold.

**To close it:** P1's bench half first (the reading has to be trustworthy, and the valve needs its
real supply), then `~/.platformio/penv/bin/python tools/pulse_witness.py /dev/ttyACM1`:
exit 0 means the dive file's windows are real on metal. Re-run there 2026-11-28: **exit 2,
vacuous, and structurally so on this rig** — the pre-dive sequence still aborts at the
SOLENOID step (no pressure source to show a rise), which keeps `AUTO` refused by
`ERR predive required`, so no dive injects. The pack being plugged in removed one blocker;
the witness now waits on the pressure rig, not on power.


### P6 — calibration sequencers: implemented, tested, and unreachable

`cal::O2Sequencer` is instantiated nowhere (confirmed by grep over `src/main.cpp` and
`src/comms/`), `CAL O2` and `CAL DEPTH` answer `ERR not implemented`
(`src/comms/cmd_router.cpp`), and `test_designwire` now pins that the design sheet shows
the refusal rather than a success — so no UI can be built against a promise the firmware
does not keep. This is a feature decision, not a defect: it stays open because the code
exists, is host-tested, and does nothing.

**To close it:** decide — instantiate it behind a real command with its own acceptance
leg, or delete the driver. Either way update `test_designwire`, which currently asserts
the refusal.


### P7 — C3 bench items, never done

These need hands and parts, not code. They are listed here so they cannot be lost:

* MS5837 swap and potting.
* The battery divider (§1 P1 has the procedure).
* Mechanical O₂ inhibit, with the board actually injecting.
* Solenoid driver fails-closed rig.
* The handset's screen. `handset/example_esp32_handset/` is complete working firmware —
  buttons, buzzer, link-loss alarm — and deliberately has **no display**: it prints its
  readout to its own USB serial and tells the integrator to swap `draw()` for a real one.
  The 320 × 172 OLED UI in `design/handset-screen/` therefore has no implementation, and
  the design sheet is a handoff.
  **The board is no longer unflashed (2026-11-27), because the handset board is now an
  ESP32-S3.** Measured, not assumed: `esptool` reports `ESP32-S3 (QFN56) revision v0.2`,
  `Embedded PSRAM 16MB (AP_1v8)`, 32 MB flash, MAC 38:44:be:cc:c1:d0, reachable over both
  its USB paths. Built with `-e handset_s3` and flashed over `/dev/ttyACM0`, it boots and
  prints the map it was built with: `pins (ESP32-S3): link rx13 tx14 (controller is tx17
  rx16) | btns ack4 auto5 man6 chk7 dive15 | buzzer 16 | led 21`. Two facts came out of
  that and are recorded in `handset/README.md`, because both are the kind of thing that
  otherwise costs an afternoon: the module's flash is **octal** (the mask ROM says `Octal
  Flash Mode Enabled`), so the core's default quad loader boot-looped on `assert failed:
  do_core_init startup.c:328 (flash_ret == ESP_OK)` until `board_build.arduino.memory_type
  = opi_opi` was set; and the classic handset's pins 25/26/27/32/33 and 19 are flash, USB
  or non-existent on an S3, so the button/buzzer pins moved.
  **The link itself is now witnessed on metal (2026-11-27).** Wired to the controller's
  GPIO17/16, the S3 handset runs the live protocol: the self-test counted 1154 bytes
  arrived in 1.5 s, `T,` and `D,` both came back in answer to the handset's `GET T`, and
  a `TIME SET` typed on the handset console reached the controller and returned as
  `clock +3s` in the handset's own readout — so both directions carry commands, not just
  telemetry. Getting there took the pin sweep, because both failures were copper and not
  firmware: first the controller's TX was in the buzzer hole (16), then the data pair was
  straight through instead of crossed. `handset/README.md` carries that with the readings.
  Still un-witnessed on metal, which is what keeps this item open: the **buttons, buzzer
  and link LED — nothing is attached to 4/5/6/7/15, to 16 or to 21 yet** — and the screen,
  which still has no implementation. With no buttons on the board, the C2 refusal matrix
  runs over the bench bridge — since 2026-11-28 that is *this same S3*, flashed with
  `handset/bridge_esp32`'s `bridge_s3` env and answering `/dev/ttyACM1` (the wire did not
  move: the bridge uses the handset's own 13/14 holes); still a different role from the
  handset image, which is what the C3 legs need back on this board.
* One C3-b leg was stale rather than unfinished: it cross-checked depth "vs BMP085 on the
  same standpipe", and that chip left the rig on 2026-11-23 for never ACKing. **Closed
  2026-11-28**: the leg is re-pointed at what this bench can actually compare against —
  marks on the standpipe read with a tape, each `T,` depth within 0.3 m — with the
  hysteresis half (up and back down through 3 m) kept intact, and BENCH.md now says out
  loud that a second-sensor comparison is not a leg this rig can run. The other live
  references to that part are gone too: PLAN.md's src tree named `depth_sensor.*` and
  `bmp085.cpp`, and **neither file has existed since `1e1e15a`** — the tree now lists the
  seven files that are really in `src/sensors/`. What stays: the dated 2026-11-23 entries
  in PLAN.md and BENCH.md's C3-a paragraph, because those are the record of why depth is
  claimed by identity at boot, and deleting them would delete the reason the gate exists.

**To close them:** BENCH.md's C3 checklist, one leg at a time, each with a measured
number in the log rather than a tick.


### P8 — the config box's 284-byte payload

Batch C's open item. Round E item 7 measured it and deliberately left it **not done**,
decision recorded there. `sizeof(ctrl::Config) = 284`, published as it is; the cost today
is one published copy (284 B) plus three per-task snapshots (852 B) ≈ **1.1 KB of 327 KB**
DRAM and ~0 flash, and `stk=` is unaffected because those copies have been statics since
Round C's copy-placement finding. Narrowing the three snapshots would recover roughly 850 B
— and only the snapshots: the writer path (`hookSettings` → `applySettings` →
`configPublish`) reads carried fields it never writes, so a narrow *published* payload
breaks it. That is why the cost is reviewability rather than size: a narrow `Applied`
struct re-lists fields, and a field a later reader wants then needs a new publish path.
Round E item 7 records the measured per-reader field lists and three shapes.

**To close it:** pick a shape, and anchor it the way the existing anchors do —
`test_configbox`'s `test_no_task_reads_the_old_global` and
`test_only_the_boot_and_the_settings_path_publish` keep the publish/read sites honest, and
`test_settings`'s `test_every_key_reaches_its_own_declared_bounds` is what keeps a key from
drifting out of the box's coverage. A narrower snapshot needs a test that fails when a
consumer reads a field the snapshot no longer carries.


### P9 — the rail the solenoid actually runs on is never sensed, by anything

`vbat` is the **pack**, measured through a divider whose top leg sits on the 3.7 V rail
(`include/config/pins.h`, `PIN_VBAT_SENSE`). The injection valve runs at ~12 V from a
boost converter off that same pack — stated here from the bench description, since no
document in this repo said so (the row below). So the one battery number in the machine —
the number
in `T,`, in HB, in the dive file, behind `BIT_BATTERY` and the pre-dive BATTERY step — is a
verdict about the cell, and says **nothing at all** about whether the coil has voltage.
A pack reading 3.9 V with a dead boost stage, a current-limited one, or one that collapses
under the pulse passes pre-dive, starts the dive, and injects nothing.

| evidence | what it shows |
|---|---|
| `grep -rin 'boost\|12 V' PLAN.md docs/*.md BENCH.md` (2026-11-27) | the only hits are the two sentences this item and BENCH.md's C3-c just added. Before today **no document in this repo mentioned the boost converter at all**, and the architecture it needs — module part, input range, what feeds 3V3 and 5V, coil resistance — is still written nowhere |
| `include/config/pins.h` | one battery sense pin (35), and it is on the pack. Nothing else in the pin map measures a rail |
| the bench link tests, 2026-11-27 | the handset readout carries `FLAGS MAJOR BATT` and HB `vbat=4.92 batlow=1` — the pack number, in the pre-derivation build, refusing a pack that was charged. The 12 V side was never in the picture |

Why it matters in the specific way this repo keeps meeting: it is not a wrong number, it
is an **absent** one, and the decision consumes the number that is present. `pulses=`
counts what the firmware commanded the valve to do, not what the valve did; a coil with no
rail voltage produces a dive file that looks like a working loop with no O₂ in it. The
solenoid's own evidence chain (`control/injector.cpp`'s stuck-open detection, the pre-dive
SOLENOID TEST's ppO₂ rise) is the only thing that would notice, and the pre-dive solenoid
step needs a chamber that can show a rise — this bench has never gotten past step 2 for
that reason (PLAN.md C2).

**To close it:** first write the power architecture down — pack, boost, what feeds 3V3 and
5V, coil resistance, and the boost module's part and input range — because every runtime
number this project has quoted so far (duration, headroom, wire losses) is uncomputable
without it. Then decide between a second divider on the boost output (a real sense input,
one more `T,`-adjacent number, and it must fail closed like `vbat` now does) and the
cheaper bench-only answer: meter it under a pulse and record the margin in BENCH.md's C3
list. What is not acceptable is leaving `vbat` healthy standing in for "the valve has
power", which is what the current docs implicitly do by never mentioning the gap.

**Status 2026-11-28 — the decision half is done; the metal half is not.** The builder
answered the architecture questions (protected 18650; pack → its own 5 V regulator feeds
the board, the boost feeds only the coil; boost module marked "TEAYUEW mini boost module
0260508", chip unidentified; valve Heschen 2V025-08 = AirTAC 2V family, DC12 coil) and
PLAN.md §1 "Power architecture" is the written form, with the coil resistance and the
minimum working differential pressure recorded as **explicit unknowns** rather than
estimates. The chosen direction was the first option above, mirrored on `vbat`:

- `PIN_VRAIL_SENSE` 39 (the last free ADC1 input), divider 100k/10k — the pack's ratio —
  gated by `VRAIL_SENSE` in `defaults.h`, **0 until the divider is fitted**;
- band from ONE rated number (`VRAIL_NOMINAL_V` + the family's ±15% permitted variation):
  floor, plausibility ceiling, the pre-dive question and the alarm all derive from it, and
  a host test pins the derivation the same way the chemistry block's does;
- the verdict is inside the pre-dive BATTERY step as a fourth question — the `PRE,` line's
  passed bit is read by field position (`handset/handset_link.h` `parsePre`), so a sixth
  step could not be appended after it — with a settle deadline in the shape the DEPTH
  grace already uses, and `railWhy()`/`railWaiting()` saying on the console what the wire
  cannot;
- `BIT_BATTERY` for the fault (one marker; "do not trust this power" is one action, and
  the console names which rail), HB `vrail=`/`railstate=` appended after `cfggen=`;
- LOW is debounced by *sweeps*, not ADC reads or control cycles — a 300 ms pulse lands on
  about one 1 Hz sweep in seven, and a nuisance battery alarm teaches a diver to ignore
  the one marker that means the power is untrustworthy;
- the not-sensed build is loud: boot line, `rail:` console edge, HB `vrail=0.00
  railstate=3`, and a host test pins each sentence so the absence is never re-read as a
  pass. The `hw_accept.py` batch-E legs grade shape and the coherence rule ("not sensed"
  must not report a voltage) but **not** the value.

What remains for P9 is metal, and it is written as BENCH.md C3-c legs: fit the divider,
meter the rail at the coil (standing and under a pulse), measure the 2V025-08 coil's DC
resistance, and settle whether it opens at ambient differential pressure at all. The
±15% band is a documented rating used as a placeholder verdict until that measurement
replaces it, and `T,`/dive-file publication of the rail stays deferred until the rail is
metered — publishing an unmetered number is how `vbat` became P1.

**Update the same day — the chain is sim-witnessed.** The PC rig (`src/sim/rig.*`,
graded by `test_sim`) now feeds a scripted coil rail through the **real**
`sensors/rail.h` at SensorTask's own 1 Hz sweep rate, scripted with
`Rig::setRail()` — `0` is a dead boost, and pulse-sag is deliberately **not**
simulated because its figure is one of the C3-c measurements the bench owes. Three
scenarios: a judged rig kills its boost mid-dive and `BIT_BATTERY` rises **while the
pack number stays healthy and pulses keep counting** (the exact §1 P9 failure, now
a runnable demo); pre-dive BATTERY refuses and names the reason
(`coil rail never settled back inside its band` — the assertion on the words is what
makes the leg mutation-sensitive: breaking the feed changes the sentence, not just
the verdict); and the un-judged build — today's board, mirrored — sees none of it,
never even sampling the pin. All three mutation-checked (feed removed, alarm flag
removed, sweep gate removed — each reddened its named leg). C3-c leg 5's metal run
is now a rehearsal with the exact sentences to watch for, not a first sighting.


### P10 — a bus error line the bench cannot attribute, and an observation that does not exist

The console prints `requestFrom(): i2cWriteReadNonStop returned Error -1`. The line names
**no bus and no address**, so the bench cannot tell a probe NACK from a fault NACK — which
is how a real intermittent I²C fault gets tuned out on a bench that has learned to ignore
that line.

**Counted, not reasoned, 2026-11-28.** Every copy of this line in this repo's logs, and
what each one is:

| where | how often | what it is |
|---|---|---|
| `logs/accept_20261127_122109_console.log`, `_151346_console.log`, `_151848_console.log`, `accept_20261128_095753_console.log` | **exactly once per boot**, immediately before `depth: LPS28DFW @0x5D ready` | the two-address identity probe at `src/sensors/lps28.cpp:15` walking `{ADDR, ADDR_ALT}`: this board's XSA pad is high, so `0x5C` NACKs and `0x5D` answers. Expected, benign, and the price of a probe that decides the address instead of assuming it |
| `logs/soak_20261122_094738.log` | **pairs, 17 ms apart, every ~25 s**, at uptime 29 573 s — eight hours in | the depth arbiter recirculating (`src/sensors/depth_manager.cpp:26-30`: `RECIRCULATE_MS`) with **no depth chip fitted**: that soak is 2026-11-22, one day before the LPS28DFW went on the bench. Two NACKs per cycle is the MS5837 pair. Working as designed |
| a line said to recur "every ~6 s while `i2cerr=0 fresh=111`" | **not in any log here** | an observation from a console tail that was never captured. The cadence does not appear in the four acceptance captures or the 24 h soak; the repo cannot check what it did not keep |

So the two things that were guessed about this line are both closed by the log: it is not
the depth chip that used to hold this bench slot (**absent from `lib_deps`, absent from
`~/.platformio/lib`, and `grep -c BMP085 .pio/build/esp32/firmware.map` is 0**), and on a
bench with a fitted sensor it is a boot-time probe, not a heartbeat.

**What is still owed, and it is not the same item:** an un-captured console observation
cannot be closed or kept, so the bench half is to run `tools/soak_logger.py` over a healthy
24 h and count the line with its timestamps. If it recurs on a locked source, that is a new
finding and the arbiter, not the probe, is where to look. The tooling half stands on its
own: the message comes from the Arduino core (`Wire.cpp:499` in this core), which cannot be
made to name our bus, while the two `requestFrom` call sites in this repo are
`src/sensors/wire_transport.h:64` (every depth read on both buses) and
`src/sensors/ads1115_bus.cpp:57` (the ADS1115). Naming bus + address at the transport turns
"that line again" into an attributable event. A host test can pin the sentence; whether it
ever fires needs the console.

---

## 2. The finding index

One row per finding. The review's own labels are **M-2, M-3, m-1, m-2, m-4 and N1**; the
remaining items it produced are named by their subject, because that is how the rounds
named them and inventing an id here would be a label nothing else uses. "Status" is the
current verdict; "Resolution" is what the repo now does about it. The detail — symptom,
fix, tests, and what is still not proven — is in the round's section named in the last
column: **§A … §E are the five round sections in §3**, and §D1 means Round D's item 1.

| finding | what it was | status | resolution | section |
|---|---|---|---|---|
| **M-2** | the task watchdog was never initialised; "3 s" was a fiction | **closed** | one `esp_task_wdt_init(3, true)` in `setup()` (`src/main.cpp:1627`), its result checked and announced on failure; nothing in the linked image initialises the TWDT earlier, and it subscribes no idle task; the 3 s is now a measured 3011–3061 ms | §A, §D1 |
| **M-3** | the FLAGS byte dropped `CNS_HIGH` in **both** duplicate mappings under a comment claiming the aggregate covered it; and bit 6 was labelled `SENSOR` on a cell-proof failure | **closed** | one projection — `comms::flagsFromAlarmWord` in `src/comms/flags.h` — called by both transports, CNS appended as bit 7 with no renumbering; `test/test_flaglabels` grades all four vocabularies against each other | §A, §D2 |
| **m-1** | two 32-bit wrap bugs whose failure direction was a *lie*, not a hazard: the injector duty window, and the warm-up gate's deadline | **closed** (the gate's fix was itself wrong — §5) | wrap-safe signed comparison, native test with the sign pinned by mutation; the gate's logic now lives in `util::WarmupGate`, host-tested | §A, §E4 |
| **m-2** | values legal per key that were inverted *together*, silently — `GF_LOW=90`+`GF_HIGH=50`, `D1=40`+`D2=15` | **closed** | ordered/clamped in `settings_apply.cpp` with per-pair result bits visible on the console; host-tested | §A |
| **m-4** | the published deco view had no ordering guarantee between payload and sequence number | **fix verified live; necessity open (§1 P4)** | seqlock with fences; the torn read is witnessed on metal; the fences are 1 `memw` each on xtensa (measured) and invisible on x86 (measured) | §A, §B1, §D1, §E5 |
| **N1** | the freshness rule was stated in `deco/view.h` and broken by the order of two statements | **closed** | the event ring is staged and committed atomically at the tick's end; a native test proves an event cannot be read against a plan it was not computed from | §B1 |
| **N1, the row beside it** | an event-driven dive row's picture could come from a different cycle than its own fields | **closed** (a second bug in this same code found in E) | `EventRing::beginTx()`/`commitTx()` staging, and `bridge::EdgeSnap` + `bridge::captureEdge` taken before the first row of a drain, handed to `Hooks::appendSnapNow` — one instant per drain batch | §B2, §E1 |
| **"a value out of range reads as *no obligation*"** | int16/uint16 narrowings that wrapped | **closed** | narrowing saturates at every site that feeds a decision, each with a host test; `ndl` is `-1` on the only rows that have no picture | §B3, §B4 |
| `g_ctrlCfg` double-buffering (batch B's open item 2) | `postConfig()` copied a struct while ControlTask read it | **closed** | the applied rules are one generation — `src/control/configbox.h`, a per-reader private copy, and a third publisher refused with the skew printed on HB | §C, §D4 |
| the 3 s trip time was unmeasured (batch B open 1) | a detector whose trip time nobody knew | **closed as a measurement** (headroom open, §1 P3) | measured 3011–3061 ms on metal, CPU0 idle monitored; the mechanism the review proposed was wrong (§5) | §D1 |
| flag labels duplicated in three places (batch A residual 5) | hand-editing three files to fix a label, which re-breaks next time one is touched | **closed** | `test/test_flaglabels` grades agreement across `comms/flags.h`, `web_page.h`, the flashed `data/index.html` and §4 of the spec | §D2 |
| dive-log rounding inconsistent with the printed digits (batch B open 6) | `* SCALE + 0.5` while the repo's rule is `numfmt::scaled` | **closed** | one rounding rule at 4 call sites, both range-gated | §B, §D3 |
| the design sheet's mock wire lines | a design that showed success the firmware refused | **closed** | the mock lines are graded as wire lines; `test_designwire` pins that the design shows `ERR not implemented` | §D5 |
| the clamp verdict was console-only (batch B open 2) | `GET GF_LOW` answered what was typed while the chain ran the clamped pair | **closed** | event id 19 carries the verdict; witnessed end to end on metal | §B2, §E1 |
| `pulses` on event rows was always 0 (batch B open 4) | a column read and written nowhere | **closed** (metal witness open, §1 P5) | real per-cycle windows, one owner, `M,` the only total | §E2 |
| `LOG EXPORT` dive-lock policy undecided (batch B open 5) | the policy was open | **closed as a policy** (the web route's margin is §1 P2) | refused mid-dive, with the refusal witnessed; `/dive/N.csv` deliberately left unlocked | §E3 |
| the warm-up gate had no native coverage (batch A residual) | a safety-relevant gate tested only by reading it | **closed** | `util::WarmupGate` is host-tested, and reading it found a second bug (§5) | §A, §E4 |
| are the seqlock fences load-bearing? (batch A residual) | a claim with no measurement either way | **measured at both ends; necessity still open, §1 P4** | `env:native_o2` gate + `tools/fence_probe.cpp` counterfactual | §E5 |
| the config box's payload size (batch C open item) | unmeasured size, undecided shape | **open by decision (§1 P8)** | measured: 284 B, ~1.1 KB for the box; three shapes and the recommendation recorded | §E7 |
| the three dead `eraseAll()` functions (a nitpick) | dead code in a safety-relevant store | **closed** | deleted, with the reasoning recorded | §A |
| this bench, 2026-11-27 | `vbat=34.71` on a 5 V feed, and no bound anywhere to notice | **firmware half closed (2026-11-27); the metering half open, §1 P1** | `VBAT_IMPLAUSIBLE_V` + `Battery::implausible()` raise `BIT_BATTERY`, refuse the pre-dive BATTERY step and print which fault it is; `tools/pulse_witness.py` refuses above the ceiling too (7 captures, `--selftest`). What is NOT closed: no acceptance leg compares `vbat=` to a meter, and `VBAT_CAL` has never been trimmed | new |
| the pack chemistry was frozen into six places, and one of them was wrong | a 12 V-style `VBAT_MIN` 10.5 V sat in `defaults.h` while the rig ran a single 3.7 V cell: it refused the pre-dive BATTERY step permanently (HB `vbat=4.92 batlow=1` on a charged pack), so `AUTO` could never unlock, and the same 10.5 was copied into `predive.h`, `alarms.h`, the settings default/range, the test fixtures and `pulse_witness.py` | **closed** (2026-11-27) | one number — `VBAT_CELLS` in `include/config/defaults.h` — derives the floor, the ceiling, the settings band and the sim rail; witnesses: `VBAT_CELLS` 1 → 2 reddens only the published-band legs, and with `docs/HANDSET_PROTOCOL.md` §6's one row updated gives 448/448 native, 896/896 both levels and a clean `env:esp32` link; a new settings-table join (`test_the_published_settings_table_matches_the_registry_it_documents`) now grades all 36 keys of §6 against the registry in both directions, and the tightened band is what clears the stale 10.5 out of NVS on load without a schema bump | new |
| a series pack would have been judged by one number | 6.80 V is compatible with 4.00 + 2.80 — one dead cell behind a passing voltage gate — and nothing in the tree could see the difference | **closed as far as the bench allows** (2026-11-27) | `PIN_VBAT_MID_SENSE` (GPIO36) plus the pre-dive agreement step (`VBAT_CELL_SPREAD_MAX_V`), which refuses an unwired midpoint rather than passing it; the wiring itself is a C3-c leg with nothing attached yet | new |
| the rail the solenoid actually runs on is sensed by nothing | `vbat` is the 3.7 V pack; the valve runs at ~12 V off a boost stage nothing measures, so "battery healthy" was standing in for "the coil has voltage" — and no document mentioned the boost converter at all | **decision done, metal open, §1 P9** | architecture written (PLAN.md §1 "Power architecture", with coil resistance and minimum working differential pressure as named unknowns); the sense path is implemented — GPIO39, band derived from the coil's rated 12 V ±15%, fourth question inside the pre-dive BATTERY step, `BIT_BATTERY`, HB `vrail=/railstate=` — and compiled behind `VRAIL_SENSE=0` until the divider is fitted, loudly announced while off. Fitting + metering legs are BENCH.md C3-c | new |
| the stall warning and the stall reboot were the same number written three times | `HB_WORST_CYCLE_MS = 800` in the bench tool, `TASK_STALL_CONTROL_MS 1500` in `defaults.h`, and a literal `1500` inside the bench check's own output text — a comment claimed the first two were related and nothing enforced it, so a firmware budget change would have left the console grading an unrelated threshold in silence | **closed as a decision; the metal half still open, §1 P3** | the tool reads the budget out of the header at import (and warns on stderr if it cannot), the check's words print what it read, and `--selftest` pins the relationship both ways: grade ≥ budget reddens, grade < 30% of budget reddens (floor from this board's own 505 ms record), a renamed define reddens the join, a look-alike define is refused, and the literal coming back reddens a source check. Mutations: 800→2000 reddens 4 cases, 800→100 reddens 2, rename reddens 1, literal back reddens 1, `<=`→`<` reddens the exactly-at-grade fixture | new |
| the boot leg accepted the NVS migration sentence as the whole witness | `settings: NVS VBAT_MIN="10.500" refused … the registry default for that key is used instead` was graded as "the boot is clean". A board that prints that refusal **and still runs 10.5** passed; a board that accepts the out-of-band value **in silence** also passed. Both are the failure the sentence exists to prevent | **closed — host-side and on metal, 2026-11-28** | the matrix reads the key back (`GET VBAT_MIN`) and joins the refusal against the band and default on the board's own `GET SCHEMA` row — no number copied into the tool — over eight fixtures (refuse-and-still-run, refuse-and-run-a-third-value, the sentence losing its tail, a refusal naming an unpublished key, acceptance in silence, a refused key never read back, plus honest and clean boots that must stay green), and one case fails if no acceptance run reaches the grader. Driving the leg out also found that `GET SCHEMA` answers 36 lines and the matrix's first-match window stopped after the first, letting 35 smear into the next check's window — the `GET *` amputation again, now `collect`. **The metal half was driven 2026-11-28** by seeding it, because this bench's NVS is clean: `SET VBAT_MIN=4.400` (legal inside 2.80…4.50), a temporary build with `VBAT_CELL_RANGE_MAX_V 4.30f` (bench seed, reverted, never committed), flash — boot refuses, and six join checks run on real numbers, 123 checks 0 failures (`logs/accept_20261128_nvs_migration_seed_123p_0f.log`). The red direction, refused *and still running*, stays host-only: that is a firmware mutation, not a bench state | new |
| `M,`'s pulses was the dive's total only in the spec | the dive file's summary column claimed the dive (docs say so) and `DiveTotals.totalPulses` was fed the injector's **boot-cumulative** counter with no base: the pre-dive solenoid test's 3 pulses were charged to the first dive while every `S,` window correctly said 0 — exactly the wall-of-zeros-beside-a-nonzero-total shape the bench's pulses check hunts. It had never been sighted because this bench's pack was unplugged, so the counter never left zero | **closed** (2026-11-28, bench-first) | one base captured at the dive edge, one subtraction (`main.cpp`). `lastPulseBase` cannot double as it: that one advances every cycle to feed the `T,`/event-row window. `main.cpp` is out of the host build's reach, so the witness is the wire and it runs both ways: pre-fix firmware reddened the check on hardware twice (`logs/accept_20261128_183319_*`, `_184225_*`: `M,` total 3, `S,` windows [0,0,0]), post-fix the full battery is 181 checks 0 failures (`logs/accept_20261128_185837_*`) | new |
| a C3-b leg and PLAN.md's src tree still named depth parts that are not on the bench | the leg told the next operator to cross-check depth "vs BMP085 on the same standpipe" — a comparison this rig cannot run since 2026-11-23 — and the plan's tree listed `depth_sensor.*` and `bmp085.cpp`, **neither of which has existed since `1e1e15a`** | **closed** (2026-11-28) | the leg is re-pointed at the reference the bench actually has (tape marks at 0.0/2.0/5.0 m, each `T,` depth within 0.3 m, hysteresis through 3 m kept) and says out loud that a second-sensor comparison is not runnable here; the tree line now lists the seven files really in `src/sensors/`; the dated 2026-11-23 records in PLAN.md and BENCH.md's C3-a paragraph stay, because they are the reason depth is identity-gated at boot | new |
| a bus error line the bench cannot attribute, plus an observation that was never captured | `requestFrom(): i2cWriteReadNonStop returned Error -1` names no bus and no address, so a probe NACK and a fault NACK look identical — and the "every ~6 s on a healthy bench" version of it, repeated in this session's own notes, is **not in any log the repo keeps** | **open (the tooling half), §1 P10**; both guesses about the line closed by counting | every copy in the repo's logs identified: once per boot in all four acceptance captures, immediately before the depth announce — the two-address identity probe (`src/sensors/lps28.cpp:15`) on a board whose XSA pad puts the chip at `0x5D`, so `0x5C` NACKs by design; and pairs 17 ms apart every ~25 s in `logs/soak_20261122_094738.log` at uptime 29 573 s, which is the arbiter recirculating one day **before** the LPS28DFW was fitted. Not the removed chip: absent from `lib_deps`, from `~/.platformio/lib`, and **0** `BMP085` symbols in `firmware.map`. Owed: a timestamped soak count, and a transport-level message that names bus + address (the core's line at `Wire.cpp:499` cannot) | new |
| the acceptance matrix's reply window was a per-row custom, and a third multi-line answer walked into it | `GET *` had the amputation in 2026-11-26 — first-match stops at the first line of a 36-line reply, and the tail then answers the NEXT command — `GET SCHEMA` got its marker today, and `GET HIST` (one `HIST,` line per cell, `src/control/cellhist.h:62`, all of it a single `write()`) did not. Two of three rows patched by hand with the rule kept in a comment: the third one was still waiting | **closed** (2026-11-28): the rule lives in one reader (`ReplyWindow`) and a row declares `family:<regex>`. The window drains the family, steps over the controller's volunteered `T,`/`D,`/`E,`/`HB ` lines, and pushes back the first line belonging to neither — pushback is what makes draining safe instead of merely thorough. The old `collect` marker, which drained to silence and could not push back, is retired. Nine host cases grade **two consecutive windows** (what the first consumed, what the second found), including the defect reproduced on purpose: with no family, two `HIST,` lines answer `GET VBAT_MIN`. Five mutations, five reddened — drop a family marker, drop the pushback, end the drain at interleaved telemetry, remove the family boundary, restore `GET *`'s old shape. A sixth mutation attempt reported zero failures and meant nothing: it was a syntax error, so the suite never ran; re-run as a valid row it reddened. `--selftest` is 179 cases, the extra three being the replay guard below. **The rule then failed its first live drive (2026-11-28, over the S3 bridge): the drain's only exit was a silent line, and a bench is never silent** — the first family row (`GET *`) was still inside its drain **801 s** into a matrix-only run (`logs/stall_repro_run2_stdout.log`, last console line `[801.32s]`, its handset phase log never written), and two earlier full batteries (started 17:13:53 and 17:56:54) were killed with the console log frozen at its 2260 B boot flush and no phase log at all. Proven from outside first — `ptrace` tools were unavailable, so the witness is `/proc/<pid>/io` deltas: ~170 B/s read (the volunteered stream), no writes to the handset port — then read back from the code, one unbounded loop. Host fixtures could not have caught it: they run out and answer `""`, which is the exact silence a bench withholds. The drain is now bounded twice — the window's deadline, and at most 10 consecutive stepped-over lines (a family is ONE `write()`; at 2 Hz telemetry, ten volunteered lines means it is over) — and the tenth case drives a 4000-line never-quiet flood, its bound asserted as a literal so widening the constant cannot mutate the test green. Three more mutations, three named reddens: limit→10⁹, restore-the-unbounded-loop, remove-step-over. | new |
| `--replay` with an unreadable path meant "no replay" | truthiness on a path argument: `--replay ""` (an empty shell variable, 2026-11-28) fell through to the live branch and opened the controller console, whose CP2102 reset pulse rebooted the board mid-session — the rule that says do not open that port was prose, and a typo walked through it | **closed** (2026-11-28): an unreadable `--replay` exits 2 with the reason, the replay test is now `is not None`, and it sits before `import serial`. Three cases: one graded as a subprocess on a non-existent path (deliberately not `""` — a selftest must never be able to open `/dev/ttyUSB0`), two by source scan. Reverting the guard reddens all three | new |


---

---

## 3. The five rounds, in order

Each round is carried through **verbatim** from the document it was written in, with its
base commit, its gate counts and its bench state as they stood that day. Read them for the
*why* — the symptom, the reasoning about the failure direction, what the tests pin, and
what the round itself said it had not proven. Read §1 for what is open today: a round's own
"still open" list is a snapshot of that afternoon, and each one is marked as such.

The provenance of every number below is the round that measured it. `437` native cases,
`874/874` across `env:native` and `env:native_o2`, `158 PASS`, `3011–3061 ms` — each is
quoted where it was taken, and §6 says what was re-run after this merge.

### Round A — what was fixed first: the watchdog that did not exist, the flags byte, and two wraps whose failure direction was a lie

Scope: findings **M-2, M-3, m-1, m-2, m-4** and one nitpick from the 2026-11-27
review. Bench state: base `038df54`, native suite **398/398** (was 393),
`pio run -e esp32` and `pio run -e esp32_stalltest` both SUCCESS, no new
warnings from the touched sources. Nothing committed; `graphify-out/` left
untracked and untouched. **No hardware was flashed or read** — everything below
is code, static SDK evidence and native tests.

---

#### M-2 — the task watchdog was never initialised, and the 3 s was a fiction

**Symptom.** `esp_task_wdt_init()` was called nowhere, so the TWDT the four
registered tasks (`main.cpp:163` sensor, `:279` hud, `:601` control,
`uart2_task.cpp:41`) expect did not exist: a stalled task would spin forever
instead of dumping backtraces and panicking into the TASK_WDT boot path that
drives SAFE mode.

**Deviation from the suggested fix.** The review proposed
`build_flags = -DCONFIG_ESP_TASK_WDT_INIT=0`. That flag is a **phantom on this
toolchain**: the SDK `env:esp32` compiles against (Arduino core 2.1.7,
ESP-IDF v4.4-style `tools/sdk/esp32`) has no consumer for that macro at all.
Adding it would have been a second no-op, and "correcting" the call to the IDF
v5 `esp_task_wdt_config_t` signature would not compile here. So the fix is an
**explicit, checked init** instead.

**Fix.** `src/main.cpp:1295-1301`

```
const esp_err_t wdtErr = esp_task_wdt_init(3, true);
if (wdtErr != ESP_OK)  Serial.printf("** TWDT init FAILED (%s) …");
else                   Serial.println("TWDT: 3 s timeout, panic on  (registered: sensor, hud, control, uart2)");
```

Ordering is correct: init at `:1295`, tasks created at `:1360-1366` and
`uart2_task.cpp:87`, so every `esp_task_wdt_add()` runs against an initialised
watchdog. The result is printed because a detector that is off in silence is
indistinguishable from a detector that never fires.

**Evidence chain** (recorded in the comment block `src/main.cpp:1258-1294`, so
the next reader can re-check it rather than trust prose):

1. `tools/sdk/esp32/qio_qspi/include/sdkconfig.h` defines `CONFIG_ESP_TASK_WDT`,
   `CONFIG_ESP_TASK_WDT_PANIC`, `CONFIG_ESP_TASK_WDT_TIMEOUT_S 5` and
   `CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU0` — and **not**
   `CONFIG_ESP_TASK_WDT_INIT`. No header in that SDK reads the macro.
2. `nm -U` over every archive in `tools/sdk/esp32/**/lib` finds **no reference to
   `esp_task_wdt_init`**; `firmware.map` lists `src/main.cpp.o` as the sole
   puller of `task_wdt.c.obj`. Nothing boots the watchdog before our line.
3. `objdump` of `task_wdt.c.obj` shows the already-initialised branch rewriting
   `ctx->timeout`/`ctx->panic` and re-arming the hardware
   (`wdt_hal_disable/config_stage/enable`) rather than returning early — so a
   second init **reconfigures** instead of no-opping.

**Consequence of 2 + 3:** the SDK's 5 s is the interval the **CPU0 idle task**
is subscribed with; the timeout every task registered below is checked against
is the **3 s** raised here. Comments that claimed the watchdog "re-initialises"
were corrected: `src/util/wdtfeed.h:45`, `src/comms/uart_link.cpp:30`.

**Not verified on hardware.** The 3 s trip has not been measured on the bench.
Recommended: stall the control task (`while(1);` behind a build flag) and time
the reset from serial, confirming the reboot reason reads TASK_WDT.

---

#### M-3 — CNS overload was dropped on both transports

**Symptom.** Both duplicate flags-byte mappings aggregate alarm bits
3/6/7/9 into FLAGS bit 3 and their comment claimed the aggregate covered CNS —
it did not: `BIT_CNS_HIGH` (alarm bit 8, a **WARNING** level) fell through both
functions, so a diver over his oxygen budget was invisible on `T,` and on the
web, while a *sensor fault* was the only thing that bit 3 ever meant.

**Fix — append-only, bit 7 only, no renumbering, no `row_format` bump:**

| edit | file:line |
|---|---|
| `if (a & (1u << alarms::BIT_CNS_HIGH)) f |= 1 << 7;` | `src/comms/uart_link.cpp:155` (inside `emitT`, `:124`) |
| the same line in the duplicate mapping | `src/comms/cmd_router.cpp:192` (`telemetryFlags`, `:180`; feeds `GET T` **and** the web) |
| comment rewritten to name the 3/6/7/9 aggregate and say what was dropped | `src/comms/cmd_router.cpp:170-179`, `src/comms/uart_link.cpp:145-153` |
| `Telemetry::flags` field comment gains bit 7, append-only noted | `src/comms/uart_proto.h:120-123` |
| §4 FLAGS table gains `| 7 | CNS over the diver's alarm limit (added 2026-11-27) |` + narrative that names the drift direction | `docs/HANDSET_PROTOCOL.md:378` and the paragraphs at `:380-397` |
| web pill labels corrected | `src/comms/web_page.h:352` and `data/index.html:326` (re-synced with `tools/sync_webdata.py`) |

The two label bugs found while doing this: index 6 was labelled **SOLENOID**
(bit 6 has always been cell-proof FAIL; a stuck-open valve rides in bit 3) and
index 7 was labelled **PROOF FAIL**, which is what bit 6 means. Both corrected,
and bit 7 renders as a *warn* pill, matching the alarm's own WARNING level
(`i<3||i===6?"bad":"warn"`). The alarm word itself is untouched — the `E,*,1/2`
rows still carry the same 10 bit ids.

**Tests (new).**
- `test/test_uartlink/test_uart_link.cpp:259`
  `test_cns_high_gets_its_own_flag_bit_and_stays_out_of_the_fault_aggregate` —
  CNS alone ⇒ exactly bit 7; a stale sensor ⇒ bit 3 with no invented bit 7;
  CNS + stale ⇒ both.
- `test/test_router/test_router.cpp:574`
  `test_get_t_projects_cns_exactly_as_the_pushed_stream_does` — the two
  duplicate mappings must agree, so the pull path is pinned against the pushed
  stream rather than against a hand-written expectation.

---

#### m-1 — two 32-bit wrap bugs whose failure direction was a *lie*, not a hazard

**`Injector::dutyPct`** (`src/control/injector.cpp:17-39`). The window test was
`nowMs >= p.startMs && p.startMs + windowMs >= nowMs`. Both unsigned comparisons
go false for the first window after the `millis()` wrap, and the second can wrap
round past `nowMs` on its own. Direction: **duty reads 0%** — "the solenoid has
not run at all this window" — about a machine that had been injecting normally.
Now: `const int32_t ageMs = static_cast<int32_t>(nowMs - p.startMs);` with
`ageMs >= 0 && ageMs < windowMs` (`:32`), the signed-difference form the rest of
the codebase already uses. Exact for any gap under 2^31 ms; a 10 s window is
four orders of magnitude inside that.

**CAL AIR warm-up gate** (`src/main.cpp:682-691`). `now < PREDIVE_WARMUP_MS`
became true again for the first 120 s after the wrap (49.7 days of uptime — a
soak or an un-power-cycled bench board reaches it), refusing an air calibration
with "cells still warming up" on a machine two months past warm-up. Now
`static_cast<int32_t>(PREDIVE_WARMUP_MS - now) > 0` (`:685`), and the seconds
left are printed from the signed value. Direction: a **refusal with a wrong
reason** — safe, but indistinguishable from the honest refusal, which is why it
counted.

**Tests.** `test/test_injector/test_injector.cpp:93`
`test_duty_window_is_wrap_safe`, plus `:272` registration. **Negative check
run:** reverting `dutyPct` to the old unsigned form made that test fail
(observed, then the file was restored) — so the case is load-bearing, not
decorative. The warm-up gate itself has no native coverage (see residuals).

---

#### m-2 — legal-per-key values that were inverted *together*, silently

Registry ranges are per key, so `GF_LOW=90` + `GF_HIGH=50` and `D1=40` +
`D2=15` both pass validation and then reach the chains inverted.

**Fix (`src/comms/settings_apply.cpp`, declarations in `.h:38-56`):**
- **GF pair** (`:116-134`, `CLAMP_GF_ORDER = 1 << 5` at `.h:40`): when
  `gfLow > gfHigh`, **both** keys take the stricter edge (`gfLow := gfHigh`).
  A swap was rejected deliberately: the gradient line interpolates from the
  fast compartment at `gfLow` up to the slow ones at `gfHigh`, so swapping
  would leave the **permissive** factor governing the tissues that load first —
  the direction that lengthens the ceiling a diver is allowed to skip.
- **Depth band edges** (`:42-60`, `CLAMP_BAND_ORDER = 1 << 6` at `.h:41`): the
  engine reads `d < D1` as the shallow band and `d > D2` as the deep band, so
  crossed edges make every depth past the deeper edge select **band 0** — the
  lowest setpoint and shortest deco in the deepest water. Here a **swap** is the
  right repair (`std::swap`, `:58`): both depths the diver entered survive.
  Equal edges collapse the middle band to nothing but do not oscillate — the
  hysteresis is measured against the same edges it switches on — so nothing is
  invented to prop the middle band up.
- **Verdict is named, not masked.** `clampName()` (`.h:49`) and `clampList()`
  (`.h:55`) map each clamp bit to a comma/space-free name; printed at boot
  (`src/main.cpp:1316-1327`) and on every SET (`src/comms/session_wiring.cpp:287-305`).
  Chosen over inventing a new event id or protocol field: the registry keeps
  what the diver typed, so `GET GF_LOW` answers the typed number and only the
  verdict says the chain is running something else.

**Tests.** `test/test_apply/test_settings_apply.cpp:112-161` (crossed GF ⇒ both
keys at 50 and `CLAMP_GF_ORDER`; legal GF pair ⇒ `CLAMP_NONE`; crossed bands ⇒
swapped and `CLAMP_BAND_ORDER`; equal/legal bands ⇒ `CLAMP_NONE`), `:243`
`test_clamp_verdict_is_readable` (graded: every clamp bit has a name, the name
contains no comma or space, and an unknown bit must not borrow somebody else's
name), and `test/test_router/test_router.cpp:546`
`test_cross_key_sets_are_clamped_before_they_reach_the_chain` (the SET path, not
just the applier).

---

#### m-4 — the published deco view had no ordering guarantee between payload and seq

`src/deco/view.h`: `publishView` (`:131`) writes the payload and then the
sequence, and `viewSnapshot` (`:146`) re-checks the sequence after copying the
payload. Neither side was ordered relative to the other field. Added:
- **release fence** after the payload copy, before the seq store — `:140`.
  Without it a compiler may hoist payload stores past the seq store, and a
  reader can accept a generation whose payload was written *after* the seq it
  matched.
- **acquire fence** between the payload read and the second seq read — `:157`.
  Without it the plain payload loads may be reordered before the seq check, so
  the seq recheck no longer says anything about what was actually copied.

Result: a torn `D,` line (ceiling from one generation, stop list from another)
is excluded by the memory model rather than by luck. Both edits carry the WHY
and the direction of the drift.

**Coverage.** `test/test_controlplane/test_controlplane.cpp:324`
`test_the_seqlock_publishes_generations_never_torn_pairs` already runs a real
two-thread publish/read race (2000 generations, invariant `ceilingCm == ndl*10`
and `stopDepthCm[0] == ndl*100`). Note the honest limit: on x86 `-O0` the
compiler does not reorder, so no native test can *observe* the fences; the test
proves the shape stays torn-pair-free, the fences are the portable guarantee.

---

#### Nitpick — three dead `eraseAll()` functions

`DiveStore::eraseAll`, `SurfaceStore::eraseAll`, `ContinuityStore::eraseAll`
deleted (30 lines, `git diff --stat src/log/`): no caller in `src/`, `test/` or
`tools/`. `BENCH.md:96-101` already documents the real route (esptool
`erase_region 0x290000 0x160000` over the LittleFS partition, then the mount
that formats it), so removing them removes an attractive-nuisance that would have
silently wiped dive history from a debug build.

`DiveStore::exportRaw` remains dead — out of batch scope. `pendingRecord` is
**not** dead: `corrupt()` uses it.

---

#### Gates and test count

| gate | result |
|---|---|
| `pio test -e native` | **398/398 PASSED** (base 393, **+5** cases) |
| `pio run -e esp32` | **SUCCESS** — RAM 16.9% (55428/327680), Flash 78.9% (1034349/1310720) |
| `pio run -e esp32_stalltest` | **SUCCESS** |
| warnings from touched sources | none |

New cases: `test_duty_window_is_wrap_safe`, `test_clamp_verdict_is_readable`,
`test_cross_key_sets_are_clamped_before_they_reach_the_chain`,
`test_get_t_projects_cns_exactly_as_the_pushed_stream_does`,
`test_cns_high_gets_its_own_flag_bit_and_stays_out_of_the_fault_aggregate`.
No case was removed or weakened.

---

#### Residual risks — superseded by §1; kept as the 2026-11-27 snapshot

1. **3 s is now real.** Every registered task is checked against it, and
   `CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU0` means the CPU0 idle task is
   subscribed too — nothing feeds idle. Worst measured control cycle is ~288 ms
   and the LOG EXPORT row gap is ~0.15 s (`util::wdtFeedDue`/`wdtFeed`), so
   today's headroom is ~10x, but a long `LOG EXPORT` or web response that
   monopolises core 0 for >3 s will panic-reset into SAFE. The trip is **not
   measured on hardware** — do it once on the bench before trusting it.
2. **The clamp verdict is console-only.** No protocol field and no event id:
   a handset still sees `OK GF_LOW` and `GET GF_LOW` still answers the typed
   value while the chain runs the clamped one. Fixing that properly needs a
   protocol addition (§4/§8) that this batch deliberately avoided.
3. **m-4's fences are unobservable natively** (x86 `-O0` never reorders). They
   are compiler-level guarantees; only an ARM compiler at `-O2`/`-Os` could
   demonstrate the difference, and no such build is in the gates.
4. **CAL AIR warm-up arithmetic lives in `main.cpp` with no native coverage.**
   An extraction into `src/util/warmup.h` (so `warmupLeftMs` is testable in
   `test/test_predive/`) was implemented but did not survive concurrent edits to
   the working tree; the gate is correct as written, unverified by tests.
5. **Flag labels are duplicated in two places.** `FLAGNAMES` in
   `src/comms/web_page.h` and `data/index.html` must be kept in step with §4 by
   hand (`tools/sync_webdata.py` propagates one from the other, and was re-run);
   nothing tests that the doc table and the JS array agree, and no test parses
   §4's rows. Adding bit 8+ later needs all three places.
6. **Dead code left:** `DiveStore::exportRaw`.
7. **Working tree drift.** While this batch ran, the tree was edited
   concurrently; the report describes the state verified on disk
   (2026-11-27, base `038df54` + the diff in `git status`), not an intermediate
   one. Re-verify with `git diff` before committing anything from it.

#### Status of these residuals, as of Round E (same day)

Batch Round E worked this list. Recorded here so the
list is not read as current:

* **2 — closed.** The clamp verdict is protocol now: `E,<t>,19,<mask>`, mask =
  `comms::ApplyClamp`, appended to §4 without renumbering anything (batch E §1).
  `GET GF_LOW` still answers the typed value — §6 says so on purpose.
* **3 — half closed, and the wording here was too optimistic.** `env:native_o2` is
  in the gates now, and the fence-removal mutation **still passes at `-O0` and
  `-O2` on x86-64**: this host cannot witness those fences at all (their
  `fetch_add(acq_rel)` is a `lock xadd` barrier by itself). On the shipping core
  they are one `memw` instruction each, measured in the linked ELF (batch E §5).
  Nobody has yet demonstrated the pairing that breaks without them.
* **4 — closed, with a correction.** The extraction described above was not merely
  untested: its signed difference `int32_t(120000 - now)` wraps positive once
  `millis()` passes ~2^31, which pins the calibration refusal on forever. Replaced
  with a latching `util::WarmupGate` and pinned by `test/test_warmup`, including a
  scan that bans that arithmetic in `src/main.cpp` (batch E §4).
* **6 — closed.** `DiveStore::exportRaw` deleted, suite green (`e0c3b25`).
* **1 — closed by batch D** (measured expiry 3061/3011 ms), **5 — still open**,
  **7 — as written**.

### Round B — event-row honesty, one instant per batch, and narrowing that cannot lie

Scope: the 2026-11-27 review's **N1 freshness rule** (stated but broken by the
order of two statements), the row-coherence finding beside it, and the int16/uint16
narrowing that a review call described as "a value out of range reads as *no
obligation*". Bench state: base `038df54` with local commits `144928d` (Batch A),
`0eabfdd` (its write-up) and `adeb689` (the handset design sheet) beneath it;
native suite **406/406** (was 398 after Batch A), `pio run -e esp32` and
`pio run -e esp32_stalltest` both SUCCESS, no warnings from the touched sources.
**No hardware was flashed or read in this batch** — code, a host probe and native
tests. `graphify-out/` left untracked.

---

#### 1. The event ring is staged, so an edge cannot be read against a stale plan

**Symptom.** `deco/view.h` publishes a deco generation and states N1's freshness
rule: the picture an event-driven dive row carries is the one ControlTask computed
*for that edge*. `publishEvents()` pushed edges into the ring and `main.cpp`
recomputed the plan afterwards. Between those two statements the ring was already
readable by the handset task on the other core, so a drain landing in that window
filed "the machine decided X" beside the plan of the last 30 s grid. The rule was
documented and the code next to it contradicted it — nothing observable, because
the window is one `schedule()` call wide and the consumer is on another core.

**Fix.** `src/comms/telemetry_bridge.h:159-207` — `EventRing` gains `beginTx()` /
`commitTx()`. `push()` advances a producer-private cursor `write_` and only moves
`head_` when no batch is staged; `commitTx()` publishes with a **release store on
`head_`**, which is what orders every event payload before the head move. That is
why `head_`/`tail_` became `std::atomic<uint16_t>`: a `volatile` plain store
carries no ordering, and "the payload is visible before the head is" *is* an
ordering claim. A consumer that sees an event sees a whole `Event`, never a torn
slot.

**Call site.** `src/main.cpp:687` (`beginTx`, top of the cycle, before any push)
and `:891` (`commitTx`, after the plan republish at `:886-889`). The pair brackets
`publishEvents()` and the recompute — "the publish and the commit are one pair;
swapping them is the bug they exist to prevent". The only `continue` in the
iteration is *above* `beginTx`, so a staged batch never outlives the cycle that
staged it. `publishCalAir()` and the other out-of-cycle producers push outside a
batch, and `push()` publishes those immediately — a commit that never comes must
not hold an event hostage.

**Tests** (`test/test_uartlink/test_uart_link.cpp`): a staged batch is invisible
until committed; a push outside a batch is readable at once and staging is not
sticky; a staged batch still respects the ring's `kSize - 1` capacity (staging is
not an unbounded buffer — the honest answer to a full ring stays a counted refusal).
`test/test_controlplane/test_controlplane.cpp` pins the *order* as text:
`beginTx < publishEvents < viewPlan < commitTx`, and that `drainEvents` calls
`captureEdge` exactly once.

#### 2. One drain batch describes one instant

**Symptom.** Each event drained its own snapshot row by reading the live bridge
scalars, so eight edges drained together were eight rows read from eight control
cycles. A dive file showing the clamp engage beside an override then showed the
depth and ppO2 underneath them drifting between the rows that recorded both.
Separately, the event row's `t_s` was counted from the command task's file-open
instant — up to one control cycle later than the `E,` line beside it, so a verdict
and the row written to explain it could sit a second apart.

**Fix.** `bridge::EdgeSnap` (`telemetry_bridge.h:282-313`) +
`bridge::captureEdge(tel, totals, nowMs, out)` (`telemetry_bridge.cpp:204`): copies
the published scalars and takes one `deco::viewSnapshot` generation. It also
carries `diveStartMs` from the published telemetry, so the `S,` row and the `E,`
line from the same edge state the same `t_s`. `UartLink::drainEvents(sink, ctx,
nowMs)` (`uart_link.cpp:181-210`) captures **before the first row of the batch**
and passes the snap to `Hooks::appendSnapNow(const EdgeSnap&, void*)`;
`session_wiring.cpp:240-260` fills `RowInputs` from it. The boot-window case
(`hasPic == false`) is the only row with no picture, and it now answers the `ndl`
sentinel rather than zeros.

**Tests**: `test_one_drain_batch_reads_one_instant` moves the bridge scalars after
every row the hook is handed — which is what ControlTask does while the command
task is halfway across a batch — and asserts that stamp, depth, ppO2, ceiling and
NDL are identical across the three rows of one drain, and that `diveStartMs` is the
chain's, not the pump's. `test_a_capture_before_the_first_picture_says_there_is_no
_picture` covers the boot window end to end (`captureEdge` → `fillSnapshot` → the
sentinel).

#### 3. `ndl` is `-1` on the only rows that have no picture

`fillSnapshot`'s no-picture branch used to leave `ndl = 0`. `0` minutes means "your
no-deco time is finished, you are obligated now" — a reading, and the most alarming
one available on a row whose picture does not exist. It now writes
`deco::kNDL_UNKNOWN` (-1), the marker `D,` has always used for "no number", so one
client rule serves the wire and the file. `ceiling_m`/`ceiling_cm`/`deco_min` stay
`0`: the row format defines no absence for them, and a negative ceiling would read
as a *shallower* obligation, not as no picture. Pinned in
`test/test_logfmt/test_divelog.cpp` and documented in `docs/HANDSET_PROTOCOL.md` §5.

#### 4. Narrowing saturates; it never wraps

**Reproduced on host** (`/tmp/opencode/probe/probe.cpp`): every compartment seeded
to `kSeedMaxBar = 100.0` (the widest load `setTissueState` accepts), CNS clock 999 %,
60 m absolute — `ceilingBar = 96.843`, `schedule()` 16 stops, `capped = 1`, and
`viewFull` wrote `ceilingCm = 29800`, `stop0Depth = 29864`, `cnsX10 = 9990`,
`decoTotalX10 = 23980`. `29800` is `95336 - 65536`: the engine's 953 m ceiling,
printed as a plausible shallow ceiling, which this document and the web page both
read as "no decompression obligation". No dive reaches it — the sensor bottoms near
25 bar absolute — so the only route is a well-formed and wrong continuity record,
i.e. a future unit or format mistake. The guard is the net for that route, and the
**direction** is the argument: saturation keeps the diver *more* obligated than the
truth and is recognisable as the column's own limit, where a wrap hands back a
number from a different dive.

**Helpers** (`src/util/numfmt.h:77-117`): `narrow<T>(long long)` and
`scaledInto<T>(double, decimals)`. Hand-rolled ternary clamps were replaced by
them, and the rule is enforced on the text: `test/test_numfmt` scans the five
storing files for `static_cast<…>(numfmt::scaled` and fails with "wraps; go through
numfmt::scaledInto".

**Sites**: `deco/view.h` (ceiling, CNS, deco total, stop depth and minutes),
`log/rowfill.cpp` (depth, voted, cells, setpoint, battery, window pulses),
`comms/telemetry_bridge.cpp` (`EVT_SP_CHANGE` payload, `toMeta` min/max ppO2,
`totalPulses`), `main.cpp` (totals depth/voted, GF pair, sp1..sp4, surface spread),
`control/cellhist.cpp` (the `HIST,` bin width). `countsFromFloat` and the
already-clamped `cnsPeakPct`/`creditS` sites were left as they are.

**Pulses deliberately changed.** `S,`'s window `pulses` used to truncate — pinned by
a test that called truncation "the behaviour the firmware has always had". But
`M,`'s `totalPulses` now saturates, and §5 sells `snapshots`/`events` and the pulse
columns to clients as a **truncation check**; two columns of one count in two
different arithmetics would fail a check the spec publishes. The pinned test was
rewritten to pin saturation and say why, and the unsigned subtraction is now
signed before narrowing, so a counter that moved backwards reports `0` instead of a
huge count. Widest legal window delta is ~50 injections (25% duty of a 10 s window
at a 50 ms pulse), so this is unreachable in a real dive. Documented in §5 and §8.

**§8/§6 documentation added:** the `ndl` rule for pictureless rows, the saturation
rule and its direction, the `pulses` column being a window (event rows have none),
and the seven cross-key ordering clamps with what each one does to the diver's
numbers — including that `GET <key>` answers the typed value while the chain runs
the clamped one, because nothing on the wire carries a clamp verdict.

---

#### Still open then — superseded by §1; kept as the 2026-11-27 snapshot

1. **The 3 s TWDT trip has never been measured**, and
   `CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU0` watches an idle task nobody feeds.
   A detector whose trip time is unknown is a claim, not a guarantee.
2. **The clamp verdict is console-only.** `GET GF_LOW` answers what was typed while
   the chain runs the clamped pair. Fixing it properly means a protocol field,
   which is a decision rather than a fix.
3. **Flag labels are duplicated three times** — `web_page.h`, `data/index.html`,
   §4 of the spec — with nothing checking that they agree. The M-3 fix edited all
   three by hand.
4. **`pulses` on event rows is always 0** (`Telemetry::pulsesWindow` is read and
   written nowhere). Making it real moves a byte on an existing column: a protocol
   decision, documented as one in §8 rather than taken as a side effect.
5. **`LOG EXPORT` dive-lock policy** remains undecided (Batch A left it open).
6. **`src/log/continuity.cpp:135,144` still rounds with `+ 0.5`** while the repo's
   stored-number rule is `numfmt::scaled` (nearest, ties to even, decided on the
   exact product). Both sites are range-gated so nothing wraps; the residue is that
   a stored CNS or tissue count can sit one count away from the digits
   `numfmt::fixed` printed for the same float, on an exact tie. It changes bytes in
   a stored record, so it wants its own commit.

**Done after this write-up, in `2c4f6fe`:** the stale air-calibration prose —
`BENCH.md`, the console line, the two enum comments, the web page's verdict
caption and the bench harness's hint still said "spread band" /
"spread-limited", the rule `66694f9` replaced. The wire was untouched (code 5 is
still 5); `BENCH.md`'s dated record keeps the old words with "as it was then",
because that entry is a measurement made under the old rule.

**Done after this write-up, in `e662b69`:** `graphify-out/` is ignored — regenerated
tool output, nothing in the repo reads it.

**Done after this write-up, on the bench:** this batch's code was flashed to the
controller with an explicit `--upload-port /dev/ttyUSB0` (two identical CP2102
boards: controller `ttyUSB0`, handset bridge `ttyUSB1` — `tools/port_id.py`) and
`tools/hw_accept.py --expect-web --handset /dev/ttyUSB1 --cal-air` ran green — exit
0, 126 PASS, `stk=` worst 512, `u2stk=` worst 3088, worst control cycle 443 ms
against the 1500 ms stall budget. `logs/accept_20261127_075435_*.log`. (`stk=` at
exactly the 512 threshold is what the next paragraph's stack finding turned on.)

**Done after this write-up, in `896fca8`:** the `g_ctrlCfg` double-buffering item
above. It became a published generation rather than a double buffer — a reader
needs one whole config per use, not one of two alternating ones — and the applier's
carried fields came with it. See §Round C in this file.

**Done after this write-up, in `2c4f6fe`:** the stale air-calibration prose —
`BENCH.md`, the console line, the two enum comments, the web page's verdict
caption and the bench harness's hint still said "spread band" /
"spread-limited", the rule `66694f9` replaced. The wire was untouched (code 5 is
still 5); `BENCH.md`'s dated record keeps the old words with "as it was then",
because that entry is a measurement made under the old rule.

### Round C — the applied rules as one generation, not a global four tasks share

Scope: the finding Batch B left open as its item 2 — *"`g_ctrlCfg` double-buffering:
`postConfig()` copies a struct while ControlTask reads it; the ordering clamps and
the settings apply path both touch that one object"*. Bench state: base `e662b69`
(Batch B, pushed) with `896fca8` (this batch) on top. Native suite **412/412** (was
406), `pio run -e esp32` and `pio run -e esp32_stalltest` SUCCESS with no warnings
from the touched sources, and — unlike Batch B — **the bench ran this image**:
`tools/hw_accept.py --expect-web --handset /dev/ttyUSB1 --cal-air`, exit 0, 126 PASS.

---

#### 1. Two things were wrong with the global, and only one of them is obvious

`ctrl::Config` — setpoint ladder and its depth bands, gradient factors, alarm edges,
the injection walls, the max-on fuse, the proof window, the pre-dive patience —
lived in one global, `g_ctrlCfg`. The settings path wrote it field by field on the
command task while three other tasks read it: SensorTask (`deco.waterDensity`, which
converts the pressure reading into the depth every screen and every dive row shows),
SupervisorTask (`inj.maxOnMs`, the fuse that forces the solenoid LOW when ControlTask
has stopped cutting it) and ControlTask (present mask, gradient pair, setpoint ladder,
pre-dive grace).

**Coherence.** On a 32-bit core no single word is torn, so this is not the race
people reach for. What was torn was the *pair*, and the pairs are precisely what
`settings_apply.cpp` exists to make impossible: a new `ALLO_HIGH` beside the old
`ALLO_CUTOFF`, a lowered `GF_LOW` beside the old `GF_HIGH`, a sorted setpoint ladder
beside the old depth edges. The applier clamps those combinations over a complete
set; the global handed a reader a state the applier guarantees cannot exist. A dive
row written in that window reports a gradient pair that no accepted `SET` ever
produced, and the file is the artifact a diver or an investigator reads later.

**The applier's own dependency.** `applySettings()` compares and clamps against
fields it does not write — `inj.highWallBar`, `inj.maxOnMs`, `inj.windowMs`,
`pulseTest.pulses` (`src/comms/settings_apply.cpp:74-93`, the alarm-edge,
pulse-under-fuse and duty-wall rules). Those four exist only because
`buildControlConfig()` set them at boot. Applying *onto* the object it reads made
that dependency invisible: nothing
said which generation the carried fields came from, nothing stopped a second apply
pass from deciding against a config another pass was mid-update on, and a reader
could watch the applier's intermediate state. Two `hookSettings` calls cannot
overlap today (the session lock keeps commands off each other), so this half is not
a live race — it is the reason the next writer would silently inherit it.

Both halves are fixed by the same change: apply onto a private copy, publish the
finished result as one generation, and have readers take a whole generation per use.

#### 2. The box — `src/control/configbox.h`, `.cpp`

```
void     configPublish(const Config&);   // the settings path and setup() only
bool     configRead(Config& out);        // one whole generation, or false
uint32_t configGeneration();             // 0 = nothing published yet
```

**A mutex, not the seqlock `deco/view.h` uses.** The payload is a struct copy: 284
bytes measured on host (`sizeof(ctrl::Config)`), in and out, and the writer's slow
work — the registry walk and the ordering clamps — happens on the caller's own copy
*before* it takes the lock. A seqlock reads the payload while it may be being
written and decides afterwards; that data race is free to accept for a deco picture,
where a failed retry means a slightly older picture, and is not free to accept for
the number a solenoid fuse judges an output by. Blocking a control cycle for a
284-byte copy is nothing beside the `schedule()` call a few lines later; the tick
budget is 1500 ms.

**A read before the first publish returns false and leaves the caller's struct
exactly as it was.** That rule is the whole point of the callers' fallbacks: a
value-initialised `ctrl::Config` is `presentMask = 0` and `maxDutyPct = 0`, which
reads as "no cells fitted, no oxygen ever" far more convincingly than "nothing
published yet". SensorTask seeds `WATER_DENSITY_KGL` (a density of 0.0 divides the
depth conversion to infinity, and an infinite depth reads as a sensor reading rather
than as a bug); ControlTask and SupervisorTask seed the shipping walls; `hookSettings`
refuses and says `settings: NOT APPLIED — no configuration published yet` on the
console rather than apply onto zeros. Every one of those branches is unreachable by
construction, and each comment says why the alternative is worse than unreachable.

#### 3. Ordering, and which way each bound cuts

* **`setup()` publishes before it creates any task** (`src/main.cpp:1433`, 44 lines
  before the first `xTaskCreatePinnedToCore`), and the chain is built from the same
  struct (`src/main.cpp:1447`). That is what makes the fallbacks above genuinely
  unreachable, and it is pinned as text rather than asserted in prose.
* **`hookSettings` publishes BEFORE `postConfig`.** ControlTask reads the generation
  at the top of its cycle and the chain takes the queued one in `cycle()`'s step 0,
  so a tightened limiter reaches the pin fuse no later than it reaches the engine.
  The worst case is the 100 ms command latency `loop.h` already states — this change
  adds no new bound — but the direction depends on the SET, so the comment names
  both: a max-on **tightened** can be judged once by the entry fuse against the
  looser old value (that fuse is the backstop for a ControlTask that is not reaching
  `cycle()` at all; the fuse *inside* `cycle()` runs on the chain's own copy and cuts
  under the new number the same tick), and a max-on **loosened** is judged against
  the older, stricter value, which cannot miss anything. Either way the entry fuse is
  policing the HIGH interval this iteration inherited from the previous one — which
  is the interval it exists to police.
* **Every task re-reads inside its loop**, so a `SET` reaches the sensor's ruler, the
  fuse and the dive row on the next tick. A single read outside the loop would pass a
  "does it ever read" scan and leave the task running on whatever boot handed it for
  the rest of the dive; the test therefore looks for the read *after* `for (;;)`.

#### 4. Where the copy lives — and the bench result that decided it

The readers' snapshots are **task-private statics** (`g_sensorRules`,
`g_controlRules`, `g_supervisorRules`), not locals. The reason is a measurement, not
taste: with a generation on the stack, the first acceptance run on this code failed
its own headroom check —

```
[FAIL] stk= headroom >= 512 (worst 320)     # 896fca8's first run, locals on the stack
[PASS] stk= headroom >= 512 (worst 608)     # the same image after moving them
```

The supervisor runs on a 3072-byte stack whose measured headroom was *exactly* the
bench's 512-byte warning threshold before this batch, so a 284-byte copy on it
pushed the minimum below the gate. A static written and read by exactly one task is a
per-task snapshot; what made the old global dangerous was that someone else wrote it
mid-flight. The cost is DRAM, and it is measurable on the build report: 54372 B at
16.6 % on the Batch B image, 56284 B at 17.2 % here (+1912 B — the published copy
is 284 of it, the three snapshots 852). Note the direction: moving the copies off
the stack made the RAM number *worse* and the stack number *better*, which is why
the RAM line alone would never have caught this and `stk=` did.

#### 5. Tests — `test/test_configbox/test_configbox.cpp` (6 cases)

* Nothing published: `configRead` returns false and leaves the caller's seeded values
  untouched (both fields asserted — this is the "zeros look plausible" rule).
* A publish **replaces** the whole generation rather than merging field by field; the
  partial-update case is asserted on a specific field (`inj.maxOnMs`), because that is
  the old global's behaviour.
* Writer thread plus reader thread, 4000 generations, every generation a
  deterministic function of one seed: a read that blended two publishes cannot decode
  to a single seed, and the seed sequence is checked monotone. This is the evidence the
  mutex does its job, in the shape of the real thing (settings task writing, control
  task reading once per iteration).
* `hookSettings` order as text: `configRead < applySettings < configPublish <
  postConfig`, and `postConfig(cfg)` — the queued struct must be the one that was
  published, not a re-read that could pick up someone else's generation.
* The old global is banned in the four sources that used it: `g_ctrlCfg.` and
  `extern ctrl::Config g_ctrlCfg` banned raw (comments included), the bare name banned
  in comment-stripped source. Comments are exempt from the broad ban because prose has
  to be able to say what the old thing was.
* Boot ordering as text: `buildControlConfig < applySettings < configPublish <
  new Loop < xTaskCreatePinnedToCore`.

**Mutation-checked**, each turning exactly one case red: publishing after
`postConfig`; publishing after the task creation; deleting SensorTask's in-loop read.
The third one is the interesting result — the first version of the scan only asked
whether the task ever calls `configRead`, and the pre-loop seed line satisfied it, so
the mutation stayed green. That is why the scan anchors on `for (;;)`.

#### 6. Unchanged, and what stays open — superseded by §1

**Nothing moved on the wire.** No row format, no column, no event id, no `SET`/`GET`
behaviour. The clamp verdict is still console-only, so `GET GF_LOW` still answers the
number the diver typed while the chain runs the clamped one — a protocol decision,
recorded in `docs/HANDSET_PROTOCOL.md` §6 and unchanged here.

1. **The published payload is the whole `ctrl::Config` (284 B), not the ~40 bytes the
   readers actually use.** Deliberate: a narrow `Applied` struct would duplicate the
   field list and any field a future reader wants would need a new publish path — the
   kind of gap that gets filled by reaching back for the global. The cost is DRAM, not
   stack.
2. **Nothing reports the skew between the chain's copy and the published generation.**
   They come from the same struct, and the bound is one tick, but `HB` carries no
   `cfggen=` field to make that visible on the bench. Adding one is a protocol change
   on the heartbeat line, which is append-only and worth doing only if someone is
   going to read it.
3. **`configPublish` has two callers by design** (`setup()` and `hookSettings`), and no
   test rejects a third. `configbox.h` states that the settings path is the only
   mid-flight writer; if a factory-reset path ever applies defaults mid-dive, it must
   go through the same read-apply-publish shape or it reintroduces the impossible pair.
4. Still open from Batch B and untouched by this one: the 3 s TWDT trip has never been
   measured, `CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU0` watches an idle task nobody
   feeds, flag labels live in three places with nothing checking they agree, event
   rows' `pulses` column is 0 by protocol decision, `LOG EXPORT`'s dive-lock policy is
   undecided, and `src/log/continuity.cpp:135,144` still rounds with `+ 0.5` (changes
   stored bytes, wants its own commit).

**Closed by Round D the same day:** items 2 and 3
(`HB` now carries `cfggen=<published>/<chain>`, and `test_configbox` counts publish
sites and refuses a third), and item 4's first three clauses — the TWDT trip is
measured on the board at **3011–3061 ms** (and a neighbour's feed does not mask a
stalled task), the idle-task claim turned out to be false in the other direction (no
idle handle is ever subscribed here), the flag labels are graded across all four
places that name them, and the continuity rounding moved to `numfmt::scaled`. Items
1, the `pulses` column and `LOG EXPORT`'s dive-lock policy are unchanged.

**Closed by Round E later the same day:** the
`pulses` column (event-driven `S,` rows now subtract the same base the grid rows do,
and §9 states that the two row kinds' windows OVERLAP — `M,` is the total), and
`LOG EXPORT`'s dive-lock policy (refused mid-dive with `ERR export locked in dive`;
`LOG LIST` stays open; the web's `/dive/N.csv` stays unlocked, and §2/§5 say why).
**Item 1 stays open and is measured:** `sizeof(ctrl::Config)` = **284 B** confirmed on
the batch E tree, `env:esp32` RAM 56292 B / 17.2 %, and the decision not to narrow it
is recorded with the numbers in batch E §7 rather than left implied. The clamp
verdict this section called console-only is protocol now: `E,<t>,19,<mask>`.

### Round D — the last five review findings, and one measured premise that was wrong

Batches Round A, Round B and
Round C closed the safety findings. This one closes the
remaining five items from that review's "code findings, no new rigging" list.
Every number below came off the controller or out of the repo's own logs, and
where a review claim did not survive checking, that is said with the evidence.

Bench state for the whole batch: controller on `/dev/ttyUSB0` (identified with
`tools/port_id.py` — two identical CP2102s are attached, the handset bridge is
`ttyUSB1`), pack unplugged (`vbat` ≈ 1.56 V, `batlow=1`), cells uncalibrated at
first boot, LPS28DFW depth chip reading.

---

#### 1. The 3 s task watchdog — measured at last, and the review's mechanism was wrong

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

#### 2. Flag labels — one fault, one name, in all four places that name it

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

#### 3. Dive-log rounding — the stored count and the printed digits share one rule

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

#### 4. Config generation — a third publisher is refused, and the skew is a number on HB

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

#### 5. The design's mock wire lines are graded as wire lines

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

#### Verification, Round D

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

#### Still open then — superseded by §1; kept as the 2026-11-27 snapshot

- Flash contention **during a dive**: exports read while the dive log writes, with
  the depth sensor squeezed, is the unmeasured case behind the review's slow-cycle
  worry. `worst=` peaked at 8 ms at the surface, so the margin question is now
  about that configuration specifically. **Partly closed by
  Round E §3:** `LOG EXPORT` is refused mid-dive
  (`ERR export locked in dive`), so the handset path cannot contend at all; the
  web's `/dive/N.csv` deliberately still can, and remains the unmeasured case.
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

### Round E — Round D's "still open" list, and one fix that was itself a bug

Batches Round A, Round B,
Round C and Round D left seven
named loose ends. All seven are dealt with here: six are closed with a witness, the
seventh (the config box's 284-byte payload) is measured and deliberately left alone,
with the reason written down. Two of the seven turned out to be bigger than the
review thought: item 4's earlier fix was wrong in a second way, and item 5's claim
cannot be witnessed on this host at all — both are reported with the measurement
rather than smoothed over.

Bench state for the whole batch: controller on `/dev/ttyUSB0`, handset bridge on
`/dev/ttyUSB1` (ports named with `tools/port_id.py` — two identical CP2102s), pack
**unplugged** (`vbat` 1.56 V, `batlow=1`), cells calibrated, LPS28DFW depth chip.
Nothing was flashed without `--upload-port /dev/ttyUSB0`, and no pio build ran while
a bench reader held a port.

| commit | item |
|---|---|
| `e0c3b25` | 6 — dead `DiveStore::exportRaw` removed |
| `2521df3` | 4 — the warm-up gate latches; the clock arithmetic it replaced was wrong twice |
| `12fdba5` | 3 — `LOG EXPORT` is dive-locked |
| `acedcf3` | 1 — the settings clamp verdict is protocol: `E,<t>,19,<mask>` |
| `6caf839` | 2 — event-driven dive rows report real injections |
| `9e83d82` | 5 — `env:native_o2`, and the measured answer about the seqlock fences |
| `2ba5757` | 1 — the clamp verdict's handset-side test and acceptance-matrix leg |

---

#### 1. The clamp verdict became a protocol event, not a console line

**What was open** (batch A's open list, item 2, restated by batch C §6): the
settings applier enforced an ordering and the wire said nothing. `SET GF_LOW=90`
answered `OK SET GF_LOW`, `GET GF_LOW` went on answering **90**, and the chain ran
**85**. The only trace of the clamp was `settings: CLAMPED GF_ORDER …` on the
console, which nobody keeps.

**What landed.** A new `E,` id, **19 `EVT_SETTINGS_CLAMPED`**, whose `value` is the
applier's `comms::ApplyClamp` bitmask — 1 ALLO_HIGH, 2 CUTOFF, 4 PULSE_MS, 8 DUTY,
16 SP_ORDER, 32 GF_ORDER, 64 BAND_ORDER — documented in §4's table and §6. Ids 11
and 12 stay what they always were (the cell-proof setpoint clamp); nothing was
renumbered, the new id was appended, `T,`/`HB` field order untouched, FLAGS bits
untouched. That is the append-only rule doing its job: a 2026-11-24 handset parser
(`handset_link.h parseEvent`) is id-agnostic and needed no change at all.

The publishing path is the part worth recording. The event ring is **SPSC**: only
ControlTask may push. The clamp happens on the command task, inside `hookSettings`.
So the command task never touches the ring — it latches
`std::atomic<uint16_t> g_clampPending` with `fetch_or`, and ControlTask drains it
with `exchange(0)` inside its existing staged `beginTx`/`commitTx` block and calls
`bridge::publishClampVerdict`. Boot-time clamping in `setup()` feeds the same latch,
so a settings blob that arrives clamped is reported the same way. A `SET` that
clamped nothing publishes nothing — same rule the CAL AIR progress rows follow, and
for the same reason: a row per SET would bury the dive in rows for a fact that did
not happen.

**Witnessed both directions.**

* `test/test_uartlink/test_uart_link.cpp` (`2ba5757`): mask 0 publishes no row; mask
  0x20 arrives as id 19 with value 0x20 at the right `t_s`; a multi-bit mask (0x21)
  arrives intact (dropping a bit would name one enforced ordering and hide another,
  and the diver would fix half the pair); the row reaches the handset downlink as
  `E,7,19,32` and drags a snapshot row into the dive file beside it.
* `hw_accept.py`: the surface matrix sends `SET GF_LOW=90` (the GF keys are
  dive-locked, and a refused SET clamps nothing, so this has to be at the surface),
  expects `OK`, then `GET GF_LOW` → **90** — the gap itself, proved on the wire,
  because the chain is running 85 — and `grade_stream` then demands **exactly one**
  `E,<t>,19,<mask>` naming GF_ORDER alone. Zero rows means the verdict is
  console-only again; two means something publishes for SETs that needed no clamp; a
  wrong mask names an ordering that was never enforced. Five lying captures in
  `--selftest` (missing row, doubled row, SP_ORDER instead of GF_ORDER, GF_ORDER
  plus an invented bit, honest stream): **131 cases, 0 failures**.
* Hardware, the same day: `logs/accept_20261127_151848_*.log` —
  `~ [26.55s] settings: CLAMPED GF_ORDER — ordering enforced over what was set` on
  the console and
  `[PASS] the clamp verdict arrived unprompted as E,<t>,19,<mask>` /
  `[PASS] the clamp row names GF_ORDER and nothing else` on the handset link.
* `web_page.h eventName()` decodes the mask for the surface page, and
  `data/index.html` was regenerated with `tools/sync_webdata.py`, so the two label
  lists batch A's open item 5 warns about stayed in step.

#### 2. `pulses` in an event-driven dive row stopped being a hard-wired zero

**What was open.** Every event-driven `S,` row has recorded `0` in its `pulses`
column since that column existed. Not "no injections at this moment" — the value
was never computed. `Telemetry::pulsesWindow` was declared, read by the capture, and
written by nobody, while the 30 s grid rows beside it carried real windows. §9 has
always told a reader not to sum the column across row kinds; the zeros did not save
a reader either, because they looked like measurements.

**What landed.** One pulse base, owned by the task that counts:

* ControlTask publishes `pulsesWindow = pulseCount() - lastPulseBase` **every**
  cycle, next to the total it already published (the field widened to `uint32_t`).
  `lastPulseBase` still advances in exactly one place — the 30 s grid block — and the
  publisher statics moved above the scalar block so it can read them without moving
  the base;
* `captureEdge` copies both scalars into `EdgeSnap::pulsesNow/Prev`, one instant,
  like every other number in the capture, so eight edges drained together still
  describe one control cycle and one injection window;
* `hookAppendSnap` passes them to rowfill. Grid and event rows now subtract the same
  base, which means their windows **OVERLAP by design** — an event row reports the
  span the following grid row also covers. §9 was rewritten to say that out loud with
  the one rule that keeps a reader honest: the dive's total is `M,`'s pulses, never a
  column sum. A single window larger than the total remains the case that cannot
  happen if both kinds read one base.

`row_format` stays 3 and `DIVE_VERSION` stays 3: the column set and the on-flash
record layout did not move. One column's value stopped being a placeholder, and §9
records the date that happened — the only thing a reader of an old dive file needs.

**Witnessed, and then a hole found in the witness.**

* `test/test_uartlink`'s drain-batch coherence test pins the capture's pulse pair: a
  row of one batch that captured a later count fails, and a moved window base fails.
  The values are pinned too — total 120, base 113, read back off the published
  scalars.
* `hw_accept.py`'s dive-file grader now parses every window, rejects a wall of zeros
  beside a non-zero total, and rejects any window over `M,`'s total. Two new lying
  captures in `--selftest` — the honest dive with every window zeroed, and an event
  row claiming 40 of 16 — each fail; the honest corpus stays green.
* **The live half was vacuous, and saying so is the finding.** In
  `logs/accept_20261127_151848_*`, the dive the acceptance run drove never injected:
  `M,` total **0**, every `S,` window **0**, `HB` `pulses=0` across the run. So those
  three checks passed without witnessing anything — a wall of zeros cannot tell a
  placeholder from a measurement. The reason is a gate worth remembering: the
  injector only ever wants a pulse in **AUTO** (`control/injector.cpp`), AUTO is
  locked out until PREDIVE PASSes, and PREDIVE's BATTERY step refuses below
  `vbatMin` = 10.5 V (`control/predive.h`). With the pack unplugged
  (`vbat=1.56`, `batlow=1`) **no injecting dive is reachable at all** — verified on
  the wire: `AUTO` → `ERR predive required`. So the grader now prints an explicit
  note when `M,` total is 0 instead of letting a PASS imply a witness, and
  `tools/pulse_witness.py` drives the injecting dive properly — reads `vbat` off
  `T,`/`HB`, refuses with exit 2 and the pack explanation (run and confirmed:
  `power: vbat=1.56 V … an injecting dive is unreachable here`), otherwise grades the
  export with `hw_accept`'s own `grade_dive_file` rather than a second copy of the
  arithmetic. **The non-vacuous witness is still owed**: it needs the pack connected,
  and that is listed under Still open.

#### 3. `LOG EXPORT` is dive-locked

**What was open.** `LOG EXPORT` read a flash file while the live dive file was being
written to the same flash, mid-dive — the slow-cycle worry batch D's acceptance
table had measured at the surface only (`worst=` 4–9 ms at the surface under a web
storm, exports during a dive explicitly listed as not covered).

**What landed.** `cmd_router.cpp` refuses it while a dive is open:
`ERR export locked in dive`. `LOG LIST` deliberately stays open — it is a bounded
scan of the index, it does not read a dive's contents, and a diver needs to see what
is stored. The web's `/dive/N.csv` is deliberately **not** locked: `async_tcp` is
unregistered from the watchdog by design, the export path there already returns
`503 "no memory for export"` under load, and locking it would take the surface's one
way to fetch a dive away from the diver's own client. That divergence — handset
refuses, web does not — is stated in §2 and §5 of `docs/HANDSET_PROTOCOL.md` rather
than left for a reader to discover.

**Witnessed both directions.** `test/test_router`: a mid-dive `LOG EXPORT` expects
the refusal line; removing the lock line from `cmd_router.cpp` gives exactly 1
failure, restoring it gives green. On hardware, mid-dive in the acceptance matrix:

```
[PASS] LOG EXPORT 1: want ^ERR export locked in dive, got ERR export locked in dive
       — no export mid-dive: the flash belongs to the live dive file
```

#### 4. The warm-up gate: batch A's fix was wrong, and wrong twice

**What was open** (batch A's open list, item 4): "CAL AIR warm-up arithmetic lives
in `main.cpp` with no native coverage. An extraction into `src/util/warmup.h` was
implemented but did not survive concurrent edits."

**What was actually there was worse than untested.** Batch A's extraction computed
the remaining time as a **signed difference** — roughly `int32_t(120000 - now)`. That
is correct for 24.9 days and then wraps: past `millis()` ≈ 2³¹, the expression turns
**positive** again, so `warmupLeft` looked nonzero, the gate refused a calibration
forever, and `warmup_left_s` printed a number that had no relation to the 120 s
window. On a dive controller that boots, dives, and is left powered for weeks, that
is a real date, not a corner case. The bug had never been seen because the bench
reboots every flash.

**What landed.** No clock arithmetic in the refusal path at all: `util::WarmupGate`
latches. `update()` is called once per ControlTask cycle with `now`; the gate opens
when it has *seen* `now` pass its own start, and the refusal path only asks
`elapsed()` / `remainingSeconds()`. A counter that only ever compares a monotonic
clock against a latched start cannot wrap into a lie.

**Witnessed, including the two wrong forms.** `test/test_warmup` pins the old
arithmetic's failure shape (the plain subtraction and an off-by-one variant both fail
as tests, so a future "cleanup" back to subtraction is caught), pins the latched
gate's open/closed edges, and anchors a **scan** that bans comparing a warm-up window
against `millis()` inside `src/main.cpp` — the same class of anchor batch D's item 2
lesson is about: a scan must assert a minimum count of what it actually graded, or it
passes by matching nothing.

#### 5. The seqlock fences: an `-O2` gate now exists, and it says the claim is unprovable here

**What was open** (batch A's m-4 residual, its open list item 3): the fences around
the `deco/view.h` seqlock were called load-bearing, and "x86 `-O0` never reorders",
so no gate could see them.

**What landed.** `env:native_o2` in `platformio.ini`: `env:native` with `-O2` added,
flags interpolated so the only variable is the optimization level, all 34 suites.
The comment in the file carries the measurement. Measured on this tree, today, on
x86-64 with the host GCC:

| run | result |
|---|---|
| baseline, both levels | **437/437 at `-O0`, 437/437 at `-O2`** (874 together, same tree, same day) |
| both fences commented out in `src/deco/view.h` | **874/874 still green** — `-O0` and `-O2` both |
| fences restored | `git diff src/deco/view.h` empty; 437/437 at `-O2` again |

So the gate has **no teeth against fence removal on x86-64**, which is m-4's own
reading confirmed rather than dodged: `fetch_add(acq_rel)` lowers to `lock xadd` — a
full hardware barrier — and this compiler does not sink plain loads and stores across
an atomic RMW. No host run can demonstrate those fences are load-bearing.

What the target actually does with them **is** visible, and it is the other half of
the answer. `tools/fence_probe.cpp` is a checked-in probe: the same seqlock pattern
as shipped, the same pattern with each fence removed, and the sequence reads relaxed.
`tools/fence_probe_report.py` builds it with the toolchain `env:esp32` links with
(**xtensa-esp-elf-g++ 14.2.0**, crosstool-NG esp-14.2.0_20260121), disassembles it,
and counts `memw` — Xtensa's memory barrier — into
`logs/m4_fence_probe_20261127_disasm.txt`:

| function | `memw` count |
|---|---|
| probe: publish with the release fence | 3 |
| probe: same, release fence removed | 2 |
| probe: read with the acquire fence | 4 |
| probe: same, acquire fence removed | 3 |
| `publishView` as compiled into the shipping `firmware.elf` | 3 |
| `viewSnapshot` as compiled into the shipping `firmware.elf` | 5 |

Each explicit fence costs one real Xtensa `memw` on the target — the target compiler
does **not** fold them away the way x86 does. That is the honest boundary: the fences
are observable instructions on the weakly-ordered core and invisible to any test on
this host. What remains unprovable anywhere in this repo's gates is the *necessity*
claim — that removing them would let a reader pair a payload with the wrong sequence
number on ESP32. That stays an open residual, and it is written in `platformio.ini`,
in this document, and in batch A's open list.

What `env:native_o2` does prove, and why it stays in the gate: the whole host suite
survives the optimizer. No undefined behaviour hiding under `-O0`, and no
timing-graded case (the `viewScalar` budget, the seqlock reader-liveness loop) breaks
when its loop is optimized.

#### 6. `DiveStore::exportRaw` deleted

Dead: nothing called it, and a second export path in a store whose whole contract is
"one reader at a time" is the kind of surface that gets used by accident three
commits later. `e0c3b25`. The suite went green without it — the honest test that a
function is dead is deleting it and running everything.

#### 7. The config box publishes all 284 bytes — measured, and left alone

Batch C's open list item 1. Measured on this tree:

```
sizeof(ctrl::Config) = 284          # compiled probe against src/control/configbox.h
RAM 56292 B / 327680 = 17.2 %       # pio run -e esp32, this tree
Flash 1037513 B / 1310720 = 79.2 %
```

Batch C's numbers for the same change were 54372 B (16.6 %) before and 56284 B
(17.2 %) after, with the published copy 284 B of the difference and the three
per-task snapshots 852 B. Nothing here changes that arithmetic, and the reasons
batch C gives still hold: a narrow `Applied` struct duplicates the field list, and a
field a future reader wants then needs a new publish path — the gap that gets filled
by reaching back for the global, which is the bug the box exists to prevent. The cost
is DRAM, and DRAM has 271 KB free; the constraint that matters on this build is the
supervisor's **stack**, and that is why the copies live where they do (batch C §4,
`stk=` measured at exactly the 512 B gate floor).

Deliberately deferred, per the review directive to do this last: the honest way to
shrink it is a reader-side subset with a test that keeps the subset in step with the
consumers, which is a new anchor to maintain for 240 bytes of DRAM that is not
causing anything. Recorded here so the number is tracked and the decision to not
spend on it is on the record.

**Scheduled for the next session** (the user's call: record it now, do it next
session). The facts that session needs, measured on this tree rather than guessed:

* What the three readers actually touch — every field reference to the per-task
  snapshots in `src/main.cpp`: SensorTask `deco.waterDensity` (1 field);
  SupervisorTask `inj.maxOnMs` (1); ControlTask `presentMask`, `inj.maxOnMs`,
  `predive.depthGraceMs`, `deco.gfLow`, `deco.gfHigh`, `sp.sp[4]` (6, of which the
  GF pair is only read to publish `M,`/`T,` fields — the *applier's* pair coherence
  lives in the box, not in that reader). ControlTask also seeds
  `inj.maxDutyPct` in its unreachable-before-first-publish branch.
* What the writer needs: the whole struct. `hookSettings`
  (`src/comms/session_wiring.cpp:341`) does read-published → `applySettings` →
  `configPublish` → `postConfig`, and `applySettings` reads carried fields it never
  writes (the injection wall, max-on, proof-window pulses). A narrow *published*
  payload breaks that path; only the three reader snapshots can shrink.
* The cost either way: one published copy (284 B) + three snapshots (852 B) ≈ 1.1 KB
  of 327 KB DRAM. Narrowing the snapshots recovers roughly 850 B. `stk=` is unaffected
  — those copies are statics since batch C §4 precisely so they are not on the
  supervisor's stack.
* Three shapes, in increasing diff: per-reader typed accessors under the same mutex
  (re-lists the fields in `configbox.h`, which is exactly the duplication batch C
  avoided); one ~40 B `Applied` struct published alongside the `Config` and read by
  all three (no per-reader duplication, bigger diff: `test_configbox`'s publish-site
  and `configRead` anchors, `HB cfggen=` semantics, §8's "the applied config"
  wording); or leave it as measured and stop spending thought on it.

Whichever shape is chosen, the anchor that must come with it is a test that fails when
a reader consumes a field the published payload does not carry — the failure mode of
every narrow-payload design is a future reader quietly reaching back for `g_ctrlCfg`,
which is the global this box was built to remove.

---

#### Verification, Round E

| step | result |
|---|---|
| native suite | **437 cases, 437 passed** (34 suites, every directory in test/). At both levels: `pio test -e native -e native_o2` → **874/874** |
| `pio run -e esp32` | SUCCESS — RAM 56292 B (17.2 %), Flash 1037513 B (79.2 %) |
| flash | `pio run -e esp32 -t upload --upload-port /dev/ttyUSB0`, then `tools/port_id.py` and `fuser` checked both ports free first |
| full acceptance — `hw_accept.py --expect-web --handset /dev/ttyUSB1 --cal-air --dive-persist` | **158 PASS, 0 failures**, against the image built from this tree. `logs/accept_20261127_151848_{console,handset,framing,calair,watch}.log`. New legs: `LOG EXPORT 1` mid-dive → `ERR export locked in dive`; `SET GF_LOW=90` → `OK`, `GET GF_LOW` → `90`, and exactly one `E,<t>,19,32` naming GF_ORDER alone; dive-file pulses grader and the vacuity note. Over its HB beats: `worst=` max **5 ms**, `u2stk=` floor **4144 B**, `cfggen=` 1/1, one `surface: record DEFERRED - store busy` followed by `record 195 id 17 (delayed write)` — the bounded surface gate retrying, not dropping |
| `hw_accept.py --selftest` | **131 cases, 0 failures**, including 5 new clamp-verdict lies and 2 new dive-file pulse lies |
| mutations | router lock line removed → 1 failure, restored → green; both `view.h` fences removed → 874/874 (**the negative result**, §5), `view.h` byte-identical afterwards; fence probe rebuilt for the target: `tools/fence_probe.cpp` → `tools/fence_probe_report.py` → `logs/m4_fence_probe_20261127_disasm.txt` |
| dive-file window checks on hardware | **passed vacuously** — the run's dive injected nothing (`M,` total 0, `HB pulses=0`), which the grader now prints as a note. `tools/pulse_witness.py` refuses with exit 2 for the same reason: `AUTO` is gated by PREDIVE's BATTERY step, `vbat=1.56` |
| harness hygiene | two acceptance launches were interrupted by the harness (a SIGTERM'd run and one partial console log, `logs/accept_20261127_151346_console.log`, 28 lines). That file is an interrupted launch, not a verdict; the verdict is `151848` |
| board left in | the `env:esp32` image from this tree; cells calibrated, pack unplugged |

#### Still open then — superseded by §1; kept as the 2026-11-27 snapshot

1. **The non-vacuous pulse witness on hardware.** Item 2's window arithmetic is
   witnessed by native tests and by two lying captures in `--selftest`, but not yet
   by a dive this board actually injected — unreachable with the pack unplugged.
   Connect the pack and run `tools/pulse_witness.py /dev/ttyUSB1`: exit 0 means the
   dive file's windows are real on metal.
2. **m-4's necessity claim.** The fences are one `memw` each on xtensa (measured) and
   invisible on x86 (measured). Nobody has demonstrated a payload/sequence pairing
   that breaks without them; that would need either a weakly-ordered host run or a
   targeted stress on the ESP32 itself.
3. **`worst=` toward the watchdog**, unchanged from batch D: nothing stress-tests the
   control cycle toward the measured 3011–3061 ms expiry on purpose. Item 3 removed
   one mid-dive flash contender; it did not measure the remaining ones.
4. **`CAL O2` / `CAL DEPTH`** still `ERR not implemented` — unchanged by this batch,
   and `test_designwire` still pins that the design shows the refusal.
5. **Flag labels in two places** (batch A open item 5) — **WRONG WHEN WRITTEN, not
    open.** Batch D closed it: `test/test_flaglabels` parses all four places that name
    a FLAGS bit — `comms/flags.h`, `FLAGNAMES` in `src/comms/web_page.h`, the flashed
    copy `data/index.html`, and §4's table in this spec — and grades their agreement,
    asserting §4 publishes exactly bits 0..7 in order and that each row carries the
    keyword the page's pill is built on. Re-verified on this tree: that case reads
    `docs/HANDSET_PROTOCOL.md`, asserts the row count and the keywords, and batch D's
    mutation table shows shortening §4's bit-3 row reddens it. Batch E regenerated the
    page for id 19 and the suite stayed green. The line is kept with the correction so
    batch A's item 5 is not read as still open.
6. **The config box's 284-byte payload** — §7 above: measured, deliberately not done.
7. C3 bench items unchanged: MS5837 swap/potting, battery measurement, mechanical
   solenoid inhibit, fails-closed rig, and the handset UI itself.

---

## 4. Closed without a fix, and remedies that were measured and rejected

An item leaves §1 in two ways other than being fixed: it is closed because the thing it
worried about was deleted, or it is closed as a **decision not to change anything**, with
the measurement that justified leaving it. Both are recorded here so nobody re-opens them
from the summary alone, and so nobody "fixes" them by accident.

### 4.1 `DiveStore::exportRaw` — deleted rather than repaired (Round E item 6)

It was dead: nothing called it, and a second export path in a store whose whole contract
is "one reader at a time" is the kind of surface that gets used by accident three commits
later. The honest test that a function is dead is deleting it and running everything — the
suite went green without it. Round A's residual 6 listed it as dead code; it is gone.

### 4.2 The config box's 284-byte payload — measured, and left exactly as it is (Round E item 7)

The original worry ("a few hundred bytes duplicated per reader") was a guess, so it was
measured: `sizeof(ctrl::Config) = 284`, three readers, ~1.1 KB against 327 KB of free
DRAM and ~0 against flash. Size is not the cost. The cost is that the box has to cover the
union of every surface that can change a setting, or it becomes the silent gap Round A's
M-3 closed. Three implementation shapes and the reader field lists are written out in
Round E item 7; the decision, taken on the bench, was to leave it and pick a shape in a
later session. **Still open as §1 P8** — the measurement closed the worry, not the item.

### 4.3 `/dive/N.csv` — deliberately left unlocked (Round E item 3)

`LOG EXPORT` (the handset path) is refused mid-dive; the web's dive export is not, and
that asymmetry is a decision, not an oversight: the handset path was the one the bench
could drive during a dive, and the refusal is witnessed. The web route's exposure during a
dive is **still open as §1 P2** and needs a number before it gets a policy.

### 4.4 The seqlock fences — kept, with the necessity claim withdrawn (Round E item 5)

Two arguments that used to justify them are both dead: "they cost nothing" (each is a real
`memw` on xtensa — measured by difference: fences out removes exactly one) and "the suite
is green with them, so they must matter" (the suite is also green with them **deleted**, at
both optimisations, on this host). They stay because a cheap correct default on the target
is the right place to sit when the necessity claim cannot be witnessed here — not because
anything proved they are needed. **§1 P4.**

### 4.5 `VBAT_CAL` stays at `1.00f`, untrimmed, on purpose

There is no meter reading behind any trim, and the sense path is now known to be
untrustworthy (§1 P1). Trimming a divider ratio to make a saturated ADC look plausible
would destroy the only evidence there is about which side of the divider the 5 V is on.
The knob is untouched until BENCH.md C3-c is done with a meter.

### 4.6 `CAL O2` / `CAL DEPTH` — the refusal is the feature for now (Round D item 5)

`test_designwire` was made to assert that the design sheet shows `ERR not implemented`.
That is a decision: a graded mock wire line that shows a refusal cannot be mistaken for a
promise. Implementing the sequencer is a feature decision, open as §1 P6, and whichever way
it goes, `test_designwire` has to change with it.

### 4.7 Human-readable labels in the web Settings page — investigated, **not doing it**

Asked and answered on 2026-11-27 without touching a file: the page's field label is the
wire key itself (`lab.innerHTML = key + …`, `data/index.html:467`), so all 36 settings keys
reach the diver as `GF_LOW`, `D1`, `SP_HYST`, `CELLPROOF_MAXSP` with unitless range lines.
Renaming keys is not available anyway — they are NVS keys under a hard 15-char limit, the
wire keys on `SET`/`GET` and in the `K,` schema, and what the dive file's clamp rows and the
console use — so it would have been a display-side map: ~2 KB over a 38,525 B page against
273 KB of free flash, no RAM, no protocol change, with `tools/sync_webdata.py` plus
`web_probe.py`'s page graders as the gate.

**Decision: not doing it.** Recorded so it is not re-proposed from the observation alone.
The keys stay as they are on screen — which is defensible on its own terms: they are the
same strings a diver meets in a toast, in an `E,` clamp row, on the console and in the spec,
so the UI currently names things the way every other surface does.

---

## 5. Corrections: claims that did not survive checking

Kept deliberately, with the evidence, because a consolidated document that only records
what is true now hides how the wrong claims were caught — and the next reader is the one
who has to catch the next one.

| claim, where it was written | what checking found | where the correction lives |
|---|---|---|
| "the watchdog trips at 3 s; `-DCONFIG_ESP_TASK_WDT_INIT=0` avoids double-init" (the review's fix for M-2) | `CONFIG_ESP_TASK_WDT_INIT` is a **phantom on this toolchain** — no such symbol in the SDK the `env:esp32` build compiles against, so the flag would have done nothing. The init is explicit in code instead | Round A M-2 |
| "3 s is now real… the trip is not measured on hardware" (Round A residual 1) | still unmeasured a day later, and measured after that: **3011–3061 ms** against the 3000 ms asked for by `esp_task_wdt_init(3, true)` (`CONFIG_ESP_TASK_WDT_TIMEOUT_MS` is not even this SDK's symbol — it uses `CONFIG_ESP_TASK_WDT_TIMEOUT_S 5`, which decides nothing here); and the mechanism the review assumed is not just wrong in detail, it is absent: `esp_task_wdt_init` in this build subscribes **no idle task at all** | Round D item 1 |
| "the warm-up extraction was implemented but did not survive concurrent edits" (Round A residual 4) | the code **was** on disk, and its arithmetic wrapped: a signed `120000 - millis()` turns positive past ~24.9 days, so the gate refused calibration forever and printed an unrelated countdown. Wrong once in the review's framing, wrong twice in the fix | Round E item 4 |
| "nothing tests that the doc table and the JS array agree" (Round A residual 5, repeated verbatim into Round E's open list) | false when Round E wrote it: Round D had landed `test/test_flaglabels`, which parses `comms/flags.h`, `FLAGNAMES` in `src/comms/web_page.h`, the flashed `data/index.html` **and** §4's table in `docs/HANDSET_PROTOCOL.md`, asserting §4 publishes exactly bits 0..7 in order with each pill's keyword. Batch D's mutation table shows shortening §4's bit-3 row reddens it | Round E item 1's follow-up; commit `b9922fd` |
| "the fences are compiler-level guarantees, unobservable natively; only an ARM compiler at `-O2` could show the difference" (Round A residual 3) | the honest version is narrower and worse: an `env:native_o2` gate on **x86-64** shows **874/874 green with both fences commented out**, at `-O0` and `-O2`. So this host cannot witness them at all; on the target each fence is one `memw` | Round E item 5, `tools/fence_probe.cpp` |
| "`vbat` is a pack voltage; `batlow=0` means the pack is above `vbatMin`" (implicit in the alarms, predive and the dive snapshot) | the bench drove the sense pad to its clamp and `vbat=34.71` **cleared the dive-blocking battery step on a 5 V feed**. No upper bound or validity check existed anywhere in `src/ include/ test/` at the time it was measured | §1 P1 — the bound exists now (`VBAT_IMPLAUSIBLE_V`, `Battery::implausible()`); the claim is still not safe as a *measurement* until the divider is metered, and `vbat` still says nothing about the 12 V rail — that rail now has its own sense input and verdict, compiled off until fitted (§1 P9) |
| six citations **this file itself** carried when the five documents were merged — `esp_task_wdt_reconfigure()`, `eventInstant`/`commitEvents`/`abortEvents`, `test_cfgparity`, `__atomic_thread_fence`, "all four fences deleted", and `CONFIG_ESP_TASK_WDT_TIMEOUT_MS=3000` — plus two numbers, "every measured `worst=` sits at 4–9 ms" and "CPU0 idle monitored" (2026-11-27, the audit run after the merge) | none of those identifiers exist in this tree: the real call is `esp_task_wdt_init(3, true)`, the staging pair is `beginTx()`/`commitTx()`, the fences are `std::atomic_thread_fence` and `src/deco/view.h` has **two**, and the parity anchors are in `test_configbox` and `test_settings`. "4–9 ms" described only the storm runs: the repo's 639 `worst=` samples peak at **505 ms**. And "CPU0 idle monitored" repeated, in the very section about verifying claims, the exact premise Round D had disproved. All corrected in place. The finding for the reader: a citation copied out of a summary is not a citation — **grep the identifier before you put it in a sentence** | §1 and §2 as they now stand |
| a dive-file grader that printed **PASS** while the file contained `M,` total 0, and a pulse witness that would have reported a PASS on a dive that injected nothing | both now say so: `tools/hw_accept.py` prints a vacuity note when the total is 0, and `tools/pulse_witness.py` exits **2** rather than print a vacuous PASS. A check that cannot witness anything must say so | Round E item 2's follow-up |
| "`tools/float_audit.py` is in the gate" (BENCH.md's gate list, where it has been since the float bug came back the first time) | the list was true and the practice was not: the battery build was flashed on 2026-11-28 carrying `floor %.2f V, ceiling %.2f V` in a **SensorTask** `printf` — a violation of the rule, produced while the rule was open in the same session. The audit was run *after* the flash, when it found the violation; nothing had stopped the flash. `vbat=34.71` on this very bench is the reason that task's stack matters. What closed it is a hook, not a reminder: `tools/upload_gate.py` runs the audit as a pre-action on the upload target, witnessed reddening a planted `%.2f` with **no `Wrote` line** while the same tree still built | C2, "And the flash does it for you" |
| "the settings table in `docs/HANDSET_PROTOCOL.md` §6 documents the keys the board has" | it documented 30 of 36 keys, and published `VBAT_MIN` as type F with default 10.5 and band 3.0 to 30.0 after that key had been re-derived from the chemistry — a table a handset builder would build against and the parser would refuse. Nothing graded it, even though §4's flags table has had a doc-vs-code join since Round D | `test_the_published_settings_table_matches_the_registry_it_documents`, mutation-checked by editing the doc (old row, drifted registry default, deleted row — each reddens it by key name) |
| a date the bench's own clock contradicts | six doc lines written today — four in REVIEW.md, two in BENCH.md — read **2026-11-29**. `date` here says 2026-11-28, and so do all three commits those lines describe (`702067e`, `666b852`, `56256ab`). Nothing checks a date, so the error would have outlived the session that made it — the same family as the `src/main.cpp:1373` citation that had drifted onto a comment while the constant it named lives at `include/config/defaults.h:223` | corrected in place: all six dates to 2026-11-28, and the drifted citation replaced with the two lines that actually enforce the budget (`defaults.h:223`, `main.cpp:1524`/`:1543`). Found by running `date` before quoting the notes, which is the rule in §1 applied to my own working memory |
| "the console cries `requestFrom(): … Error -1` every ~6 s on a healthy bench" — said in this session, and nearly repeated into a doc draft | counted instead of reasoned: the repo's four acceptance captures carry the line **once per boot** (the LPS28 two-address probe, `src/sensors/lps28.cpp:15`, on a board whose chip sits at `0x5D`), and the only recurrence anywhere is pairs every ~25 s in `logs/soak_20261122_094738.log` — a run from the day before the LPS28DFW was fitted, where an unlocked arbiter recirculating is correct behaviour. No log shows a ~6 s cadence, and the `i2cerr=0 fresh=111` half of the sentence came from a tail that was never captured | §1 P10 rewritten around the counts, and the claim marked as an observation the bench owes a capture for rather than a finding it owes a fix for. Two other attributions fell out of the same count: the line is not the removed depth chip (`firmware.map` has 0 `BMP085` symbols), and `Wire.cpp:532` was the wrong core line — this core logs it at `Wire.cpp:499` |
| the reply-window rule recorded as **closed** — "nine host cases grade two consecutive windows and five planted mutations each reddened a case" (this file's own §2 row, `240652a`) | the rule's first live drive disproved the sufficiency of that evidence, not the rule: the drain's only exit was a silent line and a bench with 2 Hz telemetry never goes silent, so `GET *`'s drain ran 801 s without ending (`logs/stall_repro_run2_stdout.log`, last console line `[801.32s]`; two earlier full batteries killed with the console log frozen at its 2260 B boot flush and no phase log written at all). The host suite is **structurally** blind to this class: every fixture is a finite list, so its last read answers `""` — the exact courtesy a live bench never extends | `a7fa5b1`: the drain bounded twice (the window's deadline re-checked inside the loop, and at most 10 consecutive stepped-over non-family lines — a family is ONE `write()`, so at 2 Hz ten volunteered lines means it is over), plus a 4000-line never-quiet fixture whose bound is asserted as a literal so widening the constant cannot mutate the test green. Three further mutations reddened exactly their named cases. §2's row rewritten to say what the nine cases could not see |
| "`M,` total 0 beside zero `S,` windows" had been passing the wall-of-zeros check for weeks, read as the chain honestly injecting nothing | the input was never non-zero: this bench's pack sat unplugged, so the injector counter never left 0 and the shape the check hunts was **unreachable**, not absent. Plugging the pack in for the battery leg is what made the defect visible — `M,` total 3, every `S,` window 0 | §2's new row plus `cd82d4b`: `DiveTotals.totalPulses` was fed the boot-cumulative counter, so the pre-dive solenoid test's pulses were charged to the first dive. Both directions witnessed on hardware: pre-fix reddens (`logs/accept_20261128_183319_*`, `_184225_*`), post-fix 181 checks 0 failures (`logs/accept_20261128_185837_*`). The general lesson is §1's: a green check on an input that cannot move is not evidence |
| "the refused 10.5 is **not** written back, so that line repeats every boot until a `SET VBAT_MIN` replaces it" (BENCH.md, battery item) | half right and the wrong half load-bearing. `nvs_store.cpp::load()` does not write back — but `Store::save` rewrites **every** key from the in-force registry, so the first settings write of **any** key buries the stale string. Seed run, 2026-11-28: boot read `settings: 35 keys from NVS` beside `settings: NVS VBAT_MIN="4.400" refused (out of range) - …`; the matrix's own `SET SP2=0.55`/`SET GF_LOW=90` saved the registry; the next boot read `settings: 36 keys from NVS` with no refusal and `GET VBAT_MIN` already `3.400`, with no `SET` ever aimed at that key | BENCH.md's battery item rewritten around the mechanism, `hw_accept.py`'s finding text now says it, and the footprint is arithmetic rather than prose: the seeded matrix is 123 checks (`logs/accept_20261128_nvs_migration_seed_123p_0f.log`), the same matrix on a clean NVS is 118 (`logs/accept_20261128_after_seed_clean_118p_0f.log`) — 123 − 118 = 5 checks that exist only because a refusal existed |


---

## 6. Consolidation log — what happened to the five documents

**Merged:** `docs/REVIEW_BATCH_A_2026-11-27.md` (282 lines), `_B_` (188), `_C_` (195),
`_D_` (385), `_E_` (355) — 1405 lines — into this file, then deleted. They were removed
rather than kept as a trail, because two open lists in one repo is how an item gets worked
twice or never; git history and the artifacts in `logs/` hold the evidence, and the last
commit holding them whole is named below.

**What was changed in the merged text, and only this:**

* each file's H1 dropped, replaced by a `### Round X — …` header inside §3;
* every `##` heading in a body demoted to `####` (and `###` to `#####`) so the section
  numbering of this file is unambiguous;
* each round's own "Still open" / "Residual risks" heading marked **superseded by §1**, so
  a stale list cannot be mistaken for the current one — the text underneath is unchanged,
  because it is what the §1 items were derived from;
* intra-document links to `REVIEW_BATCH_X_2026-11-27.md` retargeted to the round's section
  in this file. No other sentence was edited: the symptom/fix/test/verdict prose, every
  measured number, and every table is carried through verbatim.

**References rewritten to point here:** `BENCH.md` (the §C4 copy-size finding, and the
battery/warm-up/cross-check mentions), `PLAN.md` (§1's pin-table note and the review
pointer), `platformio.ini` (the `native_o2` gate comment), `tools/fence_probe.cpp` (the
m-4 residual citation), and the `hw_accept.py` / `pulse_witness.py` header comments that
cited batch letters. Nothing in `test/` read a review document, so no test changed.

**Re-run after the merge, on this tree, 2026-11-27:** `pio test -e native` **437/437**;
`pio test -e native -e native_o2` **874/874** — the two suites that read a *document*
(`test_flaglabels` parses §4 of the spec, `test_designwire` parses the design sheet) stayed
green, which is what shows nothing in `test/` depended on a removed review file;
`pio run -e esp32` SUCCESS at **RAM 56292 B (17.2 %)** and **Flash 1037513 B (79.2 %)**,
byte-identical to the pre-merge build, as expected when the only source change is a
comment; `pio run -e esp32_stalltest` SUCCESS; `tools/hw_accept.py --selftest` **131 legs,
0 failures**; and `tools/fence_probe.cpp`, whose citation was rewritten, recompiles clean.

Then a repo-wide grep for the removed paths:
`grep -rn 'REVIEW_BATCH' --include='*.md' --include='*.h' --include='*.cpp' --include='*.py' --include='*.ini' .`
returns nothing outside this file. The only mentions of the old filenames anywhere are
the five in §6 and the one in the header above — prose about the merge, not paths anything
has to resolve. `grep -rn 'docs/REVIEW'` resolves to this file everywhere it appears.

**Recovered from:** the five documents exist intact in git at the commit before the removal
(`git log --diff-filter=D --name-only -- 'docs/REVIEW_BATCH_*'` names it), and in
`docs/` history since `0eabfdd`. If this file ever contradicts a number in those
documents, **the documents are the primary record and this file is wrong** — fix it here
and say so in §5.

