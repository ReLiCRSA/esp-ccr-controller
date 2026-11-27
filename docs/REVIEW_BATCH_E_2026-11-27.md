# Review batch E, 2026-11-27 — batch D's "still open" list, and one fix that was itself a bug

Batches [A](REVIEW_BATCH_A_2026-11-27.md), [B](REVIEW_BATCH_B_2026-11-27.md),
[C](REVIEW_BATCH_C_2026-11-27.md) and [D](REVIEW_BATCH_D_2026-11-27.md) left seven
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

## 1. The clamp verdict became a protocol event, not a console line

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

## 2. `pulses` in an event-driven dive row stopped being a hard-wired zero

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

## 3. `LOG EXPORT` is dive-locked

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

## 4. The warm-up gate: batch A's fix was wrong, and wrong twice

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

## 5. The seqlock fences: an `-O2` gate now exists, and it says the claim is unprovable here

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

## 6. `DiveStore::exportRaw` deleted

Dead: nothing called it, and a second export path in a store whose whole contract is
"one reader at a time" is the kind of surface that gets used by accident three
commits later. `e0c3b25`. The suite went green without it — the honest test that a
function is dead is deleting it and running everything.

## 7. The config box publishes all 284 bytes — measured, and left alone

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

## Verification, batch E

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

## Still open

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
5. **Flag labels in two places** (batch A open item 5): `web_page.h` and
   `data/index.html`, kept in step by `tools/sync_webdata.py`. This batch regenerated
   them for id 19 and re-ran the acceptance legs that read them, but nothing yet
   parses §4's table against the JS array.
6. **The config box's 284-byte payload** — §7 above: measured, deliberately not done.
7. C3 bench items unchanged: MS5837 swap/potting, battery measurement, mechanical
   solenoid inhibit, fails-closed rig, and the handset UI itself.
