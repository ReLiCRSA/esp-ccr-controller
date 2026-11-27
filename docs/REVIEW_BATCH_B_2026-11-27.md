# Review Batch B — event-row honesty, one instant per batch, and narrowing that cannot lie

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

## 1. The event ring is staged, so an edge cannot be read against a stale plan

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

## 2. One drain batch describes one instant

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

## 3. `ndl` is `-1` on the only rows that have no picture

`fillSnapshot`'s no-picture branch used to leave `ndl = 0`. `0` minutes means "your
no-deco time is finished, you are obligated now" — a reading, and the most alarming
one available on a row whose picture does not exist. It now writes
`deco::kNDL_UNKNOWN` (-1), the marker `D,` has always used for "no number", so one
client rule serves the wire and the file. `ceiling_m`/`ceiling_cm`/`deco_min` stay
`0`: the row format defines no absence for them, and a negative ceiling would read
as a *shallower* obligation, not as no picture. Pinned in
`test/test_logfmt/test_divelog.cpp` and documented in `docs/HANDSET_PROTOCOL.md` §5.

## 4. Narrowing saturates; it never wraps

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

## Still open, in the order I would take them

1. **The bench has not seen this code.** Re-run `tools/hw_accept.py --expect-web
   --handset /dev/ttyUSB1 --cal-air` after flashing with an explicit
   `--upload-port` (two identical CP2102 boards: controller `ttyUSB0`, handset
   bridge `ttyUSB1` — `tools/port_id.py`).
2. **Batch C: `g_ctrlCfg` double-buffering.** `postConfig()` copies a struct while
   ControlTask reads it; the ordering clamps and the settings apply path both touch
   that one object.
3. **The 3 s TWDT trip has never been measured**, and
   `CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU0` watches an idle task nobody feeds.
   A detector whose trip time is unknown is a claim, not a guarantee.
4. **The clamp verdict is console-only.** `GET GF_LOW` answers what was typed while
   the chain runs the clamped pair. Fixing it properly means a protocol field,
   which is a decision rather than a fix.
5. **Flag labels are duplicated three times** — `web_page.h`, `data/index.html`,
   §4 of the spec — with nothing checking that they agree. The M-3 fix edited all
   three by hand.
6. **`pulses` on event rows is always 0** (`Telemetry::pulsesWindow` is read and
   written nowhere). Making it real moves a byte on an existing column: a protocol
   decision, documented as one in §8 rather than taken as a side effect.
7. **`LOG EXPORT` dive-lock policy** remains undecided (Batch A left it open).
8. **`src/log/continuity.cpp:135,144` still rounds with `+ 0.5`** while the repo's
   stored-number rule is `numfmt::scaled` (nearest, ties to even, decided on the
   exact product). Both sites are range-gated so nothing wraps; the residue is that
   a stored CNS or tissue count can sit one count away from the digits
   `numfmt::fixed` printed for the same float, on an exact tie. It changes bytes in
   a stored record, so it wants its own commit.
9. **`graphify-out/` is untracked tool output** and wants a `.gitignore` entry;
   nothing in the repo references it.

**Done after this write-up, in `2c4f6fe`:** the stale air-calibration prose —
`BENCH.md`, the console line, the two enum comments, the web page's verdict
caption and the bench harness's hint still said "spread band" /
"spread-limited", the rule `66694f9` replaced. The wire was untouched (code 5 is
still 5); `BENCH.md`'s dated record keeps the old words with "as it was then",
because that entry is a measurement made under the old rule.
