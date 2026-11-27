# Review Batch C — the applied rules as one generation, not a global four tasks share

Scope: the finding Batch B left open as its item 2 — *"`g_ctrlCfg` double-buffering:
`postConfig()` copies a struct while ControlTask reads it; the ordering clamps and
the settings apply path both touch that one object"*. Bench state: base `e662b69`
(Batch B, pushed) with `896fca8` (this batch) on top. Native suite **412/412** (was
406), `pio run -e esp32` and `pio run -e esp32_stalltest` SUCCESS with no warnings
from the touched sources, and — unlike Batch B — **the bench ran this image**:
`tools/hw_accept.py --expect-web --handset /dev/ttyUSB1 --cal-air`, exit 0, 126 PASS.

---

## 1. Two things were wrong with the global, and only one of them is obvious

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

## 2. The box — `src/control/configbox.h`, `.cpp`

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

## 3. Ordering, and which way each bound cuts

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

## 4. Where the copy lives — and the bench result that decided it

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

## 5. Tests — `test/test_configbox/test_configbox.cpp` (6 cases)

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

## 6. Unchanged, and what stays open

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
