# Review Batch A — what was fixed, where, and what is still open

Scope: findings **M-2, M-3, m-1, m-2, m-4** and one nitpick from the 2026-11-27
review. Bench state: base `038df54`, native suite **398/398** (was 393),
`pio run -e esp32` and `pio run -e esp32_stalltest` both SUCCESS, no new
warnings from the touched sources. Nothing committed; `graphify-out/` left
untracked and untouched. **No hardware was flashed or read** — everything below
is code, static SDK evidence and native tests.

---

## M-2 — the task watchdog was never initialised, and the 3 s was a fiction

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

## M-3 — CNS overload was dropped on both transports

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

## m-1 — two 32-bit wrap bugs whose failure direction was a *lie*, not a hazard

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

## m-2 — legal-per-key values that were inverted *together*, silently

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

## m-4 — the published deco view had no ordering guarantee between payload and seq

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

## Nitpick — three dead `eraseAll()` functions

`DiveStore::eraseAll`, `SurfaceStore::eraseAll`, `ContinuityStore::eraseAll`
deleted (30 lines, `git diff --stat src/log/`): no caller in `src/`, `test/` or
`tools/`. `BENCH.md:96-101` already documents the real route (esptool
`erase_region 0x290000 0x160000` over the LittleFS partition, then the mount
that formats it), so removing them removes an attractive-nuisance that would have
silently wiped dive history from a debug build.

`DiveStore::exportRaw` remains dead — out of batch scope. `pendingRecord` is
**not** dead: `corrupt()` uses it.

---

## Gates and test count

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

## Residual risks

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
