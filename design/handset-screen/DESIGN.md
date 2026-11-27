# ESP-CCR wired handset — UI design

**Panel:** 320 × 172 px landscape colour OLED. **Input:** UP, DOWN, ENTER.
**Refresh:** the controller sends `T,` at 2 Hz and `D,` at 0.5 Hz; that is the
frame budget the UI is designed to, not an animation budget.

Sources of truth this design is derived from, in precedence order: the shipped
code (`src/control/`, `src/comms/`, `handset/handset_link.h`), then
`docs/HANDSET_PROTOCOL.md`, then `handset/README.md`. Where this document and the
code disagree, the code wins and this document is wrong.

Files here:

| file | what it is |
|---|---|
| `tokens.css` | the token set for any CSS consumer (web config page, simulator) |
| `screens/*.svg` | 41 states, each exactly `viewBox="0 0 320 172"`, real `<text>`, real bench values |
| `index.html` | contact sheet at true 1:1 size, with 3× zoom and sunlight / low-light / no-colour legibility toggles. Opens from disk, no network |
| `nav.svg` | the three-button state machine: edges, nesting depth, alarm override |

---

## 1. The panel's budget

Three facts decide everything else.

**It is small.** 320 × 172 px is roughly 1.5 in diagonal. At that size a 10 px
label is unreadable in daylight and a 11 px label is the floor. Nothing in this
design is smaller than 11 px, and the two numbers that matter are 44 px.

**It is read through a mask, in daylight, with gloves.** The legibility toggles
in `index.html` are not decoration: "bright sunlight" cuts contrast and washes
the panel, "low light" dims it, "no colour" removes hue entirely. A screen that
fails any of those three is not a screen, it is a demo. The sheet's own chrome —
the desk it sits on, the labels under each card, the wash applied by the sunlight
toggle — uses its own neutral greys; those surfaces are a review tool, not part
of the panel, and no colour outside the token list is ever drawn inside a screen.

**It refreshes at 2 Hz.** A 500 ms frame means any looping animation is a
stroboscope a diver can count. Animation here is limited to one-shot transitions
(120 ms) and bars that grow (250 ms), and `prefers-reduced-motion` removes even
those. Nothing on this panel blinks. A critical alarm is loud because it is a
*solid filled band*, not because it flashes.

## 2. Tokens — colour is meaning, and only meaning

Contrast is WCAG relative-luminance, computed against the panel black `#05080C`
and the raised strip `#101A24`. The AA floor for 11 px text is 4.5:1.

| token | hex | meaning | on `bg` | on `panel` |
|---|---|---|---|---|
| `bg` | `#05080C` | the only background | — | 1.14:1 |
| `panel` | `#101A24` | raised strip: reply band, selected row | 1.14:1 | — |
| `track` | `#16232F` | empty gauge / viewport track | 1.26:1 | 1.10:1 |
| `line` | `#263645` | 1 px separator, neutral outline | 1.62:1 | 1.42:1 |
| `ink` | `#EAF1F8` | primary text | **17.62:1** | 15.42:1 |
| `dim` | `#8DA2B4` | labels, captions, notes | **7.61:1** | 6.66:1 |
| `live` | `#3ED0E6` | **the controller reported this now — measured this cycle, or answered this poll** | **10.86:1** | 9.51:1 |
| `ok` | `#35D6A0` | **a verdict the controller actually passed** | **10.78:1** | 9.44:1 |
| `warn` | `#FFB020` | **degraded, not alarming: a warning-class FLAGS bit, or a fault the controller reports without raising a bit** | **10.97:1** | 9.61:1 |
| `crit` | `#FF4B4B` | **alarm severity, stated by the controller: a critical-class FLAGS bit, an `ERR` refusal of something the diver could otherwise do, or a FAIL verdict** | **6.08:1** | 5.32:1 |
| `held` | `#6D8397` | **this is not this cycle's measurement** | **5.11:1** | 4.47:1 |
| `onfill` | `#05080C` | text drawn on a filled meaning band | — | — |

Two rules, and they are absolute:

1. **A meaning colour is never decoration.** Cyan appears only on a number the
   controller measured this cycle, or on the word that says so. Green appears
   only on a verdict the controller itself reported as passed — never on a
   receipt, never on an `OK` that merely means "the line parsed". Red appears
   only where the controller itself states alarm severity — a critical-class
   FLAGS bit, an `ERR` refusal, or a verdict it reported as FAIL. Amber appears
   where something is degraded but nothing is alarming: a warning-class FLAGS
   bit, or a fault the controller reports without raising a bit (`used=0` on
   `DS,`, `on=0` with a `held_by` on `WIFI,`). That distinction matters, because a red on a
   screen whose FLAGS byte is `0x00` would tell the diver an alarm is set when
   none is. Two refusals are deliberately **not** red: `ERR unknown key` and
   `ERR not implemented` are drawn in `held`, because they say the controller
   has no word for what was asked — nothing is wrong and nothing is degraded,
   the request simply predates this build. Red is for a refusal of something the
   diver could otherwise do: `ERR predive required`, `ERR dive locked`,
   `ERR cal locked`, `ERR safe mode`. If a screen needs a colour to be
   understood, that screen is broken;
   if a decorative element borrowed a meaning colour, a diver would learn to
   distrust the meaning.
2. **Every meaning is also a word and a shape.** The "no colour" toggle must
   leave every screen readable, and it does: critical is a *solid filled band*
   with dark text plus a word; warning is a 1 px outline plus a word; held is a
   grey-blue outline, the literal word `HELD`, and an age in seconds. Nothing is
   conveyed by hue alone, so nothing is lost to colour-blindness or to a washed
   panel.

`held` is the weakest meaning colour at 4.47:1 on `panel`, just under the AA
floor — so `held` is never used for running text on a raised strip. There it
becomes a fill or an outline instead. This is deliberate: a held value should
not be as loud as a measured one anyway.

`crit` sits at 6.08:1 rather than the 15:1 `ink` gets, on purpose. Red must not
be the most legible colour on the panel; if it were, every red label would beat
the depth number for the diver's eye. Red is loud through *area* (a filled band
across the whole strip), not through luminance.

## 3. Type — and why DEPTH and ppO2 are the biggest thing on the panel

| token | px | weight | used for |
|---|---|---|---|
| hero | 44 | 600 | DEPTH m, ppO2 bar |
| hero-2 | 26 | 600 | ceiling, secondary hero |
| title | 16 | 700 | screen titles, third-tier values |
| value | 13 | 500 | row values, the reply line |
| label | 11 | 500 | labels, chips, hints, captions |

One family for all of it — `DejaVu Sans`, falling back through `Noto Sans` to
any grotesque with the same two properties. Both properties are requirements,
and the second one is a defect I found in a rendered screenshot rather than a
taste:

1. **Uniform digit advance.** At 2 Hz a number changes under the diver's eye; if
   `1` and `7` were different widths the caption beside it would shuffle every
   frame and read as instability. In this face every digit is 1303/2048 em.
2. **`O` must not be `0`.** The monospace face on this bench puts a dot inside
   *both* glyphs, so the reply band came back rendering `CTRL · 0K TIME` where
   the wire said `OK`. On a life-support display a letter that can be read as a
   digit is not a typographic detail; `OK` is a word the diver acts on.

**Why 44 px is spent on exactly two numbers.** A diver's priorities underwater,
in order, are: *am I breathing the gas I think I am breathing*, *how deep am I*,
*how long can I stay*, *what must I do on the way up*. ppO2 and depth are the
first two, and they are the only two the handset can render at hero size on a
172 px panel. Everything else is a number you consult on a decision, not one you
watch continuously: ceiling and NDL at 26/16 px, stops and provenance at 11 px.
Consequences follow from that ranking, and they are worth stating plainly:
the deco plan is *smaller* than depth, so a diver who wants the plan presses
DOWN once. That is a good trade, because the plan only matters when it changes,
and depth matters every second.

The two heroes sit in **the same two bands on every state of the HUD** — labels
at y=58, values at y=100, provenance at y=114 — so the eye never has to re-search
for them after an alarm appears. The unit is folded into the label line
(`ppO2 BAR`) rather than sitting beside the digits, because a 5-character value
(`18.35`) at 44 px is 137 px wide and two of them plus a gutter is exactly 304 px.
There is no room for a unit and no need for one: the label is always there.

**The alarm never covers the numbers.** An alarm is a band at y=18..40, above
the heroes. A design that overlays the alarm on the readouts destroys the very
information the alarm is about.

## 4. The HUD band grid

Fixed bands, summing to 172 exactly:

| band | px | y | content |
|---|---|---|---|
| status | 20 | 0–20 | mode chip, battery V, clock, link chip |
| condition | 24 | 20–44 | the alarm, the warning, or `NO ALARMS` |
| hero | 74 | 44–118 | DEPTH and ppO2, side by side, both 44 px, each with a provenance caption |
| deco | 24 | 118–142 | ceiling (26 px), NDL (16 px), CNS (16 px) + gauge |
| stops | 14 | 142–156 | stop list, or a provenance line when there are no stops |
| reply | 16 | 156–172 | the controller's last reply verbatim, and the key hint |

The reply band is always present. It is prefixed `CTRL ·` when the line is the
controller's real reply and `HANDSET ·` when the handset made the sentence up —
including when the controller said *nothing*, which is why link loss reads
`HANDSET · NO LINE 4 s` and the system screen's band says
`HANDSET · GET DS · GET WIFI` rather than pretending to be an answer.
That distinction is the difference between an acknowledgement and a guess, and a
diver must be able to tell them apart at a glance.

The hint in the band's right corner is `dim` by default, and takes `warn` or
`crit` only when the affordance it names is itself a safety action: `HELD 1.5s =
RESTART` in amber on SAFE mode, `NO KEY DISMISSES IT` in red on link loss.

What follows `CTRL ·` is the controller's line **verbatim**, not a summary of it:
`CTRL · D,-1,0.0,0.00,0.0`, `CTRL · E,131,7,5`, `CTRL · OK SAFETY`. When a line is
too long for the band it is cut with `…` (`CTRL · DS,1,0,1,12,…`), which says *the
rest exists and is on this screen* — the full value always appears in a row above.
A band that paraphrased would let the handset's reading of a reply overwrite the
reply itself, and the whole point of the band is that the two are separable.

## 5. The three-button model

Three primitives, no more:

| primitive | behaviour |
|---|---|
| **UP / DOWN** | move the cursor in the current screen; on a list, move the highlighted row; **held ⇒ auto-repeat that accelerates** (~250 ms → ~60 ms) |
| **ENTER** | act on the highlighted thing: open a row, send a command, write a value |
| **ENTER held 1.5 s** | go home. From anywhere. The only universal exit |

There is no back key, so *home* is the exit. 1.5 s is long enough that brushing
the panel with a gloved hand cannot leave the dive screen, and short enough that
a diver who needs out gets out. It is labelled `HELD=HOME` in the reply band's
right corner on every non-home screen, because an affordance nobody can see is
not an affordance.

Two derived affordances cover everything else:

* **Cancel is `HELD` too, because there is nothing else to cancel with.** On a
  confirm screen and in the key editor, held ENTER goes home, and going home
  sends nothing: the arm window on `DIVE START` is dropped by leaving the screen,
  and an edited value is only ever written by a press of ENTER on the row that
  says so. `confirm-dive-start` says this out loud — `EXIT · HELD ENTER = HOME`
  and `NOTHING SENT · LEAVING DROPS THE ARM` — because a diver who thinks a
  screen has a cancel key will trust an arming that has already gone away.
* **Nesting is capped at 3.** Depth 1 is the three destinations; depth 2 is what
  the rail opens; depth 3 is the key editor. Anything wider would be a list, so
  it *is* a list: lists scroll with UP/DOWN instead of nesting. Six rows of 20 px
  fit between y=30 and y=148, plus a 3 px viewport bar on the right and an `n/m`
  count. A 36-key settings list is therefore shallow and scrollable rather than
  a folder tree a wet-handed diver can get lost in.

### Press budget

The budget that matters is the worst case *while something is going wrong*.

| from | to | presses | how |
|---|---|---|---|
| cold boot | HUD | 0 | home is the first thing on the panel |
| HUD | deco detail | 1 | DOWN |
| HUD | the rail | 1 | UP |
| HUD | commands | 1 | ENTER |
| HUD | **ACK a critical alarm** | **1** | ENTER — and the same 1 press from *any* screen while a critical bit is set |
| HUD | a rail screen (pre-dive, air cal, cells, system, log) | 2 | UP, ENTER |
| HUD | a settings group | 2 | UP, ENTER on SETTINGS |
| HUD | editing any of the 36 keys | 3 | UP, ENTER, ENTER |
| HUD | arm + send `DIVE START` / `DIVE END` | 2 | ENTER on the row, ENTER again |
| any screen inside the box | HUD | 1 | ENTER held 1.5 s |
| depth-3 key editor | HUD | 1 | the same held ENTER — the exit does not depend on where you are |

So: **worst case down is 3 presses, worst case back is 1 press, and ACK from the
HUD while critical is 1 press.** The last one is a design requirement, not a
convenience. `ACK` is the only command the handset makes *reachable from every
screen*: while a critical FLAGS bit is set, ENTER means ACK wherever the cursor
is, and the banner says `ENTER=ACK`. The override is drawn in `nav.svg` as a red
dashed edge across the whole navigable box.

**`ENTER=ACK` appears on critical screens and nowhere else.** `Manager::ack()`
(`src/control/alarms.cpp:109`) suppresses nothing but `Level::CRITICAL` bits, so
on a warning-only panel — battery low, setpoint clamp, CNS high, cell drift — an
ACK press silences nothing and the bit stays set until the condition recovers. A
screen that advertised ACK there would promise an effect the machine cannot
produce, and the diver would press it twice, then three times, while the buzzer
went on. Those screens therefore keep `ENTER=MENU`, exactly as the plain HUD
does — the hint slot is only ~84 px wide because a warning banner occupies the
left of the same band (rule 17) — and say the fact out loud in the note line —
`ACK DOES NOT SILENCE A WARNING` on `alarm-warn-batt`, `alarm-warn-clamp` and
`alarm-cns-high`. The
critical screens keep the 1-press ACK and put the complementary truth in the same
line: `ACK SILENCES THE BUZZER · THE BIT STAYS SET` (`alarm-ppo2-low`). The
drift direction this pins is one-sided on purpose: a warning losing its ACK
affordance is a diver who cannot silence a beep he was never able to silence;
a critical losing it is a diver hammering ENTER at a hypoxia.

### States a button cannot reach, and what that means

Three states are decided by the controller, not by the diver, and the UI says so
instead of pretending otherwise:

* **A critical alarm** appears because a bit in `T,` FLAGS is set. It clears when
  the bit clears. **The handset keeps no alarm latch of its own** — a fixed
  sensor stops alarming on its own, and `ACK` silences the buzzer without ever
  pretending the condition is gone.
* **Link loss** is a full-screen, loudest-thing-on-the-panel state that
  **no key dismisses**. It can only leave by the link returning: silence from a
  life-support controller is the fact, and a button that made it go away would
  hide the fact. Below the headline it shows the last received values, explicitly
  marked `HELD`, and the instruction to hold at a safe depth.
* **SAFE mode** is detected from the `T,` line's mode field, not from an `ERR` —
  the downlink never stopped, so the state is visible before the diver tries
  anything. Here `ACK` means something bigger, and the screen says so in the
  one slot reserved for button affordances — the reply band's hint reads
  `HELD 1.5s = RESTART` in amber, where every other screen says `HELD=HOME` in
  grey-blue. Holding ENTER
  is required so that a brush against the panel cannot reboot a controller
  mid-dive. And because `D,` is quiet in SAFE mode, the deco band shows
  `NO DECO PICTURE · CONTROL OFF` rather than a plausible `0.00 / 0.0` —
  zeros would be a fake deco plan.

## 6. Screen inventory — 41 states, each with its data source and cadence

Every screen below is an actual file in `screens/`, drawn with real bench values
taken from the protocol's own examples. "Source" is the wire line that produced
it; "cadence" is how often that line moves, which is the screen's refresh rate.

### Dive HUD — the home panel and its states

| screen | state | source | cadence | the lie it prevents |
|---|---|---|---|---|
| `hud-standby` | STANDBY | `T,0.11,…,STANDBY,FLAGS0x00,12.41` · `D,-1,0.0,0.00,0.0` · `TIME,…,1,0` | `T,` 2 Hz, `D,` 0.5 Hz | NDL `-1` drawn as a dash, not `0` |
| `hud-auto` | AUTO, dive running, no alarms | `T,18.35,0.712,…,AUTO,FLAGS0x00,12.41` · `D,17,24.5,6.00,6|4.5,3|20.0,12.3` · `DS,1,1,1,0,MS5837-30BA @0x76` · `TIME,1795523937,1,0` | 2 Hz / 0.5 Hz | the reference state: `FLAGS0x00` and the strip reads `NO ALARMS`. It previously drew `FLAGS0x10` beside `12.41 V` — the battery bit with a healthy voltage, a contradiction no rule catches except quoting the line |
| `hud-manual` | MANUAL | `T,…,MANUAL,FLAGS0x00,…` · `E,612,3,2` | 2 Hz | implies nothing about injection: it says *injection is yours* |
| `hud-held-ppo2` | ppO2 HELD, bit 3 | `T,18.35,0.712,…,AUTO,FLAGS0x08,12.41` | 2 Hz | a held number presented as live |
| `hud-held-depth` | DEPTH HELD | `DS,1,0,1,12,LPS28DFW @0x5D` | `GET DS` ~10 s | `T,` cannot tell held from measured; `used=0` does |
| `hud-ndl-unknown` | NDL unknown | `D,-1,0.0,0.00,0.0` | 0.5 Hz | `-1` read as "zero minutes left" |
| `hud-ndl-unlimited` | NDL effectively unlimited | `D,480,0.0,0.00,0.0` | 0.5 Hz | `480` read as "8 hours of bottom time" |
| `deco-detail` | the whole deco picture | `DECO -> D,17,24.5,6.00,6|4.5,3|20.0,12.3` | 0.5 Hz | `DECO` has no `OK` reply, so the band says `HANDSET · DECO SENT · NO OK` — the `D,` line *is* the answer; stop pairs are printed as the wire sends them (`PAIR 6|4.5`) |

### Alarm and link states

| screen | state | source | cadence | note |
|---|---|---|---|---|
| `alarm-ppo2-low` | critical, bit 0 | `T,4.20,0.183,…,FLAGS0x01,…` · `E,742,1,0` | 2 Hz | filled band; `ACK` silences the buzzer, the bit stays set |
| `alarm-sensor-fail` | critical, bit 3 | `T,12.10,0.698,…,FLAGS0x08,…` · `E,812,1,3` | 2 Hz | bit 3 aggregates sensor + valve + hypoxia; the screen says there is no split |
| `alarm-cns-high` | warning, bit 7 | `T,21.40,0.842,…,FLAGS0x80,…` · `D,9,38.0,5.50,5|3.0,96.4` | 2 Hz / 0.5 Hz | CNS has its own alarm bit *and* a standing gauge on the HUD; amber, not red, because `alarms.h` calls it a WARNING |
| `alarm-warn-batt` | warning, bit 4 | `T,9.80,…,FLAGS0x10,10.42` · `D,18,24.5,6.00,6|4.5,3|20.0,14.8` | 2 Hz | 10.42 V against `VBAT_MIN 10.50` — a warning, not a lockout |
| `alarm-warn-clamp` | warning, bit 5 | `T,12.30,…,FLAGS0x20,…` · `PROOF,5,004,004,2` | 2 Hz / on demand | says what capped the setpoint and where to lift it |
| `link-lost` | silence > 3 s | nothing arriving; `kLinkTimeoutMs = 3000` | immediate | undismissable; last values marked HELD |
| `safe-mode` | SAFE | `T,0.00,…,SAFE,FLAGS0x08,…` · `GET D -> ERR no deco picture: control not running` | 2 Hz | `ACK` here means RESTART and is labelled as RESTART, not silence; every other key comes back `CTRL · ERR safe mode` in red, because the press did nothing |

### Surface checks

| screen | state | source | cadence | note |
|---|---|---|---|---|
| `predive-running` | step 1 running | `GET PRE -> PRE,2,1,0,0,0,0` | poll 1 Hz while open | per-step status, never a single "OK" |
| `predive-pass` | passed | `PRE,2,2,2,2,2,1` | 1 Hz | the green is the controller's verdict, not a receipt |
| `predive-fail` | failed at the air step | `PRE,2,3,0,0,0,0` · `AUTO -> ERR predive required` | 1 Hz | shows the refusal beside the failure |

### Air calibration — the receipt/verdict split, drawn as four states

| screen | state | source | cadence | note |
|---|---|---|---|---|
| `cal-air-receipt` | code 0 | `CAL AIR -> OK CAL AIR` · `GET CALAIR -> CALAIR,0,0,0,0,0,0,0,0` | on demand | `OK CAL AIR` is a receipt. Code 0 gets **no tick**, ever |
| `cal-air-warmup` | code 3, warm-up | `CALAIR,3,4,0,0,0,0,0,9` | poll 1 Hz | countdown from 120 s after power-on, in seconds |
| `cal-air-sampling` | code 1, sampling | `CALAIR,1,2,7,12,1450,1454,1468,0` | 1 Hz | progress out of 12 (`7 / 12`), spread against the band, raw running counts |
| `cal-air-ok` | code 2, verdict | `CALAIR,2,0,12,27,1550,1554,1568,0` · `E,96,7,2` | 1 Hz | the only green on this panel; raw counts and spread shown as the evidence |
| `cal-air-failed` | code 5 | `CALAIR,5,6,12,48,1550,1554,1442,0` · `E,131,7,5` | 1 Hz | spread 48 over band 30, the outlier cell named, `COMMITTED NOTHING` |
| `cal-unsupported` | refused | `CAL O2 -> ERR not implemented` · `CAL DEPTH -> ERR not implemented` | on demand | the sequencer exists but is never instantiated — no tick, no arm state |

### Cells, proof, system, log

| screen | state | source | cadence | note |
|---|---|---|---|---|
| `cells-test` | pulsing | `TEST CELLS -> OK TEST CELLS` · `GET CELLS -> CELLS,2,003,004,7` | poll 1 Hz while open | masks are octal; labels are 1-based (`CELL 1/2/3`), wire bits are 0..2, and the mapping is stated on the screen |
| `cellproof` | FAIL, clamp lifted | `PROOF,5,004,004,2` · `CELLPROOF OVERRIDE -> OK CLAMP LIFTED` · `E,318,12,1` | on demand | names who lifted the clamp; a FAIL never rearms |
| `system` | source / radio / clock / battery | `DS,1,1,1,0,MS5837-30BA @0x76` · `WIFI,1,0,512,2` · `TIME,1795523937,1,0` | `GET DS`/`GET WIFI` ~10 s | identity text is displayed, never switched on; only `@0xNN` and `none` are stable |
| `system-held-depth` | owns the slot, not used | `DS,1,0,1,12,LPS28DFW @0x5D` | ~10 s | `locked=1 used=0` → the metres on the HUD are stale, with age |
| `system-radio-held` | intent on, radio actually down | `WIFI ON -> OK` · `WIFI,0,3,0,0` | ~10 s | `on` is what the transport obeys, not what was typed; `held_by` gets a word (`DEPTH`) *and* the raw `3`; the band carries the polled line verbatim (`CTRL · WIFI,0,3,0,0`), not the older `OK` from the write |
| `clock-unset` | epoch 0 | `TIME,0,0,0` | on demand | "NO CLOCK", never a 1970 date; says dives will carry no date, and that setting an epoch needs keys this handset does not have (`SET VIA WEB`) |
| `log-list` | directory | `L,3,1795523937,7` · `L,2,1795437100,0` · `L,1,0,4` | on demand | `seq 0` labelled as a pre-2026-11-24 dive; epoch 0 as UNDATED; rows sorted by `seq` descending with the no-`seq` rows last, and the footer says that in words — a directory whose order is unexplained invites a diver to read the top row as the newest dive |
| `dive-summary` | one dive | `M,1842,3410,0.196,1.320,412,64,1290,38,30,85,0.400,0.700,1.000,1.300` | on demand | `max_depth_cm` converted to metres where it is displayed, with the field named |

### Settings, commands, refusals, unknown states

| screen | state | source | cadence | note |
|---|---|---|---|---|
| `settings-list` | 8 groups | `GET SCHEMA` → 36 × `K,<key>,<T>,<min>,<max>,<flags>,<default>` | on demand | groups come from the flags byte, not from a hard-coded list |
| `settings-group` | SETPOINTS, mid-dive | `K,SP1,F,0.200,1.600,3,0.400` · `SET SP2=0.750 -> ERR dive locked` | on demand | dive-locked keys are **greyed, not hidden**, and still pressable |
| `settings-edit` | numeric editor | `K,GF_LOW,U,10,95,2,30` · `SET GF_LOW=35 -> OK SAFETY` | on demand | range and step in the title, default shown, the exact line it will send shown |
| `settings-string` | `AP_SSID` | `K,AP_SSID,S,1,16,1,ESP-CCR` · `GET AP_SSID -> AP_SSID=ESP-CCR` | on demand | read-only here; see §8 |
| `command-menu` | mid-dive | `CAL AIR -> ERR cal locked in dive` · `WIFI ON -> ERR wifi locked in dive` | on demand | retired commands are not offered: no `PULSE` button exists |
| `confirm-dive-start` | armed | nothing sent yet; the second ENTER sends `DIVE START -> OK DIVE` | 6 s window | `HANDSET · ARMED, NOT SENT` — the first press is not a write |
| `reply-err-predive` | refusal | `AUTO -> ERR predive required` | one-shot | the refusal is shown in the eye line, not swallowed |
| `reply-err-dive-locked` | refusal | `SET SP2=0.750 -> ERR dive locked` | one-shot | says the value did not change, and which key still can be written |
| `unknown-states` | newer controller | `CALAIR,7,…` · `WIFI,0,9,0,0` · `E,127,99,1` · `K,NEWKEY,Z,…` · `GET DS -> ERR unknown key` | any | one rendering: *"a state this handset predates"*. Never `0`, never `OK`, never a tick |

## 7. Honesty rules → pixels

Each rule is a claim the protocol makes that a naive UI would get wrong. Every
one has a specific pixel consequence and a screen that proves it.

| # | rule | where it lives in pixels | proven by |
|---|---|---|---|
| 1 | `D,` NDL `-1` means *no valid vote*, not zero; `480` means *effectively unlimited* | NDL drawn as `—` in `held` grey-blue with the caption `NDL -1 ON THE WIRE · NOT ZERO MINUTES LEFT`; `480` drawn as `>480`, never as `480` | `hud-ndl-unknown`, `hud-ndl-unlimited` |
| 2 | `T,` FLAGS **bit 3** means the voted ppO2 is not this cycle's belief | the ppO2 hero switches to `held`, its caption becomes `HELD · FLAGS BIT 3`, and a chip reads `HELD <age> s` | `hud-held-ppo2`, `alarm-sensor-fail` |
| 3 | `T,` cannot distinguish a measured depth from a held one — `DS,locked,used` does | `used=0` makes the DEPTH hero `held`, adds `HELD <age_s> s`, and puts `DEPTH HELD · AGE 12 s` in the condition strip | `hud-held-depth`, `system-held-depth` |
| 4 | `OK CAL AIR` is a receipt that a request was accepted, not a verdict; `CALAIR,` decides | the receipt appears as `CTRL · OK CAL AIR` with a `NOT A VERDICT` chip in amber; code 0 renders `NO CAL AIR SINCE POWER-UP` in `held` with no tick; green appears **only** at code 2 | `cal-air-receipt`, `cal-air-ok` |
| 5 | an `ERR` is a fact and must never be read as an `OK` | the refusal is shown twice: once at 16 px in the eye line, once verbatim in the reply band as `CTRL · ERR …` | `reply-err-predive`, `reply-err-dive-locked`, `cal-unsupported` |
| 6 | SAFE mode is visible in the `T,` mode field; `D,` is silent there | condition strip `SAFE MODE · CONTROL OFF`, the deco band replaced by `NO DECO PICTURE · CONTROL OFF`, and the reply band's hint slot reading `HELD 1.5s = RESTART` in amber instead of `HELD=HOME` in grey-blue | `safe-mode` |
| 7 | silence for >3 s is a safety event, not a UI state | full-screen loudest state: red band, `LINK LOST` at 44 px, held values, reply `HANDSET · NO LINE 4 s` — prefixed `HANDSET ·` because the controller sent nothing — with the hint `NO KEY DISMISSES IT` in red | `link-lost` |
| 8 | key flags are per key: bit 0 dive-lock, bit 1 safety | dive-locked keys stay visible in `held` with a `SAFETY+LOCK` chip; safety keys carry a `SAFETY` chip; the group list is built from the flags byte returned by `GET SCHEMA` | `settings-group`, `settings-list` |
| 9 | `DIVE START` / `DIVE END` are consequential | two presses: ENTER arms for 6 s with `HANDSET · ARMED, NOT SENT`, ENTER again sends; there is no cancel key, so the screen says `EXIT · HELD ENTER = HOME` and the stops line says `LEAVING DROPS THE ARM` | `confirm-dive-start` |
| 10 | the radio is a policy, not a switch: intent ≠ actual, and `held_by` names the reason | headline `AP DOWN · HELD BY DEPTH`, `INTENT (TYPED) on · answered OK` above `ON · AP UP 0` with the chip `TRANSPORT OBEYS`, caption `ON IS NOT WHAT YOU TYPED · HELD_BY 3 = DEPTH`. The raw `3` stays visible beside the word — the word is the reading, the number is the receipt, and the enum appends | `system-radio-held` |
| 11 | epoch 0 means *no clock*, not 1 January 1970 | `NO CLOCK` at 44 px in `held`, `EPOCH 0 · NOT 1970 · NEVER SET`, and `UNDATED DIVES` on the log row | `clock-unset`, `log-list` |
| 12 | CNS has its own bit (7) *and* is a standing quantity, and its severity is the controller's, not the handset's | the gauge sits on the HUD permanently at 90 px with a filled portion in the CNS's own colour; bit 7 raises a filled band and the amber warning treatment — amber because `alarms.h` lists `CNS_HIGH` as `Level::WARNING`, and a handset that promotes it to red cries wolf at a different alarm than the machine is raising | `alarm-cns-high`, `hud-auto` |
| 13 | there is no touch, the handset must not latch alarms, and no dive *graph* fits honestly | no hover/touch affordance anywhere; alarm presence follows `T,` FLAGS alone; the dive summary is numbers and one gauge, not a depth profile — a 320 px trace would be decorative | all screens, `dive-summary` |
| 14 | ids, codes and `held_by` values append and never renumber | one shared rendering: title `UNKNOWN STATE` with `NEWER CONTROLLER` beside it, each unknown value in `held` beside `UNKNOWN CODE / UNKNOWN CAUSE / UNKNOWN EVENT / UNKNOWN TYPE`, and the line `NEVER 0 · NEVER OK · NEVER A GREEN TICK`; `ERR unknown key` means *this controller cannot say*, so no `none` is invented | `unknown-states` |
| 15 | the header clock is a derived value, and its source has to be on the same screen | every screen that draws a clock quotes `TIME,1795523937,1,0` in its data-source line, and the header reads `12:38` — the UTC conversion of that epoch, checked against the same epoch the log rows use (`2026-11-24 12:38` in `log-list`, `2026-11-24T12:38:57Z` in `dive-summary`). A mockup whose clock and whose quoted epoch disagree is a mockup where the conversion was invented | every clocked screen, `log-list`, `dive-summary` |
| 16 | a meaning colour is only ever a FLAGS bit, a verdict, or staleness — never emphasis | `dive-summary` draws `0.196` and `64 %` in `ink`, not amber: `0.196` is above `LOW_BAR 0.18` and `64 %` is below `CNS_ALARM 80`, so neither is an alarm the controller raised. Emphasising a number that the machine considers normal is how a mockup teaches a diver to distrust the colour | `dive-summary`, `alarm-cns-high` |
| 17 | the panel is 320 px wide and the condition strip is shared with the key hint | widths are measured with the panel's own face (DejaVu Sans) at the drawn size plus its `0.4 px` letter-spacing, and the rule is that the strip's banner ends left of the hint's left edge. `ENTER=COMMANDS` is 117.8 px and collides with `WARNING · SETPOINT CLAMP` (banner ends at x=206.5), so the hint reads `ENTER=MENU` at 82.4 px; the 11 px notes in the provenance slot are capped at 300 px, which is why the battery line is `ACK DOES NOT SILENCE A WARNING · 10.42 V` and not a sentence about `VBAT_MIN` | `alarm-warn-clamp`, `alarm-warn-batt`, `alarm-cns-high` |

## 8. Decisions worth arguing about

**String keys (`AP_SSID` 1–16, `AP_PASS` 8–31) are not editable on the handset,
and the screen says why.** Three buttons cannot type. A character wheel on a
172 px panel is 26 positions per character, sixteen characters deep, operated
with gloves, for a value that is only ever set once — and a mistyped passphrase
produces a network the diver cannot join, which reads as a broken controller.
So `settings-string` shows the current value, its length limits, and
`EDITING · NOT ON HANDSET · ON WEB`, and the handset never sends `SET AP_SSID=`.
`AP_PASS`'s value is not echoed at all: the handset has no reason to put a
passphrase on a screen in a public boat. The web config page owns both keys; the
handset's job is to point at it honestly rather than to half-implement it. The
alternative — a wheel — was rejected because its failure mode (wrong value,
plausibly set) is worse than its benefit.

**ppO2 can read HELD while the cells are actually measuring, and that is
accepted.** `T,` bit 3 aggregates sensor-class faults, so the handset cannot tell
"my ppO2 is stale" from "something in the sensor/valve/hypoxia family is wrong".
The UI follows the bit rather than second-guessing it. The failure direction is
deliberate: it can over-report `HELD` (a diver sees a quiet grey number and
presses UP for the sensor screen) and can never under-report it. Over-caution on
a life-support readout is the only direction worth choosing. The aggregation is
named on the alarm screen so nobody invents a split that does not exist.

**SAFE's restart lives on `ENTER held 1.5 s`, labelled `RESTART`.** `ACK` means
RESTART in SAFE mode, which is a bigger action than silence, and a single brush
of a gloved hand must not reboot a controller. Holding the same key for 1.5 s is
the handset's only deliberate gesture, so it is reused rather than inventing a
second one; the label is the word `RESTART`, never `ACK`, because a diver should
never have to remember what a key means on a particular screen.

**The alarm never covers the readouts, and the readouts never move.** Condition
at y=18..40, heroes at y=44..118, always. An overlay that hides depth to announce
"depth sensor failure" destroys the number the alarm refers to.

**No dive graph.** A depth profile on 320 × 172 px at 2 Hz would be a decoration
whose axis nobody can read through a mask; the summary shows the four numbers a
post-dive conversation actually needs (max depth, duration, ppO2 min/max, CNS
peak) and names the wire field each came from — `M,` field 2 is centimetres, and
the caption says so, because a diver comparing two tools must be able to see the
conversion.

**The handset's own sentences are labelled as its own.** `CTRL ·` prefixes lines
the controller sent; `HANDSET ·` prefixes things like `ARMED, NOT SENT`, the
request echoes on the system screen, and the link-loss notice — the last of which
is the important one, because on that screen the controller's silence is exactly
the fact being reported. Nothing the handset inferred is allowed to look like an
acknowledgement.

## 9. What was actually checked, and how

* **Text overflow.** Every `<text>` in all 41 screens was measured against its
  cell and the 320 px canvas using the advance widths read straight out of the
  face the renderer uses (regular and bold), plus a pessimistic 0.70 em for any
  character the table lacks — the check over-reports rather than under-reports.
  Result: clean, no string crosses its band or the canvas. This is why some
  captions read tersely than I would have liked: the panel is the argument.
* **Nothing drawn on top of something else.** Horizontal overflow is only one
  way a 320 × 172 px panel lies. The build also computes each string's glyph box
  — cap height and ascenders taken as 0.76 em above the baseline, descenders
  counted only when the string actually has one — and reports any pair of strings
  whose boxes share pixels, any string poking out of a chip outline or crossing a
  bar it is not part of, and a left string and a right-aligned string landing
  closer than 8 px apart. Touching strings read as one unreadable string. Twenty
  collisions came out of the first pass (a calibration headline sitting on row 1,
  a refusal headline sitting on its own reply row, a status-strip legend touching
  `NO ALARMS`), and two more survived only because the box maths had been
  optimistic about cap height: a provenance line cutting through the `UNDATED
  DIVES` chip outline on `clock-unset`, and the 16 px `64 %` on `dive-summary`
  grazing the row above it. Each was fixed by moving a baseline — never by
  shrinking type.
* **Contrast.** Computed with WCAG relative luminance for every token against
  `bg` and `panel` (§2). Every meaning colour clears 4.5:1 on the panel black;
  the one token that dips below on the raised strip (`held`, 4.47:1) is never
  used as running text there.
* **Legibility simulation.** `index.html` renders each screen at true 1:1 and can
  wash it with glare and reduced contrast, dim it, or strip hue entirely. The
  grey-strip case is the colour-blind test: critical is still a filled band with
  a word, held is still `HELD` with an age, warning is still an outline with a
  word. The 3× view exists to check letterforms and 1 px strokes, because 1 px on
  a 320 px panel is the thinnest thing that exists and it must survive.
* **Every state is drawn, not described.** 41 SVGs, each carrying its state name
  and the wire lines that produced it inside `<title>`/`<desc>`, so a screen and
  its data source cannot drift apart when the file is read later.

## 10. Implementation handoff

**Wire → state.** `T,` drives the HUD and the condition strip (mode, battery,
clock, link, FLAGS). `D,` drives the deco band and deco detail. `E,` events are
facts, surfaced as captions, never as decorations. `GET PRE|CELLS|PROOF|CALAIR|DS|WIFI|TIME|LOG LIST`
are polled while their screen is open (~1 Hz for the check screens, ~10 s for
`DS`/`WIFI`), everything else is a one-shot reply into the reply band.

**Cell indexing.** The wire uses bits 0..2 and octal masks (`007` = all three,
`004` = cell 3). Screens label `CELL 1/2/3`. The mapping is written on the cells
screen, because an off-by-one here is a diver trusting the wrong sensor.

**Repaint rules for a 2 Hz frame.** Only the values that changed; the band layout
is fixed; the reply band changes only when a reply arrives; the only animations
are one-shot (120 ms) and a growing bar (250 ms). No blinking, no marquee, no
scroll-on-change of a hero digit.

**Selection.** One highlighted row per screen, drawn as a filled strip plus a 3 px
left bar — never colour alone. Lists show six rows, a 3 px viewport bar, and an
`n/m` count so the diver knows how much is off-screen.

**Never do these.** Never colour a value for emphasis. Never draw green for an
`OK` that is a receipt. Never render `0` for an unknown number. Never hide a
dive-locked key — grey it and let the press produce the real `ERR`. Never offer a
retired command (`PULSE`). Never let a key dismiss a link loss. Never latch an
alarm the controller has already cleared. Never show a date derived from epoch 0.

## 11. Out of scope, and what to revisit

* The physical handset's own firmware (key debouncing, the 1.5 s hold timer, the
  UART framing) — this is the screen contract, not the driver.
* Buzzer patterns: the protocol's `ACK` semantics are reflected in the copy
  (`ACK SILENCES THE BUZZER · THE BIT STAYS SET`), but tone design belongs with
  the firmware and needs its own honesty rule — a silenced buzzer must never
  read as a cleared alarm.
* The dive *replay* (`ENTER=OPEN` on a log entry) is reachable in the model but
  not drawn: whatever it shows must obey the same no-graph rule.

**Closed since delivery — `FLAGS` bit 7 (CNS high).** That bullet read: *"drawn
here as shipping because it is being added in `src/control/alarms.h` right now.
If it does not land, `alarm-cns-high` is the only screen that changes."* It
landed, so `alarm-cns-high` is a shipping screen, not a conditional one, and
the amber treatment is the controller's own severity rather than the handset's
judgement: `alarms::BIT_CNS_HIGH` is alarm-word bit 8
(`src/control/alarms.h:49`) at `Level::WARNING` (`src/control/alarms.h:132`),
raised at `src/control/alarms.cpp:103` when CNS reaches `cnsAlarmPct`, and both
transports project it into `FLAGS` bit 7 — the 2 Hz `T,` stream
(`src/comms/uart_link.cpp:155`) and the `GET T` reply
(`src/comms/cmd_router.cpp:192`). It is an append inside the byte's free top
bit: bits 0..6 kept the meanings §4 has published since v1, no row format
moved, so a handset that stops reading at `FLAGS` sees fewer facts, never
different ones. The two projections are pinned equal by tests, which is also
what keeps this screen's wire line honest:
`test/test_uartlink/test_uart_link.cpp:323` asserts CNS alone is exactly
`FLAGS0x80` — nothing else riding along, in particular not bit 3, a handset's
"go check the sensors" — and `test/test_router/test_router.cpp:570-593` asserts
`GET T` answers the same `0x80`, with the cell-proof pill (bit 6) coexisting as
`0xC0` without either swallowing the other. What remains unguarded is the copy
inside the SVG: `FLAGS0x80` appears in exactly one of the 41 screens and
nothing in this folder re-derives it from the firmware, so a renumbering of the
`alarms.h` ids would make that one `<desc>` stale rather than wrong on the
panel.
