#!/usr/bin/env python3
"""C2 bench acceptance for the ESP-CCR (PLAN.md 15: "bench acceptance").

Answers one question after a flash: does the board boot clean, talk the
handset protocol, and REFUSE the things it promises to refuse? The refusals
are the point — every check below mirrors a host-tested invariant
(test_router / test_session), and a refusal that only holds in Unity is not
a refusal.

NEVER run this while tools/soak_logger.py is running: a second reader on
/dev/ttyUSB0 corrupts the soak logging (and this script would misread it).

Usage (after the soak ends and the new build is flashed):
  /home/wernervg/.local/share/pipx/venvs/platformio/bin/python \
      tools/hw_accept.py --expect-web --handset /dev/ttyACM1

  --boot-port PORT   USB-CDC console (default /dev/ttyUSB0)
  --handset PORT     USB-UART adapter (or a second ESP32 running
                     handset/bridge_esp32: on the classic board that port was
                     /dev/ttyUSB1, on the S3 bench it is /dev/ttyACM1 — the
                     board's UART bridge side, never the ACM0 native-USB side)
                     wired to the controller's UART2 17/16. Omit to skip
                     the protocol checks — boot checks still run.
  --handset-settle S seconds to wait after opening the handset port: that open
                     resets the board behind it (1.5 is enough for an ESP32
                     bridge to boot; 0 for a plain adapter)
  --expect-web       require the settings/session/web boot lines (post-A5d
                     firmware; M1 does not print them)
  --boot-wait SECS   console capture window (default 20)
  --replay FILE      grade a captured console log instead of opening a port
                     (host-side development of the boot checks; no hardware)

Exit 0 = every check passed. Non-zero = at least one failed, and the failing
lines are listed. Raw console capture is kept under logs/.
"""
import argparse
import threading
import inspect
import re
import sys
import time
from collections import Counter
from datetime import datetime
from pathlib import Path

# Same status-line contract soak_logger.py enforces (append-only fields):
HB_RE = re.compile(
    r"^HB st=(\d) c0=(-?\d+) c1=(-?\d+) c2=(-?\d+) fresh=(\d)(\d)(\d) "
    r"smp=(\d+) i2cerr=(\d+) rec=(\d+) vbat=([\d.]+) stale=(\d) batlow=(\d)"
)

# Boot console patterns. `must_none` firing is an instant FAIL; `expect_*`
# only apply when asked for (the acceptance run happens right after a flash,
# so the boot banner must be IN the capture window).
BOOT_BANNER = re.compile(r"=== ESP-CCR controller .* boot ===")
RESET_REASON = re.compile(r"^reset reason: ")
SETTINGS_LINE = re.compile(r"^settings: (\d+) keys from NVS")
CONTROL_LINE = re.compile(r"^control: SP=[\d.]+/")
WEB_AP_LINE = re.compile(r'^web: AP "([A-Za-z0-9_-]+)" on 192\.168\.4\.1')
# `log: LittleFS mounted, 10 dives, seq 3..12, used 36864 of 1441792 bytes`.
# The `seq a..b` group is the eviction evidence: a full FIFO always lists ten
# dives, so only the sequence range says whether the NEWEST dive survived (a
# board whose dives all predate the sequence field says `seq n/a`). Group `note`
# is the firmware's warning: empty on a clean mount, and it says so when the
# mount failed and the partition had to be formatted (which erases dive history).
LOG_LINE = re.compile(
    r"^log: LittleFS mounted, (?P<dives>\d+) dives?"
    r"(?:, seq (?:n/a|(?P<seqlo>\d+)\.\.(?P<seqhi>\d+)))?"
    r", used (?P<used>\d+) of (?P<total>\d+) bytes(?P<note>.*)$")
# `surface: 7 records, seq 3..9` / `surface: no records yet` /
# `surface: ** /surface.bin will not read ...`. The calibration and pre-dive
# records have no dive file and no handset command, so this line is the only
# view of that log the bench gets before someone downloads it, and the seq range
# is the capacity evidence (same reasoning as the dive FIFO's `seq a..b`).
SURFACE_LINE = re.compile(
    r"^surface: (?:(?P<n>\d+) records?, seq (?P<lo>\d+)\.\.(?P<hi>\d+)"
    r"|no records yet|(?P<bad>\*\*.*))")
# No `$` on that pattern: the empty-log line goes on to name the export
# (`no records yet (GET /surface.csv will be empty)`), and an anchored version
# matched none of the three forms — which read as a firmware bug for one run.
# One record as the machine reports writing it: `surface: record 12 id 7 value 2`.
# The seq in it is the join key to the CSV row, so a console line, a flash record
# and an HTTP row can be compared as the same event instead of as three claims.
SURFACE_RECORD = re.compile(r"^surface: record (?P<seq>\d+) id (?P<id>\d+)"
                            r"(?: value (?P<value>\d+))?(?: .*)?$")
SURFACE_FAIL = re.compile(r"^surface: record NOT written")

# The depth announce, both forms, captured off the bench link on 2026-11-24:
#   depth: LPS28DFW @0x5D ready, 1016.7 hPa
#   depth: ** LPS28DFW @0x5D identified, no sane pressure within 3 s (reads 0.0
#            hPa) - depth will not be used
# There is deliberately NO line when no depth sensor is fitted: the absence is
# the signal for "nothing fitted", so these checks grade whatever the board has
# unless --expect-depth says a sensor must be there.
DEPTH_READY = re.compile(
    r"^depth: (?P<who>[A-Za-z0-9._ -]+?) @(?P<addr>0x[0-9A-Fa-f]{2}) ready, "
    r"(?P<hp>-?[\d.]+) hPa")
DEPTH_NODATA = re.compile(
    r"^depth: \*\* (?P<who>[A-Za-z0-9._ -]+?) @(?P<addr>0x[0-9A-Fa-f]{2}) "
    r"identified, no sane pressure within (?P<grace>\d+) s \(reads (?P<hp>[^)]*)\)")
# Which identity+address pairs the firmware's identity gates accept (WHO_AM_I
# 0xB4 for the LPS28DFW at 0x5C/0x5D, PROM CRC for the MS5837 at 0x76/0x77). A
# depth line naming anything else means an unidentified chip won the depth slot.
DEPTH_IDENTS = {"LPS28DFW": {"0x5C", "0x5D"}, "MS5837-30BA": {"0x76", "0x77"}}
# Absolute pressure a surface sensor must be able to read. Loose on purpose -
# weather and altitude - because this check is not about calibration: it exists
# because the announce used to print a PLACEHOLDER. Before 2026-11-24 the line
# fired the instant the chip was identified, before the first conversion, and
# printed `ready, 0.0 hPa` on every boot, healthy sensor or not.
DEPTH_HPA_MIN, DEPTH_HPA_MAX = 250.0, 1200.0
AL_RE = re.compile(r"\bal=(0x[0-9A-Fa-f]+)\b")
BIT_SENSOR_STALE = 0x80          # alarms::BIT_SENSOR_STALE, bit 7 of HB's al=

# `clock: 2026-11-24T09:12:03Z (set 41 s ago)`, or `unset: ...`, or a value
# `carried across a reset, not verified since`. The boot must always state it:
# a dive log whose dates nobody can account for is the defect this replaced.
CLOCK_LINE = re.compile(r"^clock: ")
# The firmware's own FIFO depth (logfmt::MAX_DIVE_SLOTS, src/log/divelog.h). A
# full FIFO evicts a dive instead of growing, so "one more dive than before" is
# only the right expectation while a slot is free.
MAX_DIVE_SLOTS = 10
MUST_NONE = [
    (re.compile(r"SAFE MODE"), "SAFE boot"),
    (re.compile(r"FATAL"), "FATAL line"),
    (re.compile(r"session NOT initialised"), "session failed to initialise"),
    (re.compile(r"uart2 session NOT started"), "uart2 task create failed"),
    (re.compile(r"web AP failed to start|web: softAP failed|web: server allocation"),
     "web stack failed"),
]

# proto::MAX_LINE is 64; overrunning the handset line buffer is a tested
# refusal ("ERR line too long") — send one char past it, then more.
OVERLONG = "X" * 120

# (command, expected reply-prefix regex, why it is on the list[, "burst"])
#
# "burst" exists because `GET T` and `GET D` answer with a line the controller
# ALSO sends unprompted, so matching the prefix proves nothing: on 2026-11-23
# the harness scored `GET T` as PASS while the command was busy panicking the
# uart2 task, because a 2 Hz unsolicited `T,` line happened to arrive and matched
# ^T,. A burst check requires TWO matching lines within `burst_window` of each
# other, which the unsolicited cadence alone cannot produce (2 Hz = 500 ms
# apart), so a real reply is the only way to satisfy it.
#
# The clock entries stamp the board with the bench PC's wall time, which is
# what the dive log gets dated with: the controller has no RTC and the AP has
# no uplink, so a client at the surface IS the time source. Doing it here means
# every later phase (dive-persist, web_probe's CSV check) sees real dates
# instead of epoch 0, and `TIME SET 7` keeps the refusal honest.
_NOW_EPOCH = int(time.time())

# The ruler test pair. WATER_DENSITY is dive-locked because one density now drives
# the depth in `T,`, the setpoint tiers, the cellproof arm depth and the deco plan.
# The refused value is deliberately MIDDLE of the range, not an endpoint: a
# boundary value can be refused by a SECOND rule, and then the check passes for the
# wrong reason. Caught on the bench 2026-11-24 — asked for 0.990, the declared
# minimum, a build WITHOUT the dive-lock flag answered `ERR out of range`, because
# the parser compared a strtod double against a float bound and 0.990f is
# 0.99000000953674316. The entry saw an `ERR` and called it a dive lock. Both
# halves are fixed now (the parser compares in float, and test_settings asserts
# every schema key's own endpoints are reachable); the check still avoids boundary
# values, because "an ERR arrived" is never the whole question.
# A value away from the default matters for the other half too: if the lock leaks,
# the mid-dive read shows 1.000 where the dive should be running on 1.025.
DENSITY_REFUSED = "1.000"
DENSITY_RESTORE = "1.025"

# A string value past what the command parser can carry (its field is 40 bytes; 39
# characters is the longest thing that survives). 40 forces a refusal. The line
# itself is 52 characters, comfortably inside the 64-character LINE cap, so what
# refuses here is the FIELD cap — which is the whole point of the leg, and why
# grade_value_caps rejects `ERR line too long` as a wrong-rule answer.
VALUE_TOO_LONG = "z" * 40

PROTOCOL_CHECKS = [
    ("DIVE END", r"^(OK|ERR no dive)", "clean slate before the sequence"),
    ("GET T", r"^T,", "telemetry line parses", "burst"),
    ("GET D", r"^D,", "deco picture on request", "burst"),
    # `family:` = the reply is one write() of many lines: drain the family,
    # don't stop at its first line. See ReplyWindow for the rule and for why
    # a first-match window here grades the NEXT command's answer.
    ("GET *", r"^[A-Z0-9_]+=", "full settings listing", "family:^[A-Z0-9_]+="),
    # One line per key (36), so the family is `K,`. The NVS-migration leg grades
    # the refused key's band from these rows and needs them WHOLE; the checks
    # that follow need them GONE, which is what the pushback is for.
    ("GET SCHEMA", r"^K,SP2,F,", "the pages' ranges come from the server",
     "family:^K,"),
    # The chemistry migration of 1 P1, read back instead of taken on trust: the
    # boot refuses a stored VBAT_MIN the tightened band cannot hold and promises
    # the registry default runs instead. grade_nvs_migration joins that promise
    # against this reply. Never probe a refusal with a schema endpoint (bench
    # rule from 2026-11-24), so the value asked for here is read, never written.
    ("GET VBAT_MIN", r"^VBAT_MIN=[\d.]+$", "the migrated key's live value, on request"),
    # One `HIST,` line PER CELL (`src/control/cellhist.h:62`), all of it in one
    # reply, so stopping at the first match leaves two lines to answer the NEXT
    # command. A refused histogram is a single `ERR` line: the family only needs
    # to match the continuation, so `^HIST,` is enough and the ERR reply ends the
    # window on the next non-family line.
    ("GET HIST", r"^(HIST,|ERR no histogram)", "histogram answer, or the truth",
     "family:^HIST,"),
    ("GET FAULT", r"^FAULT,", "fuse/stuck-open/duty on request"),
    # Which chip owns depth, and whether the control path is using it (2026-11-25).
    # The matrix only asks for the SHAPE; the meaning is graded against the boot
    # announce in grade_depth_source below, because `1,1,1,0,...` says nothing until
    # you know which chip the console named at boot and whether "in use" is true.
    ("GET DS", r"^DS,\d,\d,\d,\d+,", "depth source answers on request"),
    # Radio policy (2026-11-26): the matrix asks only for the SHAPE here; the
    # MEANING is graded by grade_wifi below, because `WIFI,1,0,598` says nothing
    # until you know the AP was up at boot, what WIFI_IDLE_MIN is on this board,
    # and whether the countdown agrees with that setting from the same session.
    ("GET WIFI", r"^WIFI,\d,\d,\d+,\d+$",
     "radio state, hold reason and station count answer on request"),
    ("AUTO", r"^ERR predive required", "AUTO lockout on a cold boot"),
    ("SET SP2=0.55", r"^OK", "surface write lands"),
    ("GET SP2", r"^SP2=0\.550", "registry read-back"),
    ("SET SP2=2.5", r"^ERR", "range refusal"),
    # The clamp verdict as a protocol event (id 19, added 2026-11-27). GF_LOW
    # is legal per key at 90 (range 10..95) and inverts the pair against
    # GF_HIGH, so the applier clamps it — while the registry keeps 90, which
    # is exactly the gap id 19 closes. The read-back below PROVES the gap
    # exists on the wire; grade_stream grades the `E,<t>,19,<mask>` row that
    # names it, mask bit5 (GF_ORDER) alone: one clamp, one name. At the
    # SURFACE, because GF keys are dive-locked and a refused SET clamps
    # nothing. The dwell gives the row its control cycle.
    ("SET GF_LOW=90", r"^OK",
     "legal per key and inverted as a pair: accepted at the registry", "dwell:1.0"),
    ("GET GF_LOW", r"^GF_LOW=90$",
     "the registry keeps what was typed — the reason the verdict needs its own row"),
    ("SET GF_LOW=30", r"^OK", "the pair restored to the default: exactly one clamp row "
                              "for the whole run, and every later phase sees the plan "
                              "DIVE_WANT states", "dwell:1.0"),
    # Value caps on the wire (2026-11-24, see grade_value_caps): the parser's value
    # field used to be 23 characters while `AP_PASS` publishes 8..31, so a longer
    # passphrase answered OK and was stored short. Both writes here are REFUSED, so
    # the access point is never re-armed and no later phase waits on a station
    # rejoining — the leg costs three lines and no radio churn.
    ("GET AP_PASS", r"^AP_PASS=", "string read-back (the cap leg reads it twice)"),
    (f"SET AP_PASS={VALUE_TOO_LONG}", r"^ERR value too long",
     "a value the parser cannot carry is refused, never stored shortened"),
    ("GET AP_PASS", r"^AP_PASS=",
     "read again, so 'did the refusal keep the value?' is answerable"),
    # The clock goes up before the matrix's own dive, so THAT dive is stamped
    # with a real epoch too, and every later phase sees dated logs.
    # Calibration honesty (PLAN 16: `OK` meant "the parser accepted it"). CAL O2
    # armed a two-step handshake whose consumer (cal::O2Sequencer) has never been
    # instantiated on the board, and CAL DEPTH raised a request flag nothing ever
    # read; both used to answer OK, which on a rebreather reads as "tared" or
    # "calibrated". They are refused now, and the air calibration's real verdict
    # is readable instead of console-only — see the CALAIR entries below and
    # BENCH.md "Calibration honesty".
    ("GET CALAIR", r"^CALAIR,0,0,0,0,0,0,0,0$",
     "nothing calibrated yet: code 0, not a result invented out of a zero"),
    ("CAL DEPTH", r"^ERR not implemented$", "depth tare refused: nothing consumes it"),
    ("CAL O2", r"^ERR not implemented$", "O2 calibration refused: no sequencer on the board"),
    ("CAL O2 CONFIRM", r"^ERR not implemented$", "no phantom two-step arm to confirm"),
    # The warm-up gate is the interesting case: the reply to `CAL AIR` is the same
    # `OK CAL AIR` whether the cells are sampled or refused, so the verdict has to
    # arrive on its own. Under 120 s of uptime the gate refuses, and that refusal
    # must be visible on the protocol (and as an unsolicited E, row — graded after
    # the matrix, see the stream checks in main()).
    ("CAL AIR", r"^OK CAL AIR$", "accepted at the surface (the reply is only ever a receipt)", "dwell:1.5"),
    ("GET CALAIR", r"^CALAIR,3,\d+,0,0,0,0,0,[1-9]\d*$",
     "warm-up refusal is on the protocol with the seconds left, not only on the console"),
    (f"TIME SET {_NOW_EPOCH}", r"^OK TIME", "wall clock accepted from the bench"),
    ("TIME SET 7", r"^ERR epoch out of range", "implausible epoch refused, clock unchanged"),
    ("GET TIME", r"^TIME,\d{10},1,", "clock set since this boot, read back"),
    # "dwell:2.0" holds the dive OPEN before the next command. Without it the
    # whole matrix runs in under a second: DIVE START and DIVE END land 0.45 s
    # apart, the deferred radio teardown (loopTask polls at 1 s, stop() drains
    # 250 ms) never acts, the two intents cancel, and "did the dive take the
    # radio down" is unanswerable — measured on the bench 2026-11-23. A real
    # dive is minutes, so the dwell is what makes the check mean what it says.
    ("DIVE START", r"^OK DIVE", "dive opens", "dwell:2.0"),
    ("SET SP2=0.40", r"^ERR dive locked", "diver outranks the client"),
    # `LOG EXPORT` is dive-locked (decision 2026-11-27, §2/§5): the export
    # streams a whole dive off the same LittleFS flash the LIVE dive file is
    # being written to, on the task that keeps the handset pumped — the
    # configuration the review could not measure is made unreachable instead
    # of measured. The refused probe uses slot 1: with the matrix's own dive
    # the newest, a regression that removed the lock would stream rows here,
    # and the regex fails on the first of them.
    ("LOG EXPORT 1", r"^ERR export locked in dive",
     "no export mid-dive: the flash belongs to the live dive file"),
    # The depth ruler. `WATER_DENSITY` is the single density behind the metres in
    # the `T,` row, the setpoint tiers those metres select, the cellproof arm
    # depth and the deco engine's ambient pressures (see grade_depth_scale below),
    # so a mid-dive write moves the diver's whole depth scale up to ~2.5% (the
    # legal range is 0.990..1.100) at a pressure that has not changed. Refused
    # from 2026-11-24, like setpoints and alarms. The refused value is chosen FAR
    # from the default on purpose: if the lock ever leaks, the 0.990 is obvious in
    # the post-dive read-back below, and a surface-only comparison could never see
    # it (at the surface the whole density range moves the depth by 4 mm).
    (f"SET WATER_DENSITY={DENSITY_REFUSED}", r"^ERR dive locked",
     "the depth ruler stays frozen under a diver"),
    # Read the ruler back DURING the dive, immediately after the refusal. The
    # post-dive read alone cannot catch a leak: the matrix's own surface restore
    # rewrites the value, so by then a leaked 1.000 is gone and the box looks
    # clean. This is the line that sees which scale the dive is running on, while
    # it runs.
    ("GET WATER_DENSITY", r"^WATER_DENSITY=",
     "the ruler is readable mid-dive, to ask which value the dive is using"),
    ("WIFI ON", r"^ERR wifi locked in dive", "radio stays down in a dive"),
    # And WHO is holding it, in the machine's own words: off, blamed on the dive,
    # no idle countdown while down. The refusal above says the request was denied;
    # this line says the state is the one the denial promises. (held_by is an
    # append-only enum: 0 none, 1 diver, 2 dive, 3 depth, 4 idle.)
    ("GET WIFI", r"^WIFI,0,2,0,0$",
     "mid-dive: radio down, reason dive, countdown parked, no stations"),
    ("CAL AIR", r"^ERR cal locked in dive", "calibration never under a diver"),
    ("FROBNICATE", r"^ERR unknown command", "unknown commands do nothing"),
    # `PULSE` retired 2026-11-26 (HANDSET_PROTOCOL.md §8): no handset-originated
    # injection, and the older protocol this replaces let the verb work MID-DIVE.
    # So the pin is placed in the dive half, where the old firmware answered OK:
    # a handset built against the old line learns the truth at the parse level,
    # at the moment it used to be able to fire the valve.
    ("PULSE", r"^ERR unknown command", "the retired manual-pulse verb stays dead mid-dive"),
    (OVERLONG, r"^ERR line too long", "line buffer cannot be flooded"),
    # dwell covers the re-arm: loopTask applies it within its poll interval and
    # softAP + captive DNS + server start are all inside that one console line.
    ("DIVE END", r"^OK DIVE", "dive closes", "dwell:4.0"),
    (f"SET WATER_DENSITY={DENSITY_RESTORE}", r"^OK",
     "the ruler is writable at the surface (the lock is about the dive, not the key)",
     "dwell:1.0"),
    ("LOG LIST", r"^(OK|L,)", "log store answers"),
    # The density is not decoration: since 2026-11-24 it is the one number both
    # depth paths use — the depth in the `T,` row and the ambient pressure the
    # deco engine plans against. Reading it back is what lets the bench check
    # the two agree, below.
    ("GET WATER_DENSITY", r"^WATER_DENSITY=",
     "the density the depth scale runs on is readable by the client"),
]

# `GET T` and `GET D` answer with the same line shape the controller streams
# unprompted, so a prefix match proves nothing. The fix cannot be "require two
# matching lines close together": the reply can land exactly where the next
# unsolicited line would have, and then two lines 500 ms apart look identical to
# reply+unsolicited. What DOES work is aligning the send to the cadence: wait for
# one streamed line, send immediately after it, and require a matching line
# within BURST_WINDOW_S. The next unsolicited line cannot arrive for another
# ~500 ms (T, at 2 Hz, D, at 0.5 Hz), so anything inside the window is the reply.
# BURST_WINDOW_S of 0.45 leaves ~50 ms of margin against the 500 ms cadence —
# tight enough that a missing reply fails, loose enough that a reply (written in
# the same task pass that read the command) never does.
# One clock for every log this tool writes: the handset transcript and the
# console watch carry the same t= offsets, so a console line can be placed next
# to the command that should have caused it. That alignment is what was missing
# on 2026-11-23 when a radio check failed with "0 lines" and the two logs could
# not be read against each other. main() re-bases it when the run starts.
T_ORIGIN = time.time()

BURST_WINDOW_S = 0.45
BURST_ANCHOR_S = 1.5
BURST_REPLY_S = 1.0

# A crash in the middle of the command sequence must fail the run LOUDLY. On
# 2026-11-23 a panic mid-matrix looked like ten unrelated "no reply" failures
# plus one bogus PASS, and nothing in the report said the board had rebooted.
CRASH_MARKERS = [
    ("Guru Meditation", "panic"),
    ("Stack canary watchpoint", "stack overflow"),
    ("abort()", "abort"),
    ("ELF file SHA256", "panic dump"),
    ("** SAFE MODE", "latched into SAFE"),
    ("!! FATAL", "supervisor declared a fatal"),
    ("PRIOR FAULT", "fault record from a previous boot"),
]
HB_STACK_RE = re.compile(r"stk=(\d+)(?: u2stk=(\d+))?")

# H2 observability fields, appended to HB after stk=/u2stk= under the same
# append-only rule. `worst=` is the worst ControlTask cycle-to-cycle span
# since the last beat — exactly the quantity the supervisor measures when it
# decides to esp_restart() at TASK_STALL_CONTROL_MS = 1500 ms. The graded
# line is 800 ms: two full supervisor periods below the reboot budget, and
# orders of magnitude above anything the control chain legitimately costs
# (worst-case 30 s row fill, ndlMinutes + schedule() at the deepest load the
# depth sensor can legally report: ~1.35 ms host-measured, pinned by the
# test_controlplane budget test). A board beating at 900 ms is not rebooting
# yet — it is one long flash write away from it, and that is a finding.
# `logskip=`/`surfskip=`/`contskip=` are CUMULATIVE lost-record counters since
# boot (rowgate.h deferral ran out of road: a row the busy store never took;
# contskip= is the H1 one — a continuity snapshot that never reached flash,
# which widens the crash window the next dive seeds from).
# A healthy bench run loses nothing: a nonzero is a real hole in a dive file
# and is graded like one.
HB_CYCLE_RE = re.compile(r"worst=(\d+) logskip=(\d+) surfskip=(\d+) contskip=(\d+)")
# The grade is DELIBERATELY stricter than the reboot, and the decision is
# pinned by selftest_stall_budget(): it reads TASK_STALL_CONTROL_MS out of
# include/config/defaults.h and asserts this line stays below the budget it
# is a warning FOR. A board beating at 900 ms is not rebooting — the gap
# between "we warn" and "we restart" existing at all is the point, because
# the warning fires while there is still margin to act in. If the firmware
# budget ever moves, this constant has to be re-chosen on purpose, and the
# selftest is what says so in red instead of in prose.
# The measured record on this board is 505 ms (BENCH.md gate 5, 639 samples),
# so 800 ms sits well above every legitimate cost and well below the reboot.
HB_WORST_CYCLE_MS = 800

# Batch D: the applied-rules generation pair, appended after contskip= (again
# append-only). `<published>/<chain>` is how many control-configuration
# generations the settings path has published since boot, and which one
# ControlTask last read (src/control/configbox.h). The pair is what makes "the
# chain is running the rules the last SET produced" a bench-visible claim
# instead of a reasoning exercise.
HB_CFGGEN_RE = re.compile(r"cfggen=(\d+)/(\d+)")

# Batch E (docs/REVIEW.md 1 P9): the coil rail, appended after cfggen= under the
# same append-only rule. `vrail` is the divider-scaled rail volts and `railstate`
# the firmware's verdict: 0 sensed/in-band, 1 low for VRAIL_LOW_SAMPLES sweeps,
# 2 implausible, 3 NOT SENSED (VRAIL_SENSE=0 on today's bench, or an input that
# has never produced a reading). What this leg deliberately does NOT grade is
# whether the rail is IN BAND: those numbers are unmetered until the BENCH.md
# C3-c legs put a meter on the divider, and grading a calibration that does not
# exist yet is how vbat became P1. Shape and coherence are gradeable now, and
# the coherence rule is the one that keeps `railstate=3` honest: an input that
# says "there is no reading" must not also report a voltage.
HB_RAIL_RE = re.compile(r"vrail=([\d.]+) railstate=(\d)")

# The stall -> expiry interval measured on the controller 2026-11-27 with
# env:esp32_twdtmeasure / env:esp32_twdtmask (BENCH.md): 3061 and 3011 ms with
# only SensorTask subscribed, 3059 and 3019 ms with all four tasks subscribed.
# `esp_task_wdt_init(3, true)` asks for 3000 ms, and the excess is the ISR plus
# the console dump, so the timeout is confirmed rather than assumed — which
# matters because the 800 ms line above is the alarm threshold and 1500 ms is
# the supervisor's reboot budget: a control cycle that degrades past 3 s does
# not get a slow dive, it gets a panic and a SAFE boot. The number is quoted in
# the check's message so the margin is visible next to the threshold.
TWDT_MEASURED_MS = 3000
TWDT_MEASURED_SPREAD_MS = "3011..3061"


def grade_depth(lines, expect_depth):
    """The depth announce, graded against the rest of the boot.

    Three things, each a different lie:
      * the line names an identity+address pair the firmware's identity gates
        would actually accept (a near-relative chip getting a depth slot);
      * a `ready` line carries a pressure that is a READING, not a placeholder -
        this is the check that would have caught the announce printing 0.0 hPa
        from before the first conversion, which every healthy boot printed;
      * if the sensor identifies but never reads, the alarm word says so: a
        `no sane pressure` line with `al` clean would mean the console shouts
        about a dead sensor while the control chain believes it is fine.
    """
    checks = []
    ready = next((DEPTH_READY.search(l) for l in lines if DEPTH_READY.search(l)), None)
    nodata = next((DEPTH_NODATA.search(l) for l in lines
                   if DEPTH_NODATA.search(l)), None)
    als = [int(m.group(1), 16) for l in lines if (m := AL_RE.search(l))]
    stale = any(a & BIT_SENSOR_STALE for a in als)

    if not (ready or nodata):
        # No line at all: correct for a bench with no depth sensor fitted, wrong
        # for one that claims to have it. --expect-depth is how the operator says
        # which bench this is; C3-a runs with it.
        checks.append(("depth source announced at boot", not expect_depth,
                       "no `depth:` line: no depth sensor claimed depth"
                       + ("" if not expect_depth else
                          " — but --expect-depth was asked for, so a fitted, "
                          "identified sensor must announce itself")))
        return checks

    m = ready or nodata
    who, addr = m.group("who").strip(), m.group("addr")
    checks.append(("depth source is one the identity gates accept",
                   addr in DEPTH_IDENTS.get(who, set()),
                   f"{who} @{addr}, accepted {DEPTH_IDENTS}"))
    if ready:
        try:
            hp = float(ready.group("hp"))
        except ValueError:
            hp = float("nan")
        checks.append(("the depth announce carries a reading, not a placeholder",
                       DEPTH_HPA_MIN <= hp <= DEPTH_HPA_MAX,
                       f"{ready.group(0)} — want {DEPTH_HPA_MIN}..{DEPTH_HPA_MAX} hPa "
                       f"at the surface; 0.0 here is the announce printing before "
                       f"the first conversion"))
        checks.append(("HB does not report the depth sensor stale",
                       not als or not stale,
                       " ".join(f"al=0x{a:04X}" for a in als) or "no HB al= to compare"))
    else:
        checks.append(("a sensor that identifies but never reads raises the alarm",
                       bool(als) and stale,
                       (f"{nodata.group(0)} but HB says "
                        f"{' '.join(f'al=0x{a:04X}' for a in als) or 'no al= field'}: "
                        f"the console says no data while the chain believes there is")
                       if als else f"{nodata.group(0)} with no HB al= to compare"))
    return checks



# The firmware's own surface reference: SURFACE_PRESSURE_BAR in
# include/config/defaults.h. Stated here as a number on purpose — the tool is
# the independent witness, and if it imported the firmware's formula it could
# not catch the firmware's formula being wrong.
SURFACE_BAR = 1.013
SEA_DENSITY = 1.025   # the firmware default; the link's own value wins when it says


def depth_from_bar_abs(bar_abs, density=SEA_DENSITY):
    """Depth from an absolute pressure, stated in the tool, from the physics.

    rho [kg/L] * g / 100 = bar per metre — the /100 is the whole lesson of
    2026-11-24: written /100000 with a kg/L density, every depth came out 1000x
    too deep and the bench dosed for 37 m in an open room.

    The density is a PARAMETER, taken off the link (`GET WATER_DENSITY`), not a
    constant baked into the witness: it is the one number behind every depth this
    controller publishes. Defaulting to sea water is honest at the surface, which
    is where the boot announce puts this comparison — the whole legal range
    (0.990..1.100) is worth 4 mm there — but it is wrong anywhere deeper, and that
    is what the deeper case in `selftest_scale` pins.

    Never negative, mirroring the firmware's rule for what a diver reads: above
    the reference plane (altitude, weather, a sensor drifting high) the answer
    is 0 m, not a negative depth to dose against.
    """
    bar_per_m = density * 9.80665 / 100.0
    return max(0.0, (bar_abs - SURFACE_BAR) / bar_per_m)


def stream_density(stream):
    """The density the link itself reported, or None if it never said."""
    for l in stream:
        if l.startswith("WATER_DENSITY="):
            try:
                return float(l.split("=", 1)[1])
            except ValueError:
                return None
    return None


def grade_depth_scale(lines, stream):
    """The console's hPa and the handset's metres must describe one sample.

    This is the check the 1000x scale error cannot survive: the announce said
    1016.7 hPa while the same build streamed `T,37.04` on a bench sitting in an
    open room. Nothing downstream could see it, because the setpoint bands and
    the deco engine were both acting on the wrong metres *consistently*.
    Tolerance is 5 cm of water (~0.5 hPa): generous for weather between the boot
    line and the telemetry row, and three orders of magnitude tighter than the
    error being hunted.
    """
    ready = next((DEPTH_READY.search(l) for l in lines if DEPTH_READY.search(l)), None)
    if not ready:
        return []          # no reading on the console: nothing to compare
    try:
        hpa = float(ready.group("hp"))
    except (TypeError, ValueError):
        return []
    trow = next((l for l in stream if l.startswith("T,")), None)
    if not trow:
        return [("a `T,` row exists to compare with the depth announce", False,
                 f"announce said `{ready.group(0)}` and no `T,` row arrived: the "
                 f"two views of the same sensor cannot be reconciled")]
    try:
        shown = float(trow.split(",")[1])
    except (IndexError, ValueError):
        return [("the `T,` row's depth column parses", False, f"row: {trow}")]
    dens = stream_density(stream)
    used = dens if dens is not None else SEA_DENSITY
    want = depth_from_bar_abs(hpa / 1000.0, used)
    where = "the density the link reported" if dens is not None else "sea default"
    return [("the depth on the wire is the announce's pressure",
             abs(shown - want) <= 0.05,
             f"`{ready.group(0)}` at {used} kg/L ({where}) wants {want:.3f} m "
             f"below the surface reference ({SURFACE_BAR} bar); the wire says "
             f"{shown:.2f} m [{trow}] — 1016.7 hPa became `T,37.04` on 2026-11-24, "
             f"which is a kg/L density divided by 100000 instead of 100")]


def grade_density_lock(pairs):
    """Did the dive-locked ruler STAY put? The refusal's words are not the point.

    `SET WATER_DENSITY` is dive-locked since 2026-11-24: one density drives the
    `T,` metres, the setpoint tiers those metres select, the cellproof arm depth
    and the deco engine's ambient pressures, so a mid-dive write moves the diver's
    depth scale ~2.5% under his feet at a pressure that has not changed.

    Answering `ERR dive locked` is half the job. The other half is that the VALUE
    did not move — and that half is invisible to a matrix entry, which only asks
    whether the words arrived. This is the shape where an "ack (or refusal) with no
    consumer" defect hides: a router that prints the refusal and writes the
    registry anyway passes every reply-only check, on the bench and in the host
    suite, and the dive then runs on a depth scale nobody agreed to.

    So the matrix asks for a mid-range value while the dive is open and reads the
    ruler back TWICE — once DURING the dive, immediately after the refusal, and
    once after restoring it at the surface — pairing every command with its own
    reply. Only the mid-dive read catches a leak: the surface restore rewrites the
    value afterwards, so a final read alone would find a clean box over a dive that
    ran on the wrong scale. And no depth comparison can catch it either, because at
    the surface the whole legal density range is worth 4 mm of water.
    """
    asked = [p for p in pairs if p[0].startswith("SET WATER_DENSITY=")]
    gets = [(c, r) for c, r in pairs if c == "GET WATER_DENSITY"]
    refused_cmd = next(((c, r) for c, r in asked
                        if c.endswith(f"={DENSITY_REFUSED}")), None)
    surface = next(((c, r) for c, r in asked
                    if c.endswith(f"={DENSITY_RESTORE}")), None)
    mid = gets[0][1] if len(gets) >= 2 else None
    read = gets[-1][1] if gets else None
    if refused_cmd is None or mid is None or read is None:
        return [("the ruler leg ran (a refused write, a mid-dive read and a "
                 "post-dive read)", False,
                 f"writes seen: {[c for c, _ in asked]}; reads: {len(gets)} "
                 f"(want 2) — with any of them missing, 'did it land, and under "
                 f"what scale was this dive running?' cannot be answered and the "
                 f"leg is decoration")]

    def value_of(lines):
        # None-safe on purpose: a missing read is reported as its own failing
        # check below, never as a traceback out of a C2 run halfway through the
        # bench (a crash mid-run loses every check after it).
        if lines is None:
            return "(no read)", None
        line = next((x for x in lines if x.startswith("WATER_DENSITY=")), "")
        try:
            return line, float(line.split("=", 1)[1])
        except (ValueError, IndexError):
            return line, None

    got, shown = value_of(read)
    mgot, mval = value_of(mid)
    want = float(DENSITY_RESTORE)

    checks = [
        ("the mid-dive ruler write was refused",
         any(l.startswith("ERR dive locked") for l in refused_cmd[1]),
         f"`{refused_cmd[0]}` answered {refused_cmd[1] or ['(nothing)']}"),
        (f"the ruler did not move DURING the dive: the mid-dive read says "
         f"{DENSITY_RESTORE}, not {DENSITY_REFUSED}",
         mval is not None and abs(mval - want) < 1e-6,
         f"the read taken right after the refused write answers `{mgot}`. That is "
         f"the scale the dive was running on — the depth in `T,`, the setpoint "
         f"tiers, the cellproof arm depth and the deco plan all follow it — and "
         f"the post-dive read below cannot see this, because the matrix's own "
         f"surface restore rewrites the value afterwards"),
        (f"the refused write never landed at all: after the dive the box still "
         f"says {DENSITY_RESTORE}",
         shown is not None and abs(shown - want) < 1e-6,
         f"`GET WATER_DENSITY` answers `{got}` after the dive, where the matrix "
         f"restored {DENSITY_RESTORE}: a refusal that prints ERR and writes the "
         f"registry anyway is invisible to every reply-only check"),
        ("the ruler is writable at the surface (the lock is about the dive, not "
         "the key)",
         surface is not None and any(l.startswith("OK") for l in surface[1]),
         f"`SET WATER_DENSITY={DENSITY_RESTORE}` answered "
         f"{(surface[1] if surface is not None else []) or ['(never sent)']} — a "
         f"key that can never be written is not dive-locked, it is broken"),
    ]
    if mval is None:
        checks.append(("the mid-dive read-back parses", False, f"reply: {mid}"))
    if shown is None:
        checks.append(("the post-dive read-back parses", False, f"reply: {read}"))
    return checks


def grade_value_caps(pairs):
    """A value the parser cannot carry must be REFUSED, never quietly shortened.

    The command parser's value field was 23 characters while `AP_PASS` publishes
    8..31, so until 2026-11-24 a passphrase of 24..31 characters answered `OK` and
    stored something shorter than the client sent — the same "ack with no consumer"
    shape as the settings-applier defects, one layer further down, and invisible to
    the client that typed it. The host suite pins the parser and the schema against
    each other (`test_every_published_key_and_max_length_value_reaches_the_parser`,
    and a `static_assert` tying the buffer to `settings::MAX_VAL_STR`); this leg is
    the wire's copy of the same question, and it is deliberately built out of
    REFUSED writes: the passphrase never changes, so the access point is not
    re-armed and no later phase of the run depends on a station rejoining.

    Two reads around the write, paired with their own replies, because the refusal's
    words are only half the claim: the other half is that the value is exactly what
    it was.
    """
    reads = [(c, r) for c, r in pairs if c == "GET AP_PASS"]
    over = next(((c, r) for c, r in pairs
                 if c.startswith("SET AP_PASS=") and len(c) > len("SET AP_PASS=") + 31),
                None)
    # None-safety is deliberate: a missing read or write must come back as its own
    # failing check, never as a traceback that aborts the run and hides every check
    # after it (learned on this bench, where a crash mid-C2 loses the whole leg).
    if over is None or len(reads) < 2:
        return [("the value-cap leg ran (two passphrase reads around a refused write)",
                 False,
                 f"reads: {len(reads)} (want 2); over-long write: "
                 f"{'yes' if over is not None else 'no'} — with either missing, "
                 f"'did the refusal keep the value?' cannot be answered")]

    def value_of(lines):
        line = next((x for x in lines if x.startswith("AP_PASS=")), "")
        return line.split("=", 1)[1] if "=" in line else None

    before = value_of(reads[0][1] or []) if reads else None
    after = value_of(reads[-1][1] or []) if reads else None
    wrote = over[0] if over is not None else "(no over-long write was sent)"
    replied = (over[1] if over is not None else None) or []
    return [
        ("an over-long value is refused with the registry's own words",
         any(l.startswith("ERR value too long") for l in replied),
         f"`{wrote}` answered {replied or ['(nothing)']} — "
         f"note the distinction being graded: `ERR line too long` would be the LINE "
         f"cap (64 chars) answering, not the FIELD cap, and a check that accepts "
         f"either is testing the wrong rule"),
        ("the refused write left the passphrase exactly as it was",
         before is not None and after is not None and before == after,
         f"read before: {before!r}, read after: {after!r}. A parser that copies what "
         f"fits and answers OK changes the access point's key under a client that "
         f"typed a longer one — the client then cannot join, and the log says the "
         f"write succeeded"),
    ]


def selftest_value_caps():
    """The value-cap grader, proven on lying captures.

    `honest` is what the matrix performs: two passphrase readings around a write the
    parser refuses because it cannot carry the value. The lies are the shapes a
    regression takes: the value moved (the truncation defect itself), a refusal from
    a DIFFERENT rule (the line cap is not the field cap — the same
    "right ERR, wrong reason" trap that hid a dive-lock defect the same day), and a
    leg that never ran.
    """
    long_write = f"SET AP_PASS={'z' * 40}"
    honest = [
        ("GET AP_PASS", ["AP_PASS=diveonly"]),
        (long_write, ["ERR value too long"]),
        ("GET AP_PASS", ["T,0.04,0.214,0.214,0.214,0.214,0.400,STANDBY,FLAGS0x10,1.56",
                         "AP_PASS=diveonly"]),
    ]
    cases = [
        ("an honest leg: refused, and the key did not move", honest, None),
        ("the truncation defect: refused in words, stored anyway",
         [("GET AP_PASS", ["AP_PASS=diveonly"]),
          (long_write, ["ERR value too long"]),
          ("GET AP_PASS", ["AP_PASS=" + "z" * 23])],
         "left the passphrase"),
        ("silently accepted and shortened: no refusal at all",
         [("GET AP_PASS", ["AP_PASS=diveonly"]),
          (long_write, ["OK"]),
          ("GET AP_PASS", ["AP_PASS=" + "z" * 23])],
         "registry's own words"),
        ("the LINE cap refusing is not the FIELD cap refusing",
         [("GET AP_PASS", ["AP_PASS=diveonly"]),
          (long_write, ["ERR line too long"]),
          ("GET AP_PASS", ["AP_PASS=diveonly"])],
         "registry's own words"),
        ("only one read: 'did it move?' is unanswerable",
         [(long_write, ["ERR value too long"]),
          ("GET AP_PASS", ["AP_PASS=diveonly"])],
         "value-cap leg ran"),
        ("no over-long write: the matrix entry went missing",
         [("GET AP_PASS", ["AP_PASS=diveonly"]),
          ("GET AP_PASS", ["AP_PASS=diveonly"])],
         "value-cap leg ran"),
        ("a read that answered no value at all (ERR instead of a passphrase)",
         [("GET AP_PASS", ["ERR unknown key"]),
          (long_write, ["ERR value too long"]),
          ("GET AP_PASS", ["AP_PASS=diveonly"])],
         "left the passphrase"),
        ("neither read answered a value: comparing two nothings proves nothing",
         [("GET AP_PASS", ["ERR unknown key"]),
          (long_write, ["ERR value too long"]),
          ("GET AP_PASS", ["ERR unknown key"])],
         "left the passphrase"),
    ]
    checks = []
    for name, pairs, want in cases:
        got = grade_value_caps(pairs)
        failed = [c[0] for c in got if not c[1]]
        if want is None:
            checks.append((f"value caps: {name}", len(got) == 2 and not failed,
                           f"expected two graded checks, none failed: {failed}"))
        else:
            checks.append((f"value caps: {name}", any(want in f for f in failed),
                           f"no failure matching {want!r}; failures seen: {failed}"))
    return checks


def grade_boot(lines, expect_web, expect_depth=False):
    """Return list of (check_name, passed, detail)."""
    checks = []
    # soak_logger's raw captures prefix lines with [HH:MM:SS]; the console
    # itself does not. Normalize so replay grades the same as capture.
    lines = [re.sub(r"^\[\d\d:\d\d:\d\d\]\s*", "", l) for l in lines]
    banner = next((l for l in lines if BOOT_BANNER.search(l)), None)
    checks.append(("boot banner in capture", banner is not None,
                   banner or "no boot banner — was the board reset/flash logged?"))

    hb = [l for l in lines if HB_RE.match(l)]
    checks.append(("HB status lines >= 2", len(hb) >= 2,
                   f"{len(hb)} lines matched the soak_logger contract"))

    bad = []
    for pat, why in MUST_NONE:
        for l in lines:
            if pat.search(l):
                bad.append(f"{why}: {l.strip()}")
    checks.append(("no SAFE/FATAL/session failures", not bad, "; ".join(bad)))

    if banner:
        rl = next((l for l in lines if RESET_REASON.search(l)), None)
        checks.append(("reset reason line", rl is not None, rl or ""))

    if expect_web:
        sl = next((l for l in lines if SETTINGS_LINE.search(l)), None)
        if sl is None and any("nvs_open failed: NOT_FOUND" in l for l in lines):
            # Fresh board: NVS namespace does not exist yet, firmware boots on
            # compile-time defaults (correct behaviour). The NVS round-trip is
            # proven by the next acceptance run: after any SET + restart this
            # line MUST appear.
            checks.append(("settings: fresh board (no NVS yet)", True,
                           "defaults in force; recheck after first SET + reboot"))
        else:
            checks.append(("settings loaded from NVS", sl is not None, sl or ""))
        cl = next((l for l in lines if CONTROL_LINE.search(l)), None)
        checks.append(("control chain booted", cl is not None, cl or ""))
        wl = next((l for l in lines if WEB_AP_LINE.search(l)), None)
        checks.append(("web AP announced", wl is not None, wl or ""))
    ll = next((l for l in lines if LOG_LINE.search(l)), None)
    checks.append(("dive log mounted at boot", ll is not None, ll or ""))
    # The surface log states itself at boot, in all three states: records with a
    # seq range, "no records yet", or a warning that it exists but will not read.
    # A missing line is a failure because the absence is indistinguishable from an
    # empty log — which is the exact confusion this line exists to prevent.
    sl = next((l for l in lines if SURFACE_LINE.search(l)), None)
    checks.append(("surface log stated at boot", sl is not None,
                   sl or "no `surface:` line: an unreadable surface log would look "
                         "like an empty one"))
    if sl:
        checks.append(("surface log is readable", not SURFACE_LINE.search(sl).group("bad"), sl))
    if ll:
        # The suffix is the firmware's own warning. A LittleFS mount that fails
        # is answered with a format, which erases every dive on the partition;
        # until 2026-11-23 that happened in silence and was indistinguishable
        # from a diver who had simply not dived.
        note = LOG_LINE.search(ll).group("note").strip()
        checks.append(("dive log mount did not format (history intact)", not note, ll))
    checks.extend(grade_depth(lines, expect_depth))
    cl = next((l for l in lines if CLOCK_LINE.search(l)), None)
    checks.append(("wall clock stated at boot", cl is not None,
                   cl or "no `clock:` line: the boot does not say whether dives "
                         "will carry a date"))
    return checks


def fetch_surface_csv(base, timeout=12.0):
    """GET /surface.csv. urllib only: the bench tools must not need anything the
    pipx PlatformIO python does not already have.

    This is the log's ONLY export path — there are no handset commands for it — so
    it is also the only way the gate can see a surface record after the moment it
    happened."""
    import urllib.request

    req = urllib.request.Request(base.rstrip("/") + "/surface.csv")
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return r.status, r.headers.get("Content-Type", ""), r.read().decode("utf-8", "replace")


def parse_surface_csv(text):
    """(identity row fields, [row fields]). `R,` needs all 13 columns: a short row
    is a truncated file, and grading it as a row would hide that."""
    head, rows = None, []
    for line in text.splitlines():
        f = line.split(",")
        if f[0] == "SURFACE" and len(f) > 4:
            head = f
        elif f[0] == "R" and len(f) > 12:
            rows.append(f)
    return head, rows


# `R,` columns by name, in the order the firmware writes them. Everything in this
# tool reads the export through this table — the grader's own checks and the
# expectations each phase builds — because a column index typed twice is two
# chances to be wrong, and a wrong index makes a real log read as a lying one (or
# the reverse). Order is append-only: new columns go on the right.
SURF_COL = {"seq": 1, "epoch": 2, "iso": 3, "id": 4, "name": 5, "value": 6,
            "raw0": 7, "raw1": 8, "raw2": 9, "spread": 10, "samples": 11,
            "flags": 12}


def grade_surface_csv(text, want_rows, boot_hi, console_lines):
    """Cross-checks the exported surface log against what the machine said.

    `want_rows`: what the run just did, as (name, {field: expected}) — the raw
    counts and spread of a calibration must match, field for field, the `CALAIR,`
    line the handset was answered with. `console_lines`: the console's own
    `surface: record <seq> ...` lines, whose seq is the join key.

    Four records of one event — the console line, the record on flash, the CSV row
    and the protocol answer — and the check is that they are the same event."""
    checks = []
    head, rows = parse_surface_csv(text)
    checks.append(("surface.csv carries its identity row", head is not None,
                   text.splitlines()[0] if text.strip() else "empty response"))
    if head is None:
        return checks
    # id <-> name must agree, both ways: a pre-dive failure filed under the pass id
    # (or an appended id left unlabelled) is the log claiming something the machine
    # never recorded. The ids below are the whole vocabulary so far — 17/18 joined
    # with H1's dive continuity; anything else must arrive as `id<n>`, never as a
    # row with a name it did not earn.
    KNOWN = {"7": "cal_air", "13": "pre_dive_pass", "16": "pre_dive_fail",
             "17": "continuity_restored", "18": "continuity_lost"}
    mismatch = [r for r in rows
                if KNOWN.get(r[SURF_COL["id"]]) and r[SURF_COL["name"]] != KNOWN[r[SURF_COL["id"]]]]
    checks.append(("id and name agree", not mismatch,
                   str(mismatch[0]) if mismatch else "every known id carries its name"))
    unnamed = [r for r in rows
               if r[SURF_COL["id"]] not in KNOWN
               and r[SURF_COL["name"]] != f"id{r[SURF_COL['id']]}"]
    checks.append(("unknown ids are named, not hidden", not unnamed,
                   str(unnamed[0]) if unnamed else "none needed"))
    # A pass has no step to name. If one does, the row is a failure wearing a
    # pass's label — the one mutation an id/name table cannot catch, because the
    # row is internally consistent.
    bogus = [r for r in rows
             if r[SURF_COL["name"]] == "pre_dive_pass" and r[SURF_COL["value"]] != "0"]
    checks.append(("a pass names no failing step", not bogus,
                   str(bogus[0]) if bogus else "no pass carries a step"))
    undated = [r for r in rows
               if r[SURF_COL["epoch"]] == "0" and r[SURF_COL["iso"]] != "clock-unset"]
    checks.append(("a record with no clock says so", not undated,
                   str(undated[0]) if undated else "epoch 0 renders as clock-unset"))
    n, lo, hi = int(head[2]), int(head[3]), int(head[4])
    checks.append(("identity row's count matches the rows delivered", n == len(rows),
                   f"SURFACE says {n}, {len(rows)} rows arrived: a truncated file "
                   f"must not read as a complete log"))
    checks.append(("seq range spans the rows", bool(rows) and lo <= int(rows[0][SURF_COL["seq"]]) and
                   hi >= int(rows[-1][SURF_COL["seq"]]),
                   f"header {lo}..{hi}, rows {rows[0][SURF_COL['seq']]}..{rows[-1][SURF_COL['seq']]}"
                   if rows else "no rows"))
    if boot_hi is not None:
        checks.append(("this run's records joined the log", hi > boot_hi,
                       f"boot said seq up to {boot_hi}, the log now ends at {hi}: "
                       f"the verdict was never recorded"))

    for name, want in want_rows:
        got = next((r for r in reversed(rows) if r[SURF_COL["name"]] == name), None)
        checks.append((f"a {name} record is in the export", got is not None,
                       f"names seen: {[r[SURF_COL['name']] for r in rows]}"
                       if rows else "no rows"))
        if not got:
            continue
        bad = [f"{k}={got[v]} want {w}" for k, v, w in want
               if got[v] != str(w)]
        checks.append((f"the {name} record carries the same numbers as the wire",
                       not bad, "; ".join(bad) or str(got)))
        # The console's own line, joined on seq: the record the machine says it
        # wrote IS the row that came back over HTTP.
        recs = [SURFACE_RECORD.match(re.sub(r"^\[\d\d:\d\d:\d\d\]\s*", "", l))
                for l in console_lines]
        recs = [m for m in recs if m]
        joined = next((m for m in recs
                       if int(m.group("seq")) == int(got[SURF_COL["seq"]])), None)
        checks.append((f"the console reported writing record {got[1]}", joined is not None,
                       f"console records: {[m.group(0) for m in recs] or 'none'}"))
    return checks


GOOD_SURFACE_CSV = (
    "SURFACE,1,2,3,4\n"
    "R,3,1795518150,2026-11-24T11:02:30Z,7,cal_air,2,1514,1519,1528,21,12,0\n"
    "R,4,1795518200,2026-11-24T11:03:20Z,16,pre_dive_fail,4,0,0,0,0,0,0\n")
GOOD_CONSOLE = ["surface: record 3 id 7 value 2", "surface: record 4 id 16"]
CAL_WANT = [("value", SURF_COL["value"], 2), ("samples", SURF_COL["samples"], 12),
            ("spread", SURF_COL["spread"], 21), ("raw0", SURF_COL["raw0"], 1514),
            ("raw1", SURF_COL["raw1"], 1519), ("raw2", SURF_COL["raw2"], 1528)]


# Boot fragments for the depth-announce grader's own selftest, built from the
# two real captures of 2026-11-24 (the broken announce, then the fixed one).
DEPTH_BOOT_OK = [
    "=== ESP-CCR controller M1 boot ===",
    "depth: LPS28DFW @0x5D ready, 1016.7 hPa",
    "HB st=0 c0=1515 c1=1518 c2=1524 fresh=111 smp=30 i2cerr=0 rec=0 vbat=1.56 "
    "stale=0 batlow=1 al=0x0010",
]
DEPTH_HB_STALE = ("HB st=0 c0=1515 c1=1518 c2=1524 fresh=111 smp=30 i2cerr=0 rec=0 "
                  "vbat=1.56 stale=1 batlow=1 al=0x0090")


def selftest_depth():
    """Proves the depth-announce graders can fail, no board needed.

    Same rule as the surface-log cases below: every case is a boot that lies in
    exactly one known way, and a case that does NOT fail means the check is
    decoration. The first case here is the real defect this whole pass exists
    for — the announce fired the instant the chip was identified, so every
    healthy boot printed `ready, 0.0 hPa` and the line the bench reads as
    evidence could never carry a measurement.
    """
    nodata = ("depth: ** LPS28DFW @0x5D identified, no sane pressure within 3 s "
              "(reads 0.0 hPa) - depth will not be used")
    cases = [
        # name, boot lines, --expect-depth, expected failing check (None = clean)
        ("an honest boot with the sensor fitted", DEPTH_BOOT_OK, True, None),
        ("the same honest boot, sensor not declared", DEPTH_BOOT_OK, False, None),
        ("placeholder pressure — the announce printed before the first "
         "conversion",
         [l.replace("1016.7 hPa", "0.0 hPa") for l in DEPTH_BOOT_OK], True,
         "carries a reading, not a placeholder"),
        ("a chip the identity gates would refuse owns the depth slot",
         [l.replace("LPS28DFW @0x5D", "LPS22HB @0x5C") for l in DEPTH_BOOT_OK],
         True, "identity gates accept"),
        ("the announce says no data but the control chain believes the sensor",
         [nodata, DEPTH_BOOT_OK[2]], True, "raises the alarm"),
        ("the announce says ready while the control chain calls the sensor stale",
         [DEPTH_BOOT_OK[0], DEPTH_BOOT_OK[1], DEPTH_HB_STALE], True,
         "does not report the depth sensor stale"),
        ("a sensor is claimed to be fitted and nothing announced",
         DEPTH_BOOT_OK[:1], True, "depth source announced at boot"),
        ("nothing fitted, nothing announced: honest", DEPTH_BOOT_OK[:1], False,
         None),
    ]
    checks = []
    for name, lines, expect, want in cases:
        failed = [c[0] for c in grade_depth(lines, expect) if not c[1]]
        if want is None:
            checks.append((f"depth: {name}", not failed, f"unexpected failures: {failed}"))
        else:
            checks.append((f"depth: {name}", any(want in f for f in failed),
                           f"no failure matching {want!r}; failures seen: {failed}"))
    # The no-data form WITH the alarm it implies is the other honest shape.
    honest_nodata = [DEPTH_BOOT_OK[0], nodata, DEPTH_HB_STALE]
    failed = [c[0] for c in grade_depth(honest_nodata, True) if not c[1]]
    checks.append(("depth: identified-but-deaf, with the alarm raised, is honest",
                   not failed, f"unexpected failures: {failed}"))
    return checks


# Dive exports for the dive-file grader's own selftest. Every row below is a
# literal captured off the handset link, not typed from the grammar:
#   DIVE_HONEST      slot 10 / seq 60, exported 2026-11-24 off the build that
#                    writes event rows — note the interleaving, the verdict row
#                    then the snapshot that shows the state it was decided in;
#   DIVE_PRE_FIX     slot 8 / seq 58, the clamp dive recorded BEFORE the fix:
#                    the wire streamed ids 10/11/12 (see WIRE_PRE_FIX) and the
#                    file holds none of them, while `M,` also claims 0 snapshots
#                    beside the 13 samples a few lines above it.
DIVE_HONEST = [
    "DIVE,10,1795540808,3,3,2026-11-24T17:20:08Z",
    "E,0,1,4",
    "S,0,0.04,0.214,0.213,0.214,0.214,0.400,0x0010,0,0,0,0,0,1562,2,0,0,0",
    "E,0,6,0",
    "S,0,0.04,0.214,0.213,0.214,0.214,0.400,0x0010,0,0,0,0,0,1562,2,0,0,0",
    "E,0,6,1",
    "S,0,0.04,0.214,0.213,0.214,0.214,0.400,0x0010,0,0,0,0,0,1562,2,0,0,0",
    "E,0,6,2",
    "S,0,0.04,0.214,0.213,0.214,0.214,0.400,0x0010,0,0,0,0,0,1562,2,0,0,0",
    "M,13,4,0.213,0.214,0,0,4,4,30,85,0.400,0.550,1.000,1.300",
]
DIVE_HONEST_WIRE = ["E,0,1,4", "E,0,6,0", "E,0,6,1", "E,0,6,2"]

DIVE_PRE_FIX = [
    "DIVE,8,1795538524,3,3,2026-11-24T16:42:04Z",
] + [
    "S,0,0.03,0.213,0.212,0.213,0.215,0.400,0x0010,0,0,0,0,0,1562,2,0,0,0",
    "S,0,0.03,0.213,0.212,0.213,0.215,0.400,0x0010,0,0,0,0,0,1562,2,0,0,0",
    "S,0,0.03,0.213,0.212,0.213,0.215,0.400,0x0010,0,0,0,0,0,1562,2,0,0,0",
    "S,0,0.03,0.213,0.212,0.213,0.215,0.400,0x0010,0,0,0,0,0,1562,2,0,0,0",
    "S,30,0.03,0.213,0.213,0.213,0.215,0.400,0x0010,0,0,480,0,0,1622,2,0,0,0",
    "S,34,3.32,0.213,0.213,0.213,0.215,1.100,0x0010,0,0,0,0,0,1623,2,0,0,0",
    "S,54,6.52,0.213,0.212,0.213,0.215,1.100,0x0030,0,0,0,0,0,1628,2,0,0,0",
    "S,54,6.52,0.213,0.212,0.213,0.215,1.100,0x0030,0,0,0,0,0,1628,2,0,0,0",
    "S,54,6.52,0.213,0.212,0.213,0.215,1.100,0x0030,0,0,0,0,0,1628,2,0,0,0",
    "S,60,6.44,0.213,0.213,0.213,0.215,0.900,0x0030,0,0,480,6,0,1594,2,0,0,0",
    "S,66,6.30,0.213,0.213,0.213,0.215,1.000,0x0010,0,0,0,0,0,1624,2,0,0,0",
    "S,66,6.30,0.213,0.213,0.213,0.215,1.000,0x0010,0,0,0,0,0,1624,2,0,0,0",
    "S,66,6.30,0.213,0.213,0.213,0.215,1.000,0x0010,0,0,0,0,0,1624,2,0,0,0",
    "M,66,743,0.212,0.215,0,0,0,0,0,0,0.000,0.000,0.000,0.000",
]
# Same dive, as the handset saw it live (clamp_leg.log, 2026-11-24): the proof
# FAIL, the clamp, and the diver's override were all on the wire.
WIRE_PRE_FIX = ["E,0,1,4", "E,0,6,0", "E,0,6,1", "E,0,6,2", "E,34,4,110",
                "E,54,1,5", "E,54,10,0", "E,54,11,0", "E,66,12,1"]

# What the same link answered for the plan in force (GET GF_LOW/GF_HIGH/SP1..4).
DIVE_WANT = {"GF_LOW": "GF_LOW=30", "GF_HIGH": "GF_HIGH=85",
             "SP1": "SP1=0.400", "SP2": "SP2=0.550", "SP3": "SP3=1.000",
             "SP4": "SP4=1.300"}

def selftest_scale():
    """The announce-vs-`T,`-row grader, proven on lying captures.

    Both rows are real: `T,0.04,…` is what this bench streams at the surface
    with the LPS28DFW fitted, and `T,37.04,…` is what the same bench streamed on
    2026-11-24 with the 1000x scale error in `BAR_PER_M_SEAWATER` — same room,
    same sensor, same announce. The altitude pair is synthetic in one way only
    (a sensor reading 1000.0 hPa is a mountain or a weather front, not this
    bench): it pins the rule that the depth a diver reads never goes negative.
    """
    t_ok = "T,0.04,0.214,0.214,0.214,0.214,0.400,STANDBY,FLAGS0x10,1.56"
    t_broken = "T,37.04,0.213,0.213,0.213,0.215,1.000,STANDBY,FLAGS0x10,1.56"
    high_boot = [l.replace("1016.7 hPa", "1000.0 hPa") for l in DEPTH_BOOT_OK]
    cases = [
        # name, console lines, handset stream, expected failing check (None = clean)
        ("an honest surface: announce and `T,` agree", DEPTH_BOOT_OK, [t_ok], None),
        ("the 1000x scale error: 1016.7 hPa streamed as 37.04 m", DEPTH_BOOT_OK,
         [t_broken], "is the announce's pressure"),
        ("no telemetry row to compare against", DEPTH_BOOT_OK, [],
         "a `T,` row exists"),
        ("an unparseable depth column", DEPTH_BOOT_OK, ["T,x.y,0.214"],
         "depth column parses"),
        ("above the reference plane the diver reads 0 m, not a negative depth",
         high_boot, ["T,0.00,0.214,0.214,0.214,0.214,0.400,STANDBY,FLAGS0x10,1.56"],
         None),
        ("a negative depth on the wire is a lie about the same sensor",
         high_boot, ["T,-0.13,0.214,0.214,0.214,0.214,0.400,STANDBY,FLAGS0x10,1.56"],
         "is the announce's pressure"),
        # Deeper, and synthetic in one way only (this bench cannot hold 1300 hPa):
        # the density only becomes visible once the depth is metres rather than
        # centimetres, which is why the surface comparison alone cannot guard the
        # density. 1300.0 hPa is 2.92 m at 1.000 and 2.86 m at 1.025 — 6 cm apart,
        # just over the 5 cm tolerance, so a witness that ignores the link's own
        # density is caught here and nowhere else.
        ("deeper, on fresh water: the witness must use the density the link says",
         [l.replace("1016.7 hPa", "1300.0 hPa") for l in DEPTH_BOOT_OK],
         ["T,2.92,0.214,0.214,0.214,0.214,0.700,STANDBY,FLAGS0x10,1.56",
          "WATER_DENSITY=1.000"], None),
        ("deeper, and the two really do disagree: 2.86 m is a sea-water depth",
         [l.replace("1016.7 hPa", "1300.0 hPa") for l in DEPTH_BOOT_OK],
         ["T,2.86,0.214,0.214,0.214,0.214,0.700,STANDBY,FLAGS0x10,1.56",
          "WATER_DENSITY=1.000"], "is the announce's pressure"),
    ]
    checks = []
    for name, lines, stream, want in cases:
        got = grade_depth_scale(lines, stream)
        failed = [c[0] for c in got if not c[1]]
        if want is None:
            why = "; ".join(c[2] for c in got if c[1])
            checks.append((f"depth scale: {name}", bool(got) and not failed,
                           f"expected graded checks, none failed. What the witness "
                           f"computed: {why}"))
        else:
            checks.append((f"depth scale: {name}", any(want in f for f in failed),
                           f"no failure matching {want!r}; failures seen: {failed}"))
    # A sensor-less boot has nothing to reconcile: silence is the honest answer,
    # and it must not become a failure (that would make --expect-depth wrong on
    # a bench with no depth sensor fitted).
    checks.append(("depth scale: no announce, nothing claimed",
                   grade_depth_scale(DEPTH_BOOT_OK[:1], [t_ok]) == [],
                   "a bench with no depth sensor must not be graded on one"))
    nodata = ["depth: ** LPS28DFW @0x5D identified, no sane pressure within 3 s "
              "(reads 0.0 hPa) - depth will not be used"]
    checks.append(("depth scale: an announce with no reading cannot be reconciled",
                   grade_depth_scale(nodata, [t_ok]) == [],
                   "comparing a placeholder hPa against a depth would be inventing one"))
    return checks


def selftest_density_lock():
    """The ruler-lock grader, proven on lying captures.

    `honest` is the exchange the matrix performs: a dive-locked write refused with
    the right words, the ruler read back DURING the dive, the value restored at the
    surface, and a final read agreeing with the restore. The lies are the shapes a
    regression takes: the ruler moved while the dive ran, the box ended on the
    refused value, the lock missing, an `ERR` for an unrelated reason, and a leg
    that never ran. Every value comes from the same constants the matrix writes, so
    changing the probe cannot silently desync the fixtures.
    """
    refused_cmd = f"SET WATER_DENSITY={DENSITY_REFUSED}"
    restore_cmd = f"SET WATER_DENSITY={DENSITY_RESTORE}"
    read_ok = ("GET WATER_DENSITY", [f"WATER_DENSITY={DENSITY_RESTORE}"])
    honest = [
        (refused_cmd, ["ERR dive locked"]),
        read_ok,
        (restore_cmd, ["OK"]),
        read_ok,
    ]
    cases = [
        # name, pairs, expected failing check (None = must pass clean)
        ("an honest dive-locked ruler: refused, read mid-dive, restored, read again",
         honest, None),
        (f"the ruler moved WHILE the dive ran: the mid-dive read says "
         f"{DENSITY_REFUSED}",
         [(refused_cmd, ["ERR dive locked"]),
          ("GET WATER_DENSITY", [f"WATER_DENSITY={DENSITY_REFUSED}"]),
          (restore_cmd, ["OK"]),
          read_ok],
         "DURING the dive"),
        (f"unchanged mid-dive, but the box ends on {DENSITY_REFUSED}",
         [(refused_cmd, ["ERR dive locked"]),
          read_ok,
          (restore_cmd, ["OK"]),
          ("GET WATER_DENSITY", [f"WATER_DENSITY={DENSITY_REFUSED}"])],
         "never landed at all"),
        ("the lock is gone: the mid-dive write answers OK and lands",
         [(refused_cmd, ["OK"]),
          ("GET WATER_DENSITY", [f"WATER_DENSITY={DENSITY_REFUSED}"]),
          (restore_cmd, ["OK"]),
          read_ok],
         "was refused"),
        ("an ERR for some unrelated reason is not a dive lock",
         [(refused_cmd, ["ERR out of range"]),
          read_ok,
          (restore_cmd, ["OK"]),
          read_ok],
         "was refused"),
        ("the ruler can never be written: that is a broken key, not a locked one",
         [(refused_cmd, ["ERR dive locked"]),
          read_ok,
          (restore_cmd, ["ERR dive locked"]),
          read_ok],
         "writable at the surface"),
        ("only one read: the mid-dive question cannot be asked at all",
         [(refused_cmd, ["ERR dive locked"]),
          (restore_cmd, ["OK"]),
          read_ok],
         "ruler leg ran"),
        ("no mid-dive write at all: the matrix entry went missing",
         [read_ok, (restore_cmd, ["OK"]), read_ok],
         "ruler leg ran"),
        ("an unparseable mid-dive read-back",
         [(refused_cmd, ["ERR dive locked"]),
          ("GET WATER_DENSITY", ["WATER_DENSITY=oops"]),
          (restore_cmd, ["OK"]),
          read_ok],
         "mid-dive read-back parses"),
        ("an unparseable post-dive read-back",
         [(refused_cmd, ["ERR dive locked"]),
          read_ok,
          (restore_cmd, ["OK"]),
          ("GET WATER_DENSITY", ["WATER_DENSITY=oops"])],
         "post-dive read-back parses"),
    ]
    checks = []
    for name, pairs, want in cases:
        got = grade_density_lock(pairs)
        failed = [c[0] for c in got if not c[1]]
        if want is None:
            checks.append((f"dive lock: {name}", len(got) == 4 and not failed,
                           f"expected four graded checks, none failed: {failed}"))
        else:
            checks.append((f"dive lock: {name}", any(want in f for f in failed),
                           f"no failure matching {want!r}; failures seen: {failed}"))
    return checks


DS_LINE = re.compile(r"^DS,(\d),(\d),(\d),(\d+),(.*)$")


def grade_depth_source(lines, pairs, expect_depth):
    """`GET DS` on the wire, against what the console said at boot.

    Two transports, one fact. The console announce (`depth: LPS28DFW @0x5D ready,
    1016.7 hPa`) is the boot decision, printed once, before any client is
    listening; `DS,` is that decision plus its live state, asked seconds later. A
    build that published the arbiter's choice into one and something else into the
    other — the second driver fitted, a stale name, a snapshot that never got
    filled in — is caught by the two not agreeing, which no single-transport check
    can see.

    `used` is graded too, and it is the field a diver's safety depends on: a locked,
    identified, healthy sensor that has stopped producing readings leaves the depth
    on `T,` HELD, and the `T,` row looks identical either way.
    """
    checks = []
    reply = next((r for c, r in pairs if c == "GET DS"), None)
    if not reply:
        return [("the depth-source leg ran (`GET DS` answered)", False,
                 "no reply to `GET DS` — an older controller answers `ERR unknown key`, "
                 "which means 'this controller cannot say', not 'there is no sensor'")]

    line = next((l for l in reply if DS_LINE.match(l)), None)
    if line is None:
        return [("`GET DS` answered a well-formed `DS,` line", False,
                 f"`GET DS` answered {reply} — want "
                 f"`DS,<locked>,<used>,<healthy>,<age_s>,<identity>`")]
    m = DS_LINE.match(line)
    locked, used, healthy = (m.group(1) == "1", m.group(2) == "1", m.group(3) == "1")
    age, who_addr = int(m.group(4)), m.group(5).strip()

    ready = next((DEPTH_READY.search(l) for l in lines if DEPTH_READY.search(l)), None)
    nodata = next((DEPTH_NODATA.search(l) for l in lines if DEPTH_NODATA.search(l)), None)
    announce = None
    if ready or nodata:
        g = (ready or nodata).group
        announce = f"{g('who').strip()} @{g('addr')}"

    checks.append(("`GET DS` names a source, or says none",
                   who_addr != "" and "," not in who_addr,
                   f"identity field is {who_addr!r} — the field is free text and must "
                   f"never contain a comma, because it is the last field on the line"))

    if announce is None:
        # Nothing claimed depth at boot. Then the wire must not either, and the
        # honest answer is the whole-zero one with the name "none".
        checks.append(("no depth sensor at boot: the wire says none too",
                       not locked and who_addr == "none",
                       f"announce said nothing, `DS,` says {locked=}, {who_addr!r} — a "
                       f"name with no announce behind it is a sensor nobody identified"))
        if expect_depth and not locked:
            checks.append(("a bench asked for depth: something must own the slot",
                           False, "--expect-depth was given, but no source owns depth "
                                  "and no announce named one either"))
        return checks

    # The cross-transport claim, and the one this leg exists for.
    checks.append(("the wire names the chip the console named at boot",
                   who_addr == announce,
                   f"console announced {announce!r}, `GET DS` answers {who_addr!r} — "
                   f"one of these is not the sensor the depth is coming from"))
    name, _, addr = who_addr.rpartition("@")
    checks.append(("and that chip is one the identity gates accept",
                   addr in DEPTH_IDENTS.get(name.strip(), set()),
                   f"{who_addr!r}, accepted {DEPTH_IDENTS} — a near-relative part "
                   f"(an LPS22HB, say) must never be handed the depth slot"))
    checks.append(("a source owns the depth slot", locked,
                   f"`{line}` — the console announced {announce!r} at boot, so the "
                   f"slot is awarded until the next power cycle"))
    checks.append(("the control path is USING it (the depth on `T,` is measured, not held)",
                   used,
                   f"`{line}` — locked but not used means the controller has stopped "
                   f"taking depth from a sensor that still holds the slot, and every "
                   f"`T,` row still prints a plausible depth. This is the field a "
                   f"handset must look at, and the reason `GET DS` exists"))
    if used:
        checks.append(("a source in use has read recently", age <= 5,
                       f"`{line}` — claimed in use, last good read {age} s ago. The "
                       f"firmware's own staleness gate is far tighter than this, so "
                       f"the two fields contradict each other"))
    checks.append(("and its driver reports clean transactions", healthy,
                   f"`{line}` — identified at boot, and by now its transaction stream "
                   f"is in backoff"))
    return checks


def selftest_depth_source():
    """The depth-source grader, proven on lying captures.

    The honest capture is this bench: an LPS28DFW on the XSA-strapped breakout
    (hence 0x5D, not the default 0x5C), identified at boot, reading. The lies are
    the ways the two transports come apart: the wire naming a different chip than
    the console, a near-relative part getting the slot, a sensor that holds the
    slot while nobody is reading it, an answer that contradicts itself, and a
    leg that never ran at all.
    """
    honest_pairs = [("GET DS", ["DS,1,1,1,0,LPS28DFW @0x5D"])]

    def pairs_with(ds):
        return [("GET DS", [ds])]

    cases = [
        # name, boot lines, pairs, expected failing check (None = clean)
        ("an honest bench: console and wire name the same chip",
         DEPTH_BOOT_OK, honest_pairs, None),
        ("the wire names a different address than the console probed",
         DEPTH_BOOT_OK, pairs_with("DS,1,1,1,0,LPS28DFW @0x5C"),
         "names the chip the console named"),
        ("a near-relative part is handed the depth slot",
         DEPTH_BOOT_OK, pairs_with("DS,1,1,1,0,LPS22HB @0x5C"),
         "identity gates accept"),
        ("the snapshot was never filled in: locked, and no name",
         DEPTH_BOOT_OK, pairs_with("DS,1,0,0,0,none"), "names the chip the console named"),
        ("locked and identified but NOT used: the depth on `T,` is held",
         DEPTH_BOOT_OK, pairs_with("DS,1,0,1,12,LPS28DFW @0x5D"),
         "is USING it"),
        ("an answer that contradicts itself: in use, silent for 15 minutes",
         DEPTH_BOOT_OK, pairs_with("DS,1,1,1,900,LPS28DFW @0x5D"),
         "read recently"),
        ("no source owns depth, though the console named one",
         DEPTH_BOOT_OK, pairs_with("DS,0,0,0,0,none"), "a source owns the depth slot"),
        ("a truncated line is not an answer",
         DEPTH_BOOT_OK, pairs_with("DS,1,1,1"), "well-formed"),
        ("an identity field carrying a comma would shift every field after it",
         # The field-count check catches it first, which is the honest result: a
         # comma in the last field is caught as a malformed line rather than as a
         # comma inside a name, and either way the answer is refused.
         DEPTH_BOOT_OK, pairs_with("DS,1,1,1,0,LPS28DFW @0x5D,x"),
         "names a source, or says none"),
        ("older firmware that has no such command (`ERR unknown key`)",
         DEPTH_BOOT_OK, [("GET DS", ["ERR unknown key"])], "answered a well-formed"),
    ]
    checks = []
    for name, boot, pr, want in cases:
        got = grade_depth_source(boot, pr, True)
        failed = [c[0] for c in got if not c[1]]
        if want is None:
            why = "; ".join(c[0] for c in got if c[1])
            checks.append((f"depth source: {name}", bool(got) and not failed,
                           f"expected no failures, saw: {failed}. Graded: {why}"))
        else:
            checks.append((f"depth source: {name}", any(want in f for f in failed),
                           f"no failure matching {want!r}; failures seen: {failed}"))

    # The leg never ran: no reply at all. A grader that returns [] here would make
    # "not tested" indistinguishable from "passed".
    checks.append(("depth source: `GET DS` got no reply at all",
                   any(not ok_ for _, ok_, _ in grade_depth_source(DEPTH_BOOT_OK, [], True)),
                   "a missing leg must report itself as a failure"))

    # A bench with no depth sensor fitted: silence is the honest answer, on both
    # transports, and must not be graded as a fault.
    no_sensor = [("GET DS", ["DS,0,0,0,0,none"])]
    checks.append(("depth source: no sensor fitted, both transports say none",
                   all(ok_ for _, ok_, _ in grade_depth_source(DEPTH_BOOT_OK[:1], no_sensor, False)),
                   str(grade_depth_source(DEPTH_BOOT_OK[:1], no_sensor, False))))
    # ... but a name on the wire with nothing behind it is a fabricated source.
    checks.append(("depth source: a name with no announce behind it",
                   any(not ok_ for _, ok_, _ in grade_depth_source(
                       DEPTH_BOOT_OK[:1], pairs_with("DS,1,1,1,0,LPS28DFW @0x5D"), False)),
                   "an identity nobody announced at boot was accepted"))
    return checks

WIFI_LINE = re.compile(r"^WIFI,(\d),(\d+),(\d+)(?:,(\d+))?$")


def grade_wifi(pairs, boot_lines):
    """`GET WIFI` against the settings and the boot console — one machine, two views.

    The line has three fields and they are NOT independent: the countdown must
    agree with the WIFI_IDLE_MIN that the SAME session read from `GET *`, an up
    radio has no hold, a down radio has no countdown, and a dive in progress is
    blamed on the dive. Each cross-field rule below is a way the three fields
    could tell three different stories; the lying captures in selftest_wifi()
    prove each rule fires.
    """
    checks = []
    getstar = next((r for c, r in pairs if c == "GET *"), None)
    idle_min = None
    if getstar:
        row = next((l for l in getstar if l.startswith("WIFI_IDLE_MIN=")), None)
        if row:
            try:
                idle_min = int(float(row.split("=", 1)[1]))
            except ValueError:
                pass
    # Scan the pair for the WIFI, line rather than trusting pair[0]: a matrix
    # window can open on a leaked tail from the PREVIOUS command's answer, and
    # a grader that reads line one then parses nothing has found a harness bug,
    # not a firmware lie. (Found on the bench 2026-11-26, when GET * grew and
    # its tail started landing in the next window.)
    replies = [next((l for l in r if WIFI_LINE.match(l)), None)
               for c, r in pairs if c == "GET WIFI"]
    replies = [l for l in replies if l]
    checks.append(("the radio leg ran twice (boot and mid-dive)", len(replies) >= 2,
                   f"{len(replies)} `GET WIFI` answers — want the boot shape AND "
                   f"the mid-dive one, or the reason column was never tested"))
    if len(replies) < 2 or idle_min is None:
        if idle_min is None:
            checks.append(("WIFI_IDLE_MIN is readable from GET *", False,
                           "GET * did not list the key: the countdown has no ruler"))
        return checks

    def parse(l):
        m = WIFI_LINE.match(l)
        return (int(m.group(1)), int(m.group(2)), int(m.group(3))) if m else None

    for what, l in (("boot", replies[0]), ("mid-dive", replies[1])):
        f = parse(l)
        if f is None:
            checks.append((f"radio: {what} answer parses", False, f"{what} answered {l!r}"))
            continue
        on, held, left = f
        checks.append((f"radio: {what} fields agree with each other ({l})",
                       (on and held == 0) or (not on and 1 <= held <= 4 and left == 0),
                       "an up radio cannot also be held, and a down radio has no "
                       "countdown running — the three fields told different stories"))
        bound = (left <= idle_min * 60) if on else True
        checks.append((f"radio: {what} countdown agrees with WIFI_IDLE_MIN={idle_min}",
                       bound,
                       f"{l} — the countdown promises a fire the setting does not "
                       f"(GET * and GET WIFI are the same registry: a mismatch is "
                       f"one of them lying)"))
    # Bench-specific: this run reset the board minutes ago, the matrix opened its
    # dive with the radio up, and the boot console announced the AP. A radio that
    # the IDLE rule had already taken down before the first GET WIFI means the
    # board's persisted WIFI_IDLE_MIN is not the default — a previous interrupted
    # --wifi-idle run — and every later radio check runs in a state the operator
    # did not choose.
    on0, held0, _ = parse(replies[0]) or (0, 9, 0)
    ap_boot = any(l.startswith("web: AP ") for l in boot_lines)
    checks.append(("radio: the AP was still up when the matrix started",
                   (on0 == 1) or not ap_boot,
                   f"{replies[0]} at boot with the AP announced this run — held by "
                   f"{held0}; if that is 4 (idle), the persisted WIFI_IDLE_MIN is "
                   f"short: the --wifi-idle leg restores 10, check for an aborted run"))
    mid = parse(replies[1])
    checks.append(("radio: mid-dive is blamed on the dive",
                   mid == (0, 2, 0),
                   f"{replies[1]} — mid-dive, with intent taken by DIVE START, the "
                   f"closest reason is the dive (2): any other number means the "
                   f"reason column and the dive disagree"))
    return checks


def selftest_wifi():
    """Proves the grade_wifi cross-field rules can fail. Same discipline as the
    surface-log and dive-export selftests: one known lie per case, and a case
    that cannot fail is decoration."""
    boot = ['web: AP "ESP-CCR-ABCD" on 192.168.4.1']
    def pairs_with(first, second, idle="10"):
        star = [f"WIFI_IDLE_MIN={idle}", "AP_SSID=ESP-CCR"]
        return [("GET *", star), ("GET WIFI", [first]), ("GET WIFI", [second])]
    honest = ("WIFI,1,0,598", "WIFI,0,2,0")
    cases = [
        ("an honest boot+dive pair", pairs_with(*honest), boot, None),
        ("an up radio that also claims a holder",
         pairs_with("WIFI,1,3,598", honest[1]), boot, "fields agree with each other"),
        ("a down radio blamed on nothing",
         pairs_with("WIFI,0,0,0", honest[1]), boot, "fields agree with each other"),
        ("a down radio still counting down",
         pairs_with("WIFI,0,4,90", honest[1]), boot, "fields agree with each other"),
        ("a countdown longer than the setting allows",
         pairs_with("WIFI,1,0,601", honest[1]), boot, "countdown agrees"),
        ('WIFI_IDLE_MIN=0 but a countdown is running',
         pairs_with("WIFI,1,0,300", honest[1], idle="0"), boot, "countdown agrees"),
        ("the mid-dive pair never ran",
         pairs_with(honest[0], honest[0])[:2], boot, "leg ran twice"),
        ("the boot radio had already idled out",
         pairs_with("WIFI,0,4,0", honest[1]), boot, "AP was still up"),
        ("mid-dive blames the diver, not the dive",
         pairs_with(honest[0], "WIFI,0,1,0"), boot, "blamed on the dive"),
        ("GET * does not carry the ruler key",
         [("GET *", ["AP_SSID=ESP-CCR"]), ("GET WIFI", [honest[0]]),
          ("GET WIFI", [honest[1]])], boot, "WIFI_IDLE_MIN is readable"),
    ]
    checks = []
    for name, pr, bl, expect in cases:
        got = grade_wifi(pr, bl)
        failed = [c[0] for c in got if not c[1]]
        if expect is None:
            checks.append((name, not failed, f"unexpected failures: {failed}"))
        else:
            checks.append((name, any(expect in f for f in failed),
                           f"no failure matching {expect!r}; failures seen: {failed}"))
    return checks


def split_dive_rows(lines):
    """`S,`/`E,` rows from a dive export, in file order.

    The export writes the two kinds as separate `write()` batches, so a chunked
    transport can tear a row at a batch boundary and pyserial hands back two
    halves (`S,2,14.6,0.612,0.220,...` arriving as `S,2,14.6,0.6` + `12,...`).
    Framing is graded separately and at length (run_framing); what this splitter
    guarantees is that the row grader sees what the FILE holds: short fragments
    are re-joined until each line starts with a row id. A row that never
    completes stays its own line and fails its shape check — the honest verdict:
    the file is corrupt, and no repair here may make that read as clean."""
    out = []
    for ln in lines:
        t = ln.strip()
        if not t:
            continue
        if not out or t.split(",")[0] in ("S", "E", "M", "DIVE"):
            out.append(t)
        else:
            out[-1] += t
    return out


def grade_dive_file(drows, want, wire=()):
    """Grade one exported dive against itself, and against the settings it claims.

    Five claims, each a different lie:

      * the dive FILE holds the verdict rows the wire streamed (ids 10/11/12 for
        a failed cell proof, 1/2 for alarms, 3 for mode, 4 for setpoint): until
        2026-11-24 the wire carried them and the file did not, so a dive
        downloaded afterwards showed a setpoint getting capped without ever
        saying why — the effects survived, the verdicts did not;
      * those rows arrive in the order they happened, so a reader can put the
        override after the failure rather than guess;
      * the `M,` summary agrees with the rows beside it. It printed
        `snapshots 0, events 0, gf 0/0, sp 0.000 x4` next to a dozen samples:
        a summary line contradicting its own dive;
      * no impossible ppO2. A dive that never voted published the -32768
        sentinel as `-32.768` bar, in the same three decimals as a real reading;
      * every `S,` row's `ndl` answers within the published domain — a number
        0..480, or exactly one of the named sentinels (-1 unknown / 480
        unlimited, the same pair `D,` answers since 2026-11-28 closed review
        N5's two-sentinels-for-one-fact). Vacuous on captures older than that
        date — they all answered inside the domain too — and it witnesses the
        new firmware's -1 rows on a live `--dive-persist` run.

    `want` is what the same link answered for GET GF_LOW / GF_HIGH / SP1..SP4,
    `wire` the `E,` rows seen live while the dive was open. A claim nobody could
    check (an empty `want`, a missing M, column) is a FAIL, never a pass.

    The join is deliberately ONE-WAY: every row the handset streamed while the
    dive was open must be in the file, but a row streamed outside that window
    (an alarm edge a second before `DIVE START`, the last ones drained after
    `DIVE END`) legitimately has no file row. Asserting equality in both
    directions would fail a dive that worked as designed.
    """
    # Re-joined first: see split_dive_rows. Every count below is a count of
    # what the FILE holds, not of what the transport happened to frame.
    drows = split_dive_rows(drows)
    samp = [l for l in drows if l.startswith("S,")]
    ev = [l for l in drows if l.startswith("E,")]
    meta = next((l for l in drows if l.startswith("M,")), "")
    mf = meta.split(",")

    # ---- the pulses column: every window must be real, and both row kinds
    # must read one base (decision 2026-11-27, batch E: an event-driven `S,`
    # row carries a real window now, where it used to be hard-wired to 0).
    #
    # Three claims, graded on the exported file:
    #   * every `S,` row's window parses — a column the grader cannot read is a
    #     column that moved without the format saying so;
    #   * the file is not a wall of zeros beside a non-zero total. That is what
    #     the export used to look like for its event rows: injections counted
    #     for the grid rows, `0` published for the decisions in between;
    #   * no window exceeds the dive's total. The two kinds' windows OVERLAP by
    #     design (§9), so their SUM telling a bigger story than the total is not
    #     evidence; one window larger than everything the dive injected is
    #     arithmetic that cannot happen if both kinds subtract the same base.
    def window(line):
        """The `pulses` column of an `S,` row (§5: column 14, 1-based)."""
        f = line.split(",")
        try:
            return int(f[13])
        except (IndexError, ValueError):
            return None

    sw = [window(l) for l in samp]
    good_sw = [w for w in sw if w is not None]
    total = None
    if len(mf) > 5 and mf[5].isdigit():
        total = int(mf[5])
    if total == 0:
        # Say it out loud. A dive that never injected is legitimate — at the
        # surface, in STANDBY, or with AUTO refused by a failed pre-dive check
        # (its BATTERY step refuses below vbatMin, and the injector only ever
        # wants a pulse in AUTO) — but every window check below is then vacuous:
        # a wall of zeros is indistinguishable from a real one. A PASS printed
        # over a vacuous check is the shape of bug this bench has been burned by.
        print(f"    note: M, total is 0 — the chain injected nothing in this dive, "
              f"so the pulses-window checks below witness nothing. Run the dive in "
              f"AUTO (pack above vbatMin) to make them mean something.")

    def val(key):
        s = want.get(key, "")
        try:
            return float(s.split("=", 1)[1])
        except (ValueError, IndexError, TypeError):
            return None

    def same(key, col):
        v, shown = val(key), None
        try:
            shown = float(mf[col])
        except (ValueError, IndexError, TypeError):
            return False
        return v is not None and abs(v - shown) < 1e-6

    def num(col):
        try:
            return float(mf[col])
        except (ValueError, IndexError, TypeError):
            return None

    def ndl_in_domain(line):
        # S,<t>,<depth>,<voted>,<c0>,<c1>,<c2>,<sp>,<alarm>,<cns>,<ceil_m>,<ndl>,...
        f = line.split(",")
        if len(f) <= 11:
            return False  # a documented column the row does not carry
        try:
            n = int(f[11])
        except ValueError:
            return False
        return n == -1 or 0 <= n <= 480

    # Multiset, not set: two identical rows (a rejoin for two different cells
    # reads alike in value but not in id, and a duplicate edge is a duplicate
    # bug) must both be in the file.
    missing = sorted((Counter(wire) - Counter(ev)).elements())
    it = iter(ev)
    in_order = all(any(w == x for x in it) for w in wire)

    # Event-driven rows split out by whether they sit beside an injection: an
    # `E,` row inside a 30 s span that injected pulses must NOT read 0 (that is
    # the placeholder shape the decision removed), and one inside a silent span
    # legitimately does.
    checks = [
        ("dive file: every S, row has a pulses column that parses",
         len(good_sw) == len(samp),
         f"{len(samp)} S, rows, {len(good_sw)} parsed — a window the grader "
         f"cannot read is a column that moved without the format saying so"),
        ("dive file: the pulses windows are not a wall of zeros beside a "
         "non-zero total (the placeholder shape: the file counted injections "
         "for some row kinds and hard-wired 0 for the others)",
         not (total and total > 0 and good_sw and not any(good_sw)),
         f"M, total {total}, S, windows {good_sw[:12]}"),
        ("dive file: no row claims more injections in its window than the dive "
         "injected in total (§9: the two row kinds' windows OVERLAP, so their "
         "sum may exceed the total by design — but a window larger than the "
         "total is arithmetic that cannot happen if both kinds read one base)",
         total is not None and all(w <= total for w in good_sw),
         f"windows {good_sw[:12]} vs M, total {total}"),
    ]
    checks += [
        ("dive file: the link saw the dive's verdict rows live, so the join "
         "below is not vacuous", bool(wire),
         "no E, row was captured while the dive was open. Every dive opens with "
         "per-cell vote-rejoin rows (id 6), so an empty capture means the "
         "watching loop read nothing — and a join against nothing can never "
         "fail, which is the kind of check that has burned this bench before"),
        ("dive file: every verdict the wire streamed is in the file", not missing,
         f"missing {missing or 'none'}: the file holds {len(ev)} E, rows, the wire "
         f"streamed {len(wire)} — a dive downloaded afterwards must answer 'what "
         f"did the machine decide, and when' with the bytes the handset saw live"),
        ("dive file: the verdict rows are in the order they happened", in_order,
         f"file {ev} vs wire {list(wire)}: the override must sit after the "
         f"failure it lifted, not just somewhere in the dive"),
        ("dive file: it holds verdict rows at all", bool(ev),
         f"{len(samp)} S, rows and {len(ev)} E, rows. Every dive opens with "
         f"per-cell vote-rejoin rows (id 6), so an empty event set means the "
         f"publication or the file join is broken, not that nothing happened"),
        ("dive file: M, snapshot count matches the S, rows",
         len(mf) > 7 and mf[7].isdigit() and int(mf[7]) == len(samp),
         f"M, says {mf[7] if len(mf) > 7 else '?'} snapshots, the file holds "
         f"{len(samp)} S, rows"),
        ("dive file: M, event count matches the E, rows",
         len(mf) > 8 and mf[8].isdigit() and int(mf[8]) == len(ev),
         f"M, says {mf[8] if len(mf) > 8 else '?'} events, the file holds "
         f"{len(ev)} E, rows"),
        ("dive file: M, states the GF pair the dive ran on",
         same("GF_LOW", 9) and same("GF_HIGH", 10),
         f"M, gf {mf[9:11] if len(mf) > 10 else '?'} vs the link's "
         f"{want.get('GF_LOW', '?')} / {want.get('GF_HIGH', '?')}"),
        ("dive file: M, states the setpoint ladder the dive ran on",
         all(same(k, 11 + i) for i, k in enumerate(("SP1", "SP2", "SP3", "SP4"))),
         f"M, sp {mf[11:15] if len(mf) > 14 else '?'} vs the link's "
         + " ".join(want.get(k, "?").split("=")[-1] for k in ("SP1", "SP2", "SP3", "SP4"))),
        ("dive file: no impossible ppO2 in the summary",
         num(3) is not None and num(4) is not None and num(3) >= 0.0 and num(4) >= 0.0,
         f"M, min_ppo2 {mf[3] if len(mf) > 3 else '?'} max_ppo2 "
         f"{mf[4] if len(mf) > 4 else '?'}: a dive that never voted used to "
         f"publish the -32768 sentinel as -32.768 bar"),
        ("dive file: every S, ndl column is a number or the named sentinel pair",
         all(ndl_in_domain(l) for l in samp),
         "an S, row stated an ndl outside the published domain — the pair is "
         "-1 (no valid vote, the same unknown `D,` answers) and 480 (the "
         "engine's own unlimited), anything else 0..480 (HANDSET_PROTOCOL "
         "§2/§5.1, one convention for file and wire since 2026-11-28). A row "
         "answering e.g. 999 states a number the controller never believed"),
    ]
    return checks


def selftest_dive_file():
    """Proves the dive-file graders can fail, no board needed.

    The first case is the honest dive as it now exports off the fixed build; the
    second is the same bench, one day earlier, and it must fail on the missing
    verdict rows. Both are real captures. The rest are one-change corruptions of
    the honest dive, each the shape of a bug the fix could plausibly regress
    into — the counts lying again, the plan columns zeroed again, the never-voted
    sentinel printed as a ppO2.
    """
    def row(idx, new):
        out = list(DIVE_HONEST)
        out[idx] = new
        return out

    def drop(prefix):
        return [l for l in DIVE_HONEST if not l.startswith(prefix)]

    def zerod_pulses(rowline):
        """Zero the `pulses` window of one `S,` row (§5 column 14)."""
        f = rowline.split(",")
        f[13] = "0"
        return ",".join(f)

    def set_pulses(rowline, v):
        f = rowline.split(",")
        f[13] = str(v)
        return ",".join(f)

    # An injecting dive: 20 grid-row windows of 7 (140 total) with an event row
    # beside them. Real bench numbers from the 2026-11-27 acceptance run's dive
    # (slot 1, `M,` total 140). This corpus exists because the honest dive above
    # never injected, so it cannot witness the placeholder shape at all.
    S_INJ = ("S,30,0.04,0.214,0.213,0.214,0.214,0.400,0x0010,0,0,-1,0,7,1562,"
             "2,0,0,0")
    S_INJ2 = ("S,60,0.04,0.214,0.213,0.214,0.214,0.400,0x0010,0,0,-1,0,9,1562,"
              "2,0,0,0")
    S_EV = ("S,41,0.04,0.214,0.213,0.214,0.214,0.400,0x0010,0,0,-1,0,3,1562,"
            "2,0,0,0")     # event-driven: 3 of the 7 in this 30 s span
    DIVE_INJECT = [
        "DIVE,10,1795540808,3,3,2026-11-24T17:20:08Z",
        "E,0,6,0", S_INJ,
        "E,41,1,4", S_EV,
        S_INJ2,
        "M,60,4,0.213,0.214,16,0,3,2,30,85,0.400,0.550,1.000,1.300",
    ]

    honest_m = DIVE_HONEST[-1].split(",")

    def meta(**over):
        f = list(honest_m)
        # M,<dur>,<cm>,<min>,<max>,<pulses>,<cns>,<snaps>,<evs>,<gfl>,<gfh>,<sp1..4>
        col = {"snaps": 7, "evs": 8, "gfl": 9, "gfh": 10, "sp1": 11, "sp2": 12,
               "sp3": 13, "sp4": 14, "min": 3, "max": 4}
        for k, v in over.items():
            f[col[k]] = v
        return ",".join(f)

    cases = [
        # name, export rows, settings the link reported, wire E, rows, expected
        # failing check (None = must pass clean)
        ("an honest dive: verdict rows, samples and a summary that agrees",
         DIVE_HONEST, DIVE_WANT, DIVE_HONEST_WIRE, None),
        ("the same dive as it shipped before 2026-11-24: the wire streamed the "
         "proof FAIL, the clamp and the override; the file holds none of them",
         DIVE_PRE_FIX, DIVE_WANT, WIRE_PRE_FIX, "streamed is in the file"),
        ("the clamp rows are there but out of order (override before failure)",
         drop("E,") + ["E,66,12,1", "E,54,10,0", "E,54,11,0"], DIVE_WANT,
         ["E,54,10,0", "E,54,11,0", "E,66,12,1"], "order they happened"),
        ("M, claims 0 snapshots beside 13 samples (the counts never patched)",
         DIVE_PRE_FIX, DIVE_WANT, [], "snapshot count matches"),
        ("M, claims 0 events while the file now holds 4",
         row(len(DIVE_HONEST) - 1, meta(evs="0")), DIVE_WANT, DIVE_HONEST_WIRE,
         "event count matches"),
        ("M, claims 9 snapshots the file does not hold",
         row(len(DIVE_HONEST) - 1, meta(snaps="9")), DIVE_WANT, DIVE_HONEST_WIRE,
         "snapshot count matches"),
        ("M, states GF 0/0 while the link says 30/85",
         row(len(DIVE_HONEST) - 1, meta(gfl="0", gfh="0")), DIVE_WANT,
         DIVE_HONEST_WIRE, "GF pair"),
        ("M, states a 0.000 ladder while the dive ran 0.40/0.55/1.00/1.30",
         row(len(DIVE_HONEST) - 1,
             meta(sp1="0.000", sp2="0.000", sp3="0.000", sp4="0.000")),
         DIVE_WANT, DIVE_HONEST_WIRE, "setpoint ladder"),
        ("a dive that never voted publishes the sentinel as -32.768 bar",
         row(len(DIVE_HONEST) - 1, meta(min="0.000", max="-32.768")), DIVE_WANT,
         DIVE_HONEST_WIRE, "impossible ppO2"),
        ("the placeholder shape: a dive that injected 16 pulses and every "
         "window reads 0 — the file's event rows were hard-wired to 0 until the "
         "2026-11-27 decision, and this is what that looked like",
         [set_pulses(l, 0) if l.startswith("S,") else l for l in DIVE_INJECT],
         DIVE_WANT, ["E,0,6,0", "E,41,1,4"], "wall of zeros"),
        ("an event row's window reads 40 while the dive injected 16: the base "
         "the event row subtracts is not the grid row's (one base, or the same "
         "injections count twice)",
         [set_pulses(l, 40) if l == S_EV else l for l in DIVE_INJECT],
         DIVE_WANT, ["E,0,6,0", "E,41,1,4"], "injections in its window"),
        ("an S, row states NDL 999: neither a number the engine saturates at "
         "nor the named pair — the column invented one (N5's domain check)",
         row(2, DIVE_HONEST[2].replace(",0x0010,0,0,0,0,0,", ",0x0010,0,0,999,0,0,")),
         DIVE_WANT, DIVE_HONEST_WIRE, "named sentinel pair"),
        ("no M, row at all: every claim in it is unchecked",
         drop("M,"), DIVE_WANT, DIVE_HONEST_WIRE, "snapshot count matches"),
        ("the settings query came back empty: a claim nobody can check is a fail",
         DIVE_HONEST, {}, DIVE_HONEST_WIRE, "GF pair"),
        ("the watching loop captured nothing: a join against nothing can never "
         "fail, so it must not be allowed to pass quietly",
         DIVE_HONEST, DIVE_WANT, [], "not vacuous"),
        ("a truncated export: 3 of 4 samples arrive, the summary says 4",
         drop("S,")[:3] + [l for l in DIVE_HONEST if not l.startswith("S,")],
         DIVE_WANT, DIVE_HONEST_WIRE, "snapshot count matches"),
    ]
    checks = []
    for name, rows, wn, wire, want in cases:
        failed = [c[0] for c in grade_dive_file(rows, wn, wire) if not c[1]]
        if want is None:
            checks.append((f"dive file: {name}", not failed,
                           f"unexpected failures: {failed}"))
        else:
            checks.append((f"dive file: {name}", any(want in f for f in failed),
                           f"no failure matching {want!r}; failures seen: {failed}"))
    return checks



# --------------------------------------------------------------------------------
# The stall budget: the console warning and the supervisor reboot are one quantity,
# measured twice. A comment that says "800 is below 1500" is prose; this join makes
# the relationship a gate, so moving TASK_STALL_CONTROL_MS without re-choosing the
# grade reddens the run instead of silently widening the margin.
#
# The two numbers live in two files ON PURPOSE: the firmware owns the reboot budget,
# this tool owns the warning that precedes it. selftest_stall_budget() is what keeps
# them honest to each other.
# --------------------------------------------------------------------------------
FIRMWARE_DEFAULTS_H = Path(__file__).resolve().parent.parent / "include/config/defaults.h"

# How far below the reboot the warning may drift and still be about STALLING. The
# floor comes from this board's own record, not from taste: the worst ControlTask
# cycle ever measured on a healthy bench is 505 ms (BENCH.md gate 5, 639 samples),
# so a grade below ~500 ms stops meaning "stalling" and starts failing a board
# that is fine.
STALL_BUDGET_MIN_FRACTION = 0.30


def stall_budget_from(text):
    """Pulls TASK_STALL_CONTROL_MS out of header TEXT, not a path: the fixture
    case is a string, and a file read inside the grader would make that case a
    statement about the filesystem instead of about the join."""
    m = re.search(r"^\s*#define\s+TASK_STALL_CONTROL_MS\s+(\d+)", text, re.M)
    if not m:
        raise ValueError("no `#define TASK_STALL_CONTROL_MS <n>` in the firmware "
                         "header: the stall budget moved or was renamed, and the "
                         "tool's grade is now an unanchored number")
    return int(m.group(1))


def firmware_stall_budget(path=FIRMWARE_DEFAULTS_H):
    return stall_budget_from(Path(path).read_text(encoding="utf-8"))


# Read ONCE at import, and say so loudly if the header cannot answer: a tool that
# quietly falls back to a remembered number is the failure mode this join exists
# to prevent. selftest_stall_budget() reddens on the same condition.
try:
    FIRMWARE_STALL_BUDGET_MS = firmware_stall_budget()
except Exception as _budget_err:  # noqa: BLE001 - the message IS the finding
    FIRMWARE_STALL_BUDGET_MS = None
    print(f"hw_accept: WARNING: cannot read the stall budget: {_budget_err}",
          file=sys.stderr)


def selftest_stall_budget():
    """Records the 800-vs-1500 decision as an executable, not a comment.

    Nothing here claims 800 ms is right forever. It claims 800 is chosen AGAINST
    the firmware's reboot budget, in both directions: a warning above the reboot
    is useless, and a warning absurdly far below it is grading a different
    problem. If either number moves, this stops being green and forces the
    choice to be made again on purpose.
    """
    checks = []
    budget = None
    try:
        budget = firmware_stall_budget()
    except Exception as e:  # an unreadable header IS the finding
        checks.append(("the stall budget is readable from the firmware header",
                       False, str(e)))
    if budget is not None:
        checks.append((f"the stall budget reads as a plausible number (got {budget} ms)",
                       100 <= budget <= 60_000,
                       f"TASK_STALL_CONTROL_MS = {budget} in {FIRMWARE_DEFAULTS_H}"))
        checks.append((f"the worst= grade ({HB_WORST_CYCLE_MS} ms) is stricter than the "
                       f"{budget} ms supervisor reboot it warns about",
                       HB_WORST_CYCLE_MS < budget,
                       f"HB_WORST_CYCLE_MS = {HB_WORST_CYCLE_MS}, budget = {budget}: a "
                       "board can reboot before the console ever says anything"))
        checks.append((f"the worst= grade is at least {STALL_BUDGET_MIN_FRACTION:.0%} of "
                       "the reboot budget, so it grades stalling and not ordinary noise",
                       HB_WORST_CYCLE_MS >= STALL_BUDGET_MIN_FRACTION * budget,
                       f"HB_WORST_CYCLE_MS = {HB_WORST_CYCLE_MS} vs budget {budget}"))
        # The number being pinned must be the number the pass/fail turns on, AND
        # the reboot budget in the check's own words must come from the header:
        # the old name printed a literal 1500, a third copy of a number the tool
        # had no business owning.
        src = inspect.getsource(grade_watch)
        checks.append(("the live grader compares against the pinned constant",
                       "worst_cycle <= HB_WORST_CYCLE_MS" in src,
                       "grade_watch compares a literal: re-choosing the constant "
                       "would change nothing on the bench"))
        checks.append(("the grader names the budget it read, not a literal 1500",
                       "FIRMWARE_STALL_BUDGET_MS" in src and "reboot budget 1500" not in src,
                       "the check text carries its own copy of the firmware's number"))
    # The join itself, both directions: a header with the define is read, a header
    # with only a LOOK-ALIKE define raises instead of quietly borrowing someone
    # else's number.
    good = "#define TASK_STALL_CONTROL_MS       1500\n"
    lookalike = "#define TASK_STALL_CONTROL_MS_OTHER 999\n#define TASK_WATCHDOG_MS 1\n"
    checks.append(("the reader finds the define in a synthetic header",
                   stall_budget_from(good) == 1500, ""))
    try:
        got = stall_budget_from(lookalike)
        checks.append(("the reader refuses a header without the exact define",
                       False, f"it read {got} from a header that has no such define"))
    except ValueError:
        checks.append(("the reader refuses a header without the exact define", True, ""))
    # And the boundary of the grade, so the pinned constant is shown to be live:
    # one under passes, one over fails, and a cycle at the reboot budget fails
    # loudly — a board that stalled to the reboot point must not grade clean.
    beat = ("HB seq=1 open=0 depth=0.00 sp=0.00 d=0.00 valve=0 cons=0 rem=0 bails=0 "
            "i2cerr=0 fresh=113 bat=3.87 batadc=3.87 batadcstate=0 batinv=0 "
            "batraw=1512 batrawstate=1 caloff=0 cfggen=0 vrail=0.00 railstate=3 ")
    for worst, should_fail in ((HB_WORST_CYCLE_MS - 1, False),
                               (HB_WORST_CYCLE_MS, False),
                               (HB_WORST_CYCLE_MS + 1, True),
                               (budget or 1500, True)):
        line = beat + f"worst={worst} logskip=0 surfskip=0 contskip=0"
        ch = grade_watch([line])
        bad = [c for c in ch if c[0].startswith("worst= cycle margin") and not c[1]]
        checks.append((f"worst={worst} is graded "
                       f"{'a cycle-time failure' if should_fail else 'clean'}",
                       bool(bad) == should_fail,
                       bad[0][2] if bad else next(c[2] for c in ch
                                                  if c[0].startswith("worst= cycle margin"))))
    return checks


# --------------------------------------------------------------------------------
# The NVS migration leg: "a stored value the new band cannot hold is refused, the
# refusal is said out loud, and the registry default is what actually runs."
#
# REVIEW.md (1 P1) carries that as a prose claim: the boot line repeats every boot
# until a diver writes a legal value. The boot leg graded only that the boot is
# clean, so a board that printed the refusal AND STILL USED the stale number — and
# a board that accepted the stale number in silence — both passed. Both are graded
# here, and neither grade copies a firmware number into this tool: the key's band
# and its default come from the board's own `GET SCHEMA`, so the join is
# board-to-board.
# --------------------------------------------------------------------------------
NVS_REFUSAL_RE = re.compile(
    r'^settings: NVS (?P<key>[A-Z0-9_]+)="(?P<val>[^"]*)" refused \((?P<why>[^)]+)\)'
    r' - the registry default for that key is used instead$')

# `K,<key>,<type>,<min>,<max>,<flags>,<default>` — the board's own published table
# (src/comms/cmd_router.cpp renderSchema). Field order is the wire contract.
SCHEMA_ROW_RE = re.compile(
    r"^K,(?P<key>[A-Z0-9_]+),(?P<type>[A-Z]),(?P<min>-?[\d.]+),(?P<max>-?[\d.]+),"
    r"(?P<flags>\d+),(?P<def>-?[\d.]+)$")


def parse_schema(rows):
    """{key: (type, min, max, default)} from the `K,` lines of `GET SCHEMA`."""
    out = {}
    for l in rows:
        m = SCHEMA_ROW_RE.match(l.strip())
        if m:
            out[m.group("key")] = (m.group("type"), float(m.group("min")),
                                   float(m.group("max")), float(m.group("def")))
    return out


def grade_nvs_migration(boot_lines, schema_rows, readback):
    """boot_lines: the captured boot. schema_rows: this session's `K,` lines.
    readback: {key: value} answered by `GET <key>` AFTER that boot.

    A refusal that is merely PRESENT is reported as a finding with its remedy, not
    as a firmware failure: an old value in NVS is the diver's business, and the
    open item closes the day the refusal stops appearing.
    """
    checks = []
    schema = parse_schema(schema_rows)
    refs = [m for m in (NVS_REFUSAL_RE.match(l.strip()) for l in boot_lines) if m]
    # The refusal sentence is a contract: key, the stored text quoted verbatim, the
    # reason, and the consequence. A truncated tail is the interesting failure,
    # because the tail is the only part that says WHAT RUNS INSTEAD.
    loose = [l for l in boot_lines if "settings: NVS" in l and "refused" in l]
    checks.append(("every NVS refusal states the key, the stored value, the reason "
                   "and what is used instead",
                   len(loose) == len(refs),
                   next((l for l in loose if not NVS_REFUSAL_RE.match(l.strip())), "")))
    for m in refs:
        key, stored = m.group("key"), m.group("val")
        row = schema.get(key)
        checks.append((f"the refused key {key} is one the board publishes a schema row for",
                       row is not None,
                       f"{key} refused at boot but absent from GET SCHEMA: the refusal "
                       "names a key the handset cannot even write to"))
        live = readback.get(key)
        if live is None:
            checks.append((f"{key}: the refused value is not what is running", False,
                           f"{key} refused at boot and never read back: nothing proves "
                           "the default replaced it"))
            continue
        checks.append((f"{key}: the refused value {stored} is NOT live",
                       live.strip() != stored.strip(),
                       f"boot refused {stored} and GET {key} answers {live}: the "
                       "refusal is decoration, the stale value is in the chain"))
        if row is None:
            continue
        try:
            lv = float(live)
        except ValueError:
            checks.append((f"{key}: the read-back parses as a number", False,
                           f"GET {key} answered {live!r}"))
            continue
        checks.append((f"{key}: the live value sits inside the band the board publishes "
                       "for it", row[1] <= lv <= row[2],
                       f"{live} is outside [{row[1]}, {row[2]}]: the value the band "
                       "rejects is the value in force"))
        # The sentence promises the registry DEFAULT, and the board publishes that
        # default on the same schema row: two board answers joined, not a number
        # copied into the tool.
        checks.append((f"{key}: the live value is the published default",
                       abs(lv - row[3]) < 1e-6,
                       f"boot says the default is used; GET {key} answers {live}, the "
                       f"schema default is {row[3]}"))
    # The other direction — the one that passes today: a stored value the band
    # cannot hold, accepted in silence. Anything live and out of band with no
    # refusal naming it is that defect.
    silent = []
    for key, val in readback.items():
        row = schema.get(key)
        if row is None or row[0] == "S":
            continue
        try:
            lv = float(val)
        except ValueError:
            continue
        if not (row[1] <= lv <= row[2]) and not any(m.group("key") == key for m in refs):
            silent.append(f"{key}={val} outside [{row[1]}, {row[2]}] with no boot refusal")
    checks.append(("nothing out of band is running that the boot did not refuse",
                   not silent, "; ".join(silent)))
    if refs:
        first = next(l for l in boot_lines if NVS_REFUSAL_RE.match(l.strip()))
        checks.append((f"NVS holds a value the current band rejects — {first} — run "
                       "`SET <KEY>` to replace it: the refusal repeats every boot "
                       "until the stale string is written back — and one settings "
                       "save rewrites EVERY key from the in-force registry, so a "
                       "write to any other key clears it without touching this one "
                       "(bench-seeded 2026-11-28: 35 keys + refusal, then 36 keys "
                       "and no refusal, with no `SET` to that key)", True,
                       "a bench condition, recorded as a finding, not a firmware fault"))
    return checks


def selftest_nvs_migration():
    """Lying boots, both directions.

    The honest case is this bench's own boot, captured not typed: VBAT_MIN stored
    as 10.500 under the old 12 V chemistry, the tightened band rejecting it, the
    board answering 3.400. The mutations are the two that used to pass green —
    refuse-and-still-use, and accept-in-silence — plus the sentence losing its
    tail, which is the half that says what runs instead.
    """
    schema = ["K,SP2,F,0.2,4,0,0.21",
              "K,VBAT_MIN,F,3,4.2,0,3.4",
              "K,GF_LOW,F,1,3,0,1.3",
              "K,AP_PASS,S,0,0,0,0"]
    honest_boot = ["ets Jun  8 2016",
                   'settings: NVS VBAT_MIN="10.500" refused (out of range) - the '
                   "registry default for that key is used instead",
                   "settings: 35 keys from NVS"]

    def g(boot, readback):
        return grade_nvs_migration(boot, schema, readback)

    def bad(ch):
        return [c for c in ch if not c[1]]

    checks = [("the honest boot passes clean", not bad(g(honest_boot, {
        "VBAT_MIN": "3.400", "SP2": "0.210", "GF_LOW": "1.300"})),
        "; ".join(c[0] for c in bad(g(honest_boot, {
            "VBAT_MIN": "3.400", "SP2": "0.210", "GF_LOW": "1.300"}))))]
    muts = [
        ("refuses the value and STILL runs it", honest_boot,
         {"VBAT_MIN": "10.500", "SP2": "0.210"}, "is NOT live"),
        ("refuses it, then runs something that is neither the stored value nor the default",
         honest_boot, {"VBAT_MIN": "3.900", "SP2": "0.210"}, "is the published default"),
        ("the refusal loses its tail — the half that says what runs instead",
         ['settings: NVS VBAT_MIN="10.500" refused (out of range)'],
         {"VBAT_MIN": "3.400"}, "what is used instead"),
        ("refuses a key that is not in the published table",
         ['settings: NVS VBATMINX="10.500" refused (out of range) - the registry '
          "default for that key is used instead"], {"VBATMINX": "3.400"},
         "publishes a schema row"),
        ("the stale value is accepted in silence: no refusal, out-of-band value live",
         ["settings: 36 keys from NVS"], {"VBAT_MIN": "10.500"},
         "nothing out of band is running"),
        ("the refused key is never read back, so 'the default is in force' is unproven",
         honest_boot, {"SP2": "0.210"}, "is not what is running"),
    ]
    for name, boot, read, expect in muts:
        ch = g(boot, read)
        checks.append((f"mutation: {name}", any(expect in c[0] for c in bad(ch)),
                       "; ".join(c[0] for c in bad(ch)) or "nothing failed"))
    # A clean NVS must invent nothing: no refusal, everything inside its band.
    clean = g(["settings: 36 keys from NVS"], {"VBAT_MIN": "3.600", "SP2": "0.300"})
    checks.append(("a board with nothing stale reports nothing", not bad(clean),
                   "; ".join(c[0] for c in bad(clean))))
    # A grader that exists and is never called is the failure mode this repo has
    # been bitten by before (a check hunting an `E,` row its own line filter threw
    # away). The fixtures prove the grader; this proves the wiring.
    checks.append(("the live run actually calls this leg",
                   "grade_nvs_migration(" in inspect.getsource(main),
                   "grade_nvs_migration is defined and graded by its own selftest, "
                   "but no acceptance run reaches it"))
    return checks

def selftest_port_hygiene():
    """A replay must never reach a port. Graded as a subprocess, because the
    point is what the process DOES, not what one function returns.

    `--replay` with a bad path used to be falsy, fell through to the live branch
    and opened the console port — whose CP2102 reset reboots the controller. The
    bench found that by losing a boot mid-session, so the guard is graded here
    rather than trusted.

    The subprocess case deliberately passes a NON-EXISTENT path, not `""`: an
    empty one is the shape that reaches the live branch, and a selftest must not
    be able to open `/dev/ttyUSB0` if the guard ever regresses. That shape is
    covered by the source scan instead, which cannot touch a port.
    """
    import subprocess

    checks = []
    me = str(Path(__file__).resolve())
    out = subprocess.run([sys.executable, me, "--replay", "/nonexistent/accept.log"],
                         capture_output=True, text=True, timeout=30)
    checks.append(("a replay it cannot read exits instead of opening a port",
                   out.returncode == 2 and "Refusing to fall back" in out.stderr,
                   f"rc={out.returncode} stderr={out.stderr.strip()[:200]}"))
    src = inspect.getsource(main)
    live = src.split("import serial")[0] if "import serial" in src else src
    checks.append(("the replay branch is taken before pyserial is imported",
                   "if args.replay is not None" in live,
                   "the replay test sits after the serial import: a bad argument "
                   "then reaches the wire"))
    checks.append(("an empty --replay is not silently treated as 'no replay'",
                   "if args.replay is not None:" in src and "if args.replay:" not in src,
                   "truthiness on a path argument: `--replay \"\"` means 'live'"))
    return checks


def selftest_matrix_window():
    """The reply-window rule, proven on scripted transcripts. No port, no board.

    The defect is not that a multi-line answer is long — it is that a window
    which stops at the first match leaves the rest of ONE write() in the buffer,
    and those lines then answer the NEXT command. So every case here grades two
    consecutive windows: what the first consumed, and what the second found. A
    case that passes with only the first window graded would be decoration.
    """
    hist3 = ["HIST,0,60,10,25,3,40,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,"
             "19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37",
             "HIST,1,60,10,25,3,41,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,"
             "19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37",
             "HIST,2,60,10,25,3,39,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,"
             "19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37"]
    next_answer = "SP2=0.550"
    # The volunteered line a live bench puts on the wire, copied out of
    # logs/accept_20261128_185837_handset.log (five identical copies there) rather
    # than shaped by hand: a fixture `T,` this firmware cannot emit would let the
    # pattern that steps over it drift with nobody noticing.
    bench_t = "T,0.20,0.223,0.223,0.223,0.223,0.400,STANDBY,FLAGS0x00,3.64"
    stream = []

    def src(lines):
        it = iter(lines)
        return lambda: next(it, "")

    def run(transcript, family_pat, next_pat):
        win = ReplyWindow(src(transcript))
        noop = lambda l: stream.append(l)
        ok1, seen1 = win.read(re.compile(family_pat[0]), family_pat[1], noop,
                              time.monotonic() + 5.0)
        ok2, seen2 = win.read(re.compile(next_pat), None, noop, time.monotonic() + 5.0)
        return ok1, seen1, ok2, seen2

    checks = []
    hist_pat = r"^(HIST,|ERR no histogram)"
    hist_fam = re.compile(r"^HIST,")

    # 1. The fix: all three cells, telemetry stepped over, the next command's
    #    line pushed back and handed on intact.
    t1 = hist3 + [bench_t, next_answer]
    ok1, seen1, ok2, seen2 = run(t1, (hist_pat, hist_fam), r"^[A-Z0-9_]+=")
    checks.append(("the histogram answer is taken WHOLE: three cells, not one",
                   sum(1 for l in seen1 if l.startswith("HIST,")) == 3,
                   f"{len(seen1)} lines seen: {[l.split(',')[0] for l in seen1]}"))
    checks.append(("the next command's own line is not swallowed by the drain",
                   ok2 and seen2[:1] == [next_answer], f"second window saw {seen2}"))
    checks.append(("nothing is lost or doubled across the two windows",
                   seen1 + seen2 == t1,
                   f"first {len(seen1)} + second {len(seen2)} != {len(t1)} sent"))

    # 2. The defect, reproduced on purpose, so the fix is shown to change
    #    behaviour: with no family the window stops at line 1 and the next
    #    command is answered by a HIST row.
    ok1, seen1, ok2, seen2 = run(hist3 + [next_answer], (hist_pat, None), r"^[A-Z0-9_]+=")
    smeared = bool(seen2) and seen2[0].startswith("HIST,")
    checks.append(("a first-match window DOES smear HIST lines into the next "
                   "command — the thing the family marker exists to stop",
                   smeared and sum(1 for l in seen1 if l.startswith("HIST,")) == 1,
                   f"first window {[l.split(',')[0] for l in seen1]}, second window "
                   f"{[l.split(',')[0] for l in seen2]}"))

    # 3. A refused histogram is one line: the window must end, and the following
    #    answer must arrive untouched.
    ok1, seen1, ok2, seen2 = run(["ERR no histogram", next_answer],
                                 (hist_pat, hist_fam), r"^[A-Z0-9_]+=")
    checks.append(("a refused histogram is one line, and ends the window",
                   ok1 and seen1 == ["ERR no histogram"] and seen2[:1] == [next_answer],
                   f"{seen1} / {seen2}"))

    # 4. An over-broad family is its own bug: it eats the next command's reply.
    ok1, seen1, ok2, seen2 = run(hist3 + [next_answer], (hist_pat, re.compile(r".*")),
                                 r"^[A-Z0-9_]+=")
    checks.append(("a family that matches everything leaves the next command nothing",
                   not seen2 and not ok2,
                   f"second window saw {seen2}: a drain with no boundary is the same "
                   "defect pointing the other way"))

    # 5. The row table itself: the firmware names its in-place listing
    #    formatters (src/comms/cmd_router.h:244-246: `GET *`, `GET D`, `GET HIST`,
    #    SCHEMA). Three of them are asked as one question by this matrix; all
    #    three must carry a family. `GET T`/`GET D` are burst-mode by design and
    #    are deliberately not in this list.
    listings = {"GET *": r"^[A-Z0-9_]+=", "GET SCHEMA": r"^K,", "GET HIST": r"^HIST,"}
    rows = {c[0].strip(): c[3] for c in PROTOCOL_CHECKS if len(c) > 3}
    for cmd, want in listings.items():
        marker = rows.get(cmd, "")
        checks.append((f"{cmd} declares a reply family",
                       str(marker).startswith("family:"),
                       f"row marker: {marker!r}: a multi-line answer without a family "
                       "leaves its tail to answer the next command"))
        if str(marker).startswith("family:"):
            fam = marker.split(":", 1)[1]
            sample = {"GET *": "SP2=0.550",
                      "GET SCHEMA": "K,SP2,F,0.2,4,0,0.21",
                      "GET HIST": "HIST,0,60,10,25,3,40"}[cmd]
            checks.append((f"{cmd}'s family matches a line {cmd} really answers",
                           re.compile(fam).match(sample) is not None,
                           f"family {fam!r} does not match {sample!r}"))
    # 6. No row still asks for the old semantics by name: `collect` drained to
    #    silence and cost a window each, and it could not push back.
    stale = [c[0] for c in PROTOCOL_CHECKS if len(c) > 3 and c[3] == "collect"]
    checks.append(("no row still uses the retired 'collect' marker", not stale,
                   f"still marked collect: {stale}"))

    # 7. The live bench never goes silent: `T,` arrives at 2 Hz forever, so a
    #    drain whose only exit is a quiet line never exits. Found on the first
    #    matrix run over the S3 bridge, 2026-11-28: `GET *` was ANSWERED, the
    #    drain then consumed volunteered telemetry forever, and the repro run sent
    #    no further command for the 801 s it was allowed to live — bytes flowing,
    #    process alive, no phase log ever closed
    #    (`logs/stall_repro_run2_stdout.log`, last console line `[801.32s]`).
    #    Every fixture above runs out and answers `""`, which is exactly the
    #    silence a bench does not do; this one keeps talking. The flood line is
    #    `bench_t` above — the bench's own `T,`, copied from a capture. The flood
    #    is finite only so the suite cannot hang: the bound under test must stop
    #    the drain long before it runs out.
    flood = ["SP1=0.300", "SP2=0.550"] + [bench_t] * 4000 + [next_answer]
    ok1, seen1, ok2, seen2 = run(flood, (r"^[A-Z0-9_]+=", re.compile(r"^[A-Z0-9_]+=")),
                                 r"^[A-Z0-9_]+=")
    over = sum(1 for l in seen1 if UNSOLICITED.match(l))
    checks.append(("a family drain ends on a stream that never goes quiet — the "
                   "S3-bench livelock",
                   # 10 is a literal on purpose: asserting against the class's own
                   # constant would let a mutation just widen the constant and stay
                   # green. The bound itself is the contract.
                   ok1 and over <= 10,
                   f"the drain stepped over {over} volunteered lines: only the skip "
                   "budget or the deadline can end a family on a live bench"))
    checks.append(("and the command after that flood is still answered",
                   ok2 and seen2[-1] == next_answer and seen1 + seen2 == flood,
                   f"second window saw {len(seen2)} lines, last "
                   f"{seen2[-1:] }: nothing may be lost or doubled"))
    return checks


def selftest():
    """Proves the surface-log graders can fail.

    Every case below is a log that lies in exactly one known way. A case that
    does NOT fail means the check is decoration — which is the lesson already
    learned on this bench, when a check was hunting an `E,` row its own line
    filter was throwing away and could never have passed. Run it after touching
    the graders or the export format (`hw_accept.py --selftest`). The depth
    announce's own lying boots are in `selftest_depth()`, run alongside.
    """
    # (what the log claims, the bytes, the expected verdict: (failing check name,
    #  or None for "must pass clean"))
    cases = [
        ("a complete, honest log", GOOD_SURFACE_CSV, None),
        ("header says 3 records, 2 arrive (truncated file)",
         GOOD_SURFACE_CSV.replace("SURFACE,1,2,3,4", "SURFACE,1,3,3,4"),
         "identity row's count matches"),
        ("identity row missing entirely",
         GOOD_SURFACE_CSV.split("\n", 1)[1], "identity row"),
        ("seq range narrower than the rows (header describes another file)",
         GOOD_SURFACE_CSV.replace("SURFACE,1,2,3,4", "SURFACE,1,2,4,4"),
         "seq range spans the rows"),
        ("this run wrote nothing (seq did not move)",
         GOOD_SURFACE_CSV, "check-hi"),
        ("the calibration record disagrees with the wire (raw0 wrong)",
         GOOD_SURFACE_CSV.replace("1514,1519,1528", "1414,1519,1528"),
         "same numbers as the wire"),
        ("the verdict code disagrees with the wire (OK vs warm-up)",
         GOOD_SURFACE_CSV.replace(",7,cal_air,2,", ",7,cal_air,3,"),
         "same numbers as the wire"),
        ("a pre-dive failure filed as a pass",
         GOOD_SURFACE_CSV.replace(",16,pre_dive_fail,4,", ",13,pre_dive_pass,4,"),
         "record is in the export"),
        ("an appended id with no name (silently unlabelled)",
         GOOD_SURFACE_CSV.replace(",7,cal_air,", ",21,cal_air,"),
         "unknown ids are named"),
        ("a record with no clock rendered as a date",
         GOOD_SURFACE_CSV.replace(",1795518150,2026-11-24T11:02:30Z,", ",0,2026-11-24T11:02:30Z,"),
         "says so"),
        ("the console never said it wrote the record",
         GOOD_SURFACE_CSV, "console-noise"),
        ("the console's record number is a different record",
         GOOD_SURFACE_CSV, "console-seq"),
        ("the console claims record 0: the stamped seq never came back",
         GOOD_SURFACE_CSV, "console-seq0"),
    ]
    checks = []
    for name, body, expect in cases:
        rows_want = [("cal_air", CAL_WANT), ("pre_dive_fail", [("value", 6, 4)])]
        hi = 2 if expect != "check-hi" else 4
        if expect == "console-seq":
            # Same verdict, wrong seq: the join must be on the number, not on "some
            # record was written this run".
            console = ["surface: record 9 id 7 value 2", "surface: record 4 id 16"]
        elif expect == "console-seq0":
            # The real 2026-11-26 metal regression: the bounded surface gate
            # appended its own pending copy, the store stamped the seq on THAT
            # copy, and the caller printed its pre-append record - every
            # record written, the console claiming record 0 for all of them.
            console = ["surface: record 0 id 7 value 2", "surface: record 0 id 16"]
        else:
            console = GOOD_CONSOLE if expect != "console-noise" else []
        if expect == "check-hi":
            rows_want = [("cal_air", CAL_WANT)]
        if expect == "console-noise":
            rows_want = [("cal_air", CAL_WANT)]
        got = grade_surface_csv(body, rows_want, hi, console)
        failed = [c[0] for c in got if not c[1]]
        if expect is None:
            checks.append((name, not failed, f"unexpected failures: {failed}"))
            continue
        if expect == "check-hi":
            checks.append((name, "this run's records joined the log" in "".join(failed),
                           f"failures seen: {failed}"))
        elif expect in ("console-noise", "console-seq", "console-seq0"):
            checks.append((name, any("console reported writing" in f for f in failed),
                           f"failures seen: {failed}"))
        else:
            checks.append((name, any(expect in f for f in failed),
                           f"no failure matching {expect!r}; failures seen: {failed}"))
    return checks


def grade_stream(stream):
    """The facts the controller volunteers, graded over everything it sent.

    A command's reply only ever speaks for the command. The air calibration's
    verdict is a fact about the machine, so it has to arrive UNPROMPTED, as an
    `E,<t>,7,<code>` row: that is what a handset with no query in flight sees, and
    it is the half of the story the `OK CAL AIR` receipt cannot tell."""
    checks = []
    ev = next((l for l in stream if re.match(r"^E,\d+,7,3$", l)), None)
    checks.append(("the warm-up refusal was streamed unprompted as E,<t>,7,3",
                   ev is not None,
                   "no `E,*,7,3` row in the stream: the verdict stayed on the console"
                   if ev is None else ev))
    # And nothing claimed a calibration happened: id 7 with value 2 (CALAIR_OK)
    # would be a lie on a board whose cells were refused for warm-up.
    lie = next((l for l in stream if re.match(r"^E,\d+,7,2$", l)), None)
    checks.append(("no CALAIR_OK row for a calibration that was refused",
                   lie is None, lie or ""))

    # The clamp verdict (id 19). The matrix inverts the GF pair at the surface
    # and restores it, so the honest board streams EXACTLY one `E,<t>,19,32`:
    # GF_ORDER alone. Zero rows means the verdict is console-only again — the
    # hole the id exists to close. More than one means the applier or the
    # latch is publishing for SETs that needed no clamp. A wrong mask is the
    # loudest failure: it names an ordering the applier never enforced.
    clamp_rows = [l for l in stream if re.match(r"^E,\d+,19,\d+$", l)]
    checks.append(("the clamp verdict arrived unprompted as E,<t>,19,<mask>",
                   len(clamp_rows) == 1,
                   f"{len(clamp_rows)} id-19 rows in the stream "
                   f"{clamp_rows[:4]} — want exactly 1: the matrix clamps exactly once"
                   if len(clamp_rows) != 1 else clamp_rows[0]))
    if len(clamp_rows) == 1:
        mask = int(clamp_rows[0].split(",")[3])
        named = [nm for bit, nm in CLAMP_BITS if mask & bit]
        checks.append(("the clamp row names GF_ORDER and nothing else",
                       mask == 0x20,
                       f"mask {mask} names {named or 'nothing'}, want exactly "
                       f"[GF_ORDER]: the matrix inverts the GF pair and nothing else"))
    return checks


# ApplyClamp's bits, in one place: the grader and its lying captures both
# read this, so a mask named here and a mask printed there cannot drift.
CLAMP_BITS = [(1, "ALLO_HIGH"), (2, "CUTOFF"), (4, "PULSE_MS"), (8, "DUTY"),
              (16, "SP_ORDER"), (32, "GF_ORDER"), (64, "BAND_ORDER")]


def selftest_stream():
    """The event-stream grader, proven on lying captures.

    The honest capture is the exact shape the matrix produces: one warm-up
    refusal (E,*,7,3), no false OK, and exactly one clamp verdict naming
    GF_ORDER. Each lie below is a regression that already happened somewhere
    in this repo's history — a verdict admitted only on the console."""
    honest = ["T,0.05,0.212,0.208,0.210,0.210,0.400,STANDBY,FLAGS0x00,12.40",
              "E,12,7,3",
              "E,31,19,32"]
    out = []
    res = grade_stream(honest)
    out.append(("honest stream passes", all(ok for _, ok, _ in res),
                [d for _, ok, d in res if not ok]))

    # The verdict back to being console-only: the SET succeeds, nothing arrives.
    res = grade_stream([l for l in honest if not l.startswith("E,31,19")])
    out.append(("a stream WITHOUT the clamp row is refused",
                any(not ok for _, ok, _ in res), ""))

    # A second row: the latch publishing for un-clamped SETs, or the applier
    # clamping a pair that was never inverted.
    res = grade_stream(honest + ["E,44,19,0", "E,58,19,32"])
    out.append(("two clamp rows (one for a SET that clamped nothing) is refused",
                any(not ok for _, ok, _ in res), ""))

    # The mask lying: naming SP_ORDER while the GF pair was the inversion.
    res = grade_stream(["E,12,7,3", "E,31,19,16"])
    out.append(("a clamp row naming the WRONG ordering is refused",
                any(not ok for _, ok, _ in res), ""))

    # The mask lying quietly: GF_ORDER plus a bit the applier never set.
    res = grade_stream(["E,12,7,3", "E,31,19,33"])
    out.append(("a clamp row with an extra invented bit is refused",
                any(not ok for _, ok, _ in res), ""))
    return out


def grade_watch(lines, boot_lines=None, cal_lines=None):
    # `boot_lines` contributes to the stack-headroom scan only: HB beats every
    # 5 s, so the boot capture is usually where the headroom numbers live. A
    # missing tripwire is a fault wherever it is missing.
    """Grade what the console said WHILE the commands were being sent.

    The boot checks only see the boot. A command that panics the board resets
    it, and every later command then fails for a reason the report does not
    state. So the console stays open through the protocol phase and is graded
    here: any crash marker is a FAIL, and the stack headroom HB reports
    (stk= / u2stk=) must stay above the firmware's own warning level.
    """
    checks = []
    for marker, meaning in CRASH_MARKERS:
        hit = next((l for l in lines if marker in l), None)
        checks.append((f"no {meaning} during command phase", hit is None, hit or ""))

    safe_hb = next((l for l in lines if re.match(r"^HB st=3", l)), None)
    checks.append(("no SAFE-state heartbeat during command phase",
                   safe_hb is None, safe_hb or ""))

    panic_reset = next((l for l in lines if "reset reason: PANIC" in l), None)
    checks.append(("reset reason stayed clean", panic_reset is None, panic_reset or ""))

    # The matrix opens and closes exactly one dive with the radio up on entry, so
    # the console must show BOTH halves of the auto-restore: DIVE START drops the
    # radio (PLAN 9: milliamps, and an antenna on the loop) and DIVE END puts back
    # what it took. Nothing else in the sequence can produce these lines — the
    # mid-dive `WIFI ON` is refused. This is protocol-visible: the handset answers
    # "OK DIVE" either way, so a dive that ended with the radio down would only be
    # discovered by a diver surfacing to a web page that is not there.
    downs = sum(1 for l in lines if l.startswith("web: radio off"))
    ups = sum(1 for l in lines if l.startswith("web: AP "))
    window = f"{len(lines)} console lines were captured in the watch window"
    # The policy's own line per transition (2026-11-26). `web: radio off` counts
    # the teardown, but since the radio went policy-driven the reason matters:
    # the matrix opens and closes exactly one dive with the radio up on entry,
    # so the machine must have printed BOTH halves naming the dive. A radio
    # that came back for the wrong reason, or a transition no line marks, is
    # the family of drift these paired witnesses exist to catch.
    pd_ = sum(1 for l in lines if l.startswith("wifi: radio down"))
    pu_ = sum(1 for l in lines if l.startswith("wifi: radio up"))
    checks.append(("the radio policy printed the dive's transitions",
                   pd_ >= 1 and pu_ >= 1,
                   f"{pd_} down / {pu_} up 'wifi:' lines; {window} — the handset "
                   f"answers OK DIVE either way; this line is where the reason lives"))
    checks.append(("DIVE START took the radio down", downs >= 1,
                   f"{downs} 'web: radio off' lines; {window}"))
    checks.append(("DIVE END brought the radio back", ups >= downs and ups >= 1,
                   f"{downs} down / {ups} up; {window} — a surface with no AP is "
                   f"a fault the handset reply cannot show"))
    # The matrix asked for a CAL AIR inside the warm-up gate, so the console must
    # have said REFUSED. Two records of one event, and they have to agree: the
    # protocol answered CALAIR,3 (handset side) and this is the machine's own
    # console. A console that says "sampling" while the protocol says "refused" is
    # the drift this whole change exists to kill.
    window = cal_lines if cal_lines is not None else lines
    refused = next((l for l in window if l.startswith("cal air: REFUSED")), None)
    checks.append(("the console refused the warm-up calibration too",
                   refused is not None,
                   refused or "no 'cal air: REFUSED' line — the protocol's refusal has "
                              "no counterpart in the machine's own output"))
    started = next((l for l in window if l.startswith("cal air: sampling")), None)
    checks.append(("a refused calibration did not also start sampling",
                   started is None, started or ""))

    for marker, meaning in (("softAP failed", "AP would not come up"),
                            ("captive DNS failed to bind", "captive portal dead")):
        hit = next((l for l in lines + list(boot_lines or []) if marker in l), None)
        checks.append((f"no {marker!r} ({meaning})", hit is None, hit if hit else ""))

    # "esp_wifi_get_mac failed with 12289" is NOT a check. ESP-IDF's own
    # esp_wifi_stop() logs it while taking an AP down — confirmed by injecting
    # the firmware's pre-fix call order and watching the marker land in the same
    # place either way, and by the Arduino WiFi library never calling
    # esp_wifi_get_mac at all. A check that fires on healthy hardware is worse
    # than none: the next real fault gets waved through. What is checked instead
    # is the consequence that matters and is observable — the AP name, which must
    # carry the MAC tail so a phone that joined before a dive can rejoin after.
    ap_names = [l.split('"')[1] for l in lines + list(boot_lines or [])
                if l.startswith("web: AP ") and len(l.split('"')) > 1]
    tails = [n for n in ap_names if re.search(r"-[0-9A-Fa-f]{4}$", n)]
    checks.append(("AP name carries a MAC tail", bool(ap_names) and len(tails) == len(ap_names),
                   f"names seen: {sorted(set(ap_names)) or ['(no AP line)']}"))
    idf_noise = sum(1 for l in lines + list(boot_lines or [])
                    if "esp_wifi_get_mac failed" in l)
    if idf_noise:
        print(f"  note: {idf_noise}x ESP-IDF stop-path MAC line (expected on a "
              f"healthy board, see BENCH.md)")

    worst = {"stk": None, "u2stk": None}
    for l in list(lines) + list(boot_lines or []):
        m = HB_STACK_RE.search(l)
        if not m:
            continue
        for key, val in zip(("stk", "u2stk"), m.groups()):
            if val is None:
                continue
            v = int(val)
            worst[key] = v if worst[key] is None else min(worst[key], v)
    if worst["stk"] is None:
        # The tripwire is not optional: without it a stack regression is
        # invisible until it reboots the controller mid-dive.
        checks.append(("HB reports stack headroom (stk=)", False,
                       "no stk= in any HB line: the tripwire is missing"))
    else:
        checks.append((f"stk= headroom >= 512 (worst {worst['stk']})",
                       worst["stk"] >= 512, "see HB lines"))
    if worst["u2stk"] is None:
        checks.append(("HB reports uart2 headroom (u2stk=)", False,
                       "no u2stk= in any HB line: the task that crashed is unwatched"))
    else:
        checks.append((f"u2stk= headroom >= 512 (worst {worst['u2stk']})",
                       worst["u2stk"] >= 512, "see HB lines"))

    # H2: the stall margin and the lost-record counters, same grading shape as
    # stk= — a missing field is itself a FAIL (the firmware must always tell),
    # then the threshold. worst= resets every beat (per-beat margin); the
    # skip counters are cumulative since boot, so the maximum seen is the
    # current total.
    worst_cycle = None
    lost = {"logskip": None, "surfskip": None, "contskip": None}
    for l in list(lines) + list(boot_lines or []):
        m = HB_CYCLE_RE.search(l)
        if not m:
            continue
        w, lr, ls, cs = (int(x) for x in m.groups())
        worst_cycle = w if worst_cycle is None else max(worst_cycle, w)
        for key, val in zip(("logskip", "surfskip", "contskip"), (lr, ls, cs)):
            lost[key] = val if lost[key] is None else max(lost[key], val)
    if worst_cycle is None:
        checks.append(("HB reports ControlTask cycle margin (worst=)", False,
                       "no worst= in any HB line: the 1500 ms stall budget is "
                       "assumed, not measured"))
    else:
        checks.append((f"worst= cycle margin <= {HB_WORST_CYCLE_MS} ms "
                       f"(worst {worst_cycle}, reboot budget "
                       f"{FIRMWARE_STALL_BUDGET_MS or 'UNREADABLE from defaults.h'} "
                       f"read from defaults.h, measured TWDT expiry "
                       f"{TWDT_MEASURED_SPREAD_MS} ms)",
                       worst_cycle <= HB_WORST_CYCLE_MS, "see HB lines"))
    for key, what in (("logskip", "dive-file row"), ("surfskip", "surface record"),
                      ("contskip", "continuity snapshot")):
        if lost[key] is None:
            checks.append((f"HB reports lost {what} count ({key}=)", False,
                           f"no {key}= in any HB line: a skipped record would "
                           "be a hole in the dive nobody is told about"))
        else:
            checks.append((f"{key}= lost {what}s == 0 (max {lost[key]})",
                           lost[key] == 0, "see HB lines"))

    # Batch D: the config generation pair. Graded like every other HB field —
    # a missing field is itself the failure, because the claim it carries would
    # otherwise go back to being a comment. Three lies beyond that, each a
    # different fault in control/configbox.h's contract:
    #   * chain > published  — the chain reports a generation no one published,
    #     which is only possible if something other than the box is deciding it;
    #   * published - chain > 1 — a control cycle has stopped reading. The bound
    #     is one because a SET landing between ControlTask's read and the next
    #     cycle's pin fuse is normal; two behind is a cycle that did not run, and
    #     that is the failure the supervisor reboots for, one beat earlier;
    #   * both halves equal to a stale constant on every beat after a SET — the
    #     counter not moving is the same as the field not being there.
    cfggen = []
    for l in list(lines) + list(boot_lines or []):
        m = HB_CFGGEN_RE.search(l)
        if m:
            cfggen.append((int(m.group(1)), int(m.group(2))))
    if not cfggen:
        checks.append(("HB reports the config generation pair (cfggen=)", False,
                       "no cfggen= in any HB line: 'the chain runs the rules the last SET "
                       "produced' is again an assumption, not a reading"))
    else:
        behind = max(p - c for p, c in cfggen)
        ahead = max(c - p for p, c in cfggen)
        checks.append((f"cfggen= chain never ahead of published (worst {ahead})",
                       ahead <= 0, "the chain reported a generation nobody published"))
        checks.append((f"cfggen= chain at most one generation behind (worst {behind})",
                       behind <= 1, "a control cycle stopped reading the applied rules: "
                                    "the supervisor reboot for a stalled cycle, visible here "
                                    "one beat earlier"))

    # Batch E: the coil rail (see HB_RAIL_RE for why the VALUE is not graded).
    rail = []
    for l in list(lines) + list(boot_lines or []):
        m = HB_RAIL_RE.search(l)
        if m:
            rail.append((float(m.group(1)), int(m.group(2))))
    if not rail:
        checks.append(("HB reports the coil rail (vrail=)", False,
                       "no vrail= in any HB line: the 12 V the valve actually runs "
                       "on is again invisible on the bench"))
    else:
        worst_rs = max(rs for _, rs in rail)
        checks.append((f"railstate= within the published 0-3 (worst {worst_rs})",
                       worst_rs <= 3,
                       "railstate outside 0-3: HB is reporting a verdict this "
                       "firmware cannot produce"))
        liars = [v for v, rs in rail if rs == 3 and v > 0.004]
        checks.append((f"not-sensed rail reports no voltage ({len(liars)} lying lines)",
                       not liars,
                       "railstate=3 says 'there is no reading' while vrail= carries "
                       "a number: the two halves of one field contradict each other"))
    return checks


def selftest_control_margin():
    """The ControlTask-margin and lost-record grader, proven on lying captures.

    The honest case is what the fixed firmware prints: per-beat cycle spans
    far below the 800 ms line and both skip counters still at zero. Every lie
    is a shape a regression takes: the fields vanishing (an HB change drops
    the tail — the margin goes back to being folklore), the margin blown
    without a reboot (the control loop degrading toward the cliff the
    supervisor would fall off), a counter stuck visible-but-nonzero (the
    store losing rows under a log export), and a partially-appended tail
    (one field edited out mid-format, the append-only rule broken in
    silence). Each must redden the check named for it.
    """
    def hb(worst, lskip, sskip, cont=0, tail="stk=1408 u2stk=1536", cfggen="cfggen=4/4",
           rail="vrail=0.00 railstate=3"):
        return (f"HB st=1 c0=2050 c1=2049 c2=2051 fresh=111 smp=120 i2cerr=0 "
                f"rec=0 vbat=12.40 stale=0 batlow=0 vote=1.320 sp=1.30 "
                f"al=0x0000 mode=0 pulses=4 sol=0 ctrl=1 {tail}"
                + (f" worst={worst} logskip={lskip} surfskip={sskip} contskip={cont}"
                   if worst is not None else "")
                + (f" {cfggen}" if cfggen else "")
                + (f" {rail}" if rail else ""))

    def new_leg(checks):
        keep = ("worst=", "logskip=", "surfskip=", "contskip=", "cfggen=",
                "(worst=)", "(logskip=)", "(surfskip=)", "(contskip=)",
                "vrail=", "railstate=", "not-sensed")
        return [c for c in checks if any(s in c[0] for s in keep)]

    cases = [
        # name, HB lines, expected failing check (None = everything must pass)
        ("an honest run: 45 ms and 120 ms beats, nothing lost",
         [hb(45, 0, 0), hb(120, 0, 0)], None),
        ("pre-H2 firmware: the tail ends at u2stk=, the margin is folklore",
         [hb(None, None, None)], "(worst=)"),
        ("the margin blown: a beat at 1420 ms, one supervisor check from reboot",
         [hb(45, 0, 0), hb(1420, 0, 0)], "worst= cycle margin"),
        ("one millisecond past the line: 801 (the threshold is not a suggestion)",
         [hb(801, 0, 0)], "worst= cycle margin"),
        ("a dive-file row lost to the busy store and said out loud",
         [hb(45, 3, 0)], "logskip= lost"),
        ("a surface verdict record lost",
         [hb(45, 0, 2)], "surfskip= lost"),
        ("the cumulative counter climbing: 0 then 4 rows gone",
         [hb(45, 0, 0), hb(60, 4, 0)], "logskip= lost"),
        ("a continuity snapshot lost: the crash window the next dive seeds from grew",
         [hb(45, 0, 0, 3)], "contskip= lost"),
        ("the H1 tail truncated: contskip= edited out of the format",
         [(lambda h: h[:h.index(" contskip=")])
          (f"HB st=1 c0=2050 c1=2049 c2=2051 fresh=111 smp=120 i2cerr=0 "
           f"rec=0 vbat=12.40 stale=0 batlow=0 vote=1.320 sp=1.30 "
           f"al=0x0000 mode=0 pulses=4 sol=0 ctrl=1 stk=1408 u2stk=1536 "
           f"worst=45 logskip=0 surfskip=0 contskip=0")],
         "(contskip=)"),
        ("the append-only rule broken: worst= there, the counters edited out",
         [f"HB st=1 c0=2050 c1=2049 c2=2051 fresh=111 smp=120 i2cerr=0 "
          f"rec=0 vbat=12.40 stale=0 batlow=0 vote=1.320 sp=1.30 "
          f"al=0x0000 mode=0 pulses=4 sol=0 ctrl=1 stk=1408 u2stk=1536 worst=45"],
         "(logskip=)"),
        # Batch D: the config generation pair. Same grading shape — a missing
        # field fails by itself, then the pair's own arithmetic.
        ("pre-batch-D firmware: no cfggen=, so the config skew is folklore again",
         [hb(45, 0, 0, cfggen=None)], "(cfggen=)"),
        ("one behind is NORMAL: a SET landed between the read and the pin fuse",
         [hb(45, 0, 0, cfggen="cfggen=5/4")], None),
        ("two behind: a control cycle stopped reading the applied rules",
         [hb(45, 0, 0), hb(50, 0, 0, cfggen="cfggen=7/5")], "cfggen= chain at most one"),
        ("the chain ahead of published: it claims a generation nobody published",
         [hb(45, 0, 0, cfggen="cfggen=4/9")], "cfggen= chain never ahead"),
        # Batch E: the coil rail. Default fixture is today's flashed truth —
        # vrail=0.00 railstate=3, the review item (1 P9) as a number.
        ("pre-batch-E firmware: no vrail=, the coil rail is invisible again",
         [hb(45, 0, 0, rail=None)], "(vrail=)"),
        ("a sensed, in-band rail: the honest post-fit line must pass too",
         [hb(45, 0, 0, rail="vrail=11.90 railstate=0")], None),
        ("railstate outside the published range: a verdict the firmware cannot produce",
         [hb(45, 0, 0, rail="vrail=11.90 railstate=9")], "railstate= within the published"),
        ("railstate=3 says 'no reading' while vrail= reports 12.10: one field lies",
         [hb(45, 0, 0, rail="vrail=12.10 railstate=3")], "not-sensed rail reports no voltage"),
    ]
    checks = []
    for name, hb_lines, want in cases:
        got = new_leg(grade_watch(list(hb_lines)))
        failed = [c[0] for c in got if not c[1]]
        if want is None:
            # An honest capture grades eight checks in this leg — the four
            # thresholds, the two cfggen= pair rules and the two rail shape/
            # coherence rules; the "HB reports ... (field=)" checks only appear
            # when a field is missing.
            checks.append((f"control margin: {name}", len(got) == 8 and not failed,
                           f"expected the eight threshold checks, none failed: "
                           f"{len(got)} seen, {failed}"))
        else:
            checks.append((f"control margin: {name}",
                           any(want in f for f in failed),
                           f"no failure matching {want!r}; failures seen: {failed}"))
    return checks


def selftest_soak_contract():
    """Pin the two copies of the HB status-line regex against each other.

    soak_logger.py (24 h captures) and hw_accept.py (acceptance grading) each
    carry their own HB_RE; the comment in each names the other as the same
    contract, and until now that was only a comment. If they drift — one
    learns the batch-E rail tail, the other does not; one mis-anchors the
    appended fields — the soak keeps logging and the grader keeps PASSing
    lines the logger never counted, and nobody finds out until a soak's
    summary is read. Both lines below are the CURRENT firmware format:
    everything before vrail= is the pre-batch-E line, byte-identical.
    """
    checks = []
    try:
        import soak_logger
    except Exception as exc:  # the join itself must not be skippable silently
        return [("soak_logger HB_RE is importable for the join", False,
                 f"import failed: {exc}")]
    line_e = ("HB st=1 c0=2050 c1=2049 c2=2051 fresh=111 smp=120 i2cerr=0 "
              "rec=0 vbat=3.64 stale=0 batlow=0 vote=1.320 sp=1.30 "
              "al=0x0010 mode=0 pulses=4 sol=0 ctrl=1 stk=1408 u2stk=1536 "
              "worst=45 logskip=0 surfskip=0 contskip=0 cfggen=4/4 "
              "vrail=11.90 railstate=0")
    line_pre = line_e.split(" vrail=")[0]
    for name, line in (("batch-E line (rail tail present)", line_e),
                       ("pre-batch-E line (no rail tail)", line_pre)):
        mh, ms = HB_RE.match(line), soak_logger.HB_RE.match(line)
        checks.append((f"both HB_RE copies parse the {name}",
                       bool(mh and ms),
                       f"hw_accept={'ok' if mh else 'MISS'} "
                       f"soak_logger={'ok' if ms else 'MISS'}"))
        if mh and ms:
            checks.append((f"shared fields agree on the {name}",
                           mh.groups()[:13] == ms.groups()[:13],
                           f"{mh.groups()} vs {ms.groups()[:13]}"))
    # The rail tail specifically: soak_logger must EXTRACT it, not merely
    # match past it — the tail is appended at the end of a long line, so a
    # regex that only tolerates `vrail=` right after `batlow=` matches the
    # line and still reports no rail, which is a silent data loss exactly
    # the size of §1 P9. The pre-batch-E line must report no tail instead
    # of failing the line.
    m = soak_logger.HB_RE.match(line_e)
    checks.append(("soak_logger extracts vrail/railstate from the batch-E line",
                   bool(m) and m.group(14) == "11.90" and m.group(15) == "0",
                   "the appended tail matched but was not captured"))
    m2 = soak_logger.HB_RE.match(line_pre)
    checks.append(("soak_logger reports no rail tail on a pre-batch-E line",
                   bool(m2) and m2.group(14) is None,
                   "a pre-batch-E line parsed with a phantom rail value"))
    return checks


def _read_line(ser, seen, log, t0):
    line = ser.readline().decode(errors="replace").strip()
    if line:
        seen.append(line)
        log.append(f"[{time.time() - t0:7.2f}s] << {line}")
    return line


def _await_burst(ser, pat, seen, log, send, t0):
    """True only if the controller ANSWERED, not merely kept streaming.

    Waits for a streamed line as a cadence anchor, sends, then looks for a
    matching line within BURST_WINDOW_S of the send. See the constants above for
    why the window separates reply from unsolicited traffic.
    """
    deadline = time.time() + BURST_ANCHOR_S
    anchored = False
    while time.time() < deadline:
        line = _read_line(ser, seen, log, t0)
        if not line:
            break
        if pat.match(line):
            anchored = True
            break
    if not anchored:
        log.append(f"[{time.time() - t0:7.2f}s] (no {pat.pattern!r} line to "
                   f"anchor on: the downlink is dead)")
        return False

    send()
    sent_at = time.time()
    log.append(f"[{sent_at - t0:7.2f}s] >> (burst resend)")
    while time.time() - sent_at < BURST_REPLY_S:
        line = _read_line(ser, seen, log, t0)
        if not line:
            break
        if pat.match(line):
            if time.time() - sent_at <= BURST_WINDOW_S:
                return True
            log.append(f"[{time.time() - t0:7.2f}s] (matching line arrived too "
                       f"late to be the reply)")
            return False
    return False


# Lines the controller volunteers without being asked — the telemetry rows the
# stream graders live on, and the heartbeat. They are evidence, but they are
# never the tail of a reply, so a reply reader may step over them.
UNSOLICITED = re.compile(r"^(?:[TDE],|HB )")


class ReplyWindow:
    """One command's answer, read from a line source, with the multi-line rule
    stated HERE instead of in a per-row comment.

    Three listing formatters answer with MANY lines written by ONE `write()`:
    `GET *` (one `KEY=` per setting), `GET SCHEMA` (one `K,` per key) and
    `GET HIST` (one `HIST,` per cell — `src/control/cellhist.h:62`). A window
    that stops at the first match leaves the rest of that single reply sitting
    in the port's buffer; the next command is sent, and those leftover lines
    arrive inside the NEXT command's window and are graded as its answer. That
    is not a theoretical risk: it was found once on `GET *` (the `WIFI_*` keys
    sit at the end of the table, and the ruler grader was reading an amputated
    line list), and `GET HIST` had the same exposure today.

    So: a row may declare a `family:` regex. The window keeps draining while the
    family continues, steps over volunteered telemetry, and PUSHES BACK the first
    line that belongs to neither — pushback is what makes draining safe, because
    without it the drain would eat the next command's first line. Rows with no
    family keep first-match semantics: for them, everything after the match IS
    the next command's answer.

    The drain must be bounded TWICE, because "no more family lines" and "no more
    lines" are not the same test on a live bench. That was not theory: the first
    matrix run over the S3 bridge (2026-11-28) livelocked on `GET *` — the drain
    looped on `T,`/`D,` arriving at 2 Hz, which never produce the silence gap
    that was its only exit, and the phase never ended. Host fixtures cannot
    reproduce that: a fixture runs out and answers `""`. So:
      * the drain obeys the window's deadline, and
      * it steps over at most FAMILY_SKIP_MAX consecutive non-family lines.
    A family is one `write()` — microsecond gaps on the wire; at 2 Hz telemetry
    a real family can interleave at most a line or two. Ten consecutive
    volunteered lines mean the family is over and the stream is just talking;
    stop, and the stop-line goes to pushback like any other non-family line.

    `GET T`/`GET D` are deliberately not family-driven: they are asked in burst
    mode, where the continuous `T,`/`D,` stream is the cadence anchor and its
    "leftovers" are the point. See _await_burst.
    """

    FAMILY_SKIP_MAX = 10

    def __init__(self, source):
        self.source = source          # callable -> line, "" for silence
        self.pushed = []              # read-too-early, owed to the next window

    def _line(self):
        return self.pushed.pop(0) if self.pushed else self.source()

    def read(self, pat, family, on_line, deadline):
        """Returns (matched, lines). `lines` excludes anything pushed back."""
        seen = []
        ok = False
        while time.monotonic() < deadline:
            line = self._line()
            if not line:
                break
            seen.append(line)
            on_line(line)
            if not pat.match(line):
                continue
            ok = True
            if family is None:
                break
            skipped = 0
            while time.monotonic() < deadline:
                nxt = self._line()
                if not nxt:
                    return ok, seen
                if family.match(nxt):
                    seen.append(nxt)
                    on_line(nxt)
                    skipped = 0
                    continue
                if UNSOLICITED.match(nxt) and skipped < self.FAMILY_SKIP_MAX:
                    seen.append(nxt)
                    on_line(nxt)
                    skipped += 1
                    continue
                self.pushed.append(nxt)   # not ours: hand it to the next window
                return ok, seen
            return ok, seen            # deadline inside the drain: the family
                                       # was never going to end. Take what we have.
        return ok, seen


def run_protocol(port, baud, log, settle=1.5, stream=None, pairs=None):
    """Open the handset adapter and run PROTOCOL_CHECKS. No pyserial import
    at module level: the boot/replay path must work without a handset.

    `stream`, if given, collects EVERY line the controller sent, including the
    unsolicited T,/D,/E, rows. Those are the evidence for facts the controller
    volunteers — a refused calibration, a clamp that engaged — which no command's
    own reply can show (`OK CAL AIR` is a receipt, not a verdict).

    `pairs`, if given, collects (command, [reply lines]) in the order they were
    sent. The stream alone cannot answer "did that refused WRITE land?": the reply
    to `GET WATER_DENSITY` is a bare `WATER_DENSITY=…`, and on the stream it lands
    among unsolicited rows that mean something else. Pairing keeps each cause next
    to its effect, which is what `grade_density_lock` needs.

    `settle`: opening the port resets the board behind it. On a plain USB-UART
    adapter that costs nothing; on a second ESP32 running handset/bridge_esp32
    (the bench stand-in for an adapter) the open is a reset pulse and the ROM
    log arrives, so wait for it to boot and flush before the first command.
    A check that produced NO line at all is sent once more: the first bytes
    after a reset can be swallowed by the bridge's own bootloader."""
    import serial  # pipx platformio venv has it

    ser = serial.Serial(port, baud, timeout=1.0)
    # ONE window reader for the whole run, so a line read early by one check is
    # handed to the next instead of being re-read, re-graded or lost.
    window = ReplyWindow(lambda: ser.readline().decode(errors="replace").strip())
    results = []
    # Both logs are stamped from the same wall clock: the handset log says what
    # was sent and when, the console watch says what the board printed and when.
    # Without that alignment "the console never said the radio went down" cannot
    # be placed next to the DIVE START that should have caused it.
    t0 = T_ORIGIN
    try:
        time.sleep(settle)
        ser.reset_input_buffer()
        # reset_input_buffer() only clears OUR side. Un-terminated bytes sitting
        # in the controller's line buffer are parsed together with the next
        # command: verified on the bench 2026-11-23 by sending `XY` with no
        # newline and then `AUTO\r\n`, which came back as `ERR unknown command`
        # (it saw `XYAUTO`) and recovered on the following line. On this bench
        # those bytes arrive by themselves: opening the handset port resets the
        # bridge ESP32, and its boot banner goes out its TX straight into the
        # controller. So flush the controller's buffer, and report what it drew,
        # because a bare newline answering `ERR unknown command` is proof there
        # were stray bytes pending - without it the first check of the run can
        # fail for a reason that has nothing to do with the firmware.
        ser.write(b"\r\n")
        ser.flush()
        flush_end = time.time() + 0.6
        drew = []
        while time.time() < flush_end:
            l = ser.readline().decode(errors="replace").strip()
            if l and not l.startswith(("T,", "D,", "E,")):
                drew.append(l)
        if drew:
            log.append(f"[{time.time() - t0:7.2f}s] (flushed stray bytes from the "
                       f"controller's line buffer: {drew})")
        for check in PROTOCOL_CHECKS:
            cmd, want, why = check[0], check[1], check[2]
            burst = len(check) > 3 and check[3] == "burst"
            # `family:<regex>`: the reply is many lines from one write(); drain
            # the family and push back what follows it. Rows without one are
            # single-line answers and keep first-match semantics.
            family = None
            if len(check) > 3 and check[3].startswith("family:"):
                family = re.compile(check[3].split(":", 1)[1])
            dwell = 0.0
            if len(check) > 3 and check[3].startswith("dwell:"):
                dwell = float(check[3].split(":", 1)[1])
            pat = re.compile(want)
            seen = []
            ok = False
            for attempt in (1, 2):
                log.append(f"[{time.time() - t0:7.2f}s] >> {cmd.strip()}")
                ser.write((cmd.strip() + "\r\n").encode())
                ser.flush()
                if burst:
                    payload = (cmd.strip() + "\r\n").encode()
                    ok = _await_burst(ser, pat, seen, log,
                                      lambda p=payload: (ser.write(p), ser.flush()), t0)
                    if stream is not None:
                        stream.extend(seen)
                else:
                    ok, seen = window.read(
                        pat, family,
                        lambda l: (stream.append(l) if stream is not None else None,
                                   log.append(f"[{time.time() - t0:7.2f}s] << {l}")),
                        time.monotonic() + 3000.0)
                if ok or seen:
                    break
                log.append(f"[{time.time() - t0:7.2f}s] (no line at all for "
                           f"{cmd!r}: retrying once)")
            if pairs is not None:
                pairs.append((cmd.strip(), list(seen)))
            got = "; ".join(seen[-4:]) if seen else "(no reply in 3 s)"
            results.append((cmd if len(cmd) < 24 else cmd[:20] + "...", ok,
                            f"want {want}, got {got} — {why}"))
            if dwell:
                log.append(f"[{time.time() - t0:7.2f}s] (holding {dwell:.1f}s: "
                           f"deferred actions land in this window)")
                time.sleep(dwell)
        ser.reset_input_buffer()
    finally:
        ser.close()
    return results


TELEM = re.compile(r"^[TDE],")


def _send_and_drain(ser, payload, t0, log, window=1.0):
    """Send one payload, then collect every NON-telemetry reply line that
    arrives in the window. Silence is a result here: for a dropped line the
    correct answer is no line at all, so the window is held open rather than
    broken off at the first timeout (the unsolicited downlink is 2 Hz, so a
    quiet port really is quiet)."""
    ser.write(payload.encode())
    ser.flush()
    log.append(f"[{time.time() - t0:7.2f}s] >> {payload.strip()[:40]!r}"
               + ("..." if len(payload.strip()) > 40 else ""))
    out = []
    end = time.time() + window
    while time.time() < end:
        line = ser.readline().decode(errors="replace").strip()
        if not line:
            continue
        if TELEM.match(line):
            continue
        out.append(line)
        log.append(f"[{time.time() - t0:7.2f}s] << {line}")
    return out


def run_framing(port, baud, log, settle=1.5):
    """Can an over-long line make the controller do something? (PLAN 16.)

    The matrix's own over-long entry only proves the words `ERR line too long`
    appear. That was not enough: the code behind those words had TWO faults, and
    both were about what happens to the REST of the line. `uart2_task.cpp` reset
    its buffer on overflow but kept reading, so the tail of one physical line was
    terminated as a SECOND command; and when the ERR fell inside its own 2 s rate
    limit the buffer was not reset at all, so the terminator dispatched the
    TRUNCATED first 64 bytes. Bench-observed on 2026-11-23: an over-long
    `SET SP2=0.33` plus 120 junk bytes answered `OK SAFETY` and left SP2 at 0.330
    — an over-long WRITE landing. Framing decides whether the router sees a
    command at all, so this phase asserts the three things that matter: the
    refusal is the only answer (no phantom second command), a rate-limited
    overflow is still dropped whole, and neither write changed anything.

    The two overflows go back to back on purpose: the second one is sent inside
    the error window, which is exactly the case the old code got wrong.
    """
    import serial

    ser = serial.Serial(port, baud, timeout=1.0)
    checks = []
    t0 = T_ORIGIN
    try:
        time.sleep(settle)
        ser.reset_input_buffer()
        ser.write(b"\r\n")
        ser.flush()
        end = time.time() + 0.6
        drew = []
        while time.time() < end:
            l = ser.readline().decode(errors="replace").strip()
            if l and not TELEM.match(l):
                drew.append(l)
        if drew:
            log.append(f"[{time.time() - t0:7.2f}s] (flushed stray bytes: {drew})")

        # Precondition: the value an over-long write must not change.
        base = [l for l in _send_and_drain(ser, "GET SP2\r\n", t0, log)
                if l.startswith("SP2=")]
        want = base[0] if base else None
        checks.append(("framing: SP2 baseline readable", want is not None,
                       want or "no SP2= reply, the phase proves nothing"))
        if want is None:
            return checks

        # Fresh error window, so check 1 is about framing and not about whatever
        # the previous tool in the pipeline sent.
        time.sleep(2.2)
        # The space before the junk is the whole point of the payload: the
        # settings parser tolerates a tail that starts after a space
        # (settings.cpp:131), so a truncated `SET SP2=0.33 <junk>` is a VALID
        # write — which is the shape that landed on the bench 2026-11-23
        # (`OK SAFETY`, SP2 read back 0.330). Glue the junk straight onto the
        # digits instead and the registry rejects it for a different reason,
        # and this check would be about the parser rather than about framing.
        junk = "SET SP2=0.33 " + "Z" * 120 + "\r\n"
        got = _send_and_drain(ser, junk, t0, log)
        checks.append(("over-long line: only `ERR line too long`, no second command",
                       got == ["ERR line too long"],
                       f"want ['ERR line too long'], got {got}"))

        # Immediately again: inside the 2 s window the ERR text is suppressed,
        # and THAT is the case that used to execute the truncated prefix.
        got2 = _send_and_drain(ser, "SET SP2=0.31 " + "Z" * 120 + "\r\n", t0, log)
        checks.append(("overflow inside the error window answers nothing",
                       got2 == [], f"want silence, got {got2}"))

        back = [l for l in _send_and_drain(ser, "GET SP2\r\n", t0, log)
                if l.startswith("SP2=")]
        checks.append(("neither over-long SET wrote",
                       bool(back) and back[0] == want,
                       f"{want} -> {back[:1] or '(no reply: the link died)'}"))
        checks.append(("the line after a dropped line still works",
                       bool(back), back[0] if back else "no reply after the dropped line"))
    finally:
        ser.close()
    return checks


def run_cal_air(console, port, baud, watch, settle=1.5, log=None,
                web_host=None, boot_hi=None):
    """The air calibration, end to end, with its numbers on the wire.

    The matrix proves the REFUSAL half (a warm-up refusal becomes a protocol row).
    This phase proves the half that matters to a diver: the calibration actually
    runs and its verdict — samples, raw averages, worst spread — reaches the
    protocol, unprompted as an E, row and on request as CALAIR,, and agrees with
    what the machine printed on its own console.

    It does not guess the warm-up clock. It asks for a calibration inside the
    gate, reads the refusal's `warmup_left_s` back off the protocol, and waits
    exactly that long — the honest reply is what schedules the check. Takes about
    two minutes, and leaves the cells calibrated until the next reset.
    """
    import serial

    ser = serial.Serial(port, baud, timeout=1.0)
    log = log if log is not None else []
    checks = []
    t0 = T_ORIGIN

    def ask(cmd, want=None, window=2.0):
        ser.write((cmd + "\r\n").encode())
        ser.flush()
        out, end = [], time.time() + window
        while time.time() < end:
            l = ser.readline().decode(errors="replace").strip()
            if not l:
                break
            # NOT TELEM.match(l): that pattern covers T,/D,/E, and the E, rows are
            # the evidence this phase is after — the verdict arriving unprompted.
            if not l.startswith(("T,", "D,")):
                log.append(f"[{time.time() - t0:7.2f}s] << {l}")
            if want and re.match(want, l):
                out.append(l)
                break
            if not l.startswith(("T,", "D,")):
                out.append(l)
        return out

    try:
        time.sleep(settle)
        ser.reset_input_buffer()
        ser.write(b"\r\n"); ser.flush()
        end = time.time() + 0.6
        while time.time() < end:
            ser.readline()

        # 1. Ask inside the warm-up gate and read the seconds left off the reply.
        ask("CAL AIR", r"^OK CAL AIR$")
        left = None
        for l in ask("GET CALAIR", r"^CALAIR,3,"):
            f = l.split(",")
            if len(f) == 9:
                left = int(f[8])
        checks.append(("warm-up refusal says how long is left", left is not None and left > 0,
                       f"warmup_left_s={left} — without it this phase would be guessing"))
        if left is None:
            return checks

        log.append(f"[{time.time() - t0:7.2f}s] (waiting out the warm-up: "
                   f"{left} s, as the controller said)")
        time.sleep(left + 3.0)

        # 2. Ask again: this time it samples. Watch until the verdict is terminal.
        asked = time.time()
        ask("CAL AIR", r"^OK CAL AIR$")
        progress, verdict, events = [], [], []
        while time.time() - asked < 45.0:
            for l in ask("GET CALAIR", None, window=2.0):
                if l.startswith("CALAIR,"):
                    verdict = l
                    if l.split(",")[1] != "1":
                        break
                elif l.startswith("E,"):
                    events.append(l)
            if verdict and verdict.split(",")[1] != "1":
                break
            progress.append(verdict)
        f = verdict.split(",") if verdict else []
        code = int(f[1]) if len(f) == 9 else None
        checks.append(("the calibration left 'running' and reached a verdict",
                       code in (2, 4, 5, 6), f"last answer: {verdict}"))
        checks.append(("it sampled 12 samples, not fewer",
                       len(f) == 9 and int(f[3]) >= 12, f"last answer: {verdict}"))
        checks.append(("the calibration ended unprompted as E,<t>,7,2",
                       any(re.match(r"^E,\d+,7,2$", e) for e in events),
                       f"events seen: {events or 'none'}"))
        ok = code == 2
        checks.append(("the verdict is CALAIR_OK with numbers, not a bare code", ok,
                       f"answer: {verdict}" + ("" if ok else
                       " — no cell window stayed inside its stability band on this bench "
                       "(code 5) or aborted: a real bench condition, not a harness fault")
                       if not ok else verdict))
        if ok:
            checks.append(("a committed calibration carries non-zero raw counts",
                           all(int(x) > 0 for x in f[5:8]), f"raw: {f[5:8]}"))
            # 3. Two records of one event. The console line is the machine's own
            # words; CALAIR, is the client's. They must carry the same numbers, or
            # the thing an engineer reads and the thing a diver's handset shows
            # have come apart — the drift this change exists to prevent.
            cons = next((l for l in watch if l.startswith("cal air: OK")), None)
            checks.append(("the console reported the same calibration", cons is not None,
                           cons or "no 'cal air: OK' on the console while the protocol "
                                   "says the calibration committed"))
            if cons:
                nums = [int(x) for x in re.findall(r"=\s*(\d+)", cons)]
                spread = re.search(r"worst spread\s+(\d+)", cons)
                samples = re.search(r"\((\d+) samples\)", cons)
                agree = (len(nums) == 3 and nums == [int(x) for x in f[5:8]]
                         and bool(spread) and int(spread.group(1)) == int(f[4])
                         and bool(samples) and int(samples.group(1)) == int(f[3]))
                checks.append(("console and CALAIR, agree on the numbers", agree,
                               f"console: {cons!r} vs wire: {verdict!r}"))
        # The refusal's row must not still be the newest verdict: `CALAIR,3` after a
        # successful calibration would mean the record never updated.
        checks.append(("the newest verdict replaced the earlier refusal",
                       code == 2, f"newest answer: {verdict}"))

        # And the durable copy. Everything above proves the verdict reached a client
        # that was listening; only this proves the machine WROTE it down, which is
        # the half a handset cannot check (it has no command for this log) and the
        # half the whole surface log exists for.
        if web_host and ok and len(f) == 9:
            try:
                status, ctype, text = fetch_surface_csv(web_host)
            except Exception as e:                      # noqa: BLE001 - report, do not crash
                checks.append(("GET /surface.csv answers", False, f"{type(e).__name__}: {e}"))
            else:
                checks.append(("GET /surface.csv answers 200 text/csv",
                               status == 200 and ctype.startswith("text/csv"),
                               f"{status} {ctype!r}"))
                # `CALAIR,<code>,<age>,<samples>,<spread>,<raw0>,<raw1>,<raw2>,<left>`
                # against the record's columns, by name from one table.
                wire = dict(zip(("code", "age", "samples", "spread", "raw0", "raw1",
                                 "raw2", "left"), f[1:9]))
                want = [(k, SURF_COL[k], wire[v]) for k, v in
                        (("value", "code"), ("samples", "samples"), ("spread", "spread"),
                         ("raw0", "raw0"), ("raw1", "raw1"), ("raw2", "raw2"))]
                for name, ok_, detail in grade_surface_csv(
                        text, [("cal_air", want)], boot_hi, watch):
                    checks.append((f"surface log: {name}", ok_, detail))
                _, rows = parse_surface_csv(text)
                newest = next((r for r in reversed(rows)
                               if r[SURF_COL["name"]] == "cal_air"), None)
                # The matrix stamps the clock before this phase, so a record made
                # now has a real date. Epoch 0 here means the record was written by
                # a machine that had lost its clock, which the CSV renders honestly
                # as `clock-unset` — and this run would have had no way to know.
                checks.append(("surface log: the record carries a date",
                               newest is not None and int(newest[SURF_COL["epoch"]]) > 0 and
                               newest[SURF_COL["iso"]] != "clock-unset",
                               str(newest) if newest else "no cal_air row"))
    finally:
        ser.close()
    return checks


def run_predive(port, baud, watch, settle=1.5, log=None, web_host=None, boot_hi=None):
    """The pre-dive verdict, on flash.

    A failed pre-dive check is the clearest case for a surface log: the dive it
    prevents never happens, so there is no dive file that could hold the row, and
    until now the only trace was a `PRE,` line a client had to be polling at that
    moment. This phase runs the checks (the `--cal-air` phase, run first, has
    already carried the board past the 120 s warm-up, so this one is short), reads
    the verdict twice — polled on the handset, then read back out of flash over
    HTTP — and requires them to agree on the step that failed.

    The sequence aborts at its first failure, so the step named here is whichever
    one the bench actually defeats. As configured (no pressure rig on the solenoid,
    pack unplugged) the verdict is `pre_dive_fail` at step 2, solenoid, and the
    battery step never runs. That is the honest state of the machine, not a harness
    fault: what is graded is that the record says the same thing the protocol does,
    not that the check passed.
    """
    import serial

    ser = serial.Serial(port, baud, timeout=1.0)
    log = log if log is not None else []
    checks = []
    t0 = T_ORIGIN

    def ask(cmd, window=2.0):
        ser.write((cmd + "\r\n").encode())
        ser.flush()
        out, end = [], time.time() + window
        while time.time() < end:
            l = ser.readline().decode(errors="replace").strip()
            if not l:
                break
            if not l.startswith(("T,", "D,")):
                log.append(f"[{time.time() - t0:7.2f}s] << {l}")
                out.append(l)
        return out

    try:
        time.sleep(settle)
        ser.reset_input_buffer()
        ser.write(b"\r\n")
        ser.flush()
        end = time.time() + 0.6
        while time.time() < end:
            ser.readline()

        ask("CHECK")
        pre = None
        asked = time.time()
        while time.time() - asked < 200.0:
            for l in ask("GET PRE", window=2.0):
                if l.startswith("PRE,"):
                    pre = l
            if pre:
                f = pre.split(",")
                if len(f) > 6 and (f[6] == "1" or any(x == "3" for x in f[1:6])):
                    break
        f = pre.split(",") if pre else []
        checks.append(("the pre-dive sequence reached a verdict",
                       len(f) > 6 and (f[6] == "1" or any(x == "3" for x in f[1:6])),
                       f"last answer: {pre}"))
        if len(f) <= 6:
            return checks

        passed = f[6] == "1"
        step = next((i for i in range(5) if f[1 + i] == "3"), None)
        checks.append(("a pre-dive failure names the step that failed",
                       passed or step is not None,
                       f"{pre} — no step reports FAIL, so the record could not say "
                       f"what to fix"))
        verdict = ("pre_dive_pass", 0) if passed else ("pre_dive_fail", step)

        # The console's own line, with the record's seq: that seq is what joins the
        # machine's claim to the row that comes back out of flash.
        rec = None
        said = time.time()
        while time.time() - said < 10.0:
            for l in watch:
                m = SURFACE_RECORD.match(l)
                if m and int(m.group("id")) in (13, 16):
                    rec = m
            if rec or any(SURFACE_FAIL.match(l) for l in watch):
                break
            time.sleep(0.5)
        checks.append(("the console reported writing the pre-dive record", rec is not None,
                       "no `surface: record N id 13|16` line — the verdict was never "
                       "written down" if rec is None else rec.group(0)))
        checks.append(("the record was not refused by the log",
                       not any(SURFACE_FAIL.match(l) for l in watch),
                       next((l for l in watch if SURFACE_FAIL.match(l)), "")))

        if web_host:
            try:
                status, ctype, text = fetch_surface_csv(web_host)
            except Exception as e:                      # noqa: BLE001
                checks.append(("GET /surface.csv answers", False, f"{type(e).__name__}: {e}"))
            else:
                checks.append(("GET /surface.csv answers 200 text/csv",
                               status == 200 and ctype.startswith("text/csv"),
                               f"{status} {ctype!r}"))
                want = [("value", SURF_COL["value"], verdict[1])]
                for name, ok_, detail in grade_surface_csv(
                        text, [(verdict[0], want)], boot_hi, watch):
                    checks.append((f"surface log: {name}", ok_, detail))
                _, rows = parse_surface_csv(text)
                both = {r[SURF_COL["name"]] for r in rows}
                # A FAIL record must not be filed as a pass, and the other way
                # round: this is the check that a `value` means what the protocol
                # says it means, not just that some row exists.
                checks.append(("surface log: the verdict is filed under the right id",
                               verdict[0] in both and
                               ((verdict[0] == "pre_dive_pass") ==
                                any(r[SURF_COL["name"]] == "pre_dive_pass" and
                                    int(r[SURF_COL["value"]]) == 0 for r in rows)),
                               f"verdict {verdict}, names: {sorted(both)}"))
    finally:
        ser.close()
    return checks


def _handset_session(port, baud, settle, log):
    """Open the handset port and clear the bridge's boot noise (opening the
    port resets the spare-ESP32 bridge). Returns the open Serial; caller closes."""
    import serial
    ser = serial.Serial(port, baud, timeout=1.0)
    time.sleep(settle)
    ser.reset_input_buffer()
    ser.write(b"\r\n")
    ser.flush()
    end = time.time() + 0.6
    while time.time() < end:
        ser.readline()
    return ser


def _ask(ser, log, cmd, window=2.0):
    ser.write((cmd + "\r\n").encode())
    ser.flush()
    out, end = [], time.time() + window
    while time.time() < end:
        l = ser.readline().decode(errors="replace").strip()
        if not l:
            break
        if not l.startswith(("T,", "D,")):
            log.append(f"[{time.time() - T_ORIGIN:7.2f}s] << {l}")
            out.append(l)
    return out


def _ask_depth(ser, log, window=2.0):
    """GET T on the handset link, returning the T, lines themselves.

    `_ask` drops the unsolicited `T,`/`D,` stream — right for command
    answers, fatal for reading depth: the first bench run of --wifi-depth
    (2026-11-26) found the depth polls parsing everything EXCEPT depth, and
    passing the squeeze half off an unrelated event row's timestamp. No host
    selftest could catch that: they grade the scorer on captured caps, and
    this poller only ever ran here, against a real thumb.
    """
    ser.write(b"GET T\r\n")
    ser.flush()
    out, end = [], time.time() + window
    while time.time() < end:
        l = ser.readline().decode(errors="replace").strip()
        if l.startswith("T,"):
            log.append(f"[{time.time() - T_ORIGIN:7.2f}s] << {l}")
            out.append(l)
    return out


def run_wifi_depth(port, baud, watch, settle=1.5, squeeze_wait=90.0, log=None):
    """The DEPTH half of the radio policy, on the real sensor.

    The bench's thumb pressure sensor reaches ~5.5 m, so this leg drives the
    same latch the diver's actual depth will: past WIFI_OFF_DEPTH the radio must
    drop with reason 3 (depth) and the console must name it; after release, the
    AP must return no sooner than WIFI_ON_HOLD_S of STRAIGHT shallow water. The
    timing witnesses are why the capture records timestamps, not just outcomes:
    a firmware that drops on depth but restores the moment depth dips back is a
    different machine, and only the clock can tell them apart.

    Operator part: squeeze on cue, release on cue. If either cue goes unmet the
    leg FAILS with the reason — opt-in means un-run is a report, not a skip.
    """
    log = log if log is not None else []
    cap = {}
    ser = _handset_session(port, baud, settle, log)
    try:
        offD = onD = holdS = None
        for l in _ask(ser, log, "GET WIFI_OFF_DEPTH"):
            if "=" in l:
                offD = float(l.split("=", 1)[1])
        for l in _ask(ser, log, "GET WIFI_ON_DEPTH"):
            if "=" in l:
                onD = float(l.split("=", 1)[1])
        for l in _ask(ser, log, "GET WIFI_ON_HOLD_S"):
            if "=" in l:
                holdS = float(l.split("=", 1)[1])
        cap.update(offD=offD, onD=onD, holdS=holdS)
        if offD is None or onD is None or holdS is None:
            cap["unreadable"] = True
            return cap

        print(f"\n  >>> SQUEEZE the thumb pressure sensor past {offD} m now <<<")
        t0, maxd, t_deep = time.time(), None, None
        while time.time() - t0 < squeeze_wait:
            for l in _ask_depth(ser, log):
                try:
                    d = float(l.split(",")[1])
                except (ValueError, IndexError):
                    continue
                maxd = d if maxd is None else max(maxd, d)
                if d > offD:
                    t_deep = time.time()
                    break
            if t_deep:
                break
        cap.update(max_depth=maxd, t_deep=t_deep)
        if not t_deep:
            return cap

        off_line = None
        t_end = time.time() + 5.0
        while time.time() < t_end and off_line is None:
            for l in _ask(ser, log, "GET WIFI"):
                if l.startswith("WIFI,0,"):
                    off_line = l
                    break
        cap["off_line"] = off_line
        cap["console_down"] = any(l.startswith("wifi: radio down (depth)") for l in watch)

        print("  >>> RELEASE the squeeze now <<<")
        t0, t_shallow = time.time(), None
        while time.time() - t0 < squeeze_wait:
            for l in _ask_depth(ser, log):
                try:
                    d = float(l.split(",")[1])
                except (ValueError, IndexError):
                    continue
                if d < onD:
                    t_shallow = time.time()
                    break
            if t_shallow:
                break
        cap["t_shallow"] = t_shallow
        if not t_shallow:
            return cap

        # Hold witness: at shallow+20 s the straight-shallow timer has 40 s to
        # run, so the radio must still be down. This is the check a flap-prone
        # firmware cannot pass.
        time.sleep(20.0)
        mid_line = next((l for l in _ask(ser, log, "GET WIFI") if l.startswith("WIFI,")), None)
        cap["mid_line"] = mid_line

        up_line, t_up = None, None
        t_end = t_shallow + holdS + 15.0
        while time.time() < t_end and up_line is None:
            for l in _ask(ser, log, "GET WIFI"):
                if l.startswith("WIFI,1,"):
                    up_line, t_up = l, time.time()
                    break
        cap.update(up_line=up_line, t_up=t_up,
                   console_up=any(l.startswith("wifi: radio up") for l in watch))
    finally:
        ser.close()
    return cap


def grade_wifi_depth(cap):
    checks = []
    if cap.get("unreadable"):
        return [("the WIFI_* settings are readable", False,
                 "GET WIFI_OFF_DEPTH/WIFI_ON_DEPTH/WIFI_ON_HOLD_S did not answer — "
                 "the leg cannot name its own thresholds")]
    offD, onD, holdS = cap["offD"], cap["onD"], cap["holdS"]
    md = cap.get("max_depth")
    checks.append(("the squeeze actually crossed the off depth",
                   md is not None and cap.get("t_deep") is not None,
                   f"max depth seen {md} m (off depth {offD} m) — the operator cue "
                   f"was not met, so the leg did not run"))
    if not cap.get("t_deep"):
        return checks
    off_line = cap.get("off_line")
    m = WIFI_LINE.match(off_line) if off_line else None
    checks.append(("depth past the threshold took the radio down",
                   m is not None and m.group(1) == "0",
                   f"{off_line!r} — the radio stayed up (or answered something else) "
                   f"past {offD} m: the milliamps argument is the whole feature"))
    checks.append(("and blamed the depth, not something else",
                   m is not None and m.group(2) == "3",
                   f"{off_line!r} — held_by must be 3 (depth): a radio down for a "
                   f"different reason here means the latch and the reason column "
                   f"disagree"))
    checks.append(("the console said depth too",
                   cap.get("console_down") is True,
                   "no 'wifi: radio down (depth)' console line for a wire that "
                   "says the radio is down: two views of one machine"))
    if not cap.get("t_shallow"):
        checks.append(("the release crossed back under the on depth", False,
                       f"never observed below {onD} m — the return half was never witnessed"))
        return checks
    mid = cap.get("mid_line")
    mm = WIFI_LINE.match(mid) if mid else None
    checks.append(("the 60 s straight-shallow hold held",
                   mm is not None and mm.group(1) == "0",
                   f"{mid!r} at shallow+20 s — back up before {holdS} s straight is "
                   f"the flap this leg exists to catch"))
    up_line, t_up = cap.get("up_line"), cap.get("t_up")
    mu = WIFI_LINE.match(up_line) if up_line else None
    checks.append(("the radio came back after the hold",
                   mu is not None and mu.group(1) == "1",
                   f"{up_line!r} within {holdS + 15:.0f} s of shallow — a diver "
                   f"surfacing to no AP is the fault this half witnesses"))
    if mu and mu.group(1) == "1":
        lat = t_up - cap["t_shallow"]
        checks.append((f"and no sooner than the hold allows (back after {lat:.0f} s)",
                       holdS - 5.0 <= lat <= holdS + 15.0,
                       f"back at {lat:.0f} s after first shallow: outside "
                       f"{holdS:.0f} s straight the timer is not being kept"))
    checks.append(("the console said the radio came up",
                   cap.get("console_up") is True,
                   "no 'wifi: radio up' line for a radio the wire says came back"))
    return checks


def run_wifi_idle(port, baud, watch, settle=1.5, log=None, wlan=None, boot_lines=()):
    """The IDLE half of the radio policy, hands-free: one setting, one minute,
    a countdown you can watch shrink, and the word that restores the radio.

    The leg sends `WIFI ON` first — an intent event, which rewinds the
    firmware's window to that instant — and only then sets WIFI_IDLE_MIN=1, so
    the fire is a deterministic ~60 s from the intent event (the timestamps
    below are measured from THERE, because that is where the machine restarted
    its clock; from the SET the same honest fire reads 7 s early). The countdown field is sampled
    mid-window and required to be strictly shrinking: a frozen countdown and a
    running one answer the same 'off in N s' once, and differently forever.
    """
    log = log if log is not None else []
    cap = {}
    ser = _handset_session(port, baud, settle, log)
    try:
        cap["before"] = next((l for l in _ask(ser, log, "GET WIFI") if l.startswith("WIFI,")), None)
        cap["station_seen"] = 0
        if wlan and cap["before"]:
            m = WIFI_LINE.match(cap["before"])
            if m and int(m.group(4) or 0) > 0:
                # The bench PC joins the AP itself (NM autojoin, from any past
                # web_probe). nmcli is the bench machine's own lever; asked for
                # by name via --wifi-idle-wlan, so the tool never touches a
                # network it was not told to.
                _nmcli(log, ["device", "disconnect", wlan])
                t_end = time.time() + 20.0
                while time.time() < t_end:
                    l = next((x for x in _ask(ser, log, "GET WIFI") if x.startswith("WIFI,")), None)
                    cap["before"] = l or cap["before"]
                    mm = WIFI_LINE.match(cap["before"])
                    if mm and int(mm.group(4) or 0) == 0:
                        break
                    time.sleep(2.0)
        # The window is timed from the INTENT EVENT, not from the SET:
        # `WIFI ON` is the applyWifi that rewinds the firmware's window, and it
        # is the firmware that keeps the clock. Measuring from the SET (which
        # lands ~7 s later — every _ask eats its full window because the 2 Hz
        # stream prevents the blank-line break) reddened an honest board on the
        # first bench run: the fire was 63 s after WIFI ON, exactly the setting,
        # and 56 s after arming, which is not anything's setting at all.
        t_anchor = time.time()
        ok = any(l.startswith("OK") for l in _ask(ser, log, "WIFI ON"))
        cap["intent_ok"] = ok
        armed = any(l.startswith("OK") for l in _ask(ser, log, "SET WIFI_IDLE_MIN=1"))
        rb = next((l for l in _ask(ser, log, "GET WIFI_IDLE_MIN") if "=" in l), None)
        cap["armed_ok"] = armed
        cap["armed_readback"] = rb.split("=", 1)[1] if rb else None

        on_obs, off_line, t_off = [], None, None
        while time.time() - t_anchor < 135.0:
            time.sleep(4.0)
            l = next((x for x in _ask(ser, log, "GET WIFI") if x.startswith("WIFI,")), None)
            if l is None:
                continue
            m = WIFI_LINE.match(l)
            if m and m.group(1) == "1":
                on_obs.append((time.time() - t_anchor, int(m.group(3)),
                               int(m.group(4) or 0)))
                if int(m.group(4) or 0) > cap["station_seen"]:
                    cap["station_seen"] = int(m.group(4) or 0)
            elif m:
                off_line, t_off = l, time.time() - t_anchor
                break
        cap.update(on_obs=on_obs, off_line=off_line, t_off=t_off,
                   console_down=any(l.startswith("wifi: radio down (idle") for l in watch))

        _ask(ser, log, "WIFI ON")
        cap["restore_line"] = next((l for l in _ask(ser, log, "GET WIFI") if l.startswith("WIFI,")), None)
        cap["console_up"] = any(l.startswith("wifi: radio up") for l in watch)
        any(l.startswith("OK") for l in _ask(ser, log, "SET WIFI_IDLE_MIN=10"))
        rb = next((l for l in _ask(ser, log, "GET WIFI_IDLE_MIN") if "=" in l), None)
        cap["restore_readback"] = rb.split("=", 1)[1] if rb else None
        if wlan:
            # Rejoin what we took off, so a later web_probe run starts from the
            # state it assumes. The AP name and passphrase are read from the
            # machine, not assumed: names carry a MAC tail, passphrases are the
            # diver's.
            ap_name = next((l.split('"')[1] for l in boot_lines
                            if l.startswith("web: AP ") and len(l.split('"')) > 1), None)
            ap_pass = None
            for l in _ask(ser, log, "GET AP_PASS"):
                if l.startswith("AP_PASS="):
                    ap_pass = l.split("=", 1)[1]
            if ap_name and ap_pass:
                _nmcli(log, ["device", "wifi", "connect", ap_name, "password", ap_pass])
    finally:
        ser.close()
    return cap


def _nmcli(log, argv):
    """One nmcli call, outcome logged. Bench-machine-only lever; the caller
    asked for it by interface name."""
    import subprocess
    try:
        r = subprocess.run(["nmcli"] + argv, capture_output=True, text=True, timeout=20)
        log.append(f"[{time.time() - T_ORIGIN:7.2f}s] (nmcli {' '.join(argv)!r}: "
                   f"rc={r.returncode} {r.stdout.strip()[:80]}{r.stderr.strip()[:80]})")
        return r.returncode == 0
    except Exception as e:                              # noqa: BLE001
        log.append(f"[{time.time() - T_ORIGIN:7.2f}s] (nmcli failed: {e})")
        return False


def grade_wifi_idle(cap):
    checks = []
    b = WIFI_LINE.match(cap.get("before") or "")
    # Attribution before cascade. A station on the AP makes this leg's premise
    # false and its failure SHAPE misleading: the countdown sits at exactly the
    # full window forever and nothing ever fires, which looks like a dead timer
    # and is the rewind rule working. Found on the bench 2026-11-26 by the one
    # station the harness always forgets: the bench PC's own wlan0, autojoined
    # to ESP-CCR from an earlier web_probe. When stations were seen, say THAT
    # and stop — four cascading "timer" failures would send the next operator
    # hunting a firmware bug that does not exist.
    if cap.get("station_seen"):
        checks.append(("no station was associated during the leg", False,
                       f"{cap['station_seen']} station(s) seen on the AP — the "
                       f"idle rule cannot fire while anything is connected, and "
                       f"the countdown pinned at the full window is the rewind, "
                       f"not a stall. Disconnect (the bench PC autojoins ESP-CCR; "
                       f"--wifi-idle-wlan wlan0 does it for you) and rerun"))
        checks.append(("WIFI_IDLE_MIN restored to 10 before the leg exits",
                       cap.get("restore_readback") in ("10", "10.0", "10.000"),
                       f"readback {cap.get('restore_readback')!r}"))
        return checks
    checks.append(("the leg started with the radio up",
                   b is not None and b.group(1) == "1",
                   f"{cap.get('before')!r} at leg start — with the radio already "
                   f"held down, the 60 s window this leg grades never started"))
    checks.append(("WIFI_IDLE_MIN=1 armed and read back",
                   cap.get("armed_ok") and cap.get("armed_readback") in ("1", "1.0", "1.000"),
                   f"ok={cap.get('armed_ok')} readback={cap.get('armed_readback')!r} — "
                   f"without the read-back the fire could be the old 10-minute rule"))
    obs = cap.get("on_obs") or []
    shrinking = len(obs) >= 2 and obs[-1][1] < obs[0][1]
    checks.append(("the countdown was seen shrinking",
                   shrinking,
                   f"observations {obs} — a frozen or absent idle_left_s field means "
                   f"the 'off in N s' promise was decoration"))
    late = [o for o in obs if o[0] >= 50.0]
    checks.append(("the radio was still up after 50 s of a 60 s window",
                   bool(late),
                   f"last on-observation: {obs[-1] if obs else None} s after the "
                   f"WIFI ON that armed the window — if the fire beat the window "
                   f"the timer is not the setting"))
    off_line, t_off = cap.get("off_line"), cap.get("t_off")
    m = WIFI_LINE.match(off_line) if off_line else None
    checks.append(("one idle minute with no station took the radio down",
                   m is not None and m.group(1) == "0",
                   f"never went off within the sample window of the intent event: "
                   f"last answer {off_line!r}"))
    checks.append(("and blamed the idle rule, by number",
                   m is not None and m.group(2) == "4",
                   f"{off_line!r} — held_by must be 4 (idle): the reason column is "
                   f"the only part of this line a phone can act on"))
    if m and m.group(1) == "0" and t_off is not None:
        checks.append((f"and fired on the minute, not before ({t_off:.0f} s)",
                       60.0 <= t_off <= 130.0,
                       f"fired at {t_off:.0f} s after the intent event that "
                       f"rewound the window: the window is the setting"))
    checks.append(("the console said idle too",
                   cap.get("console_down") is True,
                   "no 'wifi: radio down (idle...' console line for a radio the "
                   "wire says the idle rule took down"))
    r = WIFI_LINE.match(cap.get("restore_line") or "")
    checks.append(("WIFI ON cleared the idle hold (words outrank idle)",
                   r is not None and r.group(1) == "1",
                   f"{cap.get('restore_line')!r} after WIFI ON — words are exactly "
                   f"the intent event that clears an idle hold; depth they cannot "
                   f"clear, and this leg proves the pair apart"))
    checks.append(("the console said the radio came up",
                   cap.get("console_up") is True,
                   "no 'wifi: radio up' line for a radio the wire says came back"))
    checks.append(("WIFI_IDLE_MIN restored to 10 before the leg exits",
                   cap.get("restore_readback") in ("10", "10.0", "10.000"),
                   f"readback {cap.get('restore_readback')!r} — leaving the board "
                   f"armed at 1 minute makes its next surface quiet by surprise"))
    return checks


def selftest_wifi_legs():
    """Lying transcripts for the two radio-policy legs. Same discipline: each
    case breaks exactly one rule, and the grader must name it."""
    honest_depth = dict(offD=2.0, onD=1.0, holdS=60.0, max_depth=5.4, t_deep=1.0,
                        off_line="WIFI,0,3,0,0", console_down=True, t_shallow=21.0,
                        mid_line="WIFI,0,3,0,0", up_line="WIFI,1,0,600,0", t_up=81.5,
                        console_up=True)
    depth_cases = [
        ("depth leg: honest transcript", honest_depth, None),
        ("depth leg: operator never squeezed",
         {**honest_depth, "max_depth": 0.4, "t_deep": None}, "squeeze actually crossed"),
        ("depth leg: radio stayed up at depth",
         {**honest_depth, "off_line": None}, "took the radio down"),
        ("depth leg: wrong reason (blamed the dive)",
         {**honest_depth, "off_line": "WIFI,0,2,0,0"}, "blamed the depth"),
        ("depth leg: console silent",
         {**honest_depth, "console_down": False}, "console said depth"),
        ("depth leg: came back early (flap)",
         {**honest_depth, "mid_line": "WIFI,1,0,600,0"}, "hold held"),
        ("depth leg: timer too short",
         {**honest_depth, "t_up": 51.0}, "no sooner than the hold"),
        ("depth leg: never came back",
         {**honest_depth, "up_line": None, "t_up": None}, "came back after the hold"),
        ("depth leg: up without a console line",
         {**honest_depth, "console_up": False}, "console said the radio came up"),
    ]
    honest_idle = dict(before="WIFI,1,0,540,0", intent_ok=True, armed_ok=True,
                       armed_readback="1", on_obs=[(10.4, 50, 0), (30.4, 30, 0),
                                                   (50.4, 10, 0)],
                       off_line="WIFI,0,4,0,0", t_off=62.1, console_down=True,
                       restore_line="WIFI,1,0,60,0", console_up=True,
                       restore_readback="10", station_seen=0)
    idle_cases = [
        ("idle leg: honest transcript", honest_idle, None),
        ("idle leg: started with the radio down",
         {**honest_idle, "before": "WIFI,0,4,0,0"}, "started with the radio up"),
        ("idle leg: setting never armed",
         {**honest_idle, "armed_readback": "10"}, "armed and read back"),
        ("idle leg: frozen countdown",
         {**honest_idle, "on_obs": [(10.4, 50, 0), (30.4, 50, 0), (50.4, 50, 0)]},
         "countdown was seen shrinking"),
        ("idle leg: a station was on the AP all along",
         {**honest_idle, "station_seen": 1, "on_obs": [(10.4, 60, 1), (30.4, 60, 1),
                                                       (50.4, 60, 1)],
          "off_line": None, "t_off": None, "console_down": False},
         "station was associated"),
        ("idle leg: no observation inside the window",
         {**honest_idle, "on_obs": [(10.4, 50)]}, "still up after 50 s"),
        ("idle leg: never turned off",
         {**honest_idle, "off_line": None, "t_off": None}, "no station took the radio down"),
        ("idle leg: wrong reason (depth)",
         {**honest_idle, "off_line": "WIFI,0,3,0,0"}, "blamed the idle rule"),
        ("idle leg: fired at 20 s, not 60",
         {**honest_idle, "t_off": 20.3}, "fired on the minute"),
        ("idle leg: last up-observation too early to trust the clock",
         {**honest_idle, "on_obs": [(10.4, 50, 0), (30.4, 30, 0)]},
         "still up after 50 s"),
        ("idle leg: console silent",
         {**honest_idle, "console_down": False}, "console said idle"),
        ("idle leg: WIFI ON did not restore",
         {**honest_idle, "restore_line": "WIFI,0,4,0,0"}, "cleared the idle hold"),
        ("idle leg: up with no console line",
         {**honest_idle, "console_up": False}, "console said the radio came up"),
        ("idle leg: left armed at 1",
         {**honest_idle, "restore_readback": "1"}, "restored to 10"),
    ]
    checks = []
    for name, cap, expect in [*depth_cases, *idle_cases]:
        got = grade_wifi_depth(cap) if "offD" in cap else grade_wifi_idle(cap)
        failed = [c[0] for c in got if not c[1]]
        if expect is None:
            checks.append((name, not failed, f"unexpected failures: {failed}"))
        else:
            checks.append((name, any(expect in f for f in failed),
                           f"no failure matching {expect!r}; failures seen: {failed}"))
    return checks


def run_dive_persist(console, port, baud, watch, settle=1.5, boot_n=None):
    """Does a closed dive survive a reset?
    record, never the dive". Nothing on the bench had ever tested that promise,
    and when the dive-CSV work asked the question the answer looked like "no":
    after a boot, `LOG LIST` answered empty and `GET /dive/<n>.csv` answered
    `no such dive`, while an esptool `read_flash` of the LittleFS partition
    showed `/dive1.bin` sitting there with its records intact. Two different
    faults wore that symptom identically — an unreadable filesystem, and a
    filesystem nobody had mounted yet (the store used to be mounted by DIVE
    START, so every surface read before the first dive of a boot was blind).
    The boot banner says which one it is now, and this says whether the bytes
    outlived the reset.
    """
    import serial  # pipx platformio venv has it

    ser = serial.Serial(port, baud, timeout=1.0)
    checks = []
    time.sleep(settle)

    def slots():
        """Dives `LOG LIST` reports, as (slot, epoch, seq) rows, after the
        standard line-buffer flush (un-terminated bytes in the controller's
        buffer would be parsed with this command; see the flush comment in
        run_protocol). The firmware lists newest-written first, and `seq` is
        the field that makes that claim mean something on a board with no
        clock — the whole point of the sequence number."""
        ser.reset_input_buffer()
        ser.write(b"\r\n")
        ser.flush()
        time.sleep(0.3)
        ser.reset_input_buffer()
        ser.write(b"LOG LIST\r\n")
        ser.flush()
        out, end = [], time.time() + 3.0
        while time.time() < end:
            l = ser.readline().decode(errors="replace").strip()
            if l.startswith("L,"):
                parts = l.split(",")
                if len(parts) > 1 and parts[1].isdigit():
                    ep = int(parts[2]) if len(parts) > 2 and parts[2].isdigit() else 0
                    sq = int(parts[3]) if len(parts) > 3 and parts[3].isdigit() else 0
                    out.append((int(parts[1]), ep, sq))
        return out

    def ask(line, window=3.0, stop_at=None):
        """One command, fixed collection window. The controller streams T,/D,/E,
        rows every 500 ms, so there is no 'the link went quiet' end to a read:
        the window is the window. `stop_at` ends the read on a line that is the
        data's own terminator — for a dive export that is the `M,` trailer,
        which matters because live `E,` rows pushed AFTER the trailer are
        byte-identical to exported ones and would otherwise be counted as file
        rows (see spec §5: the ambiguity is the reader's position, not the
        controller disagreeing with itself)."""
        ser.reset_input_buffer()
        ser.write((line + "\r\n").encode())
        ser.flush()
        out, end = [], time.time() + window
        while time.time() < end:
            l = ser.readline().decode(errors="replace").strip()
            if not l:
                continue
            out.append(l)
            if stop_at and stop_at(l):
                break
        return out

    before = slots()
    ser.write(b"DIVE START\r\n")
    ser.flush()
    # Watching the link while the dive runs, rather than sleeping through it:
    # the `E,` rows the handset sees live are the reference the dive FILE has to
    # match. Both records come out of the same pop in UartLink::drainEvents, so
    # a row the handset saw and the file does not hold is a verdict the diver
    # cannot read back after the dive — which is exactly how this used to work.
    # Reading starts the instant the command goes out, not after a pause: the
    # dive's own opening rows (the per-cell vote rejoins, id 6) are published on
    # the next control cycle, and a capture that misses them leaves the join
    # below comparing the file against nothing.
    wire_events = []
    end = time.time() + 7.5            # one 30 s grid row will not land; event
    while time.time() < end:           # rows and the header already make a dive
        l = ser.readline().decode(errors="replace").strip()
        if l.startswith("E,"):
            wire_events.append(l)
    ser.write(b"DIVE END\r\n")
    ser.flush()
    time.sleep(3.0)
    mid = slots()
    print(f"    dive watch: {len(wire_events)} E, rows streamed live — "
          f"{wire_events[:6]}")

    # The dive just closed, read back out of flash and graded against itself
    # (see grade_dive_file: the file's verdict rows, and the `M,` summary
    # agreeing with the rows beside it rather than contradicting them).
    newest = max(mid, key=lambda r: r[2], default=None)
    drows = ask(f"LOG EXPORT {newest[0]}", 7.0,
                stop_at=lambda l: l.startswith("M,")) if newest else []
    want = {k: next((l for l in ask(f"GET {k}") if l.startswith(f"{k}=")), "")
            for k in ("GF_LOW", "GF_HIGH", "SP1", "SP2", "SP3", "SP4")}
    checks.extend(grade_dive_file(drows, want, wire_events))

    mark = len(watch)   # everything the console watcher sees after here is the new boot
    # Run-mode reset through EN. DTR stays low on purpose: pulsing it drives IO0
    # and latches the ROM into download mode, which looks like a dead board.
    console.setDTR(False)
    console.setRTS(True)
    time.sleep(0.12)
    console.setRTS(False)
    time.sleep(18.0)                   # boot banner + the log line lands
    after = slots()
    ser.close()

    new_boot = watch[mark:]
    ll = next((l for l in new_boot if LOG_LINE.search(l)), None)

    # A closed dive must appear in the list. The count only has to grow while a
    # slot is free: the FIFO holds ten dives and a full one evicts, so once it
    # is full the set stays at ten and the new dive replaces an old one.
    # Asserting "+1" in the full case would fail a run that worked as designed.
    # What the old check could NOT say was WHICH dive got evicted, because a
    # full FIFO lists ten slots whatever the order of writing was — measured on
    # the bench 2026-11-24, the eviction hammered slot 1 and ate the newest
    # dives while every count still passed (PLAN 16). The sequence number is
    # what lets this check name the newest dive and prove it survived.
    want = min(len(before) + 1, MAX_DIVE_SLOTS)
    slotset = lambda rows: [s for s, _e, _q in rows]
    checks.append(("dive closed this run is in LOG LIST",
                   len(mid) == want,
                   f"before {slotset(before) or '[]'} ({len(before)}), after DIVE END "
                   f"{slotset(mid) or '[]'} ({len(mid)}), expected {want}"
                   + (", FIFO full so one dive is evicted"
                      if len(before) >= MAX_DIVE_SLOTS else "")))
    checks.append(("dive history survives a reset",
                   bool(mid) and set(after) == set(mid),
                   f"{len(mid)} dives before the reset, {len(after)} after: "
                   f"{after or '[]'}"))
    checks.append(("the new boot reports its dive log", ll is not None,
                   ll or "no `log:` line: the boot banner does not say what the "
                         "filesystem holds"))

    seqs_mid = [q for (_s, _e, q) in mid]
    new_seq = max(seqs_mid) if seqs_mid else 0
    checks.append(("the closed dive carries a dive sequence",
                   new_seq > 0 and seqs_mid.count(new_seq) == 1,
                   f"sequences in LOG LIST: {seqs_mid}; 0 means the file predates "
                   f"the sequence field and must not repeat as a live one"))
    stamped = next((r for r in mid if r[2] == new_seq), None)
    checks.append(("the closed dive is stamped with the wall clock",
                   bool(stamped) and stamped[1] >= 946684800,   # 2000-01-01
                   f"newest dive (seq {new_seq}) epoch "
                   f"{stamped[1] if stamped else '-'}: TIME SET in the protocol "
                   f"phase is what makes this a date, not 0"))

    if ll:
        m = LOG_LINE.search(ll)
        note = m.group("note").strip()
        checks.append(("reset did not format the log (history intact)", not note, ll))
        hi = m.group("seqhi")
        checks.append(("the boot banner states the dive-sequence range",
                       hi is not None or not seqs_mid, ll))
        checks.append(("the newest dive's sequence survived the reset",
                       hi is not None and int(hi) == new_seq,
                       f"boot banner top vs LOG LIST newest: {hi} vs {new_seq}"))
    cl = next((l for l in new_boot if CLOCK_LINE.search(l)), None)
    # Measured on this bench 2026-11-24: the EN pulse is a CHIP_PU power-down
    # (the new boot reports `rst:0x1 (POWERON_RESET)`), and it wipes the RTC
    # no-init carry exactly like a battery pull. So the honest expectation
    # after THIS reset is `clock: unset` — the gate is that the boot says so
    # out loud instead of resurrecting a plausible date from silicon garbage
    # (the magic+range rejection, host-pinned by test_carry_needs_both_magic_and_range).
    # The carry itself covers the resets the firmware performs on itself
    # (panic, watchdog, SAFE ACK), which keep RTC memory.
    checks.append(("the boot after the reset states the clock honestly",
                   cl is not None and ("unset" in cl or "carried" in cl),
                   cl or "no `clock:` line: dives after a power cycle would be "
                         "undated in silence"))
    # The surface log gets the same persistence question as the dives, and the boot
    # banner is the ONLY way to ask it: there is no handset command that reads that
    # log back, so "did the calibration verdict outlive the power-down?" has no
    # other answer on the bench. Graded on both numbers the line prints: the count
    # (a log that came back with one record when five were written this run has
    # lost four) and the top seq (a compaction never renumbers, so a lower top seq
    # means the NEWEST ones went, which are the ones a diver cares about). Skipped
    # when nothing was written this run: a fresh filesystem with no calibration is
    # a legitimately empty log, not a lost one.
    sl2 = next((l for l in new_boot if SURFACE_LINE.search(l)), None)
    checks.append(("the new boot reports its surface log", sl2 is not None,
                   sl2 or "no `surface:` line after the reset: an unreadable surface "
                          "log reads exactly like an empty one"))
    wrote = [int(m.group("seq")) for l in watch[:mark]
             if (m := SURFACE_RECORD.match(l))]
    if wrote:
        m2 = SURFACE_LINE.search(sl2) if sl2 else None
        want_n = (boot_n or 0) + len(wrote)
        detail = (f"boot after reset: {sl2!r}; the first boot said {boot_n} records and "
                  f"this run wrote {len(wrote)} (seq {min(wrote)}..{max(wrote)}), "
                  f"want at least {want_n}")
        checks.append(("every surface record written before the reset survived it",
                       bool(m2 and m2.group("n")) and int(m2.group("n")) >= want_n
                       and bool(m2.group("hi")) and int(m2.group("hi")) >= max(wrote),
                       detail))
    return checks


def main() -> int:
    global T_ORIGIN
    T_ORIGIN = time.time()

    ap = argparse.ArgumentParser()
    ap.add_argument("--boot-port", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--handset", default=None)
    ap.add_argument("--handset-settle", type=float, default=1.5)
    ap.add_argument("--watch-tail", type=float, default=12.0,
                    help="seconds to keep watching the console after the last "
                         "command: the command phase itself runs in ~2 s, and HB "
                         "beats every 5 s, so without a tail the stack-headroom "
                         "checks have nothing to read. It also covers a crash "
                         "that lands after the last reply (a SET that breaks the "
                         "control path shows up on the next tick, not on the wire).")
    ap.add_argument("--expect-web", action="store_true")
    ap.add_argument("--boot-wait", type=float, default=20.0)
    ap.add_argument("--cal-air", action="store_true",
                    help="run the air calibration for real and check its verdict "
                         "reaches the protocol (CALAIR, and an unprompted E,*,7,2 "
                         "row) and matches the console. Adds ~2 min — it has to "
                         "wait out the 120 s cell warm-up — and leaves the cells "
                         "calibrated until the next reset.")
    ap.add_argument("--predive", action="store_true",
                    help="run the pre-dive checks for real and require their verdict "
                         "in the surface log (console line + /surface.csv row naming "
                         "the step that failed). Adds ~1 min after --cal-air, less "
                         "without it: the checks wait out the same 120 s cell warm-up. "
                         "Pulses the solenoid on the bench.")
    ap.add_argument("--web-host", default="http://192.168.4.1",
                    help="base URL for /surface.csv, used by --cal-air and --predive")
    ap.add_argument("--expect-depth", action="store_true",
                    help="a depth sensor is fitted on this bench: its boot announce "
                         "is then required, and graded for identity, address and a "
                         "plausible reading (BENCH.md C3-a). Without this flag a "
                         "sensor-less boot is accepted as-is.")
    ap.add_argument("--selftest", action="store_true",
                    help="grade known-lying surface logs, known-lying depth "
                         "announces, known-lying dive exports and known-lying "
                         "ControlTask margin/skip heartbeats (no board "
                         "needed) to prove the graders can "
                         "fail. Run it after touching the graders or the formats.")
    ap.add_argument("--replay", default=None)
    ap.add_argument("--wifi-depth", action="store_true",
                    help="opt-in radio-policy leg with the REAL depth sensor: it "
                         "asks you to squeeze the thumb pressure sensor past "
                         "WIFI_OFF_DEPTH, requires the radio to drop with the "
                         "DEPTH reason and the console to say so, then asks you "
                         "to release and requires the AP back no sooner than "
                         "WIFI_ON_HOLD_S of straight shallow water. ~2-3 min of "
                         "operator action; run it while watching the bench.")
    ap.add_argument("--wifi-squeeze-wait", type=float, default=90.0,
                    help="seconds to wait for the operator to cross the depth "
                         "threshold in --wifi-depth (both directions)")
    ap.add_argument("--wifi-idle", action="store_true",
                    help="opt-in radio-policy leg, no hands needed: sets "
                         "WIFI_IDLE_MIN=1, rewinds the window with WIFI ON, and "
                         "requires the idle hold to fire ~60 s later (protocol "
                         "reason 4 AND the console line), the countdown to be "
                         "seen shrinking, WIFI ON to restore, and the setting to "
                         "come back to 10. Adds ~100 s.")
    ap.add_argument("--wifi-idle-wlan", default=None, metavar="IFACE",
                    help="interface the bench PC itself may be joined to the "
                         "AP on (usually wlan0): the leg disconnects it before "
                         "arming the one-minute window and rejoins it after. "
                         "The bench laptop autojoins ESP-CCR from any earlier "
                         "web_probe, and a connected station makes the idle "
                         "rule CORRECTLY refuse to fire.")
    ap.add_argument("--dive-persist", action="store_true",
                    help="after the protocol checks, close a dive, read that dive "
                         "back out of flash and grade it against what the link "
                         "streamed while it ran, then reset the controller through "
                         "the console's EN line and ask `LOG LIST` again: dive "
                         "history must outlive the reset, and the file must hold "
                         "the verdict rows and a summary that agrees with its own "
                         "rows. Adds ~35 s and a deliberate reboot, so it is "
                         "opt-in.")
    args = ap.parse_args()

    if args.selftest:
        # No board, no ports: this section is about the grader, not the machine.
        fails = 0
        for name, ok_, detail in [*selftest_depth(), *selftest_scale(),
                                  *selftest_depth_source(), *selftest_wifi(),
                                  *selftest_density_lock(),
                                  *selftest_value_caps(),
                                  *selftest_dive_file(), *selftest_wifi_legs(),
                                  *selftest_control_margin(),
                                  *selftest_soak_contract(), *selftest_stream(),
                                  *selftest_stall_budget(),
                                  *selftest_matrix_window(),
                                  *selftest_port_hygiene(),
                                  *selftest_nvs_migration(),
                                  *selftest()]:
            print(f"  [{'PASS' if ok_ else 'FAIL'}] {name}" + ("" if ok_ else f": {detail}"))
            fails += 0 if ok_ else 1
        print(f"\n{fails} failure(s)")
        return 1 if fails else 0

    lines = []
    watch = []          # console lines from the protocol phase, see grade_watch
    watch_stamps = []   # (seconds since the watch started, line) for the log copy
    # `is not None`, not truthiness: `--replay ""` used to fall through into the
    # live branch and OPEN THE CONSOLE PORT, whose CP2102 reset pulse reboots the
    # controller. Happened on this bench 2026-11-28 with an empty shell variable.
    # A replay that was asked for but cannot be read must exit, never go looking
    # for a board.
    if args.replay is not None and not Path(args.replay).is_file():
        print(f"--replay: {args.replay!r} is not a readable file. Refusing to fall "
              f"back to the live path: that opens a port, and opening the controller "
              f"console resets the board.", file=sys.stderr)
        return 2
    if args.replay is not None:
        lines = Path(args.replay).read_text(errors="replace").splitlines()
        print(f"replaying {args.replay} ({len(lines)} lines)")
    else:
        import serial  # pipx platformio venv has it

        # A second reader on the console port corrupts soak logging and would
        # make this run meaningless. soak_logger.py must be stopped first —
        # there is no portable way to detect another reader on the port, so
        # this stays an explicit, loud rule of the bench, not a check.
        print(f"!! tools/soak_logger.py must NOT be running: it owns "
              f"{args.boot_port}, and two readers both misread. Ctrl-C it "
              f"first (pgrep -f soak_logger).")
        ser = serial.Serial(args.boot_port, args.baud, timeout=1.0)
        deadline = time.time() + args.boot_wait
        watcher = None
        try:
            # Classic auto-reset circuit: DTR drives IO0, RTS drives EN.
            # Pulsing DTR (as first drafted here did) latches the ROM into
            # download mode — a silent board. Release IO0, pulse EN: run-mode
            # reset, boot banner included.
            ser.setDTR(False)
            ser.setRTS(True)
            time.sleep(0.12)
            ser.setRTS(False)
            while time.time() < deadline:
                line = ser.readline().decode(errors="replace").rstrip()
                if line:
                    lines.append(line)
                    print(f"  | {line}")
        finally:
            if args.handset:
                # Do not close the console before the handset phase: a reader is
                # the only way to see a command crash the board. Keep one thread
                # reading it for the whole run.
                def _watch():
                    t0 = T_ORIGIN
                    while True:
                        try:
                            l = ser.readline().decode(errors="replace").rstrip()
                        except Exception:
                            return
                        if not l:
                            continue
                        watch.append(l)
                        # The copy written to *_watch.log is prefixed with the
                        # seconds since the watch started; `watch` itself stays
                        # plain text so the matchers keep working. Without the
                        # stamp, "0 'web: radio off' lines" could not be told
                        # apart from "the harness was not reading then" — same
                        # symptom, opposite conclusions.
                        watch_stamps.append((time.time() - t0, l))
                        print(f"  ~ [{time.time() - t0:6.2f}s] {l}")
                watcher = threading.Thread(target=_watch, daemon=True)
                watcher.start()
            else:
                ser.close()

    logdir = Path("logs")
    logdir.mkdir(exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    (logdir / f"accept_{stamp}_console.log").write_text("\n".join(lines) + "\n")

    failures = 0
    print("\n--- boot checks ---")
    for name, ok, detail in grade_boot(lines, args.expect_web, args.expect_depth):
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + ("" if ok else f": {detail}"))
        failures += 0 if ok else 1

    if args.handset:
        print("\n--- handset protocol checks (UART2) ---")
        proto_log = []
        proto_stream = []
        proto_pairs = []
        for name, ok, detail in run_protocol(args.handset, args.baud, proto_log,
                                             args.handset_settle, proto_stream,
                                             proto_pairs):
            print(f"  [{'PASS' if ok else 'FAIL'}] {name}: {detail}")
            failures += 0 if ok else 1
        (logdir / f"accept_{stamp}_handset.log").write_text("\n".join(proto_log) + "\n")

        # Facts the controller volunteers: graded over the whole stream, not over
        # one command's reply.
        print("\n--- unsolicited rows (the verdict, not the receipt) ---")
        for name, ok, detail in grade_stream(proto_stream):
            print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + ("" if ok else f": {detail}"))
            failures += 0 if ok else 1

        # One sensor, two views: the console prints the pressure, the handset
        # gets the metres. They come from the same sample, so a scale error in
        # the conversion shows up as the two disagreeing — which is precisely
        # how the 1000x bug looked, and nothing downstream could see it because
        # the setpoint bands and the deco engine were both acting on the wrong
        # metres consistently.
        if args.expect_depth:
            print("\n--- depth: the console's hPa and the wire's metres agree ---")
            for name, ok, detail in grade_depth_scale(lines, proto_stream):
                print(f"  [{'PASS' if ok else 'FAIL'}] {name}"
                      + ("" if ok else f": {detail}"))
                failures += 0 if ok else 1

        # A refusal is only half a lock. The matrix asked for a dive-locked ruler
        # write and got the words; these checks ask which value is actually in the
        # box, by pairing each write with its own reply and reading the density
        # back after the dive. Needs no depth sensor, so it is not under
        # --expect-depth.
        print("\n--- the dive-locked ruler: refused writes stay refused ---")
        for name, ok, detail in grade_density_lock(proto_pairs):
            print(f"  [{'PASS' if ok else 'FAIL'}] {name}"
                  + ("" if ok else f": {detail}"))
            failures += 0 if ok else 1

        # One sensor, two views again — this time about WHICH sensor. The console
        # announce is the boot decision, printed once; `GET DS` is that decision
        # plus its live state, seconds later. They must name the same chip, and
        # `used` must be true on a bench whose sensor is reading.
        print("\n--- depth source: the console's chip and the wire's chip are one chip ---")
        for name, ok_, detail in grade_depth_source(lines, proto_pairs, args.expect_depth):
            print(f"  [{'PASS' if ok_ else 'FAIL'}] {name}"
                  + ("" if ok_ else f": {detail}"))
            failures += 0 if ok_ else 1

        print("\n--- radio policy: three fields, one state, read twice ---")
        for name, ok_, detail in grade_wifi(proto_pairs, lines):
            print(f"  [{'PASS' if ok_ else 'FAIL'}] {name}"
                  + ("" if ok_ else f": {detail}"))
            failures += 0 if ok_ else 1

        print("\n--- NVS migration: a refused value must not be the one running ---")
        # The boot's refusal sentence and the live value are two answers to one
        # question, taken seconds apart. Pairs, not the raw stream: a bare
        # `VBAT_MIN=` reply on the stream is indistinguishable from a listing
        # line, and the whole point is which number is IN the chain.
        nvs_schema = next((r for c, r in proto_pairs if c == "GET SCHEMA"), [])
        nvs_read = {}
        for cmd, replies in proto_pairs:
            if not cmd.startswith("GET "):
                continue
            key = cmd[4:].strip()
            hit = next((l for l in replies if l.startswith(key + "=")), None)
            if hit:
                nvs_read[key] = hit.split("=", 1)[1].strip()
        for name, ok_, detail in grade_nvs_migration(lines, nvs_schema, nvs_read):
            print(f"  [{'PASS' if ok_ else 'FAIL'}] {name}"
                  + ("" if ok_ else f": {detail}"))
            failures += 0 if ok_ else 1

        print("\n--- value caps: an unwritable value is refused, not shortened ---")
        for name, ok, detail in grade_value_caps(proto_pairs):
            print(f"  [{'PASS' if ok else 'FAIL'}] {name}"
                  + ("" if ok else f": {detail}"))
            failures += 0 if ok else 1

        # Line framing, on the wire: see run_framing for why the matrix's own
        # over-long entry was not enough. Inside the console watch on purpose —
        # a framing bug that wedged the uart2 task would show up as a lost
        # stack headroom or a reboot, not as a wrong reply.
        print("\n--- line framing (over-long lines cannot execute) ---")
        frame_log = []
        for name, ok, detail in run_framing(args.handset, args.baud, frame_log,
                                            args.handset_settle):
            print(f"  [{'PASS' if ok else 'FAIL'}] {name}: {detail}")
            failures += 0 if ok else 1
        (logdir / f"accept_{stamp}_framing.log").write_text(
            "\n".join(frame_log) + "\n")

        # The calibration-specific console checks describe the MATRIX's traffic:
        # --cal-air asks for a calibration on purpose two minutes later, and a
        # "cal air: sampling" line from that must not read as the matrix's refused
        # request having started something. So the window is marked here.
        cal_mark = len(watch)

        # The boot banner's surface seq range is the baseline: the records this run
        # makes must extend it, which is the only way to tell "written" from
        # "already there from last time".
        surf_boot = next((l for l in lines if SURFACE_LINE.search(l)), None)
        surf_hi = surf_n = None
        if surf_boot:
            m = SURFACE_LINE.search(surf_boot)
            surf_hi = int(m.group("hi")) if m.group("hi") else 0
            surf_n = int(m.group("n")) if m.group("n") else 0

        if args.cal_air and not args.replay:
            # Before dive-persist: that phase resets the board, and this one has
            # two minutes of waiting that a reset would throw away.
            print("\n--- the air calibration's verdict (CALAIR, + E,*,7 rows) ---")
            cal_log = []
            for name, ok_, detail in run_cal_air(ser, args.handset, args.baud, watch,
                                                 args.handset_settle, cal_log,
                                                 args.web_host if args.expect_web else None,
                                                 surf_hi):
                print(f"  [{'PASS' if ok_ else 'FAIL'}] {name}" + ("" if ok_ else f": {detail}"))
                failures += 0 if ok_ else 1
            (logdir / f"accept_{stamp}_calair.log").write_text("\n".join(cal_log) + "\n")

        if args.predive and not args.replay:
            print("\n--- the pre-dive verdict, durable (surface log) ---")
            pre_log = []
            for name, ok_, detail in run_predive(args.handset, args.baud, watch,
                                                 args.handset_settle, pre_log,
                                                 args.web_host if args.expect_web else None,
                                                 surf_hi):
                print(f"  [{'PASS' if ok_ else 'FAIL'}] {name}" + ("" if ok_ else f": {detail}"))
                failures += 0 if ok_ else 1
            (logdir / f"accept_{stamp}_predive.log").write_text("\n".join(pre_log) + "\n")

        # Radio-policy legs (2026-11-26). Opt-in because --wifi-depth needs the
        # operator's thumb and both cost minutes. Run BEFORE the watch tail so
        # the console watcher is still open for their `wifi:` transition lines.
        if args.wifi_depth and not args.replay:
            print("\n--- radio policy: depth hold, on the real sensor (thumb squeeze) ---")
            wl = []
            cap = run_wifi_depth(args.handset, args.baud, watch,
                                 args.handset_settle, squeeze_wait=args.wifi_squeeze_wait,
                                 log=wl)
            for name, ok_, detail in grade_wifi_depth(cap):
                print(f"  [{'PASS' if ok_ else 'FAIL'}] {name}" + ("" if ok_ else f": {detail}"))
                failures += 0 if ok_ else 1
            (logdir / f"accept_{stamp}_wifi_depth.log").write_text("\n".join(wl) + "\n")
        if args.wifi_idle and not args.replay:
            print("\n--- radio policy: idle hold (one minute, no station) ---")
            wl = []
            cap = run_wifi_idle(args.handset, args.baud, watch, args.handset_settle,
                                log=wl, wlan=args.wifi_idle_wlan, boot_lines=lines)
            for name, ok_, detail in grade_wifi_idle(cap):
                print(f"  [{'PASS' if ok_ else 'FAIL'}] {name}" + ("" if ok_ else f": {detail}"))
                failures += 0 if ok_ else 1
            (logdir / f"accept_{stamp}_wifi_idle.log").write_text("\n".join(wl) + "\n")

        if args.watch_tail > 0:
            print(f"\n--- watching the console for {args.watch_tail:.0f} s "
                  f"after the last command ---")
            time.sleep(args.watch_tail)
        print("\n--- console watch (during the command phase) ---")
        for name, ok, detail in grade_watch(watch, lines, watch[:cal_mark]):
            print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + ("" if ok else f": {detail}"))
            failures += 0 if ok else 1
        (logdir / f"accept_{stamp}_watch.log").write_text(
            "\n".join(f"[{t:7.2f}s] {l}" for t, l in watch_stamps) + "\n")
    else:
        print("\n(handset checks skipped: pass --handset PORT)")
    cal_mark = len(watch)

    if args.handset and not args.replay and args.dive_persist:
        # Last, on purpose: the reset below puts a second boot banner in the
        # console stream, and the console-watch checks must have graded the
        # command phase's traffic before it is mixed with a deliberate reset.
        print("\n--- dive history across a reset ---")
        for name, ok, detail in run_dive_persist(ser, args.handset, args.baud, watch,
                                                 args.handset_settle, surf_n):
            print(f"  [{'PASS' if ok else 'FAIL'}] {name}: {detail}")
            failures += 0 if ok else 1

    print(f"\n{failures} failure(s)")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
