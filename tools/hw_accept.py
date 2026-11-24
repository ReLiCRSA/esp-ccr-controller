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
      tools/hw_accept.py --expect-web --handset /dev/ttyUSB1

  --boot-port PORT   USB-CDC console (default /dev/ttyUSB0)
  --handset PORT     USB-UART adapter (or a second ESP32 running
                     handset/bridge_esp32) wired to UART2 16/17. Omit to skip
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
PROTOCOL_CHECKS = [
    ("DIVE END", r"^(OK|ERR no dive)", "clean slate before the sequence"),
    ("GET T", r"^T,", "telemetry line parses", "burst"),
    ("GET D", r"^D,", "deco picture on request", "burst"),
    ("GET *", r"^[A-Z0-9_]+=", "full settings listing"),
    ("GET SCHEMA", r"^K,SP2,F,", "the pages' ranges come from the server"),
    ("GET HIST", r"^(HIST,|ERR no histogram)", "histogram answer, or the truth"),
    ("GET FAULT", r"^FAULT,", "fuse/stuck-open/duty on request"),
    ("AUTO", r"^ERR predive required", "AUTO lockout on a cold boot"),
    ("SET SP2=0.55", r"^OK", "surface write lands"),
    ("GET SP2", r"^SP2=0\.550", "registry read-back"),
    ("SET SP2=2.5", r"^ERR", "range refusal"),
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
    ("WIFI ON", r"^ERR wifi locked in dive", "radio stays down in a dive"),
    ("CAL AIR", r"^ERR cal locked in dive", "calibration never under a diver"),
    ("FROBNICATE", r"^ERR unknown command", "unknown commands do nothing"),
    (OVERLONG, r"^ERR line too long", "line buffer cannot be flooded"),
    # dwell covers the re-arm: loopTask applies it within its poll interval and
    # softAP + captive DNS + server start are all inside that one console line.
    ("DIVE END", r"^OK DIVE", "dive closes", "dwell:4.0"),
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


def depth_from_bar_abs(bar_abs):
    """Depth from an absolute pressure, stated in the tool, from the physics.

    rho [kg/L] * g / 100 = bar per metre — the /100 is the whole lesson of
    2026-11-24: written /100000 with a kg/L density, every depth came out 1000x
    too deep and the bench dosed for 37 m in an open room.

    Sea-water density (1.025) is assumed, and that is honest at the surface,
    which is the only place this comparison can be made: the announce is a
    SURFACE reading, so the depth being compared is centimetres, and the whole
    legal density range (0.990..1.100) moves that by 0.004 m. A density that
    never reaches the conversion is therefore invisible here — the arithmetic is
    pinned on the host (test/test_depthscale), and C3-a's squeeze leg, holding a
    few metres while the density changes, is the wire demonstration.

    Never negative, mirroring the firmware's rule for what a diver reads: above
    the reference plane (altitude, weather, a sensor drifting high) the answer
    is 0 m, not a negative depth to dose against.
    """
    bar_per_m = 1.025 * 9.80665 / 100.0
    return max(0.0, (bar_abs - SURFACE_BAR) / bar_per_m)


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
    want = depth_from_bar_abs(hpa / 1000.0)
    return [("the depth on the wire is the announce's pressure",
             abs(shown - want) <= 0.05,
             f"`{ready.group(0)}` wants {want:.3f} m below the surface reference "
             f"({SURFACE_BAR} bar); the wire says {shown:.2f} m [{trow}] — "
             f"1016.7 hPa became `T,37.04` on 2026-11-24, which is a kg/L density "
             f"divided by 100000 instead of 100")]


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
    # never recorded. The three ids below are the whole vocabulary so far; anything
    # else must arrive as `id<n>`, never as a row with a name it did not earn.
    KNOWN = {"7": "cal_air", "13": "pre_dive_pass", "16": "pre_dive_fail"}
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
    ]
    checks = []
    for name, lines, stream, want in cases:
        got = grade_depth_scale(lines, stream)
        failed = [c[0] for c in got if not c[1]]
        if want is None:
            checks.append((f"depth scale: {name}", bool(got) and not failed,
                           f"expected graded checks, none failed: {failed}"))
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


def grade_dive_file(drows, want, wire=()):
    """Grade one exported dive against itself, and against the settings it claims.

    Four claims, each a different lie:

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
        sentinel as `-32.768` bar, in the same three decimals as a real reading.

    `want` is what the same link answered for GET GF_LOW / GF_HIGH / SP1..SP4,
    `wire` the `E,` rows seen live while the dive was open. A claim nobody could
    check (an empty `want`, a missing M, column) is a FAIL, never a pass.

    The join is deliberately ONE-WAY: every row the handset streamed while the
    dive was open must be in the file, but a row streamed outside that window
    (an alarm edge a second before `DIVE START`, the last ones drained after
    `DIVE END`) legitimately has no file row. Asserting equality in both
    directions would fail a dive that worked as designed.
    """
    samp = [l for l in drows if l.startswith("S,")]
    ev = [l for l in drows if l.startswith("E,")]
    meta = next((l for l in drows if l.startswith("M,")), "")
    mf = meta.split(",")

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

    # Multiset, not set: two identical rows (a rejoin for two different cells
    # reads alike in value but not in id, and a duplicate edge is a duplicate
    # bug) must both be in the file.
    missing = sorted((Counter(wire) - Counter(ev)).elements())
    it = iter(ev)
    in_order = all(any(w == x for x in it) for w in wire)

    checks = [
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
    ]
    checks = []
    for name, body, expect in cases:
        rows_want = [("cal_air", CAL_WANT), ("pre_dive_fail", [("value", 6, 4)])]
        hi = 2 if expect != "check-hi" else 4
        if expect == "console-seq":
            # Same verdict, wrong seq: the join must be on the number, not on "some
            # record was written this run".
            console = ["surface: record 9 id 7 value 2", "surface: record 4 id 16"]
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
        elif expect in ("console-noise", "console-seq"):
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
    return checks


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


def run_protocol(port, baud, log, settle=1.5, stream=None):
    """Open the handset adapter and run PROTOCOL_CHECKS. No pyserial import
    at module level: the boot/replay path must work without a handset.

    `stream`, if given, collects EVERY line the controller sent, including the
    unsolicited T,/D,/E, rows. Those are the evidence for facts the controller
    volunteers — a refused calibration, a clamp that engaged — which no command's
    own reply can show (`OK CAL AIR` is a receipt, not a verdict).

    `settle`: opening the port resets the board behind it. On a plain USB-UART
    adapter that costs nothing; on a second ESP32 running handset/bridge_esp32
    (the bench stand-in for an adapter) the open is a reset pulse and the ROM
    log arrives, so wait for it to boot and flush before the first command.
    A check that produced NO line at all is sent once more: the first bytes
    after a reset can be swallowed by the bridge's own bootloader."""
    import serial  # pipx platformio venv has it

    ser = serial.Serial(port, baud, timeout=1.0)
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
                    deadline = time.time() + 3.0
                    while time.time() < deadline:
                        line = ser.readline().decode(errors="replace").strip()
                        if not line:
                            break
                        seen.append(line)
                        if stream is not None:
                            stream.append(line)
                        log.append(f"[{time.time() - t0:7.2f}s] << {line}")
                        if pat.match(line):
                            ok = True
                            break
                if ok or seen:
                    break
                log.append(f"[{time.time() - t0:7.2f}s] (no line at all for "
                           f"{cmd!r}: retrying once)")
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
                       " — the cells did not settle in the spread band on this bench "
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
                         "announces and known-lying dive exports (no board "
                         "needed) to prove the graders can "
                         "fail. Run it after touching the graders or the formats.")
    ap.add_argument("--replay", default=None)
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
                                  *selftest_dive_file(),
                                  *selftest()]:
            print(f"  [{'PASS' if ok_ else 'FAIL'}] {name}" + ("" if ok_ else f": {detail}"))
            fails += 0 if ok_ else 1
        print(f"\n{fails} failure(s)")
        return 1 if fails else 0

    lines = []
    watch = []          # console lines from the protocol phase, see grade_watch
    watch_stamps = []   # (seconds since the watch started, line) for the log copy
    if args.replay:
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
        for name, ok, detail in run_protocol(args.handset, args.baud, proto_log,
                                             args.handset_settle, proto_stream):
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
