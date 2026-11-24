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


def grade_boot(lines, expect_web):
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
    if ll:
        # The suffix is the firmware's own warning. A LittleFS mount that fails
        # is answered with a format, which erases every dive on the partition;
        # until 2026-11-23 that happened in silence and was indistinguishable
        # from a diver who had simply not dived.
        note = LOG_LINE.search(ll).group("note").strip()
        checks.append(("dive log mount did not format (history intact)", not note, ll))
    cl = next((l for l in lines if CLOCK_LINE.search(l)), None)
    checks.append(("wall clock stated at boot", cl is not None,
                   cl or "no `clock:` line: the boot does not say whether dives "
                         "will carry a date"))
    return checks


def grade_watch(lines, boot_lines=None):
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


def run_protocol(port, baud, log, settle=1.5):
    """Open the handset adapter and run PROTOCOL_CHECKS. No pyserial import
    at module level: the boot/replay path must work without a handset.

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
                else:
                    deadline = time.time() + 3.0
                    while time.time() < deadline:
                        line = ser.readline().decode(errors="replace").strip()
                        if not line:
                            break
                        seen.append(line)
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


def run_dive_persist(console, port, baud, watch, settle=1.5):
    """Does a closed dive survive a reset?

    `dive_store.h` promises "a reset between dives must cost at most the last
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

    before = slots()
    ser.write(b"DIVE START\r\n")
    ser.flush()
    time.sleep(7.0)                    # one 30 s grid row will not land; event
    ser.write(b"DIVE END\r\n")        # rows and the header already make a dive
    ser.flush()
    time.sleep(3.0)
    mid = slots()

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
    ap.add_argument("--replay", default=None)
    ap.add_argument("--dive-persist", action="store_true",
                    help="after the protocol checks, close a dive, reset the "
                         "controller through the console's EN line and ask "
                         "`LOG LIST` again: dive history must outlive the reset. "
                         "Adds ~35 s and a deliberate reboot, so it is opt-in.")
    args = ap.parse_args()

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
    for name, ok, detail in grade_boot(lines, args.expect_web):
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + ("" if ok else f": {detail}"))
        failures += 0 if ok else 1

    if args.handset:
        print("\n--- handset protocol checks (UART2) ---")
        proto_log = []
        for name, ok, detail in run_protocol(args.handset, args.baud, proto_log,
                                             args.handset_settle):
            print(f"  [{'PASS' if ok else 'FAIL'}] {name}: {detail}")
            failures += 0 if ok else 1
        (logdir / f"accept_{stamp}_handset.log").write_text("\n".join(proto_log) + "\n")

        if args.watch_tail > 0:
            print(f"\n--- watching the console for {args.watch_tail:.0f} s "
                  f"after the last command ---")
            time.sleep(args.watch_tail)
        print("\n--- console watch (during the command phase) ---")
        for name, ok, detail in grade_watch(watch, lines):
            print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + ("" if ok else f": {detail}"))
            failures += 0 if ok else 1
        (logdir / f"accept_{stamp}_watch.log").write_text(
            "\n".join(f"[{t:7.2f}s] {l}" for t, l in watch_stamps) + "\n")
    else:
        print("\n(handset checks skipped: pass --handset PORT)")

    if args.handset and not args.replay and args.dive_persist:
        # Last, on purpose: the reset below puts a second boot banner in the
        # console stream, and the console-watch checks must have graded the
        # command phase's traffic before it is mixed with a deliberate reset.
        print("\n--- dive history across a reset ---")
        for name, ok, detail in run_dive_persist(ser, args.handset, args.baud, watch,
                                                 args.handset_settle):
            print(f"  [{'PASS' if ok else 'FAIL'}] {name}: {detail}")
            failures += 0 if ok else 1

    print(f"\n{failures} failure(s)")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
