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
    ("DIVE START", r"^OK DIVE", "dive opens"),
    ("SET SP2=0.40", r"^ERR dive locked", "diver outranks the client"),
    ("WIFI ON", r"^ERR wifi locked in dive", "radio stays down in a dive"),
    ("CAL AIR", r"^ERR cal locked in dive", "calibration never under a diver"),
    ("FROBNICATE", r"^ERR unknown command", "unknown commands do nothing"),
    (OVERLONG, r"^ERR line too long", "line buffer cannot be flooded"),
    ("DIVE END", r"^OK DIVE", "dive closes"),
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


def _read_line(ser, seen, log):
    line = ser.readline().decode(errors="replace").strip()
    if line:
        seen.append(line)
        log.append(f"<< {line}")
    return line


def _await_burst(ser, pat, seen, log, send):
    """True only if the controller ANSWERED, not merely kept streaming.

    Waits for a streamed line as a cadence anchor, sends, then looks for a
    matching line within BURST_WINDOW_S of the send. See the constants above for
    why the window separates reply from unsolicited traffic.
    """
    deadline = time.time() + BURST_ANCHOR_S
    anchored = False
    while time.time() < deadline:
        line = _read_line(ser, seen, log)
        if not line:
            break
        if pat.match(line):
            anchored = True
            break
    if not anchored:
        log.append(f"(no {pat.pattern!r} line to anchor on: the downlink is dead)")
        return False

    send()
    sent_at = time.time()
    while time.time() - sent_at < BURST_REPLY_S:
        line = _read_line(ser, seen, log)
        if not line:
            break
        if pat.match(line):
            if time.time() - sent_at <= BURST_WINDOW_S:
                return True
            log.append("(matching line arrived too late to be the reply)")
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
    try:
        time.sleep(settle)
        ser.reset_input_buffer()
        for check in PROTOCOL_CHECKS:
            cmd, want, why = check[0], check[1], check[2]
            burst = len(check) > 3 and check[3] == "burst"
            pat = re.compile(want)
            seen = []
            ok = False
            for attempt in (1, 2):
                ser.write((cmd.strip() + "\r\n").encode())
                ser.flush()
                if burst:
                    payload = (cmd.strip() + "\r\n").encode()
                    ok = _await_burst(ser, pat, seen, log,
                                      lambda p=payload: (ser.write(p), ser.flush()))
                else:
                    deadline = time.time() + 3.0
                    while time.time() < deadline:
                        line = ser.readline().decode(errors="replace").strip()
                        if not line:
                            break
                        seen.append(line)
                        log.append(f"<< {line}")
                        if pat.match(line):
                            ok = True
                            break
                if ok or seen:
                    break
                log.append(f"(no line at all for {cmd!r}: retrying once)")
            got = "; ".join(seen[-4:]) if seen else "(no reply in 3 s)"
            results.append((cmd if len(cmd) < 24 else cmd[:20] + "...", ok,
                            f"want {want}, got {got} — {why}"))
        ser.reset_input_buffer()
    finally:
        ser.close()
    return results


def main() -> int:
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
    args = ap.parse_args()

    lines = []
    watch = []          # console lines from the protocol phase, see grade_watch
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
                    while True:
                        try:
                            l = ser.readline().decode(errors="replace").rstrip()
                        except Exception:
                            return
                        if not l:
                            continue
                        watch.append(l)
                        print(f"  ~ {l}")
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
        (logdir / f"accept_{stamp}_watch.log").write_text("\n".join(watch) + "\n")
    else:
        print("\n(handset checks skipped: pass --handset PORT)")

    print(f"\n{failures} failure(s)")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
