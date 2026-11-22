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
  --handset PORT     USB-UART adapter wired to UART2 16/17 (omit to skip
                     the protocol checks — boot checks still run)
  --expect-web       require the settings/session/web boot lines (post-A5d
                     firmware; M1 does not print them)
  --boot-wait SECS   console capture window (default 20)
  --replay FILE      grade a captured console log instead of opening a port
                     (host-side development of the boot checks; no hardware)

Exit 0 = every check passed. Non-zero = at least one failed, and the failing
lines are listed. Raw console capture is kept under logs/.
"""
import argparse
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

# (command, expected reply-prefix regex, why it is on the list)
PROTOCOL_CHECKS = [
    ("DIVE END", r"^(OK|ERR no dive)", "clean slate before the sequence"),
    ("GET T", r"^T,", "telemetry line parses"),
    ("GET SCHEMA", r"^K,SP2,F,", "the pages' ranges come from the server"),
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
        checks.append(("settings loaded from NVS", sl is not None, sl or ""))
        cl = next((l for l in lines if CONTROL_LINE.search(l)), None)
        checks.append(("control chain booted", cl is not None, cl or ""))
        wl = next((l for l in lines if WEB_AP_LINE.search(l)), None)
        checks.append(("web AP announced", wl is not None, wl or ""))
    return checks


def run_protocol(port, baud, log):
    """Open the handset adapter and run PROTOCOL_CHECKS. No pyserial import
    at module level: the boot/replay path must work without a handset."""
    import serial  # pipx platformio venv has it

    ser = serial.Serial(port, baud, timeout=1.0)
    results = []
    try:
        time.sleep(0.2)
        ser.reset_input_buffer()
        for cmd, want, why in PROTOCOL_CHECKS:
            ser.write((cmd.strip() + "\r\n").encode())
            ser.flush()
            deadline = time.time() + 3.0
            seen = []
            pat = re.compile(want)
            ok = False
            while time.time() < deadline:
                line = ser.readline().decode(errors="replace").strip()
                if not line:
                    break
                seen.append(line)
                log.append(f"<< {line}")
                if pat.match(line):
                    ok = True
                    break
            got = "; ".join(seen) if seen else "(no reply in 3 s)"
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
    ap.add_argument("--expect-web", action="store_true")
    ap.add_argument("--boot-wait", type=float, default=20.0)
    ap.add_argument("--replay", default=None)
    args = ap.parse_args()

    lines = []
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
        try:
            ser.setDTR(True)   # ESP32 boards: DTR pulse resets into the boot log
            time.sleep(0.1)
            ser.setDTR(False)
            while time.time() < deadline:
                line = ser.readline().decode(errors="replace").rstrip()
                if line:
                    lines.append(line)
                    print(f"  | {line}")
        finally:
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
        for name, ok, detail in run_protocol(args.handset, args.baud, proto_log):
            print(f"  [{'PASS' if ok else 'FAIL'}] {name}: {detail}")
            failures += 0 if ok else 1
        (logdir / f"accept_{stamp}_handset.log").write_text("\n".join(proto_log) + "\n")
    else:
        print("\n(handset checks skipped: pass --handset PORT)")

    print(f"\n{failures} failure(s)")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
