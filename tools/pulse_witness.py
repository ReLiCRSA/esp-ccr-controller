#!/usr/bin/env python3
"""Bench witness for the dive file's pulses window (review batch E, item 2).

Why this exists: `hw_accept.py`'s dive-file grader checks the `pulses` windows,
but a dive that never injected passes them vacuously — `M,` total 0 and a wall of
zeros is indistinguishable from a placeholder. The 2026-11-27 acceptance run was
exactly that case. This drives a dive the chain actually injects through, then
hands the export to the SAME grader `--dive-persist` uses (no second copy of the
arithmetic to drift), and refuses to call a dive with no injections a pass.

The gate to know about before running it: the injector only ever wants a pulse in
AUTO (`control/injector.cpp`), AUTO is locked out until the pre-dive sequence
PASSes, and its BATTERY step refuses outside a band — below `vbatMin` OR above the
chemistry's plausibility ceiling (`control/predive.cpp` S_BATTERY; the numbers come
from `VBAT_CELLS` in `include/config/defaults.h`, mirrored below). A board on USB
power alone (T,'s last field `1.56`, HB `batlow=1`) CANNOT produce an injecting
dive, and this script says so and exits 2 instead of printing a vacuous pass.
Connect the pack first.

`--selftest` needs no board: it grades this tool's own power gate against captured
console lines, in both directions, because a refusal that has never been seen firing
is not a refusal and a refusal that fires on a charged pack is a broken witness.

Exit codes: 0 witnessed (injections happened, at least one window non-zero, no
window over the total, grader clean) / 1 the grader or the window arithmetic
failed / 2 the witness was unreachable or vacuous (power, pre-dive, or a dive
that injected nothing).
"""
import re
import sys
import time

import serial

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from hw_accept import grade_dive_file, split_dive_rows  # noqa: E402

_ARGS = [a for a in sys.argv[1:] if not a.startswith("--")]
# Default is the S3 bench bridge (2026-11-28); the classic bridge answered ttyUSB1.
PORT = _ARGS[0] if _ARGS else "/dev/ttyACM1"
BAUD = 115200
RUN_S = 70.0          # two 30 s grid rows, plus the one written at dive start
# The pre-dive battery floor, derived from VBAT_CELLS in include/config/defaults.h
# (a 1S Li-ion at 3.40 V/cell). It used to read 10.5, which is the number that
# quietly refused every pre-dive on a rig running a 3.7 V pack -- the witness that
# needs the pack plugged in also has to ask about the right floor.
VBAT_MIN = 3.4
# The reading's plausibility ceiling, also from defaults.h (VBAT_CELL_IMPLAUSIBLE_V x
# VBAT_CELLS). It is here for the same reason the floor is: this witness exists to refuse
# instead of printing a PASS it cannot support, and a pack the board cannot read is not a
# pack that is fine. 2026-11-27 produced vbat=34.71 from a pinned sense pad, which clears
# any floor.
VBAT_MAX = 4.6


def read_vbat(lines):
    """First `HB `vbat=/batlow= or `T,`last-field in `lines`, as (volts, batlow).

    Two transports carry the same number and the tool may be pointed at either the
    controller console or the handset link, so both shapes are accepted; `batlow` is
    console-only and comes back None over `T,`.
    """
    for l in lines:
        m = (re.search(r"vbat=([\d.]+)", l) if l.startswith("HB ")
             else re.search(r",([\d.]+)\s*$", l) if l.startswith("T,") else None)
        if m:
            lb = re.search(r"batlow=(\d)", l)
            return float(m.group(1)), (int(lb.group(1)) if lb else None)
    return None, None


def power_verdict(vbat):
    """[] if an injecting dive is reachable at this reading; else the reasons to stop.

    A list, not a bool, because the two faults need different words and a reader who
    is standing at the bench has to know which one to go fix.
    """
    out = []
    if vbat < VBAT_MIN:
        out.append(f"the pre-dive BATTERY step refuses below vbatMin={VBAT_MIN} V, "
                   f"and only AUTO injects (control/predive.cpp S_BATTERY, "
                   f"control/injector.cpp). An injecting dive is unreachable here: "
                   f"connect the pack and re-run. Not a firmware fault.")
    # The fail-closed half, and the one this tool did not have until the firmware
    # grew it: above this line the READING is the fault, so the witness cannot tell
    # whether a pack is connected at all and must not report what a dive then
    # produces as a measurement.
    if vbat > VBAT_MAX:
        out.append(f"vbat={vbat:.2f} V is above the {VBAT_MAX} V ceiling for this "
                   f"chemistry: that is the sense node (an open divider, a loose sense "
                   f"pad), not a charged pack -- see docs/REVIEW.md 1 P1. Refusing "
                   f"rather than grading a dive whose battery verdict came from a "
                   f"number that is not a measurement.")
    return out


# (line as it came off the bench, want a refusal?, what the refusal must say)
# The 34.71 capture is not invented: it is the 2026-11-27 pinned-pad reading from
# docs/REVIEW.md 1 P1. The 4.60/4.61 pair is the boundary in both directions -- the
# ceiling is a fault detector, not a charge limit, so a full charge must pass.
POWER_CAPTURES = [
    ("HB seq=1 smp=8 i2cerr=0 rec=0 vbat=1.56 stale=1 batlow=1", True, "connect the pack"),
    ("HB seq=1 smp=8 i2cerr=0 rec=0 vbat=3.90 stale=0 batlow=0", False, ""),
    ("HB seq=1 smp=8 i2cerr=0 rec=0 vbat=4.60 stale=0 batlow=0", False, ""),
    ("HB seq=1 smp=8 i2cerr=0 rec=0 vbat=4.61 stale=0 batlow=0", True, "sense node"),
    ("HB seq=1 smp=8 i2cerr=0 rec=0 vbat=34.71 stale=0 batlow=0", True, "sense node"),
    ("T,0.00,0.217,0.219,0.217,0.217,0.400,STANDBY,FLAGS0x18,3.92", False, ""),
    ("T,0.00,0.217,0.219,0.217,0.217,0.400,STANDBY,FLAGS0x18,1.56", True, "connect the pack"),
]


def selftest_power_gate():
    """Grades this tool's own power gate without a board. Returns the failure count."""
    bad = 0
    refused = 0
    passed = 0
    for line, want_refusal, needle in POWER_CAPTURES:
        vbat, _ = read_vbat([line])
        if vbat is None:
            print(f"  FAIL  the parser reads nothing from: {line}")
            bad += 1
            continue
        why = power_verdict(vbat)
        got = bool(why)
        if got != want_refusal:
            print(f"  FAIL  vbat={vbat:.2f}: want "
                  f"{'refuse' if want_refusal else 'proceed'}, got "
                  f"{'refuse' if got else 'proceed'}")
            bad += 1
        elif got:
            refused += 1
            if needle not in " ".join(why):
                print(f"  FAIL  vbat={vbat:.2f} refuses but never says {needle!r}")
                bad += 1
        else:
            passed += 1
    # The guard against grading nothing, the pattern test_settings uses: a refusal
    # leg that fires on everything is as useless as one that fires on nothing.
    if refused < 2 or passed < 2:
        print(f"  FAIL  the captures must refuse at least 2 readings and pass at least "
              f"2 (refused {refused}, passed {passed}) -- one-sided captures prove nothing")
        bad += 1
    print(f"selftest: {len(POWER_CAPTURES)} power-gate captures, "
          f"{refused} refused, {passed} accepted, {bad} failures")
    return bad


def main() -> int:
    ser = serial.Serial(PORT, BAUD, timeout=0.2)
    time.sleep(0.4)
    ser.reset_input_buffer()

    def ask(cmd, wait=1.5, stop=None):
        if cmd:
            ser.write((cmd + "\r\n").encode())
            ser.flush()
        out, t0 = [], time.time()
        while time.time() - t0 < wait:
            ln = ser.readline().decode(errors="replace").strip()
            if ln:
                out.append(ln)
                if stop and stop(ln):
                    break
        return out

    # vbat reaches the handset link as `T,'s last field and the console's HB
    # carries `vbat=`/`batlow=`; either tells us whether an injecting dive is
    # reachable at all, so check it before driving a dive that cannot inject.
    vbat, batlow = read_vbat(ask("", 3.0))
    if vbat is not None:
        print(f"power: vbat={vbat:.2f} V" + (f" batlow={batlow}" if batlow is not None else ""))
        for why in power_verdict(vbat):
            print("  " + why)
            ser.close()
            return 2

    print("-- pre-dive, then AUTO --")
    print("  ", ask("CHECK", 2.0)[:1])
    verdict = None
    for _ in range(30):
        for l in ask("", 1.0):
            if re.match(r"^E,\d+,13,", l):
                verdict = l
        if verdict:
            break
    print("  ", verdict or "no pre-dive PASS row seen on the wire")
    print("  ", ask("AUTO", 1.5)[:1])

    # The settings the dive's M, row has to agree with, and the verdict rows
    # streamed while the dive is open: both feed grade_dive_file unchanged.
    want = {}
    for key in ("GF_LOW", "GF_HIGH", "SP1", "SP2", "SP3", "SP4"):
        r = ask(f"GET {key}", 1.2)
        for l in r:
            if l.startswith(key + "="):
                want[key] = l
                break

    print("-- dive --")
    print("  ", ask("DIVE START", 1.5)[:1])
    wire, end = [], time.time() + RUN_S
    while time.time() < end:
        ln = ser.readline().decode(errors="replace").strip()
        if ln.startswith("E,"):
            wire.append(ln)
    print("  ", next((l for l in ask("", 1.0) if l.startswith("HB")), "(no HB)"))
    print("  ", ask("DIVE END", 2.0)[:1])
    time.sleep(2.0)

    slots = [l.split(",") for l in ask("LOG LIST", 2.0) if l.startswith("L,")]
    if not slots:
        print("no slots in LOG LIST"); return 1
    newest = max(slots, key=lambda f: int(f[1]))
    rows = ask(f"LOG EXPORT {newest[1]}", 8.0, stop=lambda l: l.startswith("M,"))
    print(f"-- export slot {newest[1]}: {len(rows)} transport lines --")

    # One-way join, re-joined rows, the M, summary's own arithmetic: every claim
    # below is the acceptance harness's grader, not a copy of it.
    checks = grade_dive_file(rows, want, wire)
    bad = [c[0] for c in checks if not c[1]]
    for name, ok, detail in checks:
        if not ok:
            print(f"  [FAIL] {name}: {detail}")

    joined = split_dive_rows(rows)
    samp = [l.split(",") for l in joined if l.startswith("S,")]
    mrow = next((l.split(",") for l in joined if l.startswith("M,")), None)
    win = [int(f[13]) for f in samp if len(f) > 13 and f[13].isdigit()]
    total = int(mrow[5]) if mrow and len(mrow) > 5 and mrow[5].isdigit() else None
    print(f"\nS, rows {len(samp)}, windows {win}, M, total {total}")
    if total == 0:
        print("  RESULT: the chain injected nothing in this dive, so the window "
              "checks above witnessed nothing. Vacuous, not a pass: check the "
              "pre-dive verdict and the pack.")
        return 2
    nz = [w for w in win if w]
    over = [w for w in win if w > total]
    print(f"  non-zero windows       : {len(nz)} of {len(win)}")
    print(f"  windows over the total : {over or 'none'}")
    print(f"  grader                 : {len(checks) - len(bad)}/{len(checks)} passed")
    ok = bool(nz) and not over and not bad
    print(f"  RESULT: {'witnessed' if ok else 'NOT witnessed'}")
    ser.close()
    return 0 if ok else 1


if __name__ == "__main__":
    if "--selftest" in sys.argv:
        sys.exit(1 if selftest_power_gate() else 0)
    sys.exit(main())
