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
PASSes, and its BATTERY step refuses below `vbatMin` — default 10.5 V,
`control/predive.h`. A board on USB power alone (T,'s last field `1.56`, HB
`batlow=1`) CANNOT produce an injecting dive, and this script says so and exits 2
instead of printing a vacuous pass. Connect the pack first.

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

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB1"
BAUD = 115200
RUN_S = 70.0          # two 30 s grid rows, plus the one written at dive start
VBAT_MIN = 10.5       # predive::Config::vbatMin, the default in control/predive.h


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
    vbat = None
    for l in ask("", 3.0):
        m = (re.search(r"vbat=([\d.]+)", l) if l.startswith("HB ")
             else re.search(r",([\d.]+)\s*$", l) if l.startswith("T,") else None)
        if m:
            vbat = float(m.group(1))
            break
    if vbat is not None:
        print(f"power: vbat={vbat:.2f} V")
        if vbat < VBAT_MIN:
            print(f"  the pre-dive BATTERY step refuses below vbatMin={VBAT_MIN} V, "
                  f"and only AUTO injects (control/predive.cpp S_BATTERY, "
                  f"control/injector.cpp). An injecting dive is unreachable here: "
                  f"connect the pack and re-run. Not a firmware fault.")
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
    sys.exit(main())
