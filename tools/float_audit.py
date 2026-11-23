#!/usr/bin/env python3
"""Float printf is a stack bomb on this device — check that none came back.

Why this script exists, in one line: on 2026-11-23 the first `GET T` from a real
handset panicked the controller with "Stack canary watchpoint triggered
(uart2)". newlib's float conversion (_dtoa_r through cvt/_vsnprintf_r) wants
kilobytes of stack, and every task that answers a client has a few. The first fix
removed the seven %f in the telemetry line and the crash RECURRRED — the culprit
then was _svfprintf_r itself, plus a %.6g hiding in GET SCHEMA. A bug class that
returns once is a bug class that will return again, so the rule is enforced by a
command instead of by memory:

    float format specifiers (%f %F %e %E %g %G) are not allowed in src/,
    except on lines marked  /*float-ok: <why>*/ .

Everything the device puts on the handset wire, in a dive log, or on the console
goes through util/numfmt.cpp instead, whose writers use integer digits and cannot
grow. Renderers that genuinely need %g (the settings schema, whose values are
compile-time constants) run once at boot from Session::begin, before the
small-stack tasks exist — that is what the marker is for.

Run it before flashing. Exit 0 = clean, 1 = violations listed.
"""
import re
import sys
from pathlib import Path

SRC = Path(__file__).resolve().parent.parent / "src"
SKIP_NAMES = {"web_page.h"}          # the browser page: JS, not firmware
FLOAT_SPEC = re.compile(r"%[-+ #0]*[0-9]*(?:\.[0-9]+)?(?:ll|l|L|h)?[fFeEgG]")
MARKER = "/*float-ok"          # the comment continues: "/*float-ok: <why>*/"

C_STYLE = re.compile(r"^\s*(//|/\*|\*)")


def main() -> int:
    bad = []
    marked = 0
    for path in sorted(SRC.rglob("*")):
        if not path.is_file() or path.suffix not in (".cpp", ".h") or path.name in SKIP_NAMES:
            continue
        for n, line in enumerate(path.read_text(errors="replace").splitlines(), 1):
            if C_STYLE.match(line):        # a comment ABOUT float printf is fine
                continue
            if not FLOAT_SPEC.search(line):
                continue
            if MARKER in line:
                marked += 1
                continue
            bad.append(f"{path.relative_to(SRC.parent)}:{n}: {line.strip()[:110]}")

    print(f"float-format audit: {marked} marked exception(s)")
    if bad:
        print(f"VIOLATIONS ({len(bad)}): float printf on a task with a fixed stack "
              f"has rebooted this controller twice.\n")
        for b in bad:
            print(f"  {b}")
        print("\nUse numfmt::fixed() / numfmt::fixedScaled(). If a renderer really "
              "must use %g, it must run at boot before the tasks start, and mark "
              "the line with /*float-ok: why*/ and say why.")
        return 1
    print("clean: no unmarked float format in src/")
    return 0


if __name__ == "__main__":
    sys.exit(main())
