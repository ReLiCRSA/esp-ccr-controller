#!/usr/bin/env python3
"""Extract the built-in web page into data/ for LittleFS installs.

One source of truth: src/comms/web_page.h holds the page the firmware falls
back on, and this writes that same text to data/index.html (and style.css /
app.js if the page ever splits). Upload with:

    pio run -e esp32 -t uploadfs

Run it after editing web_page.h and commit both, or the installed copy and the
built-in copy drift — and the drift only shows up on the one board that has a
filesystem.
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src" / "comms" / "web_page.h"
DATA = ROOT / "data"


def page_text() -> str:
    text = SRC.read_text(encoding="utf-8")
    m = re.search(r'R"HTML\((.*)\)HTML"', text, re.S)
    if not m:
        raise SystemExit(f"no R\"HTML(...)HTML\" literal found in {SRC}")
    return m.group(1)


def main() -> int:
    html = page_text().lstrip()
    DATA.mkdir(exist_ok=True)
    out = DATA / "index.html"
    out.write_text(html, encoding="utf-8")
    print(f"wrote {out.relative_to(ROOT)} ({len(html)} bytes)")
    # Cheap integrity check: the page must speak the protocol it is served by.
    for token in ("GET T", "GET SCHEMA", "SET ", "LOG LIST", "/dive/"):
        if token not in html:
            print(f"warning: page no longer mentions {token!r}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
