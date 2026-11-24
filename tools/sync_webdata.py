#!/usr/bin/env python3
"""Extract the built-in web page into data/ as the bench's reference copy.

One source of truth: src/comms/web_page.h holds the only copy the firmware
serves, and this writes that same text to data/index.html. `tools/web_probe.py`
then fetches / over the AP and fails unless the bytes match, so the page in the
repo, the page in app flash and the page a diver reads cannot drift apart.

Do NOT install this onto the board. `pio run -e esp32 -t uploadfs` rewrites the
whole LittleFS partition, and the dive logs live on that partition: the way to
update a page is to build the firmware, which takes twenty seconds and keeps the
dive history. The route that would serve an installed copy was removed on
2026-11-24 for exactly that reason (PLAN 16), and a board may still be holding
an old installed copy that nothing reads.
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
