#!/usr/bin/env python3
"""24 h soak harness for the ESP-CCR M1 board (PLAN.md M1 exit criteria).

Runs headless (no interactive terminal needed). Reads the status lines the
firmware prints every 5 s, logs everything raw, and watches for:
  * HB line gaps      -> SensorTask/HudTask stall or reboot
  * boot banners      -> counted as resets (yank SDA/SCL and see this move)
  * SAFE MODE / FATAL -> the fault-protection stack firing
  * stale=1 / i2cerr / rec growth -> bus health accounting per boot epoch

Exit code 0 = no unexpected resets in a clean soak (no yanking), 1 otherwise.

Run with PlatformIO's python (has pyserial):
  /home/wernervg/.local/share/pipx/venvs/platformio/bin/python \
      tools/soak_logger.py --port /dev/ttyUSB0 --hours 24

Fault-injection drill during the soak (do physically, note the time):
  1. Yank SCL for ~5 s  -> expect i2cerr/rec growth, stale=1, NO reset.
  2. Yank SDA mid-read  -> same, recovery clocks it free, NO reset.
  3. Replug cleanly     -> fresh=110 returns within ~1 s, counters frozen.
A reset during (1)-(3) is the failure the design promises never to have.
"""
import argparse
import json
import re
import sys
import time
from datetime import datetime
from pathlib import Path

import serial

HB_RE = re.compile(
    r"^HB st=(\d) c0=(-?\d+) c1=(-?\d+) c2=(-?\d+) fresh=(\d)(\d)(\d) "
    r"smp=(\d+) i2cerr=(\d+) rec=(\d+) vbat=([\d.]+) stale=(\d) batlow=(\d)"
)
HB_PERIOD_S = 5.0
GAP_FAULT_S = 15.0  # >3 missed status lines -> treat as stall/reset event


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--hours", type=float, default=24.0)
    ap.add_argument("--logdir", default="logs")
    args = ap.parse_args()

    logdir = Path(args.logdir)
    logdir.mkdir(exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    raw_path = logdir / f"soak_{stamp}.log"
    sum_path = logdir / f"soak_{stamp}_summary.json"

    stats = {
        "started": datetime.now().isoformat(),
        "boots": 0,
        "safe_boots": 0,
        "fatal": 0,
        "hb_lines": 0,
        "gap_events": 0,
        "stale_lines": 0,
        "batlow_lines": 0,
        "last_stats": None,
        "max_i2cerr": 0,   # max per-epoch counters seen
        "max_rec": 0,
    }

    ser = serial.Serial(args.port, args.baud, timeout=1.0)
    print(f"[soak] {time.strftime('%X')} logging raw to {raw_path}")
    t_end = time.monotonic() + args.hours * 3600.0
    last_hb = time.monotonic()

    with raw_path.open("a", buffering=1) as raw:
        buf = b""
        while time.monotonic() < t_end:
            chunk = ser.read(256)
            if chunk:
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    line = line.decode("utf-8", "replace").strip()
                    if not line:
                        continue
                    raw.write(f"[{time.strftime('%X')}] {line}\n")

                    if "M1 boot" in line:
                        stats["boots"] += 1
                        print(f"[soak] {time.strftime('%X')} BOOT #{stats['boots']}")
                    elif "SAFE MODE" in line:
                        stats["safe_boots"] += 1
                        print(f"[soak] {time.strftime('%X')} ** SAFE MODE **")
                    elif "FATAL" in line or "PRIOR FAULT" in line:
                        stats["fatal"] += 1
                        print(f"[soak] {time.strftime('%X')} ** {line} **")

                    m = HB_RE.match(line)
                    if m:
                        stats["hb_lines"] += 1
                        last_hb = time.monotonic()
                        smp, cerr, rec = int(m.group(8)), int(m.group(9)), int(m.group(10))
                        stats["max_i2cerr"] = max(stats["max_i2cerr"], cerr)
                        stats["max_rec"] = max(stats["max_rec"], rec)
                        if m.group(12) == "1":
                            stats["stale_lines"] += 1
                        if m.group(13) == "1":
                            stats["batlow_lines"] += 1
                        stats["last_stats"] = {
                            "c": [int(m.group(i)) for i in (2, 3, 4)],
                            "fresh": m.group(5) + m.group(6) + m.group(7),
                            "smp": smp, "i2cerr": cerr, "rec": rec,
                            "vbat": float(m.group(11)),
                        }
                        # live one-liner so `tail`/nohup.out stays readable
                        print(f"[soak] {time.strftime('%X')} cells={stats['last_stats']['c']} "
                              f"fresh={stats['last_stats']['fresh']} smp={smp} "
                              f"i2cerr={cerr} rec={rec} boots={stats['boots']}")

            if time.monotonic() - last_hb > GAP_FAULT_S:
                stats["gap_events"] += 1
                last_hb = time.monotonic()  # re-arm, avoid spam
                print(f"[soak] {time.strftime('%X')} !! NO STATUS LINES for "
                      f">{GAP_FAULT_S:.0f}s (stall or reboot?)")

    stats["finished"] = datetime.now().isoformat()
    sum_path.write_text(json.dumps(stats, indent=2))
    print(f"[soak] done. boots={stats['boots']} safe={stats['safe_boots']} "
          f"fatal={stats['fatal']} gaps={stats['gap_events']} -> {sum_path}")
    clean = stats["boots"] <= 1 and stats["gap_events"] == 0
    return 0 if clean else 1


if __name__ == "__main__":
    sys.exit(main())
