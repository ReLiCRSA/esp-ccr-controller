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
    # Batch E (docs/REVIEW.md 1 P9): the coil rail, appended after batlow — but
    # note WHERE the anchor stops: the regex ends at batlow= on purpose, so a
    # soak that straddles a firmware flash (or a reflash to older firmware
    # mid-drill, which soaks do) still parses every line it can. The optional
    # group carries the rail when the image has it — `.*?` spans the fields
    # that sit BETWEEN batlow= and the appended tail (vote=..cfggen=..),
    # because HB is append-only and vrail= is not adjacent to batlow=.
    # hw_accept.py keeps its own copy of this contract; the two are pinned
    # against each other by `hw_accept.py --selftest` — two graders of one
    # wire format drifting apart is the quiet way a 24 h capture becomes
    # 24 h of nothing.
    r"(?:.*? vrail=([\d.]+) railstate=(\d))?"
)
HB_PERIOD_S = 5.0
GAP_FAULT_S = 15.0  # >3 missed status lines -> treat as stall/reset event


def apply_hb(m, stats) -> dict:
    """Fold one HB_RE match into the soak stats. Split out of the read loop
    so the counting — the part a drift in the wire format breaks silently —
    is gradeable by --selftest without a port or a board. The rail counters
    stay SEPARATE from batlow on purpose: a rail complaint with the pack
    healthy is the §1 P9 case, and folding it into one word puts us back to
    not knowing which power rail a soak actually watched."""
    smp, cerr, rec = int(m.group(8)), int(m.group(9)), int(m.group(10))
    stats["max_i2cerr"] = max(stats["max_i2cerr"], cerr)
    stats["max_rec"] = max(stats["max_rec"], rec)
    if m.group(12) == "1":
        stats["stale_lines"] += 1
    if m.group(13) == "1":
        stats["batlow_lines"] += 1
    last = {
        "c": [int(m.group(i)) for i in (2, 3, 4)],
        "fresh": m.group(5) + m.group(6) + m.group(7),
        "smp": smp, "i2cerr": cerr, "rec": rec,
        "vbat": float(m.group(11)),
    }
    # groups 14/15: the rail tail (see HB_RE). Absent on a pre-batch-E
    # image; a 3 on a batch-E image is a statement about the BUILD, not a
    # fault — and the soak record should say how long that was true.
    vrail, railstate = m.group(14), m.group(15)
    if railstate is None:
        stats["rail_pre_batch_e_lines"] += 1
    else:
        last["vrail"] = float(vrail)
        last["railstate"] = int(railstate)
        if railstate == "1":
            stats["rail_low_lines"] += 1
        elif railstate == "2":
            stats["rail_high_lines"] += 1
        elif railstate == "3":
            stats["rail_unseen_lines"] += 1
    # last_stats mirrors the LAST line, not the last line with a rail: the
    # straddled-reflash soak must not carry a stale rail reading forward from
    # an image that no longer reports one.
    stats["last_stats"] = last
    return last


def selftest() -> int:
    """Grade apply_hb — the counting — without a port or a board. The regex
    contract itself is pinned in hw_accept.py --selftest (the two graders'
    HB_RE copies joined); this half pins what the soak SUMMARY says once a
    line has matched: every railstate counted under its own name, the
    straddled-reflash case (a pre-batch-E line clears the rail keys from
    last_stats rather than carrying a stale reading forward from an image
    that no longer reports one), and the pre-existing counters undisturbed."""
    base_stats = {
        "boots": 0, "safe_boots": 0, "fatal": 0, "hb_lines": 0, "gap_events": 0,
        "stale_lines": 0, "batlow_lines": 0,
        "rail_low_lines": 0, "rail_high_lines": 0, "rail_unseen_lines": 0,
        "rail_pre_batch_e_lines": 0,
        "last_stats": None, "max_i2cerr": 0, "max_rec": 0,
    }
    hb = ("HB st=1 c0=2050 c1=2049 c2=2051 fresh=111 smp=120 i2cerr=3 "
          "rec=1 vbat=3.64 stale=0 batlow=0 vote=1.320 sp=1.30 al=0x0000 "
          "mode=0 pulses=4 sol=0 ctrl=1 stk=1408 u2stk=1536 worst=45 "
          "logskip=0 surfskip=0 contskip=0 cfggen=4/4 vrail={v} railstate={r}")
    shapes = [("3", "rail_unseen_lines"), ("0", None), ("1", "rail_low_lines"),
              ("2", "rail_high_lines")]
    stats = dict(base_stats)
    fails = []
    for rstate, key in shapes:
        stats["hb_lines"] += 1
        m = HB_RE.match(hb.format(v="11.90" if rstate != "3" else "0.00", r=rstate))
        if not m:
            fails.append(f"railstate={rstate} line did not match HB_RE at all")
            continue
        last = apply_hb(m, stats)
        want = {"3": 0.0, "0": 11.9, "1": 11.9, "2": 11.9}[rstate]
        if last.get("vrail") != want or last.get("railstate") != int(rstate):
            fails.append(f"railstate={rstate}: last_stats {last}")
        if key and stats[key] != 1:
            fails.append(f"railstate={rstate} did not count under {key}")
    # the straddle: a pre-batch-E line after a rail line clears the keys
    stats["hb_lines"] += 1
    m = HB_RE.match(hb.split(" vrail=")[0])
    last = apply_hb(m, stats)
    if "vrail" in last or stats["rail_pre_batch_e_lines"] != 1:
        fails.append(f"pre-batch-E line did not clear/counter: {last}")
    # the pre-existing counters, undisturbed by any of this
    if (stats["max_i2cerr"] != 3 or stats["max_rec"] != 1
            or stats["stale_lines"] != 0 or stats["batlow_lines"] != 0):
        fails.append(f"legacy counters drifted: {stats}")
    for f in fails:
        print(f"[soak selftest] FAIL: {f}")
    print(f"[soak selftest] {5} fixtures, {len(fails)} failure(s)")
    return 1 if fails else 0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--hours", type=float, default=24.0)
    ap.add_argument("--logdir", default="logs")
    ap.add_argument("--selftest", action="store_true",
                    help="grade the HB counting on fixture lines and exit "
                         "(no port opened — safe next to a live bench)")
    args = ap.parse_args()
    if args.selftest:
        return selftest()

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
        # Rail accounting (docs/REVIEW.md 1 P9), kept separate from batlow:
        # a rail complaint with `vbat` healthy is the exact case this soak
        # exists to accumulate history for, and folding it into batlow would
        # put us back to one word for two faults. rail_unseen_lines counts
        # HB lines whose image reports railstate=3 — today's answer on an
        # unfitted board, and the soak record should say how long that was
        # true rather than leaving a reader to diff firmware dates.
        "rail_low_lines": 0,       # railstate=1
        "rail_high_lines": 0,      # railstate=2
        "rail_unseen_lines": 0,    # railstate=3 (not sensed)
        "rail_pre_batch_e_lines": 0,  # HB line with no rail field at all
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
                        last = apply_hb(m, stats)
                        smp, cerr, rec = last["smp"], last["i2cerr"], last["rec"]
                        vrail, railstate = m.group(14), m.group(15)
                        # live one-liner so `tail`/nohup.out stays readable
                        rail_txt = ("" if railstate is None else
                                    f" vrail={vrail} railstate={railstate}")
                        print(f"[soak] {time.strftime('%X')} cells={last['c']} "
                              f"fresh={last['fresh']} smp={smp} "
                              f"i2cerr={cerr} rec={rec} boots={stats['boots']}{rail_txt}")

            if time.monotonic() - last_hb > GAP_FAULT_S:
                stats["gap_events"] += 1
                last_hb = time.monotonic()  # re-arm, avoid spam
                print(f"[soak] {time.strftime('%X')} !! NO STATUS LINES for "
                      f">{GAP_FAULT_S:.0f}s (stall or reboot?)")

    stats["finished"] = datetime.now().isoformat()
    sum_path.write_text(json.dumps(stats, indent=2))
    print(f"[soak] done. boots={stats['boots']} safe={stats['safe_boots']} "
          f"fatal={stats['fatal']} gaps={stats['gap_events']} "
          f"rail(low/high/unseen)={stats['rail_low_lines']}/"
          f"{stats['rail_high_lines']}/{stats['rail_unseen_lines']} -> {sum_path}")
    clean = stats["boots"] <= 1 and stats["gap_events"] == 0
    return 0 if clean else 1


if __name__ == "__main__":
    sys.exit(main())
