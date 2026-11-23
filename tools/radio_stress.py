#!/usr/bin/env python3
"""Hammer the WiFi teardown/setup path and fail loudly if the driver asserts.

Why: the radio is designed to come DOWN at DIVE START (PLAN 9 counts milliamps
and the antenna in water), and doing `esp_wifi_stop()` from the task that is
answering a client — station still associated, driver mid-ioctl — raced the WiFi
driver's own worker and asserted inside it:

    assert failed: spinlock_acquire spinlock.h:122 (result == core_id || result == SPINLOCK_FREE)
    backtrace: semphr_give_wrapper <- ieee80211_ioctl_process <- ppTask

That rebooted the controller at the one moment the radio is meant to drop, and it
was intermittent: it showed up in 2 of 4 acceptance runs. comms::web::setRadio()
now only records the intent, loopTask performs the teardown after a driver drain.
This script is the regression check for that ordering, because a race that
"did not happen twice" is not fixed.

One WIFI OFF is one full esp_wifi_stop(); one WIFI ON is a full softAP. The
console port is watched the whole time (port_id.open_quiet, so opening it does
not reset the controller — the boot epoch stays intact).

    tools/radio_stress.py [--cycles 12]

Exit 0 = no fault markers, 1 = the console said otherwise.
"""
import argparse
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from port_id import open_quiet          # noqa: E402  (shared port handling)

MARKERS = ("assert failed", "Guru", "canary", "SAFE MODE", "reset reason",
           "Rebooting", "FATAL")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--console", default="/dev/ttyUSB0")
    ap.add_argument("--handset", default="/dev/ttyUSB1")
    ap.add_argument("--cycles", type=int, default=12)
    ap.add_argument("--off-dwell", type=float, default=1.6,
                    help="loopTask polls at 50 ms and stop() drains the driver "
                         "for 250 ms, so a cycle needs >0.3 s to land")
    args = ap.parse_args()

    con = open_quiet(args.console)
    hs = open_quiet(args.handset)
    lines = []
    stop = False

    def pump(stream):
        while not stop:
            l = stream.readline().decode(errors="replace").rstrip("\r\n")
            if l:
                lines.append(l)
            time.sleep(0.005)

    threading.Thread(target=pump, args=(con,), daemon=True).start()
    time.sleep(7.0)          # boot epoch, if the console open reset anything
    lines.clear()

    print(f"{args.cycles} radio teardown/setup cycles over {args.handset} "
          f"(console {args.console})")
    for i in range(args.cycles):
        hs.write(b"WIFI OFF\r\n"); hs.flush()
        time.sleep(args.off_dwell)
        hs.write(b"WIFI ON\r\n"); hs.flush()
        time.sleep(2.4)
        print(f"  cycle {i + 1:2d}/{args.cycles}")

    bad = [l for l in lines if any(m in l for m in MARKERS)]
    hb = [l for l in lines if l.startswith("HB ")]
    print(f"\nconsole lines: {len(lines)}   HB beats: {len(hb)}   fault markers: {len(bad)}")
    for l in bad[:10]:
        print(f"  !! {l[:140]}")
    if hb:
        print(f"last HB: {hb[-1]}")

    stop = True
    con.close()
    hs.close()
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
