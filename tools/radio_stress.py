#!/usr/bin/env python3
"""Hammer the WiFi teardown/setup path and prove the radio actually re-arms.

Why: the radio is designed to come DOWN at DIVE START (PLAN 9 counts milliamps
and the antenna in water), and doing `esp_wifi_stop()` from the task that is
answering a client — station still associated, driver mid-ioctl — raced the WiFi
driver's own worker and asserted inside it:

    assert failed: spinlock_acquire spinlock.h:122 (result == core_id || result == SPINLOCK_FREE)
    backtrace: semphr_give_wrapper <- ieee80211_ioctl_process <- ppTask

That rebooted the controller at the one moment the radio is meant to drop, and it
was intermittent: it showed up in 2 of 4 acceptance runs. comms::web::setRadio()
now only records the intent, loopTask performs the teardown after a driver drain.

What this checks, and why the first version was not enough. It used to watch for
fault markers only, which means a `WIFI ON` that answered `OK` and left the AP
down would PASS — the state machine would be broken and the report would say the
opposite. A deferred action needs its completion proven, so every cycle is now
graded on the console lines the firmware prints for the radio itself:

    WIFI OFF  ->  "web: radio off (dive)"            within --max-down
    WIFI ON   ->  web: AP "<ssid>" on <ip> (heap N)  within --max-up

plus: the AP name identical every cycle (a MAC read that fails after a teardown
would silently rename the AP and invalidate a phone's saved network), no softAP
or captive-DNS bind failure, no esp_wifi_get_mac error, and heap reported across
cycles because each AP line carries it.

    tools/radio_stress.py [--cycles 12]

Exit 0 = every cycle did what the firmware said. 1 = it said otherwise.
Not a substitute for joining the AP from a phone: only a real client proves the
page and the WebSocket serve after a cycle (see BENCH.md).
"""
import argparse
import re
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from port_id import open_quiet          # noqa: E402  (shared port handling)

MARKERS = ("assert failed", "Guru", "canary", "SAFE MODE", "reset reason",
           "Rebooting", "FATAL")
BAD_LINES = {
    "** web: softAP failed": "AP would not come up",
    "** web: captive DNS failed to bind :53":
        "captive portal dead while the AP looks fine",
    "** web: server allocation failed": "no web server",
}

# Reported, not failed. ESP-IDF's own esp_wifi_stop() logs
#   E (21603) wifi_init_default: esp_wifi_get_mac failed with 12289
# (12289 = ESP_ERR_WIFI_NOT_INIT) partway through taking an AP down: the driver
# is already de-initialised when IDF reads the MAC for its own teardown bookkeeping.
# Verified it is not ours — `grep -rn esp_wifi_get_mac` over the Arduino WiFi
# library finds no caller at all (only WiFiProv, which this firmware does not
# use). What IS ours is a macAddress() read against a driver we have not started
# yet, and that shows up in a different place: AFTER "web: radio off" and BEFORE
# the next "web: AP". Only that position is a fault; see setup_path_mac_errors().
MAC_ERR = "esp_wifi_get_mac failed"
IDF_TEARDOWN_NOISE = MAC_ERR


# The tail is the four hex digits the firmware appends from the MAC. Without it
# every board on the bench advertises the same name, and a phone that joined one
# cannot tell them apart.
TAIL_RE = re.compile(r"-[0-9A-Fa-f]{4}$")
AP_RE = re.compile(r'^web: AP "(?P<ssid>[^"]*)" on (?P<ip>\S+) \(heap (?P<heap>\d+)\)')
OFF_RE = re.compile(r"^web: radio off")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--console", default="/dev/ttyUSB0")
    ap.add_argument("--handset", default="/dev/ttyUSB1")
    ap.add_argument("--log", default=None,
                    help="write the console capture here (timestamped). Debugging "
                         "a deferred action without a saved timeline costs time.")
    ap.add_argument("--cycles", type=int, default=12)
    ap.add_argument("--off-dwell", type=float, default=1.6,
                    help="loopTask polls at 50 ms and stop() drains the driver "
                         "for 250 ms, so a teardown needs >0.3 s to land")
    ap.add_argument("--max-down", type=float, default=3.0,
                    help="seconds allowed between WIFI OFF and the console "
                         "confirming the radio is down")
    ap.add_argument("--max-up", type=float, default=6.0,
                    help="seconds allowed between WIFI ON and the AP line "
                         "(softAP + DNS bind + server start are all in there)")
    args = ap.parse_args()

    con = open_quiet(args.console)
    hs = open_quiet(args.handset)
    lines = []            # (arrival_time, text)
    stop = False

    def pump(stream):
        while not stop:
            try:
                l = stream.readline().decode(errors="replace").rstrip("\r\n")
            except Exception:
                return
            if l:
                lines.append((time.time(), l))
            time.sleep(0.005)

    threading.Thread(target=pump, args=(con,), daemon=True).start()
    time.sleep(7.0)          # boot epoch, if the console open reset anything
    lines.clear()

    print(f"{args.cycles} radio teardown/setup cycles over {args.handset} "
          f"(console {args.console})")
    problems = []

    for i in range(args.cycles):
        # --- down ---
        sent = time.time()
        hs.write(b"WIFI OFF\r\n"); hs.flush()
        down_at = None
        while time.time() - sent < args.max_down:
            if any(t > sent and OFF_RE.search(l) for t, l in lines):
                down_at = time.time() - sent
                break
            time.sleep(0.05)
        if down_at is None:
            problems.append(f"cycle {i+1}: WIFI OFF answered but the console "
                            f"never said the radio was down ({args.max_down}s)")

        time.sleep(args.off_dwell - (down_at or 0))

        # --- up ---
        sent = time.time()
        hs.write(b"WIFI ON\r\n"); hs.flush()
        up_at = None
        while time.time() - sent < args.max_up:
            fresh = [l for t, l in lines if t > sent and AP_RE.search(l)]
            if fresh:
                up_at = time.time() - sent
                break
            time.sleep(0.05)
        if up_at is None:
            problems.append(f"cycle {i+1}: WIFI ON answered but no AP line "
                            f"({args.max_up}s) — the diver would surface to a "
                            f"web page that is not there")

        time.sleep(2.4 - (up_at or 0))
        print(f"  cycle {i+1:2d}/{args.cycles}: radio down in "
              f"{'-' if down_at is None else f'{down_at:.2f}'}s, "
              f"AP up in {'-' if up_at is None else f'{up_at:.2f}'}s")

    # ---------------- grading the whole run -------------------------------
    text = [l for _, l in lines]
    ap_lines = [AP_RE.match(l) for l in text if AP_RE.search(l)]
    ssids = {m.group("ssid") for m in ap_lines}
    heaps = [int(m.group("heap")) for m in ap_lines]

    idf = [l for l in text if IDF_TEARDOWN_NOISE in l]
    if idf:
        print(f"note: {len(idf)}x ESP-IDF teardown MAC line (not ours, see the "
              f"tool's header): {idf[0][:100]}")

    for marker, meaning in BAD_LINES.items():
        hit = next((l for l in text if marker in l), None)
        if hit:
            # Quote the line itself: the marker says which rule fired, the line
            # says what the firmware was doing when it fired, and those are not
            # the same place.
            problems.append(f"console said {marker!r}: {meaning}\n"
                            f"       line: {hit[:150]}")

    faults = [l for l in text if any(m in l for m in MARKERS)]
    for l in faults[:5]:
        problems.append(f"fault marker: {l[:140]}")

    downs = sum(1 for l in text if OFF_RE.search(l))
    ups = len(ap_lines)
    if downs != args.cycles or ups != args.cycles:
        problems.append(f"radio transitions: {downs} down / {ups} up, expected "
                        f"{args.cycles} each — the intent is recorded but not "
                        f"always carried out")
    if len(ssids) > 1:
        problems.append(f"AP renamed between cycles: {sorted(ssids)} — a phone "
                        f"that joined once cannot rejoin")
    no_tail = [n for n in ssids if not TAIL_RE.search(n)]
    if no_tail:
        problems.append(f"AP name without a MAC tail: {sorted(no_tail)} — every "
                        f"board on the bench then advertises the same network")

    print(f"\nconsole lines: {len(text)}   radio down: {downs}   AP up: {ups}")
    if ssids:
        print(f"AP name: {'consistent' if len(ssids) == 1 else 'INCONSISTENT'} "
              f"({sorted(ssids)})")
    if len(heaps) >= 2:
        drift = heaps[0] - heaps[-1]
        print(f"heap at first AP up: {heaps[0]}   last: {heaps[-1]}   "
              f"drift: {drift:+d} bytes over {args.cycles} cycles")
        if drift > 2048:
            problems.append(f"heap fell {drift} B across {args.cycles} cycles: "
                            f"the teardown/setup path is leaking")

    if args.log:
        t0 = lines[0][0] if lines else 0.0
        Path(args.log).write_text(
            "\n".join(f"[{t - t0:7.2f}s] {l}" for t, l in lines) + "\n")
        print(f"console capture: {args.log}")

    if problems:
        print(f"\nFAIL ({len(problems)}):")
        for pr in problems:
            print(f"  !! {pr}")
        return 1
    print("\nPASS: every cycle tore down and re-armed, same AP name every time")
    return 0


if __name__ == "__main__":
    sys.exit(main())
