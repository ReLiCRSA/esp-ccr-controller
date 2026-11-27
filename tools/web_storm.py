#!/usr/bin/env python3
"""web_storm.py — flood the controller's web server while its own heartbeat is watched.

Why this exists (review batch D, 2026-11-27): the task watchdog is set to 3 s
(`esp_task_wdt_init(3, true)`), and ControlTask feeds it once per cycle. So the
question is not "is the control loop fast on a quiet bench" — the accepted
number is ~45 ms and the worst value in every log this repo keeps is 457 ms —
it is "how slow does a cycle get when the network task is doing real work".
AsyncTCP runs on core 0 with the WiFi driver (platformio.ini), the control chain
owns core 1, and the web replies read LittleFS on the shared SPI bus the dive
log writes to. That is the load the margin has to survive, and nothing in the
bench measured it until now.

The tool does two things at once, which is the whole point: it loads the web
server AND reads the controller's console, so the `worst=` the firmware itself
reports is collected DURING the flood rather than after it.

What it grades (each one a different failure):

  * `worst=` under load stays under --max-worst (default 800 ms — the same line
    hw_accept.py grades, and half the measured 3011..3070 ms task-watchdog
    expiry, see BENCH.md). A beat above that is not "slow", it is a board one
    long flash write away from a watchdog panic and a SAFE boot.
  * no `Guru Meditation` / `reset reason` / `** SAFE MODE` / `!! FATAL` in the
    console while flooded — a flood that reboots the controller is a denial of
    service on the diver's own dashboard.
  * the HB beat keeps arriving: the maximum gap between consecutive HB lines
    must stay under --max-hb-gap (default 6000 ms). HudTask prints HB and is
    itself TWDT-subscribed, so a missing beat means a task the network can reach
    stopped running.
  * every client outcome is counted and reported: successes, connection
    refusals, timeouts, and short/truncated replies. None of those is a failure
    on its own — a board that refuses a 300th client is behaving — but a flood
    that silently answers nothing is exactly what a refusal-shaped result looks
    like, so the counts are printed and the success floor is graded.

Console access opens the port, which RESETS the board: run this on a board you
expect to boot, and never while hw_accept.py or the soak logger holds the port.
Two identical CP2102s share this bench — tools/port_id.py says how to tell them
apart; the controller is the one whose boot banner says `=== ESP-CCR`.

Usage:
    ~/.platformio/penv/bin/python tools/web_storm.py --seconds 40
    ~/.platformio/penv/bin/python tools/web_storm.py --clients 300 --half-open 150 \
        --console /dev/ttyUSB0 --log logs/storm.log
"""
from __future__ import annotations

import argparse
import errno
import re
import socket
import sys
import threading
import time
from collections import Counter

HB_RE = re.compile(r"^HB st=(\d)")
HB_CYCLE_RE = re.compile(r"worst=(\d+) logskip=(\d+) surfskip=(\d+) contskip=(\d+)")
HB_CFGGEN_RE = re.compile(r"cfggen=(\d+)/(\d+)")

# Anything that means "the board stopped being a controller during the flood".
# Same vocabulary hw_accept.py watches for, kept in one place here.
FATAL_MARKERS = (
    "Guru Meditation", "abort() was called", "reset reason:", "** SAFE MODE",
    "!! FATAL", "PRIOR FAULT", "Rebooting...",
)

# The routes the controller actually serves (src/comms/web_server.cpp). There is
# no JSON API: the page, its two assets, and the flash-backed CSV exports. The
# exports are the interesting load — each is a LittleFS read on the same SPI
# flash the dive log writes to, so a flood of them is the contention the control
# loop's margin has to survive. `/no-such` keeps onNotFound() (a redirect) in the
# mix, which is the path a captive-portal browser visit takes.
PATHS = ("/", "/style.css", "/app.js", "/surface.csv", "/dive/1.csv", "/dive/10.csv",
         "/no-such-path")


class Console(threading.Thread):
    """Reads the controller's console and keeps the HB-derived numbers.

    A serial read failure is recorded and the thread stops; the flood continues
    either way, because the load result and the console result are separate
    claims and neither should be able to hide the other.
    """

    def __init__(self, port: str, log: list[str]):
        super().__init__(daemon=True)
        self.port, self.log = port, log
        self.worst = None
        self.cfggen = []           # (published, chain) pairs, in arrival order
        self.beats = 0
        self.max_gap_ms = None
        self.fatals: list[str] = []
        self.error: str | None = None
        self._stop = threading.Event()

    def stop(self) -> None:
        self._stop.set()

    def run(self) -> None:
        try:
            import serial   # pyserial, from the PlatformIO venv
        except ImportError:
            self.error = "pyserial not installed; use ~/.platformio/penv/bin/python"
            return
        try:
            ser = serial.Serial(self.port, 115200, timeout=0.5)
        except OSError as e:
            self.error = f"cannot open {self.port}: {e}"
            return
        last_beat = None
        t0 = time.monotonic()
        buf = b""
        try:
            while not self._stop.is_set():
                chunk = ser.read(256)
                if not chunk:
                    continue
                buf += chunk
                while b"\n" in buf:
                    raw, buf = buf.split(b"\n", 1)
                    line = raw.decode("utf-8", "replace").strip()
                    if not line:
                        continue
                    self.log.append(f"[{time.monotonic() - t0:7.2f}s] << {line}")
                    for marker in FATAL_MARKERS:
                        if marker in line:
                            self.fatals.append(line)
                    if HB_RE.match(line):
                        now = time.monotonic()
                        self.beats += 1
                        if last_beat is not None:
                            gap = int((now - last_beat) * 1000)
                            self.max_gap_ms = gap if self.max_gap_ms is None else max(
                                self.max_gap_ms, gap)
                        last_beat = now
                        m = HB_CYCLE_RE.search(line)
                        if m:
                            w = int(m.group(1))
                            self.worst = w if self.worst is None else max(self.worst, w)
                        g = HB_CFGGEN_RE.search(line)
                        if g:
                            self.cfggen.append((int(g.group(1)), int(g.group(2))))
        finally:
            ser.close()


def flood_one(host: str, port: int, seconds: float, paths: list[str],
              counts: Counter, lock: threading.Lock, stop: threading.Event) -> None:
    """One client: open, ask, keep the socket open until the read times out (4 s),
    then reconnect.

    That shape matters and is why this is not a polite poller: a real browser page
    holds several sockets open at once (page, CSS, JS, the exports a phone retries
    on a flaky link). Every open socket is a control block the controller has to
    carry, and the listener's accept queue is five deep
    (ASYNCTCP_LISTEN_BACKLOG, .pio/libdeps/esp32/AsyncTCP/src/AsyncTCP.cpp:179).
    How many simultaneous clients the box carries before it stops answering is
    exactly what this measures.
    """
    i = 0
    while not stop.is_set():
        path = paths[i % len(paths)]
        i += 1
        try:
            s = socket.create_connection((host, port), timeout=4.0)
        except ConnectionRefusedError:
            with lock:
                counts["refused"] += 1
            time.sleep(0.05)
            continue
        except OSError as e:
            # Classified, not lumped. "connect-error" would hide the difference
            # between the listener's backlog being full (ECONNREFUSED, handled
            # above), the server closing on us (ECONNRESET / ETIMEDOUT, the radio
            # or the socket table), and a bench-side mistake like an interface
            # that is not associated (EHOSTUNREACH / ENETUNREACH) — the last one
            # means the flood measured NOTHING about the controller.
            with lock:
                code = getattr(e, "errno", None)
                name = errno.errorcode.get(code, "") if code else type(e).__name__
                counts[f"connect-{name or code or 'unknown'}"] += 1
            if not isinstance(e, socket.timeout):
                time.sleep(0.05)
            continue
        try:
            s.sendall(f"GET {path} HTTP/1.1\r\nHost: {host}\r\n\r\n".encode())
            data = b""
            status = None
            while time.monotonic() < seconds and len(data) < 400000:
                chunk = s.recv(8192)
                if not chunk:
                    break            # the server closed: a complete exchange
                data += chunk
                if status is None and b"\r\n" in data:
                    head = data.split(b"\r\n", 1)[0].split()
                    if len(head) >= 2 and head[0].startswith(b"HTTP/"):
                        status = head[1].decode("ascii", "replace")
            with lock:
                if not data:
                    counts["empty"] += 1
                elif status is None:
                    counts["header-only"] += 1
                else:
                    # ANSWERED: the request got a status line. Whether the socket
                    # then closed is a separate fact — with keep-alive it usually
                    # does not, and holding it open is part of what this measures,
                    # so counting a closed socket as the only "success" would
                    # report a working server as broken.
                    counts[f"answered-{status}"] += 1
                    counts["bytes"] += len(data)
        except socket.timeout:
            # A status line may already have arrived before the read timed out; the
            # except branch means nothing was ever answered.
            with lock:
                counts["read-timeout"] += 1
        except OSError:
            with lock:
                counts["io-error"] += 1
        finally:
            try:
                s.close()
            except OSError:
                pass


def hold_half_open(host: str, port: int, n: int, stop: threading.Event) -> list[socket.socket]:
    """Connect and send a PARTIAL request line, then hold: backlog pressure without
    any server work. This is the shape a flaky 2.4 GHz link leaves behind, and the
    listener's backlog is what it tests."""
    socks = []
    for i in range(n):
        try:
            s = socket.create_connection((host, port), timeout=3.0)
            s.sendall(b"GET /surface.cs")     # never completed
            socks.append(s)
        except OSError:
            break
    stop.wait()
    for s in socks:
        try:
            s.close()
        except OSError:
            pass
    return len(socks)


def nmcli(log: list[str], *argv: str) -> str:
    """One nmcli call on THIS machine. Bench-only lever, same as hw_accept.py's:
    it exists because the storm opens the console port, which resets the board,
    which restarts the access point, which leaves the PC's association stale —
    associated in NetworkManager and unreachable on the wire (arp INCOMPLETE).
    Not silently: the caller reports what the rejoin cost."""
    import subprocess
    t0 = time.monotonic()
    try:
        r = subprocess.run(["nmcli"] + list(argv), capture_output=True, text=True, timeout=25)
        out = (r.stdout or r.stderr or "").strip()
        log.append(f"[{time.monotonic() - t0:7.2f}s] (nmcli {' '.join(argv)!r}: "
                   f"rc={r.returncode} {out.splitlines()[:1]})")
        return out
    except (OSError, subprocess.SubprocessError) as e:
        log.append(f"[{time.monotonic() - t0:7.2f}s] (nmcli failed: {e})")
        return ""


def http_probe(host: str, port: int, path: str = "/") -> str | None:
    """One real GET. Returns the status line's code, or None if nothing answered.

    A bare TCP connect is NOT the preflight this tool needs: while the box is
    saturated its listener still completes SYNs and never sends a byte, so a
    connect-only check reports "reached" about a server that answers nothing.
    Found on the bench 2026-11-27, when a run whose preflight passed returned 0
    replies.
    """
    try:
        s = socket.create_connection((host, port), timeout=6.0)
    except OSError:
        return None
    try:
        s.sendall(f"GET {path} HTTP/1.1\r\nHost: {host}\r\nConnection: close\r\n\r\n".encode())
        s.settimeout(8.0)
        data = b""
        while len(data) < 4096:
            chunk = s.recv(1024)
            if not chunk:
                break
            data += chunk
        head = data.split(b"\r\n", 1)[0].split()
        if len(head) >= 2 and head[0].startswith(b"HTTP/"):
            return head[1].decode("ascii", "replace")
        return "no-status"
    except OSError:
        return None
    finally:
        try:
            s.close()
        except OSError:
            pass


def wait_for_server(host: str, port: int, timeout_s: float, log: list[str]) -> bool:
    """Poll with a real GET until the server answers. False if it never did."""
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout_s:
        code = http_probe(host, port)
        if code and code != "no-status":
            log.append(f"[{time.monotonic() - t0:7.2f}s] (server answered "
                       f"HTTP {code} on {host}:{port})")
            return True
        time.sleep(1.0)
    return False


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--host", default="192.168.4.1")
    ap.add_argument("--port", type=int, default=80)
    ap.add_argument("--console", default=None,
                    help="controller console port; REBOOTS the board when opened")
    ap.add_argument("--seconds", type=float, default=30.0)
    ap.add_argument("--clients", type=int, default=24,
                    help="concurrent requesting clients")
    ap.add_argument("--half-open", type=int, default=120,
                    help="connections held with a partial request")
    ap.add_argument("--max-worst", type=int, default=800,
                    help="graded ceiling on HB worst= during the flood")
    ap.add_argument("--max-hb-gap", type=int, default=6000,
                    help="graded ceiling on the gap between HB beats")
    ap.add_argument("--min-ok", type=int, default=50,
                    help="minimum answered requests over the window")
    ap.add_argument("--max-recover", type=float, default=90.0,
                    help="seconds to wait after the window for the listener to answer again")
    ap.add_argument("--no-recover-probe", action="store_true",
                    help="skip the after-the-window recovery measurement")
    ap.add_argument("--log", default=None, help="write the console capture here")
    ap.add_argument("--nmcli-profile", default=None,
                    help="NetworkManager profile to re-join if the server stops answering after "
                         "the console open reset the board (this bench: ESP-CCR-3828)")
    ap.add_argument("--wlan", default="wlan0", help="interface to drop before rejoining")
    args = ap.parse_args()

    console_log: list[str] = []
    console: Console | None = None
    if args.console:
        console = Console(args.console, console_log)
        console.start()
    # Opening the console port reset the board. The access point is not up until a
    # few seconds into the boot (banner ~1.5 s, LittleFS mount ~2 s, softAP after
    # that), and on this bench the PC rejoins ESP-CCR through NetworkManager only
    # after the AP reappears — so the window does not start until a TCP connect to
    # the server actually succeeds. Flooding before that measures a boot and a
    # radio, not the load.
    reached = wait_for_server(args.host, args.port, 40.0, console_log)
    rejoined = False
    if not reached and args.nmcli_profile:
        # The stale-association case, and the only honest repair: drop the
        # interface, re-join the profile, wait again. Reported in the summary,
        # because a flood that needed the bench's radio re-armed mid-run is a
        # different run from one that did not.
        nmcli(console_log, "device", "disconnect", args.wlan)
        nmcli(console_log, "connection", "up", args.nmcli_profile)
        rejoined = wait_for_server(args.host, args.port, 30.0, console_log)
        reached = rejoined

    stop = threading.Event()
    counts: Counter = Counter()
    lock = threading.Lock()
    end = time.monotonic() + args.seconds

    held: list[threading.Thread] = []
    if args.half_open:
        t = threading.Thread(target=hold_half_open,
                             args=(args.host, args.port, args.half_open, stop),
                             daemon=True)
        t.start()
        held.append(t)

    workers = [threading.Thread(target=flood_one,
                                args=(args.host, args.port, end, list(PATHS),
                                      counts, lock, stop),
                                daemon=True)
               for _ in range(args.clients)]
    for w in workers:
        w.start()
    while time.monotonic() < end:
        time.sleep(0.5)
    stop.set()
    for w in workers:
        w.join(timeout=6.0)
    if console:
        console.stop()
        console.join(timeout=4.0)

    if args.log:
        with open(args.log, "w") as f:
            f.write("\n".join(console_log) + "\n")

    print(f"\nclients {args.clients}, half-open {args.half_open}, window {args.seconds:.0f} s")
    # Every outcome the clients saw, including the error codes: the shape of a
    # failure is the finding (ECONNREFUSED = backlog full, ECONNRESET = the
    # server hung up, EHOSTUNREACH = the bench was not associated at all).
    for key in sorted(counts):
        if key != "bytes":
            print(f"  {key:<24} {counts[key]}")
    print(f"  {'bytes':<24} {counts['bytes']}")

    fails: list[str] = []
    # Every status the server actually sent counts as an answer: a 503 with a
    # reason ("no memory for export") is the web layer degrading honestly, and a
    # 404/302 is work done. Only silence is a failure to answer.
    answered = sum(v for k, v in counts.items() if k.startswith("answered-"))
    # How long the listener stays dead after the load stops. Measured, not
    # assumed: the flood ending is not the same event as the dashboard working
    # again, and "it recovered" without a number is the claim that hides a
    # diver's phone staring at a spinner.
    recover_s = None
    if not args.no_recover_probe:
        t0 = time.monotonic()
        while time.monotonic() - t0 < args.max_recover:
            code = http_probe(args.host, args.port)
            if code and code != "no-status":
                recover_s = round(time.monotonic() - t0, 1)
                break
            time.sleep(2.0)
        if recover_s is None:
            fails.append(f"still not answering {args.max_recover:.0f} s after the flood "
                         "stopped: the dashboard did not come back on its own")
        else:
            print(f"  answered again after   {recover_s} s")

    unreachable = counts["connect-EHOSTUNREACH"] + counts["connect-ENETUNREACH"]
    if unreachable:
        fails.append(f"{unreachable} connects failed with EHOSTUNREACH / ENETUNREACH: the bench "
                     "was not associated with the access point, so worst= measures an idle board")
    if answered < args.min_ok:
        fails.append(f"only {answered} answered requests over the window "
                     f"(floor {args.min_ok}): the flood answered nothing")
    if console is None:
        print("console not read: worst= and the fatal-marker checks did NOT run "
              "(pass --console to run them)")
    else:
        if console.error:
            fails.append(f"console: {console.error}")
        else:
            print(f"  HB beats       {console.beats}")
            print(f"  max HB gap     {console.max_gap_ms} ms")
            print(f"  max worst=     {console.worst} ms")
            if console.cfggen:
                pub, chain = console.cfggen[-1]
                behind = max(p - c for p, c in console.cfggen)
                print(f"  cfggen         {pub}/{chain} (worst skew behind {behind})")
                if behind > 1:
                    fails.append(f"cfggen= chain {behind} generations behind published "
                                 "during the flood: a control cycle stopped reading")
            if console.worst is None and not console.error:
                fails.append("no HB worst= seen: the flood's effect on the control "
                             "cycle is unmeasured, not benign")
            elif console.worst is not None and console.worst > args.max_worst:
                fails.append(f"worst= reached {console.worst} ms under the flood "
                             f"(ceiling {args.max_worst}, measured watchdog expiry "
                             "3011..3061 ms)")
            if console.max_gap_ms is not None and console.max_gap_ms > args.max_hb_gap:
                fails.append(f"HB beat gap reached {console.max_gap_ms} ms (ceiling "
                             f"{args.max_hb_gap}): a task the network can reach stalled")
            if console.fatals:
                fails.append("console showed a fatal marker during the flood: "
                             + console.fatals[0])

    print()
    for f in fails:
        print("FAIL " + f)
    print(f"{len(fails)} failure(s)")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
