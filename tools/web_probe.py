#!/usr/bin/env python3
"""Verify the surface web stack end to end: page, captive DNS, and the WebSocket.

Why this exists: the handset link is the in-water transport, but the web AP is
how a diver sees the dive plan, the settings page and the CSV exports at the
surface, and both are supposed to run the SAME command path with the SAME
refusals. Until 2026-11-23 nothing on the bench proved that: radio_stress only
proves the AP comes back up, and BENCH.md said "join it from a phone and look" —
which is not a check anyone can run twice and compare.

What it verifies, and what each part is for:

  HTTP     / serves, and the bytes are IDENTICAL to data/index.html (the check
           that found the stale-LittleFS defect, see BENCH.md C2-f); the page
           references nothing external (the AP has no internet, so any CDN link
           is a broken page under water); /favicon.ico is 204; an unknown path
           302s to /, which is what makes the captive portal work; /app.js 404s,
           because the page is one self-contained file.
  DNS      any name resolves to the AP: the other captive-portal half.
  WebSocket handshake verified against RFC 6455 (Sec-WebSocket-Accept computed
           here, no library: a dependency would be verifying itself); the
           `#boot <uptimeMs> <freeHeap>` hello; then the `#<lines>` marker
           contract for every command - the browser cannot know where an answer
           ends unless the marker count equals the answer's line count - and for
           the over-long case that the marker says `dropped`.
  EQUALITY each command sent twice, BACK TO BACK, once over the WebSocket and
           once on UART2. Twelve must be byte-identical (one command path, two
           transports: the project's central claim); `GET DS` is compared with its
           `age_s` field excepted (seconds since the sensor's last good read, and
           the two sends are ~0.4 s apart); the over-long case is compared on its
           refusal only, because UART2 additionally dispatches the leftover tail as
           its own line - docs/HANDSET_PROTOCOL.md says why. (one command path, two
           transports: the project's central claim); the over-long case is
           compared on its refusal only, because UART2 additionally dispatches
           the leftover tail as its own line - docs/HANDSET_PROTOCOL.md says why.
  DIVE     --dive-cycle: DIVE START from the browser must take the radio out
           from under an associated client and kill its socket, the dive must
           then close over UART2 (the web cannot close the dive that switched
           itself off), and the AP must come back with a fresh WebSocket
           working. Needs --handset for exactly that reason.

Needs a station joined to the controller's AP first: SSID `ESP-CCR-XXXX`, where
XXXX is the MAC tail a scan shows; WPA2; password = the AP_PASS setting. HOW the
machine joins is the operator's setup and this project does not prescribe it -
the tool never connects, disconnects or reconfigures anything. It talks HTTP and
WebSocket to --host, and only ever SCANs the air (read-only) to confirm the AP is
broadcasting. If the station is not joined, the checks that need it report
themselves as not evaluated rather than failing the firmware for it.

    tools/web_probe.py [--host 192.168.4.1] [--handset /dev/ttyUSB1]
                       [--dive-cycle] [--no-dns]

Exit 0 = every claim held. 1 = one did not. --handset is optional; without it the
cross-transport equality and dive-cycle phases are skipped and the rest still
runs. Verdicts are three-state: PASS, FAIL, and SKIP for anything this machine
could not observe.
"""
import argparse
import base64
import hashlib
import os
import re
import socket
import struct
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from port_id import open_quiet          # noqa: E402  (console reader, no reset)

WS_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"     # RFC 6455

# The answer marker is the ONLY '#' line that means "answer complete". The
# WebSocket also sends `#boot <uptimeMs> <freeHeap>` when a client connects.
MARKER_RE = re.compile(r"#\d+( dropped)?")

# (command, why, mode). Ordered so that NOTHING before the last two changes
# state: the same command is run on both transports back to back, and if a write
# sat in the middle the second transport would correctly answer differently and
# the check would report that as a disagreement. Refusals and reads first; the
# one write and its read-back last. GET HIST is deliberately absent: the
# histogram is derived from a live ring, so the same command 10 s apart
# legitimately answers differently on the same transport, which proves nothing
# about the other one.
#
# mode "identical"   - the non-stream reply lines must be byte-identical.
# "fresh-window"     - extra flag: wait out the UART task's 2 s overflow-error
#   rate limit first, or that answer is legitimately missing (silence).
MATRIX = [
    ("AUTO", "cold-boot lockout", "identical"),
    ("SET SP2=2.5", "range refusal", "identical"),
    ("FROBNICATE", "unknown command does nothing", "identical"),
    ("GET SCHEMA", "ranges the page will enforce", "identical"),
    ("GET *", "every setting", "identical"),
    ("GET FAULT", "fuse/duty picture", "identical"),
    # Which chip owns depth. Compared with one field excepted, and the exception
    # is stated rather than waved at: `age_s` is seconds since that chip's last
    # good read, and the two sends are ~0.4 s apart, so THAT field may legitimately
    # differ. The name, the address and the three flags must be the same bytes on
    # both transports — there is one command path, and a web bridge that answered
    # depth provenance out of its own copy is exactly what this catches.
    ("GET DS", "which chip owns depth, and whether it is being used",
     "identical-except-age"),
    # Calibration honesty. `CAL O2` and `CAL DEPTH` are in the grammar but have no
    # consumer on the board, so they must be REFUSED — and refused the same way on
    # both transports, or a handset has a calibration the phone does not (and vice
    # versa). `GET CALAIR` is the verdict of the one calibration that does run:
    # code 0 (nothing asked since this power-up) is deterministic here because this
    # probe never asks for one, so the two transports have to agree byte for byte.
    ("CAL DEPTH", "depth tare refused: nothing consumes it", "identical"),
    ("CAL O2", "O2 calibration refused: no sequencer on the board", "identical"),
    ("CAL O2 CONFIRM", "no phantom two-step arm to confirm", "identical"),
    ("GET CALAIR", "the air calibration's real verdict", "identical"),
    # Calibration honesty. `CAL O2` and `CAL DEPTH` are in the grammar but have no
    # consumer on the board, so they must be REFUSED — and refused the same way on
    # both transports, or a handset has a calibration the phone does not (and vice
    # versa). `GET CALAIR` is the verdict of the one calibration that does run:
    # code 0 (nothing asked since this power-up) is deterministic here because this
    # probe never asks for one, so the two transports have to agree byte for byte.
    ("CAL DEPTH", "depth tare refused: nothing consumes it", "identical"),
    ("CAL O2", "O2 calibration refused: no sequencer on the board", "identical"),
    ("CAL O2 CONFIRM", "no phantom two-step arm to confirm", "identical"),
    ("GET CALAIR", "the air calibration's real verdict", "identical"),
    ("SET SP2=0.70", "surface write lands", "identical"),
    ("GET SP2", "and reads back", "identical"),
    # The over-long case now agrees on both transports byte for byte, which is
    # itself the result: UART2 used to answer `ERR line too long` AT THE MOMENT
    # the line passed MAX_LINE, reset its buffer and let the REST of that
    # physical line become a second command — and inside its own 2 s rate-limit
    # window it did not reset the buffer at all, so the truncated prefix ran
    # (an over-long `SET SP2=0.33` wrote 0.330, PLAN 16). proto::LineReader drops
    # the whole line, and `fresh-window` starts the check in an unmuted error
    # window so it is about the protocol, not about whatever the last tool in the
    # pipeline sent. The web path always discarded the whole frame.
    ("GET " + "Z" * 200, "overlong line dropped whole", "identical", "fresh-window"),
]


def mark(ok):
    return "PASS" if ok is True else ("SKIP" if ok is None else "FAIL")


EXTERNAL = re.compile(r"""https?://|//cdn|integrity\s*=""")


def grade_surface_body(body):
    """The exported surface log, graded against itself.

    Three things have to hold, and each is a lie the export could tell:
      * the identity row's count equals the rows delivered (a truncated file
        presented as complete);
      * the sequence range actually spans the rows (a header describing a
        different file than the rows);
      * every row's id matches the name in its own row, and an id this build does
        not name renders as `id<n>` rather than being dropped (an appended id
        silently disappearing is how an append-only log stops being one).
    """
    head, rows = None, []
    for line in body.decode(errors="replace").splitlines():
        f = line.split(",")
        if f[0] == "SURFACE" and len(f) > 4:
            head = f
        elif f[0] == "R":
            rows.append(f)
    checks = []
    checks.append(("surface.csv has an identity row before any record", head is not None,
                   body.decode(errors="replace").splitlines()[:1] and
                   body.decode(errors="replace").splitlines()[0] or "empty body"))
    if head is None:
        return checks
    n, lo, hi = int(head[2]), int(head[3]), int(head[4])
    checks.append(("surface.csv identity count matches the rows delivered", n == len(rows),
                   f"SURFACE says {n}, {len(rows)} rows arrived"))
    checks.append(("surface.csv rows all carry the 13 documented columns",
                   all(len(r) == 13 for r in rows),
                   next((str(r) for r in rows if len(r) != 13), "all rows are 13 columns")))
    if rows:
        checks.append(("surface.csv seq range spans the rows",
                       lo <= int(rows[0][1]) and hi >= int(rows[-1][1]),
                       f"header {lo}..{hi}, rows {rows[0][1]}..{rows[-1][1]}"))
        # id and name must agree, in both directions.
        KNOWN = {"7": "cal_air", "13": "pre_dive_pass", "16": "pre_dive_fail"}
        mismatch = [r for r in rows
                    if KNOWN.get(r[4]) is not None and r[5] != KNOWN[r[4]]]
        checks.append(("surface.csv id and name agree", not mismatch,
                       str(mismatch[0]) if mismatch else "every known id carries its name"))
        unnamed = [r for r in rows if r[4] not in KNOWN and r[5] != f"id{r[4]}"]
        checks.append(("surface.csv names unknown ids instead of hiding them",
                       not unnamed, str(unnamed[0]) if unnamed else "none needed"))
        undated = [r for r in rows if r[2] == "0" and r[3] != "clock-unset"]
        checks.append(("surface.csv says so when a record has no date", not undated,
                       str(undated[0]) if undated else "records without a clock read clock-unset"))
    return checks


def http_probe(host, out):
    """Status line for each URL that matters. Returns the list of (url, ok, detail)."""
    import urllib.error
    import urllib.request

    checks = []
    got = {}
    for path, want in (("/", "200"), ("/favicon.ico", "204"),
                       ("/no-such-path", "302 -> /"), ("/dive/99.csv", "302 -> /"),
                       ("/surface.csv", "200 + identity row"),
                       ("/app.js", "404 (no such asset any more)")):
        _ = want
        req = urllib.request.Request(f"http://{host}{path}")
        class NoRedirect(urllib.request.HTTPRedirectHandler):
            def redirect_request(self, *a, **k):
                return None
        opener = urllib.request.build_opener(NoRedirect)
        try:
            r = opener.open(req, timeout=8)
            code, body = r.getcode(), r.read()
            loc = r.headers.get("Location", "")
        except urllib.error.HTTPError as e:
            code, body, loc = e.code, e.read(), e.headers.get("Location", "")
        except Exception as e:                        # noqa: BLE001
            checks.append((path, False, f"{type(e).__name__}: {e}"))
            continue
        got[path] = body
        if path == "/":
            # What this is really asserting: a real HTML document, not an error
            # page, not an empty body. The window used to be 200 bytes, which a
            # harmless comment at the top of the page broke — so it asserted the
            # page's first two hundred bytes, not that the page is a page.
            head = body[:4096].lower()
            ok = code == 200 and body.lstrip().lower().startswith(b"<!doctype html") \
                and b"<html" in head
            detail = f"{code}, {len(body)} bytes, html"
        elif path == "/surface.csv":
            # The surface log's ONLY export path: calibration and pre-dive records,
            # which no dive file can hold and no handset command can read. The
            # first line must be the identity row — a file that starts with a data
            # row has no count and no sequence range, and a reader cannot tell a
            # shed history from a complete one.
            first = body.split(b"\n", 1)[0].decode(errors="replace")
            ok = code == 200 and first.startswith("SURFACE,")
            detail = f"{code}, {len(body)} bytes, first line {first!r}"
        elif path == "/app.js":
            ok = code == 404
            detail = f"{code} - correct: the page is one self-contained file"
        elif path == "/favicon.ico":
            ok = code == 204
            detail = f"{code} (deliberate: no icon, no 404 noise)"
        else:
            ok = code == 302 and loc == "/"
            detail = f"{code} Location: {loc or '(none)'}"
        checks.append((path, ok, detail))
        out.append(f"  GET {path:<14} {detail}")

    # ---- the surface log's own honesty --------------------------------------
    surf = got.get("/surface.csv")
    if surf is None:
        checks.append(("surface.csv body was fetched", False, "no response captured"))
    else:
        checks.extend(grade_surface_body(surf))

    served = got.get("/", b"")
    page = served.decode(errors="replace")

    # Byte-exact against the repo. `want` for this path is the generated
    # data/index.html, which sync_webdata.py derives from web_page.h: three
    # artifacts that must agree, and a stale LittleFS is the failure mode.
    gen = Path(__file__).resolve().parent.parent / "data" / "index.html"
    if gen.exists():
        same = served == gen.read_bytes()
        def dig(b):
            return hashlib.md5(b).hexdigest()[:10]
        checks.append(("served page is byte-identical to data/index.html", same,
                       f"served {len(served)} B md5 {dig(served)} vs repo "
                       f"{gen.stat().st_size} B md5 {dig(gen.read_bytes())}"
                       if not same else f"{len(served)} bytes, matches {gen}"))
    else:
        checks.append(("data/index.html exists to compare against", False,
                       "run tools/sync_webdata.py first"))

    # A self-contained page must ask for nothing it cannot serve. Every href/src
    # in it has to be a fragment, the favicon, or a route this server has.
    refs = sorted(set(re.findall(r'(?:href|src)="([^"]+)"', page)))
    dead = [r for r in refs if not r.startswith(("#", "/", "mailto:"))]
    checks.append(("page is self-contained (no cross-site references)", not dead,
                   f"references: {refs}"))
    ext = [l.strip()[:90] for l in page.splitlines() if EXTERNAL.search(l)]
    checks.append(("page references nothing external", not ext,
                   ext[0] if ext else "the AP has no uplink: a CDN link is a dead page"))
    # The page must POST the clock, not merely display one: the controller has
    # no RTC and the AP has no uplink, so the dive log only carries dates if the
    # surface client supplies wall time. A page that shows the clock but never
    # sets it leaves dive dates to whether someone remembers to type a command.
    posts = re.search(r'send\("TIME SET\s*"', page)
    checks.append(("page posts the phone clock on open (TIME SET)", bool(posts),
                   posts.group(0) if posts else
                   "no TIME SET in the served page: dives will carry no date "
                   "unless someone sets the clock by hand"))
    reads = re.search(r'"GET TIME"', page)
    checks.append(("page shows the controller's clock state", bool(reads),
                   reads.group(0) if reads else "no GET TIME: the page cannot "
                                                "say whether the clock landed"))
    # A button is a promise. The page may not offer a command the controller
    # refuses: `CAL O2`/`CAL DEPTH` are refused as unimplemented, so a button for
    # one would be a control that does nothing on a machine that dives. (And the
    # refusal is what the page must be able to show instead.)
    ghost = [c for c in ('data-c="CAL DEPTH"', 'data-c="CAL O2"', 'data-c="CAL O2 CONFIRM"')
             if c in page]
    checks.append(("page offers no button for a command the controller refuses", not ghost,
                   f"still offered: {ghost} — the firmware answers `ERR not implemented`"))
    # ...and it must be able to ask for the verdict of the calibration that DOES
    # run: `CAL AIR`'s reply is only a receipt, so the page has to read CALAIR.
    calair = re.search(r'"GET CALAIR"', page)
    checks.append(("page can read the air calibration's verdict", bool(calair),
                   calair.group(0) if calair else
                   "no GET CALAIR: the page can start the calibration and then says "
                   "nothing about how it ended"))
    renders = re.search(r'CALAIR\[1\]', page)
    checks.append(("page reads the verdict code and has wording for it", bool(renders),
                   renders.group(0) if renders else
                   "the CALAIR status field is never read: the page would show a "
                   "number, or nothing, where a diver needs words"))

    # The surface log is web-only by decision, so the page is the only way to read
    # it: a controller that records verdicts nobody can retrieve has not recorded
    # them. And the page must say what a verdict MEANS, not print a code.
    surf_ref = re.search(r'"/?surface\.csv"', page) or re.search(r'surface\.csv', page)
    checks.append(("page can read the surface log", bool(surf_ref),
                   surf_ref.group(0) if surf_ref else
                   "no /surface.csv in the page: the calibration and pre-dive records "
                   "would be unreadable by anyone but a serial console"))
    # Both patterns are taken from the surface renderer's own wording, chosen so
    # they cannot be satisfied by the rest of the page: "pre-dive checks" alone is
    # in the CHECK button's label, and "no calibration since power-up" is on the
    # checklist view already, so either one on its own would pass on a page with no
    # surface log at all (checked against the previous page, where they did).
    words = [w for w in ("shed by capacity", "pre_dive_fail") if re.search(w, page)]
    checks.append(("page renders surface records as words, not bare codes",
                   len(words) == 2,
                   ", ".join(words) if len(words) == 2 else
                   f"missing wording for a capacity-shed log or a failed step: {words or 'none'}"))

    linked = sorted(set(re.findall(r'(?:href|src)="([^"]+)"', page)))
    out.append(f"  page links: {linked}")
    return checks, page, got


def dns_probe(ap_ip):
    """The captive portal only works if any name at all answers with the AP."""
    try:
        r = subprocess.run(["dig", "+noedns", "+short", f"@{ap_ip}",
                            "captive-portal-probe.example", "A"],
                           capture_output=True, text=True, timeout=10)
        answers = r.stdout.split()
    except Exception as e:                            # noqa: BLE001
        return [("captive DNS answers", False, f"{type(e).__name__}: {e}")]
    ok = answers == [ap_ip]
    return [(f"any name resolves to {ap_ip}", ok, f"answers: {answers or 'none'}")]


# --------------------------------------------------------------- websocket ----
class Ws:
    """The smallest client that is honest about the handshake.

    Written rather than installed: the bench has no websocket library and the
    point of this tool is to verify the server's contract, so a dependency would
    also be verifying itself.
    """

    def __init__(self, host, port=80, path="/ws", timeout=8.0):
        self.s = socket.create_connection((host, port), timeout=timeout)
        self.s.settimeout(timeout)
        self.buf = b""
        key = base64.b64encode(os.urandom(16)).decode()
        self.s.sendall(
            f"GET {path} HTTP/1.1\r\nHost: {host}\r\nUpgrade: websocket\r\n"
            f"Connection: Upgrade\r\nSec-WebSocket-Key: {key}\r\n"
            "Sec-WebSocket-Version: 13\r\n\r\n".encode())
        head = b""
        while b"\r\n\r\n" not in head:
            chunk = self.s.recv(4096)
            if not chunk:
                raise RuntimeError("handshake: connection closed")
            head += chunk
        status, _, rest = head.partition(b"\r\n\r\n")
        self.buf = rest
        lines = status.decode(errors="replace").split("\r\n")
        self.upgrade = lines[0]
        self.accept = ""
        for l in lines[1:]:
            if l.lower().startswith("sec-websocket-accept:"):
                self.accept = l.split(":", 1)[1].strip()
        self.want_accept = base64.b64encode(
            hashlib.sha1((key + WS_GUID).encode()).digest()).decode()

    def handshake_ok(self):
        return "101" in self.upgrade and self.accept == self.want_accept

    def send(self, text):
        payload = text.encode()
        mask = os.urandom(4)
        n = len(payload)
        if n < 126:
            hdr = struct.pack("!BB", 0x81, 0x80 | n)
        elif n < 65536:
            hdr = struct.pack("!BBH", 0x81, 0x80 | 126, n)
        else:
            hdr = struct.pack("!BBQ", 0x81, 0x80 | 127, n)
        self.s.sendall(hdr + mask + bytes(b ^ mask[i % 4] for i, b in enumerate(payload)))

    def _need(self, n):
        while len(self.buf) < n:
            chunk = self.s.recv(4096)
            if not chunk:
                raise ConnectionError("socket closed")
            self.buf += chunk

    def frame(self):
        self._need(2)
        b0, b1 = self.buf[0], self.buf[1]
        ln = b1 & 0x7F
        off = 2
        if ln == 126:
            self._need(4); ln = struct.unpack("!H", self.buf[2:4])[0]; off = 4
        elif ln == 127:
            self._need(10); ln = struct.unpack("!Q", self.buf[2:10])[0]; off = 10
        self._need(off + ln)
        payload, self.buf = self.buf[off:off + ln], self.buf[off + ln:]
        return b0 & 0x0F, payload

    def lines_until_marker(self, want_lines=None, limit=6.0):
        """Read protocol lines until the `#<n>` answer marker.

        Returns (answer_lines, marker_line, unsolicited). The controller streams
        T,/D,/E, lines unprompted on the same socket, so they are separated from
        the answer rather than counted into it.
        """
        end = time.time() + limit
        answer, marker, unsolicited, control = [], None, [], []
        while time.time() < end and marker is None:
            try:
                op, payload = self.frame()
            except (ConnectionError, socket.timeout):
                break
            if op == 9:                        # ping: answer it, keep reading
                self.s.sendall(b"\x8a\x00")
                continue
            if op == 8:
                break
            for l in payload.decode(errors="replace").split("\n"):
                l = l.strip("\r")
                if not l:
                    continue
                if MARKER_RE.fullmatch(l):
                    marker = l
                elif l.startswith("#"):
                    control.append(l)      # e.g. the "#boot <uptimeMs> <freeHeap>" hello
                elif re.match(r"^[TDE],", l):
                    unsolicited.append(l)
                else:
                    answer.append(l)
        return answer, marker, unsolicited, control

    def close(self):
        try:
            self.s.close()
        except Exception:                       # noqa: BLE001
            pass


def ws_probe(host):
    checks = []
    try:
        ws = Ws(host)
    except Exception as e:                                    # noqa: BLE001
        return [("websocket connect", False, f"{type(e).__name__}: {e}")]

    accept_ok = ws.accept == ws.want_accept
    checks.append(("handshake is RFC 6455 correct", ws.handshake_ok(),
                   f"{ws.upgrade.splitlines()[0]}, Sec-WebSocket-Accept "
                   f"{'matches' if accept_ok else 'WRONG'}"))

    _a, _m, _u, hello = ws.lines_until_marker(limit=3.0)
    boots = [l for l in hello if l.startswith("#boot")]
    checks.append(("server sends the #boot hello on connect", bool(boots),
                   boots[0] if boots else f"control lines: {hello}"))

    for cmd, why, mode, *opt in MATRIX:
        ws.send(cmd)
        answer, marker, unsol, _ctl = ws.lines_until_marker()
        # The marker is how the browser knows the answer is finished. A count
        # that disagrees with the answer means the page renders half a table.
        m = re.match(r"^#(\d+)( dropped)?$", marker or "")
        ok = bool(m) and answer and int(m.group(1)) == len(answer)
        checks.append((f"{cmd!r} answers with a matching line marker ({why})", ok,
                       f"{len(answer)} answer lines, marker {marker!r}, "
                       f"{len(unsol)} streamed lines ignored"))
        if "fresh-window" in opt:
            # `dropped` is how the server says "the command you sent arrived
            # truncated" (web_server.cpp:144). Without it the page cannot tell a
            # truncated command from a command that was understood and answered
            # with one line, and shows nothing useful.
            checks.append(("the overlong answer is marked `dropped`",
                           bool(m) and m.group(2) is not None,
                           f"marker {marker!r}, answer {answer}"))
    ws.close()
    return checks


def uart_reply_lines(ser, cmd, wait=3.0):
    """One command on an open UART2 handle; the non-stream lines it answers with."""
    ser.write((cmd + "\r\n").encode())
    ser.flush()
    got, end = [], time.time() + wait
    while time.time() < end:
        raw = ser.readline().decode(errors="replace").strip()
        if not raw:
            break
        if not re.match(r"^[TDE],", raw):
            got.append(raw)
        elif got:
            break                      # answer finished, the stream resumed
    return got


def compare_transports(host, port, cmds, checks, settle=1.5):
    """Each command twice, back to back: once over the WebSocket, once on UART2.

    This is the project's central claim - one command path, two transports - and
    it is only worth proving if both reads see the same machine, which is why
    they interleave rather than run as two separate passes.
    """
    ws = Ws(host)
    if not ws.handshake_ok():
        checks.append(("websocket handshake before the comparison", False, ws.upgrade))
        ws.close()
        return
    ser = open_quiet(port)
    time.sleep(settle)
    ser.reset_input_buffer()
    # reset_input_buffer() only clears OUR side. Un-terminated bytes held in the
    # controller's line buffer are parsed together with the next command: `XY`
    # then `AUTO\r\n` is answered `ERR unknown command` because it sees `XYAUTO`
    # (verified on the bench 2026-11-23). Opening this port resets the bridge
    # ESP32 on this bench, and its boot banner lands in the controller's RX, so
    # flush the controller's buffer - and report what the flush drew, because a
    # bare newline answered `ERR unknown command` is proof bytes were pending.
    ser.write(b"\r\n")
    ser.flush()
    drew, end = [], time.time() + 0.6
    while time.time() < end:
        l = ser.readline().decode(errors="replace").strip()
        if l and not re.match(r"^[TDE],", l):
            drew.append(l)
    if drew:
        print(f"  note: stray bytes were pending in the controller's line "
              f"buffer - a bare newline drew {drew}")
    try:
        for cmd, why, mode, *opt in cmds:
            if "fresh-window" in opt:
                # UART2's `ERR line too long` is rate limited to one per 2 s, so
                # a stale window would answer this command with silence and the
                # comparison would be about timing, not framing.
                time.sleep(2.1)
            ws.send(cmd)
            web, marker, _u, _c = ws.lines_until_marker()
            uart = uart_reply_lines(ser, cmd)
            if mode == "identical":
                same = web == uart and bool(web)
                detail = f"web {web[:2]} | uart {uart[:2]} | marker {marker}"
            elif mode == "identical-except-age":
                def scrub(lines):
                    out = []
                    for l in lines:
                        f = l.split(",")
                        # isdigit() here is a check, not a convenience: an answer
                        # whose age field is not a number is not a DS, answer.
                        if len(f) >= 6 and f[0] == "DS" and f[4].isdigit():
                            f[4] = "<age>"
                        out.append(",".join(f))
                    return out
                got = [l for l in web if l.startswith("DS,")]
                same = scrub(web) == scrub(uart) and bool(got)
                detail = (f"web {web[:2]} | uart {uart[:2]} | marker {marker} — "
                          f"age field excepted (the sends are ~0.4 s apart)")
            else:
                same = False
                detail = f"unknown comparison mode {mode!r} in the check table"
            checks.append((f"{cmd!r} agrees on both transports ({why})", same, detail))
            print(f"  [{'PASS' if same else 'FAIL'}] {cmd[:40]!r}")
            if not same:
                print(f"      web : {web}")
                print(f"      uart: {uart}")
            time.sleep(0.35)           # let the 2 Hz stream settle between commands
    finally:
        ws.close()
        ser.close()


def dive_row_equality(host, port, checks):
    """`LOG EXPORT n` on the handset and `GET /dive/<n>.csv` on the web are the
    same bytes.

    This is BENCH.md C2 step 4, which asserted it for a long time without ever
    comparing anything: two writers produced two different row shapes (14 vs 17
    fields, alarms hex vs decimal, ceiling m vs cm, battery mV vs V) and the
    check passed anyway. They share one formatter in the firmware now
    (`logfmt::snapshotCsvLine` and friends); this proves it on the wire, where
    truncation by either transport would show up.

    `E,` rows are counted but not compared: an exported event row is
    indistinguishable on the wire from a live event the controller pushes while
    the export is in flight (same `E,<t>,<id>,<value>` shape). That ambiguity is
    in the protocol document; a client that needs the exported events reads the
    file, not the stream.
    """
    import urllib.request

    ser = open_quiet(port)
    time.sleep(1.2)
    ser.reset_input_buffer()
    ser.write(b"\r\n")            # flush the controller's line buffer, not ours
    ser.flush()
    time.sleep(0.3)
    ser.reset_input_buffer()

    ser.write(b"LOG LIST\r\n")
    ser.flush()
    slots, end = [], time.time() + 2.5
    while time.time() < end:
        l = ser.readline().decode(errors="replace").strip()
        if l.startswith("L,"):
            parts = l.split(",")
            if len(parts) > 1 and parts[1].isdigit():
                slots.append(int(parts[1]))
    if not slots:
        ser.close()
        checks.append(("dive rows identical on both transports", None,
                       "no dive on the board to export (open and close one, or "
                       "run --dive-cycle)"))
        return

    slot = max(slots)
    try:
        with urllib.request.urlopen(f"http://{host}/dive/{slot}.csv", timeout=10) as r:
            http_rows = [l.strip() for l in r.read().decode(errors="replace").split("\n")
                         if l.strip()]
    except Exception as e:                                        # noqa: BLE001
        ser.close()
        checks.append(("dive rows identical on both transports", False,
                       f"GET /dive/{slot}.csv: {type(e).__name__}: {e}"))
        return

    ser.write(f"LOG EXPORT {slot}\r\n".encode())
    ser.flush()
    uart_rows, events, end = [], 0, time.time() + 4.0
    while time.time() < end:
        l = ser.readline().decode(errors="replace").strip()
        if l.startswith(("DIVE,", "S,", "M,")):
            uart_rows.append(l)
        elif l.startswith("E,"):
            events += 1
    comparable_http = [l for l in http_rows if not l.startswith("E,")]
    same = comparable_http == uart_rows and len(uart_rows) > 1
    checks.append(("dive rows identical on both transports", same,
                   f"slot {slot}: {len(uart_rows)} DIVE/S/M rows on UART2 == "
                   f"{len(comparable_http)} over HTTP; {events} E, rows on the "
                   f"wire not compared (live events share the shape)"))
    if not same:
        for a, b in zip(comparable_http, uart_rows):
            if a != b:
                print(f"      first difference:\n        http: {a}\n        uart: {b}")
                break

    # The identity row's v3 append: a human-readable UTC start, rendered by the
    # firmware from the stored epoch (one truth on the flash). `clock-unset` is
    # the honest answer when the dive started with no clock — a wrong date is
    # worse than no date, but this gate exists to make sure "no date" is loud
    # and rare, not the default.
    head_row = next((l for l in http_rows if l.startswith("DIVE,")), "")
    m = re.match(r"^DIVE,(\d+),(\d+),(\d+),(\d+),"
                 r"(\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}Z|clock-unset)$", head_row)
    checks.append(("dive CSV identity row is the v3 shape with a date field",
                   bool(m) and m.group(5) != "clock-unset",
                   head_row or "no DIVE, row in the export"))
    ser.close()


def sh(cmd):
    try:
        return subprocess.run(cmd, shell=True, capture_output=True, text=True,
                              timeout=25).stdout.strip()
    except Exception as e:                                        # noqa: BLE001
        return f"{type(e).__name__}"


def http_code(host, timeout=4.0):
    import urllib.error
    import urllib.request
    try:
        with urllib.request.urlopen(f"http://{host}/", timeout=timeout) as r:
            return r.getcode()
    except urllib.error.HTTPError as e:
        return e.code
    except Exception:                                             # noqa: BLE001
        return None


def ap_broadcasting():
    """True/False if a SCAN can see the AP, None if no scan is available.

    Scanning is read-only. Connecting is not: the first version of this tool
    tried to bring a network connection up on the machine it runs on when the
    station did not rejoin after a dive. That is the operator's decision, and it
    assumed a connection profile this project knows nothing about - which broke
    the moment the operator's own setup replaced the one the tool had made up.
    """
    if sh("command -v nmcli") == "":
        return None
    out = sh("timeout 20 nmcli -f SSID dev wifi list --rescan yes 2>/dev/null")
    return "ESP-CCR" in out


def wifi_linked():
    """Station-side truth. Empty string when `iw` is unavailable: not connected
    and not observable are different things, and a check must not confuse them."""
    if sh("command -v iw") == "":
        return None
    return "Connected to" in sh("iw dev wlan0 link")


def uart_answer(port, cmd, want, tries=2):
    """Send on UART2 and return the matching reply, or None."""
    ser = open_quiet(port)
    try:
        time.sleep(1.0)
        ser.reset_input_buffer()
        ser.write(b"\r\n")            # flush any residue as its own line
        ser.flush()
        time.sleep(0.3)
        ser.reset_input_buffer()
        for _ in range(tries):
            sent = time.time()
            ser.write((cmd + "\r\n").encode())
            ser.flush()
            while time.time() - sent < 3.0:
                l = ser.readline().decode(errors="replace").strip()
                if not l:
                    continue
                if want is None or l.startswith(want):
                    return l
    finally:
        ser.close()
    return None


def dive_cycle(host, handset, checks):
    """DIVE START from the browser: the radio must actually go, and come back.

    This is the diver's real surface experience and nothing else on the bench
    covers it: radio_stress proves the state machine turns over, but only a
    joined station shows that a dive takes the network out from under a client
    and that the surface is reachable again afterwards. The dive has to be closed
    over UART2, because the web cannot close the dive that turned its own radio
    off - which is itself the point of the handset being the in-water transport.
    """
    if not handset:
        checks.append(("--dive-cycle needs --handset", False,
                       "the dive must be closed on the other transport"))
        return
    before_http, before_link = http_code(host), wifi_linked()
    checks.append(("baseline: surface reachable before the dive", before_http == 200,
                   f"http={before_http}, wifi_link={before_link}"))
    if before_http != 200:
        return

    ws = Ws(host)
    ws.send("DIVE START")
    answer, marker, _u, _c = ws.lines_until_marker(limit=3.0)
    print(f"  DIVE START over the WebSocket -> {answer} marker={marker}")

    down_at, gone = None, False
    for i in range(12):
        time.sleep(0.5)
        up = http_code(host, 1.5)
        ln = wifi_linked()
        if up != 200 and (ln is False or ln is None):
            down_at, gone = 0.5 * (i + 1), True
            break
    checks.append(("DIVE START took the radio out from under the client", gone,
                   f"unreachable after {down_at}s" if gone
                   else "the AP was still serving 6 s after DIVE START"))

    died = True
    try:
        ws.lines_until_marker(limit=2.0)
    except Exception:                                             # noqa: BLE001
        died = True
    ws.close()
    checks.append(("the client's socket ends with the radio", died,
                   "read after the teardown raised"))

    closed = uart_answer(handset, "DIVE END", want="OK")
    checks.append(("the dive closes on the other transport",
                   bool(closed) and closed.startswith("OK"),
                   f"UART2 answered {closed!r}"))

    # The firmware side is what this proves. Whether the PC rejoins is the
    # operator's call, so a station that stays away is SKIP, not FAIL.
    broadcast, linked, code = None, wifi_linked(), None
    deadline = time.time() + 25
    while time.time() < deadline:
        broadcast = ap_broadcasting()
        linked = wifi_linked()
        if linked:
            code = http_code(host, 3.0)
            if code == 200:
                break
        time.sleep(1.5)
    checks.append(("the AP is broadcasting again after DIVE END", broadcast,
                   "scan sees it" if broadcast else
                   "no ESP-CCR network in a scan" if broadcast is False else
                   "no nmcli scan available"))
    if not linked:
        checks.append(("surface reachable again after DIVE END", None,
                       "station not linked: reconnecting it is the operator's "
                       "call, so not evaluated"))
        return
    checks.append(("surface reachable again after DIVE END", code == 200,
                   f"http={code}"))

    if code == 200:
        try:
            ws2 = Ws(host)
            ws2.send("GET SP2")
            a, m, _u, _c = ws2.lines_until_marker(limit=4.0)
            ws2.close()
            checks.append(("a fresh WebSocket works after the dive", bool(a) and bool(m),
                           f"{a} marker={m}"))
        except Exception as e:                                    # noqa: BLE001
            checks.append(("a fresh WebSocket works after the dive", False,
                           f"{type(e).__name__}: {e}"))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="192.168.4.1")
    ap.add_argument("--handset", default=None,
                    help="UART2 adapter/bridge port, for the cross-transport check")
    ap.add_argument("--no-dns", action="store_true")
    ap.add_argument("--dive-cycle", action="store_true",
                    help="open a dive from the WebSocket and prove the radio goes "
                         "down under the client and comes back (needs --handset: "
                         "the web cannot close the dive that switched it off)")
    args = ap.parse_args()

    out = []
    print(f"--- HTTP {args.host}")
    checks, page, got = http_probe(args.host, out)
    print("\n".join(out))
    # Bytes, not characters: the page is UTF-8 and carries multi-byte characters
    # (ppO₂, →), so `len(page)` on the decoded text reads 25 642 as 25 640 and the
    # line below contradicts the byte-identity check two lines later.
    print(f"  page size {len(got['/'])} bytes, "
          f"{page.count(chr(10))} lines")
    for name, ok, detail in checks:
        print(f"  [{mark(ok)}] {name}: {detail}")

    if not args.no_dns:
        print("--- captive DNS")
        dns_checks = dns_probe(args.host)
        for name, ok, detail in dns_checks:
            print(f"  [{mark(ok)}] {name}: {detail}")
        checks += dns_checks

    print("--- WebSocket /ws")
    ws_checks = ws_probe(args.host)
    for name, ok, detail in ws_checks:
        print(f"  [{mark(ok)}] {name}\n      {detail}")
    checks += ws_checks

    if args.handset:
        print(f"--- each command twice, back to back: WebSocket then UART2 "
              f"({args.handset})")
        compare_transports(args.host, args.handset, MATRIX, checks)
        print("--- dive export: handset LOG EXPORT vs web /dive/<n>.csv")
        before = len(checks)
        dive_row_equality(args.host, args.handset, checks)
        for name, ok, detail in checks[before:]:
            print(f"  [{mark(ok)}] {name}: {detail}")

    if args.dive_cycle:
        print(f"--- dive cycle from the browser ({args.host})")
        before = len(checks)
        dive_cycle(args.host, args.handset, checks)
        for name, ok, detail in checks[before:]:
            print(f"  [{mark(ok)}] {name}" + (f": {detail}" if detail else ""))

    bad = [c for c in checks if c[1] is False]
    skipped = [c for c in checks if c[1] is None]
    print(f"\n{len(checks)} checks, {len(bad)} failure(s), "
          f"{len(skipped)} not evaluated")
    for name, _, detail in bad:
        print(f"  !! {name}: {detail}")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
