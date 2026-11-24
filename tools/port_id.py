#!/usr/bin/env python3
"""Identify the bench's serial ports without touching either board, and prove
the handset bridge's own UART path with a jumper (no controller involved).

Why this exists: the controller and the handset bridge are the same CP2102
dev-module hardware, and esptool happily flashes EITHER of them. Flashing the
bridge sketch into the controller destroys a dive controller. So: classify the
ports first, then pass the port explicitly to `-t upload`.

How it classifies without resetting anything: pyserial asserts DTR/RTS on
open(), and on these boards DTR=IO0, RTS=EN — the open is a reset pulse (and a
download-mode latch can make the board deaf). Every port here is opened with
DTR/RTS pre-cleared, so listening is passive. Then the content decides:

  own boot banner ("=== ESP-CCR controller", "reset reason:")  -> the CONTROLLER
                                                                  console: NEVER
                                                                  flash here
  HB st=... WITHOUT that banner        -> a LINK carrying the controller's
                                          console stream. This is the wiring
                                          mistake to catch: the console TX
                                          (silkscreen TX = GPIO1) is not the
                                          handset port (GPIO17). The board on
                                          the other end IS the spare, and IS
                                          safe to flash.
  T,/D,/E, lines                       -> the HANDSET downlink (UART2 through
                                          a bridge/adapter): link is live
  ">>>" / MicroPython                  -> spare still has MicroPython on it
  boot log then silence                -> the bridge, in pipe mode (correct:
                                          pipe mode prints nothing of its own)

Note the first version of this script labelled a spare that merely HEARD the
controller's HB lines as "CONTROLLER console". Content from another board is
not that board's own boot, so the two are split apart here.

--loopback PORT answers "is the bridge itself working?" with one jumper and no
controller: put a wire between the spare's GPIO16 and GPIO17 (its UART2 RX and
TX). Pipe mode then hands back exactly what the PC sends, so an echo proves the
PC path, the sketch and the board's UART2 are all fine, and any remaining
failure is in the two signal wires or the controller end.

Never run this while tools/soak_logger.py holds the console port: two readers,
both blind.

Usage:
  python3 tools/port_id.py [--listen SECS] [--ports /dev/ttyUSB0 ...]
  python3 tools/port_id.py --loopback /dev/ttyUSB1 [--repeat 5]
"""
import argparse
import glob
import re
import sys
import time

HB_RE = re.compile(r"^HB st=\d")
# Printed by the controller AT ITS OWN BOOT, on its own console only.
BOOT_RE = re.compile(r"=== ESP-CCR controller|reset reason:|settings: \d+ keys")
TELEM_RE = re.compile(r"^[TDE],")          # T, D, E, downlink records
MICRO_RE = re.compile(r"MicroPython|^>{3,}\s*$")
ROM_RE = re.compile(r"^ets |^load:0x|^entry 0x|^rst:0x|^configsip:|SPI_FAST_FLASH_BOOT")


def open_quiet(port: str):
    """pyserial port with DTR/RTS held low across the open.

    What that buys: no download-mode latch. DTR driving IO0 is what holds the ROM
    in the bootloader, which looks like a dead board, so keeping DTR low matters.

    What it does not buy, measured on the bench 2026-11-24: it does not avoid the
    reset. Opening either CP2102 pulses EN whatever pyserial does with the modem
    lines — open the console and a fresh boot banner arrives inside a second,
    every time. So a "quiet look" at the console is really "restart the controller,
    then read its boot log". Useful, but only if it is treated that way: never
    conclude from an open that the board rebooted by itself, and never compare
    state across one without counting the reset in between. The docstring here
    used to promise no reset pulse, and measurements were made on that promise:
    dive-log state read across a port open is a state read across a reboot.
    """
    import serial  # the pipx platformio venv has it

    s = serial.Serial()
    s.port = port
    s.baudrate = 115200
    s.timeout = 0.5
    s.dtr = False
    s.rts = False
    s.dsrdtr = False
    s.rtscts = False
    s.open()
    return s


def read_lines(s, listen_s: float) -> list[str]:
    out: list[str] = []
    t0 = time.time()
    while time.time() - t0 < listen_s:
        line = s.readline().decode(errors="replace").strip()
        if line:
            out.append(line)
    return out


def classify(lines: list[str]) -> tuple[str, bool]:
    """-> (verdict, safe_to_flash_bridge)"""
    own_boot = any(BOOT_RE.search(l) for l in lines)
    telem = any(TELEM_RE.match(l) for l in lines)
    hb = any(HB_RE.match(l) for l in lines)
    micro = any(MICRO_RE.search(l) for l in lines)

    if own_boot:
        return ("CONTROLLER's own console — NEVER flash the bridge here", False)
    if telem:
        return ("HANDSET link is LIVE: controller UART2 downlink arriving", True)
    if hb:
        return ("relay of the controller's CONSOLE stream — wrong pin: console "
                "TX (silkscreen TX/GPIO1) is not GPIO17", True)
    if micro:
        return ("MicroPython REPL still on this board (bridge not flashed, or "
                "the write failed)", True)
    if lines:
        return ("own boot log, no protocol traffic — bridge in pipe mode "
                "(silence is correct), or nothing wired to it", True)
    return ("silent — blank board, or wiring/power is not what we think", True)


def cmd_listen(args) -> int:
    ports = args.ports or sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
    if not ports:
        print("no serial ports at all — is the USB cable a DATA cable?")
        return 1
    print(f"listening {args.listen:.0f}s per port (DTR/RTS held low: no "
          f"download-mode latch, but the open still resets the board)\n")
    flashed_ok = True
    for p in ports:
        try:
            s = open_quiet(p)
        except Exception as e:                                    # noqa: BLE001
            print(f"{p}: cannot open: {e}\n")
            continue
        try:
            lines = read_lines(s, args.listen)
        finally:
            s.close()
        verdict, safe = classify(lines)
        if not safe:
            flashed_ok = False
        print(f"{p}: {verdict}")
        for l in lines[:4]:
            print(f"    | {l[:120]}")
        if len(lines) > 4:
            print(f"    | ... {len(lines) - 4} more lines")
        if not lines:
            print("    | (nothing)")
        print()

    print("Flash the bridge ONLY to a port that is not the CONTROLLER's own console:")
    print("  pio run -d handset/bridge_esp32 -t upload --upload-port /dev/ttyUSB?")
    print("  (this board needs BOOT held, or GPIO0->GND + power cycle: its bridge")
    print("   chip drives EN but not IO0)")
    print("Then run the acceptance with that port:")
    print("  tools/hw_accept.py --expect-web --handset /dev/ttyUSB?")
    return 0 if flashed_ok else 2


def cmd_loopback(args) -> int:
    """Spare board only: jumper GPIO16 <-> GPIO17, then every byte sent must come
    back. Nothing here involves the controller."""
    port = args.loopback
    print(f"{port}: sending markers, expecting them back (jumper GPIO16 <-> GPIO17")
    print(f"on the BRIDGE board; pipe mode echoes through its own UART2).\n")
    try:
        s = open_quiet(port)
    except Exception as e:                                        # noqa: BLE001
        print(f"cannot open {port}: {e}")
        return 1
    ok = 0
    try:
        time.sleep(args.settle)          # the open resets the bridge: let it boot
        s.reset_input_buffer()
        for i in range(args.repeat):
            marker = f"HS-LOOP-{i}-abc\n"
            s.write(marker.encode())
            s.flush()
            got = s.read(len(marker) + 8).decode(errors="replace")
            if got.strip():
                ok += 1
                print(f"  sent {marker.strip()!r:22} -> got {got.strip()!r}")
            else:
                print(f"  sent {marker.strip()!r:22} -> (nothing back)")
            time.sleep(0.15)
    finally:
        s.close()
    print()
    if ok == args.repeat:
        print("VERDICT: bridge is GOOD (PC path + sketch + its UART2 all work).")
        print("Any remaining silence on the protocol is the two signal wires or")
        print("the controller end: it must be GPIO17 -> bridge GPIO16 and")
        print("GPIO16 <- bridge GPIO17, with GND tied.")
        return 0
    if ok == 0:
        print("VERDICT: no echo. The bridge's own UART2 path is dead: check the")
        print("jumper is on GPIO16/GPIO17 (not TX/RX), and that this board is the")
        print("one running the bridge (a MicroPython REPL would not echo).")
        return 1
    print(f"VERDICT: partial echo ({ok}/{args.repeat}) — marginal link: check")
    print("jumper contact, and that GND is shared with anything driving the pins.")
    return 1


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--listen", type=float, default=8.0,
                    help="seconds to listen per port (T, comes at 2 Hz; a bridge "
                         "needs ~0.3 s to boot after the port opens)")
    ap.add_argument("--ports", nargs="*", default=None)
    ap.add_argument("--loopback", metavar="PORT", default=None,
                    help="run the jumper test on PORT (bridge's GPIO16<->GPIO17)")
    ap.add_argument("--repeat", type=int, default=5)
    ap.add_argument("--settle", type=float, default=1.5,
                    help="seconds to wait after opening the port (that open "
                         "resets an ESP32 behind it)")
    args = ap.parse_args()

    if args.loopback:
        return cmd_loopback(args)
    return cmd_listen(args)


if __name__ == "__main__":
    sys.exit(main())
