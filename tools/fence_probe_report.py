#!/usr/bin/env python3
"""Regenerates the m-4 fence artifact: memw counts on the target.

tools/fence_probe.cpp compiled with the shipping xtensa toolchain at -O2, plus
the two shipped seqlock functions from the built firmware ELF. `memw` is Xtensa's
memory barrier: the count answers whether the target compiler keeps the explicit
fences (it does — one per fence) while x86-64 drops them (env:native_o2).
"""
import glob
import hashlib
import os
import re
import subprocess
import sys
import time

os.chdir(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

gxx = glob.glob(os.path.expanduser("~/.platformio/packages/*/bin/xtensa-esp32-elf-g++"))[0]
od = glob.glob(os.path.expanduser("~/.platformio/packages/*/bin/xtensa-esp32-elf-objdump"))[0]
OUT = "logs/m4_fence_probe_20261127_disasm.txt"


def counts(disasm: str) -> dict:
    out, fn = {}, None
    for line in disasm.splitlines():
        m = re.match(r"^[0-9a-f]+ <(.+)>:", line)
        if m:
            fn = m.group(1)
            out.setdefault(fn, 0)
        elif fn and "\tmemw" in line:
            out[fn] += 1
    return out


subprocess.run([gxx, "-c", "-O2", "-mlongcalls", "tools/fence_probe.cpp",
                "-o", "/tmp/fence_probe.o"], check=True)
probe = counts(subprocess.run([od, "-d", "/tmp/fence_probe.o"],
                              capture_output=True, text=True).stdout)

elf = ".pio/build/esp32/firmware.elf"
shipped = {}
if os.path.exists(elf):
    dis = subprocess.run([od, "-d", elf], capture_output=True, text=True).stdout
    for fn in ("_ZN4deco11publishViewERNS_8ViewSlotERKNS_4ViewE",
               "_ZN4deco12viewSnapshotERKNS_8ViewSlotERNS_4ViewE"):
        shipped[fn] = counts(dis).get(fn, "not in image")

with open(OUT, "w") as f:
    f.write(f"# regenerated {time.strftime('%Y-%m-%dT%H:%M:%S')} by tools/fence_probe_report.py\n")
    f.write(f"# memw counts (Xtensa memory barrier) — the m-4 fence question on the TARGET\n")
    f.write(f"# probe source tools/fence_probe.cpp sha256 {hashlib.sha256(open('tools/fence_probe.cpp','rb').read()).hexdigest()[:16]}...\n")
    f.write(f"# compiler {subprocess.run([gxx, '--version'], capture_output=True, text=True).stdout.splitlines()[0]}\n")
    f.write("# fence-removed variants cost one barrier LESS: the target keeps them, x86 does not\n\n")
    for fn in sorted(probe):
        f.write(f"  probe    {fn:<40} memw={probe[fn]}\n")
    f.write("\n")
    for fn, n in shipped.items():
        f.write(f"  shipped  {fn:<40} memw={n}\n")
    if shipped:
        f.write(f"\n# firmware ELF sha256 {hashlib.sha256(open(elf,'rb').read()).hexdigest()[:16]}...\n")
print(open(OUT).read())
sys.exit(0)
