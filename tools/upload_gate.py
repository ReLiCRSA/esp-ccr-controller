# ============================================================================
# upload_gate.py — a PlatformIO hook: the audit runs before a board is written
#
# Why this exists, in one line: on 2026-11-28 I flashed the battery build with a
# `%.2f` in a SensorTask printf — the exact bug class this repo's own
# float_audit.py exists to stop, and the class that put the controller into SAFE
# mode twice (see tools/float_audit.py's header). The audit was listed in
# BENCH.md's gate list and I simply did not run it, because running it was a
# step in a document rather than something the toolchain does. BENCH.md's own
# rule for a bug class that returns is to enforce it with a command, so this
# hook makes `pio run -e esp32 -t upload` fail by itself. It calls the audit
# rather than repeating its rules, so the hook and the check cannot drift apart.
#
# Wired in platformio.ini as `extra_scripts = post:tools/upload_gate.py` on the
# esp32 env, and it registers a pre-action on the UPLOAD target only: plain
# builds, `pio test` and `-t uploadfs` are untouched.
# ============================================================================
import os
import subprocess
import sys

# PlatformIO execs this script's text instead of importing it: `env` has to be
# pulled in with Import(), and there is no __file__ (both were found by failing,
# on 2026-11-28, in the first version of this hook). The project directory comes
# from the toolchain so the audit is found through it, not by walking paths.
Import("env")
root = env.get("PROJECT_DIR")


def refuse_before_upload(source, target, env, **kw):
    audit = os.path.join(root, "tools", "float_audit.py")
    # sys.executable is the PlatformIO core python, so no interpreter lookup here.
    rc = subprocess.call([sys.executable, audit], cwd=root)
    if rc != 0:
        print("*** upload refused: tools/float_audit.py failed (exit %d)." % rc)
        print("*** Nothing was written to the board. Render with util/numfmt, "
              "or mark a genuinely boot-only line with /*float-ok: why*/.")
        # Raise rather than env.Exit/exit: this runs inside an SCons action, and
        # an exception is what reliably stops the target (the first version's
        # env.Exit attempt died in the action wrapper instead).
        raise RuntimeError("float_audit failed - upload aborted")


env.AddPreAction("upload", refuse_before_upload)
