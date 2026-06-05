#!/usr/bin/env python
"""
| File: tools/check_driver.py
| Description: Preflight check — verify the NVIDIA driver matches the pinned Isaac Sim build.
|
| Isaac Sim 5.1 is validated on the NVIDIA 580.x driver branch. Driver 595.x is
| known to segfault inside librtx.scenedb.plugin.so ~4 s into SimulationApp boot
| (a widely reported regression). This converts that opaque native crash into a
| clear, early, actionable message.
|
| Design note: this DIAGNOSES a mismatch; it does NOT try to auto-select an Isaac
| Sim version from the driver. The validated stack is pinned (pyproject.toml +
| uv.lock); the driver is the externally-managed variable we only check.
|
| Usage:  uv run python tools/check_driver.py   (exit 0 = ok/unknown, 1 = known-bad)
"""

import shutil
import subprocess
import sys

PINNED_ISAACSIM = "5.1"
VALIDATED_DRIVER_BRANCHES = {"580"}   # major branch(es) known-good for Isaac Sim 5.1
KNOWN_BAD_BRANCHES = {"595"}          # known to crash in librtx.scenedb on boot


def driver_version():
    """Return the NVIDIA driver version string, or None if undeterminable."""
    if not shutil.which("nvidia-smi"):
        return None
    proc = subprocess.run(
        ["nvidia-smi", "--query-gpu=driver_version", "--format=csv,noheader"],
        capture_output=True, text=True,
    )
    out = proc.stdout.strip()
    return out.splitlines()[0].strip() if out else None


def main():
    version = driver_version()
    if version is None:
        print("[preflight] WARN: nvidia-smi not found; cannot verify GPU driver.")
        return 0

    major = version.split(".")[0]
    validated = "/".join(sorted(VALIDATED_DRIVER_BRANCHES))

    if major in KNOWN_BAD_BRANCHES:
        print(
            f"[preflight] ERROR: NVIDIA driver {version} (branch {major}) is known to crash\n"
            f"            Isaac Sim {PINNED_ISAACSIM} in librtx.scenedb.plugin.so on boot.\n"
            f"            This project pins Isaac Sim {PINNED_ISAACSIM}, validated on driver {validated}.x.\n"
            f"            Fix:  sudo apt install nvidia-driver-580-open && sudo reboot\n"
            f"            (see application/rl/README.md -> 'Validated stack')",
            file=sys.stderr,
        )
        return 1

    if major not in VALIDATED_DRIVER_BRANCHES:
        print(
            f"[preflight] WARN: NVIDIA driver {version} (branch {major}) is untested with "
            f"Isaac Sim {PINNED_ISAACSIM} (validated on {validated}.x). Proceeding anyway."
        )
        return 0

    print(f"[preflight] OK: NVIDIA driver {version} matches the validated branch for Isaac Sim {PINNED_ISAACSIM}.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
