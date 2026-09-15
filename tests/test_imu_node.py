#!/usr/bin/env python3
"""The IMU-force-twin copy (nmpc_cascade_imu_node.py, 2026-09-15): the parts the gate and the boot
test do not exercise, headless.
  1. accel_frd_to_flu: PX4's FRD sample -> FLU specific force (a level hover reads +g up).
  2. the staleness guard: a sample older than IMU_STALE_S is NO reading (None), one warning; a
     fresh one is a reading again; the gate's seam (no receive time) is never stale.
  3. a missing reading ZEROES the model-based cancellation (M_slosh_ahead) instead of holding it.
  4. the estimator kind guard: the copy refuses `--est pwc` (no force channel to replace).
    ~/envs/isaac_env/bin/python3 tests/test_imu_node.py
"""
import io
import os
import subprocess
import sys
import time
import contextlib
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.join(HERE, "..")
sys.path.insert(0, ROOT)
from controller_path import controller_dir     # noqa: E402
PKG = os.path.join(controller_dir(), "px4_vtol_direct_actuator")
sys.path.insert(0, PKG)
sys.argv = [sys.argv[0], "--test", "--est", "mdob", "--slosh_pred", "off"]
from px4_vtol_direct_actuator import nmpc_cascade_imu_node as imu   # noqa: E402
base = imu.base
fails = 0


def check(cond, msg):
    global fails
    print(("PASS " if cond else "FAIL ") + msg); fails += 0 if cond else 1


# 1. the frame
a = imu.accel_frd_to_flu([0.5, 0.2, -9.81])
check(np.allclose(a, [0.5, -0.2, 9.81]), f"FRD (0.5, 0.2, -9.81) -> FLU {a} (hover reads +g up)")
# 2. staleness
l1 = imu.L1Imu(wc=2.0, ks=6.0, umax=(2.5, 2.5, 1.5), delay=0.03, tau_spin=0.12, tau_surf=0.03)
base.IMU_ACCEL = np.array([0.0, 0.0, 9.81]); base.IMU_ACCEL_T = None
check(l1._accel() is not None, "gate seam (no receive time) is never stale")
base.IMU_ACCEL_T = time.monotonic() - 1.0
buf = io.StringIO()
with contextlib.redirect_stdout(buf):
    r1 = l1._accel(); r2 = l1._accel()
check(r1 is None and r2 is None, "a 1 s-old sample is no reading")
check(buf.getvalue().count("stale") == 1, f"exactly one stale warning ({buf.getvalue().count('stale')})")
base.IMU_ACCEL_T = time.monotonic()
buf = io.StringIO()
with contextlib.redirect_stdout(buf):
    r3 = l1._accel()
check(r3 is not None and "back" in buf.getvalue(), "a fresh sample is a reading again, announced once")
# 3. a missing reading zeroes the cancellation
l1.w_hat = np.zeros(3); l1.M_slosh_ahead = np.array([0.0, 0.0, 0.3]); l1.M_slosh = np.array([0.0, 0.0, 0.3]); l1.F_hat = np.array([0.0, 0.5, 0.0])
base.IMU_ACCEL = None; base.IMU_ACCEL_T = None
l1.R = np.eye(3); l1._t = 0.0
try:
    l1.propagate(np.zeros(3), np.zeros(3), 0.005)
    ok = float(np.linalg.norm(l1.M_slosh_ahead)) == 0.0 and float(np.linalg.norm(l1.F_hat)) == 0.0
    check(ok, f"no reading -> M_slosh_ahead {l1.M_slosh_ahead}, F_hat {l1.F_hat} zeroed")
except Exception as e:      # noqa: BLE001
    check(False, f"propagate without a reading raised {e!r}")
# 4. the guard
r = subprocess.run([sys.executable, os.path.join(PKG, "nmpc_cascade_imu_node.py"), "--test", "--est", "pwc", "--dur", "1"],
                   capture_output=True, text=True, timeout=120)
check(r.returncode != 0 and "pwc has no force channel" in (r.stdout + r.stderr), "`--est pwc` is refused")
print("ALL PASS" if not fails else f"{fails} FAILED")
sys.exit(1 if fails else 0)
