"""Shared setup for the 2026-10-02 decoupled (geometric+L1, retuned) flights: reuses the 0928 analysis
(../../wb_vs_decoupled_flight_20260928/tools: common.py's model + loaders, metrics.analyse) on the 1002 bags.

Flights (npz names in $AM_NPZ; r1/r2 are the same file as e1, tp the same as e2):
  r1  13:42 bag, circle run 1 (planner EXECUTING 28.56-56.72 s)
  r2  13:42 bag, circle run 2 (69.48 s - the operator's SAFETY revert at 96.86 s, 0.8 s before the plan end)
  tp  13:48 bag, PS4 teleoperation (planner TELEOP 19.19-96.94 s)
Both runs are scored on the same relative window [0.1, 27.2] s after the run start, so they cover identical parts
of the plan (run 2 lost the last 0.8 s of the slow-down to the revert).
"""
import os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "wb_vs_decoupled_flight_20260928", "tools"))
import common as C            # noqa: E402
import metrics as M           # noqa: E402
from common import *          # noqa: E402,F401

C.FLIGHTS.update({"r1": "1002 run 1", "r2": "1002 run 2", "tp": "1002 PS4"})
RUN_T = 27.3                  # scored length of each circle run [s] (trim 0.1 s each end -> [0.1, 27.2])
_orig_windows = C.windows


def windows(nm):
    if nm not in ("r1", "r2", "tp"):
        return _orig_windows(nm)
    d, t0 = load(nm)
    md = edges(d, t0, "gmode"); a = next(t for t, v in md if v == "DIRECT")
    b = next(t for t, v in edges(d, t0, "ctype") if t > a and v.startswith("Baseline"))   # SAFETY command (operator)
    st = edges(d, t0, "pl_status")
    if nm == "tp":
        c0 = next(t for t, v in st if v == "TELEOP"); return a, b, c0, b
    runs = [t for t, v in st if v.startswith("EXECUTING T=") and float(v.split("=")[1].rstrip("s")) >= 20.0]
    c0 = runs[0] if nm == "r1" else runs[1]
    return a, b, c0, c0 + RUN_T


C.windows = windows; M.windows = windows
# the decoupled law's 2026-10-01 gains (params_..._geometric_l1_..._t650.yaml, fsc_autopilot_ros2 afbeb35)
KP_NEW = np.array([20.11, 20.11, 13.5]); KV_NEW = np.array([11.05, 11.05, 10.82])
KP_OLD = np.array([4.0, 4.0, 8.0]); KV_OLD = np.array([6.0, 6.0, 10.0])
CLOCK_STEP_T = 73.57          # Orin clock stepped +0.6 s (all Orin header stamps; recv-hdr -1.125 -> -1.723 s)
L1_RESET_T = 84.58            # PX4-clock odometry stamps jumped 0.65 s -> the L1 client re-seeded (u_L1 := 0)
