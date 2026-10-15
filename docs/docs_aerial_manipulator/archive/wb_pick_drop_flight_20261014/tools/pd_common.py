"""Shared setup for the 2026-10-14 whole-body pick-and-drop FREE flights (T650-AM, 4-D L1, fused feedback, no payload).

  d1  flight_wb_l1_4d_pick_and_drop_20261014_172845   full plastic bin under the drop point; ABORT over the bin
  d2  flight_wb_l1_4d_pick_and_drop_20261014_183914   bin cut down to a holed ~5 cm tray; yawed, operator reverted
npz: ../npz/d1.npz, d2.npz (10-09 extract_bag.py + np1_compat.py); history bags: ../npz/hist/<bag>.npz
(tools/extract_subset.py, a topic subset of the same extractor).
Reuses the 10-09 helpers (../../wb_pick_place_flight_20261009/tools/pp_common.py -> 0928 common.py).
Run with PYTHONNOUSERSITE=1 /usr/bin/python3 (numpy 1.21 + apt matplotlib/scipy).

Conventions (from the 0928 / 10-09 analyses):
  wb_control_debug: tau_flu [21:24] = commanded body torque, ACTUAL FLU frame; d_hat [31:41] = [d_t (world, 3),
  d_r (MODEL body frame, 3), d_rho (4)]; model body = actual body rotated: model +x = actual -y (right),
  model +y = actual +x (forward), z shared. motors [41:45] = commands in ActuatorMotors order.
  ActuatorMotors order on this airframe (PX4 Quad-X): 0 front-right CCW, 1 rear-left CCW, 2 front-left CW,
  3 rear-right CW  (FRONT = arm side = actual body +x).
  WholeBodyReference b1_d is a MODEL heading: actual yaw = atan2(b1_d.y, b1_d.x) + pi/2.
  vehicle_attitude q is NED/FRD (w, x, y, z): ENU yaw = pi/2 - NED yaw.
"""
import os, sys, math
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
NPZ = os.path.abspath(os.path.join(HERE, "..", "npz"))
HIST = os.path.join(NPZ, "hist")
OUT = os.path.abspath(os.path.join(HERE, "..", "analysis"))
FIG = os.path.abspath(os.path.join(HERE, "..", "figures"))
os.environ["AM_NPZ"] = NPZ
T09 = os.path.abspath(os.path.join(HERE, "..", "..", "wb_pick_place_flight_20261009", "tools"))
sys.path.insert(0, T09)
import pp_common as PC   # noqa: E402
from pp_common import WB, wbdebug, mocap_pose, yaw_of, tilt_of, rosout, pp_phases, edges, interp, wrap  # noqa: E402,F401

RUNS = {"d1": ("PD-1", "17:28", "flight_wb_l1_4d_pick_and_drop_20261014_172845", "full bin"),
        "d2": ("PD-2", "18:39", "flight_wb_l1_4d_pick_and_drop_20261014_183914", "cut-down tray")}
MOTOR = ["FR (CCW)", "RL (CCW)", "FL (CW)", "RR (CW)"]
SPIN = np.array([+1.0, +1.0, -1.0, -1.0])          # +1 = CCW rotor (its drag torque yaws the body CW, i.e. -z)
ARM_XY = 0.22990663                                 # rotor x/y offset from the body origin (x650_new.usd), m


def load(path_or_run):
    p = path_or_run
    if not p.endswith(".npz"):
        p = os.path.join(NPZ, f"{p}.npz") if p in RUNS else os.path.join(HIST, f"{p}.npz")
    d = np.load(p)
    t0 = min(d[k][0] for k in d.files if k.endswith("__recv"))
    return d, t0


def px4_rpy(d, t0):
    """vehicle_attitude -> t, roll, pitch, yaw (ENU/FLU, rad)."""
    t = d["vatt__recv"] - t0
    q = d["vatt__q"]                 # NED/FRD w x y z
    w, x, y, z = q.T
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1, 1))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return t, roll, -pitch, wrap(np.pi / 2 - yaw)   # FRD pitch (nose up +) -> FLU pitch (nose down +)


def ref_yaw(d, t0):
    t = d["wbref__recv"] - t0
    return t, wrap(np.arctan2(d["wbref__b1_d.y"], d["wbref__b1_d.x"]) + np.pi / 2)


def direct_windows(d, t0, mode_key="wbmode"):
    """[(t_enter, t_leave)] of every DIRECT stretch."""
    ev = edges(d, t0, mode_key)
    out, a = [], None
    for t, v in ev:
        if v == "DIRECT" and a is None:
            a = t
        elif v != "DIRECT" and a is not None:
            out.append((a, t)); a = None
    if a is not None:
        out.append((a, float(d[f"{mode_key}__recv"][-1] - t0)))
    return out
