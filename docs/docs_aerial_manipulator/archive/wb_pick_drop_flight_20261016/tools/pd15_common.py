"""Shared setup for the 2026-10-16 whole-body pick-and-drop hardware flights (T650-AM, 4-D L1, fused feedback).
Copied from ../../wb_pick_drop_flight_20261015/tools/pd15_common.py; only the run table differs.

  e1  flight_wb_l1_4d_pick_and_drop_empty_20261016_134438   NO payload (control)
  d1  flight_wb_l1_4d_pick_and_drop_20261016_135100         with the basket
  d2  flight_wb_l1_4d_pick_and_drop_20261016_140020         with the basket
npz: ../npz/<run>.npz (tools/run_extract.sh). Reuses the 10-09 helpers (pp_common -> 0928 common.py).
Run with PYTHONNOUSERSITE=1 /usr/bin/python3 (numpy 1.21 + apt matplotlib/scipy).
"""
import os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
NPZ = os.path.abspath(os.path.join(HERE, "..", "npz"))
OUT = os.path.abspath(os.path.join(HERE, "..", "analysis"))
FIG = os.path.abspath(os.path.join(HERE, "..", "figures"))
os.environ["AM_NPZ"] = NPZ
sys.path.insert(0, os.path.abspath(os.path.join(HERE, "..", "..", "wb_pick_place_flight_20261009", "tools")))
import pp_common as PC   # noqa: E402
from pp_common import (WB, wbdebug, mocap_pose, yaw_of, tilt_of, rosout, pp_phases, edges, interp, wrap,  # noqa: E402,F401
                       joints_model, odom, wbref, fk_world, rms, gripper, direct_span, LEGS)

RUNS = {"e1": ("E-1", "13:44", "flight_wb_l1_4d_pick_and_drop_empty_20261016_134438", "no payload"),
        "d1": ("D-1", "13:51", "flight_wb_l1_4d_pick_and_drop_20261016_135100", "basket"),
        "d2": ("D-2", "14:00", "flight_wb_l1_4d_pick_and_drop_20261016_140020", "basket")}
# pick_place/arrival_error (planner publishPpArrival, 10 Hz): leg, claw?, |e|, e_xyz (target - claw, world), tol,
# within, settled, phase
ARR = dict(leg=0, claw=1, norm=2, e=slice(3, 6), tol=6, within=7, settled=8, phase=9)
# pick_place/info: goal per leg k at 14 + 7k: x_b(3), yaw_deg, ee(3)
def info_goal(info_row, k):
    b = 14 + 7 * k
    return info_row[b:b + 3], info_row[b + 3], info_row[b + 4:b + 7]


def load(run):
    d = np.load(os.path.join(NPZ, f"{run}.npz"))
    t0 = min(d[k][0] for k in d.files if k.endswith("__recv"))
    return d, t0


def px4_rpy(d, t0):
    """vehicle_attitude -> t, roll, pitch, yaw in ENU/FLU (rad); pitch > 0 = NOSE DOWN (FLU about +y)."""
    t = d["vatt__recv"] - t0
    w, x, y, z = d["vatt__q"].T            # NED/FRD
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1, 1))   # FRD: nose UP positive
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return t, roll, -pitch, wrap(np.pi / 2 - yaw)


def leg_windows(d, t0):
    """{label: (ta, tb)} for each FLYING leg / phase, from the pick-and-place status."""
    out = {}
    for a, b, lab in pp_phases(d, t0):
        if lab.startswith("FLYING"):
            out[lab.replace("FLYING ", "")] = (a, b)
    return out


def band(x, t, lo, hi):
    """Zero-phase band-pass (Butterworth 2) of an irregularly sampled series, resampled at 100 Hz."""
    from scipy.signal import butter, filtfilt
    tu = np.arange(t[0], t[-1], 0.01)
    xu = np.interp(tu, t, x)
    b, a = butter(2, [lo / 50.0, hi / 50.0], btype="band")
    return tu, filtfilt(b, a, xu)


def obj_pose(d, t0, max_tilt=60.0):
    """Raw VRPN pose of the basket (obj_0) WITHOUT the 180-deg orientation flips the rigid-body solve makes when
    the claw is near (10-09 finding): a flipped sample also moves the solved origin ~20 mm, so drop it whole."""
    to, Po, Qo, Vo = mocap_pose(d, t0, "obj_vrpn")
    ok = tilt_of(Qo) < max_tilt
    return to[ok], Po[ok], Qo[ok]
