"""Shared setup for the 2026-10-09 whole-body pick-and-place hardware flights (T650-AM, 4-D L1, fused feedback).

Reuses the 0928 analysis' model and loaders (../../wb_vs_decoupled_flight_20260928/tools/common.py).
npz: extract_bag.py + np1_compat.py output in ../npz (override with $AM_NPZ):
  p1  flight_wb_l1_4d_pick_and_place_20261009_172557   full mission, COMPLETE
  p2  flight_wb_l1_4d_pick_and_place_20261009_180228   pick + place, then the PX4 -> Orin link froze (fly-away)
  p3  flight_wb_l1_4d_pick_and_place_20261009_181817   pick, then Reset and flown home with the basket
Run with PYTHONNOUSERSITE=1 /usr/bin/python3 (numpy 1.21 + apt matplotlib/scipy).
"""
import os, sys, math, re
HERE = os.path.dirname(os.path.abspath(__file__))
os.environ.setdefault("AM_NPZ", os.path.abspath(os.path.join(HERE, "..", "npz")))
sys.path.insert(0, os.path.join(HERE, "..", "..", "wb_vs_decoupled_flight_20260928", "tools"))
import common as C            # noqa: E402
from common import *          # noqa: E402,F401

OUT = os.path.abspath(os.path.join(HERE, "..", "analysis"))
FIG = os.path.abspath(os.path.join(HERE, "..", "figures"))
RUNS = {"p1": ("PP-1", "17:25", "flight_wb_l1_4d_pick_and_place_20261009_172557"),
        "p2": ("PP-2", "18:02", "flight_wb_l1_4d_pick_and_place_20261009_180228"),
        "p3": ("PP-3", "18:18", "flight_wb_l1_4d_pick_and_place_20261009_181817")}
C.FLIGHTS.update({k: f"{v[0]} {v[1]}" for k, v in RUNS.items()})

# wb_control_debug indices (single_aerial_manipulator_whole_body_direct_actuation_client.cpp, publish block)
WB = dict(mode=0, q=slice(5, 9), q_ref=slice(9, 13), tau=slice(13, 17), u1=17, tau_body=slice(18, 21),
          tau_flu=slice(21, 24), e_y=slice(24, 28), e_R=slice(28, 31), d_hat=slice(31, 41), motors=slice(41, 45),
          x_cd=slice(45, 48), x_c=slice(48, 51), n_sat=51, unalloc=slice(52, 56), stream_fresh=56, l1_on=57,
          F_cons=slice(58, 62), d_sig_c=slice(62, 72), w_e=slice(72, 78), w_hat=slice(78, 88), n_clamped=88,
          F_raw=slice(97, 101), w_q=slice(101, 105), chi_free=105, qd_obs=106, qdot=slice(107, 111),
          qdot_pv=slice(111, 115))
# pick_place/info layout (planner publishPpInfo)
I_ADJ, I_PLANNED, I_DONE, I_FLYING, I_TOL, I_T, I_GOAL, I_CAP, I_PICKYAW = 0, 4, 5, 6, 7, 8, 14, 56, 80
LEGS = ["go_to_start", "execute_pick", "go_to_place_start", "execute_place", "go_to_land_start", "execute_land"]
ANSI = re.compile(r"\x1b\[[0-9;]*m")


def wbdebug(d, t0):
    t = d["wb__recv"] - t0
    D = d["wb__data"]
    ok = D[:, 0] == 1.0          # DIRECT ticks carry the full layout; SAFETY publishes a shorter prefix
    return t[ok], D[ok]


def mocap_pose(d, t0, key="mocap"):
    """/uav_0/mocap or /obj_0/mocap: t (receive), position, quaternion wxyz, velocity."""
    t = d[f"{key}__recv"] - t0
    P = np.column_stack([d[f"{key}__pose.position.{k}"] for k in "xyz"])
    Q = np.column_stack([d[f"{key}__pose.orientation.{k}"] for k in "wxyz"])
    ok = np.all(np.isfinite(P), axis=1) & (np.linalg.norm(Q, axis=1) > 0.5)
    V = np.column_stack([d[f"{key}__twist.linear.{k}"] for k in "xyz"]) if f"{key}__twist.linear.x" in d.files else np.zeros_like(P)
    return t[ok], P[ok], quat_cont(Q[ok]), V[ok]


def yaw_of(Q):
    w, x, y, z = Q[:, 0], Q[:, 1], Q[:, 2], Q[:, 3]
    return np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def tilt_of(Q):
    w, x, y, z = Q[:, 0], Q[:, 1], Q[:, 2], Q[:, 3]
    return np.degrees(np.arccos(np.clip(1 - 2 * (x * x + y * y), -1, 1)))


def rosout(d, t0, name_part=None, contains=None):
    out = []
    if "rosout__recv" not in d.files:
        return out
    for t, n, m in zip(d["rosout__recv"] - t0, d["rosout__name"], d["rosout__msg"]):
        m = ANSI.sub("", str(m)); n = str(n)
        if name_part and name_part not in n:
            continue
        if contains and contains not in m:
            continue
        out.append((float(t), n, m))
    return out


def pp_phases(d, t0):
    """[(t0, t1, label)] from the pick-and-place status (FLYING x / WAITING x / DONE x ...), waiting lines merged."""
    ev = []
    for t, v in edges(d, t0, "pp_status"):
        tok = v.split(" T=")[0]
        if v.startswith("WAITING"):
            tok = "WAITING " + v.split()[1].rstrip(":")
        elif v.startswith("DONE"):
            tok = "DONE " + v.split()[1]
        elif v.startswith("ABORTING"):
            tok = "ABORTING"
        if not ev or ev[-1][1] != tok:
            ev.append((t, tok))
    tend = float(d["pp_status__recv"][-1] - t0)
    return [(ev[i][0], ev[i + 1][0] if i + 1 < len(ev) else tend, ev[i][1]) for i in range(len(ev))]


def direct_span(d, t0):
    """DIRECT entry and the end of the law's ticks (SAFETY edge, or the last DIRECT debug tick)."""
    md = edges(d, t0, "wbmode")
    a = next(t for t, v in md if v == "DIRECT")
    b = next((t for t, v in md if v == "SAFETY" and t > a), None)
    tw, _ = wbdebug(d, t0)
    tw = tw[tw >= a]
    end = float(tw[-1]) if b is None else min(b, float(tw[-1]))
    # the law's feedback must be live: stop where the PX4 -> companion stream stops (flight 2 froze at 144.67 s,
    # one second before the node reverted; np.interp would otherwise draw a line across the gap)
    to = d["odom__recv"] - t0
    gap = np.where((np.diff(to) > 0.5) & (to[:-1] > a) & (to[:-1] < end))[0]
    if len(gap):
        end = float(to[gap[0]]) - 0.05
    return a, end


def gripper(d, t0):
    """Gripper joint position from joint_states (columns after the four arm joints), t."""
    t = d["js__recv"] - t0
    p = d["js__position"]
    return t, p[:, 4:]
