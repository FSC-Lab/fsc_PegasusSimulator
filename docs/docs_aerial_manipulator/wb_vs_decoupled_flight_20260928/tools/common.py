"""Shared loaders and model for the 2026-09-28 whole-body vs decoupled circle analysis.

Run with `PYTHONNOUSERSITE=1 /usr/bin/python3` (numpy 1.21 + the apt matplotlib); the npz files come from
extract_bag.py + np1_compat.py, directory = $AM_NPZ.
Flights: w1/w2 = whole-body 4-D L1 (2026-09-28 17:11 / 17:22), d1/d2 = decoupled geometric+L1 + position-mode arm
(18:28 / 18:36), a1..a4 = last week's whole-body flights (2026-09-24 12:02 / 12:05 / 16:56 / 17:21).
Conventions (verified in check_conventions.py):
  joint_states order on this rig = [j2, j3, j1, j4] (broadcaster order); hardware->model sign = [-1, 1, 1, -1] on j1..j4
  WholeBodyReference is MODEL frame: positions are world, b1_d/b1_de are model headings; actual yaw = atan2(b1_d)+pi/2
  R0_model = R0_actual * Rz(-90 deg)
"""
import os, sys, math
import numpy as np
np.set_printoptions(precision=4, suppress=True, linewidth=200)
DATA = os.environ.get("AM_NPZ", "/tmp/claude-1000/-home-shiqi-fsc-PegasusSimulator/bf8404ab-c60b-4598-9acf-e8041f10c130/scratchpad/npz")
_REPO = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", ".."))
sys.path.insert(0, os.path.join(_REPO, "extensions", "fsc_aerial_manipulation"))
from fsc_aerial_manipulation.robotic_arm.utils_planner import transition_planner as TP  # noqa: E402

BASE_COM = [0.0, -0.017854, 0.0]            # hardware wb_base_com_* = planner base_com = bridge --base-com
PARAMS = TP.make_params_t650(base_com=BASE_COM)                                   # link armature (the flown bridge / 0924 model)
PARAMS_JD = TP.make_params_t650(base_com=BASE_COM, armature_diag=TP.BENCH_ARMATURE_T650)  # kinematics identical
RZM90 = TP._Rz(-0.5 * math.pi)
G = 9.80665
JS_ORDER = [2, 3, 1, 4]
JS_IDX = [JS_ORDER.index(j) for j in (1, 2, 3, 4)]
HW_SIGN = np.array([-1.0, 1.0, 1.0, -1.0])
KT = np.array([162.4, 154.0, 150.5, 153.4])          # Present Current counts per N.m (kappa_tau), j1..j4
KPWM = np.array([169.47, 149.70, 135.25, 148.51])    # duty counts per N.m (kappa_pwm), j1..j4

FLIGHTS = {
    "w1": "WB-1 17:11", "w2": "WB-2 17:22", "d1": "DEC-1 18:28", "d2": "DEC-2 18:36",
    "a1": "0924 F1 12:02", "a2": "0924 F2 12:05", "a3": "0924 F3 16:56", "a4": "0924 F4 17:21",
}


def load(nm):
    d = np.load(f"{DATA}/{nm}.npz")
    t0 = min(d[k][0] for k in d.files if k.endswith("__recv"))
    return d, t0


def edges(d, t0, key, prefix=""):
    out, last = [], None
    if f"{key}__recv" not in d.files:
        return out
    for ti, v in zip(d[f"{key}__recv"] - t0, d[f"{key}__data"]):
        v = str(v)
        if v != last:
            out.append((float(ti), v)); last = v
    return [e for e in out if e[1].startswith(prefix)]


def windows(nm):
    """(direct0, direct1, circle0, circle1): DIRECT span and the 28.2 s EE-trajectory EXECUTING span (planner status)."""
    d, t0 = load(nm)
    mk = "wbmode" if "wbmode__recv" in d.files else "gmode"
    md = edges(d, t0, mk)
    a = next(t for t, v in md if v == "DIRECT")
    b = next((t for t, v in md if v == "SAFETY" and t > a), None)
    if b is None:
        b = float(d[f"{mk}__recv"][-1] - t0)
    st = edges(d, t0, "pl_status")
    c0 = c1 = None
    for i, (t, v) in enumerate(st):          # the EE-trajectory run = the EXECUTING segment with T >= 20 s
        if v.startswith("EXECUTING T=") and float(v.split("=")[1].rstrip("s")) >= 20.0:
            c0 = t; c1 = st[i + 1][0] if i + 1 < len(st) else b
    if nm == "a1":
        c1 = min(c1, 75.51)                  # 0924 F1: the mocap feed froze at 75.51 s (the fly-away); scored up to it
    return a, b, c0, c1


def quat_R(w, x, y, z):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def quat_cont(Q):
    Q = Q.copy()
    for k in range(1, len(Q)):
        if Q[k] @ Q[k - 1] < 0:
            Q[k] = -Q[k]
    return Q


def interp(tq, t, X):
    X = np.asarray(X)
    if X.ndim == 1:
        return np.interp(tq, t, X)
    return np.column_stack([np.interp(tq, t, X[:, i]) for i in range(X.shape[1])])


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def build_r0(b3, b1d):
    """Planner/bridge attitude from the thrust direction and the model heading b1_d (model frame)."""
    b3 = b3 / np.linalg.norm(b3)
    b2 = np.cross(b3, b1d); b2 /= np.linalg.norm(b2)
    b1 = np.cross(b2, b3)
    return np.column_stack([b1, b2, b3])


def joints_model(d, t0):
    """Measured joints in MODEL convention [rad], j1..j4, from joint_states (all rigs). Returns t, q, qdot."""
    t = d["js__recv"] - t0
    p = d["js__position"][:, :4][:, JS_IDX] * HW_SIGN
    v = d["js__velocity"][:, :4][:, JS_IDX] * HW_SIGN
    ok = np.all(np.isfinite(p), axis=1)
    return t[ok], p[ok], v[ok]


def odom(d, t0):
    t = d["odom__recv"] - t0
    P = np.column_stack([d[f"odom__pose.pose.position.{k}"] for k in "xyz"])
    Q = quat_cont(np.column_stack([d[f"odom__pose.pose.orientation.{k}"] for k in "wxyz"]))
    V = np.column_stack([d[f"odom__twist.twist.linear.{k}"] for k in "xyz"])
    W = np.column_stack([d[f"odom__twist.twist.angular.{k}"] for k in "xyz"])
    return t, P, Q, V, W


def wbref(d, t0):
    t = d["wbref__recv"] - t0
    g = lambda f: np.column_stack([d[f"wbref__{f}.{k}"] for k in "xyz"])
    return t, dict(x_cd=g("x_cd"), x_cd_dot=g("x_cd_dot"), x_cd_ddot=g("x_cd_ddot"), b1_d=g("b1_d"), b1_d_dot=g("b1_d_dot"),
                   r_ed=g("r_ed"), r_ed_dot=g("r_ed_dot"), b1_de=g("b1_de"), q_d=d["wbref__q_d"][:, :4], qdot_d=d["wbref__qdot_d"][:, :4])


def fk_world(P, Q, q, params=PARAMS):
    """Measured CoM x_c, EE r_e, EE heading b1_e (model convention) and R0_model per sample."""
    n = len(P); xc = np.zeros((n, 3)); re = np.zeros((n, 3)); b1e = np.zeros((n, 3)); R0m = np.zeros((n, 3, 3))
    for k in range(n):
        Ra = quat_R(*Q[k]); Rm = Ra @ RZM90
        r0c, r0e, Re = TP.arm_fk_model(q[k], params)
        xc[k] = P[k] + Rm @ r0c; re[k] = P[k] + Rm @ r0e; b1e[k] = Rm @ Re[:, 0]; R0m[k] = Rm
    return xc, re, b1e, R0m


def ref_base(ref):
    """The bridge's conversion of the planner stream: airframe position, model attitude R0, actual yaw, r_0e term."""
    n = len(ref["x_cd"]); xb = np.zeros((n, 3)); R0 = np.zeros((n, 3, 3)); yaw = np.zeros(n)
    for k in range(n):
        R = build_r0(ref["x_cd_ddot"][k] + np.array([0, 0, G]), ref["b1_d"][k])
        r0c, r0e, _ = TP.arm_fk_model(ref["q_d"][k], PARAMS)
        xb[k] = ref["x_cd"][k] - R @ r0c; R0[k] = R
        yaw[k] = math.atan2(ref["b1_d"][k][1], ref["b1_d"][k][0]) + 0.5 * math.pi
    return xb, R0, yaw


def rms(x, axis=0):
    return np.sqrt(np.nanmean(np.asarray(x) ** 2, axis=axis))
