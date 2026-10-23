"""Shared loaders for the 2026-09-29 PS4-teleoperation flight (whole-body 4-D L1, T650-AM hardware).

Bag: docs/experimental_data_ros2_bag/0929 - T650-AM whole-body-L1-4D Gamepad-*/.../flight_wb_l1_4d_ps4_20260929_160027
npz: extract_bag.py output, directory $AM_NPZ (default: this session's scratchpad), name p1.npz.

Conventions (same rig as the 0928 analysis, verified there in check_conventions.py):
  joint_states order = [j2, j3, j1, j4, gripper_left, gripper_right]; hardware->model sign [-1, 1, 1, -1] on j1..j4
  WholeBodyReference is MODEL frame; actual yaw = atan2(b1_d) + pi/2; R0_model = R0_actual * Rz(-90 deg)
  wb_control_debug layout: single_aerial_manipulator_whole_body_direct_actuation_client.cpp (publish block)
  teleop/state layout: whole_body_trajectory_planner_node.cpp publishTeleopState()
"""
import os, sys, math
import numpy as np
np.set_printoptions(precision=4, suppress=True, linewidth=200)
DATA = os.environ.get("AM_NPZ", "/tmp/claude-1000/-home-shiqi-fsc-PegasusSimulator/fbaebf87-fe31-4b14-b3fb-445e78181b06/scratchpad/npz")
_REPO = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", ".."))
sys.path.insert(0, os.path.join(_REPO, "extensions", "fsc_aerial_manipulation"))
from fsc_aerial_manipulation.robotic_arm.utils_planner import transition_planner as TP  # noqa: E402

BASE_COM = [0.0, -0.017854, 0.0]
PARAMS = TP.make_params_t650(base_com=BASE_COM, armature_diag=TP.BENCH_ARMATURE_T650)
RZM90 = TP._Rz(-0.5 * math.pi)
G = 9.80665
JS_ORDER = [2, 3, 1, 4]
JS_IDX = [JS_ORDER.index(j) for j in (1, 2, 3, 4)]
HW_SIGN = np.array([-1.0, 1.0, 1.0, -1.0])

# wb_control_debug indices
WB = dict(mode=0, q=slice(5, 9), q_ref=slice(9, 13), tau=slice(13, 17), u1=17, tau_body=slice(18, 21),
          tau_flu=slice(21, 24), e_y=slice(24, 28), e_R=slice(28, 31), d_hat=slice(31, 41), motors=slice(41, 45),
          x_cd=slice(45, 48), x_c=slice(48, 51), n_sat=51, unalloc=slice(52, 56), stream_fresh=56, l1_on=57,
          F_cons=slice(58, 62), d_sig_c=slice(62, 72), w_e=slice(72, 78), w_hat=slice(78, 88), n_clamped=88,
          F_raw=slice(97, 101), w_q=slice(101, 105), chi_free=105, qd_obs=106, qdot=slice(107, 111),
          qdot_pv=slice(111, 115))
# teleop/state indices
TS = dict(state=0, com=slice(1, 4), yaw=4, x_b=slice(5, 8), pe=slice(8, 11), qe=slice(11, 15), q=slice(15, 19),
          s=slice(19, 22), com_blocked=22, arm_blocked=23, leash=24, homing=25, sigma_nd=26, com_speed_xy=27,
          com_speed_z=28, yaw_rate=29, ee_speed=30, roll_rate=31, time_scale=32, joy_fresh=33, armed=34,
          box_lo=slice(35, 39), box_hi=slice(39, 43))
# planner defaults (teleop_axes / teleop_buttons; the hardware yaml sets none)
AXES = [1, 0, 4, 3, 6, 7]          # ee fwd, ee left, ee up, roll, com left (dpad h), com fwd (dpad v)
BUTTONS = [2, 0, 3, 1, 10]         # triangle up, cross down, square yaw left, circle yaw right, PS home
DEADZONE = 0.08


def load(nm="p1"):
    d = np.load(f"{DATA}/{nm}.npz", allow_pickle=True)
    t0 = min(d[k][0] for k in d.files if k.endswith("__recv"))
    return d, t0


def edges(d, t0, key):
    out, last = [], None
    if f"{key}__recv" not in d.files:
        return out
    for ti, v in zip(d[f"{key}__recv"] - t0, d[f"{key}__data"]):
        v = str(v)
        if v != last:
            out.append((float(ti), v)); last = v
    return out


def windows(d, t0):
    """DIRECT span and TELEOP span (planner status)."""
    md = edges(d, t0, "wbmode")
    a = next(t for t, v in md if v == "DIRECT")
    b = next(t for t, v in md if v == "SAFETY" and t > a)
    st = edges(d, t0, "pl_status")
    c = next(t for t, v in st if v == "TELEOP")
    e = next(t for t, v in st if v.startswith("TELEOP_STOP") and t > c)
    return a, b, c, e


def wbdebug(d, t0):
    t = d["wb__recv"] - t0
    D = d["wb__data"]
    ok = D[:, 0] == 1.0          # DIRECT ticks carry the full layout; SAFETY publishes a shorter prefix
    return t[ok], D[ok]


def pad(d, t0):
    """Operator input as the planner reads it (deadzone + rescale), per joy message. Returns t, dict of channels."""
    t = d["joy__recv"] - t0
    A = d["joy__axes"]; B = d["joy__buttons"]

    def ax(i):
        v = A[:, AXES[i]].copy()
        s = np.sign(v); m = np.abs(v)
        return np.where(m < DEADZONE, 0.0, s * np.minimum(1.0, (m - DEADZONE) / (1 - DEADZONE)))
    ch = dict(ee_fwd=ax(0), ee_left=ax(1), ee_up=ax(2), roll=ax(3), com_left=ax(4), com_fwd=ax(5))
    ch["com_up"] = B[:, BUTTONS[0]] - B[:, BUTTONS[1]]
    ch["yaw"] = B[:, BUTTONS[2]] - B[:, BUTTONS[3]]
    ch["home"] = B[:, BUTTONS[4]]
    return t, ch


def quat_R(w, x, y, z):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def tilt_deg(qw, qx, qy, qz):
    """Angle between body z and world z."""
    r22 = 1 - 2 * (qx * qx + qy * qy)
    return np.degrees(np.arccos(np.clip(r22, -1, 1)))


def joints_model(d, t0):
    t = d["js__recv"] - t0
    p = d["js__position"][:, :4][:, JS_IDX] * HW_SIGN
    v = d["js__velocity"][:, :4][:, JS_IDX] * HW_SIGN
    ok = np.all(np.isfinite(p), axis=1)
    return t[ok], p[ok], v[ok]


def runs(mask):
    """[start, end) index pairs of True runs."""
    m = np.concatenate([[False], np.asarray(mask, bool), [False]])
    dm = np.diff(m.astype(int))
    return list(zip(np.where(dm == 1)[0], np.where(dm == -1)[0]))


def rms(x, axis=0):
    return np.sqrt(np.nanmean(np.asarray(x) ** 2, axis=axis))


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi
