"""
transition_planner.py — the whole-body MODEL, FK, IK and rest algebra for the
aerial manipulator (2026-08-23; the straight-line planner removed 2026-09-17).

What lives here now, and what every other tool in this repo imports:

  make_params_t650()      the T650 whole-body model (the flight variant, with
                          the gripper offset), shared with the C++ t650Defaults
  arm_fk_model(q, params) the arm chain: link CoM, EE position, EE rotation
  rest_ref(...)           the 16-field reference of a STATIC rest pose
  ik_position_azimuth /   damped-Newton 4-DOF IK (EE position + heading
  ik_world(...)           azimuth), multi-seed, on the exact chain
  _sigma_nd(q, params)    the nondimensional singularity margin

Consumers: generate_wb_truth.py / generate_wb_l1_truth.py (the C++ parity
fixtures), dump_flat_reference.py, l1_observer.py's self-test, the campaign
drivers and wb_entry_sim.py, and fsc_trajectory_planner's kinematics fixture
(scripts/dump_python_fixtures.py there).

TRANSITIONS ARE PLANNED BY flat_bspline_planner.plan_transition — same rest
specs, same returned reference dict, constraints ENFORCED rather than
verified. The straight-line Picard planner that used to live here was removed
with its C++ backend on 2026-09-17 (see the note where it stood).

Everything here lives in the MODEL frame (AM_realign's y-forward body frame —
the frame controller.make_params()/wb_types.hpp declare); the flight node owns
every actual<->model conversion at its ROS boundary.

Offline validation: run this file directly (system python3, no Isaac):
    python3 transition_planner.py
"""

import importlib.util as _ilu
import os
import sys

import numpy as np

# --- imports: the catalogue engine + the controller model ------------------
if __package__ in (None, ""):
    _HERE = os.path.dirname(os.path.abspath(__file__))
    _EXT = os.path.abspath(os.path.join(_HERE, "..", "..", ".."))
    sys.path.insert(0, _EXT)

from fsc_aerial_manipulation.robotic_arm.utils_controller import (  # noqa: E402
    controller as C,
)
from fsc_aerial_manipulation.robotic_arm.utils_planner import (  # noqa: E402
    compatible_trajectory as CT,
)

_E3 = np.array([0.0, 0.0, 1.0])


# ===========================================================================
# T650 model variant (mirrors application/robotic_arm/utils/generate_wb_truth
# .py's make_params_t650 — the CORRECT model-frame body override; do NOT copy
# 05's un-rotated delta)
# ===========================================================================

# Model frame adapter (AM_realign y-forward <- AM_xfwd x-forward), columns.
R_MODEL = np.array([[0.0, 1.0, 0.0],
                    [-1.0, 0.0, 0.0],
                    [0.0, 0.0, 1.0]])

# AM_xfwd.usda authored /body values (asset ~line 3570; verified 2026-08-22).
AM_XFWD_BODY_MASS = 2.4760795
AM_XFWD_BODY_DIAG_INERTIA = np.diag([0.06334175, 0.06301228, 0.09868092])

# WHERE THE END-EFFECTOR IS (2026-08-23, user's definition: r_e is the GRIPPER).
# controller.make_params() deliberately ends the chain at the WRIST
# (`ee_pos = joint_pos[3]  # EE = the wrist (manip_joint4)`), so l_i[3] is zero
# there. The whole-body task variable y = [r_e; b_1e] is meant to be the
# gripper, so this variant extends the chain by the measured pad midpoint —
# 01_aerial_manipulator_track.py's `PAD_OFF_EE`, the same number that demo adds
# by hand precisely BECAUSE the model's EE is the wrist.
#
# It is very nearly COAXIAL with joint 4 (the asset's gripper-assembly CoM sits
# 39.6 mm along the joint-4 axis with only 5.7 mm off it), and this offset is
# purely along it. That is what keeps the 4-DOF split intact: q4 is a roll about
# the EE axis, so it still CANNOT move r_e — position stays a (q1,q2,q3)
# problem and q4 buys the heading, exactly as the z-x-z recovery assumes.
#
# Applied ONLY here, never in controller.make_params(): the legacy demos
# (01_track, the pick/push plans) are flight-validated against the wrist
# convention and add PAD_OFF_EE themselves where they need the pad, so moving
# the shared model would double-count it.
GRIPPER_OFF_WRIST = np.array([0.0, 0.0, -0.108])   # wrist -> GRASP POINT [m]
# 2026-08-31 (user decision): the EE is the URDF's end_effector_link -- the
# grasp point on the wrist axis between the claw fingertip pads (tips reach
# 0.124), NOT the 0.0494 pad-midpoint measurement it used to be. One point,
# three models: this planner, the C++ wb_model (kGripperOffWrist), and the
# arm GS's l4 all carry 0.108 now; change one, change all, regenerate
# wb_truth_t650.json.


def _load_t650_params_module():
    """t650_params is pure python, but its package __init__ drags in Isaac
    (pxr) — load the module straight from its file (generate_wb_truth's
    trick)."""
    here = os.path.dirname(os.path.abspath(__file__))
    path = os.path.abspath(os.path.join(
        here, "..", "..", "rotorcraft", "t650_params.py"))
    spec = _ilu.spec_from_file_location("t650_params", path)
    mod = _ilu.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def make_params_t650(base_com=None):
    """controller.make_params() with the T650 body override applied CORRECTLY
    (rotated into the model frame — swaps xx<->yy, flips Ixy).

    base_com: bare-airframe CoM relative to the body origin, MODEL frame [m].
    None/zeros is the asset's own answer and what every simulation run uses.
    On HARDWARE it is a flight measurement (see the C++ wb_base_com_* block)
    and it MUST match the value the whole-body node is configured with —
    the whole-body planner builds x_cd from this model while the law computes x_c from
    its own, so a disagreement is a constant CoM-position error of exactly
    (m_base/m_total) * the difference."""
    t650 = _load_t650_params_module()
    p = C.make_params()
    dm = float(t650.BODY_MASS) - AM_XFWD_BODY_MASS
    p["m_i"][0] = p["m_i"][0] + dm
    dI_actual = (np.asarray(t650.INERTIA_TENSOR, dtype=float)
                 - AM_XFWD_BODY_DIAG_INERTIA)
    p["I_i_i"][0] = p["I_i_i"][0] + R_MODEL.T @ dI_actual @ R_MODEL
    # EE = the gripper pad midpoint (see GRIPPER_OFF_WRIST). Only l_i[3] moves;
    # com_i[3] is measured from O_chain[3] = manip_joint4, so every mass
    # property — and therefore M, C, g — is untouched by this.
    p["l_i"][3] = p["l_i"][3] + GRIPPER_OFF_WRIST
    p["base_com"] = (np.zeros(3) if base_com is None
                     else np.asarray(base_com, dtype=float))
    return p


# ===========================================================================
# joint limits (the validated OM-X working range — same numbers as the C++
# WbReferenceBuilder's kQMin/kQMax and every arm config's min/max_position)
# ===========================================================================

# q4 widened to +-120 deg (2026-08-23, user): the WRIST ROLL was the binding
# constraint on EE heading — with the position inside the usable envelope the
# residual refusals were q4 asking for 92-99 deg against a +-90 software stop.
# The asset authors manip_joint4 at +-180, so this stays well inside the
# physical stop. The three configs listed in Command.md 7.14.3 must agree.
Q_MIN = np.array([-0.610865, -1.396263, -0.698132, -2.0943951])
Q_MAX = np.array([0.610865, 0.872665, 0.872665, 2.0943951])

# Non-dimensional singularity margin of the certified-safe set (the
# singularity_sweep analysis; compatible_trajectory prints the same number).
SIGMA_ND_MARGIN = 0.10


def _sigma_nd(q, params):
    """sigma_min of J_3y^0 with translational rows scaled by 1/Lchar."""
    lchar = 0.5 * sum(np.linalg.norm(l) for l in params["l_i"])
    s_nd = np.diag([1.0 / lchar] * 3 + [1.0])
    j = CT._J3y(q, params)
    return float(np.linalg.svd(s_nd @ j, compute_uv=False)[-1])


# ===========================================================================
# rest-point algebra
# ===========================================================================

def _Rz(a):
    return CT._Rz(a)


def arm_fk_model(q, params):
    """(r_0c^0, r_0e^0, R_e^0) at joint config q — base->CoM and base->EE
    offsets in the base frame plus the EE orientation, on the controller's
    exact chain."""
    r0c, r0e = CT._arm_kin(q, params)
    re = np.eye(3)
    for i in range(params["n"]):
        re = re @ CT._rot(params["h_i_im1"][i], q[i])
    return r0c, r0e, re


def rest_ref(params, x_b, phi, q):
    """Full whole-body reference dict for a STATIC HOLD at base position x_b
    (world), model heading phi and joint vector q. All derivatives zero;
    compatible by construction (thrust vertical at rest => R0 = Rz(phi))."""
    x_b = np.asarray(x_b, float)
    q = np.asarray(q, float)
    r0 = _Rz(phi)
    r0c, r0e, re = arm_fk_model(q, params)
    z3 = np.zeros(3)
    return {
        "x_cd": x_b + r0 @ r0c, "x_cd_dot": z3.copy(),
        "x_cd_ddot": z3.copy(), "x_cd_d3": z3.copy(), "x_cd_d4": z3.copy(),
        "b1_d": np.array([np.cos(phi), np.sin(phi), 0.0]),
        "b1_d_dot": z3.copy(), "b1_d_ddot": z3.copy(),
        "r_ed": x_b + r0 @ r0e, "r_ed_dot": z3.copy(), "r_ed_ddot": z3.copy(),
        "b1_de": r0 @ re[:, 0], "b1_de_dot": z3.copy(),
        "b1_de_ddot": z3.copy(),
        "q_d": q.copy(), "qdot_d": np.zeros(4),
    }


def _unwrap_near(a, ref):
    """a + 2*pi*k nearest to ref."""
    return a + 2.0 * np.pi * np.round((ref - a) / (2.0 * np.pi))


# ===========================================================================
# inverse kinematics: EE position + heading azimuth -> q (model chain)
# ===========================================================================

def _azimuth_residual(b1e, az_target):
    """Wrapped angle from b1e's horizontal projection to az_target [rad].
    Raises if the projection is degenerate (b1e near vertical)."""
    h = np.hypot(b1e[0], b1e[1])
    if h < 1e-6:
        raise ValueError("EE heading is vertical — azimuth undefined")
    az = np.arctan2(b1e[1], b1e[0])
    d = az_target - az
    return np.arctan2(np.sin(d), np.cos(d))


def ik_position_azimuth(params, r_e_rel, azim_rel, q_seed,
                        tol=1e-10, maxit=80, step_max=0.5):
    """Solve r_0e^0(q) = r_e_rel and azimuth(b1_e^0(q)) = azim_rel (both in
    the BASE frame, model axes). Damped Newton with an FD Jacobian on the
    exact chain — 4 constraints against 4 joints, square like the paper's
    task. Returns (q, info) with info["ok"], info["residual"], info["sigma_nd"],
    info["limit_ok"]. On a failed primary seed, retries from the folded home
    and from zero."""
    r_e_rel = np.asarray(r_e_rel, float)
    seeds = [np.asarray(q_seed, float),
             np.array([0.0, np.deg2rad(40.0), np.deg2rad(40.0), 0.0]),
             np.zeros(4)]

    def residual(q):
        _, r0e, re = arm_fk_model(q, params)
        return np.concatenate([
            r0e - r_e_rel, [_azimuth_residual(re[:, 0], azim_rel)]])

    best = None
    for seed in seeds:
        q = seed.copy()
        try:
            for _ in range(maxit):
                f = residual(q)
                if np.linalg.norm(f) < tol:
                    break
                jac = np.zeros((4, 4))
                eps = 1e-6
                for j in range(4):
                    qp = q.copy()
                    qp[j] += eps
                    jac[:, j] = (residual(qp) - f) / eps
                dq = np.linalg.lstsq(jac, -f, rcond=None)[0]
                n = np.linalg.norm(dq)
                if n > step_max:
                    dq *= step_max / n
                q = q + dq
            res = float(np.linalg.norm(residual(q)))
        except ValueError:
            continue
        cand = (res, q)
        if best is None or res < best[0]:
            best = cand
        if res < 1e-8:
            break
    if best is None:
        return np.asarray(q_seed, float), {
            "ok": False, "residual": np.inf, "sigma_nd": 0.0,
            "limit_ok": False, "reason": "degenerate EE heading"}
    res, q = best
    q[0] = np.arctan2(np.sin(q[0]), np.cos(q[0]))
    q[3] = np.arctan2(np.sin(q[3]), np.cos(q[3]))
    limit_ok = bool(np.all(q >= Q_MIN - 1e-9) and np.all(q <= Q_MAX + 1e-9))
    sig = _sigma_nd(q, params)
    ok = res < 1e-8 and limit_ok and sig >= SIGMA_ND_MARGIN
    reason = ""
    if res >= 1e-8:
        reason = "IK did not converge (target outside the reachable set?)"
    elif not limit_ok:
        reason = ("joint limits: q = "
                  f"{np.round(np.degrees(q), 1).tolist()} deg vs "
                  f"[{np.round(np.degrees(Q_MIN), 0).tolist()}, "
                  f"{np.round(np.degrees(Q_MAX), 0).tolist()}]")
    elif sig < SIGMA_ND_MARGIN:
        reason = (f"singularity margin: sigma_nd = {sig:.3f} < "
                  f"{SIGMA_ND_MARGIN:.2f}")
    return q, {"ok": ok, "residual": res, "sigma_nd": sig,
               "limit_ok": limit_ok, "reason": reason}


def ik_world(params, x_b, phi, p_e_world, azim_world, q_seed, **kw):
    """World-frame convenience: base pose (x_b, phi) + inertial EE target
    (position, heading azimuth) -> joint vector."""
    r0 = _Rz(phi)
    r_e_rel = r0.T @ (np.asarray(p_e_world, float) - np.asarray(x_b, float))
    return ik_position_azimuth(params, r_e_rel, azim_world - phi, q_seed, **kw)


# ===========================================================================
# transition planning
# ===========================================================================

# Min-snap phase peak factors (computed once, numerically exact).
_UU = np.linspace(0.0, 1.0, 4001)
PEAK_DS = float(max(CT._minsnap3(u)[1] for u in _UU))     # ~2.1875
PEAK_D2S = float(max(abs(CT._minsnap3(u)[2]) for u in _UU))

_DEFAULT_TRANSITION_OPTS = {
    "v_max": 0.30,      # peak EE / CoM translational speed [m/s]
    "a_max": 0.15,      # peak translational acceleration [m/s^2]
    "w_max": 0.30,      # peak angular rate, any angle channel [rad/s]
    "T_min": 3.0,       # never faster than this [s]
    "T_max": 40.0,
    "deg": 16,          # p_c polynomial degree (classic planner default)
    "N": 201,           # Picard grid
    "maxit": 60,
    "tol": 1e-10,
    "relax": 1.0,
    "Nfine": 801,
    "beta_min_deg": 5.0,  # wrist-singularity guard on both endpoints
    "verbose": False,
    # Called PERIODICALLY inside the solve (every `yield_every` grid samples,
    # not merely once per Picard iteration — a single iteration is ~26 ms of
    # solid GIL, which alone overruns a 10 ms timer). The solve is small-array
    # numpy in Python loops, so it holds the GIL almost continuously; a hook
    # that sleeps hands the interpreter back. None = run flat out (the
    # offline/self-test path).
    "yield_hook": None,
    "yield_every": 32,      # grid samples between hook calls
}


# The STRAIGHT-LINE planner (plan_transition + _rest_angles) was REMOVED
# 2026-09-17 with the C++ backend it was the reference for: nothing called it
# any more (the whole-body stack plans every transition with the flat B-spline
# planner, flat_bspline_planner.plan_transition, which takes the same rest
# specs and returns the same reference dict), and its only verification link,
# fsc_trajectory_planner's StraightLine.ParityWithPython, went with it. The
# model, FK, IK and rest algebra above are UNCHANGED and are what the rest of
# this repo imports. Recover the planner from this file before that commit.


def _selftest():
    print("=== transition_planner offline validation (T650 model) ===")
    params = make_params_t650()
    home = np.array([0.0, np.deg2rad(40.0), np.deg2rad(40.0), 0.0])

    # 1) IK round-trip: FK(q*) -> ik_world -> q*
    q_star = np.array([np.deg2rad(15.0), np.deg2rad(30.0),
                       np.deg2rad(35.0), np.deg2rad(-20.0)])
    x_b = np.array([0.2, -0.1, 1.2])
    phi = np.deg2rad(25.0)
    rr = rest_ref(params, x_b, phi, q_star)
    az = np.arctan2(rr["b1_de"][1], rr["b1_de"][0])
    q_ik, info = ik_world(params, x_b, phi, rr["r_ed"], az, home)
    err_ik = float(np.max(np.abs(q_ik - q_star)))
    print(f"IK round-trip: max |dq| = {err_ik:.2e} rad, ok={info['ok']}, "
          f"sigma_nd={info['sigma_nd']:.3f}")
    assert err_ik < 1e-7 and info["ok"]

    # 2) FK <-> rest_ref agreement at a second, unrelated rest
    q2_ = np.array([np.deg2rad(-20.0), np.deg2rad(45.0),
                    np.deg2rad(-10.0), np.deg2rad(60.0)])
    rr2 = rest_ref(params, np.array([-0.3, 0.4, 0.9]), np.deg2rad(-40.0), q2_)
    r0c, r0e, _ = arm_fk_model(q2_, params)
    x_c = rr2["x_cd"]
    p_e = rr2["r_ed"]
    off = np.linalg.norm((x_c - p_e) - _Rz(np.deg2rad(-40.0)) @ (r0c - r0e))
    print(f"rest_ref vs FK chain: |x_c - p_e - R0 (r0c - r0e)| = {off:.2e} m")
    assert off < 1e-12
    print(f"sigma_nd at that rest: {_sigma_nd(q2_, params):.3f}")

    print("=== MODEL / FK / IK CHECKS PASSED ===")


if __name__ == "__main__":
    _selftest()
