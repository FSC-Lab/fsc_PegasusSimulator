"""
transition_planner.py — dynamically-compatible SETPOINT-TO-SETPOINT
transitions for the whole-body aerial manipulator (2026-08-23).

The engine behind the whole-body REFERENCE GOVERNOR (fsc_autopilot_ros2's
single_aerial_manipulator_whole_body_direct_actuation fork): in whole-body
DIRECT mode the operator's drone-GS base target and arm-GS inertial EE target
define a GOAL rest configuration, and this module plans the compatible
transition from the CURRENT rest setpoint to it — the paper's full reference
set (system-CoM position through snap, base heading, EE position + heading
through 2 derivatives, and the consistent joint reference), with the CoM
trajectory SOLVED from the prescribed EE task exactly like
compatible_trajectory.py's Picard fixed point (thrust-direction <-> arm-
kinematics coupling). A large GS step therefore never reaches the law as a
step.

NEW FILE, additive on purpose: compatible_trajectory.py (the flight-validated
catalogue engine) is imported, never modified.

Everything here lives in the MODEL frame (AM_realign's y-forward body frame —
the frame controller.make_params()/wb_types.hpp declare). The governor owns
every actual<->model conversion at its ROS boundary, mirroring the C++
frame_adapter.

Structure of a transition (all rest-to-rest):

  rest spec        r = {"x_b": base position (world), "phi": MODEL heading
                        angle [rad], "q": joint vector (4,)}
  prescribed task  every scalar channel A->B on ONE shared MIN-SNAP phase
                   sigma(t): EE position straight-line p_e0 -> p_e1, z-x-z
                   EE-orientation angles (al, be, ga), platform heading pp,
                   and the beta split q2/(q2+q3). Min-snap (septic, zero
                   jerk at both ends) is REQUIRED, not a nicety: the solved
                   CoM VELOCITY depends on the prescribed jerk through the
                   thrust-direction map, so min-jerk endpoints would put a
                   CoM-velocity step at the hold<->transition joins.
  rest identity    at a rest point R0 = Rz(phi) exactly, so the z-x-z task
                   angles are (al, be, ga) = (phi + q1, q2 + q3, q4) and the
                   endpoints of every channel are algebraic in (phi, q) — the
                   planned trajectory starts and ends EXACTLY on the holds.
  CoM solve        single-segment Picard fixed point on a degree-`deg`
                   polynomial p_c (the classic planner's flight-validated
                   unconstrained fit), recovered q via the z-x-z
                   decomposition with the time-varying split.

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
GRIPPER_OFF_WRIST = np.array([0.0, 0.0, -0.0494])   # wrist -> pad midpoint [m]


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


def make_params_t650():
    """controller.make_params() with the T650 body override applied CORRECTLY
    (rotated into the model frame — swaps xx<->yy, flips Ixy)."""
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


def _rest_angles(phi, q):
    """(al, be, ga, pp, split) of the rest task at (phi, q)."""
    be = q[1] + q[2]
    split = q[1] / be if abs(be) > 1e-9 else 0.5
    return phi + q[0], be, q[3], phi, split


def plan_transition(params, rest0, rest1, opts=None):
    """Plan the compatible transition rest0 -> rest1.

    rest = {"x_b": (3,) world base position, "phi": model heading [rad],
            "q": (4,) joints}. Returns a plan dict:
      T          duration [s]
      ref(t)     full reference dict (rest_ref keys) at time t (clamped)
      q1         terminal joints (rest1["q"], for the governor's next hold)
      diag       defect / margins / peaks / endpoint mismatch
    Raises ValueError with an operator-readable reason when infeasible.
    """
    o = dict(_DEFAULT_TRANSITION_OPTS, **(opts or {}))
    q0 = np.asarray(rest0["q"], float)
    q1 = np.asarray(rest1["q"], float)
    for name, q in (("start", q0), ("goal", q1)):
        be = np.degrees(q[1] + q[2])
        if be < o["beta_min_deg"]:
            raise ValueError(
                f"{name} fold beta = {be:.1f} deg < {o['beta_min_deg']:.0f} "
                "deg (wrist singularity) — z-x-z recovery is ill-posed")
        if np.any(q < Q_MIN - 1e-9) or np.any(q > Q_MAX + 1e-9):
            raise ValueError(f"{name} joints outside the working range")

    al0, be0, ga0, pp0, sp0 = _rest_angles(float(rest0["phi"]), q0)
    al1, be1, ga1, pp1, sp1 = _rest_angles(float(rest1["phi"]), q1)
    # continuity: goal angles on the branch nearest the start
    al1 = _unwrap_near(al1, al0)
    ga1 = _unwrap_near(ga1, ga0)
    pp1 = _unwrap_near(pp1, pp0)

    p_e0 = rest_ref(params, rest0["x_b"], rest0["phi"], q0)["r_ed"]
    p_e1 = rest_ref(params, rest1["x_b"], rest1["phi"], q1)["r_ed"]
    d_pos = float(np.linalg.norm(p_e1 - p_e0))
    # the CoM travels a comparable distance; base translation bounds it too
    d_base = float(np.linalg.norm(
        np.asarray(rest1["x_b"], float) - np.asarray(rest0["x_b"], float)))
    d_move = max(d_pos, d_base)
    d_ang = max(abs(al1 - al0), abs(be1 - be0), abs(ga1 - ga0),
                abs(pp1 - pp0))

    # duration from the min-snap peak factors
    t_v = PEAK_DS * d_move / o["v_max"] if d_move > 0 else 0.0
    t_a = np.sqrt(PEAK_D2S * d_move / o["a_max"]) if d_move > 0 else 0.0
    t_w = PEAK_DS * d_ang / o["w_max"] if d_ang > 0 else 0.0
    T = float(np.clip(max(t_v, t_a, t_w), o["T_min"], o["T_max"]))

    dp = p_e1 - p_e0

    def task(tq):
        """Prescribed task at time tq — the classic-task structure with every
        channel A->B on one min-snap phase (chain rule for TIME derivs)."""
        s, ds, d2s = CT._minsnap3(min(1.0, max(0.0, tq / T)))
        sd, sdd = ds / T, d2s / T ** 2
        tk = {}
        tk["r_ed"] = p_e0 + dp * s
        tk["r_ed_dot"] = dp * sd
        tk["r_ed_ddot"] = dp * sdd

        al, al1_, al2_ = al0 + (al1 - al0) * s, (al1 - al0), 0.0
        be, be1_, be2_ = be0 + (be1 - be0) * s, (be1 - be0), 0.0
        ga, ga1_, ga2_ = ga0 + (ga1 - ga0) * s, (ga1 - ga0), 0.0
        A, B, Cc = CT._Rz(al), CT._Rx(be), CT._Rz(ga)
        dA = CT._SZ @ A * al1_
        d2A = CT._SZ @ CT._SZ @ A * al1_ ** 2 + CT._SZ @ A * al2_
        dB = CT._SX @ B * be1_
        d2B = CT._SX @ CT._SX @ B * be1_ ** 2 + CT._SX @ B * be2_
        dC = CT._SZ @ Cc * ga1_
        d2C = CT._SZ @ CT._SZ @ Cc * ga1_ ** 2 + CT._SZ @ Cc * ga2_
        re = A @ B @ Cc
        dre = dA @ B @ Cc + A @ dB @ Cc + A @ B @ dC
        d2re = (d2A @ B @ Cc + A @ d2B @ Cc + A @ B @ d2C
                + 2.0 * (dA @ dB @ Cc + dA @ B @ dC + A @ dB @ dC))
        tk["R_e"] = re
        tk["b1_de"] = re[:, 0]
        tk["b1_de_dot"] = dre[:, 0] * sd
        tk["b1_de_ddot"] = d2re[:, 0] * sd ** 2 + dre[:, 0] * sdd

        pp = pp0 + (pp1 - pp0) * s
        pp1_, pp2_ = (pp1 - pp0), 0.0
        cp, sp = np.cos(pp), np.sin(pp)
        b1d = np.array([cp, sp, 0.0])
        b1d_s1 = pp1_ * np.array([-sp, cp, 0.0])
        b1d_s2 = (pp2_ * np.array([-sp, cp, 0.0])
                  + pp1_ ** 2 * np.array([-cp, -sp, 0.0]))
        tk["b1_d"] = b1d
        tk["b1_d_dot"] = b1d_s1 * sd
        tk["b1_d_ddot"] = b1d_s2 * sd ** 2 + b1d_s1 * sdd

        tk["split"] = sp0 + (sp1 - sp0) * s
        return tk

    def recover_q(pcdd, tk):
        """Thrust dir -> R0 -> z-x-z of R0' R_e -> q, with the time-varying
        split (compatible_trajectory's _recover_q, split per-sample)."""
        ac = pcdd + params["g"] * _E3
        r0 = CT._build_R0(ac / np.linalg.norm(ac), tk["b1_d"])
        a, b, c = CT._zxz_angles(r0.T @ tk["R_e"])
        sp = tk["split"]
        return np.array([a, sp * b, (1.0 - sp) * b, c]), r0, ac

    # ---- Picard fixed point on a single-segment polynomial p_c -------------
    _yield = o["yield_hook"]
    _every = max(1, int(o["yield_every"]))
    n = int(o["N"])
    t = np.linspace(0.0, T, n)
    pe = np.zeros((3, n))
    tks = []
    for k in range(n):
        tk = task(t[k])
        tks.append(tk)
        pe[:, k] = tk["r_ed"]
    bounds = [0.0, T]
    pc = CT._fit_pc_segments(t, pe, bounds, o["deg"])
    hist = []
    for _ in range(int(o["maxit"])):
        pcdd = CT._eval_pc(pc, t)[2]
        pc_new = np.zeros((3, n))
        for k in range(n):
            if _yield is not None and k % _every == 0:
                _yield()
            q, r0, _ = recover_q(pcdd[:, k], tks[k])
            r0c, r0e = CT._arm_kin(q, params)
            pc_new[:, k] = pe[:, k] + r0 @ (r0c - r0e)
        pc_cur = CT._eval_pc(pc, t)[0]
        pc_next = CT._fit_pc_segments(
            t, (1.0 - o["relax"]) * pc_cur + o["relax"] * pc_new,
            bounds, o["deg"])
        step = float(np.max(np.linalg.norm(
            CT._eval_pc(pc_next, t)[0] - pc_cur, axis=0)))
        hist.append(step)
        pc = pc_next
        if step < o["tol"]:
            break

    # ---- diagnostics on a fine grid ----------------------------------------
    nf = int(o["Nfine"])
    tf = np.linspace(0.0, T, nf)
    pcf, pc1f, _, _, _ = CT._eval_pc(pc, tf)
    pcddf = CT._eval_pc(pc, tf)[2]
    e_dyn = np.zeros(nf)
    sig_nd = np.zeros(nf)
    qs = np.zeros((4, nf))
    for k in range(nf):
        if _yield is not None and k % _every == 0:
            _yield()
        tk = task(tf[k])
        q, r0, _ = recover_q(pcddf[:, k], tk)
        r0c, r0e = CT._arm_kin(q, params)
        e_dyn[k] = np.linalg.norm(pcf[:, k] - (tk["r_ed"] + r0 @ (r0c - r0e)))
        sig_nd[k] = _sigma_nd(q, params)
        qs[:, k] = q
    # endpoint mismatch of the unconstrained fit vs the exact rest CoM
    x_c0 = rest_ref(params, rest0["x_b"], rest0["phi"], q0)["x_cd"]
    x_c1 = rest_ref(params, rest1["x_b"], rest1["phi"], q1)["x_cd"]
    end_err = max(float(np.linalg.norm(CT._eval_pc(pc, 0.0)[0] - x_c0)),
                  float(np.linalg.norm(CT._eval_pc(pc, T)[0] - x_c1)))

    diag = {
        "T": T, "iterations": len(hist),
        "max_dyn_defect": float(e_dyn.max()),
        "min_sigma_nd": float(sig_nd.min()),
        "q_min_deg": np.degrees(qs.min(axis=1)),
        "q_max_deg": np.degrees(qs.max(axis=1)),
        "peak_com_speed": float(np.max(np.linalg.norm(pc1f, axis=0))),
        "peak_ee_speed": PEAK_DS * d_pos / T if T > 0 else 0.0,
        "endpoint_mismatch": end_err,
    }
    if diag["min_sigma_nd"] < SIGMA_ND_MARGIN:
        raise ValueError(
            f"transition leaves the certified-safe set: min sigma_nd = "
            f"{diag['min_sigma_nd']:.3f} < {SIGMA_ND_MARGIN:.2f}")
    lo_ok = np.all(diag["q_min_deg"] >= np.degrees(Q_MIN) - 1e-6)
    hi_ok = np.all(diag["q_max_deg"] <= np.degrees(Q_MAX) + 1e-6)
    if not (lo_ok and hi_ok):
        raise ValueError(
            "recovered joint path exceeds the working range: "
            f"min {np.round(diag['q_min_deg'], 1).tolist()} "
            f"max {np.round(diag['q_max_deg'], 1).tolist()} deg")
    if diag["endpoint_mismatch"] > 2e-3:
        raise ValueError(
            f"CoM fit endpoint mismatch {diag['endpoint_mismatch'] * 1e3:.2f}"
            " mm — refusing a stepped hold handover")

    def ref(tq):
        tq = float(np.clip(tq, 0.0, T))
        p, p1, p2, p3, p4 = CT._eval_pc(pc, tq)
        tk = task(tq)
        q, _, _ = recover_q(p2, tk)
        # qdot by central FD of the recovered q (endpoints one-sided)
        h = 1e-3
        ta, tb = max(0.0, tq - h), min(T, tq + h)
        qa, _, _ = recover_q(CT._eval_pc(pc, ta)[2], task(ta))
        qb, _, _ = recover_q(CT._eval_pc(pc, tb)[2], task(tb))
        out = {"x_cd": p, "x_cd_dot": p1, "x_cd_ddot": p2,
               "x_cd_d3": p3, "x_cd_d4": p4,
               "q_d": q, "qdot_d": (qb - qa) / (tb - ta)}
        for k in ("b1_d", "b1_d_dot", "b1_d_ddot",
                  "r_ed", "r_ed_dot", "r_ed_ddot",
                  "b1_de", "b1_de_dot", "b1_de_ddot"):
            out[k] = tk[k]
        return out

    if o["verbose"]:
        print(f"[transition] T = {T:.2f} s, {len(hist)} Picard iters, "
              f"defect {diag['max_dyn_defect']:.2e} m, "
              f"sigma_nd >= {diag['min_sigma_nd']:.3f}, "
              f"endpoint mismatch {diag['endpoint_mismatch'] * 1e3:.3f} mm")
    return {"T": T, "ref": ref, "q1": q1.copy(), "diag": diag}


# ===========================================================================
# offline validation
# ===========================================================================

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

    # 2) plan a combined base+arm transition and validate everything
    rest0 = {"x_b": np.array([0.0, 0.0, 1.2]), "phi": 0.0, "q": home}
    rest1 = {"x_b": np.array([0.4, 0.3, 1.4]), "phi": np.deg2rad(20.0),
             "q": q_star}
    plan = plan_transition(params, rest0, rest1, {"verbose": True})
    T = plan["T"]

    # endpoints match the holds
    for tq, rest in ((0.0, rest0), (T, rest1)):
        want = rest_ref(params, rest["x_b"], rest["phi"], rest["q"])
        got = plan["ref"](tq)
        e_q = float(np.max(np.abs(got["q_d"] - want["q_d"])))
        e_pe = float(np.linalg.norm(got["r_ed"] - want["r_ed"]))
        e_pc = float(np.linalg.norm(got["x_cd"] - want["x_cd"]))
        e_v = float(np.linalg.norm(got["x_cd_dot"]))
        print(f"t={tq:5.2f}: |dq|={e_q:.2e} rad, |d r_ed|={e_pe:.2e} m, "
              f"|d x_cd|={e_pc:.2e} m, |v_c|={e_v:.2e} m/s")
        assert e_q < 1e-3 and e_pe < 1e-9 and e_pc < 2e-3 and e_v < 2e-3

    # 3) FD-consistency of every prescribed derivative chain (interior)
    h = 1e-5
    worst = 0.0
    for tq in np.linspace(0.15 * T, 0.85 * T, 9):
        ra, r0_, rb = (plan["ref"](tq - h), plan["ref"](tq),
                       plan["ref"](tq + h))
        for k, kd in (("r_ed", "r_ed_dot"), ("b1_de", "b1_de_dot"),
                      ("b1_d", "b1_d_dot"), ("x_cd", "x_cd_dot"),
                      ("x_cd_dot", "x_cd_ddot")):
            fd = (rb[k] - ra[k]) / (2 * h)
            worst = max(worst, float(np.max(np.abs(fd - r0_[kd]))))
    print(f"FD-consistency (all chains): worst {worst:.2e}")
    assert worst < 1e-4

    # 4) defect + margins already asserted inside plan_transition
    d = plan["diag"]
    print(f"defect {d['max_dyn_defect']:.2e} m | sigma_nd {d['min_sigma_nd']:.3f} | "
          f"q range [{np.round(d['q_min_deg'], 1).tolist()}, "
          f"{np.round(d['q_max_deg'], 1).tolist()}] deg | "
          f"peak CoM speed {d['peak_com_speed']:.3f} m/s")

    # 5) heading-only and arm-only degenerate transitions plan cleanly
    plan_yaw = plan_transition(params, rest0,
                               {"x_b": rest0["x_b"], "phi": np.deg2rad(30.0),
                                "q": home})
    plan_arm = plan_transition(params, rest0,
                               {"x_b": rest0["x_b"], "phi": 0.0, "q": q_star})
    print(f"yaw-only T={plan_yaw['T']:.2f} s, arm-only T={plan_arm['T']:.2f} s"
          f" (defects {plan_yaw['diag']['max_dyn_defect']:.1e} / "
          f"{plan_arm['diag']['max_dyn_defect']:.1e} m)")

    # 6) infeasible targets are refused with a reason
    try:
        plan_transition(params, rest0,
                        {"x_b": rest0["x_b"], "phi": 0.0,
                         "q": np.array([0.0, 0.02, 0.02, 0.0])})
        raise AssertionError("wrist-singular goal was not refused")
    except ValueError as e:
        print(f"refusal (expected): {e}")

    print("=== ALL TRANSITION-PLANNER CHECKS PASSED ===")


if __name__ == "__main__":
    _selftest()
