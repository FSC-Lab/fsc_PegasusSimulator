#!/usr/bin/env python3
"""interaction_bench.py -- physical interaction with the whole-body 4-D L1 law,
offline (2026-09-27).

QUESTION. Can the 4-D attribution's phase flag chi (wb_l1_contact, plus the
threshold fallback wb_l1_collision_threshold_n) carry a pick-and-place and a
box push, what should trigger it (the gripper, a threshold on the estimated
interaction wrench, or both), and how much payload / push force does the
shipped H1b tune carry before something runs out?

WHAT THIS IS. circle_bench.py's loop (the exact Python law = controller.py +
l1_observer.py, the parity-locked source of the C++; the MIRROR plant
identified from the 0918/0921/0924 flights; hardware-like feedback: 20 ms fused
odometry + calibrated velocity noise, encoder quantisation, the 12 ms arm
velocity observer, 16 ms transport, rotor lag) at RTF 1, with three things
added and nothing in the flown files touched:
  * an ENVIRONMENT acting at the end-effector point, on the TRUE state:
      force   -- a commanded world-frame force, ramped in and out (the user's
                 simplest emulation of a push or a pull);
      box     -- a box with Coulomb friction behind a unilateral contact
                 spring; the vehicle translates 0.3 m into it;
      payload -- a cube resting on desk 1, attached rigidly to the last link
                 when the gripper closes (exact rigid body: mass, CoM and
                 inertia added to link 4), carried to desk 2 and pressed onto
                 it through a unilateral desk spring, released when the
                 gripper opens;
  * a runtime chi schedule -- the Python observer's gains object is live, so
    `contact` can follow the gripper or a task window here although the C++
    node reads it once at startup;
  * the per-joint cap on the observer-driven share of u3 (wb_u3_estimate_max,
    j4 0.06 in the yaml) that wb_entry_sim.Law does not carry, and optional
    HARDWARE servo caps (max_effort [0.34, 2.44, 1.42, 0.39] N.m), which the
    arm controller applies silently below the law's own 3.0 N.m clamp.

Frame: everything here is the MODEL frame at heading 0, where the arm points
along +y. "radial" = along the arm (+y), "lateral" = across it (+x), z up.

It is a screen, like every bench in this repo: trust orderings, mechanisms and
margins; fly the conclusion.
"""
import argparse
import copy
import json
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", "..", ".."))
sys.path.insert(0, os.path.join(REPO, "docs", "docs_aerial_manipulator", "circle_tune_20260927", "tools"))
sys.path.insert(0, os.path.join(REPO, "docs", "docs_aerial_manipulator", "sim2real_tuning_20260926", "tools"))
sys.path.insert(0, os.path.join(REPO, "application", "robotic_arm", "utils"))
sys.path.insert(0, os.path.join(REPO, "extensions", "fsc_aerial_manipulation"))
import circle_bench as CB                                   # noqa: E402
import circle_sweep as CS                                   # noqa: E402
import wb_entry_sim as ES                                   # noqa: E402
from fsc_aerial_manipulation.robotic_arm.utils_controller import controller as C       # noqa: E402
from fsc_aerial_manipulation.robotic_arm.utils_planner import transition_planner as TP  # noqa: E402

hat, vee, E3 = C.hat, C.vee, C.E3
N = 4
H1B = json.load(open(os.path.join(REPO, "docs", "docs_aerial_manipulator", "circle_tune_20260927",
                                  "analysis", "tuned_H1b.json")))["best"]
HW_CAP = np.array([57.6117, 364.8743, 192.0391, 57.6117]) / np.array([169.47, 149.70, 135.25, 148.51])
EST_CAP = np.array([0.0, 0.0, 0.0, 0.06])      # wb_u3_estimate_max_j1..j4 (0 = off)
G = 9.81


# ============================================================== the law
class InteractionLaw(ES.Law):
    """wb_entry_sim.Law restricted to the flown configuration (no posture PID,
    EE reference anchored at the CoM, u3_internal_ff with the 4-D d_int_f)
    plus wb_controller.cpp's per-joint cap on the observer-driven share of u3.
    Returns the observer's attribution internals for scoring."""

    def __call__(self, X, dyn, ref, dt):
        g = self.g; p = self.p; n = p["n"]; m = sum(p["m_i"]); gg = p["g"]
        r_0 = X[0:3]; R_0 = X[3:12].reshape(3, 3, order="F")
        omega_0 = X[15 + n:18 + n]; qdot = X[18 + n:18 + 2 * n]
        v_0 = X[12 + n:15 + n]
        V = np.concatenate([v_0, omega_0, qdot])
        om_hat = hat(omega_0); e3 = E3
        A = dyn["A"]; r_0c_0 = dyn["r_0c_0"]; r_0e_0 = dyn["r_0e_0"]; R_e_0 = dyn["R_e_0"]
        M_r = dyn["M_r"]; C_r = dyn["C_r"]; C_rp = dyn["C_rp"]; C_p = dyn["C_p"]
        J_y = dyn["J_y"]; J_y_dot = dyn["J_y_dot"]; Lambda_y = dyn["Lambda_y"]
        J_1y = dyn["J_1y"]; J_2y = dyn["J_2y"]; J_3y = dyn["J_3y"]
        omega_0e_0 = dyn["omega_0e_0"]; J_we = dyn["J_q_omega_e"]; J_we_dot = dyn["J_q_dot_omega_e"]
        T = dyn["T"]
        xi = T @ V
        xc_dot = xi[0:3]; rho = xi[6:6 + n]
        est = self.l1.update(dyn, R_0, xi, dt)
        d_e_hat = np.concatenate([est["d_t"], est["d_r"], est["d_rho"]])
        d_t_hat = d_e_hat[0:3]; d_r_hat = d_e_hat[3:6]
        if getattr(self, "base_ff", False):
            # WHAT-IF (not in the flown law): split the base rows of the lumped
            # estimate into the internal part, filtered at omega_c_t/r as today,
            # and the ATTRIBUTED contact wrench's image J_y^T F, carried at the
            # reading bandwidth omega_x. In free flight (F_y = 0, chi = 1) the
            # input is d_c itself, so d_t/d_r are bit-identical to the flown law.
            gl = self.l1.g
            if not hasattr(self, "_dbi"):
                self._dbi = np.zeros(6)
            img_raw = np.zeros(6) if est["chi_free"] else (J_y.T @ est["F_raw"])[0:6]
            a6 = np.exp(-np.asarray(gl.omega_c)[0:6] * dt)
            self._dbi = a6 * self._dbi + (1 - a6) * (self.l1._d_c[0:6] - img_raw)
            db = self._dbi + (J_y.T @ est["F_y"])[0:6]
            if self.base_ff == "rot":        # moment rows only; force rows stay lumped
                db[0:3] = d_t_hat
            d_t_hat = np.clip(db[0:3], -gl.max_force_n, gl.max_force_n)
            d_r_hat = np.clip(db[3:6], -gl.max_torque_nm, gl.max_torque_nm)
        # ---- translation
        x_c = r_0 + R_0 @ r_0c_0
        e_x = x_c - ref["x_cd"]; e_vx = xc_dot - ref["x_cd_dot"]
        f_d = -g.k_x * e_x - g.k_v * e_vx + m * ref["x_cd_ddot"] + m * gg * e3 - d_t_hat
        u1 = float(f_d @ (R_0 @ e3))
        xc_ddot = (u1 / m) * (R_0 @ e3) - gg * e3 + (1.0 / m) * d_t_hat
        e_ax = xc_ddot - ref["x_cd_ddot"]
        f_d_dot = -g.k_x * e_vx - g.k_v * e_ax + m * ref["x_cd_d3"]
        u1_dot = float(f_d_dot @ (R_0 @ e3) + f_d @ (R_0 @ om_hat @ e3))
        xc_d3 = (u1_dot / m) * (R_0 @ e3) + (u1 / m) * (R_0 @ om_hat @ e3)
        e_jx = xc_d3 - ref["x_cd_d3"]
        f_d_ddot = -g.k_x * e_ax - g.k_v * e_jx + m * ref["x_cd_d4"]
        # ---- rotation
        nf = np.linalg.norm(f_d); b3c = f_d / nf
        P1 = np.eye(3) - np.outer(b3c, b3c)
        s = P1 @ ref["b1_d"]; b3c_dot = (1.0 / nf) * P1 @ f_d_dot
        ns = np.linalg.norm(s); b1c = s / ns
        P1_dot = -(np.outer(b3c_dot, b3c) + np.outer(b3c, b3c_dot))
        P2 = np.eye(3) - np.outer(b1c, b1c)
        s_dot = P1_dot @ ref["b1_d"] + P1 @ ref["b1_d_dot"]
        b1c_dot = (1.0 / ns) * P2 @ s_dot
        P2_dot = -(np.outer(b1c_dot, b1c) + np.outer(b1c, b1c_dot))
        b2c = np.cross(b3c, b1c)
        R0c = np.column_stack([b1c, b2c, b3c])
        b2c_dot = np.cross(b3c_dot, b1c) + np.cross(b3c, b1c_dot)
        R0c_dot = np.column_stack([b1c_dot, b2c_dot, b3c_dot])
        omega_0c = vee(R0c.T @ R0c_dot)
        b3c_ddot = ((1.0 / nf) * (P1_dot @ f_d_dot + P1 @ f_d_ddot) - (b3c @ f_d_dot) / nf**2 * (P1 @ f_d_dot))
        P1_ddot = -(np.outer(b3c_ddot, b3c) + 2 * np.outer(b3c_dot, b3c_dot) + np.outer(b3c, b3c_ddot))
        s_ddot = P1_ddot @ ref["b1_d"] + 2 * P1_dot @ ref["b1_d_dot"] + P1 @ ref["b1_d_ddot"]
        b1c_ddot = (1.0 / ns) * (P2_dot @ s_dot + P2 @ s_ddot) - (b1c @ s_dot) / ns * b1c_dot
        b2c_ddot = (np.cross(b3c_ddot, b1c) + 2 * np.cross(b3c_dot, b1c_dot) + np.cross(b3c, b1c_ddot))
        R0c_ddot = np.column_stack([b1c_ddot, b2c_ddot, b3c_ddot])
        omega_0c_dot = vee(R0c_dot.T @ R0c_dot + R0c.T @ R0c_ddot)
        e_R = 0.5 * vee(R0c.T @ R_0 - R_0.T @ R0c)
        e_w = omega_0 - R_0.T @ R0c @ omega_0c
        u2 = (M_r @ (R_0.T @ R0c @ omega_0c_dot - om_hat @ R_0.T @ R0c @ omega_0c
                     - np.linalg.solve(g.M_r_d, g.k_R * e_R + g.k_w * e_w))
              + C_r @ omega_0 + C_rp @ rho - d_r_hat)
        # ---- arm
        R_e = R_0 @ R_e_0; R_0e = R_e_0.T
        r_e = r_0 + R_0 @ r_0e_0
        omega_e = R_0e @ (omega_0 + omega_0e_0); omega_e_hat = hat(omega_e)
        omega_e_dot = R_0e @ (J_we_dot @ qdot + om_hat @ omega_0e_0)
        omega_e_dot_hat = hat(omega_e_dot)
        b2e = R_e @ np.array([0.0, 1, 0]); b3e = R_e @ e3
        ydot = J_y @ xi; r_e_dot = ydot[0:3]
        omega_3e = float(omega_e @ e3)
        P1e = np.eye(3) - np.outer(b3e, b3e)
        s_e = P1e @ ref["b1_de"]; b3e_dot = R_e @ omega_e_hat @ e3
        nse = np.linalg.norm(s_e); b1ec = s_e / nse
        P1e_dot = -(np.outer(b3e_dot, b3e) + np.outer(b3e, b3e_dot))
        P2e = np.eye(3) - np.outer(b1ec, b1ec)
        s_e_dot = P1e_dot @ ref["b1_de"] + P1e @ ref["b1_de_dot"]
        b1ec_dot = (1.0 / nse) * P2e @ s_e_dot
        P2e_dot = -(np.outer(b1ec_dot, b1ec) + np.outer(b1ec, b1ec_dot))
        b3e_ddot = R_e @ (omega_e_hat @ omega_e_hat + omega_e_dot_hat) @ e3
        P1e_ddot = -(np.outer(b3e_ddot, b3e) + 2 * np.outer(b3e_dot, b3e_dot) + np.outer(b3e, b3e_ddot))
        s_e_ddot = P1e_ddot @ ref["b1_de"] + 2 * P1e_dot @ ref["b1_de_dot"] + P1e @ ref["b1_de_ddot"]
        b1ec_ddot = (1.0 / nse) * (P2e_dot @ s_e_dot + P2e @ s_e_ddot) - (b1ec @ s_e_dot) / nse * b1ec_dot
        omega_3ec = float(b3e @ np.cross(b1ec, b1ec_dot))
        omega_3ec_dot = float(b3e_dot @ np.cross(b1ec, b1ec_dot) + np.cross(b3e, b1ec) @ b1ec_ddot)
        # EE reference anchored at the CoM (wb_ee_anchor_com)
        r_ed = ref["r_ed"] + e_x; r_ed_dot = ref["r_ed_dot"] + e_vx; r_ed_ddot = ref["r_ed_ddot"] + e_ax
        e_xE = r_e - r_ed
        e_y = np.array([*e_xE, float(-(b1ec @ b2e))])
        e_vy = np.array([*(r_e_dot - r_ed_dot), omega_3e - omega_3ec])
        yddot_d = np.array([*r_ed_ddot, omega_3ec_dot])
        M_y = self.cfg.M_Y
        Lam_Myinv = Lambda_y @ np.linalg.solve(M_y, np.eye(4))
        F_hat_y = est["F_y"]
        F_trans = u1 * (R_0 @ e3) - m * gg * e3
        tau_rot = u2 - C_r @ omega_0 - C_rp @ rho
        coupling_ff = J_1y @ F_trans + J_2y @ tau_rot
        est_task = -(Lam_Myinv - np.eye(4)) @ F_hat_y
        d_int = est["d_int_f"]                       # u3_internal_ff_use_w_hat, four_d
        d_int_task = J_1y @ d_int[0:3] + J_2y @ d_int[3:6] + J_3y @ d_int[6:]
        coupling_ff = coupling_ff + d_int_task
        est_task = est_task + d_int_task
        inner = (coupling_ff + Lambda_y @ (J_y_dot @ xi - yddot_d)
                 + Lam_Myinv @ (self.cfg.D_y @ e_vy + self.cfg.K_y @ e_y)
                 - (Lam_Myinv - np.eye(4)) @ F_hat_y)
        JJt = J_3y @ J_3y.T + self.cfg.dls_lambda ** 2 * np.eye(4)
        sol = lambda v: J_3y.T @ np.linalg.solve(JJt, v)   # noqa: E731
        u3_est = -sol(est_task)
        cap = np.where(EST_CAP > 0, EST_CAP, np.inf)
        u3 = -sol(inner - est_task) + np.clip(u3_est, -cap, cap) - C_rp.T @ omega_0 + C_p @ rho
        u = np.concatenate([u1 * (R_0 @ e3), u2, u3])
        tau = T.T @ u
        tau_joint = tau[6:]
        n_sat = 0
        tm = self.cfg.tau_max
        tcl = np.clip(tau_joint, -tm, tm)
        n_sat = int(np.count_nonzero(tcl != tau_joint))
        if n_sat:
            tau_joint = tcl
            u3 = tau_joint - u1 * (A.T @ e3)
            u = np.concatenate([u1 * (R_0 @ e3), u2, u3])
            tau = T.T @ u
        self.l1.propagate(dyn, xi, u, dt)
        return {"u1": u1, "tau_body": tau[3:6], "tau_joint": tau_joint, "e_x": e_x, "e_y": e_y,
                "e_R": e_R, "n_sat": n_sat, "d_t_hat": d_t_hat, "d_r_hat": d_r_hat, "F_hat_y": F_hat_y,
                "u3": u3, "est": est, "u3_est": u3_est}


# ============================================================ references
def minsnap(tau):
    """Septic rest-to-rest profile s(tau), tau in [0,1], and d^k s/dtau^k, k=1..4."""
    t = np.clip(tau, 0.0, 1.0)
    s = 35 * t**4 - 84 * t**5 + 70 * t**6 - 20 * t**7
    s1 = 140 * t**3 - 420 * t**4 + 420 * t**5 - 140 * t**6
    s2 = 420 * t**2 - 1680 * t**3 + 2100 * t**4 - 840 * t**5
    s3 = 840 * t - 5040 * t**2 + 8400 * t**3 - 4200 * t**4
    s4 = 840 - 10080 * t + 25200 * t**2 - 16800 * t**3
    if tau <= 0.0 or tau >= 1.0:
        s1 = s2 = s3 = s4 = 0.0
    return s, s1, s2, s3, s4


class Schedule:
    """A rest hold plus a list of rigid translations (t0, T, dvec): the WHOLE
    reference (CoM chain and EE chain) moves together at a fixed arm pose, the
    compatible reference of a pure translation (tilt-free along z; along x/y
    the arm offset is rotated by the ~1 deg acceleration tilt, which the
    CoM-anchored EE reference absorbs)."""

    def __init__(self, base_ref):
        self.base = base_ref
        self.moves = []

    def add(self, t0, T, dvec):
        self.moves.append((float(t0), float(T), np.asarray(dvec, float)))
        return t0 + T

    def __call__(self, t):
        off = np.zeros((5, 3))
        for t0, T, dv in self.moves:
            s = minsnap((t - t0) / T)
            for k in range(5):
                off[k] += dv * s[k] / T**k
        r = {k: (v.copy() if isinstance(v, np.ndarray) else v) for k, v in self.base.items()}
        for k, nm in enumerate(("", "_dot", "_ddot", "_d3", "_d4")):
            if k == 0:
                r["x_cd"] = r["x_cd"] + off[0]; r["r_ed"] = r["r_ed"] + off[0]
            else:
                if "x_cd" + nm in r:
                    r["x_cd" + nm] = r["x_cd" + nm] + off[k]
                if k <= 2:
                    r["r_ed" + nm] = r["r_ed" + nm] + off[k]
        return r


# ============================================================== environment
def link4_with_payload(params, mp, side=0.05):
    """Exact rigid attachment: a point-ish cube of mass mp at the grasp point
    (l_i[3] in the link-4 frame) merged into link 4."""
    p = copy.deepcopy(params)
    if mp <= 0:
        return p
    m4 = p["m_i"][4]; c4 = np.asarray(p["com_i"][3], float); l4 = np.asarray(p["l_i"][3], float)
    mn = m4 + mp
    cn = (m4 * c4 + mp * l4) / mn
    S = lambda r: (r @ r) * np.eye(3) - np.outer(r, r)   # noqa: E731
    In = np.asarray(p["I_i_i"][4], float) + m4 * S(c4 - cn) + mp * S(l4 - cn) + mp * side**2 / 6 * np.eye(3)
    p["m_i"][4] = mn; p["com_i"][3] = cn; p["I_i_i"][4] = In
    return p


class Box:
    """A box behind a unilateral contact spring, Coulomb friction with stiction."""

    def __init__(self, face, direction, f_kin, mu_ratio=1.25, m=0.5, k=3000.0, c=30.0):
        self.b = float(face); self.v = 0.0; self.d = np.asarray(direction, float)
        self.fk = float(f_kin); self.fs = self.fk * mu_ratio; self.m = m; self.k = k; self.c = c
        self.moved = 0.0

    def force(self, r_e, v_e, dt):
        """World force ON THE END-EFFECTOR, and advance the box."""
        pe = float(self.d @ r_e); ve = float(self.d @ v_e)
        delta = pe - self.b
        fc = max(0.0, self.k * delta + self.c * (ve - self.v)) if delta > 0 else 0.0
        if self.v == 0.0 and fc <= self.fs:
            pass                                      # stuck
        else:
            fr = self.fk * (np.sign(self.v) if self.v != 0.0 else np.sign(fc))
            vn = self.v + (fc - fr) / self.m * dt
            if self.v != 0.0 and np.sign(vn) != np.sign(self.v):
                vn = 0.0                              # friction stops it
            self.v = vn
        self.b += self.v * dt; self.moved += self.v * dt
        return -fc * self.d, fc


# ================================================================ policies
def chi_policy(name):
    """name = <base>[+thr<N>], base in free|contact|gripper|task.
    -> (l1 overrides, runtime flag function(events) -> contact)."""
    parts = name.split("+")
    base = parts[0]
    ov = {}
    for p_ in parts[1:]:
        if p_.startswith("thr"):
            ov["collision_threshold_n"] = float(p_[3:])
    fn = {"free": (lambda ev: False), "contact": (lambda ev: True),
          "task": (lambda ev: ev["task"]), "gripper": (lambda ev: ev["grip"])}[base]
    return ov, fn


def reading_bw(rd):
    """The WRENCH reading/rendering bandwidth omega_x, decoupled from the u3
    feed-forward's per-block omega_x_t/r/q (which keep the H1b value, so free
    flight is unchanged: F_y is gated to zero there)."""
    if rd is None:
        return {}
    w = H1B["omega_x"]
    return dict(omega_x=float(rd), omega_x_t=w, omega_x_r=w, omega_x_q=w)


# ================================================================ simulate
def simulate(scn, chi="free", seed=0, hw_caps=True, fb=None, gains=None, l1_over=None,
             q0=(0.0, 27.1, 38.8, 0.0), keep=False, rd=None, **kw):
    """scn: 'force' | 'box' | 'payload' | 'hover'. Returns metrics (and traces)."""
    p = dict(H1B if gains is None else {**H1B, **gains})
    fb = dict(CB.FB if fb is None else fb)
    rng = np.random.default_rng(seed)
    model = TP.make_params_t650(armature_diag=CS.J_ARM)
    l1 = CB.make_l1(p)
    pol_over, pol_fn = chi_policy(chi)
    l1.update(pol_over)
    l1.update(reading_bw(rd))
    if l1_over:
        l1.update(l1_over)
    case = ES.Case(posture=0.0, ee_ref="relative", int_ff=True, int_ff_source="four_d",
                   mass=1.0, inertia=1.0, com=(-0.017854, 0.0, 0.0), kf=1.0, kf_alloc=1.0,
                   observer="l1", delay_ms=16.0, gains={}, l1=l1)
    plant0 = ES.plant_params(model, case)
    model["base_com"] = TP.R_MODEL.T @ np.array([-0.017854, 0.0, 0.0])
    kf_alloc, km_ratio, fric_plant = CS.KF_ALLOC, CS.KM_ALLOC_RATIO, CS.FRIC_SCALE
    fbias_m = TP.R_MODEL.T @ CS.FORCE_BIAS_BODY; tbias_m = TP.R_MODEL.T @ CS.TORQUE_BIAS_BODY
    kf_plant, km_plant = CS.KF_PLANT, CS.KM_PLANT
    cfg = CB.make_cfg(p)
    law = InteractionLaw(model, cfg, case)
    law.base_ff = kw.pop("base_ff", False)
    dt = case.dt
    B = np.zeros((4, 4))
    for i, r in enumerate(ES.ROTOR_POS):
        B[0, i] = 1.0; B[1, i] = r[1]; B[2, i] = -r[0]; B[3, i] = ES.ROT_DIR[i] * km_ratio
    Bp = np.linalg.pinv(B); rp = ES.ROTOR_POS; rdir = ES.ROT_DIR

    def allocate(thrust, tau):
        f = Bp @ np.array([thrust, *tau])
        return np.clip(np.sqrt(np.maximum(f, 0.0) / kf_alloc), 0.0, ES.OMEGA_MAX), f

    # ---- initial hover at 1.2 m, arm at q0, heading 0 --------------------
    q0 = np.radians(np.asarray(q0, float))
    X = np.zeros(12 + 3 * N + 6)
    X[0:3] = [0.0, 0.0, 1.2]; X[3:12] = np.eye(3).flatten(order="F"); X[12:12 + N] = q0
    base = TP.rest_ref(model, X[0:3], 0.0, q0)
    sch = Schedule(base)
    plant = plant0
    gp = C.dynamics(X, plant)["g"]
    omega_rot, _ = allocate(gp[2], gp[3:6])
    n_del = int(round(16e-3 / dt)); fifo = [omega_rot.copy() for _ in range(n_del)]
    n_odo = int(round(fb["odo_lag_s"] / dt)); n_att = int(round(fb["att_lag_s"] / dt))
    hist_X = [X.copy() for _ in range(max(n_odo, n_att) + 1)]
    vnoise = CB._AR1(rng, fb["vel_noise"], fb["vel_bw_hz"], dt, 3)
    a_qd = np.exp(-dt / fb["qd_lag_s"]) if fb["qd_lag_s"] > 0 else 0.0
    qd_obs = np.zeros(N)
    r_e0 = base["r_ed"].copy()

    # ---- the scenario -----------------------------------------------------
    t_settle = kw.get("t_settle", 15.0)           # DIRECT entry transient + trims converge
    ev = {"task": False, "grip": False}
    env = {"F": np.zeros(3)}
    rad = np.array([0.0, 1.0, 0.0]); lat = np.array([1.0, 0.0, 0.0]); up = E3
    dirs = {"radial": rad, "-radial": -rad, "lateral": lat, "-lateral": -lat, "up": up, "down": -up}
    marks = {}
    if scn == "force":
        F0 = float(kw.get("F", 2.0)); dv = dirs[kw.get("dir", "radial")]
        t_on = t_settle; T_r = kw.get("ramp", 1.0); T_h = kw.get("hold", 15.0)
        t_off = t_on + T_r + T_h
        t_end = t_off + T_r + 12.0
        marks.update(t_on=t_on, t_off=t_off)

        def env_step(t, r_e, v_e, a_e):
            s_on = minsnap((t - t_on) / T_r)[0]; s_off = minsnap((t - t_off) / T_r)[0]
            ev["task"] = (t_on - 0.5) <= t <= (t_off + T_r + 2.0)
            return F0 * (s_on - s_off) * dv, F0 * (s_on - s_off) * dv
    elif scn == "box":
        f_kin = float(kw.get("F", 2.0)); dv = dirs[kw.get("dir", "radial")]
        gap = 0.02; preload = kw.get("preload", 0.01); push = kw.get("push", 0.30)
        box = Box(float(dv @ r_e0) + gap, dv, f_kin, k=kw.get("k_env", 3000.0))
        t0 = t_settle
        t1 = sch.add(t0, 2.0, dv * (gap + preload))           # approach + preload
        t2 = sch.add(t1 + 2.0, kw.get("T_push", 8.0), dv * push)  # push 0.3 m
        t3 = sch.add(t2 + 4.0, 2.0, -dv * 0.06)                # back off
        t_end = t3 + 8.0
        marks.update(t_contact=t0, t_push0=t1 + 2.0, t_push1=t2, t_back=t2 + 4.0)

        def env_step(t, r_e, v_e, a_e):
            ev["task"] = t0 <= t <= t3 + 1.0
            f, fc = box.force(r_e, v_e, dt)
            return f, f
    elif scn == "payload":
        mp = float(kw.get("m", 0.1)); side = 0.05; press = kw.get("press", 0.01)
        k_desk = kw.get("k_env", 5000.0); c_desk = 40.0
        z_desk = float(r_e0[2]) - side / 2          # desk-1 top: cube centre = EE at t_close
        carry = np.asarray(kw.get("carry", (-0.6, 0.0, 0.0)), float)
        st = {"att": False, "z_desk": z_desk}
        t_close = t_settle; t_lift0 = t_close + 2.0
        t1 = sch.add(t_lift0, 3.0, up * 0.10)
        t2 = sch.add(t1 + 1.0, 6.0, carry)
        t3 = sch.add(t2 + 1.0, 3.0, -up * (0.10 + press))
        t_open = t3 + kw.get("place_hold", 8.0)
        if kw.get("unload", False):        # take the press off BEFORE opening
            sch.add(t_open, 1.5, up * press)
            t_open += 2.5
        t4 = sch.add(t_open + 2.0, 3.0, up * (0.10 + press))
        t_end = t4 + 8.0
        marks.update(t_close=t_close, t_lift=t_lift0, t_carry0=t1 + 1.0, t_carry1=t2, t_place0=t2 + 1.0,
                     t_place1=t3, t_open=t_open, t_rise=t_open + 2.0)
        plant_att = link4_with_payload(plant0, mp, side)

        def env_step(t, r_e, v_e, a_e):
            nonlocal plant
            if not st["att"] and t_close <= t < t_open:
                st["att"] = True; plant = plant_att
            if st["att"] and t >= t_open:
                st["att"] = False; plant = plant0
            ev["grip"] = t_close <= t < t_open
            ev["task"] = t_close - 0.5 <= t <= t_open + 2.0
            if not st["att"]:
                return np.zeros(3), np.zeros(3)
            zb = r_e[2] - side / 2
            delta = st["z_desk"] - zb
            n_f = max(0.0, k_desk * delta - c_desk * v_e[2]) if delta > 0 else 0.0
            f_env = np.array([0.0, 0.0, n_f])               # desk on the payload -> on the EE
            f_equiv = f_env + mp * (-G * E3 - a_e)          # what the arm feels: weight + inertia + desk
            return f_env, f_equiv
    else:   # hover
        t_end = t_settle + kw.get("hold", 20.0)

        def env_step(t, r_e, v_e, a_e):
            return np.zeros(3), np.zeros(3)

    steps = int(t_end / dt)
    keys = ("t", "ee", "ex", "tilt", "chi", "fn_true", "fhat", "nsat", "rot_sat")
    H = {k: np.zeros(steps) for k in keys}
    H3 = {k: np.zeros((steps, 3)) for k in ("F_true", "F_hat_f", "F_raw", "e_ee", "e_x", "d_t")}
    H4 = {k: np.zeros((steps, N)) for k in ("tau", "q", "wq")}
    Hbox = np.zeros(steps)
    verdict = "completed"; k_end = steps
    v_e_prev = np.zeros(3)
    for k in range(steps):
        t = k * dt
        ref = sch(t)
        # ---- feedback (circle_bench's calibrated hardware-like measurement)
        Xo = hist_X[-1 - n_odo]; Xa = hist_X[-1 - n_att]
        Ra_true = Xa[3:12].reshape(3, 3, order="F")
        dv_ = rng.standard_normal(3) * fb["att_noise"]; nv = np.linalg.norm(dv_)
        Ra = Ra_true @ C.joint_rotation(dv_ / (nv + 1e-12), nv) if fb["att_noise"] > 0 else Ra_true
        Ro = Xo[3:12].reshape(3, 3, order="F")
        v_world = Ro @ Xo[12 + N:15 + N] + (vnoise() if fb["vel_noise"] > 0 else 0.0)
        Xm = X.copy()
        Xm[0:3] = Xo[0:3] + rng.standard_normal(3) * fb["pos_noise"]
        Xm[3:12] = Ra.flatten(order="F")
        Xm[12 + N:15 + N] = Ra.T @ v_world
        Xm[15 + N:18 + N] = X[15 + N:18 + N] + rng.standard_normal(3) * fb["gyro_noise"]
        qt = X[12:12 + N]
        Xm[12:12 + N] = np.round(qt / fb["enc_quant"]) * fb["enc_quant"] if fb["enc_quant"] > 0 else qt
        qd_obs = a_qd * qd_obs + (1 - a_qd) * X[18 + N:18 + 2 * N]
        Xm[18 + N:18 + 2 * N] = qd_obs + rng.standard_normal(N) * fb["qd_noise"]
        # ---- the phase flag (runtime, as a gripper / task signal would set it)
        law.l1.g.contact = bool(pol_fn(ev))
        dyn = C.dynamics(Xm, model)
        out = law(Xm, dyn, ref, dt)
        est = out["est"]
        u1 = float(out["u1"]); tau_body = np.asarray(out["tau_body"], float)
        tau_joint = np.clip(np.asarray(out["tau_joint"], float), -cfg.tau_max, cfg.tau_max)
        q = X[12:12 + N]; qd = X[18 + N:18 + 2 * N]
        load = np.abs(tau_joint)
        tau_joint = tau_joint + CS.FF_SCALE * (CS.FRIC_FC + CS.FRIC_MU * load) * np.tanh(Xm[18 + N:18 + 2 * N] / CS.FF_W_MEAS)
        if hw_caps:
            tau_joint = np.clip(tau_joint, -HW_CAP, HW_CAP)
        w_cmd, f_rot = allocate(u1, tau_body)
        rot_sat = bool(np.any(f_rot < 0) or np.any(np.sqrt(np.maximum(f_rot, 0) / kf_alloc) > ES.OMEGA_MAX))
        fifo.append(w_cmd); w_cmd = fifo.pop(0)
        omega_rot = w_cmd + (omega_rot - w_cmd) * np.exp(-ES.LAMBDA_ROTOR * dt)
        fr_ = kf_plant * omega_rot ** 2
        thrust_a = fr_.sum()
        tau_a = np.array([rp[:, 1] @ fr_, -(rp[:, 0] @ fr_), rdir @ (km_plant * omega_rot ** 2)])
        fr = fric_plant * (CS.FRIC_FC + CS.FRIC_MU * load) * np.tanh(qd / CS.FRIC_W)
        fr = np.clip(fr, -0.02 * np.abs(qd) / dt, 0.02 * np.abs(qd) / dt)
        over = q - ES.Q_MAX; under = ES.Q_MIN - q
        tau_stop = (-np.where(over > 0, ES.K_STOP * over + ES.C_STOP * np.maximum(qd, 0), 0.0)
                    + np.where(under > 0, ES.K_STOP * under + ES.C_STOP * np.maximum(-qd, 0), 0.0))
        # ---- the environment at the EE (true state)
        dynp = C.dynamics(X, plant)
        R = X[3:12].reshape(3, 3, order="F")
        V = X[12 + N:18 + 2 * N]
        Jv = dynp["J_y"][0:3] @ dynp["T"]              # world EE velocity from V
        r_e = X[0:3] + R @ dynp["r_0e_0"]
        v_e = Jv @ V
        a_e = (v_e - v_e_prev) / dt if k > 0 else np.zeros(3)
        v_e_prev = v_e
        f_env, f_equiv = env_step(t, r_e, v_e, a_e)
        Q = np.concatenate([fbias_m + [0.0, 0.0, thrust_a], tau_a + tbias_m, tau_joint - fr + tau_stop])
        Q = Q + Jv.T @ f_env
        xi = V.copy()
        xi = xi + np.linalg.solve(dynp["M"], Q - dynp["C"] @ xi - dynp["g"]) * dt
        v0, w0, qdn = xi[0:3], xi[3:6], xi[6:6 + N]
        x_c = X[0:3] + R @ dynp["r_0c_0"]
        H["t"][k] = t
        H3["e_ee"][k] = r_e - ref["r_ed"]; H["ee"][k] = np.linalg.norm(H3["e_ee"][k])
        H3["e_x"][k] = x_c - ref["x_cd"]; H["ex"][k] = np.linalg.norm(H3["e_x"][k])
        H["tilt"][k] = np.degrees(np.arccos(np.clip(R[2, 2], -1, 1)))
        H["chi"][k] = 0.0 if est["chi_free"] else 1.0
        H3["F_true"][k] = f_equiv; H["fn_true"][k] = np.linalg.norm(f_equiv)
        H3["F_hat_f"][k] = est["F_f"][0:3]; H3["F_raw"][k] = est["F_raw"][0:3]
        H["fhat"][k] = np.linalg.norm(est["F_f"][0:3])
        H3["d_t"][k] = out["d_t_hat"]
        H4["tau"][k] = tau_joint; H4["q"][k] = np.degrees(q); H4["wq"][k] = est["w_q_hat"]
        H["nsat"][k] = out["n_sat"]; H["rot_sat"][k] = rot_sat
        if scn == "box":
            Hbox[k] = box.moved
        X[0:3] = X[0:3] + R @ v0 * dt
        nw = np.linalg.norm(w0)
        R = R @ C.joint_rotation(w0 / (nw + 1e-12), nw * dt)
        uu, _, vt = np.linalg.svd(R); R = uu @ vt
        X[3:12] = R.flatten(order="F")
        X[12:12 + N] = X[12:12 + N] + qdn * dt
        X[12 + N:18 + N] = np.concatenate([v0, w0]); X[18 + N:18 + 2 * N] = qdn
        hist_X.append(X.copy()); hist_X.pop(0)
        # the rig's own guards: tilt 20 deg, drift 0.75 m (horizontal CoM error)
        if (not np.isfinite(H["ex"][k]) or H["tilt"][k] > 20.0
                or np.linalg.norm(H3["e_x"][k][0:2]) > 0.75):
            verdict = "ABORT"; k_end = k + 1; break
    H = {kk: v[:k_end] for kk, v in H.items()}
    H3 = {kk: v[:k_end] for kk, v in H3.items()}
    H4 = {kk: v[:k_end] for kk, v in H4.items()}
    kw = dict(kw, rd=rd, hw=hw_caps, base_ff=law.base_ff, **({"gains": gains} if gains else {}))
    res = dict(scn=scn, chi=chi, kw={k: v for k, v in kw.items() if not isinstance(v, np.ndarray)},
               verdict=verdict, t_end=float(H["t"][-1]), marks=marks)
    res.update(score(scn, H, H3, H4, marks, t_settle, cfg, law, kw, Hbox[:k_end] if scn == "box" else None))
    if keep:
        res["H"] = H; res["H3"] = H3; res["H4"] = H4
        if scn == "box":
            res["box"] = Hbox[:k_end]
    return res


def score(scn, H, H3, H4, marks, t_settle, cfg, law, kw, box):
    t = H["t"]
    act = t >= t_settle - 0.01
    out = {}
    if not act.any():
        return out
    tau = H4["tau"][act]
    out["tau_pk"] = np.abs(tau).max(axis=0).round(3).tolist()
    out["tau_util"] = float((np.abs(tau) / HW_CAP).max())       # 1.0 = at a hardware cap
    out["cap_pct"] = float(100 * np.mean(np.any(np.abs(tau) >= HW_CAP - 1e-6, axis=1)))
    out["tilt_pk"] = float(H["tilt"][act].max())
    out["ex_pk"] = float(H["ex"][act].max())
    out["ee_pk"] = float(H["ee"][act].max())
    out["rot_sat_pct"] = float(100 * np.mean(H["rot_sat"][act]))
    # hover noise floor of the collision reading, before any interaction
    pre = (t > t_settle - 5.0) & (t < t_settle)
    out["fhat_noise"] = float(np.percentile(H["fhat"][pre], 99)) if pre.any() else float("nan")
    chi = H["chi"]
    on = np.where(np.diff(chi) > 0.5)[0]; off = np.where(np.diff(chi) < -0.5)[0]
    out["chi_on_t"] = [float(t[i + 1]) for i in on][:6]
    out["chi_off_t"] = [float(t[i + 1]) for i in off][:6]
    out["chi_end"] = int(chi[-1])
    if scn == "force":
        a, b = marks["t_on"] + 1.0 + kw.get("hold", 15.0) - 3.0, marks["t_on"] + 1.0 + kw.get("hold", 15.0)
        w = (t >= a) & (t < b)
        if w.any():
            e = H3["e_ee"][w].mean(axis=0); F = H3["F_true"][w].mean(axis=0)
            out["ee_def"] = e.round(4).tolist()
            out["F_true"] = F.round(3).tolist()
            out["F_hat"] = H3["F_hat_f"][w].mean(axis=0).round(3).tolist()
            out["ex_hold"] = H3["e_x"][w].mean(axis=0).round(4).tolist()
            Ky = cfg.K_y[0, 0]
            out["ee_def_pred_contact"] = (F / Ky).round(4).tolist()
        post = t > marks["t_off"] + 1.0 + 8.0
        out["ee_after"] = float(H["ee"][post].mean()) if post.any() else float("nan")
        out["wq_end"] = H4["wq"][-1].round(3).tolist()
    elif scn == "box":
        out["box_moved"] = float(box[-1])
        w = (t >= marks["t_push0"] + 1.0) & (t < marks["t_push1"])
        out["fc_push_mean"] = float(H["fn_true"][w].mean()) if w.any() else float("nan")
        out["fc_pk"] = float(H["fn_true"][act].max())
        out["ee_push_rms"] = float(np.sqrt(np.mean(H["ee"][w] ** 2))) if w.any() else float("nan")
        out["ex_push_pk"] = float(H["ex"][w].max()) if w.any() else float("nan")
        post = t > marks["t_back"] + 4.0
        out["ee_after"] = float(H["ee"][post].mean()) if post.any() else float("nan")
    elif scn == "payload":
        def win(a, b):
            return (t >= a) & (t < b)
        w_carry = win(marks["t_carry0"], marks["t_carry1"])
        w_place = win(marks["t_place1"] + 1.0, marks["t_place1"] + kw.get("place_hold", 8.0))
        w_after = win(marks["t_rise"] + 4.0, t[-1] + 1)
        out["ee_carry_rms"] = float(np.sqrt(np.mean(H["ee"][w_carry] ** 2))) if w_carry.any() else float("nan")
        out["ee_carry_z"] = float(H3["e_ee"][w_carry, 2].mean()) if w_carry.any() else float("nan")
        # desk-2 contact force = f_equiv + m g (remove the carried weight)
        mp = float(kw.get("m", 0.1))
        if w_place.any():
            fz = H3["F_true"][w_place, 2] + mp * G
            out["place_force"] = float(fz.mean()); out["place_force_pk"] = float(fz.max())
            out["place_force_end"] = float(fz[-50:].mean())
        wp = win(marks["t_place0"], marks["t_open"])
        out["place_force_max"] = float((H3["F_true"][wp, 2] + mp * G).max()) if wp.any() else float("nan")
        out["ee_after"] = float(H["ee"][w_after].mean()) if w_after.any() else float("nan")
        out["wq_end"] = H4["wq"][-1].round(3).tolist()
        out["tilt_carry"] = float(H["tilt"][w_carry].max()) if w_carry.any() else float("nan")
        w_lift = win(marks["t_close"], marks["t_carry0"])
        w_rel = win(marks["t_open"], marks["t_open"] + 6.0)
        out["ex_lift_pk"] = float(H["ex"][w_lift].max()) if w_lift.any() else float("nan")
        out["ex_release_pk"] = float(H["ex"][w_rel].max()) if w_rel.any() else float("nan")
        out["ex_place_pk"] = float(H["ex"][wp].max()) if wp.any() else float("nan")
    return out


def fmt(r):
    s = f"{r['scn']:7} {r['chi']:22} {json.dumps(r['kw'])[:46]:46} "
    if r["verdict"] != "completed":
        s += f"ABORT t={r['t_end']:.1f}  "
    else:
        s += "ok            "
    s += (f"tilt {r.get('tilt_pk', np.nan):4.1f} ex {r.get('ex_pk', np.nan)*1e3:5.0f} ee {r.get('ee_pk', np.nan)*1e3:5.0f}mm "
          f"util {r.get('tau_util', np.nan):4.2f} cap {r.get('cap_pct', np.nan):4.1f}% chi_on {r.get('chi_on_t')} off {r.get('chi_off_t')}")
    for k in ("ee_def", "F_hat", "ee_def_pred_contact", "ee_after", "box_moved", "fc_push_mean", "fc_pk",
              "place_force", "place_force_max", "place_force_end", "ee_carry_rms", "ee_carry_z", "fhat_noise"):
        if k in r:
            v = r[k]
            s += f" {k}={np.round(v, 4) if not isinstance(v, list) else v}"
    return s


def _job(j):
    t0 = time.time()
    scn, chi, kw = j
    try:
        r = simulate(scn, chi, **kw)
    except Exception as e:   # noqa: BLE001
        r = dict(scn=scn, chi=chi, kw=kw, verdict=f"ERROR {e!r}", t_end=0.0)
    r["wall_s"] = time.time() - t0
    return r


def run_jobs(jobs, out_path=None, procs=None):
    import multiprocessing as mp
    res = []
    with mp.get_context("fork").Pool(procs or min(len(jobs), 30)) as pool:
        for r in pool.imap(_job, jobs):
            print(fmt(r) if r["verdict"] in ("completed", "ABORT") else f"{r['scn']} {r['chi']} {r['kw']} {r['verdict']}",
                  flush=True)
            res.append(r)
    if out_path:
        def clean(o):
            if isinstance(o, dict):
                return {k: clean(v) for k, v in o.items() if k not in ("H", "H3", "H4", "box")}
            if isinstance(o, (np.floating,)):
                return float(o)
            if isinstance(o, np.ndarray):
                return o.tolist()
            return o
        json.dump([clean(r) for r in res], open(out_path, "w"), indent=1)
    return res


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--smoke", action="store_true")
    a = ap.parse_args()
    np.set_printoptions(precision=3, suppress=True)
    if a.smoke:
        t0 = time.time()
        r = simulate("force", "free", F=2.0, dir="radial", hold=6.0, t_settle=8.0)
        print(fmt(r), f"[{time.time()-t0:.0f} s]")
