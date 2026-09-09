#!/usr/bin/env python3
"""
wb_entry_sim.py — the DIRECT-entry transient of the whole-body law, offline.

WHY (2026-09-09). The rig needs a joint-posture PID on u3 that the published
law does not have (Command.md 7.14.6 / 7.15.7). Reading the flown traces
(l1_final_20260906/l1_nopid_A.npz, l1_seed_20260909/l1_seedA.npz) shows the
SAME mechanism every time: at the SAFETY->DIRECT switch the observers restart
at zero, the plant's unlearned disturbance (thrust deficit + CoM moment) drives
a 0.8-1.3 m base excursion, and the WORLD-FIXED end-effector reference is then
far outside the arm's ~5 cm workspace at the folded home. The EE error tracks
the base error one-to-one, the impedance law drives the arm into its joint
stops (q3 hits +50 deg at t = 1 s), and from the stops it crosses into the
elbow-singular branch where the DLS solve rails the servos and the reaction
takes the base. With the posture PID the arm simply rides the base and the EE
task is silently unsatisfied through the same 0.8 m transient.

This script closes exactly that loop offline (exact law, exact model, the L1
observer, the plant injections the launcher applies, joint stops, rotor lag,
transport delay) so the three questions of the day can be answered in seconds
per case rather than 5 minutes and a crash per flight:
  1. WHICH disturbance triggers it (ablation);
  2. whether any GAIN set of the published law survives without the PID;
  3. whether the candidate LAW modifications (a base-relative EE reference in
     free flight; a joint-space potential) survive, before flying them.

It is a screening tool: the plant is the model with injections, not PhysX.
Trust the ORDERING and the mechanism, fly the winner.

Usage:
    python3 wb_entry_sim.py --validate            # reproduce PID off/on at the full injection
    python3 wb_entry_sim.py --ablate              # Q1: one disturbance at a time
    python3 wb_entry_sim.py --sweep               # Q2: law gains, PID off
    python3 wb_entry_sim.py --fixes               # Q3: candidate modifications
    python3 wb_entry_sim.py --case posture=0,ee_ref=relative,kf=0.85,mass=1.1
"""

import argparse
import copy
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.abspath(os.path.join(_HERE, "..", "..", ".."))
sys.path.insert(0, os.path.join(_REPO, "extensions", "fsc_aerial_manipulation"))

from fsc_aerial_manipulation.robotic_arm.utils_controller import (  # noqa: E402
    control_params as CP,
    controller as C,
    l1_observer as L1,
)
from fsc_aerial_manipulation.robotic_arm.utils_planner import (  # noqa: E402
    transition_planner as TP,
)

hat, vee, E3 = C.hat, C.vee, C.E3

# ---------------------------------------------------------------- plant truth
KF_TRUE = 4.041283e-05        # t650_params.ROTOR_CONSTANT (2026-08-25 re-anchor)
KM_TRUE = 2.474152e-06        # t650_params.ROLLING_MOMENT_COEFFICIENT
LAMBDA_ROTOR = 10.0265        # 1/s, MN4010 spin-up bandwidth
OMEGA_MAX = 730.0507
HOME = np.array([0.0, np.deg2rad(40.0), np.deg2rad(40.0), 0.0])
# rotor geometry in the MODEL frame (the law's RotorMixer, AM_xfwd channels)
ROTOR_POS = np.array([[0.229907, 0.229907, 0.182191],
                      [-0.229907, -0.229907, 0.182191],
                      [0.229907, -0.229907, 0.182191],
                      [-0.229907, 0.229907, 0.182191]])
ROT_DIR = np.array([-1.0, -1.0, 1.0, 1.0])
# joint stops: the OM-X working range the planner/C++/arm config all carry
Q_MIN, Q_MAX = TP.Q_MIN, TP.Q_MAX
K_STOP, C_STOP = 300.0, 3.0   # stop stiffness [N.m/rad] / damping


# =============================================================== configuration
def make_gains(**over):
    """The whole-body L1 sim yaml's LAW gains (2026-09-09) as ControlParams."""
    v = dict(k_x=16.0, k_v=12.0, k_R=2.0, k_w=1.5,
             mrd=(0.116522, 0.136107, 0.125102),
             ky=(2.0, 2.0, 2.0, 0.3), dy=(4.0, 4.0, 4.0, 0.3),
             my=(1.0, 1.0, 1.0, 0.05), ko=(0.5, 0.1, 0.1),
             use_gmo=True, dls_lambda=0.3, tau_max=3.0)
    for k, val in over.items():
        if k not in v:
            raise KeyError(f"unknown law gain '{k}' (have {sorted(v)})")
        v[k] = val
    ky = np.atleast_1d(np.asarray(v["ky"], float))
    dy = np.atleast_1d(np.asarray(v["dy"], float))
    my = np.atleast_1d(np.asarray(v["my"], float))
    if ky.size == 1:
        ky = np.array([ky[0]] * 3 + [0.3])
    if dy.size == 1:
        dy = np.array([dy[0]] * 3 + [0.3])
    if my.size == 1:
        my = np.array([my[0]] * 3 + [0.05])
    kt, kr, kq = v["ko"]
    return CP.ControlParams(
        k_x=float(v["k_x"]), k_v=float(v["k_v"]), k_R=float(v["k_R"]),
        k_w=float(v["k_w"]),
        M_r_d=np.diag(np.broadcast_to(np.atleast_1d(
            np.asarray(v["mrd"], float)), (3,)).copy()),
        K_y=np.diag(ky), D_y=np.diag(dy), use_gmo=bool(v["use_gmo"]),
        K_o=np.diag([kt] * 3 + [kr] * 3 + [kq] * 4),
        impedance_mode="shaped", M_Y=np.diag(my),
        dls_lambda=float(v["dls_lambda"]), tau_max=float(v["tau_max"]),
        source="wb_entry_sim", scenario="am_t650_whole_body_l1")


def make_l1(**over):
    """The yaml's wb_l1_* block."""
    v = dict(a_t=2.0, a_r=2.0, a_q=2.0, omega_c_t=2.0, omega_c_r=0.5,
             omega_c_q=0.5, omega_i=2.0, omega_x=20.0, adapt_period_s=0.0,
             decompose=True, lc_var_f=100.0, lc_var_m=0.25, lc_var_q=0.0025,
             max_force_n=20.0, max_torque_nm=2.0, max_joint_nm=1.5,
             max_wrench_force_n=15.0, max_wrench_torque_nm=3.0)
    for k, val in over.items():
        if k not in v:
            raise KeyError(f"unknown L1 gain '{k}'")
        v[k] = val
    return L1.L1Gains(**v)


class Case:
    """Everything one simulation varies. Defaults = the shipped rig with the
    posture PID ON and the FULL plant injection (mass/inertia x1.10, CoM
    10/10/5 mm, kf x0.85 plant-side), i.e. what flies today."""

    def __init__(self, **kw):
        self.posture = 1.0          # 0 = published law only
        self.posture_kp, self.posture_kd, self.posture_ki, self.posture_imax = \
            2.0, 0.25, 0.05, 0.8
        self.ee_ref = "world"       # "world" (paper) | "relative" (to the CoM)
        self.joint_pot = 0.0        # joint-space potential in TASK space (Q3b), N.m/rad
        self.int_ff = False         # book the ESTIMATED INTERNAL disturbance into u3's
                                    # coupling feedforward (Q3c): the base wrench the arm
                                    # pre-compensates is u + d_hat_int, not u alone
        self.int_ff_blocks = "all"      # "all": the full (J_y^#)^T d_hat; "rho": the arm
                                        # block only (the u3 channel's own disturbance);
                                        # "base": the u1/u2 blocks only
        self.int_ff_source = "lumped"   # "lumped": the filtered d_e_hat (free flight only);
                                        # "internal": the L1 note's Step-2 estimate T^-T w_hat,
                                        # which leaves a contact wrench to the impedance
        self.mass = 1.10            # plant total-mass scale
        self.inertia = 1.10         # plant body-inertia scale
        self.com = (0.010, 0.010, 0.005)   # plant body CoM shift, ACTUAL frame [m]
        self.kf = 0.85              # plant kf scale (allocator believes KF_TRUE)
        self.kf_alloc = 1.0         # allocator belief scale (1.0 = matched; 1/0.85 = legacy)
        self.delay_ms = 16.0        # DDS + PX4 HIL transport delay
        self.observer = "l1"        # "l1" | "gmo" | "none"
        self.gains = {}             # law-gain overrides
        self.l1 = {}                # L1 overrides
        self.t_end = 30.0
        self.dt = 1.0 / 250.0
        self.seed_t = False         # seed d_t from the exact deficit (UDE handover)
        self.seed_r = False         # seed d_r from the exact CoM moment
        self.z0 = 1.0
        for k, val in kw.items():
            if not hasattr(self, k):
                raise KeyError(f"unknown case field '{k}'")
            setattr(self, k, val)

    def label(self):
        return (f"PID={'on' if self.posture else 'OFF'} ee={self.ee_ref} "
                f"pot={self.joint_pot:g} mass x{self.mass:g} I x{self.inertia:g} "
                f"com {tuple(round(c * 1e3) for c in self.com)} mm kf x{self.kf:g}"
                + (f" alloc x{self.kf_alloc:g}" if self.kf_alloc != 1.0 else "")
                + f" obs={self.observer}"
                + (f" gains={self.gains}" if self.gains else "")
                + (f" l1={self.l1}" if self.l1 else "")
                + (" seed_t" if self.seed_t else "")
                + (" seed_r" if self.seed_r else ""))


def plant_params(model, case):
    """The PLANT: the model plus the launcher's injections (06's semantics —
    mass scale on the vehicle TOTAL carried by the body, inertia scale on the
    body tensor, CoM shift on the body, all in the plant only)."""
    p = copy.deepcopy(model)
    total = sum(p["m_i"])
    p["m_i"][0] = p["m_i"][0] + (case.mass - 1.0) * total
    p["I_i_i"][0] = p["I_i_i"][0] * case.inertia
    # 06 shifts the body CoM in the ACTUAL (x-forward) frame; the model frame
    # is R_MODEL rotated: v_model = R_MODEL^T v_actual.
    p["base_com"] = TP.R_MODEL.T @ np.asarray(case.com, float)
    return p


# ================================================================= allocator
def _alloc_matrix():
    B = np.zeros((4, 4))
    for i, r in enumerate(ROTOR_POS):
        B[0, i] = 1.0
        B[1, i] = r[1]
        B[2, i] = -r[0]
        B[3, i] = ROT_DIR[i] * KM_TRUE / KF_TRUE
    return B


_B = _alloc_matrix()
_B_PINV = np.linalg.pinv(_B)


def allocate(thrust, tau_body, kf_believed):
    f = np.maximum(_B_PINV @ np.array([thrust, *tau_body]), 0.0)
    return np.clip(np.sqrt(f / kf_believed), 0.0, OMEGA_MAX)


def wrench_from_rotors(omega, kf_plant):
    f = kf_plant * omega ** 2
    thrust = f.sum()
    tau = np.zeros(3)
    for i, r in enumerate(ROTOR_POS):
        tau[0] += r[1] * f[i]
        tau[1] += -r[0] * f[i]
        tau[2] += ROT_DIR[i] * KM_TRUE * omega[i] ** 2
    return thrust, tau


# ============================================================== the law
class Law:
    """controller.py's MatlabController.__call__ (the flown law, the C++
    port's source of truth) with three HOOKS the study needs and the flown
    code does not expose:
      * the L1 observer supplying d_e_hat and the attributed F_hat_y
        (wb_controller.cpp's l1_active branch);
      * the joint-posture PID exactly as wb_controller.cpp adds it to u3;
      * the EE-reference mode (Q3a) and a task-space joint potential (Q3b).
    Everything else is a verbatim copy; the copy exists so the flown files
    stay untouched."""

    def __init__(self, params, cfg, case):
        self.p, self.cfg, self.g, self.case = params, cfg, cfg, case
        self._qdot_prev = None
        self._omega0_prev = None
        self._p_hat = None
        self._i_torque = np.zeros(params["n"])
        self.l1 = L1.L1DisturbanceObserver(make_l1(**case.l1)) \
            if case.observer == "l1" else None
        self._seed = None

    def seed(self, d0):
        self._seed = np.asarray(d0, float)

    def __call__(self, X, dyn, ref, dt):
        g = self.g; p = self.p; n = p["n"]; m = sum(p["m_i"]); gg = p["g"]
        case = self.case
        r_0 = X[0:3]; R_0 = X[3:12].reshape(3, 3, order="F")
        q = X[12:12 + n]
        omega_0 = X[15 + n:18 + n]; qdot = X[18 + n:18 + 2 * n]
        v_0 = X[12 + n:15 + n]
        V = np.concatenate([v_0, omega_0, qdot])
        om_hat = hat(omega_0); e3 = E3

        A = dyn["A"]
        r_0c_0 = dyn["r_0c_0"]; r_0e_0 = dyn["r_0e_0"]; R_e_0 = dyn["R_e_0"]
        M_r = dyn["M_r"]; C_r = dyn["C_r"]; C_rp = dyn["C_rp"]; C_p = dyn["C_p"]
        J_y = dyn["J_y"]; J_y_dot = dyn["J_y_dot"]; Lambda_y = dyn["Lambda_y"]
        J_1y = dyn["J_1y"]; J_2y = dyn["J_2y"]; J_3y = dyn["J_3y"]
        omega_0e_0 = dyn["omega_0e_0"]; J_we = dyn["J_q_omega_e"]; J_we_dot = dyn["J_q_dot_omega_e"]
        T = dyn["T"]
        xi = T @ V
        xc_dot = xi[0:3]; rho = xi[6:6 + n]

        # ---- disturbance observer -------------------------------------------
        M_tilde = dyn["M_tilde"]; C_tilde = dyn["C_tilde"]; g_tilde = dyn["g_tilde"]
        pmom = M_tilde @ xi
        l1_est = None
        if case.observer == "none":
            d_e_hat = np.zeros(6 + n)
        elif case.observer == "l1":
            if self._seed is not None:
                self.l1._d_f = self._seed.copy()
                self._seed = None
            l1_est = self.l1.update(dyn, R_0, xi, dt)
            d_e_hat = np.concatenate([l1_est["d_t"], l1_est["d_r"], l1_est["d_rho"]])
        else:
            if self._p_hat is None:
                self._p_hat = pmom.copy()
                if self._seed is not None:
                    ko = np.diag(g.K_o)
                    off = np.where(ko > 0, self._seed / np.where(ko > 0, ko, 1), 0.0)
                    self._p_hat = pmom - off
                    self._seed = None
            d_e_hat = g.K_o @ (pmom - self._p_hat)
        d_t_hat = d_e_hat[0:3]; d_r_hat = d_e_hat[3:6]; d_rho_hat = d_e_hat[6:6 + n]

        # ---- translation ----
        x_c = r_0 + R_0 @ r_0c_0
        e_x = x_c - ref["x_cd"]
        e_vx = xc_dot - ref["x_cd_dot"]
        f_d = -g.k_x * e_x - g.k_v * e_vx + m * ref["x_cd_ddot"] + m * gg * e3 - d_t_hat
        u1 = float(f_d.T @ (R_0 @ e3))
        xc_ddot = (u1 / m) * (R_0 @ e3) - gg * e3 + (1.0 / m) * d_t_hat
        e_ax = xc_ddot - ref["x_cd_ddot"]
        f_d_dot = -g.k_x * e_vx - g.k_v * e_ax + m * ref["x_cd_d3"]
        u1_dot = float(f_d_dot.T @ (R_0 @ e3) + f_d.T @ (R_0 @ om_hat @ e3))
        xc_d3 = (u1_dot / m) * (R_0 @ e3) + (u1 / m) * (R_0 @ om_hat @ e3)
        e_jx = xc_d3 - ref["x_cd_d3"]
        f_d_ddot = -g.k_x * e_ax - g.k_v * e_jx + m * ref["x_cd_d4"]

        # ---- rotation ----
        nf = np.linalg.norm(f_d)
        b3c = f_d / nf
        P1 = np.eye(3) - np.outer(b3c, b3c)
        s = P1 @ ref["b1_d"]
        b3c_dot = (1.0 / nf) * P1 @ f_d_dot
        ns = np.linalg.norm(s)
        b1c = s / ns
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
        b3c_ddot = ((1.0 / nf) * (P1_dot @ f_d_dot + P1 @ f_d_ddot)
                    - (b3c.T @ f_d_dot) / nf**2 * (P1 @ f_d_dot))
        P1_ddot = -(np.outer(b3c_ddot, b3c) + 2 * np.outer(b3c_dot, b3c_dot) + np.outer(b3c, b3c_ddot))
        s_ddot = P1_ddot @ ref["b1_d"] + 2 * P1_dot @ ref["b1_d_dot"] + P1 @ ref["b1_d_ddot"]
        b1c_ddot = (1.0 / ns) * (P2_dot @ s_dot + P2 @ s_ddot) - (b1c.T @ s_dot) / ns * b1c_dot
        b2c_ddot = (np.cross(b3c_ddot, b1c) + 2 * np.cross(b3c_dot, b1c_dot) + np.cross(b3c, b1c_ddot))
        R0c_ddot = np.column_stack([b1c_ddot, b2c_ddot, b3c_ddot])
        omega_0c_dot = vee(R0c_dot.T @ R0c_dot + R0c.T @ R0c_ddot)
        e_R = 0.5 * vee(R0c.T @ R_0 - R_0.T @ R0c)
        e_w = omega_0 - R_0.T @ R0c @ omega_0c
        u2 = (M_r @ (R_0.T @ R0c @ omega_0c_dot - om_hat @ R_0.T @ R0c @ omega_0c
                     - np.linalg.solve(g.M_r_d, g.k_R * e_R + g.k_w * e_w))
              + C_r @ omega_0 + C_rp @ rho - d_r_hat)

        # ---- arm ----
        R_e = R_0 @ R_e_0
        R_0e = R_e_0.T
        r_e = r_0 + R_0 @ r_0e_0
        omega_e = R_0e @ (omega_0 + omega_0e_0)
        omega_e_hat = hat(omega_e)
        qddot = np.zeros(n); omega_0_dot = np.zeros(3)
        self._qdot_prev = qdot.copy(); self._omega0_prev = omega_0.copy()
        omega_e_dot = R_0e @ (omega_0_dot + J_we @ qddot + J_we_dot @ qdot + om_hat @ omega_0e_0)
        omega_e_dot_hat = hat(omega_e_dot)
        b2e = R_e @ np.array([0.0, 1, 0]); b3e = R_e @ e3
        ydot = J_y @ xi
        r_e_dot = ydot[0:3]
        omega_3e = float(omega_e.T @ e3)
        P1e = np.eye(3) - np.outer(b3e, b3e)
        s_e = P1e @ ref["b1_de"]
        b3e_dot = R_e @ omega_e_hat @ e3
        nse = np.linalg.norm(s_e)
        b1ec = s_e / nse
        P1e_dot = -(np.outer(b3e_dot, b3e) + np.outer(b3e, b3e_dot))
        P2e = np.eye(3) - np.outer(b1ec, b1ec)
        s_e_dot = P1e_dot @ ref["b1_de"] + P1e @ ref["b1_de_dot"]
        b1ec_dot = (1.0 / nse) * P2e @ s_e_dot
        P2e_dot = -(np.outer(b1ec_dot, b1ec) + np.outer(b1ec, b1ec_dot))
        b3e_ddot = R_e @ (omega_e_hat @ omega_e_hat + omega_e_dot_hat) @ e3
        P1e_ddot = -(np.outer(b3e_ddot, b3e) + 2 * np.outer(b3e_dot, b3e_dot) + np.outer(b3e, b3e_ddot))
        s_e_ddot = P1e_ddot @ ref["b1_de"] + 2 * P1e_dot @ ref["b1_de_dot"] + P1e @ ref["b1_de_ddot"]
        b1ec_ddot = (1.0 / nse) * (P2e_dot @ s_e_dot + P2e @ s_e_ddot) - (b1ec.T @ s_e_dot) / nse * b1ec_dot
        omega_3ec = float(b3e.T @ np.cross(b1ec, b1ec_dot))
        omega_3ec_dot = float(b3e_dot.T @ np.cross(b1ec, b1ec_dot)
                              + np.cross(b3e, b1ec).T @ b1ec_ddot)

        # -------- HOOK Q3a: the EE reference in free flight ---------------
        # "world": the paper's r_ed as streamed (world-fixed).
        # "relative": r_ed + e_x, i.e. the desired EE OFFSET from the CoM is
        # what is held; the reference moves with the base error and its
        # derivatives carry the base error's derivatives (e_vx, e_ax are the
        # law's own translational-dynamics estimates -- no differentiation).
        if case.ee_ref == "relative":
            r_ed = ref["r_ed"] + e_x
            r_ed_dot = ref["r_ed_dot"] + e_vx
            r_ed_ddot = ref["r_ed_ddot"] + e_ax
        else:
            r_ed, r_ed_dot, r_ed_ddot = ref["r_ed"], ref["r_ed_dot"], ref["r_ed_ddot"]

        e_xE = r_e - r_ed
        e_RE3 = float(-(b1ec.T @ b2e))
        e_y = np.array([e_xE[0], e_xE[1], e_xE[2], e_RE3])
        e_vE = r_e_dot - r_ed_dot
        e_wE3 = omega_3e - omega_3ec
        e_vy = np.array([e_vE[0], e_vE[1], e_vE[2], e_wE3])
        yddot_d = np.array([r_ed_ddot[0], r_ed_ddot[1], r_ed_ddot[2], omega_3ec_dot])

        M_y = Lambda_y if self.cfg.M_Y is None else self.cfg.M_Y
        Lam_Myinv = Lambda_y @ np.linalg.solve(M_y, np.eye(4))
        if l1_est is not None:
            F_hat_y = l1_est["F_y"]
        else:
            F_hat_y = J_1y @ d_t_hat + J_2y @ d_r_hat + J_3y @ d_rho_hat

        F_trans = u1 * (R_0 @ e3) - m * gg * e3
        tau_rot = u2 - C_r @ omega_0 - C_rp @ rho
        coupling_ff = J_1y @ F_trans + J_2y @ tau_rot
        # -------- HOOK Q3c: internal-disturbance-consistent coupling --------
        # The paper's d_e is the CONTACT wrench only; an internal disturbance
        # (thrust deficit, CoM moment) reaches the EE loop as the phantom
        # F_y^int = (J_y^#)^T d_int, and the impedance renders compliance
        # against it (M_y Lam^-1 scaled). Once the observer knows d_int, the
        # net base wrench is u + d_int, so the coupling feedforward must carry
        # it: inner += (J_y^#)^T d_hat_int. In free flight the whole estimate
        # is internal, so the lumped filtered d_e_hat is used here.
        if case.int_ff:
            if case.int_ff_source == "internal" and l1_est is not None:
                d_int = np.linalg.solve(T.T, l1_est["w_hat"])     # T^-T w_hat
            else:
                d_int = d_e_hat
            blk = case.int_ff_blocks
            add = np.zeros(4)
            if blk in ("all", "base"):
                add = add + J_1y @ d_int[0:3] + J_2y @ d_int[3:6]
            if blk in ("all", "rho"):
                add = add + J_3y @ d_int[6:]
            coupling_ff = coupling_ff + add
        # -------- HOOK Q3b: a joint-space potential expressed IN THE TASK --
        # K_q (q - q_d) mapped through J_3y^{-T}... kept simple: add
        # J_3y (K_q e_q) to the task-space restoring force so the SAME DLS
        # solve renders it — the law's structure is unchanged, only K_y e_y is
        # replaced by K_y e_y + J_3y K_q e_q (a configuration-dependent
        # stiffness, positive semidefinite).
        pot = np.zeros(4)
        if case.joint_pot > 0.0:
            pot = Lam_Myinv @ (J_3y @ (case.joint_pot * (q - ref["q_d"])))
        inner = (coupling_ff
                 + Lambda_y @ (J_y_dot @ xi - yddot_d)
                 + Lam_Myinv @ (g.D_y @ e_vy + g.K_y @ e_y) + pot
                 - (Lam_Myinv - np.eye(4)) @ F_hat_y)
        _JJt = J_3y @ J_3y.T + self.cfg.dls_lambda**2 * np.eye(4)
        _sol = lambda v: J_3y.T @ np.linalg.solve(_JJt, v)   # noqa: E731
        u3 = -_sol(inner) - C_rp.T @ omega_0 + C_p @ rho

        # -------- the joint-posture PID, exactly as wb_controller.cpp -------
        if case.posture > 0.0:
            if case.posture_ki > 0.0 and case.posture_imax > 0.0:
                self._i_torque += case.posture_ki * (ref["q_d"] - q) * dt
                self._i_torque = np.clip(self._i_torque, -case.posture_imax, case.posture_imax)
            u3 = u3 + case.posture * (case.posture_kp * (ref["q_d"] - q)
                                      + case.posture_kd * (ref["qdot_d"] - qdot)) + self._i_torque

        u = np.concatenate([u1 * (R_0 @ e3), u2, u3])
        tau = T.T @ u
        tau_joint = tau[6:]
        n_sat = 0
        if self.cfg.tau_max is not None:
            _tm = self.cfg.tau_max
            tau_clamped = np.clip(tau_joint, -_tm, _tm)
            n_sat = int(np.count_nonzero(tau_clamped != tau_joint))
            if n_sat:
                tau_joint = tau_clamped
                u3 = tau_joint - u1 * (A.T @ e3)
                u = np.concatenate([u1 * (R_0 @ e3), u2, u3])
                tau = T.T @ u

        # ---- observer propagation ----
        if case.observer == "l1":
            self.l1.propagate(dyn, xi, u, dt)
        elif case.observer == "gmo":
            p_hat_dot = C_tilde.T @ xi - g_tilde + u + d_e_hat
            self._p_hat = self._p_hat + p_hat_dot * dt

        return {"u1": u1, "tau_body": tau[3:6], "tau_joint": tau_joint,
                "e_x": e_x, "e_y": e_y, "e_R": e_R, "n_sat": n_sat,
                "d_t_hat": d_t_hat, "d_r_hat": d_r_hat, "F_hat_y": F_hat_y,
                "u3": u3}


# ================================================================ simulation
def simulate(case, verbose=False, trace=False):
    model = TP.make_params_t650()
    plant = plant_params(model, case)
    n = model["n"]
    cfg = make_gains(**case.gains)
    law = Law(model, cfg, case)
    dt = case.dt
    kf_plant = KF_TRUE * case.kf
    kf_alloc = KF_TRUE * case.kf_alloc

    # ---- initial state: level hover, arm at home, at rest -----------------
    X = np.zeros(12 + 3 * n + 6)
    X[0:3] = [0.0, 0.0, case.z0]
    X[3:12] = np.eye(3).flatten(order="F")
    X[12:12 + n] = HOME

    # ---- the rest hold the planner captures at DIRECT entry, on the MODEL ---
    dyn0 = law_dyn0 = C.dynamics(X, model)
    x_cd = X[0:3] + dyn0["r_0c_0"]
    r_ed = X[0:3] + dyn0["r_0e_0"]
    b1_de = dyn0["R_e_0"][:, 0]
    z3 = np.zeros(3)
    ref = {"x_cd": x_cd, "x_cd_dot": z3, "x_cd_ddot": z3, "x_cd_d3": z3, "x_cd_d4": z3,
           "b1_d": np.array([1.0, 0.0, 0.0]), "b1_d_dot": z3, "b1_d_ddot": z3,
           "r_ed": r_ed, "r_ed_dot": z3, "r_ed_ddot": z3,
           "b1_de": b1_de, "b1_de_dot": z3, "b1_de_ddot": z3,
           "q_d": HOME.copy(), "qdot_d": np.zeros(n)}

    # ---- the SAFETY equilibrium the plant is handed over from --------------
    # PX4/SAFETY hold the PLANT level at rest: true weight and the true CoM
    # moment already compensated by its integrators. The DIRECT law starts
    # from ZERO estimates -- that IS the handover.
    dynp = C.dynamics(X, plant)
    gp = dynp["g"]                          # [force; moment; joint] at rest
    thrust_true = gp[2]
    tau_true = gp[3:6]                      # the base moment gravity needs
    w_true = allocate(thrust_true, tau_true, kf_plant)
    omega_rot = w_true.copy()
    if case.seed_t:
        law.seed(np.concatenate([[0, 0, thrust_true - sum(model["m_i"]) * model["g"]] , np.zeros(7)]))
    if case.seed_r:
        d0 = np.zeros(10)
        d0[2] = (thrust_true - sum(model["m_i"]) * model["g"]) if case.seed_t else 0.0
        d0[3:6] = tau_true - law_dyn0["g"][3:6]
        law.seed(d0)

    n_del = int(round(case.delay_ms * 1e-3 / dt))
    cmd_fifo = [allocate(thrust_true, tau_true, kf_alloc) for _ in range(max(n_del, 0))]
    steps = int(case.t_end / dt)
    hist = {k: [] for k in ("t", "ex", "ey", "q", "tau", "tilt", "nsat", "dtz", "eR")}
    verdict = "completed"
    t_fail = None

    for k in range(steps):
        t = k * dt
        dyn = C.dynamics(X, model)          # what the LAW sees (its own model)
        out = law(X, dyn, ref, dt)
        u1 = float(out["u1"])
        tau_body = np.asarray(out["tau_body"], float)
        tau_joint = np.clip(np.asarray(out["tau_joint"], float), -cfg.tau_max, cfg.tau_max)

        w_cmd = allocate(u1, tau_body, kf_alloc)
        if n_del > 0:
            cmd_fifo.append(w_cmd)
            w_cmd = cmd_fifo.pop(0)
        omega_rot = w_cmd + (omega_rot - w_cmd) * np.exp(-LAMBDA_ROTOR * dt)
        thrust_a, tau_a = wrench_from_rotors(omega_rot, kf_plant)

        # joint stops (plant)
        q = X[12:12 + n]; qd = X[18 + n:18 + 2 * n]
        tau_stop = np.zeros(n)
        over = q - Q_MAX; under = Q_MIN - q
        tau_stop -= np.where(over > 0, K_STOP * over + C_STOP * np.maximum(qd, 0), 0.0)
        tau_stop += np.where(under > 0, K_STOP * under + C_STOP * np.maximum(-qd, 0), 0.0)

        dynp = C.dynamics(X, plant)
        Q = np.concatenate([[0.0, 0.0, thrust_a], tau_a, tau_joint + tau_stop])
        xi = np.concatenate([X[12 + n:15 + n], X[15 + n:18 + n], X[18 + n:18 + 2 * n]])
        xi_dot = np.linalg.solve(dynp["M"], Q - dynp["C"] @ xi - dynp["g"])
        xi = xi + xi_dot * dt
        v0, w0, qd = xi[0:3], xi[3:6], xi[6:6 + n]
        R = X[3:12].reshape(3, 3, order="F")
        X[0:3] = X[0:3] + R @ v0 * dt
        nw = np.linalg.norm(w0)
        R = R @ C.joint_rotation(w0 / (nw + 1e-12), nw * dt)
        uu, _, vt = np.linalg.svd(R)
        R = uu @ vt
        X[3:12] = R.flatten(order="F")
        X[12:12 + n] = X[12:12 + n] + qd * dt
        X[12 + n:18 + n] = np.concatenate([v0, w0])
        X[18 + n:18 + 2 * n] = qd

        tilt = np.degrees(np.arccos(np.clip(R[2, 2], -1, 1)))
        ex = float(np.linalg.norm(out["e_x"]))
        hist["t"].append(t); hist["ex"].append(ex)
        hist["ey"].append(float(np.linalg.norm(out["e_y"][:3])))
        hist["q"].append(np.degrees(X[12:12 + n]).copy())
        hist["tau"].append(tau_joint.copy()); hist["tilt"].append(tilt)
        hist["nsat"].append(out["n_sat"]); hist["dtz"].append(float(out["d_t_hat"][2]))
        hist["eR"].append(float(np.linalg.norm(out["e_R"])))
        if trace and (k % 125 == 0):
            print(f"  t={t:5.2f} |e_x|={ex*1e3:6.0f} mm |e_y|={hist['ey'][-1]*1e3:6.0f} mm "
                  f"q={np.round(hist['q'][-1]).astype(int)} tau={np.round(tau_joint,2)} "
                  f"nsat={out['n_sat']} tilt={tilt:5.1f} u1={u1:5.1f} dtz={hist['dtz'][-1]:6.2f}")
        # the driver's abort envelope
        if not np.isfinite(ex) or tilt > 35.0 or ex > 1.5 or abs(X[2] - case.z0) > 0.8:
            verdict = "ABORT"; t_fail = t
            break

    for k in hist:
        hist[k] = np.asarray(hist[k])
    tau = hist["tau"]
    q = hist["q"]
    res = dict(
        verdict=verdict, t_fail=t_fail,
        ex_max=float(hist["ex"].max()), ey_max=float(hist["ey"].max()),
        ex_last=float(hist["ex"][-int(2 / dt):].mean()) if verdict == "completed" else np.nan,
        ey_last=float(hist["ey"][-int(2 / dt):].mean()) if verdict == "completed" else np.nan,
        tilt_max=float(hist["tilt"].max()),
        tau_max=float(np.abs(tau).max()),
        clamp_pct=float(100.0 * np.mean(hist["nsat"] > 0)),
        q_min=q.min(axis=0), q_max=q.max(axis=0),
        q_final=q[-1], hist=hist)
    return res


def fmt(res):
    if res["verdict"] == "ABORT":
        head = f"ABORT t={res['t_fail']:5.1f}s"
    else:
        head = f"ok   settled |e_x| {res['ex_last']*1e3:5.1f} mm |e_y| {res['ey_last']*1e3:5.1f} mm"
    return (f"{head:44s} peak e_x {res['ex_max']*1e3:6.0f} mm e_y {res['ey_max']*1e3:6.0f} mm "
            f"tilt {res['tilt_max']:5.1f} deg tau {res['tau_max']:4.2f} clamp {res['clamp_pct']:4.1f}% "
            f"q3 [{res['q_min'][2]:5.0f},{res['q_max'][2]:4.0f}] q_end {np.round(res['q_final']).astype(int)}")


def run(case, tag="", trace=False):
    r = simulate(case, trace=trace)
    print(f"{tag:26s} {fmt(r)}")
    return r


# ================================================================== studies
def validate():
    print("=== VALIDATE: the flown configuration (full injection), PID on vs off ===")
    run(Case(posture=1.0), "PID on (flies)")
    run(Case(posture=0.0), "PID off (aborts ~8 s)", trace=True)
    print("--- legacy allocator-side injection (what 09-06 flew: kf plant 1.0, alloc x1/0.85)")
    run(Case(posture=1.0, kf=1.0, kf_alloc=1 / 0.85), "PID on, alloc-side")
    run(Case(posture=0.0, kf=1.0, kf_alloc=1 / 0.85), "PID off, alloc-side")


def ablate():
    print("=== Q1 ABLATION: published law (PID off), one disturbance at a time ===")
    base = dict(posture=0.0, mass=1.0, inertia=1.0, com=(0, 0, 0), kf=1.0)
    run(Case(**base), "L0 motor delay only")
    run(Case(**{**base, "delay_ms": 0.0}), "L0 no transport delay")
    run(Case(**{**base, "mass": 1.05, "inertia": 1.05}), "+5% mass/inertia")
    run(Case(**{**base, "com": (0.005, 0.005, 0.0025)}), "+CoM 5/5/2.5 mm")
    run(Case(**{**base, "mass": 1.05, "inertia": 1.05, "com": (0.005, 0.005, 0.0025)}), "+5% model (all)")
    run(Case(**{**base, "kf": 0.95}), "+5% thrust loss")
    run(Case(**{**base, "kf": 0.90}), "+10% thrust loss")
    run(Case(**{**base, "kf": 0.85}), "+15% thrust loss")
    run(Case(**{**base, "mass": 1.05, "inertia": 1.05, "com": (0.005, 0.005, 0.0025), "kf": 0.95}), "+5% model +5% thrust")
    run(Case(**{**base, "mass": 1.10, "inertia": 1.10, "com": (0.010, 0.010, 0.005), "kf": 0.85}), "FULL (10%/15%)")
    print("--- the same with the PID ON, for the base transient reference")
    run(Case(posture=1.0, mass=1.0, inertia=1.0, com=(0, 0, 0), kf=1.0), "PID on, delay only")
    run(Case(posture=1.0), "PID on, FULL")
    print("--- CoM shift alone, larger")
    run(Case(**{**base, "com": (0.010, 0.010, 0.005)}), "+CoM 10/10/5 mm")
    run(Case(**{**base, "mass": 1.10, "inertia": 1.10}), "+10% mass/inertia")


def sweep():
    print("=== Q2 GAIN SWEEP: PID off, FULL injection, one gain group at a time ===")
    base = dict(posture=0.0)
    run(Case(**base), "shipped")
    for kx, kv in ((8, 8), (24, 15), (32, 20), (48, 28)):
        run(Case(**base, gains=dict(k_x=kx, k_v=kv)), f"k_x={kx} k_v={kv}")
    for kR, kw in ((1.0, 1.0), (3.0, 2.0), (4.0, 2.5), (6.0, 3.5)):
        run(Case(**base, gains=dict(k_R=kR, k_w=kw)), f"k_R={kR} k_w={kw}")
    for ky, dy in ((0.5, 2.0), (8.0, 8.0), (20.0, 9.0), (50.0, 15.0), (200.0, 24.0)):
        run(Case(**base, gains=dict(ky=ky, dy=dy)), f"K_y={ky} D_y={dy}")
    for my in (0.25, 0.5, 2.0, 4.0):
        run(Case(**base, gains=dict(my=my)), f"M_y={my}")
    for dls in (0.0, 0.6, 1.0, 1.5):
        run(Case(**base, gains=dict(dls_lambda=dls)), f"dls={dls}")
    for wt, wr, wq in ((1.0, 0.25, 0.25), (4.0, 0.5, 0.5), (2.0, 1.5, 0.5), (2.0, 0.5, 2.0), (6.0, 2.0, 1.0)):
        run(Case(**base, l1=dict(omega_c_t=wt, omega_c_r=wr, omega_c_q=wq)), f"omega_c={wt}/{wr}/{wq}")
    print("--- handover seeds (estimator initial condition, not a control term)")
    run(Case(**base, seed_t=True), "seed d_t (UDE)")
    run(Case(**base, seed_t=True, seed_r=True), "seed d_t + d_r")


def fixes():
    print("=== Q3 CANDIDATE MODIFICATIONS, PID off, FULL injection ===")
    run(Case(posture=0.0), "A published law")
    run(Case(posture=0.0, ee_ref="relative"), "B EE ref relative to CoM")
    run(Case(posture=0.0, ee_ref="relative", gains=dict(ky=8.0, dy=8.0)), "B + K_y 8/D_y 8")
    run(Case(posture=0.0, ee_ref="relative", gains=dict(ky=20.0, dy=12.0)), "B + K_y 20/D_y 12")
    for kq in (0.5, 1.0, 2.0, 4.0):
        run(Case(posture=0.0, joint_pot=kq), f"C task-space joint pot {kq}")
    run(Case(posture=1.0, posture_ki=0.0, posture_imax=0.0), "D PD only (no I)")
    run(Case(posture=1.0, posture_kd=0.0, posture_ki=0.0, posture_imax=0.0), "D P only")
    print("--- ablation of the fix B")
    for kf, mass, com in ((1.0, 1.0, (0, 0, 0)), (0.85, 1.0, (0, 0, 0)), (1.0, 1.1, (0.01, 0.01, 0.005))):
        run(Case(posture=0.0, ee_ref="relative", kf=kf, mass=mass, inertia=mass, com=com),
            f"B kf {kf} mass {mass} com {com}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--validate", action="store_true")
    ap.add_argument("--ablate", action="store_true")
    ap.add_argument("--sweep", action="store_true")
    ap.add_argument("--fixes", action="store_true")
    ap.add_argument("--case", default=None, help="comma list of Case fields, e.g. posture=0,kf=0.9")
    ap.add_argument("--trace", action="store_true")
    a = ap.parse_args()
    np.set_printoptions(precision=3, suppress=True, linewidth=160)
    if a.validate:
        validate()
    if a.ablate:
        ablate()
    if a.sweep:
        sweep()
    if a.fixes:
        fixes()
    if a.case:
        kw = {}
        for kv in a.case.split(","):
            k, v = kv.split("=")
            k = k.strip(); v = v.strip()
            if k in ("ee_ref", "observer"):
                kw[k] = v
            elif k == "com":
                kw[k] = tuple(float(x) for x in v.split("/"))
            elif k.startswith("g."):
                kw.setdefault("gains", {})[k[2:]] = float(v)
            elif k.startswith("l1."):
                kw.setdefault("l1", {})[k[3:]] = float(v)
            else:
                kw[k] = float(v)
        run(Case(**kw), "case", trace=a.trace)
    if not (a.validate or a.ablate or a.sweep or a.fixes or a.case):
        validate()


if __name__ == "__main__":
    main()
