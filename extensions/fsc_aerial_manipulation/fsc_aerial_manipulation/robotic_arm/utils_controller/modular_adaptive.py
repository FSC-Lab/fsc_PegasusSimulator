#!/usr/bin/env python3
"""modular_adaptive.py -- the MODULAR ADAPTIVE aerial-manipulation law of

    R. D. Yadav, S. Dantu, W. Pan, S. Sun, S. Roy, S. Baldi,
    "Modular Adaptive Aerial Manipulation Under Unknown Dynamic Coupling
    Forces", IEEE/ASME Trans. Mechatronics 30(4):2688-2698, 2025
    (docs/comparison references/ in fsc_PegasusSimulator; arXiv 2410.08285),

written for the SIMULATION comparison against the whole-body L1 impedance law
(2026-09-30). Pure numpy; the C++ node
`autopilot_modular_adaptive_direct_actuation_node` (fsc_autopilot_ros2,
single_aerial_manipulator_modular_adaptive_direct_actuation) is a port of
THIS file and is held to it by a parity fixture (generate_modular_truth.py).

THE LAW (paper eqs. 7-29, Algorithm 1). Three modules, one per subdynamics,
each the same template:

    xi_j = [e_j; edot_j],  r_j = B^T P_j xi_j,  A_j^T P_j + P_j A_j = -Q_j
    tau_j = Mbar_j ( -Lambda_j xi_j - dtau_j + ff_j )
    dtau_j = rho_j r_j / |r_j|        if |r_j| >= varpi_j
             rho_j r_j / varpi_j      otherwise                 (boundary layer)
    rho_j  = K0 + K1 |xi| + K2 |xi|^2 + K3 |chi_ddot| + zeta_j
    K_i'   = |r_j| |xi|^i - nu_i K_i      (i = 0, 1, 2)
    K_3'   = |r_j| |chi_ddot| - nu_3 K_3
    zeta'  = 0                                          if |r_j| >= varpi_j
             -(1 + (K3|chi_ddot| + sum K_i|xi|^i)|r_j|) zeta + eps_j   otherwise

  position  j = p : e = p - p_d (world), ff = p_ddot_d, tau_p = R U (thrust vector)
  attitude  j = q : e = e_R (geometric, eq. 14), edot = e_Omega (eq. 15),
                    ff = the desired angular acceleration, tau_q = body moment
  arm       j = a : e = alpha - alpha_d, ff = alpha_ddot_d, tau_a = joint torques

|xi| and |chi_ddot| are the WHOLE system's (paper eq. 12: "||xi|| >= ||e||" over
chi = [p; q; alpha]) -- every module's gain sees every module's error. That is
the coupling the analysis bounds; only the GAINS are modular.

WHAT THE PAPER LEAVES OPEN, AND WHAT THIS FILE CHOOSES (each one stated in the
C++ banner too, so a log says which way a run went):

 1. Lambda vs lambda. Table II lists Lambda_j AND lambda_j1, lambda_j2, but the
    error dynamics (9) ddot e = -Lambda xi - dtau + sigma and the analysis' (35)
    xi' = A xi + B(sigma - dtau) agree only if Lambda xi = lambda1 e + lambda2 edot.
    Taken here as  Lambda xi := Lambda (lambda1 e + lambda2 edot)  with P solved
    for THAT closed-loop A = [[0, I], [-Lambda lambda1, -Lambda lambda2]]: every
    Table II number is used and Theorem 1's proof is unchanged (it needs only A
    Hurwitz and P its Lyapunov solution). With Lambda = I it is the literal (35).
 2. Q_j. Table II's Q is 3x3 (2x2 for the arm) while P_j is 6x6; read as the
    per-axis Q = diag(q_e, q_v) of each [e_i; edot_i] pair (q_e = q_v = 1 is the
    table's identity).
 3. GRAVITY. tau_p (8a) carries no weight term: g_p sits in E_p, so with
    Table II's gains the adaptive term would have to supply the whole weight
    through rho r / varpi -- metres of error at hover. The paper's own vehicle
    cannot have flown that way. Added, as the ONE structural addition:
    tau_p += m_g g e3 with m_g the nominal mass (pos_gravity_ff). It moves the
    known part of g_p from E_p into the command, i.e. sigma_p is redefined by a
    constant and the bound (11) keeps its form (K0* absorbs the rest). The ARM
    gets nothing of the kind: its gravity is configuration-dependent and the
    paper's claim is "no model"; the arm module must carry it with dtau_a.
 4. The desired attitude derivatives (q_dot_d, q_ddot_d in 15/18a) are not
    specified. Here: the FLAT reference's -- the planner's CoM chain through
    snap and heading chain through two derivatives give the desired attitude
    trajectory R_ff(t), Omega_ff, Omega_ff_dot exactly (the same map the
    whole-body law uses); R_d itself is built from the commanded tau_p and the
    reference heading, as the paper does. Geometric form of (15)/(18a):
       e_Omega = Omega - R^T w_d,  ff_q = R^T a_d - hat(Omega) R^T w_d
    with w_d = R_ff Omega_ff, a_d = R_ff Omega_ff_dot (world frame).
 5. chi_ddot. "computed numerically": a first-order-filtered difference of the
    measured [p_dot; Omega; alpha_dot] at the law rate (acc_filter_hz), exact
    discretisation. Same for alpha_ddot_d from the streamed alpha_dot_d (the
    planner publishes no joint acceleration).
 6. The adaptive laws are integrated EXACTLY over a tick with their inputs
    held (zero-order hold), not by Euler -- the repo's rotor-lag / L1-predictor
    lesson; for nu*dt << 1 the two agree.
 7. The base reference. The paper tracks the QUADROTOR position p. The planner
    streams the system-CoM chain (WholeBodyReference); the base reference is
    p_d = x_cd - R_ff r_0c(alpha_d) with its derivatives formed analytically
    from the same chain (r_0c from the planner's own kinematic model). That is
    the REFERENCE layer -- the decoupled_reference_bridge's conversion -- and it
    makes both rigs fly the same planned end-effector motion; the CONTROL law
    uses no model at all (beyond m_g above).

FRAMES. The law runs in the ACTUAL frames: ENU world, FLU body (x = the arm
side), joint angles in the model convention (identity in Isaac). The reference
stream is MODEL frame (WholeBodyReference's contract); ReferenceConverter owns
that one boundary, exactly like frame_adapter.hpp: R_actual = R_model R_MODEL^T.

    python3 modular_adaptive.py      # self-test (P, flat map, CoM FK, base chain, closed loop)
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

G = 9.81            # the whole-body model's g (wb_model.hpp WholeBodyParams::g)
E3 = np.array([0.0, 0.0, 1.0])
# columns = MODEL axes expressed in the ACTUAL frame (frame_adapter.hpp modelFromActual)
R_MODEL = np.array([[0.0, 1.0, 0.0],
                    [-1.0, 0.0, 0.0],
                    [0.0, 0.0, 1.0]])


def hat(v):
    return np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])


def vee(S):
    return np.array([S[2, 1], S[0, 2], S[1, 0]])


def joint_rotation(h, q):
    """Rodrigues -- byte-matched to controller.joint_rotation / wb_model jointRotation."""
    a = np.asarray(h, float) / (np.linalg.norm(h) + 1e-15)
    K = hat(a)
    return np.eye(3) + math.sin(q) * K + (1.0 - math.cos(q)) * (K @ K)


# =============================================================================
# One module (paper eqs. 7/8/12/13, 17/18/21/22, 24/25/28/29)
# =============================================================================
@dataclass
class ModuleGains:
    mbar: np.ndarray            # Mbar_j diagonal (n)
    lam1: np.ndarray            # lambda_j1 diagonal (n)
    lam2: np.ndarray            # lambda_j2 diagonal (n)
    Lam: np.ndarray             # Lambda_j diagonal (n) -- see choice 1
    q_e: float = 1.0            # Q_j = diag(q_e, q_v) per axis -- choice 2
    q_v: float = 1.0
    nu: np.ndarray = field(default_factory=lambda: np.full(4, 10.0))   # nu_j0..nu_j3
    eps: float = 1e-4           # epsilon_j: the zeta forcing term
    varpi: float = 0.1          # varpi_j: boundary-layer width
    K_init: float = 0.01        # K_hat_ji(0), all i
    zeta_init: float = 0.1      # zeta_j(0)

    def __post_init__(self):
        for k in ("mbar", "lam1", "lam2", "Lam", "nu"):
            setattr(self, k, np.asarray(getattr(self, k), float))


def lyapunov_pair(a, b, q_e, q_v):
    """P of A = [[0, 1], [-a, -b]] with A^T P + P A = -diag(q_e, q_v); returns (p11, p12, p22)."""
    p12 = q_e / (2.0 * a)
    p22 = (p12 + 0.5 * q_v) / b
    p11 = a * p22 + b * p12
    return p11, p12, p22


class AdaptiveModule:
    def __init__(self, g: ModuleGains):
        self.g = g
        self.n = len(g.mbar)
        self.K1 = g.Lam * g.lam1          # effective stiffness  (acceleration units)
        self.K2 = g.Lam * g.lam2          # effective damping
        P = [lyapunov_pair(a, b, g.q_e, g.q_v) for a, b in zip(self.K1, self.K2)]
        self.p12 = np.array([p[1] for p in P])
        self.p22 = np.array([p[2] for p in P])
        self.reset()

    def reset(self):
        self.K = np.full(4, float(self.g.K_init))
        self.zeta = float(self.g.zeta_init)

    def command(self, e, edot, ff, xi_n, chi_n):
        """Acceleration-level command v (tau_j = Mbar v) with the CURRENT estimates."""
        g = self.g
        r = self.p12 * e + self.p22 * edot
        nr = float(np.linalg.norm(r))
        rho = (self.K[0] + self.K[1] * xi_n + self.K[2] * xi_n ** 2 + self.K[3] * chi_n + self.zeta)
        if nr >= g.varpi:
            dtau = rho * r / nr
        else:
            dtau = rho * r / g.varpi
        v = -self.K1 * e - self.K2 * edot - dtau + ff
        return v, dict(r=r, nr=nr, rho=rho, dtau=dtau)

    def adapt(self, nr, xi_n, chi_n, dt):
        """Exact ZOH update of K_hat_0..3 and zeta (choice 6). Inputs held over dt."""
        g = self.g
        drive = np.array([nr, nr * xi_n, nr * xi_n ** 2, nr * chi_n])
        for i in range(4):
            nu = g.nu[i]
            if nu > 0.0:
                a = math.exp(-nu * dt)
                self.K[i] = a * self.K[i] + drive[i] * (1.0 - a) / nu
            else:
                self.K[i] = self.K[i] + drive[i] * dt
        if nr < g.varpi:
            c = 1.0 + (self.K[3] * chi_n + self.K[0] + self.K[1] * xi_n + self.K[2] * xi_n ** 2) * nr
            a = math.exp(-c * dt)
            self.zeta = a * self.zeta + g.eps * (1.0 - a) / c
        # |r| >= varpi: zeta' = 0


# =============================================================================
# Reference layer (choice 4 / 7): flat attitude and the base reference
# =============================================================================
def flat_attitude(f, f_dot, f_ddot, b1d, b1d_dot, b1d_ddot):
    """Desired rotation + body rate + body angular acceleration of the flat map
    b3 = f/|f|, b1 = proj(b1d) -- the whole-body law's two-level construction
    (wb_controller.cpp) fed the REFERENCE chain only (f = x_cd_ddot + g e3)."""
    nf = np.linalg.norm(f)
    b3 = f / nf
    P1 = np.eye(3) - np.outer(b3, b3)
    s = P1 @ b1d
    b3d = (P1 @ f_dot) / nf
    ns = np.linalg.norm(s)
    b1 = s / ns
    P1d = -(np.outer(b3d, b3) + np.outer(b3, b3d))
    P2 = np.eye(3) - np.outer(b1, b1)
    sd = P1d @ b1d + P1 @ b1d_dot
    b1dot = (P2 @ sd) / ns
    P2d = -(np.outer(b1dot, b1) + np.outer(b1, b1dot))
    b2 = np.cross(b3, b1)
    R = np.column_stack([b1, b2, b3])
    b2dot = np.cross(b3d, b1) + np.cross(b3, b1dot)
    Rd = np.column_stack([b1dot, b2dot, b3d])
    w = vee(R.T @ Rd)
    b3dd = (P1d @ f_dot + P1 @ f_ddot) / nf - (b3 @ f_dot) / nf ** 2 * (P1 @ f_dot)
    P1dd = -(np.outer(b3dd, b3) + 2.0 * np.outer(b3d, b3d) + np.outer(b3, b3dd))
    sdd = P1dd @ b1d + 2.0 * (P1d @ b1d_dot) + P1 @ b1d_ddot
    b1dd = (P2d @ sd + P2 @ sdd) / ns - (b1 @ sd) / ns * b1dot
    b2dd = np.cross(b3dd, b1) + 2.0 * np.cross(b3d, b1dot) + np.cross(b3, b1dd)
    Rdd = np.column_stack([b1dd, b2dd, b3dd])
    wdot = vee(Rd.T @ Rd + R.T @ Rdd)
    return R, w, wdot


def com_offset(q, params):
    """r_0c(q): system CoM relative to the body origin, MODEL base frame -- the
    r_0c_0 of controller.dynamics() (and wb_model computeDynamics), FK only."""
    n = params["n"]
    mi = params["m_i"]
    Rk = np.eye(3)
    O = np.zeros(3)
    acc = mi[0] * np.asarray(params.get("base_com", np.zeros(3)), float)
    for i in range(n):
        Rk = Rk @ joint_rotation(params["h_i_im1"][i], q[i])
        acc = acc + mi[i + 1] * (O + Rk @ params["com_i"][i])
        O = O + Rk @ params["l_i"][i]
    return acc / sum(mi)


def com_jets(q, qd, params, eps_j=1e-6, eps_h=1e-4):
    """r_0c(q), J_c(q) qd, and the directional second derivative H[qd, qd]."""
    r = com_offset(q, params)
    n = len(q)
    J = np.zeros((3, n))
    for k in range(n):
        dq = np.zeros(n); dq[k] = eps_j
        J[:, k] = (com_offset(q + dq, params) - com_offset(q - dq, params)) / (2.0 * eps_j)
    Hvv = (com_offset(q + eps_h * qd, params) - 2.0 * r + com_offset(q - eps_h * qd, params)) / eps_h ** 2
    return r, J, Hvv


class ReferenceConverter:
    """WholeBodyReference (MODEL frame) -> what the modular law tracks (ACTUAL frames).

    The one place the model<->actual boundary is crossed. alpha_ddot_d is a
    filtered difference of the streamed alpha_dot_d, updated only when a NEW
    sample arrives (sample_changed) -- the law latches the last sample between
    planner ticks exactly as the C++ node does."""

    def __init__(self, params, qdd_filter_hz=10.0):
        self.params = params
        self.wq = 2.0 * math.pi * qdd_filter_hz
        self.reset()

    def reset(self):
        self.qd_prev = None
        self.t_since = 0.0
        self.qdd = np.zeros(self.params["n"])

    def convert(self, ref, dt, sample_changed):
        n = self.params["n"]
        q_d = np.asarray(ref["q_d"], float)
        qd_d = np.asarray(ref["qdot_d"], float)
        # ---- alpha_ddot_d: difference across planner samples, first-order filter
        self.t_since += dt
        if self.qd_prev is None:
            self.qd_prev = qd_d.copy(); self.t_since = 0.0
        elif sample_changed and self.t_since > 1e-6:
            raw = (qd_d - self.qd_prev) / self.t_since
            a = math.exp(-self.wq * self.t_since)
            self.qdd = a * self.qdd + (1.0 - a) * raw
            self.qd_prev = qd_d.copy(); self.t_since = 0.0
        # ---- flat attitude of the reference (MODEL frame)
        f = np.asarray(ref["x_cd_ddot"], float) + G * E3
        Rm, wm, wdm = flat_attitude(f, np.asarray(ref["x_cd_d3"], float), np.asarray(ref["x_cd_d4"], float),
                                    np.asarray(ref["b1_d"], float), np.asarray(ref["b1_d_dot"], float),
                                    np.asarray(ref["b1_d_ddot"], float))
        # ---- base reference p_d = x_cd - R r_0c(alpha_d), analytic chain
        r, J, Hvv = com_jets(q_d, qd_d, self.params)
        Jqd = J @ qd_d
        W = hat(wm)
        p_d = np.asarray(ref["x_cd"], float) - Rm @ r
        v_d = np.asarray(ref["x_cd_dot"], float) - Rm @ (W @ r + Jqd)
        a_d = np.asarray(ref["x_cd_ddot"], float) - Rm @ (hat(wdm) @ r + W @ W @ r + 2.0 * W @ Jqd
                                                          + J @ self.qdd + Hvv)
        Ra = Rm @ R_MODEL.T                       # ACTUAL desired attitude (x = arm side)
        return dict(p_d=p_d, v_d=v_d, a_d=a_d, R_ff=Ra, b1_head=Ra[:, 0],
                    w_d=Rm @ wm, wdot_d=Rm @ wdm,           # world frame, frame-free
                    q_d=q_d, qdot_d=qd_d, qddot_d=self.qdd.copy())


# =============================================================================
# The law
# =============================================================================
@dataclass
class ModularConfig:
    pos: ModuleGains
    att: ModuleGains
    arm: ModuleGains
    m_g: float = 3.746170          # nominal mass of the gravity term (choice 3); 0 = literal (8a)
    tau_max: float = 3.0           # joint actuator limit [N.m] (one number across law/controller/plant)
    acc_filter_hz: float = 10.0    # chi_ddot numerical differentiation cutoff (choice 5)
    min_thrust_frac: float = 0.2   # tau_p,z floor as a fraction of m_g g (an inverted thrust
                                   # direction is not a command the vehicle can take)


class ModularAdaptiveLaw:
    def __init__(self, cfg: ModularConfig):
        self.cfg = cfg
        self.mp = AdaptiveModule(cfg.pos)
        self.mq = AdaptiveModule(cfg.att)
        self.ma = AdaptiveModule(cfg.arm)
        self.reset()

    def reset(self):
        for m in (self.mp, self.mq, self.ma):
            m.reset()
        self.prev = None
        self.chi_dd = None

    def _chi_ddot(self, v, Om, qd, dt):
        x = np.concatenate([v, Om, qd])
        if self.prev is None or dt <= 0.0:
            self.prev = x.copy(); self.chi_dd = np.zeros_like(x)
            return self.chi_dd
        raw = (x - self.prev) / dt
        a = math.exp(-2.0 * math.pi * self.cfg.acc_filter_hz * dt)
        self.chi_dd = a * self.chi_dd + (1.0 - a) * raw
        self.prev = x.copy()
        return self.chi_dd

    def step(self, meas, rc, dt):
        """meas: p, v (world), R (actual body->world), Omega (FLU body), q, qdot (model joints).
        rc: ReferenceConverter.convert() output. Returns the command + diagnostics."""
        cfg = self.cfg
        p, v, R, Om = meas["p"], meas["v"], meas["R"], meas["Omega"]
        q, qd = meas["q"], meas["qdot"]
        chi_dd = self._chi_ddot(v, Om, qd, dt)

        # ---- errors of all three subsystems (the whole-system |xi| of eq. 12)
        e_p = p - rc["p_d"]
        ed_p = v - rc["v_d"]
        # attitude: R_d from the COMMANDED thrust direction -> needs tau_p first.
        # |xi| uses the attitude error of the PREVIOUS desired attitude? No: the
        # paper evaluates everything at the same instant; R_d depends on tau_p,
        # which depends on rho_p(|xi|), which contains e_q(R_d). Broken the way
        # the paper's cascade is implemented in practice: the position module
        # uses |xi| with the attitude error against the REFERENCE attitude R_ff
        # (same instant, no algebraic loop); the attitude module then uses the
        # true R_d. Recorded as xi_pos_att_source = "R_ff".
        e_q_ff = 0.5 * vee(rc["R_ff"].T @ R - R.T @ rc["R_ff"])
        ed_q_ff = Om - R.T @ rc["w_d"]
        e_a = q - rc["q_d"]
        ed_a = qd - rc["qdot_d"]
        xi_n0 = float(np.linalg.norm(np.concatenate([e_p, e_q_ff, e_a, ed_p, ed_q_ff, ed_a])))
        chi_n = float(np.linalg.norm(chi_dd))

        # ---- position module -> tau_p (world thrust vector), eq. 8
        v_p, dp = self.mp.command(e_p, ed_p, rc["a_d"], xi_n0, chi_n)
        tau_p = cfg.pos.mbar * v_p + cfg.m_g * G * E3
        tau_p[2] = max(tau_p[2], cfg.min_thrust_frac * cfg.m_g * G)
        u1 = float(tau_p @ (R @ E3))

        # ---- desired attitude from tau_p and the reference heading
        b3c = tau_p / np.linalg.norm(tau_p)
        s = rc["b1_head"] - b3c * float(b3c @ rc["b1_head"])
        b1c = s / np.linalg.norm(s)
        Rd = np.column_stack([b1c, np.cross(b3c, b1c), b3c])

        # ---- attitude module -> body moment, eqs. 14/15/18 (geometric form, choice 4)
        e_q = 0.5 * vee(Rd.T @ R - R.T @ Rd)
        wb = R.T @ rc["w_d"]
        ed_q = Om - wb
        ff_q = R.T @ rc["wdot_d"] - hat(Om) @ wb
        xi_n = float(np.linalg.norm(np.concatenate([e_p, e_q, e_a, ed_p, ed_q, ed_a])))
        v_q, dq = self.mq.command(e_q, ed_q, ff_q, xi_n, chi_n)
        tau_q = cfg.att.mbar * v_q

        # ---- arm module -> joint torques, eq. 25
        v_a, da = self.ma.command(e_a, ed_a, rc["qddot_d"], xi_n, chi_n)
        tau_a_raw = cfg.arm.mbar * v_a
        tau_a = np.clip(tau_a_raw, -cfg.tau_max, cfg.tau_max)
        n_sat = int(np.sum(np.abs(tau_a_raw) >= cfg.tau_max))

        # ---- adaptation (after the command: the tick used the current estimates)
        self.mp.adapt(dp["nr"], xi_n0, chi_n, dt)
        self.mq.adapt(dq["nr"], xi_n, chi_n, dt)
        self.ma.adapt(da["nr"], xi_n, chi_n, dt)

        return dict(tau_p=tau_p, u1=u1, Rd=Rd, tau_q=tau_q, tau_a=tau_a, n_sat=n_sat,
                    e_p=e_p, ed_p=ed_p, e_q=e_q, ed_q=ed_q, e_a=e_a, ed_a=ed_a,
                    xi_n=xi_n, xi_n0=xi_n0, chi_n=chi_n,
                    rho=np.array([dp["rho"], dq["rho"], da["rho"]]),
                    nr=np.array([dp["nr"], dq["nr"], da["nr"]]),
                    dtau_p=dp["dtau"], dtau_q=dq["dtau"], dtau_a=da["dtau"],
                    K_p=self.mp.K.copy(), K_q=self.mq.K.copy(), K_a=self.ma.K.copy(),
                    zeta=np.array([self.mp.zeta, self.mq.zeta, self.ma.zeta]))


# =============================================================================
# Table II, literally (S-500 + 2R arm, 2.2 kg) -- the paper's numbers, mapped
# =============================================================================
def table2_config(m_g=3.746170):
    """The paper's Table II gains on a 4-joint arm. NOT a tune for this plant:
    Mbar_pp = I on a 3.75 kg vehicle and Mbar_alpha = 0.1 I against a 0.7 N.m
    gravity load are the S-500's numbers."""
    pos = ModuleGains(mbar=np.ones(3), lam1=2 * np.array([1, 1, 2.0]), lam2=np.array([1, 1, 2.0]),
                      Lam=np.array([1.5, 1.5, 2.0]), nu=np.full(4, 10.0), eps=1e-4, varpi=0.1,
                      K_init=0.01, zeta_init=0.1)
    att = ModuleGains(mbar=np.full(3, 0.015), lam1=2 * np.full(3, 2.0), lam2=np.full(3, 2.0),
                      Lam=np.array([3.5, 3.5, 2.5]), nu=np.full(4, 20.0), eps=1e-4, varpi=1.0,
                      K_init=0.001, zeta_init=0.01)
    arm = ModuleGains(mbar=np.full(4, 0.1), lam1=2 * np.full(4, 1.5), lam2=np.full(4, 1.5),
                      Lam=np.ones(4), nu=np.full(4, 1.0), eps=1e-4, varpi=0.1,
                      K_init=1e-4, zeta_init=0.01)
    return ModularConfig(pos=pos, att=att, arm=arm, m_g=m_g)


# =============================================================================
# Self-test
# =============================================================================
def _selftest():
    import os
    import sys
    rng = np.random.default_rng(1)
    ok = True

    def check(name, err, tol):
        nonlocal ok
        good = err <= tol
        ok &= good
        print(f"  [{'PASS' if good else 'FAIL'}] {name}: {err:.3e} (tol {tol:.0e})")

    print("1. Lyapunov pair")
    for _ in range(5):
        a, b, qe, qv = rng.uniform(0.5, 20.0), rng.uniform(0.5, 10.0), rng.uniform(0.1, 5), rng.uniform(0.1, 5)
        p11, p12, p22 = lyapunov_pair(a, b, qe, qv)
        A = np.array([[0, 1.0], [-a, -b]]); P = np.array([[p11, p12], [p12, p22]])
        check("A^T P + P A + Q", np.abs(A.T @ P + P @ A + np.diag([qe, qv])).max(), 1e-12)
        check("P > 0", max(0.0, -np.linalg.eigvalsh(P).min()), 0.0)

    print("2. flat attitude map vs finite differences")
    def chain(t):
        # a smooth synthetic reference: CoM on a tilted circle, heading turning
        w = 0.7
        x = np.array([0.4 * math.cos(w * t), 0.3 * math.sin(w * t), 0.1 * math.sin(2 * w * t)])
        d = [x]
        for k in range(1, 5):
            d.append(np.array([0.4 * w ** k * math.cos(w * t + k * math.pi / 2),
                               0.3 * w ** k * math.sin(w * t + k * math.pi / 2),
                               0.1 * (2 * w) ** k * math.sin(2 * w * t + k * math.pi / 2)]))
        psi, pd = 0.3 * t + 0.2 * math.sin(t), 0.3 + 0.2 * math.cos(t)
        pdd = -0.2 * math.sin(t)
        b1 = np.array([math.cos(psi), math.sin(psi), 0.0])
        b1d = pd * np.array([-math.sin(psi), math.cos(psi), 0.0])
        b1dd = pdd * np.array([-math.sin(psi), math.cos(psi), 0.0]) - pd ** 2 * b1
        return d, b1, b1d, b1dd
    h = 1e-5
    for t in (0.3, 1.7, 4.1):
        def RW(tt):
            d, b1, b1d, b1dd = chain(tt)
            return flat_attitude(d[2] + G * E3, d[3], d[4], b1, b1d, b1dd)
        R, w, wd = RW(t)
        Rp, wp, _ = RW(t + h); Rm, wm, _ = RW(t - h)
        w_fd = vee(R.T @ (Rp - Rm) / (2 * h))
        check(f"Omega t={t}", np.abs(w - w_fd).max(), 1e-7)
        check(f"Omega_dot t={t}", np.abs(wd - (wp - wm) / (2 * h)).max(), 1e-6)
        check(f"R orthonormal t={t}", np.abs(R.T @ R - np.eye(3)).max(), 1e-12)

    here = os.path.dirname(os.path.abspath(__file__))
    sys.path.insert(0, os.path.abspath(os.path.join(here, "..", "..", "..")))
    from fsc_aerial_manipulation.robotic_arm.utils_controller import controller as C
    from fsc_aerial_manipulation.robotic_arm.utils_planner import transition_planner as TP
    params = TP.make_params_t650(armature_diag=[0.010, 0.0194, 0.0097, 0.0097])
    params["base_com"] = np.array([0.001, -0.017854, 0.0])

    print("3. CoM FK vs controller.dynamics r_0c_0")
    n = params["n"]
    for _ in range(5):
        X = np.zeros(12 + 3 * n + 6)
        X[3:12] = np.eye(3).flatten(order="F")
        X[12:12 + n] = rng.uniform(-1.0, 1.0, n)
        check("r_0c", np.abs(com_offset(X[12:12 + n], params) - C.dynamics(X, params)["r_0c_0"]).max(), 1e-14)

    print("4. base reference chain vs finite differences of p_d(t)")
    def wbref(t):
        d, b1, b1d, b1dd = chain(t)
        qq = np.array([0.1 * math.sin(t), 0.6 + 0.26 * math.sin(1.04 * t), 0.4 - 0.1 * math.cos(t), 0.2 * t])
        qqd = np.array([0.1 * math.cos(t), 0.26 * 1.04 * math.cos(1.04 * t), 0.1 * math.sin(t), 0.2])
        return dict(x_cd=d[0], x_cd_dot=d[1], x_cd_ddot=d[2], x_cd_d3=d[3], x_cd_d4=d[4],
                    b1_d=b1, b1_d_dot=b1d, b1_d_ddot=b1dd, q_d=qq, qdot_d=qqd)
    rc = ReferenceConverter(params, qdd_filter_hz=1e6)     # the qdd filter is a pure difference here
    h2 = 1e-4
    for t in (0.5, 2.2):
        rc.reset(); rc.convert(wbref(t - h2), h2, True)
        out = rc.convert(wbref(t), h2, True)                # qdd = backward difference over h2
        pp = ReferenceConverter(params).convert(wbref(t + h2), 0.0, False)["p_d"]
        pm = ReferenceConverter(params).convert(wbref(t - h2), 0.0, False)["p_d"]
        check(f"v_d t={t}", np.abs(out["v_d"] - (pp - pm) / (2 * h2)).max(), 1e-6)
        check(f"a_d t={t}", np.abs(out["a_d"] - (pp - 2 * out["p_d"] + pm) / h2 ** 2).max(), 2e-3)
        # same-motion: the actual heading is the model y axis
        check(f"heading = model y t={t}", abs(out["b1_head"] @ out["R_ff"][:, 0] - 1.0), 1e-12)

    print("5. closed loop on a decoupled double-integrator plant (UUB, no model)")
    cfg = table2_config(m_g=2.0)
    cfg.pos.mbar = np.full(3, 2.0)
    law = ModularAdaptiveLaw(cfg)
    rc0 = dict(p_d=np.zeros(3), v_d=np.zeros(3), a_d=np.zeros(3), R_ff=np.eye(3), b1_head=np.array([1.0, 0, 0]),
               w_d=np.zeros(3), wdot_d=np.zeros(3), q_d=np.zeros(4), qdot_d=np.zeros(4), qddot_d=np.zeros(4))
    p = np.array([0.3, -0.2, 0.1]); v = np.zeros(3); qa = np.full(4, 0.2); qad = np.zeros(4)
    dt = 0.004
    for k in range(int(30 / dt)):
        out = law.step(dict(p=p, v=v, R=np.eye(3), Omega=np.zeros(3), q=qa, qdot=qad), rc0, dt)
        acc = (out["tau_p"] - 2.0 * G * E3 + np.array([0.3, -0.2, 0.5])) / 2.0      # unknown constant force
        v = v + acc * dt; p = p + v * dt
        qadd = (out["tau_a"] - 0.02 * np.sign(qa)) / 0.1                           # Mbar = true inertia
        qad = qad + qadd * dt; qa = qa + qad * dt
    check("position UUB residual |e_p| after 30 s", float(np.linalg.norm(p)), 0.30)
    check("adaptive gains stay finite and positive", 0.0 if (np.all(law.mp.K > 0) and np.all(np.isfinite(law.mp.K))) else 1.0, 0.0)
    print("ALL PASS" if ok else "SOME CHECKS FAILED")
    return ok


if __name__ == "__main__":
    raise SystemExit(0 if _selftest() else 1)
