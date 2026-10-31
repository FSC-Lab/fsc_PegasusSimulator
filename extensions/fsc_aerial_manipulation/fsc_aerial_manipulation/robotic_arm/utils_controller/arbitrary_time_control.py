#!/usr/bin/env python3
"""arbitrary_time_control.py -- the ARBITRARY CONVERGENCE TIME control
architecture of

    Y. Chen, X. Liu, H. Cai, C. Shang, X. Chang, F. Chao, Y. Wang, Q. Shen,
    "Arbitrary Convergence Time Control of Unmanned Aerial Manipulator for
    Active Interaction and Dynamic Tracking", IEEE Trans. Industrial
    Electronics 72(12), 2025, pp. 13693-13702
    (docs/docs_aerial_manipulator/references/Arbitrary_Convergence_Time_...pdf),

written for the SIMULATION comparison of the box-pushing task against the
whole-body L1 impedance law (2026-10-24). Pure numpy. It is the source of truth
for the C++ flight node `autopilot_arbitrary_time_direct_actuation_node`
(fsc_autopilot_ros2, single_aerial_manipulator_arbitrary_time_direct_actuation),
held to THIS file by a parity fixture
(application/robotic_arm/utils/generate_arbitrary_time_truth.py).

INDEPENDENT OF THE WHOLE-BODY CONTROLLER on purpose (user requirement): nothing
here imports utils_planner, controller.py or l1_observer.py. The arm's mass
properties are DATA copied below from the T650 aerial-manipulator model (the
numbers fsc_trajectory_planner builds its references on), so this rig and the
planner describe the same robot -- the self-test checks that against the
planner fixture's home case -- but a later change to the whole-body code
cannot move this rig.

THE METHOD (paper Section III), decoupled by construction:

  A. ACTNDO, the arbitrary-convergence-time nonlinear disturbance observer of
     the JOINT-SPACE interaction torque tau_ext (eqs 6-11), on the arm's own
     fixed-base dynamics M q'' + C q' + G = tau_m + tau_ext (eq 4), H = M q':
       stage 1   zeta' = C' q' - G + tau_m + tau1_hat + w1,  s = H - zeta,
                 tau1_hat' = Gamma1 s,   w1 = eta_o1 phi(s) / t_go1
                 (once s = 0, tau1_hat + w1 IS tau_ext: eq 8 with s' = 0)
       stage 2   tau_hat = rho + k H,
                 rho' = -k (C' q' - G + tau_m + tau_hat) + w2 + tau2_hat,
                 nu = (tau1_hat + w1) - tau_hat,  tau2_hat' = Gamma2 nu,
                 w2 = eta_o2 phi(nu) / t_go2
     SIGN NOTE (eqs 9, 10): as printed, w_o2 enters with the sign that makes
     the proof's V_o2' POSITIVE. Implemented in the form the proof (eq 17)
     needs: tau_tilde' = -k tau_tilde - w2 - tau2_hat + tau_ext'. The adaptive
     bounds tau1_hat / tau2_hat are the paper's (eq 11); Gamma1 / Gamma2 are not
     given in the paper (they are this rig's choices).

  B. TSKFE, the Takagi-Sugeno-Kang fuzzy estimator of the quadrotor's lumped
     disturbance D_u (6 channels) with k-closest fuzzy rule interpolation
     (eqs 19-24, 29): inputs x1 = max|chi_d - chi|, x2 = max|chi_d' - chi'|
     over the 6 channels; a 5 x 5 initial rule base of the paper's triangles;
     an input that fires no rule spawns a new rule whose consequent is the
     similarity-weighted mean (eq 21) of the k = 3 closest; output (eq 22)
     D_hat_e = sum W_r f_r,e / sum W_r with f = a1 x1 + a2 x2 + a0 and
     W = mu1 mu2; consequents adapted by eq 29,
       omega_e' = c_e (M_u^-T eps)_e X phi(x)^T,   X = [x1, x2, 1].

  C. The quadrotor's arbitrary-convergence-time controller (eqs 25-28) on the
     6-DoF model M_u chi'' + C_u chi' + G_u + D_u = tau_u + J_E tau_ext,
     chi = [p_b; Phi_b] (Phi = ZYX Euler), a backstepping law per channel:
       r1 = chi_d - chi,  r2 = chi_d' - chi',  delta = -eta1 phi(r1) / t_go,
       eps = r2 - delta,
       N = eta1 [phi'(r1) r2 / t_go + phi(r1) / t_go^2] + eta2 phi(eps) / t_go,
       tau_u = C_u chi' + G_u + M_u (chi_d'' + r1 + N) - J_E tau_hat + D_hat.
     The position rows give the desired force; the paper's attitude decoupling
     module (eq 3) turns it into the thrust f = |F| and the desired roll/pitch,
     which are the attitude rows' references.

  D. The onboard manipulator's arbitrary-convergence-time admittance
     controller (eqs 34-38): the admittance model
       H_d (qc'' - qd'') + D_d (qc' - qd') + K_d (qc - qd) = tau_hat
     makes the compliant joint reference q_c, tracked by the same backstepping
     law, tau_m = C q' + G + M (qc'' + th1 + N_m) - tau_hat.

WHERE THE PAPER LEAVES A CHOICE, what this file does (each stated so a log can
say which way a run went):
  (a) THE TIME-TO-GO IS FLOORED (the one structural addition). The paper's gains
      grow as 1/(t_e - t) and 1/(t_e - t)^2 inside each window and are ZERO
      after it ("0, otherwise", eqs 7, 10, 26, 28, 36, 38). Neither half can run
      in discrete time on a real plant: the first is unbounded at t_e, and the
      second leaves the controllers with the single term M r1 -- an UNDAMPED
      unit oscillator (r1'' = -r1) on every channel, the arm and the observer
      likewise. Here t_go = max(t_e - t, T_floor): inside the window the
      paper's law verbatim until t_go reaches the floor, then the SAME law with
      d t_go/dt = 0, i.e. a constant-gain backstepping
          r1' = eps - (eta1/T) phi(r1),  eps' = -r1 - (eta2/T) phi(eps)
      (exponentially stable; small-signal r1'' + (a+b) r1' + (1+ab) r1 = 0,
      a = eta1/T, b = eta2/T). The floor is per module (observer, quadrotor,
      arm).
  (b) phi(x) = sign(x)(1 - exp(-|x|)) elementwise. The paper prints
      (e^x - 1)/e^x = 1 - e^-x, which has the right sign but grows without
      bound for x < 0; the symmetric form is the one under which Lemma 1
      applies to every scalar channel exactly (xi = |x|).
  (c) WINDOWS. The observer's windows restart at every interaction edge (the
      planner's contact flag on/off) -- t_s, "the initial time of interaction"
      (eq 7) -- and at DIRECT entry. The controllers' windows start at DIRECT
      entry only (the experiment's "switch to the proposed control strategy"),
      so they do not soften when the interaction starts
      (controller_window_on_contact = True restarts them too: t_u = t_s + DT_u
      read literally).
  (d) eta PER CHANNEL. The paper uses one eta_u1/eta_u2 pair for all six
      quadrotor channels and one eta_m1/eta_m2 for all joints. Position and
      attitude need bandwidths a factor ~3 apart on this plant (the attitude
      loop must clear the 99.7 ms rotor lag, the position loop must not), so
      the etas are per channel; equal values reproduce the paper.
  (e) THE ATTITUDE REFERENCE'S DERIVATIVES are zero (Phi_d' = Phi_d'' = 0): the
      decoupling module gives Phi_d only, and the paper is silent on its
      derivatives (the constant-reference specialisation the other decoupled
      rigs use).
  (f) WORLD-FRAME POSITION ROWS. The paper writes M_u = diag(m R^-1, ...) (body-
      frame forces) but feeds t1..t3 to the WORLD-frame decoupling module eq 3;
      the position rows are implemented in the world frame (M = m I), which is
      what eq 3 assumes.
  (g) J_E tau_hat for a 4-joint arm. J_m^-T is the damped least-squares inverse
      of the 3 x 4 position Jacobian's transpose (F = (Jv Jv' + lam^2 I)^-1 Jv
      tau); the base receives F and the moment r_e x F about its origin.
  (h) TSKFE details: a rule MATCHES when it fires with W > l_max (strictly; with
      the paper's l_max = 0 the non-strict test would match everything and no
      rule would ever be generated, contradicting the paper's 1295 new rules);
      a new rule's antecedents are the initial triangles' shapes centred on the
      input; the rule base is capped at max_rules (beyond it an unmatched
      input is evaluated by its interpolated consequent without storing it);
      consequents are clamped (safety shaping, not the paper's law).
  (i) The arm dynamics are the paper's FIXED-BASE eq 4 with gravity rotated into
      the measured base frame (g_b = R0^T g), and the joint-diagonal servo
      armature added to M (the arm's physics).

FRAMES. ENU world, FLU body (x = the arm side, the flying AM_xfwd asset). The arm
model is in the MODEL base frame (AM_realign's y-forward frame, the frame the
planner streams in); the one conversion is v_actual = Rz(-90) v_model,
R0_model = R0_actual Rz(-90). Joint angles in the model convention (identity in
Isaac).

    python3 arbitrary_time_control.py      # self-test
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

G_ACC = 9.81
E3 = np.array([0.0, 0.0, 1.0])

# ---------------------------------------------------------------------------
# 0. the arm model (T650 aerial manipulator, gripper EE, MODEL frame)
# ---------------------------------------------------------------------------
BASE_MASS = 2.95 + 0.15955                       # T650 body + the 4 AM rotors
LINK_MASS = np.array([0.1048326, 0.1423463, 0.13992727, 0.2495176])
LINK_INERTIA = np.array([[3.720000e-05, 3.749000e-05, 2.161000e-05],
                         [3.593700e-04, 3.485900e-04, 5.506000e-05],
                         [3.130200e-04, 3.253000e-05, 3.184700e-04],
                         [1.908700e-04, 2.637700e-04, 1.951000e-04]])
JOINT_AXIS = np.array([[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
_RAW_L = np.array([[+0.000500, +0.002000, -0.042000],
                   [-0.019000, +0.000070, -0.040500],
                   [+0.000089, +0.024000, -0.128000],
                   [+0.019976, +0.128927, -0.021750]])
_RAW_COM = np.array([[+0.000960, +0.001520, -0.073027],
                     [+0.000320, +0.010163, -0.137795],
                     [+0.019480, +0.107164, -0.128866],
                     [+0.018278, +0.123460, -0.061369]])
GRIPPER_OFF_WRIST = np.array([0.0, 0.0, -0.108])
ARMATURE = np.array([0.010, 0.0194, 0.0097, 0.0097])   # joint-diagonal servo rotor inertia


def _chain():
    o_raw = np.vstack([np.zeros(3), np.cumsum(_RAW_L, axis=0)])
    link_com = o_raw[:4] + _RAW_COM
    o_chain = np.vstack([np.zeros(3), o_raw[2], o_raw[3], o_raw[4], o_raw[4] + GRIPPER_OFF_WRIST])
    return np.diff(o_chain, axis=0), link_com - o_chain[:4]


LINK_VEC, LINK_COM = _chain()
RZ_M90 = np.array([[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])   # v_actual = RZ_M90 v_model


def hat(v):
    return np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])


def axis_rot(h, q):
    a = h / (np.linalg.norm(h) + 1e-15)
    k = hat(a)
    return np.eye(3) + math.sin(q) * k + (1.0 - math.cos(q)) * (k @ k)


@dataclass
class ArmKin:
    R: list          # R_1..R_4 (link frames, base frame)
    O: list          # O_0..O_4 (chain origins; O_4 = EE)
    z: list          # joint axes z_1..z_4 (base frame)
    com: list        # link CoMs (base frame)


def arm_kinematics(q) -> ArmKin:
    R = np.eye(3)
    O = [np.zeros(3)]
    Rs, zs, coms = [], [], []
    for i in range(4):
        zs.append(R @ JOINT_AXIS[i])
        R = R @ axis_rot(JOINT_AXIS[i], q[i])
        Rs.append(R)
        coms.append(O[i] + R @ LINK_COM[i])
        O.append(O[i] + R @ LINK_VEC[i])
    return ArmKin(Rs, O, zs, coms)


def total_mass():
    return BASE_MASS + LINK_MASS.sum()


def arm_fk(q, base_com):
    """(r_0c, r_0e, Jv_e): system CoM and EE in the MODEL base frame, the EE's
    position Jacobian (3 x 4)."""
    k = arm_kinematics(q)
    acc = BASE_MASS * np.asarray(base_com, float)
    for i in range(4):
        acc = acc + LINK_MASS[i] * k.com[i]
    Jv = np.zeros((3, 4))
    for j in range(4):
        Jv[:, j] = np.cross(k.z[j], k.O[4] - k.O[j])
    return acc / total_mass(), k.O[4].copy(), Jv


def arm_mass_matrix(q):
    k = arm_kinematics(q)
    M = np.diag(ARMATURE).astype(float)
    for i in range(4):
        Jv = np.zeros((3, 4))
        Jw = np.zeros((3, 4))
        for j in range(i + 1):
            Jv[:, j] = np.cross(k.z[j], k.com[i] - k.O[j])
            Jw[:, j] = k.z[j]
        Ii = k.R[i] @ np.diag(LINK_INERTIA[i]) @ k.R[i].T
        M = M + LINK_MASS[i] * Jv.T @ Jv + Jw.T @ Ii @ Jw
    return M


def arm_gravity(q, g_base):
    """G(q) = dU/dq for the gravity vector g_base (base frame, e.g. R0^T [0,0,-g])."""
    k = arm_kinematics(q)
    G = np.zeros(4)
    for i in range(4):
        for j in range(i + 1):
            G[j] -= LINK_MASS[i] * float(np.cross(k.z[j], k.com[i] - k.O[j]) @ g_base)
    return G


FD_H = 1e-6


def arm_coriolis(q, qd):
    """C(q, q') from the Christoffel symbols of M, dM/dq by central differences
    (h = 1e-6). M' - 2C is skew-symmetric (property 2)."""
    dM = []
    for kk in range(4):
        e = np.zeros(4)
        e[kk] = FD_H
        dM.append((arm_mass_matrix(q + e) - arm_mass_matrix(q - e)) / (2.0 * FD_H))
    C = np.zeros((4, 4))
    for i in range(4):
        for j in range(4):
            s = 0.0
            for kk in range(4):
                s += 0.5 * (dM[kk][i, j] + dM[j][i, kk] - dM[i][j, kk]) * qd[kk]
            C[i, j] = s
    return C


def arm_dynamics(q, qd, g_base):
    return arm_mass_matrix(q), arm_coriolis(q, qd), arm_gravity(q, g_base)


# ---------------------------------------------------------------------------
# 1. the time-to-go of an arbitrary-convergence-time window (addition (a))
# ---------------------------------------------------------------------------
def phi(x):
    x = np.asarray(x, float)
    return np.sign(x) * (1.0 - np.exp(-np.abs(x)))


def dphi(x):
    return np.exp(-np.abs(np.asarray(x, float)))


@dataclass
class Window:
    """t_e = t0 + T; t_go = max(t_e - t, floor); `shrinking` is d(t_go)/dt = -1.
    Before any trigger the window sits on its floor (constant gain)."""
    T: float
    floor: float
    t0: float = -1.0e9

    def trigger(self, t):
        self.t0 = t

    def tgo(self, t):
        return max(self.t0 + self.T - t, self.floor)

    def shrinking(self, t):
        return (self.t0 + self.T - t) > self.floor


# ---------------------------------------------------------------------------
# 2. ACTNDO (eqs 6-11)
# ---------------------------------------------------------------------------
@dataclass
class ActndoConfig:
    k: float = 8.2                       # paper experiment
    eta_o1: float = 5.0
    eta_o2: float = 5.0
    gamma1: np.ndarray = field(default_factory=lambda: np.full(4, 25.0))
    gamma2: np.ndarray = field(default_factory=lambda: np.full(4, 25.0))
    dt_o1: float = 0.4
    dt_o2: float = 0.4
    floor: float = 0.25
    est_max: float = 3.0                 # [N.m] clamp on tau_hat (safety shaping)


class Actndo:
    def __init__(self, cfg: ActndoConfig):
        self.c = cfg
        self.w1 = Window(cfg.dt_o1, cfg.floor)
        self.w2 = Window(cfg.dt_o1 + cfg.dt_o2, cfg.floor)
        self.zeta = np.zeros(4)
        self.tau1 = np.zeros(4)
        self.rho = np.zeros(4)
        self.tau2 = np.zeros(4)
        self.est = np.zeros(4)
        self.s1_est = np.zeros(4)        # tau1_hat + w1 (stage 1's reconstruction)

    def reset(self, H):
        self.zeta = np.asarray(H, float).copy()
        self.tau1 = np.zeros(4)
        self.rho = -self.c.k * np.asarray(H, float)
        self.tau2 = np.zeros(4)
        self.est = np.zeros(4)
        self.s1_est = np.zeros(4)

    def trigger(self, t):
        self.w1.trigger(t)
        self.w2.trigger(t)

    def update(self, dt, t, H, CTqd, G, tau_m):
        c = self.c
        s = H - self.zeta
        w1 = c.eta_o1 * phi(s) / self.w1.tgo(t)
        s1 = self.tau1 + w1
        nu = s1 - self.est
        w2 = c.eta_o2 * phi(nu) / self.w2.tgo(t)
        zeta_dot = CTqd - G + tau_m + self.tau1 + w1
        tau1_dot = c.gamma1 * s
        rho_dot = -c.k * (CTqd - G + tau_m + self.est) + w2 + self.tau2
        tau2_dot = c.gamma2 * nu
        self.zeta = self.zeta + dt * zeta_dot
        self.tau1 = self.tau1 + dt * tau1_dot
        self.rho = self.rho + dt * rho_dot
        self.tau2 = self.tau2 + dt * tau2_dot
        self.est = np.clip(self.rho + c.k * H, -c.est_max, c.est_max)
        self.s1_est = s1
        return self.est


# ---------------------------------------------------------------------------
# 3. TSKFE with k-FRI (eqs 19-24, 29)
# ---------------------------------------------------------------------------
def tri(x, a):
    a1, a2, a3 = a
    if x <= a1 or x >= a3:
        return 0.0
    if x <= a2:
        return (x - a1) / (a2 - a1)
    return (a3 - x) / (a3 - a2)


@dataclass
class FuzzyConfig:
    # the paper's experiment: initial sets A_x1^i = (-1.05+0.5i, -1+0.5i, -0.9+0.5i),
    # A_x2^j = (-3.2+1.5j, -3+1.5j, -2.6+1.5j), i, j = 0..4; k = 3; l_max = 0
    k_closest: int = 3
    l_max: float = 0.0
    c: np.ndarray = field(default_factory=lambda: np.array([20.9, 31.1, 33.3, 0.1, 0.1, 0.1]))
    max_rules: int = 400
    a_max: np.ndarray = field(default_factory=lambda: np.array([10.0, 10.0, 10.0, 1.0, 1.0, 1.0]))
    out_max: np.ndarray = field(default_factory=lambda: np.array([10.0, 10.0, 10.0, 1.0, 1.0, 1.0]))


X1_SHAPE = (-0.05, 0.0, 0.10)     # offsets of the initial x1 triangles about their centre
X2_SHAPE = (-0.20, 0.0, 0.40)


class TskFuzzyEstimator:
    def __init__(self, cfg: FuzzyConfig):
        self.c = cfg
        self.reset()

    def reset(self):
        self.A1, self.A2, self.P = [], [], []        # antecedents and consequents [6 x 3] = [a1, a2, a0]
        for i in range(5):
            for j in range(5):
                c1, c2 = -1.0 + 0.5 * i, -3.0 + 1.5 * j
                self.A1.append(tuple(c1 + o for o in X1_SHAPE))
                self.A2.append(tuple(c2 + o for o in X2_SHAPE))
                self.P.append(np.zeros((6, 3)))
        self.n_generated = 0
        self.n_ephemeral = 0

    def _firing(self, x1, x2):
        return np.array([tri(x1, a) * tri(x2, b) for a, b in zip(self.A1, self.A2)])

    def _interpolate(self, x1, x2):
        """k-FRI: the k closest rules by the similarity of eq 20, their consequents
        averaged with weights w = min(S1, S2) (eq 21)."""
        w = []
        for a, b in zip(self.A1, self.A2):
            d1 = sum(abs(ai - x1) for ai in a) / 3.0
            d2 = sum(abs(bi - x2) for bi in b) / 3.0
            w.append(min(1.0 / (1.0 + d1), 1.0 / (1.0 + d2)))
        w = np.array(w)
        idx = np.argsort(-w, kind="stable")[: self.c.k_closest]
        ws = w[idx]
        return sum(wi * self.P[i] for wi, i in zip(ws, idx)) / ws.sum()

    def evaluate(self, x1, x2):
        """(D_hat (6), normalised firing phi (n_rules), X). Spawns a rule if none fires."""
        W = self._firing(x1, x2)
        X = np.array([x1, x2, 1.0])
        if not np.any(W > self.c.l_max):
            Pn = self._interpolate(x1, x2)
            if len(self.P) < self.c.max_rules:
                self.A1.append(tuple(x1 + o for o in X1_SHAPE))
                self.A2.append(tuple(x2 + o for o in X2_SHAPE))
                self.P.append(Pn)
                self.n_generated += 1
                W = np.append(W, 1.0)
            else:
                self.n_ephemeral += 1
                D = np.clip(Pn @ X, -self.c.out_max, self.c.out_max)
                return D, np.zeros(len(self.P)), X
        ph = W / W.sum()
        D = np.zeros(6)
        for r in np.nonzero(ph)[0]:
            D += ph[r] * (self.P[r] @ X)
        return np.clip(D, -self.c.out_max, self.c.out_max), ph, X

    def adapt(self, dt, g, ph, X):
        """eq 29: omega_e' = c_e g_e X phi^T, g = M_u^-T eps."""
        if ph.size == 0:
            return
        for r in np.nonzero(ph)[0]:
            self.P[r] = self.P[r] + dt * np.outer(self.c.c * g * ph[r], X)
            self.P[r] = np.clip(self.P[r], -self.c.a_max[:, None], self.c.a_max[:, None])

    @property
    def n_rules(self):
        return len(self.P)


# ---------------------------------------------------------------------------
# 4. the quadrotor law (eqs 25-28 + the decoupling module eq 3)
# ---------------------------------------------------------------------------
def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def euler_zyx(R):
    """roll, pitch, yaw of R = Rz(yaw) Ry(pitch) Rx(roll)."""
    pitch = -math.asin(max(-1.0, min(1.0, R[2, 0])))
    roll = math.atan2(R[2, 1], R[2, 2])
    yaw = math.atan2(R[1, 0], R[0, 0])
    return np.array([roll, pitch, yaw])


def euler_to_r(eta):
    r, p, y = eta
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def euler_w(eta):
    """omega_body = W(eta) eta'  (ZYX)."""
    r, p, _ = eta
    cr, sr, cp, sp = math.cos(r), math.sin(r), math.cos(p), math.sin(p)
    return np.array([[1.0, 0.0, -sp], [0.0, cr, sr * cp], [0.0, -sr, cr * cp]])


def euler_w_dot(eta, eta_dot):
    r, p, _ = eta
    rd, pd = eta_dot[0], eta_dot[1]
    cr, sr, cp, sp = math.cos(r), math.sin(r), math.cos(p), math.sin(p)
    return np.array([[0.0, 0.0, -cp * pd],
                     [0.0, -sr * rd, cr * cp * rd - sr * sp * pd],
                     [0.0, -cr * rd, -sr * cp * rd - cr * sp * pd]])


@dataclass
class QuadConfig:
    mass: float = 3.746170
    inertia: np.ndarray = field(default_factory=lambda: np.array([[0.118, 0.0089, 0.0], [0.0089, 0.119, 0.0],
                                                                  [0.0, 0.0, 0.131]]))
    # per-channel [x, y, z, roll, pitch, yaw] (addition (d))
    eta1: np.ndarray = field(default_factory=lambda: np.array([1.04, 1.04, 1.5, 4.2, 4.2, 3.0]))
    eta2: np.ndarray = field(default_factory=lambda: np.array([1.04, 1.04, 1.5, 4.2, 4.2, 3.0]))
    DT: float = 1.2
    floor: float = 0.6
    ff_gain: float = 1.0              # J_E tau_hat feed-forward (1 = the paper)
    max_pos_err_m: float = 1.0
    max_vel_err_mps: float = 2.0
    max_att_err_rad: float = 0.6
    max_tilt_deg: float = 30.0
    min_thrust_frac: float = 0.3
    max_torque_xy: float = 8.0
    max_torque_z: float = 0.55


@dataclass
class QuadResult:
    thrust: float
    torque: np.ndarray        # body FLU [N.m]
    f_des: np.ndarray
    eta: np.ndarray
    eta_d: np.ndarray
    eta_dot: np.ndarray
    r1: np.ndarray            # 6
    r2: np.ndarray            # 6
    eps: np.ndarray           # 6
    g: np.ndarray             # M_u^-T eps (6), the fuzzy adaptation's signal
    x1: float
    x2: float


def quad_law(c: QuadConfig, win: Window, t, p, v, R, omega, p_d, v_d, a_d, yaw_d, D_hat, F_ext_w, M_ext_b):
    tgo = win.tgo(t)
    sgo = 1.0 if win.shrinking(t) else 0.0
    m = c.mass
    J = c.inertia
    # ---- position rows (world frame, addition (f)) ----
    r1p = np.clip(p_d - p, -c.max_pos_err_m, c.max_pos_err_m)
    r2p = np.clip(v_d - v, -c.max_vel_err_mps, c.max_vel_err_mps)
    e1, e2 = c.eta1[:3], c.eta2[:3]
    deltap = -e1 * phi(r1p) / tgo
    epsp = r2p - deltap
    Np = e1 * (dphi(r1p) * r2p / tgo + sgo * phi(r1p) / tgo ** 2) + e2 * phi(epsp) / tgo
    F = m * (a_d + r1p + Np) + m * G_ACC * E3 - c.ff_gain * F_ext_w + D_hat[:3]
    # safety shaping: thrust floor and tilt clamp of the desired force
    F[2] = max(F[2], c.min_thrust_frac * m * G_ACC)
    if c.max_tilt_deg > 0.0:
        hmax = F[2] * math.tan(math.radians(c.max_tilt_deg))
        hn = math.hypot(F[0], F[1])
        if hn > hmax:
            F[0] *= hmax / hn
            F[1] *= hmax / hn
    f = float(np.linalg.norm(F))
    # ---- the attitude decoupling module (eq 3, ENU / ZYX) ----
    sy, cy = math.sin(yaw_d), math.cos(yaw_d)
    roll_d = math.asin(max(-1.0, min(1.0, (F[0] * sy - F[1] * cy) / f)))
    pitch_d = math.atan2(F[0] * cy + F[1] * sy, F[2])
    eta_d = np.array([roll_d, pitch_d, yaw_d])
    # ---- attitude rows (Euler coordinates) ----
    eta = euler_zyx(R)
    W = euler_w(eta)
    eta_dot = np.linalg.solve(W, omega)
    r1a = np.array([wrap(eta_d[0] - eta[0]), wrap(eta_d[1] - eta[1]), wrap(eta_d[2] - eta[2])])
    r1a = np.clip(r1a, -c.max_att_err_rad, c.max_att_err_rad)
    r2a = -eta_dot                                     # addition (e): eta_d' = 0
    a1, a2 = c.eta1[3:], c.eta2[3:]
    deltaa = -a1 * phi(r1a) / tgo
    epsa = r2a - deltaa
    Na = a1 * (dphi(r1a) * r2a / tgo + sgo * phi(r1a) / tgo ** 2) + a2 * phi(epsa) / tgo
    omega_dot = W @ (r1a + Na) + euler_w_dot(eta, eta_dot) @ eta_dot
    tau = J @ omega_dot + np.cross(omega, J @ omega) - c.ff_gain * M_ext_b + D_hat[3:]
    tau[0] = float(np.clip(tau[0], -c.max_torque_xy, c.max_torque_xy))
    tau[1] = float(np.clip(tau[1], -c.max_torque_xy, c.max_torque_xy))
    tau[2] = float(np.clip(tau[2], -c.max_torque_z, c.max_torque_z))
    r1 = np.concatenate([r1p, r1a])
    r2 = np.concatenate([r2p, r2a])
    eps = np.concatenate([epsp, epsa])
    g = np.concatenate([epsp / m, np.linalg.solve(J, np.linalg.solve(W.T, epsa))])
    return QuadResult(f, tau, F, eta, eta_d, eta_dot, r1, r2, eps, g,
                      float(np.max(np.abs(r1))), float(np.max(np.abs(r2))))


# ---------------------------------------------------------------------------
# 5. the arm's admittance + arbitrary-convergence-time law (eqs 34-38)
# ---------------------------------------------------------------------------
@dataclass
class ArmConfig:
    Hd: np.ndarray = field(default_factory=lambda: np.ones(4))          # paper experiment
    Dd: np.ndarray = field(default_factory=lambda: np.full(4, 4.0))
    Kd: np.ndarray = field(default_factory=lambda: np.full(4, 9.0))
    eta1: np.ndarray = field(default_factory=lambda: np.full(4, 5.3))   # paper experiment
    eta2: np.ndarray = field(default_factory=lambda: np.full(4, 4.8))
    DT: float = 1.5
    floor: float = 0.6
    e_max: float = 0.35               # [rad] clamp on the admittance offset (safety shaping)
    q_min: np.ndarray = field(default_factory=lambda: np.radians([-33.0, -18.0, 5.0, -118.0]))
    q_max: np.ndarray = field(default_factory=lambda: np.radians([33.0, 43.0, 48.0, 118.0]))
    tau_max: float = 3.0


class ArmAdmittance:
    def __init__(self, cfg: ArmConfig):
        self.c = cfg
        self.e = np.zeros(4)
        self.ed = np.zeros(4)
        self.edd = np.zeros(4)

    def reset(self):
        self.e[:] = 0.0
        self.ed[:] = 0.0
        self.edd[:] = 0.0

    def step(self, dt, tau_hat, q_d):
        c = self.c
        self.edd = (tau_hat - c.Dd * self.ed - c.Kd * self.e) / c.Hd
        self.ed = self.ed + dt * self.edd
        self.e = self.e + dt * self.ed
        # keep q_c inside the joint box and |e| <= e_max (safety shaping)
        lo = np.maximum(-c.e_max, c.q_min - q_d)
        hi = np.minimum(c.e_max, c.q_max - q_d)
        lo = np.minimum(lo, hi)
        clipped = (self.e < lo) | (self.e > hi)
        self.e = np.clip(self.e, lo, hi)
        self.ed[clipped] = 0.0
        return self.e, self.ed, self.edd


def arm_law(c: ArmConfig, win: Window, t, q, qd, M, C, G, tau_hat, qc, qcd, qcdd):
    tgo = win.tgo(t)
    sgo = 1.0 if win.shrinking(t) else 0.0
    th1 = qc - q
    th2 = qcd - qd
    j = -c.eta1 * phi(th1) / tgo
    cc = th2 - j
    Nm = c.eta1 * (dphi(th1) * th2 / tgo + sgo * phi(th1) / tgo ** 2) + c.eta2 * phi(cc) / tgo
    tau = C @ qd + G + M @ (qcdd + th1 + Nm) - tau_hat
    return np.clip(tau, -c.tau_max, c.tau_max), th1, th2


# ---------------------------------------------------------------------------
# 6. the interaction wrench on the base (J_E tau_hat, addition (g)) and the
#    planner's flat-output stream converted to a base reference
# ---------------------------------------------------------------------------
def interaction_on_base(q, tau_hat, R_actual, lam=0.05):
    """(F world, M body-FLU) of the estimated joint-space interaction torque."""
    _, r0e, Jv = arm_fk(q, np.zeros(3))
    F_m = np.linalg.solve(Jv @ Jv.T + lam * lam * np.eye(3), Jv @ tau_hat)
    R0m = R_actual @ RZ_M90
    return R0m @ F_m, RZ_M90 @ np.cross(r0e, F_m), F_m


def build_r0(b3, b1d):
    b3 = b3 / np.linalg.norm(b3)
    s = b1d - b3 * float(b3 @ b1d)
    b1 = s / np.linalg.norm(s)
    return np.column_stack([b1, np.cross(b3, b1), b3])


def base_reference(x_cd, x_cd_ddot, b1_d, q_d, base_com, r_ed=None):
    """The base (body origin) position and actual yaw the planner's stream implies:
    R0 = buildR0((x_cd'' + g e3)/|.|, b1_d) (MODEL), x_b = x_cd - R0 r_0c(q_d),
    yaw = atan2(b1_d) + pi/2. Also the arm term R0 r_0c (its derivatives are
    differenced by the caller) and the same-motion residual |r_ed - (x_b + R0 r_0e)|."""
    R0 = build_r0(np.asarray(x_cd_ddot, float) + G_ACC * E3, np.asarray(b1_d, float))
    r0c, r0e, _ = arm_fk(q_d, base_com)
    arm_term = R0 @ r0c
    x_b = np.asarray(x_cd, float) - arm_term
    yaw = math.atan2(b1_d[1], b1_d[0]) + 0.5 * math.pi
    res = float("nan") if r_ed is None else float(np.linalg.norm(np.asarray(r_ed) - (x_b + R0 @ r0e)))
    return x_b, wrap(yaw), arm_term, res


# ---------------------------------------------------------------------------
# 7. self-test
# ---------------------------------------------------------------------------
def _selftest():
    ok = True

    def check(name, cond, detail=""):
        nonlocal ok
        ok &= bool(cond)
        print(f"  [{'PASS' if cond else 'FAIL'}] {name} {detail}")

    print("arbitrary_time_control.py self-test")
    # (1) the arm model = the planner's (fsc_trajectory_planner fixture home case)
    qh = np.array([0.0, 0.6981317007977318, 0.6981317007977318, 0.0])
    r0c, r0e, _ = arm_fk(qh, np.zeros(3))
    err = max(np.abs(r0c - [8.576915154685956e-05, 0.019495814648010003, -0.014488211489149383]).max(),
              np.abs(r0e - [0.0015650000000000004, 0.2528986252301309, -0.06068932796643191]).max(),
              abs(total_mass() - 3.7461737699999995))
    check("arm model = planner fixture home case", err < 1e-12, f"({err:.1e})")
    # (2) dynamics properties
    rng = np.random.default_rng(1)
    q = np.array([0.2, 0.5, 0.6, -0.4])
    qd = np.array([0.3, -0.2, 0.4, 0.5])
    M = arm_mass_matrix(q)
    check("M symmetric positive definite", np.allclose(M, M.T) and np.all(np.linalg.eigvalsh(M) > 0))
    h = 1e-6
    Mdot = (arm_mass_matrix(q + h * qd) - arm_mass_matrix(q - h * qd)) / (2 * h)
    S = Mdot - 2.0 * arm_coriolis(q, qd)
    check("M' - 2C skew-symmetric", np.abs(S + S.T).max() < 1e-7, f"({np.abs(S + S.T).max():.1e})")
    gb = euler_to_r([0.1, -0.15, 0.4]).T @ np.array([0.0, 0.0, -G_ACC])

    def U(qq):
        k = arm_kinematics(qq)
        return -sum(LINK_MASS[i] * float(gb @ k.com[i]) for i in range(4))
    Gfd = np.array([(U(q + h * np.eye(4)[i]) - U(q - h * np.eye(4)[i])) / (2 * h) for i in range(4)])
    check("G = dU/dq", np.abs(Gfd - arm_gravity(q, gb)).max() < 1e-7)
    # (3) ACTNDO on a simulated fixed-base arm with a step interaction torque at t = 1 s
    dt = 0.004
    obs = Actndo(ActndoConfig())
    q = np.array([0.0, 0.55, 0.5, 0.0])
    qd = np.zeros(4)
    tau_true = np.array([0.08, -0.25, 0.15, 0.02])
    g0 = np.array([0.0, 0.0, -G_ACC])
    obs.reset(arm_mass_matrix(q) @ qd)
    obs.trigger(0.0)
    errs = []
    for k in range(int(4.0 / dt)):
        t = k * dt
        if abs(t - 1.0) < 1e-9:
            obs.trigger(t)
        te = tau_true if t >= 1.0 else np.zeros(4)
        M, C, G = arm_dynamics(q, qd, g0)
        tau_m = G + M @ (-6.0 * qd - 25.0 * (q - np.array([0.0, 0.55, 0.5, 0.0])))   # a stiff PD to keep it near
        H = M @ qd
        obs.update(dt, t, H, C.T @ qd, G, tau_m)
        qdd = np.linalg.solve(M, tau_m + te - C @ qd - G)
        qd = qd + dt * qdd
        q = q + dt * qd
        errs.append(np.abs(obs.est - te).max())
    check("ACTNDO converges to a step tau_ext", errs[-1] < 2e-3 and max(errs[int(2.2 / dt):]) < 0.02,
          f"(final {errs[-1]:.1e}, after 2.2 s max {max(errs[int(2.2 / dt):]):.1e})")
    # (4) the quadrotor law on a rigid-body plant with an unknown constant force + torque
    qc = QuadConfig()
    fz = TskFuzzyEstimator(FuzzyConfig())
    win = Window(qc.DT, qc.floor)
    win.trigger(0.0)
    p = np.array([0.2, -0.1, 1.0])
    v = np.zeros(3)
    R = euler_to_r([0.02, -0.03, 0.3])
    om = np.zeros(3)
    d_f = np.array([0.5, -0.4, -1.5])
    d_t = np.array([0.05, -0.08, 0.03])
    m, J = qc.mass, qc.inertia
    for k in range(int(12.0 / dt)):
        t = k * dt
        D, ph, X = fz.evaluate(*_xs_last) if k > 0 else fz.evaluate(0.0, 0.0)
        res = quad_law(qc, win, t, p, v, R, om, np.zeros(3) + [0.0, 0.0, 1.0], np.zeros(3), np.zeros(3), 0.0,
                       D, np.zeros(3), np.zeros(3))
        fz.adapt(dt, res.g, ph, X)
        _xs_last = (res.x1, res.x2)
        F = res.thrust * R[:, 2] - m * G_ACC * E3 + d_f
        v = v + dt * F / m
        p = p + dt * v
        omd = np.linalg.solve(J, res.torque + d_t - np.cross(om, J @ om))
        om = om + dt * omd
        R = R @ axis_rot(om / (np.linalg.norm(om) + 1e-15), np.linalg.norm(om) * dt)
    check("quadrotor law + TSKFE reject a constant wrench",
          np.linalg.norm(p - [0.0, 0.0, 1.0]) < 0.01 and abs(euler_zyx(R)[2]) < 0.01,
          f"(|e_p| {np.linalg.norm(p - [0, 0, 1.0]) * 1e3:.1f} mm, yaw {math.degrees(euler_zyx(R)[2]):.2f} deg, "
          f"{fz.n_rules} rules, {fz.n_generated} generated)")
    # (5) the arm: admittance deflection = Kd^-1 tau_ext in steady state, observer in the loop
    ac = ArmConfig()
    adm = ArmAdmittance(ac)
    win_m = Window(ac.DT, ac.floor)
    win_m.trigger(0.0)
    obs = Actndo(ActndoConfig())
    q_d = np.array([0.0, 0.55, 0.5, 0.0])
    q = q_d.copy()
    qd = np.zeros(4)
    obs.reset(np.zeros(4))
    obs.trigger(0.0)
    tau_true = np.array([0.05, -0.20, 0.10, 0.0])
    for k in range(int(10.0 / dt)):
        t = k * dt
        M, C, G = arm_dynamics(q, qd, g0)
        e, ed, edd = adm.step(dt, obs.est, q_d)
        tau, _, _ = arm_law(ac, win_m, t, q, qd, M, C, G, obs.est, q_d + e, ed, edd)
        obs.update(dt, t, M @ qd, C.T @ qd, G, tau)
        qdd = np.linalg.solve(M, tau + tau_true - C @ qd - G)
        qd = qd + dt * qdd
        q = q + dt * qd
    defl = q - q_d
    check("arm settles at the admittance deflection Kd^-1 tau_ext",
          np.abs(defl - tau_true / ac.Kd).max() < 1e-3, f"({np.round(np.degrees(defl), 3)} deg)")
    # (6) the base reference reproduces the planner's EE reference (same-motion check)
    R0 = build_r0(np.array([0.1, -0.2, G_ACC]), np.array([0.0, 1.0, 0.0]))
    q1 = np.array([0.0, 0.558, 0.489, 0.0])
    bc = np.array([0.0, -0.017854, 0.0])
    x_b = np.array([0.3, 0.4, 1.2])
    r0c, r0e, _ = arm_fk(q1, bc)
    x_cd = x_b + R0 @ r0c
    xb2, yaw, _, res = base_reference(x_cd, np.array([0.1, -0.2, 0.0]), R0[:, 0], q1, bc, x_b + R0 @ r0e)
    check("base reference inverts the CoM chain", np.abs(xb2 - x_b).max() < 1e-12 and res < 1e-12)
    print("ALL PASS" if ok else "FAILURES")
    return ok


if __name__ == "__main__":
    import sys
    sys.exit(0 if _selftest() else 1)
