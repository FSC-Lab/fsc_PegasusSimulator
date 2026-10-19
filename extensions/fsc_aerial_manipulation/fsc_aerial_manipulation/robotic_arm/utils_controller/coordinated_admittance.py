#!/usr/bin/env python3
"""coordinated_admittance.py -- the COORDINATED ADMITTANCE framework of

    Q. Liu, S. Lyu, K. Guo, J. Wang, X. Yu, L. Guo, "A coordinated framework
    of aerial manipulator for safe and compliant physical interaction",
    Control Engineering Practice 146 (2024) 105898
    (the "[passivity] A coordinated framework of aerial.pdf" reference),

written for the SIMULATION comparison of the box-pushing task against the
whole-body L1 impedance law (2026-10-10). Pure numpy. It is the source of
truth for TWO consumers:

  * the C++ flight node `autopilot_coordinated_admittance_direct_actuation_node`
    (fsc_autopilot_ros2, single_aerial_manipulator_coordinated_admittance_
    direct_actuation) ports Sections 1 and 2 below (the UAV law and the
    momentum estimator, 250 Hz) and is held to THIS file by a parity fixture
    (generate_coordinated_truth.py);
  * the ROS node `coordinated_admittance_planner.py` (fsc_autopilot_ros2
    scripts/tools) IMPORTS Sections 3-5 (admittance, adaptive weights, the
    kinematic MPC and its QP solver) and runs them as the paper's onboard
    "coordinated planner" (paper: 10 Hz on a mini PC).

THE METHOD (paper sections 3-4), decoupled by construction:

  1. UAV controller (eqs 11-14). Position PD in the inertial frame with the
     desired acceleration as feedforward,
         nu   = -K_Pp E_P - K_Pd E_P_dot + P_ddot_d
         f_t R e3 = m_s nu + m_s g e3                 (ENU; paper NED eq 11)
     f_t = |F_des|, b3d = F_des/|F_des| (eq 12), b2d = b3d x b1d / |.| (eq 13)
     with b1d the desired heading; a PD attitude law in roll-pitch-yaw EULER
     coordinates (eq 14),
         tau_eta = -K_eta_p E_eta - K_eta_d E_eta_dot (+ M_eta eta_ddot_d
                   + C_eta eta_dot_d, zero for the constant-attitude
                   reference this node is given),
     mapped to a body torque through the Euler-rate kinematics omega = W eta_dot,
     tau_body = W^-T tau_eta. The arm is NOT in the UAV model (paper Remark 3:
     a low-mass arm, quasi-static; its coupling is "implicitly addressed" by
     the planner's CoM term). The servos run position mode (paper: built-in PD).
  2. Momentum-based external generalized force estimator (eq 25, Algorithm 1),
     second order with gains K1, K2 on chi = [m_s P_dot; M_eta eta_dot]:
         z     = chi - int (u + r_hat) dt
         r_hat = K1 int (K2 z - r_hat) dt
     i.e. r_hat = (K1 K2)/(s^2 + K1 s + K1 K2) r, omega_n = sqrt(K1 K2),
     zeta = sqrt(K1/K2)/2 (the paper's k1/k2 -> 4 is critical damping). The
     applied generalized force u = [f_t R e3 - m_s g e3; dT/d eta + tau_eta]
     (dT/d eta = C_eta^T eta_dot, the paper's rotational term). SIGN NOTE:
     eq (25) prints the inner integrand as ([u] - r_hat); the observer only
     reduces the lumped force if the bracket is the NEGATIVE of the applied
     generalized force (Ruggiero et al. 2015, from which the paper takes it).
     Implemented in that consistent form; the self-test checks it converges
     to an injected force on a simulated plant.
  3. Admittance filter on the end-effector reference (eq 27),
         M_d E_ddot + D_d E_dot + K_d E = -F_hat_ext,   P_r = P_o - E,
     per axis; K_d -> 0 is full compliance, large K_d is the stiff tracking
     a push task needs (paper Table 1: push-pull used 0.5/2.0/1.0).
  4. Adaptive weights (eqs 31-32): w_uav = (1 - tanh((|F_hat| - F_max/2) a))/2
     (read as the convex form so that w_uav in [0,1] and the joint weights
     (1 - w_uav) are non-negative, which Fig. 3 and the text require);
     W_am = [|F_j|/|F| w_uav (j = x,y,z), (1 - w_uav) sum_{j<=n-i} m_j / sum m
     (i = 1..4)] applied as the INPUT weight W_u of the planner: a large weight
     penalises that DoF's acceleration, so small force = the arm moves, large
     force = the UAV base moves (the paper's soft/hard stiffness switch).
  5. Kinematic MPC planner (eqs 28-39): state h = [P_b; q] (7), double
     integrator x = [h; h_dot], u = h_ddot, B = [0; dt I] (eq 29 literally);
     cost sum ||W_e E_ter||^2 + sum ||W_c P_c^B||^2 + sum ||W_u u||^2 with
     E_ter = K_p E_e + E_e_dot (eq 37) on E_e = P_e^I - P_r; constraints
     (33)-(36): u, du, x boxes, the EE in a body-frame box, |P_c^B| <= l_m.
     The EE and CoM maps P_e^I = P_b + R_b P_e^B(q), P_c^B(q) are linearised
     about the measured q every solve (one Jacobian per tick), which is what
     makes it the paper's "MPC at the kinematic level" (Remark 7) solvable as
     ONE QP per tick. Solved by an OSQP-style ADMM written here (no third-party
     QP package on the flight computer), warm-started.

TASK-DRIVEN ADDITIONS, each stated so a log can say which way a run went:
  (a) EE HEADING. The paper plans the EE POSITION only. The box handle needs
      the claw's heading held, so the planner carries one extra quadratic
      term on q1 (psi_e = psi_b + q1 exactly while the wrist roll q4 = 0, the
      push task's case) and tracks q4 to the streamed reference. Both weights
      are config; zero recovers the paper.
  (b) THE NORM CONSTRAINTS as linear boxes: |P_c^B| <= l_m is enforced per
      horizontal axis as |P_c,xy| <= l_m/sqrt(2) (an inner approximation), and
      the terminal cone ||E_ter|| <= eps is kept as the COST (a hard version
      is infeasible after any step). Both are documented inner/relaxed forms
      of (38)-(39), not the paper's second-order cones.
  (c) The desired attitude's angular-rate feedforward (eta_dot_d, eta_ddot_d
      from Garofalo et al.) is zero: the base reference is a slowly varying
      planner output, the same constant-reference specialisation the other
      decoupled rig uses.

FRAMES. ENU world, FLU body (x = the arm side); joint angles in the model
convention (identity in Isaac). The planner's arm model (fsc_trajectory_planner
armKinematics / WholeBodyParams::t650Defaults, the one the decoupled bridge
embeds) gives P_e^B and P_c^B in the MODEL base frame (y = the arm side); the
one conversion is v_actual = Rz(-90) v_model.

    python3 coordinated_admittance.py      # self-test
"""
from __future__ import annotations

import math
import os
import sys
from dataclasses import dataclass, field

import numpy as np

G = 9.80665     # fsc::numbers::std_gravity -- the SAFETY robust controller's g
E3 = np.array([0.0, 0.0, 1.0])
SQRT2 = math.sqrt(2.0)


# =============================================================================
# Rotations and Euler-angle kinematics (shared by the law and the estimator)
# =============================================================================
def hat(v):
    return np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])


def rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def euler_zyx(R):
    """roll, pitch, yaw of R = Rz(psi) Ry(theta) Rx(phi) (paper eq 1)."""
    s = -R[2, 0]
    s = max(-1.0, min(1.0, s))
    theta = math.asin(s)
    phi = math.atan2(R[2, 1], R[2, 2])
    psi = math.atan2(R[1, 0], R[0, 0])
    return np.array([phi, theta, psi])


def euler_to_r(eta):
    phi, theta, psi = eta
    cphi, sphi = math.cos(phi), math.sin(phi)
    cth, sth = math.cos(theta), math.sin(theta)
    cps, sps = math.cos(psi), math.sin(psi)
    return np.array([[cps * cth, cps * sth * sphi - sps * cphi, cps * sth * cphi + sps * sphi],
                     [sps * cth, sps * sth * sphi + cps * cphi, sps * sth * cphi - cps * sphi],
                     [-sth, cth * sphi, cth * cphi]])


def euler_w(eta):
    """W(eta): omega_body = W eta_dot (paper eq 2's matrix)."""
    phi, theta = eta[0], eta[1]
    cphi, sphi = math.cos(phi), math.sin(phi)
    cth, sth = math.cos(theta), math.sin(theta)
    return np.array([[1.0, 0.0, -sth],
                     [0.0, cphi, sphi * cth],
                     [0.0, -sphi, cphi * cth]])


def euler_w_grad(eta):
    """dW/dphi, dW/dtheta (dW/dpsi = 0)."""
    phi, theta = eta[0], eta[1]
    cphi, sphi = math.cos(phi), math.sin(phi)
    cth, sth = math.cos(theta), math.sin(theta)
    w_phi = np.array([[0.0, 0.0, 0.0],
                      [0.0, -sphi, cphi * cth],
                      [0.0, -cphi, -sphi * cth]])
    w_th = np.array([[0.0, 0.0, -cth],
                     [0.0, 0.0, -sphi * sth],
                     [0.0, 0.0, -cphi * sth]])
    return w_phi, w_th


def kinetic_energy_gradient(eta, eta_dot, inertia):
    """dT/d eta with T = 1/2 eta_dot^T M_eta eta_dot, M_eta = W^T I W.
    Equals C_eta^T eta_dot (the rotational term of the paper's eq 24/25)."""
    W = euler_w(eta)
    w_phi, w_th = euler_w_grad(eta)
    iw = inertia @ W @ eta_dot
    return np.array([eta_dot @ (w_phi.T @ iw), eta_dot @ (w_th.T @ iw), 0.0])


# =============================================================================
# 1. UAV controller (paper eqs 11-14)
# =============================================================================
@dataclass
class FlightLawConfig:
    mass: float = 3.746170                     # m_s, the whole UAM
    kpp: np.ndarray = field(default_factory=lambda: np.array([5.0, 5.0, 4.0]))   # K_Pp [1/s^2]
    kpd: np.ndarray = field(default_factory=lambda: np.array([3.0, 3.0, 3.0]))   # K_Pd [1/s]
    # ADDITION (e): integral action on the UAV position error, nu += -K_Pi int e_p.
    # 0 = the paper's PD (eq 11). Needed on this plant: the mirror's ~1.4 N
    # thrust-model error parks the paper's PD ~8 cm low, and the kinematic
    # planner cannot absorb that (the arm's upward reach at the push pose is
    # ~3 cm; measured offline, coordinated_admittance.py 2026-10-10). The
    # integral is clamped (int_max, m.s) and the caller integrates it.
    kpi: np.ndarray = field(default_factory=lambda: np.zeros(3))                 # K_Pi [1/s^3]
    int_max: float = 0.3
    keta_p: np.ndarray = field(default_factory=lambda: np.array([3.3, 3.3, 1.7]))  # [N.m/rad]
    keta_d: np.ndarray = field(default_factory=lambda: np.array([0.95, 0.95, 0.41]))  # [N.m/(rad/s)]
    # ADDITION (j): integral action on the Euler attitude error,
    # tau_eta += -K_eta_i int e_eta. 0 = the paper's PD (eq 14). The paper's
    # UAV controller carries no arm model (Remark 3) and no integral, so the
    # arm's standing moment (~0.2 N.m pitch at the folded home on this plant,
    # plus the plant's -0.1 N.m yaw bias) parks the PD at a steady Euler
    # error tau/K_eta_p -- measured 3 deg pitch / 4 deg yaw in every hover
    # (cadm_06..08, 2026-10-10), i.e. the claw heading 4 deg off the fin and
    # the vehicle starting every DIRECT entry with a 0.3 m/s surge.
    keta_i: np.ndarray = field(default_factory=lambda: np.zeros(3))              # [N.m/(rad.s)]
    int_eta_max: float = 0.2                                                     # [rad.s]
    inertia: np.ndarray = field(default_factory=lambda: np.diag([0.118, 0.119, 0.131]))
    max_pos_err_m: float = 1.0      # 0 = off
    max_vel_err_mps: float = 2.0
    max_tilt_deg: float = 30.0
    min_thrust_frac: float = 0.3    # vertical floor on F_des, fraction of weight
    max_torque_xy: float = 8.0
    max_torque_z: float = 2.0

    def __post_init__(self):
        for k in ("kpp", "kpd", "kpi", "keta_p", "keta_d", "keta_i"):
            setattr(self, k, np.asarray(getattr(self, k), float))
        self.inertia = np.asarray(self.inertia, float)


def _sat(v, b):
    return v if b <= 0.0 else np.clip(v, -b, b)


def flight_law(cfg: FlightLawConfig, p, v, R, omega, p_d, v_d, a_d, yaw_d, e_int=None, e_eta_int=None,
               tau_ff=None):
    """One tick of the paper's UAV controller. Returns a dict with f_t [N],
    tau_body [N.m, FLU], R_des, the Euler errors and the generalized torque
    tau_eta the estimator must be fed. e_int = the integrated position error
    (addition (e)), e_eta_int = the integrated Euler attitude error (addition
    (j)), tau_ff = a generalized (Euler) disturbance torque to cancel, addition
    (n): the paper's own estimate (eq 25) fed back, tau_eta -= tau_ff; None/
    zeros = the paper's PD."""
    m = cfg.mass
    e_p = _sat(np.asarray(p, float) - np.asarray(p_d, float), cfg.max_pos_err_m)
    e_v = _sat(np.asarray(v, float) - np.asarray(v_d, float), cfg.max_vel_err_mps)
    e_i = np.zeros(3) if e_int is None else np.clip(np.asarray(e_int, float), -cfg.int_max, cfg.int_max)
    nu = -cfg.kpp * e_p - cfg.kpd * e_v - cfg.kpi * e_i + np.asarray(a_d, float)   # eq 11 (+ (e))
    f_des = m * nu + m * G * E3
    # safety shaping (the sibling nodes' tilt clamp): never ask for an
    # inverted or horizontal thrust vector
    min_fz = cfg.min_thrust_frac * m * G
    if f_des[2] < min_fz:
        f_des[2] = min_fz
    if cfg.max_tilt_deg > 0.0:
        max_lat = math.tan(math.radians(cfg.max_tilt_deg)) * f_des[2]
        lat = float(np.hypot(f_des[0], f_des[1]))
        if lat > max_lat:
            f_des[:2] *= max_lat / lat
    f_t = float(np.linalg.norm(f_des))                                    # eq 12
    b3d = f_des / f_t
    b1c = np.array([math.cos(yaw_d), math.sin(yaw_d), 0.0])
    b2d = np.cross(b3d, b1c)
    n2 = float(np.linalg.norm(b2d))
    if n2 < 1e-6:
        raise ValueError("desired thrust parallel to the heading")
    b2d /= n2                                                             # eq 13
    b1d = np.cross(b2d, b3d)
    R_des = np.column_stack([b1d, b2d, b3d])
    # eq 14 in Euler coordinates
    eta = euler_zyx(np.asarray(R, float))
    eta_d = euler_zyx(R_des)
    e_eta = np.array([wrap(eta[i] - eta_d[i]) for i in range(3)])
    W = euler_w(eta)
    eta_dot = np.linalg.solve(W, np.asarray(omega, float))
    e_eta_dot = eta_dot                                                   # eta_dot_d = 0
    e_eta_i = (np.zeros(3) if e_eta_int is None
               else np.clip(np.asarray(e_eta_int, float), -cfg.int_eta_max, cfg.int_eta_max))
    tau_eta = -cfg.keta_p * e_eta - cfg.keta_d * e_eta_dot - cfg.keta_i * e_eta_i   # eq 14 (+ (j))
    if tau_ff is not None:
        tau_eta = tau_eta - np.asarray(tau_ff, float)                     # addition (n)
    tau_body = np.linalg.solve(W.T, tau_eta)
    # per-axis clamps (actuator bookkeeping, not the law)
    if cfg.max_torque_xy > 0.0:
        tau_body[0] = min(max(tau_body[0], -cfg.max_torque_xy), cfg.max_torque_xy)
        tau_body[1] = min(max(tau_body[1], -cfg.max_torque_xy), cfg.max_torque_xy)
    if cfg.max_torque_z > 0.0:
        tau_body[2] = min(max(tau_body[2], -cfg.max_torque_z), cfg.max_torque_z)
    return dict(f_t=f_t, tau_body=tau_body, R_des=R_des, e_p=e_p, e_v=e_v,
                eta=eta, eta_d=eta_d, e_eta=e_eta, eta_dot=eta_dot, e_eta_i=e_eta_i, e_i=e_i,
                tau_eta=W.T @ tau_body, f_des=f_des)


# =============================================================================
# 2. Momentum-based external generalized force estimator (eq 25, Algorithm 1)
# =============================================================================
class MomentumEstimator:
    def __init__(self, mass, inertia, k1, k2):
        self.m = float(mass)
        self.I = np.asarray(inertia, float)
        self.k1 = np.asarray(k1, float) * np.ones(6)
        self.k2 = np.asarray(k2, float) * np.ones(6)
        self.reset(np.zeros(6))

    def momentum(self, v, eta, eta_dot):
        W = euler_w(eta)
        m_eta = W.T @ self.I @ W
        return np.concatenate([self.m * np.asarray(v, float), m_eta @ np.asarray(eta_dot, float)])

    def reset(self, chi0):
        """Start with r_hat = 0 and a ZERO momentum residual (no start-up
        transient): b = chi(0)."""
        self.b = np.asarray(chi0, float).copy()
        self.c = np.zeros(6)
        self.r_hat = np.zeros(6)

    def applied_force(self, f_t, R, eta, eta_dot, tau_eta):
        """u = [f_t R e3 - m g e3 ; dT/d eta + tau_eta] (world force, Euler torque)."""
        u_t = f_t * (np.asarray(R, float) @ E3) - self.m * G * E3
        u_r = kinetic_energy_gradient(eta, eta_dot, self.I) + np.asarray(tau_eta, float)
        return np.concatenate([u_t, u_r])

    def update(self, u, chi, dt):
        """Algorithm 1 (explicit Euler, as the paper runs it). Returns r_hat."""
        self.b = self.b + (u + self.r_hat) * dt
        z = chi - self.b
        self.c = self.c + (self.k2 * z - self.r_hat) * dt
        self.r_hat = self.k1 * self.c
        return self.r_hat.copy()

    @property
    def force(self):
        return self.r_hat[:3]

    @property
    def torque(self):
        return self.r_hat[3:]


# =============================================================================
# 3. Admittance filter (eq 27)
# =============================================================================
class AdmittanceFilter:
    def __init__(self, m_d, d_d, k_d, e_max=0.0):
        self.M = np.asarray(m_d, float) * np.ones(3)
        self.D = np.asarray(d_d, float) * np.ones(3)
        self.K = np.asarray(k_d, float) * np.ones(3)
        self.e_max = float(e_max)
        self.reset()

    def reset(self):
        self.e = np.zeros(3)
        self.e_dot = np.zeros(3)

    def _acc(self, e, e_dot, f):
        return (-f - self.D * e_dot - self.K * e) / self.M

    def update(self, f_hat, p_o, v_o, a_o, dt, substeps=4):
        """Integrate M e_ddot + D e_dot + K e = -F_hat over dt (F_hat held;
        RK4). Returns P_r, P_r_dot, P_r_ddot."""
        f = np.asarray(f_hat, float)
        h = dt / substeps
        e, ed = self.e.copy(), self.e_dot.copy()
        for _ in range(substeps):
            k1e, k1d = ed, self._acc(e, ed, f)
            k2e, k2d = ed + 0.5 * h * k1d, self._acc(e + 0.5 * h * k1e, ed + 0.5 * h * k1d, f)
            k3e, k3d = ed + 0.5 * h * k2d, self._acc(e + 0.5 * h * k2e, ed + 0.5 * h * k2d, f)
            k4e, k4d = ed + h * k3d, self._acc(e + h * k3e, ed + h * k3d, f)
            e = e + h / 6.0 * (k1e + 2 * k2e + 2 * k3e + k4e)
            ed = ed + h / 6.0 * (k1d + 2 * k2d + 2 * k3d + k4d)
        if self.e_max > 0.0:
            n = float(np.linalg.norm(e))
            if n > self.e_max:
                e *= self.e_max / n
                ed[:] = 0.0
        self.e, self.e_dot = e, ed
        e_dd = self._acc(e, ed, f)
        return (np.asarray(p_o, float) - e, np.asarray(v_o, float) - ed,
                np.asarray(a_o, float) - e_dd)


# =============================================================================
# 4. Adaptive weights (eqs 31-32)
# =============================================================================
def uav_weight(f_norm, f_max, alpha):
    return 0.5 * (1.0 - math.tanh((f_norm - 0.5 * f_max) * alpha))


def weight_vector(f_hat, f_max, alpha, link_masses, eps=1e-6, xy_only=False):
    """W_am = [w_x w_y w_z w_q1 .. w_q4] (eq 32). link_masses = m_1..m_n of
    the arm links; w_qi = (1 - w_uav) * sum_{j=1}^{n-i} m_j / sum_k m_k.
    xy_only (task-driven, 2026-10-10): the estimate's VERTICAL channel on this
    plant is the thrust-model error (kf mismatch + battery sag, ~1.5 N), not an
    interaction force, and it flipped the weights at every hover; the push is
    horizontal, so the switch reads the horizontal force only."""
    f = np.asarray(f_hat, float).copy()
    if xy_only:
        f[2] = 0.0
    fn = float(np.linalg.norm(f))
    w_uav = uav_weight(fn, f_max, alpha)
    if fn > eps:
        w_t = np.abs(f) / fn * w_uav
    else:
        w_t = np.full(3, w_uav)
    m = np.asarray(link_masses, float)
    n = len(m)
    tot = float(m.sum())
    w_q = np.array([(1.0 - w_uav) * m[: n - i].sum() / tot for i in range(1, n + 1)])
    return np.concatenate([w_t, w_q]), w_uav


# =============================================================================
# 5. The arm model (MODEL frame) -- the planner's armKinematics
# =============================================================================
# WholeBodyParams::t650Defaults restricted to what P_e^B / P_c^B need; the
# same copy decoupled_reference_bridge.py carries, checked against the
# planner's fixture in the self-test.
ARM_M = np.array([2.95 + 0.15955, 0.1048326, 0.1423463, 0.13992727, 0.2495176])
ARM_H = np.array([[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
_RAW_L = np.array([[+0.000500, +0.002000, -0.042000],
                   [-0.019000, +0.000070, -0.040500],
                   [+0.000089, +0.024000, -0.128000],
                   [+0.019976, +0.128927, -0.021750]])
_RAW_COM = np.array([[+0.000960, +0.001520, -0.073027],
                     [+0.000320, +0.010163, -0.137795],
                     [+0.019480, +0.107164, -0.128866],
                     [+0.018278, +0.123460, -0.061369]])
_GRIPPER_OFF_WRIST = np.array([0.0, 0.0, -0.108])
R_MODEL_TO_ACTUAL = rz(-0.5 * math.pi)     # v_actual = Rz(-90) v_model


def _chain():
    o_raw = np.vstack([np.zeros(3), np.cumsum(_RAW_L, axis=0)])
    link_com = o_raw[:4] + _RAW_COM
    o_chain = np.vstack([np.zeros(3), o_raw[2], o_raw[3], o_raw[4], o_raw[4] + _GRIPPER_OFF_WRIST])
    return np.diff(o_chain, axis=0), link_com - o_chain[:4]


ARM_L, ARM_COM = _chain()
# joint box: the validated OM-X working range (transition_planner.Q_MIN/Q_MAX),
# q2 narrowed to the real arm's [-20, 45] deg (the push scene's hard stops)
Q_MIN = np.array([-0.610865, -0.349066, -0.698132, -2.0943951])
Q_MAX = np.array([0.610865, 0.785398, 0.872665, 2.0943951])


def joint_rotation(h, q):
    a = h / (np.linalg.norm(h) + 1e-15)
    K = hat(a)
    return np.eye(3) + math.sin(q) * K + (1.0 - math.cos(q)) * (K @ K)


def arm_fk_model(q, base_com=None):
    """(P_c^B, P_e^B) in the MODEL base frame (y = arm side)."""
    R = np.eye(3)
    o = np.zeros(3)
    acc = ARM_M[0] * (np.zeros(3) if base_com is None else np.asarray(base_com, float))
    for i in range(4):
        R = R @ joint_rotation(ARM_H[i], q[i])
        acc = acc + ARM_M[i + 1] * (o + R @ ARM_COM[i])
        o = o + R @ ARM_L[i]
    return acc / ARM_M.sum(), o


def arm_fk_actual(q, base_com=None):
    """(P_c^B, P_e^B) in the ACTUAL FLU body frame (x = arm side)."""
    c, e = arm_fk_model(q, base_com)
    return R_MODEL_TO_ACTUAL @ c, R_MODEL_TO_ACTUAL @ e


def arm_jacobians(q, base_com=None, eps=1e-6):
    """P_c^B, J_c, P_e^B, J_e (actual frame) by central differences."""
    q = np.asarray(q, float)
    c0, e0 = arm_fk_actual(q, base_com)
    Jc = np.zeros((3, 4))
    Je = np.zeros((3, 4))
    for k in range(4):
        d = np.zeros(4)
        d[k] = eps
        cp, ep = arm_fk_actual(q + d, base_com)
        cm, em = arm_fk_actual(q - d, base_com)
        Jc[:, k] = (cp - cm) / (2.0 * eps)
        Je[:, k] = (ep - em) / (2.0 * eps)
    return c0, Jc, e0, Je


# =============================================================================
# 6. A small OSQP-style ADMM QP solver:  min 1/2 x'Px + q'x  s.t. l <= Ax <= u
# =============================================================================
class AdmmQp:
    def __init__(self, rho=1.0, sigma=1e-6, max_iter=400, eps_abs=1e-5, eps_rel=1e-5,
                 adaptive_rho=True, conv_prim=1e-3, conv_dual=1e-1):
        self.rho, self.sigma = float(rho), float(sigma)
        self.max_iter = int(max_iter)
        self.eps_abs, self.eps_rel = float(eps_abs), float(eps_rel)
        self.adaptive_rho = bool(adaptive_rho)
        # "converged" for the CONSUMER: a solution whose constraint violation
        # (r_prim, in the constraint units -- m/s^2, m, rad) is under
        # conv_prim is usable; one that is not must not be applied
        self.conv_prim, self.conv_dual = float(conv_prim), float(conv_dual)
        self.ruiz_iters = 10

    def solve(self, P, q, A, l, u, x0=None, z0=None, y0=None):
        """min 1/2 x'Px + q'x  s.t. l <= Ax <= u, by ADMM on a Ruiz-equilibrated
        copy (OSQP's scheme: the slack columns of the MPC touch hundreds of
        rows while the input columns touch a few, and un-scaled ADMM crawls on
        that). Residuals are judged in the ORIGINAL units."""
        n, m = P.shape[0], A.shape[0]
        P = np.asarray(P, float)
        q = np.asarray(q, float)
        A = np.asarray(A, float)
        l = np.asarray(l, float)
        u = np.asarray(u, float)
        # ---- Ruiz equilibration: P <- D P D, q <- D q, A <- E A D ------------
        D = np.ones(n)
        E = np.ones(m)
        Ps, qs, As = P.copy(), q.copy(), A.copy()
        for _ in range(self.ruiz_iters):
            cn = np.maximum(np.max(np.abs(Ps), axis=0) if n else 0.0,
                            np.max(np.abs(As), axis=0) if m else 0.0)
            dj = 1.0 / np.sqrt(np.where(cn > 1e-12, cn, 1.0))
            rn = np.max(np.abs(As), axis=1) if m else np.ones(0)
            ei = 1.0 / np.sqrt(np.where(rn > 1e-12, rn, 1.0))
            Ps = Ps * dj[:, None] * dj[None, :]
            qs = qs * dj
            As = As * ei[:, None] * dj[None, :]
            D *= dj
            E *= ei
        ls, us = l * E, u * E
        x = np.zeros(n) if x0 is None else np.asarray(x0, float) / D
        z = (As @ x if z0 is None else np.asarray(z0, float) * E)
        y = np.zeros(m) if y0 is None else np.asarray(y0, float) / E
        rho = self.rho
        AtA = As.T @ As
        K = Ps + self.sigma * np.eye(n) + rho * AtA
        L = np.linalg.cholesky(K)
        it = 0
        r_prim = r_dual = np.inf
        Pq_scale = np.max(np.abs(q)) if n else 0.0
        for it in range(1, self.max_iter + 1):
            rhs = self.sigma * x - qs + As.T @ (rho * z - y)
            x = np.linalg.solve(L.T, np.linalg.solve(L, rhs))
            ax = As @ x
            z = np.clip(ax + y / rho, ls, us)
            y = y + rho * (ax - z)
            if it % 10 == 0 or it == self.max_iter:
                # residuals in the original units
                r_prim = float(np.max(np.abs((ax - z) / E))) if m else 0.0
                dual_vec = (Ps @ x + qs + As.T @ y) / D
                r_dual = float(np.max(np.abs(dual_vec)))
                ep = self.eps_abs + self.eps_rel * max(float(np.max(np.abs(ax / E))) if m else 0.0,
                                                       float(np.max(np.abs(z / E))) if m else 0.0)
                ed = self.eps_abs + self.eps_rel * max(float(np.max(np.abs((Ps @ x) / D))),
                                                       float(np.max(np.abs((As.T @ y) / D))) if m else 0.0,
                                                       Pq_scale)
                if r_prim <= ep and r_dual <= ed:
                    break
                if self.adaptive_rho and it % 50 == 0:
                    ratio = math.sqrt((r_prim / max(ep, 1e-12)) / (r_dual / max(ed, 1e-12)))
                    if ratio > 5.0 or ratio < 0.2:
                        rho = float(np.clip(rho * ratio, 1e-4, 1e4))
                        K = Ps + self.sigma * np.eye(n) + rho * AtA
                        L = np.linalg.cholesky(K)
        return x * D, z / E, y * E, dict(iters=it, r_prim=r_prim, r_dual=r_dual, rho=rho,
                                         converged=bool(r_prim <= self.conv_prim and r_dual <= self.conv_dual))


# =============================================================================
# 7. The coordinated kinematic MPC planner (eqs 28-39)
# =============================================================================
@dataclass
class MpcConfig:
    dt: float = 0.1                 # planner step (paper: 10 Hz)
    n_p: int = 10                   # prediction horizon
    n_c: int = 5                    # control horizon (u held after n_c)
    k_p: float = 2.0                # eq 37's K_p
    w_e: np.ndarray = field(default_factory=lambda: np.array([10.0, 10.0, 10.0]))   # W_e
    w_c: np.ndarray = field(default_factory=lambda: np.array([0.1, 0.1]))           # W_c (xy)
    # the wrist (q4) carries NO adaptive weight (eq 32's sum is empty for the
    # last joint) and is coaxial with the grasp point, so its only cost is the
    # posture term: at 0.02 its acceleration was a flat direction of the QP
    # (solver-dependent +-2 rad/s^2, 2026-10-10) -- 0.2 gives it curvature
    w_u_base: np.ndarray = field(default_factory=lambda: np.array([0.05] * 3 + [0.02] * 3 + [0.2]))
    # W_u = diag(w_u_base + w_u_scale * W_am). The scale sets how reluctant the
    # BASE is at small force: 10 makes the arm do every small motion but lets
    # a 1 m EE transit overshoot by 16 cm (the arm runs ahead, the base
    # follows late); 3 carries a transit within 1 cm with the posture held and
    # still hands a 5 cm EE step at small force partly to the arm (offline
    # sweep, coordinated_admittance.py self-test [8]/[11]).
    w_u_scale: float = 3.0
    w_du: float = 0.0               # optional increment penalty
    w_psi: float = 5.0              # addition (a): EE heading through q1
    # addition (d): joint-posture regularisation toward the streamed q_d.
    # Without it the EE task leaves the arm's null motion and the base/arm
    # split of a long transit undetermined (measured offline: a 1 m EE transit
    # parks q2 on its -18 deg stop with the base 16 cm past the goal). Small
    # against W_e so the EE task always wins; q1 is left to the heading term.
    w_q_post: np.ndarray = field(default_factory=lambda: np.array([0.0, 1.0, 1.0, 1.0]))
    u_max: np.ndarray = field(default_factory=lambda: np.array([1.5, 1.5, 1.5, 3.0, 3.0, 3.0, 3.0]))
    du_max: np.ndarray = field(default_factory=lambda: np.array([1.0, 1.0, 1.0, 3.0, 3.0, 3.0, 3.0]))
    v_max: np.ndarray = field(default_factory=lambda: np.array([0.5, 0.5, 0.5, 1.0, 1.0, 1.0, 1.0]))
    p_min: np.ndarray = field(default_factory=lambda: np.array([-3.0, -3.0, 0.3]))
    p_max: np.ndarray = field(default_factory=lambda: np.array([3.0, 3.0, 2.2]))
    # paper Remark 2: joint ranges PRE-PROCESSED from the real kinematics --
    # q3 floored at +5 deg keeps the elbow out of its singular (negative) branch
    q_min: np.ndarray = field(default_factory=lambda: np.array([Q_MIN[0] + 0.03, Q_MIN[1] + 0.03, math.radians(5.0), Q_MIN[3] + 0.03]))
    q_max: np.ndarray = field(default_factory=lambda: Q_MAX - 0.03)
    ee_body_min: np.ndarray = field(default_factory=lambda: np.array([0.08, -0.25, -0.35]))
    ee_body_max: np.ndarray = field(default_factory=lambda: np.array([0.35, 0.25, -0.03]))
    l_m: float = 0.06               # |P_c^B| <= l_m (xy)
    # soft state/EE/CoM boxes: one slack per group, cost w_lin*s + w_quad*s^2.
    # QUADRATIC ONLY by default: the linear (exact-penalty) term is a kink at
    # s = 0 that OSQP crawls on -- measured on recorded hover ticks
    # (2026-10-10): lin 1e4 + quad 1e4 = 2750 iterations / 13 ms, quad 1e6
    # alone = 50 iterations / 0.3 ms, same plan. At 1e6 a box that binds with
    # a multiplier of ~300 is violated by ~0.15 mm / 0.01 deg.
    w_slack_lin: float = 0.0
    w_slack_quad: float = 1.0e6
    base_com: np.ndarray = field(default_factory=lambda: np.zeros(3))   # model frame

    def __post_init__(self):
        for k in ("w_e", "w_c", "w_u_base", "w_q_post", "u_max", "du_max", "v_max", "p_min", "p_max",
                  "q_min", "q_max", "ee_body_min", "ee_body_max", "base_com"):
            setattr(self, k, np.asarray(getattr(self, k), float))


def _import_osqp():
    """The osqp wheel, if present (pip install --target ~/sources/pyosqp osqp<1.0
    -- on shiqi-desktop the user-site numpy 2.x breaks the apt scipy the wheel
    needs, so it lives beside the system numpy and is picked up from there)."""
    try:
        import osqp  # noqa: F401
        return osqp
    except Exception:  # noqa: BLE001
        extra = os.path.expanduser(os.environ.get("CADM_PYOSQP", "~/sources/pyosqp"))
        if os.path.isdir(extra) and extra not in sys.path:
            sys.path.append(extra)
            try:
                import osqp  # noqa: F401
                return osqp
            except Exception:  # noqa: BLE001
                return None
        return None


class OsqpQp:
    """The same interface as AdmmQp on OSQP (the solver the C++ planner node
    uses, v0.6.x): solve(P, q, A, l, u, x0, z0, y0) -> x, z, y, info."""

    def __init__(self, eps_abs=1e-6, eps_rel=1e-6, max_iter=4000, polish=True, conv_prim=1e-3, conv_dual=1e-1):
        self.osqp = _import_osqp()
        if self.osqp is None:
            raise ImportError("osqp is not importable (see _import_osqp)")
        import scipy.sparse as sp
        self.sp = sp
        self.eps_abs, self.eps_rel, self.max_iter, self.polish = float(eps_abs), float(eps_rel), int(max_iter), bool(polish)
        self.conv_prim, self.conv_dual = float(conv_prim), float(conv_dual)

    def solve(self, P, q, A, l, u, x0=None, z0=None, y0=None):
        m = self.osqp.OSQP()
        m.setup(P=self.sp.csc_matrix(np.triu(P)), q=np.asarray(q, float), A=self.sp.csc_matrix(A),
                l=np.asarray(l, float), u=np.asarray(u, float), verbose=False, eps_abs=self.eps_abs,
                eps_rel=self.eps_rel, max_iter=self.max_iter, polish=self.polish, adaptive_rho=True,
                warm_start=True, check_termination=10)
        if x0 is not None:
            if y0 is not None:
                m.warm_start(x=np.asarray(x0, float), y=np.asarray(y0, float))
            else:
                m.warm_start(x=np.asarray(x0, float))
        r = m.solve()
        x = np.asarray(r.x, float)
        y = np.asarray(r.y, float)
        ax = A @ x
        z = np.clip(ax, l, u)
        st = r.info.status_val
        solved = st == 1
        usable = solved or (st in (2, -2) and r.info.pri_res <= self.conv_prim and r.info.dua_res <= self.conv_dual)
        return x, z, y, dict(iters=int(r.info.iter), r_prim=float(r.info.pri_res), r_dual=float(r.info.dua_res),
                             rho=float(r.info.rho_estimate), converged=bool(usable), status=int(st))


def default_qp():
    """OSQP when importable (what the C++ node runs), else the built-in ADMM."""
    try:
        return OsqpQp()
    except ImportError:
        return AdmmQp()


class CoordinatedMpc:
    """One QP per tick. State x = [p_b (world) ; q ; p_b_dot ; q_dot]."""

    def __init__(self, cfg: MpcConfig, qp=None):
        self.cfg = cfg
        self.qp = qp or default_qp()
        self.nx, self.nu = 14, 7
        dt = cfg.dt
        self.A = np.block([[np.eye(7), dt * np.eye(7)], [np.zeros((7, 7)), np.eye(7)]])
        self.B = np.vstack([np.zeros((7, 7)), dt * np.eye(7)])            # eq 29, literally
        self._build_prediction()
        self.x_warm = None
        self.z_warm = None
        self.y_warm = None
        self.last = {}

    def _build_prediction(self):
        np_, nc = self.cfg.n_p, self.cfg.n_c
        Phi = np.zeros((np_ * self.nx, self.nx))
        Gam = np.zeros((np_ * self.nx, nc * self.nu))
        Ai = np.eye(self.nx)
        powers = [np.eye(self.nx)]
        for _ in range(np_):
            powers.append(powers[-1] @ self.A)
        for i in range(1, np_ + 1):
            Phi[(i - 1) * self.nx:i * self.nx] = powers[i]
            for j in range(i):
                col = min(j, nc - 1)            # u_j = u_{nc-1} for j >= nc
                Gam[(i - 1) * self.nx:i * self.nx, col * self.nu:(col + 1) * self.nu] += \
                    powers[i - 1 - j] @ self.B
        self.Phi, self.Gam = Phi, Gam
        # difference operator on U: D U = [u0 - u_prev ; u1 - u0 ; ...] (u_prev added at solve)
        D = np.zeros((nc * self.nu, nc * self.nu))
        for t in range(nc):
            D[t * self.nu:(t + 1) * self.nu, t * self.nu:(t + 1) * self.nu] = np.eye(self.nu)
            if t > 0:
                D[t * self.nu:(t + 1) * self.nu, (t - 1) * self.nu:t * self.nu] = -np.eye(self.nu)
        self.D = D

    def solve(self, x0, R_b, p_r, v_r, psi_b, psi_e_d, q_ref, w_am, u_prev=None):
        """x0 (14), R_b (3x3 actual attitude), p_r/v_r the admittance output,
        psi_b the measured base yaw, psi_e_d the desired EE heading (world),
        q_ref the streamed joint reference (posture term, addition (d)),
        w_am the adaptive weight vector (7).
        Returns dict(h1, hdot1, u0, x_pred (n_p x 14), info)."""
        cfg = self.cfg
        x0 = np.asarray(x0, float).copy()
        # the initial VELOCITIES are clipped to +-v_max: a commanded velocity
        # beyond the box (cadm_08, 2026-10-10: 0.85 m/s against v_max 0.4
        # after a re-sync) made the step-1 box row unreachable under the input
        # box and every solve from then on sat at its iteration cap. Positions
        # and joints are NOT clipped (a vehicle seated below p_min, or a joint
        # a degree past its box, is a fact the soft boxes absorb; a faked x0
        # would mis-place the EE linearisation).
        x0_raw = x0.copy()
        x0[7:] = np.clip(x0[7:], -cfg.v_max, cfg.v_max)
        x0_clamped = bool(np.any(np.abs(x0 - x0_raw) > 1e-12))
        q0 = x0[3:7]
        np_, nc, nx, nu = cfg.n_p, cfg.n_c, self.nx, self.nu
        c0, Jc, e0, Je = arm_jacobians(q0, cfg.base_com)
        RJe = R_b @ Je
        Rc = R_b @ (e0 - Je @ q0)                     # P_e = p_b + RJe q + Rc
        # ---- cost blocks on X (stacked states) --------------------------------
        rows = []
        offs = []
        wts = []
        # terminal error E_ter_i = K_p (P_e,i - P_r,i) + (P_e_dot,i - v_r)
        for i in range(1, np_ + 1):
            Ce = np.zeros((3, nx))
            Ce[:, 0:3] = np.eye(3)
            Ce[:, 3:7] = RJe
            Cv = np.zeros((3, nx))
            Cv[:, 7:10] = np.eye(3)
            Cv[:, 10:14] = RJe
            p_ri = np.asarray(p_r, float) + np.asarray(v_r, float) * (i * cfg.dt)
            C = cfg.k_p * Ce + Cv
            d = cfg.k_p * (Rc - p_ri) - np.asarray(v_r, float)
            blk = np.zeros((3, np_ * nx))
            blk[:, (i - 1) * nx:i * nx] = C
            rows.append(blk)
            offs.append(d)
            wts.append(cfg.w_e)
            # CoM shift (xy, body frame): P_c = c0 + Jc (q - q0)
            Cc = np.zeros((2, nx))
            Cc[:, 3:7] = Jc[:2]
            blk = np.zeros((2, np_ * nx))
            blk[:, (i - 1) * nx:i * nx] = Cc
            rows.append(blk)
            offs.append((c0 - Jc @ q0)[:2])
            wts.append(cfg.w_c)
            # addition (a): EE heading psi_e = psi_b + q1 -> q1 target
            if cfg.w_psi > 0.0:
                blk = np.zeros((1, np_ * nx))
                blk[0, (i - 1) * nx + 3] = 1.0
                rows.append(blk)
                offs.append(np.array([-wrap(psi_e_d - psi_b)]))
                wts.append(np.array([cfg.w_psi]))
            # addition (d): posture q -> q_ref on the joints whose weight is > 0
            for j in range(4):
                if cfg.w_q_post[j] > 0.0:
                    blk = np.zeros((1, np_ * nx))
                    blk[0, (i - 1) * nx + 3 + j] = 1.0
                    rows.append(blk)
                    offs.append(np.array([-float(np.asarray(q_ref, float)[j])]))
                    wts.append(np.array([cfg.w_q_post[j]]))
        S = np.vstack(rows)
        s = np.concatenate(offs)
        w = np.concatenate(wts)
        Sw = S * w[:, None]
        sw = s * w
        # J = |Sw (Phi x0 + Gam U) + sw|^2 + |Wu U|^2 + w_du |D U - [u_prev;0..]|^2
        SG = Sw @ self.Gam
        r0 = Sw @ (self.Phi @ x0) + sw
        wu = cfg.w_u_base + cfg.w_u_scale * np.asarray(w_am, float)
        Wu = np.tile(wu, nc)
        P = 2.0 * (SG.T @ SG + np.diag(Wu ** 2))
        qv = 2.0 * (SG.T @ r0)
        u_prev = np.zeros(nu) if u_prev is None else np.asarray(u_prev, float)
        if cfg.w_du > 0.0:
            dprev = np.zeros(nc * nu)
            dprev[:nu] = u_prev
            P += 2.0 * cfg.w_du * (self.D.T @ self.D)
            qv += -2.0 * cfg.w_du * (self.D.T @ dprev)
        # ---- constraints: l <= A_c U <= u_c --------------------------------
        A_rows, lo, hi = [], [], []
        # inputs (33)
        A_rows.append(np.eye(nc * nu))
        lo.append(np.tile(-cfg.u_max, nc))
        hi.append(np.tile(cfg.u_max, nc))
        # increments (34)
        dprev = np.zeros(nc * nu)
        dprev[:nu] = u_prev
        A_rows.append(self.D)
        lo.append(np.tile(-cfg.du_max, nc) + dprev)
        hi.append(np.tile(cfg.du_max, nc) + dprev)
        # states (35): h box and |h_dot| <= v_max, EE body box (36), CoM box.
        # These are SOFT: one non-negative slack per group (state / EE / CoM)
        # widens the group's box symmetrically, at a penalty steep enough that
        # the slack is zero whenever the box can be met (an exact-penalty
        # weight: w_slack_lin >> any cost gradient). Hard boxes made the QP
        # infeasible whenever the state sat outside its box (cadm_08,
        # 2026-10-10: every solve at its iteration cap), and an infeasible QP
        # has no usable iterate at all.
        xl = np.concatenate([cfg.p_min, cfg.q_min, -cfg.v_max])
        xu = np.concatenate([cfg.p_max, cfg.q_max, cfg.v_max])
        Gx = np.vstack([np.hstack([np.zeros((nx, (i - 1) * nx)), np.eye(nx),
                                   np.zeros((nx, (np_ - i) * nx))]) for i in range(1, np_ + 1)])
        base = Gx @ (self.Phi @ x0)
        groups = [(Gx @ self.Gam, np.tile(xl, np_) - base, np.tile(xu, np_) - base)]
        Cee = np.zeros((3 * np_, np_ * nx))
        Cc2 = np.zeros((2 * np_, np_ * nx))
        for i in range(1, np_ + 1):
            Cee[(i - 1) * 3:i * 3, (i - 1) * nx + 3:(i - 1) * nx + 7] = Je
            Cc2[(i - 1) * 2:i * 2, (i - 1) * nx + 3:(i - 1) * nx + 7] = Jc[:2]
        ee_off = e0 - Je @ q0
        bee = Cee @ (self.Phi @ x0)
        groups.append((Cee @ self.Gam, np.tile(cfg.ee_body_min - ee_off, np_) - bee,
                       np.tile(cfg.ee_body_max - ee_off, np_) - bee))
        c_off = (c0 - Jc @ q0)[:2]
        lm = cfg.l_m / SQRT2
        bc = Cc2 @ (self.Phi @ x0)
        groups.append((Cc2 @ self.Gam, np.tile(-lm - c_off, np_) - bc, np.tile(lm - c_off, np_) - bc))
        ng = len(groups)
        nU = nc * nu
        # the free prediction (U = 0) violating a group's box is what the
        # slack will have to absorb; counted for the diagnostics
        viol = np.zeros(ng, bool)
        for gi, (Ag, lg, ug) in enumerate(groups):
            viol[gi] = bool(np.any(lg > 0.0) or np.any(ug < 0.0))
            # lower side:  lg <= Ag U + s_g      upper side:  Ag U - s_g <= ug
            A_rows.append(np.hstack([Ag, np.zeros((Ag.shape[0], ng))]))
            A_rows[-1][:, nU + gi] = 1.0
            lo.append(lg)
            hi.append(np.full(Ag.shape[0], np.inf))
            A_rows.append(np.hstack([Ag, np.zeros((Ag.shape[0], ng))]))
            A_rows[-1][:, nU + gi] = -1.0
            lo.append(np.full(Ag.shape[0], -np.inf))
            hi.append(ug)
        # the input rows carry no slack
        for r in range(2):
            A_rows[r] = np.hstack([A_rows[r], np.zeros((A_rows[r].shape[0], ng))])
        # s_g >= 0
        A_rows.append(np.hstack([np.zeros((ng, nU)), np.eye(ng)]))
        lo.append(np.zeros(ng))
        hi.append(np.full(ng, np.inf))
        A_c = np.vstack(A_rows)
        l_c = np.concatenate(lo)
        u_c = np.concatenate(hi)
        Pz = np.zeros((nU + ng, nU + ng))
        Pz[:nU, :nU] = P
        Pz[nU:, nU:] = 2.0 * cfg.w_slack_quad * np.eye(ng)
        qz = np.concatenate([qv, np.full(ng, cfg.w_slack_lin)])
        # ---- solve ------------------------------------------------------------
        x_w = self.x_warm
        if x_w is not None and x_w.shape[0] == nU + ng:
            x_w = np.concatenate([x_w[nu:nU], x_w[nU - nu:nU], x_w[nU:]])   # shift U, keep s
        else:
            x_w = None
        Xs, z, y, info = self.qp.solve(Pz, qz, A_c, l_c, u_c, x0=x_w, z0=None, y0=None)
        U = Xs[:nU]
        slack = np.maximum(Xs[nU:], 0.0)
        # an unconverged iterate is not a plan: clip it into the input box so
        # a consumer that applies it anyway never commands beyond u_max, and
        # drop the warm start (a far-off iterate is worse than a cold start)
        if not info["converged"]:
            U = np.clip(U, np.tile(-cfg.u_max, nc), np.tile(cfg.u_max, nc))
            self.x_warm = None
        else:
            self.x_warm = Xs.copy()
        cost = float(0.5 * U @ P @ U + qv @ U)
        X = (self.Phi @ x0 + self.Gam @ U).reshape(np_, nx)
        u0 = U[:nu]
        self.last = dict(h1=X[0, :7], hdot1=X[0, 7:], u0=u0, x_pred=X, info=info,
                         cost=cost, p_e_now=x0[:3] + R_b @ e0, e0=e0, c0=c0,
                         viol=int(np.sum(viol)), slack=slack, x0_clamped=x0_clamped, x0=x0)
        return self.last


def leash(cmd, meas, radius):
    """Keep a commanded position within `radius` of the measured one WITHOUT a
    jump: the command is pulled radially toward the measurement by exactly the
    excess. Continuous in both arguments (a re-sync that JUMPS the command onto
    the measurement, and takes the measured velocity with it, is what drove
    cadm_08's MPC to its iteration cap: 160 mm steps in x0 every ~0.5 s)."""
    cmd = np.asarray(cmd, float)
    d = cmd - np.asarray(meas, float)
    n = float(np.linalg.norm(d))
    if n <= radius or radius <= 0.0:
        return cmd.copy(), False
    return np.asarray(meas, float) + d * (radius / n), True


# =============================================================================
# Self-test
# =============================================================================
def _selftest():
    np.set_printoptions(precision=5, suppress=True)
    ok = True

    # -- Euler kinematics: euler_zyx inverts euler_to_r; W maps eta_dot to omega
    rng = np.random.default_rng(3)
    for _ in range(200):
        eta = rng.uniform(-1.2, 1.2, 3)
        R = euler_to_r(eta)
        err = np.abs(euler_zyx(R) - eta).max()
        ok &= err < 1e-12
        ed = rng.normal(size=3)
        h = 1e-6
        Rp, Rm = euler_to_r(eta + h * ed), euler_to_r(eta - h * ed)
        Rdot = (Rp - Rm) / (2 * h)
        om_fd = np.array([(R.T @ Rdot)[2, 1], (R.T @ Rdot)[0, 2], (R.T @ Rdot)[1, 0]])
        ok &= np.abs(om_fd - euler_w(eta) @ ed).max() < 1e-6
        # dT/deta against finite differences
        I = np.diag([0.118, 0.119, 0.131]) + 0.0089 * (np.ones((3, 3)) - np.eye(3)) * 0  # diag
        I[0, 1] = I[1, 0] = 0.0089

        def T(e):
            W = euler_w(e)
            return 0.5 * ed @ (W.T @ I @ W @ ed)
        g_fd = np.array([(T(eta + h * np.eye(3)[k]) - T(eta - h * np.eye(3)[k])) / (2 * h) for k in range(3)])
        ok &= np.abs(g_fd - kinetic_energy_gradient(eta, ed, I)).max() < 1e-6
    print(f"[1] Euler kinematics, W, dT/deta: {'OK' if ok else 'FAIL'}")

    # -- flight law at hover: f_t = m g, tau = 0; a step gives the PD force
    cfg = FlightLawConfig()
    out = flight_law(cfg, np.zeros(3), np.zeros(3), np.eye(3), np.zeros(3),
                     np.zeros(3), np.zeros(3), np.zeros(3), 0.0)
    ok2 = abs(out["f_t"] - cfg.mass * G) < 1e-12 and np.abs(out["tau_body"]).max() < 1e-12
    out = flight_law(cfg, np.array([0.1, 0.0, 0.0]), np.zeros(3), np.eye(3), np.zeros(3),
                     np.zeros(3), np.zeros(3), np.zeros(3), 0.0)
    ok2 &= abs(out["f_des"][0] + cfg.mass * cfg.kpp[0] * 0.1) < 1e-12
    ok2 &= out["R_des"][0, 2] < 0.0 and out["tau_body"][1] < 0.0   # thrust tilts to -x = nose UP (theta_d < 0) -> negative pitch torque
    print(f"[2] flight law hover/step: {'OK' if ok2 else 'FAIL'}  (f_t hover {cfg.mass*G:.4f} N)")
    ok &= ok2

    # -- estimator converges to an injected wrench on a simulated rigid body
    I = np.diag([0.118, 0.119, 0.131])
    est = MomentumEstimator(cfg.mass, I, 20.0, 5.0)
    p = np.zeros(3); v = np.zeros(3); R = np.eye(3); om = np.zeros(3)
    F_true = np.array([0.6, -0.4, -1.2])
    tau_true = np.array([0.05, -0.03, 0.02])
    dt = 0.004
    est.reset(np.zeros(6))
    hist = []
    for k in range(int(12.0 / dt)):
        law = flight_law(cfg, p, v, R, om, np.zeros(3), np.zeros(3), np.zeros(3), 0.0)
        # plant: m vdot = f R e3 - m g e3 + F_true; I omdot = tau - om x I om + tau_true (body)
        acc = (law["f_t"] * (R @ E3) - cfg.mass * G * E3 + F_true) / cfg.mass
        om_dot = np.linalg.solve(I, law["tau_body"] - np.cross(om, I @ om) + tau_true)
        eta = euler_zyx(R)
        eta_dot = np.linalg.solve(euler_w(eta), om)
        u = est.applied_force(law["f_t"], R, eta, eta_dot, law["tau_eta"])
        chi = est.momentum(v, eta, eta_dot)
        r_hat = est.update(u, chi, dt)
        v = v + acc * dt
        p = p + v * dt
        om = om + om_dot * dt
        R = R @ (np.eye(3) + hat(om * dt) + 0.5 * hat(om * dt) @ hat(om * dt))
        U_, S_, Vt_ = np.linalg.svd(R)
        R = U_ @ Vt_
        hist.append(r_hat.copy())
    r_fin = hist[-1]
    tau_eta_true = euler_w(euler_zyx(R)).T @ tau_true
    e_f = np.abs(r_fin[:3] - F_true).max()
    e_t = np.abs(r_fin[3:] - tau_eta_true).max()
    # rise time of a critically damped 2nd order with wn = 10 rad/s: ~0.33 s to 90%
    t90 = next(k * dt for k, r in enumerate(hist) if np.linalg.norm(r[:3] - F_true) < 0.1 * np.linalg.norm(F_true))
    ok3 = e_f < 2e-3 and e_t < 2e-3 and 0.2 < t90 < 0.6   # 12 s: the PD position loop (no integral) has settled, so the residual is the estimator alone
    print(f"[3] momentum estimator: F err {e_f:.2e} N, tau err {e_t:.2e} N.m, t90 {t90:.2f} s "
          f"(wn {math.sqrt(100):.0f} rad/s, zeta {0.5*math.sqrt(4):.1f}): {'OK' if ok3 else 'FAIL'}")
    ok &= ok3

    # -- admittance: steady state e = -F/K (P_r = P_o + F/K ... sign: P_r moves WITH the force)
    adm = AdmittanceFilter(0.5, 11.0, 60.0)
    F = np.array([1.2, 0.0, 0.0])
    for _ in range(400):
        p_r, v_r, a_r = adm.update(F, np.zeros(3), np.zeros(3), np.zeros(3), 0.02)
    ok4 = abs(p_r[0] - (1.2 / 60.0)) < 1e-6 and np.abs(v_r).max() < 1e-6
    print(f"[4] admittance steady state e = -F/K -> P_r = P_o + F/K = {p_r[0]*1e3:.2f} mm: {'OK' if ok4 else 'FAIL'}")
    ok &= ok4

    # -- weights
    w0, wu0 = weight_vector(np.array([0.3, 0.1, 0.0]), 3.0, 2.0, ARM_M[1:])
    w1, wu1 = weight_vector(np.array([2.5, 0.5, 0.0]), 3.0, 2.0, ARM_M[1:])
    ok5 = wu0 > 0.9 and wu1 < 0.15 and abs(w0[6]) < 1e-12 and w1[3] > w1[4] > w1[5] > w1[6]
    print(f"[5] adaptive weights: small force w_uav {wu0:.3f} W={w0}; large force w_uav {wu1:.3f} W={w1}: "
          f"{'OK' if ok5 else 'FAIL'}")
    ok &= ok5

    # -- arm model against the planner fixture (home case, base_com 0)
    q_home = np.array([0.0, 0.6981317007977318, 0.6981317007977318, 0.0])
    r0c, r0e = arm_fk_model(q_home)
    err = max(np.abs(r0c - [8.576915154685956e-05, 0.019495814648010003, -0.014488211489149383]).max(),
              np.abs(r0e - [0.0015650000000000004, 0.2528986252301309, -0.06068932796643191]).max())
    ok6 = err < 1e-12
    c0, Jc, e0, Je = arm_jacobians(q_home)
    print(f"[6] arm model = planner fixture ({err:.1e} m); actual-frame EE at home {e0}, "
          f"CoM {c0}: {'OK' if ok6 else 'FAIL'}")
    ok &= ok6

    # -- QP solver against a brute-force reference on a random problem
    rng = np.random.default_rng(1)
    n, m = 12, 20
    Mx = rng.normal(size=(n, n))
    P = Mx @ Mx.T + 0.1 * np.eye(n)
    qv = rng.normal(size=n)
    A = rng.normal(size=(m, n))
    l = -rng.uniform(0.2, 1.0, m)
    u = rng.uniform(0.2, 1.0, m)
    x, z, y, info = AdmmQp(max_iter=3000, eps_abs=1e-8, eps_rel=1e-8).solve(P, qv, A, l, u)
    ok7 = info["r_prim"] < 1e-6
    try:
        from scipy.optimize import minimize
        cons = [{"type": "ineq", "fun": lambda x_: A @ x_ - l}, {"type": "ineq", "fun": lambda x_: u - A @ x_}]
        res = minimize(lambda x_: 0.5 * x_ @ P @ x_ + qv @ x_, np.zeros(n), jac=lambda x_: P @ x_ + qv,
                       constraints=cons, method="SLSQP", options={"ftol": 1e-12, "maxiter": 500})
        j_admm = 0.5 * x @ P @ x + qv @ x
        ok7 &= abs(j_admm - res.fun) < 1e-5 * max(1.0, abs(res.fun)) and np.abs(x - res.x).max() < 1e-3
        print(f"[7] ADMM QP vs SLSQP: J {j_admm:.6f} vs {res.fun:.6f}, |dx| {np.abs(x-res.x).max():.1e}, "
              f"{info['iters']} iters: {'OK' if ok7 else 'FAIL'}")
    except ImportError:
        print(f"[7] ADMM QP (no scipy): r_prim {info['r_prim']:.1e}: {'OK' if ok7 else 'FAIL'}")
    ok &= ok7

    # -- MPC: closed-loop kinematic tracking of an EE step, small force -> the arm moves first
    import time as _time
    mcfg = MpcConfig(w_c=np.zeros(2), w_q_post=np.zeros(4))   # CoM + posture off: the weight switch alone decides
    mpc = CoordinatedMpc(mcfg)
    R_b = np.eye(3)
    x = np.concatenate([np.array([0.0, 0.0, 1.3]), q_home, np.zeros(7)])
    c0, Jc, e0, Je = arm_jacobians(q_home)
    p_e0 = x[:3] + e0
    p_r = p_e0 + np.array([0.0, 0.0, -0.05])         # 5 cm down: the arm can do it alone
    # a small force with ALL components: eq 32 penalises the base only along the
    # sensed force, so a force along x alone would leave the base free in z
    w_small, _ = weight_vector(np.array([0.2, 0.2, 0.2]), 3.0, 2.0, ARM_M[1:])
    t0 = _time.perf_counter()
    base_move = 0.0
    nsteps = 40
    u_prev = np.zeros(7)
    for k in range(nsteps):
        out = mpc.solve(x, R_b, p_r, np.zeros(3), 0.0, 0.0, q_home, w_small, u_prev)
        u_prev = out["u0"]
        x = np.concatenate([out["h1"], out["hdot1"]])     # perfect tracking of the plan
        base_move = max(base_move, float(np.linalg.norm(x[:3] - [0.0, 0.0, 1.3])))
    el = (_time.perf_counter() - t0) / nsteps
    _, pe = arm_fk_actual(x[3:7])
    ee_err = float(np.linalg.norm(x[:3] + R_b @ pe - p_r))
    ok8 = ee_err < 3e-3 and base_move < 0.025 and np.all(x[3:7] <= mcfg.q_max + 1e-9) and np.all(x[3:7] >= mcfg.q_min - 1e-9)
    print(f"[8] MPC EE step (small force): EE err {ee_err*1e3:.2f} mm, base moved {base_move*1e3:.1f} mm, "
          f"q {np.degrees(x[3:7])} deg, {el*1e3:.1f} ms/solve, {out['info']['iters']} iters: {'OK' if ok8 else 'FAIL'}")
    ok &= ok8
    # large force -> the base moves instead (the same step, UAV weights ~0, joints penalised)
    x = np.concatenate([np.array([0.0, 0.0, 1.3]), q_home, np.zeros(7)])
    w_big, _ = weight_vector(np.array([1.7, 1.7, 1.7]), 3.0, 2.0, ARM_M[1:])
    mpc2 = CoordinatedMpc(mcfg)
    arm_move = 0.0
    u_prev = np.zeros(7)
    for k in range(nsteps):
        out = mpc2.solve(x, R_b, p_r, np.zeros(3), 0.0, 0.0, q_home, w_big, u_prev)
        u_prev = out["u0"]
        x = np.concatenate([out["h1"], out["hdot1"]])
        arm_move = max(arm_move, float(np.abs(x[3:7] - q_home).max()))
    _, pe = arm_fk_actual(x[3:7])
    ee_err2 = float(np.linalg.norm(x[:3] + R_b @ pe - p_r))
    base_move2 = float(np.linalg.norm(x[:3] - [0.0, 0.0, 1.3]))
    ok9 = ee_err2 < 3e-3 and base_move2 > 0.04 and arm_move < math.radians(3.0)
    print(f"[9] MPC EE step (large force): EE err {ee_err2*1e3:.2f} mm, base moved {base_move2*1e3:.1f} mm, "
          f"arm moved {math.degrees(arm_move):.2f} deg: {'OK' if ok9 else 'FAIL'}")
    ok &= ok9
    # the paper's CoM-shift term: with W_c on, the planner folds the arm toward
    # the CoM-neutral configuration while the base keeps the EE on target
    mcfg3 = MpcConfig(w_c=np.array([3.0, 3.0]), w_q_post=np.zeros(4))
    mpc3 = CoordinatedMpc(mcfg3)
    x = np.concatenate([np.array([0.0, 0.0, 1.3]), q_home, np.zeros(7)])
    p_r = p_e0.copy()
    u_prev = np.zeros(7)
    for k in range(60):
        out = mpc3.solve(x, R_b, p_r, np.zeros(3), 0.0, 0.0, q_home, w_small, u_prev)
        u_prev = out["u0"]
        x = np.concatenate([out["h1"], out["hdot1"]])
    c_end, _ = arm_fk_actual(x[3:7])
    _, pe = arm_fk_actual(x[3:7])
    ok10 = np.linalg.norm(c_end[:2]) < np.linalg.norm(c0[:2]) and np.linalg.norm(x[:3] + pe - p_r) < 3e-3
    print(f"[10] CoM term: |P_c,xy| {np.linalg.norm(c0[:2])*1e3:.1f} -> {np.linalg.norm(c_end[:2])*1e3:.1f} mm, "
          f"q {np.degrees(x[3:7])} deg, EE err {np.linalg.norm(x[:3]+pe-p_r)*1e3:.2f} mm: {'OK' if ok10 else 'FAIL'}")
    ok &= ok10
    # a 1 m EE transit at small force with the shipped defaults: the base
    # must carry it, the arm must come back to the streamed posture
    mcfg4 = MpcConfig()
    mpc4 = CoordinatedMpc(mcfg4)
    x = np.concatenate([np.array([0.0, 0.0, 1.3]), q_home, np.zeros(7)])
    p_r = p_e0 + np.array([1.0, 0.0, 0.0])
    u_prev = np.zeros(7)
    bx_max = 0.0
    for k in range(80):
        out = mpc4.solve(x, R_b, p_r, np.zeros(3), 0.0, 0.0, q_home, w_small, u_prev)
        u_prev = out["u0"]
        x = np.concatenate([out["h1"], out["hdot1"]])
        bx_max = max(bx_max, x[0])
    _, pe = arm_fk_actual(x[3:7])
    ee_err4 = float(np.linalg.norm(x[:3] + pe - p_r))
    q_err4 = float(np.abs(x[3:7] - q_home).max())
    ok11 = ee_err4 < 3e-3 and q_err4 < math.radians(2.0) and abs(x[0] - 1.0) < 0.01 and bx_max < 1.03
    print(f"[11] 1 m transit (defaults): EE err {ee_err4*1e3:.2f} mm, base x {x[0]*1e3:.1f} mm (max {bx_max*1e3:.1f}), "
          f"posture err {math.degrees(q_err4):.2f} deg: {'OK' if ok11 else 'FAIL'}")
    ok &= ok11
    print("ALL OK" if ok else "SOME CHECKS FAILED")
    return ok


if __name__ == "__main__":
    import sys
    sys.exit(0 if _selftest() else 1)
