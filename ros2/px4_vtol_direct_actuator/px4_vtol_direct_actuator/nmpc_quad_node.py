#!/usr/bin/env python3
"""
Self-contained NMPC ROS2 node for the Isaac 4-rotor quadplane (fsc_cad_vtol).

Reuses the SO(3) path-following NMPC *design* from AVTOL_Work (multiple-shooting
NLP, RK4 discretization, ipopt, SO(3) geodesic attitude cost, second-order tilt
actuator states) but with a FRESH CasADi model of the ~3.4 kg quadplane we built
in Isaac (2 front TILTING rotors + 2 rear STATIC rotors). It solves the NMPC each
cycle and streams the optimal actuator command via PX4 `direct_actuator`, so it
flies the current Isaac vehicle while tracking a position/yaw reference.

Self-contained: does NOT import the AVTOL_Work package. Deps (all already present
for Isaac + the NMPC repo): casadi, numpy, rclpy, px4_msgs, geometry_msgs,
fsc_autopilot_ros2_msgs.

Predictor state x (22):
    [ pos(3), R_bn row-major(9), beta_L, beta_R, vel_world(3), omega_body(3),
      betad_L, betad_R ]
Control u (6):
    [ T_FL, T_FR, T_RL, T_RR (N), betacmd_L, betacmd_R (rad) ]

Run (clean shell w/ ROS + px4_msgs + the msgs sourced, casadi importable):
    python3 nmpc_quad_node.py            # ns=/uav_0, auto-arms, hover at 0,0,1.5
Drive the reference with fsc_ref / the ground station / step_response.py
(topic /uav_0/fsc_autopilot_ros2/position_controller/reference).
"""
from __future__ import annotations

import math
import os
from dataclasses import dataclass, field

import numpy as np
import casadi as ca

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy,
                       QoSHistoryPolicy, qos_profile_sensor_data)
from rcl_interfaces.msg import ParameterDescriptor

from px4_msgs.msg import (OffboardControlMode, ActuatorMotors, ActuatorServos,
                          VehicleCommand, VehicleStatus)
from fsc_autopilot_ros2_msgs.msg import Mocap, PositionControllerReference
from std_msgs.msg import String

# ============================================================================
#  STALE MODEL GUARD (2026-09-06)
# ----------------------------------------------------------------------------
#  This node has NOT been converted to the corrected plant. Its internal
#  aerodynamics still use the FLU angle of attack with FRD-published
#  coefficients, so it models a wing that lifts at NEGATIVE angle of attack and
#  a pitch axis that diverges -- the aircraft the simulator used to be, not the
#  one it is now. Flying it would command close to the opposite of what the
#  plant needs.
#
#  Converted and verified: nmpc_so3_transition_node (and nmpc6_cruise_node,
#  nmpc_full_smc_node, nmpc_so3_cascade_node, which share its aero) and
#  vtol_nmpc. The recipe is nmpc_so3_transition_node.aero_body; the check that
#  keeps them honest is tests/test_controller_aero_match.py.
#
#  FSC_ALLOW_STALE_MODEL=1 runs it anyway (offline work only).
# ============================================================================
if os.environ.get('FSC_ALLOW_STALE_MODEL') != '1':
    raise SystemExit(
        __file__ + ' models the PRE-2026-09-06 plant (inverted aerodynamic frame) and'
        ' has not been converted. Recipe: nmpc_so3_transition_node.aero_body; check:'
        ' tests/test_controller_aero_match.py. FSC_ALLOW_STALE_MODEL=1 overrides.')






# ======================================================================
#  Config — vehicle model + NMPC weights (matched to the Isaac quadplane)
# ======================================================================
@dataclass
class NMPCConfig:
    # --- horizon / discretization ---
    dt: float = 0.05          # prediction step [s]
    N: int = 12               # horizon steps (0.6 s)
    n_rk4: int = 1            # RK4 substeps per step

    # --- vehicle (fsc_cad_vtol, CG at rotor centroid) ---
    mass: float = 3.4
    g: float = 9.81
    Ivec: tuple = (0.20, 0.23, 0.36)          # (Ixx, Iyy, Izz) about CG
    # rotor positions CG-relative (FLU) FL, FR, RL, RR
    rotor_pos: tuple = ((0.285, 0.285, -0.025), (0.285, -0.284, -0.023),
                        (-0.286, 0.284, 0.024), (-0.286, -0.284, 0.026))
    rotor_spin: tuple = (1, -1, -1, 1)         # CCW/CW (reaction-torque sign)
    rotor_tilts: tuple = (1, 1, 0, 0)          # 1 = front tilting, 0 = rear static
    cQ: float = 0.12                           # yaw reaction torque per N thrust [m]
    tau_beta: float = 0.20                     # tilt-servo time constant [s]
    max_thrust: float = 18.0                   # N at motor cmd 1.0 (for normalization)

    # --- input bounds ---
    T_min: float = 0.0
    T_max: float = 17.0
    beta_min: float = 0.0
    beta_max: float = math.pi / 2

    # --- wing aero (MATCHES the Isaac sim vtol_aerodynamics coefficients so the NMPC
    #     predictor knows the wings carry lift at speed -> it can PLAN a coordinated
    #     transition, tilting in as lift builds instead of over-tilting and dropping).
    #     aero_enable=0 -> pure-multicopter model (old behaviour). ---
    aero_enable: float = 1.0
    aero_rho: float = 1.225
    aero_S: float = 0.55
    aero_b: float = 2.0
    aero_c: float = 0.28
    aero_cl0: float = 0.2
    aero_cl_alpha: float = 5.5
    aero_cl_q: float = 7.0
    aero_cl_max: float = 1.5
    aero_cd0: float = 0.04
    aero_k: float = 0.04
    aero_cy_beta: float = -0.98
    aero_cl_beta: float = -0.12
    aero_cl_p: float = -0.5
    aero_cl_r: float = 0.1
    aero_cm0: float = 0.02
    aero_cm_alpha: float = -1.2
    aero_cm_q: float = -12.0
    aero_cn_beta: float = 0.25
    aero_cn_r: float = -0.2
    aero_cn_p: float = -0.06
    aero_vmin: float = 2.0                       # floor for rate/alpha normalisation
    aero_stall_rad: float = math.radians(22.0)
    # smooth FORWARD-speed gate: wings only work with forward flow, and this keeps the
    # aero definitively OFF at hover/takeoff (no degenerate optima) and ramps it IN as
    # forward speed builds through the transition. g = 0.5*(1+tanh((u - u_on)/u_width)).
    aero_u_on: float = 6.0
    aero_u_width: float = 2.5

    # --- cost weights ---
    w_pos: float = 25.0
    w_vel: float = 4.0
    w_att: float = 30.0        # SO(3) geodesic — raised: keep airframe LEVEL + hold HEADING
                               # (weak yaw authority + latency were letting it fly diagonally)
    w_beta_sym: float = 200.0  # COUPLE the two front tilts HARD (penalize beta_L != beta_R):
                               # asymmetric tilt was the dominant instability (yaw/roll drift);
                               # at 40 the position cost overrode it, so force symmetry.
    w_omega: float = 1.0
    # Tilt is a genuine transition DOF: with w_beta < w_att the NMPC keeps the
    # airframe LEVEL (w_att) and TILTS the front rotors to make forward thrust
    # (the physical transition), instead of pitching the whole airframe. Set
    # w_beta high (>>w_att, e.g. 60) to force pure-multicopter (rotors vertical);
    # low (<w_att) to let the NMPC choose the tilt to track the reference.
    # Too low -> it zeros front-rotor thrust + full-tilts (fine at cruise speed
    # where the wings lift, but drops the vehicle at low speed -> keep it moderate).
    w_beta: float = 6.0        # tilt DOF enabled (<w_att=12) -> tilts for forward flight
    w_beta_lowspeed: float = 400.0  # SPEED-GATED tilt penalty: forces tilt~0 at low speed
                                    # (kills the hover tilt-chatter / degenerate optima); it
                                    # relaxes as forward speed builds so the transition tilt is
                                    # still free. Gate uses aero_u_on / aero_u_width.
    w_T: float = 0.002         # thrust regularization (toward hover)
    w_du: float = 0.4          # input-rate penalty
    w_so3: float = 10.0        # R^T R = I soft penalty
    w_terminal: float = 8.0

    # --- error scales (NLP conditioning) ---
    pos_scale: float = 1.0
    vel_scale: float = 2.0
    beta_scale: float = 0.5
    omega_scale: float = 2.0
    T_scale: float = 8.0
    du_T_scale: float = 5.0
    du_beta_scale: float = 0.5

    # --- ipopt ---
    max_iter: int = 120          # was 40 -> hit Max_Iterations at hover (degenerate tilt);
                                 # give it room to converge to the true (tilt=0) hover optimum
    tol: float = 1e-4
    acceptable_tol: float = 1e-3
    print_level: int = 0


# ======================================================================
#  QuadNMPC — CasADi model + multiple-shooting NLP (ipopt, warm-started)
# ======================================================================
class QuadNMPC:
    NX = 22
    NU = 6

    def __init__(self, cfg: NMPCConfig):
        self.cfg = cfg
        self._solver = None
        self._prev_w = None
        self._prev_lam_g = None
        self._prev_lam_x = None

    # ---- continuous dynamics xdot = f(x,u) ----
    def _ode(self) -> ca.Function:
        c = self.cfg
        x = ca.SX.sym("x", self.NX)
        u = ca.SX.sym("u", self.NU)

        R = ca.reshape(x[3:12], 3, 3).T          # body -> inertial
        beta = [x[12], x[13], 0.0, 0.0]          # front tilt states; rear static (0)
        vel = x[14:17]
        omega = x[17:20]
        betad_L, betad_R = x[20], x[21]
        T = [u[0], u[1], u[2], u[3]]
        betacmd = [u[4], u[5]]

        F_body = ca.SX.zeros(3)
        M_body = ca.SX.zeros(3)
        for i in range(4):
            if c.rotor_tilts[i]:
                d = ca.vertcat(ca.sin(beta[i]), 0.0, ca.cos(beta[i]))
            else:
                d = ca.vertcat(0.0, 0.0, 1.0)
            Fi = T[i] * d
            ri = ca.DM(list(c.rotor_pos[i]))
            F_body = F_body + Fi
            M_body = M_body + ca.cross(ri, Fi) - c.rotor_spin[i] * c.cQ * T[i] * d

        # --- wing aero (body FLU), mirrors the Isaac sim vtol_aerodynamics so the NMPC
        # knows the wings carry lift at speed -> it plans a coordinated tilt transition.
        if c.aero_enable > 0.5:
            vb = ca.mtimes(R.T, vel)                 # world velocity -> body
            ub, vbb, wb = vb[0], vb[1], vb[2]
            V2 = ub * ub + vbb * vbb + wb * wb
            V = ca.sqrt(V2 + 1e-6)
            Vn = ca.fmax(V, c.aero_vmin)             # floor for normalisation (no /0 at hover)
            qbar = 0.5 * c.aero_rho * V2             # -> 0 at hover so aero vanishes smoothly
            alpha_r = ca.atan2(wb, ca.fmax(ub, 1e-3))          # REAL AoA -> force DIRECTION
            alpha = ca.fmax(ca.fmin(alpha_r, c.aero_stall_rad), -c.aero_stall_rad)  # clamped -> CL magnitude only
            beta_a = ca.asin(ca.fmax(ca.fmin(vbb / Vn, 0.999), -0.999))
            p_hat = omega[0] * c.aero_b / (2.0 * Vn)
            q_hat = omega[1] * c.aero_c / (2.0 * Vn)
            r_hat = omega[2] * c.aero_b / (2.0 * Vn)
            CL = c.aero_cl0 + c.aero_cl_alpha * alpha + c.aero_cl_q * q_hat
            CL = ca.fmax(ca.fmin(CL, c.aero_cl_max), -c.aero_cl_max)
            CD = c.aero_cd0 + c.aero_k * CL * CL
            CY = c.aero_cy_beta * beta_a
            Cl = c.aero_cl_beta * beta_a + c.aero_cl_p * p_hat + c.aero_cl_r * r_hat
            Cm = c.aero_cm0 + c.aero_cm_alpha * alpha + c.aero_cm_q * q_hat
            Cn = c.aero_cn_beta * beta_a + c.aero_cn_r * r_hat + c.aero_cn_p * p_hat
            # forward-speed gate: aero engages only with forward flow (no hover exploit)
            g_aero = 0.5 * (1.0 + ca.tanh((ub - c.aero_u_on) / c.aero_u_width))
            qS = g_aero * qbar * c.aero_S
            F_body = F_body + (-qS * CD) * (vb / V) \
                     + (qS * CL) * ca.vertcat(-ca.sin(alpha_r), 0.0, ca.cos(alpha_r)) \
                     + ca.vertcat(0.0, qS * CY, 0.0)
            M_body = M_body + ca.vertcat(qS * c.aero_b * Cl, qS * c.aero_c * Cm, qS * c.aero_b * Cn)

        g_in = ca.vertcat(0.0, 0.0, -c.g)
        pos_dot = vel
        v_dot = (1.0 / c.mass) * ca.mtimes(R, F_body) + g_in

        Imat = ca.DM(np.diag(c.Ivec))
        Iinv = ca.DM(np.diag([1.0 / v for v in c.Ivec]))
        omega_dot = ca.mtimes(Iinv, M_body - ca.cross(omega, ca.mtimes(Imat, omega)))

        p_r, q_r, r_r = omega[0], omega[1], omega[2]
        oh = ca.SX(3, 3)
        oh[0, 1] = -r_r; oh[0, 2] = q_r
        oh[1, 0] = r_r;  oh[1, 2] = -p_r
        oh[2, 0] = -q_r; oh[2, 1] = p_r
        R_dot = ca.mtimes(R, oh)
        R_dot_flat = ca.reshape(R_dot.T, 9, 1)   # row-major

        betadd_L = ((betacmd[0] - beta[0]) / c.tau_beta - betad_L) / c.tau_beta
        betadd_R = ((betacmd[1] - beta[1]) / c.tau_beta - betad_R) / c.tau_beta

        xdot = ca.vertcat(pos_dot, R_dot_flat, betad_L, betad_R,
                          v_dot, omega_dot, betadd_L, betadd_R)
        return ca.Function("ode", [x, u], [xdot])

    def _rk4(self, ode: ca.Function) -> ca.Function:
        c = self.cfg
        x = ca.SX.sym("x", self.NX)
        u = ca.SX.sym("u", self.NU)
        h = c.dt / c.n_rk4
        xc = x
        for _ in range(c.n_rk4):
            k1 = ode(xc, u)
            k2 = ode(xc + h / 2 * k1, u)
            k3 = ode(xc + h / 2 * k2, u)
            k4 = ode(xc + h * k3, u)
            xc = xc + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        return ca.Function("F_rk4", [x, u], [xc])

    def build(self):
        c = self.cfg
        N, nx, nu = c.N, self.NX, self.NU
        F = self._rk4(self._ode())

        # params: [x0(nx), xref(nx), uref(nu), u_prev(nu)]  (constant set-point ref)
        p = ca.SX.sym("p", nx + nx + nu + nu)
        x0p = p[0:nx]
        xref = p[nx:2 * nx]
        uref = p[2 * nx:2 * nx + nu]
        uprev = p[2 * nx + nu:2 * nx + 2 * nu]

        w, wlb, wub, g, glb, gub = [], [], [], [], [], []
        J = ca.SX(0)
        X, U = [], []

        xlb = np.full(nx, -1e6); xub = np.full(nx, 1e6)
        xlb[3:12] = -1.5; xub[3:12] = 1.5          # DCM entries
        xlb[14:17] = -50.0; xub[14:17] = 50.0      # vel
        xlb[17:20] = -20.0; xub[17:20] = 20.0      # omega
        xlb[12:14] = c.beta_min; xub[12:14] = c.beta_max
        ulb = [c.T_min, c.T_min, c.T_min, c.T_min, c.beta_min, c.beta_min]
        uub = [c.T_max, c.T_max, c.T_max, c.T_max, c.beta_max, c.beta_max]

        for k in range(N + 1):
            xk = ca.SX.sym(f"x_{k}", nx); X.append(xk)
            w.append(xk); wlb += xlb.tolist(); wub += xub.tolist()
        for k in range(N):
            uk = ca.SX.sym(f"u_{k}", nu); U.append(uk)
            w.append(uk); wlb += ulb; wub += uub

        g.append(X[0] - x0p); glb += [0.0] * nx; gub += [0.0] * nx

        def stage(xk, uk, uprev_k, terminal=False):
            e_pos = (xk[0:3] - xref[0:3]) / c.pos_scale
            e_vel = (xk[14:17] - xref[14:17]) / c.vel_scale
            e_omega = (xk[17:20] - xref[17:20]) / c.omega_scale
            e_beta = (xk[12:14] - xref[12:14]) / c.beta_scale
            Rk = ca.reshape(xk[3:12], 3, 3).T
            Rref = ca.reshape(xref[3:12], 3, 3).T
            att = 0.5 * ca.trace(ca.DM.eye(3) - ca.mtimes(Rref.T, Rk))
            so3 = ca.sumsqr(ca.mtimes(Rk.T, Rk) - ca.DM.eye(3))
            e_T = (uk[0:4] - uref[0:4]) / c.T_scale if uk is not None else 0
            # speed-gated tilt penalty: at LOW forward speed force the tilt to ~0 (kills the
            # hover chatter / degenerate 90-deg optima); relaxes as speed builds so the
            # transition tilt stays free. gate_low = 1 at hover -> 0 at cruise.
            vh = ca.sqrt(xk[14] ** 2 + xk[15] ** 2 + 1e-6)
            gate_low = 0.5 * (1.0 - ca.tanh((vh - c.aero_u_on) / c.aero_u_width))
            cost = (c.w_pos * ca.dot(e_pos, e_pos) + c.w_vel * ca.dot(e_vel, e_vel)
                    + c.w_att * att + c.w_omega * ca.dot(e_omega, e_omega)
                    + c.w_beta * ca.dot(e_beta, e_beta) + c.w_so3 * so3
                    + c.w_beta_sym * (xk[12] - xk[13]) ** 2       # symmetric-tilt coupling
                    + c.w_beta_lowspeed * gate_low * (xk[12] ** 2 + xk[13] ** 2))  # tilt~0 at low speed
            if uk is not None:
                du = uk - uprev_k
                e_du = ca.vertcat(du[0] / c.du_T_scale, du[1] / c.du_T_scale,
                                  du[2] / c.du_T_scale, du[3] / c.du_T_scale,
                                  du[4] / c.du_beta_scale, du[5] / c.du_beta_scale)
                cost += c.w_T * ca.dot(e_T, e_T) + c.w_du * ca.dot(e_du, e_du)
            return c.w_terminal * cost if terminal else cost

        for k in range(N):
            uprev_k = uprev if k == 0 else U[k - 1]
            J += stage(X[k], U[k], uprev_k)
            g.append(X[k + 1] - F(X[k], U[k])); glb += [0.0] * nx; gub += [0.0] * nx
        J += stage(X[N], None, None, terminal=True)

        nlp = {"x": ca.vertcat(*w), "f": J, "g": ca.vertcat(*g), "p": p}
        opts = {"ipopt.max_iter": c.max_iter, "ipopt.tol": c.tol,
                "ipopt.acceptable_tol": c.acceptable_tol,
                "ipopt.print_level": c.print_level, "print_time": 0,
                "ipopt.warm_start_init_point": "yes", "ipopt.mu_strategy": "adaptive",
                "error_on_fail": False}
        self._solver = ca.nlpsol("quad_nmpc", "ipopt", nlp, opts)
        self._wlb = np.array(wlb); self._wub = np.array(wub)
        self._glb = np.array(glb); self._gub = np.array(gub)
        self._np = 2 * nx + 2 * nu
        self._nw = len(wlb)

    def solve(self, x0, xref, uref, u_prev):
        c = self.cfg
        N, nx, nu = c.N, self.NX, self.NU
        p = np.concatenate([x0, xref, uref, u_prev])
        if self._prev_w is not None and len(self._prev_w) == self._nw:
            w0 = self._prev_w
        else:
            w0 = np.zeros(self._nw)
            for k in range(N + 1):
                w0[k * nx:(k + 1) * nx] = xref
            off = (N + 1) * nx
            for k in range(N):
                w0[off + k * nu:off + (k + 1) * nu] = uref
        kw = dict(x0=w0, p=p, lbx=self._wlb, ubx=self._wub, lbg=self._glb, ubg=self._gub)
        if self._prev_lam_g is not None:
            kw["lam_g0"] = self._prev_lam_g
        if self._prev_lam_x is not None:
            kw["lam_x0"] = self._prev_lam_x
        sol = self._solver(**kw)
        st = self._solver.stats()["return_status"]
        wsol = np.array(sol["x"]).flatten()
        if "Solve_Succeeded" in st or "Acceptable" in st:
            self._prev_w = wsol.copy()
            self._prev_lam_g = np.array(sol["lam_g"]).flatten()
            self._prev_lam_x = np.array(sol["lam_x"]).flatten()
            ok = True
        else:
            self._prev_w = None; self._prev_lam_g = None; self._prev_lam_x = None
            ok = False
        u0 = wsol[(N + 1) * nx:(N + 1) * nx + nu]
        return u0, ok, st


# ======================================================================
#  helpers
# ======================================================================
def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def Rz(psi):
    c, s = math.cos(psi), math.sin(psi)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


# ======================================================================
#  ROS2 node — state in, NMPC, actuators out (PX4 direct_actuator)
# ======================================================================
class NMPCQuadNode(Node):
    def __init__(self):
        super().__init__("nmpc_quad_control")
        P = self.declare_parameter
        # dynamic_typing so a numeric override passed as an int (e.g. rate_hz:=50)
        # is accepted rather than raising InvalidParameterType against a float default
        dyn = ParameterDescriptor(dynamic_typing=True)
        P("px4_ns", "uav_0"); P("auto_arm", True); P("arm_after", 100)
        # Controller arbitration: only the ACTIVE controller drives PX4 (the geometric
        # node + this NMPC both publish direct_actuator). GS switches via a String on
        # /<ns>/ground_station/active_controller. Default standby (geometric flies first).
        P("controller_id", "nmpc"); P("active_at_start", False)
        P("rate_hz", 50.0, dyn)
        # reference governor: how fast the tracked set-point ramps to the target
        P("sp_max_vel", 1.0, dyn); P("sp_max_climb", 0.8, dyn); P("sp_max_yawrate_deg", 60.0, dyn)
        # ---- OUTER-LOOP sliding-mode controller (position -> velocity reference) -------
        # Replaces the plain governor: v_cmd = -lam*e_p - K*sat(e_p/phi), clamped to v_max.
        # Two regimes: LOOSE (cruise/transition -> lets the position change fast) and STRICT
        # (landing -> tight, slow, precise). `smc_land_z` picks the regime by target altitude.
        P("smc_on", True)
        P("smc_land_z", 1.2, dyn)          # target z below this => STRICT (landing) regime
        # loose / transition
        P("smc_lam", 0.9, dyn); P("smc_k", 1.5, dyn); P("smc_phi", 2.0, dyn)
        P("smc_vmax", 16.0, dyn); P("smc_vzmax", 3.0, dyn)
        # strict / landing
        P("smc_lam_land", 1.6, dyn); P("smc_k_land", 0.6, dyn); P("smc_phi_land", 0.4, dyn)
        P("smc_vmax_land", 1.5, dyn); P("smc_vzmax_land", 0.6, dyn)
        P("smc_lead", 0.5, dyn)            # setpoint lead time: sp_pos = p + v_cmd*lead [s]
        ns = self.get_parameter("px4_ns").get_parameter_value().string_value
        pre = f"/{ns}" if ns else ""
        self.rate = float(self.get_parameter("rate_hz").value)
        self.auto_arm = bool(self.get_parameter("auto_arm").value)
        self.arm_after = int(self.get_parameter("arm_after").value)
        self.sp_max_vel = float(self.get_parameter("sp_max_vel").value)
        self.sp_max_climb = float(self.get_parameter("sp_max_climb").value)
        self.sp_max_yawrate = math.radians(float(self.get_parameter("sp_max_yawrate_deg").value))

        self.cfg = NMPCConfig()
        self.get_logger().info("building NMPC (CasADi/ipopt)...")
        self.nmpc = QuadNMPC(self.cfg)
        self.nmpc.build()
        self.get_logger().info("NMPC ready.")

        self.state = None
        self.have_state = False
        self.beta_est = np.zeros(2)       # front tilt estimate (rad)
        self.betad_est = np.zeros(2)
        self.ref = {"x": 0.0, "y": 0.0, "z": 1.5, "yaw": 0.0}
        self.sp = None            # smoothed set-point the NMPC actually tracks
        self.sp_vff = np.zeros(3)  # governor feed-forward velocity
        self.sp_yawrate = 0.0      # governor feed-forward yaw rate
        hover = self.cfg.mass * self.cfg.g / 4.0
        self.u_prev = np.array([hover, hover, hover, hover, 0.0, 0.0])
        self.u_cmd = self.u_prev.copy()
        self.armed = self.offb = False
        self.counter = self.last_cmd = 0

        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        self.ocm_pub = self.create_publisher(OffboardControlMode, f"{pre}/fmu/in/offboard_control_mode", qos)
        self.mot_pub = self.create_publisher(ActuatorMotors, f"{pre}/fmu/in/actuator_motors", qos)
        self.srv_pub = self.create_publisher(ActuatorServos, f"{pre}/fmu/in/actuator_servos", qos)
        self.cmd_pub = self.create_publisher(VehicleCommand, f"{pre}/fmu/in/vehicle_command", qos)
        self.create_subscription(VehicleStatus, f"{pre}/fmu/out/vehicle_status_v1", self.on_status, qos)
        self.create_subscription(Mocap, f"{pre}/mocap", self.on_mocap, qos_profile_sensor_data)
        self.create_subscription(PositionControllerReference,
                                 f"{pre}/fsc_autopilot_ros2/position_controller/reference", self.on_ref, 10)
        self.controller_id = self.get_parameter("controller_id").value
        self._active = bool(self.get_parameter("active_at_start").value)
        self.create_subscription(String, f"{pre}/ground_station/active_controller",
                                 self.on_active, 10)

        # fast heartbeat + arm (100 Hz) so offboard never drops even if a solve is slow
        self.create_timer(0.01, self.heartbeat)
        # NMPC solve loop
        self.create_timer(1.0 / self.rate, self.control)
        self.get_logger().info(f"nmpc_quad_control up ns='{ns}' solve={self.rate:.0f}Hz")

    def now_us(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def on_status(self, m):
        self.armed = (m.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        self.offb = (m.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD)

    def on_ref(self, m):
        self.ref["x"], self.ref["y"], self.ref["z"] = m.position.x, m.position.y, m.position.z
        self.ref["yaw"] = (m.yaw * math.pi / 180.0
                           if m.yaw_unit == PositionControllerReference.DEGREES else m.yaw)

    def on_mocap(self, m):
        q = m.pose.orientation
        R = quat_to_R(q.x, q.y, q.z, q.w)
        x = np.zeros(22)
        x[0:3] = [m.pose.position.x, m.pose.position.y, m.pose.position.z]
        x[3:12] = R.flatten()                                   # row-major
        x[12:14] = self.beta_est
        x[14:17] = [m.twist.linear.x, m.twist.linear.y, m.twist.linear.z]   # world vel
        x[17:20] = [m.twist.angular.x, m.twist.angular.y, m.twist.angular.z]  # body rates
        x[20:22] = self.betad_est
        self.state = x
        self.have_state = True

    def gp(self, n):
        return float(self.get_parameter(n).value)

    def _advance_governor(self):
        """OUTER-LOOP sliding-mode controller: position error -> robust velocity command,
        augmented into the NMPC reference. Two regimes (chosen by target altitude):
          LOOSE (cruise/transition) -> big v_max, small lambda -> lets the position change
                                        fast so the NMPC can build speed + tilt/transition;
          STRICT (landing)          -> small v_max, tight boundary layer -> precise touchdown.
        v_cmd = -lambda*e_p - K*sat(e_p/phi); setpoint = a lead point ahead of the vehicle."""
        dt = 1.0 / self.rate
        if self.sp is None:
            s = self.state
            self.sp = {"x": s[0], "y": s[1], "z": s[2], "yaw": math.atan2(s[3 + 3], s[3 + 0])}
        r, sp, p = self.ref, self.sp, self.state[0:3]
        # pick regime: STRICT when the TARGET altitude is low (landing), else LOOSE
        sfx = "_land" if r["z"] < self.gp("smc_land_z") else ""
        lam = self.gp("smc_lam" + sfx); K = self.gp("smc_k" + sfx)
        phi = max(self.gp("smc_phi" + sfx), 1e-3)
        vmax = self.gp("smc_vmax" + sfx); vzmax = self.gp("smc_vzmax" + sfx)
        # sliding-mode on the SET-POINT error to the target (e_sp = sp - target). This
        # ADVANCES a smooth, converging, STATIONARY reference toward the target -- a
        # lead-point that moves with the vehicle every cycle wrecks the NMPC warm-start
        # (-> per-solve motor chatter). The NMPC's own feedback rejects vehicle disturbances.
        e = np.array([sp["x"] - r["x"], sp["y"] - r["y"], sp["z"] - r["z"]])
        v_cmd = -lam * e - K * np.clip(e / phi, -1.0, 1.0)     # reaching + boundary-layer sat
        vh = math.hypot(v_cmd[0], v_cmd[1])
        if vh > vmax and vh > 1e-9:
            v_cmd[0] *= vmax / vh; v_cmd[1] *= vmax / vh       # horizontal speed limit
        v_cmd[2] = clamp(v_cmd[2], -vzmax, vzmax)              # climb/descent limit
        self.sp_vff = v_cmd
        # advance the set-point toward the target (clamped so it never overshoots)
        for ax, key in ((0, "x"), (1, "y"), (2, "z")):
            new = sp[key] + v_cmd[ax] * dt
            if (new - r[key]) * (sp[key] - r[key]) < 0.0:      # crossed the target -> snap
                new = r[key]
            sp[key] = new
        # yaw: bounded ramp to the target heading
        self.sp_max_yawrate = math.radians(self.gp("sp_max_yawrate_deg"))
        dyaw = math.atan2(math.sin(r["yaw"] - sp["yaw"]), math.cos(r["yaw"] - sp["yaw"]))
        yaw_step = clamp(dyaw, -self.sp_max_yawrate * dt, self.sp_max_yawrate * dt)
        sp["yaw"] += yaw_step
        self.sp_yawrate = yaw_step / dt

    def _xref_uref(self):
        sp = self.sp
        xr = np.zeros(22)
        xr[0:3] = [sp["x"], sp["y"], sp["z"]]
        xr[3:12] = Rz(sp["yaw"]).flatten()
        xr[14:17] = self.sp_vff            # feed-forward velocity from the governor
        xr[19] = self.sp_yawrate           # feed-forward body yaw rate
        # remaining omega, beta, betad = 0 (hover, rotors vertical)
        hover = self.cfg.mass * self.cfg.g / 4.0
        ur = np.array([hover, hover, hover, hover, 0.0, 0.0])
        return xr, ur

    def _advance_tilt(self, betacmd):
        # propagate the internal tilt-actuator estimate with the commanded tilt
        dt = 1.0 / self.rate
        for i in range(2):
            add = ((betacmd[i] - self.beta_est[i]) / self.cfg.tau_beta - self.betad_est[i]) / self.cfg.tau_beta
            self.betad_est[i] += add * dt
            self.beta_est[i] = clamp(self.beta_est[i] + self.betad_est[i] * dt,
                                     self.cfg.beta_min, self.cfg.beta_max)

    def control(self):
        if not self.have_state:
            return
        self._advance_governor()
        xref, uref = self._xref_uref()
        try:
            u0, ok, st = self.nmpc.solve(self.state, xref, uref, self.u_prev)
        except Exception as e:
            self.get_logger().warn(f"NMPC solve error: {e}")
            return
        if ok and np.all(np.isfinite(u0)):
            self.u_cmd = u0
            self.u_prev = u0
        # anti-lockup: a "converged" solve that collapses to ~zero collective while ARMED is
        # a bad local optimum (it then poisons the warm start -> stuck on the ground). Re-seed
        # the warm start at hover so the next solve escapes to the real (climb/hover) optimum.
        hov = self.cfg.mass * self.cfg.g / 4.0
        if self.armed and float(np.sum(self.u_cmd[0:4])) < 0.5 * self.cfg.mass * self.cfg.g:
            self.u_prev = np.array([hov, hov, hov, hov, self.beta_est[0], self.beta_est[1]])
            self.u_cmd = self.u_prev.copy()
        self._advance_tilt(self.u_cmd[4:6])

        if self.counter % max(1, int(self.rate)) == 0:
            s = self.state
            self.get_logger().info(
                f"z {s[0]:.2f},{s[1]:.2f},{s[2]:.2f}->{xref[0]:.2f},{xref[1]:.2f},{xref[2]:.2f} "
                f"| T=[{self.u_cmd[0]:.1f},{self.u_cmd[1]:.1f},{self.u_cmd[2]:.1f},{self.u_cmd[3]:.1f}]N "
                f"tilt=[{math.degrees(self.u_cmd[4]):.0f},{math.degrees(self.u_cmd[5]):.0f}]deg "
                f"solve={st} arm={self.armed}")

    def on_active(self, m):
        was = self._active
        self._active = (m.data == self.controller_id)
        if self._active != was:
            self.get_logger().info(
                f"controller '{self.controller_id}' -> {'ACTIVE' if self._active else 'standby'}")

    def heartbeat(self):
        self.counter += 1
        if not self._active:      # standby: do not touch PX4 (the other controller drives)
            return
        # offboard heartbeat
        m = OffboardControlMode(); m.direct_actuator = True; m.timestamp = self.now_us()
        self.ocm_pub.publish(m)
        # actuators (latest NMPC command), normalized to PX4 direct_actuator
        kf = self.cfg.max_thrust
        mot = [clamp(self.u_cmd[i] / kf, 0.0, 1.0) for i in range(4)]
        tilt = [clamp(self.u_cmd[4] / (math.pi / 2), 0.0, 1.0),
                clamp(self.u_cmd[5] / (math.pi / 2), 0.0, 1.0)]
        self._pub_motors(mot)
        self._pub_servos([tilt[0], tilt[1], 0.0, 0.0, 0.0])
        # auto offboard + arm (only once state feedback is flowing, so we never
        # arm into an open-loop hover before the NMPC is closing the loop)
        if (self.auto_arm and self.have_state and self.counter >= self.arm_after
                and not (self.armed and self.offb)):
            if (self.counter - self.last_cmd) >= 100:
                self.last_cmd = self.counter
                if not self.offb:
                    self.vcmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                elif not self.armed:
                    self.vcmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def _pub_motors(self, vals):
        m = ActuatorMotors(); c = [math.nan] * 12
        for i, v in enumerate(vals):
            c[i] = v
        m.control = c; m.timestamp = m.timestamp_sample = self.now_us(); self.mot_pub.publish(m)

    def _pub_servos(self, vals):
        m = ActuatorServos(); c = [math.nan] * 8
        for j, v in enumerate(vals):
            c[j] = v
        m.control = c; m.timestamp = m.timestamp_sample = self.now_us(); self.srv_pub.publish(m)

    def vcmd(self, command, p1=0.0, p2=0.0):
        m = VehicleCommand(); m.command = command; m.param1 = p1; m.param2 = p2
        m.target_system = m.target_component = m.source_system = m.source_component = 1
        m.from_external = True; m.timestamp = self.now_us(); self.cmd_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = NMPCQuadNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.vcmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        except Exception:
            pass
        node.destroy_node()
        rclpy.ok() and rclpy.shutdown()


if __name__ == "__main__":
    main()
