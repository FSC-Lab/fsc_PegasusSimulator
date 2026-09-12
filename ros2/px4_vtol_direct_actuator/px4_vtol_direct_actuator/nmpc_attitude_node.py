#!/usr/bin/env python3
"""
Inner-loop (attitude-setpoint) NMPC for the Isaac quadplane.

Unlike nmpc_quad_node.py (which streams raw actuators via direct_actuator and is
latency-limited in HIL), this node is the OUTER loop: it models the vehicle as a
double integrator driven by a thrust-acceleration vector, solves an NMPC to track
the position/velocity reference, and converts the optimal thrust vector into a
desired attitude quaternion + normalized collective thrust that it streams to
PX4's FAST onboard attitude/rate loop as VehicleAttitudeSetpoint. PX4 does the kHz
stabilization (same low-latency path that makes the fsc autopilot hover rock solid).

Interface matches the fsc autopilot exactly:
  OffboardControlMode(attitude=True, thrust_and_torque=True)  -> fmu/in/offboard_control_mode
  VehicleAttitudeSetpoint(q_d=[w,x,y,z] NED/FRD, thrust_body=[0,0,-t]) -> fmu/in/vehicle_attitude_setpoint

Frames: mocap is ENU world / FLU body; PX4 is NED world / FRD body. The desired
attitude is built in ENU then converted with getNEDqFromENUq (fsc's transform).

NMPC:  state x=[pos(3), vel(3)] (ENU world),  control u=a_t(3) = thrust accel
vector (ENU). vel_dot = a_t + g. Constraints: tilt of a_t <= max_tilt, and
a_min <= |a_t| <= a_max. Cost tracks pos/vel + regularizes a_t toward hover (0,0,g).

Deps: casadi, numpy, rclpy, px4_msgs, fsc_autopilot_ros2_msgs (self-contained).
"""
from __future__ import annotations

# ★ Run-as-a-script bootstrap. The sibling import below is absolute, which is
# correct for an installed package but needs the package PARENT on sys.path.
# That holds under `ros2 run` and `python3 -m`, not under `python3 this_file.py`,
# which is how the launchers invoke it. NOTE this block must sit AFTER the
# __future__ import -- that is required by the language to be the first statement,
# and ast.parse() does NOT flag a violation, only compile() does.
if __package__ in (None, ""):
    import os as _os, sys as _sys
    _sys.path.insert(0, _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__))))

import math
from dataclasses import dataclass

import numpy as np
import casadi as ca

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy,
                       QoSHistoryPolicy, qos_profile_sensor_data)
from rcl_interfaces.msg import ParameterDescriptor

from px4_msgs.msg import (OffboardControlMode, VehicleAttitudeSetpoint,
                          VehicleCommand, VehicleStatus)
from fsc_autopilot_ros2_msgs.msg import Mocap, PositionControllerReference
from std_msgs.msg import String
from px4_vtol_direct_actuator.smc_outer import SMCOuter, _KEYS      # outer-loop sliding-mode (shared)


# ======================================================================
@dataclass
class NMPCConfig:
    dt: float = 0.05
    N: int = 20
    n_rk4: int = 1
    mass: float = 3.4
    g: float = 9.81
    hover_throttle: float = 0.70        # PX4 thrust_body at hover. HIL-measured: the vehicle
                                        # marginally hovered at thr=0.70 with the old 0.46 (which
                                        # under-commanded -> thrust-limited, stuck at z~0.3 despite
                                        # a_t=15). 0.70 makes hover correct + leaves climb margin to 1.0.
    max_tilt_deg: float = 18.0          # cap commanded tilt (18 keeps it out of the
                                        # bang-bang limit cycle seen at 30 in HIL)
    a_min: float = 7.0                  # min thrust accel [m/s^2] (~0.7 g). Was 3.0 (~0.3g) which let
                                        # the down-swing NEAR-FREE-FALL to the ground -> vertical limit
                                        # cycle (z 0.1<->1.3). 0.7g caps descent to ~0.3g net -> gentle.
    a_max: float = 15.0                 # max thrust accel [m/s^2] (~1.5 g)
    # cost weights — detuned for real HIL latency (gentle position, strong damping)
    w_pos: float = 5.0
    w_vel: float = 20.0                 # strong velocity damping to kill the vertical limit cycle
    w_a: float = 0.5                    # thrust-accel deviation from hover
    w_du: float = 2.0                   # input-rate (smoothness)
    w_terminal: float = 6.0
    pos_scale: float = 1.0
    vel_scale: float = 1.5
    a_scale: float = 9.81
    # ipopt
    max_iter: int = 40
    tol: float = 1e-4
    acceptable_tol: float = 1e-3
    print_level: int = 0


# ======================================================================
class AttitudeNMPC:
    NX = 6
    NU = 3

    def __init__(self, cfg: NMPCConfig):
        self.cfg = cfg
        self._solver = None
        self._prev_w = None
        self._prev_lam_g = None
        self._prev_lam_x = None

    def _F(self):
        c = self.cfg
        x = ca.SX.sym("x", 6); u = ca.SX.sym("u", 3)
        g_enu = ca.vertcat(0, 0, -c.g)
        # vel_dot = a_t + g_enu  (a_t = thrust acceleration vector, ENU)
        def ode2(x, u):
            return ca.vertcat(x[3:6], u + g_enu)
        h = c.dt / c.n_rk4
        xc = x
        for _ in range(c.n_rk4):
            k1 = ode2(xc, u); k2 = ode2(xc + h/2*k1, u)
            k3 = ode2(xc + h/2*k2, u); k4 = ode2(xc + h*k3, u)
            xc = xc + h/6*(k1 + 2*k2 + 2*k3 + k4)
        return ca.Function("F", [x, u], [xc])

    def build(self):
        c = self.cfg
        N, nx, nu = c.N, self.NX, self.NU
        F = self._F()
        tan_tilt = math.tan(math.radians(c.max_tilt_deg))
        a_hover = ca.DM([0, 0, c.g])

        # params: [x0(6), xref(6), uprev(3)]  (constant set-point + ff velocity)
        p = ca.SX.sym("p", nx + nx + nu)
        x0p, xref, uprev = p[0:nx], p[nx:2*nx], p[2*nx:2*nx+nu]

        w, wlb, wub, g, glb, gub = [], [], [], [], [], []
        J = ca.SX(0); X, U = [], []
        xlb = [-1e6]*nx; xub = [1e6]*nx
        # u bounds: x,y free (tilt constrained below); z (up thrust accel) in [a_min? , a_max]
        ulb = [-c.a_max, -c.a_max, 0.0]
        uub = [c.a_max, c.a_max, c.a_max]

        for k in range(N+1):
            xk = ca.SX.sym(f"x_{k}", nx); X.append(xk); w.append(xk); wlb += xlb; wub += xub
        for k in range(N):
            uk = ca.SX.sym(f"u_{k}", nu); U.append(uk); w.append(uk); wlb += ulb; wub += uub

        g.append(X[0] - x0p); glb += [0.0]*nx; gub += [0.0]*nx

        for k in range(N):
            xk, uk = X[k], U[k]
            uprev_k = uprev if k == 0 else U[k-1]
            e_pos = (xk[0:3]-xref[0:3])/c.pos_scale
            e_vel = (xk[3:6]-xref[3:6])/c.vel_scale
            e_a = (uk - a_hover)/c.a_scale
            du = (uk - uprev_k)/c.a_scale
            J += (c.w_pos*ca.dot(e_pos, e_pos) + c.w_vel*ca.dot(e_vel, e_vel)
                  + c.w_a*ca.dot(e_a, e_a) + c.w_du*ca.dot(du, du))
            # dynamics
            g.append(X[k+1]-F(xk, uk)); glb += [0.0]*nx; gub += [0.0]*nx
            # tilt cone: ax^2+ay^2 <= (tan_tilt*az)^2  ->  (tan_tilt*az)^2-ax^2-ay^2 >= 0
            g.append((tan_tilt*uk[2])**2 - uk[0]**2 - uk[1]**2); glb += [0.0]; gub += [1e9]
            # thrust magnitude: a_min^2 <= |a|^2 <= a_max^2
            g.append(uk[0]**2+uk[1]**2+uk[2]**2); glb += [c.a_min**2]; gub += [c.a_max**2]
        eN_p = (X[N][0:3]-xref[0:3])/c.pos_scale
        eN_v = (X[N][3:6]-xref[3:6])/c.vel_scale
        J += c.w_terminal*(c.w_pos*ca.dot(eN_p, eN_p) + c.w_vel*ca.dot(eN_v, eN_v))

        nlp = {"x": ca.vertcat(*w), "f": J, "g": ca.vertcat(*g), "p": p}
        opts = {"ipopt.max_iter": c.max_iter, "ipopt.tol": c.tol,
                "ipopt.acceptable_tol": c.acceptable_tol,
                "ipopt.print_level": c.print_level, "print_time": 0,
                "ipopt.warm_start_init_point": "yes", "ipopt.mu_strategy": "adaptive",
                "error_on_fail": False}
        self._solver = ca.nlpsol("att_nmpc", "ipopt", nlp, opts)
        self._wlb = np.array(wlb); self._wub = np.array(wub)
        self._glb = np.array(glb); self._gub = np.array(gub); self._nw = len(wlb)

    def solve(self, x0, xref, u_prev):
        c = self.cfg; N, nx, nu = c.N, self.NX, self.NU
        p = np.concatenate([x0, xref, u_prev])
        if self._prev_w is not None and len(self._prev_w) == self._nw:
            w0 = self._prev_w
        else:
            w0 = np.zeros(self._nw)
            for k in range(N+1):
                w0[k*nx:(k+1)*nx] = xref
            off = (N+1)*nx
            for k in range(N):
                w0[off+k*nu:off+(k+1)*nu] = [0, 0, c.g]
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
        u0 = wsol[(N+1)*nx:(N+1)*nx+nu]
        return u0, ok, st


# ======================================================================
#  math helpers
# ======================================================================
def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def quat_mul(a, b):
    w1, x1, y1, z1 = a; w2, x2, y2, z2 = b
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2])


# fsc getNEDqFromENUq: q_ned = q(Rie) * q_enu * q(Rbv)
# Rie=[[0,1,0],[1,0,0],[0,0,-1]] -> q=(0, √2/2, √2/2, 0);  Rbv=180 about x -> q=(0,1,0,0)
_S2 = math.sqrt(2) / 2
_Q_RIE = np.array([0.0, _S2, _S2, 0.0])
_Q_RBV = np.array([0.0, 1.0, 0.0, 0.0])


def enu_quat_to_ned(q_enu):  # q in [w,x,y,z]
    return quat_mul(quat_mul(_Q_RIE, q_enu), _Q_RBV)


def R_to_quat(R):
    """Rotation matrix -> quaternion [w,x,y,z] (robust)."""
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / s; x = 0.25*s
        y = (R[0, 1] + R[1, 0]) / s; z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / s; x = (R[0, 1] + R[1, 0]) / s
        y = 0.25*s; z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / s; x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s; z = 0.25*s
    q = np.array([w, x, y, z])
    return q / np.linalg.norm(q)


def thrust_vec_to_ned_setpoint(a_t, yaw_enu, hover_throttle, g):
    """thrust accel vector (ENU) + desired ENU yaw -> (q_ned[w,x,y,z], t_norm)."""
    a = np.asarray(a_t, float)
    n = np.linalg.norm(a)
    z_b = a / n if n > 1e-6 else np.array([0.0, 0.0, 1.0])   # body-up in ENU
    x_c = np.array([math.cos(yaw_enu), math.sin(yaw_enu), 0.0])
    y_b = np.cross(z_b, x_c)
    ny = np.linalg.norm(y_b)
    y_b = y_b/ny if ny > 1e-6 else np.array([0.0, 1.0, 0.0])
    x_b = np.cross(y_b, z_b)
    R_enu = np.column_stack((x_b, y_b, z_b))            # FLU body -> ENU world
    q_enu = R_to_quat(R_enu)
    q_ned = enu_quat_to_ned(q_enu)
    t_norm = clamp(hover_throttle * n / g, 0.0, 1.0)
    return q_ned, t_norm


# ======================================================================
#  ROS 2 node
# ======================================================================
class NMPCAttitudeNode(Node):
    def __init__(self):
        super().__init__("nmpc_attitude_control")
        dyn = ParameterDescriptor(dynamic_typing=True)
        P = self.declare_parameter
        P("px4_ns", "uav_0"); P("auto_arm", True); P("arm_after", 100)
        P("rate_hz", 50.0, dyn)
        P("sp_max_vel", 1.0, dyn); P("sp_max_climb", 0.8, dyn); P("sp_max_yawrate_deg", 60.0, dyn)
        # OUTER-LOOP sliding-mode (loose transition / strict landing). smc_on -> use SMC
        # instead of the plain bounded-ramp governor.
        P("smc_on", True)
        P("smc_land_z", 1.2, dyn); P("smc_reset_dist", 8.0, dyn)
        P("smc_lam", 0.9, dyn); P("smc_K", 1.5, dyn); P("smc_phi", 2.0, dyn)
        P("smc_vmax", 20.0, dyn); P("smc_vzmax", 3.0, dyn)
        P("smc_lam_land", 1.6, dyn); P("smc_K_land", 0.6, dyn); P("smc_phi_land", 0.4, dyn)
        P("smc_vmax_land", 1.5, dyn); P("smc_vzmax_land", 0.6, dyn)
        # controller arbitration (GS inner-loop switch): only the ACTIVE controller drives PX4
        P("controller_id", "attnmpc"); P("active_at_start", True)
        # px4 collective at hover (thrust_body). MEASURED ~0.50 here; 0.70 over-scaled the
        # thrust so B could never command a descent (a_min floor pinned it at hover -> climb).
        P("hover_throttle", 0.50, dyn)
        ns = self.get_parameter("px4_ns").get_parameter_value().string_value
        pre = f"/{ns}" if ns else ""
        self.rate = float(self.get_parameter("rate_hz").value)
        self.auto_arm = bool(self.get_parameter("auto_arm").value)
        self.arm_after = int(self.get_parameter("arm_after").value)
        self.sp_max_vel = float(self.get_parameter("sp_max_vel").value)
        self.sp_max_climb = float(self.get_parameter("sp_max_climb").value)
        self.sp_max_yawrate = math.radians(float(self.get_parameter("sp_max_yawrate_deg").value))

        self.cfg = NMPCConfig()
        self.get_logger().info("building attitude NMPC (CasADi/ipopt)...")
        self.nmpc = AttitudeNMPC(self.cfg); self.nmpc.build()
        self.get_logger().info("NMPC ready.")

        self.state = None; self.have_state = False
        self.ref = {"x": 0.0, "y": 0.0, "z": 1.5, "yaw": 0.0}
        self.sp = None; self.sp_vff = np.zeros(3)
        self.smc = SMCOuter(**{k: float(self.get_parameter("smc_" + k).value) for k in _KEYS})
        self.cid = self.get_parameter("controller_id").value
        self._active = bool(self.get_parameter("active_at_start").value)
        self.u_prev = np.array([0.0, 0.0, self.cfg.g])
        self.q_cmd = np.array([1.0, 0.0, 0.0, 0.0]); self.t_cmd = 0.0
        self.armed = self.offb = False; self.counter = self.last_cmd = 0

        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         durability=QoSDurabilityPolicy.VOLATILE,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        latch = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                           durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                           history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        self.ocm_pub = self.create_publisher(OffboardControlMode, f"{pre}/fmu/in/offboard_control_mode", qos)
        self.att_pub = self.create_publisher(VehicleAttitudeSetpoint, f"{pre}/fmu/in/vehicle_attitude_setpoint", qos)
        self.cmd_pub = self.create_publisher(VehicleCommand, f"{pre}/fmu/in/vehicle_command", qos)
        self.create_subscription(VehicleStatus, f"{pre}/fmu/out/vehicle_status_v1", self.on_status, latch)
        self.create_subscription(Mocap, f"{pre}/mocap", self.on_mocap, qos_profile_sensor_data)
        self.create_subscription(PositionControllerReference,
                                 f"{pre}/fsc_autopilot_ros2/position_controller/reference", self.on_ref, 10)
        self.create_subscription(String, f"{pre}/ground_station/active_controller", self.on_active, 10)

        self.create_timer(0.01, self.heartbeat)      # 100 Hz OCM + attitude + arm
        self.create_timer(1.0/self.rate, self.control)
        self.get_logger().info(f"nmpc_attitude_control up ns='{ns}' solve={self.rate:.0f}Hz (inner-loop)")

    def now_us(self):
        return int(self.get_clock().now().nanoseconds/1000)

    def on_status(self, m):
        self.armed = (m.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        self.offb = (m.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD)

    def on_ref(self, m):
        self.ref["x"], self.ref["y"], self.ref["z"] = m.position.x, m.position.y, m.position.z
        self.ref["yaw"] = (m.yaw*math.pi/180.0
                           if m.yaw_unit == PositionControllerReference.DEGREES else m.yaw)

    def on_active(self, m):
        self._active = (m.data == self.cid)

    def on_mocap(self, m):
        x = np.zeros(6)
        x[0:3] = [m.pose.position.x, m.pose.position.y, m.pose.position.z]
        x[3:6] = [m.twist.linear.x, m.twist.linear.y, m.twist.linear.z]   # world ENU vel
        self.state = x; self.have_state = True

    def _governor(self):
        dt = 1.0/self.rate
        if self.sp is None:
            s = self.state; self.sp = {"x": s[0], "y": s[1], "z": s[2], "yaw": self.ref["yaw"]}
        r, sp = self.ref, self.sp
        if bool(self.get_parameter("smc_on").value):
            # OUTER-LOOP sliding-mode: converging set-point + robust velocity feed-forward,
            # loose (transition) / strict (landing) by target altitude.
            self.smc.set_params(**{k: float(self.get_parameter("smc_" + k).value) for k in _KEYS})
            sp_pos, v_cmd = self.smc.update(self.state[0:3], [r["x"], r["y"], r["z"]], dt)
            sp["x"], sp["y"], sp["z"] = float(sp_pos[0]), float(sp_pos[1]), float(sp_pos[2])
            self.sp_vff = v_cmd
        else:                                           # plain bounded-ramp governor (fallback)
            ex, ey, ez = r["x"]-sp["x"], r["y"]-sp["y"], r["z"]-sp["z"]
            dh = math.hypot(ex, ey); step_h = min(dh, self.sp_max_vel*dt); vx = vy = 0.0
            if dh > 1e-6:
                vx, vy = self.sp_max_vel*ex/dh, self.sp_max_vel*ey/dh
                sp["x"] += step_h*ex/dh; sp["y"] += step_h*ey/dh
                if step_h < self.sp_max_vel*dt:
                    vx, vy = ex/dt, ey/dt
            step_z = clamp(ez, -self.sp_max_climb*dt, self.sp_max_climb*dt)
            sp["z"] += step_z; vz = step_z/dt
            self.sp_vff = np.array([vx, vy, vz])
        dyaw = math.atan2(math.sin(r["yaw"]-sp["yaw"]), math.cos(r["yaw"]-sp["yaw"]))
        sp["yaw"] += clamp(dyaw, -self.sp_max_yawrate*dt, self.sp_max_yawrate*dt)

    def control(self):
        if not self.have_state:
            return
        self._governor()
        xref = np.zeros(6)
        xref[0:3] = [self.sp["x"], self.sp["y"], self.sp["z"]]
        xref[3:6] = self.sp_vff
        try:
            a_t, ok, st = self.nmpc.solve(self.state, xref, self.u_prev)
        except Exception as e:
            self.get_logger().warn(f"NMPC solve error: {e}"); return
        if ok and np.all(np.isfinite(a_t)):
            self.u_prev = a_t
            self.q_cmd, self.t_cmd = thrust_vec_to_ned_setpoint(
                a_t, self.sp["yaw"], float(self.get_parameter("hover_throttle").value), self.cfg.g)
        if self.counter % max(1, int(self.rate)) == 0:
            s = self.state
            self.get_logger().info(
                f"pos({s[0]:.2f},{s[1]:.2f},{s[2]:.2f})->({xref[0]:.2f},{xref[1]:.2f},{xref[2]:.2f}) "
                f"a_t=[{a_t[0]:.1f},{a_t[1]:.1f},{a_t[2]:.1f}] thr={self.t_cmd:.2f} "
                f"tilt={math.degrees(math.atan2(math.hypot(a_t[0],a_t[1]),max(a_t[2],1e-3))):.0f}deg "
                f"{st} arm={self.armed} offb={self.offb}")

    def heartbeat(self):
        self.counter += 1
        if not self._active:
            return                       # standby: another inner loop is driving PX4
        m = OffboardControlMode(); m.attitude = True; m.thrust_and_torque = True
        m.timestamp = self.now_us(); self.ocm_pub.publish(m)
        a = VehicleAttitudeSetpoint(); a.timestamp = self.now_us()
        a.q_d = [float(v) for v in self.q_cmd]
        a.thrust_body = [0.0, 0.0, -float(self.t_cmd)]
        self.att_pub.publish(a)
        if (self.auto_arm and self.have_state and self.counter >= self.arm_after
                and not (self.armed and self.offb)):
            if (self.counter - self.last_cmd) >= 100:
                self.last_cmd = self.counter
                if not self.offb:
                    self.vcmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                elif not self.armed:
                    self.vcmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def vcmd(self, command, p1=0.0, p2=0.0):
        m = VehicleCommand(); m.command = command; m.param1 = p1; m.param2 = p2
        m.target_system = m.target_component = m.source_system = m.source_component = 1
        m.from_external = True; m.timestamp = self.now_us(); self.cmd_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = NMPCAttitudeNode()
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
