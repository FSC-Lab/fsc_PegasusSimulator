#!/usr/bin/env python3
"""geometric_hover_node.py — the SAFE controller. One job: go to (x, y, z, yaw) and hold.

    GS "GO / HOLD at X,Y,Z"  ->  /<ns>/.../position_controller/reference  ->  this node

ALWAYS A MULTICOPTER. The rotors are vertical at all times (tilt servo = 0) and the
control surfaces are neutral. Forward motion is by LEANING, like any quadrotor. It has
no tilt schedule, no aerodynamic model, no transition logic and no trajectory clock,
and it is the controller the STOP button hands the vehicle to. Trajectory following,
path tracking and the hover<->cruise transition belong to the NMPC nodes; this one is
what you fall back to when those go wrong, so its behaviour must be boring.

  target (x,y,z,yaw)          <- whatever the operator last typed / clicked
    |  velocity-limited setpoint generator (this node, internal)
    v  smooth p_sp, v_ff, a_ff -- self-consistent by construction
  position PID + feed-forward  -> desired specific force (world)
    |  lean cap
    v  SE(3) attitude target (yaw from the target) -> geometric attitude law -> moments
  allocation [f, Mx, My, Mz]   -> 4 rotor thrusts

WHY THE SETPOINT GENERATOR IS IN HERE (2026-09-06 streamline). This node used to
DIFFERENTIATE an incoming reference stream to get its feed-forward, with a hardcoded
30 Hz assumption, and it was fed by a separate sliding-mode node on a second topic
(reference_smc) that the ground station ALSO published to -- two publishers on one
topic, and an SMC whose re-seed condition fired on ordinary tracking lag (the 4-second
setpoint sawtooth). All of that existed to turn an operator's step input into something
smooth. A velocity-limited ramp does the same job in twenty lines, cannot re-seed, and
produces position, velocity AND acceleration that agree with each other exactly.

Output: ActuatorMotors + ActuatorServos + OffboardControlMode(direct_actuator).
Gated on /<ns>/ground_station/active_controller == controller_id.
State:  /<ns>/mocap (pose world, twist.linear world vel, twist.angular BODY rate).
"""
# ★ Run-as-a-script bootstrap. The imports below are absolute
# (`from px4_vtol_direct_actuator...`), which is correct for an installed
# package but needs the package's PARENT on sys.path. That is true under
# `ros2 run` and `python3 -m`, and false under `python3 this_file.py` -- which
# is how every launcher and every offline gate in this project invokes it.
# Adding the parent here keeps both modes working without a global path shim.
if __package__ in (None, ""):
    import os as _os, sys as _sys
    _sys.path.insert(0, _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__))))
import math
import os
import sys
import traceback

# ★ SHARED PROPELLER MODEL. The sim's FSC airframe runs thrust_model="momentum": rotor thrust is
# QUADRATIC in throttle and falls with airspeed. The legacy `T / max_thrust` mapping asks 0.47 for
# a hover that needs 0.68 -- HALF the weight -- so this controller could not lift the aircraft,
# and it is the one every failure falls back to. Imported rather than copied so it cannot drift.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
try:
    from px4_vtol_direct_actuator import nmpc_so3_transition_node as _prop
    _HAVE_PROP = True
except Exception:                                  # noqa: BLE001 - never stop the SAFE controller
    _prop = None
    _HAVE_PROP = False

import numpy as np

# ROS is imported inside run_ros() so this module can be imported -- and its control law
# tested against a plant double -- with nothing but numpy. See tests/test_geometric_hover.py.


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def wrap_pi(a):
    return math.atan2(math.sin(a), math.cos(a))


def quat_to_R(x, y, z, w):
    """ENU/FLU body->world rotation from quaternion [x,y,z,w]."""
    n = math.sqrt(x * x + y * y + z * z + w * w) + 1e-12
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


def vee(M):
    return np.array([M[2, 1], M[0, 2], M[1, 0]])


class SetpointRamp:
    """Turn a step target into a smooth, overshoot-free setpoint with matching v and a.

    Horizontal and vertical are limited separately (a quadrotor climbs and descends at
    very different rates from how it translates). The approach speed follows a sqrt
    stopping profile, so the setpoint decelerates INTO the target and stops there
    instead of ringing around it.
    """

    def __init__(self):
        self.p = None                 # position setpoint [m], world ENU
        self.v = np.zeros(3)          # its velocity [m/s]
        self.a = np.zeros(3)          # its acceleration [m/s^2]
        self.yaw = 0.0
        self.yaw_rate = 0.0

    def seed(self, p, yaw):
        """Start the setpoint at the vehicle, so engaging never commands a jump."""
        self.p = np.array(p, dtype=float)
        self.v[:] = 0.0
        self.a[:] = 0.0
        self.yaw = float(yaw)
        self.yaw_rate = 0.0

    def step(self, target, yaw_target, dt, v_max, vz_up, vz_dn, a_max, az_max, yaw_rate_max):
        if self.p is None:
            self.seed(target, yaw_target)
            return self.p.copy(), self.v.copy(), self.a.copy(), self.yaw
        v_prev = self.v.copy()
        tgt = np.asarray(target, dtype=float)

        # ---- horizontal: sqrt stopping profile toward the target ----
        e = tgt[0:2] - self.p[0:2]
        d = float(np.linalg.norm(e))
        if d > 1e-6:
            speed = min(v_max, math.sqrt(2.0 * a_max * d))
            v_want = e / d * speed
        else:
            v_want = np.zeros(2)
        dv = v_want - self.v[0:2]
        dv_max = a_max * dt
        n = float(np.linalg.norm(dv))
        if n > dv_max:
            dv *= dv_max / n
        self.v[0:2] += dv
        self.p[0:2] += self.v[0:2] * dt
        # never step past the target (a sqrt profile can overshoot by one sample)
        if float(np.dot(tgt[0:2] - self.p[0:2], e)) < 0.0:
            self.p[0:2] = tgt[0:2]
            self.v[0:2] = 0.0

        # ---- vertical: asymmetric climb/descent limits ----
        ez = float(tgt[2] - self.p[2])
        vz_cap = vz_up if ez > 0.0 else vz_dn
        vz_want = math.copysign(min(vz_cap, math.sqrt(2.0 * az_max * abs(ez))), ez) if abs(ez) > 1e-6 else 0.0
        self.v[2] += clamp(vz_want - self.v[2], -az_max * dt, az_max * dt)
        self.p[2] += self.v[2] * dt
        if (tgt[2] - self.p[2]) * ez < 0.0:
            self.p[2] = float(tgt[2])
            self.v[2] = 0.0

        # ---- yaw: slew the shortest way round ----
        eyaw = wrap_pi(yaw_target - self.yaw)
        step = clamp(eyaw, -yaw_rate_max * dt, yaw_rate_max * dt)
        self.yaw = wrap_pi(self.yaw + step)
        self.yaw_rate = step / dt if dt > 1e-9 else 0.0

        self.a = (self.v - v_prev) / dt if dt > 1e-9 else np.zeros(3)
        return self.p.copy(), self.v.copy(), self.a.copy(), self.yaw


class Gains:
    """Plain container so the control law can be exercised without ROS parameters."""

    def __init__(self, **kw):
        self.kp_xy = 3.0; self.kv_xy = 3.5
        self.kp_z = 6.0;  self.kv_z = 5.0
        self.ki_xy = 0.4; self.ki_z = 0.8
        self.i_max = 8.0; self.v_integral = 0.5
        self.kR = 9.0;    self.kW = 1.8
        self.kR_yaw = 2.0; self.kW_yaw = 0.5
        self.max_tilt_att_deg = 35.0
        self.mass = 3.45; self.gravity = 9.81; self.max_thrust = 18.0
        self.cg_x = 0.176; self.cQ = 0.12
        for k, v in kw.items():
            setattr(self, k, float(v))


def allocation_matrix(cg_x, cQ):
    """[f, Mx, My, Mz] = A @ [T_FL, T_FR, T_RL, T_RR] for the FSC quad-tiltrotor."""
    rp = np.array([[0.459 - cg_x, 0.285], [0.459 - cg_x, -0.284],
                   [-0.112 - cg_x, 0.284], [-0.112 - cg_x, -0.284]])
    spin = np.array([+1.0, -1.0, -1.0, +1.0])
    A = np.zeros((4, 4))
    A[0, :] = 1.0                  # collective
    A[1, :] = rp[:, 1]             # roll  =  y * Fz
    A[2, :] = -rp[:, 0]            # pitch = -x * Fz
    A[3, :] = -spin * cQ           # yaw   = -spin * cQ * Fz
    return A


def geometric_command(p, v, R, w, p_sp, v_ff, a_ff, yaw_sp, yaw_rate_sp, ei, dt,
                      gains, Ainv, integrate=True):
    """The whole control law, as a pure function.

    Returns (motor_commands[4], ei, diag). `ei` is the position integral, threaded by the
    caller so this stays free of hidden state and can be stepped by a test harness.
    """
    g, m_, kf = gains.gravity, gains.mass, gains.max_thrust
    kp = np.array([gains.kp_xy, gains.kp_xy, gains.kp_z])
    kv = np.array([gains.kv_xy, gains.kv_xy, gains.kv_z])
    ki = np.array([gains.ki_xy, gains.ki_xy, gains.ki_z])
    e_p = p - p_sp
    e_v = v - v_ff
    ei_prev = np.array(ei, dtype=float)
    ei = ei_prev.copy()
    # ANTI-WINDUP is by CLAMP + BACK-CALCULATION and by freezing while the rotors are
    # saturated (below) -- NOT by an error band. The old band-limited integral (only
    # integrate within 1 m) existed because this node used to chase a fast external
    # trajectory, where the error was mostly reference lag. It now chases its OWN
    # velocity-limited ramp with matching feed-forward, so the tracking error IS the
    # disturbance and must be integrated -- a band would refuse to correct exactly the
    # large steady offsets (wind, trim) the integral is for.
    #
    # The integral also FADES OUT while the setpoint is moving. What it accumulates in
    # transit is mostly aerodynamic drag, which disappears the moment the vehicle stops --
    # integrating it just charges up a lean that then has to discharge, which is exactly
    # the overshoot at the end of a long move. The integral is for STATIC trim (wind, CG),
    # so weight it toward the stationary case.
    if integrate:
        v_sp = float(np.linalg.norm(v_ff))
        fade = 1.0 / (1.0 + (v_sp / max(gains.v_integral, 1e-3)) ** 2)
        ei += e_p * dt * fade
    else:
        ei[:] = 0.0
    i_force = np.clip(ki * ei, -gains.i_max, gains.i_max)
    nz = ki > 1e-9
    ei[nz] = i_force[nz] / ki[nz]                 # back-calculate: no windup past the clamp

    e3 = np.array([0.0, 0.0, 1.0])
    F_des = m_ * (g * e3 + a_ff) - kp * e_p - kv * e_v - i_force
    F_des[2] = max(F_des[2], 0.2 * m_ * g)

    max_lean = math.radians(gains.max_tilt_att_deg)
    fz = F_des[2]; fh = math.hypot(F_des[0], F_des[1])
    if fh > fz * math.tan(max_lean):
        sc = fz * math.tan(max_lean) / (fh + 1e-9)
        F_des[0] *= sc; F_des[1] *= sc

    b3 = F_des / (np.linalg.norm(F_des) + 1e-9)
    b1c = np.array([math.cos(yaw_sp), math.sin(yaw_sp), 0.0])
    b2 = np.cross(b3, b1c); b2 /= (np.linalg.norm(b2) + 1e-9)
    b1 = np.cross(b2, b3)
    R_des = np.column_stack((b1, b2, b3))
    f = float(F_des @ (R @ e3))

    eR = 0.5 * vee(R_des.T @ R - R.T @ R_des)
    eW = w - np.array([0.0, 0.0, yaw_rate_sp])
    kR = np.array([gains.kR, gains.kR, gains.kR_yaw])
    kW = np.array([gains.kW, gains.kW, gains.kW_yaw])
    M = -kR * eR - kW * eW

    T = Ainv @ np.array([f, M[0], M[1], M[2]])
    # attitude priority: shift the COLLECTIVE until the set fits, so a saturated rotor
    # costs altitude rather than corrupting the commanded moment.
    lo, hi = float(np.min(T)), float(np.max(T))
    saturated = (lo < 0.0) or (hi > kf)
    if saturated:
        shift = -lo if lo < 0.0 else 0.0
        if hi + shift > kf:
            shift = min(shift, kf - hi)
        T = np.clip(T + shift, 0.0, kf)
        ei = ei_prev                              # conditional integration: freeze while saturated
    # ★ NEWTONS -> THROTTLE through the propeller model. `T/kf` is the flat mapping and is
    # wrong against thrust_model="momentum" in two ways (quadratic, and speed-dependent). This
    # controller is a PURE MULTICOPTER -- the tilt servos are literal zeros -- so every rotor is
    # vertical and its axial inflow is the body-z velocity, which is ~0 in hover and small in the
    # climbs and descents this controller flies.
    # body-frame velocity: this function is given the WORLD velocity `v`, and a vertical rotor's
    # axial inflow is the BODY z component. (Referencing a `vb` that does not exist here threw
    # NameError every tick; the loop is exception-wrapped, so the controller silently held level
    # hover and the aircraft simply would not climb.)
    v_ax = float((R.T @ np.asarray(v, float))[2])
    if _HAVE_PROP:
        mot = [clamp(_prop.thrust_to_throttle(float(T[i]), v_ax), 0.0, 1.0) for i in range(4)]
    else:
        mot = [clamp(float(T[i]) / kf, 0.0, 1.0) for i in range(4)]
    return mot, ei, dict(F_des=F_des, f=f, M=M, eR=eR, e_p=e_p, T=T, saturated=saturated)


def run_ros():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy,
                           QoSHistoryPolicy, qos_profile_sensor_data)
    from px4_msgs.msg import (OffboardControlMode, ActuatorMotors, ActuatorServos,
                              VehicleCommand, VehicleStatus)
    from std_msgs.msg import String
    from fsc_autopilot_ros2_msgs.msg import Mocap, PositionControllerReference

    class GeometricHover(Node):
        def __init__(self):
            super().__init__("geometric_hover_control")
            P = self.declare_parameter
            P("px4_ns", "uav_0")
            P("controller_id", "geometric")
            P("active_at_start", True)
            # DEFAULT OFF: arming is the OPERATOR's, given through the ground station's
            # ARM / OFFBOARD / TAKEOFF buttons.  With this on the node re-arms whenever it
            # is active, which silently undoes a DISARM and arms the aircraft without
            # anyone asking.  (It exists because the first ARMs after a restart are
            # rejected while the EKF settles -- with it off you may need to press ARM twice.)
            P("auto_arm", False)
            P("arm_after", 150)
            P("rate_hz", 200.0)
            P("mass", 3.45); P("gravity", 9.81); P("max_thrust", 18.0)
            # --- setpoint ramp: how fast the operator's step target is approached ---
            P("v_max", 4.0); P("vz_up", 2.0); P("vz_down", 1.0)
            P("a_max", 1.5); P("az_max", 1.5); P("yaw_rate_deg_s", 45.0)
            # --- position loop ---
            P("kp_xy", 3.0); P("kv_xy", 3.5); P("kp_z", 6.0); P("kv_z", 5.0)
            P("ki_xy", 0.4); P("ki_z", 0.8); P("i_max", 8.0); P("v_integral", 0.5)
            P("max_tilt_att_deg", 35.0)
            # --- attitude loop ---
            P("kR", 9.0); P("kW", 1.8); P("kR_yaw", 2.0); P("kW_yaw", 0.5)
            # --- airframe: cg_x 0.176 = the ASSEMBLED vehicle CG (the body link sits at
            #     0.195, plus ten 0.05 kg articulated links); cQ 0.12 = the sim's
            #     rotor_moment_ratio. Both were wrong here (0.195 / 0.06) -- audit C3/C12.
            P("cg_x", 0.176); P("cQ", 0.12)
            # --- envelope on the accepted target ---
            P("z_min", 0.0); P("z_max", 60.0); P("radius_max", 300.0)
            # a target this far from the VEHICLE is always accepted, wherever it is
            P("radius_local", 100.0)

            ns = self.get_parameter("px4_ns").value
            pre = f"/{ns}" if ns else ""
            self.cid = self.get_parameter("controller_id").value
            self._active = bool(self.get_parameter("active_at_start").value)
            self.auto_arm = bool(self.get_parameter("auto_arm").value)
            self.arm_after = int(self.get_parameter("arm_after").value)
            self.rate = float(self.gp("rate_hz"))
            self.kf = self.gp("max_thrust"); self.m = self.gp("mass"); self.g = self.gp("gravity")
            self.Ainv = np.linalg.pinv(allocation_matrix(self.gp("cg_x"), self.gp("cQ")))

            self.state = None            # (p, v_world, R, omega_body)
            self.target = None           # world position the operator asked for
            self.yaw_target = 0.0
            self.ramp = SetpointRamp()
            self.ei = np.zeros(3)
            self.armed = self.offb = False
            self.counter = self.last = 0
            self._died = False; self._rejected = 0

            qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                             history=QoSHistoryPolicy.KEEP_LAST, depth=1)
            self.ocm = self.create_publisher(OffboardControlMode, f"{pre}/fmu/in/offboard_control_mode", qos)
            self.motp = self.create_publisher(ActuatorMotors, f"{pre}/fmu/in/actuator_motors", qos)
            self.srvp = self.create_publisher(ActuatorServos, f"{pre}/fmu/in/actuator_servos", qos)
            self.cmdp = self.create_publisher(VehicleCommand, f"{pre}/fmu/in/vehicle_command", qos)
            self.create_subscription(VehicleStatus, f"{pre}/fmu/out/vehicle_status_v1", self.on_status, qos)
            self.create_subscription(Mocap, f"{pre}/mocap", self.on_mocap, qos_profile_sensor_data)
            self.create_subscription(PositionControllerReference,
                                     f"{pre}/fsc_autopilot_ros2/position_controller/reference",
                                     self.on_ref, 10)
            self.create_subscription(String, f"{pre}/ground_station/active_controller", self.on_active, 10)
            self.create_timer(1.0 / self.rate, self.loop)
            self.get_logger().info(
                f"geometric_hover up: id={self.cid} rate={self.rate:.0f}Hz "
                f"v_max={self.gp('v_max'):.1f} m/s -- MULTICOPTER ONLY (tilt hard-wired to 0)")

        # ---------------- plumbing ----------------
        def gp(self, n):
            return float(self.get_parameter(n).value)

        def gains(self):
            return Gains(**{k: self.gp(k) for k in
                            ("kp_xy", "kv_xy", "kp_z", "kv_z", "ki_xy", "ki_z", "i_max", "v_integral",
                             "kR", "kW", "kR_yaw", "kW_yaw", "max_tilt_att_deg",
                             "mass", "gravity", "max_thrust", "cg_x", "cQ")})

        def now_us(self):
            return int(self.get_clock().now().nanoseconds / 1000)

        def on_status(self, m):
            self.armed = (m.arming_state == VehicleStatus.ARMING_STATE_ARMED)
            self.offb = (m.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD)

        def on_active(self, m):
            was = self._active
            self._active = (m.data == self.cid)
            if self._active and not was:
                # engaging: restart the ramp AT the vehicle, so there is never a step
                if self.state is not None:
                    yaw = math.atan2(self.state[2][1, 0], self.state[2][0, 0])
                    self.ramp.seed(self.state[0], yaw)
                    self.target = self.state[0].copy(); self.yaw_target = yaw
                self.ei[:] = 0.0
                self.get_logger().info("ACTIVE -- holding position, awaiting a target")
            elif was and not self._active:
                self.get_logger().info("standby")

        def on_mocap(self, m):
            q = m.pose.orientation
            self.state = (np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z]),
                          np.array([m.twist.linear.x, m.twist.linear.y, m.twist.linear.z]),
                          quat_to_R(q.x, q.y, q.z, q.w),
                          np.array([m.twist.angular.x, m.twist.angular.y, m.twist.angular.z]))

        def on_ref(self, m):
            """A target position + yaw. The message's VELOCITY IS IGNORED on purpose: this
            controller flies to points and its own ramp owns the feed-forward. A trajectory
            streamed here is chased as a moving point -- safe, but it lags. Trajectory
            tracking is the NMPC nodes' job."""
            t = np.array([m.position.x, m.position.y, m.position.z], dtype=float)
            if not np.all(np.isfinite(t)):
                return
            # ENVELOPE. The radius is measured from the origin OR from the vehicle, and the
            # second half is the important one: a SAFETY fallback must always be able to
            # operate where the vehicle actually IS. The origin-only version silently refused
            # every command once another controller had flown the aircraft past `radius_max`
            # -- so the one controller meant to recover it could not even hold station, and
            # the failure was a log line rather than anything visible (2026-09-07, HIL).
            zlo, zhi, rmax = self.gp("z_min"), self.gp("z_max"), self.gp("radius_max")
            local = self.gp("radius_local")
            near_home = math.hypot(t[0], t[1]) <= rmax
            near_self = (self.state is not None
                         and float(np.linalg.norm(t[:2] - self.state[0][:2])) <= local)
            if not (zlo - 1e-6 <= t[2] <= zhi) or not (near_home or near_self):
                self._rejected += 1
                if self._rejected <= 3 or self._rejected % 500 == 0:
                    self.get_logger().warn(
                        f"target ({t[0]:.1f},{t[1]:.1f},{t[2]:.1f}) outside the envelope "
                        f"(z {zlo:.0f}..{zhi:.0f} m; within {rmax:.0f} m of home or "
                        f"{local:.0f} m of the vehicle) -- ignored")
                return
            self.target = t
            self.yaw_target = (m.yaw * math.pi / 180.0
                               if m.yaw_unit == PositionControllerReference.DEGREES else m.yaw)

        # ---------------- control ----------------
        def loop(self):
            """An exception here would stop the actuator stream and PX4 would HOLD the last
            command while the vehicle fell (HIL bag vt1). Log once, then hold a level hover."""
            try:
                self._loop()
            except Exception:
                if not self._died:
                    self._died = True
                    self.get_logger().error(
                        "LOOP EXCEPTION (holding level hover):\n" + traceback.format_exc())
                self.pub([clamp(self.m * self.g / 4.0 / self.kf, 0.0, 1.0)] * 4)

        def _loop(self):
            self.counter += 1
            o = OffboardControlMode(); o.direct_actuator = True; o.timestamp = self.now_us()
            if self._active:
                self.ocm.publish(o)
            if self.state is None:
                if self._active:
                    self.pub([math.nan] * 4)
                return
            p, v, R, w = self.state
            if not self._active:
                self.ramp.p = None       # re-seed at the vehicle when handed control back
                return

            dt = 1.0 / self.rate
            if self.target is None:      # no operator input yet -> hold where we are
                self.target = p.copy()
                self.yaw_target = math.atan2(R[1, 0], R[0, 0])
            p_sp, v_ff, a_ff, yaw_sp = self.ramp.step(
                self.target, self.yaw_target, dt,
                self.gp("v_max"), self.gp("vz_up"), self.gp("vz_down"),
                self.gp("a_max"), self.gp("az_max"), math.radians(self.gp("yaw_rate_deg_s")))

            mot, self.ei, dg = geometric_command(
                p, v, R, w, p_sp, v_ff, a_ff, yaw_sp, self.ramp.yaw_rate,
                self.ei, dt, self.gains(), self.Ainv, integrate=self.armed)
            self.pub(mot)

            if self.counter % max(1, int(self.rate / 2)) == 0:
                d = math.degrees
                e_p = dg["e_p"]
                self.get_logger().info(
                    f"p=({p[0]:.2f},{p[1]:.2f},{p[2]:.2f}) sp=({p_sp[0]:.2f},{p_sp[1]:.2f},{p_sp[2]:.2f}) "
                    f"tgt=({self.target[0]:.1f},{self.target[1]:.1f},{self.target[2]:.1f}) "
                    f"|e|={np.linalg.norm(e_p):.2f} v={np.linalg.norm(v):.1f} "
                    f"rpy=({d(math.atan2(R[2,1],R[2,2])):+.0f},"
                    f"{-d(math.asin(clamp(R[2,0],-1,1))):+.0f},{d(math.atan2(R[1,0],R[0,0])):+.0f}) "
                    f"mot=[{mot[0]:.2f},{mot[1]:.2f},{mot[2]:.2f},{mot[3]:.2f}] arm={self.armed}")

            # auto_arm is read LIVE, not from the cached init value: otherwise a GS DISARM is
            # instantly undone -- the node re-arms on the next tick and PX4 logs an endless
            # "Landing detected" / "Armed by external command" pair (seen in the live GS test).
            if (self._active and bool(self.get_parameter("auto_arm").value)
                    and self.counter >= self.arm_after
                    and not (self.armed and self.offb)):
                if self.counter - self.last >= 100:
                    self.last = self.counter
                    if not self.offb:
                        self.vcmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                    elif not self.armed:
                        self.vcmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

        # ---------------- output ----------------
        def pub(self, mot):
            """Motors only. The tilt servos are pinned to 0 (rotors UP) and the control
            surfaces to neutral -- this node cannot command a tilt, by construction."""
            if not self._active:
                return
            mm = ActuatorMotors(); c = [math.nan] * 12
            for i in range(4):
                c[i] = mot[i]
            mm.control = c; mm.timestamp = mm.timestamp_sample = self.now_us()
            self.motp.publish(mm)
            ss = ActuatorServos(); sc = [math.nan] * 8
            for i in range(5):                        # tiltL, tiltR, aileron, elevator, rudder
                sc[i] = 0.0
            ss.control = sc; ss.timestamp = ss.timestamp_sample = self.now_us()
            self.srvp.publish(ss)

        def vcmd(self, cmd, p1=0.0, p2=0.0):
            m = VehicleCommand(); m.command = cmd; m.param1 = p1; m.param2 = p2
            m.target_system = m.target_component = m.source_system = m.source_component = 1
            m.from_external = True; m.timestamp = self.now_us(); self.cmdp.publish(m)

    rclpy.init()
    try:
        rclpy.spin(GeometricHover())
    except KeyboardInterrupt:
        pass


def main():
    run_ros()


if __name__ == "__main__":
    main()
