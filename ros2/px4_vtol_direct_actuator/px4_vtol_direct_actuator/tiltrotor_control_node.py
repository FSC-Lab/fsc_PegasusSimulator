#!/usr/bin/env python3
"""
FULL-TILTROTOR stabilization controller (direct-actuator, JOINT ALLOCATION).

Body held LEVEL in pitch; the 2 FRONT tilting rotors are active effectors. Instead
of per-axis PID (which couples/oscillates), this solves the 4 rotor thrusts + 2 tilt
angles JOINTLY from a desired body wrench [Fx, Fz, tau_x, tau_y, tau_z] via a
pseudo-inverse of the geometry allocation matrix:

  cascade:  position -> desired WORLD force
            -> forward force (heading frame) = body Fx  (produced by front TILT)
            -> lateral force                = ROLL set-point (thrust vector tilts)
            -> vertical force               = body Fz    (collective thrust)
            attitude(roll_ref, pitch=0, yaw_ref) -> torques tau_x/y/z
            allocation A^+ [Fx,Fz,tau] -> 6 force components -> (T_i, theta_L/R)

Streams ActuatorMotors + ActuatorServos via PX4 direct_actuator (PX4 passthrough).
Feedback = /<ns>/mocap; target = the shared PositionControllerReference topic
(fsc_ref + step_response.py drive it). All gains are LIVE ros2 params (read each
loop) -> tune with application/vtol/step_response.py, no restart.
"""
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy,
                       QoSHistoryPolicy, qos_profile_sensor_data)

from px4_msgs.msg import (OffboardControlMode, ActuatorMotors, ActuatorServos,
                          VehicleCommand, VehicleStatus)
from fsc_autopilot_ros2_msgs.msg import Mocap, PositionControllerReference
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import String


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def wrap_pi(a):
    return math.atan2(math.sin(a), math.cos(a))


def quat_to_rpy(x, y, z, w):
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(clamp(2 * (w * y - z * x), -1.0, 1.0))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


class TiltrotorControl(Node):
    def __init__(self):
        super().__init__("vtol_tiltrotor_control")
        P = self.declare_parameter
        P("px4_ns", "uav_0"); P("rate_hz", 100.0); P("auto_arm", True); P("arm_after", 100)
        # Feedback: /mocap (default) is smoother and empirically damps this marginal loop BETTER
        # than the raw 240Hz /state/* (which is noisier -> under-damped). use_state=True opt-in.
        P("use_state", False)    # /mocap: WORLD-frame velocity directly (needed for altitude+position
                                 # DAMPING to work). /state body-velocity + a transient-attitude rotation
                                 # gave a wrong vz -> no altitude damping -> overshoot. Attitude latency is
                                 # handled by the heavy rate-damping (kd) + the near-hover thrust cap.
        # vehicle / normalization
        P("mass", 3.4); P("gravity", 9.81); P("kf", 18.0)    # kf = N of thrust at motor cmd 1.0; mass from hover-throttle (~0.46)
        P("km", 0.12)                                         # rotor drag/thrust ratio (yaw reaction)
        # rotor geometry relative to CG (FLU, m) — front tilting, rear static
        P("xf", 0.285); P("yf", 0.285); P("zf", -0.025)
        P("xr", -0.286); P("yr", 0.285); P("zr", 0.024)
        # position -> desired world force  (accel = kp*e_pos + kd*e_vel)
        # Detuned for the latency-limited direct_actuator HIL: gentle position gains
        # + a hard tilt cap keep takeoff in the linear regime (the un-detuned 1.2/40deg
        # tilt-saturated on takeoff and tumbled). Station-keeping is looser but stable.
        # VERY gentle outer loop: the attitude inner loop is laggy (HIL round-trip),
        # so an aggressive position loop outruns it -> velocity builds -> the small
        # attitude clamp can't decelerate -> fly-away. Keep the outer loop well below
        # the inner-loop bandwidth (heavy velocity damping, small position push).
        P("kp_xy", 0.12); P("kd_xy", 1.6)  # VERY gentle horizontal hold + strong velocity damping. With the CG
                                            # fix the attitude is stable and it hovers ~13 s, but the position loop
                                            # then outruns the laggy inner loop -> fly-away. Lower kp_xy (don't chase
                                            # position hard) + higher kd_xy (kill drift velocity) keeps the outer loop
                                            # below the inner-loop bandwidth. Both cap at roll_max (6deg).
        P("vz_climb", 0.7)                       # m/s max climb rate (velocity-profiled altitude, see loop). Fast
                                                 # enough to escape the collider (yaw has NO authority on the ground)
        P("vz_rampup", 0.3)                      # s to ramp the climb-velocity TARGET 0->vz_climb (soft start): a
                                                 # step vz_des -> thrust spike -> pitch flip; too slow -> linger -> diverge
        P("az_stop", 1.2)                        # m/s^2 decel of the stopping profile -> eases vz to 0 at the target
                                                 # so altitude CANNOT overshoot (overshoot -> motor sat -> loss of ctrl)
        P("z_clear", 0.22)                       # altitude at which the box collider is clear of the ground
        P("punch_g", 1.30)                       # takeoff punch thrust [x hover] until z_clear: a marginal ~1.1*hover
                                                 # just BOUNCES on the collider; a firm push breaks static contact
        P("kp_z", 1.0);  P("kd_z", 3.0)          # kd_z = velocity-loop gain tracking vz_des; kp_z = small altitude
                                                 # position stiffness (see loop). The stopping profile does the shaping.
                                                 # overshoot that cuts thrust and starves attitude authority)
        # attitude -> torque [N*m].  ROSBAG DIAGNOSIS (2026-08-07): the original
        # 2.0/1.5 gains put the attitude loop into a motor-SATURATED diverging
        # oscillation over the HIL round-trip latency (roll swung +-90..170deg,
        # mot pinned 0.95, huge tau) -> tumble on takeoff. Sign is CORRECT (probe
        # confirmed rear-higher->+pitch). Fix = drop P hard (stay out of saturation),
        # keep/raise D (damping), cut I (windup), low-pass the rate D-term.
        P("kp_roll", 0.7);  P("kd_roll", 1.8);  P("ki_roll", 0.3)
        P("kp_pitch", 0.6); P("kd_pitch", 1.8); P("ki_pitch", 0.5)   # integral ADAPTS the pitch trim (no fixed FF now).
                                                                     # NOTE: HIL round-trip latency makes this loop over-
                                                                     # correct the ~-38deg liftoff pitch disturbance no matter
                                                                     # the P/D ratio (tried 0.35/2.2 .. 0.6/1.8: all peak
                                                                     # +43..+47 then cascade). The delayed rate feedback means
                                                                     # even heavy D acts on stale rate -> can't damp the fast
                                                                     # transient. This is the direct_actuator latency wall;
                                                                     # only the PX4 inner-loop (fsc/attitude-NMPC) hovers here.
        P("kp_yaw", 1.5);   P("kd_yaw", 1.5)   # heading hold + damping. Now stable: the reaction sign was inverted
                                                # (see _alloc); pre-fix the loop was positive-feedback and diverged.
        P("i_max", 4.0)          # integral clamp [N*m] — absorbs the steady CG-offset moment
        P("rate_lpf", 0.7)       # more-responsive D-term (240Hz /state rates are clean enough); heavy D needs low lag
        P("soft_start_s", 0.0)   # ramp motor output 0->1 over this many s after arming (0 = off;
                                 # scaling all 4 motors weakens attitude authority during the ramp)
        P("pitch_ff", 0.0)       # DISABLED: the -35deg liftoff pitch is a GROUND-CONTACT transient (probe saw it
                                 # only during the ground->air lift). A constant FF that counters it on the ground
                                 # then OVER-trims once airborne -> nose sticks at +44 (P-term cancels the FF). Let
                                 # the integral adapt instead (it unwinds when the disturbance clears).
                                 # (unused now) feedforward pitch trim per unit up-force [m]: cancels the
                                 # thrust-proportional nose-down moment from the fwd CG offset
        # limits
        P("roll_max_deg", 6.0); P("tilt_max_deg", 0.0)     # small roll/pitch for translation; NO tilt (translate by pitching)
        P("thr_min", 0.05); P("thr_max", 0.95)
        P("fz_max_g", 1.12)      # cap up-force NEAR hover: a pitch transient spiking the tilt-comp
                                 # (Fz/cos) to 1.5g rocketed the vehicle up -> altitude cut thrust ->
                                 # starved attitude authority -> stuck pitch -> fly-away. 1.12g climbs gently.
        # Controller arbitration: only the ACTIVE controller drives PX4 (both this
        # geometric node and the NMPC node publish the same direct_actuator topics,
        # so exactly one may publish at a time). The GS switches by publishing this
        # id on /<ns>/ground_station/active_controller. Default active = the safe one.
        P("controller_id", "geometric"); P("active_at_start", True)

        ns = self.get_parameter("px4_ns").get_parameter_value().string_value
        pre = f"/{ns}" if ns else ""
        self.rate = self.gp("rate_hz")
        self.auto_arm = self.get_parameter("auto_arm").value
        self.arm_after = int(self.get_parameter("arm_after").value)
        self.ei_roll = 0.0; self.ei_pitch = 0.0   # attitude integral state
        self.arm_count = None                       # counter value when arming latched (soft-start / FF ramp)
        self._z0 = 0.09                             # ground altitude captured at arm (takeoff ramp origin)
        self._cleared = False                       # True once the collider has left the ground (latched)
        self.pf = self.qf = self.rf = 0.0           # low-pass filtered body rates

        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        self.ocm_pub = self.create_publisher(OffboardControlMode, f"{pre}/fmu/in/offboard_control_mode", qos)
        self.mot_pub = self.create_publisher(ActuatorMotors, f"{pre}/fmu/in/actuator_motors", qos)
        self.srv_pub = self.create_publisher(ActuatorServos, f"{pre}/fmu/in/actuator_servos", qos)
        self.cmd_pub = self.create_publisher(VehicleCommand, f"{pre}/fmu/in/vehicle_command", qos)
        self.create_subscription(VehicleStatus, f"{pre}/fmu/out/vehicle_status_v1", self.on_status, qos)
        if self.get_parameter("use_state").value:
            # Direct-from-Isaac state (ENU pose, body-frame rates, inertial linear vel) —
            # ~240Hz, no optitrack-emulator hop -> lower latency for the attitude inner loop.
            self.create_subscription(PoseStamped, f"{pre}/state/pose", self.on_pose, qos_profile_sensor_data)
            self.create_subscription(TwistStamped, f"{pre}/state/twist", self.on_twist, qos_profile_sensor_data)
        else:
            self.create_subscription(Mocap, f"{pre}/mocap", self.on_mocap, qos_profile_sensor_data)
        self.create_subscription(PositionControllerReference,
                                 f"{pre}/fsc_autopilot_ros2/position_controller/reference", self.on_ref, 10)
        self.controller_id = self.get_parameter("controller_id").value
        self._active = bool(self.get_parameter("active_at_start").value)
        self.create_subscription(String, f"{pre}/ground_station/active_controller",
                                 self.on_active, 10)

        self._have_pose = False
        self.state = None
        self.ref = {"x": 0.0, "y": 0.0, "z": 1.5, "yaw": 0.0}
        self.armed = self.offb = False
        self.counter = self.last_cmd = 0
        self.create_timer(1.0 / self.rate, self.loop)
        self.get_logger().info(f"tiltrotor_control (allocation) up ns='{ns}' rate={self.rate:.0f}Hz")

    def gp(self, n): return float(self.get_parameter(n).value)
    def now_us(self): return int(self.get_clock().now().nanoseconds / 1000)

    def on_status(self, m):
        self.armed = (m.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        self.offb = (m.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD)

    def on_ref(self, m):
        self.ref["x"], self.ref["y"], self.ref["z"] = m.position.x, m.position.y, m.position.z
        self.ref["yaw"] = m.yaw * (math.pi / 180.0) if m.yaw_unit == PositionControllerReference.DEGREES else m.yaw

    def _blank_state(self):
        return {"x": 0.0, "y": 0.0, "z": 0.09, "vx": 0.0, "vy": 0.0, "vz": 0.0,
                "roll": 0.0, "pitch": 0.0, "yaw": 0.0, "p": 0.0, "q": 0.0, "r": 0.0}

    def on_mocap(self, m):
        q = m.pose.orientation
        roll, pitch, yaw = quat_to_rpy(q.x, q.y, q.z, q.w)
        self.state = {"x": m.pose.position.x, "y": m.pose.position.y, "z": m.pose.position.z,
                      "vx": m.twist.linear.x, "vy": m.twist.linear.y, "vz": m.twist.linear.z,
                      "roll": roll, "pitch": pitch, "yaw": yaw,
                      "p": m.twist.angular.x, "q": m.twist.angular.y, "r": m.twist.angular.z}
        self._have_pose = True

    # --- direct Isaac state feedback (lower latency than /mocap) ---
    def on_pose(self, m):
        if self.state is None:
            self.state = self._blank_state()
        q = m.pose.orientation
        self.state["roll"], self.state["pitch"], self.state["yaw"] = quat_to_rpy(q.x, q.y, q.z, q.w)
        self.state["x"], self.state["y"], self.state["z"] = m.pose.position.x, m.pose.position.y, m.pose.position.z
        self._have_pose = True

    def on_twist(self, m):        # body-frame linear + angular velocity (twist_inertial isn't published;
                                  # at/near hover body ~ world, and z-damping only needs vz)
        if self.state is None:
            self.state = self._blank_state()
        self.state["p"], self.state["q"], self.state["r"] = m.twist.angular.x, m.twist.angular.y, m.twist.angular.z
        # /state/twist.linear is BODY frame; rotate to WORLD (R=Rz(yaw)Ry(pitch)Rx(roll))
        # so the position loop's world position error and velocity share a frame (mixing
        # body velocity with world position error made the vehicle fly away).
        vbx, vby, vbz = m.twist.linear.x, m.twist.linear.y, m.twist.linear.z
        cr, sr = math.cos(self.state["roll"]), math.sin(self.state["roll"])
        cp, sp = math.cos(self.state["pitch"]), math.sin(self.state["pitch"])
        cy, sy = math.cos(self.state["yaw"]), math.sin(self.state["yaw"])
        self.state["vx"] = (cy*cp)*vbx + (cy*sp*sr - sy*cr)*vby + (cy*sp*cr + sy*sr)*vbz
        self.state["vy"] = (sy*cp)*vbx + (sy*sp*sr + cy*cr)*vby + (sy*sp*cr - cy*sr)*vbz
        self.state["vz"] = (-sp)*vbx + (cp*sr)*vby + (cp*cr)*vbz

    def _alloc(self):
        """Build A (5x6) and its pinv. Cols=[FLx,FLz,FRx,FRz,RLz,RRz] force components,
        rows=[Fx,Fz,tau_x,tau_y,tau_z].  tau = r x F ; front rotors also have Fx (tilt)."""
        xf, yf, zf = self.gp("xf"), self.gp("yf"), self.gp("zf")
        xr, yr, zr = self.gp("xr"), self.gp("yr"), self.gp("zr")
        km = self.gp("km")
        # spin dirs (yaw reaction): FL+1, FR-1, RL-1, RR+1 ; reaction tau_z = -km*spin*T (~Fz).
        # This MATCHES the Isaac vtol.py sim (_apply_propulsion: k=-spin_dir*moment_ratio*T
        # about body z, spin_dirs FL+1/FR-1/RL-1/RR+1). Sign VERIFIED correct against the
        # sim; the HIL yaw divergence is a GROUND-CONTACT problem (no yaw authority while
        # the collider is on the ground), fixed by a decisive liftoff, not by flipping this.
        A = np.array([
            [1.0,   0.0,  1.0,   0.0,   0.0,   0.0],                 # Fx
            [0.0,   1.0,  0.0,   1.0,   1.0,   1.0],                 # Fz
            [0.0,   yf,   0.0,  -yf,    yr,   -yr],                  # tau_x = y*Fz
            [zf,   -xf,   zf,   -xf,   -xr,   -xr],                  # tau_y = z*Fx - x*Fz
            [-yf,  -km,   yf,   +km,   +km,   -km],                  # tau_z = -y*Fx  + reaction
        ])
        return A, np.linalg.pinv(A)

    def loop(self):
        self.counter += 1
        self.pub_ocm()
        if self.state is None or not self._have_pose:
            self.pub_motors([math.nan] * 4); self.pub_servos([math.nan] * 5); return
        s, r = self.state, self.ref
        m, g, kf = self.gp("mass"), self.gp("gravity"), self.gp("kf")
        roll_max = math.radians(self.gp("roll_max_deg")); tilt_max = math.radians(self.gp("tilt_max_deg"))

        # latch arm time + ground altitude, so the takeoff ramp starts from the ground
        if self.armed:
            if self.arm_count is None:
                self.arm_count = self.counter
                self._z0 = s["z"]
                self._cleared = False
        else:
            self.arm_count = None

        # position -> desired WORLD force. GATE it in only near hover altitude: during the
        # fragile climb the position loop's roll/pitch commands (to chase drift) pile onto the
        # liftoff disturbance and excite the attitude. Hold level while climbing, engage
        # horizontal hold as z crosses ~1.0 -> 1.3 m.
        pos_gate = clamp((s["z"] - 1.0) / 0.3, 0.0, 1.0)
        Fwx = pos_gate * m * (self.gp("kp_xy") * (r["x"] - s["x"]) + self.gp("kd_xy") * (-s["vx"]))
        Fwy = pos_gate * m * (self.gp("kp_xy") * (r["y"] - s["y"]) + self.gp("kd_xy") * (-s["vy"]))

        # velocity-profiled altitude: command a climb-velocity setpoint vz_des that eases to
        # 0 at the target via a sqrt STOPPING profile, tracked by a velocity loop (kd_z). This
        # (a) escapes the ground collider fast (yaw has no authority there), (b) CANNOT
        # overshoot the target (the profile decelerates -> no motor saturation -> attitude
        # stays controllable), and (c) ACCEPTS the takeoff-punch velocity instead of fighting
        # it (heavy position damping repeatedly killed the punch -> the vehicle fell back).
        dz = r["z"] - s["z"]
        rampup = clamp((self.counter - self.arm_count) / (self.rate * self.gp("vz_rampup")), 0.0, 1.0) \
            if self.arm_count is not None else 1.0
        vz_cap = self.gp("vz_climb") * rampup          # soft-start the climb-velocity target (no thrust spike)
        vz_des = math.copysign(min(vz_cap, math.sqrt(2.0 * self.gp("az_stop") * abs(dz))), dz)
        z_ref = r["z"]   # (debug log only)
        Fwz = m * (g + self.gp("kd_z") * (vz_des - s["vz"]) + self.gp("kp_z") * clamp(dz, -0.15, 0.15))

        # takeoff punch: until the collider clears the ground (z_clear, latched), floor the
        # vertical thrust at punch_g*hover so it breaks static box-collider contact rather
        # than bouncing on it at a marginal ~1.1*hover. Handoff to ramp+PID once airborne.
        if self.arm_count is not None and s["z"] > self.gp("z_clear"):
            self._cleared = True
        elif s["z"] < self._z0 + 0.08:          # fell back to the ground -> re-arm the punch
            self._cleared = False
        if self.arm_count is not None and not self._cleared:
            Fwz = max(Fwz, self.gp("punch_g") * m * g)
        Fv = max(Fwz, 0.15 * m * g)

        # heading frame: forward -> body Fx (tilt), lateral -> roll set-point
        cy, sy = math.cos(s["yaw"]), math.sin(s["yaw"])
        F_fwd = Fwx * cy + Fwy * sy
        F_lat = -Fwx * sy + Fwy * cy
        roll_ref = clamp(math.atan2(-F_lat, Fv), -roll_max, roll_max)
        # Translate FORWARD by PITCHING (quad-style), NOT by tilting the front rotors.
        # ROSBAG DIAG: tilt-for-forward makes a nose-down moment that couples with the
        # position loop into a runaway pitch-over -> flip on takeoff. Keep rotors vertical
        # (Fx=0 -> tilt=0) and command a pitch set-point for forward force instead.
        pitch_ref = clamp(math.atan2(F_fwd, Fv), -roll_max, roll_max)
        Fx = 0.0
        # tilt-compensate thrust but CAP it: an uncapped 1/cos(pitch) blows Fz up as the
        # vehicle tips, saturating all 4 motors and killing attitude authority (runaway).
        # tilt-compensate for the COMMANDED tilt only (roll_ref/pitch_ref), NOT the actual
        # attitude: compensating a disturbance pitch spikes thrust -> altitude overshoot ->
        # thrust cut -> starved attitude -> cascade. This decouples altitude from the wobble.
        Fz = min(Fv / max(math.cos(roll_ref) * math.cos(pitch_ref), 0.7),
                 self.gp("fz_max_g") * m * g)

        # attitude -> torques.  Standard PID on roll+pitch: the P/D stabilize; the I-term
        # trims out the steady CG-offset moment (balanced thrust sits fwd/aft of the true
        # CG -> a constant nose-down moment that P alone can't null without a droop).
        # PROBE-VERIFIED: the allocation signs are STANDARD (rear-higher/+tau_y pitches
        # nose-up; the small-signal "inversion" I chased earlier was just the fixed
        # nose-down disturbance winning). So standard PID, and the I-term builds the
        # steady rear-bias that cancels the disturbance.
        # low-pass the body rates for the D-term (raw Isaac rates are noisy -> D amplifies it)
        a = self.gp("rate_lpf")
        self.pf += a * (s["p"] - self.pf); self.qf += a * (s["q"] - self.qf); self.rf += a * (s["r"] - self.rf)

        er = roll_ref - s["roll"]; ep = pitch_ref - s["pitch"]
        imax = self.gp("i_max")
        if self.armed:
            self.ei_roll = clamp(self.ei_roll + self.gp("ki_roll") * er / self.rate, -imax, imax)
            self.ei_pitch = clamp(self.ei_pitch + self.gp("ki_pitch") * ep / self.rate, -imax, imax)
        else:
            self.ei_roll = self.ei_pitch = 0.0
        tx = self.gp("kp_roll") * er + self.gp("kd_roll") * (-self.pf) + self.ei_roll
        ty = self.gp("kp_pitch") * ep + self.gp("kd_pitch") * (-self.qf) + self.ei_pitch
        # thrust-proportional trim, GATED ON ALTITUDE: the CG nose-down moment only acts once
        # airborne (on the ground the contact cancels it), so ramp the FF in as it lifts off
        # (z 0.12->0.32). Slamming it on at arm over-corrects -> pitch overshoot -> oscillation.
        ff_gate = clamp((s["z"] - 0.10) / 0.05, 0.0, 1.0)   # ramp trim in FAST at liftoff (the moment hits immediately)
        ty += self.gp("pitch_ff") * (m * g) * ff_gate   # CONSTANT hover-based trim (not transient Fz,
                                                         # which spikes at a decisive liftoff -> over-trim)
        tz = self.gp("kp_yaw") * wrap_pi(r["yaw"] - s["yaw"]) + self.gp("kd_yaw") * (-self.rf)

        # joint allocation: wrench -> 6 force components
        _, Ainv = self._alloc()
        FLx, FLz, FRx, FRz, RLz, RRz = Ainv @ np.array([Fx, Fz, tx, ty, tz])

        # physical recovery: up-force >= 0, forward bounded by tilt limit
        FLz = max(FLz, 0.0); FRz = max(FRz, 0.0); RLz = max(RLz, 0.0); RRz = max(RRz, 0.0)
        thL = clamp(math.atan2(FLx, max(FLz, 1e-3)), 0.0, tilt_max)
        thR = clamp(math.atan2(FRx, max(FRz, 1e-3)), 0.0, tilt_max)
        T_FL = math.hypot(FLx, FLz); T_FR = math.hypot(FRx, FRz)

        # soft-start: ramp the motor output 0->1 over soft_start_s after arming so the vehicle
        # spins up + leaves the ground gently (a hard step excites the marginal pitch loop).
        # (arm_count / _z0 are latched at the top of the loop for the takeoff ramp.)
        ss = self.gp("soft_start_s")
        ramp = clamp((self.counter - self.arm_count) / (self.rate * ss), 0.0, 1.0) if (self.arm_count and ss > 0) else 1.0

        # attitude-PRIORITY desaturation: if the raw motor set would clamp (on an altitude
        # excursion the collective drops, a motor hits 0, and the attitude DIFFERENTIAL is lost
        # -> the vehicle can't correct pitch -> tumble), shift ALL motors by a common offset so
        # they fit in [0, thr_max]. This preserves the differential (attitude) at the expense of
        # the collective (altitude) -> breaks the overshoot -> saturation -> tumble cascade.
        thr_max = self.gp("thr_max")
        raw = [ramp * T_FL / kf, ramp * T_FR / kf, ramp * RLz / kf, ramp * RRz / kf]
        lo = min(raw)
        if lo < 0.0:
            raw = [v - lo for v in raw]            # shift up so the lowest motor is exactly 0
        hi = max(raw)
        if hi > thr_max:
            raw = [v - (hi - thr_max) for v in raw]  # shift down so the highest is thr_max
        mFL, mFR, mRL, mRR = (clamp(v, 0.0, thr_max) for v in raw)
        tiltL = clamp(thL / (math.pi / 2), 0.0, 1.0)
        tiltR = clamp(thR / (math.pi / 2), 0.0, 1.0)

        self.pub_motors([mFL, mFR, mRL, mRR])
        self.pub_servos([tiltL, tiltR, 0.0, 0.0, 0.0])

        if self.counter % max(1, int(self.rate / 5)) == 0:   # 5 Hz debug
            d = math.degrees
            self.get_logger().info(
                f"z {s['z']:.2f}->{z_ref:.2f} | rpy=({d(s['roll']):+.0f},{d(s['pitch']):+.0f},{d(s['yaw']):+.0f})"
                f" roll*={d(roll_ref):+.0f} yaw*={d(r['yaw']):+.0f} | e=({r['x']-s['x']:+.1f},{r['y']-s['y']:+.1f})"
                f" | mot=[{mFL:.2f},{mFR:.2f},{mRL:.2f},{mRR:.2f}] tilt=[{tiltL:.2f},{tiltR:.2f}]"
                f" | wrench Fx={Fx:+.1f} Fz={Fz:.1f} t=({tx:+.2f},{ty:+.2f},{tz:+.2f}) arm={self.armed}")

        if self._active and self.auto_arm and self.counter >= self.arm_after and not (self.armed and self.offb):
            if (self.counter - self.last_cmd) >= int(self.rate):
                self.last_cmd = self.counter
                if not self.offb:
                    self.vcmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                elif not self.armed:
                    self.vcmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def on_active(self, m):
        was = self._active
        self._active = (m.data == self.controller_id)
        if self._active != was:
            self.get_logger().info(
                f"controller '{self.controller_id}' -> {'ACTIVE' if self._active else 'standby'}")

    def pub_ocm(self):
        if not self._active:
            return
        m = OffboardControlMode(); m.direct_actuator = True; m.timestamp = self.now_us(); self.ocm_pub.publish(m)

    def pub_motors(self, vals):
        if not self._active:
            return
        m = ActuatorMotors(); c = [math.nan] * 12
        for i, v in enumerate(vals):
            c[i] = v
        m.control = c; m.timestamp = m.timestamp_sample = self.now_us(); self.mot_pub.publish(m)

    def pub_servos(self, vals):
        if not self._active:
            return
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
    node = TiltrotorControl()
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
