#!/usr/bin/env python3
"""CASCADE SO(3) transition + fly-to-point mission controller for the FSC quad-tiltrotor
(direct_actuator, mocap-based).

WHY a cascade: the SO(3) transition NMPC (nmpc_so3_transition_node.py) is CORRECT but its IPOPT
solve is 0.24-0.9 s, so run in-loop the control ran at ~1-4 Hz -- fine for a quasi-static hover but
far too slow to stabilise the FAST attitude of the tilt transition (-> pitch tumble). So we DECOUPLE:

  * OUTER (async, background thread, ~3-5 Hz): the SO(3) NMPC. Its job is the SLOW plan -- the
    optimal front-rotor TILT feed-forward beta_ff(t) for the hover->cruise lift hand-off. NEVER blocks
    the fast loop (CasADi releases the GIL during the solve -- verified the inner loop holds ~99 Hz).
  * INNER (fast, every mocap update): a geometric tiltrotor POSITION controller (Lee SE(3)) that
    tracks a MISSION reference {p_ref, v_ref, a_ref, yaw}: forward force is provided by front-rotor
    TILT (sized to the forward-force demand, then removed from F_des so the body stays ~level ->
    fly forward on TILT not by pitching), lateral by roll, altitude by collective; deceleration to
    a precise stop is the SE(3) backward LEAN (tilt can't reverse). Conventions reused verbatim from
    the HIL-validated traj_track_node.py (precise landing proven).

MISSION: on engage it flies a trapezoidal-speed straight line from the engage point to
(target_x, target_y) at cruise_alt/cruise_speed, then descends to LAND at the target (speed
controller = the velocity feed-forward + PID; precise stop = the decel profile + SE(3) lean).
Gated on /<ns>/ground_station/active_controller == controller_id (default 'so3_cascade').
"""
import math
import threading
import numpy as np

import nmpc_so3_transition_node as so3
from nmpc_so3_transition_node import (SO3TransitionNMPC, TransitionRef, predict_forward,
                                      M, G, NX, NU, MAX_THRUST, RPOS, SPIN, CQ, _q2R,
                                      RHO, S_WING, CL0, CLA, CD0, KIND, STALL)


def _clip(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def _vee(S):
    return np.array([S[2, 1], S[0, 2], S[1, 0]])


def aero_world(R, v):
    """Modelled wing force in the WORLD frame, mirroring vtol_aerodynamics.py.

    Used as a FEED-FORWARD: the rotors provide F_des MINUS this, so the altitude loop
    anticipates the wing lift instead of lagging it as speed builds.

    Rewritten 2026-09-06: it previously used the sim's old FLU alpha with FRD coefficients,
    so the modelled lift pointed the wrong way. Now the FLU->FRD conversion happens at the
    boundary and the coefficients are the published ones, exactly as in the plant.
    """
    vb = R.T @ v
    u, vv, ww = float(vb[0]), -float(vb[1]), -float(vb[2])          # FLU -> FRD
    V = math.sqrt(u * u + vv * vv + ww * ww)
    if V < 1.0:
        return np.zeros(3)
    alpha = math.atan2(ww, abs(u) + 1e-9)
    beta = math.asin(max(min(vv / V, 0.99), -0.99))
    qbar = 0.5 * so3.RHO * V * V
    sig = 0.5 * (1.0 + math.tanh(0.5 * so3.BLEND_M * (abs(alpha) - so3.STALL)))
    sig_b = 0.5 * (1.0 + math.tanh(0.5 * so3.BLEND_M * (abs(beta) - so3.BETA_BREAK)))
    sa, ca_ = math.sin(alpha), math.cos(alpha)
    CL_lin = so3.CL0 + so3.CLA * alpha
    CL = (1 - sig) * CL_lin + sig * 2.0 * math.copysign(1.0, alpha) * sa * sa * ca_
    CD = (so3.CD0 + (1 - sig) * so3.KIND * CL_lin * CL_lin + sig * 2.0 * sa * sa
          + (1 - sig_b) * so3.CDB2 * beta * beta)
    CY = ((1 - sig_b) * so3.CYB * beta
          - sig_b * so3.CY_MAX * math.copysign(1.0, beta) * math.sin(beta) ** 2)
    L = qbar * so3.S_WING * CL; D = qbar * so3.S_WING * CD; Y = qbar * so3.S_WING * CY
    vdir = np.array([u, vv, ww]) / V
    F_frd = -D * vdir + L * np.array([sa, 0.0, -ca_]) + np.array([0.0, Y, 0.0])
    return R @ np.array([F_frd[0], -F_frd[1], -F_frd[2]])           # FRD -> FLU -> world


class Mission:
    """Trapezoidal-speed fly-to-point + vertical landing reference (FSC-scaled), in the spirit of the
    AVTOL TransitionRefInterp: a smooth speed profile to a target, then a controlled descent to land.
    Returns (p_ref, v_ref, a_ref, yaw, landing) at time-since-engage `te`."""
    def __init__(self, p0, target_xy, cruise_alt, Vc, a_max, settle, descent_rate):
        self.p0 = np.array(p0, float)
        self.zc = float(cruise_alt); self.Vc = max(float(Vc), 1e-3)
        self.a = max(float(a_max), 1e-3); self.settle = float(settle); self.vd = max(float(descent_rate), 1e-3)
        d = np.array([target_xy[0] - p0[0], target_xy[1] - p0[1]], float)
        self.D = float(np.linalg.norm(d))
        self.yaw = math.atan2(d[1], d[0]) if self.D > 1e-3 else 0.0
        self.hdg = np.array([math.cos(self.yaw), math.sin(self.yaw), 0.0])
        self.tgt = np.array([target_xy[0], target_xy[1]], float)
        # trapezoidal profile (accel a -> cruise Vc -> decel a), triangular if too short
        self.ta = self.Vc / self.a
        da = 0.5 * self.a * self.ta ** 2
        if 2.0 * da >= self.D:
            self.ta = math.sqrt(self.D / self.a) if self.D > 0 else 0.0
            self.Vpk = self.a * self.ta; self.tc = 0.0
        else:
            self.Vpk = self.Vc; self.tc = (self.D - 2.0 * da) / self.Vc
        self.t_arrive = self.settle + 2.0 * self.ta + self.tc
        self.t_land = self.t_arrive + max(self.zc - 0.1, 0.0) / self.vd
        self._t_land0 = None                             # set when the landing actually starts
        self._te_eff = 0.0; self._te_last = None         # mission clock (freezes while lagging)

    def _profile(self, t):
        """horizontal distance s, speed v, accel a along the heading at time-since-engage t."""
        t = t - self.settle
        if t <= 0.0:
            return 0.0, 0.0, 0.0
        ta, tc, a, Vpk, D = self.ta, self.tc, self.a, self.Vpk, self.D
        if t < ta:
            return 0.5 * a * t * t, a * t, a
        t2 = t - ta
        if t2 < tc:
            return 0.5 * a * ta * ta + Vpk * t2, Vpk, 0.0
        t3 = t2 - tc
        if t3 < ta:
            s0 = 0.5 * a * ta * ta + Vpk * tc
            return s0 + Vpk * t3 - 0.5 * a * t3 * t3, max(Vpk - a * t3, 0.0), -a
        return D, 0.0, 0.0

    def get(self, te, s_veh=None):
        # MISSION CLOCK FREEZE: the trapezoid is nominally time-based, but its clock only
        # advances while the reference is not already leading the vehicle by >10 m. Otherwise a
        # transient (engage wobble) let the open-loop clock run the profile OUT -- vff collapsed
        # to 0, the fixed-wing tilt bootstrap died, and the traverse crawled at ~1.6 m/s on
        # position error alone (HIL bag fwE6).
        if self._te_last is None:
            self._te_last = te
        dt = max(te - self._te_last, 0.0); self._te_last = te
        s_prog, _, _ = self._profile(self._te_eff)
        if s_veh is None or s_prog <= s_veh + 10.0:
            self._te_eff += dt
        te = self._te_eff
        s, v, a = self._profile(te)
        # LEAD CLAMP: never let the trapezoid run more than 10 m ahead of the vehicle (an
        # open-loop reference runaway drove huge position-error demands -> attitude transients;
        # HIL bags fwE/fwE2). The landing is also gated on the vehicle actually being NEAR the
        # target, so a lagging traverse just extends instead of snapping to a descent far ahead.
        if s_veh is not None and s > s_veh + 10.0:
            s = s_veh + 10.0; a = 0.0
        ref = self.p0 + self.hdg * s
        # RAMP the altitude reference from the engage altitude (a z step commanded a big vertical
        # error that leaked into the pitch trim -> backward slide; HIL bag fwE3)
        dz = self.zc - self.p0[2]
        ref[2] = self.p0[2] + max(min(dz, 1.2 * te), -1.2 * te)
        vff = self.hdg * v; aff = self.hdg * a; landing = False
        near = (s_veh is None) or (s_veh >= self.D - 5.0)
        if te >= self.t_arrive and near:                 # arrived -> descend to land at the target
            if self._t_land0 is None:
                self._t_land0 = te
            td = te - self._t_land0
            ref[0], ref[1] = self.tgt[0], self.tgt[1]
            ref[2] = max(self.zc - self.vd * td, 0.0)
            vff = np.array([0.0, 0.0, -self.vd if ref[2] > 0.06 else 0.0]); aff = np.zeros(3)
            landing = True
        return ref, vff, aff, self.yaw, landing


def run_ros():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy,
                           QoSHistoryPolicy, qos_profile_sensor_data)
    from px4_msgs.msg import (OffboardControlMode, ActuatorMotors, ActuatorServos,
                              VehicleCommand, VehicleStatus)
    from std_msgs.msg import String
    from fsc_autopilot_ros2_msgs.msg import Mocap

    class CascadeNode(Node):
        def __init__(self):
            super().__init__("nmpc_so3_cascade_control")
            P = self.declare_parameter
            P("px4_ns", "uav_0"); P("controller_id", "so3_cascade"); P("active_at_start", False)
            P("auto_arm", True); P("arm_after", 150); P("pub_rate_hz", 60.0)
            # MISSION
            P("target_x", 60.0); P("target_y", 0.0); P("cruise_alt", 15.0)
            P("cruise_speed", 12.0); P("accel", 2.0); P("settle_s", 3.0); P("descent_rate", 1.0)
            # position outer gains (traj_track-validated)
            P("kp_xy", 3.0); P("kv_xy", 4.0); P("kp_z", 8.0); P("kv_z", 5.0)
            P("tilt_max_deg", 45.0); P("tilt_v_gate", 0.4); P("max_lean_deg", 30.0)
            # attitude inner gains
            P("kR", 9.0); P("kR_yaw", 2.0); P("kW", 1.4); P("kW_yaw", 0.5)
            # SO(3) NMPC tilt feed-forward blend (0 = pure position loop; >0 adds the NMPC's optimal tilt)
            P("beta_ff_gain", 0.0)
            P("wing_ff_gain", 0.8)             # wing-lift feed-forward (kills the cruise-altitude bulge)
            P("Vcruise", 16.0); P("beta_cruise_deg", 70.0); P("transition_tf", 8.0)
            # ---- FULL FIXED-WING cruise mode (AVTOL-scaled transition profile, fsc_transition_ref).
            # Above fw_tilt_on the inner loop switches from the body-level tilt-for-forward law to a
            # PROFILE-FOLLOWING airplane mode: tilt/pitch/elevator-trim FF come from the FSC-scaled
            # AVTOL trim schedule indexed by the MEASURED forward speed (path-following: the schedule
            # advances only as real airspeed builds), thrust split front(fwd)/rear(residual vertical),
            # damped-moment aero surfaces. Below fw_tilt_off it drops back to the validated
            # multicopter path (takeoff / landing / hover / slow traverse unchanged).
            P("fixed_wing", False)   # default MULTICOPTER (the working fly-to-point+land); FW branch experimental
            P("profile_v_cruise", 15.0); P("profile_tf", 20.0)
            P("fw_front_cap", 0.90)          # front-motor headroom cap (bag so3r)
            P("fw_sas_pitch", 0.30); P("fw_sas_roll", 0.20); P("fw_sas_yaw", 0.15)  # rate-SAS on surfaces
            # ENTER EARLY (profile tilt ~8 deg = ~1.2 m/s): at that point both the multicopter path
            # and the profile command near-level + near-zero tilt, so the mode change is seamless and
            # the WHOLE acceleration happens inside the profile-following law. (A late 25-deg entry
            # snapped +5 nose-up against the MC path's ~30-deg forward lean -> violent transient ->
            # dive -> speed-inflated tilt=90 mid-dive -> tumble. HIL bag fwE.)
            P("fw_tilt_on_deg", 8.0); P("fw_tilt_off_deg", 5.0)     # hysteresis on the profile tilt
            P("fw_level_gate_deg", 20.0)   # only ENTER fixed-wing from quasi-level attitude...
            P("fw_vz_gate", 1.5)           # ...and small climb rate (no mid-dive engagement)
            P("fw_tilt_rate_deg_s", 40.0)  # slew on the commanded tilt
            P("fw_pitch_rate_deg_s", 15.0) # slew on the commanded trim pitch
            P("kp_lat", 0.8); P("kv_lat", 1.6); P("max_roll_deg", 18.0)
            P("k_elev", -0.15); P("k_ail", -0.15); P("k_rud", -0.10); P("surf_v_gate", 3.0)
            P("k_fw_alt", 0.08)                # pitch-trim reduction per m above ref (anti climb-away)
            P("rotor_floor", 0.5)              # pair-thrust floor at full wing-borne [N]
            P("rotor_min", 8.0)                # pair-thrust floor in the rotor-borne regime [N]
            P("wing_ff_gain_fw", 1.0)          # FULL wing-lift subtraction in airplane mode (rotors ~0 vertical)

            ns = self.get_parameter("px4_ns").value
            pre = f"/{ns}" if ns else ""
            self.cid = self.get_parameter("controller_id").value
            self._active = bool(self.get_parameter("active_at_start").value)
            self.auto_arm = bool(self.get_parameter("auto_arm").value)
            self.arm_after = int(self.get_parameter("arm_after").value)
            self.kf = MAX_THRUST

            # allocation A: [T_FL,T_FR,T_RL,T_RR] -> [f(collective), Mx, My, Mz] (tilt=0; front boosted later)
            A = np.zeros((4, 4))
            for i in range(4):
                A[0, i] = 1.0; A[1, i] = RPOS[i, 1]; A[2, i] = -RPOS[i, 0]; A[3, i] = -SPIN[i] * CQ
            self.Ainv = np.linalg.inv(A)

            self.get_logger().info("building SO(3) NMPC (outer tilt FF)...")
            self.nmpc = SO3TransitionNMPC()
            self.get_logger().info("NMPC built.")

            # AVTOL-scaled transition profile for the fixed-wing cruise mode
            self.prof = None; self._fw_mode = False; self._fw_tilt = 0.0; self._fw_theta = 0.0
            self._last_tilt = 0.0
            if bool(self.get_parameter("fixed_wing").value):
                try:
                    from fsc_transition_ref import FSCTransitionProfile
                    self.prof = FSCTransitionProfile(V_cruise=self.gp("profile_v_cruise"),
                                                     tf=self.gp("profile_tf"))
                    self.get_logger().info("AVTOL-scaled profile loaded: Vc=%.1f v_wing_borne=%.1f"
                                           % (self.prof.V_cruise, self.prof.v_wing_borne))
                except Exception as e:
                    self.get_logger().warn(f"fixed-wing profile unavailable ({e}) -> multicopter only")

            self.lock = threading.Lock()
            self.state = None; self.armed = self.offb = False; self.counter = self.last = 0
            self.mission = None; self.ref_nmpc = None; self._t_engage = None
            self.beta_ff = 0.0; self.ubuf = []; self._outer_alive = True
            # streamed GS reference (PositionControllerReference) -- when present it OVERRIDES the internal
            # Mission, so the ground-station Trajectory/Fly-to-point tab drives this controller directly.
            self.gs_ref = None; self.gs_ref_t = 0.0

            qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                             history=QoSHistoryPolicy.KEEP_LAST, depth=1)
            self.ocm = self.create_publisher(OffboardControlMode, f"{pre}/fmu/in/offboard_control_mode", qos)
            self.motp = self.create_publisher(ActuatorMotors, f"{pre}/fmu/in/actuator_motors", qos)
            self.srvp = self.create_publisher(ActuatorServos, f"{pre}/fmu/in/actuator_servos", qos)
            self.cmdp = self.create_publisher(VehicleCommand, f"{pre}/fmu/in/vehicle_command", qos)
            self.create_subscription(VehicleStatus, f"{pre}/fmu/out/vehicle_status_v1", self.on_status, qos)
            self.create_subscription(Mocap, f"{pre}/mocap", self.on_mocap, qos_profile_sensor_data)
            self.create_subscription(String, f"{pre}/ground_station/active_controller", self.on_active, 10)
            from fsc_autopilot_ros2_msgs.msg import PositionControllerReference
            self._RefMsg = PositionControllerReference
            self.create_subscription(PositionControllerReference,
                                     f"{pre}/fsc_autopilot_ros2/position_controller/reference", self.on_ref, 10)
            self.create_timer(1.0 / float(self.get_parameter("pub_rate_hz").value), self.heartbeat)
            self.oth = threading.Thread(target=self.outer_loop, daemon=True); self.oth.start()
            self.get_logger().info(f"so3_cascade up: id={self.cid} (fly-to-point + land; NMPC tilt-FF async)")

        def gp(self, n):
            return float(self.get_parameter(n).value)

        def now_us(self):
            return int(self.get_clock().now().nanoseconds / 1000)

        def on_status(self, m):
            self.armed = (m.arming_state == VehicleStatus.ARMING_STATE_ARMED)
            self.offb = (m.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD)

        def on_active(self, m):
            was = self._active
            self._active = (m.data == self.cid)
            if self._active and not was:
                self._engage()
            elif was and not self._active:
                with self.lock:
                    self.mission = None; self.ref_nmpc = None; self._t_engage = None; self.gs_ref = None
                    self._fw_mode = False

        def on_ref(self, m):
            """Ground-station streamed setpoint (pos + velocity FF + yaw). Overrides the internal Mission."""
            yaw = (m.yaw * math.pi / 180.0) if m.yaw_unit == self._RefMsg.DEGREES else m.yaw
            with self.lock:
                self.gs_ref = (np.array([m.position.x, m.position.y, m.position.z]),
                               np.array([m.velocity.x, m.velocity.y, m.velocity.z]), float(yaw))
                self.gs_ref_t = self.get_clock().now().nanoseconds * 1e-9

        def _engage(self):
            with self.lock:
                if self.state is None:
                    return
                p, R, v, w = self.state
                yaw0 = math.atan2(R[1, 0], R[0, 0])
                self.mission = Mission(p, (self.gp("target_x"), self.gp("target_y")),
                                       self.gp("cruise_alt"), self.gp("cruise_speed"),
                                       self.gp("accel"), self.gp("settle_s"), self.gp("descent_rate"))
                self.ref_nmpc = TransitionRef(p[0], p[1], p[2], self.mission.yaw,
                                              Vcruise=self.gp("Vcruise"),
                                              beta_cruise_deg=self.gp("beta_cruise_deg"),
                                              tf=self.gp("transition_tf"))
                self.beta_ff = 0.0; self.ubuf = []; self._fw_mode = False
                self._t_engage = self.get_clock().now().nanoseconds * 1e-9
            self.get_logger().info("CASCADE mission engaged at (%.1f,%.1f,%.1f) -> target (%.1f,%.1f) yaw=%.0f D=%.1f"
                                   % (p[0], p[1], p[2], self.gp("target_x"), self.gp("target_y"),
                                      math.degrees(self.mission.yaw), self.mission.D))

        # ---------- FAST inner loop (geometric tiltrotor position control) ----------
        def on_mocap(self, m):
            q = m.pose.orientation
            R = _q2R(q.x, q.y, q.z, q.w)
            p = np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z])
            v = np.array([m.twist.linear.x, m.twist.linear.y, m.twist.linear.z])
            w = np.array([m.twist.angular.x, m.twist.angular.y, m.twist.angular.z])
            now = self.get_clock().now().nanoseconds * 1e-9
            with self.lock:
                self.state = (p, R, v, w)
                mission = self.mission; t_eng = self._t_engage; beta_ff = self.beta_ff
                gs_ref = self.gs_ref; gs_fresh = (now - self.gs_ref_t) < 0.5
            if not self._active:
                return
            # reference source: a FRESH ground-station stream overrides the internal Mission
            mission_drives = False
            if gs_ref is not None and gs_fresh:
                ref, vff, yaw = gs_ref; aff = np.zeros(3)
                landing = (ref[2] < 0.35)                # GS descends to land_z ~0.12
            elif mission is not None and t_eng is not None:
                s_veh = float((p - mission.p0) @ mission.hdg)
                ref, vff, aff, yaw, landing = mission.get(now - t_eng, s_veh=s_veh)
                mission_drives = (now - t_eng) > mission.settle
            else:
                return
            e3 = np.array([0.0, 0.0, 1.0])

            # ---- position outer loop: desired specific force (world) ----
            kp = np.array([self.gp("kp_xy"), self.gp("kp_xy"), self.gp("kp_z")])
            kv = np.array([self.gp("kv_xy"), self.gp("kv_xy"), self.gp("kv_z")])
            F_des = M * (G * e3 + aff) - kp * (p - ref) - kv * (v - vff)

            # ---- FULL FIXED-WING branch (AVTOL-scaled profile FF + geometric feedback). Decided by
            # hysteresis on the PROFILE tilt at the measured forward speed, so the mode advances only
            # as real airspeed builds and walks back on decel; landing always forces multicopter. ----
            fw = False; sch = None
            if self.prof is not None and not landing:
                fdirp = np.array([math.cos(yaw), math.sin(yaw), 0.0])
                vfwd_p = float(v[0] * fdirp[0] + v[1] * fdirp[1])
                # profile key: measured forward speed, but let the reference velocity-FF LEAD it by
                # up to 4 m/s -- at a standstill the measured-V key gives tilt=0 -> no forward
                # force -> no speed (chicken-and-egg, HIL bag fwE3); the FF lead bootstraps the tilt.
                # FLOOR at 3 m/s whenever the mission commands motion: in FW mode tilt is the ONLY
                # forward actuator (no lean), and after a backward wobble the lead alone could not
                # recover (v_key pinned 0 -> tilt 0 -> 25 s backward drift, HIL bag fwE6).
                vff_fwd_p = float(vff @ fdirp)
                # KEY THE TILT ON THE COMMANDED (reference) SPEED, not the measured speed (offline
                # probe fw_probe.py + HIL bag cascFW1). The old measured-keyed clamp (v_key <=
                # measured+4) was a CHICKEN-AND-EGG: at engage speed=0 -> tilt=0 -> no forward
                # thrust -> no speed -> tilt stays 0. The vehicle just climbed and never flew
                # forward. The mission velocity-FF ramps 0->Vc along an ACHIEVABLE accel, so keying
                # the tilt on it makes the vehicle tilt + accelerate to follow the profile (the
                # profile IS the force-balanced trim for that speed). Bounded to measured+6 so the
                # tilt can't run so far ahead of the actual speed that the wing can't carry (the
                # SO(3)-node sink failure) -- a gentle lead, not a hard lock to the reference.
                v_key = _clip(max(vff_fwd_p, vfwd_p), 0.0, vfwd_p + 6.0)
                sch = self.prof.by_speed(v_key)
                # ★ KNOWN LIMITATION (offline 6-DOF probe fw_hil_probe.py, 2026-08-11): this FW
                # (airplane-mode) branch has a SPEED-RUNAWAY. The tilt is keyed on `v_key` which
                # includes the MEASURED forward speed -> POSITIVE FEEDBACK: at cruise the tilt(V)
                # produces MORE forward force than the profile assumes (the AVTOL-scaled drag is
                # lower), the vehicle overspeeds, that raises v_key -> more tilt -> the vehicle
                # accelerates past the target and CANNOT decelerate (the tilt stays locked forward
                # by its own speed even when F_des points backward). Probe: flies straight + level
                # to 40 m then blows past the target to 260 m at 29 m/s and dives. The MULTICOPTER
                # branch (fixed_wing=False, below ~line 495) keys tilt on the FORCE DEMAND
                # (atan2(F_fwd, half_up), which goes NEGATIVE to brake) and flies the SAME mission
                # to a PRECISE landing (validated: 40/60/100 m targets, land within 0.1 m, altitude
                # +/-0.1 m, level, lat 0.06-0.12 s). => the default fixed_wing=False is the reliable
                # fly-to-point+land path; enabling this FW branch needs a force-demand / speed-
                # governed tilt (or the genuine wing-borne law) before it will track+stop. See
                # [[cascade-probe-transpose-bug]].
                if mission_drives:
                    # a MISSION traverse is OWNED by the fixed-wing profile end-to-end: it starts
                    # from the settle hover (level, still) and the profile handles tilt(V)
                    # continuously from V=0, so there is no MC<->FW boundary to flap across.
                    # (Threshold-switching mid-acceleration snapped the tilt 45<->0 at ~10 Hz and
                    # dove the vehicle into a stalled porpoise -- HIL bag fwE2 servo histogram.)
                    fw = True
                else:
                    thr = self.gp("fw_tilt_off_deg") if self._fw_mode else self.gp("fw_tilt_on_deg")
                    fw = math.degrees(sch["tilt"]) > thr
                    if fw and not self._fw_mode:
                        # ENTRY GATE: only from quasi-level, non-diving flight
                        level = R[2, 2] > math.cos(math.radians(self.gp("fw_level_gate_deg")))
                        fw = level and abs(float(v[2])) < self.gp("fw_vz_gate")
                if fw and not self._fw_mode:
                    # slew states start from the LAST APPLIED tilt (no command snap on entry)
                    self._fw_tilt = self._last_tilt; self._fw_theta = 0.0
            self._fw_mode = fw
            if fw:
                # FULL wing-lift subtraction: in airplane mode the rotors provide only what the wing
                # does not (the profile trim says ~0 rotor vertical force at cruise).
                F_des = F_des - self.gp("wing_ff_gain_fw") * aero_world(R, v)
                F_des[2] = max(F_des[2], 0.05 * M * G)
                # SLEW the commanded tilt + trim pitch (mocap ~100 Hz): the profile is the target,
                # the applied commands ramp -- no snaps even if the measured speed jumps.
                dtl = math.radians(self.gp("fw_tilt_rate_deg_s")) / 100.0
                dth = math.radians(self.gp("fw_pitch_rate_deg_s")) / 100.0
                self._fw_tilt = _clip(float(sch["tilt"]), self._fw_tilt - dtl, self._fw_tilt + dtl)
                # FW BRAKE: a decel demand (negative forward force) cannot come from the tilt
                # (Tf floors out) -- pitch nose-up instead, rotating the thrust backward and
                # raising the AoA (wing drag), like a flare. Same slew as the trim pitch.
                F_fwd_w = float(F_des @ fdirp)
                brake = _clip(-F_fwd_w / (M * G), 0.0, math.radians(12.0)) if F_fwd_w < 0.0 else 0.0
                # alt-corr is ONE-SIDED: only reduce the nose-up trim when HIGH (anti climb-away).
                # Being LOW is the rotor vertical loop's job -- adding nose-up pitch when low
                # rotated the thrust backward and slid the vehicle off its takeoff point (fwE3).
                th_tgt = _clip(sch["theta_cmd"] + brake - self.gp("k_fw_alt") * max(p[2] - ref[2], 0.0),
                               -math.radians(2.0), math.radians(14.0))
                self._fw_theta = _clip(th_tgt, self._fw_theta - dth, self._fw_theta + dth)
                tilt = self._fw_tilt; theta_cmd = self._fw_theta; self._last_tilt = tilt
                ldir = np.array([-math.sin(yaw), math.cos(yaw), 0.0])
                e_lat = float((p - ref) @ ldir); v_lat = float((v - vff) @ ldir)
                roll_des = _clip((self.gp("kp_lat") * e_lat + self.gp("kv_lat") * v_lat) / G,
                                 -math.radians(self.gp("max_roll_deg")),
                                 math.radians(self.gp("max_roll_deg")))
                cy, sy = math.cos(yaw), math.sin(yaw)
                cp, sp = math.cos(-theta_cmd), math.sin(-theta_cmd)   # nose-up + (nmpc_cruise conv.)
                cr, srr = math.cos(roll_des), math.sin(roll_des)
                Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
                Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
                Rx = np.array([[1, 0, 0], [0, cr, -srr], [0, srr, cr]])
                R_des = Rz @ Ry @ Rx
                # thrust split in the desired body frame: front pair covers the FORWARD demand via
                # the tilt, rear pair the residual vertical (the wing carries the rest). The rear
                # floor is wing-share-scheduled (authority while rotor-borne -> unload when wing-borne).
                Fb = R_des.T @ F_des
                ws = self.prof.wing_share(max(vfwd_p, 0.0))
                tr_floor = self.gp("rotor_min") * (1.0 - ws) + self.gp("rotor_floor") * ws
                # THRUST SPLIT, blended by tilt (bags fwE4/fwE5: any front-loaded split at low tilt
                # out-muscles the pitch loop -> instant pitch-over). Low tilt: EQUAL pair thrusts
                # sized to the vertical demand -- the same symmetric split the HIL-validated
                # nmpc_cruise settles on -- and forward force is whatever the SCHEDULED tilt gives
                # (the AVTOL profile was solved for exactly that tilting-thrust trim). High tilt:
                # forward-priority (front pair covers the forward demand; validated in cruise).
                cos_t = math.cos(tilt); sin_t = math.sin(tilt)
                # PITCH-BALANCED split (offline probe fw_hil_probe.py): the old split loaded the
                # FRONT pair to make forward thrust (Tf_fwd = Fb[0]/sin_t), but front-heavy thrust
                # IS a nose-up moment -> forward-demand and pitch were COUPLED and fought each other
                # -> oscillation/divergence at ANY gain. Instead, size the pair thrusts so the
                # thrust is PITCH-NEUTRAL about the CG: Tf*cos(tilt)*x_f = Tr*x_r, and
                # Tf*cos(tilt) + Tr = Fb[2] (share the vertical demand). The FORWARD thrust is then
                # Tf*sin(tilt), provided purely by the TILT (the profile sets tilt by commanded
                # speed) -- decoupled from pitch. The attitude loop only handles disturbances.
                xf, xr = 0.282, 0.289
                Tf_pair = _clip(Fb[2] / max(cos_t * (1.0 + xf / xr), 0.12),
                                self.gp("rotor_floor"), 1.84 * self.kf)
                Tr_pair = _clip(Tf_pair * cos_t * (xf / xr), tr_floor, 1.84 * self.kf)
                eR = 0.5 * _vee(R_des.T @ R - R.T @ R_des); eW = w
                kR = np.array([self.gp("kR"), self.gp("kR"), self.gp("kR_yaw")])
                kW = np.array([self.gp("kW"), self.gp("kW"), self.gp("kW_yaw")])
                Mc = -kR * eR - kW * eW
                pitch_act = -math.asin(max(-1.0, min(1.0, R[2, 0])))
                self._dbg = (math.degrees(pitch_act), math.degrees(theta_cmd),
                             float(eR[0]), float(eR[1]), float(eR[2]), float(Mc[1]),
                             float(Tf_pair), float(Tr_pair))
                # moment saturation (same caps as the HIL-validated nmpc_cruise loop)
                Mc[0] = _clip(Mc[0], -8.0, 8.0); Mc[1] = _clip(Mc[1], -8.0, 8.0)
                Mc[2] = _clip(Mc[2], -4.0, 4.0)
                # allocation (nmpc_cruise-validated pattern) + DAMPED-moment aero surfaces.
                # PITCH-SIGN (offline probe fw_pitch_iso.py, ODE ground-truth): front-HEAVY thrust
                # = nose-DOWN moment (nose-up +10deg -> the validated multicopter Ainv commands
                # front 0.55/rear 0.39 -> ODE wdot_pitch=-5.58 = nose-down = corrects). So a nose-up
                # error (eR1>0 -> Mc1<0 via Lee) must produce FRONT-HEAVY, i.e. dpitch<0. With the
                # front motors carrying `-dpitch`, that needs dpitch = +Mc[1]/... (Mc1<0 -> dpitch<0
                # -> front heavier). The earlier `-Mc[1]` made it front-light -> POSITIVE FEEDBACK.
                droll = Mc[0] / (4.0 * 0.284); dpitch = Mc[1] / (4.0 * 0.30); dyawm = Mc[2] / (4.0 * CQ)
                T4 = np.array([Tf_pair / 2 + droll - dpitch - dyawm * SPIN[0],
                               Tf_pair / 2 - droll - dpitch - dyawm * SPIN[1],
                               Tr_pair / 2 + droll + dpitch - dyawm * SPIN[2],
                               Tr_pair / 2 - droll + dpitch - dyawm * SPIN[3]])
                T4 = np.clip(T4, 0.0, self.kf)
                mot = [_clip(T4[i] / self.kf, 0.0, 1.0) for i in range(4)]
                # FRONT-MOTOR HEADROOM CAP (SO(3) night lesson, bag so3r): saturating the front pair
                # in the transition lost pitch authority -> porpoise -> stall. Keep headroom.
                fcap = self.gp("fw_front_cap")
                mot[0] = min(mot[0], fcap); mot[1] = min(mot[1], fcap)
                tservo = _clip(tilt / (math.pi / 2), 0.0, 1.0)
                Vspd = float(np.linalg.norm(v))
                sg = _clip((Vspd - self.gp("surf_v_gate")) / 4.0, 0.0, 1.0)
                elevator = _clip(sch["de_cmd"] + self.gp("k_elev") * Mc[1] * sg, -1.0, 1.0)
                aileron = _clip(self.gp("k_ail") * Mc[0] * sg, -1.0, 1.0)
                rudder = _clip(self.gp("k_rud") * Mc[2] * sg, -1.0, 1.0)
                # RATE-SAS on the surfaces (reactive body-rate damping, faster than the outer loop) --
                # damps the cruise phugoid/porpoise. Signs: +elev nose-down opposes +q, etc.
                ssas = _clip((Vspd - 4.0) / 4.0, 0.0, 1.0)
                elevator = _clip(elevator + ssas * self.gp("fw_sas_pitch") * float(w[1]), -1.0, 1.0)
                aileron = _clip(aileron - ssas * self.gp("fw_sas_roll") * float(w[0]), -1.0, 1.0)
                rudder = _clip(rudder - ssas * self.gp("fw_sas_yaw") * float(w[2]), -1.0, 1.0)
                self.publish(mot, [tservo, tservo, aileron, elevator, rudder])
                self.counter += 1
                if self.counter % 30 == 0:
                    self.get_logger().info(
                        "FW p=(%.1f,%.1f,%.1f)->(%.1f,%.1f,%.1f) V=%.1f tilt=%.0f th=%.1f ws=%.0f%% "
                        "Tf=%.1f Tr=%.1f elev=%.2f roll=%.0f"
                        % (p[0], p[1], p[2], ref[0], ref[1], ref[2], vfwd_p, math.degrees(tilt),
                           math.degrees(theta_cmd), 100 * ws, Tf_pair, Tr_pair, elevator,
                           math.degrees(roll_des)))
                return

            # wing-lift FEED-FORWARD: the rotors provide F_des MINUS the modeled aero, so the altitude
            # loop anticipates the wing lift that jumps in as speed builds (no cruise-altitude bulge).
            F_des = F_des - self.gp("wing_ff_gain") * aero_world(R, v)
            F_des[2] = max(F_des[2], 0.2 * M * G)

            # ---- tilt-for-forward: front rotors vector thrust forward (body stays level) ----
            tilt_max = math.radians(self.gp("tilt_max_deg"))
            fdir = np.array([math.cos(yaw), math.sin(yaw), 0.0])
            F_fwd = float(F_des @ fdir)
            vfwd = float(v[0] * fdir[0] + v[1] * fdir[1]); vff_fwd = float(vff @ fdir)
            tilt = 0.0
            if (not landing) and tilt_max > 0.0 and F_fwd > 0.0 and max(vfwd, vff_fwd) > self.gp("tilt_v_gate"):
                half_up = 0.5 * max(F_des[2], 1.0)
                tilt = _clip(math.atan2(F_fwd, half_up), 0.0, tilt_max)
                F_des = F_des - min(half_up * math.tan(tilt), F_fwd) * fdir
            tilt = _clip(tilt + self.gp("beta_ff_gain") * beta_ff, 0.0, tilt_max)  # + NMPC optimal-tilt FF
            self._last_tilt = tilt

            # ---- cap the lean so a big horizontal demand can't flip it ----
            max_lean = math.radians(self.gp("max_lean_deg"))
            fz = F_des[2]; fh = math.hypot(F_des[0], F_des[1])
            if fh > fz * math.tan(max_lean):
                s = fz * math.tan(max_lean) / (fh + 1e-9); F_des[0] *= s; F_des[1] *= s

            # ---- desired attitude (Lee SE(3)) + moment ----
            b3 = F_des / (np.linalg.norm(F_des) + 1e-9)
            b1c = np.array([math.cos(yaw), math.sin(yaw), 0.0])
            b2 = np.cross(b3, b1c); b2 /= (np.linalg.norm(b2) + 1e-9)
            R_des = np.column_stack((np.cross(b2, b3), b2, b3))
            f = float(F_des @ (R @ e3))
            eR = 0.5 * _vee(R_des.T @ R - R.T @ R_des); eW = w
            kR = np.array([self.gp("kR"), self.gp("kR"), self.gp("kR_yaw")])
            kW = np.array([self.gp("kW"), self.gp("kW"), self.gp("kW_yaw")])
            Mcmd = -kR * eR - kW * eW

            # ---- allocation -> 4 rotor thrusts; boost the tilted front pair by 1/cos ----
            T = self.Ainv @ np.array([f, Mcmd[0], Mcmd[1], Mcmd[2]])
            if tilt > 1e-3:
                boost = 1.0 / max(math.cos(tilt), 0.30); T[0] *= boost; T[1] *= boost
            T = np.clip(T, 0.0, self.kf)
            mot = [_clip(T[i] / self.kf, 0.0, 1.0) for i in range(4)]
            tservo = _clip(tilt / (math.pi / 2), 0.0, 1.0)
            # cut thrust once landed (near ground at the target)
            if landing and p[2] < 0.20:
                mot = [0.0, 0.0, 0.0, 0.0]
            self.publish(mot, [tservo, tservo, 0.0, 0.0, 0.0])
            self.counter += 1
            if self.counter % 30 == 0:
                self.get_logger().info(
                    "p=(%.1f,%.1f,%.1f)->(%.1f,%.1f,%.1f) V=%.1f tilt=%.0f f=%.0f eR=(%+.2f,%+.2f) %s"
                    % (p[0], p[1], p[2], ref[0], ref[1], ref[2], float(np.linalg.norm(v)),
                       math.degrees(tilt), f, eR[0], eR[1], "LAND" if landing else ""))

        # ---------- SLOW outer NMPC (background thread) -> tilt feed-forward ----------
        def outer_loop(self):
            import time
            while self._outer_alive:
                with self.lock:
                    st = self.state; ref = self.ref_nmpc; t_eng = self._t_engage; ubuf = list(self.ubuf)
                if (not self._active) or st is None or ref is None or t_eng is None or self.gp("beta_ff_gain") <= 0:
                    time.sleep(0.1); continue
                p, R, v, w = st
                x = np.concatenate([p, R.reshape(-1), [self.beta_ff], v, w])
                s_dist = float(np.dot(p - ref.p0, ref.fwd))
                t_now = self.get_clock().now().nanoseconds * 1e-9
                hold = (t_now - t_eng) < self.gp("settle_s")
                try:
                    u, ok = self.nmpc.solve(x, ref, s_dist, hold=hold)
                    with self.lock:
                        self.beta_ff = float(_clip(u[4], 0.0, math.radians(self.gp("beta_cruise_deg"))))
                        self.ubuf.append(u.copy()); self.ubuf = self.ubuf[-5:]
                except Exception:
                    time.sleep(0.1)

        def publish(self, mot, srv):
            if not self._active:
                return
            mm = ActuatorMotors(); c = [math.nan] * 12
            for i in range(4):
                c[i] = mot[i]
            mm.control = c; mm.timestamp = mm.timestamp_sample = self.now_us(); self.motp.publish(mm)
            ss = ActuatorServos(); sc = [math.nan] * 8
            for i in range(5):
                sc[i] = srv[i]
            ss.control = sc; ss.timestamp = ss.timestamp_sample = self.now_us(); self.srvp.publish(ss)

        def heartbeat(self):
            o = OffboardControlMode(); o.direct_actuator = True; o.timestamp = self.now_us()
            if self._active:
                self.ocm.publish(o)
                if self.state is None:
                    self.publish([math.nan] * 4, [math.nan] * 5)
            self.last += 1
            if self._active and self.auto_arm and self.last >= self.arm_after and not (self.armed and self.offb):
                if self.last % 40 == 0:
                    if not self.offb:
                        self.vcmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                    elif not self.armed:
                        self.vcmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

        def vcmd(self, cmd, p1=0.0, p2=0.0):
            m = VehicleCommand(); m.command = cmd; m.param1 = p1; m.param2 = p2
            m.target_system = m.target_component = m.source_system = m.source_component = 1
            m.from_external = True; m.timestamp = self.now_us(); self.cmdp.publish(m)

    rclpy.init()
    node = CascadeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._outer_alive = False


def main(args=None):
    """console_scripts entry point.

    Delegates to the SAME dispatch as `python3 nmpc_so3_cascade_node.py`, so `ros2 run` and a
    direct invocation cannot drift apart -- including the offline gate flags,
    which are how this controller is validated before it is ever flown.
    """
    run_ros()



if __name__ == "__main__":
    main()
