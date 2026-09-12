#!/usr/bin/env python3
"""AERO-AWARE tilt transition controller for the FSC tilt-quadplane (direct_actuator, mocap-based).

WHY: the plain geometric controller (traj_track) does NOT model the wing, so during a fast
traverse the wing lift over/under-powers it -> climb-away or descend-crash (validated). This
controller adds the missing piece: it SUBTRACTS the modeled wing lift/drag (aero feed-forward,
same coefficients as vtol_aerodynamics.py) from the thrust demand, so as the wing takes over
the rotors back off and TILT forward -> a stable hover->cruise->hover transition that HOLDS
altitude (offline-validated: 0.42 m cruise altitude hold, tilt -> 80 deg, wing-borne cruise).

It is the real-time feedback form of the offline-validated transition NMPC (nmpc_transition_offline.py):
same aero model, same force balance (T*sin b + L = W ; T*cos b + Tr = ...). Reuses traj_track's
proven SE(3) attitude + allocation; the only new pieces are (1) aero feed-forward on F_des and
(2) a cruise-capable tilt allocation (front-rotor thrust sized to forward+vertical via sqrt, so it
still provides forward thrust when the wing carries all the weight and the vertical demand -> 0).

Output: ActuatorMotors + ActuatorServos + OffboardControlMode(direct_actuator), mocap-based (no EKF).
Gated on /<ns>/ground_station/active_controller == controller_id (GS switch).
"""
import math
import os
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy,
                       QoSHistoryPolicy, qos_profile_sensor_data)
from px4_msgs.msg import (OffboardControlMode, ActuatorMotors, ActuatorServos,
                          VehicleCommand, VehicleStatus)
from std_msgs.msg import String
from fsc_autopilot_ros2_msgs.msg import Mocap, PositionControllerReference

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






def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def quat_to_R(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w) + 1e-12
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)]])


def vee(M):
    return np.array([M[2, 1], M[0, 2], M[1, 0]])


class AeroTransition(Node):
    def __init__(self):
        super().__init__("aero_transition_control")
        P = self.declare_parameter
        P("px4_ns", "uav_0")
        P("controller_id", "transition")
        P("active_at_start", False)
        P("auto_arm", True)
        P("arm_after", 150)
        P("rate_hz", 200.0)
        P("mass", 3.5); P("gravity", 9.81); P("max_thrust", 18.0)
        # position / velocity gains (world)
        P("kp_xy", 2.4); P("kv_xy", 2.6)
        P("kp_z", 6.0);  P("kv_z", 7.5)          # more altitude-rate damping (catches the high-speed climb)
        P("ki_z", 0.8); P("i_max", 4.0); P("i_band", 1.2)
        # attitude
        P("kR", 9.0); P("kW", 1.8); P("kR_yaw", 2.0); P("kW_yaw", 0.5)
        # PITCH TRIM: the tilted front rotors make a steady nose-down moment the SE(3) P-term can't
        # null -> the body sits ~20deg nose-DOWN in cruise -> wing at -20deg AoA -> DOWNFORCE (-17 N)
        # -> inflates the thrust demand + no wing lift. A pitch integral trims it to the commanded
        # (small nose-UP) attitude so the wing LIFTS instead.
        # PITCH TRIM / AoA management (OFF by default). It DID confirm the diagnosis (trimming the
        # nose-down flips the wing from -17 N downforce to +30 N lift), but the wing lift then becomes
        # a 2nd uncontrolled vertical actuator and -- with the rotor thrust unable to drop below its
        # boosted floor at high tilt -- the vehicle climbs away. Proper use needs a COORDINATED
        # tilt+thrust+pitch controller (the NMPC's job). Left here, disabled, for future work.
        P("ki_pitch", 0.0); P("i_pitch_max", 0.7)     # pitch integral OFF by default (leveling the body
                                                       # re-triggers the wing-lift climb-away; see below)
        P("aoa_cmd_deg", 0.0)            # commanded nose-up AoA (0 = off)
        P("aoa_v_lo", 3.0); P("aoa_v_hi", 9.0)
        P("aoa_kvz", 0.0)               # altitude-rate AoA feedback (0 = off)
        P("aoa_max_deg", 8.0)
        # TILT-MOMENT pitch feed-forward: the tilted front rotors' FORWARD thrust acts below the CG
        # -> a steady nose-DOWN moment the allocation (vertical-thrust-only pitch) never sees. Feed a
        # nose-up pitch moment = tilt_pitch_ff * F_fwd to hold the body LEVEL (so the wing stops making
        # downforce), WITHOUT a lift-generating pitch command. Tune the sign/mag live via /tmp/diag.
        P("tilt_pitch_ff", 0.04)        # N*m nose-up per N of forward thrust. MILD default: it DOES
                                        # level the body + cut the downforce (validated: 0.12 -> AoA
                                        # -24->-10deg), but leveling makes the wing LIFT and the vehicle
                                        # then CLIMBS AWAY (rotors can't offset the wing lift at high
                                        # tilt). So keep it mild for a stable ~6 m/s cruise; a coordinated
                                        # altitude/wing controller (NMPC) is needed to exploit it fully.
        # REAR-ROTOR CUTOFF in cruise: ramp the rear (fixed, vertical) rotors OFF as airspeed builds,
        # shifting their vertical share to the (tilted, boosted) front rotors + the wing.
        P("rear_off_v_lo", 5.0); P("rear_off_v_hi", 12.0)   # airspeed band over which the rear cut to 0
        # CONTROL SURFACES: the rotor pitch/roll/yaw authority collapses at high tilt (front rotors
        # point forward, rear cut) -> use the AERO surfaces (effective ~ airspeed) so the ELEVATOR
        # actually holds the pitch/AoA. Faded in above surf_v_gate. Signs tunable live.
        P("use_surfaces", 1.0)
        # elevator SIGN found live (negative): it DOES control pitch/AoA (validated: AoA -35 -> -7deg
        # as the elevator swung), but an aggressive gain porpoises (the elevator+rotor+wing+tilt all
        # couple). Gentle default -> a real, stable pitch contribution without driving the oscillation.
        P("k_elev", -0.5)               # elevator gain on the pitch attitude error
        P("k_ail", 0.0)                 # aileron gain (roll) -- enable + tune sign after the elevator
        P("k_rud", 0.0)                 # rudder gain (yaw)
        P("surf_v_gate", 3.0)           # airspeed [m/s] at which the surfaces start fading in
        # ==== DISTINCT CRUISE MODE ====  Once forward flight is committed, SNAP to a high fixed tilt
        # (fast), CUT the rear rotors, and fly like a fixed-wing: the TILTED FRONT PROPS make the
        # forward thrust, the WING (AoA via elevator/pitch) makes the lift + holds altitude. This is
        # the clean transition instead of the reactive hover-blend that never fully cruises.
        P("use_cruise", 0.0)   # 0 = GRADUAL reactive tilt (STABLE, but nose-down/wing-downforce, NOT wing-borne)
        #   NB: use_cruise=1 (snap toward cruise_tilt_deg + hold +AoA + cut rear) CRASHES at the onset in HIL
        #   -- the body pitches nose-down and falls before the wing can carry. See the airplane-mode note.
        P("cruise_enter_v", 4.0); P("cruise_exit_v", 3.0)    # commanded fwd speed to enter/exit cruise
        P("cruise_tilt_deg", 80.0)                            # committed cruise tilt (props forward)
        P("cruise_tilt_rate", 12.0)                           # SLOW slew: pace the tilt to the accel so the
        #                                                       wing engages before the vertical thrust is gone
        #                                                       (a fast snap to 80deg dropped it out of the sky)
        P("cruise_aoa_deg", 5.0)                              # baseline positive cruise AoA (wing trim -> lift)
        P("cruise_alt_kp", 2.0); P("cruise_alt_kd", 2.5)      # altitude -> AoA (deg/m, deg/(m/s))
        P("cruise_aoa_max", 12.0)                             # clamp the cruise AoA [deg]
        P("y_arm", 0.285)                                     # front rotor lateral arm [m] (roll diff)
        P("tilt_max_deg", 80.0)          # full transition tilt cap (wing carries weight at cruise)
        # tilt COORDINATION (prevents the fall/lunge): the tilt is capped by AIRSPEED (only tilt
        # far once the wing can carry the weight), slew-rate limited, and gated on YAW ALIGNMENT
        # (yaw to face the travel direction first, else the tilt pushes the wrong world way).
        P("tilt_sat_margin", 0.9)        # cap tilt so boosted front thrust <= margin*max_thrust
                                         # (headroom-based anti-fall; opens as the wing engages)
        P("tilt_rate_deg_s", 40.0)       # servo slew: FAST enough to punch through the transition
                                         # corridor (6-14 m/s, wing not yet load-bearing) before the
                                         # rotors thrust-saturate; the sat cap keeps it from falling.
        P("tilt_lpf", 0.12)              # mild low-pass on the tilt target (kills chatter, low lag)
        P("yaw_align_min", 0.75)         # cos of the max nose-vs-travel angle to allow tilt (~41 deg)
        P("yaw_rate_deg_s", 50.0)        # slew the attitude yaw cmd (smooth turns, no 180 singularity)
        P("max_roll_deg", 25.0)          # lateral realized by rolling (forward is the tilt's job)
        P("ff_lpf", 0.4)
        # AERO FEED-FORWARD -- coeffs MUST match the (scaled) sim vtol_aerodynamics.py or the FF mis-sizes
        # the rotor unload -> sag/climb. Synced 2026-08-10 to the scaled small quadplane wing (was the
        # old AVTOL wing S=0.55/CLmax=1.5, which over-estimated lift ~1.6x -> would pull the rotors off
        # too early and drop the vehicle).
        P("aero_ff", True)
        P("rho", 1.225); P("wing_area", 0.35)
        P("cl0", 0.15); P("cl_alpha", 4.8); P("cd0", 0.04); P("k_induced", 0.04)
        P("cl_max", 1.2); P("stall_deg", 22.0); P("aero_v_gate", 2.0); P("aero_min_v", 0.5)
        P("cg_x", 0.195); P("cQ", 0.06)
        P("dbg", 0.0)                    # 1 = log the F_des[z] term breakdown + wing lift/AoA

        ns = self.get_parameter("px4_ns").value
        pre = f"/{ns}" if ns else ""
        self.cid = self.get_parameter("controller_id").value
        self._active = bool(self.get_parameter("active_at_start").value)
        self.auto_arm = bool(self.get_parameter("auto_arm").value)
        self.arm_after = int(self.get_parameter("arm_after").value)
        self.rate = float(self.gp("rate_hz"))
        self.m = self.gp("mass"); self.g = self.gp("gravity"); self.kf = self.gp("max_thrust")

        cg = self.gp("cg_x")
        self.rp = np.array([[0.459 - cg, 0.285], [0.459 - cg, -0.284],
                            [-0.112 - cg, 0.284], [-0.112 - cg, -0.284]])
        self.spin = np.array([+1.0, -1.0, -1.0, +1.0])
        cQ = self.gp("cQ")
        A = np.zeros((4, 4)); A[0, :] = 1.0
        A[1, :] = self.rp[:, 1]; A[2, :] = -self.rp[:, 0]; A[3, :] = -self.spin * cQ
        self.Ainv = np.linalg.pinv(A)

        self.state = None; self.ref = None; self.yaw_ref = 0.0
        self.vff = np.zeros(3); self.aff = np.zeros(3)
        self.ei = 0.0; self._pref_prev = None; self._vff_prev = np.zeros(3)
        self.tilt_prev = 0.0; self.tilt_targ_f = 0.0; self.yaw_cmd = None
        self.eR_i = 0.0                      # pitch attitude integral (trims the nose-down)
        self._ffwd_f = 0.0                   # low-passed forward force (for the pitch feed-forward)
        self._cruise = False                 # distinct CRUISE mode (committed tilt, wing-borne)
        self._adbg = (0.0,) * 6
        self.armed = self.offb = False; self.counter = self.last = 0

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
                                 f"{pre}/fsc_autopilot_ros2/position_controller/reference", self.on_ref, 10)
        self.create_subscription(String, f"{pre}/ground_station/active_controller", self.on_active, 10)
        self.create_timer(1.0 / self.rate, self.loop)
        self.get_logger().info(f"aero_transition up: id={self.cid} rate={self.rate:.0f}Hz aero_ff={self.gp('aero_ff')>0.5}")

    def gp(self, n):
        return float(self.get_parameter(n).value)

    def now_us(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def on_status(self, m):
        self.armed = (m.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        self.offb = (m.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD)

    def on_active(self, m):
        self._active = (m.data == self.cid)

    def on_mocap(self, m):
        q = m.pose.orientation
        R = quat_to_R(q.x, q.y, q.z, q.w)
        p = np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z])
        v = np.array([m.twist.linear.x, m.twist.linear.y, m.twist.linear.z])
        w = np.array([m.twist.angular.x, m.twist.angular.y, m.twist.angular.z])
        self.state = (p, v, R, w)

    def on_ref(self, m):
        p_ref = np.array([m.position.x, m.position.y, m.position.z])
        a = self.gp("ff_lpf"); dt_ref = 1.0 / 30.0
        if self._pref_prev is not None:
            vff_raw = (p_ref - self._pref_prev) / dt_ref
            pv = np.array([m.velocity.x, m.velocity.y, m.velocity.z])
            if np.linalg.norm(pv) > 1e-6:
                vff_raw = pv
            self.vff += a * (vff_raw - self.vff)
            self.aff += a * ((self.vff - self._vff_prev) / dt_ref - self.aff)
            self.vff = np.clip(self.vff, -30.0, 30.0); self.aff = np.clip(self.aff, -40.0, 40.0)
            self._vff_prev = self.vff.copy()
            if np.linalg.norm(p_ref - self._pref_prev) > 3.0:
                self.ei = 0.0
        self._pref_prev = p_ref
        self.ref = p_ref
        self.yaw_ref = (m.yaw * math.pi / 180.0
                        if m.yaw_unit == PositionControllerReference.DEGREES else m.yaw)

    def aero_world(self, v, R):
        """Modeled wing lift+drag in WORLD frame (matches vtol_aerodynamics.py). Feed-forward:
        subtracting this from F_des makes the rotors back off as the wing takes over."""
        vb = R.T @ v                                  # body FLU velocity
        u, vy, w = vb
        V = math.sqrt(u * u + vy * vy + w * w)
        if V < self.gp("aero_min_v"):
            return np.zeros(3)
        stall = math.radians(self.gp("stall_deg"))
        alpha = clamp(math.atan2(w, u if abs(u) > 1e-6 else 1e-6), -stall, stall)
        qbar = 0.5 * self.gp("rho") * V * V
        gate = (u * u) / (u * u + self.gp("aero_v_gate") ** 2)     # needs forward speed
        CL = clamp(self.gp("cl0") + self.gp("cl_alpha") * alpha, -self.gp("cl_max"), self.gp("cl_max"))
        CD = self.gp("cd0") + self.gp("k_induced") * CL * CL
        S = self.gp("wing_area")
        L = gate * qbar * S * CL; D = gate * qbar * S * CD
        vel_dir = vb / V
        drag_b = -D * vel_dir
        lift_b = L * np.array([-math.sin(alpha), 0.0, math.cos(alpha)])
        Fa = R @ (drag_b + lift_b)
        self._adbg = (V, math.degrees(alpha), CL, L, gate, float(Fa[2]))   # diag: V,AoA,CL,L,gate,aeroZ
        return Fa

    def loop(self):
        self.counter += 1
        o = OffboardControlMode(); o.direct_actuator = True; o.timestamp = self.now_us()
        if self._active:
            self.ocm.publish(o)
        if self.state is None:
            if self._active:
                self.pub([math.nan] * 4, [math.nan] * 5)
            return
        p, v, R, w = self.state
        if self.ref is None:
            self.ref = np.array([p[0], p[1], 1.5])
        m_, g = self.m, self.g
        e3 = np.array([0.0, 0.0, 1.0])
        kp = np.array([self.gp("kp_xy"), self.gp("kp_xy"), self.gp("kp_z")])
        kv = np.array([self.gp("kv_xy"), self.gp("kv_xy"), self.gp("kv_z")])
        ep = p - self.ref; ev = v - self.vff
        # vertical integral (armed + near set-point only) nulls trim/CG
        if self.armed and abs(ep[2]) < self.gp("i_band"):
            self.ei += ep[2] * (1.0 / self.rate)
        elif not self.armed:
            self.ei = 0.0
        iz = clamp(self.gp("ki_z") * self.ei, -self.gp("i_max"), self.gp("i_max"))
        if self.gp("ki_z") > 1e-9:
            self.ei = iz / self.gp("ki_z")

        F_des = m_ * (g * e3 + self.aff) - kp * ep - kv * ev
        F_des[2] -= iz
        aeroF = self.aero_world(v, R) if self.gp("aero_ff") > 0.5 else np.zeros(3)
        F_des = F_des - aeroF                             # <-- the key: subtract modeled wing force
        F_des[2] = max(F_des[2], 0.12 * m_ * g)
        if self.gp("dbg") > 0.5 and self.counter % max(1, int(self.rate)) == 0:
            V, aoa, CL, L, gate, aZ = self._adbg
            self.get_logger().info(
                "[Fz] mg=%.1f aff=%.1f p=%.1f v=%.1f iz=%.1f aeroZ=%.1f -> Fup=%.1f || "
                "wing: V=%.1f AoA=%+.1f CL=%.2f L=%.1f gate=%.2f | z=%.2f vz=%+.2f ep_z=%+.2f vff_z=%+.2f"
                % (m_ * g, m_ * self.aff[2], -kp[2] * ep[2], -kv[2] * ev[2], -iz, -aeroF[2], F_des[2],
                   V, aoa, CL, L, gate, p[2], v[2], ep[2], self.vff[2]))

        # decompose along the commanded heading: forward is the TILT's job (body stays level)
        yaw = self.yaw_ref
        fdir = np.array([math.cos(yaw), math.sin(yaw), 0.0])
        # YAW-AROUND: if the nose is not aligned with the travel direction, HOLD position (damp
        # horizontal velocity) and let the attitude loop yaw to face travel FIRST -- then transition.
        # (Chasing the target fast in a non-aligned attitude flew backward and ran away.)
        nose = R @ np.array([1.0, 0.0, 0.0])
        align = float(nose[0] * fdir[0] + nose[1] * fdir[1])
        # DISTINCT CRUISE latch: commit when the COMMANDED forward speed is high + nose aligned.
        vff_fwd = float(self.vff @ fdir)
        if self.gp("use_cruise") > 0.5 and align > self.gp("yaw_align_min"):
            if vff_fwd > self.gp("cruise_enter_v"):
                self._cruise = True
            elif vff_fwd < self.gp("cruise_exit_v"):
                self._cruise = False
        else:
            self._cruise = False
        if align < self.gp("yaw_align_min"):
            F_des[0] = -kv[0] * v[0]                       # damp only -> hover + yaw
            F_des[1] = -kv[1] * v[1]
        F_fwd = max(float(F_des @ fdir), 0.0)
        F_des_att = F_des - F_fwd * fdir                  # remove forward -> level attitude target

        # slew the ATTITUDE yaw command toward the travel heading -> smooth turns, avoiding the
        # 180deg SE(3) singularity / thrust-axis coupling that dropped the vehicle on the return leg.
        cur_yaw = math.atan2(R[1, 0], R[0, 0])
        if self.yaw_cmd is None:
            self.yaw_cmd = cur_yaw
        dyaw = math.atan2(math.sin(yaw - self.yaw_cmd), math.cos(yaw - self.yaw_cmd))
        ymax = math.radians(self.gp("yaw_rate_deg_s")) / self.rate
        self.yaw_cmd += clamp(dyaw, -ymax, ymax)
        yaw_att = self.yaw_cmd
        # desired attitude: thrust axis from the (forward-removed) force, heading = slewed yaw_att
        b3 = F_des_att / (np.linalg.norm(F_des_att) + 1e-9)
        b1c = np.array([math.cos(yaw_att), math.sin(yaw_att), 0.0])
        b2 = np.cross(b3, b1c); b2 /= (np.linalg.norm(b2) + 1e-9)
        b1 = np.cross(b2, b3)
        # command a small POSITIVE AoA (nose-up) at cruise so the wing LIFTS instead of making
        # downforce (the -20deg nose-down was the root cause); ramp with airspeed, only when aligned.
        Vspd = float(np.linalg.norm(v))
        aoa_frac = clamp((Vspd - self.gp("aoa_v_lo")) / max(self.gp("aoa_v_hi") - self.gp("aoa_v_lo"), 0.1), 0.0, 1.0)
        # baseline nose-up AoA (wing lift) MINUS altitude-rate feedback (pitch down when climbing so
        # the wing lift self-regulates and can't run the vehicle away).
        if self._cruise:
            # CRUISE: the WING holds altitude via AoA (pitch UP to climb), not the rotor collective.
            aoa = (math.radians(self.gp("cruise_aoa_deg"))
                   + math.radians(self.gp("cruise_alt_kp")) * (self.ref[2] - p[2])
                   - math.radians(self.gp("cruise_alt_kd")) * v[2])
            amax = math.radians(self.gp("cruise_aoa_max"))
        else:
            aoa = math.radians(self.gp("aoa_cmd_deg")) * aoa_frac - math.radians(self.gp("aoa_kvz")) * v[2]
            amax = math.radians(self.gp("aoa_max_deg"))
        aoa = clamp(aoa, -amax, amax)
        if align >= self.gp("yaw_align_min") and abs(aoa) > 1e-4:
            ca, sa = math.cos(aoa), math.sin(aoa)
            b1, b3 = b1 * ca + b3 * sa, -b1 * sa + b3 * ca   # pitch about b2 (nose-up for aoa>0)
        R_des = np.column_stack((b1, b2, b3))
        f = float(F_des_att @ (R @ e3))                   # collective along body-z
        eR = 0.5 * vee(R_des.T @ R - R.T @ R_des); eW = w
        # PITCH INTEGRAL: null the steady nose-down moment (P-term alone leaves ~20deg error).
        if self.armed:
            self.eR_i = clamp(self.eR_i + eR[1] * (1.0 / self.rate),
                              -self.gp("i_pitch_max"), self.gp("i_pitch_max"))
        else:
            self.eR_i = 0.0
        kR = np.array([self.gp("kR"), self.gp("kR"), self.gp("kR_yaw")])
        kW = np.array([self.gp("kW"), self.gp("kW"), self.gp("kW_yaw")])
        M = -kR * eR - kW * eW
        M[1] += -self.gp("ki_pitch") * self.eR_i          # <-- pitch trim (integral)
        self._ffwd_f += 0.08 * (F_fwd - self._ffwd_f)     # low-pass F_fwd -> smooth pitch FF (no porpoise)
        M[1] += self.gp("tilt_pitch_ff") * self._ffwd_f   # <-- tilt-moment feed-forward (nose-up)

        # allocate vertical thrust + moments -> 4 rotor VERTICAL thrusts
        Tv = self.Ainv @ np.array([f, M[0], M[1], M[2]])
        Tv = np.clip(Tv, 0.0, self.kf)
        # REAR CUTOFF: ramp the rear rotors (2,3) off with airspeed; give their vertical share to the
        # front pair so total lift is preserved (the front is tilted/boosted + the wing helps).
        rvlo = self.gp("rear_off_v_lo"); rvhi = max(self.gp("rear_off_v_hi"), rvlo + 0.1)
        rf = clamp((Vspd - rvlo) / (rvhi - rvlo), 0.0, 1.0); rear_scale = 1.0 - rf * rf * (3 - 2 * rf)
        if rear_scale < 0.999:
            shed = (1.0 - rear_scale) * (Tv[2] + Tv[3])   # vertical thrust removed from the rear
            Tv[2] *= rear_scale; Tv[3] *= rear_scale
            Tv[0] += 0.5 * shed; Tv[1] += 0.5 * shed      # front pair picks it up
            Tv = np.clip(Tv, 0.0, self.kf)
        # front rotors (0,1) provide the forward force via tilt. The tilt is COORDINATED so it
        # doesn't remove vertical thrust before the wing can carry the weight:
        tilt_max = math.radians(self.gp("tilt_max_deg"))
        front_vert = Tv[0] + Tv[1]
        tilt_target = math.atan2(F_fwd, max(front_vert, 1e-3))
        # low-pass the tilt target so it doesn't chatter with the F_fwd/velocity-error ripple
        a_t = self.gp("tilt_lpf")
        self.tilt_targ_f += a_t * (tilt_target - self.tilt_targ_f)
        tilt_target = self.tilt_targ_f
        # (a) YAW-ALIGNMENT gate (align computed above): only tilt when the nose is aligned with
        #     the travel direction, else the body-frame tilt pushes the wrong WORLD way -> lunge.
        # (b) SATURATION-HEADROOM cap (replaces the airspeed gate): the front thrust is boosted by
        #     1/cos(tilt) to hold its vertical share, so limit tilt so that boosted thrust never
        #     saturates (saturation is what actually drops vertical thrust -> fall). As the wing
        #     engages, aero-FF lowers the vertical demand Tv -> more headroom -> tilt opens toward
        #     cruise, with NO airspeed limit cycle and full acceleration authority.
        Tv_front = max(Tv[0], Tv[1], 0.5)
        cos_min = clamp(Tv_front / (self.gp("tilt_sat_margin") * self.kf), 0.06, 1.0)
        tilt_cap = min(tilt_max, math.acos(cos_min))
        tilt_cmd = min(tilt_target, tilt_cap)
        if align < self.gp("yaw_align_min"):
            tilt_cmd = 0.0
        tilt_cmd = clamp(tilt_cmd, 0.0, tilt_max)
        # (c) slew-rate limit (real servo; prevents an instantaneous tilt jump -> lunge/fall)
        dmax = math.radians(self.gp("tilt_rate_deg_s")) / self.rate
        beta = clamp(tilt_cmd, self.tilt_prev - dmax, self.tilt_prev + dmax)
        self.tilt_prev = beta
        # front-rotor thrust: boost to preserve their vertical share at the tilt (bounded because
        # aero-FF drops the vertical demand as the wing engages, so Tv is small when beta is large).
        boost = 1.0 / max(math.cos(beta), 0.15)
        T = np.array([Tv[0] * boost, Tv[1] * boost, Tv[2], Tv[3]])
        T = np.clip(T, 0.0, self.kf)
        if self._cruise:
            # ==== CRUISE OVERRIDE ====  Committed cruise: FAST-slew the tilt to the cruise angle,
            # FRONT PROPS carry the forward thrust (only), REAR OFF; pitch/altitude are the ELEVATOR +
            # wing AoA (set above). Roll = front L/R differential (+ aileron). This is the distinct,
            # wing-borne cruise -- the rotors are NOT holding altitude here.
            beta_c = math.radians(self.gp("cruise_tilt_deg"))
            dmax_c = math.radians(self.gp("cruise_tilt_rate")) / self.rate
            beta = clamp(beta_c, beta - dmax_c, beta + dmax_c)     # snap toward cruise tilt, fast
            self.tilt_prev = beta; self.tilt_targ_f = beta
            front_pair = clamp(F_fwd / max(math.sin(beta), 0.2), 0.0, 2.0 * self.kf)
            roll_diff = clamp(M[0] / (2.0 * self.gp("y_arm")), -0.5 * self.kf, 0.5 * self.kf)
            T = np.clip(np.array([front_pair * 0.5 + roll_diff,
                                  front_pair * 0.5 - roll_diff, 0.0, 0.0]), 0.0, self.kf)
        tservo = clamp(beta / (math.pi / 2), 0.0, 1.0)
        mot = [clamp(T[i] / self.kf, 0.0, 1.0) for i in range(4)]
        # CONTROL SURFACES: fade in with airspeed; drive from the attitude error so the ELEVATOR
        # holds pitch/AoA (and aileron=roll, rudder=yaw) as the rotors lose authority at high tilt.
        if self.gp("use_surfaces") > 0.5:
            sg = clamp((Vspd - self.gp("surf_v_gate")) / 4.0, 0.0, 1.0)
            elevator = clamp(self.gp("k_elev") * eR[1] * sg, -1.0, 1.0)   # pitch (sign tuned live)
            aileron = clamp(self.gp("k_ail") * eR[0] * sg, -1.0, 1.0)     # roll
            rudder = clamp(self.gp("k_rud") * eR[2] * sg, -1.0, 1.0)      # yaw
        else:
            elevator = aileron = rudder = 0.0
        self.pub(mot, [tservo, tservo, aileron, elevator, rudder])

        if self.counter % max(1, int(self.rate / 5)) == 0:
            d = math.degrees; V = float(np.linalg.norm(v))
            self.get_logger().info(
                f"p=({p[0]:.1f},{p[1]:.1f},{p[2]:.1f})->({self.ref[0]:.1f},{self.ref[1]:.1f},{self.ref[2]:.1f}) "
                f"V={V:.1f} Ffwd={F_fwd:.0f} tilt={d(beta):.0f} {'CRUISE' if self._cruise else 'hover '} "
                f"aoa={d(aoa):+.0f} mot=[{mot[0]:.2f},{mot[1]:.2f},{mot[2]:.2f},{mot[3]:.2f}] elev={elevator:+.2f} arm={self.armed}")

        if self._active and self.auto_arm and self.counter >= self.arm_after and not (self.armed and self.offb):
            if self.counter - self.last >= 100:
                self.last = self.counter
                if not self.offb:
                    self.vcmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                elif not self.armed:
                    self.vcmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def pub(self, mot, srv):
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

    def vcmd(self, cmd, p1=0.0, p2=0.0):
        m = VehicleCommand(); m.command = cmd; m.param1 = p1; m.param2 = p2
        m.target_system = m.target_component = m.source_system = m.source_component = 1
        m.from_external = True; m.timestamp = self.now_us(); self.cmdp.publish(m)


def main():
    rclpy.init()
    try:
        rclpy.spin(AeroTransition())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
