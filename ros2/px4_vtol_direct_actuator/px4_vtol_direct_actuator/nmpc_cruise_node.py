#!/usr/bin/env python3
"""In-loop TRANSITION NMPC for the FSC tilt-quadplane (direct_actuator, mocap-based).

The reactive stacked-single-loop controller (aero_transition_node) could not hold a stable
aggressive transition -- fixing one axis (tilt/thrust/altitude) let another (roll/pitch) diverge.
This node runs the OFFLINE-VALIDATED longitudinal transition NMPC IN THE LOOP so tilt + thrust are
CO-OPTIMIZED to track the forward/altitude reference (the NMPC holds altitude to 20 m/s offline
because it plans them together), and pairs it with a deliberately GENTLE, rate-damped, saturation-
limited attitude/lateral loop so the roll axis can't blow up.

  NMPC (25 Hz, CasADi/IPOPT, warm-started):  state [s, z, vs, vz, theta] (s = forward dist along
     heading, theta = BODY PITCH nose-up +), control [Tf, Tr, beta, q] (front-pair thrust tilted by
     beta, rear-pair vertical, q = pitch-rate). Wing aero in the model (same coeffs as
     vtol_aerodynamics.py), AoA = theta - flight-path. The solver CO-OPTIMIZES pitch WITH tilt+thrust,
     so it plans a genuine POSITIVE-AoA wing-borne cruise: nose up -> wing lifts -> rotors unload
     (thrust penalty), tilt supplies only the forward thrust for drag. (The old 4-state model took
     theta as a fixed parameter, forcing a separate pitch/elevator loop that fought the tilt loop.)
  Attitude/lateral (gentle):  pitch TRACKS the NMPC's planned theta; yaw slews to the travel heading;
     a small, clamped, rate-limited roll from the lateral error; surfaces fade in as an aero assist.
  Allocation: front pair = Tf (tilt beta), rear pair = Tr; small capped roll/pitch/yaw differentials.

Output: ActuatorMotors + ActuatorServos + OffboardControlMode(direct_actuator). Gated on
/<ns>/ground_station/active_controller == controller_id.
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
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy,
                       QoSHistoryPolicy, qos_profile_sensor_data)
import casadi as ca
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


class NMPCCruise(Node):
    # ---- airframe / aero (match vtol_aerodynamics.py + quad_tiltrotor_config) ----
    M = 3.5; G = 9.81
    # aero MATCHES the scaled vtol_aerodynamics.py (small 3.4 kg quadplane wing: S=0.35, CL_max=1.2)
    RHO = 1.225; S = 0.35; CL0 = 0.15; CLA = 4.8; CD0 = 0.04; KIND = 0.071; CLMAX = 1.2
    STALL = math.radians(13.0)   # synced to sim (audit): KIND 0.05->0.071, STALL 22->13
    TF_MAX = 2 * 18.0; TR_MAX = 2 * 18.0; BETA_MAX = math.radians(84.0)
    #   BETA_MAX 90->84: with the profile tilt capped at 82 (lift-margin fix, see
    #   fsc_transition_ref), the solver's tilt never needs 90; keeping a cos(beta) floor
    #   preserves front-pair vertical authority for phugoid damping (HIL bag fwD5).
    # Planned body-pitch bounds. Wing-borne cruise needs only ~3 deg nose-up (offline), and a nose-up
    # plan pushes the thrust vector BACKWARD -- so allow only a SMALL nose-up trim (large excursions
    # tumbled it in HIL). Nose-DOWN a bit more (helps forward accel). Pitch-rate is DELIBERATELY SLOW:
    # pitch is a gentle trim, NOT a fast altitude actuator (that fast loop limit-cycled through the HIL
    # latency). The fast altitude/forward control stays with the rotors (Tf/Tr) + tilt, which respond fast.
    # Nose-DOWN is tightly limited: a tiltrotor gets forward thrust from the TILT, not by pitching the
    # nose down. Allowing nose-down let the NMPC "chase" the leading setpoint nose-down -> NEGATIVE AoA
    # -> wing DOWNFORCE (not lift). Capping nose-down forces forward thrust into the tilt and keeps the
    # body level/nose-up, so the wing makes LIFT = wing-borne. Nose-up allows the wing trim.
    TH_UP = math.radians(9.0); TH_DN = math.radians(3.0)     # nose-up (wing trim) / nose-down bounds
    Q_MAX = math.radians(7.0)                                # planned pitch-rate bound [rad/s] (slow trim)
    N = 15; DT = 0.08                                   # 1.2 s horizon (fast enough for ~25 Hz)

    def __init__(self):
        super().__init__("nmpc_cruise_control")
        P = self.declare_parameter
        P("px4_ns", "uav_0"); P("controller_id", "nmpc_cruise"); P("active_at_start", False)
        P("auto_arm", True); P("arm_after", 150); P("rate_hz", 25.0)
        P("max_thrust", 18.0); P("cg_x", 0.177); P("cQ", 0.06); P("y_arm", 0.285)
        #   cg_x = 0.177 MEASURED from the sim's balanced hover (front 8.6 N > rear 8.4 N, SO(3)
        #   bring-up); it now matters -- the pitch-balance cost term uses these thrust-moment arms.
        # attitude / lateral. HIL-TUNED: kR=9 + moment caps 8/8/4 N.m are REQUIRED -- soft gains
        # (kR=5) and low caps (3) starved the attitude loop of authority and it diverged/crashed on
        # ANY maneuver (climb or forward); hover alone was fine. Firm authority = stable transition.
        P("kR", 9.0); P("kW", 2.5); P("kR_yaw", 1.5); P("kW_yaw", 0.6)
        P("kp_lat", 0.8); P("kv_lat", 1.6); P("max_roll_deg", 18.0); P("roll_rate_deg_s", 40.0)
        P("kp_fwd", 1.0); P("kv_fwd", 2.0); P("max_pitch_deg", 15.0); P("pitch_rate_deg_s", 40.0)
        P("yaw_rate_deg_s", 30.0)           # gentle yaw slew (fast yaw near target -> spin)
        P("yaw_face_dist", 10000.0)         # face the target only if farther than this (BIG = hold heading,
        #                                     yaw-to-target flips 180 deg if the vehicle drifts past target)
        P("align_min", 0.80)               # cos(heading err) needed before commanding forward thrust
        P("engage_dist", 15.0)              # engage TILT-cruise only when the target is farther than this;
        P("disengage_dist", 10.0)           # ...and drop back to PITCH-hold when nearer (hysteresis: no chatter).
        #                                     Widened 4/1.5 -> 15/10 so the fore-aft PD hold (below) has
        #                                     ROOM to decelerate from cruise speed before the target
        #                                     instead of arriving overspeed and overshooting.
        P("cruise_alt_min", 4.0)            # only tilt-cruise ABOVE this altitude (take off vertically first)
        P("m_roll_max", 8.0); P("m_pitch_max", 8.0); P("m_yaw_max", 4.0)   # moment saturation [N m] (HIL-tuned)
        P("vzmax", 3.0)                     # reference climb/descent rate cap [m/s]
        P("vmax_fwd", 16.0)                 # forward-speed cap for the horizon reference [m/s]
        P("a_fwd", 1.5)                     # forward-accel cap for the horizon reference [m/s^2]
        # surfaces (fade in with airspeed)
        # Elevator OFF by default: eR[1]-driven elevator did NOT give stable pitch regulation in HIL
        # (it drove the pitch further nose-up as the vehicle over-pitched -> runaway to stall). Pitch is
        # controlled by differential rotor thrust (kept authoritative by rotor_min). ail/rud also off.
        P("use_surfaces", 1.0); P("k_elev", -0.15); P("k_ail", -0.15); P("k_rud", -0.10); P("surf_v_gate", 3.0)
        P("rotor_min", 8.0)                 # min NMPC pair thrust [N] -> rotors stay loaded = pitch authority
        P("use_body_aoa", 1.0)              # 1 = couple measured body pitch into the NMPC aero AoA
        #                                     (models the nose-down cruise wing downforce; 0 = old flight-path AoA)
        # CRUISE AoA-hold: the elevator drives the body to a POSITIVE AoA so the WING LIFTS. As the wing
        # carries weight the NMPC (which now models the wing) needs less vertical thrust -> it tilts the
        # rotors FORWARD and unloads them = a real wing-borne cruise. Altitude feedback trims the AoA
        # target down if high/climbing so the wing lift can't run away vertically. 0 disables (default).
        P("cruise_aoa_deg", 0.0)           # target cruise AoA [deg] (+5..6 = wing lifts); 0 = OFF
        P("k_aoa_elev", -4.0)              # elevator per rad of AoA error. NEGATIVE (rosbag wb2: elev=-1
        #                                    drove the nose DOWN -> RAISED the AoA -> need +elev to lower it)
        P("aoa_v_gate", 5.0)               # airspeed [m/s] to engage the AoA-hold (wing regime)
        P("aoa_z_min", 5.0)                # min altitude [m] to engage the AoA-hold (not takeoff/land)
        P("k_aoa_alt", 0.08); P("k_aoa_vz", 0.08)   # AoA target trim: -rad per m high, per m/s climb
        P("rear_off_cruise", 0.0)          # 1 = fade the rear rotors out at cruise speed (front+wing carry)
        P("z_bail", 1.0); P("bail_thr", 0.85)   # altitude-loss safety: level climb if we drop near ground
        # DISTANCE-SCHEDULED DECELERATION BRAKE: pitch nose-up (thrust tilts BACKWARD) to hold a sqrt
        # stopping profile v_stop=sqrt(2*a_brake*dist_to_ref) toward the target. Starts braking EARLY
        # (dist^2 scheduled) so it arrives at ~0 speed instead of coasting past. The 5-state pitch trim
        # else holds the body LEVEL (weak decel) and a tiltrotor can't reverse-tilt -> huge overshoot.
        P("a_brake", 2.5)                   # deceleration the brake schedules for [m/s^2] (g*sin(theta))
        P("k_brake", 0.20); P("brake_max_deg", 18.0)   # nose-up [rad] per m/s of excess speed; cap
        # FORE-AFT POSITION-HOLD PD (replaces the old one-sided brake) -- signed, so it damps a backward
        # slide (HIL: the old one-sided brake ran 162->82 m). Gains MUCH smaller than the lateral PD: the
        # fore-aft axis sees the full ~cruise speed (5-7 m/s) whereas the lateral velocity is tiny, so
        # lateral-magnitude gains (kv~1.6) demanded ~80deg nose-up at cruise -> saturated the 18deg cap ->
        # bang-bang -> violent fore-aft oscillation (HIL fl4). Sized so vs=6 m/s -> ~12deg (below the cap).
        P("kp_fwd_hold", 0.15); P("kv_fwd_hold", 0.40)
        #   softened 0.30/30 -> 0.20/18: a -30 deg brake at 6.5 m/s pitched past the recovery
        #   envelope and tumbled the post-arrival hold (HIL bag fwD8)
        P("cruise_trim_deg", 3.0)           # max nose-up wing-trim reference at cruise speed [deg]
        #   MODEST on purpose: a small +AoA -> the wing carries ~30-40%, but the ROTORS STAY LOADED so
        #   they keep differential-thrust pitch authority. A big trim unloaded the rotors to 0 -> the
        #   only pitch control left was the elevator (under-damped) -> pitch ran away nose-up -> climb.
        P("trim_v_gate", 6.0)               # forward speed [m/s] at which the nose-up trim starts ramping
        # ---- AVTOL-scaled TRANSITION PROFILE (fsc_transition_ref.py): the FSC-scaled trim schedule
        # V -> (tilt, theta, Ttot, de) from the user's AVTOL reference generators. With use_profile=1
        # the horizon THETA reference comes from the profile trim (instead of the heuristic
        # cruise_trim ramp) AND the NMPC gets a TILT-TRACKING term toward the profile tilt(V) --
        # this is what drives FULL fixed-wing cruise (tilt -> ~90 deg, wing carries the weight)
        # instead of the wing-assisted-multicopter drift (tilt~0) the plain thrust penalty produced.
        # The rotor floor is wing-share-SCHEDULED: full rotor_min in the hover regime (differential-
        # thrust authority), relaxing to rotor_min_cruise as the wing takes the weight (the strong
        # elevator at cruise speed provides the pitch authority instead).
        P("use_profile", 1.0)               # 1 = drive theta/tilt refs from the AVTOL-scaled profile
        P("profile_v_cruise", 15.0)         # profile cruise airspeed [m/s] (full wing-borne ~13.3+)
        P("profile_tf", 20.0)               # profile transition duration used by the generator [s]
        P("w_beta_ref", 15.0)               # NMPC weight on (beta - beta_profile(V))^2 tilt tracking
        P("w_theta_ref", 25.0)              # NMPC weight on (theta - theta_profile(V))^2 (profile mode)
        #   both FIRM on purpose: the profile refs are force-balanced trims, and weak weights left the
        #   solver in a rotor-borne local minimum (tilt ~44, theta ~0, rear rotor carrying 23 N).
        P("w_T_ref", 0.02)                  # NMPC weight on (Tf+Tr - Ttot_profile(V))^2 [1/N^2]
        #   the DECISIVE term: tilt/theta tracking alone left a KKT trap where the rear rotor carries
        #   the weight level-bodied. Tracking the profile's THRUST hand-off makes that trap expensive,
        #   so the solver must raise theta (wing lift) as it unloads -- the AVTOL Ttot schedule.
        P("w_pitch_bal", 3.0)               # NMPC weight on the PITCH-BALANCE moment
        #   (Tf*cos(beta)*x_f - Tr*x_r)^2 [1/(N.m)^2]. The point-mass model is BLIND to the pitch
        #   moment of the front/rear split: with the profile beta-ref tilting the front early, the
        #   solver loaded the REAR to compensate vertically (mF 0.38 vs mR 1.00) -> a real nose-over
        #   beyond the attitude loop's authority -> backward crash (HIL bag fwD2). This term makes
        #   the split pitch-trimmed at every tilt AND drives the correct rear unload at high tilt.
        P("rotor_min_cruise", 6.0)          # pair-thrust floor once the wing carries the weight [N]
        #   6.0 N/pair (12 N total ~ 35% weight reserve): the vertical-loop BANDWIDTH that damps
        #   the cruise phugoid lives in this rotor reserve -- at 1.5-3 N the cruise porpoised
        #   +-10 m at the ~6-10 s phugoid period in every run (fwD5..fwD9), matching the
        #   historically-stable wing-borne runs which kept ~50% rotor reserve (bags wb10/wb11).
        #   Wing still carries ~65-70% = fixed-wing cruise; push lower floors via ros2 param for
        #   envelope research.
        P("k_trim_alt", 0.08)               # reduce nose-up trim per m ABOVE z_ref [rad/m] (anti climb-away)
        P("k_trim_vz", 0.0)                 # (climb-rate trim feedback OFF: it set up a pitch/altitude limit cycle)
        P("theta_lp", 0.4)                  # low-pass alpha on the measured body pitch feeding the NMPC
        #                                     state (the raw AoA-derived pitch is noisy -> a fast x0
        #                                     feedback that limit-cycled in HIL; smoothing it damps that).

        ns = self.get_parameter("px4_ns").value
        pre = f"/{ns}" if ns else ""
        self.cid = self.get_parameter("controller_id").value
        self._active = bool(self.get_parameter("active_at_start").value)
        self.auto_arm = bool(self.get_parameter("auto_arm").value)
        self.arm_after = int(self.get_parameter("arm_after").value)
        self.rate = float(self.gp("rate_hz")); self.kf = float(self.gp("max_thrust"))

        cg = self.gp("cg_x")
        self.rp = np.array([[0.459 - cg, 0.285], [0.459 - cg, -0.284],
                            [-0.112 - cg, 0.284], [-0.112 - cg, -0.284]])
        self.spin = np.array([+1.0, -1.0, -1.0, +1.0])

        # AVTOL-scaled transition profile (the FSC-scaled reference schedule). Loaded once; the
        # horizon references + rotor floor + elevator trim index it by speed each solve.
        self.prof = None
        if self.gp("use_profile") > 0.5:
            try:
                from px4_vtol_direct_actuator.lib.fsc_transition_ref import FSCTransitionProfile
                self.prof = FSCTransitionProfile(V_cruise=self.gp("profile_v_cruise"),
                                                 tf=self.gp("profile_tf"))
                self.get_logger().info(
                    "AVTOL-scaled profile loaded: Vc=%.1f v_wing_borne=%.1f m/s"
                    % (self.prof.V_cruise, self.prof.v_wing_borne))
            except Exception as e:
                self.get_logger().warn(f"transition profile unavailable ({e}) -> heuristic trim")

        self._build_nmpc()
        self.zsol = None                    # warm-start
        self._u_applied = np.array([self.M * self.G / 2, self.M * self.G / 2, 0.0, 0.0])
        self._beta_applied = 0.0
        # last-good NMPC control (fallback when a solve does NOT converge -- see the failed-solve
        # guard in loop(): IPOPT returns a non-converged iterate on max_iter, which must never be
        # applied or warm-started from).
        self._u_good = np.array([self.M * self.G / 2, self.M * self.G / 2, 0.0, 0.0])
        self._theta_good = 0.0
        self.s0 = None                      # forward-distance origin (set on first ref)
        self.roll_prev = 0.0; self.pitch_prev = 0.0; self.yaw_cmd = None; self._cruising = False
        self._aoa_regime = False; self._aoa_cmd = 0.0; self._z_ramp = None; self._theta_f = None
        self.state = None; self.ref = None; self.yaw_ref = 0.0; self.vff = np.zeros(3)
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
        self.get_logger().info(f"nmpc_cruise up: id={self.cid} rate={self.rate:.0f}Hz N={self.N}")

    def gp(self, n):
        return float(self.get_parameter(n).value)

    # ---- NMPC (longitudinal): state [s,z,vs,vz,theta] , control [Tf,Tr,beta,q] ----
    # theta = BODY PITCH (nose-up +) is now a PLANNED STATE (rate q). The solver CO-OPTIMIZES pitch
    # with tilt+thrust, so it plans a POSITIVE-AoA wing-borne cruise: pitch nose-up -> the wing lifts ->
    # the rotors unload (thrust penalty), the tilt provides only the forward thrust for drag. The old
    # 4-state model took theta as a FIXED external parameter, so a SEPARATE pitch/elevator loop had to
    # chase the AoA and fought the tilt/thrust loop (never reached wing-borne). AoA = theta - gamma.
    def _f(self, X, U):
        s, z, vs, vz, th = X[0], X[1], X[2], X[3], X[4]
        Tf, Tr, beta, q = U[0], U[1], U[2], U[3]
        V = ca.sqrt(vs * vs + vz * vz + 0.25)
        gamma = ca.atan2(vz, ca.sqrt(vs * vs + 0.5))
        alpha_raw = th - gamma
        alpha = ca.fmax(ca.fmin(alpha_raw, self.STALL), -self.STALL)
        qbar = 0.5 * self.RHO * V * V
        gate = (vs * vs) / (vs * vs + 4.0)
        CL = ca.fmax(ca.fmin(self.CL0 + self.CLA * alpha, self.CLMAX), -self.CLMAX)
        # post-stall break (matches the sim's 2026-08-10 physics fix) -- SMOOTH tanh blend
        # (piecewise fmax/fabs kinks broke IPOPT convergence exactly at the stall boundary)
        a_abs = ca.sqrt(alpha_raw * alpha_raw + 1e-6)
        over = 0.5 * (1.0 + ca.tanh((a_abs - math.radians(26.0)) / math.radians(3.0)))
        f_st = 1.0 - 0.4 * over
        CL = CL * f_st
        CD = self.CD0 + self.KIND * CL * CL + 0.5 * (1.0 - f_st)
        L = gate * qbar * self.S * CL; D = gate * qbar * self.S * CD
        Fax = -D * vs / V - L * vz / V
        Faz = -D * vz / V + L * vs / V
        # rotor thrust in BODY frame (forward from tilt beta, up), rotated to world by body pitch th.
        fx_b = Tf * ca.sin(beta); fz_b = Tf * ca.cos(beta) + Tr
        Frx = fx_b * ca.cos(th) - fz_b * ca.sin(th)
        Frz = fx_b * ca.sin(th) + fz_b * ca.cos(th)
        return ca.vertcat(vs, vz, (Frx + Fax) / self.M,
                          (Frz + Faz - self.M * self.G) / self.M, q)

    def _build_nmpc(self):
        N, DT, nx, nu = self.N, self.DT, 5, 4
        Xs = ca.SX.sym('X', nx, N + 1); Us = ca.SX.sym('U', nu, N)
        # P layout: [x0(nx), ref(nx*(N+1)), beta_ref(N), Ttot_ref(N), w_beta, w_th, w_T, w_pm,
        # uprev(nu)]. The tail lets the AVTOL-profile tilt/thrust references + weights be set at
        # runtime (0s = legacy), and carries the PREVIOUSLY APPLIED control for the rate penalty.
        Pp = ca.SX.sym('P', nx + nx * (N + 1) + 2 * N + 4 + nu)
        x0 = Pp[0:nx]; ref = ca.reshape(Pp[nx:nx + nx * (N + 1)], nx, N + 1)
        b0 = nx + nx * (N + 1)
        beta_ref = Pp[b0:b0 + N]; Ttot_ref = Pp[b0 + N:b0 + 2 * N]
        w_beta = Pp[b0 + 2 * N]; w_th = Pp[b0 + 2 * N + 1]; w_T = Pp[b0 + 2 * N + 2]
        w_pm = Pp[b0 + 2 * N + 3]
        uprev_p = Pp[b0 + 2 * N + 4:b0 + 2 * N + 4 + nu]
        # thrust-moment arms about the CG (from the same rotor geometry as the allocation)
        xf_arm = float(self.rp[0, 0]); xr_arm = float(-self.rp[2, 0])
        # Track altitude(z)+velocity FIRMLY, forward-pos loosely (so it holds the approach speed, not
        # dive to cover a distant waypoint). THETA has only a tiny regularization weight (keep pitch
        # bounded / level at hover where the wing gives no benefit) -- otherwise it is FREE to pitch up.
        # A small THRUST^2 penalty makes the solver PREFER wing lift over rotor thrust: at cruise speed
        # pitching nose-up gives +AoA -> the wing carries the weight -> the rotors unload = wing-borne.
        wp = ca.DM([2.0, 30.0, 1.0]); wv = ca.DM([8.0, 12.0])           # [s,z,theta] , [vs,vz]
        wthr = 6.0e-3                                                    # thrust^2 penalty (rotor unload)
        # theta-weight is a PHYSICS-BASED SPEED GATE: at low speed the wing saves almost no thrust
        # (lift ~ V^2), so the theta cost keeps the body LEVEL + rotors loaded (no premature unloading
        # -> no hunting); at cruise speed the thrust saving from wing lift dominates -> it pitches up
        # and unloads = wing-borne. (offline: 16 m/s -> theta~3 deg, wing carries ~65%, rotor 37->12.6 N,
        # altitude held +-0.06 m; at 5 m/s it stays ~level, rotors loaded.)
        wdu = ca.DM([5e-3, 5e-3, 10.0, 12.0])                          # rate penalty [Tf,Tr,beta,q]
        #   beta rate raised 6->10: near the wing-borne boundary the solver flip-flopped the tilt
        #   80<->0 deg between solves (bistable thrust/wing trade, HIL bag fwD4); together with the
        #   applied-beta slew this damps the replanning limit cycle.
        g = [Xs[:, 0] - x0]; J = 0
        # Uprev = the PREVIOUSLY APPLIED control (via P), NOT a hardcoded hover trim. A constant
        # hover anchor made the first-step du penalty pull beta0 toward 0 EVERY cycle -> the solver
        # planned the tilt transition but perpetually DEFERRED it past step 0 (closed loop never
        # tilted: the "rotors stay vertical in cruise" mystery). Anchoring to the applied control
        # makes wdu a true rate penalty across cycles.
        Uprev = uprev_p
        for k in range(N):
            ep = Xs[0:2, k] - ref[0:2, k]; ev = Xs[2:4, k] - ref[2:4, k]; eth = Xs[4, k] - ref[4, k]
            # theta weight is runtime (w_th: profile mode tracks the trim pitch firmly; legacy = wp[2]).
            # beta tracking toward the AVTOL-profile tilt(V) is what drives FULL fixed-wing cruise
            # (w_beta=0 -> legacy free tilt).
            eb = Us[2, k] - beta_ref[k]; eT = Us[0, k] + Us[1, k] - Ttot_ref[k]
            epm = Us[0, k] * ca.cos(Us[2, k]) * xf_arm - Us[1, k] * xr_arm   # pitch-balance moment
            J += (ca.dot(wp[0:2] * ep, ep) + w_th * eth * eth + ca.dot(wv * ev, ev)
                  + wthr * (Us[0, k] ** 2 + Us[1, k] ** 2) + w_beta * eb * eb + w_T * eT * eT
                  + w_pm * epm * epm)
            du = Us[:, k] - (Uprev if k == 0 else Us[:, k - 1]); J += ca.dot(wdu * du, du)
            k1 = self._f(Xs[:, k], Us[:, k]); k2 = self._f(Xs[:, k] + DT / 2 * k1, Us[:, k])
            k3 = self._f(Xs[:, k] + DT / 2 * k2, Us[:, k]); k4 = self._f(Xs[:, k] + DT * k3, Us[:, k])
            g.append(Xs[:, k + 1] - (Xs[:, k] + DT / 6 * (k1 + 2 * k2 + 2 * k3 + k4)))
            Uprev = Us[:, k]
        epT = Xs[0:2, N] - ref[0:2, N]; evT = Xs[2:4, N] - ref[2:4, N]
        wpT = ca.DM([1.0, 45.0]); wvT = ca.DM([12.0, 18.0])   # terminal: hold altitude+velocity, light on fwd pos
        J += ca.dot(wpT * epT, epT) + ca.dot(wvT * evT, evT)
        Z = ca.vertcat(ca.reshape(Xs, -1, 1), ca.reshape(Us, -1, 1))
        nlp = {'x': Z, 'f': J, 'g': ca.vertcat(*g), 'p': Pp}
        # max_iter 30->60: at 25 Hz (40 ms budget) a solve is ~8 ms/iter-cluster; 30 iters routinely
        # ran OUT during the hard hover->cruise transition (returning a non-converged iterate that the
        # failed-solve guard now rejects), stalling progress. 60 lets the transition solves CONVERGE.
        # acceptable_iter=8: accept a "good enough" (acceptable_tol) point after 8 stable iters so a
        # slow-converging solve still returns Solved_To_Acceptable_Level (which the guard accepts).
        self.solver = ca.nlpsol('s', 'ipopt', nlp,
                                {'ipopt.print_level': 0, 'print_time': 0, 'ipopt.max_iter': 60,
                                 'ipopt.acceptable_tol': 1e-3, 'ipopt.tol': 1e-3,
                                 'ipopt.acceptable_iter': 8,
                                 'ipopt.warm_start_init_point': 'yes'})
        self.off = nx * (N + 1)
        self.lbx = -ca.inf * ca.DM.ones(Z.shape); self.ubx = ca.inf * ca.DM.ones(Z.shape)
        for k in range(1, N + 1):                              # theta STATE bounds (allow nose-up for lift;
            self.lbx[nx * k + 4] = -self.TH_DN                 # leave k=0 free -- it is pinned to x0)
            self.ubx[nx * k + 4] = self.TH_UP
        # Rotor thrust FLOOR (rotor_min): the NMPC keeps a minimum pair thrust so the rotors never fully
        # unload -> they RETAIN differential-thrust pitch authority + damping. Without it, at wing-borne
        # onset the rotors dropped to 0, the pitch loop lost damping, and the pitch ran away to stall.
        rmin = self.gp("rotor_min")
        for k in range(N):
            self.lbx[self.off + nu * k + 0] = rmin; self.ubx[self.off + nu * k + 0] = self.TF_MAX
            self.lbx[self.off + nu * k + 1] = rmin; self.ubx[self.off + nu * k + 1] = self.TR_MAX
            self.lbx[self.off + nu * k + 2] = 0; self.ubx[self.off + nu * k + 2] = self.BETA_MAX
            self.lbx[self.off + nu * k + 3] = -self.Q_MAX; self.ubx[self.off + nu * k + 3] = self.Q_MAX
        self.lbg = ca.DM.zeros((nx * (N + 1), 1)); self.ubg = ca.DM.zeros((nx * (N + 1), 1))
        self.nx, self.nu = nx, nu

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
        self.ref = np.array([m.position.x, m.position.y, m.position.z])
        # Use the reference velocity DIRECTLY (do NOT hold the last non-zero value): when a mission's
        # velocity FF decays to ~0 approaching the target, holding the old cruise velocity kept
        # track_ff on -> the vehicle never decelerated and OVERSHOT the target (landed ~2x too far).
        self.vff = np.array([m.velocity.x, m.velocity.y, m.velocity.z])
        self.yaw_ref = (m.yaw * math.pi / 180.0
                        if m.yaw_unit == PositionControllerReference.DEGREES else m.yaw)

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
        # ALTITUDE-LOSS SAFETY: dropped near the ground while commanded well above it -> level climb
        # (stops a diverging forward run from crashing/inverting, so it stays recoverable on geometric).
        if self._active and p[2] < self.gp("z_bail") and self.ref[2] > p[2] + 0.5:
            climb = clamp(self.gp("bail_thr"), 0.0, 1.0)
            self.pub([climb, climb, climb, climb], [0.0, 0.0, 0.0, 0.0, 0.0])
            if self.counter % 15 == 0:
                self.get_logger().warn("Z-BAIL: z=%.1f<%.1f ref_z=%.1f -> level climb thr=%.2f"
                                       % (p[2], self.gp("z_bail"), self.ref[2], climb))
            return
        # ---- HEADING: face the horizontal target when far; hold current heading at hover.
        #      The longitudinal frame uses the TRUE body yaw (so s/vs are real motion, not a
        #      scrambled world axis); the target is PROJECTED onto it, so the forward demand
        #      grows naturally as we yaw to face the target. ----
        cur_yaw = math.atan2(R[1, 0], R[0, 0])
        if self.yaw_cmd is None or not self._active:
            self.yaw_cmd = cur_yaw          # standby/first loop: latch current heading
            self.yaw_ref = cur_yaw          # ...and hold it until a reference commands a heading
        # Track the COMMANDED reference heading (self.yaw_ref, set by on_ref from the mission/GS toward
        # the target) so the tiltrotor FACES where it's going and flies FORWARD (cruise). A fixed
        # commanded yaw is also a yaw-HOLD (disturbances restore to it). This is the EXPLICIT heading,
        # not the position-relative yaw-to-target that flipped 180 deg when the vehicle drifted past.
        d = self.ref[:2] - p[:2]; dist = float(math.hypot(d[0], d[1]))
        psi_tgt = self.yaw_ref
        yrate = math.radians(self.gp("yaw_rate_deg_s")) / self.rate
        self.yaw_cmd += clamp(math.atan2(math.sin(psi_tgt - self.yaw_cmd),
                                         math.cos(psi_tgt - self.yaw_cmd)), -yrate, yrate)
        fdir = np.array([math.cos(cur_yaw), math.sin(cur_yaw), 0.0])
        ldir = np.array([-math.sin(cur_yaw), math.cos(cur_yaw), 0.0])
        align = math.cos(math.atan2(math.sin(psi_tgt - cur_yaw), math.cos(psi_tgt - cur_yaw)))

        # ---- project to the longitudinal plane along the (true) body heading ----
        s = float(p @ fdir); z = float(p[2])
        vs = float(v @ fdir); vz = float(v[2])
        s_ref = float(self.ref @ fdir)
        # RAMP the altitude reference (limit climb/descent rate) so a big z step doesn't command a
        # max-thrust climb -> fast vertical velocity corrupts the AoA + tumbles (rosbag-diagnosed).
        z_cmd = float(self.ref[2]); dzmax = self.gp("vzmax") / self.rate
        if self._z_ramp is None:
            self._z_ramp = z
        self._z_ramp = clamp(z_cmd, self._z_ramp - dzmax, self._z_ramp + dzmax)
        z_ref = self._z_ramp
        vs_ff = float(self.vff @ fdir); vz_ref = clamp(float(self.vff[2]), -self.gp("vzmax"), self.gp("vzmax"))
        # forward gate: chase the projected target ONLY when roughly FACING it AND it's ahead.
        # Otherwise HOLD with ZERO forward speed (a TRUE static hover). A nonzero hold speed made
        # the hover creep forward, which then tripped the yaw-to-target -> spin/fly-away.
        # HYSTERESIS on tilt-cruise vs pitch-hold: engage cruise only when the target is clearly far
        # ahead, stay in cruise until nearly arrived. (A single threshold chattered tilt 0<->12 deg as
        # the converging setpoint crossed it repeatedly, keeping cruise slow/jerky.)
        far = s_ref - s
        if align < self.gp("align_min") or p[2] < self.gp("cruise_alt_min"):
            self._cruising = False          # take off VERTICALLY first: no tilt-cruise below cruise alt
            #                                 (a far set-point else tilted it fwd at z=2 m -> crash)
        elif self._cruising:
            if far < self.gp("disengage_dist"):
                self._cruising = False
        elif far > self.gp("engage_dist"):
            self._cruising = True
        hold = not self._cruising
        # VELOCITY + ACCEL limited horizon reference: ramp vs from the CURRENT speed toward vlim at
        # a_fwd, with a sqrt decel profile near the target (a velocity STEP made the NMPC lunge/dive).
        vmax = self.gp("vmax_fwd"); a_fwd = self.gp("a_fwd")
        if hold:
            s_ref = s; vlim = 0.0
        else:
            vlim = min(abs(vs_ff), vmax) if abs(vs_ff) > 0.1 else vmax
        # TRACK the SMC velocity FF as the cruise speed. When the SMC commands a real cruise velocity
        # (vs_ff big) DON'T sqrt-decelerate toward its moving lead set-point -- that was throttling the
        # vehicle to ~1.5 m/s while the SMC asked for 14. Only decel toward the set-point when the FF is
        # small (final approach, SMC v -> 0). Position ref = advance at the tracked velocity.
        track_ff = (not hold) and abs(vs_ff) > 1.0
        # MEASURED body pitch (nose-up +) for the NMPC theta STATE init: theta = AoA + flight-path,
        # with the AoA computed the sim's EXACT way (v_body = R^T v_world, alpha=atan2(w,u)) so the
        # NMPC's alpha = theta - gamma matches the sim aero at the current state.
        v_body = R.T @ v
        u_b, w_b = float(v_body[0]), float(v_body[2])
        # gates at 2.0 m/s (was 0.2): below that the AoA/flight-path angles are NOISE -- a small
        # hover drift (vs ~0.5) produced theta_meas = 9 deg garbage, the attitude loop executed a
        # bogus 8-deg nose-down at hover, and the run was doomed before the transition (bag fwD7).
        alpha_meas = math.atan2(w_b, u_b) if abs(u_b) > 2.0 else 0.0
        gamma_meas = math.atan2(vz, vs) if abs(vs) > 2.0 else 0.0
        theta_raw = clamp(alpha_meas + gamma_meas, -self.TH_DN, self.TH_UP)
        a_lp = clamp(self.gp("theta_lp"), 0.02, 1.0)         # low-pass the noisy AoA-derived body pitch
        self._theta_f = theta_raw if self._theta_f is None else (1 - a_lp) * self._theta_f + a_lp * theta_raw
        theta_meas = self._theta_f
        X0 = np.array([s, z, vs, vz, theta_meas])
        # horizon reference: theta_ref = 0 (LEVEL target, tiny weight) -> the solver is free to pitch up
        # for wing lift; s/z/vs/vz as before (velocity+accel limited, sqrt-decel unless tracking a FF).
        # theta_ref is a SPEED-SCHEDULED, ALTITUDE-REGULATED nose-up WING TRIM: at cruise speed the
        # reference body pitch ramps nose-up so the NMPC targets a POSITIVE AoA (wing lift), but the
        # trim BACKS OFF if we climb above z_ref (or are climbing) so the wing can't over-lift and run
        # away vertically -- it self-limits to the AoA that HOLDS altitude (partial wing-borne, rotors
        # stay ~half-loaded = keep fast altitude authority). Below the speed gate it stays level.
        tvg = self.gp("trim_v_gate"); ttrim_max = math.radians(self.gp("cruise_trim_deg"))
        alt_corr = self.gp("k_trim_alt") * (p[2] - z_ref) + self.gp("k_trim_vz") * vz
        use_prof = self.prof is not None
        pref = [X0]; beta_refs = []; ttot_refs = []; sr = s; vr = vs
        for k in range(self.N + 1):
            vr = clamp(vlim, vr - a_fwd * self.DT, vr + a_fwd * self.DT)
            if not track_ff:
                remaining = max(s_ref - sr, 0.0)                        # distance still to cover
                # sqrt stop profile sized to the REAL braking (drag-limited ~1 m/s^2 -- a
                # tiltrotor cannot reverse-tilt; the optimistic a_fwd profile arrived overspeed
                # from cruise -> backward brake/re-cruise limit cycle, HIL bag fwD6)
                vr = min(vr, math.sqrt(2.0 * 1.0 * remaining + 1e-4))
            sr += vr * self.DT
            if use_prof:
                # AVTOL-scaled trim schedule: theta ref = the force-balanced trim pitch at this
                # speed (speed-gated to level at hover), tilt ref = the profile tilt(V). As vr
                # ramps up the schedule walks hover -> full fixed-wing; on decel it walks back.
                sch = self.prof.by_speed(vr)
                th_ref = clamp(sch["theta_cmd"] - alt_corr, -math.radians(2.0), self.TH_UP)
                if k < self.N:
                    beta_refs.append(sch["tilt"]); ttot_refs.append(sch["Ttot"])
            else:
                base_trim = ttrim_max * clamp((vr - tvg) / 4.0, 0.0, 1.0)   # full trim by (gate+4) m/s
                th_ref = clamp(base_trim - alt_corr, -math.radians(2.0), ttrim_max)
                if k < self.N:
                    beta_refs.append(0.0); ttot_refs.append(0.0)
            pref.append(np.array([sr, z_ref, vr, vz_ref, th_ref]))
        wb = self.gp("w_beta_ref") if use_prof else 0.0
        wt = self.gp("w_theta_ref") if use_prof else 1.0
        wT = self.gp("w_T_ref") if use_prof else 0.0
        wpm = self.gp("w_pitch_bal") if use_prof else 0.0
        pdata = np.concatenate(pref + [np.array(beta_refs), np.array(ttot_refs),
                                       np.array([wb, wt, wT, wpm]), self._u_applied])
        # wing-share-SCHEDULED rotor floor: full rotor_min while the rotors carry the weight
        # (differential-thrust pitch authority), relaxing toward rotor_min_cruise as the wing takes
        # over (the elevator has the authority at cruise speed; a hard 8 N floor would fight the
        # full unload that airplane-mode needs).
        if use_prof:
            ws_now = self.prof.wing_share(max(vs, 0.0))
            rmin_eff = self.gp("rotor_min") * (1.0 - ws_now) + self.gp("rotor_min_cruise") * ws_now
            for k in range(self.N):
                self.lbx[self.off + self.nu * k + 0] = rmin_eff
                self.lbx[self.off + self.nu * k + 1] = rmin_eff
                # HARD-BOUND the tilt by the schedule: the profile tilt(V) is the certified
                # force-balanced trim. Left soft, the solver gambled on FULL tilt at 11.6 m/s
                # (schedule: 69 deg) chasing the speed reward -> the wing wasn't carrying yet ->
                # vertical deficit -> stall-drop (HIL bag fwD3).
                self.ubx[self.off + self.nu * k + 2] = min(self.BETA_MAX,
                                                           beta_refs[k] + math.radians(6.0))
        else:
            ws_now = 0.0
        if self.zsol is None:
            self.zsol = np.concatenate([np.tile(X0, (self.N + 1,)),
                                        np.tile([self.M * self.G / 2, self.M * self.G / 2, 0.0, 0.0], (self.N,))])
        theta_cmd = theta_meas; qa = 0.0
        # FAILED-SOLVE GUARD (this was the transition-flakiness bug). IPOPT does NOT raise on
        # Maximum_Iterations_Exceeded -- it RETURNS a non-converged iterate. The old code applied
        # that iterate (arbitrary thrust/tilt) AND seeded the next solve from it (self.zsol = zc),
        # so one bad solve during the hard transition poisoned every later solve -> tumble/stall.
        # Now: only ACCEPT + warm-start from a CONVERGED (Solve_Succeeded / *Acceptable*) result;
        # otherwise HOLD the last-good control and leave the warm-start untouched, so a bad solve
        # degrades gracefully instead of corrupting the state. (Mirrors nmpc_so3_transition_node.)
        Tf = None
        try:
            sol = self.solver(x0=self.zsol, lbx=self.lbx, ubx=self.ubx,
                              lbg=self.lbg, ubg=self.ubg, p=pdata)
            st = self.solver.stats().get("return_status", "")
            if ("Solve_Succeeded" in st) or ("Acceptable" in st):
                zc = np.array(sol['x']).flatten()
                Tf, Tr, beta, qa = zc[self.off:self.off + 4]
                theta_cmd = float(zc[self.nx * 1 + 4])          # planned body pitch one step ahead
                self.zsol = zc                                  # warm-start ONLY from a converged solve
                self._u_good = np.array([float(Tf), float(Tr), float(beta), float(qa)])
                self._theta_good = theta_cmd
            elif self.counter % 50 == 0:
                self.get_logger().warn(f"NMPC non-converged ({st}) -> holding last-good control")
        except Exception as e:
            if self.counter % 50 == 0:
                self.get_logger().warn(f"NMPC solve raised: {e} -> holding last-good control")
        if Tf is None:                                          # non-converged or raised -> hold last-good
            Tf, Tr, beta, qa = self._u_good
            theta_cmd = self._theta_good
        Tf = clamp(Tf, 0, self.TF_MAX); Tr = clamp(Tr, 0, self.TR_MAX); beta = clamp(beta, 0, self.BETA_MAX)
        # SLEW the APPLIED tilt (35 deg/s): the raw solver beta bang-banged 80<->0 near the
        # wing-borne boundary (fwD4). The slewed value also becomes the du anchor (uprev), so the
        # next solve is pulled back toward the applied tilt -- damping the flip at its source.
        bmax = math.radians(35.0) / self.rate
        beta = clamp(beta, self._beta_applied - bmax, self._beta_applied + bmax)
        self._beta_applied = beta
        theta_cmd = clamp(theta_cmd, -self.TH_DN, self.TH_UP)
        self._u_applied = np.array([Tf, Tr, beta, float(qa)])   # first-step du anchor next cycle

        # ---- GENTLE attitude / lateral loop (body level; roll for lateral; yaw slews to heading) ----
        # thrust tilt gives horizontal accel = -g*roll*ldir, so correcting e_lat needs roll = +(...)/g.
        # (a leading minus here is POSITIVE feedback -> lateral divergence; that was the fly-away bug.)
        e_lat = float((p - self.ref) @ ldir); v_lat = float(v @ ldir)
        roll_des = clamp((self.gp("kp_lat") * e_lat + self.gp("kv_lat") * v_lat) / self.G,
                         -math.radians(self.gp("max_roll_deg")), math.radians(self.gp("max_roll_deg")))
        rmax = math.radians(self.gp("roll_rate_deg_s")) / self.rate
        roll_des = clamp(roll_des, self.roll_prev - rmax, self.roll_prev + rmax); self.roll_prev = roll_des
        # PITCH is taken STRAIGHT from the NMPC plan: the solver CO-OPTIMIZED the body pitch (theta_cmd,
        # nose-up +) with tilt+thrust to hold altitude/speed while the wing lifts (positive AoA in cruise,
        # ~level at hover). No more separate pitch-for-speed law or elevator AoA-hold fighting the tilt.
        # Controller convention: pitch_des>0 = nose-DOWN, so a nose-up plan maps to pitch_des = -theta_cmd.
        e_fwd = float((p - self.ref) @ fdir)                   # >0 = past the target (overshoot)
        pitch_des = clamp(-theta_cmd, -self.TH_UP, self.TH_DN)
        # FORE-AFT POSITION-HOLD PD (in HOLD / final approach / landing). pitch_des>0 = nose-DOWN =
        # forward accel ~ g*pitch_des, so to drive (e_fwd, vs) -> 0 the demand is
        # pitch_des = -(kp*e_fwd + kv*vs)/g -- a signed PD identical in form to the lateral roll loop.
        # This is the FIX for the fore-aft runaway: the old brake used max(vs,0)/max(e_fwd,0), so it had
        # NO damping while sliding backward and could ONLY pitch nose-up, letting an overshoot slide
        # backward undamped (162->82 m). The signed PD arrests the overshoot AND the backward slide
        # (nose-DOWN forward force when drifting back), so the vehicle settles ON the target and can
        # then descend/land there. Gated on |vs|<8 (no hard nose-up at fixed-wing speed) and HOLD only,
        # so a far-target wing-borne CRUISE (hold=0 throughout) is untouched.
        if hold and abs(vs) < 8.0:
            fwd_pd = -(self.gp("kp_fwd_hold") * e_fwd + self.gp("kv_fwd_hold") * vs) / self.G
            bmx = math.radians(self.gp("brake_max_deg"))
            pitch_des = clamp(pitch_des + fwd_pd, -bmx, bmx)
        pmax = math.radians(self.gp("pitch_rate_deg_s")) / self.rate
        pitch_des = clamp(pitch_des, self.pitch_prev - pmax, self.pitch_prev + pmax); self.pitch_prev = pitch_des
        cy, sy, cr, sr2 = math.cos(self.yaw_cmd), math.sin(self.yaw_cmd), math.cos(roll_des), math.sin(roll_des)
        cp, sp = math.cos(pitch_des), math.sin(pitch_des)
        Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
        Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
        Rx = np.array([[1, 0, 0], [0, cr, -sr2], [0, sr2, cr]])
        R_des = Rz @ Ry @ Rx                             # roll=roll_des, pitch=pitch_des(hold only), yaw=yaw_cmd
        eR = 0.5 * vee(R_des.T @ R - R.T @ R_des); eW = w
        kR = np.array([self.gp("kR"), self.gp("kR"), self.gp("kR_yaw")])
        kW = np.array([self.gp("kW"), self.gp("kW"), self.gp("kW_yaw")])
        M = -kR * eR - kW * eW
        M[0] = clamp(M[0], -self.gp("m_roll_max"), self.gp("m_roll_max"))
        M[1] = clamp(M[1], -self.gp("m_pitch_max"), self.gp("m_pitch_max"))
        M[2] = clamp(M[2], -self.gp("m_yaw_max"), self.gp("m_yaw_max"))

        # ---- allocation: front pair Tf (tilt beta), rear pair Tr, + capped moment differentials.
        # (A "pitch-balanced" front/rear pre-split was tried and REVERTED: it fought the pitch-for-
        # forward command and stalled station-keeping. The attitude loop handles the CG pitch trim.)
        droll = M[0] / (4.0 * self.gp("y_arm"))
        dpitch = M[1] / (4.0 * 0.30)
        dyawm = M[2] / (4.0 * self.gp("cQ"))
        base = np.array([Tf / 2, Tf / 2, Tr / 2, Tr / 2])
        Tv = base + np.array([+droll - dpitch - dyawm * self.spin[0],
                              -droll - dpitch - dyawm * self.spin[1],
                              +droll + dpitch - dyawm * self.spin[2],
                              -droll + dpitch - dyawm * self.spin[3]])
        # NO tilt "boost": the NMPC's Tf is the front-pair thrust MAGNITUDE and the model's vertical
        # component is Tf*cos(beta) -- so the motor command IS Tf/kf. The old 1/cos(beta) boost injected
        # extra thrust the NMPC never modeled (plant > model when tilted -> a climb bias in cruise).
        T = np.clip(Tv, 0.0, self.kf)
        tservo = clamp(beta / (math.pi / 2), 0.0, 1.0)
        mot = [clamp(T[i] / self.kf, 0.0, 1.0) for i in range(4)]
        Vspd = float(np.linalg.norm(v))
        # surfaces are an AIRSPEED-GATED aero ASSIST to the same attitude target the NMPC planned
        # (elevator helps hold the nose-up pitch/AoA the solver chose; ail/rud fade in for roll/yaw).
        if self.gp("use_surfaces") > 0.5:
            sg = clamp((Vspd - self.gp("surf_v_gate")) / 4.0, 0.0, 1.0)
            # Elevator carries the SAME DAMPED pitch-moment demand as the differential thrust (M[1] =
            # -kR*eR - kW*eW, so it includes RATE damping). Using the position error eR[1] alone (no
            # damping) ran the pitch away; the damped moment gives nose-up authority WITHOUT the runaway.
            # k_elev<0: cm_elevator<0, so elev = -M[1]/(...) to produce the demanded moment.
            # + the AVTOL-profile ELEVATOR TRIM de(V) (already speed-gated): at wing-borne cruise the
            # rotors are unloaded, so the trim carries the static Cm balance and the damped term only
            # handles disturbances -- this is what makes airplane-mode pitch stable with idle rotors.
            de_ff = self.prof.by_speed(max(vs, 0.0))["de_cmd"] if use_prof else 0.0
            elevator = clamp(de_ff + self.gp("k_elev") * M[1] * sg, -1.0, 1.0)
            # Ailerons carry the damped ROLL moment M[0] (same idea as the elevator): at wing-borne
            # cruise the rotors are unloaded so differential-thrust roll authority is weak -> a slow roll
            # divergence tumbled it after ~50 s. Aero ailerons give roll authority when rotors are light.
            aileron = clamp(self.gp("k_ail") * M[0] * sg, -1.0, 1.0)
            rudder = clamp(self.gp("k_rud") * M[2] * sg, -1.0, 1.0)
        else:
            elevator = aileron = rudder = 0.0
        # optional: fade the rear (fixed) rotors out at cruise speed -- front tilt + wing carry the load
        if self.gp("rear_off_cruise") > 0.5:
            rf = clamp((Vspd - self.gp("aoa_v_gate")) / 6.0, 0.0, 1.0)
            mot[2] *= (1.0 - rf); mot[3] *= (1.0 - rf)
        self.pub(mot, [tservo, tservo, aileron, elevator, rudder])

        if self.counter % max(1, int(self.rate / 2)) == 0:
            self.get_logger().info(
                "p=(%.1f,%.1f,%.1f) yaw=%.0f dist=%.1f hold=%d | z->%.1f vs=%.1f Tf=%.0f Tr=%.0f "
                "tilt=%.0f bref=%.0f ws=%.0f%% roll=%.0f th_cmd=%.0f aoa=%.0f elev=%.2f "
                "mot=[%.2f,%.2f,%.2f,%.2f] arm=%s"
                % (p[0], p[1], p[2], math.degrees(cur_yaw), dist, int(hold), z_ref, vs, Tf, Tr,
                   math.degrees(beta), math.degrees(beta_refs[0]), 100.0 * ws_now,
                   math.degrees(roll_des), math.degrees(theta_cmd), math.degrees(alpha_meas),
                   elevator, mot[0], mot[1], mot[2], mot[3], self.armed))

        if self._active and self.auto_arm and self.counter >= self.arm_after and not (self.armed and self.offb):
            if self.counter - self.last >= 50:
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
        rclpy.spin(NMPCCruise())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
