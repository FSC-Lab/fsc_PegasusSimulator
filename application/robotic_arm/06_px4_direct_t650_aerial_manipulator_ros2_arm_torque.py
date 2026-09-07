#!/usr/bin/env python
"""
06_px4_direct_t650_aerial_manipulator_ros2_arm_torque.py

Author: Shiqi Gao (shiqi.gao907@gmail.com)

AM-T650 WHOLE-BODY direct-actuation plant: 05's rig with the arm in TORQUE
mode. The drone side is byte-for-byte 05's (AM_xfwd.usda, T650 motor model,
PX4-primary, /uav_0/state/* to the external stack). What changed is ONLY who
computes the arm torque:

    fsc_autopilot_ros2 whole-body direct-actuation node   (the coupled
        (autopilot_whole_body_direct_actuation_node,       airframe+arm law,
         impedance + GMO, 250 Hz)                          C++ port of
        |  joint torques [N*m]                             controller.py)
        v
    open_manipulator_x_custom_controller/ExternalTorqueController
        (clamp, position-limit pull-back, stale fallback, zero-on-deactivate)
        | effort command interfaces
        v
    open_manipulator_x_isaac_bridge/IsaacTopicEffortSystem
        -> /uav_0/isaacsim_manipulator/effort_commands  (sensor_msgs/JointState)
        <- /uav_0/isaacsim_manipulator/joint_states
    THIS process (physics callback, 250 Hz):
        FRESH stream  -> apply the commanded efforts (clipped +-TAU_MAX)
        STALE/absent  -> 05's PD + gravity-comp hold at the pose latched when
                         the stream died (spawn HOME until it ever starts) —
                         a torque stream must never latch (a stale torque on
                         an arm under a flying vehicle is not a hold, it is
                         an open-loop push), so staleness reverts to the
                         flight-validated servo-emulation law instead.

The stale fallback is the third and deepest safety layer (whole-body law's
own hold -> ExternalTorqueController's PD fallback -> this); it is what holds
the arm before the stacks come up and after they die.

Run with:
  scripts/indoor_sim/start_t650_aerial_manipulator_whole_body_direct_actuation_sitl.sh <config>
(starts this plant, the arm ros2_control TORQUE stack and the arm ground
station; pairs with fsc_autopilot_ros2's
start_whole_body_direct_actuation_t650_aerial_manipulator_stack.sh, started
FIRST — it owns MicroXRCEAgent).
"""

import os
import math

import carb
from isaacsim import SimulationApp

# Indoor-launcher env conventions (same as 03/04):
#   PEGASUS_HEADLESS=1       run without a window
#   PEGASUS_STEPS=N          stop after N physics steps (smoke tests)
#   PEGASUS_PX4_LOCKSTEP=0   disable lockstep — REQUIRED for the DIRECT
#                            wall-clock DDS path; the SITL launcher pushes it
#                            onto the tmux server (an export alone is discarded
#                            when a tmux server already exists).
HEADLESS = os.environ.get("PEGASUS_HEADLESS", "0") == "1"
STEP_LIMIT = int(os.environ.get("PEGASUS_STEPS", "0"))
PX4_LOCKSTEP = os.environ.get("PEGASUS_PX4_LOCKSTEP", "1") == "1"
EXPECTED_TOTAL_MASS = os.environ.get("PEGASUS_EXPECTED_TOTAL_MASS", "").strip()
# ARM SERVO MODEL (2026-09-03). "pwm" = the real Dynamixel PWM-mode servo:
# torque delivered falls by Kt^2/R per rad/s of joint speed (joints 2 and 3,
# the two the flights identify). "pwm_0903" adds the per-joint duty ceilings
# AS FLOWN on 2/3 Sep, which reproduces those bags but is NOT today's arm.
# "ideal" is the old behaviour, the commanded effort applied exactly.
# `or "pwm"`, not a get() default: the launcher bakes the variable into the
# Isaac pane unconditionally, so an unset knob arrives as the EMPTY STRING.
ARM_SERVO_MODEL = (os.environ.get("PEGASUS_ARM_SERVO_MODEL", "") or "pwm").strip().lower()
# Per-joint back-EMF coefficients "b1,b2,b3,b4" [N.m per rad/s], normally
# forwarded by the launcher out of the paired controller yaml's
# sim_arm_backemf_b_j* keys. Empty = servo_model.py's built-in Kt^2/R.
ARM_SERVO_B = (os.environ.get("PEGASUS_ARM_SERVO_B", "") or "").strip()
simulation_app = SimulationApp({"headless": HEADLESS})

# ── imports AFTER SimulationApp (numpy import-order rule) ────────────────────
import numpy as np
import omni.usd
import omni.timeline
from omni.isaac.core.world import World
from omni.isaac.dynamic_control import _dynamic_control
from pxr import Usd, UsdPhysics, PhysxSchema, Gf

from scipy.spatial.transform import Rotation

from pegasus.simulator.params import SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.logic.backends.px4_mavlink_backend import (
    PX4MavlinkBackend, PX4MavlinkBackendConfig)
from pegasus.simulator.logic.backends.ros2_backend import ROS2Backend

from fsc_aerial_manipulation.utils import (add_dome_lighting, author_inertia_tensor,
                                           read_inertia_tensor)
from fsc_aerial_manipulation.rotorcraft.x650_rotorcraft_utils import (
    print_mass_inertia_properties,
    print_rotor_positions,
)
from fsc_aerial_manipulation.rotorcraft import t650_params
from fsc_aerial_manipulation.rotorcraft.lagged_thrust_curve import LaggedQuadraticThrustCurve
from fsc_aerial_manipulation.robotic_arm.utils_vehicle.x650_multirotor import (
    MultirotorMod, MultirotorConfig)
from fsc_aerial_manipulation.robotic_arm.utils_controller import controller as C
from fsc_aerial_manipulation.robotic_arm.servo_model import (
    DynamixelPwmServo, TAU_CAP_AS_FLOWN)

# ╔══════════════════════════════════════════════════════════════════════════╗
# ║  CONFIG (identical to 04 unless marked ARM-ROS2)                         ║
# ╚══════════════════════════════════════════════════════════════════════════╝

import fsc_aerial_manipulation.rotorcraft as _fsc_rotorcraft
ASSETS_DIR = os.path.join(os.path.dirname(_fsc_rotorcraft.__file__), "assets")
USD_FILE   = os.path.join(ASSETS_DIR, "AM_xfwd.usda")
USD_PRIM_PATH     = "/gripper_bat"
BODY_PATH         = "/body"
# CHANNEL REMAP, load-bearing — see 04's header comment.
ROTOR_PATHS       = ["/rotor0", "/rotor1", "/rotor3", "/rotor2"]
ROTOR_JOINT_NAMES = ["joint0", "joint1", "joint3", "joint2"]
ARM_JOINT_NAMES   = ["manip_joint1", "manip_joint2", "manip_joint3", "manip_joint4"]
GRIPPER_JOINT     = "gripper_joint"
GRIPPER_REST_DEG  = 0.0

SPAWN_POS   = (0.0, 0.0, 0.5)
SPAWN_EULER = (0.0, 0.0, 0.0)
VEHICLE_ID  = 0
GROUND_BODY_Z = 0.305

# Spawn pose = the aerial home every fsc_open_manipulator aerial config uses
# ([0, 40, 40, 0] deg). The PositionController's activation homing is then a
# no-op, and until ROS 2 commands flow this process holds exactly like 04.
Q_HOME = np.radians([0.0, 40.0, 40.0, 0.0])

# ── ARM-ROS2: the SIMULATED SERVO BUS (must match the IsaacTopicSystem params
# in open_manipulator_x_isaac_bridge/urdf/open_manipulator_x_isaac_aerial.urdf).
# These two topics stand in for the Dynamixel/U2D2 serial link and exist ONLY
# in simulation — hence the `isaacsim_manipulator` prefix, which marks
# everything the SIMULATOR owns. On real hardware the bus is the wire and
# nothing under it is published. Everything the REAL ARM STACK owns lives under
# `/uav_0/fsc_open_manipulator/...` (joint_states from joint_state_broadcaster,
# the controller's own topics) and is identical in sim and on hardware; that is
# the whole point of the split, so do NOT let an application subscribe here.
# States/commands are matched BY NAME using the controller-side joint names,
# so neither side depends on the other's ordering.
# NOTE: `effort` below is the applied joint torque in N*m. The hardware
# backend reports raw Dynamixel counts on the same field — see
# docs/docs_aerial_manipulator/Arm Topic Naming.md.
ARM_ROS_JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4"]
ARM_STATE_TOPIC     = "/uav_0/isaacsim_manipulator/joint_states"
# The name Arm Topic Naming.md reserved for torque mode — written ONLY by
# IsaacTopicEffortSystem, read ONLY by this plant.
ARM_EFFORT_TOPIC    = "/uav_0/isaacsim_manipulator/effort_commands"

# --- PLANNED-TRAJECTORY VISUALISATION (2026-09-04, user request) -----------
# BLUE  = the drone reference path (the system CoM the law tracks) + an arrow
#         along the vehicle's NOSE.
# RED   = the end-effector reference path + an arrow along the GRIPPER AXIS,
#         where the claw actually aims (tilts ~10 deg down at the home pose).
# Both arrows come from the whole-body planner already converted. It does NOT send the
# law's own b1_d/b1_de: those are MODEL-frame x-axes and the model frame is the
# actual one yawed by -90, so drawing them raw put the drone's heading arrow
# 90 deg off its nose (2026-09-04). The conversion belongs on the whole-body planner's
# side, where the frame boundary already lives.
# The data is the whole-body planner's own reference, already in the WORLD frame and
# already sampled, arriving as a plain Float64MultiArray (12 doubles per
# sample: x_cd, nose, r_ed, claw). Deliberately not WholeBodyReference: this
# process has core message packages only, and re-deriving the model<->actual
# frame conversion here would put a second copy of the whole-body planner's frame
# boundary somewhere with no business owning one.
TRAJ_VIZ            = os.environ.get("PEGASUS_TRAJ_VIZ", "1") == "1"
VIZ_PATH_TOPIC      = "/uav_0/whole_body_planner/viz_path"
VIZ_POSE_TOPIC      = "/uav_0/whole_body_planner/viz_pose"
VIZ_BLUE            = (0.20, 0.55, 1.00, 1.0)
VIZ_RED             = (1.00, 0.25, 0.25, 1.0)
VIZ_CURVE_WIDTH     = 3.0
VIZ_ARROW_WIDTH     = 6.0
VIZ_ARROW_DRONE     = 0.45      # m, heading arrow length on the vehicle
VIZ_ARROW_EE        = 0.22      # m, shorter so it reads as the smaller body
VIZ_REDRAW_STEPS    = 8         # debug_draw is cleared and rebuilt each time
# External-torque freshness: stale after this sim-time age (matches the
# ExternalTorqueController's command_timeout_s), re-engage only after this
# many consecutive fresh commands (hysteresis — no chattering at the
# boundary between the torque stream and the PD hold).
CMD_FRESH_S   = 0.3
REARM_FRESH_N = 3

# ── ROBUSTNESS INJECTION: PLANT-SIDE MODEL UNCERTAINTY (2026-09-05) ──────────
# The controller's model is the shipped one; the PLANT is perturbed, which is
# what "model uncertainty" means physically and keeps the controller config
# honest. Set from section 1 of the whole-body sim yaml and forwarded by the
# launcher. 1.0 / 0.0 = nominal, i.e. every existing rig is bit-identical.
def _envf(name, default):
    v = (os.environ.get(name) or "").strip()
    if not v:
        return default
    try:
        return float(v)
    except ValueError:
        raise SystemExit(f"[AM-T650-WB] {name} must be a number, got '{v}'")


PLANT_MASS_SCALE    = _envf("PEGASUS_PLANT_MASS_SCALE", 1.0)
PLANT_INERTIA_SCALE = _envf("PEGASUS_PLANT_INERTIA_SCALE", 1.0)
PLANT_COM_SHIFT = np.array([_envf("PEGASUS_PLANT_COM_SHIFT_X", 0.0),
                            _envf("PEGASUS_PLANT_COM_SHIFT_Y", 0.0),
                            _envf("PEGASUS_PLANT_COM_SHIFT_Z", 0.0)], float)

T650_BODY_MASS    = float(t650_params.BODY_MASS)
# Full 3x3 tensor, not the diagonal: it may carry products of inertia, which USD can only
# store as diagonalInertia + principalAxes (see utils.author_inertia_tensor).
T650_BODY_INERTIA = np.asarray(t650_params.INERTIA_TENSOR, float)

# ── STALE-FALLBACK law: 04's flight-validated hold (PD + gravity comp) ──────
ARM_HOLD_KP   = 3.0    # [N·m/rad]
ARM_HOLD_KD   = 0.25   # [N·m·s/rad]
ARM_HOLD_RATE = 0.5    # [rad/s] reference slew — transparent to the position
                       # controller's <= 0.2 rad/s min-jerk moves; caps the step
                       # if something publishes a raw far-away target directly
TAU_MAX       = 3.0    # [N·m] — one number with the wb yaml and the
                       # ExternalTorqueController; see servo_model.py for why
                       # PEGASUS_ARM_SERVO_MODEL=pwm_0903 deliberately differs
ARM_ARMATURE  = 353.5 ** 2 * 1.6e-7

# ── ARM SERVO: the plant between the commanded torque and the applied one ───
# Isaac applies a commanded effort exactly; the real XM430s in PWM mode do not.
# See servo_model.py for the derivation and the flight identification. `ideal`
# keeps the old behaviour for an A/B.
_B_OVERRIDE = None
if ARM_SERVO_B:
    try:
        _B_OVERRIDE = np.array([float(v) for v in ARM_SERVO_B.split(",")], float)
    except ValueError:
        _B_OVERRIDE = np.empty(0)
    if _B_OVERRIDE.shape != (4,) or not np.all(np.isfinite(_B_OVERRIDE)):
        raise SystemExit(f"[AM-T650-WB] PEGASUS_ARM_SERVO_B must be four finite "
                         f"numbers 'b1,b2,b3,b4' (got {ARM_SERVO_B!r})")

if ARM_SERVO_MODEL == "ideal":
    ARM_SERVO = DynamixelPwmServo(tau_cap=np.full(4, TAU_MAX), backemf_joints=())
elif ARM_SERVO_MODEL == "pwm":
    ARM_SERVO = DynamixelPwmServo(tau_cap=np.full(4, TAU_MAX), b=_B_OVERRIDE)
elif ARM_SERVO_MODEL == "pwm_0903":
    ARM_SERVO = DynamixelPwmServo(tau_cap=TAU_CAP_AS_FLOWN, b=_B_OVERRIDE)
else:
    raise SystemExit(f"[AM-T650-WB] PEGASUS_ARM_SERVO_MODEL must be one of "
                     f"'pwm', 'pwm_0903', 'ideal' (got {ARM_SERVO_MODEL!r})")

R_MODEL = np.array([[0.0, 1.0, 0.0],
                    [-1.0, 0.0, 0.0],
                    [0.0, 0.0, 1.0]])   # frame adapter, see 04

STATUS_PERIOD_S = 5.0


def _rot_to_quat_wxyz(R):
    """Rotation matrix → quaternion (w, x, y, z). Shepperd's method (03's)."""
    tr = np.trace(R)
    if tr > 0.0:
        s = 2.0 * np.sqrt(tr + 1.0)
        q = [0.25 * s, (R[2, 1] - R[1, 2]) / s,
             (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s]
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        q = [(R[2, 1] - R[1, 2]) / s, 0.25 * s,
             (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s]
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        q = [(R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s,
             0.25 * s, (R[1, 2] + R[2, 1]) / s]
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        q = [(R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s,
             (R[1, 2] + R[2, 1]) / s, 0.25 * s]
    q = np.asarray(q, float)
    return q / np.linalg.norm(q)


class AmT650WholeBodyArmSim:
    """05's plant with the arm TORQUE owned by the external whole-body stack."""

    def __init__(self):
        self.timeline = omni.timeline.get_timeline_interface()

        # --- Pegasus world + environment ------------------------------------
        self.pg        = PegasusInterface()
        self.pg._world = World(**self.pg._world_settings)
        self.world     = self.pg.world
        self.pg.load_environment(SIMULATION_ENVIRONMENTS["Curved Gridroom"])
        self._raise_physx_gpu_capacity()

        stage = omni.usd.get_context().get_stage()
        add_dome_lighting(stage=stage, dome_path="/World/DomeLight",
                          intensity=2500.0, exposure=0.0, color=(1.0, 1.0, 1.0))

        # --- Spawn (PX4-PRIMARY) --------------------------------------------
        self.drone_path = self._spawn_am_px4_primary()

        self._wait_for_prim(self.drone_path, max_frames=300)
        self._dedupe_physics_scenes()
        self._disable_self_collisions()
        self._disable_rotor_colliders()
        self._apply_t650_body_override()
        self._setup_gripper_drive()
        self.world.reset()
        self.stage = omni.usd.get_context().get_stage()

        print_mass_inertia_properties(self.stage, self.drone_path)
        print_rotor_positions(self.stage, self.drone_path, ROTOR_PATHS)

        # --- DC (body pose/twist) + core Articulation API (arm) -------------
        self._dc   = _dynamic_control.acquire_dynamic_control_interface()
        self._body = self._dc.get_rigid_body(self.drone_path + BODY_PATH)
        from omni.isaac.core.articulations import Articulation
        self._art = Articulation(prim_path=self.drone_path)
        self._art.initialize()
        names = list(self._art.dof_names)
        self._arm_idx  = [names.index(nm) for nm in ARM_JOINT_NAMES]
        self._grip_idx = names.index(GRIPPER_JOINT) if GRIPPER_JOINT in names else None
        print(f"[AM-T650-WB] core Articulation API active; arm dof indices "
              f"{self._arm_idx}, gripper dof {self._grip_idx}", flush=True)

        # --- Control model (gravity comp only) ------------------------------
        self.params = C.make_params()
        if getattr(self, "_body_dm", 0.0):
            self.params["m_i"][0] += self._body_dm
            if self._body_dI is not None:
                # _body_dI is already a 3x3 tensor delta — do NOT np.diag() it (on a 2-D
                # input that EXTRACTS the diagonal instead of building a matrix).
                self.params["I_i_i"][0] = self.params["I_i_i"][0] + self._body_dI
            print(f"[AM-T650-WB] control model mirrored: m0="
                  f"{self.params['m_i'][0]:.6f} kg, m_total="
                  f"{sum(self.params['m_i']):.6f} kg (hover thrust "
                  f"{sum(self.params['m_i']) * self.params['g']:.2f} N)", flush=True)

        # --- Arm spawn at HOME + rigid re-seat + ground seat (04's sequence) -
        beta_spawn = float(Q_HOME[1] + Q_HOME[2])
        if beta_spawn < math.radians(40.0):
            print(f"[AM-T650-WB] WARNING: spawn fold beta = "
                  f"{math.degrees(beta_spawn):.1f} deg is SHALLOW — a "
                  f"ground-seated vehicle with an unfolded arm rests on its "
                  f"gripper and tips over.", flush=True)
        pose_a = self._dc.get_rigid_body_pose(self._body)
        self._art.set_joint_positions(Q_HOME, joint_indices=self._arm_idx)
        self._art.set_joint_velocities(np.zeros(len(self._arm_idx)),
                                       joint_indices=self._arm_idx)
        pose_b = self._dc.get_rigid_body_pose(self._body)
        Ra = C.quat_to_rot(pose_a.r.w, pose_a.r.x, pose_a.r.y, pose_a.r.z)
        Rb = C.quat_to_rot(pose_b.r.w, pose_b.r.x, pose_b.r.y, pose_b.r.z)
        pa = np.array([pose_a.p.x, pose_a.p.y, pose_a.p.z])
        pb = np.array([pose_b.p.x, pose_b.p.y, pose_b.p.z])
        R_corr = Ra @ Rb.T
        p_corr = pa - R_corr @ pb
        pr, qr = self._art.get_world_pose()
        self._art.set_world_pose(
            position=R_corr @ np.asarray(pr, float) + p_corr,
            orientation=_rot_to_quat_wxyz(R_corr @ C.quat_to_rot(*np.asarray(qr, float))))
        print(f"[AM-T650-WB] arm spawned at HOME q = "
              f"{np.degrees(Q_HOME).round(1)} deg (body re-seated: teleport "
              f"moved it {np.linalg.norm(pb - pa):.3f} m)", flush=True)

        pose_g = self._dc.get_rigid_body_pose(self._body)
        dz = GROUND_BODY_Z - float(pose_g.p.z)
        pr, qr = self._art.get_world_pose()
        self._art.set_world_pose(
            position=np.asarray(pr, float) + np.array([0.0, 0.0, dz]))
        print(f"[AM-T650-WB] ground-seated: body z {pose_g.p.z:.3f} -> "
              f"{GROUND_BODY_Z:.3f} m (dz={dz:+.3f})", flush=True)

        # --- Arm actuation: TRUE effort control from the start ---------------
        self._set_arm_armature()
        actrl = self._art.get_articulation_controller()
        for i in self._arm_idx:
            actrl.switch_dof_control_mode(dof_index=i, mode="effort")
        print(f"[AM-T650-WB] arm servo model '{ARM_SERVO_MODEL}'"
              f"{' (b from the controller yaml)' if ARM_SERVO_B else ''}: "
              f"back-EMF droop {np.round(ARM_SERVO.b, 3).tolist()} N·m/(rad/s), "
              f"ceiling {np.round(ARM_SERVO.tau_cap, 3).tolist()} N·m "
              f"(b·dt/I = "
              f"{np.round(ARM_SERVO.explicit_stability_ratio(np.full(4, ARM_ARMATURE), 1.0 / 250.0), 2).tolist()}"
              f", explicit damping is stable below 2)", flush=True)
        if ARM_SERVO_MODEL == "pwm_0903":
            print("[AM-T650-WB] WARNING: 'pwm_0903' uses the 2/3 Sep duty "
                  "ceilings, NOT the uniform 3.0 N·m the arm has carried since "
                  "2026-09-04 — the plant and the controllers disagree ON "
                  "PURPOSE. Use 'pwm' for anything but replaying those bags.",
                  flush=True)
        print("[AM-T650-WB] arm dofs in effort mode; EXTERNAL TORQUE "
              f"(clip ±{TAU_MAX} N·m, fresh < {CMD_FRESH_S}s) with PD+gravity "
              f"fallback KP={ARM_HOLD_KP} KD={ARM_HOLD_KD} on a stale stream",
              flush=True)

        # --- ARM-ROS2: bridge node (position commands in, joint states out) --
        self._setup_arm_ros2_bridge()

        self._hold_ref = None            # slewed fallback reference
        self._q_hold_target = Q_HOME.copy()  # fallback pose (latched at stale edge)
        self._tau_cmd = np.zeros(len(self._arm_idx))  # latest external torque
        self._cmd_stamp_t = None         # sim time the last torque arrived
        self._n_cmds = 0
        self._ext_active = False         # torque stream has authority
        self._fresh_count = 0            # consecutive fresh commands (re-arm)
        self._tau_applied = np.zeros(len(self._arm_idx))
        self._tau_cmd_applied = np.zeros(len(self._arm_idx))
        self._t = 0.0
        self._status_t = -1e9
        self.stop_sim = False
        carb.log_info("[AM-T650-WB] ready")

    # ── ARM-ROS2 bridge ─────────────────────────────────────────────────────

    def _setup_arm_ros2_bridge(self):
        """rclpy pub/sub for the arm — the Isaac side of IsaacTopicSystem.

        The Pegasus ROS2Backend already initialised rclpy (its own node
        publishes /uav_0/state/*); this node is separate and uniquely named,
        per the one-node-per-instance rule.
        """
        import rclpy
        from rclpy.node import Node as RclpyNode
        from sensor_msgs.msg import JointState

        try:
            rclpy.init()
        except Exception:
            pass  # already initialised is not an error

        self._JointState = JointState
        self._rclpy = rclpy
        self._arm_node = RclpyNode(f"isaac_arm_torque_plant_{VEHICLE_ID}")
        self._arm_state_pub = self._arm_node.create_publisher(
            JointState, ARM_STATE_TOPIC, 10)
        self._arm_eff_sub = self._arm_node.create_subscription(
            JointState, ARM_EFFORT_TOPIC, self._on_arm_effort, 10)

        # --- planned-trajectory visualisation -------------------------------
        self._viz_path = None       # (N, 12) world-frame samples of the plan
        self._viz_pose = None       # (12,)  the current reference sample
        self._draw = None
        self._draw_ok = False
        if TRAJ_VIZ and not HEADLESS:
            from std_msgs.msg import Float64MultiArray
            from rclpy.qos import (DurabilityPolicy, QoSProfile,
                                   ReliabilityPolicy)
            # The path is LATCHED on the whole-body planner's side, so match it or a
            # plan made before Isaac subscribed never arrives.
            latched = QoSProfile(depth=1,
                                 reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self._viz_path_sub = self._arm_node.create_subscription(
                Float64MultiArray, VIZ_PATH_TOPIC, self._on_viz_path, latched)
            self._viz_pose_sub = self._arm_node.create_subscription(
                Float64MultiArray, VIZ_POSE_TOPIC, self._on_viz_pose, 10)
            self._acquire_debug_draw()

    def _acquire_debug_draw(self):
        """debug_draw moved namespaces across Isaac versions; try both and
        enable the extension first in case it is present but not loaded.
        Degrades to no drawing rather than taking the sim down with it."""
        import importlib
        try:
            from omni.isaac.core.utils.extensions import enable_extension
        except Exception:
            enable_extension = None
        last = None
        for ext in ("omni.isaac.debug_draw", "isaacsim.util.debug_draw"):
            try:
                if enable_extension is not None:
                    try:
                        enable_extension(ext)
                    except Exception:
                        pass
                self._draw = importlib.import_module(
                    ext + "._debug_draw").acquire_debug_draw_interface()
                print(f"[AM-T650-WB] trajectory viz: {ext} acquired "
                      f"(BLUE = drone, RED = end-effector)", flush=True)
                return
            except Exception as exc:
                last = exc
        print(f"[AM-T650-WB] trajectory viz: no debug_draw module — "
              f"curves off ({last})", flush=True)

    def _on_viz_path(self, msg):
        d = np.asarray(msg.data, float)
        self._viz_path = (d.reshape(-1, 12)
                          if d.size >= 12 and d.size % 12 == 0 else None)

    def _on_viz_pose(self, msg):
        d = np.asarray(msg.data, float)
        self._viz_pose = d if d.size == 12 else None

    @staticmethod
    def _arrow_lines(p, direction, length, starts, ends):
        """Shaft + two head strokes for one arrow."""
        d = np.asarray(direction, float)
        n = float(np.linalg.norm(d))
        if not np.isfinite(n) or n < 1e-9:
            return
        u = d / n
        p = np.asarray(p, float)
        tip = p + length * u
        # any axis not parallel to u, so the head has a plane to open in
        ref = np.array([0.0, 0.0, 1.0]) if abs(u[2]) < 0.9 else \
            np.array([1.0, 0.0, 0.0])
        perp = np.cross(u, ref)
        perp /= (np.linalg.norm(perp) + 1e-12)
        back = tip - 0.30 * length * u
        starts.append(tuple(p))
        ends.append(tuple(tip))
        for side in (+1.0, -1.0):
            starts.append(tuple(tip))
            ends.append(tuple(back + side * 0.15 * length * perp))

    def _draw_trajectory(self):
        """Redraw both curves and both heading arrows. debug_draw is cleared
        and rebuilt wholesale, so everything on screen is drawn here."""
        if self._draw is None:
            return
        starts, ends, colors, widths = [], [], [], []
        path = self._viz_path
        if path is not None and len(path) > 1:
            for col, o in ((VIZ_BLUE, 0), (VIZ_RED, 6)):
                pts = path[:, o:o + 3]
                starts.extend(tuple(v) for v in pts[:-1])
                ends.extend(tuple(v) for v in pts[1:])
                colors.extend([col] * (len(pts) - 1))
                widths.extend([VIZ_CURVE_WIDTH] * (len(pts) - 1))
        pose = self._viz_pose
        if pose is not None:
            for col, o, ln in ((VIZ_BLUE, 0, VIZ_ARROW_DRONE),
                               (VIZ_RED, 6, VIZ_ARROW_EE)):
                n0 = len(starts)
                self._arrow_lines(pose[o:o + 3], pose[o + 3:o + 6], ln,
                                  starts, ends)
                added = len(starts) - n0
                colors.extend([col] * added)
                widths.extend([VIZ_ARROW_WIDTH] * added)
        try:
            self._draw.clear_lines()
            if starts:
                self._draw.draw_lines(starts, ends, colors, widths)
                if not self._draw_ok:
                    self._draw_ok = True
                    print(f"[AM-T650-WB] trajectory viz: first draw "
                          f"({len(starts)} segments)", flush=True)
        except Exception as exc:
            print(f"[AM-T650-WB] trajectory viz disabled: {exc}", flush=True)
            self._draw = None
        print(f"[AM-T650-WB] arm ROS2 bridge up: states -> {ARM_STATE_TOPIC}, "
              f"efforts <- {ARM_EFFORT_TOPIC} (names {ARM_ROS_JOINT_NAMES})",
              flush=True)

    def _on_arm_effort(self, msg):
        # Match BY NAME; reject the WHOLE message if any joint is missing or
        # non-finite (same rule as ExternalTorqueController's own subscriber:
        # a half-valid torque vector must never reach the arm, and rejecting
        # keeps the staleness clock honest).
        tau = np.zeros(len(ARM_ROS_JOINT_NAMES))
        names = list(msg.name)
        for j, nm in enumerate(ARM_ROS_JOINT_NAMES):
            try:
                k = names.index(nm)
            except ValueError:
                return
            if k >= len(msg.effort) or not math.isfinite(msg.effort[k]):
                return
            tau[j] = float(msg.effort[k])
        self._tau_cmd = tau
        self._cmd_stamp_t = self._t
        self._n_cmds += 1

    def _publish_arm_state(self, q, qdot, tau):
        msg = self._JointState()
        msg.header.stamp = self._arm_node.get_clock().now().to_msg()
        msg.name = list(ARM_ROS_JOINT_NAMES)
        msg.position = [float(v) for v in q]
        msg.velocity = [float(v) for v in qdot]
        msg.effort = [float(v) for v in tau]
        self._arm_state_pub.publish(msg)

    # ── spawn (04's, unchanged) ─────────────────────────────────────────────

    def _spawn_am_px4_primary(self):
        """Spawn AM_xfwd with PX4 primary and the T650 motor calibration."""
        quat_xyzw = Rotation.from_euler("XYZ", SPAWN_EULER, degrees=True).as_quat()

        input_scaling = float(t650_params.MAX_ROTOR_VEL - t650_params.ZERO_POSITION_ARMED)
        mavlink_config = PX4MavlinkBackendConfig({
            "vehicle_id": VEHICLE_ID,
            "connection_type": "tcpin",
            "connection_ip": "127.0.0.1",
            "connection_baseport": 4560,
            "enable_lockstep": PX4_LOCKSTEP,
            "px4_autolaunch": False,
            "px4_dir": self.pg.px4_path,
            "px4_vehicle_model": self.pg.px4_default_airframe,
            "input_offset": [0.0] * 4,
            "input_scaling": [input_scaling] * 4,
            "zero_position_armed": [float(t650_params.ZERO_POSITION_ARMED)] * 4,
        })

        ros2_backend = ROS2Backend(
            vehicle_id=VEHICLE_ID,
            config={
                "namespace": "uav_",
                "pub_sensors": False,
                "pub_graphical_sensors": False,
                "pub_state": True,
                "pub_twist": True,
                "pub_accel": True,
                "pub_twist_inertial": True,
                "pub_tf": True,
                "sub_control": False,
            },
        )

        config = MultirotorConfig()
        config.backends = [PX4MavlinkBackend(mavlink_config), ros2_backend]
        self._px4_backend = config.backends[0]
        config.thrust_curve = LaggedQuadraticThrustCurve(config={
            "rotor_constant": [float(t650_params.ROTOR_CONSTANT)] * 4,
            "rolling_moment_coefficient":
                [float(t650_params.ROLLING_MOMENT_COEFFICIENT)] * 4,
            "min_rotor_velocity": [float(t650_params.MIN_ROTOR_VEL)] * 4,
            "max_rotor_velocity": [float(t650_params.MAX_ROTOR_VEL)] * 4,
            "rot_dir": [int(d) for d in t650_params.ROT_DIR],
            "rotor_lambda": [float(t650_params.ROTOR_LAMBDA)] * 4,
        })
        self._thrust_curve = config.thrust_curve

        drone_prim_path = f"/World/quadrotor_{VEHICLE_ID}"
        MultirotorMod(
            stage_prefix=drone_prim_path,
            usd_file=USD_FILE,
            vehicle_id=VEHICLE_ID,
            init_pos=SPAWN_POS,
            init_orientation=quat_xyzw,
            body_path=BODY_PATH,
            rotor_paths=ROTOR_PATHS,
            rotor_joint_names=ROTOR_JOINT_NAMES,
            config=config,
            usd_prim_path=USD_PRIM_PATH,
        )
        print(f"[AM-T650-WB] spawned PX4-PRIMARY AM at {drone_prim_path}: MN4010 "
              f"k_f={t650_params.ROTOR_CONSTANT:.4e} "
              f"k_m={t650_params.ROLLING_MOMENT_COEFFICIENT:.4e} "
              f"lambda={t650_params.ROTOR_LAMBDA} "
              f"omega=[{t650_params.MIN_ROTOR_VEL}, {t650_params.MAX_ROTOR_VEL}] "
              f"map u->omega: x{input_scaling:.4f} "
              f"+{t650_params.ZERO_POSITION_ARMED} | lockstep="
              f"{'ON' if PX4_LOCKSTEP else 'OFF'}", flush=True)
        return drone_prim_path

    # ── physics/model fixes (04's, unchanged) ───────────────────────────────

    def _raise_physx_gpu_capacity(self):
        stage = omni.usd.get_context().get_stage()
        scene = next((p for p in stage.Traverse() if p.IsA(UsdPhysics.Scene)), None)
        if scene is None:
            return
        api = PhysxSchema.PhysxSceneAPI.Apply(scene)
        api.CreateGpuFoundLostAggregatePairsCapacityAttr().Set(256 * 1024)
        api.CreateGpuTotalAggregatePairsCapacityAttr().Set(256 * 1024)
        api.CreateGpuFoundLostPairsCapacityAttr().Set(1024 * 1024)
        api.CreateGpuMaxRigidContactCountAttr().Set(2 * 1024 * 1024)
        api.CreateGpuMaxRigidPatchCountAttr().Set(256 * 1024)

    def _wait_for_prim(self, prim_path, max_frames=300):
        stage = omni.usd.get_context().get_stage()
        for _ in range(max_frames):
            p = stage.GetPrimAtPath(prim_path)
            if p and p.IsValid():
                return p
            self.world.step(render=False)
        carb.log_error(f"[AM-T650-WB] prim never appeared: {prim_path}")
        return None

    def _dedupe_physics_scenes(self):
        stage = omni.usd.get_context().get_stage()
        scenes = [p for p in stage.Traverse() if p.IsA(UsdPhysics.Scene)]
        if len(scenes) <= 1:
            return
        keep = next((p for p in scenes
                     if not p.GetPath().pathString.startswith(self.drone_path)), scenes[0])
        for p in scenes:
            if p.GetPath() == keep.GetPath():
                continue
            path = p.GetPath()
            stage.RemovePrim(path)
            still = stage.GetPrimAtPath(path)
            if still and still.IsValid():
                still.SetActive(False)
        print(f"[AM-T650-WB] kept one PhysicsScene: {keep.GetPath()}", flush=True)

    def _disable_self_collisions(self):
        stage = omni.usd.get_context().get_stage()
        root = stage.GetPrimAtPath(self.drone_path)
        target = next((pr for pr in Usd.PrimRange(root)
                       if pr.HasAPI(UsdPhysics.ArticulationRootAPI)), root)
        PhysxSchema.PhysxArticulationAPI.Apply(target).CreateEnabledSelfCollisionsAttr().Set(False)
        print(f"[AM-T650-WB] self-collision disabled on {target.GetPath()}", flush=True)

    def _disable_rotor_colliders(self):
        stage = omni.usd.get_context().get_stage()
        n = 0
        for rp in ROTOR_PATHS:
            root = stage.GetPrimAtPath(self.drone_path + rp)
            if not root or not root.IsValid():
                continue
            for prim in Usd.PrimRange(root):
                if prim.HasAPI(UsdPhysics.CollisionAPI):
                    UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr().Set(False)
                    n += 1
        print(f"[AM-T650-WB] disabled {n} rotor collider(s).", flush=True)

    def _apply_t650_body_override(self):
        """T650 mass + inertia onto /body, live stage — see 04's docstring."""
        self._body_dm = 0.0
        self._body_dI = None

        stage = omni.usd.get_context().get_stage()
        body_path = self.drone_path + BODY_PATH
        prim = stage.GetPrimAtPath(body_path)
        if not prim or not prim.IsValid():
            print(f"[AM-T650-WB] WARNING: body prim {body_path} not found — "
                  f"T650 override SKIPPED", flush=True)
            return

        m_rest = 0.0
        for p in Usd.PrimRange(stage.GetPrimAtPath(self.drone_path)):
            if p.GetPath() == prim.GetPath() or not p.HasAPI(UsdPhysics.RigidBodyAPI):
                continue
            a = UsdPhysics.MassAPI(p).GetMassAttr()
            if a and a.HasValue():
                m_rest += float(a.Get())

        mass_api = UsdPhysics.MassAPI.Apply(prim)
        m_old = float(mass_api.GetMassAttr().Get())
        I_attr = mass_api.GetDiagonalInertiaAttr()
        I_old = (np.array(I_attr.Get(), float)
                 if I_attr and I_attr.HasValue() else None)

        # ROBUSTNESS INJECTION. The PLANT is perturbed; the controller's model is
        # not. self._body_dm / _body_dI are deliberately computed from the
        # NOMINAL values, so 06's own in-process model (the arm-hold gravity
        # comp) keeps believing the nominal plant — the mismatch is the point.
        # PLANT_MASS_SCALE scales the VEHICLE TOTAL, not the body: the arm and
        # rotor links keep their authored masses, so the body carries the whole
        # delta. Scaling the body directly would have meant only +7.9% total at
        # a nominal +10%, and the report would have said 10% while the plant
        # flew 7.9.
        total_nominal = T650_BODY_MASS + m_rest
        mass_plant = total_nominal * PLANT_MASS_SCALE - m_rest
        inertia_plant = T650_BODY_INERTIA * PLANT_INERTIA_SCALE

        mass_api.CreateMassAttr().Set(mass_plant)
        self._body_dm = T650_BODY_MASS - m_old
        # Authors diagonalInertia + principalAxes, the only representation USD/PhysX has —
        # a straight diagonalInertia write would silently drop the Ixy product.
        moments, quat = author_inertia_tensor(mass_api, inertia_plant)
        if np.any(PLANT_COM_SHIFT):
            com_attr = mass_api.CreateCenterOfMassAttr()
            com_old = np.array(com_attr.Get(), float) if com_attr.HasValue() \
                else np.zeros(3)
            com_new = com_old + PLANT_COM_SHIFT
            com_attr.Set(Gf.Vec3f(*com_new.astype(float)))
            print(f"[AM-T650-WB] INJECTION body CoM {com_old.round(5)} -> "
                  f"{com_new.round(5)} m (shift {PLANT_COM_SHIFT.round(5)})",
                  flush=True)
        if PLANT_MASS_SCALE != 1.0 or PLANT_INERTIA_SCALE != 1.0:
            print(f"[AM-T650-WB] INJECTION vehicle TOTAL mass x{PLANT_MASS_SCALE:.4f} "
                  f"({total_nominal:.6f} -> {total_nominal * PLANT_MASS_SCALE:.6f} kg; "
                  f"body {T650_BODY_MASS:.6f} -> {mass_plant:.6f}), body inertia "
                  f"x{PLANT_INERTIA_SCALE:.4f}. The controller still believes "
                  f"the nominal model.", flush=True)
        if I_old is not None:
            self._body_dI = T650_BODY_INERTIA - np.diag(I_old)
            print(f"[AM-T650-WB] body inertia diag {I_old.round(6)} -> "
                  f"{np.diag(T650_BODY_INERTIA).round(6)} kg·m² (T650), "
                  f"Ixy {T650_BODY_INERTIA[0, 1]:+.6f}; authored as principal moments "
                  f"{moments.round(6)} + principalAxes (w,x,y,z) {quat.round(4)}",
                  flush=True)
        # Read the tensor back off the stage and confirm it round-trips.
        I_stage = read_inertia_tensor(prim)
        err = (np.abs(I_stage - T650_BODY_INERTIA).max()
               if I_stage is not None else float("nan"))
        print(f"[AM-T650-WB] inertia round-trip off the stage: max err {err:.2e} kg·m²",
              flush=True)

        # The consistency gate compares the NOMINAL total against what the
        # controller believes. A deliberate mass injection must not trip it —
        # that mismatch is the experiment, not a stale config.
        total_mass = T650_BODY_MASS + m_rest
        total_mass_plant = mass_plant + m_rest
        if EXPECTED_TOTAL_MASS:
            expected_total_mass = float(EXPECTED_TOTAL_MASS)
            if not math.isclose(total_mass, expected_total_mass, rel_tol=0.0,
                                abs_tol=5e-4):
                raise RuntimeError(
                    "AM-T650 plant/controller mass mismatch: Isaac authored "
                    f"{total_mass:.6f} kg, but the launcher expects "
                    f"{expected_total_mass:.6f} kg. Fix t650_params.py and the "
                    "paired controller YAML before flight."
                )

        print(f"[AM-T650-WB] T650 MASS OVERRIDE: {body_path} {m_old:.6f} -> "
              f"{T650_BODY_MASS:.6f} kg (delta {self._body_dm:+.6f}); "
              f"other bodies {m_rest:.6f} kg; "
              f"TOTAL {m_old + m_rest:.6f} -> {total_mass:.6f} kg "
              f"(weight {total_mass * 9.81:.2f} N)"
              + (f"; PLANT FLIES {total_mass_plant:.6f} kg "
                 f"(weight {total_mass_plant * 9.81:.2f} N) under the "
                 f"x{PLANT_MASS_SCALE:.4f} injection" if PLANT_MASS_SCALE != 1.0 else "")
              + ". "
              f"vehicle_mass in params_..._t650_aerial_manipulator.yaml MUST equal this total. "
              f"The .usda is untouched.", flush=True)

    def _setup_gripper_drive(self):
        stage = omni.usd.get_context().get_stage()
        root = stage.GetPrimAtPath(self.drone_path)
        prim = next((p for p in Usd.PrimRange(root)
                     if p.GetName() == GRIPPER_JOINT and p.IsA(UsdPhysics.RevoluteJoint)), None)
        if prim is None:
            print(f"[AM-T650-WB] WARNING: gripper joint '{GRIPPER_JOINT}' not found",
                  flush=True)
            return
        drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
        drive.GetStiffnessAttr().Set(1e6)
        drive.GetDampingAttr().Set(1e4)
        drive.GetMaxForceAttr().Set(1000.0)
        drive.GetTargetPositionAttr().Set(GRIPPER_REST_DEG)
        print(f"[AM-T650-WB] gripper drive stiffened on {prim.GetPath()}", flush=True)

    def _arm_prims(self):
        root = self.stage.GetPrimAtPath(self.drone_path)
        prims = {p.GetName(): p for p in Usd.PrimRange(root)
                 if p.IsA(UsdPhysics.RevoluteJoint)}
        return [(n, prims[n]) for n in ARM_JOINT_NAMES if n in prims]

    def _set_arm_armature(self):
        for name, prim in self._arm_prims():
            PhysxSchema.PhysxJointAPI(prim).CreateArmatureAttr().Set(float(ARM_ARMATURE))
        print(f"[AM-T650-WB] arm armature = {ARM_ARMATURE:.4f} kg·m²", flush=True)

    # ── control step: servo emulation tracking the ROS 2 reference ──────────

    def _control_step(self, step_size):
        try:
            self._control_step_inner(step_size)
        except Exception as exc:
            carb.log_error(f"[AM-T650-WB] control step: {exc}")

    def _control_step_inner(self, dt):
        self._t += dt

        # Pump the arm bridge callbacks (non-blocking).
        self._rclpy.spin_once(self._arm_node, timeout_sec=0.0)

        pose = self._dc.get_rigid_body_pose(self._body)
        p0 = np.array([pose.p.x, pose.p.y, pose.p.z])
        R0 = C.quat_to_rot(pose.r.w, pose.r.x, pose.r.y, pose.r.z)
        lin = self._dc.get_rigid_body_linear_velocity(self._body)
        ang = self._dc.get_rigid_body_angular_velocity(self._body)
        v0     = R0.T @ np.array([lin.x, lin.y, lin.z])
        omega0 = R0.T @ np.array([ang.x, ang.y, ang.z])
        q_all  = np.asarray(self._art.get_joint_positions(), float)
        qd_all = np.asarray(self._art.get_joint_velocities(), float)
        q    = q_all[self._arm_idx]
        qdot = qd_all[self._arm_idx]

        # gravity comp at the CURRENT attitude (the servo integrator's role
        # in the emulation) — model in AM_realign's old frame, adapt.
        R0_m = R0 @ R_MODEL
        v0_m = R_MODEL.T @ v0
        om_m = R_MODEL.T @ omega0
        X = np.concatenate([p0, R0_m.flatten(order="F"), q, v0_m, om_m, qdot])
        g_arm = C.dynamics(X, self.params)["g"][6:]

        # ── Torque source arbitration (hysteresis, mirrors the controller) ─
        fresh = (self._cmd_stamp_t is not None
                 and (self._t - self._cmd_stamp_t) < CMD_FRESH_S)
        if not fresh:
            self._fresh_count = 0
            if self._ext_active:
                # Stale edge: torque authority is DROPPED, never latched. Fall
                # back to the PD+gravity hold AT THE CURRENT POSE.
                self._ext_active = False
                self._q_hold_target = np.asarray(q, float).copy()
                self._hold_ref = np.asarray(q, float).copy()
                print(f"[AM-T650-WB] external torque stream STALE at "
                      f"t={self._t:.1f}s — PD+gravity hold at "
                      f"q={np.degrees(q).round(1)} deg", flush=True)
        elif not self._ext_active:
            if self._n_cmds != getattr(self, "_seen_cmds", -1):
                self._seen_cmds = self._n_cmds
                self._fresh_count += 1
            if self._fresh_count >= REARM_FRESH_N:
                self._ext_active = True
                print(f"[AM-T650-WB] external torque stream LIVE at "
                      f"t={self._t:.1f}s — applying commanded efforts",
                      flush=True)

        if self._ext_active:
            tau_cmd = np.asarray(self._tau_cmd, float)
        else:
            # 05's flight-validated servo-emulation hold, slewed, clamped.
            if self._hold_ref is None:
                self._hold_ref = np.asarray(q, float).copy()
            d_ref = np.clip(self._q_hold_target - self._hold_ref,
                            -ARM_HOLD_RATE * dt, ARM_HOLD_RATE * dt)
            self._hold_ref = self._hold_ref + d_ref
            tau_cmd = -ARM_HOLD_KP * (q - self._hold_ref) - ARM_HOLD_KD * qdot + g_arm

        # THE SERVO. Both branches go through it: the droop is a property of
        # the motor, not of whoever computed the command, and on hardware the
        # controller's own PD fallback pays it too. qd_ref is left at zero —
        # the arm controller's back-EMF feedforward tracks the REFERENCE
        # velocity, which whole-body DIRECT holds near zero (measured peak
        # 0.009 rad/s on joint 2 while the joint itself reached 2.36).
        tau = ARM_SERVO.applied(tau_cmd, qdot)
        self._art.set_joint_efforts(np.asarray(tau, float),
                                    joint_indices=self._arm_idx)
        self._tau_cmd_applied = np.clip(tau_cmd, -ARM_SERVO.tau_cap, ARM_SERVO.tau_cap)
        self._tau_applied = tau

        # Joint states out — this is what IsaacTopicSystem.read() latches and
        # what /joint_states (broadcaster) and the arm ground station show.
        # `effort` carries the APPLIED torque, which is what the hardware
        # backend reports there (the servo's Present Current); before the servo
        # model existed the two were the same number.
        self._publish_arm_state(q, qdot, tau)

        if self._t - self._status_t >= STATUS_PERIOD_S:
            self._status_t = self._t
            try:
                omega_real = np.asarray(self._thrust_curve.velocity, float)
                omega_txt = np.array2string(omega_real.round(0), separator=",")
            except Exception:
                omega_txt = "n/a"
            if self._cmd_stamp_t is None:
                cmd_txt = "none yet (PD hold at spawn pose)"
            else:
                cmd_txt = (f"n={self._n_cmds}, age "
                           f"{self._t - self._cmd_stamp_t:4.1f}s")
            mode_txt = "EXT-TORQUE" if self._ext_active else "PD-HOLD"
            print(f"[AM-T650-WB] t={self._t:7.1f}s  z={p0[2]:6.3f} m  "
                  f"|v|={np.linalg.norm(v0):5.2f} m/s  arm={mode_txt}  "
                  f"q={np.degrees(q).round(1)} deg  "
                  f"|tau|max cmd/app="
                  f"{np.abs(self._tau_cmd_applied).max():4.2f}/"
                  f"{np.abs(tau).max():4.2f} N·m  "
                  f"cmds: {cmd_txt}  omega={omega_txt} rad/s", flush=True)

    # ── main loop ───────────────────────────────────────────────────────────

    def run(self):
        self.world.add_physics_callback("am_wb_arm_torque", self._control_step)
        # No START_PAUSED (04's rationale): /uav_0/state/* must flow for the
        # mocap emulator, and /uav_0/isaacsim_manipulator/joint_states must flow for
        # the arm stack's hardware activation.
        self.timeline.play()
        steps = 0
        while (simulation_app.is_running()
               and not self.stop_sim
               and (not STEP_LIMIT or steps < STEP_LIMIT)):
            self.world.step(render=not HEADLESS)
            steps += 1
            # Drawn from the RENDER loop, not the physics callback: the lines
            # only need to keep up with the eye, and clear+rebuild at 250 Hz
            # would be pure waste.
            if self._draw is not None and steps % VIZ_REDRAW_STEPS == 0:
                self._draw_trajectory()
        carb.log_warn("[AM-T650-WB] Simulation App is closing.")
        try:
            self._arm_node.destroy_node()
        except Exception:
            pass
        self.timeline.stop()
        simulation_app.close()


def main():
    AmT650WholeBodyArmSim().run()


if __name__ == "__main__":
    main()
