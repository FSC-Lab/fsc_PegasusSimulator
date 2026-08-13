#!/usr/bin/env python
"""
05_px4_direct_t650_aerial_manipulator_ros2_arm_hold.py

Author: Shiqi Gao (shiqi.gao907@gmail.com)

AM-T650 DIRECT-actuation plant with the arm commanded OVER ROS 2 by the
fsc_open_manipulator position-mode stack, instead of 04's in-process hold.
The drone side is byte-for-byte 04's rig (AM_xfwd.usda, T650 motor model,
PX4-primary, /uav_0/state/* to the external fsc_autopilot_ros2 DIRECT stack).
What changed is ONLY who owns the arm reference:

    fsc_open_manipulator PositionController        (the REAL plugin — the same
        (arm_position_controller, aerial config,    .so the Gazebo and hardware
         home [0, 40, 40, 0] deg, min-jerk moves)   bring-ups load)
        ↕ position command/state interfaces
    open_manipulator_x_isaac_bridge/IsaacTopicSystem   (ros2_control hardware)
        → /uav_0/arm/joint_position_commands  (sensor_msgs/JointState)
        ← /uav_0/arm/joint_states
    THIS process (physics callback, 250 Hz):
        Dynamixel-servo EMULATION: PD + gravity comp TRACKING the streamed
        position reference (04's flight-validated hold law and gains — the
        reference source changed from a fixed Q_HOME to the ROS 2 command)
        → set_joint_efforts

Position mode on the real inverted arm runs each servo's internal position
loop while the software streams min-jerk references; here the PD + gravity
comp + clamp stands in for that internal loop (the gravity comp plays the
servo integrator's role of absorbing the load torque — same emulation pattern
as the AK40-10 winch emulator). Until the first command arrives — and if the
ROS 2 stack ever dies — the reference LATCHES (initially at the spawn pose,
which IS home), so this process alone behaves exactly like 04.

The PositionController homes on activation and holds; with the plant spawned
at home that activation move is a no-op, giving this integration step's goal:
the home pose commanded and held over ROS 2 for the whole flight.

Run with:
  scripts/indoor_sim/start_t650_aerial_manipulator_direct_actuator_ros2_arm_sitl.sh <config>
(starts this plant, the arm ros2_control stack and the arm ground station;
pairs with fsc_autopilot_ros2's
start_direct_actuation_t650_aerial_manipulator_stack.sh, started FIRST — it
owns MicroXRCEAgent).
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

from fsc_aerial_manipulation.utils import add_dome_lighting
from fsc_aerial_manipulation.rotorcraft.x650_rotorcraft_utils import (
    print_mass_inertia_properties,
    print_rotor_positions,
)
from fsc_aerial_manipulation.rotorcraft import t650_params
from fsc_aerial_manipulation.rotorcraft.lagged_thrust_curve import LaggedQuadraticThrustCurve
from fsc_aerial_manipulation.robotic_arm.utils_vehicle.x650_multirotor import (
    MultirotorMod, MultirotorConfig)
from fsc_aerial_manipulation.robotic_arm.utils_controller import controller as C

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

# ── ARM-ROS2: the bridge topics (must match the IsaacTopicSystem params in
# open_manipulator_x_isaac_bridge/urdf/open_manipulator_x_isaac_aerial.urdf).
# States/commands are matched BY NAME using the controller-side joint names,
# so neither side depends on the other's ordering.
ARM_ROS_JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4"]
ARM_STATE_TOPIC     = "/uav_0/arm/joint_states"
ARM_CMD_TOPIC       = "/uav_0/arm/joint_position_commands"

T650_BODY_MASS    = float(t650_params.BODY_MASS)
T650_BODY_INERTIA = np.asarray(t650_params.INERTIA_DIAG)

# ── Servo-emulation law: 04's flight-validated hold, reference now streamed ──
ARM_HOLD_KP   = 3.0    # [N·m/rad]
ARM_HOLD_KD   = 0.25   # [N·m·s/rad]
ARM_HOLD_RATE = 0.5    # [rad/s] reference slew — transparent to the position
                       # controller's <= 0.2 rad/s min-jerk moves; caps the step
                       # if something publishes a raw far-away target directly
TAU_MAX       = 3.0    # [N·m]
ARM_ARMATURE  = 353.5 ** 2 * 1.6e-7

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


class AmT650Ros2ArmSim:
    """04's plant with the arm reference owned by the ROS 2 position stack."""

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
        print(f"[AM-T650-ARM] core Articulation API active; arm dof indices "
              f"{self._arm_idx}, gripper dof {self._grip_idx}", flush=True)

        # --- Control model (gravity comp only) ------------------------------
        self.params = C.make_params()
        if getattr(self, "_body_dm", 0.0):
            self.params["m_i"][0] += self._body_dm
            if self._body_dI is not None:
                self.params["I_i_i"][0] = self.params["I_i_i"][0] + np.diag(self._body_dI)
            print(f"[AM-T650-ARM] control model mirrored: m0="
                  f"{self.params['m_i'][0]:.6f} kg, m_total="
                  f"{sum(self.params['m_i']):.6f} kg (hover thrust "
                  f"{sum(self.params['m_i']) * self.params['g']:.2f} N)", flush=True)

        # --- Arm spawn at HOME + rigid re-seat + ground seat (04's sequence) -
        beta_spawn = float(Q_HOME[1] + Q_HOME[2])
        if beta_spawn < math.radians(40.0):
            print(f"[AM-T650-ARM] WARNING: spawn fold beta = "
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
        print(f"[AM-T650-ARM] arm spawned at HOME q = "
              f"{np.degrees(Q_HOME).round(1)} deg (body re-seated: teleport "
              f"moved it {np.linalg.norm(pb - pa):.3f} m)", flush=True)

        pose_g = self._dc.get_rigid_body_pose(self._body)
        dz = GROUND_BODY_Z - float(pose_g.p.z)
        pr, qr = self._art.get_world_pose()
        self._art.set_world_pose(
            position=np.asarray(pr, float) + np.array([0.0, 0.0, dz]))
        print(f"[AM-T650-ARM] ground-seated: body z {pose_g.p.z:.3f} -> "
              f"{GROUND_BODY_Z:.3f} m (dz={dz:+.3f})", flush=True)

        # --- Arm actuation: TRUE effort control from the start ---------------
        self._set_arm_armature()
        actrl = self._art.get_articulation_controller()
        for i in self._arm_idx:
            actrl.switch_dof_control_mode(dof_index=i, mode="effort")
        print("[AM-T650-ARM] arm dofs in effort mode; servo emulation "
              f"KP={ARM_HOLD_KP} KD={ARM_HOLD_KD} clamp={TAU_MAX} N·m — runs "
              f"UNCONDITIONALLY, reference latches when no command flows",
              flush=True)

        # --- ARM-ROS2: bridge node (position commands in, joint states out) --
        self._setup_arm_ros2_bridge()

        self._hold_ref = None       # slewed reference actually tracked
        self._q_cmd = Q_HOME.copy()   # latest ROS 2 command (latched)
        self._cmd_stamp_t = None    # sim time the last command arrived
        self._n_cmds = 0
        self._tau_applied = np.zeros(len(self._arm_idx))
        self._t = 0.0
        self._status_t = -1e9
        self.stop_sim = False
        carb.log_info("[AM-T650-ARM] ready")

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
        self._arm_node = RclpyNode(f"isaac_arm_servo_bridge_{VEHICLE_ID}")
        self._arm_state_pub = self._arm_node.create_publisher(
            JointState, ARM_STATE_TOPIC, 10)
        self._arm_cmd_sub = self._arm_node.create_subscription(
            JointState, ARM_CMD_TOPIC, self._on_arm_command, 10)
        print(f"[AM-T650-ARM] arm ROS2 bridge up: states -> {ARM_STATE_TOPIC}, "
              f"commands <- {ARM_CMD_TOPIC} (names {ARM_ROS_JOINT_NAMES})",
              flush=True)

    def _on_arm_command(self, msg):
        q = self._q_cmd.copy()
        matched = 0
        for j, nm in enumerate(ARM_ROS_JOINT_NAMES):
            try:
                k = list(msg.name).index(nm)
            except ValueError:
                continue
            if k < len(msg.position) and math.isfinite(msg.position[k]):
                q[j] = float(msg.position[k])
                matched += 1
        if matched:
            self._q_cmd = q
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
        print(f"[AM-T650-ARM] spawned PX4-PRIMARY AM at {drone_prim_path}: MN4010 "
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
        carb.log_error(f"[AM-T650-ARM] prim never appeared: {prim_path}")
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
        print(f"[AM-T650-ARM] kept one PhysicsScene: {keep.GetPath()}", flush=True)

    def _disable_self_collisions(self):
        stage = omni.usd.get_context().get_stage()
        root = stage.GetPrimAtPath(self.drone_path)
        target = next((pr for pr in Usd.PrimRange(root)
                       if pr.HasAPI(UsdPhysics.ArticulationRootAPI)), root)
        PhysxSchema.PhysxArticulationAPI.Apply(target).CreateEnabledSelfCollisionsAttr().Set(False)
        print(f"[AM-T650-ARM] self-collision disabled on {target.GetPath()}", flush=True)

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
        print(f"[AM-T650-ARM] disabled {n} rotor collider(s).", flush=True)

    def _apply_t650_body_override(self):
        """T650 mass + inertia onto /body, live stage — see 04's docstring."""
        self._body_dm = 0.0
        self._body_dI = None

        stage = omni.usd.get_context().get_stage()
        body_path = self.drone_path + BODY_PATH
        prim = stage.GetPrimAtPath(body_path)
        if not prim or not prim.IsValid():
            print(f"[AM-T650-ARM] WARNING: body prim {body_path} not found — "
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

        mass_api.CreateMassAttr().Set(T650_BODY_MASS)
        self._body_dm = T650_BODY_MASS - m_old
        mass_api.CreateDiagonalInertiaAttr().Set(
            Gf.Vec3f(*[float(x) for x in T650_BODY_INERTIA]))
        if I_old is not None:
            self._body_dI = T650_BODY_INERTIA - I_old
            print(f"[AM-T650-ARM] body inertia {I_old.round(6)} -> "
                  f"{T650_BODY_INERTIA.round(6)} kg·m² (T650)", flush=True)

        print(f"[AM-T650-ARM] T650 MASS OVERRIDE: {body_path} {m_old:.6f} -> "
              f"{T650_BODY_MASS:.6f} kg (delta {self._body_dm:+.6f}); "
              f"other bodies {m_rest:.6f} kg; "
              f"TOTAL {m_old + m_rest:.6f} -> {T650_BODY_MASS + m_rest:.6f} kg "
              f"(weight {(T650_BODY_MASS + m_rest) * 9.81:.2f} N). "
              f"vehicle_mass in params_..._t650_aerial_manipulator.yaml MUST equal this total. "
              f"The .usda is untouched.", flush=True)

    def _setup_gripper_drive(self):
        stage = omni.usd.get_context().get_stage()
        root = stage.GetPrimAtPath(self.drone_path)
        prim = next((p for p in Usd.PrimRange(root)
                     if p.GetName() == GRIPPER_JOINT and p.IsA(UsdPhysics.RevoluteJoint)), None)
        if prim is None:
            print(f"[AM-T650-ARM] WARNING: gripper joint '{GRIPPER_JOINT}' not found",
                  flush=True)
            return
        drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
        drive.GetStiffnessAttr().Set(1e6)
        drive.GetDampingAttr().Set(1e4)
        drive.GetMaxForceAttr().Set(1000.0)
        drive.GetTargetPositionAttr().Set(GRIPPER_REST_DEG)
        print(f"[AM-T650-ARM] gripper drive stiffened on {prim.GetPath()}", flush=True)

    def _arm_prims(self):
        root = self.stage.GetPrimAtPath(self.drone_path)
        prims = {p.GetName(): p for p in Usd.PrimRange(root)
                 if p.IsA(UsdPhysics.RevoluteJoint)}
        return [(n, prims[n]) for n in ARM_JOINT_NAMES if n in prims]

    def _set_arm_armature(self):
        for name, prim in self._arm_prims():
            PhysxSchema.PhysxJointAPI(prim).CreateArmatureAttr().Set(float(ARM_ARMATURE))
        print(f"[AM-T650-ARM] arm armature = {ARM_ARMATURE:.4f} kg·m²", flush=True)

    # ── control step: servo emulation tracking the ROS 2 reference ──────────

    def _control_step(self, step_size):
        try:
            self._control_step_inner(step_size)
        except Exception as exc:
            carb.log_error(f"[AM-T650-ARM] control step: {exc}")

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

        # PD + gravity comp toward the LATCHED ROS 2 command, slewed, clamped.
        if self._hold_ref is None:
            self._hold_ref = np.asarray(q, float).copy()
        d_ref = np.clip(self._q_cmd - self._hold_ref,
                        -ARM_HOLD_RATE * dt, ARM_HOLD_RATE * dt)
        self._hold_ref = self._hold_ref + d_ref
        tau = -ARM_HOLD_KP * (q - self._hold_ref) - ARM_HOLD_KD * qdot + g_arm
        tau = np.clip(tau, -TAU_MAX, TAU_MAX)
        self._art.set_joint_efforts(np.asarray(tau, float),
                                    joint_indices=self._arm_idx)
        self._tau_applied = tau

        # Joint states out — this is what IsaacTopicSystem.read() latches and
        # what /joint_states (broadcaster) and the arm ground station show.
        self._publish_arm_state(q, qdot, tau)

        if self._t - self._status_t >= STATUS_PERIOD_S:
            self._status_t = self._t
            try:
                omega_real = np.asarray(self._thrust_curve.velocity, float)
                omega_txt = np.array2string(omega_real.round(0), separator=",")
            except Exception:
                omega_txt = "n/a"
            if self._cmd_stamp_t is None:
                cmd_txt = "none yet (holding spawn pose)"
            else:
                cmd_txt = (f"n={self._n_cmds}, age "
                           f"{self._t - self._cmd_stamp_t:4.1f}s")
            print(f"[AM-T650-ARM] t={self._t:7.1f}s  z={p0[2]:6.3f} m  "
                  f"|v|={np.linalg.norm(v0):5.2f} m/s  "
                  f"q_err={np.degrees(q - self._q_cmd).round(1)} deg  "
                  f"|tau|max={np.abs(tau).max():4.2f} N·m  "
                  f"cmds: {cmd_txt}  omega={omega_txt} rad/s", flush=True)

    # ── main loop ───────────────────────────────────────────────────────────

    def run(self):
        self.world.add_physics_callback("am_ros2_arm", self._control_step)
        # No START_PAUSED (04's rationale): /uav_0/state/* must flow for the
        # mocap emulator, and /uav_0/arm/joint_states must flow for the arm
        # stack's hardware activation.
        self.timeline.play()
        steps = 0
        while (simulation_app.is_running()
               and not self.stop_sim
               and (not STEP_LIMIT or steps < STEP_LIMIT)):
            self.world.step(render=not HEADLESS)
            steps += 1
        carb.log_warn("[AM-T650-ARM] Simulation App is closing.")
        try:
            self._arm_node.destroy_node()
        except Exception:
            pass
        self.timeline.stop()
        simulation_app.close()


def main():
    AmT650Ros2ArmSim().run()


if __name__ == "__main__":
    main()
