
#!/usr/bin/env python
"""
| Description: Isaac Sim side of the impedance controller test.
|              Pegasus spawns the aerial manipulator and publishes state.
|              Rotor commands can arrive directly through ROS 2 or through
|              PX4 Offboard direct-actuator control and MAVLink.
|              A small subscriber thread additionally receives joint torque
|              commands and applies them to the arm each physics step.
|
|   Architecture:
|
|     ┌── Isaac Sim (this script) ─────────────────────────────────────────┐
|     │  Pegasus Multirotor                                                │
|     │    ROS2Backend  ──publishes──►  /uav_0/state/*, joint_states       │
|     │    motor input   ◄────────────  ROS rotor command OR PX4 MAVLink    │
|     │  JointTorqueApplier thread                                         │
|     │                 ◄─subscribes──  /uav_0/joint_torque_cmd            │
|     └────────────────────────────────────────────────────────────────────┘
|              ▲ DDS
|     ┌── ImpedanceControllerNode ────────────────────────────────────────┐
|     │  subscribes  /uav_0/state/*, /uav_0/joint_states                  │
|     │  computes    thrust, R_desired, tau_body, tau_joint               │
|     │  mixer       thrust + tau_body → 4 rotor ω commands               │
|     │  publishes   rotor_velocity_command OR PX4 ActuatorMotors         │
|     │  publishes   /uav_0/joint_torque_cmd                              │
|     └───────────────────────────────────────────────────────────────────┘
|
|   To run (Linux, single machine — no WSL bridge needed):
|     Use the launch script, which starts both panes in tmux:
|          ./scripts/indoor_sim/start_aerial_manipulator.sh <your_name>_machine direct
|          ./scripts/indoor_sim/start_aerial_manipulator.sh <your_name>_machine px4-offboard
|
|     Or start the two processes manually:
|       1. Isaac Sim side (this script), via the Isaac python wrapper:
|            $ISAAC_PY application/robotic_arm/gripper_bat_impedance_demo.py
|       2. Controller node, in a ROS2-sourced shell, under the /uav_0 namespace
|          so its relative topics (state/pose, joint_states, ...) map onto the
|          Pegasus uav_0/... topics:
|            python3 application/robotic_arm/controller.py --ros-args -r __ns:=/uav_0
"""

import os
import sys
import math

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

# --- enable the PhysX debug / visualization UI (fixes greyed-out
#     Utilities > Profilers & Debuggers menu). Must run after SimulationApp. ---
from isaacsim.core.utils.extensions import enable_extension
enable_extension("omni.physx.ui")             # PhysX Debug Visualization window
enable_extension("omni.kit.profiler.window")  # (optional) Profiler window
simulation_app.update()

import numpy as np
import carb
import omni.usd
import omni.timeline
from omni.isaac.core.world import World
from omni.isaac.dynamic_control import _dynamic_control
from pxr import Usd, UsdPhysics, PhysxSchema

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

from pegasus.simulator.params import SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

from fsc_aerial_manipulation.utils import add_dome_lighting
# x650_rotorcraft_utils lives in the rotorcraft sub-package; import it by its
# fully-qualified path (the package-level spawn_rotorcraft_with_mavlink exported
# by fsc_aerial_manipulation.rotorcraft is the *older* variant without
# return_backend/thrust_config — this demo needs the x650 one).
from fsc_aerial_manipulation.rotorcraft.x650_rotorcraft_utils import (
    spawn_rotorcraft_with_mavlink,
    print_mass_inertia_properties,
    print_rotor_positions,
)

# ╔══════════════════════════════════════════════════════════════════════════╗
# ║  CONFIG                                                                  ║
# ╚══════════════════════════════════════════════════════════════════════════╝

# Vehicle USD models live in the fsc_aerial_manipulation extension package under
# rotorcraft/assets/. Resolve the folder from the installed package (editable
# install) so the path holds regardless of where the repo lives. To switch models,
# change the filename below (and the prim/joint config that follows to match).
#   available: AM_realign.usda
import fsc_aerial_manipulation.rotorcraft as _fsc_rotorcraft
ASSETS_DIR = os.path.join(os.path.dirname(_fsc_rotorcraft.__file__), "assets")
USD_FILE   = os.path.join(ASSETS_DIR, "AM_realign.usda")

# These must match the prim structure in gripper_bat_processed.usd
USD_PRIM_PATH    = "/gripper_bat"
BODY_PATH        = "/body"
ROTOR_PATHS      = ["/rotor0", "/rotor1", "/rotor2", "/rotor3"]
ROTOR_JOINT_NAMES = ["joint0", "joint1", "joint2", "joint3"]

SPAWN_POS   = (0.0, 0.0, 0.5)
SPAWN_EULER = (0.0, 0.0, 0.0)

VEHICLE_ID  = 0   # determines ROS2 topic namespace: /uav_0/...

# Rotor command path. The launch script sets this to ``px4_offboard`` when PX4
# must own the plant's motor input. Keep ``direct`` as the standalone default so
# the original two-process demo still works without PX4 SITL or px4_msgs.
CONTROL_MODE = os.environ.get("AERIAL_MANIPULATOR_CONTROL_MODE", "direct")
if CONTROL_MODE not in {"direct", "px4_offboard"}:
    raise ValueError(
        "AERIAL_MANIPULATOR_CONTROL_MODE must be 'direct' or 'px4_offboard', "
        f"got {CONTROL_MODE!r}"
    )
PX4_OFFBOARD = CONTROL_MODE == "px4_offboard"

# --- PhysX debug frame visualization -----------------------------------------
# Draw the world origin frame and every rigid-body frame (RGB = XYZ) using the
# PhysX debug visualizer. Frames only render while the sim is PLAYING.
# Flip SHOW_PHYSICS_FRAMES to False to turn it all off for a run.
SHOW_PHYSICS_FRAMES  = True
SHOW_WORLD_AXES      = True    # world origin frame
SHOW_BODY_AXES       = True    # each rigid body's link frame
SHOW_BODY_MASS_AXES  = False   # each body's centre-of-mass frame
PHYSICS_FRAME_SCALE  = 10.0     # axis length multiplier (0 → invisible)

# ROS2 topic for joint torque commands (published by WSL controller node).
# Controller publishes Float64MultiArray: [tau_joint1, tau_joint2, tau_joint3, tau_joint4].
JOINT_TORQUE_TOPIC = f"/uav_{VEHICLE_ID}/joint_torque_cmd"

# ROS2 topic for rotor velocity commands (published by WSL controller node).
# Received here (not in the Pegasus backend) so ros2_backend.py stays stock —
# the subscriber writes straight into the ROS2 backend's input_ref[].
ROTOR_CMD_TOPIC = f"/uav_{VEHICLE_ID}/rotor_velocity_command"

# Arm joints in controller order (manip_joint1 = q[0], etc.)
ARM_JOINT_NAMES = ["manip_joint1", "manip_joint2", "manip_joint3", "manip_joint4"]

# Arm actuation — direct effort (current/torque) control modeling the real
# OpenManipulator-X joints: 4× DYNAMIXEL XM430-W350-T through a 353.5:1 gearbox.
# The controller commands joint TORQUES; the plant applies them as DOF efforts ON
# TOP OF the servo's physical intrinsics — reflected gearbox inertia (armature),
# viscous damping, and Coulomb/stiction friction. A real Dynamixel is NOT an ideal
# frictionless torque source, so these are required both for a faithful sim and to
# stop the joint running away over the ROS2 loop latency. There is no position-hold
# / mode switch: whether the arm holds still or moves is decided by the commanded
# torque (the controller/trajectory), not by any plant-side lock.
#
# XM430-W350-T @ 12V:  stall torque ≈ 4.1 N·m,  no-load speed ≈ 4.81 rad/s (46 rpm),
#                      gear ratio 353.5:1,  motor rotor inertia ≈ 1.6e-7 kg·m².
ARM_MAX_FORCE        = 4.1    # [N·m]      per-joint effort clamp = XM430-W350 stall torque
ARM_VISCOUS_DAMPING  = 0.2    # [N·m·s/rad] gearbox viscous friction (system-ID to refine;
                              #   velocity-mode electromech. damping τ_stall/ω_nl ≈ 0.85 is an upper bound)
ARM_COULOMB_FRICTION = 0.15   # [N·m]      gearbox dry/stiction torque (system-ID to refine)
# Reflected rotor inertia — MUST equal ctrl.py J_arm = gear_ratio²·rotor_inertia.
ARM_ARMATURE  = 353.5 ** 2 * 1.6e-7   # ≈ 0.02 kg·m²

# ARM_TAU_SIGN: sign adapting the controller's joint-torque convention to Isaac's joint
# axis convention. The controller (MATLAB model) and the USD/PhysX joints can disagree on
# the positive-rotation direction; the log proved a global sign error (divergence from
# exactly-zero error). -1 flips it; set back to +1 if the model already matches Isaac.
ARM_TAU_SIGN = +1.0   # model now matches USD geometry (make_params fixed) → no flip needed

# PLATFORM_TUNE: hold the arm RIGID at its initial pose (stiff position drive) so the
# base is a clean quadrotor carrying a fixed payload — for tuning the quadrotor gains.
# MUST match PLATFORM_TUNE in controller.py (which commands zero joint torque and drops
# the arm coupling when True). Set False for full whole-body effort control.
PLATFORM_TUNE = True
# Startup sequencing (when PLATFORM_TUNE=False): the arm starts RIGID (position drive)
# so the drone takes off and stabilizes with the arm at its reference, then it RELEASES
# to effort control this many seconds after the controller starts commanding (jrx>0) —
# i.e. after takeoff+settle. Set slightly ≥ controller ARM_ACTIVATE_TIME so the impedance
# torque is already flowing when the arm goes free (smooth hand-off, no droop slam).
ARM_RELEASE_DELAY = 9.5    # [s after first joint-cmd received] hold, then release to torque

# Rotor model — MUST be kept identical to RotorMixerParams in ctrl.py, since the
# WSL mixer inverts exactly this model to turn a body wrench into rotor speeds.
#   k_thrust : T   = rotor_constant * omega^2
#   k_torque : Mz  = rolling_moment_coefficient * omega^2 * rot_dir
ROTOR_K_THRUST = 1.03e-5   # [N / (rad/s)²]  (rotor_constant)
ROTOR_K_TORQUE = 1.0e-6    # [N·m / (rad/s)²] (rolling_moment_coefficient)
ROTOR_DIR      = [-1, -1, 1, 1]


# ╔══════════════════════════════════════════════════════════════════════════╗
# ║  JOINT TORQUE SUBSCRIBER                                                 ║
# ╚══════════════════════════════════════════════════════════════════════════╝

class JointTorqueApplier:
    """
    Subscribes to joint_torque_cmd using the rclpy context that Pegasus's
    ROS2Backend already initialized.  spin_once() is called from the main
    simulation loop each step — no background thread, no context conflicts.
    """

    def __init__(self, n_joints: int):
        self._tau  = np.zeros(n_joints)
        self._node = None
        self._sub  = None     # subscription handle (diagnostic: publisher count)
        self._rx   = 0        # count of received joint-torque messages (diagnostic)
        self._arm_active_flag = False   # 5th message element: controller has engaged the arm

    def setup(self):
        """Create the subscriber node. Call after Pegasus ROS2Backend has started."""
        try:
            import rclpy
            from rclpy.node import Node
            from std_msgs.msg import Float64MultiArray
        except ImportError:
            carb.log_warn("[JointTorque] rclpy not available — joint torques will be zero.")
            return

        self._node = Node("isaac_joint_torque_receiver")

        def _cb(msg):
            self._rx += 1
            data = np.array(msg.data, dtype=float)
            ntau = len(self._tau)
            self._tau[:min(len(data), ntau)] = data[:ntau]
            if len(data) > ntau:                       # 5th element = arm-active flag
                self._arm_active_flag = bool(data[ntau] > 0.5)

        self._sub = self._node.create_subscription(Float64MultiArray, JOINT_TORQUE_TOPIC, _cb, 10)
        print(f"[JointTorque] Subscribed to {JOINT_TORQUE_TOPIC}", flush=True)

    def spin_once(self):
        """Process any pending messages. Call once per simulation step."""
        if self._node is not None:
            import rclpy
            rclpy.spin_once(self._node, timeout_sec=0)

    def get_torques(self) -> np.ndarray:
        return self._tau.copy()

    def destroy(self):
        if self._node is not None:
            self._node.destroy_node()
            self._node = None


class RotorCommandApplier:
    """
    Subscribes to rotor_velocity_command (Float64MultiArray) and writes the
    rotor speeds straight into the Pegasus ROS2 backend's input_ref[], which the
    Multirotor reads via backend.input_reference() each step.

    This keeps ros2_backend.py STOCK: the atomic single-message rotor update
    lives here in the demo instead of being patched into the shared extension.
    One Float64MultiArray updates all rotors in one callback (no per-rotor
    Float64 race), same as the in-backend version it replaces.
    """

    def __init__(self, backend, n_rotors: int):
        self._backend = backend
        self._n       = n_rotors
        self._node    = None
        self._rx      = 0     # count of received rotor-command messages (diagnostic)

    def setup(self):
        """Create the subscriber node. Call after Pegasus ROS2Backend has started."""
        try:
            from rclpy.node import Node
            from std_msgs.msg import Float64MultiArray
        except ImportError:
            carb.log_warn("[RotorCmd] rclpy not available — rotors will stay at zero.")
            return

        self._node = Node("isaac_rotor_command_receiver")

        def _cb(msg):
            self._rx += 1
            for i, val in enumerate(msg.data):
                if i < self._n:
                    self._backend.input_ref[i] = float(val)

        self._node.create_subscription(Float64MultiArray, ROTOR_CMD_TOPIC, _cb, 10)
        carb.log_info(f"[RotorCmd] Subscribed to {ROTOR_CMD_TOPIC}")

    def spin_once(self):
        """Process any pending rotor command. Call once per simulation step."""
        if self._node is not None:
            import rclpy
            rclpy.spin_once(self._node, timeout_sec=0)

    def destroy(self):
        if self._node is not None:
            self._node.destroy_node()
            self._node = None


class JointStatePublisher:
    """
    Reads arm joint positions + velocities via DC each step and publishes
    sensor_msgs/JointState to the /uav_0/joint_states topic.
    The controller node in WSL subscribes to this to get q and q_dot.
    """

    JOINT_STATES_TOPIC = f"/uav_{VEHICLE_ID}/joint_states"

    def __init__(self, joint_names: list):
        self._names = joint_names
        self._node  = None
        self._pub   = None

    def setup(self):
        try:
            from rclpy.node import Node
            from sensor_msgs.msg import JointState as JointStateMsg
        except ImportError:
            carb.log_warn("[JointStatePub] rclpy not available.")
            return
        self._node = Node("isaac_joint_state_publisher")
        self._pub  = self._node.create_publisher(JointStateMsg, self.JOINT_STATES_TOPIC, 10)
        carb.log_info(f"[JointStatePub] Publishing to {self.JOINT_STATES_TOPIC}")

    def publish(self, dof_states, arm_dof_indices: list):
        if self._pub is None:
            return
        from sensor_msgs.msg import JointState as JointStateMsg
        import rclpy
        msg = JointStateMsg()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.name     = self._names
        msg.position = [float(dof_states[i]['pos']) for i in arm_dof_indices]
        msg.velocity = [float(dof_states[i]['vel']) for i in arm_dof_indices]
        self._pub.publish(msg)
        rclpy.spin_once(self._node, timeout_sec=0)

    def destroy(self):
        if self._node is not None:
            self._node.destroy_node()
            self._node = None


# ╔══════════════════════════════════════════════════════════════════════════╗
# ║  DEMO                                                                    ║
# ╚══════════════════════════════════════════════════════════════════════════╝

class GripperBatImpedanceDemo:

    def __init__(self):
        self.timeline = omni.timeline.get_timeline_interface()

        # --- Pegasus world + environment -------------------------------------
        self.pg        = PegasusInterface()
        self.pg._world = World(**self.pg._world_settings)
        self.world     = self.pg.world
        self.pg.load_environment(SIMULATION_ENVIRONMENTS["Curved Gridroom"])

        # Convex-hull-decomposition colliders create many sub-shapes → the GPU
        # broadphase overflows its default pair buffers and drops contacts
        # ("found lost aggregate pairs capacity at 2048"). Raise the caps.
        self._raise_physx_gpu_capacity()

        stage = omni.usd.get_context().get_stage()
        add_dome_lighting(
            stage=stage,
            dome_path="/World/DomeLight",
            intensity=2500.0, exposure=0.0, color=(1.0, 1.0, 1.0),
        )

        # In direct mode the ROS2 backend is primary and RotorCommandApplier writes
        # controller omega commands into it. In PX4 Offboard mode the MAVLink
        # backend is primary; ROS2 remains state-only and PX4 receives normalized
        # ActuatorMotors commands from the external controller over uXRCE-DDS.
        print(f"[Demo] Loading vehicle USD: {USD_FILE}", flush=True)
        print(f"[Demo]   exists={os.path.exists(USD_FILE)}  "
              f"prim={USD_PRIM_PATH}  control_mode={CONTROL_MODE}  "
              f"arm_mode={'POSITION-HOLD (PLATFORM_TUNE)' if PLATFORM_TUNE else 'POS→TORQUE hand-off'}",
              flush=True)
        self.drone_path, self._ros2_backend = spawn_rotorcraft_with_mavlink(
            px4_path=self.pg.px4_path,
            px4_default_airframe=self.pg.px4_default_airframe,
            vehicle_id=VEHICLE_ID,
            spawn_pos=SPAWN_POS,
            spawn_euler=SPAWN_EULER,
            usd_file=USD_FILE,
            usd_prim_path=USD_PRIM_PATH,
            articulation_path="",
            body_path=BODY_PATH,
            rotor_paths=ROTOR_PATHS,
            rotor_joint_names=ROTOR_JOINT_NAMES,
            thrust_config={
                "rotor_constant": [ROTOR_K_THRUST] * 4,
                "rolling_moment_coefficient": [ROTOR_K_TORQUE] * 4,
                "rot_dir": ROTOR_DIR,
            },
            # External DDS traffic can briefly stall PX4 during Offboard mode
            # transitions. Keep HIL real-time/non-blocking so that cannot deadlock
            # the Isaac physics loop waiting for HIL_ACTUATOR_CONTROLS.
            enable_lockstep=False,
            px4_primary=PX4_OFFBOARD,
            include_px4=PX4_OFFBOARD,
            return_backend=True,   # hand back the backend so we can write input_ref
        )

        self._wait_for_prim(self.drone_path, max_frames=300)
        self.world.reset()
        self.stage = omni.usd.get_context().get_stage()
        self._setup_physics_debug_viz()  # world/body frame axes (see SHOW_PHYSICS_FRAMES)
        # Arm starts RIGID (position hold) in all cases. PLATFORM_TUNE keeps it locked
        # forever; otherwise it is released to effort control after takeoff+settle.
        self._setup_arm_position_hold()
        self._set_arm_armature()         # plant matches controller's modeled rotor inertia
        self._arm_released = False
        self._ctrl_start_time = None

        print_mass_inertia_properties(self.stage, self.drone_path)
        print_rotor_positions(self.stage, self.drone_path, ROTOR_PATHS)
        self._print_arm_joint_axes()   # ground-truth kinematics to check controller make_params

        # --- DC interface ----------------------------------------------------
        self._dc     = _dynamic_control.acquire_dynamic_control_interface()
        self._artic  = self._dc.get_articulation(self.drone_path)
        self._n_dofs = self._dc.get_articulation_dof_count(self._artic)
        self._dof_map = self._build_dof_map()
        self._arm_dof_indices = [self._dof_map[n] for n in ARM_JOINT_NAMES]
        self._efforts = [0.0] * self._n_dofs   # full-DOF effort vector (arm DOFs written each step)

        # --- ROS2 subscribers -----------------------------------------------
        self._torque_applier     = JointTorqueApplier(n_joints=len(ARM_JOINT_NAMES))
        self._joint_state_pub    = JointStatePublisher(ARM_JOINT_NAMES)
        self._rotor_applier      = None
        if not PX4_OFFBOARD:
            self._rotor_applier = RotorCommandApplier(self._ros2_backend, n_rotors=len(ROTOR_PATHS))
        self._torque_applier.setup()
        self._joint_state_pub.setup()
        if self._rotor_applier is not None:
            self._rotor_applier.setup()

        # Service our manual subscriber nodes through ONE dedicated executor. Calling
        # rclpy.spin_once() per node on the shared *global* executor starved the
        # joint-torque node (jrx=0) while the rotor node (spun first) worked — the
        # global executor's rapid add/remove churn drops the second node's ready
        # messages. A single executor holding both subscriber nodes services them
        # reliably each step.
        self._ros_exec = None
        try:
            from rclpy.executors import SingleThreadedExecutor
            self._ros_exec = SingleThreadedExecutor()
            for applier in (self._torque_applier, self._rotor_applier):
                if applier is not None and applier._node is not None:
                    self._ros_exec.add_node(applier._node)
        except ImportError:
            carb.log_warn("[Demo] SingleThreadedExecutor unavailable — falling back to per-node spin.")

        carb.log_info(
            "[Demo] Ready.  Start the ImpedanceControllerNode in WSL to begin control."
        )

    # ── Utilities ──────────────────────────────────────────────────────────

    def _wait_for_prim(self, prim_path: str, max_frames: int = 300):
        stage = omni.usd.get_context().get_stage()
        for _ in range(max_frames):
            p = stage.GetPrimAtPath(prim_path)
            if p and p.IsValid():
                return p
            self.world.step(render=False)
        carb.log_error(f"[Demo] Prim never appeared: {prim_path}")
        return None

    def _raise_physx_gpu_capacity(self):
        """Enlarge the GPU broadphase / contact buffers so convex-decomposition
        colliders don't overflow them (PhysX drops pairs at the default 2048)."""
        stage = omni.usd.get_context().get_stage()
        scene = next((p for p in stage.Traverse() if p.IsA(UsdPhysics.Scene)), None)
        if scene is None:
            carb.log_warn("[PhysX] No physics scene found; cannot raise GPU caps.")
            return
        api = PhysxSchema.PhysxSceneAPI.Apply(scene)
        # aggregate-pair buffers (the one in the warning) + related contact caps
        api.CreateGpuFoundLostAggregatePairsCapacityAttr().Set(256 * 1024)
        api.CreateGpuTotalAggregatePairsCapacityAttr().Set(256 * 1024)
        api.CreateGpuFoundLostPairsCapacityAttr().Set(1024 * 1024)
        api.CreateGpuMaxRigidContactCountAttr().Set(2 * 1024 * 1024)
        api.CreateGpuMaxRigidPatchCountAttr().Set(256 * 1024)
        print(f"[PhysX] Raised GPU broadphase/contact capacities on {scene.GetPath()}", flush=True)

    def _setup_physics_debug_viz(self):
        """Toggle PhysX debug-draw of coordinate frames from the CONFIG switches.
        Draws the world origin frame and/or each rigid-body frame (RGB = XYZ).

        Two subtleties, both learned from omni.physxui's own PhysxDebugView:
          1. Turning on the draw needs BOTH interfaces — the physx visualization
             interface (computes the lines) AND the physxui interface's
             enable_debug_visualization (actually renders them in the viewport).
             Calling only the first computes lines nobody draws.
          2. PhysX resets these parameters every time the sim starts, so a
             one-shot call before Play is wiped out. We re-apply on every
             RESUMED simulation event (same as the debug window does)."""
        if not SHOW_PHYSICS_FRAMES:
            return
        try:
            from omni.physx import (get_physx_visualization_interface,
                                    get_physx_interface)
            from omni.physxui import get_physxui_interface
            from omni.physx.bindings._physx import SimulationEvent
        except ImportError:
            carb.log_warn("[PhysX] omni.physx.ui not loaded; cannot draw frames. "
                          "Keep enable_extension('omni.physx.ui') at startup.")
            return

        def _apply():
            viz = get_physx_visualization_interface()
            get_physxui_interface().enable_debug_visualization(True)  # (1) render
            viz.enable_visualization(True)                            # (1) compute
            viz.set_visualization_scale(PHYSICS_FRAME_SCALE)
            viz.set_visualization_parameter("WorldAxes",    SHOW_WORLD_AXES)
            viz.set_visualization_parameter("BodyAxes",     SHOW_BODY_AXES)
            viz.set_visualization_parameter("BodyMassAxes", SHOW_BODY_MASS_AXES)

        _apply()  # take effect immediately if the sim is already playing

        # (2) re-apply on every Play/resume — PhysX wipes the params on sim start.
        def _on_sim_event(event):
            if event.type == int(SimulationEvent.RESUMED):
                _apply()

        events = get_physx_interface().get_simulation_event_stream_v2()
        # keep the subscription alive for the process lifetime (else it's GC'd
        # and the re-apply never fires).
        self._physx_viz_sub = events.create_subscription_to_pop(_on_sim_event)

        print(f"[PhysX] Frame viz: world={SHOW_WORLD_AXES} body={SHOW_BODY_AXES} "
              f"mass={SHOW_BODY_MASS_AXES} scale={PHYSICS_FRAME_SCALE} "
              f"(re-applied on every Play)", flush=True)

    def _print_arm_joint_axes(self):
        """Print each manip_joint's ACTUAL rotation axis + origin in the drone-body
        frame, read straight from the live USD. At the folded (q=0) pose the
        controller's model assumes every link frame coincides with the body frame,
        so make_params h_i_im1[i] should equal these body-frame axes. This is the
        ground truth to compare against controller.py make_params."""
        try:
            from pxr import Usd, UsdGeom, Gf
        except ImportError:
            print("[ARM-AXES] pxr not available; skipping.", flush=True)
            return
        cache = UsdGeom.XformCache()
        axis_map = {"X": (1.0, 0, 0), "Y": (0, 1.0, 0), "Z": (0, 0, 1.0)}
        body_prim = self.stage.GetPrimAtPath(self.drone_path + BODY_PATH)
        if not body_prim or not body_prim.IsValid():
            print(f"[ARM-AXES] body prim not found at {self.drone_path + BODY_PATH}", flush=True)
            return
        M_body_inv = cache.GetLocalToWorldTransform(body_prim).GetInverse()
        root = self.stage.GetPrimAtPath(self.drone_path)

        print("\n[ARM-AXES] === actual USD joint axes & origins (drone-body frame, q=0) ===", flush=True)
        rows = {}
        for p in Usd.PrimRange(root):
            n = p.GetName()
            if not n.startswith("manip_joint"):
                continue
            axis_t = p.GetAttribute("physics:axis").Get()
            e = Gf.Vec3d(*axis_map[axis_t])
            lr1 = p.GetAttribute("physics:localRot1").Get()
            rot1 = Gf.Rotation(Gf.Quatd(float(lr1.GetReal()),
                                        Gf.Vec3d(*[float(v) for v in lr1.GetImaginary()])))
            b1 = p.GetRelationship("physics:body1").GetTargets()[0]
            link = self.stage.GetPrimAtPath(b1)
            M_link = cache.GetLocalToWorldTransform(link)
            axis_body = M_body_inv.TransformDir(M_link.TransformDir(rot1.TransformDir(e)))
            lp1 = p.GetAttribute("physics:localPos1").Get()
            origin_body = M_body_inv.Transform(M_link.Transform(Gf.Vec3d(*[float(v) for v in lp1])))
            rows[n] = ([round(axis_body[i], 3) for i in range(3)],
                       [round(origin_body[i], 3) for i in range(3)],
                       axis_t, b1.name)
        for n in sorted(rows):
            ax, org, at, par = rows[n]
            print(f"[ARM-AXES] {n:12s} axis(body)={ax}  origin(body)={org}  "
                  f"(usd_axis={at}, parent={par})", flush=True)
        print("[ARM-AXES] controller make_params: j1=[0,0,1] j2=[-0.998,-0.062,0] "
              "j3=[-0.998,-0.062,0] j4=[0.013,-0.211,0.977]", flush=True)
        print("[ARM-AXES] MATLAB design:          j1=[0,0,1]z j2=[0,1,0]y "
              "j3=[0,1,0]y j4=[0,0,1]z\n", flush=True)

    def _build_dof_map(self) -> dict:
        """Build joint-name → DOF-index map by iterating articulation joints."""
        dof_map = {}
        dof_idx = 0
        n_joints = self._dc.get_articulation_joint_count(self._artic)
        for j in range(n_joints):
            joint  = self._dc.get_articulation_joint(self._artic, j)
            name   = self._dc.get_joint_name(joint)
            n_dofs = self._dc.get_joint_dof_count(joint)
            if n_dofs > 0:
                dof_map[name] = dof_idx
            dof_idx += n_dofs

        print(f"[Demo] Articulation DOFs ({dof_idx} total across {n_joints} joints):", flush=True)
        for name, idx in sorted(dof_map.items(), key=lambda x: x[1]):
            arm_tag = "  ← ARM" if name in ARM_JOINT_NAMES else ""
            print(f"  [{idx}] {name}{arm_tag}", flush=True)
        return dof_map

    def _get_joint_prims(self) -> dict:
        root = self.stage.GetPrimAtPath(self.drone_path)
        return {p.GetName(): p for p in Usd.PrimRange(root)
                if p.IsA(UsdPhysics.RevoluteJoint)} if root and root.IsValid() else {}

    def _setup_arm_position_hold(self):
        """PLATFORM_TUNE: hold the arm RIGID at its baked-in initial pose with a stiff
        position drive, so the base is a clean quadrotor carrying a fixed payload.
        High force limit (not the servo limit) — this is a tuning fixture, not the
        real actuator. The controller commands zero joint torque in this mode."""
        joint_prims = self._get_joint_prims()
        for name in ARM_JOINT_NAMES:
            prim = joint_prims.get(name)
            if not prim:
                carb.log_warn(f"[Demo] Arm joint not found: {name}")
                continue
            state_attr = prim.GetAttribute("state:angular:physics:position")
            initial_deg = float(state_attr.Get()) if state_attr and state_attr.HasValue() else 0.0
            drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
            drive.CreateTypeAttr().Set("force")
            drive.GetStiffnessAttr().Set(1e4)
            drive.GetDampingAttr().Set(500.0)
            drive.GetMaxForceAttr().Set(1e6)
            drive.GetTargetPositionAttr().Set(initial_deg)
            print(f"[Demo] PLATFORM_TUNE hold: {name} @ {initial_deg:.1f}° (rigid payload)", flush=True)

    def _setup_arm_effort_mode(self):
        """Configure the arm joints for direct effort (torque) control with the
        XM430-W350 servo's physical viscous damping.

        Zero position stiffness (no position-hold), but a NON-zero drive damping
        models the Dynamixel gearbox's viscous friction: with target velocity 0 the
        drive contributes τ = -b·ω, i.e. speed-proportional resistance, on top of the
        commanded effort applied each step via set_articulation_dof_efforts. No mode
        switching — the arm's motion is dictated by the commanded torque plus this
        physical friction. (Coulomb friction + armature are set in _set_arm_armature.)
        """
        joint_prims = self._get_joint_prims()
        for name in ARM_JOINT_NAMES:
            prim = joint_prims.get(name)
            if not prim:
                carb.log_warn(f"[Demo] Arm joint not found: {name}")
                continue
            drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
            drive.CreateTypeAttr().Set("force")
            drive.GetStiffnessAttr().Set(0.0)
            drive.GetDampingAttr().Set(ARM_VISCOUS_DAMPING)
            drive.GetMaxForceAttr().Set(ARM_MAX_FORCE)
            drive.GetTargetVelocityAttr().Set(0.0)
            print(f"[Demo] Effort mode: {name} "
                  f"(viscous b={ARM_VISCOUS_DAMPING} N·m·s/rad, τ_max={ARM_MAX_FORCE} N·m)", flush=True)

    def _set_arm_armature(self):
        """Add the XM430-W350 servo's reflected gearbox inertia AND Coulomb (dry)
        friction to the arm joints so the plant matches the real hardware. Without
        the armature the wrist is ~20× too light and impedance diverges; without
        Coulomb friction the joint is an unphysical ideal torque source (a real
        Dynamixel has notable gearbox stiction). Viscous damping is set as the drive
        damping in _setup_arm_effort_mode."""
        joint_prims = self._get_joint_prims()
        for name in ARM_JOINT_NAMES:
            prim = joint_prims.get(name)
            if not prim:
                continue
            japi = PhysxSchema.PhysxJointAPI(prim)
            japi.CreateArmatureAttr().Set(float(ARM_ARMATURE))
            japi.CreateJointFrictionAttr().Set(float(ARM_COULOMB_FRICTION))
            print(f"[Demo] {name}: armature={ARM_ARMATURE:.4f} kg·m²  "
                  f"coulomb={ARM_COULOMB_FRICTION:.3f} N·m", flush=True)

    # ── Torque application ─────────────────────────────────────────────────

    def _apply_arm_efforts(self, tau_joint: np.ndarray):
        """Apply the commanded joint torques directly as DOF actuation efforts.

        Pure effort control — the torque is written straight to the articulation
        every step (clamped to ±ARM_MAX_FORCE). Zero torque simply means zero
        effort; a "lock" is just the controller commanding the holding torque.
        """
        # Startup sequence: hold the arm RIGID (stiff position drive) until the drone
        # has taken off and settled, then RELEASE it to effort control. This starts the
        # torque controller with the arm at its reference (small e_y) instead of already
        # drooped — no saturation slam. PLATFORM_TUNE keeps it locked forever.
        if not self._arm_released:
            if PLATFORM_TUNE:
                return
            # Release EXACTLY when the controller reports the arm channel is active
            # (5th message element). This makes the arm go free at the same instant the
            # holding torque (gravity comp) starts flowing, with the arm still at q≈0 —
            # no droop window from clock drift between the two processes.
            if self._torque_applier._arm_active_flag:
                self._setup_arm_effort_mode()                      # stiffness→0, viscous damping
                self._arm_released = True
                print(f"[Demo] Arm RELEASED to effort control "
                      f"(controller active, t={self.world.current_time:.1f}s)", flush=True)
            if not self._arm_released:
                return   # still position-held
        # ARM_TAU_SIGN: the model's joint-torque convention vs Isaac's joint axis. The log
        # proved a sign error (system diverges from EXACTLY zero error → positive feedback),
        # and the USD joints use a uniform body0=child/body1=parent convention, so the flip
        # is global. -1 negates the model torque into Isaac's convention. (At hover q≈0 the
        # joint-angle sign is irrelevant, so flipping only the torque is enough to test the
        # hold; if it holds, we also flip the published q/qd for trajectories.)
        for k, dof_idx in enumerate(self._arm_dof_indices):
            self._efforts[dof_idx] = float(np.clip(ARM_TAU_SIGN * tau_joint[k],
                                                   -ARM_MAX_FORCE, ARM_MAX_FORCE))
        self._dc.set_articulation_dof_efforts(self._artic, self._efforts)

        # Throttled debug print of the commanded arm efforts (~1 Hz at 60 Hz sim)
        self._eff_dbg = getattr(self, "_eff_dbg", 0) + 1
        if self._eff_dbg % 60 == 0:
            arm_efforts = [round(self._efforts[i], 4) for i in self._arm_dof_indices]
            j_pubs = (self._torque_applier._node.count_publishers(JOINT_TORQUE_TOPIC)
                      if self._torque_applier._node else -1)
            rotor_rx = self._rotor_applier._rx if self._rotor_applier is not None else "PX4"
            print(f"[Demo] efforts(arm)={arm_efforts} N·m  "
                  f"tau_joint={tau_joint.round(3)} N·m  "
                  f"jrx={self._torque_applier._rx} rrx={rotor_rx} "
                  f"j_pubs={j_pubs}", flush=True)

    # ── Main loop ──────────────────────────────────────────────────────────

    def _step(self):
        # Drain pending rotor + joint-torque messages through the shared executor
        # (services both subscriber nodes reliably; see __init__). Loop to drain the
        # queues since spin_once processes one ready callback at a time.
        if self._ros_exec is not None:
            for _ in range(16):
                self._ros_exec.spin_once(timeout_sec=0.0)
        else:
            if self._rotor_applier is not None:
                self._rotor_applier.spin_once()
            self._torque_applier.spin_once()
        tau = self._torque_applier.get_torques()

        # Publish arm joint states so the WSL controller can read q and q_dot
        all_states = self._dc.get_articulation_dof_states(self._artic, _dynamic_control.STATE_ALL)
        self._joint_state_pub.publish(all_states, self._arm_dof_indices)
        self._apply_arm_efforts(tau)

    def run(self):
        self.timeline.play()
        step = 0
        while simulation_app.is_running():
            self.world.step(render=True)
            self._step()
            step += 1
            if step % 60 == 0:
                # Print arm state every ~1 s at the 60 Hz render rate
                if self._artic:
                    states = self._dc.get_articulation_dof_states(
                        self._artic, _dynamic_control.STATE_ALL
                    )
                    q_arm = [math.degrees(float(states[i]['pos'])) for i in self._arm_dof_indices]
                    tau   = self._torque_applier.get_torques()
                    if PLATFORM_TUNE:
                        arm_mode = "POS-HOLD (PLATFORM_TUNE, no torque switch)"
                    elif self._arm_released:
                        arm_mode = "TORQUE (released)"
                    else:
                        arm_mode = "POS-HOLD (waiting for arm_active flag)"
                    print(
                        f"[Demo] arm[{arm_mode}] q={[round(v,1) for v in q_arm]}°  "
                        f"tau_cmd={tau.round(2)} N·m  "
                        f"jrx={self._torque_applier._rx} "
                        f"arm_active_flag={self._torque_applier._arm_active_flag}",
                        flush=True,
                    )

        self._torque_applier.destroy()
        self._joint_state_pub.destroy()
        if self._rotor_applier is not None:
            self._rotor_applier.destroy()
        self.timeline.stop()
        simulation_app.close()


def main():
    demo = GripperBatImpedanceDemo()
    demo.run()


if __name__ == "__main__":
    main()
