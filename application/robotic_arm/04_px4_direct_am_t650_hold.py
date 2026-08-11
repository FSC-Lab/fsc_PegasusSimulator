#!/usr/bin/env python
"""
04_px4_direct_am_t650_hold.py

Author: Shiqi Gao (shiqi.gao907@gmail.com)

AERIAL MANIPULATOR plant for the fsc_autopilot_ros2 T650 DIRECT-actuation
stack: the X-FORWARD aerial-manipulator asset (AM_xfwd.usda — AM_realign
re-authored by utils_model/make_x_forward_asset.py so the arm side is BODY +X
and the rotor channels present the PX4 Quad-X convention) flown as a
PX4-PRIMARY quadrotor with the **T650 motor and drone parameters** (MN4010 +
15x5" bench calibration, 2.95 kg body, rotor lag lambda = 10.0265 1/s), while
this process does exactly ONE thing in-process — hold the arm at its home pose
with joint torque commands.

    fsc_autopilot_ros2 node (T650 DIRECT law, params_..._am_t650.yaml)
        → fmu/in/actuator_motors → PX4 gate → HIL_ACTUATOR_CONTROLS
        → PX4MavlinkBackend (PRIMARY, T650 map: omega = u*665.99 + 64.06)
        → LaggedQuadraticThrustCurve (T650 k, c, lambda)  → AM_realign rotors
    THIS process (physics callback, 250 Hz):
        arm PD + gravity comp at Q_HOME = [0, 40, 40, 0] deg  → set_joint_efforts
    Pegasus ROS2 backend → /uav_0/state/{pose,twist,twist_inertial}
        → isaacsim_optitrack_ros2_emulator → estimator → controller feedback

There is NO whole-body law here, NO mixer, NO rotor publishing and NO flight
machine: PX4 (gated by the external fsc_autopilot node) owns the rotors from
the first step, exactly as in the bare-T650 DIRECT rig
(05_px4_single_drone_t650.py + start_t650_direct_actuator_sitl.sh). This is
the safe-fallback integration step for the whole-body controller: the proven
quadrotor-only DIRECT law flies the coupled airframe+arm plant while the arm
is parked, so the back-to-safety path exists before the whole-body law ever
enters fsc_autopilot_ros2.

WHY T650 PARAMETERS ON THE AM ASSET: T650 is the airframe the real aerial
manipulator will be built on, so the plant carries its motor calibration and
body mass now. The asset authors the X650 CAD body (2.4760795 kg); at
spawn /body is re-authored to t650_params.BODY_MASS (2.95 kg) and the T650
inertia on the LIVE stage (the 255 MB .usda is never touched), giving

    total = 2.95 (body) + 0.159548 (4 authored AM rotors) + 0.636624 (arm)
          = 3.746172 kg      → hover command ≈ 0.569, static T/W 2.71

The paired controller config (fsc_autopilot_ros2
config/params_single_drone_direct_actuation_am_t650.yaml) carries this exact
mass and the tangent-line thrust map re-derived about this hover point — if
the TOTAL printed by the mass override below ever disagrees with that yaml's
vehicle_mass, fix the yaml.

ARM HOLD: software joint-space PD + gravity comp through the true effort path
(same gains and structure as the flight-validated holds in 02/03:
KP 3.0, KD 0.25, slew 0.5 rad/s, clamp 3.0 N·m). The arm SPAWNS at home and
the hold runs unconditionally every physics step — on the ground, in SAFETY
hover, and in DIRECT. Gravity comp uses the shared control model
(utils_controller.controller: make_params + dynamics), NOT the whole-body law.

Run with:  scripts/indoor_sim/start_am_t650_direct_actuator_sitl.sh <config>
(pairs with fsc_autopilot_ros2 scripts/isaacsim/
start_direct_actuation_am_t650_stack.sh, which must be started FIRST — it
owns MicroXRCEAgent, and the SITL launcher refuses to run without it).
"""

import os
import math

import carb
from isaacsim import SimulationApp

# Indoor-launcher env conventions (same as 03/05_px4_single_drone_*):
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
# T650 motor + mass model — SINGLE SOURCE OF TRUTH, shared with the bare-T650
# scenarios (05_px4_single_drone_t650.py). MN4010 bench fit incl. the flight-C
# fit factors, and the 2.95 kg body / X650-CAD-derived inertia.
from fsc_aerial_manipulation.rotorcraft import t650_params
from fsc_aerial_manipulation.rotorcraft.lagged_thrust_curve import LaggedQuadraticThrustCurve
from fsc_aerial_manipulation.robotic_arm.utils_vehicle.x650_multirotor import (
    MultirotorMod, MultirotorConfig)
from fsc_aerial_manipulation.robotic_arm.utils_controller import controller as C

# ╔══════════════════════════════════════════════════════════════════════════╗
# ║  CONFIG                                                                  ║
# ╚══════════════════════════════════════════════════════════════════════════╝

# Vehicle USD — the X-FORWARD aerial-manipulator asset, generated from
# AM_realign.usda by utils_model/make_x_forward_asset.py (re-run that script if
# this file is missing; AM_realign itself is untouched and still serves every
# whole-body demo). In AM_xfwd the mechanical front (arm side) is BODY +X,
# matching the T650/X650 bare frames and PX4's x-forward assumption — this is
# what lets PX4's own SAFETY-mode mixer and the standard Quad-X allocation fly
# this vehicle at all. AM_realign's +y front / mirrored channel indexing
# negated the ROLL row of the standard allocation and flipped the vehicle on
# lift-off (diagnosed 2026-08-10, first flight of this rig).
import fsc_aerial_manipulation.rotorcraft as _fsc_rotorcraft
ASSETS_DIR = os.path.join(os.path.dirname(_fsc_rotorcraft.__file__), "assets")
USD_FILE   = os.path.join(ASSETS_DIR, "AM_xfwd.usda")
USD_PRIM_PATH     = "/gripper_bat"
BODY_PATH         = "/body"
# CHANNEL REMAP, load-bearing: in AM_xfwd the prim layout is rotor0 front-right
# (CCW), rotor1 rear-left (CCW), rotor2 REAR-RIGHT (CW), rotor3 FRONT-LEFT (CW).
# PX4 Quad-X channel order is FR, RL, FL, RR — so channels 2 and 3 map to prims
# rotor3 and rotor2 respectively. With this ordering the plant presents the
# EXACT x650_new.usd convention to PX4 and to the controller yaml's allocation.
ROTOR_PATHS       = ["/rotor0", "/rotor1", "/rotor3", "/rotor2"]
ROTOR_JOINT_NAMES = ["joint0", "joint1", "joint3", "joint2"]
ARM_JOINT_NAMES   = ["manip_joint1", "manip_joint2", "manip_joint3", "manip_joint4"]
GRIPPER_JOINT     = "gripper_joint"
GRIPPER_REST_DEG  = 0.0     # authored rest target (jaws open)

SPAWN_POS   = (0.0, 0.0, 0.5)   # pre-seat placement only; the ground seat overrides z
SPAWN_EULER = (0.0, 0.0, 0.0)
VEHICLE_ID  = 0
GROUND_BODY_Z = 0.305   # [m] measured resting body height on the legs (02/03)

# ── Arm home pose (user-specified for this integration step) ─────────────────
# q_home = [0, 40, 40, 0] deg. beta = q2+q3 = 80 deg — the folded, ground-safe,
# best-conditioned fold direction (at q = 0 the gripper reaches 0.282 m below
# the base, BELOW the legs at the 0.305 m resting height; folded, the elbow
# keeps ~0.14 m of clearance). Hardcoded — this rig has no planner dependency.
Q_HOME = np.radians([0.0, 40.0, 40.0, 0.0])

# ── T650 body mass + inertia override (RUNTIME ONLY — the .usda is untouched) ─
# AM_realign.usda authors /body at 2.4760795 kg (raw CAD frame, no batteries).
# Re-authored on the live stage to the T650 flight values. The rotors
# (4 x 0.039887, authored in AM_realign — NOT the 0.083921 kg of x650_new.usd)
# and the arm/gripper (0.636624 kg) add on top.
T650_BODY_MASS    = float(t650_params.BODY_MASS)          # 2.95 kg
T650_BODY_INERTIA = np.asarray(t650_params.INERTIA_DIAG)  # (0.05955, 0.06605, 0.06500)

# ── Arm hold (same numbers as the flight-validated 02/03 holds) ──────────────
ARM_HOLD_KP   = 3.0    # [N·m/rad]   wn = sqrt(3/0.02) ≈ 12 rad/s per joint
ARM_HOLD_KD   = 0.25   # [N·m·s/rad] zeta ≈ 0.5
ARM_HOLD_RATE = 0.5    # [rad/s] hold-target slew — a no-op when spawned at home,
                       # kept as protection for any spawn-pose variant
TAU_MAX       = 3.0    # [N·m] torque clamp (config/px4_direct_free.yaml's value)
ARM_ARMATURE  = 353.5 ** 2 * 1.6e-7   # ≈ 0.02 kg·m² reflected rotor inertia

# FRAME ADAPTER for the gravity comp. The shared control model (make_params /
# dynamics) describes the arm in AM_realign's OLD body frame (front on +y);
# AM_xfwd's content is that geometry rotated by Rz(-90) with the body frame
# kept. The same physical configuration therefore satisfies
#     R0_model = R0_actual @ Rz(-90),   v_model = Rz(-90)^T v_actual
# (exact for any attitude, since only the frame label changed). Joint angles
# and joint torques are scalars and carry over unchanged. Only g[6:] is
# consumed here, but the whole state is transformed for consistency.
R_MODEL = np.array([[0.0, 1.0, 0.0],
                    [-1.0, 0.0, 0.0],
                    [0.0, 0.0, 1.0]])   # Rz(-90): maps old +y (arm) onto +x

STATUS_PERIOD_S = 5.0   # one status line roughly this often (physics time)


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


class AmT650HoldSim:
    """AM_realign plant, T650 parameters, PX4-primary rotors, in-process arm hold."""

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

        # --- Spawn (PX4-PRIMARY: PX4MavlinkBackend is backend[0] = the sole
        # motor authority; the ROS2 backend publishes state only) ------------
        self.drone_path = self._spawn_am_px4_primary()

        self._wait_for_prim(self.drone_path, max_frames=300)
        # --- Model/physics fixes (same set as 02/03) ------------------------
        self._dedupe_physics_scenes()
        self._disable_self_collisions()
        self._disable_rotor_colliders()
        self._apply_t650_body_override()   # mass + inertia, pre-reset (live stage)
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
        print(f"[AM-T650] core Articulation API active; arm dof indices "
              f"{self._arm_idx}, gripper dof {self._grip_idx}", flush=True)

        # --- Control model (gravity comp only — no whole-body law here) -----
        # Mirror the /body override into the model so its printed totals match
        # the plant. The arm gravity torque g[6:] itself depends only on the
        # arm link masses and the attitude, but a consistent model costs nothing.
        self.params = C.make_params()
        if getattr(self, "_body_dm", 0.0):
            self.params["m_i"][0] += self._body_dm
            if self._body_dI is not None:
                self.params["I_i_i"][0] = self.params["I_i_i"][0] + np.diag(self._body_dI)
            print(f"[AM-T650] control model mirrored: m0="
                  f"{self.params['m_i'][0]:.6f} kg, m_total="
                  f"{sum(self.params['m_i']):.6f} kg (hover thrust "
                  f"{sum(self.params['m_i']) * self.params['g']:.2f} N)", flush=True)

        # --- Arm spawn at HOME + rigid re-seat + ground seat -----------------
        # (02/03's sequence, verbatim rationale: the joint write must happen
        # post-reset pre-stepping; PhysX roots this asset's tree at an ARM
        # link, so the teleport swings the BODY and must be undone by a rigid
        # re-seat; then the whole vehicle is seated on its legs.)
        beta_spawn = float(Q_HOME[1] + Q_HOME[2])
        if beta_spawn < math.radians(40.0):
            print(f"[AM-T650] WARNING: spawn fold beta = "
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
        print(f"[AM-T650] arm spawned at HOME q = "
              f"{np.degrees(Q_HOME).round(1)} deg (body re-seated: teleport "
              f"moved it {np.linalg.norm(pb - pa):.3f} m)", flush=True)

        pose_g = self._dc.get_rigid_body_pose(self._body)
        dz = GROUND_BODY_Z - float(pose_g.p.z)
        pr, qr = self._art.get_world_pose()
        self._art.set_world_pose(
            position=np.asarray(pr, float) + np.array([0.0, 0.0, dz]))
        print(f"[AM-T650] ground-seated: body z {pose_g.p.z:.3f} -> "
              f"{GROUND_BODY_Z:.3f} m (dz={dz:+.3f})", flush=True)

        # --- Arm actuation: TRUE effort control from the start ---------------
        self._set_arm_armature()
        actrl = self._art.get_articulation_controller()
        for i in self._arm_idx:
            actrl.switch_dof_control_mode(dof_index=i, mode="effort")
        print("[AM-T650] arm dofs in effort mode; hold "
              f"KP={ARM_HOLD_KP} KD={ARM_HOLD_KD} clamp={TAU_MAX} N·m — the "
              f"hold runs UNCONDITIONALLY (ground, SAFETY hover, DIRECT)",
              flush=True)

        self._hold_ref = None
        self._t = 0.0
        self._status_t = -1e9
        self.stop_sim = False
        carb.log_info("[AM-T650] ready")

    # ── spawn ───────────────────────────────────────────────────────────────

    def _spawn_am_px4_primary(self):
        """Spawn AM_realign with PX4 primary and the T650 motor calibration.

        Inline parallel of t650_bare_frame_utils.spawn_t650_with_mavlink, but on
        MultirotorMod (the AM asset's custom prim structure) — combined here so
        neither shared helper is modified. The u→omega map and the lagged thrust
        curve are EXACTLY the bare-T650 rig's; the paired controller yaml's
        thrust map inverts this same calibration.
        """
        quat_xyzw = Rotation.from_euler("XYZ", SPAWN_EULER, degrees=True).as_quat()

        input_scaling = float(t650_params.MAX_ROTOR_VEL - t650_params.ZERO_POSITION_ARMED)
        mavlink_config = PX4MavlinkBackendConfig({
            "vehicle_id": VEHICLE_ID,
            "connection_type": "tcpin",
            "connection_ip": "127.0.0.1",
            "connection_baseport": 4560,
            # DIRECT runs wall-clock (external DDS controller): the launcher
            # pushes PEGASUS_PX4_LOCKSTEP=0. Default 1 keeps stock behaviour if
            # this script is ever run standalone against a lockstep PX4.
            "enable_lockstep": PX4_LOCKSTEP,
            "px4_autolaunch": False,      # the launcher starts PX4 SITL itself
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
                "sub_control": False,     # PX4 owns the motors in this rig
            },
        )

        config = MultirotorConfig()
        # backend[0].input_reference() is the sole motor-command source — PX4
        # (via HIL_ACTUATOR_CONTROLS) must be backend[0].
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
        print(f"[AM-T650] spawned PX4-PRIMARY AM at {drone_prim_path}: MN4010 "
              f"k_f={t650_params.ROTOR_CONSTANT:.4e} "
              f"k_m={t650_params.ROLLING_MOMENT_COEFFICIENT:.4e} "
              f"lambda={t650_params.ROTOR_LAMBDA} "
              f"omega=[{t650_params.MIN_ROTOR_VEL}, {t650_params.MAX_ROTOR_VEL}] "
              f"map u->omega: x{input_scaling:.4f} "
              f"+{t650_params.ZERO_POSITION_ARMED} | lockstep="
              f"{'ON' if PX4_LOCKSTEP else 'OFF'}", flush=True)
        return drone_prim_path

    # ── physics/model fixes (02/03's, unchanged) ────────────────────────────

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
        carb.log_error(f"[AM-T650] prim never appeared: {prim_path}")
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
        print(f"[AM-T650] kept one PhysicsScene: {keep.GetPath()}", flush=True)

    def _disable_self_collisions(self):
        stage = omni.usd.get_context().get_stage()
        root = stage.GetPrimAtPath(self.drone_path)
        target = next((pr for pr in Usd.PrimRange(root)
                       if pr.HasAPI(UsdPhysics.ArticulationRootAPI)), root)
        PhysxSchema.PhysxArticulationAPI.Apply(target).CreateEnabledSelfCollisionsAttr().Set(False)
        print(f"[AM-T650] self-collision disabled on {target.GetPath()}", flush=True)

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
        print(f"[AM-T650] disabled {n} rotor collider(s).", flush=True)

    def _apply_t650_body_override(self):
        """Re-author /body's mass AND diagonal inertia to the T650 values on the
        LIVE stage (session-layer opinion over the referenced asset — the .usda
        on disk is never written; nothing here calls stage.Save()).

        Must run BEFORE world.reset(): PhysX snapshots mass properties when the
        articulation is created, so a later write is silently ignored.

        Records self._body_dm / self._body_dI so the control model (used only
        for the arm gravity comp here) can be mirrored, and PRINTS THE TOTAL —
        the number config/params_single_drone_direct_actuation_am_t650.yaml's
        vehicle_mass must equal.
        """
        self._body_dm = 0.0
        self._body_dI = None

        stage = omni.usd.get_context().get_stage()
        body_path = self.drone_path + BODY_PATH
        prim = stage.GetPrimAtPath(body_path)
        if not prim or not prim.IsValid():
            print(f"[AM-T650] WARNING: body prim {body_path} not found — "
                  f"T650 override SKIPPED", flush=True)
            return

        # every OTHER rigid body (4 rotors + arm links + gripper assembly)
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
            print(f"[AM-T650] body inertia {I_old.round(6)} -> "
                  f"{T650_BODY_INERTIA.round(6)} kg·m² (T650)", flush=True)

        print(f"[AM-T650] T650 MASS OVERRIDE: {body_path} {m_old:.6f} -> "
              f"{T650_BODY_MASS:.6f} kg (delta {self._body_dm:+.6f}); "
              f"other bodies {m_rest:.6f} kg; "
              f"TOTAL {m_old + m_rest:.6f} -> {T650_BODY_MASS + m_rest:.6f} kg "
              f"(weight {(T650_BODY_MASS + m_rest) * 9.81:.2f} N). "
              f"vehicle_mass in params_..._am_t650.yaml MUST equal this total. "
              f"The .usda is untouched.", flush=True)

    def _setup_gripper_drive(self):
        """Stiff position drive pinning the hub at its authored rest (jaws
        open) — the model's link-4 lump was extracted at this pose."""
        stage = omni.usd.get_context().get_stage()
        root = stage.GetPrimAtPath(self.drone_path)
        prim = next((p for p in Usd.PrimRange(root)
                     if p.GetName() == GRIPPER_JOINT and p.IsA(UsdPhysics.RevoluteJoint)), None)
        if prim is None:
            print(f"[AM-T650] WARNING: gripper joint '{GRIPPER_JOINT}' not found",
                  flush=True)
            return
        drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
        drive.GetStiffnessAttr().Set(1e6)
        drive.GetDampingAttr().Set(1e4)
        drive.GetMaxForceAttr().Set(1000.0)
        drive.GetTargetPositionAttr().Set(GRIPPER_REST_DEG)
        print(f"[AM-T650] gripper drive stiffened on {prim.GetPath()}", flush=True)

    def _arm_prims(self):
        root = self.stage.GetPrimAtPath(self.drone_path)
        prims = {p.GetName(): p for p in Usd.PrimRange(root)
                 if p.IsA(UsdPhysics.RevoluteJoint)}
        return [(n, prims[n]) for n in ARM_JOINT_NAMES if n in prims]

    def _set_arm_armature(self):
        for name, prim in self._arm_prims():
            PhysxSchema.PhysxJointAPI(prim).CreateArmatureAttr().Set(float(ARM_ARMATURE))
        print(f"[AM-T650] arm armature = {ARM_ARMATURE:.4f} kg·m²", flush=True)

    # ── control step (physics callback): the arm hold, nothing else ─────────

    def _control_step(self, step_size):
        try:
            self._control_step_inner(step_size)
        except Exception as exc:
            carb.log_error(f"[AM-T650] control step: {exc}")

    def _control_step_inner(self, dt):
        self._t += dt

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

        # gravity comp: arm rows of the model's gravity vector at the CURRENT
        # attitude (the whole point of using dynamics() instead of a constant).
        # The model lives in AM_realign's old body frame — adapt (see R_MODEL).
        R0_m = R0 @ R_MODEL
        v0_m = R_MODEL.T @ v0
        om_m = R_MODEL.T @ omega0
        X = np.concatenate([p0, R0_m.flatten(order="F"), q, v0_m, om_m, qdot])
        g_arm = C.dynamics(X, self.params)["g"][6:]

        # PD + gravity comp toward Q_HOME, slewed reference, clamped
        if self._hold_ref is None:
            self._hold_ref = np.asarray(q, float).copy()
        d_ref = np.clip(Q_HOME - self._hold_ref,
                        -ARM_HOLD_RATE * dt, ARM_HOLD_RATE * dt)
        self._hold_ref = self._hold_ref + d_ref
        tau = -ARM_HOLD_KP * (q - self._hold_ref) - ARM_HOLD_KD * qdot + g_arm
        tau = np.clip(tau, -TAU_MAX, TAU_MAX)
        self._art.set_joint_efforts(np.asarray(tau, float),
                                    joint_indices=self._arm_idx)

        if self._t - self._status_t >= STATUS_PERIOD_S:
            self._status_t = self._t
            # realized (lagged) rotor speed — 0 = disarmed, ~64 = armed idle,
            # ~443 = hover for this mass
            try:
                omega_real = np.asarray(self._thrust_curve.velocity, float)
                omega_txt = np.array2string(omega_real.round(0),
                                            separator=",")
            except Exception:
                omega_txt = "n/a"
            print(f"[AM-T650] t={self._t:7.1f}s  z={p0[2]:6.3f} m  "
                  f"|v|={np.linalg.norm(v0):5.2f} m/s  "
                  f"q_err={np.degrees(q - Q_HOME).round(1)} deg  "
                  f"|tau|max={np.abs(tau).max():4.2f} N·m  "
                  f"omega={omega_txt} rad/s", flush=True)

    # ── main loop ───────────────────────────────────────────────────────────

    def run(self):
        self.world.add_physics_callback("am_hold", self._control_step)
        # No START_PAUSED here: the ROS2 stack needs /uav_0/state/* flowing to
        # feed the mocap emulator/estimator, and nothing publishes while the
        # timeline is paused. PX4 (gated by the external node) owns the rotors,
        # so playing immediately just seats the vehicle at idle.
        self.timeline.play()
        steps = 0
        while (simulation_app.is_running()
               and not self.stop_sim
               and (not STEP_LIMIT or steps < STEP_LIMIT)):
            self.world.step(render=not HEADLESS)
            steps += 1
        carb.log_warn("[AM-T650] Simulation App is closing.")
        self.timeline.stop()
        simulation_app.close()


def main():
    AmT650HoldSim().run()


if __name__ == "__main__":
    main()
