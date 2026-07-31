#!/usr/bin/env python
"""
px4_direct_actuator_aerial_manipulator_gmo.py

Author: Ben Natra (send2ben123@gmail.com)
Author: Shiqi Gao (shiqi.gao907@gmail.com)

PX4 DIRECT-ACTUATOR parallel of 02_aerial_manipulator_gmo.py — same control
objective (takeoff -> hover/circle, EE impedance, GMO observer; the whole-body
law runs IN-PROCESS at the 250 Hz physics dt, unchanged), but the ROTOR path is
routed through the "direct thrust" pipeline proven on the bare-frame X650:

  GMO law (in-process) -> mixer -> omega[4]
    -> /uav_0/px4_bridge/rotor_omega                (std_msgs, this process)
    -> px4_direct_actuator_rotor_bridge.py          (external, system python)
       omega -> u = (omega - 81.8374)/735.7537 -> OffboardControlMode(direct_actuator)
       + ActuatorMotors -> uXRCE-DDS -> PX4 SITL    (arm/offboard GATE only)
    -> HIL_ACTUATOR_CONTROLS (MAVLink TCP 4560)
    -> PX4MavlinkBackend (PRIMARY backend, bench-calibrated normalized map)
    -> LaggedQuadraticThrustCurve                   (MN4014+15x5 bench: k_f, k_m,
                                                     first-order spin-up lambda=10.51)

Differences vs 02_aerial_manipulator_gmo.py (ONLY the rotor/plant path — the
control law, arm effort path, gripper, EE-force task, logging are identical):
  * spawn is PX4-PRIMARY (backend[0]=PX4MavlinkBackend, lockstep off); the ROS2
    backend publishes state only (sub_control=False).
  * thrust curve is the bench-calibrated LaggedQuadraticThrustCurve — the same
    motor delay model the bare-frame X650 scenarios use (X650_ROTOR_LAMBDA).
  * the mixer runs on the bench k_thrust/k_torque (must equal the plant curve).
  * rotor omega is PUBLISHED to the external bridge instead of written to
    backend.input_ref; px4_msgs stays OUT of the Isaac process (python-ABI safe).
  * the control clock HOLDS until the bridge confirms PX4 armed+OFFBOARD
    (engaged=True), so takeoff starts exactly when motor authority exists.

Run with:  scripts/start_px4_direct_actuator_aerial_manipulator_gmo.sh <config>
(PX4 SITL + MicroXRCEAgent + bridge are started by the launcher.)

controller_gmo.py itself is reused UNMODIFIED (library import), as are the
02_* in-process rigs and x650_rotorcraft_utils (this file spawns inline).

Original controller notes (unchanged): controller_gmo.py has NO joint-space
posture anchor and adds the GMO (estimates the transformed EE disturbance and
cancels it in thrust/base-moment, plus the arm channel in shaped-impedance
mode). Its make_params registers the arm joint chain correctly (joint k pivots
about manip_joint_k), so manip_joint3's gravity matches the plant.

TASK (EE_FORCE_ENABLE): takeoff → hover → apply a CONSTANT force at the
end-effector (default 1 N along inertial +y), mirroring main_sim.m's
'const_wrench' disturbance. The GMO estimates it (d_t_hat converges to the
applied force) and the controller rejects it (the vehicle holds position). Set
EE_FORCE_ENABLE=False for the plain trajectory (no disturbance).

Two-phase flight, selected by the MODE knob below (mirrors the MATLAB harness):
  Phase 1 TAKEOFF  climb the BODY to TAKEOFF_ALTITUDE (a CoM-reference ramp).
  Phase 2 TRACK    the MODE trajectory runs, re-anchored at the takeoff setpoint
                   (zero initial position/orientation error) with its own clock
                   (t=0 at handover).
The ARM is driven by the DESIGNED GMO control law in BOTH phases — there is no
joint-space PD in the loop (TAKEOFF_ARM_HOLD, default False). Set MODE = "hover"
(hold the takeoff point) or "circle" (nominal MATLAB circle).

Each physics step (world.add_physics_callback → _control_step):
  0. spin_once (bridge 'engaged' flag); HOLD (arm PD + omega=0) until PX4 is
     armed+OFFBOARD
  1. read body pose/twist (DC) + arm joint states (core Articulation API)
  2. build X, generate the takeoff / trajectory reference  (controller_gmo.py)
  3. dynamics() + MatlabController()  → thrust, tau_body, tau_joint  (+ GMO)
  4. mixer  → 4 rotor speeds → PUBLISH to the PX4 rotor bridge (NOT input_ref;
     motor authority is PX4 via HIL_ACTUATOR_CONTROLS → lagged thrust curve)
  5. tau_joint → art.set_joint_efforts (arm dofs in true effort mode);
     the designed GMO law throughout (legacy software PD hold only if
     TAKEOFF_ARM_HOLD is enabled)
  6. gripper_joint position target from the trajectory's grip flag

Run with:  scripts/start_px4_direct_actuator_aerial_manipulator_gmo.sh <config_name>
"""

import os
import sys
import math
import json

# ── Sweep / batch overrides ──────────────────────────────────────────────────
# utils/gain_sweep.py launches this demo repeatedly with the AM_SWEEP env var
# holding a JSON dict. Recognized keys:
#   headless (bool)         run without a window
#   t_end (s)               auto-stop + save after this sim time
#   log_path (str)          npz destination (default log/gmo_log.npz)
#   traj_type (str)         override controller TRAJ_TYPE
#   k_x k_v k_R k_w         scalars
#   K_y D_y                 4-lists (diagonal)
#   M_r_d                   3-list (diagonal)
#   K_o                     GMO observer gain (6+n=10-list, diagonal)
#   DLS_LAMBDA TAU_MAX      scalars
# A crashed run (on the ground / flipped after takeoff) ends early and the npz
# carries crashed=True.
SWEEP = json.loads(os.environ.get("AM_SWEEP", "{}"))

from isaacsim import SimulationApp
_HEADLESS = bool(SWEEP.get("headless", False))
simulation_app = SimulationApp({"headless": _HEADLESS})

# --- enable the PhysX debug / visualization UI so the physics debugger and its
#     frame/axis draw are available (fixes the greyed-out Utilities > Profilers &
#     Debuggers menu, and is what SHOW_PHYSICS_FRAMES below drives). Skipped when
#     headless — there is no viewport to draw into. Must run after SimulationApp. ---
if not _HEADLESS:
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
from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, Gf

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
for sub in ("utils", "geometric_controller"):
    p = os.path.join(SCRIPT_DIR, sub)
    if p not in sys.path:
        sys.path.insert(0, p)

from scipy.spatial.transform import Rotation

from pegasus.simulator.params import SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.logic.backends.px4_mavlink_backend import PX4MavlinkBackend, PX4MavlinkBackendConfig
from pegasus.simulator.logic.backends.ros2_backend import ROS2Backend

from fsc_aerial_manipulation.utils import add_dome_lighting
from fsc_aerial_manipulation.rotorcraft.x650_rotorcraft_utils import (
    print_mass_inertia_properties,
    print_rotor_positions,
)
# Bench-calibrated X650 motor/thrust constants — SINGLE SOURCE OF TRUTH, shared
# with every bare-frame X650 scenario ("we both use the x650 drone, so the
# thrust and motor parameters are the same").
from fsc_aerial_manipulation.rotorcraft.x650_bare_frame_utils import (
    X650_ROTOR_CONSTANT,
    X650_ROLLING_MOMENT_COEFFICIENT,
    X650_ZERO_POSITION_ARMED,
    X650_MIN_ROTOR_VEL,
    X650_MAX_ROTOR_VEL,
    X650_ROTOR_LAMBDA,
)
from fsc_aerial_manipulation.rotorcraft.lagged_thrust_curve import LaggedQuadraticThrustCurve
from fsc_aerial_manipulation.robotic_arm.x650_multirotor import MultirotorMod, MultirotorConfig
from fsc_aerial_manipulation.robotic_arm import controller_gmo as C   # GMO controller, kinematics re-registered

# Data logs (.npz) are written to a `log/` folder inside the controller package,
# parallel to plot_results.py's `images/` output dir (resolved from the installed
# package so it holds regardless of CWD / editable-install location).
LOG_DIR = os.path.join(os.path.dirname(C.__file__), "log")

# ╔══════════════════════════════════════════════════════════════════════════╗
# ║  CONFIG  (matches the ROS2 demo so the same fixed model is used)         ║
# ╚══════════════════════════════════════════════════════════════════════════╝

# ── Flight mode (the knob you switch) ─────────────────────────────────────────
# Selects what the vehicle does AFTER the takeoff phase. EVERY mode runs the same
# two-phase sequence, mirroring the MATLAB test harness:
#   Phase 1  TAKEOFF — climb to TAKEOFF_ALTITUDE at CLIMB_RATE with the arm
#            software-LOCKED (PD + gravity comp through the effort path); the
#            coupled impedance law is not applied while on/near the ground.
#   Phase 2  TRACK   — the arm switches to the full torque-control impedance law
#            and the selected trajectory runs. At the phase-1→2 handover the
#            trajectory is RE-ANCHORED at the current CoM / EE offset / heading,
#            so it STARTS at the takeoff setpoint with zero initial position AND
#            orientation error, and its clock starts at 0 (not sim t=0).
# Modes (map 1:1 onto controller_gmo.TRAJ_TYPE / TRAJ_CONFIG):
#   "hover"  — hold the takeoff setpoint. RECOMMENDED first GMO check: with the
#              posture anchor gone, confirm the arm holds on hover (GMO + task
#              impedance only) before moving to a tracking trajectory.
#   "circle" — track ONE rest-to-rest circle in the horizontal plane at the
#              takeoff altitude (radius/period from TRAJ_CONFIG["circle"],
#              default r=1.0 m, T=20 s). The base heading follows the tangent and
#              the EE tracks the compatible circle with all joint angles held at
#              zero — the nominal MATLAB circle test.
#   "circle_bent" — the same circle with the arm held at a better-conditioned
#              pose (q_hold = [0,0.8,0.8,0]; cond(J_3y) ~35 vs ~96 at q=0). With
#              the posture anchor removed the arm is now free in its null space,
#              so this is mainly a J_3y-conditioning stress test for the GMO law.
# Override per-run without editing this file: AM_SWEEP='{"traj_type":"circle"}'.
MODE = "hover"

# ── End-effector disturbance (TASK: takeoff → hover → constant EE force) ───────
# After the vehicle is hovering, apply a CONSTANT force at the end-effector (the
# gripper body) to exercise the disturbance observer — mirrors main_sim.m's
# 'const_wrench' mode. The GMO should ESTIMATE it (watch d_t_hat converge to the
# applied force in the log) and the controller REJECT it (the vehicle holds
# position instead of drifting off in the force direction). Best run with
# MODE="hover"; set EE_FORCE_ENABLE=False (and MODE="circle") for the plain circle.
EE_FORCE_ENABLE = False
EE_FORCE_WORLD  = np.array([0.0, 2.0, 0.0])   # [N] constant force, INERTIAL frame (+y = 1 N)
EE_FORCE_START  = 3.0                          # [s] into the hover (after takeoff→track handover)
EE_FORCE_BODY   = "/manip_base"                # gripper body it's applied to (= the EE link)
# Live force arrows (debug_draw): RED = applied EE force, CYAN = GMO estimate d_t_hat.
# PhysX's own debug visualizer shows colliders/CoM/contacts but NOT apply_body_force,
# so we draw the vectors ourselves. Degrades gracefully if debug_draw is unavailable.
EE_FORCE_VIZ    = False
FORCE_VIZ_SCALE = 0.15                          # [m per N] arrow length per newton

# Vehicle USD — resolved from the installed fsc_aerial_manipulation package so
# the path holds regardless of CWD (same asset the ROS2 demo and the paired
# 01_aerial_manipulator_inproc.py use).
import fsc_aerial_manipulation.rotorcraft as _fsc_rotorcraft
ASSETS_DIR = os.path.join(os.path.dirname(_fsc_rotorcraft.__file__), "assets")
USD_FILE   = os.path.join(ASSETS_DIR, "AM_realign.usda")
USD_PRIM_PATH    = "/gripper_bat"
BODY_PATH        = "/body"
ROTOR_PATHS      = ["/rotor0", "/rotor1", "/rotor2", "/rotor3"]
ROTOR_JOINT_NAMES = ["joint0", "joint1", "joint2", "joint3"]
ARM_JOINT_NAMES  = ["manip_joint1", "manip_joint2", "manip_joint3", "manip_joint4"]
# Arm state/effort I/O goes through the core Articulation API (tensor-backed):
# raw q/qdot/tau are the model coordinates directly — no sign corrections.

SPAWN_POS   = (0.0, 0.0, 0.5)
SPAWN_EULER = (0.0, 0.0, 0.0)
VEHICLE_ID  = 0

# ── Drone mass override (RUNTIME ONLY — AM_realign.usda is NEVER modified) ────
# AM_realign.usda authors the airframe (/body) at 2.4760795 kg, the raw CAD frame
# with no batteries; the bare-frame x650.usd carries 3.416 kg for the SAME airframe
# (CAD frame + the ~2 kg of batteries/electronics, see CLAUDE.md's X650 section).
# This knob re-authors the mass on the LIVE stage so both rigs fly the same
# physical vehicle without touching the (255 MB, gitignored, no-git-history) asset.
#
#   DRONE_MASS_OVERRIDE  None -> keep whatever the asset authors.
#   DRONE_MASS_TARGET    "body"  the value IS /body's mass; the rotors (4x0.039887)
#                                and the arm/gripper (0.636624) add on top, so the
#                                TOTAL becomes value + 0.796170 kg.
#                        "total" the value is the WHOLE vehicle mass; /body is
#                                back-solved as value - (rotors + arm).
# 3.416 kg on "body" => total 4.2122 kg (vs 3.2722 kg as authored). Hover then sits
# at ~477 rad/s/rotor (58% of the bench max, u~0.54), T/W ~2.9 — still ample margin.
DRONE_MASS_OVERRIDE = 3.416      # [kg]  None disables the override entirely
DRONE_MASS_TARGET   = "body"     # "body" | "total"

# Scale the airframe's diagonal inertia by the same ratio as its mass?
# Default False, following the X650 precedent (CLAUDE.md, "X650 CAD inertia
# correction"): the added battery mass sits near the CoM, so it changes the mass
# far more than the rotational inertia — mass-ratio scaling was tried there and
# was the WRONG move, reverted back to the raw CAD inertia. When True, the same
# absolute inertia delta is mirrored into the controller model (exact, since the
# model's I0 is body inertia + a fixed rotor parallel-axis term).
DRONE_MASS_SCALE_INERTIA = False

# --- PhysX debug frame visualization -----------------------------------------
# Draw the world origin frame and every rigid-body frame (RGB = XYZ) using the
# PhysX debug visualizer. Frames only render while the sim is PLAYING (and never
# in headless / sweep runs). Flip SHOW_PHYSICS_FRAMES to False to turn it all off.
SHOW_PHYSICS_FRAMES  = False   # PhysX debug visualizer (Utilities > Profilers & Debuggers > Physics)
SHOW_WORLD_AXES      = True    # world origin frame
SHOW_BODY_AXES       = True    # each rigid body's link frame
SHOW_BODY_MASS_AXES  = False   # each body's centre-of-mass frame
PHYSICS_FRAME_SCALE  = 10.0    # axis length multiplier (0 → invisible)

CLIMB_RATE  = 0.4   # [m/s] takeoff altitude ramp (matches controller node)

# Rotor model — the BENCH-CALIBRATED X650 constants (imported above from
# x650_bare_frame_utils, same values as every bare-frame X650 scenario). Used
# for BOTH the plant thrust curve and the controller mixer so commanded thrust
# equals realized thrust. rot_dir matches the AM_realign USD spin pairing.
ROTOR_DIR = [-1, -1, 1, 1]

# PX4 normalized-control map: omega = u*scaling + idle (PX4MavlinkBackend).
# The external rotor bridge INVERTS this exact map — its input_scaling /
# zero_position_armed ROS params must stay equal to these values.
PX4_INPUT_SCALING = X650_MAX_ROTOR_VEL - X650_ZERO_POSITION_ARMED   # 735.7537 rad/s

# ROS2 topics linking this process to px4_direct_actuator_rotor_bridge.py
# (std_msgs only — px4_msgs never enters the Isaac python).
BRIDGE_OMEGA_TOPIC   = "/uav_{vid}/px4_bridge/rotor_omega"
BRIDGE_ENGAGED_TOPIC = "/uav_{vid}/px4_bridge/engaged"

# Reflected rotor inertia — must equal controller_gmo.py make_params J_arm.
ARM_ARMATURE   = 353.5 ** 2 * 1.6e-7   # ≈ 0.02 kg·m²

# Safety clamps so a runaway control command can't blow up the physics / crash the
# sim. Normal operation is well inside these; they only cap pathological spikes.
# TAU_MAX sized to the joints: gravity scale is ~0.25 N·m, normal impedance ~0.5;
# the old 5.0 let a transient dump 250 rad/s² into a 0.02 kg·m² joint — with
# true effort control (no hidden drive damping) that flailed the arm at spawn
# and its reactions (>> the ~3.7 N·m the rotors can differential) tumbled the body.
TAU_MAX   = 1.5      # [N·m]   max arm joint torque (normal ~0.5)
# Rotor clamp = the bench-measured max (817.59 rad/s @100% throttle). With the
# bench k_thrust the AM hovers around ~420 rad/s, so headroom is ~3.8x in force.
OMEGA_MAX = X650_MAX_ROTOR_VEL   # [rad/s]

# TAKEOFF_ARM_HOLD — arm control during the takeoff CLIMB window.
#   False (DEFAULT): the DESIGNED GMO control law (res["tau_joint"]) drives the
#     arm for the WHOLE flight — there is NO joint-space PD anywhere in the loop,
#     and the disturbance observer is active from t=0. This is the "controller is
#     always my designed version with the observer" behavior.
#   True : legacy behavior — a software joint-space PD (+ gravity comp) holds the
#     arm during the climb, and the coupled law / GMO engage only at handover.
#     This is a SAFETY FALLBACK: the coupled law is only proven AIRBORNE (ground
#     contact is not in the model), so if the designed law flails the arm on the
#     ground at spawn (the drone rests on its legs at z=0.5), flip this to True.
# NOTE: the body (thrust + attitude) is ALWAYS on the designed law; this flag
# only affects the ARM channel during the climb. The climb ramp itself (a CoM
# reference that lifts the body to altitude) runs regardless.
TAKEOFF_ARM_HOLD = True

# Legacy software-hold PD gains — used ONLY when TAKEOFF_ARM_HOLD = True. Gentle
# PD-to-zero + gravity comp through the same effort path (drive gains are never
# switched mid-sim; mid-sim gain writes went stale in PhysX before).
ARM_HOLD_KP = 3.0    # [N·m/rad]   ωn = sqrt(3/0.02) ≈ 12 rad/s per joint
ARM_HOLD_KD = 0.25   # [N·m·s/rad] ζ ≈ 0.5

# ARM_ALWAYS_PD_HOLD — a THIRD arm-control choice on top of TAKEOFF_ARM_HOLD.
# Force the joint-space PD+gravity hold for the ENTIRE flight (not just takeoff):
#   False (DEFAULT): normal behavior — PD hold only during the climb (when
#     TAKEOFF_ARM_HOLD), then the designed task-space IMPEDANCE law once airborne.
#   True : the arm is ALWAYS driven by joint-space PD+gravity (holds q_ref, zeros
#     for most modes); the designed impedance / GMO arm channel NEVER engages
#     (self.ctrl.hold stays True → u3=0, GMO frozen). Fly with a "rigid" PD-held
#     arm — e.g. to isolate base/attitude behavior from the arm impedance law, or
#     as a fallback if the impedance law misbehaves in flight. The BODY (thrust +
#     attitude) still runs the FULL designed law, so the drone still flies its
#     trajectory; only the ARM stops using the impedance law.
# Uses the same ARM_HOLD_KP/KD gains. Override per-run: AM_SWEEP='{"ARM_ALWAYS_PD_HOLD":true}'.
# NOTE: with this True the EE disturbance force (EE_FORCE_ENABLE) is not applied
# (its gate is `not arm_hold`) — a PD-held arm isn't the GMO-rejection test.
ARM_ALWAYS_PD_HOLD = True

# Gripper: the fingers are actuated by ROTATING THE HUB via gripper_joint
# (manip_base ↔ hub revolute; the prismatic + lg_rev/rg_rev linkage converts hub
# rotation into finger travel). AM_realign re-zeroed the joint at the old open
# rest pose (the asset is saved with jaws open, limits ±50°), and flipped it to
# parent-first (body0=manip_base) — so the sign convention INVERTED vs the old
# asset: closing is now NEGATIVE hub rotation (old: rest −15.4° open, +35°
# closed = +50.4° of travel; new: 0° open, ≈−50° closed, right at the authored
# lower limit). Direction derived from the linkage geometry (hub pivots at
# ±y0.015 push the x-aligned links outward for +q) — VERIFY on the first run
# via the npz grip/grip_q traces + visual, flip GRIPPER_CLOSED to +50° if wrong.
GRIPPER_JOINT      = "gripper_joint"
GRIPPER_REST_DEG   = 0.0                              # authored rest target (open)
GRIPPER_OPEN       = math.radians(GRIPPER_REST_DEG)   # [rad] jaws open (= rest)
GRIPPER_CLOSED     = math.radians(-50.0)              # [rad] jaws closed (grasp)
GRIPPER_RATE       = 0.5                              # [rad/s] open/close ramp

# ── pick-and-place props ─────────────────────────────────────────────────────
# Spawned automatically when TRAJ_TYPE == "pickplace": a pedestal + graspable
# cube at the PICK grasp point and an empty pedestal at the PLACE point.
# Positions come from the same FK the trajectory uses, plus (a) the finger-pad
# midpoint (0.0494 m beyond the EE origin along its z, from the USD geometry)
# and (b) the MEASURED steady EE offset at the grasp pose (combo_pickplace run:
# e_y ≈ [−0.010, +0.044, +0.004] — the residual-CoM droop), so the cube sits
# between the open fingers when the gripper closes. The cube's world position
# is recorded in the npz ("obj_p") — the carry is measurable, not just visual.
PICKPLACE_PROPS = True
OBJ_SIZE   = 0.03    # [m] cube edge (pad gap ~7 cm open, ~3.5 cm closed)
OBJ_MASS   = 0.03    # [kg] deliberately unmodeled payload — part of the test
EE_SAG     = np.array([+0.0075, -0.0467, +0.0131])  # measured grasp offset [m]
                     # ^ measured from the 2026-07-14 AM_realign pickplace log:
                     # steady (r_e − r_ed) at grasp [+0.0089, −0.0442, +0.0042]
                     # PLUS the pad-vs-cube delta [−0.002, −0.0026, +0.009] from
                     # the arm settling at q≈−0.58 vs q_d=−0.498 (the posture/
                     # impedance equilibrium overshoots the reference, rotating
                     # the pad direction ~9° beyond the placement math). In that
                     # run the pads sat 9 mm above the cube center, caught only
                     # its top edge and dropped it on retract. Re-measure if
                     # gains or q_target change.
PAD_OFF_EE = np.array([0.0, 0.0, -0.0494])        # pad midpoint, EE frame [m]
STAND_XY   = 0.02    # [m] pedestal cross-section — MUST stay < OBJ_SIZE so the
                     # closing pads wrap the cube's faces, not the pedestal

# select the trajectory from the in-script MODE knob; AM_SWEEP's traj_type (batch
# runs) overrides it when present.
C.TRAJ_TYPE = str(SWEEP.get("traj_type", MODE))

# module-level sweep overrides (see AM_SWEEP block at the top; gain overrides
# on the controller instance are applied in InProcessDemoV2.__init__)
if "DLS_LAMBDA" in SWEEP:
    C.DLS_LAMBDA = float(SWEEP["DLS_LAMBDA"])
if "USE_GMO" in SWEEP:
    C.USE_GMO = bool(SWEEP["USE_GMO"])
if "M_Y" in SWEEP:            # shaped-impedance task inertia (4-list -> diagonal)
    C.M_Y = np.diag(np.asarray(SWEEP["M_Y"], float))
if "TAU_MAX" in SWEEP:
    TAU_MAX = float(SWEEP["TAU_MAX"])
if "ARM_ALWAYS_PD_HOLD" in SWEEP:   # fly with the arm on joint-space PD+gravity the whole time
    ARM_ALWAYS_PD_HOLD = bool(SWEEP["ARM_ALWAYS_PD_HOLD"])


class InProcessDemoV2:

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
        # motor authority; ROS2 backend publishes state only). Inline parallel
        # of x650_rotorcraft_utils.spawn_rotorcraft_with_mavlink(px4_primary=True)
        # — NOT a modification of that shared helper — so this scenario can
        # install LaggedQuadraticThrustCurve + the calibrated PX4 map without
        # touching the in-process rigs. ------------------------------------
        self.drone_path = self._spawn_am_px4_primary()

        self._wait_for_prim(self.drone_path, max_frames=300)
        # --- Model/physics fixes (same as the ROS2 demo) --------------------
        self._dedupe_physics_scenes()     # one PhysicsScene only
        self._disable_self_collisions()   # adjacent-link hulls overlap → thrash
        self._disable_rotor_colliders()   # props must not collide
        self._zero_arm_joint_states()     # start arm at 0, not the baked state
        self._apply_body_mass_override()  # DRONE_MASS_OVERRIDE (live stage, not the .usda)
        self._setup_gripper_drive()       # stiff position drive + OPEN target (bakes pre-play)
        self._obj = None
        self._spawn_pickplace_props()     # pedestals + graspable cube (pickplace only)
        self.world.reset()
        self.stage = omni.usd.get_context().get_stage()
        self._setup_physics_debug_viz()  # world/body frame axes (see SHOW_PHYSICS_FRAMES)

        print_mass_inertia_properties(self.stage, self.drone_path)
        print_rotor_positions(self.stage, self.drone_path, ROTOR_PATHS)

        # --- DC handles (body pose/twist only; the arm goes through the core
        # Articulation API below) -------------------------------------------
        self._dc     = _dynamic_control.acquire_dynamic_control_interface()
        self._body   = self._dc.get_rigid_body(self.drone_path + BODY_PATH)
        # gripper (EE) body handle for the optional end-effector disturbance force
        self._ee_body = self._dc.get_rigid_body(self.drone_path + EE_FORCE_BODY)
        self._ee_force_announced = False
        self._ee_force_now = np.zeros(3)
        self._ee_world = None                 # EE world position (stashed for the render-loop draw)
        self._last_d_t_hat = np.zeros(3)      # GMO translational estimate (for the cyan arrow)
        self._draw_ok = False
        if EE_FORCE_ENABLE:
            print(f"[InProc] EE disturbance armed: {EE_FORCE_WORLD} N (world) at "
                  f"'{EE_FORCE_BODY}', from {EE_FORCE_START:.1f}s into hover "
                  f"(handle {'ok' if self._ee_body else 'NOT FOUND'})", flush=True)
        # --- debug_draw interface for the live force arrows (optional) -------
        self._draw = None
        if EE_FORCE_VIZ:
            import importlib
            try:
                from omni.isaac.core.utils.extensions import enable_extension
            except Exception:
                enable_extension = None
            last = None
            # the debug_draw ext moved namespaces across Isaac versions; try both,
            # enabling the extension first in case it's present but not loaded.
            for ext in ("omni.isaac.debug_draw", "isaacsim.util.debug_draw"):
                try:
                    if enable_extension is not None:
                        try:
                            enable_extension(ext)
                        except Exception:
                            pass
                    dd = importlib.import_module(ext + "._debug_draw")
                    self._draw = dd.acquire_debug_draw_interface()
                    print(f"[InProc] force viz: {ext} acquired (RED=applied, CYAN=GMO est)", flush=True)
                    break
                except Exception as exc:
                    last = exc
            if self._draw is None:
                print(f"[InProc] force viz: no debug_draw module — arrows off ({last})", flush=True)

        # --- Core Articulation API for the ARM (required). dynamic_control is
        # deprecated and its dof layer is broken here (wrong velocities, silent
        # gain writes); the tensor-backed Articulation API is the supported path
        # for dof state reads, effort writes and runtime gain changes.
        from omni.isaac.core.articulations import Articulation
        self._art = Articulation(prim_path=self.drone_path)
        self._art.initialize()
        names = list(self._art.dof_names)
        self._arm_idx = [names.index(nm) for nm in ARM_JOINT_NAMES]
        self._grip_idx = names.index(GRIPPER_JOINT) if GRIPPER_JOINT in names else None
        print(f"[InProc] core Articulation API active; arm dof indices {self._arm_idx}"
              f", gripper dof {self._grip_idx}", flush=True)

        # --- ROS2 I/O to the external PX4 rotor bridge (std_msgs ONLY — the
        # px4_msgs python bindings are built for system python 3.10 and must
        # never be imported inside Isaac's python; the bridge owns them). ------
        import rclpy
        from std_msgs.msg import Bool, Float64MultiArray
        try:
            rclpy.init()
        except Exception:
            pass          # already initialised by the Pegasus ROS2 backend
        self._rclpy = rclpy
        self._Float64MultiArray = Float64MultiArray
        self._ros_node = rclpy.create_node(f"am_gmo_px4_tx_{VEHICLE_ID}")
        self._omega_pub = self._ros_node.create_publisher(
            Float64MultiArray, BRIDGE_OMEGA_TOPIC.format(vid=VEHICLE_ID), 10)
        self._engaged = False
        self._engaged_announced = False

        def _engaged_cb(msg):
            self._engaged = bool(msg.data)

        self._ros_node.create_subscription(
            Bool, BRIDGE_ENGAGED_TOPIC.format(vid=VEHICLE_ID), _engaged_cb, 10)
        print(f"[InProc] PX4 rotor-bridge I/O up: pub "
              f"{BRIDGE_OMEGA_TOPIC.format(vid=VEHICLE_ID)}, sub "
              f"{BRIDGE_ENGAGED_TOPIC.format(vid=VEHICLE_ID)} — control clock "
              f"HOLDS until PX4 is armed+OFFBOARD", flush=True)

        # --- Controller + mixer (in-process, controller_gmo.py) -----------------
        self.params = C.make_params()
        # Mirror the /body mass override into the CONTROL MODEL. make_params()
        # hardcodes m0 = /body + 4 rotors as authored in AM_realign.usda, so
        # without this the law's thrust feedforward (m·g, m = sum(m_i)) would be
        # short by delta·g — the GMO would have to absorb a ~9 N constant bias.
        # controller_gmo.py itself stays UNMODIFIED (it is shared with the ROS2
        # and 02_* rigs); only this instance's params dict is shifted.
        if getattr(self, "_body_dm", 0.0):
            self.params["m_i"][0] += self._body_dm
            if self._body_dI is not None:
                # I_i_i[0] = body inertia + a FIXED rotor parallel-axis term, so
                # adding the body's absolute inertia delta is exact (not a rescale).
                self.params["I_i_i"][0] = self.params["I_i_i"][0] + np.diag(self._body_dI)
            print(f"[InProc] control model mass shifted to match the plant: "
                  f"m0={self.params['m_i'][0]:.6f} kg, m_total="
                  f"{sum(self.params['m_i']):.6f} kg (hover thrust "
                  f"{sum(self.params['m_i']) * self.params['g']:.2f} N)", flush=True)
        self.ctrl   = C.MatlabController(self.params)
        # arm hold ONLY if the legacy takeoff PD is explicitly enabled; otherwise
        # the designed GMO law drives the arm from the first step (the control
        # loop re-asserts self.ctrl.hold every step, so this is just the seed).
        self.ctrl.hold = ARM_ALWAYS_PD_HOLD or TAKEOFF_ARM_HOLD
        print(f"[InProc] GMO controller: USE_GMO={C.USE_GMO} "
              f"impedance={C.IMPEDANCE_MODE} "
              f"M_Y={'natural' if C.M_Y is None else 'shaped'} "
              f"DLS_LAMBDA={C.DLS_LAMBDA} "
              f"TAKEOFF_ARM_HOLD={TAKEOFF_ARM_HOLD} "
              f"ARM_ALWAYS_PD_HOLD={ARM_ALWAYS_PD_HOLD}", flush=True)
        if ARM_ALWAYS_PD_HOLD:
            print("[InProc] ARM_ALWAYS_PD_HOLD=True — arm on joint-space PD+gravity "
                  "the WHOLE flight (impedance/GMO arm channel disabled); body still "
                  "flies the designed law", flush=True)
        # sweep gain overrides on the controller instance (AM_SWEEP)
        g = self.ctrl.g
        for k in ("k_x", "k_v", "k_R", "k_w"):
            if k in SWEEP:
                setattr(g, k, float(SWEEP[k]))
        for k in ("K_y", "D_y", "K_o"):
            if k in SWEEP:
                setattr(g, k, np.diag(np.asarray(SWEEP[k], float)))
        if "M_r_d" in SWEEP:
            g.M_r_d = np.diag(np.asarray(SWEEP["M_r_d"], float))
        if SWEEP:
            print(f"[InProc] AM_SWEEP overrides: {SWEEP}", flush=True)
        # auto-stop + crash bookkeeping (sweep runs)
        self._t_end = float(SWEEP["t_end"]) if "t_end" in SWEEP else None
        self._crashed = False
        self._done = False
        # Mixer on the BENCH-calibrated constants — MUST equal the plant's
        # LaggedQuadraticThrustCurve constants (else commanded thrust/yaw torque
        # != realized). rotor_positions/rot_dir keep their AM_realign defaults.
        self.mixer  = C.RotorMixer(C.RotorMixerParams(
            k_thrust=X650_ROTOR_CONSTANT,
            k_torque=X650_ROLLING_MOMENT_COEFFICIENT))

        # --- Arm actuation: TRUE effort control from the start --------------
        # switch_dof_control_mode("effort") zeros the drive gains — no hidden
        # drive damping (the model assumes an undamped joint; task damping comes
        # from D_y in the controller). The takeoff hold is a SOFTWARE hold
        # through this same effort path — drive gains are never touched mid-sim.
        self._set_arm_armature()
        actrl = self._art.get_articulation_controller()
        for i in self._arm_idx:
            actrl.switch_dof_control_mode(dof_index=i, mode="effort")
        print("[InProc] arm dofs in effort mode", flush=True)

        self._tr = None
        self._climb_z = 0.0
        self._t = 0.0
        self._climbing = True       # True during the takeoff climb window
        self._log = 0
        self._grip_cmd = GRIPPER_OPEN            # rate-limited gripper command [rad];
        self._grip_q = 0.0                       # starts at the authored rest (open)
        if not hasattr(self, "_grip_idx"):
            self._grip_idx = None
        # history buffers for MATLAB-style plotting (saved on exit → plot script)
        self._hist = {k: [] for k in
                      ("t", "p", "v0", "omega0", "e_R", "e_y", "q", "qdot",
                       "thrust", "tau_j", "u3",
                       "t_thrust", "t_imp", "t_dist", "t_cpl", "tau_b", "tau_b_arm",
                       "grip", "grip_q",
                       "x_c", "x_cd", "r_e", "r_ed", "q_d", "obj_p",
                       # GMO disturbance estimates (transformed frame) + applied EE force
                       "d_t_hat", "d_r_hat", "d_rho_hat", "ee_force")}
        carb.log_info("[InProc] ready")

    def _spawn_am_px4_primary(self):
        """Spawn the AM_realign X650+arm with PX4 as the PRIMARY backend and the
        bench-calibrated LAGGED thrust curve (the 'direct thrust' plant).

        Inline parallel of x650_rotorcraft_utils.spawn_rotorcraft_with_mavlink
        (px4_primary=True) + x650_bare_frame_utils.spawn_x650_with_mavlink's
        calibration — combined here so neither shared helper is modified.
        """
        quat_xyzw = Rotation.from_euler("XYZ", SPAWN_EULER, degrees=True).as_quat()

        mavlink_config = PX4MavlinkBackendConfig({
            "vehicle_id": VEHICLE_ID,
            "connection_type": "tcpin",
            "connection_ip": "127.0.0.1",
            "connection_baseport": 4560,
            # Wall-clock PX4 + wall-clock DDS controller stream: the same
            # no-lockstep configuration every bare-frame X650 offboard rig uses.
            "enable_lockstep": False,
            "px4_autolaunch": False,      # the launcher starts PX4 SITL itself
            "px4_dir": self.pg.px4_path,
            "px4_vehicle_model": self.pg.px4_default_airframe,
            # EXACT bench map: omega = u*scaling + idle. The external rotor
            # bridge inverts this map — keep its ROS params equal to these.
            "input_offset": [0.0] * 4,
            "input_scaling": [PX4_INPUT_SCALING] * 4,
            "zero_position_armed": [X650_ZERO_POSITION_ARMED] * 4,
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
        # backend[0].input_reference() is the sole motor-command source in
        # multirotor.py — PX4 (via HIL_ACTUATOR_CONTROLS) must be backend[0].
        config.backends = [PX4MavlinkBackend(mavlink_config), ros2_backend]
        # The "motor delay model": bench-calibrated quadratic curve + first-order
        # spin-up lag, identical constants to the bare-frame X650 scenarios.
        config.thrust_curve = LaggedQuadraticThrustCurve(config={
            "rotor_constant": [X650_ROTOR_CONSTANT] * 4,
            "rolling_moment_coefficient": [X650_ROLLING_MOMENT_COEFFICIENT] * 4,
            "min_rotor_velocity": [X650_MIN_ROTOR_VEL] * 4,
            "max_rotor_velocity": [X650_MAX_ROTOR_VEL] * 4,
            "rot_dir": list(ROTOR_DIR),
            "rotor_lambda": [X650_ROTOR_LAMBDA] * 4,
        })

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
        print(f"[InProc] spawned PX4-PRIMARY AM at {drone_prim_path}: "
              f"k_f={X650_ROTOR_CONSTANT:.4e} k_m={X650_ROLLING_MOMENT_COEFFICIENT:.4e} "
              f"lambda={X650_ROTOR_LAMBDA} omega=[{X650_MIN_ROTOR_VEL}, "
              f"{X650_MAX_ROTOR_VEL}] map u->omega: x{PX4_INPUT_SCALING:.4f} "
              f"+{X650_ZERO_POSITION_ARMED}", flush=True)
        return drone_prim_path

    def _publish_omega(self, omega):
        """Send the 4 desired rotor speeds [rad/s] to the external PX4 bridge."""
        msg = self._Float64MultiArray()
        msg.data = [float(w) for w in omega]
        self._omega_pub.publish(msg)

    def _record(self, p0, v0, omega0, q, qdot, tau_applied, res, ref, x_c, r_e):
        h = self._hist
        h["t"].append(self._t)
        h["p"].append(np.asarray(p0, float).copy())
        h["v0"].append(np.asarray(v0, float).copy())
        h["omega0"].append(np.asarray(omega0, float).copy())
        h["e_R"].append(np.asarray(res["e_R"], float).copy())
        h["e_y"].append(np.asarray(res["e_y"], float).copy())
        h["q"].append(np.asarray(q, float).copy())
        h["qdot"].append(np.asarray(qdot, float).copy())
        h["thrust"].append(float(res["thrust"]))
        h["tau_j"].append(np.asarray(tau_applied, float).copy())   # APPLIED torque (hold or task law)
        h["u3"].append(np.asarray(res["u3"], float).copy())
        h["tau_b"].append(np.asarray(res["tau_body"], float).copy())
        h["tau_b_arm"].append(np.asarray(res["tau_body_arm"], float).copy())
        h["grip"].append(float(getattr(self, "_grip_cmd", 0.0)))
        h["grip_q"].append(float(getattr(self, "_grip_q", 0.0)))
        # actual vs reference (CoM / EE / arm) — for the MATLAB-style plots
        h["x_c"].append(np.asarray(x_c, float).copy())
        h["x_cd"].append(np.asarray(ref["x_cd"], float).copy())
        h["r_e"].append(np.asarray(r_e, float).copy())
        h["r_ed"].append(np.asarray(ref["r_ed"], float).copy())
        h["q_d"].append(np.asarray(ref.get("q_d", np.zeros(4)), float).copy())
        # object world position (pickplace props) — proves/disproves the carry
        if self._obj is not None:
            try:
                pos, _ = self._obj.get_world_pose()
                h["obj_p"].append(np.asarray(pos, float).copy())
            except Exception:
                h["obj_p"].append(np.zeros(3))
        else:
            h["obj_p"].append(np.zeros(3))
        for k in ("t_thrust", "t_imp", "t_dist", "t_cpl"):
            h[k].append(np.asarray(res[k], float).copy())
        # GMO disturbance estimates (active from t=0 by default; zero only if
        # USE_GMO is off, or during the climb when TAKEOFF_ARM_HOLD is enabled)
        for k in ("d_t_hat", "d_r_hat", "d_rho_hat"):
            h[k].append(np.asarray(res[k], float).copy())
        # applied EE disturbance force (world frame; zero until it turns on)
        h["ee_force"].append(np.asarray(getattr(self, "_ee_force_now", np.zeros(3)), float).copy())

    def _save_history(self):
        h = self._hist
        if not h["t"]:
            return
        os.makedirs(LOG_DIR, exist_ok=True)
        path = SWEEP.get("log_path") or os.path.join(LOG_DIR, "gmo_px4_log.npz")
        np.savez(path, crashed=np.array(self._crashed),
                 **{k: np.array(v) for k, v in h.items()})
        plot_script = os.path.join(os.path.dirname(C.__file__), "plot_results.py")
        print(f"[InProc] saved {len(h['t'])} samples -> {path} "
              f"(crashed={self._crashed})\n"
              f"         plot with the Isaac python (ISAAC_PY), e.g.:\n"
              f"           ~/isaacsim/python_r_fsc.sh {plot_script} {path}", flush=True)

    # ── physics/model fixes (ported from the ROS2 demo) ─────────────────────

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
        if not SHOW_PHYSICS_FRAMES or _HEADLESS:
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

    def _wait_for_prim(self, prim_path, max_frames=300):
        stage = omni.usd.get_context().get_stage()
        for _ in range(max_frames):
            p = stage.GetPrimAtPath(prim_path)
            if p and p.IsValid():
                return p
            self.world.step(render=False)
        carb.log_error(f"[InProc] prim never appeared: {prim_path}")
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
        print(f"[InProc] kept one PhysicsScene: {keep.GetPath()}", flush=True)

    def _disable_self_collisions(self):
        stage = omni.usd.get_context().get_stage()
        root = stage.GetPrimAtPath(self.drone_path)
        target = next((pr for pr in Usd.PrimRange(root)
                       if pr.HasAPI(UsdPhysics.ArticulationRootAPI)), root)
        PhysxSchema.PhysxArticulationAPI.Apply(target).CreateEnabledSelfCollisionsAttr().Set(False)
        print(f"[InProc] self-collision disabled on {target.GetPath()}", flush=True)

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
        print(f"[InProc] disabled {n} rotor collider(s).", flush=True)

    def _zero_arm_joint_states(self):
        stage = omni.usd.get_context().get_stage()
        root = stage.GetPrimAtPath(self.drone_path)
        targets = set(ARM_JOINT_NAMES)
        n = 0
        for prim in Usd.PrimRange(root):
            if prim.GetName() in targets and prim.IsA(UsdPhysics.RevoluteJoint):
                for attr in ("state:angular:physics:position",
                             "state:angular:physics:velocity"):
                    a = prim.GetAttribute(attr)
                    if a and a.HasValue():
                        a.Set(0.0)
                n += 1
        print(f"[InProc] zeroed baked state on {n} arm joint(s).", flush=True)

    def _apply_body_mass_override(self):
        """Re-author the airframe (/body) mass on the LIVE stage per
        DRONE_MASS_OVERRIDE. The .usda on disk is NEVER touched.

        Safe because the asset is brought in as a USD *reference*
        (VehicleMod._init_prim references /gripper_bat), so this authors an
        opinion in the session/root layer that overrides — but does not edit —
        the referenced asset layer, and nothing in this script ever calls
        stage.Save()/Export().

        Must run BEFORE world.reset(): PhysX snapshots mass properties when the
        articulation is created, so a later write is silently ignored.

        Records the delta in self._body_dm / self._body_dI so the CONTROLLER's
        model (controller_gmo.make_params, which hardcodes the asset's authored
        values) can be shifted by the same amount — plant and model must agree
        or the hover feedforward is off by delta*g.
        """
        self._body_dm = 0.0
        self._body_dI = None
        if DRONE_MASS_OVERRIDE is None:
            return

        stage = omni.usd.get_context().get_stage()
        body_path = self.drone_path + BODY_PATH
        prim = stage.GetPrimAtPath(body_path)
        if not prim or not prim.IsValid():
            print(f"[InProc] WARNING: body prim {body_path} not found — "
                  f"mass override SKIPPED", flush=True)
            return

        # every OTHER rigid body (4 rotors + arm links + gripper assembly): needed
        # to turn a "total vehicle mass" request into a /body mass, and to report
        # the resulting total.
        m_rest = 0.0
        for p in Usd.PrimRange(stage.GetPrimAtPath(self.drone_path)):
            if p.GetPath() == prim.GetPath() or not p.HasAPI(UsdPhysics.RigidBodyAPI):
                continue
            a = UsdPhysics.MassAPI(p).GetMassAttr()
            if a and a.HasValue():
                m_rest += float(a.Get())

        mass_api = UsdPhysics.MassAPI.Apply(prim)   # idempotent (already applied)
        m_old = float(mass_api.GetMassAttr().Get())
        if DRONE_MASS_TARGET == "total":
            m_new = float(DRONE_MASS_OVERRIDE) - m_rest
            if m_new <= 0.0:
                print(f"[InProc] WARNING: requested total {DRONE_MASS_OVERRIDE:.4f} kg is "
                      f"below the non-body mass {m_rest:.6f} kg — override SKIPPED",
                      flush=True)
                return
        elif DRONE_MASS_TARGET == "body":
            m_new = float(DRONE_MASS_OVERRIDE)
        else:
            print(f"[InProc] WARNING: DRONE_MASS_TARGET='{DRONE_MASS_TARGET}' is not "
                  f"'body' or 'total' — mass override SKIPPED", flush=True)
            return

        mass_api.CreateMassAttr().Set(m_new)
        self._body_dm = m_new - m_old

        if DRONE_MASS_SCALE_INERTIA and m_old > 0.0:
            I_old = np.array(mass_api.GetDiagonalInertiaAttr().Get(), float)
            I_new = I_old * (m_new / m_old)
            mass_api.CreateDiagonalInertiaAttr().Set(Gf.Vec3f(*[float(x) for x in I_new]))
            self._body_dI = I_new - I_old
            print(f"[InProc] body inertia scaled x{m_new / m_old:.4f}: "
                  f"{I_old.round(6)} -> {I_new.round(6)} kg·m²", flush=True)

        print(f"[InProc] MASS OVERRIDE ({DRONE_MASS_TARGET}={DRONE_MASS_OVERRIDE} kg): "
              f"{body_path} {m_old:.6f} -> {m_new:.6f} kg "
              f"(delta {self._body_dm:+.6f}); other bodies {m_rest:.6f} kg; "
              f"TOTAL {m_old + m_rest:.6f} -> {m_new + m_rest:.6f} kg "
              f"(weight {(m_new + m_rest) * 9.81:.2f} N). The .usda is untouched.",
              flush=True)

    def _setup_gripper_drive(self):
        """Stiff position drive on gripper_joint, initial target OPEN. Runs before
        world.reset() so the gains bake into PhysX (USD target attr is in DEGREES;
        gains proven in gripper_drone_demo.py). Runtime targets go through the
        core Articulation API in _apply_gripper."""
        stage = omni.usd.get_context().get_stage()
        root = stage.GetPrimAtPath(self.drone_path)
        prim = next((p for p in Usd.PrimRange(root)
                     if p.GetName() == GRIPPER_JOINT and p.IsA(UsdPhysics.RevoluteJoint)), None)
        if prim is None:
            print(f"[InProc] WARNING: gripper joint '{GRIPPER_JOINT}' not found — "
                  f"grip commands will be ignored", flush=True)
            return
        drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
        drive.GetStiffnessAttr().Set(1e6)
        drive.GetDampingAttr().Set(1e4)
        drive.GetMaxForceAttr().Set(1000.0)
        # keep the AUTHORED rest target — no hub motion at spawn; the model's
        # link-4 lump was extracted at this pose
        drive.GetTargetPositionAttr().Set(GRIPPER_REST_DEG)
        print(f"[InProc] gripper drive stiffened on {prim.GetPath()} "
              f"(target = authored rest {GRIPPER_REST_DEG:.1f} deg)", flush=True)

    def _spawn_pickplace_props(self):
        """Pedestal + cube at the pick grasp point, empty pedestal at the place
        point (see the PICKPLACE_PROPS block for how positions are derived).
        Runs before world.reset() so physics picks the props up cleanly."""
        if not PICKPLACE_PROPS or C.TRAJ_TYPE != "pickplace":
            return
        try:
            from omni.isaac.core.objects import DynamicCuboid, FixedCuboid
            cfg = C.TRAJ_CONFIG["pickplace"]
            qT = np.array(cfg["q_target"], float)
            _, r_c0, _, _ = C._arm_fk(np.zeros(4))
            r_eT, r_cT, _, _ = C._arm_fk(qT)
            R_e = np.eye(3)
            # (self.params doesn't exist yet — this runs pre-reset; params are cheap)
            for h, qq in zip(C.make_params()["h_i_im1"], qT):
                R_e = R_e @ C.joint_rotation(h, qq)
            anchor = np.array([SPAWN_POS[0] + r_c0[0], SPAWN_POS[1] + r_c0[1],
                               C.TAKEOFF_ALTITUDE])
            grasp = {}
            for tag in ("pick", "place"):
                wp = np.array(cfg[f"{tag}_wp"], float)
                grasp[tag] = anchor + wp + (r_eT - r_cT) + R_e @ PAD_OFF_EE + EE_SAG
            for tag, g in grasp.items():
                h = float(g[2] - OBJ_SIZE / 2.0)
                FixedCuboid(prim_path=f"/World/{tag}_stand", name=f"{tag}_stand",
                            position=np.array([g[0], g[1], h / 2.0]),
                            scale=np.array([STAND_XY, STAND_XY, h]),
                            color=np.array([0.35, 0.35, 0.4]))
            self._obj = DynamicCuboid(
                prim_path="/World/pick_object", name="pick_object",
                position=grasp["pick"], scale=np.array([OBJ_SIZE] * 3),
                color=np.array([0.9, 0.15, 0.15]), mass=OBJ_MASS)
            try:
                from omni.isaac.core.materials import PhysicsMaterial
                mat = PhysicsMaterial("/World/Physics/obj_material",
                                      static_friction=1.2, dynamic_friction=1.0,
                                      restitution=0.0)
                self._obj.apply_physics_material(mat)
            except Exception as exc:
                print(f"[InProc] object friction material skipped ({exc})", flush=True)
            print(f"[InProc] pickplace props: cube at {np.round(grasp['pick'], 3)}, "
                  f"place stand at {np.round(grasp['place'], 3)}", flush=True)
        except Exception as exc:
            self._obj = None
            print(f"[InProc] WARNING: pickplace props failed ({exc}) — "
                  f"flying without objects", flush=True)

    def _draw_force_arrows(self):
        """Live debug_draw arrows from the EE: applied disturbance force (RED) and the
        GMO's translational estimate d_t_hat (CYAN). Called from the RENDER loop;
        cleared + redrawn each frame, so cyan grows to overlap red as the observer
        converges. Uses stashed self._ee_world / _ee_force_now / _last_d_t_hat."""
        ee = self._ee_world
        if ee is None:
            return
        base = tuple(np.asarray(ee, float))
        starts, ends, colors, widths = [], [], [], []
        for vec, color in ((self._ee_force_now,  (1.0, 0.2, 0.2, 1.0)),   # applied  = red
                           (self._last_d_t_hat,  (0.2, 0.9, 1.0, 1.0))):  # GMO est  = cyan
            v = np.asarray(vec, float)
            if v.shape != (3,) or np.linalg.norm(v) < 1e-4:
                continue
            tip = np.asarray(ee, float) + FORCE_VIZ_SCALE * v
            starts.append(base); ends.append(tuple(tip)); colors.append(color); widths.append(6.0)
        try:
            self._draw.clear_lines()
            if starts:
                self._draw.draw_lines(starts, ends, colors, widths)
                if not self._draw_ok:
                    self._draw_ok = True
                    print(f"[InProc] force viz: first arrow drawn ({len(starts)} line(s))", flush=True)
        except Exception as exc:
            print(f"[InProc] force viz draw FAILED ({exc}) — disabling arrows", flush=True)
            self._draw = None

    def _apply_gripper(self, grip_closed, dt):
        """Rate-limited gripper command from the trajectory's grip flag."""
        if self._grip_idx is None:
            return
        target = GRIPPER_CLOSED if grip_closed >= 0.5 else GRIPPER_OPEN
        step = max(-GRIPPER_RATE * dt, min(GRIPPER_RATE * dt, target - self._grip_cmd))
        self._grip_cmd += step
        from omni.isaac.core.utils.types import ArticulationAction
        self._art.get_articulation_controller().apply_action(ArticulationAction(
            joint_positions=np.array([self._grip_cmd]),
            joint_indices=np.array([self._grip_idx])))

    def _arm_prims(self):
        root = self.stage.GetPrimAtPath(self.drone_path)
        prims = {p.GetName(): p for p in Usd.PrimRange(root)
                 if p.IsA(UsdPhysics.RevoluteJoint)}
        return [(n, prims[n]) for n in ARM_JOINT_NAMES if n in prims]

    def _set_arm_armature(self):
        for name, prim in self._arm_prims():
            PhysxSchema.PhysxJointAPI(prim).CreateArmatureAttr().Set(float(ARM_ARMATURE))
        print(f"[InProc] arm armature = {ARM_ARMATURE:.4f} kg·m²", flush=True)

    # ── state ───────────────────────────────────────────────────────────────

    def _read_state(self):
        pose = self._dc.get_rigid_body_pose(self._body)
        lin  = self._dc.get_rigid_body_linear_velocity(self._body)
        ang  = self._dc.get_rigid_body_angular_velocity(self._body)
        p0 = np.array([pose.p.x, pose.p.y, pose.p.z])
        R0 = C.quat_to_rot(pose.r.w, pose.r.x, pose.r.y, pose.r.z)
        v0 = R0.T @ np.array([lin.x, lin.y, lin.z])     # world → body
        omega0 = R0.T @ np.array([ang.x, ang.y, ang.z]) # world → body
        q_all  = np.asarray(self._art.get_joint_positions(), float)
        qd_all = np.asarray(self._art.get_joint_velocities(), float)
        q    = q_all[self._arm_idx]
        qdot = qd_all[self._arm_idx]
        if self._grip_idx is not None:
            self._grip_q = float(q_all[self._grip_idx])   # measured hub angle
        return R0, p0, v0, omega0, q, qdot

    def _build_X(self, R0, p0, q, v0, omega0, qdot):
        return np.concatenate([p0, R0.flatten(order="F"), q, v0, omega0, qdot])

    # ── control step (physics callback) ─────────────────────────────────────
    # Registered via world.add_physics_callback → runs once per PHYSICS step
    # with the true step_size (world.step(render=True) advances ~4 physics
    # steps per call, so render-loop control would run 4x slow with a 4x-wrong dt).

    def _control_step(self, step_size):
        try:
            self._control_step_inner(step_size)
        except Exception as exc:
            carb.log_error(f"[InProc] control step: {exc}")

    def _control_step_inner(self, dt):
        R0, p0, v0, omega0, q, qdot = self._read_state()

        # Service the bridge subscription (engaged flag) — non-blocking.
        self._rclpy.spin_once(self._ros_node, timeout_sec=0.0)

        # HOLD until PX4 is armed + OFFBOARD (bridge publishes engaged=True):
        # keep the control clock at zero so the takeoff ramp starts the moment
        # motor authority actually exists (PX4 SITL boots ~15-25 s after Isaac).
        # Meanwhile: publish omega=0 (the bridge streams u=0 prestream setpoints
        # from it) and software-hold the arm exactly like the takeoff hold.
        if not self._engaged:
            self._tr = None                      # (re)anchor at engagement
            self._publish_omega(np.zeros(4))
            g_arm = C.dynamics(self._build_X(R0, p0, q, v0, omega0, qdot),
                               self.params)["g"][6:]
            tau_hold = np.clip(-ARM_HOLD_KP * q - ARM_HOLD_KD * qdot + g_arm,
                               -TAU_MAX, TAU_MAX)
            self._art.set_joint_efforts(np.asarray(tau_hold, float),
                                        joint_indices=self._arm_idx)
            return
        if not self._engaged_announced:
            self._engaged_announced = True
            print("[InProc] PX4 ENGAGED (armed+OFFBOARD) — starting takeoff clock",
                  flush=True)

        if self._tr is None:
            # first-pose setup (anchor the trajectory) — mirrors the ROS2 node
            X0   = self._build_X(R0, p0, q, v0, omega0, qdot)
            dyn0 = C.dynamics(X0, self.params)
            x_c  = p0 + R0 @ dyn0["r_0c_0"]
            self._tr = C.build_traj(np.array([x_c[0], x_c[1], C.TAKEOFF_ALTITUDE]),
                                    R0 @ (dyn0["r_0e_0"] - dyn0["r_0c_0"]),
                                    R0 @ np.array([1.0, 0, 0]))
            self._climb_z = x_c[2]
            self._t = 0.0
            print(f"[InProc] first p0={p0.round(3)} traj='{self._tr['type']}' "
                  f"climb {x_c[2]:.2f}->{C.TAKEOFF_ALTITUDE:.2f} m  dt={dt:.4f}",
                  flush=True)
            return

        self._t += dt
        # sweep bookkeeping: auto-stop at t_end; flag + stop on a crash
        # (on the ground after the spin-up, or flipped past ~78 deg)
        if self._t_end is not None and self._t >= self._t_end:
            self._done = True
        if self._t > 1.0 and (p0[2] < 0.05 or R0[2, 2] < 0.2):
            if not self._crashed:
                print(f"[InProc] CRASH detected at t={self._t:.2f} "
                      f"(z={p0[2]:.3f}, R33={R0[2,2]:.2f})", flush=True)
            self._crashed = True
            self._done = True
        # Two-phase flight. The BODY always climbs to altitude then runs the
        # trajectory (a CoM reference — NOT a joint command). The ARM is driven
        # by the DESIGNED GMO law the whole time; self.ctrl.hold (which zeros u3
        # and freezes the GMO) is asserted ONLY if the legacy takeoff PD is
        # explicitly enabled via TAKEOFF_ARM_HOLD (a ground-contact fallback).
        climbing = self._t < C.TAKEOFF_TIME
        # ARM_ALWAYS_PD_HOLD forces the joint-space PD hold for the whole flight;
        # otherwise it's the takeoff-climb-only hold (TAKEOFF_ARM_HOLD).
        arm_hold = ARM_ALWAYS_PD_HOLD or (TAKEOFF_ARM_HOLD and climbing)
        self.ctrl.hold = arm_hold
        if self._climbing and not climbing:
            # RE-ANCHOR once, at the climb→trajectory handover. The integrator-
            # free loops hover with a steady tilt+offset (e_R~0.03 roll → thrust
            # tilts → the position loop settles ~0.1 m off in y, and e_y inherits
            # it). Anchoring here starts e_x=e_y=0; the vehicle holds its actual
            # equilibrium instead of the spawn xy.
            self._climbing = False
            dyn_a = C.dynamics(self._build_X(R0, p0, q, v0, omega0, qdot), self.params)
            x_c_now = p0 + R0 @ dyn_a["r_0c_0"]
            self._tr = C.build_traj(x_c_now, R0 @ (dyn_a["r_0e_0"] - dyn_a["r_0c_0"]),
                                    R0 @ np.array([1.0, 0, 0]))
            print(f"[InProc] climb done (t={self._t:.1f}s) → re-anchored ref at "
                  f"x_c={x_c_now.round(3)}", flush=True)

        if climbing:
            dz = C.TAKEOFF_ALTITUDE - self._climb_z
            step = max(-CLIMB_RATE * dt, min(CLIMB_RATE * dt, dz))
            self._climb_z += step
            vz = step / dt if dt > 0 else 0.0
            z3 = np.zeros(3)
            ref = {"x_cd": np.array([self._tr["x_c0"][0], self._tr["x_c0"][1], self._climb_z]),
                   "x_cd_dot": np.array([0.0, 0.0, vz]), "x_cd_ddot": z3,
                   "x_cd_d3": z3, "x_cd_d4": z3}
            ref = C._fixed_ee(ref, self._tr)
        else:
            ref = C.generate_reference(self._t - C.TAKEOFF_TIME, self._tr)

        X   = self._build_X(R0, p0, q, v0, omega0, qdot)
        dyn = C.dynamics(X, self.params)
        res = self.ctrl(X, dyn, ref, dt)

        # rotors → PX4 direct-actuator path: publish desired omega to the bridge
        # (bridge: u=(omega-idle)/scaling → ActuatorMotors → PX4 gate →
        # HIL_ACTUATOR_CONTROLS → PX4MavlinkBackend → lagged thrust curve).
        # Clamped to the bench max so the wire signal matches what the plant
        # can actually do (PX4 additionally clips u to [0,1]).
        omega = np.clip(self.mixer.mix(res["thrust"], res["tau_body"]), 0.0, OMEGA_MAX)
        self._publish_omega(omega)

        # arm efforts EVERY step. Default: the DESIGNED GMO law drives the arm
        # (res["tau_joint"]) the whole flight — no joint-space PD. The legacy
        # software PD hold runs ONLY when TAKEOFF_ARM_HOLD is enabled, during the
        # climb (arm_hold). One actuation path, one clamp.
        if arm_hold:
            g_arm = dyn["g"][6:]
            # hold to the trajectory's q_hold (zeros for every mode except
            # circle_bent) so the arm reaches the commanded pose BEFORE the
            # trajectory starts → zero initial joint/EE error at the handover,
            # and circle_bent starts already at its non-singular pose.
            q_hold = self._tr.get("q_hold", None)
            q_ref = q_hold if q_hold is not None else np.zeros_like(q)
            tau_j = -ARM_HOLD_KP * (q - q_ref) - ARM_HOLD_KD * qdot + g_arm
        else:
            tau_j = res["tau_joint"]
        tau_j = np.clip(tau_j, -TAU_MAX, TAU_MAX)
        self._art.set_joint_efforts(np.asarray(tau_j, float),
                                    joint_indices=self._arm_idx)

        # gripper: trajectory grip flag. Refs without one (takeoff/hover) default
        # to OPEN = the authored rest pose → the hub never moves until the
        # trajectory commands the grasp.
        self._apply_gripper(ref.get("grip", 0.0), dt)

        # --- End-effector disturbance force (the TASK): a constant WORLD-frame
        # force on the gripper, re-applied EVERY step (apply_body_force is one-shot),
        # once we are hovering. Converted to the body's LOCAL frame — the codebase's
        # apply_body_force convention is global=False (see vehicle.py / slung-load).
        self._ee_force_now = np.zeros(3)
        if (EE_FORCE_ENABLE and self._ee_body and not arm_hold
                and (self._t - C.TAKEOFF_TIME) >= EE_FORCE_START):
            # commanded force -> drawn (red arrow) even if the physical apply fails,
            # so we can tell "not commanded" apart from "commanded but apply failed".
            self._ee_force_now = EE_FORCE_WORLD.copy()
            try:
                pe = self._dc.get_rigid_body_pose(self._ee_body)
                R_ee = C.quat_to_rot(pe.r.w, pe.r.x, pe.r.y, pe.r.z)
                f_local = R_ee.T @ EE_FORCE_WORLD            # world -> gripper local
                self._dc.apply_body_force(self._ee_body,
                                          carb._carb.Float3(f_local.tolist()),
                                          carb._carb.Float3([0.0, 0.0, 0.0]), False)
                if not self._ee_force_announced:
                    self._ee_force_announced = True
                    print(f"[InProc] EE force ON at t={self._t:.1f}s: "
                          f"{EE_FORCE_WORLD} N (world) — watch d_t_hat converge to it",
                          flush=True)
            except Exception as exc:
                if not self._ee_force_announced:
                    self._ee_force_announced = True
                    print(f"[InProc] EE apply_body_force FAILED ({exc}) — force NOT applied "
                          f"(articulation-link force issue?)", flush=True)

        # stash the arrow inputs; the actual debug_draw is done in run() (render loop —
        # debug_draw from the physics callback often doesn't reach the viewport).
        self._ee_world = p0 + R0 @ dyn["r_0e_0"]
        self._last_d_t_hat = np.asarray(res["d_t_hat"], float).copy()

        self._record(p0, v0, omega0, q, qdot, tau_j, res, ref,
                     x_c=p0 + R0 @ dyn["r_0c_0"], r_e=p0 + R0 @ dyn["r_0e_0"])

        self._log += 1
        if self._log >= 250:
            self._log = 0
            drive = (res["t_thrust"] + res["t_cpl"]).round(3)     # pushes arm off
            correct = (res["t_imp"] + res["t_dist"]).round(3)      # fights to fix it
            print(f"[InProc] t={self._t:5.1f} z={p0[2]:.3f} thr={res['thrust']:.1f} "
                  f"e_R={res['e_R'].round(3)}\n"
                  f"          q={q.round(3)} qdot={qdot.round(3)} "
                  f"tau_j={tau_j.round(3)}\n"
                  f"          u2={np.asarray(res['u2']).round(3)} "
                  f"u3={np.asarray(res['u3']).round(3)}\n"
                  f"          tau_body={res['tau_body'].round(3)} "
                  f"N1ᵀu3={res['tau_body_arm'].round(3)} omega0={omega0.round(3)}\n"
                  f"          arm err e_y={res['e_y'].round(3)}  "
                  f"drive={drive}  correct={correct}\n"
                  f"          GMO[{'on' if res['gmo_active'] else 'off'}] "
                  f"d_t={np.asarray(res['d_t_hat']).round(3)} "
                  f"d_r={np.asarray(res['d_r_hat']).round(3)} "
                  f"d_rho={np.asarray(res['d_rho_hat']).round(3)}\n"
                  f"          EE_force(world)={self._ee_force_now.round(3)} "
                  f"(d_t should converge to it)\n"
                  f"          q_d={np.asarray(ref.get('q_d', np.zeros(4))).round(3)} "
                  f"grip cmd={math.degrees(self._grip_cmd):.1f}deg "
                  f"meas={math.degrees(self._grip_q):.1f}deg", flush=True)

    # ── main loop ───────────────────────────────────────────────────────────

    def run(self):
        self.world.add_physics_callback("am_control", self._control_step)
        # self.timeline.pause()
        self.timeline.play()
        render = not bool(SWEEP.get("headless", False))   # headless: skip rendering
        while simulation_app.is_running() and not self._done:
            self.world.step(render=render)   # control runs inside the physics steps
            if render and self._draw is not None:   # draw force arrows in the RENDER context
                self._draw_force_arrows()
        self._save_history()
        self.timeline.stop()
        simulation_app.close()


def main():
    InProcessDemoV2().run()


if __name__ == "__main__":
    main()
