#!/usr/bin/env python
"""
02_aerial_manipulator_free.py

Author: Shiqi Gao (shiqi.gao907@gmail.com)

FREE-FLYING, IN-PROCESS aerial-manipulator demo running controller.py (the
COUPLED feedback-linearizing law + generalized-momentum disturbance observer,
parameterized by robotic_arm/config/free.yaml)
inside a physics-step callback — no ROS2 round-trip, true 250 Hz control at the
physics dt.

"free" = the rotors are driven DIRECTLY from this law (backend.input_ref every
physics step): no autopilot in the loop, no arming/offboard gate, and IDEAL
INSTANTANEOUS rotors (QuadraticThrustCurve — no spin-up lag). The PX4-gated
sibling 03_px4_direct_aerial_manipulator_free.py runs the same control law
but routes omega through PX4 and uses the bench-calibrated LaggedQuadratic-
ThrustCurve (lambda = 10.51 1/s), so results between the two are NOT directly
comparable on actuator-limited maneuvers.

Differs from 01_aerial_manipulator_track.py only in the controller it drives:
controller.py has NO joint-space posture anchor and adds the GMO (estimates
the transformed EE disturbance and cancels it in thrust/base-moment, plus the
arm channel in shaped-impedance mode). Its make_params registers the arm joint
chain correctly (joint k pivots about manip_joint_k), so manip_joint3's gravity
matches the plant.

SCOPE — this script is the FREE-FLIGHT TRAJECTORY-TRACKING rig and nothing else:
takeoff → track the MODE trajectory → (for the showcase) land. It never touches
its environment. Everything involving CONTACT or an applied external wrench
lives in its own main+controller pair and must stay there:
  02_aerial_manipulator_pick.py + config/pick.yaml   grasp / carry / place
  02_aerial_manipulator_push.py + config/push.yaml   box push, and the
      constant-EE-wrench ('const_wrench') GMO disturbance-rejection test
The GMO itself is still fully active here and its estimates (d_t_hat, d_r_hat,
d_rho_hat) are logged every step — what was removed is only the ability to
APPLY an external force, not the observer that would see one.

Three-phase flight, selected by the MODE knob below — the same workflow a real
experiment uses (takeoff and landing are position SETPOINTS; only the middle
phase tracks a plan; every switch is gated on the measured hover error, never
on the clock):
  Phase 1 TAKEOFF  x_cd = the task trajectory's own start point (ONE setpoint,
                   read from the anchored plan — no climb ramp). Arm PD-held at
                   home. Ends when the hover gate clears at that point.
  Phase 2 TASK     the MODE trajectory runs, re-anchored at the actual hover
                   pose (zero initial position/orientation error), clock t=0 at
                   the handover. Arm on the whole-body law, GMO on (u3 consumes
                   d_e_hat, so observer and law switch together).
  Phase 3 LANDING  (landing modes only) x_cd = the task end point with z at
                   LAND_SP_COM_Z (ONE setpoint — no descent profile). Arm back
                   on the PD hold. Once settled there the rotors ramp off and
                   the app saves the npz and closes itself.
A rendered run starts PAUSED so the initial settings can be inspected — press
PLAY in the GUI to fly (START_PAUSED below).

Each physics step (world.add_physics_callback → _control_step):
  1. read body pose/twist (DC) + arm joint states (core Articulation API)
  2. build X, generate the takeoff / trajectory reference  (utils_planner)
  3. dynamics() + MatlabController()  → thrust, tau_body, tau_joint  (+ GMO)
  4. mixer  → 4 rotor speeds → backend.input_ref
  5. tau_joint → art.set_joint_efforts (arm dofs in true effort mode);
     the designed GMO law throughout (legacy software PD hold only if
     TAKEOFF_ARM_HOLD is enabled)

Run with:  scripts/start_aerial_manipulator_free.sh <config_name>
(no WSL controller needed)
"""

import os
import sys
import math
import json

# ── Sweep / batch overrides ──────────────────────────────────────────────────
# utils/px4_gmo_gain_sweep.py launches this demo repeatedly with the AM_SWEEP env
# var holding a JSON dict. Recognized keys (all optional):
#   headless (bool)         run without a window
#   t_end (s)               auto-stop + save after this sim time
#   log_path (str)          npz destination (default log/gmo_log.npz)
#   traj_type (str)         override controller TRAJ_TYPE (the MODE knob)
#   k_x k_v k_R k_w         scalars   (geometric position/attitude gains)
#   K_y D_y                 4-lists   (diagonal, EE impedance stiffness/damping)
#   M_r_d                   3-list    (diagonal, desired translational inertia)
#   M_Y                     4-list    (diagonal, shaped task inertia)
#   K_o                     6+n=10-list (diagonal, GMO observer gain)
#   USE_GMO (bool)          disable the observer for an A/B run
#   DLS_LAMBDA TAU_MAX      scalars
# A crashed run (on the ground / flipped after takeoff) ends early and the npz
# carries crashed=True.
SWEEP = json.loads(os.environ.get("AM_SWEEP", "{}"))

from isaacsim import SimulationApp
_HEADLESS = bool(SWEEP.get("headless", False))
simulation_app = SimulationApp({"headless": _HEADLESS})

# --- PhysX debug frame visualization -----------------------------------------
# Draw the world origin frame and every rigid-body frame (RGB = XYZ) using the
# PhysX debug visualizer. Frames only render while the sim is PLAYING (and never
# in headless / sweep runs). Flip SHOW_PHYSICS_FRAMES to False to turn it all off.
# Declared HERE, above the rest of the config, because the extensions it needs
# must be enabled immediately after SimulationApp and before the omni imports.
SHOW_PHYSICS_FRAMES  = False   # PhysX debug visualizer (Utilities > Profilers & Debuggers > Physics)
SHOW_WORLD_AXES      = True    # world origin frame
SHOW_BODY_AXES       = True    # each rigid body's link frame
SHOW_BODY_MASS_AXES  = False   # each body's centre-of-mass frame
PHYSICS_FRAME_SCALE  = 10.0    # axis length multiplier (0 → invisible)

# Enabling omni.physx.ui is what un-greys Utilities > Profilers & Debuggers and
# makes the frame draw available. Gated on SHOW_PHYSICS_FRAMES so a normal run
# does not pay the extension-load cost for a debugger it will not open; skipped
# headless too (no viewport to draw into). Must run after SimulationApp.
if SHOW_PHYSICS_FRAMES and not _HEADLESS:
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
for sub in ("utils", "geometric_controller"):
    p = os.path.join(SCRIPT_DIR, sub)
    if p not in sys.path:
        sys.path.insert(0, p)

from pegasus.simulator.params import SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.logic.vehicle_manager import VehicleManager

from fsc_aerial_manipulation.utils import (add_dome_lighting,
                                           draw_flight_volume, volume_report,
                                           draw_pose_marker)
from fsc_aerial_manipulation.rotorcraft.x650_rotorcraft_utils import (
    spawn_rotorcraft_with_mavlink,
    print_mass_inertia_properties,
    print_rotor_positions,
)
from fsc_aerial_manipulation.robotic_arm.utils_controller import controller as C        # model + control law (one law, every scenario)
from fsc_aerial_manipulation.robotic_arm.utils_controller import control_params as CP   # gains: robotic_arm/config/free.yaml
from fsc_aerial_manipulation.robotic_arm import utils_planner as P     # ALL desired trajectories

# A run's outputs all live under ONE root, robotic_arm/results/ (resolved from
# the installed package, so it holds regardless of CWD / editable-install
# location):  results/log/ for the .npz this demo writes, results/figures/ for
# the .png utils_plot/plot_results.py renders from it.
import fsc_aerial_manipulation.robotic_arm as _ra
_RA_DIR = os.path.dirname(_ra.__file__)   # robotic_arm/, NOT the
                                          # controller's own folder
LOG_DIR = os.path.join(_RA_DIR, "results", "log")

# ╔══════════════════════════════════════════════════════════════════════════╗
# ║  CONFIG  (matches the ROS2 demo so the same fixed model is used)         ║
# ╚══════════════════════════════════════════════════════════════════════════╝

# ── Flight mode (the knob you switch) ─────────────────────────────────────────
# Selects what the vehicle does in the middle TASK phase. EVERY mode runs the
# same three-phase sequence (see the FLIGHT PHASES block below for the gates):
#   Phase 1  TAKEOFF — fly to ONE setpoint (the task trajectory's start point,
#            read from the anchored plan) with the arm software-LOCKED (PD +
#            gravity comp through the effort path); the coupled impedance law
#            is not applied while on/near the ground. No climb ramp — the
#            setpoint step IS the takeoff, exactly like a real flight.
#   Phase 2  TASK    — the arm switches to the full torque-control impedance law
#            and the selected trajectory runs. At the phase-1→2 handover the
#            trajectory is RE-ANCHORED at the current CoM / EE offset / heading,
#            so it STARTS at the takeoff setpoint with zero initial position AND
#            orientation error, and its clock starts at 0 (not sim t=0).
#   Phase 3  LANDING — (landing modes only) ONE setpoint at the task end point,
#            z dropped to LAND_SP_COM_Z; arm back on the PD hold; rotors ramp
#            off once settled and the app closes itself.
# Modes (map 1:1 onto utils_planner.TRAJ_CONFIG — see that package for the
# reference math; NOTHING in this file builds a reference dict). NAMING RULE
# (2026-08-08): every mode is `<shape>_<who>` — <shape> the flown path (hover /
# circle / figure8 / poly), <who> = "drone" (arm LOCKED at one pose, carried
# rigidly — only the base flies the shape) or "whole" (base AND arm move
# together). Old names: hover→hover_drone, circle→circle_drone, circle_bent→
# circle_drone + q_hold knob, compatible_showcase→poly_whole, compatible→gone.
#
#   ── arm locked (…_drone) ──
#   "hover_drone"   — hold the takeoff setpoint. RECOMMENDED first GMO check:
#              with the posture anchor gone, confirm the arm holds on hover
#              (GMO + task impedance only) before moving to a tracking mode.
#   "circle_drone"  — ONE rest-to-rest circle in the horizontal plane at the
#              takeoff altitude (r=1.0 m, T=20 s), heading on the tangent, arm
#              held still. Optional "q_hold" catalogue knob = the old
#              circle_bent (bent better-conditioned pose, cond(J_3y) ~35 vs
#              ~96 at q=0 — a J_3y-conditioning stress test for the GMO law).
#   "figure8_drone" — ONE rest-to-rest figure-8 (Gerono lemniscate, A=1.2 m
#              half-length, ±0.35 m lateral, T=24 s), crossing point at the
#              anchor, heading on the tangent — a richer heading/accel test
#              than the circle (the yaw rate REVERSES at the crossing).
#   "poly_drone"    — the poly_whole showcase WAYPOINTS flown by the base
#              alone (min-snap segments fly-in → yaw-cycle hold → fly-out,
#              arm locked at the folded home pose, LANDS at the end). The
#              direct A/B against poly_whole: with the arm locked, the
#              mid-phase platform yaw SWINGS the EE around instead of the
#              arm keeping it pinned.
#   ── whole body (…_whole) ──
#   "hover_whole"   — showcase_drone_gimbal: the base HOVERS at the takeoff
#              setpoint while a moving EE command sweeps the arm (fold + arm
#              yaw in quadrature → the EE traces a closed loop under the
#              stationary drone). Dynamically EXACT: arm motion is internal
#              and cannot move the system CoM — the drone is the gimbal,
#              holding still while its tool does the work.
#   "circle_whole"  — the circle_drone path while the arm FOLDS/UNFOLDS
#              (beta 50..80°, two full min-snap-phased cycles, wrist 0) — the
#              EE reference is the FK-exact swept offset carried with the
#              base yaw (arm_sweep.py).
#   "figure8_whole" — same sweep over the figure-8 path.
#   "poly_whole"    — showcase_end_effector_gimbal (ex "compatible_showcase"):
#              the 4-PHASE WHOLE-BODY DEMO FLIGHT on the offline
#              dynamically-compatible plan (Python port of MATLAB
#              plan_compatible_trajectory.m — EE task + heading PRESCRIBED,
#              CoM SOLVED by polynomial+Picard, planned once and cached, a
#              planning summary prints at startup). Arm SPAWNS folded at home
#              (Q_SPAWN = q_hold = [0, 40°, 46°, 0], beta = 86°), held there
#              through the climb → fly-in while UNFOLDING → PINNED phase (EE
#              position and heading exactly fixed while the platform yaws and
#              the arm folds/unfolds — the drone repositions around its
#              frozen tool tip) → fly-out while FOLDING back, ending in a
#              hover hold above the landing spot → setpoint LANDING. Knobs in
#              utils_planner.TRAJ_CONFIG["poly_whole"].
# Every mode's tr["q_hold"] (the takeoff-hold arm pose) doubles as Q_SPAWN;
# keep TAKEOFF_ARM_HOLD = True for any mode whose q_hold is non-zero.
#
# THREE ways to choose the trajectory, highest precedence first — pick whichever
# suits the run; none of them needs the others:
#   1. AM_SWEEP='{"traj_type":"circle_drone"}'   batch/sweep runs, JSON, also
#                                                carries gains and headless
#   2. AM_MODE=circle_drone                      ONE name, nothing else — the
#      ./scripts/start_aerial_manipulator_free.sh <config> circle_drone
#                                                launcher's optional 2nd
#                                                argument just sets AM_MODE
#   3. MODE below                                the default when neither is set
# A name that is not in the catalogue fails immediately at startup (before Isaac
# spends a minute loading the stage) and prints the full list of valid names.
MODE = "poly_whole"

# Start PAUSED (rendered runs only): the scene is fully built — vehicle seated
# on the ground, arm at the folded home pose (Q_SPAWN), trajectory anchored,
# waypoint markers drawn — and physics is frozen until PLAY is pressed in the
# GUI, so the initial settings can be inspected first. One silent physics step
# runs before the pause (see run()): the arm pose and ground-seat live in the
# physics state and would otherwise not reach the renderer — the paused view
# would show the authored midair, q=0 asset instead of the true initial state.
# Press PLAY to fly. Do NOT press STOP: stop→play runs a world reset, which
# restores PhysX's reset snapshot (captured at q = 0) and silently undoes the
# spawn-pose teleport — the same gotcha the Q_SPAWN block below documents.
# Headless / sweep runs have no PLAY button and start immediately.
START_PAUSED = True

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

# ── Indoor flight / mocap volume ─────────────────────────────────────────────
# Wireframe box marking the REAL room, so a trajectory can be checked against it
# here before anyone flies it on hardware. Purely visual (BasisCurves, no
# collider) — the vehicle passes straight through it; it does not constrain the
# controller. FSC indoor mocap field: 4.5 x 4.5 x 2.0 m.
#   CENTER is the box's xy centre. The vehicle SPAWNS at SPAWN_POS, so a box
#   centred on the origin extends +/-2.25 m around the takeoff point — set the
#   centre to wherever the room origin actually sits relative to takeoff.
# At startup the demo prints a volume_report() for the flown trajectory,
# INFLATED by ROTOR_RADIUS: the rotor disc, not the body origin, is what clips
# the ceiling and the walls.
SHOW_FLIGHT_VOLUME   = True
FLIGHT_VOLUME_SIZE   = (4.5, 4.5, 2.0)   # [m] (x, y, ceiling height)
FLIGHT_VOLUME_CENTER = (0.0, 2.0)        # [m] xy centre (see the fit report)
FLIGHT_VOLUME_FLOOR  = 0.0               # [m] floor height
ROTOR_RADIUS         = 0.25              # [m] half-span used to inflate the check

# ── Trajectory waypoint markers ──────────────────────────────────────────────
# Wireframe crosses (+ a heading ray) at the three points that define the
# showcase, drawn in the SAME colours the plots use for those phases so the
# viewport and the figures read as one system:
# Each marker is drawn the way the EE command is actually structured: a POINT
# for the 3-D position and an ARROW for the 1-D yaw — the only two things the
# end-effector task controls.
#   fly-in start  BLUE  where the trajectory begins after the takeoff handover
#   gimbal point  RED   the EE pose held frozen through the whole pinned phase
#   fly-out end   BLUE  where the transit ends, above the landing spot
# Purely visual (BasisCurves, no collider). They are placed from the ANCHORED
# trajectory, so they are drawn/refreshed when it is built and again at the
# climb->track re-anchor, and authored from the RENDER loop rather than the
# physics callback (authoring USD prims from a physics step is not safe).
SHOW_WAYPOINT_MARKERS = True
WP_SIZE      = 0.22       # [m] arrow length (the dot scales with it)
WP_COL_START = (0.15, 0.40, 0.95)   # fly-in start  — BLUE
WP_COL_PIN   = (0.95, 0.15, 0.15)   # gimbal point  — RED (the highlight)
WP_COL_END   = (0.15, 0.40, 0.95)   # fly-out end   — BLUE
# Arrow yaw offset [deg]. The EE task's yaw reference is b1_de = the EE's
# X-AXIS, which on this asset sits 90 deg off the MECHANICAL FRONT (body +y) —
# the same frame quirk that path_yaw_deg exists for. Drawing the raw b1_de puts
# the arrow along world +x while the vehicle faces and travels along +y, which
# reads as wrong. The offset rotates only the DRAWN arrow about z, so it still
# turns exactly with the commanded yaw (it is the same 1-D DOF, just referenced
# to the front instead of the x-axis).
# TEMPORARY: set back to 0.0 together with path_yaw_deg once the USDA is
# re-authored with the mechanical front on body +x.
WP_ARROW_YAW_OFFSET_DEG = 90.0

SPAWN_POS   = (0.0, 0.0, 0.5)
SPAWN_EULER = (0.0, 0.0, 0.0)
VEHICLE_ID  = 0
# The vehicle is SEATED ON THE GROUND at init (takeoff starts from the legs,
# like a real flight, not from midair): after the arm teleport below, the whole
# vehicle is translated vertically so the measured BODY height equals the
# resting height from the 2026-08-06 landing run. SPAWN_POS's z is therefore
# only the authored pre-seat placement; the seat overrides it. Done by
# measurement (not by editing SPAWN_POS) because the articulation root and the
# body link are different frames on this asset — the root sits ~0.11 m below
# the body — and the resting height is known for the BODY.
GROUND_BODY_Z = 0.305   # [m] measured resting body height on the legs

# Rotor model — must match controller.py RotorMixerParams and the spawn thrust curve.
ROTOR_K_THRUST = 1.03e-5
ROTOR_K_TORQUE = 1.0e-6
ROTOR_DIR      = [-1, -1, 1, 1]

# Reflected rotor inertia — must equal controller.py make_params J_arm.
ARM_ARMATURE   = 353.5 ** 2 * 1.6e-7   # ≈ 0.02 kg·m²

# Safety clamp so a runaway rotor command can't blow up the physics / crash the
# sim. Normal operation is well inside it; it only caps pathological spikes.
# The ARM's torque limit is NOT here any more — tau_max is a control parameter
# and lives in robotic_arm/config/free.yaml, so the law's clamp and the demo's
# clamp are one number (resolved into TAU_MAX below, after the config loads).
# It is sized to the joints: gravity scale is ~0.25 N·m, normal impedance ~0.5;
# the old 5.0 let a transient dump 250 rad/s² into a 0.02 kg·m² joint — with
# true effort control (no hidden drive damping) that flailed the arm at spawn
# and its reactions (>> the ~3.7 N·m the rotors can differential) tumbled the body.
OMEGA_MAX = 2000.0   # [rad/s] max rotor speed (hover ~850)

# TAKEOFF_ARM_HOLD — arm control during the TAKEOFF phase.
#   True (DEFAULT, the flight/real-experiment configuration): a software
#     joint-space PD (+ gravity comp) holds the arm at home through the takeoff,
#     and the coupled law / GMO engage together at the hover-gated handover. The
#     coupled law is only proven AIRBORNE — ground contact is not in the model,
#     and on the ground its phantom F_trans (the legs' reaction is missing from
#     the model) leaks into the arm through the coupling terms.
#   False: the DESIGNED GMO control law (res["tau_joint"]) drives the arm from
#     t=0 — no joint-space PD anywhere, observer active from the first step.
#     Only for airborne-start style debugging; known to flail on the ground.
# NOTE: the body (thrust + attitude) is ALWAYS on the designed law; this flag
# only affects the ARM channel during takeoff. The takeoff setpoint itself
# (the body flying to the task start point) runs regardless.
TAKEOFF_ARM_HOLD = True

# Legacy software-hold PD gains — used ONLY when TAKEOFF_ARM_HOLD = True. Gentle
# PD-to-zero + gravity comp through the same effort path (drive gains are never
# switched mid-sim; mid-sim gain writes went stale in PhysX before).
ARM_HOLD_KP = 3.0    # [N·m/rad]   ωn = sqrt(3/0.02) ≈ 12 rad/s per joint
ARM_HOLD_KD = 0.25   # [N·m·s/rad] ζ ≈ 0.5
# The hold TARGET is rate-limited toward q_hold instead of stepped: with the
# folded home at q2=q3=40° the raw 0→40° spawn step made the ζ≈0.5 PD overshoot
# and bang q2 into its +50° hard stop (measured: q2 hit 50.0° at t=0.35 s).
# 0.5 rad/s reaches home in ~1.4 s, well inside the 4 s climb. Since the arm
# now SPAWNS at the hold target (Q_SPAWN), the slew is a no-op during a normal
# takeoff — it is kept as protection for the hold's re-engagement paths (the
# landing re-seed, and any run whose spawn pose differs from q_hold).
ARM_HOLD_RATE = 0.5  # [rad/s] hold-target slew rate

# ── FLIGHT PHASES: setpoint takeoff → task → setpoint landing ────────────────
# Both phase switches are gated on the measured HOVER ERROR at the phase
# boundary point — the takeoff→task handover at the trajectory's start point,
# the task→landing switch at its end point — and on NOTHING else. There is no
# time floor and no timeout: the system flies the next phase exactly when it
# has demonstrably arrived, the same criterion a real experiment uses. (While
# a gate is pending the live errors print once a second, so a tolerance that
# can never clear is visible instead of silent.)
#
# The gate is 3-D CoM position error + body speed (+ arm-on-target at takeoff).
# HOVER_POS_TOL must sit ABOVE the loops' steady lateral offset: with no
# integrator anywhere, the vehicle hovers with a steady tilt that parks the CoM
# ~0.1 m off in xy. That offset is exactly what the task-start RE-ANCHOR
# absorbs (the trajectory starts from the actual hover pose, zero initial
# error); tighten the tolerance below ~0.12 m and the gate never clears.
#
# The arm-hold ⇄ whole-body-law switch rides the same gates, and the GMO engages
# WITH the law, not after it: u3 consumes d_e_hat, so a law running on a frozen
# observer would track without its disturbance term. The observer is held reset
# (p_hat = p, d_e_hat = 0) whenever the hold owns the arm, which is exactly the
# clean-start state at each engagement.
# SEQUENCING RULE — change ONE thing at a time. The arm's torque-source switch
# always happens at a CONSTANT reference, and a reference STEP is only commanded
# once the arm is already on its new source. This, not any torque smoothing, is
# what prevents the arm slam: the first version dropped the landing reference
# 1.17 m in the same step that handed the arm back, so the whole-body law (still
# owning the arm) saw thrust cut out, F_trans = u1·R0e3 − mg·e3 ≈ −32 N flooded
# u3 through the coupling feedforward, and the demanded arm torque hit 90 N·m —
# 22x TAU_MAX, saturating two joints. MEASURED once the switch is sequenced
# properly, the arm torque is CONTINUOUS across it with no blending at all:
# at a settled hover with e_y = 0, everything at rest and F_trans = 0, the law's
# u3 is ~0 and its arm torque reduces to the same gravity term the PD hold
# applies — step 0.005 N·m at takeoff, 0.000 at landing (0.1% of TAU_MAX,
# 0.2 rad/s^2). So the switch is a plain assignment; there is no cross-fade.
HOVER_POS_TOL  = 0.15  # [m]   3-D CoM error to the phase setpoint
HOVER_V_TOL    = 0.20  # [m/s] body speed
HOVER_Q_TOL    = 0.05  # [rad] arm within ~3° of its hold target (takeoff gate)
HOVER_DWELL    = 2.0   # [s]   gate must hold CONTINUOUSLY this long to fire —
                       #       a grazing pass through the tolerance ball (low
                       #       speed at the edge of the ball for one step) is
                       #       not "arrived"; the dwell resets on any violation

# ── Landing (setpoint phase, landing modes only) ─────────────────────────────
# Ground contact is NOT in the control model — the coupled law is only proven
# AIRBORNE, which is exactly why TAKEOFF_ARM_HOLD exists for the climb. Touchdown
# is the same situation in reverse, so the LANDING phase (entered at the task's
# end point, latched — once in, never back to the law) holds the arm on the PD
# and freezes the GMO for the whole descent.
# The setpoint parks the CoM ~4 cm ABOVE the measured resting height (CoM
# 0.290 m on the legs), so the position loop never commands the reference into
# the ground; once the vehicle settles there (same hover gate), the rotors RAMP
# OFF over LAND_DISARM_TIME and gravity takes the last centimetres. After the
# ramp the app waits CLOSE_DELAY, saves the npz, and closes itself.
LAND_SP_COM_Z    = 0.33   # [m] landing-setpoint CoM height (resting CoM = 0.290)
LAND_DISARM_TIME = 2.0    # [s] rotor ramp-to-zero once settled at the setpoint
CLOSE_DELAY      = 2.0    # [s] settle time after the ramp before save-and-exit

# ── Terminal output: two-panel status frame ──────────────────────────────────
# One self-contained frame printed at PANEL_PERIOD (and immediately on every
# phase event), NOT a cursor-addressed TUI: the launcher pipes stdout through
# `tee`, and Isaac/carb write their own lines into the same stream, so anything
# that repositions the cursor would be shredded. A plain block of lines survives
# both the pipe and the log file.
#   LEFT  the flight-phase log — one line per event, colour-tagged by phase
#   RIGHT the live state, one row per group (pos / att / arm / err / ctrl / dist)
# Colour is ON by default and NOT gated on isatty(): stdout is a pipe here (the
# tee), so an isatty() check would silently strip colour from the terminal too.
# Set AM_NO_COLOR=1 to get a plain-text log; sweeps/headless disable it anyway.
PANEL_ENABLE  = True
PANEL_PERIOD  = 1.0    # [s] state refresh (phase events render a frame at once)
# Widths are sized to the CONTENT, not guessed: the widest state row is the
# ARM/joint and CTRL/arm lines (4 values = 58 chars) and the longest event
# label is 24. Total box ≈ 93 columns. Shrink PANEL_W_RIGHT only together
# with the number formats in _panel — rows that no longer fit raise a red event
# rather than truncating silently.
# Event labels are ASCII-only ("->", not an arrow glyph): the arrow rendered
# badly in the user's terminal font.
PANEL_W_LEFT  = 26     # [chars] flight-phase column
PANEL_W_RIGHT = 60     # [chars] state column
PANEL_COLOR   = (not os.environ.get("AM_NO_COLOR")) and not _HEADLESS

# Gripper: this is a tracking-only rig, so the jaws are never actuated and the
# gripper dof is never read back — grasping lives in 02_aerial_manipulator_pick.py.
# The ONE thing still needed here is the stiff position drive that pins the hub
# at its authored rest (jaws open): the controller model lumps link 4's mass and
# inertia at exactly that pose, so a floppy hub would put the plant off-model.
# _setup_gripper_drive bakes it in pre-play and nothing touches it afterwards.
GRIPPER_JOINT      = "gripper_joint"
GRIPPER_REST_DEG   = 0.0                              # authored rest target (open)

# Select the trajectory: AM_SWEEP's traj_type (batch runs) beats AM_MODE (the
# one-name env var the launcher's optional 2nd argument sets), which beats the
# in-script MODE knob. See the MODE block above.
# set_traj_type validates the name eagerly, so a typo fails HERE — at import,
# before the stage loads and long before the takeoff — and the message lists
# every valid name instead of leaving you to grep the catalogue.
_MODE_SRC = "MODE (in-script)"
if "traj_type" in SWEEP:
    _MODE_REQ, _MODE_SRC = str(SWEEP["traj_type"]), "AM_SWEEP traj_type"
elif os.environ.get("AM_MODE"):
    _MODE_REQ, _MODE_SRC = os.environ["AM_MODE"].strip(), "AM_MODE"
else:
    _MODE_REQ = MODE
try:
    TRAJ_TYPE = P.set_traj_type(_MODE_REQ)
except ValueError as exc:
    print(f"\n[InProc] BAD TRAJECTORY MODE from {_MODE_SRC}: {exc}\n"
          f"         set it with the launcher's 2nd argument, e.g.\n"
          f"           ./scripts/start_aerial_manipulator_free.sh "
          f"<config> hover_drone\n"
          f"         or AM_MODE=<name>, or edit MODE in this file.\n", flush=True)
    simulation_app.close()
    sys.exit(2)
print(f"[InProc] trajectory: {TRAJ_TYPE}   (from {_MODE_SRC})", flush=True)

# landing protection is only meaningful for a trajectory whose final phase
# ends above the landing spot (see the LAND_* block above) — both poly modes
# end hovering at land_wp and then descend on the setpoint
LAND_ENABLE = TRAJ_TYPE in ("poly_whole", "poly_drone")

# ── Spawn arm pose ───────────────────────────────────────────────────────────
# The arm SPAWNS at the pose the takeoff hold will command (tr["q_hold"]) —
# the FOLDED HOME pose for the poly modes, the sweep start pose for the
# …_whole sweeps — instead of hanging at q=0. At q=0 the gripper pad reaches
# 0.282 m below the base, only ~2 cm off the floor at the 0.305 m resting body
# height, and the old zero-then-slew start swept it near the ground through
# the first ~1.5 s of the climb. Spawned at the hold pose the takeoff hold
# starts exactly ON target — the arm keeps that pose continuously from spawn
# through the whole climb, and the landing protection holds the same pose back
# down to touchdown.
# Resolved WITHOUT Isaac (utils_planner.initial_arm_pose is pure numpy; for
# poly_whole it reads the cached get_plan solve build_traj will reuse, so
# planning still happens once). Applied by writing the articulation joint
# state directly after _art.initialize() — NOT by authoring the USD
# JointStateAPI attrs (that does not survive this startup sequence, see the
# comment at the set_joint_positions call).
Q_SPAWN = P.initial_arm_pose(TRAJ_TYPE, params=C.make_params())


# ── two-panel terminal frame (see the PANEL_* block) ─────────────────────────
# Phase → colour. The tag is what the eye tracks down a long log, so the palette
# is deliberately coarse: cool while climbing, green while working, warm while
# coming down, red only for something that needs reading.
_ANSI = {"grey": "90", "red": "31", "green": "32", "yellow": "33",
         "blue": "34", "magenta": "35", "cyan": "36", "white": "37",
         "bold": "1"}
PHASE_COLOR = {"init": "grey", "takeoff": "cyan", "task": "green",
               "landing": "yellow", "done": "magenta", "warn": "red"}


def _c(text, color):
    """Wrap in an ANSI colour, or return it untouched when colour is off."""
    if not PANEL_COLOR or color is None:
        return text
    return f"\033[{_ANSI.get(color, '37')}m{text}\033[0m"


def _cell(text, width, color=None):
    """Pad/truncate to an exact VISIBLE width, then colour it. Colouring after
    padding is the point — escape codes are not printable width, so padding a
    coloured string misaligns the box."""
    return _c(str(text)[:width].ljust(width), color)


def _rpy_deg(R):
    """Body roll/pitch/yaw [deg] from the rotation matrix (Z-Y-X convention)."""
    pitch = math.asin(max(-1.0, min(1.0, -R[2, 0])))
    roll = math.atan2(R[2, 1], R[2, 2])
    yaw = math.atan2(R[1, 0], R[0, 0])
    return np.degrees([roll, pitch, yaw])


def _vec(v, fmt="%+6.2f", sep=" "):
    return sep.join(fmt % x for x in np.asarray(v, float).ravel())


def _rot_to_quat_wxyz(R):
    """Rotation matrix -> quaternion [w,x,y,z] (Isaac core's scalar-first
    convention), the standard branch-per-largest-diagonal form."""
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

# ── Control parameters: robotic_arm/config/free.yaml ─────────────────────────
# EVERY gain and law knob comes from that file — there are no gain defaults in
# the controller any more (all three scenario controllers were merged into one
# on 2026-08-09; what differs between free flight, pick and push is now only
# these numbers). The TRAJECTORY TYPE is passed as the variant, so a mode that
# needs its own tuning gets a section named after it in the YAML and nothing
# else changes; a mode with no section simply flies `default`.
#
# AM_SWEEP still overrides for batch runs. Its historical UPPER-CASE keys are
# mapped onto the file's names so existing sweep scripts keep working.
_SWEEP_ALIAS = {"DLS_LAMBDA": "dls_lambda", "USE_GMO": "use_gmo",
                "TAU_MAX": "tau_max", "M_Y": "M_Y",
                "k_x": "k_x", "k_v": "k_v", "k_R": "k_R", "k_w": "k_w",
                "K_y": "K_y", "D_y": "D_y", "K_o": "K_o", "M_r_d": "M_r_d"}
_overrides = {_SWEEP_ALIAS[k]: v for k, v in SWEEP.items() if k in _SWEEP_ALIAS}
CFG = CP.load("free", variant=TRAJ_TYPE, overrides=_overrides or None)

# ONE saturation limit for both sides. The law clamps its own joint torque and
# rebuilds the base moment from the REALIZED u3 (see the controller's
# saturation block); the clamp in the control step below then covers the
# PD-hold path, which the controller never sees. Both read cfg.tau_max, so the
# coupling and the applied torque stay derived from the same number.
# tau_max: null in the YAML means "no clamp in the law" — the demo-side clamp
# then has nothing to agree with, so it is disabled too rather than inventing
# a limit the controller does not know about.
TAU_MAX = np.inf if CFG.tau_max is None else float(CFG.tau_max)


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
        if SHOW_FLIGHT_VOLUME:
            draw_flight_volume(stage, size=FLIGHT_VOLUME_SIZE,
                               center=FLIGHT_VOLUME_CENTER,
                               floor_z=FLIGHT_VOLUME_FLOOR)
            print(f"[InProc] flight volume drawn: {FLIGHT_VOLUME_SIZE} m "
                  f"centred {FLIGHT_VOLUME_CENTER}", flush=True)
        add_dome_lighting(stage=stage, dome_path="/World/DomeLight",
                          intensity=2500.0, exposure=0.0, color=(1.0, 1.0, 1.0))

        # --- Spawn (ROS2 backend present but driven IN-PROCESS via input_ref) -
        self.drone_path = spawn_rotorcraft_with_mavlink(
            px4_path=self.pg.px4_path,
            px4_default_airframe=self.pg.px4_default_airframe,
            vehicle_id=VEHICLE_ID,
            spawn_pos=SPAWN_POS, spawn_euler=SPAWN_EULER,
            usd_file=USD_FILE, usd_prim_path=USD_PRIM_PATH,
            articulation_path="", body_path=BODY_PATH,
            rotor_paths=ROTOR_PATHS, rotor_joint_names=ROTOR_JOINT_NAMES,
            thrust_config={
                "rotor_constant": [ROTOR_K_THRUST] * 4,
                "rolling_moment_coefficient": [ROTOR_K_TORQUE] * 4,
                "rot_dir": ROTOR_DIR,
            },
            enable_lockstep=False,
            px4_primary=False,
            include_px4=False,        # no PX4 SITL in-process → skip MAVLink backend
        )

        self._wait_for_prim(self.drone_path, max_frames=300)
        # --- Model/physics fixes (same as the ROS2 demo) --------------------
        self._dedupe_physics_scenes()     # one PhysicsScene only
        self._disable_self_collisions()   # adjacent-link hulls overlap → thrash
        self._disable_rotor_colliders()   # props must not collide
        self._setup_gripper_drive()       # stiff position drive + OPEN target (bakes pre-play)
        self.world.reset()
        self.stage = omni.usd.get_context().get_stage()
        self._setup_physics_debug_viz()  # world/body frame axes (see SHOW_PHYSICS_FRAMES)

        print_mass_inertia_properties(self.stage, self.drone_path)
        print_rotor_positions(self.stage, self.drone_path, ROTOR_PATHS)

        # --- DC handles (body pose/twist only; the arm goes through the core
        # Articulation API below) -------------------------------------------
        self._dc     = _dynamic_control.acquire_dynamic_control_interface()
        self._body   = self._dc.get_rigid_body(self.drone_path + BODY_PATH)
        # --- Core Articulation API for the ARM (required). dynamic_control is
        # deprecated and its dof layer is broken here (wrong velocities, silent
        # gain writes); the tensor-backed Articulation API is the supported path
        # for dof state reads, effort writes and runtime gain changes.
        from omni.isaac.core.articulations import Articulation
        self._art = Articulation(prim_path=self.drone_path)
        self._art.initialize()
        names = list(self._art.dof_names)
        self._arm_idx = [names.index(nm) for nm in ARM_JOINT_NAMES]
        print(f"[InProc] core Articulation API active; arm dof indices {self._arm_idx}",
              flush=True)

        # SPAWN the arm AT the takeoff-hold pose (Q_SPAWN, the folded home) by
        # writing the joint state directly into the articulation view. This must
        # happen HERE — post-reset, before any physics stepping: authoring the
        # USD JointStateAPI attrs pre-reset does NOT take effect on this rig
        # (verified: q(t0) still read 0 — the pre-reset world.step() calls in
        # _wait_for_prim let PhysX capture its reset-snapshot at the parsed q=0,
        # and world.reset() restores that snapshot over the authored values).
        # PhysX roots this asset's reduced-coordinate tree at an ARM link, so
        # the joint teleport swings the BODY around the arm instead of the arm
        # around the body (measured: the body came out rolled by -(q2+q3) =
        # -86 deg about x and displaced — the anchored reference then chased a
        # sideways thrust axis and the vehicle tumbled at t=1 s). The teleport
        # is therefore followed by a RIGID RE-SEAT: measure the body pose
        # before/after the joint write and left-apply the world-frame
        # correction to the articulation root — this moves the whole vehicle
        # rigidly (joint angles untouched) back to its level spawn placement.
        # If a future asset roots the tree at the body, the correction is an
        # exact no-op.
        pose_a = self._dc.get_rigid_body_pose(self._body)
        self._art.set_joint_positions(np.asarray(Q_SPAWN, float),
                                      joint_indices=self._arm_idx)
        self._art.set_joint_velocities(np.zeros(len(self._arm_idx)),
                                       joint_indices=self._arm_idx)
        pose_b = self._dc.get_rigid_body_pose(self._body)
        Ra = C.quat_to_rot(pose_a.r.w, pose_a.r.x, pose_a.r.y, pose_a.r.z)
        Rb = C.quat_to_rot(pose_b.r.w, pose_b.r.x, pose_b.r.y, pose_b.r.z)
        pa = np.array([pose_a.p.x, pose_a.p.y, pose_a.p.z])
        pb = np.array([pose_b.p.x, pose_b.p.y, pose_b.p.z])
        R_corr = Ra @ Rb.T                     # X_des = X_corr · X_now
        p_corr = pa - R_corr @ pb
        pr, qr = self._art.get_world_pose()
        self._art.set_world_pose(
            position=R_corr @ np.asarray(pr, float) + p_corr,
            orientation=_rot_to_quat_wxyz(R_corr @ C.quat_to_rot(*np.asarray(qr, float))))
        print(f"[InProc] arm spawned at the takeoff-hold pose "
              f"q = {np.degrees(Q_SPAWN).round(1)} deg (body re-seated: "
              f"teleport moved it {np.linalg.norm(pb - pa):.3f} m / "
              f"{np.degrees(np.arccos(np.clip((np.trace(Rb @ Ra.T) - 1) / 2, -1, 1))):.1f} deg)",
              flush=True)

        # GROUND-SEAT (see GROUND_BODY_Z): translate the whole vehicle straight
        # down so the measured body height equals the resting height — takeoff
        # starts from the legs on the ground, not from the authored midair
        # spawn. A pure vertical world translation of the articulation root:
        # joint angles and attitude untouched, exact regardless of where PhysX
        # roots the tree. At the home pose the lowest arm point (the elbow,
        # 0.165 m below the base) keeps ~0.14 m of ground clearance here.
        pose_g = self._dc.get_rigid_body_pose(self._body)
        dz = GROUND_BODY_Z - float(pose_g.p.z)
        pr, qr = self._art.get_world_pose()
        self._art.set_world_pose(
            position=np.asarray(pr, float) + np.array([0.0, 0.0, dz]))
        print(f"[InProc] ground-seated: body z {pose_g.p.z:.3f} -> "
              f"{GROUND_BODY_Z:.3f} m (dz={dz:+.3f})", flush=True)

        # --- Pegasus backend (write rotor input_ref directly) ---------------
        self._vehicle = VehicleManager.get_vehicle_manager().get_vehicle(self.drone_path)
        self._backend = self._vehicle._backends[0]

        # --- Controller + mixer (in-process, controller.py + config/free.yaml) ---
        # CFG was loaded at module scope (with the trajectory type as the
        # variant and any AM_SWEEP overrides already merged), so the controller
        # is constructed with its parameters rather than mutated afterwards —
        # there is no window in which it holds numbers nobody chose.
        self.params = C.make_params()
        self.ctrl   = C.MatlabController(self.params, CFG)
        # arm hold ONLY if the legacy takeoff PD is explicitly enabled; otherwise
        # the designed GMO law drives the arm from the first step (the control
        # loop re-asserts self.ctrl.hold every step, so this is just the seed).
        self.ctrl.hold = TAKEOFF_ARM_HOLD
        print(f"[InProc] control params: {CFG.summary()}", flush=True)
        print(f"[InProc] TAKEOFF_ARM_HOLD={TAKEOFF_ARM_HOLD}", flush=True)
        print(f"[InProc] phase gates (hover error ONLY, no time gating): "
              f"pos<={HOVER_POS_TOL} m, |v|<={HOVER_V_TOL} m/s, "
              f"dq<={HOVER_Q_TOL} rad (takeoff), dwell {HOVER_DWELL}s; "
              f"arm switch is a plain assignment at a constant reference; "
              f"GMO engages WITH the law", flush=True)
        if SWEEP:
            print(f"[InProc] AM_SWEEP overrides: {SWEEP}", flush=True)
        # auto-stop + crash bookkeeping (sweep runs)
        self._t_end = float(SWEEP["t_end"]) if "t_end" in SWEEP else None
        self._crashed = False
        self._done = False
        self.mixer  = C.RotorMixer(C.RotorMixerParams())

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
        self._t = 0.0
        # three-phase state machine (see the FLIGHT PHASES block). Transitions
        # are one-way: takeoff → task → landing, each latched when its hover
        # gate clears.
        self._phase = "takeoff"
        self._sp_takeoff = None     # CoM setpoint: the task trajectory's start point
        self._sp_task_end = None    # task end point (set at re-anchor; landing gate)
        self._sp_land = None        # CoM setpoint: task end xy at LAND_SP_COM_Z
        self._release_t = None      # task-clock zero: stamped when the hold→law
                                    # blend COMPLETES (not at the phase switch)
        self._land_start_t = None   # sim time the landing phase began (phase table)
        self._descend = False       # landing: True once the arm is fully on the
                                    # PD and the descent setpoint is commanded
        self._x_c = None            # measured CoM, fed forward one step (hover gates)
        self._gate_ok_since = None  # when the hover gate last became satisfied
                                    # (dwell timer; None = currently violated)
        # two-panel terminal frame (see the PANEL_* block)
        self._events = []           # (t, label, colour) flight-phase log
        self._panel_t = -1e9        # last frame render (PANEL_PERIOD throttle)
        self._gate_txt = ""         # live pending-gate line, "" when none
        self._sat_warned = False    # arm saturation already reported once
        self._n_sat = 0             # joints at +-TAU_MAX this step (law + hold)
        self._hold_ref = None       # slewed arm-hold reference (see ARM_HOLD_RATE)
        self._wp_pending = False    # draw waypoint markers on the next render pass
        self._touchdown = False     # True once settled at the landing setpoint
        self._disarmed = False      # True once the post-landing rotor ramp finishes
        self._disarm_t = None       # sim time the ramp finished (CLOSE_DELAY)
        self._land_t = 0.0          # elapsed time in the rotor ramp-off
        # history buffers for MATLAB-style plotting (saved on exit → plot script)
        self._hist = {k: [] for k in
                      ("t", "p", "v0", "omega0", "e_R", "e_y", "q", "qdot",
                       "thrust", "tau_j", "u3",
                       "t_thrust", "t_imp", "t_dist", "t_cpl", "tau_b", "tau_b_arm",
                       "x_c", "x_cd", "r_e", "r_ed", "q_d",
                       # GMO disturbance estimates (transformed frame)
                       "d_t_hat", "d_r_hat", "d_rho_hat")}
        # seed the panel's phase log so the LEFT column is never blank
        self._events.append((0.0, f"seated z={GROUND_BODY_Z:.3f}",
                             PHASE_COLOR["init"]))
        carb.log_info("[InProc] ready")

    # ── terminal frame ──────────────────────────────────────────────────────

    def _event(self, label, phase="init"):
        """Record a flight-phase event for the LEFT panel. Events are rare (~8
        a flight), so each one also forces a frame immediately — the panel is
        the only output, and a phase change should not wait up to PANEL_PERIOD
        to appear."""
        self._events.append((self._t, label, PHASE_COLOR.get(phase, "white")))
        self._panel_t = -1e9        # force a render on this step
        if not PANEL_ENABLE:        # panel off: keep the plain line
            print(f"[InProc] {self._t:6.1f}s  {label}", flush=True)

    def _panel(self, p0, R0, q, res, tau_j, ref, x_c):
        """Print one two-panel frame: flight-phase log | live state.

        Composed as plain lines (no cursor control) so it survives the
        launcher's `tee` and interleaves safely with Isaac's own logging.
        """
        L, W = PANEL_W_LEFT, PANEL_W_RIGHT
        col = PHASE_COLOR.get(self._phase, "white")

        # LEFT: the phase log, newest last. Keep whatever fits the box height.
        ev = self._events[-max(6, min(len(self._events), 10)):]
        left = [(f"{t:5.1f} {lbl}", c) for t, lbl, c in ev]

        # RIGHT: one row per group. Errors: CoM tracking, EE task (e_y is
        # [dx,dy,dz,heading]), attitude.
        # Every quantity is NAMED in the label column and carries an explicit
        # [unit] — the value area is pure numbers, no inline symbols. Channels
        # that have components get one row each rather than being collapsed to
        # a norm, so a fault shows which axis it is on.
        def _row(grp, name, unit, vals):
            return f"{grp:<6} {name:<8} {unit:<8}{vals}"

        rpy = _rpy_deg(R0)
        e_c = np.asarray(x_c, float) - np.asarray(ref["x_cd"], float)
        e_y = np.asarray(res["e_y"], float)
        rows = [
            _row("DRONE", "pos", "[m]", _vec(p0, "%+8.3f")),
            _row("", "att rpy", "[deg]", _vec(rpy, "%+8.2f")),
            _row("ARM", "joint", "[deg]", _vec(np.degrees(q), "%+8.2f")),
            # body channel first (CoM + attitude), then the EE task channel
            _row("ERR", "CoM", "[m]", _vec(e_c, "%+8.3f")),
            # e_R = 0.5*vee(R_d'R - R'R_d) — the GEOMETRIC attitude error, NOT
            # roll/pitch/yaw: a body-frame 3-vector, dimensionless, equal to
            # sin(phi) for a single-axis error (so ~= radians while small, and
            # only meaningful below 90 deg).
            _row("", "att geom", "[-]", _vec(res["e_R"], "%+8.3f")),
            _row("", "EE pos", "[m]", _vec(e_y[:3], "%+8.3f")),
            _row("", "EE yaw", "[-]", f"{e_y[3]:+8.3f}"),
            # one row per actuator channel: thrust, body moment, arm torque
            _row("CTRL", "thrust", "[N]", f"{res['thrust']:8.2f}"),
            _row("", "moment", "[N.m]", _vec(res["tau_body"], "%+8.3f")),
            _row("", "arm", "[N.m]", _vec(tau_j, "%+8.2f")),
            _row("DIST", "force", "[N]", _vec(res["d_t_hat"], "%+8.2f")),
            _row("", "moment", "[N.m]", _vec(res["d_r_hat"], "%+8.2f")),
            _row("", "joint", "[N.m]", _vec(res["d_rho_hat"], "%+8.2f")),
        ]
        # Silent truncation is the failure mode of a fixed-width panel — a number
        # quietly loses its last digits and the log reads fine. Shout instead.
        for r in rows:
            if len(r) > W:
                self._events.append((self._t, f"PANEL row '{r[:4]}' needs "
                                     f"{len(r)} cols > {W}", "red"))
                break
        right = [(r, None) for r in rows]

        n = max(len(left), len(right))
        left += [("", None)] * (n - len(left))
        right += [("", None)] * (n - len(right))

        arm = "PD hold" if self.ctrl.hold else "whole-body"
        gmo = "off" if self.ctrl.gmo_inhibit else "on"
        sat = f"   SAT {self._n_sat}/4" if self._n_sat else ""
        head_r = f"STATE   t ={self._t:7.1f} s   arm: {arm}   GMO: {gmo}{sat}"

        out = ["┌" + "─" * (L + 2) + "┬" + "─" * (W + 2) + "┐",
               "│ " + _cell("FLIGHT", L, "bold") + " │ "
               + _cell(head_r, W, col) + " │",
               "├" + "─" * (L + 2) + "┼" + "─" * (W + 2) + "┤"]
        for (lt, lc), (rt, rc) in zip(left, right):
            out.append("│ " + _cell(lt, L, lc) + " │ " + _cell(rt, W, rc) + " │")
        out.append("└" + "─" * (L + 2) + "┴" + "─" * (W + 2) + "┘")
        if self._gate_txt:
            out.append("  " + _c(f"gate: {self._gate_txt}", "grey"))
        print("\n".join(out), flush=True)

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
        # actual vs reference (CoM / EE / arm) — for the MATLAB-style plots
        h["x_c"].append(np.asarray(x_c, float).copy())
        h["x_cd"].append(np.asarray(ref["x_cd"], float).copy())
        h["r_e"].append(np.asarray(r_e, float).copy())
        h["r_ed"].append(np.asarray(ref["r_ed"], float).copy())
        h["q_d"].append(np.asarray(ref.get("q_d", np.zeros(4)), float).copy())
        for k in ("t_thrust", "t_imp", "t_dist", "t_cpl"):
            h[k].append(np.asarray(res[k], float).copy())
        # GMO disturbance estimates (zero whenever the PD hold owns the arm —
        # takeoff and landing phases — or when USE_GMO is off)
        for k in ("d_t_hat", "d_r_hat", "d_rho_hat"):
            h[k].append(np.asarray(res[k], float).copy())

    def _report_flight_volume(self):
        """Sample the whole planned reference and check it against the room box,
        BEFORE the trajectory runs. Sampling the reference (not the flown path)
        is the point: it flags an out-of-room plan on the first physics step
        rather than after a 30 s flight. Both the CoM and EE references are
        checked, inflated by ROTOR_RADIUS — the rotor disc reaches further than
        either reference point and is what actually clips a wall or ceiling."""
        if not SHOW_FLIGHT_VOLUME or self._tr is None:
            return
        try:
            T = float(self._tr.get("T", 0.0))
            pts = []
            for tq in np.linspace(0.0, max(T, 1e-3), 400):
                ref = P.generate_reference(tq, self._tr)
                pts.append(ref["x_cd"])
                pts.append(ref["r_ed"])
            # the takeoff setpoint (the task start point) and, for landing
            # modes, the landing setpoint below the task end
            if self._sp_takeoff is not None:
                pts.append(np.asarray(self._sp_takeoff, float))
            if LAND_ENABLE:
                end = P.generate_reference(max(T, 0.0), self._tr)["x_cd"]
                pts.append(np.array([end[0], end[1], LAND_SP_COM_Z]))
            ok, txt = volume_report(np.array(pts), size=FLIGHT_VOLUME_SIZE,
                                    center=FLIGHT_VOLUME_CENTER,
                                    floor_z=FLIGHT_VOLUME_FLOOR,
                                    label=f"'{self._tr['type']}' reference",
                                    clearance=ROTOR_RADIUS)
            print(txt, flush=True)
            if not ok:
                print("[flight volume] WARNING: this trajectory leaves the room "
                      "box — re-centre FLIGHT_VOLUME_CENTER or shrink the task "
                      "(TRAJ_CONFIG target/land_wp) before flying it for real.",
                      flush=True)
        except Exception as exc:
            print(f"[flight volume] check skipped ({exc})", flush=True)

    def _draw_waypoints(self):
        """Point + arrow at the three showcase waypoints, taken from the
        ANCHORED trajectory so they land where the vehicle will actually go.
        The point is the commanded 3-D EE position, the arrow its commanded
        1-D yaw (b1_de) — exactly the four numbers the EE task controls."""
        # NOT gated on _HEADLESS: these are USD prims, not debug_draw, so they
        # belong in the stage either way (screenshots, USD export, and it makes
        # the geometry verifiable from a headless run).
        if not SHOW_WAYPOINT_MARKERS or self._tr is None:
            return
        try:
            plan = self._tr.get("plan")
            tp = (plan or {}).get("tp", {})
            if tp.get("mode") == "showcase":            # poly_whole
                T1, T2, T3 = tp["T1"], tp["T2"], tp["T3"]
            elif self._tr.get("type") == "poly_drone":  # same waypoint schedule
                T1, T2, T3 = self._tr["T1"], self._tr["T2"], self._tr["T3"]
            else:
                return              # only the waypoint flights have these
            marks = (("StartFlyIn", 0.0, WP_COL_START),
                     ("GimbalPoint", T1 + 0.5 * T2, WP_COL_PIN),
                     ("EndFlyOut", T1 + T2 + T3, WP_COL_END))
            off = math.radians(WP_ARROW_YAW_OFFSET_DEG)
            for name, tq, col in marks:
                ref = P.generate_reference(tq, self._tr)
                # commanded yaw -> a HORIZONTAL arrow at that heading, rotated
                # onto the vehicle's mechanical front (see WP_ARROW_YAW_OFFSET_DEG)
                b = np.asarray(ref["b1_de"], float)
                psi = math.atan2(b[1], b[0]) + off
                head = np.array([math.cos(psi), math.sin(psi), 0.0])
                draw_pose_marker(self.stage, f"/World/Waypoints/{name}",
                                 ref["r_ed"], heading=head,
                                 size=WP_SIZE, color=col)
            g = P.generate_reference(T1 + 0.5 * T2, self._tr)["r_ed"]
            print(f"[InProc] waypoint markers drawn — gimbal point at "
                  f"{np.round(g, 3)}", flush=True)
        except Exception as exc:
            print(f"[InProc] waypoint markers skipped ({exc})", flush=True)

    def _phase_table(self):
        """Flight-phase boundaries (sim time) + names, recorded in the npz so
        plot_results.py can shade each phase without re-deriving them from a
        config that may have changed since the run. Always starts with the
        takeoff setpoint phase; the poly_whole plan contributes its
        sub-phases (a legacy in-plan 'land' band only if T4 > 0), poly_drone
        its own fly-in/yaw/fly-out schedule, any other
        trajectory a single 'track' band; the post-plan hover wait is 'hold'
        and the setpoint descent (from the hover-gated switch on) 'landing'."""
        edges, names = [0.0, float(self._release_time())], ["takeoff"]
        tr = self._tr or {}
        tp = (tr.get("plan") or {}).get("tp", {})
        if tp.get("mode") == "showcase":
            for key, nm in (("T1", "fly-in"), ("T2", "pinned"),
                            ("T3", "fly-out"), ("T4", "land")):
                if float(tp.get(key, 0.0)) > 0.0:
                    edges.append(edges[-1] + float(tp[key]))
                    names.append(nm)
        elif tr.get("type") == "poly_drone":
            for key, nm in (("T1", "fly-in"), ("T2", "yaw"), ("T3", "fly-out")):
                edges.append(edges[-1] + float(tr[key]))
                names.append(nm)
        elif tr.get("T"):
            edges.append(edges[-1] + float(tr["T"]))
            names.append(str(tr.get("type", "track")))
        t_end = float(self._hist["t"][-1]) if self._hist["t"] else edges[-1]
        if self._land_start_t is not None:
            # hover wait between plan end and the gated landing switch, if any
            if self._land_start_t > edges[-1] + 1e-6:
                edges.append(float(self._land_start_t))
                names.append("hold")
            if t_end > edges[-1] + 1e-6:
                edges.append(t_end)
                names.append("landing")
        elif t_end > edges[-1] + 1e-6:
            edges.append(t_end)
            names.append("hold")
        return np.array(edges, float), np.array(names)

    def _save_history(self):
        h = self._hist
        if not h["t"]:
            return
        os.makedirs(LOG_DIR, exist_ok=True)
        path = SWEEP.get("log_path") or os.path.join(LOG_DIR, "gmo_log.npz")
        phase_t, phase_names = self._phase_table()
        np.savez(path, crashed=np.array(self._crashed),
                 phase_t=phase_t, phase_names=phase_names,
                 **{k: np.array(v) for k, v in h.items()})
        plot_script = os.path.join(_RA_DIR, "utils_plot", "plot_results.py")
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

    def _setup_gripper_drive(self):
        """Stiff position drive on gripper_joint, target = the authored rest
        (jaws open). Runs before world.reset() so the gains bake into PhysX (USD
        target attr is in DEGREES; gains proven in gripper_drone_demo.py). This
        demo never retargets it — the drive just holds the hub at the pose the
        controller's lumped link-4 inertia was extracted at."""
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
        return R0, p0, v0, omega0, q, qdot

    def _build_X(self, R0, p0, q, v0, omega0, qdot):
        return np.concatenate([p0, R0.flatten(order="F"), q, v0, omega0, qdot])

    def _release_time(self):
        """Sim time the takeoff→task handover actually happened. Purely
        hover-error gated (see the FLIGHT PHASES block), so there is no nominal
        value to fall back on — before the handover it reads as 'now' (the
        phase table then simply has no task band yet)."""
        return self._t if self._release_t is None else self._release_t

    def _traj_t(self):
        """Trajectory clock: 0 at the handover, so the plan always starts at its
        own t=0 no matter how long the takeoff took to satisfy the gate."""
        return self._t - self._release_time()

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

        if self._tr is None:
            # first-pose setup (anchor the trajectory) — mirrors the ROS2 node
            X0   = self._build_X(R0, p0, q, v0, omega0, qdot)
            dyn0 = C.dynamics(X0, self.params)
            x_c  = p0 + R0 @ dyn0["r_0c_0"]
            self._tr = P.build_traj(np.array([x_c[0], x_c[1], P.TAKEOFF_ALTITUDE]),
                                    R0 @ (dyn0["r_0e_0"] - dyn0["r_0c_0"]),
                                    R0 @ np.array([1.0, 0, 0]),
                                    params=self.params)
            # TAKEOFF SETPOINT = the task trajectory's own start point, read
            # straight from the anchored plan (requirement: no climb ramp — one
            # setpoint, like a real flight). The vertical channel is critically
            # damped (zeta = k_v/(2*sqrt(k_x)) = 1), so the step flies a clean
            # no-overshoot climb on the position loop alone.
            self._sp_takeoff = np.asarray(
                P.generate_reference(0.0, self._tr)["x_cd"], float).copy()
            self._t = 0.0
            print(f"[InProc] first p0={p0.round(3)} traj='{self._tr['type']}' "
                  f"takeoff setpoint x_cd={self._sp_takeoff.round(3)}  dt={dt:.4f}",
                  flush=True)
            self._report_flight_volume()
            self._wp_pending = True
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
        # ── Three-phase state machine (see the FLIGHT PHASES block) ──────────
        # takeoff → task → landing, one-way, every switch gated on the measured
        # HOVER ERROR at the phase boundary point — never on the clock. The CoM
        # is fed forward from the previous step (self._x_c: dynamics() runs
        # after this block), a 4 ms lag at 250 Hz — irrelevant at hover.
        v_gate = float(np.linalg.norm(v0))
        x_c_meas = self._x_c if self._x_c is not None else p0

        def _dwelled(ok):
            """Track the hover-gate dwell (see HOVER_DWELL): returns the time
            the gate has been CONTINUOUSLY satisfied; any violation resets it."""
            if not ok:
                self._gate_ok_since = None
                return 0.0
            if self._gate_ok_since is None:
                self._gate_ok_since = self._t
            return self._t - self._gate_ok_since

        if self._phase == "takeoff":
            q_gate = self._tr.get("q_hold", None)
            if q_gate is None:
                q_gate = np.zeros_like(q)
            dq_gate = float(np.max(np.abs(q - q_gate)))
            # the arm-on-target check verifies the HOLD reached q_hold — with
            # the hold disabled the law owns the arm and no such target exists
            dq_ok = (not TAKEOFF_ARM_HOLD) or dq_gate <= HOVER_Q_TOL
            pos_err = float(np.linalg.norm(x_c_meas - self._sp_takeoff))
            dwell = _dwelled(pos_err <= HOVER_POS_TOL and v_gate <= HOVER_V_TOL
                             and dq_ok)
            if dwell >= HOVER_DWELL:
                self._phase = "task"
                self._gate_ok_since = None
                self._gate_txt = ""
                self._event(f"TAKEOFF ok {pos_err:.3f} m", "takeoff")
                # RE-ANCHOR once, at the takeoff→task handover. The integrator-
                # free loops hover with a steady tilt+offset (e_R~0.03 roll →
                # thrust tilts → the position loop settles ~0.1 m off in y).
                # Anchoring at the ACTUAL hover pose starts the task with
                # e_x = e_y = 0 — the tolerance-sized residual to the planned
                # start point moves into the anchor instead of into the errors.
                dyn_a = C.dynamics(self._build_X(R0, p0, q, v0, omega0, qdot),
                                   self.params)
                x_c_now = p0 + R0 @ dyn_a["r_0c_0"]
                self._tr = P.build_traj(x_c_now,
                                        R0 @ (dyn_a["r_0e_0"] - dyn_a["r_0c_0"]),
                                        R0 @ np.array([1.0, 0, 0]),
                                        params=self.params)
                # task END point (re-anchored): the landing gate's target and
                # the landing setpoint's xy
                self._sp_task_end = np.asarray(P.generate_reference(
                    float(self._tr.get("T", 0.0)), self._tr)["x_cd"], float).copy()
                self._wp_pending = True      # re-anchored: markers must move too
                # Task clock starts HERE. Safe to switch the arm and start the
                # plan together: the plan is rest-to-rest, so generate_reference(0)
                # is the rest state (no derivative step), and the re-anchor moves
                # the reference ONTO the vehicle (e_x, e_y -> 0) rather than away
                # from it. That is exactly the state the 0.005 N·m switch step
                # was measured at. Only a step that CREATES error needs isolating.
                self._release_t = self._t
                self._event("TASK arm->law", "task")
            else:
                self._gate_txt = (f"takeoff  pos {pos_err:.3f}/{HOVER_POS_TOL} m "
                                  f"|v| {v_gate:.3f}/{HOVER_V_TOL} m/s "
                                  f"dq {dq_gate:.3f}/{HOVER_Q_TOL} rad "
                                  f"dwell {dwell:.1f}/{HOVER_DWELL} s")
        elif self._phase == "task":
            if (LAND_ENABLE
                    and self._traj_t() >= float(self._tr.get("T", 0.0))):
                # plan is over — the reference is clamped at the task END point;
                # begin the landing SEQUENCE once the vehicle has demonstrably
                # hovered there for HOVER_DWELL. This switches ONLY the arm
                # (law→hold, reference stays at the end point); the descent
                # setpoint is commanded later, by the landing branch below.
                pos_err = float(np.linalg.norm(x_c_meas - self._sp_task_end))
                dwell = _dwelled(pos_err <= HOVER_POS_TOL
                                 and v_gate <= HOVER_V_TOL)
                if dwell >= HOVER_DWELL:
                    self._phase = "landing"
                    self._land_start_t = self._t
                    self._gate_ok_since = None
                    self._sp_land = np.array([self._sp_task_end[0],
                                              self._sp_task_end[1],
                                              LAND_SP_COM_Z])
                    self._gate_txt = ""
                    self._event("TASK ok arm->hold", "landing")
                else:
                    self._gate_txt = (f"task end  pos {pos_err:.3f}/"
                                      f"{HOVER_POS_TOL} m  |v| {v_gate:.3f}/"
                                      f"{HOVER_V_TOL} m/s  dwell {dwell:.1f}/"
                                      f"{HOVER_DWELL} s")
        elif self._phase == "landing" and not self._descend:
            # THE isolation that fixes the arm slam. This branch cannot run on
            # the step the phase became "landing" (the elif chain took the task
            # branch then), so by the time it does, ctrl.hold has been True for
            # a full control call: u3 is already zero and the law no longer
            # touches the arm. Only now is the 1.17 m descent step commanded,
            # and the reference has been constant at the end point throughout
            # the switch. Re-check the hover gate so a disturbed vehicle waits.
            pos_err = float(np.linalg.norm(x_c_meas - self._sp_task_end))
            if pos_err <= HOVER_POS_TOL and v_gate <= HOVER_V_TOL:
                self._descend = True
                self._event(f"DESCEND z={LAND_SP_COM_Z}", "landing")

        climbing = self._phase == "takeoff"
        landing  = self._phase == "landing"    # latched by the state machine
        arm_hold = (TAKEOFF_ARM_HOLD and climbing) or landing
        # The arm's torque SOURCE — a plain switch, no cross-fade (see the
        # SEQUENCING RULE: measured step across it is 0.1% of TAU_MAX once the
        # reference is held constant). hold=True zeroes u3 BEFORE τ is formed,
        # so tau_body never pre-compensates a reaction the arm is not producing.
        self.ctrl.hold = arm_hold
        # GMO runs exactly when the law does: u3 consumes d_e_hat, so the law
        # must not run on a frozen observer, and a hold phase must not let
        # contact forces leak into the estimate.
        self.ctrl.gmo_inhibit = bool(arm_hold)

        # ALL references come from utils_planner — nothing is built here.
        # Takeoff and landing are SETPOINTS (one point each, no profile). The
        # landing phase HOLDS the task end point on the step the arm switches
        # back to the PD, and steps to the descent setpoint only once _descend
        # latches on the following step (see the SEQUENCING RULE).
        if self._phase == "takeoff":
            ref = P.setpoint_reference(self._sp_takeoff, self._tr)
        elif self._phase == "landing":
            ref = P.setpoint_reference(
                self._sp_land if self._descend else self._sp_task_end, self._tr)
        else:
            ref = P.generate_reference(self._traj_t(), self._tr)

        X   = self._build_X(R0, p0, q, v0, omega0, qdot)
        dyn = C.dynamics(X, self.params)
        res = self.ctrl(X, dyn, ref, dt)


        # rotors → input_ref (clamped: a runaway command can't blow up the sim)
        omega = np.clip(self.mixer.mix(res["thrust"], res["tau_body"]), 0.0, OMEGA_MAX)
        # rotor ramp-off: once the vehicle has SETTLED at the landing setpoint
        # (same hover gate — 4 cm above the resting height), wind the rotors to
        # zero and let gravity take the last centimetres, instead of the
        # position loop hovering just off the ground indefinitely. Latched.
        if landing and self._descend:
            if not self._touchdown:
                pos_err = float(np.linalg.norm(x_c_meas - self._sp_land))
                if pos_err <= HOVER_POS_TOL and v_gate <= HOVER_V_TOL:
                    self._touchdown = True
                    self._event("TOUCHDOWN", "landing")
            if self._touchdown:
                self._land_t += dt
                scale = max(0.0, 1.0 - self._land_t / LAND_DISARM_TIME)
                if scale == 0.0 and not self._disarmed:
                    self._disarmed = True
                    self._disarm_t = self._t
                    self._event(f"LANDED z={p0[2]:.3f} m", "done")
                omega = omega * scale
        # auto-close: the flight is over once the rotors are off and the vehicle
        # has had CLOSE_DELAY to settle on its legs — save the npz and exit
        if self._disarmed and self._t - self._disarm_t >= CLOSE_DELAY:
            self._done = True
        for i in range(min(4, len(self._backend.input_ref))):
            self._backend.input_ref[i] = float(omega[i])

        # arm efforts EVERY step. The task phase runs the DESIGNED GMO law
        # (res["tau_joint"]); the software PD hold owns the arm during takeoff
        # (when TAKEOFF_ARM_HOLD) and landing. One actuation path, one clamp.
        if arm_hold:
            g_arm = dyn["g"][6:]
            # ONE hold target for both ends of the flight: the trajectory's
            # q_hold (the folded HOME pose for the poly modes, the sweep start
            # pose for the …_whole sweeps). The task starts AND ends there —
            # takeoff pre-positions the arm for zero initial EE error, and the
            # landing hold keeps the same pose the task already returned to,
            # so nothing moves at touchdown.
            q_hold = self._tr.get("q_hold", None)
            q_target = q_hold if q_hold is not None else np.zeros_like(q)
            # SLEW the hold reference toward the target rather than stepping it
            # (see ARM_HOLD_RATE): a 40° step at spawn overshot into the q2
            # hard stop; the ramp keeps the transient inside the limits.
            if self._hold_ref is None:
                self._hold_ref = np.asarray(q, float).copy()
            d_ref = np.clip(q_target - self._hold_ref,
                            -ARM_HOLD_RATE * dt, ARM_HOLD_RATE * dt)
            self._hold_ref = self._hold_ref + d_ref
            tau_j = -ARM_HOLD_KP * (q - self._hold_ref) - ARM_HOLD_KD * qdot + g_arm
        else:
            self._hold_ref = None      # re-seed from the live q on re-engagement
            tau_j = res["tau_joint"]   # ctrl.hold=False, so this is the full law
        # Final actuator guard. On the LAW path this is idempotent — the
        # controller already clamped at ± TAU_MAX and rebuilt tau_body from the
        # realized u3 — so it only ever bites on the HOLD path, whose torque the
        # controller never sees. Kept after the branch so nothing can reach
        # set_joint_efforts unclamped whatever produced it.
        tau_pre = tau_j
        tau_j = np.clip(tau_j, -TAU_MAX, TAU_MAX)
        # Saturation count from BOTH paths. The two are disjoint (the law is
        # clamped in the controller, the hold here), so they simply add — and a
        # saturating hold is now as visible as a saturating law.
        self._n_sat = res["n_sat"] + int(np.count_nonzero(tau_j != tau_pre))
        # Saturation is a FAULT signal, not routine: the arm cannot deliver what
        # it is being asked for, so it tracks worse than commanded. The vehicle
        # stays CONSISTENT either way (tau_body is rebuilt from the realized u3
        # on the law path, and carries no arm reaction at all on the hold path)
        # — but it is worth seeing. Reported once; the panel header then carries
        # the live count.
        if self._n_sat and not self._sat_warned:
            self._sat_warned = True
            src = "hold" if arm_hold else "law"
            self._event(f"ARM SAT {self._n_sat}/4 {src} @{TAU_MAX}", "warn")
        self._art.set_joint_efforts(np.asarray(tau_j, float),
                                    joint_indices=self._arm_idx)

        # Measured CoM, fed to the next step's hover gates (a one-step lag at
        # 250 Hz = 4 ms; dyn is only available here, after the control call).
        x_c_now = p0 + R0 @ dyn["r_0c_0"]
        self._x_c = x_c_now

        self._record(p0, v0, omega0, q, qdot, tau_j, res, ref,
                     x_c=x_c_now, r_e=p0 + R0 @ dyn["r_0e_0"])

        # ONE frame per PANEL_PERIOD (and immediately after any phase event,
        # which resets _panel_t). Everything else this loop prints goes through
        # _event, so the terminal carries exactly two things: the phase log and
        # the current state.
        if PANEL_ENABLE and self._t - self._panel_t >= PANEL_PERIOD:
            self._panel_t = self._t
            self._panel(p0, R0, q, res, tau_j, ref, x_c_now)

    # ── main loop ───────────────────────────────────────────────────────────

    def run(self):
        self.world.add_physics_callback("am_control", self._control_step)
        if START_PAUSED and not _HEADLESS:
            # ONE physics step BEFORE pausing. The arm-home teleport and the
            # ground-seat live in the PHYSICS state and only reach the renderer
            # on a physics sync — pausing straight away would show the authored
            # USD pose instead (arm hanging at q=0, vehicle at the midair
            # SPAWN_POS: exactly what the init code replaced). The first
            # control callback only anchors the trajectory and RETURNS (no
            # actuation), so this advances the resting plant 4 ms with zero
            # commands — a physical no-op — and as a bonus the waypoint
            # markers and the flight-volume report exist during inspection.
            self.timeline.play()
            self.world.step(render=False)
            if self._wp_pending:
                self._wp_pending = False
                self._draw_waypoints()
            self.timeline.pause()
            self.world.render()          # show the true initial state
            print("[InProc] PAUSED for inspection — press PLAY in the GUI to "
                  "fly (do NOT press STOP: stop→play resets PhysX and reverts "
                  "the arm spawn pose)", flush=True)
            while simulation_app.is_running() and not self.timeline.is_playing():
                simulation_app.update()
            if simulation_app.is_running():
                print("[InProc] PLAY — flight begins", flush=True)
                self._events.append((0.0, "PLAY", PHASE_COLOR["init"]))
        else:
            self.timeline.play()
        render = not _HEADLESS           # headless: skip rendering
        while simulation_app.is_running() and not self._done:
            self.world.step(render=render)   # control runs inside the physics steps
            if self._wp_pending:     # author marker prims outside the physics step
                self._wp_pending = False
                self._draw_waypoints()
        self._save_history()
        self.timeline.stop()
        simulation_app.close()


def main():
    InProcessDemoV2().run()


if __name__ == "__main__":
    main()
