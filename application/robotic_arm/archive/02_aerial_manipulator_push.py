#!/usr/bin/env python
"""
02_aerial_manipulator_push.py

Author: Shiqi Gao (shiqi.gao907@gmail.com)

IN-PROCESS aerial-manipulator demo running controller.py (the COUPLED
feedback-linearizing law + generalized-momentum disturbance observer) inside a
physics-step callback — no ROS2 round-trip, true 250 Hz control at the physics dt.

Sibling of 02_aerial_manipulator_pick.py — the same rig and controller family
(the same law and the same GMO, selected by config/push.yaml), but the
physical interaction is PUSHING, not grasping: the closed jaws shove a box a
fixed distance across a table. The EE gets a straight LINE command at FIXED
YAW; with the arm held at a fixed pose, the dynamically-compatible CoM
reference of that line degenerates to the line minus the constant CoM→EE
offset — exactly what the FK-consistent schedule reference produces — and the
contact force (nowhere in the reference model) is carried by the task
impedance + the GMO. That is the demonstration.

TASK (default MODE = "push_home"): a full ground-to-ground mission along
WORLD +y, the direction the arm faces —
  takeoff   fly the BODY to the task start point with the arm FOLDED at its
            HOME pose ([0,40,40,0] deg)
  fly-out   translate to above the push start while the arm UNFOLDS to the
            push pose (q = 0, tool vertical) and the jaws CLOSE — the closed
            fingers are the pusher, nothing is ever between them
  approach  vertical descent so the hanging fingers sit BESIDE the box
  push      the base flies a STRAIGHT +y line at constant height and fixed
            heading; the fingers meet the box after a small spawn gap and
            slide it ~0.32 m down the table against sliding friction
  retreat   back off along −y (contact ends — the "stop pushing"), climb out
  fly-land  translate PAST the table while the arm FOLDS back to HOME
  land      vertical descent onto the legs, arm folded, rotors ramped off
The arm has exactly TWO setpoints (home and push) and the schedule min-jerks
between them during the transit flights — it is FIXED at the push pose from
the approach until after the retreat, so the push geometry never changes while
there is contact. The DRONE does the pushing; real contact friction on the
fingers is the unmodelled disturbance the GMO rejects.

The home pose is REQUIRED, not decorative: at q = 0 the finger tips reach 0.356 m
below the base, i.e. 51 mm BELOW GROUND at the 0.305 m resting body height, so
the vehicle cannot sit down with the arm hanging. Folded, the lowest arm point
(the elbow) keeps 0.140 m of clearance. Takeoff and landing therefore both run
the software PD arm hold (TAKEOFF_ARM_HOLD / the LAND_* block), because the
coupled law is only proven airborne — ground contact is not in the model.

ALTERNATIVE TASK (EE_FORCE_ENABLE, default False): takeoff → hover → apply a
CONSTANT force at the end-effector (default 1 N along inertial +y), mirroring
main_sim.m's 'const_wrench' disturbance. The GMO estimates it (d_t_hat converges
to the applied force) and the controller rejects it (the vehicle holds position).

Three-phase flight, selected by the MODE knob below — the same structure (and
the same code) as 02_aerial_manipulator_free.py, and the same workflow a real
experiment uses (takeoff and landing are position SETPOINTS; only the middle
phase tracks a plan; every switch is gated on the measured hover error, never
on the clock):
  Phase 1 TAKEOFF  x_cd = the task trajectory's own start point (ONE setpoint,
                   read from the anchored plan — no climb ramp). Arm PD-held at
                   home. Ends when the hover gate clears at that point.
  Phase 2 TASK     the MODE trajectory runs, re-anchored at the actual hover
                   pose (zero initial position/orientation error), clock t=0 at
                   the handover. Arm on the whole-body law, GMO on — which is
                   what carries the push contact force.
  Phase 3 LANDING  (landing modes only) x_cd = the task end point with z at
                   LAND_SP_COM_Z (ONE setpoint — no descent profile; the plan
                   is built with land_in_plan=False so its own descent segment
                   is gone). Arm back on the PD hold. Once settled there the
                   rotors ramp off and the app saves the npz and closes itself.
A rendered run starts PAUSED so the initial settings can be inspected — press
PLAY in the GUI to fly (START_PAUSED below).

Each physics step (world.add_physics_callback → _control_step):
  1. read body pose/twist (DC) + arm joint states (core Articulation API)
  2. build X, generate the setpoint / trajectory reference  (controller.py)
  3. dynamics() + MatlabController()  → thrust, tau_body, tau_joint  (+ GMO)
  4. mixer  → 4 rotor speeds → backend.input_ref
  5. tau_joint → art.set_joint_efforts (arm dofs in true effort mode);
     the designed GMO law in the TASK phase, the software PD hold during
     takeoff (when TAKEOFF_ARM_HOLD) and landing
  6. gripper effort from the trajectory's grip flag

Run with:  scripts/start_aerial_manipulator_push.sh <config_name>
(no WSL controller needed)
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
#   log_path (str)          npz destination (default log/push_log.npz)
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

from pegasus.simulator.params import SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.logic.vehicle_manager import VehicleManager

from fsc_aerial_manipulation.utils import add_dome_lighting
from fsc_aerial_manipulation.rotorcraft.x650_rotorcraft_utils import (
    spawn_rotorcraft_with_mavlink,
    print_mass_inertia_properties,
    print_rotor_positions,
)
from fsc_aerial_manipulation.robotic_arm.utils_controller import controller as C        # model + control law (one law, every scenario)
from fsc_aerial_manipulation.robotic_arm.utils_controller import control_params as CP   # gains: robotic_arm/config/push.yaml
from fsc_aerial_manipulation.robotic_arm import utils_planner as P                      # ALL desired trajectories

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
# reference math; NOTHING in this file builds a reference dict):
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
#   "push_home" — DEFAULT. The BOX-PUSH mission: STARTS AND ENDS ON THE GROUND
#              and the arm has TWO setpoints: a folded HOME pose ([0,40,40,0]
#              deg) for takeoff and landing, and the PUSH pose (q=0, tool
#              vertical, jaws held closed) for the task. The base flies a
#              straight fixed-yaw +y line at push height so the hanging closed
#              fingers shove the box push_dist down the table, then a retreat
#              leg breaks contact before the climb-out. Nothing is planned —
#              step setpoints the schedule min-jerks between. The home pose is
#              REQUIRED to land: at q=0 the finger tips reach 51 mm BELOW
#              ground at the resting body height. See TRAJ_CONFIG ("push_home")
#              and the LAND_* block below.
#   (controller.py also retains its pick sibling's pickplace_* modes,
#    selectable here for A/B runs — but the props THIS demo spawns are the push
#    table + box, sized for push_home only.)
# Override per-run without editing this file: AM_SWEEP='{"traj_type":"circle"}'.
MODE = "push_home"  # "hover"|"circle"|"circle_bent"|"push_home"

# Start PAUSED (rendered runs only): the scene is fully built — vehicle seated
# on the ground, arm at the folded home pose (Q_SPAWN), table and box spawned —
# and physics is frozen until PLAY is pressed in the GUI, so the initial
# geometry can be inspected first (this is the demo where the box/finger
# clearances decide whether the push works at all). One silent physics step
# runs before the pause (see run()): the arm pose and ground-seat live in the
# physics state and would otherwise not reach the renderer — the paused view
# would show the authored midair, q=0 asset instead of the true initial state.
# Press PLAY to fly. Do NOT press STOP: stop→play runs a world reset, which
# restores PhysX's reset snapshot (captured at q = 0) and silently undoes the
# spawn-pose teleport — the same gotcha the Q_SPAWN block below documents.
# Headless / sweep runs have no PLAY button and start immediately.
START_PAUSED = True

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
EE_FORCE_VIZ    = True
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
# The vehicle is SEATED ON THE GROUND at init (takeoff starts from the legs,
# like a real flight, not from midair): after the arm teleport below, the whole
# vehicle is translated vertically so the measured BODY height equals the
# resting height from the 2026-08-06 landing run. SPAWN_POS's z is therefore
# only the authored pre-seat placement; the seat overrides it. Done by
# measurement (not by editing SPAWN_POS) because the articulation root and the
# body link are different frames on this asset — the root sits ~0.11 m below
# the body — and the resting height is known for the BODY. The push props are
# placed from SPAWN_POS's xy and TAKEOFF_ALTITUDE only, so a purely vertical
# seat leaves the table/box geometry untouched.
GROUND_BODY_Z = 0.305   # [m] measured resting body height on the legs

# Rotor model — must match controller.py RotorMixerParams and the spawn thrust curve.
ROTOR_K_THRUST = 1.03e-5
ROTOR_K_TORQUE = 1.0e-6
ROTOR_DIR      = [-1, -1, 1, 1]

# Reflected rotor inertia — must equal controller.py make_params J_arm.
ARM_ARMATURE   = 353.5 ** 2 * 1.6e-7   # ≈ 0.02 kg·m²

# Safety clamps so a runaway control command can't blow up the physics / crash the
# sim. Normal operation is well inside these; they only cap pathological spikes.
# TAU_MAX sized to the joints: gravity scale is ~0.25 N·m, normal impedance ~0.5;
# the old 5.0 let a transient dump 250 rad/s² into a 0.02 kg·m² joint — with
# true effort control (no hidden drive damping) that flailed the arm at spawn
# and its reactions (>> the ~3.7 N·m the rotors can differential) tumbled the body.
TAU_MAX   = 1.5      # [N·m]   max arm joint torque (normal ~0.5)
OMEGA_MAX = 2000.0   # [rad/s] max rotor speed (hover ~850)

# ── Takeoff / landing arm handling (mirrors 02_aerial_manipulator_free.py) ────
# TAKEOFF_ARM_HOLD — arm control during the TAKEOFF phase.
#   True (DEFAULT here): a software joint-space PD (+ gravity comp) holds the arm
#     at the trajectory's q_hold (the folded HOME pose) through the takeoff, and
#     the coupled law / GMO engage together at the hover-gated handover. The
#     coupled law is only proven AIRBORNE — ground contact is not in the model —
#     and "push_home" both starts and ends on the ground, so the hold is what
#     keeps the arm folded and clear of the floor through the climb.
#   False: the DESIGNED GMO control law (res["tau_joint"]) drives the arm for the
#     WHOLE flight — no joint-space PD anywhere, observer active from t=0. Fine
#     for the airborne-only modes (hover, circle, circle_bent).
# NOTE: the body (thrust + attitude) is ALWAYS on the designed law; this flag
# only affects the ARM channel during takeoff. The takeoff setpoint itself
# (the body flying to the task start point) runs regardless.
TAKEOFF_ARM_HOLD = True

# Software-hold PD gains — used during the takeoff hold and the landing hold.
# Gentle PD + gravity comp through the same effort path (drive gains are never
# switched mid-sim; mid-sim gain writes went stale in PhysX before).
ARM_HOLD_KP = 3.0    # [N·m/rad]   ωn = sqrt(3/0.02) ≈ 12 rad/s per joint
ARM_HOLD_KD = 0.25   # [N·m·s/rad] ζ ≈ 0.5
# The hold TARGET is rate-limited toward q_hold rather than stepped: a raw
# 0 → 40° step at spawn makes the ζ≈0.5 PD overshoot into q2's +50° hard stop.
# Since the arm now SPAWNS at the hold target (Q_SPAWN), this is a no-op on a
# normal takeoff and exists to protect the hold's re-engagement paths (the
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
# WITH the law: this controller ties the observer to ctrl.hold directly
# (gmo_active = USE_GMO and not hold), so a law never runs on a frozen observer
# and a hold phase never lets ground contact leak into the estimate.
# SEQUENCING RULE — change ONE thing at a time. The arm's torque-source switch
# always happens at a CONSTANT reference, and a reference STEP is only commanded
# once the arm is already on its new source. This, not any torque smoothing, is
# what prevents an arm slam at the landing handover: dropping the landing
# reference by land_drop in the SAME step that hands the arm back lets the
# whole-body law (still owning the arm) see the thrust cut out, and the
# coupling feedforward F_trans = u1·R0e3 − mg·e3 then floods u3 with a torque
# demand many times TAU_MAX. Sequenced properly the arm torque is CONTINUOUS
# across the switch with no blending at all: at a settled hover with e_y = 0,
# everything at rest and F_trans = 0, the law's u3 is ~0 and its arm torque
# reduces to the same gravity term the PD hold applies.
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
# and freezes the GMO for the whole descent. The plan itself no longer descends:
# the demo builds it with land_in_plan=False, so the schedule ends at land_wp at
# cruise altitude and the descent is this one setpoint.
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
#   RIGHT the live state, one row per group (pos / att / arm / grip / err /
#         ctrl / dist / box)
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
GRIPPER_REST_DEG   = 0.0                              # authored asset rest (jaws mid-open)
# DEAD CONSTANTS — kept only because they document the old position-servo era.
# The grip is effort-driven (see _gripper_effort / _setup_gripper_drive, drive
# stiffness 0), so nothing reads these: "open" is wherever the constant opening
# effort parks the jaws, i.e. the authored +50° limit. Closing is NEGATIVE hub
# rotation (confirmed in the log: grip cmd reached −50° when grasping).
# The "+30° widens the pad gap ~7 cm → ~9 cm" claim these carried was WRONG —
# that was the grip-link ORIGIN spacing. Measured jaw gap at the pinch band is
# 14.6 mm at 0° and 41.9 mm at the +50° open stop (see PUSH_PROPS below).
GRIPPER_OPEN_DEG   = 30.0
GRIPPER_OPEN       = math.radians(GRIPPER_OPEN_DEG)   # [rad] unused
GRIPPER_CLOSED     = math.radians(-50.0)              # [rad] unused
GRIPPER_RATE       = 0.5                              # [rad/s] unused
# Gripper in TORQUE (effort) mode: a constant CLOSING torque holds the cube with a
# steady squeeze (a position drive can relax and drop it); opening spreads the jaws
# to the +limit. Closing is NEGATIVE hub rotation → closing effort is negative. The
# drive stiffness is 0 (see _setup_gripper_drive) so effort is the sole actuation.
# NOTE (2026-07-23): the grip effort MUST be applied in its OWN apply_action call —
# appending it to the arm's set_joint_efforts scrambled the effort→DOF mapping
# (measured grip_q stuck at 0 while the arm diverged into the q3 limit). Tunable.
GRIP_CLOSE_EFFORT  = 0.8    # [N·m] steady closing squeeze (grasp / carry)
GRIP_OPEN_EFFORT   = 0.5    # [N·m] opening torque (spread jaws to the +limit)
GRIP_DAMPING       = 2.0    # [N·m·s/rad] drive damping for a stable close (no servo)

# ── push props ───────────────────────────────────────────────────────────────
# Spawned automatically when TRAJ_TYPE is "push_home": ONE long TABLE under the
# whole push line, with the BOX standing on it just ahead of the fingers at the
# push start. All positions come from the same FK the trajectory uses plus the
# measured gripper geometry below. The box's world position is recorded in the
# npz ("obj_p") — the push distance is measurable, not just visual.
PUSH_PROPS = True

# ── MEASURED gripper geometry (headless gripper_joint sweep at arm q=0, from
# the pick demo; every offset is in the EE frame, +z up the tool axis) ────────
# The fingers are L-shaped and reach 0.124 m below the EE origin; the finger
# pads are ~58 mm wide along y, so their LEADING face — the pushing surface on
# the +y side — sits ~0.029 m ahead of the EE origin. Directly above the jaws
# the gripper body is WIDE:
#   hub          z ∈ [−0.0579, −0.0393], x ±0.011, y ±0.020   (EE frame)
#   manip_base   z ∈ [−0.0457, +0.0025], x ±0.064, y ±0.038   <- a 128 mm plate
# The box stays AHEAD of the jaws (+y) for the whole push — nothing is ever
# between them — but its near face rides at y ≈ +0.029, inside the plate's
# ±0.038 y-extent, so anything taller than the ceiling below gets pushed by
# the gripper BODY instead of the fingers.
# CEILING LOWERED −0.062 → −0.080 (2026-08-07, measured): the first sizing put
# the box top at EE−0.065, 10 mm under the pinch-band top at EE−0.055 — but the
# EE rides ±20–25 mm VERTICALLY while in contact (headless run: e_z −25..+20 mm,
# GMO d_t_z sustained +1.5–2 N, peak +4.8 N = the jaw structure pressing DOWN
# on the box's top edge), and that top-edge vertical contact is what tipped the
# cube over in rendered runs. The top must clear the pinch-band bottom by the
# full measured excursion: EE−0.055 − 0.025 = −0.080.
FINGER_TIP_OFF  = -0.1239   # lowest finger point, EE frame [m]
FINGER_LEAD_Y   = 0.029     # leading (pushing) face of the finger pads, EE +y [m]
BOX_TOP_CEIL_EE = -0.080    # [m] box TOP must stay below this EE-frame height

# The box: pushed, never grasped. RESIZED for a smooth slide (2026-08-07, after
# rendered runs showed occasional tip-over — see the ceiling note above for the
# main culprit). Three levers, all in the tip-resisting direction:
#   LOW + WIDE — top at EE−0.090 (10 mm under the lowered ceiling), base 0.12 m
#     along the push direction. Fingers contact the side face from 30 to 64 mm
#     above the table (centroid 47 mm, CoM 32 mm): quasi-static slide-not-tip
#     needs μ < (base/2)/centroid ≈ 1.3, and a horizontal transient only tips
#     it above ~7 N — vs the ~2 N breakaway spike after the fixes.
#   SLIPPERY BOTH SIDES — μ 0.25/0.20 on the box AND the table top: PhysX's
#     default friction combine is AVERAGE, so a slippery box on a default-
#     material table still lands at μ_eff ≈ 0.35. Sliding load on the fingers
#     ≈ μ·m·g ≈ 0.4 N — still a real, GMO-visible disturbance.
#   (Third lever is time: T_push 6→8 s in TRAJ_CONFIG, contact onset at
#    ~0.04 m/s instead of 0.055.)
BOX_XYZ  = np.array([0.10, 0.12, 0.064])  # [m] (y = push direction, z = height)
BOX_MASS = 0.20                            # [kg] deliberately unmodeled
BOX_FRICTION = (0.25, 0.20)                # static, dynamic — box AND table top
PUSH_TIP_CLEAR = 0.030   # [m] finger-tip clearance above the table top, sized
                         # to the measured −25 mm contact-phase EE sag; box
                         # height then puts the top at EE−0.090 (ceiling −0.080,
                         # checked in the spawn print)
BOX_GAP_Y      = 0.03    # [m] finger leading face -> box near face at the push
                         # start: the first 30 mm of the push closes this gap in
                         # free flight, so the box slides ≈ push_dist − 0.03 m
                         # and the descent can never land ON the box

# ONE long table under the whole push line (the pick demo's two 0.30 m squares
# become a single 0.30 m × ~0.6 m top spanning box start → box end + margins;
# slab/leg dimensions unchanged).
TABLE_XY        = 0.30    # [m] table-top width (x)
TABLE_TOP_THK   = 0.03    # [m] top slab thickness
TABLE_LEG       = 0.035   # [m] square leg cross-section
TABLE_LEG_INSET = 0.02    # [m] leg inset from the table-top edge
TABLE_MARGIN_Y0 = 0.06    # [m] table margin behind the box's start face
TABLE_MARGIN_Y1 = 0.10    # [m] table margin beyond the box's final position
# Clearance at the push hover (CoM 1.5 m, arm q=0), same measured stack as the
# pick demo: EE 1.308, finger tips 1.184, and the landing legs at 1.235 —
# 51 mm ABOVE the tips, so the tips (not the legs) set the table clearance.

# Select the trajectory from the in-script MODE knob (AM_SWEEP's traj_type
# overrides it for batch runs). set_traj_type validates the name eagerly
# against the planner's catalogue, so a typo fails at startup — before the
# stage loads — with the full list of valid names.
TRAJ_TYPE = P.set_traj_type(SWEEP.get("traj_type", MODE))

# ── Control parameters: robotic_arm/config/push.yaml ─────────────────────
# EVERY gain and law knob comes from that file — there are no gain defaults in
# the controller any more (the three scenario controllers merged into one on
# 2026-08-09; what differs between free flight, pick and push is now only these
# numbers). AM_SWEEP still overrides for batch runs, with its historical
# UPPER-CASE keys mapped onto the file's names.
_SWEEP_ALIAS = {"DLS_LAMBDA": "dls_lambda", "USE_GMO": "use_gmo",
                "TAU_MAX": "tau_max", "M_Y": "M_Y",
                "k_x": "k_x", "k_v": "k_v", "k_R": "k_R", "k_w": "k_w",
                "K_y": "K_y", "D_y": "D_y", "K_o": "K_o", "M_r_d": "M_r_d"}
_overrides = {_SWEEP_ALIAS[k]: v for k, v in SWEEP.items() if k in _SWEEP_ALIAS}
CFG = CP.load("push", overrides=_overrides or None)

# landing protection is only meaningful for a trajectory whose final phase
# descends to the ground (see the LAND_* block above)
LAND_ENABLE = TRAJ_TYPE == "push_home"

# ── Spawn arm pose ───────────────────────────────────────────────────────────
# The arm SPAWNS at the pose the takeoff hold will command (the folded HOME pose
# for "push_home") instead of hanging at q = 0. At q = 0 the finger tips
# reach 0.356 m below the base — 51 mm BELOW the floor at the 0.305 m resting
# body height — so spawning hung would start the run with the gripper through
# the ground and sweep it along the floor for the first seconds of the climb.
# Folded, the lowest arm point (the elbow) keeps 0.140 m of clearance and the
# hold starts exactly ON target, so the arm holds one pose continuously from
# spawn through the climb, and the landing protection holds that same pose back
# down to touchdown. Applied by writing the articulation joint state after
# _art.initialize() — NOT by authoring USD JointStateAPI attrs, which do not
# survive this startup sequence (see the set_joint_positions call).
def _takeoff_arm_pose():
    cfg = P.TRAJ_CONFIG.get(TRAJ_TYPE, {})
    for key in ("q_home", "q_hold"):
        if key in cfg:
            return np.asarray(cfg[key], float)
    return np.zeros(4)

Q_SPAWN = _takeoff_arm_pose()


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


def _tilt_deg(quat_wxyz):
    """Angle between the box's own +z and world +z [deg] — the tip-over
    indicator. The box is spawned upright, so anything past a few degrees while
    the fingers are on it means the shove is tipping it instead of sliding it
    (the failure the box was resized and the table made slippery to prevent)."""
    w, x, y, z = np.asarray(quat_wxyz, float)
    # third column of the rotation matrix = the body z-axis in world
    bz = np.array([2 * (x * z + w * y), 2 * (y * z - w * x),
                   1 - 2 * (x * x + y * y)])
    return math.degrees(math.acos(max(-1.0, min(1.0, float(bz[2])))))


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
    return np.array(q, float)


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
        self._zero_arm_joint_states()     # start arm at 0, not the baked state
        self._setup_gripper_drive()       # stiff position drive + OPEN target (bakes pre-play)
        self._obj = None
        self._spawn_push_props()          # long table + box (push_home only)
        self.world.reset()
        self.stage = omni.usd.get_context().get_stage()

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

        # SPAWN the arm AT the takeoff-hold pose (Q_SPAWN, the folded home) by
        # writing the joint state directly into the articulation view. This must
        # happen HERE — post-reset, before any physics stepping: authoring the
        # USD JointStateAPI attrs pre-reset does NOT take effect on this rig
        # (the pre-reset world.step() calls in _wait_for_prim let PhysX capture
        # its reset-snapshot at the parsed q=0, and world.reset() then restores
        # that snapshot over the authored values).
        # PhysX roots this asset's reduced-coordinate tree at an ARM link, so the
        # joint teleport swings the BODY around the arm instead of the arm around
        # the body. The teleport is therefore followed by a RIGID RE-SEAT:
        # measure the body pose before/after the joint write and left-apply the
        # world-frame correction to the articulation root, moving the whole
        # vehicle rigidly (joint angles untouched) back to its level spawn
        # placement. If a future asset roots the tree at the body, this is an
        # exact no-op.
        if np.any(np.abs(Q_SPAWN) > 1e-9):
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
        # spawn, and the landing setpoint then brings it back to the same
        # height. A pure vertical world translation of the articulation root:
        # joint angles and attitude untouched, exact regardless of where PhysX
        # roots the tree. At the folded home pose the lowest arm point (the
        # elbow, 0.165 m below the base) keeps ~0.14 m of ground clearance.
        # ONLY for the ground-to-ground mission (LAND_ENABLE), because seating
        # requires the FOLDED arm: the airborne-only modes ("hover", "circle",
        # "circle_bent") spawn the arm hanging at q = 0, whose finger tips reach
        # 0.356 m below the base — 51 mm THROUGH the floor at the resting
        # height. Those modes neither start nor end on the ground, so they keep
        # the authored midair SPAWN_POS and simply fly from there.
        if LAND_ENABLE:
            pose_g = self._dc.get_rigid_body_pose(self._body)
            dz = GROUND_BODY_Z - float(pose_g.p.z)
            pr, qr = self._art.get_world_pose()
            self._art.set_world_pose(
                position=np.asarray(pr, float) + np.array([0.0, 0.0, dz]))
            print(f"[InProc] ground-seated: body z {pose_g.p.z:.3f} -> "
                  f"{GROUND_BODY_Z:.3f} m (dz={dz:+.3f})", flush=True)
        else:
            print(f"[InProc] airborne-only mode '{TRAJ_TYPE}': no ground seat "
                  f"(arm hangs at q=0 — it would be through the floor); "
                  f"spawning at the authored z={SPAWN_POS[2]:.2f} m", flush=True)

        # --- Pegasus backend (write rotor input_ref directly) ---------------
        self._vehicle = VehicleManager.get_vehicle_manager().get_vehicle(self.drone_path)
        self._backend = self._vehicle._backends[0]

        # --- Controller + mixer (in-process, controller.py) ----------------
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
        # gripper dof ALSO in effort mode (torque grip). Both arm+gripper efforts
        # then go through ONE set_joint_efforts call; leaving the gripper on its
        # position drive made effort writes no-op (grip_q stuck at 0) and, mixed
        # with apply_action, wiped the arm efforts.
        if self._grip_idx is not None:
            actrl.switch_dof_control_mode(dof_index=self._grip_idx, mode="effort")
        print(f"[InProc] arm dofs {self._arm_idx} + gripper dof {self._grip_idx} in effort mode", flush=True)

        self._tr = None
        self._t = 0.0
        # three-phase state machine (see the FLIGHT PHASES block). Transitions
        # are one-way: takeoff → task → landing, each latched when its hover
        # gate clears.
        self._phase = "takeoff"
        self._sp_takeoff = None     # CoM setpoint: the task trajectory's start point
        self._sp_task_end = None    # task end point (set at re-anchor; landing gate)
        self._sp_land = None        # CoM setpoint: task end xy at LAND_SP_COM_Z
        self._release_t = None      # task-clock zero: stamped at the takeoff→task
                                    # handover
        self._land_start_t = None   # sim time the landing phase began (phase table)
        self._descend = False       # landing: True once the arm is fully on the
                                    # PD and the descent setpoint is commanded
        self._x_c = None            # measured CoM, fed forward one step (hover gates)
        self._gate_ok_since = None  # when the hover gate last became satisfied
                                    # (dwell timer; None = currently violated)
        self._hold_ref = None       # slewed arm-hold reference (see ARM_HOLD_RATE)
        self._touchdown = False     # True once settled at the landing setpoint
        self._disarmed = False      # True once the post-landing rotor ramp finishes
        self._disarm_t = None       # sim time the ramp finished (CLOSE_DELAY)
        self._land_t = 0.0          # elapsed time in the rotor ramp-off
        # two-panel terminal frame (see the PANEL_* block)
        self._events = []           # (t, label, colour) flight-phase log
        self._panel_t = -1e9        # last frame render (PANEL_PERIOD throttle)
        self._gate_txt = ""         # live pending-gate line, "" when none
        self._sat_warned = False    # arm saturation already reported once
        self._n_sat = 0             # joints at +-TAU_MAX this step (law + hold)
        self._grip_cmd = 0.0                     # applied grip effort [N·m] (torque mode)
        self._grip_q = 0.0                       # measured hub angle [rad], starts at rest
        self._obj_p0 = None                      # box position at t=0 (slide distance)
        if not hasattr(self, "_grip_idx"):
            self._grip_idx = None
        # history buffers for MATLAB-style plotting (saved on exit → plot script)
        self._hist = {k: [] for k in
                      ("t", "p", "v0", "omega0", "e_R", "e_y", "q", "qdot",
                       "thrust", "tau_j", "u3",
                       "t_thrust", "t_imp", "t_dist", "t_cpl", "tau_b", "tau_b_arm",
                       "grip", "grip_q",
                       "x_c", "x_cd", "r_e", "r_ed", "q_d", "obj_p", "obj_q",
                       # GMO disturbance estimates (transformed frame) + applied EE force
                       "d_t_hat", "d_r_hat", "d_rho_hat", "ee_force")}
        # seed the panel's phase log so the LEFT column is never blank
        self._events.append((0.0, (f"seated z={GROUND_BODY_Z:.3f}" if LAND_ENABLE
                                   else f"air start z={SPAWN_POS[2]:.2f}"),
                             PHASE_COLOR["init"]))
        carb.log_info("[InProc] ready")

    # ── terminal frame ──────────────────────────────────────────────────────

    def _event(self, label, phase="init"):
        """Record a flight-phase event for the LEFT panel. Events are rare (~10
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
        ev = self._events[-max(6, min(len(self._events), 14)):]
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
            # here the closed jaws are the PUSHER, so the commanded effort and
            # the angle it actually holds are both worth seeing — a hub drifting
            # open mid-push means the pusher is deforming under the contact load.
            _row("GRIP", "effort", "[N.m]", f"{self._grip_cmd:+8.2f}"),
            _row("", "angle", "[deg]", f"{math.degrees(self._grip_q):+8.2f}"),
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
            # DIST is the whole point of this rig: while the fingers are on the
            # box, d_t_hat IS the measured contact force the GMO is rejecting.
            _row("DIST", "force", "[N]", _vec(res["d_t_hat"], "%+8.2f")),
            _row("", "moment", "[N.m]", _vec(res["d_r_hat"], "%+8.2f")),
            _row("", "joint", "[N.m]", _vec(res["d_rho_hat"], "%+8.2f")),
        ]
        # the box: how far it has actually been shoved (the mission's one
        # number) and how far from upright it is (the tip-over failure mode).
        if self._obj is not None and self._obj_p0 is not None:
            try:
                pos, orn = self._obj.get_world_pose()
                slide = float(np.asarray(pos, float)[1] - self._obj_p0[1])
                rows.append(_row("BOX", "pos", "[m]", _vec(pos, "%+8.3f")))
                rows.append(_row("", "slide +y", "[m]", f"{slide:8.3f}"))
                rows.append(_row("", "tilt", "[deg]", f"{_tilt_deg(orn):8.2f}"))
            except Exception:
                pass
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
        # this controller family ties the observer to `hold` directly
        # (gmo_active = USE_GMO and not hold), so law and GMO always switch
        # together — no separate inhibit flag to report.
        gmo = "on" if res.get("gmo_active") else "off"
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
        h["grip"].append(float(getattr(self, "_grip_cmd", 0.0)))
        h["grip_q"].append(float(getattr(self, "_grip_q", 0.0)))
        # actual vs reference (CoM / EE / arm) — for the MATLAB-style plots
        h["x_c"].append(np.asarray(x_c, float).copy())
        h["x_cd"].append(np.asarray(ref["x_cd"], float).copy())
        h["r_e"].append(np.asarray(r_e, float).copy())
        h["r_ed"].append(np.asarray(ref["r_ed"], float).copy())
        h["q_d"].append(np.asarray(ref.get("q_d", np.zeros(4)), float).copy())
        # object world pose (push props) — position proves/disproves the push,
        # orientation (wxyz) catches a tip-over the position trace can miss
        if self._obj is not None:
            try:
                pos, orn = self._obj.get_world_pose()
                h["obj_p"].append(np.asarray(pos, float).copy())
                h["obj_q"].append(np.asarray(orn, float).copy())
            except Exception:
                h["obj_p"].append(np.zeros(3))
                h["obj_q"].append(np.array([1.0, 0.0, 0.0, 0.0]))
        else:
            h["obj_p"].append(np.zeros(3))
            h["obj_q"].append(np.array([1.0, 0.0, 0.0, 0.0]))
        for k in ("t_thrust", "t_imp", "t_dist", "t_cpl"):
            h[k].append(np.asarray(res[k], float).copy())
        # GMO disturbance estimates (active from t=0 by default; zero only if
        # USE_GMO is off, or during the climb when TAKEOFF_ARM_HOLD is enabled)
        for k in ("d_t_hat", "d_r_hat", "d_rho_hat"):
            h[k].append(np.asarray(res[k], float).copy())
        # applied EE disturbance force (world frame; zero until it turns on)
        h["ee_force"].append(np.asarray(getattr(self, "_ee_force_now", np.zeros(3)), float).copy())

    def _phase_table(self):
        """Flight-phase boundaries (sim time) + names, recorded in the npz so
        plot_results.py can shade each phase without re-deriving them from a
        config that may have changed since the run. Always starts with the
        takeoff SETPOINT phase, whose length is however long the hover gate
        took (self._release_time(), not a fixed TAKEOFF_TIME); a
        schedule-driven trajectory then contributes one band per segment (its
        "name"), anything else a single band; the post-plan hover wait is
        'hold' and the setpoint descent (from the hover-gated switch on)
        'landing'."""
        edges, names = [0.0, float(self._release_time())], ["takeoff"]
        tr = self._tr or {}
        t_task0 = edges[-1]
        for seg in tr.get("sched", []):
            edges.append(t_task0 + float(seg["t1"]))
            names.append(str(seg.get("name", "track")))
        if not tr.get("sched") and tr.get("T"):
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
        # push_log.npz — one npz per rig in this shared LOG_DIR (gmo_log.npz,
        # pick_log.npz, track_log.npz, ...). A shared name would let whichever
        # rig ran last silently overwrite the other's data, and plot_results'
        # "newest log" default would then plot a run from the wrong rig.
        path = SWEEP.get("log_path") or os.path.join(LOG_DIR, "push_log.npz")
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

    def _setup_gripper_drive(self):
        """gripper_joint in TORQUE (effort) mode: stiffness 0 so there is NO position
        servo — the grip is a constant closing torque applied (in its own call) in
        _apply_gripper. A little damping keeps the close stable. Runs before
        world.reset() so the gains bake into PhysX."""
        stage = omni.usd.get_context().get_stage()
        root = stage.GetPrimAtPath(self.drone_path)
        prim = next((p for p in Usd.PrimRange(root)
                     if p.GetName() == GRIPPER_JOINT and p.IsA(UsdPhysics.RevoluteJoint)), None)
        if prim is None:
            print(f"[InProc] WARNING: gripper joint '{GRIPPER_JOINT}' not found — "
                  f"grip commands will be ignored", flush=True)
            return
        drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
        drive.GetStiffnessAttr().Set(0.0)              # no position servo (effort mode)
        drive.GetDampingAttr().Set(GRIP_DAMPING)       # light damping for a stable close
        drive.GetMaxForceAttr().Set(1000.0)
        drive.GetTargetPositionAttr().Set(GRIPPER_REST_DEG)
        print(f"[InProc] gripper drive set to TORQUE mode on {prim.GetPath()} "
              f"(stiffness 0, damping {GRIP_DAMPING}); grip via constant effort", flush=True)

    def _spawn_table(self, tag, xy, top_z, size_x=TABLE_XY, size_y=TABLE_XY):
        """A table (top slab + 4 legs) whose TOP SURFACE sits at top_z. The top
        may be rectangular (size_y ≠ size_x) — the push table spans the whole
        slide, unlike the pick demo's square tables. Returns the top-slab
        object so the caller can put a physics material on the sliding surface
        (PhysX's default friction combine is AVERAGE, so only making the BOX
        slippery would still leave μ_eff ≈ half the default-material value)."""
        from omni.isaac.core.objects import FixedCuboid
        top = FixedCuboid(prim_path=f"/World/{tag}_table_top", name=f"{tag}_table_top",
                          position=np.array([xy[0], xy[1], top_z - TABLE_TOP_THK / 2.0]),
                          scale=np.array([size_x, size_y, TABLE_TOP_THK]),
                          color=np.array([0.55, 0.42, 0.28]))
        leg_h = max(top_z - TABLE_TOP_THK, 1e-3)          # slab underside -> floor
        dx = size_x / 2.0 - TABLE_LEG_INSET - TABLE_LEG / 2.0
        dy = size_y / 2.0 - TABLE_LEG_INSET - TABLE_LEG / 2.0
        for i, (sx, sy) in enumerate(((+1, +1), (+1, -1), (-1, +1), (-1, -1))):
            FixedCuboid(prim_path=f"/World/{tag}_table_leg{i}",
                        name=f"{tag}_table_leg{i}",
                        position=np.array([xy[0] + sx * dx, xy[1] + sy * dy, leg_h / 2.0]),
                        scale=np.array([TABLE_LEG, TABLE_LEG, leg_h]),
                        color=np.array([0.30, 0.25, 0.20]))
        return top

    def _spawn_push_props(self):
        """ONE long table under the push line + the box standing on it just
        ahead of the fingers at the push start (see the PUSH_PROPS block for
        how the positions are derived). Runs before world.reset() so physics
        picks the props up cleanly."""
        if not PUSH_PROPS or TRAJ_TYPE != "push_home":
            return
        try:
            from omni.isaac.core.objects import DynamicCuboid
            cfg = P.TRAJ_CONFIG[TRAJ_TYPE]
            qP = np.array(cfg["q_push"], float)
            # anchor xy comes from the CoM at the SPAWN pose (the arm is folded
            # there, which moves the CoM relative to the body)
            _, r_c0, _, _ = P.arm_fk(Q_SPAWN)
            r_eP, r_cP, _, _ = P.arm_fk(qP)
            anchor = np.array([SPAWN_POS[0] + r_c0[0], SPAWN_POS[1] + r_c0[1],
                               P.TAKEOFF_ALTITUDE])
            ps = (np.array(cfg["push_wp"], float)
                  + np.array([0.0, 0.0, cfg.get("dz_push", 0.0)]))
            # ee = where the EE ORIGIN sits when the vehicle hovers at the push
            # START; every prop height/offset is an EE-frame offset from it
            # (finger tips, pad leading face, gripper-body ceiling).
            ee = anchor + ps + (r_eP - r_cP)
            push_dist = float(cfg["push_dist"])
            travel = push_dist - BOX_GAP_Y             # expected box slide [m]
            table_top = float(ee[2] + FINGER_TIP_OFF - PUSH_TIP_CLEAR)
            near_y = float(ee[1] + FINGER_LEAD_Y + BOX_GAP_Y)   # box near face
            # the table spans the whole slide, with margins fore and aft
            y0 = near_y - TABLE_MARGIN_Y0
            y1 = near_y + float(BOX_XYZ[1]) + travel + TABLE_MARGIN_Y1
            table = self._spawn_table("push", np.array([ee[0], 0.5 * (y0 + y1)]),
                                      table_top, size_x=TABLE_XY, size_y=(y1 - y0))
            self._obj = DynamicCuboid(
                prim_path="/World/push_object", name="push_object",
                position=np.array([ee[0], near_y + BOX_XYZ[1] / 2.0,
                                   table_top + BOX_XYZ[2] / 2.0]),
                scale=BOX_XYZ.copy(),
                color=np.array([0.9, 0.15, 0.15]), mass=BOX_MASS)
            box_top_off = (table_top + float(BOX_XYZ[2])) - ee[2]   # EE frame
            tip_clear = (ee[2] + FINGER_TIP_OFF) - table_top
            print(f"[InProc] push props: table top z={table_top:.3f} m "
                  f"({TABLE_XY:.2f} x {y1 - y0:.2f} m, y {y0:.3f}..{y1:.3f}), "
                  f"box {BOX_XYZ.round(3)} m {BOX_MASS:.2f} kg, near face "
                  f"{BOX_GAP_Y*1000:.0f} mm ahead of the pads, top at "
                  f"EE{box_top_off*1000:+.0f} mm (ceiling "
                  f"{BOX_TOP_CEIL_EE*1000:+.0f}), finger tips clear the table "
                  f"by {tip_clear*1000:.0f} mm, expected slide ~{travel*1000:.0f} mm",
                  flush=True)
            if box_top_off > BOX_TOP_CEIL_EE:
                print(f"[InProc] WARNING: box top (EE{box_top_off*1000:+.0f} mm) is "
                      f"ABOVE the gripper-body ceiling ({BOX_TOP_CEIL_EE*1000:+.0f}) — "
                      f"the box would ride the plate/hub, not the fingers", flush=True)
            try:
                from omni.isaac.core.materials import PhysicsMaterial
                mat = PhysicsMaterial("/World/Physics/obj_material",
                                      static_friction=BOX_FRICTION[0],
                                      dynamic_friction=BOX_FRICTION[1],
                                      restitution=0.0)
                # SAME material on the box AND the table top: PhysX combines
                # friction by AVERAGING the two materials, so a slippery box on
                # a default-material table would still see μ_eff ≈ 0.35.
                self._obj.apply_physics_material(mat)
                table.apply_physics_material(mat)
            except Exception as exc:
                print(f"[InProc] box/table friction material skipped ({exc})", flush=True)
        except Exception as exc:
            self._obj = None
            print(f"[InProc] WARNING: push props failed ({exc}) — "
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

    def _gripper_effort(self, grip_closed):
        """Constant grip TORQUE from the trajectory's grip flag (effort mode). Negative
        (closing) squeeze keeps the jaws shut — here the closed fingers are the
        PUSHER, they never hold anything; positive spreads the jaws. Stores the effort
        in self._grip_cmd for logging and RETURNS it (N·m) so the caller applies it in
        the SAME set_joint_efforts call as the arm — both dofs are in effort mode, so
        one consistent write drives both. Returns None if the gripper is absent."""
        if self._grip_idx is None:
            return None
        tau = -GRIP_CLOSE_EFFORT if grip_closed >= 0.5 else GRIP_OPEN_EFFORT
        self._grip_cmd = float(tau)     # logged as applied effort [N·m]
        return float(tau)

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
            # land_in_plan=False: the plan ends at land_wp at cruise altitude and
            # the demo's LANDING phase flies the descent as one setpoint, with
            # the arm handed back to the PD hold FIRST (see the SEQUENCING RULE).
            self._tr = P.build_traj(np.array([x_c[0], x_c[1], P.TAKEOFF_ALTITUDE]),
                                    R0 @ (dyn0["r_0e_0"] - dyn0["r_0c_0"]),
                                    R0 @ np.array([1.0, 0, 0]),
                                    land_in_plan=not LAND_ENABLE)
            # TAKEOFF SETPOINT = the task trajectory's own start point, read
            # straight from the anchored plan (no climb ramp — one setpoint,
            # like a real flight). The vertical channel is critically damped
            # (zeta = k_v/(2*sqrt(k_x)) = 1), so the step flies a clean
            # no-overshoot climb on the position loop alone.
            self._sp_takeoff = np.asarray(
                P.generate_reference(0.0, self._tr)["x_cd"], float).copy()
            self._t = 0.0
            if self._obj is not None:
                try:
                    pos, _ = self._obj.get_world_pose()
                    self._obj_p0 = np.asarray(pos, float).copy()
                except Exception:
                    self._obj_p0 = None
            print(f"[InProc] first p0={p0.round(3)} traj='{self._tr['type']}' "
                  f"takeoff setpoint x_cd={self._sp_takeoff.round(3)}  dt={dt:.4f}",
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
                # Prop placement is UNAFFECTED: the table and box were spawned
                # from the same nominal anchor, and the re-anchor only absorbs
                # the ~0.1 m hover offset the vehicle is actually sitting at
                # (the push itself keeps its BOX_GAP_Y approach margin).
                dyn_a = C.dynamics(self._build_X(R0, p0, q, v0, omega0, qdot),
                                   self.params)
                x_c_now = p0 + R0 @ dyn_a["r_0c_0"]
                self._tr = P.build_traj(x_c_now,
                                        R0 @ (dyn_a["r_0e_0"] - dyn_a["r_0c_0"]),
                                        R0 @ np.array([1.0, 0, 0]),
                                        land_in_plan=not LAND_ENABLE)
                # task END point (re-anchored): the landing gate's target and
                # the landing setpoint's xy
                self._sp_task_end = np.asarray(P.generate_reference(
                    float(self._tr.get("T", 0.0)), self._tr)["x_cd"], float).copy()
                # Task clock starts HERE. Safe to switch the arm and start the
                # plan together: the schedule's first segment is a rest hold at
                # the anchor (no derivative step), and the re-anchor moves the
                # reference ONTO the vehicle (e_x, e_y -> 0) rather than away
                # from it. Only a step that CREATES error needs isolating.
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
            # THE isolation that keeps the arm handover clean. This branch
            # cannot run on the step the phase became "landing" (the elif chain
            # took the task branch then), so by the time it does, ctrl.hold has
            # been True for a full control call: u3 is already zero and the law
            # no longer touches the arm. Only now is the descent step commanded,
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
        # SEQUENCING RULE). hold=True zeroes u3 BEFORE τ is formed, so tau_body
        # never pre-compensates a reaction the arm is not producing, and it is
        # also what freezes the GMO for the two ground phases.
        self.ctrl.hold = arm_hold

        # References: takeoff and landing are SETPOINTS (one point each, no
        # profile), the task phase runs the schedule. The landing phase HOLDS
        # the task end point on the step the arm switches back to the PD, and
        # steps to the descent setpoint only once _descend latches on the
        # following step (see the SEQUENCING RULE). The hold's q_hold is passed
        # through as q_d so the recorded joint reference shows the pose the arm
        # is actually being held at; grip stays 0 (jaws open) in both setpoint
        # phases — the jaws only close for the push itself, and the retreat leg
        # has already opened them again before the plan ends.
        q_hold = self._tr.get("q_hold", None)
        if self._phase == "takeoff":
            ref = P.setpoint_reference(self._sp_takeoff, self._tr, q_d=q_hold)
        elif self._phase == "landing":
            ref = P.setpoint_reference(
                self._sp_land if self._descend else self._sp_task_end,
                self._tr, q_d=q_hold)
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
        # (res["tau_joint"]) — no joint-space PD, the disturbance observer
        # carries the push contact force. The software PD hold owns the arm
        # during takeoff (when TAKEOFF_ARM_HOLD) and landing. One actuation
        # path, one clamp.
        if arm_hold:
            g_arm = dyn["g"][6:]
            # ONE hold target for both ends of the flight: the trajectory's
            # q_hold (the folded HOME pose). The plan starts AND ends there —
            # takeoff pre-positions the arm for zero initial joint/EE error,
            # and the landing hold keeps the same pose the fly-land leg already
            # folded back to, so nothing moves at touchdown.
            q_target = q_hold if q_hold is not None else np.zeros_like(q)
            # SLEW the hold reference toward the target rather than stepping it
            # (see ARM_HOLD_RATE): a 40° step would overshoot into q2's hard stop.
            if self._hold_ref is None:
                self._hold_ref = np.asarray(q, float).copy()
            self._hold_ref = self._hold_ref + np.clip(
                q_target - self._hold_ref, -ARM_HOLD_RATE * dt, ARM_HOLD_RATE * dt)
            tau_j = -ARM_HOLD_KP * (q - self._hold_ref) - ARM_HOLD_KD * qdot + g_arm
        else:
            self._hold_ref = None      # re-seed from the live q on re-engagement
            tau_j = res["tau_joint"]   # ctrl.hold=False, so this is the full law
        # Final actuator guard, covering BOTH paths. This scenario's YAML sets
        # tau_max: null, so the LAW does not clamp and this is the only clamp —
        # the controller never learns about it (the base moment it
        # returned still pre-compensates the DEMANDED arm reaction, not the
        # realized one). That makes saturation worth reporting rather than
        # absorbing silently — and on this rig it is a real possibility, since
        # the push contact loads the arm against the clamp.
        tau_pre = tau_j
        tau_j = np.clip(tau_j, -TAU_MAX, TAU_MAX)
        # Saturation is a FAULT signal, not routine: the arm cannot deliver what
        # it is being asked for, so it tracks worse than commanded. Reported
        # once; the panel header then carries the live count.
        self._n_sat = int(np.count_nonzero(tau_j != tau_pre))
        if self._n_sat and not self._sat_warned:
            self._sat_warned = True
            src = "hold" if arm_hold else "law"
            self._event(f"ARM SAT {self._n_sat}/4 {src} @{TAU_MAX}", "warn")
        # ONE effort write for BOTH the arm and the (effort-mode) gripper. Both dofs
        # are in effort mode, so a single set_joint_efforts drives them consistently.
        # Sort by dof index so efforts stay matched to indices regardless of any
        # internal ordering. gripper effort = constant closing/opening squeeze.
        eff = np.asarray(tau_j, float)
        idx = np.asarray(self._arm_idx)
        tau_grip = self._gripper_effort(ref.get("grip", 0.0))
        if tau_grip is not None and self._grip_idx is not None:
            eff = np.append(eff, tau_grip)
            idx = np.append(idx, self._grip_idx)
        order = np.argsort(idx)
        self._art.set_joint_efforts(eff[order], joint_indices=idx[order])

        # --- End-effector disturbance force (the TASK): a constant WORLD-frame
        # force on the gripper, re-applied EVERY step (apply_body_force is one-shot),
        # once we are hovering. Converted to the body's LOCAL frame — the codebase's
        # apply_body_force convention is global=False (see vehicle.py / slung-load).
        self._ee_force_now = np.zeros(3)
        if (EE_FORCE_ENABLE and self._ee_body and self._phase == "task"
                and self._traj_t() >= EE_FORCE_START):
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
            # commands — a physical no-op.
            self.timeline.play()
            self.world.step(render=False)
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
            if render and self._draw is not None:   # draw force arrows in the RENDER context
                self._draw_force_arrows()
        self._save_history()
        self.timeline.stop()
        simulation_app.close()


def main():
    InProcessDemoV2().run()


if __name__ == "__main__":
    main()
