#!/usr/bin/env python
"""
07_px4_t650_aerial_manipulator_valve_turning.py  (2026-10-15)

Author: Shiqi Gao (shiqi.gao907@gmail.com)

THE VALVE-TURNING SCENE on 06's plant -- the Isaac twin of the indoor run for
the whole-body planner's push-and-pull mode in its TURN form (fsc_trajectory_
planner push_pull.hpp, `push_pull_turn_deg`; whole_body_planner/push_pull/*;
the arm ground station's "Valve Turning" tab), flown by the
whole-body 4-D L1 law with its CONTACT phase flag switched on from the grasp
to the release. A physical-interaction task like the box push: the vehicle
grasps the handle on the beam, flies a circle that turns the beam one full
revolution on its bearing, lets go, and flies round to the landing spot.

It does not copy 06: it loads 06 as a module (06 stays byte-identical, and
every plant knob, the servo model, the spawn / re-seat / ground seat and the
ROS 2 arm bridge are the free-flight ones) and subclasses its sim class to add
the scene:

  field     4.5 m (x, forward) by 4.2 m (y), centred on the origin -- not
            drawn: the room's own floor
  vehicle   spawned FACING +x (yaw 0, the arm on the nose), a little off the
            start mark (-1.0, 0), at (-0.90, -0.07) m, as a real start mark is
  table     THE LAB'S CONSOLE TABLE, HOOBRO BB52UXG01G1 (the box-pushing
            scene's, 07_px4_t650_aerial_manipulator_box_pushing.py):
            1.000 m long x 0.150 m deep x 0.800 m tall, IN FRONT of the
            vehicle, its length along y, centred on (0.675, 0): x 0.600..0.750,
            y -0.50..0.50, top at 0.800 m -- the box push's layout since
            2026-10-28 (its 10-21 move: the table at x 1.20 sat near the lab
            field's edge; was centred on (1.20, 0), the start at (0, 0)).
            Static colliders.
  valve     THE USER'S VALVE MECHANISM (Valve_Assembly.stl -> Valve_Assembly
            .usda, 2026-10-15; since 2026-10-28 the DEFAULT is the v2 asset,
            Valve_Assembly_v2.usda: + the fabric pad's half-rings on the hub
            and the five MOCAP MARKERS of the experiment): a 150 x 50 x 20 mm base plate with a 608
            bearing, and ONE rotating part on it -- a 50 mm hub, a 225 x 50 x
            14 mm BEAM (0.200 m one way from the axis, 0.025 the other, top
            54 mm above the plate bottom) and, at radius 0.160 m, a vertical
            FIN HANDLE 5 mm thick (radial) x 40 mm wide (tangential) x 130 mm
            tall on a 10 x 50 x 20 mm root, its top 204 mm above the plate
            bottom. BOLTED to the table top (the base is a KINEMATIC rigid
            body: fixed), the plate's long axis along the table's depth, the
            axis on the table's centre line at PEGASUS_VALVE_XY, the beam
            pointing PEGASUS_VALVE_YAW_DEG (default 180: toward the start).
            The rotor is a DYNAMIC rigid body on a z REVOLUTE JOINT at the
            axis (the bearing), with the bearing's viscous damping on the
            joint's drive and a Coulomb friction torque applied every physics
            step (PhysX maximal joints carry no friction knob). The CAD solid
            at uniform PLA density: 243 g, 0.0031 kg m^2 about the axis; the
            scene flies the printed rotor's 204 g (inertia scaled with it,
            0.0026 kg m^2) and the bearing measured for the asset: v2 (the
            pad) 0.072 N.m in the stick-slip drive model (the 10-27 free-spin
            bench), v1 a conservative 0.048 N.m (see PEGASUS_VALVE_FRICTION).

THE TASK (the Box Pushing tab, Turn = 360): Go To Start flies to the fin and
yaws so the nose lies along the turn's TANGENT at the fin (the beam's yaw +
push_pull_yaw_from_box_deg: +90 for a counter-clockwise turn), the claw
0.10 m above and 0.12 m behind the fin's grasp point; Ready To Push opens the
gripper and slides diagonally down-forward onto the fin; Push closes the
gripper, switches the law to CONTACT and ROTATES the claw, the base and the
heading together about the bearing axis by 360 deg (the arm held still, a
rigid rotation of the whole vehicle: the claw on its 0.160 m circle, the base
on a 0.25 m one behind it); Exit To Push opens the gripper, climbs and folds
the arm home; Go To Land turns on clockwise to -360 deg and hovers over the
landing spot 0.5 m behind the start.

GEOMETRY (the planner's push pose [0, -5, 30, 0] deg: the claw 0.187 m ahead
of and 0.275 m under the body origin, pitched 25 deg from vertical; the T650
landing gear: skids 0.2425 m under the origin, 0.28 m apart inside): the grasp
point 25 mm under the fin top = 0.979 m puts the body origin at 1.254 m and
the skids at 1.012 m -- 0.8 cm ABOVE the fin top (1.004), 0.16 m above the
beam top (0.854) and 0.21 m above the table. The 60 deg box-push pose
[0, 26, 34, 0] would put the skids 2.7 cm over the beam: the fin is only
0.18 m above the table at the grasp. The planner checks every step against
the table (push_pull_table) AND the valve's swept circle (push_pull_obstacle,
z_top = the beam top), 5 cm clearance each.

THE GRIP SHIM: the sim claw cannot clamp 5 mm -- its slider-crank stops the
pads ~19 mm apart (~14 mm at the fingertips; measured, pick-and-place
2026-10-07), where the real gripper's foam pads close fully. So the fin's bare
part (above the root) gets an INVISIBLE collider PEGASUS_VALVE_FIN_GRIP_MM
wide (default 20, the box push's), standing in for the foam. 0 = the bare
CAD fin (the claw closes on air).

  mocap     obj_0 = THE ROTOR as Motive defines a rigid body made from its
            markers (2026-10-28, PEGASUS_VALVE_MOCAP_PIVOT=markers, the default
            with the v2 asset): the pivot at the markers' CENTROID (Motive's
            default), the axes the rotor's (+x along the beam toward the fin,
            z up -- the rigid body created with the fin pointing along the
            room's +x on a level table). The five markers of
            Valve_Assembly_v2.stl (redesigned 2026-10-28 night; read from the
            asset's fsc:mocap_*) put it [0.0935, 0.0014, 0.0899] m from the
            bearing axis on the plate bottom, so the planner carries
            push_pull_valve_axis_offset = [-0.0935, -0.0014, -0.0899]: the axis
            point = obj_0 + Rz(yaw) * that (the turn's centre, the keep-outs),
            the grasp point = the axis point + push_pull_ee_offset = [0.160,
            0.0006, 0.179] turned with the beam (the fin's radius and bearing,
            the claw 25 mm under the fin top). =axis (v1's only choice): obj_0
            on the axis at the plate-bottom plane, the axis offset 0.
            claw_0 = the real claw point, ground truth only (not mocap).
            Published as <body>/state/{pose,twist,twist_inertial}; the OptiTrack
            emulator turns obj_0 into /obj_0/mocap (the 4-D stack lists it).

06's EE marker cube is forced OFF here: it would publish obj_0 too.

Knobs (environment):
  PEGASUS_VALVE_XY               the bearing axis "x,y" [m], default "0.675,0.00" (the table's centre)
  PEGASUS_VALVE_YAW_DEG          the beam's initial yaw [deg], default 180 (the fin toward the start)
  PEGASUS_VALVE_RISER            a block under the plate [m], default 0 (a higher valve allows a
                                 more folded arm: the gear is the limit, see GEOMETRY)
  PEGASUS_VALVE_ROTOR_MASS       the rotor's mass [kg], default 0.204 = the printed rotor (the
                                 asset is solid PLA, 0.243; 0 = the asset's). The pick-and-drop
                                 payload, same material and printer, weighs 180 g printed over
                                 172.5 cm^3 of CAD solid = 1044 kg/m^3; the rotor is 195.8 cm^3
  PEGASUS_VALVE_DAMPING          the bearing's viscous damping [N.m.s/rad], default 0.0
  PEGASUS_VALVE_FRICTION         the bearing's Coulomb friction torque [N.m], default per asset:
                                 v2 (the pad, fsc:hub_pad_rings) 0.072 in the drive model = the
                                 10-27 free-spin bench (72 mN.m [63, 84] at the 204 g rotor,
                                 archive/valve_damper_spin_20261027); v1 0.048
                                 (applied as -f tanh(w / 0.05)). CONSERVATIVE, for robustness
                                 (user, 2026-10-17): the real valve, pulled by a 10 N spring
                                 gauge (0.2 N per division) at the fin, never passed 0.2 N =
                                 0.032 N.m at r 0.160 m; 0.048 N.m = 0.30 N at the fin, 1.5x that
                                 bound, at every speed of the turn (the gauge's flat zero was not
                                 checked -- the margin covers up to half a division). A sealed
                                 608 at this load is likely a few mN.m: fly the frictionless end
                                 too (PEGASUS_VALVE_FRICTION=0).
  PEGASUS_VALVE_FRICTION_MODEL   tanh | drive (2026-10-26; default drive with v2, tanh with v1). tanh CANNOT hold the rotor
                                 still: below f it creeps at tau dt / I (8 deg/s at 0.9 x 0.10 N.m)
                                 and its momentum clamp caps the resistance at ~0.65 N.m per rad/s.
                                 drive = stick-slip Coulomb friction in the joint drive (force limit
                                 f): stays put at 0.9 f, slips at f (archive/valve_yaw_sweep_20261026,
                                 tools/friction_check.py). Needs PEGASUS_VALVE_DAMPING 0.
  PEGASUS_VALVE_STICTION_DAMPING the drive model's damping [N.m.s/rad], default 1000
  PEGASUS_VALVE_LOAD_TORQUE      a HANGING-WEIGHT load on the shaft [N.m] = m g r_drum, default 0 =
                                 off (2026-10-21, the user's drum/pulley idea). The string is
                                 just taut at the start (payload on the floor): winding in the
                                 turn's direction lifts it (full torque after
                                 PEGASUS_VALVE_LOAD_TAKEUP_DEG, default 3), unwinding lowers it
                                 and past the start the string is slack (no torque). It is a
                                 CONSTANT torque, not friction: it back-drives the valve the
                                 moment nothing holds it. The payload's own inertia (m r^2,
                                 ~15 % of the rotor's at 0.2 N.m on a 20 mm drum) is neglected,
                                 so a back-drive is slightly faster than real.
  PEGASUS_VALVE_LOAD_DIR         the turn direction the load resists, default +1 (the planned
                                 +360 deg turn winds the string)
  PEGASUS_VALVE_RATCHET=1        a one-way pawl: rotation against LOAD_DIR is stopped and the
                                 pawl carries the weight (it fades in below 0.02 rad/s of forward
                                 rate), default 0
  PEGASUS_VALVE_FIN_GRIP_MM      the fin's clamped width in the sim [mm], default 20; 0 = bare
  PEGASUS_VALVE_USD              the asset, default assets/Valve_Assembly_v2.usda (2026-10-28; was
                                 Valve_Assembly.usda, v1: no pad, no markers)
  PEGASUS_VALVE_MOCAP_PIVOT      markers | axis: where obj_0's origin is published, default markers
                                 with an asset that carries them (fsc:mocap_*), else axis. markers
                                 needs the planner's push_pull_valve_axis_offset = minus the centroid
                                 (the valve-turning yaml carries the v2 asset's); axis needs it 0
  PEGASUS_VALVE_GRIP_TORQUE      gripper drive torque cap [N.m], default 0.3
  PEGASUS_VALVE_SPAWN_XY         where the vehicle starts, "x,y" [m], default "-0.90,-0.07"
  PEGASUS_VALVE_SPAWN_YAW_DEG    its heading [deg], default 0 (+x)
  PEGASUS_VALVE_WAYPOINTS=0      hide the waypoint markers and labels
  PEGASUS_VALVE_AM_ASSET         airframe asset, default AM_T650.usda (the T650's 7 cm shorter gear)
  PEGASUS_VALVE_GROUND_BODY_Z    resting body height [m], default 0.235 with AM_T650 (0.305 with AM_xfwd)
  PEGASUS_VALVE_Q2_LIMIT_DEG     q2's hard stops "lo,hi" [deg], default "-20,45" (the real arm's
                                 range; the asset authors -90,50); "asset" keeps -90,50

WAYPOINT VIEW: blue ball = a drone-body goal (Go To Start's hover by the
handle, the Land hover), orange = an end-effector (claw) target (the grasp
point; after a full turn the turned point coincides with it), after Plan. Each
with a label (coordinates, yaw); the same table prints here whenever it
changes. Visual only.

Run with:
  scripts/indoor_sim/start_t650_aerial_manipulator_whole_body_L1_4D_valve_turning_sitl.sh <config>
"""

import importlib.util
import math
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_spec = importlib.util.spec_from_file_location(
    "am06_plant", os.path.join(_HERE, "06_px4_t650_aerial_manipulator_free_flight.py"))
am06 = importlib.util.module_from_spec(_spec)
sys.modules["am06_plant"] = am06
_spec.loader.exec_module(am06)          # starts the SimulationApp, defines the plant

import numpy as np                      # noqa: E402  (after SimulationApp, see memory note)
import omni.usd                         # noqa: E402
from pxr import Gf, PhysicsSchemaTools, PhysxSchema, Usd, UsdGeom, UsdPhysics, UsdShade  # noqa: E402


def _xy(env, default):
    raw = (os.environ.get(env, "") or default).strip()
    try:
        v = tuple(float(s) for s in raw.split(","))
        assert len(v) == 2
        return v
    except (ValueError, AssertionError):
        raise SystemExit(f"{env} must be 'x,y' in metres, got {raw!r}")


# ── the scene ──────────────────────────────────────────────────────────────
SPAWN_XY = _xy("PEGASUS_VALVE_SPAWN_XY", "-0.90,-0.07")   # the start mark (-1.0, 0) + a placement error (2026-10-28; was 0.10,-0.07)
SPAWN_YAW_DEG = am06._envf("PEGASUS_VALVE_SPAWN_YAW_DEG", 0.0)

# THE TABLE: HOOBRO BB52UXG01G1, 39.4" W x 5.9" D x 31.5" H = 1.000 x 0.150 x
# 0.800 m, the box-pushing scene's (the same constants; it is the experiment's
# table too). Its LENGTH runs along y (left-right in front of the start), its
# depth along x.
TABLE_CENTER_XY = (0.675, 0.0)          # the box push's layout (2026-10-28; was (1.20, 0)): x 0.600..0.750
TABLE_LENGTH_Y  = 1.000                 # [m] 39.4 in
TABLE_DEPTH_X   = 0.150                 # [m] 5.9 in
TABLE_HEIGHT    = 0.800                 # [m] 31.5 in, to the top surface
TABLE_TOP_THICK = 0.018                 # [m] particleboard top
TABLE_LEG       = 0.025                 # [m] square metal legs
TABLE_SHELF_Z   = 0.250                 # [m] the lower shelf's top
TABLE_SHELF_THICK = 0.015               # [m]

# THE VALVE: the asset's frame is the bearing axis on the plate-bottom plane,
# +x along the beam toward the fin (stl_to_usda_valve.py).
VALVE_USD = (os.environ.get("PEGASUS_VALVE_USD", "") or os.path.join(am06.ASSETS_DIR, "Valve_Assembly_v2.usda")).strip()
VALVE_XY  = _xy("PEGASUS_VALVE_XY", "0.675,0.00")
VALVE_YAW_DEG = am06._envf("PEGASUS_VALVE_YAW_DEG", 180.0)
VALVE_RISER = am06._envf("PEGASUS_VALVE_RISER", 0.0)
if not os.path.isfile(VALVE_USD):
    raise SystemExit(f"[AM-T650-VALVE] {VALVE_USD} not found -- generate it with "
                     f"docs/docs_aerial_manipulator/archive/valve_turning_20261015/tools/stl_to_usda_valve.py")
_valve_stage = Usd.Stage.Open(VALVE_USD)        # kept alive: a prim of a dropped stage expires
_root = _valve_stage.GetDefaultPrim()
VALVE = {k: float(_root.GetAttribute(f"fsc:{k}").Get()) for k in (
    "fin_radius_m", "fin_bearing_y_m", "fin_thickness_m", "fin_width_m", "fin_top_z_m",
    "fin_exposed_bottom_z_m", "beam_reach_m", "beam_back_m", "beam_width_m", "beam_top_z_m",
    "plate_size_x_m", "plate_size_y_m", "plate_top_z_m", "rotor_mass_kg", "rotor_inertia_axis_kgm2")}
# (2026-10-28) optional attributes: v2's pad, and the mocap markers (stl_to_usda_valve.py)
def _opt(name, default):
    a = _root.GetAttribute(f"fsc:{name}")
    return float(a.Get()) if a and a.HasAuthoredValue() else default
VALVE_HUB_PAD = _opt("hub_pad_rings", 0.0) > 0.0
_mk = _root.GetAttribute("fsc:mocap_markers_m")
MOCAP_MARKERS = np.array([[float(c) for c in m] for m in _mk.Get()]) if _mk and _mk.HasAuthoredValue() else None
MOCAP_CENTROID = (np.array([_opt("mocap_pivot_x_m", 0.0), _opt("mocap_pivot_y_m", 0.0), _opt("mocap_pivot_z_m", 0.0)])
                  if MOCAP_MARKERS is not None else None)
del _root, _valve_stage, _mk
ROTOR_MASS = am06._envf("PEGASUS_VALVE_ROTOR_MASS", 0.204)     # [kg] the printed rotor; 0 = the asset's
BEARING_DAMPING = am06._envf("PEGASUS_VALVE_DAMPING", 0.0)     # [N.m.s/rad]
# [N.m] per asset (2026-10-28): v2 (the fabric pad on the hub) 0.072 = the 10-27 free-spin bench
# (archive/valve_damper_spin_20261027); v1 (the sealed 608 alone) 0.048 = 1.5x the measured 0.2 N x
# 0.160 m bound
BEARING_FRICTION = am06._envf("PEGASUS_VALVE_FRICTION", 0.072 if VALVE_HUB_PAD else 0.048)
BEARING_FRICTION_W = 0.05               # [rad/s] the tanh width of the friction torque
# THE FRICTION MODEL (2026-10-26, the heading-channel sweep, archive/valve_yaw_sweep_20261026):
#   tanh  (the default; every flight before 2026-10-26) -f tanh(w / 0.05) as an explicit torque,
#         momentum-clamped to I |w| / dt. It cannot hold the rotor still (a torque below f makes it
#         creep at tau dt / I) and never resists with more than I |w| / dt (~0.65 N.m per rad/s at
#         250 Hz), so above ~0.08 N.m the clamp, not f, sets the resistance during a turn.
#   drive stick-slip Coulomb friction INSIDE the solver: the joint's angular drive as a velocity
#         constraint (target 0, damping PEGASUS_VALVE_STICTION_DAMPING) whose force limit is f. It
#         holds any torque below f and resists with f while slipping. Replaces the viscous drive
#         (PEGASUS_VALVE_DAMPING must be 0).
# default: drive with v2 (the pad is Coulomb, static = kinetic on the bench), tanh with v1 (every
# flight before 2026-10-26)
FRICTION_MODEL = (os.environ.get("PEGASUS_VALVE_FRICTION_MODEL", "") or ("drive" if VALVE_HUB_PAD else "tanh")).strip().lower()
STICTION_DAMPING = am06._envf("PEGASUS_VALVE_STICTION_DAMPING", 1000.0)  # [N.m.s/rad] (creep at 0.9 f: 0.013 deg/s)
if FRICTION_MODEL not in ("tanh", "drive"):
    raise SystemExit(f"PEGASUS_VALVE_FRICTION_MODEL must be tanh or drive, got {FRICTION_MODEL!r}")
if FRICTION_MODEL == "drive" and BEARING_DAMPING > 0.0:
    raise SystemExit("PEGASUS_VALVE_FRICTION_MODEL=drive uses the joint drive for the friction: "
                     "PEGASUS_VALVE_DAMPING must be 0")
LOAD_TORQUE = am06._envf("PEGASUS_VALVE_LOAD_TORQUE", 0.0)      # [N.m] hanging weight m g r_drum, 0 = off
LOAD_DIR = 1.0 if am06._envf("PEGASUS_VALVE_LOAD_DIR", 1.0) >= 0.0 else -1.0
LOAD_TAKEUP_DEG = max(0.1, am06._envf("PEGASUS_VALVE_LOAD_TAKEUP_DEG", 3.0))
RATCHET = am06._envf("PEGASUS_VALVE_RATCHET", 0.0) > 0.5
RATCHET_W = 0.02                        # [rad/s] forward rate below which the pawl takes the weight
FIN_GRIP_MM = am06._envf("PEGASUS_VALVE_FIN_GRIP_MM", 20.0)
GRASP_BELOW_TOP = 0.025                 # [m] the claw point under the fin top (the box push's rule)
# THE MOCAP PIVOT (2026-10-28): where obj_0's origin is published, in the rotor frame --
# Motive's DEFAULT pivot of a rigid body made from the markers (their centroid), or the axis
MOCAP_PIVOT_MODE = (os.environ.get("PEGASUS_VALVE_MOCAP_PIVOT", "") or
                    ("markers" if MOCAP_MARKERS is not None else "axis")).strip().lower()
if MOCAP_PIVOT_MODE not in ("markers", "axis"):
    raise SystemExit(f"PEGASUS_VALVE_MOCAP_PIVOT must be markers or axis, got {MOCAP_PIVOT_MODE!r}")
if MOCAP_PIVOT_MODE == "markers" and MOCAP_MARKERS is None:
    raise SystemExit(f"PEGASUS_VALVE_MOCAP_PIVOT=markers: {os.path.basename(VALVE_USD)} carries no mocap markers "
                     f"(regenerate Valve_Assembly_v2.usda from the markers' STL, or use axis)")
MOCAP_PIVOT = MOCAP_CENTROID if MOCAP_PIVOT_MODE == "markers" else np.zeros(3)
# the planner's push_pull_valve_axis_offset to match: obj_0 -> the axis, obj_0's frame
AXIS_OFFSET = tuple(float(-v) for v in MOCAP_PIVOT)
# the planner's push_pull_ee_offset for this valve (rotor frame, from the axis
# on the plate-bottom plane)
EE_OFFSET = (VALVE["fin_radius_m"], VALVE["fin_bearing_y_m"], VALVE["fin_top_z_m"] - GRASP_BELOW_TOP)
VALVE_BASE_Z = TABLE_HEIGHT + VALVE_RISER       # the plate bottom = obj_0's z

FRICTION_STATIC  = 0.29                 # the table's (measured, 2026-10-05): the gear, if it ever lands on it
FRICTION_DYNAMIC = 0.29
GRIP_FRICTION_STATIC  = 1.2             # the fin: wins every pad contact (combine max)
GRIP_FRICTION_DYNAMIC = 1.0
GRIP_TORQUE_MAX = am06._envf("PEGASUS_VALVE_GRIP_TORQUE", 0.3)   # [N.m] gripper drive cap

SCENE_ROOT = "/World/valve_turning"
VALVE_PRIM = SCENE_ROOT + "/valve"
ROTOR_PRIM = VALVE_PRIM + "/rotor"
JOINT_PRIM = VALVE_PRIM + "/joint"
WP_ROOT    = SCENE_ROOT + "/waypoints"
ROTOR_BODY = "obj_0"                    # the rotor, live: the planner's push_pull_box_topic body
CLAW_TRUTH_BODY = "claw_0"              # the real claw point, ground truth only
CLAW_BEYOND_PADS = 0.039                # [m] claw point beyond the pad CoM midpoint (07 pick-and-place's probe)
FIXED_PUB_DIV = 4                       # publish every 4th physics step (62.5 Hz)
# CONTACT TRUTH: the PhysX full contact report on the rotor, summed per publish
# interval and divided by its duration -> mean force; the box push's layout
# (normalised to "the rotor is actor0"; [6:12] -- the table's share -- stays 0,
# the rotor touches nothing but the vehicle):
#   [0:3]  vehicle-rotor normal force   [3:6]  vehicle-rotor friction-anchor force
#   [6:9]  0                            [9:12] 0
#   [12:15] sum p x f, vehicle-rotor normal   [15:18] sum p x f, vehicle-rotor friction
#   (moments about the WORLD origin)
#   [18] interval [s]   [19] number of vehicle-rotor contact points in the interval
CONTACT_TRUTH_TOPIC = "/push_pull_truth/contact"
BEAM_LOG_TURN_DEG = 2.0                 # print the beam's angle when it turned this much
ANGLE_TOPIC = f"/{ROTOR_BODY}/state/angle"   # the beam's angle, unwrapped [rad], + the rate

WAYPOINTS_VIZ = (os.environ.get("PEGASUS_VALVE_WAYPOINTS", "") or "1").strip() != "0"
PL_INFO_TOPIC = "/uav_0/whole_body_planner/push_pull/info"
WP_REFRESH_S = 0.5                      # [s, sim]
WP_BODY_RGB = (0.20, 0.55, 1.00)
WP_CLAW_RGB = (1.00, 0.60, 0.10)
WP_BODY_RADIUS = 0.020
WP_CLAW_RADIUS = 0.015
# planned goals in push_pull/info: [19 + 7k ..] = base x, y, z, yaw_deg, claw x, y, z
# (six steps since 2026-10-16: [18 + 7k] before; 4 = go_to_land_start, 5 = land)
WP_SHOW = ((0, "body", "1 Go To Start drone"), (1, "claw", "2 Grasp EE"),
           (2, "claw", "3 Turned EE"), (4, "body", "5 Land Start drone"),
           (5, "body", "6 Land drone"))

CAMERA_POS    = [-2.3, 2.7, 2.4]        # behind-left of the start, high (2026-10-28: the box push's; was -1.0, 2.4, 2.3)
CAMERA_TARGET = [0.1, 0.0, 0.9]

# THE T650 LANDING GEAR (the box-pushing scene's choice, 2026-10-06): the
# experiment drone is the T650, its gear ~7 cm shorter than the X650 gear
# AM_xfwd.usda carries. AM_T650.usda is a copy of AM_xfwd.usda with the gear
# shortened by 0.070 m: skids 0.2425 m under the body origin. The planner's
# push_pull_gear_depth carries the same 0.2425.
AM_ASSET = (os.environ.get("PEGASUS_VALVE_AM_ASSET", "") or "AM_T650.usda").strip()
am06.USD_FILE = os.path.join(am06.ASSETS_DIR, AM_ASSET)
if not os.path.isfile(am06.USD_FILE):
    raise SystemExit(f"{am06.USD_FILE} not found -- generate it with "
                     f"robotic_arm/utils_model/make_t650_gear_asset.py (or set PEGASUS_VALVE_AM_ASSET)")
am06.GROUND_BODY_Z = am06._envf("PEGASUS_VALVE_GROUND_BODY_Z",
                                0.305 - 0.070 if AM_ASSET == "AM_T650.usda" else 0.305)
print(f"\033[1;35m[AM-T650-VALVE] airframe asset {AM_ASSET} (resting body height "
      f"{am06.GROUND_BODY_Z:.3f} m)\033[0m", flush=True)
# THE REAL ARM'S q2 RANGE (2026-10-06, user): about [-20, 45] deg on the
# hardware, against the asset's [-90, 50]. Authored on manip_joint2's PhysX
# limits at spawn, so in this scene q2 meets a hard stop where the real arm does.
_q2_raw = (os.environ.get("PEGASUS_VALVE_Q2_LIMIT_DEG", "") or "-20,45").strip()
if _q2_raw.lower() == "asset":
    Q2_LIMIT_DEG = None
else:
    try:
        Q2_LIMIT_DEG = tuple(float(v) for v in _q2_raw.split(","))
        assert len(Q2_LIMIT_DEG) == 2 and Q2_LIMIT_DEG[0] < Q2_LIMIT_DEG[1]
    except (ValueError, AssertionError):
        raise SystemExit(f"PEGASUS_VALVE_Q2_LIMIT_DEG must be 'lo,hi' or 'asset', got {_q2_raw!r}")
am06.SPAWN_POS = (float(SPAWN_XY[0]), float(SPAWN_XY[1]), float(am06.SPAWN_POS[2]))
am06.SPAWN_EULER = (0.0, 0.0, SPAWN_YAW_DEG)
if am06.EE_MARKER_CUBE:
    print("\033[1;33m[AM-T650-VALVE] PEGASUS_EE_MARKER_CUBE=1 ignored: obj_0 is the valve's rotor in "
          "this scene, and the marker cube would publish the same body.\033[0m", flush=True)
am06.EE_MARKER_CUBE = False


def _color(prim_geom, rgb):
    prim_geom.CreateDisplayColorAttr([Gf.Vec3f(*map(float, rgb))])


def _box(stage, path, center, size, rgb):
    """A unit cube scaled to `size` at `center` (in its parent's frame)."""
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    xf = UsdGeom.Xformable(cube)
    xf.AddTranslateOp().Set(Gf.Vec3d(*map(float, center)))
    xf.AddScaleOp().Set(Gf.Vec3f(*map(float, size)))
    _color(cube, rgb)
    return cube


def _physics_material(stage, path, static, dynamic, combine=None):
    mat = UsdShade.Material.Define(stage, path)
    pm = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    pm.CreateStaticFrictionAttr(float(static))
    pm.CreateDynamicFrictionAttr(float(dynamic))
    pm.CreateRestitutionAttr(0.0)
    if combine:
        PhysxSchema.PhysxMaterialAPI.Apply(mat.GetPrim()).CreateFrictionCombineModeAttr().Set(combine)
    return mat


def _bind_physics(prim, mat):
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(
        mat, UsdShade.Tokens.weakerThanDescendants, "physics")


class AmT650ValveTurning(am06.AmT650WholeBodyArmSim):

    # 06 calls this right after loading the environment and before
    # world.reset(): the scene exists when PhysX first reads the stage.
    def _spawn_am_px4_primary(self):
        self._build_scene(omni.usd.get_context().get_stage())
        drone_path = super()._spawn_am_px4_primary()
        self._set_q2_limit(drone_path)    # 06 assigns self.drone_path from this return value
        return drone_path

    def _set_q2_limit(self, drone_path):
        if Q2_LIMIT_DEG is None:
            print("[AM-T650-VALVE] q2 stops: the asset's", flush=True)
            return
        stage = omni.usd.get_context().get_stage()
        root = stage.GetPrimAtPath(drone_path)
        prim = next((p for p in Usd.PrimRange(root)
                     if p.GetName() == "manip_joint2" and p.IsA(UsdPhysics.RevoluteJoint)), None)
        if prim is None:
            raise SystemExit("[AM-T650-VALVE] no manip_joint2 to put the q2 stops on")
        j = UsdPhysics.RevoluteJoint(prim)
        old = (j.GetLowerLimitAttr().Get(), j.GetUpperLimitAttr().Get())
        j.GetLowerLimitAttr().Set(float(Q2_LIMIT_DEG[0]))
        j.GetUpperLimitAttr().Set(float(Q2_LIMIT_DEG[1]))
        print(f"\033[1;35m[AM-T650-VALVE] q2 hard stops {Q2_LIMIT_DEG[0]:.0f}..{Q2_LIMIT_DEG[1]:.0f} deg "
              f"(the real arm's range; asset {old[0]:.0f}..{old[1]:.0f})\033[0m", flush=True)

    def _build_scene(self, stage):
        UsdGeom.Xform.Define(stage, SCENE_ROOT)
        slide_mat = _physics_material(stage, SCENE_ROOT + "/slide_material",
                                      FRICTION_STATIC, FRICTION_DYNAMIC)
        grip_mat = _physics_material(stage, SCENE_ROOT + "/grip_material",
                                     GRIP_FRICTION_STATIC, GRIP_FRICTION_DYNAMIC, combine="max")

        # the table: static colliders (no rigid body) -- the box-pushing scene's
        cx, cy = TABLE_CENTER_XY
        wood, metal = (0.42, 0.30, 0.20), (0.15, 0.15, 0.15)
        top = _box(stage, SCENE_ROOT + "/table_top",
                   (cx, cy, TABLE_HEIGHT - 0.5 * TABLE_TOP_THICK),
                   (TABLE_DEPTH_X, TABLE_LENGTH_Y, TABLE_TOP_THICK), wood)
        parts = [top]
        leg_h = TABLE_HEIGHT - TABLE_TOP_THICK
        for i, (sx, sy) in enumerate(((1, 1), (1, -1), (-1, 1), (-1, -1))):
            lx = cx + sx * (0.5 * TABLE_DEPTH_X - 0.5 * TABLE_LEG)
            ly = cy + sy * (0.5 * TABLE_LENGTH_Y - 0.5 * TABLE_LEG)
            parts.append(_box(stage, f"{SCENE_ROOT}/table_leg{i}", (lx, ly, 0.5 * leg_h),
                              (TABLE_LEG, TABLE_LEG, leg_h), metal))
        parts.append(_box(stage, SCENE_ROOT + "/table_shelf",
                          (cx, cy, TABLE_SHELF_Z - 0.5 * TABLE_SHELF_THICK),
                          (TABLE_DEPTH_X, TABLE_LENGTH_Y - 2 * TABLE_LEG, TABLE_SHELF_THICK), wood))
        if VALVE_RISER > 0.0:
            parts.append(_box(stage, SCENE_ROOT + "/riser",
                              (VALVE_XY[0], VALVE_XY[1], TABLE_HEIGHT + 0.5 * VALVE_RISER),
                              (VALVE["plate_size_x_m"], VALVE["plate_size_y_m"], VALVE_RISER), (0.3, 0.3, 0.32)))
        for p in parts:
            UsdPhysics.CollisionAPI.Apply(p.GetPrim())
        _bind_physics(top.GetPrim(), slide_mat)

        # the valve: the asset REFERENCED onto an Xform at its placement (the
        # bearing axis on the table top, the beam along VALVE_YAW_DEG). The
        # asset carries the two rigid bodies (base kinematic, rotor dynamic),
        # the revolute joint, the colliders and the rotor's mass properties.
        p0 = (VALVE_XY[0], VALVE_XY[1], VALVE_BASE_Z)
        valve = UsdGeom.Xform.Define(stage, VALVE_PRIM)
        xf = UsdGeom.Xformable(valve)
        xf.AddTranslateOp().Set(Gf.Vec3d(*map(float, p0)))
        xf.AddRotateZOp().Set(float(VALVE_YAW_DEG))
        valve.GetPrim().GetReferences().AddReference(VALVE_USD)
        n_col = 0
        for p in Usd.PrimRange(valve.GetPrim()):
            if p.HasAPI(UsdPhysics.CollisionAPI):
                g = p.GetAttribute("fsc:grip")
                _bind_physics(p, grip_mat if (g and g.Get()) else slide_mat)
                n_col += 1
        rotor = stage.GetPrimAtPath(ROTOR_PRIM)
        if not rotor or not rotor.HasAPI(UsdPhysics.RigidBodyAPI):
            raise SystemExit(f"[AM-T650-VALVE] {ROTOR_PRIM} is not the asset's rotor rigid body -- regenerate the asset")
        # the grip shim: the fin's bare part, FIN_GRIP_MM wide across the jaws
        shim = FIN_GRIP_MM * 1e-3
        if shim > VALVE["fin_thickness_m"]:
            h = VALVE["fin_top_z_m"] - VALVE["fin_exposed_bottom_z_m"]
            g = _box(stage, ROTOR_PRIM + "/grip_shim",
                     (VALVE["fin_radius_m"], VALVE["fin_bearing_y_m"], VALVE["fin_exposed_bottom_z_m"] + 0.5 * h),
                     (shim, VALVE["fin_width_m"], h), (0.30, 0.85, 0.40))
            UsdPhysics.CollisionAPI.Apply(g.GetPrim())
            _bind_physics(g.GetPrim(), grip_mat)
            UsdGeom.Imageable(g.GetPrim()).MakeInvisible()     # physics only: the visual is the CAD fin
            n_col += 1
        mass = UsdPhysics.MassAPI(rotor)
        m_asset = float(mass.GetMassAttr().Get())
        m_rotor = ROTOR_MASS if ROTOR_MASS > 0.0 else m_asset
        if abs(m_rotor - m_asset) > 1e-9:            # same shape, another total: inertia scales with it
            k = m_rotor / m_asset
            mass.GetMassAttr().Set(float(m_rotor))
            mass.GetDiagonalInertiaAttr().Set(Gf.Vec3f(*[float(k * v) for v in mass.GetDiagonalInertiaAttr().Get()]))
        self._rotor_mass = m_rotor
        # the bearing: viscous damping on the joint's angular drive (stiffness 0)
        joint = stage.GetPrimAtPath(JOINT_PRIM)
        if not joint or not joint.IsA(UsdPhysics.RevoluteJoint):
            raise SystemExit(f"[AM-T650-VALVE] {JOINT_PRIM} is not the asset's revolute joint -- regenerate the asset")
        drive = UsdPhysics.DriveAPI.Get(joint, "angular")
        if not drive:
            drive = UsdPhysics.DriveAPI.Apply(joint, "angular")
        drive.CreateStiffnessAttr(0.0)
        # USD Physics angular drives are per DEGREE: damping [N.m / (deg/s)].
        # Authored in N.m.s/rad as the knob says -> divide by 180/pi. (The first
        # flight, v1 2026-10-15, wrote 0.02 straight in and got 1.15 N.m.s/rad:
        # the beam crept while the arm wound onto its stops, then the force
        # guard tripped at 9.5 N with the beam at 78 deg/s -- speed
        # proportional to force, a damper, not a runaway.)
        if FRICTION_MODEL == "drive" and BEARING_FRICTION > 0.0:
            # stick-slip Coulomb friction: a stiff velocity drive whose torque limit is the
            # friction (maxForce is a TORQUE for an angular drive, not per degree)
            drive.CreateDampingAttr(float(STICTION_DAMPING * math.pi / 180.0))
            drive.CreateMaxForceAttr(float(BEARING_FRICTION))
        else:
            drive.CreateDampingAttr(float(BEARING_DAMPING * math.pi / 180.0))
        drive.CreateTargetVelocityAttr(0.0)
        # every contact of the rotor's colliders reported (threshold 0): the
        # ground-truth wrench the vehicle puts on it (CONTACT_TRUTH_TOPIC)
        PhysxSchema.PhysxContactReportAPI.Apply(rotor).CreateThresholdAttr().Set(0.0)
        self._valve_p0 = np.array(p0)
        print(f"[AM-T650-VALVE] valve: {VALVE_USD} referenced on {VALVE_PRIM}, {n_col} colliders, rotor "
              f"{m_rotor * 1e3:.0f} g (asset {m_asset * 1e3:.0f}), bearing damping {BEARING_DAMPING} N.m.s/rad, "
              f"Coulomb friction {BEARING_FRICTION} N.m ("
              + (f"STICK-SLIP in the joint drive, stiction damping {STICTION_DAMPING:g} N.m.s/rad"
                 if FRICTION_MODEL == "drive" else "tanh, explicit") + "); "
              + (f"GRIP SHIM {FIN_GRIP_MM:.0f} mm on the {VALVE['fin_thickness_m'] * 1e3:.0f} mm fin (invisible)"
                 if shim > VALVE["fin_thickness_m"] else
                 f"NO grip shim: the bare {VALVE['fin_thickness_m'] * 1e3:.0f} mm fin (the sim claw cannot clamp it)"),
              flush=True)

    def _setup_gripper_drive(self):
        super()._setup_gripper_drive()
        stage = omni.usd.get_context().get_stage()
        root = stage.GetPrimAtPath(self.drone_path)
        prim = next((p for p in Usd.PrimRange(root)
                     if p.GetName() == am06.GRIPPER_JOINT and p.IsA(UsdPhysics.RevoluteJoint)), None)
        if prim is None:
            return
        UsdPhysics.DriveAPI.Get(prim, "angular").GetMaxForceAttr().Set(float(GRIP_TORQUE_MAX))
        print(f"[AM-T650-VALVE] gripper drive torque capped at {GRIP_TORQUE_MAX} N.m (06: 1000) -- "
              f"a closed gripper squeezes the handle with a bounded force", flush=True)

    def __init__(self):
        super().__init__()
        if not am06.HEADLESS:
            self.pg.set_viewport_camera(CAMERA_POS, CAMERA_TARGET)
        self._wp_ready = False
        if WAYPOINTS_VIZ:
            self._wp_setup_view()
        ax, ay = VALVE_XY
        yaw = math.radians(VALVE_YAW_DEG)
        fin = (ax + VALVE["fin_radius_m"] * math.cos(yaw) - VALVE["fin_bearing_y_m"] * math.sin(yaw),
               ay + VALVE["fin_radius_m"] * math.sin(yaw) + VALVE["fin_bearing_y_m"] * math.cos(yaw))
        reach = VALVE["beam_reach_m"]
        print(f"\033[1;35m[AM-T650-VALVE] VALVE-TURNING SCENE: vehicle at ({SPAWN_XY[0]:.2f}, "
              f"{SPAWN_XY[1]:.2f}) m, yaw {SPAWN_YAW_DEG:.0f} deg (+x forward). TABLE (HOOBRO "
              f"BB52UXG01G1) {TABLE_LENGTH_Y:.3f} m long along y x {TABLE_DEPTH_X:.3f} m deep x "
              f"{TABLE_HEIGHT:.3f} m tall, centred on ({TABLE_CENTER_XY[0]:.2f}, {TABLE_CENTER_XY[1]:.2f}). "
              f"VALVE axis at ({ax:.3f}, {ay:.3f}), plate bottom {VALVE_BASE_Z:.3f} m"
              + (f" (riser {VALVE_RISER:.3f})" if VALVE_RISER > 0 else "")
              + f", beam {VALVE['beam_back_m'] * 1e3:.0f}+{reach * 1e3:.0f} mm long pointing {VALVE_YAW_DEG:.0f} deg, "
              f"top {VALVE_BASE_Z + VALVE['beam_top_z_m']:.3f} m; FIN at radius {VALVE['fin_radius_m']:.3f} m "
              f"= ({fin[0]:.3f}, {fin[1]:.3f}), {VALVE['fin_thickness_m'] * 1e3:.0f} mm thick (radial) x "
              f"{VALVE['fin_width_m'] * 1e3:.0f} mm wide, top {VALVE_BASE_Z + VALVE['fin_top_z_m']:.3f} m, grasp "
              f"{VALVE_BASE_Z + EE_OFFSET[2]:.3f} m (clamped as {max(FIN_GRIP_MM, 1e3 * VALVE['fin_thickness_m']):.0f} mm). "
              f"Rotor {self._rotor_mass * 1e3:.0f} g, {VALVE['rotor_inertia_axis_kgm2'] * self._rotor_mass / VALVE['rotor_mass_kg'] * 1e3:.2f} "
              f"x1e-3 kg m^2 about the axis -> mocap {ROTOR_BODY} ("
              + (f"origin = the {len(MOCAP_MARKERS)} markers' CENTROID, Motive's default pivot, "
                 f"[{MOCAP_PIVOT[0]:.4f}, {MOCAP_PIVOT[1]:.4f}, {MOCAP_PIVOT[2]:.4f}] m from the axis on the plate "
                 f"bottom" if MOCAP_PIVOT_MODE == "markers" else "origin = the axis on the plate bottom")
              + f", the rotor's axes, yaw = the beam's). PLANNER: push_pull_valve_axis_offset [{AXIS_OFFSET[0]:.4f}, "
              f"{AXIS_OFFSET[1]:.4f}, {AXIS_OFFSET[2]:.4f}], push_pull_ee_offset (from the axis) [{EE_OFFSET[0]:.4f}, "
              f"{EE_OFFSET[1]:.4f}, {EE_OFFSET[2]:.4f}] m; obstacle (push_pull_obstacle) [{ax - reach:.3f}, {ax + reach:.3f}, "
              f"{ay - reach:.3f}, {ay + reach:.3f}, {VALVE_BASE_Z + VALVE['beam_top_z_m']:.4f}]. "
              f"Unmodelled by every controller.\033[0m", flush=True)
        if LOAD_TORQUE > 0.0:
            print(f"\033[1;35m[AM-T650-VALVE] HANGING-WEIGHT LOAD {LOAD_TORQUE:.3f} N.m resisting "
                  f"{'+' if LOAD_DIR > 0 else '-'} rotation ({LOAD_TORQUE / VALVE['fin_radius_m']:.2f} N at the fin; "
                  f"= {LOAD_TORQUE / (9.81 * 0.02):.2f} kg on a 20 mm drum), take-up {LOAD_TAKEUP_DEG:.1f} deg, "
                  f"{'RATCHET (one-way pawl)' if RATCHET else 'NO ratchet: it back-drives when released'}\033[0m",
                  flush=True)

    def _setup_arm_ros2_bridge(self):
        super()._setup_arm_ros2_bridge()
        from geometry_msgs.msg import PoseStamped, TwistStamped
        from rclpy.qos import qos_profile_sensor_data
        from std_msgs.msg import Float64MultiArray
        node = self._arm_node
        # obj_0 = the rotor, live, in the emulator's input format
        self._rotor_pose_pub = node.create_publisher(PoseStamped, f"/{ROTOR_BODY}/state/pose",
                                                     qos_profile_sensor_data)
        self._rotor_twist_pubs = [node.create_publisher(TwistStamped, f"/{ROTOR_BODY}/state/{t}",
                                                        qos_profile_sensor_data)
                                  for t in ("twist", "twist_inertial")]
        self._rotor_msg = PoseStamped()
        self._rotor_msg.header.frame_id = "map"
        self._rotor_twist = TwistStamped()
        self._rotor_twist.header.frame_id = "map"
        self._rotor_h = None
        self._angle0 = None
        self._angle_unwrapped = 0.0
        self._angle_prev = None
        self._angle_logged = None
        self._angle_pub = node.create_publisher(Float64MultiArray, ANGLE_TOPIC, 10)
        self._angle_msg = Float64MultiArray()
        self._claw_pub = node.create_publisher(PoseStamped, f"/{CLAW_TRUTH_BODY}/state/pose",
                                               qos_profile_sensor_data)
        self._claw_msg = PoseStamped()
        self._claw_msg.header.frame_id = "map"
        self._claw_pad_com = None
        self._fixed_step = 0
        self._ct_pub = node.create_publisher(Float64MultiArray, CONTACT_TRUTH_TOPIC, 10)
        self._ct_msg = Float64MultiArray()
        self._ct_acc = np.zeros(18)
        self._ct_time = 0.0
        self._ct_npts = 0
        self._ct_err = None
        try:
            from omni.physx import get_physx_simulation_interface
            self._ct_sub = get_physx_simulation_interface().subscribe_full_contact_report_events(
                self._on_contacts)
            print(f"[AM-T650-VALVE] contact truth: the rotor's PhysX contacts -> {CONTACT_TRUTH_TOPIC} "
                  f"(raw normal / friction sums, {250 // FIXED_PUB_DIV:.0f} Hz)", flush=True)
        except Exception as exc:              # ground truth must never stop the plant
            self._ct_sub = None
            print(f"[AM-T650-VALVE] contact truth OFF: {exc}", flush=True)
        if WAYPOINTS_VIZ:
            self._wp_setup_ros()

    def _pose(self, h):
        P = self._dc.get_rigid_body_pose(h)
        return (np.array([P.p.x, P.p.y, P.p.z]),
                am06.C.quat_to_rot(P.r.w, P.r.x, P.r.y, P.r.z))

    def _rotor_pose(self):
        if not self._rotor_h:
            self._rotor_h = self._dc.get_rigid_body(ROTOR_PRIM)
            if not self._rotor_h:
                return None
        return self._dc.get_rigid_body_pose(self._rotor_h)

    def _publish_claw_truth(self, stamp):
        dc = self._dc
        if self._claw_pad_com is None:
            pads = [dc.get_rigid_body(self.drone_path + p) for p in am06.EE_MARKER_PAD_LINKS]
            wrist = dc.get_rigid_body(self.drone_path + am06.EE_MARKER_WRIST_LINK)
            if not all(pads) or not wrist:
                return
            coms = []
            for path in am06.EE_MARKER_PAD_LINKS:
                c = UsdPhysics.MassAPI(self.stage.GetPrimAtPath(self.drone_path + path)).GetCenterOfMassAttr().Get()
                coms.append(np.array([c[0], c[1], c[2]], float))
            self._claw_pad_com = (pads, wrist, coms)
        pads, wrist, coms = self._claw_pad_com
        mid = np.zeros(3)
        for h, c in zip(pads, coms):
            p, R = self._pose(h)
            mid += 0.5 * (p + R @ c)
        _, R_w = self._pose(wrist)
        claw = mid + CLAW_BEYOND_PADS * (R_w @ np.array([0.0, 0.0, -1.0]))
        m = self._claw_msg
        m.header.stamp = stamp
        m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, claw)
        self._claw_pub.publish(m)

    def _on_contacts(self, headers, data, anchors):
        """One physics step's contact report: add the rotor's contact impulses
        to the publish-interval accumulator (normal and friction-anchor parts
        kept apart, as reported)."""
        try:
            acc = self._ct_acc
            for h in headers:
                a0 = str(PhysicsSchemaTools.intToSdfPath(h.actor0))
                a1 = str(PhysicsSchemaTools.intToSdfPath(h.actor1))
                # normalised to "the rotor is actor0": one sign convention for
                # every pair (the box push's analysis calibrated it on the table)
                if a0 == ROTOR_PRIM:
                    other, sgn = a1, 1.0
                elif a1 == ROTOR_PRIM:
                    other, sgn = a0, -1.0
                else:
                    continue
                if not other.startswith(self.drone_path):
                    continue
                o, n = h.contact_data_offset, h.num_contact_data
                for i in range(o, o + n):
                    c = data[i]
                    f = sgn * np.array([c.impulse[0], c.impulse[1], c.impulse[2]])
                    acc[0:3] += f
                    pp = np.array([c.position[0], c.position[1], c.position[2]])
                    acc[12:15] += np.cross(pp, f)
                    self._ct_npts += 1
                o, n = h.friction_anchors_offset, h.num_friction_anchors_data
                for i in range(o, o + n):
                    c = anchors[i]
                    f = sgn * np.array([c.impulse[0], c.impulse[1], c.impulse[2]])
                    acc[3:6] += f
                    pp = np.array([c.position[0], c.position[1], c.position[2]])
                    acc[15:18] += np.cross(pp, f)
        except Exception as exc:              # report once, never stop the plant
            if self._ct_err is None:
                self._ct_err = exc
                print(f"[AM-T650-VALVE] contact truth callback failed: {exc!r}", flush=True)

    def _publish_contact_truth(self):
        T = self._ct_time
        if T <= 0.0:
            return
        self._ct_msg.data = [float(v) for v in (self._ct_acc / T)] + [float(T), float(self._ct_npts)]
        self._ct_pub.publish(self._ct_msg)
        self._ct_acc[:] = 0.0
        self._ct_time = 0.0
        self._ct_npts = 0

    def _bearing_friction(self, dt):
        """The bearing's Coulomb friction: a torque about the world z opposing
        the rotor's angular rate, -f tanh(w / w0), momentum-clamped so it can
        never reverse the rotor in one step (I w / dt)."""
        if BEARING_FRICTION <= 0.0 or not self._rotor_h or FRICTION_MODEL == "drive":
            return 0.0                    # (drive: the joint drive carries the friction)
        w = self._dc.get_rigid_body_angular_velocity(self._rotor_h)
        wz = float(w.z)
        I = VALVE["rotor_inertia_axis_kgm2"] * self._rotor_mass / VALVE["rotor_mass_kg"]
        tau = -BEARING_FRICTION * math.tanh(wz / BEARING_FRICTION_W)
        cap = I * abs(wz) / max(dt, 1e-6)
        tau = max(-cap, min(cap, tau))
        if tau != 0.0:
            from omni.isaac.dynamic_control import _dynamic_control as dc_mod   # noqa: F401
            self._dc.apply_body_torque(self._rotor_h, [0.0, 0.0, tau], True)
        return tau

    def _valve_load(self, P, dt):
        """The hanging-weight load (PEGASUS_VALVE_LOAD_TORQUE): -dir * tau_L while
        the string is wound in the turn's direction, faded in over the take-up
        angle (0 on the floor / slack). With PEGASUS_VALVE_RATCHET a pawl stops
        any backward rotation and carries the weight. Logs each state change."""
        if LOAD_TORQUE <= 0.0 or not self._rotor_h:
            return 0.0
        R = am06.C.quat_to_rot(P.r.w, P.r.x, P.r.y, P.r.z)
        yaw = math.atan2(R[1, 0], R[0, 0])
        if getattr(self, "_ld_prev", None) is None:
            self._ld_prev, self._ld_th, self._ld_state, self._ld_peak_back = yaw, 0.0, "on the floor", 0.0
        else:
            self._ld_th += math.atan2(math.sin(yaw - self._ld_prev), math.cos(yaw - self._ld_prev))
            self._ld_prev = yaw
        wz = float(self._dc.get_rigid_body_angular_velocity(self._rotor_h).z)
        wound = LOAD_DIR * self._ld_th                                   # [rad] string wound on the drum
        k = min(1.0, max(0.0, wound / math.radians(LOAD_TAKEUP_DEG)))
        tau = -LOAD_DIR * LOAD_TORQUE * k
        fwd = LOAD_DIR * wz
        state = "lifted" if k >= 1.0 else ("taking up" if k > 0.0 else "on the floor")
        if RATCHET:
            if fwd < 0.0:                                                # the pawl: stop it, carry the weight
                I = VALVE["rotor_inertia_axis_kgm2"] * self._rotor_mass / VALVE["rotor_mass_kg"]
                tau = -I * wz / max(dt, 1e-6)
                state = "pawl holding" if k > 0.0 else state
            else:
                tau *= min(1.0, fwd / RATCHET_W)
        elif fwd < 0.0 and k > 0.0:
            state = "BACK-DRIVING"
            self._ld_peak_back = max(self._ld_peak_back, -fwd)
        if state != self._ld_state:
            print(f"[AM-T650-VALVE] t={self._t:7.2f}s LOAD {self._ld_state} -> {state} (wound "
                  f"{math.degrees(wound):+.1f} deg, rate {math.degrees(wz):+.1f} deg/s, torque {tau:+.3f} N.m"
                  + (f", peak back-drive so far {math.degrees(self._ld_peak_back):.0f} deg/s" if self._ld_peak_back else "")
                  + ")", flush=True)
            self._ld_state = state
        if tau != 0.0:
            self._dc.apply_body_torque(self._rotor_h, [0.0, 0.0, tau], True)
        return tau

    def _control_step_inner(self, dt):
        super()._control_step_inner(dt)
        self._ct_time += dt
        self._fixed_step += 1
        P = self._rotor_pose()
        if P is not None:
            self._bearing_friction(dt)
            self._valve_load(P, dt)
        if self._fixed_step % FIXED_PUB_DIV == 0:
            stamp = self._arm_node.get_clock().now().to_msg()
            if P is not None:
                m = self._rotor_msg
                m.header.stamp = stamp
                # obj_0's origin: the rotor's frame (the axis) moved to the mocap pivot
                # (2026-10-28: the markers' centroid, Motive's default) -- same axes
                pos = np.array([P.p.x, P.p.y, P.p.z])
                if MOCAP_PIVOT_MODE == "markers":
                    pos = pos + am06.C.quat_to_rot(P.r.w, P.r.x, P.r.y, P.r.z) @ MOCAP_PIVOT
                m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, pos)
                o = m.pose.orientation
                o.x, o.y, o.z, o.w = float(P.r.x), float(P.r.y), float(P.r.z), float(P.r.w)
                self._rotor_pose_pub.publish(m)
                self._rotor_twist.header.stamp = stamp
                for pub in self._rotor_twist_pubs:
                    pub.publish(self._rotor_twist)
                self._track_angle(P, dt * FIXED_PUB_DIV)
            try:
                self._publish_claw_truth(stamp)
            except Exception:                 # a ground-truth topic must never stop the plant
                pass
            try:
                self._publish_contact_truth()
            except Exception:
                pass
        if self._wp_ready:
            try:
                if self._t - self._wp_last >= WP_REFRESH_S:
                    self._wp_last = self._t
                    self._wp_refresh()
            except Exception as exc:          # a view must never stop the plant
                self._wp_ready = False
                print(f"[AM-T650-VALVE] waypoint view stopped: {exc}", flush=True)

    def _track_angle(self, P, dt):
        """The beam's angle, unwrapped from the rotor's yaw, published on
        ANGLE_TOPIC [angle rad, rate rad/s, angle deg from the start] and
        printed whenever it turned BEAM_LOG_TURN_DEG."""
        R = am06.C.quat_to_rot(P.r.w, P.r.x, P.r.y, P.r.z)
        yaw = math.atan2(R[1, 0], R[0, 0])
        if self._angle_prev is None:
            self._angle0 = yaw
            self._angle_prev = yaw
            self._angle_logged = 0.0
            self._angle_unwrapped = 0.0
            rate = 0.0
        else:
            d = math.atan2(math.sin(yaw - self._angle_prev), math.cos(yaw - self._angle_prev))
            self._angle_unwrapped += d
            self._angle_prev = yaw
            rate = d / max(dt, 1e-6)
        self._angle_msg.data = [float(self._angle0 + self._angle_unwrapped), float(rate),
                                float(math.degrees(self._angle_unwrapped))]
        self._angle_pub.publish(self._angle_msg)
        turned = math.degrees(self._angle_unwrapped)
        if abs(turned - self._angle_logged) >= BEAM_LOG_TURN_DEG:
            self._angle_logged = turned
            tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(R[2, 2])))))
            print(f"[AM-T650-VALVE] t={self._t:7.2f}s BEAM turned {turned:+.1f} deg from the start "
                  f"(yaw {math.degrees(yaw):.1f} deg, rate {math.degrees(rate):+.1f} deg/s, tilt {tilt:.2f} deg)",
                  flush=True)

    # ── waypoint view ──────────────────────────────────────────────────────
    def _wp_setup_ros(self):
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import Float64MultiArray
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._wp_info = None
        self._arm_node.create_subscription(Float64MultiArray, PL_INFO_TOPIC,
                                           lambda m: setattr(self, "_wp_info", list(m.data)), latched)

    def _wp_setup_view(self):
        UsdGeom.Xform.Define(self.stage, WP_ROOT)
        self._wp_prims = {}
        self._wp_labels = {}
        self._wp_sig = None
        self._wp_last = -1e9
        self._wp_sv = None
        if not am06.HEADLESS:
            try:
                import omni.ui as ui
                from omni.ui import color as cl
                from omni.ui import scene as sc
                from omni.kit.viewport.utility import get_active_viewport_window
                vw = get_active_viewport_window()
                with vw.get_frame("valve_turning_waypoints"):
                    self._wp_sv = sc.SceneView()
                vw.viewport_api.add_scene_view(self._wp_sv)
                self._wp_ui, self._wp_cl, self._wp_sc = ui, cl, sc
            except Exception as exc:
                self._wp_sv = None
                print(f"[AM-T650-VALVE] waypoint labels off ({exc}); markers and the printed "
                      f"table remain", flush=True)
        self._wp_ready = True

    def _wp_entries(self):
        info = getattr(self, "_wp_info", None)
        if info is None or len(info) < 61 or info[4] < 0.5:
            return {}
        out = {}
        for k, kind, name in WP_SHOW:
            g = 19 + 7 * k
            if not all(math.isfinite(v) for v in info[g:g + 7]):
                continue
            if kind == "body":
                out[str(k)] = ("body", np.array(info[g:g + 3]), info[g + 3], name)
            else:
                out[str(k)] = ("claw", np.array(info[g + 4:g + 7]), None, name)
        return out

    def _wp_make(self, key, kind):
        root = f"{WP_ROOT}/wp_{key}"
        xf = UsdGeom.Xform.Define(self.stage, root)
        xa = UsdGeom.Xformable(xf)
        parts = {"xf": xf, "t": xa.AddTranslateOp()}
        ball = UsdGeom.Sphere.Define(self.stage, root + "/ball")
        ball.CreateRadiusAttr(WP_CLAW_RADIUS if kind == "claw" else WP_BODY_RADIUS)
        _color(ball, WP_CLAW_RGB if kind == "claw" else WP_BODY_RGB)
        return parts

    def _wp_label(self, key, kind, p, text):
        if self._wp_sv is None:
            return
        sc, ui, cl = self._wp_sc, self._wp_ui, self._wp_cl
        M = sc.Matrix44.get_translation_matrix(float(p[0]), float(p[1]), float(p[2]) + 0.06)
        if key not in self._wp_labels:
            color = "#d6e8ff" if kind == "body" else "#ffc46b"
            with self._wp_sv.scene:
                tr = sc.Transform(transform=M)
                with tr:
                    with sc.Transform(look_at=sc.Transform.LookAt.CAMERA):
                        with sc.Transform(scale_to=sc.Space.SCREEN):
                            lb = sc.Label(text, alignment=ui.Alignment.CENTER_BOTTOM,
                                          color=cl(color), size=15)
            self._wp_labels[key] = (tr, lb)
        tr, lb = self._wp_labels[key]
        tr.transform = M
        lb.text = text
        tr.visible = True

    def _wp_refresh(self):
        entries = self._wp_entries()
        sig = tuple(sorted((k, tuple(np.round(e[1], 3))) for k, e in entries.items()))
        if sig == self._wp_sig:
            return
        self._wp_sig = sig
        rows = []
        for key, (kind, p, yaw, name) in sorted(entries.items()):
            if key not in self._wp_prims:
                self._wp_prims[key] = self._wp_make(key, kind)
            parts = self._wp_prims[key]
            parts["t"].Set(Gf.Vec3d(*map(float, p)))
            UsdGeom.Imageable(parts["xf"].GetPrim()).MakeVisible()
            text = f"{name}  ({p[0]:.2f}, {p[1]:.2f}, {p[2]:.2f}) m"
            if yaw is not None:
                text += f"  yaw {yaw:.0f}°"
            self._wp_label(key, kind, p, text)
            rows.append(text)
        for key, parts in self._wp_prims.items():
            if key not in entries:
                UsdGeom.Imageable(parts["xf"].GetPrim()).MakeInvisible()
                if key in self._wp_labels:
                    self._wp_labels[key][0].visible = False
        if rows:
            print("\033[1;36m[AM-T650-VALVE] WAYPOINTS (planned):\n    " + "\n    ".join(rows) + "\033[0m",
                  flush=True)


def main():
    AmT650ValveTurning().run()


if __name__ == "__main__":
    main()
