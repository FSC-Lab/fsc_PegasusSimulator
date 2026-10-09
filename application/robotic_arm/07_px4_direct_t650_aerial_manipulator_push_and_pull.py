#!/usr/bin/env python
"""
07_px4_direct_t650_aerial_manipulator_push_and_pull.py  (2026-10-03)

Author: Shiqi Gao (shiqi.gao907@gmail.com)

THE PUSH-AND-PULL SCENE on 06's plant -- the Isaac twin of the indoor run for
the whole-body planner's push-and-pull mode (fsc_trajectory_planner,
push_pull.hpp; whole_body_planner/push_pull/*; the arm ground station's
"Push & Pull" tab), flown by the whole-body 4-D L1 law with its CONTACT phase
flag switched on from the grasp to the release.

It does not copy 06: it loads 06 as a module (06 stays byte-identical, and every
plant knob, the servo model, the spawn / re-seat / ground seat and the ROS 2 arm
bridge are the free-flight ones) and subclasses its sim class to add the scene:

  field     4.5 m (x, forward) by 4.2 m (y), centred on the origin -- not
            drawn: the room's own floor
  vehicle   spawned near the centre FACING +x (yaw 0, the arm on the nose), a
            little off the origin, (0.10, -0.07) m, as a real start mark is
  table     THE LAB'S CONSOLE TABLE, HOOBRO BB52UXG01G1 (amazon.ca B0G62DWL11):
            39.4 x 5.9 x 31.5 in = 1.000 m long x 0.150 m deep x 0.800 m tall,
            IN FRONT of the vehicle, its length along y: centred on (1.20, 0)
            -- x 1.125..1.275, y -0.50..0.50, top at 0.800 m. A 18 mm top, four
            25 mm legs and a lower shelf, all static colliders.
  box       ONE dynamic rigid body, 400 g in total, on the table near its LEFT
            (+y) end: a 200 (y, along the push) x 100 (x) x 60 mm box, centre
            (1.20, 0.25, 0.83), and on its +y face (the LEFT side, toward the
            start) a vertical FIN HANDLE -- 20 mm thick along x (the jaws close
            along it), 90 mm out along +y, 20..110 mm above the table top (50 mm
            above the box top) -- for the gripper to grasp from above.
            PhysX spreads the 400 g over both colliders (uniform density).

THE TASK (the Push & Pull tab): Go To Start flies to the table's LEFT end and
yaws rightward to -90 deg (the nose along -y, the length of the table), the claw
0.20 m above the fin; Ready To Push opens the gripper and descends onto it; Push
closes the gripper, switches the law to CONTACT and slides the box 0.50 m along
the table (-y) with the arm held still; Exit To Push opens the gripper, climbs
and folds the arm home; Go To Land turns on clockwise to -360 deg and hovers over
the landing spot 0.5 m behind the start.

FRICTION, SIZED FOR A SAFE DEMONSTRATION (2026-10-03):
  * box on table: static 0.29 = dynamic 0.29 on BOTH surfaces (default average
    combine) -> 1.14 N to break the box loose AND to keep it sliding (400 g).
    2026-10-05: 0.29 is the user's MEASURED friction of the experiment table and
    400 g the trial box -- the same force as the first design (200 g at 0.6,
    1.18 N), which is why the 10-04 numbers below still apply.
    EQUAL ON PURPOSE (2026-10-04, flights pl_5 / pl_6, otherwise identical): at
    0.7 / 0.6 the force dropped 14 % at break-away while the law's observers
    still carried the stuck box's reaction and its moment -- the base surged
    7 cm, the world-held claw folded the arm onto q3's +50 deg stop and the
    push aborted; at 0.6 / 0.6 the same flight pushed the box 502 mm, 2.6 deg
    of yaw. For the REAL box: a base whose static and sliding friction match
    (PTFE glides, felt) -- not grippy rubber, which sticks and then lets go.
    The 2026-09-27 interaction campaign's along-the-arm capacity is ~6 N (8-10 N
    nears the tilt guard): ~4.4x margin; the planner's force guard aborts at 5 N.
    It is also above the hardware |F_hat| noise floor (p99.9 1.3 N at the 2
    rad/s reading), so the push READS as a contact.
  * TIPPING is mass-independent: the box tips about its leading bottom edge when
    mu_s * h > L / 2 (h the grasp height above the table, L the box length along
    the push). Here 0.29 x 0.095 = 0.028 m vs 0.100 m: a 3.6x margin before the
    grip's own moment helps. For the REAL box: keep mu_s * h < L / 3, i.e. pick
    the box length, then the payload / surface for the friction.
  * fin: the pick-and-place grip material (static 1.2 / dynamic 1.0, combine
    MAX) so the pad contact uses it whatever the pads carry; the gripper drive's
    torque capped at 0.3 N.m (06 drives the jaws with a 1000 N.m drive). The
    push reaches the box through pad friction on the fin's faces (~18 N of
    capacity at the cap), the fingers' own faces behind it if they slip.

GEOMETRY CHECK (the planner's push pose [0, -20, 30, 0] deg: the claw 0.130 m
ahead of and 0.316 m under the body origin; the landing gear: two skids along the
body's x, 0.28 m apart inside, bottoms 0.313 m under the origin -- measured from
AM_xfwd.usda): the grasp point 0.895 m (15 mm under the fin top) puts the body
origin at 1.211 m and the skids 9.8 cm ABOVE the table top -- and the skids
straddle both the 150 mm table and the 100 mm box laterally anyway. The planner
checks every step against the table (push_pull_table, 5 cm clearance).

  mocap     obj_0 = the box (the box's centre and yaw), live -- what the
            planner's Get captures (push_pull_box_topic). The claw goes to it +
            push_pull_ee_offset = [0, 0.155, 0.065] m in the box frame (100 mm
            half box + 55 mm along the fin; 30 mm half box + 35 mm up the fin).
            claw_0 = the real claw point, ground truth only (not mocap).
            Published as <body>/state/{pose,twist,twist_inertial}; the OptiTrack
            emulator turns obj_0 into /obj_0/mocap (the 4-D stack lists it).

06's EE marker cube is forced OFF here: it would publish obj_0 too.

Knobs (environment):
  PEGASUS_PUSH_BOX_MASS          total box mass [kg], default 0.400 (was 0.200 until 2026-10-05)
  PEGASUS_PUSH_FRICTION_STATIC   box / table static friction, default 0.29 (measured; was 0.6)
  PEGASUS_PUSH_FRICTION_DYNAMIC  box / table dynamic friction, default 0.29 (measured; was 0.6)
  PEGASUS_PUSH_BOX_XY            the box centre "x,y" [m], default "1.20,0.25"
  PEGASUS_PUSH_BOX_YAW_DEG       the box's yaw [deg], default 0
  PEGASUS_PUSH_HANDLE            cad (default, 2026-10-07: the user's CAD box + clamp bracket + fin,
                                 assets/Box_Push.usda), post (2026-10-06: a post on the box's top centre) or fin
  PEGASUS_PUSH_BOX_USD           the CAD box asset, default assets/Box_Push.usda (handle cad)
  PEGASUS_PUSH_FIN_GRIP_MM       handle cad: the fin's clamped width in the sim [mm], default 20 -- the CAD
                                 fin is 5 mm and the sim claw's pads stop ~14 mm apart; 0 = the bare CAD fin
  PEGASUS_PUSH_POST_TOP          the post's top above the table [m], default 0.298 (grasp 25 mm below; 0.255 for the 60 deg fold)
  PEGASUS_PUSH_HANDLE_THICKNESS  the handle's thickness across the jaws [m], default 0.020
  PEGASUS_PUSH_GRIP_TORQUE       gripper drive torque cap [N.m], default 0.3
  PEGASUS_PUSH_SPAWN_XY          where the vehicle starts, "x,y" [m], default "0.10,-0.07"
  PEGASUS_PUSH_SPAWN_YAW_DEG     its heading [deg], default 0 (+x)
  PEGASUS_PUSH_WAYPOINTS=0       hide the waypoint markers and labels
  PEGASUS_PUSH_AM_ASSET          airframe asset, default AM_T650.usda (the T650's 7 cm shorter gear)
  PEGASUS_PUSH_GROUND_BODY_Z     resting body height [m], default 0.235 with AM_T650 (0.305 with AM_xfwd)
  PEGASUS_PUSH_Q2_LIMIT_DEG      q2's hard stops "lo,hi" [deg], default "-20,45" (the real arm's range, user
                                 2026-10-06; the asset authors -90,50); "asset" keeps -90,50

WAYPOINT VIEW: blue ball = a drone-body goal (Go To Start's hover above the
handle, the Land hover), orange = an end-effector (claw) target (the grasp
point, the pushed grasp point), after Plan. Each with a label (coordinates,
yaw); the same table prints here whenever it changes. Visual only.

Run with:
  scripts/indoor_sim/start_t650_aerial_manipulator_whole_body_L1_4D_push_and_pull_sitl.sh <config>
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
SPAWN_XY = _xy("PEGASUS_PUSH_SPAWN_XY", "0.10,-0.07")
SPAWN_YAW_DEG = am06._envf("PEGASUS_PUSH_SPAWN_YAW_DEG", 0.0)

# THE TABLE: HOOBRO BB52UXG01G1, 39.4" W x 5.9" D x 31.5" H (the product page,
# 2026-10-03) = 1.000 x 0.150 x 0.800 m. Its LENGTH runs along y (left-right in
# front of the start), its depth along x.
TABLE_CENTER_XY = (1.20, 0.0)
TABLE_LENGTH_Y  = 1.000                 # [m] 39.4 in
TABLE_DEPTH_X   = 0.150                 # [m] 5.9 in
TABLE_HEIGHT    = 0.800                 # [m] 31.5 in, to the top surface
TABLE_TOP_THICK = 0.018                 # [m] particleboard top
TABLE_LEG       = 0.025                 # [m] square metal legs
TABLE_SHELF_Z   = 0.250                 # [m] the lower shelf's top
TABLE_SHELF_THICK = 0.015               # [m]

# THE BOX: 200 (y, along the push) x 100 (x, across the table) x 60 mm, 400 g with
# its handle. 200 mm along the push for the tipping margin (header); 100 mm wide
# on the 150 mm deep table leaves 25 mm a side.
# 2026-10-06: the user's REAL box, 240 (along the push) x 160 (across) x 95 mm
# (was 100 x 200 x 60). 160 mm across overhangs the 150 mm-deep console table by
# 5 mm a side -- it still rests on it.
BOX_SIZE  = (0.160, 0.240, 0.095)       # [m] x, y, z
BOX_MASS  = am06._envf("PEGASUS_PUSH_BOX_MASS", 0.400)       # [kg] box + handle (2026-10-05; was 0.200)
BOX_XY    = _xy("PEGASUS_PUSH_BOX_XY", "1.20,0.25")
BOX_YAW_DEG = am06._envf("PEGASUS_PUSH_BOX_YAW_DEG", 0.0)
BOX_DROP_GAP = 0.0005                   # [m] spawned this far above the table

# THE FIN HANDLE on the box's +y face: thin along the box's x (the jaws close
# along it -- the drone faces the box's -y, so its lateral axis is the box's x),
# 90 mm out along +y, from 20 mm above the table to 50 mm above the box top.
HANDLE_THICKNESS = am06._envf("PEGASUS_PUSH_HANDLE_THICKNESS", 0.020)   # [m] (pick-and-place: 20 mm
                                        # leaves 8.75 mm a side in the 37.5 mm open fingertip gap)
HANDLE_LENGTH    = 0.090                # [m] out from the box face
HANDLE_BOTTOM    = 0.020                # [m] above the table top (never drags)
HANDLE_TOP       = 0.110                # [m] above the table top
GRASP_BELOW_TOP  = 0.015                # [m] the claw point under the fin top (the pick's)
GRASP_FROM_FACE  = 0.055                # [m] the claw point out from the box face: the ~50 mm
                                        # wide pads span 30..80 mm, inside the 90 mm fin
# THE HANDLE (2026-10-06, user: "adjust the handle ... the interaction should be
# the force parallel to the table, not too much torque"): a vertical POST on the
# box's top CENTRE -- the push line through the box's friction centre (no yaw
# lever; the fin sat 0.155 m behind it) -- tall enough that the push pose can fold
# the arm forward ([0, 20, 40, 0] deg: the claw 0.13 m below the system CoM, not
# 0.28 m, so a 1.14 N push is 0.14 N.m of pitch, not 0.32) while the gear clears
# the table by ~6.5 cm. The jaws (closing across the table, the box's x) clamp its
# 20 mm faces; the claw, pitched 60 deg forward, slides onto it from behind
# (planner push_pull_approach_back) so the push presses the post into the palm.
# Tipping: mu * h / (L / 2) = 0.29 x 0.23 / 0.12 = 0.56 (L/3 rule: 0.067 < 0.080).
# PEGASUS_PUSH_HANDLE=fin keeps the 2026-10-03 side fin;
# PEGASUS_PUSH_HANDLE=post keeps the 2026-10-06 post.
#
# THE CAD BOX (2026-10-07, user: "This is the CAD model for our box with the
# handle for push and pull task", total 400 g): assets/Box_Push.usda, generated
# from Box_Push.stl by docs/docs_aerial_manipulator/archive/
# push_pull_cad_box_20261007/tools/stl_to_usda_push.py. A box 240 (y, along the
# push) x 160 (x) x 100 mm (the CAD's 100, not the 95 measured) with a printed
# clamp bracket on its top centre: two framed side plates down its long faces, a
# strip across the top, a screw pad, a 10 mm rib, and a FIN 60 mm along the push
# x 5 mm across, its top 258 mm above the table -- the jaws close across the 5 mm,
# the claw point 25 mm under the fin top (233 mm above the table). The asset's
# frame IS the box frame here (origin = box centre = obj_0, +y along the push);
# its colliders are the box and the bracket as exact blocks, its mass properties
# the CAD solid's.
# THE GRIP SHIM: the sim claw cannot clamp 5 mm -- its slider-crank stops the
# pads ~19 mm apart (~14 mm at the fingertips; measured, pick-and-place
# 2026-10-07), where the real gripper's foam pads close fully. So the fin's bare
# part (above the rib) gets an INVISIBLE collider PEGASUS_PUSH_FIN_GRIP_MM wide
# (default 20, the clamped width that flew 2026-10-04..06), standing in for the
# foam the sim claw lacks. 0 = the bare CAD fin (the claw closes on air).
HANDLE = (os.environ.get("PEGASUS_PUSH_HANDLE", "") or "cad").strip()
if HANDLE not in ("cad", "post", "fin"):
    raise SystemExit(f"PEGASUS_PUSH_HANDLE must be cad, post or fin, got {HANDLE!r}")
BOX_USD = (os.environ.get("PEGASUS_PUSH_BOX_USD", "") or os.path.join(am06.ASSETS_DIR, "Box_Push.usda")).strip()
FIN_GRIP_MM = am06._envf("PEGASUS_PUSH_FIN_GRIP_MM", 20.0)
CAD_GRASP_BELOW_TOP = 0.025             # [m] the claw point under the fin top (the post's rule)
CAD = {}
if HANDLE == "cad":
    if not os.path.isfile(BOX_USD):
        raise SystemExit(f"[AM-T650-PUSH] handle 'cad' needs {BOX_USD} (generate it with "
                         f"push_pull_cad_box_20261007/tools/stl_to_usda_push.py), or set PEGASUS_PUSH_HANDLE=post")
    _cad_stage = Usd.Stage.Open(BOX_USD)        # kept alive: a prim of a dropped stage expires
    _root = _cad_stage.GetDefaultPrim()
    CAD = {k: float(_root.GetAttribute(f"fsc:{k}").Get()) for k in (
        "box_size_x_m", "box_size_y_m", "box_size_z_m", "fin_thickness_m", "fin_length_m",
        "fin_centre_x_m", "fin_centre_y_m", "fin_top_z_m", "fin_exposed_bottom_z_m")}
    BOX_SIZE = (CAD["box_size_x_m"], CAD["box_size_y_m"], CAD["box_size_z_m"])
    del _root, _cad_stage
POST_DEPTH_Y   = 0.030                  # [m] along the push (the jaws' pads are ~50 mm wide)
POST_TOP       = am06._envf("PEGASUS_PUSH_POST_TOP", 0.298)   # [m] above the table top (grasp 0.273 m; 0.255 for the 60 deg fold)
POST_GRASP_BELOW_TOP = 0.025            # [m] the claw point under the post top -> 0.230 m above the table
# the planner's push_pull_ee_offset for this box (box frame, from its centre)
if HANDLE == "cad":
    EE_OFFSET = (CAD["fin_centre_x_m"], CAD["fin_centre_y_m"], CAD["fin_top_z_m"] - CAD_GRASP_BELOW_TOP)
elif HANDLE == "post":
    EE_OFFSET = (0.0, 0.0, POST_TOP - POST_GRASP_BELOW_TOP - 0.5 * BOX_SIZE[2])
else:
    EE_OFFSET = (0.0,
                 0.5 * BOX_SIZE[1] + GRASP_FROM_FACE,
                 HANDLE_TOP - GRASP_BELOW_TOP - 0.5 * BOX_SIZE[2])

FRICTION_STATIC  = am06._envf("PEGASUS_PUSH_FRICTION_STATIC", 0.29)    # measured table, 2026-10-05
FRICTION_DYNAMIC = am06._envf("PEGASUS_PUSH_FRICTION_DYNAMIC", 0.29)
GRIP_FRICTION_STATIC  = 1.2             # the fin: wins every pad contact (combine max)
GRIP_FRICTION_DYNAMIC = 1.0
GRIP_TORQUE_MAX = am06._envf("PEGASUS_PUSH_GRIP_TORQUE", 0.3)   # [N.m] gripper drive cap

SCENE_ROOT = "/World/push_pull"
BOX_PRIM   = SCENE_ROOT + "/box"
WP_ROOT    = SCENE_ROOT + "/waypoints"
BOX_BODY   = "obj_0"                    # the box, live: the planner's push_pull_box_topic body
CLAW_TRUTH_BODY = "claw_0"              # the real claw point, ground truth only
CLAW_BEYOND_PADS = 0.039                # [m] claw point beyond the pad CoM midpoint (07 pick-and-place's probe)
FIXED_PUB_DIV = 4                       # publish every 4th physics step (62.5 Hz)
# CONTACT TRUTH (2026-10-05): the PhysX full contact report on the box, summed per
# publish interval and divided by its duration -> mean force. Published RAW, as
# reported but normalised to "the box is actor0" (the impulse sign convention is
# unverified in Isaac 5.1 -- the analysis calibrates it on the table, whose normal
# force must hold the box's weight up):
#   [0:3]  vehicle-box normal force    [3:6]  vehicle-box friction-anchor force
#   [6:9]  table-box normal force      [9:12] table-box friction-anchor force
#   [12:15] sum p x f, vehicle-box normal   [15:18] sum p x f, vehicle-box friction
#   (moments about the WORLD origin; the analysis moves them to the claw point)
#   [18] interval [s]   [19] number of vehicle-box contact points in the interval
CONTACT_TRUTH_TOPIC = "/push_pull_truth/contact"
BOX_LOG_MOVE = 0.01                     # [m] print the box pose when it moved this far

WAYPOINTS_VIZ = (os.environ.get("PEGASUS_PUSH_WAYPOINTS", "") or "1").strip() != "0"
PL_INFO_TOPIC = "/uav_0/whole_body_planner/push_pull/info"
WP_REFRESH_S = 0.5                      # [s, sim]
WP_BODY_RGB = (0.20, 0.55, 1.00)
WP_CLAW_RGB = (1.00, 0.60, 0.10)
WP_BODY_RADIUS = 0.020
WP_CLAW_RADIUS = 0.015
# planned goals in push_pull/info: [18 + 7k ..] = base x, y, z, yaw_deg, claw x, y, z
WP_SHOW = ((0, "body", "1 Go To Start drone"), (1, "claw", "2 Grasp EE"),
           (2, "claw", "3 Pushed EE"), (4, "body", "5 Land drone"))

CAMERA_POS    = [-1.2, 2.6, 2.2]        # behind-left of the start, high
CAMERA_TARGET = [1.0, 0.1, 0.9]

# THE T650 LANDING GEAR (2026-10-06, user): the experiment drone is the T650, its
# gear ~7 cm shorter than the X650 gear AM_xfwd.usda carries (bottom plate to
# ground 22-23 cm vs 29-30 cm). AM_T650.usda is a copy of AM_xfwd.usda with the
# gear shortened by 0.070 m (utils_model/make_t650_gear_asset.py; the original is
# untouched): skids 0.2425 m under the body origin (was 0.3125), so the vehicle
# rests 0.070 m lower. The planner's push_pull_gear_depth carries the same
# 0.2425. PEGASUS_PUSH_AM_ASSET=AM_xfwd.usda flies the X650 gear again (then also
# set PEGASUS_PUSH_GROUND_BODY_Z=0.305 and the planner's gear depth 0.313).
AM_ASSET = (os.environ.get("PEGASUS_PUSH_AM_ASSET", "") or "AM_T650.usda").strip()
am06.USD_FILE = os.path.join(am06.ASSETS_DIR, AM_ASSET)
if not os.path.isfile(am06.USD_FILE):
    raise SystemExit(f"{am06.USD_FILE} not found -- generate it with "
                     f"robotic_arm/utils_model/make_t650_gear_asset.py (or set PEGASUS_PUSH_AM_ASSET)")
am06.GROUND_BODY_Z = am06._envf("PEGASUS_PUSH_GROUND_BODY_Z",
                                0.305 - 0.070 if AM_ASSET == "AM_T650.usda" else 0.305)
print(f"\033[1;35m[AM-T650-PUSH] airframe asset {AM_ASSET} (resting body height "
      f"{am06.GROUND_BODY_Z:.3f} m)\033[0m", flush=True)
# THE REAL ARM'S q2 RANGE (2026-10-06, user): about [-20, 45] deg on the
# hardware, against the asset's [-90, 50]. Authored on manip_joint2's PhysX
# limits at spawn, so in this scene q2 meets a hard stop where the real arm does.
# The planner's own joint box is unchanged (compiled shared constants, q2 <= 50).
_q2_raw = (os.environ.get("PEGASUS_PUSH_Q2_LIMIT_DEG", "") or "-20,45").strip()
if _q2_raw.lower() == "asset":
    Q2_LIMIT_DEG = None
else:
    try:
        Q2_LIMIT_DEG = tuple(float(v) for v in _q2_raw.split(","))
        assert len(Q2_LIMIT_DEG) == 2 and Q2_LIMIT_DEG[0] < Q2_LIMIT_DEG[1]
    except (ValueError, AssertionError):
        raise SystemExit(f"PEGASUS_PUSH_Q2_LIMIT_DEG must be 'lo,hi' or 'asset', got {_q2_raw!r}")
am06.SPAWN_POS = (float(SPAWN_XY[0]), float(SPAWN_XY[1]), float(am06.SPAWN_POS[2]))
am06.SPAWN_EULER = (0.0, 0.0, SPAWN_YAW_DEG)
if am06.EE_MARKER_CUBE:
    print("\033[1;33m[AM-T650-PUSH] PEGASUS_EE_MARKER_CUBE=1 ignored: obj_0 is the box in this "
          "scene, and the marker cube would publish the same body.\033[0m", flush=True)
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


class AmT650PushAndPull(am06.AmT650WholeBodyArmSim):

    # 06 calls this right after loading the environment and before
    # world.reset(): the scene exists when PhysX first reads the stage.
    def _spawn_am_px4_primary(self):
        self._build_scene(omni.usd.get_context().get_stage())
        drone_path = super()._spawn_am_px4_primary()
        self._set_q2_limit(drone_path)    # 06 assigns self.drone_path from this return value
        return drone_path

    def _set_q2_limit(self, drone_path):
        if Q2_LIMIT_DEG is None:
            print("[AM-T650-PUSH] q2 stops: the asset's", flush=True)
            return
        stage = omni.usd.get_context().get_stage()
        root = stage.GetPrimAtPath(drone_path)
        prim = next((p for p in Usd.PrimRange(root)
                     if p.GetName() == "manip_joint2" and p.IsA(UsdPhysics.RevoluteJoint)), None)
        if prim is None:
            raise SystemExit("[AM-T650-PUSH] no manip_joint2 to put the q2 stops on")
        j = UsdPhysics.RevoluteJoint(prim)
        old = (j.GetLowerLimitAttr().Get(), j.GetUpperLimitAttr().Get())
        j.GetLowerLimitAttr().Set(float(Q2_LIMIT_DEG[0]))
        j.GetUpperLimitAttr().Set(float(Q2_LIMIT_DEG[1]))
        print(f"\033[1;35m[AM-T650-PUSH] q2 hard stops {Q2_LIMIT_DEG[0]:.0f}..{Q2_LIMIT_DEG[1]:.0f} deg "
              f"(the real arm's range; asset {old[0]:.0f}..{old[1]:.0f})\033[0m", flush=True)

    def _build_scene(self, stage):
        UsdGeom.Xform.Define(stage, SCENE_ROOT)
        slide_mat = _physics_material(stage, SCENE_ROOT + "/slide_material",
                                      FRICTION_STATIC, FRICTION_DYNAMIC)
        grip_mat = _physics_material(stage, SCENE_ROOT + "/grip_material",
                                     GRIP_FRICTION_STATIC, GRIP_FRICTION_DYNAMIC, combine="max")

        # the table: static colliders (no rigid body). Its TOP carries the same
        # friction as the box's underside, so the box / table contact (default
        # average combine) is exactly FRICTION_STATIC / _DYNAMIC.
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
        for p in parts:
            UsdPhysics.CollisionAPI.Apply(p.GetPrim())
        _bind_physics(top.GetPrim(), slide_mat)

        # the box: ONE dynamic body, an UNSCALED Xform at the box centre (the
        # mocap backend reads orientation with ExtractRotation, exact only on an
        # unscaled matrix), with the box and the fin as collider children
        bx, by, bz = BOX_SIZE
        p0 = (BOX_XY[0], BOX_XY[1], TABLE_HEIGHT + 0.5 * bz + BOX_DROP_GAP)
        body = UsdGeom.Xform.Define(stage, BOX_PRIM)
        xf = UsdGeom.Xformable(body)
        xf.AddTranslateOp().Set(Gf.Vec3d(*map(float, p0)))
        xf.AddRotateZOp().Set(float(BOX_YAW_DEG))
        prim = body.GetPrim()
        if HANDLE == "cad":
            self._build_cad_box(stage, prim, slide_mat, grip_mat)
            self._box_p0 = np.array(p0)
            return
        box = _box(stage, BOX_PRIM + "/box", (0.0, 0.0, 0.0), BOX_SIZE, (0.85, 0.55, 0.20))
        if HANDLE == "post":
            post_h = POST_TOP - bz                                  # above the box top
            fin = _box(stage, BOX_PRIM + "/handle",
                       (0.0, 0.0, 0.5 * bz + 0.5 * post_h),          # box frame
                       (HANDLE_THICKNESS, POST_DEPTH_Y, post_h), (0.85, 0.85, 0.30))
        else:
            fin_h = HANDLE_TOP - HANDLE_BOTTOM
            fin_zc = -0.5 * bz + HANDLE_BOTTOM + 0.5 * fin_h        # box frame
            fin = _box(stage, BOX_PRIM + "/handle",
                       (0.0, 0.5 * by + 0.5 * HANDLE_LENGTH, fin_zc),
                       (HANDLE_THICKNESS, HANDLE_LENGTH, fin_h), (0.85, 0.85, 0.30))
        for part, mat in ((box, slide_mat), (fin, grip_mat)):
            UsdPhysics.CollisionAPI.Apply(part.GetPrim())
            _bind_physics(part.GetPrim(), mat)
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(float(BOX_MASS))
        # every contact of the box's colliders reported (threshold 0): the
        # ground-truth wrench the vehicle puts on it (CONTACT_TRUTH_TOPIC)
        PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr().Set(0.0)
        self._box_p0 = np.array(p0)

    def _build_cad_box(self, stage, prim, slide_mat, grip_mat):
        """The CAD box: Box_Push.usda REFERENCED onto the box body (the unscaled
        Xform at the box centre = obj_0), the rigid body here. The asset carries
        the visual mesh, the colliders (box + bracket blocks, the fin and rib
        flagged fsc:grip) and the CAD mass properties; the grip shim is added."""
        prim.GetReferences().AddReference(BOX_USD)
        n_col = 0
        for p in Usd.PrimRange(prim):
            if p.HasAPI(UsdPhysics.CollisionAPI):
                g = p.GetAttribute("fsc:grip")
                _bind_physics(p, grip_mat if (g and g.Get()) else slide_mat)
                n_col += 1
        shim = FIN_GRIP_MM * 1e-3
        if shim > CAD["fin_thickness_m"]:
            h = CAD["fin_top_z_m"] - CAD["fin_exposed_bottom_z_m"]
            g = _box(stage, BOX_PRIM + "/grip_shim",
                     (CAD["fin_centre_x_m"], CAD["fin_centre_y_m"], CAD["fin_exposed_bottom_z_m"] + 0.5 * h),
                     (shim, CAD["fin_length_m"], h), (0.30, 0.85, 0.40))
            UsdPhysics.CollisionAPI.Apply(g.GetPrim())
            _bind_physics(g.GetPrim(), grip_mat)
            UsdGeom.Imageable(g.GetPrim()).MakeInvisible()     # physics only: the visual is the CAD fin
            n_col += 1
        UsdPhysics.RigidBodyAPI.Apply(prim)
        mass = UsdPhysics.MassAPI(prim)               # the asset's: mass, CoM, inertia, principal axes
        m_asset = float(mass.GetMassAttr().Get())
        if abs(BOX_MASS - m_asset) > 1e-9:            # same shape, another total: inertia scales with it
            k = BOX_MASS / m_asset
            mass.GetMassAttr().Set(float(BOX_MASS))
            mass.GetDiagonalInertiaAttr().Set(Gf.Vec3f(*[float(k * v) for v in mass.GetDiagonalInertiaAttr().Get()]))
        PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr().Set(0.0)
        com = mass.GetCenterOfMassAttr().Get()
        print(f"[AM-T650-PUSH] box 'cad': {BOX_USD} referenced on {BOX_PRIM}, {n_col} colliders, "
              f"{BOX_MASS * 1e3:.0f} g, CoM {[round(1e3 * c, 1) for c in com]} mm from the box centre; "
              + (f"GRIP SHIM {FIN_GRIP_MM:.0f} mm on the {CAD['fin_thickness_m'] * 1e3:.0f} mm fin (invisible)"
                 if shim > CAD["fin_thickness_m"] else
                 f"NO grip shim: the bare {CAD['fin_thickness_m'] * 1e3:.0f} mm fin (the sim claw cannot clamp it)"),
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
        print(f"[AM-T650-PUSH] gripper drive torque capped at {GRIP_TORQUE_MAX} N.m (06: 1000) -- "
              f"a closed gripper squeezes the handle with a bounded force", flush=True)

    def __init__(self):
        super().__init__()
        if not am06.HEADLESS:
            self.pg.set_viewport_camera(CAMERA_POS, CAMERA_TARGET)
        self._wp_ready = False
        if WAYPOINTS_VIZ:
            self._wp_setup_view()
        g = 9.81
        grasp_h = (0.5 * BOX_SIZE[2] + EE_OFFSET[2]) if HANDLE == "cad" else \
            (POST_TOP - POST_GRASP_BELOW_TOP) if HANDLE == "post" else (HANDLE_TOP - GRASP_BELOW_TOP)
        tip_margin = (0.5 * BOX_SIZE[1]) / (FRICTION_STATIC * grasp_h)
        print(f"\033[1;35m[AM-T650-PUSH] PUSH-AND-PULL SCENE: vehicle at ({SPAWN_XY[0]:.2f}, "
              f"{SPAWN_XY[1]:.2f}) m, yaw {SPAWN_YAW_DEG:.0f} deg (+x forward). TABLE (HOOBRO "
              f"BB52UXG01G1) {TABLE_LENGTH_Y:.3f} m long along y x {TABLE_DEPTH_X:.3f} m deep x "
              f"{TABLE_HEIGHT:.3f} m tall, centred on ({TABLE_CENTER_XY[0]:.2f}, {TABLE_CENTER_XY[1]:.2f}): "
              f"x {TABLE_CENTER_XY[0] - 0.5 * TABLE_DEPTH_X:.3f}..{TABLE_CENTER_XY[0] + 0.5 * TABLE_DEPTH_X:.3f}, "
              f"y {TABLE_CENTER_XY[1] - 0.5 * TABLE_LENGTH_Y:.3f}..{TABLE_CENTER_XY[1] + 0.5 * TABLE_LENGTH_Y:.3f}. "
              f"BOX {BOX_MASS * 1e3:.0f} g = {BOX_SIZE[0] * 1e3:.0f} x {BOX_SIZE[1] * 1e3:.0f} x "
              f"{BOX_SIZE[2] * 1e3:.0f} mm (x, y, z) + "
              + (f"the CAD clamp bracket with a {CAD.get('fin_thickness_m', 0) * 1e3:.0f} x "
                 f"{CAD.get('fin_length_m', 0) * 1e3:.0f} mm FIN on its top centre (top "
                 f"{(0.5 * BOX_SIZE[2] + CAD.get('fin_top_z_m', 0)) * 1e3:.0f} mm, grasp {grasp_h * 1e3:.0f} mm "
                 f"above the table, clamped as {max(FIN_GRIP_MM, 1e3 * CAD.get('fin_thickness_m', 0)):.0f} mm)"
                 if HANDLE == "cad" else
                 f"a {HANDLE_THICKNESS * 1e3:.0f} x {POST_DEPTH_Y * 1e3:.0f} mm POST on its top centre "
                 f"(top {POST_TOP * 1e3:.0f} mm, grasp {grasp_h * 1e3:.0f} mm above the table)"
                 if HANDLE == "post" else
                 f"a {HANDLE_THICKNESS * 1e3:.0f} mm fin on its +y face ({HANDLE_LENGTH * 1e3:.0f} mm out, "
                 f"{HANDLE_BOTTOM * 1e3:.0f}..{HANDLE_TOP * 1e3:.0f} mm above the table)")
              + f", centre {np.round(self._box_p0, 4).tolist()} m, yaw {BOX_YAW_DEG:.1f} deg -> "
              f"mocap {BOX_BODY}. EE offset (planner push_pull_ee_offset) "
              f"[{EE_OFFSET[0]:.3f}, {EE_OFFSET[1]:.3f}, {EE_OFFSET[2]:.3f}] m. Box / table friction "
              f"{FRICTION_STATIC}/{FRICTION_DYNAMIC}: break-away {FRICTION_STATIC * BOX_MASS * g:.2f} N, "
              f"sliding {FRICTION_DYNAMIC * BOX_MASS * g:.2f} N; tipping margin "
              f"L/2 / (mu_s h) = {tip_margin:.2f} (> 1 = the box slides before it tips). "
              f"Unmodelled by every controller.\033[0m", flush=True)

    def _setup_arm_ros2_bridge(self):
        super()._setup_arm_ros2_bridge()
        from geometry_msgs.msg import PoseStamped, TwistStamped
        from rclpy.qos import qos_profile_sensor_data
        node = self._arm_node
        # obj_0 = the box, live, in the emulator's input format
        self._box_pose_pub = node.create_publisher(PoseStamped, f"/{BOX_BODY}/state/pose",
                                                   qos_profile_sensor_data)
        self._box_twist_pubs = [node.create_publisher(TwistStamped, f"/{BOX_BODY}/state/{t}",
                                                      qos_profile_sensor_data)
                                for t in ("twist", "twist_inertial")]
        self._box_msg = PoseStamped()
        self._box_msg.header.frame_id = "map"
        self._box_twist = TwistStamped()
        self._box_twist.header.frame_id = "map"
        self._box_h = None
        self._box_logged = None
        self._claw_pub = node.create_publisher(PoseStamped, f"/{CLAW_TRUTH_BODY}/state/pose",
                                               qos_profile_sensor_data)
        self._claw_msg = PoseStamped()
        self._claw_msg.header.frame_id = "map"
        self._claw_pad_com = None
        self._fixed_step = 0
        from std_msgs.msg import Float64MultiArray
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
            print(f"[AM-T650-PUSH] contact truth: the box's PhysX contacts -> {CONTACT_TRUTH_TOPIC} "
                  f"(raw normal / friction sums, {250 // FIXED_PUB_DIV:.0f} Hz)", flush=True)
        except Exception as exc:              # ground truth must never stop the plant
            self._ct_sub = None
            print(f"[AM-T650-PUSH] contact truth OFF: {exc}", flush=True)
        if WAYPOINTS_VIZ:
            self._wp_setup_ros()

    def _pose(self, h):
        P = self._dc.get_rigid_body_pose(h)
        return (np.array([P.p.x, P.p.y, P.p.z]),
                am06.C.quat_to_rot(P.r.w, P.r.x, P.r.y, P.r.z))

    def _box_pose(self):
        if not self._box_h:
            self._box_h = self._dc.get_rigid_body(BOX_PRIM)
            if not self._box_h:
                return None
        return self._dc.get_rigid_body_pose(self._box_h)

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
        """One physics step's contact report: add the box's contact impulses to
        the publish-interval accumulator (vehicle and table kept apart, normal
        and friction-anchor parts kept apart, as reported)."""
        try:
            acc = self._ct_acc
            for h in headers:
                a0 = str(PhysicsSchemaTools.intToSdfPath(h.actor0))
                a1 = str(PhysicsSchemaTools.intToSdfPath(h.actor1))
                # normalised to "the box is actor0": one sign convention for every
                # pair, which the analysis calibrates on the table's normal force
                if a0 == BOX_PRIM:
                    other, sgn = a1, 1.0
                elif a1 == BOX_PRIM:
                    other, sgn = a0, -1.0
                else:
                    continue
                if other.startswith(self.drone_path):
                    k = 0
                elif other.startswith(SCENE_ROOT + "/table"):
                    k = 6
                else:
                    continue
                o, n = h.contact_data_offset, h.num_contact_data
                for i in range(o, o + n):
                    c = data[i]
                    f = sgn * np.array([c.impulse[0], c.impulse[1], c.impulse[2]])
                    acc[k:k + 3] += f
                    if k == 0:
                        pp = np.array([c.position[0], c.position[1], c.position[2]])
                        acc[12:15] += np.cross(pp, f)
                        self._ct_npts += 1
                o, n = h.friction_anchors_offset, h.num_friction_anchors_data
                for i in range(o, o + n):
                    c = anchors[i]
                    f = sgn * np.array([c.impulse[0], c.impulse[1], c.impulse[2]])
                    acc[k + 3:k + 6] += f
                    if k == 0:
                        pp = np.array([c.position[0], c.position[1], c.position[2]])
                        acc[15:18] += np.cross(pp, f)
        except Exception as exc:              # report once, never stop the plant
            if self._ct_err is None:
                self._ct_err = exc
                print(f"[AM-T650-PUSH] contact truth callback failed: {exc!r}", flush=True)

    def _publish_contact_truth(self):
        T = self._ct_time
        if T <= 0.0:
            return
        self._ct_msg.data = [float(v) for v in (self._ct_acc / T)] + [float(T), float(self._ct_npts)]
        self._ct_pub.publish(self._ct_msg)
        self._ct_acc[:] = 0.0
        self._ct_time = 0.0
        self._ct_npts = 0

    def _control_step_inner(self, dt):
        super()._control_step_inner(dt)
        self._ct_time += dt
        self._fixed_step += 1
        if self._fixed_step % FIXED_PUB_DIV == 0:
            stamp = self._arm_node.get_clock().now().to_msg()
            P = self._box_pose()
            if P is not None:
                m = self._box_msg
                m.header.stamp = stamp
                m.pose.position.x, m.pose.position.y, m.pose.position.z = \
                    float(P.p.x), float(P.p.y), float(P.p.z)
                o = m.pose.orientation
                o.x, o.y, o.z, o.w = float(P.r.x), float(P.r.y), float(P.r.z), float(P.r.w)
                self._box_pose_pub.publish(m)
                self._box_twist.header.stamp = stamp
                for pub in self._box_twist_pubs:
                    pub.publish(self._box_twist)
                self._log_box(P)
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
                print(f"[AM-T650-PUSH] waypoint view stopped: {exc}", flush=True)

    def _log_box(self, P):
        """Print the box's pose whenever it has moved BOX_LOG_MOVE: the push, as
        Isaac saw it (displacement from the start, tilt, yaw)."""
        p = np.array([P.p.x, P.p.y, P.p.z])
        if self._box_logged is not None and np.linalg.norm(p - self._box_logged) < BOX_LOG_MOVE:
            return
        self._box_logged = p
        R = am06.C.quat_to_rot(P.r.w, P.r.x, P.r.y, P.r.z)
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(R[2, 2])))))
        yaw = math.degrees(math.atan2(R[1, 0], R[0, 0]))
        d = p - self._box_p0
        print(f"[AM-T650-PUSH] t={self._t:7.2f}s BOX at [{p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f}] m: moved "
              f"[{d[0] * 1e3:+.0f}, {d[1] * 1e3:+.0f}, {d[2] * 1e3:+.0f}] mm, tilt {tilt:.1f} deg, yaw "
              f"{yaw:.1f} deg", flush=True)

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
                with vw.get_frame("push_pull_waypoints"):
                    self._wp_sv = sc.SceneView()
                vw.viewport_api.add_scene_view(self._wp_sv)
                self._wp_ui, self._wp_cl, self._wp_sc = ui, cl, sc
            except Exception as exc:
                self._wp_sv = None
                print(f"[AM-T650-PUSH] waypoint labels off ({exc}); markers and the printed "
                      f"table remain", flush=True)
        self._wp_ready = True

    def _wp_entries(self):
        info = getattr(self, "_wp_info", None)
        if info is None or len(info) < 53 or info[4] < 0.5:
            return {}
        out = {}
        for k, kind, name in WP_SHOW:
            g = 18 + 7 * k
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
            print("\033[1;36m[AM-T650-PUSH] WAYPOINTS (planned):\n    " + "\n    ".join(rows) + "\033[0m",
                  flush=True)


def main():
    AmT650PushAndPull().run()


if __name__ == "__main__":
    main()
