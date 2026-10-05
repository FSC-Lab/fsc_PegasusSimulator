#!/usr/bin/env python
"""
07_px4_t650_aerial_manipulator_pick_and_place.py  (2026-10-01)

Author: Shiqi Gao (shiqi.gao907@gmail.com)

THE PICK-AND-PLACE SCENE on 06's plant -- the Isaac twin of the indoor run for
the whole-body planner's pick-and-place mode (fsc_trajectory_planner,
whole_body_planner/pick_place/*; the arm ground station's Pick & Place tabs).

It does not copy 06: it loads 06 as a module (06 stays byte-identical, and
every plant knob, the servo model, the spawn / re-seat / ground seat and the
ROS 2 arm bridge are the free-flight ones) and subclasses its sim class to add
the scene and the two mocap bodies the planner captures:

  field     4.5 m (x, length, forward) by 4.2 m (y, width), centred on the origin --
            not drawn: the room's own floor
  vehicle   spawned NEAR the centre FACING +x (yaw 0, the arm on the nose), the
            field's forward direction -- by default a little OFF the origin,
            (0.10, -0.07) m, as a real start mark sits off the mocap origin, so
            Adjust has a real offset to measure
  pillars   two cylinders, 1.0 m tall, 0.10 m diameter, static colliders:
            PICK at (1.0, 1.0) m (front-left), PLACE at (-1.0, -1.0) m
            (back-right)
  payload   ONE dynamic rigid body, 200 g in total, resting on the PICK pillar:
            a 110 x 110 x 65 mm box plus a vertical HANDLE plate standing in
            the middle of its top, 200 mm tall, for the gripper to grasp from
            above. PhysX spreads the 200 g over both colliders (uniform density).

  mocap     obj_0  = the payload's BOTTOM BOX: its centre (CG) and yaw, live
            (2026-10-03, user: on hardware the markers sit around the box, so
            the rigid body is the box). Get captures its position AND yaw; the
            planner adds pick_place_ee_offset in that object frame. The
            box centre rests 32.5 mm above the pillar top, the handle top 200 mm
            above the box top.
            drop_0 = the PLACE pillar's top centre, fixed (the planner types the
            place point by default, so it only matters with a place topic set)
            payload_0 = the payload body (BOX centre), live: Isaac ground
            truth for the carry (the same pose obj_0 carries)
            Published as <body>/state/{pose,twist,twist_inertial}; the
            OptiTrack emulator turns them into /obj_0/mocap and /drop_0/mocap
            when its body list carries them (the 4-D stack scripts do).

GRASPING is real PhysX contact, no attachment:
  * the payload carries a high-friction material (static 1.2, dynamic 1.0)
    whose friction COMBINE MODE is "max", so a pad-to-handle contact uses it
    whatever the pads' own material is;
  * the gripper drive's torque is capped (GRIP_TORQUE_MAX): 06 drives the
    jaws with a 1e6 stiffness / 1000 N.m drive, fine in free air but a
    kilonewton clamp on anything between them. Capped, a closed gripper
    squeezes the handle with a bounded force and stalls on it.
  * the jaws cannot close fully (pad separation 54.7 mm at the -50 deg
    limit), so the handle's THICKNESS must sit between the closed and the
    open face gap. A one-time probe at t = 1 s raycasts between the open
    pads and prints that gap and the jaws' closing axis.

06's EE marker cube is forced OFF here: it would publish obj_0 too.

Knobs (environment):
  PEGASUS_PNP_PAYLOAD_MASS   total payload mass [kg], default 0.200
  PEGASUS_PNP_PAYLOAD_YAW_DEG the payload's yaw [deg] = obj_0's; the handle's
                             THIN axis (the jaws close along it) is the payload's
                             y, so a drone at this yaw picks it; default 0
  PEGASUS_PNP_HANDLE_THICKNESS the handle plate's thickness [m] (along the
                             jaws' closing axis), default 0.020
  PEGASUS_PNP_GRIP_TORQUE    gripper drive torque cap [N.m], default 0.3
  PEGASUS_PNP_GRASP_TEST=1   run the seated grasp test (see GRASP_TEST below)
  PEGASUS_PNP_WAYPOINTS=0    hide the waypoint markers and labels
  PEGASUS_PNP_SPAWN_XY       where the vehicle starts, "x,y" [m], default "0.10,-0.07"
  PEGASUS_PNP_SPAWN_YAW_DEG  its heading [deg], default 0 (+x)

WAYPOINT VIEW: blue ball = a drone-body goal ("Start drone", "Place Start
drone", "Land Start drone", "Land drone"; its yaw is in the label); orange
sphere = an end-effector (claw) target ("Pick EE", "Place EE"); grey sphere =
where the drone body will be at the pick / place ("Pick drone", "Place drone",
after Plan only). Each
carries a label with its coordinates and yaw; the same table, with each value's
source ([typed], [captured], [live, not captured], [planned]), prints here
whenever it changes.

Planner offset for this payload (the pick-and-place yaml,
..._sim_pick_place.yaml): ONE EE offset for the pick AND the place (2026-10-03,
user decision), pick_place_ee_offset z = +0.22 m from the BOX CENTRE (32.5 mm
half box + 200 mm handle - 12.5 mm: the claw 12.5 mm below the handle top,
7.5 mm short of where the open jaws pinch it; was pick 0.250 / place 0.260
from the pillar tops), in the box's frame. The typed place point is therefore
in the same reference -- where the box CG is released: the place pillar top +
32.5 mm + PLACE_DROP = 1.04 m, so the box bottom stops 7.5 mm above the pillar
(never set down while clamped, see PLACE_DROP).
The planner reaches both from 0.10 m straight above (pick_place_approach_dz),
with the claw held in the world for those descents (pick_place_world_anchor).

Run with:
  scripts/indoor_sim/start_t650_aerial_manipulator_whole_body_L1_4D_pick_and_place_sitl.sh <config>
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
from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics, UsdShade  # noqa: E402

# ── the scene ──────────────────────────────────────────────────────────────
FIELD_LENGTH_X = 4.5                    # [m] along the forward axis; informative only, not drawn
FIELD_WIDTH_Y  = 4.2                    # [m]
# the start mark, a little OFF the mocap origin like a real one (user request):
# Adjust then measures a real offset
_SPAWN_XY_RAW = (os.environ.get("PEGASUS_PNP_SPAWN_XY", "") or "0.10,-0.07").strip()
try:
    SPAWN_XY = tuple(float(v) for v in _SPAWN_XY_RAW.split(","))
    assert len(SPAWN_XY) == 2
except (ValueError, AssertionError):
    raise SystemExit(f"PEGASUS_PNP_SPAWN_XY must be 'x,y' in metres, got {_SPAWN_XY_RAW!r}")
SPAWN_YAW_DEG = am06._envf("PEGASUS_PNP_SPAWN_YAW_DEG", 0.0)   # 0 = +x, the field's forward

PILLAR_HEIGHT   = 1.0                   # [m] to the top of the cap
PILLAR_DIAMETER = 0.10                  # [m]
# THE CAP (2026-10-03, user request: a flat cylinder on each pillar so the
# payload is not dropped off it): 160 mm across -- the 110 mm box's whole
# footprint at ANY yaw (its diagonal is 155.6 mm), so a box set down up to
# ~80 mm off the pillar axis still rests on it with its centre supported
# (the 2026-10-02 places landed 18-30 mm off axis, on a 100 mm pillar). 10 mm
# thick, its top at PILLAR_HEIGHT: every point and offset is unchanged.
CAP_DIAMETER    = am06._envf("PEGASUS_PNP_CAP_DIAMETER", 0.16)   # [m]
CAP_THICKNESS   = 0.010                 # [m]
PICK_PILLAR_XY  = (1.0, 1.0)            # [m] front-left, on the 1 m grid
PLACE_PILLAR_XY = (-1.0, -1.0)          # [m] back-right

PAYLOAD_SIZE = (0.110, 0.110, 0.065)    # [m] the box, x, y, z
PAYLOAD_MASS = am06._envf("PEGASUS_PNP_PAYLOAD_MASS", 0.200)   # [kg] box + handle
PAYLOAD_DROP_GAP = 0.0005               # [m] spawned this far above the pillar top

# The handle: a vertical plate in the middle of the box top. THICKNESS is along
# the payload's y axis (the jaws close along it), WIDTH along its x axis -- so
# the payload's yaw IS the yaw of a drone whose jaws meet the handle square
# (2026-10-03, user design: the planner aligns the drone's yaw with obj_0's).
#
# THICKNESS is set by the gripper, measured by this script's probe (2026-10-01):
# the pads' inner faces are 43.3 mm apart at the ground station's OPEN (0 deg)
# and close 19.5 mm in total by the -50 deg limit (pad travel, from the asset's
# joint frames), i.e. ~23.8 mm. Anything thinner than that is never clamped.
# (2026-10-01 estimate, from the pad midpoint: 30 mm would stall the jaws at
# about -34 deg with 6.6 mm a side -- SUPERSEDED below by the fingertip gap.)
#
# YAW: the jaws close along the vehicle's LATERAL axis (body +y, probe), and on
# Execute To Pick the planner turns the drone to the captured obj_0 yaw
# (pick_place_align_yaw). With the thin axis on the payload's y the two agree,
# so the payload sits at yaw 0 by default (PEGASUS_PNP_PAYLOAD_YAW_DEG): the
# plate's width along +x, "vertical" in the top view, and a drone at yaw 0
# picks it. (Until 2026-10-03 the drone faced the bearing start -> object and
# the thin axis sat at that bearing + 90 deg, 139.9 deg.)
HANDLE_HEIGHT    = 0.200                # [m]
HANDLE_WIDTH     = 0.060                # [m]
# 2026-10-02 MEASURED, flying the six legs (pick_place_tune_20261001): the OPEN
# jaws are only 37-38 mm apart near the FINGERTIPS (JAW GAP probe; the 43.3 mm
# above is the pad midpoint), so the 30 mm first used here left 3.5-4 mm a
# side -- less than the claw's lateral error on the way down (3-6 mm with the
# claw world-held, 6-20 mm CoM-held): four of four 30 mm grasps put a
# fingertip on the handle top. 20 mm leaves 8.75 mm a side and the jaws still
# clamp it well short of their limit (-37 deg, measured). The PHYSICAL
# payload's handle is a design decision this number only informs.
HANDLE_THICKNESS = am06._envf("PEGASUS_PNP_HANDLE_THICKNESS", 0.020)   # [m]
if (os.environ.get("PEGASUS_PNP_HANDLE_YAW_DEG") or "").strip():
    raise SystemExit("PEGASUS_PNP_HANDLE_YAW_DEG was replaced by PEGASUS_PNP_PAYLOAD_YAW_DEG "
                     "(2026-10-03): the payload's yaw, its handle's thin axis on the payload's y")
PAYLOAD_YAW_DEG  = am06._envf("PEGASUS_PNP_PAYLOAD_YAW_DEG", 0.0)   # [deg]
# Where the claw point (the model's grasp point) sits below the handle top at
# the grasp. MEASURED 2026-10-02: the handle enters the open jaws only ~20 mm
# before they pinch it (two pick flights stalled 20-22 mm in; the gripper then
# clamped at -3.5 deg), although the centre-line probes (PAD EXTENT / JAW GAP
# / HANDLE CEILING) show 37-44 mm of gap and 44 mm of headroom -- the pinch is
# off the centre line. 15 mm keeps 5 mm off it.
GRASP_BELOW_TOP  = 0.015                # [m]
# The pick yaml's pick_place_ee_offset z, from the BOX CENTRE (obj_0) -- for
# this banner only. 0.22, not 0.2175 = GRASP_BELOW_TOP from the top: the arm GS
# edits every field to 1 cm (user, 2026-10-03), so the default is one it can
# show exactly; the claw then sits 12.5 mm below the handle top.
PICK_EE_OFFSET_Z = 0.22                 # [m]
PLACE_DROP       = 0.0075               # [m] box bottom above the place pillar on release (a
                                        # box SET DOWN while still clamped closes the chain
                                        # vehicle-arm-payload-pillar and the base drags it off)

# GRASP TEST (PEGASUS_PNP_GRASP_TEST=1): with the vehicle seated and the arm
# at home, the payload is held with its handle between the open pads, the
# gripper closes, and once it stalls on the handle the payload is let go and
# watched for slip; then the gripper opens and it must drop.
GRASP_TEST      = (os.environ.get("PEGASUS_PNP_GRASP_TEST", "") or "0").strip() == "1"
GRASP_TEST_T0   = 2.0                   # [s, sim] start
GRASP_TEST_GRIP_H = 0.030               # [m] pads this far above the box top (box clears the floor)
GRASP_TEST_EDGE = 0.012                 # [m] handle's rear edge this far behind the pad centres
GRASP_TEST_OBSERVE_S = 6.0              # [s] watch for slip
GRASP_TEST_SLIP_MAX = 0.003             # [m] PASS threshold

# WAYPOINT VIEW (2026-10-01, user request): the six pick-and-place leg goals as
# markers + labels with their coordinates, following the planner live --
# before Plan the nominal base poses (+ the Adjust offset) and the claw
# targets (captured point, else the live mocap point, + the claw offset),
# after Plan the planner's own goals. Visual only: no collider anywhere.
WAYPOINTS_VIZ   = (os.environ.get("PEGASUS_PNP_WAYPOINTS", "") or "1").strip() != "0"
PLANNER_NODE    = "/uav_0/whole_body_trajectory_planner"
PP_INFO_TOPIC   = "/uav_0/whole_body_planner/pick_place/info"
WP_POLL_S       = 2.0                   # [s] planner parameter poll
WP_REFRESH_S    = 0.5                   # [s, sim] marker refresh
WP_BASE_PARAMS  = {0: "pick_place_start", 2: "pick_place_place_start",
                   4: "pick_place_land_start", 5: "pick_place_land"}
# ONE EE offset for both claw legs (planner 2026-10-03)
WP_CLAW_PARAMS  = {1: "pick_place_ee_offset", 3: "pick_place_ee_offset"}
# the TYPED place pose [x, y, z, yaw_deg] (2026-10-02 planner: only Pick is
# measured; Place is typed -- read off mocap before the flight -- and, unlike
# the base poses, NOT shifted by the Adjust offset (2026-10-03); its yaw turns
# the EE offset there (2026-10-03))
WP_PLACE_POINT_PARAM = "pick_place_place_point"
WP_NAMES        = ["Start", "Pick", "Place Start", "Place", "Land Start", "Land"]
WP_BODY_RGB     = (0.20, 0.55, 1.00)
WP_CLAW_RGB     = (1.00, 0.60, 0.10)
WP_PLANBODY_RGB = (0.85, 0.85, 0.85)
WP_BODY_RADIUS  = 0.020                 # [m] ball only (the yaw is in the label)
WP_CLAW_RADIUS  = 0.015                 # [m]

# friction: the payload's material wins every contact it is in ("max")
GRIP_FRICTION_STATIC  = 1.2
GRIP_FRICTION_DYNAMIC = 1.0
PILLAR_FRICTION_STATIC  = 0.8
PILLAR_FRICTION_DYNAMIC = 0.7
GRIP_TORQUE_MAX = am06._envf("PEGASUS_PNP_GRIP_TORQUE", 0.3)   # [N.m] gripper drive cap

SCENE_ROOT   = "/World/pick_place"
PAYLOAD_PRIM = SCENE_ROOT + "/payload"
WP_ROOT      = SCENE_ROOT + "/waypoints"
PICK_BODY    = "obj_0"                  # the payload box (CG + yaw), live: the planner's pick_place_pick_topic body
DROP_BODY    = "drop_0"                 # the PLACE pillar top: its pick_place_place_topic body
PAYLOAD_TRUTH_BODY = "payload_0"        # the payload, live: ground truth only (not mocap)
CLAW_TRUTH_BODY = "claw_0"              # the real claw point, ground truth only (not mocap)
CLAW_BEYOND_PADS = 0.039                # [m] claw point beyond the pad CoM midpoint, along the claw axis (probe)
FIXED_PUB_DIV = 4                       # publish obj_0 / drop_0 every 4th physics step (62.5 Hz)
PROBE_AT_S   = 1.0                      # [s, sim] the one-time open-gripper gap probe

# where the viewport looks from (windowed runs): behind-left of the start, high
CAMERA_POS    = [-4.2, 3.6, 3.0]
CAMERA_TARGET = [0.3, 0.0, 0.6]

# Applied to 06 BEFORE its sim class is built: the spawn reads these at call time.
am06.SPAWN_POS = (float(SPAWN_XY[0]), float(SPAWN_XY[1]), float(am06.SPAWN_POS[2]))
am06.SPAWN_EULER = (0.0, 0.0, SPAWN_YAW_DEG)
if am06.EE_MARKER_CUBE:
    print("\033[1;33m[AM-T650-PNP] PEGASUS_EE_MARKER_CUBE=1 ignored: obj_0 is the payload box "
          "in this scene, and the marker cube would publish the same body.\033[0m", flush=True)
am06.EE_MARKER_CUBE = False


def _color(prim_geom, rgb):
    prim_geom.CreateDisplayColorAttr([Gf.Vec3f(*map(float, rgb))])


def _box(stage, path, center, size, rgb):
    """A unit cube scaled to `size` at `center`."""
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


class AmT650PickAndPlace(am06.AmT650WholeBodyArmSim):

    # 06 calls this right after loading the environment and before
    # world.reset(): the scene exists when PhysX first reads the stage.
    def _spawn_am_px4_primary(self):
        self._build_scene(omni.usd.get_context().get_stage())
        return super()._spawn_am_px4_primary()

    def _build_scene(self, stage):
        UsdGeom.Xform.Define(stage, SCENE_ROOT)
        pillar_mat = _physics_material(stage, SCENE_ROOT + "/pillar_material",
                                       PILLAR_FRICTION_STATIC, PILLAR_FRICTION_DYNAMIC)
        grip_mat = _physics_material(stage, SCENE_ROOT + "/payload_material",
                                     GRIP_FRICTION_STATIC, GRIP_FRICTION_DYNAMIC, combine="max")

        # pillars: static colliders (no rigid body)
        for name, (x, y), rgb in (("pick_pillar", PICK_PILLAR_XY, (0.30, 0.50, 0.80)),
                                  ("place_pillar", PLACE_PILLAR_XY, (0.30, 0.70, 0.40))):
            cyl = UsdGeom.Cylinder.Define(stage, f"{SCENE_ROOT}/{name}")
            h = PILLAR_HEIGHT - CAP_THICKNESS
            cyl.CreateRadiusAttr(0.5 * PILLAR_DIAMETER)
            cyl.CreateHeightAttr(h)
            cyl.CreateAxisAttr("Z")
            UsdGeom.Xformable(cyl).AddTranslateOp().Set(Gf.Vec3d(x, y, 0.5 * h))
            _color(cyl, rgb)
            UsdPhysics.CollisionAPI.Apply(cyl.GetPrim())
            _bind_physics(cyl.GetPrim(), pillar_mat)
            # the cap: a flat disc, its top at PILLAR_HEIGHT
            cap = UsdGeom.Cylinder.Define(stage, f"{SCENE_ROOT}/{name}_cap")
            cap.CreateRadiusAttr(0.5 * CAP_DIAMETER)
            cap.CreateHeightAttr(CAP_THICKNESS)
            cap.CreateAxisAttr("Z")
            UsdGeom.Xformable(cap).AddTranslateOp().Set(
                Gf.Vec3d(x, y, PILLAR_HEIGHT - 0.5 * CAP_THICKNESS))
            _color(cap, tuple(0.6 * c for c in rgb))
            UsdPhysics.CollisionAPI.Apply(cap.GetPrim())
            _bind_physics(cap.GetPrim(), pillar_mat)

        # payload: ONE dynamic body, an UNSCALED Xform at the box centre (the
        # mocap backend reads orientation with ExtractRotation, exact only on
        # an unscaled matrix), with the box and the handle as collider children.
        sx, sy, sz = PAYLOAD_SIZE
        p0 = (PICK_PILLAR_XY[0], PICK_PILLAR_XY[1], PILLAR_HEIGHT + 0.5 * sz + PAYLOAD_DROP_GAP)
        body = UsdGeom.Xform.Define(stage, PAYLOAD_PRIM)
        xf = UsdGeom.Xformable(body)
        xf.AddTranslateOp().Set(Gf.Vec3d(*map(float, p0)))
        xf.AddRotateZOp().Set(float(PAYLOAD_YAW_DEG))
        prim = body.GetPrim()
        box = _box(stage, PAYLOAD_PRIM + "/box", (0.0, 0.0, 0.0), PAYLOAD_SIZE, (0.95, 0.50, 0.12))
        handle = _box(stage, PAYLOAD_PRIM + "/handle",
                      (0.0, 0.0, 0.5 * sz + 0.5 * HANDLE_HEIGHT),
                      (HANDLE_WIDTH, HANDLE_THICKNESS, HANDLE_HEIGHT), (0.85, 0.85, 0.30))
        for part in (box, handle):
            UsdPhysics.CollisionAPI.Apply(part.GetPrim())
            _bind_physics(part.GetPrim(), grip_mat)
        UsdPhysics.RigidBodyAPI.Apply(prim)
        # mass only: PhysX derives the centre of mass and the inertia from the
        # two colliders at the uniform density that gives this total
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(float(PAYLOAD_MASS))
        self._payload_p0 = np.array(p0)

    def _setup_gripper_drive(self):
        super()._setup_gripper_drive()
        stage = omni.usd.get_context().get_stage()
        root = stage.GetPrimAtPath(self.drone_path)
        prim = next((p for p in Usd.PrimRange(root)
                     if p.GetName() == am06.GRIPPER_JOINT and p.IsA(UsdPhysics.RevoluteJoint)), None)
        if prim is None:
            return
        UsdPhysics.DriveAPI.Get(prim, "angular").GetMaxForceAttr().Set(float(GRIP_TORQUE_MAX))
        print(f"[AM-T650-PNP] gripper drive torque capped at {GRIP_TORQUE_MAX} N.m "
              f"(06: 1000) -- a closed gripper squeezes the handle with a bounded force",
              flush=True)

    def __init__(self):
        super().__init__()
        from fsc_aerial_manipulation.utils import ROS2RigidBodyBackend

        # payload_0 = the payload, live ground truth. The handle exists only once PhysX has
        # seen the prim, hence the guard (06's marker-cube pattern).
        class _PayloadBackend(ROS2RigidBodyBackend):
            def update_sim_state(this, dt):
                if not this.get_dc_interface().get_rigid_body(this.payload_path):
                    return
                super().update_sim_state(dt)

        self._payload_backend = _PayloadBackend(
            world=self.world, payload_path=PAYLOAD_PRIM,
            config={"topic_prefix": PAYLOAD_TRUTH_BODY, "pub_state": True})
        self._probed = False
        self._gt_phase = "idle" if GRASP_TEST else "done"
        if GRASP_TEST:
            print("\033[1;33m[AM-T650-PNP] GRASP TEST armed: the payload will be moved into "
                  f"the gripper at t = {GRASP_TEST_T0} s -- not a flight scenario.\033[0m", flush=True)

        if not am06.HEADLESS:
            self.pg.set_viewport_camera(CAMERA_POS, CAMERA_TARGET)
        self._wp_ready = False
        if WAYPOINTS_VIZ:
            self._wp_setup_view()

        sz = PAYLOAD_SIZE[2]
        print(f"\033[1;35m[AM-T650-PNP] PICK-AND-PLACE SCENE: vehicle at "
              f"({SPAWN_XY[0]:.2f}, {SPAWN_XY[1]:.2f}) m, yaw {SPAWN_YAW_DEG:.0f} deg (+x forward); pillars {PILLAR_HEIGHT} m tall, "
              f"{PILLAR_DIAMETER * 1e3:.0f} mm diameter under a {CAP_DIAMETER * 1e3:.0f} x {CAP_THICKNESS * 1e3:.0f} mm cap, PICK at {PICK_PILLAR_XY} (payload box -> mocap "
              f"{PICK_BODY}), PLACE at {PLACE_PILLAR_XY} (top -> mocap {DROP_BODY}); payload "
              f"{PAYLOAD_MASS * 1e3:.0f} g = box "
              f"{PAYLOAD_SIZE[0] * 1e3:.0f} x {PAYLOAD_SIZE[1] * 1e3:.0f} x {sz * 1e3:.0f} mm + "
              f"handle {HANDLE_THICKNESS * 1e3:.0f} x {HANDLE_WIDTH * 1e3:.0f} x "
              f"{HANDLE_HEIGHT * 1e3:.0f} mm (payload yaw {PAYLOAD_YAW_DEG:.1f} deg, thin axis = its y), box centre "
              f"{np.round(self._payload_p0, 4).tolist()} m (ground truth {PAYLOAD_TRUTH_BODY}/state), "
              f"handle top {self._payload_p0[2] + 0.5 * sz + HANDLE_HEIGHT:.4f} m. EE offset (pick AND "
              f"place) [0, 0, {PICK_EE_OFFSET_Z:.2f}] from the box centre (claw "
              f"{(0.5 * sz + HANDLE_HEIGHT - PICK_EE_OFFSET_Z) * 1e3:.1f} mm below the handle top); place "
              f"point = the place pillar top + {(0.5 * sz + PLACE_DROP) * 1e3:.1f} mm (the box CG at release). Friction "
              f"{GRIP_FRICTION_STATIC}/{GRIP_FRICTION_DYNAMIC} (combine max). Unmodelled by "
              f"every controller.\033[0m", flush=True)

    def _setup_arm_ros2_bridge(self):
        super()._setup_arm_ros2_bridge()
        from geometry_msgs.msg import PoseStamped, TwistStamped
        from rclpy.qos import qos_profile_sensor_data
        node = self._arm_node
        # the mocap bodies, the emulator's input format: obj_0 = the payload box
        # (live, below), drop_0 = the PLACE pillar top (fixed)
        self._fixed_bodies = []
        self._payload_h = None
        for body, (x, y) in ((PICK_BODY, PICK_PILLAR_XY), (DROP_BODY, PLACE_PILLAR_XY)):
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x, pose.pose.position.y = float(x), float(y)
            pose.pose.position.z = float(PILLAR_HEIGHT)
            pose.pose.orientation.w = 1.0
            self._fixed_bodies.append((
                node.create_publisher(PoseStamped, f"/{body}/state/pose", qos_profile_sensor_data),
                [node.create_publisher(TwistStamped, f"/{body}/state/{t}", qos_profile_sensor_data)
                 for t in ("twist", "twist_inertial")],
                pose))
        self._fixed_twist = TwistStamped()
        # claw_0 = the REAL claw point (07's probe: 39 mm out along the claw
        # axis from the two pads' CoM midpoint), ground truth only -- what the
        # planner's model claw (current_ee) should coincide with. Lets the
        # EE offsets cancel any systematic model-vs-asset difference.
        self._claw_pub = node.create_publisher(PoseStamped, f"/{CLAW_TRUTH_BODY}/state/pose",
                                               qos_profile_sensor_data)
        self._claw_msg = PoseStamped()
        self._claw_msg.header.frame_id = "map"
        self._claw_pad_com = None
        self._fixed_twist.header.frame_id = "map"
        self._fixed_step = 0
        if WAYPOINTS_VIZ:
            self._wp_setup_ros()

    def _payload_pose(self):
        """The payload body's raw dc pose (its origin = the BOX centre), or None."""
        if not self._payload_h:
            self._payload_h = self._dc.get_rigid_body(PAYLOAD_PRIM)
            if not self._payload_h:
                return None
        return self._dc.get_rigid_body_pose(self._payload_h)

    def _pose(self, h):
        P = self._dc.get_rigid_body_pose(h)
        return (np.array([P.p.x, P.p.y, P.p.z]),
                am06.C.quat_to_rot(P.r.w, P.r.x, P.r.y, P.r.z))

    def _pad_geometry(self):
        """(pad centre L, pad centre R, their midpoint, unit closing axis L->R,
        wrist position, wrist rotation), world frame; None if not found."""
        dc = self._dc
        pads = [dc.get_rigid_body(self.drone_path + p) for p in am06.EE_MARKER_PAD_LINKS]
        wrist = dc.get_rigid_body(self.drone_path + am06.EE_MARKER_WRIST_LINK)
        if not all(pads) or not wrist:
            return None
        com = []
        for path, h in zip(am06.EE_MARKER_PAD_LINKS, pads):
            p, R = self._pose(h)
            c = UsdPhysics.MassAPI(self.stage.GetPrimAtPath(self.drone_path + path)) \
                .GetCenterOfMassAttr().Get()
            com.append(p + R @ np.array([c[0], c[1], c[2]], float))
        sep = com[1] - com[0]
        p_w, R_w = self._pose(wrist)
        return com[0], com[1], 0.5 * (com[0] + com[1]), sep / np.linalg.norm(sep), p_w, R_w

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
        # position only (orientation left identity): the claw point is what is scored
        self._claw_pub.publish(m)

    def _grip_angle(self):
        if self._grip_idx is None:
            return float("nan")
        return float(self._art.get_joint_positions()[self._grip_idx])

    def _probe_gripper(self):
        """Raycast between the two finger pads: their inner-face gap and the
        axis the jaws close along, in the world and in the vehicle body frame."""
        from omni.physx import get_physx_scene_query_interface
        g = self._pad_geometry()
        if g is None:
            print("[AM-T650-PNP] gripper probe: pad bodies not found", flush=True)
            return
        cl, cr, mid, u, p_w, R_w = g
        sep = cr - cl
        sq = get_physx_scene_query_interface()
        hits = []
        for d in (u, -u):
            r = sq.raycast_closest(tuple(map(float, mid)), tuple(map(float, d)), 0.2)
            hits.append((r.get("hit", False), float(r.get("distance", float("nan"))),
                         str(r.get("rigidBody", ""))))
        gap = hits[0][1] + hits[1][1] if hits[0][0] and hits[1][0] else float("nan")
        p_b, R_b = self._pose(self._body)
        u_body = R_b.T @ u
        grip = self._grip_angle()
        print(f"\033[1;36m[AM-T650-PNP] GRIPPER PROBE (gripper at {math.degrees(grip):.1f} deg): "
              f"pad CoM separation {np.linalg.norm(sep) * 1e3:.1f} mm, inner-face gap "
              f"{gap * 1e3:.1f} mm (rays {hits[0][1] * 1e3:.1f} / {hits[1][1] * 1e3:.1f} mm, "
              f"bodies {hits[0][2].split('/')[-1]} / {hits[1][2].split('/')[-1]}); closing axis "
              f"world {np.round(u, 3).tolist()} = body(FLU) {np.round(u_body, 3).tolist()}; "
              f"pad midpoint in the wrist frame {np.round(R_w.T @ (mid - p_w), 4).tolist()} m\033[0m",
              flush=True)
        # Along the claw axis a (the wrist's -z, out through the claw): where
        # the pads' inner faces exist (rays across the gap at each station),
        # and the first gripper body ABOVE the pads (the deepest a handle can
        # go between the jaws). s is measured from the pad midpoint, + = out.
        a = R_w @ np.array([0.0, 0.0, -1.0])
        pad_names = tuple(p.split('/')[-1] for p in am06.EE_MARKER_PAD_LINKS)
        both = []
        for s in np.arange(-0.060, 0.0601, 0.002):
            o = mid + s * a
            ok = True
            for d in (u, -u):
                r = sq.raycast_closest(tuple(map(float, o)), tuple(map(float, d)), 0.04)
                if not (r.get("hit", False) and str(r.get("rigidBody", "")).split('/')[-1] in pad_names):
                    ok = False
                    break
            if ok:
                both.append(s)
        up = sq.raycast_closest(tuple(map(float, mid)), tuple(map(float, -a)), 0.2)
        claw = 0.039                                   # the model's claw point, + along a (probe above)
        if both:
            print(f"\033[1;36m[AM-T650-PNP] PAD EXTENT along the claw axis (from the pad midpoint, + = out): "
                  f"[{min(both) * 1e3:+.0f}, {max(both) * 1e3:+.0f}] mm, i.e. the pad faces span "
                  f"{(claw - max(both)) * 1e3:.0f}..{(claw - min(both)) * 1e3:.0f} mm ABOVE the claw point; "
                  f"first body above the pad midpoint: "
                  f"{(float(up['distance']) * 1e3 if up.get('hit', False) else float('nan')):.0f} mm "
                  f"({str(up.get('rigidBody', '-')).split('/')[-1]}) = "
                  f"{((float(up['distance']) if up.get('hit', False) else float('nan')) + claw) * 1e3:.0f} mm "
                  f"above the claw point\033[0m", flush=True)
        else:
            print("[AM-T650-PNP] PAD EXTENT: no station with both pad faces in reach", flush=True)
        # The jaw GAP along the claw axis (the fingers swing on a hub, so it
        # need not be constant): rays across the gap at each height, on the
        # centre line and +-25 mm along the handle's long axis (l = a x u).
        l_ax = np.cross(a, u)
        l_ax /= np.linalg.norm(l_ax)
        rows = []
        for h in np.arange(-0.016, 0.0641, 0.004):            # height above the claw point
            o0 = mid + (claw - h) * a
            gaps = []
            for dl in (-0.025, 0.0, 0.025):
                o = o0 + dl * l_ax
                d2 = []
                for d in (u, -u):
                    r = sq.raycast_closest(tuple(map(float, o)), tuple(map(float, d)), 0.06)
                    d2.append(float(r["distance"]) if r.get("hit", False) and
                              str(r.get("rigidBody", "")).startswith(self.drone_path) else float("nan"))
                gaps.append(d2[0] + d2[1])
            rows.append((h, gaps))
        print("[AM-T650-PNP] JAW GAP vs height above the claw point [mm] (l = -25 / 0 / +25 mm): " +
              "  ".join(f"{h * 1e3:+.0f}: " + "/".join("-" if g != g else f"{g * 1e3:.0f}" for g in gs)
                        for h, gs in rows), flush=True)
        # The lowest gripper surface over a 30 x 60 mm handle footprint: rays
        # cast UP the claw axis from 30 mm below the claw point. The handle
        # top can rise no higher than this between the open jaws.
        hits = []
        for dc in np.arange(-0.015, 0.0151, 0.005):
            for dl in np.arange(-0.030, 0.0301, 0.010):
                o = mid + (claw + 0.030) * a + dc * u + dl * l_ax
                r = sq.raycast_closest(tuple(map(float, o)), tuple(map(float, -a)), 0.20)
                if r.get("hit", False) and str(r.get("rigidBody", "")).startswith(self.drone_path):
                    hits.append((float(r["distance"]) - 0.030, dc, dl, str(r["rigidBody"]).split('/')[-1]))
        if hits:
            lo = min(hits)
            print(f"\033[1;36m[AM-T650-PNP] HANDLE CEILING: the lowest gripper surface over a 30 x 60 mm "
                  f"handle footprint is {lo[0] * 1e3:.1f} mm above the claw point (at c {lo[1] * 1e3:+.0f}, "
                  f"l {lo[2] * 1e3:+.0f} mm, {lo[3]}); per row min "
                  + ", ".join(f"l {dl * 1e3:+.0f}: {min(h[0] for h in hits if abs(h[2] - dl) < 1e-6) * 1e3:.0f}"
                              for dl in sorted({h[2] for h in hits})) + " mm\033[0m", flush=True)

    def _probe_footprint(self):
        """The vehicle's UNDERSIDE in its own frame: rays cast upward on a 2 cm
        grid from 3 mm above the floor (the vehicle is seated). Where the body
        comes within 0.10 m of the floor is the landing gear; its extent bounds
        how close the gear can get to a handle under the claw."""
        from omni.physx import get_physx_scene_query_interface
        sq = get_physx_scene_query_interface()
        p_b, R_b = self._pose(self._body)
        z0 = 0.003
        low = []
        for x in np.arange(-0.40, 0.4001, 0.02):
            for y in np.arange(-0.40, 0.4001, 0.02):
                o = p_b + R_b @ np.array([x, y, 0.0])
                r = sq.raycast_closest((float(o[0]), float(o[1]), z0), (0.0, 0.0, 1.0), 0.6)
                if not r.get("hit", False):
                    continue
                body = str(r.get("rigidBody", ""))
                if not body.startswith(self.drone_path):
                    continue
                z_rel = z0 + float(r["distance"]) - float(p_b[2])     # underside height vs the body origin
                low.append((x, y, z_rel, body.split('/')[-1]))
        gear = [g for g in low if g[2] < -0.20]
        if not gear:
            print("[AM-T650-PNP] FOOTPRINT PROBE: no underside below -0.20 m found", flush=True)
            return
        g = np.array([[a, b, c] for a, b, c, _ in gear])
        parts = sorted({n for *_, n in gear})
        print(f"\033[1;36m[AM-T650-PNP] FOOTPRINT PROBE (body frame, x = nose, y = left; "
              f"body origin {p_b[2]:.3f} m above the floor): underside below -0.20 m at "
              f"{len(gear)} of {len(low)} grid points, x [{g[:, 0].min():+.2f}, {g[:, 0].max():+.2f}] m, "
              f"y [{g[:, 1].min():+.2f}, {g[:, 1].max():+.2f}] m, lowest {g[:, 2].min():+.3f} m; "
              f"bodies {parts}\033[0m", flush=True)
        # per forward station: the lowest underside, so the gear profile reads off
        for x in np.arange(-0.40, 0.4001, 0.04):
            col = [c for a, b, c, _ in low if abs(a - x) < 0.011]
            ys = sorted(b for a, b, c, _ in low if abs(a - x) < 0.011 and c < -0.20)
            if col:
                print(f"[AM-T650-PNP]   x {x:+.2f} m: lowest underside {min(col):+.3f} m "
                      f"({len(ys)} gear points{', y ' + ' '.join(f'{v:+.2f}' for v in ys) if ys else ''})",
                      flush=True)

    def _control_step_inner(self, dt):
        super()._control_step_inner(dt)
        self._fixed_step += 1
        if self._fixed_step % FIXED_PUB_DIV == 0:
            stamp = self._arm_node.get_clock().now().to_msg()
            self._fixed_twist.header.stamp = stamp
            for i, (pose_pub, twist_pubs, pose) in enumerate(self._fixed_bodies):
                if i == 0:                    # obj_0: the payload box's live pose
                    pb = self._payload_pose()
                    if pb is None:
                        continue              # PhysX has not seen it yet
                    P = pb
                    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = \
                        float(P.p.x), float(P.p.y), float(P.p.z)
                    o = pose.pose.orientation
                    o.x, o.y, o.z, o.w = float(P.r.x), float(P.r.y), float(P.r.z), float(P.r.w)
                pose.header.stamp = stamp
                pose_pub.publish(pose)
                for pub in twist_pubs:
                    pub.publish(self._fixed_twist)
            try:
                self._publish_claw_truth(stamp)
            except Exception:                 # a ground-truth topic must never stop the plant
                pass
        if not self._probed and self._t >= PROBE_AT_S:
            self._probed = True
            try:
                self._probe_gripper()
                self._probe_footprint()
            except Exception as exc:          # a probe must never stop the plant
                print(f"[AM-T650-PNP] gripper probe failed: {exc}", flush=True)
        if GRASP_TEST and self._gt_phase != "done":
            try:
                self._grasp_test_step()
            except Exception as exc:
                self._gt_phase = "done"
                print(f"[AM-T650-PNP] GRASP TEST aborted: {exc}", flush=True)
        if self._wp_ready:
            try:
                self._wp_poll()
                if self._t - self._wp_last >= WP_REFRESH_S:
                    self._wp_last = self._t
                    self._wp_refresh()
            except Exception as exc:          # a view must never stop the plant
                self._wp_ready = False
                print(f"[AM-T650-PNP] waypoint view stopped: {exc}", flush=True)

    # ── waypoint view ──────────────────────────────────────────────────────
    def _wp_setup_ros(self):
        from rcl_interfaces.srv import GetParameters
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import Float64MultiArray
        node = self._arm_node
        self._wp_info = None
        self._wp_params = {}
        self._wp_future = None
        self._wp_poll_t = -1e9
        self._wp_GetParameters = GetParameters
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(Float64MultiArray, PP_INFO_TOPIC,
                                 lambda m: setattr(self, "_wp_info", list(m.data)), latched)
        self._wp_client = node.create_client(GetParameters, PLANNER_NODE + "/get_parameters")

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
                with vw.get_frame("pick_place_waypoints"):
                    self._wp_sv = sc.SceneView()
                vw.viewport_api.add_scene_view(self._wp_sv)
                self._wp_ui, self._wp_cl, self._wp_sc = ui, cl, sc
            except Exception as exc:
                self._wp_sv = None
                print(f"[AM-T650-PNP] waypoint labels off ({exc}); markers and the "
                      f"printed table remain", flush=True)
        self._wp_ready = True
        print("[AM-T650-PNP] waypoint view on: blue = '<point> drone' body goal (yaw in the label), "
              "orange = '<point> EE' claw target, grey = 'Pick/Place drone' body once planned",
              flush=True)

    def _wp_poll(self):
        """Fetch the planner's nominal poses and claw offsets every WP_POLL_S."""
        import time
        if self._wp_future is not None and self._wp_future.done():
            try:
                for n, v in zip(self._wp_req_names, self._wp_future.result().values):
                    if v.type == 8:                       # PARAMETER_DOUBLE_ARRAY
                        self._wp_params[n] = list(v.double_array_value)
            except Exception:
                pass
            self._wp_future = None
        if (self._wp_future is None and time.monotonic() - self._wp_poll_t >= WP_POLL_S
                and self._wp_client.service_is_ready()):
            req = self._wp_GetParameters.Request()
            req.names = list(dict.fromkeys(list(WP_BASE_PARAMS.values())
                                           + list(WP_CLAW_PARAMS.values()) + [WP_PLACE_POINT_PARAM]))
            self._wp_req_names = list(req.names)
            self._wp_future = self._wp_client.call_async(req)
            self._wp_poll_t = time.monotonic()

    def _wp_entries(self):
        """key -> (kind, position, yaw_deg or None, name, source), the
        planner's own goals once planned, else what Plan would start from
        (its ppTargets logic: the base-pose parameters as they stand -- Adjust has
        already written its shift into them --, a captured base point's x, y)."""
        info = self._wp_info if self._wp_info is not None and len(self._wp_info) >= 80 else None

        def fin(i):
            return info is not None and math.isfinite(info[i])

        planned = info is not None and info[4] > 0.5
        out = {}
        for k in range(6):
            g, c = 14 + 7 * k, 56 + 4 * k
            name = f"{k + 1} {WP_NAMES[k]}"
            if planned and fin(g):
                if k in WP_CLAW_PARAMS:
                    out[f"{k}c"] = ("claw", np.array(info[g + 4:g + 7]), None, name + " EE", "planned")
                    out[f"{k}b"] = ("planbody", np.array(info[g:g + 3]), info[g + 3],
                                    name + " drone", "planned")
                else:
                    out[f"{k}"] = ("body", np.array(info[g:g + 3]), info[g + 3], name + " drone",
                                   "planned")
                continue
            captured = info is not None and fin(c + 3) and info[c + 3] > 0.5
            if k in WP_BASE_PARAMS:
                v = self._wp_params.get(WP_BASE_PARAMS[k])
                if v is None or len(v) != 4:
                    continue
                p, src = np.array(v[:3], float), "typed"   # Adjust already wrote its shift in
                if captured:
                    p[:2], src = info[c:c + 2], "captured"
                out[f"{k}"] = ("body", p, v[3], name + " drone", src)
            else:
                o = self._wp_params.get(WP_CLAW_PARAMS[k])
                o = np.array(o, float) if o is not None and len(o) == 3 else np.zeros(3)
                if captured:
                    p, src = np.array(info[c:c + 3], float), "captured"
                    if k == 1:            # the pick offset is in the captured OBJECT frame
                        yw = math.radians(info[80]) if len(info) > 80 and fin(80) else 0.0
                        o = np.array([math.cos(yw) * o[0] - math.sin(yw) * o[1],
                                      math.sin(yw) * o[0] + math.cos(yw) * o[1], o[2]])
                elif k == 1:              # Pick, not captured: the live box pose (what Get would give)
                    pb = self._payload_pose()
                    if pb is None:
                        continue
                    p, src = np.array([pb.p.x, pb.p.y, pb.p.z], float), "live, not captured"
                    yw = math.atan2(2.0 * (pb.r.w * pb.r.z + pb.r.x * pb.r.y),
                                    1.0 - 2.0 * (pb.r.y ** 2 + pb.r.z ** 2))
                    o = np.array([math.cos(yw) * o[0] - math.sin(yw) * o[1],
                                  math.sin(yw) * o[0] + math.cos(yw) * o[1], o[2]])
                elif k == 3:              # Place: the typed POSE, NOT shifted by Adjust (ppTargets)
                    v = self._wp_params.get(WP_PLACE_POINT_PARAM)
                    if v is None or len(v) not in (3, 4):
                        continue
                    p, src = np.array(v[:3], float), "typed"
                    yw = math.radians(v[3]) if len(v) == 4 else 0.0   # the offset turns with it
                    o = np.array([math.cos(yw) * o[0] - math.sin(yw) * o[1],
                                  math.sin(yw) * o[0] + math.cos(yw) * o[1], o[2]])
                out[f"{k}c"] = ("claw", p + o, None, name + " EE", src)
        return out

    def _wp_make(self, key, kind):
        root = f"{WP_ROOT}/wp_{key}"
        xf = UsdGeom.Xform.Define(self.stage, root)
        xa = UsdGeom.Xformable(xf)
        parts = {"xf": xf, "t": xa.AddTranslateOp(), "r": xa.AddRotateZOp()}
        rgb = {"body": WP_BODY_RGB, "claw": WP_CLAW_RGB, "planbody": WP_PLANBODY_RGB}[kind]
        ball = UsdGeom.Sphere.Define(self.stage, root + "/ball")
        ball.CreateRadiusAttr(WP_CLAW_RADIUS if kind == "claw" else WP_BODY_RADIUS)
        _color(ball, rgb)
        return parts

    def _wp_label(self, key, kind, p, text):
        if self._wp_sv is None:
            return
        sc, ui, cl = self._wp_sc, self._wp_ui, self._wp_cl
        M = sc.Matrix44.get_translation_matrix(float(p[0]), float(p[1]), float(p[2]) + 0.06)
        if key not in self._wp_labels:
            color = {"body": "#d6e8ff", "claw": "#ffc46b", "planbody": "#e6e6e6"}[kind]
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
        sig = tuple(sorted((k, e[0], tuple(np.round(e[1], 3)),
                            None if e[2] is None else round(float(e[2]), 1), e[4])
                           for k, e in entries.items()))
        if sig == self._wp_sig:
            return
        self._wp_sig = sig
        rows = []
        for key, (kind, p, yaw, name, src) in sorted(entries.items()):
            if key not in self._wp_prims:
                self._wp_prims[key] = self._wp_make(key, kind)
            parts = self._wp_prims[key]
            parts["t"].Set(Gf.Vec3d(*map(float, p)))
            parts["r"].Set(float(yaw or 0.0))
            UsdGeom.Imageable(parts["xf"].GetPrim()).MakeVisible()
            text = f"{name}  ({p[0]:.2f}, {p[1]:.2f}, {p[2]:.2f}) m"
            if yaw is not None:
                text += f"  yaw {yaw:.0f}°"
            self._wp_label(key, kind, p, text)       # the viewport: no source tag
            rows.append(f"{text}  [{src}]")          # the printed table keeps it
        for key, parts in self._wp_prims.items():
            if key not in entries:
                UsdGeom.Imageable(parts["xf"].GetPrim()).MakeInvisible()
                if key in self._wp_labels:
                    self._wp_labels[key][0].visible = False
        # the printed table only on a change of >= 1 cm / a source (the markers
        # follow every millimetre; a carried, uncaptured payload would spam)
        psig = tuple(sorted((k, tuple(np.round(e[1], 2)), e[4]) for k, e in entries.items()))
        if psig != getattr(self, "_wp_psig", None):
            self._wp_psig = psig
            print("\033[1;36m[AM-T650-PNP] WAYPOINTS:\n    " + "\n    ".join(rows) + "\033[0m",
                  flush=True)

    # ── the seated grasp test ──────────────────────────────────────────────
    def _gt_say(self, text):
        print(f"\033[1;36m[AM-T650-PNP] GRASP TEST t={self._t:6.2f}s: {text}\033[0m", flush=True)

    def _gt_hold_payload(self):
        dc = self._dc
        p, q_wxyz = self._gt_pose
        dc.set_rigid_body_pose(self._gt_h, am06._dynamic_control.Transform(
            tuple(map(float, p)), (float(q_wxyz[1]), float(q_wxyz[2]), float(q_wxyz[3]), float(q_wxyz[0]))))
        zero = am06.carb._carb.Float3(0.0, 0.0, 0.0)
        dc.set_rigid_body_linear_velocity(self._gt_h, zero)
        dc.set_rigid_body_angular_velocity(self._gt_h, zero)

    def _gt_rel(self):
        """The payload body's position relative to the pad midpoint, world."""
        p, _ = self._pose(self._gt_h)
        return p - self._pad_geometry()[2]

    def _grasp_test_step(self):
        t, ph = self._t, self._gt_phase
        if ph == "idle":
            if t < GRASP_TEST_T0:
                return
            self._gt_h = self._dc.get_rigid_body(PAYLOAD_PRIM)
            g = self._pad_geometry()
            if not self._gt_h or g is None:
                raise RuntimeError("payload or pad bodies not found")
            cl, cr, mid, u, p_w, R_w = g
            # payload frame: y = the closing axis (the handle's thin axis),
            # z = up, x = y x z (the handle's width, ~ along the claw)
            ez = np.array([0.0, 0.0, 1.0])
            z_p = ez - (ez @ u) * u
            z_p /= np.linalg.norm(z_p)
            x_p = np.cross(u, z_p)
            R_p = np.column_stack([x_p, u, z_p])
            s = 1.0 if (p_w - mid) @ x_p < 0.0 else -1.0     # the wrist's side (was y_p = -x_p)
            y_c = s * (GRASP_TEST_EDGE - 0.5 * HANDLE_WIDTH)  # handle centre vs pads, along -x_p
            m_local = np.array([y_c, 0.0, 0.5 * PAYLOAD_SIZE[2] + GRASP_TEST_GRIP_H])
            p_body = mid - R_p @ m_local
            self._gt_pose = (p_body, am06._rot_to_quat_wxyz(R_p))
            self._gt_hold_payload()
            self._grip_cmd = -am06.GRIPPER_LIMIT_RAD
            self._gt_t = t
            self._gt_q = [(t, self._grip_angle())]
            self._gt_phase = "close"
            self._gt_say(f"handle held between the open pads (closing axis "
                         f"{np.round(u, 3).tolist()}, |u_z| {abs(u[2]):.3f}); box bottom at "
                         f"{p_body[2] - 0.5 * PAYLOAD_SIZE[2]:.3f} m; closing the gripper")
        elif ph == "close":
            self._gt_hold_payload()
            qg = self._grip_angle()
            self._gt_q.append((t, qg))
            while self._gt_q and self._gt_q[0][0] < t - 0.3:
                self._gt_q.pop(0)
            moved = max(q for _, q in self._gt_q) - min(q for _, q in self._gt_q)
            if qg <= -am06.GRIPPER_LIMIT_RAD + math.radians(1.0):
                self._gt_phase = "done"
                self._gt_say(f"FAIL -- the jaws closed to {math.degrees(qg):.1f} deg (the limit): "
                             f"the handle is not clamped")
            elif t - self._gt_t > 1.0 and moved < math.radians(0.2):
                self._gt_stall = qg
                self._gt_rel0 = self._gt_rel()
                self._gt_t = t
                self._gt_next = t + 1.0
                self._gt_phase = "observe"
                self._gt_say(f"gripper STALLED on the handle at {math.degrees(qg):.1f} deg "
                             f"(limit -50); payload released -- watching for slip")
            elif t - self._gt_t > 8.0:
                self._gt_phase = "done"
                self._gt_say(f"FAIL -- the gripper never stalled ({math.degrees(qg):.1f} deg)")
        elif ph == "observe":
            d = self._gt_rel() - self._gt_rel0
            if t >= self._gt_next:
                self._gt_next += 1.0
                self._gt_say(f"payload moved {d[2] * 1e3:+.2f} mm vertically, "
                             f"{np.linalg.norm(d) * 1e3:.2f} mm total, relative to the pads; "
                             f"box bottom {self._pose(self._gt_h)[0][2] - 0.5 * PAYLOAD_SIZE[2]:.3f} m "
                             f"above the floor; gripper {math.degrees(self._grip_angle()):.1f} deg")
            if t - self._gt_t >= GRASP_TEST_OBSERVE_S:
                ok = np.linalg.norm(d) <= GRASP_TEST_SLIP_MAX
                self._gt_say(f"{'PASS' if ok else 'FAIL'} -- {np.linalg.norm(d) * 1e3:.2f} mm of slip "
                             f"in {GRASP_TEST_OBSERVE_S:.0f} s (limit "
                             f"{GRASP_TEST_SLIP_MAX * 1e3:.0f} mm), gripper held at "
                             f"{math.degrees(self._grip_angle()):.1f} deg; now OPENING")
                self._grip_cmd = 0.0
                self._gt_z_open = self._pose(self._gt_h)[0][2]
                self._gt_t = t
                self._gt_phase = "release"
        elif ph == "release":
            if t - self._gt_t >= 3.0:
                dz = self._pose(self._gt_h)[0][2] - self._gt_z_open
                self._gt_say(f"{'released' if dz < -0.05 else 'NOT released'} on OPEN: the payload "
                             f"moved {dz * 1e3:+.0f} mm vertically (gripper "
                             f"{math.degrees(self._grip_angle()):.1f} deg). Test done.")
                self._gt_phase = "done"


def main():
    AmT650PickAndPlace().run()


if __name__ == "__main__":
    main()
