#!/usr/bin/env python
"""
08_px4_t650_aerial_manipulator_mail_route.py  (2026-11-03)

THE MAIL-ROUTE SCENE -- the Isaac twin of the three-letter mail delivery (user
design, 2026-11-03): THREE red envelopes on three hats, posted ALTERNATELY into
the two mailboxes on the console table -- letter 1 the RED box, letter 2 the
BLUE box, letter 3 the RED box -- by the planner's mail route
(fsc_trajectory_planner pick_place_route) flown from the arm GS's "Mail Route"
tab, one letter at a time, the operator choosing which.

A COPY of 07_px4_t650_aerial_manipulator_mail_delivery.py (user: "just copy"),
edited for three hats and two live mailboxes; 07 is not touched. Like 07 it
loads 07_px4_t650_aerial_manipulator_pick_and_place.py as a module (which loads
06) and subclasses its sim class -- the gripper's torque cap, the obj_0 / drop_0
mocap publishing, the claw ground truth, the probes and the waypoint view are
that scene's. What differs from 07:

  hats      THREE pillars + printed hats in a LINE in front of the mailboxes,
            extending away from them along -x: the nearest
            PEGASUS_ROUTE_HAT_LINE x (default 0.70 m: 1.0 m in front of the box
            fronts, beyond Mailbox Start), then every spacing (default 0.6 m)
            further out, all on y (default 0.0, between the boxes)
  envelopes ONE per hat, the 07 envelope each (150 x 90 x 4 mm, 60 g, weighted far
            end, the invisible grip pad), TURNED 90 deg by default
            (PEGASUS_MAIL_ENVELOPE_YAW_DEG 90): the near edge overhangs the hat's
            -y rim, so each pick comes in from the side of the line and the
            vehicle turns to the boxes on the way there. Ground truth:
            payload_0 (letter 1, the base scene's payload prim), payload_1,
            payload_2
  mailboxes BOTH live: each envelope's monitor says which box it ended in
  frame     the printed LOCALIZATION FRAME (07's), placed on hat 1, 2, 3, the RED
            box's top or the BLUE box's top: services /mail_frame/on_hat_1,
            /on_hat_2, /on_hat_3, /on_red, /on_blue, /remove. With
            PEGASUS_MAIL_FRAME_AUTO_MOVE (default 1) it follows the operator's
            Gets by itself, on the planner's parameter events: it starts on hat 1
            and goes to the first point the route has not measured yet (hat 1,
            hat 2, hat 3, RED, BLUE), then lifts off -- the five Get buttons are
            the whole procedure
  vehicle   spawned OFF the line of hats (PEGASUS_PNP_SPAWN_XY default -0.5,-1.0)

Knobs (environment; 07's PEGASUS_MAIL_* apply as there, except the pillar's):
  PEGASUS_ROUTE_HAT_LINE   "x_nearest,y,spacing" [m], default "0.70,0.0,0.6"
  PEGASUS_MAIL_PILLAR_HEIGHT  every pillar's top under its hat [m], default 0.77
  PEGASUS_MAIL_FRAME       where the frame starts: hat1 (default) | hat2 | hat3 | red | blue | off

Run with (the route's own launcher):
  scripts/indoor_sim/start_t650_aerial_manipulator_whole_body_L1_4D_mail_route_sitl.sh <config>
"""

import importlib.util
import json
import math
import os
import sys
from types import SimpleNamespace

# The pick-and-place scene reads its knobs at import: this scene builds its own
# payloads and drops the place pillar and the bin, so pin the ones that would
# otherwise need assets or print a misleading banner.
os.environ["PEGASUS_PNP_TASK"] = "drop"          # 0.77 m pillar default (the lab's)
os.environ["PEGASUS_PNP_BIN"] = "0"
os.environ["PEGASUS_PNP_PAYLOAD"] = "plate"     # no CAD asset: the envelopes are built here
os.environ["PEGASUS_PNP_GRASP_TEST"] = "0"
os.environ["PEGASUS_PNP_PLATFORM"] = "hat"
# the vehicle stands OFF the line of hats (the base scene's default start mark is on it)
if not (os.environ.get("PEGASUS_PNP_SPAWN_XY", "") or "").strip():
    os.environ["PEGASUS_PNP_SPAWN_XY"] = "-0.5,-1.0"

_HERE = os.path.dirname(os.path.abspath(__file__))
_spec = importlib.util.spec_from_file_location(
    "am07_pick_place", os.path.join(_HERE, "07_px4_t650_aerial_manipulator_pick_and_place.py"))
pnp = importlib.util.module_from_spec(_spec)
sys.modules["am07_pick_place"] = pnp
_spec.loader.exec_module(pnp)           # starts the SimulationApp (06), defines the PnP scene
am06 = pnp.am06

import numpy as np                      # noqa: E402  (after SimulationApp, see memory note)
import omni.usd                         # noqa: E402
from pxr import Gf, Usd, UsdGeom, UsdPhysics  # noqa: E402


def _floats(name, default, n):
    raw = (os.environ.get(name, "") or default).strip()
    try:
        v = tuple(float(x) for x in raw.split(","))
        assert len(v) == n
    except (ValueError, AssertionError):
        raise SystemExit(f"{name} must be {n} comma-separated numbers, got {raw!r}")
    return v


TAG = "[AM-T650-ROUTE]"

# ── the envelopes (07's, one per hat) ─────────────────────────────────────────
ENV_L, ENV_W, ENV_T = _floats("PEGASUS_MAIL_ENVELOPE_SIZE", "0.150,0.090,0.004", 3)
ENV_MASS = am06._envf("PEGASUS_MAIL_ENVELOPE_MASS", 0.060)                 # [kg]
ENV_COM_FROM_FAR = am06._envf("PEGASUS_MAIL_ENVELOPE_COM", 0.040)          # [m]
ENV_OVERHANG = am06._envf("PEGASUS_MAIL_OVERHANG", 0.070)                  # [m]
ENV_YAW_DEG = am06._envf("PEGASUS_MAIL_ENVELOPE_YAW_DEG", 90.0)            # [deg] turned 90: picked from the side
GRIP_MM = am06._envf("PEGASUS_MAIL_GRIP_MM", 20.0)                         # [mm]
GRIP_DEPTH = 0.045                      # [m] the pad covers the near 45 mm (the jaws take ~31 mm)
TIP_MASS_FROM_FAR = 0.015               # [m] where the steel strip's centre sits, from the far end
ENV_FRICTION = (0.6, 0.5)               # paper over PLA, static / dynamic
ENV_RED = (0.78, 0.07, 0.07)
ENV_LABEL = (0.96, 0.95, 0.90)          # the address label (visual only)
ENV_STAMP = (0.15, 0.35, 0.75)
if not 0.0 < ENV_COM_FROM_FAR <= 0.5 * ENV_L + 1e-9:
    raise SystemExit(f"PEGASUS_MAIL_ENVELOPE_COM must be in (0, L/2], got {ENV_COM_FROM_FAR}")

# ── the three pick pillars, in a line ───────────────────────────────────────────
N_LETTERS = 3
HAT_X0, HAT_Y, HAT_SPACING = _floats("PEGASUS_ROUTE_HAT_LINE", "0.70,0.0,0.6", 3)
HAT_XY = [(HAT_X0 - HAT_SPACING * k, HAT_Y) for k in range(N_LETTERS)]   # nearest first
PILLAR_HEIGHT = am06._envf("PEGASUS_MAIL_PILLAR_HEIGHT", 0.77)            # [m] under the hat
PLATFORM_TOP = PILLAR_HEIGHT + pnp.PLATFORM_ABOVE_PILLAR                  # [m] the hat's top
PLATFORM_RADIUS = 0.5 * pnp.PLATFORM_DIAMETER
if HAT_SPACING < 2 * PLATFORM_RADIUS + 0.05:
    raise SystemExit(f"{TAG} PEGASUS_ROUTE_HAT_LINE spacing {HAT_SPACING} m: the hats ({2 * PLATFORM_RADIUS:.3f} m) overlap")
# envelope k's prim and ground-truth body: letter 1 is the base scene's payload
ENV_PRIM = [pnp.PAYLOAD_PRIM] + [f"{pnp.SCENE_ROOT}/payload_{k}" for k in range(1, N_LETTERS)]
ENV_TRUTH = [pnp.PAYLOAD_TRUTH_BODY] + [f"payload_{k}" for k in range(1, N_LETTERS)]

# ── the table and the mailboxes (07's) ──────────────────────────────────────────
TABLE_FRONT_X = am06._envf("PEGASUS_MAIL_TABLE_FRONT_X", 1.70)
TABLE_Y = am06._envf("PEGASUS_MAIL_TABLE_Y", 0.0)
TABLE_LENGTH_Y = 1.000                  # [m] HOOBRO BB52UXG01G1 (the box-pushing scene's)
TABLE_DEPTH_X = 0.150
TABLE_HEIGHT = 0.800
TABLE_TOP_THICK = 0.018
TABLE_LEG = 0.025
TABLE_SHELF_Z = 0.250
TABLE_SHELF_THICK = 0.015
BOX_W, BOX_D, BOX_H = _floats("PEGASUS_MAIL_BOX_SIZE", "0.235,0.125,0.2725", 3)
WALL = am06._envf("PEGASUS_MAIL_WALL", 0.005)
MOUTH_W, MOUTH_H, MOUTH_Z = _floats("PEGASUS_MAIL_MOUTH", "0.190,0.090,0.2025", 3)
RED_Y = am06._envf("PEGASUS_MAIL_RED_Y", 0.25)
BLUE_Y = am06._envf("PEGASUS_MAIL_BLUE_Y", -0.25)
RED_RGB = (0.80, 0.06, 0.06)
BLUE_RGB = (0.08, 0.22, 0.72)
TRIM_RGB = (0.12, 0.12, 0.12)
if not (0.0 < MOUTH_W < BOX_W - 2 * WALL and 0.5 * MOUTH_H + WALL < MOUTH_Z < BOX_H - 0.5 * MOUTH_H - WALL):
    raise SystemExit(f"{TAG} the mouth {MOUTH_W} x {MOUTH_H} at {MOUTH_Z} does not fit the {BOX_W} x {BOX_H} face")
for _y in (RED_Y, BLUE_Y):
    if abs(_y - TABLE_Y) > 0.5 * TABLE_LENGTH_Y - 0.5 * BOX_W + 1e-9:
        raise SystemExit(f"{TAG} a mailbox at y = {_y} overhangs the table's end")
if abs(RED_Y - BLUE_Y) < BOX_W:
    raise SystemExit(f"{TAG} the two mailboxes overlap (|{RED_Y} - {BLUE_Y}| < {BOX_W})")
if TABLE_FRONT_X - max(x for x, _ in HAT_XY) < 0.75:
    print(f"\033[1;33m{TAG} WARNING: the nearest hat is only {TABLE_FRONT_X - max(x for x, _ in HAT_XY):.2f} m in "
          f"front of the boxes -- Mailbox Start (0.709 m out) then hovers over it\033[0m", flush=True)

# ── the airframe and the arm's stops (07's) ─────────────────────────────────────
AM_ASSET = (os.environ.get("PEGASUS_MAIL_AM_ASSET", "") or "AM_T650.usda").strip()
am06.USD_FILE = os.path.join(am06.ASSETS_DIR, AM_ASSET)
if not os.path.isfile(am06.USD_FILE):
    raise SystemExit(f"{am06.USD_FILE} not found -- generate it with "
                     f"robotic_arm/utils_model/make_t650_gear_asset.py (or set PEGASUS_MAIL_AM_ASSET)")
am06.GROUND_BODY_Z = 0.305 - 0.070 if AM_ASSET == "AM_T650.usda" else 0.305

# ── the printed localization frame (07's) ───────────────────────────────────────
FRAME_CAD = os.path.normpath(os.path.join(_HERE, "..", "..", "docs", "docs_aerial_manipulator", "archive",
                                          "mail_delivery_20261023", "cad"))
with open(os.path.join(FRAME_CAD, "localization_frame.json")) as _f:
    FRAME = json.load(_f)
FRAME_CENTROID = np.array(FRAME["markers"]["centroid"], float) * 1e-3   # [m] obj_0 from O, frame axes
FRAME_LOCATOR = {k: np.array(v[:3], float) for k, v in FRAME["locator"].items()}   # the planner's
FRAME_SPOTS = [f"hat{k + 1}" for k in range(N_LETTERS)] + ["red", "blue"]   # the Gets' order
FRAME_START = (os.environ.get("PEGASUS_MAIL_FRAME", "") or "hat1").strip().lower()
if FRAME_START not in FRAME_SPOTS + ["off"]:
    raise SystemExit(f"PEGASUS_MAIL_FRAME must be one of {FRAME_SPOTS + ['off']}, got {FRAME_START!r}")
FRAME_AUTO_OFF = (os.environ.get("PEGASUS_MAIL_FRAME_AUTO_OFF", "") or "0").strip().lower() not in ("0", "false", "no")
FRAME_AUTO_MOVE = (os.environ.get("PEGASUS_MAIL_FRAME_AUTO_MOVE", "") or "1").strip().lower() not in ("0", "false", "no")
FRAME_LIFT_Z = 0.15                     # [m] above the vehicle's rest height = "it took off"
FRAME_PRIM = pnp.SCENE_ROOT + "/localization_frame"
FRAME_RGB = (0.56, 0.69, 0.82)          # the drawing page's plate
HOLDER_RGB = {"A": (0.165, 0.471, 0.839), "B": (0.929, 0.631, 0.0),    # the page's design colours
              "C": (0.910, 0.482, 0.643), "D": (0.0, 0.514, 0.0)}
MARKER_RGB = (0.95, 0.95, 0.95)
FRAME_WHERE = {**{f"hat{k + 1}": f"hat {k + 1}" for k in range(N_LETTERS)},
               "red": "the red mailbox's top", "blue": "the blue mailbox's top", "off": "lifted off"}
ROUTE_PARAMS = ("pick_place_route_letter_points", "pick_place_route_box_points")


def _read_stl_mm(path):
    """A binary STL's triangles, (N, 3, 3) [mm]."""
    with open(path, "rb") as f:
        b = f.read()
    n = int.from_bytes(b[80:84], "little")
    if len(b) < 84 + 50 * n:
        raise SystemExit(f"{TAG} {path}: not a binary STL")
    rec = np.frombuffer(b, dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", 9), ("a", "<u2")]), count=n, offset=84)
    return rec["v"].reshape(n, 3, 3).astype(float)


def _rz(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _frame_place(where):
    """(O, yaw_deg) of the frame placed on `where`, or None: O = the centre of its
    underside, world. On a hat its skirts centre it on the cap and its slot holds
    the envelope (frame yaw = the envelope's); on a mailbox its skirt flats close
    on the front and back faces, centred across the box, +x into the mouth."""
    if where.startswith("hat"):
        x, y = HAT_XY[int(where[3:]) - 1]
        return np.array([x, y, PLATFORM_TOP]), ENV_YAW_DEG
    if where in ("red", "blue"):
        return np.array([TABLE_FRONT_X + 0.5 * BOX_D, RED_Y if where == "red" else BLUE_Y,
                         TABLE_HEIGHT + BOX_H]), 0.0
    return None


_q2_raw = (os.environ.get("PEGASUS_MAIL_Q2_LIMIT_DEG", "") or "-20,45").strip()
Q2_LIMIT_DEG = None if _q2_raw.lower() == "asset" else _floats("PEGASUS_MAIL_Q2_LIMIT_DEG", "-20,45", 2)

# the waypoint view names the points as the arm GS's Mail Route tab does
pnp.WP_NAMES = ["Pick Start", "Letter", "Mailbox Start", "Mailbox", "Land Start", "Land"]
WATCH_S = 0.5                           # [s, sim] the envelope-state monitor
CAMERA_POS = [-2.0, -2.6, 2.4]          # behind-right of the line, high: the hats, the table and the route
CAMERA_TARGET = [0.6, 0.0, 0.8]


def _mailbox_parts(cx_front, cy):
    """(name, centre, size) of a hollow mailbox's walls, world frame: its FRONT
    face at x = cx_front (facing -x), centred on y = cy, standing on the table."""
    z0 = TABLE_HEIGHT
    x0, x1 = cx_front, cx_front + BOX_D
    xm = 0.5 * (x0 + x1)
    t = WALL
    parts = [
        ("floor", (xm, cy, z0 + 0.5 * t), (BOX_D, BOX_W, t)),
        ("roof", (xm, cy, z0 + BOX_H - 0.5 * t), (BOX_D, BOX_W, t)),
        ("back", (x1 - 0.5 * t, cy, z0 + 0.5 * BOX_H), (t, BOX_W, BOX_H - 2 * t)),
        ("side_l", (xm, cy + 0.5 * (BOX_W - t), z0 + 0.5 * BOX_H), (BOX_D - 2 * t, t, BOX_H - 2 * t)),
        ("side_r", (xm, cy - 0.5 * (BOX_W - t), z0 + 0.5 * BOX_H), (BOX_D - 2 * t, t, BOX_H - 2 * t)),
    ]
    # the FRONT, cut round the mouth: below, above, and the two jambs
    m_lo, m_hi = MOUTH_Z - 0.5 * MOUTH_H, MOUTH_Z + 0.5 * MOUTH_H
    fx = x0 + 0.5 * t
    parts.append(("front_lo", (fx, cy, z0 + t + 0.5 * (m_lo - t)), (t, BOX_W, m_lo - t)))
    parts.append(("front_hi", (fx, cy, z0 + 0.5 * (m_hi + BOX_H - t)), (t, BOX_W, BOX_H - t - m_hi)))
    jamb = 0.5 * (BOX_W - MOUTH_W)
    for side, s in (("front_jl", 1.0), ("front_jr", -1.0)):
        parts.append((side, (fx, cy + s * (0.5 * MOUTH_W + 0.5 * jamb), z0 + MOUTH_Z),
                      (t, jamb, MOUTH_H)))
    return parts


class AmT650MailRoute(pnp.AmT650PickAndPlace):

    def _spawn_am_px4_primary(self):
        self._build_scene(omni.usd.get_context().get_stage())
        drone_path = am06.AmT650WholeBodyArmSim._spawn_am_px4_primary(self)
        self._set_q2_limit(drone_path)    # 06 assigns self.drone_path from this return value
        return drone_path

    def _set_q2_limit(self, drone_path):
        if Q2_LIMIT_DEG is None:
            print(f"{TAG} q2 stops: the asset's", flush=True)
            return
        stage = omni.usd.get_context().get_stage()
        root = stage.GetPrimAtPath(drone_path)
        prim = next((p for p in Usd.PrimRange(root)
                     if p.GetName() == "manip_joint2" and p.IsA(UsdPhysics.RevoluteJoint)), None)
        if prim is None:
            raise SystemExit(f"{TAG} no manip_joint2 to put the q2 stops on")
        j = UsdPhysics.RevoluteJoint(prim)
        old = (j.GetLowerLimitAttr().Get(), j.GetUpperLimitAttr().Get())
        j.GetLowerLimitAttr().Set(float(Q2_LIMIT_DEG[0]))
        j.GetUpperLimitAttr().Set(float(Q2_LIMIT_DEG[1]))
        print(f"\033[1;35m{TAG} q2 hard stops {Q2_LIMIT_DEG[0]:.0f}..{Q2_LIMIT_DEG[1]:.0f} deg "
              f"(the real arm's range; asset {old[0]:.0f}..{old[1]:.0f})\033[0m", flush=True)

    # ── the scene ───────────────────────────────────────────────────────────
    def _build_scene(self, stage):
        root = pnp.SCENE_ROOT
        UsdGeom.Xform.Define(stage, root)
        pillar_mat = pnp._physics_material(stage, root + "/pillar_material",
                                           pnp.PILLAR_FRICTION_STATIC, pnp.PILLAR_FRICTION_DYNAMIC)
        env_mat = pnp._physics_material(stage, root + "/envelope_material", *ENV_FRICTION, combine="max")
        box_mat = pnp._physics_material(stage, root + "/mailbox_material", 0.5, 0.45)

        # the THREE pick pillars under their printed hats (the pick-and-place scene's)
        for k, (x, y) in enumerate(HAT_XY):
            cyl = UsdGeom.Cylinder.Define(stage, f"{root}/pick_pillar_{k + 1}")
            cyl.CreateRadiusAttr(0.5 * pnp.PILLAR_DIAMETER)
            cyl.CreateHeightAttr(PILLAR_HEIGHT)
            cyl.CreateAxisAttr("Z")
            UsdGeom.Xformable(cyl).AddTranslateOp().Set(Gf.Vec3d(x, y, 0.5 * PILLAR_HEIGHT))
            pnp._color(cyl, (0.30, 0.50, 0.80))
            UsdPhysics.CollisionAPI.Apply(cyl.GetPrim())
            pnp._bind_physics(cyl.GetPrim(), pillar_mat)
            hat = UsdGeom.Xform.Define(stage, f"{root}/pick_pillar_{k + 1}_hat")
            UsdGeom.Xformable(hat).AddTranslateOp().Set(Gf.Vec3d(x, y, PILLAR_HEIGHT))
            hat.GetPrim().GetReferences().AddReference(pnp.HAT_USD)
            for p in Usd.PrimRange(hat.GetPrim()):
                if p.HasAPI(UsdPhysics.CollisionAPI):
                    pnp._bind_physics(p, pillar_mat)

        # the thin console table (static colliders)
        tx = TABLE_FRONT_X + 0.5 * TABLE_DEPTH_X
        troot = root + "/table"
        UsdGeom.Xform.Define(stage, troot)
        wood, metal = (0.42, 0.30, 0.20), (0.15, 0.15, 0.15)
        parts = [pnp._box(stage, troot + "/top", (tx, TABLE_Y, TABLE_HEIGHT - 0.5 * TABLE_TOP_THICK),
                          (TABLE_DEPTH_X, TABLE_LENGTH_Y, TABLE_TOP_THICK), wood)]
        leg_h = TABLE_HEIGHT - TABLE_TOP_THICK
        for i, (sx, sy) in enumerate(((1, 1), (1, -1), (-1, 1), (-1, -1))):
            parts.append(pnp._box(stage, f"{troot}/leg{i}",
                                  (tx + sx * (0.5 * TABLE_DEPTH_X - 0.5 * TABLE_LEG),
                                   TABLE_Y + sy * (0.5 * TABLE_LENGTH_Y - 0.5 * TABLE_LEG), 0.5 * leg_h),
                                  (TABLE_LEG, TABLE_LEG, leg_h), metal))
        parts.append(pnp._box(stage, troot + "/shelf", (tx, TABLE_Y, TABLE_SHELF_Z - 0.5 * TABLE_SHELF_THICK),
                              (TABLE_DEPTH_X, TABLE_LENGTH_Y - 2 * TABLE_LEG, TABLE_SHELF_THICK), wood))
        for p in parts:
            UsdPhysics.CollisionAPI.Apply(p.GetPrim())
            pnp._bind_physics(p.GetPrim(), box_mat)

        # the two mailboxes: hollow, static, the mouth cut in the front
        for name, cy, rgb in (("mailbox_red", RED_Y, RED_RGB), ("mailbox_blue", BLUE_Y, BLUE_RGB)):
            mroot = f"{root}/{name}"
            UsdGeom.Xform.Define(stage, mroot)
            for part, centre, size in _mailbox_parts(TABLE_FRONT_X, cy):
                cube = pnp._box(stage, f"{mroot}/{part}", centre, size, rgb)
                UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
                pnp._bind_physics(cube.GetPrim(), box_mat)
            # a dark frame round the mouth (visual only, 2 mm proud of the face)
            e = 0.006
            z = TABLE_HEIGHT + MOUTH_Z
            fx = TABLE_FRONT_X - 0.001
            for k, (c, s) in enumerate((((fx, cy, z + 0.5 * MOUTH_H + 0.5 * e), (0.002, MOUTH_W + 2 * e, e)),
                                        ((fx, cy, z - 0.5 * MOUTH_H - 0.5 * e), (0.002, MOUTH_W + 2 * e, e)),
                                        ((fx, cy + 0.5 * MOUTH_W + 0.5 * e, z), (0.002, e, MOUTH_H)),
                                        ((fx, cy - 0.5 * MOUTH_W - 0.5 * e, z), (0.002, e, MOUTH_H)))):
                pnp._box(stage, f"{mroot}/trim{k}", c, s, TRIM_RGB)

        self._env = []
        for k in range(N_LETTERS):
            self._build_envelope(stage, env_mat, k)
        self._build_frame(stage)

    # ── the localization frame (visual; obj_0 while placed) ──────────────────
    def _build_frame(self, stage):
        """The printed frame from its CAD (07's): the plate, then each holder and its
        marker at their pockets. The marker centres it draws are checked against
        the JSON's, which the planner's locators were computed from."""
        xf = UsdGeom.Xform.Define(stage, FRAME_PRIM)
        self._frame_prim = xf.GetPrim()
        self._frame_ops = (xf.AddTranslateOp(), xf.AddRotateZOp())

        def mesh(path, tris_mm, rgb):
            m = UsdGeom.Mesh.Define(stage, path)
            pts = (tris_mm.reshape(-1, 3) * 1e-3).astype(float)
            m.CreatePointsAttr([Gf.Vec3f(*p) for p in pts])
            m.CreateFaceVertexCountsAttr([3] * len(tris_mm))
            m.CreateFaceVertexIndicesAttr(list(range(len(pts))))
            m.CreateSubdivisionSchemeAttr("none")
            m.CreateDoubleSidedAttr(True)
            pnp._color(m, rgb)

        mesh(FRAME_PRIM + "/plate", _read_stl_mm(os.path.join(FRAME_CAD, "localization_frame.stl")), FRAME_RGB)
        floor = FRAME["plate_t"] - FRAME["pocket_d"]           # [mm] the pockets' floor above O
        balls = []
        for k, h in enumerate(FRAME["holders"]):
            R, t = _rz(h["heading"]), np.array([h["x"], h["y"], floor])
            for part, rgb in (("holder", HOLDER_RGB[h["t"]]), ("marker", MARKER_RGB)):
                tris = _read_stl_mm(os.path.join(FRAME_CAD, "holders", f"{part}_{h['t']}.stl"))
                mesh(f"{FRAME_PRIM}/{part}{k}_{h['t']}", tris @ R.T + t, rgb)
            ball = R @ np.array(FRAME["designs"][h["t"]]["ball"], float) + t
            if np.linalg.norm(ball - np.array(h["ball"], float)) > 0.05:
                raise SystemExit(f"{TAG} frame holder {k} ({h['t']}): marker at {ball} vs the JSON's {h['ball']}")
            balls.append(ball)
        if np.linalg.norm(np.mean(balls, axis=0) * 1e-3 - FRAME_CENTROID) > 5e-5:
            raise SystemExit(f"{TAG} the frame's marker centroid does not match the JSON's")
        self._frame_where = None
        self._frame_req = None
        self._frame_auto_done = False
        self._frame_rest_z = None
        self._set_frame(FRAME_START)

    def _set_frame(self, where):
        """Place (or lift off) the frame; obj_0 follows it. Prints what the planner's
        Get + locator will write, against the scene's own envelope / mouth."""
        place = _frame_place(where)
        img = UsdGeom.Imageable(self._frame_prim)
        self._frame_where = where
        if place is None:
            img.MakeInvisible()
            self._frame_obj0 = None
            print(f"\033[1;36m{TAG} LOCALIZATION FRAME lifted off: obj_0 is no longer published\033[0m", flush=True)
            return
        O, yaw = place
        self._frame_ops[0].Set(Gf.Vec3d(*map(float, O)))
        self._frame_ops[1].Set(float(yaw))
        img.MakeVisible()
        R = _rz(yaw)
        c = O + R @ FRAME_CENTROID
        hz = math.radians(yaw) * 0.5
        self._frame_obj0 = SimpleNamespace(p=SimpleNamespace(x=float(c[0]), y=float(c[1]), z=float(c[2])),
                                           r=SimpleNamespace(w=math.cos(hz), x=0.0, y=0.0, z=math.sin(hz)))
        if where.startswith("hat"):
            tgt = c + R @ FRAME_LOCATOR["pick"]
            k = int(where[3:]) - 1
            ref, what = self._env[k]["p0"], f"envelope {k + 1}'s centre"
        else:
            tgt = c + R @ FRAME_LOCATOR["place"]
            ref, what = None, "the release point"
        err = "" if ref is None else f" ({np.linalg.norm(tgt - ref) * 1e3:.1f} mm from the scene's envelope centre)"
        print(f"\033[1;36m{TAG} LOCALIZATION FRAME on {FRAME_WHERE[where]}: O {np.round(O, 4).tolist()} m, yaw "
              f"{yaw:.0f} deg; obj_0 (markers' centroid) {np.round(c, 4).tolist()} m; Get + the locator -> {what} "
              f"{np.round(tgt, 4).tolist()} m{err}\033[0m", flush=True)

    def _pick_body_pose(self):
        return getattr(self, "_frame_obj0", None)

    def _frame_service(self, where, res):
        self._frame_req = where
        place = _frame_place(where)
        res.success = True
        res.message = (f"localization frame -> {FRAME_WHERE[where]}" +
                       ("" if place is None else
                        f": obj_0 {np.round(place[0] + _rz(place[1]) @ FRAME_CENTROID, 4).tolist()} m, yaw {place[1]:.0f} deg"))
        return res

    def _on_planner_param(self, ev):
        """FRAME_AUTO_MOVE: the operator's moves, from the planner's own parameter events --
        each route Get writes its entry of pick_place_route_letter_points / _box_points; the
        frame then goes to the first point not measured yet (hat 1, 2, 3, RED, BLUE), then off."""
        if not ev.node.endswith("/whole_body_trajectory_planner"):
            return
        for p in list(ev.changed_parameters) + list(ev.new_parameters):
            if p.name not in ROUTE_PARAMS:
                continue
            self._route_points[p.name] = list(p.value.double_array_value)
            done = []
            for name, n in zip(ROUTE_PARAMS, (N_LETTERS, 2)):
                v = self._route_points.get(name, [])
                done += [len(v) >= 4 * (i + 1) and all(math.isfinite(x) for x in v[4 * i:4 * i + 4])
                         for i in range(n)]
            nxt = next((spot for spot, d in zip(FRAME_SPOTS, done) if not d), "off")
            if nxt != getattr(self, "_frame_where", None):
                self._frame_req = nxt
                print(f"\033[1;36m{TAG} a Get is done: the frame goes onto {FRAME_WHERE[nxt]}, as the operator "
                      f"moves it" + ("" if nxt == "off" else " -- press its Get next") + "\033[0m", flush=True)

    def _frame_step(self):
        req = getattr(self, "_frame_req", None)
        if req is not None:
            self._frame_req = None
            self._set_frame(req)
        if not FRAME_AUTO_OFF or getattr(self, "_frame_auto_done", True) or getattr(self, "_body", None) is None:
            return
        z = float(self._dc.get_rigid_body_pose(self._body).p.z)
        self._frame_rest_z = z if self._frame_rest_z is None else min(self._frame_rest_z, z)
        if z > self._frame_rest_z + FRAME_LIFT_Z:
            self._frame_auto_done = True
            if self._frame_where != "off":
                print(f"\033[1;36m{TAG} the vehicle took off (body {z:.2f} m) at t = {self._t:.1f} s: the frame is "
                      f"lifted off\033[0m", flush=True)
                self._set_frame("off")

    def _build_envelope(self, stage, mat, k):
        """ENVELOPE k (07's envelope, on hat k): ONE dynamic body, an UNSCALED Xform at
        its geometric centre, x along its long axis toward the far (weighted) end. Its
        colliders: the slab and the invisible grip pad at the near edge; its mass
        properties are SET: ENV_MASS with the centre of mass ENV_COM_FROM_FAR from
        the far end -- a uniform plate plus a tip mass (the steel strip)."""
        prim_path = ENV_PRIM[k]
        yaw = math.radians(ENV_YAW_DEG)
        # the near edge (-x end) overhangs the hat's rim by ENV_OVERHANG, toward the
        # side the vehicle comes in from: along -x(env)
        u = np.array([math.cos(yaw), math.sin(yaw), 0.0])
        centre = (np.array([HAT_XY[k][0], HAT_XY[k][1], 0.0])
                  - (PLATFORM_RADIUS + ENV_OVERHANG - 0.5 * ENV_L) * u)
        centre[2] = PLATFORM_TOP + 0.5 * ENV_T + pnp.PAYLOAD_DROP_GAP
        body = UsdGeom.Xform.Define(stage, prim_path)
        xf = UsdGeom.Xformable(body)
        xf.AddTranslateOp().Set(Gf.Vec3d(*map(float, centre)))
        xf.AddRotateZOp().Set(float(ENV_YAW_DEG))
        prim = body.GetPrim()
        slab = pnp._box(stage, prim_path + "/envelope", (0.0, 0.0, 0.0), (ENV_L, ENV_W, ENV_T), ENV_RED)
        UsdPhysics.CollisionAPI.Apply(slab.GetPrim())
        pnp._bind_physics(slab.GetPrim(), mat)
        top = 0.5 * ENV_T + 0.0003
        pnp._box(stage, prim_path + "/label", (0.010, -0.008, top), (0.060, 0.040, 0.0006), ENV_LABEL)
        pnp._box(stage, prim_path + "/stamp", (0.5 * ENV_L - 0.022, 0.5 * ENV_W - 0.020, top),
                 (0.022, 0.018, 0.0006), ENV_STAMP)
        pad_on = False
        if GRIP_MM * 1e-3 > ENV_T:
            pad = pnp._box(stage, prim_path + "/grip_pad",
                           (-0.5 * ENV_L + 0.5 * GRIP_DEPTH, 0.0, 0.0),
                           (GRIP_DEPTH, ENV_W, GRIP_MM * 1e-3), (0.30, 0.85, 0.40))
            UsdPhysics.CollisionAPI.Apply(pad.GetPrim())
            pnp._bind_physics(pad.GetPrim(), mat)
            UsdGeom.Imageable(pad.GetPrim()).MakeInvisible()   # physics only: stands in for the foam
            pad_on = True
        self._grip_pad = GRIP_MM * 1e-3 if pad_on else 0.0
        UsdPhysics.RigidBodyAPI.Apply(prim)
        x_com = 0.5 * ENV_L - ENV_COM_FROM_FAR
        x_tip = 0.5 * ENV_L - TIP_MASS_FROM_FAR
        m_t = ENV_MASS * x_com / x_tip
        m_p = ENV_MASS - m_t
        Ip = np.array([(ENV_W ** 2 + ENV_T ** 2), (ENV_L ** 2 + ENV_T ** 2), (ENV_L ** 2 + ENV_W ** 2)]) * m_p / 12.0
        bar = (0.020, ENV_W, 0.003)
        It = np.array([(bar[1] ** 2 + bar[2] ** 2), (bar[0] ** 2 + bar[2] ** 2), (bar[0] ** 2 + bar[1] ** 2)]) * m_t / 12.0
        dp, dt = -x_com, x_tip - x_com       # along x, from the centre of mass
        inertia = Ip + It + np.array([0.0, m_p * dp ** 2 + m_t * dt ** 2, m_p * dp ** 2 + m_t * dt ** 2])
        mass = UsdPhysics.MassAPI.Apply(prim)
        mass.CreateMassAttr(float(ENV_MASS))
        mass.CreateCenterOfMassAttr(Gf.Vec3f(float(x_com), 0.0, 0.0))
        mass.CreateDiagonalInertiaAttr(Gf.Vec3f(*map(float, inertia)))
        mass.CreatePrincipalAxesAttr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        self._env.append({"prim": prim_path, "p0": centre, "pad": pad_on, "h": None, "state": None})
        self._env_tip_mass = m_t
        if k == 0:
            self._payload_p0 = centre       # the base scene's payload = letter 1

    # ── the base scene's setup, without its payload-specific banner ─────────────
    def __init__(self):
        am06.AmT650WholeBodyArmSim.__init__(self)
        from fsc_aerial_manipulation.utils import ROS2RigidBodyBackend

        class _PayloadBackend(ROS2RigidBodyBackend):
            def update_sim_state(this, dt):
                if not this.get_dc_interface().get_rigid_body(this.payload_path):
                    return
                super().update_sim_state(dt)

        # payload_0 = letter 1 (the base scene's payload prim), payload_1 / _2 = letters 2 / 3
        self._payload_backends = [
            _PayloadBackend(world=self.world, payload_path=ENV_PRIM[k],
                            config={"topic_prefix": ENV_TRUTH[k], "pub_state": True})
            for k in range(N_LETTERS)]
        self._payload_backend = self._payload_backends[0]
        self._probed = False
        self._gt_phase = "done"
        if not am06.HEADLESS:
            self.pg.set_viewport_camera(CAMERA_POS, CAMERA_TARGET)
        self._wp_ready = False
        if pnp.WAYPOINTS_VIZ:
            self._wp_setup_view()
        self._env_watch_t = -1e9
        self._route_points = {}

        mouth_z = TABLE_HEIGHT + MOUTH_Z
        hats = ", ".join(f"{k + 1}: ({x:.2f}, {y:.2f})" for k, (x, y) in enumerate(HAT_XY))
        print(f"\033[1;35m{TAG} MAIL-ROUTE SCENE: vehicle at ({pnp.SPAWN_XY[0]:.2f}, {pnp.SPAWN_XY[1]:.2f}) m, "
              f"yaw {pnp.SPAWN_YAW_DEG:.0f} deg, airframe {AM_ASSET}. THREE HATS in a line, {HAT_SPACING:.2f} m "
              f"apart, nearest first -- {hats} m -- pillars {PILLAR_HEIGHT:.3f} m (platform top {PLATFORM_TOP:.3f} "
              f"m, radius {PLATFORM_RADIUS * 1e3:.1f} mm). ENVELOPES ({', '.join(ENV_TRUTH)} = letters 1-3, ground "
              f"truth): {ENV_L * 1e3:.0f} x {ENV_W * 1e3:.0f} x {ENV_T * 1e3:.0f} mm, {ENV_MASS * 1e3:.0f} g, centre "
              f"of mass {ENV_COM_FROM_FAR * 1e3:.0f} mm from the far end, yaw {ENV_YAW_DEG:.0f} deg (near edge "
              f"{ENV_OVERHANG * 1e3:.0f} mm beyond each hat's rim); "
              + (f"GRIP PAD {self._grip_pad * 1e3:.0f} mm (invisible). " if self._grip_pad > 0.0 else "NO grip pad. ")
              + f"TABLE front x {TABLE_FRONT_X:.3f}; MAILBOXES RED y {RED_Y:+.3f} (letters 1, 3), BLUE y "
              f"{BLUE_Y:+.3f} (letter 2), mouth {MOUTH_W * 1e3:.0f} x {MOUTH_H * 1e3:.0f} mm at {mouth_z:.4f} m. "
              f"obj_0 = the LOCALIZATION FRAME (starts on {FRAME_WHERE[FRAME_START]}; services /mail_frame/on_hat_1 "
              f"..3, /on_red, /on_blue, /remove; follows the route's Gets: "
              f"{'yes' if FRAME_AUTO_MOVE else 'no'}).\033[0m", flush=True)

    def _setup_arm_ros2_bridge(self):
        super()._setup_arm_ros2_bridge()
        # drop_0: the RED mailbox's mouth centre (the base scene put it on its place pillar's top)
        pose = self._fixed_bodies[1][2]
        pose.pose.position.x = float(TABLE_FRONT_X)
        pose.pose.position.y = float(RED_Y)
        pose.pose.position.z = float(TABLE_HEIGHT + MOUTH_Z)
        if FRAME_AUTO_MOVE:
            from rcl_interfaces.msg import ParameterEvent
            from rclpy.qos import qos_profile_parameter_events
            self._frame_param_sub = self._arm_node.create_subscription(
                ParameterEvent, "/parameter_events", self._on_planner_param, qos_profile_parameter_events)
        from std_srvs.srv import Trigger
        spots = [(f"on_hat_{k + 1}", f"hat{k + 1}") for k in range(N_LETTERS)] + \
                [("on_red", "red"), ("on_blue", "blue"), ("remove", "off")]
        self._frame_srvs = [
            self._arm_node.create_service(Trigger, f"/mail_frame/{name}",
                                          lambda req, res, w=where: self._frame_service(w, res))
            for name, where in spots]

    # ── where is each envelope? (printed on every change: the run's verdict) ──
    def _env_pose(self, k):
        e = self._env[k]
        if not e["h"]:
            e["h"] = self._dc.get_rigid_body(e["prim"])
            if not e["h"]:
                return None
        return self._dc.get_rigid_body_pose(e["h"])

    def _envelope_state(self, k):
        P = self._env_pose(k)
        if P is None:
            return None, None
        p = np.array([P.p.x, P.p.y, P.p.z])
        R = am06.C.quat_to_rot(P.r.w, P.r.x, P.r.y, P.r.z)
        c = p + R @ np.array([0.5 * ENV_L - ENV_COM_FROM_FAR, 0.0, 0.0])   # its centre of mass
        x0, x1 = TABLE_FRONT_X + WALL, TABLE_FRONT_X + BOX_D - WALL
        mouth_lo = TABLE_HEIGHT + MOUTH_Z - 0.5 * MOUTH_H
        for name, cy in (("IN THE RED MAILBOX", RED_Y), ("IN THE BLUE MAILBOX", BLUE_Y)):
            if x0 < c[0] < x1 and abs(c[1] - cy) < 0.5 * BOX_W - WALL and c[2] < mouth_lo:
                return name, p
        for j, (hx, hy) in enumerate(HAT_XY):
            if abs(c[2] - (PLATFORM_TOP + 0.5 * ENV_T)) < 0.01 and np.hypot(c[0] - hx, c[1] - hy) < PLATFORM_RADIUS:
                return f"on hat {j + 1}", p
        if c[2] < 0.05:
            return "ON THE FLOOR", p
        if TABLE_FRONT_X <= c[0] <= TABLE_FRONT_X + TABLE_DEPTH_X and abs(c[2] - TABLE_HEIGHT) < 0.03:
            return "ON THE TABLE", p
        return "carried / in the air", p

    def _release_grip_pad(self):
        """07's rule per envelope: the invisible grip pad stands in for the real jaws'
        foam while an envelope is HELD; once the jaws open with it at a mailbox its
        collider goes off -- the envelope falls in as a 4 mm plate."""
        opening = self._grip_angle() > math.radians(-15.0)
        for k, e in enumerate(self._env):
            if not e["pad"]:
                continue
            P = self._env_pose(k)
            if P is None:
                continue
            near_box = P.p.x > TABLE_FRONT_X - 0.10 and P.p.z > TABLE_HEIGHT
            if near_box and opening:
                prim = self.stage.GetPrimAtPath(e["prim"] + "/grip_pad")
                if prim:
                    UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr().Set(False)
                e["pad"] = False
                print(f"\033[1;36m{TAG} jaws open at the mailbox (gripper {math.degrees(self._grip_angle()):.0f} deg) "
                      f"at t = {self._t:.1f} s: envelope {k + 1}'s grip pad collider is OFF -- a 4 mm plate "
                      f"again\033[0m", flush=True)

    def _control_step_inner(self, dt):
        try:
            self._frame_step()
        except Exception as exc:              # a stand-in must never stop the plant
            self._frame_auto_done = True
            print(f"{TAG} localization frame step failed: {exc}", flush=True)
        super()._control_step_inner(dt)
        try:
            self._release_grip_pad()
        except Exception as exc:              # a stand-in must never stop the plant
            for e in self._env:
                e["pad"] = False
            print(f"{TAG} grip pad release failed: {exc}", flush=True)
        if self._t - self._env_watch_t < WATCH_S:
            return
        self._env_watch_t = self._t
        for k, e in enumerate(self._env):
            try:
                state, p = self._envelope_state(k)
            except Exception:                 # a monitor must never stop the plant
                continue
            if state is None or state == e["state"]:
                continue
            e["state"] = state
            tilt = 0.0
            P = self._env_pose(k)
            if P is not None:
                R = am06.C.quat_to_rot(P.r.w, P.r.x, P.r.y, P.r.z)
                tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(R[2, 2])))))
            print(f"\033[1;36m{TAG} ENVELOPE {k + 1} {state} at t = {self._t:.1f} s: centre "
                  f"{np.round(p, 3).tolist()} m, tilt {tilt:.0f} deg\033[0m", flush=True)


def main():
    AmT650MailRoute().run()


if __name__ == "__main__":
    main()
