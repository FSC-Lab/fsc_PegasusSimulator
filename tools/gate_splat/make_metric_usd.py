"""Bake METRIC scale + gravity alignment + a canonical origin into gate_metric.usda.

Transform chain (splat/PLY frame -> Isaac world):
    X_world = Rz(yaw) @ A(gravity) @ [s * X_ply] + t

  * s     : metric scale from the measured 1.50 m inner opening WIDTH
            (measure_gate_opening.py; cloud + triangulation agree on 1.845 units).
            Camera-height check: s=0.813 puts the phone at 1.60 m above the floor
            (a standing person); the height=1.5m hypothesis would put it at 1.95 m.
  * A     : camera-up gravity alignment (as in make_aligned_usd.py)
  * yaw   : rotates the gate plane normal onto +X, so flying "through the gate"
            is motion along world X
  * t     : gate opening centre at (x=0, y=0); floor at z=0

Outputs:
    gate_metric.usda   - reference this in Isaac Sim (keep gate.usdz beside it)
    gate_metric.json   - the numbers, incl. the world->splat camera transform

Run:  ~/miniconda3/envs/py312/bin/python make_metric_usd.py
"""
import json
import os

import numpy as np

DATA = os.path.expanduser(os.environ.get("GATE_DATASET_DIR", "~/Downloads/IMG_4701"))
OUT_USD = os.path.join(DATA, "gate_metric.usda")
OUT_JSON = os.path.join(DATA, "gate_metric.json")

KNOWN_WIDTH_M = 1.50
MEASURED_WIDTH_UNITS = 1.845  # banner-cutout width: triangulation 1.837, cloud 1.855

# exporter's Z-up rotation stamped on the Volume prim inside gate.usdz
R_EXP = np.array([[-1, 0, 0], [0, 0, -1], [0, -1, 0]], float)

with open(os.path.join(DATA, "transforms.json")) as fp:
    frames = sorted(json.load(fp)["frames"], key=lambda f: f["file_path"])
with open(os.path.join(DATA, "gate_scale.json")) as fp:
    gs = json.load(fp)

# ---- gravity alignment from camera up-vectors (phone held upright) ----
ups = np.array([(R_EXP @ np.array(f["transform_matrix"], float)[:3, :3])[:, 1] for f in frames])
up = ups.mean(0)
up /= np.linalg.norm(up)
z = np.array([0.0, 0.0, 1.0])
v, c = np.cross(up, z), float(up @ z)
s_ = np.linalg.norm(v)
vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
A = np.eye(3) + vx + vx @ vx * ((1 - c) / s_**2)

# ---- scale ----
s = KNOWN_WIDTH_M / MEASURED_WIDTH_UNITS

# ---- gate geometry in the aligned (pre-yaw) frame ----
corners_ply = np.array([gs["corners_splat"][n] for n in ["TL", "TR", "BR", "BL"]])
corners_al = (A @ R_EXP @ corners_ply.T).T * s
centre_al = corners_al.mean(0)

# gate plane normal (horizontal component), yawed onto +X
TL, TR, BR, BL = corners_al
n = np.cross(TR - TL, BL - TL)
n /= np.linalg.norm(n)
# point the normal toward the side the cameras are on (drone spawns facing the gate)
cams_al = np.array([(A @ (R_EXP @ np.array(f["transform_matrix"], float)[:3, 3])) * s for f in frames])
if (cams_al.mean(0) - centre_al) @ n < 0:
    n = -n
yaw = -np.arctan2(n[1], n[0])
Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])

# ---- floor level (aligned frame, metric) ----
from plyfile import PlyData

import glob

_plys = sorted(glob.glob(os.path.join(DATA, "outputs", "*", "*", "splat.ply")))
if not _plys:
    raise FileNotFoundError(
        f"No exported splat.ply under {DATA}/outputs/*/*/.\n"
        "Run spirulae-splat's scripts/export_ply_3dgs.py on the trained model first, "
        "or set GATE_DATASET_DIR / GATE_SPLAT_PLY."
    )
SPLAT_PLY = os.environ.get("GATE_SPLAT_PLY", _plys[-1])  # newest training run
print(f"floor level measured from: {SPLAT_PLY}")
g = PlyData.read(SPLAT_PLY)["vertex"]
P = np.stack([g["x"], g["y"], g["z"]], 1)
alpha = 1 / (1 + np.exp(-np.asarray(g["opacity"])))
P = P[alpha > 0.5]
Pa = (A @ R_EXP @ P.T).T * s
near = (np.abs(Pa[:, 0] - centre_al[0]) < 3.5) & (np.abs(Pa[:, 1] - centre_al[1]) < 3.5) \
       & (Pa[:, 2] < corners_al[:, 2].min())
hist, edges = np.histogram(Pa[near][:, 2], bins=200)
z_floor = 0.5 * (edges[np.argmax(hist)] + edges[np.argmax(hist) + 1])

# ---- final rigid offset: gate centre -> (0,0), floor -> z=0 ----
centre_yawed = Rz @ centre_al
floor_yawed_z = z_floor  # Rz does not change z
t = np.array([-centre_yawed[0], -centre_yawed[1], -floor_yawed_z])

# full 4x4 applied on TOP of the USDZ's internal R_EXP:  M = T Rz A S  (S=s*I)
M = np.eye(4)
M[:3, :3] = Rz @ A * s
M[:3, 3] = t

corners_w = (Rz @ corners_al.T).T + t
cams_w = (Rz @ cams_al.T).T + t
print(f"scale s          : {s:.6f} m/unit")
print(f"yaw              : {np.degrees(yaw):.2f} deg (gate normal -> +X)")
print(f"floor z (pre-t)  : {z_floor:.4f} m")
print(f"opening corners_w:")
for nme, cw in zip(["TL", "TR", "BR", "BL"], corners_w):
    print(f"  {nme}: {np.round(cw, 3).tolist()}")
print(f"opening centre   : {np.round(corners_w.mean(0), 3).tolist()}  "
      f"(width {np.linalg.norm(corners_w[1]-corners_w[0]):.3f} m, "
      f"height {np.linalg.norm(corners_w[0]-corners_w[3]):.3f} m)")
print(f"camera height    : mean {cams_w[:, 2].mean():.3f} m")

# ---- write USD ----
from pxr import Usd, UsdGeom, Gf

stage = Usd.Stage.CreateNew(OUT_USD) if not os.path.exists(OUT_USD) else Usd.Stage.Open(OUT_USD)
stage.GetRootLayer().Clear()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
world = UsdGeom.Xform.Define(stage, "/World")
stage.SetDefaultPrim(world.GetPrim())
gate = UsdGeom.Xform.Define(stage, "/World/gate")
gate.GetPrim().GetReferences().AddReference("./gate.usdz")
xf = UsdGeom.Xformable(gate.GetPrim())
xf.ClearXformOpOrder()
xf.AddTransformOp().Set(Gf.Matrix4d(*M.T.flatten()))  # USD row-vector convention
stage.GetRootLayer().Save()
print(f"\nwrote {OUT_USD}")

with open(OUT_JSON, "w") as fp:
    json.dump({
        "scale_m_per_unit": s,
        "known_width_m": KNOWN_WIDTH_M,
        "measured_width_units": MEASURED_WIDTH_UNITS,
        "yaw_deg": float(np.degrees(yaw)),
        "M_world_from_ply": M.tolist(),
        "opening_corners_world": {k: v.tolist() for k, v in zip(["TL", "TR", "BR", "BL"], corners_w)},
        "opening_centre_world": corners_w.mean(0).tolist(),
        "opening_width_m": float(np.linalg.norm(corners_w[1] - corners_w[0])),
        "opening_height_m": float(np.linalg.norm(corners_w[0] - corners_w[3])),
        "sill_height_m": float(corners_w[:, 2].min()),
        "camera_mean_height_m": float(cams_w[:, 2].mean()),
    }, fp, indent=2)
print(f"wrote {OUT_JSON}")
