"""Wrap an AprilTag-aligned splat as arena_metric.usda and measure its floor.

The gate scene needed make_metric_usd.py to bake scale, gravity and origin into the
wrapper. This one does not: the capture had three AprilTags with tape-measured
centres (markers.yaml), so spirulae-splat's export_ply_3dgs.py --dataset_dir already
wrote splat.ply in the tag frame (metres, z up, tag 0 at the origin). The only
transform left is the USDZ exporter's own axis swap, stamped on the volume prim:

    S = ((-1,0,0), (0,0,-1), (0,-1,0))      # (x,y,z) -> (-x,-z,-y), S @ S = I

so the wrapper applies S once more to cancel it, and world == PLY coordinates.

Floor height: the tags sit on a board on the mats, so the floor is a little below
z = 0. Measured as the densest height bin of opaque gaussians in an annulus around
the origin (the inner radius excludes the board itself).

Outputs (in the dataset dir):
    arena_metric.usda  - reference this in Isaac Sim (keep arena.usdz beside it)
    arena_metric.json  - floor height + scene extent

Run:  ~/miniconda3/envs/3dgrut/bin/python make_arena_usd.py
"""
import glob
import json
import os

import numpy as np
from plyfile import PlyData

DATA = os.path.expanduser(os.environ.get("ARENA_DATASET_DIR", "~/Downloads/IMG_7141"))
OUT_USD = os.path.join(DATA, "arena_metric.usda")
OUT_JSON = os.path.join(DATA, "arena_metric.json")

S = np.array([[-1, 0, 0], [0, 0, -1], [0, -1, 0]], float)

FLOOR_R_MIN, FLOOR_R_MAX = 1.5, 4.0   # m, annulus around the tag origin
FLOOR_Z_WINDOW = 0.25                 # m, search band around z = 0

_plys = sorted(glob.glob(os.path.join(DATA, "outputs", "*", "*", "splat.ply")))
if not _plys:
    raise FileNotFoundError(
        f"No exported splat.ply under {DATA}/outputs/*/*/. Run spirulae-splat's "
        "scripts/export_ply_3dgs.py <run dir> --dataset_dir <dataset> first."
    )
SPLAT_PLY = os.environ.get("ARENA_SPLAT_PLY", _plys[-1])  # newest training run
print(f"splat: {SPLAT_PLY}")

g = PlyData.read(SPLAT_PLY)["vertex"]
P = np.stack([g["x"], g["y"], g["z"]], 1)
alpha = 1 / (1 + np.exp(-np.asarray(g["opacity"])))
Po = P[alpha > 0.5]

r = np.hypot(Po[:, 0], Po[:, 1])
band = (r > FLOOR_R_MIN) & (r < FLOOR_R_MAX) & (np.abs(Po[:, 2]) < FLOOR_Z_WINDOW)
hist, edges = np.histogram(Po[band][:, 2], bins=int(2 * FLOOR_Z_WINDOW / 0.005))
k = np.argmax(hist)
z_floor = 0.5 * (edges[k] + edges[k + 1])

lo, hi = np.percentile(Po, 1, axis=0), np.percentile(Po, 99, axis=0)
print(f"gaussians        : {len(P):,} ({len(Po):,} with opacity > 0.5)")
print(f"floor z          : {z_floor:.4f} m  ({hist[k]} gaussians in the peak 5 mm bin)")
print(f"extent (1-99 %)  : x {lo[0]:.2f}..{hi[0]:.2f}  y {lo[1]:.2f}..{hi[1]:.2f}  "
      f"z {lo[2]:.2f}..{hi[2]:.2f} m")

from pxr import Gf, Usd, UsdGeom

stage = Usd.Stage.CreateNew(OUT_USD) if not os.path.exists(OUT_USD) else Usd.Stage.Open(OUT_USD)
stage.GetRootLayer().Clear()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
world = UsdGeom.Xform.Define(stage, "/World")
stage.SetDefaultPrim(world.GetPrim())
arena = UsdGeom.Xform.Define(stage, "/World/arena")
arena.GetPrim().GetReferences().AddReference("./arena.usdz")
M = np.eye(4)
M[:3, :3] = S  # cancels the exporter's swap; S is symmetric, so row/column form agree
xf = UsdGeom.Xformable(arena.GetPrim())
xf.ClearXformOpOrder()
xf.AddTransformOp().Set(Gf.Matrix4d(*M.T.flatten()))
stage.GetRootLayer().Save()
print(f"\nwrote {OUT_USD}")

with open(OUT_JSON, "w") as fp:
    json.dump({
        "splat_ply": SPLAT_PLY,
        "frame": "AprilTag board: tag0 origin, +x to tag1, +y to tag2, z up, metres",
        "floor_z_m": float(z_floor),
        "extent_p1_m": lo.tolist(),
        "extent_p99_m": hi.tolist(),
        "num_gaussians": int(len(P)),
        "M_world_from_usdz_volume": M.tolist(),
    }, fp, indent=2)
print(f"wrote {OUT_JSON}")
