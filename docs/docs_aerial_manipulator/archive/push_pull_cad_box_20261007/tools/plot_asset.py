#!/usr/bin/env python3
"""Box_Push.usda as the scene builds it: the CAD visual, the collider blocks, the
sim grip shim (20 mm) and the claw point. Two passes (pxr lives in the user
site with numpy 2, the system matplotlib needs numpy 1):
    /usr/bin/python3 plot_asset.py --dump               # reads the usda -> ../figures/box_push_asset.npz
    PYTHONNOUSERSITE=1 /usr/bin/python3 plot_asset.py   # draws it"""
import os
import sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
NPZ = os.path.join(HERE, "..", "figures", "box_push_asset.npz")
if "--dump" in sys.argv:
    from pxr import Usd, UsdGeom, UsdPhysics
    A = os.path.abspath(os.path.join(HERE, *[".."] * 5, "extensions", "fsc_aerial_manipulation",
                                     "fsc_aerial_manipulation", "rotorcraft", "assets", "Box_Push.usda"))
    st = Usd.Stage.Open(A)
    root = st.GetDefaultPrim()
    vis = UsdGeom.Mesh(st.GetPrimAtPath("/Box_Push/Visual"))
    cp = [p for p in Usd.PrimRange(root) if p.HasAPI(UsdPhysics.CollisionAPI)]
    np.savez(NPZ, P=np.array(vis.GetPointsAttr().Get()), F=np.array(vis.GetFaceVertexIndicesAttr().Get()),
             names=np.array([p.GetName() for p in cp]),
             col=np.array([np.array(UsdGeom.Mesh(p).GetPointsAttr().Get()) for p in cp]),
             grip=np.array([bool(p.GetAttribute("fsc:grip").Get()) for p in cp]),
             **{k: root.GetAttribute(f"fsc:{k}").Get() for k in (
                 "fin_top_z_m", "fin_exposed_bottom_z_m", "fin_length_m", "box_half_height_m")})
    print("wrote", os.path.abspath(NPZ)); sys.exit(0)
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

z = np.load(NPZ)
cad = {k: float(z[k]) for k in ("fin_top_z_m", "fin_exposed_bottom_z_m", "fin_length_m", "box_half_height_m")}
P = z["P"] * 1e3
F = z["F"].reshape(-1, 3)
cols = [(n, c * 1e3, g) for n, c, g in zip(z["names"], z["col"], z["grip"])]
shim = (20.0, cad["fin_length_m"] * 1e3, cad["fin_exposed_bottom_z_m"] * 1e3, cad["fin_top_z_m"] * 1e3)
grasp_z = (cad["fin_top_z_m"] - 0.025) * 1e3
hb = cad["box_half_height_m"] * 1e3


def boxfaces(lo, hi):
    c = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
    q = [(0, 1, 3, 2), (4, 5, 7, 6), (0, 1, 5, 4), (2, 3, 7, 6), (0, 2, 6, 4), (1, 3, 7, 5)]
    return [c[list(f)] for f in q]


fig = plt.figure(figsize=(15, 6.2))
for k, (el, az, title) in enumerate([(22, -55, "the CAD box (mesh) as the scene loads it"),
                                     (0, 0, "side view, looking across the table (y = along the push)"),
                                     (0, -90, "side view, looking along the push (x = across the jaws)")]):
    ax = fig.add_subplot(1, 3, k + 1, projection="3d")
    ax.add_collection3d(Poly3DCollection(P[F], facecolor="#d9a441", edgecolor="none", alpha=0.55 if k else 0.9))
    if k:
        for name, c, grip in cols:
            for f in boxfaces(c.min(0), c.max(0)):
                ax.add_collection3d(Poly3DCollection([f], facecolor=(0, 0, 0, 0),
                                                     edgecolor="#2a78d6" if grip else "#555", linewidths=0.7))
        lo = (-shim[0] / 2, -shim[1] / 2, shim[2]); hi = (shim[0] / 2, shim[1] / 2, shim[3])
        for f in boxfaces(lo, hi):
            ax.add_collection3d(Poly3DCollection([f], facecolor="#4fb37a", alpha=0.18, edgecolor="#2e8b57",
                                                 linewidths=0.8))
        ax.scatter([0], [0], [grasp_z], color="#e66767", s=40, depthshade=False)
    ax.set_xlim(-130, 130); ax.set_ylim(-130, 130); ax.set_zlim(-60, 220)
    ax.set_box_aspect((1, 1, 1.08))
    ax.view_init(elev=el, azim=az)
    ax.set_xlabel("x [mm]"); ax.set_ylabel("y [mm]"); ax.set_zlabel("z from box centre [mm]")
    ax.set_title(title, fontsize=9)
fig.suptitle(f"Box_Push.usda: grey = collider blocks, blue = fin + rib (grip material), green = sim grip shim "
             f"(20 mm, invisible), red = claw point ({grasp_z + hb:.0f} mm above the table)", fontsize=9)
fig.tight_layout()
out = os.path.join(HERE, "..", "figures", "box_push_asset.png")
fig.savefig(out, dpi=110)
print("wrote", os.path.abspath(out))
