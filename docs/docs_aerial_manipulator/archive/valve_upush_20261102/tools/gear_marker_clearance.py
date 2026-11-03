#!/usr/bin/env python3
"""Landing gear vs the valve, at the U-push grasp and through the turn (2026-11-03, user question:
"is the T650's clearance ~6-7 cm off the valve in altitude?").

The gear = the points of AM_T650.usda's body mesh more than 0.20 m below the body origin (the
skids and the lower struts; the mesh is the one make_t650_gear_asset.py shortened). They are placed
with the run's TRUE body pose (/uav_0/state/pose); the valve's five mocap marker balls (r 15 mm,
fsc:mocap_markers_m, rotor frame) with obj_0's pose (the markers' centroid, the rotor's axes), and
the beam/fin as boxes. Prints, for the hold after Ready and every 2 s of the turn: the gap from each
marker ball to the nearest gear point, the gear's lowest point above the plate, and the fin's top.
Also the same with the HARDWARE gear (0.248 m measured vs the asset's 0.2425: 5.5 mm deeper).
Run: /usr/bin/python3 -I gear_marker_clearance.py <run.npz>   (usd-core from the user site)"""
import sys, os, numpy as np
from pxr import Usd, UsdGeom
HERE = os.path.dirname(os.path.abspath(__file__))
ASSETS = os.path.join(HERE, *[".."] * 5, "extensions", "fsc_aerial_manipulation", "fsc_aerial_manipulation",
                      "rotorcraft", "assets")
npz = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "..", "sim", "runs", "n1_generated.npz")

stage = Usd.Stage.Open(os.path.join(ASSETS, "AM_T650.usda"))
body = stage.GetPrimAtPath("/gripper_bat/body")
mesh = [p for p in Usd.PrimRange(body) if p.IsA(UsdGeom.Mesh) and p.GetName() == "body"][0]
xc = UsdGeom.XformCache()
M, _ = xc.ComputeRelativeTransform(mesh, body)
pts = np.array(UsdGeom.Mesh(mesh).GetPointsAttr().Get(), float)
Mn = np.array(M).T                                   # Gf row-vector convention -> column
P = (Mn[:3, :3] @ pts.T).T + Mn[:3, 3]
gear = P[P[:, 2] < -0.20]
print(f"gear points {len(gear)}; body frame: z {gear[:,2].min():.4f}..{gear[:,2].max():.4f}, "
      f"x {gear[:,0].min():.3f}..{gear[:,0].max():.3f}, y {gear[:,1].min():.3f}..{gear[:,1].max():.3f}")
sk = gear[gear[:, 2] < gear[:, 2].min() + 0.02]
for side in (1, -1):
    s = sk[np.sign(sk[:, 1]) == side]
    print(f"  skid y{'+' if side > 0 else '-'}: x {s[:,0].min():.3f}..{s[:,0].max():.3f}, "
          f"y {s[:,1].min():.3f}..{s[:,1].max():.3f}")

vstage = Usd.Stage.Open(os.path.join(ASSETS, "Valve_Assembly_v2.usda"))
vp = vstage.GetDefaultPrim()
mk = np.array(vp.GetAttribute("fsc:mocap_markers_m").Get(), float)
rad = vp.GetAttribute("fsc:mocap_marker_radius_m").Get()
piv = np.array([vp.GetAttribute(f"fsc:mocap_pivot_{a}_m").Get() for a in "xyz"])


def R(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


z = np.load(npz, allow_pickle=True)
bt = z["base_truth"]; ob = z["box"]
mt = dict(zip(z["marks_name"], z["marks_t"]))
t_push = mt["push:start"]
times = [("hold, before the turn", t_push - 0.3)] + \
        [(f"turn +{k:4.1f} s", t_push + k) for k in np.arange(2.0, 20.0, 2.0)]


def at(t, a):
    return a[min(np.searchsorted(a[:, 0], t), len(a) - 1)]


rows = []
for name, t in times:
    b = at(t, bt); o = at(t, ob)
    Rb, pb = R(b[4:8]), b[1:4]
    Ro, po = R(o[4:8]), o[1:4]
    axis = po - Ro @ piv                              # the rotor frame's origin (axis, plate bottom)
    G = (Rb @ gear.T).T + pb                          # gear, world
    Gr = (Ro.T @ (G - axis).T).T                      # gear, rotor frame
    gaps = [np.min(np.linalg.norm(Gr - m, axis=1)) - rad for m in mk]
    near = int(np.argmin(gaps))
    rows.append(gaps)
    # the beam: 225 x 50 mm, top 54 mm above the plate, rotor x -0.025..0.200 (axis -> past the fin), |y| < 0.025
    over = (Gr[:, 0] > -0.025) & (Gr[:, 0] < 0.200) & (np.abs(Gr[:, 1]) < 0.025)
    dx = np.maximum(np.maximum(-0.025 - Gr[:, 0], Gr[:, 0] - 0.200), 0.0)
    dy = np.maximum(np.abs(Gr[:, 1]) - 0.025, 0.0)
    beam = (f"over the beam: lowest {(Gr[over, 2].min() - 0.054)*1e3:5.1f} mm above its top" if over.any() else
            f"not over the beam (nearest {np.min(np.hypot(dx, dy))*1e3:4.0f} mm away horizontally)")
    print(f"{name:24s} {beam} |")
    print(f"{name:24s} gear lowest {Gr[:,2].min()*1e3:6.1f} mm above the plate | "
          f"marker gaps (mm) {' '.join(f'{g*1e3:5.0f}' for g in gaps)} | nearest #{near+1}")
rows = np.array(rows)
print(f"marker tops above the plate (mm): {' '.join(f'{(m[2]+rad)*1e3:5.0f}' for m in mk)}; fin top 204, beam top 54")
print(f"smallest gap over the hold + turn: {rows.min()*1e3:.0f} mm (marker #{int(np.argmin(rows.min(0)))+1}); "
      f"the hardware gear is 5.5 mm deeper -- at most that much less where the nearest point is below the ball")
