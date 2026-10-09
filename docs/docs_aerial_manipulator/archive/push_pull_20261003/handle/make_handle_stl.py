#!/usr/bin/env python3
"""Write push_handle.stl (and a preview PNG) for the push-and-pull post handle.

    PYTHONNOUSERSITE=1 /usr/bin/python3 make_handle_stl.py

Same dimensions as push_handle.scad (the parametric source, which also carries
the M4 holes, centring notches and the grasp-height groove). This STL is the
solid body only -- base plate, post and four root ribs -- as separate closed
solids that overlap; every slicer unions them. Units: mm, origin = the box's
top centre, +z up, x ACROSS the table (the jaws' closing direction), y ALONG
the push.
"""
import os
import struct

import numpy as np

BOX_H, POST_TOP, GRASP_DOWN = 95.0, 298.0, 25.0
POST_T, POST_D = 20.0, 30.0           # across the jaws / along the push
PLATE_X, PLATE_Y, PLATE_H = 70.0, 90.0, 4.0
RIB_H, RIB_L, RIB_T = 35.0, 25.0, 4.0
POST_H = POST_TOP - BOX_H             # 203 mm above the box top, plate included
HERE = os.path.dirname(os.path.abspath(__file__))


def box(x0, x1, y0, y1, z0, z1):
    v = np.array([[x, y, z] for z in (z0, z1) for y in (y0, y1) for x in (x0, x1)], float)
    # vertex index = x + 2 y + 4 z ; faces outward
    q = [(0, 2, 3, 1), (4, 5, 7, 6), (0, 1, 5, 4), (2, 6, 7, 3), (0, 4, 6, 2), (1, 3, 7, 5)]
    return [(v[a], v[b], v[c]) for a, b, c, d in q for (a, b, c) in ((a, b, c), (a, c, d))]


def wedge(origin, along, ht, length, thick):
    """Right-angle rib: legs `length` along the unit vector `along` and `ht` up,
    `thick` wide across it, right angle at `origin` (on the post face, plate top)."""
    a = np.asarray(along, float)
    side = np.cross([0, 0, 1.0], a) * (thick / 2.0)
    o = np.asarray(origin, float)
    p = [o - side, o + a * length - side, o + [0, 0, ht] - side,
         o + side, o + a * length + side, o + [0, 0, ht] + side]
    tris = [(0, 2, 1), (3, 4, 5),                       # the two triangular sides
            (0, 1, 4), (0, 4, 3),                       # bottom
            (0, 3, 5), (0, 5, 2),                       # back (against the post)
            (1, 2, 5), (1, 5, 4)]                       # hypotenuse
    out = []
    for i, j, k in tris:
        t = (p[i], p[j], p[k])
        n = np.cross(t[1] - t[0], t[2] - t[0])
        c = (t[0] + t[1] + t[2]) / 3.0
        centroid = np.mean(p, axis=0)
        if np.dot(n, c - centroid) < 0:                 # make every normal point outward
            t = (t[0], t[2], t[1])
        out.append(t)
    return out


tris = []
tris += box(-PLATE_X / 2, PLATE_X / 2, -PLATE_Y / 2, PLATE_Y / 2, 0.0, PLATE_H)
tris += box(-POST_T / 2, POST_T / 2, -POST_D / 2, POST_D / 2, 0.0, POST_H)
for s in (-1.0, 1.0):
    tris += wedge([s * POST_T / 2, 0, PLATE_H], [s, 0, 0], RIB_H, RIB_L, RIB_T)
    tris += wedge([0, s * POST_D / 2, PLATE_H], [0, s, 0], RIB_H, RIB_L, RIB_T)

out = os.path.join(HERE, "push_handle.stl")
with open(out, "wb") as f:
    f.write(b"push_handle: post on the box top centre, mm".ljust(80, b" "))
    f.write(struct.pack("<I", len(tris)))
    for a, b, c in tris:
        n = np.cross(b - a, c - a)
        n = n / (np.linalg.norm(n) or 1.0)
        f.write(struct.pack("<12fH", *n, *a, *b, *c, 0))
vol = 0.0
for a, b, c in tris:
    vol += np.dot(a, np.cross(b, c)) / 6.0
print(f"wrote {out}: {len(tris)} triangles; height {POST_H:.0f} mm; grasp {POST_H - GRASP_DOWN:.0f} mm above "
      f"the box top = {POST_TOP - GRASP_DOWN:.0f} mm above the table; summed solid volume {vol / 1e3:.0f} cm^3 "
      f"(overlaps counted twice)")

# preview
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

fig = plt.figure(figsize=(9, 6))
for k, (el, az, title) in enumerate([(18, -60, "handle on the box top"), (0, 0, "side view, looking across the table")]):
    ax = fig.add_subplot(1, 2, k + 1, projection="3d")
    ax.add_collection3d(Poly3DCollection([list(t) for t in tris], facecolor="#c9c4b8", edgecolor="#555", linewidths=0.3))
    # the box top outline (160 across x 240 along y) and the grasp line
    bx, by = 80, 120
    ax.plot([-bx, bx, bx, -bx, -bx], [-by, -by, by, by, -by], [0, 0, 0, 0, 0], color="#2a78d6", lw=1)
    ax.plot([-POST_T / 2 - 15, POST_T / 2 + 15], [0, 0], [POST_H - GRASP_DOWN] * 2, color="#eb6834", lw=2)
    ax.set_xlim(-130, 130); ax.set_ylim(-130, 130); ax.set_zlim(0, 260)
    ax.set_box_aspect((1, 1, 1))
    ax.view_init(elev=el, azim=az)
    ax.set_xlabel("x across [mm]"); ax.set_ylabel("y along push [mm]"); ax.set_zlabel("z [mm]")
    ax.set_title(title, fontsize=10)
fig.suptitle(f"Post {POST_T:.0f} x {POST_D:.0f} mm, {POST_H:.0f} mm above the box top; orange = grasp "
             f"({POST_TOP - GRASP_DOWN:.0f} mm above the table); blue = box top 160 x 240 mm", fontsize=9)
fig.tight_layout()
png = os.path.join(HERE, "push_handle_preview.png")
fig.savefig(png, dpi=110)
print("wrote", png)
