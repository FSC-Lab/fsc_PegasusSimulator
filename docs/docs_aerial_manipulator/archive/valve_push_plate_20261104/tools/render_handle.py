#!/usr/bin/env python3
"""Pictures of the handle (the v20 fin) with the two holes for the push plate, for the design page.

    PYTHONNOUSERSITE=1 PYTHONPATH=~/sources/pymanifold /usr/bin/python3 render_handle.py

Writes ../img/h1_handle.png (the fin on its own, dimensioned), h2_on_valve.png (the handle end of
Valve_Assembly_v20.stl with the holes marked) and h3_fitted.png (the plate, screws and nuts fitted).
Valve frame, mm: x along the beam from the shaft, y along the push (the plate on the fin's -y edge),
z up from the table. The fin: x 157.5..162.5, y -19.4..20.6, z 54 (its foot, in the socket) .. 204.
"""
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt                                        # noqa: E402
from mpl_toolkits.mplot3d import proj3d                                # noqa: E402
from mpl_toolkits.mplot3d.art3d import Poly3DCollection                # noqa: E402
import manifold3d as mf                                                # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
ARCH = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(ARCH, "valve_handle_joint_20261030", "tools"))
import check_on_model as cm                                            # noqa: E402  (reads the v20 STL into shells)

IMG = os.path.join(os.path.dirname(HERE), "img")
BG, INK, INK2, INK3 = "#111214", "#ecebe6", "#b9b8b0", "#85847d"
COL = {"rotor": "#cfc8b8", "fin": "#cfc8b8", "hole": "#1a1b1e", "new": "#fab219", "plate": "#a08cf0",
       "steel": "#c9cdd3", "nut": "#9aa0a8", "marker": "#d9dce0", "mount": "#3a3d42"}
LIGHT = np.array([0.45, -0.55, 0.70]) / np.linalg.norm([0.45, -0.55, 0.70])

FIN = dict(x=(157.5, 162.5), y=(-19.4, 20.6), z=(54.0, 204.0))
FIN_YC = 0.6
FOOT = [(FIN_YC + d, 59.0) for d in (-14.0, -6.0, 6.0, 14.0)]
EDGE_IN, HOLE_D = 7.5, 3.2
NEW = [(FIN["y"][0] + EDGE_IN, FIN["z"][1] - d) for d in (15.0, 60.0)]          # (y, z)
PLATE = dict(x=(100.0, 220.0), y=(-24.4, -19.4), z=(125.0, 204.0))
SLOT, CHEEK, CHEEK_L = 5.3, 3.0, 15.0


def box(x, y, z):
    return mf.Manifold.cube((x[1] - x[0], y[1] - y[0], z[1] - z[0])).translate((x[0], y[0], z[0]))


def xcyl(y, z, r, x0, x1, n=40):
    """A cylinder along x through (y, z)."""
    c = mf.Manifold.cylinder(x1 - x0, r, r, n)                       # along +z from 0
    return c.rotate((0, 90, 0)).translate((x0, y, z))                # +z -> +x


def xhex(y, z, af, x0, x1):
    r = af / np.sqrt(3.0)
    pts = [(r * np.cos(a), r * np.sin(a)) for a in np.radians(np.arange(0, 360, 60) + 30)]
    h = mf.CrossSection([pts]).extrude(x1 - x0)
    return h.rotate((0, 90, 0)).translate((x0, y, z))


def tris(m, edge=3.0):
    m = m.refine_to_length(edge)
    me = m.to_mesh()
    V = np.asarray(me.vert_properties)[:, :3]
    return V[np.asarray(me.tri_verts)]


def ring(x, y, z, r0, r1, n=48):
    """A flat annulus in the plane x = const (a highlight decal round a hole)."""
    a = np.linspace(0, 2 * np.pi, n + 1)
    T = []
    for i in range(n):
        p = [(x, y + r * np.cos(t), z + r * np.sin(t)) for r, t in ((r0, a[i]), (r1, a[i]), (r1, a[i + 1]), (r0, a[i + 1]))]
        T += [[p[0], p[1], p[2]], [p[0], p[2], p[3]]]
    return np.array(T)


def shade(T, color):
    rgb = np.array(matplotlib.colors.to_rgb(color))
    nrm = np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0])
    nrm /= np.linalg.norm(nrm, axis=1, keepdims=True) + 1e-12
    k = 0.40 + 0.60 * np.abs(nrm @ LIGHT)
    return np.clip(rgb[None, :] * k[:, None], 0, 1)


class Pic:
    def __init__(self, size=(6.4, 4.8), view=(14, -28), dist=6.6):
        self.fig = plt.figure(figsize=size, dpi=130, facecolor=BG)
        self.ax = self.fig.add_axes([0, 0, 1, 1], projection="3d", facecolor=BG)
        self.view, self.dist = view, dist
        self.T, self.C = [], []

    def add(self, T, color, flat=False):
        T = T if isinstance(T, np.ndarray) else tris(T)
        self.T.append(T)
        self.C.append(np.tile(matplotlib.colors.to_rgb(color), (len(T), 1)) if flat else shade(T, color))

    def frame(self, lo, hi, pad=0.04):
        T, C = np.concatenate(self.T), np.concatenate(self.C)
        self.ax.add_collection3d(Poly3DCollection(T, facecolors=C, edgecolors=C, linewidths=0.25))
        lo, hi = np.asarray(lo, float), np.asarray(hi, float)
        ctr, span = (lo + hi) / 2, (hi - lo).max() * (0.5 + pad)
        for k, s in zip("xyz", range(3)):
            getattr(self.ax, f"set_{k}lim")(ctr[s] - span, ctr[s] + span)
        self.ax.set_box_aspect((1, 1, 1))
        self.ax.view_init(*self.view)
        self.ax.dist = self.dist
        self.ax.set_axis_off()
        self.fig.canvas.draw()

    def p2(self, xyz):
        x, y, _ = proj3d.proj_transform(*xyz, self.ax.get_proj())
        d = self.ax.transData.transform((x, y))
        return self.fig.transFigure.inverted().transform(d)

    def line(self, a, b, color=INK3, lw=0.8, ls=(0, (2, 2))):
        pa, pb = self.p2(a), self.p2(b)
        self.fig.add_artist(matplotlib.lines.Line2D([pa[0], pb[0]], [pa[1], pb[1]], color=color, lw=lw, ls=ls,
                                                    transform=self.fig.transFigure))

    def dim(self, a, b, text, color=INK2, off=(0, 0), fs=10, weight="normal"):
        pa, pb = self.p2(a), self.p2(b)
        self.fig.add_artist(matplotlib.patches.FancyArrowPatch(pa, pb, arrowstyle="<|-|>", mutation_scale=8, color=color,
                                                               lw=0.9, transform=self.fig.transFigure))
        m = (pa + pb) / 2
        self.fig.text(m[0] + off[0], m[1] + off[1], text, color=color, fontsize=fs, ha="center", va="center",
                      family="monospace", weight=weight)

    def label(self, at, text, dxy, color=INK, fs=10, ha="left", weight="normal"):
        p = self.p2(at)
        q = p + np.asarray(dxy)
        self.fig.add_artist(matplotlib.lines.Line2D([p[0], q[0]], [p[1], q[1]], color=color, lw=0.7, transform=self.fig.transFigure))
        self.fig.text(q[0] + (0.006 if ha == "left" else -0.006), q[1], text, color=color, fontsize=fs, ha=ha, va="center",
                      family="monospace", weight=weight)

    def arrow(self, a, b, color, lw=2.2):
        pa, pb = self.p2(a), self.p2(b)
        self.fig.add_artist(matplotlib.patches.FancyArrowPatch(pa, pb, arrowstyle="-|>", mutation_scale=14, color=color,
                                                               lw=lw, transform=self.fig.transFigure))

    def ringmark(self, xyz, color, ms=13):
        p = self.p2(xyz)
        self.fig.add_artist(matplotlib.lines.Line2D([p[0]], [p[1]], marker="o", ms=ms, mfc="none", mec=color, mew=2.2,
                                                    transform=self.fig.transFigure))

    def edge(self, a, b, color, lw=3.0):
        pa, pb = self.p2(a), self.p2(b)
        self.fig.add_artist(matplotlib.lines.Line2D([pa[0], pb[0]], [pa[1], pb[1]], color=color, lw=lw, alpha=0.9,
                                                    transform=self.fig.transFigure))

    def save(self, name):
        out = os.path.join(IMG, name)
        self.fig.savefig(out, facecolor=BG)
        plt.close(self.fig)
        print("wrote", os.path.relpath(out, os.path.dirname(HERE)))


def holes_cut(m, x0=140.0, x1=180.0, which=("foot", "new")):
    for y, z in (FOOT if "foot" in which else []) + (NEW if "new" in which else []):
        m = m - xcyl(y, z, HOLE_D / 2, x0, x1)
    return m


def highlights(pic, x):
    for y, z in NEW:
        pic.ringmark((x, y, z), COL["new"])


def h1_handle():
    """The handle on its own, as drawn in Onshape, with the new holes and their dimensions."""
    fin = holes_cut(box(FIN["x"], FIN["y"], FIN["z"]))
    pic = Pic(size=(6.4, 5.6), view=(12, -24), dist=6.0)
    pic.add(fin, COL["fin"])
    pic.frame((150, -60, 50), (170, 40, 215))
    highlights(pic, FIN["x"][1])
    xf, y0, zt = FIN["x"][1], FIN["y"][0], FIN["z"][1]
    (yn, z1), (_, z2) = NEW
    # from the top, beside the pushing edge
    pic.line((xf, y0, zt), (xf, y0 - 30, zt))
    pic.line((xf, yn, z1), (xf, y0 - 16, z1)); pic.line((xf, yn, z2), (xf, y0 - 30, z2))
    pic.dim((xf, y0 - 13, zt), (xf, y0 - 13, z1), "15", color=COL["new"], off=(-0.025, 0), weight="bold")
    pic.dim((xf, y0 - 27, zt), (xf, y0 - 27, z2), "60", color=COL["new"], off=(-0.025, 0), weight="bold")
    # from the pushing edge, above the top
    pic.line((xf, y0, zt), (xf, y0, zt + 12)); pic.line((xf, yn, z1), (xf, yn, zt + 12))
    pic.dim((xf, y0, zt + 9), (xf, yn, zt + 9), "7.5", color=COL["new"], off=(0, 0.022), weight="bold")
    pic.label((xf, yn + 2.5, (z1 + z2) / 2 + 20), "2 × Ø3.2 thru, new", (0.10, 0.02), color=COL["new"], weight="bold")
    pic.line((xf, yn + 2.5, (z1 + z2) / 2 + 20), (xf, yn + 1.6, z1 - 2.5), color=COL["new"], lw=0.7, ls="-")
    pic.line((xf, yn + 2.5, (z1 + z2) / 2 + 20), (xf, yn + 1.6, z2 + 2.5), color=COL["new"], lw=0.7, ls="-")
    pic.label((FIN["x"][0], y0, 95), "pushing edge", (-0.07, -0.02), ha="right", weight="bold")
    pic.fig.text(pic.p2((FIN["x"][0], y0, 95))[0] - 0.076, pic.p2((FIN["x"][0], y0, 95))[1] - 0.05,
                 "marker D's side, −Y", color=INK2, fontsize=9, ha="right", va="center", family="monospace")
    pic.label((xf, FIN["y"][1], 120), "trailing edge", (0.06, 0.0), color=INK2)
    pic.label((xf, FOOT[3][0], FOOT[3][1]), "4 × Ø3.2, existing", (0.07, -0.035), color=INK2)
    pic.line((xf, y0, 74), (xf, FIN["y"][1], 74), color=INK3, lw=0.9)
    pic.label((xf, FIN["y"][1], 74), "socket top", (0.05, 0.035), color=INK3, fs=9)
    pic.label((xf, FIN["y"][1] - 6, zt), "top, 204 above the table", (0.06, 0.03), color=INK2, fs=9)
    pic.save("h1_handle.png")


def valve_end():
    shells = cm.read_shells(cm.MODEL)
    crop = box((118.0, 215.0), (-62.0, 62.0), (30.0, 230.0))
    ri = max(range(len(shells)), key=lambda i: shells[i].bounding_box()[3] - shells[i].bounding_box()[0])
    rotor = shells[ri] ^ crop
    others = []
    for i, s in enumerate(shells):
        if i == ri:
            continue
        b = s.bounding_box()
        if b[0] > 85 and b[3] < 215 and b[5] > 45:                    # markers C, D, E and their mounts
            others.append(s)
    return rotor, others


def color_of(s):
    b = s.bounding_box()
    dims = np.array(b[3:]) - np.array(b[:3])
    return COL["marker"] if dims.max() > 29 else COL["mount"]


def h2_h3():
    rotor, others = valve_end()
    rotor = holes_cut(rotor, which=("new",))
    lo, hi = (118, -62, 30), (215, 62, 215)
    # 2: the holes marked
    pic = Pic(size=(6.4, 5.2), view=(20, -38), dist=6.3)
    pic.add(rotor, COL["rotor"])
    for s in others:
        pic.add(s, color_of(s))
    pic.frame(lo, hi)
    xf, y0 = FIN["x"][1], FIN["y"][0]
    highlights(pic, xf)
    pic.edge((FIN["x"][0], y0, 74), (FIN["x"][0], y0, 204), COL["new"])
    a, b2 = pic.p2((xf, NEW[0][0], NEW[0][1])), pic.p2((xf, NEW[1][0], NEW[1][1]))
    q = np.array([max(a[0], b2[0]) + 0.12, a[1] + 0.04])
    for p in (a, b2):
        pic.fig.add_artist(matplotlib.lines.Line2D([p[0], q[0]], [p[1], q[1]], color=COL["new"], lw=0.8, transform=pic.fig.transFigure))
    pic.fig.text(q[0] + 0.006, q[1], "drill 2 × Ø3.2 here", color=COL["new"], fontsize=10, ha="left", va="center",
                 family="monospace", weight="bold")
    pic.label((FIN["x"][0], y0, 110), "pushing edge", (-0.05, -0.06), ha="right", weight="bold")
    d = [s for s in others if color_of(s) == COL["marker"]]
    md = min(d, key=lambda s: s.bounding_box()[1])                     # the ball furthest to -y = marker D
    b = md.bounding_box()
    pic.label(((b[0] + b[3]) / 2, b[1], (b[2] + b[5]) / 2 + 10), "marker D", (-0.06, 0.04), ha="right", color=INK2)
    pic.arrow((150, -95, 150), (150, -62, 150), COL["new"])
    pic.fig.text(pic.p2((150, -95, 150))[0], pic.p2((150, -95, 150))[1] - 0.035, "the claw pushes from here", color=COL["new"],
                 fontsize=9, ha="center", family="monospace")
    pic.save("h2_on_valve.png")
    # 3: fitted
    plate = box(PLATE["x"], PLATE["y"], PLATE["z"])
    s2 = SLOT / 2
    xc = sum(FIN["x"]) / 2
    for xa, xb in ((xc - s2 - CHEEK, xc - s2), (xc + s2, xc + s2 + CHEEK)):
        plate = plate + box((xa, xb), (PLATE["y"][1], PLATE["y"][1] + CHEEK_L), PLATE["z"])
    screws, nuts = None, None
    xo, xi = xc + s2 + CHEEK, xc - s2 - CHEEK
    for y, z in NEW:
        plate = plate - xcyl(y, z, HOLE_D / 2, 140, 180)
        sc = xcyl(y, z, 2.75, xo, xo + 3.0) + xcyl(y, z, 1.5, xo - 16.0, xo)
        nu = xhex(y, z, 5.5, xi - 2.4, xi) - xcyl(y, z, 1.5, xi - 3, xi + 1)
        screws = sc if screws is None else screws + sc
        nuts = nu if nuts is None else nuts + nu
    pic = Pic(size=(6.4, 5.2), view=(38, 40), dist=6.3)
    pic.add(rotor, COL["rotor"])
    for s in others:
        pic.add(s, color_of(s))
    pic.add(plate, COL["plate"])
    pic.add(screws, COL["steel"])
    pic.add(nuts, COL["nut"])
    pic.frame(lo, hi)
    a, b2 = pic.p2((xo + 3.0, NEW[0][0], NEW[0][1])), pic.p2((xo + 3.0, NEW[1][0], NEW[1][1]))
    q = np.array([max(a[0], b2[0]) + 0.10, (a[1] + b2[1]) / 2 + 0.03])
    for p in (a, b2):
        pic.fig.add_artist(matplotlib.lines.Line2D([p[0], q[0]], [p[1], q[1]], color=INK, lw=0.8, transform=pic.fig.transFigure))
    pic.fig.text(q[0] + 0.006, q[1], "2 × M3 × 16, nuts behind", color=INK, fontsize=10, ha="left", va="center", family="monospace")
    pa = pic.p2((112, PLATE["y"][1], 196))
    pic.label((112, PLATE["y"][1], 196), "push plate", (0.05, -0.015), color=COL["plate"], weight="bold")
    pic.fig.text(pa[0] + 0.056, pa[1] - 0.05, "pushing face on its far side", color=INK2, fontsize=9, ha="left", va="center",
                 family="monospace")
    pic.label((xo, PLATE["y"][1] + CHEEK_L, 128), "cheeks round the fin", (0.07, -0.05), color=INK2)
    pic.save("h3_fitted.png")


if __name__ == "__main__":
    os.makedirs(IMG, exist_ok=True)
    h1_handle()
    h2_h3()
