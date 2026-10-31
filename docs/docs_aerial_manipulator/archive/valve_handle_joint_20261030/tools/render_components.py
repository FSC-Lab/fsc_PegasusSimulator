#!/usr/bin/env python3
"""Pictures of each component of the roll-fin handle, for the design page (the page's 3D view has the
assembled and exploded views).

Writes img/*.png (dark background, the page's panel colour). Uses the geometry of roll_grip_design.py
and the valve model read by check_on_model.py, so the pictures are the print files.

    PYTHONNOUSERSITE=1 PYTHONPATH=~/sources/pymanifold /usr/bin/python3 render_components.py
"""
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt                                        # noqa: E402
from mpl_toolkits.mplot3d.art3d import Poly3DCollection                # noqa: E402
import manifold3d as mf                                                # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import roll_grip_design as rg                                          # noqa: E402
import check_on_model as cm                                            # noqa: E402

IMG = os.path.join(os.path.dirname(HERE), "img")
BG, INK, INK2 = "#111214", "#ecebe6", "#b9b8b0"
COL = {"base": "#7b8794", "fin": "#6fb5a6", "steel": "#c9cdd3", "rubber": "#5a5e66", "rotor": "#cfc8b8", "cut": "#d03b3b"}
LIGHT = np.array([0.35, -0.55, 0.75]) / np.linalg.norm([0.35, -0.55, 0.75])


def tris(m):
    me = m.to_mesh()
    V = np.asarray(me.vert_properties)[:, :3]
    return V[np.asarray(me.tri_verts)]


def shaded(T, color, alpha=1.0):
    rgb = np.array(matplotlib.colors.to_rgb(color))
    nrm = np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0])
    nrm /= np.linalg.norm(nrm, axis=1, keepdims=True) + 1e-12
    k = 0.42 + 0.58 * np.clip(nrm @ LIGHT, 0, 1)
    return np.clip(rgb[None, :] * k[:, None], 0, 1), alpha


def draw(parts, out, view=(22, -58), size=(5.2, 4.0), labels=(), pad=0.06, zoom=1.0, dist=6.4, fs=10):
    """parts: [(manifold or triangle array, colour, alpha)]; labels: [(xyz, text, align)]."""
    fig = plt.figure(figsize=size, dpi=110, facecolor=BG)
    ax = fig.add_axes([0, 0, 1, 1], projection="3d", facecolor=BG)
    allv = []
    for p, color, alpha in parts:
        T = p if isinstance(p, np.ndarray) else tris(p)
        fc, a = shaded(T, color, alpha)
        pc = Poly3DCollection(T, facecolors=fc, edgecolors=fc, linewidths=0.35, alpha=a)
        ax.add_collection3d(pc)
        allv.append(T.reshape(-1, 3))
    V = np.concatenate(allv)
    lo, hi = V.min(0), V.max(0)
    ctr, span = (lo + hi) / 2, (hi - lo).max() * (0.5 + pad) / zoom
    ax.set_xlim(ctr[0] - span, ctr[0] + span); ax.set_ylim(ctr[1] - span, ctr[1] + span); ax.set_zlim(ctr[2] - span, ctr[2] + span)
    ax.set_box_aspect((1, 1, 1))
    ax.view_init(*view)
    ax.dist = dist
    ax.set_axis_off()
    for xyz, text, ha in labels:
        ax.text(*xyz, text, color=INK, fontsize=fs, ha=ha, family="monospace")
    fig.savefig(out, facecolor=BG)
    plt.close(fig)
    print("wrote", os.path.relpath(out, os.path.dirname(HERE)))


def main():
    os.makedirs(IMG, exist_ok=True)
    base, fin = rg.handle_base(), rg.roll_fin()
    screw, ring, bolts, nuts = rg.m4_screw(), rg.o_ring(), rg.m3_bolts(), rg.m3_nuts()

    # 0. today's fin and where to saw it
    shells = cm.read_shells(cm.MODEL)
    ri = max(range(len(shells)), key=lambda i: shells[i].bounding_box()[3] - shells[i].bounding_box()[0])
    crop = rg.box((128.0, 210.0), (-60.0, 60.0), (30.0, 230.0))
    rotor = shells[ri] ^ crop
    fx, fy = rg.FIN["x"], rg.FIN["y"]
    top = rg.box((fx[0] - 0.5, fx[1] + 0.5), (fy[0] - 0.5, fy[1] + 0.5), (rg.Z_CUT, 260.0))
    draw([(rotor - top, COL["rotor"], 1.0), (rotor ^ top, COL["cut"], 0.55)], os.path.join(IMG, "c0_cut.png"),
         view=(14, 40), labels=[((160, -22, rg.Z_CUT + 2), "saw here  ", "right"), ((160, 23, 185), "  removed", "left")], dist=6.0)

    draw([(base, COL["base"], 1.0)], os.path.join(IMG, "c1_base.png"), view=(16, 32), dist=5.6)
    draw([(fin, COL["fin"], 1.0)], os.path.join(IMG, "c2_fin.png"), view=(14, 28), dist=5.4)
    # hardware, each standing on its own axis
    s = rg
    m4 = mf.Manifold.cylinder(rg.SCREW_L, 2.0, 2.0, 32) + mf.Manifold.cylinder(rg.HEAD_H, rg.HEAD_D / 2, rg.HEAD_D / 2, 40).translate((0, 0, rg.SCREW_L)) \
        - mf.CrossSection([rg.hexagon(3.0, False)]).extrude(2.6).translate((0, 0, rg.SCREW_L + rg.HEAD_H - 2.5))
    draw([(m4, COL["steel"], 1.0)], os.path.join(IMG, "c3_m4.png"), view=(18, -50), size=(3.2, 4.0), dist=5.4)
    r_mid = (rg.ORING_ID + rg.ORING_CS) / 2
    oring = mf.CrossSection.circle(rg.ORING_CS / 2, 32).translate((r_mid, 0)).revolve(64)
    draw([(oring, "#7a7f88", 1.0)], os.path.join(IMG, "c4_oring.png"), view=(48, -50), size=(3.2, 3.2), dist=6.2)
    m3 = mf.Manifold.cylinder(16, 1.5, 1.5, 24) + mf.Manifold.cylinder(3, 2.75, 2.75, 32).translate((0, 0, 16)) \
        - mf.CrossSection([rg.hexagon(2.5, False)]).extrude(1.6).translate((0, 0, 17.5))
    draw([(m3, COL["steel"], 1.0)], os.path.join(IMG, "c5_m3.png"), view=(18, -50), size=(3.2, 4.0), dist=5.4)
    nut = mf.CrossSection([rg.hexagon(5.5, False)]).extrude(2.4) - mf.Manifold.cylinder(6, 1.5, 1.5, 24).translate((0, 0, -1))
    draw([(nut, COL["steel"], 1.0)], os.path.join(IMG, "c6_nut.png"), view=(48, -50), size=(3.2, 3.2), dist=6.2)


if __name__ == "__main__":
    main()
