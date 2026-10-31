#!/usr/bin/env python3
"""THE MAIL ENVELOPE'S PRINTED CORE (2026-10-28, rev C, all PLA): one parametric
model, written as a binary STL (for printing), an OpenSCAD source (for editing)
and a JSON of the dimensions and the mass estimate (for the drawing page).

    /usr/bin/python3 make_envelope_core.py [--out DIR]

A rectangular PLA plate, printed in one piece, no inserts: a solid grip band at
the near edge, a 2 x 2 grid of identical rectangular through-windows with 6 mm
webs, and a solid band at the far end.

Rev B carried a steel bar in a far-end pocket to put the centre of mass 40 mm
from the far end. Rev C drops it on the user's request (no steel; the centre of
mass is not a hard requirement any more). The windows sit nearer the grip than
the far end, so the far band is the widest solid region; printed like the rest
the centre of mass is near the middle, and printing the far band at 100 %
infill (a slicer modifier, no geometry change) moves it ~11 mm toward the far
end.

Frame: x from the NEAR edge (the one the jaws grip) toward the FAR end (the end
that lies on the hat and enters the mailbox first), y across from the RIGHT
long edge (looking from the near edge to the far end), z up from the bottom
face. Every dimension is a round number from that corner, for CAD.

Every feature is an axis-aligned block, so the solid is a union of cells of one
rectilinear grid and its boundary is written cell face by cell face. Every
vertex sits on that grid, so the mesh is closed (checked before writing: each
directed edge appears once, and its reverse once).
"""
import argparse
import itertools
import json
import os
import struct

HERE = os.path.dirname(os.path.abspath(__file__))

# ---- dimensions [mm] -----------------------------------------------------------
L, W, T = 150.0, 90.0, 3.8          # core; ~0.15 mm of paper a face gives ~4.1 mm finished
GRIP = 35.0                         # solid grip band from the near edge (the jaws reach 31 mm in)
HOLE_X, HOLE_Y = 30.0, 36.0         # each window
WEB = 6.0                           # rim and ribs between the windows
NX, NY = 2, 2                       # windows along x, across y
FAR = L - GRIP - NX * HOLE_X - (NX - 1) * WEB      # the solid far band
assert abs(W - (NY * HOLE_Y + (NY + 1) * WEB)) < 1e-9, "webs + windows must fill the width"

# ---- materials and print estimate ----------------------------------------------
RHO_PLA = 1.24e-3                            # g/mm^3
PAPER_GSM, PAPER_AREA_M2 = 120.0, 0.030      # both faces + overlap
WALLS, LAYERS_TB, LAYER, INFILL = 3, 4, 0.2, 0.20
LINE = 0.45


def holes():
    """The windows as (x0, x1, y0, y1), y from the right long edge."""
    out = []
    for i in range(NX):
        x0 = GRIP + i * (HOLE_X + WEB)
        for j in range(NY):
            y0 = WEB + j * (HOLE_Y + WEB)
            out.append((x0, x0 + HOLE_X, y0, y0 + HOLE_Y))
    return out


def grid():
    xs, ys = {0.0, L}, {0.0, W}
    for x0, x1, y0, y1 in holes():
        xs |= {x0, x1}
        ys |= {y0, y1}
    return sorted(xs), sorted(ys), [0.0, T]


def occupied(xm, ym, zm):
    return not any(x0 < xm < x1 and y0 < ym < y1 for x0, x1, y0, y1 in holes())


def mesh():
    xs, ys, zs = grid()
    nx, ny, nz = len(xs) - 1, len(ys) - 1, len(zs) - 1
    occ = {}
    for i, j, k in itertools.product(range(nx), range(ny), range(nz)):
        occ[i, j, k] = occupied(0.5 * (xs[i] + xs[i + 1]), 0.5 * (ys[j] + ys[j + 1]), 0.5 * (zs[k] + zs[k + 1]))

    def full(i, j, k):
        return 0 <= i < nx and 0 <= j < ny and 0 <= k < nz and occ[i, j, k]

    tris = []
    for (i, j, k), on in occ.items():
        if not on:
            continue
        x0, x1, y0, y1, z0, z1 = xs[i], xs[i + 1], ys[j], ys[j + 1], zs[k], zs[k + 1]
        # each face: outward normal, corners counter-clockwise seen from outside
        faces = [((-1, 0, 0), (i - 1, j, k), [(x0, y0, z0), (x0, y0, z1), (x0, y1, z1), (x0, y1, z0)]),
                 ((1, 0, 0), (i + 1, j, k), [(x1, y0, z0), (x1, y1, z0), (x1, y1, z1), (x1, y0, z1)]),
                 ((0, -1, 0), (i, j - 1, k), [(x0, y0, z0), (x1, y0, z0), (x1, y0, z1), (x0, y0, z1)]),
                 ((0, 1, 0), (i, j + 1, k), [(x0, y1, z0), (x0, y1, z1), (x1, y1, z1), (x1, y1, z0)]),
                 ((0, 0, -1), (i, j, k - 1), [(x0, y0, z0), (x0, y1, z0), (x1, y1, z0), (x1, y0, z0)]),
                 ((0, 0, 1), (i, j, k + 1), [(x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)])]
        for n, nb, q in faces:
            if not full(*nb):
                tris.append((n, q[0], q[1], q[2]))
                tris.append((n, q[0], q[2], q[3]))
    return tris


def check_closed(tris):
    edges = {}
    for _, a, b, c in tris:
        for p, q in ((a, b), (b, c), (c, a)):
            edges[(p, q)] = edges.get((p, q), 0) + 1
    bad = [e for e, n in edges.items() if n != 1 or edges.get((e[1], e[0]), 0) != 1]
    if bad:
        raise SystemExit(f"mesh not closed: {len(bad)} bad directed edges, e.g. {bad[:3]}")


def volume_and_moment(tris):
    v = mx = 0.0
    for _, a, b, c in tris:
        d = (a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0])
             + a[2] * (b[0] * c[1] - b[1] * c[0])) / 6.0
        v += d
        mx += d * (a[0] + b[0] + c[0]) / 4.0
    return v, mx / v


def write_stl(tris, path):
    with open(path, "wb") as f:
        f.write(b"mail envelope core rev C (all PLA), mm".ljust(80, b" "))
        f.write(struct.pack("<I", len(tris)))
        for n, a, b, c in tris:
            f.write(struct.pack("<12fH", *n, *a, *b, *c, 0))


def print_fraction(narrow, infill=INFILL):
    """Filled share of a printed region whose narrow side is `narrow` mm:
    WALLS perimeters each side, LAYERS_TB solid top/bottom layers, `infill` inside."""
    walls = min(1.0, 2 * WALLS * LINE / narrow)
    skins = min(1.0, 2 * LAYERS_TB * LAYER / T)
    return walls + (1 - walls) * (skins + (1 - skins) * infill)


def mass_budget(far_infill=INFILL):
    """Estimated part masses [g] and their x [mm] from the near edge, from the
    region geometry and the print settings above (an estimate: weigh the part)."""
    zone = NX * HOLE_X + (NX - 1) * WEB
    web_v = zone * W * T - NX * NY * HOLE_X * HOLE_Y * T
    rows = [(f"grip band (0-{GRIP:.0f})", GRIP * W * T * RHO_PLA * print_fraction(GRIP), 0.5 * GRIP),
            (f"window zone ({GRIP:.0f}-{GRIP + zone:.0f}), {WEB:.0f} mm webs",
             web_v * RHO_PLA * print_fraction(WEB), GRIP + 0.5 * zone),
            (f"far band ({L - FAR:.0f}-{L:.0f}), {far_infill * 100:.0f} % infill",
             FAR * W * T * RHO_PLA * print_fraction(FAR, far_infill), L - 0.5 * FAR),
            ("paper wrap, 120 g/m2", PAPER_GSM * PAPER_AREA_M2, 0.5 * L)]
    m = sum(r[1] for r in rows)
    x = sum(r[1] * r[2] for r in rows) / m
    return rows, m, x


SCAD = """// Mail envelope core, rev C (2026-10-28), all PLA, units mm. Generated by
// make_envelope_core.py, the same dimensions as envelope_core.stl.
// Origin = the near-edge / right-edge corner of the bottom face. x from the NEAR
// edge (the jaws grip it) to the FAR end (lies on the hat, enters the mailbox
// first); y across from the right long edge; z up.
L = {L}; W = {W}; T = {T};
GRIP = {GRIP};                      // solid grip band from the near edge
HOLE = [{hx}, {hy}];               // each through-window, x by y
WEB = {WEB}; NX = {NX}; NY = {NY};    // window grid; the webs between and round them
// the far band is what is left: L - GRIP - NX*HOLE[0] - (NX-1)*WEB = {FAR}

difference() {{
    cube([L, W, T]);
    for (i = [0 : NX - 1], j = [0 : NY - 1])
        translate([GRIP + i * (HOLE[0] + WEB), WEB + j * (HOLE[1] + WEB), -1])
            cube([HOLE[0], HOLE[1], T + 2]);
}}
"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=HERE)
    a = ap.parse_args()
    tris = mesh()
    check_closed(tris)
    vol, xg = volume_and_moment(tris)
    rows, m, x = mass_budget()
    rows_s, m_s, x_s = mass_budget(far_infill=1.0)
    os.makedirs(a.out, exist_ok=True)
    stl = os.path.join(a.out, "envelope_core.stl")
    write_stl(tris, stl)
    with open(os.path.join(a.out, "envelope_core.scad"), "w") as f:
        f.write(SCAD.format(L=L, W=W, T=T, GRIP=GRIP, hx=HOLE_X, hy=HOLE_Y, WEB=WEB, NX=NX, NY=NY, FAR=FAR))
    info = dict(rev="C", L=L, W=W, T=T, grip=GRIP, far=FAR, web=WEB, nx=NX, ny=NY, hole=[HOLE_X, HOLE_Y],
                holes=holes(), triangles=len(tris), solid_volume_mm3=vol, solid_cx_mm=xg,
                solid_pla_g=vol * RHO_PLA, budget=rows, mass_g=m, com_from_near_mm=x, com_from_far_mm=L - x,
                solid_far=dict(budget=rows_s, mass_g=m_s, com_from_near_mm=x_s, com_from_far_mm=L - x_s),
                print=dict(walls=WALLS, top_bottom_layers=LAYERS_TB, layer_mm=LAYER, infill=INFILL))
    with open(os.path.join(a.out, "envelope_core.json"), "w") as f:
        json.dump(info, f, indent=1)
    print(f"closed mesh, {len(tris)} triangles -> {stl}")
    print(f"{NX * NY} windows {HOLE_X:.0f} x {HOLE_Y:.0f} mm, webs {WEB:.0f} mm, grip band {GRIP:.0f}, far band {FAR:.0f}")
    print(f"solid volume {vol / 1e3:.2f} cm3 ({vol * RHO_PLA:.1f} g at 100 % PLA), its centroid x {xg:.1f} mm")
    for n, mm, xx in rows:
        print(f"   {n:42s} {mm:5.1f} g at x {xx:5.1f} mm")
    print(f"ESTIMATE {m:.1f} g, centre of mass {x:.1f} mm from the near edge = {L - x:.1f} mm from the far end")
    print(f"far band at 100 % infill: {m_s:.1f} g, centre of mass {L - x_s:.1f} mm from the far end")


if __name__ == "__main__":
    main()
