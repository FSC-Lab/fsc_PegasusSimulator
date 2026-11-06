#!/usr/bin/env python3
"""Valve_Assembly_v20.stl + THE PUSH PLATE -> Valve_Assembly_v21.stl (2026-11-04).

    /usr/bin/python3 make_v21_stl.py [--width 0.120] [--bottom 0.125] [--top 0.204] [--thick 0.005] [--out ...]

The user's design (2026-11-04): close the gripper and push with its NOSE only, on a flat plate fixed
to the fin's PUSHING edge, so the fin can never be inside the claw. The plate is ONE closed solid
added to the v20 assembly, unchanged otherwise (every v20 triangle is copied byte for byte):

  the plate     a flat rectangle standing on the fin's pushing (back, -y) edge, square to the push:
                WIDTH along the beam (radial, rotor x) centred on the fin's 0.160 m radius,
                HEIGHT from --bottom to --top above the plate bottom, THICK along the push (y);
                its pushing face at y = fin back edge - THICK
  the clip      two 3 mm cheeks either side of the fin, 15 mm along its faces, the plate's height:
                the plate slides onto the fin's back edge and one M3 bolt through cheeks and fin
                holds it (not modelled)

SUPERSEDED 2026-11-04: Valve_Assembly_v21.stl is now the user's own Onshape design of this part (the
same plate, two M3 holes through cheeks and fin, a 5.0 mm slot). This script refuses to overwrite a
file it did not write; pass --out elsewhere (or --force) to rebuild the original model.

Frames: the STL's own CAD frame (metres; the bearing axis at (-1.1, 3.0) mm, the plate bottom at
z = -4.5 mm), the same as v20's -- stl_to_usda_valve.py moves it to the asset frame. Dimensions on the
command line are in the ASSET frame (from the axis on the plate bottom), as the design page and the
planner use them.
"""
import argparse
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ASSETS = os.path.abspath(os.path.join(HERE, *[".."] * 5, "extensions", "fsc_aerial_manipulation",
                                      "fsc_aerial_manipulation", "rotorcraft", "assets"))
AXIS_MM = np.array([-1.1, 3.0])          # the bearing axis in the CAD frame [mm]
PLATE_BOTTOM_MM = -4.5
# the fin (asset frame, m): radial 0.1575..0.1625, along the push -0.0194..0.0206, z 0.074..0.204
FIN_X = (0.1575, 0.1625)
FIN_BACK_Y = -0.0194
CHEEK_T, CHEEK_L = 0.003, 0.015


def read_stl(path):
    raw = open(path, "rb").read()
    n = int(np.frombuffer(raw[80:84], "<u4")[0])
    if 84 + 50 * n != len(raw):
        raise SystemExit(f"{path}: not a binary STL")
    dt = np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
    return raw[:80], np.frombuffer(raw[84:84 + 50 * n], dt).copy()


def ear_clip(poly):
    """Triangulate a simple CCW polygon (list of (x, y)) -> index triples."""
    idx = list(range(len(poly)))
    P = np.asarray(poly, float)
    tris = []

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    def inside(p, a, b, c):
        return cross(a, b, p) >= -1e-15 and cross(b, c, p) >= -1e-15 and cross(c, a, p) >= -1e-15

    guard = 0
    while len(idx) > 3 and guard < 10000:
        guard += 1
        for k in range(len(idx)):
            i0, i1, i2 = idx[k - 1], idx[k], idx[(k + 1) % len(idx)]
            a, b, c = P[i0], P[i1], P[i2]
            if cross(a, b, c) <= 1e-18:
                continue
            if any(inside(P[j], a, b, c) for j in idx if j not in (i0, i1, i2)):
                continue
            tris.append((i0, i1, i2))
            idx.pop(k)
            break
        else:
            raise SystemExit("ear clipping failed (polygon not simple?)")
    tris.append(tuple(idx))
    return tris


def prism(poly_xy, z0, z1):
    """A closed, outward-wound prism: the CCW outline poly_xy extruded z0..z1 -> (n, 3, 3)."""
    poly = list(poly_xy)
    a = 0.5 * sum(poly[i][0] * poly[(i + 1) % len(poly)][1] - poly[(i + 1) % len(poly)][0] * poly[i][1]
                  for i in range(len(poly)))
    if a < 0:
        poly = poly[::-1]
    T = []
    for i, j, k in ear_clip(poly):
        T.append([(*poly[i], z1), (*poly[j], z1), (*poly[k], z1)])          # top, CCW from above
        T.append([(*poly[i], z0), (*poly[k], z0), (*poly[j], z0)])          # bottom, reversed
    n = len(poly)
    for i in range(n):
        p, q = poly[i], poly[(i + 1) % n]
        T.append([(*p, z0), (*q, z0), (*q, z1)])
        T.append([(*p, z0), (*q, z1), (*p, z1)])
    return np.array(T, float)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--src", default=os.path.join(ASSETS, "Valve_Assembly_v20.stl"))
    ap.add_argument("--out", default=os.path.join(ASSETS, "Valve_Assembly_v21.stl"))
    ap.add_argument("--width", type=float, default=0.120, help="radial width, centred on the fin [m]")
    ap.add_argument("--bottom", type=float, default=0.125, help="its bottom above the plate bottom [m]")
    ap.add_argument("--top", type=float, default=0.204, help="its top above the plate bottom [m] (the fin top 0.204)")
    ap.add_argument("--thick", type=float, default=0.005, help="along the push [m]")
    ap.add_argument("--centre", type=float, default=0.160, help="its centre's radius [m] (the fin's)")
    ap.add_argument("--force", action="store_true", help="overwrite --out even if this script did not write it")
    a = ap.parse_args()
    if os.path.exists(a.out) and not a.force and not open(a.out, "rb").read(80).startswith(b"Valve_Assembly_v21: v20 + push plate"):
        raise SystemExit(f"{a.out} was not written by this script (the user's CAD since 2026-11-04): "
                         "pass --out elsewhere, or --force")

    xc, hw = a.centre, 0.5 * a.width
    y0, y1 = FIN_BACK_Y - a.thick, FIN_BACK_Y              # the plate: its pushing face at y0
    yc = FIN_BACK_Y + CHEEK_L                              # the cheeks' far end along the fin
    xl, xr = FIN_X[0] - CHEEK_T, FIN_X[1] + CHEEK_T
    # the outline, asset frame (x radial, y along the push), counter-clockwise from above
    outline = [(xc - hw, y0), (xc + hw, y0), (xc + hw, y1), (xr, y1), (xr, yc), (FIN_X[1], yc),
               (FIN_X[1], y1), (FIN_X[0], y1), (FIN_X[0], yc), (xl, yc), (xl, y1), (xc - hw, y1)]
    T = prism(outline, a.bottom, a.top)                    # asset frame, m
    # -> the CAD frame, m
    T_cad = T.copy()
    T_cad[..., 0] += AXIS_MM[0] * 1e-3
    T_cad[..., 1] += AXIS_MM[1] * 1e-3
    T_cad[..., 2] += PLATE_BOTTOM_MM * 1e-3

    header, src = read_stl(a.src)
    dt = src.dtype
    add = np.zeros(len(T_cad), dt)
    add["v"] = T_cad.astype(np.float32)
    e1 = T_cad[:, 1] - T_cad[:, 0]
    e2 = T_cad[:, 2] - T_cad[:, 0]
    nrm = np.cross(e1, e2)
    nrm /= np.linalg.norm(nrm, axis=1, keepdims=True)
    add["n"] = nrm.astype(np.float32)
    out = np.concatenate([src, add])
    hdr = (b"Valve_Assembly_v21: v20 + push plate (fsc_PegasusSimulator make_v21_stl.py)").ljust(80, b" ")[:80]
    with open(a.out, "wb") as f:
        f.write(hdr)
        f.write(np.uint32(len(out)).tobytes())
        f.write(out.tobytes())
    # the closed solid's volume (signed tetrahedra) as a check of the winding
    v6 = np.einsum("ij,ij->i", T[:, 0], np.cross(T[:, 1], T[:, 2]))
    vol = v6.sum() / 6.0
    print(f"wrote {a.out}: {len(src)} v20 triangles + {len(add)} for the push plate")
    print(f"  push plate (asset frame): radial {xc - hw:.4f}..{xc + hw:.4f} m ({a.width * 1e3:.0f} mm wide), "
          f"z {a.bottom:.4f}..{a.top:.4f} ({(a.top - a.bottom) * 1e3:.0f} mm tall), pushing face at y {y0:.4f} "
          f"({a.thick * 1e3:.0f} mm thick); clip cheeks {CHEEK_T * 1e3:.0f} mm x {CHEEK_L * 1e3:.0f} mm on the fin")
    print(f"  solid volume {vol * 1e6:.1f} cm^3 ({'outward' if vol > 0 else 'INWARD -- winding wrong'})")


if __name__ == "__main__":
    main()
