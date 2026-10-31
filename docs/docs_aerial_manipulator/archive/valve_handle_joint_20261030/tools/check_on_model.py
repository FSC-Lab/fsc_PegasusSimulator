#!/usr/bin/env python3
"""Fit the roll-fin handle onto the current valve model and check it against every body in it.

Reads rotorcraft/assets/Valve_Assembly_v2.stl (metres, CAD frame), cuts the fin at roll_grip_design.Z_CUT,
adds the printed handle body, the grip sleeve and the screws, and reports the overlap volume and the
smallest gap between each new part and each body of the model. Writes the fitted assembly in the
model's own frame and units, so it opens in place over the CAD:

  stl/Valve_Assembly_v2_roll_fin.stl      the model with the fin cut + the new parts (metres, CAD frame)

    PYTHONPATH=~/sources/pymanifold /usr/bin/python3 check_on_model.py
"""
import os
import struct
import sys

import numpy as np
import manifold3d as mf

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import roll_grip_design as rg   # noqa: E402

REPO = os.path.abspath(os.path.join(HERE, *[".."] * 5))
MODEL = os.path.join(REPO, "extensions/fsc_aerial_manipulation/fsc_aerial_manipulation/rotorcraft/assets/Valve_Assembly_v2.stl")
CAD_TO_VALVE = -np.array([-1.1, 3.0, -4.5])            # mm: the valve frame's origin in the CAD frame, negated


def read_shells(path):
    b = open(path, "rb").read()
    n = struct.unpack("<I", b[80:84])[0]
    assert 84 + 50 * n == len(b), "binary STL expected"
    a = np.frombuffer(b[84:], dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]), count=n)
    T = a["v"].astype(float) * 1000.0 + CAD_TO_VALVE
    V, inv = np.unique(T.reshape(-1, 3).round(4), axis=0, return_inverse=True)
    F = inv.reshape(-1, 3)
    par = np.arange(len(V))

    def find(i):
        while par[i] != i:
            par[i] = par[par[i]]
            i = par[i]
        return i
    for t in F:
        r = find(t[0])
        for v in t[1:]:
            par[find(v)] = r
    root = np.array([find(i) for i in range(len(V))])
    shells = []
    for r in np.unique(root):
        keep = root[F[:, 0]] == r
        Fi = F[keep]
        used, Fl = np.unique(Fi, return_inverse=True)
        m = mf.Manifold(mf.Mesh(vert_properties=V[used].astype(np.float32), tri_verts=Fl.reshape(-1, 3).astype(np.uint32)))
        if m.status() != mf.Error.NoError or m.volume() <= 0:
            Fl = Fl.reshape(-1, 3)[:, ::-1]         # inward-wound shell: flip it
            m = mf.Manifold(mf.Mesh(vert_properties=V[used].astype(np.float32), tri_verts=np.ascontiguousarray(Fl).astype(np.uint32)))
        shells.append(m)
    return shells


def name_of(m):
    lo = np.array(m.bounding_box()[:3]); hi = np.array(m.bounding_box()[3:])
    c, d = (lo + hi) / 2, hi - lo
    if d[0] > 150:
        return "rotor (hub, beam, root, fin)"
    if abs(d[0] - 30) < 1 and abs(d[2] - 30) < 1 and abs(d[1] - 30) < 1:
        return f"marker ball at ({c[0]:.0f}, {c[1]:.0f}, {c[2]:.0f})"
    if lo[2] > 45 and d[2] > 40:
        return f"marker on its mount, x {lo[0]:.0f}..{hi[0]:.0f}"
    if lo[2] >= 45:
        return f"marker mount, x {lo[0]:.0f}..{hi[0]:.0f}"
    if d[0] > 140:
        return "base plate"
    return f"static part x {lo[0]:.0f}..{hi[0]:.0f}, z {lo[2]:.0f}..{hi[2]:.0f}"


def write_stl_m(path, parts):
    tris = []
    for m in parts:
        me = m.to_mesh()
        V = (np.asarray(me.vert_properties)[:, :3] - CAD_TO_VALVE) / 1000.0
        tris.append(V[np.asarray(me.tri_verts)])
    T = np.concatenate(tris)
    with open(path, "wb") as f:
        f.write(b"Valve_Assembly_v2 + roll-fin handle, metres, CAD frame".ljust(80, b" "))
        f.write(struct.pack("<I", len(T)))
        for a, b, c in T:
            nrm = np.cross(b - a, c - a)
            nrm = nrm / (np.linalg.norm(nrm) or 1.0)
            f.write(struct.pack("<12fH", *nrm, *a, *b, *c, 0))


def main():
    shells = read_shells(MODEL)
    print(f"{os.path.relpath(MODEL, REPO)}: {len(shells)} closed bodies")
    rotor_i = max(range(len(shells)), key=lambda i: shells[i].bounding_box()[3] - shells[i].bounding_box()[0])
    fx, fy = rg.FIN["x"], rg.FIN["y"]
    removed = rg.box((fx[0] - 0.5, fx[1] + 0.5), (fy[0] - 0.5, fy[1] + 0.5), (rg.Z_CUT, 260.0))
    fin_top = shells[rotor_i] ^ removed
    shells[rotor_i] = shells[rotor_i] - removed
    print(f"the fin above z {rg.Z_CUT:.0f} cut off the rotor: {fin_top.volume() / 1e3:.1f} cm^3 "
          f"(box {np.round(fin_top.bounding_box(), 1).tolist()})")

    # the stump drilled for the two M3 bolts (the assembly as built: drilled through the clamp's holes)
    for y, z in rg.BOLTS:
        shells[rotor_i] -= mf.Manifold.cylinder(40, rg.M3_HOLE / 2, rg.M3_HOLE / 2, 32).transform(
            rg.frame(np.array([140.0, y, z]), rg.X, np.array([0, 1.0, 0])))

    hb, fin = rg.handle_base(), rg.roll_fin()
    screw, ring = rg.m4_screw(), rg.o_ring()
    bolts, nuts = rg.m3_bolts().decompose(), rg.m3_nuts().decompose()
    new = [("base", hb), ("roll fin", fin), ("M4 screw", screw), ("O-ring", ring)] \
        + [(f"M3 bolt {i + 1}", b) for i, b in enumerate(bolts)] + [(f"M3 nut {i + 1}", n) for i, n in enumerate(nuts)]

    print("\noverlap (cm^3) / smallest gap (mm) between each new part and each body of the model:")
    worst = 0.0
    for i, body in enumerate(shells):
        cells = []
        for nm, part in new[:4] + [("M3 bolts + nuts", rg.m3_bolts() + rg.m3_nuts())]:
            ov = (body ^ part).volume()
            worst = max(worst, ov)
            gap = body.min_gap(part, 40.0)
            cells.append(f"{nm}: " + (f"OVERLAP {ov / 1e3:.3f}" if ov > 1e-3 else (f"{gap:.1f}" if gap < 40 else ">40")))
        print(f"  {name_of(body):42s} " + " | ".join(cells))
    pair = max((((a[1] ^ b[1]).volume(), a[0], b[0]) for k, a in enumerate(new) for b in new[k + 1:]), key=lambda t: t[0])
    print(f"  largest overlap with the model: {worst:.4f} mm^3; between two new parts: {pair[0]:.4f} mm^3 ({pair[1]} / {pair[2]})")
    ov = (fin ^ hb).volume(); g = fin.min_gap(hb, 40.0)
    print(f"  roll fin to base: overlap {ov / 1e3:.3f} cm^3, gap {g:.2f} mm (the O-ring's 0.8 mm; 1 mm to the web's step, 3 mm above the web)")

    bodies = shells + [p for _, p in new]
    out = os.path.join(os.path.dirname(HERE), "stl", "Valve_Assembly_v2_roll_fin.stl")
    write_stl_m(out, bodies)
    print(f"\nwrote {os.path.relpath(out, REPO)}: {len(bodies)} bodies (metres, the model's CAD frame: opens in place over Valve_Assembly_v2)")
    if "--asset" in sys.argv:
        asset = os.path.join(os.path.dirname(MODEL), "Valve_Assembly_v3.stl")
        write_stl_m(asset, bodies)
        print(f"wrote {os.path.relpath(asset, REPO)}: the same file")


if __name__ == "__main__":
    main()
