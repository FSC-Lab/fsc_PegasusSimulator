#!/usr/bin/env python3
"""(2026-10-22) Geometry + marker layout for the marker-layout page, from Valve_Assembly.stl.

    PYTHONNOUSERSITE=1 /usr/bin/python3 marker_page_data.py

Writes ../marker_page_data.json: the three closed shells (plate, bearing, rotor) as indexed meshes
in the valve frame (origin on the bearing axis at the plate-bottom plane, +x along the beam toward
the fin, mm), the rotor's upward faces grouped by height for the top view, and the markers of
marker_layout.py with their pairwise distances.
"""
import importlib.util
import itertools
import json
import os
import struct

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
_spec = importlib.util.spec_from_file_location("ml", os.path.join(HERE, "marker_layout.py"))
ml = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(ml)
OUT = os.path.join(HERE, "..", "marker_page_data.json")
MARKER_D = 12.7          # marker sphere diameter assumed (1/2 in)
BASE_H = 4.0             # marker base height assumed


def main():
    V = ml.load()                                           # (n, 3, 3) valve frame, mm
    flat = V.reshape(-1, 3).round(3)
    uniq, inv = np.unique(flat, axis=0, return_inverse=True)
    F = inv.reshape(-1, 3)
    # connected components over shared vertices = the STL's closed shells
    parent = np.arange(len(uniq))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i
    for a, b, c in F:
        for u, v in ((a, b), (b, c)):
            ru, rv = find(u), find(v)
            if ru != rv:
                parent[ru] = rv
    root = np.array([find(i) for i in range(len(uniq))])
    comp = root[F[:, 0]]
    shells = {}
    for r in np.unique(comp):
        tri = F[comp == r]
        pts = uniq[np.unique(tri)]
        zmax = pts[:, 2].max()
        shells[r] = (tri, zmax, pts)
    # rotor = the tallest (fin top 204 mm), plate = the widest in x, bearing = the rest
    rotor = max(shells, key=lambda r: shells[r][1])
    rest = [r for r in shells if r != rotor]
    plate = max(rest, key=lambda r: np.ptp(shells[r][2][:, 0]))
    bearing = [r for r in rest if r != plate]
    names = {rotor: "rotor", plate: "plate"}
    for r in bearing:
        names[r] = "bearing"
    meshes = {}
    for r, name in names.items():
        tri = shells[r][0]
        used = np.unique(tri)
        remap = -np.ones(len(uniq), int)
        remap[used] = np.arange(len(used))
        P = uniq[used]
        T = remap[tri]
        m = meshes.setdefault(name, {"x": [], "y": [], "z": [], "i": [], "j": [], "k": []})
        off = len(m["x"])
        m["x"] += [round(float(v), 1) for v in P[:, 0]]
        m["y"] += [round(float(v), 1) for v in P[:, 1]]
        m["z"] += [round(float(v), 1) for v in P[:, 2]]
        m["i"] += [int(v) + off for v in T[:, 0]]
        m["j"] += [int(v) + off for v in T[:, 1]]
        m["k"] += [int(v) + off for v in T[:, 2]]
    # top view: the rotor's upward faces, by height
    tri = shells[rotor][0]
    P3 = uniq[tri]
    N = np.cross(P3[:, 1] - P3[:, 0], P3[:, 2] - P3[:, 0])
    N /= np.linalg.norm(N, axis=1, keepdims=True) + 1e-12
    up = N[:, 2] > 0.99
    levels = {}
    for t, z in zip(P3[up], P3[up][:, :, 2].mean(1)):
        levels.setdefault(f"{z:.1f}", []).append([round(float(c), 2) for c in t[:, :2].ravel()])
    markers = []
    for name, x, y, h in ml.MARKERS + [ml.M5]:
        zc = ml.BEAM_TOP + h + BASE_H + MARKER_D / 2
        markers.append({"name": name, "x": x, "y": y, "post": h, "z_base": ml.BEAM_TOP + h, "z_centre": zc,
                        "r_axis": round(float(np.hypot(x, y)), 1), "plans": [5] if name == "M5" else [4, 5]})
    C = {m["name"]: np.array([m["x"], m["y"], m["z_centre"]]) for m in markers}
    dist = {}
    for n in (4, 5):
        names = [m["name"] for m in markers if n in m["plans"]]
        dist[str(n)] = sorted(({"a": a, "b": b, "d": round(float(np.linalg.norm(C[a] - C[b])), 1)}
                               for a, b in itertools.combinations(names, 2)), key=lambda e: e["d"])
    out = {"meshes": meshes, "top_levels": levels, "markers": markers, "distances": dist,
           "marker_d": MARKER_D, "base_h": BASE_H, "base_d": ml.BASE_D, "beam_top": ml.BEAM_TOP}
    with open(OUT, "w") as f:
        json.dump(out, f, separators=(",", ":"))
    print("wrote", os.path.abspath(OUT), os.path.getsize(OUT), "bytes")
    for name, m in meshes.items():
        print(f"  {name}: {len(m['x'])} vertices, {len(m['i'])} triangles, z {min(m['z'])}..{max(m['z'])}, "
              f"x {min(m['x'])}..{max(m['x'])}")
    print("  top levels:", {k: len(v) for k, v in sorted(levels.items(), key=lambda kv: float(kv[0]))})
    for n, dd in dist.items():
        print(f"  {n}-marker distances:", [(e["a"] + "-" + e["b"], e["d"]) for e in dd])


if __name__ == "__main__":
    main()
