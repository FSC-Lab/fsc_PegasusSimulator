#!/usr/bin/env python3
"""THE MARKER HOLDERS, TAKEN FROM THE USER'S VALVE CAD (2026-10-29): splits
Valve_Assembly_v2.stl into its closed shells, finds the five marker parts (a
30 mm ball on a flat tab) and the holder each one sits in, sorts the holders
into the four designs by their size, and writes each design once, in its own
frame, for the localization frame (make_localization_frame.py) and the drawing
page:

    /usr/bin/python3 extract_marker_holders.py [--out DIR]
      -> holders/holder_<T>.stl, holders/marker_<T>.stl, holders/holders.json,
         holders/valve_context.stl (the valve less its holders and markers, for the page)

The four designs (T): a flat 2 mm base with a tower carrying the marker's tab
on two cross pins; they differ in the tower and so in the ball's height and
offset. Frame of each written part: origin at the CENTRE OF THE BASE'S BOTTOM
FACE, axes as in the valve CAD (z up). The ball centre and the base footprint
in that frame go into holders.json.

    A  base 20 x 30, tower 52 tall, ball 10 mm to -y,  73.5 above the base bottom
    B  base 30 x 20, tower 22 tall, ball 10 mm to -x,  42.5
    C  base 15 x 30, tower 22 tall, ball 10 mm to +y,  42.5
    D  base 20 x 30, tower 13.5,    ball 33 mm to +y,  18.5 (the tab lies flat)

Pure numpy; the STL reader, the shell split and the sphere fit are the valve
converter's (archive/valve_turning_20261015/tools/stl_to_usda_valve.py),
loaded without its USD imports.
"""
import argparse
import json
import os
import struct

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", "..", "..", ".."))
VALVE_STL = os.path.join(REPO, "extensions/fsc_aerial_manipulation/fsc_aerial_manipulation/rotorcraft/assets/"
                         "Valve_Assembly_v2.stl")
CONVERTER = os.path.join(REPO, "docs/docs_aerial_manipulator/archive/valve_turning_20261015/tools/stl_to_usda_valve.py")
# the designs by (tower height, base x, base y) [mm], CAD orientation
DESIGNS = {"A": (52.0, 20.1, 30.1), "B": (22.0, 30.1, 20.1), "C": (22.0, 15.1, 30.1), "D": (13.5, 20.1, 30.1)}


def converter():
    src = open(CONVERTER).read().replace("from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics", "")
    ns = {"__file__": CONVERTER}
    exec(compile(src, CONVERTER, "exec"), ns)
    return ns


def ball_fit(M, c0, r0=15.0):
    V = np.unique(np.round(M.reshape(-1, 3), 4), axis=0)
    c = np.array(c0, float)
    for _ in range(8):
        P = V[np.abs(np.linalg.norm(V - c, axis=1) - r0) < 1.0]
        sol = np.linalg.lstsq(np.c_[2 * P, np.ones(len(P))], (P ** 2).sum(1), rcond=None)[0]
        c = sol[:3]
        r = float(np.sqrt(sol[3] + c @ c))
    return c, r


def box_gap(a, b):
    lo1, hi1 = a.reshape(-1, 3).min(0), a.reshape(-1, 3).max(0)
    lo2, hi2 = b.reshape(-1, 3).min(0), b.reshape(-1, 3).max(0)
    return float(np.linalg.norm(np.maximum(0, np.maximum(lo1 - hi2, lo2 - hi1))))


def write_stl(tris, path, name):
    tris = np.asarray(tris, dtype=np.float64)
    with open(path, "wb") as f:
        f.write(name.encode()[:80].ljust(80, b" "))
        f.write(struct.pack("<I", len(tris)))
        for a, b, c in tris:
            n = np.cross(b - a, c - a)
            n = n / (np.linalg.norm(n) or 1.0)
            f.write(struct.pack("<12fH", *n, *a, *b, *c, 0))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.join(HERE, "holders"))
    a = ap.parse_args()
    ns = converter()
    T = ns["read_stl"](VALVE_STL) * 1e3                     # the converter reads metres
    lab = ns["shells"](T)
    shells = {k: T[lab == k] for k in np.unique(lab)}
    markers = {k: S for k, S in shells.items() if ns["marker_ball"](S) is not None}
    if len(markers) < 4:
        raise SystemExit(f"expected the valve CAD's 5 marker parts, found {len(markers)}")
    rest = {k: S for k, S in shells.items() if k not in markers}
    out, used = {}, set()
    valve = dict(holders=[], markers=[])         # where each part sits on the valve (valve-CAD axes, mm)
    for km, M in markers.items():
        # the holder: the small part nearest the marker's tab
        cand = [(box_gap(M, S), k) for k, S in rest.items()
                if np.ptp(S.reshape(-1, 3), axis=0).max() < 60.0 and k not in used]
        gap, kh = min(cand)
        if gap > 1.0:
            # (the valve's fifth marker sits on a mount that is part of the rotor itself)
            print(f"marker shell {km}: no separate holder touches it (nearest {gap:.1f} mm) -- skipped")
            c, _ = ball_fit(M, ns["marker_ball"](M))
            valve["markers"].append(dict(holder=None, ball=c.round(3).tolist()))
            continue
        used.add(kh)
        H = rest[kh]
        V = H.reshape(-1, 3)
        z0 = V[:, 2].min()
        base = V[np.abs(V[:, 2] - z0) < 0.01]
        bc = np.array([0.5 * (base[:, 0].min() + base[:, 0].max()), 0.5 * (base[:, 1].min() + base[:, 1].max()), z0])
        size = (float(np.ptp(base[:, 0])), float(np.ptp(base[:, 1])))
        height = float(V[:, 2].max() - z0)
        t = min(DESIGNS, key=lambda d: abs(DESIGNS[d][0] - height) + abs(DESIGNS[d][1] - size[0])
                + abs(DESIGNS[d][2] - size[1]))
        d = DESIGNS[t]
        if abs(d[0] - height) + abs(d[1] - size[0]) + abs(d[2] - size[1]) > 1.0:
            raise SystemExit(f"holder {kh}: height {height:.2f}, base {size} matches no known design")
        c, r = ball_fit(M, ns["marker_ball"](M))
        valve["holders"].append(dict(t=t, base_centre=bc.round(3).tolist()))
        valve["markers"].append(dict(holder=t, ball=c.round(3).tolist()))
        rec = dict(base_size=[round(size[0], 3), round(size[1], 3)], holder_height=round(height, 3),
                   ball=(c - bc).round(3).tolist(), ball_r=round(r, 3), valve_cad_base_centre=bc.round(3).tolist(),
                   holder_triangles=len(H), marker_triangles=len(M))
        if t in out:                                        # a design used twice on the valve: must agree
            if np.abs(np.array(out[t]["ball"]) - rec["ball"]).max() > 0.05:
                raise SystemExit(f"two holders of design {t} carry their balls differently")
            continue
        out[t] = rec
        os.makedirs(a.out, exist_ok=True)
        write_stl(H - bc, os.path.join(a.out, f"holder_{t}.stl"), f"marker holder {t} (from Valve_Assembly_v2), mm")
        write_stl(M - bc, os.path.join(a.out, f"marker_{t}.stl"), f"marker on holder {t} (30 mm ball + tab), mm")
    if sorted(out) != sorted(DESIGNS):
        raise SystemExit(f"found designs {sorted(out)}, expected {sorted(DESIGNS)}")
    # the rest of the valve (plate, bearing, pad rings, rotor with its built-in fifth mount), for the
    # drawing page's "which holder is which" view: the holders and markers are drawn from their own files
    ctx = np.concatenate([S for k, S in rest.items() if k not in used])
    write_stl(ctx, os.path.join(a.out, "valve_context.stl"), "Valve_Assembly_v2 less its holders and markers, mm")
    valve["holders"].sort(key=lambda h: h["t"])
    with open(os.path.join(a.out, "holders.json"), "w") as f:
        json.dump(dict(source=os.path.relpath(VALVE_STL, REPO), frame="origin = centre of the base's bottom face, "
                       "valve-CAD axes, z up, mm", designs=dict(sorted(out.items())),
                       valve=dict(valve, context_triangles=len(ctx),
                                  note="valve-CAD axes, mm; each holder sits there unturned (its own frame = the "
                                       "valve's axes); holder null = the rotor's built-in mount")), f, indent=1)
    for t, rec in sorted(out.items()):
        print(f"{t}: base {rec['base_size'][0]:.2f} x {rec['base_size'][1]:.2f}, height {rec['holder_height']:.1f}, "
              f"ball at {rec['ball']} (r {rec['ball_r']:.2f})")
    print(f"-> {a.out}")


if __name__ == "__main__":
    main()
