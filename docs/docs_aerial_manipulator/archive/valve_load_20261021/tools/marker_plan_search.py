#!/usr/bin/env python3
"""(2026-10-22) Search the 4- and 5-marker plans on the valve rotor's flat top (z = 50 mm).

    PYTHONNOUSERSITE=1 /usr/bin/python3 marker_plan_search.py

A base fits only if a disk of BASE_D (+1 mm margin) lies entirely on the CAD's z = 50 faces (so it
clears the axis hole, the rounded back end, the side edges and the raised bracket). The 4-marker plan
is searched in four zones (one marker on a 30 mm post, both sides of the beam, spread along it); the
5-marker plan keeps those four and adds one. Score: the smallest gap between any two pairwise
distances (Motive labels markers by their distances), then the spread. Prints both plans.
"""
import itertools
import os
import struct

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
STL = os.path.join(HERE, "..", "..", "..", "..", "..", "extensions", "fsc_aerial_manipulation",
                   "fsc_aerial_manipulation", "rotorcraft", "assets", "Valve_Assembly.stl")
BASE_D, MARGIN, POST = 12.0, 2.5, 30.0   # 2.5 mm: a marker placed +-2 mm off still sits on the flat top
MARKER_D, BASE_H, TOP = 12.7, 4.0, 50.0
MIN_SPACING = 30.0
rng = np.random.default_rng(0)


def flat_top_triangles():
    raw = open(STL, "rb").read()
    n = struct.unpack("<I", raw[80:84])[0]
    a = np.frombuffer(raw[84:84 + n * 50], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
    V = a["v"].reshape(-1, 3, 3).astype(float)
    if abs(V).max() < 5:
        V *= 1000.0
    V[:, :, 0] += 1.1
    V[:, :, 1] -= 3.0
    V[:, :, 2] += 4.5
    N = np.cross(V[:, 1] - V[:, 0], V[:, 2] - V[:, 0])
    N /= np.linalg.norm(N, axis=1, keepdims=True) + 1e-12
    sel = (N[:, 2] > 0.99) & (np.abs(V[:, :, 2].mean(1) - TOP) < 0.2)
    return V[sel][:, :, :2]


T = flat_top_triangles()
_ring = [(0.0, 0.0)] + [(r * np.cos(t), r * np.sin(t)) for r in (BASE_D / 2 + MARGIN, (BASE_D / 2 + MARGIN) / 2)
                        for t in np.linspace(0, 2 * np.pi, 24, endpoint=False)]


def on_top(px, py):
    a, b, c = T[:, 0], T[:, 1], T[:, 2]
    d1 = (px - b[:, 0]) * (a[:, 1] - b[:, 1]) - (a[:, 0] - b[:, 0]) * (py - b[:, 1])
    d2 = (px - c[:, 0]) * (b[:, 1] - c[:, 1]) - (b[:, 0] - c[:, 0]) * (py - c[:, 1])
    d3 = (px - a[:, 0]) * (c[:, 1] - a[:, 1]) - (c[:, 0] - a[:, 0]) * (py - a[:, 1])
    neg = (d1 < -1e-9) | (d2 < -1e-9) | (d3 < -1e-9)
    pos = (d1 > 1e-9) | (d2 > 1e-9) | (d3 > 1e-9)
    return bool(np.any(~(neg & pos)))


def fits(x, y):
    return all(on_top(x + dx, y + dy) for dx, dy in _ring)


def grid(xs, ys):
    return [(x, y) for x in xs for y in ys if fits(x, y)]


def centre(p):
    x, y, h = p
    return np.array([x, y, TOP + h + BASE_H + MARKER_D / 2])


def score(points):
    C = [centre(p) for p in points]
    d = sorted(np.linalg.norm(a - b) for a, b in itertools.combinations(C, 2))
    # spacing measured FLAT: a post's base occupies the table under its marker, and two markers
    # stacked one above the other merge in a camera looking down
    if min(np.linalg.norm(a[:2] - b[:2]) for a, b in itertools.combinations(C, 2)) < MIN_SPACING:
        return None
    gap = min(np.diff(d))
    P = np.array(C)[:, :2]
    spread = np.linalg.svd(P - P.mean(0), compute_uv=False)[-1]      # thinness across the beam
    return gap, spread, d


def main():
    zones = {  # (xs, ys, post height)
        "M1": (np.arange(-8, 20, 1.0), np.arange(12, 20, 0.5), 0.0),
        "M2": (np.arange(18, 48, 2.0), np.arange(-18, -8, 1.0), POST),
        "M3": (np.arange(78, 108, 2.0), np.arange(8, 19, 1.0), 0.0),
        "M4": (np.arange(112, 132, 1.0), np.arange(-18, -4, 1.0), 0.0),
    }
    cand = {k: [(x, y, h) for x, y in grid(xs, ys)] for k, (xs, ys, h) in zones.items()}
    for k, v in cand.items():
        print(f"{k}: {len(v)} positions fit")
    best = None
    for _ in range(60000):
        pts = [cand[k][rng.integers(len(cand[k]))] for k in ("M1", "M2", "M3", "M4")]
        s = score(pts)
        if s is None:
            continue
        key = (round(s[0], 1), round(s[1], 1))
        if best is None or key > best[0]:
            best = (key, pts, s[2])
    (gap4, spread4), plan4, d4 = best
    print("\n4-marker plan  (x, y, post) mm:")
    for k, p in zip(("M1", "M2", "M3", "M4"), plan4):
        print(f"  {k}: x {p[0]:+6.1f}  y {p[1]:+6.1f}  post {p[2]:.0f}")
    print(f"  distances {[round(v, 1) for v in d4]}  smallest gap {gap4} mm  cross-beam spread {spread4}")
    # the fifth: anywhere on the flat top (both sides), flat, keeping the four
    xs, ys = np.arange(-8, 132, 1.0), np.arange(-18, 19, 1.0)
    best5 = None
    for x, y in grid(xs, ys):
        pts = plan4 + [(x, y, 0.0)]
        s = score(pts)
        if s is None:
            continue
        key = (round(s[0], 1), round(s[1], 1))
        if best5 is None or key > best5[0]:
            best5 = (key, (x, y, 0.0), s[2])
    (gap5, spread5), m5, d5 = best5
    print(f"\n5-marker plan = the four + M5 at x {m5[0]:+.1f}  y {m5[1]:+.1f} (flat)")
    print(f"  distances {[round(v, 1) for v in d5]}  smallest gap {gap5} mm  cross-beam spread {spread5}")


def joint(n=250000):
    """Both plans at once: M1..M4 = the 4-marker plan, M1..M5 the 5-marker plan; maximise the
    5-marker gap while keeping the 4-marker gap at least 10 mm."""
    zones = {
        "M1": (np.arange(-8, 20, 1.0), np.arange(12, 20, 0.5), 0.0),
        "M2": (np.arange(18, 48, 2.0), np.arange(-18, -8, 1.0), POST),
        "M3": (np.arange(78, 108, 2.0), np.arange(8, 19, 1.0), 0.0),
        "M4": (np.arange(112, 132, 1.0), np.arange(-18, -4, 1.0), 0.0),
        "M5": (np.arange(50, 90, 2.0), np.arange(-18, 19, 1.0), 0.0),
    }
    cand = {k: [(x, y, h) for x, y in grid(xs, ys)] for k, (xs, ys, h) in zones.items()}
    best = None
    for _ in range(n):
        pts = [cand[k][rng.integers(len(cand[k]))] for k in ("M1", "M2", "M3", "M4", "M5")]
        s4, s5 = score(pts[:4]), score(pts)
        if s4 is None or s5 is None or s4[0] < 10.0:
            continue
        key = (round(min(s5[0], s4[0]), 1), round(s4[0], 1), round(s5[1], 1))
        if best is None or key > best[0]:
            best = (key, pts, s4, s5)
    key, pts, s4, s5 = best
    print("\nJOINT search (5 = 4 + M5):")
    for k, p in zip(("M1", "M2", "M3", "M4", "M5"), pts):
        print(f"  {k}: x {p[0]:+6.1f}  y {p[1]:+6.1f}  post {p[2]:.0f}")
    print(f"  4 markers: distances {[round(v, 1) for v in s4[2]]}  smallest gap {s4[0]:.1f} mm")
    print(f"  5 markers: distances {[round(v, 1) for v in s5[2]]}  smallest gap {s5[0]:.1f} mm")


if __name__ == "__main__":
    import sys
    joint() if "--joint" in sys.argv else main()
