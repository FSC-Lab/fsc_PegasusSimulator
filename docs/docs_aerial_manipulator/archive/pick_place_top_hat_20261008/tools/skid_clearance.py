#!/usr/bin/env python3
"""Landing-gear skids vs the pillar platform, over recorded pick-and-place runs (2026-10-08).

    PYTHONNOUSERSITE=1 /usr/bin/python3 skid_clearance.py <run.npz> ... [--top 1.0]

For every odometry sample of the DIRECT flight: the two skid boxes (body frame,
AM_xfwd.usda, as in pick_place_box_payload_20261007/tools/hook_score.py),
sampled on their surface, against the platform on EACH pillar, relative to the
platform top the run flew over (--top, 1.0 m for the 10-07 runs):
  cap   the 10-03 disc, r 80 mm, 10 mm thick
  hat   Top_Hat.STL: the platform r 100.29 mm, 5 mm thick, on the r 61 mm
        sleeve that runs 55 mm further down
Prints the minimum gap per run (and where it happened), per platform model.
"""
import argparse

import numpy as np

SKID_X, SKID_Y, SKID_Z = (-0.163, 0.163), (0.142, 0.168), (-0.3125, -0.275)
PILLARS = {"pick": (1.0, 1.0), "place": (-1.0, -1.0)}
MODELS = {   # (radius, z_lo, z_hi) relative to the platform top
    "cap": [(0.080, -0.010, 0.0)],
    "hat": [(0.10029, -0.005, 0.0), (0.061, -0.060, -0.005)],
}


def quat_R(w, x, y, z):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def skid_points():
    xs = np.linspace(*SKID_X, 41)
    pts = []
    for s in (1, -1):
        ys = s * np.array(SKID_Y)
        for y in np.linspace(ys.min(), ys.max(), 4):
            for z in np.linspace(*SKID_Z, 4):
                pts += [[x, y, z] for x in xs]
    return np.array(pts)


def cyl_gap(P, c, r, z0, z1):
    """Distance from points P (n,3) to a solid z-axis cylinder (0 inside)."""
    dr = np.maximum(0.0, np.linalg.norm(P[:, :2] - c, axis=1) - r)
    dz = np.maximum(0.0, np.maximum(z0 - P[:, 2], P[:, 2] - z1))
    return np.hypot(dr, dz)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("npz", nargs="+")
    ap.add_argument("--top", type=float, default=1.0)
    a = ap.parse_args()
    S = skid_points()
    for path in a.npz:
        d = np.load(path, allow_pickle=True)
        names, mt = list(d["marks_name"]), d["marks_t"]
        od = d["odom"]
        t0 = mt[names.index("direct")] if "direct" in names else od[0, 0]
        od = od[(od[:, 0] >= t0) & (od[:, 0] <= mt[-1])][::2]
        res = {m: (np.inf, None) for m in MODELS}
        for row in od:
            P = row[1:4] + S @ quat_R(*row[7:11]).T
            for m, parts in MODELS.items():
                for pname, c in PILLARS.items():
                    for r, z0, z1 in parts:
                        g = cyl_gap(P, np.array(c), r, a.top + z0, a.top + z1).min()
                        if g < res[m][0]:
                            res[m] = (g, (pname, round(float(row[0]), 1), np.round(row[1:4], 3).tolist()))
        print(path.split("/")[-1], "  ".join(f"{m}: {1e3 * g:.0f} mm at {w}" for m, (g, w) in res.items()))


if __name__ == "__main__":
    main()
