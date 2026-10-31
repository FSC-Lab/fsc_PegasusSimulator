#!/usr/bin/env python3
"""The landing-gear skids against the tray, over recorded pick-and-drop runs (2026-10-26).

    PYTHONNOUSERSITE=1 /usr/bin/python3 tray_clearance.py <run.npz> ...

For every odometry sample from the start of Ready To Drop to the end of Exit To Drop, the two
skid boxes (body frame, AM_xfwd.usda, as in pick_place_top_hat_20261008/tools/skid_clearance.py)
sampled on their surface, against the tray (the 2026-10-15 default: TURNED 90 deg, 290 mm along
x, 371 along y, centred on the place hat (-1, -1), rim 0.813 m) taken as a solid box from the
floor to the rim. Prints the smallest gap and where it happened, plus the body height at the
open. The payload's own clearance over the tray is drop_score.py's.

The simulated gear is AM_xfwd's: its skids reach 0.3125 m below the body origin, ~65 mm lower
than the real T650 gear (0.248 m measured on 2026-10-20), so a gap here is a lower bound for
the hardware.
"""
import sys

import numpy as np

SKID_X, SKID_Y, SKID_Z = (-0.163, 0.163), (0.142, 0.168), (-0.3125, -0.275)
TRAY_C, TRAY_HALF, RIM = np.array([-1.0, -1.0]), np.array([0.145, 0.1855]), 0.813


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


def box_gap(P):
    """Distance from points P (n,3) to the solid tray box (floor .. rim), 0 inside."""
    d = np.maximum(0.0, np.abs(P[:, :2] - TRAY_C) - TRAY_HALF)
    dz = np.maximum(0.0, P[:, 2] - RIM)
    return np.sqrt((d ** 2).sum(axis=1) + dz ** 2)


def main():
    S = skid_points()
    for path in sys.argv[1:]:
        d = np.load(path, allow_pickle=True)
        names, mt = list(d["marks_name"]), d["marks_t"]
        mark = lambda n: mt[names.index(n)] if n in names else None
        t0, t1, t_open = mark("ready_place:start"), mark("exit_place:end"), mark("gripper_open:start")
        if t0 is None:
            print(path.split("/")[-1], ": Ready To Drop never started")
            continue
        od = d["odom"]
        od = od[(od[:, 0] >= t0) & (od[:, 0] <= (t1 if t1 is not None else od[-1, 0]))]
        best = (np.inf, None)
        for row in od:
            g = box_gap(row[1:4] + S @ quat_R(*row[7:11]).T).min()
            if g < best[0]:
                best = (g, (round(float(row[0] - t0), 1), np.round(row[1:4], 3).tolist()))
        zb = np.interp(t_open, d["odom"][:, 0], d["odom"][:, 3]) if t_open is not None else float("nan")
        print(f"{path.split('/')[-1]}: skids vs the tray, Ready To Drop .. Exit To Drop: {best[0] * 1e3:.0f} mm "
              f"(at +{best[1][0]} s into Ready To Drop, body {best[1][1]}); body z at the open {zb:.3f} m")


if __name__ == "__main__":
    main()
