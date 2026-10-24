"""2026-10-19 question: is Drop Margin 0.15 m safe, from the 10-16 flights flown at 0.20?

For D-1 and D-2, over Ready To Drop (execute_place) up to the release: the basket's LOWEST point (its mocap
pose applied to the CAD's low points -- bottom corners, the two side marker balls at bottom height, the
front/back balls and brackets; Box_Payload_Assembly.usda, origin = basket centre) against the plane the
basket's bottom stood on at the Drop row's Get (Get z - 32.5 mm = the cover, ~ the box rim). Binned by the
basket's horizontal distance from the drop point, so the rim crossing is visible. The 0.15 column shifts the
measured track by the planner's change: the drop rest 0.05 m lower, Drop Start unchanged, so a straight leg
is lowered by 0.05 * s, s = the fraction of the leg's horizontal travel done.
usage: drop_margin_check.py [run ...]   (default d1 d2)"""
import sys
import numpy as np
from pd15_common import *   # noqa: F401,F403
from pp_common import quat_R
from drop16 import drop_point

HALF_H = 0.0325
# low points of the payload, obj_0 frame [m]: corners of the basket's bottom; spheres (centre, radius)
CORNERS = np.array([[sx * 0.055, sy * 0.055, -HALF_H] for sx in (-1, 1) for sy in (-1, 1)])
BRACKETS = np.array([[sx * x, sy * 0.048, 0.0275] for sx in (-1, 1) for x in (0.055, 0.0775) for sy in (-1, 1)])
BALLS = [(np.array([0.0, 0.138, -0.0175]), 0.015), (np.array([0.0, -0.138, -0.0175]), 0.015),
         (np.array([0.0985, 0.0, 0.030]), 0.015), (np.array([-0.0985, 0.0, 0.030]), 0.015)]


def lowest(P, Q):
    R = quat_R(*Q)          # (w, x, y, z)
    z = [P[2] + (R @ c)[2] for c in np.vstack([CORNERS, BRACKETS])]
    z += [P[2] + (R @ c)[2] - r for c, r in BALLS]
    return min(z)


def main(run):
    d, t0 = load(run)
    win = leg_windows(d, t0)
    pt, ee_t, yaw = drop_point(d)
    to, Po, Qo = obj_pose(d, t0)
    a, b = win["execute_place (approach)"]
    # release: the basket's first fast downward sample after the leg (drop16: +0.08 / +0.12 s after the click)
    vz = np.gradient(Po[:, 2], to)
    after = np.where((to > b) & (vz < -0.3))[0]
    t_rel = to[after[0]] - 0.05
    m = (to >= a) & (to <= t_rel)
    plane = pt[2] - HALF_H
    low = np.array([lowest(p, q) for p, q in zip(Po[m], Qo[m])]) - plane
    ctr = Po[m, 2] - pt[2]
    dist = np.linalg.norm(Po[m, :2] - pt[:2], axis=1)
    s = np.clip(1.0 - dist / dist[0], 0.0, 1.0)
    low15 = low - 0.05 * s
    tilt = tilt_of(Qo[m])
    print(f"\n===== {run} {RUNS[run][0]}  drop point (cover Get) z {pt[2]:.3f}  leg {a:.1f}-{b:.1f} s, release {t_rel + 0.05:.2f} s")
    print(f"  basket centre above its cover height at release: {ctr[-1] * 1e3:.0f} mm (margin 200)")
    print("  basket xy from the drop point | n | lowest point above the cover plane, mm: min @0.20 -> @0.15 | tilt max deg")
    for lo, hi in [(0.60, 9), (0.40, 0.60), (0.30, 0.40), (0.25, 0.30), (0.20, 0.25), (0.15, 0.20), (0.10, 0.15),
                   (0.05, 0.10), (0.0, 0.05)]:
        k = (dist >= lo) & (dist < hi)
        if k.any():
            print(f"  {lo * 1e3:4.0f}-{min(hi, 9.99) * 1e3:4.0f} mm | {k.sum():4d} | {low[k].min() * 1e3:5.0f} -> {low15[k].min() * 1e3:5.0f}"
                  f" | {tilt[k].max():4.1f}")
    hold = to[m] >= b
    print(f"  hold (leg end -> release, {hold.sum()} samples): lowest {low[hold].min() * 1e3:.0f} -> {low15[hold].min() * 1e3:.0f} mm,"
          f" centre z std {ctr[hold].std() * 1e3:.1f} mm, tilt {tilt[hold].mean():.1f} deg mean")


if __name__ == "__main__":
    for r in (sys.argv[1:] or ["d1", "d2"]):
        main(r)
