#!/usr/bin/env python3
"""Ready To Place / Exit To Place profile of one pick-and-drop run (2026-10-14).

    drop_leg_profile.py runs/drop6.npz [runs/drop5.npz ...]

Per run: the body altitude against the fraction of the leg's horizontal path,
the vertical speed at the end of the climb (a step-like climb stops the drone
there), the payload's lowest bottom while any part of it is over the bin's outer
rectangle, and where the payload came to rest. The rim: DROP_RIM [m], default
0.813 (the 35 mm tray of 2026-10-15); the 172 mm box of drop1..drop7 was 0.950.
"""
import os
import sys

import numpy as np

BIN_C = np.array([-1.0, -1.0])      # bin centre on the place hat
BIN_HALF = np.array([0.145, 0.1855])  # turned 90 deg: 290 mm along x, 371 mm along y
PAY_HALF = np.array([0.1135, 0.153])  # payload footprint with its marker balls (227 x 306 mm)
PAY_BOTTOM = 0.0325                 # payload origin above its bottom (rests at 0.813 on a 0.7805 floor)
RIM = float(os.environ.get("DROP_RIM", "0.813"))


def mark(d, name):
    i = list(d["marks_name"]).index(name)
    return d["marks_t"][i]


def leg(d, a, b):
    t0, t1 = mark(d, a), mark(d, b)
    o = d["odom"]
    m = (o[:, 0] >= t0) & (o[:, 0] <= t1)
    return o[m], t0, t1


def profile(path):
    d = np.load(path, allow_pickle=True)
    print(f"== {path}")
    o, t0, t1 = leg(d, "ready_place:start", "ready_place:end")
    xy, z = o[:, 1:3], o[:, 3]
    chord = xy[-1] - xy[0]           # progress along the start->end chord (arc length on
    L = np.linalg.norm(chord)        # noisy odometry integrates the noise)
    frac = (xy - xy[0]) @ (chord / L) / L
    vz = np.gradient(z, o[:, 0])
    vxy = np.linalg.norm(np.gradient(xy, o[:, 0], axis=0), axis=1)
    print(f"Ready To Place {t1 - t0:.1f} s, body z {z[0]:.3f} -> {z[-1]:.3f}, chord {L:.3f} m")
    for f in (0.1, 0.25, 0.5, 0.65, 0.84, 1.0):
        k = int(np.argmax(frac >= f - 1e-9)) if (frac >= f - 1e-9).any() else len(z) - 1
        print(f"  {f:4.0%} of the path: t {o[k, 0] - t0:5.1f} s  z {z[k]:.3f}  vz {vz[k]:+.3f}  vxy {vxy[k]:.3f}")
    top = np.argmax(z >= z[-1] - 0.01)
    print(f"  climb within 1 cm of the drop altitude at {o[top, 0] - t0:.1f} s, "
          f"{frac[top]:.0%} of the path")
    mid = (o[:, 0] > t0 + 0.15 * (t1 - t0)) & (o[:, 0] < t1 - 0.15 * (t1 - t0))
    sp = np.hypot(vz, vxy)[mid]
    print(f"  slowest speed in the middle 70 %: {sp.min():.3f} m/s")

    p = d["payload"]
    pm = (p[:, 0] >= t0) & (p[:, 0] <= mark(d, "gripper_open:start"))
    over = np.all(np.abs(p[pm, 1:3] - BIN_C) < BIN_HALF + PAY_HALF, axis=1)
    if over.any():
        low = (p[pm, 3] - PAY_BOTTOM)[over].min()
        print(f"  payload bottom over the bin's outline: min {low:.3f} m ({low - RIM:+.3f} vs the rim)")
    else:
        print("  payload never over the bin's outline before the release")

    o2, e0, e1 = leg(d, "exit_place:start", "exit_place:end")
    print(f"Exit To Place {e1 - e0:.1f} s, body end ({o2[-1, 1]:.3f}, {o2[-1, 2]:.3f}, {o2[-1, 3]:.3f}), "
          f"max z {o2[:, 3].max():.3f}")
    w, qx, qy, qz = p[-1, 4:8]       # the scorer's convention: payload rows are t, x, y, z, w, x, y, z
    tilt = np.degrees(np.arccos(np.clip(1 - 2 * (qx * qx + qy * qy), -1, 1)))
    print(f"payload final ({p[-1, 1]:.3f}, {p[-1, 2]:.3f}, {p[-1, 3]:.3f}), tilt {tilt:.1f} deg, "
          f"aborted {bool(d['aborted'])}")


if __name__ == "__main__":
    for p in sys.argv[1:]:
        profile(p)
