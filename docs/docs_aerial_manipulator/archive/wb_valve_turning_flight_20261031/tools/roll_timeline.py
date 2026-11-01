#!/usr/bin/env python3
"""1 s bins from Go To Start to Ready + 1 s: where the vehicle is, what the arm is doing, and the roll channel.
    PYTHONNOUSERSITE=1 /usr/bin/python3 roll_timeline.py 180423 211808"""
import sys
import numpy as np
import vt_common as V

for tag in sys.argv[1:]:
    b = V.Bag(tag)
    ed = b.edges("pl_pstatus")
    tg = [t for t, s in ed if s.startswith("FLYING go_to_start")][0]
    th = [t for t, s in ed if s.startswith("DONE go_to_start")][0]
    tr = [t for t, s in ed if s.startswith("FLYING ready")][0]
    tw = b.t("wb"); W = b.f("wb", "data")
    ok = np.array([len(r) >= 118 for r in W]); Wm = np.array([np.asarray(r[:118], float) for r in W[ok]]); tw = tw[ok]
    tm = b.t("mocap"); g = lambda k: b.f("mocap", k)
    qw, qx, qy, qz = (g(f"pose.orientation.{c}") for c in "wxyz")
    roll = np.degrees(np.arctan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx ** 2 + qy ** 2)))
    pitch = np.degrees(np.arcsin(np.clip(2 * (qw * qy - qz * qx), -1, 1)))
    yaw = np.degrees(np.unwrap(np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy ** 2 + qz ** 2))))
    print(f"\n### {tag}: Go To Start {tg:.2f}, hold {th:.2f}, Ready {tr:.2f}")
    print("  t-hold    x      y     z   yaw | q1   q2   q3   q4 deg | roll pitch deg (mocap) | eR roll mrad (mean/sd) | d_roll filt | tau roll cmd | d_yaw  d_pitch")
    for a in np.arange(tg - 2, tr + 1.01, 1.0):
        m = (tw >= a) & (tw < a + 1); mm = (tm >= a) & (tm < a + 1)
        if not m.any():
            continue
        X = Wm[m]
        print(f"  {a - th:+6.1f}  {X[:,48].mean():+.2f} {X[:,49].mean():+.2f} {X[:,50].mean():.2f} {yaw[mm].mean():+6.1f} | "
              + " ".join(f"{np.degrees(X[:,5+i]).mean():+4.0f}" for i in range(4))
              + f" | {roll[mm].mean():+5.2f} {pitch[mm].mean():+5.2f}             | {1e3*X[:,29].mean():+6.1f} {1e3*X[:,29].std():5.1f}          | {X[:,35].mean():+.3f}      | {X[:,21].mean():+.3f}       | {X[:,36].mean():+.3f} {X[:,34].mean():+.3f}")
