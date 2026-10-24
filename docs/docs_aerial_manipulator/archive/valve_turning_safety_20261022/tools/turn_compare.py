#!/usr/bin/env python3
"""(2026-10-22) The turn, bin by bin, on a FIXED window: the push's start mark + the 1.5 s
settle, then the planned 36 s turn (campaign_score.py's "fast half" uses the push's end mark,
which an aborted run never gets, so its window runs past the abort). Per 4.5 s bin: the
vehicle's tilt (max), the law's standing pitch error e_R about model x (mean, mrad), the CoM's
along-nose offset (mean, mm), q2 min / q3 min..max (deg). A run is a path to an npz.
    PYTHONNOUSERSITE=1 /usr/bin/python3 turn_compare.py run.npz [run.npz ...]
"""
import os
import sys

import numpy as np

SETTLE, TURN, BIN = 1.5, 36.0, 4.5


def q2R(q):
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def tilt(q):
    return np.degrees(np.arccos(np.clip(1 - 2 * (q[:, 1] ** 2 + q[:, 2] ** 2), -1, 1)))


def bins(path):
    d = np.load(path, allow_pickle=True)
    marks = dict(zip(d["marks_name"].tolist(), d["marks_t"].tolist()))
    od, J, dbg = d["odom"], d["joints"], d["dbg"]
    t0 = marks["push:start"] + SETTLE
    dire = od[:, 11] > 0.5
    tend = od[dire, 0].max()
    rows = []
    for k in range(int(TURN / BIN) + 1):
        a, b = t0 + k * BIN, t0 + min((k + 1) * BIN, TURN + 2.0 if k == int(TURN / BIN) else (k + 1) * BIN)
        if a > tend:
            break
        b = min(b, tend)
        mo = (od[:, 0] >= a) & (od[:, 0] < b) & dire
        mj = (J[:, 0] >= a) & (J[:, 0] < b)
        md = (dbg[:, 0] >= a) & (dbg[:, 0] < b)
        if mo.sum() < 5 or mj.sum() < 5 or md.sum() < 5:
            break
        D = dbg[md, 2:]
        ex = D[:, 48:51] - D[:, 45:48]
        Q = np.column_stack([np.interp(dbg[md, 0], od[:, 0], od[:, 7 + i]) for i in range(4)])
        lon = 1e3 * np.mean([(q2R(Q[i]).T @ ex[i])[0] for i in range(len(ex))])
        q = np.degrees(J[mj, 1:5])
        rows.append((a - t0, tilt(od[mo, 7:11]).max(), 1e3 * D[:, 28].mean(), lon,
                     q[:, 1].min(), q[:, 2].min(), q[:, 2].max()))
    return rows, bool(d["aborted"])


if __name__ == "__main__":
    for p in sys.argv[1:]:
        rows, ab = bins(p)
        print(f"{os.path.basename(p)[:-4]}  ({'ABORTED' if ab else 'complete'})")
        print("   t[s]  tilt  e_R[mrad]  lon[mm]  q2min  q3min..max")
        for r in rows:
            print(f"  {r[0]:5.1f}  {r[1]:4.1f}  {r[2]:8.1f}  {r[3]:7.1f}  {r[4]:5.1f}  {r[5]:5.1f}..{r[6]:4.1f}")
