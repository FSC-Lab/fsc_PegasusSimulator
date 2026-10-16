#!/usr/bin/env python3
"""Where in the mission the 1.2-1.8 Hz roll/pitch band is largest: the 5 s windows ranked, each named
by the mission step it falls in.  PYTHONNOUSERSITE=1 /usr/bin/python3 wobble_where.py ../runs/D2.npz ..."""
import sys
import numpy as np
from scipy.signal import butter, filtfilt

for path in sys.argv[1:]:
    d = np.load(path, allow_pickle=True)
    marks = sorted((float(t), str(n)) for n, t in zip(d["marks_name"], d["marks_t"]))
    o = d["odom"]; t = o[:, 0]; w, x, y, z = o[:, 7:11].T
    roll = np.degrees(np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)))
    pitch = np.degrees(np.arcsin(np.clip(2 * (w * y - z * x), -1, 1)))
    a = dict(marks).get if False else None
    t0 = next(tt for tt, n in marks if n == "direct")
    t1 = next((tt for tt, n in marks if n == "execute_land:end"), t[-1])
    tu = np.arange(t0, t1, 0.01)
    b, aa = butter(2, [1.2 / 50, 1.8 / 50], btype="band")
    bp = filtfilt(b, aa, np.interp(tu, t, pitch)); br = filtfilt(b, aa, np.interp(tu, t, roll))
    rows = []
    for i in range(0, len(tu) - 500, 100):
        rp, rr = np.sqrt(np.mean(bp[i:i + 500] ** 2)), np.sqrt(np.mean(br[i:i + 500] ** 2))
        tc = tu[i + 250]
        step = [n for tt, n in marks if tt <= tc][-1]
        rows.append((max(rp, rr), "pitch" if rp >= rr else "roll", tc - t0, step))
    rows.sort(reverse=True)
    print(f"{path.split('/')[-1]}: worst 5 s windows of the 1.2-1.8 Hz band (s after DIRECT entry)")
    for v, ax, tc, step in rows[:4]:
        print(f"   {v:5.2f} deg rms {ax:5s} at {tc:6.1f} s  (after mark '{step}')")
