#!/usr/bin/env python3
"""(2026-10-22) The vehicle's TRUE tilt (Isaac base_truth) through the turn: mean, std about a
2 s moving mean (the ringing), the dominant ringing frequency, and the odometry's rate.
    PYTHONNOUSERSITE=1 /usr/bin/python3 tilt_spectrum.py run.npz [...]"""
import os
import sys
import numpy as np


def run(p):
    d = np.load(p, allow_pickle=True)
    m = dict(zip(d["marks_name"].tolist(), d["marks_t"].tolist()))
    bt, od = d["base_truth"], d["odom"]
    t0 = m["push:start"] + 1.5
    dire = od[:, 11] > 0.5
    t1 = min(t0 + 36.0, od[dire, 0].max())
    s = (bt[:, 0] >= t0) & (bt[:, 0] <= t1)
    t = bt[s, 0]; q = bt[s, 4:8]
    # roll/pitch-free tilt components: the body z axis's horizontal part, world frame
    w, x, y, z = q.T
    zx = 2 * (x * z + w * y); zy = 2 * (y * z - w * x)
    tg = np.linspace(t[0], t[-1], int((t[-1] - t[0]) * 100))
    zx, zy = np.interp(tg, t, zx), np.interp(tg, t, zy)
    tl = np.degrees(np.arcsin(np.clip(np.hypot(zx, zy), 0, 1)))
    k = 200
    out = []
    for c in (zx, zy):
        mv = np.convolve(c, np.ones(k) / k, mode="same")
        r = (c - mv)[k:-k]
        f = np.fft.rfftfreq(len(r), 0.01); P = np.abs(np.fft.rfft(r)) ** 2
        b = (f > 0.3) & (f < 5)
        out.append((np.degrees(r.std()), f[b][np.argmax(P[b])]))
    to = od[(od[:, 0] >= t0) & (od[:, 0] <= t1), 0]
    print(f"{os.path.basename(p)[:-4]:18s} {'ABORT' if bool(d['aborted']) else 'ok   '} "
          f"turn {t1 - t0:4.1f} s  tilt mean {tl.mean():4.2f} max {tl.max():5.2f}  "
          f"ring x {out[0][0]:4.2f} deg @{out[0][1]:4.2f} Hz  y {out[1][0]:4.2f} deg @{out[1][1]:4.2f} Hz  "
          f"odom {1 / np.median(np.diff(to)):5.1f} Hz")


for p in sys.argv[1:]:
    try:
        run(p)
    except Exception as e:
        print(os.path.basename(p), "ERR", e)
