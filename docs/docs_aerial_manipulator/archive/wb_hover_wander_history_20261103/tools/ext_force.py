#!/usr/bin/env python3
"""The horizontal external force on the vehicle in every still hold, from the raw mocap alone (wind + anything else the
model does not carry), independent of the controller's own estimate:
    F_ext,xy = m a_xy - T (R e3)_xy,   T = m (g + a_z) / (R e3)_z
a = the raw VRPN position (120 Hz) differentiated twice after a 4 Hz low-pass; R = the VRPN attitude (a constant
body-frame offset only adds a constant, removed by the band-pass). Reported: the 0.1-1 Hz (slow push) and 1-3 Hz
(gusty) band std per world axis [N], m = 3.746 kg. Reads holds.csv; appends the columns and prints a summary."""
import os, csv
import numpy as np
import history as H
M, G = 3.746, 9.81
HERE = os.path.dirname(os.path.abspath(__file__))

def bandpass(t, x, lo, hi):
    dt = t[1] - t[0]; y = x - np.polyval(np.polyfit(t, x, 1), t)
    F = np.fft.rfft(y); f = np.fft.rfftfreq(len(y), dt)
    F[(f < lo) | (f > hi)] = 0
    return t, np.fft.irfft(F, len(y))

def lowpass(t, x, hi, dt=0.01):
    tt = np.arange(t[0], t[-1], dt); y = np.interp(tt, t, x); m = np.polyval(np.polyfit(tt, y, 1), tt)
    F = np.fft.rfft(y - m); f = np.fft.rfftfreq(len(y), dt); F[f > hi] *= 0
    return tt, np.fft.irfft(F, len(y)) + m

rows = list(csv.DictReader(open(os.path.join(HERE, "..", "holds.csv"))))
cache = {}
for r in rows:
    ts = r["ts"]
    if ts not in cache:
        p = H.find_npz(ts); H._c.NPZ = os.path.dirname(p); b = H._c.Bag(ts)
        tv = b.t("vrpn"); P = np.stack([b.f("vrpn", f"pose.position.{k}") for k in "xyz"], 1)
        Q = np.stack([b.f("vrpn", f"pose.orientation.{k}") for k in "wxyz"], 1)
        cache = {ts: (tv, P, Q)}
    tv, P, Q = cache[ts]
    a0 = float(r["t0"]); a1 = a0 + float(r["dur"])
    w = (tv >= a0 - 1.0) & (tv <= a1 + 1.0)
    if w.sum() < 200: continue
    t = tv[w]; p = P[w]; q = Q[w]
    acc = []
    for k in range(3):
        tt, y = lowpass(t, p[:, k], 4.0)
        acc.append(np.gradient(np.gradient(y, tt), tt))
    acc = np.array(acc).T
    w_, x_, y_, z_ = [np.interp(tt, t, q[:, k]) for k in range(4)]
    e3 = np.stack([2 * (x_ * z_ + w_ * y_), 2 * (y_ * z_ - w_ * x_), 1 - 2 * (x_ * x_ + y_ * y_)], 1)
    T = M * (G + acc[:, 2]) / e3[:, 2]
    F = M * acc[:, :2] - T[:, None] * e3[:, :2]
    keep = (tt >= a0) & (tt <= a1)
    out = {}
    for lab, lo, hi in (("slow", 0.1, 1.0), ("gust", 1.0, 3.0)):
        for k, ax in enumerate("xy"):
            _, yb = bandpass(tt, F[:, k], lo, hi)
            out[f"F{lab}_{ax}"] = round(yb[keep].std(), 3)
    r.update(out)
keys = list(rows[0].keys()) + [k for k in ("Fslow_x", "Fslow_y", "Fgust_x", "Fgust_y") if k not in rows[0]]
with open(os.path.join(HERE, "..", "holds.csv"), "w", newline="") as f:
    wr = csv.DictWriter(f, fieldnames=keys); wr.writeheader(); wr.writerows(rows)
for r in rows:
    if "Fslow_x" not in r: continue
    print(f"{r['task']:11s} {r['ts']} {'W' if r['wall']=='1' else ' '} {float(r['dur']):5.1f}s {r['pose']:16s} ({float(r['x']):+5.2f},{float(r['y']):+5.2f},{float(r['z']):4.2f}) {r['state'][:22]:22s} "
          f"CoM rms {float(r['rms_x']):5.1f}/{float(r['rms_y']):5.1f} | F_ext slow {r['Fslow_x']:5.3f}/{r['Fslow_y']:5.3f} gust {r['Fgust_x']:5.3f}/{r['Fgust_y']:5.3f} N")
