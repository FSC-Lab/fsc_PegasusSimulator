#!/usr/bin/env python3
"""The laps' attitude shaking by frequency: gyro roll / pitch rate power in bands (0.3-1, 1-2, 2-3, 3-5, 5-10 Hz), the
dominant peak, 10-28 (no wall, old trim) vs 11-02 (wall up, new trim) -- a narrow peak points at the vehicle / control
loop, a broadband rise at turbulence."""
import os, json
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); CACHE = os.path.join(HERE, "..", "cache")
SC = json.load(open(os.path.join(CACHE, "scores.json")))
import body_axes as B
BANDS = [(0.3, 1), (1, 2), (2, 3), (3, 5), (5, 10)]
print(f"{'lap':28s} | roll-rate rms per band [deg/s]: " + "  ".join(f"{a}-{b}" for a, b in BANDS) + " | peak (roll)  | pitch-rate same")
for key, lab in B.LAB.items():
    if key not in SC: continue
    a, b = SC[key]["window"]
    p = os.path.realpath(os.path.join(CACHE, key + ".npz")); d = np.load(p, allow_pickle=True)
    t0 = min(d[k][0] for k in d.files if k.endswith("__recv"))
    ts = d["sc__recv"] - t0; g = np.stack(d["sc__gyro_rad"]).astype(float)
    T = np.arange(a, b, 0.005); G = np.degrees(np.stack([np.interp(T, ts, g[:, k]) for k in range(2)], 1))
    out = []
    for k in range(2):
        y = G[:, k] - np.polyval(np.polyfit(T, G[:, k], 1), T)
        F = np.fft.rfft(y * np.hanning(len(y))); f = np.fft.rfftfreq(len(y), 0.005); P = np.abs(F) ** 2
        P = P / P.sum() * (y.std() ** 2)
        bands = [np.sqrt(P[(f >= lo) & (f < hi)].sum()) for lo, hi in BANDS]
        m = (f > 0.3) & (f < 10); pk = f[m][np.argmax(P[m])]
        out.append((bands, pk))
    print(f"{lab:17s} {key:10s} | " + "  ".join(f"{v:4.2f}" for v in out[0][0]) + f" | {out[0][1]:4.2f} Hz | " + "  ".join(f"{v:4.2f}" for v in out[1][0]) + f" | {out[1][1]:4.2f} Hz")
