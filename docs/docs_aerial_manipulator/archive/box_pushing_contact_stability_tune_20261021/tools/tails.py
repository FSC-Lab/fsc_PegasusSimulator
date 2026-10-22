#!/usr/bin/env python3
"""Is the hardware-like Isaac condition too harsh in its TAILS? The CoM error (xy) distribution in
the hover windows (CoM anchor, no contact): rms, p95, max -- hardware 10-21 vs the force-model runs.
    /usr/bin/python3 tails.py"""
import glob
import os
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
NPZ = os.path.join(HERE, "..", "..", "wb_box_pushing_flight_20261021", "npz")


def st(e):
    return f"rms {np.sqrt(np.mean(e ** 2)):5.1f}  p95 {np.percentile(e, 95):5.1f}  max {e.max():5.1f} mm"


for tag, wins in (("170123", [(28.9, 36.9)]), ("164338", [(39.5, 55.3)])):
    d = np.load(os.path.join(NPZ, f"b{tag}.npz"), allow_pickle=True)
    t0 = min(d[k][0] for k in d.files if k.endswith("__recv") and len(d[k]))
    t = d["wb__recv"] - t0
    D = d["wb__data"].astype(float)
    for a, b in wins:
        m = (t >= a) & (t <= b)
        e = np.linalg.norm(D[m, 48:50] - D[m, 45:47], axis=1) * 1e3
        print(f"HW {tag} hover {b - a:4.1f} s: {st(e)}")
allv = []
for p in sorted(glob.glob(os.path.join(HERE, "..", "runs", "[ghijklrz]_*.npz"))):
    z = np.load(p, allow_pickle=True)
    if "sim_plant_force_noise_n=0.32" not in open(p[:-4] + ".args").read():
        continue
    m = {str(k): float(v) for k, v in zip(z["marks_name"], z["marks_t"])}
    if "go_to_start:start" not in m:
        continue
    D = z["dbg"].astype(float)
    w = (D[:, 0] >= m["go_to_start:start"] - 10) & (D[:, 0] <= m["go_to_start:start"])
    e = np.linalg.norm(D[w, 2 + 48:2 + 50] - D[w, 2 + 45:2 + 47], axis=1) * 1e3
    allv.append(e)
    print(f"{os.path.basename(p)[:-4]:12s} hover 10 s: {st(e)}")
e = np.concatenate(allv)
print(f"ALL force-model hovers ({len(allv)} x 10 s): {st(e)}")
