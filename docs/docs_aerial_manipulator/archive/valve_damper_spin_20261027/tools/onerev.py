#!/usr/bin/env python3
"""Is the 1/rev torque CONSERVATIVE (gravity on the tilted axis: same sign both directions) or
DISSIPATIVE (friction that varies around the hub: always against the motion)?
Energy regression per coast with the 1/rev regressors plain (gravity) or times the direction s
(friction modulation); compares the fits by residual and leave-one-coast-out prediction."""
import sys, numpy as np
C = np.load(sys.argv[1]); t, th, w, segs = C["t"], C["th"], C["w"], C["segs"]
rows = []
for i0, ir, s, ths in segs:
    i0, ir = int(i0), int(ir)
    v = np.clip(s * w[i0:ir + 1], 0, None); D = np.clip(s * (ths - th[i0:ir + 1]), 0, None)
    x = th[i0:ir + 1]; k = D > np.radians(2)
    g = np.c_[-(np.cos(x) - np.cos(ths)), -(np.sin(ths) - np.sin(x))]
    rows.append((np.c_[D, g, s * g][k], 0.5 * v[k] ** 2))
def fit(cols, idx):
    X = np.concatenate([rows[i][0][:, cols] for i in idx]); y = np.concatenate([rows[i][1] for i in idx])
    return np.linalg.lstsq(X, y, rcond=None)[0]
n = len(rows)
for cols, lab in [([0], "Coulomb only"), ([0, 1, 2], "+ conservative 1/rev"), ([0, 3, 4], "+ dissipative 1/rev"),
                  ([0, 1, 2, 3, 4], "+ both")]:
    b = fit(cols, range(n))
    cv = 0.0; tot = 0.0
    for j in range(n):                                    # leave one coast out
        bj = fit(cols, [i for i in range(n) if i != j])
        r = rows[j][1] - rows[j][0][:, cols] @ bj; cv += r @ r; tot += len(r)
    amp = [np.hypot(b[k], b[k + 1]) for k in range(1, len(b), 2)]
    print(f"{lab:22s} a {b[0]:6.2f}  1/rev amp {np.round(amp, 2)}  LOO rms {np.sqrt(cv / tot):.3f} rad^2/s^2")
