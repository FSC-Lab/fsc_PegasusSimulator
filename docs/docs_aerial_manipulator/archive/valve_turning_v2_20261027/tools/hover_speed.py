#!/usr/bin/env python3
"""The claw's speed in the Go To Start hover (Isaac truth, claw_0), over the planner's 0.3 s window:
the share of the hover below 30 / 50 / 70 mm/s and the longest run below each. Ends at Ready (or 15 s).
    hover_speed.py tag [tag ...]"""
import os, sys, numpy as np
R = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "runs")
for tag in sys.argv[1:]:
    d = np.load(os.path.join(R, tag + ".npz"), allow_pickle=True)
    m = dict(zip(d["marks_name"].tolist(), d["marks_t"].tolist()))
    c = d["claw"]; t = c[:, 0]; p = c[:, 1:4]
    t0 = m["go_to_start:end"]; t1 = min(m.get("ready:start", t0 + 15), t0 + 15)
    k = (t >= t0) & (t <= t1); tk, pk = t[k], p[k]
    j = np.searchsorted(tk, tk - 0.3); v = np.linalg.norm(pk - pk[j], axis=1) / np.maximum(tk - tk[j], 1e-3)
    v = v[tk - tk[0] >= 0.3]; tt = tk[tk - tk[0] >= 0.3]
    out = []
    for lim in (0.03, 0.05, 0.07):
        b = v < lim; run = best = 0.0
        for i in range(1, len(b)):
            run = run + (tt[i] - tt[i - 1]) if (b[i] and b[i - 1]) else 0.0; best = max(best, run)
        out.append(f"<{lim*1e3:.0f}: {100*b.mean():3.0f} % (longest {best:4.1f} s)")
    print(f"{tag:14s} hover {t1 - t0:4.1f} s  median {1e3*np.median(v):4.0f} mm/s  " + "  ".join(out))
