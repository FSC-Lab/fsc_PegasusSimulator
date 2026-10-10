"""Per-topic receive-time gaps: last message, largest gaps (with their start times).  usage: gaps.py p2 [tmin tmax]"""
import os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
NPZ = os.environ.get("AM_NPZ", os.path.join(HERE, "..", "npz"))
nm = sys.argv[1]
d = np.load(f"{NPZ}/{nm}.npz")
t0 = min(d[k][0] for k in d.files if k.endswith("__recv"))
tend = max(d[k][-1] for k in d.files if k.endswith("__recv")) - t0
lo = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
hi = float(sys.argv[3]) if len(sys.argv) > 3 else 1e9
print(f"bag span {tend:.2f} s")
for k in sorted(d.files):
    if not k.endswith("__recv"):
        continue
    t = d[k] - t0
    if len(t) < 50:
        continue
    m = (t >= lo) & (t <= hi)
    tt = t[m]
    if len(tt) < 3:
        print(f"{k[:-6]:18s} n={len(t):6d} first {t[0]:7.2f} last {t[-1]:7.2f}  (nothing in window)"); continue
    dt = np.diff(tt)
    i = np.argsort(dt)[::-1][:3]
    big = ", ".join(f"{dt[j]*1e3:.0f} ms @ {tt[j]:.2f}" for j in i)
    print(f"{k[:-6]:18s} n={len(t):6d} first {t[0]:7.2f} last {t[-1]:7.2f}  median {np.median(dt)*1e3:6.1f} ms  top gaps: {big}")
