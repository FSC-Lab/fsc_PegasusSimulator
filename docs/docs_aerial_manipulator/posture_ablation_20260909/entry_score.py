#!/usr/bin/env python3
"""Score the SAFETY->DIRECT entry transient of one or more runs.
    /usr/bin/python3 entry_score.py run.npz [run2.npz ...]
peak |e_x| and the time from the DIRECT edge for it to fall below 50 mm and
stay there -- the two numbers the entry tune targets."""
import sys, os, numpy as np
HOME = np.array([0., 40., 40., 0.])
print(f"{'run':22s} {'peak|e_x|':>9s} {'@t':>5s} {'rec<50mm':>9s} {'rec<100':>8s} "
      f"{'tilt pk':>7s} {'e_y pk':>7s} {'e_y set':>8s} {'q-home mx':>9s} {'tau pk':>7s} {'clamp':>6s}")
for p in sys.argv[1:]:
    z = np.load(p, allow_pickle=True)
    dbg, log, t0 = z["dbg"], z["log"], float(z["t_direct"])
    if not np.isfinite(t0):
        print(f"{os.path.basename(p)[:22]:22s}  never entered DIRECT"); continue
    t = dbg[:, 0] - t0; d = dbg[:, 1:]; tl = log[:, 0] - t0
    end = tl[log[:, 17] > 0.5].max()
    sel = (t > 0) & (t <= end - 2); ql = (tl > 0) & (tl <= end - 2)
    ex = np.linalg.norm(d[:, 48:51] - d[:, 45:48], axis=1)
    ey = np.linalg.norm(d[:, 24:27], axis=1)
    q = np.degrees(d[:, 5:9]); dq = np.abs(q - HOME).max(axis=1)
    tau = np.abs(d[:, 13:17]).max(axis=1)
    ts, es = t[sel], ex[sel]
    ipk = int(np.argmax(es))
    def rec(th):
        b = es < th
        for i in range(ipk, len(ts)):
            if b[i:].all(): return ts[i]
        return np.nan
    last = (t >= end - 25) & (t <= end - 5)
    print(f"{os.path.basename(p)[:22]:22s} {es[ipk]*1e3:9.0f} {ts[ipk]:5.1f} {rec(0.05):9.1f} {rec(0.10):8.1f} "
          f"{log[ql,7].max():7.1f} {ey[sel].max()*1e3:7.1f} {ey[last].mean()*1e3:8.2f} {dq[sel].max():9.1f} "
          f"{tau[sel].max():7.2f} {100*np.mean(d[sel,51]>0):5.1f}%"
          + ("   ABORTED" if bool(z["aborted"]) else ""))
