#!/usr/bin/env python3
"""Mismatch grid for the internal-disturbance compensation.

    /usr/bin/python3 grid_table.py

Settled = mean over a 20 s window ending 5 s before the SAFETY revert (the last
samples of DIRECT are contaminated by the mode switch). rms = whole DIRECT soak.
"""
import numpy as np, json, os
HERE = os.path.dirname(os.path.abspath(__file__))
HOME = np.array([0.0, 40.0, 40.0, 0.0])

CELLS = [   # (thrust %, model %, tag, npz)
    (10, 5,  "grid_t10m5_A",  "l1_grid_t10m5_A.npz"),
    (10, 5,  "grid_t10m5_B",  "l1_grid_t10m5_B.npz"),
    (15, 5,  "grid_t15m5_A",  "l1_grid_t15m5_A.npz"),
    (15, 5,  "grid_t15m5_B",  "l1_grid_t15m5_B.npz"),
    (10, 10, "grid_t10m10_A", "l1_grid_t10m10_A.npz"),
    (10, 10, "grid_t10m10_B", "l1_grid_t10m10_B.npz"),
    (15, 10, "fix_ab_A",      "l1_fix_ab_A.npz"),
    (15, 10, "fix_ab_B",      "l1_fix_ab_B.npz"),
    (15, 10, "fix_ab_C",      "l1_fix_ab_C.npz"),
]

def score(path):
    z = np.load(path, allow_pickle=True)
    dbg, log, t0 = z["dbg"], z["log"], float(z["t_direct"])
    if not np.isfinite(t0):
        return dict(ok=False, why="never entered DIRECT")
    t = dbg[:, 0] - t0; d = dbg[:, 1:]; tl = log[:, 0] - t0
    end = tl[log[:, 17] > 0.5].max()
    sel = (t > 0) & (t <= end - 2)
    last = (t >= end - 25) & (t <= end - 5)
    ql = (tl > 0) & (tl <= end - 2); qlast = (tl >= end - 25) & (tl <= end - 5)
    ex = np.linalg.norm(d[:, 48:51] - d[:, 45:48], axis=1)
    ey = np.linalg.norm(d[:, 24:27], axis=1)
    q = np.degrees(d[:, 5:9]); dq = np.abs(q - HOME).max(axis=1)
    tau = np.abs(d[:, 13:17]).max(axis=1)
    return dict(ok=not bool(z["aborted"]), why=str(z["abort_reason"]),
                direct_s=float(end),
                ey_set=float(ey[last].mean() * 1e3),
                ey_rms=float(np.sqrt((ey[sel] ** 2).mean()) * 1e3),
                ey_pk=float(ey[sel].max() * 1e3),
                ex_set=float(ex[last].mean() * 1e3), ex_pk=float(ex[sel].max() * 1e3),
                dq_set=float(dq[last].mean()), dq_max=float(dq[sel].max()),
                q_set=np.round(q[last].mean(axis=0), 1).tolist(),
                tau_set=float(tau[last].mean()), tau_pk=float(tau[sel].max()),
                clamp=float(100 * np.mean(d[sel, 51] > 0)),
                tilt_pk=float(log[ql, 7].max()),
                dz=float(np.nanmean(d[last, 33])),
                u1=float(np.nanmean(d[last, 17])))

rows = []
for th, mo, tag, f in CELLS:
    f = os.path.join(HERE, f)
    if not os.path.exists(f):
        print(f"  (missing {f})"); continue
    r = score(f); r.update(thrust=th, model=mo, tag=tag); rows.append(r)

h = (f"{'thrust':>7s} {'model':>6s} {'run':16s} {'result':>10s} {'d_hat_z':>8s} "
     f"{'e_y set':>8s} {'e_y rms':>8s} {'e_y pk':>7s} {'e_x pk':>7s} "
     f"{'q-home':>7s} {'tau pk':>7s} {'clamp':>6s} {'settled q [deg]':>25s}")
print(h); print("-" * len(h))
for r in rows:
    res = "completed" if r["ok"] else f"ABORT {r['direct_s']:.0f}s"
    print(f"{r['thrust']:6d}% {r['model']:5d}% {r['tag']:16s} {res:>10s} {r['dz']:8.2f} "
          f"{r['ey_set']:8.1f} {r['ey_rms']:8.1f} {r['ey_pk']:7.1f} {r['ex_pk']:7.0f} "
          f"{r['dq_set']:7.1f} {r['tau_pk']:7.2f} {r['clamp']:5.1f}% {str(r['q_set']):>25s}")
json.dump(rows, open(os.path.join(HERE, "grid_metrics.json"), "w"), indent=1)
print("\nwrote grid_metrics.json")
