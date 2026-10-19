"""(2026-10-18, Opus) One row per valve run: outcome (and what tripped), the beam's turn, the
in-turn CoM offset along the nose (the law's own x_c - x_cd, body frame, fast half), the
standing pitch error e_R about model x, the arm's extremes and time on its stops, the tilt, the
net contact force (PhysX truth, normal + friction anchors), and -- for an abort -- what happened
in the 8 s after it (tilt max, lowest altitude, landed upright?). From the repo root:
    PYTHONNOUSERSITE=1 /usr/bin/python3 docs/.../tools/campaign_score.py tag [tag ...]"""
import sys
import numpy as np
R = "docs/docs_aerial_manipulator/archive/valve_turning_20261015/runs/"


def q2R(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def tilt(q):
    return np.degrees(np.arccos(np.clip(1 - 2 * (q[:, 1] ** 2 + q[:, 2] ** 2), -1, 1)))


def at(tq, ts, X):
    return np.column_stack([np.interp(tq, ts, X[:, k]) for k in range(X.shape[1])])


def row(tag):
    d = np.load(R + tag + ".npz", allow_pickle=True)
    marks = dict(zip(d["marks_name"].tolist(), d["marks_t"].tolist()))
    od, J, dbg, c, box = d["odom"], d["joints"], d["dbg"], d["contact"], d["box"]
    direct = od[:, 11] > 0.5
    if "push:start" not in marks or not direct.any():
        return f"{tag:14s} (no turn)"
    t0 = marks["push:start"]
    tend = od[direct, 0].max()
    t1 = marks.get("push:end", tend)
    out = "complete" if not bool(d["aborted"]) else str(d["reason"])
    why = "ok"
    if bool(d["aborted"]):
        why = ("tilt guard" if "tilt" in out else "force guard" if "force" in out else
               "injected" if "INJECT" in " ".join(d["events"].tolist()) and "SAFETY GUARD" not in out else
               "watchdog")
    yaw = np.degrees(np.unwrap(np.arctan2(2 * (box[:, 4] * box[:, 7] + box[:, 5] * box[:, 6]),
                                          1 - 2 * (box[:, 6] ** 2 + box[:, 7] ** 2))))
    mb = (box[:, 0] >= t0) & (box[:, 0] <= t1)
    turned = yaw[mb][-1] - yaw[mb][0] if mb.any() else float("nan")
    T = t1 - t0
    # fast half of the turn (the middle 50 %), the law's debug
    tt, D = dbg[:, 0], dbg[:, 2:]
    mf = (tt >= t0 + 0.25 * T) & (tt <= t0 + 0.75 * T)
    lon = er = float("nan")
    if mf.sum() > 50:
        ex = D[mf, 48:51] - D[mf, 45:48]
        Q = at(tt[mf], od[:, 0], od[:, 7:11])
        exb = np.array([q2R(Q[i] / np.linalg.norm(Q[i])).T @ ex[i] for i in range(len(ex))])
        lon = 1e3 * exb[:, 0].mean()
        er = 1e3 * D[mf, 28].mean()
    mj = (J[:, 0] >= t0) & (J[:, 0] <= min(t1, tend))
    q = np.degrees(J[mj, 1:5]); dtj = np.gradient(J[mj, 0])
    on_stop = 100 * dtj[(q[:, 1] < -19.5) | (q[:, 2] > 49.5)].sum() / max(dtj.sum(), 1e-9)
    mo = (od[:, 0] >= t0) & (od[:, 0] <= min(t1, tend)) & direct
    tl = tilt(od[mo, 7:11]).max() if mo.any() else float("nan")
    mc = (c[:, 0] >= t0) & (c[:, 0] <= min(t1, tend))
    F = np.linalg.norm(c[mc, 1:4] + c[mc, 4:7], axis=1) if mc.any() else np.zeros(1)
    after = ""
    if bool(d["aborted"]):
        ta = None
        for e in d["events"].tolist():
            if "ABORTING" in e or "left DIRECT" in e:
                try:
                    ta = float(e.split("s]")[0].strip("[ ")); break
                except Exception:
                    pass
        ta = tend if ta is None else min(ta, tend + 0.05)
        w = (od[:, 0] >= ta) & (od[:, 0] <= ta + 8)
        upright = tilt(od[-50:, 7:11]).max() < 20
        after = (f" | after: tilt {tilt(od[w, 7:11]).max():5.1f}, z min {od[w, 3].min():4.2f}, "
                 f"moved {np.linalg.norm(od[w, 1:3][-1] - od[w, 1:3][0]):4.2f} m, "
                 f"{'DIRECT kept' if od[w, 11].min() > 0.5 else 'DIRECT lost'}, "
                 f"{'upright' if upright else 'NOT UPRIGHT'} at {ta - t0:4.1f} s")
    return (f"{tag:14s} {why:11s} beam {turned:+6.0f} | lon {lon:5.1f} mm  e_R {er:6.1f} mrad | "
            f"q2 {q[:, 1].min():5.1f} q3 {q[:, 2].min():5.1f}..{q[:, 2].max():4.1f} stop {on_stop:4.1f}% | "
            f"tilt {tl:4.1f} | F p95 {np.percentile(F, 95):4.2f}{after}")


if __name__ == "__main__":
    for t in sys.argv[1:]:
        try:
            print(row(t))
        except Exception as exc:
            print(f"{t:14s} ERROR {exc!r}")
