#!/usr/bin/env python3
"""One worst-case run's story (2026-10-16): the event log from the push on, and the time
series that matter around it -- the box's lag behind its plan and yaw, the claw-handle gap,
the guard-filtered force reading (vector, 3 rad/s, as the planner's guard), the TRUE contact
force on the box (PhysX), tilt, q2 / q3, the box's height (did it fall?).
    PYTHONNOUSERSITE=1 /usr/bin/python3 worst_case.py <tag> [--dt 0.5]
"""
import argparse, os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); RUNS = os.path.join(HERE, "..", "runs")
sys.path.insert(0, os.path.join(HERE, "..", "..", "push_pull_20261003", "tools"))
from pl_metrics import mark  # noqa: E402
from pl_score import tilt_deg  # noqa: E402

ap = argparse.ArgumentParser(); ap.add_argument("tag"); ap.add_argument("--dt", type=float, default=0.5)
ap.add_argument("--span", type=float, default=0.0, help="seconds after the push start to stop (0 = the run's end)")
a = ap.parse_args()
z = np.load(os.path.join(RUNS, a.tag + ".npz"), allow_pickle=True)
ps = mark(z, "push:start") or mark(z, "ready:start")
print("reason:", str(z["reason"])[:200])
for e in z["events"]:
    s = str(e); t = float(s[1:s.index("s]")])
    if t >= ps - 8 and any(k in s for k in ("push_pull:", "INJECT", "GUARD", "ABORT", "gripper", "MARK", "refused", "planner abort", "SAFETY", "landing")):
        print("  ", s[:170])
yt = open(os.path.join(RUNS, a.tag + ".yaml")).read()
off = np.array(eval(yt.split("push_pull_ee_offset:")[1].split("\n")[0].split("#")[0]))
b, ref, cl, o, j, d, ct = z["box"], z["ref"], z["claw"], z["odom"], z["joints"], z["dbg"].astype(float), z["contact"]
G = np.zeros((len(d), 3))
for i in range(1, len(d)):
    al = np.exp(-3.0 * min(max(d[i, 0] - d[i - 1, 0], 0.0), 0.1)); G[i] = al * G[i - 1] + (1 - al) * d[i, 99:102]
t_end = (ps + a.span) if a.span > 0 else min(b[-1, 0], o[-1, 0])
ip = lambda arr, k, t: np.interp(t, arr[:, 0], arr[:, k])
def yaw(t):
    q = [ip(b, k, t) for k in (4, 5, 6, 7)]; w, x, y, zq = q
    return np.degrees(np.arctan2(2 * (w * zq + x * y), 1 - 2 * (y * y + zq * zq)))
b0 = np.array([ip(b, k, ps) for k in (1, 2, 3)]); r0 = np.array([ip(ref, k, ps) for k in (1, 2, 3)]); y0 = yaw(ps)
print(f"{'t-push':>6s} {'lag':>6s} {'boxyaw':>6s} {'gap':>5s} {'Fguard':>6s} {'Ftrue':>6s} {'tilt':>5s} {'q2':>5s} {'q3':>5s} {'box z':>6s} {'mode':>4s}")
for t in np.arange(ps, t_end, a.dt):
    B = np.array([ip(b, k, t) for k in (1, 2, 3)]); R = np.array([ip(ref, k, t) for k in (1, 2, 3)])
    lag = ((R - r0)[1] * -1 - (B - b0)[1] * -1) * 1e3
    yw = yaw(t); c = np.array([ip(cl, k, t) for k in (1, 2, 3)])
    a_ = np.radians(yw); gp = c - (B + np.array([[np.cos(a_), -np.sin(a_), 0], [np.sin(a_), np.cos(a_), 0], [0, 0, 1]]) @ off)
    k = min(np.searchsorted(d[:, 0], t), len(d) - 1); fg = np.linalg.norm(G[k])
    kc = min(np.searchsorted(ct[:, 0], t), len(ct) - 1); ft = np.linalg.norm(ct[kc, 1:4] + ct[kc, 4:7]) if len(ct) else np.nan
    ko = min(np.searchsorted(o[:, 0], t), len(o) - 1); tl = float(tilt_deg(o[ko:ko + 1, 7], o[ko:ko + 1, 8], o[ko:ko + 1, 9], o[ko:ko + 1, 10])[0])
    kj = min(np.searchsorted(j[:, 0], t), len(j) - 1)
    print(f"{t - ps:6.1f} {lag:6.0f} {yw - y0:6.1f} {np.linalg.norm(gp) * 1e3:5.0f} {fg:6.1f} {ft:6.1f} {tl:5.1f} "
          f"{np.degrees(j[kj, 2]):5.1f} {np.degrees(j[kj, 3]):5.1f} {B[2]:6.3f} {'D' if o[ko, 11] > 0.5 else 'S':>4s}")
