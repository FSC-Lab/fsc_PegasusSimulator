#!/usr/bin/env python3
"""The 10-21 speed sweep's per-run push numbers (box_pushing_speed_20261021/tools/speed_metrics.py,
same definitions) for the hardware push 170123, next to the three Isaac runs at the same settings
(240 g, 0.10 m, 21 mm/s). Hardware: the release window = push end -> the abort's end (there was
no normal Exit), the box = obj_0, the vehicle tilt = the raw mocap, the plan = the streamed r_ed_dot.
Also the law's EE task error e_y (mean per axis, world) in the world-held phases."""
import numpy as np
from bp_common import Bag, tilt_deg
from phases import sim_run, win
import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "box_pushing_speed_20261021", "tools"))
import speed_metrics as SM   # noqa: E402

SETTLE = 1.5


def hw_row():
    b = Bag("170123")
    p0, p1, r0, x1 = 39.58, 45.84, 46.62, 58.93
    m0 = p0 + SETTLE
    row = {"tag": "HW 170123", "ok": True, "v_mean": 21.0, "T": p1 - m0}
    tb = b.t("obj")
    B = np.column_stack([b.f("obj", f"pose.position.{c}") for c in "xyz"])
    u = np.array([0.0, -1.0, 0.0])
    w = (tb >= p0 - 0.2) & (tb <= r0 + 0.2)
    t, s = tb[w], (B[w] - B[w][0]) @ u
    row["trav"] = s[-1] * 1e3
    row["trav_at_push_end"] = np.interp(p1, t, s) * 1e3
    row["trav_max"] = s.max() * 1e3
    k = np.searchsorted(t, t - 0.2)
    ok = (t - t[k]) > 0.15
    vb = np.full_like(t, np.nan)
    vb[ok] = (s[ok] - s[k[ok]]) / (t[ok] - t[k[ok]])
    mv = (t >= m0) & (t <= p1)
    sm = s[mv]; tm = t[mv]
    row["v_box"] = (sm[-1] - sm[0]) / (tm[-1] - tm[0]) * 1e3
    row["pk_box"] = np.nanmax(vb[mv]) * 1e3
    trd = b.t("wbref")
    vd = np.column_stack([b.f("wbref", f"r_ed_dot.{c}") for c in "xyz"]) @ u
    wr = (trd >= m0) & (trd <= p1)
    row["pk_plan"] = vd[wr].max() * 1e3
    vpi = np.interp(t[mv], trd, vd)
    moving = vpi > 0.3 * vd[wr].max()
    row["stuck"] = 100.0 * np.sum(moving & (vb[mv] < 3e-3)) / max(moving.sum(), 1)
    A = b.f("pl_arr", "data"); ta = b.t("pl_arr")
    wa = (ta >= p0) & (ta <= p1)
    row["lag"] = np.max(np.abs(A[wa, 11])) * 1e3
    row["btilt"] = np.max(A[wa, 12])
    js = b.f("js", "position"); tj = b.t("js")
    q2, q3 = np.degrees(js[:, 0]), np.degrees(js[:, 1])
    def mm(a, c):
        w = (tj >= a) & (tj <= c)
        return q3[w].min(), q2[w].max()
    row["q3min"], row["q2max"] = mm(p0, x1)
    row["q3mv"], row["q2mv"] = mm(m0, p1)
    row["rq3"], row["rq2"] = mm(p1, x1)
    tm_ = b.t("mocap")
    tl = tilt_deg(*[b.f("mocap", f"pose.orientation.{c}") for c in "wxyz"])
    row["vtilt"] = tl[(tm_ >= p0) & (tm_ <= p1)].max()
    return row


rows = [hw_row()] + [SM.one(t) for t in ("v21_1", "v21_2", "v21_3")]
print("run          ok " + " ".join(f"{k:>{len(f.format(0.0))}s}" for k, f in SM.COLS))
for r in rows:
    print(SM.fmt(r))
h = rows[0]
print(f"\nHW box travel: at push end {h['trav_at_push_end']:.1f} mm, max {h['trav_max']:.1f} mm, at the abort start {h['trav']:.1f} mm (the box was pulled back after the push)")

# the law's own EE task error, world axes, in the world-held phases
def ey_means(name, dbg, a, c):
    d = win(dbg, a, c)
    e = d[:, 2 + 24:2 + 27] * 1e3
    print(f"  {name:10s} e_y mean [{e[:,0].mean():+5.1f} {e[:,1].mean():+5.1f} {e[:,2].mean():+5.1f}] mm, std [{e[:,0].std():4.1f} {e[:,1].std():4.1f} {e[:,2].std():4.1f}]")
b = Bag("170123")
D = np.column_stack([b.t("wb"), np.zeros(len(b.t("wb"))), b.f("wb", "data")])
print("\nlaw's EE task error e_y = y - y_d (world):")
ey_means("HW onhand", D, 37.5, 38.9)
ey_means("HW contact", D, 39.6, 45.8)
for t in ("v21_1", "v21_2", "v21_3"):
    s = sim_run(t); m = s["marks"]
    ey_means(f"SIM {t[-1]} onh", s["dbg"], m["ready:end"] + 1.0, m["gripper_close:start"])
    ey_means(f"SIM {t[-1]} cont", s["dbg"], m["push:start"], m["push:end"])
