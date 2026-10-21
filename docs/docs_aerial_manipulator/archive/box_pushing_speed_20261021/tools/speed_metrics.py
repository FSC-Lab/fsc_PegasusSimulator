#!/usr/bin/env python3
"""The push-speed sweep's per-run numbers (2026-10-21), from ../runs/<tag>.npz:

    PYTHONNOUSERSITE=1 /usr/bin/python3 speed_metrics.py [tag ...] [--group]

  v_mean    the planned mean speed [mm/s] (the run's .args: SPEED, or d / push time)
  T         the push's moving time as flown [s] (push:end - push:start - the 1.5 s settle)
  trav      the box's travel along the push, push start -> release [mm]
  v_box     the box's mean speed over the moving time [mm/s]
  pk_box    the box's peak speed (0.2 s differences) [mm/s] vs pk_plan, the claw reference's
  stuck     the share of the moving time with the box below 3 mm/s while the plan moves
            faster than 30 % of its peak [%] -- the stick in the middle of the push
  lag       the planner's box lag, largest |value| in the push [mm] (its guard: 40)
  btilt     the planner's box tilt, largest in the push [deg] (its guard: 2)
  q3min     the lowest q3 from the push start to the end of the exit [deg] (its guard: 10) --
            in practice at the grasp (0.1-0.3 s into the push step), before anything moves
  q2max     the highest q2 over the same window [deg] (its stop: 45)
  q3mv/q2mv the same over the MOVING part of the push only (the speed's own effect)
  rq3/rq2   the same over the RELEASE window (release start -> exit end: the jaws open)
  vtilt     the vehicle's largest tilt in the push [deg] (the planner's guard: 10)
--group prints the mean / min / max per speed, completed runs only.
"""
import argparse
import glob
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
RUNS = os.path.join(HERE, "..", "runs")
SETTLE = 1.5
# the driver's joint record: the Isaac joint_states in joint1..joint4 order (joint_names)
COL_Q2, COL_Q3 = 1, 2


def args_of(tag):
    kv = {}
    p = os.path.join(RUNS, tag + ".args")
    if os.path.isfile(p):
        for ln in open(p).read().split():
            if "=" in ln:
                k, v = ln.split("=", 1)
                kv[k] = v
    return kv


def mark(z, name):
    names = [str(n) for n in z["marks_name"]]
    return float(z["marks_t"][names.index(name)]) if name in names else float("nan")


def window(a, t0, t1):
    return a[(a[:, 0] >= t0) & (a[:, 0] <= t1)]


def one(tag):
    z = np.load(os.path.join(RUNS, tag + ".npz"), allow_pickle=True)
    kv = args_of(tag)
    d = float(kv.get("DIST", kv.get("push_pull_push_distance", "nan")))
    v_set = kv.get("push_pull_push_mean_speed")
    T_set = kv.get("push_pull_push_time_s")
    v_mean = float(v_set) if v_set else d / float(T_set) if T_set else float("nan")
    p0, p1 = mark(z, "push:start"), mark(z, "push:end")
    r0, x1 = mark(z, "release:start"), mark(z, "exit:end")
    row = {"tag": tag, "ok": not bool(z["aborted"]), "reason": str(z["reason"]),
           "mass": float(kv.get("MASS", "nan")), "dist": d, "v_mean": v_mean * 1e3}
    if not np.isfinite(p0) or not np.isfinite(p1):
        return row
    m0 = p0 + SETTLE
    row["T"] = p1 - m0
    box = z["box"]
    u = np.array([0.0, -1.0, 0.0])   # the push: -y
    b = window(box, p0 - 0.2, (r0 if np.isfinite(r0) else p1) + 0.2)
    s = (b[:, 1:4] - b[0, 1:4]) @ u
    row["trav"] = s[-1] * 1e3
    # the box's speed: 0.2 s differences
    t = b[:, 0]
    k = np.searchsorted(t, t - 0.2)
    ok = (t - t[k]) > 0.15
    vb = np.full_like(t, np.nan)
    vb[ok] = (s[ok] - s[k[ok]]) / (t[ok] - t[k[ok]])
    mv = (t >= m0) & (t <= p1)
    sm = window(np.column_stack([t, s]), m0, p1)
    row["v_box"] = (sm[-1, 1] - sm[0, 1]) / (sm[-1, 0] - sm[0, 0]) * 1e3 if len(sm) > 2 else np.nan
    row["pk_box"] = np.nanmax(vb[mv]) * 1e3 if mv.any() else np.nan
    # the plan: the claw reference's speed along the push
    rd = window(z["refd"], m0, p1)
    vp = rd[:, 1:4] @ u
    row["pk_plan"] = vp.max() * 1e3 if len(vp) else np.nan
    if len(vp) and mv.any():
        vpi = np.interp(t[mv], rd[:, 0], vp)
        moving = vpi > 0.3 * vp.max()
        stuck = moving & (vb[mv] < 3e-3)
        row["stuck"] = 100.0 * stuck.sum() / max(moving.sum(), 1)
    ar = window(z["arrival"], p0, p1)
    row["lag"] = np.max(np.abs(ar[:, 12])) * 1e3 if len(ar) else np.nan
    row["btilt"] = np.max(ar[:, 13]) if len(ar) else np.nan
    j = window(z["joints"], p0, x1 if np.isfinite(x1) else p1)
    row["q3min"] = np.degrees(j[:, 1 + COL_Q3].min()) if len(j) else np.nan
    row["q2max"] = np.degrees(j[:, 1 + COL_Q2].max()) if len(j) else np.nan
    jr = window(z["joints"], r0, x1) if np.isfinite(r0) and np.isfinite(x1) else np.zeros((0, 10))
    row["rq3"] = np.degrees(jr[:, 1 + COL_Q3].min()) if len(jr) else np.nan
    row["rq2"] = np.degrees(jr[:, 1 + COL_Q2].max()) if len(jr) else np.nan
    jm = window(z["joints"], m0, p1)
    row["q3mv"] = np.degrees(jm[:, 1 + COL_Q3].min()) if len(jm) else np.nan
    row["q2mv"] = np.degrees(jm[:, 1 + COL_Q2].max()) if len(jm) else np.nan
    o = window(z["base_truth"], p0, p1)
    if len(o):
        w, x, y = o[:, 4], o[:, 5], o[:, 6]
        r22 = 1.0 - 2.0 * (x * x + y * y)
        row["vtilt"] = np.degrees(np.arccos(np.clip(r22, -1, 1))).max()
    return row


COLS = [("v_mean", "{:6.1f}"), ("T", "{:5.1f}"), ("trav", "{:5.0f}"), ("v_box", "{:5.1f}"),
        ("pk_box", "{:6.1f}"), ("pk_plan", "{:7.1f}"), ("stuck", "{:5.0f}"), ("lag", "{:4.0f}"),
        ("btilt", "{:5.2f}"), ("q3min", "{:5.1f}"), ("q2max", "{:5.1f}"), ("q3mv", "{:5.1f}"),
        ("q2mv", "{:5.1f}"), ("rq3", "{:5.1f}"), ("rq2", "{:5.1f}"), ("vtilt", "{:5.1f}")]


def fmt(row):
    out = [f"{row['tag']:12s} {'y' if row['ok'] else 'N'} "]
    for k, f in COLS:
        v = row.get(k, np.nan)
        out.append(f.format(v) if np.isfinite(v) else " " * len(f.format(0.0)))
    if not row["ok"]:
        out.append("  " + row["reason"][:90])
    return " ".join(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("tags", nargs="*")
    ap.add_argument("--group", action="store_true")
    a = ap.parse_args()
    tags = a.tags or sorted(os.path.basename(p)[:-4] for p in glob.glob(os.path.join(RUNS, "v*.npz")))
    rows = [one(t) for t in tags]
    print("run          ok " + " ".join(f"{k:>{len(f.format(0.0))}s}" for k, f in COLS))
    for r in rows:
        print(fmt(r))
    if a.group:
        print("\nper speed, completed runs: mean [min .. max]")
        for v in sorted({round(r["v_mean"], 1) for r in rows}):
            g = [r for r in rows if round(r["v_mean"], 1) == v]
            okg = [r for r in g if r["ok"]]
            line = f"  {v:5.1f} mm/s  {len(okg)}/{len(g)} complete"
            for k in ("T", "trav", "v_box", "pk_box", "stuck", "lag", "btilt", "q3min", "q2max",
                      "q3mv", "q2mv", "rq3", "rq2", "vtilt"):
                x = np.array([r.get(k, np.nan) for r in okg], float)
                x = x[np.isfinite(x)]
                if len(x):
                    line += f" | {k} {x.mean():.1f} [{x.min():.1f}..{x.max():.1f}]"
            print(line)


if __name__ == "__main__":
    main()
