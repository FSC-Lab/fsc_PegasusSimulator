#!/usr/bin/env python3
"""Step and whole-run metrics on the controller-feedback topic, real vs IsaacSim.

Method follows fsc_autopilot_ros2/docs/sim_to_real_fidelity.md so the numbers are directly
comparable to the X650 campaign: overshoot, 10-90% rise and +-5% settling are all measured
from each trace's OWN 10% crossing rather than from commanded onset, and the trajectory
RMSE is computed after aligning on that crossing with the small initial-condition offset
removed -- so it measures response shape, not onset timing.
"""
import json
import sys

import numpy as np

from plot_odom import (detect_steps, load_real, load_sim, series, vel_series, win)

HORIZON = 10.0


def step_trace(side, st, pre=1.0, post_max=13.0):
    y, _ = series(side, st["axis"])
    hold = min(st["hold"] or post_max, post_max)
    return win(side["t"], y, st["t"], pre, hold)


def crossing(t, y, y0, size, frac):
    """First time after 0 at which the trace passes frac of the way to the target."""
    target = y0 + frac * size
    post = t >= 0
    tp, yp = t[post], y[post]
    s = np.sign(size)
    idx = np.where(s * (yp - target) >= 0)[0]
    return float(tp[idx[0]]) if len(idx) else None


def step_metrics(t, y, st):
    pre = y[t < 0]
    y0 = float(pre.mean()) if len(pre) else float(y[0])
    size = st["to"] - y0
    if abs(size) < 1e-6:
        return {}
    s = np.sign(size)
    t10 = crossing(t, y, y0, size, 0.1)
    t90 = crossing(t, y, y0, size, 0.9)
    out = {"y0": y0, "size": size}
    if t10 is not None and t90 is not None:
        out["rise"] = t90 - t10
        out["t10"] = t10
    post = t >= 0
    if post.sum() > 3:
        peak = np.max(s * (y[post] - y0))
        out["overshoot"] = 100.0 * (peak - abs(size)) / abs(size)
    # settling within +-5% of the step size, measured from the 10% crossing
    if t10 is not None:
        tol = 0.05 * abs(size)
        m = t >= t10
        tm, ym = t[m], y[m]
        bad = np.where(np.abs(ym - st["to"]) > tol)[0]
        if len(bad) == 0:
            out["settle"] = 0.0
        elif bad[-1] + 1 < len(tm):
            out["settle"] = float(tm[bad[-1] + 1] - t10)
    return out


def shape_rmse(tr, yr, mr, ts, ys, ms):
    if "t10" not in mr or "t10" not in ms:
        return np.nan
    grid = np.arange(0.0, HORIZON, 0.02)
    a = np.interp(grid, tr - mr["t10"], yr - mr["y0"])
    b = np.interp(grid, ts - ms["t10"], ys - ms["y0"])
    return float(np.sqrt(np.mean((b - a) ** 2)))


def main():
    tag = sys.argv[1] if len(sys.argv) > 1 else "C"
    real = load_real(f"exp_{tag}.npz")
    sim = load_sim(sys.argv[2] if len(sys.argv) > 2 else f"sim_stack_{tag}.npz")
    steps = detect_steps(real["ref_t"], real["ref_pos"], real["ref_yaw"])

    rows = []
    for st in steps:
        tr, yr = step_trace(real, st)
        ts, ys = step_trace(sim, st)
        if len(tr) < 20 or len(ts) < 20:
            continue
        mr, ms = step_metrics(tr, yr, st), step_metrics(ts, ys, st)
        if not mr or not ms:
            continue
        rows.append(dict(idx=st["idx"], axis=st["axis"], to=st["to"], size=mr["size"],
                         hold=st["hold"],
                         real_over=mr.get("overshoot"), sim_over=ms.get("overshoot"),
                         real_rise=mr.get("rise"), sim_rise=ms.get("rise"),
                         real_settle=mr.get("settle"), sim_settle=ms.get("settle"),
                         rmse=shape_rmse(tr, yr, mr, ts, ys, ms)))

    f = lambda v, w, p=2: (f"{v:{w}.{p}f}" if v is not None and np.isfinite(v)
                           else f"{'-':>{w}}")
    print(f"\n{'='*104}\nPER-STEP, controller-feedback topic (flight {tag})\n{'='*104}")
    print(f"{'#':>2} {'axis':>4} {'target':>7} {'size':>6} | {'over% r':>8} {'over% s':>8} "
          f"| {'rise r':>7} {'rise s':>7} | {'set r':>6} {'set s':>6} | {'RMSE':>7}")
    print("-" * 104)
    for r in rows:
        print(f"{r['idx']:2d} {r['axis']:>4} {r['to']:7.2f} {r['size']:6.2f} | "
              f"{f(r['real_over'],8,1)} {f(r['sim_over'],8,1)} | "
              f"{f(r['real_rise'],7)} {f(r['sim_rise'],7)} | "
              f"{f(r['real_settle'],6)} {f(r['sim_settle'],6)} | {f(r['rmse'],7,4)}")

    def agg(axes, kr, ks):
        a = np.array([r[kr] for r in rows if r["axis"] in axes
                      and r[kr] is not None and r[ks] is not None
                      and np.isfinite(r[kr]) and np.isfinite(r[ks])])
        b = np.array([r[ks] for r in rows if r["axis"] in axes
                      and r[kr] is not None and r[ks] is not None
                      and np.isfinite(r[kr]) and np.isfinite(r[ks])])
        return (len(a), a.mean(), b.mean(), (b - a).mean()) if len(a) else None

    print(f"\n{'='*104}\nAGGREGATES (paired)\n{'='*104}")
    print(f"{'group':22} {'metric':16} {'n':>3} {'real':>9} {'sim':>9} {'bias':>9}")
    print("-" * 104)
    for gname, axes in (("position (x,y,z)", ("x", "y", "z")), ("yaw", ("yaw",))):
        for label, kr, ks in (("overshoot [%]", "real_over", "sim_over"),
                              ("rise 10-90 [s]", "real_rise", "sim_rise"),
                              ("settling +-5% [s]", "real_settle", "sim_settle")):
            a = agg(axes, kr, ks)
            if a:
                print(f"{gname:22} {label:16} {a[0]:3d} {a[1]:9.3f} {a[2]:9.3f} "
                      f"{a[3]:+9.3f}")
        rm = np.array([r["rmse"] for r in rows if r["axis"] in axes
                       and np.isfinite(r["rmse"])])
        if len(rm):
            unit = "deg" if axes == ("yaw",) else "m"
            print(f"{gname:22} {'shape RMSE':16} {len(rm):3d} {'':>9} {'':>9} "
                  f"{rm.mean():9.4f}  ({unit}; median {np.median(rm):.4f}, "
                  f"worst {rm.max():.4f})")
        print()

    # ---- whole-run agreement, resampled onto the real timeline ----
    lo = max(real["t"][0], sim["t"][0])
    hi = min(real["t"][-1], sim["t"][-1])
    m = (real["t"] >= lo) & (real["t"] <= hi)
    t = real["t"][m]
    print(f"{'='*104}\nWHOLE RUN ({lo:.1f}..{hi:.1f}s, n={m.sum()})\n{'='*104}")
    print(f"{'quantity':16} {'real RMS':>10} {'sim RMS':>10} {'RMS diff':>10} "
          f"{'bias':>10} {'corr':>7}")
    print("-" * 104)
    chans = ([(f"pos {a}", real["pos"][m, k], np.interp(t, sim["t"], sim["pos"][:, k]),
               "m") for k, a in enumerate("xyz")]
             + [(f"vel {a}", real["vel"][m, k], np.interp(t, sim["t"], sim["vel"][:, k]),
                 "m/s") for k, a in enumerate("xyz")]
             + [(f"eul {a}", real["eul"][m, k], np.interp(t, sim["t"], sim["eul"][:, k]),
                 "deg") for k, a in enumerate(["roll", "pitch", "yaw"])])
    for name, a, b, unit in chans:
        d = b - a
        print(f"{name+' ['+unit+']':16} {np.sqrt((a**2).mean()):10.4f} "
              f"{np.sqrt((b**2).mean()):10.4f} {np.sqrt((d**2).mean()):10.4f} "
              f"{d.mean():+10.4f} {np.corrcoef(a, b)[0,1]:7.3f}")

    with open(f"odom_metrics_{tag}.json", "w") as fh:
        json.dump(rows, fh, indent=2, default=lambda o: None)
    print(f"\nwrote odom_metrics_{tag}.json")


if __name__ == "__main__":
    main()
