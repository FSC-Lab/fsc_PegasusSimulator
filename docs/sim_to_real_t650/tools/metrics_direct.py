#!/usr/bin/env python3
"""Step and whole-run metrics for the DIRECT-actuation comparison, real vs IsaacSim.

Identical method to metrics_odom.py (overshoot, 10-90% rise and +-5% settling measured from
each trace's OWN 10% crossing; shape RMSE after aligning on that crossing with the initial
offset removed), so DIRECT numbers are directly comparable to the baseline campaign's.
Only the loaders differ -- the DIRECT segment is sliced out and t = 0 is set at DIRECT
engagement rather than at the first waypoint.

  metrics_direct.py <tag> <real_npz> <ref_npz> <sim_npz>
"""
import json
import sys

import numpy as np

from metrics_odom import shape_rmse, step_metrics, step_trace
from plot_direct import load_real, load_sim
from plot_odom import detect_steps, usable_steps


def main():
    tag, real_npz, ref_npz, sim_npz = sys.argv[1:5]
    real = load_real(real_npz, ref_npz)
    sim = load_sim(sim_npz)
    steps = usable_steps(
        detect_steps(real["ref_t"], real["ref_pos"], real["ref_yaw"]))

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
    print(f"\n{'='*104}\nPER-STEP, DIRECT actuation, flight {tag}\n{'='*104}")
    print(f"{'#':>2} {'axis':>4} {'target':>7} {'size':>6} | {'over% r':>8} {'over% s':>8} "
          f"| {'rise r':>7} {'rise s':>7} | {'set r':>6} {'set s':>6} | {'RMSE':>7}")
    print("-" * 104)
    for r in rows:
        print(f"{r['idx']:2d} {r['axis']:>4} {r['to']:7.2f} {r['size']:6.2f} | "
              f"{f(r['real_over'],8,1)} {f(r['sim_over'],8,1)} | "
              f"{f(r['real_rise'],7)} {f(r['sim_rise'],7)} | "
              f"{f(r['real_settle'],6)} {f(r['sim_settle'],6)} | {f(r['rmse'],7,4)}")

    def agg(axes, kr, ks):
        pairs = [(r[kr], r[ks]) for r in rows if r["axis"] in axes
                 and r[kr] is not None and r[ks] is not None
                 and np.isfinite(r[kr]) and np.isfinite(r[ks])]
        if not pairs:
            return None
        a = np.array([p[0] for p in pairs])
        b = np.array([p[1] for p in pairs])
        return len(a), a.mean(), b.mean(), (b - a).mean()

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
                 "deg") for k, a in enumerate(["roll", "pitch", "yaw"])]
             + [(f"rate {a}", np.degrees(real["omega"][m, k]),
                 np.degrees(np.interp(t, sim["t"], sim["omega"][:, k])), "deg/s")
                for k, a in enumerate(["p", "q", "r"])])
    summary = {}
    for name, a, b, unit in chans:
        d = b - a
        summary[name] = dict(real_rms=float(np.sqrt((a ** 2).mean())),
                             sim_rms=float(np.sqrt((b ** 2).mean())),
                             rms_diff=float(np.sqrt((d ** 2).mean())),
                             bias=float(d.mean()),
                             corr=float(np.corrcoef(a, b)[0, 1]))
        print(f"{name+' ['+unit+']':16} {np.sqrt((a**2).mean()):10.4f} "
              f"{np.sqrt((b**2).mean()):10.4f} {np.sqrt((d**2).mean()):10.4f} "
              f"{d.mean():+10.4f} {np.corrcoef(a, b)[0,1]:7.3f}")

    out = f"data/direct_metrics_{tag}.json"
    with open(out, "w") as fh:
        json.dump({"steps": rows, "whole_run": summary}, fh, indent=2,
                  default=lambda o: None)
    print(f"\nwrote {out}")


if __name__ == "__main__":
    main()
