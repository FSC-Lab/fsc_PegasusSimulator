#!/usr/bin/env python3
"""Per-step quantitative metrics, real vs simulated, for the T650 baseline controller.

Every metric is measured on the baseline-removed response and, where the earlier X650
campaign's method applies, anchored on the signal's own 10% crossing rather than on
commanded onset -- so residual onset jitter cannot contaminate it and the persistent hover
trim (which differs in sign between the two vehicles) drops out.
"""
import json
import sys

import numpy as np

from plot_steps import channel, load_side, smooth, win
from steps import detect_steps, ref_onsets

PRE, POST_MAX = 1.5, 13.0


def response(side, st, fc=6.0):
    """Baseline-removed response for this step, plus its time axis."""
    want = "cmd" if st["axis"] == "z" else "meas"
    t, y, _ = channel(side, st, want)
    hold = min(st["hold"] or POST_MAX, POST_MAX)
    tw, yw = win(t, y, st["t"], PRE, hold)
    if len(tw) < 20:
        return None, None
    yw = smooth(tw, yw, fc)
    base = yw[tw < -0.2]
    return tw, yw - (base.mean() if len(base) else 0.0)


def peak_metrics(t, y):
    """Peak of the initial transient, its timing, and 10-90% rise on that leading edge."""
    post = t >= 0
    tp, yp = t[post], y[post]
    if len(tp) < 10:
        return {}
    k = np.argmax(np.abs(yp))
    pk = yp[k]
    s = np.sign(pk) if pk != 0 else 1.0
    sig = s * yp[: k + 1]
    out = {"peak": float(pk), "t_peak": float(tp[k])}
    for lo, hi, name in ((0.1, 0.9, "rise_10_90"),):
        a = np.where(sig >= lo * abs(pk))[0]
        b = np.where(sig >= hi * abs(pk))[0]
        if len(a) and len(b):
            out[name] = float(tp[b[0]] - tp[a[0]])
            out["t10"] = float(tp[a[0]])
    return out


def settling(t, y, tol_frac=0.05):
    """Time from the 10% crossing until |y| stays inside tol_frac*|peak| for good."""
    m = peak_metrics(t, y)
    if "t10" not in m:
        return None
    tol = tol_frac * abs(m["peak"])
    post = t >= m["t10"]
    tp, yp = t[post], y[post]
    bad = np.where(np.abs(yp) > tol)[0]
    if len(bad) == 0:
        return 0.0
    if bad[-1] + 1 >= len(tp):
        return None  # never settled inside the available hold
    return float(tp[bad[-1] + 1] - m["t10"])


def undershoot(t, y):
    """Magnitude of the opposite-sign excursion after the initial peak (the decelerate
    phase for a translational step) -- a damping indicator."""
    m = peak_metrics(t, y)
    if "t_peak" not in m:
        return None
    s = np.sign(m["peak"])
    post = t > m["t_peak"]
    if post.sum() < 5:
        return None
    return float(-s * np.min(s * y[post]))


def analyse(tag, real_npz, sim_npz):
    t_ref, ref, _ = ref_onsets(real_npz, f"ref_{tag}.npz")
    steps = detect_steps(t_ref, ref)
    R, S = load_side(real_npz, "real"), load_side(sim_npz, "sim")

    rows = []
    for st in steps:
        tr, yr = response(R, st)
        ts, ys = response(S, st)
        if tr is None or ts is None:
            continue
        mr, ms = peak_metrics(tr, yr), peak_metrics(ts, ys)

        # shape RMSE over a common 10 s horizon, each side anchored on its own 10%
        # crossing so onset jitter does not leak into it
        rmse = np.nan
        if "t10" in mr and "t10" in ms:
            grid = np.arange(0.0, 10.0, 0.02)
            a = np.interp(grid, tr - mr["t10"], yr)
            b = np.interp(grid, ts - ms["t10"], ys)
            rmse = float(np.sqrt(np.mean((b - a) ** 2)))

        rows.append(dict(
            tag=tag, idx=st["idx"], axis=st["axis"], size=st["size"],
            real_peak=mr.get("peak"), sim_peak=ms.get("peak"),
            real_rise=mr.get("rise_10_90"), sim_rise=ms.get("rise_10_90"),
            real_under=undershoot(tr, yr), sim_under=undershoot(ts, ys),
            real_settle=settling(tr, yr), sim_settle=settling(ts, ys),
            rmse=rmse,
        ))
    return rows


def agg(rows, axes, key_r, key_s, scale=1.0, magnitude=False):
    a = np.array([r[key_r] for r in rows if r["axis"] in axes and r[key_r] is not None
                  and r[key_s] is not None], dtype=float)
    b = np.array([r[key_s] for r in rows if r["axis"] in axes and r[key_r] is not None
                  and r[key_s] is not None], dtype=float)
    if len(a) == 0:
        return None
    if magnitude:
        # Steps come in +/- pairs; averaging signed peaks cancels them to ~0.
        a, b = np.abs(a), np.abs(b)
    return dict(n=len(a), real=a.mean() * scale, sim=b.mean() * scale,
                bias=(b - a).mean() * scale,
                pct=100.0 * (b - a).mean() / abs(a.mean()) if a.mean() else np.nan)


def main():
    rows = []
    for tag, sim in (("A", "sim_stack_A.npz"), ("B", "sim_stack_B.npz")):
        rows += analyse(tag, f"exp_{tag}.npz", sim)

    print(f"\n{'='*118}\nPER-STEP METRICS (baseline removed; peak/rise/undershoot on the "
          f"step's own response)\n{'='*118}")
    hdr = (f"{'flt':>3} {'#':>2} {'axis':>4} {'size':>6} | {'peak r':>8} {'peak s':>8} "
           f"{'d%':>7} | {'rise r':>7} {'rise s':>7} {'d':>6} | {'undr r':>7} {'undr s':>7} "
           f"| {'set r':>6} {'set s':>6} | {'rmse':>7}")
    print(hdr); print("-" * 118)
    f = lambda v, w, p=3: (f"{v:{w}.{p}f}" if v is not None and np.isfinite(v)
                           else f"{'-':>{w}}")
    for r in rows:
        d = (100 * (r["sim_peak"] - r["real_peak"]) / abs(r["real_peak"])
             if r["real_peak"] else None)
        dr = (r["sim_rise"] - r["real_rise"]
              if r["real_rise"] is not None and r["sim_rise"] is not None else None)
        print(f"{r['tag']:>3} {r['idx']:2d} {r['axis']:>4} {r['size']:6.2f} | "
              f"{f(r['real_peak'],8)} {f(r['sim_peak'],8)} {f(d,7,1)} | "
              f"{f(r['real_rise'],7,2)} {f(r['sim_rise'],7,2)} {f(dr,6,2)} | "
              f"{f(r['real_under'],7)} {f(r['sim_under'],7)} | "
              f"{f(r['real_settle'],6,2)} {f(r['sim_settle'],6,2)} | {f(r['rmse'],7)}")

    print(f"\n{'='*118}\nAGGREGATES (paired: a step counts only where both sides produced "
          f"a value)\n{'='*118}")
    print(f"{'group':28} {'metric':14} {'n':>3} {'real':>9} {'sim':>9} {'bias':>9} {'bias %':>8}")
    print("-" * 118)
    groups = [("horizontal (x,y) steps", ("x", "y")), ("vertical (z) steps", ("z",)),
              ("yaw steps", ("yaw",))]
    for gname, axes in groups:
        for label, kr, ks, mag in (("|peak|", "real_peak", "sim_peak", True),
                                   ("rise 10-90 [s]", "real_rise", "sim_rise", False),
                                   ("|undershoot|", "real_under", "sim_under", True),
                                   ("settling [s]", "real_settle", "sim_settle", False),
                                   ("shape RMSE", "rmse", "rmse", False)):
            a = agg(rows, axes, kr, ks, magnitude=mag)
            if a is None:
                continue
            if kr == ks:
                print(f"{gname:28} {label:14} {a['n']:3d} {'':>9} {'':>9} "
                      f"{a['real']:9.4f} {'':>8}")
            else:
                print(f"{gname:28} {label:14} {a['n']:3d} {a['real']:9.4f} {a['sim']:9.4f} "
                      f"{a['bias']:+9.4f} {a['pct']:+7.1f}%")
        print()

    with open("step_metrics.json", "w") as fh:
        json.dump(rows, fh, indent=2, default=lambda o: None)
    print("wrote step_metrics.json")


if __name__ == "__main__":
    main()
