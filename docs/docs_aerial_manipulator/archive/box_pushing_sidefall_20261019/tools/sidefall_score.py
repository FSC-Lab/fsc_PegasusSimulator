#!/usr/bin/env python3
"""Score one 2026-10-19 box-pushing run, aimed at the SIDE-EDGE fall (the box dragged
over the table's long edge with the jaws closed on the fin).

    PYTHONNOUSERSITE=1 /usr/bin/python3 sidefall_score.py <tag> [<tag> ...] [--timeline]

Per run: outcome and the first guard; where the box's CoM was relative to the table's
side edge (x 1.350) when it tripped; the box's tilt; how much of the box's weight the
claw carried (PhysX contact truth, vehicle -> box, upward part); when the jaws opened;
the vehicle's peak tilt and sideways excursion from the push start to the run's end;
q2 / q3 extremes and the peak applied arm torque; where the box ended (table / floor).
--timeline prints the 0.25 s story around the trip.
"""
import argparse, os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); RUNS = os.path.join(HERE, "..", "runs")
sys.path.insert(0, os.path.join(HERE, "..", "..", "push_pull_20261003", "tools"))
from pl_metrics import mark  # noqa: E402
from pl_score import tilt_deg  # noqa: E402

TABLE_X = (1.200, 1.350)        # the box-pushing table's depth (07 scene, 2026-10-16)
TABLE_TOP = 0.800
G = 9.81


def lp(t, x, tau):
    y = np.array(x, dtype=float)
    for i in range(1, len(y)):
        a = np.exp(-max(t[i] - t[i - 1], 0.0) / tau)
        y[i] = a * y[i - 1] + (1 - a) * x[i]
    return y


def base(tag):
    """a tag in this campaign's runs/, or a path to another campaign's run (no .npz)"""
    return tag[:-4] if tag.endswith(".npz") else (tag if os.sep in tag else os.path.join(RUNS, tag))


def args_of(tag):
    p = base(tag) + ".args"
    kv = {}
    if os.path.exists(p):
        for line in open(p):
            if "=" in line:
                k, v = line.strip().split("=", 1); kv[k] = v
    return kv


def run(tag, timeline):
    z = np.load(base(tag) + ".npz", allow_pickle=True)
    kv = args_of(tag); m = float(kv.get("MASS", "nan"))
    ps = mark(z, "push:start")
    b, o, bt, j, g, ct = z["box"], z["odom"], z["base_truth"], z["joints"], z["grip"], z["contact"]
    ev = [str(e) for e in z["events"]]
    t_of = lambda s: float(s[1:s.index("s]")])
    guard = [e for e in ev if "SAFETY GUARD" in e or "GUARD" in e and "push_pull:" in e]
    t_trip = t_of(guard[0]) if guard else None
    left = [e for e in ev if "left DIRECT" in e or "node left DIRECT" in e]
    opened = [e for e in ev if "gripper OPEN" in e]
    t_open_cmd = t_of(opened[0]) if opened else None
    # the jaws' measured opening (06's gripper position, rad: closed ~ -0.2..-0.35, open 0)
    t_open = None
    if t_open_cmd is not None and len(g):
        w = (g[:, 0] >= t_open_cmd) & (g[:, 1] > -0.05)
        t_open = float(g[w, 0][0]) if w.any() else None
    # contact truth: on the box BY the vehicle, normal + friction anchors, sign calibrated
    # on the table (its normal must hold the box up)
    tn = ct[:, 7:10]
    sn = 1.0 if np.nanmean(tn[np.abs(tn[:, 2]) > 0.5, 2]) > 0 else -1.0
    Fv = sn * (ct[:, 1:4] + ct[:, 4:7])
    Fup = lp(ct[:, 0], Fv[:, 2], 0.2)
    Ntab = lp(ct[:, 0], sn * tn[:, 2], 0.2)
    t0 = ps if ps is not None else b[0, 0]
    t1 = min(b[-1, 0], o[-1, 0])
    # the clamped span: push start -> the jaws open (abort) or the release (completed)
    t_rel = t_open or mark(z, "gripper_release:start") or mark(z, "release:start") or t1
    wc = (ct[:, 0] >= t0) & (ct[:, 0] <= t_rel)
    # the vehicle's span: push start -> it leaves the table (Go To Land Start) or lands
    ends = [mark(z, "go_to_land_start:start")] + [t_of(e) for e in ev if "after the abort: Go To Land Start" in e
                                                 or "landing by reference" in e]
    tv = min([x for x in ends if x is not None and x > t0] + [t1])
    # the box
    bx = b[:, 1]; bz = b[:, 3]
    btilt = tilt_deg(b[:, 4], b[:, 5], b[:, 6], b[:, 7])
    cross = b[(b[:, 0] >= t0) & (bx >= TABLE_X[1]), 0]
    t_cross = float(cross[0]) if len(cross) else None
    ip = lambda arr, k, t: float(np.interp(t, arr[:, 0], arr[:, k]))
    # the vehicle (truth)
    wv = (bt[:, 0] >= t0) & (bt[:, 0] <= tv)
    vt = tilt_deg(bt[wv, 4], bt[wv, 5], bt[wv, 6], bt[wv, 7])
    p0 = bt[wv][0, 1:3]
    side = np.max(np.abs(bt[wv, 1] - p0[0])) if wv.any() else np.nan
    wj = (j[:, 0] >= t0) & (j[:, 0] <= tv)
    q2, q3 = np.degrees(j[wj, 2]), np.degrees(j[wj, 3])
    tau = np.abs(j[wj, 5:9]).max(axis=0)
    mode_s = o[(o[:, 0] >= t0) & (o[:, 0] <= tv) & (o[:, 11] < 0.5), 0]
    w0, x0, y0, z0 = b[0, 4:8]
    yaw0 = np.degrees(np.arctan2(2 * (w0 * z0 + x0 * y0), 1 - 2 * (y0 * y0 + z0 * z0)))
    print(f"== {os.path.basename(tag)}  box {m * 1e3:.0f} g, box yaw {yaw0:.1f} deg")
    print(f"   outcome : {'ABORTED' if bool(z['aborted']) else 'completed'}  {str(z['reason'])[:120]}")
    print(f"   guard   : {guard[0][:150] if guard else '-'}")
    if t_trip is not None:
        print(f"             trip {t_trip - t0:.2f} s into the push; box CoM x {ip(b, 1, t_trip) - TABLE_X[1]:+.3f} m "
              f"from the side edge, box tilt {np.interp(t_trip, b[:, 0], btilt):.1f} deg")
    if t_cross is not None:
        print(f"   CoM over the side edge at {t_cross - t0:.2f} s into the push")
    print(f"   claw carried (up, 0.2 s lp) max {np.max(Fup[wc]):.2f} N of the box's {m * G:.2f} N weight; "
          f"table normal min {np.min(Ntab[wc]):.2f} N")
    if t_open_cmd is not None:
        print(f"   jaws    : OPEN commanded {t_open_cmd - (t_trip or t_open_cmd):+.2f} s after the trip, measured open "
              f"{(t_open - t_open_cmd) if t_open else float('nan'):.2f} s later")
    print(f"   vehicle : peak tilt {vt.max():.1f} deg, sideways {side * 1e3:.0f} mm, "
          f"{'LEFT DIRECT at ' + format(mode_s[0] - t0, '.1f') + ' s' if len(mode_s) else 'in DIRECT throughout'} "
          f"(push start -> leaving the table, {tv - t0:.0f} s)")
    print(f"   arm     : q2 {q2.min():.1f}..{q2.max():.1f}  q3 {q3.min():.1f}..{q3.max():.1f} deg, "
          f"peak |tau| {np.round(tau, 2).tolist()} N.m")
    zf = b[-1, 3]
    print(f"   box end : ({b[-1, 1]:.3f}, {b[-1, 2]:.3f}, {zf:.3f}) "
          f"{'ON THE TABLE' if zf > TABLE_TOP else 'ON THE FLOOR / fallen'}, tilt {btilt[-1]:.0f} deg, "
          f"lateral {1e3 * (b[-1, 1] - b[0, 1]):+.0f} mm")
    if timeline and (t_trip or t_cross):
        tc = t_trip or t_cross
        print(f"   {'t-trip':>6s} {'box dx':>7s} {'btilt':>5s} {'box z':>6s} {'F up':>5s} {'N tab':>5s} {'grip':>6s} "
              f"{'vtilt':>5s} {'q2':>5s} {'q3':>5s} {'tau2':>5s} {'tau3':>5s} mode")
        for t in np.arange(tc - 2.0, min(tc + 6.0, t1), 0.25):
            kc = min(np.searchsorted(ct[:, 0], t), len(ct) - 1)
            kb = min(np.searchsorted(bt[:, 0], t), len(bt) - 1)
            kj = min(np.searchsorted(j[:, 0], t), len(j) - 1)
            ko = min(np.searchsorted(o[:, 0], t), len(o) - 1)
            print(f"   {t - tc:6.2f} {1e3 * (ip(b, 1, t) - b[0, 1]):7.0f} {np.interp(t, b[:, 0], btilt):5.1f} "
                  f"{ip(b, 3, t):6.3f} {Fup[kc]:5.2f} {Ntab[kc]:5.2f} {ip(g, 1, t):6.3f} "
                  f"{float(tilt_deg(*bt[kb, 4:8])):5.1f} {np.degrees(j[kj, 2]):5.1f} {np.degrees(j[kj, 3]):5.1f} "
                  f"{j[kj, 6]:5.2f} {j[kj, 7]:5.2f} {'D' if o[ko, 11] > 0.5 else 'S'}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("tags", nargs="+"); ap.add_argument("--timeline", action="store_true")
    a = ap.parse_args()
    for t in a.tags:
        run(t, a.timeline)
