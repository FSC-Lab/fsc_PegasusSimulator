#!/usr/bin/env python3
"""Score scripts/fadapt_cells.sh: per cell, adaptation off vs on -- landed, the corridor's peak pitch,
the back-transition's peak pitch, solve failures BEFORE the stop (the post-stop hover's are a known
nuisance, 2026-09-21), altitude band, and the bias the adaptation learned.

    python3 scripts/fadapt_metrics.py [dir]        (default ~/flight_bags/fadapt/cells)
"""
import glob, os, re, sys
import numpy as np

D = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser("~/flight_bags/fadapt/cells")
ROW = re.compile(r"^\s+(\d+\.\d+)\s+(-?\d+\.\d+)\s+(-?\d+\.\d+)\s+(-?\d+\.\d+)\s+(-?\d+\.\d+)\s+(-?\d+)\s+([+-]\d+\.\d+)\s+([+-]\d+\.\d+).*ok=(\d)")


def score(path, t_bt=60.0):
    rows = []
    for line in open(path, errors="replace"):
        m = ROW.match(line)
        if m:
            t, z, V, vcmd, tilt, bcmd, th, aoa, ok = m.groups()
            rows.append((float(t), float(z), float(V), float(th), int(ok)))
    s = open(path, errors="replace").read()
    ground = re.search(r"GROUND at t=([\d.]+)", s)
    if not rows:
        return None
    a = np.array(rows)
    cor = a[(a[:, 0] >= 2.0) & (a[:, 0] <= 25.0)]
    bt = a[a[:, 0] >= t_bt]
    stop = next((r[0] for r in bt if r[2] < 3.0), None)
    t_end = stop if stop is not None else t_bt + 20.0
    fails_flight = int(((a[:, 4] == 0) & (a[:, 0] < t_end)).sum())
    fb = re.search(r"bias \(a_s, a_z\) after t 30 s mean \(([+-][\d.]+), ([+-][\d.]+)\)", s)
    landed = ground is None and stop is not None
    return dict(landed=landed, cor=cor[:, 3].max() if len(cor) else np.nan,
                btpk=bt[:, 3].max() if len(bt) else np.nan, ff=fails_flight,
                zmin=a[a[:, 0] > 2.0][:, 1].min(), zmax=a[a[:, 0] > 2.0][:, 1].max(),
                fb=(float(fb.group(1)), float(fb.group(2))) if fb else None)


cells = sorted({re.sub(r"_(on|off)$", "", os.path.basename(p)) for p in glob.glob(os.path.join(D, "*_o[nf]*")) if os.path.isdir(p)})
print(f"{'cell':12s} {'adapt':5s} {'landed':6s} {'corridor pk':>11s} {'BT pk':>6s} {'flight fails':>12s} {'z band':>12s}  bias (a_s, a_z) m/s^2")
for c in cells:
    for mode in ("off", "on"):
        runs = [score(p) for p in sorted(glob.glob(os.path.join(D, f"{c}_{mode}", "s*.txt")))]
        runs = [r for r in runs if r is not None]
        if not runs:
            continue
        n_ok = sum(r["landed"] for r in runs)
        fl = [r for r in runs if r["landed"]] or runs
        fbs = [r["fb"] for r in runs if r["fb"]]
        fbt = f"({np.mean([f[0] for f in fbs]):+.3f}, {np.mean([f[1] for f in fbs]):+.3f})" if fbs else "-"
        print(f"{c:12s} {mode:5s} {n_ok}/{len(runs):<4d} {max(r['cor'] for r in fl):11.1f} {max(r['btpk'] for r in fl):6.1f} "
              f"{sum(r['ff'] for r in runs):12d} {min(r['zmin'] for r in fl):5.1f}-{max(r['zmax'] for r in fl):5.1f}  {fbt}")
