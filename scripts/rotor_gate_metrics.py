#!/usr/bin/env python3
"""Score scripts/rotor_gate.sh's turning-mission cells, split at the RETURN-TO-HOME STOP (the guidance's
`RTH STOP t=` line; in wind the ground speed after the stop is the drift, so a speed test never fires):
missions completed through the stop, seeds lost AFTER it (the cascade still hovering -- in a real flight the
safe controller has taken over), solve failures before it, the front pair's throttle peak, z min.
    python3 scripts/rotor_gate_metrics.py [tags...]      (~/flight_bags/rotor_trade/wind/<tag>)
"""
import glob, os, re, sys, numpy as np
D = os.path.expanduser(os.environ.get("GATE_WIND_DIR") or "~/flight_bags/rotor_trade/wind")   # GATE_WIND_DIR: another gate's wind dir
ROW = re.compile(r"^\s+([\d.]+)\s+([\d.-]+)\s+([\d.]+)\s+([\d.]+)\s+([\d.-]+)\s+([\d.-]+)\s+([+\-\d.]+)\s+([+\-\d.]+)\s+([+\-\d.]+).*?ok=(\d)(.*)$")
def one(path):
    rows, mots = [], []
    for ln in open(path, errors="replace"):
        m = ROW.match(ln)
        if m:
            rows.append((float(m.group(1)), float(m.group(2)), float(m.group(3)), float(m.group(4)), int(m.group(10))))
            mm = re.search(r"mot=\[([\d.,]+)\]", m.group(11))
            mots.append([float(x) for x in mm.group(1).split(",")] if mm else [np.nan] * 4)
    if not rows: return None
    a = np.array(rows); mo = np.array(mots)
    s = open(path, errors="replace").read()
    g = re.search(r"GROUND at t=([\d.]+)", s); tg = float(g.group(1)) if g else None
    ms = re.search(r"RTH STOP t=([\d.]+)", s)            # the guidance's own stop (ground speed in wind is the drift)
    st = float(ms.group(1)) if ms else next((r[0] for r in rows if r[0] > 100 and r[3] == 0.0 and r[2] < 1.0), None)
    done = "solve fails" in s
    te = st if st is not None else (tg if tg is not None else a[-1, 0])
    fl = a[:, 0] < te
    thr = np.nanmax(mo[fl & (a[:, 0] > 20), :2]) if (fl & (a[:, 0] > 20)).any() else np.nan
    return dict(reached=st is not None and (tg is None or tg > st), ground=tg is not None, post=tg is not None and st is not None and tg > st,
                ff=int(((a[:, 4] == 0) & fl).sum()), done=done, thr=thr, zmin=a[(a[:, 0] > 5) & fl, 1].min())
tags = sys.argv[1:] or ["ref", "b15x8", "a16x8", "d4014_16x8", "c14x10"]
print(f"{'build':11s} {'cell':12s} {'mission':>8s} {'lost after stop':>15s} {'fails in flight':>15s} {'front thr max':>13s} {'z min':>6s}")
for t in tags:
    for c in ("still", "turb05", "turb10", "turb10_vc18"):
        rs = [one(p) for p in sorted(glob.glob(os.path.join(D, t, f"{c}_s*.txt")))]
        rs = [r for r in rs if r and r["done"]]
        if not rs: continue
        print(f"{t:11s} {c:12s} {sum(r['reached'] for r in rs)}/{len(rs):<6d} {sum(r['post'] for r in rs):15d} {sum(r['ff'] for r in rs):15d} {np.nanmax([r['thr'] for r in rs]):13.2f} {min(r['zmin'] for r in rs):6.1f}")
