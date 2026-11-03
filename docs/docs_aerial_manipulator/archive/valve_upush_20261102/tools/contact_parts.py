#!/usr/bin/env python3
"""Which claw part touched the valve, where, and how it pushed (2026-11-02, archive/valve_upush_20261102).
Reads runs/<tag>.contacts.csv (07's PEGASUS_VALVE_CONTACT_LOG) + runs/<tag>.npz (the push marks).
Contact points are expressed in the ROTOR's frame (x along the beam toward the fin, z up, origin the
rotor body origin): r = radius, s = along the fin (+ = the turn's direction), h = height."""
import sys, os, numpy as np
from collections import defaultdict
R = sys.argv[1] if len(sys.argv) > 1 and os.path.isdir(sys.argv[1]) else \
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "sim", "runs")
tags = [a for a in sys.argv[1:] if not os.path.isdir(a)]
DT = 0.004

def qrot(w, x, y, z):
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)], [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])

def mark(d, name):
    i = np.where(d["marks_name"] == name)[0]
    return float(d["marks_t"][i[0]]) if len(i) else None

for tag in tags:
    rows, rot = [], {}
    for line in open(os.path.join(R, tag + ".contacts.csv")):
        if line.startswith("#"): continue
        f = line.strip().split(",")
        if f[0] == "R": rot[f[1]] = np.array(list(map(float, f[2:])))
        elif f[0] == "C": rows.append(f)
    d = np.load(os.path.join(R, tag + ".npz"), allow_pickle=True)
    t0, t1 = mark(d, "push:start"), mark(d, "push:end")
    # the scene logs on ITS clock, the driver on its own: align on the first contact episode
    # (the contact truth topic in the npz carries the driver's receive times)
    c = d["contact"]; withpts = c[c[:, 20] > 0, 0]
    off = (min(float(f[1]) for f in rows) - float(withpts.min())) if (rows and len(withpts)) else 0.0
    print(f"   clock offset scene - driver {off:+.2f} s")
    print(f"== {tag}: {len(rows)} contact points logged; push {t0}..{t1}")
    agg = defaultdict(list)
    for f in rows:
        t = float(f[1]) - off; key = f[1]
        if key not in rot: continue
        P = rot[key]; Rm = qrot(*P[3:7]); o = P[:3]
        p = Rm.T @ (np.array(list(map(float, f[4:7]))) - o)
        imp = Rm.T @ np.array(list(map(float, f[10:13])))
        phase = "turn" if (t0 is not None and t1 is not None and t0 <= t <= t1) else ("before" if t0 is not None and t < t0 else "after")
        link = f[2].strip("/").split("/")[-1] if f[2] else f[2]
        agg[(phase, f[2])].append((t, *p, *(imp / DT)))
    nsteps = {}
    for f in rows:
        t = float(f[1]) - off
        ph = "turn" if (t0 is not None and t1 is not None and t0 <= t <= t1) else ("before" if t0 is not None and t < t0 else "after")
        nsteps.setdefault(ph, set()).add(f[1])
    for (phase, link), v in sorted(agg.items()):
        a = np.array(v)
        F = a[:, 4:7]
        Fs = F.sum(0); Fm = np.abs(F).sum(0)
        steps = len(set(np.round(a[:, 0], 4)))
        nrm = np.linalg.norm(F, axis=1)
        if nrm.sum() > 1e-9:
            dirn = F.sum(0) / nrm.sum()            # force-weighted mean direction (1 = all along one axis)
            print(f"  {phase:6s} {link:32s} pushes on {steps:4d} logged steps: per-step force on the valve "
                  f"radial {Fs[0]/max(steps,1):+5.2f} along-the-turn {Fs[1]/max(steps,1):+5.2f} vertical {Fs[2]/max(steps,1):+5.2f} N; "
                  f"direction r/t/z {dirn[0]:+.2f}/{dirn[1]:+.2f}/{dirn[2]:+.2f}; height z {np.percentile(a[:,3],10)*1e3:4.0f}..{np.percentile(a[:,3],90)*1e3:4.0f} mm")
        r = np.hypot(a[:, 1], a[:, 2])
        print(f"  {phase:6s} {link:48s} n {len(a):5d} t {a[:,0].min():6.1f}..{a[:,0].max():6.1f}  "
              f"pos x {np.median(a[:,1])*1e3:6.0f} y {np.median(a[:,2])*1e3:6.0f} z {np.median(a[:,3])*1e3:5.0f} mm "
              f"| mean F on valve x {F[:,0].mean():+5.2f} y {F[:,1].mean():+5.2f} z {F[:,2].mean():+5.2f} N")
