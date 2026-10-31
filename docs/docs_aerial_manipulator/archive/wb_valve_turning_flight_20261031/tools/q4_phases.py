#!/usr/bin/env python3
"""q4 (wrist roll) through each 2026-10-31 valve flight, by phase: std / p-p / dominant frequency of the
measured q4, the law's j4 torque [16], the law's heading error e_y[3] [27], the free-axis weight / offset
[116]/[117], the heading-radius offset [119], and the arm controller's j4 error-damping duty law[31]."""
import sys, numpy as np
from vt_common import Bag, BAGS

def dom_freq(t, x, fmin=0.15, fmax=8.0):
    if len(t) < 64: return float("nan"), float("nan")
    tu = np.arange(t[0], t[-1], 0.01); xu = np.interp(tu, t, x); xu = xu - np.polyval(np.polyfit(tu, xu, 1), tu)
    w = np.hanning(len(xu)); F = np.fft.rfft(xu * w); f = np.fft.rfftfreq(len(xu), 0.01)
    m = (f >= fmin) & (f <= fmax); i = np.argmax(np.abs(F[m])); return f[m][i], 2*np.abs(F[m][i])/w.sum()

for tag in (sys.argv[1:] or BAGS):
    b = Bag(tag)
    names = [s.strip() for s in str(b.f("js", "name")[0]).replace("[","").replace("]","").replace("'","").split(",")] \
        if b.f("js","name").ndim == 1 else list(b.f("js","name")[0])
    tj = b.t("js"); P = b.f("js", "position"); i4 = names.index("joint4"); i1 = names.index("joint1")
    q4 = np.degrees(P[:, i4]); q1 = np.degrees(P[:, i1])
    tw = b.t("wb"); W = b.f("wb", "data")
    L = np.array([len(r) for r in W]); full = L >= 120
    tl = b.t("law"); LD = np.array([r if len(r) >= 32 else np.full(32, np.nan) for r in b.f("law","data")])
    # phases from the push-pull status edges
    ed = b.edges("pl_pstatus")
    marks = []
    for t, s in ed:
        k = s.split(" -- ")[0].split(" (")[0]
        if not marks or marks[-1][1] != k: marks.append((t, k))
    print(f"===== {tag}  joints {names}")
    tdir = [t for t, s in b.edges("wbmode") if s == "DIRECT"]
    bounds = [(tj[0], tdir[0] if tdir else tj[-1], "SAFETY before DIRECT")]
    for k, (t, s) in enumerate(marks):
        t1 = marks[k+1][0] if k+1 < len(marks) else tj[-1]
        if t1 - t > 1.0: bounds.append((t, t1, s))
    print(f"{'phase':46s} {'dur':>5s} {'q4 mean':>8s} {'q4 std':>7s} {'q4 p-p':>7s} {'f[Hz]':>6s} {'amp':>5s} | {'q1 std':>6s} | {'tau4 std':>8s} {'eh4 std':>7s} | {'wfree':>5s} {'off mm':>7s} {'hdg deg':>7s} | {'errdamp4':>8s}")
    for t0, t1, s in bounds:
        m = (tj >= t0) & (tj <= t1)
        if m.sum() < 50: continue
        f, a = dom_freq(tj[m], q4[m])
        mw = (tw >= t0) & (tw <= t1) & full
        if mw.sum() > 10:
            Wm = np.array([r[:120] for r in W[mw]])
            tau4 = Wm[:, 16].std(); eh = np.degrees(np.arcsin(np.clip(Wm[:, 27], -1, 1))).std()
            wf = Wm[:, 116].max(); off = 1e3*np.abs(Wm[:, 117]).max(); hd = np.degrees(np.abs(Wm[:, 119]).max())
        else: tau4 = eh = wf = off = hd = float("nan")
        ml = (tl >= t0) & (tl <= t1)
        ed4 = np.nanstd(LD[ml, 31]) if ml.sum() > 10 else float("nan")
        print(f"{s[:46]:46s} {t1-t0:5.1f} {q4[m].mean():+8.1f} {q4[m].std():7.2f} {np.ptp(q4[m]):7.1f} {f:6.2f} {a:5.1f} | {q1[m].std():6.2f} | {tau4:8.3f} {eh:7.2f} | {wf:5.2f} {off:7.1f} {hd:7.1f} | {ed4:8.2f}")
