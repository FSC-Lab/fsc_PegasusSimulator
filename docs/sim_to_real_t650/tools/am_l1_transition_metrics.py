#!/usr/bin/env python3
"""A/B scorer for the AM-T650 GEOMETRIC+L1 tuning campaign (2026-08-24).

Focused on the three things under tune, in the order they matter:
  1. SAFETY->DIRECT entry transient   (L1 must learn the kf mismatch from zero)
  2. DIRECT->SAFETY exit balloon      (the SAFETY UDE dumps its DIRECT-mode estimate)
  3. DIRECT step response + steady-state hold

Complements am_l1_robustness_metrics.py, which scores the injection accounting.
This one scores the TRANSITIONS, which that script only reports as a single number.

  /usr/bin/python3 am_l1_transition_metrics.py runX.npz [runY.npz ...]
"""
import sys
import numpy as np

G = 9.81


def ev_t(ev, key):
    for e in ev:
        if key in str(e):
            return float(str(e).split("[")[1].split("s]")[0])
    return None


def phases(ev):
    ph = {}
    for e in ev:
        s = str(e)
        if "PHASE " in s:
            ph.setdefault(s.split("PHASE ")[1].split()[0],
                          float(s.split("[")[1].split("s]")[0]))
    return ph


def step_metrics(t, sig, ref, t0, t1):
    """rise 10-90, overshoot %, error at end of window."""
    m = (t >= t0) & (t < t1)
    if m.sum() < 10:
        return None
    s, r = sig[m], ref[m]
    tt = t[m] - t0
    s0, s1 = s[0], r[-1]
    d = s1 - s0
    if abs(d) < 1e-6:
        return None
    frac = (s - s0) / d
    try:
        i10 = np.where(frac >= 0.1)[0][0]
        i90 = np.where(frac >= 0.9)[0][0]
        rise = tt[i90] - tt[i10]
    except IndexError:
        rise = np.nan
    over = 100.0 * (frac.max() - 1.0) if d > 0 else 100.0 * (frac.max() - 1.0)
    return rise, over, (s[-1] - s1) * 1000.0


def score(path):
    f = np.load(path, allow_pickle=True)
    log, dbg, ev = f["log"], f["dbg"], f["events"]
    if len(log) == 0:
        print(f"\n### {path}: EMPTY (aborted run)")
        return
    t, x, y, z = log[:, 0], log[:, 1], log[:, 2], log[:, 3]
    vz, tilt = log[:, 6], log[:, 7]
    rx, ry, rz = log[:, 8], log[:, 9], log[:, 10]
    ph = phases(ev)
    tD = ev_t(ev, "mode -> DIRECT")
    tS = ph.get("SAFETY_SETTLE")
    exy = np.hypot(x - rx, y - ry)
    ez = z - rz

    print(f"\n{'='*76}\n### {path.split('/')[-1]}")

    # ---- 1. entry -------------------------------------------------------
    if tD is not None:
        m = (t >= tD) & (t < tD + 20)
        tt, e_z, e_xy, ti = t[m] - tD, ez[m], exy[m], tilt[m]
        # settle = last time |ez|>20mm or |exy|>50mm
        bad = (np.abs(e_z) > 0.020) | (e_xy > 0.050)
        settle = tt[bad][-1] if bad.any() else 0.0
        print(f"  ENTRY  SAFETY->DIRECT @ {tD:6.1f}s")
        print(f"    z sag        {e_z.min()*1000:8.1f} mm   |ez| max {np.abs(e_z).max()*1000:7.1f} mm")
        print(f"    xy excursion {e_xy.max()*1000:8.1f} mm")
        print(f"    tilt peak    {ti.max():8.2f} deg")
        print(f"    settle(20mm/50mm) {settle:6.1f} s")

    # ---- 2. exit --------------------------------------------------------
    if tS is not None:
        # END THE WINDOW AT LAND_WAIT. The driver drops the reference to the landing
        # altitude a few seconds after the abort, so a fixed 25 s window scores the
        # (intended) descent as if it were balloon -- which reads as ~557 mm on a run
        # whose real balloon is 136 mm.
        tEnd = min(ph.get("LAND_WAIT", tS + 25), tS + 25)
        m = (t >= tS) & (t < tEnd)
        tt, zz, e_z, vv, ti = t[m] - tS, z[m], ez[m], vz[m], tilt[m]
        back = np.where(np.abs(e_z) < 0.050)[0]
        # first index AFTER the peak where it is back inside 50 mm
        ipk = int(np.argmax(e_z))
        aft = back[back > ipk]
        trec = tt[aft[0]] if len(aft) else np.nan
        print(f"  EXIT   DIRECT->SAFETY @ {tS:6.1f}s")
        print(f"    balloon      {e_z.max()*1000:8.1f} mm   (peak z {zz.max():.3f} m)")
        print(f"    peak vz      {vv.max():8.3f} m/s   undershoot {e_z.min()*1000:7.1f} mm")
        print(f"    tilt peak    {ti.max():8.2f} deg    recover<50mm {trec:5.1f} s")

    # ---- 3. steps + steady ---------------------------------------------
    for name, sig, ref in (("X", x, rx), ("Z", z, rz)):
        for k in (f"STEP{name}", f"STEP{name}_RET"):
            if k not in ph:
                continue
            nxt = min([v for v in ph.values() if v > ph[k]], default=t[-1])
            r = step_metrics(t, sig, ref, ph[k], nxt)
            if r:
                print(f"  STEP {k:<12} rise {r[0]:5.2f} s  overshoot {r[1]:6.1f} %"
                      f"  err@end {r[2]:7.1f} mm")

    if "SOAK2" in ph:
        a = ph["SOAK2"]
        b = min([v for v in ph.values() if v > a], default=t[-1])
        m = (t >= a) & (t < b)
        if m.sum() > 5:
            en = np.linalg.norm(np.stack([x[m]-rx[m], y[m]-ry[m], z[m]-rz[m]]), axis=0)
            print(f"  STEADY SOAK2   |e| rms {en.mean()*1000:6.1f} mm  max {en.max()*1000:6.1f} mm"
                  f"  tilt max {tilt[m].max():5.2f} deg")

    # ---- u_L1 -----------------------------------------------------------
    if len(dbg) and tD is not None and tS is not None:
        d = dbg[(dbg[:, 0] >= tD) & (dbg[:, 0] < tS)]
        if len(d):
            u = d[:, 1+16]
            print(f"  u_L1   thrust mean {u.mean():6.3f} N  max {u.max():6.3f}  min {u.min():6.3f}"
                  f"   |tau| max {np.abs(d[:, 1+17:1+20]).max():.3f} N.m")
            print(f"         motors {d[:, 1+32:1+36].min():.4f}..{d[:, 1+32:1+36].max():.4f}"
                  f"   f_b mean {d[:, 1+12].mean():.3f} N")


for p in sys.argv[1:]:
    score(p)


def oscillation_check(path):
    """Guard against the ~2.4 Hz rate mode a too-stiff attitude pair excites on this node."""
    f = np.load(path, allow_pickle=True)
    log, ev = f["log"], f["events"]
    if len(log) == 0:
        return
    t, tilt = log[:, 0], log[:, 7]
    tD = ev_t(ev, "mode -> DIRECT")
    tS = phases(ev).get("SAFETY_SETTLE")
    if tD is None or tS is None:
        return
    m = (t >= tD + 10) & (t < tS)          # skip the entry transient
    if m.sum() < 200:
        return
    s = tilt[m] - tilt[m].mean()
    dt = np.median(np.diff(t[m]))
    fr = np.fft.rfftfreq(len(s), dt)
    P = np.abs(np.fft.rfft(s)) ** 2
    k = int(np.argmax(P[1:])) + 1
    half = len(s) // 2
    growth = s[half:].std() / max(s[:half].std(), 1e-9)
    print(f"  OSC    tilt dominant {fr[k]:5.2f} Hz  amp(rms) {s.std():.4f} deg"
          f"  2nd-half/1st-half {growth:5.2f}"
          + ("   <-- GROWING" if growth > 1.3 else ""))


for p in sys.argv[1:]:
    oscillation_check(p)
