#!/usr/bin/env python3
"""A q3 RATE guard for the valve, replayed on every recorded Isaac valve run (the box push's 2026-10-24
q2-wall-aware rule): on every joint sample p = q3 + tau q3_dot + max(0, q2 + tau q2_dot - Q2MAX) -- the
fold q2 + q3 is what the arm holds, so a q3 drop that q2's wall will stop gets that credit -- trip when
p < lim and q3_dot < -min_rate for hold s. The valve flies the STATIC rule only today (q3 < 3 deg for
0.2 s); 10-22 found the box's plain rate rule false-trips 41 of 57 completed valve turns.
Engaged window: ready:end -> exit:start (or the first abort / DIRECT loss).
    valve_q3_replay.py <runs dir> [...]   -> per-campaign table + a grid over (lim, tau, min_rate)."""
import os, re, sys, glob
import numpy as np
Q2MAX = 45.0; VEL_TC = 0.02; HOLD = 0.05; STATIC_LIM, STATIC_PER, TICK = 3.0, 0.2, 0.1

def ev_t(z, pats, after=0.0):
    for e in z["events"]:
        s = str(e); m = re.match(r"\[\s*([0-9.]+)s\]", s)
        if m and float(m.group(1)) > after and any(p in s for p in pats): return float(m.group(1))
    return None
def lpf(t, x, tc):
    y = x.copy()
    for i in range(1, len(x)):
        y[i] = y[i-1] + (1 - np.exp(-(t[i]-t[i-1]) / tc)) * (x[i] - y[i-1])
    return y
def load(path):
    z = np.load(path, allow_pickle=True); mk = {str(n): float(x) for n, x in zip(z["marks_name"], z["marks_t"])}
    if "ready:end" not in mk or "joints" not in z.files or len(z["joints"]) < 100: return None
    J = z["joints"]; t = J[:, 0]; q2, q3 = np.degrees(J[:, 2]), np.degrees(J[:, 3])
    t0 = mk["ready:end"]; t_ab = ev_t(z, ["ABORTING (SAFETY GUARD"], t0); t_nd = ev_t(z, ["push_pull: NOT IN DIRECT"], t0)
    t1 = min(x for x in (mk.get("exit:start"), t_ab, t_nd, t[-1]) if x is not None)
    w = (t >= t0) & (t <= t1 + 1.5)
    buckle = bool((q3[(t >= t0) & (t <= t1 + 1.5)] < 0).any())
    reason = str(z["reason"])[:60] if bool(z["aborted"]) else ""
    ok = not bool(z["aborted"]) and "exit:start" in mk
    return dict(tag=os.path.basename(path)[:-4], t=t[w], q2=q2[w], q3=q3[w], t0=t0, t1=t1, ok=ok, buckle=buckle, reason=reason,
                q2d=lpf(t[w], np.gradient(q2[w], t[w]), VEL_TC), q3d=lpf(t[w], np.gradient(q3[w], t[w]), VEL_TC))
def static_trip(r):
    since = None
    for tk in np.arange(r["t0"], r["t"][-1], TICK):
        q = np.interp(tk, r["t"], r["q3"])
        if q < STATIC_LIM:
            since = tk if since is None else since
            if tk - since >= STATIC_PER - 1e-9: return tk
        else: since = None
    return None
def rate_trip(r, lim, tau, minr, t_end):
    p = r["q3"] + tau * r["q3d"] + np.maximum(0.0, r["q2"] + tau * r["q2d"] - Q2MAX)
    bad = (p < lim) & (r["q3d"] < -minr) & (r["t"] >= r["t0"]) & (r["t"] <= t_end)
    since = None
    for i in np.where(bad | (np.r_[False, bad[:-1]]))[0]:
        if bad[i]:
            since = r["t"][i] if since is None else since
            if r["t"][i] - since >= HOLD - 1e-9: return r["t"][i]
        else: since = None
    return None
def main():
    dirs = sys.argv[1:]
    runs = [x for d in dirs for x in (load(p) for p in sorted(glob.glob(os.path.join(d, "*.npz")))) if x is not None]
    good = [r for r in runs if r["ok"]]; bk = [r for r in runs if r["buckle"]]
    print(f"{len(runs)} runs with a grasp: {len(good)} completed turns, {len(bk)} with q3 < 0 (a buckle into the elbow branch)")
    print(f"completed turns: q3 min over the engaged window p5/median {np.percentile([r['q3'][(r['t']<=r['t1'])].min() for r in good],5):.1f} / "
          f"{np.median([r['q3'][(r['t']<=r['t1'])].min() for r in good]):.1f} deg")
    print("\nthe static rule (today): catches each buckle at")
    for r in bk:
        ts = static_trip(r); tz = r["t"][np.argmax(r["q3"] < 0)]
        print(f"  {r['tag']:24s} q3 crossed 0 at +{tz - r['t0']:5.1f} s; static trip {'%+.2f s vs the crossing (q3 %.1f)' % (ts - tz, np.interp(ts, r['t'], r['q3'])) if ts else 'never'}  [{r['reason']}]")
    print("\ngrid: false trips on completed turns | buckles caught BEFORE q3 crossed 0 (lead, q3 at the trip)")
    for lim in (3.0, 6.0, 10.0):
        for tau in (0.1, 0.2):
            for minr in (10.0, 30.0, 60.0):
                ft = sum(rate_trip(r, lim, tau, minr, r["t1"]) is not None for r in good)
                leads = []
                for r in bk:
                    tz = r["t"][np.argmax(r["q3"] < 0)]; tr = rate_trip(r, lim, tau, minr, tz)
                    if tr is not None: leads.append((tz - tr, np.interp(tr, r["t"], r["q3"])))
                print(f"  lim {lim:4.1f} tau {tau:.1f} min_rate {minr:4.0f}: false {ft:2d}/{len(good)} | caught {len(leads)}/{len(bk)}"
                      + (f", lead median {np.median([l[0] for l in leads]):.2f} s (min {min(l[0] for l in leads):.2f}), q3 at trip median {np.median([l[1] for l in leads]):.1f}" if leads else ""))

if __name__ == "__main__":
    main()
