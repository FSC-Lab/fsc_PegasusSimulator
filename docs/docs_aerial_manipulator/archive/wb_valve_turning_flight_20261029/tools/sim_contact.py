#!/usr/bin/env python3
"""The hardware metrics on Isaac runs (pl_mission.py npz): in the contact phase (push:start -> push:end
or the abort), the CoM error in the nose frame (fore-aft / lateral) from the law's own debug
(dbg column = debug index + 2), its dominant frequency and envelope growth, the arm joints' ranges, and
the valve-vs-plan error from obj_0's yaw.  Same definitions as wander.py / sway_growth.py on the bags.
    sim_contact.py <runs dir> tag [tag ...]"""
import sys, os
import numpy as np

def dom_freq(x, dt):
    x = x - x.mean(); f = np.fft.rfftfreq(len(x), dt); P = np.abs(np.fft.rfft(x * np.hanning(len(x))))**2
    k = (f > 0.1) & (f < 3); return f[k][np.argmax(P[k])]

def growth(t, x, thr=8.0):
    s = np.sign(x); pk = []; i = 0
    while i < len(x):
        j = i
        while j < len(x) and s[j] == s[i]: j += 1
        k = i + np.argmax(np.abs(x[i:j])); pk.append((t[k], abs(x[k]))); i = j
    a = np.array([p for p in pk if p[1] > thr])
    return np.polyfit(a[:, 0], np.log(a[:, 1]), 1)[0] if len(a) > 3 else np.nan

def run(d, tag):
    z = np.load(os.path.join(d, tag + ".npz"), allow_pickle=True)
    M = dict(zip([str(n) for n in z["marks_name"]], z["marks_t"]))
    t0 = M.get("push:start"); t1 = M.get("push:end", M.get("abort:start", None))
    if t1 is None:
        ev = [str(e) for e in z["events"] if "ABORT" in str(e)]
        t1 = float(ev[0].split("]")[0].strip("[ s")) if ev else z["dbg"][-1, 0]
    D = z["dbg"]; D = D[np.isfinite(D[:, 52])]
    g = np.arange(t0 + 0.3, t1, 0.02)
    col = lambda i: np.interp(g, D[:, 0], D[:, i + 2])
    ec = np.stack([col(48) - col(45), col(49) - col(46), col(50) - col(47)], 1)
    O = z["odom"]; w, x, y, zz = O[:, 7], O[:, 8], O[:, 9], O[:, 10]
    psi = np.interp(g, O[:, 0], np.unwrap(np.arctan2(2*(w*zz + x*y), 1 - 2*(y*y + zz*zz))))
    nose = np.stack([np.cos(psi), np.sin(psi)], 1); lat = np.stack([-np.sin(psi), np.cos(psi)], 1)
    ef, el = np.sum(ec[:, :2]*nose, 1)*1e3, np.sum(ec[:, :2]*lat, 1)*1e3
    q = np.degrees(np.stack([col(5+i) for i in range(4)], 1))
    eR = np.linalg.norm(np.stack([col(28), col(29), col(30)], 1), axis=1)
    # the valve vs its plan: the claw reference's azimuth about the circle it sweeps (fitted), against
    # obj_0's yaw, both from the push step's start; and its regression on 0.75-style fore-aft carry
    Rf = z["ref"]; mr = (Rf[:, 0] >= t0) & (Rf[:, 0] <= t1)
    X, Y = Rf[mr, 1], Rf[mr, 2]
    A = np.c_[2 * X, 2 * Y, np.ones(mr.sum())]; cx, cy, _ = np.linalg.lstsq(A, X**2 + Y**2, rcond=None)[0]
    az = np.unwrap(np.arctan2(Rf[:, 2] - cy, Rf[:, 1] - cx)); plan = np.degrees(np.interp(g, Rf[:, 0], az) - np.interp(t0, Rf[:, 0], az))
    B = z["box"]; by = np.unwrap(np.arctan2(2*(B[:, 4]*B[:, 7] + B[:, 5]*B[:, 6]), 1 - 2*(B[:, 6]**2 + B[:, 7]**2)))
    valve = np.degrees(np.interp(g, B[:, 0], by) - np.interp(t0, B[:, 0], by))
    err = valve - plan; xr = np.degrees(ef / 1e3 / 0.160)
    slope = np.polyfit(xr, err, 1)[0]; corr = np.corrcoef(xr, err)[0, 1]
    out = dict(vrms=np.sqrt(np.mean(err**2)), vmin=err.min(), vmax=err.max(), slope=slope, corr=corr, final=valve[-1], T=t1 - t0, ab=bool(z["aborted"]), why=str(z["reason"])[:60], ef=ef.std(), el=el.std(), elpp=np.ptp(el),
               ff=dom_freq(ef/1e3, 0.02), fl=dom_freq(el/1e3, 0.02), gr=growth(g, el), q=q, eR=np.sqrt(np.mean(eR**2)))
    return out

if __name__ == "__main__":
    d = sys.argv[1]
    print(f"{'tag':18s} {'contact':>7s} {'abort':>5s}  fore-aft sd  lateral sd (p-p)  f_fa  f_lat  growth  |e_R|  q1 range     q2 range    q3 min  valve-plan rms (range)  slope/corr  end  reason")
    for tag in sys.argv[2:]:
        try:
            r = run(d, tag); q = r["q"]
            print(f"{tag:18s} {r['T']:6.1f}s {str(r['ab']):>5s}  {r['ef']:7.1f} mm  {r['el']:7.1f} ({r['elpp']:4.0f})   {r['ff']:.2f}  {r['fl']:.2f}  {r['gr']:+.2f}   {r['eR']:.3f}  "
                  f"{q[:,0].min():+5.1f}..{q[:,0].max():+5.1f}  {q[:,1].min():+5.1f}..{q[:,1].max():+5.1f}  {q[:,2].min():5.1f}   {r['vrms']:5.1f} ({r['vmin']:+5.1f}..{r['vmax']:+5.1f})  {r['slope']:.2f}/{r['corr']:.2f}  {r['final']:5.1f}  {r['why']}")
        except Exception as e:
            print(f"{tag:18s} failed: {e}")
