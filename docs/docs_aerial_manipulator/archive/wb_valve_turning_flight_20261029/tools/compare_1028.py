#!/usr/bin/env python3
"""Today's contact phase against the two 2026-10-28 valve flights: the base's state at the grasp (CoM error
along the nose / sideways, q1, q4, claw heading error), the sway's half-cycle envelope in contact and its
growth rate, the mode frequency, and how far into the turn each flight got."""
import os, numpy as np, importlib.util
import vt_common as V
from valve_axis import step_times
HERE = os.path.dirname(os.path.abspath(__file__))
def bag(tag):
    if tag in ("180652", "181833"): V._c.NPZ = os.path.join(HERE, "..", "..", "wb_valve_turning_flight_20261028", "npz")
    else: V._c.NPZ = os.path.join(HERE, "..", "npz")
    return V.Bag(tag)
for tag in ("180652", "181833", "145526"):
    b = bag(tag); st = step_times(b)
    tw = b.t("wb"); D = b.f("wb", "data"); ok = np.isfinite(D[:, 50]) & (D[:, 0] == 1); tw, D = tw[ok], D[ok]
    ta = b.t("vatt"); qa = b.f("vatt", "q")
    psi = np.interp(tw, ta, np.unwrap(np.pi/2 - np.arctan2(2*(qa[:, 0]*qa[:, 3]+qa[:, 1]*qa[:, 2]), 1-2*(qa[:, 2]**2+qa[:, 3]**2))))
    ec = D[:, 48:51] - D[:, 45:48]
    ef = (ec[:, 0]*np.cos(psi) + ec[:, 1]*np.sin(psi))*1e3; es = (-ec[:, 0]*np.sin(psi) + ec[:, 1]*np.cos(psi))*1e3
    q = np.degrees(D[:, 5:9]); hd = np.degrees(np.arcsin(np.clip(D[:, 27], -1, 1)))
    tg = st["push"]; tab = st.get("abort", tw[-1])
    w0 = (tw >= tg) & (tw < tg + 0.5)
    wr = (tw >= st["handle"] - 3.0) & (tw < st["handle"])
    print(f"===== {tag}: grasp {tg:.2f}, abort {tab:.2f} ({tab - tg:.2f} s clamped, {tab - tg - 1.5:.2f} s into the turn)  abort: {st.get('abort_msg', '')[:90]}")
    print(f"   Ready (3 s before the handle): |heading err| max {np.abs(hd[wr]).max():.1f} deg, q4 range {np.ptp(q[wr,3]):.0f} deg, sideways p-p {np.ptp(es[wr]):.0f} mm")
    print(f"   at the grasp (0.5 s): CoM fwd {ef[w0].mean():+.0f} side {es[w0].mean():+.0f} mm | q1 {q[w0,0].mean():+.1f} q4 {q[w0,3].mean():+.1f} | heading err {hd[w0].mean():+.1f} deg")
    wc = (tw >= tg + 0.2) & (tw <= tab)
    for name, x in (("side", es), ("fwd", ef)):
        xx = x[wc] - np.mean(x[wc]); tt = tw[wc]
        s = np.sign(xx); pk = []; i = 0
        while i < len(xx):
            j = i
            while j < len(xx) and s[j] == s[i]: j += 1
            k = i + np.argmax(np.abs(xx[i:j]))
            if abs(xx[k]) > 5 and (j - i) > 25: pk.append((tt[k], abs(xx[k])))
            i = j
        a = np.array(pk)
        rate = np.polyfit(a[:, 0], np.log(a[:, 1]), 1)[0] if len(a) > 2 else np.nan
        X = np.abs(np.fft.rfft(xx))**2; f = np.fft.rfftfreq(len(xx), np.median(np.diff(tt))); m = (f > 0.2) & (f < 1.5)
        print(f"   {name:4s}: peaks " + " ".join(f"{p[1]:.0f}" for p in pk) + f" mm | growth {rate:+.2f} 1/s" + (f" (x2 in {np.log(2)/rate:.1f} s)" if rate > 0 else "") + f" | f {f[m][np.argmax(X[m])]:.2f} Hz")
