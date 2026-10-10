"""Trim off vs trim on at the place: replay both on the loaded hover of PP-2 (and the unloaded hovers for reference).

off   the planner's gate is then a real distance check (|claw error| <= 50 mm at the press), nothing is shifted, and the
      claw is at the bottom T_OFF after the press (the 0.20 m descent alone)
on    the claw must be within 50 mm of its own W-second mean, the descent is shifted by that mean, flown sideways,
      held 2 s, then down: at the bottom T_ON after the press (8.3-8.4 s in both flown places)
The operator can also wait for a smaller reading before pressing: press only when |error| <= limit.
Landing error = |e(t + T) - shift|, horizontal. Writes ../analysis/trim_choice.json.
"""
import json
import tracking as T
from pp_common import *

HOVERS = [("p2", 97.2, 118.2, "PP-2 at the place, basket on the claw"), ("p1", 51.7, 90.7, "PP-1 behind the stem, no payload"),
          ("p3", 40.9, 63.3, "PP-3 after the missed pick, no payload")]
T_OFF, T_ON = 3.2, 8.4

if __name__ == "__main__":
    out = {}
    for nm, a, b, lab in HOVERS:
        d, t0, S = T.build(nm); t = S["t"]; dt = t[1] - t[0]
        m = (t >= a) & (t <= b); e = S["e_ee"][m][:, :2]; n = len(e); en = np.linalg.norm(e, axis=1)
        c = np.cumsum(np.vstack([np.zeros((1, 2)), e]), axis=0)
        def st(r, sel, total):
            return dict(rms=float(1e3 * np.sqrt((r[sel] ** 2).mean())), p95=float(1e3 * np.percentile(r[sel], 95)), max=float(1e3 * r[sel].max()),
                        open=float(sel.sum() / total), seconds=float(sel.sum() * dt)) if sel.sum() > 25 else None
        res = {}
        print(f"== {lab} ({b - a:.0f} s)")
        k = int(round(T_OFF / dt)); idx = np.arange(0, n - k); land = np.linalg.norm(e[idx + k], axis=1)
        for lim in (1e9, 0.050, 0.030, 0.020):
            sel = en[idx] <= lim; r = st(land, sel, len(idx)); key = "off_any" if lim > 1 else f"off_{int(lim * 1e3)}"
            res[key] = r
            if r: print(f"   trim OFF, press when |error| <= {'any' if lim > 1 else int(lim * 1e3):>3} mm: landing rms {r['rms']:5.1f}, 95 % inside {r['p95']:5.1f}, worst {r['max']:5.1f} mm | the gate is open {100 * r['open']:3.0f} % of the time")
        k = int(round(T_ON / dt))
        for W in (1.0, 5.0, 10.0):
            nw = int(round(W / dt)); idx = np.arange(nw, n - k)
            if len(idx) < 50:
                print(f"   trim ON, {W:.0f} s window: only {max(len(idx), 0) * dt:.1f} s of data left -- cannot be judged"); res[f"on_{int(W)}"] = None; continue
            trim = (c[idx + 1] - c[idx + 1 - nw]) / nw
            ok = (np.linalg.norm(e[idx] - trim, axis=1) <= 0.050) & (np.linalg.norm(trim, axis=1) <= 0.15)
            land = np.linalg.norm(e[idx + k] - trim, axis=1); r = st(land, ok, len(idx)); res[f"on_{int(W)}"] = r
            if r: print(f"   trim ON, {W:4.1f} s window                : landing rms {r['rms']:5.1f}, 95 % inside {r['p95']:5.1f}, worst {r['max']:5.1f} mm | gate open {100 * r['open']:3.0f} %, shift rms {1e3 * np.sqrt((trim ** 2).sum(1).mean()):.0f} mm ({len(idx) * dt:.0f} s of start times)")
        out[nm] = dict(label=lab, res=res)
    json.dump(out, open(os.path.join(OUT, "trim_choice.json"), "w"), indent=1)
