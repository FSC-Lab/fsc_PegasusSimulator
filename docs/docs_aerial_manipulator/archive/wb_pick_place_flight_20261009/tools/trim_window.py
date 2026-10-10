"""Would the planner's descent trim help? Replay it on the long hovers of the flights.

For every instant t of a hover: trim_W(t) = the claw's mean horizontal error over the previous W seconds (the planner's
rule, W = 1 s as flown). If the descent started at t, the claw would reach the bottom about tau later with its error
then, e(t + tau), minus the trim that was baked in. Landing error = |e(t + tau) - trim_W(t)|; W = 0 is no trim.
Writes ../analysis/trim_window.json.
"""
import json
import tracking as T
from pp_common import *

HOVERS = [("p1", 51.7, 90.7, "PP-1 behind the stem, no payload"), ("p3", 40.9, 63.3, "PP-3 after the missed pick, no payload"),
          ("p2", 97.2, 118.2, "PP-2 at the place, basket on the claw")]
WS = [0.0, 1.0, 2.0, 3.0, 5.0, 7.0, 10.0]
TAU = 6.0          # press -> bottom of the descent: sideways trim ~1 s + 2 s hold + 3.2 s descent

if __name__ == "__main__":
    out = {}
    for nm, a, b, lab in HOVERS:
        d, t0, S = T.build(nm); t = S["t"]; dt = t[1] - t[0]
        m = (t >= a) & (t <= b); e = S["e_ee"][m][:, :2]; tt = t[m]
        n_tau = int(round(TAU / dt)); res = {}
        print(f"== {lab}: {b - a:.1f} s, claw error rms {1e3 * np.sqrt((e ** 2).sum(1).mean()):.1f} mm, mean {np.round(1e3 * e.mean(0), 1)} mm")
        for W in WS:
            nw = int(round(W / dt)); k0 = max(nw, 1); idx = np.arange(k0, len(tt) - n_tau)
            if len(idx) < 50:
                continue
            if nw == 0:
                trim = np.zeros((len(idx), 2))
            else:
                c = np.cumsum(np.vstack([np.zeros((1, 2)), e]), axis=0); trim = (c[idx + 1] - c[idx + 1 - nw]) / nw
            land = e[idx + n_tau] - trim; r = np.linalg.norm(land, axis=1)
            res[W] = dict(rms=float(1e3 * np.sqrt((r ** 2).mean())), p95=float(1e3 * np.percentile(r, 95)), trim_rms=float(1e3 * np.sqrt((trim ** 2).sum(1).mean())),
                          trim_max=float(1e3 * np.linalg.norm(trim, axis=1).max()), n=int(len(idx)))
            print(f"   window {W:4.1f} s: landing error rms {res[W]['rms']:5.1f} mm, 95 % inside {res[W]['p95']:5.1f} mm | the trim itself: rms {res[W]['trim_rms']:5.1f}, largest {res[W]['trim_max']:5.1f} mm  ({len(idx) * dt:.0f} s of start times)")
        out[nm] = dict(label=lab, t=[a, b], tau=TAU, windows={str(k): v for k, v in res.items()})
    json.dump(out, open(os.path.join(OUT, "trim_window.json"), "w"), indent=1)
