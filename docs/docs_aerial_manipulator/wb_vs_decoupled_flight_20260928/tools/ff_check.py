"""Was the new arm compensation live on the drone? From the arm controller's own law_debug and the law's debug array,
circle run, whole-body flights.
  FF p99       99th percentile of the 0.1 s-averaged friction feed-forward (law_debug[18..21] / kappa_pwm) [N.m]
  follows      when the measured and reference joint velocities have opposite signs (both > 2 deg/s), the share of
               samples where the feed-forward has the sign of the MEASURED velocity
  when stuck   mean |FF| while the joint is stuck (|q'| < 0.3 deg/s) and the reference moves (> 5 deg/s)
  observer     share of law ticks that closed on the arm's velocity observer (wb_control_debug[106], new builds only)
Writes ../analysis/ff_check.json."""
import json
from common import *
def lp(x, k=25): return np.convolve(x, np.ones(k) / k, "same")
res = {}
for nm in ("a3", "a4", "w1", "w2"):
    d, t0 = load(nm); a, b, c0, c1 = windows(nm)
    tl = d["law__recv"] - t0; L = d["law__data"]; m = (tl > c0) & (tl < c1); tl = tl[m]; aux = L[m, 18:22] / KPWM
    tw = d["wb__recv"] - t0; D = d["wb__data"]
    tu = np.arange(c0, c1, 0.004); Qm = interp(tu, tw, D[:, 5:9]); QD = interp(tu, tw, D[:, 9:13])
    V = np.column_stack([lp(np.gradient(Qm[:, k], 0.004)) for k in range(4)]); VD = np.gradient(QD, 0.004, axis=0)
    v = interp(tl, tu, V); vd = interp(tl, tu, VD); r = {}
    for j in (1, 2):
        f = lp(aux[:, j]); dis = (np.sign(v[:, j]) != np.sign(vd[:, j])) & (np.abs(v[:, j]) > np.radians(2)) & (np.abs(vd[:, j]) > np.radians(2))
        stuck = (np.abs(v[:, j]) < np.radians(0.3)) & (np.abs(vd[:, j]) > np.radians(5))
        r[f"j{j+1}"] = dict(ff_p99=float(np.percentile(np.abs(f), 99)), follows_measured_pct=float(100 * (np.sign(f) == np.sign(v[:, j]))[dis].mean()),
                            n_disagree=int(dis.sum()), ff_when_stuck=float(np.abs(f[stuck]).mean()), n_stuck=int(stuck.sum()))
    mw = (tw > c0) & (tw < c1)
    r["observer_pct"] = float(100 * np.nanmean(D[mw, 106])) if D.shape[1] > 106 else None
    res[nm] = r; print(FLIGHTS[nm], r)
json.dump(res, open("../analysis/ff_check.json", "w"), indent=1)
