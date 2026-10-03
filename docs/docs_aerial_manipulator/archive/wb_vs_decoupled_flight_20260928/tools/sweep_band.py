"""How much of the airframe error sits at the arm-sweep frequency. Airframe error in the circle frame (radial,
along-track), circle run minus 3 s at each end, Hann periodogram; share of the variance in 0.13-0.21 Hz (the 6 s
sweep is 0.166 Hz; the 12 s sweep of F3 sits at 0.083 Hz, so F3 is scored in 0.06-0.11 Hz). Writes ../analysis/sweep_band.json."""
import json
from common import *
from scipy.signal import periodogram
import metrics as M
res = {}
for nm in ("a3", "a4", "w1", "w2", "d1", "d2"):
    o, S = M.analyse(nm); t = S["t"] - S["t"][0]; m = (t > 3) & (t < t[-1] - 3)
    rb = S["xb"]; rad = rb[:, :2] / np.linalg.norm(rb[:, :2], axis=1)[:, None]; tan = np.column_stack([-rad[:, 1], rad[:, 0]])
    e = S["Pu"] - rb; ch = {"radial": np.sum(e[:, :2] * rad, 1), "along": np.sum(e[:, :2] * tan, 1)}
    lo, hi = (0.06, 0.11) if nm == "a3" else (0.13, 0.21)
    r = {}
    for k, x in ch.items():
        x = x[m] - x[m].mean(); f, P = periodogram(x, fs=100, window="hann")
        r[k] = dict(share_pct=float(100 * P[(f >= lo) & (f < hi)].sum() / P[f > 0].sum()), pp_mm=float(1e3 * np.ptp(x)), mean_mm=float(1e3 * ch[k][m].mean()))
    res[nm] = r
    print(f"{FLIGHTS[nm]:14s} sweep band {lo}-{hi} Hz: along-track {r['along']['share_pct']:.0f} % of variance ({r['along']['pp_mm']:.0f} mm p-p, mean {r['along']['mean_mm']:+.0f}) | radial {r['radial']['share_pct']:.0f} % ({r['radial']['pp_mm']:.0f} mm p-p, mean {r['radial']['mean_mm']:+.0f})")
json.dump(res, open("../analysis/sweep_band.json", "w"), indent=1)
