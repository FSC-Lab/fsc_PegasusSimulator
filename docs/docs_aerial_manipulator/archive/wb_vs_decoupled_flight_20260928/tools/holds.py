"""Hover before the circle (the DIRECT-entry hold, planner HOLD, >= 5 s): CoM horizontal error, standing offset
(|mean|) and wobble (std per axis, rms of x and y). Whole-body flights only (law debug x_c vs x_cd).
Writes ../analysis/holds.json."""
import json
from common import *
res = {}
for nm in ("a1", "a2", "a3", "a4", "w1", "w2"):
    d, t0 = load(nm); a, b, c0, c1 = windows(nm); st = edges(d, t0, "pl_status")
    h = next((t, st[i + 1][0]) for i, (t, v) in enumerate(st) if v.startswith("HOLD") and i + 1 < len(st) and st[i + 1][0] - t > 5)
    tw = d["wb__recv"] - t0; D = d["wb__data"]; m = (tw > h[0] + 1.0) & (tw < h[1])
    e = 1e3 * (D[m, 48:50] - D[m, 45:47])
    res[nm] = dict(window=[h[0], h[1]], offset_mm=float(np.linalg.norm(e.mean(0))), wobble_mm=float(np.sqrt((e.std(0) ** 2).mean())))
    print(FLIGHTS[nm], res[nm])
json.dump(res, open("../analysis/holds.json", "w"), indent=1)
