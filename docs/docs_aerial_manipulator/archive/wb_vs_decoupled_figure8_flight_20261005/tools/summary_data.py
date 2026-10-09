"""Data for section 2 of the free-flight comparison report (the 1005 figure-8 runs), on the definitions of section 1
(../../decoupled_flight_20261002/tools/summary_data.py): metrics.analyse of the 0928 tools -- odometry + encoders
through the planner's model, against the planner's stream -- over each figure-8 run's whole EXECUTING span (0.1 s
trimmed at each end). The circle's radius row becomes the figure-8's extent along world x and y (plan 1.40 x 0.70 m).

    AM_NPZ=<dir with w3 w4 w5 w6 r5 r6 .npz> PYTHONNOUSERSITE=1 /usr/bin/python3 summary_data.py
-> ../analysis/fig8_metrics.json, ../analysis/fig8_ee3d.json
"""
import json, os
import f1005 as F
from f1005 import *
import metrics as M

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "analysis")
r4 = lambda A: np.round(np.asarray(A), 4).tolist()
met, ee3d = {}, {}
for nm, (tag, when, grp, v, bag, idx) in F.RUNS.items():
    o, S = M.analyse(nm); m = o["metrics"]
    d, t0 = load(nm); tb = d["batt__recv"] - t0; w = o["window"]
    m["pack_v"] = float(d["batt__voltage_v"][(tb > w[0]) & (tb < w[1])].mean())
    ext = lambda P: (P[:, :2].max(0) - P[:, :2].min(0)).tolist()
    m["ee_extent_m"] = ext(S["re"]); m["ee_extent_ref_m"] = ext(S["red"])
    m["run_s"] = float(w[1] - w[0])
    met[nm] = dict(tag=tag, when=when, grp=grp, speed=v, bag=bag, run_index=idx, window=w, direct=o["direct"], n=o["n"], metrics=m)
    k = slice(None, None, 10)
    err = 1e3 * np.linalg.norm(S["re"] - S["red"], axis=1)
    ee3d[nm] = dict(label=f"{tag} · {when} · {v:.2f} m/s", grp=grp, t=np.round(S["t"][k] - S["t"][0], 2).tolist(),
                    ee=r4(S["re"][k]), ee_ref=r4(S["red"][k]), base=r4(S["Pu"][k]), base_ref=r4(S["xb"][k]),
                    err=np.round(err[k], 1).tolist(), rms=round(m["ee_pos"]["rms_norm"], 1))
    print(f"{tag:6s} {when} {v:.2f} m/s  window {w[0]:.2f}-{w[1]:.2f}  EE {m['ee_pos']['rms_norm']:6.1f} mm (max {m['ee_pos']['max_norm']:.0f})"
          f"  head {m['ee_head']['rms']:5.2f} deg  airframe {m['base_pos']['rms_norm']:6.1f}  CoM {m['com_pos']['rms_norm']:6.1f}"
          f"  tilt {m['tilt_max_deg']:.2f}  sat {m['sat']}  ext {np.round(m['ee_extent_m'],3)} / ref {np.round(m['ee_extent_ref_m'],3)}  V {m['pack_v']:.2f}")
json.dump(met, open(f"{OUT}/fig8_metrics.json", "w"), indent=1)
json.dump(ee3d, open(f"{OUT}/fig8_ee3d.json", "w"), separators=(",", ":"))
