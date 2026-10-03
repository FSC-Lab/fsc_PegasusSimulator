"""Data for the single-section report (2026-10-03 restructure): the six circle flights of 09-28 and 10-02 on ONE
scoring window, so every column of the error table uses one definition.

  flights   WB-1/WB-2 (09-28, whole-body 4-D L1), DEC-1/DEC-2 (09-28, decoupled, old gains),
            DEC-3/DEC-4 (10-02, decoupled, tuned gains; runs 1 and 2 of the 13:42 flight)
  window    the first 27.2 s of each circle run (0.1 s trimmed at the start): run 2 of 10-02 was cut 0.8 s before
            the plan's end by the operator's SAFETY command, so all six stop at the same point of the plan
  metrics   metrics.analyse of the 0928 tools (odometry + encoders through the planner's model, vs the plan stream)

    AM_NPZ=<dir with w1 w2 d1 d2 r1 r2 .npz> PYTHONNOUSERSITE=1 /usr/bin/python3 summary_data.py
-> ../analysis/summary_metrics.json, ../analysis/summary_ee3d.json
"""
import json
import c1002 as CC
from c1002 import *
import metrics as M

OUT = "../analysis"
FL = [  # key, tag, date/time, group
    ("w1", "WB-1", "09-28 17:11", "wb"), ("w2", "WB-2", "09-28 17:22", "wb"),
    ("d1", "DEC-1", "09-28 18:28", "old"), ("d2", "DEC-2", "09-28 18:36", "old"),
    ("r1", "DEC-3", "10-02 13:42 run 1", "new"), ("r2", "DEC-4", "10-02 13:42 run 2", "new")]


def window(nm):
    a, b, c0, c1 = CC.windows(nm)
    return a, b, c0, c0 + CC.RUN_T


M.windows = window
r4 = lambda A: np.round(np.asarray(A), 4).tolist()
met, ee3d = {}, {}
for nm, tag, when, grp in FL:
    o, S = M.analyse(nm); m = o["metrics"]
    d, t0 = load(nm); tb = d["batt__recv"] - t0; w = o["window"]
    m["pack_v"] = float(d["batt__voltage_v"][(tb > w[0]) & (tb < w[1])].mean())
    met[nm] = dict(tag=tag, when=when, grp=grp, window=w, direct=o["direct"], n=o["n"], metrics=m)
    k = slice(None, None, 10)
    err = 1e3 * np.linalg.norm(S["re"] - S["red"], axis=1)
    ee3d[nm] = dict(label=f"{tag} · {when}", grp=grp, t=np.round(S["t"][k] - S["t"][0], 2).tolist(),
                    ee=r4(S["re"][k]), ee_ref=r4(S["red"][k]), base=r4(S["Pu"][k]), base_ref=r4(S["xb"][k]),
                    err=np.round(err[k], 1).tolist(), rms=round(m["ee_pos"]["rms_norm"], 1))
    print(f"{tag:6s} {when:18s} window {w[0]:.2f}-{w[1]:.2f}  EE {m['ee_pos']['rms_norm']:6.1f} mm  head {m['ee_head']['rms']:5.2f} deg"
          f"  airframe {m['base_pos']['rms_norm']:6.1f}  V {m['pack_v']:.2f}")
json.dump(met, open(f"{OUT}/summary_metrics.json", "w"), indent=1)
json.dump(ee3d, open(f"{OUT}/summary_ee3d.json", "w"), separators=(",", ":"))
