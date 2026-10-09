#!/usr/bin/env python3
"""hw_holds.py -- the 1005 whole-body bags' STATIC DIRECT holds (planner status HOLD / READY, >= 8 s, inside
DIRECT): EE / CoM error rms against the planner stream, the speed-independent wander floor.
    AM_NPZ=<dir> PYTHONNOUSERSITE=1 /usr/bin/python3 hw_holds.py
"""
import os, sys, json
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "wb_vs_decoupled_figure8_flight_20261005", "tools"))
import f1005 as F          # noqa: E402
import metrics as M        # noqa: E402
rms = lambda x: float(np.sqrt(np.mean(np.square(x))))
out = {}
for nm in ("w3", "w5"):
    d, t0 = F.load(nm)
    mk = "wbmode" if "wbmode__recv" in d.files else "gmode"
    md = F.edges(d, t0, mk); st = F.edges(d, t0, "pl_status")
    direct = []
    for i, (t, v) in enumerate(md):
        if v == "DIRECT":
            nxt = next((tt for tt, vv in md[i + 1:] if vv != "DIRECT"), float(d[f"{mk}__recv"][-1] - t0))
            direct.append((t, nxt))
    rows = []
    for i, (t, v) in enumerate(st):
        t1 = st[i + 1][0] if i + 1 < len(st) else float(d["pl_status__recv"][-1] - t0)
        if not (v.startswith("HOLD") or v.startswith("READY")): continue
        for a, b in direct:
            lo, hi = max(t, a) + 2.0, min(t1, b) - 0.5          # 2 s after the edge: transients out
            if hi - lo >= 8.0:
                M.windows = (lambda lo=lo, hi=hi, a=a, b=b: (lambda nm_: (a, b, lo, hi)))()
                o, S = M.analyse(nm)
                e = S["re"] - S["red"]; ec = S["xc"] - S["xcd"]
                rows.append(dict(status=v, t=[lo, hi], ee_rms=1e3 * rms(np.linalg.norm(e, axis=1)),
                                 ee_rms_demeaned=1e3 * rms(np.linalg.norm(e - e.mean(0), axis=1)),
                                 com_rms=1e3 * rms(np.linalg.norm(ec, axis=1)), ee_max=1e3 * float(np.linalg.norm(e, axis=1).max())))
                print(f"{nm} {v:28s} {lo:7.1f}-{hi:7.1f} ({hi-lo:5.1f} s)  EE rms {rows[-1]['ee_rms']:5.1f} (demeaned {rows[-1]['ee_rms_demeaned']:5.1f}, max {rows[-1]['ee_max']:5.1f})  CoM {rows[-1]['com_rms']:5.1f} mm")
    out[nm] = rows
json.dump(out, open(os.path.join(HERE, "..", "analysis", "hw_holds.json"), "w"), indent=1)
