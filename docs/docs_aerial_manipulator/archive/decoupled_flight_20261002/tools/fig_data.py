"""Data for the 1002 section's three interactive figures -> ../analysis/{ee3d_dec,err_ts_dec,teleop_ts}.json

ee3d_dec    the four decoupled circle runs in 3-D (0928 DEC-1/DEC-2 on the old gains, 1002 run 1/run 2 on the new):
            EE planned / measured, airframe planned / measured, 10 Hz (the 0928 fig_data.py definition)
err_ts_dec  EE and airframe error in the circle frame + EE heading error, 25 Hz bin means, time since the run start
teleop_ts   PS4 flight: EE and airframe reference vs measured (x, y, z), EE error norm, 25 Hz bin means
"""
import json
from c1002 import *
import metrics as M

OUT = "../analysis"
r4 = lambda A: np.round(np.asarray(A), 4).tolist()
LAB = {"d1": "DEC-1 09-28", "d2": "DEC-2 09-28", "r1": "Run 1 10-02", "r2": "Run 2 10-02"}
GAIN = {"d1": "old", "d2": "old", "r1": "new", "r2": "new"}


def binmean(t, v, w=0.04, dec=2):
    n = int(np.ceil(t[-1] / w)); k = np.minimum((t / w).astype(int), n - 1); c = np.maximum(np.bincount(k, minlength=n), 1)
    return np.round(np.bincount(k, weights=v, minlength=n) / c, dec).tolist()


ee3d, ts = {}, {}
for nm in ("d1", "d2", "r1", "r2"):
    o, S = M.analyse(nm); k = slice(None, None, 10)
    err = 1e3 * np.linalg.norm(S["re"] - S["red"], axis=1)
    ee3d[nm] = dict(label=LAB[nm], gain=GAIN[nm], t=np.round(S["t"][k] - S["t"][0], 2).tolist(), ee=r4(S["re"][k]), ee_ref=r4(S["red"][k]),
                    base=r4(S["Pu"][k]), base_ref=r4(S["xb"][k]), err=np.round(err[k], 1).tolist(), rms=round(o["metrics"]["ee_pos"]["rms_norm"], 1))
    t = S["t"] - S["t"][0]; rf = S["red"]; rad = rf[:, :2] / np.linalg.norm(rf[:, :2], axis=1)[:, None]; tan = np.column_stack([-rad[:, 1], rad[:, 0]])
    e = S["re"] - rf; rb = S["xb"]; radb = rb[:, :2] / np.linalg.norm(rb[:, :2], axis=1)[:, None]; tanb = np.column_stack([-radb[:, 1], radb[:, 0]]); eb = S["Pu"] - rb
    ch = dict(rad=1e3 * np.sum(e[:, :2] * rad, 1), tan=1e3 * np.sum(e[:, :2] * tan, 1), ver=1e3 * e[:, 2], head=S["e"]["ee_head"],
              brad=1e3 * np.sum(eb[:, :2] * radb, 1), btan=1e3 * np.sum(eb[:, :2] * tanb, 1), bver=1e3 * eb[:, 2])
    ts[nm] = dict(label=LAB[nm], gain=GAIN[nm], t=binmean(t, t), **{c: binmean(t, v) for c, v in ch.items()})
    print(nm, "window", np.round(o["window"], 2), "EE rms", round(o["metrics"]["ee_pos"]["rms_norm"], 1))
json.dump(ee3d, open(f"{OUT}/ee3d_dec.json", "w"), separators=(",", ":"))
json.dump(ts, open(f"{OUT}/err_ts_dec.json", "w"), separators=(",", ":"))

o, S = M.analyse("tp"); t = S["t"] - S["t"][0]
tp = dict(t=binmean(t, t), window=o["window"])
for i, ax in enumerate("xyz"):
    tp[f"ee_{ax}"] = binmean(t, S["re"][:, i], dec=4); tp[f"ee_ref_{ax}"] = binmean(t, S["red"][:, i], dec=4)
    tp[f"b_{ax}"] = binmean(t, S["Pu"][:, i], dec=4); tp[f"b_ref_{ax}"] = binmean(t, S["xb"][:, i], dec=4)
tp["ee_err"] = binmean(t, 1e3 * np.linalg.norm(S["re"] - S["red"], axis=1))
tp["b_err"] = binmean(t, 1e3 * np.linalg.norm(S["Pu"] - S["xb"], axis=1))
json.dump(tp, open(f"{OUT}/teleop_ts.json", "w"), separators=(",", ":"))
import os
print({f: os.path.getsize(f"{OUT}/{f}") for f in ("ee3d_dec.json", "err_ts_dec.json", "teleop_ts.json")})
