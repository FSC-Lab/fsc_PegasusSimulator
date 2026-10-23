#!/usr/bin/env python3
"""bench_series.py -- time series of both laws on the circle bench for the report
(EE absolute error and CoM error vs plan time), hardware clock (RTF 1) and the
Isaac clock emulation (RTF 0.48). Writes ../analysis/bench_series.json."""
import json, multiprocessing as mp, os
import numpy as np
import modular_bench as MB, mapped as MP, tune_modular as TM

HERE = os.path.dirname(os.path.abspath(__file__))
P = dict(MP.TABLE2_MAPPED); P.update(json.load(open(os.path.join(HERE, "..", "analysis", "modular_final_gains.json")))["best"])


def job(c):
    who, rtf = c
    kw = dict(keep=True) if rtf == 1.0 else dict(keep=True, rtf=rtf, t_settle=20.0)
    r = MB.run_wb(None, TM.STREAM, **kw) if who == "WB" else MB.run(MP.make(P), TM.STREAM, **kw)
    H = r["H"]; t0, t1 = MB.CB.stream(TM.STREAM)["run"]
    m = (H["t"] >= t0) & (H["t"] <= t1)
    idx = np.where(m)[0][::max(1, m.sum() // 400)]
    out = {"t": np.round(H["t"][idx] - t0, 3).tolist(), "ee": np.round(H["ee"][idx] * 1e3, 2).tolist(),
           "com": np.round(H["ex"][idx] * 1e3, 2).tolist(), "head": np.round(H["hd"][idx], 3).tolist(),
           "q2e": np.round(np.degrees(r["Hq"][idx, 1] - r["Hqd"][idx, 1]), 2).tolist(),
           "q3e": np.round(np.degrees(r["Hq"][idx, 2] - r["Hqd"][idx, 2]), 2).tolist()}
    summ = {k: v for k, v in r.items() if k not in ("H", "Hq", "Hqd")}
    return c, out, summ


with mp.get_context("fork").Pool(4) as pool:
    res = {f"{w}@{rtf}": {"series": s, "summary": sm} for (w, rtf), s, sm in
           pool.imap(job, [("WB", 1.0), ("MOD", 1.0), ("WB", 0.48), ("MOD", 0.48)])}
json.dump(res, open(os.path.join(HERE, "..", "analysis", "bench_series.json"), "w"), default=float)
for k, v in res.items():
    print(k, "EE rms %.1f mm, CoM rms %.1f mm, heading rms %.2f deg" % (v["summary"]["ee_rms"] * 1e3, v["summary"]["com_rms"] * 1e3, v["summary"]["head_rms"]))
