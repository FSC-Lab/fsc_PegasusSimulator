#!/usr/bin/env python3
"""Closed-loop metrics of the residual estimators from a gate `--log` CSV (nmpc_cascade_steso_node
--test --log FILE): the raw reading floors in cruise, the force twin's reading lag vs the plant's
bob force (offset-tank runs), and the estimator's tracking of the bob.

    ~/envs/isaac_env/bin/python3 scripts/est_loop_metrics.py LOG.csv [LOG2.csv ...] [--t0 15 --t1 40]
"""
import csv
import sys

import numpy as np


def load(path):
    rows = list(csv.reader(open(path))); h = rows[0]
    A = np.array([[float(x) for x in r] for r in rows[1:]])
    return h, A


def lag_corr(t, e, u, kmin=-20, kmax=80):
    e = e - e.mean(); u = u - u.mean(); dt = np.median(np.diff(t)); best = None
    for k in range(kmin, kmax + 1):
        c = np.corrcoef(e[k:], u[:len(u) - k])[0, 1] if k >= 0 else np.corrcoef(e[:k], u[-k:])[0, 1]
        if best is None or c > best[1]:
            best = (k, c)
    return best[0] * dt, best[1]


def metrics(path, t0=15.0, t1=40.0):
    h, A = load(path); t = A[:, h.index("t")]; w = (t > t0) & (t < t1)
    c = lambda name: A[w, h.index(name)]
    out = {"file": path}
    # raw reading floors (HF part: tick differences / sqrt 2, immune to slow residuals)
    hf = lambda x: float(np.std(np.diff(x)) / np.sqrt(2.0))
    out["sigma_sd"] = [float(c(f"sigma_{a}").std()) for a in "xyz"]
    out["sigma_hf"] = [hf(c(f"sigma_{a}")) for a in "xyz"]
    out["Fhat_sd"] = [float(c(f"Fhat_{a}").std()) for a in "xyz"]
    out["Fhat_hf"] = [hf(c(f"Fhat_{a}")) for a in "xyz"]
    out["u_sd"] = [float(c(f"u_{a}").std()) for a in "xyz"]
    if "Ftrue_x" in h:
        tru = c("Ftrue_x"); out["Ftrue_x_rms"] = float(np.sqrt(np.mean(tru ** 2)))
        if out["Ftrue_x_rms"] > 0.02:
            out["raw_lag_x"], out["raw_corr_x"] = lag_corr(t[w], c("Fhat_x"), tru)
            out["pred_lag_x"], out["pred_corr_x"] = lag_corr(t[w], c("Fslosh_x"), tru)
            out["pred_err_x"] = float(np.sqrt(np.mean((c("Fslosh_x") - tru) ** 2)))
        truy = c("Ftrue_y"); out["Ftrue_y_rms"] = float(np.sqrt(np.mean(truy ** 2)))
        if out["Ftrue_y_rms"] > 0.02:
            out["raw_lag_y"], out["raw_corr_y"] = lag_corr(t[w], c("Fhat_y"), truy)
            out["pred_lag_y"], out["pred_corr_y"] = lag_corr(t[w], c("Fslosh_y"), truy)
    return out


if __name__ == "__main__":
    skip = {sys.argv[i + 1] for i, a in enumerate(sys.argv) if a in ("--t0", "--t1") and i + 1 < len(sys.argv)}
    args = [a for a in sys.argv[1:] if not a.startswith("--") and a not in skip]
    t0 = float(sys.argv[sys.argv.index("--t0") + 1]) if "--t0" in sys.argv else 15.0
    t1 = float(sys.argv[sys.argv.index("--t1") + 1]) if "--t1" in sys.argv else 40.0
    for p in args:
        m = metrics(p, t0, t1)
        print(f"== {p}  (t {t0:.0f}-{t1:.0f} s)")
        print(f"   sigma sd [N.m] {['%.4f' % v for v in m['sigma_sd']]}  HF {['%.4f' % v for v in m['sigma_hf']]}   u_ad sd {['%.4f' % v for v in m['u_sd']]}")
        print(f"   F_hat sd [N]   {['%.4f' % v for v in m['Fhat_sd']]}  HF {['%.4f' % v for v in m['Fhat_hf']]}")
        if "raw_lag_x" in m:
            print(f"   bob x rms {m['Ftrue_x_rms']:.3f} N: raw reading lag {1e3 * m['raw_lag_x']:.0f} ms corr {m['raw_corr_x']:.3f}; predictor lag {1e3 * m['pred_lag_x']:.0f} ms corr {m['pred_corr_x']:.3f} err {m['pred_err_x']:.3f} N")
        if "raw_lag_y" in m:
            print(f"   bob y rms {m['Ftrue_y_rms']:.3f} N: raw reading lag {1e3 * m['raw_lag_y']:.0f} ms corr {m['raw_corr_y']:.3f}; predictor lag {1e3 * m['pred_lag_y']:.0f} ms corr {m['pred_corr_y']:.3f}")
