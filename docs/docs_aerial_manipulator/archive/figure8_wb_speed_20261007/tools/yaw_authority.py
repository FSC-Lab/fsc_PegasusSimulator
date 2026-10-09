#!/usr/bin/env python3
"""yaw_authority.py -- the law's commanded yaw torque on a figure-8 run, split into
  tau_z(t) = bias + k_acc * psi_ref_ddot(t) + k_rate * psi_ref_dot(t) + residual
by least squares on a 100 Hz grid over the run window (tau_z low-passed at 2 Hz for the fit; the residual's
>2 Hz part is reported as noise). k_acc is the effective yaw inertia the law has to drive: equal k_acc on
hardware and in the mirror sim means the sim's yaw authority is right; a larger hardware k_acc means a
weaker real yaw actuator than modelled. Then predicts the p99 / max |tau_z| at other speeds by replaying
the PLAN's yaw reference of a sim run at that speed through the hardware fit.

  hardware: AM_NPZ=<dir> PYTHONNOUSERSITE=1 /usr/bin/python3 yaw_authority.py hw w3 w4 w5 w6 --json out.json
  sim:      /usr/bin/python3 yaw_authority.py sim <run.npz> ... [--hwfit out.json]
"""
import json, os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))


def lp(x, dt, fc):
    """zero-phase first-order low-pass, forward-backward"""
    a = np.exp(-2 * np.pi * fc * dt); y = np.array(x, float)
    for rng in (range(1, len(y)), range(len(y) - 2, -1, -1)):
        for i in rng:
            y[i] = a * y[i + (-1 if rng.step == 1 else 1)] + (1 - a) * y[i]
    return y


def cdiff(x, dt):
    return np.gradient(x, dt)


def fit(t, yaw_ref, tauz, dt=0.01):
    psi = lp(np.unwrap(yaw_ref), dt, 3.0); pd = cdiff(psi, dt); pdd = lp(cdiff(pd, dt), dt, 3.0)
    tl = lp(tauz, dt, 2.0)
    X = np.column_stack([np.ones_like(t), pdd, pd])
    c, *_ = np.linalg.lstsq(X, tl, rcond=None)
    res_lf = tl - X @ c; hf = tauz - tl
    r2 = 1 - np.var(res_lf) / np.var(tl)
    return dict(bias=float(c[0]), k_acc=float(c[1]), k_rate=float(c[2]), r2=float(r2),
                lf_resid_rms=float(np.sqrt(np.mean(res_lf ** 2))), hf_rms=float(np.sqrt(np.mean(hf ** 2))),
                p99=float(np.percentile(np.abs(tauz), 99.5)), max=float(np.abs(tauz).max()),
                pdd_p99=float(np.percentile(np.abs(pdd), 99.5)), pd_p99=float(np.degrees(np.percentile(np.abs(pd), 99.5)))), (pd, pdd, tauz - X @ c)


def hw(names):
    sys.path.insert(0, os.path.join(HERE, "..", "..", "wb_vs_decoupled_figure8_flight_20261005", "tools"))
    import f1005 as F, metrics as M
    out = {}
    for nm in names:
        o, S = M.analyse(nm); d, t0 = F.load(nm)
        tw = d["wb__recv"] - t0; D = d["wb__data"]
        tz = np.interp(S["t"], tw, D[:, 23])
        r, (pd, pdd, resid) = fit(S["t"], S["yaw_ref"], tz)
        out[nm] = r; out[nm]["resid"] = resid.tolist()
    return out


def sim_load(p):
    sys.path.insert(0, os.path.join(HERE, "..", "..", "..", "..", "..", "application", "robotic_arm", "utils"))
    import am_ee_compare_score as SC
    d = np.load(p, allow_pickle=True)
    marks = dict(str(x).split("=") for x in d["marks"]); t0, t1 = float(marks["run_start"]), float(marks["run_end"])
    raw = d["dbg"]
    n = max(len(x) for x in raw); D = np.array([np.asarray(x, float) for x in raw if len(x) == n]) if raw.dtype == object else np.asarray(raw, float)
    w = d["wbref"]; tg = np.arange(t0 + 0.1, t1 - 0.1, 0.01)
    yaw = np.interp(tg, w[:, 0], np.unwrap(np.arctan2(w[:, 11], w[:, 10])))
    tz = np.interp(tg, D[:, 0], D[:, 1 + 23])
    return tg, yaw, tz


def main():
    mode = sys.argv[1]; args = sys.argv[2:]
    js = args[args.index("--json") + 1] if "--json" in args else None
    hwf = args[args.index("--hwfit") + 1] if "--hwfit" in args else None
    args = [a for a in args if a not in ("--json", "--hwfit", js, hwf)]
    if mode == "hw":
        out = hw(args)
        for k, r in out.items():
            print(f"{k}: tau_z = {r['bias']:+.3f} + {r['k_acc']:.4f}*psi_dd + {r['k_rate']:+.4f}*psi_d  (R2 {r['r2']:.2f}, LF resid {r['lf_resid_rms']:.3f}, HF {r['hf_rms']:.3f})  "
                  f"|tau_z| p99 {r['p99']:.3f} max {r['max']:.3f}  ref psi_dd p99 {r['pdd_p99']:.2f} rad/s2, psi_d p99 {r['pd_p99']:.1f} deg/s")
        if js: json.dump(out, open(js, "w"))
        return
    H = json.load(open(hwf)) if hwf else None
    for p in args:
        tg, yaw, tz = sim_load(p)
        r, (pd, pdd, resid) = fit(tg, yaw, tz)
        line = (f"{os.path.basename(p):20s} tau_z = {r['bias']:+.3f} + {r['k_acc']:.4f}*psi_dd + {r['k_rate']:+.4f}*psi_d (R2 {r['r2']:.2f}, HF {r['hf_rms']:.3f})"
                f"  |tau_z| p99 {r['p99']:.3f} max {r['max']:.3f}  psi_dd p99 {r['pdd_p99']:.2f}, psi_d p99 {r['pd_p99']:.1f} deg/s")
        if H:
            # hardware prediction on THIS run's reference: each hardware fit + that run's own residual (resampled)
            preds = []
            for k, h in H.items():
                res = np.asarray(h["resid"]); rr = np.resize(res, len(tg))
                z = h["bias"] + h["k_acc"] * pdd + h["k_rate"] * pd + rr
                preds.append((np.percentile(np.abs(z), 99.5), np.abs(z).max()))
            pr = np.array(preds)
            line += f"  | HW-predicted |tau_z| p99 {pr[:,0].min():.3f}..{pr[:,0].max():.3f}, max {pr[:,1].min():.3f}..{pr[:,1].max():.3f}"
        print(line)


if __name__ == "__main__":
    main()
