#!/usr/bin/env python3
"""Full-state circle-tracking RMSE, before (shipped) vs after (H1b) the tune.

    /usr/bin/python3 circle_states.py   -> analysis/circle_states.json

Two sources, one processing:
  isaac  the matched Isaac flights of 2026-09-27 (mirror plant, flown and
         showcase circle; robustness plant, showcase circle), extracted to
         data/isaac_*.npz. Window = the planner's longest EXECUTING leg, DIRECT
         ticks, plant time (the sim's wall clock x its measured RTF).
  bench  circle_bench.simulate(full=True): the exact law at RTF 1 (the clock
         hardware has) on the same streams, calibrated feedback noise, seeds
         11/12/13 pooled.

Per state, the RMSE of the tracking error over the circle:
  CoM position   x_c - x_cd                         [mm]
  CoM velocity   0.2 s local slope of x_c - x_cd    [mm/s]
  attitude       e_R, body roll/pitch/yaw            [deg]
  body rate      e_w = w - R^T R0c w_0c              [deg/s]  (bench only: the node
                 does not log the desired rate)
  EE position    (x_c - x_cd) + e_y[0:3] = r_e - r_ed  [mm]
  EE heading     asin(e_y[3])                        [deg]
  joints         q - q_d                             [deg]
  joint rates    0.2 s local slope of q - q_d        [deg/s]
e_R / e_w come out of the law in the MODEL body frame (model +x = actual -y,
model +y = actual +x): roll = e[1], pitch = -e[0], yaw = e[2].
"""
import json
import multiprocessing as mp
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
S2R = os.path.join(HERE, "..", "..", "sim2real_tuning_20260926", "tools")
sys.path.insert(0, S2R)
sys.path.insert(0, HERE)
import circle_bench as CB  # noqa: E402
import compare as CP  # noqa: E402
from state_rmse import win_slope  # noqa: E402

AN = os.path.join(HERE, "..", "analysis")
DATA = os.path.join(HERE, "..", "data")
SHOW, FLOWN = "r075_L24_sync15_cw", "r050_L24_half10"
CASES = [("flown", "mirror", FLOWN), ("show", "mirror", SHOW), ("show_rob", "robustness", SHOW)]
ISAAC = {"ship": "isaac_ship_{c}_1252.npz", "h1b": "isaac_h1b_{c}_1519.npz"}
SEEDS = (11, 12, 13)


def body_rpy(e):
    """MODEL-frame body vector -> actual roll / pitch / yaw components."""
    return np.column_stack([e[:, 1], -e[:, 0], e[:, 2]])


def rms(v):
    return float(np.sqrt(np.nanmean(np.asarray(v, float) ** 2)))


def score(t, ep, eR, ew, ee, hd, qe):
    """RMSE per state from per-axis error histories over the run window."""
    t = np.asarray(t, float) - t[0]
    ev = np.column_stack([win_slope(t, ep[:, k], t, 0.2) for k in range(3)])
    qde = np.column_stack([win_slope(t, qe[:, j], t, 0.2) for j in range(4)])
    out = {}
    for name, E, sc in (("com_pos", ep, 1e3), ("com_vel", ev, 1e3), ("att", body_rpy(np.degrees(eR)), 1.0),
                        ("rate", body_rpy(np.degrees(ew)) if ew is not None else None, 1.0), ("ee_pos", ee, 1e3)):
        if E is None:
            continue
        E = np.asarray(E, float) * sc
        for k, ax in enumerate(("x", "y", "z") if name in ("com_pos", "com_vel", "ee_pos") else ("roll", "pitch", "yaw")):
            out[f"{name}_{ax}"] = rms(E[:, k])
        out[f"{name}_norm"] = rms(np.linalg.norm(E, axis=1))
    out["ee_head"] = rms(hd)
    for j in range(4):
        out[f"q{j+1}"] = rms(np.degrees(qe[:, j]))
        out[f"qd{j+1}"] = rms(np.degrees(qde[:, j]))
    return out


def isaac_one(path):
    r = CP.load(path, True)
    ta, tb, _ = max(r["legs"], key=lambda x: x[1] - x[0])
    t, D = r["t"], r["D"]
    m = (t >= ta) & (t <= tb) & r["direct"]
    Dm = D[m]
    ep = Dm[:, 48:51] - Dm[:, 45:48]
    ee = ep + Dm[:, 24:27]
    hd = np.degrees(np.arcsin(np.clip(Dm[:, 27], -1, 1)))
    qe = Dm[:, 5:9] - Dm[:, 9:13]
    s = score(t[m], ep, np.arcsin(np.clip(Dm[:, 28:31], -1, 1)), None, ee, hd, qe)
    a, b = r["t_direct"]
    s["_meta"] = {"rtf": r["rtf"], "run_s": float(tb - ta), "aborted": bool(np.isfinite(b) and b < tb - 0.5),
                  "sat_pct": float(100 * np.mean(Dm[:, 51] > 0)),
                  "clamp_pct": float(100 * np.mean(np.any(np.abs(Dm[:, 13:17]) > 2.99, axis=1))),
                  "tau_pk": float(np.abs(Dm[:, 13:17]).max()),
                  "eR_mean_deg_model": np.degrees(np.arcsin(np.clip(Dm[:, 28:31], -1, 1)).mean(0)).tolist()}
    return s


def bench_one(a):
    law, prof, stream, seed = a
    p = dict(CB.BASE)
    if law == "h1b":
        p.update(json.load(open(os.path.join(AN, "tuned_H1b.json")))["best"])
    r = CB.simulate(p, CB.stream(stream), profile=prof, seed=seed, full=True)
    if r["verdict"] != "completed":
        return a, None, r
    F = r["F"]; t0, t1 = F["run"]
    m = (F["t"] >= t0) & (F["t"] <= t1)
    ep = F["xc"][m] - F["xcd"][m]
    return a, (F["t"][m], ep, F["eR"][m], F["ew"][m], F["ee"][m], F["hd"][m], F["qe"][m]), \
        {k: r[k] for k in ("ee_rms", "com_rms", "tau_pk", "sat_pct")}


def clock_check(law):
    """The bench on ISAAC's clock (rtf 0.48, mirror, showcase, seed 11): if it
    reproduces the Isaac flight, the Isaac-only rows are the wall-clock artefact."""
    p = dict(CB.BASE)
    if law == "h1b":
        p.update(json.load(open(os.path.join(AN, "tuned_H1b.json")))["best"])
    r = CB.simulate(p, CB.stream(SHOW), seed=11, full=True, rtf=0.48)
    F = r["F"]; t0, t1 = F["run"]
    m = (F["t"] >= t0) & (F["t"] <= t1)
    s = score(F["t"][m], F["xc"][m] - F["xcd"][m], F["eR"][m], F["ew"][m], F["ee"][m], F["hd"][m], F["qe"][m])
    s["_meta"] = {"eR_mean_deg_model": np.degrees(F["eR"][m].mean(0)).tolist(), "verdict": r["verdict"]}
    return law, s


def main():
    res = {"isaac": {}, "bench": {}, "isaac_clock_check": {}}
    for case, prof, _ in CASES:
        for law, pat in ISAAC.items():
            fp = os.path.join(DATA, pat.format(c=case))
            res["isaac"].setdefault(case, {})[law] = isaac_one(fp)
    jobs = [(law, prof, stream, sd) for case, prof, stream in CASES for law in ISAAC for sd in SEEDS]
    with mp.get_context("fork").Pool(min(len(jobs), 18)) as pool:
        got = pool.map(bench_one, jobs)
        for law, s_ in pool.map(clock_check, list(ISAAC)):
            res["isaac_clock_check"][law] = s_
    pooled = {}
    for (law, prof, stream, sd), H, meta in got:
        case = next(c for c, p_, s_ in CASES if p_ == prof and s_ == stream)
        pooled.setdefault((case, law), []).append((H, meta))
    for (case, law), runs in pooled.items():
        ok = [h for h, _ in runs if h is not None]
        if len(ok) < len(runs):
            res["bench"].setdefault(case, {})[law] = {"_meta": {"aborted_seeds": len(runs) - len(ok)}}
            continue
        # pool the seeds: concatenate the error histories (each scored with its own time base)
        per = [score(*h) for h in ok]
        s = {k: float(np.sqrt(np.mean([p[k] ** 2 for p in per]))) for k in per[0]}
        s["_meta"] = {"seeds": len(ok), "ee_rms_mm": [m_["ee_rms"] * 1e3 for _, m_ in runs],
                      "tau_pk": max(m_["tau_pk"] for _, m_ in runs), "sat_pct": max(m_["sat_pct"] for _, m_ in runs)}
        res["bench"].setdefault(case, {})[law] = s
    json.dump(res, open(os.path.join(AN, "circle_states.json"), "w"), indent=1)
    keys = [k for k in res["bench"]["flown"]["ship"] if not k.startswith("_")]
    for src in ("isaac", "bench"):
        print(f"== {src}")
        print(f"{'state':14}" + "".join(f"{c+'/'+l:>14}" for c, _, _ in CASES for l in ISAAC))
        for k in keys:
            row = f"{k:14}"
            for c, _, _ in CASES:
                for l in ISAAC:
                    v = res[src][c][l].get(k)
                    row += f"{v:14.2f}" if v is not None else f"{'—':>14}"
            print(row)
        for c, _, _ in CASES:
            for l in ISAAC:
                print(c, l, res[src][c][l]["_meta"])


if __name__ == "__main__":
    main()
