#!/usr/bin/env python3
"""Fly every reference stream of the trajectory grid on circle_bench with a given
gain set (default: the shipped law) and tabulate the tracking errors.

    /usr/bin/python3 eval_grid.py [--gains shipped|<json with a "best" dict>] [--profile mirror] [--tag x]
Writes ../analysis/grid_<tag>.json.
"""
import argparse
import glob
import json
import multiprocessing as mp
import os

import numpy as np

import circle_bench as CB

HERE = os.path.dirname(os.path.abspath(__file__))


def job(a):
    name, p, prof, delay = a
    r = CB.simulate(p, CB.stream(name), profile=prof, delay_ms=delay)
    d = np.load(os.path.join(CB.DATA, f"stream_{name}.npz"), allow_pickle=True)
    v = np.hypot(d["wbref__r_ed_dot.x"], d["wbref__r_ed_dot.y"])
    r["ee_speed_mean"] = float(v[v > 0.02].mean()); r["meta"] = d["meta"].tolist()
    return name, r


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gains", default="shipped")
    ap.add_argument("--profile", default="mirror")
    ap.add_argument("--delay", type=float, default=16.0)
    ap.add_argument("--tag", default="shipped")
    ap.add_argument("--jobs", type=int, default=16)
    ap.add_argument("--only", default="", help="comma-separated stream names")
    a = ap.parse_args()
    p = dict(CB.BASE)
    if a.gains != "shipped":
        p.update(json.load(open(a.gains))["best"])
    names = sorted(os.path.basename(f)[7:-4] for f in glob.glob(os.path.join(CB.DATA, "stream_r0*_L*.npz")))
    if a.only:
        names = a.only.split(",")
    out = {}
    with mp.get_context("fork").Pool(a.jobs) as pool:
        for name, r in pool.imap_unordered(job, [(n, p, a.profile, a.delay) for n in names]):
            out[name] = r
            if r["verdict"] == "completed":
                print(f"{name:22} v {r['ee_speed_mean']:.3f} m/s | EEabs {r['ee_rms']*1e3:5.1f} (pk {r['ee_pk']*1e3:4.0f}) "
                      f"| CoM {r['com_rms']*1e3:5.1f} | head {r['head_rms']:.2f}° | |eR| {r['eR_mean']:.4f} | "
                      f"q2/q3 {r['q_err_rms'][1]:.2f}/{r['q_err_rms'][2]:.2f}° | tilt {r['tilt_pk']:.1f}", flush=True)
            else:
                print(f"{name:22} ABORT at {r['t_end']:.1f} s", flush=True)
    json.dump({"gains": p, "profile": a.profile, "res": out},
              open(os.path.join(HERE, "..", "analysis", f"grid_{a.tag}.json"), "w"), indent=1, default=float)


if __name__ == "__main__":
    main()
