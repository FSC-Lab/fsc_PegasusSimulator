#!/usr/bin/env python3
"""Screen gain candidates on the world-held bench (wh_bench.py), hardware raw-mocap noise.

    nice -n 10 /usr/bin/python3 sweep.py [round] > ../sweep_<round>.txt

Each candidate flies the 10-21 room-held hover for t_settle = 20 s + the 2.5 s run, with 2
noise seeds (common random numbers: every candidate sees the same noise). A candidate PASSES a
seed when it completes and keeps q3 >= 10 deg (the planner's guard) and q2 <= 45 deg (the real
arm's limit) over the scored window. Ranked by the swing rms."""
import multiprocessing as mp
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import wh_bench as W   # noqa: E402

P = W.PUSH


def cand(**kw):
    c = dict(P)
    c.update(kw)
    return c


ROUNDS = {
    "r1": [
        ("baseline (push yaml)", cand()),
        ("k_w 1.5", cand(k_w=1.5)),
        ("k_w 1.8", cand(k_w=1.8)),
        ("k_R/k_w 2.134/1.567 (pick-and-drop)", cand(k_R=2.134, k_w=1.567)),
        ("k_x/k_v 32/12.58", cand(k_x=32.0)),
        ("k_x/k_v 32/16", cand(k_x=32.0, k_v=16.0)),
        ("k_x/k_v 20/10", cand(k_x=20.0, k_v=10.0)),
        ("k_v 16", cand(k_v=16.0)),
        ("D_y 10", cand(dy=10.0)),
        ("D_y 30", cand(dy=30.0)),
        ("K_y 80 / D_y 16.5", cand(ky=80.0, dy=16.5)),
        ("K_y 200 / D_y 26", cand(ky=200.0, dy=26.0)),
        ("omega_c_t 2.0", cand(omega_c_t=2.0)),
        ("omega_c_r 0.5", cand(omega_c_r=0.5)),
        ("mrd x1.2", cand(mrd_s=P["mrd_s"] * 1.2)),
        ("K_psi/D_psi 2.5/0.93", cand(ky_psi=2.5, dy_psi=0.93)),
    ],
}
ROUNDS["r2"] = [
    ("baseline (push yaml)", cand()),
    ("observer x0.7 (all three)", cand(omega_c_t=0.70, omega_c_r=0.52, omega_c_q=0.59)),
    ("observer x0.5 (all three)", cand(omega_c_t=0.50, omega_c_r=0.37, omega_c_q=0.42)),
    ("omega_c_t 0.5 only", cand(omega_c_t=0.50)),
    ("omega_c_r 0.37 only", cand(omega_c_r=0.37)),
    ("omega_c_q 0.42 only", cand(omega_c_q=0.42)),
    ("per-block omega_x 0.05", cand(omega_x_blk=0.05)),
    ("omega_q 0.1", cand(omega_q=0.1)),
    ("k_x 32 + observer x0.5", cand(k_x=32.0, omega_c_t=0.50, omega_c_r=0.37, omega_c_q=0.42)),
    ("k_x 32 + observer x0.7", cand(k_x=32.0, omega_c_t=0.70, omega_c_r=0.52, omega_c_q=0.59)),
]
ROUNDS["r3"] = [
    ("baseline (push yaml)", cand()),
    ("free-axis weight 0.25", cand(free_w=0.25)),
    ("free-axis weight 0.5", cand(free_w=0.5)),
    ("free-axis weight 0.75", cand(free_w=0.75)),
    ("free-axis weight 1.0", cand(free_w=1.0)),
    ("free-axis 0.5 + k_x 32 + observer x0.5", cand(free_w=0.5, k_x=32.0, omega_c_t=0.50, omega_c_r=0.37, omega_c_q=0.42)),
]
FEEDBACK = {
    "hw": W.FB_RAW,                                   # the 10-21 raw-mocap noise
    "isaac": dict(W.FB_RAW, vel_noise=0.022),          # what Isaac injects
}


def job(a):
    name, p, seed, fbk = a
    try:
        o = W.run(p, "world", fb=FEEDBACK[fbk], seed=seed)
    except Exception as e:      # a numerical failure is a failed case, not a crash of the sweep
        o = {"verdict": f"error {e!r}"[:60]}
    return name, seed, fbk, o


def ok(o):
    return (o["verdict"] == "completed" and o["q3_rng"][0] >= 10.0 and o["q2_rng"][1] <= 45.0)


SEEDS = [int(x) for x in os.environ.get('SEEDS', '0,1').split(',')]


def main():
    rnd = sys.argv[1] if len(sys.argv) > 1 else "r1"
    fbs = sys.argv[2].split(",") if len(sys.argv) > 2 else ["hw"]
    jobs = [(n, p, s, f) for n, p in ROUNDS[rnd] for f in fbs for s in SEEDS]
    with mp.Pool(min(int(os.environ.get('NPROC', '12')), len(jobs))) as pool:
        res = pool.map(job, jobs)
    by = {}
    for n, s, f, o in res:
        by.setdefault((n, f), []).append((s, o))
    rows = []
    for (n, f), lst in by.items():
        npass = sum(ok(o) for _, o in lst)
        sw = [o["swing_rms"] for _, o in lst if o["verdict"] == "completed"]
        rows.append((n, f, npass, np.mean(sw) if sw else np.inf, lst))
    rows.sort(key=lambda r: (-r[2], r[3]))
    for n, f, npass, _, lst in rows:
        print(f"\n[{f}] {n}: {npass}/{len(lst)} pass")
        for s, o in sorted(lst, key=lambda x: x[0]):
            print("   " + W.fmt(f"seed {s}", o))


if __name__ == "__main__":
    main()
