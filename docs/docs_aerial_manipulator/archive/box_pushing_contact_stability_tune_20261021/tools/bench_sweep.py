#!/usr/bin/env python3
"""Tier-3 screen of the box-pushing contact tune on the room-held bench (PLAN.md section 3/4).

    nice -n 19 taskset -c 16-31 /usr/bin/python3 bench_sweep.py <round> [w,..] > ../bench_<round>.txt

The bench is ../../box_pushing_gain_tune_20261021/tools/wh_bench.py, unmodified: the exact law
(controller.py + l1_observer.py) on the mirror plant, rotor lag, 16 ms transport, arm friction and
stops, the 10-21 flight's own room-held reference, raw-mocap feedback at the HARDWARE level
(4.5 cm/s AR(1) 5 Hz). No box. Each candidate x free-axis weight x seed; a seed PASSES when the
20 s hold completes with q3 >= 10 and q2 <= 45 (the guard / the real limit). The delay margin
(the first transport delay in DELAYS that fails a seed) is computed for every candidate that
passes all seeds at 16 ms. Ordering only -- the bench's absolute verdict is not a flight's.
"""
import multiprocessing as mp
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)      # this folder's wh_bench / circle_bench copies (the slow attitude error)
import wh_bench as W   # noqa: E402
# FB_MODEL=att (2026-10-22): the HARDWARE-CALIBRATED model -- the raw mocap's broadband velocity
# noise (45 mm/s, AR(1) 100 Hz: only ~4 mm/s below 2 Hz, as measured on 10-21) plus a slow PX4
# attitude-estimate error 0.35 deg per axis at 0.2 Hz (measured 0.42-0.56 deg 2-D); reproduces the
# hardware's hover wander (15-18 vs 14-19 mm) and room-held swing (5-8 deg, q3 to 7). FB_MODEL=vel
# (default) = the 10-21 gain tune's 4.5 cm/s AR(1) 5 Hz velocity noise.
FBS = {"vel": W.FB_RAW,
       "att": dict(W.FB_RAW, vel_noise=0.045, vel_bw_hz=100.0, att_noise=0.0061, att_bw_hz=0.2)}
FB = FBS[os.environ.get("FB_MODEL", "vel")]

P = W.PUSH
SEEDS = [int(x) for x in os.environ.get("SEEDS", "0,1,2").split(",")]
DELAYS = [16.0, 24.0, 32.0, 40.0]


def cand(**kw):
    c = dict(P)
    c.update(kw)
    return c


ROUNDS = {
    "base": [("shipped", cand())],
    "t3": [
        ("shipped", cand()),
        ("D1 omega_c_t 0.7", cand(omega_c_t=0.7)),
        ("D1 omega_c_t 1.5", cand(omega_c_t=1.5)),
        ("D2 omega_c_q 0.6", cand(omega_c_q=0.6)),
        ("D2 omega_c_q 1.2", cand(omega_c_q=1.2)),
        ("D3 omega_c_r 0.5", cand(omega_c_r=0.5)),
        ("D3 omega_c_r 1.0", cand(omega_c_r=1.0)),
        ("D4 D_y 25", cand(dy=25.0)),
        ("D4 D_y 30", cand(dy=30.0)),
        ("D5 K_y/D_y 90/17.5", cand(ky=90.0, dy=17.5)),
        ("D5 K_y/D_y 150/22.6", cand(ky=150.0, dy=22.6)),
        ("D6 omega_x_blk 0.05", cand(omega_x_blk=0.05)),
        ("D6 omega_x_blk 0.2", cand(omega_x_blk=0.2)),
        ("D7 k_x 40", cand(k_x=40.0)),
        ("D7 k_x 32", cand(k_x=32.0)),
        ("D8 k_w 1.5", cand(k_w=1.5)),
    ],
}
# the base wander under hardware-level velocity noise: the velocity gain multiplies the noisy
# mocap velocity (2026-10-22, after the Tier-3 round found no gain that moves the CoM wander)
ROUNDS["att"] = [
    ("shipped", cand()),
    ("omega_c_t 1.5", cand(omega_c_t=1.5)),
    ("omega_c_t 2.0", cand(omega_c_t=2.0)),
    ("omega_c_t 2.927", cand(omega_c_t=2.927)),
    ("k_x 64", cand(k_x=64.0)),
    ("k_x 64 + omega_c_t 2.0", cand(k_x=64.0, omega_c_t=2.0)),
    ("k_x/k_v 64/14", cand(k_x=64.0, k_v=14.0)),
    ("omega_c_t 0.7", cand(omega_c_t=0.7)),
    ("omega_c_r 1.0", cand(omega_c_r=1.0)),
    ("K_y/D_y 150/22.6", cand(ky=150.0, dy=22.6)),
    ("k_w 1.5", cand(k_w=1.5)),
]
ROUNDS["kv"] = [
    ("shipped", cand()),
    ("k_v 10", cand(k_v=10.0)),
    ("k_v 8", cand(k_v=8.0)),
    ("k_x/k_v 40/10", cand(k_x=40.0, k_v=10.0)),
    ("k_v 10 + omega_c_t 0.7", cand(k_v=10.0, omega_c_t=0.7)),
    ("k_v 15", cand(k_v=15.0)),
]


def job(a):
    name, p, w, seed, delay = a
    try:
        o = W.run(p, "world", fb=FB, seed=seed, delay_ms=delay, free_w=w)
    except Exception as e:      # a numerical failure is a failed case
        o = {"verdict": f"error {e!r}"[:60]}
    return name, w, seed, delay, o


def ok(o):
    return o["verdict"] == "completed" and o["q3_rng"][0] >= 10.0 and o["q2_rng"][1] <= 45.0


def main():
    rnd = sys.argv[1] if len(sys.argv) > 1 else "base"
    ws = [float(x) for x in (sys.argv[2] if len(sys.argv) > 2 else "0.75,0").split(",")]
    nproc = int(os.environ.get("NPROC", "8"))
    cands = ROUNDS[rnd]
    jobs = [(n, p, w, s, 16.0) for n, p in cands for w in ws for s in SEEDS]
    with mp.Pool(nproc) as pool:
        res = pool.map(job, jobs)
    by = {}
    for n, w, s, d, o in res:
        by.setdefault((n, w), {})[(s, d)] = o
    # the delay margin: the passing-at-16 candidates at the longer delays, until a seed fails
    todo = [k for k, v in by.items() if all(ok(v[(s, 16.0)]) for s in SEEDS)]
    for d in DELAYS[1:]:
        jobs = [(n, dict(cands)[n], w, s, d) for (n, w) in todo for s in SEEDS]
        if not jobs:
            break
        with mp.Pool(nproc) as pool:
            res = pool.map(job, jobs)
        for n, w, s, dd, o in res:
            by[(n, w)][(s, dd)] = o
        todo = [k for k in todo if all(ok(by[k][(s, d)]) for s in SEEDS)]
    for w in ws:
        rows = []
        for (n, ww), v in by.items():
            if ww != w:
                continue
            base = [v[(s, 16.0)] for s in SEEDS]
            npass = sum(ok(o) for o in base)
            margin = None
            for d in DELAYS:
                if not all((s, d) in v and ok(v[(s, d)]) for s in SEEDS):
                    margin = d
                    break
            sw = [o["swing_rms"] for o in base if o["verdict"] == "completed"]
            rows.append((n, npass, np.mean(sw) if sw else np.inf, margin, base))
        rows.sort(key=lambda r: (-r[1], r[2]))
        print(f"\n===== free-axis weight {w} (hardware raw-mocap noise; seeds {SEEDS}; first failing delay)")
        for n, npass, swm, margin, base in rows:
            ms = f"fails at {margin:.0f} ms" if margin else f"> {DELAYS[-1]:.0f} ms"
            print(f"\n[{w}] {n}: {npass}/{len(SEEDS)} pass at 16 ms | swing mean {swm:.2f} deg | margin {ms}")
            for s, o in zip(SEEDS, base):
                print("   " + W.fmt(f"seed {s}", o))
        sys.stdout.flush()


if __name__ == "__main__":
    main()
