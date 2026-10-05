#!/usr/bin/env python3
"""One-at-a-time sensitivity of the whole-body law's pick-and-place stability
(wb_payload_bench.py) around the shipped pick-and-place gains.

    OMP_NUM_THREADS=1 /usr/bin/python3 wb_payload_sweep.py [stage]
"""
import multiprocessing as mp
import sys

import numpy as np

import wb_payload_bench as B

S = B.SHIPPED
STAGES = {
    "oat": [("shipped", {})]
    + [(f"k_R {v}", dict(k_R=v)) for v in (1.2, 2.0, 2.4)]
    + [(f"k_w {v}", dict(k_w=v)) for v in (0.9, 1.5, 1.8)]
    + [(f"mrd_s {v}", dict(mrd_s=v)) for v in (1.3, 1.6, 2.0)]
    + [(f"omega_c_t {v}", dict(omega_c_t=v)) for v in (2.0, 4.0)]
    + [(f"omega_c_r {v}", dict(omega_c_r=v)) for v in (0.5, 1.0)]
    + [(f"omega_c_q {v}", dict(omega_c_q=v)) for v in (0.5, 1.2)]
    + [("k_x/k_v 40/11.3", dict(k_x=40.0, k_v=11.3)), ("k_x/k_v 60/15", dict(k_x=60.0, k_v=15.0))]
    + [("ky/dy 150/22", dict(ky=150.0, dy=22.0)), ("ky/dy 280/31", dict(ky=280.0, dy=31.0))]
    + [(f"omega_x {v}", dict(omega_x=v)) for v in (0.15, 0.3)],
    "combo": [("shipped", {}), ("k_w 1.5", dict(k_w=1.5)), ("k_w 1.8", dict(k_w=1.8)),
              ("k_w 2.1", dict(k_w=2.1)), ("k_w 1.8 + oct 3.0", dict(k_w=1.8, omega_c_t=3.0)),
              ("k_w 1.8 + k_R 1.4", dict(k_w=1.8, k_R=1.4)),
              ("k_w 1.5 + oct 3.5", dict(k_w=1.5, omega_c_t=3.5)),
              ("k_w 1.8 + kx/kv 55/14", dict(k_w=1.8, k_x=55.0, k_v=14.0))],
    "refine": [("shipped", {}), ("k_w 1.4", dict(k_w=1.4)), ("k_w 1.5", dict(k_w=1.5)),
               ("k_w 1.6", dict(k_w=1.6)), ("k_w 1.5 + k_R 1.4", dict(k_w=1.5, k_R=1.4)),
               ("k_w 1.5 + k_R 1.8", dict(k_w=1.5, k_R=1.8)),
               ("k_w 1.5 + oct 3.5", dict(k_w=1.5, omega_c_t=3.5)),
               ("k_w 1.4 + oct 3.5", dict(k_w=1.4, omega_c_t=3.5))],
}


def main():
    stage = sys.argv[1] if len(sys.argv) > 1 else "oat"
    delay = float(sys.argv[2]) if len(sys.argv) > 2 else 16.0
    C = STAGES[stage]
    jobs = [(nm, dict(S, **d), s, delay) for nm, d in C for s in range(2)]
    with mp.Pool(min(len(jobs), 18)) as pool:
        R = pool.map(B.job, jobs)
    by = {}
    for nm, s, dl, sc in R:
        by.setdefault(nm, []).append(sc)
    for nm, _ in C:
        print(B.fmt(nm, by[nm]), flush=True)


if __name__ == "__main__":
    main()
