#!/usr/bin/env python3
"""The push-axis impedance change (2026-10-25) on the 10-24 evening clamped-box bench.

wb_box_pushing_flight_20261024_evening/tools/box_bench.py (the exact law + L1 observer, the box clamped
in the jaws on a Coulomb table, raw-mocap feedback + the hardware-like drift) with ONE switch: the
shaped task inertia and damping along the push (world y at the flown heading -90 deg).
  base : M_y 1.0,  D_y 20.2 (the 10-09 push tune)
  new  : M_y 1.64, D_y 25.8 (M_y = Lambda along the push; same damping ratio 0.92 at K_y 120)
Cases: the clamp moment k_c 6 (the bench's closest to the flight), then with the hardware's contact
reading noise added to F_y (N1, N2), and a hover on the push reference with no box (free flight).
The bench flies the 10-24 flight's own hold, i.e. the flown pose [0, 32, 28, 0] (Lambda 1.70 there).

Usage: imp_bench.py          (SEEDS=1,2,3 JOBS=8 CASES=... to narrow)
"""
import os
import sys
from concurrent.futures import ProcessPoolExecutor

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "wb_box_pushing_flight_20261024_evening", "tools"))
import box_bench as BB          # noqa: E402

CB, CS = BB.CB, BB.CS
_make_cfg = CB.make_cfg


def make_cfg(p):
    """CB.make_cfg plus the per-axis push-y keys (my_y, dy_y); absent = the bench's isotropic values."""
    if "my_y" not in p and "dy_y" not in p:
        return _make_cfg(p)
    return CS.hw_gains(k_x=p["k_x"], k_v=p["k_v"], k_R=p["k_R"], k_w=p["k_w"],
                       mrd=tuple(CB.MRD0 * p["mrd_s"]),
                       ky=(p["ky"], p["ky"], p["ky"], p["ky_psi"]),
                       dy=(p["dy"], p.get("dy_y", p["dy"]), p["dy"], p["dy_psi"]),
                       my=(1.0, p.get("my_y", 1.0), 1.0, 0.05),
                       dls_lambda=p["dls"])


CB.make_cfg = make_cfg

P0 = dict(BB.P)
P1 = dict(BB.P, my_y=1.64, dy_y=25.8)
S_KW = dict(dist=0.40, T_push=0.40 / 0.021)
B28 = dict(m=0.28, k_c=6.0)
CASES = {
    "K6": ("280 g, clamp k_c 6", dict(box=B28, fb=BB.HW)),
    "N1": ("280 g, + F_y noise 1.0/0.3/0.3 N", dict(box=B28, fb=BB.HW, fy_noise=(1.0, 0.3, 0.3, 0.3))),
    "N2": ("280 g, + F_y noise 1.5/0.3/0.3 N", dict(box=B28, fb=BB.HW, fy_noise=(1.5, 0.3, 0.3, 0.3))),
    "A0": ("no box (hover on the push ref)", dict(box=None, fb=BB.HW)),
}


def job(a):
    name, p, kw, seed = a
    return BB.job((name, p, kw, S_KW, seed))


if __name__ == "__main__":
    sel = os.environ.get("CASES", ",".join(CASES)).split(",")
    seeds = [int(s) for s in os.environ.get("SEEDS", "1,2,3").split(",")]
    jobs = [(f"{c} {tag}", p, CASES[c][1], s) for c in sel for tag, p in (("base", P0), ("new ", P1)) for s in seeds]
    names = {f"{c} {tag}": f"{c} {tag} {CASES[c][0]}" for c in sel for tag in ("base", "new ")}
    with ProcessPoolExecutor(max_workers=int(os.environ.get("JOBS", "8"))) as ex:
        for n, seed, o in ex.map(job, jobs):
            print(BB.fmt(f"{names[n]} [s{seed}]", o), flush=True)
