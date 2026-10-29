#!/usr/bin/env python3
"""Round 4: a ROTATIONALLY STIFF grasp (the hardware energy split, energy_flow2.py, says the sway is pumped by
a roll-moment disturbance; the bench's grasp so far was position + a soft twist only, like Isaac's near-point
fingers).  k_t / k_v = stiffness about the fin's tangential / vertical axes, k_n about the jaws' axis."""
import sys, os
from concurrent.futures import ProcessPoolExecutor
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import valve_bench as V

def G(kt, kv, kn=0.3, c=0.02):
    return dict(k_t=kt, k_v=kv, k_n=kn, c=c)

CASES = {
    "S0  soft (as before)":            dict(),
    "S1  rot k_t=k_v=1":               dict(grip_rot=G(1, 1)),
    "S2  rot k_t=k_v=3":               dict(grip_rot=G(3, 3)),
    "S3  rot k_t=k_v=10":              dict(grip_rot=G(10, 10)),
    "S4  rot k_t=k_v=30":              dict(grip_rot=G(30, 30)),
    "S5  rot k_t=10 only":             dict(grip_rot=G(10, 0.3)),
    "S6  rot k_v=10 only":             dict(grip_rot=G(0.3, 10)),
    "S3c rot 10, contact flag OFF":    dict(grip_rot=G(10, 10), contact=False),
    "S3n rot 10, no drift":            dict(grip_rot=G(10, 10), fb=dict(V.WH.FB_RAW, force_noise=0.0)),
}

def run(a):
    name, kw, seed = a
    S = V.hold_stream()
    v, H = V.simulate(V.VALVE, S, seed=seed, **kw)
    return name, seed, V.score(S, v, H)

if __name__ == "__main__":
    jobs = [(n, kw, s) for n, kw in CASES.items() for s in (0, 1)]
    with ProcessPoolExecutor(14) as ex:
        for name, seed, o in ex.map(run, jobs):
            print(V.fmt(f"{name} s{seed}", o), flush=True)
