#!/usr/bin/env python3
"""Free-axis weight under the hardware-calibrated attitude-error model (bench, no box)."""
import multiprocessing as mp, os, sys
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import wh_bench as W   # noqa: E402
FB = dict(W.FB_RAW, vel_noise=0.045, vel_bw_hz=100.0, att_noise=0.0061, att_bw_hz=0.2)
def job(a):
    w, oc, seed = a
    p = dict(W.PUSH, omega_c_t=oc)
    try:
        return w, oc, seed, W.run(p, "world", fb=FB, seed=seed, free_w=w)
    except Exception as e:
        return w, oc, seed, {"verdict": repr(e)[:60]}
if __name__ == "__main__":
    jobs = [(w, oc, s) for w in (0.75, 0.9, 1.0) for oc in (1.0, 2.0) for s in (0, 1, 2)]
    with mp.Pool(10) as pool:
        res = pool.map(job, jobs)
    for w, oc, s, o in res:
        print(W.fmt(f"w {w:.2f} omega_c_t {oc:.1f} seed {s}", o))
