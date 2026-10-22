#!/usr/bin/env python3
"""What drives the room-held swing on the bench? Feedback-noise and observer ablations at the push
yaml's gains, 2 seeds each (wh_bench.py)."""
import multiprocessing as mp
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import wh_bench as W
P = W.PUSH
FB = W.FB_RAW
CASES = [
    ("hw noise (4.5 cm/s, 5 Hz band)", P, FB, "world"),
    ("same noise, claw follows vehicle", P, FB, "relative"),
    ("Isaac level (2.2 cm/s, 5 Hz)", P, dict(FB, vel_noise=0.022), "world"),
    ("4.5 cm/s spread to 50 Hz (white-ish FD noise)", P, dict(FB, vel_bw_hz=50.0), "world"),
    ("no velocity noise (other noise on)", P, dict(FB, vel_noise=0.0), "world"),
    ("no noise at all (force bias only)", P, dict(W.CB.FB_IDEAL), "world"),
    ("hw noise, observer 3x slower", dict(P, omega_c_t=0.33, omega_c_r=0.25, omega_c_q=0.28), FB, "world"),
    ("hw noise, observer 3x faster", dict(P, omega_c_t=3.0, omega_c_r=2.2, omega_c_q=2.5), FB, "world"),
    ("hw noise, odo lag 30 ms", P, dict(FB, odo_lag_s=0.030), "world"),
]
def job(a):
    (n, p, fb, ee), s = a
    try:
        return n, s, W.run(p, ee, fb=fb, seed=s)
    except Exception as e:
        return n, s, {"verdict": f"error {e!r}"[:50]}
if __name__ == "__main__":
    jobs = [(c, s) for c in CASES for s in (0, 1)]
    with mp.Pool(12) as pool:
        res = pool.map(job, jobs)
    for c in CASES:
        print(f"\n{c[0]}")
        for n, s, o in res:
            if n == c[0]:
                print("   " + W.fmt(f"seed {s}", o))
