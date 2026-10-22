#!/usr/bin/env python3
"""Calibrate the bench's disturbance model to the 10-21 hardware flight (hover CoM wander 14-19 mm
@0.12-0.19 Hz, world-held swing (q2-q3)/2 10.2 deg @0.54 Hz with the free axis off). Velocity noise
as the hardware's raw mocap (broadband ~45 mm/s, only 2-4 mm/s below 2 Hz); the candidate low-frequency
source is a slow PX4 attitude-estimate error (hardware 0.42-0.56 deg rms 2-D)."""
import multiprocessing as mp, os, sys
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import wh_bench as W   # noqa: E402
FBV = dict(W.FB_RAW, vel_noise=0.045, vel_bw_hz=100.0)
MODELS = {
    "vel 45 white, no att": dict(FBV),
    "att 0.35deg/axis 0.2Hz": dict(FBV, att_noise=0.0061, att_bw_hz=0.2),
    "att 0.35deg/axis 0.4Hz": dict(FBV, att_noise=0.0061, att_bw_hz=0.4),
    "att 0.50deg/axis 0.2Hz": dict(FBV, att_noise=0.0087, att_bw_hz=0.2),
    "old: vel 45 AR 5 Hz": dict(W.FB_RAW),
}
def job(a):
    name, ee, seed = a
    try:
        return name, ee, seed, W.run(W.PUSH, ee, fb=MODELS[name], seed=seed, free_w=0.0)
    except Exception as e:
        return name, ee, seed, {"verdict": repr(e)[:60]}
if __name__ == "__main__":
    jobs = [(n, ee, s) for n in MODELS for ee in ("relative", "world") for s in (0, 1)]
    with mp.Pool(int(os.environ.get("NPROC", "8"))) as pool:
        res = pool.map(job, jobs)
    for n, ee, s, o in sorted(res, key=lambda r: (list(MODELS).index(r[0]), r[1], r[2])):
        print(W.fmt(f"{n[:22]:22s} {'CoM-anchor' if ee == 'relative' else 'room-held'} s{s}", o)); sys.stdout.flush()
