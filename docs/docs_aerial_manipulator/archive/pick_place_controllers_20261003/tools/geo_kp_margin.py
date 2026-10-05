#!/usr/bin/env python3
"""Transport-delay margin of the decoupled geometric + L1 law with STIFFER
position gains, on the 10-01 tuning bench (archive/geometric_l1_tune_20261001,
geo_bench.py: the node + the reference bridge + 06's position-mode servo on
circle_bench's mirror plant, Isaac's 60 Hz mocap feedback).

    OMP_NUM_THREADS=1 /usr/bin/python3 geo_kp_margin.py

Why (2026-10-03): in the pick-and-place runs the law's base drifted 7-10 cm
sideways while the payload was clamped on the cap (the closed chain), and its
steady hover offset is F/Kp ~ 37-79 mm. A stiffer Kp halves both -- if the law
keeps the margin the 10-01 tune held it to (44 ms; it diverged in Isaac at a
28 ms bar). Kv is scaled with sqrt(Kp) (the same damping ratio).
"""
import math
import multiprocessing as mp
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ARCH = os.path.abspath(os.path.join(HERE, "..", ".."))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", "..", "..", ".."))
for p in (os.path.join(ARCH, "sim2real_tuning_20260926", "tools"),
          os.path.join(REPO, "application", "robotic_arm", "utils"),
          os.path.join(REPO, "extensions", "fsc_aerial_manipulation"),
          os.path.join(ARCH, "circle_tune_20260927", "tools"),
          os.path.join(ARCH, "geometric_l1_tune_20261001", "tools")):
    sys.path.insert(0, p)
import geo_bench as GB   # noqa: E402

YAML = os.path.join(ARCH, "rtf_profile_20261001", "variants", "geometric_l1_mirror_sim.yaml")
BEST = dict(kp_xy=20.11070184217302, kv_xy=11.049739199786696, kp_z=13.504130000065347,
            kv_z=10.824266269370995, kr_xy=3.3365396649769536, kw_xy=0.9504995573655382,
            kr_z=1.7368344797710988, kw_z=0.41054038990874886, as_v=4.334146343238414,
            as_w=2.79059465699368, omega_c=1.0)


def cand(kp_xy, kp_z=40.0, kv_z=18.0, kv_xy=None):
    g = dict(BEST, kp_z=kp_z, kv_z=kv_z)
    g["kp_xy"] = kp_xy
    g["kv_xy"] = BEST["kv_xy"] * math.sqrt(kp_xy / BEST["kp_xy"]) if kv_xy is None else kv_xy
    return g


def job(a):
    name, g, d, seed = a
    r = GB.run(g, delay_ms=d, seed=seed, fb=GB.FB_ISAAC, yaml_path=YAML)
    return name, d, seed, r


def main():
    # 2026-10-03 second pass (tools/geo_sway_screen.py): the payload-carrying
    # 0.6 Hz sway wants a SOFTER position pair, not a stiffer one
    C = {"shipped (z 40/18)": cand(BEST["kp_xy"]), "kp_xy 30": cand(30.0),
         "kp 12.5 kv 9": cand(12.5, kv_xy=9.0), "kp 15 kv 10": cand(15.0, kv_xy=10.0)}
    if len(sys.argv) > 1 and sys.argv[1] == "--sway":
        C = {k: C[k] for k in ("shipped (z 40/18)", "kp 12.5 kv 9", "kp 15 kv 10")}
    jobs = [(n, g, d, s) for n, g in C.items() for d in (28.0, 36.0, 44.0) for s in (0, 1)]
    with mp.Pool(min(len(jobs), 18)) as pool:
        R = pool.map(job, jobs)
    print("candidate            delay  seed  verdict    EE rms mm  tilt pk  com rms mm")
    for name, d, s, r in sorted(R, key=lambda x: (x[0], x[1], x[2])):
        print(f"{name:20s} {d:4.0f}   {s}    {r['verdict']:9s}  {r.get('ee_rms', float('nan')) * 1e3:7.1f}   "
              f"{r.get('tilt_pk', float('nan')):5.1f}   {r.get('com_rms', float('nan')) * 1e3:7.1f}", flush=True)


if __name__ == "__main__":
    main()
