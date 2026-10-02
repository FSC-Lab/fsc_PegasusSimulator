#!/usr/bin/env python3
"""explore.py -- hand-picked geometric+L1 gain candidates on the bench, each flown
at the nominal 16 ms transport delay (A) and at 28 ms (B, the delay margin the
whole-body and modular tunes were both held to).

    OMP_NUM_THREADS=1 /usr/bin/python3 explore.py [--set name]
"""
import argparse
import json
import multiprocessing as mp
import os
import time

import geo_bench as GB

HERE = os.path.dirname(os.path.abspath(__file__))
HW = GB.GEO_HW

SETS = {
    "first": [
        ("HW", {}),
        ("kp8 kv8", dict(kp_xy=8, kv_xy=8)),
        ("kp16 kv12", dict(kp_xy=16, kv_xy=12)),
        ("kp32 kv16", dict(kp_xy=32, kv_xy=16)),
        ("kp16 kv12 kr2/0.8", dict(kp_xy=16, kv_xy=12, kr_xy=2.0, kw_xy=0.8)),
        ("kp32 kv16 kr2/0.8", dict(kp_xy=32, kv_xy=16, kr_xy=2.0, kw_xy=0.8)),
        ("krz1.5", dict(kr_z=1.5)),
        ("krz2.5 kwz0.5", dict(kr_z=2.5, kw_z=0.5)),
        ("kp16 kv12 kr2/0.8 krz2/0.45", dict(kp_xy=16, kv_xy=12, kr_xy=2.0, kw_xy=0.8, kr_z=2.0, kw_z=0.45)),
        ("wc10", dict(omega_c=10.0)),
        ("as10", dict(as_v=10.0, as_w=10.0)),
        ("kpz16 kvz12", dict(kp_z=16, kv_z=12)),
        ("kp16 kv12 kr1.5/0.7", dict(kp_xy=16, kv_xy=12, kr_xy=1.5, kw_xy=0.7)),
        ("kp24 kv14 kr2.5/0.9 krz2/0.45", dict(kp_xy=24, kv_xy=14, kr_xy=2.5, kw_xy=0.9, kr_z=2.0, kw_z=0.45)),
    ],
}


def job(args):
    nm, g, tag, kw = args
    t0 = time.time()
    r = GB.run(g, **kw)
    return nm, tag, r, time.time() - t0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--set", default="first")
    ap.add_argument("--jobs", type=int, default=28)
    ap.add_argument("--extra", default="", help="json list of [name, gains] to append")
    a = ap.parse_args()
    cands = list(SETS.get(a.set, []))
    if a.extra:
        cands += [tuple(x) for x in json.loads(open(a.extra).read())]
    tasks = []
    for nm, g in cands:
        tasks.append((nm, g, "A", dict()))
        tasks.append((nm, g, "B", dict(delay_ms=28.0)))
    res = {}
    with mp.get_context("fork").Pool(a.jobs) as pool:
        for nm, tag, r, dtw in pool.imap_unordered(job, tasks):
            res[(nm, tag)] = r
    out = []
    for nm, g in cands:
        A, B = res[(nm, "A")], res[(nm, "B")]
        b = "B ok" if B["verdict"] == "completed" else f"B ABORT@{B['t_end']:.0f}"
        if B["verdict"] == "completed":
            b += f" ({B['ee_rms']*1e3:.0f} mm, tilt {B['tilt_pk']:.1f})"
        print(GB.CB.fmt(nm, A) + " | " + b, flush=True)
        out.append(dict(name=nm, g=g, A=A, B=B))
    json.dump(out, open(os.path.join(HERE, "..", "analysis", f"explore_{a.set}.json"), "w"), default=float, indent=1)


if __name__ == "__main__":
    main()
