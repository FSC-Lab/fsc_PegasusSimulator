#!/usr/bin/env python3
"""Fit the REPLAY camera on a recorded take (2026-10-18): every logged prim (every
vehicle body + the task's moving body) over the replayed window, plus fixed scene
points given on the command line, must stay MARGIN px inside the 1244 x 1400 frame;
among the cameras that do, the one where the vehicle is largest. Grid over the
viewing direction (azimuth, elevation) and distance; focal fitted by bisection.

    fit_cam_log.py TAKE [--start EVENT[+-s]] [--end EVENT[+-s]] [--az A0 A1] [--el E0 E1]
                   [--fixed x,y,z ...] [--margin px] [--top n]
"""
import argparse
import itertools
import json
import os
import re
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
W, H, HA = 1244, 1400, 20.955
ap = argparse.ArgumentParser()
ap.add_argument("take")
ap.add_argument("--start", default="CLIMB-0.5")
ap.add_argument("--end", default="END")
ap.add_argument("--az", nargs=2, type=float, default=[0, 360])
ap.add_argument("--el", nargs=2, type=float, default=[20, 65])
ap.add_argument("--dist", nargs=2, type=float, default=[2.5, 6.0])
ap.add_argument("--fixed", action="append", default=[], help="x,y,z (repeat; --fixed=-1,-1,0 for negatives)")
ap.add_argument("--margin", type=float, default=50.0)
ap.add_argument("--top", type=int, default=12)
a = ap.parse_args()
D = os.path.join(HERE, "..", "takes", a.take)
tl = json.load(open(os.path.join(D, "timeline.json")))
meta = json.load(open(os.path.join(D, "poses.json")))


def event_time(spec):
    m = re.fullmatch(r"([A-Za-z_:]+)([+-][0-9.]+)?", spec)
    name, off = m.group(1), float(m.group(2) or 0.0)
    ts = [e["t"] for e in tl["events"] if e["ev"] == name or e.get("name") == name]
    if not ts:
        sys.exit(f"no event {name}")
    return ts[0] + off


N = meta["n_prims"]
raw = np.fromfile(os.path.join(D, "poses.bin"), dtype="<f8")
raw = raw[: (raw.size // (2 + 7 * N)) * (2 + 7 * N)].reshape(-1, 2 + 7 * N)
t0, t1 = event_time(a.start), event_time(a.end)
sel = (raw[:, 0] >= t0) & (raw[:, 0] <= t1)
P = raw[sel, 2:].reshape(-1, N, 7)[::5, :, :3].reshape(-1, 3)
P = P[np.all(np.isfinite(P), axis=1)]
# the vehicle's rotor tips reach ~0.19 m past the rotor bodies' origins: a 0.2 m sphere on every point
pts = [(p, 0.20) for p in P] + [(np.array(list(map(float, f.split(",")))), 0.05) for f in a.fixed]
X = np.array([p for p, _ in pts]); R = np.array([r for _, r in pts])
C = 0.5 * (X.min(axis=0) + X.max(axis=0))
body = raw[sel, 2:].reshape(-1, N, 7)[:, 0, :3]          # the first logged prim = the vehicle body
print(f"{len(P)} logged points over {t1 - t0:.1f} s, box {np.round(X.min(0), 2).tolist()} .. "
      f"{np.round(X.max(0), 2).tolist()}", flush=True)


def proj(pos, tgt, f):
    fwd = tgt - pos; fwd /= np.linalg.norm(fwd)
    r = np.cross(fwd, [0, 0, 1.0]); r /= np.linalg.norm(r)
    up = np.cross(r, fwd); k = W / (HA / f)
    d = X - pos; z = d @ fwd
    return W / 2 + k * (d @ r) / z, H / 2 - k * (d @ up) / z, k / z


best = []
for az, el, dist, dz in itertools.product(np.arange(a.az[0], a.az[1], 5.0), np.arange(a.el[0], a.el[1] + 0.1, 3.0),
                                          np.arange(a.dist[0], a.dist[1] + 1e-9, 0.25), (-0.2, 0.0, 0.2)):
    tgt = C + np.array([0, 0, dz])
    e, z_ = np.radians(el), np.radians(az)
    pos = tgt + dist * np.array([np.cos(e) * np.cos(z_), np.cos(e) * np.sin(z_), np.sin(e)])
    lo, hi = 8.0, 80.0
    for _ in range(22):
        f = 0.5 * (lo + hi)
        u, v, kz = proj(pos, tgt, f)
        m = R * kz + a.margin
        ok = np.all(kz > 0) and np.all((u >= m) & (u <= W - m) & (v >= m) & (v <= H - m))
        lo, hi = (f, hi) if ok else (lo, f)
    f = lo
    if f <= 8.01:
        continue
    fwd = tgt - pos; fwd /= np.linalg.norm(fwd)
    kz_body = (W / (HA / f)) / ((body - pos) @ fwd)
    best.append((float(np.median(kz_body)) * 0.65, az, el, dist, dz, f, pos, tgt))
best.sort(key=lambda b: -b[0])
seen = set()
for b in [b for b in best if not (b[1] in seen or seen.add(b[1]))][:a.top]:
    s, az, el, dist, dz, f, pos, tgt = b
    print(f"vehicle {s:4.0f}px (median)  az {az:5.1f} el {el:4.1f} dist {dist:.2f} focal {f:5.1f}  "
          f"pos {np.round(pos, 2).tolist()} target {np.round(tgt, 2).tolist()}")
