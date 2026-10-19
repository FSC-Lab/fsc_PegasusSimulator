#!/usr/bin/env python3
"""Grid-search the video camera: every mission point (+ the vehicle's half-size around
the flight points) inside the 1244 x 1400 pane, as large as possible."""
import itertools
import numpy as np
W, H, HA = 1244, 1400, 20.955
PTS = [((0.10, -0.07, 0.10), 0.40), ((0.10, -0.07, 1.3), 0.40), ((1.2, 0.5, 0.8), 0.05),
       ((1.35, -0.5, 0.8), 0.05), ((1.2, 0.5, 0.0), 0.05), ((1.35, -0.5, 0.0), 0.05),
       ((1.275, 0.25, 1.0), 0.40), ((1.275, -0.25, 1.0), 0.40), ((1.275, 0.6, 1.25), 0.40),
       ((-0.5, 0.0, 1.3), 0.40), ((-0.5, 0.0, 0.8), 0.40)]
C = np.mean([p for p, _ in PTS], axis=0)


def proj(pos, tgt, f, p):
    fwd = tgt - pos; fwd /= np.linalg.norm(fwd)
    r = np.cross(fwd, [0, 0, 1.0]); r /= np.linalg.norm(r)
    up = np.cross(r, fwd); k = W / (HA / f)
    d = np.array(p) - pos; z = d @ fwd
    return W / 2 + k * (d @ r) / z, H / 2 - k * (d @ up) / z, k / z


best = []
for az, el, dist, dz in itertools.product(range(110, 200, 5), range(25, 56, 3), np.arange(2.6, 5.01, 0.2), (-0.2, -0.1, 0.0, 0.1)):
    tgt = C + np.array([0, 0, dz])
    a, e = np.radians(az), np.radians(el)
    pos = tgt + dist * np.array([np.cos(e) * np.cos(a), np.cos(e) * np.sin(a), np.sin(e)])
    # largest focal at which everything fits (bisection)
    lo, hi = 10.0, 80.0
    for _ in range(25):
        f = 0.5 * (lo + hi); ok = True
        for p, s in PTS:
            u, v, kz = proj(pos, tgt, f, p); m = s * kz + 30
            if not (m <= u <= W - m and m <= v <= H - m):
                ok = False; break
        lo, hi = (f, hi) if ok else (lo, f)
    f = lo
    scale = np.mean([proj(pos, tgt, f, p)[2] * 0.65 for p, _ in PTS[6:9]])   # the vehicle at the box
    best.append((scale, az, el, dist, dz, f, pos, tgt))
best.sort(key=lambda b: -b[0])
seen = set()
for b in [b for b in best if not (b[1] in seen or seen.add(b[1]))][:18]:
    s, az, el, dist, dz, f, pos, tgt = b
    print(f"scale {s:5.0f}px  az {az:3d} el {el:2d} dist {dist:.1f} dz {dz:+.1f} focal {f:5.1f}  pos {np.round(pos, 2).tolist()} tgt {np.round(tgt, 2).tolist()}")
