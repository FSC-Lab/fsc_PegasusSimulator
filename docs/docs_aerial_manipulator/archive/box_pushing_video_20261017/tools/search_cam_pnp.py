#!/usr/bin/env python3
"""Grid-search the video camera for the whole-body PICK-AND-PLACE (2026-10-18; the
search_cam.py method): every mission point (+ the vehicle's half-size around the
flight points) inside the 1244 x 1400 pane, the vehicle as large as possible.
Points from the sim _pick_and_place.yaml: spawn (0.10, -0.07), Start (0, 0, 1.0),
PICK pillar (1, 1), PLACE pillar (-1, -1), Place Start (0, -1, 1.0), Land Start /
Land (-1, 0, 1.0 / 0.8); the body ~0.25 m behind the claw at the pillars."""
import itertools
import sys
import numpy as np
W, H, HA = 1244, 1400, 20.955
PTS_PLACE = [((0.10, -0.07, 0.15), 0.40), ((0.0, 0.0, 1.0), 0.40),
       ((1.0, 1.0, 1.0), 0.10), ((1.0, 1.0, 0.0), 0.08), ((0.80, 0.80, 1.25), 0.40),
       ((-1.0, -1.0, 1.0), 0.10), ((-1.0, -1.0, 0.0), 0.08), ((-0.75, -1.0, 1.30), 0.40),
       ((0.0, -1.0, 1.0), 0.40), ((-1.0, 0.0, 1.0), 0.40), ((-1.0, 0.0, 0.8), 0.40)]
# the PICK-AND-DROP (search_cam_pnp.py drop; the sim _pick_and_drop.yaml): 0.77 m pillars, Drop Start
# (0.8, -1, 1.0), the body ~1.35 m over the tray at (-1, -1) and backing out 0.20 m, Land Start / Land
# (-0.5, 0.8, 1.0 / 0.35)
PTS_DROP = [((0.10, -0.07, 0.15), 0.40), ((0.0, 0.0, 1.0), 0.40),
            ((1.0, 1.0, 0.85), 0.10), ((1.0, 1.0, 0.0), 0.08), ((0.80, 0.80, 1.20), 0.40),
            ((-1.0, -1.0, 0.82), 0.20), ((-1.0, -1.0, 0.0), 0.08), ((-0.75, -1.0, 1.40), 0.40),
            ((-0.55, -1.0, 1.50), 0.40), ((0.8, -1.0, 1.0), 0.40),
            ((-0.5, 0.8, 1.0), 0.40), ((-0.5, 0.8, 0.35), 0.40)]
DROP = len(sys.argv) > 1 and sys.argv[1] == "drop"
PTS = PTS_DROP if DROP else PTS_PLACE
C = np.mean([p for p, _ in PTS], axis=0)


def proj(pos, tgt, f, p):
    fwd = tgt - pos; fwd /= np.linalg.norm(fwd)
    r = np.cross(fwd, [0, 0, 1.0]); r /= np.linalg.norm(r)
    up = np.cross(r, fwd); k = W / (HA / f)
    d = np.array(p) - pos; z = d @ fwd
    return W / 2 + k * (d @ r) / z, H / 2 - k * (d @ up) / z, k / z


best = []
for az, el, dist, dz in itertools.product(range(0, 360, 5), range(20, 66, 3), np.arange(3.0, 7.01, 0.25), (-0.2, 0.0, 0.2)):
    tgt = C + np.array([0, 0, dz])
    a, e = np.radians(az), np.radians(el)
    pos = tgt + dist * np.array([np.cos(e) * np.cos(a), np.cos(e) * np.sin(a), np.sin(e)])
    lo, hi = 10.0, 80.0
    for _ in range(25):
        f = 0.5 * (lo + hi); ok = True
        for p, s in PTS:
            u, v, kz = proj(pos, tgt, f, p)
            if kz <= 0:
                ok = False; break
            m = s * kz + 30
            if not (m <= u <= W - m and m <= v <= H - m):
                ok = False; break
        lo, hi = (f, hi) if ok else (lo, f)
    f = lo
    scale = np.mean([proj(pos, tgt, f, p)[2] * 0.65 for p in ((0.80, 0.80, 1.25), (-0.75, -1.0, 1.35))])
    best.append((scale, az, el, dist, dz, f, pos, tgt))
best.sort(key=lambda b: -b[0])
seen = set()
for b in [b for b in best if not (b[1] in seen or seen.add(b[1]))][:20]:
    s, az, el, dist, dz, f, pos, tgt = b
    print(f"scale {s:5.0f}px  az {az:3d} el {el:2d} dist {dist:.2f} dz {dz:+.1f} focal {f:5.1f}  pos {np.round(pos, 2).tolist()} tgt {np.round(tgt, 2).tolist()}")
