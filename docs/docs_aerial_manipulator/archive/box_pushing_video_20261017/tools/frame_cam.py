#!/usr/bin/env python3
"""Project the mission's key points through a pinhole camera (USD default aperture
20.955 x 15.29 mm) to choose the video camera. Usage: frame_cam.py px py pz tx ty tz focal [fit]"""
import sys
import numpy as np
W, H = 1244, 1400
HA = 20.955
pos, tgt, f = np.array(sys.argv[1:4], float), np.array(sys.argv[4:7], float), float(sys.argv[7])
fit = sys.argv[8] if len(sys.argv) > 8 else "h"
fwd = tgt - pos; fwd /= np.linalg.norm(fwd)
right = np.cross(fwd, [0, 0, 1.0]); right /= np.linalg.norm(right)
up = np.cross(right, fwd)
# pixels per unit of tan: horizontal fit -> W spans HA/f; vertical fit -> H spans VA/f (VA = HA*H/W)
k = (W / (HA / f)) if fit == "h" else (H / (HA * 0.7297 / f))
PTS = {"start ground": (0.10, -0.07, 0.0), "start hover": (0.10, -0.07, 1.3),
       "table L top": (1.2, 0.5, 0.8), "table R top": (1.2, -0.5, 0.8), "table L foot": (1.2, 0.5, 0.0),
       "table R foot": (1.35, -0.5, 0.0), "box": (1.275, 0.25, 0.85), "box end": (1.275, -0.25, 0.85),
       "body push0": (1.275, 0.47, 1.1), "body push1": (1.275, -0.03, 1.1),
       "land start": (-0.5, 0.0, 1.3), "land": (-0.5, 0.0, 0.8)}
for n, p in PTS.items():
    d = np.array(p) - pos
    z = d @ fwd
    u = W / 2 + k * (d @ right) / z
    v = H / 2 - k * (d @ up) / z
    print(f"{n:13s} u {u:7.0f} v {v:7.0f}  dist {z:4.2f}  ({0.65 * k / z:4.0f} px per 0.65 m)")
