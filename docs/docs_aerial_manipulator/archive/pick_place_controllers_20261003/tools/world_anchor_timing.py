#!/usr/bin/env python3
"""How long the WORLD-anchored hover lasts before it diverges (offline bench):
decides whether the anchor is safe for a short descent + grasp window only."""
import sys
import numpy as np
import pose_hover_screen as S

for pose in [(0, -30, 30, 0), (0, -20, 40, 0)]:
    for seed in range(3):
        r = S.HS.run(S.GAINS, "world", T=60.0, seed=seed, pose=pose, delay_ms=16.0, keep=True)
        H = r.get("H")
        msg = f"{str(pose):18s} seed {seed}: {r['verdict']:9s} t_end {r['t_end']:5.1f} s"
        if H is not None:
            t, ee, tilt = H["t"], H["ee"], H["tilt"]
            n = len(t)
            # first time after the settle window that the EE error exceeds 20 mm / tilt 10 deg
            for name, x, lim in (("EE>20mm", ee, 0.02), ("tilt>10deg", tilt, 10.0)):
                idx = np.nonzero(x[:n] > lim)[0]
                idx = idx[t[idx] > 2.0] if len(idx) else idx
                msg += f"  {name} at {t[idx[0]]:.1f} s" if len(idx) else f"  {name} never"
        print(msg, flush=True)
