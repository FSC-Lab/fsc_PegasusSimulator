#!/usr/bin/env python3
"""lag.py <run.npz> [rig] -- command -> applied arm-torque lag (j2, j3) in a pnp_mission_v2 npz,
plus the arm module's peak torque and the time it left DIRECT. Rig: mod (modular_control_debug,
tau at debug[12..15]) or wb (wb_control_debug, tau at [13..16]). Debug column = index + 2."""
import sys
import numpy as np
f = sys.argv[1]
rig = sys.argv[2] if len(sys.argv) > 2 else "mod"
d = np.load(f, allow_pickle=True)
db = d["dbg"].astype(float)
j = d["joints"]
tm = float(d["marks_t"][0]) if len(d["marks_t"]) else db[0, 0]
off = (12 if rig == "mod" else 13) + 2
direct = db[:, 2] > 0.5
t_end = db[direct, 0].max() if direct.any() else tm
print(f"{f}: DIRECT {tm:.2f} -> {t_end:.2f} s ({t_end - tm:.2f} s)")
for t0 in np.arange(tm + 0.05, t_end - 1.0, 10.0):
    w = (db[:, 0] > t0) & (db[:, 0] < min(t0 + 6.0, t_end))
    tc = db[w, 0]
    out = []
    for jj in (1, 2):
        cmd = db[w, off + jj]
        if cmd.std() < 1e-4:
            continue
        c, lag = max((np.corrcoef(cmd, np.interp(tc + l, j[:, 0], j[:, 5 + jj]))[0, 1], l)
                     for l in np.arange(0.0, 0.12, 0.004))
        out.append(f"j{jj + 1} lag {lag * 1e3:.0f} ms (r {c:.2f}, cmd std {cmd.std():.3f})")
    print(f"  t+{t0 - tm:5.1f}: " + "; ".join(out))
w = direct
print("  peak |tau| in DIRECT:", np.round(np.abs(db[w, off:off + 4]).max(0), 2))
