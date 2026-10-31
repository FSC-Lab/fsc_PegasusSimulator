#!/usr/bin/env python3
"""Wrist-roll kick at each gripper opening: for every bag npz given, the q4 excursion in the 2 s after the
jaws start opening.  Usage: grip_kick.py <archive>/*/npz/b*.npz  (numpy-1 system python)."""
import sys, glob, numpy as np
for f in sys.argv[1:]:
    try:
        z = np.load(f, allow_pickle=True)
        if "js__position" not in z.files: continue
        t = z["js__recv"]; t = t - t[0]; P = z["js__position"]; names = str(z["js__name"][0]).split(",") if z["js__name"].ndim == 1 else list(z["js__name"][0])
    except Exception as e:
        print(f, "ERR", e); continue
    names = [n.strip() for n in names]
    try: ig = names.index("gripper_left_joint"); i4 = names.index("joint4")
    except ValueError: print(f, "names", names); continue
    g = P[:, ig]; q4 = np.degrees(P[:, i4])
    # opening edges: gripper rises > 0.008 m within 1 s from a closed-ish value
    ev = []; i = 0
    while i < len(t):
        j = np.searchsorted(t, t[i] + 0.8)
        if j < len(t) and g[j] - g[i] > 0.012 and g[i] < 0.0:
            ev.append(i); i = np.searchsorted(t, t[i] + 3.0)
        else: i += 25
    out = []
    for i in ev:
        w = (t >= t[i] - 0.3) & (t <= t[i] + 2.0)
        out.append(f"open@{t[i]:.1f}s q4 {q4[i]:+.0f}->{q4[w][np.argmax(np.abs(q4[w]-q4[i]))]:+.0f} (|dq4|max {np.abs(q4[w]-q4[i]).max():.0f})")
    print(f.split("archive/")[-1], "|", "; ".join(out) if out else "no open edges")
