#!/usr/bin/env python3
"""Every valve flight's Go To Start transit (start mark -> above the handle, ~16-17 s, the same plan) and the start-mark
hover before it: the gyro roll / pitch rate in the 0.3-1 Hz band (the vehicle's slow modes) and 1-5 Hz, and the standing
roll trim (motor L-R). Three groups: before the wall (10-28/29), wall up but BEFORE the 11-01 crash (10-31, 11-01),
after the crash (11-03)."""
import os, sys, re
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "wb_hover_wander_history_20261103", "tools"))
import history as H
FL = [("180652", "before wall"), ("181833", "before wall"), ("145128", "before wall"), ("145526", "before wall"),
      ("174943", "wall, pre-crash"), ("175725", "wall, pre-crash"), ("180423", "wall, pre-crash"), ("211808", "wall, pre-crash"),
      ("173835", "wall, pre-crash"), ("174531", "wall, pre-crash"), ("175259", "wall, pre-crash"), ("175932", "wall, pre-crash (crashed)"),
      ("164233", "wall, post-crash"), ("164722", "wall, post-crash"), ("165100", "wall, post-crash"), ("165700", "wall, post-crash")]
def bp(x, lo, hi, dt):
    n = np.arange(len(x)); y = x - np.polyval(np.polyfit(n, x, 1), n)
    F = np.fft.rfft(y); f = np.fft.rfftfreq(len(y), dt); F[(f < lo) | (f > hi)] = 0
    return np.fft.irfft(F, len(y))
print(f"{'flight':8s} {'group':26s} | Go To Start s | gyro 0.3-1 Hz r/p | 1-5 Hz r/p deg/s | roll trim L-R")
for ts, grp in FL:
    p = H.find_npz(ts); H._c.NPZ = os.path.dirname(p); b = H._c.Bag(ts)
    ed = b.edges("pl_pstatus")
    a = next((t for t, s in ed if s.startswith("FLYING go_to_start")), None); c = next((t for t, s in ed if a and t > a and s.startswith("DONE go_to_start")), None)
    if a is None or c is None: print(ts, "no go_to_start"); continue
    md = b.edges("wbmode"); t_dir = [t for t, s in md if s == "DIRECT"][0]
    ts_ = b.t("sc"); g = np.degrees(np.stack(b.f("sc", "gyro_rad")).astype(float))
    T = np.arange(t_dir + 1, b.t("wbmode")[-1] - 0.5, 0.005)
    G = np.stack([np.interp(T, ts_, g[:, k]) for k in range(2)], 1)
    S = {(lo, hi): np.stack([bp(G[:, k], lo, hi, 0.005) for k in range(2)], 1) for lo, hi in ((0.3, 1), (1, 5))}
    m = (T >= a) & (T <= c)
    r = lambda k: np.sqrt((S[k][m] ** 2).mean(0))
    tm = b.t("act_motors"); mc = np.stack(b.f("act_motors", "control")).astype(float)[:, :4]
    wm = (tm >= a) & (tm <= c); mm = mc[wm].mean(0); roll = (mm[2] + mm[1]) - (mm[0] + mm[3])
    print(f"{ts:8s} {grp:26s} | {c - a:5.1f}        | {r((0.3, 1))[0]:4.2f} / {r((0.3, 1))[1]:4.2f}       | {r((1, 5))[0]:4.2f} / {r((1, 5))[1]:4.2f}        | {roll:+.3f}")
