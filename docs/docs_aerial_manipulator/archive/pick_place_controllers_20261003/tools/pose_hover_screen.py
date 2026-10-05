#!/usr/bin/env python3
"""Offline screen: the 4-D whole-body law HOVERING at a candidate pick / place
arm pose, CoM- or WORLD-anchored EE reference (the planner's pick anchor), on
circle_bench's mirror plant (exact law, hardware-like feedback, rotor lag,
16 ms transport, arm friction) with the pick-and-place yaml's gains.

    /usr/bin/python3 pose_hover_screen.py [--T 60] [--seeds 2] [--poses "0,-30,30,0;0,-20,40,0"]

Why (2026-10-03, user report): hovering at the pick pose [0, -30, 30, 0] the
system swings a little, which is worse during the pick and place. Scores the
SWING over the hold window: tilt peak-to-peak, |e_R|, the EE and CoM error,
the arm's joint motion, and the dominant tilt frequency.
"""
import argparse
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ARCH = os.path.abspath(os.path.join(HERE, "..", ".."))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", "..", "..", ".."))
# the archived tools' own relative paths broke when they moved into archive/
sys.path.insert(0, os.path.join(ARCH, "sim2real_tuning_20260926", "tools"))
sys.path.insert(0, os.path.join(REPO, "application", "robotic_arm", "utils"))
sys.path.insert(0, os.path.join(REPO, "extensions", "fsc_aerial_manipulation"))
sys.path.insert(0, os.path.join(ARCH, "circle_tune_20260927", "tools"))
sys.path.insert(0, os.path.join(ARCH, "pick_place_tune_20261001", "tools"))
import hover_anchor_screen as HS   # noqa: E402

# the pick-and-place yaml: H1b with k_R / k_w 1.6 / 1.2
GAINS = dict(HS.H1B, k_R=1.6, k_w=1.2)


def score(r, T_hold=30.0):
    H = r["H"]
    t = H["t"]
    m = t >= t[-1] - T_hold
    tilt = H["tilt"][m]
    q = np.asarray(r["Hq"])[m] if "Hq" in r else None
    out = dict(verdict=r["verdict"], tilt_pp=float(tilt.max() - tilt.min()), tilt_mean=float(tilt.mean()),
               eR=float(np.mean(H["eR"][m])), ee_std=float(np.std(H["ee"][m])) * 1e3,
               ee_mean=float(np.mean(H["ee"][m])) * 1e3, com_std=float(np.std(H["ex"][m])) * 1e3,
               tau_pk=float(H["tau"][m].max()))
    if q is not None and q.ndim == 2:
        out["q_std"] = float(np.degrees(np.std(q, axis=0)).max())
    # dominant tilt frequency
    x = tilt - tilt.mean()
    dt = float(np.median(np.diff(t[m])))
    f = np.fft.rfftfreq(len(x), dt)
    P = np.abs(np.fft.rfft(x * np.hanning(len(x))))
    sel = f > 0.05
    out["f_tilt"] = float(f[sel][np.argmax(P[sel])]) if sel.any() else float("nan")
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--T", type=float, default=60.0)
    ap.add_argument("--seeds", type=int, default=2)
    ap.add_argument("--anchors", default="relative,world")
    ap.add_argument("--poses", default="0,-30,30,0;0,-40,40,0;0,-20,40,0;0,-10,40,0;0,0,40,0;0,10,30,0;0,-25,45,0")
    ap.add_argument("--set", nargs="*", default=[])
    a = ap.parse_args()
    p = dict(GAINS)
    for kv in a.set:
        k, v = kv.split("=")
        p[k] = float(v)
    poses = [tuple(float(x) for x in s.split(",")) for s in a.poses.split(";")]
    print("pose                 anchor    verdict    tilt_pp  tilt   |eR|    EE mean/std mm  CoM std mm  q_std deg  f_tilt Hz  tau_pk")
    for pose in poses:
        for anchor in a.anchors.split(","):
            rows = []
            for s in range(a.seeds):
                r = HS.run(p, anchor, T=a.T, seed=s, pose=pose, delay_ms=16.0, keep=True)
                if r["verdict"] != "completed":
                    rows.append(dict(verdict=r["verdict"]))
                    continue
                rows.append(score(r))
            ok = [x for x in rows if x.get("verdict") == "completed"]
            if len(ok) < len(rows):
                print(f"{str(pose):20s} {anchor:9s} {'ABORT x%d' % (len(rows) - len(ok)):9s}")
                if not ok:
                    continue
            mean = {k: float(np.mean([x[k] for x in ok])) for k in ok[0] if k != "verdict"}
            print(f"{str(pose):20s} {anchor:9s} {'ok':9s} {mean['tilt_pp']:7.3f}  {mean['tilt_mean']:5.2f}  "
                  f"{mean['eR']:.4f}  {mean['ee_mean']:6.2f}/{mean['ee_std']:5.2f}    {mean['com_std']:6.2f}     "
                  f"{mean.get('q_std', float('nan')):6.3f}    {mean['f_tilt']:5.2f}     {mean['tau_pk']:.2f}",
                  flush=True)


if __name__ == "__main__":
    main()
