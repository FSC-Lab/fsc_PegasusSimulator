#!/usr/bin/env python3
"""The slosh predictor's measurement noises from a real hover bag (hardware-transfer piece 7).

  ~/envs/isaac_env/bin/python3 scripts/ekf_retune.py <bag dir|db3> [--window T0 T1] [--gate LOG.csv] [--rf 0.3 --rm 0.1]

The EKF's r_F / r_M (SLOSH_PRED_RF / _RM, 0.3 N / 0.1 N.m from the 2026-09-14 sweep) were tuned on
the gate, whose residual floors are the sim's sensor noise through the twins: with the IMU force
channel and the momentum observer (imu-mdob) a still hover reads F_hat_y 0.033 N and sigma
(0.052, 0.067, 0.101) N.m rms (--gate re-measures them from a hover-only gate --log). A real
airframe's floors are its IMU's, its mocap's/EKF2's and its vibration's; the retune keeps the
sweep's RATIO of r to floor and scales r by (floor flown / floor gate), per channel, clamped to
[0.5x, 5x] so a bad window cannot send the filter somewhere the sweep never looked. Then fly the
hover again and score it (ana_slosh_est.py against the ring-test's bob, or the status feed's
pred corr) before the mission.

What it reads: the flown cascade's status feed in the bag (`.../cascade_adaptive/status` or
`.../cascade/status`, String at ~3-5 Hz: enough for a floor -- variance survives sampling; the
1.14 Hz bob is below its Nyquist): `slosh F=(x,y,z)` = the force twin's raw reading, `sig=(x,y,z)`
= the moment observer's raw reading (bags before 2026-09-16 lack it: the moment floor is then
unavailable, and Mres is the bias state, NOT a floor), `pred bob=(x,y)mm`, `bF`, `bM`.
The HOVER WINDOW is the longest run with V < 0.5 m/s and |tilt| < 5 deg lasting >= 5 s (or
--window). The force floor is the y (lateral) channel: an offset tank's fore-aft bob shows on x in
any hover with pitch motion; a bob estimate above 3 mm rms in the window is flagged (the window is
not still, or the tank is ringing -- wait, or pick another)."""
import argparse
import glob
import os
import re
import sqlite3
import sys

import numpy as np

GATE_F = 0.033                         # N, F_hat_y rms, imu-mdob hover on the gate (2026-09-16)
GATE_M = (0.052, 0.067, 0.101)         # N.m, sigma x/y/z rms, same run


def parse_feed(bag):
    from rclpy.serialization import deserialize_message
    from std_msgs.msg import String
    if os.path.isdir(bag):
        bag = glob.glob(os.path.join(bag, "*.db3"))[0]
    cur = sqlite3.connect(bag).cursor()
    tid = {n: i for n, i in cur.execute("select name, id from topics")}
    cands = [k for k in tid if k.endswith("/status") and ("cascade" in k)]
    if not cands:
        raise SystemExit("no cascade status feed in the bag")
    t0 = cur.execute("select min(timestamp) from messages").fetchone()[0]
    best = None
    for k in cands:
        rows = [((ts - t0) / 1e9, deserialize_message(d, String).data) for ts, d in
                cur.execute("select timestamp, data from messages where topic_id=? order by timestamp", (tid[k],))]
        n_on = sum(1 for _, s in rows if "settled=1" in s or "slosh F=" in s)
        if best is None or n_on > best[1]:
            best = (k, n_on, rows)
    return best[0], best[2]


VEC = lambda key, s, n: (lambda m: [float(v) for v in m.group(1).split(",")] if m else [np.nan] * n)(re.search(key + r"=\(([^)]*)\)", s))
NUM = lambda key, s: (lambda m: float(m.group(1)) if m else np.nan)(re.search(key + r"=([-+0-9.eE]+)", s))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag"); ap.add_argument("--window", nargs=2, type=float); ap.add_argument("--gate")
    ap.add_argument("--rf", type=float, default=0.3); ap.add_argument("--rm", type=float, default=0.1)
    a = ap.parse_args()
    topic, rows = parse_feed(a.bag)
    t = np.array([r[0] for r in rows]); S = [r[1] for r in rows]
    V = np.array([NUM("V", s) for s in S]); tilt = np.array([NUM("tilt", s) for s in S])
    F = np.array([VEC("slosh F", s, 3) for s in S]); sig = np.array([VEC("sig", s, 3) for s in S])
    bob = np.array([VEC(r"pred bob", s, 2) for s in S]); bF = np.array([VEC("bF", s, 2) for s in S]); bM = np.array([VEC("bM", s, 3) for s in S])
    has_sig = np.isfinite(sig[:, 0]).any()
    print(f"{a.bag}: {topic}, {len(t)} status samples over {t[-1] - t[0]:.0f} s ({len(t) / max(t[-1] - t[0], 1):.1f} Hz)"
          f"{'' if has_sig else ' -- no sig= field (a bag before 2026-09-16): moment floor unavailable'}")
    if a.window:
        w = (t >= a.window[0]) & (t <= a.window[1])
    else:
        still = (V < 0.5) & (np.abs(tilt) < 5.0) & np.isfinite(F[:, 1])
        best = (0, 0); i = 0
        while i < len(t):
            if still[i]:
                j = i
                while j + 1 < len(t) and still[j + 1]:
                    j += 1
                if t[j] - t[i] > best[1] - best[0]:
                    best = (t[i], t[j])
                i = j + 1
            else:
                i += 1
        if best[1] - best[0] < 5.0:
            raise SystemExit(f"no still hover >= 5 s in the feed (longest {best[1] - best[0]:.1f} s); give --window T0 T1")
        w = (t >= best[0]) & (t <= best[1])
    print(f"hover window {t[w][0]:.1f}-{t[w][-1]:.1f} s ({w.sum()} samples): V {np.nanmax(V[w]):.2f} m/s max, tilt {np.nanmax(np.abs(tilt[w])):.0f} deg max")
    bob_rms = np.sqrt(np.nanmean(bob[w] ** 2, axis=0))
    print(f"  predictor's bob in the window: rms ({bob_rms[0]:.1f}, {bob_rms[1]:.1f}) mm" + ("  !! > 3 mm: the tank is moving, this is not a floor" if bob_rms.max() > 3.0 else ""))
    print(f"  bias states: bF ({np.nanmean(bF[w, 0]):+.2f}, {np.nanmean(bF[w, 1]):+.2f}) N   bM ({np.nanmean(bM[w, 0]):+.2f}, {np.nanmean(bM[w, 1]):+.2f}, {np.nanmean(bM[w, 2]):+.2f}) N.m  (what the hover reveals: CG, IMU bias)")
    fF = float(np.nanstd(F[w, 1])); fFx = float(np.nanstd(F[w, 0]))
    gF, gM = GATE_F, np.array(GATE_M)
    if a.gate:
        A = np.genfromtxt(a.gate, delimiter=",", names=True); wg = A["t"] > 5.0
        gF = float(A["Fhat_y"][wg].std()); gM = np.array([A[c][wg].std() for c in ("sigma_x", "sigma_y", "sigma_z")])
        print(f"  gate floors from {a.gate}: F_y {gF:.3f} N, sigma ({gM[0]:.3f}, {gM[1]:.3f}, {gM[2]:.3f}) N.m")
    print(f"  FORCE floor (F_hat_y rms): flown {fF:.3f} N vs gate {gF:.3f}  (x: {fFx:.3f}, the fore-aft bob if the tank is offset)")
    kF = min(5.0, max(0.5, fF / gF)); rF = a.rf * kF
    line = f"  -> r_F {a.rf:.2f} x {kF:.2f} = {rF:.2f} N"
    if has_sig:
        fM = np.nanstd(sig[w], axis=0)
        print(f"  MOMENT floor (sigma rms): flown ({fM[0]:.3f}, {fM[1]:.3f}, {fM[2]:.3f}) N.m vs gate ({gM[0]:.3f}, {gM[1]:.3f}, {gM[2]:.3f})")
        kM = min(5.0, max(0.5, float(np.mean(fM / gM)))); rM = a.rm * kM
        line += f"   r_M {a.rm:.2f} x {kM:.2f} = {rM:.2f} N.m"
    else:
        rM = a.rm; line += f"   r_M {a.rm:.2f} (unchanged: no sig= in this bag)"
    print(line + "   (clamped to [0.5x, 5x] of the sweep's values)")
    print(f"  export SLOSH_PRED_RF={rF:.3f} SLOSH_PRED_RM={rM:.3f}   # then re-fly the hover and score the prediction before the mission")
    return 0


if __name__ == "__main__":
    sys.exit(main())
