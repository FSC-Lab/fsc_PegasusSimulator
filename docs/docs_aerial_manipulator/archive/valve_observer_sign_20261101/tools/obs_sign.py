#!/usr/bin/env python3
"""(2026-11-01) Does the 4-D L1 observer read the valve's tangential force with the right sign in Isaac?

    PYTHONNOUSERSITE=1 /usr/bin/python3 obs_sign.py <runs dir> tag [tag ...]

Everything is the force ON THE CLAW, resolved on the TURN direction t (+ = the way the valve is being
turned, the rotor's +y at the fin for a + turn):
  truth   minus the PhysX contact force of the vehicle on the valve (contact [1:7], scene sign +1 =
          on the valve by the vehicle; checked per run: the vehicle's mean power on the rotor must be > 0)
  obs     the observer's raw F_hat (wb_control_debug [97..99], world axes, the force on the claw:
          J_yq^T F = joint-row residual)
  used    what the law consumed, F_hat_y ([58..60])
A valve that resists the turn pushes the claw BACKWARD, so the right sign on the turn axis is NEGATIVE.

Windows inside the turn (push:start + 0.3 .. push:end or the abort): ALL samples, and SLIPS = the valve
turning faster than SLIP_DEG_S (its rate from obj_0's yaw, 0.1 s central difference).
"""
import os
import sys

import numpy as np

D = 2                                                   # dbg column = debug index + 2
PIVOT = np.array([0.09349, 0.00136, 0.08989])           # obj_0 = the markers' centroid, rotor frame
G = np.array([0.160, 0.0006, 0.179])                    # the grip point, rotor frame
SLIP_DEG_S = float(os.environ.get("SLIP_DEG_S", "15"))


def q2R(q):
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def interp_rows(t, tt, A):
    return np.column_stack([np.interp(t, tt, A[:, k]) for k in range(A.shape[1])])


def run(path):
    d = np.load(path, allow_pickle=True)
    M = dict(zip([str(n) for n in d["marks_name"]], d["marks_t"]))
    t0 = M.get("push:start")
    t1 = M.get("push:end")
    if t0 is None:
        return None
    if t1 is None:
        ev = [str(e) for e in d["events"] if "ABORT" in str(e) or "left DIRECT" in str(e)]
        t1 = float(ev[0].split("s]")[0].strip("[ ")) if ev else d["dbg"][-1, 0]
    ct, box, dbg = d["contact"], d["box"], d["dbg"]
    g = np.arange(t0 + 0.3, t1, 0.01)
    B = interp_rows(g, box[:, 0], box[:, 1:8])
    yaw = np.unwrap(np.arctan2(2 * (box[:, 4] * box[:, 7] + box[:, 5] * box[:, 6]),
                               1 - 2 * (box[:, 6] ** 2 + box[:, 7] ** 2)))
    yaw_g = np.interp(g, box[:, 0], yaw)
    w = (np.interp(g + 0.05, box[:, 0], yaw) - np.interp(g - 0.05, box[:, 0], yaw)) / 0.1   # rad/s
    turn = np.sign(yaw_g[-1] - yaw_g[0]) or 1.0
    tdir = np.zeros((len(g), 3))
    for i in range(len(g)):
        R = q2R(B[i, 3:7])
        r = R @ (G - np.array([0.0, 0.0, G[2]]))                # horizontal part, rotor frame -> world
        t = np.cross([0.0, 0.0, 1.0], r)
        tdir[i] = turn * t / np.linalg.norm(t)
    Fv = interp_rows(g, ct[:, 0], ct[:, 1:4] + ct[:, 4:7])      # on the valve, by the vehicle
    power = np.mean(np.sum(Fv * tdir, 1) * np.abs(w))
    truth = -np.sum(Fv * tdir, 1)
    m = np.isfinite(dbg[:, 97 + D])
    obs = np.sum(interp_rows(g, dbg[m, 0], dbg[m, 97 + D:100 + D]) * tdir, 1)
    used = np.sum(interp_rows(g, dbg[m, 0], dbg[m, 58 + D:61 + D]) * tdir, 1)
    chi = np.interp(g, dbg[m, 0], dbg[m, 105 + D])
    wd = np.degrees(turn * w)
    slip = wd > SLIP_DEG_S
    mv = np.abs(wd) > SLIP_DEG_S                       # slips either way; sign(w) = the valve's motion
    sg = np.sign(wd)
    return dict(mv=mv, sg=sg, g=g, truth=truth, obs=obs, used=used, slip=slip, wd=wd, power=power, chi=chi,
                turn=np.degrees(yaw_g[-1] - yaw_g[0]), aborted=bool(d["aborted"]))


def stats(x, m):
    return (np.mean(x[m]), np.sqrt(np.mean((x[m] - np.mean(x[m])) ** 2))) if m.sum() > 5 else (np.nan, np.nan)


if __name__ == "__main__":
    d = sys.argv[1]
    print(f"slip = valve faster than {SLIP_DEG_S:.0f} deg/s along the turn; force ON THE CLAW along the turn "
          f"direction, mean (sd) N; correct sign = NEGATIVE")
    print(f"{'run':12s} {'turn':>5s} {'power':>6s} {'chi':>4s} | {'ALL: truth':>13s} {'obs':>13s} {'used':>13s} | "
          f"{'slip%':>5s} {'peak':>5s} | {'SLIP: truth':>13s} {'obs':>13s} {'used':>13s} | corr obs~truth")
    for tag in sys.argv[2:]:
        r = run(os.path.join(d, tag + ".npz"))
        if r is None:
            print(f"{tag:12s} no turn"); continue
        a = np.ones(len(r["g"]), bool); s = r["slip"]
        fmt = lambda v: f"{v[0]:+6.2f} ({v[1]:4.2f})"
        c = np.corrcoef(r["obs"], r["truth"])[0, 1]
        print(f"{tag:12s} {r['turn']:5.0f} {r['power']*1e3:5.1f}m {np.mean(r['chi']):4.2f} | {fmt(stats(r['truth'], a)):>13s} "
              f"{fmt(stats(r['obs'], a)):>13s} {fmt(stats(r['used'], a)):>13s} | {100*s.mean():4.0f}% {r['wd'].max():5.0f} | "
              f"{fmt(stats(r['truth'], s)):>13s} {fmt(stats(r['obs'], s)):>13s} {fmt(stats(r['used'], s)):>13s} | {c:+.2f}"
              f" || ALONG MOTION, |w|>thr ({100*r['mv'].mean():.0f}%): truth {np.mean(r['truth'][r['mv']]*r['sg'][r['mv']]):+.2f}"
              f" obs {np.mean(r['obs'][r['mv']]*r['sg'][r['mv']]):+.2f} used {np.mean(r['used'][r['mv']]*r['sg'][r['mv']]):+.2f}")
