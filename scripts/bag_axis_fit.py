#!/usr/bin/env python3
"""What does the PLANT do per unit of the controller's own commanded moment?

    source scripts/rosenv.sh; python3 scripts/bag_axis_fit.py <bag_dir> [--eng T] [--dur S] [--cg_dx M]

Fits, per body axis, angular acceleration = g * M_model(t - lag) over a hover window, where
M_model is the moment the controller's rotor model says it commanded (momentum prop through the
spin lag, reaction torque CQ*T, thrust x arm). 1/g is the EFFECTIVE inertia the aircraft flew
with; against the assembled (0.265, 0.303, 0.490) it is a direct measure of unmodelled plant
torque. Also prints the yaw-diagonal actuator statistics (FL+RR-FR-RL) per phase.

This is how the 2026-09-13 prop-spin cap was found: the 2026-09-12 23:55 hover bag fitted as
Izz_eff 0.11 (R^2 0.96) with a rail-to-rail diagonal at 2.7 Hz, while the geometric controller's
hover on the same rig had a diagonal of 0.000. application/vtol/09_vtol_inertia_probe.py then
measured the torque in Isaac directly. Use this first on any flight whose lateral behaviour the
gate does not reproduce.
"""
import argparse
import glob
import os
import sqlite3
import sys

import numpy as np
from rclpy.serialization import deserialize_message
from fsc_autopilot_ros2_msgs.msg import Mocap
from px4_msgs.msg import ActuatorMotors, ActuatorServos
from std_msgs.msg import String

SPIN = np.array([1, -1, -1, 1]); CQ = 0.12; TMAX = 18.0
X = np.array([0.282, 0.282, -0.289, -0.289]); Y = np.array([0.284, -0.284, 0.284, -0.284])
I_CTL = {"roll": 0.265, "pitch": 0.303, "yaw": 0.490}


def load(bagdir):
    bag = glob.glob(os.path.join(bagdir, "*.db3"))[0]
    con = sqlite3.connect(bag); cur = con.cursor()
    topics = {tid: name for tid, name, typ in cur.execute("select id, name, type from topics")}
    t0 = cur.execute("select min(timestamp) from messages").fetchone()[0]

    def rows(sub, cls):
        tid = [k for k, v in topics.items() if v.endswith(sub)][0]
        return [((ts - t0) / 1e9, deserialize_message(d, cls))
                for ts, d in cur.execute("select timestamp, data from messages where topic_id=? order by timestamp", (tid,))]
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--eng", type=float, default=None, help="cascade engage time [s]; default: first active_controller=cascade")
    ap.add_argument("--dur", type=float, default=28.0, help="hover window after engage [s] (stops at the first tilt > 0.15)")
    ap.add_argument("--cg_dx", type=float, default=0.0, help="airframe CG shift flown (VTOL_CG_DX) [m]")
    a = ap.parse_args()
    rows = load(a.bag)
    ac = rows("active_controller", String)
    eng = a.eng if a.eng is not None else next((t for t, m in ac if m.data == "cascade"), None)
    if eng is None:
        print("no cascade engage in this bag"); return 1
    mot = rows("/actuator_motors", ActuatorMotors); srv = rows("/actuator_servos", ActuatorServos)
    T = np.array([t for t, _ in mot]); M = np.array([m.control[:4] for _, m in mot])
    Ts = np.array([t for t, _ in srv]); tilt = np.array([m.control[0] for _, m in srv])
    t_tr = next((t for t, x in zip(Ts, tilt) if t > eng and x > 0.15), Ts[-1])
    a0, b0 = eng + 1.5, min(t_tr, eng + a.dur)
    diag = M[:, 0] + M[:, 3] - M[:, 1] - M[:, 2]
    sel = (T > a0) & (T < b0)
    d = diag[sel]
    print(f"cascade engaged at {eng:.1f} s; hover window {a0:.1f}-{b0:.1f} s ({b0 - a0:.1f} s); actuator rate {1/np.median(np.diff(T)):.0f} Hz")
    if len(d) > 100:
        x0 = d - d.mean(); f = np.fft.rfftfreq(len(x0), np.median(np.diff(T[sel]))); P = np.abs(np.fft.rfft(x0)) ** 2
        print(f"  yaw diagonal FL+RR-FR-RL: pk-pk {np.ptp(d):.2f} sd {d.std():.3f} peak {f[1 + np.argmax(P[1:])]:.1f} Hz "
              f"(quiet hover: sd < 0.02; the pre-fix rail-to-rail thrash: sd 0.4-0.8)")
    mc = [(t, m.twist.angular.x, m.twist.angular.y, m.twist.angular.z) for t, m in rows("/mocap", Mocap) if a0 < t < b0]
    if len(mc) < 100:
        print("  hover too short for the inertia fit"); return 0
    tm = np.array([r[0] for r in mc]); W = np.array([r[1:] for r in mc]); Wd = np.gradient(W, tm, axis=0)
    tmo = T[(T > a0 - 1) & (T < b0)]; thr = M[(T > a0 - 1) & (T < b0)]
    xs = X - a.cg_dx
    for tau in (0.06, 0.12):
        om = np.zeros(4); Mx = []; My = []; Mz = []; tprev = tmo[0]
        for t, th in zip(tmo, thr):
            dd = t - tprev; tprev = t; om += (th - om) * min(dd / tau, 1.0); Tn = TMAX * om ** 2
            Mx.append(np.sum(Tn * Y)); My.append(np.sum(-Tn * xs)); Mz.append(np.sum(-SPIN * CQ * Tn))
        for ax, Mm, name in ((0, Mx, "roll"), (1, My, "pitch"), (2, Mz, "yaw")):
            Mm = np.array(Mm); best = None
            for lag in np.arange(0.0, 0.16, 0.01):
                Mi = np.interp(tm - lag, tmo, Mm); Mi = Mi - Mi.mean(); w = Wd[:, ax] - Wd[:, ax].mean()
                if np.dot(Mi, Mi) < 1e-9 or w.var() < 1e-12:
                    continue
                g = float(np.dot(Mi, w) / np.dot(Mi, Mi)); r2 = 1 - (w - g * Mi).var() / w.var()
                if best is None or r2 > best[0]:
                    best = (r2, lag, g)
            if best is None:
                continue
            r2, lag, g = best
            Ie = abs(1 / g) if abs(g) > 1e-6 else float("nan")
            print(f"  spin tau {tau:.2f} {name:5s}: I_eff {Ie:6.3f} (assembled {I_CTL[name]:.3f}, x{I_CTL[name]/Ie if Ie == Ie and Ie > 0 else float('nan'):.1f})  "
                  f"lag {lag:.2f} s  R^2 {r2:.2f}  rate sd {np.degrees(W[:, ax].std()):5.1f} deg/s  modelled torque sd {Mm.std():.2f} N m")
    return 0


if __name__ == "__main__":
    sys.exit(main())
