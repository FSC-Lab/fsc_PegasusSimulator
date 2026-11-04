#!/usr/bin/env python3
"""The vehicle's wander in every STATIC planner hold (reference still), per flight, grouped by the arm pose:
the law's horizontal CoM error x_c - x_cd (wb_control_debug [48..50]-[45..47]) -- rms about its mean, p-p, mean;
the base speed (EKF2 velocity, vehicle_odometry); and the odometry-velocity noise the law feeds back
(odom twist minus EKF2 velocity, > 2 Hz part). Holds shorter than 3 s are skipped; the first 1.5 s of each hold dropped."""
import sys, os, re, glob
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
import importlib.util
_spec = importlib.util.spec_from_file_location("bpc", os.path.join(HERE, "..", "..", "wb_box_pushing_flight_20261021", "tools", "bp_common.py"))
_c = importlib.util.module_from_spec(_spec); _spec.loader.exec_module(_c)

def analyse(path, label=""):
    _c.NPZ = os.path.dirname(path); tag = os.path.basename(path)[1:-4]; b = _c.Bag(tag)
    tw = b.t("wb"); W = b.f("wb", "data")
    ok = np.array([len(r) >= 57 and r[0] > 0.5 for r in W])
    tw = tw[ok]; D = np.array([np.asarray(r[:57], float) for r, k in zip(W, ok) if k])
    if len(D) == 0: return
    e = D[:, 48:51] - D[:, 45:48]; xcd = D[:, 45:48]; q = D[:, 5:9]
    # static reference: x_cd changes < 1 mm over 0.5 s
    dt = np.median(np.diff(tw)); k = max(1, int(0.5 / dt))
    still = np.zeros(len(tw), bool); still[k:] = np.linalg.norm(xcd[k:] - xcd[:-k], axis=1) < 0.001
    # segments
    segs = []; i = 0
    while i < len(tw):
        if still[i]:
            j = i
            while j + 1 < len(tw) and still[j + 1]: j += 1
            if tw[j] - tw[i] >= 3.0: segs.append((i, j))
            i = j + 1
        else: i += 1
    tp = b.t("vodom_px"); vp = np.stack(b.f("vodom_px", "velocity")).astype(float)
    to = b.t("odom"); vo = np.stack([b.f("odom", f"twist.twist.linear.{c}") for c in "xyz"], 1)
    for i, j in segs:
        a = tw[i] + 1.5; c = tw[j]
        w = (tw >= a) & (tw <= c)
        if w.sum() < 50: continue
        eh = e[w, :2]; m = eh.mean(0); osc = eh - m
        qd = np.degrees(q[w].mean(0))
        pose = "home" if abs(qd[1] - 40) < 5 and abs(qd[2] - 40) < 5 else ("valve" if abs(qd[1] - 5) < 4 and abs(qd[2] - 25) < 4 else
               ("box" if abs(qd[1] - 28) < 4 and abs(qd[2] - 32) < 4 else f"[{qd[1]:.0f},{qd[2]:.0f}]"))
        z = xcd[w, 2].mean()
        wp = (tp >= a) & (tp <= c); sp = np.linalg.norm(vp[wp, :2], axis=1)
        # odom velocity noise: odom minus EKF2 (interp), > 2 Hz
        wo = (to >= a) & (to <= c)
        vpi = np.stack([np.interp(to[wo], tp, vp[:, 0]), np.interp(to[wo], tp, vp[:, 1])], 1)
        vpi = np.stack([vpi[:, 1], vpi[:, 0]], 1)   # NED (x north, y east) -> ENU (x east, y north)
        d = vo[wo, :2] - vpi
        print(f"{label}{tag} {a:6.1f}-{c:6.1f} ({c-a:4.1f} s) arm {pose:6s} z {z:4.2f} | CoM err osc rms ({osc[:,0].std()*1e3:4.1f},{osc[:,1].std()*1e3:4.1f}) "
              f"p-p ({np.ptp(eh[:,0])*1e3:4.0f},{np.ptp(eh[:,1])*1e3:4.0f}) mean ({m[0]*1e3:+4.0f},{m[1]*1e3:+4.0f}) mm | EKF2 |v| med/p90 "
              f"{np.median(sp)*1e3:3.0f}/{np.percentile(sp,90)*1e3:3.0f} mm/s | odom-EKF2 v rms ({d[:,0].std()*1e3:4.0f},{d[:,1].std()*1e3:4.0f}) mm/s")

if __name__ == "__main__":
    for p in sys.argv[1:]:
        try: analyse(p)
        except Exception as ex: print(p, "ERR", ex)
