#!/usr/bin/env python3
"""The base above the handle: the law's CoM error (wb_control_debug x_c - x_cd, [48..50]-[45..47]) -- mean (standing offset)
vs oscillation; its dominant frequency; and the base speed from three sources: the odometry the law/planner use (raw mocap
here), PX4's EKF2 velocity (vehicle_odometry, IMU-fused) and the raw VRPN -- to tell real motion from feedback noise.
Usage: base_wander.py <npz>..."""
import sys, os, re, glob
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
import importlib.util
_spec = importlib.util.spec_from_file_location("bpc", os.path.join(HERE, "..", "..", "wb_box_pushing_flight_20261021", "tools", "bp_common.py"))
_c = importlib.util.module_from_spec(_spec); _spec.loader.exec_module(_c)
sys.path.insert(0, HERE)
from claw_motion import phases

def wbdebug(b):
    t = b.t("wb"); d = b.f("wb", "data")
    L = max(len(r) for r in d)
    keep = np.array([len(r) >= 57 for r in d])
    D = np.array([np.asarray(r[:57], float) for r, k in zip(d, keep) if k])
    return t[keep], D

def dom_freq(t, x):
    if len(t) < 50 or t[-1] - t[0] < 2.0: return np.nan, np.nan
    tt = np.arange(t[0], t[-1], 0.02)
    y = np.interp(tt, t, x); y = y - y.mean()
    f = np.fft.rfftfreq(len(y), 0.02); P = np.abs(np.fft.rfft(y * np.hanning(len(y)))) ** 2
    m = (f > 0.05) & (f < 5)
    i = np.argmax(P[m]); return f[m][i], np.sqrt(np.sum(P[m][(f[m] > f[m][i]*0.7) & (f[m] < f[m][i]*1.3)]) / np.sum(P[m]))

def analyse(path):
    _c.NPZ = os.path.dirname(path); tag = os.path.basename(path)[1:-4]; b = _c.Bag(tag)
    ph = phases(b)
    t, D = wbdebug(b)
    direct = D[:, 0] > 0.5
    e = D[:, 48:51] - D[:, 45:48]
    # base speed sources
    to = b.t("odom"); po = np.stack([b.f("odom", f"pose.pose.position.{c}") for c in "xyz"], 1)
    vo = np.stack([b.f("odom", f"twist.twist.linear.{c}") for c in "xyz"], 1)
    tp = b.t("vodom_px") if b.has("vodom_px") else None
    if tp is not None:
        vp = np.stack(b.f("vodom_px", "velocity")).astype(float)   # NED
    ws = []
    if ph["direct"] and ph["gts"]: ws.append(("HOVER", ph["direct"] + 3, ph["gts"]))
    if ph["gts_done"]: ws.append(("ABOVE", ph["gts_done"] + 1.0, ph["ready"] or ph["abort"] or ph["gts_done"] + 15))
    if ph["ready_done"]: ws.append(("ON", ph["ready_done"], ph["push"] or ph["abort"] or ph["ready_done"] + 10))
    print(f"===== {tag}")
    for lab, a, c in ws:
        w = (t >= a) & (t <= c) & direct
        if w.sum() < 20: continue
        eh = e[w, :2]; mean = eh.mean(0); osc = eh - mean
        fx, sx = dom_freq(t[w], e[w, 0]); fy, sy = dom_freq(t[w], e[w, 1])
        wo = (to >= a) & (to <= c)
        vodo = np.linalg.norm(vo[wo, :2], axis=1)
        s = f"  {lab:6s} {c-a:5.1f} s | CoM err mean ({mean[0]*1e3:+5.0f},{mean[1]*1e3:+5.0f}) mm |mean| {np.linalg.norm(mean)*1e3:4.0f}, osc rms ({osc[:,0].std()*1e3:4.0f},{osc[:,1].std()*1e3:4.0f}) mm, p-p ({np.ptp(eh[:,0])*1e3:4.0f},{np.ptp(eh[:,1])*1e3:4.0f}) | f_dom x {fx:4.2f} Hz y {fy:4.2f} Hz"
        s += f" | |v_xy| odom med/p90 {np.median(vodo)*1e3:4.0f}/{np.percentile(vodo,90)*1e3:4.0f}"
        if tp is not None:
            wp = (tp >= a) & (tp <= c)
            if wp.sum() > 5:
                vpp = np.linalg.norm(vp[wp, :2], axis=1)
                s += f", EKF2 {np.median(vpp)*1e3:4.0f}/{np.percentile(vpp,90)*1e3:4.0f} mm/s"
        print(s)

if __name__ == "__main__":
    for p in sys.argv[1:]:
        analyse(p)
