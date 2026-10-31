#!/usr/bin/env python3
"""Is the contact sway driven by the RAW mocap feed?  Free DIRECT hover vs the contact phase, per flight:
(1) the velocity the law receives (odom twist = the raw mocap's finite difference) minus the derivative of
the odom position smoothed at 3 Hz -- broadband, and in the sway's 0.4-0.8 Hz band; (2) the position
noise (odom position minus its 5 Hz smooth); (3) the feed's rate, repeated samples and gaps; (4) PX4's
attitude (the law's) minus the mocap attitude, along the nose / across it (= the sway's direction), in
the 0.4-0.8 Hz band, and its correlation with the lateral CoM error; (5) the sway itself."""
import numpy as np
from vt_common import Bag, BAGS
from valve_axis import step_times

WIN = {"180652": {"free hover (DIRECT, before Plan)": (14.6, 18.4), "hold above the handle": (33.05, 34.7),
                  "CONTACT (push -> abort)": (38.7, 47.6)},
       "181833": {"free hover (DIRECT, before Plan)": (17.9, 20.2), "hold above the handle": (35.1, 36.65),
                  "CONTACT (push -> abort)": (40.6, 44.0)}}
dt = 0.01
def lp0(x, fc):          # zero-phase first-order, both directions
    a = 1 - np.exp(-2*np.pi*fc*dt); y = x.copy()
    for i in range(1, len(y)): y[i] = y[i-1] + a*(y[i] - y[i-1])
    for i in range(len(y)-2, -1, -1): y[i] = y[i+1] + a*(y[i] - y[i+1])
    return y
def band(x, f1, f2):
    X = np.fft.rfft(x - x.mean()); f = np.fft.rfftfreq(len(x), dt); X[(f < f1) | (f > f2)] = 0
    return np.fft.irfft(X, len(x))
def R_of(w, x, y, z):
    return np.array([[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)], [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
                     [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]])
P = np.diag([1.0, -1.0, -1.0]); NE = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1.0]])
if __name__ != "__main__": BAGS = []   # imported for WIN only
for tag in BAGS:
    b = Bag(tag); st = step_times(b)
    to = b.t("odom"); p = np.stack([b.f("odom", f"pose.pose.position.{c}") for c in "xyz"], 1)
    v = np.stack([b.f("odom", f"twist.twist.linear.{c}") for c in "xyz"], 1)
    tm = b.t("mocap"); tv = b.t("vatt"); qv = b.f("vatt", "q")
    qm = np.stack([b.f("mocap", f"pose.orientation.{c}") for c in "wxyz"], 1)
    tw = b.t("wb"); W = b.f("wb", "data"); ok = np.isfinite(W[:, 50]); tw, W = tw[ok], W[ok]
    print(f"===== {tag}   (odom child_frame '{b.f('odom', 'child_frame_id')[0]}', header '{b.f('odom', 'header.frame_id')[0]}')")
    for lab, (a, c) in WIN[tag].items():
        g = np.arange(a, c, dt)
        pg = np.stack([np.interp(g, to, p[:, i]) for i in range(3)], 1); vg = np.stack([np.interp(g, to, v[:, i]) for i in range(3)], 1)
        ps = np.stack([lp0(pg[:, i], 3.0) for i in range(3)], 1); vs = np.gradient(ps, dt, axis=0)
        ev = (vg - vs)[:, :2]; ep = (pg - np.stack([lp0(pg[:, i], 5.0) for i in range(3)], 1))[:, :2]
        # the nose frame from PX4 yaw (ENU)
        Rg = []; Rm = []
        for t in g:
            i = min(np.searchsorted(tv, t), len(tv)-1); j = min(np.searchsorted(tm, t), len(tm)-1)
            Rg.append(NE @ R_of(*qv[i]) @ P); Rm.append(R_of(*qm[j]))
        Rg, Rm = np.array(Rg), np.array(Rm)
        yaw = np.arctan2(Rg[:, 1, 0], Rg[:, 0, 0]).mean(); u = np.array([np.cos(yaw), np.sin(yaw)]); n = np.array([-u[1], u[0]])
        e_t = Rg[:, :2, 2] - Rm[:, :2, 2]              # tilt-vector error, rad
        eal, eac = e_t @ u, e_t @ n
        ec = np.stack([np.interp(g, tw, W[:, 48+i] - W[:, 45+i]) for i in range(2)], 1); lat = ec @ n; vlat = np.gradient(lat, dt)
        evl = ev @ n
        mm = (tm >= a) & (tm <= c); dts = np.diff(tm[mm]); px = b.f("mocap", "pose.position.x")[mm]
        rep = np.mean(np.diff(px) == 0) * 100
        B = lambda x: band(x, 0.4, 0.8)
        print(f"  {lab:34s} {c-a:4.1f} s | vel noise rms {np.linalg.norm(ev.std(0))*1e3:5.1f} mm/s, 0.4-0.8 Hz lateral {B(evl).std()*1e3:4.1f} mm/s"
              f" (x k_v 12.58 = {12.58*B(evl).std():.3f} N) | pos noise {np.linalg.norm(ep.std(0))*1e3:.2f} mm"
              f" | feed {1/np.median(dts):.0f} Hz, max gap {dts.max()*1e3:.0f} ms, repeats {rep:.1f} %")
        print(f"  {'':34s}        | PX4-mocap tilt err 0.4-0.8 Hz: along {np.degrees(B(eal).std()):.2f}, across {np.degrees(B(eac).std()):.2f} deg"
              f" (x 37 N = {37*B(eac).std():.2f} N lateral) | lateral sway 0.4-0.8 Hz {B(lat).std()*1e3:5.1f} mm, its velocity {B(vlat).std()*1e3:5.1f} mm/s"
              f" | corr(across err, sway) {np.corrcoef(B(eac), B(lat))[0,1]:+.2f}, (vel noise, sway vel) {np.corrcoef(B(evl), B(vlat))[0,1]:+.2f}")
