#!/usr/bin/env python3
"""Sidewall A/B on matched free flights (10-28 no wall / 11-02 wall up, same plans, same gains): per flight, over the
planner's HOLD windows (>= 2 s, first 1 s dropped) and its moving (EXECUTING) windows, without a payload:
  F_imu   the horizontal external force from the IMU: m x the accelerometer body x/y (thrust is along body z)
          (a body-fixed offset is constant there), band-passed 0.1-1 Hz (slow push) and 1-3 Hz (gusty) [N]
  M_hat   the law's own slow moment estimate (wb_control_debug [34..35], filtered), 0.05-1 Hz [mN.m]
  F_hat   the law's slow translational estimate ([31..32]), 0.1-1 Hz [N]
  tilt    the true (mocap) tilt's 0.1-1 Hz variation [deg] -- holds only (moving legs tilt on purpose)
  wander  the law's CoM error about its mean, rms [mm] -- holds only
Every signal is filtered over the WHOLE flight, then masked, so short windows carry no filter edge."""
import os, sys, re
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "wb_hover_wander_history_20261103", "tools"))
import history as H
M, G, DT = 3.746, 9.81, 0.01
ARCH = os.path.join(HERE, "..", "..")
FLIGHTS = [  # tag, npz dir, label, wall
    ("144602", "wb_hover_wander_history_20261103", "circle 0.13 10-28", 0), ("150821", "wb_hover_wander_history_20261103", "circle 0.13 10-28", 0),
    ("182844", "wb_sidewall_ab_20261103", "circle 0.13 11-02", 1),
    ("165603", "wb_hover_wander_history_20261103", "PnD 5 cm 10-28", 0), ("192323", "wb_sidewall_ab_20261103", "PnD 5 cm 11-02", 1),
    ("175015", "wb_hover_wander_history_20261103", "circle 0.20 10-27", 0), ("180659", "wb_hover_wander_history_20261103", "circle 0.10 10-27", 0),
    ("152202", "wb_hover_wander_history_20261103", "fig-8 0.13 10-28", 0), ("150625", "wb_hover_wander_history_20261103", "fig-8 0.10 10-29", 0),
]
def bp(x, lo, hi):
    y = x - np.polyval(np.polyfit(np.arange(len(x)), x, 1), np.arange(len(x)))
    F = np.fft.rfft(y); f = np.fft.rfftfreq(len(y), DT); F[(f < lo) | (f > hi)] = 0
    return np.fft.irfft(F, len(y))
def lp(x, hi):
    m = np.polyval(np.polyfit(np.arange(len(x)), x, 1), np.arange(len(x)))
    F = np.fft.rfft(x - m); f = np.fft.rfftfreq(len(x), DT); F[f > hi] = 0
    return np.fft.irfft(F, len(x)) + m
def analyse(tag, d):
    H._c.NPZ = os.path.join(ARCH, d, "npz"); b = H._c.Bag(tag)
    md = b.edges("wbmode"); t_dir = [t for t, s in md if s == "DIRECT"]; t_saf = [t for t, s in md if s == "SAFETY" and t > t_dir[0]]
    T0, T1 = t_dir[0] + 2.0, (t_saf[0] if t_saf else b.t("wbmode")[-1]) - 0.5
    T = np.arange(T0, T1, DT)
    tv = b.t("vrpn"); P = np.stack([b.f("vrpn", f"pose.position.{k}") for k in "xyz"], 1)
    Q = np.stack([b.f("vrpn", f"pose.orientation.{k}") for k in "wxyz"], 1)
    p = np.stack([np.interp(T, tv, P[:, k]) for k in range(3)], 1)
    q = np.stack([np.interp(T, tv, Q[:, k]) for k in range(4)], 1); q /= np.linalg.norm(q, axis=1)[:, None]
    a = np.stack([np.gradient(np.gradient(lp(p[:, k], 4.0), DT), DT) for k in range(3)], 1)
    w, x, y, z = q.T
    e3 = np.stack([2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)], 1)
    yaw = np.unwrap(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
    Tth = M * (G + a[:, 2]) / e3[:, 2]
    Fw = M * a[:, :2] - Tth[:, None] * e3[:, :2]
    c, s = np.cos(yaw), np.sin(yaw)
    Fb = np.stack([c * Fw[:, 0] + s * Fw[:, 1], -s * Fw[:, 0] + c * Fw[:, 1]], 1)
    # THE IMU: the accelerometer's body x/y reading is the non-thrust force / m (thrust lies along body z; a constant
    # thrust-axis misalignment and the accelerometer bias are removed by the band-pass)
    ts_ = b.t("sc"); acc = np.stack(b.f("sc", "accelerometer_m_s2")).astype(float)
    Fi = np.stack([np.interp(T, ts_, acc[:, k]) for k in range(2)], 1) * M
    sig = {"Fs": np.stack([bp(Fi[:, k], 0.1, 1.0) for k in range(2)], 1), "Fg": np.stack([bp(Fi[:, k], 1.0, 3.0) for k in range(2)], 1),
           "Fm": np.stack([bp(Fb[:, k], 0.1, 1.0) for k in range(2)], 1)}
    tw, D = H.wb(b)
    for name, cols, lo, hi in (("Mh", (34, 35), 0.05, 1.0), ("Fh", (31, 32), 0.1, 1.0)):
        sig[name] = np.stack([bp(np.interp(T, tw, D[:, k]), lo, hi) for k in cols], 1)
    tilt = np.stack([bp(e3[:, k], 0.1, 1.0) for k in range(2)], 1)
    ecom = np.stack([np.interp(T, tw, D[:, 48 + k] - D[:, 45 + k]) for k in range(2)], 1)
    # payload (pick-and-drop): obj_0 lifted above its rest = carried
    carried = np.zeros(len(T), bool)
    if "PnD" in tag_label[tag] and b.has("obj"):
        to = b.t("obj"); zo = b.f("obj", "pose.position.z"); z0 = np.median(zo[:50])
        zi = np.interp(T, to, zo); carried = zi > z0 + 0.03
        if carried.any():
            i0 = np.argmax(carried); i1 = len(carried) - np.argmax(carried[::-1]) - 1
            carried[i0:i1 + 1] = True
    st = b.edges("pl_status")
    hold = np.zeros(len(T), bool); move = np.zeros(len(T), bool)
    for i, (ts, v) in enumerate(st):
        te = st[i + 1][0] if i + 1 < len(st) else T1
        if v.startswith("HOLD") and te - ts >= 2.0: hold |= (T >= ts + 1.0) & (T <= te)
        if v.startswith("EXECUTING"): move |= (T >= ts) & (T <= te)
    hold &= ~carried; move &= ~carried
    out = {}
    for lab, m in (("hold", hold), ("move", move)):
        if m.sum() < 100: continue
        r = {k: np.sqrt((v[m] ** 2).mean(0).sum()) for k, v in sig.items()}
        r["s"] = m.sum() * DT
        if lab == "hold":
            r["tilt"] = np.degrees(np.sqrt((tilt[m] ** 2).mean(0).sum()))
            # CoM wander: about each hold's own mean
            segs = np.split(np.arange(len(T))[m], np.where(np.diff(np.arange(len(T))[m]) > 1)[0] + 1)
            r["wander"] = np.sqrt(np.mean(np.concatenate([((ecom[s_] - ecom[s_].mean(0)) ** 2).sum(1) for s_ in segs if len(s_) > 50]))) * 1e3
        out[lab] = r
    return out
tag_label = {t: l for t, _, l, _ in FLIGHTS}
if __name__ == "__main__":
    print(f"{'flight':20s} {'wall':4s} | {'HOLD s':>6s} {'F_imu slow':>10s} {'gust':>5s} {'M_hat':>6s} {'F_hat':>5s} {'tilt':>5s} {'wander':>6s} | {'MOVE s':>6s} {'F_imu slow':>10s} {'gust':>5s} {'M_hat':>6s} {'F_hat':>5s}")
    for tag, d, lab, wall in FLIGHTS:
        o = analyse(tag, d)
        h = o.get("hold", {}); mv = o.get("move", {})
        g = lambda r, k, sc=1, n=2: f"{r[k]*sc:.{n}f}" if k in r else "--"
        print(f"{lab:13s} {tag} {'yes' if wall else 'no':4s} | {g(h,'s',1,0):>6s} {g(h,'Fs'):>10s} {g(h,'Fg'):>5s} {g(h,'Mh',1e3,0):>6s} {g(h,'Fh'):>5s} {g(h,'tilt'):>5s} {g(h,'wander',1,1):>6s} | "
              f"{g(mv,'s',1,0):>6s} {g(mv,'Fs'):>10s} {g(mv,'Fg'):>5s} {g(mv,'Mh',1e3,0):>6s} {g(mv,'Fh'):>5s}")
