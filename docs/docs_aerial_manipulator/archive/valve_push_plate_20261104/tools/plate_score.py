#!/usr/bin/env python3
"""Score the push-plate runs (2026-11-04). For each run: the fin-guard scorer's summary (outcome, the guards'
readings during the turn, valve turned, tilt, q3, DIRECT, contacts by link -- ../../valve_fin_guard_20261103/tools/
score_guard.py), then:
  NOSE   where the claw touched the valve during the turn, in the rotor frame (Isaac's contact report,
         runs/<tag>.contacts.csv): on the PLATE's face (radial x and height z ranges -> the footprint the plate must
         cover), on the fin, or anywhere else; the contact force along / across the push (mean, p95)
  YAW    the law's yaw torque command tau_z (wb_control_debug [23]) and its unallocated part ([54]) during the turn,
         the four motors' time on a limit, and the controller's sideways force reading on the claw (raw F_hat [97..99]
         low-passed 0.5 s, in the vehicle's axes)
Run: PYTHONNOUSERSITE=1 /usr/bin/python3 plate_score.py [runs/*.npz]"""
import glob
import os
import re
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
GUARD = os.path.join(HERE, "..", "..", "valve_fin_guard_20261103", "tools", "score_guard.py")
GRIPPER = ("/hub", "/left_grip", "/right_grip", "/left_link", "/right_link")
FACE_Y, FIN_BACK_Y = -0.0244, -0.0194
DBG = 2           # the recorder's dbg columns = the debug index + 2


def qrot(w, x, y, z):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def lp(x, t, tau):
    y = np.zeros_like(x)
    for i in range(1, len(x)):
        a = min(1.0, (t[i] - t[i - 1]) / tau)
        y[i] = y[i - 1] + a * (x[i] - y[i - 1])
    return y


def nose_lines(f, t0, t1, z=None):
    path = f[:-4] + ".contacts.csv"
    if not os.path.exists(path):
        return ["NOSE  no contact log"]
    Rt, Rp, Rq, C = [], [], [], []
    for line in open(path):
        v = line.rstrip().split(",")
        if v[0] == "R":
            Rt.append(float(v[1])); Rp.append([float(x) for x in v[2:5]]); Rq.append([float(x) for x in v[5:9]])
        elif v[0] == "C":
            C.append((float(v[1]), v[2], [float(x) for x in v[4:7]], [float(x) for x in v[10:13]]))
    if not C or not Rt:
        return ["NOSE  no contacts"]
    Rt = np.array(Rt); Rp = np.array(Rp); Rq = np.array(Rq)
    # the scene logs on ITS clock, the driver on its own: align on the first contact (the contact truth topic
    # in the npz carries the driver's receive times; contact_parts.py's rule)
    off = 0.0
    if z is not None:
        c = z["contact"]
        withpts = c[c[:, 20] > 0, 0]
        if len(withpts):
            off = min(t for t, *_ in C) - float(withpts.min())
    t0, t1 = t0 + off, t1 + off
    on_plate, on_fin, other = [], [], {}
    imp_plate = []
    for t, link, p, imp in C:
        if t < t0 or t > t1:
            continue
        k = min(np.searchsorted(Rt, t), len(Rt) - 1)
        Rm = qrot(*Rq[k])
        pr = Rm.T @ (np.array(p) - Rp[k])
        if link not in GRIPPER:
            other[link] = other.get(link, 0) + 1
            continue
        if pr[1] <= FIN_BACK_Y - 0.0005 and pr[2] > 0.10:
            on_plate.append(pr); imp_plate.append((t, Rm.T @ np.array(imp)))
        else:
            on_fin.append(pr)
    out = []
    if on_plate:
        P = np.array(on_plate)
        out.append(f"NOSE  on the plate face: {len(P)} contacts, radial x {P[:,0].min()*1e3:6.1f}..{P[:,0].max()*1e3:6.1f} mm "
                   f"(plate 100..220), height z {P[:,2].min()*1e3:6.1f}..{P[:,2].max()*1e3:6.1f} mm (plate 125..204), "
                   f"face y {P[:,1].min()*1e3:6.1f}..{P[:,1].max()*1e3:6.1f}")
        # the force on the valve per physics step (the impulses summed per step / dt)
        ts = np.array([t for t, _ in imp_plate]); I = np.array([i for _, i in imp_plate])
        steps = np.unique(np.round(ts, 4))
        F = np.array([I[np.isclose(ts, s, atol=2e-4)].sum(0) for s in steps]) / 0.004
        out.append(f"      the force on the plate [N]: along the push (+y) mean {F[:,1].mean():5.2f} p95 {np.percentile(F[:,1],95):5.2f} | "
                   f"radial |x| p95 {np.percentile(np.abs(F[:,0]),95):5.2f} | vertical |z| p95 {np.percentile(np.abs(F[:,2]),95):5.2f} "
                   f"({len(steps)} steps in contact)")
    else:
        out.append("NOSE  NO contact on the plate face during the turn")
    if on_fin:
        P = np.array(on_fin)
        out.append(f"      on the FIN / elsewhere on the rotor: {len(P)} contacts, y {P[:,1].min()*1e3:.0f}..{P[:,1].max()*1e3:.0f}, "
                   f"z {P[:,2].min()*1e3:.0f}..{P[:,2].max()*1e3:.0f} mm")
    if other:
        out.append(f"      NON-gripper contacts: {other}")
    return out


def yaw_lines(z, t0, t1):
    d = z["dbg"]
    w = (d[:, 0] >= t0) & (d[:, 0] <= t1)
    if not w.any() or d.shape[1] < 102:
        return ["YAW   no debug in the turn"]
    tz = d[w, 23 + DBG]; un = d[w, 54 + DBG]
    m = d[w, 41 + DBG:45 + DBG]
    lim = ((m < 0.005) | (m > 0.995)).any(1)
    od = z["odom"]
    qo = od[:, 7:11]
    yaw = np.arctan2(2 * (qo[:, 0] * qo[:, 3] + qo[:, 1] * qo[:, 2]), 1 - 2 * (qo[:, 2] ** 2 + qo[:, 3] ** 2))
    t = d[:, 0]
    F = np.nan_to_num(d[:, 97 + DBG:100 + DBG].astype(float))
    Fl = np.stack([lp(F[:, i], t, 0.5) for i in range(3)], 1)
    ps = np.interp(t, od[:, 0], np.unwrap(yaw))
    left = np.stack([-np.sin(ps), np.cos(ps)], 1)
    nose = np.stack([np.cos(ps), np.sin(ps)], 1)
    Fs = (Fl[:, :2] * left).sum(1)[w]
    Fn = (Fl[:, :2] * nose).sum(1)[w]
    return [f"YAW   tau_z |.| p95 {np.percentile(np.abs(tz),95):5.3f} max {np.abs(tz).max():5.3f} N.m | unallocated yaw max "
            f"{np.abs(un).max():5.3f} N.m | a motor on a limit {100 * lim.mean():4.1f} % of the turn | F_hat (0.5 s) "
            f"sideways |.| p95 {np.percentile(np.abs(Fs),95):4.2f} max {np.abs(Fs).max():4.2f} N, along the nose "
            f"{np.percentile(Fn,5):+5.2f}..{np.percentile(Fn,95):+5.2f} N"]


def main():
    files = sys.argv[1:] or sorted(glob.glob(os.path.join(HERE, "..", "sim", "runs", "*.npz")))
    for f in files:
        r = subprocess.run([sys.executable, GUARD, f], capture_output=True, text=True)
        print(r.stdout.rstrip())
        z = np.load(f, allow_pickle=True)
        mt = dict(zip(z["marks_name"], z["marks_t"]))
        t0 = mt.get("push:start")
        if t0 is None:
            print(f"{'':18s} (no turn)")
            continue
        t1 = mt.get("push:end", z["box"][-1, 0])
        for e in z["events"]:
            if re.search(r"push_pull: ABORTING", e):
                t1 = min(t1, float(re.match(r"\[\s*([\d.]+)s\]", e).group(1)))
                break
        for line in nose_lines(f, t0, t1, z) + yaw_lines(z, t0, t1):
            print(f"{'':18s} {line}")


if __name__ == "__main__":
    main()
