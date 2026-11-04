#!/usr/bin/env python3
"""The yaw moment the valve contact puts on the vehicle (2026-11-04, the user's question: with the plate the arm
feels no sideways force, so no large yaw torque?).

For each run, from Isaac's contact report (runs/<tag>.contacts.csv: contact point + the impulse on the valve over one
physics step, world frame, the scene's clock) and the recorded vehicle state (npz):
  F      the force ON THE VEHICLE = -(impulses summed per step) / dt, split along the nose / sideways (left) / up,
         the nose from the odometry yaw
  Mz     its moment about the vertical axis through the law's CoM x_c (wb_control_debug [48..50]):
         Mz = (p - x_c)_x F_y - (p - x_c)_y F_x, summed over the step's contact points
  tau_z  the law's yaw torque command (wb_control_debug [23]) over the same window, for scale
Windows: HOLD = ready:end -> push:start (the nose in front of the face), TURN = push:start -> push:end (or the
abort). Reported over the steps in contact: |F_side| p95 / max, F_side / F_push p95 (an effective friction ratio),
|Mz| p95 / max, and the time with |Mz| > 0.05 N.m (8 % of the ~0.65 N.m yaw budget).
Run: PYTHONNOUSERSITE=1 /usr/bin/python3 contact_yaw.py"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import sim_page as SP  # noqa: E402

DT = 0.004
DBG = 2
LOG_DT = 0.02     # the scene logs the contacts every 5th physics step
MZ_BIG = 0.05     # [N.m] a 'large' contact yaw moment: 8 % of the ~0.65 N.m yaw budget


def load_contacts(path):
    C = []
    for line in open(path):
        v = line.rstrip().split(",")
        if v[0] == "C":
            C.append((float(v[1]), v[2], [float(x) for x in v[4:7]], [float(x) for x in v[10:13]]))
    return C


def analyse(f):
    z = np.load(f, allow_pickle=True)
    mt = dict(zip(z["marks_name"], z["marks_t"]))
    path = f[:-4] + ".contacts.csv"
    if "push:start" not in mt or not os.path.exists(path):
        return None
    C = load_contacts(path)
    if not C:
        return None
    c = z["contact"]
    withpts = c[c[:, 20] > 0, 0]
    off = (min(t for t, *_ in C) - float(withpts.min())) if len(withpts) else 0.0
    d = z["dbg"]
    zc = d[:, 50 + DBG].astype(float)
    d = d[np.isfinite(zc) & (zc > 0.3)]          # DIRECT rows only (SAFETY rows are shorter and zero-padded)
    td = d[:, 0].astype(float)
    xc = d[:, 48 + DBG:51 + DBG].astype(float)
    tz = d[:, 23 + DBG].astype(float)
    od = z["odom"]
    q = od[:, 7:11]
    yaw = np.unwrap(np.arctan2(2 * (q[:, 0] * q[:, 3] + q[:, 1] * q[:, 2]), 1 - 2 * (q[:, 2] ** 2 + q[:, 3] ** 2)))
    t_ab = z["box"][-1, 0]
    out = {}
    wins = {"HOLD": (mt.get("ready:end"), mt["push:start"]),
            "TURN": (mt["push:start"], mt.get("push:end", t_ab))}
    # group the contacts by physics step
    steps = {}
    for t, link, p, imp in C:
        tn = t - off
        key = round(t, 4)
        steps.setdefault(key, []).append((tn, link, np.array(p), -np.array(imp) / DT))
    for name, (t0, t1) in wins.items():
        if t0 is None or t1 is None:
            continue
        Fn, Fs, Fz, Mz, links = [], [], [], [], {}
        for key, rows in steps.items():
            tn = rows[0][0]
            if tn < t0 or tn > t1:
                continue
            ps = np.interp(tn, od[:, 0], yaw)
            nose = np.array([np.cos(ps), np.sin(ps), 0.0])
            left = np.array([-np.sin(ps), np.cos(ps), 0.0])
            cx = np.array([np.interp(tn, td, xc[:, i]) for i in range(3)])
            F = np.zeros(3)
            m = 0.0
            for _, link, p, Fv in rows:
                F += Fv
                r = p - cx
                m += r[0] * Fv[1] - r[1] * Fv[0]
                links[link] = links.get(link, 0) + 1
            if np.linalg.norm(F) < 1e-3:
                continue
            Fn.append(F @ nose); Fs.append(F @ left); Fz.append(F[2]); Mz.append(m)
        if not Fn:
            out[name] = None
            continue
        Fn, Fs, Fz, Mz = map(np.array, (Fn, Fs, Fz, Mz))
        push = np.maximum(-Fn, 1e-6)          # the plate pushes the vehicle backward (-nose)
        w = (td >= t0) & (td <= t1)
        out[name] = dict(n=len(Fn), push_mean=float(push.mean()), push_p95=float(np.percentile(push, 95)),
                         side_p95=float(np.percentile(np.abs(Fs), 95)), side_max=float(np.abs(Fs).max()),
                         ratio_p95=float(np.percentile(np.abs(Fs) / push, 95)),
                         up_p95=float(np.percentile(np.abs(Fz), 95)),
                         mz_p95=float(np.percentile(np.abs(Mz), 95)), mz_max=float(np.abs(Mz).max()),
                         mz_over_s=float((np.abs(Mz) > MZ_BIG).sum() * LOG_DT),
                         tz_p95=float(np.percentile(np.abs(tz[w]), 95)) if w.any() else float("nan"),
                         links=links)
    return out


def main():
    print(f"{'run':24s} {'setup':52s} {'win':4s} {'n':>4s} {'push':>10s} {'side p95/max':>13s} {'side/push':>9s} "
          f"{'Mz p95/max [N.m]':>17s} {'>0.05':>6s} {'tau_z p95':>9s}")
    for tag, where, claw, cond in SP.RUNS:
        f = os.path.join(where, tag + ".npz")
        if not os.path.exists(f):
            continue
        r = analyse(f)
        if r is None:
            print(f"{tag:24s} {claw[:26] + ' | ' + cond[:23]:52s} (no turn / no contact log)")
            continue
        for name in ("HOLD", "TURN"):
            s = r.get(name)
            if not s:
                print(f"{tag:24s} {claw[:26] + ' | ' + cond[:23]:52s} {name:4s}    - no contact")
                continue
            print(f"{tag:24s} {claw[:26] + ' | ' + cond[:23]:52s} {name:4s} {s['n']:4d} {s['push_mean']:4.2f}/{s['push_p95']:4.2f} N "
                  f"{s['side_p95']:5.3f}/{s['side_max']:5.3f} {s['ratio_p95']:8.2f} {s['mz_p95']:7.4f}/{s['mz_max']:7.4f} "
                  f"{s['mz_over_s']:5.2f}s "
                  f"{s['tz_p95']:8.3f}  {sorted(s['links'])}")


if __name__ == "__main__":
    main()
