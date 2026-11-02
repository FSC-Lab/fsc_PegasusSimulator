#!/usr/bin/env python3
"""(2026-11-01) What the observer's tangential error (raw F_hat minus the true force on the claw, along the
turn) follows in the Isaac valve turns: the arm joints' motion (friction the frozen joint-row trim w_q_hat
cannot follow in contact), the joints' acceleration, the CoM's acceleration along the turn, the valve's
angular acceleration. Least squares over the turn; the R^2 of each group alone and of all together, and the
time shift that best lines the raw reading up with the truth.

    PYTHONNOUSERSITE=1 /usr/bin/python3 obs_error.py <runs dir> tag [tag ...]
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import obs_sign as OS  # noqa: E402

D = OS.D


def lp(x, dt, tau):
    a = 1 - np.exp(-dt / tau); y = np.zeros_like(x); y[0] = x[0]
    for k in range(1, len(x)):
        y[k] = y[k - 1] + a * (x[k] - y[k - 1])
    return y


def r2(X, y):
    X = np.column_stack([np.ones(len(y)), X])
    b = np.linalg.lstsq(X, y, rcond=None)[0]
    return 1 - np.var(y - X @ b) / np.var(y), b


if __name__ == "__main__":
    d = sys.argv[1]
    print(f"{'run':12s} | R^2 of (raw - truth) on: joint-friction  joint-accel  CoM-accel  valve-accel  ALL | "
          f"slip-window mean error: total  explained-by-ALL | best lag obs vs truth")
    for tag in sys.argv[2:]:
        path = os.path.join(d, tag + ".npz")
        r = OS.run(path)
        z = np.load(path, allow_pickle=True)
        g, dt = r["g"], 0.01
        dbg = z["dbg"]; m = np.isfinite(dbg[:, 97 + D])
        q = OS.interp_rows(g, dbg[m, 0], dbg[m, 5 + D:9 + D])
        qd = lp(np.gradient(q, dt, axis=0), dt, 0.03)
        qdd = lp(np.gradient(qd, dt, axis=0), dt, 0.03)
        fr = np.tanh(qd / 0.015)                                   # the plant's friction shape, w 0.015 rad/s
        xc = OS.interp_rows(g, dbg[m, 0], dbg[m, 48 + D:51 + D])
        tdir = None
        # the turn direction, again (obs_sign keeps it local): from truth/obs projection is enough -> recompute
        box = z["box"]; B = OS.interp_rows(g, box[:, 0], box[:, 1:8])
        yaw = np.unwrap(np.arctan2(2 * (B[:, 3] * B[:, 6] + B[:, 4] * B[:, 5]), 1 - 2 * (B[:, 5] ** 2 + B[:, 6] ** 2)))
        turn = np.sign(yaw[-1] - yaw[0]) or 1.0
        rr = np.array([OS.q2R(B[i, 3:7]) @ (OS.G - np.array([0, 0, OS.G[2]])) for i in range(len(g))])
        tdir = turn * np.cross([0.0, 0.0, 1.0], rr); tdir /= np.linalg.norm(tdir, axis=1)[:, None]
        ac = lp(np.gradient(lp(np.gradient(xc, dt, axis=0), dt, 0.03), dt, axis=0), dt, 0.03)
        act = np.sum(ac * tdir, 1)
        wdd = lp(np.gradient(np.radians(r["wd"]), dt), dt, 0.03)
        e = lp(r["obs"] - r["truth"], dt, 0.1)                     # 1.6 Hz: the 250 Hz deadbeat noise out
        groups = dict(fr=fr, qdd=qdd, ac=act[:, None], wdd=wdd[:, None])
        R = {k: r2(v, e)[0] for k, v in groups.items()}
        Xall = np.column_stack(list(groups.values()))
        Ra, b = r2(Xall, e)
        fit = np.column_stack([np.ones(len(e)), Xall]) @ b
        s = r["slip"]
        lags = np.arange(-30, 31)
        cc = [np.corrcoef(np.roll(r["obs"], -k)[40:-40], r["truth"][40:-40])[0, 1] for k in lags]
        kb = lags[int(np.argmax(cc))]
        part = {}
        for k, v in groups.items():
            _, bk = r2(v, e); part[k] = ((np.column_stack([np.ones(len(e)), v]) @ bk - bk[0])[s]).mean()
        tr = lp(r["truth"], dt, 0.1)[:, None]
        Rt, bt = r2(tr, e); Rtf, _ = r2(np.column_stack([tr, fr, qdd]), e)
        print(f"              truth alone R^2 {Rt:.2f} (slope {bt[1]:+.2f}: obs = {1 + bt[1]:.2f} x truth)   truth + joint motion R^2 {Rtf:.2f}")
        print("              slip bias explained alone: " + "  ".join(f"{k} {v:+.2f}" for k, v in part.items()))
        print(f"{tag:12s} |   {R['fr']:.2f}        {R['qdd']:.2f}        {R['ac']:.2f}       {R['wdd']:.2f}      {Ra:.2f} | "
              f"{e[s].mean():+6.2f} N   {(fit[s] - b[0]).mean():+6.2f} N | {kb * 10:+4d} ms (corr {max(cc):.2f})")
