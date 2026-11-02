#!/usr/bin/env python3
"""(2026-11-01) obs_sign.py's observer numbers on the HARDWARE valve turns, same definitions:
the force ON THE CLAW along the turn direction (the observer's raw F_hat [97..99] and what the law used
[58..60]), over the turn step and over SLIPS (the valve faster than SLIP_DEG_S along the turn). No truth on
hardware: the turn direction comes from the claw reference itself (the tangent of the circle fitted to the
streamed r_ed over the step, oriented the way r_ed moves), the valve's rate from obj_0's yaw (its rotor-fixed
axes, wherever Motive put them, turn with the rotor).

    PYTHONNOUSERSITE=1 /usr/bin/python3 obs_sign_hw.py
"""
import importlib.util
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ARCH = os.path.join(HERE, "..", "..")
SLIP_DEG_S = float(os.environ.get("SLIP_DEG_S", "15"))
FLIGHTS = [("wb_valve_turning_flight_20261028", "180652"), ("wb_valve_turning_flight_20261028", "181833"),
           ("wb_valve_turning_flight_20261029", "145526")]


def load(folder):
    tools = os.path.join(ARCH, folder, "tools")
    sys.path.insert(0, tools)
    spec = importlib.util.spec_from_file_location("vt_" + folder, os.path.join(tools, "vt_common.py"))
    m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
    sys.path.remove(tools)
    return m


def stats(x, m):
    return (np.mean(x[m]), np.sqrt(np.mean((x[m] - np.mean(x[m])) ** 2))) if m.sum() > 5 else (np.nan, np.nan)


print(f"slip = valve faster than {SLIP_DEG_S:.0f} deg/s along the turn; force ON THE CLAW along the turn, "
      f"mean (sd) N; a valve resisting the turn reads NEGATIVE")
print(f"{'flight':8s} {'turn':>5s} {'T':>5s} | {'ALL: obs':>13s} {'used':>13s} | {'slip%':>5s} {'peak':>5s} | "
      f"{'SLIP: obs':>13s} {'used':>13s}")
for folder, tag in FLIGHTS:
    vt = load(folder)
    b = vt.Bag(tag)
    st = {}
    for t, s in b.edges("pl_pstatus"):
        s = str(s)
        if s.startswith("FLYING push") and "push" not in st: st["push"] = t
        if (s.startswith("ABORTING") or s.startswith("DONE push")) and "end" not in st and "push" in st: st["end"] = t
    t0, t1 = st["push"], st.get("end", b.t("wb")[-1])
    g = np.arange(t0 + 0.3, t1, 0.01)
    tr = b.t("wbref"); R = np.stack([b.f("wbref", f"r_ed.{c}") for c in "xy"], 1)
    mr = (tr >= t0) & (tr <= t1)
    X, Y = R[mr, 0], R[mr, 1]
    A = np.c_[2 * X, 2 * Y, np.ones(mr.sum())]; cx, cy, _ = np.linalg.lstsq(A, X ** 2 + Y ** 2, rcond=None)[0]
    az = np.unwrap(np.arctan2(Y - cy, X - cx)); turn = np.sign(az[-1] - az[0]) or 1.0
    rx, ry = np.interp(g, tr[mr], X) - cx, np.interp(g, tr[mr], Y) - cy
    tdir = turn * np.stack([-ry, rx, np.zeros_like(rx)], 1); tdir /= np.linalg.norm(tdir, axis=1)[:, None]
    to = b.t("obj"); q = [b.f("obj", f"pose.orientation.{c}") for c in "wxyz"]
    yaw = np.unwrap(np.arctan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] ** 2 + q[3] ** 2)))
    wd = np.degrees(turn * (np.interp(g + 0.05, to, yaw) - np.interp(g - 0.05, to, yaw)) / 0.1)
    turned = np.degrees(turn * (np.interp(g[-1], to, yaw) - np.interp(g[0], to, yaw)))
    tw = b.t("wb"); W = np.asarray(b.f("wb", "data"), float); m = np.isfinite(W[:, 99])
    obs = sum(np.interp(g, tw[m], W[m, 97 + i]) * tdir[:, i] for i in range(3))
    used = sum(np.interp(g, tw[m], W[m, 58 + i]) * tdir[:, i] for i in range(3))
    a = np.ones(len(g), bool); s = wd > SLIP_DEG_S
    fmt = lambda v: f"{v[0]:+6.2f} ({v[1]:4.2f})"
    print(f"{tag:8s} {turned:5.0f} {t1 - t0:4.1f}s | {fmt(stats(obs, a)):>13s} {fmt(stats(used, a)):>13s} | "
          f"{100 * s.mean():4.0f}% {wd.max():5.0f} | {fmt(stats(obs, s)):>13s} {fmt(stats(used, s)):>13s}"
          + (lambda mv, sg: f" || ALONG MOTION, |w|>thr ({100 * mv.mean():.0f}%, back {100 * (mv & (sg < 0)).mean():.0f}%): "
             f"obs {np.mean(obs[mv] * sg[mv]):+.2f} ({np.std(obs[mv] * sg[mv]):.2f})  used {np.mean(used[mv] * sg[mv]):+.2f}")(np.abs(wd) > SLIP_DEG_S, np.sign(wd)))
