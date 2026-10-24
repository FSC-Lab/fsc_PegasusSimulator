#!/usr/bin/env python3
"""Every hardware box push: how far the claw is from its target when Go To Start ends, when Ready starts
and when Ready ends, from push_pull/arrival_error (target - claw): SHORT = how far the claw is behind its
target along the push (+ = has not arrived, - = overshot past it), ACROSS = target - claw along box x.
eCoM = the same for the law's CoM (x_cd - x_c). Then what the arm did in the 1.5 s after
Ready ended (q3 drop, fastest q3 fall) and how the grasp ended.
Usage: /usr/bin/python3 arrival_table.py"""
import numpy as np
from bp_common import Bag, yaw_deg

FLIGHTS = [("170123", "10-21"), ("172853", "10-23"), ("150423", "10-24 #1"), ("151159", "10-24 #2"),
           ("162850", "10-24 A"), ("163413", "10-24 B"), ("164419", "10-24 C")]


def first(edges, prefix):
    for t, s in edges:
        if s.startswith(prefix):
            return t
    return np.nan


print(f"{'flight':9s} | {'GoToStart end':>14s} | {'wait':>4s} | {'Ready start':>12s} | {'Ready end':>12s} | "
      f"{'eCoM end':>9s} | {'q3 drop':>7s} {'fall':>5s} | outcome")
print(f"{'':9s} | {'short  across':>14s} | {'s':>4s} | {'short acr':>12s} | {'short acr':>12s} | {'short':>9s} | "
      f"{'deg':>7s} {'deg/s':>5s} |")
for tag, day in FLIGHTS:
    try:
        b = Bag(tag)
    except Exception as e:  # noqa: BLE001
        print(day, "not loadable", e)
        continue
    st = b.edges("pl_pstatus")
    t_gs = first(st, "DONE go_to_start")
    t_r0 = first(st, "FLYING ready")
    t_r1 = first(st, "DONE ready")
    oy = np.radians(np.median(yaw_deg(*[b.f("obj", f"pose.orientation.{k}") for k in "wxyz"])[b.t("obj") < t_gs]))
    R2 = np.array([[np.cos(oy), np.sin(oy)], [-np.sin(oy), np.cos(oy)]])
    ta, A = b.t("pl_arr"), b.f("pl_arr", "data")

    def err(t, step):
        idx = [i for i in range(len(ta)) if A[i, 0] == step and abs(ta[i] - t) < 0.3]
        if not idx:
            return np.array([np.nan, np.nan])
        i = idx[np.argmin(np.abs(ta[idx] - t))]
        ab = R2 @ A[i, 3:5] * 1e3                          # (across, along) of target - claw
        return np.array([-ab[1], ab[0]])   # (short, across): the push runs along -box y
    def ahead_across(t, step):
        return err(t, step)
    e_gs = ahead_across(t_gs + 0.05, 0)
    e_r0 = ahead_across(t_r0 - 0.15, 0)
    e_r1 = ahead_across(t_r1 + 0.05, 1)
    tw, W = b.t("wb"), b.f("wb", "data")
    k = np.searchsorted(tw, t_r1)
    ecom = R2 @ (W[k, 45:47] - W[k, 48:50]) * 1e3
    tj, Q = b.t("js"), b.f("js", "position")
    m = (tj >= t_r1) & (tj <= t_r1 + 1.5)
    q3 = np.degrees(Q[m, 1])
    q3_0 = np.degrees(np.interp(t_r1, tj, Q[:, 1]))
    tg = np.arange(t_r1, t_r1 + 1.5, 0.02)
    qg = np.degrees(np.interp(tg, tj, Q[:, 1]))
    rate = np.convolve(np.gradient(qg, tg), np.ones(5) / 5, mode="same")[2:-2]
    g = b.f("js", "position")[:, 4]
    closed = [t for t, v in zip(tj, g) if t > t_r1 and v < 0.0]
    t_ab = first([x for x in st if x[0] > t_r1], "ABORTING")
    t_push = first([x for x in st if x[0] > t_r1], "FLYING push")
    out = []
    if closed:
        out.append(f"closed {closed[0]-t_r1:.1f} s after Ready")
    if np.isfinite(t_push):
        out.append(f"pushed")
    if np.isfinite(t_ab):
        out.append(f"abort {t_ab-t_r1:.1f} s after Ready")
    print(f"{day:9s} | {e_gs[0]:+6.0f} {e_gs[1]:+6.0f} | {t_r0-t_gs:4.1f} | {e_r0[0]:+5.0f} {e_r0[1]:+5.0f}  | "
          f"{e_r1[0]:+5.0f} {e_r1[1]:+5.0f}  | {-ecom[1]:+8.0f} | {q3.min()-q3_0:+7.1f} {np.nanmin(rate):+5.0f} | {', '.join(out)}")
