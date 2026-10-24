#!/usr/bin/env python3
"""The q3 fold that tripped the 10-23 rate guard against every Isaac fold/buckle of the
2026-10-21/22 contact-stability campaign and the 10-21 hardware push: each trace aligned on its
first downward crossing of q3 = 20 deg inside the engaged window (claw on the handle), q3 rate
filtered exactly as the planner's guard does (finite difference, 0.02 s first-order low-pass).
Prints q2 / q3 / q3-rate every 0.04 s from -0.3 s to +1.0 s, how the episode ended and when
each guard variant would have fired.
Usage: /usr/bin/python3 fold_compare.py"""
import os
import re
import numpy as np
from bp_common import Bag

HERE = os.path.dirname(os.path.abspath(__file__))
SIM = os.path.join(HERE, "..", "..", "box_pushing_contact_stability_tune_20261021", "runs")


def lpf_rate(t, q, tc=0.02):
    v = np.zeros_like(q)
    for i in range(1, len(q)):
        d = t[i] - t[i - 1]
        if 0 < d < 0.1:
            v[i] = v[i - 1] + (1 - np.exp(-d / tc)) * ((q[i] - q[i - 1]) / d - v[i - 1])
        else:
            v[i] = v[i - 1] if d <= 0 else 0.0
    return v


def ev_time(z, pats, after=0.0):
    for e in z["events"]:
        s = str(e)
        m = re.match(r"\[\s*([0-9.]+)s\]", s)
        if m and float(m.group(1)) > after and any(p in s for p in pats):
            return float(m.group(1)), s
    return None, ""


def sim(tag):
    z = np.load(os.path.join(SIM, tag + ".npz"), allow_pickle=True)
    mk = {str(n): float(x) for n, x in zip(z["marks_name"], z["marks_t"])}
    J = z["joints"]
    t, q2, q3 = J[:, 0], np.degrees(J[:, 2]), np.degrees(J[:, 3])
    t0 = mk.get("ready:end")
    t_end, why = ev_time(z, ["ABORTING (SAFETY GUARD", "push_pull: NOT IN DIRECT"], after=t0)
    if t_end is None:
        t_end = mk.get("exit:start", t[-1])
        why = "completed (exit)"
    why = "LEFT DIRECT" if "NOT IN DIRECT" in why else ("rate/static guard" if "GUARD" in why else why)
    return dict(name="SIM " + tag, t=t, q2=q2, q3=q3, t0=t0, t_end=t_end, why=why)


def hw(tag, t0, t_end, why):
    b = Bag(tag)
    P = np.degrees(b.f("js", "position"))
    th = b.f("js", "hdr") if "js__hdr" in b.z.files else b.t("js")
    t = b.t("js")
    # use header-stamp spacing on the receive-time axis
    tt = t[0] + (th - th[0])
    return dict(name=f"HW {tag}", t=tt, q2=P[:, 0], q3=P[:, 1], t0=t0, t_end=t_end, why=why)


RUNS = [hw("172853", 45.00, 48.836, "RATE GUARD (10-23)"),
        hw("170123", 36.50, 46.62, "static guard (10-21)")]
for tag in ["z_ship_6", "z_ship_7", "h_ship_5", "g_cal0_1", "cal45_2", "v_ship_6", "v_ship_7", "j_fwx15_2", "j_fwx15_3"]:
    RUNS.append(sim(tag))


def first_cross(t, q, t0, t1, lev=20.0):
    m = np.flatnonzero((t >= t0) & (t <= t1))
    for i in m[1:]:
        if q[i - 1] >= lev > q[i]:
            return i
    return None


for r in RUNS:
    v = lpf_rate(r["t"], r["q3"])
    r["v"] = v
    i = first_cross(r["t"], r["q3"], r["t0"], r["t_end"] + 1.0)
    print(f"\n=== {r['name']}: engaged {r['t0']:.2f} s, ended {r['t_end']:.2f} s by {r['why']}")
    if i is None:
        w = (r["t"] >= r["t0"]) & (r["t"] <= r["t_end"])
        print(f"    no q3 = 20 crossing; engaged q3 min {r['q3'][w].min():.1f}")
        continue
    tc = r["t"][i]
    print(f"    q3 crosses 20 deg at {tc:.2f} s ({tc - r['t_end']:+.2f} s vs the end)")
    w = (r["t"] >= tc) & (r["t"] <= tc + 1.0)
    print(f"    next 1.0 s: q3 min {r['q3'][w].min():6.1f}, fastest fold {v[w].min():6.0f} deg/s, q2 max {r['q2'][w].max():5.1f}")
    for dt in np.arange(-0.28, 1.01, 0.04):
        k = np.searchsorted(r["t"], tc + dt)
        if k >= len(r["t"]):
            break
        mark = " <- end" if abs(r["t"][k] - r["t_end"]) < 0.021 else ""
        print(f"    {dt:+5.2f}  q2 {r['q2'][k]:6.1f}  q3 {r['q3'][k]:6.1f}  v3 {v[k]:6.0f}{mark}")
