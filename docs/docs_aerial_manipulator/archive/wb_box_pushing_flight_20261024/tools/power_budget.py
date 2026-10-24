#!/usr/bin/env python3
"""Battery, rotor commands and thrust headroom of every box push (10-21, 10-23, the three 10-24
flights), per phase. Rotor command = /fmu/in/actuator_motors control[0..3] (the law's output,
0..1); 'sat' = the share of samples with any rotor >= 0.98; thrust = the law's collective u1
(wb_control_debug [0]). The allocator's own saturation lines are counted from /rosout.
Usage: /usr/bin/python3 power_budget.py"""
import numpy as np
from bp_common import Bag

PHASES = ("hover", "go_to_start", "ready", "hold", "push", "exit")


def marks(b):
    """the planner's push-and-pull status edges -> phase windows"""
    t, s = b.t("pl_pstatus"), [str(x) for x in b.f("pl_pstatus", "data")]
    ev = {}
    for ti, si in zip(t, s):
        for key, name in (("FLYING go_to_start", "go_to_start"), ("FLYING ready", "ready"),
                          ("DONE ready", "hold"), ("FLYING push", "push"), ("ABORTING", "abort"),
                          ("FLYING exit", "exit"), ("DONE push", "pushed")):
            if si.startswith(key) and name not in ev:
                ev[name] = ti
    return ev


def win(t, a, b):
    return (t >= a) & (t < b)


for tag, day in (("170123", "10-21"), ("172853", "10-23"), ("150423", "10-24 #1"), ("162850", "10-24 A"), ("163413", "10-24 B"), ("164419", "10-24 C"),
                 ("151159", "10-24 #2"), ("151634", "10-24 #3")):
    try:
        b = Bag(tag)
    except Exception as e:  # noqa: BLE001
        print(tag, "not loadable:", e)
        continue
    ev = marks(b)
    tb, V = b.t("batt"), b.f("batt", "voltage_v").astype(float)
    tm = b.t("motors")
    M = np.vstack(b.f("motors", "control")).astype(float)[:, :4]
    tw = b.t("wb")
    W = np.vstack(b.f("wb", "data")).astype(float)
    u1 = W[:, 0]
    nsat = 0
    if b.has("rosout"):
        nsat = sum("allocator saturated" in str(m) for m in b.f("rosout", "msg"))
    t_gs = ev.get("go_to_start", np.nan)
    w = [("hover", t_gs - 8.0, t_gs), ("go_to_start", t_gs, ev.get("ready", t_gs)),
         ("ready", ev.get("ready", np.nan), ev.get("hold", np.nan)),
         ("hold", ev.get("hold", np.nan), ev.get("push", ev.get("abort", np.nan))),
         ("push", ev.get("push", np.nan), ev.get("abort", ev.get("pushed", np.nan)))]
    print(f"\n{day} ({tag}): battery {V[V > 5].max():.2f} -> {V[V > 5].min():.2f} V; "
          f"allocator-saturation log lines: {nsat}")
    print(f"  {'phase':12s} {'V':>6s} {'rotor mean':>10s} {'rotor max':>9s} {'sat %':>6s} {'u1 mean':>8s} {'u1 max':>7s}")
    for name, a, c in w:
        if not (np.isfinite(a) and np.isfinite(c) and c > a):
            continue
        mv, mm, mw = win(tb, a, c), win(tm, a, c), win(tw, a, c)
        if not mm.any():
            continue
        R = M[mm]
        print(f"  {name:12s} {np.nanmean(V[mv]) if mv.any() else np.nan:6.2f} {np.nanmean(R):10.3f} "
              f"{np.nanmax(R):9.3f} {100 * np.mean(np.nanmax(R, axis=1) >= 0.98):6.1f} "
              f"{np.mean(u1[mw]):8.2f} {np.max(u1[mw]):7.2f}")
