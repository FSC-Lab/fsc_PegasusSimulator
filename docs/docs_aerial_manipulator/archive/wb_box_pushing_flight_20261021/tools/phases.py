#!/usr/bin/env python3
"""The formal push (170123) against the Isaac runs of the same task (240 g, 0.10 m, 21 mm/s,
box_pushing_speed_20261021/runs/v21_*), phase by phase, with ONE set of definitions.

Phases:  hover   = go_to_start ends -> ready starts (jaws open, CoM anchor)
         onhand  = ready ends -> the gripper reports closed (claw WORLD-held, jaws open)
         settle  = push starts -> +1.5 s (jaws closed, CONTACT, reference still)
         slide   = settle end -> push ends (the box should move)
         post    = push ends -> release/abort starts (jaws still closed, contact OFF)
Columns: q2/q3 min..max and std [deg]; CoM error |x_c - x_cd| rms / max [mm]; the law's EE task
error |e_y[0:3]| rms / max [mm]; base horizontal std [mm]; vehicle tilt max [deg]; box travel
along the push in the phase (min..max) [mm]."""
import os
import sys
import numpy as np
from bp_common import Bag, tilt_deg

SIM = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "box_pushing_speed_20261021", "runs")


def hw_run():
    b = Bag("170123")
    js = b.f("js", "position")
    r = {"name": "HW 170123"}
    r["joints"] = np.column_stack([b.t("js"), js[:, 2], js[:, 0], js[:, 1], js[:, 3]])     # t q1 q2 q3 q4
    r["dbg"] = np.column_stack([b.t("wb"), np.zeros(len(b.t("wb"))), b.f("wb", "data")])
    r["box"] = np.column_stack([b.t("obj")] + [b.f("obj", f"pose.position.{c}") for c in "xyz"]
                               + [b.f("obj", f"pose.orientation.{c}") for c in "wxyz"])
    r["base"] = np.column_stack([b.t("mocap")] + [b.f("mocap", f"pose.position.{c}") for c in "xyz"]
                                + [b.f("mocap", f"pose.orientation.{c}") for c in "wxyz"])
    r["marks"] = {"go_to_start:end": 31.32, "ready:start": 33.31, "ready:end": 36.50,
                  "gripper_close:end": 39.55, "push:start": 39.58, "push:end": 45.84,
                  "release:start": 46.62}
    return r


def sim_run(tag):
    z = np.load(os.path.join(SIM, tag + ".npz"), allow_pickle=True)
    r = {"name": "SIM " + tag}
    r["joints"] = z["joints"][:, :5]
    r["dbg"] = z["dbg"].astype(float)
    r["box"] = z["box"][:, :8]
    r["base"] = z["base_truth"][:, :8]
    r["marks"] = {str(n): float(t) for n, t in zip(z["marks_name"], z["marks_t"])}
    r["contact"] = z["contact"]
    return r


def win(a, t0, t1):
    return a[(a[:, 0] >= t0) & (a[:, 0] <= t1)]


def phases(r):
    m = r["marks"]
    return {"hover": (m["go_to_start:end"], m["ready:start"]),
            "onhand": (m["ready:end"], m["gripper_close:end"]),
            "settle": (m["push:start"], m["push:start"] + 1.5),
            "slide": (m["push:start"] + 1.5, m["push:end"]),
            "post": (m["push:end"], m["release:start"])}


def metrics(r, t0, t1):
    out = {}
    j = win(r["joints"], t0, t1)
    q2, q3 = np.degrees(j[:, 2]), np.degrees(j[:, 3])
    out["q2"] = f"{q2.min():5.1f}..{q2.max():5.1f} ({q2.std():4.1f})"
    out["q3"] = f"{q3.min():5.1f}..{q3.max():5.1f} ({q3.std():4.1f})"
    d = win(r["dbg"], t0, t1)
    ex = np.linalg.norm(d[:, 2 + 48:2 + 51] - d[:, 2 + 45:2 + 48], axis=1) * 1e3
    ey = np.linalg.norm(d[:, 2 + 24:2 + 27], axis=1) * 1e3
    out["com"] = f"{np.sqrt(np.mean(ex**2)):5.1f}/{ex.max():5.1f}"
    out["ee"] = f"{np.sqrt(np.mean(ey**2)):5.1f}/{ey.max():5.1f}"
    bs = win(r["base"], t0, t1)
    out["base_std"] = f"{1e3*np.sqrt(bs[:,1].var()+bs[:,2].var()):5.1f}"
    out["tilt"] = f"{tilt_deg(*bs[:, 4:8].T).max():4.1f}"
    bx = win(r["box"], t0 - 0.05, t1)
    s = -(bx[:, 2] - bx[0, 2]) * 1e3           # the push is -y in both
    out["box"] = f"{s.min():6.1f}..{s.max():6.1f}"
    return out


def main():
    runs = [hw_run()] + [sim_run(t) for t in ("v21_1", "v21_2", "v21_3")]
    for ph in ("hover", "onhand", "settle", "slide", "post"):
        print(f"\n--- {ph}")
        print(f"{'run':12s} {'dur':>5s} | {'q2 min..max (std)':>20s} | {'q3 min..max (std)':>20s} | CoM rms/max | EE rms/max | base std | tilt | box travel")
        for r in runs:
            t0, t1 = phases(r)[ph]
            mm = metrics(r, t0, t1)
            print(f"{r['name']:12s} {t1-t0:5.2f} | {mm['q2']:>20s} | {mm['q3']:>20s} | {mm['com']:>11s} | {mm['ee']:>10s} | {mm['base_std']:>8s} | {mm['tilt']:>4s} | {mm['box']}")


if __name__ == "__main__":
    main()
