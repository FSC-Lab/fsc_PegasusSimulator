#!/usr/bin/env python3
"""Identify the DROP OFFSET of a rig from pick-and-drop runs (2026-10-15, the decoupled rig).

    /usr/bin/python3 drop_offset_id.py [--press 2,5] runs/geo_drop1.npz runs/geo_off1_a.npz ...

The error that matters is where the payload is released relative to the COMMANDED drop point (the
planner's place point x, y: the payload origin's target). The payload rides rigidly in the gripper,
so its position at any moment of the hold over the tray is where it would be released if Drop were
pressed then: each run gives the error against the press time (seconds after Ready To Drop ends),
up to the moment it actually opened. The commanded point is read from runs/<run prefix>.planner.yaml
(make_drop_offset_yaml.py; e.g. geo_off1_a -> geo_off1.planner.yaml), else the yaml default (-1, -1).
It is checked against the recorded claw reference at the hold (place point + Rz(yaw) * EE offset).

Prints, per press time, the mean and spread of the error and the offset that cancels the mean
(world, and in the drop frame: forward / left of the drop heading), then the clearance of the
payload's footprint to the tray's inner walls that each run would have had with that offset.
"""
import argparse
import os
import re

import numpy as np

TRAY_C = np.array([-1.0, -1.0])
HIN = np.array([0.145, 0.1855]) - 0.0025     # tray inner half, turned 90 deg
# the payload's parts LOW enough to meet the 35 mm tray wall when it lands flat (payload frame, from
# Box_Payload_Assembly.usda's colliders, z below the origin's +2.5 mm): the basket (x, y +-55), its arms
# (x +-48, y +-117.5) and the two SIDE marker balls (x +-15, y +-153). The front / back balls (x +-113)
# sit 47.5 mm above the payload's bottom and pass over the wall -- so x has ~87 mm of clearance, y 30.
LOW = np.array([[sx * a, sy * b] for a, b in ((0.055, 0.055), (0.048, 0.1175), (0.015, 0.153))
                for sx in (-1, 1) for sy in (-1, 1)])
EE_OFF_X = -0.04                             # the sim pick-and-drop EE Offset x
DROP_YAW = np.pi                             # the drop heading (-180 deg)


def yaw(q):
    w, x, y, z = q
    return np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def place_point(path):
    tag = os.path.basename(path)[:-4]
    for cand in (tag, re.sub(r"_[a-z]$", "", tag)):
        f = os.path.join(os.path.dirname(path), cand + ".planner.yaml")
        if os.path.exists(f):
            m = re.search(r"pick_place_place_point:\s*\[([^\]]*)\]", open(f).read())
            return np.array([float(v) for v in m.group(1).split(",")[:2]]), os.path.basename(f)
    return TRAY_C.copy(), "yaml default"


def to_drop(v):          # world (x, y) -> (forward, left) of the drop heading
    c, s = np.cos(DROP_YAW), np.sin(DROP_YAW)
    return np.array([c * v[0] + s * v[1], -s * v[0] + c * v[1]])


def clearance(c_rel, ps):  # payload centre relative to the tray centre, its yaw -> mm to the inner walls
    R = np.array([[np.cos(ps), -np.sin(ps)], [np.sin(ps), np.cos(ps)]])
    cor = LOW @ R.T + c_rel
    return (HIN - np.abs(cor).max(axis=0)) * 1e3


def run(path, presses):
    d = np.load(path, allow_pickle=True)
    n, mt = list(d["marks_name"]), d["marks_t"]
    t_arr, t_open = mt[n.index("ready_place:end")], mt[n.index("gripper_open:start")]
    pp, src = place_point(path)
    pay, ref = d["payload"], d["ref"]
    # check: the claw reference at the hold = place point + Rz(180) * (EE_OFF_X, 0)
    k = np.argmin(abs(ref[:, 0] - (t_open - 0.2)))
    want = pp + np.array([-EE_OFF_X, 0.0])
    chk = (ref[k, 1:3] - want) * 1e3
    z0 = np.interp(t_open, pay[:, 0], pay[:, 3])
    kr = np.argmax((pay[:, 0] >= t_open) & (pay[:, 3] < z0 - 0.01))
    out = {"name": os.path.basename(path), "src": src, "pp": pp, "chk": chk, "dwell": t_open - t_arr,
           "rel": pay[kr, 0] - t_arr, "e": {}, "yaw": {}}
    for tp in list(presses) + ["release"]:
        t = pay[kr, 0] if tp == "release" else t_arr + tp
        if tp != "release" and t > t_open + 1e-3:
            continue
        i = np.argmin(abs(pay[:, 0] - t))
        out["e"][tp] = pay[i, 1:3] - pp
        out["yaw"][tp] = yaw(pay[i, 4:8])
    rest = pay[-1, 1:3] - TRAY_C
    out["rest"] = rest
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("runs", nargs="+")
    ap.add_argument("--press", default="1,2,3,4,5")
    a = ap.parse_args()
    presses = [float(v) for v in a.press.split(",")]
    R = [run(p, presses) for p in a.runs]
    print("per run: payload error = release position - commanded drop point, world mm (x, y)")
    for r in R:
        cols = "  ".join(f"{('t+' + format(k, 'g') + 's') if k != 'release' else 'release'} "
                         f"({v[0] * 1e3:+5.1f},{v[1] * 1e3:+5.1f})" for k, v in r["e"].items())
        print(f"  {r['name']:16s} target ({r['pp'][0]:+.4f}, {r['pp'][1]:+.4f}) [{r['src']}], "
              f"claw ref check ({r['chk'][0]:+.1f}, {r['chk'][1]:+.1f}) mm, opened +{r['dwell']:.1f} s")
        print(f"      {cols}")
    print("\npooled, by press time (s after Ready To Drop ends):")
    best = None
    for tp in list(presses) + ["release"]:
        E = np.array([r["e"][tp] for r in R if tp in r["e"]])
        if len(E) < 2:
            continue
        m, s = E.mean(axis=0), E.std(axis=0, ddof=1)
        off = -m
        f = to_drop(off)
        lab = "release" if tp == "release" else f"t+{tp:g} s"
        print(f"  {lab:9s} n={len(E)}  mean ({m[0] * 1e3:+5.1f}, {m[1] * 1e3:+5.1f})  std ({s[0] * 1e3:4.1f}, {s[1] * 1e3:4.1f}) mm"
              f"  -> offset world ({off[0]:+.3f}, {off[1]:+.3f}) m = forward {f[0] * 1e3:+.0f} / left {f[1] * 1e3:+.0f} mm")
        if tp == "release":
            best = off
    if best is not None:
        print("\nwith the pooled release offset, each run's payload footprint clearance to the inner walls"
              " (in-sample; leave-one-out):")
        for r in R:
            loo = -np.mean([q["e"]["release"] for q in R if q is not r], axis=0)
            a1 = clearance(r["e"]["release"] + best, r["yaw"]["release"])
            a2 = clearance(r["e"]["release"] + loo, r["yaw"]["release"])
            got = clearance(r["e"]["release"] + r["pp"] - TRAY_C, r["yaw"]["release"])
            print(f"  {r['name']:16s} as flown x {got[0]:+4.0f} y {got[1]:+4.0f} | pooled x {a1[0]:+4.0f} y {a1[1]:+4.0f}"
                  f" | leave-one-out x {a2[0]:+4.0f} y {a2[1]:+4.0f} mm")


if __name__ == "__main__":
    main()
