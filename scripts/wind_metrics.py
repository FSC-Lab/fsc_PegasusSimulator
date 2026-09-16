#!/usr/bin/env python3
"""Scores a wind_sweep.sh directory (piece 4, 2026-09-15): per cell, over its seeds --
  landed      the RTH stop was reached and the aircraft never touched the ground
  fails       outer solve failures, summed over the seeds
  pk          worst corridor peak pitch [deg] (t < 15 s)
  Vair_min    the PLANT's lowest airspeed while wing-borne (from the trace's Vair= field, t > 15 s
              until the brake), against the stall (15.12 m/s at 0.502 kg)
  thr_max     the front pair's highest throttle while wing-borne (the tailwind headroom)
  orbit       rho - R in the orbit, mean +- sd over its last 40 s (the trace's ct= field in loiter)
  roll        worst |roll| outside the turns is not separable here: worst |roll| overall
  stop        RTH stop distance from home [m]
PASS (the plan's): every seed landed, fails <= 4 per seed, pk <= still-air pk + 1 deg, Vair_min >=
stall + 0.5 (hover excluded), orbit |mean| <= 1 m (still air) -- reported, not enforced, in wind,
stop <= 10 m.  Usage: wind_metrics.py <dir>"""
import glob
import os
import re
import sys

import numpy as np

V_STALL = 15.12
LINE = re.compile(r"^\s+([\d.]+)\s+([\d.-]+)\s+([\d.]+)\s+([\d.]+)\s+([\d.-]+)\s+([\d.-]+)\s+([+\-\d.]+)\s+([+\-\d.]+)\s+([+\-\d.]+)\s+([+\-\d.]+)\s+([+\-\d.]+)\s+([\d.]+)\s+([\d.]+) ct=\s*([+\-\d.]+)m ok=(\d).*?mot=\[([\d.,]+)\](.*)$")


def parse(path):
    s = open(path, errors="replace").read()
    o = {"ground": "GROUND at" in s}
    m = re.search(r"solve fails (\d+)/(\d+)", s); o["fails"] = int(m.group(1)) if m else None
    m = re.search(r"worst \|roll\| ([\d.]+) deg", s); o["worst_roll"] = float(m.group(1)) if m else None
    m = re.search(r"RTH STOP t=([\d.]+): ([\d.]+) m from home", s); o["stop_m"] = float(m.group(2)) if m else None
    m = re.search(r"GUIDE t=([\d.]+): legs done: loitering", s); t_orbit = float(m.group(1)) if m else None
    m = re.search(r"GUIDE t=([\d.]+): 'rth'", s); t_rth = float(m.group(1)) if m else None
    m = re.search(r"GUIDE t=([\d.]+): rth: braking", s); t_brake = float(m.group(1)) if m else None
    m = re.search(r"estimate w=\(([+\-\d.]+),([+\-\d.]+),", s); o["w_est"] = (float(m.group(1)), float(m.group(2))) if m else None
    pk = 0.0; vair = []; thr = []; orbit = []
    for ln in s.split("\n"):
        mm = LINE.match(ln)
        if not mm:
            continue
        t = float(mm.group(1)); th = float(mm.group(7)); ct = float(mm.group(14)); mot = [float(v) for v in mm.group(16).split(",")]
        rest = mm.group(17)
        if t < 15.0:
            pk = max(pk, th)
        wingborne = t > 15.0 and (t_brake is None or t < t_brake)
        if wingborne:
            mv = re.search(r"Vair=([\d.]+)", rest)
            vair.append(float(mv.group(1)) if mv else float(mm.group(3)))
            thr.append(max(mot[0], mot[1]))
        if t_orbit is not None and t >= t_orbit and (t_rth is None or t < t_rth):
            orbit.append((t, ct))
    o["pk"] = pk; o["vair_min"] = min(vair) if vair else None; o["thr_max"] = max(thr) if thr else None
    if orbit:
        t_end = orbit[-1][0]; last = [c for t, c in orbit if t >= t_end - 40.0]
        o["orbit_mean"] = float(np.mean(last)); o["orbit_sd"] = float(np.std(last))
    else:
        o["orbit_mean"] = o["orbit_sd"] = None
    return o


def main():
    d = sys.argv[1]
    cells = {}
    for f in sorted(glob.glob(os.path.join(d, "*_s*.txt"))):
        name = os.path.basename(f)[:-4]
        cell, _, sd = name.rpartition("_s")
        cells.setdefault(cell, []).append(parse(f))
    still = cells.get("still")
    pk0 = max(r["pk"] for r in still) if still else None
    print(f"wind sweep {d}" + (f"  (still air: pk {pk0:.1f})" if pk0 is not None else ""))
    hdr = f"{'cell':13s} {'landed':7s} {'fails':5s} {'pk':5s} {'Vair_min':8s} {'thr_max':7s} {'orbit rho-R':14s} {'roll':5s} {'stop':6s} {'w_est':14s} verdict"
    print(hdr); print("-" * len(hdr))
    for cell, rs in cells.items():
        n = len(rs); landed = sum(1 for r in rs if r["stop_m"] is not None and not r["ground"])
        fails = sum(r["fails"] or 0 for r in rs); pk = max(r["pk"] for r in rs)
        vmin = min((r["vair_min"] for r in rs if r["vair_min"] is not None), default=float("nan"))
        thr = max((r["thr_max"] for r in rs if r["thr_max"] is not None), default=float("nan"))
        om = [r["orbit_mean"] for r in rs if r["orbit_mean"] is not None]; osd = [r["orbit_sd"] for r in rs if r["orbit_sd"] is not None]
        roll = max((r["worst_roll"] or 0.0) for r in rs)
        stops = [r["stop_m"] for r in rs if r["stop_m"] is not None]
        w = [r["w_est"] for r in rs if r["w_est"] is not None]
        why = []
        if landed < n: why.append(f"{landed}/{n} landed")
        if fails > 4 * n: why.append(f"fails {fails}")      # <= 4 per seed over a 260 s mission (5200 solves)
        if pk0 is not None and pk > pk0 + 1.0: why.append(f"pk {pk:.1f}")
        if vmin < V_STALL + 0.5: why.append(f"Vair {vmin:.1f}")
        if stops and max(stops) > 10.0: why.append(f"stop {max(stops):.0f} m")
        print(f"{cell:13s} {landed:2d}/{n:<4d} {fails:5d} {pk:5.1f} {vmin:8.2f} {thr:7.2f} "
              f"{(f'{np.mean(om):+6.1f} +- {np.mean(osd):4.1f}' if om else '   --   '):14s} {roll:5.1f} "
              f"{(f'{np.mean(stops):5.1f}' if stops else '  --'):6s} "
              f"{(f'({np.mean([a for a, _ in w]):+.1f},{np.mean([b for _, b in w]):+.1f})' if w else '--'):14s} "
              f"{'PASS' if not why else 'FAIL'} {'; '.join(why)}")


if __name__ == "__main__":
    main()
