#!/usr/bin/env python3
"""The tolerance table from a scripts/robust_sweep.sh directory: one row per cell (a plant constant
at one level), the pass verdict against the nominal cell, and per parameter the widest level that
still passes -- the identification spec.
    ~/envs/isaac_env/bin/python3 scripts/robust_metrics.py ~/flight_bags/robust/<tag> [--csv OUT]
PASS (relative to the flown line's own numbers: 16/16, crossing peak 14.1-14.3, fails-sum 1-3,
z 29.9-30.0, stop 8.8 s): every seed lands (final z > 20), fails-sum <= 4, worst crossing peak
<= nominal + 3 deg, min z >= 29.0 m, t_stop max <= nominal + 3 s.
"""
import csv
import os
import re
import sys

d = sys.argv[1]
PAT = re.compile(r"SUMMARY (\S+): (\d+)/(\d+) \| worst pkpitch\s+([\d.]+) pkAoA\s+([\d.]+) \| err rms mean\s+([\d.]+) worst pk\s+([\d.]+) \| revs mean\s+([\d.]+) max (\d+) \| fails sum (\d+) elvsat sum (\d+) \| minz\s+([\d.]+) maxz\s+([\d.]+) \| t_stop max\s+([\d.]+) \| roll worst\s+([\d.]+)")
rows = {}


def seeds_before_bt(name):
    """Seeds ke_metrics could not score because the run never reached the back-transition
    ('sN: no BT rows' in the cell's summary.txt) -- a crash in the corridor or cruise, which a
    summary counts as NOT landed; a cell where every seed did so has no SUMMARY line at all and
    used to print MISSING, hiding the worst cells of the sweep (2026-09-15)."""
    try:
        return sum(1 for ln in open(os.path.join(d, name, "summary.txt")) if "no BT rows" in ln)
    except OSError:
        return 0


for line in open(os.path.join(d, "cells.txt")):
    name, flags, summ = line.rstrip("\n").split("|", 2)
    m = PAT.search(summ)
    early = seeds_before_bt(name)
    if not m:
        if early:
            rows[name] = dict(flags=flags, ok=0, n=early, pk=float("nan"), aoa=float("nan"), err=float("nan"), revs=float("nan"),
                              fails=0, elvsat=0, minz=0.0, maxz=0.0, tstop=float("nan"), roll=float("nan"), missing=False, early=early)
        else:
            rows[name] = dict(flags=flags, missing=True)
        continue
    g = m.groups()
    rows[name] = dict(flags=flags, ok=int(g[1]), n=int(g[2]) + early, pk=float(g[3]), aoa=float(g[4]), err=float(g[5]), revs=float(g[7]),
                      fails=int(g[9]), elvsat=int(g[10]), minz=float(g[11]), maxz=float(g[12]), tstop=float(g[13]), roll=float(g[14]), missing=False, early=early)
nom = rows.get("nominal")
if nom is None or nom["missing"]:
    print("no nominal cell -- cannot judge"); sys.exit(1)


def verdict(r):
    if r["missing"]:
        return "MISSING", ""
    why = []
    if r["ok"] < r["n"]: why.append(f"{r['ok']}/{r['n']} landed" + (f" ({r['early']} crashed before the BT)" if r.get("early") else ""))
    if r["fails"] > 4: why.append(f"fails {r['fails']}")
    if r["pk"] > nom["pk"] + 3.0: why.append(f"pk {r['pk']:.1f}")
    if r["minz"] < 29.0: why.append(f"minz {r['minz']:.1f}")
    if r["tstop"] > nom["tstop"] + 3.0: why.append(f"stop {r['tstop']:.1f}s")
    return ("PASS" if not why else "FAIL"), "; ".join(why)


print(f"robust sweep {d}  (nominal: {nom['ok']}/{nom['n']} pk {nom['pk']:.1f} fails {nom['fails']} minz {nom['minz']:.1f} stop {nom['tstop']:.1f})")
hdr = f"{'cell':13s} {'landed':7s} {'pk':5s} {'fails':5s} {'revs':4s} {'minz':5s} {'stop':5s} {'roll':5s} {'verdict':7s} why"
print(hdr); print("-" * len(hdr))
table = []
for name, r in rows.items():
    v, why = verdict(r)
    if r["missing"]:
        print(f"{name:13s} MISSING"); table.append((name, r["flags"], "MISSING", why)); continue
    print(f"{name:13s} {r['ok']:2d}/{r['n']:<4d} {r['pk']:5.1f} {r['fails']:5d} {r['revs']:4.1f} {r['minz']:5.1f} {r['tstop']:5.1f} {r['roll']:5.1f} {v:7s} {why}")
    table.append((name, r["flags"], v, why))
# the envelope: per parameter, the passing levels
print("\nENVELOPE (levels that pass, per parameter):")
groups = {}
for name, r in rows.items():
    if name == "nominal":
        continue
    p, _, lvl = name.rpartition("_")
    groups.setdefault(p, []).append((float(lvl) if re.match(r"^-?[\d.]+$", lvl) else lvl, verdict(r)[0]))
for p, lv in groups.items():
    lv.sort(key=lambda x: (isinstance(x[0], str), x[0]))
    print(f"  {p:8s} " + "  ".join(f"{l}:{'ok' if v == 'PASS' else 'X'}" for l, v in lv))
if "--csv" in sys.argv:
    out = sys.argv[sys.argv.index("--csv") + 1]
    with open(out, "w", newline="") as f:
        w = csv.writer(f); w.writerow(["cell", "flags", "verdict", "why"]); w.writerows(table)
    print("->", out)
