#!/usr/bin/env python3
"""RATE_Q2CAP sensitivity to the assumed q2 limit, and whether it would have fired shortly
AFTER the recorded q3-guard trips (the abort changes the arm only gradually: the EE anchor
blends to the CoM over 1.0 s, the jaws take ~0.5 s to open).
Usage: /usr/bin/python3 guard_sensitivity.py"""
import numpy as np
import guard_variants as G

rows = list(G.hw_runs()) + list(G.sim_runs())
print("q2 limit assumed | false trips (48 completed) | buckles caught (lead s, q3) | HW 10-23 | HW 10-21")
for q2max in (44.0, 44.5, 45.0, 45.5, 46.0, 47.0):
    G.Q2MAX = q2max
    res = {}
    for r in rows:
        t, q2, q3 = r["q"]
        t0 = r["t_end"] - 1e9
        pred, lim, v3 = G.rules(t, q2, q3)["RATE_Q2CAP"]
        res[r["name"]] = (r, pred, v3)
    def trip(name, t0, t1):
        r, pred, v3 = res[name]
        return G.rate_trip(r["q"][0], pred, v3, t0, t1, 10.0)
    ok = [r for r in rows if r["outcome"] == "completed"]
    # engaged window start: re-derive from the stored first sample >= engaged start
    ft = 0
    for r in ok:
        t = r["q"][0]
        if r["RATE_Q2CAP"] is not None or trip(r["name"], r["t_end"] - 200, r["t_end"]) is not None:
            pass
    # recompute with the true engaged windows
    import guard_variants as GV
    out = []
    for r in GV.hw_runs():
        out.append(r)
    for r in GV.sim_runs():
        out.append(r)
    fals = [r["name"] for r in out if r["outcome"] == "completed" and r["RATE_Q2CAP"] is not None]
    bk = [(r["name"][4:], round(r["t_end"] - r["RATE_Q2CAP"], 2),
           round(float(np.interp(r["RATE_Q2CAP"], r["q"][0], r["q"][2])), 1))
          for r in out if r["outcome"] == "buckle" and r["RATE_Q2CAP"] is not None and r["RATE_Q2CAP"] < r["t_end"]]
    hw = {r["name"]: r for r in out if r["name"].startswith("HW")}
    def hws(n):
        r = hw[n]
        v = r["RATE_Q2CAP"]
        return "-" if v is None else f"{v - r['t_end']:+.2f}s"
    print(f"  {q2max:5.1f}          | {len(fals):2d}                         | {bk} | {hws('HW 172853')} | {hws('HW 170123')}")

G.Q2MAX = 45.0
print("\nAfter the recorded q3-guard trips (window extended 0.5 s past the trip; the abort is acting):")
for r in G.hw_runs():
    pass
for r in list(G.hw_runs()) + [x for x in G.sim_runs() if x["name"] in ("SIM v_ship_6", "SIM v_ship_7")]:
    t, q2, q3 = r["q"]
    pred, lim, v3 = G.rules(t, q2, q3)["RATE_Q2CAP"]
    tt = G.rate_trip(t, pred, v3, r["t_end"] - 1.0, r["t_end"] + 0.5, 10.0)
    w = (t >= r["t_end"]) & (t <= r["t_end"] + 0.5)
    k = np.searchsorted(t, r["t_end"])
    print(f"  {r['name']:14s} at the trip q2 {q2[k]:5.1f} q3 {q3[k]:5.1f}; next 0.5 s q3 min {q3[w].min():5.1f}, q2 max {q2[w].max():5.1f}; "
          f"RATE_Q2CAP {'never' if tt is None else f'{tt - r[chr(116)+chr(95)+chr(101)+chr(110)+chr(100)]:+.2f} s'}")
