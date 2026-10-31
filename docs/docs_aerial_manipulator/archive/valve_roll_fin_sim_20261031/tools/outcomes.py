#!/usr/bin/env python3
"""(2026-10-31) Per drift seed and handle: ok / the abort (which guard, when in the push step, DIRECT kept
or lost). The push step = 1.5 s settle + the 18 s turn. v2 = today's chain2 runs (v2_sN) and the 10-31
control (vs_hw_sN, same build); v3 = runs/rf_sN (free) and rl_sN (locked).
    PYTHONNOUSERSITE=1 /usr/bin/python3 outcomes.py        (writes ../outcomes.txt too)
"""
import os
import re

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
RUNS = os.path.join(HERE, "..", "runs") + "/"
V2 = os.path.join(HERE, "..", "..", "valve_safety_fixes_20261030", "sim", "runs") + "/"


def outcome(path):
    if not os.path.exists(path):
        return None
    z = np.load(path, allow_pickle=True)
    M = dict(zip(z["marks_name"].tolist(), z["marks_t"].tolist()))
    if not bool(z["aborted"]):
        return "ok"
    ev = [str(e) for e in z["events"]]
    ab = next((e for e in ev if "ABORTING" in e), None)
    if ab is None:
        return "aborted ?"
    ta = float(ab.split("s]")[0].strip("[ "))
    guard = re.search(r"SAFETY GUARD: (q3[^(]*|[a-z][^(-]*)", ab)
    g = guard.group(1).strip() if guard else "?"
    g = re.sub(r"\s+", " ", g)[:42]
    od = z["odom"]
    w = (od[:, 0] >= ta) & (od[:, 0] <= ta + 8)
    kept = bool(od[w, 11].min() > 0.5) if w.any() else False
    return f"ABORT {ta - M.get('push:start', ta):4.1f}s ({g}); DIRECT {'kept' if kept else 'LOST'}"


lines = []
tally = {"v2": [0, 0], "free": [0, 0], "locked": [0, 0]}
for s in range(2, 9):
    row = {}
    v2 = [outcome(RUNS + f"v2_s{s}.npz"), outcome(V2 + f"vs_hw_s{s}.npz")]
    row["v2"] = [o for o in v2 if o]
    row["free"] = [o for o in [outcome(RUNS + f"rf_s{s}.npz")] if o]
    row["locked"] = [o for o in [outcome(RUNS + f"rl_s{s}.npz")] if o]
    for k, v in row.items():
        tally[k][0] += sum(o == "ok" for o in v)
        tally[k][1] += len(v)
    lines.append(f"seed {s}")
    for k in ("v2", "free", "locked"):
        lines.append(f"   {k:7s} " + (" | ".join(row[k]) if row[k] else "-"))
lines.append("completed / flown: " + ", ".join(f"{k} {a}/{n}" for k, (a, n) in tally.items()))
txt = "\n".join(lines)
print(txt)
open(os.path.join(HERE, "..", "outcomes.txt"), "w").write(txt + "\n")
