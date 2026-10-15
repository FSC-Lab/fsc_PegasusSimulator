"""Hardware takeover table for the summary page -> ../analysis/takeover_table.json
(takeover.json + touchdown.json + the /fmu/in/actuator_motors stream after each switch)."""
import os, json
import numpy as np
from pd_common import load, HIST, NPZ, OUT, RUNS
tk = {r["label"][-15:]: r for r in json.load(open(os.path.join(OUT, "takeover.json"))) if "trace" in r}
td = {r["flight"]: r for r in json.load(open(os.path.join(OUT, "touchdown.json")))}
rows = []
for k, r in sorted(tk.items()):
    if k not in td:
        continue
    nm = r["label"]; p = os.path.join(HIST, nm + ".npz")
    if not os.path.exists(p):
        p = os.path.join(NPZ, [kk for kk, v in RUNS.items() if v[2] == nm][0] + ".npz")
    d, t0 = load(p); a = r["t_switch"]
    t = d["act_motors__recv"] - t0; U = d["act_motors__control"][:, :4]
    before = t[t <= a]
    from_direct = bool(len(before) and a - before.max() < 0.1)       # motor stream live at the switch = whole-body/geo DIRECT
    nan_after = int(((t > a) & np.isnan(U).all(1)).sum())
    rig = "decoupled" if "decoupled" in nm else "whole-body"
    rows.append(dict(flight=f"{k[4:6]}-{k[6:8]} {k[9:11]}:{k[11:13]}", rig=rig, from_mode="DIRECT" if from_direct else "SAFETY",
                     nan_msgs=nan_after, nan_rate_hz=250 if nan_after else 0, z_switch=round(r["z_switch"], 2),
                     sink=round(td[k]["sink"], 2), tilt_contact=round(td[k]["tilt_contact"], 1), tilt_end=round(r["tilt_end"], 1),
                     disarm=r["why"]))
json.dump(rows, open(os.path.join(OUT, "takeover_table.json"), "w"), indent=1)
for x in rows:
    print(x)
