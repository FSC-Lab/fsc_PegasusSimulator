"""Data for the short summary page -> ../analysis/artifact_data.json
(1) PX4 pitch at the drop point, 20 Hz: PD-1 over the full bin (86-106 s) vs 10-09 PP-1 over the hat (124-144 s);
(2) median rotor commands per flight in quasi-static DIRECT hover (yaw_history.json)."""
import os, json
import numpy as np
from pd_common import load, px4_rpy, OUT, HIST

def pitch(path, a, b):
    d, t0 = load(path); tp, ro, pi_, ya = px4_rpy(d, t0)
    tt = np.arange(a, b, 0.05)
    return [round(float(x), 3) for x in np.degrees(np.interp(tt, tp, pi_))]

out = {"pitch_t": [round(x, 2) for x in np.arange(0, 20, 0.05)],
       "pitch_bin": pitch("d1", 86.0, 106.0),
       "pitch_hat": pitch(os.path.join(HIST, "flight_wb_l1_4d_pick_and_place_20261009_172557.npz"), 124.0, 144.0)}
rows = json.load(open(os.path.join(OUT, "yaw_history.json")))
out["flights"] = [{"stamp": r["stamp"].replace("ps4", ""), "rig": r["rig"], "u": [round(x, 3) for x in r["u"]],
                   "tau_z": round(r["tau_z"], 3)} for r in rows]
json.dump(out, open(os.path.join(OUT, "artifact_data.json"), "w"))
print(len(out["flights"]), "flights;", len(out["pitch_bin"]), "pitch samples")
