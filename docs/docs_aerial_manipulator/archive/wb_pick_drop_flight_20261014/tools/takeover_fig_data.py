"""Simulator takeover test, old vs fixed node -> ../analysis/takeover_sim.json (for the summary page).
Height through the scenario at 10 Hz (time from the pilot's switch to STAB), the phase boundaries, and the
summary numbers: motor messages from the node after the switch (all-NaN), the STAB height hold, the re-arm,
the climb and the hand-back."""
import os, json, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
RUNS = os.path.join(HERE, "..", "runs"); OUT = os.path.join(HERE, "..", "analysis")
CASES = {"fixed": sys.argv[1] if len(sys.argv) > 1 else "wb_fixed_4", "old": sys.argv[2] if len(sys.argv) > 2 else "wb_old_1"}
res = {}
for k, tag in CASES.items():
    d = np.load(os.path.join(RUNS, f"{tag}.npz"), allow_pickle=True)
    L = d["log"]; ts = float(d["t_switch"]); ph = [str(x) for x in d["phases"]]
    t = L[:, 0] - ts
    grid = np.arange(max(-5.0, t[0]), t[-1], 0.1)
    z = np.interp(grid, t, L[:, 3])
    starts = {}
    for i, name in enumerate(ph):
        m = np.where(L[:, 9] == i)[0]
        if len(m):
            starts[name] = round(float(t[m[0]]), 2)
    mt = d["motor_t"] - ts; mn = d["motor_nan"]
    hold = L[:, 9] == ph.index("stab_hold")
    res[k] = dict(tag=tag, t=[round(float(x), 2) for x in grid], z=[round(float(x), 3) for x in z], phase_start=starts,
                  motor_msgs_after=int((mt > 0.05).sum()), motor_nan_after=int(((mt > 0.05) & mn).sum()),
                  hold_min=round(float(L[hold, 3].min()), 3), hold_z0=round(float(L[hold, 3][0]), 3),
                  hold_std_mm=round(float(L[hold, 3].std() * 1e3), 1), rearmed=bool(d["rearmed"]),
                  climb_max=round(float(d["z_climb_max"]), 3), handback_dxy_mm=round(float(d["handback_dxy"]) * 1e3),
                  handback_dz_mm=round(float(d["handback_dz"]) * 1e3), mode_after=str(d["mode_after_switch"]))
    print(k, {kk: vv for kk, vv in res[k].items() if kk not in ("t", "z")})
json.dump(res, open(os.path.join(OUT, "takeover_sim.json"), "w"))
