#!/usr/bin/env python3
"""Data for the pick-and-place comparison report (artifact "Pick-and-Place Controller
Comparison", 2026-10-04): the ten benchmark runs (bench_{wb,geo}_1..5) through the same
scorers the README quotes -- bench_metrics.analyse (rho_UAV, phases, t_10 %, way-points),
pnp_score.score (task accuracy, tilt) and wb_mission_stats.stats (attitude ripple).

    PYTHONNOUSERSITE=1 /usr/bin/python3 report_data.py [out.json]

Writes ../runs/report_data.json by default.
"""
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import bench_metrics as BM          # noqa: E402
import pnp_score as PS              # noqa: E402
import wb_mission_stats as MS       # noqa: E402

RUNS = [f"bench_{rig}_{i}" for i in range(1, 6) for rig in ("wb", "geo")]
FS_MISSION = 20.0                   # Hz, whole-mission trace
FS_EVENT = 50.0                     # Hz, event-aligned traces
LIFT_WIN = (-3.0, 12.0)
REL_WIN = (-3.0, 10.0)


def resample(t, y, t0, t1, fs):
    tt = np.arange(t0, t1, 1.0 / fs)
    tt = tt[(tt >= t[0]) & (tt <= t[-1])]
    return tt, np.interp(tt, t, y)


def lift_time(d):
    names, mt = list(d["marks_name"]), d["marks_t"]
    pay = d["payload"]
    tc = mt[names.index("gripper_close:end")]
    i = np.where((pay[:, 0] > tc) & (pay[:, 3] > BM.CAP_TOP + BM.BOX_HALF + 0.005))[0]
    return float(pay[i[0], 0]) if len(i) else None


def clean(o):
    """JSON has no NaN: a value that never happened (t_10 % never reached) becomes null."""
    if isinstance(o, dict):
        return {k: clean(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [clean(v) for v in o]
    if isinstance(o, (float, np.floating)):
        return float(o) if np.isfinite(o) else None
    if isinstance(o, np.integer):
        return int(o)
    return o


def r3(x):
    return [round(float(v), 4) for v in x]


def main():
    out_path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "..", "runs", "report_data.json")
    out = dict(L=BM.L, runs=[])
    for fn in RUNS:
        a = BM.analyse(fn)
        d = np.load(os.path.join(BM.RUNS, fn + ".npz"), allow_pickle=True)
        names, mt = list(d["marks_name"]), d["marks_t"]
        t0 = float(mt[names.index("direct")]) if "direct" in names else float(a["t"][0])
        rho = a["eps"] / BM.L
        tt, rr = resample(a["t"], rho, t0, a["t"][-1], FS_MISSION)
        tl = lift_time(d)
        to = float(mt[names.index("gripper_open:start")])
        tl_t, tl_r = resample(a["t"], rho, tl + LIFT_WIN[0], tl + LIFT_WIN[1], FS_EVENT)
        to_t, to_r = resample(a["t"], rho, to + REL_WIN[0], to + REL_WIN[1], FS_EVENT)
        phases = []
        for nm, s, e in BM.PHASES:
            if s in names and e in names:
                phases.append([nm, round(mt[names.index(s)] - t0, 2), round(mt[names.index(e)] - t0, 2)])
        sc = PS.score(os.path.join(BM.RUNS, fn + ".npz"))
        st = MS.stats(fn) or {}
        out["runs"].append(dict(
            run=fn, rig=a["rig"], success=a["success"],
            mission=dict(t=r3(tt - t0), rho=r3(rr)),
            lift=dict(t=r3(tl_t - tl), rho=r3(tl_r)),
            release=dict(t=r3(to_t - to), rho=r3(to_r)),
            phases=phases,
            phase_stats={k: dict(max_mm=v["max"] * 1e3, rms_mm=v["rms"] * 1e3, T=v["T"])
                         for k, v in a["phase"].items()},
            t10=a["t10"], wp_mm={k: v * 1e3 for k, v in a["wp"].items()},
            score=dict(place_off_mm=sc.get("place_off_mm"), tilt_max=sc.get("tilt_max_direct"),
                       ee_close_mm=sc.get("ee_err_at_close_mm"), ee_open_mm=sc.get("ee_err_at_open_mm"),
                       hover_pick_mm=sc.get("hover_err_pick_mm"), hover_place_mm=sc.get("hover_err_place_mm"),
                       slip_mm=sc.get("carry_slip_mm"), mission_s=sc.get("mission_s")),
            ripple=dict(roll=st.get("roll"), pitch=st.get("pitch"), ee_rms=st.get("ee")),
        ))
    with open(out_path, "w") as f:
        json.dump(clean(out), f, separators=(",", ":"), allow_nan=False)
    print(f"wrote {out_path} ({os.path.getsize(out_path) / 1e3:.0f} kB)")


if __name__ == "__main__":
    main()
