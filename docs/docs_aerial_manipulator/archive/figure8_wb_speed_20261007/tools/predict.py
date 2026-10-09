#!/usr/bin/env python3
"""predict.py -- the whole-body figure-8's HARDWARE EE rms predicted at higher mean speeds (2026-10-07).

Inputs
  * Isaac sweep (A 0.70 / B 0.35, s = 1, RTF 1, mirror plant): every wb npz in ../data and the 10-05 A 0.70
    flights in ../../figure8_compare_20261005/data, scored by am_ee_compare_score.analyse (the same
    definition as the hardware scorer: odometry + encoders through the planner FK vs the plan stream, over
    the whole EXECUTING span);
  * the 1005 hardware figure-8 (../../wb_vs_decoupled_figure8_flight_20261005/analysis/fig8_metrics.json):
    WB-3/4 at 0.10, WB-5/6 at 0.13; DEC-5 0.10 / DEC-6 0.13;
  * ../analysis/hw_decompose.json (repeatable vs random part of the two-run pairs) and hw_holds.json.

Models of hardware rms(v) (sim(v) = linear fit of the Isaac rms over the hardware-bounded speeds):
  A  proportional:        hw = R sim,                      R from the four hardware runs (rms-pooled)
  B  constant wander:     hw^2 = (c sim)^2 + n^2,          c = repeatable/sim, n^2 = pooled random variance
  C  wander grows:        hw^2 = (c sim)^2 + n0^2 + (k v)^2, n0 = static-hold wander, k from the two pairs
  D  linear in speed:     hw = a + b v through the two hardware speed means
Peak = rms x the largest hardware peak/rms ratio of the four runs. Footprint = the planned base + EE
reference box (Isaac stream) grown by the predicted peak.

    /usr/bin/python3 predict.py [--json ../analysis/prediction.json]
"""
import glob, json, math, os, sys
from collections import defaultdict
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
CAMP = os.path.join(HERE, "..")
sys.path.insert(0, os.path.join(HERE, "..", "..", "..", "..", "..", "application", "robotic_arm", "utils"))
import am_ee_compare_score as SC  # noqa: E402

HWM = json.load(open(os.path.join(CAMP, "..", "wb_vs_decoupled_figure8_flight_20261005", "analysis", "fig8_metrics.json")))
DEC = json.load(open(os.path.join(CAMP, "analysis", "hw_decompose.json")))
HOLDS = json.load(open(os.path.join(CAMP, "analysis", "hw_holds.json")))
OLD = os.path.join(CAMP, "..", "figure8_compare_20261005", "data")


def speed_of(path):
    b = os.path.basename(path)
    if b.startswith("wb_A070_v"):
        return int(b.split("_v")[1][:3]) / 100.0
    return int(b.split("_v")[1][:3]) / 100.0


def box(P):
    return np.array([P[:, 0].min(), P[:, 0].max(), P[:, 1].min(), P[:, 1].max()])


def score(path):
    res, ser = SC.analyse(path)
    if ser is None:                                    # no trajectory run (a guard trip while hovering)
        return dict(file=os.path.basename(path), v=speed_of(path), aborted=True, reason=res.get("error", ""), no_run=True,
                    fused=os.path.basename(path).endswith("_f.npz"))
    d, marks = SC.load(path)
    t0, t1 = ser["t0"], ser["t1"]
    w = d["wbref"]; w = w[(w[:, 0] >= t0) & (w[:, 0] <= t1)]
    e = ser["ee"]
    bb = box(w[:, 30:32]); eb = box(e[:, 4:6])
    return dict(file=os.path.basename(path), v=speed_of(path), aborted=bool(res["aborted"]), reason=res["reason"], no_run=False,
                fused=os.path.basename(path).endswith("_f.npz"),
                s=res.get("s"), s_max=res.get("s_max"), ee_rms=res["ee_pos_rms_mm"], ee_peak=res["ee_pos_max_mm"],
                base_rms=res["base_pos_rms_mm"], base_peak=res["base_pos_max_mm"], tilt=res["tilt_max_deg"],
                head_rms=float(math.sqrt(np.mean(np.asarray(ser["heading"][1]) ** 2))),
                joint_rms=res["joint_rms_deg"], plan_box=[float(x) for x in np.r_[min(bb[0], eb[0]), max(bb[1], eb[1]), min(bb[2], eb[2]), max(bb[3], eb[3])]])


def main():
    out_json = sys.argv[sys.argv.index("--json") + 1] if "--json" in sys.argv else None
    paths = sorted(glob.glob(os.path.join(CAMP, "data", "wb_v*.npz"))) + sorted(glob.glob(os.path.join(OLD, "wb_A070_v*.npz")))
    runs = [score(p) for p in paths]
    for r in runs:
        if r["no_run"]:
            print(f"{r['file']:22s} v {r['v']:.2f}  NO RUN ({r['reason']}: guard trip while hovering in DIRECT)"); continue
        print(f"{r['file']:22s} v {r['v']:.2f}  EE rms {r['ee_rms']:5.1f} peak {r['ee_peak']:5.1f}  base {r['base_rms']:5.1f}  head {r['head_rms']:.2f}  tilt {r['tilt']:.1f}"
              f"  s {r['s']}/{r['s_max']:.3f}" + (f"  ABORTED {r['reason']}" if r["aborted"] else ""))
    ok = [r for r in runs if not r["aborted"] and not r["fused"]]
    fz = [r for r in runs if not r["aborted"] and r["fused"]]
    byv = defaultdict(list)
    for r in ok: byv[r["v"]].append(r)
    V = np.array(sorted(byv)); S = np.array([np.mean([r["ee_rms"] for r in byv[v]]) for v in V])
    hwb = V <= 0.2001
    lin = np.polyfit(V[hwb], S[hwb], 1)
    sim = lambda v: float(np.polyval(lin, v))
    print(f"\nIsaac rms fit (v <= 0.20): sim(v) = {lin[1]:.2f} + {lin[0]:.1f} v  [mm, v in m/s]")

    hw = {k: v for k, v in HWM.items() if v["grp"] == "wb"}
    dec = {k: v for k, v in HWM.items() if v["grp"] != "wb"}
    hv = np.array([v["speed"] for v in hw.values()]); hr = np.array([v["metrics"]["ee_pos"]["rms_norm"] for v in hw.values()])
    hpk = np.array([v["metrics"]["ee_pos"]["max_norm"] for v in hw.values()])
    dec_rms = [v["metrics"]["ee_pos"]["rms_norm"] for v in dec.values()]
    peak_ratio = float((hpk / hr).max())
    # A
    R = math.sqrt(np.mean(hr ** 2) / np.mean([sim(v) ** 2 for v in hv]))
    # B / C
    cs = [DEC[f"{v:.2f}"]["repeatable_rms_mm"] / sim(v) for v in (0.10, 0.13)]; c = float(np.mean(cs))
    nr = [DEC[f"{v:.2f}"]["random_rms_mm"] for v in (0.10, 0.13)]
    nB = math.sqrt(np.mean(np.square(nr)))
    holds = [h["ee_rms"] for rows in HOLDS.values() for h in rows]
    n0 = math.sqrt(np.mean(np.square(holds)))
    k2 = np.mean([max(n ** 2 - n0 ** 2, 0) / v ** 2 for n, v in zip(nr, (0.10, 0.13))]); kC = math.sqrt(k2)
    # D
    m10 = math.sqrt(np.mean(hr[hv < 0.115] ** 2)); m13 = math.sqrt(np.mean(hr[hv > 0.115] ** 2))
    bD = (m13 - m10) / 0.03; aD = m10 - bD * 0.10
    models = {
        "A": lambda v: R * sim(v),
        "B": lambda v: math.sqrt((c * sim(v)) ** 2 + nB ** 2),
        "C": lambda v: math.sqrt((c * sim(v)) ** 2 + n0 ** 2 + (kC * v) ** 2),
        "D": lambda v: aD + bD * v,
    }
    print(f"hardware: WB rms {np.round(hr, 1)} at {hv};  DEC rms {np.round(dec_rms, 1)}; peak/rms max {peak_ratio:.2f}")
    print(f"model A: R = {R:.2f}   B: c = {c:.2f} ({cs[0]:.2f}/{cs[1]:.2f}), n = {nB:.1f} mm   C: n0 = {n0:.1f} mm (holds {np.round(holds,1)}), k = {kC:.0f} mm/(m/s)   D: {aD:.1f} + {bD:.0f} v")
    print("check at the flown speeds (hardware rms-mean 0.10 / 0.13 = %.1f / %.1f):" % (m10, m13),
          "  ".join(f"{k} {f(0.10):.1f}/{f(0.13):.1f}" for k, f in models.items()))

    dec_lo, dec_hi = min(dec_rms), max(dec_rms)
    table = []
    print(f"\n{'v':>5s} {'n':>2s} {'Isaac rms':>9s} {'Isaac peak':>10s} {'tilt':>5s} | " + " ".join(f"{k:>6s}" for k in models) + " | band      peak(hi)  footprint (hi)")
    for v in V:
        rs = byv[v]
        pred = {k: f(v) for k, f in models.items()}
        lo, hi = min(pred.values()), max(pred.values())
        pb = np.array([r["plan_box"] for r in rs])
        planned = np.array([pb[:, 0].min(), pb[:, 1].max(), pb[:, 2].min(), pb[:, 3].max()])
        pk = 1e-3 * hi * peak_ratio
        fp = [float(planned[1] - planned[0] + 2 * pk), float(planned[3] - planned[2] + 2 * pk)]
        row = dict(v=float(v), n=len(rs), sim_rms=float(np.mean([r["ee_rms"] for r in rs])), sim_rms_runs=[r["ee_rms"] for r in rs],
                   sim_peak=float(max(r["ee_peak"] for r in rs)), tilt=float(max(r["tilt"] for r in rs)), pred=pred, band=[lo, hi],
                   peak_hi_mm=1e3 * pk, planned_box=planned.tolist(), planned_size=[float(planned[1] - planned[0]), float(planned[3] - planned[2])],
                   footprint_hi=fp, hw_bounds=bool(v <= 0.2001))
        table.append(row)
        print(f"{v:5.2f} {len(rs):2d} {row['sim_rms']:9.1f} {row['sim_peak']:10.1f} {row['tilt']:5.1f} | " + " ".join(f"{pred[k]:6.1f}" for k in models)
              + f" | {lo:4.0f}-{hi:3.0f} mm  {1e3*pk:5.0f} mm  {planned[1]-planned[0]:.2f}x{planned[3]-planned[2]:.2f} -> {fp[0]:.2f}x{fp[1]:.2f} m"
              + ("" if row["hw_bounds"] else "  (sim-only planner bounds)"))
    if fz:
        print("\nEKF2-fused feedback (the hardware's path) vs raw mocap, Isaac EE rms [mm]:")
        for r in sorted(fz, key=lambda r: r["v"]):
            raw = [x["ee_rms"] for x in byv.get(r["v"], [])]
            print(f"  v {r['v']:.2f}: fused {r['ee_rms']:.1f}  raw {np.round(raw, 1)}  (raw fit {sim(r['v']):.1f})")
    # speed of parity with the decoupled hardware rms, per model
    vv = np.linspace(0.10, 0.35, 2501)
    par = {}
    for k, f in models.items():
        y = np.array([f(x) for x in vv])
        par[k] = dict(lo=float(vv[np.argmax(y >= dec_lo)]) if (y >= dec_lo).any() else None,
                      hi=float(vv[np.argmax(y >= dec_hi)]) if (y >= dec_hi).any() else None)
    print(f"\nspeed at which the predicted WB hardware rms reaches the decoupled hardware rms ({dec_lo:.1f}-{dec_hi:.1f} mm):")
    for k, p in par.items():
        fmt = lambda x: f"{x:.3f}" if x is not None else "> 0.35"
        print(f"  model {k}: {fmt(p['lo'])} - {fmt(p['hi'])} m/s")
    if out_json:
        json.dump(dict(runs=runs, sim_fit=lin.tolist(), hw=dict(v=hv.tolist(), rms=hr.tolist(), peak=hpk.tolist(), dec=dec_rms, peak_ratio=peak_ratio),
                       params=dict(R=R, c=c, nB=nB, n0=n0, kC=kC, aD=aD, bD=bD), table=table, parity=par), open(out_json, "w"), indent=1)
        print("wrote", out_json)


if __name__ == "__main__":
    main()
