#!/usr/bin/env python3
"""final_eval.py -- the offline verdict: the modular law's CMA finalists and the
whole-body H1b tune, flown on the SAME bench under the SAME conditions.

Per controller (common random numbers across controllers):
  mirror  seeds 0..2    the comparison circle (r 0.50 m, 24 s lap, q2 25 +- 15 deg @ 6 s)
  delay   28 ms         transport-delay margin (x1.75 the nominal 16 ms)
  robust                the _sim_robustness stress plant
  isaac   rtf 0.48      the Isaac wall-clock emulation (circle_bench rtf=0.48), the
                        regime the Isaac flights actually run in
Writes ../analysis/final_eval.json and prints a table. Finalists = the K best
DISTINCT candidates of the stage-2 log (relative gain distance > 10 %).

    OMP_NUM_THREADS=1 /usr/bin/python3 final_eval.py [--k 4]
"""
import argparse, json, multiprocessing as mp, os, time
import numpy as np
import modular_bench as MB
import circle_bench as CB
import mapped as MP
import tune_modular as TM

HERE = os.path.dirname(os.path.abspath(__file__))
STREAM = TM.STREAM
CASES = {"mirror_s0": dict(seed=0), "mirror_s1": dict(seed=1), "mirror_s2": dict(seed=2),
         "delay28": dict(delay_ms=28.0), "robust": dict(profile="robustness"),
         "isaac": dict(rtf=0.48, t_settle=20.0),
         "armdelay32": dict(arm_delay_ms=32.0, plant_armature=[0.02] * 4),
         "isaac_like": dict(rtf=0.48, t_settle=20.0, arm_delay_ms=24.0, plant_armature=[0.02] * 4)}


def job(args):
    who, p, case = args
    kw = CASES[case]
    t0 = time.time()
    if who == "WB H1b":
        r = MB.run_wb(None, STREAM, **kw)
    else:
        r = MB.run(MP.make(p), STREAM, **kw)
    r.pop("H", None); r.pop("Hq", None); r.pop("Hqd", None)
    return who, case, r, time.time() - t0


def finalists(k):
    recs = [json.loads(l) for l in open(os.path.join(HERE, "..", "analysis", "cma_modular_log.jsonl"))]
    recs.sort(key=lambda r: r["J"])
    out = []
    for r in recs:
        z = np.log([r["p"][n] for n in TM.NAMES])
        if all(np.max(np.abs(z - np.log([o["p"][n] for n in TM.NAMES]))) > 0.10 for o in out):
            out.append(r)
        if len(out) >= k:
            break
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--k", type=int, default=4)
    ap.add_argument("--jobs", type=int, default=30)
    ap.add_argument("--gains", default="", help="evaluate THIS gains json (plus WB) instead of CMA finalists")
    ap.add_argument("--out", default="final_eval.json")
    a = ap.parse_args()
    whos = {"WB H1b": None}
    fins = [] if a.gains else finalists(a.k)
    if a.gains:
        p = dict(MP.TABLE2_MAPPED); p.update(json.load(open(a.gains))["best"])
        whos["MOD final"] = p
    for i, r in enumerate(fins):
        p = dict(MP.TABLE2_MAPPED); p.update(TM.FIXED); p.update(r["p"])
        whos[f"MOD#{i} (J {r['J']:.0f})"] = p
    tasks = [(w, p, c) for w, p in whos.items() for c in CASES]
    res = {}
    with mp.get_context("fork").Pool(a.jobs) as pool:
        for who, case, r, dtw in pool.imap_unordered(job, tasks):
            res.setdefault(who, {})[case] = r
            print(f"  {who:22s} {case:10s} {r['verdict']:9s} [{dtw:.0f}s]", flush=True)
    print()
    hdr = f"{'controller':22s} | " + " | ".join(f"{c:>17s}" for c in CASES)
    print(hdr); print("-" * len(hdr))
    for who in whos:
        cells = []
        for c in CASES:
            r = res[who][c]
            cells.append(f"{r['ee_rms']*1e3:6.1f} / {r['com_rms']*1e3:6.1f}" if r["verdict"] == "completed"
                         else f"ABORT @{r['t_end']:5.1f}s")
        print(f"{who:22s} | " + " | ".join(f"{x:>17s}" for x in cells))
    print("(cells: EE abs rms / CoM rms [mm])")
    json.dump({"whos": {w: p for w, p in whos.items()}, "res": res, "stream": STREAM},
              open(os.path.join(HERE, "..", "analysis", a.out), "w"), indent=1, default=float)


if __name__ == "__main__":
    main()
