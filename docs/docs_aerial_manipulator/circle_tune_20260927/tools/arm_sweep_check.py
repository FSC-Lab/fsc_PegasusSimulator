#!/usr/bin/env python3
"""Bench the tuned law on the ARM SWEEP the drone will actually fly (2026-09-28).

The H1b tune was scored on circles whose q2 sweep is 12-48 s per cycle; the drone's
yaml (origin/dev_CCM 4a9c6d6) now flies fold 55, q2 25 +- 15 deg at a 6 s period
(~16 deg/s). This runs shipped (pre-tune) vs H1b on that stream, on flight 3's 12 s
sweep and on the flown 48 s design, mirror + robustness plants, seeds 11-13, and a
transport-delay margin on the mirror plant.
    /usr/bin/python3 arm_sweep_check.py  -> analysis/arm_sweep_check.json
"""
import json, multiprocessing as mp, os
import numpy as np
import circle_bench as CB

AN = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "analysis")
H1B = dict(CB.BASE); H1B.update(json.load(open(os.path.join(AN, "tuned_H1b.json")))["best"])
LAWS = {"before": dict(CB.BASE), "H1b": H1B}
STREAMS = {"6 s sweep (drone now)": "r050_L24_f55_a15_p6", "12 s sweep (flight 3)": "r050_L24_f55_a15_p12",
           "48 s sweep (tuned on)": "r050_L24_half10"}
JOBS = [(l, s, prof, sd, 16.0) for l in LAWS for s in STREAMS for prof in ("mirror", "robustness") for sd in (11, 12, 13)]
JOBS += [(l, s, "mirror", 11, d) for l in LAWS for s in STREAMS for d in (24.0, 28.0)]


def job(a):
    law, st, prof, seed, dly = a
    r = CB.simulate(LAWS[law], CB.stream(STREAMS[st]), profile=prof, seed=seed, delay_ms=dly)
    return a, {k: v for k, v in r.items() if k not in ("H", "Hq", "Hqd", "F")}


def main():
    with mp.get_context("fork").Pool(24) as P:
        res = P.map(job, JOBS)
    out = {}
    for (law, st, prof, seed, dly), r in res:
        out.setdefault(f"{law}|{st}|{prof}|{dly:.0f}", []).append(r)
    json.dump({k: v for k, v in out.items()}, open(os.path.join(AN, "arm_sweep_check.json"), "w"), default=float, indent=1)
    print(f"{'stream':24} {'plant':10} {'law':7} {'done':5} {'EE abs':>7} {'CoM':>6} {'head':>5} {'|eR|':>6} {'q2/q3 err':>11} {'tau pk':>6} {'clamp%':>6}")
    for st in STREAMS:
        for prof in ("mirror", "robustness"):
            for law in LAWS:
                rs = out[f"{law}|{st}|{prof}|16"]
                ok = [r for r in rs if r["verdict"] == "completed"]
                m = lambda k, s=1: np.mean([r[k] for r in ok]) * s if ok else float("nan")  # noqa: E731
                qe = np.mean([r["q_err_rms"] for r in ok], axis=0) if ok else [np.nan] * 4
                print(f"{st:24} {prof:10} {law:7} {len(ok)}/{len(rs):<3} {m('ee_rms',1e3):7.1f} {m('com_rms',1e3):6.1f} {m('head_rms'):5.2f} {m('eR_mean'):6.4f} "
                      f"{qe[1]:5.2f}/{qe[2]:<5.2f} {max(r['tau_pk'] for r in ok) if ok else float('nan'):6.2f} {max(r['sat_pct'] for r in ok) if ok else float('nan'):6.1f}")
    print("delay margin (mirror, seed 11): EE abs mm or ABORT")
    for st in STREAMS:
        for law in LAWS:
            cells = []
            for d in (16, 24, 28):
                r = out[f"{law}|{st}|mirror|{d}"][0]
                cells.append(f"{d} ms {r['ee_rms']*1e3:5.1f}" if r["verdict"] == "completed" else f"{d} ms ABORT")
            print(f"  {st:24} {law:7} " + " | ".join(cells))


if __name__ == "__main__":
    main()
