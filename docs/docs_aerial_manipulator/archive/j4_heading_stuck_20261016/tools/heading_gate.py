#!/usr/bin/env python3
"""Whole-vehicle regression of a stiffer EE heading (2026-10-16). The 2026-09-27 circle bench, UNCHANGED
(../../circle_tune_20260927/tools/circle_bench.py: exact Python law + 4-D L1 + mirror plant + flight-like feedback, RTF 1),
flying the shipped H1b law against H1b with candidate wb_ky_psi / wb_dy_psi. Same tests as that campaign's final gate
(minus the RTF-0.48 Isaac emulations, which no longer apply): the showcase and flown circles x3 seeds, extra delay 24-40 ms,
a gust, and the robustness plant at 16/24 ms.
    PYTHONNOUSERSITE=1 /usr/bin/python3 heading_gate.py [ky:dy ...]  -> ../analysis/heading_gate.json"""
import json, multiprocessing as mp, os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, *[".."] * 5))
# circle_bench.py resolves the repo root one level short since it moved into archive/; give it its imports here
for p_ in (os.path.join(HERE, "..", "..", "sim2real_tuning_20260926", "tools"),
           os.path.join(REPO, "application", "robotic_arm", "utils"),
           os.path.join(REPO, "extensions", "fsc_aerial_manipulation"),
           os.path.join(HERE, "..", "..", "circle_tune_20260927", "tools")):
    sys.path.insert(0, p_)
import circle_bench as CB  # noqa: E402
_make_l1 = CB.make_l1


def _make_l1_hold(p):
    """circle_bench.make_l1 + the j4 stuck hold when the candidate asks for it (p["stuck_j4"])."""
    kw = _make_l1(p)
    if p.get("stuck_j4"):
        kw = dict(kw, stuck_hold_j4=True, stuck_speed=0.02, stuck_filter_s=0.25)
    return kw


CB.make_l1 = _make_l1_hold
AN = os.path.join(HERE, "..", "analysis")
H1B = dict(CB.BASE, **json.load(open(os.path.join(HERE, "..", "..", "circle_tune_20260927", "analysis", "tuned_H1b.json")))["best"])
SHOW, FLOWN = "r075_L24_sync15_cw", "r050_L24_half10"
pairs = [a.split(":") for a in sys.argv[1:]] or [("2.0", "0.829"), ("2.5", "0.93"), ("3.0", "1.015")]
C = {"H1b": H1B}
for ky, dy in pairs:
    C[f"psi{ky}"] = dict(H1B, ky_psi=float(ky), dy_psi=float(dy))
if len(sys.argv) == 1:                                   # the shipped pair, with the j4 hold
    C["psi2.5+hold"] = dict(H1B, ky_psi=2.5, dy_psi=0.93, stuck_j4=True)
T = []
for sd in (11, 12, 13):
    T += [(f"show_s{sd}", dict(s=SHOW, seed=sd)), (f"flown_s{sd}", dict(s=FLOWN, seed=sd))]
for dl in (24.0, 28.0, 32.0, 36.0, 40.0):
    T.append((f"delay{int(dl)}", dict(s=SHOW, delay_ms=dl)))
T.append(("gust", dict(s=SHOW, gust=(10.0, 3.0, 3.0, 0.0))))
T += [("rob16", dict(s=SHOW, profile="robustness")), ("rob24", dict(s=SHOW, profile="robustness", delay_ms=24.0))]


def job(a):
    n, tn, kw = a; kw = dict(kw); s = kw.pop("s")
    r = CB.simulate(C[n], CB.stream(s), **kw)
    return n, tn, {k: (float(v) if isinstance(v, (np.floating, float, int)) else v) for k, v in r.items()
                   if k not in ("H", "Hq", "Hqd")}


if __name__ == "__main__":
    print("H1b:", {k: H1B[k] for k in ("ky_psi", "dy_psi", "k_R", "k_w", "omega_x")})
    out = {n: {} for n in C}
    with mp.get_context("fork").Pool(30) as pool:
        for n, tn, r in pool.imap_unordered(job, [(n, tn, kw) for n in C for tn, kw in T]):
            out[n][tn] = r
    json.dump(out, open(os.path.join(AN, "heading_gate.json"), "w"), indent=1, default=str)
    ok = lambda R, t: R[t]["verdict"] == "completed"
    e = lambda R, t, k="ee_rms": (f"{R[t][k]*1e3:.1f}" if ok(R, t) else "ABORT")
    for n, R in out.items():
        sh = np.mean([R[f"show_s{s}"]["ee_rms"] for s in (11, 12, 13)]) * 1e3
        fl = np.mean([R[f"flown_s{s}"]["ee_rms"] for s in (11, 12, 13)]) * 1e3
        print(f"{n:8s} EE rms show {sh:5.1f} flown {fl:5.1f} mm | delay 24/28/32/36/40 "
              + "/".join(e(R, f"delay{d}") for d in (24, 28, 32, 36, 40))
              + f" | gust {e(R, 'gust')} | rob16 {e(R, 'rob16')} rob24 {e(R, 'rob24')}")
    print("available metric keys:", sorted(next(iter(out["H1b"].values())).keys()))
