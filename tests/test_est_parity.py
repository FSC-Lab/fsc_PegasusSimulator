#!/usr/bin/env python3
"""Piece-3 audit: the residual estimators' knobs in the cascade variant are at PARITY, so the
closed-loop comparison (piece 4) compares STRUCTURE, not tuning.

    ~/envs/isaac_env/bin/python3 tests/test_est_parity.py        (~2 min)

Reads the node's defaults (nmpc_cascade_steso_node.py, imported by path with no --est / env)
and measures each kind on scripts/est_parity.py's estimator-alone harness (200 Hz, the gate's
noise). Pins:
  1. MOMENT observers at equal noise floor: leso and mdob within 10% of the STESO's raw sigma
     floor on every axis (the cost C(s) pays). And the structural fact recorded with it: at that
     floor the LESO's lag/gain at the tank's 1.14 Hz is within 15% / 0.05 of the STESO's, while the
     first-order MDOB lags more than 1.5x (the reason the comparison carries MDOB_K=22 as well).
  2. FORCE twins at equal reading lag: leso and mdob within 8 ms of the STESO's harness lag +10 ms
     band (10-25 ms at 1.14 Hz), gains >= 0.98, and their floors BELOW the STESO's (the STESO's is
     its chatter: recorded as a finding, not hidden).
  3. The predictor's meas_lag table has one measured entry per kind, in 0-20 ms; the flown
     kinds' entries agree within 5 ms (they were tuned to the same lag).
  4. The knob defaults are the solved parity values (re-solve with `scripts/est_parity.py --solve`
     and update both the node and this pin together if the harness or the noise model changes).
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
sys.path.insert(0, os.path.join(HERE, "..", "scripts"))
import est_parity as ep                                      # noqa: E402

FAILS = []


def check(cond, msg):
    print(("  ok   " if cond else "  FAIL ") + msg)
    if not cond:
        FAILS.append(msg)


def main():
    knobs, lag_table = ep.node_defaults()
    print("node defaults:", {k: v for k, v in knobs.items()}, "meas_lag:", lag_table)
    print("== 4. the defaults are the solved parity values")
    want = {"steso": (20.0, 3.0), "leso": (42.0, 99.0), "mdob": (11.0, 66.0)}
    for kind, (km, kf) in want.items():
        check(abs(knobs[kind][0] - km) < 1e-9 and abs(knobs[kind][1] - kf) < 1e-9,
              f"{kind}: moment knob {knobs[kind][0]} / force knob {knobs[kind][1]} == pinned {km} / {kf}")

    print("== 1. moment observers: equal noise floor (and the lag/gain that comes with it)")
    fl = {k: ep.floor(k, knobs[k][0], "M") for k in knobs}
    lg = {k: ep.lag_gain(k, knobs[k][0], "M") for k in knobs}
    for k in ("leso", "mdob"):
        r = fl[k] / fl["steso"]
        check(np.all(np.abs(r - 1.0) < 0.10), f"{k}: raw sigma floor / STESO's = {np.round(r, 3).tolist()} on roll/pitch/yaw (within 10%)")
    check(abs(lg["leso"][0] / lg["steso"][0] - 1.0) < 0.15 and abs(lg["leso"][1] - lg["steso"][1]) < 0.05,
          f"leso at equal floor matches the STESO at 1.14 Hz: lag {1e3 * lg['leso'][0]:.1f} vs {1e3 * lg['steso'][0]:.1f} ms, gain {lg['leso'][1]:.3f} vs {lg['steso'][1]:.3f}")
    check(lg["mdob"][0] > 1.5 * lg["steso"][0],
          f"mdob (first order) at equal floor lags {1e3 * lg['mdob'][0]:.1f} ms vs the STESO's {1e3 * lg['steso'][0]:.1f} (> 1.5x: the structural fact; MDOB_K=22 is its equal-lag row)")
    f22 = ep.floor("mdob", 22.0, "M", 1); l22, _ = ep.lag_gain("mdob", 22.0, "M")
    check(1.7 < f22 / fl["steso"][1] < 2.3 and abs(l22 / lg["steso"][0] - 1.0) < 0.2,
          f"mdob K=22: lag {1e3 * l22:.1f} ms (equal to the STESO's within 20%) at {f22 / fl['steso'][1]:.2f}x its floor")

    print("== 2. force twins: equal reading lag at the tank's mode, lower floors")
    lf = {k: ep.lag_gain(k, knobs[k][1], "F") for k in knobs}
    ff = {k: ep.floor(k, knobs[k][1], "F", 0) for k in knobs}
    for k in ("leso", "mdob"):
        check(0.010 <= lf[k][0] <= 0.025 and lf[k][1] >= 0.98,
              f"{k} force twin: lag {1e3 * lf[k][0]:.1f} ms, gain {lf[k][1]:.3f} at 1.14 Hz (10-25 ms band, gain >= 0.98)")
        check(ff[k] < ff["steso"], f"{k} force twin floor {ff[k]:.4f} N < STESO's {ff['steso']:.4f} (the STESO's is its chatter 1.1 L_F dt m = {1.1 * 3.0 * ep.DT * ep.M_KG:.3f})")
    check(lf["steso"][0] < 0.010 and lf["steso"][1] > 0.99, f"steso force twin (L_F 3): lag {1e3 * lf['steso'][0]:.1f} ms, gain {lf['steso'][1]:.3f} (exact inside L_F)")

    print("== 3. the predictor's measurement lag per kind")
    check(lag_table is not None and set(lag_table) == {"steso", "leso", "mdob"} and all(0.0 <= v <= 0.020 for v in lag_table.values()),
          f"PRED_LAG_BY_KIND {lag_table}: one measured entry per kind, 0-20 ms")
    if lag_table:
        check(max(lag_table.values()) - min(lag_table.values()) <= 0.005, "the three entries agree within 5 ms (the twins were tuned to one lag)")

    print()
    if FAILS:
        print(f"FAILED {len(FAILS)}:"); [print("  - " + f) for f in FAILS]; return 1
    print("ESTIMATOR PARITY -- PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
