#!/usr/bin/env python3
"""Robustness of the j4 candidates across the model's uncertain numbers (each varied alone), scored on the
two things the change must do: kill the LOADED swing, and not leave the wrist parked off target or
lagging. PYTHONNOUSERSITE=1 /usr/bin/python3 j4_robust.py"""
import numpy as np
import j4_model as M

VARS = [("nominal", {}),
        ("kinetic friction -25 %", dict(F_K=M.F_K * 0.75)), ("kinetic friction +25 %", dict(F_K=M.F_K * 1.25)),
        ("static friction 0.035", dict(F_S=0.035)), ("static friction 0.060", dict(F_S=0.060)),
        ("viscous 0.010", dict(B_V=0.010)), ("viscous 0.030", dict(B_V=0.030)),
        ("Ke_true -2 %", dict(KE_TRUE=M.KE_TRUE * 0.98)), ("Ke_true +2 %", dict(KE_TRUE=M.KE_TRUE * 1.02)),
        ("law K 0.064 D 0.047", dict(K_LAW=0.064, D_LAW=0.047)), ("law K 0.045 D 0.040", dict(K_LAW=0.045, D_LAW=0.040)),
        ("L1 ff 0.2 rad/s", dict(W_FF=0.2)), ("L1 ff 0.6 rad/s", dict(W_FF=0.6)),
        ("transport 5 ticks", dict(DELAY=5))]
CANDS = [(7.7776 / M.KAPPA, 0.0, "shipped"), (0.0335, 0.05, "relay 0.0335, d 0.05"),
         (0.030, 0.05, "relay 0.030, d 0.05"), (0.030, 0.07, "relay 0.030, d 0.07")]


def run():
    base = {k: getattr(M, k) for k in ("F_K", "F_S", "B_V", "KE_TRUE", "K_LAW", "D_LAW", "W_FF", "DELAY")}
    print(f"{'variation':24s} | " + " | ".join(f"{c[2]:>22s}" for c in CANDS))
    print(f"{'':24s} | " + " | ".join(f"{'swing p-p / parked':>22s}" for _ in CANDS))
    for name, ov in VARS:
        for k, v in base.items():
            setattr(M, k, v)
        for k, v in ov.items():
            setattr(M, k, v)
        cells = []
        for f_r, d, _ in CANDS:
            o = M.simulate(T=60, f_r=f_r, d=d, loaded=True, q0=np.radians(15))
            m = o[:, 0] > 20
            pp = np.degrees(np.ptp(o[m, 1])); park = np.degrees(np.abs(o[m, 1]).mean())
            cells.append(f"{pp:9.1f} / {park:5.1f} deg")
        print(f"{name:24s} | " + " | ".join(f"{c:>22s}" for c in cells))
    for k, v in base.items():
        setattr(M, k, v)


if __name__ == "__main__":
    run()
