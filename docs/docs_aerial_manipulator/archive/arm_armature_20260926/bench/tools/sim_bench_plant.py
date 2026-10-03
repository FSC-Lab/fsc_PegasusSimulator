#!/usr/bin/env python3
"""Offline arm sim (../../arm_stiffness_sim.py: exact law, 4-D hardware config, stick-slip friction,
Present Velocity 48 ms / 0.024 rad/s, flight 3's arm design) with the PLANT set to the bench-measured
armature instead of the 0.020 guess.

    PYTHONNOUSERSITE=1 /usr/bin/python3 sim_bench_plant.py <plant> <law> [present|observer]

plants (both reproduce every measured row total: j1 0.010, j2 0.0194, j3 0.0097, j4 0.0097):
    hht   J_arm 0.0097 h h^T on each link (the calibrated law structure, j2-j3 coupled)
    diag  joint-diagonal [0.010, 0.0194, 0.0097, 0.0097] (same rows, no j2-j3 coupling)
laws:
    flown    J_arm 0.020, K_y 20 / D_y 12            (as flown 0918-0924)
    cal      J_arm 0.0097, K_y 20 / D_y 12            (change J_arm only)
    cal33    J_arm 0.0097, K_y 33 / D_y 20            (J_arm changed, task gains x1.65 = same joint gains as flown)
    cal50    J_arm 0.0097, K_y 50 / D_y 20
    cal100   J_arm 0.0097, K_y 100 / D_y 28
    dcal     joint-DIAGONAL model [0.010, 0.0194, 0.0097, 0.0097], K_y 20 / D_y 12
    dcal50   same, K_y 50 / D_y 20
    D<a>_<b> joint-DIAGONAL bench model, K_y a / D_y b (the adopted structure)
    J<v>     h h^T J_arm = v (a TUNED value), K_y 20 / D_y 12
    K<a>_<b> h h^T J_arm 0.0097, K_y a / D_y b (the joint-space equivalent of a tuned J_arm)
"""
import os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", ".."))
import arm_stiffness_sim as AS

JCAL = 0.0097
PLANTS = {"hht": ("hht", JCAL), "diag": ("diagv", [0.010, 0.0194, 0.0097, 0.0097]),
          # the calibration's own uncertainty band (0.0077..0.0120 of 0.0097)
          "diag075": ("diagv", [0.75 * v for v in (0.010, 0.0194, 0.0097, 0.0097)]),
          "diag130": ("diagv", [1.30 * v for v in (0.010, 0.0194, 0.0097, 0.0097)])}
LAWS = {"flown": dict(J_model=AS.JA0),
        "cal": dict(J_model=JCAL),
        "cal33": dict(J_model=JCAL, gains=dict(ky=(33.0, 33.0, 33.0, 0.3), dy=(20.0, 20.0, 20.0, 0.3))),
        "cal50": dict(J_model=JCAL, gains=dict(ky=(50.0, 50.0, 50.0, 0.3), dy=(20.0, 20.0, 20.0, 0.3))),
        "cal100": dict(J_model=JCAL, gains=dict(ky=(100.0, 100.0, 100.0, 0.3), dy=(28.0, 28.0, 28.0, 0.3))),
        # law model = the bench rows placed JOINT-DIAGONALLY (armature_diag hook, not the flown structure)
        "dcal":   dict(model_kind="diag", J_model=-2.0),
        "dcal50": dict(model_kind="diag", J_model=-2.0, gains=dict(ky=(50.0, 50.0, 50.0, 0.3), dy=(20.0, 20.0, 20.0, 0.3)))}
DIAG_BENCH = [0.010, 0.0194, 0.0097, 0.0097]

if __name__ == "__main__":
    pn, ln = sys.argv[1], sys.argv[2]
    vel = sys.argv[3] if len(sys.argv) > 3 else "present"     # present | observer
    AS.VEL_SOURCE = vel
    # optional 4th arg "<law ms>+<cmd ms>": arm-channel transport delays
    dly = sys.argv[4] if len(sys.argv) > 4 else "0+0"
    AS.LAW_ARM_DELAY_S, AS.ARM_CMD_DELAY_S = (float(x) / 1000.0 for x in dly.split("+"))
    kind, val = PLANTS[pn]
    orig = AS.model_params
    SENT = -1.0

    def patched(k, J=AS.JA0):
        if k == "diag" and J == SENT:                       # simulate() builds its plant this way
            if kind == "hht":
                return orig("hht", val)
            L = orig("none"); L["armature_diag"] = list(val); return L
        if k == "diag" and J == -2.0:                       # the law's joint-diagonal bench model
            L = orig("none"); L["armature_diag"] = list(DIAG_BENCH); return L
        return orig(k, J)
    AS.model_params = patched
    if ln.startswith("J"):                                   # J<value>: h h^T J_arm inflated, K_y 20 / D_y 12
        kw = dict(J_model=float(ln[1:]))
    elif ln.startswith("D"):                                 # D<ky>_<dy>: joint-diagonal bench model
        ky, dy = (float(x) for x in ln[1:].split("_"))
        kw = dict(model_kind="diag", J_model=-2.0,
                  gains=dict(ky=(ky, ky, ky, 0.3), dy=(dy, dy, dy, 0.3)))
    elif ln.startswith("K"):                                 # K<ky>_<dy>: calibrated J_arm, scaled task gains
        ky, dy = (float(x) for x in ln[1:].split("_"))
        kw = dict(J_model=JCAL, gains=dict(ky=(ky, ky, ky, 0.3), dy=(dy, dy, dy, 0.3)))
    else:
        kw = dict(LAWS[ln])
    kw.setdefault("model_kind", "hht")
    o, H = AS.simulate(f"law {ln:10s} | plant {pn:7s} | qdot {vel:8s} | dly {dly:5s}", J_true=SENT, **kw)
    print(AS.fmt(o), flush=True)
    tag = ("" if vel == "present" else "_obs") + ("" if dly == "0+0" else "_d" + dly.replace("+", "p"))
    np.savez_compressed(os.path.join(HERE, "..", f"sim_{pn}_{ln}{tag}.npz"), **H)
