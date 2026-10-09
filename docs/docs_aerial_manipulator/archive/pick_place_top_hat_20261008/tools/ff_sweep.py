#!/usr/bin/env python3
"""Friction feed-forward variants on the offline arm sim, current hardware law (2026-10-08).

    PYTHONNOUSERSITE=1 /usr/bin/python3 ff_sweep.py [case ...]

arm_armature_20260926/arm_stiffness_sim.py with its 2026-10-08 FF_SOURCE option: exact law,
4-D hardware config at K_y 211.9 / D_y 26.82 (H1b), joint-diagonal bench armature, the arm's
velocity observer as the law's qdot, arm transport 8 + 8 ms, stick-slip friction at the flight-
identified ratios, flight 3's arm design (q2 25 +- 15 deg, 12 s). Only the friction feed-forward's
velocity source / width / scale change between cases. Scored like the campaign (q2 / q3 error
rms, stuck share, overshoot). ORDINAL evidence only: this sim under-predicts the flown joint
error 3-5x (bench/tune/README.md: 0.3 / 0.8 deg at K_y 200 vs 1.7 / 1.4 flown).
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ARM = os.path.join(HERE, "..", "..", "arm_armature_20260926")
sys.path.insert(0, ARM); sys.path.insert(0, os.path.join(ARM, "bench", "tools"))
import arm_stiffness_sim as AS                                        # noqa: E402

DIAG_BENCH = [0.010, 0.0194, 0.0097, 0.0097]
SENT = -1.0
HW_SCALE = np.array([1.0, 0.70, 0.65, 1.0])
CASES = {
    "ref_0924":     dict(src="reference", w=0.015, scale=np.ones(4)),      # as flown 0918-0924
    "meas_hw":      dict(src="measured", w=0.03, scale=HW_SCALE),          # the hardware since 2026-09-28
    "meas_s1":      dict(src="measured", w=0.03, scale=np.ones(4)),
    "meas_w015":    dict(src="measured", w=0.015, scale=HW_SCALE),
    "meas_w06":     dict(src="measured", w=0.06, scale=HW_SCALE),
    "blend30":      dict(src="blend", w=0.03, scale=HW_SCALE, blend=0.3),
    "blend60":      dict(src="blend", w=0.03, scale=HW_SCALE, blend=0.6),
}


def main():
    names = sys.argv[1:] or list(CASES)
    orig = AS.model_params

    def patched(k, J=AS.JA0):
        if k == "diag" and J == SENT:
            L = orig("none"); L["armature_diag"] = list(DIAG_BENCH); return L
        if k == "diag" and J == -2.0:
            L = orig("none"); L["armature_diag"] = list(DIAG_BENCH); return L
        return orig(k, J)
    AS.model_params = patched
    AS.VEL_SOURCE = "observer"
    AS.LAW_ARM_DELAY_S, AS.ARM_CMD_DELAY_S = 0.008, 0.008
    ky, dy = 211.9, 26.82
    for nm in names:
        c = CASES[nm]
        AS.FF_SOURCE, AS.FF_W_MEAS, AS.FF_SCALE, AS.FF_BLEND = c["src"], c["w"], np.asarray(c["scale"], float), c.get("blend", 0.3)
        o, H = AS.simulate(f"FF {nm:10s}", J_true=SENT, model_kind="diag", J_model=-2.0,
                           gains=dict(ky=(ky, ky, ky, 0.3), dy=(dy, dy, dy, 0.3)))
        print(AS.fmt(o), flush=True)
        e = np.degrees(H["q"] - H["qref"])
        hold = H["t"] < 2.0
        print(f"   hold-offset j2/j3 {e[hold, 1].mean():+.2f}/{e[hold, 2].mean():+.2f} deg; "
              f"moving rms j2/j3 {np.sqrt((e[~hold, 1] ** 2).mean()):.2f}/{np.sqrt((e[~hold, 2] ** 2).mean()):.2f} deg; "
              f"stuck j2/j3 {100 * H['stuck'][:, 1].mean():.0f}/{100 * H['stuck'][:, 2].mean():.0f} %; "
              f"cap {100 * H['cap'].mean():.1f} %", flush=True)


if __name__ == "__main__":
    main()
