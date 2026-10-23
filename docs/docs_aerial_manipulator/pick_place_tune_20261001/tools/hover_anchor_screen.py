#!/usr/bin/env python3
"""Offline screen: the 4-D whole-body law HOVERING at the pick pose, CoM- vs
WORLD-anchored EE reference, on circle_bench's mirror plant (exact law, the
hardware-like feedback model, rotor lag, 16 ms transport, arm friction).

    /usr/bin/python3 hover_anchor_screen.py [--anchor world|relative] [--set ky_psi=0.6 ...]

Why: pick-and-place run 2 (runs/pnp_r2) flew the world anchor with the H1b
gains and diverged in plain hover 12 s after DIRECT entry -- the EE heading
error grew at ~0.4 Hz to a tilt trip. The CoM anchor the gains were tuned on
never excites it. This reproduces that offline and screens gains before an
Isaac flight. The stream is a static rest at the pick pose (the planner's own
rest_ref at the execute_pick goal), so only the regulation loop is tested.
"""
import argparse
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", "..", ".."))
sys.path.insert(0, os.path.join(REPO, "docs", "docs_aerial_manipulator", "circle_tune_20260927", "tools"))
import circle_bench as CB                                   # noqa: E402

TP = CB.TP
H1B = dict(k_x=50.03, k_v=12.58, k_R=2.134, k_w=1.567, mrd_s=1.0710, ky=211.9, dy=26.82,
           ky_psi=0.2484, dy_psi=0.2903, omega_c_t=2.927, omega_c_r=0.7428, omega_c_q=0.8479,
           omega_x=0.2072, omega_q=0.2, dls=0.3)


def static_stream(q_deg=(0, -20, 20, 0), x_b=(0.929, 0.916, 1.571), yaw_deg=50.7, T=80.0):
    model = TP.make_params_t650(armature_diag=CB.CS.J_ARM)
    model["base_com"] = CB.R_MODEL.T @ np.array([-0.017854, 0.0, 0.0])
    r = TP.rest_ref(model, np.array(x_b), math.radians(yaw_deg) - 0.5 * math.pi, np.radians(q_deg))
    t = np.arange(0.0, T, 0.01)
    S = {"t": t, "name": "static_pick"}
    for k in ("x_cd", "x_cd_dot", "x_cd_ddot", "x_cd_d3", "x_cd_d4", "b1_d", "b1_d_dot", "b1_d_ddot",
              "r_ed", "r_ed_dot", "r_ed_ddot", "b1_de", "b1_de_dot", "b1_de_ddot"):
        S[k] = np.tile(np.asarray(r[k], float), (len(t), 1))
    S["q_d"] = np.tile(r["q_d"], (len(t), 1)); S["qdot_d"] = np.zeros((len(t), 4))
    S["run"] = (10.0, T - 5.0)
    return S


def run(p, anchor, T=60.0, seed=0, pose=(0, -20, 20, 0), **kw):
    orig = CB.ES.Case

    def case(**k):
        k["ee_ref"] = anchor
        return orig(**k)
    CB.ES.Case = case
    try:
        return CB.simulate(p, static_stream(q_deg=pose, T=T), seed=seed, **kw)
    finally:
        CB.ES.Case = orig


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--anchor", default="world")
    ap.add_argument("--T", type=float, default=60.0)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--delay-ms", type=float, default=16.0)
    ap.add_argument("--set", nargs="*", default=[])
    ap.add_argument("--pose", type=float, nargs=4, default=[0, -20, 20, 0], help="hold pose [deg]")
    a = ap.parse_args()
    p = dict(H1B)
    for kv in a.set:
        k, v = kv.split("=")
        p[k] = float(v)
    r = run(p, a.anchor, T=a.T, seed=a.seed, pose=a.pose, delay_ms=a.delay_ms)
    keys = ("verdict", "t_end", "ee_rms", "ee_pk", "com_rms", "head_rms", "eR_mean", "tilt_pk", "tau_pk", "sat_pct")
    print(f"anchor={a.anchor} pose={a.pose} {' '.join(a.set) or 'H1b'}: " +
          " ".join(f"{k}={r[k]:.4g}" if isinstance(r.get(k), float) else f"{k}={r.get(k)}" for k in keys if k in r))


if __name__ == "__main__":
    main()
