#!/usr/bin/env python3
"""modular_bench.py -- the modular adaptive law (Yadav et al. TMECH 2025) on the
SAME offline circle bench the whole-body law was tuned on (2026-09-30).

circle_tune_20260927/tools/circle_bench.py is reused UNCHANGED: its plant
(the mirror plant identified from the 0918/0921/0924 flights -- rotor lag,
allocator kf/km mismatch, body-fixed force/torque bias, arm friction + the
arm-side friction feed-forward, joint stops, transport delay), its feedback
realism (fused-odometry lag/noise, attitude/gyro noise, encoder quantisation,
the 12 ms arm velocity observer) and its scoring on the TRUE state. The only
thing swapped is the law: circle_bench.simulate() builds `ES.Law(model, cfg,
case)` per run, and run() below points that name at an adapter around
modular_adaptive.ModularAdaptiveLaw for the duration of the call.

The adapter does exactly what the C++ node does at its boundary: model-frame
state -> actual FLU/ENU measurements, the streamed MODEL-frame reference ->
ReferenceConverter, and the law's actual-frame body moment back to the model
frame for the plant's allocator.
"""
import contextlib
import copy
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", "..", ".."))
sys.path.insert(0, os.path.join(REPO, "docs", "docs_aerial_manipulator", "circle_tune_20260927", "tools"))
sys.path.insert(0, os.path.join(REPO, "extensions", "fsc_aerial_manipulation"))
import circle_bench as CB                                                          # noqa: E402
from fsc_aerial_manipulation.robotic_arm.utils_controller import modular_adaptive as MA  # noqa: E402

R_MODEL = MA.R_MODEL
N = 4


class Adapter:
    def __init__(self, mcfg, model, dt):
        self.law = MA.ModularAdaptiveLaw(copy.deepcopy(mcfg))
        self.rc = MA.ReferenceConverter(model, qdd_filter_hz=mcfg_qdd_hz(mcfg))
        self.prev_sig = None
        self.last = None

    def __call__(self, Xm, dyn, ref, dt):
        n = N
        p = Xm[0:3]
        Rm = Xm[3:12].reshape(3, 3, order="F")
        q = Xm[12:12 + n]
        v_body_m = Xm[12 + n:15 + n]
        w_body_m = Xm[15 + n:18 + n]
        qd = Xm[18 + n:18 + 2 * n]
        R = Rm @ R_MODEL.T                       # actual attitude
        meas = dict(p=p, v=Rm @ v_body_m, R=R, Omega=R_MODEL @ w_body_m, q=q, qdot=qd)
        sig = np.concatenate([ref["x_cd"], ref["qdot_d"], ref["q_d"]])
        changed = self.prev_sig is None or not np.array_equal(sig, self.prev_sig)
        self.prev_sig = sig
        rc = self.rc.convert(ref, dt, changed)
        out = self.law.step(meas, rc, dt)
        self.last = (out, rc)
        return dict(u1=out["u1"], tau_body=R_MODEL.T @ out["tau_q"], tau_joint=out["tau_a"],
                    e_R=out["e_q"], n_sat=out["n_sat"], R0c=out["Rd"] @ R_MODEL,
                    omega_0c=np.zeros(3))


def mcfg_qdd_hz(mcfg):
    return getattr(mcfg, "qdd_filter_hz", 10.0)


@contextlib.contextmanager
def _patched(mcfg):
    orig = CB.ES.Law
    CB.ES.Law = lambda model, cfg, case: Adapter(mcfg, model, case.dt)
    try:
        yield
    finally:
        CB.ES.Law = orig


def run(mcfg, stream="r050_L24_f55_a15_p6", **kw):
    """One bench flight of the modular law. kw -> circle_bench.simulate (profile,
    delay_ms, fb, seed, t_settle, gust, rtf, keep, full ...)."""
    with _patched(mcfg):
        return CB.simulate(dict(CB.BASE), CB.stream(stream), **kw)


def run_wb(p=None, stream="r050_L24_f55_a15_p6", **kw):
    """The whole-body law on the same bench (p = circle_bench gain dict; None = H1b)."""
    return CB.simulate(dict(p or H1B), CB.stream(stream), **kw)


# The 2026-09-27 shipped whole-body tune (hardware + _sim yamls), circle_bench keys.
H1B = dict(k_x=50.03, k_v=12.58, k_R=2.134, k_w=1.567, mrd_s=1.071,
           ky=211.9, dy=26.82, ky_psi=0.2484, dy_psi=0.2903,
           omega_c_t=2.927, omega_c_r=0.7428, omega_c_q=0.8479, omega_x=0.2072, omega_q=0.2, dls=0.3)


if __name__ == "__main__":
    import time
    t0 = time.time()
    r = run(MA.table2_config())
    print(CB.fmt("modular, Table II literal", r), f"[{time.time() - t0:.0f} s]")
