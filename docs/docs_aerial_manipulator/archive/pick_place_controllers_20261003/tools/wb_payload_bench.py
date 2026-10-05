#!/usr/bin/env python3
"""Offline bench for the WHOLE-BODY law's pick-and-place stability (2026-10-03,
user request: "optimize the stability for the whole-body controller as well").

    OMP_NUM_THREADS=1 /usr/bin/python3 wb_payload_bench.py [--set k_R=1.8 ...] [--delay 16]

circle_bench's mirror plant (the exact 4-D law, hardware-like feedback, rotor lag,
transport delay, arm friction) hovering at the pick / place pose [0, -20, 30, 0]
with the CoM-anchored EE reference -- and THE PAYLOAD: at +10 s the 200 g box
is added rigidly to the gripper link (its CG 0.2225 m beyond the claw, the
measured grasp), at +25 s it is removed. The law knows nothing about it (the
payload is model uncertainty, by design), exactly as at the lift and the release.

Isaac (wb_1..4) shows quiet-hold tilt ripple 0.8-2.2 deg p-p at 1.0-1.4 Hz and
PITCH kicks of 2.7-4.9 deg p-p at the load (lift) and the release; this scores
the same: the quiet ripple, the load and unload kicks (peak tilt deviation, EE
error peak), and how long each takes to settle.
"""
import argparse
import math
import multiprocessing as mp
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ARCH = os.path.abspath(os.path.join(HERE, "..", ".."))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", "..", "..", ".."))
for p_ in (os.path.join(ARCH, "sim2real_tuning_20260926", "tools"),
           os.path.join(REPO, "application", "robotic_arm", "utils"),
           os.path.join(REPO, "extensions", "fsc_aerial_manipulation"),
           os.path.join(ARCH, "circle_tune_20260927", "tools"),
           os.path.join(ARCH, "pick_place_tune_20261001", "tools")):
    sys.path.insert(0, p_)
import circle_bench as CB          # noqa: E402
import hover_anchor_screen as HS   # noqa: E402

# the pick-and-place yaml's gains: H1b with k_R / k_w 1.6 / 1.2
SHIPPED = dict(HS.H1B, k_R=1.6, k_w=1.2)
POSE = (0.0, -20.0, 30.0, 0.0)
M_PAYLOAD = 0.2
PAYLOAD_IN_LINK4 = np.array([0.0, 0.0, -0.108 - 0.2225])   # the claw + the box CG below it
T_LOAD, T_UNLOAD, T_END = 10.0, 25.0, 40.0                 # after the run start
DT = 0.004                                                 # the law tick (250 Hz), RTF 1: plant step = law step


def loaded(params):
    """The plant with the box rigidly in the gripper (point mass, parallel axis)."""
    q = dict(params)
    m_i = list(q["m_i"]); com_i = [np.asarray(c, float) for c in q["com_i"]]; I_i = [np.asarray(I, float) for I in q["I_i_i"]]
    m, c, I = m_i[4], com_i[3], I_i[4]
    m2 = m + M_PAYLOAD
    c2 = (m * c + M_PAYLOAD * PAYLOAD_IN_LINK4) / m2

    def shift(mass, r):
        return mass * (np.dot(r, r) * np.eye(3) - np.outer(r, r))
    I2 = I + shift(m, c - c2) + shift(M_PAYLOAD, PAYLOAD_IN_LINK4 - c2)
    m_i[4] = m2; com_i[3] = c2; I_i[4] = I2
    q["m_i"] = m_i; q["com_i"] = com_i; q["I_i_i"] = I_i
    return q


class _Proxy:
    """Stands in for circle_bench's controller module: the PLANT's dynamics
    (called exactly once per step, after one call at init) switch to the
    payload-carrying plant inside [t_load, t_unload)."""

    def __init__(self, C, t_start, dtp, t_load, t_unload):
        self._C, self.t_start, self.dtp = C, t_start, dtp
        self.t_load, self.t_unload = t_load, t_unload
        self.plant_id, self.plant_loaded, self.calls = None, None, 0

    def __getattr__(self, k):
        return getattr(self._C, k)

    def dynamics(self, X, params):
        if params.get("_plant_marker", False):
            if self.plant_id != id(params):
                self.plant_id, self.plant_loaded, self.calls = id(params), loaded(params), 0
            t = self.t_start + (self.calls - 1) * self.dtp
            self.calls += 1
            if self.t_load <= t < self.t_unload:
                return self._C.dynamics(X, self.plant_loaded)
        return self._C.dynamics(X, params)


def run(p, seed=0, delay_ms=16.0, t_settle=8.0):
    S = HS.static_stream(q_deg=POSE, x_b=(0.87, 1.0, 1.77), yaw_deg=0.0, T=T_END + 20.0)
    t_run0 = S["run"][0]
    S["run"] = (t_run0, t_run0 + T_END)
    orig_C, orig_pp, orig_case = CB.C, CB.ES.plant_params, CB.ES.Case

    def plant_params(model, case):
        pp = orig_pp(model, case)
        pp["_plant_marker"] = True
        return pp

    def case(**k):
        k["ee_ref"] = "relative"
        return orig_case(**k)
    proxy = _Proxy(orig_C, t_run0 - t_settle, DT,
                   t_run0 + T_LOAD, t_run0 + T_UNLOAD)
    CB.C, CB.ES.plant_params, CB.ES.Case = proxy, plant_params, case
    try:
        r = CB.simulate(p, S, seed=seed, delay_ms=delay_ms, t_settle=t_settle, keep=True)
    finally:
        CB.C, CB.ES.plant_params, CB.ES.Case = orig_C, orig_pp, orig_case
    return r, t_run0


def score(r, t_run0):
    out = dict(verdict=r["verdict"])
    if r["verdict"] != "completed":
        return out
    H = r["H"]; t = H["t"] - t_run0
    tilt, ee = H["tilt"], H["ee"] * 1e3

    def win(a, b):
        return (t >= a) & (t < b)
    q = win(2.0, T_LOAD)
    out["ripple_pp"] = float(np.ptp(tilt[q]))
    out["ee_quiet"] = float(np.sqrt(np.mean(ee[q] ** 2)))
    base = float(np.mean(tilt[q]))
    for nm, t0 in (("load", T_LOAD), ("unload", T_UNLOAD)):
        w = win(t0, t0 + 6.0)
        out[f"{nm}_tilt"] = float(np.max(np.abs(tilt[w] - base)))
        out[f"{nm}_ee"] = float(np.max(ee[w]))
        # settle: the last time the EE error exceeds 2x the quiet rms in the window
        thr = max(2.0 * out["ee_quiet"], 3.0)
        over = np.where(ee[w] > thr)[0]
        out[f"{nm}_settle"] = float(t[w][over[-1]] - t0) if len(over) else 0.0
    q2 = win(T_UNLOAD + 8.0, T_END)
    out["ripple_after"] = float(np.ptp(tilt[q2]))
    return out


def job(a):
    name, p, seed, delay = a
    r, t0 = run(p, seed=seed, delay_ms=delay)
    return name, seed, delay, score(r, t0)


def fmt(name, rows):
    ok = [r for r in rows if r["verdict"] == "completed"]
    if len(ok) < len(rows):
        return f"{name:34s} {len(rows) - len(ok)}/{len(rows)} ABORT"
    m = {k: np.mean([r[k] for r in ok]) for k in ok[0] if k != "verdict"}
    return (f"{name:34s} ripple {m['ripple_pp']:4.2f} deg  EE {m['ee_quiet']:4.1f} mm | load: tilt {m['load_tilt']:4.2f} "
            f"EE {m['load_ee']:5.1f} mm settle {m['load_settle']:4.1f} s | unload: tilt {m['unload_tilt']:4.2f} "
            f"EE {m['unload_ee']:5.1f} mm settle {m['unload_settle']:4.1f} s | ripple after {m['ripple_after']:4.2f}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--set", nargs="*", default=[])
    ap.add_argument("--delay", type=float, default=16.0)
    ap.add_argument("--seeds", type=int, default=2)
    a = ap.parse_args()
    p = dict(SHIPPED)
    for kv in a.set:
        k, v = kv.split("=")
        p[k] = float(v)
    jobs = [("candidate", p, s, a.delay) for s in range(a.seeds)]
    with mp.Pool(len(jobs)) as pool:
        R = pool.map(job, jobs)
    print(fmt("candidate " + " ".join(a.set), [r[3] for r in R]))


if __name__ == "__main__":
    main()
