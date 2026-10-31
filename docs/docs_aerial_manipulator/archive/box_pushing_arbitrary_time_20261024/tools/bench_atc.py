#!/usr/bin/env python3
"""bench_atc.py -- offline closed-loop bench for the arbitrary-convergence-time
quadrotor law (arbitrary_time_control.py, the parity-locked reference of the C++
node) before Isaac flights.

Plant: a rigid body (the rig's mass and inertia) with the MN4010 rotor lag on the
delivered thrust and body torque (first order, lambda = 10.0265 1/s), the mirror
plant's constant disturbances (body-fixed force bias (+0.55, -0.50) N, yaw torque
-0.095 N.m, the allocator's thrust belief 3.7 % low -> the plant delivers x1.037),
a feedback delay of N ticks on every measured signal and white mocap velocity
noise. The arm is not in this bench (Isaac tests the coupling).

Scenario: DIRECT entry at t = 0 from a hover 0.05 m off the reference, a 0.3 m
x-step at t = 10 s, a 90 deg yaw step at t = 20 s.

    PYTHONNOUSERSITE=1 /usr/bin/python3 bench_atc.py [--delay-ms 16] [--set key=val ...]
"""
import argparse
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, *[".."] * 5))
sys.path.insert(0, os.path.join(REPO, "extensions", "fsc_aerial_manipulation"))
from fsc_aerial_manipulation.robotic_arm.utils_controller import arbitrary_time_control as A  # noqa: E402

DT = 0.004
LAM = 10.0265


def run(delay_ms=16.0, noise=0.022, T=30.0, qc=None, fc=None, seed=0, verbose=False):
    rng = np.random.default_rng(seed)
    qc = qc or A.QuadConfig()
    fz = A.TskFuzzyEstimator(fc or A.FuzzyConfig())
    win = A.Window(qc.DT, qc.floor)
    win.trigger(0.0)
    m, J = qc.mass, qc.inertia
    p = np.array([0.05, -0.03, 1.25])
    v = np.zeros(3)
    R = A.euler_to_r([0.0, 0.0, 0.0])
    om = np.zeros(3)
    T_act = m * A.G_ACC
    tau_act = np.zeros(3)
    nd = int(round(delay_ms / 1000.0 / DT))
    buf = []
    x1 = x2 = 0.0
    log = []
    for k in range(int(T / DT)):
        t = k * DT
        p_d = np.array([0.0, 0.0, 1.3]) + (np.array([0.3, 0.0, 0.0]) if t >= 10.0 else 0.0)
        yaw_d = math.radians(90.0) if t >= 20.0 else 0.0
        buf.append((p.copy(), v.copy() + rng.normal(0.0, noise, 3), R.copy(), om.copy()))
        pm, vm, Rm, omm = buf[max(0, len(buf) - 1 - nd)]
        D, ph, X = fz.evaluate(x1, x2)
        res = A.quad_law(qc, win, t, pm, vm, Rm, omm, p_d, np.zeros(3), np.zeros(3), yaw_d, D,
                         np.zeros(3), np.zeros(3))
        fz.adapt(DT, res.g, ph, X)
        x1, x2 = res.x1, res.x2
        a = math.exp(-LAM * DT)
        T_act = res.thrust * 1.037 + (T_act - res.thrust * 1.037) * a
        tau_act = res.torque + (tau_act - res.torque) * a
        F = T_act * R[:, 2] - m * A.G_ACC * A.E3 + R @ np.array([0.55, -0.50, 0.0])
        v = v + DT * F / m
        p = p + DT * v
        tq = tau_act + np.array([0.0, 0.0, -0.095])
        om = om + DT * np.linalg.solve(J, tq - np.cross(om, J @ om))
        n = np.linalg.norm(om)
        if n > 1e-12:
            R = R @ A.axis_rot(om / n, n * DT)
        eta = A.euler_zyx(R)
        log.append([t, *(p - p_d), math.degrees(math.acos(max(-1, min(1, R[2, 2])))),
                    math.degrees(A.wrap(eta[2] - yaw_d)), fz.n_rules])
        if not np.all(np.isfinite(p)) or np.linalg.norm(p - p_d) > 5.0:
            break
    return np.array(log)


def score(L, label):
    def win(a, b):
        return L[(L[:, 0] >= a) & (L[:, 0] < b)]
    ent = win(0, 10)
    stp = win(10, 20)
    yaw = win(20, 30)
    ok = L[-1, 0] > 29.0
    hold = win(7, 10)
    print(f"{label:28s} {'OK ' if ok else 'DIVERGED'} entry peak {1e3 * np.abs(ent[:, 1:4]).max():6.1f} mm, "
          f"hold rms {1e3 * np.sqrt((hold[:, 1:4] ** 2).sum(1).mean()):5.1f} mm, tilt max {ent[:, 4].max():5.2f} deg | "
          f"x-step overshoot {1e3 * max(0.0, stp[:, 1].max()):5.1f} mm, settle err {1e3 * abs(stp[-200:, 1]).mean():5.1f} mm | "
          f"yaw-step peak pos err {1e3 * np.abs(yaw[:, 1:4]).max():6.1f} mm, yaw err end {abs(yaw[-1, 5]):5.2f} deg, "
          f"rules {int(L[-1, 6])}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--delay-ms", type=float, nargs="*", default=[0.0, 16.0, 32.0, 48.0])
    ap.add_argument("--set", nargs="*", default=[], help="QuadConfig field=value (eta1/eta2 as 6 comma values)")
    ap.add_argument("--c", default=None, help="TSKFE adaptation gains c, 6 comma values")
    a = ap.parse_args()
    qc = A.QuadConfig()
    for kv in a.set:
        k, v = kv.split("=")
        cur = getattr(qc, k)
        setattr(qc, k, np.array([float(x) for x in v.split(",")]) if isinstance(cur, np.ndarray) else float(v))
    fc = A.FuzzyConfig(c=np.array([float(x) for x in a.c.split(',')])) if a.c else None
    for d in a.delay_ms:
        score(run(d, qc=qc, fc=fc), f"delay {d:.0f} ms")


if __name__ == "__main__":
    main()
