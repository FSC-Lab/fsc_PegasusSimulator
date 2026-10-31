#!/usr/bin/env python3
"""bench_arm.py -- the arm half of the arbitrary-time law (ACTNDO + admittance +
eqs 36-38, arbitrary_time_control.py) on a simulated arm, to separate an arm-loop
instability from the arm-base coupling.

Plant: the arm's dynamics with the PLANT's armature (Isaac authors 0.020 kg m^2 on
every joint, the model carries 0.010/0.0194/0.0097/0.0097), optionally on a base
that is FREE IN YAW (inertia Ib about the joint-1 axis, the joint-1 torque reacts
on it, a yaw PD with the rotor lag holds it), the joint velocity the law sees
through the arm controller's 20 Hz / zeta 1 velocity observer, one tick of
transport delay.

    PYTHONNOUSERSITE=1 /usr/bin/python3 bench_arm.py [--yaw] [--no-obs] [--ff 1.0]
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
G0 = np.array([0.0, 0.0, -A.G_ACC])


def run(yaw=False, obs_on=True, ff=1.0, plant_arm=0.020, T=20.0, wv=2 * math.pi * 20.0, delay=1,
        acfg=None, ocfg=None, verbose=False, act_delay=0):
    ac = acfg or A.ArmConfig()
    oc = ocfg or A.ActndoConfig()
    model_arm = A.ARMATURE.copy()
    q_d = np.array([0.0, 0.558, 0.489, 0.0])
    q = q_d + np.array([0.01, 0.0, 0.0, 0.0])
    qd = np.zeros(4)
    v_est = np.zeros(4)
    x1 = q_d + np.array([0.01, 0.0, 0.0, 0.0])
    x2 = np.zeros(4)
    adm = A.ArmAdmittance(ac)
    win = A.Window(ac.DT, ac.floor)
    win.trigger(0.0)
    obs = A.Actndo(oc)
    obs.reset(np.zeros(4))
    obs.trigger(0.0)
    Ib = 0.131                 # base yaw inertia
    psi = 0.0
    r = 0.0                    # base yaw rate
    tau_yaw_act = 0.0
    tau_prev = np.zeros(4)
    act_buf = []
    hist = []
    buf = []
    rng = np.random.default_rng(1)
    for k in range(int(T / DT)):
        t = k * DT
        buf.append((q.copy(), v_est.copy(), r))
        qm, vm, rm = buf[max(0, len(buf) - 1 - delay)]
        A.ARMATURE[:] = model_arm
        M, C, G = A.arm_dynamics(qm, vm, G0)
        tau_hat = obs.update(DT, t, M @ vm, C.T @ vm, G, tau_prev) if obs_on else np.zeros(4)
        e, ed, edd = adm.step(DT, tau_hat, q_d)
        tau, _, _ = A.arm_law(ac, win, t, qm, vm, M, C, G, tau_hat, q_d + e, ed, edd)
        tau_prev = tau
        act_buf.append(tau.copy())
        tau_applied = act_buf[max(0, len(act_buf) - 1 - act_delay)]
        # base yaw: PD on psi with the rotor lag, minus the J_E feed-forward (~ -tau_hat_1)
        tau_yaw_cmd = -3.4 * psi - 1.31 * rm - ff * tau_hat[0]
        tau_yaw_cmd = float(np.clip(tau_yaw_cmd, -0.55, 0.55))
        tau_yaw_act = tau_yaw_cmd + (tau_yaw_act - tau_yaw_cmd) * math.exp(-10.0265 * DT)
        # plant (+ the current-loop residual: ~22 mN.m rms per joint, white here)
        noise_tau = rng.normal(0.0, 0.022, 4)
        A.ARMATURE[:] = plant_arm
        Mp, Cp, Gp = A.arm_dynamics(q, qd, G0)
        A.ARMATURE[:] = model_arm
        if yaw:
            # joint 1 rotates the arm about the base's z: absolute arm yaw accel = qdd1 + rdot;
            # the base sees -tau1 (reaction). Approximate the coupling on joint 1 only.
            rdot = (tau_yaw_act - tau_applied[0]) / Ib
            qdd = np.linalg.solve(Mp, tau_applied + noise_tau - Cp @ qd - Gp)
            qdd[0] -= rdot          # relative acceleration
            r += DT * rdot
            psi += DT * r
        else:
            qdd = np.linalg.solve(Mp, tau_applied + noise_tau - Cp @ qd - Gp)
        qd = qd + DT * qdd
        q = q + DT * qd
        # the arm controller's velocity observer (2nd order, zeta 1, bandwidth wv) on q:
        # x1' = x2 + 2 zeta w (q - x1), x2' = w^2 (q - x1); the law sees x2
        err = q - x1
        x1 = x1 + DT * (x2 + 2.0 * wv * err)
        x2 = x2 + DT * (wv * wv * err)
        v_est = x2.copy()
        hist.append([t, *np.degrees(q - q_d), *tau, *tau_hat, psi])
    return np.array(hist)


def report(H, label):
    late = H[H[:, 0] > H[-1, 0] - 5.0]
    print(f"{label:40s} joint err std (last 5 s) {np.round(late[:, 1:5].std(0), 3)} deg, "
          f"tau std {np.round(late[:, 5:9].std(0), 3)}, |tau_hat| max {np.abs(late[:, 9:13]).max():.3f}, "
          f"base yaw std {math.degrees(late[:, 13].std()):.3f} deg")


def main():
    import itertools
    ap = argparse.ArgumentParser()
    ap.add_argument("--sense", type=int, nargs="*", default=[2], help="sensing delay [ticks] (observer lag is modelled separately)")
    ap.add_argument("--act", type=int, nargs="*", default=[9], help="actuation delay [ticks] (Isaac: 9 = 36 ms measured)")
    ap.add_argument("--floor", type=float, nargs="*", default=[0.6, 0.9, 1.2])
    ap.add_argument("--k", type=float, nargs="*", default=[8.2, 4.0, 2.0])
    ap.add_argument("--DT", type=float, default=None, help="arm window DT_m (default: 2.5 x floor)")
    ap.add_argument("--T", type=float, default=12.0)
    a = ap.parse_args()
    for sd, ad, fl, k in itertools.product(a.sense, a.act, a.floor, a.k):
        ac = A.ArmConfig(DT=a.DT if a.DT else 2.5 * fl, floor=fl)
        oc = A.ActndoConfig(k=k)
        H = run(yaw=True, obs_on=True, ff=1.0, delay=sd, act_delay=ad, acfg=ac, ocfg=oc, T=a.T)
        early = H[(H[:, 0] > 3.0) & (H[:, 0] < 6.0)]
        late = H[H[:, 0] > H[-1, 0] - 3.0]
        g = late[:, 1:5].std(0) / np.maximum(early[:, 1:5].std(0), 1e-6)
        report(H, f"sense {sd} act {ad} floor {fl} k {k} | growth x{np.round(g, 1)}")


if __name__ == "__main__":
    main()
