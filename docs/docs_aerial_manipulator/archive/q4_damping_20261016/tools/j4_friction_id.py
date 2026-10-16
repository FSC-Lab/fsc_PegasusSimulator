#!/usr/bin/env python3
"""j4's REAL friction from the UNLOADED slides of the 10-15 flights (after the basket left: D-1 from the
drop, D-2 from the pull-out). An empty wrist is gravity-neutral about its own axis, so on a slide

    J*qddot = tau_delivered - f_k*sign(qdot) - b_v*qdot          (tau_delivered = Kt * Present Current)

Fitted for J, f_k, b_v (+ offset) on samples with |qdot| > 0.05 rad/s; J is also pinned at the model's
armature + gripper to check the friction does not hinge on it. Compared with what the arm controller's
friction relay adds on the same samples (law_debug[18..21], j4 = [21], counts / 148.51) -- the relay is
meant to EQUAL the real friction. Also: the torque at breakaway (the largest |delivered| in the 0.2 s before
each stick ends) = static friction.

    PYTHONNOUSERSITE=1 /usr/bin/python3 j4_friction_id.py
"""
import os
import sys
import numpy as np
from scipy.signal import butter, filtfilt

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from ident_motor import NPZ, LSB, KT  # noqa: E402

KP4 = 148.51                                   # duty counts per N*m (j4)
J_MODEL = 0.0097 + 0.0012                      # armature (bench, joint-diagonal) + gripper about the roll axis
# unloaded windows, seconds after DIRECT entry (from the q4 envelope: D-1 drop -> end, D-2 pull-out -> end)
WIN = {"d1": (60.0, 1e9), "d2": (24.0, 1e9)}


def lpf(x, fs, fc):
    b, a = butter(2, fc / (fs / 2))
    return filtfilt(b, a, x)


def main():
    rows = []
    for run, (a, b) in WIN.items():
        d = np.load(os.path.join(NPZ, f"{run}.npz"))
        tj = d["js__recv"]; tl = d["law__recv"]; L = d["law__data"]; direct = L[:, 0] > 0.5
        td0, td1 = tl[direct][0], tl[direct][-1]
        fs = 250.0
        tu = np.arange(max(td0 + a, tj[0]), min(td0 + b, td1) - 0.5, 1 / fs)
        q = np.interp(tu, tj, d["js__position"][:, 3])
        qd = lpf(np.gradient(q, tu), fs, 8.0); qdd = lpf(np.gradient(qd, tu), fs, 5.0)
        tau = lpf(KT[3] * np.interp(tu, tj, d["js__effort"][:, 3]) * LSB, fs, 8.0)
        relay = np.interp(tu, tl, L[:, 21]) / KP4
        law = np.interp(tu, tl, L[:, 12])
        rows.append((run, tu, q, qd, qdd, tau, relay, law))

    print(f"{'run':3s} {'fit':22s} {'J':>7s} {'f_k':>7s} {'b_v':>7s} {'c':>7s} {'R2':>5s} | relay on these samples")
    for run, tu, q, qd, qdd, tau, relay, law in rows + [("all",) + tuple(np.concatenate([r[k] for r in rows]) for k in range(1, 8))]:
        m = np.abs(qd) > 0.05
        A = np.c_[qdd[m], np.sign(qd[m]), qd[m], np.ones(m.sum())]
        x, *_ = np.linalg.lstsq(A, tau[m], rcond=None)
        r2 = 1 - np.var(tau[m] - A @ x) / np.var(tau[m])
        A2 = np.c_[np.sign(qd[m]), qd[m], np.ones(m.sum())]
        x2, *_ = np.linalg.lstsq(A2, tau[m] - J_MODEL * qdd[m], rcond=None)
        rel = np.mean(np.abs(relay[m]))
        print(f"{run:3s} {'J free':22s} {x[0]:7.4f} {x[1]:7.4f} {x[2]:7.4f} {x[3]:+7.4f} {r2:5.2f} | |relay| mean {rel:.4f} N*m"
              f" ({m.mean()*100:.0f} % of the window moving)")
        print(f"{'':3s} {'J pinned '+str(round(J_MODEL,4)):22s} {J_MODEL:7.4f} {x2[0]:7.4f} {x2[1]:7.4f} {x2[2]:+7.4f}")

    # breakaway: the end of each stick (|qd| < 0.01 for >= 0.5 s, then |qd| > 0.05)
    print("\nbreakaways (unloaded): delivered torque at the moment the stick ends, and the law's own share")
    for run, tu, q, qd, qdd, tau, relay, law in rows:
        still = np.abs(qd) < 0.01
        k = 0
        while k < len(tu) - 1:
            if still[k]:
                j = k
                while j < len(tu) - 1 and still[j]:
                    j += 1
                if (tu[j] - tu[k]) >= 0.5 and j < len(tu) - 1:
                    w = (tu >= tu[j] - 0.2) & (tu <= tu[j])
                    kk = np.argmax(np.abs(tau[w]))
                    print(f"  {run} stuck {tu[j]-tu[k]:4.1f} s at {np.degrees(q[k]):6.1f} deg -> breakaway at "
                          f"|tau| {np.abs(tau[w]).max():.3f} N*m (law {law[w][kk]:+.3f}, relay {relay[w][kk]:+.3f})")
                k = j + 1
            else:
                k += 1


if __name__ == "__main__":
    main()
