#!/usr/bin/env python3
"""Per-hold arm-torque chatter and CoM wander, for the 2026-09-14 mismatch campaign.

    /usr/bin/python3 hold_chatter.py <run.npz> [run2.npz ...]

wb_l1_metrics.py scores the whole soak and the last 30 s; this scores EVERY
hold of the mission separately (the 14 s before each leg start, plus the final
hold), which is what separates a sustained limit cycle from one that a leg
kicked and that then decays. Two tables:

  1. per hold: std of the j2 torque command [N.m] and the fraction of its
     power in 0.6-1.3 Hz (the attitude loop's own sqrt(k_R/I) = 0.88 Hz mode)
  2. final hold: per-axis CoM error mean/std (an OFFSET vs a WANDER -- the
     |e| mean the metrics report cannot tell them apart), EE task error std,
     the attributed EE force mean/std, and the j2/j3 torque std with the
     dominant frequency.

Layout: wb_control_debug as wb_l1_metrics.py documents it (dbg[:,1:]);
[9..12] q_d_smooth, [13..16] tau_joint, [24..27] e_y, [45..48] x_cd,
[48..51] x_c, [58..61] F_hat_y force rows.
"""
import sys

import numpy as np


def band_frac(x, dt, lo=0.6, hi=1.3):
    x = x - x.mean()
    f = np.fft.rfftfreq(len(x), dt)
    P = np.abs(np.fft.rfft(x)) ** 2
    P[0] = 0.0
    return P[(f > lo) & (f < hi)].sum() / max(P.sum(), 1e-30)


def dom_freq(x, dt):
    x = x - x.mean()
    f = np.fft.rfftfreq(len(x), dt)
    P = np.abs(np.fft.rfft(x)) ** 2
    P[0] = 0.0
    k = int(np.argmax(P))
    return f[k], P[k] / max(P.sum(), 1e-30)


def main(paths):
    print("1. per hold: tau_j2 std [N.m] / power in 0.6-1.3 Hz")
    for p in paths:
        z = np.load(p, allow_pickle=True)
        dbg = z["dbg"]
        t, d = dbg[:, 0], dbg[:, 1:]
        direct = d[:, 0] == 1
        marks = [(float(s.split()[0]), s.split()[1]) for s in z["leg_marks"]]
        ends = marks + [(t[direct][-1], "final")]
        cells = []
        for tm, nm in ends:
            w = (t > tm - 15) & (t < tm - 1) & direct
            if w.sum() < 500:
                continue
            dt = np.median(np.diff(t[w]))
            tau2 = d[w, 14]
            cells.append(f"{nm[:9]:9s} {tau2.std():.3f}/{100 * band_frac(tau2, dt):2.0f}%")
        print(f"  {p.split('/')[-1]:24s} " + "  ".join(cells))

    print("\n2. final hold (16 s before the end of DIRECT)")
    print(f"  {'run':24s} {'e_com y mean/std [mm]':>22s} {'e_y std [mm]':>13s} "
          f"{'F_hat_y,z mean/std [N]':>24s} {'tau_j2 std @ f':>20s} {'tau_j3 std':>10s}")
    for p in paths:
        z = np.load(p, allow_pickle=True)
        dbg = z["dbg"]
        t, d = dbg[:, 0], dbg[:, 1:]
        direct = d[:, 0] == 1
        tD, dD = t[direct], d[direct]
        w = (tD > tD[-1] - 17) & (tD < tD[-1] - 1)
        dt = np.median(np.diff(tD[w]))
        e = 1e3 * (dD[w, 48:51] - dD[w, 45:48])
        ey = 1e3 * dD[w, 24:27]
        fy = dD[w, 58:61]
        tau = dD[w, 13:17]
        f2, _ = dom_freq(tau[:, 1], dt)
        print(f"  {p.split('/')[-1]:24s} {e[:, 1].mean():+7.1f} / {e[:, 1].std():5.1f}        "
              f"{np.linalg.norm(ey, axis=1).std():5.1f}         "
              f"{fy[:, 2].mean():+7.3f} / {fy[:, 2].std():5.3f}      "
              f"{tau[:, 1].std():.4f} @ {f2:.2f} Hz   {tau[:, 2].std():.4f}")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    main(sys.argv[1:])
