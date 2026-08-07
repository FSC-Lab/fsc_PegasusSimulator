#!/usr/bin/env python3
"""sensor_combined comparison, real T650 flight vs IsaacSim, over the step sequence.

Split into two bands because they answer different questions:
  < 2 Hz   rigid-body motion -- does the simulated vehicle move like the real one?
  > 2 Hz   airframe vibration and sensor noise -- does the simulated IMU look like the
           real one? (It does not, and that is the point.)
"""
import sys

import numpy as np

from plot_steps import load_side
from steps import detect_steps, ref_onsets

AX = ["x", "y", "z"]


def lowpass(t, y, fc):
    dt = np.gradient(t)
    a = np.clip(1 - np.exp(-2 * np.pi * fc * dt), 0, 1)
    f = np.empty_like(y); f[0] = y[0]
    for i in range(1, len(y)):
        f[i] = f[i - 1] + a[i] * (y[i] - f[i - 1])
    b = np.empty_like(f); b[-1] = f[-1]
    for i in range(len(f) - 2, -1, -1):
        b[i] = b[i + 1] + a[i] * (f[i] - b[i + 1])
    return b


def band_stats(t, y, fc=2.0):
    lo = np.stack([lowpass(t, y[:, k], fc) for k in range(y.shape[1])], 1)
    return lo, y - lo


def run(tag, sim_npz, fc=2.0):
    t_ref, ref, _ = ref_onsets(f"exp_{tag}.npz", f"ref_{tag}.npz")
    steps = detect_steps(t_ref, ref)
    lo_t, hi_t = steps[0]["t"] - 2.0, steps[-1]["t"] + 2.0
    R, S = load_side(f"exp_{tag}.npz", "real"), load_side(sim_npz, "sim")

    out = {}
    for side, name in ((R, "real"), (S, "sim")):
        m = (side["t_sc"] >= lo_t) & (side["t_sc"] <= hi_t)
        t = side["t_sc"][m]
        g_lo, g_hi = band_stats(t, side["gyro"][m], fc)
        a_lo, a_hi = band_stats(t, side["accel"][m], fc)
        out[name] = dict(n=int(m.sum()), t=t, g_lo=g_lo, g_hi=g_hi, a_lo=a_lo, a_hi=a_hi,
                         accel=side["accel"][m], gyro=side["gyro"][m],
                         rate=m.sum() / (t[-1] - t[0]))
    return tag, steps, out


def main():
    print(f"\n{'='*104}\nsensor_combined: real vs IsaacSim over the step sequence\n{'='*104}")
    for tag, sim in (("A", "sim_stack_A.npz"), ("B", "sim_stack_B.npz")):
        tag, steps, o = run(tag, sim)
        print(f"\n--- FLIGHT {tag}  (window covers steps 0..{steps[-1]['idx']}) ---")
        print(f"    samples: real {o['real']['n']} @ {o['real']['rate']:.1f} Hz   "
              f"sim {o['sim']['n']} @ {o['sim']['rate']:.1f} Hz")
        print(f"\n    {'channel':12} {'band':9} | {'real RMS':>10} {'sim RMS':>10} "
              f"{'sim/real':>9}")
        print("    " + "-" * 60)
        for label, kl, kh, unit in (("gyro", "g_lo", "g_hi", "rad/s"),
                                    ("accel", "a_lo", "a_hi", "m/s^2")):
            for k in range(3):
                for band, key in (("<2 Hz", kl), (">2 Hz", kh)):
                    r = float(np.sqrt(np.mean(o["real"][key][:, k] ** 2)))
                    s = float(np.sqrt(np.mean(o["sim"][key][:, k] ** 2)))
                    # the <2 Hz accel_z band is dominated by the ~9.8 constant, remove it
                    if label == "accel" and k == 2 and band == "<2 Hz":
                        r = float(np.std(o["real"][key][:, k]))
                        s = float(np.std(o["sim"][key][:, k]))
                        band = "<2 Hz*"
                    print(f"    {label+'_'+AX[k]:12} {band:9} | {r:10.4f} {s:10.4f} "
                          f"{s/r if r else np.nan:9.2f}")
            print()
        for name in ("real", "sim"):
            a = o[name]["accel"]
            print(f"    {name:4} |a| mean {np.linalg.norm(a, axis=1).mean():7.4f} m/s^2   "
                  f"a_z mean {a[:,2].mean():8.4f}   "
                  f"broadband accel RMS(x,y,z) "
                  f"{np.sqrt((o[name]['a_hi']**2).mean(axis=0)).round(4)}")
        print("    * std, not RMS: the <2 Hz accel_z band is dominated by the ~-9.8 offset")


if __name__ == "__main__":
    main()
