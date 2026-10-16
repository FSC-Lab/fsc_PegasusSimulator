#!/usr/bin/env python3
"""Per-joint torque-cap audit of hardware flights: how close each arm joint came to every limit that can clip it.

    PYTHONNOUSERSITE=1 /usr/bin/python3 caps_audit.py <npz> [<npz> ...]

Joint order everywhere here: j1..j4 (law_debug is hardware j1..j4; joint_states is joint2, joint3, joint1, joint4).
  max_effort  the arm controller's clamp on the TORQUE TERM (duty counts). Pass-through (DIRECT): the term is
              tau_ext*kappa + aux + gravity_corr (law_debug [9..12], [18..21], [22..25]) -- fully logged, so the
              audit reports the PRE-clamp demand and the time it sat above the cap. Local hold (SAFETY): the PD is
              not logged; duty - ke*qd_ref (= torque term + current trim) is reported instead.
  PWM Limit   the servo's EEPROM clamp on the TOTAL written duty (torque term + back-EMF feed-forward + trim):
              law_debug [13..16]. Above it the firmware truncates silently.
  current     Present Current (2.69 mA counts): peak, and the worst 10 s mean |I| -- the heat indicator (the
              2026-08-26 j2 overload shutdown came at ~250 counts sustained).
"""
import os
import sys
import numpy as np

MAXEFF = np.array([57.6117, 364.8743, 192.0391, 57.6117])        # hardware yaml max_effort (counts)
PWMLIM = np.array([180.0, 545.0, 330.0, 180.0])                  # xacro joint*_pwm_limit
KPWM = np.array([169.47, 149.70, 135.25, 148.51])                # counts per N*m
JS_OF = [2, 0, 1, 3]                                             # j1..j4 -> joint_states column


def audit(path):
    d = np.load(path)
    tl = d["law__recv"]; L = d["law__data"]
    tj = d["js__recv"]; C = d["js__effort"]
    pt = L[:, 0] > 0.5
    T_pt = L[:, 9:13] * KPWM + L[:, 18:22] + L[:, 22:26]      # pass-through pre-clamp torque term
    duty = L[:, 13:17]
    dt_l = np.median(np.diff(tl))
    rows = []
    for j in range(4):
        r = {}
        a = np.abs(T_pt[pt, j])
        r["pt_peak"] = a.max() if a.size else np.nan
        r["pt_p999"] = np.percentile(a, 99.9) if a.size else np.nan
        r["pt_over_s"] = (a > MAXEFF[j]).sum() * dt_l
        h = np.abs(duty[~pt, j])                                  # hold: total duty (term + trim + FF)
        r["hold_peak"] = h.max() if h.size else np.nan
        tot = np.abs(duty[:, j])
        r["duty_peak"] = tot.max()
        r["duty_over_s"] = (tot >= PWMLIM[j] - 0.5).sum() * dt_l
        i = np.abs(C[:, JS_OF[j]])
        r["i_peak"] = i.max()
        w = max(1, int(round(10.0 / np.median(np.diff(tj)))))
        r["i_10s"] = np.convolve(i, np.ones(w) / w, mode="valid").max() if len(i) > w else i.mean()
        rows.append(r)
    hw = d["dxl__dxl_hw_state"] if "dxl__dxl_hw_state" in d.files else np.zeros(1)
    return rows, (tl[pt][-1] - tl[pt][0]) if pt.any() else 0.0, int(np.count_nonzero(hw))


def main(paths):
    for p in paths:
        rows, t_direct, hw = audit(p)
        name = "/".join(p.split("/")[-3:])
        print(f"\n=== {name}   DIRECT {t_direct:.0f} s   servo hardware-error samples {hw}")
        print(f"{'jt':3s} | {'max_effort':>10s} {'DIRECT term peak':>16s} {'p99.9':>6s} {'>cap s':>6s} | "
              f"{'SAFETY duty pk':>14s} | {'PWM lim':>7s} {'duty peak':>9s} {'>=lim s':>7s} | {'I peak':>6s} {'I 10s':>6s}")
        for j, r in enumerate(rows):
            print(f"j{j+1}  | {MAXEFF[j]:10.1f} {r['pt_peak']:9.1f} ({r['pt_peak']/MAXEFF[j]*100:3.0f}%) {r['pt_p999']:6.1f} "
                  f"{r['pt_over_s']:6.2f} | {r['hold_peak']:14.1f} | {PWMLIM[j]:7.0f} {r['duty_peak']:9.1f} "
                  f"{r['duty_over_s']:7.2f} | {r['i_peak']:6.0f} {r['i_10s']:6.0f}")


if __name__ == "__main__":
    main(sys.argv[1:])
