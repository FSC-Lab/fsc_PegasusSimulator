#!/usr/bin/env python3
"""Where does the evening's unexplained roll torque act? (user question 2026-11-01)
wb_control_debug (client source): [31..40] the FILTERED estimate the loops use, [62..71] d_sigma_c = the UNFILTERED
per-tick residual; rows 0-2 translational (world), 3-5 rotational (model: x pitch, y roll, z yaw), 6-9 joints q1..q4.
A torque on the BASE (aero on the rotors / airframe, a rotor not delivering) appears in the base rows only; a force on
the ARM appears in the base rows AND the joint rows. Per phase: mean / slow std (0.5 s average) of the filtered
estimate, and the band power of the unfiltered residual.
    PYTHONNOUSERSITE=1 /usr/bin/python3 roll_source.py"""
import numpy as np
import vt_common as V

A = "/home/shiqi/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/"
FL = [("wb_valve_turning_flight_20261028", "180652", "10-28 18:06"), ("wb_valve_turning_flight_20261028", "181833", "10-28 18:18"),
      ("wb_valve_turning_flight_20261029", "145526", "10-29 14:55"), ("wb_valve_turning_flight_20261031", "174943", "10-31 17:49"),
      ("wb_valve_turning_flight_20261031", "175725", "10-31 17:57"), ("wb_valve_turning_flight_20261031", "180423", "10-31 18:04"),
      ("wb_valve_turning_flight_20261031", "211808", "10-31 21:18")]
NM = ["pitch", "roll", "yaw", "q1", "q2", "q3", "q4"]


def uni(t, X, dt=0.004):
    tu = np.arange(t[0], t[-1], dt)
    return tu, np.column_stack([np.interp(tu, t, X[:, i]) for i in range(X.shape[1])])


def smooth(y, n):
    k = np.ones(n) / n
    if y.ndim == 1:
        return np.convolve(y, k, mode="same")
    return np.column_stack([np.convolve(y[:, i], k, mode="same") for i in range(y.shape[1])])


for d, tag, lab in FL:
    V._c.NPZ = A + d + "/npz"
    b = V.Bag(tag)
    ed = b.edges("pl_pstatus")
    tg = [t for t, s in ed if s.startswith("FLYING go_to_start")][0]
    th = [t for t, s in ed if s.startswith("DONE go_to_start")][0]
    tr = [t for t, s in ed if s.startswith("FLYING ready")][0]
    tw = b.t("wb"); W = b.f("wb", "data")
    ok = np.array([len(r) >= 118 for r in W]); Wm = np.array([np.asarray(r[:118], float) for r in W[ok]]); tw = tw[ok]
    print(f"\n### {lab}   Go To Start {th - tg:.1f} s, room hold {tr - th:.2f} s, hold at x {Wm[np.searchsorted(tw, th), 48]:+.2f} y {Wm[np.searchsorted(tw, th), 49]:+.2f}")
    for ph, (a_, b_) in (("H hover 4 s", (tg - 4, tg)), ("G Go To Start", (tg, th)), ("R hold+0.9 s Ready", (th, tr + 0.9))):
        m = (tw >= a_) & (tw <= b_) & (Wm[:, 0] == 1.0)
        t, X = tw[m], Wm[m]
        tu, F = uni(t, X[:, [34, 35, 36, 37, 38, 39, 40]])
        tu, C = uni(t, X[:, [65, 66, 67, 68, 69, 70, 71]])
        s = slice(62, -62)
        slow = np.std(smooth(F[:, :], 125)[s], axis=0)
        lo = np.std(smooth(C, 125)[s], axis=0)                        # < 2 Hz part of the raw residual (std = mean removed)
        hi = np.std((C - smooth(C, 125))[s], axis=0)                  # > 2 Hz
        # correlation of the slow roll residual with the slow joint residuals
        cs = smooth(C, 125)[s]
        cc = [np.corrcoef(cs[:, 1], cs[:, k])[0, 1] for k in range(7)]
        print(f"  {ph:20s} filtered mean " + " ".join(f"{NM[i]} {F[:, i].mean():+.3f}" for i in range(7)))
        print(f"  {'':20s} filtered slow sd " + " ".join(f"{NM[i]} {slow[i]:.3f}" for i in range(7)))
        print(f"  {'':20s} raw <2 Hz sd      " + " ".join(f"{NM[i]} {lo[i]:.3f}" for i in range(7)) + "   >2 Hz roll %.3f" % hi[1])
        print(f"  {'':20s} corr(raw roll, .) " + " ".join(f"{NM[i]} {cc[i]:+.2f}" for i in range(7)))
