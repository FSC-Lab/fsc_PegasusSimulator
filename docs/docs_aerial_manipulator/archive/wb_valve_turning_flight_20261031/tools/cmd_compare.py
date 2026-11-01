#!/usr/bin/env python3
"""Thrust vs torque commands of every hardware valve flight, phase by phase (user question 2026-11-01: "which command is
abnormal, thrust or torque?"). From wb_control_debug (client source): [17] u1 [N], [21..23] body torque command FLU
[N.m] (x roll, y pitch, z yaw; = u1 hat(r_0c) e3 + u2 + N1^T u3 in the body frame), [28..30] e_R (model: x pitch,
y roll), [34..36] the filtered rotational estimate (model: x pitch, y roll, z yaw), [41..44] motor commands [0..1],
[52..55] allocator unallocated wrench.
Phases: H = 4 s of DIRECT hover before Go To Start; G = the last 3 s of Go To Start; R = the room hold (Go To Start
done -> Ready); D = the first 0.9 s of Ready.
Columns: mean and 'hf' = std after removing a 0.5 s moving average (the fast part of the command).
    PYTHONNOUSERSITE=1 /usr/bin/python3 cmd_compare.py"""
import numpy as np
import vt_common as V

A = "/home/shiqi/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/"
FL = [("wb_valve_turning_flight_20261028", "180652", "10-28 18:06"), ("wb_valve_turning_flight_20261028", "181833", "10-28 18:18"),
      ("wb_valve_turning_flight_20261029", "145526", "10-29 14:55"), ("wb_valve_turning_flight_20261031", "174943", "10-31 17:49"),
      ("wb_valve_turning_flight_20261031", "175725", "10-31 17:57"), ("wb_valve_turning_flight_20261031", "180423", "10-31 18:04"),
      ("wb_valve_turning_flight_20261031", "211808", "10-31 21:18")]
MG = 3.746170 * 9.81


def hf(t, x, w=0.5):
    """std of x after removing a w-second centred moving average (uniform resample at 250 Hz)."""
    tu = np.arange(t[0], t[-1], 0.004)
    y = np.interp(tu, t, x)
    n = max(3, int(w / 0.004) | 1)
    k = np.ones(n) / n
    m = np.convolve(y, k, mode="same")
    s = slice(n // 2, len(y) - n // 2)
    return float(np.std((y - m)[s]))


rows = []
for d, tag, lab in FL:
    V._c.NPZ = A + d + "/npz"
    b = V.Bag(tag)
    ed = b.edges("pl_pstatus")
    tg = [t for t, s in ed if s.startswith("FLYING go_to_start")][0]
    th = [t for t, s in ed if s.startswith("DONE go_to_start")][0]
    tr = [t for t, s in ed if s.startswith("FLYING ready")][0]
    tw = b.t("wb")
    W = b.f("wb", "data")
    ok = np.array([len(r) >= 118 for r in W])
    Wm = np.array([np.asarray(r[:118], float) for r in W[ok]])
    tw = tw[ok]
    vb = None
    if b.has("batt"):
        tb = b.t("batt")
        vk = [k for k in b.z.files if k.startswith("batt__") and "voltage" in k]
        if vk:
            vv = np.asarray(b.f("batt", vk[0].split("__", 1)[1]), float)
            vb = vv
    for ph, (a_, b_) in (("H", (tg - 4, tg)), ("G", (th - 3, th)), ("R", (th, tr)), ("D", (tr, tr + 0.9))):
        m = (tw >= a_) & (tw <= b_) & (Wm[:, 0] == 1.0)     # DIRECT ticks only
        t, X = tw[m], Wm[m]
        mo = X[:, 41:45]    # PX4 Quad-X: m0 FR (CCW), m1 RL (CCW), m2 FL (CW), m3 RR (CW)
        dif = np.c_[(mo[:, 1] + mo[:, 2]) - (mo[:, 0] + mo[:, 3]), (mo[:, 0] + mo[:, 2]) - (mo[:, 1] + mo[:, 3]),
                    (mo[:, 2] + mo[:, 3]) - (mo[:, 0] + mo[:, 1])]
        r = dict(flight=lab, ph=ph,
                 u1=X[:, 17].mean(), u1_hf=hf(t, X[:, 17]),
                 tx=X[:, 21].mean(), ty=X[:, 22].mean(), tz=X[:, 23].mean(),
                 tx_hf=hf(t, X[:, 21]), ty_hf=hf(t, X[:, 22]), tz_hf=hf(t, X[:, 23]),
                 eR_roll=1e3 * X[:, 29].std(), eR_pitch=1e3 * X[:, 28].std(),
                 mot=X[:, 41:45].mean(0), mot_hf=np.mean([hf(t, X[:, 41 + i]) for i in range(4)]),
                 dif=dif.mean(0), slow=[np.ptp(np.convolve(X[:, 21 + i], np.ones(125) / 125, mode='valid')) if len(X) > 125 else np.nan for i in range(3)],
                 unalloc=np.abs(X[:, 52:56]).max(), d_roll=X[:, 35].mean(), d_pitch=X[:, 34].mean())
        if vb is not None:
            mb = (tb >= a_) & (tb <= b_)
            r["V"] = float(np.nanmean(vb[mb])) if mb.any() else np.nan
        rows.append(r)

f = lambda x, p=2: f"{x:+.{p}f}"   # noqa: E731
for ph, name in (("H", "hover before Go To Start"), ("G", "last 3 s of Go To Start"), ("R", "room hold"), ("D", "first 0.9 s of Ready")):
    print(f"\n== {name}")
    print(f"{'flight':12s} {'V':>5s} | {'u1 N':>6s} {'u1/mg':>6s} {'u1 hf':>6s} | {'tau roll':>8s} {'pitch':>6s} {'yaw':>6s} | "
          f"{'hf roll':>7s} {'pitch':>6s} {'yaw':>6s} N.m | {'eR roll':>7s} {'pitch':>6s} mrad | motors mean               hf    | motor differentials              | torque 0.5 s avg p-p r/p/y | unalloc")
    for r in rows:
        if r["ph"] != ph:
            continue
        print(f"{r['flight']:12s} {r.get('V', np.nan):5.2f} | {r['u1']:6.2f} {r['u1']/MG:6.3f} {r['u1_hf']:6.3f} | {f(r['tx'],3):>8s} {f(r['ty'],3):>6s} "
              f"{f(r['tz'],3):>6s} | {r['tx_hf']:7.3f} {r['ty_hf']:6.3f} {r['tz_hf']:6.3f}     | {r['eR_roll']:7.1f} {r['eR_pitch']:6.1f}      | "
              f"{' '.join(f'{m:.3f}' for m in r['mot'])} {r['mot_hf']:.4f} | L-R {r['dif'][0]:+.3f} F-R {r['dif'][1]:+.3f} CW-CCW {r['dif'][2]:+.3f} | slow p-p {' '.join(f'{v:.3f}' for v in r['slow'])} | {r['unalloc']:6.3f}")
