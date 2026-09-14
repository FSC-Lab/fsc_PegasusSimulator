#!/usr/bin/env python3
"""Score a whole-body GROUND test (inert props, vehicle seated).

    PYTHONNOUSERSITE=1 /usr/bin/python3 ground_score.py ground_A.npz

The flying scorers ask "how well did it hold station?". That question is
meaningless here: the floor holds station. What this asks instead is

  * did the vehicle STAY SEATED, or did the arm lever it over?
  * what did the law do with a thrust channel that answers to the floor --
    where did the collective and the disturbance estimate go, and did either
    rail?
  * did the ARM still do its job (plan, execute, track) while that happened?

Debug layout is the campaign's (wb_l1_metrics.py's header).
"""
import sys
import numpy as np

D_MODE, D_U1, D_NSAT, D_STREAM, D_L1, D_NCLAMP = 0, 17, 51, 56, 57, 88
D_TAU, D_EY, D_ER = slice(13, 17), slice(24, 28), slice(28, 31)
D_DHAT, D_MOT = slice(31, 41), slice(41, 45)
D_XCD, D_XC, D_FY = slice(45, 48), slice(48, 51), slice(58, 62)
D_WHAT = slice(78, 88)


def main(path):
    z = np.load(path, allow_pickle=True)
    log, dbg = z["log"], z["dbg"]
    t, d = dbg[:, 0], dbg[:, 1:]
    direct = np.nan_to_num(d[:, D_MODE]) > 0.5
    td = float(z["t_direct"])
    marks = [m.split(None, 1) for m in z["leg_marks"]]

    print(f"== {path.split('/')[-1]} ==")
    print(f"aborted      : {bool(z['aborted'])}  {str(z['abort_reason'])}")
    print(f"leg_fail     : {str(z['leg_fail']) or 'none'}")
    print(f"observer     : {'L1' if np.nanmax(d[direct, D_L1]) > 0.5 else 'GMO'}")
    print(f"DIRECT       : {t[direct].max() - t[direct].min():.1f} s "
          f"(entered t={td:.1f} s)")
    print(f"legs         : {', '.join(n for _, n in marks)}")

    # ---- did it stay seated? --------------------------------------------
    lt, x, y, zz, tilt = log[:, 0], log[:, 1], log[:, 2], log[:, 3], log[:, 7]
    ind = log[:, 17] > 0.5
    seat = np.array([x[ind][0], y[ind][0]])
    slide = np.hypot(x - seat[0], y - seat[1])
    print("\n-- did it stay seated --")
    print(f"body z       : {zz[ind].min():.4f} .. {zz[ind].max():.4f} m "
          f"(seated resting height 0.305)")
    print(f"tilt         : max {tilt[ind].max():.2f} deg "
          f"(watchdog 20, |e_R| max {np.nanmax(np.linalg.norm(d[direct, D_ER], axis=1)):.4f})")
    print(f"horiz slide  : max {slide[ind].max()*1000:.1f} mm")
    print(f"final        : z {zz[-1]:.4f} m  tilt {tilt[-1]:.2f} deg  "
          f"slide {slide[-1]*1000:.1f} mm")

    # ---- what the law did with a dead thrust channel --------------------
    u1, dz = d[:, D_U1], d[:, D_DHAT][:, 2]
    mot = d[:, D_MOT]
    late = direct & (t > t[direct].min() + 10.0)
    print("\n-- the thrust channel, answering to the floor --")
    print(f"u1 (collective): entry {np.nanmean(u1[direct][:50]):7.2f} N "
          f"-> end {np.nanmean(u1[direct][-250:]):7.2f} N   "
          f"max {np.nanmax(u1[direct]):7.2f} N")
    print(f"                 (a hover for this plant is ~40.4 N; "
          f"the props delivered 0.00 N throughout)")
    print(f"d_hat_z        : entry {np.nanmean(dz[direct][:50]):7.2f} N "
          f"-> end {np.nanmean(dz[direct][-250:]):7.2f} N   "
          f"min {np.nanmin(dz[direct]):7.2f} N")
    print(f"rotor cmd      : max {np.nanmax(mot[direct]):.4f} of 1.0, "
          f"mean {np.nanmean(mot[direct]):.4f}")
    print(f"outputs on a bound : {100*np.nanmean(d[direct, D_NCLAMP] > 0):.2f} % of DIRECT samples")
    print(f"rotor saturated    : {100*np.nanmean(d[direct, D_NSAT] > 0):.2f} %")
    print(f"streamed ref fresh : {100*np.nanmean(d[direct, D_STREAM] > 0.5):.2f} %")

    # ---- the arm ---------------------------------------------------------
    tau = np.abs(d[:, D_TAU])
    ey = np.linalg.norm(d[:, D_EY][:, :3], axis=1)
    print("\n-- the arm --")
    print(f"peak |tau|   : {np.nanmax(tau[direct]):.3f} of 3.0 N.m")
    print(f"on the clamp : {100*np.nanmean(tau[direct] > 2.999):.2f} % of joint-samples")
    print(f"EE error     : mean {1000*np.nanmean(ey[direct]):.1f} mm, "
          f"max {1000*np.nanmax(ey[direct]):.1f} mm")
    q = np.degrees(log[:, 13:17])       # broadcaster order [q2, q3, q1, q4]
    print(f"joints (broadcaster order q2,q3,q1,q4), DIRECT span [deg]:")
    for i, nm in enumerate(["q2", "q3", "q1", "q4"]):
        print(f"   {nm}: {np.nanmin(q[ind, i]):7.2f} .. {np.nanmax(q[ind, i]):7.2f}"
              f"   final {q[-1, i]:7.2f}")

    # ---- per leg ---------------------------------------------------------
    if marks:
        print("\n-- per leg (EE error, tilt, slide) --")
        tm = [float(a) for a, _ in marks] + [t[direct].max()]
        for i, (_, nm) in enumerate(marks):
            w = direct & (t >= tm[i]) & (t < tm[i + 1])
            wl = (lt >= tm[i]) & (lt < tm[i + 1])
            if not w.any():
                continue
            print(f"   {nm:<10s} EE peak {1000*np.nanmax(ey[w]):6.1f} mm  "
                  f"settled {1000*np.nanmean(ey[w][-100:]):5.1f} mm  "
                  f"tilt max {np.nanmax(tilt[wl]):5.2f} deg  "
                  f"slide {1000*np.nanmax(slide[wl]):5.1f} mm  "
                  f"|tau| {np.nanmax(tau[w]):.2f}")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "ground_A.npz")
