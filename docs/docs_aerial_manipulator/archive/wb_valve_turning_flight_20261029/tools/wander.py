#!/usr/bin/env python3
"""(1) Is the valve's angle error the free-axis anchor carrying the vehicle's fore-aft wander?
regress theta_valve - theta_plan on (fore-aft CoM error / r).  (2) Wander in free hover (claw
room-held above the handle) vs in contact, per direction, with the dominant frequency.  (3) How close
every other guard came: low-passed |F_hat| (5 N), vehicle tilt (10), q3 (3 deg), q2 (45), q1 (+-35),
q4, and the joint torques."""
import numpy as np
from vt_common import Bag, BAGS, tilt_deg
from valve_axis import step_times

R_CLAW = 0.160
def dom_freq(x, dt):
    x = x - x.mean(); f = np.fft.rfftfreq(len(x), dt); P = np.abs(np.fft.rfft(x * np.hanning(len(x))))**2
    k = (f > 0.1) & (f < 3); return f[k][np.argmax(P[k])]
for tag in BAGS:
    b = Bag(tag); st = step_times(b); T = np.load(f"../npz/turn_{tag}.npz"); g = T["g"]; dt = 0.02
    ta = st.get("abort", g[-1])
    # (1)
    w = (g >= st["push"] + 0.3) & (g <= ta)
    err = T["th_v"] - T["th_plan"]; x = np.degrees(T["ec_t"] / R_CLAW)
    k, c0 = np.polyfit(x[w], err[w], 1); r = np.corrcoef(x[w], err[w])[0, 1]
    print(f"===== {tag}")
    print(f"  valve - plan: rms {np.sqrt(np.mean(err[w]**2)):.1f} deg, range {err[w].min():+.1f}..{err[w].max():+.1f};"
          f" vs fore-aft CoM error/r: slope {k:.2f} (free-axis weight 0.75), corr {r:.2f}, offset {c0:+.1f} deg")
    # (2) windows
    gs = st["push"] - 6.0   # ~the Go To Start hold above the handle + Ready
    wins = {"hold above the handle (free, room-held)": ((g >= st["handle"] - 4.5) & (g <= st["handle"] - 3.2)),
            "Ready descent + jaws closing": ((g >= st["handle"] - 3.0) & (g <= st["push"])),
            "contact (push step -> abort)": w}
    for lab, m in wins.items():
        if m.sum() < 10: continue
        et, er, ez = T["ec_t"][m]*1e3, T["ec_r"][m]*1e3, T["ec"][m, 2]*1e3
        print(f"  {lab:40s} {m.sum()*dt:4.1f} s  CoM err rms fore-aft {et.std():5.1f} lateral {er.std():5.1f} vert {ez.std():5.1f} mm"
              f"  p-p {np.ptp(et):5.0f}/{np.ptp(er):5.0f}  |e_R| rms {np.sqrt(np.mean(np.sum(T['eR'][m]**2,1))):.3f}")
    print(f"  contact dominant freq: fore-aft {dom_freq(T['ec_t'][w], dt):.2f} Hz, lateral {dom_freq(T['ec_r'][w], dt):.2f} Hz, q1 {dom_freq(T['q'][w,0], dt):.2f} Hz")
    # (3) guards and joints over contact
    ar = b.f("pl_arr", "data"); tr = b.t("pl_arr"); m3 = (tr >= st["push"]) & (tr <= ta)
    q = T["q"][w]; tau = T["tau"][w]
    print(f"  |F_hat| LP max {np.nanmax(ar[m3,10]):.2f} N (guard 5);  joints q1 {q[:,0].min():+.1f}..{q[:,0].max():+.1f} (+-35)  "
          f"q2 {q[:,1].min():+.1f}..{q[:,1].max():+.1f} (-20..45)  q3 {q[:,2].min():+.1f}..{q[:,2].max():+.1f} (guard 3, stop 50)  q4 {q[:,3].min():+.1f}..{q[:,3].max():+.1f}")
    print(f"  joint torque max |tau| {np.round(np.abs(tau).max(0),2)} N.m;  plan q_d {np.round(T['qd'][w].mean(0),1)}")
    tv = b.t("vatt"); qv = b.f("vatt", "q"); mv = (tv >= st["push"]) & (tv <= ta)
    tl = tilt_deg(qv[mv, 0], qv[mv, 1], qv[mv, 2], qv[mv, 3])
    print(f"  vehicle tilt in contact: median {np.median(tl):.1f}, max {tl.max():.1f} deg (guard 10)")
