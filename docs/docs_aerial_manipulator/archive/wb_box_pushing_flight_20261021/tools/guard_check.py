#!/usr/bin/env python3
"""The raw-mocap guard against what the mocap actually did, per bag.

  raw VRPN (/vrpn_mocap/uav_0/pose, laptop) -> processor (/uav_0/mocap, laptop) -> WiFi -> the
  estimator on the Orin: guard (state_estimator/mocap_guard), the law's odometry
  (state_estimator/local_position/odom) and what is forwarded to PX4 (fmu/in/vehicle_visual_odometry).

Prints: VRPN gaps, the processor's frozen runs / jumps / flips, each guard episode (cause, duration,
state reached), whether any bad sample reached the odometry or PX4, and the odometry's continuity.
Usage: /usr/bin/python3 guard_check.py [tag ...]"""
import sys
import numpy as np
from bp_common import Bag, BAGS, GUARD, tilt_deg


def quat_angle(q1, q2):
    d = np.abs(np.sum(q1 * q2, axis=1))
    return np.degrees(2 * np.arccos(np.clip(d, 0, 1)))


def run(tag):
    b = Bag(tag)
    print(f"\n================ {tag}")
    tv = b.t("vrpn")
    dtv = np.diff(tv)
    big = np.where(dtv > 0.030)[0]
    print(f"  raw VRPN uav_0: {len(tv)} samples {len(tv)/(tv[-1]-tv[0]):.1f} Hz; gaps > 30 ms: "
          + ", ".join(f"{tv[i]:.2f}s {dtv[i]*1e3:.0f}ms" for i in big[:20]) + (" ..." if len(big) > 20 else ""))
    # processor output
    tm = b.t("mocap")
    P = np.column_stack([b.f("mocap", f"pose.position.{c}") for c in "xyz"])
    Q = np.column_stack([b.f("mocap", f"pose.orientation.{c}") for c in "wxyz"])
    same = np.all(np.diff(P, axis=0) == 0, axis=1)
    runs, i = [], 0
    while i < len(same):
        if same[i]:
            j = i
            while j < len(same) and same[j]:
                j += 1
            n = j - i + 1
            if n >= 3:
                runs.append((tm[i], tm[j], n))
            i = j
        else:
            i += 1
    print(f"  processor /uav_0/mocap: {len(tm)} samples, frozen runs (>= 3 identical): "
          + (", ".join(f"{a:.2f}-{c:.2f}s ({n} samples, {1e3*(c-a):.0f} ms)" for a, c, n in runs) or "none"))
    step = np.linalg.norm(np.diff(P, axis=0), axis=1)
    rot = quat_angle(Q[1:], Q[:-1])
    tl = tilt_deg(*Q.T)
    jumps = np.where((step > 0.020) | (rot > 10))[0]
    print(f"  one-sample steps > 20 mm or > 10 deg: {len(jumps)}"
          + ("" if not len(jumps) else ": " + ", ".join(f"{tm[k+1]:.2f}s {1e3*step[k]:.0f}mm/{rot[k]:.0f}deg tilt {tl[k+1]:.0f}" for k in jumps[:12])))
    # guard episodes
    g = b.f("guard", "data")
    tg = b.t("guard")
    ep = g[:, 9]
    rej = g[:, 8]
    print(f"  guard: at bag start episodes {int(ep[0])} rejected {int(rej[0])}; at end episodes {int(ep[-1])} rejected {int(rej[-1])}; "
          f"worst state {GUARD[int(g[:,0].max())]}; mocap rate in the guard {np.nanmin(g[:,7]):.0f}..{np.nanmax(g[:,7]):.0f}/s")
    for k in np.where(np.diff(ep) > 0)[0]:
        t1 = tg[k + 1]
        # the episode's end: the first sample with [13] last cause + [14] duration refreshed
        w = (tg >= t1 - 0.05) & (tg <= t1 + 1.5)
        st = int(g[w, 0].max())
        k2 = np.where((tg > t1) & (g[:, 10] == 0))[0]
        dur = g[k2[0], 14] if len(k2) else np.nan
        cause = g[k2[0], 13] if len(k2) else np.nan
        print(f"    episode #{int(ep[k+1])} at {t1:.2f}s: cause {['none','stale','frozen','jump','test'][int(cause)] if np.isfinite(cause) else '?'}, "
              f"duration {1e3*dur:.0f} ms, worst state {GUARD[st]}, rejected +{int(rej[k+1]-rej[k])} "
              f"(src during: {sorted(set(int(x) for x in g[w,3]))})")
    # bad samples reaching the odometry / PX4: compare against the frozen runs and the jumps
    to = b.t("odom")
    Po = np.column_stack([b.f("odom", f"pose.pose.position.{c}") for c in "xyz"]) if "odom__pose.pose.position.x" in b.z.files else None
    tvv = b.t("vvo")
    Pv = b.f("vvo", "position")   # NED
    Pv_enu = np.column_stack([Pv[:, 1], Pv[:, 0], -Pv[:, 2]])
    for a, c, n in runs:
        fp = P[np.searchsorted(tm, a)]
        wv = (tvv >= a) & (tvv <= c + 0.05)
        hit = np.sum(np.all(np.abs(Pv_enu[wv] - fp) < 1e-9, axis=1))
        wo = (to >= a) & (to <= c + 0.05)
        hito = np.sum(np.all(np.abs(Po[wo] - fp) < 1e-9, axis=1)) if Po is not None else -1
        print(f"    frozen run {a:.2f}s: frozen pose forwarded to PX4 {hit} times in the run "
              f"(PX4 samples in it {wv.sum()}); in the law's odometry {hito} times (samples {wo.sum()})")
    if Po is not None:
        so = np.linalg.norm(np.diff(Po, axis=0), axis=1)
        dto = np.diff(to)
        ok = dto < 0.05
        print(f"  odometry continuity: largest one-sample step {1e3*so.max():.1f} mm at {to[np.argmax(so)+1]:.2f}s; "
              f"p99.9 {1e3*np.quantile(so,0.999):.1f} mm; odom gaps > 30 ms: {int(np.sum(dto>0.03))} (max {1e3*dto.max():.0f} ms)")
        # law odometry tilt (flip check)
        Qo = np.column_stack([b.f("odom", f"pose.pose.orientation.{c}") for c in "wxyz"])
        ro = quat_angle(Qo[1:], Qo[:-1])
        print(f"  odometry: largest one-sample rotation {ro.max():.2f} deg, max tilt {tilt_deg(*Qo.T).max():.1f} deg")
    # PX4 EKF2 fusing vision?
    if b.has("esf"):
        te = b.t("esf"); ev = b.f("esf", "cs_ev_pos")
        armed_t = b.t("vstatus")[b.f("vstatus", "arming_state") == 2]
        if len(armed_t):
            w = (te >= armed_t[0]) & (te <= armed_t[-1])
            print(f"  EKF2 cs_ev_pos while armed: {int(ev[w].min())}..{int(ev[w].max())} ({w.sum()} samples at 1 Hz); "
                  f"inertial dead reckoning ever: {int(b.f('esf','cs_inertial_dead_reckoning')[w].max())}")


for tag in (sys.argv[1:] or BAGS):
    run(tag)
