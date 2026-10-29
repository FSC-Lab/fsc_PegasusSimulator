#!/usr/bin/env python3
"""Fore-aft ring of a C++ FSC-autopilot hold, from its bag (scripts/start_vtol_fsc_hil.sh records -a).

    source scripts/rosenv.sh; ~/envs/isaac_env/bin/python3 scripts/ana_fsc_ring.py [--hold_x 10] BAG_DIR [BAG_DIR ..]

For the hold whose reference x is --hold_x (fly_fsc_cpp.py's 10 m point): the half-cycle peaks of the position
error and their ratios (< 1 decays, >= 1 is a limit cycle), the UDE's x estimate (its sd in still air is the
ring the estimator is fed), the attitude controller's pitch reference -> feedback lag (cross-correlation) and the
UDE x against |yaw rate| (yaw-by-tilt coupling). Written for the 2026-10-28 CAD retune (CLAUDE.md).
"""
import glob, math, sqlite3, sys
import numpy as np
from rclpy.serialization import deserialize_message
from fsc_autopilot_ros2_msgs.msg import Mocap, PositionControllerReference, AttitudeControllerState, UDEState


def eul(q):
    w, x, y, z = q.w, q.x, q.y, q.z
    r = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    p = math.asin(max(-1, min(1, 2 * (w * y - z * x))))
    yw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return np.degrees([r, p, yw])


import argparse
_ap = argparse.ArgumentParser()
_ap.add_argument('--hold_x', type=float, default=10.0)
_ap.add_argument('bags', nargs='+')
_a = _ap.parse_args()
for bag in _a.bags:
    cur = sqlite3.connect(glob.glob(bag + "/*.db3")[0]).cursor()
    tid = {n: i for i, n in cur.execute("select id,name from topics")}

    def rows(n, c):
        return [(t / 1e9, deserialize_message(d, c)) for t, d in
                cur.execute("select timestamp,data from messages where topic_id=? order by timestamp", (tid[n],))]
    ns = "/uav_0"
    moc = rows(ns + "/mocap", Mocap); t0 = moc[0][0]
    tm = np.array([t - t0 for t, _ in moc]); X = np.array([m.pose.position.x for _, m in moc])
    VX = np.array([m.twist.linear.x for _, m in moc]); WZ = np.array([m.twist.angular.z for _, m in moc])
    ref = rows(ns + "/fsc_autopilot_ros2/position_controller/reference", PositionControllerReference)
    tr = np.array([t - t0 for t, _ in ref]); RX = np.array([r.position.x for _, r in ref])
    ude = rows(ns + "/fsc_autopilot_ros2/position_controller/ude", UDEState)
    tu = np.array([t - t0 for t, _ in ude]); UX = np.array([u.disturbance_estimate.x for _, u in ude])
    UY = np.array([u.disturbance_estimate.y for _, u in ude])
    att = rows(ns + "/fsc_autopilot_ros2/attitude_controller/state", AttitudeControllerState)
    ta = np.array([t - t0 for t, _ in att])
    AR = np.array([eul(a.reference) for _, a in att]); AF = np.array([eul(a.feedback) for _, a in att])
    # the hold at x=10: reference x within 0.01 of 10 and constant
    h = np.flatnonzero(np.abs(RX - _a.hold_x) < 0.01)
    if not len(h):
        print(bag, f"no x={_a.hold_x:g} hold"); continue
    t1, t2 = tr[h[0]], tr[h[-1]]
    t1 += 1.0
    w = (tm > t1) & (tm < t2)
    e = X[w] - _a.hold_x; tt = tm[w]
    # half-cycle peaks of the position error
    pk = []
    s = np.sign(e)
    for k in np.split(np.arange(len(e)), np.flatnonzero(np.diff(s) != 0) + 1):
        if len(k) > 5:
            j = k[np.argmax(np.abs(e[k]))]; pk.append((tt[j], e[j]))
    amps = [abs(p) for _, p in pk]
    ratios = [amps[i + 1] / amps[i] for i in range(len(amps) - 1) if amps[i] > 0.02]
    per = 2 * np.mean(np.diff([t for t, _ in pk])) if len(pk) > 2 else float("nan")
    wu = (tu > t1) & (tu < t2); wa = (ta > t1) & (ta < t2)
    # which Euler channel carries the fore-aft: the one whose reference has the bigger swing
    ch = 1 if AR[wa, 1].std() > AR[wa, 0].std() else 0
    r_ = AR[wa, ch]; f_ = AF[wa, ch]; tta = ta[wa]
    # lag by cross-correlation on a 100 Hz grid
    g = np.arange(tta[0], tta[-1], 0.01); ri = np.interp(g, tta, r_) ; fi = np.interp(g, tta, f_)
    ri -= ri.mean(); fi -= fi.mean()
    lags = np.arange(0, 150)
    cc = [np.dot(ri[:len(ri) - L], fi[L:]) / (np.linalg.norm(ri[:len(ri) - L]) * np.linalg.norm(fi[L:]) + 1e-12) for L in lags]
    L = lags[int(np.argmax(cc))]
    wz = np.interp(tu[wu], tm, WZ)
    c_ux_wz = np.corrcoef(UX[wu], np.abs(wz))[0, 1] if wu.sum() > 10 else float("nan")
    c_ux_x = np.corrcoef(UX[wu], np.interp(tu[wu], tm, X) - _a.hold_x)[0, 1]
    print(f"{bag.split('/')[-1]}: hold x={_a.hold_x:g} {t1:.0f}-{t2:.0f}s  half-cycle peaks {['%+.2f' % p for _, p in pk]}")
    print(f"   ratios {['%.2f' % r for r in ratios]}  period {per:.1f}s  err rms {np.sqrt((e**2).mean()):.3f}")
    print(f"   UDE x sd {UX[wu].std():.2f} N (mean {UX[wu].mean():+.2f})  y sd {UY[wu].std():.2f}   corr(UDE x, x err) {c_ux_x:+.2f}  corr(UDE x,|yaw rate|) {c_ux_wz:+.2f}")
    print(f"   attitude ch {'pitch' if ch else 'roll'}: ref sd {r_.std():.2f} deg, fb sd {f_.std():.2f} deg, fb lags ref {L*10} ms (corr {max(cc):.2f}); |yaw rate| mean {np.degrees(np.abs(wz)).mean():.2f} deg/s")
