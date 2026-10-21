#!/usr/bin/env python3
"""How far does IMU dead-reckoning drift on the real T650 without the mocap?

The bridge flies on EKF2 coasting on the IMU. That drift has never been
measured on this vehicle (no flight lost vision long enough with EKF2 logged),
so this replays the RECORDED IMU instead: from many start points t0 in each
hardware flight, integrate sensor_combined's accelerometer forward through
EKF2's own attitude (vehicle_attitude) and compare with OptiTrack.

  A  bias-calibrated: a constant body-frame accelerometer bias fitted over
     [t0 - CAL_S, t0] against the mocap velocity -- roughly what EKF2 knows
     after fusing the mocap; the initial velocity is the mocap's at t0.
  B  no bias correction -- a pessimistic bound.

Not EKF2 itself (no filter, no lever arm, attitude from a vision-aided EKF2),
so read it as the order of magnitude the bridge can expect, and confirm with
the Isaac fault-injection test and a hardware hover test. Windows whose
mocap truth is not clean (gaps, freezes, flips) are skipped.

Run: PYTHONNOUSERSITE=1 /usr/bin/python3 -I coast_drift.py
"""

import glob
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ARCH = os.path.abspath(os.path.join(HERE, '..', '..'))
FLIGHTS = sorted(glob.glob(os.path.join(ARCH, 'wb_pick_drop_flight_20261014', 'npz', 'hist', '*.npz')))
G = 9.80665
CAL_S = 8.0            # [s] bias calibration window before t0
HORIZONS = (0.25, 0.5, 1.0, 2.0, 3.0, 5.0)
STEP_S = 1.0           # [s] between window starts
AIR_Z = 0.45
V_HALF = 0.05          # [s] half-width of the mocap velocity finite difference


def quat_to_R(q):
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    R = np.empty((len(q), 3, 3))
    R[:, 0, 0] = 1 - 2 * (y * y + z * z)
    R[:, 0, 1] = 2 * (x * y - w * z)
    R[:, 0, 2] = 2 * (x * z + w * y)
    R[:, 1, 0] = 2 * (x * y + w * z)
    R[:, 1, 1] = 1 - 2 * (x * x + z * z)
    R[:, 1, 2] = 2 * (y * z - w * x)
    R[:, 2, 0] = 2 * (x * z - w * y)
    R[:, 2, 1] = 2 * (y * z + w * x)
    R[:, 2, 2] = 1 - 2 * (x * x + y * y)
    return R


def cumtrapz(y, t):
    out = np.zeros_like(y)
    dt = np.diff(t)
    out[1:] = np.cumsum(0.5 * (y[1:] + y[:-1]) * dt.reshape((-1,) + (1,) * (y.ndim - 1)), axis=0)
    return out


def to_recorder_clock(stamp, recv):
    """Map PX4 sample stamps onto the recorder's (laptop) clock.

    The IMU is stamped on the Orin (or PX4-boot time), the mocap on the laptop;
    their clocks were 0.348 s apart on 09-18 and the Orin's stepped +0.6 s on
    10-02. A rolling median of (arrival - stamp) over 10 s follows both; what is
    left is the few-ms transport latency."""
    d = recv - stamp
    grid = np.arange(stamp[0], stamp[-1] + 1.0, 1.0)
    off = np.array([np.median(d[(stamp > g - 5.0) & (stamp < g + 5.0)]) if
                    np.any((stamp > g - 5.0) & (stamp < g + 5.0)) else np.nan for g in grid])
    ok = np.isfinite(off)
    return stamp + np.interp(stamp, grid[ok], off[ok])


def load(path):
    z = np.load(path, allow_pickle=True)
    if 'sc__timestamp' not in z.files or 'vatt__q' not in z.files:
        return None
    ts = to_recorder_clock(z['sc__timestamp'] * 1e-6, z['sc__recv']) + \
        z['sc__accelerometer_timestamp_relative'] * 1e-6
    f = z['sc__accelerometer_m_s2'].astype(float)
    o = np.argsort(ts)
    ts, f = ts[o], f[o]
    keep = np.r_[True, np.diff(ts) > 1e-5]
    ts, f = ts[keep], f[keep]
    ta = to_recorder_clock(z['vatt__timestamp_sample'] * 1e-6, z['vatt__recv'])
    qa = z['vatt__q'].astype(float)
    o = np.argsort(ta)
    ta, qa = ta[o], qa[o]
    # hemisphere continuity, then normalized lerp onto the IMU times
    for i in range(1, len(qa)):
        if np.dot(qa[i], qa[i - 1]) < 0:
            qa[i] = -qa[i]
    q = np.c_[[np.interp(ts, ta, qa[:, j]) for j in range(4)]].T
    q /= np.linalg.norm(q, axis=1, keepdims=True)
    R_ned = quat_to_R(q)                       # FRD body -> NED world
    a_ned = np.einsum('nij,nj->ni', R_ned, f) + np.array([0.0, 0.0, G])
    P = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])   # NED -> ENU
    a = a_ned @ P.T
    R = np.einsum('ij,njk->nik', P, R_ned)     # FRD body -> ENU world
    tm = z['mocap__hdr'].astype(float)
    pm = np.c_[z['mocap__pose.position.x'], z['mocap__pose.position.y'], z['mocap__pose.position.z']]
    o = np.argsort(tm)
    tm, pm = tm[o], pm[o]
    keep = np.r_[True, np.diff(tm) > 1e-5]
    return dict(ts=ts, a=a, R=R, tm=tm[keep], pm=pm[keep], ta=ta)


IMU_GAP_MAX = 0.020    # [s] the recorded IMU crossed the WiFi to the laptop recorder: a gap
                       # there is a recording artefact (EKF2 on the Pixhawk had every
                       # sample), and integrating across it reads as drift


def imu_clean(ts, ta, t_a, t_b):
    for tt in (ts, ta):
        i, j = np.searchsorted(tt, [t_a, t_b])
        if j - i < 10 or np.diff(tt[max(i - 1, 0):j + 1]).max() > IMU_GAP_MAX:
            return False
    return True


def mocap_clean(tm, pm, t_a, t_b):
    i, j = np.searchsorted(tm, [t_a, t_b])
    if j - i < 10:
        return False
    seg_t, seg_p = tm[i:j], pm[i:j]
    if np.diff(seg_t).max() > 0.10 or seg_p[:, 2].min() < AIR_Z:
        return False
    steps = np.linalg.norm(np.diff(seg_p, axis=0), axis=1)
    if steps.max() > 0.030:          # a flip / jump in the truth
        return False
    same = np.all(np.diff(seg_p, axis=0) == 0, axis=1)
    return not (same[1:] & same[:-1]).any()      # 3+ identical = a freeze; repeated pairs are normal


def interp3(t, tt, yy):
    return np.c_[[np.interp(t, tt, yy[:, k]) for k in range(3)]].T


def main():
    errs = {v: {h: [] for h in HORIZONS} for v in ('A', 'B')}
    bias = []
    per_flight = []
    for path in FLIGHTS:
        d = load(path)
        if d is None:
            continue
        ts, a, R, tm, pm = d['ts'], d['a'], d['R'], d['tm'], d['pm']
        A_cum = cumtrapz(a, ts)
        R_cum = cumtrapz(R, ts)
        H = max(HORIZONS)
        t_lo = max(ts[0], tm[0]) + CAL_S + 1.0
        t_hi = min(ts[-1], tm[-1]) - H - 0.2
        n_win = 0
        n_gap = 0
        e1 = []
        for t0 in np.arange(t_lo, t_hi, STEP_S):
            if not mocap_clean(tm, pm, t0 - CAL_S, t0 + H):
                continue
            if not imu_clean(ts, d['ta'], t0 - CAL_S, t0 + H):
                n_gap += 1
                continue
            # calibration: 0.5 s sub-intervals, dv_mocap = int(a) - int(R) b
            rows, rhs = [], []
            for s in np.arange(t0 - CAL_S, t0 - 0.5 + 1e-9, 0.5):
                i, j = np.searchsorted(ts, [s, s + 0.5])
                pts = interp3(np.array([s + 0.5 + V_HALF, s + 0.5 - V_HALF, s + V_HALF, s - V_HALF]), tm, pm)
                dv = (pts[0] - pts[1]) / (2 * V_HALF) - (pts[2] - pts[3]) / (2 * V_HALF)
                rows.append(R_cum[j] - R_cum[i])
                rhs.append((A_cum[j] - A_cum[i]) - dv)
            b, *_ = np.linalg.lstsq(np.vstack(rows), np.concatenate(rhs), rcond=None)
            bias.append(b)
            i0, i1 = np.searchsorted(ts, [t0, t0 + H + 0.01])
            tt = ts[i0:i1]
            p0 = interp3(np.array([t0]), tm, pm)[0]
            v0 = ((interp3(np.array([t0 + V_HALF]), tm, pm) - interp3(np.array([t0 - V_HALF]), tm, pm))
                  / (2 * V_HALF))[0]
            truth = interp3(tt, tm, pm)
            for variant, bb in (('A', b), ('B', np.zeros(3))):
                acc = a[i0:i1] - np.einsum('nij,j->ni', R[i0:i1], bb)
                v = v0 + cumtrapz(acc, tt)
                p = p0 + cumtrapz(v, tt)
                for h in HORIZONS:
                    k = np.searchsorted(tt, t0 + h)
                    k = min(k, len(tt) - 1)
                    e = p[k] - truth[k]
                    errs[variant][h].append((np.hypot(e[0], e[1]), abs(e[2])))
                    if variant == 'A' and h == 1.0:
                        e1.append(np.hypot(e[0], e[1]))
            n_win += 1
        if n_win:
            per_flight.append((os.path.basename(path)[7:-4], n_win, np.percentile(e1, 50) * 1e3,
                               np.percentile(e1, 90) * 1e3, np.percentile(e1, 99) * 1e3, n_gap))
    print(f'{len(per_flight)} flights, {sum(f[1] for f in per_flight)} windows '
          f'(start every {STEP_S:.0f} s, bias calibrated over the {CAL_S:.0f} s before)\n')
    for variant, title in (('A', 'A  bias-calibrated (EKF2-like)'), ('B', 'B  no bias correction')):
        print(title)
        print(f'  {"horizon":>8s} | {"horizontal mm  p50   p90   p99   max":>40s} | {"vertical mm  p50   p90   p99":>30s}')
        for h in HORIZONS:
            x = np.array(errs[variant][h]) * 1e3
            hx, vz = x[:, 0], x[:, 1]
            print(f'  {h:7.2f}s | {np.percentile(hx, 50):18.1f} {np.percentile(hx, 90):5.1f} '
                  f'{np.percentile(hx, 99):5.1f} {hx.max():6.1f} | {np.percentile(vz, 50):14.1f} '
                  f'{np.percentile(vz, 90):5.1f} {np.percentile(vz, 99):5.1f}')
        print()
    bb = np.array(bias)
    print('fitted body-frame accelerometer bias [m/s^2]: median', np.round(np.median(bb, axis=0), 3),
          ' p90 |b|', round(float(np.percentile(np.linalg.norm(bb, axis=1), 90)), 3))
    print('\nper flight, variant A, horizontal error at 1 s (mm): windows / p50 / p90 / p99 '
          '/ windows skipped for an IMU recording gap')
    for f in per_flight:
        print(f'  {f[0]:46s} {f[1]:4d} {f[2]:7.1f} {f[3]:7.1f} {f[4]:7.1f} {f[5]:4d}')


if __name__ == '__main__':
    main()
