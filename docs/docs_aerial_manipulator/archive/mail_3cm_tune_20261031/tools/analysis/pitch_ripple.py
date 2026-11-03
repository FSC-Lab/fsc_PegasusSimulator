#!/usr/bin/env python3
"""The pitch RIPPLE in the place: how hard the attitude loop is oscillating while the claw holds
the envelope at the mouth, per flight (any bag with wb_control_debug -- slots [18] tau_body and
[28] e_R are in every bag of this study):

    cd runs && /usr/bin/python3 ../tools/analysis/pitch_ripple.py [tag ...]   (default: every c3_/c4_ bag)

Window: execute_place start .. the gripper's open or the abort (whichever first). Printed: the
band-passed (0.5-3 Hz) RMS of the pitch moment [N.m] and of the pitch error [deg], and the
frequency of the largest spectral peak of the moment in that band.
"""
import glob
import re
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from std_msgs.msg import Float32MultiArray

TOP = '/uav_0/fsc_autopilot_ros2/whole_body_direct_actuation/wb_control_debug'


def read(bag):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    r.set_filter(rosbag2_py.StorageFilter(topics=[TOP]))
    ts, rows = [], []
    while r.has_next():
        _, raw, st = r.read_next()
        d = deserialize_message(raw, Float32MultiArray).data
        if len(d) > 30:
            rows.append((d[18], d[28]))
            ts.append(st)
    return np.array(ts, dtype=np.int64), np.array(rows, dtype=float)


def first(ev, pat, after=-1.0):
    for m in re.finditer(r'^\s*([\d.]+) .*' + pat, ev, re.M):
        if float(m.group(1)) > after:
            return float(m.group(1))
    return None


def band(x, fs, lo=0.5, hi=3.0):
    X = np.fft.rfft(x - x.mean())
    f = np.fft.rfftfreq(len(x), 1 / fs)
    m = (f >= lo) & (f <= hi)
    y = np.fft.irfft(np.where(m, X, 0), len(x))
    return y, f[m][np.abs(X[m]).argmax()] if m.any() else np.nan


def main():
    tags = sys.argv[1:] or sorted(re.sub(r'\.bag$', '', b) for b in glob.glob('c[34]_*.bag'))
    print(f"{'flight':14s} {'result':8s} {'win s':>6s} | ripple rms: moment N.m  e_R deg | peak Hz")
    for tag in tags:
        try:
            ev = open(f'work/events_{tag}.txt').read()
        except OSError:
            continue
        t0 = int(re.search(r't0_ns (\d+)', ev).group(1))
        a = first(ev, r'execute_place: executing')
        if a is None:
            continue
        ab = first(ev, r'ABORTING', a)
        op = first(ev, r'isaac_gripper: goal: position 0\.0190', a)
        b = min(x for x in (ab, op, first(ev, r'mode: SAFETY', a), 1e9) if x is not None)
        ts, X = read(f'{tag}.bag')
        t = (ts - t0) / 1e9
        m = (t >= a) & (t <= b - 0.5)              # stop short of the abort's own transient
        if m.sum() < 500:
            continue
        fs = (m.sum() - 1) / (t[m][-1] - t[m][0])
        tau_b, fpk = band(X[m, 0], fs)
        e_b, _ = band(np.degrees(X[m, 1]), fs)
        print(f"{tag:14s} {'abort' if ab and (op is None or ab < op + 0.05) else 'ok':8s} {t[m][-1] - t[m][0]:6.1f} |"
              f"            {np.sqrt((tau_b ** 2).mean()):6.3f}  {np.sqrt((e_b ** 2).mean()):6.2f} | {fpk:5.2f}")


if __name__ == '__main__':
    main()
