#!/usr/bin/env python3
"""Is the higher 'noise' in contact REAL motion? The IMU's own acceleration (PX4 sensor_combined, FRD):
rms above 5 Hz and in 1-5 Hz, free hover vs contact -- and the mocap position's content above 5 Hz in the
same windows. If both rise together, the vehicle really shakes; if only the mocap does, the solve is worse."""
import numpy as np
from vt_common import Bag, BAGS
from mocap_noise import WIN
def band(x, dt, f1, f2):
    X = np.fft.rfft(x - x.mean()); f = np.fft.rfftfreq(len(x), dt); X[(f < f1) | (f > f2)] = 0
    return np.fft.irfft(X, len(x))
for tag in BAGS:
    b = Bag(tag); ts = b.t("sc")
    acc = np.stack([b.f("sc", "accelerometer_m_s2")[:, i] for i in range(3)], 1)
    to = b.t("mocap"); p = np.stack([b.f("mocap", f"pose.position.{c}") for c in "xyz"], 1)
    print(f"===== {tag}")
    for lab, (a, c) in WIN[tag].items():
        m = (ts >= a) & (ts <= c); dt = np.median(np.diff(ts[m]))
        g = np.arange(a, c, dt); A = np.stack([np.interp(g, ts[m], acc[m, i]) for i in range(3)], 1)
        hi = np.linalg.norm([band(A[:, i], dt, 5, 0.45/dt).std() for i in range(2)]); mid = np.linalg.norm([band(A[:, i], dt, 1, 5).std() for i in range(2)])
        go = np.arange(a, c, 1/120); P = np.stack([np.interp(go, to, p[:, i]) for i in range(2)], 1)
        phi = np.linalg.norm([band(P[:, i], 1/120, 5, 50).std() for i in range(2)])
        print(f"  {lab:34s} IMU horiz accel rms: 1-5 Hz {mid:.2f}, >5 Hz {hi:.2f} m/s^2 | mocap position >5 Hz {phi*1e3:.2f} mm"
              f"  (a pos amplitude x at 5 Hz carries {(2*np.pi*5)**2*phi:.2f} m/s^2)")
