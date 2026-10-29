#!/usr/bin/env python3
"""The drone's attitude as two sources give it, free vs contact: the mocap rigid body (what
wb_attitude_from_odometry would feed the law) and PX4 (what the law uses): each one's content above
2 Hz (noise for a vehicle whose true attitude motion there is small), and their disagreement in the
sway band 0.4-0.8 Hz."""
import numpy as np
from vt_common import Bag, BAGS
from mocap_noise import WIN, R_of, NE, P
def band(x, dt, f1, f2):
    X = np.fft.rfft(x - x.mean()); f = np.fft.rfftfreq(len(x), dt); X[(f < f1) | (f > f2)] = 0
    return np.fft.irfft(X, len(x))
dt = 0.005
for tag in BAGS:
    b = Bag(tag); tv = b.t("vatt"); qv = b.f("vatt", "q"); tm = b.t("mocap")
    qm = np.stack([b.f("mocap", f"pose.orientation.{c}") for c in "wxyz"], 1)
    print(f"===== {tag}")
    for lab, (a, c) in WIN[tag].items():
        g = np.arange(a, c, dt)
        zm = np.array([R_of(*qm[min(np.searchsorted(tm, t), len(tm)-1)])[:2, 2] for t in g])
        zp = np.array([(NE @ R_of(*qv[min(np.searchsorted(tv, t), len(tv)-1)]) @ P)[:2, 2] for t in g])
        hm = np.degrees(np.linalg.norm([band(zm[:, i], dt, 2, 40).std() for i in range(2)]))
        hp = np.degrees(np.linalg.norm([band(zp[:, i], dt, 2, 40).std() for i in range(2)]))
        dis = np.degrees(np.linalg.norm([band(zp[:, i] - zm[:, i], dt, 0.4, 0.8).std() for i in range(2)]))
        print(f"  {lab:34s} tilt content > 2 Hz: mocap {hm:.2f} deg, PX4 {hp:.2f} deg | PX4 - mocap in 0.4-0.8 Hz {dis:.2f} deg")
