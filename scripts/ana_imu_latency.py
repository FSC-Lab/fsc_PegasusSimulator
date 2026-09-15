#!/usr/bin/env python3
"""The accelerometer's age at the node, from a bag: PX4's `sensor_combined` (FRD body, specific
force) against the specific force reconstructed from the MOCAP (the controller's clock: R^T (dv/dt
+ g) from /uav_0/mocap), cross-correlated over the wing-borne part of the flight, per axis. The lag
that comes out is what `IMU_DELAY` on nmpc_cascade_imu_node should carry. Also prints the two
streams' rates and the sensor's rms noise about the mocap-derived truth.

    ~/envs/isaac_env/bin/python3 scripts/ana_imu_latency.py <bag dir or db3>
"""
import glob, os, sys, sqlite3
import numpy as np
sys.path.insert(0, os.path.expanduser("~/envs/fsc_autopilot_ros2-master/px4_vtol_direct_actuator/px4_vtol_direct_actuator"))
from rclpy.serialization import deserialize_message
from fsc_autopilot_ros2_msgs.msg import Mocap
from px4_msgs.msg import SensorCombined

G = 9.80665
bag = sys.argv[1]
if os.path.isdir(bag):
    bag = glob.glob(os.path.join(bag, "*.db3"))[0]
cur = sqlite3.connect(bag).cursor()
tid = {n: i for n, i in cur.execute("select name, id from topics")}
t0 = cur.execute("select min(timestamp) from messages").fetchone()[0]


def rows(name, cls):
    return [((ts - t0) / 1e9, deserialize_message(d, cls)) for ts, d in
            cur.execute("select timestamp, data from messages where topic_id=? order by timestamp", (tid[name],))]


def Rm(q):   # geometry_msgs quaternion (x, y, z, w) -> rotation matrix, body FLU -> world ENU
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


ns = [k for k in tid if k.endswith("/mocap")][0].rsplit("/", 1)[0]
moc = rows(f"{ns}/mocap", Mocap)
imu = rows(f"{ns}/fmu/out/sensor_combined", SensorCombined)
tm = np.array([t for t, _ in moc]); V = np.array([[m.twist.linear.x, m.twist.linear.y, m.twist.linear.z] for _, m in moc])
R = [Rm(m.pose.orientation) for _, m in moc]
ti = np.array([t for t, _ in imu]); A = np.array([[m.accelerometer_m_s2[0], -m.accelerometer_m_s2[1], -m.accelerometer_m_s2[2]] for _, m in imu])  # FRD -> FLU
print(f"mocap {len(tm)} msgs at {len(tm) / (tm[-1] - tm[0]):.1f} Hz, sensor_combined {len(ti)} at {len(ti) / (ti[-1] - ti[0]):.1f} Hz")
# the mocap-derived specific force in the body frame (central difference of the world velocity)
dv = np.gradient(V, tm, axis=0) + np.array([0.0, 0.0, G])
Fm = np.array([R[i].T @ dv[i] for i in range(len(tm))])
# wing-borne window: |v| > 12 m/s
spd = np.linalg.norm(V[:, :2], axis=1); w = spd > 12.0
if w.sum() < 100:
    print("no wing-borne window"); sys.exit(0)
ta, tb = tm[w][0], tm[w][-1]
print(f"window {ta:.0f}-{tb:.0f} s (|v| > 12 m/s)")
# resample both onto a 200 Hz grid, low-pass both at 10 Hz (the comparison is about phase at the
# tank's 1 Hz and the loop's few Hz, not the IMU's white noise), cross-correlate per axis
fs = 200.0; tg = np.arange(ta, tb, 1.0 / fs)
def lp(x, fc):
    y = np.zeros_like(x); a = 1.0 - np.exp(-2 * np.pi * fc / fs); y[0] = x[0]
    for i in range(1, len(x)): y[i] = y[i - 1] + a * (x[i] - y[i - 1])
    return y
for ax, name in enumerate("xyz"):
    m = lp(np.interp(tg, tm, Fm[:, ax]), 10.0); s = lp(np.interp(tg, ti, A[:, ax]), 10.0)
    m = m - m.mean(); s = s - s.mean()
    lags = np.arange(-60, 61)   # +-300 ms
    cc = [np.corrcoef(m[max(0, -k):len(m) - max(0, k)], s[max(0, k):len(s) - max(0, -k)])[0, 1] for k in lags]
    k = lags[int(np.argmax(cc))]
    raw_err = np.interp(tg, ti, A[:, ax]) - np.interp(tg, tm, Fm[:, ax])
    print(f"  {name}: peak corr {max(cc):+.3f} at IMU lag {1e3 * k / fs:+.0f} ms behind the mocap (positive = the accelerometer is older);"
          f"  rms of the truth {np.sqrt(np.mean(np.interp(tg, tm, Fm[:, ax])**2)):.2f} m/s^2, sensor - truth rms {np.sqrt(np.mean(raw_err**2)):.3f}")
