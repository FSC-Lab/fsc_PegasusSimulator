#!/usr/bin/env python3
"""PX4 -> Orin stream health across the hardware bags 10-09..10-20: stalls in /fmu/out/sensor_combined
(400 Hz) on PX4's own clock -- outages (>= 1 s), stalls (> 20 ms) and missing samples -- each split into
before arming / in flight is not attempted: whole bag.  Run from docs/experimental_data_ros2_bag with ROS 2
sourced, /usr/bin/python3.  (10-19 tool, extended with the stall count and the 10-09/10/20 bags.)"""
import sqlite3, glob, numpy as np
from rclpy.serialization import deserialize_message
from px4_msgs.msg import SensorCombined
for db in sorted(glob.glob("**/*.db3", recursive=True)):
    if db.split("/")[0][:4] not in ("1009", "1010", "1014", "1015", "1016", "1019", "1020"):
        continue
    c = sqlite3.connect(db).cursor()
    r = c.execute("SELECT id FROM topics WHERE name='/uav_0/fmu/out/sensor_combined'").fetchone()
    if not r:
        continue
    ts = np.array([deserialize_message(bytes(b), SensorCombined).timestamp for (b,) in
                   c.execute("SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp", (r[0],))], dtype=np.int64)
    if len(ts) < 1000:
        continue
    d = np.diff(ts) / 1e3
    per = np.median(d)
    st = d[(d > 20) & (d < 1000)]
    miss = int(np.sum(np.round(d[d > 1.5 * per] / per) - 1))
    print(f"{db.split('/')[-2][:50]:50s} {len(ts)/400:6.1f} s  outages>=1s {int(np.sum(d >= 1000))} "
          f"(longest {d.max()/1e3:5.1f} s)  stalls 20ms-1s {len(st):2d} (max {st.max() if len(st) else 0:4.0f} ms)  "
          f"missing {miss:6d}")
