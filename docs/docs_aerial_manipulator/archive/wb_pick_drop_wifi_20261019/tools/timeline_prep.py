#!/usr/bin/env python3
"""Series for the 2026-10-19 failure timeline -> ../npz/timeline.npz (plain float arrays, numpy-1 readable).

Stream health = messages per 0.25 s bin / nominal, by RECEIVE time at the laptop recorder:
  PX4 -> Agent -> ROS   /uav_0/fmu/out/vehicle_attitude (400 Hz)
  Orin -> laptop WiFi   /uav_0/fsc_open_manipulator/joint_states (250 Hz, published on the Orin)
  laptop -> Orin WiFi   /uav_0/fmu/in/vehicle_visual_odometry (120 Hz: the Orin estimator relays the
                        laptop's mocap, so it exists only if mocap crossed the WiFi to the Orin)
Vehicle height and tilt from /vrpn_mocap/uav_0/pose (laptop, never stopped); joint 2 from joint_states.

usage: timeline_prep.py      (ROS 2 sourced, /usr/bin/python3; reads ../npz/pd.npz and the bag)
"""
import os
import sqlite3

import numpy as np
from geometry_msgs.msg import PoseStamped
from rclpy.serialization import deserialize_message

HERE = os.path.dirname(os.path.abspath(__file__))
NPZ = os.path.join(HERE, "..", "npz")
BAG = os.path.join(HERE, "..", "..", "..", "..", "experimental_data_ros2_bag",
                   "1019 - T650-AM whole-body Pick&Drop [Fail, WIFI]-20261019T230203Z-1-001",
                   "1019 - T650-AM whole-body Pick&Drop [Fail, WIFI]", "flight_wb_l1_4d_pick_and_drop_20261019_170849")


def main():
    d = np.load(os.path.join(NPZ, "pd.npz"), allow_pickle=True)
    t0 = min(d[k][0] for k in d.files if k.endswith("__recv") and len(d[k]))
    edges = np.arange(0.0, 85.0, 0.25)
    out = {"bin_t": edges[:-1] + 0.125}
    for key, topic, hz in (("px4", "uav_0__fmu__out__vehicle_attitude", 400.0),
                           ("orin_to_laptop", "uav_0__fsc_open_manipulator__joint_states", 250.0),
                           ("laptop_to_orin", "uav_0__fmu__in__vehicle_visual_odometry", 120.0)):
        t = (d[topic + "__recv"] - t0) / 1e9
        out[key] = np.histogram(t, edges)[0] / (hz * 0.25)
    js = "uav_0__fsc_open_manipulator__joint_states"
    out["js_t"] = (d[js + "__recv"] - t0) / 1e9
    out["q2_deg"] = np.degrees(d[js + "__pos"][:, 0].astype(float))  # broadcaster order: joint2 first

    db = [f for f in os.listdir(BAG) if f.endswith(".db3")][0]
    c = sqlite3.connect(os.path.join(BAG, db)).cursor()
    tid = c.execute("SELECT id FROM topics WHERE name='/vrpn_mocap/uav_0/pose'").fetchone()[0]
    T, Z, TILT = [], [], []
    for ts, blob in c.execute("SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp", (tid,)):
        m = deserialize_message(bytes(blob), PoseStamped)
        q = m.pose.orientation
        T.append((ts - t0) / 1e9)
        Z.append(m.pose.position.z)
        TILT.append(np.degrees(np.arccos(np.clip(1 - 2 * (q.x * q.x + q.y * q.y), -1, 1))))
    out["mc_t"], out["z"], out["tilt"] = np.array(T), np.array(Z), np.array(TILT)
    np.savez(os.path.join(NPZ, "timeline.npz"), **{k: np.asarray(v, dtype=float) for k, v in out.items()})
    print("wrote", os.path.join(NPZ, "timeline.npz"))


if __name__ == "__main__":
    main()
