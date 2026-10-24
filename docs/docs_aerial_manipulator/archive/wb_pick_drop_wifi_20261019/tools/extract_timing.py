#!/usr/bin/env python3
"""rosbag2 -> npz of per-message TIMING for the 2026-10-19 WiFi investigation.

For every topic: the bag's receive time (`<key>__recv`, ns since epoch, the recorder's clock),
and, when the message has one, its header stamp (`<key>__stamp`, ns) or PX4 `timestamp`
(`<key>__px4ts`, us since PX4 boot).  A few topics also keep their payload:
strings (mode, status, controller type, mocap/fused status), timesync, vehicle status,
battery, rosout, the gripper action status, wb_control_debug[0..2] and joint positions.

usage: extract_timing.py <bag dir> <out.npz>      (ROS 2 + ~/ros2_ws/install sourced, /usr/bin/python3)
"""
import os
import sqlite3
import sys

import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def key_of(name):
    return name.strip("/").replace("/", "__")


def main(bag, out):
    db = [f for f in os.listdir(bag) if f.endswith(".db3")][0]
    con = sqlite3.connect(os.path.join(bag, db))
    cur = con.cursor()
    topics = cur.execute("SELECT id, name, type FROM topics").fetchall()
    res = {}
    for tid, name, typ in topics:
        k = key_of(name)
        try:
            cls = get_message(typ)
        except (ModuleNotFoundError, AttributeError, ValueError) as e:
            print(f"   skip {name} ({typ}): {e}")
            continue
        recv, stamp, px4ts, extra = [], [], [], {}
        rows = con.cursor().execute("SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp", (tid,))
        for ts, blob in rows:
            m = deserialize_message(bytes(blob), cls)
            recv.append(ts)
            if hasattr(m, "header"):
                stamp.append(m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec)
            if hasattr(m, "timestamp") and isinstance(getattr(m, "timestamp"), int):
                px4ts.append(m.timestamp)
            if typ == "std_msgs/msg/String":
                extra.setdefault("data", []).append(m.data)
            elif typ == "rcl_interfaces/msg/Log":
                extra.setdefault("name", []).append(m.name)
                extra.setdefault("msg", []).append(m.msg)
                extra.setdefault("level", []).append(m.level)
                extra.setdefault("logstamp", []).append(m.stamp.sec * 1_000_000_000 + m.stamp.nanosec)
            elif typ == "px4_msgs/msg/TimesyncStatus":
                extra.setdefault("rtt", []).append(m.round_trip_time)
                extra.setdefault("est_off", []).append(m.estimated_offset)
                extra.setdefault("obs_off", []).append(m.observed_offset)
                extra.setdefault("remote", []).append(m.remote_timestamp)
            elif typ == "px4_msgs/msg/VehicleStatus":
                extra.setdefault("nav_state", []).append(m.nav_state)
                extra.setdefault("arming_state", []).append(m.arming_state)
                extra.setdefault("failsafe", []).append(int(m.failsafe))
                extra.setdefault("rc_lost", []).append(int(getattr(m, "rc_signal_lost", getattr(m, "gcs_connection_lost", False))))
                extra.setdefault("offboard_lost", []).append(int(getattr(m, "offboard_control_signal_lost", False)))
            elif typ == "px4_msgs/msg/BatteryStatus":
                extra.setdefault("voltage", []).append(m.voltage_v)
                extra.setdefault("current", []).append(m.current_a)
            elif typ == "action_msgs/msg/GoalStatusArray":
                extra.setdefault("status", []).append(";".join(str(s.status) for s in m.status_list))
            elif typ == "std_msgs/msg/Float32MultiArray" and "wb_control_debug" in name:
                extra.setdefault("d0", []).append(list(m.data[:3]) if len(m.data) >= 3 else [np.nan] * 3)
                extra.setdefault("n", []).append(len(m.data))
            elif typ == "sensor_msgs/msg/JointState" and name.endswith("/joint_states"):
                extra.setdefault("pos", []).append(list(m.position[:5]) + [np.nan] * (5 - len(m.position[:5])))
                extra.setdefault("names", []).append(",".join(m.name))
            elif typ == "dynamixel_interfaces/msg/DynamixelState":
                extra.setdefault("comm_state", []).append(m.comm_state)
                extra.setdefault("hw_state", []).append(",".join(str(x) for x in m.dxl_hw_state))
            elif typ == "px4_msgs/msg/VehicleOdometry" and "vehicle_visual_odometry" in name:
                extra.setdefault("ts_sample", []).append(m.timestamp_sample)
            elif typ in ("fsc_autopilot_ros2_msgs/msg/Mocap", "geometry_msgs/msg/PoseStamped") and ("mocap" in name or "vrpn" in name):
                p = m.pose.position
                extra.setdefault("pos", []).append([p.x, p.y, p.z])
        res[k + "__recv"] = np.array(recv, dtype=np.int64)
        if stamp:
            res[k + "__stamp"] = np.array(stamp, dtype=np.int64)
        if px4ts:
            res[k + "__px4ts"] = np.array(px4ts, dtype=np.int64)
        for f, v in extra.items():
            if v and isinstance(v[0], str):
                res[k + "__" + f] = np.array(v, dtype=object)
            else:
                res[k + "__" + f] = np.array(v)
        res[k + "__type"] = np.array(typ, dtype=object)
        print(f"{len(recv):7d} {name}")
    np.savez_compressed(out, **res)


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
