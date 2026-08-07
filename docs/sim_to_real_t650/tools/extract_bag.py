#!/usr/bin/env python3
"""Extract PX4 topics from a rosbag2 sqlite3 bag into a .npz for offline analysis."""
import sqlite3
import sys
import os
import numpy as np
from rclpy.serialization import deserialize_message
from px4_msgs.msg import (
    SensorCombined,
    VehicleAttitudeSetpoint,
    VehicleAttitude,
    VehicleStatus,
)

TYPES = {
    "px4_msgs/msg/SensorCombined": SensorCombined,
    "px4_msgs/msg/VehicleAttitudeSetpoint": VehicleAttitudeSetpoint,
    "px4_msgs/msg/VehicleAttitude": VehicleAttitude,
    "px4_msgs/msg/VehicleStatus": VehicleStatus,
}


def read_bag(bagdir):
    db3 = [f for f in os.listdir(bagdir) if f.endswith(".db3")]
    con = sqlite3.connect(os.path.join(bagdir, db3[0]))
    cur = con.cursor()
    topics = {}
    for tid, name, tname in cur.execute("SELECT id, name, type FROM topics"):
        topics[tid] = (name, tname)
    out = {}
    for tid, (name, tname) in topics.items():
        cls = TYPES.get(tname)
        if cls is None:
            continue
        rows = cur.execute(
            "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp", (tid,)
        ).fetchall()
        out[name] = [(t, deserialize_message(bytes(d), cls)) for t, d in rows]
    con.close()
    return out


def main(bagdir, outnpz):
    data = read_bag(bagdir)
    arrays = {}

    ns = "/uav_0"

    # --- sensor_combined ---
    key = f"{ns}/fmu/out/sensor_combined"
    if key in data:
        msgs = data[key]
        arrays["sc_bag_t"] = np.array([t for t, _ in msgs], dtype=np.int64)
        arrays["sc_ts"] = np.array([m.timestamp for _, m in msgs], dtype=np.int64)
        arrays["sc_gyro"] = np.array([m.gyro_rad for _, m in msgs], dtype=np.float64)
        arrays["sc_gyro_dt"] = np.array(
            [m.gyro_integral_dt for _, m in msgs], dtype=np.float64
        )
        arrays["sc_accel"] = np.array(
            [m.accelerometer_m_s2 for _, m in msgs], dtype=np.float64
        )
        arrays["sc_accel_dt"] = np.array(
            [m.accelerometer_integral_dt for _, m in msgs], dtype=np.float64
        )

    # --- attitude setpoints (both the debug echo and what actually went to PX4) ---
    for label, key in [
        ("sp", f"{ns}/fmu/in/vehicle_attitude_setpoint"),
        ("spdbg", f"{ns}/fsc_autopilot_ros2/attitude_setpoint_debug"),
    ]:
        if key not in data:
            continue
        msgs = data[key]
        arrays[f"{label}_bag_t"] = np.array([t for t, _ in msgs], dtype=np.int64)
        arrays[f"{label}_ts"] = np.array([m.timestamp for _, m in msgs], dtype=np.int64)
        arrays[f"{label}_q_d"] = np.array([m.q_d for _, m in msgs], dtype=np.float64)
        arrays[f"{label}_thrust"] = np.array(
            [m.thrust_body for _, m in msgs], dtype=np.float64
        )
        arrays[f"{label}_yaw_sp_move_rate"] = np.array(
            [m.yaw_sp_move_rate for _, m in msgs], dtype=np.float64
        )

    # --- vehicle attitude ---
    key = f"{ns}/fmu/out/vehicle_attitude"
    if key in data:
        msgs = data[key]
        arrays["att_bag_t"] = np.array([t for t, _ in msgs], dtype=np.int64)
        arrays["att_ts"] = np.array([m.timestamp for _, m in msgs], dtype=np.int64)
        arrays["att_q"] = np.array([m.q for _, m in msgs], dtype=np.float64)

    # --- vehicle status ---
    key = f"{ns}/fmu/out/vehicle_status_v1"
    if key in data:
        msgs = data[key]
        arrays["st_bag_t"] = np.array([t for t, _ in msgs], dtype=np.int64)
        arrays["st_ts"] = np.array([m.timestamp for _, m in msgs], dtype=np.int64)
        arrays["st_arming"] = np.array(
            [m.arming_state for _, m in msgs], dtype=np.int32
        )
        arrays["st_navstate"] = np.array([m.nav_state for _, m in msgs], dtype=np.int32)

    np.savez_compressed(outnpz, **arrays)
    print(f"wrote {outnpz}")
    for k, v in sorted(arrays.items()):
        print(f"  {k:22s} {v.shape}")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
