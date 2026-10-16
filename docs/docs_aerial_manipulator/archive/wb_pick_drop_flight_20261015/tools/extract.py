#!/usr/bin/env python3
"""rosbag2 -> npz for the 2026-10-15 pick-and-drop flights: the 10-09 extractor minus the 400 Hz IMU topics
(sensor_combined, imu/data) that this analysis does not use.  Usage: extract.py <bag dir> <out.npz>
Run with ROS 2 + ~/ros2_ws/install sourced, /usr/bin/python3; then ../../wb_pick_place_flight_20261009/tools/np1_compat.py."""
import os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "wb_pick_place_flight_20261009", "tools"))
import extract_bag as E  # noqa: E402

E.TOPICS = {k: v for k, v in E.TOPICS.items() if k not in ("sc",)}
E.TOPICS["grip_status"] = "/uav_0/fsc_open_manipulator/gripper_controller/gripper_cmd/_action/status"

if __name__ == "__main__":
    E.main(sys.argv[1], sys.argv[2])
