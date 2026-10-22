#!/usr/bin/env python3
"""rosbag2 -> npz for the 2026-10-21 box-pushing flights: the 10-09 generic extractor plus the
box-pushing planner topics, the raw-mocap guard's health and the gripper action status.
Usage: extract.py <bag dir> <out.npz>   (ROS 2 + ~/ros2_ws/install sourced, /usr/bin/python3;
then ../../wb_pick_place_flight_20261009/tools/np1_compat.py for the numpy-1 plotting python).
Reads the .db3 directly, so a bag whose recorder died without writing metadata.yaml works too."""
import os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "wb_pick_place_flight_20261009", "tools"))
import extract_bag as E  # noqa: E402

P = "/uav_0"
E.TOPICS.update({
    "pl_pstatus": f"{P}/whole_body_planner/push_pull/status",
    "pl_info":    f"{P}/whole_body_planner/push_pull/info",
    "pl_arr":     f"{P}/whole_body_planner/push_pull/arrival_error",
    "pl_path":    f"{P}/whole_body_planner/push_pull/path",
    "pl_dpath":   f"{P}/whole_body_planner/push_pull/drone_path",
    "guard":      f"{P}/state_estimator/mocap_guard",
    "grip_status": f"{P}/fsc_open_manipulator/gripper_controller/gripper_cmd/_action/status",
    "imu":        f"{P}/state_estimator/enu/imu/data",
})

if __name__ == "__main__":
    E.main(sys.argv[1], sys.argv[2])
