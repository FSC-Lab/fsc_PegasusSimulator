#!/usr/bin/env python3
"""rosbag2 -> npz for the 2026-10-28 valve-turning flights: the 10-21 box-pushing extractor (the valve
uses the same push-and-pull planner mode and the same raw-mocap guard) plus the planner's claw
heading / skeleton and the Orin system monitor.  The two bags were recorded with `ros2 bag record -a`,
so the hidden gripper action topics are absent (the gripper's state comes from joint_states).
Usage: extract.py <bag dir> <out.npz>   (ROS 2 + ~/ros2_ws/install sourced, /usr/bin/python3)."""
import os, sys, importlib.util
HERE = os.path.dirname(os.path.abspath(__file__))
_spec = importlib.util.spec_from_file_location(
    "extract_1021", os.path.join(HERE, "..", "..", "wb_box_pushing_flight_20261021", "tools", "extract.py"))
X = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(X)
P = "/uav_0"
X.E.TOPICS.update({
    "ee_heading": f"{P}/whole_body_planner/current_ee_heading",
    "skeleton":   f"{P}/whole_body_planner/current_skeleton",
    "sysmon":     f"{P}/system_monitor/status",
    "pcref":      f"{P}/fsc_autopilot_ros2/position_controller/reference",
    "pevents":    "/parameter_events",
    "tvsp":       f"{P}/fmu/in/vehicle_torque_setpoint",
})
if __name__ == "__main__":
    X.E.main(sys.argv[1], sys.argv[2])
