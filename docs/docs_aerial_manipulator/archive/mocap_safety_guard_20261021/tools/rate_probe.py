#!/usr/bin/env python3
"""Publish rates of the sim's PX4 topics and the drone mocap over 10 s (direct subscriber;
`ros2 topic hz` sees nothing on this desktop). Run with the workspace sourced, during a flight."""
import time, rclpy
from rclpy.qos import qos_profile_sensor_data
from px4_msgs.msg import SensorCombined, VehicleAttitude, VehicleOdometry
from fsc_autopilot_ros2_msgs.msg import Mocap
rclpy.init(); n = rclpy.create_node("rate_probe_x")
c = {}
def sub(t, T, q):
    c[t] = 0
    n.create_subscription(T, t, lambda m, t=t: c.__setitem__(t, c[t] + 1), q)
sub("/uav_0/fmu/out/sensor_combined", SensorCombined, qos_profile_sensor_data)
sub("/uav_0/fmu/out/vehicle_attitude", VehicleAttitude, qos_profile_sensor_data)
sub("/uav_0/fmu/out/vehicle_odometry", VehicleOdometry, qos_profile_sensor_data)
sub("/uav_0/mocap", Mocap, 10)
t0 = time.time()
while time.time() - t0 < 3: rclpy.spin_once(n, timeout_sec=0.05)
for k in c: c[k] = 0
t0 = time.time()
while time.time() - t0 < 10: rclpy.spin_once(n, timeout_sec=0.01)
for k, v in c.items(): print(f"{k}: {v / 10.0:.1f} Hz")
