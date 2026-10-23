#!/usr/bin/env python3
"""Ask the whole-body trajectory planner for one EE shape on a FAKE hover rig
(no Isaac, no PX4) and print its own READY/INFEASIBLE verdict and diagnostics.

    PYTHONNOUSERSITE=1 /usr/bin/python3 ee_plan_probe.py <params.yaml> <circle|figure8>

Use it to check an ee_traj_* change (fold, q2 split, period, radius) before
spending a flight on it: the planner runs the same feasibility checks it would
in DIRECT -- joint box, fold, sigma_nd, speed/accel/yaw-rate, joint torque,
rotor force -- and reports the largest feasible time scale.
"""
import math
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleAttitude
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray, String

NS = "/uav_eeprobe"
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
HOME = [0.0, 0.698132, 0.698132, 0.0]
INFO_NAMES = ["s", "s_max", "T_total", "T_lap", "laps", "ramp", "type_id",
              "ee_pos_err_m", "ee_rot_err_deg", "peak_v", "peak_a", "peak_qdot",
              "peak_tau_j", "min_sigma_nd"]


class Rig(Node):
    def __init__(self):
        super().__init__("ee_probe_rig", namespace=NS)
        self.mode = self.create_publisher(
            String, "fsc_autopilot_ros2/whole_body_direct_actuation/mode", LATCHED)
        self.odom = self.create_publisher(Odometry, "state_estimator/local_position/odom", 10)
        self.att = self.create_publisher(
            VehicleAttitude, "fmu/out/vehicle_attitude",
            QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                       durability=DurabilityPolicy.VOLATILE))
        self.js = self.create_publisher(JointState, "fsc_open_manipulator/joint_states", 10)
        self.sel = self.create_publisher(String, "whole_body_planner/ee_trajectory/select", 10)
        self.scale = self.create_publisher(Float64, "whole_body_planner/ee_trajectory/time_scale", 10)
        self.status = None
        self.info = None
        self.create_subscription(String, "whole_body_planner/ee_trajectory/status",
                                 lambda m: setattr(self, "status", m.data), LATCHED)
        self.create_subscription(Float64MultiArray, "whole_body_planner/ee_trajectory/info",
                                 lambda m: setattr(self, "info", list(m.data)), LATCHED)
        self.create_timer(0.02, self.tick)

    def tick(self):
        o = Odometry()
        o.header.stamp = self.get_clock().now().to_msg()
        o.pose.pose.position.z = 1.2
        o.pose.pose.orientation.w = 1.0
        self.odom.publish(o)
        a = VehicleAttitude()
        ned = 0.5 * math.pi
        a.q = [math.cos(0.5 * ned), 0.0, 0.0, math.sin(0.5 * ned)]
        self.att.publish(a)
        j = JointState()
        j.header.stamp = self.get_clock().now().to_msg()
        j.name = [f"joint{i + 1}" for i in range(4)]
        j.position = list(HOME)
        j.velocity = [0.0] * 4
        self.js.publish(j)


def main():
    yaml, shape = sys.argv[1], sys.argv[2]
    rclpy.init()
    rig = Rig()
    threading.Thread(target=rclpy.spin, args=(rig,), daemon=True).start()
    gov = subprocess.Popen(
        ["ros2", "launch", "fsc_trajectory_planner",
         "whole_body_trajectory_planner_launch.py",
         f"uav_prefix:={NS.lstrip('/')}", f"params_file:={yaml}"],
        stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        time.sleep(6)
        rig.mode.publish(String(data="DIRECT"))
        time.sleep(3)
        rig.sel.publish(String(data=shape))
        t0 = time.time()
        while time.time() - t0 < 240:
            s = rig.status or ""
            if s.startswith("READY") or "INFEASIBLE" in s:
                break
            time.sleep(0.2)
        print(f"[{shape}] {rig.status}")
        if (rig.status or "").startswith("READY") and rig.info:
            for n, v in zip(INFO_NAMES, rig.info):
                print(f"    {n:<16s} {v:.4f}")
            # The slider sets the worst case, so re-plan at s_max and report the
            # rates there -- peak |q_dot| and the yaw rate both scale with s.
            s_max = rig.info[1]
            rig.info = None
            rig.scale.publish(Float64(data=s_max))
            t0 = time.time()
            while time.time() - t0 < 240 and rig.info is None:
                time.sleep(0.2)
            if rig.info:
                d = dict(zip(INFO_NAMES, rig.info))
                print(f"    -- at s_max = {d['s']:.4f}: peak |q_dot| "
                      f"{math.degrees(d['peak_qdot']):.2f} deg/s, |v| {d['peak_v']:.3f} m/s, "
                      f"|a| {d['peak_a']:.3f} m/s^2, tau_j {d['peak_tau_j']:.3f} N.m, "
                      f"sigma_nd {d['min_sigma_nd']:.3f}, T {d['T_total']:.1f} s")
    finally:
        import os
        import signal
        os.killpg(os.getpgid(gov.pid), signal.SIGINT)
        time.sleep(2)
    rclpy.shutdown()


main()
