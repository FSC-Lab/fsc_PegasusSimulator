#!/usr/bin/env python3
"""Fly the pick-and-drop mission's six legs against the REAL whole_body_trajectory_planner on a
perfect-plant loopback rig, with a given planner yaml, and report the arm reference it streams.

    carry_plan_check.py <params yaml>      (run through carry_plan_check.sh: private ROS domain 77)

The rig is test_pick_place_loopback.py's (fsc_trajectory_planner/test), reduced to one pass: DIRECT
hold at the start point, the basket (/obj_0/mocap) on the pick pillar, Get, Plan, then every leg; after
each leg the rig teleports odometry, attitude and joints onto the leg's goal. Written 2026-10-16 to check
the carry pose = pick pose change (the arm held fixed from the pick to the drop).
"""
import math
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleAttitude
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger
from fsc_autopilot_ros2_msgs.msg import Mocap, WholeBodyReference

NS = "/uav_carrycheck"
SIGN = np.array([-1.0, 1.0, 1.0, -1.0])            # arm_joint_sign (hardware -> model)
HOME = np.radians([0.0, 40.0, 40.0, 0.0])
OBJ = np.array([0.874, 0.826, 0.893])              # the basket on the pick hat, 10-15 flights
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
LEGS = ["go_to_start", "execute_pick", "go_to_place_start", "execute_place", "go_to_land_start", "execute_land"]


class Rig(Node):
    def __init__(self):
        super().__init__("carry_check_rig", namespace=NS)
        self.mode = self.create_publisher(String, "fsc_autopilot_ros2/whole_body_direct_actuation/mode", LATCHED)
        self.odom = self.create_publisher(Odometry, "state_estimator/local_position/odom", 10)
        self.att = self.create_publisher(VehicleAttitude, "fmu/out/vehicle_attitude",
                                         QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                                                    durability=DurabilityPolicy.VOLATILE))
        self.js = self.create_publisher(JointState, "fsc_open_manipulator/joint_states", 10)
        self.obj = self.create_publisher(Mocap, "/obj_0/mocap", 10)
        self.pp_status, self.info, self.refs, self.arr = None, None, [], None
        self.create_subscription(Float64MultiArray, "whole_body_planner/pick_place/arrival_error",
                                 lambda m: setattr(self, "arr", np.asarray(m.data)), 10)
        self.create_subscription(String, "whole_body_planner/pick_place/status",
                                 lambda m: setattr(self, "pp_status", m.data), LATCHED)
        self.create_subscription(Float64MultiArray, "whole_body_planner/pick_place/info",
                                 lambda m: setattr(self, "info", np.asarray(m.data)), LATCHED)
        self.create_subscription(WholeBodyReference, "fsc_autopilot_ros2/whole_body_direct_actuation/reference",
                                 lambda m: self.refs.append((time.monotonic(), np.degrees(np.array(m.q_d[:4])))), 50)
        self.cli = {n: self.create_client(Trigger, f"whole_body_planner/pick_place/{n}")
                    for n in LEGS + ["plan", "capture_pick", "descend_pick", "exit_pick", "exit_place"]}
        self.xyz, self.yaw, self.q = np.array([0.0, 0.0, 1.0]), 0.0, HOME.copy()
        self.rng = np.random.default_rng(0)
        self.create_timer(0.02, self.feed)

    def feed(self):
        o = Odometry()
        o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = map(float, self.xyz)
        o.pose.pose.orientation.w = 1.0
        self.odom.publish(o)
        a = VehicleAttitude()
        ned = 0.5 * math.pi - self.yaw
        a.q = [math.cos(0.5 * ned), 0.0, 0.0, math.sin(0.5 * ned)]
        self.att.publish(a)
        j = JointState()
        j.name = ["joint1", "joint2", "joint3", "joint4"]
        j.position = list(map(float, SIGN * self.q))
        j.velocity = [0.0] * 4
        self.js.publish(j)
        m = Mocap()
        p = OBJ + self.rng.normal(0.0, 0.0005, 3)
        m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, p)
        m.pose.orientation.w = 1.0
        self.obj.publish(m)

    def teleport(self, leg, dz=0.0):
        g = self.info[14 + 7 * leg: 21 + 7 * leg]
        self.xyz = np.array(g[0:3]) + np.array([0.0, 0.0, dz])
        self.yaw = math.radians(g[3])
        self.q = np.radians(self.refs[-1][1])
        spin_for(self, 0.6)          # let the 50 Hz feed publish the new pose


def spin_for(rig, s):
    t = time.monotonic() + s
    while time.monotonic() < t:
        rclpy.spin_once(rig, timeout_sec=0.01)


def call(rig, name):
    c = rig.cli[name]
    if not c.wait_for_service(timeout_sec=10.0):
        sys.exit(f"service {name} not available")
    f = c.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(rig, f, timeout_sec=10.0)
    r = f.result()
    print(f"  {name}: {'OK' if r.success else 'REFUSED'} -- {r.message[:150]}")
    return r.success


def wait_status(rig, prefixes, timeout):
    t = time.monotonic() + timeout
    while time.monotonic() < t:
        rclpy.spin_once(rig, timeout_sec=0.02)
        s = rig.pp_status or ""
        if any(s.startswith(p) for p in prefixes):
            return s
    sys.exit(f"timeout waiting for {prefixes}; status {rig.pp_status!r}")


def main():
    rclpy.init()
    rig = Rig()
    m = String(); m.data = "DIRECT"
    rig.mode.publish(m)
    spin_for(rig, 3.0)
    marks = {}
    call(rig, "capture_pick")
    call(rig, "plan")
    wait_status(rig, ["READY"], 20)
    spin_for(rig, 0.5)
    print("  planned leg T [s]:", np.round(rig.info[8:14], 2))
    for k, leg in enumerate(LEGS):
        marks[leg] = [time.monotonic(), None]
        if not call(rig, leg):
            sys.exit(1)
        if leg == "execute_pick":
            wait_status(rig, ["WAITING execute_pick"], 60)
            rig.teleport(1)                  # final yaw + arm pose; base on the leg's final goal
            spin_for(rig, 0.6)
            rig.xyz = rig.xyz + rig.arr[3:6]  # then onto the approach rest: move by the planner's own claw error
            spin_for(rig, 6.5)              # > the hardware yaml's 5 s descent-trim window
            print(f"  at the approach rest: claw error {rig.arr[2] * 1e3:.1f} mm")
            call(rig, "descend_pick")
            wait_status(rig, ["DONE execute_pick"], 30)
            rig.teleport(1)
            marks["exit_pick"] = [time.monotonic(), None]
            call(rig, "exit_pick")
            wait_status(rig, ["DONE exit_pick"], 30)
            marks["exit_pick"][1] = time.monotonic()
            rig.xyz = rig.xyz + np.array([0.0, 0.0, 0.20])     # exit_pick: 20 cm straight up
            spin_for(rig, 0.6)
        elif leg == "execute_place":
            wait_status(rig, ["DONE execute_place"], 60)
            rig.teleport(3)
            marks["execute_place"][1] = time.monotonic()
            marks["exit_place"] = [time.monotonic(), None]
            call(rig, "exit_place")
            wait_status(rig, ["DONE exit_place"], 30)
            marks["exit_place"][1] = time.monotonic()
            nose = np.array([math.cos(rig.yaw), math.sin(rig.yaw), 0.0])
            rig.xyz = rig.xyz - 0.20 * nose + np.array([0.0, 0.0, 0.20])   # backed out 20 cm, up 20 cm
            spin_for(rig, 0.6)
            continue
        elif leg == "execute_land":
            wait_status(rig, ["COMPLETE"], 40)
        else:
            wait_status(rig, [f"DONE {leg}"], 60)
            rig.teleport(k)
        if marks[leg][1] is None:
            marks[leg][1] = time.monotonic()
    pick = np.degrees(np.radians([0.0, 32.0, 38.0, 0.0]))
    print("\n  arm reference q_d per leg (deg): start .. end, max |q_d - pick pose|")
    T = np.array([r[0] for r in rig.refs]); Q = np.array([r[1] for r in rig.refs])
    for leg in ["go_to_start", "execute_pick", "exit_pick", "go_to_place_start", "execute_place", "exit_place",
                "go_to_land_start", "execute_land"]:
        a, b = marks[leg]
        sel = (T >= a) & (T <= b)
        if not sel.any():
            continue
        q = Q[sel]
        print(f"  {leg:18s} {np.round(q[0], 1)} .. {np.round(q[-1], 1)}   max dev from pick pose "
              f"{np.abs(q - pick).max():6.2f}")
    rig.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
