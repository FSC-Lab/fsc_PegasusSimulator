#!/usr/bin/env python3
"""The Pick-and-Drop workflow page's arm-GS snapshots (2026-10-19): fly the REAL planner through the
operator's workflow on a perfect-plant loopback rig, PRESSING THE REAL Pick & Drop tab's buttons
(gs_harness/gs_pnd_snapshot), and photograph the tab at each step.

    pnd_rig.py <params yaml> <out dir>      (run through snapshots.sh: private ROS domain 77)

The rig is wb_pick_drop_flight_20261015/tools/carry_plan_check.py's (odometry, PX4 attitude, joint
states and /obj_0/mocap fed at 50 Hz; after each leg the vehicle is teleported onto the leg's goal),
extended with the operator's ground steps -- the basket on the container's cover for the Drop row's
Get, on the pick hat for the Pick row's Get (then the doc's hand edit: x + 0.01, yaw 0), the vehicle
on its start mark for Adjust -- and with the basket HANGING from the claw after the grasp, the way it
did on 10-15 D-1, so the drop aim and the automatic drop run as in flight. The tab's own automation
(Exit To Pick after the grasp, the release on drop_now, Exit To Drop on the OPEN report) is the
station's code; only the gripper is emulated (by the harness).
"""
import json
import math
import sys
import time

import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleAttitude
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from fsc_autopilot_ros2_msgs.msg import Mocap, WholeBodyReference

NS = "/uav_pndshot"
SIGN = np.array([-1.0, 1.0, 1.0, -1.0])            # arm_joint_sign (hardware -> model)
HOME = np.radians([0.0, 40.0, 40.0, 0.0])
GROUND_Z = 0.235                                   # resting body height, T650 gear
START_MARK = np.array([0.11, -0.07])               # where the vehicle stands: Adjust measures (+0.11, -0.07)
PICK_OBJ = np.array([0.993, 1.004, 0.897])         # basket on the pick hat (Get), 0.77 m pillar + hat
COVER_OBJ = np.array([-1.004, -0.997, 0.846])      # basket standing on the container's cover (Get)
HANG = np.array([0.003, -0.017, -0.160])           # basket - claw, vehicle frame, 10-15 D-1
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def rz(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


class Rig(Node):
    def __init__(self):
        super().__init__("pnd_snapshot_rig", namespace=NS)
        self.mode = self.create_publisher(String, "fsc_autopilot_ros2/whole_body_direct_actuation/mode", LATCHED)
        self.odom = self.create_publisher(Odometry, "state_estimator/local_position/odom", 10)
        self.att = self.create_publisher(VehicleAttitude, "fmu/out/vehicle_attitude",
                                         QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                                                    durability=DurabilityPolicy.VOLATILE))
        self.js = self.create_publisher(JointState, "fsc_open_manipulator/joint_states", 10)
        self.obj = self.create_publisher(Mocap, "/obj_0/mocap", 10)
        self.cmd = self.create_publisher(String, "/pndshot/cmd", 10)
        self.acks = []
        self.create_subscription(String, "/pndshot/ack", lambda m: self.acks.append(m.data), 10)
        self.pp_status, self.info, self.refs, self.arr = None, None, [], None
        self.create_subscription(Float64MultiArray, "whole_body_planner/pick_place/arrival_error",
                                 lambda m: setattr(self, "arr", np.asarray(m.data)), 10)
        self.create_subscription(String, "whole_body_planner/pick_place/status",
                                 lambda m: setattr(self, "pp_status", m.data), LATCHED)
        self.create_subscription(Float64MultiArray, "whole_body_planner/pick_place/info",
                                 lambda m: setattr(self, "info", np.asarray(m.data)), LATCHED)
        self.create_subscription(WholeBodyReference, "fsc_autopilot_ros2/whole_body_direct_actuation/reference",
                                 lambda m: self.refs.append(np.array(m.q_d[:4])), 50)
        self.drops = []
        self.create_subscription(String, "whole_body_planner/pick_place/drop_now",
                                 lambda m: self.drops.append(m.data), 10)
        self.params = self.create_client(SetParameters, NS + "/whole_body_trajectory_planner/set_parameters")
        self.xyz, self.yaw, self.q = np.array([*START_MARK, GROUND_Z]), 0.0, HOME.copy()
        self.obj_mode, self.obj_xyz, self.claw_body = "fixed", COVER_OBJ.copy(), None
        self.rng = np.random.default_rng(0)
        self.create_timer(0.02, self.feed)

    def claw(self):
        return self.xyz + rz(self.yaw) @ self.claw_body

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
        p = self.claw() + rz(self.yaw) @ HANG if self.obj_mode == "hang" else self.obj_xyz
        p = p + self.rng.normal(0.0, 0.0005, 3)
        m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, p)
        m.pose.orientation.w = 1.0
        self.obj.publish(m)

    def teleport(self, leg):
        g = self.info[14 + 7 * leg: 21 + 7 * leg]
        self.xyz = np.array(g[0:3])
        self.yaw = math.radians(g[3])
        self.q = np.array(self.refs[-1])
        spin_for(self, 0.6)


def spin_for(rig, s):
    t = time.monotonic() + s
    while time.monotonic() < t:
        rclpy.spin_once(rig, timeout_sec=0.01)


def gs(rig, cmd, must=True):
    """Send a command to the GS harness and wait for its ack."""
    n = len(rig.acks)
    m = String(); m.data = cmd
    rig.cmd.publish(m)
    t = time.monotonic() + 10.0
    while time.monotonic() < t:
        rclpy.spin_once(rig, timeout_sec=0.01)
        for a in rig.acks[n:]:
            if a.endswith(cmd) or (" " + cmd + ":") in a:
                print(f"  GS {a}")
                if must and not a.startswith("ok"):
                    sys.exit(f"GS refused: {a}")
                return a.startswith("ok")
    sys.exit(f"no ack for {cmd}")


def wait_status(rig, prefixes, timeout):
    t = time.monotonic() + timeout
    while time.monotonic() < t:
        rclpy.spin_once(rig, timeout_sec=0.02)
        s = rig.pp_status or ""
        if any(s.startswith(p) for p in prefixes):
            return s
    sys.exit(f"timeout waiting for {prefixes}; status {rig.pp_status!r}")


def wait_until(rig, pred, timeout, what):
    t = time.monotonic() + timeout
    while time.monotonic() < t:
        rclpy.spin_once(rig, timeout_sec=0.02)
        if pred():
            return
    sys.exit(f"timeout waiting for {what}; status {rig.pp_status!r}")


def set_param(rig, name, values):
    rig.params.wait_for_service(timeout_sec=5.0)
    req = SetParameters.Request()
    req.parameters.append(Parameter(name=name, value=ParameterValue(
        type=ParameterType.PARAMETER_DOUBLE_ARRAY, double_array_value=[float(v) for v in values])))
    f = rig.params.call_async(req)
    rclpy.spin_until_future_complete(rig, f, timeout_sec=5.0)
    ok = f.result() is not None and all(r.successful for r in f.result().results)
    print(f"  set {name} = {np.round(values, 3).tolist()}: {'OK' if ok else 'REFUSED'}")
    if not ok:
        sys.exit(1)


def main():
    p = yaml.safe_load(open(sys.argv[1]))["/**/whole_body_trajectory_planner"]["ros__parameters"]
    assert p.get("pick_place_place_drop"), "the yaml must fly the drop place"
    rclpy.init()
    rig = Rig()
    m = String(); m.data = "SAFETY"
    rig.mode.publish(m)
    spin_for(rig, 4.0)                          # the station loads the planner's values

    # --- on the ground: the basket on the container's COVER -> the Drop row's Get
    rig.obj_xyz = COVER_OBJ.copy()
    spin_for(rig, 1.0)
    gs(rig, "click:Get#1")
    spin_for(rig, 2.0)
    gs(rig, "snap:01_drop_get")
    # --- the basket on the pick hat -> the Pick row's Get, then the hand edit x + 0.01, yaw 0
    rig.obj_xyz = PICK_OBJ.copy()
    spin_for(rig, 1.0)
    gs(rig, "click:Get#0")
    spin_for(rig, 2.0)
    pp = rig.info
    set_param(rig, "pick_place_pick_point", [round(PICK_OBJ[0] + 0.01, 3), PICK_OBJ[1], PICK_OBJ[2], 0.0])
    spin_for(rig, 2.0)
    gs(rig, "snap:02_pick_get")
    # --- the vehicle on its start mark -> Adjust
    gs(rig, "click:Adjust")
    spin_for(rig, 2.0)
    gs(rig, "snap:03_adjust")

    # --- take off (SAFETY), switch to DIRECT
    rig.xyz = np.array([*START_MARK, 1.0])
    spin_for(rig, 1.0)
    m.data = "DIRECT"
    rig.mode.publish(m)
    wait_status(rig, ["NOT PLANNED"], 15)
    spin_for(rig, 1.5)
    gs(rig, "snap:04_direct")
    gs(rig, "click:Plan")
    wait_status(rig, ["READY"], 30)
    spin_for(rig, 1.5)
    gs(rig, "snap:05_ready")
    xb3, yaw3, ee3 = rig.info[14 + 21:17 + 21], math.radians(rig.info[17 + 21]), rig.info[18 + 21:21 + 21]
    rig.claw_body = rz(yaw3).T @ (np.asarray(ee3) - np.asarray(xb3))

    # --- 1 Go To Start
    gs(rig, "click:1 Go To Start")
    wait_status(rig, ["DONE go_to_start"], 60)
    rig.teleport(0)
    # --- 2.1 Ready To Pick (the gripper opens first), the approach rest behind the stem
    gs(rig, "click:2.1 Ready To Pick")
    wait_status(rig, ["WAITING execute_pick"], 60)
    rig.teleport(1)
    spin_for(rig, 0.6)
    rig.xyz = rig.xyz + rig.arr[3:6]            # onto the approach rest: by the planner's own claw error
    spin_for(rig, 6.5)                          # > the 5 s descent-trim window
    gs(rig, "snap:06_ready_to_pick")
    # --- 2.2 Pick: slide in; the station closes the gripper on the target; Exit To Pick by itself
    gs(rig, "click:2.2 Pick")
    wait_status(rig, ["DONE execute_pick"], 40)
    rig.teleport(1)                             # claw on the target: the station closes
    wait_until(rig, lambda: (rig.pp_status or "").startswith("FLYING exit_pick"), 15, "Exit To Pick")
    rig.obj_mode = "hang"
    spin_for(rig, 0.4)
    gs(rig, "snap:07_exit_pick")
    wait_status(rig, ["DONE exit_pick"], 30)
    rig.xyz = rig.xyz + np.array([0.0, 0.0, p.get("pick_place_approach_dz", 0.20)])
    spin_for(rig, 0.6)
    # --- 3 Go To Drop Start, then HOVER >= 6 s (the basket aim's settle + window)
    gs(rig, "click:3 Go To Drop Start")
    wait_status(rig, ["DONE go_to_place_start"], 60)
    rig.teleport(2)
    spin_for(rig, p.get("pick_place_drop_aim_settle_s", 2.0) + p.get("pick_place_drop_aim_window_s", 4.0) + 1.0)
    gs(rig, "snap:08_drop_start")
    # --- 4.1 Ready To Drop: aimed at the basket, holds on the drop point
    gs(rig, "click:4.1 Ready To Drop")
    spin_for(rig, 0.8)
    gs(rig, "snap:09_ready_to_drop")
    wait_status(rig, ["DONE execute_place"], 60)
    rig.teleport(3)
    spin_for(rig, 0.9)                          # inside the 2 s wait: the auto-drop gate text
    gs(rig, "snap:10_drop_hold")
    # --- 4.2 Drop, automatic: drop_now -> the station opens -> OPEN -> 4.3 Exit To Drop by itself
    wait_until(rig, lambda: len(rig.drops) > 0, 20, "the planner's automatic drop")
    rig.obj_mode, rig.obj_xyz = "fixed", np.array([COVER_OBJ[0], COVER_OBJ[1], COVER_OBJ[2] - 0.03])
    spin_for(rig, 0.3)
    gs(rig, "snap:11_drop_open")
    wait_until(rig, lambda: (rig.pp_status or "").startswith("FLYING exit_place"), 15, "Exit To Drop")
    spin_for(rig, 0.4)
    gs(rig, "snap:12_exit_drop")
    wait_status(rig, ["DONE exit_place"], 30)
    nose = np.array([math.cos(rig.yaw), math.sin(rig.yaw), 0.0])
    back = p.get("pick_place_drop_exit_back", 0.20)
    rig.xyz = rig.xyz - back * nose + np.array([0.0, 0.0, p.get("pick_place_approach_dz", 0.20)])
    spin_for(rig, 0.6)
    # --- 5 Go To Land Start (the station closes the gripper), 6 Execute To Land
    gs(rig, "click:5 Go To Land Start")
    wait_status(rig, ["DONE go_to_land_start"], 60)
    rig.teleport(4)
    gs(rig, "click:6 Execute To Land")
    wait_status(rig, ["COMPLETE"], 40)
    spin_for(rig, 1.0)
    gs(rig, "snap:13_complete")
    print("  drop_now:", rig.drops[:1])
    gs(rig, "quit", must=False)
    rig.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
