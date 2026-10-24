#!/usr/bin/env python3
"""The Mail Envelope tab, end to end WITHOUT Isaac (2026-10-23): the REAL planner
(fsc_trajectory_planner) on the mail-delivery yaml and the REAL arm-GS tab
(gs_harness/gs_mail_snapshot, offscreen), flown on a perfect-plant loopback rig
that PRESSES THE TAB'S OWN BUTTONS through the operator's workflow and
photographs the tab at each step.

    mail_rig.py <params yaml> <out dir>      (run through mail_gs_check.sh: ROS domain 77)

A copy of pick_and_drop_workflow_20261019/tools/pnd_rig.py, for the INSERT place:
odometry, PX4 attitude, joint states and /obj_0/mocap fed at 50 Hz; after each leg
the vehicle is teleported onto the leg's goal. The envelope lies on the pick hat
until the grasp, then rides 60 mm ahead of the claw (its centre), and after the
insert's OPEN it is left in the mailbox. Checks, beyond "the mission completes":
the tab is ACTIVE only after its own Plan (Pick & Place / Pick & Drop modes off);
the plan's arm pose has q4 = 90 deg from the pick to the place; Ready To Insert
holds the claw the Insert Margin BEHIND the place target at its height; the
station closes the gripper only on the target, opens it only on the place target,
starts Exit To Insert only after the OPEN, and closes the gripper after the exit.
"""
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

NS = "/uav_mailshot"
SIGN = np.ones(4)                                  # the SIM yaml's arm_joint_sign
HOME = np.radians([0.0, 40.0, 40.0, 0.0])
GROUND_Z = 0.235                                   # resting body height, T650 gear
START_MARK = np.array([0.10, -0.07])               # the scene's spawn: Adjust measures (+0.10, -0.07)
ENVELOPE = np.array([0.9047, 1.0, 0.780])          # its centre on the pick hat (07 mail-delivery scene)
AHEAD = np.array([0.06, 0.0, 0.0])                 # the envelope's centre - the claw, vehicle frame, once held
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
FAILS = []


def check(ok, what):
    print(f"  [{'PASS' if ok else 'FAIL'}] {what}")
    if not ok:
        FAILS.append(what)


def rz(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


class Rig(Node):
    def __init__(self):
        super().__init__("mail_check_rig", namespace=NS)
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
        self.params = self.create_client(SetParameters, NS + "/whole_body_trajectory_planner/set_parameters")
        self.xyz, self.yaw, self.q = np.array([*START_MARK, GROUND_Z]), 0.0, HOME.copy()
        self.obj_mode, self.obj_xyz, self.claw_body = "fixed", ENVELOPE.copy(), None
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
        p = self.claw() + rz(self.yaw) @ AHEAD if self.obj_mode == "held" else self.obj_xyz
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
            return True
    sys.exit(f"timeout waiting for {what}; status {rig.pp_status!r}")


def set_param(rig, name, value):
    rig.params.wait_for_service(timeout_sec=5.0)
    req = SetParameters.Request()
    if isinstance(value, bool):
        pv = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=value)
    else:
        pv = ParameterValue(type=ParameterType.PARAMETER_DOUBLE_ARRAY, double_array_value=[float(v) for v in value])
    req.parameters.append(Parameter(name=name, value=pv))
    f = rig.params.call_async(req)
    rclpy.spin_until_future_complete(rig, f, timeout_sec=5.0)
    ok = f.result() is not None and all(r.successful for r in f.result().results)
    print(f"  set {name} = {value}: {'OK' if ok else 'REFUSED'}")
    if not ok:
        sys.exit(1)


def main():
    p = yaml.safe_load(open(sys.argv[1]))["/**/whole_body_trajectory_planner"]["ros__parameters"]
    assert p.get("pick_place_place_insert") and not p.get("pick_place_place_drop"), "not the mail yaml"
    back = float(p["pick_place_place_approach_back"])
    rclpy.init()
    rig = Rig()
    m = String(); m.data = "SAFETY"
    rig.mode.publish(m)
    spin_for(rig, 4.0)                          # the station loads the planner's values

    # the tab must NOT be active while the planner flies another place: switch to the
    # set-down, see the tab say so, and let ITS Plan switch it back (later)
    set_param(rig, "pick_place_place_insert", False)
    spin_for(rig, 1.5)
    gs(rig, "snap:00_other_mode")

    # --- on the ground: the envelope on the pick hat -> the Envelope row's Get
    gs(rig, "click:Get#0")
    spin_for(rig, 2.0)
    gs(rig, "snap:01_envelope_get")
    gs(rig, "click:Adjust")
    spin_for(rig, 2.0)
    gs(rig, "snap:02_adjust")

    # --- take off (SAFETY), switch to DIRECT, Plan IN THE MAIL TAB
    rig.xyz = np.array([*START_MARK, 1.0])
    spin_for(rig, 1.0)
    m.data = "DIRECT"
    rig.mode.publish(m)
    wait_status(rig, ["NOT PLANNED"], 15)
    spin_for(rig, 1.5)
    gs(rig, "click:Plan")
    wait_status(rig, ["READY"], 40)
    spin_for(rig, 1.5)
    gs(rig, "snap:03_ready")
    info = rig.info
    print("  leg durations", np.round(info[8:14], 1).tolist(), "s; exits", np.round(info[81:83], 1).tolist(), "s")
    xb3, yaw3, ee3 = info[14 + 21:17 + 21], math.radians(info[17 + 21]), info[18 + 21:21 + 21]
    rig.claw_body = rz(yaw3).T @ (np.asarray(ee3) - np.asarray(xb3))
    check(abs(rig.claw_body[0] - 0.247) < 0.005 and abs(rig.claw_body[2] + 0.0675) < 0.005,
          f"claw in the body frame at the place {np.round(rig.claw_body, 4).tolist()} (0.247 ahead, 0.068 below)")

    # --- 1 Go To Start
    gs(rig, "click:1 Go To Start")
    wait_status(rig, ["DONE go_to_start"], 60)
    rig.teleport(0)
    # --- 2.1 Ready To Pick: the gripper opens first; the hold BEHIND the envelope's edge
    gs(rig, "click:2.1 Ready To Pick")
    wait_status(rig, ["WAITING execute_pick"], 60)
    rig.teleport(1)
    spin_for(rig, 0.6)
    rig.xyz = rig.xyz + rig.arr[3:6]            # onto the approach rest (the planner's own claw error)
    spin_for(rig, 1.5)
    check(abs(np.degrees(rig.q[3]) - 90.0) < 0.5, f"Ready To Pick: wrist roll q4 = {np.degrees(rig.q[3]):.1f} deg")
    gs(rig, "snap:04_ready_to_pick")
    # --- 2.2 Pick: slide in; the station closes ON the target; Exit To Pick by itself
    gs(rig, "click:2.2 Pick")
    wait_status(rig, ["DONE execute_pick"], 40)
    rig.teleport(1)
    wait_until(rig, lambda: (rig.pp_status or "").startswith("FLYING exit_pick"), 15, "Exit To Pick")
    rig.obj_mode = "held"
    check(True, "Pick: closed on the target, Exit To Pick started by itself")
    spin_for(rig, 0.4)
    gs(rig, "snap:05_exit_pick")
    wait_status(rig, ["DONE exit_pick"], 30)
    rig.xyz = rig.xyz + np.array([0.0, 0.0, p.get("pick_place_approach_dz", 0.20)])
    spin_for(rig, 0.6)
    # --- 3 Go To Mailbox Start
    gs(rig, "click:3 Go To Mailbox Start")
    wait_status(rig, ["DONE go_to_place_start"], 60)
    rig.teleport(2)
    check(abs(np.degrees(rig.q[3]) - 90.0) < 0.5, f"carried with q4 = {np.degrees(rig.q[3]):.1f} deg")
    # --- 4.1 Ready To Insert: the hold the Insert Margin BEHIND the target, level
    gs(rig, "click:4.1 Ready To Insert")
    wait_status(rig, ["WAITING execute_place"], 60)
    rig.teleport(3)
    spin_for(rig, 0.6)
    e = np.asarray(rig.arr[3:6])                 # (approach claw) - (goal claw): the rig sits on the goal
    nose = np.array([math.cos(rig.yaw), math.sin(rig.yaw), 0.0])
    check(abs(float(np.dot(e, nose)) + back) < 0.002 and np.linalg.norm(e - np.dot(e, nose) * nose) < 0.002,
          f"Ready To Insert holds {-float(np.dot(e, nose)) * 1e3:.0f} mm behind the target, level "
          f"(off-axis {np.linalg.norm(e - np.dot(e, nose) * nose) * 1e3:.1f} mm; Insert Margin {back * 1e3:.0f})")
    rig.xyz = rig.xyz + e                        # onto the approach rest
    spin_for(rig, 1.5)
    gs(rig, "snap:06_ready_to_insert")
    # --- 4.2 Insert: slide in; the station OPENS on the target; Exit To Insert on the OPEN report
    gs(rig, "click:4.2 Insert")
    wait_status(rig, ["DONE execute_place"], 40)
    rig.teleport(3)
    wait_until(rig, lambda: (rig.pp_status or "").startswith("FLYING exit_place"), 15, "Exit To Insert")
    check(True, "Insert: opened on the target, Exit To Insert started by itself after the OPEN")
    rig.obj_mode, rig.obj_xyz = "fixed", rig.claw() + rz(rig.yaw) @ (AHEAD + np.array([0.0, 0.0, -0.15]))
    spin_for(rig, 0.4)
    gs(rig, "snap:07_exit_insert")
    wait_status(rig, ["DONE exit_place"], 30)
    rig.xyz = rig.xyz - p.get("pick_place_place_exit_back", 0.10) * nose + np.array([0.0, 0.0, p.get("pick_place_approach_dz", 0.20)])
    spin_for(rig, 1.0)
    # --- 5 Go To Land Start (the gripper closed after the exit), 6 Execute To Land
    gs(rig, "click:5 Go To Land Start")
    wait_status(rig, ["DONE go_to_land_start"], 60)
    rig.teleport(4)
    check(abs(np.degrees(rig.q[3])) < 0.5, f"Land Start: the arm folded home, q4 = {np.degrees(rig.q[3]):.1f} deg")
    gs(rig, "click:6 Execute To Land")
    wait_status(rig, ["COMPLETE"], 40)
    spin_for(rig, 1.0)
    gs(rig, "snap:08_complete")
    gs(rig, "quit", must=False)
    rig.destroy_node()
    rclpy.shutdown()
    print("ALL PASS" if not FAILS else f"FAILED: {FAILS}")
    return 1 if FAILS else 0


if __name__ == "__main__":
    raise SystemExit(main())
