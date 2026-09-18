#!/usr/bin/env python3
"""Loopback test of the DECOUPLED rig's planner wiring + reference bridge
(no Isaac, no PX4, no controller): the planner is launched exactly as the
decoupled stack script launches it (ros2 run, the whole-body 4-D sim yaml,
mode_topic and arm_reference_topic remapped to the geometric+L1 fork and the
position controller), the bridge beside it, and a fake rig supplies mode /
odometry / attitude / joint states.

Checks: the bridge is SILENT in SAFETY; in DIRECT it converts the planner's
hold to a position_controller/reference at the rig's own pose (~100 Hz,
yaw = actual yaw); the joint trajectory streams on the position controller's
topic; select circle -> READY; go_to_start executes and the converted base
reference moves CONTINUOUSLY (max inter-sample step) and ends on the
planner's own start_rest base pose; a SAFETY revert silences the bridge.

    PYTHONNOUSERSITE=1 /usr/bin/python3 test_decoupled_wiring_loopback.py
"""
import math
import os
import signal
import subprocess
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleAttitude
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory
from fsc_autopilot_ros2_msgs.msg import PositionControllerReference

NS = "/uav_dectest"
WS = os.path.expanduser(os.environ.get("FSC_AUTOPILOT_WS", "~/ros2_ws"))
YAML = f"{WS}/src/fsc_autopilot_ros2/config/params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_sim.yaml"
BRIDGE = f"{WS}/src/fsc_autopilot_ros2/scripts/tools/decoupled_reference_bridge.py"
PEG = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
HOME = [0.0, 0.698132, 0.698132, 0.0]
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
POS = np.array([0.3, 0.2, 1.2]); YAW = 0.35


class Rig(Node):
    def __init__(self):
        super().__init__("dectest_rig", namespace=NS)
        self.mode = self.create_publisher(String, "fsc_autopilot_ros2/geometric_l1_direct_actuation/mode", LATCHED)
        self.odom = self.create_publisher(Odometry, "state_estimator/local_position/odom", 10)
        self.att = self.create_publisher(VehicleAttitude, "fmu/out/vehicle_attitude",
                                         QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                                                    durability=DurabilityPolicy.VOLATILE))
        self.js = self.create_publisher(JointState, "fsc_open_manipulator/joint_states", 10)
        self.select = self.create_publisher(String, "whole_body_planner/ee_trajectory/select", 10)
        self.status = ""; self.ee_status = ""; self.refs = []; self.armrefs = []; self.start_rest = None
        self.create_subscription(String, "whole_body_planner/status", lambda m: setattr(self, "status", m.data), LATCHED)
        self.create_subscription(String, "whole_body_planner/ee_trajectory/status", lambda m: setattr(self, "ee_status", m.data), LATCHED)
        self.gs_pub = self.create_publisher(PositionControllerReference, "fsc_autopilot_ros2/position_controller/reference", 10)
        self.create_subscription(PositionControllerReference, "fsc_autopilot_ros2/position_controller/reference_direct",
                                 lambda m: self.refs.append((time.time(), m.position.x, m.position.y, m.position.z, m.yaw, m.yaw_unit,
                                                             m.velocity.x, m.velocity.y, m.velocity.z)), 10)
        self.create_subscription(JointTrajectory, "fsc_open_manipulator/position_controller/reference_joint_trajectory",
                                 lambda m: self.armrefs.append((time.time(), list(m.points[0].positions) if m.points else [])), 10)
        self.create_subscription(PoseStamped, "whole_body_planner/ee_trajectory/start_rest",
                                 lambda m: setattr(self, "start_rest", (m.pose.position.x, m.pose.position.y, m.pose.position.z)), LATCHED)
        self.go = self.create_client(Trigger, "whole_body_planner/ee_trajectory/go_to_start")
        self.q = list(HOME); self.pos = POS.copy()
        self.create_timer(0.01, self.feed)

    def feed(self):
        o = Odometry(); o.header.stamp = self.get_clock().now().to_msg()
        o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = map(float, self.pos)
        o.pose.pose.orientation.z = math.sin(YAW / 2); o.pose.pose.orientation.w = math.cos(YAW / 2)
        self.odom.publish(o)
        ned = 0.5 * math.pi - YAW          # ENU yaw = 90 deg - NED yaw (the planner's own test rig)
        a = VehicleAttitude(); a.q = [float(math.cos(0.5 * ned)), 0.0, 0.0, float(math.sin(0.5 * ned))]
        self.att.publish(a)
        j = JointState(); j.header.stamp = o.header.stamp
        j.name = ["joint1", "joint2", "joint3", "joint4"]; j.position = [float(x) for x in self.q]
        j.velocity = [0.0] * 4; j.effort = [0.0] * 4
        self.js.publish(j)


def main():
    rclpy.init()
    rig = Rig()
    ex = rclpy.executors.SingleThreadedExecutor(); ex.add_node(rig)
    th = threading.Thread(target=ex.spin, daemon=True); th.start()
    env = dict(os.environ)
    planner = subprocess.Popen(
        ["ros2", "run", "fsc_trajectory_planner", "whole_body_trajectory_planner", "--ros-args",
         "-r", f"__ns:={NS}", "--params-file", YAML,
         "-p", "mode_topic:=fsc_autopilot_ros2/geometric_l1_direct_actuation/mode",
         "-p", "arm_reference_topic:=fsc_open_manipulator/position_controller/reference_joint_trajectory"],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env, preexec_fn=os.setsid)
    bridge = subprocess.Popen(
        [sys.executable, BRIDGE, "--namespace", NS, "--da", "geometric_l1_direct_actuation", "--pegasus-root", PEG],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env, preexec_fn=os.setsid)
    fails = 0
    def check(name, ok, detail=""):
        nonlocal fails
        print(f"  [{'PASS' if ok else 'FAIL'}] {name} {detail}", flush=True)
        if not ok:
            fails += 1
    try:
        time.sleep(4.0)
        rig.mode.publish(String(data="SAFETY")); time.sleep(3.0)
        check("bridge converts nothing in SAFETY", len(rig.refs) == 0, f"({len(rig.refs)} refs)")
        gs = PositionControllerReference(); gs.position.x, gs.position.y, gs.position.z = 1.0, 2.0, 3.0; gs.yaw = 12.0
        for _ in range(5):
            rig.gs_pub.publish(gs); time.sleep(0.05)
        time.sleep(0.5)
        check("GS reference forwarded in SAFETY", len(rig.refs) >= 5 and abs(rig.refs[-1][3] - 3.0) < 1e-9,
              f"({len(rig.refs)} forwarded, last z {rig.refs[-1][3] if rig.refs else None})")
        rig.refs.clear()
        rig.mode.publish(String(data="DIRECT")); time.sleep(4.0)
        n = len(rig.refs)
        check("bridge streams in DIRECT", n > 150, f"({n} refs in ~4 s)")
        if n:
            r = np.array([x[1:5] for x in rig.refs[-50:]])
            check("converted hold = rig base pose", np.allclose(r[:, :3], POS, atol=0.005),
                  f"mean {r[:, :3].mean(axis=0).round(4)} vs {POS}")
            check("converted yaw = actual yaw", np.allclose(r[:, 3], YAW, atol=0.01), f"{r[:, 3].mean():.4f} vs {YAW}")
            check("yaw unit RADIANS", all(x[5] == 1 for x in rig.refs[-50:]))
        check("joint trajectory streams to position_controller", len(rig.armrefs) > 100,
              f"({len(rig.armrefs)} msgs) last {np.round(rig.armrefs[-1][1], 3) if rig.armrefs else None}")
        rig.select.publish(String(data="circle"))
        for _ in range(300):
            if rig.ee_status.startswith("READY") or rig.ee_status.startswith("INFEASIBLE"):
                break
            time.sleep(0.1)
        check("circle READY", rig.ee_status.startswith("READY"), rig.ee_status)
        rig.refs.clear()
        if rig.go.wait_for_service(timeout_sec=5.0):
            fut = rig.go.call_async(Trigger.Request())
            for _ in range(100):
                if fut.done():
                    break
                time.sleep(0.05)
            print("  go_to_start ->", fut.result().message if fut.done() else "no response")
        t_end = time.time() + 70.0
        seen_exec = False
        while time.time() < t_end:
            if rig.status.startswith("EXECUTING"):
                seen_exec = True
            if seen_exec and rig.status == "HOLD":
                break
            time.sleep(0.1)
        check("go_to_start executed", seen_exec, rig.status)
        # the rig teleports onto the start rest, as the planner's own test does
        if rig.start_rest is not None:
            rig.pos = np.array(rig.start_rest)
        r = np.array([x[1:4] for x in rig.refs])
        if r.shape[0] > 10:
            step = np.linalg.norm(np.diff(r, axis=0), axis=1).max()
            check("converted base reference continuous", step < 0.01, f"max step {step*1e3:.2f} mm over {r.shape[0]} samples")
            if rig.start_rest is not None:
                check("ends on the planner's start_rest base pose", np.allclose(r[-1], rig.start_rest, atol=0.01),
                      f"{r[-1].round(3)} vs {np.round(rig.start_rest, 3)}")
            v = np.array([x[6:9] for x in rig.refs])
            check("velocity feedforward non-trivial during the move", np.linalg.norm(v, axis=1).max() > 0.02,
                  f"peak |v_b| {np.linalg.norm(v, axis=1).max():.3f} m/s")
        rig.refs.clear(); rig.mode.publish(String(data="SAFETY")); time.sleep(2.0)
        n0 = len(rig.refs); time.sleep(2.0)
        check("bridge converts nothing after SAFETY revert", len(rig.refs) == n0, f"({len(rig.refs)-n0} refs in 2 s)")
    finally:
        for p in (planner, bridge):
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGINT)
            except Exception:
                pass
        time.sleep(1.5)
        for p in (planner, bridge):
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGKILL)
            except Exception:
                pass
        out = bridge.stdout.read() if bridge.stdout else ""
        print("--- bridge log tail:\n" + "\n".join(out.splitlines()[-6:]))
        ex.shutdown(); rig.destroy_node(); rclpy.try_shutdown()
    print(f"{'ALL PASS' if fails == 0 else f'{fails} FAILURES'}")
    sys.exit(1 if fails else 0)


if __name__ == "__main__":
    main()
