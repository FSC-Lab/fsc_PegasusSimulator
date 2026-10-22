#!/usr/bin/env python3
"""Report item 8 check (2026-10-21): replay the 10-21 hardware mocap (/uav_0/mocap, the processor's
output with its finite-difference twist) and EKF2 (fmu/out/vehicle_odometry) around the drone flip
and two VRPN pauses through the BUILT raw feedback node (mocap_guard:=true), and compare the speed
it publishes with the processor's. Private ROS domain, namespace /vchk.

    source /opt/ros/humble/setup.bash; source ~/ros2_ws/install/setup.bash
    ROS_DOMAIN_ID=79 /usr/bin/python3 replay_check.py [holdoff_s]
"""
import os
import signal
import subprocess
import sys
import threading
import time

import numpy as np
import rclpy
from fsc_autopilot_ros2_msgs.msg import Mocap
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "wb_box_pushing_flight_20261021", "tools"))
from bp_common import Bag  # noqa: E402

EVENTS = [("164338", 75.9, 76.9, "drone flip"), ("164338", 79.2, 80.1, "VRPN pause"),
          ("170123", 76.3, 77.2, "VRPN pause (box flight)")]


class Rig(Node):
    def __init__(self):
        super().__init__("vchk_rig", namespace="/vchk")
        self.mp = self.create_publisher(Mocap, "mocap", qos_profile_sensor_data)
        self.ep = self.create_publisher(VehicleOdometry, "fmu/out/vehicle_odometry", qos_profile_sensor_data)
        self.out = []
        self.create_subscription(Odometry, "state_estimator/local_position/odom",
                                 lambda m: self.out.append((time.monotonic(), m)), 50)


def play(rig, tag, t0, t1):
    b = Bag(tag)
    tm = b.t("mocap")
    sel = np.where((tm >= t0) & (tm <= t1))[0]
    te = b.t("vodom")
    sele = np.where((te >= t0 - 0.2) & (te <= t1))[0]
    ev = [(te[i], "e", i) for i in sele] + [(tm[i], "m", i) for i in sel]
    ev.sort(key=lambda x: x[0])
    P = {c: b.f("mocap", f"pose.position.{c}") for c in "xyz"}
    Q = {c: b.f("mocap", f"pose.orientation.{c}") for c in "xyzw"}
    V = {c: b.f("mocap", f"twist.linear.{c}") for c in "xyz"}
    W = {c: b.f("mocap", f"twist.angular.{c}") for c in "xyz"}
    EP, EQ, EV = (np.vstack(b.f("vodom", k)) for k in ("position", "q", "velocity"))
    EW = np.vstack(b.f("vodom", "angular_velocity"))
    pf, vf = b.f("vodom", "pose_frame"), b.f("vodom", "velocity_frame")
    rig.out.clear()
    proc = []
    start = time.monotonic() - ev[0][0]
    for t, k, i in ev:
        d = t + start - time.monotonic()
        if d > 0:
            time.sleep(d)
        if k == "m":
            m = Mocap()
            m.header.frame_id = "map"
            m.header.stamp = rig.get_clock().now().to_msg()
            m.pose.position.x, m.pose.position.y, m.pose.position.z = (float(P[c][i]) for c in "xyz")
            m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = (
                float(Q[c][i]) for c in "xyzw")
            m.twist.linear.x, m.twist.linear.y, m.twist.linear.z = (float(V[c][i]) for c in "xyz")
            m.twist.angular.x, m.twist.angular.y, m.twist.angular.z = (float(W[c][i]) for c in "xyz")
            rig.mp.publish(m)
            proc.append(np.linalg.norm([V[c][i] for c in "xyz"]))
        else:
            o = VehicleOdometry()
            o.pose_frame, o.velocity_frame = int(pf[i]), int(vf[i])
            o.position = [float(x) for x in EP[i]]
            o.q = [float(x) for x in EQ[i]]
            o.velocity = [float(x) for x in EV[i]]
            o.angular_velocity = [float(x) for x in EW[i]]
            rig.ep.publish(o)
    time.sleep(0.3)
    out = np.array([np.linalg.norm([m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.linear.z])
                    for _, m in rig.out])
    ekf = np.linalg.norm(EV[sele], axis=1)
    return np.array(proc), out, ekf


def main():
    hold = sys.argv[1] if len(sys.argv) > 1 else "0.1"
    rclpy.init()
    rig = Rig()
    th = threading.Thread(target=rclpy.spin, args=(rig,), daemon=True)
    th.start()
    exe = os.path.expanduser("~/ros2_ws/install/fsc_drone_state_estimator_ros2/lib/"
                             "fsc_drone_state_estimator_ros2/indoor_mocap_feedback_node")
    node = subprocess.Popen([exe, "--ros-args", "-r", "__ns:=/vchk", "-p", "mocap_guard:=true",
                             "-p", f"mocap_guard_vel_holdoff_s:={hold}"],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
    try:
        time.sleep(3.0)
        print(f"mocap_guard_vel_holdoff_s {hold}:")
        for tag, t0, t1, what in EVENTS:
            proc, out, ekf = play(rig, tag, t0, t1)
            print(f"  {what:24s} ({tag} {t0}-{t1} s): processor max |v| {proc.max():.2f} m/s, "
                  f"published max |v| {out.max():.2f} m/s ({len(out)} samples), EKF2 max {ekf.max():.2f}")
    finally:
        os.killpg(node.pid, signal.SIGTERM)
        node.wait(timeout=10)
        rclpy.shutdown()
        th.join(timeout=5)


if __name__ == "__main__":
    main()
