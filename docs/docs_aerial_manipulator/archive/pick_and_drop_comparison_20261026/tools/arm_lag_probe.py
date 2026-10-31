#!/usr/bin/env python3
"""Passive probe of the arm torque path (2026-10-26): receive time + values of
  A  /uav_0/fsc_open_manipulator/external_torque_controller/joint_torque_command   (the law's command)
  B  /uav_0/isaacsim_manipulator/effort_commands                                   (the arm stack's output to Isaac)
  C  /uav_0/isaacsim_manipulator/joint_states                                      (Isaac's APPLIED effort)
for <seconds>, then saves an npz. Read-only: it publishes nothing.
    /usr/bin/python3 arm_lag_probe.py <seconds> <out.npz>"""
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

NAMES = ["joint1", "joint2", "joint3", "joint4"]


class Probe(Node):
    def __init__(self):
        super().__init__("arm_lag_probe_passive")
        self.rows = {"A": [], "B": [], "C": []}
        for k, topic in (("A", "/uav_0/fsc_open_manipulator/external_torque_controller/joint_torque_command"),
                         ("B", "/uav_0/isaacsim_manipulator/effort_commands"),
                         ("C", "/uav_0/isaacsim_manipulator/joint_states")):
            self.create_subscription(JointState, topic, lambda m, k=k: self.cb(k, m), qos_profile_sensor_data)

    def cb(self, k, m):
        names = list(m.name)
        try:
            e = [m.effort[names.index(n)] for n in NAMES]
        except (ValueError, IndexError):
            return
        st = m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec
        self.rows[k].append([time.time(), st] + e)


def main():
    dur, out = float(sys.argv[1]), sys.argv[2]
    rclpy.init()
    n = Probe()
    t_end = time.time() + dur
    while time.time() < t_end:
        rclpy.spin_once(n, timeout_sec=0.05)
    np.savez(out, **{k: np.array(v) for k, v in n.rows.items()})
    print({k: len(v) for k, v in n.rows.items()}, "->", out)


if __name__ == "__main__":
    main()
