#!/usr/bin/env python3
"""Arm the vehicle and put PX4 in OFFBOARD, without turning auto_arm on permanently.

WHY THIS EXISTS: `auto_arm` defaults FALSE on every controller by operator directive -- the
vehicle must never arm itself. But a controller has to be PUBLISHING OffboardControlMode before
PX4 will accept OFFBOARD, and the first few ARM requests after a restart are rejected while the
EKF settles. So the sequence is: make the safe controller active, let it stream, then request
OFFBOARD + ARM repeatedly until VehicleStatus says armed -- and put auto_arm back the way it was.

Usage: arm_vehicle.py [--ns uav_0] [--ctrl geometric] [--timeout 60] [--disarm]
"""
import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy)
from px4_msgs.msg import VehicleCommand, VehicleStatus
from std_msgs.msg import String


class Arm(Node):
    def __init__(self, ns, ctrl):
        super().__init__("arm_vehicle")
        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        self.cmd = self.create_publisher(VehicleCommand, f"/{ns}/fmu/in/vehicle_command", qos)
        self.ctl = self.create_publisher(String, f"/{ns}/ground_station/active_controller", 10)
        self.create_subscription(VehicleStatus, f"/{ns}/fmu/out/vehicle_status_v1",
                                 self.on_status, qos)
        self.st = None
        self.ctrl = ctrl

    def on_status(self, m):
        self.st = m

    def armed(self):
        return self.st is not None and int(self.st.arming_state) == 2

    def offboard(self):
        return self.st is not None and int(self.st.nav_state) == 14

    def send(self, command, p1=0.0, p2=0.0):
        m = VehicleCommand()
        m.command = command
        m.param1 = p1
        m.param2 = p2
        m.target_system = m.target_component = m.source_system = m.source_component = 1
        m.from_external = True
        m.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.cmd.publish(m)

    def hold_ctrl(self):
        self.ctl.publish(String(data=self.ctrl))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ns", default="uav_0")
    ap.add_argument("--ctrl", default="geometric")
    ap.add_argument("--timeout", type=float, default=60.0)
    ap.add_argument("--disarm", action="store_true")
    a = ap.parse_args()

    rclpy.init()
    n = Arm(a.ns, a.ctrl)
    t0 = time.time()
    while time.time() - t0 < 3.0:
        rclpy.spin_once(n, timeout_sec=0.1)
        n.hold_ctrl()

    if a.disarm:
        for _ in range(20):
            n.send(400, 0.0)
            rclpy.spin_once(n, timeout_sec=0.1)
        print(f"disarm requested; armed={n.armed()}")
        return 0

    t0 = time.time()
    tries = 0
    while time.time() - t0 < a.timeout:
        n.hold_ctrl()
        if n.armed() and n.offboard():
            print(f"ARMED + OFFBOARD after {time.time()-t0:.1f}s ({tries} requests)")
            return 0
        # PX4 rejects the first requests while the EKF settles -- keep asking.
        n.send(176, 1.0, 6.0)      # DO_SET_MODE -> custom main mode 6 = OFFBOARD
        n.send(400, 1.0)           # COMPONENT_ARM_DISARM -> arm
        tries += 1
        for _ in range(10):
            rclpy.spin_once(n, timeout_sec=0.05)
    st = n.st
    print(f"FAILED to arm in {a.timeout:.0f}s "
          f"(arming_state={getattr(st,'arming_state','?')} nav_state={getattr(st,'nav_state','?')})")
    return 1


if __name__ == "__main__":
    sys.exit(main())
