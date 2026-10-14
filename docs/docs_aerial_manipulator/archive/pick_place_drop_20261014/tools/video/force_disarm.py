#!/usr/bin/env python3
"""SIMULATION ONLY: stop the propellers after the landing. PX4's land detector never
reports this rig landed, so the normal disarm is refused ("Disarming denied: not
landed"); this sends VEHICLE_CMD_COMPONENT_ARM_DISARM with PX4's force flag (param2 =
21196) until vehicle_status_v1 reports disarmed. Exits 0 once disarmed, 1 on timeout.
Refuses unless the body is within 0.06 m of its resting height (on the ground)."""
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import VehicleCommand, VehicleStatus
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

NS = "/uav_0"
REST_Z = 0.305          # the AM_xfwd gear's resting body height in the pick-and-place scene


class Disarm(Node):
    def __init__(self):
        super().__init__("video_force_disarm")
        rel = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=10,
                         reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.VOLATILE)
        be = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=5,
                        reliability=QoSReliabilityPolicy.BEST_EFFORT, durability=QoSDurabilityPolicy.VOLATILE)
        self.pub = self.create_publisher(VehicleCommand, f"{NS}/fmu/in/vehicle_command", rel)
        self.armed = None
        self.z = None
        self.create_subscription(VehicleStatus, f"{NS}/fmu/out/vehicle_status_v1",
                                 lambda m: setattr(self, "armed", m.arming_state == 2), be)
        self.create_subscription(PoseStamped, f"{NS}/state/pose",
                                 lambda m: setattr(self, "z", m.pose.position.z), be)

    def send(self):
        m = VehicleCommand()
        m.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        m.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        m.param1 = 0.0
        m.param2 = 21196.0
        m.target_system = m.source_system = 1
        m.target_component = m.source_component = 1
        m.from_external = True
        self.pub.publish(m)


def main():
    rclpy.init()
    n = Disarm()
    t0 = time.time()
    while time.time() - t0 < 20.0:
        rclpy.spin_once(n, timeout_sec=0.1)
        if n.armed is False:
            print("disarmed", flush=True)
            return 0
        if n.armed and n.z is not None:
            if abs(n.z - REST_Z) > 0.06:
                print(f"not on the ground (z {n.z:.3f}) -- refusing to force a disarm", flush=True)
                return 1
            n.send()
            time.sleep(0.4)
    print(f"timeout: armed={n.armed} z={n.z}", flush=True)
    return 1


if __name__ == "__main__":
    sys.exit(main())
