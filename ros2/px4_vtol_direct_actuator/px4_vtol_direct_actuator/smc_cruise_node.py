#!/usr/bin/env python3
"""CRUISE outer-loop SMC — a SEPARATE SMC tuned for wing-borne cruise (fast).

Same sliding-mode law as smc_outer (converging set-point + robust velocity FF) but the set-point
is allowed to LEAD the vehicle FAR ahead (big reset_dist) and at HIGH speed (big vmax). The plain
smc_outer keeps its set-point only ~8 m ahead, so the inner nmpc_cruise sqrt-decelerates toward it
and caps at ~5 m/s -> never reaches wing-borne speed. Leading far ahead lets nmpc build to ~15-18 m/s
(where the SCALED wing carries the aircraft's weight and the rotors unload = genuine wing-borne).

    GS/target --(reference)--> smc_cruise --(reference_cruise)--> nmpc_cruise (remap)
"""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from fsc_autopilot_ros2_msgs.msg import PositionControllerReference, Mocap

from smc_outer import SMCOuter, _KEYS


class SMCCruiseNode(Node):
    def __init__(self):
        super().__init__("smc_cruise")
        P = self.declare_parameter
        P("px4_ns", "uav_0")
        P("rate_hz", 50.0)
        P("in_topic", "fsc_autopilot_ros2/position_controller/reference")
        P("out_topic", "fsc_autopilot_ros2/position_controller/reference_cruise")
        # CRUISE gains: lead FAR ahead (reset_dist) at HIGH speed (vmax) so the inner loop can
        # accelerate to wing-borne speed; smoother reaching (lower lam) for a gentle build-up.
        P("land_z", 1.2); P("reset_dist", 30.0); P("amax", 2.5)   # gentle accel -> no yaw-flip lurch
        P("lam", 0.6); P("K", 1.2); P("phi", 3.0); P("vmax", 14.0); P("vzmax", 3.0)
        P("lam_land", 1.6); P("K_land", 0.6); P("phi_land", 0.4)
        P("vmax_land", 1.5); P("vzmax_land", 0.6)

        ns = self.get_parameter("px4_ns").value
        pre = f"/{ns}" if ns else ""
        self.rate = float(self.get_parameter("rate_hz").value)
        self.smc = SMCOuter(**{k: self.gp(k) for k in _KEYS})
        self.target = {"x": 0.0, "y": 0.0, "z": 1.5, "yaw": 0.0}
        self.p = None

        it = self.get_parameter("in_topic").value
        ot = self.get_parameter("out_topic").value
        self.pub = self.create_publisher(PositionControllerReference, f"{pre}/{ot}", 10)
        self.create_subscription(PositionControllerReference, f"{pre}/{it}", self.on_ref, 10)
        self.create_subscription(Mocap, f"{pre}/mocap", self.on_mocap, qos_profile_sensor_data)
        self.create_timer(1.0 / self.rate, self.loop)
        self.get_logger().info(f"smc_cruise up: {pre}/{it} -> {pre}/{ot} @ {self.rate:.0f} Hz")

    def gp(self, n):
        return float(self.get_parameter(n).value)

    def on_ref(self, m):
        self.target["x"], self.target["y"], self.target["z"] = m.position.x, m.position.y, m.position.z
        self.target["yaw"] = (m.yaw * math.pi / 180.0
                              if m.yaw_unit == PositionControllerReference.DEGREES else m.yaw)

    def on_mocap(self, m):
        self.p = np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z])

    def loop(self):
        if self.p is None:
            return
        self.smc.set_params(**{k: self.gp(k) for k in _KEYS})
        sp, v = self.smc.update(self.p, [self.target["x"], self.target["y"], self.target["z"]],
                                1.0 / self.rate)
        msg = PositionControllerReference()
        msg.position.x, msg.position.y, msg.position.z = float(sp[0]), float(sp[1]), float(sp[2])
        msg.velocity.x, msg.velocity.y, msg.velocity.z = float(v[0]), float(v[1]), float(v[2])
        msg.yaw = math.degrees(self.target["yaw"])
        msg.yaw_unit = PositionControllerReference.DEGREES
        self.pub.publish(msg)


def main():
    rclpy.init()
    n = SMCCruiseNode()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
