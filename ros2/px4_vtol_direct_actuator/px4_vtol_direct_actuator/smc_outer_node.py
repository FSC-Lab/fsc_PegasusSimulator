#!/usr/bin/env python3
"""Standalone OUTER-LOOP sliding-mode controller node.

Subscribes the RAW position target (from the GS / a trajectory) and the vehicle mocap,
runs the SMC (converging set-point + robust velocity, loose/strict regimes), and republishes
an AUGMENTED PositionControllerReference (set-point position + velocity feed-forward) on a
separate topic. An inner-loop controller (e.g. the geometric traj_track) subscribes to that
augmented topic (remap its reference input to `out_topic`). This is path A of the
SMC-outer / stable-inner cascade.

    GS/target --(in_topic)--> smc_outer --(out_topic)--> inner loop (traj_track)
"""
# ★ Run-as-a-script bootstrap. The imports below are absolute
# (`from px4_vtol_direct_actuator...`), which is correct for an installed
# package but needs the package's PARENT on sys.path. That is true under
# `ros2 run` and `python3 -m`, and false under `python3 this_file.py` -- which
# is how every launcher and every offline gate in this project invokes it.
# Adding the parent here keeps both modes working without a global path shim.
if __package__ in (None, ""):
    import os as _os, sys as _sys
    _sys.path.insert(0, _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__))))
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from fsc_autopilot_ros2_msgs.msg import PositionControllerReference, Mocap

from px4_vtol_direct_actuator.lib.smc_outer import SMCOuter, _KEYS


class SMCOuterNode(Node):
    def __init__(self):
        super().__init__("smc_outer")
        P = self.declare_parameter
        P("px4_ns", "uav_0")
        P("rate_hz", 50.0)
        P("in_topic", "fsc_autopilot_ros2/position_controller/reference")
        P("out_topic", "fsc_autopilot_ros2/position_controller/reference_smc")
        # SMC gains (loose = cruise/transition, *_land = strict/landing)
        P("land_z", 1.2); P("reset_dist", 8.0); P("amax", 100.0)   # amax = velocity-cmd accel limit
        P("lam", 0.9); P("K", 1.5); P("phi", 2.0); P("vmax", 20.0); P("vzmax", 3.0)
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
        self.get_logger().info(f"smc_outer up: {pre}/{it} -> {pre}/{ot} @ {self.rate:.0f} Hz")

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
        self.smc.set_params(**{k: self.gp(k) for k in _KEYS})   # live-tunable gains
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
    n = SMCOuterNode()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
