#!/usr/bin/env python3
"""Publish a RAW PositionControllerReference (drives SMC -> inner controller).
   Usage: pubref.py X Y Z [YAW_DEG] [--rate HZ] [--secs S]   (default: stream 5 Hz forever)
   Lives in the repo -- /tmp gets wiped (see memory tmp-wiped-persist-tooling)."""
import sys, math
import rclpy
from rclpy.node import Node
from fsc_autopilot_ros2_msgs.msg import PositionControllerReference

a = [x for x in sys.argv[1:] if not x.startswith("--")]
X, Y, Z = float(a[0]), float(a[1]), float(a[2])
YAW = float(a[3]) if len(a) > 3 else 0.0
rate = float(sys.argv[sys.argv.index("--rate") + 1]) if "--rate" in sys.argv else 5.0
secs = float(sys.argv[sys.argv.index("--secs") + 1]) if "--secs" in sys.argv else 0.0
NS = "uav_0"
rclpy.init()
n = Node("pubref")
pub = n.create_publisher(PositionControllerReference,
                         f"/{NS}/fsc_autopilot_ros2/position_controller/reference", 10)
m = PositionControllerReference()
m.position.x, m.position.y, m.position.z = X, Y, Z
m.velocity.x = m.velocity.y = m.velocity.z = 0.0
m.yaw = YAW
try:
    m.yaw_unit = PositionControllerReference.DEGREES
except Exception:
    pass
print(f"[pubref] streaming ({X},{Y},{Z}) yaw={YAW} at {rate} Hz  (Ctrl-C to stop)")
i = 0
try:
    while rclpy.ok():
        pub.publish(m); rclpy.spin_once(n, timeout_sec=0.0)
        i += 1
        import time; time.sleep(1.0 / rate)
        if secs > 0 and i / rate >= secs:
            break
except KeyboardInterrupt:
    pass
print("[pubref] done")
