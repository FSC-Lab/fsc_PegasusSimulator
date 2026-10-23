#!/usr/bin/env python3
"""Record the vehicle, the marker cube and the arm through arming and takeoff.

CSV columns: t, body xyz, body roll/pitch/yaw (deg), cube xyz, |cube - body|,
q1..q4 (deg), motor0..3 (normalised PX4 command). 50 Hz, one row per body
pose sample, stops on Ctrl-C or --seconds.
"""
import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from fsc_autopilot_ros2_msgs.msg import Mocap
from px4_msgs.msg import ActuatorMotors

BE = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE)


def rpy(q):
    x, y, z, w = q
    r = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    p = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return [math.degrees(v) for v in (r, p, yw)]


class Trace(Node):
    def __init__(self, out, seconds):
        super().__init__("takeoff_trace")
        self.f = open(out, "w")
        self.f.write("t,bx,by,bz,roll,pitch,yaw,cx,cy,cz,dist,q1,q2,q3,q4,m0,m1,m2,m3\n")
        self.t0 = time.monotonic()
        self.seconds = seconds
        self.cube = None
        self.q = [float("nan")] * 4
        self.m = [float("nan")] * 4
        self.last = -1.0
        self.n = 0
        self.create_subscription(PoseStamped, "/uav_0/state/pose", self.on_pose, BE)
        self.create_subscription(Mocap, "/obj_0/mocap", self.on_cube, 10)
        self.create_subscription(JointState, "/uav_0/fsc_open_manipulator/joint_states",
                                 self.on_js, 10)
        self.create_subscription(ActuatorMotors, "/uav_0/fmu/out/actuator_motors",
                                 self.on_motors, BE)

    def on_cube(self, m):
        self.cube = (m.pose.position.x, m.pose.position.y, m.pose.position.z)

    def on_js(self, m):
        # broadcaster order on this rig is [q2, q3, q1, q4] -- relabel by name
        idx = {n: i for i, n in enumerate(m.name)}
        self.q = [math.degrees(m.position[idx[n]]) if n in idx else float("nan")
                  for n in ("joint1", "joint2", "joint3", "joint4")]

    def on_motors(self, m):
        self.m = list(m.control[:4])

    def on_pose(self, m):
        t = time.monotonic() - self.t0
        if t - self.last < 0.02:
            return
        self.last = t
        p = m.pose.position
        o = m.pose.orientation
        r, pch, yw = rpy((o.x, o.y, o.z, o.w))
        if self.cube:
            c = self.cube
            d = math.dist((p.x, p.y, p.z), c)
        else:
            c = (float("nan"),) * 3
            d = float("nan")
        row = [t, p.x, p.y, p.z, r, pch, yw, *c, d, *self.q, *self.m]
        self.f.write(",".join(f"{v:.4f}" for v in row) + "\n")
        self.n += 1
        if self.n % 50 == 0:
            self.f.flush()
        if self.seconds and t > self.seconds:
            self.f.close()
            raise SystemExit(0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--seconds", type=float, default=0.0)
    a = ap.parse_args()
    rclpy.init()
    n = Trace(a.out, a.seconds)
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        n.f.close()
        rclpy.try_shutdown()


if __name__ == "__main__":
    sys.exit(main())
