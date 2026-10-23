#!/usr/bin/env python3
"""Stream an end-effector force profile into 07's injector during whole-body
DIRECT, and record what the controller made of it (2026-09-27).

    /usr/bin/python3 interaction_force_driver.py --out run.npz \
        --seg 15:15:radial:4 --seg 42:15:down:3 ...

Each --seg is  t0:hold:dir:F  -- start t0 SIM seconds after DIRECT entry, a
1 s min-snap ramp in, `hold` s at F newtons, a 1 s ramp out. dir is relative
to the vehicle HEADING at the first segment: radial = along the arm (body +x,
horizontal), -radial, lateral = body +y (horizontal), -lateral, up, down.
Time is SIM time (07's truth stamp), so the profile is the same at any RTF.

Records at the arrival rate: wb_control_debug (all fields), 07's applied force
+ grasp point, odometry pose, the WB mode string. Ends 12 sim-s after the last
segment, or on SAFETY."""
import argparse, math, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import Float32MultiArray, Float64MultiArray, String
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry

NS = "/uav_0"
DA = f"{NS}/fsc_autopilot_ros2/whole_body_direct_actuation"


def minsnap(x):
    x = min(max(x, 0.0), 1.0)
    return 35 * x**4 - 84 * x**5 + 70 * x**6 - 20 * x**7


class Driver(Node):
    def __init__(self, a):
        super().__init__("interaction_force_driver")
        self.segs = []
        for s in a.seg:
            t0, hold, d, F = s.split(":")
            self.segs.append((float(t0), float(hold), d, float(F)))
        self.mode = None; self.t_entry = None; self.t_sim = None; self.yaw0 = None
        self.odom = None; self.done = False; self.t_end = max(t0 + h + 2 for t0, h, _, _ in self.segs) + 12.0
        self.rec = {"wb_t": [], "wb": [], "fs": [], "od": [], "cmd": [], "mode": []}
        self.pub = self.create_publisher(Vector3Stamped, f"{NS}/isaacsim_manipulator/ee_force_cmd", 10)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, f"{DA}/mode", self.on_mode, latched)
        self.create_subscription(Float32MultiArray, f"{DA}/wb_control_debug", self.on_wb, 50)
        self.create_subscription(Float64MultiArray, f"{NS}/isaacsim_manipulator/ee_force_state", self.on_fs, 50)
        self.create_subscription(Odometry, f"{NS}/state_estimator/local_position/odom", self.on_od, 20)
        self.create_timer(1.0 / a.rate, self.tick)
        self.last_print = 0.0

    def on_mode(self, m):
        if m.data != self.mode:
            self.get_logger().info(f"mode -> {m.data} (t_sim {self.t_sim})")
            self.rec["mode"].append((time.time(), self.t_sim if self.t_sim is not None else -1.0, m.data))
        if m.data == "DIRECT" and self.mode != "DIRECT" and self.t_sim is not None:
            self.t_entry = self.t_sim
        if m.data != "DIRECT" and self.mode == "DIRECT" and self.t_entry is not None:
            self.get_logger().warn("left DIRECT -- stopping the profile")
            self.done = True
        self.mode = m.data

    def on_wb(self, m):
        self.rec["wb_t"].append((time.time(), self.t_sim if self.t_sim is not None else np.nan))
        v = np.full(115, np.nan); d = np.asarray(m.data, float); v[:min(len(d), 115)] = d[:115]
        self.rec["wb"].append(v)

    def on_fs(self, m):
        self.t_sim = m.data[0]
        self.rec["fs"].append([time.time(), *m.data])
        if self.mode == "DIRECT" and self.t_entry is None:
            self.t_entry = self.t_sim

    def on_od(self, m):
        p = m.pose.pose.position; q = m.pose.pose.orientation
        self.odom = (p.x, p.y, p.z, q.w, q.x, q.y, q.z)
        self.rec["od"].append([time.time(), self.t_sim if self.t_sim is not None else np.nan, *self.odom])

    def force(self, tr):
        F = np.zeros(3)
        c, s = math.cos(self.yaw0), math.sin(self.yaw0)
        dirs = {"radial": (c, s, 0), "-radial": (-c, -s, 0), "lateral": (-s, c, 0),
                "-lateral": (s, -c, 0), "up": (0, 0, 1), "down": (0, 0, -1)}
        for t0, hold, d, Fm in self.segs:
            k = minsnap(tr - t0) - minsnap(tr - t0 - 1.0 - hold)
            F += Fm * k * np.asarray(dirs[d], float)
        return F

    def tick(self):
        if self.done or self.t_entry is None or self.t_sim is None:
            return
        tr = self.t_sim - self.t_entry
        if self.yaw0 is None and self.odom is not None:
            w, x, y, z = self.odom[3:]
            self.yaw0 = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
            self.get_logger().info(f"DIRECT at t_sim {self.t_entry:.2f}; heading {math.degrees(self.yaw0):.1f} deg; "
                                   f"profile ends at +{self.t_end:.0f} s sim")
        if self.yaw0 is None:
            return
        F = self.force(tr)
        m = Vector3Stamped(); m.header.stamp = self.get_clock().now().to_msg(); m.header.frame_id = "world"
        m.vector.x, m.vector.y, m.vector.z = map(float, F)
        self.pub.publish(m)
        self.rec["cmd"].append([time.time(), self.t_sim, tr, *F])
        if time.time() - self.last_print > 5.0:
            self.last_print = time.time()
            self.get_logger().info(f"+{tr:6.1f} s sim  F_cmd {np.round(F, 2)}")
        if tr > self.t_end:
            self.done = True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seg", action="append", required=True)
    ap.add_argument("--rate", type=float, default=50.0)
    ap.add_argument("--out", required=True)
    ap.add_argument("--timeout", type=float, default=1500.0, help="wall s")
    a = ap.parse_args()
    rclpy.init()
    n = Driver(a)
    t0 = time.time()
    try:
        while rclpy.ok() and not n.done and time.time() - t0 < a.timeout:
            rclpy.spin_once(n, timeout_sec=0.05)
        # a few zero-force publishes so 07 drops the force at once
        for _ in range(10):
            m = Vector3Stamped(); n.pub.publish(m); rclpy.spin_once(n, timeout_sec=0.02)
    finally:
        R = n.rec
        np.savez_compressed(a.out, wb_t=np.array(R["wb_t"]), wb=np.array(R["wb"]), fs=np.array(R["fs"]),
                            od=np.array(R["od"]), cmd=np.array(R["cmd"]),
                            mode=np.array(R["mode"], dtype=object), t_entry=n.t_entry if n.t_entry else np.nan,
                            yaw0=n.yaw0 if n.yaw0 is not None else np.nan, segs=np.array(a.seg))
        print(f"saved {a.out}: {len(R['wb'])} debug, {len(R['fs'])} truth, {len(R['cmd'])} cmd samples")
        n.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
