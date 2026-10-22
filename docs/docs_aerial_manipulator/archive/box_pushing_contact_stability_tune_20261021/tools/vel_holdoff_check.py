#!/usr/bin/env python3
"""Check of the 2026-10-21 estimator velocity hold-off (mocap_guard_vel_holdoff_s): the processor's
twist is a finite difference of the published poses, so it is 0 while a pose is republished frozen
and SPIKES right after the pause. The published odometry must carry EKF2's velocity then, and the
processor's otherwise. Feeds /uav_0/mocap at 120 Hz with twist = FD of the published positions
(exactly what the processor does) and a fake EKF2 (true velocity). Run beside
  ros2 launch fsc_drone_state_estimator_ros2 indoor_mocap_feedback_launch.py uav_prefix:=uav_0 mocap_guard:=true
on a private domain."""
import math, threading, time, os
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry
from fsc_autopilot_ros2_msgs.msg import Mocap
from std_msgs.msg import Float64MultiArray

def truth(t):
    return np.array([0.3 * math.sin(0.8 * t), 0.3 * math.cos(1.0 * t), 1.2])
def vel(t):
    return np.array([0.24 * math.cos(0.8 * t), -0.3 * math.sin(1.0 * t), 0.0])

class N(Node):
    def __init__(self):
        super().__init__("vel_holdoff_check")
        self.t0 = time.time(); self.lock = threading.Lock()
        self.freeze_until = -1.0; self.prev = None; self.frozen = None
        self.rows = []      # (t, published mocap twist x,y, odom twist x,y, true vx,vy)
        self.last_mocap_tw = None; self.health = []
        self.mp = self.create_publisher(Mocap, "/uav_0/mocap", 10)
        self.ep = self.create_publisher(VehicleOdometry, "/uav_0/fmu/out/vehicle_odometry", qos_profile_sensor_data)
        self.create_subscription(Odometry, "/uav_0/state_estimator/local_position/odom", self.on_odom, 50)
        self.create_subscription(Float64MultiArray, "/uav_0/state_estimator/mocap_guard",
                                 lambda m: self.health.append((time.time() - self.t0, m.data[0])), qos_profile_sensor_data)
        self.create_timer(1 / 120.0, self.pub_mocap); self.create_timer(1 / 100.0, self.pub_ekf)
    def pub_mocap(self):
        now = time.time(); t = now - self.t0
        with self.lock:
            p = truth(t)
            if t < self.freeze_until:
                if self.frozen is None: self.frozen = self.prev.copy() if self.prev is not None else p
                p = self.frozen
            else:
                self.frozen = None
            tw = np.zeros(3) if self.prev is None else (p - self.prev) * 120.0   # the processor's FD
            self.prev = p.copy(); self.last_mocap_tw = tw
        m = Mocap(); m.header.stamp.sec = int(now); m.header.stamp.nanosec = int((now - int(now)) * 1e9)
        m.header.frame_id = "map"
        m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, p)
        m.pose.orientation.w = 1.0
        m.twist.linear.x, m.twist.linear.y, m.twist.linear.z = map(float, tw)
        self.mp.publish(m)
    def pub_ekf(self):
        t = time.time() - self.t0; p = truth(t); v = vel(t)
        o = VehicleOdometry(); o.timestamp = int(t * 1e6); o.timestamp_sample = o.timestamp
        o.pose_frame = VehicleOdometry.POSE_FRAME_NED; o.position = [float(p[1]), float(p[0]), float(-p[2])]
        o.q = [1.0, 0.0, 0.0, 0.0]; o.velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED
        o.velocity = [float(v[1]), float(v[0]), float(-v[2])]; o.angular_velocity = [0.0, 0.0, 0.0]
        self.ep.publish(o)
    def on_odom(self, m):
        t = time.time() - self.t0; v = vel(t)
        with self.lock:
            mt = self.last_mocap_tw
        self.rows.append((t, m.twist.twist.linear.x, m.twist.twist.linear.y, v[0], v[1],
                          np.nan if mt is None else mt[0], np.nan if mt is None else mt[1]))

def main():
    rclpy.init(); n = N(); threading.Thread(target=rclpy.spin, args=(n,), daemon=True).start()
    ok = True
    def check(c, w):
        nonlocal ok; ok &= bool(c); print(("PASS " if c else "FAIL ") + w, flush=True)
    time.sleep(4.0)
    R = lambda a, b: np.array([r for r in n.rows if a <= r[0] <= b])
    t1 = time.time() - n.t0; time.sleep(1.5); h = R(t1, t1 + 1.5)
    e_fd = np.max(np.hypot(h[:, 1] - h[:, 5], h[:, 2] - h[:, 6]))
    check(len(h) > 100 and e_fd < 1e-6, f"healthy: odometry twist = the processor's twist (max diff {e_fd:.2e} m/s, {len(h)} msgs)")
    for k, pause in enumerate((0.09, 0.03)):
        tp = time.time() - n.t0
        with n.lock: n.freeze_until = tp + pause
        time.sleep(2.0)
        w = R(tp, tp + pause + 0.10)          # the pause + the hold-off after it
        spike = np.max(np.hypot(w[:, 5], w[:, 6]))
        e_ekf = np.max(np.hypot(w[:, 1] - w[:, 3], w[:, 2] - w[:, 4]))
        pk = np.max(np.hypot(w[:, 1], w[:, 2]))
        check(spike > 1.0 if pause > 0.05 else True,
              f"pause {pause * 1e3:.0f} ms: the processor's FD twist spikes to {spike:.2f} m/s")
        check(pk < 0.5 and e_ekf < 0.05, f"pause {pause * 1e3:.0f} ms: odometry twist peak {pk:.3f} m/s, "
              f"max {e_ekf * 1e3:.1f} mm/s from EKF2's true velocity (the spike held off)")
        late = R(tp + pause + 1.2, tp + pause + 1.9)
        e_late = np.max(np.hypot(late[:, 1] - late[:, 5], late[:, 2] - late[:, 6]))
        check(e_late < 1e-6, f"pause {pause * 1e3:.0f} ms: 1.2 s later the odometry twist is the processor's again ({e_late:.2e})")
    print("ALL PASS" if ok else "SOME FAILED", flush=True); os._exit(0 if ok else 1)

if __name__ == "__main__":
    main()
