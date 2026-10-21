#!/usr/bin/env python3
"""Loopback check of the raw-mocap guard in the running estimator (no Isaac, no PX4).

Start the estimator first, on a private domain:
  ROS_DOMAIN_ID=78 ros2 launch fsc_drone_state_estimator_ros2 indoor_mocap_feedback_launch.py \
      uav_prefix:=uav_0 mocap_guard:=true
then run this (same domain, the system python with the workspace sourced).

It feeds /uav_0/mocap (120 Hz, a moving pose) and a fake EKF2 on
/uav_0/fmu/out/vehicle_odometry (100 Hz, the same motion + a 12 mm offset, NED),
and checks, in order:
  1 healthy: the odometry IS the mocap, the health says MOCAP, every sample forwarded to PX4
  2 a 2.0 s FREEZE (fresh stamps): BRIDGING -> CONFIRMED -> LOST; the odometry is EKF2 + the
    offset frozen at the last good sample (= the true motion, continuous at the switch); NO
    frozen sample forwarded to PX4 beyond the first ~50 ms
  3 recovery: back to MOCAP after the 1 s recovery window, the odometry blended back
  4 a one-sample FLIP: rejected (not forwarded), bridged for a moment, never CONFIRMED
  5 the test_fault service (0.5 s default): BRIDGING -> CONFIRMED -> MOCAP, never LOST, reason "test"
  6 EKF2 silent during a fault: no odometry at all (the flight node's timeout then trips)
  7 a second fault DURING the blend back (2026-10-21 review): EKF2 drifts 0.225 m in a
    1.5 s freeze, so the recovery blends back over ~2.2 s; 0.6 s into it a 0.3 s freeze --
    the odometry must not step (the old bridge jumped by the blend left, ~0.16 m) and
    must end on the mocap again
With --guard-off: checks the plain pass-through (no health topic, every sample forwarded).
"""

import argparse
import math
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
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger

OFFSET = np.array([0.012, -0.007, 0.004])   # EKF2 = truth - OFFSET  (so mocap - EKF2 = OFFSET)


def truth(t):
    # smooth: |v| <= ~0.25 m/s, nothing a guard should ever reject
    return np.array([0.3 * math.sin(0.4 * t), 0.3 * math.cos(0.5 * t), 1.2 + 0.05 * math.sin(0.3 * t)])


class Loop(Node):
    def __init__(self, ns):
        super().__init__("guard_loopback")
        self.ns = ns
        self.t0 = time.time()
        self.mode = "normal"      # normal | freeze | flip | mute_ekf
        self.frozen = None
        self.flip_once = False
        self.ekf_on = True
        self.ekf_drift = np.zeros(3)     # EKF2's dead-reckoning error, grown at drift_rate
        self.drift_rate = np.zeros(3)
        self.lock = threading.Lock()
        self.odom, self.health, self.fwd = [], [], []
        self.mocap_pub = self.create_publisher(Mocap, f"{ns}/mocap", 10)
        self.ekf_pub = self.create_publisher(VehicleOdometry, f"{ns}/fmu/out/vehicle_odometry",
                                             qos_profile_sensor_data)
        self.create_subscription(Odometry, f"{ns}/state_estimator/local_position/odom", self.on_odom, 50)
        self.create_subscription(Float64MultiArray, f"{ns}/state_estimator/mocap_guard", self.on_health,
                                 qos_profile_sensor_data)
        self.create_subscription(VehicleOdometry, f"{ns}/fmu/in/vehicle_visual_odometry", self.on_fwd,
                                 qos_profile_sensor_data)
        self.test_cli = self.create_client(Trigger, f"{ns}/state_estimator/mocap_guard/test_fault")
        self.create_timer(1.0 / 120.0, self.pub_mocap)
        self.create_timer(1.0 / 100.0, self.pub_ekf)

    def now(self):
        return time.time() - self.t0

    def pub_mocap(self):
        now = time.time()
        t = now - self.t0
        p = truth(t)
        m = Mocap()
        m.header.stamp.sec = int(now)
        m.header.stamp.nanosec = int((now - int(now)) * 1e9)
        m.header.frame_id = "map"
        q = [1.0, 0.0, 0.0, 0.0]
        with self.lock:
            if self.mode == "freeze":
                if self.frozen is None:
                    self.frozen = p.copy()
                p = self.frozen
            else:
                self.frozen = None
            if self.flip_once:
                self.flip_once = False
                p = p + np.array([0.18, 0.0, 0.0])
                q = [math.cos(math.radians(55)), 0.0, 0.0, math.sin(math.radians(55))]
        m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, p)
        m.pose.orientation.w, m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z = q
        self.mocap_pub.publish(m)

    def pub_ekf(self):
        if not self.ekf_on:
            return
        t = self.now()
        with self.lock:
            self.ekf_drift = self.ekf_drift + self.drift_rate / 100.0
            p = truth(t) - OFFSET + self.ekf_drift
        v = (truth(t + 1e-3) - truth(t - 1e-3)) / 2e-3
        o = VehicleOdometry()
        o.timestamp = int(t * 1e6)
        o.timestamp_sample = o.timestamp
        o.pose_frame = VehicleOdometry.POSE_FRAME_NED
        o.position = [float(p[1]), float(p[0]), float(-p[2])]        # ENU -> NED
        o.q = [1.0, 0.0, 0.0, 0.0]
        o.velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED
        o.velocity = [float(v[1]), float(v[0]), float(-v[2])]
        o.angular_velocity = [0.0, 0.0, 0.0]
        self.ekf_pub.publish(o)

    def on_odom(self, m):
        p = m.pose.pose.position
        ts = m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec - self.t0
        with self.lock:
            self.odom.append((self.now(), p.x, p.y, p.z, ts))

    def on_health(self, m):
        with self.lock:
            self.health.append((self.now(), *m.data))

    def on_fwd(self, m):
        with self.lock:
            self.fwd.append((self.now(), *m.position))


def window(rows, a, b):
    return [r for r in rows if a <= r[0] < b]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ns", default="/uav_0")
    ap.add_argument("--guard-off", action="store_true")
    a = ap.parse_args()
    rclpy.init()
    n = Loop(a.ns)
    threading.Thread(target=rclpy.spin, args=(n,), daemon=True).start()
    ok = True

    def check(cond, what):
        nonlocal ok
        ok &= bool(cond)
        print(("PASS " if cond else "FAIL ") + what, flush=True)

    def err_vs_truth(rows):
        # against the truth at each message's own stamp (no transport delay in it)
        return max((np.linalg.norm(np.array(r[1:4]) - truth(r[4] if len(r) > 4 else r[0])) for r in rows),
                   default=float("nan"))

    time.sleep(4.0)
    # 1 healthy
    t = n.now()
    time.sleep(2.0)
    od, he, fw = window(n.odom, t, n.now()), window(n.health, t, n.now()), window(n.fwd, t, n.now())
    check(len(od) > 200, f"healthy: odometry flows ({len(od)} msgs in 2 s)")
    check(err_vs_truth(od) < 1e-5, f"healthy: odometry = the mocap (max err {err_vs_truth(od) * 1e3:.4f} mm)")
    check(len(fw) > 200, f"healthy: forwarded to PX4 ({len(fw)} in 2 s)")
    if a.guard_off:
        check(not n.health, "guard off: no health topic")
        print("ALL PASS" if ok else "SOME FAILED", flush=True)
        import os
        os._exit(0 if ok else 1)
    check(he and all(h[1] == 0 for h in he), f"healthy: health MOCAP ({len(he)} msgs, {len(he) / 2:.0f} Hz)")
    # 2 freeze 2.0 s
    t_f = n.now()
    with n.lock:
        n.mode = "freeze"
    time.sleep(2.0)
    with n.lock:
        n.mode = "normal"
    t_e = n.now()
    he = window(n.health, t_f, t_e + 0.1)
    states = [int(h[1]) for h in he]
    seen = [s for i, s in enumerate(states) if i == 0 or s != states[i - 1]]
    check(seen[:4] == [0, 1, 2, 3] or seen[:3] == [1, 2, 3], f"freeze: state sequence {seen}")
    first = {s: next((h[0] - t_f for h in he if int(h[1]) == s), None) for s in (1, 2, 3)}
    check(first[1] is not None and first[1] < 0.15, f"freeze: BRIDGING after {first[1]}")
    check(first[2] is not None and 0.15 < first[2] < 0.35, f"freeze: CONFIRMED after {first[2]}")
    check(first[3] is not None and 1.0 < first[3] < 1.2, f"freeze: LOST after {first[3]}")
    check(all(int(h[2]) == 2 for h in he if h[1] > 0), "freeze: cause = frozen")
    od = window(n.odom, t_f + 0.2, t_e)
    check(len(od) > 100, f"freeze: the bridge publishes ({len(od)} msgs)")
    e = err_vs_truth(od)
    check(e < 0.002, f"freeze: the bridged odometry follows the TRUE motion (max err {e * 1e3:.2f} mm)")
    fw = window(n.fwd, t_f + 0.1, t_e)
    check(len(fw) == 0, f"freeze: frozen samples forwarded to PX4 after 0.1 s: {len(fw)}")
    # 3 recovery
    time.sleep(2.5)
    he = window(n.health, t_e, n.now())
    back = next((h[0] - t_e for h in he if int(h[1]) == 0), None)
    check(back is not None and 0.9 < back < 1.3, f"recovery: MOCAP again after {back} s (1 s window)")
    od = window(n.odom, t_e, n.now())
    jumps = max((np.linalg.norm(np.array(od[i][1:4]) - np.array(od[i - 1][1:4]))
                 for i in range(1, len(od))), default=0.0)
    check(jumps < 0.005, f"recovery: no step in the odometry (largest step {jumps * 1e3:.2f} mm)")
    check(err_vs_truth(window(n.odom, n.now() - 0.5, n.now())) < 1e-5, "recovery: odometry = the mocap again")
    # 4 one-sample flip
    t_f = n.now()
    with n.lock:
        n.flip_once = True
    time.sleep(1.0)
    he = window(n.health, t_f, n.now())
    check(not any(int(h[1]) >= 2 for h in he), "flip: never CONFIRMED")
    fw = window(n.fwd, t_f, n.now())
    check(not any(abs(f[2] - truth(f[0])[0]) > 0.1 for f in fw), "flip: the flipped sample NOT forwarded to PX4")
    od = window(n.odom, t_f, n.now())
    check(err_vs_truth(od) < 0.002, f"flip: odometry never shows the flip (max err {err_vs_truth(od) * 1e3:.2f} mm)")
    # 5 test fault
    n.test_cli.wait_for_service(timeout_sec=5.0)
    t_f = n.now()
    fut = n.test_cli.call_async(Trigger.Request())
    while not fut.done():
        time.sleep(0.01)
    check(fut.result().success, f"test fault: {fut.result().message}")
    time.sleep(3.0)
    he = window(n.health, t_f, n.now())
    states = sorted(set(int(h[1]) for h in he))
    check(2 in states and 3 not in states and 0 in states,
          f"test fault: states seen {states} (CONFIRMED then MOCAP, never LOST)")
    check(all(int(h[2]) == 4 for h in he if h[1] > 0), "test fault: cause = test")
    # 6 EKF2 silent during a fault
    n.ekf_on = False
    t_f = n.now()
    with n.lock:
        n.mode = "freeze"
    time.sleep(1.0)
    od = window(n.odom, t_f + 0.3, n.now())
    check(len(od) == 0, f"EKF2 silent + mocap frozen: no odometry ({len(od)} msgs)")
    he = window(n.health, t_f + 0.3, n.now())
    check(he and all(int(h[4]) == 3 for h in he), "EKF2 silent: health says source NONE")
    with n.lock:
        n.mode = "normal"
    n.ekf_on = True
    time.sleep(2.0)
    # 7 a second fault during the blend back
    t7 = n.now()
    with n.lock:
        n.drift_rate = np.array([0.15, 0.0, 0.0])
        n.mode = "freeze"
    time.sleep(1.5)
    with n.lock:
        n.drift_rate = np.zeros(3)
        n.mode = "normal"
    t_e = n.now()
    back = None
    while back is None and n.now() < t_e + 3.0:
        back = next((h[0] for h in window(n.health, t_e, n.now()) if int(h[1]) == 0), None)
        time.sleep(0.01)
    check(back is not None, f"re-fault: the first fault recovered ({None if back is None else round(back - t_e, 2)} s)")
    time.sleep(0.6)
    blending = [h for h in window(n.health, back or t_e, n.now()) if int(h[4]) == 2]
    check(blending, f"re-fault: a blend back was running when the second fault came ({len(blending)} health msgs)")
    with n.lock:
        n.mode = "freeze"
    time.sleep(0.3)
    with n.lock:
        n.mode = "normal"
    time.sleep(1.3 + 3.5)    # recover (1 s of good samples), then the rest of the blend
    he = window(n.health, t7, n.now())
    starts = sum(1 for i in range(1, len(he)) if int(he[i][1]) > 0 and int(he[i - 1][1]) == 0)
    check(starts >= 2, f"re-fault: two episodes seen ({starts})")
    # judged on the odometry's OFFSET from the truth (at each message's stamp): the
    # blend back moves it smoothly; a freeze's first ~50-80 ms (published before it is
    # recognised as frozen) dips it by up to v x that time, ~20 mm here, and the bridge
    # then takes it back -- that is the frozen detector's latency, not a step. The old
    # bridge lost the blend left at the second fault: ~160 mm in one message.
    od = window(n.odom, t7, n.now())
    off = [np.array(r[1:4]) - truth(r[4]) for r in od]
    jumps = max((np.linalg.norm(off[i] - off[i - 1]) for i in range(1, len(off))), default=0.0)
    check(jumps < 0.030, f"re-fault: the odometry's offset never steps (largest {jumps * 1e3:.2f} mm "
                         "between messages, freeze-onset latency included)")
    e = err_vs_truth(window(n.odom, n.now() - 0.5, n.now()))
    check(e < 1e-5, f"re-fault: odometry = the mocap again at the end (max err {e * 1e3:.4f} mm)")
    print("ALL PASS" if ok else "SOME FAILED", flush=True)
    import os
    os._exit(0 if ok else 1)


if __name__ == "__main__":
    sys.exit(main())
