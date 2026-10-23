#!/usr/bin/env python3
"""Record the controller's position/velocity feedback beside Isaac's ground truth.

    /usr/bin/python3 fused_feedback_recorder.py --out run.npz [--namespace /uav_0]

Runs until SIGINT/SIGTERM (or --duration), then writes one npz. Every sample
carries the RECORDER's receive time (one monotonic clock for all streams) and
the message's own header stamp, so the scorer can ask two different questions:

  * receive-time alignment -- "at the instant the controller could have this
    feedback sample, how far is it from where the vehicle really is?" -- which
    includes every transport and filter delay, i.e. what the loop actually sees;
  * header-stamp alignment -- the estimate's error at the time it CLAIMS to
    describe, i.e. the estimator's own accuracy with its latency removed.

Streams (all under the namespace):
  state/pose, state/twist_inertial   Isaac ground truth, published every physics
                                     step by Pegasus's ROS2Backend
  mocap                              the OptiTrack emulator's 250 Hz resampling of
                                     that truth -- the estimator's INPUT, and the
                                     raw-mocap stack's feedback
  state_estimator/local_position/odom  the controller's FEEDBACK (fused or raw,
                                     whichever stack is running)
  fmu/out/estimator_status_flags     EKF2 fusion flags (cs_ev_*, cs_yaw_align)
  fmu/out/timesync_status            the PX4<->ROS offset the fused bridge uses
  fsc_autopilot_ros2/whole_body_direct_actuation/mode   SAFETY / DIRECT
"""
import argparse
import os
import signal
import threading
import time

import numpy as np
import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from px4_msgs.msg import EstimatorStatusFlags, TimesyncStatus
from fsc_autopilot_ros2_msgs.msg import Mocap


def _stamp(h):
    return h.stamp.sec + 1e-9 * h.stamp.nanosec


class Recorder(Node):
    def __init__(self, ns):
        super().__init__("fused_feedback_recorder")
        self.t0 = time.monotonic()
        self.d = {k: [] for k in ("truth_pose", "truth_vel", "mocap", "odom",
                                  "flags", "timesync", "mode")}
        # ROS time of receipt as well, so stamps (ROS clock) and receipt can be
        # compared directly; monotonic is the analysis clock.
        sd = qos_profile_sensor_data
        self.create_subscription(PoseStamped, f"{ns}/state/pose", self.on_pose, sd)
        self.create_subscription(TwistStamped, f"{ns}/state/twist_inertial",
                                 self.on_vel, sd)
        self.create_subscription(Mocap, f"{ns}/mocap", self.on_mocap, sd)
        self.create_subscription(Odometry,
                                 f"{ns}/state_estimator/local_position/odom",
                                 self.on_odom, 50)
        self.create_subscription(EstimatorStatusFlags,
                                 f"{ns}/fmu/out/estimator_status_flags",
                                 self.on_flags, sd)
        self.create_subscription(TimesyncStatus, f"{ns}/fmu/out/timesync_status",
                                 self.on_ts, sd)
        self.create_subscription(
            String, f"{ns}/fsc_autopilot_ros2/whole_body_direct_actuation/mode",
            self.on_mode, 10)

    def now2(self):
        return time.monotonic() - self.t0, self.get_clock().now().nanoseconds * 1e-9

    def on_pose(self, m):
        t, r = self.now2()
        p, q = m.pose.position, m.pose.orientation
        self.d["truth_pose"].append((t, r, _stamp(m.header), p.x, p.y, p.z,
                                     q.w, q.x, q.y, q.z))

    def on_vel(self, m):
        t, r = self.now2()
        v = m.twist.linear
        self.d["truth_vel"].append((t, r, _stamp(m.header), v.x, v.y, v.z))

    def on_mocap(self, m):
        t, r = self.now2()
        p, v = m.pose.position, m.twist.linear
        self.d["mocap"].append((t, r, _stamp(m.header), p.x, p.y, p.z,
                                v.x, v.y, v.z))

    def on_odom(self, m):
        t, r = self.now2()
        p, q = m.pose.pose.position, m.pose.pose.orientation
        v = m.twist.twist.linear
        self.d["odom"].append((t, r, _stamp(m.header), p.x, p.y, p.z,
                               v.x, v.y, v.z, q.w, q.x, q.y, q.z))

    def on_flags(self, m):
        t, r = self.now2()
        self.d["flags"].append((t, r, float(m.cs_yaw_align), float(m.cs_ev_pos),
                                float(m.cs_ev_hgt), float(m.cs_ev_vel),
                                float(m.cs_ev_yaw), float(m.cs_ev_yaw_fault)))

    def on_ts(self, m):
        t, r = self.now2()
        self.d["timesync"].append((t, r, float(m.source_protocol),
                                   float(m.estimated_offset),
                                   float(m.observed_offset)))

    def on_mode(self, m):
        t, r = self.now2()
        code = {"SAFETY": 0.0, "DIRECT": 1.0}.get(m.data.strip().upper(), -1.0)
        if not self.d["mode"] or self.d["mode"][-1][2] != code:
            self.d["mode"].append((t, r, code))

    def save(self, path):
        os.makedirs(os.path.dirname(os.path.abspath(path)) or ".", exist_ok=True)
        np.savez_compressed(path, **{k: np.array(v, dtype=float)
                                     for k, v in self.d.items()})
        print("wrote", path, {k: len(v) for k, v in self.d.items()}, flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--namespace", default="/uav_0")
    ap.add_argument("--duration", type=float, default=0.0,
                    help="stop after this many seconds (0 = until signalled)")
    a = ap.parse_args()

    rclpy.init()
    node = Recorder(a.namespace)
    stop = {"flag": False}

    def _sig(*_):
        stop["flag"] = True
    signal.signal(signal.SIGINT, _sig)
    signal.signal(signal.SIGTERM, _sig)

    # ~1000 msg/s in total: spin in a thread (spin_once handles ONE callback
    # per call and would fall behind), keep the main thread for the signal.
    ex = rclpy.executors.SingleThreadedExecutor()
    ex.add_node(node)
    th = threading.Thread(target=ex.spin, daemon=True)
    th.start()
    t_end = time.monotonic() + a.duration if a.duration > 0 else None
    try:
        while not stop["flag"] and (t_end is None or time.monotonic() < t_end):
            time.sleep(0.1)
    finally:
        ex.shutdown(timeout_sec=1.0)
        node.save(a.out)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
