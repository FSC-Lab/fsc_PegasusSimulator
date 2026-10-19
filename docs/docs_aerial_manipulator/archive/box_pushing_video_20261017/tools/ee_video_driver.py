#!/usr/bin/env python3
"""The END-EFFECTOR TRAJECTORY mission (circle / figure-8) for the presentation video.
A copy of fsc_trajectory_planner/test/ee_trajectory_sim_driver.py -- the same
ground-station interface and the same gates -- with the fixed waits cut so no
phase idles on camera, an ABSOLUTE time scale, and wall-clock event times for
the video's cuts (one JSON line per event on stdout: {"t": unix, "ev": name}).

    offboard -> arm -> SAFETY climb to hover_z -> settle -> DIRECT -> settle
      -> select <shape> -> READY -> time scale -> go_to_start -> at start
      -> start -> run -> HOLD -> hold POST_HOLD s -> exit (vehicle left hovering
         in DIRECT; the recording ends there and the simulation is closed)
"""
import argparse
import json
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Float64MultiArray, String
from std_srvs.srv import SetBool, Trigger

from fsc_autopilot_ros2_msgs.msg import PositionControllerReference
from px4_msgs.msg import VehicleStatus

PX4_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE,
                     history=HistoryPolicy.KEEP_LAST, depth=10)
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
DA = "fsc_autopilot_ros2/whole_body_direct_actuation"
EE = "whole_body_planner/ee_trajectory"


class Driver(Node):
    def __init__(self, a):
        super().__init__("ee_video_driver")
        self.a = a
        ns = a.namespace.rstrip("/")
        self.phase, self.tp = "WAIT", time.time()
        self.done = self.aborted = False
        self.reason = ""
        self.odom = self.armed = self.info = self.start_err = None
        self.mode = self.plan_status = self.ee_status = ""
        self.settle_since = None
        self.ref = np.array([0.0, 0.0, a.hover_z])
        self.run_T = 0.0
        sub = self.create_subscription
        sub(Odometry, f"{ns}/state_estimator/local_position/odom", self.on_odom, 10)
        sub(String, f"{ns}/{DA}/mode", lambda m: setattr(self, "mode", m.data), LATCHED)
        sub(VehicleStatus, f"{ns}/fmu/out/vehicle_status_v1", lambda m: setattr(self, "armed", m.arming_state == 2), PX4_QOS)
        sub(String, f"{ns}/whole_body_planner/status", lambda m: setattr(self, "plan_status", m.data), LATCHED)
        sub(String, f"{ns}/{EE}/status", lambda m: setattr(self, "ee_status", m.data), LATCHED)
        sub(Float64MultiArray, f"{ns}/{EE}/info", lambda m: setattr(self, "info", list(m.data)), LATCHED)
        sub(Float64MultiArray, f"{ns}/{EE}/start_error", lambda m: setattr(self, "start_err", list(m.data)), 10)
        self.ref_pub = self.create_publisher(PositionControllerReference,
                                             f"{ns}/fsc_autopilot_ros2/position_controller/reference", 10)
        self.select_pub = self.create_publisher(String, f"{ns}/{EE}/select", 10)
        self.scale_pub = self.create_publisher(Float64, f"{ns}/{EE}/time_scale", 10)
        self.offboard = self.create_client(Trigger, f"{ns}/rc/offboard")
        self.arm = self.create_client(Trigger, f"{ns}/rc/arm")
        self.direct = self.create_client(SetBool, f"{ns}/{DA}/set_direct_mode")
        self.go = self.create_client(Trigger, f"{ns}/{EE}/go_to_start")
        self.start = self.create_client(Trigger, f"{ns}/{EE}/start")
        self.futs = []
        self.create_timer(1.0 / a.rate, self.tick)

    def on_odom(self, m):
        p, v = m.pose.pose.position, m.twist.twist.linear
        self.odom = np.array([p.x, p.y, p.z, v.x, v.y, v.z])

    def ev(self, event, **kw):
        print(json.dumps({"t": time.time(), "ev": event, **kw}), flush=True)

    def goto(self, ph):
        self.phase, self.tp, self.settle_since = ph, time.time(), None
        self.ev(ph, mode=self.mode, plan=self.plan_status, ee=self.ee_status)

    def call(self, cli, req, name):
        if not cli.service_is_ready():
            self.ev("service_not_ready", name=name)
            return
        fut = cli.call_async(req)
        fut.add_done_callback(lambda f: self.ev("service", name=name, result=str(f.result())[:160]))
        self.futs.append(fut)

    def send_ref(self):
        m = PositionControllerReference()
        m.header.stamp = self.get_clock().now().to_msg()
        m.position.x, m.position.y, m.position.z = map(float, self.ref)
        m.yaw, m.yaw_unit = 0.0, PositionControllerReference.DEGREES
        self.ref_pub.publish(m)

    def settled(self, tol_m=0.08, tol_v=0.10, hold=2.0):
        if self.odom is None:
            return False
        if np.linalg.norm(self.odom[:3] - self.ref) < tol_m and np.linalg.norm(self.odom[3:6]) < tol_v:
            self.settle_since = self.settle_since or time.time()
            return time.time() - self.settle_since > hold
        self.settle_since = None
        return False

    def abort(self, why):
        self.aborted, self.reason, self.done = True, why, True
        self.ev("ABORT", reason=why)
        self.call(self.direct, SetBool.Request(data=False), "set_direct_mode(false)")

    def tick(self):
        a, el = self.a, time.time() - self.tp
        if self.mode != "DIRECT":
            self.send_ref()   # SAFETY needs the full-rate stream
        if self.phase == "WAIT":
            if self.odom is not None and self.mode and self.armed is not None:
                if self.armed:
                    self.abort("vehicle already ARMED -- clean relaunch needed")
                    return
                self.ref = self.odom[:3].copy()
                self.goto("OFFBOARD")
        elif self.phase == "OFFBOARD":
            if el > 1.0:
                self.call(self.offboard, Trigger.Request(), "rc/offboard")
                self.goto("ARM")
        elif self.phase == "ARM":
            if el > 3.0:
                self.call(self.arm, Trigger.Request(), "rc/arm")
                self.goto("CLIMB")
        elif self.phase == "CLIMB":
            if el > 1.0:
                self.ref = np.array([self.odom[0], self.odom[1], a.hover_z])
                self.goto("TAKEOFF")
        elif self.phase == "TAKEOFF":
            if self.settled():
                self.call(self.direct, SetBool.Request(data=True), "set_direct_mode(true)")
                self.goto("CONFIRM")
            elif el > 60.0:
                self.abort("takeoff did not settle")
        elif self.phase == "CONFIRM":
            if self.mode == "DIRECT":
                self.goto("DIRECT")
            elif el > 5.0:
                self.abort("DIRECT was refused")
        elif self.phase == "DIRECT":
            if el > a.direct_settle:
                self.select_pub.publish(String(data=a.shape))
                self.goto("SELECT")
        elif self.phase == "SELECT":
            if self.ee_status.startswith("READY") and self.info:
                s = min(a.time_scale, self.info[1])
                self.ev("READY", s_max=self.info[1], s=s)
                self.scale_pub.publish(Float64(data=s))
                self.ee_status = "PENDING_RESCALE"
                self.s_req = s
                self.goto("RESCALE")
            elif self.ee_status.startswith("INFEASIBLE") or el > 60.0:
                self.abort(f"trajectory not READY: {self.ee_status}")
        elif self.phase == "RESCALE":
            if self.ee_status.startswith("READY") and self.info and abs(self.info[0] - self.s_req) < 1e-3:
                self.run_T = self.info[2]
                self.ev("RESCALED", status=self.ee_status, T_total=self.run_T, T_lap=self.info[3])
                self.call(self.go, Trigger.Request(), "ee_trajectory/go_to_start")
                self.goto("GO_TO_START")
            elif el > 60.0:
                self.abort(f"rescale did not settle: {self.ee_status}")
        elif self.phase == "GO_TO_START":
            at = el > 2.0 and self.plan_status == "HOLD" and self.start_err and self.start_err[3] > 0.5
            if at:
                self.at_since = getattr(self, "at_since", None) or time.time()
                if time.time() - self.at_since > a.start_dwell:
                    self.call(self.start, Trigger.Request(), "ee_trajectory/start")
                    self.goto("START")
            else:
                self.at_since = None
                if el > 90.0:
                    self.abort(f"never at start: plan={self.plan_status} err={self.start_err}")
        elif self.phase == "START":
            if self.plan_status.startswith("EXECUTING"):
                self.goto("RUN")
            elif el > 10.0:
                self.abort(f"start did not begin: plan={self.plan_status}")
        elif self.phase == "RUN":
            if self.plan_status == "HOLD" and el > 5.0:
                self.goto("RUN_COMPLETE")
            elif el > self.run_T + 60.0:
                self.abort("run overran")
        elif self.phase == "RUN_COMPLETE":
            if el > a.post_hold:
                self.goto("END")
                self.done = True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--namespace", default="/uav_0")
    ap.add_argument("--rate", type=float, default=50.0)
    ap.add_argument("--hover-z", type=float, default=1.0)
    ap.add_argument("--shape", default="circle", choices=["circle", "figure8"])
    ap.add_argument("--time-scale", type=float, default=1.0, help="absolute, capped at the planner's s_max")
    ap.add_argument("--direct-settle", type=float, default=6.0)
    ap.add_argument("--start-dwell", type=float, default=1.0, help="at the start pose this long before Start [s]")
    ap.add_argument("--post-hold", type=float, default=4.0)
    a = ap.parse_args()
    rclpy.init()
    d = Driver(a)
    try:
        while rclpy.ok() and not d.done:
            rclpy.spin_once(d, timeout_sec=0.05)
    except KeyboardInterrupt:
        d.ev("interrupted")
    finally:
        d.ev("EXIT", aborted=d.aborted, reason=d.reason)
        d.destroy_node()
        rclpy.shutdown()
    return 1 if d.aborted else 0


if __name__ == "__main__":
    raise SystemExit(main())
