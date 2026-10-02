#!/usr/bin/env python3
"""Bring an Isaac AM-T650 rig to DIRECT for a PS4 teleoperation session -- and
land it afterwards (2026-09-27; the decoupled rig since 2026-10-01).

    /usr/bin/python3 ps4_teleop_bringup.py up     # offboard -> arm -> climb -> DIRECT
    /usr/bin/python3 ps4_teleop_bringup.py land   # SAFETY -> descend -> disarm

Works on every rig whose stack runs the whole-body trajectory planner, i.e.
whose DIRECT reference the pad can own: the whole-body 4-D rig (and the
modular one, which answers in the same namespace) and the DECOUPLED
geometric+L1 rig. --da names the controller's direct-actuation namespace (its
/mode topic and set_direct_mode service); the default `auto` takes whichever of
the known ones is publishing a mode, and refuses when none or more than one is.

`up` streams the SAFETY position reference (full rate, the controller needs
it) at the vehicle's own x/y, climbs to --hover-z, waits for it to settle,
enters whole-body DIRECT and EXITS -- in DIRECT the whole-body trajectory
planner owns the reference (HOLD, then the pad once the PS4 Remote tab is
engaged), and a second publisher on the drone-GS topic would only be queued
as a pending target. `land` reverts to SAFETY (the node re-seeds its hold on
the current pose), descends by reference at 0.2 m/s to --land-z and disarms.

Run it on the UDP-only DDS profile on shiqi-desktop (a shell-started client
can miss the ROS nodes' topics over shared memory):
    FASTRTPS_DEFAULT_PROFILES_FILE=.../fastdds_udp_only.xml /usr/bin/python3 ...
Refuses `up` against an already-armed vehicle (clean relaunch first).
"""
import argparse
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from fsc_autopilot_ros2_msgs.msg import PositionControllerReference
from px4_msgs.msg import VehicleStatus

PX4_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE,
                     history=HistoryPolicy.KEEP_LAST, depth=10)
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
# Direct-actuation namespaces whose stacks run the trajectory planner (the
# modular rig answers under the whole-body one on purpose).
KNOWN_DA = ("whole_body_direct_actuation", "geometric_l1_direct_actuation")


def detect_da(namespace, timeout=20.0):
    """The one KNOWN_DA whose /mode topic has a publisher (the controller latches
    it at startup). None when zero or several do."""
    ns = namespace.rstrip("/")
    probe = Node("ps4_teleop_bringup_probe")
    found = []
    t_end = time.time() + timeout
    try:
        while time.time() < t_end:
            found = [da for da in KNOWN_DA
                     if probe.count_publishers(f"{ns}/fsc_autopilot_ros2/{da}/mode") > 0]
            if found:
                # a moment longer, so a second stack is not missed by discovery order
                rclpy.spin_once(probe, timeout_sec=1.0)
                found = [da for da in KNOWN_DA
                         if probe.count_publishers(f"{ns}/fsc_autopilot_ros2/{da}/mode") > 0]
                break
            rclpy.spin_once(probe, timeout_sec=0.5)
    finally:
        probe.destroy_node()
    if len(found) != 1:
        print(f"--da auto: {len(found)} controller mode topics found under {ns} "
              f"({', '.join(found) or 'none'}) -- pass --da explicitly", flush=True)
        return None
    print(f"--da auto: {found[0]}", flush=True)
    return found[0]


class Bringup(Node):
    def __init__(self, a, da):
        super().__init__("ps4_teleop_bringup")
        self.a = a
        ns = a.namespace.rstrip("/")
        DA = f"fsc_autopilot_ros2/{da}"
        self.da = da
        self.odom = None
        self.mode = ""
        self.armed = None
        self.plan = ""
        self.ref = None
        self.create_subscription(Odometry, f"{ns}/state_estimator/local_position/odom", self.on_odom, 10)
        self.create_subscription(String, f"{ns}/{DA}/mode", lambda m: setattr(self, "mode", m.data), LATCHED)
        self.create_subscription(VehicleStatus, f"{ns}/fmu/out/vehicle_status_v1",
                                 lambda m: setattr(self, "armed", m.arming_state == 2), PX4_QOS)
        self.create_subscription(String, f"{ns}/whole_body_planner/status",
                                 lambda m: setattr(self, "plan", m.data), LATCHED)
        self.ref_pub = self.create_publisher(
            PositionControllerReference, f"{ns}/fsc_autopilot_ros2/position_controller/reference", 10)
        self.offboard = self.create_client(Trigger, f"{ns}/rc/offboard")
        self.arm = self.create_client(Trigger, f"{ns}/rc/arm")
        self.disarm = self.create_client(Trigger, f"{ns}/rc/disarm")
        self.direct = self.create_client(SetBool, f"{ns}/{DA}/set_direct_mode")
        self.clear = self.create_client(Trigger, f"{ns}/whole_body_planner/clear")
        self.t0 = time.time()

    def on_odom(self, m):
        p, v = m.pose.pose.position, m.twist.twist.linear
        self.odom = np.array([p.x, p.y, p.z, v.x, v.y, v.z])

    def log(self, s):
        print(f"[{time.time() - self.t0:6.1f}s] {s}", flush=True)

    def spin_for(self, seconds, stream=False):
        t_end = time.time() + seconds
        while time.time() < t_end:
            if stream:
                self.send_ref()
            rclpy.spin_once(self, timeout_sec=1.0 / self.a.rate)

    def send_ref(self):
        if self.ref is None:
            return
        m = PositionControllerReference()
        m.header.stamp = self.get_clock().now().to_msg()
        m.position.x, m.position.y, m.position.z = map(float, self.ref)
        m.yaw, m.yaw_unit = 0.0, PositionControllerReference.DEGREES
        self.ref_pub.publish(m)

    def call(self, cli, req, name, timeout=10.0):
        if not cli.wait_for_service(timeout_sec=timeout):
            self.log(f"service {name} not available")
            return None
        f = cli.call_async(req)
        t_end = time.time() + timeout
        while not f.done() and time.time() < t_end:
            self.send_ref()
            rclpy.spin_once(self, timeout_sec=0.02)
        r = f.result()
        self.log(f"{name} -> {getattr(r, 'success', None)} {getattr(r, 'message', '')}")
        return r

    def wait(self, cond, timeout, what, stream=True):
        t_end = time.time() + timeout
        while time.time() < t_end:
            if cond():
                return True
            if stream:
                self.send_ref()
            rclpy.spin_once(self, timeout_sec=1.0 / self.a.rate)
        self.log(f"TIMEOUT waiting for {what}")
        return False

    def settled(self, since, tol_m=0.08, tol_v=0.10, hold=3.0):
        if self.odom is None:
            return None
        ok = (np.linalg.norm(self.odom[:3] - self.ref) < tol_m and
              np.linalg.norm(self.odom[3:6]) < tol_v)
        if not ok:
            return None
        return since if since is not None else time.time()

    def up(self):
        a = self.a
        if not self.wait(lambda: self.odom is not None and self.armed is not None and self.mode,
                         60, "odometry, PX4 status and the controller mode", stream=False):
            return 1
        if self.armed:
            self.log("vehicle already ARMED -- refusing (clean relaunch needed)")
            return 1
        self.ref = self.odom[:3].copy()
        self.spin_for(1.0, stream=True)
        self.call(self.offboard, Trigger.Request(), "rc/offboard")
        self.spin_for(3.0, stream=True)
        self.call(self.arm, Trigger.Request(), "rc/arm")
        self.spin_for(2.0, stream=True)
        self.ref = np.array([self.odom[0], self.odom[1], a.hover_z])
        self.log(f"climbing to z = {a.hover_z:.2f} m (SAFETY)")
        since = None
        t_end = time.time() + 90.0
        while time.time() < t_end:
            self.send_ref()
            rclpy.spin_once(self, timeout_sec=1.0 / a.rate)
            since = self.settled(since)
            if since is not None and time.time() - since > 3.0:
                break
        else:
            self.log("takeoff did not settle -- staying in SAFETY")
            return 1
        self.log(f"hovering at {np.round(self.odom[:3], 3)}; settling {a.settle:.0f} s")
        self.spin_for(a.settle, stream=True)
        r = self.call(self.direct, SetBool.Request(data=True), "set_direct_mode(true)")
        if not self.wait(lambda: self.mode == "DIRECT", 10, "DIRECT"):
            self.log(f"DIRECT refused ({getattr(r, 'message', '')}) -- still in SAFETY, hovering")
            return 1
        # The SAFETY stream overlaps the switch by a tick or two, and the
        # planner captures a drone-GS setpoint seen in DIRECT as a PENDING
        # target (never executed). Clear it so the session starts from HOLD.
        self.ref = None
        self.spin_for(1.0)
        if not self.plan.startswith("HOLD"):
            self.call(self.clear, Trigger.Request(), "whole_body_planner/clear")
        self.wait(lambda: self.plan.startswith("HOLD"), 10, "planner HOLD", stream=False)
        # The SAFETY -> DIRECT handover has its own transient (the DIRECT law's
        # observer starts from zero: 0.2-0.4 m and a few degrees on the
        # geometric+L1 rig, measured 2026-10-01). Do not hand the vehicle to the
        # pad in the middle of it: report ready once the airframe is still again.
        t_sw, still = time.time(), None
        while time.time() - t_sw < a.settle_direct:
            rclpy.spin_once(self, timeout_sec=1.0 / a.rate)
            if self.odom is None:
                continue
            slow = np.linalg.norm(self.odom[3:6]) < 0.10   # above the mirror plant's mocap velocity noise
            still = (still or time.time()) if slow else None
            if still is not None and time.time() - still > 2.0:
                break
        self.log(f"DIRECT-entry transient {'settled' if still else 'NOT settled'} after "
                 f"{time.time() - t_sw:.1f} s (|v| {np.linalg.norm(self.odom[3:6]):.3f} m/s)")
        self.log(f"{self.da} DIRECT, planner {self.plan} -- ready for the PS4 Remote tab")
        return 0

    def land(self):
        a = self.a
        if not self.wait(lambda: self.odom is not None and self.mode, 20, "odometry and mode", stream=False):
            return 1
        if self.mode == "DIRECT":
            self.call(self.direct, SetBool.Request(data=False), "set_direct_mode(false)")
            self.wait(lambda: self.mode != "DIRECT", 10, "SAFETY", stream=False)
        self.ref = self.odom[:3].copy()
        self.spin_for(3.0, stream=True)
        self.log(f"descending from z = {self.ref[2]:.2f} to {a.land_z:.2f} m")
        while self.ref[2] > a.land_z + 1e-6:
            self.ref[2] = max(a.land_z, self.ref[2] - 0.2 / a.rate)
            self.send_ref()
            rclpy.spin_once(self, timeout_sec=1.0 / a.rate)
        self.spin_for(a.land_wait, stream=True)
        self.call(self.disarm, Trigger.Request(), "rc/disarm")
        self.spin_for(2.0, stream=True)
        self.log("done (PX4 may deny the disarm on this rig -- 'not landed'; it is harmless on the ground)")
        return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("action", choices=["up", "land"])
    ap.add_argument("--namespace", default="/uav_0")
    ap.add_argument("--da", default="auto", choices=("auto",) + KNOWN_DA,
                    help="the controller's direct-actuation namespace (auto = the one publishing a mode)")
    ap.add_argument("--rate", type=float, default=50.0)
    ap.add_argument("--hover-z", type=float, default=1.2)
    ap.add_argument("--settle", type=float, default=8.0)
    ap.add_argument("--settle-direct", type=float, default=20.0,
                    help="max seconds to wait for the DIRECT-entry transient to settle before 'ready'")
    ap.add_argument("--land-z", type=float, default=0.35)
    ap.add_argument("--land-wait", type=float, default=15.0)
    a = ap.parse_args()
    rclpy.init()
    da = detect_da(a.namespace) if a.da == "auto" else a.da
    if da is None:
        rclpy.shutdown()
        return 1
    node = Bringup(a, da)
    try:
        rc = node.up() if a.action == "up" else node.land()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
