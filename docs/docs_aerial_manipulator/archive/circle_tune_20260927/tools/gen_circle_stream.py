#!/usr/bin/env python3
"""Record the whole-body planner's EE-CIRCLE reference stream offline (no Isaac,
no PX4): the built fsc_trajectory_planner node on the MIRROR yaml's planner
section, driven by a fake vehicle (the pattern of that package's
test_ee_trajectory_loopback.py): DIRECT hold -> select circle -> READY ->
go_to_start -> the rig teleports onto the start rest -> start -> record every
WholeBodyReference sample of the run.

    ROS_DOMAIN_ID=77 FASTRTPS_DEFAULT_PROFILES_FILE=<udp-only xml> \
      /usr/bin/python3 gen_circle_stream.py --radius 0.75 --out ../data/stream_r075.npz

Only ee_traj_circle_radius / ee_traj_lap_time are overridden (a temporary copy
of the yaml); every other planner key is the mirror's, i.e. the hardware's.
Saved in the wbref__* layout the offline benches read.
"""
import argparse
import math
import os
import re
import signal
import subprocess
import tempfile
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleAttitude
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray, String
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped
from fsc_autopilot_ros2_msgs.msg import WholeBodyReference

NS = "/uav_gen"
YAML = os.path.expanduser("~/ros2_ws/src/fsc_autopilot_ros2/config/"
                          "params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_sim.yaml")
HOME = np.array([0.0, 0.698132, 0.698132, 0.0])
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
V3 = ("x_cd", "x_cd_dot", "x_cd_ddot", "x_cd_d3", "x_cd_d4", "b1_d", "b1_d_dot", "b1_d_ddot",
      "r_ed", "r_ed_dot", "r_ed_ddot", "b1_de", "b1_de_dot", "b1_de_ddot")


class Rig(Node):
    def __init__(self, z0):
        super().__init__("circle_stream_rig", namespace=NS)
        self.mode = self.create_publisher(String, "fsc_autopilot_ros2/whole_body_direct_actuation/mode", LATCHED)
        self.odom = self.create_publisher(Odometry, "state_estimator/local_position/odom", 10)
        self.att = self.create_publisher(VehicleAttitude, "fmu/out/vehicle_attitude",
                                         QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                                                    durability=DurabilityPolicy.VOLATILE))
        self.js = self.create_publisher(JointState, "fsc_open_manipulator/joint_states", 10)
        self.select = self.create_publisher(String, "whole_body_planner/ee_trajectory/select", 10)
        self.scale = self.create_publisher(Float64, "whole_body_planner/ee_trajectory/time_scale", 10)
        self.status = self.ee_status = self.info = self.start_err = self.start_rest = None
        self.statuses = []
        self.refs = []
        self.create_subscription(String, "whole_body_planner/status", self._st, LATCHED)
        self.create_subscription(String, "whole_body_planner/ee_trajectory/status",
                                 lambda m: setattr(self, "ee_status", m.data), LATCHED)
        self.create_subscription(Float64MultiArray, "whole_body_planner/ee_trajectory/info",
                                 lambda m: setattr(self, "info", list(m.data)), LATCHED)
        self.create_subscription(Float64MultiArray, "whole_body_planner/ee_trajectory/start_error",
                                 lambda m: setattr(self, "start_err", list(m.data)), 10)
        self.create_subscription(PoseStamped, "whole_body_planner/ee_trajectory/start_rest",
                                 lambda m: setattr(self, "start_rest", m), LATCHED)
        self.create_subscription(WholeBodyReference, "fsc_autopilot_ros2/whole_body_direct_actuation/reference",
                                 lambda m: self.refs.append((time.monotonic(), m)), 200)
        self.go_cli = self.create_client(Trigger, "whole_body_planner/ee_trajectory/go_to_start")
        self.start_cli = self.create_client(Trigger, "whole_body_planner/ee_trajectory/start")
        self.odom_xyz = np.array([0.0, 0.0, z0]); self.yaw = 0.0; self.q = HOME.copy()
        self.create_timer(0.02, self.feed)

    def _st(self, m):
        self.status = m.data
        self.statuses.append((time.monotonic(), m.data))

    def feed(self):
        o = Odometry()
        o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = map(float, self.odom_xyz)
        o.pose.pose.orientation.w = 1.0
        self.odom.publish(o)
        a = VehicleAttitude(); ned = 0.5 * math.pi - self.yaw
        a.q = [math.cos(0.5 * ned), 0.0, 0.0, math.sin(0.5 * ned)]
        self.att.publish(a)
        j = JointState(); j.name = ["joint1", "joint2", "joint3", "joint4"]
        j.position = list(map(float, self.q)); j.velocity = [0.0] * 4
        self.js.publish(j)


def wait(cond, timeout, what):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if cond():
            return
        time.sleep(0.05)
    raise RuntimeError("timeout: " + what)


def call(cli, what):
    if not cli.wait_for_service(timeout_sec=10):
        raise RuntimeError("no service " + what)
    fut = cli.call_async(Trigger.Request()); t0 = time.monotonic()
    while not fut.done():
        time.sleep(0.05)
        if time.monotonic() - t0 > 10:
            raise RuntimeError("service timeout " + what)
    return fut.result()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--radius", type=float, default=0.5)
    ap.add_argument("--lap-time", type=float, default=None, help="default: the yaml's ee_traj_lap_time")
    ap.add_argument("--z", type=float, default=1.0)
    ap.add_argument("--yaml", default=YAML)
    ap.add_argument("--out", required=True)
    ap.add_argument("--set", action="append", default=[], help="planner key=value override (repeatable)")
    ap.add_argument("--ns", default="uav_gen")
    a = ap.parse_args()
    global NS
    NS = "/" + a.ns
    s = open(a.yaml).read()
    s = re.sub(r"^(\s*ee_traj_circle_radius:\s*)[0-9.]+", rf"\g<1>{a.radius:.4f}", s, flags=re.M)
    if a.lap_time:
        s = re.sub(r"^(\s*ee_traj_lap_time:\s*)[0-9.]+", rf"\g<1>{a.lap_time:.4f}", s, flags=re.M)
    for kv in a.set:
        k, v = kv.split("=")
        if v in ("true", "false"):
            s, n = re.subn(rf"^(\s*{k}:\s*)(true|false)", rf"\g<1>{v}", s, flags=re.M)
        else:
            s, n = re.subn(rf"^(\s*{k}:\s*)[-0-9.]+", rf"\g<1>{float(v):.4f}", s, flags=re.M)
        if n != 1:
            raise SystemExit(f"planner key {k} matched {n} lines")
    tmp = tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False); tmp.write(s); tmp.close()
    rclpy.init()
    rig = Rig(a.z)
    threading.Thread(target=rclpy.spin, args=(rig,), daemon=True).start()
    gov = subprocess.Popen(["ros2", "launch", "fsc_trajectory_planner", "whole_body_trajectory_planner_launch.py",
                            f"uav_prefix:={NS.lstrip('/')}", f"params_file:={tmp.name}"],
                           stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, start_new_session=True)
    try:
        wait(lambda: rig.status is not None, 30, "planner up")
        rig.mode.publish(String(data="DIRECT"))
        wait(lambda: rig.status == "HOLD", 20, "HOLD")
        time.sleep(1.0)
        rig.select.publish(String(data="circle"))
        t_sel = time.monotonic()
        while time.monotonic() - t_sel < 20 and not (rig.ee_status and rig.ee_status.startswith(("READY", "INFEASIBLE", "PLANNING", "CALC"))):
            if rig.ee_status == "NOT IN DIRECT":      # the mode raced the select: ask again
                time.sleep(0.5); rig.select.publish(String(data="circle"))
            time.sleep(0.2)
        try:
            wait(lambda: rig.ee_status and rig.ee_status.startswith(("READY", "INFEASIBLE")), 150, "READY")
        except RuntimeError:
            raise RuntimeError(f"timeout: READY (ee_status last = {rig.ee_status!r}, status = {rig.status!r})")
        if not rig.ee_status.startswith("READY"):
            raise RuntimeError("planner refused the circle: " + rig.ee_status)
        wait(lambda: rig.info is not None, 10, "info")
        s_now, s_max, T, T_lap = rig.info[:4]
        print(f"[gen] r={a.radius} {rig.ee_status} | s {s_now:.3f} s_max {s_max:.3f} T {T:.2f} s lap {T_lap:.2f} s")
        if s_now < 0.999:
            raise RuntimeError(f"the planner caps this circle at s = {s_now:.3f} < 1 (s_max {s_max:.3f})")
        r = call(rig.go_cli, "go_to_start")
        if not r.success:
            raise RuntimeError("go_to_start refused: " + r.message)
        wait(lambda: rig.status and rig.status.startswith("EXECUTING"), 30, "go-to-start EXECUTING")
        wait(lambda: rig.status == "HOLD", 120, "arrived")
        wait(lambda: rig.start_rest is not None, 5, "start_rest")
        m = rig.refs[-1][1]; sr = rig.start_rest.pose
        rig.odom_xyz = np.array([sr.position.x, sr.position.y, sr.position.z])
        rig.yaw = 2.0 * math.atan2(sr.orientation.z, sr.orientation.w); rig.q = np.array(m.q_d)
        wait(lambda: rig.start_err is not None and rig.start_err[3] == 1.0, 15, "at start")
        time.sleep(1.0)
        rig.refs.clear(); rig.statuses.clear()
        t_start = time.monotonic()
        r = call(rig.start_cli, "start")
        if not r.success:
            raise RuntimeError("start refused: " + r.message)
        wait(lambda: rig.status and rig.status.startswith("EXECUTING"), 5, "EXECUTING")
        wait(lambda: rig.status == "HOLD", rig.info[2] + 30, "run complete")
        time.sleep(1.0)
        refs = list(rig.refs)
        out = {"wbref__recv": np.array([t for t, _ in refs]) - t_start,
               "wbref__q_d": np.array([list(m.q_d) for _, m in refs]),
               "wbref__qdot_d": np.array([list(m.qdot_d) for _, m in refs])}
        for k in V3:
            arr = np.array([[getattr(getattr(m, k), c) for c in "xyz"] for _, m in refs])
            for i, c in enumerate("xyz"):
                out[f"wbref__{k}.{c}"] = arr[:, i]
        out["wb__recv"] = np.array([0.0])
        out["pl_status__recv"] = np.array([t for t, _ in rig.statuses]) - t_start
        out["pl_status__data"] = np.array([d for _, d in rig.statuses])
        out["meta"] = np.array([a.radius, T_lap, T, s_max])
        out["settings"] = np.array([f"radius={a.radius}"] + a.set)
        np.savez(a.out, **out)
        ee = np.column_stack([out["wbref__r_ed.x"], out["wbref__r_ed.y"]])
        v = np.linalg.norm(np.column_stack([out["wbref__r_ed_dot.x"], out["wbref__r_ed_dot.y"]]), axis=1)
        vc = np.linalg.norm(np.column_stack([out["wbref__x_cd_dot.x"], out["wbref__x_cd_dot.y"]]), axis=1)
        print(f"[gen] {len(refs)} samples, EE radius {np.hypot(ee[:, 0], ee[:, 1]).mean():.3f} m, "
              f"EE speed mean/peak {v[v > 0.02].mean():.3f}/{v.max():.3f} m/s, CoM speed peak {vc.max():.3f} m/s, "
              f"statuses {[d for _, d in rig.statuses][:4]}  -> {a.out}")
    finally:
        os.killpg(os.getpgid(gov.pid), signal.SIGINT)
        try:
            gov.communicate(timeout=10)
        except subprocess.TimeoutExpired:
            gov.kill()
        os.unlink(tmp.name)
        rig.destroy_node(); rclpy.try_shutdown()


if __name__ == "__main__":
    main()
