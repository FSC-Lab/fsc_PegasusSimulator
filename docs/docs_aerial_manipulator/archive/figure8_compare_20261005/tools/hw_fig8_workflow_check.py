#!/usr/bin/env python3
"""hw_fig8_workflow_check.py -- the HARDWARE figure-8 workflow, end to end, without a
vehicle (2026-10-05, before the first figure-8 flights).

What runs, all real code except the vehicle:
  * the trajectory planner, launched with the SAME arguments as the hardware stack
    (--rig wb: start_whole_body_l1_4d_..._fused.sh; --rig decoupled:
    start_geometric_l1_..._fused.sh, i.e. the geometric mode topic, the position-mode
    arm reference topic and the 0.30 m Start gate), reading the HARDWARE 4-D yaml;
  * for --rig decoupled, the decoupled reference bridge, with the stack's arguments;
  * the arm GS's EE Trajectory panel (tools/gs_harness, offscreen), which reads its
    Figure-8 fields once the planner's parameters load, selects Figure-8 and presses
    Go To Start / Start Trajectory / Back To Origin / Start Transition only when the
    panel enables them;
  * a fake vehicle: DIRECT, hovering at --hover (base x y z, yaw deg), which after the
    go-to-start transition is moved onto the start rest (the planner's own pose).
Everything sits under /uav_hwcheck; run it on a private domain:

    source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
    ROS_DOMAIN_ID=77 /usr/bin/python3 hw_fig8_workflow_check.py --rig wb --harness <bin>

Checks: the GS's Figure-8 defaults (A 0.70, B 0.35, 0.10 m/s, 1 lap) and what it
writes to the planner (lap 42.68 s, 1 lap); READY at s = 1; the streamed run (long
axis on world x: EE extents 2A x 2B, the crossing at the EE's hover point, joint
ranges, start = end, smooth); the go-to-start yaw turn <= 45 deg; the footprint
(EE + CoM reference) relative to the hover point; Back To Origin completes; and for
the decoupled wiring the bridge's same-motion residual and the arm reference stream.
"""
import argparse
import math
import os
import signal
import subprocess
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleAttitude
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from trajectory_msgs.msg import JointTrajectory
from fsc_autopilot_ros2_msgs.msg import WholeBodyReference

NS = "/uav_hwcheck"
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
HOME = np.array([0.0, 0.698132, 0.698132, 0.0])       # model frame
SIGN = np.array([-1.0, 1.0, 1.0, -1.0])                # hardware arm_joint_sign
BASE_COM = "[0.0,-0.017854,0.0]"
FSC = os.path.expanduser("~/ros2_ws/src/fsc_autopilot_ros2")
YAML = FSC + "/config/params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650.yaml"
BRIDGE = FSC + "/scripts/tools/decoupled_reference_bridge.py"
RIGS = {
    "wb": dict(mode="fsc_autopilot_ros2/whole_body_direct_actuation/mode",
               arm_ref="fsc_open_manipulator/external_torque_controller/reference_joint_trajectory",
               extra=[]),
    "decoupled": dict(mode="fsc_autopilot_ros2/geometric_l1_direct_actuation/mode",
                      arm_ref="fsc_open_manipulator/position_controller/reference_joint_trajectory",
                      extra=["mode_topic:=fsc_autopilot_ros2/geometric_l1_direct_actuation/mode",
                             "arm_reference_topic:=fsc_open_manipulator/position_controller/reference_joint_trajectory",
                             "ee_traj_start_pos_tol:=0.30"]),
}


def figure8_length(a, b, n=2000):
    th = np.linspace(0.0, 2.0 * math.pi, n + 1)
    f = np.hypot(a * np.cos(th), 2.0 * b * np.cos(2.0 * th))
    w = np.ones(n + 1); w[1:-1:2] = 4.0; w[2:-1:2] = 2.0
    return float((2.0 * math.pi / n) / 3.0 * np.sum(w * f))


class Rig(Node):
    def __init__(self, rig, hover):
        super().__init__("hw_fig8_rig", namespace=NS)
        self.mode = self.create_publisher(String, RIGS[rig]["mode"], LATCHED)
        self.odom = self.create_publisher(Odometry, "state_estimator/local_position/odom", 10)
        self.att = self.create_publisher(VehicleAttitude, "fmu/out/vehicle_attitude",
                                         QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                                                    durability=DurabilityPolicy.VOLATILE))
        self.js = self.create_publisher(JointState, "fsc_open_manipulator/joint_states", 10)
        self.status, self.ee_status, self.info, self.start_rest = None, None, None, None
        self.lock = threading.Lock()
        self.refs, self.arm_refs, self.bridge = [], [], []
        self.status_log = []
        self.create_subscription(String, "whole_body_planner/status", self.on_status, LATCHED)
        self.create_subscription(String, "whole_body_planner/ee_trajectory/status",
                                 lambda m: setattr(self, "ee_status", m.data), LATCHED)
        self.create_subscription(Float64MultiArray, "whole_body_planner/ee_trajectory/info",
                                 lambda m: setattr(self, "info", list(m.data)), LATCHED)
        self.create_subscription(PoseStamped, "whole_body_planner/ee_trajectory/start_rest",
                                 lambda m: setattr(self, "start_rest", m), LATCHED)
        self.create_subscription(WholeBodyReference, "fsc_autopilot_ros2/whole_body_direct_actuation/reference",
                                 self.on_ref, 50)
        self.create_subscription(JointTrajectory, RIGS[rig]["arm_ref"], self.on_arm_ref, 50)
        self.create_subscription(Float64MultiArray, "decoupled_bridge/base_reference_vector",
                                 lambda m: self.bridge.append((self.status, list(m.data))), 50)
        self.getp = self.create_client(GetParameters, NS + "/whole_body_trajectory_planner/get_parameters")
        self.xyz = np.array(hover[:3], float)
        self.yaw = math.radians(hover[3])
        self.q = HOME.copy()
        self.create_timer(0.02, self.feed)

    def on_status(self, m):
        self.status = m.data
        self.status_log.append((time.monotonic(), m.data))

    def on_ref(self, m):
        with self.lock:
            self.refs.append((time.monotonic(), self.status, m))

    def on_arm_ref(self, m):
        if m.points:
            with self.lock:
                self.arm_refs.append((self.status, list(m.points[0].positions)))

    def feed(self):
        o = Odometry()
        o.header.stamp = self.get_clock().now().to_msg()
        o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = map(float, self.xyz)
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(self.yaw / 2), math.cos(self.yaw / 2)
        self.odom.publish(o)
        a = VehicleAttitude()
        ned = 0.5 * math.pi - self.yaw
        a.q = [math.cos(0.5 * ned), 0.0, 0.0, math.sin(0.5 * ned)]
        self.att.publish(a)
        j = JointState()
        j.header.stamp = o.header.stamp
        j.name = [f"joint{i + 1}" for i in range(4)]
        j.position = list(map(float, SIGN * self.q))
        j.velocity = [0.0] * 4
        self.js.publish(j)


def wait(cond, timeout, what):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if cond():
            return
        time.sleep(0.05)
    raise AssertionError("timeout: " + what)


def yaw_of(q):
    return 2.0 * math.atan2(q.z, q.w)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rig", choices=sorted(RIGS), default="wb")
    ap.add_argument("--yaml", default=YAML)
    ap.add_argument("--harness", required=True, help="gs_fig8_harness binary")
    ap.add_argument("--png", default="/tmp/gs_fig8", help="screenshot prefix")
    ap.add_argument("--hover", nargs=4, type=float, default=[0.0, 0.0, 1.2, 0.0],
                    metavar=("X", "Y", "Z", "YAW_DEG"))
    a = ap.parse_args()
    results = []

    def check(ok, text):
        results.append(bool(ok))
        print(("  OK    " if ok else "  FAIL  ") + text, flush=True)

    rclpy.init()
    rig = Rig(a.rig, a.hover)
    threading.Thread(target=rclpy.spin, args=(rig,), daemon=True).start()
    procs = [subprocess.Popen(
        ["ros2", "launch", "fsc_trajectory_planner", "whole_body_trajectory_planner_launch.py",
         f"uav_prefix:={NS.lstrip('/')}", f"params_file:={a.yaml}", f"base_com:={BASE_COM}",
         "arm_joint_sign:=[-1.0,1.0,1.0,-1.0]", "hold_ee_world:=false"] + RIGS[a.rig]["extra"],
        stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT, start_new_session=True)]
    if a.rig == "decoupled":
        procs.append(subprocess.Popen(
            ["/usr/bin/python3", BRIDGE, "--namespace", NS, "--da", "geometric_l1_direct_actuation",
             "--base-com", BASE_COM, "--controller-topic", "fsc_autopilot_ros2/position_controller/reference_direct"],
            stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT, start_new_session=True))
    gs = None
    try:
        wait(lambda: rig.status is not None, 30, "planner up")
        rig.mode.publish(String(data="DIRECT"))
        wait(lambda: rig.status == "HOLD", 20, "HOLD")
        time.sleep(1.0)
        with rig.lock:
            hold = rig.refs[-1][2]
        ee_hover = np.array([hold.r_ed.x, hold.r_ed.y, hold.r_ed.z])
        print(f"[{a.rig}] DIRECT hold: base {a.hover[:3]} yaw {a.hover[3]:.0f} deg, EE (hover) "
              f"{np.round(ee_hover, 3).tolist()}", flush=True)

        env = dict(os.environ, QT_QPA_PLATFORM="offscreen")
        gs = subprocess.Popen([a.harness, NS + "/fsc_open_manipulator", f"{a.png}_{a.rig}"], env=env,
                              stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, start_new_session=True)
        gsval = {}
        presses = []

        def read_gs():
            for line in gs.stdout:
                if line.startswith("GSVAL "):
                    _, k, v = line.rstrip("\n").split(" ", 2)
                    if k == "press":
                        presses.append(v)
                    gsval[k] = v
        threading.Thread(target=read_gs, daemon=True).start()

        # the go-to-start transition: EXECUTING, then HOLD -> the vehicle is moved onto the start rest
        wait(lambda: "selected" in gsval, 30, "GS selected Figure-8")
        print("1. arm GS Figure-8 page as it opens on the hardware yaml:", flush=True)
        L = figure8_length(0.70, 0.35)
        check(abs(float(gsval["fig8_a"]) - 0.70) < 1e-9 and abs(float(gsval["fig8_b"]) - 0.35) < 1e-9,
              f"Half-Length / Half-Width {float(gsval['fig8_a']):.2f} / {float(gsval['fig8_b']):.2f} m (0.70 / 0.35)")
        check(abs(float(gsval["fig8_vel"]) - 0.10) < 1e-9, f"Mean Velocity {float(gsval['fig8_vel']):.3f} m/s (0.100)")
        check(int(float(gsval["fig8_laps"])) == 1, f"Laps {int(float(gsval['fig8_laps']))} (1)")
        check(abs(float(gsval["circle_vel"]) - 0.13) < 5e-4 and int(float(gsval["circle_laps"])) == 1,
              f"circle page unchanged: r {float(gsval['circle_radius']):.2f} m, {float(gsval['circle_vel']):.3f} m/s, "
              f"{int(float(gsval['circle_laps']))} lap")
        wait(lambda: "ee_status" in gsval, 60, "READY")
        r = rig.getp.call(GetParameters.Request(names=["ee_traj_fig8_a", "ee_traj_fig8_b", "ee_traj_lap_time",
                                                      "ee_traj_laps", "ee_traj_q2_cycles_per_lap"]))
        vals = [v.integer_value if v.type == ParameterType.PARAMETER_INTEGER else v.double_value for v in r.values]
        print("2. what the GS wrote to the planner, and the plan:", flush=True)
        check(abs(vals[2] - L / 0.10) < 0.01 and vals[3] == 1 and abs(vals[0] - 0.70) < 1e-9,
              f"ee_traj_lap_time {vals[2]:.3f} s (= {L:.4f} m / 0.10 m/s), laps {vals[3]}, "
              f"q2 period {vals[2] / vals[4]:.3f} s ({vals[4]} per lap)")
        inf = rig.info
        check(gsval["ee_status"].startswith("READY") and abs(inf[0] - 1.0) < 1e-9,
              f"{gsval['ee_status']} | s {inf[0]:.2f} (max {inf[1]:.2f}), run {inf[2]:.1f} s, lap {inf[3]:.1f} s, "
              f"peak |v| {inf[9]:.3f} m/s |a| {inf[10]:.3f} m/s^2 |qdot| {math.degrees(inf[11]):.1f} deg/s, "
              f"tau_j {inf[12]:.2f} N.m, sigma_nd {inf[13]:.3f}")

        wait(lambda: "Go To Start" in presses, 30, "GS pressed Go To Start")
        wait(lambda: rig.status and rig.status.startswith("EXECUTING"), 30, "go-to-start EXECUTING")
        wait(lambda: rig.status == "HOLD", 90, "go-to-start HOLD")
        wait(lambda: rig.start_rest is not None, 5, "start_rest")
        with rig.lock:
            q_start = np.array(rig.refs[-1][2].q_d)
        sr = rig.start_rest.pose
        rig.xyz = np.array([sr.position.x, sr.position.y, sr.position.z])
        turn = math.degrees(math.remainder(yaw_of(sr.orientation) - math.radians(a.hover[3]), 2 * math.pi))
        rig.yaw = yaw_of(sr.orientation)
        rig.q = q_start
        print("3. go-to-start (GS button), vehicle moved onto the start rest:", flush=True)
        check(abs(turn) <= 45.0 + 1e-6, f"yaw turn {turn:+.1f} deg (<= 45), start rest base "
              f"{np.round(rig.xyz, 3).tolist()} yaw {math.degrees(rig.yaw):.1f} deg")

        wait(lambda: "Start Trajectory" in presses, 30, "GS pressed Start Trajectory")
        t_start = time.monotonic()
        wait(lambda: rig.status and rig.status.startswith("EXECUTING"), 10, "run EXECUTING")
        wait(lambda: gsval.get("run") == "complete", inf[2] + 40, "run complete")
        t_end = time.monotonic()
        with rig.lock:
            run = [m for (t, s, m) in rig.refs if t_start <= t <= t_end and s and s.startswith("EXECUTING")]
            arm = [p for (s, p) in rig.arm_refs if s and s.startswith("EXECUTING")]
        ee = np.array([[m.r_ed.x, m.r_ed.y, m.r_ed.z] for m in run])
        com = np.array([[m.x_cd.x, m.x_cd.y, m.x_cd.z] for m in run])
        q = np.degrees(np.array([m.q_d for m in run]))
        ext = np.ptp(ee, axis=0)
        ctr = 0.5 * (ee.min(axis=0) + ee.max(axis=0))
        print(f"4. the run ({len(run)} samples over {t_end - t_start:.1f} s; streamed reference, world frame):", flush=True)
        check(abs(ext[0] - 1.40) < 0.01 and abs(ext[1] - 0.70) < 0.01 and ext[2] < 0.002,
              f"EE extents x {ext[0]:.3f} / y {ext[1]:.3f} / z {ext[2]*1e3:.1f} mm  (2A 1.40 along world x, 2B 0.70)")
        check(np.linalg.norm(ctr[:2] - ee_hover[:2]) < 0.01,
              f"8 centred at {np.round(ctr[:2], 3).tolist()}, the EE hover point {np.round(ee_hover[:2], 3).tolist()} "
              f"(off by {np.linalg.norm(ctr[:2] - ee_hover[:2])*1e3:.1f} mm)")
        step = np.linalg.norm(np.diff(ee, axis=0), axis=1).max()
        check(step < 0.01 and np.linalg.norm(ee[-1] - ee[0]) < 5e-3 and np.abs(q[-1] - q[0]).max() < 0.5,
              f"smooth (max EE step {step*1e3:.2f} mm) and ends on its start (EE {np.linalg.norm(ee[-1]-ee[0])*1e3:.2f} mm, "
              f"joints {np.abs(q[-1]-q[0]).max():.2f} deg)")
        check(np.abs(q[:, 0]).max() < 0.5 and 9.5 < q[:, 1].min() and q[:, 1].max() < 40.5 and q[:, 2].max() < 50.0,
              f"joints: q1 {q[:,0].min():+.1f}..{q[:,0].max():+.1f}, q2 {q[:,1].min():.1f}..{q[:,1].max():.1f} "
              f"(25 +- 15), q3 {q[:,2].min():.1f}..{q[:,2].max():.1f} (< 50 stop), q4 {q[:,3].min():+.1f}..{q[:,3].max():+.1f} deg")
        box = np.array([min(ee[:, 0].min(), com[:, 0].min()), max(ee[:, 0].max(), com[:, 0].max()),
                        min(ee[:, 1].min(), com[:, 1].min()), max(ee[:, 1].max(), com[:, 1].max())])
        print(f"      planned footprint (EE + CoM reference): x {box[0]:+.2f}..{box[1]:+.2f}, y {box[2]:+.2f}..{box[3]:+.2f} m "
              f"= {box[1]-box[0]:.2f} x {box[3]-box[2]:.2f} m, centre ({0.5*(box[0]+box[1]):+.2f}, {0.5*(box[2]+box[3]):+.2f}) "
              f"vs the base hover ({a.hover[0]:+.2f}, {a.hover[1]:+.2f})", flush=True)
        if arm:
            aq = np.degrees(np.array(arm) * SIGN)
            check(len(arm) > 0.5 * len(run) and abs(aq[:, 1].min() - q[:, 1].min()) < 0.5 and abs(aq[:, 1].max() - q[:, 1].max()) < 0.5,
                  f"arm reference stream {RIGS[a.rig]['arm_ref'].split('/')[1]}: {len(arm)} samples, q2 "
                  f"{aq[:,1].min():.1f}..{aq[:,1].max():.1f} deg (hardware sign applied)")
        else:
            check(False, f"arm reference stream {RIGS[a.rig]['arm_ref']}: no samples")
        if a.rig == "decoupled":
            res = np.array([v[23] for (s, v) in rig.bridge if s and s.startswith("EXECUTING") and len(v) > 23])
            check(res.size > 0 and res.max() < 2e-3,
                  f"bridge same-motion residual max {res.max()*1e3:.3f} mm over {res.size} samples (< 2 mm)"
                  if res.size else "bridge: no samples in the run")

        wait(lambda: gsval.get("DONE") == "ok" or "FAIL" in gsval, 120, "Back To Origin")
        print("5. Back To Origin (GS button) -> PLANNED -> Start Transition (GS button):", flush=True)
        check(gsval.get("DONE") == "ok", "flown, back to HOLD" if gsval.get("DONE") == "ok" else gsval.get("FAIL", "?"))
        print(f"   GS presses, in order: {presses}; screenshots {a.png}_{a.rig}_ready.png / _done.png", flush=True)
    except AssertionError as e:
        check(False, str(e))
    finally:
        for p in ([gs] if gs else []) + procs:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGINT)
            except ProcessLookupError:
                pass
        time.sleep(2)
        rclpy.try_shutdown()
    ok = all(results)
    print(f"=== {a.rig}: {'ALL PASS' if ok else 'SOME CHECKS FAILED'} ({sum(results)}/{len(results)}) ===")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
