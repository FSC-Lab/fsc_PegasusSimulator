#!/usr/bin/env python3
"""hw_planner_check.py -- load the HARDWARE 4-D yaml into a real planner on a fake hover
rig (no Isaac, no PX4) and confirm what the drone will plan (2026-10-05).

    source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
    ROS_DOMAIN_ID=77 /usr/bin/python3 hw_planner_check.py [params.yaml]

The planner is launched the way the hardware stacks launch it (base_com
[0,-0.017854,0], arm_joint_sign [-1,1,1,-1]) under /uav_hwcheck, so nothing on the
default domain or namespace is touched. Checks, in order:
  1. the new keys reached the node (ee_traj_q2_cycles_per_lap, ee_traj_a/w_max, fig8_a/b);
  2. the circle at the yaml's own lap plans as before (s = 1 feasible);
  3. the figure-8 at the lap the arm GS writes for Mean Velocity 0.20 (22.8646 s) plans
     at s = 1 -- the q2 period follows the lap through ee_traj_q2_cycles_per_lap;
  4. control: with ee_traj_q2_cycles_per_lap 0 the same figure-8 is REFUSED (the circle's
     6.04 s q2 period does not divide the 22.865 s lap), proving the knob is what makes it fly.
"""
import math
import os
import signal
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleAttitude
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray, String

NS = "/uav_hwcheck"
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
HOME = [0.0, 0.698132, 0.698132, 0.0]
SIGN = [-1.0, 1.0, 1.0, -1.0]
YAW_DEG = 45.0          # takeoff heading for the figure-8 (long axis on world x)
INFO = ["s", "s_max", "T_total", "T_lap", "laps", "ramp", "type_id", "ee_pos_err_m", "ee_rot_err_deg",
        "peak_v", "peak_a", "peak_qdot", "peak_tau_j", "min_sigma_nd"]
HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_YAML = os.path.expanduser("~/ros2_ws/src/fsc_autopilot_ros2/config/"
                                  "params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650.yaml")


class Rig(Node):
    def __init__(self):
        super().__init__("hw_planner_check_rig", namespace=NS)
        self.mode = self.create_publisher(String, "fsc_autopilot_ros2/whole_body_direct_actuation/mode", LATCHED)
        self.odom = self.create_publisher(Odometry, "state_estimator/local_position/odom", 10)
        self.att = self.create_publisher(VehicleAttitude, "fmu/out/vehicle_attitude",
                                         QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                                                    durability=DurabilityPolicy.VOLATILE))
        self.js = self.create_publisher(JointState, "fsc_open_manipulator/joint_states", 10)
        self.sel = self.create_publisher(String, "whole_body_planner/ee_trajectory/select", 10)
        self.scale = self.create_publisher(Float64, "whole_body_planner/ee_trajectory/time_scale", 10)
        self.status, self.info = None, None
        self.create_subscription(String, "whole_body_planner/ee_trajectory/status",
                                 lambda m: setattr(self, "status", m.data), LATCHED)
        self.create_subscription(Float64MultiArray, "whole_body_planner/ee_trajectory/info",
                                 lambda m: setattr(self, "info", list(m.data)), LATCHED)
        self.setp = self.create_client(SetParameters, NS + "/whole_body_trajectory_planner/set_parameters")
        self.getp = self.create_client(GetParameters, NS + "/whole_body_trajectory_planner/get_parameters")
        self.create_timer(0.02, self.tick)

    def tick(self):
        o = Odometry()
        o.header.stamp = self.get_clock().now().to_msg()
        o.pose.pose.position.z = 1.2
        y = math.radians(YAW_DEG)
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(y / 2), math.cos(y / 2)
        self.odom.publish(o)
        a = VehicleAttitude()
        ned = math.pi / 2 - y                      # PX4 NED yaw of an ENU yaw y
        a.q = [math.cos(0.5 * ned), 0.0, 0.0, math.sin(0.5 * ned)]
        self.att.publish(a)
        j = JointState()
        j.header.stamp = self.get_clock().now().to_msg()
        j.name = [f"joint{i + 1}" for i in range(4)]
        j.position = [s * q for s, q in zip(SIGN, HOME)]
        j.velocity = [0.0] * 4
        self.js.publish(j)


def call(rig, cli, req, what):
    assert cli.wait_for_service(timeout_sec=20), what + " service missing"
    f = cli.call_async(req)
    t0 = time.time()
    while not f.done() and time.time() - t0 < 20:
        time.sleep(0.05)
    return f.result()


def setp(rig, **kv):
    req = SetParameters.Request()
    for k, v in kv.items():
        p = Parameter(); p.name = k
        p.value = (ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=v) if isinstance(v, int)
                   else ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(v)))
        req.parameters.append(p)
    r = call(rig, rig.setp, req, "set_parameters")
    assert all(x.successful for x in r.results), [x.reason for x in r.results]


def plan(rig, shape, scale=1.0):
    rig.status, rig.info = None, None
    rig.sel.publish(String(data=shape))
    t0 = time.time()
    while time.time() - t0 < 60:
        s = rig.status or ""
        if s.startswith("READY") or "INFEASIBLE" in s:
            break
        time.sleep(0.1)
    if not (rig.status or "").startswith("READY"):
        return rig.status, None
    rig.info = None
    rig.scale.publish(Float64(data=scale))
    t0 = time.time()
    while time.time() - t0 < 30 and (rig.info is None or abs(rig.info[0] - min(scale, rig.info[1])) > 1e-6):
        time.sleep(0.1)
    return rig.status, dict(zip(INFO, rig.info)) if rig.info else None


def main():
    yaml = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_YAML
    rclpy.init()
    rig = Rig()
    threading.Thread(target=rclpy.spin, args=(rig,), daemon=True).start()
    gov = subprocess.Popen(["ros2", "launch", "fsc_trajectory_planner", "whole_body_trajectory_planner_launch.py",
                            f"uav_prefix:={NS.lstrip('/')}", f"params_file:={yaml}",
                            "base_com:=[0.0,-0.017854,0.0]", "arm_joint_sign:=[-1.0,1.0,1.0,-1.0]"],
                           stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT, start_new_session=True)
    ok = True
    try:
        time.sleep(6)
        rig.mode.publish(String(data="DIRECT"))
        time.sleep(3)
        keys = ["ee_traj_fig8_a", "ee_traj_fig8_b", "ee_traj_lap_time", "ee_traj_q2_period_s",
                "ee_traj_q2_cycles_per_lap", "ee_traj_v_max", "ee_traj_a_max", "ee_traj_w_max",
                "v_max", "a_max", "w_max", "ee_traj_fold_deg", "ee_traj_q2_center_deg", "ee_traj_q2_amp_deg"]
        r = call(rig, rig.getp, GetParameters.Request(names=keys), "get_parameters")
        print("1. parameters on the node (" + os.path.basename(yaml) + "):")
        for k, v in zip(keys, r.values):
            val = v.integer_value if v.type == ParameterType.PARAMETER_INTEGER else v.double_value
            print(f"     {k:28s} {val}")
        st, inf = plan(rig, "circle")
        good = inf is not None and abs(inf["s"] - 1.0) < 1e-6
        ok &= good
        print(f"2. circle at the yaml lap: {'OK ' if good else 'FAIL'} {st}"
              + (f" | s {inf['s']:.2f} (max {inf['s_max']:.3f}), T {inf['T_total']:.2f} s, lap {inf['T_lap']:.2f} s,"
                 f" peak |v| {inf['peak_v']:.3f} |a| {inf['peak_a']:.3f}" if inf else ""))
        setp(rig, ee_traj_fig8_a=0.75, ee_traj_fig8_b=0.375, ee_traj_lap_time=22.8646, ee_traj_laps=1)
        st, inf = plan(rig, "figure8")
        good = inf is not None and abs(inf["s"] - 1.0) < 1e-6
        ok &= good
        print(f"3. figure-8 A 0.75 / B 0.375, lap 22.8646 s (Mean Velocity 0.20): {'OK ' if good else 'FAIL'} {st}"
              + (f" | s {inf['s']:.2f} (max {inf['s_max']:.3f}), T {inf['T_total']:.2f} s, peak |v| {inf['peak_v']:.3f}"
                 f" |a| {inf['peak_a']:.3f} |qdot| {math.degrees(inf['peak_qdot']):.1f} deg/s, tau_j {inf['peak_tau_j']:.2f},"
                 f" sigma_nd {inf['min_sigma_nd']:.3f}" if inf else ""))
        setp(rig, ee_traj_q2_cycles_per_lap=0)
        st, inf = plan(rig, "figure8")
        good = inf is None
        ok &= good
        print(f"4. control, ee_traj_q2_cycles_per_lap 0 (period stays 6.0415 s): {'OK ' if good else 'FAIL'} {st}")
    finally:
        os.killpg(os.getpgid(gov.pid), signal.SIGINT)
        time.sleep(2)
    rclpy.shutdown()
    print("ALL PASS" if ok else "SOME CHECKS FAILED")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
