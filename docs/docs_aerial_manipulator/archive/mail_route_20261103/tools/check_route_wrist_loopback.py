#!/usr/bin/env python3
"""Does the MAIL ROUTE command the wrist roll at every letter's pick? (2026-11-03, after the
first hardware route: "the claw never turned, even at the hold above the letter".)

    ROS_DOMAIN_ID=93 /usr/bin/python3 check_route_wrist_loopback.py [YAML]

The route loopback rig (fsc_trajectory_planner/test/test_mail_route_loopback.py: a perfect
plant, the frame Got on each hat and box) against the planner loaded with the EXPERIMENT route
yaml (default ..._t650_mail_route.yaml), the hardware base_com. For each letter: select it, Go
To Pick Start, Ready To Pick to the hold above the letter, and read the streamed whole-body
reference's q_d and the arm reference there; then ABORT (nothing gripped) and the next letter.
Every hold must carry the pick pose the 10-31 single-envelope flights flew, [0, 36, 44, 90] deg
(the arm reference at arm_joint_sign +1 here; the hardware's [-1, 1, 1, -1] flips q1 and q4 only).
Starts a planner: never beside an Isaac campaign.
"""
import math
import os
import signal
import subprocess
import sys
import tempfile
import threading
import time

import numpy as np

TEST = os.path.expanduser("~/Workspaces/fsc_autopilot_ws/src/fsc_trajectory_planner/test")
sys.path.insert(0, TEST)
import rclpy  # noqa: E402
from trajectory_msgs.msg import JointTrajectory  # noqa: E402
import test_mail_route_loopback as M  # noqa: E402

L = M.L
YAML = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser(
    "~/Workspaces/fsc_autopilot_ws/src/fsc_autopilot_ros2/config/"
    "params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_mail_route.yaml")
FLOWN = np.array([0.0, 36.0, 44.0, 90.0])
GOV_CMD = ["ros2", "run", "fsc_trajectory_planner", "whole_body_trajectory_planner",
           "--ros-args", "-r", f"__ns:={L.NS}", "--params-file", YAML,
           "-p", f"pick_place_pick_topic:={L.OBJ_TOPIC}", "-p", f"pick_place_land_topic:={L.LAND_TOPIC}",
           "-p", "pick_place_descent_trim:=false", "-p", "base_com:=[0.0,-0.017854,0.0]", "-p", "hold_ee_world:=false",
           # the rig's room (hats along x at y 0, boxes at x 1.709): its fence and take-off spot
           "-p", "pick_place_fence_min_z:=0.30", "-p", "pick_place_fence_max_z:=2.2",
           "-p", "pick_place_start:=[0.70,-0.70,1.10,90.0]",
           "-p", "pick_place_land_start:=[-0.5,-1.0,1.0,0.0]", "-p", "pick_place_land:=[-0.5,-1.0,0.8,0.0]"]


def main():
    rclpy.init()
    rig = M.RouteRig()
    arm = []
    rig.create_subscription(JointTrajectory, "fsc_open_manipulator/external_torque_controller/reference_joint_trajectory",
                            lambda m: arm.append(list(m.points[0].positions[:4])) if m.points else None, 50)
    threading.Thread(target=rclpy.spin, args=(rig,), daemon=True).start()
    log = tempfile.NamedTemporaryFile("w+", prefix="route_wrist_node_", suffix=".log", delete=False)
    gov = subprocess.Popen(GOV_CMD, stdout=log, stderr=subprocess.STDOUT, text=True, start_new_session=True)
    rows, ok = [], True
    try:
        L.wait(lambda: rig.pp_status is not None and rig.info is not None, 20, "planner up")
        print(f"planner up on {os.path.basename(YAML)}: pick pose {L.get_param(rig, 'pick_place_pick_pose_deg')}")
        for k in range(3):
            rig.put_frame(*M.hat_frame(k))
            r = L.call(rig, f"capture_letter_{k + 1}")
            assert r.success, r.message
        for b, name in enumerate(["RED", "BLUE"]):
            rig.put_frame(*M.box_frame(name))
            r = L.call(rig, f"get_box_{b + 1}")
            assert r.success, r.message
        rig.put_frame(np.array([5.0, 5.0, 0.0]), 0.0)
        rig.odom_xyz = M.HOVER.copy()
        rig.yaw = 0.0
        time.sleep(0.5)
        rig.mode.publish(M.String(data="DIRECT"))
        L.wait(lambda: rig.status == "HOLD", 10, "planner HOLD")
        assert L.call(rig, "route_plan").success
        L.wait(lambda: rig.pp_status.startswith("READY"), 60, "route READY")
        print(f"route planned: {rig.pp_status}")
        for k in range(3):
            if k > 0:
                r = L.call(rig, f"select_letter_{k + 1}")
                assert r.success, r.message
            q_start = None
            L.fly(rig, 0)                                   # Go To Pick Start (the rig onto it)
            q_start = np.degrees(rig.refs[-1][1].q_d)
            M.fly_claw(rig, 1, above=lambda rg: None)       # Ready To Pick: the hold above the letter
            time.sleep(0.3)
            q = np.degrees(np.array(rig.refs[-1][1].q_d))
            qa = np.degrees(np.array(arm[-1])) if arm else np.full(4, np.nan)
            good = np.allclose(q, FLOWN, atol=0.05) and np.allclose(qa, FLOWN, atol=0.05)
            ok &= good
            rows.append((k + 1, q_start, q, qa, good, rig.pp_status))
            print(f"letter {k + 1}: Pick Start q_d {np.round(q_start, 2)} -> above the letter q_d {np.round(q, 2)}, "
                  f"arm reference {np.round(qa, 2)}  {'OK' if good else 'WRONG'}  | {rig.pp_status[:90]}")
            r = L.call(rig, "abort")
            assert r.success, r.message
            L.wait(lambda: rig.pp_status.startswith("ABORTED") and rig.status == "HOLD", 30, "aborted")
            base, yaw, _ = rig.goal(1)
            rig.at(np.array(base) - M.PICK_BACK * M.nose(yaw) + np.array([0.0, 0.0, 0.30]), yaw)
    finally:
        # the whole group: `ros2 run` does not forward a signal to the node it started
        os.killpg(gov.pid, signal.SIGINT)
        try:
            gov.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(gov.pid, signal.SIGKILL)
    print("PASS: every letter's pick hold carries the flown pick pose (wrist 90 deg)" if ok else "FAIL", flush=True)
    os._exit(0 if ok else 1)   # not sys.exit: the daemon spin thread aborts rclpy's teardown


if __name__ == "__main__":
    main()
