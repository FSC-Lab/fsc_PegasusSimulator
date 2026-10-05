#!/usr/bin/env python3
"""Fly the PUSH-AND-PULL mission in 07's Isaac scene (07_px4_direct_t650_
aerial_manipulator_push_and_pull.py) with the whole-body 4-D L1 rig -- the arm
GS's Push & Pull tab, scripted:

    FASTRTPS_DEFAULT_PROFILES_FILE=<udp-only xml> /usr/bin/python3 pl_mission.py --out run.npz

  on the ground: Reset, Adjust, Get (the box, obj_0)
  offboard -> arm -> SAFETY climb to hover_z -> settle -> DIRECT -> settle -> Plan
  1 Go To Start      the claw 0.20 m above the fin handle, yaw -90
  2 Ready To Push    OPEN the gripper, hover (the descent trim's window), ready:
                     straight down onto the handle
  3 Push             once the claw is within 2 cm of the grasp point and 3 mm
                     across the fin: CLOSE through the GripperCommand action (the
                     GS's path); on the stall, push: CONTACT on, 1.5 s settle,
                     0.50 m along -y, arm fixed
  4 Exit To Push     OPEN, wait for the jaws, exit: CONTACT off, up 0.20 m, arm home
  5 Go To Land       270 deg clockwise to the hover over the landing spot
  -> SAFETY -> land by reference -> disarm

Records the box (ground truth = obj_0's state, the same pose the mocap
carries), the odometry, the measured claw (claw_0), the streamed reference,
the planner's arrival error and info, the gripper, the joints and the
whole-body node's debug array, tagged by step, for pl_score.py. Refuses to
start against an armed vehicle (a clean relaunch is needed between flights).
"""
import argparse
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, Float64, Float64MultiArray, String
from std_srvs.srv import SetBool, Trigger

from fsc_autopilot_ros2_msgs.msg import PositionControllerReference, WholeBodyReference
from px4_msgs.msg import VehicleStatus

PX4_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE,
                     history=HistoryPolicy.KEEP_LAST, depth=10)
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
DA = "fsc_autopilot_ros2/whole_body_direct_actuation"
REF_TOPIC = DA + "/reference"
PL = "whole_body_planner/push_pull"
STEPS = ["go_to_start", "ready", "close", "push", "release", "exit", "go_to_land"]
GRIP_CLOSE, GRIP_OPEN = -0.8727, 0.0           # rad (06's gripper bus, +-50 deg)


class Abort(Exception):
    pass


class Driver(Node):
    def __init__(self, a):
        super().__init__("pl_mission")
        self.a = a
        ns = a.namespace.rstrip("/")
        self.t0 = time.time()
        self.events, self.marks = [], []
        self.odom = self.armed = None
        self.mode = ""
        self.status = self.pl_status = ""
        self.info = self.arrival = self.grip = self.box = None
        self.step = -1
        self.phase = "init"
        self.ref = np.array([0.0, 0.0, a.hover_z])
        self.stream_ref = True
        self.grip_cmd = None
        self.L = {k: [] for k in ("odom", "dbg", "box", "ee", "ref", "grip", "joints", "arrival", "claw")}
        sub = self.create_subscription
        sub(Odometry, f"{ns}/state_estimator/local_position/odom", self.on_odom, 10)
        sub(String, f"{ns}/{DA}/mode", lambda m: setattr(self, "mode", m.data), LATCHED)
        sub(Float32MultiArray, f"{ns}/{DA}/wb_control_debug", self.on_dbg, 50)
        sub(VehicleStatus, f"{ns}/fmu/out/vehicle_status_v1",
            lambda m: setattr(self, "armed", m.arming_state == 2), PX4_QOS)
        sub(String, f"{ns}/whole_body_planner/status", self.on_status, LATCHED)
        sub(String, f"{ns}/{PL}/status", self.on_pl_status, LATCHED)
        sub(Float64MultiArray, f"{ns}/{PL}/info", lambda m: setattr(self, "info", np.asarray(m.data)), LATCHED)
        sub(Float64MultiArray, f"{ns}/{PL}/arrival_error", self.on_arrival, 10)
        sub(PoseStamped, f"{ns}/whole_body_planner/current_ee", self.on_ee, 10)
        sub(WholeBodyReference, f"{ns}/{REF_TOPIC}", self.on_ref, 50)
        sub(PoseStamped, "/obj_0/state/pose", self.on_box, qos_profile_sensor_data)
        sub(JointState, f"{ns}/isaacsim_manipulator/gripper_state", self.on_grip, qos_profile_sensor_data)
        sub(PoseStamped, "/claw_0/state/pose", self.on_claw, qos_profile_sensor_data)
        sub(JointState, f"{ns}/isaacsim_manipulator/joint_states", self.on_joints, qos_profile_sensor_data)
        self.ref_pub = self.create_publisher(PositionControllerReference,
                                             f"{ns}/fsc_autopilot_ros2/position_controller/reference", 10)
        self.grip_pub = self.create_publisher(Float64, f"{ns}/isaacsim_manipulator/gripper_command", 10)
        from rclpy.action import ActionClient
        from control_msgs.action import GripperCommand
        self.grip_ac = ActionClient(self, GripperCommand, a.grip_action)
        self.cli = {n: self.create_client(Trigger, f"{ns}/rc/{n}") for n in ("offboard", "arm", "disarm")}
        for n in ("go_to_start", "ready", "push", "exit", "go_to_land", "adjust", "plan", "capture_box",
                  "reset", "abort"):
            self.cli[n] = self.create_client(Trigger, f"{ns}/{PL}/{n}")
        self.direct = self.create_client(SetBool, f"{ns}/{DA}/set_direct_mode")
        self.create_timer(1.0 / a.rate, self.tick)

    # ---- callbacks ---------------------------------------------------------
    def now(self):
        return time.time() - self.t0

    def on_odom(self, m):
        p, v, q = m.pose.pose.position, m.twist.twist.linear, m.pose.pose.orientation
        self.odom = np.array([p.x, p.y, p.z, v.x, v.y, v.z])
        self.L["odom"].append([self.now(), p.x, p.y, p.z, v.x, v.y, v.z, q.w, q.x, q.y, q.z,
                               1.0 if self.mode == "DIRECT" else 0.0, self.step])

    def on_dbg(self, m):
        if self.mode == "DIRECT":
            self.L["dbg"].append(np.concatenate([[self.now(), self.step], np.asarray(m.data, np.float32)[:120]]))

    def on_status(self, m):
        if m.data != self.status:
            self.ev(f"planner: {m.data}")
        self.status = m.data

    def on_pl_status(self, m):
        if m.data != self.pl_status:
            self.ev(f"push_pull: {m.data}")
            # the arm GS's rule: a planner ABORT (button or SAFETY GUARD) opens
            # the jaws at once -- the planner holds still for the release
            if m.data.startswith("ABORTING") and not self.pl_status.startswith("ABORTING"):
                self.grip_cmd = GRIP_OPEN
                self.ev("gripper OPEN (the planner is aborting)")
        self.pl_status = m.data

    def on_arrival(self, m):
        self.arrival = list(m.data)
        self.L["arrival"].append([self.now(), *self.arrival[:11], self.step])

    def on_ee(self, m):
        p, q = m.pose.position, m.pose.orientation
        self.L["ee"].append([self.now(), p.x, p.y, p.z, q.w, q.x, q.y, q.z, self.step])

    def on_ref(self, m):
        r = m.r_ed
        self.L["ref"].append([self.now(), r.x, r.y, r.z, m.x_cd.x, m.x_cd.y, m.x_cd.z,
                              *list(m.q_d)[:4], m.b1_d.x, m.b1_d.y, self.step])

    def on_box(self, m):
        p, q = m.pose.position, m.pose.orientation
        self.box = np.array([p.x, p.y, p.z, q.w, q.x, q.y, q.z])
        self.L["box"].append([self.now(), *self.box, self.step])

    def on_claw(self, m):
        p = m.pose.position
        self.L["claw"].append([self.now(), p.x, p.y, p.z, self.step])

    def on_grip(self, m):
        if m.position:
            self.grip = float(m.position[0])
            eff = float(m.effort[0]) if m.effort else float("nan")
            self.L["grip"].append([self.now(), self.grip, eff, self.step])

    def on_joints(self, m):
        if len(m.position) >= 4:
            eff = list(m.effort[:4]) if len(m.effort) >= 4 else [np.nan] * 4
            self.L["joints"].append([self.now(), *m.position[:4], *eff, self.step])
            self.joint_names = list(m.name)

    def tick(self):
        if self.stream_ref and self.mode != "DIRECT":
            m = PositionControllerReference()
            m.header.stamp = self.get_clock().now().to_msg()
            m.position.x, m.position.y, m.position.z = map(float, self.ref)
            m.yaw, m.yaw_unit = 0.0, PositionControllerReference.DEGREES
            self.ref_pub.publish(m)
        if self.grip_cmd is not None:
            self.grip_pub.publish(Float64(data=float(self.grip_cmd)))

    # ---- helpers -----------------------------------------------------------
    def ev(self, s):
        line = f"[{self.now():7.2f}s] {s}"
        self.events.append(line)
        print(line, flush=True)

    def mark(self, name):
        self.marks.append((self.now(), name))
        self.ev(f"MARK {name}")

    def guard(self, what):
        if self.armed and self.mode != "DIRECT" and self.phase == "mission":
            raise Abort(f"left DIRECT while {what}")
        if self.phase == "mission" and self.pl_status.startswith(("ABORTING", "ABORTED")):
            raise Abort(f"push-and-pull ABORTED while {what}: {self.pl_status}")

    def wait(self, cond, timeout, what):
        t = time.time()
        while time.time() - t < timeout:
            if cond():
                return time.time() - t
            self.guard(what)
            time.sleep(0.05)
        raise Abort(f"timeout ({timeout:.0f} s): {what}  [mode={self.mode} plan={self.status} "
                    f"push_pull={self.pl_status}]")

    def call(self, cli, req, name, must=True):
        if not cli.wait_for_service(timeout_sec=10.0):
            raise Abort(f"service {name} not available")
        fut = cli.call_async(req)
        self.wait(lambda: fut.done(), 15, f"{name} response")
        r = fut.result()
        self.ev(f"{name}: success={r.success} {r.message}")
        if must and not r.success:
            raise Abort(f"{name} refused: {r.message}")
        return r

    def trig(self, name, must=True):
        return self.call(self.cli[name], Trigger.Request(), name, must)

    def settled(self, tol_m=0.08, tol_v=0.10, hold=3.0, timeout=60.0):
        since, t = None, time.time()
        while time.time() - t < timeout:
            if self.odom is not None:
                ok = (np.linalg.norm(self.odom[:3] - self.ref) < tol_m and np.linalg.norm(self.odom[3:6]) < tol_v)
                since = (since or time.time()) if ok else None
                if since and time.time() - since > hold:
                    return
            time.sleep(0.05)
        raise Abort("hover did not settle")

    def begin(self, s):
        self.step = STEPS.index(s)
        self.mark(f"{s}:start")

    def start(self, name):
        """Call a step, retrying while the planner says the vehicle / claw is
        still settling (what the operator does: wait, press again)."""
        t = time.time()
        while True:
            r = self.trig(name, must=False)
            if r.success:
                return
            if "wait for it to settle" not in r.message or time.time() - t > self.a.settle_timeout:
                raise Abort(f"{name} refused: {r.message}")
            time.sleep(0.5)

    def step_to_end(self, name, k, at_handle=None):
        self.start(name)
        self.wait(lambda: self.status.startswith("EXECUTING"), 20, f"{name} EXECUTING")
        T = float(self.status.split("T=")[1].split("s")[0]) if "T=" in self.status else 60.0
        self.ev(f"{name}: executing, T = {T:.1f} s")
        self.wait(lambda: self.status == "HOLD" and self.info is not None and int(self.info[5]) == k
                  and (at_handle is None or (self.info[53] > 0.5) == at_handle), T + 40, f"{name} complete")

    def gripper_open(self, what):
        self.mark(f"gripper_{what}:start")
        self.grip_cmd = GRIP_OPEN
        t = time.time()
        while time.time() - t < 3.0 and (self.grip is None or abs(self.grip - GRIP_OPEN) > np.radians(3.0)):
            time.sleep(0.02)
        self.ev(f"gripper {what}: at {np.degrees(self.grip or np.nan):.1f} deg")
        self.mark(f"gripper_{what}:end")

    def gripper_close(self):
        """The arm GS's path: a GripperCommand CLOSE goal, ABORTED + stalled once
        the fingers stop on the fin."""
        from control_msgs.action import GripperCommand
        self.mark("gripper_close:start")
        self.grip_cmd = None                      # the action server drives the bus now
        goal = GripperCommand.Goal()
        goal.command.position = self.a.grip_closed_m
        goal.command.max_effort = 50.0
        if not self.grip_ac.wait_for_server(timeout_sec=5.0):
            raise Abort(f"no gripper action server at {self.a.grip_action}")
        fut = self.grip_ac.send_goal_async(goal)
        t = time.time()
        while not fut.done() and time.time() - t < 5.0:
            time.sleep(0.01)
        gh = fut.result() if fut.done() else None
        if gh is None or not gh.accepted:
            raise Abort("gripper CLOSE goal not accepted")
        rf = gh.get_result_async()
        while not rf.done() and time.time() - t < 12.0:
            time.sleep(0.01)
        if not rf.done():
            raise Abort("gripper CLOSE: no result within 12 s")
        res = rf.result().result
        self.grip_cmd = GRIP_CLOSE                # hold the close on the bus
        self.ev(f"gripper close (action): stalled={res.stalled} reached={res.reached_goal} "
                f"position {res.position:.4f} m, at {np.degrees(self.grip or np.nan):.1f} deg")
        self.mark("gripper_close:end")
        if not res.stalled:
            raise Abort("the gripper closed on NOTHING -- not pushing")

    # ---- the mission -------------------------------------------------------
    def mission(self):
        a = self.a
        self.wait(lambda: self.odom is not None and self.mode and self.armed is not None
                  and self.box is not None and self.pl_status, 90, "vehicle + box + planner feeds")
        if self.armed:
            raise Abort("vehicle already ARMED -- refusing (clean relaunch needed)")
        self.box0 = self.box.copy()
        self.ev(f"box at rest {np.round(self.box0[:3], 4).tolist()}")
        self.trig("reset", must=False)
        self.trig("adjust")
        time.sleep(1.0)
        self.trig("capture_box")
        self.ref = self.odom[:3].copy()
        self.grip_cmd = GRIP_OPEN
        time.sleep(1.0)
        self.trig("offboard")
        time.sleep(3.0)
        self.trig("arm")
        time.sleep(2.0)
        self.ref = np.array([self.odom[0], self.odom[1], a.hover_z])
        self.ev(f"climbing to {a.hover_z} m")
        self.settled()
        self.call(self.direct, SetBool.Request(data=True), "set_direct_mode(true)")
        self.wait(lambda: self.mode == "DIRECT", 5, "DIRECT")
        self.phase = "mission"
        self.mark("direct")
        self.stream_ref = False
        time.sleep(a.direct_settle)
        self.wait(lambda: self.status == "HOLD", 15, "planner HOLD")
        self.trig("plan")
        self.wait(lambda: self.pl_status.startswith("READY"), 60, "mission READY")
        self.ev(f"planned step durations {np.round(self.info[8:13], 1).tolist()} s")
        # 1 Go To Start
        self.begin("go_to_start")
        self.step_to_end("go_to_start", 0, at_handle=False)
        self.mark("go_to_start:end")
        # 2 Ready To Push: open, hover (the trim window), then down onto the fin
        self.begin("ready")
        self.gripper_open("open")
        time.sleep(a.hover_dwell)
        self.step_to_end("ready", 1, at_handle=True)
        moved = np.linalg.norm(self.box[:3] - self.box0[:3])
        if moved > a.max_disturb:
            raise Abort(f"the box was disturbed {moved * 1e3:.1f} mm by the descent -- not closing")
        self.mark("ready:end")
        # 3 Push: close once the claw is there, push on the stall
        self.begin("close")
        t, since = time.time(), None
        while True:
            ok = (self.arrival is not None and len(self.arrival) >= 10 and int(self.arrival[0]) == 1
                  and self.arrival[1] > 0.5 and self.arrival[2] <= a.grip_tol
                  and abs(self.arrival[9]) <= a.grip_axis_tol)
            since = (since or time.time()) if ok else None
            if since and time.time() - since >= a.grip_dwell_in:
                break
            if time.time() - t > a.grip_timeout:
                raise Abort(f"the claw never came within {a.grip_tol * 1e3:.0f} mm / "
                            f"{a.grip_axis_tol * 1e3:.0f} mm across the fin (last "
                            f"{self.arrival[2] * 1e3:.0f} / {self.arrival[9] * 1e3:.1f} mm)")
            self.guard("close")
            time.sleep(0.02)
        self.ev(f"close: claw {self.arrival[2] * 1e3:.1f} mm from the grasp point, "
                f"{self.arrival[9] * 1e3:+.1f} mm across the fin after {time.time() - t:.1f} s")
        self.gripper_close()
        self.begin("push")
        self.box_at_push = self.box.copy()
        self.step_to_end("push", 2, at_handle=True)
        self.mark("push:end")
        d = self.box[:3] - self.box_at_push[:3]
        self.ev(f"push: the box moved [{d[0] * 1e3:+.0f}, {d[1] * 1e3:+.0f}, {d[2] * 1e3:+.0f}] mm")
        time.sleep(a.push_hold)
        # 4 Exit To Push: open, wait for the jaws, exit
        self.begin("release")
        self.gripper_open("release")
        time.sleep(a.release_wait)
        self.begin("exit")
        self.step_to_end("exit", 3, at_handle=False)
        self.mark("exit:end")
        # 5 Go To Land
        time.sleep(a.leg_pause)
        self.begin("go_to_land")
        self.step_to_end("go_to_land", 4)
        self.mark("go_to_land:end")
        self.wait(lambda: self.pl_status.startswith("COMPLETE"), 10, "COMPLETE")
        time.sleep(a.post_hold)
        self.phase = "done"

    def land(self):
        self.phase = "land"
        # a planner ABORT (release hold, climb, arm home) plays out in DIRECT
        # before the SAFETY handover, as it does under the GS
        if self.mode == "DIRECT" and self.pl_status.startswith("ABORTING"):
            t = time.time()
            while time.time() - t < 25.0 and self.mode == "DIRECT" and \
                    not self.pl_status.startswith("ABORTED"):
                time.sleep(0.05)
            self.ev(f"planner abort finished: {self.pl_status}")
            time.sleep(2.0)
        self.step = -1
        if self.mode == "DIRECT":
            self.ref = self.odom[:3].copy()
            self.stream_ref = True
            try:
                self.call(self.direct, SetBool.Request(data=False), "set_direct_mode(false)", must=False)
                self.wait(lambda: self.mode != "DIRECT", 5, "SAFETY")
            except Abort as e:
                self.ev(str(e))
        self.stream_ref = True
        time.sleep(self.a.abort_settle)
        self.ref = self.odom[:3].copy()
        self.ev("landing by reference")
        while self.ref[2] > self.a.land_z:
            self.ref[2] = max(self.a.land_z, self.ref[2] - 0.2 / 50.0)
            time.sleep(0.02)
        time.sleep(self.a.land_wait)
        try:
            self.trig("disarm", must=False)
        except Abort as e:
            self.ev(str(e))

    def save(self, path, aborted, reason):
        arr = {}
        for k, v in self.L.items():
            if not v:
                arr[k] = np.zeros((0, 1))
                continue
            w = max(len(r) for r in v)
            arr[k] = np.array([list(r) + [np.nan] * (w - len(r)) for r in v],
                              dtype=np.float32 if k == "dbg" else np.float64)
        np.savez_compressed(path, **arr, events=np.array(self.events),
                            marks_t=np.array([m[0] for m in self.marks]),
                            marks_name=np.array([m[1] for m in self.marks]),
                            info=self.info if self.info is not None else np.zeros(0),
                            box0=getattr(self, "box0", np.zeros(7)),
                            box_final=self.box if self.box is not None else np.zeros(7),
                            joint_names=np.array(getattr(self, "joint_names", [])),
                            aborted=aborted, reason=reason)
        print(f"saved {path}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--namespace", default="/uav_0")
    ap.add_argument("--rate", type=float, default=50.0)
    ap.add_argument("--hover-z", type=float, default=1.0)
    ap.add_argument("--land-z", type=float, default=0.30)
    ap.add_argument("--grip-tol", type=float, default=0.02, help="the GS's push_pull_grip_tol [m]")
    ap.add_argument("--grip-axis-tol", type=float, default=0.003, help="the GS's push_pull_grip_axis_tol [m]")
    ap.add_argument("--grip-dwell-in", type=float, default=0.3)
    ap.add_argument("--grip-timeout", type=float, default=30.0)
    ap.add_argument("--grip-action", default="/uav_0/fsc_open_manipulator/gripper_controller/gripper_cmd")
    ap.add_argument("--grip-closed-m", type=float, default=-0.010, help="the GS's gripper_closed [m]")
    ap.add_argument("--settle-timeout", type=float, default=45.0)
    ap.add_argument("--hover-dwell", type=float, default=2.0,
                    help="hover above the handle this long before Ready's descent [s] (the trim window)")
    ap.add_argument("--max-disturb", type=float, default=0.008)
    ap.add_argument("--push-hold", type=float, default=1.0, help="hold on the handle after the push [s]")
    ap.add_argument("--release-wait", type=float, default=1.0, help="the GS's push_pull_release_wait_s")
    ap.add_argument("--direct-settle", type=float, default=15.0)
    ap.add_argument("--leg-pause", type=float, default=2.0)
    ap.add_argument("--post-hold", type=float, default=5.0)
    ap.add_argument("--abort-settle", type=float, default=6.0)
    ap.add_argument("--land-wait", type=float, default=12.0)
    ap.add_argument("--out", default="pl_run.npz")
    a = ap.parse_args()
    rclpy.init()
    d = Driver(a)
    spin = threading.Thread(target=rclpy.spin, args=(d,), daemon=True)
    spin.start()
    aborted, reason = False, ""
    try:
        d.mission()
        d.ev("mission complete")
    except Abort as e:
        aborted, reason = True, str(e)
        d.ev("ABORT: " + reason)
    except KeyboardInterrupt:
        aborted, reason = True, "interrupted"
    finally:
        try:
            if d.armed:
                d.land()
        finally:
            d.save(a.out, aborted, reason)
            print("ABORTED: " + reason if aborted else "completed")
            d.destroy_node()
            rclpy.try_shutdown()
            spin.join(timeout=5.0)
    return 1 if aborted else 0


if __name__ == "__main__":
    raise SystemExit(main())
