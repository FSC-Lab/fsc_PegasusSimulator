#!/usr/bin/env python3
"""Fly the AUTONOMOUS pick-and-place mission in 07's Isaac scene with the
2026-10-03 step flow, on either controller -- the arm GS's Pick & Place tab,
scripted:

    FASTRTPS_DEFAULT_PROFILES_FILE=<udp-only xml> /usr/bin/python3 pnp_mission_v2.py --rig wb|geo --out run.npz

  on the ground: Reset, Adjust, Get (obj_0)            (the place pose is typed)
  offboard -> arm -> SAFETY climb to hover_z -> settle -> DIRECT -> settle -> Plan
  1   go_to_start
  2.1 Ready To Pick    OPEN the gripper, execute_pick: hover at the safety margin
  2.2 Pick             descend_pick; on the target, CLOSE once the EE is within 2 cm
  2.3 Exit To Pick     exit_pick as soon as the jaws stall (the closed chain drifts)
  3   go_to_place_start
  4.1 Ready To Place   execute_place: hover above the place pillar
  4.2 Place            descend_place; OPEN once the EE is within 2 cm
  4.3 Exit To Place    exit_place
  5   go_to_land_start, 6 execute_land -> SAFETY -> land by reference -> disarm

--rig picks the controller's namespace (whole-body 4-D L1 vs the decoupled
geometric + L1); the planner, its topics and the task are the same. Records
the payload ground truth, odometry, the measured claw, the streamed reference,
the planner's arrival error, the gripper, the joints, the controller's debug
array and the events, tagged by step, for pnp_score.py. Refuses to start
against an armed vehicle (a clean relaunch is needed between flights).
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
RIGS = {"wb": ("fsc_autopilot_ros2/whole_body_direct_actuation", "wb_control_debug"),
        "geo": ("fsc_autopilot_ros2/geometric_l1_direct_actuation", "l1_control_debug"),
        # the modular adaptive node answers under the whole-body namespace on purpose (2026-10-08)
        "mod": ("fsc_autopilot_ros2/whole_body_direct_actuation", "modular_control_debug")}
REF_TOPIC = "fsc_autopilot_ros2/whole_body_direct_actuation/reference"   # the planner's stream (both rigs)
PP = "whole_body_planner/pick_place"
STEPS = ["go_to_start", "ready_pick", "pick", "exit_pick", "go_to_place_start",
         "ready_place", "place", "exit_place", "go_to_land_start", "execute_land"]
GRIP_CLOSE, GRIP_OPEN = -0.8727, 0.0           # rad (06's gripper bus, +-50 deg)


class Abort(Exception):
    pass


class Driver(Node):
    def __init__(self, a):
        super().__init__("pnp_mission_v2")
        self.a = a
        ns = a.namespace.rstrip("/")
        da, dbg = RIGS[a.rig]
        self.t0 = time.time()
        self.events, self.marks = [], []
        self.late = []                       # --timetable: (step, seconds late)
        self.odom = self.mode = self.armed = None
        self.mode = ""
        self.status = self.pp_status = ""
        self.info = self.arrival = self.grip = self.payload = None
        self.step = -1
        self.phase = "init"
        self.ref = np.array([0.0, 0.0, a.hover_z])
        self.stream_ref = True
        self.grip_cmd = None
        self.L = {k: [] for k in ("odom", "dbg", "payload", "ee", "ref", "grip", "joints", "arrival", "claw")}
        sub = self.create_subscription
        sub(Odometry, f"{ns}/state_estimator/local_position/odom", self.on_odom, 10)
        sub(String, f"{ns}/{da}/mode", lambda m: setattr(self, "mode", m.data), LATCHED)
        sub(Float32MultiArray, f"{ns}/{da}/{dbg}", self.on_dbg, 50)
        sub(VehicleStatus, f"{ns}/fmu/out/vehicle_status_v1",
            lambda m: setattr(self, "armed", m.arming_state == 2), PX4_QOS)
        sub(String, f"{ns}/whole_body_planner/status", self.on_status, LATCHED)
        sub(String, f"{ns}/{PP}/status", self.on_pp_status, LATCHED)
        sub(Float64MultiArray, f"{ns}/{PP}/info", lambda m: setattr(self, "info", np.asarray(m.data)), LATCHED)
        sub(Float64MultiArray, f"{ns}/{PP}/arrival_error", self.on_arrival, 10)
        sub(PoseStamped, f"{ns}/whole_body_planner/current_ee", self.on_ee, 10)
        sub(WholeBodyReference, f"{ns}/{REF_TOPIC}", self.on_ref, 50)
        sub(PoseStamped, "/payload_0/state/pose", self.on_payload, qos_profile_sensor_data)
        sub(JointState, f"{ns}/isaacsim_manipulator/gripper_state", self.on_grip, qos_profile_sensor_data)
        sub(PoseStamped, "/claw_0/state/pose", self.on_claw, qos_profile_sensor_data)
        sub(JointState, f"{ns}/isaacsim_manipulator/joint_states", self.on_joints, qos_profile_sensor_data)
        self.ref_pub = self.create_publisher(PositionControllerReference,
                                             f"{ns}/fsc_autopilot_ros2/position_controller/reference", 10)
        self.grip_pub = self.create_publisher(Float64, f"{ns}/isaacsim_manipulator/gripper_command", 10)
        if a.grip_via_action:
            from rclpy.action import ActionClient
            from control_msgs.action import GripperCommand
            self.grip_ac = ActionClient(self, GripperCommand, a.grip_action)
        self.cli = {n: self.create_client(Trigger, f"{ns}/rc/{n}") for n in ("offboard", "arm", "disarm")}
        for n in ("go_to_start", "execute_pick", "go_to_place_start", "execute_place", "go_to_land_start",
                  "execute_land", "descend_pick", "descend_place", "exit_pick", "exit_place",
                  "adjust", "plan", "capture_pick", "reset", "abort"):
            self.cli[n] = self.create_client(Trigger, f"{ns}/{PP}/{n}")
        self.direct = self.create_client(SetBool, f"{ns}/{da}/set_direct_mode")
        self.clear = self.create_client(Trigger, f"{ns}/whole_body_planner/clear")
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

    def on_pp_status(self, m):
        if m.data != self.pp_status:
            self.ev(f"pick_place: {m.data}")
        self.pp_status = m.data

    def on_arrival(self, m):
        self.arrival = list(m.data)
        self.L["arrival"].append([self.now(), *self.arrival[:10], self.step])

    def on_ee(self, m):
        p, q = m.pose.position, m.pose.orientation
        self.L["ee"].append([self.now(), p.x, p.y, p.z, q.w, q.x, q.y, q.z, self.step])

    def on_ref(self, m):
        r = m.r_ed
        # [12..14] b1_d (MODEL body-x heading), appended 2026-10-03 for the airframe
        # reference x_b_ref = x_cd - R(psi_d) r_0c(q_d) of the Suarez et al. benchmark
        # [15..17] x_cd_ddot and [18..20] b1_de, appended 2026-10-08 for the comparison
        # metrics (the compatible attitude reference and the EE heading reference)
        self.L["ref"].append([self.now(), r.x, r.y, r.z, m.x_cd.x, m.x_cd.y, m.x_cd.z,
                              *list(m.q_d)[:4], self.step, m.b1_d.x, m.b1_d.y, m.b1_d.z,
                              m.x_cd_ddot.x, m.x_cd_ddot.y, m.x_cd_ddot.z, m.b1_de.x, m.b1_de.y, m.b1_de.z])

    def on_payload(self, m):
        p, q = m.pose.position, m.pose.orientation
        self.payload = np.array([p.x, p.y, p.z, q.w, q.x, q.y, q.z])
        self.L["payload"].append([self.now(), *self.payload, self.step])

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

    def align(self, ref_mark, slot, what):
        """--timetable (2026-10-09): hold until `slot` s after the mark `ref_mark`, so
        the next step starts at the SAME mission time on every run and every
        controller. Only called where hovering is safe. A step already past its
        slot is LATE (recorded; the campaign re-flies such a run)."""
        t_ref = [t for t, n in self.marks if n == ref_mark][-1]
        dt = t_ref + slot - self.now()
        if dt < -0.05:
            self.late.append((what, -dt))
            self.ev(f"TIMETABLE LATE: {what} {-dt:.2f} s behind its slot")
            return
        self.ev(f"timetable: {what} in {max(dt, 0.0):.2f} s")
        t_end = time.time() + max(dt, 0.0)
        while time.time() < t_end:
            self.guard(what)
            time.sleep(0.01)

    def started_late(self, step, name):
        """--timetable: the planner refused the call for a while (still settling)."""
        t_mark = [t for t, n in self.marks if n == f"{step}:start"][-1]
        lag = self.now() - t_mark
        if self.a.timetable and lag > 0.3:
            self.late.append((f"{name} accepted", lag))
            self.ev(f"TIMETABLE LATE: {name} accepted {lag:.2f} s after its mark")
        if self.phase == "mission" and self.pp_status.startswith(("ABORTING", "ABORTED")):
            raise Abort(f"pick-and-place ABORTED while {what}: {self.pp_status}")

    def wait(self, cond, timeout, what):
        t = time.time()
        while time.time() - t < timeout:
            if cond():
                return time.time() - t
            self.guard(what)
            time.sleep(0.05)
        raise Abort(f"timeout ({timeout:.0f} s): {what}  [mode={self.mode} plan={self.status} "
                    f"pick_place={self.pp_status}]")

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
        """Call a leg / exit, retrying while the planner says the body is still
        settling onto its hold (what the operator does: wait, press again)."""
        t = time.time()
        while True:
            r = self.trig(name, must=False)
            if r.success:
                return
            if "wait for it to settle" not in r.message or time.time() - t > self.a.settle_timeout:
                raise Abort(f"{name} refused: {r.message}")
            time.sleep(0.5)

    def leg(self, name, step):
        """A planner leg to its end (a claw leg to its HOLD above the target)."""
        self.begin(step)
        self.start(name)
        self.started_late(step, name)
        self.wait(lambda: self.status.startswith("EXECUTING"), 20, f"{name} EXECUTING")
        T = float(self.status.split("T=")[1].split("s")[0]) if "T=" in self.status else 60.0
        self.ev(f"{name}: executing, T = {T:.1f} s")
        if name in ("execute_pick", "execute_place"):
            self.wait(lambda: self.pp_status.startswith("WAITING") and self.status == "HOLD", T + 40,
                      f"{name} holding above")
        else:
            k = ["go_to_start", "execute_pick", "go_to_place_start", "execute_place",
                 "go_to_land_start", "execute_land"].index(name)
            self.wait(lambda: self.status == "HOLD" and self.info is not None and int(self.info[5]) == k,
                      T + 40, f"{name} complete")
        self.mark(f"{step}:end")

    def descend(self, leg_k, step, gate=True):
        """Pick / Place: descend (once the claw has settled above), then the EE within grip_tol
        (gate=False: return as soon as the descent leg is complete -- the hook place)."""
        self.begin(step)
        name = "descend_pick" if leg_k == 1 else "descend_place"
        # hover first, as the operator does: the planner averages the claw's
        # error over this (its descent trim) and the anchor blend settles
        time.sleep(self.a.hover_dwell)
        t = time.time()
        while True:      # the planner refuses until the claw is inside arrival_tol above the target
            r = self.trig(name, must=False)
            if r.success:
                break
            if time.time() - t > self.a.settle_timeout or "wait for it to settle" not in r.message:
                raise Abort(f"{name} refused: {r.message}")
            time.sleep(0.5)
        self.wait(lambda: self.status == "HOLD" and self.info is not None and int(self.info[5]) == leg_k
                  and self.info[85] > 0.5, 40, f"{name} complete")
        if not gate:
            e = self.arrival[2] * 1e3 if self.arrival is not None else float("nan")
            self.ev(f"{step}: descent complete, EE {e:.1f} mm from the target")
            return
        t, since = time.time(), None
        # the handle's THIN axis = the jaws' closing axis = the object's y axis
        q = self.payload0[3:7]
        yaw = np.arctan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] ** 2 + q[3] ** 2))
        axis = np.array([-np.sin(yaw), np.cos(yaw), 0.0])
        while True:
            ok = (self.arrival is not None and int(self.arrival[0]) == leg_k and self.arrival[1] > 0.5
                  and self.arrival[2] <= self.a.grip_tol)
            if ok and leg_k == 1 and self.a.grip_axis_tol > 0.0:
                # 2026-10-03: closing off-centre makes the first finger shove the
                # vehicle sideways (decoupled rig: then the clamped chain tips over)
                ok = abs(float(np.dot(self.arrival[3:6], axis))) <= self.a.grip_axis_tol
            since = (since or time.time()) if ok else None
            if since and time.time() - since >= self.a.grip_dwell_in:
                if leg_k == 1:
                    self.ev(f"{step}: closing-axis error {np.dot(self.arrival[3:6], axis) * 1e3:+.1f} mm "
                            f"after {time.time() - t:.1f} s at the bottom")
                break
            if time.time() - t > self.a.grip_timeout:
                raise Abort(f"{step}: the EE never came within {self.a.grip_tol * 1e3:.0f} mm "
                            f"(last {self.arrival[2] * 1e3:.0f} mm)")
            self.guard(step)
            time.sleep(0.02)
        self.ev(f"{step}: EE {self.arrival[2] * 1e3:.1f} mm from the target -- gripper")

    def gripper(self, target, what):
        if self.a.grip_via_action and target == GRIP_CLOSE:
            return self.gripper_action_close(what)
        self.mark(f"gripper_{what}:start")
        self.grip_cmd = target
        g0 = self.grip if self.grip is not None else 0.0
        t = time.time()
        while time.time() - t < 3.0 and (self.grip is None or abs(self.grip - g0) < np.radians(2.0)):
            time.sleep(0.02)
        # stalled = moved less than 0.5 deg over the last stall_s (a clamped
        # gripper jitters ~0.5 deg: a 0.1 deg test never fired and the payload
        # sat clamped on the cap for 5 s -- geo_6)
        hist = []
        while time.time() - t < 8.0:
            now, g = time.time(), self.grip
            if g is not None:
                hist.append((now, g))
            hist = [h for h in hist if now - h[0] <= self.a.stall_s]
            if (len(hist) >= 3 and hist[-1][0] - hist[0][0] >= 0.8 * self.a.stall_s
                    and abs(hist[-1][1] - hist[0][1]) < np.radians(0.5)):
                break
            time.sleep(0.02)
        self.ev(f"gripper {what}: settled at {np.degrees(self.grip or np.nan):.1f} deg "
                f"(command {np.degrees(target):.0f})")
        self.mark(f"gripper_{what}:end")

    def gripper_action_close(self, what):
        """The arm GS's path (2026-10-03): a GripperCommand CLOSE goal; its result
        comes back ABORTED + stalled once the fingers stop on the handle (the
        server's stall_time after they stop), and the GS then starts Exit To Pick
        at once -- so the caller exits right after this returns."""
        from control_msgs.action import GripperCommand
        self.mark(f"gripper_{what}:start")
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
        self.ev(f"gripper {what} (action): stalled={res.stalled} reached={res.reached_goal} "
                f"position {res.position:.4f} m, at {np.degrees(self.grip or np.nan):.1f} deg")
        self.mark(f"gripper_{what}:end")
        # the box payload's HOOK grasp (2026-10-07): the fingers close AROUND the
        # 3 mm stem without clamping it, so a full close IS the grasp (the arm GS's
        # hook mode accepts it the same way)
        if not res.stalled and not self.a.hook_place:
            raise Abort("the gripper closed on NOTHING -- not lifting")

    def exit(self, leg_k, step):
        self.begin(step)
        name = "exit_pick" if leg_k == 1 else "exit_place"
        self.start(name)
        self.started_late(step, name)
        self.wait(lambda: self.status.startswith("EXECUTING"), 20, f"{name} EXECUTING")
        k = 83 if leg_k == 1 else 84
        self.wait(lambda: self.status == "HOLD" and self.info is not None and self.info[k] > 0.5, 40,
                  f"{name} complete")
        self.mark(f"{step}:end")

    # ---- the mission -------------------------------------------------------
    def mission(self):
        a = self.a
        self.wait(lambda: self.odom is not None and self.mode and self.armed is not None
                  and self.payload is not None and self.pp_status, 90, "vehicle + payload + planner feeds")
        if self.armed:
            raise Abort("vehicle already ARMED -- refusing (clean relaunch needed)")
        self.payload0 = self.payload.copy()
        self.ev(f"payload at rest {np.round(self.payload0[:3], 4).tolist()}")
        # on the ground: Reset, Adjust, Get (the place pose is typed)
        self.trig("reset", must=False)
        self.trig("adjust")
        time.sleep(1.0)
        self.trig("capture_pick")
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
        # stop the SAFETY setpoint stream BEFORE the switch (2026-10-04, bench_geo_5):
        # the mode topic lags the switch, so setpoints still sent "until DIRECT is
        # seen" reached the planner in DIRECT as a drone target (PLANNED, never
        # HOLD). SAFETY keeps its last setpoint, so the hover is unaffected.
        self.stream_ref = False
        time.sleep(0.3)
        self.call(self.direct, SetBool.Request(data=True), "set_direct_mode(true)")
        self.wait(lambda: self.mode == "DIRECT", 5, "DIRECT")
        self.phase = "mission"
        self.mark("direct")
        self.stream_ref = False
        time.sleep(a.direct_settle)
        if self.status.split(" ")[0].split(":")[0] in ("PENDING", "PLANNED", "CALCULATING"):
            # a stray drone target captured at the switch: drop it (the PS4 bring-up does too)
            self.call(self.clear, Trigger.Request(), "planner clear", must=False)
        self.wait(lambda: self.status == "HOLD", 15, "planner HOLD")
        self.trig("plan")
        self.wait(lambda: self.pp_status.startswith("READY"), 60, "mission READY")
        self.ev(f"planned leg durations {np.round(self.info[8:14], 1).tolist()} s, exits "
                f"{np.round(self.info[81:83], 1).tolist()} s")
        self.leg("go_to_start", "go_to_start")
        if a.timetable:
            self.align("go_to_start:start", a.slot_ready_pick, "ready_pick")
        else:
            time.sleep(a.leg_pause)
        # 2.1 Ready To Pick: OPEN first, then to the safety margin above the handle
        self.grip_cmd = GRIP_OPEN
        self.leg("execute_pick", "ready_pick")
        # 2.2 Pick
        self.descend(1, "pick")
        moved = np.linalg.norm(self.payload[:3] - self.payload0[:3])
        if moved > a.max_disturb:
            raise Abort(f"payload disturbed {moved * 1e3:.1f} mm by the descent -- not closing")
        self.payload_at_close = self.payload.copy()
        self.gripper(GRIP_CLOSE, "close")
        # 2.3 Exit To Pick at once (the closed chain with the pillar drifts) -- or
        # after --clamp-hold s, an operator's reaction time (2026-10-03: the user's
        # manual decoupled run sat clamped ~4 s and tipped over at the lift)
        if a.timetable:
            # the hook grasp: the fingers close AROUND the stem without clamping it,
            # so holding here is safe; this slack absorbs the gate + close time
            self.align("ready_pick:start", a.slot_exit_pick, "exit_pick")
        elif a.clamp_hold > 0.0:
            self.mark("clamp_hold:start")
            time.sleep(a.clamp_hold)
            self.mark("clamp_hold:end")
        self.exit(1, "exit_pick")
        self.leg("go_to_place_start", "go_to_place_start")
        if a.timetable:
            self.align("exit_pick:start", a.slot_ready_place, "ready_place")
        else:
            time.sleep(a.leg_pause)
        self.leg("execute_place", "ready_place")
        if a.hook_place:
            # the box payload's HOOK place (2026-10-07): jaws CLOSED through the
            # descent (an open claw holds the arch only on its fingers' outer
            # corners, under the curving arms -- the touchdown slid it 19 mm sideways
            # onto a finger, hook4); at the end of the descent OPEN and back out AT
            # ONCE: when the basket lands the vehicle lurches ~10-13 cm back and
            # returns (the observer unlearning the payload), and waiting for the
            # claw to come back within 2 cm is waiting for a closed claw to re-enter
            # around the stem (hook3: a finger hit it and the basket fell off)
            self.descend(3, "place", gate=False)
            self.mark("gripper_open:start")
            self.grip_cmd = GRIP_OPEN
            self.exit(3, "exit_place")
            self.mark("gripper_open:end")
            if a.timetable:
                # hovering above the place point, the basket released: safe slack
                self.align("ready_place:start", a.slot_land_start, "go_to_land_start")
            self.leg("go_to_land_start", "go_to_land_start")
            if a.timetable:
                self.align("go_to_land_start:start", a.slot_land, "execute_land")
            else:
                time.sleep(a.leg_pause)
            self.leg("execute_land", "execute_land")
            self.wait(lambda: self.pp_status.startswith("COMPLETE"), 10, "COMPLETE")
            time.sleep(a.post_hold)
            self.phase = "done"
            return
        if a.open_before_place:
            # the box payload's HOOK grasp (2026-10-07): open while hovering above
            # the place -- open fingers still carry the arch, and give the stem
            # ~17 mm a side when the vehicle lurches back and returns after the
            # basket lands (the arm GS does the same at Ready To Place in hook mode)
            self.gripper(GRIP_OPEN, "open_early")
        self.descend(3, "place")
        self.gripper(GRIP_OPEN, "open")
        time.sleep(a.release_dwell)
        self.exit(3, "exit_place")
        self.leg("go_to_land_start", "go_to_land_start")
        time.sleep(a.leg_pause)
        self.leg("execute_land", "execute_land")
        self.wait(lambda: self.pp_status.startswith("COMPLETE"), 10, "COMPLETE")
        time.sleep(a.post_hold)
        self.phase = "done"

    def land(self):
        self.phase = "land"
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
                            payload0=getattr(self, "payload0", np.zeros(7)),
                            payload_final=self.payload if self.payload is not None else np.zeros(7),
                            joint_names=np.array(getattr(self, "joint_names", [])),
                            rig=self.a.rig, aborted=aborted, reason=reason,
                            timetable=np.array([self.a.slot_ready_pick, self.a.slot_exit_pick,
                                                self.a.slot_ready_place, self.a.slot_land_start,
                                                self.a.slot_land]) if self.a.timetable else np.zeros(0),
                            late_name=np.array([l[0] for l in self.late]),
                            late_s=np.array([l[1] for l in self.late]))
        print(f"saved {path}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rig", choices=sorted(RIGS), required=True)
    ap.add_argument("--namespace", default="/uav_0")
    ap.add_argument("--rate", type=float, default=50.0)
    ap.add_argument("--hover-z", type=float, default=1.0)
    ap.add_argument("--land-z", type=float, default=0.30)
    ap.add_argument("--grip-tol", type=float, default=0.02, help="the GS's pick_place_grip_tol [m]")
    ap.add_argument("--grip-dwell-in", type=float, default=0.3, help="EE inside grip_tol this long [s]")
    ap.add_argument("--grip-timeout", type=float, default=30.0)
    ap.add_argument("--settle-timeout", type=float, default=45.0, help="retry the descent this long [s]")
    ap.add_argument("--hover-dwell", type=float, default=2.0,
                    help="hover above the target this long before Pick / Place [s] (the descent trim's window)")
    ap.add_argument("--stall-s", type=float, default=0.25)
    ap.add_argument("--release-dwell", type=float, default=0.5)
    ap.add_argument("--grip-axis-tol", type=float, default=0.0,
                    help="pick: also require |error along the closing axis| <= this before closing [m] (0 = off)")
    ap.add_argument("--grip-via-action", action="store_true",
                    help="close the pick through the GripperCommand action and exit on its result (the arm GS's path)")
    ap.add_argument("--grip-action", default="/uav_0/fsc_open_manipulator/gripper_controller/gripper_cmd")
    ap.add_argument("--grip-closed-m", type=float, default=-0.010, help="the GS's gripper_closed [m]")
    ap.add_argument("--hook-place", action="store_true",
                    help="the box payload's hook place: jaws closed through the descent, then open and exit at once")
    ap.add_argument("--open-before-place", action="store_true",
                    help="open the gripper at Ready To Place, before the descent (the box payload's hook grasp)")
    ap.add_argument("--clamp-hold", type=float, default=0.0,
                    help="wait this long clamped on the pick pillar before Exit To Pick [s]")
    ap.add_argument("--max-disturb", type=float, default=0.008)
    ap.add_argument("--direct-settle", type=float, default=15.0)
    ap.add_argument("--leg-pause", type=float, default=2.0)
    # --timetable (2026-10-09): every step starts at the same mission time on every
    # run and every controller, for the comparison figures. The slots [s] are the
    # step-to-step offsets below, sized from the 2026-10-08 runs (largest flown +
    # margin); a step past its slot is recorded as LATE (npz late_name / late_s).
    # Needs the planner's pick_place_descent_trim_first_min = 0 (the sim
    # _pick_and_place.yaml) so the place descent lasts the same on every run.
    ap.add_argument("--timetable", action="store_true",
                    help="start every step at a fixed mission time (replaces --leg-pause / --clamp-hold)")
    ap.add_argument("--slot-ready-pick", type=float, default=5.3, help="go_to_start start -> Ready To Pick")
    ap.add_argument("--slot-exit-pick", type=float, default=18.3, help="Ready To Pick start -> Exit To Pick")
    ap.add_argument("--slot-ready-place", type=float, default=22.3, help="Exit To Pick start -> Ready To Place")
    ap.add_argument("--slot-land-start", type=float, default=25.0, help="Ready To Place start -> go_to_land_start")
    ap.add_argument("--slot-land", type=float, default=18.3, help="go_to_land_start start -> execute_land")
    ap.add_argument("--post-hold", type=float, default=5.0)
    ap.add_argument("--abort-settle", type=float, default=6.0)
    ap.add_argument("--land-wait", type=float, default=12.0)
    ap.add_argument("--out", default="pnp_run.npz")
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
