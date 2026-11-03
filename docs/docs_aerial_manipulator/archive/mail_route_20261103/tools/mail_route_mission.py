#!/usr/bin/env python3
"""Fly the MAIL ROUTE (2026-11-03) in 08's Isaac scene -- the arm GS's "Mail Route"
tab, scripted. A COPY of pick_place_controllers_20261003/tools/pnp_mission_v2.py
(its --mail path; the timetable, abort tests, drop and hook places dropped),
which is not touched:

    FASTRTPS_DEFAULT_PROFILES_FILE=<udp-only xml> /usr/bin/python3 mail_route_mission.py --out run.npz

  on the ground: Reset, Adjust, the five Gets in order -- Letter 1, 2, 3, RED box,
      BLUE box (08 moves the localization frame onto each in turn by itself)
  offboard -> arm -> SAFETY climb to hover_z -> settle -> DIRECT -> settle -> Plan
  for each letter of --order (default 1,2,3: nearest hat first):
    Letter k            the planner's route/select_letter_<k> (the tab's letter button)
    1   go_to_start     to the letter's Pick Start
    2.1 Ready To Pick   OPEN the gripper, execute_pick: hold behind the letter's near edge
    2.2 Pick            descend_pick; CLOSE (GripperCommand) once the EE is within 2 cm
                        and centred VERTICALLY to 8 mm
    2.3 Exit To Pick    as soon as the close returns
    3   go_to_place_start   the letter's mailbox's Place Start
    4.1 Ready To Insert execute_place; the planner slides in by itself and signals
                        the release on pick_place/route/drop_now
    4.2 Insert          OPEN (GripperCommand) on that release
    4.3 Exit To Insert  on the open's SUCCEEDED; then CLOSE
  5 go_to_land_start, 6 execute_land -> SAFETY -> land by reference -> disarm

A letter the planner ABORTS (its tilt guard, or the flight node's emergency
abort) is given up and the route goes on with the next one -- the operator's
"select another letter" -- as long as the vehicle is still in DIRECT. Records
the three envelopes' ground truth (payload_0..2), odometry, the measured claw,
the streamed reference, the planner's arrival error and info, the gripper, the
joints, the controller's debug array and the events; at the end it prints where
each envelope is. Refuses to start against an armed vehicle.
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
DA, DBG = "fsc_autopilot_ros2/whole_body_direct_actuation", "wb_control_debug"
REF_TOPIC = "fsc_autopilot_ros2/whole_body_direct_actuation/reference"
PP = "whole_body_planner/pick_place"
LEGS = ["go_to_start", "execute_pick", "go_to_place_start", "execute_place", "go_to_land_start", "execute_land"]
STEPS = ["select", "go_to_start", "ready_pick", "pick", "exit_pick", "go_to_place_start",
         "ready_place", "place", "exit_place", "go_to_land_start", "execute_land"]
GRIP_CLOSE, GRIP_OPEN = -0.8727, 0.0           # rad (06's gripper bus, +-50 deg)
N_LETTERS = 3
BOX_OF = ["RED", "BLUE", "RED"]               # the planner's alternation


class Abort(Exception):
    pass


class LetterFailed(Exception):
    """The planner aborted this letter; the vehicle holds in DIRECT: go on with the next."""
    pass


class Driver(Node):
    def __init__(self, a):
        super().__init__("mail_route_mission")
        self.a = a
        ns = a.namespace.rstrip("/")
        self.t0 = time.time()
        self.events, self.marks = [], []
        self.odom = self.armed = None
        self.mode = ""
        self.status = self.pp_status = ""
        self.info = self.arrival = self.grip = None
        self.payload = [None] * N_LETTERS
        self.letter = -1                      # the letter being flown (0-based), for the records
        self.step = -1
        self.phase = "init"
        self.ref = np.array([0.0, 0.0, a.hover_z])
        self.stream_ref = True
        self.grip_cmd = None
        self.results = {}                     # letter -> "posted" / "aborted: ..." / "not flown"
        self.L = {k: [] for k in ("odom", "dbg", "ee", "ref", "grip", "joints", "arrival", "claw", "route_info")}
        for k in range(N_LETTERS):
            self.L[f"env{k}"] = []            # envelope k's ground truth (payload_k)
        sub = self.create_subscription
        sub(Odometry, f"{ns}/state_estimator/local_position/odom", self.on_odom, 10)
        sub(String, f"{ns}/{DA}/mode", lambda m: setattr(self, "mode", m.data), LATCHED)
        sub(Float32MultiArray, f"{ns}/{DA}/{DBG}", self.on_dbg, 50)
        sub(VehicleStatus, f"{ns}/fmu/out/vehicle_status_v1",
            lambda m: setattr(self, "armed", m.arming_state == 2), PX4_QOS)
        sub(String, f"{ns}/whole_body_planner/status", self.on_status, LATCHED)
        sub(String, f"{ns}/{PP}/status", self.on_pp_status, LATCHED)
        sub(Float64MultiArray, f"{ns}/{PP}/info", self.on_info, LATCHED)
        sub(Float64MultiArray, f"{ns}/{PP}/arrival_error", self.on_arrival, 10)
        # the ROUTE's autonomous release (its own topic: the Mail Envelope tab acts on pick_place/drop_now)
        self.drop_now = None
        sub(String, f"{ns}/{PP}/route/drop_now", lambda m: setattr(self, "drop_now", m.data), 10)
        sub(PoseStamped, f"{ns}/whole_body_planner/current_ee", self.on_ee, 10)
        sub(WholeBodyReference, f"{ns}/{REF_TOPIC}", self.on_ref, 50)
        for k in range(N_LETTERS):
            sub(PoseStamped, f"/payload_{k}/state/pose", lambda m, k=k: self.on_payload(k, m),
                qos_profile_sensor_data)
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
        for n in LEGS + ["descend_pick", "descend_place", "exit_pick", "exit_place", "adjust", "plan",
                         "reset", "abort"]:
            self.cli[n] = self.create_client(Trigger, f"{ns}/{PP}/{n}")
        for k in range(N_LETTERS):
            self.cli[f"capture_letter_{k + 1}"] = self.create_client(Trigger, f"{ns}/{PP}/route/capture_letter_{k + 1}")
            self.cli[f"select_letter_{k + 1}"] = self.create_client(Trigger, f"{ns}/{PP}/route/select_letter_{k + 1}")
        for b in range(2):
            self.cli[f"get_box_{b + 1}"] = self.create_client(Trigger, f"{ns}/{PP}/route/get_box_{b + 1}")
        # the route's own Plan, as the Mail Route tab (2026-11-03 audit)
        self.cli["route_plan"] = self.create_client(Trigger, f"{ns}/{PP}/route/plan")
        self.direct = self.create_client(SetBool, f"{ns}/{DA}/set_direct_mode")
        self.clear = self.create_client(Trigger, f"{ns}/whole_body_planner/clear")
        self.create_timer(1.0 / a.rate, self.tick)

    # ---- callbacks ---------------------------------------------------------
    def now(self):
        return time.time() - self.t0

    def on_odom(self, m):
        p, v, q = m.pose.pose.position, m.twist.twist.linear, m.pose.pose.orientation
        self.odom = np.array([p.x, p.y, p.z, v.x, v.y, v.z])
        self.L["odom"].append([self.now(), p.x, p.y, p.z, v.x, v.y, v.z, q.w, q.x, q.y, q.z,
                               1.0 if self.mode == "DIRECT" else 0.0, self.step, self.letter])

    def on_dbg(self, m):
        if self.mode == "DIRECT":
            self.L["dbg"].append(np.concatenate([[self.now(), self.step, self.letter],
                                                 np.asarray(m.data, np.float32)[:150]]))

    def on_status(self, m):
        if m.data != self.status:
            self.ev(f"planner: {m.data}")
        self.status = m.data

    def on_pp_status(self, m):
        if m.data != self.pp_status:
            self.ev(f"pick_place: {m.data}")
        self.pp_status = m.data

    def on_info(self, m):
        self.info = np.asarray(m.data)
        self.L["route_info"].append([self.now(), *(self.info[88:] if len(self.info) > 88 else [])])

    def on_arrival(self, m):
        self.arrival = list(m.data)
        self.L["arrival"].append([self.now(), *self.arrival[:10], self.step, self.letter])

    def on_ee(self, m):
        p, q = m.pose.position, m.pose.orientation
        self.L["ee"].append([self.now(), p.x, p.y, p.z, q.w, q.x, q.y, q.z, self.step, self.letter])

    def on_ref(self, m):
        r = m.r_ed
        self.L["ref"].append([self.now(), r.x, r.y, r.z, m.x_cd.x, m.x_cd.y, m.x_cd.z,
                              *list(m.q_d)[:4], self.step, m.b1_d.x, m.b1_d.y, m.b1_d.z,
                              m.x_cd_ddot.x, m.x_cd_ddot.y, m.x_cd_ddot.z, m.b1_de.x, m.b1_de.y, m.b1_de.z,
                              self.letter])

    def on_payload(self, k, m):
        p, q = m.pose.position, m.pose.orientation
        self.payload[k] = np.array([p.x, p.y, p.z, q.w, q.x, q.y, q.z])
        self.L[f"env{k}"].append([self.now(), *self.payload[k], self.step, self.letter])

    def on_claw(self, m):
        p = m.pose.position
        self.L["claw"].append([self.now(), p.x, p.y, p.z, self.step, self.letter])

    def on_grip(self, m):
        if m.position:
            self.grip = float(m.position[0])
            eff = float(m.effort[0]) if m.effort else float("nan")
            self.L["grip"].append([self.now(), self.grip, eff, self.step, self.letter])

    def on_joints(self, m):
        if len(m.position) >= 4:
            eff = list(m.effort[:4]) if len(m.effort) >= 4 else [np.nan] * 4
            self.L["joints"].append([self.now(), *m.position[:4], *eff, self.step, self.letter])
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
        if self.phase == "mission" and self.letter >= 0 and self.pp_status.startswith(("ABORTING", "ABORTED")):
            raise LetterFailed(f"the planner aborted letter {self.letter + 1} while {what}: {self.pp_status}")

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
        t = time.time()
        while not fut.done():
            if time.time() - t > 15.0:
                raise Abort(f"{name}: no response")
            time.sleep(0.02)
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
        self.mark(f"L{self.letter + 1}:{s}:start" if self.letter >= 0 else f"{s}:start")

    def end(self, s):
        self.mark(f"L{self.letter + 1}:{s}:end" if self.letter >= 0 else f"{s}:end")

    def start(self, name):
        """Call a leg / exit, retrying while the planner says the body is still
        settling onto its hold (what the operator does: wait, press again)."""
        t = time.time()
        while True:
            r = self.trig(name, must=False)
            if r.success:
                return
            if "motion is in flight" in r.message and (self.pp_status or "").startswith(f"FLYING {name}"):
                # (2026-11-02) the arm GS already started it: its own OPEN on the auto release
                # won the race and it starts Exit To Insert on that open's SUCCEEDED
                self.ev(f"{name}: already FLYING -- started by the arm GS")
                return
            if "wait for it to settle" not in r.message or time.time() - t > self.a.settle_timeout:
                raise Abort(f"{name} refused: {r.message}")
            time.sleep(0.5)

    def leg(self, name, step):
        """A planner leg to its end (a claw leg to its HOLD in front of the target)."""
        self.begin(step)
        self.start(name)
        self.wait(lambda: self.status.startswith("EXECUTING"), 20, f"{name} EXECUTING")
        T = float(self.status.split("T=")[1].split("s")[0]) if "T=" in self.status else 60.0
        self.ev(f"{name}: executing, T = {T:.1f} s")
        if name in ("execute_pick", "execute_place"):
            self.wait(lambda: self.pp_status.startswith("WAITING") and self.status == "HOLD", T + 40,
                      f"{name} holding in front of its target")
        else:
            k = LEGS.index(name)
            self.wait(lambda: self.status == "HOLD" and self.info is not None and int(self.info[5]) == k,
                      T + 40, f"{name} complete")
        self.end(step)

    def descend_pick(self):
        """Pick: slide in (once the claw has settled behind the letter), then the EE within
        grip_tol and centred VERTICALLY (the jaws close vertically, q4 = 90 deg)."""
        self.begin("pick")
        time.sleep(self.a.hover_dwell)
        t = time.time()
        while True:      # the planner refuses until the claw is inside arrival_tol
            r = self.trig("descend_pick", must=False)
            if r.success:
                break
            if time.time() - t > self.a.settle_timeout or "wait for it to settle" not in r.message:
                raise Abort(f"descend_pick refused: {r.message}")
            time.sleep(0.5)
        self.wait(lambda: self.status == "HOLD" and self.info is not None and int(self.info[5]) == 1
                  and self.info[85] > 0.5, 40, "descend_pick complete")
        t, since = time.time(), None
        while True:
            ok = (self.arrival is not None and int(self.arrival[0]) == 1 and self.arrival[1] > 0.5
                  and self.arrival[2] <= self.a.grip_tol and abs(self.arrival[5]) <= self.a.grip_vertical_tol)
            since = (since or time.time()) if ok else None
            if since and time.time() - since >= self.a.grip_dwell_in:
                break
            if time.time() - t > self.a.grip_timeout:
                raise Abort(f"pick: the EE never came within {self.a.grip_tol * 1e3:.0f} mm, centred "
                            f"(last {self.arrival[2] * 1e3:.0f} mm, vertical {self.arrival[5] * 1e3:+.1f} mm)")
            self.guard("pick")
            time.sleep(0.02)
        self.ev(f"pick: EE {self.arrival[2] * 1e3:.1f} mm from the target, vertical "
                f"{self.arrival[5] * 1e3:+.1f} mm -- gripper CLOSE")

    def gripper_action(self, open_, what):
        """The arm GS's GripperCommand goal. CLOSE: any result is the grasp (a 4 mm
        envelope stalls or closes fully). OPEN: must reach SUCCEEDED (or, preempted by
        the arm GS's own open on the same release, the jaws must reach open)."""
        from control_msgs.action import GripperCommand
        from action_msgs.msg import GoalStatus
        self.mark(f"L{self.letter + 1}:gripper_{what}:start")
        self.grip_cmd = None                      # the action server drives the bus now
        goal = GripperCommand.Goal()
        goal.command.position = self.a.grip_open_m if open_ else self.a.grip_closed_m
        goal.command.max_effort = 50.0
        if not self.grip_ac.wait_for_server(timeout_sec=5.0):
            raise Abort(f"no gripper action server at {self.a.grip_action}")
        fut = self.grip_ac.send_goal_async(goal)
        t = time.time()
        while not fut.done() and time.time() - t < 5.0:
            time.sleep(0.01)
        gh = fut.result() if fut.done() else None
        if gh is None or not gh.accepted:
            raise Abort(f"gripper {what} goal not accepted")
        rf = gh.get_result_async()
        while not rf.done() and time.time() - t < 12.0:
            time.sleep(0.01)
        if not rf.done():
            raise Abort(f"gripper {what}: no result within 12 s")
        res, status = rf.result().result, rf.result().status
        self.grip_cmd = GRIP_OPEN if open_ else GRIP_CLOSE   # hold it on the bus
        self.ev(f"gripper {what} (action): status {status} stalled={res.stalled} reached={res.reached_goal} "
                f"after {time.time() - t:.2f} s, at {np.degrees(self.grip or np.nan):.1f} deg")
        if open_ and status == GoalStatus.STATUS_ABORTED:
            # (2026-11-02) TWO clients answer the planner's release: this driver and the running
            # arm GS's Mail Route tab. The server serves the newest goal and aborts the older,
            # so this one can read ABORTED while the jaws open normally
            t2 = time.time()
            while time.time() - t2 < 3.0 and not (self.grip is not None and self.grip > GRIP_OPEN - 0.0873):
                time.sleep(0.02)
            if self.grip is not None and self.grip > GRIP_OPEN - 0.0873:
                self.ev(f"gripper {what}: goal ABORTED (preempted by the arm GS's own open), but the jaws "
                        f"reached {np.degrees(self.grip):.1f} deg -- taken as SUCCEEDED")
                status = GoalStatus.STATUS_SUCCEEDED
        self.mark(f"L{self.letter + 1}:gripper_{what}:end")
        if open_ and status != GoalStatus.STATUS_SUCCEEDED:
            raise Abort("gripper OPEN did not succeed -- the arm GS would not start Exit To Insert")

    def exit(self, leg_k, step):
        self.begin(step)
        name = "exit_pick" if leg_k == 1 else "exit_place"
        self.start(name)
        self.wait(lambda: self.status.startswith("EXECUTING") or self.pp_status.startswith(f"FLYING {name}")
                  or (self.info is not None and self.info[83 + (leg_k == 3)] > 0.5), 20, f"{name} EXECUTING")
        k = 83 if leg_k == 1 else 84
        self.wait(lambda: self.status == "HOLD" and self.info is not None and
                  (self.info[k] > 0.5 or (leg_k == 3 and self.route_state(self.letter) == 2)), 60, f"{name} complete")
        self.end(step)

    def route_state(self, k):
        """The planner's state of letter k (0 to do, 1 picked, 2 posted, 3 dropped), -1 unknown."""
        if self.info is None or len(self.info) <= 92 + k:
            return -1
        return int(round(self.info[92 + k]))

    # ---- the mission -------------------------------------------------------
    def mission(self):
        a = self.a
        self.wait(lambda: self.odom is not None and self.mode and self.armed is not None
                  and all(p is not None for p in self.payload) and self.pp_status, 90,
                  "vehicle + three envelopes + planner feeds")
        if self.armed:
            raise Abort("vehicle already ARMED -- refusing (clean relaunch needed)")
        self.payload0 = [p.copy() for p in self.payload]
        for k in range(N_LETTERS):
            self.ev(f"envelope {k + 1} at rest {np.round(self.payload0[k][:3], 4).tolist()}")
        # on the ground: Reset, Adjust, the five Gets -- 08 moves the frame onto each in turn
        self.trig("reset", must=False)
        if not self.a.no_adjust:
            self.trig("adjust")
        time.sleep(1.0)
        for name in [f"capture_letter_{k + 1}" for k in range(N_LETTERS)] + ["get_box_1", "get_box_2"]:
            self.trig(name)
            time.sleep(1.5)                       # the frame moves on, a capture window of the new pose
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
        self.stream_ref = False                   # before the switch (2026-10-04, bench_geo_5)
        time.sleep(0.3)
        self.call(self.direct, SetBool.Request(data=True), "set_direct_mode(true)")
        self.wait(lambda: self.mode == "DIRECT", 5, "DIRECT")
        self.phase = "mission"
        self.mark("direct")
        time.sleep(a.direct_settle)
        if self.status.split(" ")[0].split(":")[0] in ("PENDING", "PLANNED", "CALCULATING"):
            self.call(self.clear, Trigger.Request(), "planner clear", must=False)
        self.wait(lambda: self.status == "HOLD", 15, "planner HOLD")
        self.trig("route_plan")
        self.wait(lambda: self.pp_status.startswith("READY"), 60, "route READY")
        self.ev(f"route planned: {self.info[92 + 2 * N_LETTERS]:.1f} s of flight")

        for k in [int(x) - 1 for x in a.order.split(",")]:
            self.letter = k
            try:
                self.fly_letter(k)
                self.results[k] = "posted"
            except LetterFailed as e:
                self.results[k] = f"aborted: {e}"
                self.ev(f"LETTER {k + 1} FAILED -- {e}")
                self.grip_cmd = GRIP_OPEN         # the arm GS opens on ABORTING ...
                self.wait_unguarded(lambda: self.pp_status.startswith("ABORTED") and self.status == "HOLD",
                                    60, "the abort's hold")
                self.grip_cmd = GRIP_CLOSE        # ... and closes on ABORTED
                time.sleep(a.leg_pause)
            time.sleep(a.leg_pause)
        self.letter = -1
        self.leg("go_to_land_start", "go_to_land_start")
        time.sleep(a.leg_pause)
        self.leg("execute_land", "execute_land")
        self.wait(lambda: self.pp_status.startswith("COMPLETE"), 10, "COMPLETE")
        time.sleep(a.post_hold)
        self.phase = "done"

    def wait_unguarded(self, cond, timeout, what):
        t = time.time()
        while time.time() - t < timeout:
            if cond():
                return
            if self.armed and self.mode != "DIRECT":
                raise Abort(f"left DIRECT while {what}")
            time.sleep(0.05)
        raise Abort(f"timeout ({timeout:.0f} s): {what}")

    def fly_letter(self, k):
        a = self.a
        self.begin("select")
        self.trig(f"select_letter_{k + 1}")
        self.ev(f"LETTER {k + 1} -> mailbox {BOX_OF[k]}")
        self.end("select")
        self.leg("go_to_start", "go_to_start")
        time.sleep(a.leg_pause)
        # 2.1 Ready To Pick: OPEN first, then behind the letter's near edge
        self.grip_cmd = GRIP_OPEN
        self.leg("execute_pick", "ready_pick")
        # 2.2 Pick
        moved = np.linalg.norm(self.payload[k][:3] - self.payload0[k][:3])
        if moved > a.max_disturb:
            self.ev(f"WARNING: envelope {k + 1} moved {moved * 1e3:.1f} mm on its hat before the pick")
        self.descend_pick()
        self.gripper_action(False, "close")
        # 2.3 Exit To Pick at once
        self.exit(1, "exit_pick")
        self.leg("go_to_place_start", "go_to_place_start")
        time.sleep(a.leg_pause)
        # 4.1 Ready To Insert: the planner slides in and releases by itself
        self.drop_now = None
        self.leg("execute_place", "ready_place")
        self.begin("place")
        if a.insert_press_after > 0.0:
            t0 = time.time()
            while time.time() - t0 < a.insert_press_after and self.pp_status.startswith("WAITING execute_place"):
                self.guard("ready to insert")
                time.sleep(0.1)
            # the slide has not started: press Insert, again every 3 s while the planner refuses
            while self.pp_status.startswith("WAITING execute_place") and time.time() - t0 < 60.0:
                r = self.trig("descend_place", must=False)
                self.ev(f"place: the slide had not started after {time.time() - t0:.0f} s -- Insert pressed: "
                        f"{r.message}")
                if r.success:
                    break
                for _ in range(30):
                    self.guard("ready to insert")
                    time.sleep(0.1)
        self.wait(lambda: self.status == "HOLD" and self.info is not None and int(self.info[5]) == 3
                  and self.info[85] > 0.5, 60, "auto insert complete")
        e = self.arrival[2] * 1e3 if self.arrival is not None else float("nan")
        self.ev(f"place: inserted, EE {e:.1f} mm from the release point")
        self.wait(lambda: self.drop_now is not None, 30, "the planner's release")
        self.ev(f"place: planner released -- {self.drop_now}")
        # 4.2 the OPEN, 4.3 Exit To Insert on its SUCCEEDED
        self.gripper_action(True, "open")
        time.sleep(a.release_dwell)
        self.end("place")
        self.exit(3, "exit_place")
        self.grip_cmd = GRIP_CLOSE                # the GS closes it once the exit is done
        self.ev(f"letter {k + 1}: Exit To Insert done -- gripper CLOSE")

    def land(self):
        self.phase = "land"
        self.step = -1
        if self.mode == "DIRECT" and not self.pp_status.startswith(("ABORTED", "COMPLETE")):
            # (2026-11-03, route1) the OPERATOR's abort first -- out of the mailbox mouth, up,
            # arm home -- and only then SAFETY: a SAFETY switch straight from the insert hold
            # (the arm reaching forward, the letter 0.11 m from the box face) pitched the
            # vehicle into the mailbox
            try:
                self.grip_cmd = GRIP_OPEN
                r = self.trig("abort", must=False)
                if r.success:
                    self.wait_unguarded(lambda: self.pp_status.startswith("ABORTED") and self.status == "HOLD",
                                        40, "the abort before SAFETY")
                    self.grip_cmd = GRIP_CLOSE
            except Abort as e:
                self.ev(f"abort before SAFETY: {e}")
        if self.mode == "DIRECT":
            self.ref = self.odom[:3].copy()
            self.stream_ref = True
            try:
                self.call(self.direct, SetBool.Request(data=False), "set_direct_mode(false)", must=False)
                self.wait_unguarded(lambda: self.mode != "DIRECT", 5, "SAFETY")
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

    def where(self, k):
        """Where envelope k is, by its ground truth (the scene's mailbox geometry, defaults)."""
        P = self.payload[k]
        if P is None:
            return "unknown"
        w, x, y, z = P[3:7]
        ex = np.array([1 - 2 * (y * y + z * z), 2 * (x * y + w * z), 2 * (x * z - w * y)])
        c = P[:3] + (0.5 * self.a.env_l - self.a.env_com) * ex           # its centre of mass
        x0, x1 = self.a.box_front_x + 0.005, self.a.box_front_x + 0.125 - 0.005
        for name, cy in (("RED", self.a.red_y), ("BLUE", self.a.blue_y)):
            if x0 < c[0] < x1 and abs(c[1] - cy) < 0.5 * 0.235 - 0.005 and c[2] < self.a.mouth_z:
                return f"in the {name} mailbox"
        if np.linalg.norm(P[:3] - self.payload0[k][:3]) < 0.01:
            return "still on its hat"
        if c[2] < 0.05:
            return "on the floor"
        return f"elsewhere {np.round(c, 3).tolist()}"

    def save(self, path, aborted, reason):
        arr = {}
        for k, v in self.L.items():
            if not v:
                arr[k] = np.zeros((0, 1))
                continue
            w = max(len(r) for r in v)
            arr[k] = np.array([list(r) + [np.nan] * (w - len(r)) for r in v],
                              dtype=np.float32 if k == "dbg" else np.float64)
        final = [self.where(k) for k in range(N_LETTERS)]
        np.savez_compressed(path, **arr, events=np.array(self.events),
                            marks_t=np.array([m[0] for m in self.marks]),
                            marks_name=np.array([m[1] for m in self.marks]),
                            info=self.info if self.info is not None else np.zeros(0),
                            payload0=np.array(getattr(self, "payload0", [np.zeros(7)] * N_LETTERS)),
                            payload_final=np.array([p if p is not None else np.zeros(7) for p in self.payload]),
                            envelope_final=np.array(final), order=self.a.order,
                            results=np.array([self.results.get(k, "not flown") for k in range(N_LETTERS)]),
                            joint_names=np.array(getattr(self, "joint_names", [])),
                            aborted=aborted, reason=reason)
        for k in range(N_LETTERS):
            want = BOX_OF[k]
            got = final[k]
            ok = got == f"in the {want} mailbox"
            print(f"LETTER {k + 1} -> {want}: {self.results.get(k, 'not flown')}; envelope {got}"
                  f"{'  OK' if ok else ''}", flush=True)
        print(f"saved {path}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--namespace", default="/uav_0")
    ap.add_argument("--order", default="1,2,3", help="the letters in flying order (the tab's buttons)")
    ap.add_argument("--no-adjust", action="store_true",
                    help="skip Adjust (a take-off off the START mark: Adjust refuses an offset over 2 m)")
    ap.add_argument("--rate", type=float, default=50.0)
    ap.add_argument("--hover-z", type=float, default=1.0)
    ap.add_argument("--land-z", type=float, default=0.30)
    ap.add_argument("--grip-tol", type=float, default=0.02, help="the GS's pick_place_grip_tol [m]")
    ap.add_argument("--grip-vertical-tol", type=float, default=0.008, help="the GS's pick_place_grip_vertical_tol [m]")
    ap.add_argument("--grip-dwell-in", type=float, default=0.3, help="EE inside the gates this long [s]")
    ap.add_argument("--grip-timeout", type=float, default=30.0)
    ap.add_argument("--settle-timeout", type=float, default=45.0, help="retry a refused step this long [s]")
    ap.add_argument("--hover-dwell", type=float, default=2.0, help="hold behind the letter before Pick [s]")
    ap.add_argument("--release-dwell", type=float, default=0.5)
    ap.add_argument("--grip-action", default="/uav_0/fsc_open_manipulator/gripper_controller/gripper_cmd")
    ap.add_argument("--grip-open-m", type=float, default=0.019, help="the arm GS's OPEN position [m]")
    ap.add_argument("--grip-closed-m", type=float, default=-0.010, help="the arm GS's gripper_closed [m]")
    ap.add_argument("--insert-press-after", type=float, default=20.0,
                    help="[s] press Insert if the autonomous slide has not started this long after Ready To "
                         "Insert, as the operator does (0 = never, wait for the planner; route1 waited 60 s "
                         "on the 0.03 m/s claw-speed gate)")
    ap.add_argument("--max-disturb", type=float, default=0.008)
    ap.add_argument("--direct-settle", type=float, default=15.0)
    ap.add_argument("--leg-pause", type=float, default=2.0)
    ap.add_argument("--post-hold", type=float, default=5.0)
    ap.add_argument("--abort-settle", type=float, default=6.0)
    ap.add_argument("--land-wait", type=float, default=12.0)
    # the scene's geometry, for the end-of-run verdict (08's defaults)
    ap.add_argument("--box-front-x", type=float, default=1.70)
    ap.add_argument("--red-y", type=float, default=0.25)
    ap.add_argument("--blue-y", type=float, default=-0.25)
    ap.add_argument("--mouth-z", type=float, default=0.80 + 0.2025 - 0.045, help="the mouth's lower lip [m]")
    ap.add_argument("--env-l", type=float, default=0.150)
    ap.add_argument("--env-com", type=float, default=0.040)
    ap.add_argument("--out", default="mail_route_run.npz")
    a = ap.parse_args()
    rclpy.init()
    d = Driver(a)
    spin = threading.Thread(target=rclpy.spin, args=(d,), daemon=True)
    spin.start()
    aborted, reason = False, ""
    try:
        d.mission()
        d.ev("mission complete")
    except (Abort, LetterFailed) as e:
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
