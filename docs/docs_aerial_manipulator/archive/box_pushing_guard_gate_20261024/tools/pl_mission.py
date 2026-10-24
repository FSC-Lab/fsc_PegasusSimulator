#!/usr/bin/env python3
"""Fly the BOX-PUSHING mission in 07's Isaac scene (07_px4_t650_aerial_
manipulator_box_pushing.py) with the whole-body 4-D L1 rig -- the arm
GS's Box Pushing tab, scripted:

    FASTRTPS_DEFAULT_PROFILES_FILE=<udp-only xml> /usr/bin/python3 pl_mission.py --out run.npz

  on the ground: Reset, Adjust, Get (the box, obj_0)
  offboard -> arm -> SAFETY climb to hover_z -> settle -> DIRECT -> settle -> Plan
  1 Go To Start      the claw the Vertical Margin (0.15 m) above and the Side
                     Margin (0.12 m) behind the fin, yaw -90
  2 Ready To Push    OPEN the gripper, hover (--hover-dwell), ready: the claw
                     slides down-forward onto the fin
  3 Push             once the claw is within 2 cm of the grasp point and 3 mm
                     across the fin: CLOSE through the GripperCommand action (the
                     GS's path); on the stall, push: CONTACT on, 1.5 s settle,
                     0.50 m along -y in 24 s, arm fixed
  4 Exit To Push     (default since 2026-10-19, the GS's sequence) RELEASE, OPEN,
                     exit on the open report: up 0.20 m, arm home
  5 Go To Land Start one table-checked transition to Land Start (1.3 m), turning
                     270 deg clockwise
  6 Land             straight down to the hover over the landing spot
  -> SAFETY -> land by reference -> disarm
After an abort near the table -- the planner's own, or one of this driver's
(e.g. no grasp, a refused step) -- the vehicle leaves by the planner's abort
and Go To Land Start before landing: never down in place beside the table.

Records the box (ground truth = obj_0's state, the same pose the mocap
carries), the odometry, the measured claw (claw_0), the streamed reference,
the planner's arrival error and info, the gripper, the joints and the
whole-body node's debug array, tagged by step, for pl_score.py. Refuses to
start against an armed vehicle (a clean relaunch is needed between flights).

THE RAW-MOCAP GUARD (2026-10-21, ../../mocap_safety_guard_20261021): the
estimator's guard health ({ns}/state_estimator/mocap_guard, [0] state 0 MOCAP /
1 BRIDGING / 2 CONFIRMED / 3 LOST, [1] cause, [2] age, ...) is recorded in the
npz as `guard` rows [time, step, *data], its state changes as events. With
--mocap-fault TYPE@STEP+DELAY[:DURATION][,BODY] (repeatable) the driver starts a
mocap fault in the Isaac emulator DELAY s after the mission mark STEP -- one
ATOMIC set_parameters call on its fault_* parameters (a refused value starts
nothing), marked mocap_fault_<TYPE>:start, the emulator's answer in the npz:
    --mocap-fault freeze@push+4:2.5      the drone's pose frozen 2.5 s, 4 s into the push
    --mocap-fault jump@go_to_start+3:0   one jumped sample, 3 s into Go To Start
    --mocap-fault stop@exit+1:0.4        no drone mocap for 0.4 s, 1 s into the exit
"""
import argparse
import threading
import time

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import SetParametersAtomically
from rclpy.parameter import Parameter as RosParameter
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
# 2026-10-16: go_to_land became go_to_land_start (one table-checked transition) + land
STEPS = ["go_to_start", "ready", "close", "push", "release", "exit", "go_to_land_start", "land"]
I_AT_HANDLE = 61   # push_pull/info with six steps ([53] with five)
GRIP_CLOSE, GRIP_OPEN = -0.8727, 0.0           # rad (06's gripper bus, +-50 deg)
# the Isaac mocap emulator's fault types (isaacsim_optitrack_ros2_emulator, 2026-10-21):
# stop = nothing published, freeze = the last message republished (fresh stamps), jump =
# position + fault_offset_m and yaw + fault_yaw_deg; none ends an active fault at once
MOCAP_FAULT_TYPES = ("stop", "freeze", "jump", "none")
GUARD_STATES = {0: "MOCAP", 1: "BRIDGING", 2: "CONFIRMED", 3: "LOST"}
GUARD_CAUSES = {0: "none", 1: "stale", 2: "frozen", 3: "jump", 4: "test"}


class Abort(Exception):
    pass


def parse_mocap_fault(spec, default_body):
    """--mocap-fault TYPE@STEP+DELAY[:DURATION][,BODY] -> dict. STEP is a mark this
    driver records: a bare name (go_to_start, push, ...) also matches its ':start' mark."""
    s, body = spec.strip(), default_body
    if "," in s:
        s, body = (x.strip() for x in s.split(",", 1))
    try:
        typ, rest = s.split("@", 1)
        step, tail = rest.rsplit("+", 1)            # a mark name never carries '+'
        delay, _, dur = tail.partition(":")
        typ, step, delay = typ.strip(), step.strip(), float(delay)
        dur = float(dur) if dur.strip() else (0.0 if typ == "jump" else 1.0)
    except ValueError:
        raise SystemExit(f"--mocap-fault {spec!r}: expected TYPE@STEP+DELAY[:DURATION][,BODY], "
                         f"e.g. freeze@push+4:2.5")
    if typ not in MOCAP_FAULT_TYPES:
        raise SystemExit(f"--mocap-fault {spec!r}: TYPE must be one of {', '.join(MOCAP_FAULT_TYPES)}")
    if not step or not body:
        raise SystemExit(f"--mocap-fault {spec!r}: STEP and BODY must not be empty")
    if not (math.isfinite(delay) and delay >= 0.0 and math.isfinite(dur) and 0.0 <= dur <= 1.0e6):
        raise SystemExit(f"--mocap-fault {spec!r}: DELAY must be >= 0 and DURATION in [0, 1e6] s")
    marks = {step} if ":" in step else {step, step + ":start"}
    return {"spec": spec, "type": typ, "step": step, "marks": marks, "delay": delay,
            "duration": dur, "body": body, "state": "pending", "result": "not reached"}


class Driver(Node):
    def __init__(self, a):
        super().__init__("pl_mission")
        self.a = a
        ns = a.namespace.rstrip("/")
        self.t0 = time.time()
        self.events, self.marks = [], []
        self.late = []                       # --timetable: (step, seconds late)
        self.odom = self.armed = None
        self.mode = ""
        self.status = self.pl_status = ""
        self.info = self.arrival = self.grip = self.box = self.claw = None
        self.step = -1
        self.phase = "init"
        self.ref = np.array([0.0, 0.0, a.hover_z])
        self.stream_ref = True
        self.grip_cmd = None
        self.inject = (a.inject.split('@')[0], float(a.inject.split('@')[1])) if a.inject else None
        self.inject_done = False
        if self.inject and self.inject[0] not in ('abort', 'open', 'exit'):
            raise SystemExit(f'--inject {a.inject}: kind must be abort, open or exit')
        self.L = {k: [] for k in ("odom", "dbg", "box", "ee", "ref", "grip", "joints", "arrival", "claw",
                                  "contact", "base_truth", "refd", "cadm", "guard")}
        # --mocap-fault (2026-10-21, the raw-mocap guard's tests): faults started through the
        # Isaac mocap emulator's runtime fault_* parameters; the client exists only when asked
        self.mocap_faults = [parse_mocap_fault(s, ns.strip("/").split("/")[-1] or "uav_0")
                             for s in a.mocap_fault]
        if len(a.mocap_fault_offset) != 3:
            raise SystemExit(f"--mocap-fault-offset needs 3 values [dx, dy, dz], got {a.mocap_fault_offset}")
        # ATOMICALLY: a plain SetParameters applies each value on its own, so a refused fault_body
        # still let fault_type start the fault on the emulator's default body (the drone) -- found
        # in the 2026-10-21 loopback. All or nothing; the emulator checks a body in the same call.
        self.mf_cli = (self.create_client(SetParametersAtomically,
                                          f"{a.emulator_node.rstrip('/')}/set_parameters_atomically")
                       if self.mocap_faults else None)
        self.guard_state = None
        # 2026-10-10: the controller namespace is an option (--da) so the SAME mission
        # flies on the coordinated-admittance rig (Liu et al. CEP 2024): mode topic,
        # set_direct_mode and the debug array (--dbg-topic, --dbg-len) follow it; the
        # planner's reference stream (REF_TOPIC) is rig-independent.
        self.da = a.da
        sub = self.create_subscription
        sub(Odometry, f"{ns}/state_estimator/local_position/odom", self.on_odom, 10)
        sub(String, f"{ns}/{a.da}/mode", lambda m: setattr(self, "mode", m.data), LATCHED)
        sub(Float32MultiArray, f"{ns}/{a.da}/{a.dbg_topic}", self.on_dbg, 50)
        sub(Float64MultiArray, f"{ns}/coordinated_planner/state",
            lambda m: self.L["cadm"].append([self.now(), self.step, *m.data]) if self.mode == "DIRECT" else None, 50)
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
        # ground truth for the metrics (pl_metrics.py): the box's PhysX contacts and the
        # airframe's true pose (Isaac, not the estimate the law flies on)
        sub(Float64MultiArray, "/push_pull_truth/contact",
            lambda m: self.L["contact"].append([self.now(), *m.data, self.step]), 50)
        sub(PoseStamped, f"{ns}/state/pose", self.on_base_truth, qos_profile_sensor_data)
        # the estimator's raw-mocap guard health (2026-10-21; BEST_EFFORT, silent with the guard off)
        sub(Float64MultiArray, f"{ns}/state_estimator/mocap_guard", self.on_guard, qos_profile_sensor_data)
        self.ref_pub = self.create_publisher(PositionControllerReference,
                                             f"{ns}/fsc_autopilot_ros2/position_controller/reference", 10)
        self.grip_pub = self.create_publisher(Float64, f"{ns}/isaacsim_manipulator/gripper_command", 10)
        from rclpy.action import ActionClient
        from control_msgs.action import GripperCommand
        self.grip_ac = ActionClient(self, GripperCommand, a.grip_action)
        self.cli = {n: self.create_client(Trigger, f"{ns}/rc/{n}") for n in ("offboard", "arm", "disarm")}
        for n in ("go_to_start", "ready", "push", "exit", "go_to_land_start", "land", "adjust", "plan",
                  "capture_box", "reset", "abort", "release"):
            self.cli[n] = self.create_client(Trigger, f"{ns}/{PL}/{n}")
        self.direct = self.create_client(SetBool, f"{ns}/{a.da}/set_direct_mode")
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
            self.L["dbg"].append(np.concatenate([[self.now(), self.step], np.asarray(m.data, np.float32)[:self.a.dbg_len]]))

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
        # [11] box lag, [12] box tilt (2026-10-16); after the step, the turn's PATH FOLLOWING
        # (2026-10-18, arrival [13..20]: active, rate, progress, lead, tilt, wrist, limit, held)
        fl = (self.arrival[13:21] + [0.0] * 8)[:8]
        self.L["arrival"].append([self.now(), *self.arrival[:13], self.step, *fl])

    def on_ee(self, m):
        p, q = m.pose.position, m.pose.orientation
        self.L["ee"].append([self.now(), p.x, p.y, p.z, q.w, q.x, q.y, q.z, self.step])

    def on_ref(self, m):
        r = m.r_ed
        # cols 13..18 = x_cd_ddot, b1_de (2026-10-09: the MATLAB export's compatible
        # attitude / EE heading reference); the step tag stays the LAST column
        self.L["ref"].append([self.now(), r.x, r.y, r.z, m.x_cd.x, m.x_cd.y, m.x_cd.z,
                              *list(m.q_d)[:4], m.b1_d.x, m.b1_d.y,
                              m.x_cd_ddot.x, m.x_cd_ddot.y, m.x_cd_ddot.z,
                              m.b1_de.x, m.b1_de.y, m.b1_de.z, self.step])
        v, a = m.r_ed_dot, m.r_ed_ddot
        self.L["refd"].append([self.now(), v.x, v.y, v.z, a.x, a.y, a.z, self.step])

    def on_box(self, m):
        p, q = m.pose.position, m.pose.orientation
        self.box = np.array([p.x, p.y, p.z, q.w, q.x, q.y, q.z])
        self.L["box"].append([self.now(), *self.box, self.step])
        # the body's yaw, unwrapped from the push step's start (the valve's
        # beam angle, 2026-10-15; a pushed box barely turns)
        yaw = np.arctan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        prev = getattr(self, "_box_yaw_prev", None)
        if prev is not None and self.step == STEPS.index("push"):
            self.turned_since_push = getattr(self, "turned_since_push", 0.0) + \
                float(np.arctan2(np.sin(yaw - prev), np.cos(yaw - prev)))
        self._box_yaw_prev = yaw

    def on_base_truth(self, m):
        p, q = m.pose.position, m.pose.orientation
        self.L["base_truth"].append([self.now(), p.x, p.y, p.z, q.w, q.x, q.y, q.z, self.step])

    def on_claw(self, m):
        p = m.pose.position
        self.claw = np.array([p.x, p.y, p.z])
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

    def on_guard(self, m):
        d = list(m.data)
        self.L["guard"].append([self.now(), self.step, *d])
        code = lambda i: int(round(d[i])) if len(d) > i and math.isfinite(d[i]) else -1   # noqa: E731
        st = code(0)
        if st != self.guard_state:            # log the edges only
            age = f", last good sample {d[2]:.2f} s ago" if len(d) > 2 else ""
            self.ev(f"mocap guard: {GUARD_STATES.get(st, st)} (cause {GUARD_CAUSES.get(code(1), code(1))}{age})")
            self.guard_state = st

    def tick(self):
        if self.stream_ref and self.mode != "DIRECT":
            m = PositionControllerReference()
            m.header.stamp = self.get_clock().now().to_msg()
            m.position.x, m.position.y, m.position.z = map(float, self.ref)
            m.yaw, m.yaw_unit = 0.0, PositionControllerReference.DEGREES
            self.ref_pub.publish(m)
        self.inject_tick()
        self.mocap_fault_tick()
        if self.grip_cmd is not None:
            self.grip_pub.publish(Float64(data=float(self.grip_cmd)))

    def mocap_fault_tick(self):
        """--mocap-fault TYPE@STEP+DELAY[:DURATION][,BODY] (2026-10-21, the raw-mocap guard's
        tests): DELAY s after the mark STEP (its latest, as --inject), ONE atomic set_parameters
        call on the Isaac mocap emulator -- fault_body, fault_duration_s, for a jump
        fault_offset_m and fault_yaw_deg, then fault_type, whose set STARTS the fault (the
        emulator applies the whole call before its next publish tick); a refused value refuses
        the whole call, so no fault starts. Non-blocking; each fault fires once."""
        for f in self.mocap_faults:
            if f["state"] != "pending":
                continue
            t_mark = [t for t, n in self.marks if n in f["marks"]]
            if not t_mark or self.now() < t_mark[-1] + f["delay"]:
                continue
            if not self.mf_cli.service_is_ready():
                f.setdefault("t_due", self.now())
                if self.now() - f["t_due"] < 2.0:
                    continue                      # discovery: allow it 2 s past its time
                f["state"], f["result"] = "failed", "NOT INJECTED: the emulator's set_parameters_atomically is not available"
                self.ev(f"MOCAP FAULT {f['spec']}: {f['result']} ({self.a.emulator_node})")
                continue
            ps = [RosParameter("fault_body", RosParameter.Type.STRING, f["body"]),
                  RosParameter("fault_duration_s", RosParameter.Type.DOUBLE, float(f["duration"]))]
            if f["type"] == "jump":
                ps += [RosParameter("fault_offset_m", RosParameter.Type.DOUBLE_ARRAY,
                                    [float(x) for x in self.a.mocap_fault_offset]),
                       RosParameter("fault_yaw_deg", RosParameter.Type.DOUBLE, float(self.a.mocap_fault_yaw_deg))]
            ps.append(RosParameter("fault_type", RosParameter.Type.STRING, f["type"]))   # LAST: it starts it
            req = SetParametersAtomically.Request()
            req.parameters = [p.to_parameter_msg() for p in ps]
            f["state"], f["t_sent"] = "sent", self.now()
            what = (f"{f['type']} on {f['body']}, " +
                    ("one sample" if f["type"] == "jump" and f["duration"] == 0.0 else f"{f['duration']:g} s") +
                    (f" (offset {self.a.mocap_fault_offset} m, yaw {self.a.mocap_fault_yaw_deg:g} deg)"
                     if f["type"] == "jump" else ""))
            self.ev(f"MOCAP FAULT {what}: {self.now() - t_mark[-1]:.2f} s after {f['step']} (phase {self.phase})")
            self.mark(f"mocap_fault_{f['type']}:start")
            fut = self.mf_cli.call_async(req)
            fut.add_done_callback(lambda fu, f=f: self.mocap_fault_done(f, fu))

    def mocap_fault_done(self, f, fut):
        if fut.exception() is not None or fut.result() is None:
            f["state"], f["result"] = "failed", f"call failed: {fut.exception() or 'no response'}"
        else:
            r = fut.result().result
            f["state"] = "accepted" if r.successful else "refused"
            f["result"] = "accepted" if r.successful else f"REFUSED (no fault started): {r.reason}"
        self.ev(f"MOCAP FAULT {f['spec']}: {f['result']}")

    def inject_tick(self):
        """--inject KIND@S (2026-10-16, the box-pushing campaign's worst cases): S seconds
        after the push step starts, once -- abort = the arm GS's ABORT button (the planner's
        abort service; the jaws open on its ABORTING status, as under the GS); open = the
        jaws open while the planner keeps pushing in CONTACT (the grasp lost); exit = the
        Exit To Push button pressed mid-slide."""
        if not self.inject or self.inject_done:
            return
        t0 = [t for t, n in self.marks if n == "push:start"]
        if not t0 or self.now() < t0[-1] + self.inject[1]:
            return
        self.inject_done = True
        kind = self.inject[0]
        self.ev(f"INJECT {kind} at {self.now() - t0[-1]:.2f} s into the push")
        self.mark(f"inject_{kind}:start")
        if kind == "open":
            self.grip_cmd = GRIP_OPEN
        else:
            fut = self.cli[kind].call_async(Trigger.Request())
            fut.add_done_callback(lambda f: self.ev(
                f"INJECT {kind}: {'accepted' if f.result() and f.result().success else 'REFUSED'}: "
                f"{f.result().message if f.result() else 'no response'}"))

    # ---- helpers -----------------------------------------------------------
    def ev(self, s):
        line = f"[{self.now():7.2f}s] {s}"
        self.events.append(line)
        print(line, flush=True)

    def mark(self, name):
        self.marks.append((self.now(), name))
        self.ev(f"MARK {name}")

    def align(self, ref_mark, slot, what):
        """--timetable (2026-10-09): hold until `slot` s after the mark `ref_mark`,
        so the next step starts at the SAME mission time on every run (the
        pick-and-place driver's rule). Only called where hovering, or gripping
        in CONTACT after the push, is safe. A step already past its slot is
        LATE (recorded in the npz; the campaign re-flies such a run)."""
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

    def guard(self, what):
        if self.armed and self.mode != "DIRECT" and self.phase == "mission":
            # the arm GS's rule (2026-10-18): the node left DIRECT (its watchdog) with
            # the claw on the handle -> open the jaws at once; nothing else releases
            # them, and the SAFETY hold plus the arm's fold drag a clamped handle
            if what in ("ready", "close", "push", "push complete", "release", "exit") or \
                    self.pl_status.startswith(("FLYING ready", "FLYING push", "DONE ready")):
                self.grip_cmd = GRIP_OPEN
                self.ev("gripper OPEN (the node left DIRECT with the claw on the handle)")
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
        # (copied 2026-10-22 for the contact-stability tune: the velocity test reads the odometry's
        # velocity LOW-PASSED at 0.5 s -- with the hardware-level raw-mocap noise injected in Isaac,
        # 4.5 cm/s per axis below 5 Hz, the instantaneous |v| < 0.10 m/s never held 3 s; a pilot
        # judges the hover on its drift, not on one sample)
        since, t, v_f, t_prev = None, time.time(), None, None
        while time.time() - t < timeout:
            if self.odom is not None:
                now = time.time()
                v = np.asarray(self.odom[3:6], float)
                if v_f is None:
                    v_f = v.copy()
                else:
                    v_f = v_f + (1.0 - np.exp(-(now - t_prev) / 0.5)) * (v - v_f)
                t_prev = now
                ok = (np.linalg.norm(self.odom[:3] - self.ref) < tol_m and np.linalg.norm(v_f) < tol_v)
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
                  and (at_handle is None or (self.info[I_AT_HANDLE] > 0.5) == at_handle), T + 40, f"{name} complete")

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
        # The climb's reference stream can overlap the switch by a tick, and the
        # planner then holds that setpoint as a PENDING drone target (PLANNED,
        # never executed; cad_02, 2026-10-07). Clear it, as ps4_teleop_bringup does.
        if not self.status.startswith("HOLD"):
            self.ev(f"planner {self.status!r} after the switch -- clearing the stray pending target")
            fut = self.clear.call_async(Trigger.Request())
            t_c = time.time()
            while not fut.done() and time.time() - t_c < 5.0:
                time.sleep(0.05)
        self.wait(lambda: self.status == "HOLD", 15, "planner HOLD")
        self.trig("plan")
        # 2026-10-16: a REFUSED plan (the box-on-table check, a joint limit, the table
        # keep-out) ends the run at once with the planner's reason, not a 60 s timeout
        self.wait(lambda: self.pl_status.startswith(("READY", "INFEASIBLE")), 60, "mission READY")
        if self.pl_status.startswith("INFEASIBLE"):
            raise Abort(f"plan refused: {self.pl_status}")
        self.ev(f"planned step durations {np.round(self.info[8:14], 1).tolist()} s")
        # 1 Go To Start
        self.begin("go_to_start")
        self.step_to_end("go_to_start", 0, at_handle=False)
        self.mark("go_to_start:end")
        if a.gs_auto:
            # 2026-10-21: the ARM GS's AUTOMATIC Ready To Push and Push (run_pl.sh with
            # ARM_GS_AUTO_STEPS=true): after Go To Start the station opens the gripper and
            # descends by itself, closes the first time the claw is in its gate and pushes on
            # the grasp -- exactly the hardware sequence. The driver only watches and marks.
            self.grip_cmd = None
            self.wait(lambda: self.pl_status.startswith("FLYING ready"), 25, "the GS's automatic Ready To Push")
            self.step = STEPS.index("ready")
            self.mark("ready:start")
            self.wait(lambda: self.pl_status.startswith("DONE ready"), 30, "Ready To Push complete")
            self.mark("ready:end")
            moved = np.linalg.norm(self.box[:3] - self.box0[:3])
            if moved > a.max_disturb:
                self.ev(f"WARNING: the box was disturbed {moved * 1e3:.1f} mm by the descent (the GS closes anyway)")
            self.step = STEPS.index("close")
            self.mark("close:start")
            g0 = self.grip
            self.wait(lambda: self.grip is not None and g0 is not None and self.grip < g0 - np.radians(1.0),
                      20, "the GS closing the gripper")
            self.mark("gripper_close:start")
            self.wait(lambda: self.pl_status.startswith("FLYING push"), 15, "the GS's automatic Push")
            self.mark("gripper_close:end")
            self.step = STEPS.index("push")
            self.mark("push:start")
            self.box_at_push = self.box.copy()
            self.turned_since_push = 0.0
            self.wait(lambda: self.pl_status.startswith("DONE push"), 60, "push complete")
            mt = lambda n: [t_ for t_, n_ in self.marks if n_ == n][-1]   # noqa: E731
            self.ev(f"the GS's automatic steps: Ready flew {mt('ready:start') - mt('go_to_start:end'):.2f} s after "
                    f"Go To Start, the jaws closed {mt('gripper_close:start') - mt('ready:end'):.2f} s after Ready")
        else:
            if a.timetable:
                self.align("go_to_start:start", a.slot_ready, "ready")
            # 2 Ready To Push: open, hover (the trim window), then down onto the fin
            self.begin("ready")
            self.gripper_open("open")
            if a.timetable:
                self.align("ready:start", a.slot_descent, "ready descent")
            else:
                time.sleep(a.hover_dwell)
            self.step_to_end("ready", 1, at_handle=True)
            moved = np.linalg.norm(self.box[:3] - self.box0[:3])
            if moved > a.max_disturb:
                raise Abort(f"the box was disturbed {moved * 1e3:.1f} mm by the descent -- not closing")
            self.mark("ready:end")
            if a.timetable:
                self.align("ready:start", a.slot_close, "close")
            # 3 Push: close once the claw is there, push on the stall (the jaws' stall
            # itself is never delayed: a gripped box must be pushed in CONTACT at once)
            self.begin("close")
            t, since = time.time(), None
            while True:
                if a.grip_live_box:
                    # --grip-live-box (2026-10-10, the coordinated-admittance rig): gate on the measured
                    # claw against the LIVE box (obj_0), not the planner's captured grasp point -- a
                    # swaying base whose finger pads brush the fin drags the box along the fin, so the
                    # captured point walks away while the claw is in fact on the fin. Across the fin
                    # (x, the closing axis) within grip_axis_tol, along it (y) within grip_tol.
                    # 2026-10-18: in the BODY's frame (its yaw) minus the grasp point's offset in that
                    # frame (--grip-ee-offset, the planner's push_pull_ee_offset x, y) -- the box sits at
                    # yaw 0 with the grasp point over its origin (the old world-frame test, numerically);
                    # the valve's grasp point is 0.160 m out along the beam in the rotating rotor frame.
                    rel = None
                    if self.claw is not None and self.box is not None:
                        qw, qx, qy, qz = self.box[3:7]
                        yw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
                        dx, dy = self.claw[0] - self.box[0], self.claw[1] - self.box[1]
                        rel = (math.cos(yw) * dx + math.sin(yw) * dy - a.grip_ee_offset[0],
                               -math.sin(yw) * dx + math.cos(yw) * dy - a.grip_ee_offset[1])
                    ok = rel is not None and abs(rel[0]) <= a.grip_axis_tol and abs(rel[1]) <= a.grip_tol
                    if rel is not None:
                        self.arrival = (self.arrival or [0.0] * 10)
                        self.arrival = list(self.arrival)
                        self.arrival[2] = float(abs(rel[1]))
                        self.arrival[9] = float(rel[0])
                else:
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
            self.turned_since_push = 0.0
            self.step_to_end("push", 2, at_handle=True)
        self.mark("push:end")
        d = self.box[:3] - self.box_at_push[:3]
        self.ev(f"push: the box moved [{d[0] * 1e3:+.0f}, {d[1] * 1e3:+.0f}, {d[2] * 1e3:+.0f}] mm, "
                f"turned {np.degrees(self.turned_since_push) if hasattr(self, 'turned_since_push') else 0.0:+.1f} deg "
                f"(the valve's beam, 2026-10-15: obj_0 is the rotor there)")
        if a.gs_exit:
            # 2026-10-21: the ARM GS's AUTOMATIC Exit To Push owns release -> open -> exit
            # (run_pl.sh with ARM_GS_AUTO_EXIT_DELAY > 0), exactly as on hardware; the
            # driver only watches and marks: release:start = the jaws start opening (the
            # GS opens on the planner's release reply, a few ms after it), exit:start =
            # the planner flying the exit.
            self.step = STEPS.index("release")
            # stop holding the close on the bus: the plant keeps its last target, and
            # the GS's own OPEN (through the gripper action server) must be the only
            # command -- held here it fought the open (the first --gs-exit flight)
            self.grip_cmd = None
            g0, t = self.grip, time.time()
            while self.grip is None or g0 is None or abs(self.grip - g0) < np.radians(1.0):
                if time.time() - t > 6.0:
                    raise Abort("the arm GS never started its automatic exit (jaws still closed "
                                "6 s after the push) -- is ARM_GS_AUTO_EXIT_DELAY > 0?")
                self.guard("release")
                time.sleep(0.005)
            self.mark("release:start")
            self.mark("gripper_release:start")
            self.wait(lambda: self.grip is not None and abs(self.grip - GRIP_OPEN) <= np.radians(3.0),
                      5, "the GS's gripper OPEN")
            self.mark("gripper_release:end")
            self.wait(lambda: self.pl_status.startswith("FLYING exit"), 5, "the GS's Exit To Push")
            self.step = STEPS.index("exit")
            self.mark("exit:start")
            self.wait(lambda: self.status == "HOLD" and self.info is not None and int(self.info[5]) == 3
                      and self.info[I_AT_HANDLE] < 0.5, 40, "exit complete")
            self.mark("exit:end")
            mt = lambda n: [t_ for t_, n_ in self.marks if n_ == n][-1]   # noqa: E731
            self.ev(f"the GS's automatic exit: the jaws start opening {mt('release:start') - mt('push:end'):.2f} s "
                    f"after the push ended, the exit flies {mt('exit:start') - mt('push:end'):.2f} s after")
        else:
            if a.timetable:
                self.align("push:start", a.slot_release, "release")
            else:
                time.sleep(a.push_hold)
            # 4 Exit To Push: open, wait for the jaws, exit
            self.begin("release")
            if a.release_at_open:
                # 2026-10-17, the release-window fix -- the arm GS's sequence: the
                # planner RELEASES first (the claw's reference back to the CoM
                # anchor), then the gripper opens, and the exit starts on its OPEN
                # report, not on a slot
                self.trig("release")
            self.gripper_open("release")
            if a.release_at_open:
                if self.grip is None or abs(self.grip - GRIP_OPEN) > np.radians(3.0):
                    raise Abort("the gripper did not report OPEN -- no exit (the jaws may hold the handle)")
            elif a.timetable:
                self.align("release:start", a.slot_exit, "exit")
            else:
                time.sleep(a.release_wait)
            self.begin("exit")
            self.step_to_end("exit", 3, at_handle=False)
            self.mark("exit:end")
        # 5 Go To Land Start: one table-checked transition to Land Start
        if a.timetable:
            self.align("exit:start", a.slot_land, "go_to_land_start")
        else:
            time.sleep(a.leg_pause)
        self.begin("go_to_land_start")
        self.step_to_end("go_to_land_start", 4)
        self.mark("go_to_land_start:end")
        # 6 Land: straight down to the land hover
        if a.timetable:
            self.align("go_to_land_start:start", a.slot_land_step, "land")
        else:
            time.sleep(a.leg_pause)
        self.begin("land")
        self.step_to_end("land", 5)
        self.mark("land:end")
        self.wait(lambda: self.pl_status.startswith("COMPLETE"), 10, "COMPLETE")
        time.sleep(a.post_hold)
        self.phase = "done"

    def land(self):
        self.phase = "land"
        # 2026-10-19: an abort of THIS driver's (no grasp, a refused step, a timeout, too
        # much disturbance) near the table used to land in place -- beside the table, over
        # the box, possibly with the jaws still closed on the fin. Do what the operator
        # does: open the jaws, then the planner's ABORT (release hold, climb, arm home);
        # the branch below then leaves by Go To Land Start.
        info = self.info
        near = info is not None and len(info) > I_AT_HANDLE and (
            info[I_AT_HANDLE] > 0.5 or 0 <= int(info[5]) <= 3 or 0 <= int(info[6]) <= 3)
        if self.mode == "DIRECT" and near and not self.pl_status.startswith(("ABORTING", "ABORTED")):
            self.ev("driver abort near the table: gripper OPEN, then the planner's ABORT")
            try:
                self.gripper_open("abort")
                self.trig("abort", must=False)
                t = time.time()
                while time.time() - t < 5.0 and not self.pl_status.startswith(("ABORTING", "ABORTED")):
                    time.sleep(0.05)
            except Abort as e:
                self.ev(f"the planner's abort could not be requested: {e}")
        # a planner ABORT (release hold, climb, arm home) plays out in DIRECT
        # before the SAFETY handover, as it does under the GS
        if self.mode == "DIRECT" and self.pl_status.startswith("ABORTING"):
            t = time.time()
            while time.time() - t < 25.0 and self.mode == "DIRECT" and \
                    not self.pl_status.startswith("ABORTED"):
                time.sleep(0.05)
            self.ev(f"planner abort finished: {self.pl_status}")
            time.sleep(2.0)
            # 2026-10-16: an abort leaves the vehicle 0.30 m above the push point, BESIDE
            # the table -- descending in place there came down on the table / box (Isaac,
            # h_heavy, 20.6 deg). The operator's procedure, flown here: Get the box, Plan,
            # Go To Land Start (the planner's table-checked path off the table), then land.
            if self.a.after_abort_land_start and self.mode == "DIRECT" and self.pl_status.startswith("ABORTED"):
                try:
                    # the planner keeps Go To Land Start flyable after an abort (2026-10-16);
                    # an older one needs Get + Plan first
                    r = self.trig("go_to_land_start", must=False)
                    if not r.success:
                        self.ev(f"after the abort: Go To Land Start refused ({r.message}) -- Get + Plan first")
                        self.trig("capture_box")
                        time.sleep(1.0)
                        self.trig("plan")
                        t = time.time()
                        while time.time() - t < 60 and not self.pl_status.startswith(("READY", "INFEASIBLE")):
                            time.sleep(0.05)
                        if not self.pl_status.startswith("READY"):
                            raise Abort(f"re-plan after the abort: {self.pl_status}")
                        self.start("go_to_land_start")
                    self.ev("after the abort: Go To Land Start (off the table)")
                    t = time.time()
                    while time.time() - t < 90 and self.mode == "DIRECT" and \
                            not self.pl_status.startswith(("DONE go_to_land_start", "INFEASIBLE", "ABORT")):
                        time.sleep(0.05)
                    self.ev(f"after the abort: {self.pl_status}")
                    time.sleep(2.0)
                except Abort as e:
                    self.ev(f"after the abort, Go To Land Start not flown: {e} -- landing in place")
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
        # --mocap-fault (2026-10-21): what was asked and what the emulator answered; only when asked
        if self.mocap_faults:
            arr["mocap_fault_spec"] = np.array([f["spec"] for f in self.mocap_faults])
            arr["mocap_fault_result"] = np.array([f["result"] for f in self.mocap_faults])
            arr["mocap_fault_t_sent"] = np.array([f.get("t_sent", np.nan) for f in self.mocap_faults])
        np.savez_compressed(path, **arr, events=np.array(self.events),
                            marks_t=np.array([m[0] for m in self.marks]),
                            marks_name=np.array([m[1] for m in self.marks]),
                            info=self.info if self.info is not None else np.zeros(0),
                            box0=getattr(self, "box0", np.zeros(7)),
                            box_final=self.box if self.box is not None else np.zeros(7),
                            joint_names=np.array(getattr(self, "joint_names", [])),
                            timetable=np.array([self.a.slot_ready, self.a.slot_descent, self.a.slot_close,
                                                self.a.slot_release, self.a.slot_exit, self.a.slot_land,
                                                self.a.slot_land_step])
                            if self.a.timetable else np.zeros(0),
                            release_at_open=bool(self.a.release_at_open),
                            late_name=np.array([l[0] for l in self.late]),
                            late_s=np.array([l[1] for l in self.late]),
                            aborted=aborted, reason=reason)
        print(f"saved {path}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--namespace", default="/uav_0")
    ap.add_argument("--da", default=DA,
                    help="the controller's direct-actuation namespace (mode topic, set_direct_mode, debug); "
                         "fsc_autopilot_ros2/coordinated_admittance_direct_actuation for the Liu et al. rig")
    ap.add_argument("--dbg-topic", default="wb_control_debug",
                    help="the debug array under --da (coordinated_control_debug on the Liu et al. rig)")
    ap.add_argument("--dbg-len", type=int, default=120)
    ap.add_argument("--rate", type=float, default=50.0)
    # 1.3 m (2026-10-06): the post-handle push pose holds the body lower over the
    # box; from a 1.0 m hover the planner refuses Go To Start (gear vs table)
    ap.add_argument("--hover-z", type=float, default=1.3)
    # 0.24 (2026-10-06): the T650 gear (AM_T650.usda) rests the body at 0.235 m;
    # with the X650 gear (0.305) pass --land-z 0.30
    ap.add_argument("--land-z", type=float, default=0.24)
    ap.add_argument("--grip-tol", type=float, default=0.02, help="the GS's push_pull_grip_tol [m]")
    ap.add_argument("--grip-axis-tol", type=float, default=0.003, help="the GS's push_pull_grip_axis_tol [m]")
    ap.add_argument("--grip-dwell-in", type=float, default=0.3)
    ap.add_argument("--grip-live-box", action="store_true",
                    help="gate the close on the measured claw vs the LIVE box pose (x across within grip_axis_tol, y along the fin within grip_tol)")
    ap.add_argument("--grip-timeout", type=float, default=30.0)
    ap.add_argument("--grip-ee-offset", type=lambda v: [float(x) for x in v.split(",")], default=[0.0, 0.0],
                    help="with --grip-live-box: the grasp point's x, y in the body's frame [m] (the planner's "
                         "push_pull_ee_offset; the box 0,0, the valve 0.160,0.0006)")
    ap.add_argument("--grip-action", default="/uav_0/fsc_open_manipulator/gripper_controller/gripper_cmd")
    ap.add_argument("--grip-closed-m", type=float, default=-0.010, help="the GS's gripper_closed [m]")
    ap.add_argument("--settle-timeout", type=float, default=45.0)
    ap.add_argument("--hover-dwell", type=float, default=2.0,
                    help="hover above the handle this long before Ready's descent [s]")
    ap.add_argument("--max-disturb", type=float, default=0.008)
    ap.add_argument("--push-hold", type=float, default=1.0, help="hold on the handle after the push [s]")
    ap.add_argument("--release-wait", type=float, default=1.0, help="the GS's push_pull_release_wait_s")
    ap.add_argument("--gs-auto", action="store_true",
                    help="2026-10-21: the arm GS's automatic Ready To Push + Push (and, implied, its "
                         "automatic Exit) run steps 2-4 (ARM_GS_AUTO_STEPS=true); the driver only watches")
    ap.add_argument("--gs-exit", action="store_true",
                    help="2026-10-21: the arm GS's automatic Exit To Push runs release / open / exit "
                         "(start the stack with ARM_GS_AUTO_EXIT_DELAY > 0); the driver only watches")
    ap.add_argument("--direct-settle", type=float, default=15.0)
    ap.add_argument("--leg-pause", type=float, default=2.0)
    ap.add_argument("--post-hold", type=float, default=5.0)
    ap.add_argument("--abort-settle", type=float, default=6.0)
    ap.add_argument("--land-wait", type=float, default=12.0)
    # --timetable (2026-10-09): every step starts at a fixed mission time on every
    # run, for the results campaign (the pick-and-place driver's rule). Slots [s]
    # are offsets from the PREVIOUS step's start mark, sized from the 2026-10-07
    # runs (flown + margin): go_to_start 9.6 s -> ready; open + dwell -> descent 3.0 s
    # -> close; the jaws' stall -> push (never delayed); push 1.5 + 24 s -> release;
    # open -> exit 6 s -> go_to_land_start (23.9 s: one transition) -> land. Only the CLOSE step's own length (the claw's
    # arrival on the fin, 0.3-2 s) varies between runs. A step past its slot is
    # recorded as LATE (npz late_name / late_s).
    ap.add_argument("--timetable", action="store_true",
                    help="start every step at a fixed mission time (replaces the dwell / hold / pause waits)")
    ap.add_argument("--slot-ready", type=float, default=11.0, help="go_to_start start -> Ready (gripper open)")
    ap.add_argument("--slot-descent", type=float, default=2.5, help="Ready start -> its descent onto the fin")
    ap.add_argument("--slot-close", type=float, default=7.0, help="Ready start -> the close step (wait for the claw)")
    ap.add_argument("--slot-release", type=float, default=27.0, help="Push start -> release (gripper open)")
    ap.add_argument("--slot-exit", type=float, default=3.0, help="release start -> Exit To Push")
    ap.add_argument("--slot-land", type=float, default=8.5, help="Exit start -> Go To Land Start")
    ap.add_argument("--slot-land-step", type=float, default=27.0,
                    help="Go To Land Start start -> Land (one transition: 23.9 s planned; was 50.5 for the "
                         "dropped clear leg + arc)")
    # 2026-10-19: the release-first exit is the DEFAULT -- it is the arm GS's Exit To Push
    # since 2026-10-17, and the old open-first order is the sequence that crashed once (q3
    # through 0). --release-at-open is kept (a no-op) for the scripts that pass it.
    ap.add_argument("--release-at-open", dest="release_at_open", action="store_true", default=True,
                    help="(the default) call push_pull/release before opening the gripper and start the "
                         "exit as soon as it is open (the arm GS's Exit To Push); --slot-exit is then unused")
    ap.add_argument("--open-first-exit", dest="release_at_open", action="store_false",
                    help="the pre-2026-10-17 exit: open with the claw still world-held, exit after a slot / "
                         "--release-wait (A/B only)")
    ap.add_argument("--no-after-abort-land-start", dest="after_abort_land_start", action="store_false",
                    help="after a planner ABORT, land in place instead of Get + Plan + Go To Land Start first")
    ap.add_argument("--inject", default="",
                    help="KIND@S: abort / open / exit, S s after the push starts (worst-case tests, 2026-10-16)")
    # 2026-10-21, the raw-mocap guard's tests (../../mocap_safety_guard_20261021)
    ap.add_argument("--mocap-fault", action="append", default=[], metavar="SPEC",
                    help="TYPE@STEP+DELAY[:DURATION][,BODY], repeatable: DELAY s after the mission mark STEP, "
                         "start a mocap fault in the Isaac emulator. TYPE stop | freeze | jump (none ends one). "
                         "STEP a mark this driver records -- go_to_start, ready, close, push, release, exit, "
                         "go_to_land_start, land (= their ':start'), or a full mark such as push:end, direct, "
                         "gripper_close:end, mocap_fault_freeze:start. DURATION [s] default 1.0 (jump 0 = one "
                         "sample). BODY default the vehicle (uav_0). The emulator runs ONE fault at a time: a "
                         "new one ends the active one. E.g. freeze@push+4:2.5, jump@go_to_start+3:0, "
                         "stop@exit+1:0.4")
    ap.add_argument("--mocap-fault-offset", type=lambda v: [float(x) for x in v.split(",")],
                    default=[0.18, 0.0, 0.0], help="a jump's position offset dx,dy,dz [m] (the emulator's "
                                                   "fault_offset_m; default 0.18,0,0)")
    ap.add_argument("--mocap-fault-yaw-deg", type=float, default=110.0,
                    help="a jump's yaw [deg] (the emulator's fault_yaw_deg; default 110)")
    ap.add_argument("--emulator-node", default="/isaacsim_mocap_emulator",
                    help="the Isaac mocap emulator node whose fault_* parameters --mocap-fault sets")
    ap.add_argument("--out", default="pl_run.npz")
    a = ap.parse_args()
    if a.gs_auto:
        a.gs_exit = True          # the GS that ran the push runs its exit too
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
