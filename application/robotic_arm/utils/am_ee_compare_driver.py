#!/usr/bin/env python3
"""am_ee_compare_driver.py -- fly ONE end-effector trajectory on either
aerial-manipulator rig and record everything both controllers can be scored
on (2026-09-18, the whole-body-vs-decoupled comparison).

    /usr/bin/python3 am_ee_compare_driver.py --rig wb|decoupled --shape circle|figure8 \
        [--radius 0.75] [--fig8-a 0.75 --fig8-b 0.375] [--lap-time 30] [--laps 2] \
        [--time-scale 1.0 | --scale 0.8] --out run.npz

Same sequence on both rigs, so the two flights are matched to the second:
SAFETY takeoff to --hover-z -> settle -> DIRECT (the rig's own service) ->
DIRECT settle, then a STABILITY GATE (|v| < gate speed and the base within
gate radius of its own position for gate seconds, after at least
--direct-settle s -- "after the drone takes off and both controllers
stabilize") -> the planner's ee_traj_* parameters are SET (radius, figure-8
half axes, lap time, laps) -> select -> READY -> time scale -> Go-to-start ->
Start gate -> RUN -> hold -> SAFETY -> land -> disarm.

WHAT IS RECORDED (npz), all on the driver's clock:
  log     odometry [t x y z vx vy vz qx qy qz qw direct]
  js      measured joints [t q1 q2 q3 q4] (fsc_open_manipulator/joint_states, by name)
  wbref   the planner's WholeBodyReference stream, converted where needed:
          [t x_cd(3) x_cd_dot(3) x_cd_ddot(3) b1_d(3) r_ed(3) r_ed_dot(3) b1_de(3)
           q_d(4) qdot_d(4) | x_b(3) v_b(3) a_b(3) yaw_ref]   (the last block is the
          decoupled bridge's conversion, computed here for BOTH rigs)
  ee      [t cur_ee(3) ref_ee(3)] from the planner's current_ee / reference_pose
          (the format ee_run_score.py reads)
  dbg     the rig's control debug array [t ...] (wb_control_debug 107 / l1_control_debug 44)
  armref  the arm reference stream actually consumed [t q1..q4 qd1..qd4]
  events, info (planner info at READY), marks {run_start, run_end, direct_enter}
The EE HEADING is not on any topic (current_ee carries no orientation): the
scorer reconstructs it by FK from `js` + `log` with the same model the
planner uses, against b1_de in `wbref`.
"""
import argparse
import math
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, Float64, String
from std_srvs.srv import SetBool, Trigger
from trajectory_msgs.msg import JointTrajectory
from fsc_autopilot_ros2_msgs.msg import PositionControllerReference, WholeBodyReference
from px4_msgs.msg import VehicleStatus

PX4_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE,
                     history=HistoryPolicy.KEEP_LAST, depth=10)
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
EE = "whole_body_planner/ee_trajectory"
RIGS = {
    "wb": dict(da="fsc_autopilot_ros2/whole_body_direct_actuation",
               dbg="fsc_autopilot_ros2/whole_body_direct_actuation/wb_control_debug",
               armref="fsc_open_manipulator/external_torque_controller/reference_joint_trajectory"),
    "decoupled": dict(da="fsc_autopilot_ros2/geometric_l1_direct_actuation",
                      dbg="fsc_autopilot_ros2/geometric_l1_direct_actuation/l1_control_debug",
                      armref="fsc_open_manipulator/position_controller/reference_joint_trajectory"),
}
G = 9.80665
JOINTS = ["joint1", "joint2", "joint3", "joint4"]


def _bridge_helpers():
    """The decoupled bridge's conversion (one implementation, imported)."""
    here = os.path.dirname(os.path.abspath(__file__))
    cand = [os.path.join(os.path.expanduser(os.environ.get("FSC_AUTOPILOT_WS", "~/ros2_ws")),
                         "src", "fsc_autopilot_ros2", "scripts", "tools")]
    for c in cand:
        if os.path.isfile(os.path.join(c, "decoupled_reference_bridge.py")):
            sys.path.insert(0, c)
            break
    import decoupled_reference_bridge as B
    peg = os.path.abspath(os.path.join(here, "..", "..", ".."))
    TP, params = B._load_model(peg)
    return B, TP, params


class Driver(Node):
    def __init__(self, a):
        super().__init__("am_ee_compare_driver")
        self.a = a
        self.rig = RIGS[a.rig]
        ns = a.namespace.rstrip("/")
        self.B, self.TP, self.params = _bridge_helpers()
        self.Rz90 = self.TP._Rz(0.5 * math.pi)
        self.t0 = time.time()
        self.phase = "WAIT"; self.tp = self.t0
        self.done = False; self.aborted = False; self.reason = ""
        self.events = []; self.marks = {}
        self.odom = None; self.mode = ""; self.armed = None
        self.plan_status = ""; self.ee_status = ""; self.info = None; self.start_err = None
        self.ref_pose = None; self.cur_ee = None
        self.log = []; self.js = []; self.wbref = []; self.ee_log = []; self.dbg = []; self.armref = []
        self.settle_since = None; self.gate_since = None; self.gate_anchor = None
        self.ref = np.array([0.0, 0.0, a.hover_z]); self.run_T = 0.0
        self.last_arm = None; self.last_t = None; self.v_arm = np.zeros(3); self.a_arm = np.zeros(3)
        self.params_done = False; self.params_future = None
        self.mode_direct_since = None

        da = self.rig["da"]
        self.create_subscription(Odometry, f"{ns}/state_estimator/local_position/odom", self.on_odom, 10)
        self.create_subscription(String, f"{ns}/{da}/mode", self.on_mode, LATCHED)
        self.create_subscription(VehicleStatus, f"{ns}/fmu/out/vehicle_status_v1",
                                 lambda m: setattr(self, "armed", m.arming_state == 2), PX4_QOS)
        self.create_subscription(String, f"{ns}/whole_body_planner/status",
                                 lambda m: setattr(self, "plan_status", m.data), LATCHED)
        self.create_subscription(String, f"{ns}/{EE}/status", lambda m: setattr(self, "ee_status", m.data), LATCHED)
        self.create_subscription(Float64MultiArrayT, f"{ns}/{EE}/info",
                                 lambda m: setattr(self, "info", list(m.data)), LATCHED)
        self.create_subscription(Float64MultiArrayT, f"{ns}/{EE}/start_error",
                                 lambda m: setattr(self, "start_err", list(m.data)), 10)
        self.create_subscription(PoseStamped, f"{ns}/{EE}/reference_pose", self.on_ref_pose, 10)
        self.create_subscription(PoseStamped, f"{ns}/whole_body_planner/current_ee", self.on_cur_ee, 10)
        self.create_subscription(JointState, f"{ns}/fsc_open_manipulator/joint_states", self.on_js, 10)
        self.create_subscription(WholeBodyReference,
                                 f"{ns}/fsc_autopilot_ros2/whole_body_direct_actuation/reference", self.on_wbref, 10)
        self.create_subscription(Float32MultiArray, f"{ns}/{self.rig['dbg']}", self.on_dbg, 10)
        self.create_subscription(JointTrajectory, f"{ns}/{self.rig['armref']}", self.on_armref, 10)
        self.ref_pub = self.create_publisher(PositionControllerReference,
                                             f"{ns}/fsc_autopilot_ros2/position_controller/reference", 10)
        self.select_pub = self.create_publisher(String, f"{ns}/{EE}/select", 10)
        self.scale_pub = self.create_publisher(Float64, f"{ns}/{EE}/time_scale", 10)
        self.offboard = self.create_client(Trigger, f"{ns}/rc/offboard")
        self.arm = self.create_client(Trigger, f"{ns}/rc/arm")
        self.disarm = self.create_client(Trigger, f"{ns}/rc/disarm")
        self.direct = self.create_client(SetBool, f"{ns}/{da}/set_direct_mode")
        self.go = self.create_client(Trigger, f"{ns}/{EE}/go_to_start")
        self.start = self.create_client(Trigger, f"{ns}/{EE}/start")
        self.params_cli = self.create_client(SetParameters, f"{ns}/whole_body_trajectory_planner/set_parameters")
        self.futs = []
        self.create_timer(1.0 / a.rate, self.tick)

    # ---- callbacks
    def now(self):
        return time.time() - self.t0

    def on_mode(self, m):
        new = m.data
        if new != self.mode:
            self.ev(f"mode -> {new}")
            if new == "DIRECT":
                self.mode_direct_since = time.time()
                self.marks.setdefault("direct_enter", self.now())
            elif self.mode == "DIRECT" and self.phase in (
                    "SELECT", "RESCALE", "GO_TO_START", "START", "RUN", "POST_HOLD",
                    "DIRECT_SETTLE", "SET_PARAMS"):
                # THE NODE REVERTED ON ITS OWN -- a watchdog trip, not our abort.
                # Freeze the SAFETY reference stream at the pose of that instant,
                # which is what an operator who is not touching the station leaves
                # behind and what the node's own re-seed already holds: anything
                # else would command a move and mask what the guard does.
                self.marks["guard_trip"] = self.now()
                if self.odom is not None:
                    self.ref = np.array(self.odom[:3])
                self.ev(f"!! NODE REVERTED TO {new} DURING {self.phase} -- guard trip; "
                        f"freezing the reference at {np.round(self.ref, 3).tolist()} "
                        f"and recording for {self.a.guard_watch} s")
                self.goto("GUARD_WATCH")
        self.mode = new

    def on_odom(self, m):
        p, v, q = m.pose.pose.position, m.twist.twist.linear, m.pose.pose.orientation
        self.odom = np.array([p.x, p.y, p.z, v.x, v.y, v.z])
        self.log.append([self.now(), p.x, p.y, p.z, v.x, v.y, v.z, q.x, q.y, q.z, q.w,
                         1.0 if self.mode == "DIRECT" else 0.0])

    def on_js(self, m):
        q = [float("nan")] * 4
        names = list(m.name)
        for j, nm in enumerate(JOINTS):
            if nm in names:
                k = names.index(nm)
                if k < len(m.position):
                    q[j] = m.position[k]
        self.js.append([self.now(), *q])

    def on_ref_pose(self, m):
        self.ref_pose = np.array([self.now(), m.pose.position.x, m.pose.position.y, m.pose.position.z])

    def on_cur_ee(self, m):
        self.cur_ee = np.array([self.now(), m.pose.position.x, m.pose.position.y, m.pose.position.z])
        if self.ref_pose is not None and self.phase == "RUN":
            self.ee_log.append([self.now(), *self.cur_ee[1:], *self.ref_pose[1:]])

    def on_dbg(self, m):
        self.dbg.append([self.now(), *m.data])

    def on_armref(self, m):
        if not m.points:
            return
        pt = m.points[0]
        q = [float("nan")] * 4; qd = [float("nan")] * 4
        for j, nm in enumerate(JOINTS):
            if nm in m.joint_names:
                k = m.joint_names.index(nm)
                if k < len(pt.positions):
                    q[j] = pt.positions[k]
                if k < len(pt.velocities):
                    qd[j] = pt.velocities[k]
        self.armref.append([self.now(), *q, *qd])

    def on_wbref(self, m):
        x_cd = np.array([m.x_cd.x, m.x_cd.y, m.x_cd.z]); x_cd_dot = np.array([m.x_cd_dot.x, m.x_cd_dot.y, m.x_cd_dot.z])
        x_cd_ddot = np.array([m.x_cd_ddot.x, m.x_cd_ddot.y, m.x_cd_ddot.z])
        b1d = np.array([m.b1_d.x, m.b1_d.y, m.b1_d.z]); r_ed = np.array([m.r_ed.x, m.r_ed.y, m.r_ed.z])
        r_ed_dot = np.array([m.r_ed_dot.x, m.r_ed_dot.y, m.r_ed_dot.z]); b1de = np.array([m.b1_de.x, m.b1_de.y, m.b1_de.z])
        q = np.array(m.q_d, float); qd = np.array(m.qdot_d, float)
        try:
            R0 = self.B.build_r0(x_cd_ddot + np.array([0, 0, G]), b1d)
        except ValueError:
            return
        r0c, _, _ = self.TP.arm_fk_model(q, self.params)
        arm_term = R0 @ r0c
        x_b = x_cd - arm_term
        yaw = math.atan2(b1d[1], b1d[0]) + 0.5 * math.pi
        t = time.time()
        if self.last_t is not None:
            dt = t - self.last_t
            if 1e-4 < dt < 0.5:
                v_raw = (arm_term - self.last_arm) / dt
                a_raw = (v_raw - self.v_arm) / dt
                self.v_arm += min(1.0, dt * 2 * math.pi * 8.0) * (v_raw - self.v_arm)
                self.a_arm += min(1.0, dt * 2 * math.pi * 4.0) * (a_raw - self.a_arm)
        self.last_t = t; self.last_arm = arm_term
        self.wbref.append([self.now(), *x_cd, *x_cd_dot, *x_cd_ddot, *b1d, *r_ed, *r_ed_dot, *b1de, *q, *qd,
                           *x_b, *(x_cd_dot - self.v_arm), *(x_cd_ddot - self.a_arm), yaw])

    # ---- helpers
    def ev(self, s):
        line = f"[{self.now():7.2f}s] {s}"
        self.events.append(line); print(line, flush=True)

    def goto(self, ph):
        self.phase = ph; self.tp = time.time(); self.settle_since = None
        self.ev(f"PHASE {ph}  mode={self.mode} plan={self.plan_status} ee={self.ee_status}")

    def call(self, cli, req, name):
        if not cli.service_is_ready():
            self.ev(f"service {name} not ready"); return
        fut = cli.call_async(req)
        fut.add_done_callback(lambda f: self.ev(f"service {name} -> {f.result()}"))
        self.futs.append(fut)

    def send_ref(self):
        m = PositionControllerReference()
        m.header.stamp = self.get_clock().now().to_msg()
        m.position.x, m.position.y, m.position.z = map(float, self.ref)
        m.yaw, m.yaw_unit = float(self.a.yaw_deg), PositionControllerReference.DEGREES
        self.ref_pub.publish(m)

    def settled(self, tol_m=0.08, tol_v=0.10, hold=3.0):
        if self.odom is None:
            return False
        err = np.linalg.norm(self.odom[:3] - self.ref); spd = np.linalg.norm(self.odom[3:6])
        if err < tol_m and spd < tol_v:
            if self.settle_since is None:
                self.settle_since = time.time()
            return time.time() - self.settle_since > hold
        self.settle_since = None
        return False

    def stable(self):
        """The comparison's stability gate: quiet and not drifting."""
        a = self.a
        if self.odom is None:
            return False
        spd = np.linalg.norm(self.odom[3:6])
        if spd < a.gate_speed:
            if self.gate_since is None:
                self.gate_since = time.time(); self.gate_anchor = self.odom[:3].copy()
            drift = np.linalg.norm(self.odom[:3] - self.gate_anchor)
            if drift > a.gate_radius:
                self.gate_since = time.time(); self.gate_anchor = self.odom[:3].copy()
                return False
            return time.time() - self.gate_since > a.gate_time
        self.gate_since = None
        return False

    def set_planner_params(self):
        a = self.a
        req = SetParameters.Request()
        def dbl(name, v):
            p = Parameter(); p.name = name
            p.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(v)); return p
        def integ(name, v):
            p = Parameter(); p.name = name
            p.value = ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=int(v)); return p
        req.parameters = [dbl("ee_traj_circle_radius", a.radius), dbl("ee_traj_fig8_a", a.fig8_a),
                          dbl("ee_traj_fig8_b", a.fig8_b), dbl("ee_traj_lap_time", a.lap_time),
                          integ("ee_traj_laps", a.laps)]
        # The q2 sinusoid must complete whole cycles over laps*lap_time or the
        # planner refuses ("run does not start/end at rest" -- measured on the
        # 32 s-lap circle against the yaml's 48 s period). Default: one cycle.
        q2p = a.q2_period if a.q2_period is not None else a.laps * a.lap_time
        req.parameters.append(dbl("ee_traj_q2_period_s", q2p))
        if not self.params_cli.service_is_ready():
            return False
        self.params_future = self.params_cli.call_async(req)
        return True

    def abort(self, why):
        self.aborted = True; self.reason = why
        self.ev("ABORT: " + why)
        self.call(self.direct, SetBool.Request(data=False), "set_direct_mode(false)")
        self.goto("LAND")

    # ---- the state machine
    def tick(self):
        a = self.a
        el = time.time() - self.tp
        if self.mode != "DIRECT":
            self.send_ref()
        if self.odom is not None and self.mode == "DIRECT" and self.phase not in ("LAND", "DONE"):
            q = None
            if self.log:
                qx, qy, qz, qw = self.log[-1][7:11]
                tilt = math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (qx * qx + qy * qy)))))
                if tilt > a.abort_tilt:
                    self.abort(f"tilt {tilt:.1f} deg"); return
        if self.phase == "WAIT":
            if self.odom is not None and self.mode and self.armed is not None:
                if self.armed:
                    self.ev("vehicle already ARMED -- refusing (clean relaunch needed)"); self.done = True; return
                self.ref = np.array([self.odom[0], self.odom[1], self.odom[2]]); self.goto("OFFBOARD")
        elif self.phase == "OFFBOARD":
            if el > 1.0:
                self.call(self.offboard, Trigger.Request(), "rc/offboard"); self.goto("ARM")
        elif self.phase == "ARM":
            if el > 3.0:
                self.call(self.arm, Trigger.Request(), "rc/arm"); self.goto("CLIMB")
        elif self.phase == "CLIMB":
            if el > 2.0:
                self.ref = np.array([self.odom[0], self.odom[1], a.hover_z]); self.goto("TAKEOFF")
        elif self.phase == "TAKEOFF":
            if self.settled():
                self.goto("ENTER_DIRECT")
            elif el > 60.0:
                self.abort("takeoff did not settle")
        elif self.phase == "ENTER_DIRECT":
            self.call(self.direct, SetBool.Request(data=True), "set_direct_mode(true)"); self.goto("CONFIRM")
        elif self.phase == "CONFIRM":
            if self.mode == "DIRECT":
                self.goto("DIRECT_SETTLE")
            elif el > 5.0:
                self.abort("DIRECT was refused")
        elif self.phase == "DIRECT_SETTLE":
            if el > a.direct_settle and self.stable():
                self.ev(f"STABLE after {el:.1f} s in DIRECT (|v|<{a.gate_speed} m/s, drift<{a.gate_radius} m "
                        f"for {a.gate_time} s) -- setting the planner's shape parameters")
                if self.set_planner_params():
                    self.goto("SET_PARAMS")
                else:
                    self.abort("planner parameter service not ready")
            elif el > a.direct_settle + 90.0:
                self.abort("never stable in DIRECT")
        elif self.phase == "SET_PARAMS":
            if self.params_future is not None and self.params_future.done():
                res = self.params_future.result()
                ok = all(r.successful for r in res.results)
                self.ev(f"planner params set: {[(r.successful, r.reason) for r in res.results]}")
                if not ok:
                    self.abort("planner refused a parameter"); return
                self.select_pub.publish(String(data=a.shape)); self.goto("SELECT")
            elif el > 10.0:
                self.abort("set_parameters timed out")
        elif self.phase == "SELECT":
            if self.ee_status.startswith("READY") and self.info:
                s_max = self.info[1]
                s = max(0.05, min(a.time_scale if a.time_scale is not None else a.scale * s_max, s_max))
                self.ev(f"READY: s_max {s_max:.3f} -> requesting s = {s:.3f}")
                self.req_s = s
                self.scale_pub.publish(Float64(data=s)); self.ee_status = "PENDING_RESCALE"; self.goto("RESCALE")
            elif self.ee_status.startswith("INFEASIBLE") or el > 60.0:
                self.abort(f"trajectory not READY: {self.ee_status}")
        elif self.phase == "RESCALE":
            if self.ee_status.startswith("READY") and self.info and abs(self.info[0] - self.req_s) < 1e-3:
                self.run_T = self.info[2]
                self.ev(f"{self.ee_status}  T_total {self.run_T:.1f}s lap {self.info[3]:.1f}s")
                self.call(self.go, Trigger.Request(), "ee_trajectory/go_to_start"); self.goto("GO_TO_START")
            elif el > 60.0:
                self.abort(f"rescale did not settle: {self.ee_status}")
        elif self.phase == "GO_TO_START":
            if el > 2.0 and self.plan_status == "HOLD" and self.start_err and self.start_err[3] > 0.5:
                if el > a.start_hold and self.stable():
                    self.call(self.start, Trigger.Request(), "ee_trajectory/start"); self.goto("START")
            elif el > 120.0:
                self.abort(f"never at start: plan={self.plan_status} err={self.start_err}")
        elif self.phase == "START":
            if self.plan_status.startswith("EXECUTING"):
                self.marks["run_start"] = self.now(); self.goto("RUN")
            elif el > 10.0:
                self.abort(f"start did not begin: plan={self.plan_status}")
        elif self.phase == "RUN":
            # OPERATOR ABORT IN THE MIDDLE OF THE TRACKED TRAJECTORY: the drone
            # station's "Return to baseline" during the demo. Same switchMode
            # path a watchdog trip takes, so it measures what the guard leaves
            # behind without having to provoke a real divergence.
            if a.abort_mid_run > 0.0 and el > a.abort_mid_run and "guard_trip" not in self.marks:
                self.ev(f"commanding SAFETY {el:.1f} s into the tracked trajectory")
                self.call(self.direct, SetBool.Request(data=False), "set_direct_mode(false)")
            if self.plan_status == "HOLD" and el > 5.0:
                self.marks["run_end"] = self.now(); self.ev("run complete"); self.goto("POST_HOLD")
            elif el > self.run_T + 60.0:
                self.abort("run overran")
        elif self.phase == "GUARD_WATCH":
            # Record what the guard leaves behind: does it HOLD or descend, does
            # the arm fold, does the planner stop. No command but the frozen
            # reference above.
            if el > a.guard_watch:
                self.marks["guard_watch_end"] = self.now()
                self.goto("LAND")
        elif self.phase == "POST_HOLD":
            if el > a.post_hold:
                self.ev("mission complete -> SAFETY")
                self.call(self.direct, SetBool.Request(data=False), "set_direct_mode(false)"); self.goto("ABORT_SETTLE")
        elif self.phase == "ABORT_SETTLE":
            if el > 1.0 and self.odom is not None and self.mode != "DIRECT":
                if el < 1.5:
                    self.ref = np.array([self.odom[0], self.odom[1], self.odom[2]])
                if el > a.abort_settle:
                    self.goto("LAND")
        elif self.phase == "LAND":
            if self.mode == "DIRECT":
                return
            if el < 0.2 and self.odom is not None:
                self.ref = np.array([self.odom[0], self.odom[1], self.odom[2]])
            self.ref[2] = max(a.land_z, self.ref[2] - 0.2 / a.rate)
            if self.ref[2] <= a.land_z + 1e-6 and el > a.land_wait:
                self.call(self.disarm, Trigger.Request(), "rc/disarm"); self.goto("DONE")
        elif self.phase == "DONE":
            if el > 2.0:
                self.done = True

    def save(self, path):
        arr = lambda x, w: np.array(x) if x else np.zeros((0, w))
        np.savez(path, log=arr(self.log, 12), js=arr(self.js, 5), wbref=arr(self.wbref, 40),
                 ee=arr(self.ee_log, 7), dbg=np.array(self.dbg, dtype=object) if self.dbg else np.zeros((0, 1)),
                 armref=arr(self.armref, 9), events=np.array(self.events), info=np.array(self.info or []),
                 marks=np.array([f"{k}={v}" for k, v in self.marks.items()]),
                 rig=self.a.rig, shape=self.a.shape, radius=self.a.radius, fig8_a=self.a.fig8_a,
                 fig8_b=self.a.fig8_b, lap_time=self.a.lap_time, laps=self.a.laps,
                 time_scale=getattr(self, "req_s", 0.0), aborted=self.aborted, reason=self.reason)
        ee = arr(self.ee_log, 7)
        if ee.shape[0] > 10:
            err = np.linalg.norm(ee[:, 1:4] - ee[:, 4:7], axis=1)
            print(f"EE tracking during the run: {ee.shape[0]} samples, mean {err.mean()*1e3:.1f} mm, "
                  f"p95 {np.percentile(err, 95)*1e3:.1f} mm, max {err.max()*1e3:.1f} mm")


from std_msgs.msg import Float64MultiArray as Float64MultiArrayT  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--rig", required=True, choices=sorted(RIGS))
    ap.add_argument("--namespace", default="/uav_0")
    ap.add_argument("--rate", type=float, default=50.0)
    ap.add_argument("--hover-z", type=float, default=1.2)
    ap.add_argument("--yaw-deg", type=float, default=0.0,
                    help="SAFETY takeoff heading [deg]. The figure-8's LONG axis lies along the "
                         "nose at Start, so 0 puts it on world x (the arm's axis) and 90 on world y.")
    ap.add_argument("--land-z", type=float, default=0.35)
    ap.add_argument("--shape", default="circle", choices=["circle", "figure8"])
    ap.add_argument("--radius", type=float, default=0.75)
    ap.add_argument("--fig8-a", type=float, default=0.75, help="figure-8 half-length along the heading [m]")
    ap.add_argument("--fig8-b", type=float, default=0.375, help="figure-8 half-width [m]")
    ap.add_argument("--lap-time", type=float, default=30.0, help="one lap at time scale 1 [s]")
    ap.add_argument("--laps", type=int, default=2)
    ap.add_argument("--q2-period", type=float, default=None, help="ee_traj_q2_period_s (must divide laps*lap_time)")
    ap.add_argument("--time-scale", type=float, default=None, help="absolute time scale (clamped to s_max)")
    ap.add_argument("--scale", type=float, default=1.0, help="fraction of s_max when --time-scale is not given")
    ap.add_argument("--direct-settle", type=float, default=20.0, help="minimum seconds in DIRECT before the gate")
    ap.add_argument("--gate-speed", type=float, default=0.03)
    ap.add_argument("--gate-radius", type=float, default=0.03)
    ap.add_argument("--gate-time", type=float, default=4.0)
    ap.add_argument("--start-hold", type=float, default=6.0, help="seconds at the start rest before Start")
    ap.add_argument("--post-hold", type=float, default=8.0)
    ap.add_argument("--abort-settle", type=float, default=12.0)
    ap.add_argument("--land-wait", type=float, default=20.0)
    ap.add_argument("--abort-tilt", type=float, default=35.0)
    ap.add_argument("--abort-mid-run", type=float, default=0.0,
                    help="command SAFETY this many seconds into the tracked "
                         "trajectory (0 = never). The recording then follows "
                         "the same GUARD_WATCH path as a watchdog trip.")
    ap.add_argument("--guard-watch", type=float, default=25.0,
                    help="seconds to record after the NODE reverts on its own "
                         "(watchdog trip), before landing")
    ap.add_argument("--out", default="am_ee_compare_run.npz")
    a = ap.parse_args()
    rclpy.init()
    d = Driver(a)
    try:
        while rclpy.ok() and not d.done:
            rclpy.spin_once(d, timeout_sec=0.1)
    except KeyboardInterrupt:
        d.ev("interrupted")
    finally:
        d.save(a.out)
        print(f"saved {a.out} aborted={d.aborted} {d.reason}")
        d.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
