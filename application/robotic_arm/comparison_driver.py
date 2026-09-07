#!/usr/bin/env python3
"""Fly ONE comparison task on ONE of the two controllers, and record it.

    /usr/bin/python3 comparison_driver.py --controller wb --task hover_arm_swing
    /usr/bin/python3 comparison_driver.py --controller l1 --task hover_arm_swing

Same task module, same table, same mission state machine, same abort envelope,
same log format -- the ONLY thing the `--controller` flag changes is which
reference topics the identical numbers are published on, and which debug array
is recorded.  That is the whole point: if the two runs differ, it is the
control design differing, not the command.

WHAT EACH STACK IS TOLD
-----------------------
  both  `PositionControllerReference` = the base path x_b(t) with its velocity
        and acceleration feedforward and the yaw psi(t).  In SAFETY this flies
        the takeoff and landing; in DIRECT the WB node keeps using it for its
        entry gate and its SAFETY-abort target, and the L1 node tracks it.

  l1    the joint path q(t) as a servo POSITION command on the Isaac arm bus
        (`isaacsim_manipulator/position_commands`), which 05's Dynamixel
        emulation tracks with its PD + gravity-comp law.  The arm's reaction on
        the airframe is then a disturbance for the L1 augmentation to reject.

  wb    the same joint path as the arm reference the node's hold and entry gate
        track, PLUS the full `WholeBodyReference` (CoM chain through snap, base
        heading, EE position/heading chains, q_d/qdot_d) streamed at 100 Hz in
        DIRECT, which is the coupled law's actual reference.  The node commands
        the arm's TORQUE itself.

Note the deliberate asymmetry in the arm's ACTUATION -- position servo vs
direct torque.  It is not a fairness defect, it is the difference between the
two designs: the whole-body law requires the arm in torque (current) mode,
the geometric+L1 law is built to fly with a position-mode servo arm.  The
commanded joint motion is identical; how each stack realises it is the thing
under test.  Both runs log the MEASURED joints so the realised arm motion can
be compared too.

The Isaac plant, the airframe/motor model, the rotor lag and the arm model are
identical between the two runs by construction (05 and 06 spawn the same
AM_xfwd asset with the same t650_params), and the two comparison yamls are
diffed at startup by the launcher.
"""

import argparse
import os
import sys
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float32MultiArray, String
from std_srvs.srv import SetBool, Trigger

from fsc_autopilot_ros2_msgs.msg import (PositionControllerReference,
                                         WholeBodyReference)
from px4_msgs.msg import VehicleStatus

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from utils_comparison import comparison_tasks as CTK  # noqa: E402

PX4_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE,
                     history=HistoryPolicy.KEEP_LAST, depth=10)
LATCH = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                   durability=DurabilityPolicy.TRANSIENT_LOCAL,
                   history=HistoryPolicy.KEEP_LAST, depth=1)

ARM_JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4"]

CONTROLLERS = {
    "wb": {
        "label": "whole-body direct actuation",
        "da_ns": "fsc_autopilot_ros2/whole_body_direct_actuation",
        "debug": "wb_control_debug",
        "node": "autopilot_whole_body_direct_actuation_node",
    },
    #: The whole-body law again, with the L1 adaptive augmented disturbance
    #: observer instead of the GMO.  Same node namespace, same debug topic,
    #: same command path -- only the executable and the yaml differ, so
    #: `wb` vs `wb_l1` isolates the ESTIMATOR the way `wb` vs `l1` isolates
    #: the control design.  Its debug array carries 33 extra fields (the
    #: estimator's internals) that the `wb` run publishes as zeros.
    "wb_l1": {
        "label": "whole-body direct actuation + L1 adaptive observer",
        "da_ns": "fsc_autopilot_ros2/whole_body_direct_actuation",
        "debug": "wb_control_debug",
        "node": "autopilot_whole_body_l1_direct_actuation_node",
    },
    "l1": {
        "label": "geometric + L1 adaptive direct actuation",
        "da_ns": "fsc_autopilot_ros2/geometric_l1_direct_actuation",
        "debug": "l1_control_debug",
        "node": "autopilot_geometric_l1_direct_actuation_node",
    },
}

#: Topic the comparison yamls point `wb_arm_reference_topic` at.  In the
#: production stack this is the ExternalTorqueController's smoothed reference;
#: here the driver owns the arm reference on BOTH sides, so it publishes the
#: same numbers here that it puts on the servo bus for the L1 run.
WB_ARM_REF_TOPIC = "comparison/arm_reference"
#: Isaac's servo bus (05).  Only the L1 run writes it; 06 does not subscribe.
ISAAC_ARM_CMD_TOPIC = "isaacsim_manipulator/position_commands"
ISAAC_ARM_STATE_TOPIC = "isaacsim_manipulator/joint_states"


def quat_to_tilt_deg(qw, qx, qy, qz):
    r33 = 1.0 - 2.0 * (qx * qx + qy * qy)
    return float(np.degrees(np.arccos(np.clip(r33, -1.0, 1.0))))


def yaw_from_quat(qw, qx, qy, qz):
    return float(np.arctan2(2.0 * (qw * qz + qx * qy),
                            1.0 - 2.0 * (qy * qy + qz * qz)))


def _v3(a):
    m = Vector3()
    m.x, m.y, m.z = float(a[0]), float(a[1]), float(a[2])
    return m


class ComparisonDriver(Node):

    def __init__(self, a, table):
        super().__init__("am_comparison_driver")
        self.a = a
        self.tb = table
        self.cfg = CONTROLLERS[a.controller]
        ns = a.namespace.rstrip("/")
        da = f"{ns}/{self.cfg['da_ns']}"

        self.pub_ref = self.create_publisher(
            PositionControllerReference,
            f"{ns}/fsc_autopilot_ros2/position_controller/reference", 10)
        # JointTrajectory since 2026-09-05: the whole-body node takes its arm
        # reference from the planner that owns it, not from the controller's
        # echo, and the planners publish JointTrajectory.
        self.pub_arm_ref = self.create_publisher(
            JointTrajectory, f"{ns}/{WB_ARM_REF_TOPIC}", 10)
        self.pub_arm_cmd = self.create_publisher(
            JointState, f"{ns}/{ISAAC_ARM_CMD_TOPIC}", 10)
        self.pub_wb = self.create_publisher(
            WholeBodyReference, f"{da}/reference", 10)

        self.create_subscription(
            Odometry, f"{ns}/state_estimator/local_position/odom",
            self.on_odom, 10)
        self.create_subscription(
            JointState, f"{ns}/{ISAAC_ARM_STATE_TOPIC}", self.on_joints, 10)
        self.create_subscription(
            Float32MultiArray, f"{da}/{self.cfg['debug']}", self.on_dbg, 10)
        self.create_subscription(String, f"{da}/mode", self.on_mode, LATCH)
        self.create_subscription(
            VehicleStatus, f"{ns}/fmu/out/vehicle_status_v1",
            self.on_status, PX4_QOS)

        self.cli_off = self.create_client(Trigger, f"{ns}/rc/offboard")
        self.cli_arm = self.create_client(Trigger, f"{ns}/rc/arm")
        self.cli_dis = self.create_client(Trigger, f"{ns}/rc/disarm")
        self.cli_dir = self.create_client(SetBool, f"{da}/set_direct_mode")

        self.odom = None
        self.q_meas = None
        self.qd_meas = None
        self.tau_meas = None
        self.mode = ""
        self.dbg = None
        self.armed = None

        self.log = []
        self.dbg_log = []
        self.events = []
        self.phase = "WAIT"
        self.t0 = time.time()
        self.tp = self.t0
        self.task_t0 = None
        self.settle_since = None
        self.aborted = False
        self.abort_reason = ""
        self.done = False
        self.futs = []
        self.dis_tries = 0

        # what is being commanded right now
        self.ref_p = np.array(table["base_home"], float)
        self.ref_v = np.zeros(3)
        self.ref_a = np.zeros(3)
        self.ref_psi = float(table["psi_home"])
        self.q_ref = np.array(table["q_home"], float)
        self.qd_ref = np.zeros(4)
        self.k_task = -1

        self.create_timer(1.0 / a.rate, self.tick)

    # ---- callbacks -----------------------------------------------------
    def on_odom(self, m):
        p, o = m.pose.pose.position, m.pose.pose.orientation
        v = m.twist.twist.linear
        self.odom = (p.x, p.y, p.z, v.x, v.y, v.z,
                     quat_to_tilt_deg(o.w, o.x, o.y, o.z),
                     yaw_from_quat(o.w, o.x, o.y, o.z),
                     o.w, o.x, o.y, o.z)

    def on_joints(self, m):
        q = np.full(4, np.nan)
        qd = np.full(4, np.nan)
        tau = np.full(4, np.nan)
        names = list(m.name)
        for j, nm in enumerate(ARM_JOINT_NAMES):
            if nm not in names:
                return
            k = names.index(nm)
            if k < len(m.position):
                q[j] = m.position[k]
            if k < len(m.velocity):
                qd[j] = m.velocity[k]
            if k < len(m.effort):
                tau[j] = m.effort[k]
        self.q_meas, self.qd_meas, self.tau_meas = q, qd, tau

    def on_dbg(self, m):
        self.dbg = np.asarray(m.data, dtype=np.float64)
        if self.phase not in ("WAIT", "OFFBOARD", "ARM"):
            self.dbg_log.append(
                np.concatenate(([time.time() - self.t0], self.dbg)))

    def on_mode(self, m):
        if m.data != self.mode:
            self.ev(f"mode -> {m.data}")
        self.mode = m.data

    def on_status(self, m):
        self.armed = (m.arming_state == 2)

    # ---- helpers -------------------------------------------------------
    def now(self):
        return time.time() - self.t0

    def in_phase(self):
        return time.time() - self.tp

    def ev(self, s):
        line = f"[{self.now():7.2f}s] {s}"
        print(line, flush=True)
        self.events.append(line)

    def goto(self, ph):
        self.tp = time.time()
        self.phase = ph
        self.ev(f"PHASE {ph}  ref={np.round(self.ref_p,3).tolist()} "
                f"q_ref={np.round(np.degrees(self.q_ref),1).tolist()} deg "
                f"mode='{self.mode}'")

    def call(self, cli, req, name):
        # Fire-and-forget: tick() runs inside the executor callback, so
        # spinning for the future from here would deadlock the future it is
        # waiting on.  Results are reported from _drain().
        if not cli.service_is_ready():
            self.ev(f"SERVICE NOT READY (sent anyway): {name}")
        self.futs.append((name, cli.call_async(req), time.time()))
        self.ev(f"service {name} requested")

    def _drain(self):
        keep = []
        for name, fut, t in self.futs:
            if fut.done():
                try:
                    self.ev(f"service {name} -> {fut.result()}")
                except Exception as e:                        # noqa: BLE001
                    self.ev(f"service {name} FAILED: {e}")
            elif time.time() - t > 10.0:
                self.ev(f"service {name} NO RESPONSE after 10 s")
            else:
                keep.append((name, fut, t))
        self.futs = keep

    def is_direct(self):
        m = self.mode.lower()
        return "direct" in m and "safety" not in m

    # ---- reference publication ----------------------------------------
    def send_base_ref(self):
        m = PositionControllerReference()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "map"
        m.position.x, m.position.y, m.position.z = map(float, self.ref_p)
        m.velocity.x, m.velocity.y, m.velocity.z = map(float, self.ref_v)
        m.acceleration.x, m.acceleration.y, m.acceleration.z = \
            map(float, self.ref_a)
        m.yaw = float(self.ref_psi)
        m.yaw_unit = PositionControllerReference.RADIANS
        self.pub_ref.publish(m)

    def send_arm_ref(self):
        m = JointState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.name = list(ARM_JOINT_NAMES)
        m.position = [float(v) for v in self.q_ref]
        m.velocity = [float(v) for v in self.qd_ref]
        m.effort = [0.0] * 4
        # The WB node's hold and DIRECT-entry gate track this; the L1 run's
        # servo bus takes the same numbers.  Publishing both on both runs
        # would be harmless but is avoided so each run has exactly one arm
        # command path.  The two paths need different message types since
        # 2026-09-05: the WB node's arm reference is a planner's
        # JointTrajectory, while the L1 servo bus is still a JointState.
        if self.a.controller.startswith("wb"):
            tj = JointTrajectory()
            tj.header = m.header
            tj.joint_names = list(ARM_JOINT_NAMES)
            pt = JointTrajectoryPoint()
            pt.positions = list(m.position)
            pt.velocities = list(m.velocity)
            pt.accelerations = [0.0] * 4
            tj.points.append(pt)
            self.pub_arm_ref.publish(tj)
        else:
            self.pub_arm_cmd.publish(m)

    def send_wb_ref(self, k):
        t = self.tb
        m = WholeBodyReference()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "map"
        m.x_cd = _v3(t["x_cd"][k])
        m.x_cd_dot = _v3(t["x_cd_dot"][k])
        m.x_cd_ddot = _v3(t["x_cd_ddot"][k])
        m.x_cd_d3 = _v3(t["x_cd_d3"][k])
        m.x_cd_d4 = _v3(t["x_cd_d4"][k])
        m.b1_d = _v3(t["b1_d"][k])
        m.b1_d_dot = _v3(t["b1_d_dot"][k])
        m.b1_d_ddot = _v3(t["b1_d_ddot"][k])
        m.r_ed = _v3(t["r_ed"][k])
        m.r_ed_dot = _v3(t["r_ed_dot"][k])
        m.r_ed_ddot = _v3(t["r_ed_ddot"][k])
        m.b1_de = _v3(t["b1_de"][k])
        m.b1_de_dot = _v3(t["b1_de_dot"][k])
        m.b1_de_ddot = _v3(t["b1_de_ddot"][k])
        m.q_d = [float(v) for v in t["q"][k]]
        m.qdot_d = [float(v) for v in t["qdot_d"][k]]
        self.pub_wb.publish(m)

    def set_from_table(self, k):
        t = self.tb
        self.ref_p = t["x_b"][k].copy()
        self.ref_v = t["x_b_dot"][k].copy()
        self.ref_a = t["x_b_ddot"][k].copy()
        self.ref_psi = float(t["psi"][k])
        self.q_ref = t["q"][k].copy()
        self.qd_ref = t["qdot_d"][k].copy()
        self.k_task = k

    def set_hold(self, k):
        """Static hold at table sample k -- all derivatives zeroed."""
        t = self.tb
        self.ref_p = t["x_b"][k].copy()
        self.ref_v = np.zeros(3)
        self.ref_a = np.zeros(3)
        self.ref_psi = float(t["psi"][k])
        self.q_ref = t["q"][k].copy()
        self.qd_ref = np.zeros(4)
        self.k_task = k

    # ---- main loop -----------------------------------------------------
    def tick(self):
        if self.done:
            return
        self._drain()

        streaming = self.phase not in ("WAIT", "OFFBOARD")
        if streaming:
            self.send_base_ref()
            self.send_arm_ref()
        if self.a.controller.startswith("wb") and self.is_direct() and streaming:
            self.send_wb_ref(max(self.k_task, 0))

        if self.odom is not None and streaming:
            self.log.append(np.concatenate((
                [self.now()], self.odom,
                self.ref_p, self.ref_v, [self.ref_psi],
                self.q_ref, self.qd_ref,
                self.q_meas if self.q_meas is not None else np.full(4, np.nan),
                self.qd_meas if self.qd_meas is not None else np.full(4, np.nan),
                self.tau_meas if self.tau_meas is not None else np.full(4, np.nan),
                self.tb["x_cd"][max(self.k_task, 0)],
                self.tb["r_ed"][max(self.k_task, 0)],
                self.tb["b1_de"][max(self.k_task, 0)],
                [1.0 if self.is_direct() else 0.0],
                [float(max(self.k_task, 0))],
            )))

        # ---- abort envelope ----
        if self.odom and self.phase in ("DIRECT_SETTLE", "TASK", "POST_HOLD"):
            x, y, z, vx, vy, vz, tilt = self.odom[:7]
            h = self.tb["base_home"]
            bad = (tilt > self.a.abort_tilt
                   or z > h[2] + 1.5 or z < 0.30
                   or abs(x - h[0]) > 2.5 or abs(y - h[1]) > 2.5
                   or np.linalg.norm([vx, vy, vz]) > 2.5)
            if bad and not self.aborted:
                self.aborted = True
                self.abort_reason = (f"pos=({x:.2f},{y:.2f},{z:.2f}) "
                                     f"tilt={tilt:.1f} deg "
                                     f"|v|={np.linalg.norm([vx,vy,vz]):.2f}")
                self.ev(f"!! ABORT ENVELOPE: {self.abort_reason}")
                self.call(self.cli_dir, SetBool.Request(data=False),
                          "set_direct_mode(false)")
                self.set_hold(0)
                self.goto("LAND_WAIT")
                return

        p = self.phase
        if p == "WAIT":
            if (self.odom is not None and self.mode and self.armed is not None
                    and self.q_meas is not None):
                if self.armed:
                    # An already-armed vehicle means the previous run left PX4
                    # latched "in flight" and the SAFETY UDE integrating the
                    # ground reaction; both survive a re-arm and silently
                    # suppress takeoff thrust.  Relaunch instead.
                    self.ev("!! PX4 IS ALREADY ARMED -- refusing to fly. "
                            "Do the full clean + relaunch.")
                    self.done = True
                    return
                self.set_hold(0)
                self.ref_p = np.array(self.odom[:3])   # hold the ground pose
                self.ev(f"stack up. mode='{self.mode}' "
                        f"q={np.round(np.degrees(self.q_meas),1).tolist()} deg")
                self.goto("OFFBOARD")
        elif p == "OFFBOARD":
            if self.in_phase() > 1.0:
                self.send_base_ref()
                self.call(self.cli_off, Trigger.Request(), "rc/offboard")
                self.goto("ARM")
        elif p == "ARM":
            if self.in_phase() > 3.0:
                self.call(self.cli_arm, Trigger.Request(), "rc/arm")
                self.set_hold(0)
                self.goto("TAKEOFF")
        elif p == "TAKEOFF":
            x, y, z, vx, vy, vz = self.odom[:6]
            err = np.linalg.norm(np.array([x, y, z]) - self.ref_p)
            spd = np.linalg.norm([vx, vy, vz])
            if err < 0.12 and spd < 0.10:
                self.settle_since = self.settle_since or time.time()
                if time.time() - self.settle_since > 5.0:
                    self.ev(f"settled: err={err*1000:.0f} mm "
                            f"speed={spd:.3f} m/s")
                    self.goto("ENTER_DIRECT")
            else:
                self.settle_since = None
            if self.in_phase() > 90.0:
                self.ev("!! takeoff never settled -> landing")
                self.goto("LAND_WAIT")
        elif p == "ENTER_DIRECT":
            if self.in_phase() > 0.5:
                self.call(self.cli_dir, SetBool.Request(data=True),
                          "set_direct_mode(true)")
                self.goto("CONFIRM")
        elif p == "CONFIRM":
            if self.is_direct():
                self.ev(f"DIRECT confirmed: '{self.mode}'")
                self.goto("DIRECT_SETTLE")
            elif self.in_phase() > 8.0:
                self.ev("!! DIRECT not confirmed -> landing")
                self.goto("LAND_WAIT")
        elif p == "DIRECT_SETTLE":
            # Let the mode-switch transient and (for WB) the GMO / (for L1)
            # the adaptation settle BEFORE the task starts, so the task window
            # measures tracking rather than engagement.
            if self.in_phase() > self.a.direct_settle:
                self.task_t0 = time.time()
                self.ev(f"TASK START: {self.tb['title']}")
                self.goto("TASK")
        elif p == "TASK":
            tt = time.time() - self.task_t0
            k = int(round(tt / self.tb["dt"]))
            if k >= len(self.tb["t"]):
                self.set_hold(len(self.tb["t"]) - 1)
                self.ev("TASK COMPLETE")
                self.goto("POST_HOLD")
            else:
                self.set_from_table(k)
        elif p == "POST_HOLD":
            if self.in_phase() > 8.0:
                self.call(self.cli_dir, SetBool.Request(data=False),
                          "set_direct_mode(false)")
                self.goto("SAFETY_SETTLE")
        elif p == "SAFETY_SETTLE":
            if self.in_phase() > 12.0:
                self.goto("LAND_WAIT")
        elif p == "LAND_WAIT":
            if self.in_phase() > 1.0:
                self.goto("LAND")
        elif p == "LAND":
            self.ref_p[2] = max(self.a.land_z,
                                self.ref_p[2] - 0.20 / self.a.rate)
            self.ref_v = np.zeros(3)
            self.ref_a = np.zeros(3)
            if self.ref_p[2] <= self.a.land_z + 1e-6 and self.in_phase() > 12.0:
                self.goto("DISARM")
        elif p == "DISARM":
            # PX4 denies disarm until its land detector sees low thrust, so
            # retry rather than assume.  Leaving the vehicle armed poisons the
            # NEXT run.
            if self.armed is False:
                self.ev("disarmed -- clean end state")
                self.goto("END")
            elif (self.in_phase() > 3.0 * (self.dis_tries + 1)
                  and self.dis_tries < 6):
                self.dis_tries += 1
                self.call(self.cli_dis, Trigger.Request(),
                          f"rc/disarm (try {self.dis_tries})")
            elif self.dis_tries >= 6:
                self.ev("!! STILL ARMED after 6 attempts -- relaunch before "
                        "the next run")
                self.goto("END")
        elif p == "END":
            if self.in_phase() > 2.0:
                self.done = True


LOG_COLUMNS = (
    ["t", "x", "y", "z", "vx", "vy", "vz", "tilt_deg", "yaw",
     "qw", "qx", "qy", "qz",
     "ref_x", "ref_y", "ref_z", "ref_vx", "ref_vy", "ref_vz", "ref_psi"]
    + [f"q_ref{i+1}" for i in range(4)]
    + [f"qd_ref{i+1}" for i in range(4)]
    + [f"q_meas{i+1}" for i in range(4)]
    + [f"qd_meas{i+1}" for i in range(4)]
    + [f"tau_meas{i+1}" for i in range(4)]
    + ["x_cd_x", "x_cd_y", "x_cd_z",
       "r_ed_x", "r_ed_y", "r_ed_z",
       "b1de_x", "b1de_y", "b1de_z",
       "direct", "k_task"]
)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--controller", required=True, choices=sorted(CONTROLLERS))
    ap.add_argument("--task", required=True, choices=sorted(CTK.TASKS))
    ap.add_argument("--namespace", default="/uav_0")
    ap.add_argument("--rate", type=float, default=100.0)
    ap.add_argument("--direct-settle", type=float, default=15.0)
    ap.add_argument("--land-z", type=float, default=0.35)
    ap.add_argument("--abort-tilt", type=float, default=35.0)
    ap.add_argument("--out", default=None,
                    help="npz path; default results/<task>/<controller>.npz")
    a = ap.parse_args()

    print(f"building task '{a.task}' ...", flush=True)
    table = CTK.TASKS[a.task]().build()
    feas = CTK.feasibility_report(table)
    print(f"  {table['title']}: {table['t'][-1]:.1f} s, "
          f"IK ok={feas['all_ik_ok']}, limits ok={feas['limit_ok']}, "
          f"sigma_nd min={feas['sigma_nd_min']:.3f}", flush=True)
    if not (feas["all_ik_ok"] and feas["limit_ok"] and feas["sigma_ok"]
            and feas["servo_slew_ok"]):
        print("REFUSING TO FLY: the task is not feasible offline.", flush=True)
        return 2

    repo = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
    out = a.out or os.path.join(repo, "results", a.task,
                                f"{a.controller}.npz")
    os.makedirs(os.path.dirname(out), exist_ok=True)

    rclpy.init()
    d = ComparisonDriver(a, table)
    try:
        while rclpy.ok() and not d.done:
            rclpy.spin_once(d, timeout_sec=0.05)
    except KeyboardInterrupt:
        d.ev("interrupted")
    log = np.array(d.log) if d.log else np.zeros((0, len(LOG_COLUMNS)))
    dbg = np.array(d.dbg_log) if d.dbg_log else np.zeros((0, 1))
    np.savez_compressed(
        out, log=log, columns=np.array(LOG_COLUMNS), debug=dbg,
        events=np.array(d.events), controller=a.controller, task=a.task,
        controller_label=CONTROLLERS[a.controller]["label"],
        aborted=d.aborted, abort_reason=d.abort_reason,
        direct_settle=a.direct_settle,
        **{f"tb_{k}": v for k, v in table.items()
           if isinstance(v, (np.ndarray, float, int))})
    print(f"\nwrote {out}  ({len(log)} samples, {len(dbg)} debug rows)"
          f"{'  [ABORTED]' if d.aborted else ''}", flush=True)
    d.destroy_node()
    rclpy.shutdown()
    return 1 if d.aborted else 0


if __name__ == "__main__":
    sys.exit(main())
