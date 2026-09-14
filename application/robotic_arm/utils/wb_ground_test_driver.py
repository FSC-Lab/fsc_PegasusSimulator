#!/usr/bin/env python3
"""Drive ONE whole-body GROUND test: engage DIRECT while seated, move the arm.

    /usr/bin/python3 wb_ground_test_driver.py --out ground.npz

WHAT THIS IS FOR. The ground-test rig
(start_t650_aerial_manipulator_whole_body_L1_adaptive_ground_test_sitl.sh) runs
the whole-body + L1 stack against a plant whose props produce NOTHING
(PEGASUS_PLANT_KF_SCALE = PEGASUS_PLANT_KM_SCALE = 0). The vehicle is seated on
its legs and cannot leave the ground. The question this driver answers is the
one the flying missions cannot: what does the coupled law do when it is given
authority over a vehicle whose thrust channel is answering to the FLOOR, and
then asked to move the arm?

DIFFERENCES FROM wb_l1_campaign_driver.py, all forced by the plant:
  * NO CLIMB and NO LANDING. There is no thrust; the vehicle stays seated. The
    mission is offboard -> arm -> DIRECT -> hold -> arm move -> back -> SAFETY.
  * IT PUBLISHES NO POSITION REFERENCE. wb_ground_reference_hold.py already
    holds `outer_ref_` on the vehicle's own measured pose (that is what opens
    the DIRECT position gate without a takeoff), and TWO publishers on
    `position_controller/reference` interleave silently with no way to tell
    them apart. The only thing this driver publishes is the planner's EE
    target.
  * THE ABORT ENVELOPE IS A TIP-OVER TEST, not a flight envelope. A seated
    vehicle cannot drift, so lateral motion or tilt means the arm is levering
    it over -- which is the interesting outcome, so the envelope is generous
    and the watchdog (20 deg) is expected to be what fires first.

THE ARM MOVE is the campaign's own compatible-trajectory leg, reused verbatim
because it is the one that is known feasible at the folded home: world +y
0.08 m, DOWN 0.07 m, EE heading +60 deg, which maps to
q = [17.6, 31.0, 29.8, 61.9] deg and moves every joint by >= 9 deg. Down is
the direction with room (it UNFOLDS); out and in are both outside the
workspace. See the campaign driver's build_legs() for the full derivation.

The npz layout is IDENTICAL to the campaign driver's, so wb_l1_metrics.py and
the other scorers read it unchanged.
"""

import argparse
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, String
from std_srvs.srv import SetBool, Trigger
from px4_msgs.msg import VehicleStatus
from fsc_autopilot_ros2_msgs.msg import WholeBodyReference

DA_NS = "fsc_autopilot_ros2/whole_body_direct_actuation"
PX4_QOS = QoSProfile(depth=5,
                     history=QoSHistoryPolicy.KEEP_LAST,
                     reliability=QoSReliabilityPolicy.BEST_EFFORT,
                     durability=QoSDurabilityPolicy.VOLATILE)


def tilt_deg(qw, qx, qy, qz):
    c = 1.0 - 2.0 * (qx * qx + qy * qy)
    return float(np.degrees(np.arccos(np.clip(c, -1.0, 1.0))))


class GroundDriver(Node):

    def __init__(self, a):
        super().__init__("wb_ground_test_driver")
        self.a = a
        ns = a.namespace.rstrip("/")
        da = f"{ns}/{DA_NS}"

        self.t0 = time.time()
        self.tp = self.t0
        self.phase = "WAIT"
        self.done = False
        self.aborted = False
        self.abort_reason = ""
        self.events = []
        self.futs = []

        self.odom = None
        self.dbg = None
        self.mode = ""
        self.armed = None
        self.q_meas = None
        self.tau_meas = None
        self.log = []
        self.dbg_log = []
        self.wbref = None
        self.wbref_log = []

        self.plan_status = ""
        self.cur_ee = None
        self.home_p = None
        self.home_psi = 0.0
        self.home_ee = None
        self.seat_p = None           # pose at DIRECT entry, for the tip metric
        self.legs = []
        self.leg_i = 0
        self.leg_marks = []
        self.sent = False
        self.saw_replan = False
        self.saw_exec = False
        self.leg_fail = None
        self.t_direct = np.nan

        self.create_subscription(
            Odometry, f"{ns}/state_estimator/local_position/odom",
            self.on_odom, 10)
        self.create_subscription(
            Float32MultiArray, f"{da}/wb_control_debug", self.on_dbg, 10)
        self.create_subscription(String, f"{da}/mode", self.on_mode, 10)
        self.create_subscription(
            VehicleStatus, f"{ns}/fmu/out/vehicle_status_v1", self.on_status,
            PX4_QOS)
        self.create_subscription(
            JointState, f"{ns}/fsc_open_manipulator/joint_states",
            self.on_joints, 10)
        self.create_subscription(
            String, f"{ns}/whole_body_planner/status", self.on_plan, 10)
        self.create_subscription(
            PoseStamped, f"{ns}/whole_body_planner/current_ee",
            self.on_cur_ee, 10)
        self.create_subscription(
            WholeBodyReference, f"{da}/reference", self.on_wbref, 10)

        self.pub_ee = self.create_publisher(
            PoseStamped, f"{ns}/whole_body_planner/ee_target", 10)

        self.cli_off = self.create_client(Trigger, f"{ns}/rc/offboard")
        self.cli_arm = self.create_client(Trigger, f"{ns}/rc/arm")
        self.cli_dis = self.create_client(Trigger, f"{ns}/rc/disarm")
        self.cli_dir = self.create_client(SetBool, f"{da}/set_direct_mode")
        self.cli_send = self.create_client(
            Trigger, f"{ns}/whole_body_planner/send")

        self.create_timer(1.0 / a.rate, self.tick)

    # ---- subscriptions ---------------------------------------------------
    def on_odom(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        v = m.twist.twist.linear
        yaw = np.arctan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.odom = np.array([p.x, p.y, p.z, v.x, v.y, v.z,
                              tilt_deg(q.w, q.x, q.y, q.z), yaw])

    def on_dbg(self, m):
        self.dbg = np.asarray(m.data, dtype=float)
        if self.phase not in ("WAIT",):
            self.dbg_log.append(np.concatenate(([self.now()], self.dbg)))

    def on_mode(self, m):
        if m.data != self.mode:
            self.ev(f"mode -> {m.data}")
        self.mode = m.data

    def on_status(self, m):
        self.armed = (m.arming_state == 2)

    def on_joints(self, m):
        if len(m.position) >= 4:
            self.q_meas = np.asarray(m.position[:4], dtype=float)
        if len(m.effort) >= 4 and len(m.name) >= 4:
            try:
                idx = [list(m.name).index(f"joint{j + 1}") for j in range(4)]
            except ValueError:
                idx = list(range(4))
            self.tau_meas = np.asarray([m.effort[i] for i in idx], dtype=float)

    def on_wbref(self, m):
        v = lambda a: (a.x, a.y, a.z)          # noqa: E731
        self.wbref = (list(v(m.x_cd)) + list(v(m.x_cd_dot)) + list(v(m.b1_d))
                      + list(v(m.r_ed)) + list(v(m.r_ed_dot))
                      + list(v(m.b1_de)) + list(m.q_d) + list(m.qdot_d))

    def on_plan(self, m):
        if m.data != self.plan_status:
            self.ev(f"planner -> {m.data}")
        self.plan_status = m.data
        if self.plan_state() != "PLANNED":
            self.saw_replan = True

    def on_cur_ee(self, m):
        p, q = m.pose.position, m.pose.orientation
        yaw = np.arctan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.cur_ee = np.array([p.x, p.y, p.z, yaw])

    # ---- helpers ---------------------------------------------------------
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
        z = self.odom[2] if self.odom is not None else float("nan")
        self.ev(f"PHASE {ph}  z={z:.3f} mode='{self.mode}'")

    def call(self, cli, req, name):
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

    def send_ee_target(self, p, yaw):
        m = PoseStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "map"
        m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, p)
        m.pose.orientation.z = float(np.sin(0.5 * yaw))
        m.pose.orientation.w = float(np.cos(0.5 * yaw))
        self.pub_ee.publish(m)

    def plan_state(self):
        st = self.plan_status.replace(":", " ").split()
        return st[0].upper() if st else ""

    def build_legs(self):
        """The campaign's compatible-trajectory leg, out and back.

        Reused verbatim (world +y lateral, DOWN, +60 deg EE heading) because
        it is the one move mapped offline and flown: it moves all four joints
        and stays inside every limit and the 0.10 singularity keep-out.
        """
        if self.home_ee is None:
            self.ev("!! no current_ee from the planner -- no arm leg to fly")
            return []
        e = self.home_ee
        lat, down, daz = self.a.ee_lat, self.a.ee_step, np.deg2rad(self.a.ee_yaw)
        return [
            ("arm_out", "ee", (e[:3] + [0.0, lat, -down], e[3] + daz)),
            ("arm_back", "ee", (e[:3].copy(), e[3])),
        ]

    def start_leg(self):
        name, _kind, payload = self.legs[self.leg_i]
        self.sent = False
        self.saw_exec = False
        self.saw_replan = self.plan_state() != "PLANNED"
        self.send_ee_target(payload[0], payload[1])
        self.leg_marks.append((self.now(), name))
        self.ev(f"LEG {self.leg_i + 1}/{len(self.legs)} '{name}' target "
                f"{np.round(payload[0], 3).tolist()} az "
                f"{np.degrees(payload[1]):.0f} deg -- waiting for PLANNED")

    def next_leg(self):
        self.leg_i += 1
        if self.leg_i >= len(self.legs):
            self.goto("POST_HOLD")
        else:
            self.start_leg()
            self.goto("LEG_PLAN")

    # ---- main loop -------------------------------------------------------
    def tick(self):
        if self.done:
            return
        self._drain()

        if self.wbref is not None:
            self.wbref_log.append([self.now()] + list(self.wbref))
        if self.odom is not None:
            self.log.append(np.concatenate((
                [self.now()], self.odom, np.full(3, np.nan), [np.nan],
                self.q_meas if self.q_meas is not None else np.full(4, np.nan),
                [1.0 if self.is_direct() else 0.0],
                self.tau_meas if self.tau_meas is not None
                else np.full(4, np.nan))))

        # ---- abort envelope: a TIP-OVER test, not a flight envelope -------
        if (self.odom is not None
                and self.phase in ("DIRECT_SETTLE", "LEG_PLAN", "LEG_EXEC",
                                   "LEG_SETTLE", "POST_HOLD")):
            x, y, z, vx, vy, vz, tilt = self.odom[:7]
            d = (np.linalg.norm([x - self.seat_p[0], y - self.seat_p[1]])
                 if self.seat_p is not None else 0.0)
            bad = (tilt > self.a.abort_tilt or d > self.a.abort_drift
                   or z > 0.9 or z < 0.10)
            if bad and not self.aborted:
                self.aborted = True
                self.abort_reason = (f"pos=({x:.2f},{y:.2f},{z:.3f}) "
                                     f"tilt={tilt:.1f} deg slide={d:.3f} m")
                self.ev(f"!! ABORT ENVELOPE: {self.abort_reason}")
                self.call(self.cli_dir, SetBool.Request(data=False),
                          "set_direct_mode(false)")
                self.goto("REVERT_SETTLE")
                return

        p = self.phase
        if p == "WAIT":
            if self.odom is not None and self.mode and self.armed is not None:
                if self.armed:
                    self.ev("!! PX4 IS ALREADY ARMED -- refusing to start. "
                            "Do the full clean + relaunch.")
                    self.done = True
                    return
                self.ev(f"stack up. mode='{self.mode}' "
                        f"z={self.odom[2]:.3f} m (seated) "
                        f"tilt={self.odom[6]:.2f} deg")
                self.goto("OFFBOARD")
        elif p == "OFFBOARD":
            if self.in_phase() > 2.0:
                self.call(self.cli_off, Trigger.Request(), "rc/offboard")
                self.goto("ARM")
        elif p == "ARM":
            if self.in_phase() > 3.0:
                self.call(self.cli_arm, Trigger.Request(), "rc/arm")
                self.goto("SEATED")
        elif p == "SEATED":
            # Let PX4 settle in OFFBOARD and the props spin up (to nothing).
            if self.in_phase() > self.a.seated_wait:
                if not self.armed:
                    self.ev("!! not armed after the arm request -- stopping")
                    self.done = True
                    return
                self.ev(f"armed and seated: z={self.odom[2]:.3f} m "
                        f"tilt={self.odom[6]:.2f} deg")
                self.goto("ENTER_DIRECT")
        elif p == "ENTER_DIRECT":
            if self.in_phase() > 1.0:
                self.seat_p = np.array(self.odom[:3])
                self.call(self.cli_dir, SetBool.Request(data=True),
                          "set_direct_mode(true)")
                self.goto("DIRECT_WAIT")
        elif p == "DIRECT_WAIT":
            if self.is_direct():
                self.t_direct = self.now()
                self.home_p = np.array(self.odom[:3])
                self.home_psi = float(self.odom[7])
                self.ev("DIRECT ENGAGED ON THE GROUND")
                self.goto("DIRECT_SETTLE")
            elif self.in_phase() > 15.0:
                self.ev("!! DIRECT never engaged (gate refused?) -- stopping")
                self.leg_fail = "direct entry refused"
                self.done = True
                return
        elif p == "DIRECT_SETTLE":
            if self.in_phase() > self.a.direct_settle:
                self.home_ee = (self.cur_ee.copy()
                                if self.cur_ee is not None else None)
                if self.home_ee is not None:
                    self.ev(f"planner EE anchor "
                            f"{np.round(self.home_ee[:3], 3).tolist()} az "
                            f"{np.degrees(self.home_ee[3]):.1f} deg")
                self.legs = self.build_legs()
                if not self.legs:
                    self.goto("POST_HOLD")
                else:
                    self.leg_i = 0
                    self.start_leg()
                    self.goto("LEG_PLAN")
        elif p == "LEG_PLAN":
            st = self.plan_state()
            if st == "INFEASIBLE":
                self.ev(f"!! leg refused: {self.plan_status}")
                self.leg_fail = self.plan_status
                self.next_leg()
            elif st == "PLANNED" and self.saw_replan and not self.sent:
                self.sent = True
                self.call(self.cli_send, Trigger.Request(),
                          "whole_body_planner/send")
                self.goto("LEG_EXEC")
            elif self.in_phase() > self.a.plan_timeout:
                self.ev(f"!! plan timeout (status '{self.plan_status}')")
                self.leg_fail = f"plan timeout: {self.plan_status}"
                self.next_leg()
        elif p == "LEG_EXEC":
            st = self.plan_state()
            if st == "EXECUTING":
                self.saw_exec = True
            if self.saw_exec and st == "HOLD":
                self.ev("leg complete")
                self.goto("LEG_SETTLE")
            elif self.in_phase() > self.a.exec_timeout:
                self.ev(f"!! exec timeout (status '{self.plan_status}')")
                self.leg_fail = f"exec timeout: {self.plan_status}"
                self.goto("LEG_SETTLE")
        elif p == "LEG_SETTLE":
            if self.in_phase() > self.a.hold_between:
                self.next_leg()
        elif p == "POST_HOLD":
            if self.in_phase() > self.a.post_hold:
                self.call(self.cli_dir, SetBool.Request(data=False),
                          "set_direct_mode(false)")
                self.goto("REVERT_SETTLE")
        elif p == "REVERT_SETTLE":
            if self.in_phase() > self.a.abort_settle:
                self.call(self.cli_dis, Trigger.Request(), "rc/disarm")
                self.goto("FINISH")
        elif p == "FINISH":
            if self.in_phase() > 4.0:
                self.done = True

    # ---- output ----------------------------------------------------------
    def save(self, path):
        os.makedirs(os.path.dirname(os.path.abspath(path)) or ".",
                    exist_ok=True)
        log = np.array(self.log) if self.log else np.zeros((0, 22))
        if self.dbg_log:
            w = max(len(r) for r in self.dbg_log)
            dbg = np.full((len(self.dbg_log), w), np.nan)
            for i, r in enumerate(self.dbg_log):
                dbg[i, :len(r)] = r
        else:
            w, dbg = 0, np.zeros((0, 1))
        wbref = (np.array(self.wbref_log, dtype=float) if self.wbref_log
                 else np.zeros((0, 1)))
        np.savez_compressed(
            path, log=log, dbg=dbg, wbref=wbref,
            wbref_cols=np.array(
                ["t"] + [f"x_cd_{c}" for c in "xyz"]
                + [f"x_cd_dot_{c}" for c in "xyz"] + [f"b1_d_{c}" for c in "xyz"]
                + [f"r_ed_{c}" for c in "xyz"] + [f"r_ed_dot_{c}" for c in "xyz"]
                + [f"b1_de_{c}" for c in "xyz"] + [f"q_d{i}" for i in range(4)]
                + [f"qdot_d{i}" for i in range(4)], dtype=object),
            events=np.array(self.events, dtype=object),
            aborted=self.aborted, abort_reason=self.abort_reason,
            t_direct=self.t_direct, hover_z=np.nan, soak=0.0,
            leg_marks=np.array([f"{t:.3f} {n}" for t, n in self.leg_marks],
                               dtype=object),
            leg_fail="" if self.leg_fail is None else str(self.leg_fail),
            log_cols=np.array(
                ["t", "x", "y", "z", "vx", "vy", "vz", "tilt_deg", "yaw",
                 "ref_x", "ref_y", "ref_z", "ref_psi",
                 "q1", "q2", "q3", "q4", "direct",
                 "tau_app1", "tau_app2", "tau_app3", "tau_app4"],
                dtype=object),
            dbg_cols=np.array(["t"] + [f"d{i}" for i in range(w)],
                              dtype=object))
        print(f"\nwrote {path}  log {log.shape}  dbg {dbg.shape}", flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--namespace", default="/uav_0")
    ap.add_argument("--rate", type=float, default=50.0)
    ap.add_argument("--seated-wait", type=float, default=8.0,
                    help="s armed and seated before DIRECT")
    ap.add_argument("--direct-settle", type=float, default=20.0,
                    help="s of DIRECT before the arm is asked to move")
    ap.add_argument("--ee-lat", type=float, default=0.08)
    ap.add_argument("--ee-step", type=float, default=0.07,
                    help="EE DOWN step [m] -- the direction with room")
    ap.add_argument("--ee-yaw", type=float, default=60.0)
    ap.add_argument("--plan-timeout", type=float, default=25.0)
    ap.add_argument("--exec-timeout", type=float, default=45.0)
    ap.add_argument("--hold-between", type=float, default=12.0)
    ap.add_argument("--post-hold", type=float, default=15.0)
    ap.add_argument("--abort-settle", type=float, default=12.0)
    ap.add_argument("--abort-tilt", type=float, default=35.0)
    ap.add_argument("--abort-drift", type=float, default=1.0,
                    help="horizontal slide from the seated pose [m]")
    ap.add_argument("--out", default="wb_ground_run.npz")
    a = ap.parse_args()

    rclpy.init()
    n = GroundDriver(a)
    try:
        while rclpy.ok() and not n.done:
            rclpy.spin_once(n, timeout_sec=0.05)
    except KeyboardInterrupt:
        n.ev("interrupted")
    finally:
        n.save(a.out)
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 1 if n.aborted else 0


if __name__ == "__main__":
    sys.exit(main())
