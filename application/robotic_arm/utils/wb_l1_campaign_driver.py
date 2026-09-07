#!/usr/bin/env python3
"""Fly ONE whole-body hover-soak mission and record it, on either observer.

    /usr/bin/python3 wb_l1_campaign_driver.py --out run.npz [--soak 90]

WHAT THIS IS FOR. The whole-body rig's `_sim` configuration is the one that
carries every deliberate deviation from a perfect plant -- a +15% allocator kf
(thrust loss), plant mass and inertia x1.10 with a 10/10/5 mm CoM shift (model
uncertainty), and the MN4010 rotor lag (motor delay, lambda = 10.0265 1/s).
That is the configuration in which "does the disturbance augmentation work?"
is a meaningful question, so this driver flies THAT stack rather than the
matched-model comparison one.

THE MISSION is the rig's standard test:

    offboard -> arm -> SAFETY climb to 1 m -> settle -> DIRECT
      -> settle  (the whole-body planner streams the rest hold captured at entry)
      -> STEP X   +/- step, out and back
      -> STEP Y   +/- step, out and back
      -> STEP YAW +/- step, out and back
      -> COMPATIBLE TRAJECTORY: an inertial END-EFFECTOR target, planned by
         the transition planner and executed -- the whole-body path where the
         arm and the base move together
      -> hold -> SAFETY abort -> descend -> disarm attempt -> done

IN DIRECT THE WHOLE-BODY PLANNER OWNS THE REFERENCE, so a "step" is not a
reference publish. The drone-GS target is captured as PENDING and never
executed; the planner solves a compatible transition, reports PLANNED, and
only an explicit Send starts it. This driver runs that handshake
(target -> PLANNED -> Send -> EXECUTING -> HOLD) and refuses to move on if the
planner reports INFEASIBLE, which is why it is a state machine and not a
sequence of sleeps.

`--soak` still exists and inserts a plain hold before the steps; `--no-steps`
falls back to the hover-only mission the 2026-09-06 sweep flew, so those runs
remain reproducible.

WHY A HOVER SOAK MEASURES THE THING. In free flight the true interaction
wrench is EXACTLY zero, so whatever the controller reports as an end-effector
force is entirely phantom -- the note's own proposed experiment ("settled by
comparing the free-flight reading of F_hat_y with the forces of the task").
And because the plant is mismatched, the estimate has real work to do: the
+15% kf alone is ~5.5 N of collective the law must find before the vehicle
holds station.

CONTROLLER-AGNOSTIC. The GMO and L1 rigs share a node namespace, a debug topic
and a mode service, so the same driver flies both and the npz layout is
identical; debug[57] says which observer was live. That is what makes an A/B
possible with one script.

OPERATING NOTES, all learned the hard way on this rig:
  * PX4 never disarms it ("Disarming denied: not landed", forever), so the
    vehicle stays armed and the next run finds PX4 latched in flight AND the
    SAFETY UDE integrating the ground reaction -- the takeoff then produces
    zero lift with no error message. Every run needs a full clean + relaunch;
    this driver REFUSES to start against an already-armed vehicle.
  * Land to z >= 0.35. The resting height is 0.305 m and commanding below it
    pushes the vehicle into the ground and tips it.
  * Never gate arming on vehicle_status.pre_flight_checks_pass: it is false on
    this rig while it is armed and flying. The cycle script gates on the EKF
    status flags instead.
"""

import argparse
import os
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, String
from std_srvs.srv import SetBool, Trigger

from fsc_autopilot_ros2_msgs.msg import (PositionControllerReference,
                                         WholeBodyReference)
from px4_msgs.msg import VehicleStatus

PX4_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE,
                     history=HistoryPolicy.KEEP_LAST, depth=10)

DA_NS = "fsc_autopilot_ros2/whole_body_direct_actuation"


def tilt_deg(qw, qx, qy, qz):
    """Angle between body +z and world +z."""
    c = 1.0 - 2.0 * (qx * qx + qy * qy)
    return float(np.degrees(np.arccos(np.clip(c, -1.0, 1.0))))


class Driver(Node):
    def __init__(self, a):
        super().__init__("wb_l1_campaign_driver")
        self.a = a
        ns = a.namespace.rstrip("/")
        da = f"{ns}/{DA_NS}"

        self.t0 = time.time()
        self.tp = self.t0
        self.phase = "WAIT"
        self.done = False
        self.aborted = False
        self.abort_reason = ""
        self.settle_since = None
        self.events = []
        self.futs = []

        self.odom = None          # x y z vx vy vz tilt yaw
        self.dbg = None
        self.mode = ""
        self.armed = None
        self.q_meas = None
        self.log = []
        self.dbg_log = []

        self.ref_p = np.array([0.0, 0.0, a.hover_z])
        self.ref_psi = 0.0
        # ---- whole-body planner handshake state ----
        self.plan_status = ""     # IDLE HOLD PENDING CALCULATING PLANNED
        #                           INFEASIBLE:<reason> EXECUTING
        self.cur_ee = None        # inertial EE pose held by the planner
        self.home_p = None        # base hold captured at DIRECT entry
        self.home_psi = 0.0
        self.home_ee = None
        self.legs = []            # the mission, built at DIRECT entry
        self.leg_i = 0
        self.leg_marks = []       # (t, name) for the metrics
        self.sent = False
        self.leg_fail = None
        self.saw_replan = False   # the planner left PLANNED since this target
        self.saw_exec = False     # this leg's Send actually reached EXECUTING
        self._last_pub = None     # last base reference actually published

        self.create_subscription(
            Odometry, f"{ns}/state_estimator/local_position/odom",
            self.on_odom, 10)
        self.create_subscription(
            Float32MultiArray, f"{da}/wb_control_debug", self.on_dbg, 10)
        self.create_subscription(String, f"{da}/mode", self.on_mode, 10)
        self.create_subscription(
            # _v1 on this PX4 v1.16 / px4_msgs release/1.16 pairing -- the
            # un-suffixed name exists in the topic list but never publishes,
            # which strands the driver in WAIT with no error at all.
            VehicleStatus, f"{ns}/fmu/out/vehicle_status_v1", self.on_status,
            PX4_QOS)
        self.create_subscription(
            JointState, f"{ns}/fsc_open_manipulator/joint_states",
            self.on_joints, 10)
        # The planner's own state machine and the EE pose it is holding.
        # Status is LATCHED, so this is reliable even if we subscribe late.
        self.create_subscription(
            String, f"{ns}/whole_body_planner/status", self.on_plan, 10)
        self.create_subscription(
            PoseStamped, f"{ns}/whole_body_planner/current_ee",
            self.on_cur_ee, 10)
        # THE FULL REFERENCE SET the law actually consumes, straight off the
        # wire. The debug array carries only the CoM reference and the task
        # ERRORS, so without this a "reference vs state" plot has to be
        # reconstructed, and the EE and joint references cannot be
        # reconstructed at all. Model frame by contract (see the msg).
        self.wbref = None
        self.wbref_log = []
        self.create_subscription(
            WholeBodyReference,
            f"{da}/reference", self.on_wbref, 10)

        self.pub_ee = self.create_publisher(
            PoseStamped, f"{ns}/whole_body_planner/ee_target", 10)

        self.pub_ref = self.create_publisher(
            PositionControllerReference,
            f"{ns}/fsc_autopilot_ros2/position_controller/reference", 10)

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
        if self.phase not in ("WAIT", "OFFBOARD"):
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

    def on_wbref(self, m):
        v = lambda a: (a.x, a.y, a.z)          # noqa: E731
        self.wbref = (list(v(m.x_cd)) + list(v(m.x_cd_dot)) + list(v(m.b1_d))
                      + list(v(m.r_ed)) + list(v(m.r_ed_dot))
                      + list(v(m.b1_de)) + list(m.q_d) + list(m.qdot_d))

    def on_plan(self, m):
        if m.data != self.plan_status:
            self.ev(f"planner -> {m.data}")
        self.plan_status = m.data
        # A new target drives the planner out of PLANNED (it sets PENDING
        # synchronously). Seeing that is how a leg knows the PLANNED it is
        # about to Send is ITS OWN and not the previous leg's.
        if self.plan_state() not in ("PLANNED",):
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
        self.ev(f"PHASE {ph}  ref={np.round(self.ref_p, 3).tolist()} "
                f"mode='{self.mode}'")

    def call(self, cli, req, name):
        # Fire-and-forget: tick() runs inside the executor callback, so
        # spinning for the future here would deadlock on it.
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
        """The planner's state WORD alone.

        The status carries suffixes -- "PLANNED T=5.2s" and
        "INFEASIBLE: <reason>" -- so this must take the first token, splitting
        on whitespace AND ':'. Stripping only the colon (the first version of
        this) never matched "PLANNED", so every leg silently timed out after
        plan_timeout while the planner was reporting a perfectly good plan.
        """
        st = self.plan_status.replace(":", " ").split()
        return st[0].upper() if st else ""

    def build_legs(self):
        """The mission, anchored on the pose held at DIRECT entry.

        Each leg is (name, kind, payload). `base` legs publish a drone-GS
        target; the `ee` leg publishes an inertial end-effector target, which
        is the compatible-trajectory path -- the transition planner solves a
        CoM trajectory consistent with the arm motion, and both move.
        """
        h, psi = self.home_p, self.home_psi
        d, dy = self.a.step_xy, np.deg2rad(self.a.step_yaw)
        legs = [
            ("step_x+", "base", (h + [d, 0, 0], psi)),
            ("step_x-", "base", (h, psi)),
            ("step_y+", "base", (h + [0, d, 0], psi)),
            ("step_y-", "base", (h, psi)),
            ("step_yaw+", "base", (h, psi + dy)),
            ("step_yaw-", "base", (h, psi)),
        ]
        if self.home_ee is not None:
            e = self.home_ee
            s = self.a.ee_step
            # A COMPATIBLE TRAJECTORY THAT MOVES ALL FOUR JOINTS: down,
            # sideways, AND a change of EE heading. Down alone (the first
            # version of this leg) is a pure fold -- it moves q2/q3 and leaves
            # q1 = q4 = 0, so the arm-yaw and wrist channels of the law are
            # never exercised. Adding a lateral offset drives q1, and the
            # heading step drives q4.
            #
            # THE LATERAL AXIS IS WORLD Y, NOT X, and that is not a free
            # choice -- it was measured. The target is published in WORLD and
            # the planner solves in the MODEL frame, related by
            # frame_adapter's R0_model = R0_actual*Rz(-90), so at the flown
            # yaw ~ 0:
            #
            #     model +y (along the arm, RADIALLY OUT)  ->  world +x
            #     model +/-x (across it, what drives q1)  ->  world -/+y
            #
            # A world-X lateral step is therefore a reach OUTWARD, and the arm
            # has no room there: flown 2026-09-06, the planner refused it in
            # 0.13 s with "IK did not converge". World Y is the axis with
            # room. Its SIGN does not matter -- the workspace is symmetric
            # across the arm, so both model signs solve with the same margin,
            # and +60 deg of heading is enough that q4 moves a lot either way.
            # Mapped offline with transition_planner.ik_position_azimuth
            # before flying:
            #
            #   world +y 0.08, down 0.07, heading +60 deg  (= model -x)
            #     -> q = [ 17.6, 31.0, 29.8, 61.9] deg, |dq| >= 9.0, sig 0.297
            #   the mirrored step                          (= model +x)
            #     -> q = [-17.5, 31.5, 29.1, 83.8] deg, |dq| >= 8.5, sig 0.295
            #
            # Both are inside every joint limit and far above the planner's
            # 0.10 singularity keep-out.
            #
            # WHY NOT OUT, AND WHY NOT IN. The feasible set around the folded
            # home is small and lopsided, and both obvious guesses are
            # outside it (both measured in flight, 2026-09-06):
            #
            #   OUT+DOWN by 0.15 m -> "IK did not converge": the gripper at
            #     home already sits 0.26 m from the body origin, at the arm's
            #     own reach.
            #   IN+UP by 0.05 m    -> "joint limits: q3 = 63.0 deg vs 50":
            #     retracting FOLDS the arm further, and q3 is already at 40.
            #
            # Mapping the planner's own IK over a grid (transition_planner.
            # ik_position_azimuth from q_home) shows the room is DOWNWARD,
            # which UNFOLDS: 0.06 m down solves to q = [0, 27.1, 38.8, 0] deg,
            # comfortably inside every limit, and is the largest step of the
            # pure-axis moves that stays there. The planner also publishes the
            # true reachable set on whole_body_planner/workspace_rz.
            lat, daz = self.a.ee_lat, np.deg2rad(self.a.ee_yaw)
            # THE COMPATIBLE-TRAJECTORY LEGS. Two of them, and they test
            # different things:
            #   traj_ee   -- arm only: the EE target moves, the planner solves
            #                whatever CoM path keeps it dynamically feasible.
            #   traj_both -- WHOLE SYSTEM: the base translates in x, y AND z
            #                while the arm simultaneously unfolds, swings and
            #                rolls its wrist. Every one of the law's task
            #                channels moves at once, which is the case a
            #                lumped disturbance estimate is worst at.
            d3 = np.asarray(self.a.both_base, float)
            arm = np.array([0.0, lat, -s])
            legs += [
                ("traj_ee", "ee",
                 (e[:3] + [0.0, lat, -s], e[3] + daz)),
                ("traj_ee_back", "ee", (e[:3].copy(), e[3])),
                ("traj_both", "both",
                 (h + d3, psi + np.deg2rad(self.a.both_yaw),
                  e[:3] + d3 + arm, e[3] + daz)),
                ("traj_both_back", "both",
                 (h.copy(), psi, e[:3].copy(), e[3])),
            ]
        else:
            self.ev("!! no current_ee from the planner -- skipping the "
                    "compatible-trajectory leg")
        return legs

    def start_leg(self):
        name, kind, payload = self.legs[self.leg_i]
        self.sent = False
        self.saw_exec = False
        self.saw_replan = self.plan_state() != "PLANNED"
        if kind == "base":
            self.ref_p, self.ref_psi = np.asarray(payload[0], float), payload[1]
            self.send_ref()
        elif kind == "both":
            # WHOLE-SYSTEM MOVE. The planner holds a drone-GS target as a
            # PENDING base and folds it into the next EE plan, so publishing
            # the base first and the EE second gives ONE trajectory in which
            # the base translates AND the arm reconfigures simultaneously --
            # rather than the two being planned and flown one after the other.
            # The EE target is INERTIAL, so it must carry the base delta too
            # or the arm would be asked to stay put in the world while the
            # vehicle flies out from under it.
            base_p, base_psi, ee_p, ee_yaw = payload
            self.ref_p, self.ref_psi = np.asarray(base_p, float), base_psi
            self.send_ref()
            time.sleep(0.4)          # let the planner latch it as PENDING
            self.send_ee_target(ee_p, ee_yaw)
        else:
            self.send_ee_target(payload[0], payload[1])
        self.leg_marks.append((self.now(), name))
        self.ev(f"LEG {self.leg_i + 1}/{len(self.legs)} '{name}' ({kind}) "
                f"target sent -- waiting for PLANNED")

    def send_ref(self):
        m = PositionControllerReference()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "map"
        m.position.x, m.position.y, m.position.z = map(float, self.ref_p)
        m.yaw = float(self.ref_psi)
        # yaw_unit defaults to DEGREES; this driver works in radians.
        m.yaw_unit = PositionControllerReference.RADIANS
        self.pub_ref.publish(m)
        self._last_pub = (self.ref_p.copy(), self.ref_psi)

    def send_ref_if_needed(self):
        """Stream at full rate in SAFETY; in DIRECT publish only on CHANGE.

        In DIRECT the whole_body_planner treats every drone-GS reference as a
        base target, and its unchanged-target guard is inactive while it is
        HOLDing -- so a 50 Hz stream of the SAME setpoint drags it out of HOLD
        and into a re-plan on the very next tick after any leg completes.
        Measured on the first flight of this mission.
        """
        if not self.is_direct():
            self.send_ref()
            return
        p, psi = self._last_pub if self._last_pub else (None, None)
        if (p is None
                or np.linalg.norm(self.ref_p - p) > 1e-4
                or abs(np.arctan2(np.sin(self.ref_psi - psi),
                                  np.cos(self.ref_psi - psi))) > 1e-4):
            self.send_ref()

    # ---- main loop -------------------------------------------------------
    def tick(self):
        if self.done:
            return
        self._drain()

        streaming = self.phase not in ("WAIT", "OFFBOARD")
        if streaming:
            self.send_ref_if_needed()
        # These three logs share one clock (self.now()) but NOT one gate: the
        # whole-body reference only exists in DIRECT, while the state log has
        # to cover the SAFETY takeoff too. Keep them as siblings -- nesting
        # the state log under the reference silently deletes every pre-DIRECT
        # sample, and the takeoff analysis then has nothing to work on.
        if self.wbref is not None:
            self.wbref_log.append([self.now()] + list(self.wbref))
        if self.odom is not None:
            self.log.append(np.concatenate((
                [self.now()], self.odom, self.ref_p, [self.ref_psi],
                self.q_meas if self.q_meas is not None else np.full(4, np.nan),
                [1.0 if self.is_direct() else 0.0])))

        # ---- abort envelope ----
        if (self.odom is not None
                and self.phase in ("DIRECT_SETTLE", "SOAK", "LEG_PLAN",
                                   "LEG_EXEC", "LEG_SETTLE", "POST_HOLD")):
            x, y, z, vx, vy, vz, tilt = self.odom[:7]
            bad = (tilt > self.a.abort_tilt
                   or z > self.a.hover_z + 1.5 or z < 0.30
                   or abs(x) > 2.5 or abs(y) > 2.5
                   or np.linalg.norm([vx, vy, vz]) > 2.5)
            if bad and not self.aborted:
                self.aborted = True
                self.abort_reason = (f"pos=({x:.2f},{y:.2f},{z:.2f}) "
                                     f"tilt={tilt:.1f} deg "
                                     f"|v|={np.linalg.norm([vx, vy, vz]):.2f}")
                self.ev(f"!! ABORT ENVELOPE: {self.abort_reason}")
                self.call(self.cli_dir, SetBool.Request(data=False),
                          "set_direct_mode(false)")
                self.goto("LAND")
                return

        p = self.phase
        if p == "WAIT":
            if (self.odom is not None and self.mode and self.armed is not None):
                if self.armed:
                    # An already-armed vehicle means the previous run left PX4
                    # latched "in flight" AND the SAFETY UDE integrating the
                    # ground reaction; both survive a re-arm and silently
                    # suppress takeoff thrust. Relaunch instead of flying.
                    self.ev("!! PX4 IS ALREADY ARMED -- refusing to fly. "
                            "Do the full clean + relaunch.")
                    self.done = True
                    return
                self.ref_p = np.array(self.odom[:3])     # hold the ground pose
                self.ref_psi = float(self.odom[7])
                self.ev(f"stack up. mode='{self.mode}' "
                        f"z={self.odom[2]:.3f} m")
                self.goto("OFFBOARD")
        elif p == "OFFBOARD":
            if self.in_phase() > 1.0:
                self.send_ref()
                self.call(self.cli_off, Trigger.Request(), "rc/offboard")
                self.goto("ARM")
        elif p == "ARM":
            if self.in_phase() > 3.0:
                self.call(self.cli_arm, Trigger.Request(), "rc/arm")
                self.goto("CLIMB")
        elif p == "CLIMB":
            if self.in_phase() > 2.0:
                self.ref_p = np.array([self.ref_p[0], self.ref_p[1],
                                       self.a.hover_z])
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
                self.goto("LAND")
        elif p == "ENTER_DIRECT":
            if self.in_phase() > 0.5:
                self.call(self.cli_dir, SetBool.Request(data=True),
                          "set_direct_mode(true)")
                self.goto("CONFIRM")
        elif p == "CONFIRM":
            if self.is_direct():
                self.ev("DIRECT confirmed")
                self.t_direct = self.now()
                self.goto("DIRECT_SETTLE")
            elif self.in_phase() > 12.0:
                self.ev("!! DIRECT never engaged (gates refused?) -> landing")
                self.goto("LAND")
        elif p == "DIRECT_SETTLE":
            if self.in_phase() > self.a.direct_settle:
                # Anchor the mission on what the planner is actually holding,
                # not on the reference we asked SAFETY for: the whole-body
                # planner captured its own rest hold at the DIRECT edge and
                # the CoM anchor makes the two differ by centimetres.
                self.home_p = np.array(self.odom[:3])
                self.home_psi = float(self.odom[7])
                self.home_ee = None if self.cur_ee is None else self.cur_ee.copy()
                self.ev(f"mission anchor: base {np.round(self.home_p, 3).tolist()} "
                        f"yaw {np.degrees(self.home_psi):.1f} deg | EE "
                        f"{None if self.home_ee is None else np.round(self.home_ee, 3).tolist()}")
                self.goto("SOAK")
        elif p == "SOAK":
            if self.in_phase() > self.a.soak:
                if self.a.no_steps:
                    self.ev("hover-only mission complete -> SAFETY")
                    self.call(self.cli_dir, SetBool.Request(data=False),
                              "set_direct_mode(false)")
                    self.goto("ABORT_SETTLE")
                else:
                    self.legs = self.build_legs()
                    self.leg_i = 0
                    if not self.legs:
                        self.goto("POST_HOLD")
                    else:
                        self.start_leg()
                        self.goto("LEG_PLAN")
        elif p == "LEG_PLAN":
            st = self.plan_state()
            if st == "PLANNED" and self.saw_replan:
                self.call(self.cli_send, Trigger.Request(),
                          f"send({self.legs[self.leg_i][0]})")
                self.sent = True
                self.goto("LEG_EXEC")
            elif st == "INFEASIBLE":
                # A refused goal is data, not a crash: log it, skip the leg,
                # keep flying. The planner names the reason.
                self.ev(f"!! leg '{self.legs[self.leg_i][0]}' INFEASIBLE: "
                        f"{self.plan_status}")
                self.leg_fail = self.leg_fail or self.plan_status
                self.next_leg()
            elif self.in_phase() > self.a.plan_timeout:
                self.ev(f"!! leg '{self.legs[self.leg_i][0]}' never reached a "
                        f"FRESH PLANNED (status '{self.plan_status}', "
                        f"replan seen: {self.saw_replan}) -- skipping")
                self.leg_fail = self.leg_fail or "plan timeout"
                self.next_leg()
        elif p == "LEG_EXEC":
            st = self.plan_state()
            if st == "EXECUTING":
                self.saw_exec = True
            # EXECUTING -> HOLD is completion. HOLD is also where the leg
            # started, so require EXECUTING to have been seen first -- without
            # it a Send that never took would read as an instant success.
            if st == "HOLD" and self.saw_exec and self.in_phase() > 2.0:
                self.ev(f"leg '{self.legs[self.leg_i][0]}' complete")
                self.goto("LEG_SETTLE")
            elif self.in_phase() > self.a.exec_timeout:
                self.ev(f"!! leg '{self.legs[self.leg_i][0]}' did not finish "
                        f"in {self.a.exec_timeout:.0f} s (status "
                        f"'{self.plan_status}', executing seen: "
                        f"{self.saw_exec}) -- moving on")
                self.leg_fail = self.leg_fail or "exec timeout"
                self.goto("LEG_SETTLE")
        elif p == "LEG_SETTLE":
            if self.in_phase() > self.a.hold_between:
                self.next_leg()
        elif p == "POST_HOLD":
            if self.in_phase() > self.a.hold_between:
                self.ev("mission complete -> SAFETY")
                self.call(self.cli_dir, SetBool.Request(data=False),
                          "set_direct_mode(false)")
                self.goto("ABORT_SETTLE")
        elif p == "ABORT_SETTLE":
            if self.in_phase() > self.a.abort_settle:
                self.goto("LAND")
        elif p == "LAND":
            # Land by reference. PX4 will not disarm this rig until its land
            # detector sees low thrust, which it never does -- the disarm below
            # is attempted, not relied on.
            self.ref_p = np.array([self.ref_p[0], self.ref_p[1], self.a.land_z])
            if self.in_phase() > self.a.land_wait:
                self.call(self.cli_dis, Trigger.Request(), "rc/disarm")
                self.goto("DONE")
        elif p == "DONE":
            if self.in_phase() > 3.0:
                self.done = True

    def next_leg(self):
        self.leg_i += 1
        if self.leg_i >= len(self.legs):
            self.leg_marks.append((self.now(), "post_hold"))
            self.goto("POST_HOLD")
        else:
            self.start_leg()
            self.goto("LEG_PLAN")

    # ---- output ----------------------------------------------------------
    def save(self, path):
        os.makedirs(os.path.dirname(os.path.abspath(path)) or ".",
                    exist_ok=True)
        log = np.array(self.log) if self.log else np.zeros((0, 17))
        # The debug array length differs between SAFETY (short) and DIRECT
        # (89), so pad to the widest row rather than assuming one length.
        if self.dbg_log:
            w = max(len(r) for r in self.dbg_log)
            dbg = np.full((len(self.dbg_log), w), np.nan)
            for i, r in enumerate(self.dbg_log):
                dbg[i, :len(r)] = r
        else:
            dbg = np.zeros((0, 1))
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
            t_direct=getattr(self, "t_direct", np.nan),
            hover_z=self.a.hover_z, soak=self.a.soak,
            leg_marks=np.array([f"{t:.3f} {n}" for t, n in self.leg_marks],
                               dtype=object),
            leg_fail="" if self.leg_fail is None else str(self.leg_fail),
            log_cols=np.array(
                ["t", "x", "y", "z", "vx", "vy", "vz", "tilt_deg", "yaw",
                 "ref_x", "ref_y", "ref_z", "ref_psi",
                 "q1", "q2", "q3", "q4", "direct"], dtype=object),
            dbg_cols=np.array(["t"] + [f"d{i}" for i in range(w if self.dbg_log else 0)],
                              dtype=object))
        print(f"\nwrote {path}  log {log.shape}  dbg {dbg.shape}", flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--namespace", default="/uav_0")
    ap.add_argument("--rate", type=float, default=50.0)
    ap.add_argument("--hover-z", type=float, default=1.0,
                    help="hover altitude [m]; the rig's standard test is 1 m")
    ap.add_argument("--land-z", type=float, default=0.35)
    ap.add_argument("--soak", type=float, default=20.0,
                    help="plain DIRECT hold before the steps [s]")
    ap.add_argument("--step-xy", type=float, default=0.5,
                    help="x and y step size [m]")
    ap.add_argument("--step-yaw", type=float, default=30.0,
                    help="yaw step [deg]")
    ap.add_argument("--both-base", type=float, nargs=3,
                    default=[0.35, 0.25, 0.12],
                    help="base translation [m] on the combined leg; the EE "
                         "target carries it too, so the arm reconfigures on "
                         "top of it rather than being dragged.")
    ap.add_argument("--both-yaw", type=float, default=20.0,
                    help="base heading change [deg] on the combined leg.")
    ap.add_argument("--ee-lat", type=float, default=0.08,
                    help="lateral EE offset [m] on the compatible-trajectory "
                         "leg; drives the arm-yaw joint q1. 0 = fold only.")
    ap.add_argument("--ee-yaw", type=float, default=60.0,
                    help="EE heading change [deg] on the compatible-"
                         "trajectory leg; drives the wrist joint q4.")
    ap.add_argument("--ee-step", type=float, default=0.06,
                    help="end-effector step for the compatible-trajectory leg "
                         "[m], applied DOWNWARD (-z) from the held pose. See "
                         "the note in build_legs for why down and why 0.06.")
    ap.add_argument("--plan-timeout", type=float, default=25.0,
                    help="seconds to wait for the planner to reach PLANNED")
    ap.add_argument("--exec-timeout", type=float, default=45.0,
                    help="seconds to wait for a Send to complete (HOLD)")
    ap.add_argument("--hold-between", type=float, default=6.0,
                    help="settle time after each leg completes [s]")
    ap.add_argument("--no-steps", action="store_true",
                    help="hover-only mission (what the 2026-09-06 sweep flew)")
    ap.add_argument("--direct-settle", type=float, default=20.0,
                    help="seconds after DIRECT entry excluded from the soak; "
                         "the observer needs tens of seconds to learn a 13% "
                         "thrust deficit with the GMO")
    ap.add_argument("--abort-settle", type=float, default=12.0)
    ap.add_argument("--land-wait", type=float, default=20.0)
    ap.add_argument("--abort-tilt", type=float, default=35.0)
    ap.add_argument("--out", default="wb_l1_run.npz")
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
        for line in d.events[-8:]:
            print(line)
        print("ABORTED: " + d.abort_reason if d.aborted else "completed")
        d.destroy_node()
        rclpy.shutdown()
    return 1 if d.aborted else 0


if __name__ == "__main__":
    raise SystemExit(main())
