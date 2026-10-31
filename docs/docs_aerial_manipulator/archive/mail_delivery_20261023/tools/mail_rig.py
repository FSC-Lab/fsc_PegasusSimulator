#!/usr/bin/env python3
"""The Mail Envelope tab, end to end WITHOUT Isaac (2026-10-23): the REAL planner
(fsc_trajectory_planner) on the mail-delivery yaml and the REAL arm-GS tab
(gs_harness/gs_mail_snapshot, offscreen), flown on a perfect-plant loopback rig
that PRESSES THE TAB'S OWN BUTTONS through the operator's workflow and
photographs the tab at each step.

    mail_rig.py <params yaml> <out dir>      (run through mail_gs_check.sh: ROS domain 77)

A copy of pick_and_drop_workflow_20261019/tools/pnd_rig.py, for the INSERT place:
odometry, PX4 attitude, joint states and /obj_0/mocap fed at 50 Hz; after each leg
the vehicle is teleported onto the leg's goal. The envelope lies on the pick hat
until the grasp, then rides 60 mm ahead of the claw (its centre), and after the
insert's OPEN it is left in the mailbox. Checks, beyond "the mission completes":
the tab is ACTIVE only after its own Plan (Pick & Place / Pick & Drop modes off);
the plan's arm pose has q4 = 90 deg from the pick to the place; Ready To Insert
holds the claw the Insert Margin BEHIND the place target at its height; the
station closes the gripper only on the target, opens it only on the place target,
starts Exit To Insert only after the OPEN, and closes the gripper after the exit.
"""
import math
import sys
import time

import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleAttitude
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from fsc_autopilot_ros2_msgs.msg import Mocap, WholeBodyReference

NS = "/uav_mailshot"
SIGN = np.ones(4)                                  # the SIM yaml's arm_joint_sign
HOME = np.radians([0.0, 40.0, 40.0, 0.0])
GROUND_Z = 0.235                                   # resting body height, T650 gear
START_MARK = np.array([0.10, -0.07])               # the scene's spawn: Adjust measures (+0.10, -0.07)
ENVELOPE = np.array([0.9047, 1.0, 0.780])          # its centre on the pick hat (07 mail-delivery scene)
AHEAD = np.array([0.06, 0.0, 0.0])                 # the envelope's centre - the claw, vehicle frame, once held
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
FAILS = []
KICK_BACK = float(__import__("os").environ.get("MAIL_RIG_KICK_BACK", "0.03"))   # [m], 0 = no kick
# (2026-10-30, the user's windowed rehearsal hovered in the mouth, never released) the wrist PARKED
# off 90 deg once the envelope is in the mouth, as flown (q4 93.6-94.5 deg the whole hold) [deg];
# and PRESS_RELEASE = 1: if the planner's release does not come within 6 s, press 4.2 Insert (the
# manual release) and expect the station to open and start Exit To Insert by itself
Q4_OFF_DEG = float(__import__("os").environ.get("MAIL_RIG_Q4_OFF_DEG", "0"))
PRESS_RELEASE = __import__("os").environ.get("MAIL_RIG_PRESS_RELEASE", "0") == "1"
# (2026-10-30, the release WINDOW) the claw parked off the release point after the slide: "along,off" [m]
# along the insertion line (+ = deeper) and sideways off it
RELEASE_OFF = [float(v) for v in __import__("os").environ.get("MAIL_RIG_RELEASE_OFF", "0,0").split(",")]


def check(ok, what):
    print(f"  [{'PASS' if ok else 'FAIL'}] {what}")
    if not ok:
        FAILS.append(what)


def rz(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


class Rig(Node):
    def __init__(self):
        super().__init__("mail_check_rig", namespace=NS)
        self.mode = self.create_publisher(String, "fsc_autopilot_ros2/whole_body_direct_actuation/mode", LATCHED)
        self.odom = self.create_publisher(Odometry, "state_estimator/local_position/odom", 10)
        self.att = self.create_publisher(VehicleAttitude, "fmu/out/vehicle_attitude",
                                         QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                                                    durability=DurabilityPolicy.VOLATILE))
        self.js = self.create_publisher(JointState, "fsc_open_manipulator/joint_states", 10)
        self.obj = self.create_publisher(Mocap, "/obj_0/mocap", 10)
        self.cmd = self.create_publisher(String, "/pndshot/cmd", 10)
        self.acks = []
        self.create_subscription(String, "/pndshot/ack", lambda m: self.acks.append(m.data), 10)
        self.pp_status, self.info, self.refs, self.arr = None, None, [], None
        self.create_subscription(Float64MultiArray, "whole_body_planner/pick_place/arrival_error",
                                 lambda m: setattr(self, "arr", np.asarray(m.data)), 10)
        self.create_subscription(String, "whole_body_planner/pick_place/status",
                                 lambda m: setattr(self, "pp_status", m.data), LATCHED)
        self.create_subscription(Float64MultiArray, "whole_body_planner/pick_place/info",
                                 lambda m: setattr(self, "info", np.asarray(m.data)), LATCHED)
        self.create_subscription(WholeBodyReference, "fsc_autopilot_ros2/whole_body_direct_actuation/reference",
                                 self.on_ref, 50)
        # the CoM reference with the planner state at its arrival (2026-10-30: the exit's start point)
        self.xcd = []
        # the release KICK (2026-10-30, the mail crash): on the planner's release signal the vehicle
        # is moved kick_back m BEHIND the hold, as the released vehicle sits in Isaac
        self.kick_back, self.kicked = 0.0, False
        self.create_subscription(String, "whole_body_planner/pick_place/drop_now", self.on_drop_now, 10)
        self.params = self.create_client(SetParameters, NS + "/whole_body_trajectory_planner/set_parameters")
        self.xyz, self.yaw, self.q = np.array([*START_MARK, GROUND_Z]), 0.0, HOME.copy()
        self.obj_mode, self.obj_xyz, self.claw_body = "fixed", ENVELOPE.copy(), None
        self.rng = np.random.default_rng(0)
        self.create_timer(0.02, self.feed)

    def on_ref(self, m):
        self.refs.append(np.array(m.q_d[:4]))
        self.xcd.append((self.pp_status or "", np.array([m.x_cd.x, m.x_cd.y, m.x_cd.z])))

    def on_drop_now(self, m):
        if self.kick_back > 0.0 and not self.kicked:
            self.xyz = self.xyz - self.kick_back * np.array([math.cos(self.yaw), math.sin(self.yaw), 0.0])
            self.kicked = True

    def claw(self):
        return self.xyz + rz(self.yaw) @ self.claw_body

    def feed(self):
        o = Odometry()
        o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = map(float, self.xyz)
        o.pose.pose.orientation.w = 1.0
        self.odom.publish(o)
        a = VehicleAttitude()
        ned = 0.5 * math.pi - self.yaw
        a.q = [math.cos(0.5 * ned), 0.0, 0.0, math.sin(0.5 * ned)]
        self.att.publish(a)
        j = JointState()
        j.name = ["joint1", "joint2", "joint3", "joint4"]
        j.position = list(map(float, SIGN * self.q))
        j.velocity = [0.0] * 4
        self.js.publish(j)
        m = Mocap()
        p = self.claw() + rz(self.yaw) @ AHEAD if self.obj_mode == "held" else self.obj_xyz
        p = p + self.rng.normal(0.0, 0.0005, 3)
        m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, p)
        m.pose.orientation.w = 1.0
        self.obj.publish(m)

    def teleport(self, leg):
        g = self.info[14 + 7 * leg: 21 + 7 * leg]
        self.xyz = np.array(g[0:3])
        self.yaw = math.radians(g[3])
        self.q = np.array(self.refs[-1])
        spin_for(self, 0.6)


def spin_for(rig, s):
    t = time.monotonic() + s
    while time.monotonic() < t:
        rclpy.spin_once(rig, timeout_sec=0.01)


def gs(rig, cmd, must=True):
    n = len(rig.acks)
    m = String(); m.data = cmd
    rig.cmd.publish(m)
    t = time.monotonic() + 10.0
    while time.monotonic() < t:
        rclpy.spin_once(rig, timeout_sec=0.01)
        for a in rig.acks[n:]:
            if a.endswith(cmd) or (" " + cmd + ":") in a:
                print(f"  GS {a}")
                if must and not a.startswith("ok"):
                    sys.exit(f"GS refused: {a}")
                return a.startswith("ok")
    sys.exit(f"no ack for {cmd}")


def wait_status(rig, prefixes, timeout):
    t = time.monotonic() + timeout
    while time.monotonic() < t:
        rclpy.spin_once(rig, timeout_sec=0.02)
        s = rig.pp_status or ""
        if any(s.startswith(p) for p in prefixes):
            return s
    sys.exit(f"timeout waiting for {prefixes}; status {rig.pp_status!r}")


def wait_until(rig, pred, timeout, what):
    t = time.monotonic() + timeout
    while time.monotonic() < t:
        rclpy.spin_once(rig, timeout_sec=0.02)
        if pred():
            return True
    sys.exit(f"timeout waiting for {what}; status {rig.pp_status!r}")


def set_param(rig, name, value):
    rig.params.wait_for_service(timeout_sec=5.0)
    req = SetParameters.Request()
    if isinstance(value, bool):
        pv = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=value)
    else:
        pv = ParameterValue(type=ParameterType.PARAMETER_DOUBLE_ARRAY, double_array_value=[float(v) for v in value])
    req.parameters.append(Parameter(name=name, value=pv))
    f = rig.params.call_async(req)
    rclpy.spin_until_future_complete(rig, f, timeout_sec=5.0)
    ok = f.result() is not None and all(r.successful for r in f.result().results)
    print(f"  set {name} = {value}: {'OK' if ok else 'REFUSED'}")
    if not ok:
        sys.exit(1)


def main():
    p = yaml.safe_load(open(sys.argv[1]))["/**/whole_body_trajectory_planner"]["ros__parameters"]
    assert p.get("pick_place_place_insert") and not p.get("pick_place_place_drop"), "not the mail yaml"
    back = float(p["pick_place_place_approach_back"])
    # (2026-10-30) obj_0 is the LOCALIZATION FRAME when the yaml carries the locators: the rig puts it
    # where the frame's markers would be, so Get + the locator lands on the envelope / the mailbox target
    ploc = np.asarray(p.get("pick_place_pick_locator") or [0.0, 0.0, 0.0])[:3]
    qloc = np.asarray(p.get("pick_place_place_locator") or [0.0, 0.0, 0.0])[:3]
    place = np.asarray(p["pick_place_place_point"][:3], float)
    auto_insert = bool(p.get("pick_place_insert_auto"))
    rclpy.init()
    rig = Rig()
    m = String(); m.data = "SAFETY"
    rig.mode.publish(m)
    spin_for(rig, 4.0)                          # the station loads the planner's values

    # the tab must NOT be active while the planner flies another place: switch to the
    # set-down, see the tab say so, and let ITS Plan switch it back (later)
    set_param(rig, "pick_place_place_insert", False)
    spin_for(rig, 1.5)
    gs(rig, "snap:00_other_mode")

    # --- on the ground: the frame on the pick hat -> the Envelope row's Get; then on the
    # mailbox -> the Mailbox row's Get (Plan is refused without it since 2026-10-30)
    rig.obj_xyz = ENVELOPE - ploc
    spin_for(rig, 1.0)
    gs(rig, "click:Get#0")
    spin_for(rig, 2.0)
    gs(rig, "snap:01_envelope_get")
    rig.obj_xyz = place - qloc
    spin_for(rig, 1.0)
    gs(rig, "click:Get#1")
    spin_for(rig, 2.0)
    gs(rig, "click:Adjust")
    spin_for(rig, 2.0)
    gs(rig, "snap:02_adjust")

    # --- take off (SAFETY), switch to DIRECT, Plan IN THE MAIL TAB
    rig.xyz = np.array([*START_MARK, 1.0])
    spin_for(rig, 1.0)
    m.data = "DIRECT"
    rig.mode.publish(m)
    wait_status(rig, ["NOT PLANNED"], 15)
    spin_for(rig, 1.5)
    gs(rig, "click:Plan")
    wait_status(rig, ["READY"], 40)
    spin_for(rig, 1.5)
    gs(rig, "snap:03_ready")
    info = rig.info
    print("  leg durations", np.round(info[8:14], 1).tolist(), "s; exits", np.round(info[81:83], 1).tolist(), "s")
    xb3, yaw3, ee3 = info[14 + 21:17 + 21], math.radians(info[17 + 21]), info[18 + 21:21 + 21]
    rig.claw_body = rz(yaw3).T @ (np.asarray(ee3) - np.asarray(xb3))
    sys.path.insert(0, __import__("os").path.dirname(__import__("os").path.abspath(__file__)))
    import mail_geometry as G              # the planner's own arm model
    _, r0e, _ = G.TP.arm_fk_model(np.radians(p["pick_place_place_pose_deg"]), G.TP.make_params_t650())
    want_b = np.array([r0e[1], -r0e[0], r0e[2]])
    check(np.linalg.norm(rig.claw_body - want_b) < 0.005,
          f"claw in the body frame at the place {np.round(rig.claw_body, 4).tolist()} "
          f"(the place pose's model FK {np.round(want_b, 4).tolist()})")

    # --- 1 Go To Start
    gs(rig, "click:1 Go To Start")
    wait_status(rig, ["DONE go_to_start"], 60)
    rig.teleport(0)
    # --- 2.1 Ready To Pick: the gripper opens first; the hold BEHIND the envelope's edge
    gs(rig, "click:2.1 Ready To Pick")
    wait_status(rig, ["WAITING execute_pick"], 60)
    rig.teleport(1)
    spin_for(rig, 0.6)
    rig.xyz = rig.xyz + rig.arr[3:6]            # onto the approach rest (the planner's own claw error)
    spin_for(rig, 1.5)
    check(abs(np.degrees(rig.q[3]) - 90.0) < 0.5, f"Ready To Pick: wrist roll q4 = {np.degrees(rig.q[3]):.1f} deg")
    gs(rig, "snap:04_ready_to_pick")
    # --- 2.2 Pick: slide in; the station closes ON the target; Exit To Pick by itself
    gs(rig, "click:2.2 Pick")
    wait_status(rig, ["DONE execute_pick"], 40)
    rig.teleport(1)
    wait_until(rig, lambda: (rig.pp_status or "").startswith("FLYING exit_pick"), 15, "Exit To Pick")
    rig.obj_mode = "held"
    check(True, "Pick: closed on the target, Exit To Pick started by itself")
    spin_for(rig, 0.4)
    gs(rig, "snap:05_exit_pick")
    wait_status(rig, ["DONE exit_pick"], 30)
    rig.xyz = rig.xyz + np.array([0.0, 0.0, p.get("pick_place_approach_dz", 0.20)])
    spin_for(rig, 0.6)
    # --- 3 Go To Mailbox Start
    gs(rig, "click:3 Go To Mailbox Start")
    wait_status(rig, ["DONE go_to_place_start"], 60)
    rig.teleport(2)
    check(abs(np.degrees(rig.q[3]) - 90.0) < 0.5, f"carried with q4 = {np.degrees(rig.q[3]):.1f} deg")
    # --- 4.1 Ready To Insert: the hold the Insert Margin back along the insertion line (level, or
    # pitched pick_place_place_insert_pitch_deg down: back AND up, 2026-10-30)
    gs(rig, "click:4.1 Ready To Insert")
    wait_status(rig, ["WAITING execute_place"], 60)
    rig.teleport(3)
    spin_for(rig, 0.6)
    e = np.asarray(rig.arr[3:6])                 # (approach claw) - (goal claw): the rig sits on the goal
    nose = np.array([math.cos(rig.yaw), math.sin(rig.yaw), 0.0])
    th = math.radians(float(p.get("pick_place_place_insert_pitch_deg", 0.0)))
    u = math.cos(th) * nose + np.array([0.0, 0.0, -math.sin(th)])     # the insertion line, into the box
    left = np.array([-math.sin(rig.yaw), math.cos(rig.yaw), 0.0])
    check(np.linalg.norm(e + back * u) < 0.002,
          f"Ready To Insert holds {-float(np.dot(e, u)) * 1e3:.0f} mm back along the insertion line "
          f"({math.degrees(th):.0f} deg down: {-float(np.dot(e, nose)) * 1e3:.0f} mm back, {e[2] * 1e3:.0f} mm up; "
          f"off the line {np.linalg.norm(e - np.dot(e, u) * u) * 1e3:.1f} mm; Insert Margin {back * 1e3:.0f})")
    rig.xyz = rig.xyz + e                        # onto the approach rest
    spin_for(rig, 1.5)
    gs(rig, "snap:06_ready_to_insert")
    # --- 4.2 Insert: slide in; the station OPENS on the target; Exit To Insert on the OPEN report.
    # With pick_place_insert_auto (2026-10-30) nothing is pressed: the planner slides in once the
    # claw is aligned and still, signals the release once it is still at the target, the station
    # opens on that and starts Exit To Insert
    if not auto_insert:
        gs(rig, "click:4.2 Insert")
    wait_status(rig, ["DONE execute_place"], 40)
    rig.teleport(3)
    rig.q[3] += math.radians(Q4_OFF_DEG)
    rig.xyz = rig.xyz + RELEASE_OFF[0] * u + RELEASE_OFF[1] * left      # the claw parked off the release point
    rig.kick_back = KICK_BACK
    n_hold = len(rig.xcd)
    q4_tol = float(p.get("pick_place_insert_auto_q4_tol_deg", p.get("pick_place_drop_auto_q4_tol_deg", 3.0)))
    rel_short = float(p.get("pick_place_insert_auto_release_short", 0.015))
    rel_long = float(p.get("pick_place_insert_auto_release_long", 0.020))
    rel_off = float(p.get("pick_place_insert_auto_release_off_axis", 0.030))
    pressed = False
    if auto_insert and PRESS_RELEASE:
        t_end = time.monotonic() + 6.0
        while time.monotonic() < t_end and not (rig.pp_status or "").startswith("FLYING exit_place"):
            rclpy.spin_once(rig, timeout_sec=0.02)
        if not (rig.pp_status or "").startswith("FLYING exit_place"):
            st = rig.pp_status or ""
            print(f"  no release after 6 s -- status: {st[st.find('auto release'):][:110]!r}")
            why = ("wrist q4" if Q4_OFF_DEG > q4_tol else
                   "short of" if RELEASE_OFF[0] < -rel_short else "past" if RELEASE_OFF[0] > rel_long else
                   "off the insertion line" if abs(RELEASE_OFF[1]) > rel_off else "?")
            check(why != "?" and why in st, f"the planner's release is held, as expected, by '{why}'")
            gs(rig, "snap:06b_release_waiting")
            gs(rig, "click:4.2 Insert")          # the manual release: only clickable when enabled
            rig.on_drop_now(None)                # the release kick: the open, not the signal, causes it
            pressed = True
            # Exit To Insert is LOCKED until the gripper reports OPEN (0.7 s in the harness)
            spin_for(rig, 0.2)
            check(not gs(rig, "click:4.3 Exit To Insert", must=False),
                  "Exit To Insert is locked between the release and the gripper's OPEN report")
    wait_until(rig, lambda: (rig.pp_status or "").startswith("FLYING exit_place"), 20, "Exit To Insert")
    if auto_insert and not pressed:
        inside = (Q4_OFF_DEG <= q4_tol and -rel_short <= RELEASE_OFF[0] <= rel_long
                  and abs(RELEASE_OFF[1]) <= rel_off)
        check(inside, f"released BY THE PLANNER with the wrist {Q4_OFF_DEG:.1f} deg off (gate {q4_tol:.1f}), "
              f"the claw {RELEASE_OFF[0] * 1e3:+.0f} mm along / {RELEASE_OFF[1] * 1e3:+.0f} mm off the line "
              f"(window -{rel_short * 1e3:.0f}/+{rel_long * 1e3:.0f}, {rel_off * 1e3:.0f})")
    check(True, "Insert: " + ("slid in by the planner, released BY HAND (4.2 Insert), " if pressed else
                              "slid in and released BY THE PLANNER (nothing pressed), " if auto_insert
                              else "opened on the target, ") + "Exit To Insert started by itself after the OPEN")
    rig.obj_mode, rig.obj_xyz = "fixed", rig.claw() + rz(rig.yaw) @ (AHEAD + np.array([0.0, 0.0, -0.15]))
    spin_for(rig, 0.4)
    gs(rig, "snap:07_exit_insert")
    wait_status(rig, ["DONE exit_place"], 30)
    # THE EXIT STARTS WHERE THE VEHICLE IS (2026-10-30): after a release kick of KICK_BACK m the
    # exit's CoM reference must start that far behind the hold and never come back toward the box
    hold_x = [x for st, x in rig.xcd[n_hold:] if not st.startswith("FLYING exit_place")]
    ex_x = [x for st, x in rig.xcd[n_hold:] if st.startswith("FLYING exit_place")]
    if KICK_BACK > 0.0 and hold_x and ex_x:
        # the hold keeps streaming while the exit is planned: the exit plan begins at the first
        # sample that leaves the hold's CoM reference
        h = hold_x[-1]
        k0 = next((k for k, x in enumerate(ex_x) if np.linalg.norm(x - h) > 0.0005), len(ex_x) - 1)
        a = np.array([np.dot(x, nose) for x in ex_x[k0:]])
        sat = float(np.dot(RELEASE_OFF[0] * u + RELEASE_OFF[1] * left, nose))   # where the rig parked the claw
        start = a[0] - np.dot(h, nose) - sat
        check(rig.kicked and abs(start + KICK_BACK) < 0.004,
              f"release kick {KICK_BACK * 1e3:.0f} mm back: the exit's CoM reference starts "
              f"{start * 1e3:+.1f} mm along the nose from the hold's (want {-KICK_BACK * 1e3:.0f})")
        # the danger was never the reference moving forward but the reference starting AHEAD of
        # the vehicle: measure it against where the vehicle sat (the hold less the kick)
        ahead = np.max(a) - (np.dot(h, nose) + sat - KICK_BACK)
        check(ahead < 0.001,
              f"the exit never asks the vehicle toward the mailbox: the CoM reference is at most "
              f"{ahead * 1e3:+.1f} mm ahead of where the vehicle sat ({len(a)} reference samples)")
    rig.xyz = rig.xyz - p.get("pick_place_place_exit_back", 0.10) * nose + np.array([0.0, 0.0, p.get("pick_place_approach_dz", 0.20)])
    spin_for(rig, 1.0)
    # --- 5 Go To Land Start (the gripper closed after the exit), 6 Execute To Land
    gs(rig, "click:5 Go To Land Start")
    wait_status(rig, ["DONE go_to_land_start"], 60)
    rig.teleport(4)
    check(abs(np.degrees(rig.q[3])) < 0.5, f"Land Start: the arm folded home, q4 = {np.degrees(rig.q[3]):.1f} deg")
    gs(rig, "click:6 Execute To Land")
    wait_status(rig, ["COMPLETE"], 40)
    spin_for(rig, 1.0)
    gs(rig, "snap:08_complete")
    gs(rig, "quit", must=False)
    rig.destroy_node()
    rclpy.shutdown()
    print("ALL PASS" if not FAILS else f"FAILED: {FAILS}")
    return 1 if FAILS else 0


if __name__ == "__main__":
    raise SystemExit(main())
