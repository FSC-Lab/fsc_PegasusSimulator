#!/usr/bin/env python3
"""Synthetic-pad Isaac check of PS4 teleoperation on an AM-T650 rig (2026-10-01).

    FASTRTPS_DEFAULT_PROFILES_FILE=.../fastdds_udp_only.xml \
      /usr/bin/python3 ps4_pad_check.py --out run.npz

Run it with the vehicle hovering in DIRECT and the planner in HOLD (after
`ps4_teleop_bringup.py up`), and with NO real pad running -- it publishes the
pad itself on <ns>/rc/input and refuses to start if anything else does.

It engages teleoperation through the ARM STATION's own service
(fsc_open_manipulator/ps4_remote/set_engaged -- what the tab's button calls;
falls back to the planner's teleop/engage when the station is absent), then
drives a DualShock 4 the way joy_node + gamepad_input would, one channel at a
time: PS (pad home), D-pad forward/left/back/right, triangle/cross, square/
circle, both sticks, a dropped pad, release. For every leg it compares the
planner's TARGET change (teleop/state) with the MEASURED change (odometry for
the airframe, the planner's current_ee_body for the grasp point, joint_states
for the joints) and scores direction and size; over the whole run it records
the controller's mode (a SAFETY revert fails the run), the tilt, the tracking
error against the reference the law is actually flying and -- on the decoupled
rig -- the reference bridge's same-motion residual (its vector [23]).
"""
import argparse
import math
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import SetBool

from fsc_autopilot_ros2_msgs.msg import PositionControllerReference

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
KNOWN_DA = ("whole_body_direct_actuation", "geometric_l1_direct_actuation")
PAD_HOME_DEG = np.array([0.0, 30.0, 30.0, 0.0])


def pad(**kw):
    """DualShock 4 on joy_node: 8 axes (0/1 left stick, 2 L2, 3/4 right stick,
    5 R2, 6/7 D-pad), 13 buttons (0 cross, 1 circle, 2 triangle, 3 square,
    10 PS). Triggers rest at +1. a<i>=value / b<i>=1 set one channel."""
    j = Joy()
    j.axes = [0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0]
    j.buttons = [0] * 13
    for k, v in kw.items():
        if k.startswith("a"):
            j.axes[int(k[1:])] = float(v)
        else:
            j.buttons[int(k[1:])] = int(v)
    return j


def yaw_of(q):
    x, y, z, w = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def tilt_of(q):
    x, y, _, _ = q
    r22 = 1.0 - 2.0 * (x * x + y * y)
    return math.degrees(math.acos(max(-1.0, min(1.0, r22))))


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class Check(Node):
    def __init__(self, a, da):
        super().__init__("ps4_pad_check")
        ns = a.namespace.rstrip("/")
        self.ns, self.da = ns, da
        self.mode = ""
        self.status = ""
        self.note = ""
        self.tstate = []
        self.odom = None
        self.ee_body = None
        self.q = None
        self.ref = None
        self.vec = None
        self.joy = None
        self.reverted = False
        # time series (wall clock, s since start)
        self.t0 = time.time()
        self.log = {k: [] for k in ("odom", "ref", "vec", "tstate", "q", "ee_body", "mode")}
        self.create_subscription(String, f"{ns}/fsc_autopilot_ros2/{da}/mode", self.on_mode, LATCHED)
        self.create_subscription(String, f"{ns}/whole_body_planner/status",
                                 lambda m: setattr(self, "status", m.data), LATCHED)
        self.create_subscription(String, f"{ns}/whole_body_planner/teleop/note",
                                 lambda m: setattr(self, "note", m.data), LATCHED)
        self.create_subscription(Float64MultiArray, f"{ns}/whole_body_planner/teleop/state",
                                 self.on_tstate, 10)
        self.create_subscription(Odometry, f"{ns}/state_estimator/local_position/odom", self.on_odom, 10)
        self.create_subscription(PoseStamped, f"{ns}/whole_body_planner/current_ee_body", self.on_ee_body, 10)
        self.create_subscription(JointState, f"{ns}/fsc_open_manipulator/joint_states", self.on_js, 10)
        self.create_subscription(PositionControllerReference,
                                 f"{ns}/fsc_autopilot_ros2/position_controller/reference_direct",
                                 self.on_ref, 50)
        self.create_subscription(Float64MultiArray, f"{ns}/decoupled_bridge/base_reference_vector",
                                 self.on_vec, 50)
        self.joy_pub = self.create_publisher(Joy, f"{ns}/rc/input", 10)
        self.gs_engage = self.create_client(SetBool, f"{ns}/fsc_open_manipulator/ps4_remote/set_engaged")
        self.pl_engage = self.create_client(SetBool, f"{ns}/whole_body_planner/teleop/engage")
        self.create_timer(1.0 / 25.0, self.on_pad_tick)    # joy_node's autorepeat order

    # ── callbacks ────────────────────────────────────────────────────────────
    def now(self):
        return time.time() - self.t0

    def on_mode(self, m):
        if self.mode == "DIRECT" and m.data != "DIRECT":
            self.reverted = True
        self.mode = m.data
        self.log["mode"].append((self.now(), m.data))

    def on_tstate(self, m):
        self.tstate = list(m.data)
        if self.tstate:
            self.log["tstate"].append([self.now()] + self.tstate[:35])

    def on_odom(self, m):
        p, v, o = m.pose.pose.position, m.twist.twist.linear, m.pose.pose.orientation
        self.odom = np.array([p.x, p.y, p.z, v.x, v.y, v.z, o.x, o.y, o.z, o.w])
        self.log["odom"].append([self.now(), *self.odom])

    def on_ee_body(self, m):
        p = m.pose.position
        self.ee_body = np.array([p.x, p.y, p.z])
        self.log["ee_body"].append([self.now(), *self.ee_body])

    def on_js(self, m):
        d = dict(zip(m.name, m.position))
        if all(f"joint{i}" in d for i in range(1, 5)):
            self.q = np.array([d[f"joint{i}"] for i in range(1, 5)])   # sim sign map = +1
            self.log["q"].append([self.now(), *self.q])

    def on_ref(self, m):
        y = m.yaw if m.yaw_unit == PositionControllerReference.RADIANS else math.radians(m.yaw)
        self.ref = np.array([m.position.x, m.position.y, m.position.z, y])
        self.log["ref"].append([self.now(), *self.ref])

    def on_vec(self, m):
        self.vec = list(m.data)
        self.log["vec"].append([self.now()] + self.vec[:24])

    def on_pad_tick(self):
        if self.joy is not None:
            self.joy.header.stamp = self.get_clock().now().to_msg()
            self.joy_pub.publish(self.joy)

    # ── helpers ──────────────────────────────────────────────────────────────
    def say(self, s):
        print(f"[{self.now():6.1f}s] {s}", flush=True)

    def spin(self, seconds):
        t_end = time.time() + seconds
        while time.time() < t_end:
            rclpy.spin_once(self, timeout_sec=0.01)

    def wait(self, cond, timeout):
        t_end = time.time() + timeout
        while time.time() < t_end:
            if cond():
                return True
            rclpy.spin_once(self, timeout_sec=0.02)
        return False

    def call(self, cli, data, timeout=5.0):
        if not cli.wait_for_service(timeout_sec=timeout):
            return None
        f = cli.call_async(SetBool.Request(data=data))
        self.wait(f.done, timeout)
        return f.result()

    def ts(self, i):
        return self.tstate[i] if len(self.tstate) > i else float("nan")

    def snap(self):
        """What a leg compares: targets (teleop/state) and measurements."""
        o = self.odom
        return {
            "com_t": np.array([self.ts(1), self.ts(2), self.ts(3)]),
            "yaw_t": self.ts(4),
            "s_t": np.array([self.ts(19), self.ts(20), self.ts(21)]),
            "q_t": np.array([self.ts(15), self.ts(16), self.ts(17), self.ts(18)]),
            "p": o[:3].copy(), "yaw": yaw_of(o[6:10]),
            "ee": self.ee_body.copy() if self.ee_body is not None else np.full(3, np.nan),
            "q": self.q.copy() if self.q is not None else np.full(4, np.nan),
            "t": self.now(),
        }

    def window_stats(self, t_a, t_b):
        """Tilt and tracking error vs the reference the law flies, over [t_a, t_b]."""
        od = np.array([r for r in self.log["odom"] if t_a <= r[0] <= t_b])
        rf = np.array([r for r in self.log["ref"] if t_a <= r[0] <= t_b])
        vc = np.array([r for r in self.log["vec"] if t_a <= r[0] <= t_b])
        out = {"tilt_max": float("nan"), "e_pos_max": float("nan"), "e_yaw_max": float("nan"),
               "ee_res_max": float("nan")}
        if len(od):
            out["tilt_max"] = max(tilt_of(r[7:11]) for r in od)
        if len(od) and len(rf):
            ip = np.searchsorted(rf[:, 0], od[:, 0]).clip(1, len(rf)) - 1
            e = od[:, 1:4] - rf[ip, 1:4]
            out["e_pos_max"] = float(np.max(np.linalg.norm(e, axis=1)))
            ey = [abs(wrap(yaw_of(r[7:11]) - rf[i, 4])) for r, i in zip(od, ip)]
            out["e_yaw_max"] = math.degrees(max(ey))
        if len(vc):
            out["ee_res_max"] = float(np.max(np.abs(vc[:, 24]))) * 1e3
        return out


# ── the legs ─────────────────────────────────────────────────────────────────
# (name, pad kwargs, press seconds, settle seconds, what to compare)
# kind: com_fwd / com_left / com_up are projections of the CoM change on the
# heading-frame axes, yaw the heading change, ee_* the grasp point vs airframe
# (actual body frame: x nose, y left, z up), q4 the wrist roll, home the pose.
LEGS = [
    ("PS: arm to the pad home",   {"b10": 1}, 0.3, 9.0, "home"),
    ("D-pad up: CoM forward",     {"a7": 1.0}, 2.5, 6.0, "com_fwd+"),
    ("D-pad left: CoM left",      {"a6": 1.0}, 2.5, 6.0, "com_left+"),
    ("D-pad down: CoM back",      {"a7": -1.0}, 2.5, 6.0, "com_fwd-"),
    ("D-pad right: CoM right",    {"a6": -1.0}, 2.5, 6.0, "com_left-"),
    ("triangle: CoM up",          {"b2": 1}, 2.0, 5.0, "com_up+"),
    ("cross: CoM down",           {"b0": 1}, 2.0, 5.0, "com_up-"),
    ("square: yaw left",          {"b3": 1}, 2.25, 6.0, "yaw+"),
    ("circle: yaw right",         {"b1": 1}, 2.25, 6.0, "yaw-"),
    ("left stick left: EE left",  {"a0": 1.0}, 1.5, 4.0, "ee_y+"),
    ("left stick right: EE right", {"a0": -1.0}, 1.5, 4.0, "ee_y-"),
    ("right stick down: EE down", {"a4": -1.0}, 1.0, 4.0, "ee_z-"),
    ("right stick up: EE up",     {"a4": 1.0}, 1.0, 4.0, "ee_z+"),
    ("left stick up: EE forward", {"a1": 1.0}, 0.6, 4.0, "ee_x+"),
    ("left stick down: EE back",  {"a1": -1.0}, 0.6, 4.0, "ee_x-"),
    ("right stick left: roll",    {"a3": 1.0}, 1.5, 4.0, "q4"),
    ("right stick right: roll",   {"a3": -1.0}, 1.5, 4.0, "q4"),
]


def score(kind, s0, s1):
    """(target change, measured change, unit, ok)."""
    psi = s0["yaw"]
    fwd = np.array([math.cos(psi), math.sin(psi), 0.0])
    left = np.array([-math.sin(psi), math.cos(psi), 0.0])
    if kind == "home":
        err = np.max(np.abs(np.degrees(s1["q"]) - PAD_HOME_DEG))
        tgt = np.max(np.abs(np.degrees(s1["q_t"]) - PAD_HOME_DEG))
        return tgt, err, "deg off pad home (target, measured)", bool(tgt < 0.5 and err < 3.0)
    base, sign = kind[:-1], (1.0 if kind[-1] == "+" else -1.0)
    if kind == "q4":
        dt = math.degrees(s1["q_t"][3] - s0["q_t"][3])
        dm = math.degrees(s1["q"][3] - s0["q"][3])
        return dt, dm, "deg", bool(abs(dt) > 10.0 and dm * dt > 0 and abs(dm) > 0.5 * abs(dt))
    if base.startswith("com_"):
        axis = {"com_fwd": fwd, "com_left": left, "com_up": np.array([0.0, 0.0, 1.0])}[base]
        dt = float(np.dot(s1["com_t"] - s0["com_t"], axis))
        dm = float(np.dot(s1["p"] - s0["p"], axis))
        return dt, dm, "m", bool(sign * dt > 0.15 and sign * dm > 0.5 * sign * dt)
    if base == "yaw":
        dt = math.degrees(wrap(s1["yaw_t"] - s0["yaw_t"]))
        dm = math.degrees(wrap(s1["yaw"] - s0["yaw"]))
        return dt, dm, "deg", bool(sign * dt > 20.0 and sign * dm > 0.5 * sign * dt)
    i = {"ee_x": 0, "ee_y": 1, "ee_z": 2}[base]
    dt = float(s1["s_t"][i] - s0["s_t"][i])
    dm = float(s1["ee"][i] - s0["ee"][i])
    # fore/aft is wall-limited to ~1.5 cm from the pad home: score direction only there
    need = 0.004 if base == "ee_x" else 0.015
    return dt, dm, "m", bool(sign * dt > need and sign * dm > 0.5 * sign * dt)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--namespace", default="/uav_0")
    ap.add_argument("--da", default="auto", choices=("auto",) + KNOWN_DA)
    ap.add_argument("--out", default="")
    a = ap.parse_args()
    rclpy.init()
    n = None
    rc = 1
    try:
        n = Check(a, a.da if a.da != "auto" else "geometric_l1_direct_actuation")
        if a.da == "auto":
            n.spin(3.0)
            live = [d for d in KNOWN_DA
                    if n.count_publishers(f"{n.ns}/fsc_autopilot_ros2/{d}/mode") > 0]
            if len(live) != 1:
                n.say(f"--da auto: found {live or 'none'} -- pass --da"); return 1
            if live[0] != n.da:
                n.destroy_node()
                n = Check(a, live[0])
        n.say(f"controller namespace: {n.da}")
        if not n.wait(lambda: n.odom is not None and n.mode and n.status and n.q is not None
                      and n.ee_body is not None, 30):
            n.say("FAIL: no odometry / mode / planner status / joints / current_ee_body"); return 1
        if n.mode != "DIRECT" or not n.status.startswith("HOLD"):
            n.say(f"FAIL: need DIRECT + planner HOLD (mode {n.mode}, planner {n.status})"); return 1
        others = n.count_publishers(f"{n.ns}/rc/input") - 1     # minus this node's own
        if others > 0:
            n.say(f"FAIL: {others} other publisher(s) on rc/input -- stop the real pad first"); return 1

        # ── engage ──
        n.joy = pad()
        n.spin(1.0)
        r = n.call(n.gs_engage, True)
        via = "arm station ps4_remote/set_engaged"
        if r is None:
            r = n.call(n.pl_engage, True)
            via = "planner teleop/engage (no arm station)"
        n.say(f"engage via {via}: {getattr(r, 'success', None)} {getattr(r, 'message', '')}")
        t_engage = n.now()
        pre = n.window_stats(0.0, t_engage)
        n.say(f"before engaging (the tail of the DIRECT-entry transient): tilt <= {pre['tilt_max']:.1f} deg, "
              f"|x - ref| <= {pre['e_pos_max']*1e3:.0f} mm")
        if not n.wait(lambda: n.status.startswith("TELEOP") and n.ts(34) > 0.5, 10):
            n.say(f"FAIL: never live (planner {n.status}, pad fresh {n.ts(33)}, live {n.ts(34)})"); return 1
        n.say(f"TELEOP live: time scale {n.ts(32):.2f}, rates xy {n.ts(27):.2f} m/s z {n.ts(28):.2f} "
              f"yaw {n.ts(29):.0f} deg/s EE {n.ts(30):.3f} m/s roll {n.ts(31):.0f} deg/s")
        n.spin(2.0)

        rows = []
        for name, kw, press, settle, kind in LEGS:
            s0 = n.snap()
            n.joy = pad(**kw)
            n.spin(press)
            n.joy = pad()
            if kind == "home":     # the move runs at 10 deg/s per joint: wait for it
                n.wait(lambda: n.ts(25) < 0.5, settle)
                n.spin(2.0)
            else:
                n.spin(settle)
            s1 = n.snap()
            st = n.window_stats(s0["t"], s1["t"])
            dt, dm, unit, ok = score(kind, s0, s1)
            rows.append((name, dt, dm, unit, ok, st, n.note))
            n.say(f"{'PASS' if ok else 'FAIL'}  {name:28s} target {dt:+8.3f} measured {dm:+8.3f} {unit:4s} "
                  f"| tilt<= {st['tilt_max']:4.1f} deg  |x-ref|<= {st['e_pos_max']*1e3:5.0f} mm  "
                  f"|yaw-ref|<= {st['e_yaw_max']:4.1f} deg  same-motion<= {st['ee_res_max']:.3f} mm"
                  + (f"  wall: {n.note}" if n.note else ""))
            if n.reverted:
                n.say("FAIL: the controller left DIRECT"); break

        # ── dropped pad ──
        p0 = n.odom[:3].copy()
        n.joy = None
        n.spin(1.0)
        lost = n.ts(33) < 0.5 and n.ts(34) < 0.5 and n.status.startswith("TELEOP")
        n.spin(1.0)
        moved = float(np.linalg.norm(n.odom[:3] - p0))
        n.joy = pad()
        back = n.wait(lambda: n.ts(33) > 0.5 and n.ts(34) > 0.5, 5)
        ok_drop = lost and back and moved < 0.05
        n.say(f"{'PASS' if ok_drop else 'FAIL'}  dropped pad: inputs disarmed {lost}, held (moved "
              f"{moved*1e3:.0f} mm in 2 s), live again after re-centring {back}")

        # ── release ──
        r = n.call(n.gs_engage, False) or n.call(n.pl_engage, False)
        n.say(f"release: {getattr(r, 'message', '')}")
        released = n.wait(lambda: n.status.startswith("HOLD"), 20)
        n.say(f"{'PASS' if released else 'FAIL'}  release -> planner {n.status}")
        n.joy = None
        n.spin(3.0)
        st_all = n.window_stats(t_engage, n.now())      # the teleop session only
        legs_ok = all(r[4] for r in rows) and len(rows) == len(LEGS)
        ok_all = legs_ok and ok_drop and released and not n.reverted
        n.say(f"WHOLE TELEOP SESSION: mode {n.mode} (left DIRECT: {n.reverted}), tilt <= {st_all['tilt_max']:.1f} deg, "
              f"|x - ref| <= {st_all['e_pos_max']*1e3:.0f} mm, |yaw - ref| <= {st_all['e_yaw_max']:.1f} deg, "
              f"same-motion residual <= {st_all['ee_res_max']:.3f} mm")
        n.say(f"VERDICT: {'ALL PASS' if ok_all else 'FAILURES'} "
              f"({sum(r[4] for r in rows)}/{len(LEGS)} legs, dropped pad {ok_drop}, release {released})")
        if a.out:
            np.savez(a.out, **{k: np.array(v, dtype=object if k == 'mode' else float)
                               for k, v in n.log.items() if len(v)},
                     legs=np.array([(r[0], r[1], r[2], r[3], r[4]) for r in rows], dtype=object))
            n.say(f"saved {a.out}")
        rc = 0 if ok_all else 2
    finally:
        if n is not None:
            n.joy = None
            n.destroy_node()
        rclpy.shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
