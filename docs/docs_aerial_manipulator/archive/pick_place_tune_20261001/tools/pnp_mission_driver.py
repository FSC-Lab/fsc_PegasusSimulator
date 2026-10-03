#!/usr/bin/env python3
"""Fly the full PICK-AND-PLACE mission in 07's Isaac scene and GRASP for real.

    FASTRTPS_DEFAULT_PROFILES_FILE=<udp-only xml> /usr/bin/python3 pnp_mission_driver.py --out run.npz

    offboard -> arm -> SAFETY climb to hover_z -> settle -> DIRECT -> settle
      -> Adjust, Get obj_0, Get drop_0, Plan          (the Pick & Place tab's setup row)
      -> go_to_start -> execute_pick -> arrival gate -> CLOSE the gripper
      -> go_to_place_start (the retreat lifts the payload off the pillar)
      -> execute_place (sweep, approach, descent) -> arrival gate -> OPEN
      -> go_to_land_start -> execute_land -> SAFETY -> land by reference -> disarm

Unlike fsc_trajectory_planner/test/pick_place_sim_driver.py this uses the
scene's own mocap bodies (07 publishes the pillar tops as obj_0 / drop_0) and
drives the gripper, so the payload is really picked, carried and placed. It
records the ground truth of the payload (payload_0/state/pose), the measured
claw (current_ee, FK of the measured base + joints), the streamed reference
the law consumes, the law's debug array, the gripper and the joints -- all
tagged by leg -- for pnp_metrics.py. Refuses to start against an armed vehicle.
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
PP = "whole_body_planner/pick_place"
LEGS = ["go_to_start", "execute_pick", "go_to_place_start", "execute_place",
        "go_to_land_start", "execute_land"]
GRIP_CLOSE, GRIP_OPEN = -0.8727, 0.0           # rad (06's gripper bus, +-50 deg)


class Abort(Exception):
    pass


class Driver(Node):
    def __init__(self, a):
        super().__init__("pnp_mission_driver")
        self.a = a
        ns = a.namespace.rstrip("/")
        self.t0 = time.time()
        self.events, self.marks = [], []
        self.odom = None
        self.mode = ""
        self.armed = None
        self.status = ""
        self.pp_status = ""
        self.info = None
        self.arrival = None
        self.grip = None
        self.payload = None
        self.leg = -1
        self.phase = "init"
        self.ref = np.array([0.0, 0.0, a.hover_z])
        self.stream_ref = True
        self.grip_cmd = None
        self.L = {k: [] for k in ("odom", "dbg", "payload", "ee", "ref", "grip", "joints", "arrival", "claw")}

        sub = self.create_subscription
        sub(Odometry, f"{ns}/state_estimator/local_position/odom", self.on_odom, 10)
        sub(String, f"{ns}/{DA}/mode", lambda m: setattr(self, "mode", m.data), LATCHED)
        sub(Float32MultiArray, f"{ns}/{DA}/wb_control_debug", self.on_dbg, 50)
        sub(VehicleStatus, f"{ns}/fmu/out/vehicle_status_v1",
            lambda m: setattr(self, "armed", m.arming_state == 2), PX4_QOS)
        sub(String, f"{ns}/whole_body_planner/status", self.on_status, LATCHED)
        sub(String, f"{ns}/{PP}/status", self.on_pp_status, LATCHED)
        sub(Float64MultiArray, f"{ns}/{PP}/info", lambda m: setattr(self, "info", np.asarray(m.data)), LATCHED)
        sub(Float64MultiArray, f"{ns}/{PP}/arrival_error", self.on_arrival, 10)
        sub(PoseStamped, f"{ns}/whole_body_planner/current_ee", self.on_ee, 10)
        sub(WholeBodyReference, f"{ns}/{DA}/reference", self.on_ref, 50)
        sub(PoseStamped, "/payload_0/state/pose", self.on_payload, qos_profile_sensor_data)
        sub(JointState, f"{ns}/isaacsim_manipulator/gripper_state", self.on_grip, qos_profile_sensor_data)
        sub(PoseStamped, "/claw_0/state/pose", self.on_claw, qos_profile_sensor_data)
        sub(JointState, f"{ns}/fsc_open_manipulator/joint_states", self.on_joints, 10)
        self.ref_pub = self.create_publisher(PositionControllerReference,
                                             f"{ns}/fsc_autopilot_ros2/position_controller/reference", 10)
        self.grip_pub = self.create_publisher(Float64, f"{ns}/isaacsim_manipulator/gripper_command", 10)
        self.cli = {n: self.create_client(Trigger, f"{ns}/rc/{n}") for n in ("offboard", "arm", "disarm")}
        for n in LEGS + ["adjust", "plan", "capture_pick", "capture_place", "reset"]:
            self.cli[n] = self.create_client(Trigger, f"{ns}/{PP}/{n}")
        self.direct = self.create_client(SetBool, f"{ns}/{DA}/set_direct_mode")
        self.create_timer(1.0 / a.rate, self.tick)

    # ---- callbacks ---------------------------------------------------------
    def now(self):
        return time.time() - self.t0

    def on_odom(self, m):
        p, v = m.pose.pose.position, m.twist.twist.linear
        q = m.pose.pose.orientation
        self.odom = np.array([p.x, p.y, p.z, v.x, v.y, v.z])
        self.L["odom"].append([self.now(), p.x, p.y, p.z, v.x, v.y, v.z, q.w, q.x, q.y, q.z,
                               1.0 if self.mode == "DIRECT" else 0.0, self.leg])

    def on_dbg(self, m):
        if self.mode == "DIRECT":
            self.L["dbg"].append(np.concatenate([[self.now(), self.leg], np.asarray(m.data, np.float32)]))

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
        self.L["arrival"].append([self.now(), *self.arrival[:9]])

    def on_ee(self, m):
        p, q = m.pose.position, m.pose.orientation
        self.L["ee"].append([self.now(), p.x, p.y, p.z, q.w, q.x, q.y, q.z, self.leg])

    def on_ref(self, m):
        r = m.r_ed if hasattr(m, "r_ed") else None
        if r is None:
            return
        self.L["ref"].append([self.now(), r.x, r.y, r.z, m.x_cd.x, m.x_cd.y, m.x_cd.z,
                              *list(m.q_d)[:4], self.leg])

    def on_payload(self, m):
        p, q = m.pose.position, m.pose.orientation
        self.payload = np.array([p.x, p.y, p.z, q.w, q.x, q.y, q.z])
        self.L["payload"].append([self.now(), *self.payload, self.leg])

    def on_claw(self, m):
        p = m.pose.position
        self.L["claw"].append([self.now(), p.x, p.y, p.z, self.leg])

    def on_grip(self, m):
        if m.position:
            self.grip = float(m.position[0])
            eff = float(m.effort[0]) if m.effort else float("nan")
            self.L["grip"].append([self.now(), self.grip, eff, self.leg])

    def on_joints(self, m):
        if len(m.position) >= 4:
            self.L["joints"].append([self.now(), *m.position[:4], *(list(m.effort[:4]) or [np.nan] * 4),
                                     self.leg])
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

    def wait(self, cond, timeout, what):
        t = time.time()
        while time.time() - t < timeout:
            if cond():
                return time.time() - t
            if self.armed and self.mode != "DIRECT" and self.phase == "mission":
                raise Abort(f"left DIRECT while {what}")
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

    def fly_leg(self, k):
        self.leg = k
        name = LEGS[k]
        t_call = self.now()
        self.mark(f"{name}:start")
        self.trig(name)
        self.wait(lambda: self.status.startswith("EXECUTING"), 20, f"{name} EXECUTING")
        T = float(self.status.split("T=")[1].split("s")[0]) if "T=" in self.status else float("nan")
        self.ev(f"{name}: executing, T = {T:.1f} s")
        self.wait(lambda: self.status == "HOLD" and self.info is not None and int(self.info[5]) == k,
                  (T if T == T else 60.0) + 40, f"{name} complete")
        self.mark(f"{name}:end")
        self.ev(f"{name}: complete after {self.now() - t_call:.1f} s")

    def gate(self, k):
        """The 50 mm arrival gate must stay open gate_hold s (what the operator waits for)."""
        t, since = time.time(), None
        while time.time() - t < self.a.gate_timeout:
            ok = self.arrival is not None and int(self.arrival[0]) == k and self.arrival[8] > 0.5
            since = (since or time.time()) if ok else None
            if since and time.time() - since >= self.a.gate_hold:
                self.ev(f"{LEGS[k]}: claw inside the gate {self.a.gate_hold:.0f} s, error "
                        f"{self.arrival[2] * 1e3:.1f} mm")
                return True
            time.sleep(0.05)
        self.ev(f"{LEGS[k]}: claw NOT inside the gate within {self.a.gate_timeout:.0f} s")
        return False

    def gripper(self, target, what, dwell=None):
        self.mark(f"gripper_{what}:start")
        self.grip_cmd = target
        g0 = self.grip if self.grip is not None else 0.0
        t, last, still = time.time(), None, None
        # the jaws must first LEAVE their start angle (the bus slews at 0.5 rad/s;
        # an unchanged reading before the motion starts is not "settled")
        while time.time() - t < 3.0 and (self.grip is None or abs(self.grip - g0) < np.radians(2.0)):
            time.sleep(0.02)
        while time.time() - t < 8.0:
            g = self.grip
            if g is not None and last is not None and abs(g - last) < 0.002:
                still = still or time.time()
                if time.time() - still > self.a.stall_s:
                    break
            else:
                still = None
            last = g
            time.sleep(0.05)
        self.ev(f"gripper {what}: settled at {np.degrees(self.grip or np.nan):.1f} deg "
                f"(command {np.degrees(target):.0f})")
        time.sleep(self.a.grip_dwell if dwell is None else dwell)
        self.mark(f"gripper_{what}:end")

    # ---- the mission -------------------------------------------------------
    def mission(self):
        a = self.a
        self.wait(lambda: self.odom is not None and self.mode and self.armed is not None
                  and self.payload is not None, 60, "vehicle + payload feeds")
        if self.armed:
            raise Abort("vehicle already ARMED -- refusing (clean relaunch needed)")
        self.payload0 = self.payload.copy()
        self.ev(f"payload at rest {np.round(self.payload0[:3], 4).tolist()}")
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
        self.trig("reset", must=False)
        self.trig("adjust")
        time.sleep(1.0)
        self.trig("capture_pick")
        self.trig("capture_place")
        self.trig("plan")
        self.wait(lambda: self.pp_status.startswith("READY"), 40, "mission READY")
        self.ev(f"planned leg durations {np.round(self.info[8:14], 1).tolist()} s")
        self.fly_leg(0)
        time.sleep(a.leg_pause)
        self.fly_leg(1)
        self.gate(1)
        time.sleep(a.pre_grip)
        self.payload_at_close = self.payload.copy()
        moved = np.linalg.norm(self.payload[:3] - self.payload0[:3])
        if moved > a.max_disturb:
            # closing on a pushed payload knocks it off the pillar (run 1)
            raise Abort(f"payload disturbed {moved * 1e3:.1f} mm by the approach -- not closing")
        # lift as soon as the jaws stall: clamped on the pillar the vehicle is
        # in a closed chain with the table, which drifts within ~1 s
        self.gripper(GRIP_CLOSE, "close", dwell=a.lift_after_clamp)
        self.fly_leg(2)
        time.sleep(a.leg_pause)
        self.fly_leg(3)
        self.gate(3)
        # release as soon as the gate opens, lift right after: hovering over the
        # pillar the base wanders, and the open jaws straddle the handle
        self.gripper(GRIP_OPEN, "open", dwell=a.lift_after_clamp)
        self.fly_leg(4)
        time.sleep(a.leg_pause)
        self.fly_leg(5)
        self.wait(lambda: self.pp_status.startswith("COMPLETE"), 10, "COMPLETE")
        time.sleep(a.post_hold)
        self.phase = "done"

    def land(self):
        self.phase = "land"
        self.leg = -1
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
            arr[k] = np.array(v, dtype=np.float32 if k == "dbg" else np.float64) if v else np.zeros((0, 1))
        np.savez_compressed(path, **arr, events=np.array(self.events),
                            marks_t=np.array([m[0] for m in self.marks]),
                            marks_name=np.array([m[1] for m in self.marks]),
                            info=self.info if self.info is not None else np.zeros(0),
                            payload0=getattr(self, "payload0", np.zeros(7)),
                            joint_names=np.array(getattr(self, "joint_names", [])),
                            aborted=aborted, reason=reason)
        print(f"saved {path}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--namespace", default="/uav_0")
    ap.add_argument("--rate", type=float, default=50.0)
    ap.add_argument("--hover-z", type=float, default=1.0)
    ap.add_argument("--land-z", type=float, default=0.30)
    ap.add_argument("--gate-timeout", type=float, default=30.0)
    ap.add_argument("--gate-hold", type=float, default=2.0)
    ap.add_argument("--pre-grip", type=float, default=1.0, help="extra settle after the gate, before the gripper")
    ap.add_argument("--grip-dwell", type=float, default=1.5, help="hold after the gripper stops moving (release)")
    ap.add_argument("--stall-s", type=float, default=0.25,
                    help="the jaws count as stalled after this long without moving [s]")
    ap.add_argument("--lift-after-clamp", type=float, default=0.0,
                    help="wait between the jaws stalling on the handle and requesting the lift [s]")
    ap.add_argument("--max-disturb", type=float, default=0.005,
                    help="refuse to close if the approach moved the payload more than this [m]")
    ap.add_argument("--direct-settle", type=float, default=15.0)
    ap.add_argument("--leg-pause", type=float, default=3.0)
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
