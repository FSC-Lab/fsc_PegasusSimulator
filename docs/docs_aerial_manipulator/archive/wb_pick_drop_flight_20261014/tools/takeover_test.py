#!/usr/bin/env python3
"""Pilot-takeover test in Isaac (2026-10-14): the hardware landings' failure, played by a scripted pilot.

Run after `ps4_teleop_bringup.py up` (vehicle hovering in DIRECT). The 'pilot' sends sticks to PX4 over DDS
(/fmu/in/manual_control_input, data_source MAVLINK_0) and flips modes with VehicleCommand -- what the RC does:

  direct      6 s   the controller flies (whole-body or geometric+L1 DIRECT)
  stab_hold   8 s   PX4 -> STABILIZED; the pilot holds the height with the throttle stick (PD + I on z)
  stab_land         the pilot descends at 0.3 m/s and puts it down, throttle to minimum
  disarmed    3 s   disarm (the RC switch; forced, PX4 SITL never detects 'landed' on this rig)
  rearm             arm again in STAB with the throttle at minimum
  stab_climb        climb to 0.8 m with the stick, hold 4 s
  handback    8 s   PX4 -> OFFBOARD: the controller must be in SAFETY and hold where the vehicle IS

Everything this node sees on /fmu/in/actuator_motors is logged (time, NaN or finite): after the takeover the
controller must publish NOTHING there. Writes --out (npz) and prints a summary.
Run with /usr/bin/python3 and the UDP-only Fast DDS profile (shiqi-desktop SHM trap).
"""
import argparse, sys, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from px4_msgs.msg import VehicleStatus, VehicleCommand, VehicleCommandAck, ManualControlSetpoint, ActuatorMotors

BE = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST, depth=10)
REL = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE,
                 history=HistoryPolicy.KEEP_LAST, depth=10)
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
NAV = {14: "OFFBOARD", 15: "STAB", 4: "LOITER", 1: "ALTCTL", 2: "POSCTL", 18: "LAND", 12: "DESCEND"}


class Pilot(Node):
    def __init__(self, a):
        super().__init__("takeover_test_pilot")
        ns = a.namespace.rstrip("/")
        self.a = a
        self.odom = None; self.nav = None; self.armed = None; self.mode = ""
        self.motor_t, self.motor_nan = [], []
        self.acks = []
        self.create_subscription(Odometry, f"{ns}/state_estimator/local_position/odom", self.on_odom, BE)
        self.create_subscription(VehicleStatus, f"{ns}/fmu/out/vehicle_status_v1", self.on_status, BE)
        self.create_subscription(VehicleCommandAck, f"{ns}/fmu/out/vehicle_command_ack", self.on_ack, BE)
        self.create_subscription(String, f"{ns}/fsc_autopilot_ros2/{a.da}/mode", self.on_mode, LATCHED)
        self.create_subscription(ActuatorMotors, f"{ns}/fmu/in/actuator_motors", self.on_motors, BE)
        self.cmd_pub = self.create_publisher(VehicleCommand, f"{ns}/fmu/in/vehicle_command", REL)
        self.man_pub = self.create_publisher(ManualControlSetpoint, f"{ns}/fmu/in/manual_control_input", BE)
        self.t0 = time.time()
        self.log_rows = []
        self.phase = "direct"; self.stick = -1.0; self.integ = 0.0; self.z_ref = None
        self.roll = self.pitch = 0.0; self.xy_ref = None; self.yaw = 0.0
        self._last_us = 0

    # ---- callbacks
    def on_odom(self, m):
        p, v, q = m.pose.pose.position, m.twist.twist.linear, m.pose.pose.orientation
        self.odom = np.array([p.x, p.y, p.z, v.x, v.y, v.z])
        self.yaw = float(np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

    def on_status(self, m):
        self.nav = int(m.nav_state); self.armed = int(m.arming_state) == 2

    def on_ack(self, m):
        self.acks.append((time.time() - self.t0, int(m.command), int(m.result)))

    def on_mode(self, m):
        self.mode = m.data

    def on_motors(self, m):
        self.motor_t.append(time.time() - self.t0)
        self.motor_nan.append(bool(np.isnan(np.asarray(m.control[:4], float)).all()))

    # ---- PX4 inputs
    def now_us(self):
        t = self.get_clock().now().nanoseconds // 1000
        t = max(t, self._last_us + 1); self._last_us = t
        return t

    def command(self, cmd, p1=0.0, p2=0.0, p3=0.0):
        m = VehicleCommand()
        m.timestamp = self.now_us(); m.command = int(cmd)
        m.param1, m.param2, m.param3 = float(p1), float(p2), float(p3)
        m.target_system = m.target_component = m.source_system = m.source_component = 1
        m.from_external = True
        for _ in range(3):
            self.cmd_pub.publish(m)

    def send_sticks(self):
        m = ManualControlSetpoint()
        us = self.now_us()
        m.timestamp = us; m.timestamp_sample = us
        m.valid = True; m.data_source = ManualControlSetpoint.SOURCE_MAVLINK_0
        m.roll, m.pitch, m.yaw = float(self.roll), float(self.pitch), 0.0
        m.throttle = float(np.clip(self.stick, -1.0, 1.0))
        m.flaps = 0.0
        self.man_pub.publish(m)

    # ---- the pilot's height loop (stick in [-1, 1], 0 = PX4's MPC_THR_HOVER)
    def fly_height(self, dt):
        # a pilot also keeps it over the spot: STAB has no position hold (scripted drift reached 11 m in run 1,
        # and landing while drifting tips it over on the skids). Soft PD on position -> roll/pitch sticks.
        if self.xy_ref is not None:
            a = 0.6 * (self.xy_ref - self.odom[:2]) - 1.2 * self.odom[3:5]
            n = np.linalg.norm(a)
            a = a * (1.5 / n) if n > 1.5 else a
            c, sn = np.cos(self.yaw), np.sin(self.yaw)
            fwd, left = c * a[0] + sn * a[1], -sn * a[0] + c * a[1]
            tilt = 9.81 * np.radians(35.0)          # PX4 MPC_MAN_TILT_MAX default: stick 1 = 35 deg
            self.pitch = float(np.clip(fwd / tilt, -0.4, 0.4))
            self.roll = float(np.clip(-left / tilt, -0.4, 0.4))
        e = self.z_ref - self.odom[2]
        self.integ = float(np.clip(self.integ + 0.15 * e * dt, -0.5, 0.5))
        self.stick = float(np.clip(0.45 * e - 0.30 * self.odom[5] + self.integ, -1.0, 1.0))

    def tick(self, dt=0.02):
        # paced at 50 Hz in WALL time: spin_once returns on every message, so counting ticks is not a clock
        t_next = getattr(self, "_t_next", time.time()) + dt
        while True:
            rem = t_next - time.time()
            if rem <= 0:
                break
            rclpy.spin_once(self, timeout_sec=rem)
        self._t_next = max(t_next, time.time() - dt)
        self.send_sticks()
        if self.odom is not None:
            self.log_rows.append((time.time() - self.t0, *self.odom[:3], self.odom[5], self.nav if self.nav is not None else -1,
                                  1 if self.armed else 0, 1 if self.mode == "DIRECT" else 0, self.stick, PHASES.index(self.phase)))

    def run_for(self, seconds, height=False):
        t_end = time.time() + seconds
        while time.time() < t_end:
            if height:
                self.fly_height(0.02)
            self.tick()

    def wait(self, cond, timeout, height=False):
        t_end = time.time() + timeout
        while time.time() < t_end:
            if cond():
                return True
            if height:
                self.fly_height(0.02)
            self.tick()
        return False

    def say(self, s):
        print(f"[{time.time() - self.t0:6.1f}] {s}", flush=True)


TRIM = 0.20   # throttle stick that hovers this vehicle in PX4 STAB (measured: +0.22; PX4's centre stick = MPC_THR_HOVER is low for the AM)
PHASES = ["direct", "stab_hold", "stab_land", "disarmed", "rearm", "stab_climb", "handback"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--namespace", default="/uav_0")
    ap.add_argument("--da", default="whole_body_direct_actuation")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    rclpy.init()
    p = Pilot(a)
    if not p.wait(lambda: p.odom is not None and p.nav is not None and p.mode, 30):
        p.say("no odometry / status / mode"); return 1
    p.say(f"start: PX4 {NAV.get(p.nav, p.nav)}, armed {p.armed}, controller {p.mode}, z {p.odom[2]:.2f}")
    if p.mode != "DIRECT" or p.nav != 14:
        p.say("not in DIRECT + OFFBOARD -- run ps4_teleop_bringup.py up first"); return 1
    ev = {}
    # sticks present BEFORE any mode switch, throttle at minimum while the controller still flies (PX4 ignores it)
    p.stick = -1.0
    p.phase = "direct"; p.run_for(6.0)
    # ---- the takeover
    p.z_ref = p.odom[2]; p.integ = TRIM; p.stick = TRIM      # the pilot's throttle at the hover position
    p.xy_ref = p.odom[:2].copy()
    p.phase = "stab_hold"; ev["t_switch"] = time.time() - p.t0
    p.say(f"PILOT TAKES OVER: STABILIZED at z {p.odom[2]:.2f}")
    p.command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1, 7)
    ok = p.wait(lambda: p.nav == 15, 3.0, height=True)
    t_nav = time.time()
    p.wait(lambda: p.mode != "DIRECT", 0.5, height=True)
    ev["mode_after_switch"] = p.mode; ev["mode_delay_s"] = time.time() - t_nav
    p.say(f"PX4 {NAV.get(p.nav, p.nav)} ({'switched' if ok else 'DID NOT SWITCH'}), controller {p.mode} "
          f"{ev['mode_delay_s'] * 1e3:.0f} ms after PX4's switch")
    p.run_for(8.0, height=True)
    # ---- the pilot lands
    p.phase = "stab_land"; p.say("pilot descends")
    t_end = time.time() + 15.0; ground = None
    while time.time() < t_end:
        p.z_ref = max(0.10, p.z_ref - 0.3 * 0.02)
        p.fly_height(0.02); p.tick()
        if p.odom[2] < 0.30 and abs(p.odom[5]) < 0.05:
            ground = ground or time.time()
            if time.time() - ground > 1.0:
                break
        else:
            ground = None
    p.stick = -1.0; p.roll = p.pitch = 0.0; xy_keep = p.xy_ref; p.xy_ref = None; p.run_for(1.0)
    p.say(f"on the ground at z {p.odom[2]:.2f}, {np.linalg.norm(p.odom[:2] - xy_keep) * 1e3:.0f} mm from the spot")
    p.phase = "disarmed"
    p.command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0, 21196)
    p.wait(lambda: p.armed is False, 3.0); p.say(f"disarmed: {not p.armed}"); p.run_for(3.0)
    # ---- re-arm in STAB and climb
    p.phase = "rearm"
    p.command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1, 0)
    armed = p.wait(lambda: p.armed is True, 4.0)
    if not armed:
        p.say("plain arm denied -- forcing"); p.command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1, 21196)
        armed = p.wait(lambda: p.armed is True, 4.0)
    ev["rearmed"] = bool(armed); p.say(f"re-armed: {armed}, PX4 {NAV.get(p.nav, p.nav)}, controller {p.mode}")
    p.run_for(2.0)
    p.phase = "stab_climb"; p.z_ref = p.odom[2]; p.integ = TRIM; p.xy_ref = p.odom[:2].copy()
    t_end = time.time() + 12.0
    while time.time() < t_end:
        p.z_ref = min(0.80, p.z_ref + 0.3 * 0.02)
        p.fly_height(0.02); p.tick()
    ev["z_climb_max"] = float(max(r[3] for r in p.log_rows if r[9] == PHASES.index("stab_climb")))
    p.say(f"climb: max z {ev['z_climb_max']:.2f} m")
    # ---- hand back to OFFBOARD
    p.phase = "handback"; ev["t_handback"] = time.time() - p.t0
    pos0 = p.odom[:3].copy()
    p.command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1, 6)
    ok = p.wait(lambda: p.nav == 14, 3.0, height=True)
    p.say(f"hand-back: PX4 {NAV.get(p.nav, p.nav)} ({'ok' if ok else 'DID NOT SWITCH'}), controller {p.mode}")
    p.run_for(8.0)
    ev["handback_dxy"] = float(np.linalg.norm(p.odom[:2] - pos0[:2])); ev["handback_dz"] = float(p.odom[2] - pos0[2])
    ev["mode_at_end"] = p.mode
    # ---- summary
    L = np.array(p.log_rows); mt = np.array(p.motor_t); mn = np.array(p.motor_nan, bool)
    ts = ev["t_switch"]
    after = mt > ts + 0.05
    hold = L[:, 9] == PHASES.index("stab_hold")
    zh = L[hold, 3]
    print("\n=== SUMMARY ===")
    print(f"controller mode after the switch: {ev['mode_after_switch']}, {ev['mode_delay_s'] * 1e3:.0f} ms after PX4's  (must be SAFETY)")
    print(f"/fmu/in/actuator_motors after the switch: {int(after.sum())} msgs, {int((after & mn).sum())} all-NaN  (must be 0)")
    print(f"STAB height hold (8 s, pilot loop): z ref {L[hold, 3][0]:.2f}, mean {zh.mean():.2f}, min {zh.min():.2f}, std {zh.std() * 1e3:.0f} mm")
    print(f"re-armed in STAB: {ev['rearmed']}; climbed to {ev['z_climb_max']:.2f} m (target 0.80)")
    print(f"hand-back to OFFBOARD: moved {ev['handback_dxy'] * 1e3:.0f} mm in xy, {ev['handback_dz'] * 1e3:+.0f} mm in z over 8 s; controller {ev['mode_at_end']}")
    print("PX4 acks (t, command, result):", p.acks[-8:])
    np.savez(a.out, log=L, motor_t=mt, motor_nan=mn, acks=np.array(p.acks), phases=np.array(PHASES), **{k: v for k, v in ev.items()})
    p.destroy_node(); rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
