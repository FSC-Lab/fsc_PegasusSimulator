#!/usr/bin/env python3
"""Drive the bare-T650 GEOMETRIC+L1 DIRECT stack through the 769 g loaded campaign.

Mission mirrors Command.md 7.13.2 so the loaded numbers are directly comparable to the
2026-08-21 bare sweep: SAFETY takeoff to z=1.0 -> settle -> DIRECT -> soak -> 0.5 m X
step/return -> 0.25 m Z step/return -> SAFETY -> reference landing -> disarm.

Records odom, the reference actually streamed, the 37-element l1_control_debug array and
the motor commands to an npz. Reverts to SAFETY and lands on its own if the vehicle
exceeds the abort envelope.
"""
import argparse, sys, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float32MultiArray
from std_srvs.srv import SetBool, Trigger
from fsc_autopilot_ros2_msgs.msg import PositionControllerReference
from px4_msgs.msg import VehicleStatus

PX4_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE,
                     history=HistoryPolicy.KEEP_LAST, depth=10)

LATCH = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                   durability=DurabilityPolicy.TRANSIENT_LOCAL,
                   history=HistoryPolicy.KEEP_LAST, depth=1)


def quat_to_tilt_deg(qw, qx, qy, qz):
    # angle between body z and world z
    r33 = 1.0 - 2.0 * (qx * qx + qy * qy)
    return float(np.degrees(np.arccos(np.clip(r33, -1.0, 1.0))))


class Driver(Node):
    def __init__(self, a):
        super().__init__("l1_769g_campaign_driver")
        ns = a.namespace
        self.a = a
        da = f"{ns}/fsc_autopilot_ros2/geometric_l1_direct_actuation"
        self.pub_ref = self.create_publisher(
            PositionControllerReference, f"{ns}/fsc_autopilot_ros2/position_controller/reference", 10)
        self.create_subscription(Odometry, f"{ns}/state_estimator/local_position/odom", self.on_odom, 10)
        self.create_subscription(Float32MultiArray, f"{da}/l1_control_debug", self.on_dbg, 10)
        self.create_subscription(String, f"{da}/mode", self.on_mode, LATCH)
        self.create_subscription(VehicleStatus, f"{ns}/fmu/out/vehicle_status_v1",
                                 self.on_status, PX4_QOS)
        self.armed_at_start = None
        self.cli_off = self.create_client(Trigger, f"{ns}/rc/offboard")
        self.cli_arm = self.create_client(Trigger, f"{ns}/rc/arm")
        self.cli_dis = self.create_client(Trigger, f"{ns}/rc/disarm")
        self.cli_dir = self.create_client(SetBool, f"{da}/set_direct_mode")

        self.odom = None
        self.mode = ""
        self.dbg = None
        self.log = []          # [t, x,y,z, vx,vy,vz, tilt, rx,ry,rz]
        self.dbg_log = []      # [t, *37]
        self.events = []
        self.phase = "WAIT"
        self.t0 = time.time()
        self.tp = self.t0
        self.ref = np.zeros(3)
        self.home = None
        self.settle_since = None
        self.aborted = False
        self.done = False
        self.futs = []
        self.armed = None
        self.dis_tries = 0
        self.create_timer(1.0 / 50.0, self.tick)

    # ---- callbacks ----
    def on_odom(self, m):
        p, o = m.pose.pose.position, m.pose.pose.orientation
        v = m.twist.twist.linear
        self.odom = (p.x, p.y, p.z, v.x, v.y, v.z,
                     quat_to_tilt_deg(o.w, o.x, o.y, o.z))

    def on_status(self, m):
        self.armed = (m.arming_state == 2)

    def on_mode(self, m):
        if m.data != self.mode:
            self.ev(f"mode -> {m.data}")
        self.mode = m.data

    def on_dbg(self, m):
        self.dbg = np.asarray(m.data, dtype=np.float64)
        if self.phase not in ("WAIT", "OFFBOARD", "ARM"):
            self.dbg_log.append(np.concatenate(([time.time() - self.t0], self.dbg)))

    # ---- helpers ----
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
        self.ev(f"PHASE {ph}  ref={np.round(self.ref,3).tolist()} mode='{self.mode}'")

    def call(self, cli, req, name, timeout=10.0):
        """Fire-and-forget. NEVER block here: tick() runs inside the executor's
        callback, so spinning for the future from in here deadlocks the future it
        is waiting on (the request still goes out, which is why run 1 flew while
        every call reported a timeout). Results are reported from _drain()."""
        if not cli.service_is_ready():
            self.ev(f"SERVICE NOT READY (sent anyway): {name}")
        fut = cli.call_async(req)
        self.futs.append((name, fut, time.time()))
        self.ev(f"service {name} requested")
        return None

    def _drain(self):
        keep = []
        for name, fut, t in self.futs:
            if fut.done():
                try:
                    self.ev(f"service {name} -> {fut.result()}")
                except Exception as e:                      # noqa: BLE001
                    self.ev(f"service {name} FAILED: {e}")
            elif time.time() - t > 10.0:
                self.ev(f"service {name} NO RESPONSE after 10 s")
            else:
                keep.append((name, fut, t))
        self.futs = keep

    def send_ref(self):
        m = PositionControllerReference()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "map"
        m.position.x, m.position.y, m.position.z = map(float, self.ref)
        m.yaw = 0.0
        m.yaw_unit = PositionControllerReference.DEGREES
        self.pub_ref.publish(m)

    def is_direct(self):
        m = self.mode.lower()
        return "direct" in m and "safety" not in m

    # ---- main loop ----
    def tick(self):
        if self.done:
            return
        self._drain()
        if self.odom is not None and self.phase not in ("WAIT", "OFFBOARD", "ARM"):
            self.log.append(np.concatenate(([self.now()], self.odom, self.ref)))

        if self.phase not in ("WAIT", "OFFBOARD"):
            self.send_ref()

        # ---- abort envelope ----
        if self.odom and self.phase in ("SOAK", "STEPX", "STEPX_RET", "STEPZ",
                                        "STEPZ_RET", "SOAK2"):
            x, y, z, vx, vy, vz, tilt = self.odom
            bad = (tilt > 35.0 or z > 3.5 or z < 0.25 or abs(x) > 3.0 or abs(y) > 3.0
                   or np.hypot(vx, vy) > 3.0)
            if bad and not self.aborted:
                self.aborted = True
                self.ev(f"!! ABORT ENVELOPE: pos=({x:.2f},{y:.2f},{z:.2f}) tilt={tilt:.1f} deg")
                self.call(self.cli_dir, SetBool.Request(data=False), "set_direct_mode(false)")
                self.ref = np.array([self.home[0], self.home[1], 1.0])
                self.goto("LAND_WAIT")
                return

        p = self.phase
        if p == "WAIT":
            if self.odom is not None and self.mode and self.armed is not None:
                if self.armed:
                    # A vehicle already armed means the previous run left PX4 latched
                    # "in flight" and the SAFETY UDE integrating the ground reaction.
                    # Both survive a re-arm attempt and silently suppress the takeoff
                    # thrust (measured 2026-08-24: 60 s at ref z=1.0 with zero lift).
                    # Relaunch the sim instead of flying a dirty plant.
                    self.ev("!! PX4 IS ALREADY ARMED -- refusing to fly. "
                            "Do the full step-0 clean and relaunch.")
                    self.done = True
                    return
                self.home = np.array(self.odom[:3])
                self.ev(f"stack up. home={np.round(self.home,3).tolist()} mode='{self.mode}'")
                self.ref = np.array([self.home[0], self.home[1], self.home[2]])
                self.goto("OFFBOARD")
        elif p == "OFFBOARD":
            if self.in_phase() > 1.0:
                self.ref = np.array([self.home[0], self.home[1], self.home[2]])
                self.send_ref()
                self.call(self.cli_off, Trigger.Request(), "rc/offboard")
                self.goto("ARM")
        elif p == "ARM":
            if self.in_phase() > 3.0:
                self.call(self.cli_arm, Trigger.Request(), "rc/arm")
                self.ref = np.array([self.home[0], self.home[1], self.a.hover_z])
                self.goto("TAKEOFF")
        elif p == "TAKEOFF":
            x, y, z, vx, vy, vz, _ = self.odom
            err = np.linalg.norm(np.array([x, y, z]) - self.ref)
            spd = np.linalg.norm([vx, vy, vz])
            ok = err < 0.12 and spd < 0.10
            if ok:
                self.settle_since = self.settle_since or time.time()
                if time.time() - self.settle_since > 4.0:
                    self.ev(f"settled: err={err*1000:.0f} mm speed={spd:.3f} m/s")
                    self.goto("ENTER_DIRECT")
            else:
                self.settle_since = None
            if self.in_phase() > 60.0:
                self.ev("!! takeoff never settled -> landing")
                self.goto("LAND_WAIT")
        elif p == "ENTER_DIRECT":
            if self.in_phase() > 0.5:
                self.call(self.cli_dir, SetBool.Request(data=True), "set_direct_mode(true)")
                self.goto("CONFIRM")
        elif p == "CONFIRM":
            if self.is_direct():
                self.ev(f"DIRECT confirmed via mode topic: '{self.mode}'")
                self.goto("SOAK")
            elif self.in_phase() > 6.0:
                self.ev("!! DIRECT not confirmed -> landing")
                self.goto("LAND_WAIT")
        elif p == "SOAK":
            if self.in_phase() > self.a.soak:
                self.ref = self.ref + np.array([0.5, 0.0, 0.0])
                self.goto("STEPX")
        elif p == "STEPX":
            if self.in_phase() > self.a.step:
                self.ref = self.ref - np.array([0.5, 0.0, 0.0])
                self.goto("STEPX_RET")
        elif p == "STEPX_RET":
            if self.in_phase() > self.a.step:
                self.ref = self.ref + np.array([0.0, 0.0, 0.25])
                self.goto("STEPZ")
        elif p == "STEPZ":
            if self.in_phase() > self.a.step:
                self.ref = self.ref - np.array([0.0, 0.0, 0.25])
                self.goto("STEPZ_RET")
        elif p == "STEPZ_RET":
            if self.in_phase() > self.a.step:
                self.goto("SOAK2")
        elif p == "SOAK2":
            if self.in_phase() > 10.0:
                self.call(self.cli_dir, SetBool.Request(data=False), "set_direct_mode(false)")
                self.goto("SAFETY_SETTLE")
        elif p == "SAFETY_SETTLE":
            if self.in_phase() > 12.0:
                self.goto("LAND_WAIT")
        elif p == "LAND_WAIT":
            if self.in_phase() > 1.0:
                self.goto("LAND")
        elif p == "LAND":
            self.ref[2] = max(self.a.land_z, self.ref[2] - 0.20 / 50.0)
            if self.ref[2] <= self.a.land_z + 1e-6 and self.in_phase() > 12.0:
                self.goto("DISARM")
        elif p == "DISARM":
            # PX4 denies disarm until its land detector sees low thrust, so retry
            # rather than assume. Leaving the vehicle armed poisons the NEXT run:
            # PX4 stays latched "in flight" and the SAFETY UDE integrates the ground
            # reaction, which silently suppresses the takeoff thrust.
            if self.armed is False:
                self.ev("disarmed -- clean end state")
                self.goto("END")
            elif self.in_phase() > 3.0 * (self.dis_tries + 1) and self.dis_tries < 6:
                self.dis_tries += 1
                self.call(self.cli_dis, Trigger.Request(),
                          f"rc/disarm (try {self.dis_tries})")
            elif self.dis_tries >= 6:
                self.ev("!! STILL ARMED after 6 disarm attempts -- relaunch the sim "
                        "before the next run")
                self.goto("END")
        elif p == "END":
            if self.in_phase() > 2.0:
                self.done = True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--namespace", default="/uav_0")
    ap.add_argument("--hover-z", type=float, default=1.0)
    ap.add_argument("--land-z", type=float, default=0.12)
    ap.add_argument("--soak", type=float, default=45.0)
    ap.add_argument("--step", type=float, default=15.0)
    ap.add_argument("--out", default="l1_769g_run.npz")
    a = ap.parse_args()
    rclpy.init()
    d = Driver(a)
    try:
        while rclpy.ok() and not d.done:
            rclpy.spin_once(d, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        log = np.array(d.log) if d.log else np.zeros((0, 11))
        dbg = np.array(d.dbg_log) if d.dbg_log else np.zeros((0, 38))
        np.savez(a.out, log=log, dbg=dbg, events=np.array(d.events, dtype=object),
                 cols=np.array(["t", "x", "y", "z", "vx", "vy", "vz", "tilt",
                                "rx", "ry", "rz"]))
        print(f"saved {a.out}: log {log.shape}, dbg {dbg.shape}", flush=True)
        d.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
