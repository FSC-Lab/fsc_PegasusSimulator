#!/usr/bin/env python3
"""§7.14.1 DIRECT stability soak: SAFETY takeoff -> DIRECT -> soak -> SAFETY -> land.

Publishes ONLY the PositionControllerReference (SAFETY reference + the WB
node's entry gate / abort target). The PLANNER owns WholeBodyReference --
this driver must never publish it or the test is corrupted.
"""
import argparse, json, math, sys, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, String
from std_srvs.srv import Trigger, SetBool
from fsc_autopilot_ros2_msgs.msg import PositionControllerReference

LATCH = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                   durability=DurabilityPolicy.TRANSIENT_LOCAL,
                   history=HistoryPolicy.KEEP_LAST)


def quat_tilt_deg(q):                      # q = (x,y,z,w)
    x, y, z, w = q
    b3z = 1.0 - 2.0 * (x * x + y * y)
    return math.degrees(math.acos(max(-1.0, min(1.0, b3z))))


class Soak(Node):
    def __init__(self, a):
        super().__init__("wb_direct_soak")
        self.a = a
        ns = a.namespace.rstrip("/")
        da = f"{ns}/fsc_autopilot_ros2/whole_body_direct_actuation"
        self.pub_ref = self.create_publisher(
            PositionControllerReference,
            f"{ns}/fsc_autopilot_ros2/position_controller/reference", 10)
        self.create_subscription(Odometry,
            f"{ns}/state_estimator/local_position/odom", self.on_odom, 10)
        self.create_subscription(Float32MultiArray,
            f"{da}/wb_control_debug", self.on_dbg, 10)
        self.create_subscription(String, f"{da}/mode", self.on_mode, LATCH)
        self.create_subscription(JointState,
            f"{ns}/isaacsim_manipulator/joint_states", self.on_joints, 10)
        self.cli_off = self.create_client(Trigger, f"{ns}/rc/offboard")
        self.cli_arm = self.create_client(Trigger, f"{ns}/rc/arm")
        self.cli_dir = self.create_client(SetBool, f"{da}/set_direct_mode")

        self.odom = None; self.dbg = None; self.mode = ""; self.q = None
        self.phase = "WAIT"; self.t0 = time.time(); self.tp = self.t0
        self.rows = []; self.events = []; self.futs = []
        self.aborted = False; self.abort_reason = ""; self.done = False
        self.ref_p = None; self.ref_yaw = 0.0; self.home = None
        self.direct_t0 = None; self.settle_since = None
        self.create_timer(1.0 / a.rate, self.tick)

    # ---- callbacks
    def on_odom(self, m): self.odom = m
    def on_dbg(self, m): self.dbg = list(m.data)
    def on_mode(self, m): self.mode = m.data
    def on_joints(self, m): self.q = list(m.position)

    def ev(self, s):
        t = time.time() - self.t0
        self.events.append((round(t, 2), s))
        print(f"[{t:7.2f}] {s}", flush=True)

    def goto(self, p):
        self.phase = p; self.tp = time.time(); self.ev(f"-> {p}")

    def call(self, cli, req):
        if not cli.service_is_ready():
            self.ev(f"!! service not ready: {cli.srv_name}"); return
        self.futs.append(cli.call_async(req))

    def pos(self):
        p = self.odom.pose.pose.position
        return np.array([p.x, p.y, p.z])

    def publish_ref(self):
        m = PositionControllerReference()
        m.header.stamp = self.get_clock().now().to_msg()
        m.position.x, m.position.y, m.position.z = map(float, self.ref_p)
        m.yaw = float(self.ref_yaw)
        self.pub_ref.publish(m)

    # ---- state machine
    def tick(self):
        now = time.time(); el = now - self.tp
        if self.odom is None:
            return
        p = self.pos()
        q = self.odom.pose.pose.orientation
        tilt = quat_tilt_deg((q.x, q.y, q.z, q.w))

        if self.ref_p is not None:
            self.publish_ref()

        # record
        if self.dbg is not None and len(self.dbg) >= 57:
            d = self.dbg
            self.rows.append(dict(
                t=round(now - self.t0, 4), phase=self.phase, mode=self.mode,
                x=p[0], y=p[1], z=p[2], tilt=tilt,
                ref=list(map(float, self.ref_p)) if self.ref_p is not None else None,
                dmode=d[0], hold=d[1], fresh=d[56],
                qm=d[5:9], tau=d[13:17], u1=d[17],
                e_y=d[24:28], e_R=d[28:31], motors=d[41:45],
                x_cd=d[45:48], x_c=d[48:51], n_sat=d[51],
                unalloc=d[52:56], qj=self.q))

        # abort envelope (DIRECT only)
        if self.phase == "DIRECT" and not self.aborted:
            err = float(np.linalg.norm(p - self.ref_p))
            if tilt > self.a.abort_tilt or err > self.a.abort_pos or p[2] < 0.25:
                self.aborted = True
                self.abort_reason = f"tilt={tilt:.1f}deg poserr={err:.2f}m z={p[2]:.2f}"
                self.ev(f"!! ABORT: {self.abort_reason}")
                self.call(self.cli_dir, SetBool.Request(data=False))
                self.goto("LAND")
                return

        if self.phase == "WAIT":
            if el > 2.0:
                self.home = p.copy(); self.ref_p = p.copy()
                self.ev(f"home = {np.round(self.home,3).tolist()}")
                self.call(self.cli_off, Trigger.Request()); self.goto("OFFBOARD")
        elif self.phase == "OFFBOARD":
            if el > 2.5:
                self.call(self.cli_arm, Trigger.Request()); self.goto("ARM")
        elif self.phase == "ARM":
            if el > 3.0:
                self.climb_from = self.home.copy(); self.goto("TAKEOFF")
        elif self.phase == "TAKEOFF":
            # 0.25 m/s ramp to hover_z, then hold
            dz = self.a.hover_z - self.climb_from[2]
            T = max(1.0, abs(dz) / 0.25)
            s = min(1.0, el / T)
            self.ref_p = self.climb_from + np.array([0, 0, dz * s])
            if s >= 1.0:
                err = float(np.linalg.norm(p - self.ref_p))
                if err < 0.06:
                    self.settle_since = self.settle_since or now
                    if now - self.settle_since > 4.0:
                        self.ev(f"hover settled: err={err*1000:.0f} mm tilt={tilt:.2f}deg")
                        self.call(self.cli_dir, SetBool.Request(data=True))
                        self.goto("DIRECT_WAIT")
                else:
                    self.settle_since = None
                if el > 60.0:
                    self.ev("!! takeoff did not settle in 60 s"); self.goto("LAND")
        elif self.phase == "DIRECT_WAIT":
            if "DIRECT" in self.mode.upper():
                self.direct_t0 = now; self.goto("DIRECT")
            elif el > 12.0:
                self.ev(f"!! DIRECT refused (mode='{self.mode}') -- retrying once")
                self.call(self.cli_dir, SetBool.Request(data=True)); self.tp = now
                if el > 30.0:
                    self.aborted = True; self.abort_reason = "DIRECT never engaged"
                    self.goto("LAND")
        elif self.phase == "DIRECT":
            if "DIRECT" not in self.mode.upper():
                self.aborted = True
                self.abort_reason = f"fell out of DIRECT after {el:.1f}s (watchdog)"
                self.ev(f"!! {self.abort_reason}"); self.goto("LAND")
            elif el > self.a.soak:
                self.ev(f"soak complete ({el:.1f} s)")
                self.call(self.cli_dir, SetBool.Request(data=False)); self.goto("REVERT")
        elif self.phase == "REVERT":
            if el > 6.0:
                self.land_from = self.ref_p.copy(); self.goto("LAND")
        elif self.phase == "LAND":
            base = getattr(self, "land_from", self.ref_p.copy())
            self.land_from = base
            dz = self.a.land_z - base[2]
            T = max(1.0, abs(dz) / 0.2)
            s = min(1.0, el / T)
            self.ref_p = base + np.array([0, 0, dz * s])
            if s >= 1.0 and el > T + 5.0:
                self.goto("DONE")
        elif self.phase == "DONE":
            self.done = True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--namespace", default="/uav_0")
    ap.add_argument("--rate", type=float, default=50.0)
    ap.add_argument("--hover-z", type=float, default=1.2)
    ap.add_argument("--land-z", type=float, default=0.40)
    ap.add_argument("--soak", type=float, default=75.0)
    ap.add_argument("--abort-tilt", type=float, default=35.0)
    ap.add_argument("--abort-pos", type=float, default=1.5)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()

    rclpy.init()
    n = Soak(a)
    t_end = time.time() + a.soak + 240.0
    while rclpy.ok() and not n.done and time.time() < t_end:
        rclpy.spin_once(n, timeout_sec=0.05)
    out = dict(args=vars(a), aborted=n.aborted, reason=n.abort_reason,
               events=n.events, rows=n.rows)
    with open(a.out, "w") as f:
        json.dump(out, f)
    print(f"\nwrote {a.out}  rows={len(n.rows)}  aborted={n.aborted} {n.abort_reason}",
          flush=True)
    n.destroy_node(); rclpy.shutdown()
    return 1 if n.aborted else 0


if __name__ == "__main__":
    sys.exit(main())
