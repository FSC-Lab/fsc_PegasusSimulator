#!/usr/bin/env python3
"""
Drive the live fsc_autopilot_ros2 baseline stack through a recorded reference sequence and
record everything needed to compare against the real flight.

This is the simulation counterpart of the hardware flight: the SAME controller build, the
SAME estimator (EKF2-fused) and the SAME reference waypoints, with fsc_autopilot_ros2
regenerating its own attitude setpoints closed-loop. Nothing from the real bag's attitude
setpoint stream is injected -- those are the thing being predicted.

Sequence:
  WAIT_STACK  wait for the controller to subscribe to the reference topic and for the
              estimator to publish feedback.
  TAKEOFF     stream the sequence's FIRST waypoint, request OFFBOARD, arm, and let the
              controller fly there. The real bags start mid-flight already hovering, so
              the simulation must reach the same initial condition before t=0.
  SEQUENCE    t=0. Switch the streamed waypoint at each recorded relative time.
  DONE        land and disarm.

Timing: the waypoint schedule runs on PX4's own clock (see sim_now), NOT on wall clock.
Under Pegasus lockstep PX4 advances with Isaac, and Isaac runs below real time -- 0.770x
measured on this machine for this scene. A wall-clock schedule therefore compresses the
sequence by that factor in simulation time, which silently shortens every hold and
corrupts the rise/settling comparison. timesync_status and both clocks are recorded at
every waypoint switch so the ratio can be reported rather than assumed.
"""
import argparse
import math

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger

from fsc_autopilot_ros2_msgs.msg import PositionControllerReference
from px4_msgs.msg import (
    SensorCombined,
    TimesyncStatus,
    VehicleAttitude,
    VehicleAttitudeSetpoint,
    VehicleStatus,
)

PX4_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE,
                     history=HistoryPolicy.KEEP_LAST, depth=10)


class StackDriver(Node):
    def __init__(self, args):
        super().__init__("recorded_reference_driver")
        self.args = args
        ns = args.namespace

        r = np.load(args.refs)
        self.ref_t = r["t_rel"].astype(np.float64)
        self.ref_p = r["position"].astype(np.float64)
        self.ref_yaw = r["yaw"].astype(np.float64)
        self.ref_yaw_unit = r["yaw_unit"].astype(int)
        self.n_ref = len(self.ref_t)
        self.get_logger().info(
            f"loaded {self.n_ref} reference waypoints spanning "
            f"{self.ref_t[0]:.2f}..{self.ref_t[-1]:.2f}s from {args.refs}"
        )

        # ---- publishers / subscribers ----
        # Default QoS (KEEP_LAST 10, RELIABLE, VOLATILE) -- matches both the ground-station
        # GUI and fsc_autopilot_pos_streamer.py, and the QoS recorded in the real bags.
        self.pub_ref = self.create_publisher(
            PositionControllerReference,
            f"{ns}/fsc_autopilot_ros2/position_controller/reference", 10)

        self.create_subscription(SensorCombined, f"{ns}/fmu/out/sensor_combined",
                                 self.on_sc, PX4_QOS)
        self.create_subscription(VehicleAttitude, f"{ns}/fmu/out/vehicle_attitude",
                                 self.on_att, PX4_QOS)
        self.create_subscription(VehicleAttitudeSetpoint,
                                 f"{ns}/fmu/in/vehicle_attitude_setpoint",
                                 self.on_sp, PX4_QOS)
        self.create_subscription(VehicleAttitudeSetpoint,
                                 f"{ns}/fsc_autopilot_ros2/attitude_setpoint_debug",
                                 self.on_spdbg, PX4_QOS)
        self.create_subscription(TimesyncStatus, f"{ns}/fmu/out/timesync_status",
                                 self.on_ts, PX4_QOS)
        for t in (f"{ns}/fmu/out/vehicle_status", f"{ns}/fmu/out/vehicle_status_v1"):
            self.create_subscription(VehicleStatus, t, self.on_status, PX4_QOS)
        self.create_subscription(Odometry, f"{ns}/state_estimator/local_position/odom",
                                 self.on_odom, 10)

        self.cli_arm = self.create_client(Trigger, f"{ns}/rc/arm")
        self.cli_offboard = self.create_client(Trigger, f"{ns}/rc/offboard")
        self.cli_disarm = self.create_client(Trigger, f"{ns}/rc/disarm")

        # ---- state ----
        self.phase = "WAIT_STACK"
        self.t_phase0 = self.now()
        self.seq_t0 = None
        self.idx = -1
        self.status = None
        self.have_odom = False
        self.pos = np.zeros(3)
        self.vel = np.zeros(3)
        self.settle_since = None
        self.last_req = 0.0
        self.done = False
        self.px4_us_latest = 0

        # ---- recording ----
        self.rec = {k: [] for k in
                    ("sc_t", "sc_g", "sc_a", "att_t", "att_q", "sp_t", "sp_q", "sp_thr",
                     "spd_t", "spd_q", "spd_thr", "odom", "ts", "reflog", "status")}

        self.timer = self.create_timer(1.0 / args.ref_rate, self.tick)

    # ---------------- helpers ----------------
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def stamp_us(self):
        return self.get_clock().now().nanoseconds // 1000

    def sim_now(self):
        """Simulation-time clock, in seconds, taken from PX4's own hrt stamps.

        This MUST drive the waypoint schedule. Under Pegasus lockstep PX4 advances with
        Isaac, and Isaac does not keep up with wall clock -- measured at 0.770x real time
        on this machine for this scene. Scheduling the sequence on wall clock therefore
        compresses it in simulation time by that factor (a 90 s sequence of steps became
        69.3 s of simulated flight), shortening every hold by 23% and corrupting exactly
        the rise/settling comparison this study exists to make. Verified empirically: the
        received sensor_combined timestamps are raw PX4 time, quantised to the 4 ms
        physics step, and advance at 0.770x the driver's wall clock.
        """
        return self.px4_us_latest * 1e-6

    def rel(self):
        return None if self.seq_t0 is None else self.sim_now() - self.seq_t0

    def recording(self):
        return self.phase in ("TAKEOFF", "SEQUENCE")

    # ---------------- callbacks ----------------
    def on_sc(self, m):
        self.px4_us_latest = m.timestamp
        if self.recording():
            self.rec["sc_t"].append(m.timestamp)
            self.rec["sc_g"].append(np.asarray(m.gyro_rad, dtype=np.float64))
            self.rec["sc_a"].append(np.asarray(m.accelerometer_m_s2, dtype=np.float64))

    def on_att(self, m):
        if self.recording():
            self.rec["att_t"].append(m.timestamp)
            self.rec["att_q"].append(np.asarray(m.q, dtype=np.float64))

    def on_sp(self, m):
        if self.recording():
            self.rec["sp_t"].append(m.timestamp)
            self.rec["sp_q"].append(np.asarray(m.q_d, dtype=np.float64))
            self.rec["sp_thr"].append(np.asarray(m.thrust_body, dtype=np.float64))

    def on_spdbg(self, m):
        if self.recording():
            self.rec["spd_t"].append(m.timestamp)
            self.rec["spd_q"].append(np.asarray(m.q_d, dtype=np.float64))
            self.rec["spd_thr"].append(np.asarray(m.thrust_body, dtype=np.float64))

    def on_ts(self, m):
        if self.recording():
            self.rec["ts"].append((self.stamp_us(), m.timestamp, m.remote_timestamp,
                                   m.observed_offset, m.estimated_offset))

    def on_status(self, m):
        old = self.status
        self.status = m
        if old is None or (old.arming_state, old.nav_state) != (m.arming_state, m.nav_state):
            self.get_logger().info(f"PX4 status: arming={m.arming_state} nav={m.nav_state}")
        if self.recording():
            self.rec["status"].append((m.timestamp, m.arming_state, m.nav_state))

    def on_odom(self, m):
        p, v = m.pose.pose.position, m.twist.twist.linear
        q, w = m.pose.pose.orientation, m.twist.twist.angular
        self.pos = np.array([p.x, p.y, p.z])
        self.vel = np.array([v.x, v.y, v.z])
        self.have_odom = True
        if self.recording():
            # 15 columns: 0 wall_us, 1 px4_us, 2:5 pos, 5:8 vel, 8:12 quat(wxyz), 12:15
            # omega -- exactly the layout load_sim() slices. Orientation and body rates
            # were dropped from this recording at some point, which silently made every
            # attitude and yaw comparison impossible (load_sim raised IndexError on the
            # 8-column result). Yaw is the axis this campaign most needs, so they stay.
            self.rec["odom"].append((self.stamp_us(), self.px4_us_latest,
                                     p.x, p.y, p.z, v.x, v.y, v.z,
                                     q.w, q.x, q.y, q.z, w.x, w.y, w.z))

    @property
    def armed(self):
        return self.status is not None and self.status.arming_state == 2

    @property
    def offboard(self):
        return self.status is not None and self.status.nav_state == 14

    # ---------------- reference streaming ----------------
    def publish_ref(self, i):
        m = PositionControllerReference()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "ground"
        m.position.x, m.position.y, m.position.z = [float(v) for v in self.ref_p[i]]
        m.yaw = float(self.ref_yaw[i])
        m.yaw_unit = int(self.ref_yaw_unit[i])
        self.pub_ref.publish(m)

    def call(self, cli, name):
        if cli.service_is_ready():
            cli.call_async(Trigger.Request())
            self.get_logger().info(f"called {name}")
            return True
        self.get_logger().warn(f"service {name} not ready")
        return False

    # ---------------- state machine ----------------
    def tick(self):
        if self.done:
            return
        getattr(self, f"do_{self.phase.lower()}")()

    def do_wait_stack(self):
        subs = self.pub_ref.get_subscription_count()
        if subs > 0 and self.have_odom and self.status is not None:
            self.get_logger().info(
                f"stack up (reference subscribers={subs}, odom ok) -> TAKEOFF")
            self.phase = "TAKEOFF"
            self.t_phase0 = self.sim_now()
            self.settle_since = None
        elif self.now() - self.t_phase0 > 120:
            self.get_logger().error(
                f"stack did not come up (ref subs={subs}, odom={self.have_odom}, "
                f"status={self.status is not None})")
            self.finish()

    def do_takeoff(self):
        # Stream waypoint 0 -- the state the real vehicle was already holding at t=0.
        self.publish_ref(0)
        el = self.sim_now() - self.t_phase0

        if not (self.armed and self.offboard) and self.now() - self.last_req > 2.0:
            self.last_req = self.now()
            if not self.offboard:
                self.call(self.cli_offboard, "rc/offboard")
            elif not self.armed:
                self.call(self.cli_arm, "rc/arm")
            return

        err = np.linalg.norm(self.pos - self.ref_p[0])
        settled = err < self.args.settle_tol and np.linalg.norm(self.vel) < 0.15
        if settled:
            if self.settle_since is None:
                self.settle_since = self.sim_now()
            elif self.sim_now() - self.settle_since >= self.args.settle_s:
                self.get_logger().info(
                    f"settled at {np.array2string(self.pos, precision=3)} "
                    f"(target {self.ref_p[0]}, err {err:.3f} m) after {el:.1f}s -> SEQUENCE")
                self.phase = "SEQUENCE"
                self.seq_t0 = self.sim_now() - float(self.ref_t[0])
                self.idx = 0
                return
        else:
            self.settle_since = None

        if el > self.args.takeoff_timeout:
            self.get_logger().warn(
                f"takeoff did not settle in {el:.0f}s (err {err:.2f} m) -> SEQUENCE anyway")
            self.phase = "SEQUENCE"
            self.seq_t0 = self.sim_now() - float(self.ref_t[0])
            self.idx = 0

    def do_sequence(self):
        t = self.rel()
        nxt = self.idx + 1
        while nxt < self.n_ref and self.ref_t[nxt] <= t:
            self.idx = nxt
            self.rec["reflog"].append(
                (self.stamp_us(), self.px4_us_latest, self.idx, t,
                 *self.ref_p[self.idx], self.ref_yaw[self.idx]))
            self.get_logger().info(
                f"t={t:7.2f}s  ref[{self.idx}] -> pos={self.ref_p[self.idx]} "
                f"yaw={self.ref_yaw[self.idx]:.1f}")
            nxt = self.idx + 1
        self.publish_ref(self.idx)

        if t > self.ref_t[-1] + self.args.tail_s:
            self.get_logger().info(f"sequence complete at t={t:.1f}s -> DONE")
            self.finish()

    def finish(self):
        self.phase = "DONE"
        self.call(self.cli_disarm, "rc/disarm")
        rec = self.rec
        arr = lambda k, w: (np.asarray(rec[k], dtype=np.float64).reshape(-1, w)
                            if rec[k] else np.zeros((0, w)))
        np.savez_compressed(
            self.args.out,
            sc_ts=np.asarray(rec["sc_t"], dtype=np.int64),
            sc_gyro=arr("sc_g", 3), sc_accel=arr("sc_a", 3),
            att_ts=np.asarray(rec["att_t"], dtype=np.int64), att_q=arr("att_q", 4),
            sp_ts=np.asarray(rec["sp_t"], dtype=np.int64),
            sp_q_d=arr("sp_q", 4), sp_thrust=arr("sp_thr", 3),
            spdbg_ts=np.asarray(rec["spd_t"], dtype=np.int64),
            spdbg_q_d=arr("spd_q", 4), spdbg_thrust=arr("spd_thr", 3),
            odom=arr("odom", 15), timesync=arr("ts", 5), reflog=arr("reflog", 8),
            status=arr("status", 3),
            seq_t0_us=np.array([self._seq_t0_px4()], dtype=np.int64),
        )
        self.get_logger().info(
            f"wrote {self.args.out}  sc={len(rec['sc_t'])} att={len(rec['att_t'])} "
            f"sp={len(rec['sp_t'])} odom={len(rec['odom'])}")
        self.done = True

    def _seq_t0_px4(self):
        """PX4-timestamp value corresponding to sequence t=0.

        The recorded PX4 topics are stamped on PX4's own published timebase and the
        sequence is scheduled on that same clock, so seq_t0 is already the answer.
        """
        if self.seq_t0 is not None:
            return int(round(self.seq_t0 * 1e6))
        return int(self.px4_us_latest)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--refs", required=True, help="ref_A.npz / ref_B.npz")
    p.add_argument("--out", required=True)
    p.add_argument("--namespace", default="/uav_0")
    p.add_argument("--ref-rate", type=float, default=30.0)
    p.add_argument("--settle-tol", type=float, default=0.12)
    p.add_argument("--settle-s", type=float, default=4.0)
    p.add_argument("--takeoff-timeout", type=float, default=90.0)
    p.add_argument("--tail-s", type=float, default=8.0)
    args = p.parse_args()

    rclpy.init()
    node = StackDriver(args)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        if not node.done:
            node.finish()
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
