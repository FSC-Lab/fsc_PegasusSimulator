#!/usr/bin/env python3
"""Drive the live fsc_autopilot_ros2 DIRECT-actuation stack through a recorded reference
sequence and record everything needed to compare against the real flight.

Simulation counterpart of the 2026-08-07 direct-actuation flights. Same controller build,
same config file (params_single_drone_direct_actuation_t650.yaml is shared verbatim with
hardware), same reference waypoints. The controller regenerates its own attitude setpoints,
body-rate commands and motor commands closed-loop -- none of the real bag's inner-loop
streams are injected, they are what is being predicted.

Differences from stack_driver.py (the baseline-mode version):
  * An ENTER_DIRECT phase between takeoff and the sequence. The real flights took off under
    baseline/SAFETY control, switched to DIRECT while holding, then ran the steps, so the
    simulation does the same. t = 0 is the DIRECT engagement instant, matching
    make_direct_refs.py.
  * DIRECT engagement is confirmed by reading direct_actuation/mode back, not assumed from
    the service returning.
  * Records the inner-loop streams that only exist in DIRECT (motor commands, torque
    setpoints, rate-control debug).

Timing note: the DIRECT launcher disables Pegasus lockstep (PEGASUS_PX4_LOCKSTEP=0 --
lockstep deadlocks on DIRECT entry), so PX4 and Isaac free-run on independent clocks rather
than advancing together. Both clocks are recorded at every waypoint switch so the achieved
ratio is reported rather than assumed. The primary comparison signal is odometry, taken on
its own header stamp on both sides, which sidesteps the question entirely.
"""
import argparse

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from fsc_autopilot_ros2_msgs.msg import PositionControllerReference
from px4_msgs.msg import (ActuatorMotors, SensorCombined, VehicleAttitude,
                          VehicleStatus, VehicleTorqueSetpoint)

PX4_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE,
                     history=HistoryPolicy.KEEP_LAST, depth=10)


class DirectDriver(Node):
    def __init__(self, args):
        super().__init__("recorded_reference_driver_direct")
        self.args = args
        ns = args.namespace

        r = np.load(args.refs)
        self.ref_t = r["t_rel"].astype(np.float64)
        self.ref_p = r["position"].astype(np.float64)
        self.ref_yaw = r["yaw"].astype(np.float64)
        self.ref_yaw_unit = r["yaw_unit"].astype(int)
        self.n_ref = len(self.ref_t)
        self.get_logger().info(
            f"loaded {self.n_ref} waypoints spanning {self.ref_t[0]:.2f}.."
            f"{self.ref_t[-1]:.2f}s from {args.refs}")

        self.pub_ref = self.create_publisher(
            PositionControllerReference,
            f"{ns}/fsc_autopilot_ros2/position_controller/reference", 10)

        self.create_subscription(SensorCombined, f"{ns}/fmu/out/sensor_combined",
                                 self.on_sc, PX4_QOS)
        self.create_subscription(VehicleAttitude, f"{ns}/fmu/out/vehicle_attitude",
                                 self.on_att, PX4_QOS)
        self.create_subscription(ActuatorMotors, f"{ns}/fmu/in/actuator_motors",
                                 self.on_mot, PX4_QOS)
        self.create_subscription(VehicleTorqueSetpoint,
                                 f"{ns}/fmu/in/vehicle_torque_setpoint",
                                 self.on_trq, PX4_QOS)
        for t in (f"{ns}/fmu/out/vehicle_status", f"{ns}/fmu/out/vehicle_status_v1"):
            self.create_subscription(VehicleStatus, t, self.on_status, PX4_QOS)
        self.create_subscription(Odometry, f"{ns}/state_estimator/local_position/odom",
                                 self.on_odom, 10)
        self.create_subscription(String,
                                 f"{ns}/fsc_autopilot_ros2/direct_actuation/mode",
                                 self.on_mode, 10)

        self.cli_arm = self.create_client(Trigger, f"{ns}/rc/arm")
        self.cli_offboard = self.create_client(Trigger, f"{ns}/rc/offboard")
        self.cli_disarm = self.create_client(Trigger, f"{ns}/rc/disarm")
        self.cli_direct = self.create_client(
            SetBool, f"{ns}/fsc_autopilot_ros2/direct_actuation/set_direct_mode")

        self.phase = "WAIT_STACK"
        self.t_phase0 = self.now()
        self.seq_t0 = None
        self.idx = -1
        self.status = None
        self.mode = None
        self.have_odom = False
        self.pos = np.zeros(3)
        self.vel = np.zeros(3)
        self.settle_since = None
        self.last_req = 0.0
        self.done = False
        self.px4_us_latest = 0

        self.rec = {k: [] for k in
                    ("sc_t", "sc_g", "sc_a", "att_t", "att_q", "mot", "trq",
                     "odom", "reflog", "status", "modelog")}

        self.timer = self.create_timer(1.0 / args.ref_rate, self.tick)

    # ---------------- clocks ----------------
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def stamp_us(self):
        return self.get_clock().now().nanoseconds // 1000

    def sim_now(self):
        """PX4's own clock. Drives the waypoint schedule, as in the baseline driver."""
        return self.px4_us_latest * 1e-6

    def rel(self):
        return None if self.seq_t0 is None else self.sim_now() - self.seq_t0

    def recording(self):
        return self.phase in ("TAKEOFF", "ENTER_DIRECT", "SEQUENCE")

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

    def on_mot(self, m):
        if self.recording():
            self.rec["mot"].append((self.stamp_us(), self.px4_us_latest,
                                    *[float(v) for v in m.control[:4]]))

    def on_trq(self, m):
        if self.recording():
            self.rec["trq"].append((self.stamp_us(), self.px4_us_latest,
                                    *[float(v) for v in m.xyz]))

    def on_status(self, m):
        old = self.status
        self.status = m
        if old is None or (old.arming_state, old.nav_state) != (m.arming_state, m.nav_state):
            self.get_logger().info(f"PX4 status: arming={m.arming_state} nav={m.nav_state}")
        if self.recording():
            self.rec["status"].append((m.timestamp, m.arming_state, m.nav_state))

    def on_mode(self, m):
        if m.data != self.mode:
            self.get_logger().info(f"direct_actuation/mode -> {m.data}")
            if self.recording():
                self.rec["modelog"].append((self.stamp_us(), self.px4_us_latest, m.data))
        self.mode = m.data

    def on_odom(self, m):
        p, v = m.pose.pose.position, m.twist.twist.linear
        q, w = m.pose.pose.orientation, m.twist.twist.angular
        self.pos = np.array([p.x, p.y, p.z])
        self.vel = np.array([v.x, v.y, v.z])
        self.have_odom = True
        if self.recording():
            hdr = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            self.rec["odom"].append((self.stamp_us(), self.px4_us_latest, hdr,
                                     p.x, p.y, p.z, v.x, v.y, v.z,
                                     q.w, q.x, q.y, q.z, w.x, w.y, w.z))

    @property
    def armed(self):
        return self.status is not None and self.status.arming_state == 2

    @property
    def offboard(self):
        return self.status is not None and self.status.nav_state == 14

    # ---------------- reference ----------------
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
            self.get_logger().info(f"stack up (ref subs={subs}, odom ok) -> TAKEOFF")
            self.phase = "TAKEOFF"
            self.t_phase0 = self.sim_now()
            self.settle_since = None
        elif self.now() - self.t_phase0 > 120:
            self.get_logger().error(
                f"stack did not come up (ref subs={subs}, odom={self.have_odom}, "
                f"status={self.status is not None})")
            self.finish()

    def do_takeoff(self):
        """Fly to waypoint 0 under the stack's default (baseline/SAFETY) control, exactly
        as the real flights did before the operator switched to DIRECT."""
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
        if err < self.args.settle_tol and np.linalg.norm(self.vel) < 0.15:
            if self.settle_since is None:
                self.settle_since = self.sim_now()
            elif self.sim_now() - self.settle_since >= self.args.settle_s:
                self.get_logger().info(
                    f"settled at {np.array2string(self.pos, precision=3)} "
                    f"(err {err:.3f} m) after {el:.1f}s -> ENTER_DIRECT")
                self.phase = "ENTER_DIRECT"
                self.t_phase0 = self.sim_now()
                self.last_req = 0.0
                return
        else:
            self.settle_since = None

        if el > self.args.takeoff_timeout:
            self.get_logger().warn(
                f"takeoff did not settle in {el:.0f}s (err {err:.2f} m) -> ENTER_DIRECT")
            self.phase = "ENTER_DIRECT"
            self.t_phase0 = self.sim_now()
            self.last_req = 0.0

    def do_enter_direct(self):
        """Request DIRECT and confirm from the mode topic before starting the clock."""
        self.publish_ref(0)
        el = self.sim_now() - self.t_phase0

        if self.mode == "DIRECT":
            self.get_logger().info(
                f"DIRECT engaged after {el:.1f}s -> SEQUENCE (t=0 at engagement)")
            self.phase = "SEQUENCE"
            self.seq_t0 = self.sim_now() - float(self.ref_t[0])
            self.idx = 0
            return

        if self.now() - self.last_req > 2.0:
            self.last_req = self.now()
            if self.cli_direct.service_is_ready():
                self.cli_direct.call_async(SetBool.Request(data=True))
                self.get_logger().info("called set_direct_mode(true)")
            else:
                self.get_logger().warn("set_direct_mode service not ready")

        if el > self.args.direct_timeout:
            self.get_logger().error(
                f"DIRECT not engaged within {el:.0f}s (mode={self.mode}) -- aborting")
            self.finish()

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

        if self.mode != "DIRECT":
            self.get_logger().error(
                f"left DIRECT mid-sequence at t={t:.1f}s (mode={self.mode}) -- stopping")
            self.finish()
            return

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
            mot=arr("mot", 6), trq=arr("trq", 5),
            odom=arr("odom", 16), reflog=arr("reflog", 8), status=arr("status", 3),
            modelog=np.array(rec["modelog"], dtype=object) if rec["modelog"]
            else np.zeros((0, 3)),
            seq_t0_us=np.array([int(round((self.seq_t0 or 0) * 1e6))], dtype=np.int64),
        )
        self.get_logger().info(
            f"wrote {self.args.out}  sc={len(rec['sc_t'])} att={len(rec['att_t'])} "
            f"mot={len(rec['mot'])} odom={len(rec['odom'])}")
        self.done = True


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--refs", required=True)
    p.add_argument("--out", required=True)
    p.add_argument("--namespace", default="/uav_0")
    p.add_argument("--ref-rate", type=float, default=20.0)
    p.add_argument("--settle-tol", type=float, default=0.12)
    p.add_argument("--settle-s", type=float, default=3.0)
    p.add_argument("--takeoff-timeout", type=float, default=90.0)
    p.add_argument("--direct-timeout", type=float, default=30.0)
    p.add_argument("--tail-s", type=float, default=8.0)
    args = p.parse_args()

    rclpy.init()
    node = DirectDriver(args)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().warn("interrupted -- saving what was recorded")
        if not node.done:
            node.finish()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
