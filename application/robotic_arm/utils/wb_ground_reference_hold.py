#!/usr/bin/env python3
"""Seat the whole-body node's SAFETY reference on the vehicle's own pose.

WHY THIS EXISTS. `directEntryAllowed()` refuses DIRECT unless
`|state.position - outer_ref.position| <= wb_gate_pos_m` (0.15 m). In the
flying rigs that is satisfied by taking off to the commanded hover and
settling there. On the GROUND-TEST rig the props produce no thrust, so there
is no takeoff to settle from, and `outer_ref_` is still its default (0, 0, 0)
while the seated vehicle sits at z = 0.305 m -- 0.305 m of error against a
0.15 m gate, i.e. DIRECT is refused for a reason that has nothing to do with
what the test is trying to observe.

This node supplies the missing half of that handshake and nothing else: while
the whole-body node reports SAFETY it republishes a position reference AT THE
VEHICLE'S OWN MEASURED POSE, so the gate reads ~0 and the operator can engage
DIRECT whenever they are ready.

TWO RULES IT DELIBERATELY OBEYS
  * IT GOES SILENT IN DIRECT. A full-rate `position_controller/reference`
    stream drags the whole-body planner out of HOLD on every tick (the
    2026-08-23 re-plan oscillation); in DIRECT the planner owns the reference
    and this node must not be in the loop at all.
  * IN SAFETY IT TRACKS THE MEASURED POSE RATHER THAN LATCHING ONE.
    The first version latched the pose after N odometry samples, and that
    FAILED LIVE on the first run: the estimator publishes (0, 0, 0) before it
    has a fix, so the sample count was satisfied instantly and the hold was
    taken at the origin while the vehicle sat at z = 0.305 -- the gate then
    read 0.305 m against its own 0.15 m limit, i.e. exactly the refusal this
    node exists to prevent. Tracking has no startup ordering to get wrong, and
    on a seated vehicle there is no drift to chase: the reference IS where the
    vehicle is, so the gate reads ~0 by construction. It also makes the
    SAFETY-revert case automatic -- `switchMode(kSafety)` re-seeds `outer_ref_`
    to the current pose and this node agrees with it without being told.

It is a GROUND-RIG convenience and has no place in a flying stack: there it
would silently overwrite the operator's setpoint with "stay where you are".

Run it in the vehicle namespace, with an interpreter that has rclpy AND the
fsc_autopilot_ros2_msgs overlay sourced:

    python3 wb_ground_reference_hold.py --ros-args -r __ns:=/uav_0
"""

import argparse
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

from nav_msgs.msg import Odometry
from std_msgs.msg import String
from fsc_autopilot_ros2_msgs.msg import PositionControllerReference

# Matches the client's `static constexpr uint32_t QoS = 10` (reliable, keep 10).
REF_TOPIC = "fsc_autopilot_ros2/position_controller/reference"
ODOM_TOPIC = "state_estimator/local_position/odom"
MODE_TOPIC = "fsc_autopilot_ros2/whole_body_direct_actuation/mode"


def _yaw_of(q):
    """Body yaw from a geometry_msgs quaternion (ENU/FLU, same convention the
    client uses via QuaternionToEulerAngles)."""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class GroundReferenceHold(Node):

    def __init__(self, rate_hz, settle_n):
        super().__init__("wb_ground_reference_hold")
        self._rate_hz = rate_hz
        self._settle_n = settle_n

        self._odom = None          # (x, y, z, yaw) newest sample
        self._samples = 0
        self._direct = False
        self._published = 0
        self._announced = False

        self._pub = self.create_publisher(PositionControllerReference,
                                          REF_TOPIC, 10)
        self.create_subscription(Odometry, ODOM_TOPIC, self._on_odom, 10)
        # The mode topic is latched (transient local, depth 1) so a late start
        # still learns the current mode instead of assuming SAFETY.
        self.create_subscription(
            String, MODE_TOPIC, self._on_mode,
            QoSProfile(depth=1,
                       history=QoSHistoryPolicy.KEEP_LAST,
                       reliability=QoSReliabilityPolicy.RELIABLE,
                       durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(1.0 / rate_hz, self._tick)

        self.get_logger().info(
            f"ground reference hold: publishing '{REF_TOPIC}' at {rate_hz:g} Hz "
            f"while SAFETY, silent in DIRECT. Waiting for '{ODOM_TOPIC}'.")

    # -- callbacks ----------------------------------------------------------

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        self._odom = (p.x, p.y, p.z, _yaw_of(msg.pose.pose.orientation))
        self._samples += 1

    def _on_mode(self, msg):
        direct = msg.data.strip().upper() == "DIRECT"
        if direct == self._direct:
            return
        self._direct = direct
        if direct:
            self.get_logger().info(
                "DIRECT engaged -- going silent; the whole-body planner owns the "
                "reference from here.")
        else:
            # SAFETY revert: the client has just re-seeded outer_ref_ to the
            # current pose, and tracking agrees with that automatically.
            self.get_logger().info(
                "back in SAFETY -- tracking the measured pose again.")

    # -- the one job --------------------------------------------------------

    def _tick(self):
        if self._direct or self._odom is None:
            return
        # Wait for a few samples so a half-initialised first message is not
        # what reaches the controller -- but do NOT latch on one: see the
        # module docstring for the (0, 0, 0) failure that caused.
        if self._samples < self._settle_n:
            return

        x, y, z, yaw = self._odom
        msg = PositionControllerReference()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.position.x, msg.position.y, msg.position.z = x, y, z
        msg.yaw = yaw
        msg.yaw_unit = PositionControllerReference.RADIANS
        self._pub.publish(msg)

        if not self._announced:
            self._announced = True
            self.get_logger().info(
                f"tracking the measured pose: ({x:.3f}, {y:.3f}, {z:.3f}) m, "
                f"yaw {math.degrees(yaw):.1f} deg -- the DIRECT position gate "
                f"now reads ~0.")

        self._published += 1
        if self._published % int(10 * self._rate_hz) == 0:
            self.get_logger().info(
                f"tracking ({x:.3f}, {y:.3f}, {z:.3f}) m, yaw "
                f"{math.degrees(yaw):.1f} deg -- gate error ~0 (limit 0.15 m).")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--rate", type=float, default=5.0,
                    help="publish rate in SAFETY [Hz] (default 5)")
    ap.add_argument("--settle-samples", type=int, default=10,
                    help="odometry samples to wait for before capturing the "
                         "hold (default 10)")
    args, ros_args = ap.parse_known_args()

    if args.rate <= 0.0:
        print("--rate must be positive", file=sys.stderr)
        return 2

    rclpy.init(args=[sys.argv[0]] + ros_args)
    node = GroundReferenceHold(args.rate, max(1, args.settle_samples))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
