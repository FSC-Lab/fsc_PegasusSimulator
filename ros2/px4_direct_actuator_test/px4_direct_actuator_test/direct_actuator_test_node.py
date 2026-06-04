#!/usr/bin/env python3
"""
Direct-actuator offboard test node for PX4 SITL + Isaac Sim (Pegasus HIL).

Architecture this is built for
------------------------------
  PX4 SITL (none_iris)  <-- HIL_SENSOR / HIL_ACTUATOR_CONTROLS (TCP 4560) -->  Isaac Sim / Pegasus
  PX4 SITL              <-- uXRCE-DDS (this node)                          -->  /uav_0/fmu/...

PX4 cannot be put into `direct_actuator` offboard over MAVLink, so the only way
to exercise it is over uXRCE-DDS by publishing:
  * OffboardControlMode{direct_actuator: true}   -> /<ns>/fmu/in/offboard_control_mode
  * ActuatorMotors{control: [...]}               -> /<ns>/fmu/in/actuator_motors

This node streams both at a fixed rate, then requests OFFBOARD + ARM, and
reports the resulting arming/nav state from /<ns>/fmu/out/vehicle_status so you
can see exactly whether PX4 accepts the mode. Whatever PX4 then computes is
forwarded to Isaac Sim by the existing Pegasus HIL bridge unchanged.

This is a standalone test/diagnostic node. It does not depend on, import, or
modify any other node in the workspace.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
)

from px4_msgs.msg import (
    OffboardControlMode,
    ActuatorMotors,
    VehicleCommand,
    VehicleStatus,
)


class DirectActuatorTestNode(Node):
    """Streams direct-actuator setpoints and drives PX4 into armed OFFBOARD."""

    def __init__(self):
        super().__init__("direct_actuator_test")

        # ---------------- Parameters ----------------
        self.declare_parameter("px4_ns", "uav_0")
        self.declare_parameter("pub_rate_hz", 50.0)
        self.declare_parameter("num_motors", 4)
        self.declare_parameter("motor_value", 0.5)   # range [-1, 1]; 1 = max thrust
        self.declare_parameter("auto_arm", True)
        self.declare_parameter("arm_after_setpoints", 25)  # stream before requesting OFFBOARD+ARM

        ns = self.get_parameter("px4_ns").get_parameter_value().string_value
        self._rate = self.get_parameter("pub_rate_hz").get_parameter_value().double_value
        self._num_motors = self.get_parameter("num_motors").get_parameter_value().integer_value
        self._motor_value = self.get_parameter("motor_value").get_parameter_value().double_value
        self._auto_arm = self.get_parameter("auto_arm").get_parameter_value().bool_value
        self._arm_after = self.get_parameter("arm_after_setpoints").get_parameter_value().integer_value

        prefix = f"/{ns}" if ns else ""

        # ---------------- QoS (matches PX4 uXRCE-DDS expectations) ----------------
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ---------------- Publishers ----------------
        self._ocm_pub = self.create_publisher(
            OffboardControlMode, f"{prefix}/fmu/in/offboard_control_mode", qos)
        self._motors_pub = self.create_publisher(
            ActuatorMotors, f"{prefix}/fmu/in/actuator_motors", qos)
        self._cmd_pub = self.create_publisher(
            VehicleCommand, f"{prefix}/fmu/in/vehicle_command", qos)

        # ---------------- Subscriber ----------------
        self._status_sub = self.create_subscription(
            VehicleStatus, f"{prefix}/fmu/out/vehicle_status",
            self._on_vehicle_status, qos)

        # ---------------- State ----------------
        self._counter = 0
        self._last_cmd_counter = 0
        self._arming_state = None
        self._nav_state = None

        self._timer = self.create_timer(1.0 / self._rate, self._on_timer)

        self.get_logger().info(
            f"direct_actuator_test up | ns='{ns}' rate={self._rate:.0f}Hz "
            f"motors={self._num_motors}@{self._motor_value} auto_arm={self._auto_arm}")
        self.get_logger().info(
            f"pub: {prefix}/fmu/in/{{offboard_control_mode,actuator_motors,vehicle_command}}  "
            f"sub: {prefix}/fmu/out/vehicle_status")

    # ------------------------------------------------------------------ helpers
    def _now_us(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1000)

    @property
    def _armed(self) -> bool:
        return self._arming_state == VehicleStatus.ARMING_STATE_ARMED

    @property
    def _offboard(self) -> bool:
        return self._nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD

    # ------------------------------------------------------------------ callbacks
    def _on_vehicle_status(self, msg: VehicleStatus):
        changed = (msg.arming_state != self._arming_state) or (msg.nav_state != self._nav_state)
        self._arming_state = msg.arming_state
        self._nav_state = msg.nav_state
        if changed:
            self.get_logger().info(
                f"VehicleStatus: arming_state={msg.arming_state} "
                f"(armed={self._armed}) nav_state={msg.nav_state} "
                f"(offboard={self._offboard})")

    def _on_timer(self):
        self._counter += 1

        # 1) Heartbeat: tell PX4 we are driving the actuators directly.
        self._publish_offboard_control_mode()

        # 2) The actual direct-actuator setpoint.
        self._publish_actuator_motors()

        # 3) Once we have streamed long enough, switch to OFFBOARD *first* and
        #    only arm after PX4 confirms nav_state == OFFBOARD. Arming while
        #    still in POSCTL makes the position flight task fail (there is no
        #    valid local position in this HIL setup), which drops the vehicle
        #    into failsafe and then blocks the offboard switch entirely.
        #    Retry ~1 Hz until both are true (commands can be dropped).
        if self._auto_arm and self._counter >= self._arm_after:
            if not (self._armed and self._offboard):
                if (self._counter - self._last_cmd_counter) >= int(self._rate):
                    self._last_cmd_counter = self._counter
                    if not self._offboard:
                        self._engage_offboard_mode()
                    elif not self._armed:
                        self._arm()

    # ------------------------------------------------------------------ publishers
    def _publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = False
        msg.direct_actuator = True
        msg.timestamp = self._now_us()
        self._ocm_pub.publish(msg)

    def _publish_actuator_motors(self):
        msg = ActuatorMotors()
        # control is float32[12]; NaN => channel disarmed (motor stopped).
        controls = [math.nan] * 12
        n = max(0, min(self._num_motors, 12))
        for i in range(n):
            controls[i] = float(self._motor_value)
        msg.control = controls
        msg.reversible_flags = 0
        now = self._now_us()
        msg.timestamp = now
        msg.timestamp_sample = now
        self._motors_pub.publish(msg)

    def _publish_vehicle_command(self, command: int, param1: float = 0.0, param2: float = 0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self._now_us()
        self._cmd_pub.publish(msg)

    def _engage_offboard_mode(self):
        # base_mode custom enabled (1), PX4 custom main mode OFFBOARD (6)
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("-> requested OFFBOARD mode")

    def _arm(self):
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info("-> requested ARM")

    def _disarm(self):
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info("-> requested DISARM")


def main(args=None):
    rclpy.init(args=args)
    node = DirectActuatorTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Best-effort disarm so the rotors stop when the test is interrupted.
        try:
            node._disarm()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
