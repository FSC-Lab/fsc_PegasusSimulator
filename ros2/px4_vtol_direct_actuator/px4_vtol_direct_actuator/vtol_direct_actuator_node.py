#!/usr/bin/env python3
"""
VTOL direct-actuator offboard node for PX4 SITL + Isaac Sim (Pegasus HIL).

Target airframe: DUAL INDEPENDENT TILTROTOR
  - 2 motors          : motor_left, motor_right
  - 2 tilt servos     : tilt_left, tilt_right        (independent -> yaw by differential tilt)
  - 3 control surfaces: aileron, elevator, rudder

Why this node exists
--------------------
PX4 `direct_actuator` offboard cannot be commanded over MAVLink — only over
uXRCE-DDS (ROS 2). It also needs BOTH actuator groups for a tiltrotor:
  * OffboardControlMode{direct_actuator: true} -> /<ns>/fmu/in/offboard_control_mode
  * ActuatorMotors{control[12]}                -> /<ns>/fmu/in/actuator_motors   (the rotors)
  * ActuatorServos{control[8]}                 -> /<ns>/fmu/in/actuator_servos   (tilts + surfaces)
The existing px4_direct_actuator_test node only published ActuatorMotors, so the
servos never moved. This node streams both.

Whatever PX4 then emits is forwarded to Isaac Sim by the Pegasus HIL bridge as
HIL_ACTUATOR_CONTROLS, where PX4VTOLMavlinkBackend maps it back onto the VTOL's
channels (see default_dual_tiltrotor_channel_map).

  >>> ORDER MATTERS <<<
  The motor/servo index order below MUST match (a) PX4's actuator output
  assignment for your airframe AND (b) the Pegasus channel_map. Verify the whole
  chain with PEGASUS_HIL_DEBUG=1 in the Isaac pane.

Normalisation
-------------
  motors : [0, 1]   (NaN = channel disarmed / stopped)
  servos : [-1, 1]  (NaN = channel disarmed)
For the tilt servos, our convention is -1 = hover (rotor up), +1 = cruise (rotor
forward); the Pegasus channel_map converts that to the VTOL's [0,1] tilt.
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
    ActuatorServos,
    VehicleCommand,
    VehicleStatus,
)


# ---- actuator channel order (index = position in the PX4 actuator group) ----
# FSC CAD quad tiltrotor: 4 motors (2 front tilting + 2 rear lift) + 2 tilt servos
# + 3 control surfaces (aileron, elevator, rudder).
# Motors -> ActuatorMotors.control[i]
MOTOR_ORDER = ["motor_front_L", "motor_front_R", "motor_rear_L", "motor_rear_R"]
# Servos -> ActuatorServos.control[j]
SERVO_ORDER = ["tilt_left", "tilt_right", "aileron", "elevator", "rudder"]


class VTOLDirectActuatorNode(Node):
    """Streams direct-actuator motor + servo setpoints and drives PX4 to armed OFFBOARD."""

    def __init__(self):
        super().__init__("vtol_direct_actuator")

        # ---------------- Parameters ----------------
        self.declare_parameter("px4_ns", "uav_0")
        self.declare_parameter("pub_rate_hz", 50.0)
        self.declare_parameter("auto_arm", True)
        self.declare_parameter("arm_after_setpoints", 25)

        # Per-channel setpoints (held constant; edit live with `ros2 param set`).
        self.declare_parameter("motor_front_L", 0.0)   # [0, 1]
        self.declare_parameter("motor_front_R", 0.0)   # [0, 1]
        self.declare_parameter("motor_rear_L", 0.0)    # [0, 1]
        self.declare_parameter("motor_rear_R", 0.0)    # [0, 1]
        self.declare_parameter("tilt_left", -1.0)      # [-1, 1]  (-1 hover, +1 cruise)
        self.declare_parameter("tilt_right", -1.0)     # [-1, 1]
        self.declare_parameter("aileron", 0.0)         # [-1, 1]
        self.declare_parameter("elevator", 0.0)        # [-1, 1]
        self.declare_parameter("rudder", 0.0)          # [-1, 1]

        ns = self.get_parameter("px4_ns").get_parameter_value().string_value
        self._rate = self.get_parameter("pub_rate_hz").get_parameter_value().double_value
        self._auto_arm = self.get_parameter("auto_arm").get_parameter_value().bool_value
        self._arm_after = self.get_parameter("arm_after_setpoints").get_parameter_value().integer_value

        prefix = f"/{ns}" if ns else ""

        # ---------------- QoS (PX4 uXRCE-DDS expectation) ----------------
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
        self._servos_pub = self.create_publisher(
            ActuatorServos, f"{prefix}/fmu/in/actuator_servos", qos)
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
            f"vtol_direct_actuator up | ns='{ns}' rate={self._rate:.0f}Hz "
            f"auto_arm={self._auto_arm}")
        self.get_logger().info(
            f"motors={MOTOR_ORDER}  servos={SERVO_ORDER}")
        self.get_logger().info(
            f"pub: {prefix}/fmu/in/{{offboard_control_mode,actuator_motors,actuator_servos,"
            f"vehicle_command}}  sub: {prefix}/fmu/out/vehicle_status")

    # ------------------------------------------------------------------ helpers
    def _now_us(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1000)

    def _p(self, name: str) -> float:
        return float(self.get_parameter(name).get_parameter_value().double_value)

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
                f"VehicleStatus: arming_state={msg.arming_state} (armed={self._armed}) "
                f"nav_state={msg.nav_state} (offboard={self._offboard})")

    def _on_timer(self):
        self._counter += 1

        # 1) Heartbeat: tell PX4 we are commanding actuators directly.
        self._publish_offboard_control_mode()

        # 2) The actual setpoints (motors + servos).
        self._publish_actuator_motors()
        self._publish_actuator_servos()

        # 3) After streaming a bit, switch to OFFBOARD *then* arm (see the
        #    px4_direct_actuator_test node / docs for why order matters). Retry
        #    ~1 Hz until both are confirmed.
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
        controls = [math.nan] * 12              # NaN => disarmed channel (motor off)
        for i, name in enumerate(MOTOR_ORDER):
            controls[i] = self._p(name)
        msg.control = controls
        msg.reversible_flags = 0
        now = self._now_us()
        msg.timestamp = now
        msg.timestamp_sample = now
        self._motors_pub.publish(msg)

    def _publish_actuator_servos(self):
        msg = ActuatorServos()
        controls = [math.nan] * 8               # NaN => disarmed channel
        for j, name in enumerate(SERVO_ORDER):
            controls[j] = self._p(name)
        msg.control = controls
        now = self._now_us()
        msg.timestamp = now
        msg.timestamp_sample = now
        self._servos_pub.publish(msg)

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
    node = VTOLDirectActuatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._disarm()             # best-effort: stop the rotors on Ctrl+C
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
