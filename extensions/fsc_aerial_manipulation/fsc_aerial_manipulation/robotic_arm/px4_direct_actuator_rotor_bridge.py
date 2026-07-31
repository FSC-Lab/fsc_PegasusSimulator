#!/usr/bin/env python3
"""PX4 direct-actuator ROTOR BRIDGE for the aerial-manipulator GMO rig.

Companion to application/robotic_arm/px4_direct_actuator_aerial_manipulator_gmo.py.
Runs under SYSTEM python (3.10) with the px4_msgs overlay sourced — px4_msgs'
compiled typesupport must never be imported inside Isaac's own python.

Role: a thin, control-law-free transport + engagement shim.

    Isaac (in-process GMO law)                       this node                              PX4 SITL
    ~/px4_bridge/rotor_omega  ── Float64MultiArray ─▶  u_i = clip((omega_i - idle)/scale)  ─▶ fmu/in/actuator_motors
                                                       + OffboardControlMode(direct_actuator)  fmu/in/offboard_control_mode
    ~/px4_bridge/engaged      ◀───── Bool ──────────  armed && OFFBOARD (from vehicle_status)

State machine (same proven sequence as the bare-frame X650 offboard rigs):
prestream u=0 for prestream_seconds → request OFFBOARD (retry 1 s) → arm
(retry 1 s) → engaged. 'engaged' is re-published every tick so the Isaac side
can hold its control clock until motor authority actually exists.

The (idle, scale) parameters MUST equal the PX4MavlinkBackendConfig values in
the Isaac app (zero_position_armed=81.8374, input_scaling=735.7537 — the
MN4014+15x5 bench map); the round trip omega→u→omega is then exact.

Run (the launcher does this):
    source /opt/ros/humble/setup.bash
    source <px4_msgs overlay>/setup.bash      # e.g. ~/workspaces/isaacsim/install
    python3 px4_direct_actuator_rotor_bridge.py --ros-args -r __ns:=/uav_0
"""

from __future__ import annotations

import math

import rclpy
from px4_msgs.msg import (
    ActuatorMotors,
    OffboardControlMode,
    VehicleCommand,
    VehicleStatus,
)
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool, Float64MultiArray

NUM_ROTORS = 4


class Px4DirectActuatorRotorBridge(Node):
    """omega[4] (rad/s) → PX4 OFFBOARD direct-actuator stream + engagement."""

    def __init__(self) -> None:
        super().__init__("px4_direct_actuator_rotor_bridge")

        self.declare_parameter("rate_hz", 250.0)
        # MUST match the Isaac app's PX4MavlinkBackendConfig (bench map):
        #   omega = u * input_scaling + zero_position_armed
        self.declare_parameter("input_scaling", 735.7537)        # rad/s
        self.declare_parameter("zero_position_armed", 81.8374)   # rad/s
        self.declare_parameter("prestream_seconds", 1.0)
        self.declare_parameter("auto_arm", True)
        # If Isaac stops sending omega for longer than this, command u=0
        # (motors to armed idle) instead of flying blind on a stale command.
        self.declare_parameter("stale_timeout", 1.0)

        self._rate_hz = float(self.get_parameter("rate_hz").value)
        self._scaling = float(self.get_parameter("input_scaling").value)
        self._idle = float(self.get_parameter("zero_position_armed").value)
        self._prestream_s = float(self.get_parameter("prestream_seconds").value)
        self._auto_arm = bool(self.get_parameter("auto_arm").value)
        self._stale_timeout = float(self.get_parameter("stale_timeout").value)
        if self._rate_hz <= 2.0:
            raise ValueError("rate_hz must exceed PX4's 2 Hz OFFBOARD minimum")
        if self._scaling <= 0.0:
            raise ValueError("input_scaling must be positive")

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._mode_pub = self.create_publisher(
            OffboardControlMode, "fmu/in/offboard_control_mode", px4_qos
        )
        self._motors_pub = self.create_publisher(
            ActuatorMotors, "fmu/in/actuator_motors", px4_qos
        )
        self._command_pub = self.create_publisher(
            VehicleCommand, "fmu/in/vehicle_command", px4_qos
        )
        self._engaged_pub = self.create_publisher(Bool, "px4_bridge/engaged", 10)

        self.create_subscription(
            Float64MultiArray, "px4_bridge/rotor_omega", self._omega_callback, 10
        )
        # PX4 v1.16 publishes the versioned alias; subscribe to both names so
        # the bridge keeps working across px4_msgs/firmware pairings.
        self.create_subscription(
            VehicleStatus, "fmu/out/vehicle_status_v1", self._status_callback, px4_qos
        )
        self.create_subscription(
            VehicleStatus, "fmu/out/vehicle_status", self._status_callback, px4_qos
        )

        self._status: VehicleStatus | None = None
        self._omega: list[float] | None = None
        self._omega_time = -math.inf
        self._stale_warned = False
        self._state = "prestream"
        self._stream_ticks = 0
        self._last_command_time = -math.inf
        self._timer = self.create_timer(1.0 / self._rate_hz, self._tick)

        self.get_logger().info(
            f"rotor bridge ready under {self.get_namespace()}: rate={self._rate_hz:.0f} Hz, "
            f"u=(omega-{self._idle:.4f})/{self._scaling:.4f}, auto_arm={self._auto_arm}"
        )

    # ── inputs ──────────────────────────────────────────────────────────────

    def _omega_callback(self, msg: Float64MultiArray) -> None:
        data = list(msg.data)[:NUM_ROTORS]
        if len(data) < NUM_ROTORS:
            data += [0.0] * (NUM_ROTORS - len(data))
        self._omega = data
        self._omega_time = self._now_s()
        if self._stale_warned:
            self._stale_warned = False
            self.get_logger().info("rotor_omega stream recovered")

    def _status_callback(self, msg: VehicleStatus) -> None:
        previous = self._status
        self._status = msg
        if (
            previous is None
            or previous.nav_state != msg.nav_state
            or previous.arming_state != msg.arming_state
        ):
            self.get_logger().info(
                f"PX4 status: nav_state={msg.nav_state} "
                f"arming_state={msg.arming_state} failsafe={msg.failsafe}"
            )

    # ── helpers ─────────────────────────────────────────────────────────────

    def _now_us(self) -> int:
        return self.get_clock().now().nanoseconds // 1000

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _is_offboard(self) -> bool:
        return (
            self._status is not None
            and self._status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        )

    def _is_armed(self) -> bool:
        return (
            self._status is not None
            and self._status.arming_state == VehicleStatus.ARMING_STATE_ARMED
        )

    def _command_due(self, now: float) -> bool:
        return now - self._last_command_time >= 1.0

    def _publish_command(self, command: int, param1: float, param2: float = 0.0) -> None:
        msg = VehicleCommand()
        msg.timestamp = self._now_us()
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self._command_pub.publish(msg)
        self._last_command_time = self._now_s()

    def _controls(self, now: float) -> list[float]:
        """Latest omega → normalized u in [0,1]; zeros when absent or stale."""
        if self._omega is None:
            return [0.0] * NUM_ROTORS
        if now - self._omega_time > self._stale_timeout:
            if not self._stale_warned:
                self._stale_warned = True
                self.get_logger().warning(
                    f"rotor_omega stale (> {self._stale_timeout:.1f}s) — commanding u=0"
                )
            return [0.0] * NUM_ROTORS
        return [
            min(1.0, max(0.0, (w - self._idle) / self._scaling)) for w in self._omega
        ]

    # ── main tick ───────────────────────────────────────────────────────────

    def _tick(self) -> None:
        now = self._now_s()
        stamp = self._now_us()

        mode = OffboardControlMode()
        mode.timestamp = stamp
        mode.direct_actuator = True
        self._mode_pub.publish(mode)

        motors = ActuatorMotors()
        motors.timestamp = stamp
        motors.timestamp_sample = stamp
        motors.control = [float("nan")] * ActuatorMotors.NUM_CONTROLS
        for index, value in enumerate(self._controls(now)):
            motors.control[index] = float(value)
        self._motors_pub.publish(motors)

        # Engagement state machine (prestream → offboard → arm → engaged).
        if self._state == "prestream":
            self._stream_ticks += 1
            if (
                self._stream_ticks >= math.ceil(self._prestream_s * self._rate_hz)
                and self._status is not None
            ):
                self._state = "request_offboard"
                self.get_logger().info("prestream complete; requesting PX4 OFFBOARD")
        elif self._state == "request_offboard":
            if self._is_offboard():
                self._state = "request_arm" if self._auto_arm else "wait_arm"
                self.get_logger().info("PX4 confirmed OFFBOARD")
            elif self._command_due(now):
                self._publish_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        elif self._state in {"request_arm", "wait_arm"}:
            if self._is_armed():
                self._state = "engaged"
                self.get_logger().info(
                    "PX4 confirmed ARMED in OFFBOARD — bridge ENGAGED "
                    "(Isaac takeoff clock starts now)"
                )
            elif self._state == "request_arm" and self._command_due(now):
                self._publish_command(
                    VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0
                )
        elif self._state == "engaged":
            if not (self._is_armed() and self._is_offboard()):
                self._state = "request_offboard"
                self.get_logger().warning(
                    "PX4 dropped armed/OFFBOARD — disengaged, re-requesting"
                )

        engaged = Bool()
        engaged.data = (
            self._state == "engaged" and self._is_armed() and self._is_offboard()
        )
        self._engaged_pub.publish(engaged)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = Px4DirectActuatorRotorBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("bridge interrupted; PX4 stream stops now")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
