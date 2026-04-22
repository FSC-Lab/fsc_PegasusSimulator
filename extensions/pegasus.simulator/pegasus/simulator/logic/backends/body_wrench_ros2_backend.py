"""
| File: body_wrench_ros2_backend.py
| Description: ROS 2 backend for IdealQuadrotor.
|
|   Subscribes
|   ----------
|   ~control/wrench  (geometry_msgs/WrenchStamped)
|       wrench.force.z        – lift force  F_z  [N]  in body FLU frame
|       wrench.torque.{x,y,z} – body torques [N·m]
|
|   Publishes
|   ---------
|   ~state/pose   (geometry_msgs/PoseStamped)   – position [m] + attitude (quat) in ENU
|   ~state/twist  (geometry_msgs/TwistStamped)  – linear vel [m/s] body-FLU +
|                                                  angular vel [rad/s] body-FLU
"""

import carb
from isaacsim.core.utils.extensions import enable_extension
enable_extension("isaacsim.ros2.bridge")

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped, WrenchStamped

from pegasus.simulator.logic.backends.backend import Backend


class BodyWrenchROS2Backend(Backend):
    """Backend that accepts a direct body-wrench command and publishes vehicle state.

    Args:
        vehicle_id (int): Integer appended to the ROS namespace to allow multiple
            vehicles in the same ROS graph.
        namespace (str): Base namespace.  Defaults to ``"drone"``.
    """

    def __init__(self, vehicle_id: int = 0, namespace: str = "drone"):
        super().__init__(config=None)

        self._id        = vehicle_id
        self._namespace = namespace + str(vehicle_id)

        # Command state: [F_z, tau_x, tau_y, tau_z]
        self._cmd = [0.0, 0.0, 0.0, 0.0]

        try:
            rclpy.init()
        except Exception:
            pass  # already initialised

        self._node = rclpy.create_node("ideal_quadrotor_" + str(vehicle_id))

        # Publisher: pose
        self._pose_pub = self._node.create_publisher(
            PoseStamped,
            self._namespace + "/state/pose",
            rclpy.qos.qos_profile_sensor_data,
        )

        # Publisher: twist (body frame)
        self._twist_pub = self._node.create_publisher(
            TwistStamped,
            self._namespace + "/state/twist",
            rclpy.qos.qos_profile_sensor_data,
        )

        # Subscriber: wrench command
        self._wrench_sub = self._node.create_subscription(
            WrenchStamped,
            self._namespace + "/control/wrench",
            self._wrench_callback,
            rclpy.qos.qos_profile_sensor_data,
        )

    # ------------------------------------------------------------------
    # ROS callback
    # ------------------------------------------------------------------

    def _wrench_callback(self, msg: WrenchStamped):
        self._cmd = [
            msg.wrench.force.z,
            msg.wrench.torque.x,
            msg.wrench.torque.y,
            msg.wrench.torque.z,
        ]

    # ------------------------------------------------------------------
    # Backend interface
    # ------------------------------------------------------------------

    def input_reference(self):
        """Return [F_z, tau_x, tau_y, tau_z] received from the ROS topic."""
        return self._cmd

    def update_state(self, state):
        """Publish pose and twist from the latest vehicle state."""
        stamp = self._node.get_clock().now().to_msg()

        # Pose (ENU inertial frame)
        pose_msg = PoseStamped()
        pose_msg.header.stamp    = stamp
        pose_msg.header.frame_id = "map"
        pose_msg.pose.position.x = float(state.position[0])
        pose_msg.pose.position.y = float(state.position[1])
        pose_msg.pose.position.z = float(state.position[2])
        pose_msg.pose.orientation.x = float(state.attitude[0])
        pose_msg.pose.orientation.y = float(state.attitude[1])
        pose_msg.pose.orientation.z = float(state.attitude[2])
        pose_msg.pose.orientation.w = float(state.attitude[3])
        self._pose_pub.publish(pose_msg)

        # Twist (body FLU frame)
        twist_msg = TwistStamped()
        twist_msg.header.stamp    = stamp
        twist_msg.header.frame_id = self._namespace + "/base_link"
        twist_msg.twist.linear.x  = float(state.linear_body_velocity[0])
        twist_msg.twist.linear.y  = float(state.linear_body_velocity[1])
        twist_msg.twist.linear.z  = float(state.linear_body_velocity[2])
        twist_msg.twist.angular.x = float(state.angular_velocity[0])
        twist_msg.twist.angular.y = float(state.angular_velocity[1])
        twist_msg.twist.angular.z = float(state.angular_velocity[2])
        self._twist_pub.publish(twist_msg)

    def update(self, dt: float):
        """Spin ROS once (non-blocking) to drain the subscriber queue."""
        rclpy.spin_once(self._node, timeout_sec=0)

    def update_sensor(self, sensor_type: str, data):
        pass

    def update_graphical_sensor(self, sensor_type: str, data):
        pass

    def start(self):
        self._cmd = [0.0, 0.0, 0.0, 0.0]

    def stop(self):
        self._cmd = [0.0, 0.0, 0.0, 0.0]

    def reset(self):
        self._cmd = [0.0, 0.0, 0.0, 0.0]
