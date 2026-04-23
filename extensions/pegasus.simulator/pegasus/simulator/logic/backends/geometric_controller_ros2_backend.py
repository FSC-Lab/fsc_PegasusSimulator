"""
| File: geometric_controller_ros2_backend.py
| Description: ROS 2 backend for IdealQuadrotor interfacing with fsc_geometric_controller.
|
|   Subscribes
|   ----------
|   ~control/output/f  (std_msgs/Float64)           – collective thrust [N] along body z
|   ~control/output/M  (geometry_msgs/Vector3Stamped)– body-frame moments [N·m]
|
|   Publishes
|   ---------
|   ~state/pose           (geometry_msgs/PoseStamped)   – position [m] + attitude (quat) ENU/FLU
|   ~state/twist          (geometry_msgs/TwistStamped)  – body angular velocity [rad/s]
|   ~state/twist_inertial (geometry_msgs/TwistStamped)  – inertial linear velocity [m/s]
|   ~state/accel          (geometry_msgs/AccelStamped)  – inertial linear acceleration [m/s²]
|   ~state/jerk           (geometry_msgs/Vector3Stamped)– inertial linear jerk [m/s³]
"""

import carb
from isaacsim.core.utils.extensions import enable_extension
enable_extension("isaacsim.ros2.bridge")

import rclpy
from std_msgs.msg import Float64
from geometry_msgs.msg import (
    PoseStamped, TwistStamped, AccelStamped, Vector3Stamped,
)
from rclpy.qos import qos_profile_sensor_data

from pegasus.simulator.logic.backends.backend import Backend


class GeometricControllerROS2Backend(Backend):
    """Backend that feeds fsc_geometric_controller and receives its [f, M] output.

    Args:
        vehicle_id (int): Used to name the ROS node.
        namespace (str): Full ROS namespace (default ``"uav_0"``).
    """

    def __init__(self, vehicle_id: int = 0, namespace: str = "uav_0"):
        super().__init__(config=None)

        self._id        = vehicle_id
        self._namespace = namespace

        # Command state
        self._f = 0.0
        self._M = [0.0, 0.0, 0.0]

        try:
            rclpy.init()
        except Exception:
            pass  # already initialised

        self._node = rclpy.create_node("ideal_quadrotor_" + str(vehicle_id))
        ns = self._namespace

        # ---- State publishers ------------------------------------------------
        self._pose_pub = self._node.create_publisher(
            PoseStamped, ns + "/state/pose", qos_profile_sensor_data)

        self._twist_pub = self._node.create_publisher(
            TwistStamped, ns + "/state/twist", qos_profile_sensor_data)

        self._twist_inertial_pub = self._node.create_publisher(
            TwistStamped, ns + "/state/twist_inertial", qos_profile_sensor_data)

        self._accel_pub = self._node.create_publisher(
            AccelStamped, ns + "/state/accel", qos_profile_sensor_data)

        self._jerk_pub = self._node.create_publisher(
            Vector3Stamped, ns + "/state/jerk", qos_profile_sensor_data)

        # ---- Control subscribers ---------------------------------------------
        self._f_sub = self._node.create_subscription(
            Float64,
            ns + "/control/output/f",
            self._f_callback,
            qos_profile_sensor_data,
        )

        self._M_sub = self._node.create_subscription(
            Vector3Stamped,
            ns + "/control/output/M",
            self._M_callback,
            qos_profile_sensor_data,
        )

    # ------------------------------------------------------------------
    # ROS callbacks
    # ------------------------------------------------------------------

    def _f_callback(self, msg: Float64):
        self._f = msg.data

    def _M_callback(self, msg: Vector3Stamped):
        self._M = [msg.vector.x, msg.vector.y, msg.vector.z]

    # ------------------------------------------------------------------
    # Backend interface
    # ------------------------------------------------------------------

    def input_reference(self):
        """Return [f, M_x, M_y, M_z] — maps directly to [F_z, tau_x, tau_y, tau_z]."""
        return [self._f, self._M[0], self._M[1], self._M[2]]

    def update_state(self, state):
        """Publish all state topics expected by fsc_geometric_controller."""
        stamp = self._node.get_clock().now().to_msg()
        ns    = self._namespace

        # state/pose — ENU position + FLU attitude quaternion
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

        # state/twist — body-frame angular velocity
        twist_msg = TwistStamped()
        twist_msg.header.stamp    = stamp
        twist_msg.header.frame_id = ns + "/base_link"
        twist_msg.twist.angular.x = float(state.angular_velocity[0])
        twist_msg.twist.angular.y = float(state.angular_velocity[1])
        twist_msg.twist.angular.z = float(state.angular_velocity[2])
        self._twist_pub.publish(twist_msg)

        # state/twist_inertial — inertial linear velocity
        twist_i_msg = TwistStamped()
        twist_i_msg.header.stamp    = stamp
        twist_i_msg.header.frame_id = "map"
        twist_i_msg.twist.linear.x = float(state.linear_velocity[0])
        twist_i_msg.twist.linear.y = float(state.linear_velocity[1])
        twist_i_msg.twist.linear.z = float(state.linear_velocity[2])
        self._twist_inertial_pub.publish(twist_i_msg)

        # state/accel — inertial linear acceleration (finite-diff from PhysX)
        accel_msg = AccelStamped()
        accel_msg.header.stamp    = stamp
        accel_msg.header.frame_id = "map"
        accel_msg.accel.linear.x = float(state.linear_acceleration[0])
        accel_msg.accel.linear.y = float(state.linear_acceleration[1])
        accel_msg.accel.linear.z = float(state.linear_acceleration[2])
        self._accel_pub.publish(accel_msg)

        # state/jerk — inertial linear jerk (finite-diff from PhysX)
        jerk_msg = Vector3Stamped()
        jerk_msg.header.stamp    = stamp
        jerk_msg.header.frame_id = "map"
        jerk_msg.vector.x = float(state.linear_jerk[0])
        jerk_msg.vector.y = float(state.linear_jerk[1])
        jerk_msg.vector.z = float(state.linear_jerk[2])
        self._jerk_pub.publish(jerk_msg)

    def update(self, dt: float):
        """Spin ROS once (non-blocking) to drain the subscriber queues."""
        rclpy.spin_once(self._node, timeout_sec=0)

    def update_sensor(self, sensor_type: str, data):
        pass

    def update_graphical_sensor(self, sensor_type: str, data):
        pass

    def start(self):
        self._f = 0.0
        self._M = [0.0, 0.0, 0.0]

    def stop(self):
        self._f = 0.0
        self._M = [0.0, 0.0, 0.0]

    def reset(self):
        self._f = 0.0
        self._M = [0.0, 0.0, 0.0]
