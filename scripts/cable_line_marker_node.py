#!/usr/bin/env python3
"""Publishes a visualization_msgs/Marker LINE_STRIP between the drone and payload poses, for
scripts/view_drone_3d.sh's RViz view. Pure visualization helper - no Isaac Sim dependency, just
subscribes to the two PoseStamped topics Pegasus's ROS2 backend and ROS2RigidBodyBackend already
publish, and draws a line between the latest of each. RViz has no built-in way to connect two
independently-published poses, hence this small node.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker


class CableLineMarkerNode(Node):
    def __init__(self):
        super().__init__("cable_line_marker_node")

        self.declare_parameter("drone_pose_topic", "/uav_0/state/pose")
        self.declare_parameter("payload_pose_topic", "/payload/state/pose")
        self.declare_parameter("marker_topic", "/cable_line_marker")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("publish_rate_hz", 20.0)

        drone_topic = self.get_parameter("drone_pose_topic").value
        payload_topic = self.get_parameter("payload_pose_topic").value
        marker_topic = self.get_parameter("marker_topic").value
        self._frame_id = self.get_parameter("frame_id").value
        rate_hz = self.get_parameter("publish_rate_hz").value

        self._drone_point = None
        self._payload_point = None

        self.create_subscription(PoseStamped, drone_topic, self._on_drone_pose, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, payload_topic, self._on_payload_pose, qos_profile_sensor_data)
        self._marker_pub = self.create_publisher(Marker, marker_topic, 10)

        self.create_timer(1.0 / rate_hz, self._publish_marker)

        self.get_logger().info(
            f"Cable line marker: {drone_topic} <-> {payload_topic} -> {marker_topic} ({self._frame_id})"
        )

    def _on_drone_pose(self, msg: PoseStamped):
        self._drone_point = msg.pose.position

    def _on_payload_pose(self, msg: PoseStamped):
        self._payload_point = msg.pose.position

    def _publish_marker(self):
        if self._drone_point is None or self._payload_point is None:
            return

        marker = Marker()
        marker.header.frame_id = self._frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "cable"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.02  # line width (m)
        marker.color.r = 1.0
        marker.color.g = 0.85
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.points = [
            Point(x=self._drone_point.x, y=self._drone_point.y, z=self._drone_point.z),
            Point(x=self._payload_point.x, y=self._payload_point.y, z=self._payload_point.z),
        ]
        self._marker_pub.publish(marker)


def main():
    rclpy.init()
    node = CableLineMarkerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
