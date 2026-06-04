"""Launch the direct_actuator offboard test node.

Example:
    ros2 launch px4_direct_actuator_test direct_actuator_test.launch.py \
        px4_ns:=uav_0 motor_value:=0.5
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument("px4_ns", default_value="uav_0",
                              description="PX4 uXRCE-DDS client namespace (PX4_UXRCE_DDS_NS)."),
        DeclareLaunchArgument("pub_rate_hz", default_value="50.0"),
        DeclareLaunchArgument("num_motors", default_value="4"),
        DeclareLaunchArgument("motor_value", default_value="0.5",
                              description="Normalized motor command in [-1, 1] for each active motor."),
        DeclareLaunchArgument("auto_arm", default_value="true"),
        DeclareLaunchArgument("arm_after_setpoints", default_value="25"),
    ]

    node = Node(
        package="px4_direct_actuator_test",
        executable="direct_actuator_test",
        name="direct_actuator_test",
        output="screen",
        parameters=[{
            "px4_ns": LaunchConfiguration("px4_ns"),
            "pub_rate_hz": LaunchConfiguration("pub_rate_hz"),
            "num_motors": LaunchConfiguration("num_motors"),
            "motor_value": LaunchConfiguration("motor_value"),
            "auto_arm": LaunchConfiguration("auto_arm"),
            "arm_after_setpoints": LaunchConfiguration("arm_after_setpoints"),
        }],
    )

    return LaunchDescription(args + [node])
