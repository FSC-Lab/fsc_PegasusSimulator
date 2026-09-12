"""Launch the dual-tiltrotor direct_actuator offboard node.

Example:
    ros2 launch px4_vtol_direct_actuator vtol_direct_actuator.launch.py \
        px4_ns:=uav_0 motor_left:=0.6 motor_right:=0.6
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = {
        "px4_ns": "uav_0",
        "pub_rate_hz": "50.0",
        "auto_arm": "true",
        "motor_front_L": "0.0",
        "motor_front_R": "0.0",
        "motor_rear_L": "0.0",
        "motor_rear_R": "0.0",
        "tilt_left": "-1.0",
        "tilt_right": "-1.0",
        "aileron": "0.0",
        "elevator": "0.0",
        "rudder": "0.0",
    }

    declared = [DeclareLaunchArgument(k, default_value=v) for k, v in args.items()]
    params = {k: LaunchConfiguration(k) for k in args}

    return LaunchDescription(declared + [
        Node(
            package="px4_vtol_direct_actuator",
            executable="vtol_direct_actuator",
            name="vtol_direct_actuator",
            output="screen",
            parameters=[params],
        ),
    ])
