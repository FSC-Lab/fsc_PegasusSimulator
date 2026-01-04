#!/usr/bin/env python
"""
| File: rotorcraft_utils.py
| Author: Longhao Qian (longhao.qian@mail.utoronto.ca)
| License: BSD-3-Clause
| Copyright (c) 2025, Longhao Qian. All rights reserved.
| Description: Utility functions for defining rotorcraft in Isaac Sim
"""

from scipy.spatial.transform import Rotation

from pegasus.simulator.params import ROBOTS
from pegasus.simulator.logic.backends.px4_mavlink_backend import PX4MavlinkBackend, PX4MavlinkBackendConfig
from pegasus.simulator.logic.backends.ros2_backend import ROS2Backend
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig


def spawn_rotorcraft_with_mavlink(
    px4_path,
    px4_default_airframe,
    vehicle_id=0,
    spawn_pos=(0.0, 0.0, 0.0),
    spawn_euler=(0.0, 0.0, 0.0),
    connection_ip="127.0.0.1",
    connection_baseport=4560,
    vehicle_type="Iris"
):
    """
    Spawn rotorcarft with mavlink and ROS2 backend. No PX4 SITL. The SITL is launched separately in another terminal

    Args:
        px4_path: the path to the pixhawk SITL folder. Usually ~/PX4-Autopilot
        px4_default_airframe: PX4 SITL default airframe. In config it is gazebo-iris
        vehicle_id: the id of the current drone.
        spawn_pos: drone initial world position (ENU)
    """
    # common orientation
    quat_xyzw = Rotation.from_euler("XYZ", spawn_euler, degrees=True).as_quat()

    # ----- PX4 backend -----
    mavlink_config = PX4MavlinkBackendConfig({
        "vehicle_id": vehicle_id,
        "connection_type": "tcpin",
        "connection_ip": connection_ip,
        "connection_baseport": connection_baseport,   # Pegasus listens on 4560+i
        "px4_autolaunch": False,
        "px4_dir": px4_path,
        "px4_vehicle_model": px4_default_airframe,
    })

    # ----- ROS 2 backend -----
    ros2_backend = ROS2Backend(
        vehicle_id=vehicle_id,
        config={
            "namespace": f"uav_",
            "pub_sensors": False,
            "pub_graphical_sensors": False,
            "pub_state": True,
            "pub_twist": True,
            "pub_accel": True,
            "pub_twist_inertial": True,
            "pub_tf": True,
            "sub_control": False,
        },
    )

    # ----- Multirotor config -----
    config_multirotor = MultirotorConfig()
    config_multirotor.backends = [PX4MavlinkBackend(mavlink_config), ros2_backend]

    drone_prim_path = f"/World/quadrotor_{vehicle_id}"

    # IMPORTANT: use the same positional ctor as the single-vehicle example
    Multirotor(
        drone_prim_path,           # stage path
        ROBOTS[vehicle_type],      # robot config
        vehicle_id,                # vehicle_id
        spawn_pos,                 # initial position
        quat_xyzw,                 # initial orientation
        config=config_multirotor,  # backends, etc.
    )

    return drone_prim_path