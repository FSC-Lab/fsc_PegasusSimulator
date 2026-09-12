"""
| File: rotation_utils.py
| Author: Longhao Qian (longhao.qian@mail.utoronto.ca)
| License: BSD-3-Clause
| Copyright (c) 2025, Longhao Qian. All rights reserved.
"""

from .lighting_utils import add_dome_lighting
from .rotation_utils import quat_from_x_deg, quat_from_y_deg, quat_from_z_deg

__all__ = [
    "add_dome_lighting",
    "quat_from_x_deg",
    "quat_from_y_deg",
    "quat_from_z_deg",
]

# ROS2RigidBodyBackend hard-imports rclpy (Isaac's ROS2 bridge). Import it
# gracefully so the pure utilities above stay importable when rclpy is not
# available — e.g. PX4 MAVLink HIL scenes that don't use ROS2 (same pattern as
# pegasus.simulator.logic.backends.__init__). Without this guard, importing even
# add_dome_lighting would crash any Isaac scene where the ROS2 bridge isn't loaded.
try:
    from .rigid_body_backend_utils import ROS2RigidBodyBackend
    __all__.append("ROS2RigidBodyBackend")
except Exception as _exc:  # rclpy / ROS2 bridge not available
    try:
        import carb
        carb.log_warn(
            f"fsc_aerial_manipulation.utils: ROS2RigidBodyBackend unavailable ({_exc})"
        )
    except Exception:
        pass